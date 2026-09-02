#ifndef CONTROLWORKER_H
#define CONTROLWORKER_H

/*
 * 文件总览：
 * - ControlWorker 是实时力控 worker，周期性读取电机/力传感器反馈并计算力控力矩命令。
 * - Config 保存运行参数和每轴限制，Snapshot 给 UI、安全监控、诊断曲线提供一致的最新反馈快照。
 * - 本类运行在独立线程内，所有跨线程参数读写都通过 mutex 保护。
 */

#include <QObject>
#include <QMutex>
#include <QTimer>
#include <QVector>
#include <atomic>
#include <vector>

#include "forcecontroller.h"
#include "forcepid0525.h"
#include "hardwareinterface.h"
#include "onlinevelocitycontrol.h"
#include "endpointremotecontrol.h"

class ControlWorker : public QObject
{
    Q_OBJECT
public:
    enum class ForcePidOutputMode {
        Pid0624 = 1,
        Pid0525 = 2
    };

    enum class ForcePid0525DynamicTrackMode {
        AC = 0,
        FeedForwardOnly = 1
    };

    struct ForceFeedForwardOnlyDirectionalProfile {
        bool enabled = false;
        ForcePid0525DynamicTrackMode dynamicTrackMode =
                ForcePid0525DynamicTrackMode::FeedForwardOnly;
        bool useBangBangPretension = true;
        double frictionCoulombNm = 0.0;
        double frictionViscousNmPerRadPerSec = 0.0;
        double frictionVelocityDeadbandRadPerSec = 0.05;
        bool staticFrictionEnabled = false;
        double staticFrictionScale = 0.5;
        double staticFrictionForceRateDeadbandNPerSec = 1.0;
        double staticFrictionVelocityFadeStartRadPerSec = 0.02;
        double staticFrictionVelocityFadeEndRadPerSec = 0.20;
        bool staticFrictionMechanicalDirectionEnabled = false;
        bool staticFrictionExitBlendEnabled = false;
        double staticFrictionExitBlendTimeConstantSec = 0.0;
        double inertiaScale = 0.0;
        double trackTorqueSlewRateNmPerSec = 30.0;
        double trackBlendTimeSec = 0.15;
    };

    struct DiagnosticRawSample {
        qint64 wallClockMs = 0;
        qint64 intervalUs = 0;
        qint64 wallClockUs = 0;
        bool fromTrace = false;
        bool expandedTraceFrame = false;
    };

    struct SensorValueSample {
        qint64 wallClockMs = 0;
        std::vector<double> values;
        qint64 wallClockUs = 0;
        bool fromTrace = false;
        bool expandedTraceFrame = false;
    };

    struct TimingDiagnostics {
        quint64 controlLoopTickCount = 0;
        quint64 controlLoopIntervalCount = 0;
        qint64 controlLoopIntervalSumUs = 0;
        qint64 latestControlLoopIntervalUs = 0;
        quint64 sensorFrameCount = 0;
        quint64 sensorFrameIntervalCount = 0;
        qint64 sensorFrameIntervalSumUs = 0;
        qint64 latestSensorFrameIntervalUs = 0;
    };

    struct AxisConfig {
        bool isMotorAxis = false;
        bool actualTorqueLimitApplies = true;
        bool forceControlEnabled = false;
        int sensorIndex = -1;
        double motorCof = 1.0;
        // 绳力/收紧控制量映射到电机力矩命令的方向，独立于 motorCof 的半径大小。
        double motorDirectionSign = 1.0;
        double forceMax = 0.0;
        double motorMin = 0.0;
        double motorMax = 0.0;
        double motorVelMax = 0.0;
    };

    struct Config {
        int axisCount = 0;
        int sensorCount = 0;
        double ctrlCycleMs = 10.0;
        double sensorSampleHz = 2000.0;
        int forceSensorTraceSamplePeriodUs = 500;
        double forceSensorLowPassCutoffHz = 0.0;
        bool systemRunning = false;
        bool commissioningModeActive = false;
        int commissioningAxisIndex = -1;
        bool useLeadshine = false;
        bool forceThreadEnabled = false;
        bool usePid = true;
        ForcePidOutputMode forcePidOutputMode = ForcePidOutputMode::Pid0525;
        bool pvtActiveOrPaused = false;
        bool motorPositionLimitRecoveryActive = false;
        bool actualTorqueRealtimeEnabled = false;
        bool actualTorqueLimitEnabled = true;
        bool forcePidTuningHighRateSampleEnabled = false;
        bool allCableForceDragModeEnabled = false;
        bool forceFeedForwardOnlyTestModeEnabled = false;
        double actualTorqueLimitNm = 60.0;
        double initForce = 0.0;
        double forcePidDeadbandRatio = 0.0;
        double forceFeedForwardScale = 1.0;
        double forceHighTensionFeedForwardStartN = 300.0;
        double forceHighTensionFeedForwardFullN = 800.0;
        double forceHighTensionFeedForwardAddScale = 0.03;
        double forceUnloadFeedForwardScale = 0.78;
        double forceUnloadHighTensionFeedForwardAddScale = 0.025;
        double forceUnloadFeedForwardRateThresholdNPerSec = 5.0;
        double forceUnloadFeedForwardBlendTimeSec = 0.15;
        double forceExpectedRateFeedForwardGainUpNmPerNps = 0.005;
        double forceExpectedRateFeedForwardGainDownNmPerNps = 0.025;
        double forceExpectedRateFeedForwardLimitUpNm = 5.0;
        double forceExpectedRateFeedForwardLimitDownNm = 15.0;
        double forceExpectedRateFeedForwardFadeTimeSec = 0.12;
        double forceExpectedRateFeedForwardDownErrorGateN = 8.0;
        double forceExpectedRateFeedForwardDownFastDropGateNPerSec = 120.0;
        double forceExpectedRateFeedForwardDownMinScale = 0.25;
        double forceRateControlDerivativeLimitNPerSec = 1000.0;
        double forceRateControlDerivativePlatformLimitNPerSec = 400.0;
        double forceRateErrorDeadbandNPerSec = 50.0;
        double forceRateBelowExpectedCatchUpGainNmPerNps = 0.0008;
        double forceRateBelowExpectedCatchUpLimitNm = 1.0;
        double forceRateBelowExpectedBrakeGainNmPerNps = 0.0010;
        double forceRateBelowExpectedBrakeLimitNm = 1.0;
        double forceRateAboveExpectedUnloadGainNmPerNps = 0.0008;
        double forceRateAboveExpectedUnloadLimitNm = 1.0;
        double forceRateAboveExpectedRecoverGainNmPerNps = 0.00025;
        double forceRateAboveExpectedRecoverLimitNm = 0.3;
        double forcePlatformCaptureRateThresholdNPerSec = 20.0;
        double forcePlatformCaptureEnableErrorN = 5.0;
        double forcePlatformCaptureDisableErrorN = 2.0;
        double forcePlatformCaptureGainNmPerN = 0.04;
        double forcePlatformCaptureLimitUpNm = 0.8;
        double forcePlatformCaptureLimitDownNm = 0.5;
        double forcePlatformCaptureSlewRateNmPerSec = 4.0;
        double forcePlatformCaptureHoldTimeSec = 0.3;
        double forcePlatformCaptureReleaseRateNmPerSec = 1.0;
        double forcePlatformCaptureTrajectoryLookAheadSec = 0.3;
        double forcePlatformCaptureTrajectoryToleranceN = 1.0;
        double forcePlatformCaptureMeasuredRateThresholdNPerSec = 200.0;
        double forcePlatformCaptureMeasuredRateHoldTimeSec = 0.08;
        double forceFuzzyFeedForwardDropRatePerSec = 2.0;
        double forceFuzzyFeedForwardFastDescentDropRatePerSec = 1.2;
        double forceFeedForwardOnlyFrictionCoulombNm = 0.0;
        double forceFeedForwardOnlyFrictionViscousNmPerRadPerSec = 0.0;
        double forceFeedForwardOnlyFrictionVelocityDeadbandRadPerSec = 0.05;
        bool forceFeedForwardOnlyStaticFrictionEnabled = false;
        double forceFeedForwardOnlyStaticFrictionScale = 0.5;
        double forceFeedForwardOnlyStaticFrictionForceRateDeadbandNPerSec = 1.0;
        double forceFeedForwardOnlyStaticFrictionVelocityFadeStartRadPerSec = 0.02;
        double forceFeedForwardOnlyStaticFrictionVelocityFadeEndRadPerSec = 0.20;
        double forceFeedForwardOnlyInertiaScale = 0.0;
        double forceFeedForwardOnlyWinchInertiaKgM2 = 0.004682885;
        double forceFeedForwardOnlyMotorInertiaKgM2 = 0.01135;
        double forceIntegralReleaseExpectedRateThresholdNPerSec = 80.0;
        double forceIntegralReleaseOverForceThresholdN = 5.0;
        double forceIntegralReleaseTimeConstantSec = 0.08;
        double forceIntegralTorqueLimitNm = 2.0;
        double forceTorqueCommandLimitNm = 40.0;
        double forceTorqueCommandSlewRateNmPerSec = 6.0;
        double forceTorqueVelocityDampingNmPerVelocity = 0.0;
        bool forcePid0525DynamicTrackEnabled = true;
        ForcePid0525DynamicTrackMode forcePid0525DynamicTrackMode =
                ForcePid0525DynamicTrackMode::FeedForwardOnly;
        bool forcePid0525HybridEnabled = true;
        bool forcePid0525UseBangBangPretension = true;
        bool forcePid0525FreezeIntegralDuringTrack = true;
        double forcePid0525BiasLearnErrorN = 3.0;
        double forcePid0525BiasLearnHoldTimeSec = 0.3;
        double forcePid0525TrackKffNmPerN = 0.045;
        double forcePid0525TrackKpNmPerN = 0.015;
        double forcePid0525TrackPTorqueLimitNm = 0.7;
        double forcePid0525TrackTorqueSlewRateNmPerSec = 30.0;
        double forcePid0525TrackBlendTimeSec = 0.15;
        double forcePid0525TrackForceRateDampingNmPerNps = 0.0015;
        double forcePid0525TrackForceRateDampingLimitNm = 0.5;
        double forcePid0525TrackForceRateDeadbandNPerSec = 50.0;
        double forcePid0525TrackForceRateFilterHz = 6.0;
        std::vector<ForceFeedForwardOnlyDirectionalProfile> forceFeedForwardOnlyUpProfiles;
        std::vector<ForceFeedForwardOnlyDirectionalProfile> forceFeedForwardOnlyDownProfiles;
        std::vector<double> forceSensorHome;
        std::vector<double> expectedForce;
        std::vector<double> pidP;
        std::vector<double> pidI;
        std::vector<double> pidD;
        std::vector<AxisConfig> axes;
        std::vector<bool> protectedMotionAxes;
        std::vector<bool> motorPositionLimitRecoveryAxes;
    };

    // 末端遥控的Trace授权阶段。速度命令只允许在Running阶段生成；
    // Transition阶段只随正常控制循环排空、验证Trace，不增加周期内重复读取。
    enum class EndpointRemoteTracePhase {
        Inactive,
        TransitionPrepared,
        TransitionAcquiring,
        RunningProfileAwaitingFrame,
        Running,
        Faulted
    };
    enum class EndpointRemoteDispatchPhase {
        Idle = 0,
        PreparedForCompositeTraceCommand,
        CompositeCompletedAwaitNextCycle
    };

    struct Snapshot {
        quint64 sequence = 0;
        bool forceThreadRunning = false;
        bool forceExpectedFromExternal = false;
        ForcePidOutputMode forcePidOutputMode = ForcePidOutputMode::Pid0624;
        std::vector<double> motorAbsPos;
        std::vector<double> motorRelRawPos;
        std::vector<double> motorVel;
        // 在线速度扩展启用时，与位置反馈来自同一 Runtime Trace 帧。
        std::vector<double> motorTraceCommandVelocity;
        std::vector<double> motorTraceActualVelocity;
        std::vector<quint16> motorTraceStatusWord;
        std::vector<int> motorTraceStateMachine;
        bool endpointRemoteControlActive = false;
        bool endpointRemoteControlRunning = false;
        EndpointRemoteTracePhase endpointRemoteTracePhase =
                EndpointRemoteTracePhase::Inactive;
        // 原始电机坐标实际力矩；Lite 中正=放绳、负=收绳。
        std::vector<double> motorTorqueNm;
        // 已完成绳索方向映射、准备下发的原始电机坐标力矩命令。
        std::vector<double> motorCommand;
        std::vector<double> forceSensorValue;
        std::vector<double> expectedForce;
        // 最近一次 Runtime Trace 硬件帧元数据。SafetyMonitor 在 Lite
        // 单轴模式下用它判断现场总线反馈是否新鲜，避免额外诊断 API
        // 阻塞同一个 HardwareThread。
        qint64 runtimeTraceFrameWallClockUs = 0;
        qint64 runtimeTraceFrameMonotonicUs = 0;
        // Trace读取完成时计算的固定帧龄，供新控制快照的5 ms接纳判据使用。
        // latestSnapshot()不得随读取时刻改写这个值。
        qint64 runtimeTraceNewestFrameAgeUs = -1;
        // 当前读取缓存副本时的帧龄，只用于显示和链路停更诊断。
        qint64 runtimeTraceCurrentFrameAgeUs = -1;
        int runtimeTraceFrameCount = 0;
        int runtimeTraceSamplePeriodUs = 0;
        quint64 runtimeTraceLogicalFrameSequence = 0;
        HardwareInterface::RuntimeTraceUsageProfile runtimeTraceUsageProfile =
                HardwareInterface::RuntimeTraceUsageProfile::Base;
        quint64 runtimeTraceUsageProfileGeneration = 0;
        quint64 runtimeTraceConfigurationGeneration = 0;
        quint64 runtimeTraceEndpointRemoteSessionToken = 0;
        bool runtimeTraceStatusFaultLatched = false;
        int runtimeTraceStatusFaultAxis = -1;
        quint16 runtimeTraceStatusFaultWord = 0;
        int runtimeTraceStatusFaultStateMachine = -1;
        quint64 runtimeTraceStatusFaultLogicalFrameSequence = 0;
        bool runtimeTraceFromHardware = false;
        bool runtimeTraceFrameSequenceValid = false;
        bool runtimeTraceTimingReliable = false;
        bool runtimeTraceFifoCaughtUp = false;
        bool runtimeTraceLost = false;
        std::vector<qint64> forceSensorTraceFrameMonotonicUs;
        TimingDiagnostics timingDiagnostics;
    };

    struct ForcePidTraceSample {
        qint64 wallClockUs = 0;
        double controlDtSec = 0.0;
        std::vector<double> forceSensorValue;
        std::vector<double> expectedForce;
        std::vector<double> motorCommand;
        std::vector<double> motorTorqueNm;
        std::vector<double> pidOutput;
        std::vector<double> pidError;
        std::vector<double> pidPTerm;
        std::vector<double> pidITerm;
        std::vector<double> pidDTerm;
        std::vector<double> pidIntegral;
        std::vector<double> pidMeasuredDerivativeRaw;
        std::vector<double> pidMeasuredDerivativeFiltered;
        std::vector<double> pidMeasuredDerivativeControl;
        std::vector<double> pidExpectedDerivativeRaw;
        std::vector<double> pidExpectedDerivativeFiltered;
        std::vector<double> pidFeedForwardRaw;
        std::vector<double> pidFeedForwardTerm;
        std::vector<double> pidFeedForwardFrictionTerm;
        std::vector<int> pidFeedForwardSelectedDynamicProfile;
        std::vector<double> pidStaticFrictionDirection;
        std::vector<double> pidStaticFrictionSpeedScale;
        std::vector<double> pidStaticFrictionRaw;
        std::vector<double> pidStaticFrictionAfterFade;
        std::vector<double> pidStaticFrictionAfterSmooth;
        std::vector<double> pidFeedForwardVelocityTerm;
        std::vector<double> pidFeedForwardAccelerationTerm;
        std::vector<double> pidExpectedRopeVelocityRadPerSec;
        std::vector<double> pidExpectedRopeAccelerationRadPerSec2;
        std::vector<double> pidExpectedRateFeedForwardTerm;
        std::vector<double> pidExpectedRateFeedForwardScale;
        std::vector<double> pidForceRateError;
        std::vector<double> pidForceRateErrorDampingTerm;
        std::vector<double> pidPlatformCaptureTerm;
        std::vector<double> pidPlatformCaptureTargetTerm;
        std::vector<int> pidPlatformCaptureState;
        std::vector<double> pidFuzzyFeedForwardTargetScale;
        std::vector<double> pidFuzzyFeedForwardScale;
        std::vector<double> pidFuzzyFeedForwardRecoveryRate;
        std::vector<double> pidFuzzyKpScale;
        std::vector<double> pidFuzzyKiScale;
        std::vector<double> pidFuzzyVelocityDampingScale;
        std::vector<double> pidFuzzyPositivePLimit;
        std::vector<double> pidFuzzyNegativePLimit;
        std::vector<int> pidFuzzyFeedForwardRecoveryLimited;
        std::vector<int> pidFuzzyPLimitApplied;
        std::vector<int> pidFuzzyState;
        std::vector<int> pidIntegralReleaseApplied;
        std::vector<int> pidAntiWindup;
        std::vector<int> pidOutputLimited;
        std::vector<int> torqueSaturated;
        std::vector<int> torqueSlewLimited;
        std::vector<double> motorVel;
        std::vector<int> pid0525HybridState;
        std::vector<int> pid0525HybridBiasValid;
        std::vector<double> pid0525HybridHoldBiasNm;
        std::vector<double> pid0525HybridCaptureForceN;
        std::vector<double> pid0525HybridFeedForwardTermNm;
        std::vector<double> pid0525HybridFeedbackTermNm;
        std::vector<double> pid0525HybridBlend;
        std::vector<int> forceControlAxisIndex;
        std::vector<int> forceControlSensorIndex;
    };

    // 绑定硬件接口；worker 通常运行在独立线程中执行力控周期。
    explicit ControlWorker(HardwareInterface* hardware, QObject* parent = nullptr);

    // 更新力控配置，包含轴映射、PID 参数、限幅和运行开关。
    void setConfig(const Config& config);
    // 从外部轨迹或混合控制模式写入期望绳力。
    void setExternalExpectedForce(const std::vector<double>& expectedForce);
    bool setExternalExpectedForceTrajectory(const std::vector<std::vector<double>>& expectedForceTraj,
                                            const std::vector<double>& timeStamp);
    bool setExternalExpectedForceTrajectory(const std::vector<std::vector<double>>& expectedForceTraj,
                                            const std::vector<double>& timeStamp,
                                            const std::vector<std::vector<double>>& ropeVelocityRadPerSecTraj,
                                            const std::vector<std::vector<double>>& ropeAccelerationRadPerSec2Traj);
    bool startExternalExpectedForceTrajectoryClock(qint64 startMonotonicUs = 0);
    // 清除外部期望力，回退到 Config 中的 expectedForce。
    void clearExternalExpectedForce();
    // Reset PID/fuzzy/platform-capture states without changing active torque commands.
    void resetForcePidControllerState();
    // 返回最近一次控制/传感器循环更新的快照。
    Snapshot latestSnapshot() const;
    // 查询指定时间窗口内的力传感器原始采样。
    QVector<SensorValueSample> sensorValueHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 查询指定时间窗口内的传感器帧间隔诊断。
    QVector<DiagnosticRawSample> sensorTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 查询 Trace 读取调用间隔诊断。
    QVector<DiagnosticRawSample> sensorTraceReadTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 查询控制循环周期诊断。
    QVector<DiagnosticRawSample> controlLoopTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    QVector<SensorValueSample> sensorTraceValueHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    QVector<DiagnosticRawSample> sensorTraceFrameTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 会话记录开启时保留完整原始历史；默认只保留短窗口低频诊断缓存。
    void setDiagnosticRawHistoryFullRecordingEnabled(bool enabled);
    std::vector<ForcePidTraceSample> takeForcePidTraceSamples();
    void clearForcePidTraceSamples();
    // 清零控制和传感器计时统计。
    void resetTimingDiagnostics();
    // 预设轨迹八轴在线变速。准备/启动由 GUI 通过阻塞队列调用，周期执行始终留在本 worker。
    bool prepareOnlineVelocityControl(const OnlineVelocityPlan& plan,
                                      const OnlineVelocityConfig& config,
                                      QString* errorMessage = nullptr);
    bool startOnlineVelocityControl(QString* errorMessage = nullptr);
    void stopOnlineVelocityControl(bool emergency,
                                   const QString& reason = QStringLiteral("用户停止"));
    OnlineVelocityStatus onlineVelocityStatus() const;
    // 末端开环遥控与预设轨迹共用八轴在线速度硬件链路，但使用独立命令源。
    bool prepareEndpointRemoteControl(const EndpointRemoteConfig& config,
                                      quint64 inputSessionToken,
                                      QString* errorMessage = nullptr);
    bool startEndpointRemoteControl(QString* errorMessage = nullptr);
    // 线程安全地提交遥控输入。独立输入监督线程只覆盖“最新输入邮箱”，
    // 控制循环再消费，避免排队调用延迟并误触发输入心跳超时。
    void updateEndpointRemoteInput(EndpointRemoteMotionMode motionMode,
                                   const std::array<double, 3>& normalizedDirection,
                                   quint64 sequence,
                                   quint64 inputSessionToken,
                                   bool uiSourceFresh,
                                   qint64 uiSourceAgeUs);
    // 线程安全的高优先级退出邮箱。UI只提交请求，不阻塞等待ControlWorker
    // 或HardwareThread；控制循环在下一次Trace读取前优先执行停机。
    void requestEndpointRemoteStop(bool emergency,
                                   const QString& reason,
                                   quint64 inputSessionToken);
    // SafetyMonitor触发硬件急停时，用该线程安全邮箱先撤销遥控会话，
    // 避免ControlWorker正在Trace读取时继续生成后续速度命令。
    void requestEndpointRemoteSafetyStop(const QString& reason);
    void stopEndpointRemoteControl(bool emergency,
                                   const QString& reason = QStringLiteral("用户退出末端遥控"));
    EndpointRemoteStatus endpointRemoteStatus() const;

public slots:
    // 启动定时控制循环。
    void start();
    // 停止控制循环并撤销活动力矩命令。
    void stop();

signals:
    void displayInfoSignal(std::string info, std::string type);
    void actualTorqueLimitExceeded(int axisIndex,
                                   double actualTorqueNm,
                                   double limitNm,
                                   double sustainedMs);

private:
    // 主控制循环：读取硬件反馈、计算 PID/限幅并下发力矩。
    void controlLoop();
    // 传感器循环：按更高频率读取 Trace/缓存力传感器数据。
    void sensorLoop();
    // 线程安全复制当前配置，避免循环中长时间持锁。
    Config currentConfig() const;
    // 选择实际使用的期望力，并标记是否来自外部输入。
    std::vector<double> activeExpectedForce(const Config& config,
                                            bool& fromExternal,
                                            std::vector<double>* expectedForceDerivative = nullptr,
                                            std::vector<double>* expectedRateFeedForwardScale = nullptr,
                                            std::vector<double>* expectedRopeVelocityRadPerSec = nullptr,
                                            std::vector<double>* expectedRopeAccelerationRadPerSec2 = nullptr,
                                            std::vector<int>* platformCaptureTrajectoryPlatform = nullptr);
    bool externalExpectedForceTrajectoryMotionActive(qint64 nowUs,
                                                     bool* trajectoryPresent = nullptr) const;
    // 对力传感器读数应用一阶低通滤波。
    std::vector<double> applyForceSensorLowPass(const Config& config,
                                                const std::vector<double>& rawValue,
                                                double dtSec);
    // 保证力矩命令历史数组与轴数一致。
    void ensureTorqueCommandStateSize(int axisCount);
    // 保证实际力矩超限连续判定状态与轴数一致。
    void ensureActualTorqueLimitStateSize(int axisCount);
    // 清除实际力矩超限连续判定状态。
    void resetActualTorqueLimitState(int axisCount = 0);
    enum class ForcePid0525HybridState {
        Idle = 0,
        PreloadAcquire = 1,
        BiasLearnHold = 2,
        TrackBlend = 3,
        Track = 4,
        TrackExitBlend = 5
    };
    void ensureForcePid0525HybridStateSize(int axisCount);
    void resetForcePid0525HybridState(int axisCount = 0);
    void resetForcePid0525HybridAxis(int axisIndex, bool preserveCurrentTorqueAsBias = true);
    void refreshForceControlFaultLatches(const Config& config, bool forceThreadRunning);
    // 保证力反馈控制状态数组与传感器通道数一致。
    void ensureForceFeedbackStateSize(int sensorCount);
    // 重置力反馈积分和测量力微分历史。
    void resetForceFeedbackState(int sensorCount = 0);
    // 重置指定传感器通道的力反馈状态。
    void resetForceFeedbackChannel(int sensorIndex);
    // 重置力矩命令、偏置和 warm start 状态。
    void resetTorqueCommandState(int axisCount = 0);
    // 停止当前所有已激活的力矩模式轴。
    void stopActiveTorqueCommands(const Config& config);
    // 停止指定轴的力矩模式。
    bool stopTorqueModeAxis(int axisIndex);
    double warmStartTorqueForAxis(int axisIndex,
                                  const std::vector<double>& motorTorqueNm,
                                  double torqueLimitNm) const;
    // 向指定轴下发力矩模式命令，并维护命令状态缓存。
    bool commandTorqueModeAxis(int axisIndex, double torqueNm);
    // 更新对外快照，供 UI、安全监控和诊断读取。
    void updateSnapshot(const Config& config,
                        const std::vector<double>& motorAbsPos,
                        const std::vector<double>& motorRelRawPos,
                        const std::vector<double>& motorVel,
                        const std::vector<double>& motorTorqueNm,
                        const std::vector<double>& motorCommand,
                        const std::vector<double>& forceSensorValue,
                        const std::vector<double>& expectedForce,
                        bool forceThreadRunning,
                        bool expectedFromExternal,
                        const HardwareInterface::RuntimeTraceSnapshot& runtimeTraceSnapshot);
    void appendForcePidTraceSample(ForcePidTraceSample&& sample);
    // 节流输出提示信息，避免周期循环刷屏。
    void throttledInfo(const QString& message, const std::string& type, int throttleMs = 1000);
    void processOnlineVelocityControl(const Config& config,
                                      const HardwareInterface::RuntimeTraceSnapshot& traceSnapshot,
                                      qint64 nowUs);
    void publishOnlineVelocityStatus();
    void processEndpointRemoteControl(const Config& config,
                                      const HardwareInterface::RuntimeTraceSnapshot& traceSnapshot,
                                      qint64 nowUs,
                                      const EndpointRemoteTimingContext& timing);
    void completePreparedEndpointRemoteCommand(
            const HardwareInterface::EndpointRemoteTraceCommandResult& result,
            qint64 nowUs,
            qint64 workerLoopEntryUs);
    void clearPreparedEndpointRemoteCommand();
    void startEndpointRemoteAttribution(qint64 nowUs);
    void observeEndpointRemoteAttribution(
            const HardwareInterface::RuntimeTraceSnapshot& traceSnapshot,
            const EndpointRemoteTimingContext& timing);
    void observeEndpointRemoteCommandAttribution(
            const HardwareInterface::EndpointRemoteVelocityCommandReport& report);
    void freezeEndpointRemoteAttribution();
    void reportEndpointRemoteAttribution(bool finalReport, qint64 nowUs);
    void finishEndpointRemoteAttribution(qint64 nowUs);
    void publishEndpointRemoteStatus();
    void clearEndpointRemoteInputMailbox();
    void consumeEndpointRemoteInputMailbox();
    bool consumeEndpointRemoteStopRequest();
    quint64 endpointRemoteInputSessionToken() const;
    bool restoreEndpointRemoteRuntimeTraceProfile(
            quint64 inputSessionToken,
            QString* errorMessage = nullptr);

    HardwareInterface* hardwareInterface = nullptr;
    QTimer* timer = nullptr;
    mutable QMutex configMutex;
    mutable QMutex snapshotMutex;
    mutable QMutex timingHistoryMutex;
    mutable QMutex forcePidTraceMutex;
    mutable QMutex onlineVelocityMutex;
    mutable QMutex endpointRemoteInputMutex;
    Config config;
    Snapshot snapshot;
    TimingDiagnostics timingDiagnostics;
    QVector<SensorValueSample> sensorValueRawHistory;
    QVector<DiagnosticRawSample> sensorFrameRawHistory;
    QVector<DiagnosticRawSample> sensorTraceReadRawHistory;
    QVector<DiagnosticRawSample> controlLoopRawHistory;
    std::atomic_bool diagnosticRawHistoryFullRecordingEnabled{false};
    std::vector<ForcePidTraceSample> forcePidTraceRingBuffer;
    size_t forcePidTraceRingStartIndex = 0;
    size_t forcePidTraceRingCount = 0;
    qint64 lastSensorValueHistoryTrimMs = 0;
    qint64 lastSensorFrameHistoryTrimMs = 0;
    qint64 lastSensorTraceReadHistoryTrimMs = 0;
    qint64 lastControlLoopHistoryTrimMs = 0;
    qint64 lastSensorValueRawHistoryAppendUs = 0;
    qint64 lastSensorFrameRawHistoryAppendUs = 0;
    qint64 lastSensorTraceReadRawHistoryAppendUs = 0;
    qint64 lastControlLoopRawHistoryAppendUs = 0;
    std::vector<double> latestForceSensorValue;
    bool hasLatestForceSensorValue = false;
    std::vector<double> filteredForceSensorValue;
    bool hasFilteredForceSensorValue = false;
    std::vector<double> cachedMotorHome;
    std::vector<double> cachedMotorVel;
    std::vector<double> lastMotorVelocityPosition;
    std::vector<double> cachedMotorTorqueNm;
    std::vector<double> externalExpectedForce;
    bool hasExternalExpectedForce = false;
    std::vector<std::vector<double>> externalExpectedForceTrajectory;
    std::vector<double> externalExpectedForceTrajectoryTimeStamp;
    bool hasExternalExpectedForceTrajectory = false;
    qint64 externalExpectedForceTrajectoryStartUs = 0;
    std::vector<double> lastExternalExpectedForceTrajectoryValue;
    std::vector<double> lastExternalExpectedForceTrajectoryDerivative;
    std::vector<double> lastExternalExpectedForceTrajectoryRateFeedForwardScale;
    std::vector<std::vector<double>> externalExpectedRopeVelocityRadPerSecTrajectory;
    std::vector<std::vector<double>> externalExpectedRopeAccelerationRadPerSec2Trajectory;
    std::vector<double> lastExternalExpectedRopeVelocityRadPerSec;
    std::vector<double> lastExternalExpectedRopeAccelerationRadPerSec2;
    std::vector<int> lastExternalExpectedForceTrajectoryPlatform;
    bool hasLastExternalExpectedForceTrajectoryValue = false;
    bool lastForceThreadEnabled = false;
    bool lastAllCableForceDragModeEnabled = false;
    bool lastForceFeedForwardOnlyTestModeEnabled = false;
    ForcePidOutputMode lastForcePidOutputMode = ForcePidOutputMode::Pid0624;
    bool lastForcePid0525HybridEnabled = false;
    ForcePid0525DynamicTrackMode lastForcePid0525DynamicTrackMode =
            ForcePid0525DynamicTrackMode::AC;
    bool lastForcePid0525UseBangBangPretension = true;
    bool lastForcePid0525HybridMotionActive = false;
    bool softwareLimitEmergencyStopActive = false;
    bool actualTorqueLimitEmergencyStopActive = false;
    std::vector<qint64> actualTorqueLimitOverStartUs;
    std::vector<qint64> actualTorqueLimitLastSampleUs;
    std::vector<double> actualTorqueLimitPeakNm;
    ForceController forceController;
    OnlineVelocityControl onlineVelocityControl;
    OnlineVelocityStatus onlineVelocityStatusCache;
    EndpointRemoteControl endpointRemoteControl;
    EndpointRemoteStatus endpointRemoteStatusCache;
    EndpointRemoteTracePhase endpointRemoteTracePhase =
            EndpointRemoteTracePhase::Inactive;
    EndpointRemoteDispatchPhase endpointRemoteDispatchPhase =
            EndpointRemoteDispatchPhase::Idle;
    EndpointRemoteStep preparedEndpointRemoteStep;
    qint64 endpointRemoteFreshFrameDeferredStartUs = 0;
    qint64 endpointRemoteLastFreshFrameDeferredDiagnosticUs = 0;
    qint64 endpointRemoteTransitionStartUs = 0;
    qint64 endpointRemoteTransitionLastDiagnosticUs = 0;
    struct EndpointRemoteAttributionStats {
        bool active = false;
        qint64 startUs = 0;
        quint64 traceSampleCount = 0;
        qint64 latestTraceReadUs = 0;
        qint64 totalTraceReadUs = 0;
        qint64 maximumTraceReadUs = 0;
        qint64 latestTraceQueueWaitUs = 0;
        qint64 totalTraceQueueWaitUs = 0;
        qint64 maximumTraceQueueWaitUs = 0;
        qint64 latestTraceHardwareUs = 0;
        qint64 totalTraceHardwareUs = 0;
        qint64 maximumTraceHardwareUs = 0;
        qint64 latestTraceDataApiUs = 0;
        qint64 totalTraceDataApiUs = 0;
        qint64 maximumTraceDataApiUs = 0;
        qint64 maximumReliableFrameAgeUs = -1;
        quint64 fifoNotCaughtUpCount = 0;
        quint64 traceLostCount = 0;
        quint64 velocityCommandCount = 0;
        quint64 freshFrameDeferredCount = 0;
        qint64 latestValidationToSubmitUs = 0;
        qint64 totalValidationToSubmitUs = 0;
        qint64 maximumValidationToSubmitUs = 0;
        qint64 latestCommandQueueWaitUs = 0;
        qint64 totalCommandQueueWaitUs = 0;
        qint64 maximumCommandQueueWaitUs = 0;
        qint64 latestHardwarePreSdkOrRejectUs = 0;
        qint64 totalHardwarePreSdkOrRejectUs = 0;
        qint64 maximumHardwarePreSdkOrRejectUs = 0;
        quint64 sdkBatchCount = 0;
        qint64 latestSdkBatchUs = 0;
        qint64 totalSdkBatchUs = 0;
        qint64 maximumSdkBatchUs = 0;
        qint64 latestCommandHardwareUs = 0;
        qint64 totalCommandHardwareUs = 0;
        qint64 maximumCommandHardwareUs = 0;
        qint64 latestCommandCallUs = 0;
        qint64 totalCommandCallUs = 0;
        qint64 maximumCommandCallUs = 0;
        HardwareInterface::EndpointRemoteVelocityCommandReport
                latestVelocityCommandReport;
        bool motorTimingFrozen = false;
        qint64 frozenAtUs = 0;
        HardwareInterface::MotorEnableQueryTimingSnapshot frozenMotorTiming;
    } endpointRemoteAttribution;
    quint64 activeEndpointRemoteInputSessionToken = 0;
    EndpointRemoteMotionMode pendingEndpointRemoteMotionMode =
            EndpointRemoteMotionMode::None;
    std::array<double, 3> pendingEndpointRemoteDirection{};
    quint64 pendingEndpointRemoteInputSequence = 0;
    quint64 pendingEndpointRemoteInputSessionToken = 0;
    qint64 pendingEndpointRemoteInputReceivedUs = 0;
    qint64 pendingEndpointRemoteUiSourceAgeUs = 0;
    bool pendingEndpointRemoteUiSourceFresh = true;
    std::atomic_bool pendingEndpointRemoteInputValid{false};
    bool pendingEndpointRemoteStopEmergency = false;
    QString pendingEndpointRemoteStopReason;
    quint64 pendingEndpointRemoteStopSessionToken = 0;
    std::atomic_bool pendingEndpointRemoteStopValid{false};
    std::vector<double> unloadFeedForwardBlend;
    ForcePid0525 forcePid0525;
    std::vector<double> lastTorqueCommandNm;
    std::vector<double> warmStartTorqueNm;
    std::vector<double> torqueCommandBiasNm;
    std::vector<bool> torqueCommandActive;
    std::vector<bool> forceControlFaultLatched;
    std::vector<ForcePid0525HybridState> forcePid0525HybridState;
    std::vector<bool> forcePid0525HybridBiasValid;
    std::vector<double> forcePid0525HybridHoldBiasNm;
    std::vector<double> forcePid0525HybridCaptureForceN;
    std::vector<double> forcePid0525HybridStableTimeSec;
    std::vector<double> forcePid0525HybridBlendElapsedSec;
    std::vector<double> forcePid0525HybridBlendStartTorqueNm;
    std::vector<bool> forcePid0525HybridRateInitialized;
    std::vector<double> forcePid0525HybridLastActualForceN;
    std::vector<double> forcePid0525HybridLastExpectedForceN;
    std::vector<double> forcePid0525HybridActualForceRateFilteredNPerSec;
    std::vector<double> forcePid0525HybridExpectedForceRateFilteredNPerSec;
    std::vector<int> forceFeedForwardOnlyDirectionalProfileSign;
    std::vector<int> forceFeedForwardOnlyStaticFrictionDirectionSign;
    std::vector<double> forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm;
    std::vector<bool> forceFeedForwardOnlyStaticFrictionSmoothedValid;
    qint64 lastInfoMs = 0;
    qint64 lastControlLoopTimestampUs = 0;
    qint64 nextControlLoopDueUs = 0;
    qint64 lastControlLoopSampleIntervalUs = 0;
    qint64 previousControlLoopDurationUs = 0;
    qint64 lastSensorFrameTimestampUs = 0;
    qint64 lastSensorFrameWallClockUs = 0;
    qint64 lastTraceExpandedSensorFrameTimestampUs = 0;
    qint64 lastSensorTraceReadCallUs = 0;
    qint64 nextSensorReadDueUs = 0;
    qint64 lastSensorSampleIntervalUs = 0;
    qint64 lastMotorHomeRefreshUs = 0;
    qint64 lastMotorVelocityRefreshUs = 0;
    qint64 lastMotorTorqueRefreshUs = 0;
};

#endif // CONTROLWORKER_H
