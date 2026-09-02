#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

/*
 * 文件总览：
 * - HardwareInterface 是硬件访问层，封装雷赛运动控制卡、EtherCAT 电机、力传感器、PDO/Trace 采样和 PVT 执行接口。
 * - 上层模块只通过本类读写位置、速度、力矩、传感器值和诊断状态，避免 UI/控制线程直接依赖板卡 API 细节。
 * - 大量结构体用于把底层 API 返回值整理成可诊断的数据快照，便于安全监控和界面显示定位问题。
 */

#include <QObject>
#include <QtCore/QtCore>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <deque>
#include <thread>
#include <type_traits>
#include <vector>

#include "macro.h"
#include "LTDMC.h"

#pragma execution_character_set("utf-8")

class HardwareInterface : public QObject
{
    Q_OBJECT
public:
    struct DiagnosticRawSample {
        qint64 wallClockMs = 0;
        qint64 intervalUs = 0;
        QString apiEvent;
    };

    struct MotorPositionRawSample {
        qint64 wallClockMs = 0;
        qint64 monotonicUs = 0;
        qint64 intervalUs = 0;
        QString source;
        std::vector<double> positions;
        qint64 wallClockUs = 0;
        qint64 durationUs = 0;
    };

    struct MotorTraceFeedbackRawSample {
        qint64 wallClockMs = 0;
        qint64 wallClockUs = 0;
        qint64 monotonicUs = 0;
        qint64 intervalUs = 0;
        quint32 frameSequence = 0;
        bool frameSequenceValid = false;
        std::vector<qint64> feedbackRawPulse;
        std::vector<bool> feedbackValid;
    };

    struct RuntimeTraceFetchTimingSample {
        qint64 wallClockMs = 0;
        qint64 wallClockUs = 0;
        qint64 monotonicUs = 0;
        qint64 intervalUs = 0;
        qint64 apiDurationUs = 0;
        int actualReadLength = 0;
        int frameBytes = 0;
        int frameCount = 0;
        int requestedFrameCount = 0;
        int fifoValidBefore = 0;
        int fifoValidAfter = 0;
        int fifoFreeAfter = 0;
        int estimatedProducedFrameCount = 0;
        int traceSamplePeriodUs = 0;
        qint64 newestFrameAgeUs = -1;
        bool latestOnly = false;
        bool fifoCaughtUp = false;
        bool timingReliable = false;
        bool traceLost = false;
        bool frameSequenceValid = false;
        quint32 firstFrameSequence = 0;
        quint32 lastFrameSequence = 0;
        std::vector<quint32> frameSequences;
    };

    struct MotorEnableQueryTimingSnapshot {
        quint64 queryCount = 0;
        qint64 totalQueueWaitUs = 0;
        qint64 latestQueueWaitUs = 0;
        qint64 maximumQueueWaitUs = 0;
        qint64 totalApiDurationUs = 0;
        qint64 latestApiDurationUs = 0;
        qint64 maximumApiDurationUs = 0;
        qint64 totalCallDurationUs = 0;
        qint64 latestCallDurationUs = 0;
        qint64 maximumCallDurationUs = 0;
    };

    struct PvtTableUploadTimingSample {
        qint64 wallClockMs = 0;
        qint64 wallClockUs = 0;
        qint64 monotonicUs = 0;
        qint64 totalUploadUs = 0;
        double averageUploadUsPerPoint = 0.0;
        int pointCount = 0;
        int axisCount = 0;
        std::vector<int> motorIndex;
        QString source;
        bool traceStartDelayValid = false;
        quint32 traceCommandStartFrameSequence = 0;
        quint32 traceFeedbackStartFrameSequence = 0;
        quint64 traceStartDelayFrameCount = 0;
        int ethercatBusCycleUs = 500;
        qint64 traceStartDelayUs = -1;
        int traceCommandStartAxis = -1;
        int traceFeedbackStartAxis = -1;
    };

    struct DiagnosticsSnapshot {
        quint64 communicationEventCount = 0;
        quint64 communicationIntervalCount = 0;
        qint64 communicationIntervalSumUs = 0;
        qint64 latestCommunicationIntervalUs = 0;
        quint64 motorCommandEventCount = 0;
        quint64 motorCommandIntervalCount = 0;
        qint64 motorCommandIntervalSumUs = 0;
        qint64 latestMotorCommandIntervalUs = 0;
    };

    struct FieldbusConsumeTimeSnapshot {
        qint64 wallClockMs = 0;
        int apiResult = -1;
        WORD cardNo = 0;
        WORD portNum = 2;
        DWORD averageTimeUs = 0;
        DWORD maxTimeUs = 0;
        quint64 cycles = 0;
        bool success = false;
    };

    enum class ConnectionState {
        Disconnected = 0,
        Connected,
        Disabled,
        Fault
    };

    struct ConnectionItemDiagnostics {
        ConnectionState state = ConnectionState::Disconnected;
        int hardwareAxis = -1;
        int stateMachine = -1;
        int slaveAddress = -1;
        int subSlaveAddress = -1;
        int busState = -1;
        long statusWord = 0;
        long stopReason = 0;
        int errorCode = 0;
        int apiResult = 0;
        int stopReasonApiResult = 0;
    };

    struct ConnectionDiagnostics {
        ConnectionItemDiagnostics controller;
        std::vector<ConnectionItemDiagnostics> motorAxes;
        std::vector<ConnectionItemDiagnostics> forceSensors;
    };

    struct PdoTraceObjectConfig {
        WORD index = 0;
        WORD subIndex = 0;
    };

    struct PdoTraceProbeConfig {
        WORD channel = 2;
        WORD slaveAddress = 0;
        DWORD traceLength = 1024;
        DWORD readStartAddress = 0;
        DWORD readLengthBytes = 0;
        DWORD maxReadLengthBytes = 1024 * 1024;
        int waitTimeoutMs = 1000;
        int pollIntervalMs = 10;
        std::vector<PdoTraceObjectConfig> objects;
    };

    struct PdoTraceProbeResult {
        bool success = false;
        bool timedOut = false;
        short preStopResult = -1;
        short clearBeforeStartResult = -1;
        short startResult = -1;
        short getNumResult = -1;
        short stateResult = -1;
        short readResult = -1;
        short stopResult = -1;
        DWORD dataNum = 0;
        DWORD sizeOfEachPacket = 0;
        WORD traceState = 0;
        DWORD requestedReadLength = 0;
        DWORD actualReadLength = 0;
        int pollCount = 0;
        QByteArray data;
        QStringList messages;
    };

    struct TraceObjectConfig {
        short dataType = 0;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short dataBytes = 4;
    };

    struct TraceProbeConfig {
        bool configureSource = false;
        WORD source = 0;
        short traceCycle = 0;
        short lostHandle = 0;
        short traceType = 0;
        short triggerObjectIndex = 0;
        short triggerType = 0;
        int mask = 0;
        long long condition = 0;
        int bufferSizeBytes = 4096;
        int maxBufferSizeBytes = 1024 * 1024;
        int waitTimeoutMs = 1000;
        int pollIntervalMs = 10;
        std::vector<TraceObjectConfig> objects;
    };

    struct TraceProbeResult {
        bool success = false;
        bool timedOut = false;
        short setSourceResult = -1;
        short stopBeforeConfigResult = -1;
        short dataResetBeforeConfigResult = -1;
        short setConfigResult = -1;
        short resetConfigObjectResult = -1;
        short addConfigObjectResult = -1;
        short dataStartResult = -1;
        short getFlagResult = -1;
        short getStateResult = -1;
        short getDataResult = -1;
        short dataStopResult = -1;
        short startFlag = 0;
        short triggeredFlag = 0;
        short lostFlag = 0;
        int validNum = 0;
        int freeNum = 0;
        int objectTotalBytes = 0;
        int objectTotalNum = 0;
        int actualReadLength = 0;
        int pollCount = 0;
        QByteArray data;
        QStringList messages;
    };

    enum class RuntimeTraceConfigType {
        G3 = 0,
        Lite
    };

    // Runtime Trace的使用语义与G3/Lite硬件布局正交。显式状态用于约束
    // 末端遥控专用解析和命令授权，避免由多个布尔开关推断当前用途。
    enum class RuntimeTraceUsageProfile {
        Base = 0,
        PresetOnlineVelocity,
        EndpointRemoteTransition,
        EndpointRemoteRunning
    };

    enum class MotorSafetyRelativePositionSource {
        Invalid = 0,
        TraceCommandPersistentHome,
        TraceCommandSessionHome,
        TraceFeedbackSessionHome,
        EncoderFallback,
        PositionFallback
    };

    // Lite EtherCAT topology used by Runtime Trace.  The temporary seven-axis
    // layout omits hardware axis 7 (the former slave 1008), so the force
    // transmitter moves from slave 1009 to slave 1008.  Keep the standard
    // eight-axis layout available for restoring the complete machine later.
    enum class LiteRuntimeTraceTopology {
        TemporarySevenAxisSensorSlave1008 = 0,
        StandardEightAxisSensorSlave1009
    };

    struct ForceSensorReadResult {
        std::vector<double> values;
        int frameCount = 0;
        bool fromTrace = false;
    };

    struct ForceSensorTraceSample {
        std::vector<double> values;
        qint64 wallClockUs = 0;
        qint64 monotonicUs = 0;
        quint32 frameSequence = 0;
        bool frameSequenceValid = false;
    };

    // 末端遥控速度命令唯一允许使用的安全上下文。它是一次主循环Trace
    // 快照的值拷贝；专用命令入口只做纯数值校验，不再次读取Trace或位置。
    struct EndpointRemoteVelocitySafetyContext {
        std::vector<double> motorPosition;
        std::vector<double> motorSafetyRelativePosition;
        std::vector<MotorSafetyRelativePositionSource>
                motorSafetyRelativePositionSource;
        std::vector<quint16> motorStatusWord;
        std::vector<int> motorStateMachine;
        qint64 monotonicUs = 0;
        qint64 newestFrameAgeUs = -1;
        int fifoValidNum = 0;
        int fifoFreeNum = 0;
        int traceSamplePeriodUs = 0;
        quint64 logicalFrameSequence = 0;
        RuntimeTraceUsageProfile usageProfile = RuntimeTraceUsageProfile::Base;
        quint64 usageProfileGeneration = 0;
        quint64 configurationGeneration = 0;
        quint64 sessionToken = 0;
        bool fromTrace = false;
        bool frameSequenceValid = false;
        bool timingReliable = false;
        bool fifoCaughtUp = false;
        bool traceLost = false;
        bool statusFaultLatched = false;
        int statusFaultAxis = -1;
        quint16 statusFaultWord = 0;
        int statusFaultStateMachine = -1;
        quint64 statusFaultLogicalFrameSequence = 0;
    };

    // 末端遥控速度命令的显式结果状态。它用于把 HardwareThread 中的精确
    // 拒绝阶段传回 ControlWorker，避免由多个布尔量组合推断失败来源。
    enum class EndpointRemoteVelocityCommandOutcome {
        NotAttempted = 0,
        Succeeded,
        FreshFrameDeferred,
        SafetyContextRejected,
        CommandValidationRejected,
        SoftwareLimitRejected,
        SdkFailure,
        HardwareThreadDispatchFailed,
        InternalFailure
    };

    enum class EndpointRemoteCommandAdmissionProfile {
        StartFromConfirmedZero = 0,
        ActiveMotion
    };

    // 所有时间戳均来自同一个 steady_clock，仅用于现有命令的分段归因；
    // 不增加 Trace、位置、状态或任何板卡 SDK 读取。
    struct EndpointRemoteVelocityCommandReport {
        EndpointRemoteVelocityCommandOutcome outcome =
                EndpointRemoteVelocityCommandOutcome::NotAttempted;
        QString failureReason;
        qint64 traceValidationCompletedUs = 0;
        qint64 beforeSubmitUs = 0;
        qint64 hardwareThreadTaskStartUs = 0;
        qint64 sdkCallStartUs = 0;
        qint64 sdkCallEndUs = 0;
        qint64 hardwareThreadTaskEndUs = 0;
        qint64 compositeTaskStartUs = 0;
        qint64 planningStartedUs = 0;
        qint64 planningCompletedUs = 0;
        qint64 traceReadCompletedUs = 0;
        qint64 commandDeadlineUs = 0;
        qint64 remainingDeadlineBudgetUs = -1;
        qint64 entryFrameAgeUs = -1;
        double commandL2Norm = 0.0;
        double commandMaximumAbsVelocity = 0.0;
        int actuationProfile = 0;
        bool sdkCalled = false;
    };

    struct RuntimeTraceSnapshot {
        std::vector<double> motorPosition;
        std::vector<double> motorSafetyRelativePosition;
        std::vector<MotorSafetyRelativePositionSource>
                motorSafetyRelativePositionSource;
        // Online-velocity profile only. These values are decoded from the
        // exact Runtime Trace frame identified by monotonicUs.
        std::vector<quint16> motorStatusWord;
        std::vector<int> motorStateMachine;
        // 在线速度控制启用扩展 Trace 后，下面两组数据与位置/力矩来自同一帧。
        std::vector<double> motorCommandVelocity;
        std::vector<double> motorActualVelocity;
        std::vector<double> motorTorqueNm;
        std::vector<double> forceSensorValue;
        std::vector<qint64> forceSensorFrameMonotonicUs;
        std::vector<ForceSensorTraceSample> forceSensorTraceSamples;
        qint64 wallClockUs = 0;
        qint64 monotonicUs = 0;
        qint64 newestFrameAgeUs = -1;
        // Endpoint remote attribution only: split the blocking Trace call into
        // caller-to-HardwareThread queue wait, work executed on HardwareThread,
        // and accumulated dmc_trace_get_data time for this read.
        qint64 hardwareThreadQueueWaitUs = 0;
        qint64 hardwareThreadExecutionUs = 0;
        qint64 dataApiDurationUs = 0;
        qint64 totalReadCallUs = 0;
        int frameCount = 0;
        int fifoValidNum = 0;
        int fifoFreeNum = 0;
        int traceSamplePeriodUs = 0;
        quint32 frameSequence = 0;
        quint64 logicalFrameSequence = 0;
        bool fromTrace = false;
        bool frameSequenceValid = false;
        bool timingReliable = false;
        // true表示本次读取开始前已存在的帧已全部消费；API调用期间新产生
        // 的帧只计入newestFrameAgeUs，不视为历史积压。
        bool fifoCaughtUp = false;
        bool traceLost = false;
        RuntimeTraceUsageProfile usageProfile = RuntimeTraceUsageProfile::Base;
        quint64 usageProfileGeneration = 0;
        quint64 configurationGeneration = 0;
        quint64 endpointRemoteSessionToken = 0;
        EndpointRemoteVelocitySafetyContext endpointRemoteVelocitySafety;
    };

    struct EndpointRemoteTraceCommandResult {
        RuntimeTraceSnapshot traceSnapshot;
        EndpointRemoteVelocityCommandReport commandReport;
    };

    struct MotorTracePositionSample {
        double commandRelativePosition = 0.0;
        double feedbackRelativePosition = 0.0;
        qint64 wallClockUs = 0;
        qint64 monotonicUs = 0;
        qint64 commandRawPulse = 0;
        qint64 feedbackRawPulse = 0;
        bool commandRawPulseValid = false;
        bool feedbackRawPulseValid = false;
    };

    struct MotorTraceRecoveryAxisState {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        double axisEquiv = 0.0;
        qint64 savedCommandRawPulse = 0;
        qint64 savedFeedbackRawPulse = 0;
        qint64 currentCommandRawPulse = 0;
        qint64 currentFeedbackRawPulse = 0;
        double savedCommandUnitPosition = 0.0;
        double savedFeedbackUnitPosition = 0.0;
        double currentCommandUnitPosition = 0.0;
        double currentFeedbackUnitPosition = 0.0;
        double commandDeltaUnit = 0.0;
        double feedbackDeltaUnit = 0.0;
        bool savedCommandValid = false;
        bool savedFeedbackValid = false;
        bool currentCommandValid = false;
        bool currentFeedbackValid = false;
        bool commandMismatch = false;
        bool feedbackMismatch = false;
        bool restoreAvailable = false;
    };

    struct MotorTraceRecoveryState {
        bool fileLoaded = false;
        bool currentTraceRead = false;
        bool hasMismatch = false;
        QString filePath;
        QString message;
        qint64 savedWallClockUs = 0;
        qint64 savedMonotonicUs = 0;
        qint64 currentWallClockUs = 0;
        qint64 currentMonotonicUs = 0;
        std::vector<MotorTraceRecoveryAxisState> axes;
    };

    struct PvtPauseResult {
        bool success = false;
        bool beforeProgressValid = false;
        bool afterProgressValid = false;
        double beforeTrajectoryTime = 0.0;
        double afterTrajectoryTime = 0.0;
        int beforeIndex = -1;
        int afterIndex = -1;
    };

    // 默认构造硬件接口；需要后续设置控制周期和轴/传感器参数。
    HardwareInterface();
    // 构造并设置线程控制周期，供运动命令和等待逻辑使用。
    HardwareInterface(double _threadCtrlCycleMs);
    // 硬件接口不直接持有需手动释放的底层句柄，析构保持默认。
    ~HardwareInterface() override;

    // 配置电机通信、转换系数、从站号和力矩速度限制开关。
    void setMotorPara(std::vector<unsigned int> idVec, std::vector<int> comType, std::vector<QString> portInfo,
                      std::vector<double> posRaw2dataCof, std::vector<double> velRaw2dataCof,
                      std::vector<int> slaveIdVec = {},
                      std::vector<bool> torqueVelocityLimitEnabled = {});
    // 单独更新电机 EtherCAT 从站号。
    void setMotorSlaveIds(std::vector<int> slaveIdVec);
    // 设置每轴力矩模式是否需要写速度限制。
    void setMotorTorqueVelocityLimitEnabled(std::vector<bool> enabled);
    // 设置速度变化平滑时间。
    void setMotorChangeSpdTime(double s);
    // 设置电机零位/参考位置。
    void setMotorHome(std::vector<double> homeValue);
    // 设置软件安全零位：零位校准记录中的 Trace command 原始脉冲，不随重连重置。
    void setMotorSafetyHomeTraceCommandRawPulse(std::vector<qint64> rawPulse);
    void setMotorSafetyHomeEncoderUnit(std::vector<double> encoderUnit);
    // 设置雷赛轴当量，用于脉冲/工程单位转换。
    void setLeadshineAxisEquiv(std::vector<double> equivValue);
    void setLeadshineRatedMotorTorqueNm(double ratedTorqueNm);
    // 设置每轴软件位置和速度限位。
    void setMotorSoftwareLimits(std::vector<double> minPos,
                                std::vector<double> maxPos,
                                std::vector<double> maxVel = {});
    bool setMotorSoftwareLimitForAxis(int logicalIndex,
                                      double minPos,
                                      double maxPos,
                                      double maxVel);
    bool useStaticMotorHome = false;

    // 配置旧式传感器通信参数和原始值转换系数。
    void setSensorPara(std::vector<unsigned int> idVec, std::vector<int> comType, std::vector<QString> portInfo,
                       std::vector<double> raw2dataCof);
    // 配置 EtherCAT/串口力传感器地址、数据区和转换系数。
    void setSensorPara(std::vector<int> comType, std::vector<int> _sensorPort, std::vector<int> _sensorAdr, std::vector<int> _sensorDataAdr,
                       std::vector<int> _sensorDataLen, std::vector<double> raw2dataCof);
    // 设置力传感器零点。
    void setForceSensorHome(std::vector<double> homeValue);
    // 设置力传感器原始值是否按有符号数解析。
    void setForceSensorIsSigned(bool isSigned);
    bool useStaticSensorHome = false;

    // 初始化雷赛控制器和 EtherCAT 轴，并建立硬件连接。
    // Open only the Leadshine controller. This performs no axis writes, error
    // clearing, enabling, or home/reference updates.
    bool connectLSControllerOnly();
    bool connectLS();
    // 将当前轴当量写入雷赛驱动。
    bool applyLeadshineAxisEquiv();
    bool applyLeadshineAxisEquiv(int logicalIndex);
    // 设置力矩模式速度限制默认值。
    void setLeadshineTorqueVelocityLimitRpm(double velocityLimitRpm);
    // 对启用限制的轴写入力矩模式速度限制。
    bool applyLeadshineTorqueVelocityLimit(double velocityLimitRpm = 600.01);
    bool applyLeadshineTorqueVelocityLimit(int logicalIndex,
                                           double velocityLimitRpm = 600.01);
    bool configureLeadshineAxisForCommissioning(int logicalIndex);

    // 断开雷赛控制器连接并清理连接状态。
    bool disconnectLS();
    // 批量使能或失能所有电机轴。
    bool setAllMotorEnable(bool enable);
    // 使能或失能单个逻辑电机轴。
    bool setMotorEnable(int index, bool enable);
    // 对所有轴执行急停。
    bool emergencyStopAll();
    // 对给定逻辑轴执行急停和失能；用于不完整硬件调试会话。
    bool emergencyStopAxes(const std::vector<int>& logicalAxes);
    // 清除指定逻辑轴的雷赛轴错误码。
    bool clearLeadshineAxisErrorCode(int logicalIndex, bool emitErrors = true);
    bool clearLeadshineBusErrorCode(bool emitErrors = true);
    // 清除所有雷赛轴错误码。
    bool clearAllLeadshineAxisErrorCodes();
    // 查询当前雷赛硬件是否连接。
    bool isLSConnected() const;
    // 锁存 Lite 会话临时零点：优先选 Trace feedback，否则选 command；
    // 输出实际锁存的通道和原始脉冲，后续安全位置只允许同通道作差。
    bool setMotorHomeForAxis(int logicalIndex,
                             double homeValue,
                             bool* usesFeedback = nullptr,
                             qint64* rawPulse = nullptr);
    // 原子锁存多个 Lite 轴的 motorHome 和会话安全零点；安全原始位置来自同一帧新鲜 Trace。
    // 请求 commandRawPulse 输出时，每轴 command 也成为必要字段并从该帧一并返回。
    // 任一轴无效时不提交任何轴，避免整机基准只更新一部分。
    bool setMotorHomesForAxes(const std::vector<int>& logicalIndices,
                              const std::vector<double>& homeValues,
                              std::vector<bool>* usesFeedback = nullptr,
                              std::vector<qint64>* rawPulse = nullptr,
                              std::vector<qint64>* commandRawPulse = nullptr);
    // 查询指定轴运动是否完成。
    bool isMotorDone(int index) const;
    // 判断是否有已上传/活动的 PVT 轨迹。
    bool hasPvtTrajectory() const;
    // 判断 PVT 是否正在运行。
    bool isPvtMotionRunning() const;
    // 判断 PVT 是否处于暂停状态。
    bool isPvtMotionPausedState() const;
    // 判断 PVT 是否已经结束。
    bool isPvtMotionFinished() const;
    // 读取当前 PVT 时间和点索引进度。
    bool currentPvtProgress(double& currentTrajectoryTime, std::size_t& currentIndex) const;
    // 返回最近一次 PVT 启动命令成功时的本地单调时钟时间戳。
    qint64 activePvtStartTimeMonotonicUs() const;
    // 清除 PVT 轨迹缓存和运行标志。
    void clearPvtTrajectoryState();
    // 直接暂停当前 PVT。
    bool pausePvtMotion();
    // 直接恢复当前 PVT。
    bool resumePvtMotion();
    // 生成短过渡段平滑暂停 PVT，减少速度突变。
    bool smoothPausePvtMotion(double transitionTimeSec = 0.2);
    // 从暂停点生成恢复过渡段并继续 PVT。
    bool smoothResumePvtMotion(double transitionTimeSec = 0.5);
    // 返回最近一次平滑暂停的进度信息。
    PvtPauseResult lastPvtPauseResult() const;

    // 停止指定电机轴运动。
    bool motorStop(int index);
    // 在一次HardwareThread任务内连续停止指定轴，避免多轴退出被其他周期查询插入。
    bool motorStopAxes(const std::vector<int>& motorIndex);
    // 将指定电机轴回硬件零位。
    bool motorHome(int index);
    // 下发绝对位置运动命令。
    bool motorAbsPos(int index, double pos, double vel);
    // 下发相对位置运动命令。
    bool motorRelativePos(int index, double dist, double vel);
    // 下发速度模式命令。
    bool motorVel(int index, double vel);
    // 批量下发 JOG/连续速度命令，速度正负决定方向。
    bool motorVelBatch(const std::vector<int>& motorIndex,
                       const std::vector<double>& velocity,
                       double changeTimeSec = -1.0);
    // 跟随控制用的低延迟批量 JOG 速度更新；调用方提供当前位置用于限位校验，避免周期内直接读位置。
    bool motorVelBatchFast(const std::vector<int>& motorIndex,
                           const std::vector<double>& velocity,
                           double changeTimeSec,
                           std::vector<double> currentAbsolutePosition);
    // 末端遥控专用低延迟批量JOG入口。安全位置、驱动状态和Trace时序必须
    // 全部来自调用方同一帧上下文；函数内部禁止再次读取Trace或直接位置。
    bool motorVelBatchFastEndpointRemote(
            const std::vector<int>& motorIndex,
            const std::vector<double>& velocity,
            double changeTimeSec,
            const EndpointRemoteVelocitySafetyContext& safetyContext,
            qint64 maximumFeedbackAgeUs,
            quint64 sessionToken,
            EndpointRemoteVelocityCommandReport* commandReport);
    // 末端遥控运行期的唯一硬件任务：先在HardwareThread读取一次最新Trace，随后
    // 不退出该任务便完成同帧校验和速度下发，消除“规划后再次排队”造成的帧老化。
    EndpointRemoteTraceCommandResult
    readRuntimeTraceAndMotorVelBatchFastEndpointRemote(
            const std::vector<int>& motorIndex,
            const std::vector<double>& velocity,
            double changeTimeSec,
            qint64 maximumFeedbackAgeUs,
            quint64 sessionToken,
            quint64 minimumLogicalFrameSequenceExclusive,
            EndpointRemoteCommandAdmissionProfile admissionProfile,
            qint64 planningStartedUs,
            qint64 planningCompletedUs,
            int actuationProfile);
    // 清理低延迟 JOG 速度更新的运行状态缓存。
    void resetMotorVelBatchFastState(const std::vector<int>& motorIndex = {});
    // 速度模式运行到目标位置，并按 stopVel 停止。
    bool motorVelWithTargetPosAndStopVel(int index, double vel, double targetPos, double stopVel);
    // 启动指定轴力矩模式并下发原始电机坐标力矩，不做绳索方向换算。
    bool motorTorqueStart(int index, double torqueNm);
    // 在力矩模式下按原始电机坐标修改目标力矩，不做绳索方向换算。
    bool motorTorqueChange(int index, double torqueNm);
    // 读取指定轴当前绝对位置。
    double readMotorCurPos(int index);
    // 从运行期 Trace 读取指定轴绝对位置，避免轮询所有轴的 dmc_get_position_unit。
    bool readMotorTracePositionUnit(int index, double& position);
    // 读取指定轴相对零位位置。
    bool readMotorRelativeCurPos(int index, double& relativePosition);
    // 读取指定轴以软件安全零位为基准的位置。
    bool readMotorSafetyRelativeCurPos(int index, double& relativePosition);
    // 只读取指定逻辑轴的当前速度，避免 Lite 单轴调试轮询其他轴。
    bool readMotorCurrentSpeedUnit(int index, double& velocity);
    // 只返回指定逻辑轴的新鲜 Trace 实际转矩缓存；保留电机坐标正负号，不做绳索方向换算。
    bool readMotorTorqueNmTraceCached(int index, double& torqueNm);
    // 从 Trace 中读取命令/反馈相对位置。
    bool readMotorRelativeTracePositions(int index,
                                         double& commandRelativePosition,
                                         double& feedbackRelativePosition);
    // 读取单轴 Trace 相对位置样本队列。
    std::vector<MotorTracePositionSample> readMotorRelativeTracePositionSamples(int index);
    // 读取多轴 Trace 相对位置样本队列。
    std::vector<std::vector<MotorTracePositionSample>> readMotorRelativeTracePositionSamples(
            const std::vector<int>& motorIndex);
    bool readMotorTraceCommandRawPulseSnapshot(const std::vector<int>& motorIndex,
                                               std::vector<qint64>& rawPulse);
    // 只读校验指定轴是否已取得同一帧新鲜 command_raw。与普通快照不同，
    // 该接口同时检查 Trace 配置回读、FIFO 追平、时序和帧时间戳，且不改写 motorHome。
    bool readFreshMotorTraceCommandRawPulseSnapshot(
            const std::vector<int>& motorIndex,
            std::vector<qint64>& rawPulse,
            QString* failureReason = nullptr,
            bool* fifoNeedsDrain = nullptr);
    // 停止、清空并按当前运行配置重启 Runtime Trace。
    // 仅供整机尚未进入运行态时建立新的硬件快照会话，避免继承诊断读取形成的FIFO积压。
    bool restartRuntimeTraceForFreshSnapshot(QString* failureReason = nullptr);
    // 读取指定轴命令位置。
    bool readMotorCommandPosition(int index, double& commandPosition);
    // 配置指定轴位置 Trace。
    bool configureMotorPositionTrace(int index);
    // 读取指定轴命令位置和实际位置 Trace。
    bool readMotorTracePositions(int index, double& commandPosition, double& actualPosition);
    // 停止电机位置 Trace。
    void stopMotorPositionTrace();
    // 设置 Trace 位置窗口导出文件路径。
    void setMotorTracePositionWindowFilePath(const QString& filePath);
    // 返回 Trace 位置窗口导出文件路径。
    QString motorTracePositionWindowFilePath() const;
    // 将最近一段电机 Trace 位置窗口导出到文件。
    bool exportMotorTracePositionWindow(const QString& reason = QString(),
                                        QString* outputPath = nullptr,
                                        QString* errorMessage = nullptr);
    // 冻结 Trace 位置窗口记录，避免 PVT 后数据被覆盖。
    void freezeMotorTracePositionWindowRecording();
    // 根据保存文件和当前 Trace 读取结果刷新上电恢复状态。
    MotorTraceRecoveryState refreshMotorTraceRecoveryState(
            const QString& filePath = QString(),
            std::vector<int> logicalAxes = std::vector<int>(),
            bool requireSavedFeedbackChannels = true);
    // 返回缓存的上电恢复状态。
    MotorTraceRecoveryState cachedMotorTraceRecoveryState() const;
    // 读取雷赛模式字，判断轴当前控制模式。
    bool readLeadshineModeOfOperation(int index, qint8& modeOfOperation);
    // 读取雷赛跟随误差原始值。
    bool readLeadshineFollowingErrorRaw(int index, int& followingErrorRaw);
    // 读取并对比位置单位、编码器单位、Trace 命令/反馈单位的诊断信息。
    bool readLeadshinePositionUnitTraceDiagnostic(int index,
                                                  double& directUnitPosition,
                                                  double& encoderUnitPosition,
                                                  double& traceCommandUnitPosition,
                                                  double& traceActualUnitPosition,
                                                  double& axisEquiv,
                                                  double& commandUnitDiff,
                                                  double& commandPulseDiffEstimate,
                                                  double& actualZeroOffsetUnit,
                                                  double& actualZeroOffsetPulse,
                                                  double& encoderVsPositionUnitDiff,
                                                  double& encoderVsPositionPulseDiff,
                                                  double& traceActualEncoderUnitDiff,
                                                  double& traceActualEncoderPulseDiff,
                                                  QString* errorMessage = nullptr);

    // 解析 CAN 端口描述字符串为设备索引和 CAN 端口索引。
    void canPortInfoProcessor(const QString portInfo, int &deviceIndex, int &canPortIndex);
    // 批量解析 CAN 端口描述字符串。
    void canPortInfoProcessor(const std::vector<QString> portInfo, std::vector<int> &deviceIndex, std::vector<int> &canPortIndex);

    // 刷新所有电机使能状态缓存。
    void checkAllMotorState();
    // 刷新所有电机位置缓存。
    void checkAllMotorPos();
    // 刷新所有电机速度缓存。
    void checkAllMotorVel();
    // 返回所有电机使能状态缓存。
    std::vector<bool> getAllMotorState();
    // 查询单轴使能状态。
    bool isMotorEnabled(int index);
    std::vector<bool> motorCurState;
    // 返回所有电机绝对位置。
    std::vector<double> getAllMotorPos();
    // 返回所有电机工程单位位置。
    std::vector<double> getAllMotorPosUnit();
    // 返回所有电机编码器工程单位位置。
    std::vector<double> getAllMotorEncoderPosUnit();
    std::vector<double> motorCurPos;
    std::vector<double> motorCommandPos;
    // 返回所有电机速度。
    std::vector<double> getAllMotorVel();
    std::vector<double> motorCurVel;
    // 返回所有电机实际力矩，单位 Nm；正负号保持雷赛电机坐标定义。
    std::vector<double> getAllMotorTorqueNm();
    std::vector<double> getAllMotorTorqueNmTraceCached();
    // 控制线程专用：只读取一次 Trace，并返回最新帧刷新后的缓存快照。
    RuntimeTraceSnapshot readRuntimeTraceLatestSnapshot();
    // 返回当前电机零位数组。
    std::vector<double> getAllMotorHome();
    // 返回软件安全零位 Trace command 原始脉冲。
    std::vector<qint64> getAllMotorSafetyHomeTraceCommandRawPulse();
    // 返回软件安全零位换算后的工程单位。
    std::vector<double> getAllMotorSafetyHomeUnit();
    // 返回以软件安全零位为基准的编码器相对位置。
    std::vector<double> getAllMotorSafetyRelativePosUnit();

    // 读取一次力传感器快照，优先使用 Trace 缓存。
    std::vector<double> readForceSensorDataSnapshot();
    // 读取指定通道数的力传感器缓存值。
    std::vector<double> readForceSensorDataCached(int maxChannelsToRead);
    // 读取力传感器缓存并返回帧数、来源等附加信息。
    ForceSensorReadResult readForceSensorDataCachedResult(int maxChannelsToRead);
    // 读取力传感器 Trace 样本队列。
    std::vector<ForceSensorTraceSample> readForceSensorDataTraceSamples();
    void setRuntimeTraceConfigType(RuntimeTraceConfigType type);
    RuntimeTraceConfigType runtimeTraceConfigType() const;
    bool setRuntimeTraceUsageProfile(RuntimeTraceUsageProfile profile,
                                     quint64 endpointRemoteSessionToken = 0);
    RuntimeTraceUsageProfile runtimeTraceUsageProfile() const;
    void setLiteRuntimeTraceTopology(LiteRuntimeTraceTopology topology);
    LiteRuntimeTraceTopology liteRuntimeTraceTopology() const;
    // Restrict runtime Trace feedback to one motor/sensor during Lite
    // commissioning so absent axes are never polled as a side effect.
    void setRuntimeTraceCommissioningSelection(int logicalAxis, int sensorIndex);
    void clearRuntimeTraceCommissioningSelection();
    // 开启或关闭力传感器 Trace 读取。
    void setForceSensorTraceReadEnabled(bool enabled);
    // 兼容入口：现在等同于切换完整的在线速度 Trace 配置。
    bool setRuntimeTraceVelocitySignalsEnabled(bool enabled);
    // 原子切换在线速度专用 Trace：保留位置、速度和反馈力矩，并按性能开关
    // 排除力传感器对象；退出模式时一次重配恢复基础 Trace。
    bool setOnlineVelocityRuntimeTraceProfileEnabled(bool enabled);
    // 设置力传感器 Trace 采样周期。
    void setForceSensorTraceSamplePeriodUs(int periodUs);
    // 运行 PDO Trace 探针，用于检查力传感器对象字典和数据包。
    PdoTraceProbeResult runPdoTraceForceSensorProbe(const PdoTraceProbeConfig& config);
    // 运行通用 Trace 探针，用于调试控制卡 Trace 配置。
    TraceProbeResult runTraceForceSensorProbe(const TraceProbeConfig& config);
    // 返回通信和电机命令计时诊断快照。
    DiagnosticsSnapshot diagnosticsSnapshot() const;
    FieldbusConsumeTimeSnapshot fieldbusConsumeTimeSnapshot() const;
    // Lightweight endpoint-remote attribution. It never performs an extra
    // hardware query; it only times calls that the safety path already makes.
    void setMotorEnableQueryTimingEnabled(bool enabled);
    MotorEnableQueryTimingSnapshot motorEnableQueryTimingSnapshot() const;
    // 查询通信事件间隔历史。
    QVector<DiagnosticRawSample> communicationTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 查询电机命令事件间隔历史。
    QVector<DiagnosticRawSample> motorCommandTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 查询电机位置原始读取历史。
    QVector<MotorPositionRawSample> motorPositionRawHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 查询电机编码器原始读取历史。
    QVector<MotorPositionRawSample> motorEncoderRawHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    // 查询 Runtime Trace feedback 原始编码器脉冲历史。
    QVector<MotorTraceFeedbackRawSample> motorTraceFeedbackRawHistory(qint64 startWallClockMs,
                                                                      qint64 endWallClockMs) const;
    // 查询 Runtime Trace 每次 dmc_trace_get_data 批量读取摘要。
    QVector<RuntimeTraceFetchTimingSample> runtimeTraceFetchTimingHistory(qint64 startWallClockMs,
                                                                          qint64 endWallClockMs) const;
    QVector<PvtTableUploadTimingSample> pvtTableUploadTimingHistory(qint64 startWallClockMs,
                                                                     qint64 endWallClockMs) const;
    // 会话记录开启时保留完整原始历史；默认只保留短窗口诊断缓存。
    void setDiagnosticRawHistoryFullRecordingEnabled(bool enabled);
    // 会话记录专用：按固定目标周期读取绳索电机编码器 unit。
    void startSessionEncoderUnitSampling(int intervalUs = 500,
                                         std::vector<int> logicalAxes = {});
    void stopSessionEncoderUnitSampling();
    // 返回控制器、电机轴和传感器连接诊断。
    ConnectionDiagnostics connectionDiagnostics() const;
    // 直接查询 EtherCAT 主站/控制卡连接诊断，供安全监控独立判断总线是否在线。
    ConnectionItemDiagnostics controllerDiagnostics() const;
    // 查询单个逻辑轴的连接诊断。
    ConnectionItemDiagnostics motorAxisDiagnostics(int logicalIndex) const;
    // 查询单个力传感器 Trace 数据是否新鲜，不轮询其他轴或传感器。
    ConnectionItemDiagnostics forceSensorDiagnostics(int sensorIndex) const;
    // 清零诊断统计和历史缓存。
    void resetDiagnostics();
    // 手动读取关节传感器数据，保留给旧调试流程。
    void checkJointSensorDataManual();
    std::vector<double> rodCurForce;
    std::vector<double> jointCurTheta;
    std::vector<int> jointCurThetaID;

    double threadCtrlCycleMs = 0.0;
    // 下发并执行多轴位置/速度 PVT 表。
    void motorPosTraj(std::vector<int> motorIndex, std::vector<std::vector<double>> motorPosTraj,
                      std::vector<std::vector<double>> motorVel, std::vector<double> motorVelMax,
                      std::vector<double> timeStamp);

private:
    template<typename F>
    // 若当前不在硬件线程，则用 BlockingQueuedConnection 同步执行硬件访问。
    auto runOnHardwareThread(F&& f) -> decltype(f()) {
        using R = decltype(f());
        QThread* targetThread = thread();
        if (!targetThread || !targetThread->isRunning() || QThread::currentThread() == targetThread) {
            return f();
        }

        if constexpr (std::is_void_v<R>) {
            QMetaObject::invokeMethod(this, [&]() { f(); }, Qt::BlockingQueuedConnection);
        } else {
            R result{};
            QMetaObject::invokeMethod(this, [&]() { result = f(); }, Qt::BlockingQueuedConnection);
            return result;
        }
    }

    template<typename F>
    // const 版本线程切换工具，用于只读硬件查询。
    auto runOnHardwareThread(F&& f) const -> decltype(f()) {
        using R = decltype(f());
        QThread* targetThread = thread();
        if (!targetThread || !targetThread->isRunning() || QThread::currentThread() == targetThread) {
            return f();
        }

        if constexpr (std::is_void_v<R>) {
            QMetaObject::invokeMethod(const_cast<HardwareInterface*>(this), [&]() { f(); }, Qt::BlockingQueuedConnection);
        } else {
            R result{};
            QMetaObject::invokeMethod(const_cast<HardwareInterface*>(this), [&]() { result = f(); }, Qt::BlockingQueuedConnection);
            return result;
        }
    }

    // 连接标志会被 HardwareThread 写入、GUI/安全线程频繁读取。使用原子值
    // 可避免为了读取一个布尔量而阻塞等待正在执行雷赛 API 的硬件线程。
    std::atomic_bool isConnectLS{false};
    bool forceSensorIsSigned = true;
    mutable QMutex diagnosticsMutex;
    DiagnosticsSnapshot diagnostics;
    qint64 lastCommunicationEventUs = 0;
    qint64 lastMotorCommandEventUs = 0;
    QVector<DiagnosticRawSample> communicationRawHistory;
    QVector<DiagnosticRawSample> motorCommandRawHistory;
    QVector<MotorPositionRawSample> motorPositionRawSamples;
    QVector<MotorPositionRawSample> motorEncoderRawSamples;
    QVector<MotorTraceFeedbackRawSample> motorTraceFeedbackRawSamples;
    QVector<RuntimeTraceFetchTimingSample> runtimeTraceFetchTimingSamples;
    QVector<PvtTableUploadTimingSample> pvtTableUploadTimingSamples;
    bool diagnosticRawHistoryFullRecordingEnabled = false;
    qint64 lastCommunicationHistoryTrimMs = 0;
    qint64 lastMotorCommandHistoryTrimMs = 0;
    qint64 lastMotorPositionReadUs = 0;
    qint64 lastMotorPositionRawHistoryTrimMs = 0;
    qint64 lastMotorEncoderReadUs = 0;
    qint64 lastMotorEncoderRawHistoryTrimMs = 0;
    qint64 lastMotorTraceFeedbackRawUs = 0;
    qint64 lastMotorTraceFeedbackRawHistoryTrimMs = 0;
    qint64 lastRuntimeTraceFetchTimingUs = 0;
    qint64 lastRuntimeTraceFetchTimingHistoryTrimMs = 0;
    qint64 lastPvtTableUploadTimingHistoryTrimMs = 0;
    qint64 lastCommunicationRawHistoryAppendUs = 0;
    qint64 lastMotorCommandRawHistoryAppendUs = 0;
    qint64 lastMotorPositionRawHistoryAppendUs = 0;
    qint64 lastMotorEncoderRawHistoryAppendUs = 0;
    qint64 lastMotorTraceFeedbackRawHistoryAppendUs = 0;
    qint64 lastRuntimeTraceFetchTimingHistoryAppendUs = 0;
    qint64 lastPvtTableUploadTimingHistoryAppendUs = 0;
    std::atomic_bool motorEnableQueryTimingEnabled{false};
    std::atomic<quint64> motorEnableQueryCount{0};
    std::atomic<qint64> motorEnableQueryTotalQueueWaitUs{0};
    std::atomic<qint64> motorEnableQueryLatestQueueWaitUs{0};
    std::atomic<qint64> motorEnableQueryMaximumQueueWaitUs{0};
    std::atomic<qint64> motorEnableQueryTotalApiDurationUs{0};
    std::atomic<qint64> motorEnableQueryLatestApiDurationUs{0};
    std::atomic<qint64> motorEnableQueryMaximumApiDurationUs{0};
    std::atomic<qint64> motorEnableQueryTotalCallDurationUs{0};
    std::atomic<qint64> motorEnableQueryLatestCallDurationUs{0};
    std::atomic<qint64> motorEnableQueryMaximumCallDurationUs{0};
    mutable QMutex sessionEncoderUnitSamplerMutex;
    std::atomic_bool sessionEncoderUnitSamplingActive{false};
    std::thread sessionEncoderUnitSamplerThread;
    mutable QMutex connectionDiagnosticsMutex;
    mutable ConnectionDiagnostics cachedConnectionDiagnostics;
    mutable bool connectionDiagnosticsRefreshPending = false;

    std::vector<double> motorHomePos;
    std::vector<qint64> motorSafetyHomeTraceCommandRawPulse;
    std::vector<double> motorSafetyHomeEncoderUnit;
    // Lite 单轴会话的 Trace 临时零点。零点锁存后必须始终使用同一
    // Trace 通道作差；valid 独立保存，原始脉冲恰好为 0 仍是合法零点。
    std::vector<qint64> motorSessionSafetyHomeTraceRawPulse;
    std::vector<bool> motorSessionSafetyHomeTraceValid;
    std::vector<bool> motorSessionSafetyHomeTraceUsesFeedback;
    std::vector<double> motorSoftwareMinPos;
    std::vector<double> motorSoftwareMaxPos;
    std::vector<double> motorSoftwareMaxVel;

    std::vector<unsigned int> motorIdVec, sensorIdVec;
    std::vector<int> motorSlaveIdVec;
    std::vector<bool> motorTorqueVelocityLimitEnabled;
    std::vector<int> motorComType, sensorComType, sensorPort, sensorAdr, sensorDataAdr, sensorDataLen;
    std::vector<QString> motorPortInfo, sensorPortInfo;
    std::vector<double> motorPosRaw2dataCof, motorVelRaw2dataCof, motorEquivVec, sensorRaw2DataCof, sensorHomeValue;
    std::vector<double> forceSensorCachedValue;
    std::vector<double> motorTraceActualPos;
    std::vector<double> motorTraceCommandVelocity;
    std::vector<double> motorTraceActualVelocity;
    std::vector<bool> motorTraceCommandVelocityValid;
    std::vector<bool> motorTraceActualVelocityValid;
    std::vector<quint16> motorTraceStatusWord;
    std::vector<bool> motorTraceStatusWordValid;
    std::vector<qint64> motorTraceStatusWordMonotonicUs;
    std::vector<double> motorTraceTorqueNm;
    std::vector<bool> motorTraceTorqueValid;
    std::vector<qint64> motorTraceTorqueMonotonicUs;
    std::vector<bool> forceSensorCacheValid;
    std::vector<qint64> forceSensorTraceValueMonotonicUs;
    int nextForceSensorPollIndex = 0;
    RuntimeTraceConfigType activeRuntimeTraceConfigType = RuntimeTraceConfigType::G3;
    RuntimeTraceUsageProfile activeRuntimeTraceUsageProfile =
            RuntimeTraceUsageProfile::Base;
    quint64 runtimeTraceUsageProfileGeneration = 1;
    quint64 runtimeTraceConfigurationGeneration = 0;
    quint64 runtimeTraceEndpointRemoteSessionToken = 0;
    LiteRuntimeTraceTopology activeLiteRuntimeTraceTopology =
            LiteRuntimeTraceTopology::StandardEightAxisSensorSlave1009;
    int runtimeTraceCommissioningAxis = -1;
    int runtimeTraceCommissioningSensor = -1;
    // Base profile的力传感器对象偏好；在线/遥控profile由枚举语义决定。
    bool baseRuntimeTraceForceSensorEnabled = true;
    int forceSensorTraceSamplePeriodUs = 500;
    bool runtimeTraceConfigured = false;
    bool runtimeTraceUnavailable = false;
    bool runtimeTraceEverRead = false;
    bool runtimeTraceConfigReadbackValid = false;
    bool runtimeTraceTimingReliable = false;
    bool runtimeTraceFifoCaughtUp = false;
    bool runtimeTraceLost = false;
    int runtimeTraceConsecutiveFailures = 0;
    int runtimeTraceObjectTotalBytes = 0;
    int runtimeTraceObjectTotalNum = 0;
    int runtimeTraceEthercatBusCycleUs = 500;
    int runtimeTraceConfiguredCycle = 1;
    int runtimeTraceSamplePeriodUs = 500;
    int runtimeTraceLastFifoValidNum = 0;
    int runtimeTraceLastFifoFreeNum = 0;
    qint64 runtimeTraceLastRetryUs = 0;
    qint64 runtimeTraceLastFrameWallClockUs = 0;
    qint64 runtimeTraceLastFrameMonotonicUs = 0;
    qint64 runtimeTraceNewestFrameAgeUs = -1;
    qint64 runtimeTraceLastDataApiDurationUs = 0;
    bool runtimeTraceLastFrameSequenceValid = false;
    quint32 runtimeTraceLastFrameSequence = 0;
    bool runtimeTraceSequenceInitialized = false;
    quint32 runtimeTraceLastRawSequence = 0;
    quint64 runtimeTraceLastLogicalSequence = 0;
    bool runtimeTraceHostTimeAnchorValid = false;
    quint64 runtimeTraceHostTimeAnchorSequence = 0;
    qint64 runtimeTraceHostTimeAnchorWallClockUs = 0;
    qint64 runtimeTraceHostTimeAnchorMonotonicUs = 0;
    qint64 runtimeTracePositionRejectLastWarnUs = 0;
    qint64 runtimeTraceLayoutRejectLastWarnUs = 0;
    mutable qint64 forceSensorDiagnosticsLastTraceReadUs = 0;
    struct ForceSensorTraceObject {
        int sensorIndex = -1;
        short dataType = 19;
        int dataIndex = 0x6000;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 0;
        int valueBytes = 2;
    };
    std::vector<ForceSensorTraceObject> forceSensorTraceObjects;

    int motorPositionTraceSamplePeriodUs = 500;

    struct MotorCommandPositionTraceObject {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short dataType = 5;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 4;
        int valueBytes = 4;
    };
    std::vector<MotorCommandPositionTraceObject> motorCommandPositionTraceObjects;

    struct MotorPositionTraceObject {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short dataType = 6;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 4;
        int valueBytes = 4;
    };
    std::vector<MotorPositionTraceObject> motorPositionTraceObjects;

    struct MotorVelocityTraceObject {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short dataType = 4;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 4;
        int valueBytes = 4;
    };
    std::vector<MotorVelocityTraceObject> motorCommandVelocityTraceObjects;
    std::vector<MotorVelocityTraceObject> motorActualVelocityTraceObjects;

    struct MotorTorqueTraceObject {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short dataType = 8;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 4;
        int valueBytes = 4;
    };
    std::vector<MotorTorqueTraceObject> motorTorqueTraceObjects;

    struct MotorStatusWordTraceObject {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short dataType = 19;
        int dataIndex = 0x6041;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 2;
        int valueBytes = 2;
    };
    std::vector<MotorStatusWordTraceObject> motorStatusWordTraceObjects;

    enum class RuntimeTraceObjectKind {
        MotorCommandPosition = 0,
        MotorPosition,
        MotorCommandVelocity,
        MotorActualVelocity,
        MotorStatusWord,
        MotorTorque,
        ForceSensor
    };

    struct RuntimeTraceObject {
        RuntimeTraceObjectKind kind = RuntimeTraceObjectKind::MotorCommandPosition;
        int objectIndex = -1;
        int valueBytes = 4;
    };
    std::vector<RuntimeTraceObject> runtimeTraceObjects;
    bool endpointRemoteTraceStatusFaultLatched = false;
    int endpointRemoteTraceStatusFaultAxis = -1;
    quint16 endpointRemoteTraceStatusFaultWord = 0;
    int endpointRemoteTraceStatusFaultStateMachine = -1;
    quint64 endpointRemoteTraceStatusFaultLogicalFrameSequence = 0;

    struct PvtTraceStartDelayState {
        bool active = false;
        qint64 pvtUploadMonotonicUs = 0;
        int pointCount = 0;
        int axisCount = 0;
        int ethercatBusCycleUs = 500;
        std::vector<int> motorIndex;
        std::vector<qint64> commandBaselineRawPulse;
        std::vector<qint64> feedbackBaselineRawPulse;
        std::vector<bool> commandBaselineValid;
        std::vector<bool> feedbackBaselineValid;
        bool commandStartFound = false;
        quint32 commandStartFrameSequence = 0;
        int commandStartAxis = -1;
    };
    PvtTraceStartDelayState pvtTraceStartDelayState;
    std::vector<std::deque<MotorTracePositionSample>> motorTracePositionSampleQueues;
    std::deque<ForceSensorTraceSample> forceSensorTraceSampleQueue;
    std::vector<double> motorCommandTraceOffsetUnit;
    std::vector<double> motorActualTraceOffsetUnit;
    std::vector<bool> motorCommandTraceOffsetValid;
    std::vector<bool> motorActualTraceOffsetValid;

    struct MotorTracePositionWindowFrame {
        qint64 wallClockUs = 0;
        qint64 monotonicUs = 0;
        std::vector<double> commandUnitPosition;
        std::vector<double> feedbackUnitPosition;
        std::vector<double> commandRelativePosition;
        std::vector<double> feedbackRelativePosition;
        std::vector<qint64> commandRawPulse;
        std::vector<qint64> feedbackRawPulse;
        std::vector<bool> commandValid;
        std::vector<bool> feedbackValid;
    };
    std::deque<MotorTracePositionWindowFrame> motorTracePositionWindow;
    MotorTracePositionWindowFrame latestMotorTracePositionFrame;
    QString motorTracePositionWindowOutputPath;
    MotorTraceRecoveryState cachedMotorTraceRecoveryStateData;
    bool motorTracePositionWindowRecordingEnabled = false;
    bool motorTracePositionWindowFrozenAfterPvt = false;
    bool latestMotorTracePositionFrameValid = false;

    bool hasActivePvtTrajectory = false;
    bool isPvtMotionPaused = false;
    PvtPauseResult lastPvtPauseResultData;
    std::size_t pausedPvtResumeIndex = 0;
    double pausedPvtResumeTime = 0.0;
    std::vector<int> activePvtMotorIndex;
    std::vector<std::vector<double>> activePvtMotorPosTraj;
    std::vector<std::vector<double>> activePvtMotorVelTraj;
    std::vector<double> activePvtMotorVelMax;
    std::vector<double> activePvtTimeStamp;
    qint64 activePvtStartMonotonicUs = 0;

    double velChangeSpd = 0.01;
    double leadshineRatedMotorTorqueNm = 45.0;
    double leadshineTorqueVelocityLimitRpm = 600.01;
    std::vector<bool> motorJogVelocityFastActive;

    // 简单阻塞延时，供底层 API 状态切换等待使用。
    void delay(unsigned int msec);
    // 对当前硬件不支持的功能统一提示。
    void unsupportedFeature(const QString& featureName);
    // 写入单轴使能状态并可选择输出错误提示。
    bool applyLeadshineAxisEnableState(int logicalIndex, bool enable, bool emitErrors = true);
    // 从驱动刷新单轴使能状态缓存。
    bool refreshLeadshineMotorEnableState(int logicalIndex,
                                          qint64* apiDurationUs = nullptr);
    // 构建并启动底层 PVT 表，可选择更新活动轨迹缓存。
    bool startPvtTable(const std::vector<int>& motorIndex,
                       const std::vector<std::vector<double>>& motorPosTraj,
                       const std::vector<std::vector<double>>& motorVelTraj,
                       const std::vector<double>& motorVelMax,
                       const std::vector<double>& timeStamp,
                       const std::vector<double>& beginVel,
                       const std::vector<double>& endVel,
                       bool updateActiveTrajectory,
                       const QString& successMessage);
    // 直接读取当前 PVT 时间和索引进度。
    bool getPvtCurrentProgress(double& currentTrajectoryTime, std::size_t& currentIndex) const;
    // 批量读取指定轴位置。
    std::vector<double> readMotorPositions(const std::vector<int>& motorIndex) const;
    // 批量读取指定轴速度。
    std::vector<double> readMotorSpeeds(const std::vector<int>& motorIndex) const;
    // 等待活动 PVT 轴完成运动。
    bool waitPvtAxesDone(int timeoutMs);
    // 停止活动 PVT 轴并等待停止完成。
    bool stopActivePvtAxesAndWait(int timeoutMs);
    // 清理运行期混合 Trace 状态。
    void resetRuntimeTraceState();
    // 清理力传感器 Trace 状态。
    void resetForceSensorTraceState();
    // 清理电机位置 Trace 状态。
    void resetMotorPositionTraceState();
    // 配置运行期 Trace 同时采集电机命令、反馈和力传感器。
    bool configureRuntimeTraceRead();
    // 读取一次运行期 Trace 并刷新各缓存队列。
    int readRuntimeTraceCached(bool latestOnly = false);
    // 完整解析一帧 Runtime Trace。仅末端遥控Running profile会在FIFO全量
    // 排空、逐帧连续性和0x6041扫描完成后只调用一次；其他profile逐帧调用。
    bool decodeRuntimeTraceFrame(const unsigned char* frameData,
                                 int frameBytes,
                                 int objectValueStartOffset,
                                 qint64 frameWallClockUs,
                                 qint64 frameMonotonicUs,
                                 quint32 frameSequence,
                                 bool frameSequenceValid,
                                 bool appendHistorySamples,
                                 bool recordRawDiagnostic);
    void observeEndpointRemoteRuntimeTraceStatusWords(
            const unsigned char* frameData,
            int frameBytes,
            int objectValueStartOffset,
            quint64 logicalFrameSequence);
    void resetEndpointRemoteRuntimeTraceStatusFault();
    bool runtimeTraceUsageProfileIncludesVelocitySignals(
            RuntimeTraceUsageProfile profile) const;
    bool runtimeTraceUsageProfileIncludesForceSensors(
            RuntimeTraceUsageProfile profile) const;
    void armPvtTraceStartDelayMeasurement(const std::vector<int>& motorIndex,
                                          int pointCount,
                                          qint64 pvtUploadMonotonicUs);
    void observePvtTraceStartDelayFrame(
            quint32 frameSequence,
            const std::vector<qint64>& commandRawPulse,
            const std::vector<bool>& commandValid,
            const std::vector<qint64>& feedbackRawPulse,
            const std::vector<bool>& feedbackValid);
    // 配置电机位置 Trace。
    bool configureMotorPositionTraceRead();
    // 从电机位置 Trace 缓存读取指定轴位置。
    bool readMotorPositionTraceCached(int logicalIndex, double& position);
    // 从 Trace 缓存批量读取指定轴位置。
    std::vector<double> readMotorPositionsTraceCached(const std::vector<int>& motorIndex);
    // 返回当前电机位置缓存值，必要时供 Trace 不可用时兜底。
    std::vector<double> currentMotorPositionCachedValues(const std::vector<int>& motorIndex) const;
    std::vector<double> readMotorTorqueTraceCachedDirect();
    std::vector<double> currentMotorTorqueTraceCachedValues() const;
    short resolveLeadshineTraceSlaveId(int logicalIndex, int hardwareAxis) const;
    // 将原始 Trace 命令位置脉冲转换并写入缓存。
    void applyMotorCommandPositionTraceRawValue(int logicalIndex, long rawValue);
    // 将原始 Trace 反馈位置脉冲转换并写入缓存。
    void applyMotorPositionTraceRawValue(int logicalIndex, long rawValue);
    void applyMotorCommandVelocityTraceRawValue(int logicalIndex, long rawValue);
    void applyMotorActualVelocityTraceRawValue(int logicalIndex, long rawValue);
    void applyMotorTorqueTraceRawValue(int logicalIndex, long rawValue, qint64 traceMonotonicUs = 0);
    // 开始记录用于上电恢复的电机 Trace 位置窗口。
    void beginMotorTracePositionWindowRecording();
    // 将一帧命令/反馈位置写入 Trace 位置窗口。
    void appendMotorTracePositionWindowFrame(const MotorTracePositionWindowFrame& frame);
    // 因指定事件导出当前 Trace 位置窗口。
    void exportMotorTracePositionWindowForEvent(const QString& reason);
    // 裁剪 Trace 位置窗口，只保留最近一段数据。
    void trimMotorTracePositionWindow(qint64 latestMonotonicUs);
    // 直接判断活动 PVT 轴是否完成。
    bool activePvtAxesDoneDirect() const;
    // 从上次导出的 Trace 文件中读取恢复参考帧。
    bool loadMotorTraceRecoveryFrameFromFile(const QString& filePath,
                                             MotorTracePositionWindowFrame& frame,
                                             QString* errorMessage = nullptr) const;
    // 读取当前指定轴的 Trace 恢复帧，用于和保存帧比较。
    bool readCurrentMotorTraceRecoveryFrame(const std::vector<int>& logicalAxes,
                                            const MotorTracePositionWindowFrame& savedFrame,
                                            bool requireSavedFeedbackChannels,
                                            MotorTracePositionWindowFrame& frame,
                                            QString* errorMessage = nullptr);
    // 配置力传感器 Trace 读取对象。
    bool configureForceSensorTraceRead();
    // 直接从底层读取力传感器缓存值。
    std::vector<double> readForceSensorDataCachedDirect(int maxChannelsToRead);
    // 从 Trace 缓存读取力传感器值。
    std::vector<double> readForceSensorDataTraceCached();
    // 直接读取力传感器缓存并附加帧信息。
    ForceSensorReadResult readForceSensorDataCachedDirectResult(int maxChannelsToRead);
    // 读取力传感器 Trace 缓存并附加帧信息。
    ForceSensorReadResult readForceSensorDataTraceCachedResult();
    // 返回当前力传感器缓存值。
    std::vector<double> currentForceSensorCachedValues() const;
    // 读取力传感器额外 TXPDO 原始值。
    short readForceSensorTxpdoExtra(int sensorIndex, int* rawValue) const;
    // 将力传感器原始值转换为工程单位并写入缓存。
    void applyForceSensorRawValue(int sensorIndex, long rawValue, qint64 traceMonotonicUs = 0);
    // 记录通信事件时间，用于诊断采样周期。
    void recordCommunicationEvent(bool motorCommandEvent = false,
                                  const QString& apiEvent = QString());
    // 记录一次电机位置原始采样。
    void recordMotorPositionRawSample(const std::vector<double>& positions, const QString& source);
    // 记录一次电机编码器原始采样。
    void recordMotorEncoderRawSample(const std::vector<double>& positions,
                                     const QString& source,
                                     qint64 durationUs = 0,
                                     qint64 sampleWallClockUs = 0,
                                     qint64 sampleMonotonicUs = 0);
    // 记录一帧 Runtime Trace feedback 原始编码器脉冲。
    void recordMotorTraceFeedbackRawSample(qint64 frameWallClockUs,
                                           qint64 frameMonotonicUs,
                                           quint32 frameSequence,
                                           bool frameSequenceValid,
                                           const std::vector<qint64>& feedbackRawPulse,
                                           const std::vector<bool>& feedbackValid);
    // 记录一次 Runtime Trace 批量读取耗时和帧序号范围。
    void recordRuntimeTraceFetchTimingSample(qint64 readEndWallClockUs,
                                             qint64 readEndMonotonicUs,
                                             qint64 apiDurationUs,
                                             int actualReadLength,
                                             int frameBytes,
                                             int frameCount,
                                             int requestedFrameCount,
                                             int fifoValidBefore,
                                             int fifoValidAfter,
                                             int fifoFreeAfter,
                                             int estimatedProducedFrameCount,
                                             int traceSamplePeriodUs,
                                             qint64 newestFrameAgeUs,
                                             bool latestOnly,
                                             bool fifoCaughtUp,
                                             bool timingReliable,
                                             bool traceLost,
                                             bool frameSequenceValid,
                                             const std::vector<quint32>& frameSequences);
    // 读取并记录指定轴当前编码器工程单位值。
    void recordCurrentMotorEncoderUnitsForAxes(const std::vector<int>& logicalAxes);
    std::vector<int> defaultSessionEncoderUnitSamplingAxes() const;
    std::vector<int> sanitizeSessionEncoderUnitSamplingAxes(const std::vector<int>& logicalAxes) const;
    void sessionEncoderUnitSamplingLoop(std::vector<int> logicalAxes, int intervalUs);
    // 清空电机 Trace 相对位置零偏。
    void resetMotorTracePositionOffsets();
    // 保证 Trace 零偏缓存数组尺寸正确。
    void ensureMotorTracePositionOffsetStorage();
    // 确保指定轴 Trace 零偏已建立。
    bool ensureMotorTracePositionOffsets(int logicalIndex);
    // 用 Trace 单位位置和零偏计算相对位置。
    double traceAlignedRelativePosition(int logicalIndex,
                                        double traceUnitPosition,
                                        const std::vector<double>& traceOffsetUnit,
                                        const std::vector<bool>& traceOffsetValid) const;
    // 将逻辑轴号映射到底层雷赛硬件轴号。
    int resolveLeadshineAxisIndex(int logicalIndex) const;
    // 返回指定轴的雷赛当量。
    double resolveLeadshineAxisEquiv(int logicalIndex) const;
    // 生成适合日志/UI 的轴名称。
    QString axisDisplayName(int logicalIndex) const;
    // 判断指定轴是否配置了有效软件限位。
    bool hasValidMotorSoftwareLimit(int logicalIndex) const;
    // 根据绝对位置和零位计算相对位置。
    double relativeMotorPosition(int logicalIndex, double absolutePosition) const;
    // 判断指定轴是否有有效软件安全零位。
    bool hasValidMotorSafetyHome(int logicalIndex) const;
    bool hasValidMotorSessionSafetyTraceHome(int logicalIndex) const;
    bool hasValidMotorSafetyEncoderHome(int logicalIndex) const;
    // 返回指定轴软件安全零位的工程单位。
    double motorSafetyHomeUnit(int logicalIndex) const;
    // 根据绝对位置和软件安全零位计算相对位置。
    double safetyRelativeMotorPosition(int logicalIndex, double absolutePosition) const;
    bool motorSafetyRelativeFromTraceFrame(int logicalIndex,
                                           const MotorTracePositionWindowFrame& frame,
                                           double& relativePosition,
                                           MotorSafetyRelativePositionSource* source = nullptr) const;
    bool safetyRelativeMotorTargetFromAbsoluteDirect(int logicalIndex,
                                                     double absolutePosition,
                                                     double& relativePosition);
    // 直接读取指定轴编码器并换算为软件安全相对位置。
    bool readMotorSafetyRelativePositionDirect(int logicalIndex, double& relativePosition);
    // 将 Trace 原始脉冲转换为电机工程单位。
    double tracePulseToMotorUnit(int logicalIndex, qint64 rawPulse) const;
    // 直接从底层读取电机位置单位，可选更新缓存。
    bool readMotorPositionUnitDirect(int logicalIndex, double& position, bool updateCache = true);
    // 直接从底层读取编码器位置单位。
    bool readMotorEncoderUnitDirect(int logicalIndex, double& position);
    // 校验相对位置目标是否在软件限位内。
    bool validateRelativeMotorSoftwareLimit(int logicalIndex,
                                            double relativePosition,
                                            const QString& commandName,
                                            QString* errorMessage = nullptr) const;
    // 校验安全相对位置是否在软件限位内。
    bool validateSafetyRelativeMotorSoftwareLimit(int logicalIndex,
                                                  double relativePosition,
                                                  const QString& commandName,
                                                  QString* errorMessage = nullptr) const;
    // 校验自动轨迹启动前当前安全相对位置未超限。
    bool validateCurrentMotorSafetyLimitForAutomaticMotion(int logicalIndex,
                                                           const QString& commandName,
                                                           QString* errorMessage = nullptr);
    // 校验绝对位置目标是否在软件限位内。
    bool validateAbsoluteMotorSoftwareLimit(int logicalIndex,
                                            double absolutePosition,
                                            const QString& commandName,
                                            QString* errorMessage = nullptr);
    // 校验速度命令在当前位置下是否可能冲出软件限位。
    bool validateVelocityMotorSoftwareLimit(int logicalIndex,
                                            double currentAbsolutePosition,
                                            double velocity,
                                            const QString& commandName,
                                            QString* errorMessage = nullptr);
    bool validateVelocityMotorSoftwareLimitFromSnapshot(
            int logicalIndex,
            double safetyRelativePosition,
            double velocity,
            const QString& commandName,
            QString* errorMessage = nullptr) const;
    bool motorVelBatchFastDirect(
            const std::vector<int>& motorIndex,
            const std::vector<double>& velocity,
            double changeTimeSec,
            const std::vector<double>& currentAbsolutePosition,
            const EndpointRemoteVelocitySafetyContext* endpointRemoteSafetyContext,
            qint64 maximumFeedbackAgeUs,
            quint64 endpointRemoteSessionToken,
            EndpointRemoteVelocityCommandReport* endpointRemoteCommandReport);
    // 直接查询单个电机轴的底层连接诊断。
    ConnectionItemDiagnostics queryMotorAxisDiagnosticsDirect(int logicalIndex) const;
    // 直接查询 EtherCAT 主站/控制卡的底层连接诊断。
    ConnectionItemDiagnostics queryControllerDiagnosticsDirect() const;
    // 直接查询控制器、电机和传感器整体连接诊断。
    ConnectionDiagnostics queryConnectionDiagnosticsDirect() const;
    // 标记需要刷新连接诊断，供 UI 定时读取。
    void requestConnectionDiagnosticsRefresh() const;
    // 缓存最近一次连接诊断快照。
    void storeConnectionDiagnosticsSnapshot(const ConnectionDiagnostics& snapshot) const;

signals:
    void motorCurPosUpdateSignal(std::vector<double> pos);
    void motorCurVelUpdateSignal(std::vector<double> vel);
    void displayInfoSignal(std::string info, std::string type);
    void pvtTraceStartDelayMeasured(qint64 pvtUploadMonotonicUs,
                                    int pointCount,
                                    int axisCount,
                                    quint32 commandStartFrameSequence,
                                    quint32 feedbackStartFrameSequence,
                                    quint64 frameIntervalCount,
                                    int ethercatBusCycleUs,
                                    qint64 delayUs,
                                    int commandStartAxis,
                                    int feedbackStartAxis);
};

#endif // HARDWAREINTERFACE_H
