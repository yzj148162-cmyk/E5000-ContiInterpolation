#ifndef ENDPOINTREMOTECONTROL_H
#define ENDPOINTREMOTECONTROL_H

#include "compensatedcablekinematics.h"
#include "onlinevelocitycontrol.h"

#include <QString>

#include <array>
#include <vector>

constexpr int kEndpointRemoteBoundaryDirectionCount = 6;
constexpr int kEndpointRemoteAngularBoundaryDirectionCount = 6;
constexpr double kEndpointRemoteOutwardRestartSafeAccelerationRatio = 0.80;
constexpr double kEndpointRemoteOutwardRestartPoseErrorMarginMm = 20.0;

enum class EndpointRemoteBoundaryState {
    Normal,
    SoftBoundary,
    Braking,
    BlockedOutward
};

// 输入方向的物理含义。YawLockedEulerRotation 的前两个分量表示ZYX欧拉角
// Rx/Ry变化率，Rz变化率固定为零；控制器再换算对应的真实全局角速度。
enum class EndpointRemoteMotionMode {
    None = 0,
    Translation,
    YawLockedEulerRotation
};

// 末端遥控硬件执行状态。它描述“是否已经进入JOG、当前是否正在建立/清零”这一
// 单一事实来源，避免继续用多个布尔量组合推断速度命令是否应真正进入HardwareThread。
enum class EndpointRemoteActuationProfile {
    Disarmed = 0,
    VerifyingStationary,
    ArmedIdle,
    StartingJog,
    JogActive,
    Stopping,
    ZeroHolding,
    Faulted
};

enum class EndpointRemoteCommandIntent {
    None = 0,
    StartOrUpdateJog,
    EnterZeroHolding
};

// 单个50 mm体素内允许的Rx/Ry欧拉角范围。空间范围采用CSV中的毫米坐标，
// 角度采用弧度；valid=false的格子不能用于遥控。
struct EndpointRemoteVoxelAngleLimit {
    std::array<double, 3> minimumMm{{0.0, 0.0, 0.0}};
    std::array<double, 3> maximumMm{{0.0, 0.0, 0.0}};
    double rxMinimumRad = 0.0;
    double rxMaximumRad = 0.0;
    double ryMinimumRad = 0.0;
    double ryMaximumRad = 0.0;
    bool valid = false;
};

// 遥控体素姿态表。加载阶段要求规则网格完整、无重复且全部格子有效；
// 控制循环只做常数时间索引，不在实时路径中读文件或解析CSV。
struct EndpointRemoteVoxelAngleMap {
    std::array<double, 3> workspaceMinimumMm{{0.0, 0.0, 0.0}};
    std::array<double, 3> workspaceMaximumMm{{0.0, 0.0, 0.0}};
    std::array<double, 3> cellSizeMm{{0.0, 0.0, 0.0}};
    std::array<int, 3> cellCount{{0, 0, 0}};
    std::vector<EndpointRemoteVoxelAngleLimit> cells;
    QString sourceFilePath;

    bool loadCsv(const QString& filePath, QString* errorMessage = nullptr);
    bool validate(QString* errorMessage = nullptr) const;
    const EndpointRemoteVoxelAngleLimit* limitAt(
            const std::array<int, 3>& index) const;
    const EndpointRemoteVoxelAngleLimit* limitForPose(
            const std::array<double, 6>& pose,
            std::array<int, 3>* index = nullptr) const;
    bool commonRxRyRange(std::array<double, 2>* rxRangeRad,
                         std::array<double, 2>* ryRangeRad) const;
};

struct EndpointRemoteConfig {
    OnlineVelocityConfig onlineVelocity;
    CompensatedCableKinematics::Configuration kinematics;
    std::array<double, 6> initialPose{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> workspaceMinimum{{0.0, 0.0, 0.0}};
    std::array<double, 3> workspaceMaximum{{0.0, 0.0, 0.0}};
    OnlineVelocityAxisArray motorUnitPerRadian{};
    OnlineVelocityAxisArray motorPositionMinimum{};
    OnlineVelocityAxisArray motorPositionMaximum{};
    OnlineVelocityAxisArray preparedMotorPosition{};
    double preparedMotorDriftTolerance = 0.0;
    double translationSpeedMmPerSec = 0.0;
    double translationAccelerationMmPerSec2 = 0.0;
    double maximumAngularSpeedRadPerSec = 0.0;
    double maximumAngularAccelerationRadPerSec2 = 0.0;
    // Rz继续使用这里的全局硬边界；Rx/Ry硬边界由voxelAngleLimits替代。
    std::array<double, 3> orientationMinimumRad{{0.0, 0.0, 0.0}};
    std::array<double, 3> orientationMaximumRad{{0.0, 0.0, 0.0}};
    // Rx/Ry在进入遥控时改为全部体素允许范围的交集；Rz保留全局阈值。
    // 超出平动回正阈值时只允许转动恢复，不能接受平动指令。
    std::array<double, 3> translationSafeOrientationMinimumRad{{0.0, 0.0, 0.0}};
    std::array<double, 3> translationSafeOrientationMaximumRad{{0.0, 0.0, 0.0}};
    EndpointRemoteVoxelAngleMap voxelAngleLimits;
    double softBoundaryMarginMm = 0.0;
    double boundaryReleaseHysteresisMm = 0.0;
    // 朝外重启保护层采用界面最大平移速度/加速度动态计算：
    // G = vmax^2 / (2 * asafe) + vmax * Tresponse + Epose。
    // Tresponse沿用遥控安全调度预算：候选积分、命令响应和允许漏拍补推
    // 各一个配置周期。
    // 这两个参数只属于 Lite 末端遥控 profile，不改变普通在线速度控制。
    double outwardRestartSafeAccelerationRatio =
            kEndpointRemoteOutwardRestartSafeAccelerationRatio;
    double outwardRestartPoseErrorMarginMm =
            kEndpointRemoteOutwardRestartPoseErrorMarginMm;
    qint64 inputHeartbeatTimeoutUs = 250000;

    double outwardRestartResponseTimeSec() const;
    double outwardRestartSafeAccelerationMmPerSec2() const;
    double outwardRestartGuardMm() const;
    bool validate(QString* errorMessage = nullptr) const;
};

struct EndpointRemoteStep {
    enum class Action {
        None,
        CommandVelocity,
        EmergencyStop
    };

    Action action = Action::None;
    qint64 monotonicUs = 0;
    // 普通反馈检查或HardwareThread复合任务完成同帧Trace读取/校验时的单调
    // 时间。该时间只用于遥控命令分段归因，不参与控制或安全判定。
    qint64 traceValidationCompletedUs = 0;
    qint64 planningStartedUs = 0;
    qint64 planningCompletedUs = 0;
    qint64 wallClockUs = 0;
    quint64 logicalFrameSequence = 0;
    std::array<double, 6> desiredPose{};
    std::array<double, 3> targetVelocityMmPerSec{};
    std::array<double, 3> effectiveVelocityMmPerSec{};
    std::array<double, 3> targetEulerRateRadPerSec{};
    std::array<double, 3> effectiveEulerRateRadPerSec{};
    std::array<double, 3> effectiveGlobalAngularVelocityRadPerSec{};
    double effectiveGlobalAngularAccelerationRadPerSec2 = 0.0;
    OnlineVelocityAxisArray referencePosition{};
    OnlineVelocityAxisArray actualPosition{};
    OnlineVelocityAxisArray actualVelocity{};
    OnlineVelocityAxisArray commandVelocity{};
    std::array<double, 3> inputDirection{};
    EndpointRemoteMotionMode requestedMotionMode = EndpointRemoteMotionMode::None;
    EndpointRemoteMotionMode activeMotionMode = EndpointRemoteMotionMode::None;
    EndpointRemoteActuationProfile actuationProfile =
            EndpointRemoteActuationProfile::Disarmed;
    EndpointRemoteCommandIntent commandIntent =
            EndpointRemoteCommandIntent::None;
    QString diagnosticMessage;
    bool diagnosticWarning = false;
    QString reason;
};

// ControlWorker 在进入遥控调度器前采集的分层时序。调度器使用真实的
// 命令准备时刻做周期判断，同时把硬件Trace读取与线程本身的抖动分开记录。
struct EndpointRemoteTimingContext {
    qint64 workerLoopEntryUs = 0;
    qint64 workerLoopIntervalUs = 0;
    qint64 traceReadDurationUs = 0;
    qint64 preDispatchDurationUs = 0;
    qint64 previousWorkerLoopDurationUs = 0;
};

struct EndpointRemoteStatus {
    enum class State {
        Idle,
        Prepared,
        WaitingForTrace,
        Running,
        Stopped,
        Fault
    };

    State state = State::Idle;
    EndpointRemoteActuationProfile actuationProfile =
            EndpointRemoteActuationProfile::Disarmed;
    QString message;
    std::array<double, 6> initialPose{};
    std::array<double, 6> desiredPose{};
    std::array<double, 3> workspaceMinimum{};
    std::array<double, 3> workspaceMaximum{};
    std::array<double, 3> targetVelocityMmPerSec{};
    std::array<double, 3> effectiveVelocityMmPerSec{};
    std::array<double, 3> targetEulerRateRadPerSec{};
    std::array<double, 3> effectiveEulerRateRadPerSec{};
    std::array<double, 3> effectiveGlobalAngularVelocityRadPerSec{};
    double effectiveGlobalAngularAccelerationRadPerSec2 = 0.0;
    double yawLockReferenceRad = 0.0;
    double yawLockErrorRad = 0.0;
    EndpointRemoteMotionMode requestedMotionMode = EndpointRemoteMotionMode::None;
    EndpointRemoteMotionMode activeMotionMode = EndpointRemoteMotionMode::None;
    std::array<double, kEndpointRemoteAngularBoundaryDirectionCount>
            angularBoundaryDistanceRad{};
    bool translationBlockedByOrientation = false;
    bool angularBoundaryBrakingActive = false;
    bool voxelAngleBoundaryBrakingActive = false;
    QString angularBoundarySummary;
    QString orientationRecoverySummary;
    bool voxelAngleLimitAvailable = false;
    std::array<int, 3> voxelIndex{{-1, -1, -1}};
    std::array<double, 3> voxelMinimumMm{};
    std::array<double, 3> voxelMaximumMm{};
    std::array<double, 2> voxelRxRangeRad{};
    std::array<double, 2> voxelRyRangeRad{};
    QString voxelAngleLimitSummary;
    std::array<double, kEndpointRemoteBoundaryDirectionCount> boundaryDistanceMm{};
    std::array<EndpointRemoteBoundaryState,
               kEndpointRemoteBoundaryDirectionCount> boundaryState{};
    double outwardRestartGuardMm = 0.0;
    double outwardRestartSafeAccelerationMmPerSec2 = 0.0;
    double outwardRestartResponseTimeMs = 0.0;
    double outwardRestartPoseErrorMarginMm = 0.0;
    QString boundarySummary;
    QString latestBoundaryEvent;
    OnlineVelocityAxisArray referencePosition{};
    OnlineVelocityAxisArray actualPosition{};
    OnlineVelocityAxisArray actualVelocity{};
    OnlineVelocityAxisArray commandVelocity{};
    quint64 inputSequence = 0;
    quint64 commandCount = 0;
    quint64 schedulingRecoveryCount = 0;
    quint64 missedCycleCount = 0;
    quint64 boundaryEventSequence = 0;
    quint64 latestLogicalFrameSequence = 0;
    qint64 inputAgeUs = 0;
    qint64 inputSourceUiAgeUs = 0;
    bool inputSourceUiFresh = true;
    qint64 latestCommandIntervalUs = 0;
    qint64 maximumCommandIntervalUs = 0;
    qint64 latestScheduleLatenessUs = 0;
    qint64 maximumScheduleLatenessUs = 0;
    qint64 latestWorkerLoopIntervalUs = 0;
    qint64 latestTraceReadDurationUs = 0;
    qint64 maximumTraceReadDurationUs = 0;
    qint64 latestPreDispatchDurationUs = 0;
    qint64 maximumPreDispatchDurationUs = 0;
    qint64 previousWorkerLoopDurationUs = 0;
    qint64 latestCommandApiUs = 0;
    qint64 maximumCommandApiUs = 0;
    qint64 latestFullCycleUs = 0;
    qint64 maximumFullCycleUs = 0;
    qint64 latestPlanningUs = 0;
    qint64 maximumPlanningUs = 0;
    double elapsedSec = 0.0;
};

class EndpointRemoteControl
{
public:
    bool prepare(const EndpointRemoteConfig& config,
                 QString* errorMessage = nullptr);
    bool start(qint64 nowUs, QString* errorMessage = nullptr);
    void resetTraceWaitClock(qint64 nowUs);
    void updateInput(EndpointRemoteMotionMode motionMode,
                     const std::array<double, 3>& normalizedDirection,
                     quint64 sequence,
                     qint64 nowUs,
                     bool uiSourceFresh = true,
                     qint64 uiSourceAgeUs = 0);
    EndpointRemoteStep step(
            const OnlineVelocityFeedback& feedback,
            qint64 nowUs,
            const EndpointRemoteTimingContext& timing = EndpointRemoteTimingContext());
    void noteCommandResult(const EndpointRemoteStep& step,
                           bool commandOk,
                           qint64 apiDurationUs,
                           qint64 fullCycleDurationUs,
                           const QString& commandFailureReason,
                           const OnlineVelocityFeedback& commandFeedback);
    void noteCommandDeferred(const EndpointRemoteStep& step,
                             const OnlineVelocityFeedback& commandFeedback,
                             const QString& reason);
    bool cancelPreparedCommandIfInputChanged(
            const EndpointRemoteStep& preparedStep,
            qint64 nowUs);
    void stop(bool fault, const QString& reason);

    bool isActive() const;
    bool isPrepared() const;
    const EndpointRemoteConfig& currentConfig() const;
    EndpointRemoteStatus status() const;

private:
    struct Candidate {
        bool valid = false;
        QString errorMessage;
        std::array<double, 6> pose{};
        std::array<double, 3> effectiveVelocity{};
        std::array<double, 3> effectiveEulerRate{};
        std::array<double, 3> globalAngularVelocity{};
        double globalAngularAccelerationRadPerSec2 = 0.0;
        OnlineVelocityAxisArray referencePosition{};
        OnlineVelocityAxisArray commandVelocity{};
        std::array<EndpointRemoteBoundaryState,
                   kEndpointRemoteBoundaryDirectionCount> boundaryState{};
        std::vector<double> relativeMotorThetaRad;
        CompensatedCableKinematics::State kinematicsState;
    };

    bool feedbackReady(const OnlineVelocityFeedback& feedback) const;
    Candidate buildCandidate(
            const std::array<double, 3>& effectiveVelocity,
            const std::array<double, 3>& effectiveEulerRate,
            bool enforceCommandDynamics = true) const;
    std::array<double, 3> accelerationLimitedVelocity(
            const std::array<double, 3>& targetVelocity) const;
    std::array<double, 3> accelerationLimitedEulerRate(
            const std::array<double, 3>& targetEulerRate) const;
    std::array<double, kEndpointRemoteBoundaryDirectionCount> boundaryDistances(
            const std::array<double, 6>& pose) const;
    std::array<EndpointRemoteBoundaryState,
               kEndpointRemoteBoundaryDirectionCount> advanceBoundaryStates(
            const std::array<double, 6>& pose,
            const std::array<double, 3>& effectiveVelocity,
            std::array<EndpointRemoteBoundaryState,
                       kEndpointRemoteBoundaryDirectionCount> states) const;
    bool stoppingTrajectoryInsideWorkspace(
            const std::array<double, 3>& effectiveVelocity,
            std::array<bool, kEndpointRemoteBoundaryDirectionCount>* violatedFaces = nullptr,
            bool* voxelAngleLimitViolated = nullptr) const;
    bool stoppingRotationInsideBounds(
            const std::array<double, 3>& effectiveEulerRate) const;
    bool translationSegmentInsideVoxelAngleLimits(
            const std::array<double, 3>& startPosition,
            const std::array<double, 3>& endPosition,
            const std::array<double, 6>& orientationPose) const;
    bool yawLockedRotationTrajectoryInsideBounds(
            const std::array<double, 6>& startPose,
            const std::array<double, 3>& unitEulerRate,
            double angleRad) const;
    bool yawLockedAngularDynamicsWithinLimits(
            const std::array<double, 3>& effectiveEulerRate,
            std::array<double, 3>* globalAngularVelocity = nullptr,
            double* globalAngularAccelerationRadPerSec2 = nullptr,
            QString* errorMessage = nullptr) const;
    bool orientationBoundsForPose(
            const std::array<double, 6>& pose,
            bool translationSafe,
            std::array<double, 3>* minimum,
            std::array<double, 3>* maximum,
            const EndpointRemoteVoxelAngleLimit** voxelLimit = nullptr,
            std::array<int, 3>* voxelIndex = nullptr) const;
    bool orientationInsideBounds(const std::array<double, 6>& pose,
                                 bool translationSafe) const;
    void updateOrientationStatus(const std::array<double, 6>& pose);
    void updateBoundaryStatus(
            const std::array<double, 6>& candidatePose,
            std::array<EndpointRemoteBoundaryState,
                       kEndpointRemoteBoundaryDirectionCount> nextState);
    void setFault(const QString& reason);

    EndpointRemoteConfig config;
    EndpointRemoteStatus currentStatus;
    CompensatedCableKinematics kinematics;
    CompensatedCableKinematics::State committedKinematicsState;
    std::vector<double> committedRelativeMotorThetaRad;
    std::array<double, 6> committedPose{};
    EndpointRemoteMotionMode requestedMotionMode = EndpointRemoteMotionMode::None;
    EndpointRemoteMotionMode activeMotionMode = EndpointRemoteMotionMode::None;
    std::array<double, 3> requestedDirection{};
    std::array<double, 3> committedEffectiveVelocity{};
    std::array<double, 3> committedEffectiveEulerRate{};
    std::array<double, 3> committedGlobalAngularVelocity{};
    double lockedYawRad = 0.0;
    std::array<EndpointRemoteBoundaryState,
               kEndpointRemoteBoundaryDirectionCount> boundaryState{};
    OnlineVelocityAxisArray actualStartMotorPosition{};
    OnlineVelocityAxisArray lastCommandVelocity{};
    Candidate pendingCandidate;
    bool pendingCandidateValid = false;
    EndpointRemoteCommandIntent pendingCommandIntent =
            EndpointRemoteCommandIntent::None;
    EndpointRemoteActuationProfile pendingPriorActuationProfile =
            EndpointRemoteActuationProfile::Disarmed;
    bool actualStartCaptured = false;
    bool inputHeartbeatArmed = false;
    bool inputHeartbeatStale = false;
    qint64 waitStartUs = 0;
    qint64 sessionStartUs = 0;
    qint64 nextDueUs = 0;
    qint64 lastCommandDispatchUs = 0;
    qint64 lastInputUpdateUs = 0;
    quint64 lastUsedFrameSequence = 0;
    bool lastUsedFrameSequenceValid = false;
    qint64 lastSchedulingRecoveryDiagnosticUs = 0;
    quint64 suppressedSchedulingRecoveryDiagnosticCount = 0;
};

#endif // ENDPOINTREMOTECONTROL_H
