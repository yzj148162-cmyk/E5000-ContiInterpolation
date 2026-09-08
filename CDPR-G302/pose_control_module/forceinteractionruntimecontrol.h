#ifndef FORCEINTERACTIONRUNTIMECONTROL_H
#define FORCEINTERACTIONRUNTIMECONTROL_H

#include "cdprdynamics.h"
#include "compensatedcablekinematics.h"
#include "forceinteractionrunrecorder.h"
#include "onlinevelocitycontrol.h"
#include "physicalworkspaceboundary.h"
#include "wrenchsource.h"
#include "wrenchtransformer.h"

#include <array>
#include <limits>
#include <memory>

struct ForceInteractionRuntimeConfig
{
    QString machineTemplateName;
    int periodUs = 5000;
    double maximumTestDurationS = 10.0;
    bool translationOnly = true;
    SimulatedWrenchProfile wrenchProfile;
    ForceSensorTransformConfig sensorTransform;
    ForceInteractionRigidBodyConfig rigidBody;
    NewmarkBetaConfig newmark;
    ForceInteractionPlatformState initialState;
    CompensatedCableKinematics::Configuration kinematics;
    PhysicalWorkspaceBoundaryConfig physicalWorkspace;
    OnlineVelocityAxisArray motorUnitPerRadian{};
    // 以G302绞盘确认点（未确认时为本次上电同帧Trace位置）为零点的
    // 电机安全相对位置边界，单位与电机位置反馈一致。
    OnlineVelocityAxisArray motorSafetyRelativeMinimum{};
    OnlineVelocityAxisArray motorSafetyRelativeMaximum{};
    bool feedForwardEnabled = true;
    double feedForwardGain = 1.0;
    bool pidEnabled = true;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integralLimit = 10.0;
    double correctionVelocityLimit = 20.0;
    double velocityLimit = 90.0;
    double followingErrorLimit = 5.0;
    double onlineChangeTimeS = 0.001;
    qint64 traceTimeoutUs = 100000;
    DynamicWorkspaceSafetyConfig workspaceSafety;
    double brakingStopVelocityMmPerSec = 0.1;
    QString recordingDirectory;

    bool validate(QString* errorMessage = nullptr) const;
};

struct ForceInteractionRuntimeFeedback
{
    OnlineVelocityAxisArray actualPosition{};
    // 与actualPosition来自同一Trace帧，坐标原点是本次G302绞盘安全基准。
    OnlineVelocityAxisArray safetyRelativePosition{};
    std::array<bool, kOnlineVelocityAxisCount> safetyRelativePositionFromTrace{};
    OnlineVelocityAxisArray actualVelocity{};
    qint64 wallClockUs = 0;
    qint64 monotonicUs = 0;
    qint64 newestFrameAgeUs = -1;
    int frameCount = 0;
    int traceSamplePeriodUs = 0;
    quint64 logicalFrameSequence = 0;
    bool fromTrace = false;
    bool frameSequenceValid = false;
    bool timingReliable = false;
    bool fifoCaughtUp = false;
    bool traceLost = false;
};

struct ForceInteractionRuntimeStep
{
    enum class Action { None, CommandVelocity, NormalStop, EmergencyStop };
    Action action = Action::None;
    QString reason;
    OnlineVelocityAxisArray commandVelocity{};
    OnlineVelocityAxisArray actualPosition{};
    ForceInteractionRunRecord record;
};

enum class ForceInteractionControlledStopCause
{
    None = 0,
    UserRequest,
    DurationReached,
    AccelerationLimit,
    WorkspaceBoundary
};

struct ForceInteractionRuntimeStatus
{
    enum class State { Idle, Prepared, WaitingForTrace, Running, Braking,
                       Completed, Stopped, Fault };
    State state = State::Idle;
    QString message;
    QString recordFile;
    quint64 stepCount = 0;
    quint64 commandCount = 0;
    quint64 missedCycleCount = 0;
    quint64 acceptedRecordCount = 0;
    quint64 writtenRecordCount = 0;
    quint64 droppedRecordCount = 0;
    quint64 latestTraceSequence = 0;
    double elapsedS = 0.0;
    double maximumPositionError = 0.0;
    qint64 latestCalculationUs = 0;
    qint64 maximumCalculationUs = 0;
    qint64 latestApiUs = 0;
    qint64 maximumApiUs = 0;
    bool experimentValid = true;
    double latestWorkspaceClearanceMm = 0.0;
    double minimumWorkspaceClearanceMm =
            std::numeric_limits<double>::infinity();
    double workspaceTriggerDistanceMm = 0.0;
    QString safetyStopReason;
    ForceInteractionControlledStopCause controlledStopCause =
            ForceInteractionControlledStopCause::None;
    QString recordingError;
    ForceInteractionPlatformState desiredState;
    OnlineVelocityAxisArray actualStartPosition{};
    OnlineVelocityAxisArray actualStartSafetyRelativePosition{};
    OnlineVelocityAxisArray desiredCableLengthMm{};
    OnlineVelocityAxisArray referencePosition{};
    OnlineVelocityAxisArray safetyRelativeReferencePosition{};
    OnlineVelocityAxisArray safetyRelativeActualPosition{};
    OnlineVelocityAxisArray actualPosition{};
    OnlineVelocityAxisArray commandVelocity{};
};

class ForceInteractionRuntimeControl
{
public:
    // Deterministic pure-software acceptance for the coordinated braking
    // state and fault-latching invariants. Stage A invokes this exact code.
    static bool runControlledStopSelfChecks(
            const PhysicalWorkspaceBoundaryConfig& physicalWorkspace,
            QString* errorMessage = nullptr);

    bool prepare(const ForceInteractionRuntimeConfig& config,
                 QString* errorMessage = nullptr);
    bool start(qint64 nowUs, QString* errorMessage = nullptr);
    ForceInteractionRuntimeStep step(const ForceInteractionRuntimeFeedback& feedback,
                                     qint64 nowUs);
    void noteCommandResult(const ForceInteractionRuntimeStep& step,
                           bool commandOk,
                           qint64 apiDurationUs,
                           qint64 fullCycleDurationUs);
    void stop(bool fault, const QString& reason);
    void finishRecording();
    bool isActive() const;
    bool isPrepared() const;
    bool requestControlledStop(const QString& reason,
                               bool experimentFailure = false,
                               ForceInteractionControlledStopCause cause =
                                   ForceInteractionControlledStopCause::UserRequest);
    const ForceInteractionRuntimeConfig& currentConfig() const;
    ForceInteractionRuntimeStatus status() const;

private:
    void setTerminal(ForceInteractionRuntimeStatus::State state,
                     const QString& message);
    bool feedbackReady(const ForceInteractionRuntimeFeedback& feedback) const;
    ForceInteractionPlatformState advanceBrakingState(
            bool& stopped, QString* errorMessage = nullptr);

    ForceInteractionRuntimeConfig config_;
    ForceInteractionRuntimeStatus status_;
    SimulatedWrenchSource wrenchSource_;
    std::unique_ptr<WrenchTransformer> wrenchTransformer_;
    CdprDynamics dynamics_;
    CompensatedCableKinematics kinematics_;
    PhysicalWorkspaceBoundary physicalBoundary_;
    CompensatedCableKinematics::State kinematicsState_;
    std::unique_ptr<ForceInteractionRunRecorder> recorder_;
    OnlineVelocityAxisArray actualStartPosition_{};
    OnlineVelocityAxisArray actualStartSafetyRelativePosition_{};
    OnlineVelocityAxisArray lastReferencePosition_{};
    OnlineVelocityAxisArray integral_{};
    OnlineVelocityAxisArray previousError_{};
    bool actualStartCaptured_ = false;
    bool previousErrorValid_ = false;
    qint64 waitStartUs_ = 0;
    qint64 lastGoodTraceUs_ = 0;
    qint64 nextDueUs_ = 0;
    quint64 lastFrameSequence_ = 0;
    bool lastFrameSequenceValid_ = false;
    ForceInteractionPlatformState brakingState_;
    QString controlledStopReason_;
};

#endif // FORCEINTERACTIONRUNTIMECONTROL_H
