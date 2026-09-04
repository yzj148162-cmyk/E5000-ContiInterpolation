#ifndef FORCEINTERACTIONRUNTIMECONTROL_H
#define FORCEINTERACTIONRUNTIMECONTROL_H

#include "cdprdynamics.h"
#include "compensatedcablekinematics.h"
#include "forceinteractionrunrecorder.h"
#include "onlinevelocitycontrol.h"
#include "wrenchsource.h"
#include "wrenchtransformer.h"

#include <array>
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
    std::array<double, 6> poseLowerBoundsMmRad{};
    std::array<double, 6> poseUpperBoundsMmRad{};
    OnlineVelocityAxisArray motorUnitPerRadian{};
    OnlineVelocityAxisArray motorPositionMinimum{};
    OnlineVelocityAxisArray motorPositionMaximum{};
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
    QString recordingDirectory;

    bool validate(QString* errorMessage = nullptr) const;
};

struct ForceInteractionRuntimeFeedback
{
    OnlineVelocityAxisArray actualPosition{};
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

struct ForceInteractionRuntimeStatus
{
    enum class State { Idle, Prepared, WaitingForTrace, Running, Completed, Stopped, Fault };
    State state = State::Idle;
    QString message;
    QString recordFile;
    quint64 stepCount = 0;
    quint64 commandCount = 0;
    quint64 missedCycleCount = 0;
    quint64 droppedRecordCount = 0;
    quint64 latestTraceSequence = 0;
    double elapsedS = 0.0;
    double maximumPositionError = 0.0;
    qint64 latestCalculationUs = 0;
    qint64 maximumCalculationUs = 0;
    qint64 latestApiUs = 0;
    qint64 maximumApiUs = 0;
    ForceInteractionPlatformState desiredState;
    OnlineVelocityAxisArray actualStartPosition{};
    OnlineVelocityAxisArray desiredCableLengthMm{};
    OnlineVelocityAxisArray referencePosition{};
    OnlineVelocityAxisArray actualPosition{};
    OnlineVelocityAxisArray commandVelocity{};
};

class ForceInteractionRuntimeControl
{
public:
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
    const ForceInteractionRuntimeConfig& currentConfig() const;
    ForceInteractionRuntimeStatus status() const;

private:
    void setTerminal(ForceInteractionRuntimeStatus::State state,
                     const QString& message);
    bool feedbackReady(const ForceInteractionRuntimeFeedback& feedback) const;

    ForceInteractionRuntimeConfig config_;
    ForceInteractionRuntimeStatus status_;
    SimulatedWrenchSource wrenchSource_;
    std::unique_ptr<WrenchTransformer> wrenchTransformer_;
    CdprDynamics dynamics_;
    CompensatedCableKinematics kinematics_;
    CompensatedCableKinematics::State kinematicsState_;
    std::unique_ptr<ForceInteractionRunRecorder> recorder_;
    OnlineVelocityAxisArray actualStartPosition_{};
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
};

#endif // FORCEINTERACTIONRUNTIMECONTROL_H
