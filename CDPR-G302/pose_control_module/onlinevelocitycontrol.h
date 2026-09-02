#ifndef ONLINEVELOCITYCONTROL_H
#define ONLINEVELOCITYCONTROL_H

#include "runtimefeatureswitches.h"

#include <QString>

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

constexpr int kOnlineVelocityAxisCount = 8;
using OnlineVelocityAxisArray = std::array<double, kOnlineVelocityAxisCount>;

constexpr std::array<int, 5> kOnlineVelocitySupportedPeriodsUs{{
    1000, 2000, 5000, 10000, 20000
}};
constexpr int kOnlineVelocityDefaultPeriodUs = 20000;

constexpr bool isSupportedOnlineVelocityPeriodUs(int periodUs)
{
    for(const int supportedPeriodUs : kOnlineVelocitySupportedPeriodsUs){
        if(periodUs == supportedPeriodUs){
            return true;
        }
    }
    return false;
}

struct OnlineVelocityPlan {
    std::array<int, kOnlineVelocityAxisCount> axes{{0, 1, 2, 3, 4, 5, 6, 7}};
    int samplePeriodUs = 0;
    std::vector<double> timeSec;
    std::vector<OnlineVelocityAxisArray> position;
    std::vector<OnlineVelocityAxisArray> velocity;
    QString sourceName;

    bool validate(QString* errorMessage = nullptr) const;
    double durationSec() const;
};

struct OnlineVelocityConfig {
    int periodUs = kOnlineVelocityDefaultPeriodUs;
    bool feedForwardEnabled = true;
    double feedForwardGain = 1.0;
    bool pidEnabled = false;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integralLimit = 0.03;
    double maxCorrectionVelocity = 0.06;
    double maxVelocity = 0.25;
    double maxAcceleration = 0.50;
    double onlineChangeTimeSec = 0.001;
    double startVelocityThreshold = 1.0e-6;
    double endpointSettleSec = 1.0;
    qint64 traceTimeoutUs = 100000;
    qint64 initialTraceWaitTimeoutUs =
            RuntimeFeatureSwitches::kOnlineVelocityInitialTraceWaitTimeoutUs;
    QString recordDirectory;

    bool validate(QString* errorMessage = nullptr) const;
    // FIFO是否追平由底层独立校验；这里再限制反馈传输时延不超过在线控制
    // 周期且不超过5 ms，避免仅因控制周期较长而接受过旧反馈。
    qint64 traceFeedbackDelayLimitUs() const;
};

struct OnlineVelocityFeedback {
    OnlineVelocityAxisArray actualPosition{};
    OnlineVelocityAxisArray actualVelocity{};
    OnlineVelocityAxisArray tracedCommandVelocity{};
    std::array<quint16, kOnlineVelocityAxisCount> motorStatusWord{};
    std::array<int, kOnlineVelocityAxisCount> motorStateMachine{};
    qint64 wallClockUs = 0;
    qint64 monotonicUs = 0;
    qint64 newestFrameAgeUs = -1;
    int frameCount = 0;
    int fifoValidNum = 0;
    int fifoFreeNum = 0;
    int traceSamplePeriodUs = 0;
    quint64 logicalFrameSequence = 0;
    bool fromTrace = false;
    bool frameSequenceValid = false;
    bool timingReliable = false;
    bool fifoCaughtUp = false;
    bool traceLost = false;
};

struct OnlineVelocityStep {
    enum class Action {
        None,
        CommandVelocity,
        NormalStop,
        EmergencyStop
    };

    Action action = Action::None;
    qint64 wallClockUs = 0;
    qint64 monotonicUs = 0;
    qint64 controlIntervalUs = 0;
    quint64 logicalFrameSequence = 0;
    double elapsedSec = 0.0;
    OnlineVelocityAxisArray referencePosition{};
    OnlineVelocityAxisArray referenceVelocity{};
    OnlineVelocityAxisArray actualPosition{};
    OnlineVelocityAxisArray actualVelocity{};
    OnlineVelocityAxisArray tracedCommandVelocity{};
    OnlineVelocityAxisArray positionError{};
    OnlineVelocityAxisArray feedForwardTerm{};
    OnlineVelocityAxisArray pTerm{};
    OnlineVelocityAxisArray iTerm{};
    OnlineVelocityAxisArray dTerm{};
    OnlineVelocityAxisArray commandVelocity{};
    QString reason;
};

struct OnlineVelocityStatus {
    enum class State {
        Idle,
        Prepared,
        WaitingForTrace,
        Running,
        Settling,
        Completed,
        Stopped,
        Fault
    };

    State state = State::Idle;
    QString message;
    QString recordFile;
    double elapsedSec = 0.0;
    double durationSec = 0.0;
    double maxRawPositionError = 0.0;
    qint64 latestCommandApiUs = 0;
    qint64 maximumCommandApiUs = 0;
    qint64 latestFullCycleUs = 0;
    double averageFullCycleUs = 0.0;
    qint64 maximumFullCycleUs = 0;
    qint64 latestControlIntervalUs = 0;
    double averageControlIntervalUs = 0.0;
    qint64 maximumControlIntervalUs = 0;
    quint64 commandCount = 0;
    quint64 executionOverrunCount = 0;
    quint64 schedulingOverrunCount = 0;
    quint64 missedCycleCount = 0;
    quint64 droppedRecordCount = 0;
    quint64 latestLogicalFrameSequence = 0;
    OnlineVelocityAxisArray referencePosition{};
    OnlineVelocityAxisArray actualPosition{};
    OnlineVelocityAxisArray referenceVelocity{};
    OnlineVelocityAxisArray actualVelocity{};
    OnlineVelocityAxisArray commandVelocity{};
    OnlineVelocityAxisArray rawPositionError{};
};

class OnlineVelocityCsvRecorder;

class OnlineVelocityControl
{
public:
    OnlineVelocityControl();
    ~OnlineVelocityControl();

    bool prepare(const OnlineVelocityPlan& plan,
                 const OnlineVelocityConfig& config,
                 QString* errorMessage = nullptr);
    bool start(qint64 nowUs, QString* errorMessage = nullptr);
    void resetTraceWaitClock(qint64 nowUs);
    OnlineVelocityStep step(const OnlineVelocityFeedback& feedback, qint64 nowUs);
    void noteCommandResult(const OnlineVelocityStep& step,
                           bool commandOk,
                           qint64 apiDurationUs,
                           qint64 fullCycleDurationUs);
    void stop(bool fault, const QString& reason);
    void finishRecording();

    bool isActive() const;
    bool isPrepared() const;
    const OnlineVelocityPlan& preparedPlan() const;
    const OnlineVelocityConfig& currentConfig() const;
    OnlineVelocityStatus status() const;

private:
    bool feedbackReady(const OnlineVelocityFeedback& feedback) const;
    void interpolate(double timeSec,
                     OnlineVelocityAxisArray& position,
                     OnlineVelocityAxisArray& velocity) const;
    void updateStatusFromStep(const OnlineVelocityStep& step);
    void setTerminalState(OnlineVelocityStatus::State state, const QString& message);

    OnlineVelocityPlan plan;
    OnlineVelocityConfig config;
    OnlineVelocityStatus currentStatus;
    OnlineVelocityAxisArray actualStartPosition{};
    OnlineVelocityAxisArray integral{};
    OnlineVelocityAxisArray lastCommandVelocity{};
    std::array<bool, kOnlineVelocityAxisCount> motionStarted{};
    qint64 waitStartUs = 0;
    qint64 trajectoryStartUs = 0;
    qint64 lastCommandUs = 0;
    qint64 lastFeedbackMonotonicUs = 0;
    qint64 lastGoodTraceUs = 0;
    qint64 nextDueUs = 0;
    quint64 lastUsedFrameSequence = 0;
    quint64 controlIntervalCount = 0;
    qint64 controlIntervalSumUs = 0;
    bool lastUsedFrameSequenceValid = false;
    std::unique_ptr<OnlineVelocityCsvRecorder> recorder;
};

#endif // ONLINEVELOCITYCONTROL_H
