#ifndef EIGHTCABLETENSIONFEEDBACK_H
#define EIGHTCABLETENSIONFEEDBACK_H

#include "forcepid0525.h"
#include "redundanttorqueallocator.h"
#include <QtGlobal>
#include <array>
#include <string>

// No hardware access. The coordinator evaluates a COPY and commits it only
// after a successful eight-axis batch. Original 0525 worker state is separate.
class EightCableTensionFeedback
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Vector8d = RedundantTorqueAllocator::Vector8d;
    // Fixed bound for the short-stroke comparison; no extra PID tuning knob.
    static constexpr double targetBaseSlewLimitNmPerSec = 3.0;
    static constexpr const char* targetBasePolicy = "outer_period_ramp_3nmps_v1";
    static constexpr const char* sharedReferencePolicy = "ramped_nominal_base_reference_v1";
    struct Config {
        bool enabled = false;
        bool independentTargetFeedforward = false; // Off: original total-slew path.
        bool sharedExecutionReference = false; // Requires independent feedforward; frozen per session.
        Vector8d kp = Vector8d::Zero(); // Populated from resolved 0525 parameters.
        Vector8d ki = Vector8d::Zero(), kd = Vector8d::Zero();
        double deadbandRatio = 0.0;
        double torqueLimitNm = 34.5;
        double slewNmPerSec = 6.0;
        qint64 periodUs = 10000; // Fixed catalogue: 5/6/8/10 ms.
        std::string parameterSource;
    };
    struct Diagnostic {
        bool evaluated = false, valid = false, outerUpdated = false, committed = false;
        quint64 logicalFrame = 0, targetVersion = 0;
        qint64 sampleUs = 0, targetUs = 0, dtUs = 0;
        Vector8d targetN = Vector8d::Zero(), measuredN = Vector8d::Zero(), errorN = Vector8d::Zero();
        // targetN/errorN always retain the latest ALLOCATED target for evaluation.
        Vector8d executionTargetN = Vector8d::Zero(), feedbackReferenceN = Vector8d::Zero();
        Vector8d feedbackErrorN = Vector8d::Zero(); // Raw reference minus measurement, before deadband.
        bool sharedExecutionReference = false;
        Vector8d entryTorqueNm = Vector8d::Zero(), entryTensionN = Vector8d::Zero();
        Vector8d baseNm = Vector8d::Zero(), pNm = Vector8d::Zero(), correctionNm = Vector8d::Zero();
        Vector8d requestNm = Vector8d::Zero(), continuousNm = Vector8d::Zero(), commandNm = Vector8d::Zero();
        bool independentTargetFeedforward = false, targetBaseAnchored = false;
        qint64 targetBaseRemainingUs = 0;
        Vector8d appliedBaseNm = Vector8d::Zero(), feedforwardStepNm = Vector8d::Zero();
        Vector8d feedbackStepNm = Vector8d::Zero(), feedbackStateNm = Vector8d::Zero();
        std::array<bool,8> pidClipped{}, signalSlewLimited{}, hardwareLimited{};
    };
    static bool validate(const Config& config, std::string* error = nullptr);
    void initialize(const Vector8d& torqueNm, const Vector8d& tensionN, qint64 sampleUs);
    bool propose(const Config& config, const RedundantTorqueAllocator::Config& bounds,
                 const Vector8d& radius, const Vector8d& target, const Vector8d& measured,
                 const Vector8d& committedTorque, qint64 sampleUs, double quantumNm,
                 Diagnostic& diagnostic, std::string& error,
                 qint64 outerPeriodUs = 0, quint64 targetVersion = 0);
    bool initialized() const { return initialized_; }
    qint64 lastSampleUs() const { return lastSampleUs_; }
private:
    ForcePid0525 pid_;
    bool initialized_ = false;
    qint64 lastSampleUs_ = 0;
    Vector8d entryTorque_ = Vector8d::Zero(), entryTension_ = Vector8d::Zero();
    Vector8d continuousTorque_ = Vector8d::Zero();
    bool targetBaseAnchored_ = false;
    quint64 targetBaseVersion_ = 0;
    Vector8d appliedBase_ = Vector8d::Zero(), targetBaseGoal_ = Vector8d::Zero();
    double targetBaseRemainingSec_ = 0.0;
};
#endif

