#ifndef FORCECONTROLLER_H
#define FORCECONTROLLER_H

#include <vector>

class ForceController
{
public:
    struct Params {
        double feedForwardTorque = 0.0;
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double deadbandRatio = 0.0;
        double integralLimit = 200.0;
        double derivativeLowPassTauSec = 0.01;
        double motorVelocityRevPerSec = 0.0;
        double expectedForceDerivativeNPerSec = 0.0;
        double expectedRateFeedForwardGainUpNmPerNps = 0.0;
        double expectedRateFeedForwardGainDownNmPerNps = 0.0;
        double expectedRateFeedForwardLimitUpNm = 0.0;
        double expectedRateFeedForwardLimitDownNm = 0.0;
        double expectedRateFeedForwardScale = 1.0;
        double expectedRateFeedForwardDownErrorGateN = 0.0;
        double expectedRateFeedForwardDownFastDropGateNPerSec = 0.0;
        double expectedRateFeedForwardDownMinScale = 0.0;
        double forceRateControlDerivativeLimitNPerSec = 0.0;
        double forceRateControlDerivativePlatformLimitNPerSec = 0.0;
        double forceRateErrorDeadbandNPerSec = 0.0;
        double forceRateBelowExpectedCatchUpGainNmPerNps = 0.0;
        double forceRateBelowExpectedCatchUpLimitNm = 0.0;
        double forceRateBelowExpectedBrakeGainNmPerNps = 0.0;
        double forceRateBelowExpectedBrakeLimitNm = 0.0;
        double forceRateAboveExpectedUnloadGainNmPerNps = 0.0;
        double forceRateAboveExpectedUnloadLimitNm = 0.0;
        double forceRateAboveExpectedRecoverGainNmPerNps = 0.0;
        double forceRateAboveExpectedRecoverLimitNm = 0.0;
        double integralReleaseExpectedRateThresholdNPerSec = 0.0;
        double integralReleaseOverForceThresholdN = 0.0;
        double integralReleaseTimeConstantSec = 0.0;
        double platformCaptureRateThresholdNPerSec = 0.0;
        double platformCaptureEnableErrorN = 0.0;
        double platformCaptureDisableErrorN = 0.0;
        double platformCaptureGainNmPerN = 0.0;
        double platformCaptureLimitUpNm = 0.0;
        double platformCaptureLimitDownNm = 0.0;
        double platformCaptureSlewRateNmPerSec = 0.0;
        double platformCaptureHoldTimeSec = 0.0;
        double platformCaptureReleaseRateNmPerSec = 0.0;
        double platformCaptureMeasuredRateThresholdNPerSec = 0.0;
        double platformCaptureMeasuredRateHoldTimeSec = 0.0;
        double fuzzyFeedForwardDropRatePerSec = 0.0;
        double fuzzyFeedForwardFastDescentDropRatePerSec = 0.0;
        bool platformCaptureUseTrajectoryPlatformFlag = false;
        bool platformCaptureTrajectoryPlatform = false;
        bool fuzzySupervisorEnabled = true;
    };

    struct Debug {
        double measuredForce = 0.0;
        double expectedForce = 0.0;
        double dtSec = 0.0;
        double error = 0.0;
        double feedForwardRawTerm = 0.0;
        double feedForwardTerm = 0.0;
        double expectedRateFeedForwardTerm = 0.0;
        double forceRateError = 0.0;
        double forceRateErrorDampingTerm = 0.0;
        double platformCaptureTerm = 0.0;
        double platformCaptureTargetTerm = 0.0;
        // 0 off, 1 capturing on platform, 2 releasing on platform, 3 releasing off platform.
        int platformCaptureState = 0;
        double pTerm = 0.0;
        double iTerm = 0.0;
        double dTerm = 0.0;
        double integral = 0.0;
        double integralCandidate = 0.0;
        double measuredDerivativeRaw = 0.0;
        double measuredDerivativeFiltered = 0.0;
        double measuredDerivativeControl = 0.0;
        double expectedRateFeedForwardScale = 1.0;
        double expectedRateFeedForwardGateScale = 1.0;
        double expectedDerivativeRaw = 0.0;
        double expectedDerivativeFiltered = 0.0;
        double fuzzyFeedForwardTargetScale = 1.0;
        double fuzzyFeedForwardScale = 1.0;
        double fuzzyFeedForwardRecoveryRate = 0.0;
        double fuzzyKpScale = 1.0;
        double fuzzyKiScale = 1.0;
        double fuzzyVelocityDampingScale = 1.0;
        double fuzzyPositivePLimit = 0.0;
        double fuzzyNegativePLimit = 0.0;
        // 0 normal/low risk, 1 over-force moving away, 2 over-force recovering,
        // 3 under-force moving away, 4 under-force recovering, 5 motion risk near target,
        // 6 stable over-force bias, 7 stable under-force bias.
        int fuzzyState = 0;
        bool fuzzyFeedForwardRecoveryLimited = false;
        bool fuzzyPLimitApplied = false;
        bool integralReleaseApplied = false;
        double requestedFeedbackTorque = 0.0;
        double appliedFeedbackTorque = 0.0;
        bool inDeadband = false;
        bool outputLimited = false;
        bool antiWindupAdjusted = false;
        bool valid = false;
    };

    double update(int sensorIndex,
                  double measuredForce,
                  double expectedForce,
                  double dtSec,
                  const Params& params);
    void commit(int sensorIndex,
                double appliedFeedbackTorque,
                bool outputLimited,
                const Params& params);
    void ensureChannelCount(int sensorCount);
    void reset(int sensorCount = 0);
    void resetChannel(int sensorIndex);
    Debug debug(int sensorIndex) const;
    std::vector<Debug> debugSnapshot() const;

private:
    struct State {
        double integral = 0.0;
        double lastMeasuredForce = 0.0;
        double lastExpectedForce = 0.0;
        double filteredMeasuredDerivative = 0.0;
        double filteredExpectedDerivative = 0.0;
        double feedForwardScale = 1.0;
        double platformCaptureTorque = 0.0;
        double platformCaptureTargetTorque = 0.0;
        double platformCaptureSettledTimeSec = 0.0;
        double platformCaptureMeasuredRateStableTimeSec = 0.0;
        bool hasLastMeasuredForce = false;
        bool hasLastExpectedForce = false;
        bool hasFeedForwardScale = false;
        bool hasPendingIntegral = false;
        bool platformCaptureActive = false;
        bool platformCaptureWasPlatform = false;
        Debug latestDebug;
    };

    void ensureSize(int sensorCount);

    std::vector<State> states;
};

#endif // FORCECONTROLLER_H
