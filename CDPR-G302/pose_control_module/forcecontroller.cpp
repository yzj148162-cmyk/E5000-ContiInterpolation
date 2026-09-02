#include "forcecontroller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

double clampValue(double value, double minValue, double maxValue)
{
    return std::min(std::max(value, minValue), maxValue);
}

double sanitizedDt(double dtSec)
{
    if(!std::isfinite(dtSec) || dtSec <= 0.0){
        return 0.001;
    }
    return clampValue(dtSec, 1e-6, 0.1);
}

double sanitizedLimit(double limit)
{
    if(!std::isfinite(limit) || limit < 0.0){
        return 0.0;
    }
    return limit;
}

double moveToward(double value, double target, double maxStep)
{
    if(!std::isfinite(value)){
        value = 0.0;
    }
    if(!std::isfinite(target)){
        target = 0.0;
    }
    if(!std::isfinite(maxStep) || maxStep <= 0.0){
        return target;
    }
    if(value < target){
        return std::min(value + maxStep, target);
    }
    return std::max(value - maxStep, target);
}

int signOf(double value)
{
    if(value > 0.0){
        return 1;
    }
    if(value < 0.0){
        return -1;
    }
    return 0;
}

double smoothStep(double edge0, double edge1, double value)
{
    if(!std::isfinite(value)){
        return 0.0;
    }
    if(edge1 <= edge0){
        return value >= edge1 ? 1.0 : 0.0;
    }
    const double t = clampValue((value - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double platformFuzzyFeedForwardScaleFloor(double error)
{
    if(error >= 0.0){
        return 0.92;
    }

    const double overForceWeight = smoothStep(20.0, 120.0, -error);
    return clampValue(0.90 - 0.08 * overForceWeight, 0.82, 0.92);
}

double fastDescentFuzzyFeedForwardScaleFloor(double error)
{
    if(error >= 0.0){
        return 0.92;
    }

    const double overForceWeight = smoothStep(20.0, 120.0, -error);
    return clampValue(0.90 - 0.04 * overForceWeight, 0.86, 0.92);
}

double feedForwardScaleRecoveryRate(double error,
                                    double motorVelocityRevPerSec,
                                    double measuredDerivative,
                                    double expectedDerivative)
{
    const double absMotorVelocity =
            std::isfinite(motorVelocityRevPerSec) ?
                std::fabs(motorVelocityRevPerSec) :
                0.0;
    const double actualRateAhead = measuredDerivative - expectedDerivative;

    if(error < -5.0){
        const double overshootWeight = smoothStep(5.0, 120.0, -error);
        const double velocityWeight = smoothStep(0.5, 2.5, absMotorVelocity);
        const double recoveryRisk = std::max(overshootWeight, velocityWeight);
        double rate = 0.45 - 0.30 * recoveryRisk;
        if(error < -80.0){
            rate = std::min(rate, 0.12);
        }
        if(actualRateAhead > 100.0){
            rate = std::min(rate, 0.08);
        }
        return clampValue(rate, 0.08, 0.45);
    }

    if(error > 30.0){
        return 2.5;
    }
    if(absMotorVelocity > 1.5){
        return 0.6;
    }
    return 1.2;
}

struct FuzzySupervisorOutput {
    double feedForwardScale = 1.0;
    double kpScale = 1.0;
    double kiScale = 1.0;
    double velocityDampingScale = 1.0;
    double positivePLimit = 0.0;
    double negativePLimit = 0.0;
    int state = 0;
};

enum FuzzyState {
    FuzzyStateNormal = 0,
    FuzzyStateOverForceMovingAway = 1,
    FuzzyStateOverForceRecovering = 2,
    FuzzyStateUnderForceMovingAway = 3,
    FuzzyStateUnderForceRecovering = 4,
    FuzzyStateMotionRiskNearTarget = 5,
    FuzzyStateStableOverForce = 6,
    FuzzyStateStableUnderForce = 7
};

FuzzySupervisorOutput evaluateFuzzySupervisor(double expectedForce,
                                              double measuredForce,
                                              double error,
                                              double feedForwardTorque,
                                              double measuredDerivative,
                                              double expectedDerivative,
                                              double motorVelocityRevPerSec,
                                              bool enabled)
{
    FuzzySupervisorOutput output;
    if(!enabled){
        return output;
    }

    const double forceLevel = std::max(std::fabs(expectedForce), std::fabs(measuredForce));
    const double highForceWeight = smoothStep(160.0, 350.0, forceLevel);
    const double veryHighForceWeight = smoothStep(350.0, 1000.0, forceLevel);
    const double forceSeverityWeight =
            clampValue(highForceWeight + 0.35 * veryHighForceWeight, 0.0, 1.35);
    if(forceSeverityWeight <= 1e-6){
        return output;
    }

    const double measuredRate =
            std::isfinite(measuredDerivative) ? measuredDerivative : 0.0;
    const double expectedRate =
            std::isfinite(expectedDerivative) ? expectedDerivative : 0.0;
    const double actualRateAhead = measuredRate - expectedRate;
    const double absError = std::fabs(error);
    const double absRateAhead = std::fabs(actualRateAhead);
    const double absMotorVelocity =
            std::isfinite(motorVelocityRevPerSec) ?
                std::fabs(motorVelocityRevPerSec) :
                0.0;
    const double errorWeight = smoothStep(5.0, 100.0, absError);
    const double rateWeight = smoothStep(80.0, 2200.0, absRateAhead);
    const double velocityWeight = smoothStep(0.3, 3.0, absMotorVelocity);
    const double motionRisk = std::max(rateWeight, velocityWeight);
    const double lowMotionWeight = 1.0 - clampValue(motionRisk, 0.0, 1.0);
    const double biasErrorWeight = smoothStep(8.0, 80.0, absError);

    const bool actualTooHigh = error < -5.0;
    const bool actualTooLow = error > 5.0;
    const bool actualRisingTooFast = actualRateAhead > 80.0;
    const bool actualFallingTooFast = actualRateAhead < -80.0;
    const bool overForceMovingAway = actualTooHigh && actualRisingTooFast;
    const bool overForceRecovering = actualTooHigh && actualFallingTooFast;
    const bool underForceMovingAway = actualTooLow && actualFallingTooFast;
    const bool underForceRecovering = actualTooLow && actualRisingTooFast;
    const double stableUnderForceWeight =
            actualTooLow ?
                clampValue(forceSeverityWeight *
                           biasErrorWeight *
                           lowMotionWeight,
                           0.0,
                           1.0) :
                0.0;
    const double stableOverForceWeight =
            actualTooHigh ?
                clampValue(forceSeverityWeight *
                           biasErrorWeight *
                           lowMotionWeight,
                           0.0,
                           1.0) :
                0.0;
    const double overForceMovingAwayWeight =
            overForceMovingAway ?
                clampValue(forceSeverityWeight *
                           std::max(errorWeight, rateWeight),
                           0.0,
                           1.35) :
                0.0;
    const double underForceMovingAwayWeight =
            underForceMovingAway ?
                clampValue(forceSeverityWeight *
                           std::max(errorWeight, rateWeight),
                           0.0,
                           1.35) :
                0.0;
    const double overForceRecoveringWeight =
            overForceRecovering ?
                clampValue(forceSeverityWeight * motionRisk, 0.0, 1.35) :
                0.0;
    const double underForceRecoveringWeight =
            underForceRecovering ?
                clampValue(forceSeverityWeight * motionRisk, 0.0, 1.35) :
                0.0;
    const double signedMotionRisk =
            std::max(std::max(overForceMovingAwayWeight, underForceMovingAwayWeight),
                     std::max(overForceRecoveringWeight, underForceRecoveringWeight));
    const double dynamicMotionRisk = forceSeverityWeight * motionRisk;

    if(overForceMovingAway){
        output.state = FuzzyStateOverForceMovingAway;
    }
    else if(overForceRecovering){
        output.state = FuzzyStateOverForceRecovering;
    }
    else if(underForceMovingAway){
        output.state = FuzzyStateUnderForceMovingAway;
    }
    else if(underForceRecovering){
        output.state = FuzzyStateUnderForceRecovering;
    }
    else if(stableOverForceWeight > 0.05){
        output.state = FuzzyStateStableOverForce;
    }
    else if(stableUnderForceWeight > 0.05){
        output.state = FuzzyStateStableUnderForce;
    }
    else if(dynamicMotionRisk > 0.15){
        output.state = FuzzyStateMotionRiskNearTarget;
    }

    if(actualTooHigh){
        const double excessForceWeight = forceSeverityWeight * smoothStep(5.0, 100.0, -error);
        const double movingAwayRisk = overForceMovingAway ?
                    std::max(rateWeight, errorWeight) :
                    0.0;
        const double feedForwardReduction =
                0.08 * excessForceWeight +
                0.18 * excessForceWeight * movingAwayRisk +
                0.05 * velocityWeight * forceSeverityWeight +
                0.08 * veryHighForceWeight * excessForceWeight * movingAwayRisk +
                0.10 * stableOverForceWeight +
                0.20 * overForceMovingAwayWeight;
        const double minimumFeedForwardScale =
                clampValue(0.78 -
                           0.12 * veryHighForceWeight -
                           0.32 * overForceMovingAwayWeight,
                           0.30,
                           1.0);
        output.feedForwardScale =
                clampValue(1.0 - feedForwardReduction,
                           minimumFeedForwardScale,
                           1.0);
    }

    const double minimumKpScale = 0.32 - 0.07 * veryHighForceWeight;
    const double kpRisk = forceSeverityWeight * motionRisk;
    output.kpScale =
            clampValue(1.0 - 0.55 * kpRisk -
                       0.08 * veryHighForceWeight * motionRisk,
                       minimumKpScale,
                       1.0);
    if(stableUnderForceWeight > 0.0){
        output.kpScale =
                std::max(output.kpScale,
                         0.82 - 0.08 * veryHighForceWeight);
    }
    if(stableOverForceWeight > 0.0){
        output.kpScale =
                std::max(output.kpScale,
                         0.62 - 0.08 * veryHighForceWeight);
    }
    if(overForceMovingAwayWeight > 0.0){
        output.kpScale =
                std::max(output.kpScale,
                         0.55 + 0.10 * veryHighForceWeight);
    }
    if(underForceMovingAwayWeight > 0.0){
        output.kpScale =
                std::max(output.kpScale,
                         0.72 - 0.08 * veryHighForceWeight);
    }
    if(underForceRecoveringWeight > 0.0){
        output.kpScale =
                std::min(output.kpScale,
                         0.90 - 0.12 * underForceRecoveringWeight);
    }
    if(overForceRecoveringWeight > 0.0){
        output.kpScale =
                std::min(output.kpScale,
                         0.85 - 0.18 * overForceRecoveringWeight);
    }

    const double integralFreezeWeight = dynamicMotionRisk;
    output.kiScale = clampValue(1.0 - integralFreezeWeight,
                                0.0,
                                1.0);
    if(stableUnderForceWeight > 0.0){
        const double stableUnderForceKiFloor =
                0.35 + 0.35 * stableUnderForceWeight;
        output.kiScale =
                std::max(output.kiScale,
                         clampValue(stableUnderForceKiFloor, 0.35, 0.70));
    }
    if(stableOverForceWeight > 0.0){
        const double stableOverForceKiFloor =
                0.25 + 0.25 * stableOverForceWeight;
        output.kiScale =
                std::max(output.kiScale,
                         clampValue(stableOverForceKiFloor, 0.25, 0.50));
    }
    if(overForceMovingAwayWeight > 0.0 || underForceMovingAwayWeight > 0.0){
        const double movingAwayFreeze =
                std::max(overForceMovingAwayWeight, underForceMovingAwayWeight);
        output.kiScale =
                std::min(output.kiScale,
                         clampValue(0.45 - 0.30 * movingAwayFreeze, 0.0, 0.45));
    }
    if(overForceRecoveringWeight > 0.0 || underForceRecoveringWeight > 0.0){
        const double recoveringFreeze =
                std::max(overForceRecoveringWeight, underForceRecoveringWeight);
        output.kiScale =
                std::min(output.kiScale,
                         clampValue(0.65 - 0.35 * recoveringFreeze, 0.0, 0.65));
    }
    output.velocityDampingScale =
            clampValue(1.0 + 2.0 * dynamicMotionRisk,
                       1.0,
                       3.0 + 0.5 * veryHighForceWeight);

    const double feedForwardRawAbs = std::fabs(feedForwardTorque);
    const double feedForwardAbs = std::fabs(feedForwardTorque * output.feedForwardScale);
    if(output.state != FuzzyStateNormal ||
            signedMotionRisk > 0.05 ||
            stableUnderForceWeight > 0.05 ||
            stableOverForceWeight > 0.05){
        const double highForceLimitScale = 1.0 - 0.25 * veryHighForceWeight;
        double negativePLimit =
                std::max(2.0,
                         feedForwardAbs *
                         highForceLimitScale *
                         0.16);
        if(overForceMovingAwayWeight > 0.0){
            const double highRisingNegativeLimit =
                    feedForwardRawAbs *
                    (0.30 + 0.45 * overForceMovingAwayWeight +
                     0.10 * veryHighForceWeight);
            const double negativeLimitCeiling =
                    std::max(8.0,
                             feedForwardRawAbs *
                             (0.85 + 0.15 * veryHighForceWeight));
            negativePLimit =
                    std::min(std::max(negativePLimit, highRisingNegativeLimit),
                             negativeLimitCeiling);
        }
        else if(stableOverForceWeight > 0.0){
            negativePLimit =
                    std::max(negativePLimit,
                             feedForwardRawAbs *
                             (0.22 + 0.22 * stableOverForceWeight));
        }
        else if(overForceRecoveringWeight > 0.0){
            negativePLimit =
                    std::max(negativePLimit,
                             feedForwardRawAbs *
                             (0.12 + 0.12 * (1.0 - clampValue(motionRisk, 0.0, 1.0))));
        }
        output.negativePLimit =
                negativePLimit;
        double positivePLimit =
                std::max(3.0,
                         feedForwardAbs *
                         highForceLimitScale *
                         0.22);
        if(stableUnderForceWeight > 0.0){
            const double stableUnderForceLimit =
                    std::max(3.0,
                             feedForwardRawAbs *
                             (0.36 + 0.34 * stableUnderForceWeight +
                              0.08 * veryHighForceWeight));
            const double stableUnderForceCeiling =
                    std::max(6.0,
                             feedForwardRawAbs *
                             (0.70 + 0.15 * veryHighForceWeight));
            positivePLimit =
                    std::min(std::max(positivePLimit, stableUnderForceLimit),
                             stableUnderForceCeiling);
        }
        else if(underForceMovingAwayWeight > 0.0){
            const double movingAwayPositiveLimit =
                    feedForwardRawAbs *
                    (0.38 + 0.35 * underForceMovingAwayWeight +
                     0.08 * veryHighForceWeight);
            const double positiveLimitCeiling =
                    std::max(6.0,
                             feedForwardRawAbs *
                             (0.80 + 0.12 * veryHighForceWeight));
            positivePLimit =
                    std::min(std::max(positivePLimit, movingAwayPositiveLimit),
                             positiveLimitCeiling);
        }
        else if(underForceRecoveringWeight > 0.0){
            positivePLimit =
                    std::max(positivePLimit,
                             feedForwardRawAbs *
                             (0.18 + 0.12 * (1.0 - clampValue(motionRisk, 0.0, 1.0))));
        }
        output.positivePLimit = positivePLimit;
    }

    return output;
}

}

void ForceController::ensureSize(int sensorCount)
{
    sensorCount = std::max(sensorCount, 0);
    if(static_cast<int>(states.size()) < sensorCount){
        states.resize(sensorCount);
    }
}

double ForceController::update(int sensorIndex,
                               double measuredForce,
                               double expectedForce,
                               double dtSec,
                               const Params& params)
{
    if(sensorIndex < 0 ||
            !std::isfinite(measuredForce) ||
            !std::isfinite(expectedForce)){
        return 0.0;
    }

    ensureSize(sensorIndex + 1);
    State& state = states[sensorIndex];
    const double dt = sanitizedDt(dtSec);
    const double integralLimit = sanitizedLimit(params.integralLimit);
    const double error = expectedForce - measuredForce;
    const double deadbandAbs =
            std::isfinite(params.deadbandRatio) && params.deadbandRatio > 0.0 ?
                std::fabs(expectedForce) * params.deadbandRatio :
                0.0;

    Debug debug;
    debug.measuredForce = measuredForce;
    debug.expectedForce = expectedForce;
    debug.dtSec = dt;
    debug.error = error;
    debug.feedForwardRawTerm =
            std::isfinite(params.feedForwardTorque) ? params.feedForwardTorque : 0.0;
    debug.feedForwardTerm = debug.feedForwardRawTerm;
    debug.valid = true;

    const double rawMeasuredDerivative =
            state.hasLastMeasuredForce && std::isfinite(state.lastMeasuredForce) ?
                (measuredForce - state.lastMeasuredForce) / dt :
                0.0;
    const double rawExpectedDerivative =
            state.hasLastExpectedForce && std::isfinite(state.lastExpectedForce) ?
                (expectedForce - state.lastExpectedForce) / dt :
                0.0;
    double filteredMeasuredDerivative = rawMeasuredDerivative;
    double filteredExpectedDerivative = rawExpectedDerivative;
    if(state.hasLastMeasuredForce){
        const double tau =
                std::isfinite(params.derivativeLowPassTauSec) &&
                params.derivativeLowPassTauSec > 0.0 ?
                    params.derivativeLowPassTauSec :
                    0.0;
        if(tau > 0.0){
            const double alpha = dt / (tau + dt);
            filteredMeasuredDerivative =
                    state.filteredMeasuredDerivative +
                    alpha * (rawMeasuredDerivative - state.filteredMeasuredDerivative);
            filteredExpectedDerivative =
                    state.filteredExpectedDerivative +
                    alpha * (rawExpectedDerivative - state.filteredExpectedDerivative);
        }
    }
    const double controlExpectedDerivative =
            std::isfinite(params.expectedForceDerivativeNPerSec) ?
                params.expectedForceDerivativeNPerSec :
                filteredExpectedDerivative;
    const double platformCaptureRateThreshold =
            std::isfinite(params.platformCaptureRateThresholdNPerSec) ?
                std::max(0.0, params.platformCaptureRateThresholdNPerSec) :
                0.0;
    const double platformCaptureEnableError =
            std::isfinite(params.platformCaptureEnableErrorN) ?
                std::max(0.0, params.platformCaptureEnableErrorN) :
                0.0;
    const double platformCaptureDisableErrorRaw =
            std::isfinite(params.platformCaptureDisableErrorN) ?
                std::max(0.0, params.platformCaptureDisableErrorN) :
                0.0;
    const double platformCaptureDisableError =
            std::min(platformCaptureDisableErrorRaw, platformCaptureEnableError);
    const double platformCaptureGain =
            std::isfinite(params.platformCaptureGainNmPerN) ?
                std::max(0.0, params.platformCaptureGainNmPerN) :
                0.0;
    const double platformCaptureLimitUp =
            std::isfinite(params.platformCaptureLimitUpNm) ?
                std::max(0.0, params.platformCaptureLimitUpNm) :
                0.0;
    const double platformCaptureLimitDown =
            std::isfinite(params.platformCaptureLimitDownNm) ?
                std::max(0.0, params.platformCaptureLimitDownNm) :
                0.0;
    const double platformCaptureSlewRate =
            std::isfinite(params.platformCaptureSlewRateNmPerSec) ?
                std::max(0.0, params.platformCaptureSlewRateNmPerSec) :
                0.0;
    const double platformCaptureHoldTime =
            std::isfinite(params.platformCaptureHoldTimeSec) ?
                std::max(0.0, params.platformCaptureHoldTimeSec) :
                0.0;
    const double platformCaptureReleaseRate =
            std::isfinite(params.platformCaptureReleaseRateNmPerSec) ?
                std::max(0.0, params.platformCaptureReleaseRateNmPerSec) :
                0.0;
    const double platformCaptureMeasuredRateThreshold =
            std::isfinite(params.platformCaptureMeasuredRateThresholdNPerSec) ?
                std::max(0.0, params.platformCaptureMeasuredRateThresholdNPerSec) :
                0.0;
    const double platformCaptureMeasuredRateHoldTime =
            std::isfinite(params.platformCaptureMeasuredRateHoldTimeSec) ?
                std::max(0.0, params.platformCaptureMeasuredRateHoldTimeSec) :
                0.0;
    const bool platformCaptureConfigured =
            platformCaptureGain > 0.0 &&
            (platformCaptureLimitUp > 0.0 || platformCaptureLimitDown > 0.0);
    const bool platformCaptureDerivativePlatform =
            platformCaptureRateThreshold > 0.0 &&
            std::fabs(controlExpectedDerivative) <= platformCaptureRateThreshold;
    const bool platformCaptureIsPlatform =
            platformCaptureConfigured &&
            (params.platformCaptureUseTrajectoryPlatformFlag ?
                 params.platformCaptureTrajectoryPlatform :
                 platformCaptureDerivativePlatform);
    double controlMeasuredDerivative = filteredMeasuredDerivative;
    const double controlDerivativeLimitParam =
            platformCaptureIsPlatform ?
                params.forceRateControlDerivativePlatformLimitNPerSec :
                params.forceRateControlDerivativeLimitNPerSec;
    const double controlDerivativeLimit =
            std::isfinite(controlDerivativeLimitParam) ?
                std::max(0.0, controlDerivativeLimitParam) :
                0.0;
    if(controlDerivativeLimit > 0.0){
        controlMeasuredDerivative =
                clampValue(controlMeasuredDerivative,
                           -controlDerivativeLimit,
                           controlDerivativeLimit);
    }

    debug.measuredDerivativeRaw = rawMeasuredDerivative;
    debug.measuredDerivativeFiltered = filteredMeasuredDerivative;
    debug.measuredDerivativeControl = controlMeasuredDerivative;
    debug.expectedDerivativeRaw = rawExpectedDerivative;
    debug.expectedDerivativeFiltered = controlExpectedDerivative;
    debug.forceRateError = controlExpectedDerivative - controlMeasuredDerivative;
    const bool expectedRateRising = controlExpectedDerivative >= 0.0;
    const double expectedRateGain = expectedRateRising ?
                params.expectedRateFeedForwardGainUpNmPerNps :
                params.expectedRateFeedForwardGainDownNmPerNps;
    if(std::isfinite(expectedRateGain)){
        double expectedRateFeedForwardScale =
                std::isfinite(params.expectedRateFeedForwardScale) ?
                    clampValue(params.expectedRateFeedForwardScale, 0.0, 1.0) :
                    1.0;
        double expectedRateFeedForwardGateScale = 1.0;
        if(!expectedRateRising && controlExpectedDerivative < 0.0){
            const double minGateScale =
                    std::isfinite(params.expectedRateFeedForwardDownMinScale) ?
                        clampValue(params.expectedRateFeedForwardDownMinScale, 0.0, 1.0) :
                        0.0;
            const double errorGate =
                    std::isfinite(params.expectedRateFeedForwardDownErrorGateN) ?
                        std::max(0.0, params.expectedRateFeedForwardDownErrorGateN) :
                        0.0;
            if(errorGate > 0.0 && error > errorGate){
                const double errorWeight =
                        smoothStep(errorGate, errorGate + std::max(10.0, errorGate), error);
                expectedRateFeedForwardGateScale =
                        std::min(expectedRateFeedForwardGateScale,
                                 1.0 - (1.0 - minGateScale) * errorWeight);
            }

            const double fastDropGate =
                    std::isfinite(params.expectedRateFeedForwardDownFastDropGateNPerSec) ?
                        std::max(0.0, params.expectedRateFeedForwardDownFastDropGateNPerSec) :
                        0.0;
            const double actualFallingAhead =
                    controlExpectedDerivative - controlMeasuredDerivative;
            if(fastDropGate > 0.0 && actualFallingAhead > fastDropGate){
                const double fastDropWeight =
                        smoothStep(fastDropGate,
                                   fastDropGate + std::max(100.0, fastDropGate),
                                   actualFallingAhead);
                expectedRateFeedForwardGateScale =
                        std::min(expectedRateFeedForwardGateScale,
                                 1.0 - (1.0 - minGateScale) * fastDropWeight);
            }
        }
        expectedRateFeedForwardScale *= expectedRateFeedForwardGateScale;
        debug.expectedRateFeedForwardScale = expectedRateFeedForwardScale;
        debug.expectedRateFeedForwardGateScale = expectedRateFeedForwardGateScale;
        debug.expectedRateFeedForwardTerm =
                expectedRateGain * controlExpectedDerivative *
                expectedRateFeedForwardScale;
    }
    const double expectedRateLimitParam = expectedRateRising ?
                params.expectedRateFeedForwardLimitUpNm :
                params.expectedRateFeedForwardLimitDownNm;
    const double expectedRateLimit =
            std::isfinite(expectedRateLimitParam) ?
                std::max(0.0, expectedRateLimitParam) :
                0.0;
    if(expectedRateLimit > 0.0){
        debug.expectedRateFeedForwardTerm =
                clampValue(debug.expectedRateFeedForwardTerm,
                           -expectedRateLimit,
                           expectedRateLimit);
    }
    const double forceRateErrorDeadband =
            std::isfinite(params.forceRateErrorDeadbandNPerSec) ?
                std::max(0.0, params.forceRateErrorDeadbandNPerSec) :
                0.0;
    double forceRateErrorForDamping = 0.0;
    double forceRateErrorDampingGain = 0.0;
    double forceRateErrorDampingLimit = 0.0;
    if(debug.forceRateError > forceRateErrorDeadband){
        forceRateErrorForDamping =
                debug.forceRateError - forceRateErrorDeadband;
        if(error >= 0.0){
            forceRateErrorDampingGain =
                    params.forceRateBelowExpectedCatchUpGainNmPerNps;
            forceRateErrorDampingLimit =
                    params.forceRateBelowExpectedCatchUpLimitNm;
        }
        else{
            forceRateErrorDampingGain =
                    params.forceRateAboveExpectedRecoverGainNmPerNps;
            forceRateErrorDampingLimit =
                    params.forceRateAboveExpectedRecoverLimitNm;
        }
    }
    else if(debug.forceRateError < -forceRateErrorDeadband){
        forceRateErrorForDamping =
                debug.forceRateError + forceRateErrorDeadband;
        if(error >= 0.0){
            forceRateErrorDampingGain =
                    params.forceRateBelowExpectedBrakeGainNmPerNps;
            forceRateErrorDampingLimit =
                    params.forceRateBelowExpectedBrakeLimitNm;
        }
        else{
            forceRateErrorDampingGain =
                    params.forceRateAboveExpectedUnloadGainNmPerNps;
            forceRateErrorDampingLimit =
                    params.forceRateAboveExpectedUnloadLimitNm;
        }
    }
    forceRateErrorDampingGain =
            std::isfinite(forceRateErrorDampingGain) ?
                std::max(0.0, forceRateErrorDampingGain) :
                0.0;
    forceRateErrorDampingLimit =
            std::isfinite(forceRateErrorDampingLimit) ?
                std::max(0.0, forceRateErrorDampingLimit) :
                0.0;
    debug.forceRateErrorDampingTerm =
            forceRateErrorDampingGain * forceRateErrorForDamping;
    if(forceRateErrorDampingLimit > 0.0){
        debug.forceRateErrorDampingTerm =
                clampValue(debug.forceRateErrorDampingTerm,
                           -forceRateErrorDampingLimit,
                           forceRateErrorDampingLimit);
    }

    const auto clampedCaptureTarget =
            [platformCaptureLimitDown, platformCaptureLimitUp](double value) -> double {
        return clampValue(value, -platformCaptureLimitDown, platformCaptureLimitUp);
    };

    if(!platformCaptureConfigured){
        state.platformCaptureActive = false;
        state.platformCaptureWasPlatform = false;
        state.platformCaptureSettledTimeSec = 0.0;
        state.platformCaptureMeasuredRateStableTimeSec = 0.0;
        state.platformCaptureTargetTorque = 0.0;
        state.platformCaptureTorque = 0.0;
    }
    else if(platformCaptureIsPlatform){
        const bool justEnteredPlatform = !state.platformCaptureWasPlatform;
        if(justEnteredPlatform){
            state.platformCaptureActive = false;
            state.platformCaptureSettledTimeSec = 0.0;
            state.platformCaptureMeasuredRateStableTimeSec = 0.0;
            state.platformCaptureTargetTorque = 0.0;
        }

        const bool measuredRateGateEnabled =
                platformCaptureMeasuredRateThreshold > 0.0 &&
                platformCaptureMeasuredRateHoldTime > 0.0;
        const bool measuredRateStable =
                !measuredRateGateEnabled ||
                std::fabs(filteredMeasuredDerivative) <= platformCaptureMeasuredRateThreshold;
        if(measuredRateGateEnabled && measuredRateStable){
            state.platformCaptureMeasuredRateStableTimeSec += dt;
        }
        else if(measuredRateGateEnabled){
            state.platformCaptureMeasuredRateStableTimeSec = 0.0;
        }
        else{
            state.platformCaptureMeasuredRateStableTimeSec = platformCaptureMeasuredRateHoldTime;
        }
        const bool captureUpdateAllowed =
                !measuredRateGateEnabled ||
                (measuredRateStable &&
                 state.platformCaptureMeasuredRateStableTimeSec >=
                 platformCaptureMeasuredRateHoldTime);
        const double absError = std::fabs(error);
        const bool shouldCapture =
                absError >= platformCaptureEnableError &&
                platformCaptureEnableError > 0.0;
        const int errorSign = signOf(error);
        const int targetSign = signOf(state.platformCaptureTargetTorque);
        const bool reverseCorrection =
                state.platformCaptureActive &&
                errorSign != 0 &&
                targetSign != 0 &&
                errorSign != targetSign &&
                absError > platformCaptureDisableError;

        if(!captureUpdateAllowed){
            state.platformCaptureActive = false;
            state.platformCaptureSettledTimeSec = 0.0;
            state.platformCaptureTargetTorque = 0.0;
        }
        else if((justEnteredPlatform && shouldCapture) ||
                (!state.platformCaptureActive && shouldCapture) ||
                reverseCorrection){
            const double incrementalTarget =
                    state.platformCaptureTorque +
                    platformCaptureGain * error;
            state.platformCaptureTargetTorque =
                    clampedCaptureTarget(incrementalTarget);
            state.platformCaptureActive = true;
            state.platformCaptureSettledTimeSec = 0.0;
        }
        else if(state.platformCaptureActive){
            if(absError <= platformCaptureDisableError){
                state.platformCaptureSettledTimeSec += dt;
                if(state.platformCaptureSettledTimeSec >= platformCaptureHoldTime){
                    state.platformCaptureActive = false;
                    state.platformCaptureTargetTorque = 0.0;
                }
            }
            else{
                state.platformCaptureSettledTimeSec = 0.0;
            }
        }

        if(state.platformCaptureActive){
            const double maxStep = platformCaptureSlewRate > 0.0 ?
                        platformCaptureSlewRate * dt :
                        std::numeric_limits<double>::infinity();
            state.platformCaptureTorque =
                    moveToward(state.platformCaptureTorque,
                               state.platformCaptureTargetTorque,
                               maxStep);
        }
        else{
            const double maxReleaseStep = platformCaptureReleaseRate > 0.0 ?
                        platformCaptureReleaseRate * dt :
                        std::numeric_limits<double>::infinity();
            state.platformCaptureTorque =
                    moveToward(state.platformCaptureTorque, 0.0, maxReleaseStep);
        }
        state.platformCaptureWasPlatform = true;
    }
    else{
        state.platformCaptureActive = false;
        state.platformCaptureSettledTimeSec = 0.0;
        state.platformCaptureMeasuredRateStableTimeSec = 0.0;
        state.platformCaptureTargetTorque = 0.0;
        const double maxReleaseStep = platformCaptureReleaseRate > 0.0 ?
                    platformCaptureReleaseRate * dt :
                    std::numeric_limits<double>::infinity();
        state.platformCaptureTorque =
                moveToward(state.platformCaptureTorque, 0.0, maxReleaseStep);
        state.platformCaptureWasPlatform = false;
    }

    if(!std::isfinite(state.platformCaptureTorque)){
        state.platformCaptureTorque = 0.0;
    }
    debug.platformCaptureTerm = state.platformCaptureTorque;
    debug.platformCaptureTargetTerm = state.platformCaptureTargetTorque;
    if(platformCaptureIsPlatform && state.platformCaptureActive){
        debug.platformCaptureState = 1;
    }
    else if(platformCaptureIsPlatform &&
            std::fabs(state.platformCaptureTorque) > 1e-9){
        debug.platformCaptureState = 2;
    }
    else if(!platformCaptureIsPlatform &&
            std::fabs(state.platformCaptureTorque) > 1e-9){
        debug.platformCaptureState = 3;
    }

    if(deadbandAbs > 0.0 && std::fabs(error) < deadbandAbs){
        state.integral = 0.0;
        state.lastMeasuredForce = measuredForce;
        state.lastExpectedForce = expectedForce;
        state.filteredMeasuredDerivative = 0.0;
        state.filteredExpectedDerivative = 0.0;
        state.feedForwardScale = 1.0;
        state.hasLastMeasuredForce = true;
        state.hasLastExpectedForce = true;
        state.hasFeedForwardScale = false;
        state.hasPendingIntegral = false;
        debug.inDeadband = true;
        debug.integral = state.integral;
        debug.integralCandidate = state.integral;
        debug.requestedFeedbackTorque =
                debug.feedForwardTerm +
                debug.expectedRateFeedForwardTerm +
                debug.forceRateErrorDampingTerm +
                debug.platformCaptureTerm;
        state.latestDebug = debug;
        return debug.requestedFeedbackTorque;
    }

    state.lastMeasuredForce = measuredForce;
    state.lastExpectedForce = expectedForce;
    state.filteredMeasuredDerivative = filteredMeasuredDerivative;
    state.filteredExpectedDerivative = filteredExpectedDerivative;
    state.hasLastMeasuredForce = true;
    state.hasLastExpectedForce = true;

    FuzzySupervisorOutput fuzzy =
            evaluateFuzzySupervisor(expectedForce,
                                    measuredForce,
                                    error,
                                    debug.feedForwardRawTerm,
                                    controlMeasuredDerivative,
                                    controlExpectedDerivative,
                                    params.motorVelocityRevPerSec,
                                    params.fuzzySupervisorEnabled);
    const double forceLevel =
            std::max(std::fabs(expectedForce), std::fabs(measuredForce));
    const bool clampPlatformFuzzyFeedForward =
            platformCaptureIsPlatform &&
            params.fuzzySupervisorEnabled &&
            forceLevel >= 120.0;
    const bool clampFastDescentFuzzyFeedForward =
            !platformCaptureIsPlatform &&
            params.fuzzySupervisorEnabled &&
            forceLevel >= 300.0 &&
            controlExpectedDerivative < -80.0;
    double fuzzyFeedForwardScaleFloor = 0.0;
    if(clampPlatformFuzzyFeedForward){
        fuzzyFeedForwardScaleFloor =
                platformFuzzyFeedForwardScaleFloor(error);
    }
    if(clampFastDescentFuzzyFeedForward){
        fuzzyFeedForwardScaleFloor =
                std::max(fuzzyFeedForwardScaleFloor,
                         fastDescentFuzzyFeedForwardScaleFloor(error));
    }
    if(fuzzyFeedForwardScaleFloor > 0.0){
        fuzzy.feedForwardScale =
                std::max(fuzzy.feedForwardScale,
                         fuzzyFeedForwardScaleFloor);
    }
    debug.fuzzyFeedForwardTargetScale = fuzzy.feedForwardScale;
    debug.fuzzyKpScale = fuzzy.kpScale;
    debug.fuzzyKiScale = fuzzy.kiScale;
    debug.fuzzyVelocityDampingScale = fuzzy.velocityDampingScale;
    debug.fuzzyPositivePLimit = fuzzy.positivePLimit;
    debug.fuzzyNegativePLimit = fuzzy.negativePLimit;
    debug.fuzzyState = fuzzy.state;
    debug.fuzzyFeedForwardRecoveryRate =
            feedForwardScaleRecoveryRate(error,
                                         params.motorVelocityRevPerSec,
                                         controlMeasuredDerivative,
                                         controlExpectedDerivative);
    if(!params.fuzzySupervisorEnabled || forceLevel < 120.0){
        state.feedForwardScale = fuzzy.feedForwardScale;
        state.hasFeedForwardScale = true;
    }
    else if(!state.hasFeedForwardScale ||
            !std::isfinite(state.feedForwardScale)){
        state.feedForwardScale = fuzzy.feedForwardScale;
        state.hasFeedForwardScale = true;
    }
    else if(fuzzy.feedForwardScale < state.feedForwardScale){
        const double configuredDropRate =
                std::isfinite(params.fuzzyFeedForwardDropRatePerSec) ?
                    std::max(0.0, params.fuzzyFeedForwardDropRatePerSec) :
                    0.0;
        const double configuredFastDescentDropRate =
                std::isfinite(params.fuzzyFeedForwardFastDescentDropRatePerSec) ?
                    std::max(0.0, params.fuzzyFeedForwardFastDescentDropRatePerSec) :
                    0.0;
        const double dropRate =
                clampFastDescentFuzzyFeedForward &&
                configuredFastDescentDropRate > 0.0 ?
                    configuredFastDescentDropRate :
                    configuredDropRate;
        if(dropRate > 0.0){
            const double nextScale =
                    std::max(fuzzy.feedForwardScale,
                             state.feedForwardScale - dropRate * dt);
            debug.fuzzyFeedForwardRecoveryLimited =
                    nextScale > fuzzy.feedForwardScale + 1e-9;
            state.feedForwardScale = nextScale;
        }
        else{
            state.feedForwardScale = fuzzy.feedForwardScale;
        }
    }
    else{
        const double maxRecovery =
                debug.fuzzyFeedForwardRecoveryRate * dt;
        const double nextScale =
                std::min(fuzzy.feedForwardScale,
                         state.feedForwardScale + maxRecovery);
        debug.fuzzyFeedForwardRecoveryLimited =
                nextScale + 1e-9 < fuzzy.feedForwardScale;
        state.feedForwardScale = nextScale;
    }
    state.feedForwardScale =
            clampValue(state.feedForwardScale, 0.0, 1.0);
    if(fuzzyFeedForwardScaleFloor > 0.0){
        state.feedForwardScale =
                std::max(state.feedForwardScale,
                         fuzzyFeedForwardScaleFloor);
    }
    debug.fuzzyFeedForwardScale = state.feedForwardScale;
    debug.feedForwardTerm =
            debug.feedForwardRawTerm * debug.fuzzyFeedForwardScale;

    const bool baseIntegralEnabled =
            std::isfinite(params.ki) && std::fabs(params.ki) > 1e-12;
    const bool integralUpdateEnabled =
            baseIntegralEnabled && fuzzy.kiScale > 1e-9;
    double integralCandidate =
            baseIntegralEnabled ? state.integral : 0.0;
    if(integralUpdateEnabled){
        integralCandidate += error * dt * fuzzy.kiScale;
    }
    if(baseIntegralEnabled){
        if(integralLimit > 0.0){
            integralCandidate = clampValue(integralCandidate, -integralLimit, integralLimit);
        }
        else{
            integralCandidate = 0.0;
        }
    }

    debug.integral = state.integral;
    debug.integralCandidate = integralCandidate;
    const double releaseRateThreshold =
            std::isfinite(params.integralReleaseExpectedRateThresholdNPerSec) ?
                std::max(0.0, params.integralReleaseExpectedRateThresholdNPerSec) :
                0.0;
    const double releaseErrorThreshold =
            std::isfinite(params.integralReleaseOverForceThresholdN) ?
                std::max(0.0, params.integralReleaseOverForceThresholdN) :
                0.0;
    const bool releasePositiveIntegral =
            baseIntegralEnabled &&
            releaseRateThreshold > 0.0 &&
            controlExpectedDerivative <= -releaseRateThreshold &&
            error <= -releaseErrorThreshold &&
            integralCandidate > 0.0;
    if(releasePositiveIntegral){
        const double releaseTau =
                std::isfinite(params.integralReleaseTimeConstantSec) ?
                    params.integralReleaseTimeConstantSec :
                    0.0;
        if(releaseTau <= 0.0){
            integralCandidate = 0.0;
        }
        else{
            integralCandidate *= std::exp(-dt / releaseTau);
        }
        debug.integralReleaseApplied = true;
        debug.integralCandidate = integralCandidate;
    }
    debug.pTerm = params.kp * fuzzy.kpScale * error;
    if(fuzzy.positivePLimit > 0.0 && debug.pTerm > fuzzy.positivePLimit){
        debug.pTerm = fuzzy.positivePLimit;
        debug.fuzzyPLimitApplied = true;
    }
    if(fuzzy.negativePLimit > 0.0 && debug.pTerm < -fuzzy.negativePLimit){
        debug.pTerm = -fuzzy.negativePLimit;
        debug.fuzzyPLimitApplied = true;
    }
    debug.iTerm = params.ki * integralCandidate;
    debug.dTerm = -params.kd * filteredMeasuredDerivative;
    debug.requestedFeedbackTorque =
            debug.feedForwardTerm +
            debug.expectedRateFeedForwardTerm +
            debug.forceRateErrorDampingTerm +
            debug.platformCaptureTerm +
            debug.pTerm +
            debug.iTerm +
            debug.dTerm;

    if(!std::isfinite(debug.requestedFeedbackTorque)){
        resetChannel(sensorIndex);
        return 0.0;
    }

    state.hasPendingIntegral = true;
    state.latestDebug = debug;
    return debug.requestedFeedbackTorque;
}

void ForceController::commit(int sensorIndex,
                             double appliedFeedbackTorque,
                             bool outputLimited,
                             const Params& params)
{
    if(sensorIndex < 0 || sensorIndex >= static_cast<int>(states.size())){
        return;
    }

    State& state = states[sensorIndex];
    Debug debug = state.latestDebug;
    if(!debug.valid){
        return;
    }

    debug.appliedFeedbackTorque =
            std::isfinite(appliedFeedbackTorque) ? appliedFeedbackTorque : 0.0;
    debug.outputLimited = outputLimited;

    const bool integralEnabled =
            std::isfinite(params.ki) && std::fabs(params.ki) > 1e-12;
    if(state.hasPendingIntegral){
        double committedIntegral = debug.integralCandidate;
        if(outputLimited &&
                integralEnabled &&
                std::isfinite(debug.appliedFeedbackTorque)){
            const double knownTerms =
                    debug.feedForwardTerm +
                    debug.expectedRateFeedForwardTerm +
                    debug.forceRateErrorDampingTerm +
                    debug.platformCaptureTerm +
                    debug.pTerm +
                    debug.dTerm;
            committedIntegral =
                    (debug.appliedFeedbackTorque - knownTerms) / params.ki;
            const double integralLimit = sanitizedLimit(params.integralLimit);
            if(integralLimit > 0.0){
                committedIntegral = clampValue(committedIntegral, -integralLimit, integralLimit);
            }
            else{
                committedIntegral = 0.0;
            }
            debug.antiWindupAdjusted = true;
        }

        if(!std::isfinite(committedIntegral)){
            committedIntegral = state.integral;
        }
        const double integralLimit = sanitizedLimit(params.integralLimit);
        if(integralEnabled && integralLimit <= 0.0){
            committedIntegral = 0.0;
        }
        state.integral = integralEnabled ? committedIntegral : 0.0;
        state.hasPendingIntegral = false;
    }

    debug.integral = state.integral;
    debug.iTerm = params.ki * state.integral;
    state.latestDebug = debug;
}

void ForceController::ensureChannelCount(int sensorCount)
{
    ensureSize(sensorCount);
}

void ForceController::reset(int sensorCount)
{
    sensorCount = std::max(sensorCount, 0);
    states.assign(sensorCount, State());
}

void ForceController::resetChannel(int sensorIndex)
{
    if(sensorIndex < 0){
        return;
    }
    ensureSize(sensorIndex + 1);
    states[sensorIndex] = State();
}

ForceController::Debug ForceController::debug(int sensorIndex) const
{
    if(sensorIndex < 0 || sensorIndex >= static_cast<int>(states.size())){
        return Debug();
    }
    return states[sensorIndex].latestDebug;
}

std::vector<ForceController::Debug> ForceController::debugSnapshot() const
{
    std::vector<Debug> snapshot;
    snapshot.reserve(states.size());
    for(const State& state : states){
        snapshot.push_back(state.latestDebug);
    }
    return snapshot;
}
