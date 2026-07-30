#include "control/PositionVelocityPid.h"

#include <algorithm>
#include <cmath>

namespace {
double clampSymmetric(double value, double limit)
{
    const double safeLimit = std::max(0.0, limit);
    return std::clamp(value, -safeLimit, safeLimit);
}
}

void PositionVelocityPid::reset(double initialCommandVelocity)
{
    integralState_ = 0.0;
    previousCommandVelocity_ = initialCommandVelocity;
}

PositionVelocityPidOutput PositionVelocityPid::update(
    const VelocityControlConfig &config,
    double positionErrorDegree,
    double feedforwardVelocityDegreePerSecond,
    double feedbackReferenceVelocityDegreePerSecond,
    double actualVelocityDegreePerSecond,
    double commandDtSeconds,
    double feedbackDtSeconds,
    bool feedbackFresh)
{
    PositionVelocityPidOutput output;
    const double commandDt = std::max(1e-6, commandDtSeconds);
    const double feedbackDt = std::max(1e-6, feedbackDtSeconds);
    output.feedforward = config.velocityFeedforwardEnabled
        ? config.velocityFeedforwardGain * feedforwardVelocityDegreePerSecond : 0.0;

    const double previousIntegral = integralState_;
    if (config.pidEnabled) {
        if (feedbackFresh) {
            integralState_ = clampSymmetric(
                integralState_ + positionErrorDegree * feedbackDt,
                config.integralLimitDegreeSecond);
        } else {
            output.integralFrozen = true;
        }
        output.p = config.kp * positionErrorDegree;
        output.i = config.ki * integralState_;
        output.d = config.kd
            * (feedbackReferenceVelocityDegreePerSecond
               - actualVelocityDegreePerSecond);
    }

    const auto calculateRaw = [&] {
        const double correction = clampSymmetric(output.p + output.i + output.d,
                                                  config.maxPidCorrectionDegreePerSecond);
        return output.feedforward + correction;
    };
    output.rawVelocity = calculateRaw();
    double velocityLimited = clampSymmetric(output.rawVelocity,
                                             config.maxVelocityDegreePerSecond);
    output.velocitySaturated = std::abs(velocityLimited - output.rawVelocity) > 1e-12;

    if (output.velocitySaturated && config.pidEnabled
        && positionErrorDegree * output.rawVelocity > 0.0) {
        integralState_ = previousIntegral;
        output.i = config.ki * integralState_;
        output.integralFrozen = true;
        output.rawVelocity = calculateRaw();
        velocityLimited = clampSymmetric(output.rawVelocity,
                                         config.maxVelocityDegreePerSecond);
        output.velocitySaturated = std::abs(velocityLimited - output.rawVelocity) > 1e-12;
    }

    const double maximumStep =
        std::max(0.0, config.maxAccelerationDegreePerSecond2) * commandDt;
    const double step = std::clamp(velocityLimited - previousCommandVelocity_,
                                   -maximumStep, maximumStep);
    output.commandVelocity = previousCommandVelocity_ + step;
    output.accelerationLimited = std::abs(output.commandVelocity - velocityLimited) > 1e-12;
    previousCommandVelocity_ = output.commandVelocity;
    return output;
}
