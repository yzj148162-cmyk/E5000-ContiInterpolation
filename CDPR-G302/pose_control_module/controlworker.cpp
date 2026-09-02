/*
 * 文件总览：
 * - ControlWorker 的实现文件，包含控制循环、传感器采样节流、PID 力矩计算、限幅/斜率约束和诊断历史缓存。
 * - 主循环先刷新反馈和期望力，再判断安全/模式条件，最后只对启用的力控轴写入力矩命令。
 * - 文件中的时间戳统一使用 steady clock 微秒值，避免系统时间跳变影响控制周期判断。
 */

#include "controlworker.h"
#include "runtimefeatureswitches.h"

#include <QDateTime>
#include <QElapsedTimer>
#include <QThread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iterator>
#include <limits>
#include <utility>

namespace {

// 返回单调时钟微秒时间戳，用于控制周期和超时判断。
qint64 monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

QString endpointRemoteVelocityCommandOutcomeText(
        HardwareInterface::EndpointRemoteVelocityCommandOutcome outcome)
{
    using Outcome = HardwareInterface::EndpointRemoteVelocityCommandOutcome;
    switch(outcome){
    case Outcome::NotAttempted:
        return QStringLiteral("未执行");
    case Outcome::Succeeded:
        return QStringLiteral("成功");
    case Outcome::FreshFrameDeferred:
        return QStringLiteral("静止态等待下一新鲜帧");
    case Outcome::SafetyContextRejected:
        return QStringLiteral("同帧安全上下文拒绝");
    case Outcome::CommandValidationRejected:
        return QStringLiteral("命令参数/状态拒绝");
    case Outcome::SoftwareLimitRejected:
        return QStringLiteral("软件限位拒绝");
    case Outcome::SdkFailure:
        return QStringLiteral("板卡SDK失败");
    case Outcome::HardwareThreadDispatchFailed:
        return QStringLiteral("HardwareThread任务未执行");
    case Outcome::InternalFailure:
        return QStringLiteral("内部未归因失败");
    }
    return QStringLiteral("未知");
}

OnlineVelocityFeedback onlineVelocityFeedbackFromTraceSnapshot(
        const HardwareInterface::RuntimeTraceSnapshot& traceSnapshot)
{
    OnlineVelocityFeedback feedback;
    feedback.wallClockUs = traceSnapshot.wallClockUs;
    feedback.monotonicUs = traceSnapshot.monotonicUs;
    feedback.newestFrameAgeUs = traceSnapshot.newestFrameAgeUs;
    feedback.frameCount = traceSnapshot.frameCount;
    feedback.fifoValidNum = traceSnapshot.fifoValidNum;
    feedback.fifoFreeNum = traceSnapshot.fifoFreeNum;
    feedback.traceSamplePeriodUs = traceSnapshot.traceSamplePeriodUs;
    feedback.logicalFrameSequence = traceSnapshot.logicalFrameSequence;
    feedback.fromTrace = traceSnapshot.fromTrace;
    feedback.frameSequenceValid = traceSnapshot.frameSequenceValid;
    feedback.timingReliable = traceSnapshot.timingReliable;
    feedback.fifoCaughtUp = traceSnapshot.fifoCaughtUp;
    feedback.traceLost = traceSnapshot.traceLost;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    feedback.actualPosition.fill(nan);
    feedback.actualVelocity.fill(nan);
    feedback.tracedCommandVelocity.fill(nan);
    feedback.motorStatusWord.fill(0);
    feedback.motorStateMachine.fill(-1);
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(axis < static_cast<int>(traceSnapshot.motorPosition.size())){
            feedback.actualPosition[axis] = traceSnapshot.motorPosition[axis];
        }
        if(axis < static_cast<int>(traceSnapshot.motorActualVelocity.size())){
            feedback.actualVelocity[axis] = traceSnapshot.motorActualVelocity[axis];
        }
        if(axis < static_cast<int>(traceSnapshot.motorCommandVelocity.size())){
            feedback.tracedCommandVelocity[axis] =
                    traceSnapshot.motorCommandVelocity[axis];
        }
        if(axis < static_cast<int>(traceSnapshot.motorStatusWord.size())){
            feedback.motorStatusWord[axis] =
                    traceSnapshot.motorStatusWord[axis];
        }
        if(axis < static_cast<int>(traceSnapshot.motorStateMachine.size())){
            feedback.motorStateMachine[axis] =
                    traceSnapshot.motorStateMachine[axis];
        }
    }
    return feedback;
}

constexpr double kForcePid0525IntegralLimit = 200.0;
constexpr double kForcePid0525MaxDtSec = 0.010;
constexpr double kForceControllerDerivativeLowPassTauSec = 0.01;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr qint64 kForceSensorTraceMinIntervalUs = 500;
constexpr qint64 kForceSensorTraceMaxIntervalUs = 1000;
constexpr qint64 kForcePid0525OpenLoopWorkerIntervalUs = 5 * 1000;
constexpr std::size_t kForcePidTraceTransportCapacity = 10000;
constexpr bool kEnableControlWorkerDiagnosticRawHistory =
        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;
constexpr double kDefaultForceTorqueCommandLimitNm = 60.0;
constexpr double kDefaultForceTorqueCommandSlewRateNmPerSec = 40.0;
constexpr double kTorqueCommandEpsilonNm = 1e-6;
// PVT completion performs a few blocking hardware reads; keep torque hold alive
// across that handoff while still stopping on a real feedback dropout.
constexpr qint64 kTorqueForceSensorTimeoutUs = 50 * 1000;
constexpr qint64 kDiagnosticRawDefaultRetentionMs = 30 * 1000;
constexpr qint64 kDiagnosticRawTrimIntervalMs = 1000;
constexpr qint64 kDiagnosticRawDefaultSampleIntervalUs = 50 * 1000;
constexpr int kDiagnosticRawDefaultMaxSamples = 30000;
constexpr int kDiagnosticSensorValueDefaultMaxSamples = 6000;
constexpr qint64 kMotorHomeRefreshIntervalUs = 250 * 1000;
constexpr qint64 kActualTorqueLimitContinuousOverUs = 120 * 1000;
constexpr qint64 kActualTorqueLimitMinContinuityGapUs = 50 * 1000;

double smoothStep01(double edge0, double edge1, double value)
{
    if(!std::isfinite(value)){
        return 0.0;
    }
    if(edge1 <= edge0){
        return value >= edge0 ? 1.0 : 0.0;
    }
    const double t = std::min(1.0, std::max(0.0, (value - edge0) / (edge1 - edge0)));
    return t * t * (3.0 - 2.0 * t);
}

double firstOrderLowPass(double previous, double input, double dtSec, double cutoffHz)
{
    if(!std::isfinite(input)){
        return std::isfinite(previous) ? previous : 0.0;
    }
    if(!std::isfinite(previous)){
        previous = input;
    }
    if(!std::isfinite(dtSec) || dtSec <= 0.0 ||
            !std::isfinite(cutoffHz) || cutoffHz <= 0.0){
        return input;
    }
    const double tau = 1.0 / (kTwoPi * cutoffHz);
    const double alpha = std::min(1.0, std::max(0.0, dtSec / (tau + dtSec)));
    return previous + alpha * (input - previous);
}

double applySignedDeadband(double value, double deadband)
{
    if(!std::isfinite(value)){
        return 0.0;
    }
    if(!std::isfinite(deadband) || deadband <= 0.0){
        return value;
    }
    if(value > deadband){
        return value - deadband;
    }
    if(value < -deadband){
        return value + deadband;
    }
    return 0.0;
}

int forceSlopeSign(double deltaForce)
{
    constexpr double kForceSlopeSignEpsilonN = 1e-6;
    if(deltaForce > kForceSlopeSignEpsilonN){
        return 1;
    }
    if(deltaForce < -kForceSlopeSignEpsilonN){
        return -1;
    }
    return 0;
}

bool sameForceSlopeDirection(const std::vector<double>& sensorTraj,
                             int intervalIndex,
                             int referenceSign)
{
    if(intervalIndex < 0 ||
            intervalIndex + 1 >= static_cast<int>(sensorTraj.size()) ||
            referenceSign == 0){
        return false;
    }
    return forceSlopeSign(sensorTraj[intervalIndex + 1] - sensorTraj[intervalIndex]) ==
            referenceSign;
}

double expectedRateFeedForwardFadeScale(const std::vector<double>& sensorTraj,
                                        const std::vector<double>& timeStamp,
                                        int lowerIndex,
                                        int upperIndex,
                                        double trajectoryTimeSec,
                                        double configuredFadeTimeSec)
{
    if(!std::isfinite(configuredFadeTimeSec) ||
            configuredFadeTimeSec <= 0.0 ||
            lowerIndex < 0 ||
            upperIndex <= lowerIndex ||
            upperIndex >= static_cast<int>(sensorTraj.size()) ||
            upperIndex >= static_cast<int>(timeStamp.size())){
        return 1.0;
    }

    const int referenceSign =
            forceSlopeSign(sensorTraj[upperIndex] - sensorTraj[lowerIndex]);
    if(referenceSign == 0){
        return 1.0;
    }

    int segmentStartIndex = lowerIndex;
    while(segmentStartIndex > 0 &&
          sameForceSlopeDirection(sensorTraj, segmentStartIndex - 1, referenceSign)){
        --segmentStartIndex;
    }

    int segmentEndIndex = upperIndex;
    const int lastIntervalIndex =
            std::min(static_cast<int>(sensorTraj.size()),
                     static_cast<int>(timeStamp.size())) - 2;
    while(segmentEndIndex <= lastIntervalIndex &&
          sameForceSlopeDirection(sensorTraj, segmentEndIndex, referenceSign)){
        ++segmentEndIndex;
    }

    const double segmentStartTime = timeStamp[segmentStartIndex];
    const double segmentEndTime = timeStamp[segmentEndIndex];
    const double segmentDuration = segmentEndTime - segmentStartTime;
    if(!std::isfinite(segmentDuration) || segmentDuration <= 0.0){
        return 1.0;
    }

    const double effectiveFadeTime =
            std::min(configuredFadeTimeSec, segmentDuration * 0.25);
    if(effectiveFadeTime <= 0.0){
        return 1.0;
    }

    const double timeFromSegmentStart =
            std::max(0.0, trajectoryTimeSec - segmentStartTime);
    const double timeToSegmentEnd =
            std::max(0.0, segmentEndTime - trajectoryTimeSec);
    return smoothStep01(0.0, effectiveFadeTime, timeFromSegmentStart) *
            smoothStep01(0.0, effectiveFadeTime, timeToSegmentEnd);
}

double effectiveForceFeedForwardScale(double expectedForceN,
                                      double feedForwardScale,
                                      double highTensionStartN,
                                      double highTensionFullN,
                                      double highTensionAddScale)
{
    if(!std::isfinite(feedForwardScale)){
        return 0.0;
    }
    const double baseScale = std::max(0.0, feedForwardScale);
    if(!std::isfinite(expectedForceN) ||
            !std::isfinite(highTensionAddScale) ||
            highTensionAddScale <= 0.0){
        return baseScale;
    }

    const double startN =
            std::isfinite(highTensionStartN) ? std::max(0.0, highTensionStartN) : 0.0;
    const double fullN =
            std::isfinite(highTensionFullN) ? std::max(0.0, highTensionFullN) : startN;
    const double forceLevel = std::fabs(expectedForceN);
    const double weight = smoothStep01(startN, fullN, forceLevel);
    return baseScale + std::max(0.0, highTensionAddScale) * weight;
}

double forceFeedForwardTorqueNm(double expectedForceN,
                                double drumRadiusMm,
                                double feedForwardScale,
                                double highTensionStartN,
                                double highTensionFullN,
                                double highTensionAddScale)
{
    const double radiusMm = std::abs(drumRadiusMm);
    if(!std::isfinite(expectedForceN) ||
            !std::isfinite(radiusMm) ||
            radiusMm <= 0.0 ||
            !std::isfinite(feedForwardScale)){
        return 0.0;
    }

    const double effectiveScale =
            effectiveForceFeedForwardScale(expectedForceN,
                                           feedForwardScale,
                                           highTensionStartN,
                                           highTensionFullN,
                                           highTensionAddScale);
    return effectiveScale * expectedForceN * radiusMm / 1000.0;
}

double pureOpenLoopForceTorqueNm(double expectedForceN,
                                 double drumRadiusMm)
{
    const double radiusMm = std::abs(drumRadiusMm);
    if(!std::isfinite(expectedForceN) ||
            !std::isfinite(radiusMm) ||
            radiusMm <= 0.0){
        return 0.0;
    }
    return expectedForceN * radiusMm / 1000.0;
}

double feedForwardMotionSign(double angularVelocityRadPerSec,
                             double angularAccelerationRadPerSec2,
                             double velocityDeadbandRadPerSec)
{
    const double velocity =
            std::isfinite(angularVelocityRadPerSec) ? angularVelocityRadPerSec : 0.0;
    const double acceleration =
            std::isfinite(angularAccelerationRadPerSec2) ? angularAccelerationRadPerSec2 : 0.0;
    const double deadband =
            std::isfinite(velocityDeadbandRadPerSec) ?
                std::max(0.0, velocityDeadbandRadPerSec) :
                0.0;
    if(deadband > 0.0 && std::fabs(velocity) <= deadband){
        if(std::fabs(acceleration) > 1e-9){
            return acceleration > 0.0 ? 1.0 : -1.0;
        }
        return 0.0;
    }
    if(deadband > 0.0){
        return std::tanh(velocity / deadband);
    }
    if(velocity > 0.0){
        return 1.0;
    }
    if(velocity < 0.0){
        return -1.0;
    }
    if(acceleration > 0.0){
        return 1.0;
    }
    if(acceleration < 0.0){
        return -1.0;
    }
    return 0.0;
}

struct FeedForwardOnlyTerms {
    double staticForceTerm = 0.0;
    double staticFrictionTerm = 0.0;
    double frictionTerm = 0.0;
    double velocityTerm = 0.0;
    double accelerationTerm = 0.0;
    double totalTerm = 0.0;
    int selectedDynamicProfile = 0;
    double staticFrictionDirection = 0.0;
    double staticFrictionSpeedScale = 0.0;
    double staticFrictionRawTerm = 0.0;
    double staticFrictionAfterFadeTerm = 0.0;
    double staticFrictionAfterSmoothTerm = 0.0;
};

struct FeedForwardOnlyStaticFrictionTerms {
    double direction = 0.0;
    double speedScale = 0.0;
    double rawTerm = 0.0;
    double afterFadeTerm = 0.0;
    double afterSmoothTerm = 0.0;
};

struct FeedForwardOnlyRuntimeParams {
    ControlWorker::ForcePid0525DynamicTrackMode dynamicTrackMode =
            ControlWorker::ForcePid0525DynamicTrackMode::FeedForwardOnly;
    bool useBangBangPretension = false;
    double frictionCoulombNm = 0.0;
    double frictionViscousNmPerRadPerSec = 0.0;
    double frictionVelocityDeadbandRadPerSec = 0.05;
    bool staticFrictionEnabled = false;
    double staticFrictionScale = 1.0;
    double staticFrictionForceRateDeadbandNPerSec = 1.0;
    double staticFrictionVelocityFadeStartRadPerSec = 0.02;
    double staticFrictionVelocityFadeEndRadPerSec = 0.20;
    bool staticFrictionMechanicalDirectionEnabled = false;
    bool staticFrictionExitBlendEnabled = false;
    double staticFrictionExitBlendTimeConstantSec = 0.0;
    double inertiaScale = 0.0;
    double winchInertiaKgM2 = 0.00468288508;
    double motorInertiaKgM2 = 0.01135;
    double trackTorqueSlewRateNmPerSec = 40.0;
    double trackBlendTimeSec = 0.15;
    int selectedDynamicProfile = 0;
};

struct StaticFrictionPoint {
    double forceN;
    double residualForceN;
};

FeedForwardOnlyRuntimeParams feedForwardOnlyRuntimeParams(
        const ControlWorker::Config& cfg,
        int sensorIndex,
        double expectedForceRateNPerSec)
{
    FeedForwardOnlyRuntimeParams params;
    params.dynamicTrackMode = cfg.forcePid0525DynamicTrackMode;
    params.useBangBangPretension = cfg.forcePid0525UseBangBangPretension;
    params.frictionCoulombNm = cfg.forceFeedForwardOnlyFrictionCoulombNm;
    params.frictionViscousNmPerRadPerSec =
            cfg.forceFeedForwardOnlyFrictionViscousNmPerRadPerSec;
    params.frictionVelocityDeadbandRadPerSec =
            cfg.forceFeedForwardOnlyFrictionVelocityDeadbandRadPerSec;
    params.staticFrictionEnabled =
            cfg.forceFeedForwardOnlyStaticFrictionEnabled;
    params.staticFrictionScale =
            cfg.forceFeedForwardOnlyStaticFrictionScale;
    params.staticFrictionForceRateDeadbandNPerSec =
            cfg.forceFeedForwardOnlyStaticFrictionForceRateDeadbandNPerSec;
    params.staticFrictionVelocityFadeStartRadPerSec =
            cfg.forceFeedForwardOnlyStaticFrictionVelocityFadeStartRadPerSec;
    params.staticFrictionVelocityFadeEndRadPerSec =
            cfg.forceFeedForwardOnlyStaticFrictionVelocityFadeEndRadPerSec;
    params.inertiaScale = cfg.forceFeedForwardOnlyInertiaScale;
    params.winchInertiaKgM2 = cfg.forceFeedForwardOnlyWinchInertiaKgM2;
    params.motorInertiaKgM2 = cfg.forceFeedForwardOnlyMotorInertiaKgM2;
    params.trackTorqueSlewRateNmPerSec =
            cfg.forcePid0525TrackTorqueSlewRateNmPerSec;
    params.trackBlendTimeSec = cfg.forcePid0525TrackBlendTimeSec;

    const bool useUpProfile =
            !std::isfinite(expectedForceRateNPerSec) ||
            expectedForceRateNPerSec >= 0.0;
    params.selectedDynamicProfile = useUpProfile ? 1 : -1;
    const std::vector<ControlWorker::ForceFeedForwardOnlyDirectionalProfile>& profiles =
            useUpProfile ?
                cfg.forceFeedForwardOnlyUpProfiles :
                cfg.forceFeedForwardOnlyDownProfiles;
    if(sensorIndex < 0 || sensorIndex >= static_cast<int>(profiles.size()) ||
            !profiles[sensorIndex].enabled){
        params.selectedDynamicProfile = 0;
        return params;
    }

    const ControlWorker::ForceFeedForwardOnlyDirectionalProfile& profile =
            profiles[sensorIndex];
    params.dynamicTrackMode = profile.dynamicTrackMode;
    params.useBangBangPretension = profile.useBangBangPretension;
    params.frictionCoulombNm = profile.frictionCoulombNm;
    params.frictionViscousNmPerRadPerSec =
            profile.frictionViscousNmPerRadPerSec;
    params.frictionVelocityDeadbandRadPerSec =
            profile.frictionVelocityDeadbandRadPerSec;
    params.staticFrictionEnabled = profile.staticFrictionEnabled;
    params.staticFrictionScale = profile.staticFrictionScale;
    params.staticFrictionForceRateDeadbandNPerSec =
            profile.staticFrictionForceRateDeadbandNPerSec;
    params.staticFrictionVelocityFadeStartRadPerSec =
            profile.staticFrictionVelocityFadeStartRadPerSec;
    params.staticFrictionVelocityFadeEndRadPerSec =
            profile.staticFrictionVelocityFadeEndRadPerSec;
    params.staticFrictionMechanicalDirectionEnabled =
            profile.staticFrictionMechanicalDirectionEnabled;
    params.staticFrictionExitBlendEnabled =
            profile.staticFrictionExitBlendEnabled;
    params.staticFrictionExitBlendTimeConstantSec =
            profile.staticFrictionExitBlendTimeConstantSec;
    params.inertiaScale = profile.inertiaScale;
    params.trackTorqueSlewRateNmPerSec =
            profile.trackTorqueSlewRateNmPerSec;
    params.trackBlendTimeSec = profile.trackBlendTimeSec;
    return params;
}

double feedForwardOnlyDirectionalProfileSelectionRate(
        std::vector<int>& profileSigns,
        int sensorIndex,
        double expectedForceRateNPerSec,
        double forceRateDeadbandNPerSec)
{
    if(sensorIndex < 0){
        return std::isfinite(expectedForceRateNPerSec) ?
                    expectedForceRateNPerSec :
                    1.0;
    }
    if(static_cast<int>(profileSigns.size()) <= sensorIndex){
        profileSigns.resize(sensorIndex + 1, 1);
    }

    const double deadband =
            std::isfinite(forceRateDeadbandNPerSec) ?
                std::max(0.0, forceRateDeadbandNPerSec) :
                1.0;
    if(std::isfinite(expectedForceRateNPerSec)){
        if(expectedForceRateNPerSec > deadband){
            profileSigns[sensorIndex] = 1;
        }
        else if(expectedForceRateNPerSec < -deadband){
            profileSigns[sensorIndex] = -1;
        }
    }
    return profileSigns[sensorIndex] >= 0 ? 1.0 : -1.0;
}

double interpolateStaticFrictionResidualForce(
        double forceN,
        const StaticFrictionPoint* points,
        int pointCount)
{
    if(!std::isfinite(forceN) || !points || pointCount <= 0){
        return 0.0;
    }
    if(forceN <= points[0].forceN){
        return points[0].residualForceN;
    }
    if(forceN >= points[pointCount - 1].forceN){
        return points[pointCount - 1].residualForceN;
    }
    for(int index=1; index<pointCount; ++index){
        if(forceN <= points[index].forceN){
            const double span = points[index].forceN - points[index - 1].forceN;
            if(span <= 1e-9){
                return points[index].residualForceN;
            }
            const double ratio = (forceN - points[index - 1].forceN) / span;
            return points[index - 1].residualForceN +
                    ratio * (points[index].residualForceN -
                             points[index - 1].residualForceN);
        }
    }
    return points[pointCount - 1].residualForceN;
}

double staticFrictionDirectionSign(double expectedForceRateNPerSec,
                                   double angularVelocityRadPerSec,
                                   double angularAccelerationRadPerSec2,
                                   const FeedForwardOnlyRuntimeParams& params,
                                   int sensorIndex,
                                   std::vector<int>* lastDirectionSigns)
{
    const double forceRateDeadband =
            std::isfinite(params.staticFrictionForceRateDeadbandNPerSec) ?
                std::max(0.0,
                         params.staticFrictionForceRateDeadbandNPerSec) :
                0.0;
    if(params.staticFrictionMechanicalDirectionEnabled && sensorIndex >= 0){
        if(lastDirectionSigns &&
                static_cast<int>(lastDirectionSigns->size()) <= sensorIndex){
            lastDirectionSigns->resize(sensorIndex + 1, 0);
        }
        auto remember = [sensorIndex, lastDirectionSigns](double sign) -> double {
            if(std::fabs(sign) <= 1e-6){
                return 0.0;
            }
            const int storedSign = sign > 0.0 ? 1 : -1;
            if(lastDirectionSigns &&
                    sensorIndex >= 0 &&
                    sensorIndex < static_cast<int>(lastDirectionSigns->size())){
                (*lastDirectionSigns)[sensorIndex] = storedSign;
            }
            return static_cast<double>(storedSign);
        };

        const double velocityDeadband =
                std::isfinite(params.frictionVelocityDeadbandRadPerSec) ?
                    std::max(0.0, params.frictionVelocityDeadbandRadPerSec) :
                    0.0;
        if(std::isfinite(angularVelocityRadPerSec) &&
                std::fabs(angularVelocityRadPerSec) > velocityDeadband){
            return remember(angularVelocityRadPerSec);
        }
        if(lastDirectionSigns &&
                sensorIndex < static_cast<int>(lastDirectionSigns->size()) &&
                (*lastDirectionSigns)[sensorIndex] != 0){
            return (*lastDirectionSigns)[sensorIndex] > 0 ? 1.0 : -1.0;
        }
        if(std::isfinite(angularAccelerationRadPerSec2) &&
                std::fabs(angularAccelerationRadPerSec2) > 1e-6){
            return remember(angularAccelerationRadPerSec2);
        }
        if(std::isfinite(expectedForceRateNPerSec) &&
                std::fabs(expectedForceRateNPerSec) > forceRateDeadband){
            return remember(expectedForceRateNPerSec);
        }
        return 0.0;
    }

    if(std::isfinite(expectedForceRateNPerSec) &&
            std::fabs(expectedForceRateNPerSec) > forceRateDeadband){
        return expectedForceRateNPerSec > 0.0 ? 1.0 : -1.0;
    }
    return feedForwardMotionSign(
                angularVelocityRadPerSec,
                angularAccelerationRadPerSec2,
                params.frictionVelocityDeadbandRadPerSec);
}

double staticFrictionSpeedScale(double angularVelocityRadPerSec,
                                const FeedForwardOnlyRuntimeParams& params)
{
    const double speed = std::fabs(
                std::isfinite(angularVelocityRadPerSec) ?
                    angularVelocityRadPerSec :
                    0.0);
    const double fadeStart =
            std::isfinite(params.staticFrictionVelocityFadeStartRadPerSec) ?
                std::max(0.0,
                         params.staticFrictionVelocityFadeStartRadPerSec) :
                0.0;
    const double fadeEnd =
            std::isfinite(params.staticFrictionVelocityFadeEndRadPerSec) ?
                std::max(0.0,
                         params.staticFrictionVelocityFadeEndRadPerSec) :
                fadeStart;
    if(fadeEnd <= fadeStart + 1e-9){
        return speed <= fadeStart ? 1.0 : 0.0;
    }
    return 1.0 - smoothStep01(fadeStart, fadeEnd, speed);
}

double applyStaticFrictionExitBlend(double targetTorqueNm,
                                    double dtSec,
                                    const FeedForwardOnlyRuntimeParams& params,
                                    int sensorIndex,
                                    std::vector<double>* smoothedTorqueNm,
                                    std::vector<bool>* smoothedValid)
{
    if(!params.staticFrictionExitBlendEnabled ||
            sensorIndex < 0 ||
            !smoothedTorqueNm ||
            !smoothedValid){
        return targetTorqueNm;
    }
    const double timeConstant =
            std::isfinite(params.staticFrictionExitBlendTimeConstantSec) ?
                std::max(0.0, params.staticFrictionExitBlendTimeConstantSec) :
                0.0;
    if(timeConstant <= 1e-9){
        return targetTorqueNm;
    }
    if(static_cast<int>(smoothedTorqueNm->size()) <= sensorIndex){
        smoothedTorqueNm->resize(sensorIndex + 1, 0.0);
    }
    if(static_cast<int>(smoothedValid->size()) <= sensorIndex){
        smoothedValid->resize(sensorIndex + 1, false);
    }

    double& previous = (*smoothedTorqueNm)[sensorIndex];
    const bool valid = (*smoothedValid)[sensorIndex];
    if(!valid || !std::isfinite(previous)){
        previous = targetTorqueNm;
        (*smoothedValid)[sensorIndex] = true;
        return targetTorqueNm;
    }
    const double dt =
            std::isfinite(dtSec) && dtSec > 0.0 ?
                std::min(dtSec, 0.1) :
                0.0;
    if(dt <= 0.0){
        previous = targetTorqueNm;
        return targetTorqueNm;
    }

    const bool signChanging =
            previous * targetTorqueNm < -1e-9;
    const bool magnitudeDecreasing =
            std::fabs(targetTorqueNm) + 1e-9 < std::fabs(previous);
    if(signChanging || magnitudeDecreasing){
        const double alpha = 1.0 - std::exp(-dt / timeConstant);
        previous += std::min(std::max(alpha, 0.0), 1.0) *
                (targetTorqueNm - previous);
    }
    else{
        previous = targetTorqueNm;
    }
    return previous;
}

FeedForwardOnlyStaticFrictionTerms feedForwardOnlyStaticFrictionTorqueNm(
        double expectedForceN,
        double drumRadiusMm,
        double expectedForceRateNPerSec,
        double expectedAngularVelocityRadPerSec,
        double expectedAngularAccelerationRadPerSec2,
        const FeedForwardOnlyRuntimeParams& params,
        int sensorIndex,
        double dtSec,
        std::vector<int>* lastDirectionSigns,
        std::vector<double>* smoothedTorqueNm,
        std::vector<bool>* smoothedValid)
{
    FeedForwardOnlyStaticFrictionTerms terms;
    if(!params.staticFrictionEnabled){
        if(sensorIndex >= 0 && smoothedValid &&
                sensorIndex < static_cast<int>(smoothedValid->size())){
            (*smoothedValid)[sensorIndex] = false;
        }
        return terms;
    }
    const double direction =
            staticFrictionDirectionSign(
                expectedForceRateNPerSec,
                expectedAngularVelocityRadPerSec,
                expectedAngularAccelerationRadPerSec2,
                params,
                sensorIndex,
                lastDirectionSigns);
    terms.direction = direction;
    if(std::fabs(direction) <= 1e-6){
        terms.afterSmoothTerm =
                applyStaticFrictionExitBlend(0.0,
                                             dtSec,
                                             params,
                                             sensorIndex,
                                             smoothedTorqueNm,
                                             smoothedValid);
        return terms;
    }
    const double scale =
            std::isfinite(params.staticFrictionScale) ?
                std::max(0.0, params.staticFrictionScale) :
                0.0;
    if(scale <= 0.0){
        terms.afterSmoothTerm =
                applyStaticFrictionExitBlend(0.0,
                                             dtSec,
                                             params,
                                             sensorIndex,
                                             smoothedTorqueNm,
                                             smoothedValid);
        return terms;
    }

    // From 前馈标定记录.xlsx:
    // residualForce = measuredMotorTorque / 0.043 - actualForce.
    // Increasing and decreasing static sweeps are kept separate to preserve
    // hysteresis direction.
    static const StaticFrictionPoint kIncreasingTable[] = {
        {37.511000000000003, 9.0006279069767459},
        {78.0, 15.023255813953497},
        {129.0, 10.534883720930253},
        {164.0, 22.046511627906995},
        {210.42599999999999, 22.132139534883748},
        {243.0, 36.069767441860506},
        {285.0, 40.581395348837248},
        {325.0, 47.093023255813989},
        {362.0, 56.604651162790731},
        {404.0, 61.116279069767472},
        {447.51100000000002, 64.11690697674419},
        {482.0, 76.139534883721012},
        {521.0, 83.651162790697754},
        {559.0, 92.162790697674495},
        {599.0, 98.674418604651237},
        {638.0, 106.18604651162798},
        {670.0, 120.69767441860472},
        {715.0, 122.20930232558146}
    };
    static const StaticFrictionPoint kDecreasingTable[] = {
        {51.398000000000003, -4.8863720930232546},
        {99.006, -5.9827441860465029},
        {149.37299999999999, -9.8381162790697374},
        {188.554, -2.5074883720930075},
        {235.29400000000001, -2.735860465116275},
        {279.00099999999998, 0.068767441860529743},
        {320.75599999999997, 4.8253953488372758},
        {368.48099999999999, 3.6120232558139946},
        {410.58600000000001, 8.018651162790718},
        {449.20499999999998, 15.911279069767488},
        {494.52699999999999, 17.100906976744227},
        {534.25, 23.889534883721012},
        {576.90899999999999, 27.742162790697762},
        {613.29399999999998, 37.868790697674513},
        {645.19899999999996, 52.475418604651281},
        {677.42499999999995, 66.761046511628024},
        {704.0, 86.69767441860472}
    };
    const StaticFrictionPoint* table =
            direction > 0.0 ? kIncreasingTable : kDecreasingTable;
    const int tableCount =
            direction > 0.0 ?
                static_cast<int>(sizeof(kIncreasingTable) / sizeof(kIncreasingTable[0])) :
                static_cast<int>(sizeof(kDecreasingTable) / sizeof(kDecreasingTable[0]));
    const double residualForceN =
            interpolateStaticFrictionResidualForce(
                expectedForceN,
                table,
                tableCount);
    const double speedScale =
            staticFrictionSpeedScale(expectedAngularVelocityRadPerSec, params);
    const double radiusM =
            std::isfinite(drumRadiusMm) ?
                std::max(0.0, drumRadiusMm) / 1000.0 :
                0.0;
    terms.speedScale = speedScale;
    terms.rawTerm = scale * residualForceN * radiusM;
    terms.afterFadeTerm = terms.rawTerm * speedScale;
    terms.afterSmoothTerm =
            applyStaticFrictionExitBlend(terms.afterFadeTerm,
                                         dtSec,
                                         params,
                                         sensorIndex,
                                         smoothedTorqueNm,
                                         smoothedValid);
    return terms;
}

FeedForwardOnlyTerms feedForwardOnlyTorqueTerms(double expectedForceN,
                                                double drumRadiusMm,
                                                double expectedForceRateNPerSec,
                                                double expectedAngularVelocityRadPerSec,
                                                double expectedAngularAccelerationRadPerSec2,
                                                const FeedForwardOnlyRuntimeParams& params,
                                                int sensorIndex,
                                                double dtSec,
                                                std::vector<int>* lastStaticFrictionDirectionSigns,
                                                std::vector<double>* staticFrictionSmoothedTorqueNm,
                                                std::vector<bool>* staticFrictionSmoothedValid)
{
    FeedForwardOnlyTerms terms;
    terms.selectedDynamicProfile = params.selectedDynamicProfile;
    terms.staticForceTerm =
            pureOpenLoopForceTorqueNm(expectedForceN, drumRadiusMm);

    const double coulombNm =
            std::isfinite(params.frictionCoulombNm) ?
                std::max(0.0, params.frictionCoulombNm) :
                0.0;
    const double viscousNmPerRadPerSec =
            std::isfinite(params.frictionViscousNmPerRadPerSec) ?
                std::max(0.0, params.frictionViscousNmPerRadPerSec) :
                0.0;
    const double velocity =
            std::isfinite(expectedAngularVelocityRadPerSec) ?
                expectedAngularVelocityRadPerSec :
                0.0;
    const double acceleration =
            std::isfinite(expectedAngularAccelerationRadPerSec2) ?
                expectedAngularAccelerationRadPerSec2 :
                0.0;
    const double motionSign =
            feedForwardMotionSign(velocity,
                                  acceleration,
                                  params.frictionVelocityDeadbandRadPerSec);
    const FeedForwardOnlyStaticFrictionTerms staticFrictionTerms =
            feedForwardOnlyStaticFrictionTorqueNm(
                expectedForceN,
                drumRadiusMm,
                expectedForceRateNPerSec,
                velocity,
                acceleration,
                params,
                sensorIndex,
                dtSec,
                lastStaticFrictionDirectionSigns,
                staticFrictionSmoothedTorqueNm,
                staticFrictionSmoothedValid);
    terms.staticFrictionTerm = staticFrictionTerms.afterSmoothTerm;
    terms.staticFrictionDirection = staticFrictionTerms.direction;
    terms.staticFrictionSpeedScale = staticFrictionTerms.speedScale;
    terms.staticFrictionRawTerm = staticFrictionTerms.rawTerm;
    terms.staticFrictionAfterFadeTerm = staticFrictionTerms.afterFadeTerm;
    terms.staticFrictionAfterSmoothTerm = staticFrictionTerms.afterSmoothTerm;
    terms.frictionTerm = coulombNm * motionSign + terms.staticFrictionTerm;
    terms.velocityTerm = viscousNmPerRadPerSec * velocity;

    const double inertiaScale =
            std::isfinite(params.inertiaScale) ?
                std::max(0.0, params.inertiaScale) :
                0.0;
    const double winchInertia =
            std::isfinite(params.winchInertiaKgM2) ?
                std::max(0.0, params.winchInertiaKgM2) :
                0.0;
    const double motorInertia =
            std::isfinite(params.motorInertiaKgM2) ?
                std::max(0.0, params.motorInertiaKgM2) :
                0.0;
    terms.accelerationTerm =
            inertiaScale * (winchInertia + motorInertia) * acceleration;
    terms.totalTerm =
            terms.staticForceTerm +
            terms.frictionTerm +
            terms.velocityTerm +
            terms.accelerationTerm;
    return terms;
}

double moveTowardValue(double value, double target, double maxStep)
{
    if(!std::isfinite(value)){
        return target;
    }
    if(!std::isfinite(target)){
        return value;
    }
    const double step = std::max(0.0, maxStep);
    if(value < target){
        return std::min(value + step, target);
    }
    if(value > target){
        return std::max(value - step, target);
    }
    return value;
}

double blendedForceFeedForwardTorqueNm(double expectedForceN,
                                       double drumRadiusMm,
                                       const ControlWorker::Config& cfg,
                                       double unloadBlend)
{
    const double normalTorque =
            forceFeedForwardTorqueNm(expectedForceN,
                                     drumRadiusMm,
                                     cfg.forceFeedForwardScale,
                                     cfg.forceHighTensionFeedForwardStartN,
                                     cfg.forceHighTensionFeedForwardFullN,
                                     cfg.forceHighTensionFeedForwardAddScale);
    const double unloadTorque =
            forceFeedForwardTorqueNm(expectedForceN,
                                     drumRadiusMm,
                                     cfg.forceUnloadFeedForwardScale,
                                     cfg.forceHighTensionFeedForwardStartN,
                                     cfg.forceHighTensionFeedForwardFullN,
                                     cfg.forceUnloadHighTensionFeedForwardAddScale);
    const double blend =
            std::isfinite(unloadBlend) ? std::min(std::max(unloadBlend, 0.0), 1.0) : 0.0;
    return normalTorque + (unloadTorque - normalTorque) * blend;
}

// 将力传感器 Trace 周期压到硬件可稳定切换的两个档位。
qint64 boundedForceSensorTraceIntervalUs(int intervalUs)
{
    const int switchMidpointUs =
            static_cast<int>((kForceSensorTraceMinIntervalUs +
                              kForceSensorTraceMaxIntervalUs) / 2);
    return intervalUs < switchMidpointUs ?
                kForceSensorTraceMinIntervalUs :
                kForceSensorTraceMaxIntervalUs;
}

// 根据配置计算 worker 定时器周期，使循环跟随 Trace 采样频率。
qint64 traceDrivenWorkerIntervalUs(const ControlWorker::Config& cfg)
{
    if(cfg.forcePidOutputMode == ControlWorker::ForcePidOutputMode::Pid0525){
        return kForcePid0525OpenLoopWorkerIntervalUs;
    }
    return boundedForceSensorTraceIntervalUs(cfg.forceSensorTraceSamplePeriodUs);
}

int workerTimerIntervalMs(qint64 targetIntervalUs)
{
    if(targetIntervalUs <= kForceSensorTraceMaxIntervalUs){
        return 0;
    }
    return static_cast<int>(std::max<qint64>(1, (targetIntervalUs + 999) / 1000));
}

// 判断某轴是否正受 PVT/直线模组等保护，力控需避让。
bool isProtectedMotionAxis(const ControlWorker::Config& cfg, int axisIndex)
{
    return axisIndex >= 0 &&
            axisIndex < static_cast<int>(cfg.protectedMotionAxes.size()) &&
            cfg.protectedMotionAxes[axisIndex];
}

// 按时间戳裁剪诊断历史，防止长期运行内存无限增长。
bool isExpectedForceTrajectoryValid(const std::vector<std::vector<double>>& expectedForceTraj,
                                    const std::vector<double>& timeStamp)
{
    const int pointCount = static_cast<int>(timeStamp.size());
    if(expectedForceTraj.empty() || pointCount <= 0){
        return false;
    }
    for(int i=0; i<pointCount; ++i){
        if(!std::isfinite(timeStamp[i])){
            return false;
        }
        if(i > 0 && timeStamp[i] <= timeStamp[i - 1]){
            return false;
        }
    }
    for(const std::vector<double>& sensorTraj : expectedForceTraj){
        if(static_cast<int>(sensorTraj.size()) != pointCount){
            return false;
        }
        for(double force : sensorTraj){
            if(!std::isfinite(force)){
                return false;
            }
        }
    }
    return true;
}

bool isOptionalTrajectoryTableValid(const std::vector<std::vector<double>>& traj,
                                    const std::vector<double>& timeStamp)
{
    if(traj.empty()){
        return true;
    }
    const int pointCount = static_cast<int>(timeStamp.size());
    if(pointCount <= 0){
        return false;
    }
    for(const std::vector<double>& channelTraj : traj){
        if(static_cast<int>(channelTraj.size()) != pointCount){
            return false;
        }
        for(double value : channelTraj){
            if(!std::isfinite(value)){
                return false;
            }
        }
    }
    return true;
}

struct ExpectedForceInterpolation {
    std::vector<double> force;
    std::vector<double> derivative;
    std::vector<double> rateFeedForwardScale;
    std::vector<double> ropeVelocityRadPerSec;
    std::vector<double> ropeAccelerationRadPerSec2;
    std::vector<int> platformCaptureTrajectoryPlatform;
};

double interpolateTrajectoryForceAtTime(const std::vector<double>& sensorTraj,
                                        const std::vector<double>& timeStamp,
                                        double trajectoryTimeSec)
{
    const int pointCount = static_cast<int>(timeStamp.size());
    if(pointCount <= 0 || static_cast<int>(sensorTraj.size()) != pointCount){
        return 0.0;
    }
    if(pointCount == 1 || trajectoryTimeSec <= timeStamp.front()){
        return sensorTraj.front();
    }
    if(trajectoryTimeSec >= timeStamp.back()){
        return sensorTraj.back();
    }

    const auto upperIt = std::upper_bound(timeStamp.begin(), timeStamp.end(), trajectoryTimeSec);
    int upperIndex = static_cast<int>(std::distance(timeStamp.begin(), upperIt));
    int lowerIndex = std::max(0, upperIndex - 1);
    upperIndex = std::min(upperIndex, pointCount - 1);
    const double dt = timeStamp[upperIndex] - timeStamp[lowerIndex];
    const double ratio = dt > 0.0 ?
                std::min(1.0, std::max(0.0, (trajectoryTimeSec - timeStamp[lowerIndex]) / dt)) :
                0.0;
    return sensorTraj[lowerIndex] + (sensorTraj[upperIndex] - sensorTraj[lowerIndex]) * ratio;
}

bool trajectoryRemainsFlatForLookAhead(const std::vector<double>& sensorTraj,
                                       const std::vector<double>& timeStamp,
                                       double trajectoryTimeSec,
                                       double referenceForce,
                                       double lookAheadSec,
                                       double toleranceN)
{
    const int pointCount = static_cast<int>(timeStamp.size());
    if(pointCount <= 0 || static_cast<int>(sensorTraj.size()) != pointCount){
        return false;
    }

    const double boundedToleranceN =
            std::isfinite(toleranceN) ? std::max(0.0, toleranceN) : 0.0;
    const double boundedLookAheadSec =
            std::isfinite(lookAheadSec) ? std::max(0.0, lookAheadSec) : 0.0;
    const double endTimeSec =
            std::min(timeStamp.back(), trajectoryTimeSec + boundedLookAheadSec);

    const double sampledEndForce =
            interpolateTrajectoryForceAtTime(sensorTraj, timeStamp, endTimeSec);
    if(std::fabs(sampledEndForce - referenceForce) > boundedToleranceN){
        return false;
    }

    const auto firstIt = std::upper_bound(timeStamp.begin(), timeStamp.end(), trajectoryTimeSec);
    const auto endIt = std::lower_bound(timeStamp.begin(), timeStamp.end(), endTimeSec);
    const int firstIndex = static_cast<int>(std::distance(timeStamp.begin(), firstIt));
    const int endIndex = static_cast<int>(std::distance(timeStamp.begin(), endIt));
    for(int pointIndex=firstIndex; pointIndex<endIndex; ++pointIndex){
        if(std::fabs(sensorTraj[pointIndex] - referenceForce) > boundedToleranceN){
            return false;
        }
    }
    return true;
}

ExpectedForceInterpolation interpolateExpectedForceTrajectory(
        const ControlWorker::Config& cfg,
        const std::vector<std::vector<double>>& expectedForceTraj,
        const std::vector<double>& timeStamp,
        const std::vector<std::vector<double>>& ropeVelocityRadPerSecTraj,
        const std::vector<std::vector<double>>& ropeAccelerationRadPerSec2Traj,
        double trajectoryTimeSec)
{
    const int pointCount = static_cast<int>(timeStamp.size());
    const int sensorCount = std::max(cfg.sensorCount, static_cast<int>(expectedForceTraj.size()));
    ExpectedForceInterpolation result;
    result.force = cfg.expectedForce;
    if(static_cast<int>(result.force.size()) < sensorCount){
        result.force.resize(sensorCount, 0.0);
    }
    result.derivative.assign(sensorCount, 0.0);
    result.rateFeedForwardScale.assign(sensorCount, 1.0);
    result.ropeVelocityRadPerSec.assign(sensorCount, 0.0);
    result.ropeAccelerationRadPerSec2.assign(sensorCount, 0.0);
    result.platformCaptureTrajectoryPlatform.assign(sensorCount, 0);
    if(pointCount <= 0){
        return result;
    }

    int lowerIndex = 0;
    int upperIndex = 0;
    double ratio = 0.0;
    if(pointCount <= 1){
        lowerIndex = 0;
        upperIndex = 0;
    }
    else if(trajectoryTimeSec <= timeStamp.front()){
        lowerIndex = 0;
        upperIndex = 1;
    }
    else if(trajectoryTimeSec >= timeStamp.back()){
        lowerIndex = pointCount - 1;
        upperIndex = pointCount - 1;
    }
    else{
        const auto upperIt = std::upper_bound(timeStamp.begin(), timeStamp.end(), trajectoryTimeSec);
        upperIndex = static_cast<int>(std::distance(timeStamp.begin(), upperIt));
        lowerIndex = std::max(0, upperIndex - 1);
        upperIndex = std::min(upperIndex, pointCount - 1);
        const double dt = timeStamp[upperIndex] - timeStamp[lowerIndex];
        ratio = dt > 0.0 ? (trajectoryTimeSec - timeStamp[lowerIndex]) / dt : 0.0;
        ratio = std::min(1.0, std::max(0.0, ratio));
    }

    for(int sensorIndex=0; sensorIndex<static_cast<int>(expectedForceTraj.size()); ++sensorIndex){
        const std::vector<double>& sensorTraj = expectedForceTraj[sensorIndex];
        if(static_cast<int>(sensorTraj.size()) != pointCount ||
                sensorIndex >= static_cast<int>(result.force.size())){
            continue;
        }
        const double lowerForce = sensorTraj[lowerIndex];
        const double upperForce = sensorTraj[upperIndex];
        result.force[sensorIndex] = lowerForce + (upperForce - lowerForce) * ratio;
        const double dt = timeStamp[upperIndex] - timeStamp[lowerIndex];
        if(dt > 0.0 &&
                sensorIndex < static_cast<int>(result.derivative.size())){
            result.derivative[sensorIndex] = (upperForce - lowerForce) / dt;
            if(sensorIndex < static_cast<int>(result.rateFeedForwardScale.size())){
                const double fadeTimeSec =
                        std::isfinite(cfg.forceExpectedRateFeedForwardFadeTimeSec) ?
                            std::max(0.0, cfg.forceExpectedRateFeedForwardFadeTimeSec) :
                            0.0;
                if(fadeTimeSec > 0.0){
                    result.rateFeedForwardScale[sensorIndex] =
                            expectedRateFeedForwardFadeScale(sensorTraj,
                                                             timeStamp,
                                                             lowerIndex,
                                                             upperIndex,
                                                             trajectoryTimeSec,
                                                             fadeTimeSec);
                }
            }
        }
        const double rateThreshold =
                std::isfinite(cfg.forcePlatformCaptureRateThresholdNPerSec) ?
                    std::max(0.0, cfg.forcePlatformCaptureRateThresholdNPerSec) :
                    0.0;
        const bool currentSegmentFlat =
                std::fabs(result.derivative[sensorIndex]) <= rateThreshold;
        bool lookAheadFlat = false;
        if(currentSegmentFlat){
            lookAheadFlat =
                    trajectoryRemainsFlatForLookAhead(sensorTraj,
                                                      timeStamp,
                                                      trajectoryTimeSec,
                                                      result.force[sensorIndex],
                                                      cfg.forcePlatformCaptureTrajectoryLookAheadSec,
                                                      cfg.forcePlatformCaptureTrajectoryToleranceN);
        }
        result.platformCaptureTrajectoryPlatform[sensorIndex] =
                currentSegmentFlat && lookAheadFlat ? 1 : 0;
    }
    for(int sensorIndex=0; sensorIndex<static_cast<int>(ropeVelocityRadPerSecTraj.size()); ++sensorIndex){
        const std::vector<double>& sensorTraj = ropeVelocityRadPerSecTraj[sensorIndex];
        if(static_cast<int>(sensorTraj.size()) != pointCount ||
                sensorIndex >= static_cast<int>(result.ropeVelocityRadPerSec.size())){
            continue;
        }
        result.ropeVelocityRadPerSec[sensorIndex] =
                sensorTraj[lowerIndex] +
                (sensorTraj[upperIndex] - sensorTraj[lowerIndex]) * ratio;
    }
    for(int sensorIndex=0; sensorIndex<static_cast<int>(ropeAccelerationRadPerSec2Traj.size()); ++sensorIndex){
        const std::vector<double>& sensorTraj = ropeAccelerationRadPerSec2Traj[sensorIndex];
        if(static_cast<int>(sensorTraj.size()) != pointCount ||
                sensorIndex >= static_cast<int>(result.ropeAccelerationRadPerSec2.size())){
            continue;
        }
        result.ropeAccelerationRadPerSec2[sensorIndex] =
                sensorTraj[lowerIndex] +
                (sensorTraj[upperIndex] - sensorTraj[lowerIndex]) * ratio;
    }
    return result;
}

template<typename Sample>
void trimRawHistory(QVector<Sample>& history, qint64 cutoffMs)
{
    int removeCount = 0;
    while(removeCount < history.size() && history.at(removeCount).wallClockMs < cutoffMs){
        ++removeCount;
    }
    if(removeCount > 0){
        history.erase(history.begin(), history.begin() + removeCount);
    }
}

template<typename Sample>
void trimRawHistoryToMaxSamples(QVector<Sample>& history, int maxSamples)
{
    if(maxSamples <= 0 || history.size() <= maxSamples){
        return;
    }
    const int removeCount = history.size() - maxSamples;
    history.erase(history.begin(), history.begin() + removeCount);
}

template<typename Sample>
void trimRawHistoryForMode(QVector<Sample>& history,
                           qint64 nowMs,
                           bool fullRecording,
                           int defaultMaxSamples)
{
    if(fullRecording){
        return;
    }
    trimRawHistory(history, nowMs - kDiagnosticRawDefaultRetentionMs);
    trimRawHistoryToMaxSamples(history, defaultMaxSamples);
}

bool shouldAppendDiagnosticRawSample(bool fullRecording,
                                     qint64 sampleWallClockUs,
                                     qint64& lastAppendUs)
{
    if(fullRecording){
        return true;
    }
    if(sampleWallClockUs <= 0){
        return false;
    }
    if(lastAppendUs <= 0 ||
            sampleWallClockUs - lastAppendUs >= kDiagnosticRawDefaultSampleIntervalUs){
        lastAppendUs = sampleWallClockUs;
        return true;
    }
    return false;
}

}

ControlWorker::ControlWorker(HardwareInterface* hardware, QObject* parent)
    : QObject(parent),
      hardwareInterface(hardware)
{
}

void ControlWorker::setConfig(const Config& newConfig)
{
    QMutexLocker locker(&configMutex);
    config = newConfig;
}

void ControlWorker::setDiagnosticRawHistoryFullRecordingEnabled(bool enabled)
{
    enabled = enabled && RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;
    const bool wasEnabled = diagnosticRawHistoryFullRecordingEnabled.exchange(enabled);
    if(wasEnabled == enabled){
        return;
    }
    if(!enabled){
        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
        QMutexLocker locker(&timingHistoryMutex);
        trimRawHistoryForMode(sensorValueRawHistory,
                              nowMs,
                              false,
                              kDiagnosticSensorValueDefaultMaxSamples);
        trimRawHistoryForMode(sensorFrameRawHistory,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
        trimRawHistoryForMode(sensorTraceReadRawHistory,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
        trimRawHistoryForMode(controlLoopRawHistory,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
    }
}

void ControlWorker::setExternalExpectedForce(const std::vector<double>& expectedForce)
{
    QMutexLocker locker(&configMutex);
    externalExpectedForce = expectedForce;
    hasExternalExpectedForce = !externalExpectedForce.empty();
    externalExpectedForceTrajectory.clear();
    externalExpectedForceTrajectoryTimeStamp.clear();
    hasExternalExpectedForceTrajectory = false;
    externalExpectedForceTrajectoryStartUs = 0;
    lastExternalExpectedForceTrajectoryValue.clear();
    lastExternalExpectedForceTrajectoryDerivative.clear();
    lastExternalExpectedForceTrajectoryRateFeedForwardScale.clear();
    externalExpectedRopeVelocityRadPerSecTrajectory.clear();
    externalExpectedRopeAccelerationRadPerSec2Trajectory.clear();
    lastExternalExpectedRopeVelocityRadPerSec.clear();
    lastExternalExpectedRopeAccelerationRadPerSec2.clear();
    hasLastExternalExpectedForceTrajectoryValue = false;
}

bool ControlWorker::setExternalExpectedForceTrajectory(
        const std::vector<std::vector<double>>& expectedForceTraj,
        const std::vector<double>& timeStamp)
{
    return setExternalExpectedForceTrajectory(expectedForceTraj,
                                             timeStamp,
                                             {},
                                             {});
}

bool ControlWorker::setExternalExpectedForceTrajectory(
        const std::vector<std::vector<double>>& expectedForceTraj,
        const std::vector<double>& timeStamp,
        const std::vector<std::vector<double>>& ropeVelocityRadPerSecTraj,
        const std::vector<std::vector<double>>& ropeAccelerationRadPerSec2Traj)
{
    QMutexLocker locker(&configMutex);
    if(!isExpectedForceTrajectoryValid(expectedForceTraj, timeStamp) ||
            !isOptionalTrajectoryTableValid(ropeVelocityRadPerSecTraj, timeStamp) ||
            !isOptionalTrajectoryTableValid(ropeAccelerationRadPerSec2Traj, timeStamp)){
        externalExpectedForceTrajectory.clear();
        externalExpectedForceTrajectoryTimeStamp.clear();
        hasExternalExpectedForceTrajectory = false;
        externalExpectedForceTrajectoryStartUs = 0;
        lastExternalExpectedForceTrajectoryValue.clear();
        lastExternalExpectedForceTrajectoryDerivative.clear();
        lastExternalExpectedForceTrajectoryRateFeedForwardScale.clear();
        externalExpectedRopeVelocityRadPerSecTrajectory.clear();
        externalExpectedRopeAccelerationRadPerSec2Trajectory.clear();
        lastExternalExpectedRopeVelocityRadPerSec.clear();
        lastExternalExpectedRopeAccelerationRadPerSec2.clear();
        lastExternalExpectedForceTrajectoryPlatform.clear();
        hasLastExternalExpectedForceTrajectoryValue = false;
        return false;
    }

    externalExpectedForceTrajectory = expectedForceTraj;
    externalExpectedForceTrajectoryTimeStamp = timeStamp;
    externalExpectedRopeVelocityRadPerSecTrajectory = ropeVelocityRadPerSecTraj;
    externalExpectedRopeAccelerationRadPerSec2Trajectory = ropeAccelerationRadPerSec2Traj;
    hasExternalExpectedForceTrajectory = true;
    externalExpectedForceTrajectoryStartUs = 0;
    lastExternalExpectedForceTrajectoryValue.clear();
    lastExternalExpectedForceTrajectoryDerivative.clear();
    lastExternalExpectedForceTrajectoryRateFeedForwardScale.clear();
    lastExternalExpectedRopeVelocityRadPerSec.clear();
    lastExternalExpectedRopeAccelerationRadPerSec2.clear();
    lastExternalExpectedForceTrajectoryPlatform.clear();
    hasLastExternalExpectedForceTrajectoryValue = false;
    return true;
}

bool ControlWorker::startExternalExpectedForceTrajectoryClock(qint64 startMonotonicUs)
{
    QMutexLocker locker(&configMutex);
    if(!hasExternalExpectedForceTrajectory ||
            externalExpectedForceTrajectory.empty() ||
            externalExpectedForceTrajectoryTimeStamp.empty()){
        return false;
    }
    externalExpectedForceTrajectoryStartUs =
            startMonotonicUs > 0 ? startMonotonicUs : monotonicNowUs();
    return true;
}

void ControlWorker::clearExternalExpectedForce()
{
    QMutexLocker locker(&configMutex);
    externalExpectedForce.clear();
    hasExternalExpectedForce = false;
    externalExpectedForceTrajectory.clear();
    externalExpectedForceTrajectoryTimeStamp.clear();
    hasExternalExpectedForceTrajectory = false;
    externalExpectedForceTrajectoryStartUs = 0;
    lastExternalExpectedForceTrajectoryValue.clear();
    lastExternalExpectedForceTrajectoryDerivative.clear();
    lastExternalExpectedForceTrajectoryRateFeedForwardScale.clear();
    externalExpectedRopeVelocityRadPerSecTrajectory.clear();
    externalExpectedRopeAccelerationRadPerSec2Trajectory.clear();
    lastExternalExpectedRopeVelocityRadPerSec.clear();
    lastExternalExpectedRopeAccelerationRadPerSec2.clear();
    lastExternalExpectedForceTrajectoryPlatform.clear();
    hasLastExternalExpectedForceTrajectoryValue = false;
}

void ControlWorker::resetForcePidControllerState()
{
    QThread* targetThread = thread();
    if(targetThread && targetThread->isRunning() && QThread::currentThread() != targetThread){
        QMetaObject::invokeMethod(this,
                                  &ControlWorker::resetForcePidControllerState,
                                  Qt::BlockingQueuedConnection);
        return;
    }

    const Config cfg = currentConfig();
    resetForceFeedbackState(cfg.sensorCount);
    resetTorqueCommandState();
}

ControlWorker::Snapshot ControlWorker::latestSnapshot() const
{
    Snapshot latest;
    {
        QMutexLocker locker(&snapshotMutex);
        latest = snapshot;
    }
    // 保留Trace读取完成时的固定帧龄，另算缓存副本当前帧龄。命令入口和
    // SafetyMonitor的新快照接纳判据使用前者；后者只用于停更诊断，避免
    // 异步安全线程把同一份已接纳快照的自然老化误判为采集时超限。
    latest.runtimeTraceCurrentFrameAgeUs = latest.runtimeTraceNewestFrameAgeUs;
    if(latest.runtimeTraceFrameMonotonicUs > 0){
        const qint64 nowUs = monotonicNowUs();
        if(nowUs >= latest.runtimeTraceFrameMonotonicUs){
            latest.runtimeTraceCurrentFrameAgeUs = std::max(
                        latest.runtimeTraceCurrentFrameAgeUs,
                        nowUs - latest.runtimeTraceFrameMonotonicUs);
        }
    }
    return latest;
}

QVector<ControlWorker::SensorValueSample> ControlWorker::sensorValueHistory(qint64 startWallClockMs,
                                                                            qint64 endWallClockMs) const
{
    if(!kEnableControlWorkerDiagnosticRawHistory){
        Q_UNUSED(startWallClockMs);
        Q_UNUSED(endWallClockMs);
        return {};
    }
    QMutexLocker locker(&timingHistoryMutex);
    QVector<SensorValueSample> filtered;
    filtered.reserve(sensorValueRawHistory.size());
    for(const SensorValueSample& sample : sensorValueRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<ControlWorker::SensorValueSample> ControlWorker::sensorTraceValueHistory(qint64 startWallClockMs,
                                                                                 qint64 endWallClockMs) const
{
    if(!kEnableControlWorkerDiagnosticRawHistory){
        Q_UNUSED(startWallClockMs);
        Q_UNUSED(endWallClockMs);
        return {};
    }
    QMutexLocker locker(&timingHistoryMutex);
    QVector<SensorValueSample> filtered;
    filtered.reserve(sensorValueRawHistory.size());
    for(const SensorValueSample& sample : sensorValueRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs ||
                !sample.fromTrace || !sample.expandedTraceFrame){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<ControlWorker::DiagnosticRawSample> ControlWorker::sensorTimingHistory(qint64 startWallClockMs,
                                                                               qint64 endWallClockMs) const
{
    if(!kEnableControlWorkerDiagnosticRawHistory){
        Q_UNUSED(startWallClockMs);
        Q_UNUSED(endWallClockMs);
        return {};
    }
    QMutexLocker locker(&timingHistoryMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(sensorFrameRawHistory.size());
    for(const DiagnosticRawSample& sample : sensorFrameRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<ControlWorker::DiagnosticRawSample> ControlWorker::sensorTraceFrameTimingHistory(qint64 startWallClockMs,
                                                                                         qint64 endWallClockMs) const
{
    if(!kEnableControlWorkerDiagnosticRawHistory){
        Q_UNUSED(startWallClockMs);
        Q_UNUSED(endWallClockMs);
        return {};
    }
    QMutexLocker locker(&timingHistoryMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(sensorFrameRawHistory.size());
    for(const DiagnosticRawSample& sample : sensorFrameRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs ||
                !sample.fromTrace || !sample.expandedTraceFrame){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<ControlWorker::DiagnosticRawSample> ControlWorker::sensorTraceReadTimingHistory(qint64 startWallClockMs,
                                                                                        qint64 endWallClockMs) const
{
    if(!kEnableControlWorkerDiagnosticRawHistory){
        Q_UNUSED(startWallClockMs);
        Q_UNUSED(endWallClockMs);
        return {};
    }
    QMutexLocker locker(&timingHistoryMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(sensorTraceReadRawHistory.size());
    for(const DiagnosticRawSample& sample : sensorTraceReadRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<ControlWorker::DiagnosticRawSample> ControlWorker::controlLoopTimingHistory(qint64 startWallClockMs,
                                                                                    qint64 endWallClockMs) const
{
    if(!kEnableControlWorkerDiagnosticRawHistory){
        Q_UNUSED(startWallClockMs);
        Q_UNUSED(endWallClockMs);
        return {};
    }
    QMutexLocker locker(&timingHistoryMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(controlLoopRawHistory.size());
    for(const DiagnosticRawSample& sample : controlLoopRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

std::vector<ControlWorker::ForcePidTraceSample> ControlWorker::takeForcePidTraceSamples()
{
    QMutexLocker locker(&forcePidTraceMutex);
    std::vector<ForcePidTraceSample> samples;
    samples.reserve(forcePidTraceRingCount);
    if(forcePidTraceRingCount == 0 || forcePidTraceRingBuffer.empty()){
        return samples;
    }

    for(std::size_t i = 0; i < forcePidTraceRingCount; ++i){
        const std::size_t index =
                (forcePidTraceRingStartIndex + i) % forcePidTraceRingBuffer.size();
        samples.push_back(std::move(forcePidTraceRingBuffer[index]));
    }
    forcePidTraceRingBuffer.clear();
    forcePidTraceRingStartIndex = 0;
    forcePidTraceRingCount = 0;
    return samples;
}

void ControlWorker::clearForcePidTraceSamples()
{
    QMutexLocker locker(&forcePidTraceMutex);
    forcePidTraceRingBuffer.clear();
    if(forcePidTraceRingBuffer.capacity() < kForcePidTraceTransportCapacity){
        forcePidTraceRingBuffer.reserve(kForcePidTraceTransportCapacity);
    }
    forcePidTraceRingStartIndex = 0;
    forcePidTraceRingCount = 0;
}

void ControlWorker::appendForcePidTraceSample(ForcePidTraceSample&& sample)
{
    QMutexLocker locker(&forcePidTraceMutex);
    if(forcePidTraceRingBuffer.capacity() < kForcePidTraceTransportCapacity){
        forcePidTraceRingBuffer.reserve(kForcePidTraceTransportCapacity);
    }
    if(forcePidTraceRingBuffer.size() < kForcePidTraceTransportCapacity){
        forcePidTraceRingBuffer.push_back(std::move(sample));
        forcePidTraceRingCount = forcePidTraceRingBuffer.size();
        return;
    }

    forcePidTraceRingBuffer[forcePidTraceRingStartIndex] = std::move(sample);
    forcePidTraceRingStartIndex =
            (forcePidTraceRingStartIndex + 1) % forcePidTraceRingBuffer.size();
    forcePidTraceRingCount = forcePidTraceRingBuffer.size();
}

void ControlWorker::resetTimingDiagnostics()
{
    QThread* targetThread = thread();
    if(targetThread && targetThread->isRunning() && QThread::currentThread() != targetThread){
        QMetaObject::invokeMethod(this, &ControlWorker::resetTimingDiagnostics, Qt::BlockingQueuedConnection);
        return;
    }

    timingDiagnostics = TimingDiagnostics{};
    lastControlLoopTimestampUs = 0;
    nextControlLoopDueUs = 0;
    lastControlLoopSampleIntervalUs = 0;
    previousControlLoopDurationUs = 0;
    lastSensorFrameTimestampUs = 0;
    lastSensorFrameWallClockUs = 0;
    lastTraceExpandedSensorFrameTimestampUs = 0;
    lastSensorTraceReadCallUs = 0;
    nextSensorReadDueUs = 0;
    lastSensorSampleIntervalUs = 0;
    lastMotorHomeRefreshUs = 0;
    lastMotorVelocityRefreshUs = 0;
    lastMotorTorqueRefreshUs = 0;
    lastMotorVelocityPosition.clear();
    resetTorqueCommandState();
    resetForceFeedbackState();
    lastSensorValueHistoryTrimMs = 0;
    lastControlLoopHistoryTrimMs = 0;
    lastSensorFrameHistoryTrimMs = 0;
    lastSensorTraceReadHistoryTrimMs = 0;
    lastSensorValueRawHistoryAppendUs = 0;
    lastSensorFrameRawHistoryAppendUs = 0;
    lastSensorTraceReadRawHistoryAppendUs = 0;
    lastControlLoopRawHistoryAppendUs = 0;
    {
        QMutexLocker locker(&snapshotMutex);
        snapshot.timingDiagnostics = timingDiagnostics;
    }
    {
        QMutexLocker locker(&timingHistoryMutex);
        sensorValueRawHistory.clear();
        sensorFrameRawHistory.clear();
        sensorTraceReadRawHistory.clear();
        controlLoopRawHistory.clear();
    }
}

bool ControlWorker::prepareOnlineVelocityControl(const OnlineVelocityPlan& plan,
                                                 const OnlineVelocityConfig& onlineConfig,
                                                 QString* errorMessage)
{
    const Config cfg = currentConfig();
    const auto fail = [&](const QString& message) {
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(endpointRemoteControl.isActive() || endpointRemoteControl.isPrepared()){
        return fail(QStringLiteral("末端遥控已准备或正在运行，不能同时准备预设在线速度轨迹"));
    }
    if(!hardwareInterface || !cfg.systemRunning || !cfg.useLeadshine ||
            !hardwareInterface->isLSConnected()){
        return fail(QStringLiteral("完整系统和雷赛控制器必须已运行"));
    }
    if(cfg.axisCount < kOnlineVelocityAxisCount ||
            static_cast<int>(cfg.axes.size()) < kOnlineVelocityAxisCount){
        return fail(QStringLiteral("在线速度控制需要八个已配置电机轴"));
    }
    if(cfg.forceThreadEnabled || cfg.pvtActiveOrPaused || cfg.commissioningModeActive){
        return fail(QStringLiteral("在线速度控制不能与力控、PVT 或单轴调试同时运行"));
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(!cfg.axes[axis].isMotorAxis || !hardwareInterface->isMotorEnabled(axis)){
            return fail(QStringLiteral("电机轴 %1 未配置或未使能").arg(axis));
        }
        const double configuredVelocityLimit = cfg.axes[axis].motorVelMax;
        if(!std::isfinite(configuredVelocityLimit) || configuredVelocityLimit <= 0.0){
            return fail(QStringLiteral("电机轴 %1 的既有速度上限无效").arg(axis));
        }
        if(onlineConfig.maxVelocity > configuredVelocityLimit + 1.0e-12){
            return fail(QStringLiteral(
                            "在线速度上限 %1 超过电机轴 %2 的既有安全上限 %3")
                        .arg(onlineConfig.maxVelocity, 0, 'f', 6)
                        .arg(axis)
                        .arg(configuredVelocityLimit, 0, 'f', 6));
        }
    }
    if(!onlineVelocityControl.prepare(plan, onlineConfig, errorMessage)){
        return false;
    }
    publishOnlineVelocityStatus();
    return true;
}

bool ControlWorker::startOnlineVelocityControl(QString* errorMessage)
{
    if(endpointRemoteControl.isActive() || endpointRemoteControl.isPrepared()){
        if(errorMessage){
            *errorMessage = QStringLiteral("末端遥控已准备或正在运行，不能启动预设在线速度轨迹");
        }
        return false;
    }
    if(!onlineVelocityControl.isPrepared()){
        if(errorMessage){
            *errorMessage = QStringLiteral("请先准备在线速度轨迹");
        }
        return false;
    }
    const Config cfg = currentConfig();
    if(!hardwareInterface || !cfg.systemRunning || !cfg.useLeadshine ||
            cfg.forceThreadEnabled || cfg.pvtActiveOrPaused || cfg.commissioningModeActive){
        if(errorMessage){
            *errorMessage = QStringLiteral("运行状态已变化，在线速度控制启动条件不再满足");
        }
        return false;
    }
    if(cfg.axisCount < kOnlineVelocityAxisCount ||
            static_cast<int>(cfg.axes.size()) < kOnlineVelocityAxisCount){
        if(errorMessage){
            *errorMessage = QStringLiteral("在线速度控制启动时八轴配置已不完整");
        }
        return false;
    }
    const double requestedVelocityLimit =
            onlineVelocityControl.currentConfig().maxVelocity;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(!cfg.axes[axis].isMotorAxis || !hardwareInterface->isMotorEnabled(axis)){
            if(errorMessage){
                *errorMessage = QStringLiteral("在线速度控制启动时电机轴 %1 已失能或配置改变")
                        .arg(axis);
            }
            return false;
        }
        const double configuredVelocityLimit = cfg.axes[axis].motorVelMax;
        if(!std::isfinite(configuredVelocityLimit) ||
                configuredVelocityLimit <= 0.0 ||
                requestedVelocityLimit > configuredVelocityLimit + 1.0e-12){
            if(errorMessage){
                *errorMessage = QStringLiteral(
                            "在线速度控制启动时电机轴 %1 的既有速度上限已失效或变小")
                        .arg(axis);
            }
            return false;
        }
    }
    if(!onlineVelocityControl.start(monotonicNowUs(), errorMessage)){
        publishOnlineVelocityStatus();
        return false;
    }
    if(!hardwareInterface->setOnlineVelocityRuntimeTraceProfileEnabled(true)){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法配置包含命令/实际速度的 Runtime Trace");
        }
        onlineVelocityControl.stop(true,
                                   QStringLiteral("包含命令/实际速度的 Runtime Trace 配置失败"));
        onlineVelocityControl.finishRecording();
        publishOnlineVelocityStatus();
        return false;
    }
    const OnlineVelocityPlan& plan = onlineVelocityControl.preparedPlan();
    const std::vector<int> axes(plan.axes.begin(), plan.axes.end());
    hardwareInterface->resetMotorVelBatchFastState(axes);
    onlineVelocityControl.resetTraceWaitClock(monotonicNowUs());
    publishOnlineVelocityStatus();
    return true;
}

void ControlWorker::stopOnlineVelocityControl(bool emergency, const QString& reason)
{
    const bool wasActive = onlineVelocityControl.isActive();
    if(!wasActive && !onlineVelocityControl.isPrepared()){
        return;
    }
    if(!wasActive){
        onlineVelocityControl.stop(false, reason);
        publishOnlineVelocityStatus();
        return;
    }
    if(!hardwareInterface){
        onlineVelocityControl.stop(true, reason);
        publishOnlineVelocityStatus();
        return;
    }
    const OnlineVelocityPlan& plan = onlineVelocityControl.preparedPlan();
    const std::vector<int> axes(plan.axes.begin(), plan.axes.end());
    if(emergency){
        hardwareInterface->emergencyStopAxes(axes);
    }
    else{
        bool stopOk = true;
        for(int axis : axes){
            stopOk = hardwareInterface->motorStop(axis) && stopOk;
        }
        if(!stopOk){
            hardwareInterface->emergencyStopAxes(axes);
            emergency = true;
        }
    }
    hardwareInterface->resetMotorVelBatchFastState(axes);
    onlineVelocityControl.stop(emergency, reason);
    publishOnlineVelocityStatus();
}

OnlineVelocityStatus ControlWorker::onlineVelocityStatus() const
{
    QMutexLocker locker(&onlineVelocityMutex);
    return onlineVelocityStatusCache;
}

void ControlWorker::publishOnlineVelocityStatus()
{
    QMutexLocker locker(&onlineVelocityMutex);
    onlineVelocityStatusCache = onlineVelocityControl.status();
}

bool ControlWorker::prepareEndpointRemoteControl(
        const EndpointRemoteConfig& remoteConfig,
        quint64 inputSessionToken,
        QString* errorMessage)
{
    const Config cfg = currentConfig();
    const auto fail = [&](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(inputSessionToken == 0){
        return fail(QStringLiteral("末端遥控输入会话令牌无效"));
    }
    if(endpointRemoteControl.isActive() || endpointRemoteControl.isPrepared()){
        return fail(QStringLiteral("末端遥控已准备或正在运行，不能覆盖当前输入会话"));
    }
    if(onlineVelocityControl.isActive() || onlineVelocityControl.isPrepared()){
        return fail(QStringLiteral("预设在线速度轨迹已准备或正在运行，不能同时进入末端遥控"));
    }
    clearPreparedEndpointRemoteCommand();
    if(!hardwareInterface || !cfg.systemRunning || !cfg.useLeadshine ||
            !hardwareInterface->isLSConnected()){
        return fail(QStringLiteral("完整系统和雷赛控制器必须已运行"));
    }
    if(cfg.forceThreadEnabled || cfg.pvtActiveOrPaused || cfg.commissioningModeActive){
        return fail(QStringLiteral("末端遥控不能与力控、PVT或单轴调试同时运行"));
    }
    if(cfg.axisCount < kOnlineVelocityAxisCount ||
            static_cast<int>(cfg.axes.size()) < kOnlineVelocityAxisCount){
        return fail(QStringLiteral("末端遥控需要八个已配置电机轴"));
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(!cfg.axes[axis].isMotorAxis || !hardwareInterface->isMotorEnabled(axis)){
            return fail(QStringLiteral("末端遥控电机轴%1未配置或未使能")
                        .arg(axis + 1));
        }
        if(!std::isfinite(cfg.axes[axis].motorVelMax) ||
                cfg.axes[axis].motorVelMax <= 0.0 ||
                remoteConfig.onlineVelocity.maxVelocity >
                    cfg.axes[axis].motorVelMax + 1.0e-12){
            return fail(QStringLiteral("末端遥控速度上限超过电机轴%1既有安全上限")
                        .arg(axis + 1));
        }
    }
    if(!hardwareInterface->setRuntimeTraceUsageProfile(
                HardwareInterface::RuntimeTraceUsageProfile::
                    EndpointRemoteTransition,
                inputSessionToken)){
        return fail(QStringLiteral(
                    "无法进入末端遥控Runtime Trace过渡profile"));
    }
    endpointRemoteTracePhase = EndpointRemoteTracePhase::TransitionPrepared;
    endpointRemoteTransitionStartUs = 0;
    endpointRemoteTransitionLastDiagnosticUs = 0;
    clearEndpointRemoteInputMailbox();
    QString prepareError;
    if(!endpointRemoteControl.prepare(remoteConfig, &prepareError)){
        QString restoreError;
        const bool restored = restoreEndpointRemoteRuntimeTraceProfile(
                    inputSessionToken,
                    &restoreError);
        if(!restored){
            prepareError += QStringLiteral("；%1").arg(restoreError);
        }
        endpointRemoteTracePhase = restored ?
                    EndpointRemoteTracePhase::Inactive :
                    EndpointRemoteTracePhase::Faulted;
        if(errorMessage){
            *errorMessage = prepareError;
        }
        return false;
    }
    {
        QMutexLocker locker(&endpointRemoteInputMutex);
        activeEndpointRemoteInputSessionToken = inputSessionToken;
    }
    publishEndpointRemoteStatus();
    return true;
}

bool ControlWorker::startEndpointRemoteControl(QString* errorMessage)
{
    if(!endpointRemoteControl.isPrepared()){
        if(errorMessage){
            *errorMessage = QStringLiteral("请先准备末端遥控");
        }
        return false;
    }
    if(endpointRemoteTracePhase !=
            EndpointRemoteTracePhase::TransitionPrepared){
        if(errorMessage){
            *errorMessage = QStringLiteral("末端遥控Trace阶段不是TransitionPrepared");
        }
        return false;
    }
    const quint64 inputSessionToken = endpointRemoteInputSessionToken();
    if(inputSessionToken == 0){
        if(errorMessage){
            *errorMessage = QStringLiteral("末端遥控输入会话尚未建立");
        }
        return false;
    }
    clearPreparedEndpointRemoteCommand();
    const Config cfg = currentConfig();
    if(!hardwareInterface || !cfg.systemRunning || !cfg.useLeadshine ||
            cfg.forceThreadEnabled || cfg.pvtActiveOrPaused ||
            cfg.commissioningModeActive){
        if(errorMessage){
            *errorMessage = QStringLiteral("运行状态已经变化，末端遥控启动条件不再满足");
        }
        return false;
    }
    if(!endpointRemoteControl.start(monotonicNowUs(), errorMessage)){
        publishEndpointRemoteStatus();
        return false;
    }
    const std::vector<int> axes{0, 1, 2, 3, 4, 5, 6, 7};
    hardwareInterface->resetMotorVelBatchFastState(axes);
    const qint64 attributionStartUs = monotonicNowUs();
    endpointRemoteTracePhase = EndpointRemoteTracePhase::TransitionAcquiring;
    endpointRemoteTransitionStartUs = attributionStartUs;
    endpointRemoteTransitionLastDiagnosticUs = 0;
    startEndpointRemoteAttribution(attributionStartUs);
    endpointRemoteControl.resetTraceWaitClock(attributionStartUs);
    emit displayInfoSignal(
                QStringLiteral(
                    "末端遥控过渡Trace开始异步追平：每个ControlWorker周期只读取一次，"
                    "在初始等待窗口内保持速度命令未授权；取得帧龄不超过5 ms的可靠同帧安全上下文后再切换Running profile。")
                    .toStdString(),
                "info");
    publishEndpointRemoteStatus();
    return true;
}

void ControlWorker::updateEndpointRemoteInput(
        EndpointRemoteMotionMode motionMode,
        const std::array<double, 3>& normalizedDirection,
        quint64 sequence,
        quint64 inputSessionToken,
        bool uiSourceFresh,
        qint64 uiSourceAgeUs)
{
    const qint64 receivedUs = monotonicNowUs();
    QMutexLocker locker(&endpointRemoteInputMutex);
    if(inputSessionToken == 0 ||
            inputSessionToken != activeEndpointRemoteInputSessionToken){
        return;
    }
    if(pendingEndpointRemoteInputValid.load(std::memory_order_relaxed) &&
            sequence <= pendingEndpointRemoteInputSequence){
        return;
    }
    pendingEndpointRemoteMotionMode = motionMode;
    pendingEndpointRemoteDirection = normalizedDirection;
    pendingEndpointRemoteInputSequence = sequence;
    pendingEndpointRemoteInputSessionToken = inputSessionToken;
    pendingEndpointRemoteInputReceivedUs = receivedUs;
    pendingEndpointRemoteUiSourceAgeUs = uiSourceAgeUs;
    pendingEndpointRemoteUiSourceFresh = uiSourceFresh;
    pendingEndpointRemoteInputValid.store(true, std::memory_order_release);
}

void ControlWorker::requestEndpointRemoteStop(
        bool emergency,
        const QString& reason,
        quint64 inputSessionToken)
{
    QMutexLocker locker(&endpointRemoteInputMutex);
    if(inputSessionToken == 0 ||
            inputSessionToken != activeEndpointRemoteInputSessionToken){
        return;
    }
    // 急停请求不能被随后到达的普通退出请求降级。
    pendingEndpointRemoteStopEmergency =
            pendingEndpointRemoteStopEmergency || emergency;
    pendingEndpointRemoteStopReason = reason.isEmpty() ?
                QStringLiteral("用户退出末端遥控") : reason;
    pendingEndpointRemoteStopSessionToken = inputSessionToken;
    pendingEndpointRemoteStopValid.store(true, std::memory_order_release);
}

void ControlWorker::requestEndpointRemoteSafetyStop(const QString& reason)
{
    QMutexLocker locker(&endpointRemoteInputMutex);
    if(activeEndpointRemoteInputSessionToken == 0){
        return;
    }
    pendingEndpointRemoteStopEmergency = true;
    pendingEndpointRemoteStopReason = reason.isEmpty() ?
                QStringLiteral("独立安全监控已触发硬件急停") : reason;
    pendingEndpointRemoteStopSessionToken =
            activeEndpointRemoteInputSessionToken;
    pendingEndpointRemoteStopValid.store(true, std::memory_order_release);
}

void ControlWorker::clearEndpointRemoteInputMailbox()
{
    QMutexLocker locker(&endpointRemoteInputMutex);
    activeEndpointRemoteInputSessionToken = 0;
    pendingEndpointRemoteMotionMode = EndpointRemoteMotionMode::None;
    pendingEndpointRemoteDirection.fill(0.0);
    pendingEndpointRemoteInputSequence = 0;
    pendingEndpointRemoteInputSessionToken = 0;
    pendingEndpointRemoteInputReceivedUs = 0;
    pendingEndpointRemoteUiSourceAgeUs = 0;
    pendingEndpointRemoteUiSourceFresh = true;
    pendingEndpointRemoteInputValid.store(false, std::memory_order_release);
    pendingEndpointRemoteStopEmergency = false;
    pendingEndpointRemoteStopReason.clear();
    pendingEndpointRemoteStopSessionToken = 0;
    pendingEndpointRemoteStopValid.store(false, std::memory_order_release);
}

quint64 ControlWorker::endpointRemoteInputSessionToken() const
{
    QMutexLocker locker(&endpointRemoteInputMutex);
    return activeEndpointRemoteInputSessionToken;
}

bool ControlWorker::restoreEndpointRemoteRuntimeTraceProfile(
        quint64 inputSessionToken,
        QString* errorMessage)
{
    if(errorMessage){
        errorMessage->clear();
    }
    if(!hardwareInterface){
        if(errorMessage){
            *errorMessage = QStringLiteral("硬件接口不可用，无法恢复预设在线速度Trace profile");
        }
        return false;
    }
    if(hardwareInterface->setRuntimeTraceUsageProfile(
                HardwareInterface::RuntimeTraceUsageProfile::
                    PresetOnlineVelocity)){
        return true;
    }

    // 基础对象配置恢复失败时，至少退回不授权速度命令的Transition语义。
    // Transition与Running对象集相同，因此该降级不需要再次重配板卡。
    if(inputSessionToken != 0){
        hardwareInterface->setRuntimeTraceUsageProfile(
                    HardwareInterface::RuntimeTraceUsageProfile::
                        EndpointRemoteTransition,
                    inputSessionToken);
    }
    if(errorMessage){
        *errorMessage = QStringLiteral(
                    "预设在线速度Trace profile恢复失败，已保持停机并撤销遥控命令授权");
    }
    return false;
}

void ControlWorker::consumeEndpointRemoteInputMailbox()
{
    if(!pendingEndpointRemoteInputValid.load(std::memory_order_acquire)){
        return;
    }
    std::array<double, 3> direction{};
    EndpointRemoteMotionMode motionMode = EndpointRemoteMotionMode::None;
    quint64 sequence = 0;
    quint64 inputSessionToken = 0;
    qint64 receivedUs = 0;
    qint64 uiSourceAgeUs = 0;
    bool uiSourceFresh = true;
    {
        QMutexLocker locker(&endpointRemoteInputMutex);
        if(!pendingEndpointRemoteInputValid.load(std::memory_order_relaxed)){
            return;
        }
        motionMode = pendingEndpointRemoteMotionMode;
        direction = pendingEndpointRemoteDirection;
        sequence = pendingEndpointRemoteInputSequence;
        inputSessionToken = pendingEndpointRemoteInputSessionToken;
        receivedUs = pendingEndpointRemoteInputReceivedUs;
        uiSourceAgeUs = pendingEndpointRemoteUiSourceAgeUs;
        uiSourceFresh = pendingEndpointRemoteUiSourceFresh;
        pendingEndpointRemoteInputValid.store(false, std::memory_order_release);
        if(inputSessionToken == 0 ||
                inputSessionToken != activeEndpointRemoteInputSessionToken){
            return;
        }
    }
    endpointRemoteControl.updateInput(motionMode,
                                      direction,
                                      sequence,
                                      receivedUs,
                                      uiSourceFresh,
                                      uiSourceAgeUs);
}

bool ControlWorker::consumeEndpointRemoteStopRequest()
{
    if(!pendingEndpointRemoteStopValid.load(std::memory_order_acquire)){
        return false;
    }

    bool emergency = false;
    QString reason;
    quint64 inputSessionToken = 0;
    {
        QMutexLocker locker(&endpointRemoteInputMutex);
        if(!pendingEndpointRemoteStopValid.load(std::memory_order_relaxed)){
            return false;
        }
        emergency = pendingEndpointRemoteStopEmergency;
        reason = pendingEndpointRemoteStopReason;
        inputSessionToken = pendingEndpointRemoteStopSessionToken;
        pendingEndpointRemoteStopEmergency = false;
        pendingEndpointRemoteStopReason.clear();
        pendingEndpointRemoteStopSessionToken = 0;
        pendingEndpointRemoteStopValid.store(false, std::memory_order_release);
        if(inputSessionToken == 0 ||
                inputSessionToken != activeEndpointRemoteInputSessionToken){
            return false;
        }
    }

    stopEndpointRemoteControl(emergency, reason);
    return true;
}

void ControlWorker::stopEndpointRemoteControl(bool emergency,
                                              const QString& reason)
{
    clearPreparedEndpointRemoteCommand();
    const bool wasActive = endpointRemoteControl.isActive();
    const quint64 inputSessionToken = endpointRemoteInputSessionToken();
    if(!wasActive && !endpointRemoteControl.isPrepared()){
        bool restored = true;
        if(hardwareInterface && inputSessionToken != 0){
            QString restoreError;
            restored = restoreEndpointRemoteRuntimeTraceProfile(
                        inputSessionToken,
                        &restoreError);
            if(!restored){
                emit displayInfoSignal(restoreError.toStdString(), "error");
            }
        }
        endpointRemoteTracePhase = restored ?
                    EndpointRemoteTracePhase::Inactive :
                    EndpointRemoteTracePhase::Faulted;
        endpointRemoteTransitionStartUs = 0;
        endpointRemoteTransitionLastDiagnosticUs = 0;
        finishEndpointRemoteAttribution(monotonicNowUs());
        clearEndpointRemoteInputMailbox();
        return;
    }
    if(!wasActive){
        QString finalReason = reason;
        QString restoreError;
        const bool restored = restoreEndpointRemoteRuntimeTraceProfile(
                    inputSessionToken,
                    &restoreError);
        if(!restored){
            finalReason += QStringLiteral("；%1").arg(restoreError);
            emit displayInfoSignal(restoreError.toStdString(), "error");
        }
        finishEndpointRemoteAttribution(monotonicNowUs());
        endpointRemoteControl.stop(!restored, finalReason);
        endpointRemoteTracePhase = restored ?
                    EndpointRemoteTracePhase::Inactive :
                    EndpointRemoteTracePhase::Faulted;
        endpointRemoteTransitionStartUs = 0;
        endpointRemoteTransitionLastDiagnosticUs = 0;
        clearEndpointRemoteInputMailbox();
        publishEndpointRemoteStatus();
        return;
    }
    const std::vector<int> axes{0, 1, 2, 3, 4, 5, 6, 7};
    if(!hardwareInterface){
        finishEndpointRemoteAttribution(monotonicNowUs());
        endpointRemoteControl.stop(true, reason);
        endpointRemoteTracePhase = EndpointRemoteTracePhase::Faulted;
        endpointRemoteTransitionStartUs = 0;
        endpointRemoteTransitionLastDiagnosticUs = 0;
        clearEndpointRemoteInputMailbox();
        publishEndpointRemoteStatus();
        return;
    }
    freezeEndpointRemoteAttribution();
    bool axesStopOk = true;
    if(emergency){
        axesStopOk = hardwareInterface->emergencyStopAxes(axes);
    }
    else{
        bool stopOk = hardwareInterface->motorStopAxes(axes);
        if(!stopOk){
            axesStopOk = hardwareInterface->emergencyStopAxes(axes);
            emergency = true;
        }
    }
    hardwareInterface->resetMotorVelBatchFastState(axes);
    QString finalReason = reason;
    if(!axesStopOk){
        emergency = true;
        const QString stopError = QStringLiteral(
                    "末端遥控停机命令失败，未恢复Preset profile，"
                    "已保持Transition以撤销速度命令授权");
        hardwareInterface->setRuntimeTraceUsageProfile(
                    HardwareInterface::RuntimeTraceUsageProfile::
                        EndpointRemoteTransition,
                    inputSessionToken);
        finalReason += QStringLiteral("；%1").arg(stopError);
        emit displayInfoSignal(stopError.toStdString(), "error");
    }
    else{
        QString restoreError;
        if(!restoreEndpointRemoteRuntimeTraceProfile(inputSessionToken,
                                                     &restoreError)){
            emergency = true;
            finalReason += QStringLiteral("；%1").arg(restoreError);
            emit displayInfoSignal(restoreError.toStdString(), "error");
        }
    }
    endpointRemoteControl.stop(emergency, finalReason);
    endpointRemoteTracePhase = emergency ?
                EndpointRemoteTracePhase::Faulted :
                EndpointRemoteTracePhase::Inactive;
    endpointRemoteTransitionStartUs = 0;
    endpointRemoteTransitionLastDiagnosticUs = 0;
    finishEndpointRemoteAttribution(monotonicNowUs());
    clearEndpointRemoteInputMailbox();
    publishEndpointRemoteStatus();
}

EndpointRemoteStatus ControlWorker::endpointRemoteStatus() const
{
    QMutexLocker locker(&onlineVelocityMutex);
    return endpointRemoteStatusCache;
}

void ControlWorker::publishEndpointRemoteStatus()
{
    QMutexLocker locker(&onlineVelocityMutex);
    endpointRemoteStatusCache = endpointRemoteControl.status();
}

void ControlWorker::startEndpointRemoteAttribution(qint64 nowUs)
{
    endpointRemoteAttribution = EndpointRemoteAttributionStats();
    if(!RuntimeFeatureSwitches::kEndpointRemoteAttributionDiagnosticsEnabled ||
            !hardwareInterface){
        if(hardwareInterface){
            hardwareInterface->setMotorEnableQueryTimingEnabled(false);
        }
        return;
    }
    endpointRemoteAttribution.active = true;
    endpointRemoteAttribution.startUs = nowUs;
    hardwareInterface->setMotorEnableQueryTimingEnabled(true);
    emit displayInfoSignal(
                QStringLiteral("末端遥控归因诊断已启用：累计现有状态查询、Trace分层耗时及速度命令纯时间戳，退出或故障后输出最终统计，不增加硬件读取。")
                    .toStdString(),
                "info");
}

void ControlWorker::observeEndpointRemoteAttribution(
        const HardwareInterface::RuntimeTraceSnapshot& traceSnapshot,
        const EndpointRemoteTimingContext& timing)
{
    if(!endpointRemoteAttribution.active){
        return;
    }
    EndpointRemoteAttributionStats& stats = endpointRemoteAttribution;
    ++stats.traceSampleCount;
    stats.latestTraceReadUs = std::max<qint64>(0, timing.traceReadDurationUs);
    stats.totalTraceReadUs += stats.latestTraceReadUs;
    stats.maximumTraceReadUs = std::max(stats.maximumTraceReadUs,
                                        stats.latestTraceReadUs);
    stats.latestTraceQueueWaitUs = std::max<qint64>(
                0, traceSnapshot.hardwareThreadQueueWaitUs);
    stats.totalTraceQueueWaitUs += stats.latestTraceQueueWaitUs;
    stats.maximumTraceQueueWaitUs = std::max(stats.maximumTraceQueueWaitUs,
                                             stats.latestTraceQueueWaitUs);
    stats.latestTraceHardwareUs = std::max<qint64>(
                0, traceSnapshot.hardwareThreadExecutionUs);
    stats.totalTraceHardwareUs += stats.latestTraceHardwareUs;
    stats.maximumTraceHardwareUs = std::max(stats.maximumTraceHardwareUs,
                                            stats.latestTraceHardwareUs);
    stats.latestTraceDataApiUs = std::max<qint64>(
                0, traceSnapshot.dataApiDurationUs);
    stats.totalTraceDataApiUs += stats.latestTraceDataApiUs;
    stats.maximumTraceDataApiUs = std::max(stats.maximumTraceDataApiUs,
                                           stats.latestTraceDataApiUs);
    if(traceSnapshot.timingReliable && traceSnapshot.fifoCaughtUp &&
            !traceSnapshot.traceLost && traceSnapshot.newestFrameAgeUs >= 0){
        stats.maximumReliableFrameAgeUs = std::max(
                    stats.maximumReliableFrameAgeUs,
                    traceSnapshot.newestFrameAgeUs);
    }
    if(!traceSnapshot.fifoCaughtUp){
        ++stats.fifoNotCaughtUpCount;
    }
    if(traceSnapshot.traceLost){
        ++stats.traceLostCount;
    }
}

void ControlWorker::observeEndpointRemoteCommandAttribution(
        const HardwareInterface::EndpointRemoteVelocityCommandReport& report)
{
    using Outcome = HardwareInterface::EndpointRemoteVelocityCommandOutcome;
    if(!endpointRemoteAttribution.active ||
            report.outcome == Outcome::NotAttempted){
        return;
    }
    const auto duration = [](qint64 startUs, qint64 endUs) -> qint64 {
        return startUs > 0 && endUs >= startUs ? endUs - startUs : 0;
    };
    const auto accumulate = [](qint64 value,
                               qint64& latest,
                               qint64& total,
                               qint64& maximum) {
        latest = std::max<qint64>(0, value);
        total += latest;
        maximum = std::max(maximum, latest);
    };

    EndpointRemoteAttributionStats& stats = endpointRemoteAttribution;
    if(report.outcome == Outcome::FreshFrameDeferred){
        ++stats.freshFrameDeferredCount;
        stats.latestVelocityCommandReport = report;
        return;
    }
    ++stats.velocityCommandCount;
    accumulate(duration(report.traceValidationCompletedUs,
                        report.beforeSubmitUs),
               stats.latestValidationToSubmitUs,
               stats.totalValidationToSubmitUs,
               stats.maximumValidationToSubmitUs);
    const qint64 compositeTaskStartUs = report.compositeTaskStartUs > 0 ?
                report.compositeTaskStartUs :
                report.hardwareThreadTaskStartUs;
    accumulate(duration(report.beforeSubmitUs,
                        compositeTaskStartUs),
               stats.latestCommandQueueWaitUs,
               stats.totalCommandQueueWaitUs,
               stats.maximumCommandQueueWaitUs);
    const qint64 preSdkOrRejectEndUs = report.sdkCallStartUs > 0 ?
                report.sdkCallStartUs : report.hardwareThreadTaskEndUs;
    accumulate(duration(report.hardwareThreadTaskStartUs,
                        preSdkOrRejectEndUs),
               stats.latestHardwarePreSdkOrRejectUs,
               stats.totalHardwarePreSdkOrRejectUs,
               stats.maximumHardwarePreSdkOrRejectUs);
    if(report.sdkCallStartUs > 0 &&
            report.sdkCallEndUs >= report.sdkCallStartUs){
        ++stats.sdkBatchCount;
        accumulate(duration(report.sdkCallStartUs,
                            report.sdkCallEndUs),
                   stats.latestSdkBatchUs,
                   stats.totalSdkBatchUs,
                   stats.maximumSdkBatchUs);
    }
    else{
        stats.latestSdkBatchUs = 0;
    }
    accumulate(duration(compositeTaskStartUs,
                        report.hardwareThreadTaskEndUs),
               stats.latestCommandHardwareUs,
               stats.totalCommandHardwareUs,
               stats.maximumCommandHardwareUs);
    accumulate(duration(report.beforeSubmitUs,
                        report.hardwareThreadTaskEndUs),
               stats.latestCommandCallUs,
               stats.totalCommandCallUs,
               stats.maximumCommandCallUs);
    stats.latestVelocityCommandReport = report;
}

void ControlWorker::reportEndpointRemoteAttribution(bool finalReport,
                                                    qint64 nowUs)
{
    if(!endpointRemoteAttribution.active || !hardwareInterface){
        return;
    }
    EndpointRemoteAttributionStats& stats = endpointRemoteAttribution;
    const HardwareInterface::MotorEnableQueryTimingSnapshot motorTiming =
            stats.motorTimingFrozen ?
                stats.frozenMotorTiming :
                hardwareInterface->motorEnableQueryTimingSnapshot();
    const auto average = [](qint64 total, quint64 count) -> qint64 {
        return count > 0 ?
                    total / static_cast<qint64>(count) : 0;
    };
    const qint64 measurementEndUs = stats.frozenAtUs > 0 ?
                stats.frozenAtUs : nowUs;
    const qint64 elapsedUs = stats.startUs > 0 ?
                std::max<qint64>(0, measurementEndUs - stats.startUs) : 0;
    const qint64 queryRatePerSec = elapsedUs > 0 ?
                static_cast<qint64>(motorTiming.queryCount * 1000000ULL /
                                    static_cast<quint64>(elapsedUs)) : 0;
    const qint64 averageMotorCallUs = average(
                motorTiming.totalCallDurationUs, motorTiming.queryCount);
    emit displayInfoSignal(
                QStringLiteral(
                    "末端遥控归因统计[%1]：累计=%2 ms；Trace样本=%3，总读取最近/均值/最大=%4/%5/%6 us，HardwareThread排队最近/均值/最大=%7/%8/%9 us，线程内执行最近/均值/最大=%10/%11/%12 us，dmc_trace_get_data累计最近/均值/最大=%13/%14/%15 us，可靠帧年龄最大=%16 us，FIFO未追平周期=%17，Trace丢帧周期=%18；状态查询次数/频率=%19/%20 Hz，排队最近/均值/最大=%21/%22/%23 us，nmc_get_axis_state_machine最近/均值/最大=%24/%25/%26 us，单次调用最近/均值/最大=%27/%28/%29 us，八轴一轮按均值估算=%30 us。")
                    .arg(finalReport ? QStringLiteral("最终") : QStringLiteral("周期"))
                    .arg(elapsedUs / 1000)
                    .arg(stats.traceSampleCount)
                    .arg(stats.latestTraceReadUs)
                    .arg(average(stats.totalTraceReadUs, stats.traceSampleCount))
                    .arg(stats.maximumTraceReadUs)
                    .arg(stats.latestTraceQueueWaitUs)
                    .arg(average(stats.totalTraceQueueWaitUs, stats.traceSampleCount))
                    .arg(stats.maximumTraceQueueWaitUs)
                    .arg(stats.latestTraceHardwareUs)
                    .arg(average(stats.totalTraceHardwareUs, stats.traceSampleCount))
                    .arg(stats.maximumTraceHardwareUs)
                    .arg(stats.latestTraceDataApiUs)
                    .arg(average(stats.totalTraceDataApiUs, stats.traceSampleCount))
                    .arg(stats.maximumTraceDataApiUs)
                    .arg(stats.maximumReliableFrameAgeUs)
                    .arg(stats.fifoNotCaughtUpCount)
                    .arg(stats.traceLostCount)
                    .arg(motorTiming.queryCount)
                    .arg(queryRatePerSec)
                    .arg(motorTiming.latestQueueWaitUs)
                    .arg(average(motorTiming.totalQueueWaitUs,
                                 motorTiming.queryCount))
                    .arg(motorTiming.maximumQueueWaitUs)
                    .arg(motorTiming.latestApiDurationUs)
                    .arg(average(motorTiming.totalApiDurationUs,
                                 motorTiming.queryCount))
                    .arg(motorTiming.maximumApiDurationUs)
                    .arg(motorTiming.latestCallDurationUs)
                    .arg(averageMotorCallUs)
                    .arg(motorTiming.maximumCallDurationUs)
                    .arg(averageMotorCallUs * 8)
                    .toStdString(),
                "info");

    const HardwareInterface::EndpointRemoteVelocityCommandReport&
            commandReport = stats.latestVelocityCommandReport;
    emit displayInfoSignal(
                QStringLiteral(
                    "末端遥控速度命令归因[%1]：样本=%2；Trace校验完成至提交最近/均值/最大=%3/%4/%5 us，HardwareThread排队最近/均值/最大=%6/%7/%8 us，入口校验至SDK或拒绝最近/均值/最大=%9/%10/%11 us；SDK批次样本=%12，执行最近/均值/最大=%13/%14/%15 us；HardwareThread任务最近/均值/最大=%16/%17/%18 us，同步调用最近/均值/最大=%19/%20/%21 us；末次结果=%22，时间戳[Trace校验完成/提交前/HardwareThread入口/SDK开始/SDK结束/HardwareThread结束]=%23/%24/%25/%26/%27/%28 us，末次原因=%29。")
                    .arg(finalReport ? QStringLiteral("最终") : QStringLiteral("周期"))
                    .arg(stats.velocityCommandCount)
                    .arg(stats.latestValidationToSubmitUs)
                    .arg(average(stats.totalValidationToSubmitUs,
                                 stats.velocityCommandCount))
                    .arg(stats.maximumValidationToSubmitUs)
                    .arg(stats.latestCommandQueueWaitUs)
                    .arg(average(stats.totalCommandQueueWaitUs,
                                 stats.velocityCommandCount))
                    .arg(stats.maximumCommandQueueWaitUs)
                    .arg(stats.latestHardwarePreSdkOrRejectUs)
                    .arg(average(stats.totalHardwarePreSdkOrRejectUs,
                                 stats.velocityCommandCount))
                    .arg(stats.maximumHardwarePreSdkOrRejectUs)
                    .arg(stats.sdkBatchCount)
                    .arg(stats.latestSdkBatchUs)
                    .arg(average(stats.totalSdkBatchUs,
                                 stats.sdkBatchCount))
                    .arg(stats.maximumSdkBatchUs)
                    .arg(stats.latestCommandHardwareUs)
                    .arg(average(stats.totalCommandHardwareUs,
                                 stats.velocityCommandCount))
                    .arg(stats.maximumCommandHardwareUs)
                    .arg(stats.latestCommandCallUs)
                    .arg(average(stats.totalCommandCallUs,
                                 stats.velocityCommandCount))
                    .arg(stats.maximumCommandCallUs)
                    .arg(endpointRemoteVelocityCommandOutcomeText(
                             commandReport.outcome))
                    .arg(commandReport.traceValidationCompletedUs)
                    .arg(commandReport.beforeSubmitUs)
                    .arg(commandReport.hardwareThreadTaskStartUs)
                    .arg(commandReport.sdkCallStartUs)
                    .arg(commandReport.sdkCallEndUs)
                    .arg(commandReport.hardwareThreadTaskEndUs)
                    .arg(commandReport.failureReason.isEmpty() ?
                             QStringLiteral("无") : commandReport.failureReason)
                    .toStdString(),
                "info");
    emit displayInfoSignal(
                QStringLiteral(
                    "末端遥控复合提交归因[%1]：静止态等待新鲜帧=%2次；末次执行profile=%3，规划耗时=%4 us，Trace读取完成=%5 us，入口帧龄=%6 us，剩余5 ms截止预算=%7 us，命令范数/最大绝对值=%8/%9，SDK实际调用=%10，结果=%11")
                    .arg(finalReport ? QStringLiteral("最终") :
                                      QStringLiteral("周期"))
                    .arg(stats.freshFrameDeferredCount)
                    .arg(commandReport.actuationProfile)
                    .arg(commandReport.planningCompletedUs >=
                         commandReport.planningStartedUs ?
                             commandReport.planningCompletedUs -
                                 commandReport.planningStartedUs : 0)
                    .arg(commandReport.traceReadCompletedUs)
                    .arg(commandReport.entryFrameAgeUs)
                    .arg(commandReport.remainingDeadlineBudgetUs)
                    .arg(commandReport.commandL2Norm, 0, 'f', 6)
                    .arg(commandReport.commandMaximumAbsVelocity, 0, 'f', 6)
                    .arg(commandReport.sdkCalled ? 1 : 0)
                    .arg(endpointRemoteVelocityCommandOutcomeText(
                             commandReport.outcome))
                    .toStdString(),
                "info");
}

void ControlWorker::freezeEndpointRemoteAttribution()
{
    if(!endpointRemoteAttribution.active || !hardwareInterface ||
            endpointRemoteAttribution.motorTimingFrozen){
        return;
    }
    endpointRemoteAttribution.frozenMotorTiming =
            hardwareInterface->motorEnableQueryTimingSnapshot();
    endpointRemoteAttribution.motorTimingFrozen = true;
    endpointRemoteAttribution.frozenAtUs = monotonicNowUs();
    hardwareInterface->setMotorEnableQueryTimingEnabled(false);
}

void ControlWorker::finishEndpointRemoteAttribution(qint64 nowUs)
{
    if(endpointRemoteAttribution.active){
        freezeEndpointRemoteAttribution();
        reportEndpointRemoteAttribution(true, nowUs);
    }
    if(hardwareInterface){
        hardwareInterface->setMotorEnableQueryTimingEnabled(false);
    }
    endpointRemoteAttribution.active = false;
}

void ControlWorker::processOnlineVelocityControl(
        const Config& cfg,
        const HardwareInterface::RuntimeTraceSnapshot& traceSnapshot,
        qint64 nowUs)
{
    if(!onlineVelocityControl.isActive()){
        return;
    }
    if(!hardwareInterface || !cfg.systemRunning || !cfg.useLeadshine ||
            cfg.forceThreadEnabled || cfg.pvtActiveOrPaused || cfg.commissioningModeActive){
        stopOnlineVelocityControl(true,
                                  QStringLiteral("运行互锁条件变化，在线速度控制已急停"));
        return;
    }

    OnlineVelocityFeedback feedback;
    feedback.wallClockUs = traceSnapshot.wallClockUs;
    feedback.monotonicUs = traceSnapshot.monotonicUs;
    feedback.newestFrameAgeUs = traceSnapshot.newestFrameAgeUs;
    feedback.frameCount = traceSnapshot.frameCount;
    feedback.fifoValidNum = traceSnapshot.fifoValidNum;
    feedback.fifoFreeNum = traceSnapshot.fifoFreeNum;
    feedback.traceSamplePeriodUs = traceSnapshot.traceSamplePeriodUs;
    feedback.logicalFrameSequence = traceSnapshot.logicalFrameSequence;
    feedback.fromTrace = traceSnapshot.fromTrace;
    feedback.frameSequenceValid = traceSnapshot.frameSequenceValid;
    feedback.timingReliable = traceSnapshot.timingReliable;
    feedback.fifoCaughtUp = traceSnapshot.fifoCaughtUp;
    feedback.traceLost = traceSnapshot.traceLost;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    feedback.actualPosition.fill(nan);
    feedback.actualVelocity.fill(nan);
    feedback.tracedCommandVelocity.fill(nan);
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(axis < static_cast<int>(traceSnapshot.motorPosition.size())){
            feedback.actualPosition[axis] = traceSnapshot.motorPosition[axis];
        }
        if(axis < static_cast<int>(traceSnapshot.motorActualVelocity.size())){
            feedback.actualVelocity[axis] = traceSnapshot.motorActualVelocity[axis];
        }
        if(axis < static_cast<int>(traceSnapshot.motorCommandVelocity.size())){
            feedback.tracedCommandVelocity[axis] = traceSnapshot.motorCommandVelocity[axis];
        }
    }

    const OnlineVelocityStep stepResult = onlineVelocityControl.step(feedback, nowUs);
    if(stepResult.action == OnlineVelocityStep::Action::None){
        publishOnlineVelocityStatus();
        return;
    }

    const OnlineVelocityPlan& plan = onlineVelocityControl.preparedPlan();
    const std::vector<int> axes(plan.axes.begin(), plan.axes.end());
    bool commandOk = true;
    qint64 apiDurationUs = 0;
    if(stepResult.action == OnlineVelocityStep::Action::EmergencyStop){
        QElapsedTimer apiTimer;
        apiTimer.start();
        commandOk = hardwareInterface->emergencyStopAxes(axes);
        apiDurationUs = apiTimer.nsecsElapsed() / 1000;
    }
    else if(stepResult.action == OnlineVelocityStep::Action::NormalStop){
        QElapsedTimer apiTimer;
        apiTimer.start();
        for(int axis : axes){
            commandOk = hardwareInterface->motorStop(axis) && commandOk;
        }
        apiDurationUs = apiTimer.nsecsElapsed() / 1000;
        if(!commandOk){
            hardwareInterface->emergencyStopAxes(axes);
        }
    }
    else{
        std::vector<double> commandVelocity(stepResult.commandVelocity.begin(),
                                            stepResult.commandVelocity.end());
        std::vector<double> currentPosition(stepResult.actualPosition.begin(),
                                            stepResult.actualPosition.end());
        const bool positionsValid = std::all_of(
                    currentPosition.begin(), currentPosition.end(), [](double value) {
            return std::isfinite(value);
        });
        QElapsedTimer apiTimer;
        apiTimer.start();
        commandOk = positionsValid && hardwareInterface->motorVelBatchFast(
                    axes,
                    commandVelocity,
                    onlineVelocityControl.currentConfig().onlineChangeTimeSec,
                    currentPosition);
        apiDurationUs = apiTimer.nsecsElapsed() / 1000;
        if(!commandOk){
            hardwareInterface->emergencyStopAxes(axes);
        }
    }
    const qint64 fullCycleDurationUs = std::max<qint64>(
                0, monotonicNowUs() - nowUs);
    onlineVelocityControl.noteCommandResult(stepResult,
                                            commandOk,
                                            apiDurationUs,
                                            fullCycleDurationUs);

    const bool terminal = stepResult.action == OnlineVelocityStep::Action::NormalStop ||
            stepResult.action == OnlineVelocityStep::Action::EmergencyStop ||
            !commandOk;
    if(terminal){
        hardwareInterface->resetMotorVelBatchFastState(axes);
    }
    publishOnlineVelocityStatus();
}

void ControlWorker::clearPreparedEndpointRemoteCommand()
{
    endpointRemoteDispatchPhase = EndpointRemoteDispatchPhase::Idle;
    preparedEndpointRemoteStep = EndpointRemoteStep();
    endpointRemoteFreshFrameDeferredStartUs = 0;
    endpointRemoteLastFreshFrameDeferredDiagnosticUs = 0;
}

void ControlWorker::completePreparedEndpointRemoteCommand(
        const HardwareInterface::EndpointRemoteTraceCommandResult& result,
        qint64 nowUs,
        qint64 workerLoopEntryUs)
{
    if(endpointRemoteDispatchPhase !=
            EndpointRemoteDispatchPhase::PreparedForCompositeTraceCommand){
        return;
    }

    const HardwareInterface::EndpointRemoteVelocityCommandReport& report =
            result.commandReport;
    const OnlineVelocityFeedback feedback =
            onlineVelocityFeedbackFromTraceSnapshot(result.traceSnapshot);
    observeEndpointRemoteCommandAttribution(report);

    if(report.outcome == HardwareInterface::
                EndpointRemoteVelocityCommandOutcome::FreshFrameDeferred){
        if(endpointRemoteFreshFrameDeferredStartUs <= 0){
            endpointRemoteFreshFrameDeferredStartUs = nowUs;
        }
        endpointRemoteControl.noteCommandDeferred(
                    preparedEndpointRemoteStep,
                    feedback,
                    report.failureReason);
        constexpr qint64 kFreshFrameDeferredDiagnosticIntervalUs =
                5 * 1000 * 1000;
        if(endpointRemoteLastFreshFrameDeferredDiagnosticUs <= 0 ||
                nowUs - endpointRemoteLastFreshFrameDeferredDiagnosticUs >=
                    kFreshFrameDeferredDiagnosticIntervalUs){
            endpointRemoteLastFreshFrameDeferredDiagnosticUs = nowUs;
            emit displayInfoSignal(
                        QStringLiteral(
                            "末端遥控JOG启动仍在等待新鲜同帧Trace（未调用SDK）：已等待=%1 us，入口帧龄=%2 us，剩余截止预算=%3 us，逻辑帧=%4")
                            .arg(nowUs - endpointRemoteFreshFrameDeferredStartUs)
                            .arg(report.entryFrameAgeUs)
                            .arg(report.remainingDeadlineBudgetUs)
                            .arg(result.traceSnapshot.logicalFrameSequence)
                            .toStdString(),
                        "warning");
        }
        const qint64 timeoutUs = endpointRemoteControl.currentConfig()
                .onlineVelocity.initialTraceWaitTimeoutUs;
        if(timeoutUs > 0 &&
                nowUs - endpointRemoteFreshFrameDeferredStartUs > timeoutUs){
            const QString reason = QStringLiteral(
                        "末端遥控JOG启动前等待新鲜同帧Trace超时%1 ms；期间未调用速度SDK")
                    .arg(timeoutUs / 1000);
            stopEndpointRemoteControl(false, reason);
        }
        publishEndpointRemoteStatus();
        return;
    }

    EndpointRemoteStep completedStep = preparedEndpointRemoteStep;
    completedStep.monotonicUs = nowUs;
    completedStep.wallClockUs = feedback.wallClockUs;
    completedStep.logicalFrameSequence = feedback.logicalFrameSequence;
    completedStep.actualPosition = feedback.actualPosition;
    completedStep.actualVelocity = feedback.actualVelocity;
    completedStep.traceValidationCompletedUs =
            report.traceValidationCompletedUs;
    endpointRemoteDispatchPhase =
            EndpointRemoteDispatchPhase::CompositeCompletedAwaitNextCycle;
    endpointRemoteFreshFrameDeferredStartUs = 0;
    endpointRemoteLastFreshFrameDeferredDiagnosticUs = 0;

    const bool commandOk = report.outcome == HardwareInterface::
                EndpointRemoteVelocityCommandOutcome::Succeeded;
    const std::vector<int> axes{0, 1, 2, 3, 4, 5, 6, 7};
    bool axesStopOk = true;
    if(!commandOk){
        freezeEndpointRemoteAttribution();
        axesStopOk = hardwareInterface->emergencyStopAxes(axes);
    }
    const qint64 apiDurationUs =
            report.hardwareThreadTaskEndUs >=
                report.traceValidationCompletedUs ?
                report.hardwareThreadTaskEndUs -
                    report.traceValidationCompletedUs : 0;
    const qint64 fullCycleDurationUs = std::max<qint64>(
                0, monotonicNowUs() - workerLoopEntryUs);
    endpointRemoteControl.noteCommandResult(
                completedStep,
                commandOk,
                apiDurationUs,
                fullCycleDurationUs,
                report.failureReason,
                feedback);
    preparedEndpointRemoteStep = EndpointRemoteStep();

    if(!commandOk){
        const quint64 inputSessionToken = endpointRemoteInputSessionToken();
        endpointRemoteTracePhase = EndpointRemoteTracePhase::Faulted;
        finishEndpointRemoteAttribution(monotonicNowUs());
        hardwareInterface->resetMotorVelBatchFastState(axes);
        if(!axesStopOk){
            hardwareInterface->setRuntimeTraceUsageProfile(
                        HardwareInterface::RuntimeTraceUsageProfile::
                            EndpointRemoteTransition,
                        inputSessionToken);
            emit displayInfoSignal(
                        "末端遥控故障停机命令失败，未恢复Preset profile，已保持Transition以撤销速度命令授权",
                        "error");
        }
        else{
            QString restoreError;
            if(!restoreEndpointRemoteRuntimeTraceProfile(inputSessionToken,
                                                         &restoreError)){
                emit displayInfoSignal(restoreError.toStdString(), "error");
            }
        }
        clearEndpointRemoteInputMailbox();
    }
    publishEndpointRemoteStatus();
}

void ControlWorker::processEndpointRemoteControl(
        const Config& cfg,
        const HardwareInterface::RuntimeTraceSnapshot& traceSnapshot,
        qint64 nowUs,
        const EndpointRemoteTimingContext& timing)
{
    if(!endpointRemoteControl.isActive()){
        return;
    }
    if(!hardwareInterface || !cfg.systemRunning || !cfg.useLeadshine ||
            cfg.forceThreadEnabled || cfg.pvtActiveOrPaused ||
            cfg.commissioningModeActive){
        stopEndpointRemoteControl(
                    true,
                    QStringLiteral("运行互锁条件变化，末端遥控已急停"));
        return;
    }

    const quint64 inputSessionToken = endpointRemoteInputSessionToken();
    const HardwareInterface::EndpointRemoteVelocitySafetyContext&
            remoteSafety = traceSnapshot.endpointRemoteVelocitySafety;
    if(inputSessionToken == 0){
        stopEndpointRemoteControl(
                    true,
                    QStringLiteral("末端遥控输入会话已失效，已在速度命令前急停"));
        return;
    }

    consumeEndpointRemoteInputMailbox();
    observeEndpointRemoteAttribution(traceSnapshot, timing);

    if(endpointRemoteTracePhase ==
            EndpointRemoteTracePhase::TransitionAcquiring){
        const qint64 transitionAgeLimitUs = std::min<qint64>(
                    endpointRemoteControl.currentConfig()
                        .onlineVelocity.traceFeedbackDelayLimitUs(),
                    5 * 1000);
        bool transitionAxesValid =
                remoteSafety.motorPosition.size() >=
                    static_cast<std::size_t>(kOnlineVelocityAxisCount) &&
                remoteSafety.motorSafetyRelativePosition.size() >=
                    static_cast<std::size_t>(kOnlineVelocityAxisCount) &&
                remoteSafety.motorSafetyRelativePositionSource.size() >=
                    static_cast<std::size_t>(kOnlineVelocityAxisCount) &&
                remoteSafety.motorStateMachine.size() >=
                    static_cast<std::size_t>(kOnlineVelocityAxisCount);
        for(int axis = 0; transitionAxesValid &&
            axis < kOnlineVelocityAxisCount; ++axis){
            const HardwareInterface::MotorSafetyRelativePositionSource source =
                    remoteSafety.motorSafetyRelativePositionSource[axis];
            const bool traceSafetySource =
                    source == HardwareInterface::MotorSafetyRelativePositionSource::
                        TraceCommandPersistentHome ||
                    source == HardwareInterface::MotorSafetyRelativePositionSource::
                        TraceCommandSessionHome ||
                    source == HardwareInterface::MotorSafetyRelativePositionSource::
                        TraceFeedbackSessionHome;
            transitionAxesValid = traceSafetySource &&
                    std::isfinite(remoteSafety.motorPosition[axis]) &&
                    std::isfinite(
                        remoteSafety.motorSafetyRelativePosition[axis]) &&
                    remoteSafety.motorStateMachine[axis] == 4;
        }
        const bool transitionTraceValid =
                transitionAgeLimitUs > 0 &&
                remoteSafety.usageProfile ==
                    HardwareInterface::RuntimeTraceUsageProfile::
                        EndpointRemoteTransition &&
                remoteSafety.sessionToken == inputSessionToken &&
                remoteSafety.fromTrace &&
                remoteSafety.frameSequenceValid &&
                remoteSafety.timingReliable &&
                remoteSafety.fifoCaughtUp &&
                !remoteSafety.traceLost &&
                !remoteSafety.statusFaultLatched &&
                remoteSafety.traceSamplePeriodUs == 500 &&
                remoteSafety.newestFrameAgeUs >= 0 &&
                remoteSafety.newestFrameAgeUs <= transitionAgeLimitUs &&
                transitionAxesValid;
        const QString transitionDetail = QStringLiteral(
                    "profile=%1，会话=%2，帧年龄=%3 us，上限=%4 us，"
                    "来源/序号/时序/追平/丢帧/状态锁存/八轴="
                    "%5/%6/%7/%8/%9/%10/%11，"
                    "FIFO有效/空闲=%12/%13，逻辑序号=%14，"
                    "Trace总读/排队/线程内/SDK=%15/%16/%17/%18 us")
                .arg(static_cast<int>(remoteSafety.usageProfile))
                .arg(remoteSafety.sessionToken)
                .arg(remoteSafety.newestFrameAgeUs)
                .arg(transitionAgeLimitUs)
                .arg(remoteSafety.fromTrace ? 1 : 0)
                .arg(remoteSafety.frameSequenceValid ? 1 : 0)
                .arg(remoteSafety.timingReliable ? 1 : 0)
                .arg(remoteSafety.fifoCaughtUp ? 1 : 0)
                .arg(remoteSafety.traceLost ? 1 : 0)
                .arg(remoteSafety.statusFaultLatched ? 1 : 0)
                .arg(transitionAxesValid ? 1 : 0)
                .arg(traceSnapshot.fifoValidNum)
                .arg(traceSnapshot.fifoFreeNum)
                .arg(remoteSafety.logicalFrameSequence)
                .arg(traceSnapshot.totalReadCallUs)
                .arg(traceSnapshot.hardwareThreadQueueWaitUs)
                .arg(traceSnapshot.hardwareThreadExecutionUs)
                .arg(traceSnapshot.dataApiDurationUs);
        if(!transitionTraceValid){
            const qint64 transitionElapsedUs =
                    endpointRemoteTransitionStartUs > 0 ?
                        std::max<qint64>(
                            0, nowUs - endpointRemoteTransitionStartUs) : 0;
            const qint64 transitionTimeoutUs =
                    endpointRemoteControl.currentConfig()
                        .onlineVelocity.initialTraceWaitTimeoutUs;
            if(transitionTimeoutUs > 0 &&
                    transitionElapsedUs > transitionTimeoutUs){
                const QString reason = QStringLiteral(
                            "末端遥控过渡Trace在%1 ms内未取得可靠同帧安全上下文"
                            "（%2）")
                        .arg(transitionTimeoutUs / 1000)
                        .arg(transitionDetail);
                emit displayInfoSignal(reason.toStdString(), "error");
                stopEndpointRemoteControl(false, reason);
                return;
            }
            constexpr qint64 kTransitionDiagnosticIntervalUs = 5 * 1000 * 1000;
            if(endpointRemoteTransitionLastDiagnosticUs <= 0 ||
                    nowUs - endpointRemoteTransitionLastDiagnosticUs >=
                        kTransitionDiagnosticIntervalUs){
                endpointRemoteTransitionLastDiagnosticUs = nowUs;
                emit displayInfoSignal(
                            QStringLiteral(
                                "末端遥控过渡Trace仍在异步追平（已等待%1 ms）：%2")
                                .arg(transitionElapsedUs / 1000)
                                .arg(transitionDetail)
                                .toStdString(),
                            "warning");
            }
            publishEndpointRemoteStatus();
            return;
        }

        if(!hardwareInterface->setRuntimeTraceUsageProfile(
                    HardwareInterface::RuntimeTraceUsageProfile::
                        EndpointRemoteRunning,
                    inputSessionToken)){
            const QString reason = QStringLiteral(
                        "末端遥控过渡Trace已可靠，但无法切换Running profile");
            emit displayInfoSignal(reason.toStdString(), "error");
            stopEndpointRemoteControl(false, reason);
            return;
        }
        endpointRemoteTracePhase =
                EndpointRemoteTracePhase::RunningProfileAwaitingFrame;
        endpointRemoteTransitionStartUs = nowUs;
        endpointRemoteTransitionLastDiagnosticUs = 0;
        endpointRemoteControl.resetTraceWaitClock(nowUs);
        emit displayInfoSignal(
                    QStringLiteral(
                        "末端遥控过渡Trace已追平并通过同帧安全校验，"
                        "已切换Running profile；等待第一份Running profile可靠帧后才授权速度命令（%1）")
                        .arg(transitionDetail)
                        .toStdString(),
                    "info");
        publishEndpointRemoteStatus();
        return;
    }

    if(endpointRemoteTracePhase ==
            EndpointRemoteTracePhase::RunningProfileAwaitingFrame &&
            (remoteSafety.usageProfile !=
                HardwareInterface::RuntimeTraceUsageProfile::
                    EndpointRemoteRunning ||
             remoteSafety.sessionToken != inputSessionToken)){
        const qint64 awaitingElapsedUs =
                endpointRemoteTransitionStartUs > 0 ?
                    std::max<qint64>(
                        0, nowUs - endpointRemoteTransitionStartUs) : 0;
        const qint64 awaitingTimeoutUs =
                endpointRemoteControl.currentConfig()
                    .onlineVelocity.initialTraceWaitTimeoutUs;
        const QString awaitingDetail = QStringLiteral(
                    "当前profile/会话=%1/%2，期望=%3/%4，逻辑序号=%5")
                .arg(static_cast<int>(remoteSafety.usageProfile))
                .arg(remoteSafety.sessionToken)
                .arg(static_cast<int>(
                         HardwareInterface::RuntimeTraceUsageProfile::
                            EndpointRemoteRunning))
                .arg(inputSessionToken)
                .arg(remoteSafety.logicalFrameSequence);
        if(awaitingTimeoutUs > 0 &&
                awaitingElapsedUs > awaitingTimeoutUs){
            const QString reason = QStringLiteral(
                        "末端遥控Running profile在%1 ms内未产出本会话首帧"
                        "（%2）")
                    .arg(awaitingTimeoutUs / 1000)
                    .arg(awaitingDetail);
            emit displayInfoSignal(reason.toStdString(), "error");
            stopEndpointRemoteControl(false, reason);
            return;
        }
        constexpr qint64 kRunningProfileDiagnosticIntervalUs =
                5 * 1000 * 1000;
        if(endpointRemoteTransitionLastDiagnosticUs <= 0 ||
                nowUs - endpointRemoteTransitionLastDiagnosticUs >=
                    kRunningProfileDiagnosticIntervalUs){
            endpointRemoteTransitionLastDiagnosticUs = nowUs;
            emit displayInfoSignal(
                        QStringLiteral(
                            "末端遥控正在等待Running profile首帧"
                            "（已等待%1 ms）：%2")
                            .arg(awaitingElapsedUs / 1000)
                            .arg(awaitingDetail)
                            .toStdString(),
                        "warning");
        }
        publishEndpointRemoteStatus();
        return;
    }

    if(endpointRemoteTracePhase !=
            EndpointRemoteTracePhase::RunningProfileAwaitingFrame &&
            endpointRemoteTracePhase != EndpointRemoteTracePhase::Running){
        stopEndpointRemoteControl(
                    true,
                    QStringLiteral(
                        "末端遥控Trace阶段非法，已在速度命令前急停"));
        return;
    }
    if(remoteSafety.usageProfile !=
            HardwareInterface::RuntimeTraceUsageProfile::
                EndpointRemoteRunning ||
            remoteSafety.sessionToken != inputSessionToken){
        stopEndpointRemoteControl(
                    true,
                    QStringLiteral(
                        "末端遥控Running阶段profile或会话上下文失配，已在速度命令前急停"));
        return;
    }
    if(remoteSafety.statusFaultLatched){
        const QString statusWordHex =
                QString::number(remoteSafety.statusFaultWord, 16)
                .rightJustified(4, QLatin1Char('0'))
                .toUpper();
        stopEndpointRemoteControl(
                    true,
                    QStringLiteral(
                        "末端遥控排空Trace帧锁存驱动状态异常：轴%1，"
                        "0x6041=0x%2，状态=%3，逻辑序号=%4")
                    .arg(remoteSafety.statusFaultAxis + 1)
                    .arg(statusWordHex)
                    .arg(remoteSafety.statusFaultStateMachine)
                    .arg(remoteSafety.statusFaultLogicalFrameSequence));
        return;
    }

    if(endpointRemoteDispatchPhase ==
            EndpointRemoteDispatchPhase::CompositeCompletedAwaitNextCycle){
        endpointRemoteDispatchPhase = EndpointRemoteDispatchPhase::Idle;
        publishEndpointRemoteStatus();
        return;
    }
    if(endpointRemoteDispatchPhase ==
            EndpointRemoteDispatchPhase::PreparedForCompositeTraceCommand){
        // 新鲜帧不足而延期时保留同一份纯规划结果；本轮不再次规划。
        publishEndpointRemoteStatus();
        return;
    }

    const OnlineVelocityFeedback feedback =
            onlineVelocityFeedbackFromTraceSnapshot(traceSnapshot);
    const EndpointRemoteStep stepResult =
            endpointRemoteControl.step(feedback, nowUs, timing);
    if(!stepResult.diagnosticMessage.isEmpty()){
        emit displayInfoSignal(stepResult.diagnosticMessage.toStdString(),
                               stepResult.diagnosticWarning ? "warning" : "info");
    }
    if(endpointRemoteControl.status().state ==
            EndpointRemoteStatus::State::Running &&
            endpointRemoteTracePhase ==
                EndpointRemoteTracePhase::RunningProfileAwaitingFrame){
        endpointRemoteTracePhase = EndpointRemoteTracePhase::Running;
    }
    if(stepResult.action == EndpointRemoteStep::Action::None){
        publishEndpointRemoteStatus();
        return;
    }
    if(stepResult.action == EndpointRemoteStep::Action::CommandVelocity){
        preparedEndpointRemoteStep = stepResult;
        endpointRemoteDispatchPhase = EndpointRemoteDispatchPhase::
                PreparedForCompositeTraceCommand;
        publishEndpointRemoteStatus();
        return;
    }

    const std::vector<int> axes{0, 1, 2, 3, 4, 5, 6, 7};
    const QString commandFailureReason = stepResult.reason;
    QElapsedTimer apiTimer;
    apiTimer.start();
    freezeEndpointRemoteAttribution();
    const bool commandOk = hardwareInterface->emergencyStopAxes(axes);
    const bool axesStopOk = commandOk;
    const qint64 apiDurationUs = apiTimer.nsecsElapsed() / 1000;
    const qint64 cycleStartUs = timing.workerLoopEntryUs > 0 ?
                timing.workerLoopEntryUs : nowUs;
    const qint64 fullCycleDurationUs = std::max<qint64>(
                0, monotonicNowUs() - cycleStartUs);
    endpointRemoteControl.noteCommandResult(stepResult,
                                            commandOk,
                                            apiDurationUs,
                                            fullCycleDurationUs,
                                            commandFailureReason,
                                            feedback);
    if(stepResult.action == EndpointRemoteStep::Action::EmergencyStop ||
            !commandOk){
        endpointRemoteTracePhase = EndpointRemoteTracePhase::Faulted;
        finishEndpointRemoteAttribution(monotonicNowUs());
        hardwareInterface->resetMotorVelBatchFastState(axes);
        if(!axesStopOk){
            hardwareInterface->setRuntimeTraceUsageProfile(
                        HardwareInterface::RuntimeTraceUsageProfile::
                            EndpointRemoteTransition,
                        inputSessionToken);
            emit displayInfoSignal(
                        "末端遥控故障停机命令失败，未恢复Preset profile，已保持Transition以撤销速度命令授权",
                        "error");
        }
        else{
            QString restoreError;
            if(!restoreEndpointRemoteRuntimeTraceProfile(inputSessionToken,
                                                         &restoreError)){
                emit displayInfoSignal(restoreError.toStdString(), "error");
            }
        }
        clearEndpointRemoteInputMailbox();
    }
    publishEndpointRemoteStatus();
}

void ControlWorker::start()
{
    if(!timer){
        timer = new QTimer(this);
        timer->setTimerType(Qt::PreciseTimer);
        connect(timer, &QTimer::timeout, this, &ControlWorker::controlLoop);
    }

    const qint64 nowUs = monotonicNowUs();
    const qint64 targetIntervalUs = traceDrivenWorkerIntervalUs(currentConfig());
    nextControlLoopDueUs = nowUs;
    lastControlLoopSampleIntervalUs = targetIntervalUs;
    previousControlLoopDurationUs = 0;
    nextSensorReadDueUs = nowUs;
    lastSensorSampleIntervalUs = targetIntervalUs;
    resetActualTorqueLimitState(currentConfig().axisCount);
    timer->setInterval(workerTimerIntervalMs(targetIntervalUs));
    timer->start();
}

void ControlWorker::stop()
{
    if(onlineVelocityControl.isActive()){
        stopOnlineVelocityControl(true, QStringLiteral("控制线程停止"));
    }
    else if(onlineVelocityControl.isPrepared()){
        stopOnlineVelocityControl(false, QStringLiteral("控制线程停止"));
    }
    if(endpointRemoteControl.isActive()){
        stopEndpointRemoteControl(true, QStringLiteral("控制线程停止"));
    }
    else if(endpointRemoteControl.isPrepared()){
        stopEndpointRemoteControl(false, QStringLiteral("控制线程停止"));
    }
    onlineVelocityControl.finishRecording();
    if(timer){
        timer->stop();
    }
    nextControlLoopDueUs = 0;
    lastControlLoopSampleIntervalUs = 0;
    previousControlLoopDurationUs = 0;
    nextSensorReadDueUs = 0;
    lastSensorSampleIntervalUs = 0;
    lastSensorFrameTimestampUs = 0;
    lastSensorFrameWallClockUs = 0;
    lastSensorTraceReadCallUs = 0;
    lastTraceExpandedSensorFrameTimestampUs = 0;
    lastForceThreadEnabled = false;
    lastAllCableForceDragModeEnabled = false;
    lastForceFeedForwardOnlyTestModeEnabled = false;
    lastForcePidOutputMode = ForcePidOutputMode::Pid0624;
    latestForceSensorValue.clear();
    hasLatestForceSensorValue = false;
    filteredForceSensorValue.clear();
    hasFilteredForceSensorValue = false;
    cachedMotorHome.clear();
    cachedMotorVel.clear();
    lastMotorVelocityPosition.clear();
    cachedMotorTorqueNm.clear();
    resetTorqueCommandState();
    resetForceFeedbackState();
    resetActualTorqueLimitState();
    lastMotorHomeRefreshUs = 0;
    lastMotorVelocityRefreshUs = 0;
    lastMotorTorqueRefreshUs = 0;
}

ControlWorker::Config ControlWorker::currentConfig() const
{
    QMutexLocker locker(&configMutex);
    return config;
}

std::vector<double> ControlWorker::activeExpectedForce(const Config& cfg,
                                                       bool& fromExternal,
                                                       std::vector<double>* expectedForceDerivative,
                                                       std::vector<double>* expectedRateFeedForwardScale,
                                                       std::vector<double>* expectedRopeVelocityRadPerSec,
                                                       std::vector<double>* expectedRopeAccelerationRadPerSec2,
                                                       std::vector<int>* platformCaptureTrajectoryPlatform)
{
    QMutexLocker locker(&configMutex);
    if(expectedForceDerivative){
        expectedForceDerivative->assign(std::max(0, cfg.sensorCount), 0.0);
    }
    if(expectedRateFeedForwardScale){
        expectedRateFeedForwardScale->assign(std::max(0, cfg.sensorCount), 1.0);
    }
    if(expectedRopeVelocityRadPerSec){
        expectedRopeVelocityRadPerSec->assign(std::max(0, cfg.sensorCount), 0.0);
    }
    if(expectedRopeAccelerationRadPerSec2){
        expectedRopeAccelerationRadPerSec2->assign(std::max(0, cfg.sensorCount), 0.0);
    }
    if(platformCaptureTrajectoryPlatform){
        platformCaptureTrajectoryPlatform->assign(std::max(0, cfg.sensorCount), -1);
    }
    if(hasExternalExpectedForceTrajectory &&
            !externalExpectedForceTrajectory.empty() &&
            !externalExpectedForceTrajectoryTimeStamp.empty()){
        fromExternal = true;
        if(externalExpectedForceTrajectoryStartUs > 0){
            const qint64 elapsedUs = std::max<qint64>(
                        0,
                        monotonicNowUs() - externalExpectedForceTrajectoryStartUs);
            const double pvtTrajectoryTimeSec =
                    externalExpectedForceTrajectoryTimeStamp.front() +
                    static_cast<double>(elapsedUs) / 1000000.0;
            const ExpectedForceInterpolation interpolation =
                    interpolateExpectedForceTrajectory(cfg,
                                                       externalExpectedForceTrajectory,
                                                       externalExpectedForceTrajectoryTimeStamp,
                                                       externalExpectedRopeVelocityRadPerSecTrajectory,
                                                       externalExpectedRopeAccelerationRadPerSec2Trajectory,
                                                       pvtTrajectoryTimeSec);
            lastExternalExpectedForceTrajectoryValue = interpolation.force;
            lastExternalExpectedForceTrajectoryDerivative = interpolation.derivative;
            lastExternalExpectedForceTrajectoryRateFeedForwardScale =
                    interpolation.rateFeedForwardScale;
            lastExternalExpectedRopeVelocityRadPerSec =
                    interpolation.ropeVelocityRadPerSec;
            lastExternalExpectedRopeAccelerationRadPerSec2 =
                    interpolation.ropeAccelerationRadPerSec2;
            lastExternalExpectedForceTrajectoryPlatform =
                    interpolation.platformCaptureTrajectoryPlatform;
            hasLastExternalExpectedForceTrajectoryValue = true;
            if(expectedForceDerivative){
                *expectedForceDerivative = lastExternalExpectedForceTrajectoryDerivative;
            }
            if(expectedRateFeedForwardScale){
                *expectedRateFeedForwardScale =
                        lastExternalExpectedForceTrajectoryRateFeedForwardScale;
            }
            if(expectedRopeVelocityRadPerSec){
                *expectedRopeVelocityRadPerSec =
                        lastExternalExpectedRopeVelocityRadPerSec;
            }
            if(expectedRopeAccelerationRadPerSec2){
                *expectedRopeAccelerationRadPerSec2 =
                        lastExternalExpectedRopeAccelerationRadPerSec2;
            }
            if(platformCaptureTrajectoryPlatform){
                *platformCaptureTrajectoryPlatform = lastExternalExpectedForceTrajectoryPlatform;
            }
            return lastExternalExpectedForceTrajectoryValue;
        }
        if(hasLastExternalExpectedForceTrajectoryValue &&
                static_cast<int>(lastExternalExpectedForceTrajectoryValue.size()) >= cfg.sensorCount){
            if(expectedForceDerivative){
                *expectedForceDerivative = lastExternalExpectedForceTrajectoryDerivative;
            }
            if(expectedRateFeedForwardScale){
                *expectedRateFeedForwardScale =
                        lastExternalExpectedForceTrajectoryRateFeedForwardScale;
            }
            if(expectedRopeVelocityRadPerSec){
                *expectedRopeVelocityRadPerSec =
                        lastExternalExpectedRopeVelocityRadPerSec;
            }
            if(expectedRopeAccelerationRadPerSec2){
                *expectedRopeAccelerationRadPerSec2 =
                        lastExternalExpectedRopeAccelerationRadPerSec2;
            }
            if(platformCaptureTrajectoryPlatform){
                *platformCaptureTrajectoryPlatform = lastExternalExpectedForceTrajectoryPlatform;
            }
            return lastExternalExpectedForceTrajectoryValue;
        }
        const ExpectedForceInterpolation interpolation =
                interpolateExpectedForceTrajectory(cfg,
                                                   externalExpectedForceTrajectory,
                                                   externalExpectedForceTrajectoryTimeStamp,
                                                   externalExpectedRopeVelocityRadPerSecTrajectory,
                                                   externalExpectedRopeAccelerationRadPerSec2Trajectory,
                                                   externalExpectedForceTrajectoryTimeStamp.front());
        lastExternalExpectedForceTrajectoryValue = interpolation.force;
        lastExternalExpectedForceTrajectoryDerivative.assign(
                    std::max(0, cfg.sensorCount),
                    0.0);
        lastExternalExpectedForceTrajectoryRateFeedForwardScale =
                interpolation.rateFeedForwardScale;
        lastExternalExpectedRopeVelocityRadPerSec =
                interpolation.ropeVelocityRadPerSec;
        lastExternalExpectedRopeAccelerationRadPerSec2 =
                interpolation.ropeAccelerationRadPerSec2;
        lastExternalExpectedForceTrajectoryPlatform =
                interpolation.platformCaptureTrajectoryPlatform;
        hasLastExternalExpectedForceTrajectoryValue = true;
        if(expectedForceDerivative){
            *expectedForceDerivative = lastExternalExpectedForceTrajectoryDerivative;
        }
        if(expectedRateFeedForwardScale){
            *expectedRateFeedForwardScale =
                    lastExternalExpectedForceTrajectoryRateFeedForwardScale;
        }
        if(expectedRopeVelocityRadPerSec){
            *expectedRopeVelocityRadPerSec =
                    lastExternalExpectedRopeVelocityRadPerSec;
        }
        if(expectedRopeAccelerationRadPerSec2){
            *expectedRopeAccelerationRadPerSec2 =
                    lastExternalExpectedRopeAccelerationRadPerSec2;
        }
        if(platformCaptureTrajectoryPlatform){
            *platformCaptureTrajectoryPlatform = lastExternalExpectedForceTrajectoryPlatform;
        }
        return lastExternalExpectedForceTrajectoryValue;
    }
    if(hasExternalExpectedForce && static_cast<int>(externalExpectedForce.size()) == cfg.sensorCount){
        fromExternal = true;
        return externalExpectedForce;
    }
    fromExternal = false;
    return cfg.expectedForce;
}

bool ControlWorker::externalExpectedForceTrajectoryMotionActive(qint64 nowUs,
                                                                bool* trajectoryPresent) const
{
    QMutexLocker locker(&configMutex);
    const bool present =
            hasExternalExpectedForceTrajectory &&
            !externalExpectedForceTrajectory.empty() &&
            !externalExpectedForceTrajectoryTimeStamp.empty();
    if(trajectoryPresent){
        *trajectoryPresent = present;
    }
    if(!present || externalExpectedForceTrajectoryStartUs <= 0){
        return false;
    }

    const double startTimeSec = externalExpectedForceTrajectoryTimeStamp.front();
    const double endTimeSec = externalExpectedForceTrajectoryTimeStamp.back();
    if(!std::isfinite(startTimeSec) ||
            !std::isfinite(endTimeSec) ||
            endTimeSec <= startTimeSec){
        return false;
    }

    const qint64 elapsedUs = std::max<qint64>(
                0,
                nowUs - externalExpectedForceTrajectoryStartUs);
    const double trajectoryTimeSec =
            startTimeSec + static_cast<double>(elapsedUs) / 1000000.0;
    return std::isfinite(trajectoryTimeSec) && trajectoryTimeSec < endTimeSec;
}

std::vector<double> ControlWorker::applyForceSensorLowPass(const Config& cfg,
                                                           const std::vector<double>& rawValue,
                                                           double dtSec)
{
    if(!std::isfinite(cfg.forceSensorLowPassCutoffHz) || cfg.forceSensorLowPassCutoffHz <= 0.0){
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return rawValue;
    }

    double sampleHz = std::min(std::max(cfg.sensorSampleHz, 10.0), 2000.0);
    if(std::isfinite(dtSec) && dtSec > 0.0){
        sampleHz = std::min(std::max(1.0 / dtSec, 10.0), 2000.0);
    }
    const double cutoffHz = std::min(cfg.forceSensorLowPassCutoffHz, sampleHz * 0.45);
    if(!std::isfinite(cutoffHz) || cutoffHz <= 0.0){
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return rawValue;
    }

    if(!std::isfinite(dtSec) || dtSec <= 0.0){
        dtSec = 1.0 / sampleHz;
    }

    const double omegaDt = kTwoPi * cutoffHz * dtSec;
    const double alpha = std::min(1.0, std::max(0.0, omegaDt / (1.0 + omegaDt)));

    if(!hasFilteredForceSensorValue || filteredForceSensorValue.size() != rawValue.size()){
        filteredForceSensorValue = rawValue;
        hasFilteredForceSensorValue = true;
        return filteredForceSensorValue;
    }

    for(size_t i = 0; i < rawValue.size(); ++i){
        const double raw = rawValue[i];
        if(!std::isfinite(raw)){
            filteredForceSensorValue[i] = raw;
            continue;
        }
        if(!std::isfinite(filteredForceSensorValue[i])){
            filteredForceSensorValue[i] = raw;
            continue;
        }
        filteredForceSensorValue[i] += alpha * (raw - filteredForceSensorValue[i]);
    }

    return filteredForceSensorValue;
}

void ControlWorker::ensureTorqueCommandStateSize(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    if(static_cast<int>(lastTorqueCommandNm.size()) < axisCount){
        lastTorqueCommandNm.resize(axisCount, 0.0);
    }
    if(static_cast<int>(warmStartTorqueNm.size()) < axisCount){
        warmStartTorqueNm.resize(axisCount, 0.0);
    }
    if(static_cast<int>(torqueCommandBiasNm.size()) < axisCount){
        torqueCommandBiasNm.resize(axisCount, 0.0);
    }
    if(static_cast<int>(torqueCommandActive.size()) < axisCount){
        torqueCommandActive.resize(axisCount, false);
    }
    if(static_cast<int>(forceControlFaultLatched.size()) < axisCount){
        forceControlFaultLatched.resize(axisCount, false);
    }
    ensureForcePid0525HybridStateSize(axisCount);
}

void ControlWorker::ensureForcePid0525HybridStateSize(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    if(static_cast<int>(forcePid0525HybridState.size()) < axisCount){
        forcePid0525HybridState.resize(axisCount, ForcePid0525HybridState::Idle);
    }
    if(static_cast<int>(forcePid0525HybridBiasValid.size()) < axisCount){
        forcePid0525HybridBiasValid.resize(axisCount, false);
    }
    if(static_cast<int>(forcePid0525HybridHoldBiasNm.size()) < axisCount){
        forcePid0525HybridHoldBiasNm.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridCaptureForceN.size()) < axisCount){
        forcePid0525HybridCaptureForceN.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridStableTimeSec.size()) < axisCount){
        forcePid0525HybridStableTimeSec.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridBlendElapsedSec.size()) < axisCount){
        forcePid0525HybridBlendElapsedSec.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridBlendStartTorqueNm.size()) < axisCount){
        forcePid0525HybridBlendStartTorqueNm.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridRateInitialized.size()) < axisCount){
        forcePid0525HybridRateInitialized.resize(axisCount, false);
    }
    if(static_cast<int>(forcePid0525HybridLastActualForceN.size()) < axisCount){
        forcePid0525HybridLastActualForceN.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridLastExpectedForceN.size()) < axisCount){
        forcePid0525HybridLastExpectedForceN.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridActualForceRateFilteredNPerSec.size()) < axisCount){
        forcePid0525HybridActualForceRateFilteredNPerSec.resize(axisCount, 0.0);
    }
    if(static_cast<int>(forcePid0525HybridExpectedForceRateFilteredNPerSec.size()) < axisCount){
        forcePid0525HybridExpectedForceRateFilteredNPerSec.resize(axisCount, 0.0);
    }
}

void ControlWorker::resetForcePid0525HybridState(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    forcePid0525HybridState.assign(axisCount, ForcePid0525HybridState::Idle);
    forcePid0525HybridBiasValid.assign(axisCount, false);
    forcePid0525HybridHoldBiasNm.assign(axisCount, 0.0);
    forcePid0525HybridCaptureForceN.assign(axisCount, 0.0);
    forcePid0525HybridStableTimeSec.assign(axisCount, 0.0);
    forcePid0525HybridBlendElapsedSec.assign(axisCount, 0.0);
    forcePid0525HybridBlendStartTorqueNm.assign(axisCount, 0.0);
    forcePid0525HybridRateInitialized.assign(axisCount, false);
    forcePid0525HybridLastActualForceN.assign(axisCount, 0.0);
    forcePid0525HybridLastExpectedForceN.assign(axisCount, 0.0);
    forcePid0525HybridActualForceRateFilteredNPerSec.assign(axisCount, 0.0);
    forcePid0525HybridExpectedForceRateFilteredNPerSec.assign(axisCount, 0.0);
    lastForcePid0525HybridEnabled = false;
    lastForcePid0525DynamicTrackMode = ForcePid0525DynamicTrackMode::AC;
    lastForcePid0525UseBangBangPretension = true;
    lastForcePid0525HybridMotionActive = false;
}

void ControlWorker::resetForcePid0525HybridAxis(int axisIndex, bool preserveCurrentTorqueAsBias)
{
    if(axisIndex < 0){
        return;
    }
    ensureForcePid0525HybridStateSize(axisIndex + 1);
    forcePid0525HybridState[axisIndex] = ForcePid0525HybridState::Idle;
    forcePid0525HybridBiasValid[axisIndex] = false;
    forcePid0525HybridHoldBiasNm[axisIndex] = 0.0;
    forcePid0525HybridCaptureForceN[axisIndex] = 0.0;
    forcePid0525HybridStableTimeSec[axisIndex] = 0.0;
    forcePid0525HybridBlendElapsedSec[axisIndex] = 0.0;
    forcePid0525HybridBlendStartTorqueNm[axisIndex] = 0.0;
    forcePid0525HybridRateInitialized[axisIndex] = false;
    forcePid0525HybridLastActualForceN[axisIndex] = 0.0;
    forcePid0525HybridLastExpectedForceN[axisIndex] = 0.0;
    forcePid0525HybridActualForceRateFilteredNPerSec[axisIndex] = 0.0;
    forcePid0525HybridExpectedForceRateFilteredNPerSec[axisIndex] = 0.0;
    if(preserveCurrentTorqueAsBias &&
            axisIndex < static_cast<int>(lastTorqueCommandNm.size()) &&
            axisIndex < static_cast<int>(torqueCommandBiasNm.size()) &&
            std::isfinite(lastTorqueCommandNm[axisIndex])){
        torqueCommandBiasNm[axisIndex] = lastTorqueCommandNm[axisIndex];
    }
}

void ControlWorker::refreshForceControlFaultLatches(const Config& cfg, bool forceThreadRunning)
{
    ensureTorqueCommandStateSize(cfg.axisCount);
    if(!forceThreadRunning){
        std::fill(forceControlFaultLatched.begin(),
                  forceControlFaultLatched.end(),
                  false);
        return;
    }

    const int axisCount = std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
    for(int axisIndex=0; axisIndex<static_cast<int>(forceControlFaultLatched.size()); ++axisIndex){
        const bool axisStillForceControlled =
                axisIndex < axisCount &&
                cfg.axes[axisIndex].isMotorAxis &&
                cfg.axes[axisIndex].forceControlEnabled;
        if(!axisStillForceControlled){
            forceControlFaultLatched[axisIndex] = false;
        }
    }
}

void ControlWorker::ensureForceFeedbackStateSize(int sensorCount)
{
    forceController.ensureChannelCount(sensorCount);
    if(sensorCount <= 0){
        unloadFeedForwardBlend.clear();
        forceFeedForwardOnlyStaticFrictionDirectionSign.clear();
        forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm.clear();
        forceFeedForwardOnlyStaticFrictionSmoothedValid.clear();
        return;
    }
    if(static_cast<int>(unloadFeedForwardBlend.size()) < sensorCount){
        unloadFeedForwardBlend.resize(sensorCount, 0.0);
    }
    if(static_cast<int>(forceFeedForwardOnlyStaticFrictionDirectionSign.size()) < sensorCount){
        forceFeedForwardOnlyStaticFrictionDirectionSign.resize(sensorCount, 0);
    }
    if(static_cast<int>(forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm.size()) < sensorCount){
        forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm.resize(sensorCount, 0.0);
    }
    if(static_cast<int>(forceFeedForwardOnlyStaticFrictionSmoothedValid.size()) < sensorCount){
        forceFeedForwardOnlyStaticFrictionSmoothedValid.resize(sensorCount, false);
    }
}

void ControlWorker::resetForceFeedbackState(int sensorCount)
{
    forceController.reset(sensorCount);
    forcePid0525.resetAll();
    unloadFeedForwardBlend.assign(std::max(sensorCount, 0), 0.0);
    forceFeedForwardOnlyStaticFrictionDirectionSign.assign(std::max(sensorCount, 0), 0);
    forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm.assign(std::max(sensorCount, 0), 0.0);
    forceFeedForwardOnlyStaticFrictionSmoothedValid.assign(std::max(sensorCount, 0), false);
}

void ControlWorker::resetForceFeedbackChannel(int sensorIndex)
{
    forceController.resetChannel(sensorIndex);
    if(sensorIndex >= 0){
        forcePid0525.resetChannel(static_cast<size_t>(sensorIndex));
        if(sensorIndex < static_cast<int>(unloadFeedForwardBlend.size())){
            unloadFeedForwardBlend[sensorIndex] = 0.0;
        }
        if(sensorIndex < static_cast<int>(forceFeedForwardOnlyStaticFrictionDirectionSign.size())){
            forceFeedForwardOnlyStaticFrictionDirectionSign[sensorIndex] = 0;
        }
        if(sensorIndex < static_cast<int>(forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm.size())){
            forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm[sensorIndex] = 0.0;
        }
        if(sensorIndex < static_cast<int>(forceFeedForwardOnlyStaticFrictionSmoothedValid.size())){
            forceFeedForwardOnlyStaticFrictionSmoothedValid[sensorIndex] = false;
        }
    }
}

void ControlWorker::ensureActualTorqueLimitStateSize(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    if(static_cast<int>(actualTorqueLimitOverStartUs.size()) != axisCount){
        actualTorqueLimitOverStartUs.assign(axisCount, 0);
        actualTorqueLimitLastSampleUs.assign(axisCount, 0);
        actualTorqueLimitPeakNm.assign(axisCount, 0.0);
    }
}

void ControlWorker::resetActualTorqueLimitState(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    actualTorqueLimitOverStartUs.assign(axisCount, 0);
    actualTorqueLimitLastSampleUs.assign(axisCount, 0);
    actualTorqueLimitPeakNm.assign(axisCount, 0.0);
    actualTorqueLimitEmergencyStopActive = false;
}

void ControlWorker::resetTorqueCommandState(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    lastTorqueCommandNm.assign(axisCount, 0.0);
    warmStartTorqueNm.assign(axisCount, 0.0);
    torqueCommandBiasNm.assign(axisCount, 0.0);
    torqueCommandActive.assign(axisCount, false);
    forceControlFaultLatched.assign(axisCount, false);
    resetForcePid0525HybridState(axisCount);
}

void ControlWorker::stopActiveTorqueCommands(const Config& cfg)
{
    ensureTorqueCommandStateSize(cfg.axisCount);
    if(!hardwareInterface){
        resetTorqueCommandState(cfg.axisCount);
        return;
    }

    const int axisCount = static_cast<int>(torqueCommandActive.size());
    for(int axisIndex = 0; axisIndex < axisCount; ++axisIndex){
        if(!torqueCommandActive[axisIndex]){
            continue;
        }
        if(isProtectedMotionAxis(cfg, axisIndex)){
            continue;
        }
        hardwareInterface->motorStop(axisIndex);
        if(axisIndex < static_cast<int>(warmStartTorqueNm.size()) &&
                std::isfinite(lastTorqueCommandNm[axisIndex])){
            warmStartTorqueNm[axisIndex] = lastTorqueCommandNm[axisIndex];
        }
        torqueCommandActive[axisIndex] = false;
        lastTorqueCommandNm[axisIndex] = 0.0;
        if(axisIndex < static_cast<int>(torqueCommandBiasNm.size())){
            torqueCommandBiasNm[axisIndex] = 0.0;
        }
        resetForcePid0525HybridAxis(axisIndex, false);
        if(axisIndex < static_cast<int>(cfg.axes.size())){
            resetForceFeedbackChannel(cfg.axes[axisIndex].sensorIndex);
        }
    }
}

bool ControlWorker::stopTorqueModeAxis(int axisIndex)
{
    if(!hardwareInterface || axisIndex < 0){
        return false;
    }
    ensureTorqueCommandStateSize(axisIndex + 1);
    if(axisIndex < static_cast<int>(warmStartTorqueNm.size()) &&
            std::isfinite(lastTorqueCommandNm[axisIndex])){
        warmStartTorqueNm[axisIndex] = lastTorqueCommandNm[axisIndex];
    }
    if(torqueCommandActive[axisIndex]){
        hardwareInterface->motorStop(axisIndex);
    }
    torqueCommandActive[axisIndex] = false;
    lastTorqueCommandNm[axisIndex] = 0.0;
    if(axisIndex < static_cast<int>(torqueCommandBiasNm.size())){
        torqueCommandBiasNm[axisIndex] = 0.0;
    }
    resetForcePid0525HybridAxis(axisIndex, false);
    return true;
}

double ControlWorker::warmStartTorqueForAxis(int axisIndex,
                                             const std::vector<double>& motorTorqueNm,
                                             double torqueLimitNm) const
{
    double torqueNm = 0.0;
    bool hasWarmStartTorque = false;
    if(axisIndex >= 0 &&
            axisIndex < static_cast<int>(motorTorqueNm.size()) &&
            std::isfinite(motorTorqueNm[axisIndex]) &&
            std::fabs(motorTorqueNm[axisIndex]) > kTorqueCommandEpsilonNm){
        torqueNm = motorTorqueNm[axisIndex];
        hasWarmStartTorque = true;
    }
    if(!hasWarmStartTorque &&
            axisIndex >= 0 &&
            axisIndex < static_cast<int>(warmStartTorqueNm.size()) &&
            std::isfinite(warmStartTorqueNm[axisIndex])){
        torqueNm = warmStartTorqueNm[axisIndex];
        hasWarmStartTorque = std::fabs(torqueNm) > kTorqueCommandEpsilonNm;
    }
    if(!hasWarmStartTorque &&
            axisIndex >= 0 &&
            axisIndex < static_cast<int>(lastTorqueCommandNm.size()) &&
            std::isfinite(lastTorqueCommandNm[axisIndex])){
        torqueNm = lastTorqueCommandNm[axisIndex];
    }

    const double limit = std::max(0.0, torqueLimitNm);
    if(limit > 0.0){
        torqueNm = std::min(std::max(torqueNm, -limit), limit);
    }
    return torqueNm;
}

bool ControlWorker::commandTorqueModeAxis(int axisIndex, double torqueNm)
{
    if(!hardwareInterface || axisIndex < 0){
        return false;
    }
    ensureTorqueCommandStateSize(axisIndex + 1);

    const double previousTorqueNm = lastTorqueCommandNm[axisIndex];
    const bool hadPreviousTorque =
            torqueCommandActive[axisIndex] && std::isfinite(previousTorqueNm);
    auto rememberPreviousTorque = [&](){
        if(hadPreviousTorque && axisIndex < static_cast<int>(warmStartTorqueNm.size())){
            warmStartTorqueNm[axisIndex] = previousTorqueNm;
        }
    };

    if(!std::isfinite(torqueNm)){
        stopTorqueModeAxis(axisIndex);
        return false;
    }
    if(std::fabs(torqueNm) <= kTorqueCommandEpsilonNm){
        if(!torqueCommandActive[axisIndex]){
            lastTorqueCommandNm[axisIndex] = 0.0;
            return true;
        }
        const bool ok = hardwareInterface->motorTorqueChange(axisIndex, 0.0);
        if(ok){
            rememberPreviousTorque();
            torqueCommandActive[axisIndex] = true;
            lastTorqueCommandNm[axisIndex] = 0.0;
            return true;
        }
        stopTorqueModeAxis(axisIndex);
        return false;
    }

    const bool shouldStart = !torqueCommandActive[axisIndex];
    const bool ok = shouldStart ?
                hardwareInterface->motorTorqueStart(axisIndex, torqueNm) :
                hardwareInterface->motorTorqueChange(axisIndex, torqueNm);
    if(ok){
        torqueCommandActive[axisIndex] = true;
        lastTorqueCommandNm[axisIndex] = torqueNm;
        warmStartTorqueNm[axisIndex] = torqueNm;
    }
    else{
        rememberPreviousTorque();
        hardwareInterface->motorStop(axisIndex);
        torqueCommandActive[axisIndex] = false;
        lastTorqueCommandNm[axisIndex] = 0.0;
        if(axisIndex < static_cast<int>(torqueCommandBiasNm.size())){
            torqueCommandBiasNm[axisIndex] = 0.0;
        }
    }
    return ok;
}

void ControlWorker::controlLoop()
{
    // Precise pacing still uses the steady clock; slower modes let QTimer
    // sleep instead of spinning the worker event loop.
    const qint64 loopNowUs = monotonicNowUs();
    // 用户退出不再依赖ControlWorker事件队列。优先于周期门控和下一次Trace
    // 读取消费停机邮箱，避免1 ms定时循环或HardwareThread拥塞让UI同步卡住。
    if(consumeEndpointRemoteStopRequest()){
        previousControlLoopDurationUs = std::max<qint64>(
                    0, monotonicNowUs() - loopNowUs);
        return;
    }
    const Config cfg = currentConfig();
    qint64 targetIntervalUs = traceDrivenWorkerIntervalUs(cfg);
    if(onlineVelocityControl.isActive()){
        // 在线速度控制可选择 1/2/5/10/20 ms。原有力控 worker 在部分配置下
        // 固定为 5 ms，因此在线会话期间需要把外层循环收紧到所选周期；
        // 较慢的在线周期仍由 OnlineVelocityControl 自身的 nextDueUs 门控。
        targetIntervalUs = std::min<qint64>(
                    targetIntervalUs,
                    onlineVelocityControl.currentConfig().periodUs);
    }
    if(endpointRemoteControl.isActive()){
        const qint64 remotePeriodUs = endpointRemoteControl.currentConfig()
                .onlineVelocity.periodUs;
        // 遥控速度命令采用“本轮纯规划、下轮复合Trace/提交”两阶段。外层worker
        // 至少以半个在线周期运行，保持实际SDK更新周期仍等于配置周期。
        targetIntervalUs = std::min<qint64>(
                    targetIntervalUs,
                    std::max<qint64>(1000, remotePeriodUs / 2));
    }
    const double targetIntervalSec = static_cast<double>(targetIntervalUs) / 1000000.0;
    if(lastControlLoopSampleIntervalUs != targetIntervalUs){
        lastControlLoopSampleIntervalUs = targetIntervalUs;
        nextControlLoopDueUs = loopNowUs;
    }
    if(nextControlLoopDueUs <= 0){
        nextControlLoopDueUs = loopNowUs;
    }
    if(loopNowUs < nextControlLoopDueUs){
        return;
    }
    while(nextControlLoopDueUs <= loopNowUs){
        nextControlLoopDueUs += targetIntervalUs;
    }

    if(!hardwareInterface){
        return;
    }

    const qint64 loopWallClockMs = QDateTime::currentMSecsSinceEpoch();
    timingDiagnostics.controlLoopTickCount++;
    if(lastControlLoopTimestampUs > 0){
        const qint64 dtUs = std::max<qint64>(0, loopNowUs - lastControlLoopTimestampUs);
        timingDiagnostics.controlLoopIntervalCount++;
        timingDiagnostics.controlLoopIntervalSumUs += dtUs;
        timingDiagnostics.latestControlLoopIntervalUs = dtUs;
        if(kEnableControlWorkerDiagnosticRawHistory){
            const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
            if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                               loopWallClockMs * 1000,
                                               lastControlLoopRawHistoryAppendUs)){
                QMutexLocker locker(&timingHistoryMutex);
                controlLoopRawHistory.append({loopWallClockMs, dtUs});
                if(loopWallClockMs - lastControlLoopHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                    trimRawHistoryForMode(controlLoopRawHistory,
                                          loopWallClockMs,
                                          fullRawRecording,
                                          kDiagnosticRawDefaultMaxSamples);
                    lastControlLoopHistoryTrimMs = loopWallClockMs;
                }
            }
        }
    }
    lastControlLoopTimestampUs = loopNowUs;

    ensureTorqueCommandStateSize(cfg.axisCount);
    ensureActualTorqueLimitStateSize(cfg.axisCount);
    const bool leadshineConnected = cfg.useLeadshine && hardwareInterface->isLSConnected();
    if(timer){
        const int desiredTimerIntervalMs = workerTimerIntervalMs(targetIntervalUs);
        if(timer->interval() != desiredTimerIntervalMs){
            timer->setInterval(desiredTimerIntervalMs);
        }
    }

    std::vector<double> motorAbsPos;
    std::vector<double> motorRelRawPos;
    std::vector<double> motorVel;
    std::vector<double> motorTorqueNm;
    std::vector<double> motorCommand(std::max(cfg.axisCount, 0), 0.0);
    std::vector<double> forceSensorValue;
    std::vector<double> expectedForce;
    bool expectedFromExternal = false;
    bool runtimeTraceFeedbackSafe = false;
    qint64 traceReadDurationUs = 0;

    HardwareInterface::RuntimeTraceSnapshot traceSnapshot;
    bool endpointRemoteProcessed = false;
    const auto processEndpointRemoteController = [&](){
        if(endpointRemoteProcessed){
            return;
        }
        // SafetyMonitor可能在本轮Trace读取期间已经触发硬件急停并先提交
        // 停机邮箱。这里再次消费，避免等到下一轮才发现并继续生成速度命令。
        if(consumeEndpointRemoteStopRequest()){
            endpointRemoteProcessed = true;
            return;
        }
        const qint64 endpointRemoteDispatchUs = monotonicNowUs();
        EndpointRemoteTimingContext endpointRemoteTiming;
        endpointRemoteTiming.workerLoopEntryUs = loopNowUs;
        endpointRemoteTiming.workerLoopIntervalUs =
                timingDiagnostics.latestControlLoopIntervalUs;
        endpointRemoteTiming.traceReadDurationUs = traceReadDurationUs;
        endpointRemoteTiming.preDispatchDurationUs = std::max<qint64>(
                    0, endpointRemoteDispatchUs - loopNowUs);
        endpointRemoteTiming.previousWorkerLoopDurationUs =
                previousControlLoopDurationUs;
        processEndpointRemoteControl(cfg,
                                     traceSnapshot,
                                     endpointRemoteDispatchUs,
                                     endpointRemoteTiming);
        endpointRemoteProcessed = true;
    };
    if(leadshineConnected){
        bool useCompositeEndpointRemoteCommand =
                endpointRemoteControl.isActive() &&
                endpointRemoteTracePhase == EndpointRemoteTracePhase::Running &&
                endpointRemoteDispatchPhase == EndpointRemoteDispatchPhase::
                    PreparedForCompositeTraceCommand &&
                cfg.systemRunning && cfg.useLeadshine &&
                !cfg.forceThreadEnabled && !cfg.pvtActiveOrPaused &&
                !cfg.commissioningModeActive &&
                endpointRemoteInputSessionToken() != 0;
        if(useCompositeEndpointRemoteCommand){
            const qint64 planningAgeUs =
                    preparedEndpointRemoteStep.planningCompletedUs > 0 ?
                        std::max<qint64>(
                            0, loopNowUs -
                                preparedEndpointRemoteStep.planningCompletedUs) : 0;
            const qint64 maximumPlanningAgeUs = std::max<qint64>(
                        1,
                        2 * endpointRemoteControl.currentConfig()
                            .onlineVelocity.periodUs);
            if(planningAgeUs > maximumPlanningAgeUs){
                const bool activeMotion =
                        preparedEndpointRemoteStep.actuationProfile !=
                            EndpointRemoteActuationProfile::StartingJog;
                stopEndpointRemoteControl(
                            activeMotion,
                            QStringLiteral(
                                "末端遥控待提交规划已过期%1 us（上限%2 us），禁止迟到命令进入HardwareThread")
                                .arg(planningAgeUs)
                                .arg(maximumPlanningAgeUs));
                useCompositeEndpointRemoteCommand = false;
            }
        }
        if(useCompositeEndpointRemoteCommand){
            // 在进入不可分割的HardwareThread任务前先消费最新输入。尚未启动JOG时，
            // 若方向或UI租约已经变化，可安全丢弃旧规划；活动运动中的减速/更新不撤销。
            consumeEndpointRemoteInputMailbox();
            if(endpointRemoteControl.cancelPreparedCommandIfInputChanged(
                        preparedEndpointRemoteStep,
                        loopNowUs)){
                clearPreparedEndpointRemoteCommand();
                useCompositeEndpointRemoteCommand = false;
            }
        }

        // 每个ControlWorker周期仍只读一次Trace：普通周期直接读取；有待下发命令时，
        // 由同一个HardwareThread任务完成“读取Trace -> 校验 -> 立即调用速度SDK”。
        const qint64 traceReadStartUs = monotonicNowUs();
        if(useCompositeEndpointRemoteCommand){
            const std::vector<int> axes{0, 1, 2, 3, 4, 5, 6, 7};
            const std::vector<double> commandVelocity(
                        preparedEndpointRemoteStep.commandVelocity.begin(),
                        preparedEndpointRemoteStep.commandVelocity.end());
            const HardwareInterface::EndpointRemoteCommandAdmissionProfile
                    admissionProfile =
                    preparedEndpointRemoteStep.actuationProfile ==
                        EndpointRemoteActuationProfile::StartingJog ?
                        HardwareInterface::EndpointRemoteCommandAdmissionProfile::
                            StartFromConfirmedZero :
                        HardwareInterface::EndpointRemoteCommandAdmissionProfile::
                            ActiveMotion;
            const HardwareInterface::EndpointRemoteTraceCommandResult
                    commandResult =
                    hardwareInterface->
                        readRuntimeTraceAndMotorVelBatchFastEndpointRemote(
                            axes,
                            commandVelocity,
                            endpointRemoteControl.currentConfig()
                                .onlineVelocity.onlineChangeTimeSec,
                            endpointRemoteControl.currentConfig()
                                .onlineVelocity.traceFeedbackDelayLimitUs(),
                            endpointRemoteInputSessionToken(),
                            preparedEndpointRemoteStep.logicalFrameSequence,
                            admissionProfile,
                            preparedEndpointRemoteStep.planningStartedUs,
                            preparedEndpointRemoteStep.planningCompletedUs,
                            static_cast<int>(
                                preparedEndpointRemoteStep.actuationProfile));
            traceSnapshot = commandResult.traceSnapshot;
            completePreparedEndpointRemoteCommand(commandResult,
                                                  monotonicNowUs(),
                                                  loopNowUs);
        }
        else{
            traceSnapshot = hardwareInterface->readRuntimeTraceLatestSnapshot();
        }
        const qint64 traceReadCompleteUs = monotonicNowUs();
        traceReadDurationUs = traceSnapshot.totalReadCallUs > 0 ?
                    traceSnapshot.totalReadCallUs :
                    std::max<qint64>(
                        0, traceReadCompleteUs - traceReadStartUs);
        const qint64 traceReadCompleteWallClockUs =
                QDateTime::currentMSecsSinceEpoch() * 1000;
        // 在线速度命令只依赖刚取得的同帧Trace。把它提升到读取完成后的
        // 第一优先级，避免后续力传感器整理和普通快照维护侵占在线周期预算。
        processEndpointRemoteController();
        const qint64 traceFutureToleranceUs = std::max<qint64>(
                    2 * 1000,
                    static_cast<qint64>(traceSnapshot.traceSamplePeriodUs) * 4);
        const bool traceFrameFresh = traceSnapshot.monotonicUs > 0 &&
                traceReadCompleteUs + traceFutureToleranceUs >=
                    traceSnapshot.monotonicUs &&
                (traceReadCompleteUs < traceSnapshot.monotonicUs ||
                 traceReadCompleteUs - traceSnapshot.monotonicUs <=
                    kTorqueForceSensorTimeoutUs) &&
                traceSnapshot.newestFrameAgeUs >= 0 &&
                traceSnapshot.newestFrameAgeUs <= kTorqueForceSensorTimeoutUs;
        runtimeTraceFeedbackSafe = traceSnapshot.fromTrace &&
                traceSnapshot.timingReliable &&
                !traceSnapshot.traceLost &&
                traceFrameFresh;
        const qint64 traceReadCallIntervalUs = lastSensorTraceReadCallUs > 0 ?
                    std::max<qint64>(0, traceReadCompleteUs - lastSensorTraceReadCallUs) :
                    0;
        lastSensorTraceReadCallUs = traceReadCompleteUs;
        if(kEnableControlWorkerDiagnosticRawHistory){
            const qint64 traceReadCompleteWallClockMs = traceReadCompleteWallClockUs / 1000;
            const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
            if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                               traceReadCompleteWallClockUs,
                                               lastSensorTraceReadRawHistoryAppendUs)){
                QMutexLocker locker(&timingHistoryMutex);
                sensorTraceReadRawHistory.append(DiagnosticRawSample{traceReadCompleteWallClockMs,
                                                                     traceReadCallIntervalUs,
                                                                     traceReadCompleteWallClockUs});
                if(traceReadCompleteWallClockMs - lastSensorTraceReadHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                    trimRawHistoryForMode(sensorTraceReadRawHistory,
                                          traceReadCompleteWallClockMs,
                                          fullRawRecording,
                                          kDiagnosticRawDefaultMaxSamples);
                    lastSensorTraceReadHistoryTrimMs = traceReadCompleteWallClockMs;
                }
            }
        }

        motorAbsPos = traceSnapshot.motorPosition;
        const int axisCount = std::min(cfg.axisCount, static_cast<int>(motorAbsPos.size()));
        motorRelRawPos.resize(axisCount, 0.0);
        if(static_cast<int>(traceSnapshot.motorSafetyRelativePosition.size()) >= axisCount){
            for(int i=0; i<axisCount; ++i){
                motorRelRawPos[i] = traceSnapshot.motorSafetyRelativePosition[i];
            }
        }
        else{
            if(static_cast<int>(cachedMotorHome.size()) < axisCount ||
                    lastMotorHomeRefreshUs <= 0 ||
                    loopNowUs - lastMotorHomeRefreshUs >= kMotorHomeRefreshIntervalUs){
                cachedMotorHome = hardwareInterface->getAllMotorSafetyHomeUnit();
                lastMotorHomeRefreshUs = loopNowUs;
            }
            const std::vector<double> motorHome = cachedMotorHome;
            for(int i=0; i<axisCount; ++i){
                if(i < static_cast<int>(motorHome.size())){
                    motorRelRawPos[i] = motorAbsPos[i] - motorHome[i];
                }
                else{
                    motorRelRawPos[i] = std::numeric_limits<double>::quiet_NaN();
                }
            }
        }
        if(static_cast<int>(cachedMotorVel.size()) < axisCount){
            cachedMotorVel.assign(axisCount, 0.0);
        }
        if(traceSnapshot.frameCount > 0 && axisCount > 0){
            const qint64 velocityTimestampUs = traceSnapshot.monotonicUs > 0 ?
                        traceSnapshot.monotonicUs :
                        traceReadCompleteUs;
            if(static_cast<int>(lastMotorVelocityPosition.size()) >= axisCount &&
                    lastMotorVelocityRefreshUs > 0 &&
                    velocityTimestampUs > lastMotorVelocityRefreshUs){
                const double velocityDtSec =
                        static_cast<double>(velocityTimestampUs - lastMotorVelocityRefreshUs) / 1000000.0;
                if(std::isfinite(velocityDtSec) && velocityDtSec > 0.0){
                    for(int i=0; i<axisCount; ++i){
                        const double currentPosition = motorAbsPos[i];
                        const double previousPosition = lastMotorVelocityPosition[i];
                        cachedMotorVel[i] =
                                std::isfinite(currentPosition) && std::isfinite(previousPosition) ?
                                    (currentPosition - previousPosition) / velocityDtSec :
                                    0.0;
                    }
                }
            }
            lastMotorVelocityPosition = motorAbsPos;
            if(static_cast<int>(lastMotorVelocityPosition.size()) > axisCount){
                lastMotorVelocityPosition.resize(axisCount);
            }
            lastMotorVelocityRefreshUs = velocityTimestampUs;
        }
        motorVel = cachedMotorVel;
        if(static_cast<int>(traceSnapshot.motorActualVelocity.size()) >= axisCount){
            for(int axis = 0; axis < axisCount; ++axis){
                if(std::isfinite(traceSnapshot.motorActualVelocity[axis])){
                    motorVel[axis] = traceSnapshot.motorActualVelocity[axis];
                }
            }
        }
        if(static_cast<int>(motorVel.size()) > axisCount){
            motorVel.resize(axisCount);
        }

        cachedMotorTorqueNm = traceSnapshot.motorTorqueNm;
        lastMotorTorqueRefreshUs = traceReadCompleteUs;
        motorTorqueNm = cachedMotorTorqueNm;
        if(static_cast<int>(motorTorqueNm.size()) > axisCount){
            motorTorqueNm.resize(axisCount);
        }

        auto acceptForceSensorFeedback =
                [&](std::vector<double> feedbackValue,
                    int frameCount,
                    qint64 frameMonotonicUs,
                    qint64 frameWallClockUs,
                    qint64 readCompleteUs,
                    qint64 readCompleteWallClockUs,
                    bool fromTrace,
                    bool expandedTraceFrame) -> bool {
            if(feedbackValue.empty()){
                return false;
            }
            forceSensorValue = std::move(feedbackValue);
            if(static_cast<int>(forceSensorValue.size()) < cfg.sensorCount){
                forceSensorValue.resize(cfg.sensorCount, 0.0);
            }
            const bool hasNewSensorFrame = frameCount > 0;
            const qint64 effectiveFrameMonotonicUs =
                    frameMonotonicUs > 0 ? frameMonotonicUs : readCompleteUs;
            const qint64 effectiveFrameWallClockUs =
                    frameWallClockUs > 0 ? frameWallClockUs : readCompleteWallClockUs;
            qint64 sensorDtUs = 0;
            if(hasNewSensorFrame){
                timingDiagnostics.sensorFrameCount++;
                qint64 traceExpandedSensorDtUs = 0;
                bool hasPreviousTraceExpandedSensorFrame = false;
                if(expandedTraceFrame && effectiveFrameMonotonicUs > 0){
                    hasPreviousTraceExpandedSensorFrame =
                            lastTraceExpandedSensorFrameTimestampUs > 0;
                    if(hasPreviousTraceExpandedSensorFrame){
                        traceExpandedSensorDtUs =
                                std::max<qint64>(
                                    0,
                                    effectiveFrameMonotonicUs -
                                    lastTraceExpandedSensorFrameTimestampUs);
                    }
                    lastTraceExpandedSensorFrameTimestampUs = effectiveFrameMonotonicUs;
                }
                if(lastSensorFrameTimestampUs > 0){
                    sensorDtUs =
                            std::max<qint64>(0, effectiveFrameMonotonicUs - lastSensorFrameTimestampUs);
                    timingDiagnostics.sensorFrameIntervalCount++;
                    timingDiagnostics.sensorFrameIntervalSumUs += sensorDtUs;
                    timingDiagnostics.latestSensorFrameIntervalUs = sensorDtUs;
                    const bool writeFrameHistory =
                            !expandedTraceFrame || hasPreviousTraceExpandedSensorFrame;
                    if(kEnableControlWorkerDiagnosticRawHistory && writeFrameHistory){
                        const bool fullRawRecording =
                                diagnosticRawHistoryFullRecordingEnabled.load();
                        if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                                           effectiveFrameWallClockUs,
                                                           lastSensorFrameRawHistoryAppendUs)){
                            QMutexLocker locker(&timingHistoryMutex);
                            sensorFrameRawHistory.append(DiagnosticRawSample{effectiveFrameWallClockUs / 1000,
                                                                             expandedTraceFrame ?
                                                                                 traceExpandedSensorDtUs :
                                                                                 sensorDtUs,
                                                                             effectiveFrameWallClockUs,
                                                                             fromTrace,
                                                                             expandedTraceFrame});
                        }
                    }
                }
                lastSensorFrameTimestampUs = effectiveFrameMonotonicUs;
                lastSensorFrameWallClockUs = effectiveFrameWallClockUs;
            }

            const double filterDtSec =
                    sensorDtUs > 0 ? static_cast<double>(sensorDtUs) / 1000000.0 : 0.0;
            forceSensorValue = applyForceSensorLowPass(cfg, forceSensorValue, filterDtSec);
            latestForceSensorValue = forceSensorValue;
            hasLatestForceSensorValue = true;
            if(kEnableControlWorkerDiagnosticRawHistory && hasNewSensorFrame){
                const qint64 sampleWallClockUs = lastSensorFrameWallClockUs > 0 ?
                            lastSensorFrameWallClockUs :
                            traceReadCompleteWallClockUs;
                const qint64 sampleWallClockMs = sampleWallClockUs / 1000;
                const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
                if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                                   sampleWallClockUs,
                                                   lastSensorValueRawHistoryAppendUs)){
                    QMutexLocker locker(&timingHistoryMutex);
                    sensorValueRawHistory.append(SensorValueSample{sampleWallClockMs,
                                                                   latestForceSensorValue,
                                                                   sampleWallClockUs,
                                                                   fromTrace,
                                                                   expandedTraceFrame});
                    if(sampleWallClockMs - lastSensorFrameHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                        trimRawHistoryForMode(sensorFrameRawHistory,
                                              sampleWallClockMs,
                                              fullRawRecording,
                                              kDiagnosticRawDefaultMaxSamples);
                        lastSensorFrameHistoryTrimMs = sampleWallClockMs;
                    }
                    if(sampleWallClockMs - lastSensorValueHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                        trimRawHistoryForMode(sensorValueRawHistory,
                                              sampleWallClockMs,
                                              fullRawRecording,
                                              kDiagnosticSensorValueDefaultMaxSamples);
                        lastSensorValueHistoryTrimMs = sampleWallClockMs;
                    }
                }
            }
            return hasNewSensorFrame;
        };

        const bool hasSnapshotForceValue = !traceSnapshot.forceSensorValue.empty();
        const bool snapshotHasNewSensorFrame = traceSnapshot.frameCount > 0;
        bool acceptedForceSensorFeedback = false;
        bool acceptedNewSensorFrame = false;
        for(const HardwareInterface::ForceSensorTraceSample& traceSample :
            traceSnapshot.forceSensorTraceSamples){
            if(traceSample.values.empty()){
                continue;
            }
            acceptedNewSensorFrame =
                    acceptForceSensorFeedback(traceSample.values,
                                              1,
                                              traceSample.monotonicUs,
                                              traceSample.wallClockUs,
                                              traceReadCompleteUs,
                                              traceReadCompleteWallClockUs,
                                              true,
                                              true) ||
                    acceptedNewSensorFrame;
            acceptedForceSensorFeedback = true;
        }
        if(!acceptedNewSensorFrame && hasSnapshotForceValue && snapshotHasNewSensorFrame){
            acceptedNewSensorFrame =
                    acceptForceSensorFeedback(traceSnapshot.forceSensorValue,
                                              traceSnapshot.frameCount,
                                              traceSnapshot.monotonicUs,
                                              traceSnapshot.wallClockUs,
                                              traceReadCompleteUs,
                                              traceReadCompleteWallClockUs,
                                              traceSnapshot.fromTrace,
                                              false);
            acceptedForceSensorFeedback = true;
        }

        if(!acceptedNewSensorFrame &&
                !traceSnapshot.fromTrace &&
                cfg.forceThreadEnabled &&
                cfg.sensorCount > 0){
            const HardwareInterface::ForceSensorReadResult fallbackSensorRead =
                    hardwareInterface->readForceSensorDataCachedResult(1);
            if(!fallbackSensorRead.values.empty()){
                const qint64 fallbackReadCompleteUs = monotonicNowUs();
                const qint64 fallbackReadCompleteWallClockUs =
                        QDateTime::currentMSecsSinceEpoch() * 1000;
                acceptedNewSensorFrame =
                        acceptForceSensorFeedback(fallbackSensorRead.values,
                                                  fallbackSensorRead.frameCount,
                                                  fallbackSensorRead.frameCount > 0 ?
                                                      fallbackReadCompleteUs : 0,
                                                  fallbackSensorRead.frameCount > 0 ?
                                                      fallbackReadCompleteWallClockUs : 0,
                                                  fallbackReadCompleteUs,
                                                  fallbackReadCompleteWallClockUs,
                                                  fallbackSensorRead.fromTrace,
                                                  false);
                acceptedForceSensorFeedback = true;
            }
        }

        if(!acceptedForceSensorFeedback && hasSnapshotForceValue){
            acceptForceSensorFeedback(traceSnapshot.forceSensorValue,
                                      traceSnapshot.frameCount,
                                      traceSnapshot.monotonicUs,
                                      traceSnapshot.wallClockUs,
                                      traceReadCompleteUs,
                                      traceReadCompleteWallClockUs,
                                      traceSnapshot.fromTrace,
                                      false);
            acceptedForceSensorFeedback = true;
        }
        else if(!acceptedForceSensorFeedback && hasLatestForceSensorValue){
            forceSensorValue = latestForceSensorValue;
        }
        if(static_cast<int>(forceSensorValue.size()) < cfg.sensorCount){
            forceSensorValue.resize(cfg.sensorCount, 0.0);
        }
    }

    // 期望力可能来自 UI 静态值，也可能来自外部轨迹/力位混合模式；统一在这里做通道补齐和上下限裁剪。
    processOnlineVelocityControl(cfg, traceSnapshot, loopNowUs);
    // 未连接时仍运行一次遥控互锁/故障判定；已连接路径在Trace读取完成后
    // 已优先执行，这里不会重复下发。
    processEndpointRemoteController();
    std::vector<double> expectedForceDerivative;
    std::vector<double> expectedRateFeedForwardScale;
    std::vector<double> expectedRopeVelocityRadPerSec;
    std::vector<double> expectedRopeAccelerationRadPerSec2;
    std::vector<int> platformCaptureTrajectoryPlatform;
    expectedForce = activeExpectedForce(cfg,
                                        expectedFromExternal,
                                        &expectedForceDerivative,
                                        &expectedRateFeedForwardScale,
                                        &expectedRopeVelocityRadPerSec,
                                        &expectedRopeAccelerationRadPerSec2,
                                        &platformCaptureTrajectoryPlatform);
    if(static_cast<int>(expectedForce.size()) < cfg.sensorCount){
        expectedForce.resize(cfg.sensorCount, 0.0);
    }
    if(static_cast<int>(expectedForceDerivative.size()) < cfg.sensorCount){
        expectedForceDerivative.resize(cfg.sensorCount, 0.0);
    }
    if(static_cast<int>(expectedRateFeedForwardScale.size()) < cfg.sensorCount){
        expectedRateFeedForwardScale.resize(cfg.sensorCount, 1.0);
    }
    if(static_cast<int>(expectedRopeVelocityRadPerSec.size()) < cfg.sensorCount){
        expectedRopeVelocityRadPerSec.resize(cfg.sensorCount, 0.0);
    }
    if(static_cast<int>(expectedRopeAccelerationRadPerSec2.size()) < cfg.sensorCount){
        expectedRopeAccelerationRadPerSec2.resize(cfg.sensorCount, 0.0);
    }
    if(static_cast<int>(platformCaptureTrajectoryPlatform.size()) < cfg.sensorCount){
        platformCaptureTrajectoryPlatform.resize(cfg.sensorCount, -1);
    }
    if(static_cast<int>(forceFeedForwardOnlyDirectionalProfileSign.size()) < cfg.sensorCount){
        forceFeedForwardOnlyDirectionalProfileSign.resize(cfg.sensorCount, 1);
    }
    std::vector<bool> expectedClamped(cfg.sensorCount, false);
    for(const AxisConfig& axis : cfg.axes){
        if(axis.sensorIndex < 0 || axis.sensorIndex >= cfg.sensorCount || expectedClamped[axis.sensorIndex]){
            continue;
        }
        const double unclampedExpected = expectedForce[axis.sensorIndex];
        double expectedLowerBound = cfg.initForce;
        if(axis.forceMax > 1e-5){
            expectedForce[axis.sensorIndex] = std::min(expectedForce[axis.sensorIndex], axis.forceMax);
            expectedLowerBound = std::min(expectedLowerBound, axis.forceMax);
        }
        expectedForce[axis.sensorIndex] = std::max(expectedForce[axis.sensorIndex], expectedLowerBound);
        if(std::fabs(expectedForce[axis.sensorIndex] - unclampedExpected) > 1e-9){
            expectedForceDerivative[axis.sensorIndex] = 0.0;
            expectedRateFeedForwardScale[axis.sensorIndex] = 0.0;
            expectedRopeVelocityRadPerSec[axis.sensorIndex] = 0.0;
            expectedRopeAccelerationRadPerSec2[axis.sensorIndex] = 0.0;
            platformCaptureTrajectoryPlatform[axis.sensorIndex] = 1;
        }
        expectedClamped[axis.sensorIndex] = true;
    }

    bool hasForceAxis = false;
    for(const AxisConfig& axis : cfg.axes){
        hasForceAxis = hasForceAxis || axis.forceControlEnabled;
    }
    const bool forceThreadRunning = (cfg.systemRunning || cfg.commissioningModeActive) &&
            leadshineConnected &&
            cfg.forceThreadEnabled &&
            hasForceAxis;
    const bool forcePid0525HybridEnabledForLoop =
            forceThreadRunning &&
            cfg.usePid &&
            cfg.forcePidOutputMode == ForcePidOutputMode::Pid0525 &&
            (cfg.forcePid0525DynamicTrackEnabled ||
             cfg.forcePid0525HybridEnabled);
    const ForcePid0525DynamicTrackMode forcePid0525DynamicTrackModeForLoop =
            cfg.forcePid0525DynamicTrackMode;
    std::vector<bool> forcePid0525FeedForwardOnlyFullTimeByAxis(
                std::max(0, cfg.axisCount),
                false);
    if(forcePid0525HybridEnabledForLoop){
        const int axisLimit = std::min(cfg.axisCount,
                                       static_cast<int>(cfg.axes.size()));
        for(int axisIndex=0; axisIndex<axisLimit; ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(!axis.isMotorAxis ||
                    !axis.forceControlEnabled ||
                    axis.sensorIndex < 0 ||
                    axis.sensorIndex >= cfg.sensorCount){
                continue;
            }
            const double rawProfileExpectedForceRateNPerSec =
                    axis.sensorIndex < static_cast<int>(expectedForceDerivative.size()) &&
                    std::isfinite(expectedForceDerivative[axis.sensorIndex]) ?
                        expectedForceDerivative[axis.sensorIndex] :
                        0.0;
            const double profileExpectedForceRateNPerSec =
                    feedForwardOnlyDirectionalProfileSelectionRate(
                        forceFeedForwardOnlyDirectionalProfileSign,
                        axis.sensorIndex,
                        rawProfileExpectedForceRateNPerSec,
                        cfg.forceFeedForwardOnlyStaticFrictionForceRateDeadbandNPerSec);
            const FeedForwardOnlyRuntimeParams feedForwardOnlyParams =
                    feedForwardOnlyRuntimeParams(
                        cfg,
                        axis.sensorIndex,
                        profileExpectedForceRateNPerSec);
            if(feedForwardOnlyParams.dynamicTrackMode ==
                    ForcePid0525DynamicTrackMode::FeedForwardOnly &&
                    !feedForwardOnlyParams.useBangBangPretension){
                forcePid0525FeedForwardOnlyFullTimeByAxis[axisIndex] = true;
            }
        }
    }
    bool externalExpectedForceTrajectoryPresentForLoop = false;
    bool externalExpectedForceTrajectoryMotionActiveForLoop = false;
    if(forcePid0525HybridEnabledForLoop){
        externalExpectedForceTrajectoryMotionActiveForLoop =
                externalExpectedForceTrajectoryMotionActive(
                    loopNowUs,
                    &externalExpectedForceTrajectoryPresentForLoop);
    }
    // 外部期望力轨迹存在时，由它的时间轴定义动态跟随窗口；到达末尾后立即回到预紧路径。
    const bool forcePid0525HybridMotionActiveForLoop =
            forcePid0525HybridEnabledForLoop &&
            (externalExpectedForceTrajectoryPresentForLoop ?
                 externalExpectedForceTrajectoryMotionActiveForLoop :
                 cfg.pvtActiveOrPaused);
    refreshForceControlFaultLatches(cfg, forceThreadRunning);
    if(forceThreadRunning &&
            (cfg.allCableForceDragModeEnabled != lastAllCableForceDragModeEnabled ||
             cfg.forceFeedForwardOnlyTestModeEnabled != lastForceFeedForwardOnlyTestModeEnabled ||
             cfg.forcePidOutputMode != lastForcePidOutputMode ||
             (forcePid0525HybridEnabledForLoop &&
              (forcePid0525DynamicTrackModeForLoop != lastForcePid0525DynamicTrackMode ||
               cfg.forcePid0525UseBangBangPretension != lastForcePid0525UseBangBangPretension)))){
        resetForceFeedbackState(cfg.sensorCount);
        resetForcePid0525HybridState(cfg.axisCount);
    }
    if(!forcePid0525HybridEnabledForLoop && lastForcePid0525HybridEnabled){
        resetForcePid0525HybridState(cfg.axisCount);
    }
    if(forcePid0525HybridEnabledForLoop &&
            lastForcePid0525HybridMotionActive &&
            !forcePid0525HybridMotionActiveForLoop){
        ensureForcePid0525HybridStateSize(cfg.axisCount);
        for(int axisIndex=0; axisIndex<std::min(cfg.axisCount, static_cast<int>(cfg.axes.size())); ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(axisIndex < static_cast<int>(forcePid0525FeedForwardOnlyFullTimeByAxis.size()) &&
                    forcePid0525FeedForwardOnlyFullTimeByAxis[axisIndex]){
                continue;
            }
            if(axis.isMotorAxis && axis.forceControlEnabled){
                const ForcePid0525HybridState state =
                        axisIndex < static_cast<int>(forcePid0525HybridState.size()) ?
                            forcePid0525HybridState[axisIndex] :
                            ForcePid0525HybridState::Idle;
                const bool wasTracking =
                        state == ForcePid0525HybridState::TrackBlend ||
                        state == ForcePid0525HybridState::Track ||
                        state == ForcePid0525HybridState::TrackExitBlend;
                const bool biasValid =
                        axisIndex < static_cast<int>(forcePid0525HybridBiasValid.size()) &&
                        forcePid0525HybridBiasValid[axisIndex];
                if(wasTracking || biasValid){
                    double blendStart = 0.0;
                    if(axisIndex < static_cast<int>(forcePid0525HybridHoldBiasNm.size()) &&
                            std::isfinite(forcePid0525HybridHoldBiasNm[axisIndex])){
                        blendStart = forcePid0525HybridHoldBiasNm[axisIndex];
                    }
                    if(axisIndex < static_cast<int>(torqueCommandActive.size()) &&
                            torqueCommandActive[axisIndex] &&
                            axisIndex < static_cast<int>(lastTorqueCommandNm.size()) &&
                            std::isfinite(lastTorqueCommandNm[axisIndex])){
                        blendStart = lastTorqueCommandNm[axisIndex];
                    }
                    forcePid0525HybridState[axisIndex] =
                            ForcePid0525HybridState::TrackExitBlend;
                    forcePid0525HybridBiasValid[axisIndex] = true;
                    forcePid0525HybridHoldBiasNm[axisIndex] = blendStart;
                    forcePid0525HybridStableTimeSec[axisIndex] = 0.0;
                    forcePid0525HybridBlendElapsedSec[axisIndex] = 0.0;
                    forcePid0525HybridBlendStartTorqueNm[axisIndex] = blendStart;
                    forcePid0525HybridRateInitialized[axisIndex] = false;
                    if(axisIndex < static_cast<int>(torqueCommandBiasNm.size())){
                        torqueCommandBiasNm[axisIndex] = blendStart;
                    }
                    if(axis.sensorIndex >= 0 &&
                            axis.sensorIndex < cfg.sensorCount){
                        forcePid0525.resetIntegral(static_cast<size_t>(axis.sensorIndex));
                        forceController.resetChannel(axis.sensorIndex);
                        if(axis.sensorIndex < static_cast<int>(unloadFeedForwardBlend.size())){
                            unloadFeedForwardBlend[axis.sensorIndex] = 0.0;
                        }
                    }
                }
                else{
                    resetForcePid0525HybridAxis(axisIndex, true);
                    resetForceFeedbackChannel(axis.sensorIndex);
                }
            }
        }
    }
    if(!forceThreadRunning){
        resetForceFeedbackState(cfg.sensorCount);
        stopActiveTorqueCommands(cfg);
        resetForcePid0525HybridState(cfg.axisCount);
    }
    else if(leadshineConnected){
        bool needsFreshTorqueWarmStart = false;
        const int axisCount = std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
        for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(axis.isMotorAxis &&
                    axis.forceControlEnabled &&
                    axis.sensorIndex >= 0 &&
                    !torqueCommandActive[axisIndex]){
                needsFreshTorqueWarmStart = true;
                break;
            }
        }
        if(needsFreshTorqueWarmStart){
            motorTorqueNm = cachedMotorTorqueNm;
            if(static_cast<int>(motorTorqueNm.size()) > cfg.axisCount){
                motorTorqueNm.resize(cfg.axisCount);
            }
        }
    }

    bool actualTorqueLimitEmergencyDetected = false;
    int actualTorqueLimitAxisIndex = -1;
    double actualTorqueLimitValueNm = 0.0;
    qint64 actualTorqueLimitDurationUs = 0;
    double actualTorqueLimitNm = cfg.actualTorqueLimitNm;
    const bool forceTorqueFeedbackActive = forceThreadRunning && cfg.usePid;
    if(forceTorqueFeedbackActive &&
            std::isfinite(cfg.forceTorqueCommandLimitNm) &&
            cfg.forceTorqueCommandLimitNm > 0.0){
        const double torqueModeFeedbackLimit =
                std::max(cfg.forceTorqueCommandLimitNm + 0.5,
                         cfg.forceTorqueCommandLimitNm * 1.5);
        actualTorqueLimitNm =
                std::isfinite(actualTorqueLimitNm) && actualTorqueLimitNm > 0.0 ?
                    std::min(actualTorqueLimitNm, torqueModeFeedbackLimit) :
                    torqueModeFeedbackLimit;
    }
    const bool actualTorqueProtectionEnabled =
            cfg.actualTorqueLimitEnabled || forceThreadRunning;
    // 实际电机力矩保护独立于 PID 输出，即使不是力控模式，也能在反馈力矩超限时触发急停。
    if(actualTorqueProtectionEnabled &&
            leadshineConnected &&
            std::isfinite(actualTorqueLimitNm) &&
            actualTorqueLimitNm > 0.0){
        const qint64 continuityMaxGapUs =
                std::max(kActualTorqueLimitMinContinuityGapUs,
                         targetIntervalUs * 3);
        const int axisCount = std::min({cfg.axisCount,
                                        static_cast<int>(cfg.axes.size()),
                                        static_cast<int>(motorTorqueNm.size())});
        for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(cfg.commissioningModeActive &&
                    axisIndex != cfg.commissioningAxisIndex){
                if(axisIndex < static_cast<int>(actualTorqueLimitOverStartUs.size())){
                    actualTorqueLimitOverStartUs[axisIndex] = 0;
                    actualTorqueLimitLastSampleUs[axisIndex] = 0;
                    actualTorqueLimitPeakNm[axisIndex] = 0.0;
                }
                continue;
            }
            if(!axis.isMotorAxis || !axis.actualTorqueLimitApplies){
                if(axisIndex < static_cast<int>(actualTorqueLimitOverStartUs.size())){
                    actualTorqueLimitOverStartUs[axisIndex] = 0;
                    actualTorqueLimitLastSampleUs[axisIndex] = 0;
                    actualTorqueLimitPeakNm[axisIndex] = 0.0;
                }
                continue;
            }

            const double torqueNm = motorTorqueNm[axisIndex];
            if(!std::isfinite(torqueNm)){
                if(axisIndex < static_cast<int>(actualTorqueLimitOverStartUs.size())){
                    actualTorqueLimitOverStartUs[axisIndex] = 0;
                    actualTorqueLimitLastSampleUs[axisIndex] = 0;
                    actualTorqueLimitPeakNm[axisIndex] = 0.0;
                }
                continue;
            }

            const double absTorqueNm = std::fabs(torqueNm);
            if(absTorqueNm > actualTorqueLimitNm){
                const qint64 previousSampleUs =
                        axisIndex < static_cast<int>(actualTorqueLimitLastSampleUs.size()) ?
                            actualTorqueLimitLastSampleUs[axisIndex] :
                            0;
                const bool sampleGapTooLong =
                        previousSampleUs > 0 &&
                        loopNowUs - previousSampleUs > continuityMaxGapUs;
                if(axisIndex < static_cast<int>(actualTorqueLimitOverStartUs.size()) &&
                        (actualTorqueLimitOverStartUs[axisIndex] <= 0 || sampleGapTooLong)){
                    actualTorqueLimitOverStartUs[axisIndex] = loopNowUs;
                    actualTorqueLimitPeakNm[axisIndex] = torqueNm;
                }
                if(axisIndex < static_cast<int>(actualTorqueLimitLastSampleUs.size())){
                    actualTorqueLimitLastSampleUs[axisIndex] = loopNowUs;
                }
                if(axisIndex < static_cast<int>(actualTorqueLimitPeakNm.size()) &&
                        absTorqueNm > std::fabs(actualTorqueLimitPeakNm[axisIndex])){
                    actualTorqueLimitPeakNm[axisIndex] = torqueNm;
                }
                const qint64 durationUs =
                        axisIndex < static_cast<int>(actualTorqueLimitOverStartUs.size()) ?
                            loopNowUs - actualTorqueLimitOverStartUs[axisIndex] :
                            0;
                if(durationUs < kActualTorqueLimitContinuousOverUs){
                    continue;
                }

                actualTorqueLimitEmergencyDetected = true;
                const double peakTorqueNm =
                        axisIndex < static_cast<int>(actualTorqueLimitPeakNm.size()) &&
                        std::isfinite(actualTorqueLimitPeakNm[axisIndex]) ?
                            actualTorqueLimitPeakNm[axisIndex] :
                            torqueNm;
                if(actualTorqueLimitAxisIndex < 0 ||
                        std::fabs(peakTorqueNm) > std::fabs(actualTorqueLimitValueNm)){
                    actualTorqueLimitAxisIndex = axisIndex;
                    actualTorqueLimitValueNm = peakTorqueNm;
                    actualTorqueLimitDurationUs = durationUs;
                }
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
            else{
                if(axisIndex < static_cast<int>(actualTorqueLimitOverStartUs.size())){
                    actualTorqueLimitOverStartUs[axisIndex] = 0;
                    actualTorqueLimitLastSampleUs[axisIndex] = loopNowUs;
                    actualTorqueLimitPeakNm[axisIndex] = 0.0;
                }
            }
        }
    }
    else{
        resetActualTorqueLimitState(cfg.axisCount);
    }

    if(actualTorqueLimitEmergencyDetected){
        const double sustainedMs = static_cast<double>(actualTorqueLimitDurationUs) / 1000.0;
        throttledInfo(QStringLiteral("Software emergency stop: motor%1 actual torque %2 Nm stayed above limit %3 Nm for %4 ms")
                      .arg(actualTorqueLimitAxisIndex)
                      .arg(actualTorqueLimitValueNm, 0, 'f', 3)
                      .arg(actualTorqueLimitNm, 0, 'f', 3)
                      .arg(sustainedMs, 0, 'f', 1),
                      "error");
        if(!actualTorqueLimitEmergencyStopActive){
            if(cfg.commissioningModeActive && actualTorqueLimitAxisIndex >= 0){
                hardwareInterface->emergencyStopAxes(
                            std::vector<int>{actualTorqueLimitAxisIndex});
            }
            else{
                hardwareInterface->emergencyStopAll();
            }
            resetTorqueCommandState(cfg.axisCount);
            resetForceFeedbackState(cfg.sensorCount);
            actualTorqueLimitEmergencyStopActive = true;
            emit actualTorqueLimitExceeded(actualTorqueLimitAxisIndex,
                                           actualTorqueLimitValueNm,
                                           actualTorqueLimitNm,
                                           sustainedMs);
        }
    }
    else{
        actualTorqueLimitEmergencyStopActive = false;
    }

    bool emittedForcePidTraceSample = false;
    if(!actualTorqueLimitEmergencyDetected &&
            forceThreadRunning &&
            static_cast<int>(forceSensorValue.size()) >= cfg.sensorCount){
        // 力控计算路径：先按传感器通道算 PID 输出，再映射到绑定该传感器的电机轴力矩命令。
        const int sensorCount = std::max(cfg.sensorCount, 0);
        const bool collectForcePidTraceSample = cfg.forcePidTuningHighRateSampleEnabled;
        std::vector<double> pidOutput(sensorCount, 0.0);
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
        std::vector<int> pid0525HybridStateTrace;
        std::vector<int> pid0525HybridBiasValidTrace;
        std::vector<double> pid0525HybridHoldBiasTrace;
        std::vector<double> pid0525HybridCaptureForceTrace;
        std::vector<double> pid0525HybridFeedForwardTrace;
        std::vector<double> pid0525HybridFeedbackTrace;
        std::vector<double> pid0525HybridBlendTrace;
        std::vector<int> forceControlAxisIndex;
        std::vector<int> forceControlSensorIndex;
        if(collectForcePidTraceSample){
            pidError.assign(sensorCount, 0.0);
            pidPTerm.assign(sensorCount, 0.0);
            pidITerm.assign(sensorCount, 0.0);
            pidDTerm.assign(sensorCount, 0.0);
            pidIntegral.assign(sensorCount, 0.0);
            pidMeasuredDerivativeRaw.assign(sensorCount, 0.0);
            pidMeasuredDerivativeFiltered.assign(sensorCount, 0.0);
            pidMeasuredDerivativeControl.assign(sensorCount, 0.0);
            pidExpectedDerivativeRaw.assign(sensorCount, 0.0);
            pidExpectedDerivativeFiltered.assign(sensorCount, 0.0);
            pidFeedForwardRaw.assign(sensorCount, 0.0);
            pidFeedForwardTerm.assign(sensorCount, 0.0);
            pidFeedForwardFrictionTerm.assign(sensorCount, 0.0);
            pidFeedForwardSelectedDynamicProfile.assign(sensorCount, 0);
            pidStaticFrictionDirection.assign(sensorCount, 0.0);
            pidStaticFrictionSpeedScale.assign(sensorCount, 0.0);
            pidStaticFrictionRaw.assign(sensorCount, 0.0);
            pidStaticFrictionAfterFade.assign(sensorCount, 0.0);
            pidStaticFrictionAfterSmooth.assign(sensorCount, 0.0);
            pidFeedForwardVelocityTerm.assign(sensorCount, 0.0);
            pidFeedForwardAccelerationTerm.assign(sensorCount, 0.0);
            pidExpectedRopeVelocityRadPerSec.assign(sensorCount, 0.0);
            pidExpectedRopeAccelerationRadPerSec2.assign(sensorCount, 0.0);
            pidExpectedRateFeedForwardTerm.assign(sensorCount, 0.0);
            pidExpectedRateFeedForwardScale.assign(sensorCount, 1.0);
            pidForceRateError.assign(sensorCount, 0.0);
            pidForceRateErrorDampingTerm.assign(sensorCount, 0.0);
            pidPlatformCaptureTerm.assign(sensorCount, 0.0);
            pidPlatformCaptureTargetTerm.assign(sensorCount, 0.0);
            pidPlatformCaptureState.assign(sensorCount, 0);
            pidFuzzyFeedForwardTargetScale.assign(sensorCount, 1.0);
            pidFuzzyFeedForwardScale.assign(sensorCount, 1.0);
            pidFuzzyFeedForwardRecoveryRate.assign(sensorCount, 0.0);
            pidFuzzyKpScale.assign(sensorCount, 1.0);
            pidFuzzyKiScale.assign(sensorCount, 1.0);
            pidFuzzyVelocityDampingScale.assign(sensorCount, 1.0);
            pidFuzzyPositivePLimit.assign(sensorCount, 0.0);
            pidFuzzyNegativePLimit.assign(sensorCount, 0.0);
            pidFuzzyFeedForwardRecoveryLimited.assign(sensorCount, 0);
            pidFuzzyPLimitApplied.assign(sensorCount, 0);
            pidFuzzyState.assign(sensorCount, 0);
            pidIntegralReleaseApplied.assign(sensorCount, 0);
            pidAntiWindup.assign(sensorCount, 0);
            pidOutputLimited.assign(sensorCount, 0);
            torqueSaturated.assign(std::max(cfg.axisCount, 0), 0);
            torqueSlewLimited.assign(std::max(cfg.axisCount, 0), 0);
            pid0525HybridStateTrace.assign(std::max(cfg.axisCount, 0), 0);
            pid0525HybridBiasValidTrace.assign(std::max(cfg.axisCount, 0), 0);
            pid0525HybridHoldBiasTrace.assign(std::max(cfg.axisCount, 0), 0.0);
            pid0525HybridCaptureForceTrace.assign(std::max(cfg.axisCount, 0), 0.0);
            pid0525HybridFeedForwardTrace.assign(std::max(cfg.axisCount, 0), 0.0);
            pid0525HybridFeedbackTrace.assign(std::max(cfg.axisCount, 0), 0.0);
            pid0525HybridBlendTrace.assign(std::max(cfg.axisCount, 0), 0.0);
            forceControlAxisIndex.reserve(std::max(cfg.axisCount, 0));
            forceControlSensorIndex.reserve(std::max(cfg.axisCount, 0));
        }
        const auto pidCoefficient = [](const std::vector<double>& values, int index) -> double {
            return index >= 0 && index < static_cast<int>(values.size()) ? values[index] : 0.0;
        };
        const bool useForcePid0525 =
                cfg.forcePidOutputMode == ForcePidOutputMode::Pid0525;
        const bool useFeedForwardOnlyTestMode =
                cfg.forceFeedForwardOnlyTestModeEnabled && !useForcePid0525;
        const double controlDtSec =
                timingDiagnostics.latestControlLoopIntervalUs > 0 ?
                    static_cast<double>(timingDiagnostics.latestControlLoopIntervalUs) / 1000000.0 :
                    targetIntervalSec;
        const double feedbackDtSec =
                std::isfinite(controlDtSec) && controlDtSec > 0.0 ?
                    controlDtSec :
                    targetIntervalSec;
        const double limitedFeedbackDtSec =
                std::min(std::max(feedbackDtSec, 1e-6), 0.1);
        const double forcePid0525DtSec =
                std::min(std::max(feedbackDtSec, 1e-6), kForcePid0525MaxDtSec);
        const bool forcePid0525HybridActive =
                useForcePid0525 &&
                cfg.usePid &&
                (cfg.forcePid0525DynamicTrackEnabled ||
                 cfg.forcePid0525HybridEnabled);
        if(useForcePid0525 && cfg.usePid){
            std::vector<double> pidForceSensorValue = forceSensorValue;
            std::vector<double> pidExpectedForce = expectedForce;
            pidForceSensorValue.resize(sensorCount, 0.0);
            pidExpectedForce.resize(sensorCount, 0.0);
            std::vector<double> pidP = cfg.pidP;
            std::vector<double> pidI = cfg.pidI;
            std::vector<double> pidD = cfg.pidD;
            pidP.resize(sensorCount, 0.0);
            pidI.resize(sensorCount, 0.0);
            pidD.resize(sensorCount, 0.0);
            const double torqueLimit = std::max(
                        0.0,
                        std::isfinite(cfg.forceTorqueCommandLimitNm) ?
                            cfg.forceTorqueCommandLimitNm :
                            kDefaultForceTorqueCommandLimitNm);

            forcePid0525.updatePara(
                        pidP,
                        pidI,
                        pidD,
                        forcePid0525DtSec * 1000.0);
            forcePid0525.updateTustinPara(
                        std::vector<double>(sensorCount, 0.0),
                        std::vector<double>(sensorCount, -kForcePid0525IntegralLimit),
                        std::vector<double>(sensorCount, kForcePid0525IntegralLimit),
                        std::vector<double>(sensorCount, -torqueLimit),
                        std::vector<double>(sensorCount, torqueLimit));
            std::vector<int> freezeIntegral(sensorCount, 0);
            if(forcePid0525HybridActive && cfg.forcePid0525FreezeIntegralDuringTrack){
                ensureForcePid0525HybridStateSize(cfg.axisCount);
                const int axisCount = std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
                for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
                    const AxisConfig& axis = cfg.axes[axisIndex];
                    if(!axis.isMotorAxis ||
                            !axis.forceControlEnabled ||
                            axis.sensorIndex < 0 ||
                            axis.sensorIndex >= sensorCount ||
                            axisIndex >= static_cast<int>(forcePid0525HybridState.size())){
                        continue;
                    }
                    const ForcePid0525HybridState state = forcePid0525HybridState[axisIndex];
                    const bool biasValid =
                            axisIndex < static_cast<int>(forcePid0525HybridBiasValid.size()) &&
                            forcePid0525HybridBiasValid[axisIndex];
                    const bool axisFeedForwardOnlyFullTime =
                            axisIndex < static_cast<int>(
                                forcePid0525FeedForwardOnlyFullTimeByAxis.size()) &&
                            forcePid0525FeedForwardOnlyFullTimeByAxis[axisIndex];
                    if(axisFeedForwardOnlyFullTime ||
                            (forcePid0525HybridMotionActiveForLoop && biasValid) ||
                            state == ForcePid0525HybridState::TrackBlend ||
                            state == ForcePid0525HybridState::Track ||
                            state == ForcePid0525HybridState::TrackExitBlend){
                        freezeIntegral[axis.sensorIndex] = 1;
                    }
                }
            }
            pidOutput = forcePid0525.updateWithRelativeDeadband(
                        pidForceSensorValue,
                        pidExpectedForce,
                        cfg.forcePidDeadbandRatio,
                        freezeIntegral);
            if(static_cast<int>(pidOutput.size()) < sensorCount){
                pidOutput.resize(sensorCount, 0.0);
            }
            if(collectForcePidTraceSample){
                const std::vector<double>& debugError = forcePid0525.debugError();
                const std::vector<double>& debugPTerm = forcePid0525.debugPTerm();
                const std::vector<double>& debugITerm = forcePid0525.debugITerm();
                const std::vector<double>& debugDTerm = forcePid0525.debugDTerm();
                const std::vector<double>& debugIntegral = forcePid0525.debugIntegral();
                const std::vector<double>& debugOutput = forcePid0525.debugOutput();
                for(int sensorIndex=0; sensorIndex<sensorCount; ++sensorIndex){
                    const double error =
                            pidExpectedForce[sensorIndex] - pidForceSensorValue[sensorIndex];
                    pidError[sensorIndex] =
                            sensorIndex < static_cast<int>(debugError.size()) ?
                                debugError[sensorIndex] :
                                error;
                    pidPTerm[sensorIndex] =
                            sensorIndex < static_cast<int>(debugPTerm.size()) ?
                                debugPTerm[sensorIndex] :
                                pidP[sensorIndex] * error;
                    pidITerm[sensorIndex] =
                            sensorIndex < static_cast<int>(debugITerm.size()) ?
                                debugITerm[sensorIndex] :
                                0.0;
                    pidDTerm[sensorIndex] =
                            sensorIndex < static_cast<int>(debugDTerm.size()) ?
                                debugDTerm[sensorIndex] :
                                0.0;
                    pidIntegral[sensorIndex] =
                            sensorIndex < static_cast<int>(debugIntegral.size()) ?
                                debugIntegral[sensorIndex] :
                                0.0;
                    if(sensorIndex < static_cast<int>(debugOutput.size())){
                        pidOutput[sensorIndex] = debugOutput[sensorIndex];
                    }
                }
            }
        }

        std::vector<ForceController::Params> forceControllerParams;
        if(!useForcePid0525 && !useFeedForwardOnlyTestMode){
            forceControllerParams.resize(sensorCount);
            for(int sensorIndex=0; sensorIndex<sensorCount; ++sensorIndex){
                forceControllerParams[sensorIndex].feedForwardTorque = 0.0;
                forceControllerParams[sensorIndex].kp = pidCoefficient(cfg.pidP, sensorIndex);
                const double ki = pidCoefficient(cfg.pidI, sensorIndex);
                forceControllerParams[sensorIndex].ki = ki;
                forceControllerParams[sensorIndex].kd = pidCoefficient(cfg.pidD, sensorIndex);
                forceControllerParams[sensorIndex].deadbandRatio = cfg.forcePidDeadbandRatio;
                const double integralTorqueLimitNm =
                        std::isfinite(cfg.forceIntegralTorqueLimitNm) ?
                            std::max(0.0, cfg.forceIntegralTorqueLimitNm) :
                            0.0;
                forceControllerParams[sensorIndex].integralLimit =
                        std::fabs(ki) > 1e-12 && integralTorqueLimitNm > 0.0 ?
                            integralTorqueLimitNm / std::fabs(ki) :
                            0.0;
                forceControllerParams[sensorIndex].derivativeLowPassTauSec =
                        kForceControllerDerivativeLowPassTauSec;
                forceControllerParams[sensorIndex].expectedForceDerivativeNPerSec =
                        sensorIndex < static_cast<int>(expectedForceDerivative.size()) ?
                            expectedForceDerivative[sensorIndex] :
                            0.0;
                forceControllerParams[sensorIndex].expectedRateFeedForwardScale =
                        sensorIndex < static_cast<int>(expectedRateFeedForwardScale.size()) ?
                            expectedRateFeedForwardScale[sensorIndex] :
                            1.0;
                forceControllerParams[sensorIndex].expectedRateFeedForwardGainUpNmPerNps =
                        cfg.forceExpectedRateFeedForwardGainUpNmPerNps;
                forceControllerParams[sensorIndex].expectedRateFeedForwardGainDownNmPerNps =
                        cfg.forceExpectedRateFeedForwardGainDownNmPerNps;
                forceControllerParams[sensorIndex].expectedRateFeedForwardLimitUpNm =
                        cfg.forceExpectedRateFeedForwardLimitUpNm;
                forceControllerParams[sensorIndex].expectedRateFeedForwardLimitDownNm =
                        cfg.forceExpectedRateFeedForwardLimitDownNm;
                forceControllerParams[sensorIndex].expectedRateFeedForwardDownErrorGateN =
                        cfg.forceExpectedRateFeedForwardDownErrorGateN;
                forceControllerParams[sensorIndex].expectedRateFeedForwardDownFastDropGateNPerSec =
                        cfg.forceExpectedRateFeedForwardDownFastDropGateNPerSec;
                forceControllerParams[sensorIndex].expectedRateFeedForwardDownMinScale =
                        cfg.forceExpectedRateFeedForwardDownMinScale;
                forceControllerParams[sensorIndex].forceRateControlDerivativeLimitNPerSec =
                        cfg.forceRateControlDerivativeLimitNPerSec;
                forceControllerParams[sensorIndex].forceRateControlDerivativePlatformLimitNPerSec =
                        cfg.forceRateControlDerivativePlatformLimitNPerSec;
                forceControllerParams[sensorIndex].forceRateErrorDeadbandNPerSec =
                        cfg.forceRateErrorDeadbandNPerSec;
                forceControllerParams[sensorIndex].forceRateBelowExpectedCatchUpGainNmPerNps =
                        cfg.forceRateBelowExpectedCatchUpGainNmPerNps;
                forceControllerParams[sensorIndex].forceRateBelowExpectedCatchUpLimitNm =
                        cfg.forceRateBelowExpectedCatchUpLimitNm;
                forceControllerParams[sensorIndex].forceRateBelowExpectedBrakeGainNmPerNps =
                        cfg.forceRateBelowExpectedBrakeGainNmPerNps;
                forceControllerParams[sensorIndex].forceRateBelowExpectedBrakeLimitNm =
                        cfg.forceRateBelowExpectedBrakeLimitNm;
                forceControllerParams[sensorIndex].forceRateAboveExpectedUnloadGainNmPerNps =
                        cfg.forceRateAboveExpectedUnloadGainNmPerNps;
                forceControllerParams[sensorIndex].forceRateAboveExpectedUnloadLimitNm =
                        cfg.forceRateAboveExpectedUnloadLimitNm;
                forceControllerParams[sensorIndex].forceRateAboveExpectedRecoverGainNmPerNps =
                        cfg.forceRateAboveExpectedRecoverGainNmPerNps;
                forceControllerParams[sensorIndex].forceRateAboveExpectedRecoverLimitNm =
                        cfg.forceRateAboveExpectedRecoverLimitNm;
                forceControllerParams[sensorIndex].platformCaptureRateThresholdNPerSec =
                        cfg.forcePlatformCaptureRateThresholdNPerSec;
                forceControllerParams[sensorIndex].platformCaptureEnableErrorN =
                        cfg.forcePlatformCaptureEnableErrorN;
                forceControllerParams[sensorIndex].platformCaptureDisableErrorN =
                        cfg.forcePlatformCaptureDisableErrorN;
                forceControllerParams[sensorIndex].platformCaptureGainNmPerN =
                        cfg.forcePlatformCaptureGainNmPerN;
                forceControllerParams[sensorIndex].platformCaptureLimitUpNm =
                        cfg.forcePlatformCaptureLimitUpNm;
                forceControllerParams[sensorIndex].platformCaptureLimitDownNm =
                        cfg.forcePlatformCaptureLimitDownNm;
                forceControllerParams[sensorIndex].platformCaptureSlewRateNmPerSec =
                        cfg.forcePlatformCaptureSlewRateNmPerSec;
                forceControllerParams[sensorIndex].platformCaptureHoldTimeSec =
                        cfg.forcePlatformCaptureHoldTimeSec;
                forceControllerParams[sensorIndex].platformCaptureReleaseRateNmPerSec =
                        cfg.forcePlatformCaptureReleaseRateNmPerSec;
                forceControllerParams[sensorIndex].platformCaptureMeasuredRateThresholdNPerSec =
                        cfg.forcePlatformCaptureMeasuredRateThresholdNPerSec;
                forceControllerParams[sensorIndex].platformCaptureMeasuredRateHoldTimeSec =
                        cfg.forcePlatformCaptureMeasuredRateHoldTimeSec;
                forceControllerParams[sensorIndex].fuzzyFeedForwardDropRatePerSec =
                        cfg.forceFuzzyFeedForwardDropRatePerSec;
                forceControllerParams[sensorIndex].fuzzyFeedForwardFastDescentDropRatePerSec =
                        cfg.forceFuzzyFeedForwardFastDescentDropRatePerSec;
                forceControllerParams[sensorIndex].fuzzySupervisorEnabled = true;
                forceControllerParams[sensorIndex].platformCaptureUseTrajectoryPlatformFlag =
                        sensorIndex < static_cast<int>(platformCaptureTrajectoryPlatform.size()) &&
                        platformCaptureTrajectoryPlatform[sensorIndex] >= 0;
                forceControllerParams[sensorIndex].platformCaptureTrajectoryPlatform =
                        sensorIndex < static_cast<int>(platformCaptureTrajectoryPlatform.size()) &&
                        platformCaptureTrajectoryPlatform[sensorIndex] > 0;
                forceControllerParams[sensorIndex].integralReleaseExpectedRateThresholdNPerSec =
                        cfg.forceIntegralReleaseExpectedRateThresholdNPerSec;
                forceControllerParams[sensorIndex].integralReleaseOverForceThresholdN =
                        cfg.forceIntegralReleaseOverForceThresholdN;
                forceControllerParams[sensorIndex].integralReleaseTimeConstantSec =
                        cfg.forceIntegralReleaseTimeConstantSec;
            }
        }
        ensureForceFeedbackStateSize(sensorCount);
        if(!cfg.usePid && !useFeedForwardOnlyTestMode){
            resetForceFeedbackState(sensorCount);
        }
        std::vector<bool> forceFeedbackUpdated(sensorCount, false);
        std::vector<bool> forceFeedbackCommitted(sensorCount, false);
        std::vector<bool> forceFeedbackReferenced(sensorCount, false);

        for(int axisIndex=0; axisIndex<std::min(cfg.axisCount, static_cast<int>(cfg.axes.size())); ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(!axis.isMotorAxis || axis.sensorIndex < 0 || axis.sensorIndex >= cfg.sensorCount){
                continue;
            }
            if(isProtectedMotionAxis(cfg, axisIndex)){
                continue;
            }

            if(axis.forceControlEnabled){
                const double curForce = forceSensorValue[axis.sensorIndex];
                if(axis.sensorIndex >= 0 && axis.sensorIndex < static_cast<int>(forceFeedbackReferenced.size())){
                    forceFeedbackReferenced[axis.sensorIndex] = true;
                }
                if(axisIndex < static_cast<int>(forceControlFaultLatched.size()) &&
                        forceControlFaultLatched[axisIndex]){
                    stopTorqueModeAxis(axisIndex);
                    resetForceFeedbackChannel(axis.sensorIndex);
                    if(axisIndex < static_cast<int>(motorCommand.size())){
                        motorCommand[axisIndex] = 0.0;
                    }
                    throttledInfo(QString("Motor%1 torque PID remains stopped: force-control fault is latched; disable this force axis or force thread to reset.")
                                  .arg(axisIndex), "error", 3000);
                    continue;
                }
                const double hardwareDirection = axis.motorDirectionSign < 0.0 ? -1.0 : 1.0;
                const double torqueLimit = std::max(
                            0.0,
                            std::isfinite(cfg.forceTorqueCommandLimitNm) ?
                                cfg.forceTorqueCommandLimitNm :
                                kDefaultForceTorqueCommandLimitNm);
                const double torqueSlewRate = std::max(
                            0.0,
                            std::isfinite(cfg.forceTorqueCommandSlewRateNmPerSec) ?
                                cfg.forceTorqueCommandSlewRateNmPerSec :
                                kDefaultForceTorqueCommandSlewRateNmPerSec);
                const double velocityDamping = std::max(
                            0.0,
                            std::isfinite(cfg.forceTorqueVelocityDampingNmPerVelocity) ?
                                cfg.forceTorqueVelocityDampingNmPerVelocity :
                                0.0);
                const bool motorPositionLimitRecoveryAxis =
                        cfg.motorPositionLimitRecoveryActive &&
                        axisIndex < static_cast<int>(cfg.motorPositionLimitRecoveryAxes.size()) &&
                        cfg.motorPositionLimitRecoveryAxes[axisIndex];
                const bool hasValidRange = axisIndex < static_cast<int>(motorRelRawPos.size()) &&
                        std::isfinite(axis.motorMin) &&
                        std::isfinite(axis.motorMax) &&
                        axis.motorMax > axis.motorMin;
                const bool motorPositionFeedbackInvalid =
                        axisIndex >= static_cast<int>(motorRelRawPos.size()) ||
                        !std::isfinite(motorRelRawPos[axisIndex]);
                const bool forceSensorTimedOut =
                        !runtimeTraceFeedbackSafe ||
                        lastSensorFrameTimestampUs <= 0 ||
                        loopNowUs - lastSensorFrameTimestampUs > kTorqueForceSensorTimeoutUs;

                if(motorPositionFeedbackInvalid){
                    stopTorqueModeAxis(axisIndex);
                    if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                        forceControlFaultLatched[axisIndex] = true;
                    }
                    resetForceFeedbackChannel(axis.sensorIndex);
                    throttledInfo(QString("Motor%1 torque PID stopped: Trace position feedback is invalid or timed out.")
                                  .arg(axisIndex), "error");
                }
                else if(!std::isfinite(curForce)){
                    stopTorqueModeAxis(axisIndex);
                    if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                        forceControlFaultLatched[axisIndex] = true;
                    }
                    resetForceFeedbackChannel(axis.sensorIndex);
                    throttledInfo(QString("Motor%1 torque PID stopped: force feedback is invalid.")
                                  .arg(axisIndex), "error");
                }
                else if(forceSensorTimedOut){
                    stopTorqueModeAxis(axisIndex);
                    if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                        forceControlFaultLatched[axisIndex] = true;
                    }
                    resetForceFeedbackChannel(axis.sensorIndex);
                    throttledInfo(QString("Motor%1 torque PID stopped: force Trace is stale, backlogged, or its timing is not reliable.")
                                  .arg(axisIndex), "error");
                }
                else if(!motorPositionLimitRecoveryAxis &&
                        hasValidRange &&
                        (motorRelRawPos[axisIndex] <= axis.motorMin ||
                         motorRelRawPos[axisIndex] >= axis.motorMax)){
                    stopTorqueModeAxis(axisIndex);
                    if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                        forceControlFaultLatched[axisIndex] = true;
                    }
                    resetForceFeedbackChannel(axis.sensorIndex);
                    throttledInfo(QString("Motor%1 torque PID stopped: software position limit reached.")
                                  .arg(axisIndex), "error");
                }
                else if(axis.motorVelMax > 0.0 &&
                        (axisIndex >= static_cast<int>(motorVel.size()) ||
                         !std::isfinite(motorVel[axisIndex]) ||
                         std::fabs(motorVel[axisIndex]) > axis.motorVelMax)){
                    stopTorqueModeAxis(axisIndex);
                    if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                        forceControlFaultLatched[axisIndex] = true;
                    }
                    resetForceFeedbackChannel(axis.sensorIndex);
                    throttledInfo(QString("Motor%1 torque PID stopped: velocity feedback reached limit.")
                                  .arg(axisIndex), "error");
                }
                else if(axis.forceMax > 1e-5 && curForce > axis.forceMax){
                    stopTorqueModeAxis(axisIndex);
                    if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                        forceControlFaultLatched[axisIndex] = true;
                    }
                    resetForceFeedbackChannel(axis.sensorIndex);
                    throttledInfo(QString("Motor%1 torque PID stopped: force reached limit.")
                                  .arg(axisIndex), "error");
                }
                else if(torqueLimit <= kTorqueCommandEpsilonNm){
                    stopTorqueModeAxis(axisIndex);
                    resetForceFeedbackChannel(axis.sensorIndex);
                    throttledInfo(QString("Motor%1 torque PID stopped: torque command limit is zero.")
                                  .arg(axisIndex), "warning");
                }
                else{
                    bool axisTorqueSaturated = false;
                    bool axisTorqueSlewLimited = false;
                    double cmdTorque = 0.0;
                    if(useForcePid0525){
                        const bool torqueModeWasActive =
                                axisIndex < static_cast<int>(torqueCommandActive.size()) &&
                                torqueCommandActive[axisIndex];
                        double torqueBias = 0.0;
                        if(torqueModeWasActive &&
                                axisIndex < static_cast<int>(torqueCommandBiasNm.size()) &&
                                std::isfinite(torqueCommandBiasNm[axisIndex])){
                            torqueBias = torqueCommandBiasNm[axisIndex];
                        }
                        else{
                            torqueBias = warmStartTorqueForAxis(axisIndex,
                                                                 motorTorqueNm,
                                                                 torqueLimit);
                            if(axisIndex < static_cast<int>(torqueCommandBiasNm.size())){
                                torqueCommandBiasNm[axisIndex] = torqueBias;
                            }
                        }

                        double pidTorqueCorrection =
                                cfg.usePid &&
                                axis.sensorIndex < static_cast<int>(pidOutput.size()) ?
                                    pidOutput[axis.sensorIndex] :
                                    0.0;
                        pidTorqueCorrection *= hardwareDirection;
                        if(velocityDamping > 0.0 &&
                                axisIndex < static_cast<int>(motorVel.size()) &&
                                std::isfinite(motorVel[axisIndex])){
                            pidTorqueCorrection -= velocityDamping * motorVel[axisIndex];
                        }
                        cmdTorque = torqueBias + pidTorqueCorrection;
                        if(!torqueModeWasActive &&
                                std::fabs(torqueBias) > kTorqueCommandEpsilonNm){
                            cmdTorque = torqueBias;
                        }
                        const double staticPidTargetTorque = cmdTorque;

                        double activeTorqueSlewRate = torqueSlewRate;
                        double hybridFeedForwardTerm = 0.0;
                        double hybridFeedbackTerm = 0.0;
                        double hybridBlend = 0.0;
                        double hybridFeedForwardSensorTerm = 0.0;
                        double hybridFeedbackSensorTerm = 0.0;
                        double hybridDampingSensorTerm = 0.0;
                        double hybridActualForceRateRawNPerSec = 0.0;
                        double hybridActualForceRateFilteredNPerSec = 0.0;
                        double hybridExpectedForceRateRawNPerSec = 0.0;
                        double hybridExpectedForceRateFilteredNPerSec = 0.0;
                        double hybridForceRateErrorNPerSec = 0.0;
                        double hybridForceRateDampingSensorTerm = 0.0;
                        double hybridForceRateDampingTerm = 0.0;
                        double hybridHoldBias = 0.0;
                        double hybridError = 0.0;
                        FeedForwardOnlyTerms hybridFeedForwardOnlyTerms;
                        double hybridExpectedRopeVelocityRadPerSec = 0.0;
                        double hybridExpectedRopeAccelerationRadPerSec2 = 0.0;
                        bool hybridTrackPath = false;
                        bool hybridFeedForwardOnlyTrackPath = false;
                        if(forcePid0525HybridActive){
                            ensureForcePid0525HybridStateSize(cfg.axisCount);
                            if(axisIndex < static_cast<int>(forcePid0525HybridState.size()) &&
                                    forcePid0525HybridState[axisIndex] == ForcePid0525HybridState::Idle){
                                forcePid0525HybridState[axisIndex] =
                                        ForcePid0525HybridState::PreloadAcquire;
                            }
                            const double expected =
                                    axis.sensorIndex < static_cast<int>(expectedForce.size()) ?
                                        expectedForce[axis.sensorIndex] :
                                        0.0;
                            const double error = expected - curForce;
                            hybridError = error;
                            const double learnErrorN =
                                    std::isfinite(cfg.forcePid0525BiasLearnErrorN) ?
                                        std::max(0.0, cfg.forcePid0525BiasLearnErrorN) :
                                        1.0;
                            const double learnHoldTimeSec =
                                    std::isfinite(cfg.forcePid0525BiasLearnHoldTimeSec) ?
                                        std::max(0.0, cfg.forcePid0525BiasLearnHoldTimeSec) :
                                        0.5;
                            const bool biasValid =
                                    axisIndex < static_cast<int>(forcePid0525HybridBiasValid.size()) &&
                                    forcePid0525HybridBiasValid[axisIndex];
                            ForcePid0525HybridState state =
                                    axisIndex < static_cast<int>(forcePid0525HybridState.size()) ?
                                        forcePid0525HybridState[axisIndex] :
                                        ForcePid0525HybridState::Idle;
                            const double rawProfileExpectedForceRateNPerSec =
                                    axis.sensorIndex < static_cast<int>(expectedForceDerivative.size()) &&
                                    std::isfinite(expectedForceDerivative[axis.sensorIndex]) ?
                                        expectedForceDerivative[axis.sensorIndex] :
                                        0.0;
                            const double profileExpectedForceRateNPerSec =
                                    feedForwardOnlyDirectionalProfileSelectionRate(
                                        forceFeedForwardOnlyDirectionalProfileSign,
                                        axis.sensorIndex,
                                        rawProfileExpectedForceRateNPerSec,
                                        cfg.forceFeedForwardOnlyStaticFrictionForceRateDeadbandNPerSec);
                            const FeedForwardOnlyRuntimeParams feedForwardOnlyParams =
                                    feedForwardOnlyRuntimeParams(
                                        cfg,
                                        axis.sensorIndex,
                                        profileExpectedForceRateNPerSec);
                            const bool feedForwardOnlyTrackMode =
                                    feedForwardOnlyParams.dynamicTrackMode ==
                                    ForcePid0525DynamicTrackMode::FeedForwardOnly;
                            const bool feedForwardOnlyFullTime =
                                    feedForwardOnlyTrackMode &&
                                    !feedForwardOnlyParams.useBangBangPretension;
                            if(feedForwardOnlyFullTime){
                                hybridExpectedRopeVelocityRadPerSec =
                                        axis.sensorIndex < static_cast<int>(expectedRopeVelocityRadPerSec.size()) ?
                                            expectedRopeVelocityRadPerSec[axis.sensorIndex] :
                                            0.0;
                                hybridExpectedRopeAccelerationRadPerSec2 =
                                        axis.sensorIndex < static_cast<int>(expectedRopeAccelerationRadPerSec2.size()) ?
                                            expectedRopeAccelerationRadPerSec2[axis.sensorIndex] :
                                            0.0;
                                hybridExpectedForceRateRawNPerSec =
                                        axis.sensorIndex < static_cast<int>(expectedForceDerivative.size()) &&
                                        std::isfinite(expectedForceDerivative[axis.sensorIndex]) ?
                                            expectedForceDerivative[axis.sensorIndex] :
                                            0.0;
                                hybridExpectedForceRateFilteredNPerSec =
                                        hybridExpectedForceRateRawNPerSec;
                                hybridFeedForwardOnlyTerms =
                                        feedForwardOnlyTorqueTerms(
                                            expected,
                                            axis.motorCof,
                                            hybridExpectedForceRateRawNPerSec,
                                            hybridExpectedRopeVelocityRadPerSec,
                                            hybridExpectedRopeAccelerationRadPerSec2,
                                            feedForwardOnlyParams,
                                            axis.sensorIndex,
                                            forcePid0525DtSec,
                                            &forceFeedForwardOnlyStaticFrictionDirectionSign,
                                            &forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm,
                                            &forceFeedForwardOnlyStaticFrictionSmoothedValid);
                                hybridFeedForwardSensorTerm =
                                        hybridFeedForwardOnlyTerms.totalTerm;
                                hybridFeedForwardTerm =
                                        hardwareDirection *
                                        hybridFeedForwardOnlyTerms.totalTerm;
                                hybridHoldBias = 0.0;
                                hybridFeedbackSensorTerm = 0.0;
                                hybridFeedbackTerm = 0.0;
                                hybridBlend = 1.0;
                                hybridFeedForwardOnlyTrackPath = true;
                                hybridTrackPath = true;
                                cmdTorque = hybridFeedForwardTerm;
                                activeTorqueSlewRate =
                                        std::isfinite(feedForwardOnlyParams.trackTorqueSlewRateNmPerSec) ?
                                            std::max(0.0, feedForwardOnlyParams.trackTorqueSlewRateNmPerSec) :
                                            torqueSlewRate;
                                forcePid0525HybridState[axisIndex] =
                                        ForcePid0525HybridState::Track;
                                forcePid0525HybridBiasValid[axisIndex] = true;
                                forcePid0525HybridHoldBiasNm[axisIndex] = 0.0;
                                forcePid0525HybridCaptureForceN[axisIndex] = expected;
                                forcePid0525HybridStableTimeSec[axisIndex] = 0.0;
                            }
                            else if(forcePid0525HybridMotionActiveForLoop &&
                                    biasValid &&
                                    state != ForcePid0525HybridState::TrackBlend &&
                                    state != ForcePid0525HybridState::Track){
                                state = ForcePid0525HybridState::TrackBlend;
                                forcePid0525HybridState[axisIndex] = state;
                                forcePid0525HybridBlendElapsedSec[axisIndex] = 0.0;
                                double blendStart = forcePid0525HybridHoldBiasNm[axisIndex];
                                if(axisIndex < static_cast<int>(torqueCommandActive.size()) &&
                                        torqueCommandActive[axisIndex] &&
                                        axisIndex < static_cast<int>(lastTorqueCommandNm.size()) &&
                                        std::isfinite(lastTorqueCommandNm[axisIndex])){
                                    blendStart = lastTorqueCommandNm[axisIndex];
                                }
                                forcePid0525HybridBlendStartTorqueNm[axisIndex] = blendStart;
                                forcePid0525HybridRateInitialized[axisIndex] = false;
                                forcePid0525HybridLastActualForceN[axisIndex] = curForce;
                                forcePid0525HybridLastExpectedForceN[axisIndex] = expected;
                                forcePid0525HybridActualForceRateFilteredNPerSec[axisIndex] = 0.0;
                                forcePid0525HybridExpectedForceRateFilteredNPerSec[axisIndex] = 0.0;
                            }
                            if(!feedForwardOnlyFullTime){
                                state = forcePid0525HybridState[axisIndex];
                                if(forcePid0525HybridMotionActiveForLoop &&
                                    biasValid &&
                                    (state == ForcePid0525HybridState::TrackBlend ||
                                     state == ForcePid0525HybridState::Track)){
                                const double holdBias = forcePid0525HybridHoldBiasNm[axisIndex];
                                const double captureForce = forcePid0525HybridCaptureForceN[axisIndex];
                                hybridHoldBias = holdBias;
                                double dampingTorque = 0.0;
                                if(!feedForwardOnlyTrackMode &&
                                        velocityDamping > 0.0 &&
                                        axisIndex < static_cast<int>(motorVel.size()) &&
                                        std::isfinite(motorVel[axisIndex])){
                                    dampingTorque = velocityDamping * motorVel[axisIndex];
                                }
                                hybridDampingSensorTerm = dampingTorque * hardwareDirection;
                                double trackTargetTorque = holdBias - dampingTorque;
                                if(feedForwardOnlyTrackMode){
                                    hybridExpectedRopeVelocityRadPerSec =
                                            axis.sensorIndex < static_cast<int>(expectedRopeVelocityRadPerSec.size()) ?
                                                expectedRopeVelocityRadPerSec[axis.sensorIndex] :
                                                0.0;
                                    hybridExpectedRopeAccelerationRadPerSec2 =
                                            axis.sensorIndex < static_cast<int>(expectedRopeAccelerationRadPerSec2.size()) ?
                                                expectedRopeAccelerationRadPerSec2[axis.sensorIndex] :
                                                0.0;
                                    hybridExpectedForceRateRawNPerSec =
                                            axis.sensorIndex < static_cast<int>(expectedForceDerivative.size()) &&
                                            std::isfinite(expectedForceDerivative[axis.sensorIndex]) ?
                                                expectedForceDerivative[axis.sensorIndex] :
                                                0.0;
                                    hybridExpectedForceRateFilteredNPerSec =
                                            hybridExpectedForceRateRawNPerSec;
                                    hybridFeedForwardOnlyTerms =
                                            feedForwardOnlyTorqueTerms(
                                                expected,
                                                axis.motorCof,
                                                hybridExpectedForceRateRawNPerSec,
                                                hybridExpectedRopeVelocityRadPerSec,
                                                hybridExpectedRopeAccelerationRadPerSec2,
                                                feedForwardOnlyParams,
                                                axis.sensorIndex,
                                                forcePid0525DtSec,
                                                &forceFeedForwardOnlyStaticFrictionDirectionSign,
                                                &forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm,
                                                &forceFeedForwardOnlyStaticFrictionSmoothedValid);
                                    hybridFeedForwardSensorTerm =
                                            hybridFeedForwardOnlyTerms.totalTerm;
                                    hybridFeedForwardTerm =
                                            hardwareDirection *
                                            hybridFeedForwardOnlyTerms.totalTerm;
                                    hybridFeedbackSensorTerm = 0.0;
                                    hybridFeedbackTerm = 0.0;
                                    hybridFeedForwardOnlyTrackPath = true;
                                    trackTargetTorque = hybridFeedForwardTerm;
                                }
                                else{
                                    const double trackKff =
                                            std::isfinite(cfg.forcePid0525TrackKffNmPerN) ?
                                                cfg.forcePid0525TrackKffNmPerN :
                                                0.0;
                                    const double trackKp =
                                            std::isfinite(cfg.forcePid0525TrackKpNmPerN) ?
                                                cfg.forcePid0525TrackKpNmPerN :
                                                0.0;
                                    const double pLimit =
                                            std::isfinite(cfg.forcePid0525TrackPTorqueLimitNm) ?
                                                std::max(0.0, cfg.forcePid0525TrackPTorqueLimitNm) :
                                                0.0;
                                    const double rateDamping =
                                            std::isfinite(cfg.forcePid0525TrackForceRateDampingNmPerNps) ?
                                                std::max(0.0,
                                                         cfg.forcePid0525TrackForceRateDampingNmPerNps) :
                                                0.0;
                                    const double rateDampingLimit =
                                            std::isfinite(cfg.forcePid0525TrackForceRateDampingLimitNm) ?
                                                std::max(0.0,
                                                         cfg.forcePid0525TrackForceRateDampingLimitNm) :
                                                0.0;
                                    const double rateDeadband =
                                            std::isfinite(cfg.forcePid0525TrackForceRateDeadbandNPerSec) ?
                                                std::max(0.0,
                                                         cfg.forcePid0525TrackForceRateDeadbandNPerSec) :
                                                0.0;
                                    const double rateFilterHz =
                                            std::isfinite(cfg.forcePid0525TrackForceRateFilterHz) ?
                                                std::max(0.0,
                                                         cfg.forcePid0525TrackForceRateFilterHz) :
                                                0.0;
                                    hybridFeedForwardSensorTerm =
                                            trackKff * (expected - captureForce);
                                    hybridFeedForwardTerm =
                                            hardwareDirection * hybridFeedForwardSensorTerm;
                                    const double feedbackRaw = trackKp * error;
                                    const double feedbackLimited =
                                            pLimit > 0.0 ?
                                                std::min(std::max(feedbackRaw, -pLimit), pLimit) :
                                                0.0;
                                    hybridFeedbackSensorTerm = feedbackLimited;
                                    hybridFeedbackTerm = hardwareDirection * feedbackLimited;
                                    const bool rateDtValid =
                                            std::isfinite(feedbackDtSec) &&
                                            feedbackDtSec > 1e-6 &&
                                            feedbackDtSec <= kForcePid0525MaxDtSec;
                                    if(axisIndex < static_cast<int>(forcePid0525HybridRateInitialized.size()) &&
                                            axisIndex < static_cast<int>(forcePid0525HybridLastActualForceN.size()) &&
                                            axisIndex < static_cast<int>(forcePid0525HybridLastExpectedForceN.size()) &&
                                            axisIndex < static_cast<int>(forcePid0525HybridActualForceRateFilteredNPerSec.size()) &&
                                            axisIndex < static_cast<int>(forcePid0525HybridExpectedForceRateFilteredNPerSec.size())){
                                        if(!forcePid0525HybridRateInitialized[axisIndex] || !rateDtValid){
                                            forcePid0525HybridLastActualForceN[axisIndex] = curForce;
                                            forcePid0525HybridLastExpectedForceN[axisIndex] = expected;
                                            forcePid0525HybridActualForceRateFilteredNPerSec[axisIndex] = 0.0;
                                            forcePid0525HybridExpectedForceRateFilteredNPerSec[axisIndex] = 0.0;
                                            forcePid0525HybridRateInitialized[axisIndex] = true;
                                        }
                                        else{
                                            hybridActualForceRateRawNPerSec =
                                                    (curForce -
                                                     forcePid0525HybridLastActualForceN[axisIndex]) /
                                                    forcePid0525DtSec;
                                            hybridExpectedForceRateRawNPerSec =
                                                    axis.sensorIndex < static_cast<int>(expectedForceDerivative.size()) &&
                                                    std::isfinite(expectedForceDerivative[axis.sensorIndex]) ?
                                                        expectedForceDerivative[axis.sensorIndex] :
                                                        (expected -
                                                         forcePid0525HybridLastExpectedForceN[axisIndex]) /
                                                        forcePid0525DtSec;
                                            hybridActualForceRateFilteredNPerSec =
                                                    firstOrderLowPass(
                                                        forcePid0525HybridActualForceRateFilteredNPerSec[axisIndex],
                                                        hybridActualForceRateRawNPerSec,
                                                        forcePid0525DtSec,
                                                        rateFilterHz);
                                            hybridExpectedForceRateFilteredNPerSec =
                                                    firstOrderLowPass(
                                                        forcePid0525HybridExpectedForceRateFilteredNPerSec[axisIndex],
                                                        hybridExpectedForceRateRawNPerSec,
                                                        forcePid0525DtSec,
                                                        rateFilterHz);
                                            forcePid0525HybridLastActualForceN[axisIndex] = curForce;
                                            forcePid0525HybridLastExpectedForceN[axisIndex] = expected;
                                            forcePid0525HybridActualForceRateFilteredNPerSec[axisIndex] =
                                                    hybridActualForceRateFilteredNPerSec;
                                            forcePid0525HybridExpectedForceRateFilteredNPerSec[axisIndex] =
                                                    hybridExpectedForceRateFilteredNPerSec;
                                        }
                                        hybridActualForceRateFilteredNPerSec =
                                                forcePid0525HybridActualForceRateFilteredNPerSec[axisIndex];
                                        hybridExpectedForceRateFilteredNPerSec =
                                                forcePid0525HybridExpectedForceRateFilteredNPerSec[axisIndex];
                                    }
                                    hybridForceRateErrorNPerSec =
                                            hybridExpectedForceRateFilteredNPerSec -
                                            hybridActualForceRateFilteredNPerSec;
                                    const double rateErrorForControl =
                                            applySignedDeadband(hybridForceRateErrorNPerSec,
                                                                rateDeadband);
                                    if(rateDamping > 0.0 && rateDampingLimit > 0.0){
                                        const double rawRateDamping =
                                                rateDamping * rateErrorForControl;
                                        hybridForceRateDampingSensorTerm =
                                                std::min(std::max(rawRateDamping,
                                                                  -rateDampingLimit),
                                                         rateDampingLimit);
                                        hybridForceRateDampingTerm =
                                                hardwareDirection *
                                                hybridForceRateDampingSensorTerm;
                                    }
                                    trackTargetTorque =
                                            holdBias +
                                            hybridFeedForwardTerm +
                                            hybridFeedbackTerm +
                                            hybridForceRateDampingTerm -
                                            dampingTorque;
                                }
                                cmdTorque = trackTargetTorque;
                                const double selectedTrackSlewRateNmPerSec =
                                        feedForwardOnlyTrackMode ?
                                            feedForwardOnlyParams.trackTorqueSlewRateNmPerSec :
                                            cfg.forcePid0525TrackTorqueSlewRateNmPerSec;
                                activeTorqueSlewRate =
                                        std::isfinite(selectedTrackSlewRateNmPerSec) ?
                                            std::max(0.0, selectedTrackSlewRateNmPerSec) :
                                            torqueSlewRate;
                                if(state == ForcePid0525HybridState::TrackBlend){
                                    const double selectedTrackBlendTimeSec =
                                            feedForwardOnlyTrackMode ?
                                                feedForwardOnlyParams.trackBlendTimeSec :
                                                cfg.forcePid0525TrackBlendTimeSec;
                                    const double blendTime =
                                            std::isfinite(selectedTrackBlendTimeSec) ?
                                                std::max(0.0, selectedTrackBlendTimeSec) :
                                                0.15;
                                    forcePid0525HybridBlendElapsedSec[axisIndex] += forcePid0525DtSec;
                                    hybridBlend = blendTime > 1e-6 ?
                                                std::min(1.0,
                                                         forcePid0525HybridBlendElapsedSec[axisIndex] /
                                                         blendTime) :
                                                1.0;
                                    const double blendStart =
                                            forcePid0525HybridBlendStartTorqueNm[axisIndex];
                                    cmdTorque = (1.0 - hybridBlend) * blendStart +
                                            hybridBlend * trackTargetTorque;
                                    if(hybridBlend >= 1.0){
                                        forcePid0525HybridState[axisIndex] =
                                                ForcePid0525HybridState::Track;
                                    }
                                }
                                else{
                                    hybridBlend = 1.0;
                                }
                                hybridTrackPath = true;
                            }
                            else if(!forcePid0525HybridMotionActiveForLoop &&
                                    state == ForcePid0525HybridState::TrackExitBlend){
                                const double blendTime =
                                        std::isfinite(cfg.forcePid0525TrackBlendTimeSec) ?
                                            std::max(0.0, cfg.forcePid0525TrackBlendTimeSec) :
                                            0.15;
                                forcePid0525HybridBlendElapsedSec[axisIndex] += forcePid0525DtSec;
                                hybridBlend = blendTime > 1e-6 ?
                                            std::min(1.0,
                                                     forcePid0525HybridBlendElapsedSec[axisIndex] /
                                                     blendTime) :
                                            1.0;
                                const double blendStart =
                                        forcePid0525HybridBlendStartTorqueNm[axisIndex];
                                hybridHoldBias = blendStart;
                                hybridFeedbackTerm = staticPidTargetTorque - blendStart;
                                hybridFeedbackSensorTerm =
                                        hardwareDirection * hybridFeedbackTerm;
                                cmdTorque = (1.0 - hybridBlend) * blendStart +
                                        hybridBlend * staticPidTargetTorque;
                                activeTorqueSlewRate =
                                        std::isfinite(cfg.forcePid0525TrackTorqueSlewRateNmPerSec) ?
                                            std::max(0.0, cfg.forcePid0525TrackTorqueSlewRateNmPerSec) :
                                            torqueSlewRate;
                                hybridTrackPath = true;
                                if(hybridBlend >= 1.0){
                                    forcePid0525HybridState[axisIndex] =
                                            ForcePid0525HybridState::PreloadAcquire;
                                    forcePid0525HybridBiasValid[axisIndex] = false;
                                    forcePid0525HybridStableTimeSec[axisIndex] = 0.0;
                                    forcePid0525HybridBlendElapsedSec[axisIndex] = 0.0;
                                    forcePid0525HybridBlendStartTorqueNm[axisIndex] = 0.0;
                                    forcePid0525HybridRateInitialized[axisIndex] = false;
                                }
                            }
                            else if(forcePid0525HybridMotionActiveForLoop && !biasValid){
                                throttledInfo(QString("Motor%1 0525 dynamic tracking waits for learned bias before tracking.")
                                              .arg(axisIndex), "warning", 3000);
                            }
                            else if(std::isfinite(error) && std::fabs(error) <= learnErrorN){
                                forcePid0525HybridState[axisIndex] =
                                        ForcePid0525HybridState::BiasLearnHold;
                            }
                            else if(!forcePid0525HybridMotionActiveForLoop){
                                forcePid0525HybridStableTimeSec[axisIndex] = 0.0;
                                forcePid0525HybridBiasValid[axisIndex] = false;
                                forcePid0525HybridState[axisIndex] =
                                        ForcePid0525HybridState::PreloadAcquire;
                            }
                        }
                        }

                        const double cmdBeforeLimit = cmdTorque;
                        cmdTorque = std::min(std::max(cmdTorque, -torqueLimit), torqueLimit);
                        if(std::fabs(cmdTorque - cmdBeforeLimit) > kTorqueCommandEpsilonNm){
                            axisTorqueSaturated = true;
                            if(axisIndex < static_cast<int>(torqueSaturated.size())){
                                torqueSaturated[axisIndex] = 1;
                            }
                        }
                        if(activeTorqueSlewRate > 0.0 &&
                                axisIndex < static_cast<int>(lastTorqueCommandNm.size())){
                            double lastTorque = 0.0;
                            if(axisIndex < static_cast<int>(torqueCommandActive.size()) &&
                                    torqueCommandActive[axisIndex] &&
                                    std::isfinite(lastTorqueCommandNm[axisIndex])){
                                lastTorque = lastTorqueCommandNm[axisIndex];
                            }
                            else{
                                lastTorque = torqueBias;
                            }
                            const double maxStep =
                                    activeTorqueSlewRate * forcePid0525DtSec;
                            const double cmdBeforeSlew = cmdTorque;
                            cmdTorque = std::min(std::max(cmdTorque,
                                                          lastTorque - maxStep),
                                                 lastTorque + maxStep);
                            if(std::fabs(cmdTorque - cmdBeforeSlew) > kTorqueCommandEpsilonNm){
                                axisTorqueSlewLimited = true;
                                if(axisIndex < static_cast<int>(torqueSlewLimited.size())){
                                    torqueSlewLimited[axisIndex] = 1;
                                }
                            }
                        }
                        if(forcePid0525HybridActive && !hybridTrackPath &&
                                axisIndex < static_cast<int>(forcePid0525HybridState.size()) &&
                                !forcePid0525HybridMotionActiveForLoop){
                            const double expected =
                                    axis.sensorIndex < static_cast<int>(expectedForce.size()) ?
                                        expectedForce[axis.sensorIndex] :
                                        0.0;
                            const double error = expected - curForce;
                            const double learnErrorN =
                                    std::isfinite(cfg.forcePid0525BiasLearnErrorN) ?
                                        std::max(0.0, cfg.forcePid0525BiasLearnErrorN) :
                                        1.0;
                            const double learnHoldTimeSec =
                                    std::isfinite(cfg.forcePid0525BiasLearnHoldTimeSec) ?
                                        std::max(0.0, cfg.forcePid0525BiasLearnHoldTimeSec) :
                                        0.5;
                            if(std::isfinite(error) && std::fabs(error) <= learnErrorN){
                                forcePid0525HybridStableTimeSec[axisIndex] += forcePid0525DtSec;
                                forcePid0525HybridState[axisIndex] =
                                        ForcePid0525HybridState::BiasLearnHold;
                                if(forcePid0525HybridStableTimeSec[axisIndex] >= learnHoldTimeSec){
                                    forcePid0525HybridHoldBiasNm[axisIndex] = cmdTorque;
                                    forcePid0525HybridCaptureForceN[axisIndex] = expected;
                                    forcePid0525HybridBiasValid[axisIndex] = true;
                                }
                            }
                            else{
                                forcePid0525HybridStableTimeSec[axisIndex] = 0.0;
                                forcePid0525HybridBiasValid[axisIndex] = false;
                                forcePid0525HybridState[axisIndex] =
                                        ForcePid0525HybridState::PreloadAcquire;
                            }
                        }
                        if(collectForcePidTraceSample && forcePid0525HybridActive &&
                                axis.sensorIndex >= 0 &&
                                axis.sensorIndex < sensorCount){
                            pidError[axis.sensorIndex] = hybridError;
                            if(hybridTrackPath){
                                pidOutput[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            cmdTorque * hardwareDirection :
                                            (cmdTorque - hybridHoldBias) * hardwareDirection;
                                pidPTerm[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            0.0 :
                                            hybridBlend * hybridFeedbackSensorTerm;
                                pidITerm[axis.sensorIndex] = 0.0;
                                pidDTerm[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            0.0 :
                                            hybridBlend *
                                            (hybridForceRateDampingSensorTerm -
                                             hybridDampingSensorTerm);
                                pidIntegral[axis.sensorIndex] = 0.0;
                                pidMeasuredDerivativeRaw[axis.sensorIndex] =
                                        hybridActualForceRateRawNPerSec;
                                pidMeasuredDerivativeFiltered[axis.sensorIndex] =
                                        hybridActualForceRateFilteredNPerSec;
                                pidMeasuredDerivativeControl[axis.sensorIndex] =
                                        hybridActualForceRateFilteredNPerSec;
                                pidExpectedDerivativeRaw[axis.sensorIndex] =
                                        hybridExpectedForceRateRawNPerSec;
                                pidExpectedDerivativeFiltered[axis.sensorIndex] =
                                        hybridExpectedForceRateFilteredNPerSec;
                                pidFeedForwardRaw[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.staticForceTerm :
                                            hybridFeedForwardSensorTerm;
                                pidFeedForwardTerm[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.totalTerm :
                                            hybridBlend * hybridFeedForwardSensorTerm;
                                pidFeedForwardFrictionTerm[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.frictionTerm :
                                            0.0;
                                pidFeedForwardSelectedDynamicProfile[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.selectedDynamicProfile :
                                            0;
                                pidStaticFrictionDirection[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.staticFrictionDirection :
                                            0.0;
                                pidStaticFrictionSpeedScale[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.staticFrictionSpeedScale :
                                            0.0;
                                pidStaticFrictionRaw[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.staticFrictionRawTerm :
                                            0.0;
                                pidStaticFrictionAfterFade[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.staticFrictionAfterFadeTerm :
                                            0.0;
                                pidStaticFrictionAfterSmooth[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.staticFrictionAfterSmoothTerm :
                                            0.0;
                                pidFeedForwardVelocityTerm[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.velocityTerm :
                                            0.0;
                                pidFeedForwardAccelerationTerm[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridFeedForwardOnlyTerms.accelerationTerm :
                                            0.0;
                                pidExpectedRopeVelocityRadPerSec[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridExpectedRopeVelocityRadPerSec :
                                            0.0;
                                pidExpectedRopeAccelerationRadPerSec2[axis.sensorIndex] =
                                        hybridFeedForwardOnlyTrackPath ?
                                            hybridExpectedRopeAccelerationRadPerSec2 :
                                            0.0;
                                pidForceRateError[axis.sensorIndex] =
                                        hybridForceRateErrorNPerSec;
                                pidForceRateErrorDampingTerm[axis.sensorIndex] =
                                        hybridBlend * hybridForceRateDampingSensorTerm;
                            }
                        }
                        if(collectForcePidTraceSample && forcePid0525HybridActive &&
                                axisIndex < static_cast<int>(pid0525HybridStateTrace.size())){
                            pid0525HybridStateTrace[axisIndex] =
                                    static_cast<int>(forcePid0525HybridState[axisIndex]);
                            pid0525HybridBiasValidTrace[axisIndex] =
                                    forcePid0525HybridBiasValid[axisIndex] ? 1 : 0;
                            pid0525HybridHoldBiasTrace[axisIndex] =
                                    forcePid0525HybridHoldBiasNm[axisIndex];
                            pid0525HybridCaptureForceTrace[axisIndex] =
                                    forcePid0525HybridCaptureForceN[axisIndex];
                            pid0525HybridFeedForwardTrace[axisIndex] =
                                    hybridFeedForwardTerm;
                            pid0525HybridFeedbackTrace[axisIndex] =
                                    hybridFeedbackTerm;
                            pid0525HybridBlendTrace[axisIndex] =
                                    hybridBlend;
                        }

                        if(!commandTorqueModeAxis(axisIndex, cmdTorque)){
                            throttledInfo(QString("Motor%1 torque PID command failed.")
                                          .arg(axisIndex), "error");
                            if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                                forceControlFaultLatched[axisIndex] = true;
                            }
                            resetForceFeedbackChannel(axis.sensorIndex);
                            resetForcePid0525HybridAxis(axisIndex, false);
                            cmdTorque = 0.0;
                        }
                        else if(cfg.usePid &&
                                axis.sensorIndex >= 0 &&
                                axis.sensorIndex < sensorCount &&
                                !forceFeedbackCommitted[axis.sensorIndex]){
                            if(collectForcePidTraceSample){
                                pidOutputLimited[axis.sensorIndex] =
                                        (axisTorqueSaturated || axisTorqueSlewLimited) ? 1 : 0;
                            }
                            forceFeedbackCommitted[axis.sensorIndex] = true;
                        }
                        if(collectForcePidTraceSample){
                            forceControlAxisIndex.push_back(axisIndex);
                            forceControlSensorIndex.push_back(axis.sensorIndex);
                        }
                    }
                    else{
                        const bool forceAlgorithmEnabled =
                                cfg.usePid || useFeedForwardOnlyTestMode;
                        if(forceAlgorithmEnabled &&
                                axis.sensorIndex >= 0 &&
                                axis.sensorIndex < static_cast<int>(pidOutput.size()) &&
                                !forceFeedbackUpdated[axis.sensorIndex]){
                            if(useFeedForwardOnlyTestMode){
                                if(axis.sensorIndex < static_cast<int>(unloadFeedForwardBlend.size())){
                                    unloadFeedForwardBlend[axis.sensorIndex] = 0.0;
                                }
                                // Feed-forward-only is an open-loop diagnostic path.  It
                                // bypasses PID, fuzzy, high-tension scaling, unload scaling
                                // and slew limiting, but can include explicitly configured
                                // friction and expected rope kinematics feed-forward terms.
                                const double ropeVelocity =
                                        axis.sensorIndex < static_cast<int>(expectedRopeVelocityRadPerSec.size()) ?
                                            expectedRopeVelocityRadPerSec[axis.sensorIndex] :
                                            0.0;
                                const double ropeAcceleration =
                                        axis.sensorIndex < static_cast<int>(expectedRopeAccelerationRadPerSec2.size()) ?
                                            expectedRopeAccelerationRadPerSec2[axis.sensorIndex] :
                                            0.0;
                                const double expectedDerivative =
                                        axis.sensorIndex < static_cast<int>(expectedForceDerivative.size()) ?
                                            expectedForceDerivative[axis.sensorIndex] :
                                            0.0;
                                const double profileExpectedDerivative =
                                        feedForwardOnlyDirectionalProfileSelectionRate(
                                            forceFeedForwardOnlyDirectionalProfileSign,
                                            axis.sensorIndex,
                                            expectedDerivative,
                                            cfg.forceFeedForwardOnlyStaticFrictionForceRateDeadbandNPerSec);
                                const FeedForwardOnlyRuntimeParams feedForwardOnlyParams =
                                        feedForwardOnlyRuntimeParams(
                                            cfg,
                                            axis.sensorIndex,
                                            profileExpectedDerivative);
                                const FeedForwardOnlyTerms feedForwardTerms =
                                        feedForwardOnlyTorqueTerms(
                                            expectedForce[axis.sensorIndex],
                                            axis.motorCof,
                                            expectedDerivative,
                                            ropeVelocity,
                                            ropeAcceleration,
                                            feedForwardOnlyParams,
                                            axis.sensorIndex,
                                            limitedFeedbackDtSec,
                                            &forceFeedForwardOnlyStaticFrictionDirectionSign,
                                            &forceFeedForwardOnlyStaticFrictionSmoothedTorqueNm,
                                            &forceFeedForwardOnlyStaticFrictionSmoothedValid);
                                pidOutput[axis.sensorIndex] =
                                        feedForwardTerms.totalTerm;
                                if(collectForcePidTraceSample){
                                    pidError[axis.sensorIndex] =
                                            expectedForce[axis.sensorIndex] - curForce;
                                    pidExpectedDerivativeRaw[axis.sensorIndex] =
                                            expectedDerivative;
                                    pidExpectedDerivativeFiltered[axis.sensorIndex] =
                                            expectedDerivative;
                                    pidFeedForwardRaw[axis.sensorIndex] =
                                            feedForwardTerms.staticForceTerm;
                                    pidFeedForwardTerm[axis.sensorIndex] =
                                            feedForwardTerms.totalTerm;
                                    pidFeedForwardFrictionTerm[axis.sensorIndex] =
                                            feedForwardTerms.frictionTerm;
                                    pidFeedForwardSelectedDynamicProfile[axis.sensorIndex] =
                                            feedForwardTerms.selectedDynamicProfile;
                                    pidStaticFrictionDirection[axis.sensorIndex] =
                                            feedForwardTerms.staticFrictionDirection;
                                    pidStaticFrictionSpeedScale[axis.sensorIndex] =
                                            feedForwardTerms.staticFrictionSpeedScale;
                                    pidStaticFrictionRaw[axis.sensorIndex] =
                                            feedForwardTerms.staticFrictionRawTerm;
                                    pidStaticFrictionAfterFade[axis.sensorIndex] =
                                            feedForwardTerms.staticFrictionAfterFadeTerm;
                                    pidStaticFrictionAfterSmooth[axis.sensorIndex] =
                                            feedForwardTerms.staticFrictionAfterSmoothTerm;
                                    pidFeedForwardVelocityTerm[axis.sensorIndex] =
                                            feedForwardTerms.velocityTerm;
                                    pidFeedForwardAccelerationTerm[axis.sensorIndex] =
                                            feedForwardTerms.accelerationTerm;
                                    pidExpectedRopeVelocityRadPerSec[axis.sensorIndex] =
                                            ropeVelocity;
                                    pidExpectedRopeAccelerationRadPerSec2[axis.sensorIndex] =
                                            ropeAcceleration;
                                }
                            }
                            else if(cfg.usePid){
                                const double expectedDerivative =
                                        axis.sensorIndex < static_cast<int>(expectedForceDerivative.size()) ?
                                            expectedForceDerivative[axis.sensorIndex] :
                                            0.0;
                                const double unloadRateThreshold =
                                        std::isfinite(cfg.forceUnloadFeedForwardRateThresholdNPerSec) ?
                                            std::max(0.0, cfg.forceUnloadFeedForwardRateThresholdNPerSec) :
                                            0.0;
                                const double targetUnloadBlend =
                                        unloadRateThreshold > 0.0 &&
                                        expectedDerivative < -unloadRateThreshold ?
                                            1.0 :
                                            0.0;
                                const double unloadBlendTime =
                                        std::isfinite(cfg.forceUnloadFeedForwardBlendTimeSec) ?
                                            std::max(0.0, cfg.forceUnloadFeedForwardBlendTimeSec) :
                                            0.0;
                                if(axis.sensorIndex >= 0 &&
                                        axis.sensorIndex < static_cast<int>(unloadFeedForwardBlend.size())){
                                    double& blend = unloadFeedForwardBlend[axis.sensorIndex];
                                    if(unloadBlendTime <= 1e-9){
                                        blend = targetUnloadBlend;
                                    }
                                    else{
                                        blend = moveTowardValue(blend,
                                                               targetUnloadBlend,
                                                               limitedFeedbackDtSec / unloadBlendTime);
                                    }
                                }
                                const double unloadBlend =
                                        axis.sensorIndex >= 0 &&
                                        axis.sensorIndex < static_cast<int>(unloadFeedForwardBlend.size()) ?
                                            unloadFeedForwardBlend[axis.sensorIndex] :
                                            targetUnloadBlend;
                                forceControllerParams[axis.sensorIndex].feedForwardTorque =
                                        blendedForceFeedForwardTorqueNm(
                                            expectedForce[axis.sensorIndex],
                                            axis.motorCof,
                                            cfg,
                                            unloadBlend);
                                forceControllerParams[axis.sensorIndex].motorVelocityRevPerSec =
                                        axisIndex < static_cast<int>(motorVel.size()) &&
                                        std::isfinite(motorVel[axisIndex]) ?
                                            motorVel[axisIndex] :
                                            0.0;
                                pidOutput[axis.sensorIndex] =
                                        forceController.update(axis.sensorIndex,
                                                               curForce,
                                                               expectedForce[axis.sensorIndex],
                                                               limitedFeedbackDtSec,
                                                               forceControllerParams[axis.sensorIndex]);
                            }
                            forceFeedbackUpdated[axis.sensorIndex] = true;
                        }

                        cmdTorque =
                                forceAlgorithmEnabled &&
                                axis.sensorIndex < static_cast<int>(pidOutput.size()) ?
                                    pidOutput[axis.sensorIndex] * hardwareDirection :
                                    0.0;
                        double velocityDampingScale = 1.0;
                        if(!useFeedForwardOnlyTestMode &&
                                cfg.usePid &&
                                axis.sensorIndex >= 0 &&
                                axis.sensorIndex < sensorCount){
                            const ForceController::Debug debug =
                                    forceController.debug(axis.sensorIndex);
                            velocityDampingScale =
                                    std::isfinite(debug.fuzzyVelocityDampingScale) ?
                                        std::max(1.0, debug.fuzzyVelocityDampingScale) :
                                        1.0;
                        }
                        double dampingTorque = 0.0;
                        if(!useFeedForwardOnlyTestMode &&
                                velocityDamping > 0.0 &&
                                axisIndex < static_cast<int>(motorVel.size()) &&
                                std::isfinite(motorVel[axisIndex])){
                            dampingTorque =
                                    velocityDamping * velocityDampingScale * motorVel[axisIndex];
                            cmdTorque -= dampingTorque;
                        }
                        const double cmdBeforeLimit = cmdTorque;
                        cmdTorque = std::min(std::max(cmdTorque, -torqueLimit), torqueLimit);
                        if(std::fabs(cmdTorque - cmdBeforeLimit) > kTorqueCommandEpsilonNm){
                            axisTorqueSaturated = true;
                            if(axisIndex < static_cast<int>(torqueSaturated.size())){
                                torqueSaturated[axisIndex] = 1;
                            }
                        }
                        const bool hasPreviousTorqueCommand =
                                axisIndex < static_cast<int>(lastTorqueCommandNm.size()) &&
                                axisIndex < static_cast<int>(torqueCommandActive.size()) &&
                                torqueCommandActive[axisIndex] &&
                                std::isfinite(lastTorqueCommandNm[axisIndex]);
                        if(!useFeedForwardOnlyTestMode &&
                                torqueSlewRate > 0.0 &&
                                hasPreviousTorqueCommand){
                            const double lastTorque = lastTorqueCommandNm[axisIndex];
                            const double maxStep = torqueSlewRate * limitedFeedbackDtSec;
                            const double cmdBeforeSlew = cmdTorque;
                            cmdTorque = std::min(std::max(cmdTorque,
                                                          lastTorque - maxStep),
                                                 lastTorque + maxStep);
                            if(std::fabs(cmdTorque - cmdBeforeSlew) > kTorqueCommandEpsilonNm){
                                axisTorqueSlewLimited = true;
                                if(axisIndex < static_cast<int>(torqueSlewLimited.size())){
                                    torqueSlewLimited[axisIndex] = 1;
                                }
                            }
                        }
                        const bool commandAccepted = commandTorqueModeAxis(axisIndex, cmdTorque);
                        if(!commandAccepted){
                            throttledInfo(QString("Motor%1 torque PID command failed.")
                                          .arg(axisIndex), "error");
                            if(axisIndex < static_cast<int>(forceControlFaultLatched.size())){
                                forceControlFaultLatched[axisIndex] = true;
                            }
                            resetForceFeedbackChannel(axis.sensorIndex);
                            cmdTorque = 0.0;
                        }
                        else{
                            if(!useFeedForwardOnlyTestMode &&
                                    cfg.usePid &&
                                    axis.sensorIndex >= 0 &&
                                    axis.sensorIndex < sensorCount &&
                                    !forceFeedbackCommitted[axis.sensorIndex]){
                                const bool outputLimited =
                                        axisTorqueSaturated || axisTorqueSlewLimited;
                                const double appliedFeedbackTorque =
                                        (cmdTorque + dampingTorque) * hardwareDirection;
                                forceController.commit(axis.sensorIndex,
                                                       appliedFeedbackTorque,
                                                       outputLimited,
                                                       forceControllerParams[axis.sensorIndex]);
                                forceFeedbackCommitted[axis.sensorIndex] = true;
                                if(collectForcePidTraceSample){
                                    const ForceController::Debug debug =
                                            forceController.debug(axis.sensorIndex);
                                    pidError[axis.sensorIndex] = debug.error;
                                    pidPTerm[axis.sensorIndex] = debug.pTerm;
                                    pidITerm[axis.sensorIndex] = debug.iTerm;
                                    pidDTerm[axis.sensorIndex] = debug.dTerm;
                                    pidIntegral[axis.sensorIndex] = debug.integral;
                                    pidMeasuredDerivativeRaw[axis.sensorIndex] =
                                            debug.measuredDerivativeRaw;
                                    pidMeasuredDerivativeFiltered[axis.sensorIndex] =
                                            debug.measuredDerivativeFiltered;
                                    pidMeasuredDerivativeControl[axis.sensorIndex] =
                                            debug.measuredDerivativeControl;
                                    pidExpectedDerivativeRaw[axis.sensorIndex] =
                                            debug.expectedDerivativeRaw;
                                    pidExpectedDerivativeFiltered[axis.sensorIndex] =
                                            debug.expectedDerivativeFiltered;
                                    pidFeedForwardRaw[axis.sensorIndex] =
                                            debug.feedForwardRawTerm;
                                    pidFeedForwardTerm[axis.sensorIndex] =
                                            debug.feedForwardTerm;
                                    pidExpectedRateFeedForwardTerm[axis.sensorIndex] =
                                            debug.expectedRateFeedForwardTerm;
                                    pidExpectedRateFeedForwardScale[axis.sensorIndex] =
                                            debug.expectedRateFeedForwardScale;
                                    pidForceRateError[axis.sensorIndex] =
                                            debug.forceRateError;
                                    pidForceRateErrorDampingTerm[axis.sensorIndex] =
                                            debug.forceRateErrorDampingTerm;
                                    pidPlatformCaptureTerm[axis.sensorIndex] =
                                            debug.platformCaptureTerm;
                                    pidPlatformCaptureTargetTerm[axis.sensorIndex] =
                                            debug.platformCaptureTargetTerm;
                                    pidPlatformCaptureState[axis.sensorIndex] =
                                            debug.platformCaptureState;
                                    pidFuzzyFeedForwardTargetScale[axis.sensorIndex] =
                                            debug.fuzzyFeedForwardTargetScale;
                                    pidFuzzyFeedForwardScale[axis.sensorIndex] =
                                            debug.fuzzyFeedForwardScale;
                                    pidFuzzyFeedForwardRecoveryRate[axis.sensorIndex] =
                                            debug.fuzzyFeedForwardRecoveryRate;
                                    pidFuzzyKpScale[axis.sensorIndex] =
                                            debug.fuzzyKpScale;
                                    pidFuzzyKiScale[axis.sensorIndex] =
                                            debug.fuzzyKiScale;
                                    pidFuzzyVelocityDampingScale[axis.sensorIndex] =
                                            debug.fuzzyVelocityDampingScale;
                                    pidFuzzyPositivePLimit[axis.sensorIndex] =
                                            debug.fuzzyPositivePLimit;
                                    pidFuzzyNegativePLimit[axis.sensorIndex] =
                                            debug.fuzzyNegativePLimit;
                                    pidFuzzyFeedForwardRecoveryLimited[axis.sensorIndex] =
                                            debug.fuzzyFeedForwardRecoveryLimited ? 1 : 0;
                                    pidFuzzyPLimitApplied[axis.sensorIndex] =
                                            debug.fuzzyPLimitApplied ? 1 : 0;
                                    pidFuzzyState[axis.sensorIndex] =
                                            debug.fuzzyState;
                                    pidIntegralReleaseApplied[axis.sensorIndex] =
                                            debug.integralReleaseApplied ? 1 : 0;
                                    pidAntiWindup[axis.sensorIndex] =
                                            debug.antiWindupAdjusted ? 1 : 0;
                                    pidOutputLimited[axis.sensorIndex] =
                                            debug.outputLimited ? 1 : 0;
                                }
                            }
                            else if(useFeedForwardOnlyTestMode &&
                                    axis.sensorIndex >= 0 &&
                                    axis.sensorIndex < sensorCount &&
                                    !forceFeedbackCommitted[axis.sensorIndex]){
                                forceFeedbackCommitted[axis.sensorIndex] = true;
                                if(collectForcePidTraceSample){
                                    pidOutputLimited[axis.sensorIndex] =
                                            (axisTorqueSaturated || axisTorqueSlewLimited) ? 1 : 0;
                                }
                            }
                            if(collectForcePidTraceSample){
                                forceControlAxisIndex.push_back(axisIndex);
                                forceControlSensorIndex.push_back(axis.sensorIndex);
                            }
                        }
                    }
                    if(axisIndex < static_cast<int>(motorCommand.size())){
                        motorCommand[axisIndex] = cmdTorque;
                    }
                }

            }
            if(!axis.forceControlEnabled && !cfg.pvtActiveOrPaused){
                stopTorqueModeAxis(axisIndex);
                resetForceFeedbackChannel(axis.sensorIndex);
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
        }

        for(int sensorIndex=0; sensorIndex<sensorCount; ++sensorIndex){
            if(sensorIndex >= static_cast<int>(forceFeedbackReferenced.size()) ||
                    !forceFeedbackReferenced[sensorIndex]){
                resetForceFeedbackChannel(sensorIndex);
            }
        }

        if(collectForcePidTraceSample){
            const qint64 sampleWallClockUs =
                    lastSensorFrameWallClockUs > 0 ?
                        lastSensorFrameWallClockUs :
                        loopWallClockMs * 1000;
            ForcePidTraceSample traceSample;
            traceSample.wallClockUs = sampleWallClockUs;
            traceSample.controlDtSec = controlDtSec;
            traceSample.forceSensorValue = forceSensorValue;
            traceSample.expectedForce = expectedForce;
            traceSample.motorCommand = motorCommand;
            traceSample.motorTorqueNm = motorTorqueNm;
            traceSample.pidOutput = std::move(pidOutput);
            traceSample.pidError = std::move(pidError);
            traceSample.pidPTerm = std::move(pidPTerm);
            traceSample.pidITerm = std::move(pidITerm);
            traceSample.pidDTerm = std::move(pidDTerm);
            traceSample.pidIntegral = std::move(pidIntegral);
            traceSample.pidMeasuredDerivativeRaw = std::move(pidMeasuredDerivativeRaw);
            traceSample.pidMeasuredDerivativeFiltered = std::move(pidMeasuredDerivativeFiltered);
            traceSample.pidMeasuredDerivativeControl = std::move(pidMeasuredDerivativeControl);
            traceSample.pidExpectedDerivativeRaw = std::move(pidExpectedDerivativeRaw);
            traceSample.pidExpectedDerivativeFiltered = std::move(pidExpectedDerivativeFiltered);
            traceSample.pidFeedForwardRaw = std::move(pidFeedForwardRaw);
            traceSample.pidFeedForwardTerm = std::move(pidFeedForwardTerm);
            traceSample.pidFeedForwardFrictionTerm = std::move(pidFeedForwardFrictionTerm);
            traceSample.pidFeedForwardSelectedDynamicProfile =
                    std::move(pidFeedForwardSelectedDynamicProfile);
            traceSample.pidStaticFrictionDirection = std::move(pidStaticFrictionDirection);
            traceSample.pidStaticFrictionSpeedScale = std::move(pidStaticFrictionSpeedScale);
            traceSample.pidStaticFrictionRaw = std::move(pidStaticFrictionRaw);
            traceSample.pidStaticFrictionAfterFade = std::move(pidStaticFrictionAfterFade);
            traceSample.pidStaticFrictionAfterSmooth = std::move(pidStaticFrictionAfterSmooth);
            traceSample.pidFeedForwardVelocityTerm = std::move(pidFeedForwardVelocityTerm);
            traceSample.pidFeedForwardAccelerationTerm = std::move(pidFeedForwardAccelerationTerm);
            traceSample.pidExpectedRopeVelocityRadPerSec =
                    std::move(pidExpectedRopeVelocityRadPerSec);
            traceSample.pidExpectedRopeAccelerationRadPerSec2 =
                    std::move(pidExpectedRopeAccelerationRadPerSec2);
            traceSample.pidExpectedRateFeedForwardTerm = std::move(pidExpectedRateFeedForwardTerm);
            traceSample.pidExpectedRateFeedForwardScale = std::move(pidExpectedRateFeedForwardScale);
            traceSample.pidForceRateError = std::move(pidForceRateError);
            traceSample.pidForceRateErrorDampingTerm = std::move(pidForceRateErrorDampingTerm);
            traceSample.pidPlatformCaptureTerm = std::move(pidPlatformCaptureTerm);
            traceSample.pidPlatformCaptureTargetTerm = std::move(pidPlatformCaptureTargetTerm);
            traceSample.pidPlatformCaptureState = std::move(pidPlatformCaptureState);
            traceSample.pidFuzzyFeedForwardTargetScale = std::move(pidFuzzyFeedForwardTargetScale);
            traceSample.pidFuzzyFeedForwardScale = std::move(pidFuzzyFeedForwardScale);
            traceSample.pidFuzzyFeedForwardRecoveryRate = std::move(pidFuzzyFeedForwardRecoveryRate);
            traceSample.pidFuzzyKpScale = std::move(pidFuzzyKpScale);
            traceSample.pidFuzzyKiScale = std::move(pidFuzzyKiScale);
            traceSample.pidFuzzyVelocityDampingScale = std::move(pidFuzzyVelocityDampingScale);
            traceSample.pidFuzzyPositivePLimit = std::move(pidFuzzyPositivePLimit);
            traceSample.pidFuzzyNegativePLimit = std::move(pidFuzzyNegativePLimit);
            traceSample.pidFuzzyFeedForwardRecoveryLimited =
                    std::move(pidFuzzyFeedForwardRecoveryLimited);
            traceSample.pidFuzzyPLimitApplied = std::move(pidFuzzyPLimitApplied);
            traceSample.pidFuzzyState = std::move(pidFuzzyState);
            traceSample.pidIntegralReleaseApplied = std::move(pidIntegralReleaseApplied);
            traceSample.pidAntiWindup = std::move(pidAntiWindup);
            traceSample.pidOutputLimited = std::move(pidOutputLimited);
            traceSample.torqueSaturated = std::move(torqueSaturated);
            traceSample.torqueSlewLimited = std::move(torqueSlewLimited);
            traceSample.motorVel = motorVel;
            traceSample.pid0525HybridState = std::move(pid0525HybridStateTrace);
            traceSample.pid0525HybridBiasValid = std::move(pid0525HybridBiasValidTrace);
            traceSample.pid0525HybridHoldBiasNm = std::move(pid0525HybridHoldBiasTrace);
            traceSample.pid0525HybridCaptureForceN = std::move(pid0525HybridCaptureForceTrace);
            traceSample.pid0525HybridFeedForwardTermNm = std::move(pid0525HybridFeedForwardTrace);
            traceSample.pid0525HybridFeedbackTermNm = std::move(pid0525HybridFeedbackTrace);
            traceSample.pid0525HybridBlend = std::move(pid0525HybridBlendTrace);
            traceSample.forceControlAxisIndex = std::move(forceControlAxisIndex);
            traceSample.forceControlSensorIndex = std::move(forceControlSensorIndex);
            appendForcePidTraceSample(std::move(traceSample));
            emittedForcePidTraceSample = true;
        }
    }

    if(!actualTorqueLimitEmergencyDetected &&
            !(forceThreadRunning &&
              static_cast<int>(forceSensorValue.size()) >= cfg.sensorCount) &&
            lastForceThreadEnabled &&
            !cfg.pvtActiveOrPaused){
        for(int axisIndex=0; axisIndex<std::min(cfg.axisCount, static_cast<int>(cfg.axes.size())); ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(axis.isMotorAxis && axis.sensorIndex >= 0){
                stopTorqueModeAxis(axisIndex);
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
        }
    }

    // 最后再检查软件位置限位。这里使用相对电机位置，超过边界直接触发全轴急停。
    // Emit baseline samples when recording starts before force control is active.
    // Emit baseline samples when recording starts before force control is active.
    if(cfg.forcePidTuningHighRateSampleEnabled &&
            !emittedForcePidTraceSample &&
            static_cast<int>(forceSensorValue.size()) >= cfg.sensorCount){
        const int sensorCount = std::max(cfg.sensorCount, 0);
        const int axisCount = std::max(cfg.axisCount, 0);
        const qint64 sampleWallClockUs =
                lastSensorFrameWallClockUs > 0 ?
                    lastSensorFrameWallClockUs :
                    loopWallClockMs * 1000;
        const double controlDtSec =
                timingDiagnostics.latestControlLoopIntervalUs > 0 ?
                    static_cast<double>(timingDiagnostics.latestControlLoopIntervalUs) / 1000000.0 :
                    targetIntervalSec;

        ForcePidTraceSample traceSample;
        traceSample.wallClockUs = sampleWallClockUs;
        traceSample.controlDtSec = controlDtSec;
        traceSample.forceSensorValue = forceSensorValue;
        traceSample.expectedForce = expectedForce;
        traceSample.motorCommand = motorCommand;
        traceSample.motorTorqueNm = motorTorqueNm;
        traceSample.pidOutput = std::vector<double>(sensorCount, 0.0);
        traceSample.pidError = std::vector<double>(sensorCount, 0.0);
        traceSample.pidPTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidITerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidDTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidIntegral = std::vector<double>(sensorCount, 0.0);
        traceSample.pidMeasuredDerivativeRaw = std::vector<double>(sensorCount, 0.0);
        traceSample.pidMeasuredDerivativeFiltered = std::vector<double>(sensorCount, 0.0);
        traceSample.pidMeasuredDerivativeControl = std::vector<double>(sensorCount, 0.0);
        traceSample.pidExpectedDerivativeRaw = std::vector<double>(sensorCount, 0.0);
        traceSample.pidExpectedDerivativeFiltered = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFeedForwardRaw = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFeedForwardTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFeedForwardFrictionTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFeedForwardSelectedDynamicProfile = std::vector<int>(sensorCount, 0);
        traceSample.pidStaticFrictionDirection = std::vector<double>(sensorCount, 0.0);
        traceSample.pidStaticFrictionSpeedScale = std::vector<double>(sensorCount, 0.0);
        traceSample.pidStaticFrictionRaw = std::vector<double>(sensorCount, 0.0);
        traceSample.pidStaticFrictionAfterFade = std::vector<double>(sensorCount, 0.0);
        traceSample.pidStaticFrictionAfterSmooth = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFeedForwardVelocityTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFeedForwardAccelerationTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidExpectedRopeVelocityRadPerSec = std::vector<double>(sensorCount, 0.0);
        traceSample.pidExpectedRopeAccelerationRadPerSec2 = std::vector<double>(sensorCount, 0.0);
        traceSample.pidExpectedRateFeedForwardTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidExpectedRateFeedForwardScale = std::vector<double>(sensorCount, 1.0);
        traceSample.pidForceRateError = std::vector<double>(sensorCount, 0.0);
        traceSample.pidForceRateErrorDampingTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidPlatformCaptureTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidPlatformCaptureTargetTerm = std::vector<double>(sensorCount, 0.0);
        traceSample.pidPlatformCaptureState = std::vector<int>(sensorCount, 0);
        traceSample.pidFuzzyFeedForwardTargetScale = std::vector<double>(sensorCount, 1.0);
        traceSample.pidFuzzyFeedForwardScale = std::vector<double>(sensorCount, 1.0);
        traceSample.pidFuzzyFeedForwardRecoveryRate = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFuzzyKpScale = std::vector<double>(sensorCount, 1.0);
        traceSample.pidFuzzyKiScale = std::vector<double>(sensorCount, 1.0);
        traceSample.pidFuzzyVelocityDampingScale = std::vector<double>(sensorCount, 1.0);
        traceSample.pidFuzzyPositivePLimit = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFuzzyNegativePLimit = std::vector<double>(sensorCount, 0.0);
        traceSample.pidFuzzyFeedForwardRecoveryLimited = std::vector<int>(sensorCount, 0);
        traceSample.pidFuzzyPLimitApplied = std::vector<int>(sensorCount, 0);
        traceSample.pidFuzzyState = std::vector<int>(sensorCount, 0);
        traceSample.pidIntegralReleaseApplied = std::vector<int>(sensorCount, 0);
        traceSample.pidAntiWindup = std::vector<int>(sensorCount, 0);
        traceSample.pidOutputLimited = std::vector<int>(sensorCount, 0);
        traceSample.torqueSaturated = std::vector<int>(axisCount, 0);
        traceSample.torqueSlewLimited = std::vector<int>(axisCount, 0);
        traceSample.motorVel = motorVel;
        traceSample.pid0525HybridState = std::vector<int>(axisCount, 0);
        traceSample.pid0525HybridBiasValid = std::vector<int>(axisCount, 0);
        traceSample.pid0525HybridHoldBiasNm = std::vector<double>(axisCount, 0.0);
        traceSample.pid0525HybridCaptureForceN = std::vector<double>(axisCount, 0.0);
        traceSample.pid0525HybridFeedForwardTermNm = std::vector<double>(axisCount, 0.0);
        traceSample.pid0525HybridFeedbackTermNm = std::vector<double>(axisCount, 0.0);
        traceSample.pid0525HybridBlend = std::vector<double>(axisCount, 0.0);
        appendForcePidTraceSample(std::move(traceSample));
    }

    // Check software position limits after trace sampling.
    bool softwareLimitEmergencyDetected = false;
    int softwareLimitEmergencyAxisIndex = -1;
    if(leadshineConnected){
        const int axisCount = std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
        for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(cfg.commissioningModeActive &&
                    axisIndex != cfg.commissioningAxisIndex){
                continue;
            }
            if(!axis.isMotorAxis || axisIndex >= static_cast<int>(motorRelRawPos.size())){
                continue;
            }

            const double rawPos = motorRelRawPos[axisIndex];
            const bool hasValidRange = std::isfinite(axis.motorMin) &&
                    std::isfinite(axis.motorMax) &&
                    axis.motorMax > axis.motorMin;
            if(!std::isfinite(rawPos)){
                throttledInfo(QString("Software position limit check skipped: motor %1 position feedback is invalid.")
                              .arg(axisIndex), "warning");
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
                continue;
            }

            if(!hasValidRange){
                continue;
            }

            const bool reachedOrExceededLimit = rawPos > axis.motorMax || rawPos < axis.motorMin;
            const bool motorPositionLimitRecoveryAxis =
                    cfg.motorPositionLimitRecoveryActive &&
                    axisIndex < static_cast<int>(cfg.motorPositionLimitRecoveryAxes.size()) &&
                    cfg.motorPositionLimitRecoveryAxes[axisIndex];
            if(reachedOrExceededLimit){
                if(motorPositionLimitRecoveryAxis){
                    throttledInfo(QString("超限恢复：电机%1当前位置%2仍在软件位置限位[%3, %4]外，暂不重复触发软件急停。")
                                  .arg(axisIndex)
                                  .arg(rawPos, 0, 'f', 6)
                                  .arg(axis.motorMin, 0, 'f', 6)
                                  .arg(axis.motorMax, 0, 'f', 6),
                                  "warning",
                                  3000);
                    continue;
                }
                softwareLimitEmergencyDetected = true;
                if(softwareLimitEmergencyAxisIndex < 0){
                    softwareLimitEmergencyAxisIndex = axisIndex;
                }
                throttledInfo(QString("软件急停：电机%1当前位置%2已达到或超过软件位置限位[%3, %4]")
                              .arg(axisIndex)
                              .arg(rawPos, 0, 'f', 6)
                              .arg(axis.motorMin, 0, 'f', 6)
                              .arg(axis.motorMax, 0, 'f', 6),
                              "error");
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
        }
    }
    if(softwareLimitEmergencyDetected){
        if(!softwareLimitEmergencyStopActive){
            if(cfg.commissioningModeActive && softwareLimitEmergencyAxisIndex >= 0){
                hardwareInterface->emergencyStopAxes(
                            std::vector<int>{softwareLimitEmergencyAxisIndex});
            }
            else{
                hardwareInterface->emergencyStopAll();
            }
            resetTorqueCommandState(cfg.axisCount);
            resetForceFeedbackState(cfg.sensorCount);
            softwareLimitEmergencyStopActive = true;
        }
    }
    else{
        softwareLimitEmergencyStopActive = false;
    }

    lastForceThreadEnabled = forceThreadRunning;
    lastAllCableForceDragModeEnabled =
            forceThreadRunning && cfg.allCableForceDragModeEnabled;
    lastForceFeedForwardOnlyTestModeEnabled =
            forceThreadRunning && cfg.forceFeedForwardOnlyTestModeEnabled;
    lastForcePidOutputMode =
            forceThreadRunning ? cfg.forcePidOutputMode : ForcePidOutputMode::Pid0624;
    lastForcePid0525HybridEnabled = forcePid0525HybridEnabledForLoop;
    lastForcePid0525DynamicTrackMode =
            forceThreadRunning ?
                forcePid0525DynamicTrackModeForLoop :
                ForcePid0525DynamicTrackMode::AC;
    lastForcePid0525UseBangBangPretension =
            forceThreadRunning ? cfg.forcePid0525UseBangBangPretension : true;
    lastForcePid0525HybridMotionActive = forcePid0525HybridMotionActiveForLoop;
    updateSnapshot(cfg, motorAbsPos, motorRelRawPos, motorVel, motorTorqueNm, motorCommand,
                   forceSensorValue, expectedForce, forceThreadRunning, expectedFromExternal,
                   traceSnapshot);
    previousControlLoopDurationUs = std::max<qint64>(
                0, monotonicNowUs() - loopNowUs);
}

void ControlWorker::sensorLoop()
{
    // 传感器循环优先读取驱动 Trace 缓冲；一次可能返回多个历史样本，需要逐帧重建时间戳和诊断统计。
    if(!hardwareInterface){
        latestForceSensorValue.clear();
        hasLatestForceSensorValue = false;
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return;
    }

    const Config cfg = currentConfig();
    if(!cfg.useLeadshine || cfg.sensorCount <= 0){
        nextSensorReadDueUs = 0;
        lastSensorSampleIntervalUs = 0;
        lastSensorFrameTimestampUs = 0;
        lastSensorFrameWallClockUs = 0;
        lastTraceExpandedSensorFrameTimestampUs = 0;
        lastSensorTraceReadCallUs = 0;
        latestForceSensorValue.clear();
        hasLatestForceSensorValue = false;
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return;
    }

    const qint64 targetIntervalUs = traceDrivenWorkerIntervalUs(cfg);
    const qint64 loopEntryUs = monotonicNowUs();
    if(lastSensorSampleIntervalUs != targetIntervalUs){
        lastSensorSampleIntervalUs = targetIntervalUs;
        nextSensorReadDueUs = loopEntryUs;
        lastSensorFrameTimestampUs = 0;
        lastSensorFrameWallClockUs = 0;
        lastTraceExpandedSensorFrameTimestampUs = 0;
        lastSensorTraceReadCallUs = 0;
    }
    if(nextSensorReadDueUs <= 0){
        nextSensorReadDueUs = loopEntryUs;
    }
    if(loopEntryUs < nextSensorReadDueUs){
        return;
    }

    std::vector<HardwareInterface::ForceSensorTraceSample> traceSamples =
            hardwareInterface->readForceSensorDataTraceSamples();
    const qint64 readCompleteUs = monotonicNowUs();
    const qint64 readCompleteWallClockUs = QDateTime::currentMSecsSinceEpoch() * 1000;
    const qint64 traceReadCallIntervalUs = lastSensorTraceReadCallUs > 0 ?
                std::max<qint64>(0, readCompleteUs - lastSensorTraceReadCallUs) :
                0;
    lastSensorTraceReadCallUs = readCompleteUs;
    if(kEnableControlWorkerDiagnosticRawHistory){
        const qint64 readCompleteWallClockMs = readCompleteWallClockUs / 1000;
        const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
        if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                           readCompleteWallClockUs,
                                           lastSensorTraceReadRawHistoryAppendUs)){
            QMutexLocker locker(&timingHistoryMutex);
            sensorTraceReadRawHistory.append(DiagnosticRawSample{readCompleteWallClockMs,
                                                                 traceReadCallIntervalUs,
                                                                 readCompleteWallClockUs});
            if(readCompleteWallClockMs - lastSensorTraceReadHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistoryForMode(sensorTraceReadRawHistory,
                                      readCompleteWallClockMs,
                                      fullRawRecording,
                                      kDiagnosticRawDefaultMaxSamples);
                lastSensorTraceReadHistoryTrimMs = readCompleteWallClockMs;
            }
        }
    }
    if(!traceSamples.empty()){
        // Trace 样本自带时间戳时使用硬件/采样时间；缺失时用本次读取完成时间按固定采样间隔回推。
        while(nextSensorReadDueUs <= readCompleteUs){
            nextSensorReadDueUs += targetIntervalUs;
        }

        const qint64 traceFrameIntervalUs = targetIntervalUs;
        const qint64 firstSampleUs = lastSensorFrameTimestampUs > 0 ?
                    lastSensorFrameTimestampUs + traceFrameIntervalUs :
                    readCompleteUs - static_cast<qint64>(traceSamples.size() - 1) * traceFrameIntervalUs;
        const qint64 firstWallClockUs = lastSensorFrameWallClockUs > 0 ?
                    lastSensorFrameWallClockUs + traceFrameIntervalUs :
                    readCompleteWallClockUs - static_cast<qint64>(traceSamples.size() - 1) * traceFrameIntervalUs;
        bool appendedHistory = false;

        for(std::size_t sampleIndex = 0; sampleIndex < traceSamples.size(); ++sampleIndex){
            HardwareInterface::ForceSensorTraceSample& traceSample = traceSamples[sampleIndex];
            std::vector<double> forceSensorValue = std::move(traceSample.values);
            if(forceSensorValue.empty()){
                continue;
            }
            if(static_cast<int>(forceSensorValue.size()) < cfg.sensorCount){
                forceSensorValue.resize(cfg.sensorCount, 0.0);
            }

            const qint64 sampleUs = traceSample.monotonicUs > 0 ?
                        traceSample.monotonicUs :
                        firstSampleUs + static_cast<qint64>(sampleIndex) * traceFrameIntervalUs;
            const qint64 sampleWallClockUs = traceSample.wallClockUs > 0 ?
                        traceSample.wallClockUs :
                        firstWallClockUs + static_cast<qint64>(sampleIndex) * traceFrameIntervalUs;
            const qint64 sampleWallClockMs = sampleWallClockUs / 1000;
            qint64 sensorDtUs = 0;
            qint64 traceExpandedSensorDtUs = 0;
            const bool hasPreviousTraceExpandedSensorFrame =
                    lastTraceExpandedSensorFrameTimestampUs > 0;
            if(hasPreviousTraceExpandedSensorFrame){
                traceExpandedSensorDtUs =
                        std::max<qint64>(0, sampleUs - lastTraceExpandedSensorFrameTimestampUs);
            }
            lastTraceExpandedSensorFrameTimestampUs = sampleUs;

            timingDiagnostics.sensorFrameCount++;
            if(lastSensorFrameTimestampUs > 0){
                sensorDtUs = std::max<qint64>(0, sampleUs - lastSensorFrameTimestampUs);
                timingDiagnostics.sensorFrameIntervalCount++;
                timingDiagnostics.sensorFrameIntervalSumUs += sensorDtUs;
                timingDiagnostics.latestSensorFrameIntervalUs = sensorDtUs;
                if(kEnableControlWorkerDiagnosticRawHistory &&
                        hasPreviousTraceExpandedSensorFrame){
                    const bool fullRawRecording =
                            diagnosticRawHistoryFullRecordingEnabled.load();
                    if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                                       sampleWallClockUs,
                                                       lastSensorFrameRawHistoryAppendUs)){
                        QMutexLocker locker(&timingHistoryMutex);
                        sensorFrameRawHistory.append(DiagnosticRawSample{sampleWallClockMs,
                                                                         traceExpandedSensorDtUs,
                                                                         sampleWallClockUs,
                                                                         true,
                                                                         true});
                        appendedHistory = true;
                    }
                }
            }
            lastSensorFrameTimestampUs = sampleUs;
            lastSensorFrameWallClockUs = sampleWallClockUs;

            const double filterDtSec =
                    sensorDtUs > 0 ? static_cast<double>(sensorDtUs) / 1000000.0 : 0.0;
            forceSensorValue = applyForceSensorLowPass(cfg, forceSensorValue, filterDtSec);
            latestForceSensorValue = std::move(forceSensorValue);
            hasLatestForceSensorValue = true;
            if(kEnableControlWorkerDiagnosticRawHistory){
                const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
                if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                                   sampleWallClockUs,
                                                   lastSensorValueRawHistoryAppendUs)){
                    QMutexLocker locker(&timingHistoryMutex);
                    sensorValueRawHistory.append(SensorValueSample{sampleWallClockMs,
                                                                   latestForceSensorValue,
                                                                   sampleWallClockUs,
                                                                   true,
                                                                   true});
                    appendedHistory = true;
                }
            }
        }

        if(kEnableControlWorkerDiagnosticRawHistory && appendedHistory){
            const qint64 trimWallClockMs = readCompleteWallClockUs / 1000;
            const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
            QMutexLocker locker(&timingHistoryMutex);
            if(trimWallClockMs - lastSensorFrameHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistoryForMode(sensorFrameRawHistory,
                                      trimWallClockMs,
                                      fullRawRecording,
                                      kDiagnosticRawDefaultMaxSamples);
                lastSensorFrameHistoryTrimMs = trimWallClockMs;
            }
            if(trimWallClockMs - lastSensorValueHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistoryForMode(sensorValueRawHistory,
                                      trimWallClockMs,
                                      fullRawRecording,
                                      kDiagnosticSensorValueDefaultMaxSamples);
                lastSensorValueHistoryTrimMs = trimWallClockMs;
            }
        }
        return;
    }

    const HardwareInterface::ForceSensorReadResult forceSensorRead =
            hardwareInterface->readForceSensorDataCachedResult(1);
    if(forceSensorRead.fromTrace){
        while(nextSensorReadDueUs <= readCompleteUs){
            nextSensorReadDueUs += targetIntervalUs;
        }
        return;
    }
    while(nextSensorReadDueUs <= readCompleteUs){
        nextSensorReadDueUs += targetIntervalUs;
    }

    std::vector<double> forceSensorValue = forceSensorRead.values;
    if(forceSensorValue.empty()){
        latestForceSensorValue.clear();
        hasLatestForceSensorValue = false;
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return;
    }
    if(static_cast<int>(forceSensorValue.size()) < cfg.sensorCount){
        forceSensorValue.resize(cfg.sensorCount, 0.0);
    }

    const qint64 sensorNowUs = readCompleteUs;
    const qint64 sensorWallClockMs = QDateTime::currentMSecsSinceEpoch();
    const qint64 sensorWallClockUs = sensorWallClockMs * 1000;
    qint64 sensorDtUs = 0;
    const bool hasNewSensorRead = forceSensorRead.frameCount > 0;
    if(hasNewSensorRead){
        timingDiagnostics.sensorFrameCount++;
    }
    if(hasNewSensorRead && lastSensorFrameTimestampUs > 0){
        const qint64 dtUs = std::max<qint64>(0, sensorNowUs - lastSensorFrameTimestampUs);
        sensorDtUs = dtUs;
        timingDiagnostics.sensorFrameIntervalCount++;
        timingDiagnostics.sensorFrameIntervalSumUs += dtUs;
        timingDiagnostics.latestSensorFrameIntervalUs = dtUs;
        if(kEnableControlWorkerDiagnosticRawHistory){
            const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
            if(shouldAppendDiagnosticRawSample(fullRawRecording,
                                               sensorWallClockUs,
                                               lastSensorFrameRawHistoryAppendUs)){
                QMutexLocker locker(&timingHistoryMutex);
                sensorFrameRawHistory.append(DiagnosticRawSample{sensorWallClockMs,
                                                                 dtUs,
                                                                 sensorWallClockUs,
                                                                 false,
                                                                 false});
                if(sensorWallClockMs - lastSensorFrameHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                    trimRawHistoryForMode(sensorFrameRawHistory,
                                          sensorWallClockMs,
                                          fullRawRecording,
                                          kDiagnosticRawDefaultMaxSamples);
                    lastSensorFrameHistoryTrimMs = sensorWallClockMs;
                }
            }
        }
    }
    if(hasNewSensorRead){
        lastSensorFrameTimestampUs = sensorNowUs;
        lastSensorFrameWallClockUs = sensorWallClockUs;
    }

    const double filterDtSec = sensorDtUs > 0 ? static_cast<double>(sensorDtUs) / 1000000.0 : 0.0;
    forceSensorValue = applyForceSensorLowPass(cfg, forceSensorValue, filterDtSec);
    latestForceSensorValue = std::move(forceSensorValue);
    hasLatestForceSensorValue = true;
    if(kEnableControlWorkerDiagnosticRawHistory){
        const bool fullRawRecording = diagnosticRawHistoryFullRecordingEnabled.load();
        const bool appendSensorValue =
                hasNewSensorRead &&
                shouldAppendDiagnosticRawSample(fullRawRecording,
                                                sensorWallClockUs,
                                                lastSensorValueRawHistoryAppendUs);
        if(appendSensorValue){
            QMutexLocker locker(&timingHistoryMutex);
            sensorValueRawHistory.append(SensorValueSample{sensorWallClockMs,
                                                           latestForceSensorValue,
                                                           sensorWallClockUs,
                                                           false,
                                                           false});
            if(sensorWallClockMs - lastSensorValueHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistoryForMode(sensorValueRawHistory,
                                      sensorWallClockMs,
                                      fullRawRecording,
                                      kDiagnosticSensorValueDefaultMaxSamples);
                lastSensorValueHistoryTrimMs = sensorWallClockMs;
            }
        }
    }
}

void ControlWorker::updateSnapshot(const Config& cfg,
                                   const std::vector<double>& motorAbsPos,
                                   const std::vector<double>& motorRelRawPos,
                                   const std::vector<double>& motorVel,
                                   const std::vector<double>& motorTorqueNm,
                                   const std::vector<double>& motorCommand,
                                   const std::vector<double>& forceSensorValue,
                                   const std::vector<double>& expectedForce,
                                   bool forceThreadRunning,
                                   bool expectedFromExternal,
                                   const HardwareInterface::RuntimeTraceSnapshot& runtimeTraceSnapshot)
{
    QMutexLocker locker(&snapshotMutex);
    snapshot.sequence++;
    snapshot.motorAbsPos = motorAbsPos;
    snapshot.motorRelRawPos = motorRelRawPos;
    snapshot.motorVel = motorVel;
    snapshot.motorTraceCommandVelocity = runtimeTraceSnapshot.motorCommandVelocity;
    snapshot.motorTraceActualVelocity = runtimeTraceSnapshot.motorActualVelocity;
    snapshot.motorTraceStatusWord = runtimeTraceSnapshot.motorStatusWord;
    snapshot.motorTraceStateMachine = runtimeTraceSnapshot.motorStateMachine;
    const EndpointRemoteStatus remoteStatus = endpointRemoteControl.status();
    snapshot.endpointRemoteControlActive = endpointRemoteControl.isActive();
    snapshot.endpointRemoteControlRunning =
            remoteStatus.state == EndpointRemoteStatus::State::Running;
    snapshot.endpointRemoteTracePhase = endpointRemoteTracePhase;
    snapshot.motorTorqueNm = motorTorqueNm;
    snapshot.motorCommand = motorCommand;
    snapshot.forceSensorValue = forceSensorValue;
    snapshot.expectedForce = expectedForce;
    snapshot.forceThreadRunning = forceThreadRunning;
    snapshot.forceExpectedFromExternal = expectedFromExternal;
    snapshot.runtimeTraceFrameWallClockUs = runtimeTraceSnapshot.wallClockUs;
    snapshot.runtimeTraceFrameMonotonicUs = runtimeTraceSnapshot.monotonicUs;
    snapshot.runtimeTraceNewestFrameAgeUs = runtimeTraceSnapshot.newestFrameAgeUs;
    snapshot.runtimeTraceCurrentFrameAgeUs = runtimeTraceSnapshot.newestFrameAgeUs;
    snapshot.runtimeTraceFrameCount = runtimeTraceSnapshot.frameCount;
    snapshot.runtimeTraceSamplePeriodUs = runtimeTraceSnapshot.traceSamplePeriodUs;
    snapshot.runtimeTraceLogicalFrameSequence = runtimeTraceSnapshot.logicalFrameSequence;
    snapshot.runtimeTraceUsageProfile = runtimeTraceSnapshot.usageProfile;
    snapshot.runtimeTraceUsageProfileGeneration =
            runtimeTraceSnapshot.usageProfileGeneration;
    snapshot.runtimeTraceConfigurationGeneration =
            runtimeTraceSnapshot.configurationGeneration;
    snapshot.runtimeTraceEndpointRemoteSessionToken =
            runtimeTraceSnapshot.endpointRemoteSessionToken;
    snapshot.runtimeTraceStatusFaultLatched =
            runtimeTraceSnapshot.endpointRemoteVelocitySafety.statusFaultLatched;
    snapshot.runtimeTraceStatusFaultAxis =
            runtimeTraceSnapshot.endpointRemoteVelocitySafety.statusFaultAxis;
    snapshot.runtimeTraceStatusFaultWord =
            runtimeTraceSnapshot.endpointRemoteVelocitySafety.statusFaultWord;
    snapshot.runtimeTraceStatusFaultStateMachine =
            runtimeTraceSnapshot.endpointRemoteVelocitySafety
                .statusFaultStateMachine;
    snapshot.runtimeTraceStatusFaultLogicalFrameSequence =
            runtimeTraceSnapshot.endpointRemoteVelocitySafety
                .statusFaultLogicalFrameSequence;
    snapshot.runtimeTraceFromHardware = runtimeTraceSnapshot.fromTrace;
    snapshot.runtimeTraceFrameSequenceValid = runtimeTraceSnapshot.frameSequenceValid;
    snapshot.runtimeTraceTimingReliable = runtimeTraceSnapshot.timingReliable;
    snapshot.runtimeTraceFifoCaughtUp = runtimeTraceSnapshot.fifoCaughtUp;
    snapshot.runtimeTraceLost = runtimeTraceSnapshot.traceLost;
    snapshot.forceSensorTraceFrameMonotonicUs =
            runtimeTraceSnapshot.forceSensorFrameMonotonicUs;
    snapshot.forcePidOutputMode = cfg.forcePidOutputMode;
    snapshot.timingDiagnostics = timingDiagnostics;
}

void ControlWorker::throttledInfo(const QString& message, const std::string& type, int throttleMs)
{
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(nowMs - lastInfoMs < throttleMs){
        return;
    }
    lastInfoMs = nowMs;
    emit displayInfoSignal(message.toStdString(), type);
}
