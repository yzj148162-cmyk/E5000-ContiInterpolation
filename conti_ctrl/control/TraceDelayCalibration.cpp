#include "control/TraceDelayCalibration.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kMinimumFitRSquared = 0.98;
constexpr double kMaximumAcceptedDelayS = 0.020;
constexpr double kMaximumPairSpreadS = 0.0015;

int findAxisIndex(const TraceTelemetryFrame &frame, quint16 axis)
{
    for (int index = 0; index < frame.axisCount; ++index) {
        if (frame.axes[index] == axis) {
            return index;
        }
    }
    return -1;
}
}

TraceDelayFitResult TraceDelayCalibrationAnalyzer::analyze(
    const TraceDelayCalibrationConfig &config,
    int traceSamplePeriodUs,
    const QVector<TraceDelaySegmentCapture> &segments)
{
    TraceDelayFitResult result;
    result.axisResult.axis = config.axis;
    result.axisResult.valid = false;
    result.axisResult.calibrated = false;
    result.axisResult.appliedDelayMs = 8.0;
    result.axisResult.source = QStringLiteral("默认");

    if (traceSamplePeriodUs <= 0 || segments.size() != 6) {
        result.axisResult.detail = QStringLiteral("标定数据结构不完整");
        return result;
    }

    const double pulseToDegree = 1.0 / MotorUnit::kPhysicalPulsesPerDegree;
    const int requestedSamples = std::max(
        10, static_cast<int>(std::llround(
                config.sampleWindowMs * 1000.0 / traceSamplePeriodUs)));
    int lostFrames = 0;

    for (const TraceDelaySegmentCapture &segment : segments) {
        QVector<const TraceTelemetryFrame *> stableFrames;
        QVector<const TraceTelemetryFrame *> currentStableRun;
        const double speedTolerance =
            std::max(0.2, std::abs(segment.targetSpeedDegreePerSecond) * 0.02);
        const double actualSpeedTolerance =
            std::max(0.5, std::abs(segment.targetSpeedDegreePerSecond) * 0.05);
        quint64 previousSequence = 0;
        for (const TraceTelemetryFrame &frame : segment.frames) {
            if (previousSequence > 0 && frame.traceSequence > previousSequence + 1) {
                lostFrames += static_cast<int>(frame.traceSequence - previousSequence - 1);
                currentStableRun.clear();
            }
            previousSequence = frame.traceSequence;
            const int axisIndex = findAxisIndex(frame, config.axis);
            if (axisIndex < 0
                || (frame.validAxisMask & static_cast<quint8>(1U << axisIndex)) == 0U) {
                currentStableRun.clear();
                continue;
            }
            const double commandVelocity =
                frame.commandVelocityPulsePerSecond[axisIndex] * pulseToDegree;
            const double actualVelocity =
                frame.actualVelocityPulsePerSecond[axisIndex] * pulseToDegree;
            const bool stable =
                std::abs(commandVelocity - segment.targetSpeedDegreePerSecond)
                    <= speedTolerance
                && std::abs(actualVelocity - commandVelocity)
                    <= actualSpeedTolerance;
            if (stable) {
                currentStableRun.push_back(&frame);
                if (currentStableRun.size() > stableFrames.size()) {
                    stableFrames = currentStableRun;
                }
            } else {
                currentStableRun.clear();
            }
        }

        if (stableFrames.size() < requestedSamples) {
            result.axisResult.lostFrameCount = lostFrames;
            result.axisResult.detail = QStringLiteral(
                "速度 %1°/s 的稳定Trace不足：需要%2帧，实际%3帧")
                    .arg(segment.targetSpeedDegreePerSecond, 0, 'f', 3)
                    .arg(requestedSamples)
                    .arg(stableFrames.size());
            return result;
        }

        const int first = (stableFrames.size() - requestedSamples) / 2;
        double speedSum = 0.0;
        double gapSum = 0.0;
        for (int index = first; index < first + requestedSamples; ++index) {
            const TraceTelemetryFrame &frame = *stableFrames.at(index);
            const int axisIndex = findAxisIndex(frame, config.axis);
            speedSum += frame.commandVelocityPulsePerSecond[axisIndex] * pulseToDegree;
            gapSum += (frame.commandPulse[axisIndex] - frame.actualPulse[axisIndex])
                * pulseToDegree;
        }
        result.measuredSpeedDegreePerSecond.push_back(speedSum / requestedSamples);
        result.measuredPositionGapDegree.push_back(gapSum / requestedSamples);
    }

    const int count = result.measuredSpeedDegreePerSecond.size();
    double meanSpeed = 0.0;
    double meanGap = 0.0;
    for (int index = 0; index < count; ++index) {
        meanSpeed += result.measuredSpeedDegreePerSecond.at(index);
        meanGap += result.measuredPositionGapDegree.at(index);
    }
    meanSpeed /= count;
    meanGap /= count;

    double speedVariance = 0.0;
    double covariance = 0.0;
    for (int index = 0; index < count; ++index) {
        const double dv = result.measuredSpeedDegreePerSecond.at(index) - meanSpeed;
        const double de = result.measuredPositionGapDegree.at(index) - meanGap;
        speedVariance += dv * dv;
        covariance += dv * de;
    }
    if (speedVariance <= std::numeric_limits<double>::epsilon()) {
        result.axisResult.detail = QStringLiteral("标定速度缺少有效变化");
        return result;
    }

    const double slopeS = covariance / speedVariance;
    const double interceptDegree = meanGap - slopeS * meanSpeed;
    double squaredError = 0.0;
    double totalGapVariance = 0.0;
    for (int index = 0; index < count; ++index) {
        const double predicted =
            slopeS * result.measuredSpeedDegreePerSecond.at(index) + interceptDegree;
        const double residual = result.measuredPositionGapDegree.at(index) - predicted;
        squaredError += residual * residual;
        const double centered = result.measuredPositionGapDegree.at(index) - meanGap;
        totalGapVariance += centered * centered;
    }
    const double rSquared = totalGapVariance > std::numeric_limits<double>::epsilon()
        ? 1.0 - squaredError / totalGapVariance : 0.0;
    const double rmseDegree = std::sqrt(squaredError / count);

    double pairSpreadS = 0.0;
    for (int level = 0; level < 3; ++level) {
        const int positiveIndex = level * 2;
        const int negativeIndex = positiveIndex + 1;
        const double deltaVelocity =
            result.measuredSpeedDegreePerSecond.at(positiveIndex)
            - result.measuredSpeedDegreePerSecond.at(negativeIndex);
        if (std::abs(deltaVelocity) < 1e-9) {
            result.axisResult.detail = QStringLiteral("正反向速度差无效");
            return result;
        }
        const double pairDelayS =
            (result.measuredPositionGapDegree.at(positiveIndex)
             - result.measuredPositionGapDegree.at(negativeIndex))
            / deltaVelocity;
        pairSpreadS = std::max(pairSpreadS, std::abs(pairDelayS - slopeS));
    }

    result.axisResult.measuredDelayMs = slopeS * 1000.0;
    result.axisResult.staticOffsetDegree = interceptDegree;
    result.axisResult.rSquared = rSquared;
    result.axisResult.rmseDegree = rmseDegree;
    result.axisResult.pairSpreadMs = pairSpreadS * 1000.0;
    result.axisResult.lostFrameCount = lostFrames;
    result.axisResult.valid = lostFrames == 0
        && slopeS >= 0.0 && slopeS <= kMaximumAcceptedDelayS
        && rSquared >= kMinimumFitRSquared
        && pairSpreadS <= kMaximumPairSpreadS;
    result.axisResult.calibrated = result.axisResult.valid;
    if (result.axisResult.valid) {
        result.axisResult.appliedDelayMs = result.axisResult.measuredDelayMs;
        result.axisResult.source = QStringLiteral("实测");
        result.axisResult.detail = QStringLiteral("标定通过");
    } else {
        result.axisResult.appliedDelayMs = 8.0;
        result.axisResult.source = QStringLiteral("默认");
        result.axisResult.detail = QStringLiteral(
            "标定未通过：要求无丢帧、0～20 ms、R²≥0.98、正反向离散≤1.5 ms");
    }
    return result;
}
