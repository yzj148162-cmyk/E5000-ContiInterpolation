#include "control/TraceDelayCalibration.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QStringList>

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
    QStringList invalidSegments;

    for (int segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex) {
        const TraceDelaySegmentCapture &segment = segments.at(segmentIndex);
        TraceDelaySegmentDiagnostic diagnostic;
        diagnostic.segmentNumber = segmentIndex + 1;
        diagnostic.targetSpeedDegreePerSecond =
            segment.targetSpeedDegreePerSecond;
        diagnostic.capturedFrames = segment.frames.size();
        if (!segment.frames.isEmpty()) {
            diagnostic.firstSequence = segment.frames.constFirst().traceSequence;
            diagnostic.lastSequence = segment.frames.constLast().traceSequence;
        }

        QVector<const TraceTelemetryFrame *> stableFrames;
        QVector<const TraceTelemetryFrame *> currentStableRun;
        const double speedTolerance =
            std::max(0.2, std::abs(segment.targetSpeedDegreePerSecond) * 0.02);
        quint64 previousSequence = 0;
        for (const TraceTelemetryFrame &frame : segment.frames) {
            bool sequenceContinuous = true;
            if (previousSequence > 0 && frame.traceSequence != previousSequence + 1) {
                const int missing = frame.traceSequence > previousSequence + 1
                    ? static_cast<int>(frame.traceSequence - previousSequence - 1)
                    : 1;
                diagnostic.lostFrames += missing;
                lostFrames += missing;
                sequenceContinuous = false;
                currentStableRun.clear();
            }
            previousSequence = frame.traceSequence;
            const int axisIndex = findAxisIndex(frame, config.axis);
            if (axisIndex < 0
                || (frame.validAxisMask & static_cast<quint8>(1U << axisIndex)) == 0U) {
                currentStableRun.clear();
                continue;
            }
            ++diagnostic.validAxisFrames;
            const double commandVelocity =
                frame.commandVelocityPulsePerSecond[axisIndex] * pulseToDegree;
            const bool commandStable =
                std::abs(commandVelocity - segment.targetSpeedDegreePerSecond)
                <= speedTolerance;
            if (commandStable) {
                ++diagnostic.commandStableFrames;
                if (!sequenceContinuous) {
                    currentStableRun.clear();
                }
                currentStableRun.push_back(&frame);
                if (currentStableRun.size() > stableFrames.size()) {
                    stableFrames = currentStableRun;
                }
            } else {
                currentStableRun.clear();
            }
        }
        diagnostic.longestCommandStableRun = stableFrames.size();

        if (stableFrames.size() < requestedSamples) {
            diagnostic.detail = QStringLiteral(
                "指令稳定Trace不足：需要%1帧，最长连续%2帧")
                                    .arg(requestedSamples)
                                    .arg(stableFrames.size());
            invalidSegments << QStringLiteral(
                "第%1段(%2°/s)：%3")
                .arg(diagnostic.segmentNumber)
                .arg(segment.targetSpeedDegreePerSecond, 0, 'f', 3)
                .arg(diagnostic.detail);
            result.segmentDiagnostics.push_back(diagnostic);
            continue;
        }

        diagnostic.selectedFrames = requestedSamples;
        const int first = (stableFrames.size() - requestedSamples) / 2;
        diagnostic.selectedFirstSequence = stableFrames.at(first)->traceSequence;
        diagnostic.selectedLastSequence =
            stableFrames.at(first + requestedSamples - 1)->traceSequence;
        double commandSpeedSum = 0.0;
        double actualSpeedSum = 0.0;
        double actualSpeedSquaredSum = 0.0;
        double gapSum = 0.0;
        for (int index = first; index < first + requestedSamples; ++index) {
            const TraceTelemetryFrame &frame = *stableFrames.at(index);
            const int axisIndex = findAxisIndex(frame, config.axis);
            const double commandVelocity =
                frame.commandVelocityPulsePerSecond[axisIndex] * pulseToDegree;
            const double actualVelocity =
                frame.actualVelocityPulsePerSecond[axisIndex] * pulseToDegree;
            commandSpeedSum += commandVelocity;
            actualSpeedSum += actualVelocity;
            actualSpeedSquaredSum += actualVelocity * actualVelocity;
            gapSum += (frame.commandPulse[axisIndex] - frame.actualPulse[axisIndex])
                * pulseToDegree;
        }
        diagnostic.commandVelocityMeanDegreePerSecond =
            commandSpeedSum / requestedSamples;
        diagnostic.actualVelocityMeanDegreePerSecond =
            actualSpeedSum / requestedSamples;
        diagnostic.positionGapMeanDegree = gapSum / requestedSamples;
        const double actualVariance = std::max(
            0.0,
            actualSpeedSquaredSum / requestedSamples
                - diagnostic.actualVelocityMeanDegreePerSecond
                    * diagnostic.actualVelocityMeanDegreePerSecond);
        diagnostic.actualVelocityStdDegreePerSecond = std::sqrt(actualVariance);

        const double actualMeanTolerance =
            std::max(1.0, std::abs(segment.targetSpeedDegreePerSecond) * 0.05);
        if (std::abs(diagnostic.actualVelocityMeanDegreePerSecond
                     - diagnostic.commandVelocityMeanDegreePerSecond)
            > actualMeanTolerance) {
            diagnostic.detail = QStringLiteral(
                "实际速度均值未跟上指令：指令=%1°/s，实际=%2°/s，容差=%3°/s")
                                    .arg(diagnostic.commandVelocityMeanDegreePerSecond,
                                         0, 'f', 4)
                                    .arg(diagnostic.actualVelocityMeanDegreePerSecond,
                                         0, 'f', 4)
                                    .arg(actualMeanTolerance, 0, 'f', 4);
            invalidSegments << QStringLiteral(
                "第%1段(%2°/s)：%3")
                .arg(diagnostic.segmentNumber)
                .arg(segment.targetSpeedDegreePerSecond, 0, 'f', 3)
                .arg(diagnostic.detail);
            result.segmentDiagnostics.push_back(diagnostic);
            continue;
        }

        diagnostic.accepted = true;
        diagnostic.detail = QStringLiteral("已选取稳定窗口");
        result.measuredSpeedDegreePerSecond.push_back(
            diagnostic.commandVelocityMeanDegreePerSecond);
        result.measuredPositionGapDegree.push_back(
            diagnostic.positionGapMeanDegree);
        result.segmentDiagnostics.push_back(diagnostic);
    }

    result.axisResult.lostFrameCount = lostFrames;
    if (!invalidSegments.isEmpty()
        || result.measuredSpeedDegreePerSecond.size() != 6
        || result.measuredPositionGapDegree.size() != 6) {
        result.axisResult.detail = invalidSegments.isEmpty()
            ? QStringLiteral("未得到完整的六段拟合数据")
            : invalidSegments.join(QStringLiteral("；"));
        return result;
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
