#include "tracedelaycalibration.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <QStringList>

namespace {
constexpr double kDefaultDelayMs = 8.0;
constexpr double kMinimumFitRSquared = 0.98;
constexpr double kMaximumAcceptedDelayS = 0.020;
constexpr double kMaximumPairSpreadS = 0.0015;

quint32 sequenceDistance(quint32 previous, quint32 current)
{
    return current >= previous ? current - previous
                               : (std::numeric_limits<quint32>::max() - previous) + current + 1U;
}
}

TraceDelayFitResult TraceDelayCalibrationAnalyzer::analyze(
        const TraceDelayCalibrationConfig& config,
        int traceSamplePeriodUs,
        const std::vector<TraceDelayCalibrationSegment>& segments)
{
    TraceDelayFitResult result;
    result.axisResult.axis = config.axis;
    result.axisResult.appliedDelayMs = kDefaultDelayMs;
    if(traceSamplePeriodUs <= 0 || segments.size() != 6U){
        result.axisResult.detail = QStringLiteral("标定数据结构不完整");
        return result;
    }

    const int requestedSamples = std::max(10, static_cast<int>(std::llround(
        config.sampleWindowMs * 1000.0 / traceSamplePeriodUs)));
    QStringList invalid;
    int totalLost = 0;
    for(size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex){
        const auto& segment = segments[segmentIndex];
        TraceDelaySegmentDiagnostic diagnostic;
        diagnostic.segmentNumber = static_cast<int>(segmentIndex) + 1;
        diagnostic.targetVelocityUnitPerSec = segment.targetVelocityUnitPerSec;
        diagnostic.capturedFrames = static_cast<int>(segment.samples.size());
        // Trace type03/04 以整数板卡 unit/s 返回；当前两种执行器模板均约定
        // 1 unit=1°，因此稳定窗口容差至少覆盖一个量化台阶。
        const double velocityTolerance = std::max(
            1.1, std::fabs(segment.targetVelocityUnitPerSec) * 0.02);
        std::vector<const TraceDelayCalibrationSample*> longest;
        std::vector<const TraceDelayCalibrationSample*> run;
        quint32 previousSequence = 0;
        bool previousValid = false;
        for(const auto& sample : segment.samples){
            if(!sample.valid){
                run.clear();
                previousValid = false;
                continue;
            }
            if(previousValid){
                const quint32 distance = sequenceDistance(previousSequence, sample.frameSequence);
                if(distance != 1U){
                    const int lost = static_cast<int>(distance > 1U ? distance - 1U : 1U);
                    diagnostic.lostFrames += lost;
                    totalLost += lost;
                    run.clear();
                }
            }
            previousSequence = sample.frameSequence;
            previousValid = true;
            if(std::fabs(sample.commandVelocityUnitPerSec - segment.targetVelocityUnitPerSec)
                    <= velocityTolerance){
                run.push_back(&sample);
                ++diagnostic.stableFrames;
                if(run.size() > longest.size()) longest = run;
            } else {
                run.clear();
            }
        }
        if(static_cast<int>(longest.size()) < requestedSamples){
            diagnostic.detail = QStringLiteral("连续稳定 Trace 不足：需要 %1 帧，最长 %2 帧")
                    .arg(requestedSamples).arg(longest.size());
            invalid.push_back(QStringLiteral("第 %1 段：%2")
                              .arg(diagnostic.segmentNumber).arg(diagnostic.detail));
            result.segmentDiagnostics.push_back(diagnostic);
            continue;
        }

        const int first = (static_cast<int>(longest.size()) - requestedSamples) / 2;
        diagnostic.selectedFrames = requestedSamples;
        diagnostic.selectedFirstSequence = longest[first]->frameSequence;
        diagnostic.selectedLastSequence = longest[first + requestedSamples - 1]->frameSequence;
        double commandSum = 0.0, actualSum = 0.0, actualSquaredSum = 0.0, gapSum = 0.0;
        for(int i = first; i < first + requestedSamples; ++i){
            const auto& sample = *longest[i];
            commandSum += sample.commandVelocityUnitPerSec;
            actualSum += sample.actualVelocityUnitPerSec;
            actualSquaredSum += sample.actualVelocityUnitPerSec * sample.actualVelocityUnitPerSec;
            gapSum += sample.commandPositionUnit - sample.actualPositionUnit;
        }
        diagnostic.commandVelocityMean = commandSum / requestedSamples;
        diagnostic.actualVelocityMean = actualSum / requestedSamples;
        diagnostic.positionGapMean = gapSum / requestedSamples;
        diagnostic.actualVelocityStd = std::sqrt(std::max(0.0,
            actualSquaredSum / requestedSamples -
            diagnostic.actualVelocityMean * diagnostic.actualVelocityMean));
        const double actualTolerance = std::max(1.0,
            std::fabs(diagnostic.commandVelocityMean) * 0.05);
        if(std::fabs(diagnostic.actualVelocityMean - diagnostic.commandVelocityMean)
                > actualTolerance){
            diagnostic.detail = QStringLiteral("实际速度未稳定跟上指令：指令=%1，实际=%2，容差=%3")
                    .arg(diagnostic.commandVelocityMean, 0, 'f', 4)
                    .arg(diagnostic.actualVelocityMean, 0, 'f', 4)
                    .arg(actualTolerance, 0, 'f', 4);
            invalid.push_back(QStringLiteral("第 %1 段：%2")
                              .arg(diagnostic.segmentNumber).arg(diagnostic.detail));
            result.segmentDiagnostics.push_back(diagnostic);
            continue;
        }
        diagnostic.accepted = true;
        diagnostic.detail = QStringLiteral("已选取稳定窗口");
        result.fittedVelocity.push_back(diagnostic.commandVelocityMean);
        result.fittedPositionGap.push_back(diagnostic.positionGapMean);
        result.segmentDiagnostics.push_back(diagnostic);
    }

    result.axisResult.lostFrameCount = totalLost;
    if(!invalid.isEmpty() || result.fittedVelocity.size() != 6U){
        result.axisResult.detail = invalid.isEmpty() ? QStringLiteral("未得到完整六段数据")
                                                     : invalid.join(QStringLiteral("；"));
        return result;
    }

    const int count = static_cast<int>(result.fittedVelocity.size());
    double meanVelocity = 0.0, meanGap = 0.0;
    for(int i = 0; i < count; ++i){
        meanVelocity += result.fittedVelocity[i];
        meanGap += result.fittedPositionGap[i];
    }
    meanVelocity /= count;
    meanGap /= count;
    double variance = 0.0, covariance = 0.0;
    for(int i = 0; i < count; ++i){
        const double dv = result.fittedVelocity[i] - meanVelocity;
        const double dg = result.fittedPositionGap[i] - meanGap;
        variance += dv * dv;
        covariance += dv * dg;
    }
    if(variance <= std::numeric_limits<double>::epsilon()){
        result.axisResult.detail = QStringLiteral("标定速度缺少有效变化");
        return result;
    }
    const double slopeS = covariance / variance;
    const double intercept = meanGap - slopeS * meanVelocity;
    double squaredError = 0.0, totalGapVariance = 0.0;
    for(int i = 0; i < count; ++i){
        const double predicted = slopeS * result.fittedVelocity[i] + intercept;
        const double residual = result.fittedPositionGap[i] - predicted;
        squaredError += residual * residual;
        const double centered = result.fittedPositionGap[i] - meanGap;
        totalGapVariance += centered * centered;
    }
    const double rSquared = totalGapVariance > std::numeric_limits<double>::epsilon()
            ? 1.0 - squaredError / totalGapVariance : 0.0;
    double pairSpreadS = 0.0;
    for(int level = 0; level < 3; ++level){
        const int positive = level * 2;
        const int negative = positive + 1;
        const double deltaVelocity = result.fittedVelocity[positive] - result.fittedVelocity[negative];
        if(std::fabs(deltaVelocity) <= 1.0e-12){
            result.axisResult.detail = QStringLiteral("正反向速度差无效");
            return result;
        }
        const double pairDelay = (result.fittedPositionGap[positive] -
                                  result.fittedPositionGap[negative]) / deltaVelocity;
        pairSpreadS = std::max(pairSpreadS, std::fabs(pairDelay - slopeS));
    }

    auto& axis = result.axisResult;
    axis.calibrated = true;
    axis.measuredDelayMs = slopeS * 1000.0;
    axis.staticOffsetUnit = intercept;
    axis.rSquared = rSquared;
    axis.rmseUnit = std::sqrt(squaredError / count);
    axis.pairSpreadMs = pairSpreadS * 1000.0;
    axis.valid = totalLost == 0 && slopeS >= 0.0 && slopeS <= kMaximumAcceptedDelayS &&
            rSquared >= kMinimumFitRSquared && pairSpreadS <= kMaximumPairSpreadS;
    if(axis.valid){
        axis.appliedDelayMs = axis.measuredDelayMs;
        axis.source = QStringLiteral("实测");
        axis.detail = QStringLiteral("标定通过");
    } else {
        axis.detail = QStringLiteral("标定未通过：要求无丢帧、0~20 ms、R²≥0.98、正反向离散≤1.5 ms");
    }
    return result;
}
