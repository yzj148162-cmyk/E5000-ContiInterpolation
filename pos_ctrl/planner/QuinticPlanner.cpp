#include "planner/QuinticPlanner.h"

#include <QtMath>
#include <algorithm>

namespace {
constexpr double kFineStepS = 0.001;
constexpr double kMinDurationS = 0.001;
}

void QuinticPlanner::reset(const MotionConfig &config, qint64 sequenceId, qint32 startPosRaw)
{
    Q_UNUSED(sequenceId);
    config_ = config;
    startPosRaw_ = startPosRaw;
    targetDeltaRaw_ = qRound(config.deltaDeg * config.rawPerDeg);
    coarseSamples_.clear();
    cachedPoints_.clear();
    nextIndex_ = 0;

    buildCoarseSamples();
    if (config_.mode == MotionMode::PVT) {
        buildPvtSamples();
    } else {
        buildFineSamples();
    }
}

QVector<TrajectoryPoint> QuinticPlanner::generateNextBatch(const FeedbackData &feedback, int batchSize)
{
    Q_UNUSED(feedback);

    QVector<TrajectoryPoint> batch;
    if (finished()) {
        return batch;
    }

    const qsizetype endIndex = std::min(cachedPoints_.size(),
                                        nextIndex_ + static_cast<qsizetype>(std::max(1, batchSize)));
    batch.reserve(endIndex - nextIndex_);
    for (; nextIndex_ < endIndex; ++nextIndex_) {
        batch.push_back(cachedPoints_.at(nextIndex_));
    }
    return batch;
}

bool QuinticPlanner::finished() const
{
    return nextIndex_ >= cachedPoints_.size();
}

QVector<TrajectoryPoint> QuinticPlanner::fullTrajectory() const
{
    return cachedPoints_;
}

void QuinticPlanner::buildCoarseSamples()
{
    const qint32 planningPeriodMs = std::max(1, config_.systemPlanningPeriodMs);
    const double durationS = std::max(config_.durationS, kMinDurationS);
    const int coarseIntervals = std::max(1, static_cast<int>(std::ceil(durationS * 1000.0 / planningPeriodMs)));
    const int coarseCount = coarseIntervals + 1;

    coarseSamples_.reserve(coarseCount);
    for (int i = 0; i < coarseCount; ++i) {
        const double timeS = std::min(durationS, i * planningPeriodMs / 1000.0);
        coarseSamples_.push_back(sampleCoarsePoint(timeS));
    }

    // 强制保证最后一个粗点精确落在总时长终点。
    if (!coarseSamples_.isEmpty()) {
        coarseSamples_.last() = sampleCoarsePoint(durationS);
    }
}

void QuinticPlanner::buildFineSamples()
{
    const double durationS = std::max(config_.durationS, kMinDurationS);
    const int fineCount = std::max(2, static_cast<int>(std::round(durationS / kFineStepS)) + 1);

    cachedPoints_.reserve(fineCount);
    for (int i = 0; i < fineCount; ++i) {
        const double fineTimeS = std::min(durationS, i * kFineStepS);
        const int coarseSegmentIndex = findCoarseSegment(fineTimeS);
        cachedPoints_.push_back(interpolateFinePoint(fineTimeS, coarseSegmentIndex));
    }

    // 重新覆盖最后一点，确保轨迹终点没有累计误差。
    if (!cachedPoints_.isEmpty()) {
        const int lastSegmentIndex = std::max(0, static_cast<int>(coarseSamples_.size()) - 2);
        cachedPoints_.last() = interpolateFinePoint(durationS, lastSegmentIndex);
    }
}

void QuinticPlanner::buildPvtSamples()
{
    cachedPoints_.reserve(coarseSamples_.size());

    // PVTS 由控制卡负责段间恢复，因此这里直接把“系统规划周期”粗点作为整表输出，
    // 避免再细化到 1ms 后无谓放大点数并触发表长上限。
    for (const CoarseSample &sample : coarseSamples_) {
        TrajectoryPoint point;
        point.timeS = sample.timeS;
        point.targetPosRaw = qRound(sample.positionRaw);
        point.targetVelRaw = qRound(sample.velocityRaw);
        point.targetAccRaw = 0;
        cachedPoints_.push_back(point);
    }
}

int QuinticPlanner::findCoarseSegment(double timeS) const
{
    if (coarseSamples_.size() <= 1) {
        return 0;
    }

    for (int i = 0; i < coarseSamples_.size() - 1; ++i) {
        if (timeS <= coarseSamples_.at(i + 1).timeS + 1e-12) {
            return i;
        }
    }
    return static_cast<int>(coarseSamples_.size()) - 2;
}

QuinticPlanner::CoarseSample QuinticPlanner::sampleCoarsePoint(double timeS) const
{
    CoarseSample sample;
    sample.timeS = timeS;
    sample.positionRaw = startPosRaw_ + targetDeltaRaw_ * samplePositionRatio(timeS);
    sample.velocityRaw = sampleVelocityDeg(timeS) * config_.rawPerDeg;
    return sample;
}

TrajectoryPoint QuinticPlanner::interpolateFinePoint(double timeS, int coarseSegmentIndex) const
{
    if (coarseSamples_.isEmpty()) {
        return {};
    }

    const int maxSegmentIndex = std::max(0, static_cast<int>(coarseSamples_.size()) - 2);
    const int safeIndex = std::clamp(coarseSegmentIndex, 0, maxSegmentIndex);
    const CoarseSample &p0 = coarseSamples_.at(safeIndex);
    const CoarseSample &p1 = coarseSamples_.at(std::min(safeIndex + 1,
                                                        static_cast<int>(coarseSamples_.size()) - 1));

    const double segmentDuration = std::max(p1.timeS - p0.timeS, kFineStepS);
    const double u = std::clamp((timeS - p0.timeS) / segmentDuration, 0.0, 1.0);

    // Cubic Hermite：使用相邻粗点的位置和速度，在段内恢复 1ms 细点。
    const double h00 = 2.0 * qPow(u, 3) - 3.0 * qPow(u, 2) + 1.0;
    const double h10 = qPow(u, 3) - 2.0 * qPow(u, 2) + u;
    const double h01 = -2.0 * qPow(u, 3) + 3.0 * qPow(u, 2);
    const double h11 = qPow(u, 3) - qPow(u, 2);

    const double posRaw = h00 * p0.positionRaw
                        + h10 * segmentDuration * p0.velocityRaw
                        + h01 * p1.positionRaw
                        + h11 * segmentDuration * p1.velocityRaw;

    const double dh00 = 6.0 * qPow(u, 2) - 6.0 * u;
    const double dh10 = 3.0 * qPow(u, 2) - 4.0 * u + 1.0;
    const double dh01 = -6.0 * qPow(u, 2) + 6.0 * u;
    const double dh11 = 3.0 * qPow(u, 2) - 2.0 * u;
    const double velRaw = (dh00 * p0.positionRaw
                         + dh10 * segmentDuration * p0.velocityRaw
                         + dh01 * p1.positionRaw
                         + dh11 * segmentDuration * p1.velocityRaw) / segmentDuration;

    TrajectoryPoint point;
    point.timeS = timeS;
    point.targetPosRaw = qRound(posRaw);
    point.targetVelRaw = qRound(velRaw);
    point.targetAccRaw = 0;
    return point;
}

double QuinticPlanner::samplePositionRatio(double timeS) const
{
    if (config_.durationS <= 0.0) {
        return 0.0;
    }

    const double tau = std::clamp(timeS / config_.durationS, 0.0, 1.0);
    return 10.0 * qPow(tau, 3) - 15.0 * qPow(tau, 4) + 6.0 * qPow(tau, 5);
}

double QuinticPlanner::sampleVelocityDeg(double timeS) const
{
    if (config_.durationS <= 0.0) {
        return 0.0;
    }

    const double tau = std::clamp(timeS / config_.durationS, 0.0, 1.0);
    const double dsDtau = 30.0 * qPow(tau, 2) - 60.0 * qPow(tau, 3) + 30.0 * qPow(tau, 4);
    return config_.deltaDeg * dsDtau / config_.durationS;
}
