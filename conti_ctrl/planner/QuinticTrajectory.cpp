#include "planner/QuinticTrajectory.h"

#include <algorithm>
#include <cmath>

void QuinticTrajectory::reset(const ContiTestConfig &config,
                              double activeStartUnit,
                              double holdStartUnit)
{
    config_ = config;
    activeStartUnit_ = activeStartUnit;
    holdStartUnit_ = holdStartUnit;
    nextIndex_ = 0;

    const double periodS = std::max(1, config_.producerPeriodMs) / 1000.0;
    totalCount_ = std::max(2, static_cast<int>(std::ceil(config_.durationS / periodS)) + 1);
}

bool QuinticTrajectory::hasNext() const
{
    return nextIndex_ < totalCount_;
}

ContiPoint QuinticTrajectory::nextPoint()
{
    if (!hasNext()) {
        return {};
    }

    const double periodS = std::max(1, config_.producerPeriodMs) / 1000.0;
    const ContiPoint point = pointAt(std::min(config_.durationS, nextIndex_ * periodS));
    ++nextIndex_;
    return point;
}

ContiPoint QuinticTrajectory::pointAt(double timeS) const
{
    ContiPoint point;
    point.timeS = std::clamp(timeS, 0.0, config_.durationS);
    const double ratio = positionRatio(point.timeS);
    point.targetUnit[0] = activeStartUnit_ + config_.activeDeltaUnit * ratio;
    point.targetUnit[1] = holdStartUnit_;
    if (config_.stage == TestStage::DualAxis) {
        point.targetUnit[1] += config_.holdDeltaUnit * ratio;
    }
    return point;
}

int QuinticTrajectory::totalPointCount() const
{
    return totalCount_;
}

double QuinticTrajectory::positionRatio(double timeS) const
{
    if (config_.durationS <= 0.0) {
        return 1.0;
    }

    const double tau = std::clamp(timeS / config_.durationS, 0.0, 1.0);
    // 与 pos_ctrl 相同的零速度、零加速度边界五次多项式。
    return 10.0 * std::pow(tau, 3)
         - 15.0 * std::pow(tau, 4)
         + 6.0 * std::pow(tau, 5);
}
