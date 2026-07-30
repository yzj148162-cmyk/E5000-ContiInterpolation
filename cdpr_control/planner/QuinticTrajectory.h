#ifndef QUINTICTRAJECTORY_H
#define QUINTICTRAJECTORY_H

#include <QVector>

#include "common/ContiTypes.h"

// 仅负责生成理论轨迹，不访问控制卡。
// 每次 nextPoint() 对应“上位机一个周期产生一个新点”的初版模型。
class QuinticTrajectory
{
public:
    void reset(const ContiTestConfig &config, double activeStartUnit, double holdStartUnit);
    bool hasNext() const;
    ContiPoint nextPoint();
    ContiPoint pointAt(double timeS) const;
    int totalPointCount() const;

private:
    double positionRatio(double timeS) const;

private:
    ContiTestConfig config_;
    double activeStartUnit_ = 0.0;
    double holdStartUnit_ = 0.0;
    int nextIndex_ = 0;
    int totalCount_ = 0;
};

#endif // QUINTICTRAJECTORY_H
