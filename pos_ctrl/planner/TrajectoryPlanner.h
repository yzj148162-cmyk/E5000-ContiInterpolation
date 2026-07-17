#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <QVector>

#include "common/FeedbackData.h"
#include "common/TrajectoryPoint.h"
#include "config/MotionConfig.h"

// 轨迹规划接口。
// 后续如果增加补偿、插补或其他规划器，都从这里继续派生。
class TrajectoryPlanner
{
public:
    virtual ~TrajectoryPlanner() = default;

    virtual void reset(const MotionConfig &config, qint64 sequenceId, qint32 startPosRaw) = 0;
    virtual QVector<TrajectoryPoint> generateNextBatch(const FeedbackData &feedback, int batchSize) = 0;
    virtual bool finished() const = 0;
    virtual QVector<TrajectoryPoint> fullTrajectory() const = 0;
};

#endif // TRAJECTORYPLANNER_H
