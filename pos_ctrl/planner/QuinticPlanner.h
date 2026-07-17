#ifndef QUINTICPLANNER_H
#define QUINTICPLANNER_H

#include <QVector>

#include "planner/TrajectoryPlanner.h"

// 默认轨迹规划器。
// 先按系统规划周期生成五次多项式粗点，再统一细化成 1ms 下发点。
class QuinticPlanner : public TrajectoryPlanner
{
public:
    void reset(const MotionConfig &config, qint64 sequenceId, qint32 startPosRaw) override;
    QVector<TrajectoryPoint> generateNextBatch(const FeedbackData &feedback, int batchSize) override;
    bool finished() const override;
    QVector<TrajectoryPoint> fullTrajectory() const override;

private:
    struct CoarseSample
    {
        double timeS = 0.0;
        double positionRaw = 0.0;
        double velocityRaw = 0.0;
    };

    void buildCoarseSamples();
    void buildPvtSamples();
    void buildFineSamples();
    int findCoarseSegment(double timeS) const;
    CoarseSample sampleCoarsePoint(double timeS) const;
    TrajectoryPoint interpolateFinePoint(double timeS, int coarseSegmentIndex) const;
    double samplePositionRatio(double timeS) const;
    double sampleVelocityDeg(double timeS) const;

private:
    MotionConfig config_;
    qint32 startPosRaw_ = 0;
    qint32 targetDeltaRaw_ = 0;
    QVector<CoarseSample> coarseSamples_;
    QVector<TrajectoryPoint> cachedPoints_;
    qsizetype nextIndex_ = 0;
};

#endif // QUINTICPLANNER_H
