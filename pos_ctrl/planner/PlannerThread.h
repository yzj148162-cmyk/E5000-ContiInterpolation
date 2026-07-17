#ifndef PLANNERTHREAD_H
#define PLANNERTHREAD_H

#include <QScopedPointer>
#include <QThread>

#include "common/SharedContext.h"
#include "config/SystemConfig.h"

class TrajectoryPlanner;

// 规划线程。
// 该线程不直接接触硬件，只负责根据最新反馈补充未来若干个轨迹点。
class PlannerThread : public QThread
{
    Q_OBJECT

public:
    PlannerThread(SharedContext &sharedContext, const SystemConfig &systemConfig, QObject *parent = nullptr);
    ~PlannerThread() override = default;

protected:
    void run() override;

signals:
    void logMessage(const QString &message);

private:
    void clearPlannerOutputs();
    void prepareOutputsForNewRequest(const MotionRequest &request, qint32 startPosRaw);
    void handlePrecomputedTableOutput();
    void handleQueuedSequenceOutput(const FeedbackData &feedback);
    void handleRealtimeCommandOutput(const FeedbackData &feedback);

private:
    SharedContext &sharedContext_;
    SystemConfig systemConfig_;
    QScopedPointer<TrajectoryPlanner> planner_;
    qint64 lastSequenceId_ = -1;
};

#endif // PLANNERTHREAD_H
