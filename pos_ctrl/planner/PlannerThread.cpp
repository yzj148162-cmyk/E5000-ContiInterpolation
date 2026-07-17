#include "planner/PlannerThread.h"

#include <QMutexLocker>

#include "planner/QuinticPlanner.h"

PlannerThread::PlannerThread(SharedContext &sharedContext, const SystemConfig &systemConfig, QObject *parent)
    : QThread(parent)
    , sharedContext_(sharedContext)
    , systemConfig_(systemConfig)
    , planner_(new QuinticPlanner)
{
}

void PlannerThread::clearPlannerOutputs()
{
    {
        QMutexLocker locker(&sharedContext_.plannedTrajectoryMutex);
        sharedContext_.plannedTrajectory.clear();
    }
    {
        QMutexLocker locker(&sharedContext_.latestCommandMutex);
        sharedContext_.latestCommand = {};
    }

    sharedContext_.plannedTrajectoryReady.storeRelease(0);
    sharedContext_.latestCommandReady.storeRelease(0);
    sharedContext_.plannerFinished.storeRelease(0);
}

void PlannerThread::prepareOutputsForNewRequest(const MotionRequest &request, qint32 startPosRaw)
{
    planner_->reset(request.config, request.sequenceId, startPosRaw);
    lastSequenceId_ = request.sequenceId;
    sharedContext_.plannerFinished.storeRelease(0);

    {
        QMutexLocker locker(&sharedContext_.plannedTrajectoryMutex);
        sharedContext_.plannedTrajectory = planner_->fullTrajectory();
    }
    sharedContext_.plannedTrajectoryReady.storeRelease(1);

    {
        QMutexLocker locker(&sharedContext_.requestMutex);
        sharedContext_.motionRequest.restartRequested = false;
    }

    sharedContext_.trajectoryQueue.clear();
    emit logMessage(QStringLiteral("规划线程已重建轨迹，sequence=%1。").arg(request.sequenceId));
}

void PlannerThread::handlePrecomputedTableOutput()
{
    // 整表预装型任务在启动前已经准备好完整精细轨迹，运行阶段不需要持续补队列。
    sharedContext_.plannerFinished.storeRelease(1);
}

void PlannerThread::handleQueuedSequenceOutput(const FeedbackData &feedback)
{
    // 队列供货型任务只有在低于低水位时才真正补货。
    // 当前代码中 CSP 采用这一类输出，但这类机制本身并不与 CSP 绑定；
    // 后续任何“慢周期产出未来快周期序列”的任务都可以复用这里的处理方式。
    if (sharedContext_.trajectoryQueue.size() >= systemConfig_.plannerQueueLowWatermark) {
        return;
    }

    const QVector<TrajectoryPoint> batch =
        planner_->generateNextBatch(feedback, systemConfig_.plannerBatchSize);
    if (batch.isEmpty()) {
        if (planner_->finished()) {
            sharedContext_.plannerFinished.storeRelease(1);
        }
        return;
    }

    MotionRequest latestRequest;
    {
        QMutexLocker locker(&sharedContext_.requestMutex);
        latestRequest = sharedContext_.motionRequest;
    }

    // 新请求已经到来时，丢弃这批旧任务产出的点，避免重新污染共享队列。
    // 这样队列中的首批点才能稳定代表“当前请求”的数据，而不是上一轮的尾波。
    if (!latestRequest.active
        || latestRequest.restartRequested
        || latestRequest.sequenceId != lastSequenceId_) {
        return;
    }

    for (const TrajectoryPoint &point : batch) {
        sharedContext_.trajectoryQueue.push(point);
    }

    if (planner_->finished()) {
        sharedContext_.plannerFinished.storeRelease(1);
    }

    sharedContext_.queueFillCount.fetchAndAddRelaxed(1);
}

void PlannerThread::handleRealtimeCommandOutput(const FeedbackData &feedback)
{
    // 预留给后续阻抗、力分配等“实时控制量型”扩展。
    // 当前实现先按“一次生成一个下一时刻命令”占位，等后续模式真正接入后再补具体控制律。
    const QVector<TrajectoryPoint> batch = planner_->generateNextBatch(feedback, 1);
    if (batch.isEmpty()) {
        sharedContext_.latestCommandReady.storeRelease(0);
        if (planner_->finished()) {
            sharedContext_.plannerFinished.storeRelease(1);
        }
        return;
    }

    {
        QMutexLocker locker(&sharedContext_.latestCommandMutex);
        sharedContext_.latestCommand = batch.last();
    }
    sharedContext_.latestCommandReady.storeRelease(1);

    if (planner_->finished()) {
        sharedContext_.plannerFinished.storeRelease(1);
    }
}

void PlannerThread::run()
{
    emit logMessage(QStringLiteral("规划线程已启动，等待事件或周期唤醒。"));

    while (sharedContext_.running.loadAcquire()) {
        {
            QMutexLocker locker(&sharedContext_.plannerMutex);
            while (sharedContext_.running.loadAcquire() && !sharedContext_.plannerWakeup) {
                sharedContext_.plannerWaitCondition.wait(&sharedContext_.plannerMutex);
            }
            if (!sharedContext_.running.loadAcquire()) {
                break;
            }
            sharedContext_.plannerWakeup = false;
        }

        MotionRequest request;
        {
            QMutexLocker locker(&sharedContext_.requestMutex);
            request = sharedContext_.motionRequest;
        }
        if (!request.active) {
            clearPlannerOutputs();
            continue;
        }

        FeedbackData feedback;
        {
            QMutexLocker locker(&sharedContext_.feedbackMutex);
            feedback = sharedContext_.feedback;
        }

        if (request.restartRequested || request.sequenceId != lastSequenceId_) {
            prepareOutputsForNewRequest(request, feedback.actualPosRaw);
        }

        switch (planningOutputTypeForMode(request.config.mode)) {
        case PlanningOutputType::PrecomputedTable:
            handlePrecomputedTableOutput();
            break;
        case PlanningOutputType::QueuedSequence:
            handleQueuedSequenceOutput(feedback);
            break;
        case PlanningOutputType::RealtimeCommand:
            handleRealtimeCommandOutput(feedback);
            break;
        }
    }

    emit logMessage(QStringLiteral("规划线程已退出。"));
}
