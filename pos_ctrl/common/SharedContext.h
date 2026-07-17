#ifndef SHAREDCONTEXT_H
#define SHAREDCONTEXT_H

#include <QAtomicInteger>
#include <QMutex>
#include <QVector>
#include <QWaitCondition>

#include "common/FeedbackData.h"
#include "common/MotionRequest.h"
#include "common/ThreadSafeQueue.h"
#include "common/TrajectoryPoint.h"

// 三个核心线程共享的上下文。
// 这里只保存共享状态、队列和同步对象，避免耦合具体业务逻辑。
struct SharedContext
{
    ThreadSafeQueue<TrajectoryPoint> trajectoryQueue;

    QMutex feedbackMutex;
    FeedbackData feedback;

    QMutex requestMutex;
    MotionRequest motionRequest;

    // 整表预装型输出槽位。
    // 例如当前的 PVT(PVTS) 会在启动前一次性把完整精细轨迹准备到这里。
    QMutex plannedTrajectoryMutex;
    QVector<TrajectoryPoint> plannedTrajectory;

    // 实时控制量型输出槽位。
    // 当前代码尚未真正使用，但先把共享位置预留出来，方便后续扩展阻抗、力分配等在线控制量。
    QMutex latestCommandMutex;
    TrajectoryPoint latestCommand;

    QMutex plannerMutex;
    QWaitCondition plannerWaitCondition;
    bool plannerWakeup = false;

    QAtomicInteger<int> running { 0 };
    QAtomicInteger<int> plannedTrajectoryReady { 0 };
    QAtomicInteger<int> latestCommandReady { 0 };
    // 由规划线程维护的“当前请求是否已把内部缓存全部转移到共享队列”标志。
    // 硬件线程用它判断最后一批点消费完后是否可以自动收口。
    QAtomicInteger<int> plannerFinished { 0 };
    // 只统计本次运动中成功下发到控制卡的新轨迹点数。
    // 点击“开始轨迹”时清零，点击“停止轨迹”后冻结当前值。
    QAtomicInteger<qint64> sentPointCount { 0 };
    // 只统计规划线程实际向共享队列补入一批轨迹点的次数。
    // 这样该计数只反映“补队列”行为，不再混入 PVT 事件处理或空轮次唤醒。
    QAtomicInteger<qint64> queueFillCount { 0 };
};

#endif // SHAREDCONTEXT_H
