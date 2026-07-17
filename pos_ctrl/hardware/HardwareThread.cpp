#include "hardware/HardwareThread.h"

#include <QMutexLocker>

#include <chrono>
#include <thread>

namespace {
constexpr auto kWarmupDelay = std::chrono::milliseconds(20);
constexpr auto kPrimePollInterval = std::chrono::milliseconds(1);
constexpr auto kPrimeTimeout = std::chrono::milliseconds(500);
}

HardwareThread::HardwareThread(SharedContext &sharedContext, const SystemConfig &systemConfig, QObject *parent)
    : QThread(parent)
    , sharedContext_(sharedContext)
    , systemConfig_(systemConfig)
{
}

PlanningOutputType HardwareThread::currentOutputType() const
{
    return planningOutputTypeForMode(activeConfig_.mode);
}

PlannerWakePolicy HardwareThread::currentWakePolicy() const
{
    return plannerWakePolicyForOutputType(currentOutputType());
}

bool HardwareThread::shouldWakePlannerPeriodically() const
{
    // 周期唤醒只发生在“硬件线程已经进入运动执行态”时。
    // 这样 PVT(PVTS) 启动后的监测阶段不会再无意义地每 1ms 唤醒规划线程。
    // 这里判断的是“当前任务采用什么唤醒策略”，而不是直接把周期唤醒绑定到某个模式名字上。
    if (!boardInitialized_ || !motionActive_) {
        return false;
    }

    switch (currentWakePolicy()) {
    case PlannerWakePolicy::EventDriven:
        return false;
    case PlannerWakePolicy::Periodic:
    case PlannerWakePolicy::PeriodicWithBuffer:
        return true;
    }

    return false;
}

void HardwareThread::wakePlannerForEvent()
{
    // 把对规划线程的唤醒入口收口到一个函数中，
    // 避免“事件唤醒”和“周期唤醒”在多个地方分散写条件判断。
    QMutexLocker locker(&sharedContext_.plannerMutex);
    sharedContext_.plannerWakeup = true;
    sharedContext_.plannerWaitCondition.wakeAll();
}

void HardwareThread::enqueueCommand(const HardwareCommand &command)
{
    QMutexLocker locker(&commandMutex_);
    commands_.enqueue(command);
}

void HardwareThread::run()
{
    emit logMessage(QStringLiteral("硬件线程已启动，周期=%1us。").arg(systemConfig_.hardwareCycleUs));

    using Clock = std::chrono::steady_clock;
    auto nextTick = Clock::now();

    while (sharedContext_.running.loadAcquire()) {
        processPendingCommands();

        if (boardInitialized_) {
            FeedbackData feedback;
            {
                QMutexLocker locker(&sharedContext_.feedbackMutex);
                feedback = sharedContext_.feedback;
            }

            if (motionActive_) {
                if (activeConfig_.mode == MotionMode::CSP) {
                    TrajectoryPoint point;
                    bool poppedNewPoint = false;
                    if (sharedContext_.trajectoryQueue.tryPop(point)) {
                        lastPoint_ = point;
                        poppedNewPoint = true;
                    }

                    if (!poppedNewPoint
                        && sharedContext_.plannerFinished.loadAcquire()
                        && sharedContext_.trajectoryQueue.empty()) {
                        motionActive_ = false;
                        {
                            QMutexLocker locker(&sharedContext_.requestMutex);
                            sharedContext_.motionRequest.active = false;
                            sharedContext_.motionRequest.restartRequested = false;
                        }
                        sharedContext_.plannerFinished.storeRelease(0);
                        feedback.motionRunning = false;
                        emit logMessage(QStringLiteral("CSP 轨迹点已全部下发完成，硬件线程自动结束本次运动。"));
                    }

                    QString errorMessage;
                    if (motionActive_) {
                        const bool ioOk =
                            ethercat_.writeCspTargetPositionRaw(activeConfig_, lastPoint_.targetPosRaw, errorMessage);
                        if (!ioOk) {
                            emit logMessage(QStringLiteral("硬件线程发送失败: %1").arg(errorMessage));
                            feedback.fault = true;
                            motionActive_ = false;
                        } else {
                            feedback.targetPosRaw = lastPoint_.targetPosRaw;
                            feedback.motionTimeS =
                                sharedContext_.sentPointCount.loadAcquire() * systemConfig_.hardwareCycleUs / 1000000.0;
                            feedback.motionRunning = true;
                            if (poppedNewPoint) {
                                sharedContext_.sentPointCount.fetchAndAddRelaxed(1);
                                feedback.motionTimeS =
                                    sharedContext_.sentPointCount.loadAcquire() * systemConfig_.hardwareCycleUs / 1000000.0;
                            }
                        }
                    }
                } else {
                    QString errorMessage;
                    quint32 runIndex = 0;
                    if (ethercat_.readPvtRunIndex(activeConfig_, runIndex, errorMessage)
                        && !activePvtTrajectory_.isEmpty()) {
                        const qsizetype safeIndex = std::min<qsizetype>(runIndex, activePvtTrajectory_.size() - 1);
                        feedback.targetPosRaw = activePvtTrajectory_.at(safeIndex).targetPosRaw;
                        feedback.motionTimeS = activePvtTrajectory_.at(safeIndex).timeS;
                    }

                    bool done = false;
                    if (!ethercat_.isAxisMotionDone(activeConfig_, done, errorMessage)) {
                        emit logMessage(QStringLiteral("读取 PVT 运行状态失败: %1").arg(errorMessage));
                        feedback.fault = true;
                        motionActive_ = false;
                    } else if (done) {
                        motionActive_ = false;
                        {
                            QMutexLocker locker(&sharedContext_.requestMutex);
                            sharedContext_.motionRequest.active = false;
                            sharedContext_.motionRequest.restartRequested = false;
                        }
                        sharedContext_.plannerFinished.storeRelease(0);
                        // PVTS 收尾周期里，控制卡返回的 runIndex 可能已经回到起点或不再代表“最后一个有效轨迹点”。
                        // 如果此时仍沿用前面基于 runIndex 的目标点，UI 误差曲线会把“终点时间”连到“起点目标误差”，
                        // 看起来像最后被硬拉出一条直线。因此在确认 done 后，显式把目标点和时间轴钉到轨迹终点。
                        if (!activePvtTrajectory_.isEmpty()) {
                            feedback.targetPosRaw = activePvtTrajectory_.last().targetPosRaw;
                            feedback.motionTimeS = activePvtTrajectory_.last().timeS;
                        } else {
                            feedback.motionTimeS = activeConfig_.durationS;
                        }
                        feedback.motionRunning = false;
                        emit logMessage(QStringLiteral("PVTS 轨迹已执行完成，硬件线程自动结束本次运动。"));
                    } else {
                        feedback.motionRunning = true;
                    }
                }
            }

            QString errorMessage;
            if (!ethercat_.readFeedback(activeConfig_, feedback, errorMessage)) {
                emit logMessage(QStringLiteral("硬件线程读取反馈失败: %1").arg(errorMessage));
                feedback.fault = true;
            }
            feedback.motionRunning = motionActive_;
            feedback.boardInitialized = boardInitialized_;
            publishFeedback(feedback);

            if (shouldWakePlannerPeriodically()) {
                wakePlannerForEvent();
            }
        }

        nextTick += std::chrono::microseconds(systemConfig_.hardwareCycleUs);
        std::this_thread::sleep_until(nextTick);
    }

    emit logMessage(QStringLiteral("硬件线程已退出。"));
}

void HardwareThread::processPendingCommands()
{
    QQueue<HardwareCommand> localQueue;
    {
        QMutexLocker locker(&commandMutex_);
        localQueue.swap(commands_);
    }

    while (!localQueue.isEmpty()) {
        const HardwareCommand command = localQueue.dequeue();

        switch (command.type) {
        case HardwareCommand::Type::InitializeBoard:
            handleInitializeBoard();
            break;
        case HardwareCommand::Type::CloseBoard:
            handleCloseBoard();
            break;
        case HardwareCommand::Type::EnableAxis:
            handleEnableAxis(command.config);
            break;
        case HardwareCommand::Type::DisableAxis:
            handleDisableAxis(command.config);
            break;
        case HardwareCommand::Type::StartMotion:
            handleStartMotion(command.config);
            break;
        case HardwareCommand::Type::StopMotion:
            handleStopMotion();
            break;
        }
    }
}

void HardwareThread::handleInitializeBoard()
{
    QString errorMessage;
    if (!ethercat_.initializeBoard(errorMessage)) {
        emit logMessage(QStringLiteral("初始化控制卡失败: %1").arg(errorMessage));
        return;
    }

    boardInitialized_ = true;
    {
        QMutexLocker locker(&sharedContext_.feedbackMutex);
        sharedContext_.feedback.boardInitialized = true;
        sharedContext_.feedback.fault = false;
    }
    emit boardStateChanged(true);
    emit logMessage(QStringLiteral("控制卡初始化成功。"));
}

void HardwareThread::handleCloseBoard()
{
    handleStopMotion();

    QString errorMessage;
    if (!ethercat_.closeBoard(errorMessage)) {
        emit logMessage(QStringLiteral("关闭控制卡失败: %1").arg(errorMessage));
        return;
    }

    boardInitialized_ = false;
    {
        QMutexLocker locker(&sharedContext_.feedbackMutex);
        sharedContext_.feedback.boardInitialized = false;
        sharedContext_.feedback.motionRunning = false;
    }
    emit boardStateChanged(false);
    emit logMessage(QStringLiteral("控制卡已关闭。"));
}

void HardwareThread::handleEnableAxis(const MotionConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("轴使能前请先初始化控制卡。"));
        return;
    }

    QString errorMessage;
    if (!ethercat_.enableAxis(config, errorMessage)) {
        emit logMessage(QStringLiteral("轴使能失败: %1").arg(errorMessage));
        return;
    }

    emit logMessage(QStringLiteral("轴 %1 已通过雷赛库函数使能。").arg(config.axis));
}

void HardwareThread::handleDisableAxis(const MotionConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("轴失能前请先初始化控制卡。"));
        return;
    }

    if (motionActive_) {
        handleStopMotion();
    }

    QString errorMessage;
    if (!ethercat_.disableAxis(config, errorMessage)) {
        emit logMessage(QStringLiteral("轴失能失败: %1").arg(errorMessage));
        return;
    }

    emit logMessage(QStringLiteral("轴 %1 已通过雷赛库函数失能。").arg(config.axis));
}

void HardwareThread::handleStartMotion(const MotionConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("启动运动前请先初始化控制卡。"));
        return;
    }

    activeConfig_ = config;
    motionActive_ = false;
    lastPoint_ = {};
    activePvtTrajectory_.clear();
    if (config.mode == MotionMode::CSP) {
        // 新的队列供货型任务启动前先清掉存量旧点，
        // 避免硬件线程把上一轮残留误认为“本次首批点已经到位”。
        sharedContext_.trajectoryQueue.clear();
    }
    {
        QMutexLocker locker(&sharedContext_.plannedTrajectoryMutex);
        sharedContext_.plannedTrajectory.clear();
    }
    sharedContext_.plannedTrajectoryReady.storeRelease(0);
    sharedContext_.latestCommandReady.storeRelease(0);
    sharedContext_.plannerFinished.storeRelease(0);
    sharedContext_.sentPointCount.storeRelease(0);
    sharedContext_.queueFillCount.storeRelease(0);
    {
        QMutexLocker locker(&sharedContext_.feedbackMutex);
        sharedContext_.feedback.motionTimeS = 0.0;
    }

    QString errorMessage;
    if (config.mode == MotionMode::CSP) {
        // CSP 仍然先在硬件线程中配置驱动模式参数，但使能动作改为雷赛库函数，
        // 不再手动下发 CiA402 控制字序列。
        if (!ethercat_.configureCspMode(config, errorMessage)) {
            emit logMessage(QStringLiteral("CSP 配置失败: %1").arg(errorMessage));
            return;
        }
    }

    if (!ethercat_.enableAxis(config, errorMessage)) {
        emit logMessage(QStringLiteral("开始运动前轴使能失败: %1").arg(errorMessage));
        return;
    }

    if (config.mode == MotionMode::CSP) {
        emit logMessage(QStringLiteral("CSP 模式参数已配置，并已使用雷赛库函数完成轴使能。"));
    } else {
        emit logMessage(QStringLiteral("PVT(PVTS) 模式已使用雷赛库函数完成轴使能。"));
    }

    // 启动阶段属于事件唤醒，不管当前任务最终是否需要周期唤醒，都从这里显式触发规划线程准备数据。
    const auto primeDeadline = std::chrono::steady_clock::now() + kPrimeTimeout;
    while (std::chrono::steady_clock::now() < primeDeadline) {
        wakePlannerForEvent();

        if (config.mode == MotionMode::CSP) {
            if (!sharedContext_.trajectoryQueue.empty()) {
                break;
            }
        } else if (sharedContext_.plannedTrajectoryReady.loadAcquire()) {
            break;
        }
        std::this_thread::sleep_for(kPrimePollInterval);
    }

    if (config.mode == MotionMode::CSP && sharedContext_.trajectoryQueue.empty()) {
        {
            QMutexLocker locker(&sharedContext_.requestMutex);
            sharedContext_.motionRequest.active = false;
            sharedContext_.motionRequest.restartRequested = false;
        }
        sharedContext_.plannedTrajectoryReady.storeRelease(0);
        sharedContext_.plannerFinished.storeRelease(0);
        emit logMessage(QStringLiteral("开始运动失败：等待首批轨迹点入队超时。"));
        return;
    }

    if (config.mode == MotionMode::PVT && !sharedContext_.plannedTrajectoryReady.loadAcquire()) {
        {
            QMutexLocker locker(&sharedContext_.requestMutex);
            sharedContext_.motionRequest.active = false;
            sharedContext_.motionRequest.restartRequested = false;
        }
        emit logMessage(QStringLiteral("开始运动失败：等待 PVTS 轨迹整表就绪超时。"));
        return;
    }

    if (config.mode == MotionMode::PVT) {
        {
            QMutexLocker locker(&sharedContext_.plannedTrajectoryMutex);
            activePvtTrajectory_ = sharedContext_.plannedTrajectory;
        }
        if (activePvtTrajectory_.size() < 2) {
            emit logMessage(QStringLiteral("开始运动失败：PVTS 轨迹点数不足。"));
            return;
        }
        if (!ethercat_.startPvtsMotion(config, activePvtTrajectory_, errorMessage)) {
            emit logMessage(QStringLiteral("PVTS 整表下发失败: %1").arg(errorMessage));
            return;
        }
        sharedContext_.sentPointCount.storeRelease(activePvtTrajectory_.size());
    }

    std::this_thread::sleep_for(kWarmupDelay);
    motionActive_ = true;
    emit logMessage(QStringLiteral("%1 模式运动链已启动。").arg(motionModeToString(config.mode)));
}

void HardwareThread::handleStopMotion()
{
    if (!boardInitialized_) {
        motionActive_ = false;
        return;
    }

    QString errorMessage;
    if (!ethercat_.stopAxis(activeConfig_, errorMessage)) {
        emit logMessage(QStringLiteral("停止轴失败: %1").arg(errorMessage));
    }

    motionActive_ = false;
    {
        QMutexLocker locker(&sharedContext_.requestMutex);
        sharedContext_.motionRequest.active = false;
        sharedContext_.motionRequest.restartRequested = false;
    }
    sharedContext_.plannerFinished.storeRelease(0);
    sharedContext_.plannedTrajectoryReady.storeRelease(0);
    sharedContext_.latestCommandReady.storeRelease(0);
    {
        QMutexLocker locker(&sharedContext_.plannedTrajectoryMutex);
        sharedContext_.plannedTrajectory.clear();
    }
    {
        QMutexLocker locker(&sharedContext_.latestCommandMutex);
        sharedContext_.latestCommand = {};
    }
    activePvtTrajectory_.clear();
    sharedContext_.trajectoryQueue.clear();
    {
        QMutexLocker locker(&sharedContext_.feedbackMutex);
        sharedContext_.feedback.motionRunning = false;
        sharedContext_.feedback.motionTimeS = 0.0;
    }
    emit logMessage(QStringLiteral("硬件线程已停止当前运动。"));
}

void HardwareThread::publishFeedback(const FeedbackData &feedback)
{
    QMutexLocker locker(&sharedContext_.feedbackMutex);
    sharedContext_.feedback = feedback;
}
