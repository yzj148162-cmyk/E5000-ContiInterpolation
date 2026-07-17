#include "ui/UiRefreshController.h"

#include <QMutexLocker>

UiRefreshController::UiRefreshController(SharedContext &sharedContext,
                                         const SystemConfig &systemConfig,
                                         QObject *parent)
    : QObject(parent)
    , sharedContext_(sharedContext)
    , systemConfig_(systemConfig)
{
    timer_.setInterval(systemConfig_.uiRefreshMs);
    connect(&timer_, &QTimer::timeout, this, &UiRefreshController::onTimeout);
}

void UiRefreshController::start()
{
    timer_.start();
}

void UiRefreshController::stop()
{
    timer_.stop();
}

void UiRefreshController::setRefreshIntervalMs(int intervalMs)
{
    systemConfig_.uiRefreshMs = intervalMs;
    timer_.setInterval(intervalMs);
}

void UiRefreshController::onTimeout()
{
    UiSnapshot snapshot;
    {
        QMutexLocker locker(&sharedContext_.feedbackMutex);
        snapshot.feedback = sharedContext_.feedback;
    }
    snapshot.sentPointCount = sharedContext_.sentPointCount.loadAcquire();
    snapshot.queueFillCount = sharedContext_.queueFillCount.loadAcquire();
    snapshot.queueDepth = sharedContext_.trajectoryQueue.size();
    emit snapshotReady(snapshot);
}
