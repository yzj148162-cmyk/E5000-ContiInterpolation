#ifndef UIREFRESHCONTROLLER_H
#define UIREFRESHCONTROLLER_H

#include <QMetaType>
#include <QObject>
#include <QTimer>

#include "common/FeedbackData.h"
#include "common/SharedContext.h"
#include "config/SystemConfig.h"

// UI 粗粒度显示快照。
// UI 线程按较慢周期读取共享数据，用于显示，不直接参与 1ms 实时控制。
struct UiSnapshot
{
    FeedbackData feedback;
    qint64 sentPointCount = 0;
    qint64 queueFillCount = 0;
    qint64 queueDepth = 0;
};
Q_DECLARE_METATYPE(UiSnapshot)

class UiRefreshController : public QObject
{
    Q_OBJECT

public:
    UiRefreshController(SharedContext &sharedContext, const SystemConfig &systemConfig, QObject *parent = nullptr);

    void start();
    void stop();
    void setRefreshIntervalMs(int intervalMs);

signals:
    void snapshotReady(const UiSnapshot &snapshot);

private slots:
    void onTimeout();

private:
    SharedContext &sharedContext_;
    SystemConfig systemConfig_;
    QTimer timer_;
};

#endif // UIREFRESHCONTROLLER_H
