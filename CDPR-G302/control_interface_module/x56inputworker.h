#ifndef X56INPUTWORKER_H
#define X56INPUTWORKER_H

#include "x56directinputreader.h"

#include <QMutex>
#include <QObject>
#include <QString>
#include <QtGlobal>

#include <atomic>
#include <memory>
#include <vector>

class QTimer;

struct X56InputWorkerRequest
{
    quintptr windowHandle = 0;
    bool fixedBindingEnabled = false;
    QString boundInstanceId;
    quint64 sessionToken = 0;
    bool remoteRunning = false;
    bool applicationActive = false;
    quint64 safetyResetGeneration = 0;
};

struct X56InputWorkerSnapshot
{
    X56DirectInputSnapshot input;
    std::vector<X56DirectInputDeviceInfo> devices;
    quint64 sequence = 0;
    quint64 sessionToken = 0;
    quint64 appliedSafetyResetGeneration = 0;
    qint64 publishedAtUs = 0;
    qint64 latestPollDurationUs = 0;
    qint64 maximumPollDurationUs = 0;
    quint64 slowPollCount = 0;
    quint64 consecutiveReadFailureCount = 0;
    quint64 readFailureEventCount = 0;
};

// UI、心跳监督线程和X56采集线程只通过这个最新值邮箱交换数据。
// 所有入口都只做短时复制；DirectInput调用永远不在邮箱锁内执行。
class X56InputChannel
{
public:
    void configure(quintptr windowHandle,
                   bool fixedBindingEnabled,
                   const QString& boundInstanceId);
    void setSession(quint64 sessionToken, bool remoteRunning);
    void setRemoteRunning(quint64 sessionToken, bool remoteRunning);
    void setApplicationActive(bool active);
    quint64 requestSafetyReset();

    X56InputWorkerRequest request() const;
    void publish(const X56InputWorkerSnapshot& snapshot);
    X56InputWorkerSnapshot latestSnapshot() const;

private:
    mutable QMutex mutex;
    X56InputWorkerRequest currentRequest;
    X56InputWorkerSnapshot currentSnapshot;
};

// 该对象只在X56InputThread中运行。20 ms定时器只负责DirectInput采集，
// 不承担ControlWorker心跳，因此驱动阻塞不会卡住末端遥控监督心跳。
class X56InputWorker : public QObject
{
    Q_OBJECT

public:
    static constexpr int PollPeriodMs = 20;
    static constexpr qint64 SlowPollThresholdUs = 5000;

    explicit X56InputWorker(std::shared_ptr<X56InputChannel> channel,
                            QObject* parent = nullptr);
    ~X56InputWorker() override;

    // 可从其他线程调用；真正的资源释放仍由采集线程或析构函数完成。
    void requestStop();

public slots:
    void start();
    void stop();

private slots:
    void poll();

private:
    static qint64 monotonicNowUs();
    void synchronizeReader(const X56InputWorkerRequest& request);
    void resetSessionStatistics();

    std::shared_ptr<X56InputChannel> inputChannel;
    QTimer* timer = nullptr;
    X56DirectInputReader reader;
    std::atomic_bool stopRequested{false};
    quint64 readerSessionToken = 0;
    quint64 appliedSafetyResetGeneration = 0;
    quint64 snapshotSequence = 0;
    qint64 maximumPollDurationUs = 0;
    quint64 slowPollCount = 0;
    quint64 consecutiveReadFailureCount = 0;
    quint64 readFailureEventCount = 0;
};

#endif // X56INPUTWORKER_H
