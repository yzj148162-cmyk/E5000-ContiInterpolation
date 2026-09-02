#include "x56inputworker.h"

#include <QMutexLocker>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <chrono>
#include <utility>

void X56InputChannel::configure(quintptr windowHandle,
                                bool fixedBindingEnabled,
                                const QString& boundInstanceId)
{
    QMutexLocker locker(&mutex);
    currentRequest.windowHandle = windowHandle;
    currentRequest.fixedBindingEnabled = fixedBindingEnabled;
    currentRequest.boundInstanceId = boundInstanceId.trimmed().toUpper();
}

void X56InputChannel::setSession(quint64 sessionToken, bool remoteRunning)
{
    QMutexLocker locker(&mutex);
    currentRequest.sessionToken = sessionToken;
    currentRequest.remoteRunning = sessionToken != 0 && remoteRunning;
}

void X56InputChannel::setRemoteRunning(quint64 sessionToken,
                                       bool remoteRunning)
{
    QMutexLocker locker(&mutex);
    if(sessionToken == 0 || sessionToken != currentRequest.sessionToken){
        return;
    }
    currentRequest.remoteRunning = remoteRunning;
}

void X56InputChannel::setApplicationActive(bool active)
{
    QMutexLocker locker(&mutex);
    currentRequest.applicationActive = active;
}

quint64 X56InputChannel::requestSafetyReset()
{
    QMutexLocker locker(&mutex);
    return ++currentRequest.safetyResetGeneration;
}

X56InputWorkerRequest X56InputChannel::request() const
{
    QMutexLocker locker(&mutex);
    return currentRequest;
}

void X56InputChannel::publish(const X56InputWorkerSnapshot& snapshot)
{
    QMutexLocker locker(&mutex);
    currentSnapshot = snapshot;
}

X56InputWorkerSnapshot X56InputChannel::latestSnapshot() const
{
    QMutexLocker locker(&mutex);
    return currentSnapshot;
}

X56InputWorker::X56InputWorker(
        std::shared_ptr<X56InputChannel> channel,
        QObject* parent)
    : QObject(parent),
      inputChannel(std::move(channel))
{
}

X56InputWorker::~X56InputWorker()
{
    reader.shutdown();
}

qint64 X56InputWorker::monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

void X56InputWorker::requestStop()
{
    stopRequested.store(true, std::memory_order_release);
}

void X56InputWorker::start()
{
    stopRequested.store(false, std::memory_order_release);
    if(!timer){
        timer = new QTimer(this);
        timer->setTimerType(Qt::PreciseTimer);
        timer->setInterval(PollPeriodMs);
        connect(timer, &QTimer::timeout,
                this, &X56InputWorker::poll);
    }
    timer->start();
    QMetaObject::invokeMethod(this, &X56InputWorker::poll,
                              Qt::QueuedConnection);
}

void X56InputWorker::stop()
{
    requestStop();
    if(timer){
        timer->stop();
    }
    reader.endSession();
    readerSessionToken = 0;
    reader.shutdown();
}

void X56InputWorker::resetSessionStatistics()
{
    maximumPollDurationUs = 0;
    slowPollCount = 0;
    consecutiveReadFailureCount = 0;
    readFailureEventCount = 0;
}

void X56InputWorker::synchronizeReader(
        const X56InputWorkerRequest& request)
{
    reader.configure(request.windowHandle,
                     request.fixedBindingEnabled,
                     request.boundInstanceId);
    if(request.sessionToken != readerSessionToken){
        if(request.sessionToken != 0){
            reader.beginSession(request.sessionToken);
        }
        else{
            reader.endSession();
        }
        readerSessionToken = request.sessionToken;
        appliedSafetyResetGeneration = request.safetyResetGeneration;
        resetSessionStatistics();
        return;
    }
    if(request.safetyResetGeneration != appliedSafetyResetGeneration){
        reader.endSession();
        if(request.sessionToken != 0){
            reader.beginSession(request.sessionToken);
        }
        appliedSafetyResetGeneration = request.safetyResetGeneration;
    }
}

void X56InputWorker::poll()
{
    if(stopRequested.load(std::memory_order_acquire) ||
            QThread::currentThread()->isInterruptionRequested() ||
            !inputChannel){
        return;
    }

    const X56InputWorkerRequest request = inputChannel->request();
    const qint64 pollStartedUs = monotonicNowUs();
    synchronizeReader(request);
    X56DirectInputSnapshot input =
            reader.poll(request.remoteRunning,
                        request.applicationActive);
    const std::vector<X56DirectInputDeviceInfo> devices = reader.devices();
    const qint64 publishedAtUs = monotonicNowUs();
    const qint64 pollDurationUs = std::max<qint64>(
                0, publishedAtUs - pollStartedUs);

    maximumPollDurationUs = std::max(maximumPollDurationUs,
                                     pollDurationUs);
    if(pollDurationUs > SlowPollThresholdUs){
        ++slowPollCount;
    }
    if(input.readAttempted){
        if(input.readFailed){
            ++consecutiveReadFailureCount;
            ++readFailureEventCount;
        }
        else if(input.sampleFresh){
            consecutiveReadFailureCount = 0;
        }
    }

    X56InputWorkerSnapshot snapshot;
    snapshot.input = std::move(input);
    snapshot.devices = devices;
    snapshot.sequence = ++snapshotSequence;
    snapshot.sessionToken = request.sessionToken;
    snapshot.appliedSafetyResetGeneration =
            appliedSafetyResetGeneration;
    snapshot.publishedAtUs = publishedAtUs;
    snapshot.latestPollDurationUs = pollDurationUs;
    snapshot.maximumPollDurationUs = maximumPollDurationUs;
    snapshot.slowPollCount = slowPollCount;
    snapshot.consecutiveReadFailureCount =
            consecutiveReadFailureCount;
    snapshot.readFailureEventCount = readFailureEventCount;
    inputChannel->publish(snapshot);
}
