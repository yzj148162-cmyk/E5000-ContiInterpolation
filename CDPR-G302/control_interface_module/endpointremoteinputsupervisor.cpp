#include "endpointremoteinputsupervisor.h"

#include "controlworker.h"

#include <QThread>
#include <QTimer>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {
constexpr int kEndpointRemoteSupervisorPeriodMs = 50;
constexpr qint64 kEndpointRemoteSupervisorLateWarnUs = 100000;
constexpr qint64 kX56PollDiagnosticThrottleUs = 1000000;
constexpr qint64 kX56SnapshotTimeoutUs = 100000;
}

EndpointRemoteInputSupervisor::EndpointRemoteInputSupervisor(
        ControlWorker* worker,
        QObject* parent)
    : QObject(parent),
      controlWorker(worker),
      x56InputChannel(std::make_shared<X56InputChannel>())
{
}

EndpointRemoteInputSupervisor::~EndpointRemoteInputSupervisor()
{
    stopX56InputThread();
}

qint64 EndpointRemoteInputSupervisor::monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

double EndpointRemoteInputSupervisor::directionNorm(
        const std::array<double, 3>& direction)
{
    return std::sqrt(direction[0] * direction[0] +
                     direction[1] * direction[1] +
                     direction[2] * direction[2]);
}

std::array<double, 3> EndpointRemoteInputSupervisor::sanitizedDirection(
        const std::array<double, 3>& direction)
{
    std::array<double, 3> result = direction;
    if(!std::all_of(result.begin(), result.end(), [](double value){
        return std::isfinite(value);
    })){
        result.fill(0.0);
        return result;
    }
    const double norm = directionNorm(result);
    if(norm > 1.0){
        for(double& value : result){
            value /= norm;
        }
    }
    return result;
}

void EndpointRemoteInputSupervisor::beginSession(
        quint64 sessionToken,
        const std::array<double, 3>& direction,
        qint64 leaseTimeoutUs)
{
    if(sessionToken == 0){
        return;
    }
    {
        QMutexLocker locker(&stateMutex);
        inputProfile = InputProfile::AwaitingZero;
        sessionStartDiagnosticPending = true;
        activeSessionToken = sessionToken;
        heartbeatSequence = 0;
        requestedDirection = sanitizedDirection(direction);
        lastUiUpdateUs = monotonicNowUs();
        uiLeaseTimeoutUs = std::max<qint64>(1, leaseTimeoutUs);
        lastTickUs = 0;
        maximumTickIntervalUs = 0;
        x56WasSelected = false;
        uiFallbackAwaitingZero = false;
    }
    if(x56InputChannel){
        x56InputChannel->setSession(sessionToken, false);
    }
    // 立即完成第一份零方向握手，不必等待首个50 ms心跳事件。
    QMetaObject::invokeMethod(this,
                              &EndpointRemoteInputSupervisor::tick,
                              Qt::QueuedConnection);
}

void EndpointRemoteInputSupervisor::updateUiState(
        quint64 sessionToken,
        const std::array<double, 3>& direction)
{
    QMutexLocker locker(&stateMutex);
    if(inputProfile == InputProfile::Inactive || sessionToken == 0 ||
            sessionToken != activeSessionToken){
        return;
    }
    requestedDirection = sanitizedDirection(direction);
    lastUiUpdateUs = monotonicNowUs();
}

void EndpointRemoteInputSupervisor::endSession(quint64 sessionToken)
{
    quint64 endedSessionToken = 0;
    quint64 finalSequence = 0;
    qint64 maximumIntervalUs = 0;
    {
        QMutexLocker locker(&stateMutex);
        if(inputProfile == InputProfile::Inactive ||
                (sessionToken != 0 && sessionToken != activeSessionToken)){
            return;
        }
        endedSessionToken = activeSessionToken;
        // 会话结束前生成最后一份明确的零方向。即使ControlWorker的停机请求
        // 随后短暂等待HardwareThread，这份零输入也不会让旧方向继续续租。
        finalSequence = ++heartbeatSequence;
        maximumIntervalUs = maximumTickIntervalUs;
        inputProfile = InputProfile::Inactive;
        sessionStartDiagnosticPending = false;
        activeSessionToken = 0;
        heartbeatSequence = 0;
        requestedDirection.fill(0.0);
        lastUiUpdateUs = 0;
        lastTickUs = 0;
        maximumTickIntervalUs = 0;
        x56WasSelected = false;
        uiFallbackAwaitingZero = false;
    }
    if(x56InputChannel){
        x56InputChannel->setSession(0, false);
    }
    if(controlWorker && endedSessionToken != 0){
        controlWorker->updateEndpointRemoteInput(
                    EndpointRemoteMotionMode::None,
                    std::array<double, 3>{},
                    finalSequence,
                    endedSessionToken,
                    true,
                    0);
    }
    emit diagnosticMessage(
                QStringLiteral(
                    "末端遥控独立输入监督会话已结束：会话=%1，最终序号=%2，最大调度间隔=%3 us")
                .arg(endedSessionToken)
                .arg(finalSequence)
                .arg(maximumIntervalUs),
                false);
}

void EndpointRemoteInputSupervisor::configureX56(
        quintptr windowHandle,
        bool fixedBindingEnabled,
        const QString& boundInstanceId)
{
    const QString normalizedId = boundInstanceId.trimmed().toUpper();
    {
        QMutexLocker locker(&stateMutex);
        x56FixedBindingEnabled = fixedBindingEnabled;
    }
    if(x56InputChannel){
        x56InputChannel->configure(windowHandle,
                                   fixedBindingEnabled,
                                   normalizedId);
    }
}

void EndpointRemoteInputSupervisor::updateApplicationActive(bool active)
{
    if(x56InputChannel){
        x56InputChannel->setApplicationActive(active);
    }
}

void EndpointRemoteInputSupervisor::start()
{
    startX56InputThread();
    if(!timer){
        timer = new QTimer(this);
        timer->setTimerType(Qt::PreciseTimer);
        timer->setInterval(kEndpointRemoteSupervisorPeriodMs);
        connect(timer, &QTimer::timeout,
                this, &EndpointRemoteInputSupervisor::tick);
    }
    timer->start();
}

void EndpointRemoteInputSupervisor::stop()
{
    if(timer){
        timer->stop();
    }
    endSession(0);
    stopX56InputThread();
}

void EndpointRemoteInputSupervisor::startX56InputThread()
{
    if(x56InputThread && x56InputThread->isRunning()){
        return;
    }
    if(x56InputThread){
        stopX56InputThread();
    }
    if(!x56InputChannel){
        x56InputChannel = std::make_shared<X56InputChannel>();
    }
    x56InputThread = new QThread(this);
    x56InputThread->setObjectName(QStringLiteral("X56InputThread"));
    x56InputWorker = new X56InputWorker(x56InputChannel);
    x56InputWorker->moveToThread(x56InputThread);
    connect(x56InputThread, &QThread::started,
            x56InputWorker, &X56InputWorker::start);
    connect(x56InputThread, &QThread::finished,
            x56InputWorker, &QObject::deleteLater);
    // 心跳监督线程由MainWindow以HighPriority启动；采集线程保持普通优先级，
    // 避免异常驱动占用CPU时反向挤压安全心跳。
    x56InputThread->start(QThread::NormalPriority);
}

void EndpointRemoteInputSupervisor::stopX56InputThread()
{
    X56InputWorker* worker = x56InputWorker;
    QThread* thread = x56InputThread;
    x56InputWorker = nullptr;
    x56InputThread = nullptr;
    if(!thread){
        return;
    }
    if(worker){
        worker->requestStop();
        QMetaObject::invokeMethod(worker, &X56InputWorker::stop,
                                  Qt::QueuedConnection);
    }
    thread->requestInterruption();
    thread->quit();
    if(!thread->wait(1500)){
        emit diagnosticMessage(
                    QStringLiteral(
                        "X56采集线程停止超时，疑似DirectInput阻塞；正在强制结束采集线程"),
                    true);
        thread->terminate();
        thread->wait(500);
    }
    delete thread;
}

void EndpointRemoteInputSupervisor::tick()
{
    const qint64 tickStartedUs = monotonicNowUs();
    std::array<double, 3> outputDirection{};
    std::array<double, 3> selectedDirection{};
    EndpointRemoteMotionMode outputMode = EndpointRemoteMotionMode::None;
    EndpointRemoteMotionMode selectedMode = EndpointRemoteMotionMode::None;
    quint64 sessionToken = 0;
    quint64 sequence = 0;
    qint64 uiAgeUs = -1;
    qint64 selectedSourceAgeUs = -1;
    qint64 tickIntervalUs = 0;
    qint64 maximumIntervalUs = 0;
    qint64 leaseTimeoutUs = 0;
    bool uiFresh = false;
    bool selectedSourceFresh = false;
    bool newlyStale = false;
    bool newlyRecovered = false;
    bool announceStart = false;
    double requestedNorm = 0.0;
    bool readerFixedBindingEnabled = false;
    bool x56SnapshotUsable = false;
    qint64 x56SnapshotAgeUs = -1;

    {
        QMutexLocker locker(&stateMutex);
        sessionToken = activeSessionToken;
        readerFixedBindingEnabled = x56FixedBindingEnabled;
    }
    if(sessionToken != x56SnapshotStateSessionToken){
        x56SnapshotStateSessionToken = sessionToken;
        requiredX56SafetyResetGeneration = 0;
        x56SnapshotWasStale = false;
    }

    bool remoteRunning = false;
    if(controlWorker && sessionToken != 0){
        remoteRunning = controlWorker->endpointRemoteStatus().state ==
                EndpointRemoteStatus::State::Running;
    }
    if(x56InputChannel){
        x56InputChannel->setRemoteRunning(sessionToken, remoteRunning);
    }

    const X56InputWorkerSnapshot workerSnapshot = x56InputChannel ?
                x56InputChannel->latestSnapshot() :
                X56InputWorkerSnapshot();
    const qint64 nowUs = monotonicNowUs();
    if(workerSnapshot.publishedAtUs > 0){
        x56SnapshotAgeUs = std::max<qint64>(
                    0, nowUs - workerSnapshot.publishedAtUs);
    }
    const bool x56SnapshotUnavailable = workerSnapshot.sequence == 0 ||
            workerSnapshot.sessionToken != sessionToken;
    const bool x56SnapshotTimedOut = !x56SnapshotUnavailable &&
            x56SnapshotAgeUs > kX56SnapshotTimeoutUs;
    x56SnapshotUsable = !x56SnapshotUnavailable && !x56SnapshotTimedOut;

    X56DirectInputSnapshot x56Snapshot = workerSnapshot.input;
    bool x56SnapshotBecameStale = false;
    bool x56SnapshotRecovered = false;
    if(x56SnapshotUnavailable){
        x56Snapshot = X56DirectInputSnapshot();
        x56Snapshot.state = X56DirectInputSnapshot::State::AcquisitionStale;
        x56Snapshot.statusText = QStringLiteral(
                    "X56：等待独立采集线程提供当前会话快照，输入保持为零");
    }
    else if(x56SnapshotTimedOut){
        const bool preserveTakeover = x56Snapshot.takeoverRequested;
        x56Snapshot.direction.fill(0.0);
        x56Snapshot.takeoverRequested = preserveTakeover;
        x56Snapshot.takeoverActive = false;
        x56Snapshot.sampleFresh = false;
        x56Snapshot.readAttempted = false;
        x56Snapshot.readFailed = false;
        x56Snapshot.state = X56DirectInputSnapshot::State::AcquisitionStale;
        x56Snapshot.statusText = QStringLiteral(
                    "X56：独立采集快照超过100 ms未更新，输入已归零；监督心跳仍在运行");
        x56Snapshot.warning = true;
        if(sessionToken != 0 && !x56SnapshotWasStale){
            x56SnapshotBecameStale = true;
            if(x56InputChannel){
                requiredX56SafetyResetGeneration =
                        x56InputChannel->requestSafetyReset();
            }
        }
        x56SnapshotWasStale = sessionToken != 0;
    }
    else if(x56SnapshotWasStale &&
            requiredX56SafetyResetGeneration != 0 &&
            workerSnapshot.appliedSafetyResetGeneration <
                requiredX56SafetyResetGeneration){
        // 阻塞的旧poll可能在超时后才返回。只有采集线程确认应用了新的
        // 安全重置代次，才能解除锁存；否则旧的B06非零快照可能短暂复活。
        const bool preserveTakeover = x56Snapshot.takeoverRequested;
        x56Snapshot.direction.fill(0.0);
        x56Snapshot.takeoverRequested = preserveTakeover;
        x56Snapshot.takeoverActive = false;
        x56Snapshot.sampleFresh = false;
        x56Snapshot.readAttempted = false;
        x56Snapshot.readFailed = false;
        x56Snapshot.state = X56DirectInputSnapshot::State::AcquisitionStale;
        x56Snapshot.statusText = QStringLiteral(
                    "X56：采集已恢复，等待安全重置快照确认，输入继续保持为零");
        x56Snapshot.warning = true;
        x56SnapshotUsable = false;
    }
    else{
        if(sessionToken != 0 && x56SnapshotWasStale){
            x56SnapshotRecovered = true;
        }
        requiredX56SafetyResetGeneration = 0;
        x56SnapshotWasStale = false;
    }
    if(sessionToken == 0){
        requiredX56SafetyResetGeneration = 0;
        x56SnapshotWasStale = false;
    }

    if(workerSnapshot.sessionToken != observedX56WorkerSessionToken){
        observedX56WorkerSessionToken = workerSnapshot.sessionToken;
        lastReportedX56SlowPollCount = 0;
        lastReportedX56FailureEventCount = 0;
        lastX56SlowPollDiagnosticUs = 0;
    }
    if(workerSnapshot.slowPollCount < lastReportedX56SlowPollCount){
        lastReportedX56SlowPollCount = 0;
    }
    if(workerSnapshot.readFailureEventCount <
            lastReportedX56FailureEventCount){
        lastReportedX56FailureEventCount = 0;
    }

    QStringList currentX56DeviceNames;
    QStringList currentX56DeviceIds;
    const std::vector<X56DirectInputDeviceInfo> currentX56Devices =
            workerSnapshot.devices;
    for(const X56DirectInputDeviceInfo& device : currentX56Devices){
        const QString shortId = device.instanceId.left(8);
        currentX56DeviceNames.append(
                    QStringLiteral("%1 [%2]")
                    .arg(device.displayName, shortId));
        currentX56DeviceIds.append(device.instanceId);
    }
    if(readerFixedBindingEnabled && currentX56Devices.size() > 1){
        if(!x56Snapshot.statusText.contains(QStringLiteral("同型号"))){
            x56Snapshot.statusText += QStringLiteral(
                        "；检测到%1只同型号设备，当前仅接受固定绑定实例")
                    .arg(currentX56Devices.size());
        }
        x56Snapshot.warning = true;
    }
    if(currentX56DeviceNames != lastX56DeviceNames ||
            currentX56DeviceIds != lastX56DeviceIds){
        lastX56DeviceNames = currentX56DeviceNames;
        lastX56DeviceIds = currentX56DeviceIds;
        emit x56DevicesChanged(currentX56DeviceNames,
                               currentX56DeviceIds);
    }
    if(x56Snapshot.statusText != lastX56StatusText ||
            x56Snapshot.warning != lastX56StatusWarning){
        const bool firstStatus = lastX56StatusText.isEmpty();
        lastX56StatusText = x56Snapshot.statusText;
        lastX56StatusWarning = x56Snapshot.warning;
        emit x56StatusChanged(x56Snapshot.statusText,
                              x56Snapshot.warning);
        if(x56Snapshot.warning && !firstStatus && !x56Snapshot.readFailed &&
                x56Snapshot.state !=
                X56DirectInputSnapshot::State::AcquisitionStale){
            emit diagnosticMessage(x56Snapshot.statusText, true);
        }
    }

    if(workerSnapshot.readFailureEventCount >
            lastReportedX56FailureEventCount){
        lastReportedX56FailureEventCount =
                workerSnapshot.readFailureEventCount;
        emit diagnosticMessage(
                    QStringLiteral(
                        "X56 DirectInput访问失败：本次/会话最大轮询耗时=%1/%2 us，连续失败=%3；输入已归零，下一次设备访问至少延后500 ms")
                    .arg(workerSnapshot.latestPollDurationUs)
                    .arg(workerSnapshot.maximumPollDurationUs)
                    .arg(workerSnapshot.consecutiveReadFailureCount),
                    true);
    }
    if(workerSnapshot.slowPollCount > lastReportedX56SlowPollCount &&
            (lastX56SlowPollDiagnosticUs == 0 ||
             nowUs - lastX56SlowPollDiagnosticUs >=
             kX56PollDiagnosticThrottleUs)){
        lastX56SlowPollDiagnosticUs = nowUs;
        lastReportedX56SlowPollCount = workerSnapshot.slowPollCount;
        emit diagnosticMessage(
                    QStringLiteral(
                        "X56 DirectInput轮询过慢：本次/会话最大耗时=%1/%2 us，阈值=%3 us，慢轮询累计=%4，连续读取失败=%5")
                    .arg(workerSnapshot.latestPollDurationUs)
                    .arg(workerSnapshot.maximumPollDurationUs)
                    .arg(X56InputWorker::SlowPollThresholdUs)
                    .arg(workerSnapshot.slowPollCount)
                    .arg(workerSnapshot.consecutiveReadFailureCount),
                    true);
    }
    if(x56SnapshotBecameStale){
        emit diagnosticMessage(
                    QStringLiteral(
                        "X56独立采集快照超时：年龄=%1 us，阈值=%2 us；手柄方向已归零并请求重新回中，50 ms监督心跳未中断")
                    .arg(x56SnapshotAgeUs)
                    .arg(kX56SnapshotTimeoutUs),
                    true);
    }
    else if(x56SnapshotRecovered){
        emit diagnosticMessage(
                    QStringLiteral(
                        "X56独立采集快照已恢复：年龄=%1 us；恢复运动前仍需重新完成全部输入回中")
                    .arg(x56SnapshotAgeUs),
                    false);
    }

    {
        QMutexLocker locker(&stateMutex);
        if(inputProfile == InputProfile::Inactive || activeSessionToken == 0 ||
                activeSessionToken != sessionToken){
            lastTickUs = 0;
            return;
        }
        if(lastTickUs > 0){
            tickIntervalUs = std::max<qint64>(0, tickStartedUs - lastTickUs);
            maximumTickIntervalUs = std::max(maximumTickIntervalUs,
                                             tickIntervalUs);
        }
        lastTickUs = tickStartedUs;
        maximumIntervalUs = maximumTickIntervalUs;
        uiAgeUs = lastUiUpdateUs > 0 ?
                    std::max<qint64>(0, nowUs - lastUiUpdateUs) : -1;
        uiFresh = uiAgeUs >= 0 && uiAgeUs <= uiLeaseTimeoutUs;
        leaseTimeoutUs = uiLeaseTimeoutUs;

        const bool x56Selected = x56Snapshot.takeoverRequested;
        if(x56WasSelected && !x56Selected){
            // 手柄释放、失焦或断开后，必须先观察到一帧明确的UI零方向，
            // 才允许原键盘/按钮重新接管。
            uiFallbackAwaitingZero = true;
        }
        x56WasSelected = x56Selected;
        if(x56Selected){
            selectedDirection = x56Snapshot.takeoverActive ?
                        x56Snapshot.direction : std::array<double, 3>{};
            selectedMode = x56Snapshot.takeoverActive ?
                        x56Snapshot.motionMode : EndpointRemoteMotionMode::None;
            selectedSourceFresh = x56SnapshotUsable &&
                    x56Snapshot.sampleFresh;
            selectedSourceAgeUs = x56SnapshotAgeUs;
        }
        else if(uiFallbackAwaitingZero){
            selectedDirection.fill(0.0);
            selectedMode = EndpointRemoteMotionMode::None;
            selectedSourceFresh = uiFresh;
            selectedSourceAgeUs = uiAgeUs;
            if(uiFresh && directionNorm(requestedDirection) <= 1.0e-12){
                // 本周期仍发布零；下一周期才重新开放新的UI方向。
                uiFallbackAwaitingZero = false;
            }
        }
        else{
            selectedDirection = requestedDirection;
            selectedMode = directionNorm(selectedDirection) > 1.0e-12 ?
                        EndpointRemoteMotionMode::Translation :
                        EndpointRemoteMotionMode::None;
            selectedSourceFresh = uiFresh;
            selectedSourceAgeUs = uiAgeUs;
        }
        selectedDirection = sanitizedDirection(selectedDirection);
        requestedNorm = directionNorm(selectedDirection);

        switch(inputProfile){
        case InputProfile::Inactive:
            return;
        case InputProfile::AwaitingZero:
            if(!selectedSourceFresh){
                inputProfile = InputProfile::StaleZeroLatched;
                newlyStale = true;
            }
            else if(requestedNorm <= 1.0e-12){
                inputProfile = InputProfile::ActiveFresh;
            }
            break;
        case InputProfile::ActiveFresh:
            if(!selectedSourceFresh){
                inputProfile = InputProfile::StaleZeroLatched;
                newlyStale = true;
            }
            break;
        case InputProfile::StaleZeroLatched:
            if(selectedSourceFresh && requestedNorm <= 1.0e-12){
                // 输入源恢复后先发布一轮明确零方向，再重新开放非零输入。
                inputProfile = InputProfile::RecoveringZero;
                newlyRecovered = true;
            }
            break;
        case InputProfile::RecoveringZero:
            inputProfile = InputProfile::ActiveFresh;
            break;
        }

        outputDirection = inputProfile == InputProfile::ActiveFresh ?
                    selectedDirection : std::array<double, 3>{};
        outputMode = inputProfile == InputProfile::ActiveFresh ?
                    selectedMode : EndpointRemoteMotionMode::None;
        sequence = ++heartbeatSequence;
        announceStart = sessionStartDiagnosticPending;
        sessionStartDiagnosticPending = false;
        // 对ControlWorker而言，fresh=false表示监督器仍活着但当前输入源已失效；
        // 输出方向始终被强制为零。字段名保留uiFresh以兼容既有状态结构。
        selectedSourceFresh = selectedSourceFresh &&
                (inputProfile == InputProfile::ActiveFresh ||
                 inputProfile == InputProfile::RecoveringZero);
    }

    if(controlWorker){
        controlWorker->updateEndpointRemoteInput(outputMode,
                                                 outputDirection,
                                                 sequence,
                                                 sessionToken,
                                                 selectedSourceFresh,
                                                 selectedSourceAgeUs);
    }

    if(announceStart){
        emit diagnosticMessage(
                    QStringLiteral(
                        "末端遥控输入链已启动：会话=%1；监督心跳=%2 ms，X56独立采集=%3 ms，采集快照超时=%4 ms，ControlWorker心跳失效上限=%5 ms；输入发布motionMode+direction[3]")
                    .arg(sessionToken)
                    .arg(kEndpointRemoteSupervisorPeriodMs)
                    .arg(X56InputWorker::PollPeriodMs)
                    .arg(kX56SnapshotTimeoutUs / 1000)
                    .arg(leaseTimeoutUs / 1000),
                    false);
    }
    if(tickIntervalUs > kEndpointRemoteSupervisorLateWarnUs){
        emit diagnosticMessage(
                    QStringLiteral(
                        "末端遥控输入监督线程调度迟到：本次/最大间隔=%1/%2 us，目标=%3 us，会话=%4，序号=%5")
                    .arg(tickIntervalUs)
                    .arg(maximumIntervalUs)
                    .arg(kEndpointRemoteSupervisorPeriodMs * 1000)
                    .arg(sessionToken)
                    .arg(sequence),
                    true);
    }
    if(newlyStale){
        emit uiLeaseTransition(sessionToken,
                               UiLeaseTransition::StaleLatched,
                               selectedSourceAgeUs);
        emit diagnosticMessage(
                    QStringLiteral(
                        "末端遥控当前输入源失效：输入年龄=%1 us，请求方向范数=%2；独立监督线程已锁存零方向，输入恢复后必须先发布零方向才能重新运动")
                    .arg(selectedSourceAgeUs)
                    .arg(requestedNorm, 0, 'f', 6),
                    true);
    }
    else if(newlyRecovered){
        emit uiLeaseTransition(sessionToken,
                               UiLeaseTransition::Recovered,
                               selectedSourceAgeUs);
        emit diagnosticMessage(
                    QStringLiteral(
                        "末端遥控当前输入源已恢复：输入年龄=%1 us，已确认零方向，会话可继续接收新输入")
                    .arg(selectedSourceAgeUs),
                    false);
    }
}
