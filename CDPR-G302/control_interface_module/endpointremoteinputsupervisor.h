#ifndef ENDPOINTREMOTEINPUTSUPERVISOR_H
#define ENDPOINTREMOTEINPUTSUPERVISOR_H

#include "x56inputworker.h"

#include <QObject>
#include <QMetaType>
#include <QMutex>
#include <QString>
#include <QStringList>
#include <QtGlobal>

#include <array>
#include <memory>

class ControlWorker;
class QThread;
class QTimer;

// 末端遥控输入监督器只按50 ms发布心跳和选择最新输入，不访问DirectInput。
// X56由另一条20 ms采集线程写入最新快照邮箱，驱动阻塞不会拖住本监督线程。
class EndpointRemoteInputSupervisor : public QObject
{
    Q_OBJECT

public:
    enum class InputProfile {
        Inactive = 0,
        AwaitingZero,
        ActiveFresh,
        StaleZeroLatched,
        RecoveringZero
    };
    Q_ENUM(InputProfile)

    enum class UiLeaseTransition {
        StaleLatched = 0,
        Recovered
    };
    Q_ENUM(UiLeaseTransition)

    explicit EndpointRemoteInputSupervisor(ControlWorker* controlWorker,
                                           QObject* parent = nullptr);
    ~EndpointRemoteInputSupervisor() override;

    // 以下入口均可由UI线程直接调用；内部只持有短时互斥锁。
    void beginSession(quint64 sessionToken,
                      const std::array<double, 3>& direction,
                      qint64 uiLeaseTimeoutUs);
    void updateUiState(quint64 sessionToken,
                       const std::array<double, 3>& direction);
    void endSession(quint64 sessionToken);
    void configureX56(quintptr windowHandle,
                      bool fixedBindingEnabled,
                      const QString& boundInstanceId);
    void updateApplicationActive(bool active);

public slots:
    void start();
    void stop();

signals:
    void diagnosticMessage(const QString& message, bool warning);
    // GUI_PERF_DIAG：结构化租约状态只供可移除的GUI性能归因使用；
    // 不改变原有租约锁存、零方向或诊断消息行为。
    void uiLeaseTransition(quint64 sessionToken,
                           UiLeaseTransition transition,
                           qint64 uiAgeUs);
    void x56StatusChanged(const QString& statusText, bool warning);
    void x56DevicesChanged(const QStringList& displayNames,
                           const QStringList& instanceIds);

private slots:
    void tick();

private:
    static qint64 monotonicNowUs();
    static std::array<double, 3> sanitizedDirection(
            const std::array<double, 3>& direction);
    static double directionNorm(const std::array<double, 3>& direction);
    void startX56InputThread();
    void stopX56InputThread();

    ControlWorker* controlWorker = nullptr;
    QTimer* timer = nullptr;
    QMutex stateMutex;
    InputProfile inputProfile = InputProfile::Inactive;
    bool sessionStartDiagnosticPending = false;
    quint64 activeSessionToken = 0;
    quint64 heartbeatSequence = 0;
    std::array<double, 3> requestedDirection{};
    qint64 lastUiUpdateUs = 0;
    qint64 uiLeaseTimeoutUs = 250000;
    qint64 lastTickUs = 0;
    qint64 maximumTickIntervalUs = 0;
    bool x56FixedBindingEnabled = false;
    std::shared_ptr<X56InputChannel> x56InputChannel;
    QThread* x56InputThread = nullptr;
    X56InputWorker* x56InputWorker = nullptr;
    quint64 observedX56WorkerSessionToken = 0;
    quint64 lastReportedX56SlowPollCount = 0;
    quint64 lastReportedX56FailureEventCount = 0;
    qint64 lastX56SlowPollDiagnosticUs = 0;
    quint64 x56SnapshotStateSessionToken = 0;
    quint64 requiredX56SafetyResetGeneration = 0;
    bool x56SnapshotWasStale = false;
    bool x56WasSelected = false;
    bool uiFallbackAwaitingZero = false;
    QString lastX56StatusText;
    bool lastX56StatusWarning = false;
    QStringList lastX56DeviceNames;
    QStringList lastX56DeviceIds;
};

Q_DECLARE_METATYPE(EndpointRemoteInputSupervisor::UiLeaseTransition)

#endif // ENDPOINTREMOTEINPUTSUPERVISOR_H
