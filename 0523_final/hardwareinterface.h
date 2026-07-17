#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

#include <QObject>
#include <QtCore/QtCore>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <type_traits>
#include <vector>

#include "macro.h"
#include "LTDMC.h"

#pragma execution_character_set("utf-8")

class HardwareInterface : public QObject
{
    Q_OBJECT
public:
    struct DiagnosticRawSample {
        qint64 wallClockMs = 0;
        qint64 intervalUs = 0;
    };

    struct DiagnosticsSnapshot {
        quint64 communicationEventCount = 0;
        quint64 communicationIntervalCount = 0;
        qint64 communicationIntervalSumUs = 0;
        qint64 latestCommunicationIntervalUs = 0;
        quint64 motorCommandEventCount = 0;
        quint64 motorCommandIntervalCount = 0;
        qint64 motorCommandIntervalSumUs = 0;
        qint64 latestMotorCommandIntervalUs = 0;
    };

    enum class ConnectionState {
        Disconnected = 0,
        Connected,
        Disabled,
        Fault
    };

    struct ConnectionItemDiagnostics {
        ConnectionState state = ConnectionState::Disconnected;
        int hardwareAxis = -1;
        int stateMachine = -1;
        int slaveAddress = -1;
        int subSlaveAddress = -1;
        int busState = -1;
        long statusWord = 0;
        long stopReason = 0;
        int errorCode = 0;
        int apiResult = 0;
        int stopReasonApiResult = 0;
    };

    struct ConnectionDiagnostics {
        ConnectionItemDiagnostics controller;
        std::vector<ConnectionItemDiagnostics> motorAxes;
        std::vector<ConnectionItemDiagnostics> forceSensors;
    };

    struct PdoTraceObjectConfig {
        WORD index = 0;
        WORD subIndex = 0;
    };

    struct PdoTraceProbeConfig {
        WORD channel = 2;
        WORD slaveAddress = 0;
        DWORD traceLength = 1024;
        DWORD readStartAddress = 0;
        DWORD readLengthBytes = 0;
        DWORD maxReadLengthBytes = 1024 * 1024;
        int waitTimeoutMs = 1000;
        int pollIntervalMs = 10;
        std::vector<PdoTraceObjectConfig> objects;
    };

    struct PdoTraceProbeResult {
        bool success = false;
        bool timedOut = false;
        short preStopResult = -1;
        short clearBeforeStartResult = -1;
        short startResult = -1;
        short getNumResult = -1;
        short stateResult = -1;
        short readResult = -1;
        short stopResult = -1;
        DWORD dataNum = 0;
        DWORD sizeOfEachPacket = 0;
        WORD traceState = 0;
        DWORD requestedReadLength = 0;
        DWORD actualReadLength = 0;
        int pollCount = 0;
        QByteArray data;
        QStringList messages;
    };

    struct TraceObjectConfig {
        short dataType = 0;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short dataBytes = 4;
    };

    struct TraceProbeConfig {
        bool configureSource = false;
        WORD source = 0;
        short traceCycle = 0;
        short lostHandle = 0;
        short traceType = 0;
        short triggerObjectIndex = 0;
        short triggerType = 0;
        int mask = 0;
        long long condition = 0;
        int bufferSizeBytes = 4096;
        int maxBufferSizeBytes = 1024 * 1024;
        int waitTimeoutMs = 1000;
        int pollIntervalMs = 10;
        std::vector<TraceObjectConfig> objects;
    };

    struct TraceProbeResult {
        bool success = false;
        bool timedOut = false;
        short setSourceResult = -1;
        short stopBeforeConfigResult = -1;
        short dataResetBeforeConfigResult = -1;
        short setConfigResult = -1;
        short resetConfigObjectResult = -1;
        short addConfigObjectResult = -1;
        short dataStartResult = -1;
        short getFlagResult = -1;
        short getStateResult = -1;
        short getDataResult = -1;
        short dataStopResult = -1;
        short startFlag = 0;
        short triggeredFlag = 0;
        short lostFlag = 0;
        int validNum = 0;
        int freeNum = 0;
        int objectTotalBytes = 0;
        int objectTotalNum = 0;
        int actualReadLength = 0;
        int pollCount = 0;
        QByteArray data;
        QStringList messages;
    };

    struct ForceSensorReadResult {
        std::vector<double> values;
        int frameCount = 0;
        bool fromTrace = false;
    };

    struct ForceSensorTraceSample {
        std::vector<double> values;
    };

    struct MotorTracePositionSample {
        double commandRelativePosition = 0.0;
        double feedbackRelativePosition = 0.0;
    };

    HardwareInterface();
    HardwareInterface(double _threadCtrlCycleMs);
    virtual ~HardwareInterface() = default;

    void setMotorPara(std::vector<unsigned int> idVec, std::vector<int> comType, std::vector<QString> portInfo,
                      std::vector<double> posRaw2dataCof, std::vector<double> velRaw2dataCof,
                      std::vector<int> slaveIdVec = {},
                      std::vector<bool> torqueVelocityLimitEnabled = {});
    void setMotorSlaveIds(std::vector<int> slaveIdVec);
    void setMotorTorqueVelocityLimitEnabled(std::vector<bool> enabled);
    void setMotorChangeSpdTime(double s);
    void setMotorHome(std::vector<double> homeValue);
    void setLeadshineAxisEquiv(std::vector<double> equivValue);
    void setMotorSoftwareLimits(std::vector<double> minPos,
                                std::vector<double> maxPos);
    bool useStaticMotorHome = false;

    void setSensorPara(std::vector<unsigned int> idVec, std::vector<int> comType, std::vector<QString> portInfo,
                       std::vector<double> raw2dataCof);
    void setSensorPara(std::vector<int> comType, std::vector<int> _sensorPort, std::vector<int> _sensorAdr, std::vector<int> _sensorDataAdr,
                       std::vector<int> _sensorDataLen, std::vector<double> raw2dataCof);
    void setForceSensorHome(std::vector<double> homeValue);
    void setForceSensorIsSigned(bool isSigned);
    bool useStaticSensorHome = false;

    bool connectLS();
    bool applyLeadshineAxisEquiv();
    void setLeadshineTorqueVelocityLimitRpm(double velocityLimitRpm);
    bool applyLeadshineTorqueVelocityLimit(double velocityLimitRpm = 600.01);

    bool disconnectLS();
    bool setAllMotorEnable(bool enable);
    bool setMotorEnable(int index, bool enable);
    bool emergencyStopAll();
    bool clearAllLeadshineAxisErrorCodes();
    bool isLSConnected() const;
    bool isMotorDone(int index) const;
    bool hasPvtTrajectory() const;
    bool isPvtMotionRunning() const;
    bool isPvtMotionPausedState() const;
    bool isPvtMotionFinished() const;
    bool currentPvtProgress(double& currentTrajectoryTime, std::size_t& currentIndex) const;
    void clearPvtTrajectoryState();
    bool pausePvtMotion();
    bool resumePvtMotion();
    bool smoothPausePvtMotion(double transitionTimeSec = 0.5);
    bool smoothResumePvtMotion(double transitionTimeSec = 0.5);

    bool motorStop(int index);
    bool motorHome(int index);
    bool motorAbsPos(int index, double pos, double vel);
    bool motorVel(int index, double vel);
    bool motorVelWithTargetPosAndStopVel(int index, double vel, double targetPos, double stopVel);
    bool motorTorqueStart(int index, double torqueNm);
    bool motorTorqueChange(int index, double torqueNm);
    double readMotorCurPos(int index);
    bool readMotorRelativeCurPos(int index, double& relativePosition);
    bool readMotorRelativeTracePositions(int index,
                                         double& commandRelativePosition,
                                         double& feedbackRelativePosition);
    std::vector<MotorTracePositionSample> readMotorRelativeTracePositionSamples(int index);
    bool readMotorCommandPosition(int index, double& commandPosition);
    bool configureMotorPositionTrace(int index);
    bool readMotorTracePositions(int index, double& commandPosition, double& actualPosition);
    void stopMotorPositionTrace();
    bool readLeadshineFollowingErrorRaw(int index, int& followingErrorRaw);

    void canPortInfoProcessor(const QString portInfo, int &deviceIndex, int &canPortIndex);
    void canPortInfoProcessor(const std::vector<QString> portInfo, std::vector<int> &deviceIndex, std::vector<int> &canPortIndex);

    void checkAllMotorState();
    void checkAllMotorPos();
    void checkAllMotorVel();
    std::vector<bool> getAllMotorState();
    bool isMotorEnabled(int index);
    std::vector<bool> motorCurState;
    std::vector<double> getAllMotorPos();
    std::vector<double> motorCurPos;
    std::vector<double> motorCommandPos;
    std::vector<double> getAllMotorVel();
    std::vector<double> motorCurVel;
    std::vector<double> getAllMotorTorqueNm();
    std::vector<double> getAllMotorHome();

    std::vector<double> readForceSensorDataSnapshot();
    std::vector<double> readForceSensorDataCached(int maxChannelsToRead);
    ForceSensorReadResult readForceSensorDataCachedResult(int maxChannelsToRead);
    std::vector<ForceSensorTraceSample> readForceSensorDataTraceSamples();
    void setForceSensorTraceReadEnabled(bool enabled);
    void setForceSensorTraceSamplePeriodUs(int periodUs);
    PdoTraceProbeResult runPdoTraceForceSensorProbe(const PdoTraceProbeConfig& config);
    TraceProbeResult runTraceForceSensorProbe(const TraceProbeConfig& config);
    DiagnosticsSnapshot diagnosticsSnapshot() const;
    QVector<DiagnosticRawSample> communicationTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    QVector<DiagnosticRawSample> motorCommandTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    ConnectionDiagnostics connectionDiagnostics() const;
    ConnectionItemDiagnostics motorAxisDiagnostics(int logicalIndex) const;
    void resetDiagnostics();
    void checkJointSensorDataManual();
    std::vector<double> rodCurForce;
    std::vector<double> jointCurTheta;
    std::vector<int> jointCurThetaID;

    double threadCtrlCycleMs = 0.0;
    void motorPosTraj(std::vector<int> motorIndex, std::vector<std::vector<double>> motorPosTraj,
                      std::vector<std::vector<double>> motorVel, std::vector<double> timeStamp);

private:
    template<typename F>
    auto runOnHardwareThread(F&& f) -> decltype(f()) {
        using R = decltype(f());
        QThread* targetThread = thread();
        if (!targetThread || !targetThread->isRunning() || QThread::currentThread() == targetThread) {
            return f();
        }

        if constexpr (std::is_void_v<R>) {
            QMetaObject::invokeMethod(this, [&]() { f(); }, Qt::BlockingQueuedConnection);
        } else {
            R result{};
            QMetaObject::invokeMethod(this, [&]() { result = f(); }, Qt::BlockingQueuedConnection);
            return result;
        }
    }

    template<typename F>
    auto runOnHardwareThread(F&& f) const -> decltype(f()) {
        using R = decltype(f());
        QThread* targetThread = thread();
        if (!targetThread || !targetThread->isRunning() || QThread::currentThread() == targetThread) {
            return f();
        }

        if constexpr (std::is_void_v<R>) {
            QMetaObject::invokeMethod(const_cast<HardwareInterface*>(this), [&]() { f(); }, Qt::BlockingQueuedConnection);
        } else {
            R result{};
            QMetaObject::invokeMethod(const_cast<HardwareInterface*>(this), [&]() { result = f(); }, Qt::BlockingQueuedConnection);
            return result;
        }
    }

    bool isConnectLS = false;
    bool forceSensorIsSigned = true;
    mutable QMutex diagnosticsMutex;
    DiagnosticsSnapshot diagnostics;
    qint64 lastCommunicationEventUs = 0;
    qint64 lastMotorCommandEventUs = 0;
    QVector<DiagnosticRawSample> communicationRawHistory;
    QVector<DiagnosticRawSample> motorCommandRawHistory;
    qint64 lastCommunicationHistoryTrimMs = 0;
    qint64 lastMotorCommandHistoryTrimMs = 0;
    mutable QMutex connectionDiagnosticsMutex;
    mutable ConnectionDiagnostics cachedConnectionDiagnostics;
    mutable bool connectionDiagnosticsRefreshPending = false;

    std::vector<double> motorHomePos;
    std::vector<double> motorSoftwareMinPos;
    std::vector<double> motorSoftwareMaxPos;

    std::vector<unsigned int> motorIdVec, sensorIdVec;
    std::vector<int> motorSlaveIdVec;
    std::vector<bool> motorTorqueVelocityLimitEnabled;
    std::vector<int> motorComType, sensorComType, sensorPort, sensorAdr, sensorDataAdr, sensorDataLen;
    std::vector<QString> motorPortInfo, sensorPortInfo;
    std::vector<double> motorPosRaw2dataCof, motorVelRaw2dataCof, motorEquivVec, sensorRaw2DataCof, sensorHomeValue;
    std::vector<double> forceSensorCachedValue;
    std::vector<bool> forceSensorCacheValid;
    int nextForceSensorPollIndex = 0;
    bool forceSensorTraceReadEnabled = true;
    int forceSensorTraceSamplePeriodUs = 500;
    bool runtimeTraceConfigured = false;
    bool runtimeTraceUnavailable = false;
    bool runtimeTraceEverRead = false;
    int runtimeTraceConsecutiveFailures = 0;
    int runtimeTraceObjectTotalBytes = 0;
    int runtimeTraceObjectTotalNum = 0;
    qint64 runtimeTraceLastRetryUs = 0;

    struct ForceSensorTraceObject {
        int sensorIndex = -1;
        short dataType = 19;
        int dataIndex = 0x6000;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 0;
        int valueBytes = 2;
    };
    std::vector<ForceSensorTraceObject> forceSensorTraceObjects;

    int motorPositionTraceSamplePeriodUs = 500;

    struct MotorCommandPositionTraceObject {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short dataType = 5;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 4;
        int valueBytes = 4;
    };
    std::vector<MotorCommandPositionTraceObject> motorCommandPositionTraceObjects;

    struct MotorPositionTraceObject {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short dataType = 6;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 4;
        int valueBytes = 4;
    };
    std::vector<MotorPositionTraceObject> motorPositionTraceObjects;

    enum class RuntimeTraceObjectKind {
        MotorCommandPosition = 0,
        MotorPosition,
        ForceSensor
    };

    struct RuntimeTraceObject {
        RuntimeTraceObjectKind kind = RuntimeTraceObjectKind::MotorCommandPosition;
        int objectIndex = -1;
        int valueBytes = 4;
    };
    std::vector<RuntimeTraceObject> runtimeTraceObjects;
    std::vector<std::deque<MotorTracePositionSample>> motorTracePositionSampleQueues;
    std::deque<ForceSensorTraceSample> forceSensorTraceSampleQueue;

    bool hasActivePvtTrajectory = false;
    bool isPvtMotionPaused = false;
    std::size_t pausedPvtResumeIndex = 0;
    double pausedPvtResumeTime = 0.0;
    std::vector<int> activePvtMotorIndex;
    std::vector<std::vector<double>> activePvtMotorPosTraj;
    std::vector<std::vector<double>> activePvtMotorVelTraj;
    std::vector<double> activePvtTimeStamp;

    double velChangeSpd = 0.01;
    double leadshineTorqueVelocityLimitRpm = 600.01;

    void delay(unsigned int msec);
    void unsupportedFeature(const QString& featureName);
    bool applyLeadshineAxisEnableState(int logicalIndex, bool enable, bool emitErrors = true);
    bool refreshLeadshineMotorEnableState(int logicalIndex);
    bool startPvtTable(const std::vector<int>& motorIndex,
                       const std::vector<std::vector<double>>& motorPosTraj,
                       const std::vector<std::vector<double>>& motorVelTraj,
                       const std::vector<double>& timeStamp,
                       const std::vector<double>& beginVel,
                       const std::vector<double>& endVel,
                       bool updateActiveTrajectory,
                       const QString& successMessage);
    bool getPvtCurrentProgress(double& currentTrajectoryTime, std::size_t& currentIndex) const;
    std::vector<double> readMotorPositions(const std::vector<int>& motorIndex) const;
    std::vector<double> readMotorSpeeds(const std::vector<int>& motorIndex) const;
    bool waitPvtAxesDone(int timeoutMs);
    bool stopActivePvtAxesAndWait(int timeoutMs);
    void resetRuntimeTraceState();
    void resetForceSensorTraceState();
    void resetMotorPositionTraceState();
    bool configureRuntimeTraceRead();
    int readRuntimeTraceCached();
    bool configureMotorPositionTraceRead();
    bool readMotorPositionTraceCached(int logicalIndex, double& position);
    std::vector<double> readMotorPositionsTraceCached(const std::vector<int>& motorIndex);
    std::vector<double> currentMotorPositionCachedValues(const std::vector<int>& motorIndex) const;
    void applyMotorCommandPositionTraceRawValue(int logicalIndex, long rawValue);
    void applyMotorPositionTraceRawValue(int logicalIndex, long rawValue);
    bool configureForceSensorTraceRead();
    std::vector<double> readForceSensorDataCachedDirect(int maxChannelsToRead);
    std::vector<double> readForceSensorDataTraceCached();
    ForceSensorReadResult readForceSensorDataCachedDirectResult(int maxChannelsToRead);
    ForceSensorReadResult readForceSensorDataTraceCachedResult();
    std::vector<double> currentForceSensorCachedValues() const;
    short readForceSensorTxpdoExtra(int sensorIndex, int* rawValue) const;
    void applyForceSensorRawValue(int sensorIndex, long rawValue);
    void recordCommunicationEvent(bool motorCommandEvent = false);
    int resolveLeadshineAxisIndex(int logicalIndex) const;
    double resolveLeadshineAxisEquiv(int logicalIndex) const;
    QString axisDisplayName(int logicalIndex) const;
    bool hasValidMotorSoftwareLimit(int logicalIndex) const;
    double relativeMotorPosition(int logicalIndex, double absolutePosition) const;
    bool validateRelativeMotorSoftwareLimit(int logicalIndex,
                                            double relativePosition,
                                            const QString& commandName,
                                            QString* errorMessage = nullptr) const;
    bool validateAbsoluteMotorSoftwareLimit(int logicalIndex,
                                            double absolutePosition,
                                            const QString& commandName,
                                            QString* errorMessage = nullptr) const;
    bool validateVelocityMotorSoftwareLimit(int logicalIndex,
                                            double currentAbsolutePosition,
                                            double velocity,
                                            const QString& commandName,
                                            QString* errorMessage = nullptr) const;
    ConnectionItemDiagnostics queryMotorAxisDiagnosticsDirect(int logicalIndex) const;
    ConnectionDiagnostics queryConnectionDiagnosticsDirect() const;
    void requestConnectionDiagnosticsRefresh() const;
    void storeConnectionDiagnosticsSnapshot(const ConnectionDiagnostics& snapshot) const;

signals:
    void motorCurPosUpdateSignal(std::vector<double> pos);
    void motorCurVelUpdateSignal(std::vector<double> vel);
    void displayInfoSignal(std::string info, std::string type);
};

#endif // HARDWAREINTERFACE_H
