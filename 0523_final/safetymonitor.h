#ifndef SAFETYMONITOR_H
#define SAFETYMONITOR_H

#include <QObject>
#include <QMutex>
#include <QString>
#include <vector>

class ControlWorker;
class HardwareInterface;
class QTimer;

class SafetyMonitor : public QObject
{
    Q_OBJECT

public:
    enum class StopLevel {
        Warning = 0,
        ControlledStop = 1,
        SafetyStop = 2,
        EmergencyStop = 3
    };
    Q_ENUM(StopLevel)

    enum class FaultCode {
        None = 0,
        SnapshotTimeout = 1,
        HardwareDisconnected = 2,
        CableForceLow = 3,
        CableForceHigh = 4,
        CableBreak = 5,
        MotorRangeExceeded = 6,
        MotorOverspeed = 7,
        SensorInvalid = 8,
        WorkspaceExceeded = 9,
        SoftwareHang = 10,
        MotorTorqueExceeded = 11
    };
    Q_ENUM(FaultCode)

    struct AxisConfig {
        bool monitored = false;
        bool modeledForForceJacobian = false;
        int sensorIndex = -1;
        int endIndex = -1;
        double forceMin = 0.0;
        double forceMax = 0.0;
        double motorMin = 0.0;
        double motorMax = 0.0;
        double motorVelMax = 0.0;
        std::vector<double> anchorPoint;
        std::vector<double> contactPointLocal;
    };

    struct Config {
        int axisCount = 0;
        int sensorCount = 0;
        double cycleMs = 10.0;
        bool monitorEnabled = true;
        bool systemRunning = false;
        bool hardwareConnected = false;
        bool motionActive = false;
        bool forceThreadRunning = false;
        bool singleCableForceDebugMode = false;
        int snapshotTimeoutMs = 300;
        int persistentFaultCycles = 4;
        double breakForceRatio = 0.2;
        double breakDropRatio = 0.65;
        double severeForceOverRatio = 1.15;
        double severeSpeedOverRatio = 1.2;
        bool cableBreakControllabilityEnabled = false;
        int poseEndIndex = 0;
        int cableBreakControllableRankThreshold = 6;
        double forceJacobianRankTolerance = 1e-5;
        double pulleyRadius = 0.0;
        bool workspaceMonitorEnabled = false;
        bool hasActualPose = false;
        int poseTimeoutCycles = 4;
        bool softwareWatchdogEnabled = false;
        qint64 mainThreadHeartbeatMs = -1;
        int mainThreadHeartbeatTimeoutMs = 1500;
        int mainThreadHeartbeatGraceMs = 250;
        int mainThreadHeartbeatTimeoutFaultCycles = 3;
        QString watchdogLogFilePath;
        double workspaceXMin = 0.0;
        double workspaceXMax = 0.0;
        double workspaceYMin = 0.0;
        double workspaceYMax = 0.0;
        double workspaceZMin = 0.0;
        double workspaceZMax = 0.0;
        double workspaceWarningMargin = 0.0;
        double workspaceSevereOverflow = 0.0;
        std::vector<double> actualPose;
        std::vector<AxisConfig> axes;
    };

    explicit SafetyMonitor(QObject* parent = nullptr);

    void setControlWorker(ControlWorker* worker);
    void setHardwareInterface(HardwareInterface* hardware);
    void setConfig(const Config& config);
    void updateMainThreadHeartbeat(qint64 timestampMs);

public slots:
    void start();
    void stop();
    void clearFaultLatch();
    void requestEmergencyStop(const QString& reason = QStringLiteral("软件急停触发"));
    void requestInjectedCableBreak(int sensorIndex);

signals:
    void faultDetected(int level, int code, QString summary, QString detail);

private:
    void evaluateSafety();
    Config currentConfig() const;
    void resetDynamicState(bool clearLatchedFault);
    void triggerFault(StopLevel level,
                      FaultCode code,
                      const QString& summary,
                      const QString& detail);
    int findAxisIndexBySensor(const Config& cfg, int sensorIndex) const;
    int computeForceJacobianRank(const Config& cfg,
                                 int excludedAxisIndex,
                                 QString* detail = nullptr) const;
    bool isCableBreakControllable(const Config& cfg,
                                  int brokenAxisIndex,
                                  int* rank = nullptr,
                                  QString* detail = nullptr) const;
    bool handleCableBreakFault(const Config& cfg,
                               int brokenAxisIndex,
                               const QString& summary,
                               const QString& detail);
    bool issueDirectEmergencyStop() const;
    void appendWatchdogEventLog(const Config& cfg,
                                const QString& eventType,
                                FaultCode code,
                                const QString& summary,
                                const QString& detail,
                                bool stopActionAttempted,
                                bool stopActionSucceeded,
                                const QString& note = QString()) const;
    void ensureAxisStateSize(int axisCount, int sensorCount);
    void resetWorkspaceState();

    ControlWorker* controlWorker = nullptr;
    HardwareInterface* hardwareInterface = nullptr;
    QTimer* timer = nullptr;

    mutable QMutex configMutex;
    Config config;

    bool faultLatched = false;
    bool manualEmergencyRequested = false;
    QString manualEmergencyReason;
    bool cableBreakControlledStopActive = false;
    int cableBreakControlledStopAxisIndex = -1;
    bool injectedCableBreakPending = false;
    int injectedCableBreakSensorIndex = -1;
    int mainThreadHeartbeatTimeoutCycles = 0;

    quint64 lastSnapshotSequence = 0;
    qint64 lastSnapshotUpdateMs = -1;
    bool hasSeenSnapshot = false;

    std::vector<double> previousForceSensorValue;
    std::vector<int> lowForceCycles;
    std::vector<int> highForceCycles;
    std::vector<int> overSpeedCycles;
    int workspaceMissingPoseCycles = 0;
    int workspaceExceededCycles = 0;
    bool workspaceNearBoundaryActive = false;
};

#endif // SAFETYMONITOR_H
