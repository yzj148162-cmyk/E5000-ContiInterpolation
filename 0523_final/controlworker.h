#ifndef CONTROLWORKER_H
#define CONTROLWORKER_H

#include <QObject>
#include <QMutex>
#include <QTimer>
#include <QVector>
#include <vector>

#include "hardwareinterface.h"
#include "pidhandler.h"

class ControlWorker : public QObject
{
    Q_OBJECT
public:
    enum class ForcePidOutputMode {
        Velocity = 0,
        Torque = 1
    };

    struct DiagnosticRawSample {
        qint64 wallClockMs = 0;
        qint64 intervalUs = 0;
        qint64 wallClockUs = 0;
    };

    struct SensorValueSample {
        qint64 wallClockMs = 0;
        std::vector<double> values;
        qint64 wallClockUs = 0;
    };

    struct TimingDiagnostics {
        quint64 controlLoopTickCount = 0;
        quint64 controlLoopIntervalCount = 0;
        qint64 controlLoopIntervalSumUs = 0;
        qint64 latestControlLoopIntervalUs = 0;
        quint64 sensorFrameCount = 0;
        quint64 sensorFrameIntervalCount = 0;
        qint64 sensorFrameIntervalSumUs = 0;
        qint64 latestSensorFrameIntervalUs = 0;
    };

    struct AxisConfig {
        bool isMotorAxis = false;
        bool forceControlEnabled = false;
        int sensorIndex = -1;
        double motorCof = 1.0;
        double forceMax = 0.0;
        double motorMin = 0.0;
        double motorMax = 0.0;
        double motorVelMax = 0.0;
    };

    struct Config {
        int axisCount = 0;
        int sensorCount = 0;
        double ctrlCycleMs = 10.0;
        double sensorSampleHz = 1000.0;
        double forceSensorLowPassCutoffHz = 0.0;
        bool systemRunning = false;
        bool useLeadshine = false;
        bool forceThreadEnabled = false;
        bool usePid = true;
        ForcePidOutputMode forcePidOutputMode = ForcePidOutputMode::Velocity;
        bool pvtActiveOrPaused = false;
        bool actualTorqueRealtimeEnabled = false;
        bool actualTorqueLimitEnabled = false;
        double actualTorqueLimitNm = 45.0;
        double initForce = 0.0;
        double forcePidDeadbandRatio = 0.10;
        double forceTorqueCommandLimitNm = 2.0;
        double forceTorqueCommandSlewRateNmPerSec = 20.0;
        double forceTorqueVelocityDampingNmPerVelocity = 0.0;
        std::vector<double> forceSensorHome;
        std::vector<double> expectedForce;
        std::vector<double> pidP;
        std::vector<double> pidI;
        std::vector<double> pidD;
        std::vector<AxisConfig> axes;
    };

    struct Snapshot {
        quint64 sequence = 0;
        bool forceThreadRunning = false;
        bool forceExpectedFromExternal = false;
        ForcePidOutputMode forcePidOutputMode = ForcePidOutputMode::Velocity;
        std::vector<double> motorAbsPos;
        std::vector<double> motorRelRawPos;
        std::vector<double> motorVel;
        std::vector<double> motorTorqueNm;
        std::vector<double> motorCommand;
        std::vector<double> forceSensorValue;
        std::vector<double> expectedForce;
        TimingDiagnostics timingDiagnostics;
    };

    explicit ControlWorker(HardwareInterface* hardware, QObject* parent = nullptr);

    void setConfig(const Config& config);
    void setExternalExpectedForce(const std::vector<double>& expectedForce);
    void clearExternalExpectedForce();
    Snapshot latestSnapshot() const;
    QVector<SensorValueSample> sensorValueHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    QVector<DiagnosticRawSample> sensorTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    QVector<DiagnosticRawSample> controlLoopTimingHistory(qint64 startWallClockMs, qint64 endWallClockMs) const;
    void resetTimingDiagnostics();

public slots:
    void start();
    void stop();

signals:
    void displayInfoSignal(std::string info, std::string type);
    void actualTorqueLimitExceeded(int axisIndex, double actualTorqueNm, double limitNm);

private:
    void controlLoop();
    void sensorLoop();
    Config currentConfig() const;
    std::vector<double> activeExpectedForce(const Config& config, bool& fromExternal) const;
    std::vector<double> applyForceSensorLowPass(const Config& config,
                                                const std::vector<double>& rawValue,
                                                double dtSec);
    void ensureTorqueCommandStateSize(int axisCount);
    void resetTorqueCommandState(int axisCount = 0);
    void stopActiveTorqueCommands(const Config& config);
    bool commandTorqueModeAxis(int axisIndex, double torqueNm);
    void updateSnapshot(const Config& config,
                        const std::vector<double>& motorAbsPos,
                        const std::vector<double>& motorRelRawPos,
                        const std::vector<double>& motorVel,
                        const std::vector<double>& motorTorqueNm,
                        const std::vector<double>& motorCommand,
                        const std::vector<double>& forceSensorValue,
                        const std::vector<double>& expectedForce,
                        bool forceThreadRunning,
                        bool expectedFromExternal);
    void throttledInfo(const QString& message, const std::string& type, int throttleMs = 1000);

    HardwareInterface* hardwareInterface = nullptr;
    QTimer* timer = nullptr;
    mutable QMutex configMutex;
    mutable QMutex snapshotMutex;
    mutable QMutex timingHistoryMutex;
    Config config;
    Snapshot snapshot;
    TimingDiagnostics timingDiagnostics;
    QVector<SensorValueSample> sensorValueRawHistory;
    QVector<DiagnosticRawSample> sensorFrameRawHistory;
    QVector<DiagnosticRawSample> controlLoopRawHistory;
    qint64 lastSensorValueHistoryTrimMs = 0;
    qint64 lastSensorFrameHistoryTrimMs = 0;
    qint64 lastControlLoopHistoryTrimMs = 0;
    std::vector<double> latestForceSensorValue;
    bool hasLatestForceSensorValue = false;
    std::vector<double> filteredForceSensorValue;
    bool hasFilteredForceSensorValue = false;
    std::vector<double> cachedMotorHome;
    std::vector<double> cachedMotorVel;
    std::vector<double> cachedMotorTorqueNm;
    std::vector<double> externalExpectedForce;
    bool hasExternalExpectedForce = false;
    bool lastForceThreadEnabled = false;
    bool softwareLimitEmergencyStopActive = false;
    bool actualTorqueLimitEmergencyStopActive = false;
    PIDHandler forcePid;
    PIDHandler forceTorquePid;
    std::vector<double> lastTorqueCommandNm;
    std::vector<bool> torqueCommandActive;
    ForcePidOutputMode lastForcePidOutputMode = ForcePidOutputMode::Velocity;
    qint64 lastInfoMs = 0;
    qint64 lastControlLoopTimestampUs = 0;
    qint64 nextControlLoopDueUs = 0;
    qint64 lastControlLoopSampleIntervalUs = 0;
    qint64 lastSensorFrameTimestampUs = 0;
    qint64 nextSensorReadDueUs = 0;
    qint64 lastSensorSampleIntervalUs = 0;
    qint64 lastMotorHomeRefreshUs = 0;
    qint64 lastMotorVelocityRefreshUs = 0;
    qint64 lastMotorTorqueRefreshUs = 0;
};

#endif // CONTROLWORKER_H
