#include "controlworker.h"

#include <QDateTime>
#include <QThread>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace {

qint64 monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

qint64 controlIntervalUsFromMs(double intervalMs)
{
    if(!std::isfinite(intervalMs) || intervalMs <= 0.0){
        return 10 * 1000;
    }
    return std::max<qint64>(500, static_cast<qint64>(std::round(intervalMs * 1000.0)));
}

constexpr double kForcePidIntegralLimit = 200.0;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr qint64 kForceSensorTraceFrameIntervalUs = 500;
constexpr qint64 kTraceDrivenWorkerIntervalUs = kForceSensorTraceFrameIntervalUs;
constexpr double kTraceDrivenWorkerIntervalMs =
        static_cast<double>(kTraceDrivenWorkerIntervalUs) / 1000.0;
constexpr double kTraceDrivenWorkerIntervalSec =
        static_cast<double>(kTraceDrivenWorkerIntervalUs) / 1000000.0;
constexpr double kDefaultForceTorqueCommandLimitNm = 2.0;
constexpr double kDefaultForceTorqueCommandSlewRateNmPerSec = 20.0;
constexpr double kTorqueCommandEpsilonNm = 1e-6;
constexpr qint64 kTorqueForceSensorTimeoutUs = 5 * 1000;
constexpr qint64 kDiagnosticRawRetentionMs = 6 * 60 * 1000;
constexpr qint64 kDiagnosticRawTrimIntervalMs = 1000;
constexpr qint64 kMotorHomeRefreshIntervalUs = 1000 * 1000;

int signOf(double value)
{
    if(value > 0.0){
        return 1;
    }
    if(value < 0.0){
        return -1;
    }
    return 0;
}

template<typename Sample>
void trimRawHistory(QVector<Sample>& history, qint64 cutoffMs)
{
    int removeCount = 0;
    while(removeCount < history.size() && history.at(removeCount).wallClockMs < cutoffMs){
        ++removeCount;
    }
    if(removeCount > 0){
        history.erase(history.begin(), history.begin() + removeCount);
    }
}

}

ControlWorker::ControlWorker(HardwareInterface* hardware, QObject* parent)
    : QObject(parent),
      hardwareInterface(hardware)
{
}

void ControlWorker::setConfig(const Config& newConfig)
{
    QMutexLocker locker(&configMutex);
    config = newConfig;
}

void ControlWorker::setExternalExpectedForce(const std::vector<double>& expectedForce)
{
    QMutexLocker locker(&configMutex);
    externalExpectedForce = expectedForce;
    hasExternalExpectedForce = !externalExpectedForce.empty();
}

void ControlWorker::clearExternalExpectedForce()
{
    QMutexLocker locker(&configMutex);
    externalExpectedForce.clear();
    hasExternalExpectedForce = false;
}

ControlWorker::Snapshot ControlWorker::latestSnapshot() const
{
    QMutexLocker locker(&snapshotMutex);
    return snapshot;
}

QVector<ControlWorker::SensorValueSample> ControlWorker::sensorValueHistory(qint64 startWallClockMs,
                                                                            qint64 endWallClockMs) const
{
    QMutexLocker locker(&timingHistoryMutex);
    QVector<SensorValueSample> filtered;
    filtered.reserve(sensorValueRawHistory.size());
    for(const SensorValueSample& sample : sensorValueRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<ControlWorker::DiagnosticRawSample> ControlWorker::sensorTimingHistory(qint64 startWallClockMs,
                                                                               qint64 endWallClockMs) const
{
    QMutexLocker locker(&timingHistoryMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(sensorFrameRawHistory.size());
    for(const DiagnosticRawSample& sample : sensorFrameRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<ControlWorker::DiagnosticRawSample> ControlWorker::controlLoopTimingHistory(qint64 startWallClockMs,
                                                                                    qint64 endWallClockMs) const
{
    QMutexLocker locker(&timingHistoryMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(controlLoopRawHistory.size());
    for(const DiagnosticRawSample& sample : controlLoopRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

void ControlWorker::resetTimingDiagnostics()
{
    QThread* targetThread = thread();
    if(targetThread && targetThread->isRunning() && QThread::currentThread() != targetThread){
        QMetaObject::invokeMethod(this, &ControlWorker::resetTimingDiagnostics, Qt::BlockingQueuedConnection);
        return;
    }

    timingDiagnostics = TimingDiagnostics{};
    lastControlLoopTimestampUs = 0;
    nextControlLoopDueUs = 0;
    lastControlLoopSampleIntervalUs = 0;
    lastSensorFrameTimestampUs = 0;
    nextSensorReadDueUs = 0;
    lastSensorSampleIntervalUs = 0;
    lastMotorHomeRefreshUs = 0;
    lastMotorVelocityRefreshUs = 0;
    lastMotorTorqueRefreshUs = 0;
    resetTorqueCommandState();
    lastSensorValueHistoryTrimMs = 0;
    lastControlLoopHistoryTrimMs = 0;
    lastSensorFrameHistoryTrimMs = 0;
    {
        QMutexLocker locker(&snapshotMutex);
        snapshot.timingDiagnostics = timingDiagnostics;
    }
    {
        QMutexLocker locker(&timingHistoryMutex);
        sensorValueRawHistory.clear();
        sensorFrameRawHistory.clear();
        controlLoopRawHistory.clear();
    }
}

void ControlWorker::start()
{
    if(!timer){
        timer = new QTimer(this);
        timer->setTimerType(Qt::PreciseTimer);
        connect(timer, &QTimer::timeout, this, &ControlWorker::controlLoop);
    }

    const qint64 nowUs = monotonicNowUs();
    nextControlLoopDueUs = nowUs;
    lastControlLoopSampleIntervalUs = kTraceDrivenWorkerIntervalUs;
    nextSensorReadDueUs = nowUs;
    lastSensorSampleIntervalUs = kTraceDrivenWorkerIntervalUs;
    timer->setInterval(0);
    timer->start();
}

void ControlWorker::stop()
{
    if(timer){
        timer->stop();
    }
    nextControlLoopDueUs = 0;
    lastControlLoopSampleIntervalUs = 0;
    nextSensorReadDueUs = 0;
    lastSensorSampleIntervalUs = 0;
    lastForceThreadEnabled = false;
    latestForceSensorValue.clear();
    hasLatestForceSensorValue = false;
    filteredForceSensorValue.clear();
    hasFilteredForceSensorValue = false;
    cachedMotorHome.clear();
    cachedMotorVel.clear();
    cachedMotorTorqueNm.clear();
    resetTorqueCommandState();
    lastMotorVelocityRefreshUs = 0;
    lastMotorTorqueRefreshUs = 0;
}

ControlWorker::Config ControlWorker::currentConfig() const
{
    QMutexLocker locker(&configMutex);
    return config;
}

std::vector<double> ControlWorker::activeExpectedForce(const Config& cfg, bool& fromExternal) const
{
    QMutexLocker locker(&configMutex);
    if(hasExternalExpectedForce && static_cast<int>(externalExpectedForce.size()) == cfg.sensorCount){
        fromExternal = true;
        return externalExpectedForce;
    }
    fromExternal = false;
    return cfg.expectedForce;
}

std::vector<double> ControlWorker::applyForceSensorLowPass(const Config& cfg,
                                                           const std::vector<double>& rawValue,
                                                           double dtSec)
{
    if(!std::isfinite(cfg.forceSensorLowPassCutoffHz) || cfg.forceSensorLowPassCutoffHz <= 0.0){
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return rawValue;
    }

    double sampleHz = std::min(std::max(cfg.sensorSampleHz, 10.0), 2000.0);
    if(std::isfinite(dtSec) && dtSec > 0.0){
        sampleHz = std::min(std::max(1.0 / dtSec, 10.0), 2000.0);
    }
    const double cutoffHz = std::min(cfg.forceSensorLowPassCutoffHz, sampleHz * 0.45);
    if(!std::isfinite(cutoffHz) || cutoffHz <= 0.0){
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return rawValue;
    }

    if(!std::isfinite(dtSec) || dtSec <= 0.0){
        dtSec = 1.0 / sampleHz;
    }

    const double omegaDt = kTwoPi * cutoffHz * dtSec;
    const double alpha = std::min(1.0, std::max(0.0, omegaDt / (1.0 + omegaDt)));

    if(!hasFilteredForceSensorValue || filteredForceSensorValue.size() != rawValue.size()){
        filteredForceSensorValue = rawValue;
        hasFilteredForceSensorValue = true;
        return filteredForceSensorValue;
    }

    for(size_t i = 0; i < rawValue.size(); ++i){
        const double raw = rawValue[i];
        if(!std::isfinite(raw)){
            filteredForceSensorValue[i] = raw;
            continue;
        }
        if(!std::isfinite(filteredForceSensorValue[i])){
            filteredForceSensorValue[i] = raw;
            continue;
        }
        filteredForceSensorValue[i] += alpha * (raw - filteredForceSensorValue[i]);
    }

    return filteredForceSensorValue;
}

void ControlWorker::ensureTorqueCommandStateSize(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    if(static_cast<int>(lastTorqueCommandNm.size()) < axisCount){
        lastTorqueCommandNm.resize(axisCount, 0.0);
    }
    if(static_cast<int>(torqueCommandActive.size()) < axisCount){
        torqueCommandActive.resize(axisCount, false);
    }
}

void ControlWorker::resetTorqueCommandState(int axisCount)
{
    axisCount = std::max(axisCount, 0);
    lastTorqueCommandNm.assign(axisCount, 0.0);
    torqueCommandActive.assign(axisCount, false);
}

void ControlWorker::stopActiveTorqueCommands(const Config& cfg)
{
    ensureTorqueCommandStateSize(cfg.axisCount);
    if(!hardwareInterface){
        resetTorqueCommandState(cfg.axisCount);
        return;
    }

    const int axisCount = static_cast<int>(torqueCommandActive.size());
    for(int axisIndex = 0; axisIndex < axisCount; ++axisIndex){
        if(!torqueCommandActive[axisIndex]){
            continue;
        }
        hardwareInterface->motorStop(axisIndex);
        torqueCommandActive[axisIndex] = false;
        lastTorqueCommandNm[axisIndex] = 0.0;
    }
}

bool ControlWorker::commandTorqueModeAxis(int axisIndex, double torqueNm)
{
    if(!hardwareInterface || axisIndex < 0){
        return false;
    }
    ensureTorqueCommandStateSize(axisIndex + 1);

    if(!std::isfinite(torqueNm) || std::fabs(torqueNm) <= kTorqueCommandEpsilonNm){
        if(torqueCommandActive[axisIndex]){
            hardwareInterface->motorStop(axisIndex);
        }
        torqueCommandActive[axisIndex] = false;
        lastTorqueCommandNm[axisIndex] = 0.0;
        return true;
    }

    const bool shouldStart =
            !torqueCommandActive[axisIndex] ||
            signOf(lastTorqueCommandNm[axisIndex]) != signOf(torqueNm);
    if(shouldStart && torqueCommandActive[axisIndex]){
        hardwareInterface->motorStop(axisIndex);
    }
    const bool ok = shouldStart ?
                hardwareInterface->motorTorqueStart(axisIndex, torqueNm) :
                hardwareInterface->motorTorqueChange(axisIndex, torqueNm);
    if(ok){
        torqueCommandActive[axisIndex] = true;
        lastTorqueCommandNm[axisIndex] = torqueNm;
    }
    else{
        hardwareInterface->motorStop(axisIndex);
        torqueCommandActive[axisIndex] = false;
        lastTorqueCommandNm[axisIndex] = 0.0;
    }
    return ok;
}

void ControlWorker::controlLoop()
{
    const qint64 loopNowUs = monotonicNowUs();
    if(lastControlLoopSampleIntervalUs != kTraceDrivenWorkerIntervalUs){
        lastControlLoopSampleIntervalUs = kTraceDrivenWorkerIntervalUs;
        nextControlLoopDueUs = loopNowUs;
    }
    if(nextControlLoopDueUs <= 0){
        nextControlLoopDueUs = loopNowUs;
    }
    if(loopNowUs < nextControlLoopDueUs){
        return;
    }
    while(nextControlLoopDueUs <= loopNowUs){
        nextControlLoopDueUs += kTraceDrivenWorkerIntervalUs;
    }

    if(!hardwareInterface){
        return;
    }

    const qint64 loopWallClockMs = QDateTime::currentMSecsSinceEpoch();
    timingDiagnostics.controlLoopTickCount++;
    if(lastControlLoopTimestampUs > 0){
        const qint64 dtUs = std::max<qint64>(0, loopNowUs - lastControlLoopTimestampUs);
        timingDiagnostics.controlLoopIntervalCount++;
        timingDiagnostics.controlLoopIntervalSumUs += dtUs;
        timingDiagnostics.latestControlLoopIntervalUs = dtUs;
        {
            QMutexLocker locker(&timingHistoryMutex);
            controlLoopRawHistory.append({loopWallClockMs, dtUs});
            if(loopWallClockMs - lastControlLoopHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistory(controlLoopRawHistory, loopWallClockMs - kDiagnosticRawRetentionMs);
                lastControlLoopHistoryTrimMs = loopWallClockMs;
            }
        }
    }
    lastControlLoopTimestampUs = loopNowUs;

    const Config cfg = currentConfig();
    ensureTorqueCommandStateSize(cfg.axisCount);
    if(cfg.forcePidOutputMode != lastForcePidOutputMode){
        forcePid.resetAll();
        forceTorquePid.resetAll();
        stopActiveTorqueCommands(cfg);
        if(!cfg.pvtActiveOrPaused){
            for(int axisIndex = 0;
                axisIndex < std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
                ++axisIndex){
                const AxisConfig& axis = cfg.axes[axisIndex];
                if(axis.isMotorAxis && axis.forceControlEnabled){
                    hardwareInterface->motorStop(axisIndex);
                }
            }
        }
        resetTorqueCommandState(cfg.axisCount);
        lastForcePidOutputMode = cfg.forcePidOutputMode;
    }
    const bool leadshineConnected = cfg.useLeadshine && hardwareInterface->isLSConnected();
    if(timer){
        if(timer->interval() != 0){
            timer->setInterval(0);
        }
    }

    std::vector<double> motorAbsPos;
    std::vector<double> motorRelRawPos;
    std::vector<double> motorVel;
    std::vector<double> motorTorqueNm;
    std::vector<double> motorCommand(std::max(cfg.axisCount, 0), 0.0);
    std::vector<double> forceSensorValue;
    std::vector<double> expectedForce;
    bool expectedFromExternal = false;

    if(leadshineConnected){
        if(cfg.sensorCount > 0){
            sensorLoop();
        }

        motorAbsPos = hardwareInterface->getAllMotorPos();
        const int axisCount = std::min(cfg.axisCount, static_cast<int>(motorAbsPos.size()));
        const qint64 nonTraceRefreshIntervalUs = controlIntervalUsFromMs(cfg.ctrlCycleMs);
        if(static_cast<int>(cachedMotorHome.size()) < axisCount ||
                loopNowUs - lastMotorHomeRefreshUs >= kMotorHomeRefreshIntervalUs){
            cachedMotorHome = hardwareInterface->getAllMotorHome();
            lastMotorHomeRefreshUs = loopNowUs;
        }
        const std::vector<double> motorHome = cachedMotorHome;
        motorRelRawPos.resize(axisCount, 0.0);
        for(int i=0; i<axisCount; ++i){
            motorRelRawPos[i] = motorAbsPos[i];
            if(i < static_cast<int>(motorHome.size())){
                motorRelRawPos[i] -= motorHome[i];
            }
        }
        if(static_cast<int>(cachedMotorVel.size()) < axisCount ||
                loopNowUs - lastMotorVelocityRefreshUs >= nonTraceRefreshIntervalUs){
            cachedMotorVel = hardwareInterface->getAllMotorVel();
            lastMotorVelocityRefreshUs = loopNowUs;
        }
        motorVel = cachedMotorVel;
        if(static_cast<int>(motorVel.size()) > axisCount){
            motorVel.resize(axisCount);
        }
        if(cfg.actualTorqueRealtimeEnabled ||
                cfg.actualTorqueLimitEnabled ||
                cfg.forcePidOutputMode == ForcePidOutputMode::Torque){
            if(static_cast<int>(cachedMotorTorqueNm.size()) < axisCount ||
                    loopNowUs - lastMotorTorqueRefreshUs >= nonTraceRefreshIntervalUs){
                cachedMotorTorqueNm = hardwareInterface->getAllMotorTorqueNm();
                lastMotorTorqueRefreshUs = loopNowUs;
            }
            motorTorqueNm = cachedMotorTorqueNm;
            if(static_cast<int>(motorTorqueNm.size()) > axisCount){
                motorTorqueNm.resize(axisCount);
            }
        }

        if(hasLatestForceSensorValue){
            forceSensorValue = latestForceSensorValue;
        }else if(cfg.sensorCount > 0){
            sensorLoop();
            if(hasLatestForceSensorValue){
                forceSensorValue = latestForceSensorValue;
            }
        }
        if(static_cast<int>(forceSensorValue.size()) < cfg.sensorCount){
            forceSensorValue.resize(cfg.sensorCount, 0.0);
        }
    }

    expectedForce = activeExpectedForce(cfg, expectedFromExternal);
    if(static_cast<int>(expectedForce.size()) < cfg.sensorCount){
        expectedForce.resize(cfg.sensorCount, 0.0);
    }
    std::vector<bool> expectedClamped(cfg.sensorCount, false);
    for(const AxisConfig& axis : cfg.axes){
        if(axis.sensorIndex < 0 || axis.sensorIndex >= cfg.sensorCount || expectedClamped[axis.sensorIndex]){
            continue;
        }
        if(axis.forceMax > 1e-5){
            expectedForce[axis.sensorIndex] = std::min(expectedForce[axis.sensorIndex], axis.forceMax);
        }
        expectedForce[axis.sensorIndex] = std::max(expectedForce[axis.sensorIndex], cfg.initForce);
        expectedClamped[axis.sensorIndex] = true;
    }

    bool hasForceAxis = false;
    for(const AxisConfig& axis : cfg.axes){
        hasForceAxis = hasForceAxis || axis.forceControlEnabled;
    }
    const bool torquePidBlockedByPvt =
            cfg.forcePidOutputMode == ForcePidOutputMode::Torque &&
            cfg.pvtActiveOrPaused;
    if(torquePidBlockedByPvt && cfg.forceThreadEnabled){
        throttledInfo(QStringLiteral("Torque PID force control is disabled while a PVT trajectory is active."),
                      "warning");
    }
    const bool forceThreadRunning = cfg.systemRunning &&
            leadshineConnected &&
            cfg.forceThreadEnabled &&
            hasForceAxis &&
            !torquePidBlockedByPvt;
    if(!forceThreadRunning){
        forcePid.resetAll();
        forceTorquePid.resetAll();
        stopActiveTorqueCommands(cfg);
    }

    bool actualTorqueLimitEmergencyDetected = false;
    int actualTorqueLimitAxisIndex = -1;
    double actualTorqueLimitValueNm = 0.0;
    double actualTorqueLimitNm = cfg.actualTorqueLimitNm;
    const bool torquePidMode = cfg.forcePidOutputMode == ForcePidOutputMode::Torque;
    if(torquePidMode &&
            std::isfinite(cfg.forceTorqueCommandLimitNm) &&
            cfg.forceTorqueCommandLimitNm > 0.0){
        const double torqueModeFeedbackLimit =
                std::max(cfg.forceTorqueCommandLimitNm + 0.5,
                         cfg.forceTorqueCommandLimitNm * 1.5);
        actualTorqueLimitNm =
                std::isfinite(actualTorqueLimitNm) && actualTorqueLimitNm > 0.0 ?
                    std::min(actualTorqueLimitNm, torqueModeFeedbackLimit) :
                    torqueModeFeedbackLimit;
    }
    const bool actualTorqueProtectionEnabled =
            cfg.actualTorqueLimitEnabled || (torquePidMode && forceThreadRunning);
    if(actualTorqueProtectionEnabled &&
            leadshineConnected &&
            std::isfinite(actualTorqueLimitNm) &&
            actualTorqueLimitNm > 0.0){
        const int axisCount = std::min({cfg.axisCount,
                                        static_cast<int>(cfg.axes.size()),
                                        static_cast<int>(motorTorqueNm.size())});
        for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(!axis.isMotorAxis){
                continue;
            }

            const double torqueNm = motorTorqueNm[axisIndex];
            if(!std::isfinite(torqueNm)){
                continue;
            }

            const double absTorqueNm = std::fabs(torqueNm);
            if(absTorqueNm >= actualTorqueLimitNm){
                actualTorqueLimitEmergencyDetected = true;
                if(actualTorqueLimitAxisIndex < 0 ||
                        absTorqueNm > std::fabs(actualTorqueLimitValueNm)){
                    actualTorqueLimitAxisIndex = axisIndex;
                    actualTorqueLimitValueNm = torqueNm;
                }
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
        }
    }

    if(actualTorqueLimitEmergencyDetected){
        throttledInfo(QStringLiteral("Software emergency stop: motor%1 actual torque %2 Nm reached limit %3 Nm")
                      .arg(actualTorqueLimitAxisIndex)
                      .arg(actualTorqueLimitValueNm, 0, 'f', 3)
                      .arg(actualTorqueLimitNm, 0, 'f', 3),
                      "error");
        if(!actualTorqueLimitEmergencyStopActive){
            hardwareInterface->emergencyStopAll();
            resetTorqueCommandState(cfg.axisCount);
            actualTorqueLimitEmergencyStopActive = true;
            emit actualTorqueLimitExceeded(actualTorqueLimitAxisIndex,
                                           actualTorqueLimitValueNm,
                                           actualTorqueLimitNm);
        }
    }
    else{
        actualTorqueLimitEmergencyStopActive = false;
    }

    if(!actualTorqueLimitEmergencyDetected &&
            forceThreadRunning &&
            static_cast<int>(forceSensorValue.size()) >= cfg.sensorCount){
        const int sensorCount = std::max(cfg.sensorCount, 0);
        std::vector<double> pidOutput(sensorCount, 0.0);
        if(cfg.usePid){
            std::vector<double> pidForceSensorValue = forceSensorValue;
            std::vector<double> pidExpectedForce = expectedForce;
            pidForceSensorValue.resize(sensorCount, 0.0);
            pidExpectedForce.resize(sensorCount, 0.0);
            std::vector<double> pidP = cfg.pidP;
            std::vector<double> pidI = cfg.pidI;
            std::vector<double> pidD = cfg.pidD;
            pidP.resize(sensorCount, 0.0);
            pidI.resize(sensorCount, 0.0);
            pidD.resize(sensorCount, 0.0);
            const bool torqueMode = cfg.forcePidOutputMode == ForcePidOutputMode::Torque;
            const double torqueLimit = std::max(
                        0.0,
                        std::isfinite(cfg.forceTorqueCommandLimitNm) ?
                            cfg.forceTorqueCommandLimitNm :
                            kDefaultForceTorqueCommandLimitNm);
            PIDHandler& pid = torqueMode ? forceTorquePid : forcePid;
            pid.updatePara(pidP, pidI, pidD, kTraceDrivenWorkerIntervalMs);
            pid.updateTustinPara(std::vector<double>(sensorCount, 0.0),
                                 std::vector<double>(sensorCount, -kForcePidIntegralLimit),
                                 std::vector<double>(sensorCount, kForcePidIntegralLimit),
                                 std::vector<double>(sensorCount, torqueMode ? -torqueLimit : -720.0),
                                 std::vector<double>(sensorCount, torqueMode ? torqueLimit : 720.0));
            pidOutput = pid.updateWithRelativeDeadband(pidForceSensorValue,
                                                       pidExpectedForce,
                                                       cfg.forcePidDeadbandRatio);
            if(static_cast<int>(pidOutput.size()) < sensorCount){
                pidOutput.resize(sensorCount, 0.0);
            }
        }

        for(int axisIndex=0; axisIndex<std::min(cfg.axisCount, static_cast<int>(cfg.axes.size())); ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(!axis.isMotorAxis || axis.sensorIndex < 0 || axis.sensorIndex >= cfg.sensorCount){
                continue;
            }

            if(axis.forceControlEnabled){
                const double curForce = forceSensorValue[axis.sensorIndex];
                const double hardwareDirection = axis.motorCof < 0.0 ? -1.0 : 1.0;
                if(cfg.forcePidOutputMode == ForcePidOutputMode::Torque){
                    const double torqueLimit = std::max(
                                0.0,
                                std::isfinite(cfg.forceTorqueCommandLimitNm) ?
                                    cfg.forceTorqueCommandLimitNm :
                                    kDefaultForceTorqueCommandLimitNm);
                    const double torqueSlewRate = std::max(
                                0.0,
                                std::isfinite(cfg.forceTorqueCommandSlewRateNmPerSec) ?
                                    cfg.forceTorqueCommandSlewRateNmPerSec :
                                    kDefaultForceTorqueCommandSlewRateNmPerSec);
                    const double velocityDamping = std::max(
                                0.0,
                                std::isfinite(cfg.forceTorqueVelocityDampingNmPerVelocity) ?
                                    cfg.forceTorqueVelocityDampingNmPerVelocity :
                                    0.0);
                    const bool hasValidRange = axisIndex < static_cast<int>(motorRelRawPos.size()) &&
                            std::isfinite(axis.motorMin) &&
                            std::isfinite(axis.motorMax) &&
                            axis.motorMax > axis.motorMin;
                    const bool forceSensorTimedOut =
                            lastSensorFrameTimestampUs <= 0 ||
                            loopNowUs - lastSensorFrameTimestampUs > kTorqueForceSensorTimeoutUs;

                    if(!std::isfinite(curForce)){
                        commandTorqueModeAxis(axisIndex, 0.0);
                        throttledInfo(QString("Motor%1 torque PID stopped: force feedback is invalid.")
                                      .arg(axisIndex), "error");
                    }
                    else if(forceSensorTimedOut){
                        commandTorqueModeAxis(axisIndex, 0.0);
                        throttledInfo(QString("Motor%1 torque PID stopped: force trace feedback timed out.")
                                      .arg(axisIndex), "error");
                    }
                    else if(hasValidRange &&
                            (motorRelRawPos[axisIndex] <= axis.motorMin ||
                             motorRelRawPos[axisIndex] >= axis.motorMax)){
                        commandTorqueModeAxis(axisIndex, 0.0);
                        throttledInfo(QString("Motor%1 torque PID stopped: software position limit reached.")
                                      .arg(axisIndex), "error");
                    }
                    else if(axis.motorVelMax > 0.0 &&
                            (axisIndex >= static_cast<int>(motorVel.size()) ||
                             !std::isfinite(motorVel[axisIndex]) ||
                             std::fabs(motorVel[axisIndex]) > axis.motorVelMax)){
                        commandTorqueModeAxis(axisIndex, 0.0);
                        throttledInfo(QString("Motor%1 torque PID stopped: velocity feedback reached limit.")
                                      .arg(axisIndex), "error");
                    }
                    else if(axis.forceMax > 1e-5 && curForce > axis.forceMax){
                        commandTorqueModeAxis(axisIndex, 0.0);
                        throttledInfo(QString("Motor%1 torque PID stopped: force reached limit.")
                                      .arg(axisIndex), "error");
                    }
                    else if(torqueLimit <= kTorqueCommandEpsilonNm){
                        commandTorqueModeAxis(axisIndex, 0.0);
                        throttledInfo(QString("Motor%1 torque PID stopped: torque command limit is zero.")
                                      .arg(axisIndex), "warning");
                    }
                    else{
                        double cmdTorque = cfg.usePid ? pidOutput[axis.sensorIndex] : 0.0;
                        cmdTorque *= hardwareDirection;
                        if(velocityDamping > 0.0 &&
                                axisIndex < static_cast<int>(motorVel.size()) &&
                                std::isfinite(motorVel[axisIndex])){
                            cmdTorque -= velocityDamping * motorVel[axisIndex];
                        }
                        cmdTorque = std::min(std::max(cmdTorque, -torqueLimit), torqueLimit);
                        if(torqueSlewRate > 0.0 && axisIndex < static_cast<int>(lastTorqueCommandNm.size())){
                            const double lastTorque =
                                    torqueCommandActive[axisIndex] ? lastTorqueCommandNm[axisIndex] : 0.0;
                            const double maxStep = torqueSlewRate * kTraceDrivenWorkerIntervalSec;
                            cmdTorque = std::min(std::max(cmdTorque,
                                                          lastTorque - maxStep),
                                                 lastTorque + maxStep);
                        }
                        if(!commandTorqueModeAxis(axisIndex, cmdTorque)){
                            throttledInfo(QString("Motor%1 torque PID command failed.")
                                          .arg(axisIndex), "error");
                            cmdTorque = 0.0;
                        }
                        if(axisIndex < static_cast<int>(motorCommand.size())){
                            motorCommand[axisIndex] = cmdTorque;
                        }
                    }
                }
                else{
                    if(axisIndex < static_cast<int>(torqueCommandActive.size()) &&
                            torqueCommandActive[axisIndex]){
                        commandTorqueModeAxis(axisIndex, 0.0);
                    }
                    if(axis.forceMax > 1e-5 && curForce > axis.forceMax){
                        const double safeVel = -hardwareDirection * std::max(axis.motorVelMax / 2.0, 0.0);
                        hardwareInterface->motorVel(axisIndex, safeVel);
                        if(axisIndex < static_cast<int>(motorCommand.size())){
                            motorCommand[axisIndex] = safeVel;
                        }
                        throttledInfo(QString("Motor%1 force reach limited range!").arg(axisIndex), "error");
                    }
                    else{
                        double cmdVel = cfg.usePid ? pidOutput[axis.sensorIndex] : 0.0;
                        cmdVel *= hardwareDirection;
                        const double maxVel = std::max(axis.motorVelMax, 0.0);
                        cmdVel = std::min(std::max(cmdVel, -maxVel), maxVel);
                        hardwareInterface->motorVel(axisIndex, cmdVel);
                        if(axisIndex < static_cast<int>(motorCommand.size())){
                            motorCommand[axisIndex] = cmdVel;
                        }
                    }
                }

            }
            else if(!cfg.pvtActiveOrPaused){
                commandTorqueModeAxis(axisIndex, 0.0);
                hardwareInterface->motorStop(axisIndex);
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
        }
    }

    else if(!actualTorqueLimitEmergencyDetected && lastForceThreadEnabled && !cfg.pvtActiveOrPaused){
        for(int axisIndex=0; axisIndex<std::min(cfg.axisCount, static_cast<int>(cfg.axes.size())); ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(axis.isMotorAxis && axis.sensorIndex >= 0){
                commandTorqueModeAxis(axisIndex, 0.0);
                hardwareInterface->motorStop(axisIndex);
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
        }
    }

    bool softwareLimitEmergencyDetected = false;
    if(leadshineConnected){
        const int axisCount = std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
        for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
            const AxisConfig& axis = cfg.axes[axisIndex];
            if(!axis.isMotorAxis || axisIndex >= static_cast<int>(motorRelRawPos.size())){
                continue;
            }

            const double rawPos = motorRelRawPos[axisIndex];
            const bool hasValidRange = std::isfinite(axis.motorMin) &&
                    std::isfinite(axis.motorMax) &&
                    axis.motorMax > axis.motorMin;
            if(!std::isfinite(rawPos)){
                softwareLimitEmergencyDetected = true;
                throttledInfo(QString("Software emergency stop: motor %1 position feedback is invalid.")
                              .arg(axisIndex), "error");
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
                continue;
            }

            if(!hasValidRange){
                continue;
            }

            const bool reachedOrExceededLimit = rawPos > axis.motorMax || rawPos < axis.motorMin;
            if(reachedOrExceededLimit){
                softwareLimitEmergencyDetected = true;
                throttledInfo(QString("软件急停：电机%1当前位置%2已达到或超过软件位置限位[%3, %4]")
                              .arg(axisIndex)
                              .arg(rawPos, 0, 'f', 6)
                              .arg(axis.motorMin, 0, 'f', 6)
                              .arg(axis.motorMax, 0, 'f', 6),
                              "error");
                if(axisIndex < static_cast<int>(motorCommand.size())){
                    motorCommand[axisIndex] = 0.0;
                }
            }
        }
    }
    if(softwareLimitEmergencyDetected){
        if(!softwareLimitEmergencyStopActive){
            hardwareInterface->emergencyStopAll();
            resetTorqueCommandState(cfg.axisCount);
            softwareLimitEmergencyStopActive = true;
        }
    }
    else{
        softwareLimitEmergencyStopActive = false;
    }

    lastForceThreadEnabled = forceThreadRunning;
    updateSnapshot(cfg, motorAbsPos, motorRelRawPos, motorVel, motorTorqueNm, motorCommand, forceSensorValue, expectedForce,
                   forceThreadRunning, expectedFromExternal);
}

void ControlWorker::sensorLoop()
{
    if(!hardwareInterface){
        latestForceSensorValue.clear();
        hasLatestForceSensorValue = false;
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return;
    }

    const Config cfg = currentConfig();
    if(!cfg.useLeadshine || cfg.sensorCount <= 0){
        nextSensorReadDueUs = 0;
        lastSensorSampleIntervalUs = 0;
        latestForceSensorValue.clear();
        hasLatestForceSensorValue = false;
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return;
    }

    const qint64 targetIntervalUs = kTraceDrivenWorkerIntervalUs;
    const qint64 loopEntryUs = monotonicNowUs();
    if(lastSensorSampleIntervalUs != targetIntervalUs){
        lastSensorSampleIntervalUs = targetIntervalUs;
        nextSensorReadDueUs = loopEntryUs;
    }
    if(nextSensorReadDueUs <= 0){
        nextSensorReadDueUs = loopEntryUs;
    }
    if(loopEntryUs < nextSensorReadDueUs){
        return;
    }

    std::vector<HardwareInterface::ForceSensorTraceSample> traceSamples =
            hardwareInterface->readForceSensorDataTraceSamples();
    const qint64 readCompleteUs = monotonicNowUs();
    const qint64 readCompleteWallClockUs = QDateTime::currentMSecsSinceEpoch() * 1000;
    if(!traceSamples.empty()){
        while(nextSensorReadDueUs <= readCompleteUs){
            nextSensorReadDueUs += targetIntervalUs;
        }

        const qint64 traceFrameIntervalUs = kForceSensorTraceFrameIntervalUs;
        const qint64 firstSampleUs = lastSensorFrameTimestampUs > 0 ?
                    lastSensorFrameTimestampUs + traceFrameIntervalUs :
                    readCompleteUs - static_cast<qint64>(traceSamples.size() - 1) * traceFrameIntervalUs;
        const qint64 firstWallClockUs =
                readCompleteWallClockUs - static_cast<qint64>(traceSamples.size() - 1) * traceFrameIntervalUs;
        bool appendedHistory = false;

        for(std::size_t sampleIndex = 0; sampleIndex < traceSamples.size(); ++sampleIndex){
            std::vector<double> forceSensorValue = std::move(traceSamples[sampleIndex].values);
            if(forceSensorValue.empty()){
                continue;
            }
            if(static_cast<int>(forceSensorValue.size()) < cfg.sensorCount){
                forceSensorValue.resize(cfg.sensorCount, 0.0);
            }

            const qint64 sampleUs =
                    firstSampleUs + static_cast<qint64>(sampleIndex) * traceFrameIntervalUs;
            const qint64 sampleWallClockUs =
                    firstWallClockUs + static_cast<qint64>(sampleIndex) * traceFrameIntervalUs;
            const qint64 sampleWallClockMs = sampleWallClockUs / 1000;
            qint64 sensorDtUs = 0;

            timingDiagnostics.sensorFrameCount++;
            if(lastSensorFrameTimestampUs > 0){
                sensorDtUs = std::max<qint64>(0, sampleUs - lastSensorFrameTimestampUs);
                timingDiagnostics.sensorFrameIntervalCount++;
                timingDiagnostics.sensorFrameIntervalSumUs += sensorDtUs;
                timingDiagnostics.latestSensorFrameIntervalUs = sensorDtUs;
                {
                    QMutexLocker locker(&timingHistoryMutex);
                    sensorFrameRawHistory.append(DiagnosticRawSample{sampleWallClockMs,
                                                                     sensorDtUs,
                                                                     sampleWallClockUs});
                    appendedHistory = true;
                }
            }
            lastSensorFrameTimestampUs = sampleUs;

            const double filterDtSec =
                    sensorDtUs > 0 ? static_cast<double>(sensorDtUs) / 1000000.0 : 0.0;
            forceSensorValue = applyForceSensorLowPass(cfg, forceSensorValue, filterDtSec);
            latestForceSensorValue = std::move(forceSensorValue);
            hasLatestForceSensorValue = true;
            {
                QMutexLocker locker(&timingHistoryMutex);
                sensorValueRawHistory.append(SensorValueSample{sampleWallClockMs,
                                                               latestForceSensorValue,
                                                               sampleWallClockUs});
                appendedHistory = true;
            }
        }

        if(appendedHistory){
            const qint64 trimWallClockMs = readCompleteWallClockUs / 1000;
            QMutexLocker locker(&timingHistoryMutex);
            if(trimWallClockMs - lastSensorFrameHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistory(sensorFrameRawHistory, trimWallClockMs - kDiagnosticRawRetentionMs);
                lastSensorFrameHistoryTrimMs = trimWallClockMs;
            }
            if(trimWallClockMs - lastSensorValueHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistory(sensorValueRawHistory, trimWallClockMs - kDiagnosticRawRetentionMs);
                lastSensorValueHistoryTrimMs = trimWallClockMs;
            }
        }
        return;
    }

    const HardwareInterface::ForceSensorReadResult forceSensorRead =
            hardwareInterface->readForceSensorDataCachedResult(1);
    if(forceSensorRead.fromTrace){
        while(nextSensorReadDueUs <= readCompleteUs){
            nextSensorReadDueUs += targetIntervalUs;
        }
        return;
    }
    while(nextSensorReadDueUs <= readCompleteUs){
        nextSensorReadDueUs += targetIntervalUs;
    }

    std::vector<double> forceSensorValue = forceSensorRead.values;
    if(forceSensorValue.empty()){
        latestForceSensorValue.clear();
        hasLatestForceSensorValue = false;
        filteredForceSensorValue.clear();
        hasFilteredForceSensorValue = false;
        return;
    }
    if(static_cast<int>(forceSensorValue.size()) < cfg.sensorCount){
        forceSensorValue.resize(cfg.sensorCount, 0.0);
    }

    const qint64 sensorNowUs = readCompleteUs;
    const qint64 sensorWallClockMs = QDateTime::currentMSecsSinceEpoch();
    qint64 sensorDtUs = 0;
    const bool hasNewSensorRead = forceSensorRead.frameCount > 0;
    if(hasNewSensorRead){
        timingDiagnostics.sensorFrameCount++;
    }
    if(hasNewSensorRead && lastSensorFrameTimestampUs > 0){
        const qint64 dtUs = std::max<qint64>(0, sensorNowUs - lastSensorFrameTimestampUs);
        sensorDtUs = dtUs;
        timingDiagnostics.sensorFrameIntervalCount++;
        timingDiagnostics.sensorFrameIntervalSumUs += dtUs;
        timingDiagnostics.latestSensorFrameIntervalUs = dtUs;
        {
            QMutexLocker locker(&timingHistoryMutex);
            sensorFrameRawHistory.append(DiagnosticRawSample{sensorWallClockMs, dtUs, 0});
            if(sensorWallClockMs - lastSensorFrameHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistory(sensorFrameRawHistory, sensorWallClockMs - kDiagnosticRawRetentionMs);
                lastSensorFrameHistoryTrimMs = sensorWallClockMs;
            }
        }
    }
    if(hasNewSensorRead){
        lastSensorFrameTimestampUs = sensorNowUs;
    }

    const double filterDtSec = sensorDtUs > 0 ? static_cast<double>(sensorDtUs) / 1000000.0 : 0.0;
    forceSensorValue = applyForceSensorLowPass(cfg, forceSensorValue, filterDtSec);
    latestForceSensorValue = std::move(forceSensorValue);
    hasLatestForceSensorValue = true;
    {
        QMutexLocker locker(&timingHistoryMutex);
        if(hasNewSensorRead){
            sensorValueRawHistory.append(SensorValueSample{sensorWallClockMs, latestForceSensorValue, 0});
        }
        if(sensorWallClockMs - lastSensorValueHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
            trimRawHistory(sensorValueRawHistory, sensorWallClockMs - kDiagnosticRawRetentionMs);
            lastSensorValueHistoryTrimMs = sensorWallClockMs;
        }
    }
}

void ControlWorker::updateSnapshot(const Config& cfg,
                                   const std::vector<double>& motorAbsPos,
                                   const std::vector<double>& motorRelRawPos,
                                   const std::vector<double>& motorVel,
                                   const std::vector<double>& motorTorqueNm,
                                   const std::vector<double>& motorCommand,
                                   const std::vector<double>& forceSensorValue,
                                   const std::vector<double>& expectedForce,
                                   bool forceThreadRunning,
                                   bool expectedFromExternal)
{
    QMutexLocker locker(&snapshotMutex);
    snapshot.sequence++;
    snapshot.motorAbsPos = motorAbsPos;
    snapshot.motorRelRawPos = motorRelRawPos;
    snapshot.motorVel = motorVel;
    snapshot.motorTorqueNm = motorTorqueNm;
    snapshot.motorCommand = motorCommand;
    snapshot.forceSensorValue = forceSensorValue;
    snapshot.expectedForce = expectedForce;
    snapshot.forceThreadRunning = forceThreadRunning;
    snapshot.forceExpectedFromExternal = expectedFromExternal;
    snapshot.forcePidOutputMode = cfg.forcePidOutputMode;
    snapshot.timingDiagnostics = timingDiagnostics;
}

void ControlWorker::throttledInfo(const QString& message, const std::string& type, int throttleMs)
{
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(nowMs - lastInfoMs < throttleMs){
        return;
    }
    lastInfoMs = nowMs;
    emit displayInfoSignal(message.toStdString(), type);
}
