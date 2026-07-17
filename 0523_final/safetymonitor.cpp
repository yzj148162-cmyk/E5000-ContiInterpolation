#include "safetymonitor.h"

#include "controlworker.h"
#include "hardwareinterface.h"
#include "MatrixFun.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

SafetyMonitor::SafetyMonitor(QObject* parent)
    : QObject(parent)
{
}

void SafetyMonitor::setControlWorker(ControlWorker* worker)
{
    controlWorker = worker;
}

void SafetyMonitor::setHardwareInterface(HardwareInterface* hardware)
{
    hardwareInterface = hardware;
}

void SafetyMonitor::setConfig(const Config& newConfig)
{
    QMutexLocker locker(&configMutex);
    Config mergedConfig = newConfig;
    if(config.mainThreadHeartbeatMs > mergedConfig.mainThreadHeartbeatMs){
        mergedConfig.mainThreadHeartbeatMs = config.mainThreadHeartbeatMs;
    }
    config = mergedConfig;
}

void SafetyMonitor::updateMainThreadHeartbeat(qint64 timestampMs)
{
    if(timestampMs <= 0){
        return;
    }

    QMutexLocker locker(&configMutex);
    if(timestampMs > config.mainThreadHeartbeatMs){
        config.mainThreadHeartbeatMs = timestampMs;
    }
}

void SafetyMonitor::start()
{
    if(!timer){
        timer = new QTimer(this);
        timer->setTimerType(Qt::PreciseTimer);
        connect(timer, &QTimer::timeout, this, &SafetyMonitor::evaluateSafety);
    }

    const Config cfg = currentConfig();
    timer->setInterval(std::max(1, static_cast<int>(std::round(cfg.cycleMs))));
    timer->start();
}

void SafetyMonitor::stop()
{
    if(timer){
        timer->stop();
    }
    resetDynamicState(true);
}

void SafetyMonitor::clearFaultLatch()
{
    faultLatched = false;
    manualEmergencyRequested = false;
    manualEmergencyReason.clear();
    cableBreakControlledStopActive = false;
    cableBreakControlledStopAxisIndex = -1;
    injectedCableBreakPending = false;
    injectedCableBreakSensorIndex = -1;
    mainThreadHeartbeatTimeoutCycles = 0;
    lastSnapshotUpdateMs = -1;
    hasSeenSnapshot = false;
    lastSnapshotSequence = 0;
    std::fill(lowForceCycles.begin(), lowForceCycles.end(), 0);
    std::fill(highForceCycles.begin(), highForceCycles.end(), 0);
    std::fill(overSpeedCycles.begin(), overSpeedCycles.end(), 0);
    previousForceSensorValue.clear();
    resetWorkspaceState();
}

void SafetyMonitor::requestEmergencyStop(const QString& reason)
{
    manualEmergencyRequested = true;
    manualEmergencyReason = reason;
}

void SafetyMonitor::requestInjectedCableBreak(int sensorIndex)
{
    injectedCableBreakPending = true;
    injectedCableBreakSensorIndex = sensorIndex;
}

SafetyMonitor::Config SafetyMonitor::currentConfig() const
{
    QMutexLocker locker(&configMutex);
    return config;
}

void SafetyMonitor::resetDynamicState(bool clearLatchedFault)
{
    if(clearLatchedFault){
        clearFaultLatch();
        return;
    }

    manualEmergencyRequested = false;
    manualEmergencyReason.clear();
    cableBreakControlledStopActive = false;
    cableBreakControlledStopAxisIndex = -1;
    injectedCableBreakPending = false;
    injectedCableBreakSensorIndex = -1;
    mainThreadHeartbeatTimeoutCycles = 0;
    lastSnapshotUpdateMs = -1;
    hasSeenSnapshot = false;
    lastSnapshotSequence = 0;
    std::fill(lowForceCycles.begin(), lowForceCycles.end(), 0);
    std::fill(highForceCycles.begin(), highForceCycles.end(), 0);
    std::fill(overSpeedCycles.begin(), overSpeedCycles.end(), 0);
    previousForceSensorValue.clear();
    resetWorkspaceState();
}

void SafetyMonitor::ensureAxisStateSize(int axisCount, int sensorCount)
{
    const int safeAxisCount = std::max(axisCount, 0);
    const int safeSensorCount = std::max(sensorCount, 0);
    overSpeedCycles.resize(safeAxisCount, 0);
    lowForceCycles.resize(safeSensorCount, 0);
    highForceCycles.resize(safeSensorCount, 0);
}

void SafetyMonitor::resetWorkspaceState()
{
    workspaceMissingPoseCycles = 0;
    workspaceExceededCycles = 0;
    workspaceNearBoundaryActive = false;
}

void SafetyMonitor::triggerFault(StopLevel level,
                                 FaultCode code,
                                 const QString& summary,
                                 const QString& detail)
{
    const bool allowCableBreakEscalation = faultLatched &&
            cableBreakControlledStopActive &&
            code == FaultCode::CableBreak &&
            level == StopLevel::EmergencyStop;
    const bool allowOtherFaultEscalation = faultLatched &&
            cableBreakControlledStopActive &&
            (level >= StopLevel::SafetyStop || code != FaultCode::CableBreak);
    if(faultLatched &&
            level != StopLevel::Warning &&
            !allowCableBreakEscalation &&
            !allowOtherFaultEscalation){
        return;
    }

    if(level != StopLevel::Warning){
        faultLatched = true;
    }
    if(code == FaultCode::CableBreak && level == StopLevel::ControlledStop){
        cableBreakControlledStopActive = true;
    }
    else if(level != StopLevel::Warning){
        cableBreakControlledStopActive = false;
        cableBreakControlledStopAxisIndex = -1;
    }
    emit faultDetected(static_cast<int>(level),
                       static_cast<int>(code),
                       summary,
                       detail);
}

int SafetyMonitor::findAxisIndexBySensor(const Config& cfg, int sensorIndex) const
{
    if(sensorIndex < 0){
        return -1;
    }
    for(int axisIndex = 0; axisIndex < static_cast<int>(cfg.axes.size()); ++axisIndex){
        const AxisConfig& axis = cfg.axes[axisIndex];
        if(axis.monitored && axis.sensorIndex == sensorIndex){
            return axisIndex;
        }
    }
    return -1;
}

int SafetyMonitor::computeForceJacobianRank(const Config& cfg,
                                            int excludedAxisIndex,
                                            QString* detail) const
{
    if(detail){
        detail->clear();
    }
    if(!cfg.hasActualPose || cfg.actualPose.size() < 6){
        if(detail){
            *detail = QStringLiteral("断绳后可控性评估失败：当前没有可用的末端位姿。");
        }
        return -1;
    }

    const double px = cfg.actualPose[0];
    const double py = cfg.actualPose[1];
    const double pz = cfg.actualPose[2];
    const double rx = cfg.actualPose[3];
    const double ry = cfg.actualPose[4];
    const double rz = cfg.actualPose[5];
    if(!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz) ||
            !std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)){
        if(detail){
            *detail = QStringLiteral("断绳后可控性评估失败：当前末端位姿包含非数值项。");
        }
        return -1;
    }

    const Eigen::Vector3d platformCenter(px, py, pz);
    const Eigen::Matrix3d rotation =
            (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())).toRotationMatrix();

    std::vector<Eigen::Matrix<double, 6, 1>> columns;
    columns.reserve(cfg.axes.size());
    for(int axisIndex = 0; axisIndex < static_cast<int>(cfg.axes.size()); ++axisIndex){
        if(axisIndex == excludedAxisIndex){
            continue;
        }

        const AxisConfig& axis = cfg.axes[axisIndex];
        if(!axis.modeledForForceJacobian || axis.endIndex != cfg.poseEndIndex){
            continue;
        }
        if(axis.anchorPoint.size() < 3 || axis.contactPointLocal.size() < 3){
            if(detail){
                *detail = QStringLiteral("断绳后可控性评估失败：轴%1的锚点或末端接点坐标配置不完整。")
                        .arg(axisIndex + 1);
            }
            return -1;
        }

        const Eigen::Vector3d anchorPoint(axis.anchorPoint[0],
                                          axis.anchorPoint[1],
                                          axis.anchorPoint[2]);
        const Eigen::Vector3d localContactPoint(axis.contactPointLocal[0],
                                                axis.contactPointLocal[1],
                                                axis.contactPointLocal[2]);
        const Eigen::Vector3d globalContactPoint = platformCenter + rotation * localContactPoint;
        const MatrixFun::CableLengthResult cableGeometry =
                MatrixFun::cableLengthCalculate(globalContactPoint, anchorPoint, cfg.pulleyRadius);
        const Eigen::Vector3d cableVector = cableGeometry.tangentPoint - globalContactPoint;
        const double cableLength = cableGeometry.idealLength;
        if(!std::isfinite(cableLength) || cableLength <= 1e-9){
            if(detail){
                *detail = QStringLiteral("断绳后可控性评估失败：轴%1对应绳索几何退化，无法构造力雅可比。")
                        .arg(axisIndex + 1);
            }
            return -1;
        }

        const Eigen::Vector3d unitCableVector = cableVector / cableLength;
        const Eigen::Vector3d momentArm = globalContactPoint - platformCenter;
        Eigen::Matrix<double, 6, 1> column = Eigen::Matrix<double, 6, 1>::Zero();
        column.segment<3>(0) = unitCableVector;
        column.segment<3>(3) = momentArm.cross(unitCableVector);
        columns.push_back(column);
    }

    if(columns.empty()){
        if(detail){
            *detail = QStringLiteral("断绳后可控性评估失败：没有可用于构造力雅可比的剩余绳索。");
        }
        return 0;
    }

    Eigen::MatrixXd forceJacobian(6, static_cast<int>(columns.size()));
    for(int columnIndex = 0; columnIndex < static_cast<int>(columns.size()); ++columnIndex){
        forceJacobian.col(columnIndex) = columns[columnIndex];
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(forceJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singularValues = svd.singularValues();
    const double maxSingularValue = singularValues.size() > 0 ? singularValues(0) : 0.0;
    const double tolerance = std::max(cfg.forceJacobianRankTolerance,
                                      std::numeric_limits<double>::epsilon() *
                                      std::max(forceJacobian.rows(), forceJacobian.cols()) *
                                      maxSingularValue);

    int rank = 0;
    for(int index = 0; index < singularValues.size(); ++index){
        if(singularValues(index) > tolerance){
            ++rank;
        }
    }

    if(detail){
        *detail = QStringLiteral("排除疑似断绳轴%1后，剩余%2根建模绳索的力雅可比矩阵秩为%3。")
                .arg(excludedAxisIndex >= 0 ? excludedAxisIndex + 1 : 0)
                .arg(static_cast<int>(columns.size()))
                .arg(rank);
    }
    return rank;
}

bool SafetyMonitor::isCableBreakControllable(const Config& cfg,
                                             int brokenAxisIndex,
                                             int* rank,
                                             QString* detail) const
{
    const int jacobianRank = computeForceJacobianRank(cfg, brokenAxisIndex, detail);
    if(rank){
        *rank = std::max(jacobianRank, 0);
    }
    if(jacobianRank < 0){
        return false;
    }
    return jacobianRank >= std::max(cfg.cableBreakControllableRankThreshold, 1);
}

bool SafetyMonitor::handleCableBreakFault(const Config& cfg,
                                          int brokenAxisIndex,
                                          const QString& summary,
                                          const QString& detail)
{
    int jacobianRank = 0;
    QString rankDetail;
    const bool controllable = cfg.cableBreakControllabilityEnabled &&
            isCableBreakControllable(cfg, brokenAxisIndex, &jacobianRank, &rankDetail);
    if(controllable){
        cableBreakControlledStopAxisIndex = brokenAxisIndex;
        triggerFault(StopLevel::ControlledStop,
                     FaultCode::CableBreak,
                     summary,
                     QStringLiteral("%1 %2 满足秩阈值%3，执行控制盒链路缓停，并在缓停过程中持续复评。")
                         .arg(detail, rankDetail)
                         .arg(cfg.cableBreakControllableRankThreshold));
        return true;
    }

    cableBreakControlledStopAxisIndex = -1;
    const QString fallbackReason = !cfg.cableBreakControllabilityEnabled ?
                QStringLiteral("断绳后可控性评估未启用。") :
                (rankDetail.isEmpty() ?
                     QStringLiteral("断绳后可控性评估失败或秩不足。") :
                     rankDetail + QStringLiteral(" 未满足秩阈值%1。")
                     .arg(cfg.cableBreakControllableRankThreshold));
    triggerFault(StopLevel::EmergencyStop,
                 FaultCode::CableBreak,
                 summary,
                 QStringLiteral("%1 %2 已直接执行安全急停。").arg(detail, fallbackReason));
    return false;
}

bool SafetyMonitor::issueDirectEmergencyStop() const
{
    if(!hardwareInterface){
        return false;
    }
    return hardwareInterface->emergencyStopAll();
}

void SafetyMonitor::appendWatchdogEventLog(const Config& cfg,
                                           const QString& eventType,
                                           FaultCode code,
                                           const QString& summary,
                                           const QString& detail,
                                           bool stopActionAttempted,
                                           bool stopActionSucceeded,
                                           const QString& note) const
{
    if(cfg.watchdogLogFilePath.isEmpty()){
        return;
    }

    const QFileInfo fileInfo(cfg.watchdogLogFilePath);
    if(!QDir().mkpath(fileInfo.absolutePath())){
        return;
    }

    QFile file(cfg.watchdogLogFilePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)){
        return;
    }

    auto faultCodeName = [](FaultCode faultCode) -> QString {
        switch(faultCode){
        case FaultCode::None:
            return QStringLiteral("none");
        case FaultCode::SnapshotTimeout:
            return QStringLiteral("snapshot_timeout");
        case FaultCode::HardwareDisconnected:
            return QStringLiteral("hardware_disconnected");
        case FaultCode::CableForceLow:
            return QStringLiteral("cable_force_low");
        case FaultCode::CableForceHigh:
            return QStringLiteral("cable_force_high");
        case FaultCode::CableBreak:
            return QStringLiteral("cable_break");
        case FaultCode::MotorRangeExceeded:
            return QStringLiteral("motor_range_exceeded");
        case FaultCode::MotorOverspeed:
            return QStringLiteral("motor_overspeed");
        case FaultCode::SensorInvalid:
            return QStringLiteral("sensor_invalid");
        case FaultCode::WorkspaceExceeded:
            return QStringLiteral("workspace_exceeded");
        case FaultCode::SoftwareHang:
            return QStringLiteral("software_hang");
        case FaultCode::MotorTorqueExceeded:
            return QStringLiteral("motor_torque_exceeded");
        }
        return QStringLiteral("unknown");
    };

    QJsonObject object;
    object.insert(QStringLiteral("schema_version"), 1);
    object.insert(QStringLiteral("logged_at"),
                  QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    object.insert(QStringLiteral("event_type"), eventType);
    object.insert(QStringLiteral("fault_code"), faultCodeName(code));
    object.insert(QStringLiteral("summary"), summary);
    object.insert(QStringLiteral("detail"), detail);
    object.insert(QStringLiteral("stop_action_attempted"), stopActionAttempted);
    object.insert(QStringLiteral("stop_action_succeeded"), stopActionSucceeded);
    object.insert(QStringLiteral("system_running"), cfg.systemRunning);
    object.insert(QStringLiteral("hardware_connected"), cfg.hardwareConnected);
    object.insert(QStringLiteral("motion_active"), cfg.motionActive);
    object.insert(QStringLiteral("main_thread_heartbeat_ms"), cfg.mainThreadHeartbeatMs);
    object.insert(QStringLiteral("main_thread_heartbeat_timeout_ms"),
                  cfg.mainThreadHeartbeatTimeoutMs);
    object.insert(QStringLiteral("main_thread_heartbeat_grace_ms"),
                  cfg.mainThreadHeartbeatGraceMs);
    object.insert(QStringLiteral("main_thread_heartbeat_timeout_cycles"),
                  mainThreadHeartbeatTimeoutCycles);
    if(!note.isEmpty()){
        object.insert(QStringLiteral("note"), note);
    }

    file.write(QJsonDocument(object).toJson(QJsonDocument::Compact));
    file.write("\n");
    file.flush();
    file.close();
}

void SafetyMonitor::evaluateSafety()
{
    const Config cfg = currentConfig();
    if(timer){
        timer->setInterval(std::max(1, static_cast<int>(std::round(cfg.cycleMs))));
    }

    ensureAxisStateSize(cfg.axisCount, cfg.sensorCount);

    if(!cfg.monitorEnabled || !cfg.systemRunning){
        resetDynamicState(true);
        return;
    }

    if(faultLatched && !cableBreakControlledStopActive){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(cfg.softwareWatchdogEnabled && cfg.mainThreadHeartbeatMs > 0){
        const qint64 heartbeatAgeMs = std::max<qint64>(0, nowMs - cfg.mainThreadHeartbeatMs);
        const int heartbeatTimeoutMs = std::max(cfg.mainThreadHeartbeatTimeoutMs, 200);
        const int heartbeatGraceMs = std::max(cfg.mainThreadHeartbeatGraceMs, 0);
        const int requiredTimeoutCycles = std::max(cfg.mainThreadHeartbeatTimeoutFaultCycles, 1);
        if(heartbeatAgeMs > heartbeatTimeoutMs + heartbeatGraceMs){
            ++mainThreadHeartbeatTimeoutCycles;
        }
        else{
            mainThreadHeartbeatTimeoutCycles = 0;
        }

        if(mainThreadHeartbeatTimeoutCycles >= requiredTimeoutCycles){
            const bool stopOk = issueDirectEmergencyStop();
            const QString summary = QStringLiteral("主控制程序心跳超时");
            const QString detail = QStringLiteral(
                        "SafetyMonitor 检测到主线程心跳已超时 %1 ms（阈值 %2 ms），已绕过主线程直接请求硬件急停。")
                    .arg(heartbeatAgeMs)
                    .arg(heartbeatTimeoutMs + heartbeatGraceMs);
            appendWatchdogEventLog(cfg,
                                   QStringLiteral("software_watchdog_timeout"),
                                   FaultCode::SoftwareHang,
                                   summary,
                                   detail,
                                   true,
                                   stopOk,
                                   stopOk ?
                                       QStringLiteral("硬件急停已由 SafetyMonitor 线程直接触发") :
                                       QStringLiteral("硬件急停请求失败或当前未连接驱动"));
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::SoftwareHang,
                         summary,
                         stopOk ?
                             detail :
                             detail + QStringLiteral(" 硬件急停请求失败或当前未连接驱动。"));
            return;
        }
    }
    else{
        mainThreadHeartbeatTimeoutCycles = 0;
    }

    if(manualEmergencyRequested){
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::HardwareDisconnected,
                     manualEmergencyReason.isEmpty() ? QStringLiteral("软件急停触发") : manualEmergencyReason,
                     QStringLiteral("独立安全监控链路接收到软件急停请求"));
        return;
    }

    if(cableBreakControlledStopActive){
        if(!cfg.motionActive){
            cableBreakControlledStopActive = false;
            cableBreakControlledStopAxisIndex = -1;
        }
        else{
            int jacobianRank = 0;
            QString rankDetail;
            const bool stillControllable = cfg.cableBreakControllabilityEnabled &&
                    isCableBreakControllable(cfg,
                                             cableBreakControlledStopAxisIndex,
                                             &jacobianRank,
                                             &rankDetail);
            if(!stillControllable){
                cableBreakControlledStopActive = false;
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::CableBreak,
                             QStringLiteral("断绳缓停过程中剩余执行链不可控"),
                             rankDetail.isEmpty() ?
                                 QStringLiteral("缓停过程中无法继续保持6自由度可控性，已升级为安全急停。") :
                                 rankDetail + QStringLiteral(" 未满足秩阈值，已升级为安全急停。"));
                return;
            }
        }
    }

    if(!hardwareInterface || !cfg.hardwareConnected || !hardwareInterface->isLSConnected()){
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::HardwareDisconnected,
                     QStringLiteral("驱动通信链路异常"),
                     QStringLiteral("安全监控检测到驱动控制卡未连接或通信中断，已请求最高等级停机"));
        return;
    }

    if(!controlWorker){
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::SnapshotTimeout,
                     QStringLiteral("控制快照链路缺失"),
                     QStringLiteral("安全监控未绑定 ControlWorker，无法获得实时控制快照"));
        return;
    }

    const ControlWorker::Snapshot snapshot = controlWorker->latestSnapshot();
    if(!hasSeenSnapshot || snapshot.sequence != lastSnapshotSequence){
        hasSeenSnapshot = true;
        lastSnapshotSequence = snapshot.sequence;
        lastSnapshotUpdateMs = nowMs;
    }
    else if(lastSnapshotUpdateMs >= 0 &&
            nowMs - lastSnapshotUpdateMs > std::max(cfg.snapshotTimeoutMs, 100)){
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::SnapshotTimeout,
                     QStringLiteral("控制快照超时"),
                     QStringLiteral("独立安全监控在 %1 ms 内未收到新的控制快照")
                         .arg(nowMs - lastSnapshotUpdateMs));
        return;
    }

    if(!cfg.motionActive){
        previousForceSensorValue = snapshot.forceSensorValue;
        std::fill(lowForceCycles.begin(), lowForceCycles.end(), 0);
        std::fill(highForceCycles.begin(), highForceCycles.end(), 0);
        std::fill(overSpeedCycles.begin(), overSpeedCycles.end(), 0);
        resetWorkspaceState();
        return;
    }

    if(injectedCableBreakPending){
        const int injectedAxisIndex = findAxisIndexBySensor(cfg, injectedCableBreakSensorIndex);
        const QString summary = injectedAxisIndex >= 0 ?
                    QStringLiteral("故障注入：模拟绳索%1断裂").arg(injectedCableBreakSensorIndex + 1) :
                    QStringLiteral("故障注入：模拟断绳");
        const QString detail = injectedAxisIndex >= 0 ?
                    QStringLiteral("用户通过故障注入入口触发绳索/张力通道%1断裂模拟。")
                        .arg(injectedCableBreakSensorIndex + 1) :
                    QStringLiteral("用户通过故障注入入口触发断绳模拟，但当前未找到对应的绳索轴配置。");
        injectedCableBreakPending = false;
        injectedCableBreakSensorIndex = -1;
        if(injectedAxisIndex < 0){
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::CableBreak,
                         summary,
                         detail + QStringLiteral(" 已直接执行安全急停。"));
        }
        else{
            handleCableBreakFault(cfg, injectedAxisIndex, summary, detail);
        }
        return;
    }

    if(cfg.workspaceMonitorEnabled && !cfg.singleCableForceDebugMode){
        if(!cfg.hasActualPose || cfg.actualPose.size() < 3){
            workspaceNearBoundaryActive = false;
            workspaceExceededCycles = 0;
            workspaceMissingPoseCycles++;
            if(workspaceMissingPoseCycles >= std::max(cfg.poseTimeoutCycles, 1)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::SensorInvalid,
                             QStringLiteral("末端位姿反馈超时"),
                             QStringLiteral("安全监控在运动期间连续 %1 个周期未收到有效末端位姿反馈")
                                 .arg(workspaceMissingPoseCycles));
                return;
            }
        }
        else{
            workspaceMissingPoseCycles = 0;

            const double px = cfg.actualPose[0];
            const double py = cfg.actualPose[1];
            const double pz = cfg.actualPose[2];
            if(!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::SensorInvalid,
                             QStringLiteral("末端位姿反馈无效"),
                             QStringLiteral("末端位姿反馈存在非数值坐标"));
                return;
            }

            const double overflowX = std::max(cfg.workspaceXMin - px, px - cfg.workspaceXMax);
            const double overflowY = std::max(cfg.workspaceYMin - py, py - cfg.workspaceYMax);
            const double overflowZ = std::max(cfg.workspaceZMin - pz, pz - cfg.workspaceZMax);
            const double maxOverflow = std::max({0.0, overflowX, overflowY, overflowZ});

            if(maxOverflow > 0.0){
                workspaceNearBoundaryActive = false;
                workspaceExceededCycles++;

                const QString detail = QStringLiteral(
                            "末端当前位置 px=%1, py=%2, pz=%3 超出安全工作空间 X[%4, %5], Y[%6, %7], Z[%8, %9]")
                        .arg(px, 0, 'f', 3)
                        .arg(py, 0, 'f', 3)
                        .arg(pz, 0, 'f', 3)
                        .arg(cfg.workspaceXMin, 0, 'f', 3)
                        .arg(cfg.workspaceXMax, 0, 'f', 3)
                        .arg(cfg.workspaceYMin, 0, 'f', 3)
                        .arg(cfg.workspaceYMax, 0, 'f', 3)
                        .arg(cfg.workspaceZMin, 0, 'f', 3)
                        .arg(cfg.workspaceZMax, 0, 'f', 3);

                if(maxOverflow >= std::max(cfg.workspaceSevereOverflow, 1.0)){
                    triggerFault(StopLevel::EmergencyStop,
                                 FaultCode::WorkspaceExceeded,
                                 QStringLiteral("末端位姿严重越界"),
                                 detail);
                    return;
                }

                if(workspaceExceededCycles >= std::max(cfg.persistentFaultCycles, 1)){
                    triggerFault(StopLevel::SafetyStop,
                                 FaultCode::WorkspaceExceeded,
                                 QStringLiteral("末端位姿持续越界"),
                                 detail);
                    return;
                }
            }
            else{
                workspaceExceededCycles = 0;

                const double marginX = std::min(px - cfg.workspaceXMin, cfg.workspaceXMax - px);
                const double marginY = std::min(py - cfg.workspaceYMin, cfg.workspaceYMax - py);
                const double marginZ = std::min(pz - cfg.workspaceZMin, cfg.workspaceZMax - pz);
                const double minMargin = std::min({marginX, marginY, marginZ});
                const bool nearBoundary = cfg.workspaceWarningMargin > 0.0 &&
                        minMargin <= cfg.workspaceWarningMargin;

                if(nearBoundary && !workspaceNearBoundaryActive){
                    workspaceNearBoundaryActive = true;
                    triggerFault(StopLevel::Warning,
                                 FaultCode::WorkspaceExceeded,
                                 QStringLiteral("末端位姿接近安全边界"),
                                 QStringLiteral("末端当前位置 px=%1, py=%2, pz=%3，距离最近安全边界仅 %4 mm")
                                     .arg(px, 0, 'f', 3)
                                     .arg(py, 0, 'f', 3)
                                     .arg(pz, 0, 'f', 3)
                                     .arg(minMargin, 0, 'f', 3));
                }
                else if(!nearBoundary){
                    workspaceNearBoundaryActive = false;
                }
            }
        }
    }
    else{
        resetWorkspaceState();
    }

    if(cfg.singleCableForceDebugMode){
        std::fill(lowForceCycles.begin(), lowForceCycles.end(), 0);
        previousForceSensorValue = snapshot.forceSensorValue;
    }

    const int axisCount = std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
    const int sensorCount = static_cast<int>(snapshot.forceSensorValue.size());
    const int motorPosCount = static_cast<int>(snapshot.motorRelRawPos.size());
    const int motorVelCount = static_cast<int>(snapshot.motorVel.size());

    for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
        const AxisConfig& axis = cfg.axes[axisIndex];
        if(!axis.monitored){
            continue;
        }
        const bool skipBrokenAxisForceChecks = cableBreakControlledStopActive &&
                axisIndex == cableBreakControlledStopAxisIndex;
        const bool skipLowForceAndBreakChecks =
                skipBrokenAxisForceChecks || cfg.singleCableForceDebugMode;

        if(axis.sensorIndex >= 0 && axis.sensorIndex < sensorCount){
            const double forceValue = snapshot.forceSensorValue[axis.sensorIndex];
            if(!std::isfinite(forceValue)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::SensorInvalid,
                             QStringLiteral("张力传感器数据无效"),
                             QStringLiteral("传感器通道 %1 反馈了非数值张力数据")
                                 .arg(axis.sensorIndex + 1));
                return;
            }

            if(!skipBrokenAxisForceChecks && axis.forceMax > 1e-6){
                if(forceValue > axis.forceMax * cfg.severeForceOverRatio){
                    triggerFault(StopLevel::EmergencyStop,
                                 FaultCode::CableForceHigh,
                                 QStringLiteral("绳索张力严重超上限"),
                                 QStringLiteral("轴 %1 张力 %2 超出严重上限 %3")
                                     .arg(axisIndex + 1)
                                     .arg(forceValue, 0, 'f', 3)
                                     .arg(axis.forceMax * cfg.severeForceOverRatio, 0, 'f', 3));
                    return;
                }
                if(forceValue > axis.forceMax){
                    highForceCycles[axis.sensorIndex]++;
                    if(highForceCycles[axis.sensorIndex] >= std::max(cfg.persistentFaultCycles, 1)){
                        triggerFault(StopLevel::SafetyStop,
                                     FaultCode::CableForceHigh,
                                     QStringLiteral("绳索张力持续超上限"),
                                     QStringLiteral("轴 %1 张力 %2 持续大于上限 %3")
                                         .arg(axisIndex + 1)
                                         .arg(forceValue, 0, 'f', 3)
                                         .arg(axis.forceMax, 0, 'f', 3));
                        return;
                    }
                }
                else{
                    highForceCycles[axis.sensorIndex] = 0;
                }
            }

            if(!skipLowForceAndBreakChecks &&
                    axis.forceMin > 1e-6 &&
                    axis.sensorIndex < static_cast<int>(previousForceSensorValue.size())){
                const double previousForce = previousForceSensorValue[axis.sensorIndex];
                const double dropThreshold = std::max(axis.forceMin * cfg.breakForceRatio, 0.1);
                const double requiredDrop = axis.forceMin * cfg.breakDropRatio;
                if(previousForce > axis.forceMin &&
                        forceValue <= dropThreshold &&
                        (previousForce - forceValue) >= requiredDrop){
                    handleCableBreakFault(
                                cfg,
                                axisIndex,
                                QStringLiteral("检测到疑似断绳/断崖式失张"),
                                QStringLiteral("轴%1张力由%2快速跌落至%3。")
                                    .arg(axisIndex + 1)
                                    .arg(previousForce, 0, 'f', 3)
                                    .arg(forceValue, 0, 'f', 3));
                    return;
                }
            }

            if(!skipLowForceAndBreakChecks && axis.forceMin > 1e-6){
                if(axis.sensorIndex < static_cast<int>(previousForceSensorValue.size())){
                    const double previousForce = previousForceSensorValue[axis.sensorIndex];
                    const double dropThreshold = std::max(axis.forceMin * cfg.breakForceRatio, 0.1);
                    const double requiredDrop = axis.forceMin * cfg.breakDropRatio;
                    if(previousForce > axis.forceMin &&
                            forceValue <= dropThreshold &&
                            (previousForce - forceValue) >= requiredDrop){
                        triggerFault(StopLevel::EmergencyStop,
                                     FaultCode::CableBreak,
                                     QStringLiteral("检测到疑似断绳/断崖式失张"),
                                     QStringLiteral("轴 %1 张力由 %2 快速跌落至 %3")
                                         .arg(axisIndex + 1)
                                         .arg(previousForce, 0, 'f', 3)
                                         .arg(forceValue, 0, 'f', 3));
                        return;
                    }
                }

                if(forceValue < axis.forceMin){
                    lowForceCycles[axis.sensorIndex]++;
                    if(lowForceCycles[axis.sensorIndex] >= std::max(cfg.persistentFaultCycles, 1)){
                        triggerFault(StopLevel::ControlledStop,
                                     FaultCode::CableForceLow,
                                     QStringLiteral("绳索张力持续低于下限"),
                                     QStringLiteral("轴 %1 张力 %2 持续低于下限 %3")
                                         .arg(axisIndex + 1)
                                         .arg(forceValue, 0, 'f', 3)
                                         .arg(axis.forceMin, 0, 'f', 3));
                        return;
                    }
                }
                else{
                    lowForceCycles[axis.sensorIndex] = 0;
                }
            }
        }

        if(axisIndex < motorPosCount){
            const double rawPos = snapshot.motorRelRawPos[axisIndex];
            if(!std::isfinite(rawPos)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::MotorRangeExceeded,
                             QStringLiteral("电机位置反馈无效"),
                             QStringLiteral("轴 %1 反馈了非数值位置").arg(axisIndex + 1));
                return;
            }

            if(axis.motorMax > axis.motorMin &&
                    (rawPos >= axis.motorMax || rawPos <= axis.motorMin)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::MotorRangeExceeded,
                             QStringLiteral("电机位置超出安全范围"),
                             QStringLiteral("轴 %1 当前位置 %2 超出安全范围 [%3, %4]")
                                 .arg(axisIndex + 1)
                                 .arg(rawPos, 0, 'f', 3)
                                 .arg(axis.motorMin, 0, 'f', 3)
                                 .arg(axis.motorMax, 0, 'f', 3));
                return;
            }
        }

        if(axisIndex < motorVelCount){
            const double motorVel = snapshot.motorVel[axisIndex];
            if(!std::isfinite(motorVel)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::MotorOverspeed,
                             QStringLiteral("电机速度反馈无效"),
                             QStringLiteral("轴 %1 反馈了非数值速度").arg(axisIndex + 1));
                return;
            }

            if(axis.motorVelMax > 1e-6 &&
                    std::abs(motorVel) > axis.motorVelMax * cfg.severeSpeedOverRatio){
                overSpeedCycles[axisIndex]++;
                if(overSpeedCycles[axisIndex] >= std::max(cfg.persistentFaultCycles, 1)){
                    triggerFault(StopLevel::SafetyStop,
                                 FaultCode::MotorOverspeed,
                                 QStringLiteral("电机速度持续超上限"),
                                 QStringLiteral("轴 %1 当前速度 %2 持续大于安全上限 %3")
                                     .arg(axisIndex + 1)
                                     .arg(motorVel, 0, 'f', 3)
                                     .arg(axis.motorVelMax * cfg.severeSpeedOverRatio, 0, 'f', 3));
                    return;
                }
            }
            else if(axisIndex < static_cast<int>(overSpeedCycles.size())){
                overSpeedCycles[axisIndex] = 0;
            }
        }
    }

    previousForceSensorValue = snapshot.forceSensorValue;
}
