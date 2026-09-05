/*
 * 文件总览：
 * - SafetyMonitor 的实现文件，按固定周期读取 ControlWorker/HardwareInterface 快照并执行安全规则。
 * - 判定流程包含“先处理人工急停和连接类故障，再处理传感器/绳力/运动范围，最后处理工作空间与看门狗”。
 * - 断绳场景直接执行安全急停，不再尝试按剩余绳索可控性恢复或继续运动。
 */

#include "safetymonitor.h"

#include "controlworker.h"
#include "hardwareinterface.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {
constexpr qint64 kControllerCommunicationCheckIntervalMs = 100;
constexpr qint64 kCommissioningForceSensorTraceFreshTimeoutUs = 5 * 1000 * 1000;
constexpr qint64 kEndpointRemoteTraceFeedbackDelayLimitUs = 5 * 1000;

qint64 monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}
}

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
    injectedCableBreakPending = false;
    injectedCableBreakSensorIndex = -1;
    injectedMainThreadHeartbeatTimeoutPending = false;
    timedCableBreakForceZeroPending = false;
    timedCableBreakForceZeroSensorIndex = -1;
    timedCableBreakForceZeroTriggerTimeSec = -1.0;
    timedCableBreakForceZeroLastHealthyForce = std::numeric_limits<double>::quiet_NaN();
    mainThreadHeartbeatTimeoutCycles = 0;
    lastSnapshotUpdateMs = -1;
    hasSeenSnapshot = false;
    lastSnapshotSequence = 0;
    lastControllerCommunicationCheckMs = -1;
    commissioningTraceValidationStartMs = -1;
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

void SafetyMonitor::requestInjectedMainThreadHeartbeatTimeout()
{
    injectedMainThreadHeartbeatTimeoutPending = true;
    mainThreadHeartbeatTimeoutCycles = 0;
    evaluateSafety();
}

void SafetyMonitor::requestTimedCableBreakForceZero(int sensorIndex, double triggerTimeSec)
{
    timedCableBreakForceZeroPending = true;
    timedCableBreakForceZeroSensorIndex = sensorIndex;
    timedCableBreakForceZeroTriggerTimeSec =
            std::max(0.0, std::isfinite(triggerTimeSec) ? triggerTimeSec : 0.0);
    timedCableBreakForceZeroLastHealthyForce = std::numeric_limits<double>::quiet_NaN();
}

void SafetyMonitor::cancelTimedCableBreakForceZero()
{
    timedCableBreakForceZeroPending = false;
    timedCableBreakForceZeroSensorIndex = -1;
    timedCableBreakForceZeroTriggerTimeSec = -1.0;
    timedCableBreakForceZeroLastHealthyForce = std::numeric_limits<double>::quiet_NaN();
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
    injectedCableBreakPending = false;
    injectedCableBreakSensorIndex = -1;
    injectedMainThreadHeartbeatTimeoutPending = false;
    timedCableBreakForceZeroPending = false;
    timedCableBreakForceZeroSensorIndex = -1;
    timedCableBreakForceZeroTriggerTimeSec = -1.0;
    timedCableBreakForceZeroLastHealthyForce = std::numeric_limits<double>::quiet_NaN();
    mainThreadHeartbeatTimeoutCycles = 0;
    lastSnapshotUpdateMs = -1;
    hasSeenSnapshot = false;
    lastSnapshotSequence = 0;
    lastControllerCommunicationCheckMs = -1;
    commissioningTraceValidationStartMs = -1;
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
    clearWorkspaceWarningState();
}

void SafetyMonitor::clearWorkspaceWarningState()
{
    if(!workspaceNearBoundaryActive){
        return;
    }
    workspaceNearBoundaryActive = false;
    emit warningCleared(static_cast<int>(FaultCode::WorkspaceExceeded));
}

void SafetyMonitor::triggerFault(StopLevel level,
                                 FaultCode code,
                                 const QString& summary,
                                 const QString& detail)
{
    if(faultLatched && level != StopLevel::Warning){
        return;
    }

    if(level != StopLevel::Warning){
        faultLatched = true;
    }
    const bool directEmergencyStopRequired =
            level >= StopLevel::EmergencyStop &&
            code != FaultCode::SoftwareHang;
    if(directEmergencyStopRequired && controlWorker){
        // 先撤销ControlWorker侧的遥控命令授权，再把硬件急停排入同一个
        // HardwareThread。这样即使故障恰好发生在Trace读取期间，读取返回后
        // 也会先消费停机邮箱，不会再排入一条必然被失能状态拒绝的速度命令。
        controlWorker->requestEndpointRemoteSafetyStop(
                    QStringLiteral("独立安全监控已触发硬件急停：%1")
                        .arg(summary));
    }
    if(directEmergencyStopRequired){
        issueDirectEmergencyStop();
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


bool SafetyMonitor::issueDirectEmergencyStop() const
{
    if(!hardwareInterface){
        return false;
    }
    const Config cfg = currentConfig();
    if(cfg.commissioningMode && cfg.commissioningAxisIndex >= 0){
        return hardwareInterface->emergencyStopAxes(
                    std::vector<int>{cfg.commissioningAxisIndex});
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
        case FaultCode::MotorFault:
            return QStringLiteral("motor_fault");
        case FaultCode::PlcCommunicationFault:
            return QStringLiteral("plc_communication_fault");
        case FaultCode::StartupSelfCheckFailed:
            return QStringLiteral("startup_self_check_failed");
        case FaultCode::ControlBoxButtonNotReset:
            return QStringLiteral("control_box_button_not_reset");
        }
        return QStringLiteral("unknown");
    };

    auto faultDisplayName = [](FaultCode faultCode) -> QString {
        switch(faultCode){
        case FaultCode::None:
            return QStringLiteral("无故障/人工请求");
        case FaultCode::SnapshotTimeout:
            return QStringLiteral("状态快照超时");
        case FaultCode::HardwareDisconnected:
            return QStringLiteral("硬件连接异常");
        case FaultCode::CableForceLow:
            return QStringLiteral("绳索张力低");
        case FaultCode::CableForceHigh:
            return QStringLiteral("绳索张力高");
        case FaultCode::CableBreak:
            return QStringLiteral("断绳");
        case FaultCode::MotorRangeExceeded:
            return QStringLiteral("电机行程越界");
        case FaultCode::MotorOverspeed:
            return QStringLiteral("电机超速");
        case FaultCode::SensorInvalid:
            return QStringLiteral("传感器数据异常");
        case FaultCode::WorkspaceExceeded:
            return QStringLiteral("工作空间越界");
        case FaultCode::SoftwareHang:
            return QStringLiteral("软件卡死/主线程心跳超时");
        case FaultCode::MotorTorqueExceeded:
            return QStringLiteral("电机转矩超限");
        case FaultCode::MotorFault:
            return QStringLiteral("电机故障/失能");
        case FaultCode::PlcCommunicationFault:
            return QStringLiteral("工控机与PLC通信断开");
        case FaultCode::StartupSelfCheckFailed:
            return QStringLiteral("系统自检未通过");
        case FaultCode::ControlBoxButtonNotReset:
            return QStringLiteral("手柄控制盒安全按钮未复位");
        }
        return QStringLiteral("未知故障");
    };

    auto eventDisplayName = [](const QString& event) -> QString {
        if(event == QStringLiteral("software_watchdog_timeout")){
            return QStringLiteral("软件看门狗超时");
        }
        return event.isEmpty() ? QStringLiteral("安全监控事件") : event;
    };

    const QDateTime occurredAt = QDateTime::currentDateTime();
    const QString occurredAtIso = occurredAt.toString(Qt::ISODateWithMs);
    const QString eventText = eventDisplayName(eventType);
    const QString faultCode = faultCodeName(code);
    const QString faultText = faultDisplayName(code);
    const QString stopLevel = QStringLiteral("emergency_stop");
    const QString stopLevelText = QStringLiteral("安全急停");
    const QString displayMessage =
            QStringLiteral("%1；故障类型：%2；发生时间：%3；停机等级：%4；摘要：%5；详情：%6")
            .arg(eventText,
                 faultText,
                 occurredAtIso,
                 stopLevelText,
                 summary.isEmpty() ? QStringLiteral("无") : summary,
                 detail.isEmpty() ? QStringLiteral("无") : detail);

    QJsonObject object;
    object.insert(QStringLiteral("schema_version"), 2);
    object.insert(QStringLiteral("occurred_at"), occurredAtIso);
    object.insert(QStringLiteral("occurred_at_ms"), occurredAt.toMSecsSinceEpoch());
    object.insert(QStringLiteral("logged_at"), occurredAtIso);
    object.insert(QStringLiteral("logged_at_ms"), occurredAt.toMSecsSinceEpoch());
    object.insert(QStringLiteral("event_type"), eventType);
    object.insert(QStringLiteral("event_type_text"), eventText);
    object.insert(QStringLiteral("fault_code"), faultCode);
    object.insert(QStringLiteral("fault_code_value"), static_cast<int>(code));
    object.insert(QStringLiteral("fault_type"), faultCode);
    object.insert(QStringLiteral("fault_type_text"), faultText);
    object.insert(QStringLiteral("stop_level"), stopLevel);
    object.insert(QStringLiteral("stop_level_text"), stopLevelText);
    object.insert(QStringLiteral("stop_level_value"),
                  static_cast<int>(StopLevel::EmergencyStop));
    object.insert(QStringLiteral("summary"), summary);
    object.insert(QStringLiteral("detail"), detail);
    object.insert(QStringLiteral("display_message"), displayMessage);
    object.insert(QStringLiteral("reset_required"), true);
    object.insert(QStringLiteral("stop_action_attempted"), stopActionAttempted);
    object.insert(QStringLiteral("stop_action_succeeded"), stopActionSucceeded);
    object.insert(QStringLiteral("system_running"), cfg.systemRunning);
    object.insert(QStringLiteral("safety_armed"), cfg.safetyArmed);
    object.insert(QStringLiteral("commissioning_mode"), cfg.commissioningMode);
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
    // 安全监控按严重程度从“链路/心跳”到“运动状态/绳力/工作空间”逐层检查，发现故障立即返回，避免同周期重复停机。
    const Config cfg = currentConfig();
    if(timer){
        timer->setInterval(std::max(1, static_cast<int>(std::round(cfg.cycleMs))));
    }

    ensureAxisStateSize(cfg.axisCount, cfg.sensorCount);

    const bool heartbeatTimeoutInjected = injectedMainThreadHeartbeatTimeoutPending;
    if((!cfg.monitorEnabled || !cfg.safetyArmed) && !heartbeatTimeoutInjected){
        resetDynamicState(true);
        return;
    }

    if(faultLatched && !heartbeatTimeoutInjected){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(cfg.commissioningMode &&
            cfg.motionActive &&
            cfg.commissioningMotionStartMs > 0 &&
            cfg.commissioningMotionTimeoutMs > 0 &&
            nowMs - cfg.commissioningMotionStartMs >= cfg.commissioningMotionTimeoutMs){
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::MotorFault,
                     QStringLiteral("单轴调试运行超时"),
                     QStringLiteral("单轴调试动作已持续 %1 ms，超过调试上限 %2 ms，已执行安全急停。")
                         .arg(nowMs - cfg.commissioningMotionStartMs)
                         .arg(cfg.commissioningMotionTimeoutMs));
        return;
    }
    if((cfg.softwareWatchdogEnabled && cfg.mainThreadHeartbeatMs > 0) ||
            heartbeatTimeoutInjected){
        // 软件看门狗不依赖 MainWindow 响应，心跳超时后由本线程直接调用硬件急停。
        const int heartbeatTimeoutMs = std::max(cfg.mainThreadHeartbeatTimeoutMs, 200);
        const int heartbeatGraceMs = std::max(cfg.mainThreadHeartbeatGraceMs, 0);
        const int requiredTimeoutCycles = std::max(cfg.mainThreadHeartbeatTimeoutFaultCycles, 1);
        const qint64 heartbeatAgeMs = heartbeatTimeoutInjected ?
                    static_cast<qint64>(heartbeatTimeoutMs + heartbeatGraceMs +
                                        std::max(1, static_cast<int>(std::round(cfg.cycleMs)))) :
                    std::max<qint64>(0, nowMs - cfg.mainThreadHeartbeatMs);
        if(heartbeatAgeMs > heartbeatTimeoutMs + heartbeatGraceMs){
            ++mainThreadHeartbeatTimeoutCycles;
        }
        else{
            mainThreadHeartbeatTimeoutCycles = 0;
        }
        if(heartbeatTimeoutInjected){
            mainThreadHeartbeatTimeoutCycles =
                    std::max(mainThreadHeartbeatTimeoutCycles, requiredTimeoutCycles);
        }

        if(mainThreadHeartbeatTimeoutCycles >= requiredTimeoutCycles){
            injectedMainThreadHeartbeatTimeoutPending = false;
            const bool stopOk = issueDirectEmergencyStop();
            const QString summary = QStringLiteral("主控制程序心跳超时");
            const QString detail = QStringLiteral(
                        "安全监控线程检测到主线程心跳已超时 %1 ms（阈值 %2 ms），已绕过主线程直接请求硬件急停。")
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

    if(!hardwareInterface || !cfg.hardwareConnected || !hardwareInterface->isLSConnected()){
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::HardwareDisconnected,
                     QStringLiteral("驱动通信链路异常"),
                     QStringLiteral("安全监控检测到驱动控制卡未连接或通信中断，已请求最高等级停机"));
        return;
    }

    // isLSConnected() only represents whether the software once opened the card.
    // The complete-machine path therefore queries the master independently.
    // G302 commissioning uses the continuously refreshed Trace timestamps below,
    // because a synchronous master query would block its shared HardwareThread.
    if(!cfg.commissioningMode &&
            (lastControllerCommunicationCheckMs < 0 ||
            nowMs - lastControllerCommunicationCheckMs >=
            kControllerCommunicationCheckIntervalMs)){
        lastControllerCommunicationCheckMs = nowMs;
        const HardwareInterface::ConnectionItemDiagnostics controller =
                hardwareInterface->controllerDiagnostics();
        if(controller.state != HardwareInterface::ConnectionState::Connected){
            QString stateText;
            switch(controller.state){
            case HardwareInterface::ConnectionState::Disconnected:
                stateText = QStringLiteral("主站已断开");
                break;
            case HardwareInterface::ConnectionState::Fault:
                stateText = QStringLiteral("主站状态异常");
                break;
            case HardwareInterface::ConnectionState::Disabled:
                stateText = QStringLiteral("主站未使能");
                break;
            case HardwareInterface::ConnectionState::Connected:
                break;
            }
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::HardwareDisconnected,
                         QStringLiteral("EtherCAT总线/控制卡通信异常"),
                         QStringLiteral("安全监控检测到EtherCAT%1（主站状态=%2，控制卡错误码=%3，接口返回=%4）。"
                                        "软件连接标记仍存在，但现场总线不可用，已进入安全锁存并请求急停。")
                             .arg(stateText)
                             .arg(controller.busState)
                             .arg(controller.errorCode)
                             .arg(controller.apiResult));
            return;
        }
        if(cfg.forceSensorMonitoringEnabled && cfg.forceThreadRunning){
            for(int axisIndex = 0;
                axisIndex < static_cast<int>(cfg.axes.size());
                ++axisIndex){
                const AxisConfig& axis = cfg.axes[axisIndex];
                if(!axis.monitored || !axis.motionParticipant ||
                        !axis.monitorForce || axis.sensorIndex < 0){
                    continue;
                }
                const HardwareInterface::ConnectionItemDiagnostics sensor =
                        hardwareInterface->forceSensorDiagnostics(axis.sensorIndex);
                if(sensor.state != HardwareInterface::ConnectionState::Connected){
                    triggerFault(StopLevel::EmergencyStop,
                                 FaultCode::SensorInvalid,
                                 QStringLiteral("目标张力传感器反馈超时"),
                                 QStringLiteral("轴 %1 使用的张力传感器通道 %2 没有新鲜 Trace 数据（api=%3），已执行安全急停。")
                                     .arg(axisIndex + 1)
                                     .arg(axis.sensorIndex + 1)
                                     .arg(sensor.apiResult));
                    return;
                }
            }
        }
    }

    if(!controlWorker){
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::SnapshotTimeout,
                     QStringLiteral("控制快照链路缺失"),
                     QStringLiteral("安全监控未绑定 ControlWorker，无法获得实时控制快照"));
        return;
    }

    // ControlWorker 快照是安全判断的数据源；如果序号长时间不变，说明控制链路可能卡住。
    ControlWorker::Snapshot snapshot = controlWorker->latestSnapshot();
    const bool snapshotAdvanced =
            !hasSeenSnapshot || snapshot.sequence != lastSnapshotSequence;
    if(snapshotAdvanced){
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

    if(cfg.commissioningMode &&
            !cfg.commissioningHardwareCommandActive &&
            !cfg.commissioningControlSnapshotStartupActive){
        // G302 单轴调试的 ControlWorker 和所有雷赛调用共用 HardwareThread。
        // nmc_get_master_state 等同步诊断可能占用该线程约 1 s，若由安全
        // 线程高频插入，会反过来阻塞 Trace 并制造“控制快照超时”。这里
        // 使用控制快照随同携带的 Trace 帧时间戳检查总线反馈新鲜度。
        if(commissioningTraceValidationStartMs < 0){
            commissioningTraceValidationStartMs = nowMs;
        }
        const int traceFreshTimeoutMs =
                std::max(1000, std::max(cfg.snapshotTimeoutMs, 100) * 2);
        const qint64 traceNowUs = monotonicNowUs();
        const qint64 traceFrameAgeUs = snapshot.runtimeTraceFrameMonotonicUs > 0 ?
                    std::max<qint64>(0,
                        traceNowUs - snapshot.runtimeTraceFrameMonotonicUs) :
                    std::numeric_limits<qint64>::max();
        const qint64 traceFrameAgeMs =
                traceFrameAgeUs == std::numeric_limits<qint64>::max() ?
                    std::numeric_limits<qint64>::max() : traceFrameAgeUs / 1000;
        const bool traceFresh = snapshot.runtimeTraceFromHardware &&
                snapshot.runtimeTraceFrameMonotonicUs > 0 &&
                traceFrameAgeMs <= traceFreshTimeoutMs;
        if(!traceFresh &&
                nowMs - commissioningTraceValidationStartMs > traceFreshTimeoutMs){
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::HardwareDisconnected,
                         QStringLiteral("G302 Runtime Trace反馈超时"),
                         QStringLiteral("G302单轴安全监控检测到控制线程仍在运行，但最近Runtime Trace硬件帧已超过%1 ms未更新"
                                        "（帧年龄=%2 ms，快照序号=%3），已请求当前调试轴急停。")
                             .arg(traceFreshTimeoutMs)
                             .arg(traceFrameAgeMs == std::numeric_limits<qint64>::max() ?
                                      -1 : traceFrameAgeMs)
                             .arg(snapshot.sequence));
            return;
        }
        if(traceFresh && cfg.forceSensorMonitoringEnabled && cfg.forceThreadRunning){
            for(int axisIndex = 0;
                axisIndex < static_cast<int>(cfg.axes.size());
                ++axisIndex){
                const AxisConfig& axis = cfg.axes[axisIndex];
                if(!axis.monitored || !axis.motionParticipant ||
                        !axis.monitorForce || axis.sensorIndex < 0){
                    continue;
                }
                const qint64 sensorFrameUs =
                        axis.sensorIndex < static_cast<int>(
                            snapshot.forceSensorTraceFrameMonotonicUs.size()) ?
                            snapshot.forceSensorTraceFrameMonotonicUs[axis.sensorIndex] : 0;
                const qint64 sensorFrameAgeUs = sensorFrameUs > 0 ?
                            std::max<qint64>(0, traceNowUs - sensorFrameUs) :
                            std::numeric_limits<qint64>::max();
                if(sensorFrameAgeUs > kCommissioningForceSensorTraceFreshTimeoutUs){
                    triggerFault(StopLevel::EmergencyStop,
                                 FaultCode::SensorInvalid,
                                 QStringLiteral("目标张力传感器反馈超时"),
                                 QStringLiteral("G302单轴力控使用的张力传感器通道%1已超过%2 ms没有新鲜Trace数据，已执行当前轴急停。")
                                     .arg(axis.sensorIndex + 1)
                                     .arg(kCommissioningForceSensorTraceFreshTimeoutUs / 1000));
                    return;
                }
            }
        }
    }
    else{
        commissioningTraceValidationStartMs = -1;
    }

    if(!cfg.motionActive){
        previousForceSensorValue = snapshot.forceSensorValue;
        std::fill(lowForceCycles.begin(), lowForceCycles.end(), 0);
        std::fill(highForceCycles.begin(), highForceCycles.end(), 0);
        std::fill(overSpeedCycles.begin(), overSpeedCycles.end(), 0);
        resetWorkspaceState();
        return;
    }

    const int motorCheckAxisCount =
            std::min(cfg.axisCount, static_cast<int>(cfg.axes.size()));
    const ControlWorker::EndpointRemoteTracePhase endpointRemoteTracePhase =
            snapshot.endpointRemoteTracePhase;
    bool endpointRemoteUsesTraceState = false;
    switch(endpointRemoteTracePhase){
    case ControlWorker::EndpointRemoteTracePhase::TransitionAcquiring:
    case ControlWorker::EndpointRemoteTracePhase::RunningProfileAwaitingFrame:
    case ControlWorker::EndpointRemoteTracePhase::Running:
        endpointRemoteUsesTraceState = true;
        break;
    case ControlWorker::EndpointRemoteTracePhase::Inactive:
    case ControlWorker::EndpointRemoteTracePhase::TransitionPrepared:
    case ControlWorker::EndpointRemoteTracePhase::Faulted:
        break;
    }
    const bool endpointRemoteTraceRunning =
            endpointRemoteTracePhase ==
                ControlWorker::EndpointRemoteTracePhase::Running;
    // 只在ControlWorker发布新快照时接纳并验证采集时帧龄。同一个已经接纳
    // 的缓存快照在SafetyMonitor异步读取期间自然老化，不应被重新解释为
    // “采集时超过5 ms”；若ControlWorker停止发布，前面的快照超时和遥控
    // 自身的命令周期监督仍会失败关闭。
    if(endpointRemoteUsesTraceState && endpointRemoteTraceRunning &&
            snapshotAdvanced){
        if(snapshot.runtimeTraceStatusFaultLatched){
            const QString statusWordHex =
                    QString::number(snapshot.runtimeTraceStatusFaultWord, 16)
                    .rightJustified(4, QLatin1Char('0'))
                    .toUpper();
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::MotorFault,
                         QStringLiteral("末端遥控排空Trace帧驱动状态异常"),
                         QStringLiteral(
                             "末端遥控运行期排空帧锁存轴%1的0x6041=0x%2，"
                             "状态=%3，逻辑序号=%4；已执行安全急停。")
                         .arg(snapshot.runtimeTraceStatusFaultAxis + 1)
                         .arg(statusWordHex)
                         .arg(snapshot.runtimeTraceStatusFaultStateMachine)
                         .arg(snapshot.runtimeTraceStatusFaultLogicalFrameSequence));
            return;
        }
        const bool traceStateFrameReliable =
                snapshot.runtimeTraceUsageProfile ==
                    HardwareInterface::RuntimeTraceUsageProfile::
                        EndpointRemoteRunning &&
                snapshot.runtimeTraceFromHardware &&
                snapshot.runtimeTraceFrameSequenceValid &&
                snapshot.runtimeTraceTimingReliable &&
                snapshot.runtimeTraceFifoCaughtUp &&
                !snapshot.runtimeTraceLost &&
                snapshot.runtimeTraceSamplePeriodUs == 500 &&
                snapshot.runtimeTraceNewestFrameAgeUs >= 0 &&
                snapshot.runtimeTraceNewestFrameAgeUs <=
                    kEndpointRemoteTraceFeedbackDelayLimitUs;
        if(!traceStateFrameReliable){
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::MotorFault,
                         QStringLiteral("末端遥控驱动状态Trace无效"),
                         QStringLiteral(
                             "末端遥控正在运行，但安全监控取得的同帧驱动状态不可靠"
                             "（profile=%1，采集时帧龄/当前缓存帧龄=%2/%3 us，"
                             "采集上限=%4 us，来源/序号/时序/追平/丢帧=%5/%6/%7/%8/%9，"
                             "逻辑序号=%10，profile/config generation=%11/%12），"
                             "已执行安全急停。")
                             .arg(static_cast<int>(snapshot.runtimeTraceUsageProfile))
                             .arg(snapshot.runtimeTraceNewestFrameAgeUs)
                             .arg(snapshot.runtimeTraceCurrentFrameAgeUs)
                             .arg(kEndpointRemoteTraceFeedbackDelayLimitUs)
                             .arg(snapshot.runtimeTraceFromHardware ? 1 : 0)
                             .arg(snapshot.runtimeTraceFrameSequenceValid ? 1 : 0)
                             .arg(snapshot.runtimeTraceTimingReliable ? 1 : 0)
                             .arg(snapshot.runtimeTraceFifoCaughtUp ? 1 : 0)
                             .arg(snapshot.runtimeTraceLost ? 1 : 0)
                             .arg(snapshot.runtimeTraceLogicalFrameSequence)
                             .arg(snapshot.runtimeTraceUsageProfileGeneration)
                             .arg(snapshot.runtimeTraceConfigurationGeneration));
            return;
        }
    }
    for(int axisIndex = 0; axisIndex < motorCheckAxisCount; ++axisIndex){
        const AxisConfig& axis = cfg.axes[axisIndex];
        if(!axis.monitored || !axis.motionParticipant){
            continue;
        }
        if(endpointRemoteUsesTraceState){
            // Transition和等待Running首帧阶段尚未授权速度命令。Running后
            // 也只检查新发布的同帧状态，避免反复消费同一缓存帧。
            if(!endpointRemoteTraceRunning || !snapshotAdvanced){
                continue;
            }
            const bool stateAvailable =
                    axisIndex < static_cast<int>(snapshot.motorTraceStatusWord.size()) &&
                    axisIndex < static_cast<int>(snapshot.motorTraceStateMachine.size());
            const int stateMachine = stateAvailable ?
                        snapshot.motorTraceStateMachine[axisIndex] : -1;
            if(stateMachine != 4){
                const quint16 statusWord = stateAvailable ?
                            snapshot.motorTraceStatusWord[axisIndex] : 0;
                const QString statusWordHex =
                        QString::number(statusWord, 16)
                        .rightJustified(4, QLatin1Char('0'))
                        .toUpper();
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::MotorFault,
                             QStringLiteral("运动参与电机驱动状态异常"),
                             QStringLiteral(
                                 "轴%1正在参与末端遥控，但同帧0x6041=%2(0x%3)，解码状态=%4，要求=4(Operation enabled)，逻辑序号=%5；已按电机故障执行安全急停。")
                                 .arg(axisIndex + 1)
                                 .arg(statusWord)
                                 .arg(statusWordHex)
                                 .arg(stateMachine)
                                 .arg(snapshot.runtimeTraceLogicalFrameSequence));
                return;
            }
            continue;
        }
        if(!hardwareInterface->isMotorEnabled(axisIndex)){
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::MotorFault,
                         QStringLiteral("运动参与电机失能"),
                         QStringLiteral("轴 %1 正在参与运动，但驱动使能状态为失能，已按电机故障执行安全急停。")
                             .arg(axisIndex + 1));
            return;
        }
    }

    if(!cfg.forceSensorMonitoringEnabled){
        timedCableBreakForceZeroPending = false;
        timedCableBreakForceZeroSensorIndex = -1;
        timedCableBreakForceZeroTriggerTimeSec = -1.0;
        timedCableBreakForceZeroLastHealthyForce =
                std::numeric_limits<double>::quiet_NaN();
        injectedCableBreakPending = false;
        injectedCableBreakSensorIndex = -1;
        previousForceSensorValue.clear();
        std::fill(lowForceCycles.begin(), lowForceCycles.end(), 0);
        std::fill(highForceCycles.begin(), highForceCycles.end(), 0);
    }

    if(cfg.forceSensorMonitoringEnabled && timedCableBreakForceZeroPending){
        const int targetSensorIndex = timedCableBreakForceZeroSensorIndex;
        const int targetAxisIndex = findAxisIndexBySensor(cfg, targetSensorIndex);
        const bool validSensor =
                targetSensorIndex >= 0 &&
                targetSensorIndex < static_cast<int>(snapshot.forceSensorValue.size());
        if(targetAxisIndex < 0 || !validSensor){
            timedCableBreakForceZeroPending = false;
            timedCableBreakForceZeroSensorIndex = -1;
            timedCableBreakForceZeroLastHealthyForce =
                    std::numeric_limits<double>::quiet_NaN();
            triggerFault(StopLevel::EmergencyStop,
                         FaultCode::CableBreak,
                         QStringLiteral("故障注入：模拟断绳"),
                         QStringLiteral("定时张力置零断绳模拟未找到传感器通道%1对应的受监控绳索，已按断绳故障直接执行安全链路。")
                             .arg(targetSensorIndex + 1));
            return;
        }

        const AxisConfig& targetAxis = cfg.axes[targetAxisIndex];
        const double currentForce = snapshot.forceSensorValue[targetSensorIndex];
        if(std::isfinite(currentForce) &&
                targetAxis.forceMin > 1e-6 &&
                currentForce > targetAxis.forceMin){
            timedCableBreakForceZeroLastHealthyForce = currentForce;
        }

        double currentPvtTimeSec = 0.0;
        std::size_t currentPvtIndex = 0;
        const bool hasPvtProgress =
                hardwareInterface &&
                hardwareInterface->currentPvtProgress(currentPvtTimeSec, currentPvtIndex) &&
                std::isfinite(currentPvtTimeSec);
        Q_UNUSED(currentPvtIndex);
        if(hasPvtProgress &&
                currentPvtTimeSec >= timedCableBreakForceZeroTriggerTimeSec){
            if(static_cast<int>(previousForceSensorValue.size()) <= targetSensorIndex){
                previousForceSensorValue.resize(targetSensorIndex + 1, 0.0);
            }

            const bool previousHealthy =
                    std::isfinite(previousForceSensorValue[targetSensorIndex]) &&
                    targetAxis.forceMin > 1e-6 &&
                    previousForceSensorValue[targetSensorIndex] > targetAxis.forceMin;
            const bool cachedHealthy =
                    std::isfinite(timedCableBreakForceZeroLastHealthyForce) &&
                    targetAxis.forceMin > 1e-6 &&
                    timedCableBreakForceZeroLastHealthyForce > targetAxis.forceMin;
            if(!previousHealthy && cachedHealthy){
                previousForceSensorValue[targetSensorIndex] =
                        timedCableBreakForceZeroLastHealthyForce;
            }
            else if(!previousHealthy && !cachedHealthy){
                timedCableBreakForceZeroPending = false;
                timedCableBreakForceZeroSensorIndex = -1;
                timedCableBreakForceZeroLastHealthyForce =
                        std::numeric_limits<double>::quiet_NaN();
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::CableBreak,
                             QStringLiteral("故障注入：模拟绳索%1断裂")
                                 .arg(targetSensorIndex + 1),
                             QStringLiteral("PVT轨迹时间%1s到达定时张力置零点，但未缓存到上一帧健康张力，已按断绳模拟直接执行安全急停。")
                                 .arg(currentPvtTimeSec, 0, 'f', 3));
                return;
            }

            snapshot.forceSensorValue[targetSensorIndex] = 0.0;
            timedCableBreakForceZeroPending = false;
            timedCableBreakForceZeroSensorIndex = -1;
            timedCableBreakForceZeroLastHealthyForce =
                    std::numeric_limits<double>::quiet_NaN();
        }
    }

    if(cfg.forceSensorMonitoringEnabled && injectedCableBreakPending){
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
        triggerFault(StopLevel::EmergencyStop,
                     FaultCode::CableBreak,
                     summary,
                     detail + QStringLiteral(" 已直接执行安全急停。"));
        return;
    }

    if(cfg.workspaceMonitorEnabled && !cfg.singleCableForceDebugMode){
        // 工作空间监控只使用 MainWindow 提供的活动轨迹点或规划末点。
        // 单绳调试模式下可能没有有效轨迹位姿，因此跳过该项。
        if(!cfg.hasWorkspacePose || cfg.workspacePose.size() < 3){
            clearWorkspaceWarningState();
            workspaceExceededCycles = 0;
            workspaceMissingPoseCycles++;
            if(workspaceMissingPoseCycles >= std::max(cfg.poseTimeoutCycles, 1)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::SensorInvalid,
                             QStringLiteral("工作空间判定位姿超时"),
                             QStringLiteral("安全监控在运行期间连续 %1 个周期未收到有效的活动轨迹点或规划末点")
                                 .arg(workspaceMissingPoseCycles));
                return;
            }
        }
        else{
            workspaceMissingPoseCycles = 0;

            const double px = cfg.workspacePose[0];
            const double py = cfg.workspacePose[1];
            const double pz = cfg.workspacePose[2];
            if(!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::SensorInvalid,
                             QStringLiteral("工作空间判定位姿无效"),
                             QStringLiteral("活动轨迹点或规划末点存在非数值坐标"));
                return;
            }

            const double overflowX = std::max(cfg.workspaceXMin - px, px - cfg.workspaceXMax);
            const double overflowY = std::max(cfg.workspaceYMin - py, py - cfg.workspaceYMax);
            const double overflowZ = std::max(cfg.workspaceZMin - pz, pz - cfg.workspaceZMax);
            const double maxOverflow = std::max({0.0, overflowX, overflowY, overflowZ});

            if(maxOverflow > 0.0){
                clearWorkspaceWarningState();
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
                    clearWorkspaceWarningState();
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
        const bool skipLowForceAndBreakChecks = cfg.singleCableForceDebugMode;

        if(axis.monitorForce && axis.sensorIndex >= 0 && axis.sensorIndex < sensorCount){
            const double forceValue = snapshot.forceSensorValue[axis.sensorIndex];
            if(!std::isfinite(forceValue)){
                triggerFault(StopLevel::EmergencyStop,
                             FaultCode::SensorInvalid,
                             QStringLiteral("张力传感器数据无效"),
                             QStringLiteral("传感器通道 %1 反馈了非数值张力数据")
                                 .arg(axis.sensorIndex + 1));
                return;
            }

            if(axis.forceMax > 1e-6){
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
                    triggerFault(StopLevel::EmergencyStop,
                                 FaultCode::CableBreak,
                                 QStringLiteral("检测到疑似断绳/断崖式失张"),
                                 QStringLiteral("轴%1张力由%2快速跌落至%3，已直接执行安全急停。")
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

            const bool motorPositionLimitRecoveryAxis =
                    cfg.motorPositionLimitRecoveryActive &&
                    axisIndex < static_cast<int>(cfg.motorPositionLimitRecoveryAxes.size()) &&
                    cfg.motorPositionLimitRecoveryAxes[axisIndex];
            if(axis.motorMax > axis.motorMin &&
                    (rawPos >= axis.motorMax || rawPos <= axis.motorMin) &&
                    !motorPositionLimitRecoveryAxis){
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
