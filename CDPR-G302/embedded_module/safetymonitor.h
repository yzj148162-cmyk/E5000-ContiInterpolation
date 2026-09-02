#ifndef SAFETYMONITOR_H
#define SAFETYMONITOR_H

/*
 * 文件总览：
 * - SafetyMonitor 是集中安全判据模块，负责监控硬件连接、快照超时、绳力上下限、断绳、行程、速度、工作空间和软件看门狗。
 * - Config 从 MainWindow 汇总当前运行状态与限制参数，FaultCode/StopLevel 决定告警、受控停机、安全停机或急停。
 * - 触发故障后会锁存 faultLatched，避免故障消失瞬间自动恢复运动。
 */

#include <QObject>
#include <QMutex>
#include <QString>
#include <limits>
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
        MotorTorqueExceeded = 11,
        MotorFault = 12,
        PlcCommunicationFault = 13,
        StartupSelfCheckFailed = 14,
        ControlBoxButtonNotReset = 15
    };
    Q_ENUM(FaultCode)

    struct AxisConfig {
        bool monitored = false;
        bool motionParticipant = false;
        bool monitorForce = false;
        int sensorIndex = -1;
        double forceMin = 0.0;
        double forceMax = 0.0;
        double motorMin = 0.0;
        double motorMax = 0.0;
        double motorVelMax = 0.0;
    };

    struct Config {
        int axisCount = 0;
        int sensorCount = 0;
        double cycleMs = 10.0;
        bool monitorEnabled = true;
        bool systemRunning = false;
        bool safetyArmed = false;
        bool hardwareConnected = false;
        bool motionActive = false;
        bool commissioningMode = false;
        bool commissioningHardwareCommandActive = false;
        bool commissioningControlSnapshotStartupActive = false;
        int commissioningAxisIndex = -1;
        qint64 commissioningMotionStartMs = -1;
        // 单轴调试单次连续运行上限；0 表示关闭时间监控，其他安全判据仍保持启用。
        int commissioningMotionTimeoutMs = 0;
        bool forceThreadRunning = false;
        // 在线速度专用 Trace 不含力传感器时关闭所有力传感器依赖；电机位置、
        // 速度和反馈力矩保护不受该开关影响。
        bool forceSensorMonitoringEnabled = true;
        bool singleCableForceDebugMode = false;
        bool motorPositionLimitRecoveryActive = false;
        int snapshotTimeoutMs = 300;
        int persistentFaultCycles = 4;
        double breakForceRatio = 0.2;
        double breakDropRatio = 0.65;
        double severeForceOverRatio = 1.15;
        double severeSpeedOverRatio = 1.2;
        bool workspaceMonitorEnabled = false;
        // 工作空间判定只接收活动轨迹点或规划末点，不接收动捕或正运动学估计位姿。
        bool hasWorkspacePose = false;
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
        std::vector<double> workspacePose;
        std::vector<AxisConfig> axes;
        std::vector<bool> motorPositionLimitRecoveryAxes;
    };

    // 创建安全监控对象；具体数据源由 MainWindow 注入。
    explicit SafetyMonitor(QObject* parent = nullptr);

    // 绑定力控 worker，用于读取最新控制快照和传感器反馈。
    void setControlWorker(ControlWorker* worker);
    // 绑定硬件接口，用于执行急停或读取硬件连接状态。
    void setHardwareInterface(HardwareInterface* hardware);
    // 更新安全规则和运行状态快照，供下一次 evaluateSafety 使用。
    void setConfig(const Config& config);
    // 写入主线程心跳时间戳，软件看门狗用它判断 UI/主线程是否卡死。
    void updateMainThreadHeartbeat(qint64 timestampMs);

public slots:
    // 启动周期性安全检查。
    void start();
    // 停止周期性安全检查。
    void stop();
    // 清除已锁存故障，允许用户在故障解除后重新开始。
    void clearFaultLatch();
    // 手动触发软件急停，通常来自 UI 或外部控制盒。
    void requestEmergencyStop(const QString& reason = QStringLiteral("软件急停触发"));
    // 注入指定传感器断绳故障，用于安全逻辑测试。
    void requestInjectedCableBreak(int sensorIndex);
    // 注入主线程心跳异常，用于验证软件看门狗故障链路。
    void requestInjectedMainThreadHeartbeatTimeout();
    // 在 PVT 轨迹到达指定时间后，将指定张力通道的安全监控快照置零，用于物理断绳模拟测试。
    void requestTimedCableBreakForceZero(int sensorIndex, double triggerTimeSec);
    // 取消尚未触发的定时张力置零断绳模拟。
    void cancelTimedCableBreakForceZero();

signals:
    void faultDetected(int level, int code, QString summary, QString detail);
    // 工作空间边缘条件解除时通知主界面清除非锁存预警显示。
    void warningCleared(int code);

private:
    // 汇总当前配置、控制快照和硬件状态，执行一次完整安全判据评估。
    void evaluateSafety();
    // 线程安全地复制当前配置。
    Config currentConfig() const;
    // 重置计数器、断绳状态和工作空间状态；可选择同时清故障锁存。
    void resetDynamicState(bool clearLatchedFault);
    // 统一上报故障并锁存，避免同一故障重复触发。
    void triggerFault(StopLevel level,
                      FaultCode code,
                      const QString& summary,
                      const QString& detail);
    // 根据力传感器编号反查对应绳索/电机轴。
    int findAxisIndexBySensor(const Config& cfg, int sensorIndex) const;
    // 直接调用硬件接口执行急停，作为最高等级停机动作。
    bool issueDirectEmergencyStop() const;
    // 将看门狗/故障事件追加到结构化日志，便于事后追溯。
    void appendWatchdogEventLog(const Config& cfg,
                                const QString& eventType,
                                FaultCode code,
                                const QString& summary,
                                const QString& detail,
                                bool stopActionAttempted,
                                bool stopActionSucceeded,
                                const QString& note = QString()) const;
    // 保证各轴/传感器的连续故障计数数组尺寸与当前配置一致。
    void ensureAxisStateSize(int axisCount, int sensorCount);
    // 清除工作空间监控的连续越界/缺失位姿状态。
    void resetWorkspaceState();
    // 清除工作空间边缘活动标志，并只在状态发生变化时发出解除通知。
    void clearWorkspaceWarningState();

    ControlWorker* controlWorker = nullptr;
    HardwareInterface* hardwareInterface = nullptr;
    QTimer* timer = nullptr;

    mutable QMutex configMutex;
    Config config;

    bool faultLatched = false;
    bool manualEmergencyRequested = false;
    QString manualEmergencyReason;
    bool injectedCableBreakPending = false;
    int injectedCableBreakSensorIndex = -1;
    bool injectedMainThreadHeartbeatTimeoutPending = false;
    bool timedCableBreakForceZeroPending = false;
    int timedCableBreakForceZeroSensorIndex = -1;
    double timedCableBreakForceZeroTriggerTimeSec = -1.0;
    double timedCableBreakForceZeroLastHealthyForce = std::numeric_limits<double>::quiet_NaN();
    int mainThreadHeartbeatTimeoutCycles = 0;

    quint64 lastSnapshotSequence = 0;
    qint64 lastSnapshotUpdateMs = -1;
    bool hasSeenSnapshot = false;
    qint64 lastControllerCommunicationCheckMs = -1;
    qint64 commissioningTraceValidationStartMs = -1;

    std::vector<double> previousForceSensorValue;
    std::vector<int> lowForceCycles;
    std::vector<int> highForceCycles;
    std::vector<int> overSpeedCycles;
    int workspaceMissingPoseCycles = 0;
    int workspaceExceededCycles = 0;
    bool workspaceNearBoundaryActive = false;
};

#endif // SAFETYMONITOR_H
