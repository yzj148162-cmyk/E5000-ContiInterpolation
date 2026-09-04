#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/*
 * 文件总览：
 * - MainWindow 是整套 G302 控制软件的调度中心，连接 UI、硬件、轨迹规划、仿真、力控、安全监控和外部通信。
 * - 头文件中集中声明运行状态机、UI 事件槽、线程/worker 指针、参数缓存以及各类流程辅助函数。
 * - 阅读时先看 RobotRuntimeState 和各 enum，理解“系统运行、位置/PVT、力控、校准、安全停机”等状态如何互斥。
 */

#include <QMainWindow>
#include <qt_windows.h>
#include <QDebug>
#include <QMessageBox>
#include <QtDataVisualization> // 3D鍙鍖?
#include <QAbstract3DInputHandler>
#include <QElapsedTimer>
#include <QDateTime>
#include <QFileDialog>// 璇诲彇鏂囦欢
#include "intbtn.h"
#include "infolabel.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <array>
#include <cstddef>
#include <deque>
#include <functional>
#include <utility>
#include <vector>
#include <QMetaType>
#include <QThread>
#include <QTime>
#include <QTextEdit>
#include <QJsonArray>
#include <QJsonObject>
#include <QRadioButton>
#include <QStringList>
#include <QVector>

#include "macro.h"
#include "kalmanhandler.h"
#include "curvedrawer.h"
#include "positionsimulationmodel.h"
#include "motivelocalhandlerthread.h"
#include "controlworker.h"
#include "forwardkinematicssolver.h"
#include "pvtexecutionworker.h"
#include "simulationworker.h"
#include "hardwareinterface.h"
#include "datavisualizationcontroller.h"
#include "guitimingdiagnostics.h"
#include "runtimediagnostics.h"
#include "sessionrecorder.h"
#include "udppackettypes.h"

#include<QMetaType>// 淇″彿妲藉嚱鏁版湰涓嶆敮鎸乻td::vector,std::string绛夛紝鍥犳闇€瑕佹敞鍐岋紙鏋勯€犲嚱鏁癕ainWindow涔熼渶瑕佹坊鍔犱竴閮ㄥ垎锛?
Q_DECLARE_METATYPE(std::vector<double>)
Q_DECLARE_METATYPE(std::vector<int>)
Q_DECLARE_METATYPE(std::vector<std::vector<double>>)
Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(QVector<QVector3D>)

# pragma execution_character_set("utf-8")

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QDialog;
class QEvent;
class QGridLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QTableWidget;
class QTextStream;
class QTimer;
class EndpointRemoteInputSupervisor;
class ForceInteractionValidationWorker;
class MotorTorqueTestWorker;
class MonitorThread;
class SafetyMonitor;
class StructuredFaultLogWriter;
class UdpCommWorker;

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtDataVisualization;// 浣跨敤DataVisualization缁樺埗涓夌淮妯″瀷(QT6.*鍙敞閲婃湰娈?
#endif

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // 构造主窗口，初始化 UI、参数、worker 线程、硬件接口和信号槽。
    MainWindow(QWidget *parent = nullptr);
    // 析构时停止线程、释放 UI 和运行期资源。
    ~MainWindow();

    enum class PositionMotionState {
        Idle,
        Simulating,
        ExecutingPvt,
        PausedPvt,
        ExecutingOnlineVelocity
    };

    enum class ForceMotionState {
        Idle,
        DirectForceControl,
        HybridPoseForceReady,
        HybridPoseForceTrajectory
    };

    enum class RunMode {
        ProgramControl,
        RealtimeTrajectory,
        OnlineVelocityControl,
        SimulationCalculation
    };

    enum class CableHomeState {
        Unconfirmed,
        WaitingForceStable,
        Confirmed
    };

    enum class HardwareRunState {
        Disconnected,
        ControllerConnected,
        SingleAxisCommissioning,
        FullRobotReady
    };

    enum class CalibrationWorkflowStage {
        Idle,
        MechanicalHoming,
        AwaitingPretensionStart,
        PretensionBalancing,
        UpdatingSpatialBaseline,
        AwaitingConfirm,
        Completed,
        Stopped
    };

    struct RobotStateSnapshot {
        bool systemRunning = false;
        bool lsConnected = false;
        bool controlThreadRunning = false;
        CableHomeState cableHomeState = CableHomeState::Unconfirmed;
        bool posModeRunning = false;
        bool forceControlThreadChecked = false;
        bool forceControlThreadRunning = false;
        bool pvtTrajectoryAvailable = false;
        bool pvtMotionRunning = false;
        bool pvtMotionPaused = false;
        bool pvtMotionFinished = true;
        bool anyMotionRunning = false;
        RunMode runMode = RunMode::ProgramControl;
        PositionMotionState positionMotionState = PositionMotionState::Idle;
        ForceMotionState forceMotionState = ForceMotionState::Idle;
    };

    // 汇总 UI、运行态和硬件状态，供按钮联锁、安全监控和外部状态上报使用。
    RobotStateSnapshot currentRobotState(bool queryHardware = true) const;

protected:
    // 拦截窗口关闭事件，先停机停线程再允许退出。
    void closeEvent(QCloseEvent *event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    enum class GuiRefreshProfile {
        Normal = 0,
        EndpointRemoteFrozen
    };
    enum class UdpRuntimeProfile {
        Inactive = 0,
        JsonStatus200ms,
        V9Feedback10ms
    };
    enum class MotorFeedbackDisplayUnit {
        Degree,
        Revolution
    };
    enum class PretensionMode {
        ZeroPose,
        RestoredRuntimePose
    };
    Ui::MainWindow *ui;
    // 读取 UI/默认配置并初始化项目运行参数。
    bool initPara();
    // 向电机下发指令前弹出人工确认，取消时不继续执行。
    bool confirmMotorCommandFromUi(const QString& actionName,
                                   const QString& detail = QString());

    struct RobotRuntimeState {
        bool systemRunning = false;
        HardwareRunState hardwareRunState = HardwareRunState::Disconnected;
        bool safetyArmed = false;
        int commissioningAxisIndex = -1;
        qint64 commissioningMotionStartMs = -1;
        bool posModeRunning = false;
        bool pvtCommandActive = false;
        bool pvtMotionProtectedActive = false;
        std::vector<int> pvtMotionProtectedAxis;
        qint64 pvtMotionProtectedStartMs = 0;
        bool hardwareExclusiveCommandActive = false;
        int hardwareExclusiveCommandDepth = 0;
        int hardwareExclusiveSnapshotTimeoutMs = 0;
        // 仅标记 Lite 单轴调试页面主动发起的硬件调用；用于临时放宽该
        // 路径的看门狗阈值，不影响 G3、Lite 完整启动或其他运行路径。
        bool liteCommissioningHardwareCommandActive = false;
        // Lite 首次使能后，ControlWorker 在线程启动到首帧快照之间允许
        // 一个有界启动窗口；首帧到达后立即恢复正常快照超时判据。
        bool liteCommissioningControlStartupActive = false;
        quint64 liteCommissioningControlStartupSequence = 0;
        bool hybridPoseForceModeActive = false;
        bool singleCableForceDebugMode = false;
        bool motorTorqueDebugActive = false;
        bool singleMotorPointMoveActive = false;
        bool jogFollowTestActive = false;
        bool onlineVelocityControlActive = false;
        // 六维力交互阶段B复用在线速度硬件链，但保留独立会话标志，
        // 防止预设轨迹/遥控的停止与收尾逻辑误处理该会话。
        bool forceInteractionRuntimeActive = false;
        bool endpointRemoteControlActive = false;
        int singleMotorPointMoveAxis = -1;
        qint64 singleMotorPointMoveStartMs = 0;
        bool linearModuleHeightMoveActive = false;
        std::vector<int> linearModuleHeightMoveAxis;
        qint64 linearModuleHeightMoveStartMs = 0;
        bool linearModuleTraceRecoveryPvtActive = false;
        bool servoHoldActive = false;
        RunMode runMode = RunMode::ProgramControl;
        CableHomeState cableHomeState = CableHomeState::Unconfirmed;
        qint64 cableHomeConfirmedTimestampMs = -1;
        PretensionMode pretensionMode = PretensionMode::ZeroPose;
        bool posePvtTrajectoryExecutedSinceConnect = false;
        bool autoExecutePoseAfterSimulation = false;
        bool safetyFaultLatched = false;
        int safetyStopLevel = 0;
        int safetyFaultCode = 0;
        qint64 safetyFaultOccurredMs = -1;
        QString safetyFaultSummary;
        QString safetyFaultDetail;
        bool safetyWarningActive = false;
        int safetyWarningCode = 0;
        qint64 safetyWarningOccurredMs = -1;
        QString safetyWarningSummary;
        QString safetyWarningDetail;
        bool safetyWarningInjected = false;
        bool motorPositionLimitRecoveryActive = false;
        std::vector<bool> motorPositionLimitRecoveryAxes;
        bool pendingFaultInjectionCause = false;
        int pendingFaultInjectionStopLevel = 0;
        int pendingFaultInjectionFaultCode = 0;
        QString pendingFaultInjectionSummary;
        QString pendingFaultInjectionDetail;
        bool startupSelfCheckCompleted = false;
        bool startupSelfCheckPassed = false;
        qint64 startupSelfCheckMs = -1;
        QString startupSelfCheckSummary;
        QString startupSelfCheckDetail;
        bool cableBreakPhysicalSimulationArmed = false;
        int cableBreakPhysicalSimulationSensorIndex = -1;
        double cableBreakPhysicalSimulationTriggerTimeSec = -1.0;

        int controlBoxMotionControlState = -1;
        int controlBoxSpeedZeroState = -1;
        int controlBoxZeroCalibState = 0;
        int controlBoxSoftwareEmergencyStopState = -1;
        qint64 controlBoxLastStatusMs = -1;
        qint64 controlBoxMonitorStartedMs = -1;
        bool controlBoxBrakeReleaseRequired = false;
        bool controlBoxSpeedZeroReleaseRequired = false;
        bool controlBoxOverspeedButtonLatchPending = false;
        bool controlBoxOverspeedButtonSafetyLatchActive = false;
        bool controlBoxCableSpeedValid = false;
        double controlBoxMaxAbsCableSpeedMmPerSec = 0.0;
        int controlBoxMaxCableSpeedAxis = -1;
        quint64 controlBoxCableSpeedSnapshotSequence = 0;
        qint64 controlBoxCableSpeedUpdatedMs = -1;
        bool controlBoxStartRequiresStop = true;
        bool controlBoxStartInterlockReported = false;
        bool controlBoxPausedPvtMotion = false;
        bool controlBoxPausedForceControl = false;
        bool mainPauseSpeedZeroActive = false;
    };

    struct StartupSelfCheckResult {
        bool passed = false;
        QString summary;
        QString detail;
        QStringList passedItems;
        QStringList warningItems;
        QStringList failedItems;
        QJsonArray itemRecords;
    };

    struct HybridPoseForceModeConfig {
        bool enabled = false;
        std::vector<int> forceAxisIndex;
        std::vector<int> forceSensorIndex;
        std::vector<double> frozenExpectedForce;
    };

    struct LinearModuleHeightReference {
        bool valid = false;
        int logicalAxis = -1;
        int hardwareAxis = -1;
        int slaveId = 0;
        double referenceHeightM = 0.0;
        qint64 referenceTraceCommandRawPulse = 0;
        double referenceAxisEquiv = 0.0;
    };

    struct WorkspaceBounds {
        double xMin = 0.0;
        double xMax = 0.0;
        double yMin = 0.0;
        double yMax = 0.0;
        double zMin = 0.0;
        double zMax = 0.0;
        double warningMargin = 0.0;
        double severeOverflow = 0.0;
    };

    using RuntimeDiagnosticsSample = RuntimeDiagnostics::Sample;
    using RuntimeDiagnosticsSummary = RuntimeDiagnostics::Summary;

    struct TestEvidenceRecord {
        QString itemName;
        QString testId;
        QString stepNo;
        QString inputAction;
        QString expectedResult;
        QString status;
        QString detail;
        QString evidencePath;
        QString resultCode;
        qint64 timestampMs = 0;
    };

    using SessionRecordingPvtPositionCommandTable =
            SessionRecorder::PvtPositionCommandTable;
    using SessionRecordingPvtControlCyclePoint =
            SessionRecorder::PvtControlCyclePoint;
    using SessionRecordingPvtControlCycleRecord =
            SessionRecorder::PvtControlCycleRecord;
    using SessionRecordingSample = SessionRecorder::Sample;
    using SessionRecordingState = SessionRecorder::State;

    RobotRuntimeState runtimeState;
    std::vector<double> controlBoxCableSpeedLastMotorAbsPos;
    qint64 controlBoxCableSpeedLastEncoderSampleMs = -1;
    SessionRecordingState sessionRecordingState;
    std::vector<SessionRecordingPvtControlCycleRecord> lastSuccessfulPvtControlCycleRecords;
    SessionRecordingPvtControlCycleRecord pendingPvtControlCycleRecord;
    bool pendingPvtControlCycleRecordValid = false;

    // 根据系统运行按钮状态启动或关闭整机运行流程。
    void runSwitch();
    void runFullSystemSwitch();

    struct LiteAxisCommissioningState {
        bool online = false;
        bool configured = false;
        bool enabled = false;
        bool sessionZeroValid = false;
        double sessionZeroAbsolute = 0.0;
        int stateMachine = -1;
        int busState = -1;
        int errorCode = 0;
        int apiResult = 0;
    };
    std::vector<LiteAxisCommissioningState> liteAxisCommissioningStates;
    HardwareInterface::ConnectionItemDiagnostics liteCommissioningControllerDiagnostics;
    bool liteCommissioningControllerDiagnosticsValid = false;

    QWidget* liteCommissioningTab = nullptr;
    QComboBox* liteCommissioningTraceTopologyCombo = nullptr;
    QComboBox* liteCommissioningAxisCombo = nullptr;
    QLabel* liteCommissioningBannerLabel = nullptr;
    QLabel* liteCommissioningControllerStatusLabel = nullptr;
    QLabel* liteCommissioningBusStatusLabel = nullptr;
    QLabel* liteCommissioningAxisStatusLabel = nullptr;
    QLabel* liteCommissioningSessionStatusLabel = nullptr;
    QDoubleSpinBox* liteCommissioningMaxTravelSpin = nullptr;
    QDoubleSpinBox* liteCommissioningMaxVelocitySpin = nullptr;
    QDoubleSpinBox* liteCommissioningMaxTorqueSpin = nullptr;
    QDoubleSpinBox* liteCommissioningMaxForceSpin = nullptr;
    QDoubleSpinBox* liteCommissioningMaxDurationSpin = nullptr;
    QPushButton* liteCommissioningConnectionButton = nullptr;
    QPushButton* liteCommissioningRefreshButton = nullptr;
    QPushButton* liteCommissioningApplyAxisButton = nullptr;
    QPushButton* liteCommissioningClearBusButton = nullptr;
    QPushButton* liteCommissioningClearAxisButton = nullptr;
    QPushButton* liteCommissioningEnableButton = nullptr;
    QPushButton* liteCommissioningDisableButton = nullptr;
    QPushButton* liteCommissioningSetZeroButton = nullptr;
    QPushButton* liteCommissioningPrepareForceButton = nullptr;
    QPushButton* liteFullStartupButton = nullptr;
    QTimer* liteCommissioningRefreshTimer = nullptr;

    void setupLiteCommissioningTab();
    void updateLiteCommissioningTabAvailability();
    bool isLiteTemplateActive() const;
    // Lite 位置模式 UI 直接使用程序内部平台坐标；G3 保留原 15° Rx 偏置。
    double positionModeUiRxOffsetRad() const;
    HardwareInterface::LiteRuntimeTraceTopology selectedLiteRuntimeTraceTopology() const;
    bool liteCommissioningAxisAvailableInSelectedTopology(int axisIndex) const;
    int selectedLiteCommissioningAxis() const;
    void populateLiteCommissioningAxisOptions();
    // 在 HardwareThread 执行 Lite 调试硬件调用；GUI等待期间周期喂狗。
    void runLiteCommissioningHardwareCommand(
            const std::function<void()>& work,
            const QString& operationName = QString());
    void refreshLiteCommissioningDiagnostics(bool announce = false);
    void refreshLiteCommissioningUiState();
    void resetLiteCommissioningState(bool clearAxisSelection = false);
    void toggleLiteControllerConnection();
    void configureSelectedLiteAxis();
    void clearLiteBusError();
    void clearSelectedLiteAxisError();
    void setSelectedLiteAxisEnabled(bool enabled);
    void setSelectedLiteAxisSessionZero();
    void prepareSelectedLiteAxisForceControl();
    void startLiteFullSystemFromUi();
    bool ensureLiteControllerReady(const QString& actionName, bool requireBusOperational = true);
    bool ensureLiteAxisReady(int axisIndex,
                             const QString& actionName,
                             bool requireConfigured,
                             bool requireEnabled,
                             bool requireSessionZero);
    bool liteCommissioningActionActive() const;
    bool liteCommissioningSafetyArmed() const;
    std::vector<bool> liteCommissioningMotionParticipantMask(int axisCount) const;
    void syncSelectedAxisAcrossDebugPages(int axisIndex);
    void appendLiteCommissioningEvent(const QString& eventType,
                                      int axisIndex,
                                      bool success,
                                      const QString& detail);

    QWidget* onlineVelocityTestTab = nullptr;
    QComboBox* onlineVelocityPeriodCombo = nullptr;
    QCheckBox* onlineVelocityFeedForwardCheck = nullptr;
    QCheckBox* onlineVelocityPidCheck = nullptr;
    QDoubleSpinBox* onlineVelocityFeedForwardGainSpin = nullptr;
    QDoubleSpinBox* onlineVelocityKpSpin = nullptr;
    QDoubleSpinBox* onlineVelocityKiSpin = nullptr;
    QDoubleSpinBox* onlineVelocityKdSpin = nullptr;
    QDoubleSpinBox* onlineVelocityIntegralLimitSpin = nullptr;
    QDoubleSpinBox* onlineVelocityCorrectionLimitSpin = nullptr;
    QDoubleSpinBox* onlineVelocityMaxVelocitySpin = nullptr;
    QDoubleSpinBox* onlineVelocityMaxAccelerationSpin = nullptr;
    QDoubleSpinBox* onlineVelocityChangeTimeSpin = nullptr;
    QPushButton* onlineVelocityPrepareButton = nullptr;
    QPushButton* onlineVelocityStartButton = nullptr;
    QPushButton* onlineVelocityStopButton = nullptr;
    QLabel* onlineVelocityPlanStatusLabel = nullptr;
    QLabel* onlineVelocityRunStatusLabel = nullptr;
    QLabel* onlineVelocityTimingStatusLabel = nullptr;
    QLabel* onlineVelocityRecordStatusLabel = nullptr;
    QTableWidget* onlineVelocityAxisTable = nullptr;
    QDoubleSpinBox* endpointRemoteSpeedSpin = nullptr;
    QDoubleSpinBox* endpointRemoteAccelerationSpin = nullptr;
    QDoubleSpinBox* endpointRemoteAngularSpeedSpin = nullptr;
    QDoubleSpinBox* endpointRemoteAngularAccelerationSpin = nullptr;
    QPushButton* endpointRemoteEnterButton = nullptr;
    QPushButton* endpointRemoteExitButton = nullptr;
    std::array<QPushButton*, 6> endpointRemoteDirectionButtons{};
    QLabel* endpointRemoteStateLabel = nullptr;
    QLabel* endpointRemoteInitialPoseLabel = nullptr;
    QLabel* endpointRemoteDesiredPoseLabel = nullptr;
    QLabel* endpointRemoteVelocityLabel = nullptr;
    QLabel* endpointRemoteBoundaryGuardLabel = nullptr;
    QLabel* endpointRemoteWorkspaceLabel = nullptr;
    QLabel* endpointRemoteVoxelAngleLabel = nullptr;
    QLabel* endpointRemoteBoundaryLabel = nullptr;
    QCheckBox* endpointRemoteX56FixedBindingCheck = nullptr;
    QComboBox* endpointRemoteX56DeviceCombo = nullptr;
    QLabel* endpointRemoteX56StatusLabel = nullptr;
    QString endpointRemoteX56BoundInstanceId;
    std::array<bool, 6> endpointRemoteButtonHeld{};
    std::array<bool, 6> endpointRemoteKeyHeld{};
    quint64 endpointRemoteLastBoundaryEventSequence = 0;
    bool endpointRemoteFinalizing = false;
    bool endpointRemoteStopRequested = false;
    qint64 endpointRemoteLastPeriodicUiRefreshMs = -1;
    qint64 endpointRemoteLastControlSnapshotUiApplyMs = -1;
    qint64 lastForwardKinematicsUiRefreshMs = -1;
    OnlineVelocityPlan onlineVelocityPreparedPlan;
    std::vector<double> onlineVelocityPreparedEndPose;
    struct OnlineVelocityPreparedContext {
        bool valid = false;
        qint64 preparedAtMs = -1;
        qint64 cableHomeConfirmedTimestampMs = -1;
        quint64 traceSequence = 0;
        int axisCount = 0;
        int endCount = 0;
        int periodUs = 0;
        std::vector<double> startPose;
        std::vector<double> endPose;
        std::vector<double> sourceTimeStamp;
        std::vector<std::vector<double>> sourceMotorPosition;
        OnlineVelocityAxisArray plannedStartMotorPosition{};
        OnlineVelocityAxisArray plannedEndMotorPosition{};
        OnlineVelocityAxisArray capturedStartMotorPosition{};
    };
    OnlineVelocityPreparedContext onlineVelocityPreparedContext;

    void setupOnlineVelocityTestTab();
    OnlineVelocityConfig onlineVelocityConfigFromUi() const;
    void updateEndpointRemoteBoundaryGuardLabel();
    bool prepareOnlineVelocityTestPlan();
    bool captureReliableOnlineVelocityMotorPosition(
            OnlineVelocityAxisArray& motorPosition,
            quint64* traceSequence,
            QString& errorMessage);
    bool validateOnlineVelocityPreparedStartState(
            OnlineVelocityAxisArray& currentMotorPosition,
            QString& errorMessage);
    bool validateOnlineVelocityShiftedMotorLimits(
            const OnlineVelocityAxisArray& actualStartMotorPosition,
            QString& errorMessage) const;
    void invalidateOnlineVelocityPreparedPlan(const QString& reason = QString(),
                                              bool announce = false);
    void finalizeOnlineVelocitySession(const OnlineVelocityStatus& status);
    void startOnlineVelocityTest();
    void stopOnlineVelocityTest(bool emergency = false,
                                const QString& reason = QStringLiteral("用户停止"));
    void refreshOnlineVelocityTestUi();
    EndpointRemoteConfig endpointRemoteConfigFromUi(
            const std::vector<double>& initialPose,
            const OnlineVelocityAxisArray& preparedMotorPosition,
            const std::vector<WinchCompensation::AxisConfig>& winchConfig,
            const EndpointRemoteVoxelAngleMap& voxelAngleLimits) const;
    QString endpointRemoteVoxelAngleCsvPath() const;
    void enterEndpointRemoteControl();
    void stopEndpointRemoteControl(bool emergency = false,
                                   const QString& reason = QStringLiteral("用户退出末端遥控"));
    void finalizeEndpointRemoteSession(const EndpointRemoteStatus& status);
    // GUI_PERF_DIAG：先关闭采样profile，再输出内存汇总，避免诊断自身污染计时。
    void finishEndpointRemoteGuiTimingDiagnostics(quint64 sessionToken);
    void setEndpointRemoteDirectionState(int directionIndex,
                                         bool pressed,
                                         bool fromKeyboard);
    void clearEndpointRemoteDirectionState(bool sendUpdate = true);
    // 用按钮实际按下状态和 Windows 物理键状态清除漏掉释放事件后残留的方向；
    // 只允许把已确认的方向清零，不会凭平台轮询主动产生新的运动方向。
    bool reconcileEndpointRemotePhysicalInputState();
    std::array<double, 3> endpointRemoteDirectionVector() const;
    void publishEndpointRemoteUiState();
    void applyEndpointRemoteX56Configuration();
    void updateEndpointRemoteX56DeviceList(
            const QStringList& displayNames,
            const QStringList& instanceIds);
    void setupForceInteractionValidationTab();
    void refreshForceInteractionValidationInputState();
    void startForceInteractionSoftwareValidation();
    void cancelForceInteractionSoftwareValidation();
    ForceInteractionRuntimeConfig forceInteractionRuntimeConfigFromUi(
            QString* errorMessage = nullptr);
    void prepareForceInteractionRuntimeFromUi();
    void startForceInteractionRuntime();
    void stopForceInteractionRuntime(bool emergency = false,
                                     const QString& reason = QStringLiteral("用户停止阶段B"));
    void refreshForceInteractionRuntimeUi();
    void finalizeForceInteractionRuntimeSession(
            const ForceInteractionRuntimeStatus& status);
    bool computeForceInteractionRuntimeForwardPose(
            const ForceInteractionRuntimeStatus& status,
            std::vector<double>& pose,
            int* equationCount = nullptr);

    // 涓荤嚎绋嬨€備负浠€涔堜娇鐢ㄧ嚎绋嬭€屼笉鐢╭t鐨凲Timer锛堝弬鑰冨姩鎬佹洸绾跨粯鍒剁殑渚嬬▼锛夛紵
    // 鍘熷洜鈶狅細澶氱嚎绋嬪垎鎷呰繍绠楀帇鍔?
    // 鍘熷洜鈶★細QTimer浣跨敤QT鑷甫璁℃椂锛岃€岃嚜宸卞啓鐨勭嚎绋嬪彲浠ョ敤鑷繁鎯崇敤鐨勮鏃舵柟寮忥紝姣斿鍩轰簬绯荤粺鏃堕棿璁℃椂銆傝繖鏍风殑璇濆彲浠ユ湁鏁堥伩鍏嶇嚎绋嬫椂闂存埑涓嶅悓姝ョ殑鎯呭喌
    // 姣斿璁㏑OS绾跨▼涔熺敤绯荤粺鏃堕棿浣滀负鎺у埗鍛ㄦ湡鐨勫熀鍑嗭紝杩欐牱QT鍜孯OS鐢变簬閮界敤绯荤粺鏃堕棿锛屽洜姝ゅ彲鍚屾鏃堕棿鎴?
    // 鍘熷洜鈶細绾跨▼鏄被锛屽熀浜庨潰鍚戝璞＄殑鎬濊矾锛屽啓璧锋潵鏇磋嚜鐢憋紝涓斾笉鐢ㄥ儚QTimer鐢ㄥぇ閲廲onnect鏉ヨ繘琛屾帶鍒跺懆鏈熶笌鎺у埗鎿嶄綔鐨勬槧灏?
    SimulationWorker* simulationWorker = nullptr;

    QThread* hardwareThread = nullptr;
    ControlWorker* controlWorker = nullptr;
    QThread *ccThread = nullptr;
    MotorTorqueTestWorker* motorTorqueTestWorker = nullptr;
    QThread* motorTorqueTestThread = nullptr;
    QThread* structuredFaultLogThread = nullptr;
    StructuredFaultLogWriter* structuredFaultLogWriter = nullptr;
    QTimer* controlSnapshotTimer = nullptr;
    // GUI_PERF_DIAG：单一显式profile；删除本成员及全部同标记接入点即可移除。
    mutable GuiTimingDiagnostics guiTimingDiagnostics;
    QThread* endpointRemoteInputSupervisorThread = nullptr;
    EndpointRemoteInputSupervisor* endpointRemoteInputSupervisor = nullptr;
    ForceInteractionValidationWorker* forceInteractionValidationWorker = nullptr;
    ForwardKinematicsSolver forceInteractionRuntimeForwardSolver;
    std::vector<double> forceInteractionRuntimeInitialPoseMmRad;
    std::vector<double> forceInteractionRuntimeLastForwardPose;
    int forceInteractionRuntimeLastForwardEquationCount = 0;
    qint64 forceInteractionRuntimeLastForwardSolveMs = -1;
    bool forceInteractionRuntimeFinalizing = false;
    quint64 endpointRemoteInputSessionToken = 0;
    quint64 endpointRemoteInputSessionCounter = 0;
    GuiRefreshProfile guiRefreshProfile = GuiRefreshProfile::Normal;
    QTimer* safetyHeartbeatTimer = nullptr;
    bool controlWorkerConfigDirty = true;
    bool cachedControlWorkerConfigValid = false;
    qint64 lastControlWorkerConfigApplyMs = 0;
    quint64 controlWorkerConfigApplyRevision = 0;
    ControlWorker::Config cachedControlWorkerConfig;
    bool safetyMonitorConfigDirty = true;
    qint64 lastSafetyMonitorConfigApplyMs = 0;
    QTimer* forcePidParameterAutoSaveTimer = nullptr;
    PvtExecutionWorker* pvtExecutionWorker = nullptr;
    QThread* positionThread = nullptr;

    MotiveLocalHandlerThread* motiveLocalHandlerThread = nullptr;
    QThread *mlhThread = nullptr;

    MonitorThread* monitorThread = nullptr;
    QThread *monitorQThread = nullptr;
    SafetyMonitor* safetyMonitor = nullptr;
    QThread* safetyMonitorThread = nullptr;
    UdpCommWorker* udpCommWorker = nullptr;
    QThread* udpThread = nullptr;

    PositionSimulationModel* positionSimulationModel = nullptr;
    PositionSimulationModel* positionSimulationHelper = nullptr;// helper鐢ㄤ簬杈呭姪鍏跺畠瀛愬嚱鏁拌绠?
    HardwareInterface hardwareInterface;
    CurveDrawer* curveDrawer = nullptr;
    QDialog* structureParamsDialog = nullptr;
    QDialog* endParamsDialog = nullptr;
    QTimer* plannedTrajectoryVisualizationTimer = nullptr;
    QTimer* motorControlInputVisualizationTimer = nullptr;
    QTimer* simulationDataVisualizationTimer = nullptr;
    QWidget* dataVizTab = nullptr;
    QCustomPlot* motorControlPlotHost = nullptr;
    QCustomPlot* endEffectorPosPlotHost = nullptr;
    QCustomPlot* endEffectorVelPlotHost = nullptr;
    QCustomPlot* endEffectorAccPlotHost = nullptr;
    QCustomPlot* actualEndEffectorPosPlotHost = nullptr;
    QCustomPlot* actualEndEffectorVelPlotHost = nullptr;
    QCustomPlot* actualEndEffectorAccPlotHost = nullptr;
    QCustomPlot* cableTensionPlotHost = nullptr;
    QCustomPlot* cableSpeedPlotHost = nullptr;
    QCustomPlot* cableLengthPlotHost = nullptr;
    QPushButton* dataVizScreenshotButton = nullptr;
    QCheckBox* dataVizFreezeCurvesCheckBox = nullptr;
    QCheckBox* actualEndEffectorVelocitySmoothCheckBox = nullptr;
    QCheckBox* actualEndEffectorAccelerationSmoothCheckBox = nullptr;
    QWidget* forcePidTuningTab = nullptr;
    QComboBox* forcePidTuningSensorComboBox = nullptr;
    QPushButton* forcePidTuningClearPlotButton = nullptr;
    QPushButton* forcePidTuningExclusiveRefreshButton = nullptr;
    QPushButton* forcePidTuningRecordButton = nullptr;
    QPushButton* mainHybridForceControlRecordButton = nullptr;
    QCustomPlot* forcePidTuningResponsePlotHost = nullptr;
    QTimer* forcePidTuningReplotTimer = nullptr;
    QTimer* forcePidTraceDrainTimer = nullptr;
    std::vector<QRadioButton*> forcePidTuningUseFCBtnVec;
    QComboBox* forcePidTuningSequenceSensorComboBox = nullptr;
    std::vector<QDoubleSpinBox*> forcePidTuningSequenceStartPoseSpinBoxes;
    std::vector<QDoubleSpinBox*> forcePidTuningSequenceEndPoseSpinBoxes;
    QDoubleSpinBox* forcePidTuningSequenceDurationSpin = nullptr;
    QDoubleSpinBox* forcePidTuningSequenceStepMsSpin = nullptr;
    QPushButton* forcePidTuningSequenceGenerateButton = nullptr;
    QPushButton* forcePidTuningSequenceUseButton = nullptr;
    QLabel* forcePidTuningSequenceStatusLabel = nullptr;
    QComboBox* forcePidTuningContinuousSensorComboBox = nullptr;
    QComboBox* forcePidTuningContinuousTypeComboBox = nullptr;
    QDoubleSpinBox* forcePidTuningContinuousStartForceSpin = nullptr;
    QDoubleSpinBox* forcePidTuningContinuousEndForceSpin = nullptr;
    QDoubleSpinBox* forcePidTuningContinuousBiasForceSpin = nullptr;
    QDoubleSpinBox* forcePidTuningContinuousAmplitudeSpin = nullptr;
    QDoubleSpinBox* forcePidTuningContinuousFrequencySpin = nullptr;
    QDoubleSpinBox* forcePidTuningContinuousDurationSpin = nullptr;
    QDoubleSpinBox* forcePidTuningContinuousStepMsSpin = nullptr;
    QPushButton* forcePidTuningContinuousLoadCurrentButton = nullptr;
    QPushButton* forcePidTuningContinuousGenerateButton = nullptr;
    QPushButton* forcePidTuningContinuousUseButton = nullptr;
    QLabel* forcePidTuningContinuousStatusLabel = nullptr;
    QWidget* motorTorqueTestTab = nullptr;
    QDoubleSpinBox* motorTorqueServoVelocityLimitSpin = nullptr;
    QComboBox* motorTorqueAxisCombo = nullptr;
    QDoubleSpinBox* motorTorqueTargetSpin = nullptr;
    QDoubleSpinBox* motorTorquePosMinSpin = nullptr;
    QDoubleSpinBox* motorTorquePosMaxSpin = nullptr;
    QDoubleSpinBox* motorTorqueVelMaxSpin = nullptr;
    QDoubleSpinBox* motorTorqueActualPosSpin = nullptr;
    QDoubleSpinBox* motorTorqueActualVelSpin = nullptr;
    QDoubleSpinBox* motorTorqueActualTorqueSpin = nullptr;
    QDoubleSpinBox* motorTorqueSampleDurationSpin = nullptr;
    QPushButton* motorTorqueServoSetupButton = nullptr;
    QPushButton* motorTorqueEnableButton = nullptr;
    QPushButton* motorTorqueStartButton = nullptr;
    QPushButton* motorTorqueStopButton = nullptr;
    QPushButton* motorTorqueSampleButton = nullptr;
    QLabel* motorTorqueServoSetupStatusLabel = nullptr;
    QLabel* motorTorqueStatusLabel = nullptr;
    QLabel* motorTorqueSampleResultLabel = nullptr;
    bool suppressMotorTorqueLimitUiSync = false;
    struct MotorTorqueSampleState {
        bool active = false;
        int axisIndex = -1;
        int sensorIndex = -1;
        qint64 startedAtMs = 0;
        double durationSec = 1.0;
        int sampleCount = 0;
        bool hasFilter = false;
        double filteredForce = 0.0;
        double filteredTorque = 0.0;
        double filteredVelocity = 0.0;
        double sumFilteredForce = 0.0;
        double sumFilteredTorque = 0.0;
        double sumFilteredVelocity = 0.0;
        double firstForce = 0.0;
        double lastForce = 0.0;
        double minForce = 0.0;
        double maxForce = 0.0;
    };
    MotorTorqueSampleState motorTorqueSampleState;
    QWidget* jogFollowTestTab = nullptr;
    QSpinBox* jogFollowAxisSpin = nullptr;
    QDoubleSpinBox* jogFollowTargetDeltaSpin = nullptr;
    QDoubleSpinBox* jogFollowMaxVelSpin = nullptr;
    QDoubleSpinBox* jogFollowLookaheadMsSpin = nullptr;
    QDoubleSpinBox* jogFollowControlPeriodMsSpin = nullptr;
    QDoubleSpinBox* jogFollowTaccdecSpin = nullptr;
    QDoubleSpinBox* jogFollowDeadbandSpin = nullptr;
    QDoubleSpinBox* jogFollowMaxDurationSpin = nullptr;
    QDoubleSpinBox* jogFollowCurrentPosSpin = nullptr;
    QDoubleSpinBox* jogFollowTargetAbsSpin = nullptr;
    QDoubleSpinBox* jogFollowErrorSpin = nullptr;
    QDoubleSpinBox* jogFollowCommandVelSpin = nullptr;
    QPushButton* jogFollowRefreshButton = nullptr;
    QPushButton* jogFollowStartButton = nullptr;
    QPushButton* jogFollowStopButton = nullptr;
    QPushButton* jogFollowClearLogButton = nullptr;
    QLabel* jogFollowStatusLabel = nullptr;
    QTextEdit* jogFollowLogTextEdit = nullptr;
    QTimer* jogFollowTestTimer = nullptr;
    QElapsedTimer jogFollowElapsedTimer;
    bool jogFollowTestRunning = false;
    int jogFollowActiveAxisIndex = -1;
    double jogFollowStartAbsPos = 0.0;
    double jogFollowTargetAbsPos = 0.0;
    double jogFollowLastCommandVel = 0.0;
    double jogFollowMaxAbsError = 0.0;
    qint64 jogFollowTickCount = 0;
    qint64 jogFollowLastTickElapsedMs = -1;
    QWidget* testMaintenanceTab = nullptr;
    QTableWidget* testMaintenanceResultTable = nullptr;
    QTextEdit* testMaintenanceInfoTextEdit = nullptr;
    QTextEdit* testMaintenanceLogTextEdit = nullptr;
    QLabel* testMaintenanceStatusLabel = nullptr;
    QLineEdit* testMaintenanceUpdatePackageEdit = nullptr;
    QPushButton* testMaintenanceRunSelfTestButton = nullptr;
    QPushButton* testMaintenanceToggleInterfaceDisconnectButton = nullptr;
    QPushButton* testMaintenanceExportSelfTestButton = nullptr;
    QPushButton* testMaintenanceExportLogsButton = nullptr;
    QPushButton* testMaintenanceOpenDataFolderButton = nullptr;
    QPushButton* testMaintenanceSelectUpdatePackageButton = nullptr;
    QPushButton* testMaintenanceValidateUpdatePackageButton = nullptr;
    bool testInterfaceDisconnectedInjected = false;
    QVector<TestEvidenceRecord> lastSoftwareConfigSelfTestRecords;
    QString lastSoftwareConfigSelfTestJsonPath;
    QString lastSoftwareConfigSelfTestMarkdownPath;
    QJsonObject defaultParameterConfigWidgetValues;

    double curTime = 0.0;// 褰撳墠鏃堕棿(s)

    qint64 lastRunModeUiRefreshMs = -1;
    QElapsedTimer programControlPvtPlanningTimer;
    bool programControlPvtPlanningTimingActive = false;
    double lastProgramControlPvtPlanningElapsedMs = -1.0;
    qint64 lastConnectionStatusUiRefreshMs = -1;
    qint64 lastSessionRecordingUiRefreshMs = -1;
    QPushButton* sessionRecordingExportButton = nullptr;
    UdpCommStats udpCommStats;
    UdpStatusPayload udpStatusPayload;
    bool udpRealtimeBridgeActive = false;
    UdpRuntimeProfile udpRuntimeProfile = UdpRuntimeProfile::Inactive;
    qint64 lastUdpStatusPayloadUpdateMs = -1;
    quint64 udpForwardKinematicsSnapshotSequence = 0;
    bool udpForwardKinematicsCacheValid = false;
    std::vector<double> udpForwardKinematicsPoseCache;
    int udpForwardKinematicsEquationCountCache = 0;
    qint64 udpForwardKinematicsTimestampMsCache = -1;
    QTimer* udpPlatformTrajectoryIdleTimer = nullptr;
    HybridPoseForceModeConfig activeHybridPoseForceConfig;
    std::vector<std::vector<double>> activeHybridExpectedForceTraj;
    int activeHybridExpectedForcePointIndex = -1;
    UdpPoseCommand latestUdpPoseCommand;
    UdpTrajectoryChunk latestUdpTrajectoryChunk;
    UdpPlatformCommand latestUdpPlatformCommand;
    qint64 lastUdpPlatformCommandTimeMs = 0;
    qint64 lastUdpPlatformCommandUiRefreshMs = 0;
    QVector<QVector<double>> udpTrajectoryPointBuffer;
    QVector<QVector<double>> udpPlatformTrajectoryPointBuffer;
    QVector<qint64> udpPlatformTrajectoryReceiveTimeMsBuffer;
    QVector<int> udpPlatformTrajectoryPacketHasSeqBuffer;
    QVector<quint32> udpPlatformTrajectoryPacketSeqBuffer;
    QVector<quint32> udpPlatformTrajectoryPacketTimestampMsBuffer;
    bool udpPlatformTrajectoryCaptureArmed = false;
    bool udpProgramControlExternalTrajectoryStarting = false;
    bool udpProgramControlExternalTrajectoryActive = false;
    bool udpReturnHomePending = false;
    bool udpReturnHomeInProgress = false;
    std::vector<double> udpReturnHomeFallbackStartPose;
    qint64 udpPlatformTrajectoryFirstTimeMs = 0;
    qint64 udpPlatformTrajectoryLastTimeMs = 0;
    bool udpPlatformTrajectoryHasFirstPacketTimestamp = false;
    quint32 udpPlatformTrajectoryFirstPacketTimestampMs = 0;
    bool udpPlatformHasLastPacketSeq = false;
    quint32 udpPlatformLastPacketSeq = 0;
    quint32 udpPlatformLastPacketTimestampMs = 0;
    int udpPlatformPacketLossEstimate = 0;
    int udpPlatformPacketOrderAnomalyCount = 0;
    int udpPlatformPacketTimestampAnomalyCount = 0;
    quint64 lastUdpPoseSeq = 0;
    qint64 lastUdpPoseTimestampMs = 0;
    quint64 lastUdpTrajectorySeq = 0;
    qint64 lastUdpTrajectoryTimestampMs = 0;
    bool lastUdpTrajectoryChunkWasFinal = false;
    QString udpLastPacketSummaryText;
    QString udpLastPacketActionText;
    QVector<RuntimeDiagnosticsSample> runtimeDiagnosticsHistory;
    QString lastRuntimeDiagnosticsReportPath;
    qint64 lastRuntimeDiagnosticsAutoWriteMs = 0;
    bool runtimeDiagnosticsReportWriting = false;
    int lastAppliedForceSensorTraceSamplePeriodUs = -1;
    int faultInjectionCachedAxisCount = -1;
    int faultInjectionCachedSensorCount = -1;
    QString primaryOperationFeedbackText;
    QString primaryOperationFeedbackStyle;
    qint64 primaryOperationFeedbackExpireMs = 0;
    HardwareInterface::MotorTraceRecoveryState motorTraceRecoveryState;
    bool motorTraceRecoveryStateValid = false;
    int motorTraceRecoveryStartupRefreshToken = 0;
    HardwareInterface::MotorTraceRecoveryState linearModuleTraceRecoveryState;
    bool linearModuleTraceRecoveryStateValid = false;
    QGroupBox* connectionStatusGroupBox = nullptr;
    QLabel* connectionControllerStatusLabel = nullptr;
    std::vector<QLabel*> connectionAxisStatusLabels;
    std::vector<QLabel*> connectionSensorStatusLabels;
    QGroupBox* calibrationStatusGroupBox = nullptr;
    QLabel* calibrationStatusLabel = nullptr;
    QLabel* calibrationActionLabel = nullptr;
    QLabel* calibrationStepLabel = nullptr;
    QLabel* calibrationResultLabel = nullptr;
    QPushButton* calibrationStartButton = nullptr;
    QPushButton* calibrationStopButton = nullptr;
    QPushButton* calibrationConfirmButton = nullptr;
    QPushButton* calibrationKnownRuntimePoseButton = nullptr;
    QPushButton* calibrationLiteWinchReferenceButton = nullptr;
    QPushButton* calibrationSaveButton = nullptr;
    QPushButton* calibrationLoadButton = nullptr;
    QPushButton* calibrationRestoreButton = nullptr;
    QString lastCalibrationRecordPath;
    QDateTime lastCalibrationRecordTime;
    QString lastCalibrationRecordSummary;
    CalibrationWorkflowStage calibrationWorkflowStage = CalibrationWorkflowStage::Idle;
    bool liteWinchReferenceCapturePending = false;
    QElapsedTimer visualizationElapsedTimer;
    bool visualizationTimerStarted = false;
    bool forcePidTuningExclusiveRefresh = false;
    bool forcePidTuningExpectedForceSequenceActive = false;
    int forcePidTuningExpectedForceSequenceSensorIndex = -1;
    std::vector<std::vector<double>> forcePidTuningExpectedForceSequenceTraj;
    std::vector<double> forcePidTuningExpectedForceSequenceTimeStamp;
    bool forcePidTuningContinuousForceSequenceActive = false;
    bool forcePidTuningContinuousForceSequenceClockStarted = false;
    int forcePidTuningContinuousForceSequenceSensorIndex = -1;
    int forcePidTuningContinuousForceSequenceType = 0;
    std::vector<std::vector<double>> forcePidTuningContinuousForceSequenceTraj;
    std::vector<double> forcePidTuningContinuousForceSequenceTimeStamp;
    bool manualForceControlPretensionRampActive = false;
    qint64 manualForceControlPretensionRampEndMs = 0;
    std::vector<int> manualForceControlPretensionRampSensorIndex;
    std::vector<double> manualForceControlPretensionRampFinalExpectedForce;
    bool suppressForceControlSelectionSideEffects = false;
    qint64 forcePidTuningPlotStartWallClockUs = 0;
    qint64 forcePidTuningPlotSampleCount = 0;
    qint64 forcePidTuningLastPlotAppendWallClockUs = 0;
    double forcePidTuningLatestPlotTimeSec = 0.0;
    bool forcePidTuningReplotPending = false;
    struct ForcePidTuningRecordSample {
        qint64 wallClockUs = 0;
        double elapsedSec = 0.0;
        double controlDtSec = 0.0;
        std::vector<double> forceSensorValue;
        std::vector<double> expectedForce;
        std::vector<double> motorCommand;
        std::vector<double> motorTorqueNm;
        std::vector<double> pidOutput;
        std::vector<double> pidError;
        std::vector<double> pidPTerm;
        std::vector<double> pidITerm;
        std::vector<double> pidDTerm;
        std::vector<double> pidIntegral;
        std::vector<double> pidMeasuredDerivativeRaw;
        std::vector<double> pidMeasuredDerivativeFiltered;
        std::vector<double> pidMeasuredDerivativeControl;
        std::vector<double> pidExpectedDerivativeRaw;
        std::vector<double> pidExpectedDerivativeFiltered;
        std::vector<double> pidFeedForwardRaw;
        std::vector<double> pidFeedForwardTerm;
        std::vector<double> pidFeedForwardFrictionTerm;
        std::vector<int> pidFeedForwardSelectedDynamicProfile;
        std::vector<double> pidStaticFrictionDirection;
        std::vector<double> pidStaticFrictionSpeedScale;
        std::vector<double> pidStaticFrictionRaw;
        std::vector<double> pidStaticFrictionAfterFade;
        std::vector<double> pidStaticFrictionAfterSmooth;
        std::vector<double> pidFeedForwardVelocityTerm;
        std::vector<double> pidFeedForwardAccelerationTerm;
        std::vector<double> pidExpectedRopeVelocityRadPerSec;
        std::vector<double> pidExpectedRopeAccelerationRadPerSec2;
        std::vector<double> pidExpectedRateFeedForwardTerm;
        std::vector<double> pidExpectedRateFeedForwardScale;
        std::vector<double> pidForceRateError;
        std::vector<double> pidForceRateErrorDampingTerm;
        std::vector<double> pidPlatformCaptureTerm;
        std::vector<double> pidPlatformCaptureTargetTerm;
        std::vector<int> pidPlatformCaptureState;
        std::vector<double> pidFuzzyFeedForwardTargetScale;
        std::vector<double> pidFuzzyFeedForwardScale;
        std::vector<double> pidFuzzyFeedForwardRecoveryRate;
        std::vector<double> pidFuzzyKpScale;
        std::vector<double> pidFuzzyKiScale;
        std::vector<double> pidFuzzyVelocityDampingScale;
        std::vector<double> pidFuzzyPositivePLimit;
        std::vector<double> pidFuzzyNegativePLimit;
        std::vector<int> pidFuzzyFeedForwardRecoveryLimited;
        std::vector<int> pidFuzzyPLimitApplied;
        std::vector<int> pidFuzzyState;
        std::vector<int> pidIntegralReleaseApplied;
        std::vector<int> pidAntiWindup;
        std::vector<int> pidOutputLimited;
        std::vector<int> torqueSaturated;
        std::vector<int> torqueSlewLimited;
        std::vector<double> motorVel;
        std::vector<int> pid0525HybridState;
        std::vector<int> pid0525HybridBiasValid;
        std::vector<double> pid0525HybridHoldBiasNm;
        std::vector<double> pid0525HybridCaptureForceN;
        std::vector<double> pid0525HybridFeedForwardTermNm;
        std::vector<double> pid0525HybridFeedbackTermNm;
        std::vector<double> pid0525HybridBlend;
        std::vector<int> forceControlAxisIndex;
        std::vector<int> forceControlSensorIndex;
    };
    // 程序控制期间复用控制快照、PVT 下发表和可视化正运动学结果形成的低频混合控制记录。
    struct HybridControlRuntimeRecordSample {
        qint64 wallClockUs = 0;
        double elapsedSec = 0.0;
        double intervalSec = 0.0;
        quint64 controlSequence = 0;
        bool systemRunning = false;
        bool pvtActive = false;
        bool pvtPaused = false;
        bool pvtProgressValid = false;
        double pvtTrajectoryTimeSec = -1.0;
        int pvtPointIndex = -1;
        bool safetyFaultLatched = false;
        int safetyStopLevel = 0;
        int safetyFaultCode = 0;
        bool forwardKinematicsValid = false;
        int forwardKinematicsEquationCount = 0;
        std::vector<int> axisControlMode;
        std::vector<double> positionCommand;
        std::vector<double> motorAbsPos;
        std::vector<double> motorRelRawPos;
        std::vector<double> motorVel;
        std::vector<double> motorTorqueNm;
        std::vector<double> pose;
        std::vector<double> velocityRaw;
        std::vector<double> accelerationRaw;
        std::vector<double> velocityFiltered;
        std::vector<double> accelerationFiltered;
    };
    bool forcePidTuningRecordingActive = false;
    qint64 forcePidTuningRecordStartWallClockUs = 0;
    qint64 forcePidTuningRecordEndedWallClockUs = 0;
    QString forcePidTuningRecordStartedAtText;
    QString forcePidTuningRecordLastExportPath;
    std::vector<int> forcePidTuningRecordAxisIndex;
    std::vector<int> forcePidTuningRecordSensorIndex;
    std::vector<ForcePidTuningRecordSample> forcePidTuningRecordSamples;
    size_t forcePidTuningRecordRingStartIndex = 0;
    ControlWorker::Config forcePidTuningRecordConfigSnapshot;
    bool forcePidTuningRecordConfigSnapshotValid = false;
    bool hybridForceControlRecordingActive = false;
    qint64 hybridForceControlRecordStartWallClockUs = 0;
    QString hybridForceControlRecordStartedAtText;
    QString hybridForceControlRecordLastExportPath;
    std::vector<int> hybridForceControlRecordAxisIndex;
    std::vector<int> hybridForceControlRecordForceAxisIndex;
    std::vector<int> hybridForceControlRecordPositionAxisIndex;
    std::vector<int> hybridForceControlRecordSensorIndex;
    std::vector<ForcePidTuningRecordSample> hybridForceControlRecordSamples;
    size_t hybridForceControlRecordRingStartIndex = 0;
    ControlWorker::Config hybridForceControlRecordConfigSnapshot;
    bool hybridForceControlRecordConfigSnapshotValid = false;
    bool hybridControlRuntimeRecordingActive = false;
    qint64 hybridControlRuntimeRecordStartWallClockUs = 0;
    qint64 hybridControlRuntimeRecordLastSampleWallClockUs = 0;
    QString hybridControlRuntimeRecordStartedAtText;
    QString hybridControlRuntimeRecordTaskId;
    QString hybridControlRuntimeRecordFinishReason;
    QString hybridControlRuntimeRecordLastExportPath;
    std::vector<int> hybridControlRuntimeRecordForceAxisIndex;
    std::vector<int> hybridControlRuntimeRecordPositionAxisIndex;
    std::vector<double> hybridControlRuntimeRecordPvtTimeStamp;
    std::vector<std::vector<double>> hybridControlRuntimeRecordPositionCommand;
    std::vector<HybridControlRuntimeRecordSample> hybridControlRuntimeRecordSamples;
    std::vector<double> hybridControlRuntimeRecordLastPose;
    std::vector<double> hybridControlRuntimeRecordLastVelocity;
    qint64 hybridControlRuntimeRecordLastValidPoseWallClockUs = 0;
    std::vector<double> hybridControlRuntimeRecordFilteredVelocity;
    std::vector<double> hybridControlRuntimeRecordFilteredAcceleration;
    std::deque<ControlWorker::ForcePidTraceSample> pendingForcePidTraceSamples;
    double lastVisualizationPoseTime = -1.0;
    std::vector<double> lastVisualizationPose;
    std::vector<double> lastVisualizationVelocity;
    ForwardKinematicsSolver actualEndEffectorKinematicsSolver;
    double lastActualEndEffectorPoseTime = -1.0;
    std::vector<double> lastActualEndEffectorPose;
    std::vector<double> lastActualEndEffectorVelocity;
    std::vector<double> smoothedActualEndEffectorVelocity;
    std::vector<double> smoothedActualEndEffectorAcceleration;
    bool plannedTrajectoryVisualizationStarted = false;
    double plannedTrajectoryVisualizationTimeSec = 0.0;
    int plannedTrajectoryVisualizationLastPointIndex = -1;
    std::vector<int> motorControlInputVisualizationMotorIndex;
    std::vector<double> motorControlInputVisualizationTimeStamp;
    std::vector<std::vector<double>> motorControlInputVisualizationPositionUnit;
    bool motorControlInputVisualizationStarted = false;
    double motorControlInputVisualizationTimeSec = 0.0;
    int motorControlInputVisualizationPointIndex = -1;
    QVector<double> lastCableLengthVisualizationValues;
    double lastCableKinematicVisualizationTimeSec = -1.0;
    bool simulationDataVisualizationActive = false;
    bool simulationDataVisualizationHoldingFrozenResult = false;
    bool simulationDataVisualizationStarted = false;
    double simulationDataVisualizationTimeSec = 0.0;
    int simulationDataVisualizationPointIndex = -1;
    quint64 simulationDataVisualizationPlaybackToken = 0;
    std::vector<std::vector<std::vector<std::vector<double>>>> simulationDataVisualizationTrajectory;
    std::vector<double> simulationDataVisualizationTimeStamp;
    std::vector<std::vector<double>> simulationDataVisualizationCableLengthTraj;
    std::vector<std::vector<double>> simulationDataVisualizationCableForceTraj;
    QVector<double> lastSimulationCableLengthVisualizationValues;
    double lastSimulationCableKinematicVisualizationTimeSec = -1.0;
    std::vector<std::vector<double>> lastForwardKinematicsPose;
    qint64 lastForwardKinematicsPoseTimestampMs = -1;
    bool lastForwardKinematicsPoseLoadedFromSnapshot = false;
    bool currentRuntimeMotorHomeReferenceLoaded = false;
    // 当前连接实际使用的命令零位；Lite 中固定为完整启动时的上电位置。
    std::vector<double> currentRuntimeMotorHomePos;
    std::vector<double> currentRuntimeMotorHomeEncoderPos;
    // 外部已知位姿确认瞬间的电机位置，与上面的固定命令零位分开保存。
    std::vector<double> currentRuntimeReferenceMotorPos;
    std::vector<double> currentRuntimeReferenceMotorEncoderPos;
    std::vector<std::vector<double>> currentRuntimeMotorHomePlatformPose;
    qint64 lastRuntimeResultAutoSaveMs = -1;
    std::vector<std::vector<std::vector<std::vector<double>>>> plannedPoseTrajectoryDisplayTraj;
    std::vector<double> plannedPoseTrajectoryDisplayLambda;
    std::vector<std::vector<std::vector<std::vector<double>>>> activePoseTrajectoryDisplayTraj;
    std::vector<double> activePoseTrajectoryDisplayLambda;
    std::vector<double> activePoseTrajectoryDisplayTimeStamp;
    int activePoseTrajectoryDisplayPointIndex = -1;
    bool activePoseTrajectoryDisplayRunning = false;
    std::vector<double> plannedPoseTrajectoryRecordTimeStamp;
    std::vector<std::vector<double>> plannedPoseTrajectoryRecordPose;
    std::vector<double> plannedPoseTrajectoryRecordLambda;
    std::vector<int> plannedPoseTrajectoryRecordMotorIndex;
    std::vector<std::vector<double>> plannedPoseTrajectoryRecordMotorExpectedPos;
    qint64 plannedPoseTrajectoryRecordTimestampMs = -1;
    int plannedPoseTrajectoryRecoveredPointIndex = -1;
    double plannedPoseTrajectoryRecoveredEncoderRmsError = 0.0;
    double plannedPoseTrajectoryRecoveredEncoderMaxError = 0.0;
    struct PosePvtTraceCommandPointMatch {
        bool valid = false;
        qint64 traceWallClockUs = 0;
        qint64 traceMonotonicUs = 0;
        double traceElapsedSec = 0.0;
        double traceTrajectoryTimeSec = 0.0;
        double traceTimeErrorSec = 0.0;
        qint64 commandRawPulse = 0;
        qint64 feedbackRawPulse = 0;
        bool feedbackRawPulseValid = false;
        double commandRelativeUnit = 0.0;
        double feedbackRelativeUnit = 0.0;
        int samplesConsumed = 0;
    };
    struct PosePvtTraceCommandCompareState {
        bool active = false;
        bool written = false;
        bool truncated = false;
        QString encoderReferenceSource;
        QString pvtEncoderConversionMode;
        bool pvtPositionUsesEncoderZeroReference = false;
        QString trajectoryPoseSource;
        std::vector<double> trajectoryStartPose;
        std::vector<double> trajectoryEndPose;
        QString lastFilePath;
        qint64 startedAtWallClockUs = 0;
        qint64 finishedAtWallClockUs = 0;
        std::vector<int> motorIndex;
        std::vector<double> axisEquiv;
        std::vector<double> encoderReferenceValue;
        std::vector<qint64> encoderReferenceRawPulse;
        std::vector<double> relativeTimeStamp;
        std::vector<std::vector<double>> pvtPositionUnit;
        std::vector<std::vector<qint64>> pvtCommandRawPulse;
        std::vector<std::vector<PosePvtTraceCommandPointMatch>> pointMatches;
        std::vector<int> traceSampleCountByAxis;
    };
    struct PosePvtCommandTable {
        std::vector<int> cableMotorIndex;
        std::vector<int> controlledMotorIndex;
        std::vector<int> controlledAxisColumn;
        std::vector<std::vector<double>> allMotorPosTraj;
        std::vector<std::vector<double>> allMotorVelTraj;
        std::vector<std::vector<double>> controlledMotorPosTraj;
        std::vector<std::vector<double>> controlledMotorVelTraj;
        std::vector<double> timeStamp;
        std::vector<double> plannedFullTrajStartMotorTheta;
        std::vector<double> plannedFullTrajEndMotorTheta;
    };
    struct ControlBoxPvtProgressSample {
        bool valid = false;
        int attemptCount = 0;
        int validSampleCount = 0;
        int firstIndex = -1;
        int lastIndex = -1;
        int bestIndex = -1;
        double firstTrajectoryTime = 0.0;
        double lastTrajectoryTime = 0.0;
        double bestTrajectoryTime = 0.0;
        qint64 elapsedMs = 0;
    };
    struct MotorCommandLimitSnapshot {
        int axisIndex = -1;
        double minPos = 0.0;
        double maxPos = 0.0;
        double maxVel = 0.0;
        double safetyRelativeOffset = 0.0;
    };
    struct ControlBoxSpeedZeroPvtReturnState {
        bool valid = false;
        bool returnPvtActive = false;
        bool triggerProgressValid = false;
        bool pauseBeforeProgressValid = false;
        bool stoppedProgressValid = false;
        bool stableProgressSampleValid = false;
        bool autoReturnBlocked = false;
        int triggerPointIndex = -1;
        int pauseBeforePointIndex = -1;
        int stoppedPointIndex = -1;
        int stableProgressSampleAttempts = 0;
        int stableProgressSampleCount = 0;
        int stableProgressFirstIndex = -1;
        int stableProgressLastIndex = -1;
        int stableProgressBestIndex = -1;
        int returnMaxDeltaAxis = -1;
        double triggerTrajectoryTime = 0.0;
        double pauseBeforeTrajectoryTime = 0.0;
        double stoppedTrajectoryTime = 0.0;
        double stableProgressFirstTrajectoryTime = 0.0;
        double stableProgressLastTrajectoryTime = 0.0;
        double stableProgressBestTrajectoryTime = 0.0;
        double returnMaxDeltaUnit = 0.0;
        double returnMaxDeltaStartRelative = 0.0;
        double returnMaxDeltaTargetRelative = 0.0;
        qint64 stableProgressSampleElapsedMs = 0;
        std::vector<int> motorIndex;
        std::vector<qint64> targetTraceCommandRaw;
        std::vector<double> targetPvtPositionUnit;
        std::vector<double> triggerPose;
        std::vector<double> originalStartPose;
        std::vector<double> originalEndPose;
        std::vector<double> originalTrajectoryLambda;
        std::vector<double> circularCenter;
        double circularRadius = 0.0;
        int circularDirection = 1;
        std::vector<double> eightShapeNormal;
        double eightShapeRadius = 0.0;
        double eightShapeRange = 0.0;
        QString trajectoryMode;
        double originalTrajectoryDurationSec = 0.0;
        double originalTrajectoryStepSec = 0.0;
        bool triggerTrajectoryLambdaValid = false;
        double triggerTrajectoryLambda = 0.0;
        bool speedZeroResumeRequested = false;
        bool speedZeroResumeStartInProgress = false;
        bool motionControlStopReturnToStartPending = false;
        bool motionControlStopHomePvtActive = false;
        bool motionControlStopHomeStartInProgress = false;
        bool motionControlStopHomeEarlyDoneReported = false;
        bool motionControlStopHomePositionMismatchReported = false;
        double motionControlStopHomeDurationSec = 0.0;
        qint64 motionControlStopHomeStartMs = 0;
        std::vector<double> motionControlStopHomeTargetAbsPosition;
    };
    PosePvtTraceCommandCompareState posePvtTraceCommandCompare;
    ControlBoxSpeedZeroPvtReturnState controlBoxSpeedZeroPvtReturnState;
    std::vector<double> lastForceDistributionCableForce;
    std::vector<double> lastForceDistributionExpectedForce;
    std::vector<std::vector<double>> lastForceDistributionPlatformPose;
    qint64 lastForceDistributionTimestampMs = -1;
    std::vector<std::vector<double>> restoredRuntimePretensionPlatformPose;

    // 判断当前是否允许继续向信息区发送输出。
    bool outputInfoSendTrigger();
    bool useCam = true;
    // 锁定或解锁动捕/相机输入。
    void camLock(bool lockCam);
    // 重置动捕拟合确认状态。
    void resetMotiveFit();
    // 判断当前是否有有效动捕位姿。
    bool hasValidMotivePose() const;
    // 判断是否有近期可用的估计末端位姿。
    bool hasValidEstimatedEndPose(int maxAgeMs = 500) const;
    // 判断最近 maxAgeMs 内是否收到动捕位姿。
    bool hasRecentMotivePose(int maxAgeMs = 500) const;
    // 决定是否用正运动学估计替代动捕位姿。
    bool shouldUseForwardKinematicsFallback(int maxAgeMs = 500) const;
    // 从正运动学缓存读取当前末端位姿。
    bool currentForwardKinematicsEndPose(std::vector<double>& pose, int maxAgeMs = 1000) const;
    // 工作空间监控专用：只使用活动轨迹点、遥控开环期望位姿或规划末点，
    // 绝不使用动捕或正运动学估计位姿。
    bool currentWorkspaceSafetyPose(std::vector<double>& pose) const;
    // 只基于指定电机快照计算当前末端位姿，用于状态回传等同帧数据。
    bool computeForwardKinematicsEndPoseFromMotorSnapshot(const std::vector<double>& motorAbsPos,
                                                          std::vector<double>& pose,
                                                          int* equationCount = nullptr);
    // 根据当前模板给正运动学求解器配置位姿搜索边界。
    void applyForwardKinematicsBoundsForCurrentTemplate(ForwardKinematicsSolver::Request& request) const;
    // 优先读取动捕位姿，必要时回退到正运动学估计。
    bool currentEstimatedEndPose(std::vector<double>& pose, int maxAgeMs = 500) const;
    // 缓存正运动学计算出的平台位姿。
    void cacheForwardKinematicsPose(const std::vector<std::vector<double>>& platformPose);
    // 清除轨迹显示状态，可选择同时清除规划轨迹。
    void clearPoseTrajectoryDisplayState(bool clearPlannedTrajectory = false);
    // 返回当前活动轨迹显示点数。
    int activePoseTrajectoryDisplayPointCount() const;
    // 返回轨迹显示定时器刷新间隔。
    int poseTrajectoryDisplayTimerIntervalMs() const;
    // 按点索引缓存一帧轨迹显示数据。
    bool cachePoseTrajectoryDisplayPoint(int pointIndex);
    // 按轨迹时间缓存一帧轨迹显示数据。
    bool cachePoseTrajectoryDisplayPointByTime(double trajectoryTimeSec);
    // 通过当前电机位置匹配并缓存最近的轨迹显示点。
    bool cachePoseTrajectoryDisplayPointByMotorPosition(
            const std::vector<double>& motorAbsPos,
            double* rmsError = nullptr,
            double* maxAbsError = nullptr);
    // 启动活动位姿轨迹显示，供执行中或恢复后可视化。
    void startPoseTrajectoryDisplay(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& trajectory,
            const std::vector<double>& timeStamp);
    // 根据 PVT 进度刷新轨迹显示点。
    bool refreshPoseTrajectoryDisplayFromPvtProgress();
    // 清空规划轨迹运行记录。
    void clearPlannedPoseTrajectoryRuntimeRecord();
    // 保存规划轨迹、电机期望位置和时间戳，供恢复/诊断使用。
    bool storePlannedPoseTrajectoryRuntimeRecord(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& trajectory,
            const std::vector<double>& timeStamp,
            const std::vector<int>& motorIndex,
            const std::vector<std::vector<double>>& motorExpectedPos,
            const std::vector<double>& trajectoryLambda = std::vector<double>());
    // 构建规划轨迹快照 JSON。
    QJsonObject buildPlannedPoseTrajectorySnapshot() const;
    // 从 JSON 加载规划轨迹快照。
    bool loadPlannedPoseTrajectorySnapshot(const QJsonObject& root, bool announce);
    // 根据电机位置在规划轨迹中查找最接近点。
    bool findPlannedPoseTrajectoryPointByMotorPosition(
            const std::vector<double>& motorAbsPos,
            int& pointIndex,
            double* rmsError = nullptr,
            double* maxAbsError = nullptr) const;
    // 确保规划轨迹运行记录已从内存或文件加载。
    bool ensurePlannedPoseTrajectoryRuntimeRecordLoaded(bool announce = false);
    // 刷新正运动学位姿显示。
    void refreshForwardKinematicsPoseDisplay();
    // 判断是否已有正运动学绳长参考。
    bool hasForwardKinematicsCableLengthReference() const;
    // 返回配置的绳索回零位姿。
    std::vector<double> configuredCableHomePose() const;
    // 返回配置的绳索回零平台位姿矩阵。
    std::vector<std::vector<double>> configuredCableHomePlatformPose() const;
    // 返回正运动学使用的绳长参考位姿。
    std::vector<std::vector<double>> forwardKinematicsCableLengthReferencePose() const;
    // 返回正运动学求解的初值位姿。
    std::vector<std::vector<double>> forwardKinematicsInitialGuessPlatformPose() const;
    // 判断当前电机位置是否接近绳索回零位置。
    bool isCurrentMotorSnapshotAtCableHome(double toleranceMm = 0.5) const;
    bool motiveFitConfirmed = false;

    // 将当前 UI 和运行态参数同步到 ControlWorker。
    void updateControlWorkerConfig();
    // 标记 ControlWorker/SafetyMonitor 配置需要重新下发。
    void markControlWorkerConfigDirty();
    // 按脏标记和降频策略同步 ControlWorker 配置。
    bool syncControlWorkerConfig(bool forceApply = false,
                                 ControlWorker::Config* configSnapshot = nullptr);
    // 把 ControlWorker 最新快照刷新到 UI、绘图和安全监控。
    void applyControlSnapshot();
    // 应用硬件电机位置快照并刷新显示缓存。
    void applyMotorPositionSnapshot(const std::vector<double>& absPos,
                                    const std::vector<double>& relRawPos = {});
    // 根据当前电机位置计算各绳索位移。
    bool buildCurrentCableDisplacements(std::vector<std::vector<double>>& cableDisplacement) const;
    // 清理正运动学绳长参考缓存。
    void clearForwardKinematicsCableLengthCache();
    // 按建模轴顺序生成 cableHomePos，输入可为原始反馈值或已换算后的绳长单位值。
    std::vector<double> buildForwardKinematicsCableHomePosFromAxisValues(
            const std::vector<double>& axisValues,
            bool valuesAlreadyCableUnits) const;
    // 使用参考位姿和同坐标系下的 cableHomePos 重建正运动学绳长缓存。
    bool rebuildForwardKinematicsCableLengthCache(
            const std::vector<std::vector<double>>& referencePose,
            const std::vector<double>& referenceCableHomePos);
    // 根据当前电机位置计算各绳索长度。
    bool buildCurrentCableLengths(std::vector<std::vector<double>>& cableLen) const;
    // 构建当前绳长快照，供诊断和正运动学使用。
    bool buildCurrentCableLengthSnapshot(std::vector<std::vector<double>>& cableLen) const;
    // 按轴顺序展开多末端绳长矩阵。
    QVector<double> flattenCableLengthsByAxisOrder(const std::vector<std::vector<double>>& cableLen) const;
    // 将绳长数组格式化为 UI/日志文本。
    QString formatCableLengthVector(const QVector<double>& cableLen) const;
    // 记录轨迹终点绳长诊断，便于定位末端/电机误差。
    void logTrajectoryEndCableLengthDiagnostics();
    // 确保位姿运动学辅助模型已创建并带有最新参数。
    bool ensurePoseKinematicsHelpersReady(bool forceRebuild = false);
    // 根据最新电机位置刷新正运动学位姿缓存。
    void updateForwardKinematicsPoseFromMotorSnapshot();
    // 立即从当前电机位置计算正运动学位姿。
    bool refreshForwardKinematicsPoseFromCurrentMotorPosition();
    // 根据当前 UI 和运行态生成 ControlWorker 配置。
    ControlWorker::Config buildControlWorkerConfig() const;
    // 从 UI 中读取期望绳力向量。
    std::vector<double> buildExpectedForceFromUi() const;
    // 返回当前请求参与力控的传感器索引。
    std::vector<int> forceControlSelectedSensorIndicesForExpectedForceCheck() const;
    // 检查选中力控传感器是否都有有效非零期望力。
    QString forceControlExpectedForceMissingMessage(const QString& actionName) const;
    bool ensureForceControlExpectedForceReady(const QString& actionName);
    bool enableForceControlThreadIfExpectedForceReady(const QString& actionName);
    // 应用计算得到的期望力，可写入缓存、下发给力控并刷新配置。
    bool applyCalculatedExpectedForce(const std::vector<double>& expectedForce,
                                      bool storeLastDistribution,
                                      bool sendExternalToControlWorker,
                                      bool refreshControlConfig = true);
    // 在普通手动力控启动时生成当前实际力到界面期望力的张紧斜坡。
    bool startManualForceControlPretensionRampForCurrentSelection();
    // 判断位姿页是否启用全绳预紧/拖动模式；该模式只切换后续手动力控 PID。
    bool isAllCableForceDragModeRequested() const;
    // 读取指定传感器当前力，优先使用worker最近Trace样本。
    bool currentForceForSensor(int sensorIndex, double* force, QString* source = nullptr) const;
    // 取消普通手动力控张紧斜坡，可选择同时清除 ControlWorker 外部期望力。
    void cancelManualForceControlPretensionRamp(bool clearExternalExpected);
    // 张紧斜坡结束后恢复为界面期望力；返回 true 表示本轮需要清除外部期望力轨迹。
    bool finalizeManualForceControlPretensionRampIfFinished();
    // 创建数据可视化 Tab 和曲线宿主控件。
    void setupDataVisualizationTab();
    // 按冻结/独占刷新状态刷新数据可视化曲线重绘开关。
    void updateDataVisualizationRefreshEnabled();
    // 创建力控 PID 调参 Tab。
    void setupForcePidTuningTab();
    // 创建单轴力矩测试 Tab。
    void setupMotorTorqueTestTab();
    // 启动单轴力矩测试 worker 线程。
    void startMotorTorqueTestThread();
    // 停止单轴力矩测试 worker 线程。
    void stopMotorTorqueTestThread();
    // 刷新力矩测试轴选择列表。
    void refreshMotorTorqueAxisOptions();
    // 返回当前力矩测试选中的轴。
    int selectedMotorTorqueAxisIndex() const;
    // 从 UI 读取力矩模式速度限制。
    double motorTorqueServoVelocityLimitRpm() const;
    // 将力矩模式速度限制写入硬件。
    bool applyMotorTorqueServoVelocityLimitFromUi();
    // 按轴配置同步力矩测试限位 UI。
    void syncMotorTorqueLimitUiFromAxis(int axisIndex);
    // 将 UI 中的力矩测试限位写回轴配置。
    void applyMotorTorqueLimitUiToAxis(int axisIndex);
    // 更新 MotorTorqueTestWorker 配置。
    void updateMotorTorqueWorkerConfig();
    // 进入单轴力矩调试模式。
    void startMotorTorqueDebug();
    // 退出单轴力矩调试模式。
    void stopMotorTorqueDebug();
    // 开始转矩模式保持力矩采样。
    void startMotorTorqueFeedbackSample();
    // 尝试读取当前力矩调试轴绑定的力传感器实际力。
    bool currentMotorTorqueSampleForce(int axisIndex, int* sensorIndex, double* force) const;
    // 在转矩调试状态刷新时累计保持力矩采样。
    void updateMotorTorqueFeedbackSample(int axisIndex,
                                         double actualTorque,
                                         double actualVelocity);
    // 结束并展示转矩模式保持力矩采样结果。
    void finishMotorTorqueFeedbackSample(const QString& errorMessage = QString());
    // 处理力矩测试 worker 回传的状态并刷新 UI。
    void handleMotorTorqueStatus(int axisIndex,
                                 double relativePosition,
                                 double actualTorque,
                                 double actualVelocity,
                                 bool active);
    // 重新创建所有曲线对象，通常在轴数/通道数变化后调用。
    void reinitializeDataVisualizationPlots();
    // 清空可视化曲线数据。
    void clearVisualizationData();
    // 保存当前主窗口截图到安装目录 data/outputmsg/screenshot。
    void saveMainWindowScreenshot();
    // 返回可视化运行时间，负责启动内部计时器。
    double visualizationTimeSeconds();
    // 根据控制快照更新电机/绳力/速度等曲线。
    void updateVisualizationFromControlSnapshot(const ControlWorker::Snapshot& snapshot);
    // 刷新 PID 调参传感器选择列表。
    void refreshForcePidTuningSensorOptions();
    // 返回当前 PID 调参选中的传感器索引。
    int selectedForcePidTuningSensorIndex() const;
    // 创建 PID 调参页中的轨迹期望力序列控件。
    void setupForcePidTuningExpectedForceSequenceUi();
    // 刷新 PID 期望力序列可选绳索/传感器。
    void refreshForcePidTuningSequenceSensorOptions();
    // 返回当前期望力序列目标传感器索引。
    int selectedForcePidTuningSequenceSensorIndex() const;
    // 生成五次多项式直线轨迹对应的单绳期望力序列。
    void generateForcePidTuningExpectedForceSequence();
    // 清除 PID 调参真实轨迹期望力序列状态。
    void clearForcePidTuningExpectedForceSequenceState(bool clearExternalExpected);
    // 启停 PID 调参期望力序列播放。
    void setForcePidTuningExpectedForceSequenceActive(bool enabled);
    // 刷新 PID 调参期望力序列状态显示。
    void updateForcePidTuningExpectedForceSequenceStatus();
    // 创建 PID 调参页中的连续期望力验证控件。
    void setupForcePidTuningContinuousForceUi();
    // 刷新连续期望力验证可选绳索/传感器。
    void refreshForcePidTuningContinuousSensorOptions();
    // 返回当前连续期望力验证目标传感器索引。
    int selectedForcePidTuningContinuousSensorIndex() const;
    // 按 UI 生成斜坡/低频正弦期望力序列。
    void generateForcePidTuningContinuousForceSequence();
    // 清除斜坡/低频正弦期望力序列状态。
    void clearForcePidTuningContinuousForceSequenceState(bool clearExternalExpected);
    // 布防/停止 PID 调参连续期望力序列，布防后等待目标力控启动再播放。
    void setForcePidTuningContinuousForceSequenceActive(bool enabled);
    // 判断连续期望力序列目标绳索是否已经处于可启动力控状态。
    bool isForcePidTuningContinuousForceSequenceTargetReady() const;
    // 若连续期望力序列已布防且目标绳索力控已启用，则启动序列时间轴。
    bool startForcePidTuningContinuousForceSequenceClockIfReady();
    // 刷新 PID 调参连续期望力序列状态显示。
    void updateForcePidTuningContinuousForceSequenceStatus();
    // 查找指定传感器对应的建模绳索电机轴。
    int modeledCableAxisForForceSensor(int sensorIndex, QString* errorMessage = nullptr) const;
    // 初始化 PID 调参响应曲线。
    void setupForcePidTuningResponsePlot();
    // 清空 PID 调参响应曲线。
    void clearForcePidTuningResponsePlot();
    // 追加一帧 PID 调参响应数据。
    void updateForcePidTuningResponsePlot(double timeSec,
                                          const std::vector<double>& forceSensorValue,
                                          const std::vector<double>& expectedForce);
    // 执行 PID 调参曲线重绘。
    void refreshForcePidTuningResponsePlot();
    // 根据记录/独占刷新状态启停重绘定时器。
    void updateForcePidTuningReplotTimerState();
    // 设置是否独占刷新 PID 调参曲线。
    void setForcePidTuningExclusiveRefresh(bool enabled);
    // 刷新独占刷新按钮文本。
    void updateForcePidTuningExclusiveRefreshButtonText();
    // 开始记录 PID 调参采样数据。
    void startForcePidTuningRecord();
    // 停止记录 PID 调参采样数据。
    void stopForcePidTuningRecord();
    // 刷新 PID 调参记录按钮文本。
    void updateForcePidTuningRecordButtonText();
    // 导出 PID 调参记录文件。
    bool exportForcePidTuningRecord();
    // 将环形记录缓存按时间顺序拷贝成导出快照。
    static std::vector<ForcePidTuningRecordSample> orderedForcePidRecordSamples(
            const std::vector<ForcePidTuningRecordSample>& samples,
            size_t ringStartIndex);
    static void collectForcePidRecordChannels(const ControlWorker::Config& config,
                                              std::vector<int>& axisIndex,
                                              std::vector<int>& sensorIndex);
    static ForcePidTuningRecordSample filteredForcePidRecordSample(
            ForcePidTuningRecordSample sample,
            const std::vector<int>& selectedAxisIndex,
            const std::vector<int>& selectedSensorIndex);
    // 导出指定样本集合，供手动调参和力位混合自动记录复用。
    bool exportForcePidRecordSamples(const std::vector<ForcePidTuningRecordSample>& samples,
                                     const QString& filePrefix,
                                     const QString& startedAtText,
                                     const std::vector<int>& selectedAxisIndex,
                                     const std::vector<int>& selectedSensorIndex,
                                     QString* outputPath = nullptr,
                                     const ControlWorker::Config* configSnapshot = nullptr,
                                     int fallbackSensorCount = -1,
                                     int fallbackAxisCount = -1,
                                     const QString& explicitDirPath = QString(),
                                     QString* errorMessage = nullptr,
                                     bool pumpUiEvents = true,
                                     const std::vector<HybridControlRuntimeRecordSample>* runtimeSamples = nullptr,
                                     const QString& taskId = QString(),
                                     const QString& finishReason = QString());
    // 开始力位混合自动力控记录。
    void startHybridForceControlRecord(
            const std::vector<int>& forceAxisIndex,
            const std::vector<int>& sensorIndex,
            const std::vector<int>& positionAxisIndex = std::vector<int>(),
            const std::vector<std::vector<double>>& positionCommand = std::vector<std::vector<double>>(),
            const std::vector<double>& pvtTimeStamp = std::vector<double>());
    // 结束并导出力位混合自动力控记录。
    bool finishHybridForceControlRecord(bool exportRecord,
                                        const QString& finishReason = QStringLiteral("stopped"));
    // 丢弃当前力位混合自动力控记录。
    void discardHybridForceControlRecord();
    void startHybridControlRuntimeRecord(
            const std::vector<int>& forceAxisIndex,
            const std::vector<int>& positionAxisIndex,
            const std::vector<std::vector<double>>& positionCommand,
            const std::vector<double>& pvtTimeStamp,
            const QString& startedAtText);
    void captureHybridControlRuntimeRecordSample(
            double visualizationTimeSec,
            const ControlWorker::Snapshot& snapshot,
            const ForwardKinematicsSolver::Result& forwardKinematicsResult);
    bool finishHybridControlRuntimeRecord(bool exportRecord,
                                          const QString& finishReason);
    void discardHybridControlRuntimeRecord();
    bool writeHybridControlActualTrajectoryCsv(
            QString* outputPath = nullptr,
            const QString& outputDirPath = QString(),
            const std::vector<HybridControlRuntimeRecordSample>* samples = nullptr,
            const QString& startedAtText = QString(),
            const QString& taskId = QString(),
            const QString& finishReason = QString()) const;
    // 刷新位姿页力位混合力控记录按钮文本。
    void updateHybridForceControlRecordButtonText();
    // 从位姿页按钮启停力位混合力控记录。
    void toggleHybridForceControlRecordFromUi();
    // 返回 PID 调参记录存储目录。
    QString forcePidTuningRecordStorageDirPath() const;
    // 当前是否由用户显式启动了力控高频记录。
    bool forcePidTraceRecordingRequested() const;
    // 根据用户记录状态启停力控高频记录搬运定时器。
    void updateForcePidTraceDrainTimerState();
    void drainForcePidTraceSamples(bool drainAll = false);
    // 处理 ControlWorker 发来的高频传感器调参样本。
    void handleForcePidTuningSensorSample(qint64 wallClockUs,
                                          std::vector<double> forceSensorValue,
                                          std::vector<double> expectedForce,
                                          std::vector<double> motorCommand,
                                          std::vector<double> motorTorqueNm,
                                          std::vector<double> pidOutput,
                                          std::vector<double> pidError,
                                          std::vector<double> pidPTerm,
                                          std::vector<double> pidITerm,
                                          std::vector<double> pidDTerm,
                                          std::vector<double> pidIntegral,
                                          std::vector<double> pidMeasuredDerivativeRaw,
                                          std::vector<double> pidMeasuredDerivativeFiltered,
                                          std::vector<double> pidMeasuredDerivativeControl,
                                          std::vector<double> pidExpectedDerivativeRaw,
                                          std::vector<double> pidExpectedDerivativeFiltered,
                                          std::vector<double> pidFeedForwardRaw,
                                          std::vector<double> pidFeedForwardTerm,
                                          std::vector<double> pidFeedForwardFrictionTerm,
                                          std::vector<int> pidFeedForwardSelectedDynamicProfile,
                                          std::vector<double> pidStaticFrictionDirection,
                                          std::vector<double> pidStaticFrictionSpeedScale,
                                          std::vector<double> pidStaticFrictionRaw,
                                          std::vector<double> pidStaticFrictionAfterFade,
                                          std::vector<double> pidStaticFrictionAfterSmooth,
                                          std::vector<double> pidFeedForwardVelocityTerm,
                                          std::vector<double> pidFeedForwardAccelerationTerm,
                                          std::vector<double> pidExpectedRopeVelocityRadPerSec,
                                          std::vector<double> pidExpectedRopeAccelerationRadPerSec2,
                                          std::vector<double> pidExpectedRateFeedForwardTerm,
                                          std::vector<double> pidExpectedRateFeedForwardScale,
                                          std::vector<double> pidForceRateError,
                                          std::vector<double> pidForceRateErrorDampingTerm,
                                          std::vector<double> pidPlatformCaptureTerm,
                                          std::vector<double> pidPlatformCaptureTargetTerm,
                                          std::vector<int> pidPlatformCaptureState,
                                          std::vector<double> pidFuzzyFeedForwardTargetScale,
                                          std::vector<double> pidFuzzyFeedForwardScale,
                                          std::vector<double> pidFuzzyFeedForwardRecoveryRate,
                                          std::vector<double> pidFuzzyKpScale,
                                          std::vector<double> pidFuzzyKiScale,
                                          std::vector<double> pidFuzzyVelocityDampingScale,
                                          std::vector<double> pidFuzzyPositivePLimit,
                                          std::vector<double> pidFuzzyNegativePLimit,
                                          std::vector<int> pidFuzzyFeedForwardRecoveryLimited,
                                          std::vector<int> pidFuzzyPLimitApplied,
                                          std::vector<int> pidFuzzyState,
                                          std::vector<int> pidIntegralReleaseApplied,
                                          std::vector<int> pidAntiWindup,
                                          std::vector<int> pidOutputLimited,
                                          double controlDtSec,
                                          std::vector<int> torqueSaturated,
                                          std::vector<int> torqueSlewLimited,
                                          std::vector<double> motorVel,
                                          std::vector<int> pid0525HybridState,
                                          std::vector<int> pid0525HybridBiasValid,
                                          std::vector<double> pid0525HybridHoldBiasNm,
                                          std::vector<double> pid0525HybridCaptureForceN,
                                          std::vector<double> pid0525HybridFeedForwardTermNm,
                                          std::vector<double> pid0525HybridFeedbackTermNm,
                                          std::vector<double> pid0525HybridBlend,
                                          std::vector<int> forceControlAxisIndex,
                                          std::vector<int> forceControlSensorIndex);
    // 将主界面力控选择同步到调参 UI。
    void syncForcePidTuningControlEnabled();
    void setupForcePidTuningPidModePages();
    // 用平台位姿刷新 3D/曲线可视化。
    void updateVisualizationFromPlatformPose(const std::vector<std::vector<double>>& platformPose);
    // 处理动捕刚体位姿输入。
    void handleVisualizationRigidPose(const std::vector<std::vector<double>>& rigidPose);
    // 刷新动捕位姿显示文本。
    void updateMocapPoseDisplay(const std::vector<std::vector<double>>& rigidPose);
    // 将轨迹起点 UI 填充为当前动捕位姿。
    void fillTrajectoryStartFromMocapPose();
    // 处理多帧动捕位姿采集成功。
    void handleMotivePoseCaptureCompleted(std::vector<std::vector<double>> rigidPose, int sampleCount);
    // 处理多帧动捕位姿采集失败。
    void handleMotivePoseCaptureFailed(std::string reason, int markerCount);
    // 重置规划轨迹可视化游标。
    void resetPlannedTrajectoryVisualizationCursor();
    // 重启规划轨迹可视化定时器。
    void restartPlannedTrajectoryVisualizationTimer();
    // 停止规划轨迹可视化定时器。
    void stopPlannedTrajectoryVisualizationTimer();
    // 按刷新周期推进规划轨迹可视化。
    void updateVisualizationFromPlannedTrajectory(int refreshIntervalMs);
    // 判断末端轨迹曲线是否已经进入实际执行显示阶段。
    bool isPoseTrajectoryExecutionVisualizationActive() const;
    // 判断末端轨迹曲线是否正在等待仿真结束或 PVT 实际启动。
    bool isPoseTrajectoryVisualizationWaitingForExecution() const;
    // 判断当前是否应显示执行轨迹而不是实际/估计位姿。
    bool shouldUsePlannedTrajectoryVisualization() const;
    // 从活动轨迹显示缓存中采样一个可视化点。
    bool sampleActivePoseTrajectoryDisplayPointForVisualization(
            int pointIndex,
            double& trajectoryTimeSec,
            std::vector<double>& pose,
            std::vector<double>& velocity,
            std::vector<double>& acceleration) const;
    // 根据当前活动轨迹显示点刷新可视化。
    bool updateVisualizationFromCurrentPoseTrajectoryDisplayPoint();
    // 按时间从规划轨迹采样位姿、速度和加速度。
    bool samplePlannedTrajectoryForVisualization(double trajectoryTimeSec,
                                                 std::vector<double>& pose,
                                                 std::vector<double>& velocity,
                                                 std::vector<double>& acceleration) const;
    // 将单个姿态采样写入末端曲线和 3D 视图。
    void updateVisualizationPoseSample(double timeSec,
                                       const std::vector<double>& pose,
                                       const std::vector<double>* velocity = nullptr,
                                       const std::vector<double>* acceleration = nullptr);
    void resetSimulationDataVisualizationPlayback();
    bool startSimulationDataVisualizationPlayback(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& trajectory,
            const std::vector<double>& timeStamp,
            const std::vector<std::vector<double>>& cableLengthTraj,
            const std::vector<std::vector<double>>& cableForceTraj);
    void updateSimulationDataVisualizationPlayback(int refreshIntervalMs);
    int simulationDataVisualizationTimerIntervalMs() const;
    bool sampleSimulationDataVisualizationPoint(int pointIndex,
                                                std::vector<double>& pose,
                                                std::vector<double>& velocity,
                                                std::vector<double>& acceleration,
                                                QVector<double>& cableLength,
                                                QVector<double>& cableSpeed,
                                                QVector<double>& cableTension);
    QVector<double> mapSimulationCablePointForPlot(const std::vector<double>& cablePoint) const;
    QVector<double> mapSimulationCableForcePointForPlot(int pointIndex) const;
    void updateActualEndEffectorVisualizationFromSnapshot(
            double timeSec,
            const ControlWorker::Snapshot& snapshot);
    void updateActualEndEffectorVisualizationSample(double timeSec,
                                                   const std::vector<double>& pose);
    void resetMotorControlInputVisualizationCommand();
    void startMotorControlInputVisualizationFromPvtCommand(
            const std::vector<int>& motorIndex,
            const std::vector<std::vector<double>>& positionUnit,
            const std::vector<double>& timeStamp);
    void updateMotorControlInputVisualizationFromPvtCommand(int refreshIntervalMs);
    bool hasActiveMotorControlInputVisualizationCommand() const;
    QVector<double> mapMotorPositionCommandForPlot(const std::vector<int>& motorIndex,
                                                   const std::vector<double>& positionUnit) const;
    QVector<int> motorControlHybridForceGraphIndexesForPlot() const;
    QVector<double> mapHybridMotorControlInputForPlot(const std::vector<int>& motorIndex,
                                                      const std::vector<double>& positionUnit,
                                                      int pointIndex) const;
    void resetCableKinematicVisualizationState();
    void updateCableKinematicVisualizationFromSnapshot(double timeSec,
                                                       const ControlWorker::Snapshot& snapshot);
    bool buildEncoderCableLengthForVisualization(const std::vector<double>& encoderPosition,
                                                 QVector<double>& cableLength) const;
    bool buildMotorPositionCableLengthForVisualization(const std::vector<double>& motorPosition,
                                                       bool useEncoderHome,
                                                       QVector<double>& cableLength) const;
    std::vector<int> currentForceControlledCableIndicesForEnd(int endIndex) const;
    bool buildCableLengthForVisualizationFromReference(
            const std::vector<double>& motorPosition,
            const std::vector<double>& homeMotorPosition,
            const std::vector<std::vector<double>>& referencePose,
            QVector<double>& cableLength) const;
    // 将电机命令向量映射为曲线通道顺序。
    QVector<double> mapMotorCommandForPlot(const std::vector<double>& motorCommand) const;
    // 将传感器力映射为绳力曲线通道顺序。
    QVector<double> mapCableTensionForPlot(const std::vector<double>& forceSensorValue) const;
    // 返回指定传感器的安全力上限。
    double cableForceLimitForSensorIndex(int sensorIndex) const;
    // 刷新主界面绳力限位指示灯。
    void refreshCableForceLimitSignalUi();
    // 将电机速度映射为绳速曲线通道。
    QVector<double> mapCableSpeedForPlot(const std::vector<double>& motorVelocity) const;
    // 根据平台位姿计算用于绘图的几何绳长。
    QVector<double> buildGeometricCableLengthForPlot(const std::vector<std::vector<double>>& platformPose) const;
    // 将电机反馈单位转换为绳索相关显示值。
    double convertMotorFeedbackToCableValue(int axisIndex, double rawValue) const;
    // 返回轴到绳索角度的比例。
    double motorCableAngleScale(int axisIndex) const;
    // 返回电机硬件方向符号；绳索轴会合并当前机型方向（Lite 相对 G3 反向）。
    double motorHardwareDirectionSign(int axisIndex) const;
    // 将电机反馈单位增量转换为转数。
    double motorFeedbackUnitDeltaToRevolutions(int axisIndex, double unitDelta) const;
    // 读取当前所有轴 Trace 命令原始脉冲快照。
    std::vector<qint64> readCurrentTraceCommandRawPulseSnapshot();
    // 读取指定 Trace 轴的命令原始脉冲快照。
    std::vector<qint64> readCurrentTraceCommandRawPulseSnapshot(const std::vector<int>& traceAxes);
    // 安全零位需要覆盖的 Trace raw 轴。
    std::vector<int> safetyTraceRawAxes() const;
    // 判断 Trace raw 快照是否覆盖指定轴且未读到 0。
    bool isValidTraceRawPulseSnapshotForAxes(const std::vector<qint64>& rawPulse,
                                             const std::vector<int>& traceAxes,
                                             int axisCount) const;
    // 判断 Trace raw 快照是否可作为安全零位。
    bool isValidSafetyTraceRawPulseSnapshot(const std::vector<qint64>& rawPulse,
                                            int axisCount) const;
    // 创建 JOG/在线变速跟随测试 Tab。
    void setupJogFollowTestTab();
    // 创建软件配置项测试/检修 Tab。
    void setupTestMaintenanceTab();
    // 返回 JOG 跟随测试当前选择轴。
    int selectedJogFollowAxisIndex() const;
    // 读取 JOG 跟随测试轴反馈位置。
    bool readJogFollowAxisPosition(int axisIndex, double& position);
    // 刷新 JOG 跟随测试反馈显示。
    void refreshJogFollowFeedback();
    // 开始单轴 JOG 位置跟随测试。
    void startJogFollowPositionTest();
    // 停止单轴 JOG 跟随测试。
    void stopJogFollowTest(const QString& source = QString(), bool sendStop = true);
    // JOG 跟随测试周期计算。
    void handleJogFollowTestTick();
    // 刷新 JOG 跟随测试状态标签。
    void updateJogFollowStatus(const QString& text, const QString& type = QString());
    // 追加 JOG 跟随测试日志。
    void appendJogFollowTestLog(const QString& text);
    // 将电机反馈工程单位转换为 Trace 原始脉冲。
    qint64 motorFeedbackUnitToTraceRawPulse(int axisIndex,
                                            double positionUnit,
                                            bool* ok = nullptr) const;
    int sensorMinX,sensorMaxX,sensorMinY,sensorMaxY,motorMinX,motorMaxX,motorMinY,motorMaxY;// 鏄剧ず鑼冨洿
    std::vector<std::vector<std::vector<double>>> homeCableVec;
    std::vector<std::vector<double>> homeCableLen;
    bool zeroMotorHomeReferenceLoaded = false;
    bool liteWinchReferenceConfirmed = false;
    std::vector<double> zeroMotorHomePos;
    std::vector<double> zeroMotorHomeEncoderPos;
    std::vector<qint64> zeroMotorHomeTraceCommandRawPulse;
    std::vector<std::vector<double>> zeroMotorHomePlatformPose;
    std::vector<double> dynPID_P,dynPID_I,dynPID_D,orgPID_P,orgPID_I,orgPID_D;


    std::vector<double> lastTrajEndMotorTheta;// 涓婁竴娆¤建杩圭粓鐐瑰鐢垫満杞锛岀敤浜庤繛缁建杩?
    std::vector<std::vector<double>> lastTrajEndAnchorCoor;// 涓婁竴娆¤建杩圭粓鐐瑰閿氱偣鍧愭爣锛岀敤浜庤繛缁建杩?
    std::vector<double> lastSyncedMotorSoftwareMinPos;
    std::vector<double> lastSyncedMotorSoftwareMaxPos;
    std::vector<double> lastSyncedMotorSoftwareMaxVel;
    MotorFeedbackDisplayUnit lastMotorFeedbackDisplayUnit = MotorFeedbackDisplayUnit::Revolution;
    bool suppressMotorLimitUnitConversion = false;
    std::vector<std::vector<double>> plannedPoseCableForceTraj;
    std::vector<std::vector<double>> plannedPoseExpectedForceTraj;
    std::vector<std::vector<double>> activeCalculatedExpectedForceTraj;
    std::vector<double> plannedPoseForceTimeStamp;
    std::vector<qint64> plannedPoseBarycenterSolveUs;
    int activeCalculatedExpectedForcePointIndex = -1;
    std::vector<int> activePosModeMotorIndex;
    std::vector<double> activePosModeStartMotorTheta;
    std::vector<double> activePosModeContinuousStartMotorTheta;
    std::vector<double> lastPlannedPoseTrajectoryEndPose;
    struct CircularContinuousMotionParams {
        bool valid = false;
        double centerX = 0.0;
        double centerY = 0.0;
        double centerZ = 0.0;
        double radius = 0.0;
        double durationSec = 0.0;
        int directionIndex = 0;
        double stepTime = 0.0;
    };
    CircularContinuousMotionParams circularContinuousMotionParams;
    bool circularContinuousMotionRestartPending = false;
    quint64 circularContinuousMotionRestartToken = 0;
    
    // 杞ㄨ抗瀹屾垚妫€娴嬪畾鏃跺櫒
    QTimer* trajectoryCheckTimer = nullptr;
    bool isFirstTraj = true;

    bool use3DViewer = true;
    // 初始化 QtDataVisualization 3D 视图。
    bool init3DViewer();
    // 关闭并释放 QtDataVisualization 3D 视图，避免主窗口退出后残留顶层仿真窗口。
    void destroy3DViewer();
    // 更新 3D 视图中的目标点、轨迹点、锚点和绳索线段。
    void update3DViewer(QVector<QVector3D> targetPos, QVector<QVector3D> trajPos, QVector<QVector3D> anchorPos,
                        QVector<QVector3D> cablePos);
    bool is3DViewerReadyForNextInput = false;
    // 将 Qt 坐标系点转换为 803/项目约定坐标系。
    QVector3D to803Frame(QVector3D qtFrame);
    // 解析工作空间包络 OBJ 路径。
    QString workspaceEnvelopeMeshPath(const QString& relativePath) const;
    // 按当前锚点座高度选择可用的工作空间包络。
    bool selectWorkspaceEnvelopeForCurrentHeight(QString* meshPath,
                                                 QVector3D* center,
                                                 QVector3D* halfExtent,
                                                 QString* label,
                                                 QString* reason = nullptr);
    // 响应 3D 仿真窗口中的工作空间包络显示开关。
    void setWorkspaceEnvelopeRequested(bool checked);
    // 按当前开关和锚点座高度刷新工作空间包络显示。
    void refreshWorkspaceEnvelope();
    // 从 3D 图中移除已加载的工作空间包络。
    void removeWorkspaceEnvelope();
    // 回放已缓存的 3D 仿真数据。
    void replay3DViewer();
    // 请求跳过当前3D仿真/重播动画，并按正常完成流程继续。
    void skip3DSimulationPlayback();
    // 根据当前仿真/重播状态刷新3D窗口跳过按钮。
    void refresh3DSimulationSkipButtonState();
    void clearPreReplayData();// 娓呯┖涔嬪墠璁板綍鐨勭敤浜庨噸鎾豢鐪熺殑涓夌淮鏁版嵁
    bool isReplaying = false;
    bool skip3DReplayRequested = false;
    bool skip3DSimulationRequested = false;
    QVector<QVector<QVector3D>> preTargetPos, preTrajPos, preAnchorPos, preCablePos;
    bool isViewShow = false;
    QWidget* main3DViewer = nullptr;
    QWidget* graph3DContainer = nullptr;
    QCheckBox* workspaceEnvelopeCheckBox = nullptr;
    QPushButton* skipSimulationPlaybackButton = nullptr;
//    Q3DSurface *graph;
//    QSurface3DSeries *series;
    Q3DScatter *graph = nullptr;
    QScatter3DSeries *seriesFrame = nullptr;
    QScatter3DSeries *seriesTarget = nullptr;
    QScatter3DSeries *seriesTraj = nullptr;
    QScatter3DSeries *seriesAnchor = nullptr;
    QScatter3DSeries *seriesCable = nullptr;
    QScatter3DSeries *seriesAnchor2Cable = nullptr;
    QCustom3DItem* workspaceEnvelopeItem = nullptr;
    bool workspaceEnvelopeRequested = false;
    QString workspaceEnvelopeActiveMeshPath;

    // 向主界面信息区输出一条消息。
    void displayInfo(std::string info, std::string tpye = "normal");
    // 清空主界面信息区。
    void clearInfo();
    bool hasReportErr = false;
    // 末端数量变化后刷新对应 UI 和参数数组。
    void endChange(int areaNum);
    // 将 G3 模板的绳索接点和出绳点几何写入 UI。
    void applyG3TemplateGeometry();
    // 将 Lite 模板的绳索接点和出绳点几何写入 UI。
    void applyLiteTemplateGeometry();
    // 刷新运行期轴数量和力传感器数量缓存。
    void refreshAxisRuntimeCounts();
    // 初始化主操作按钮/状态提示 UI。
    void initializePrimaryOperationUi();
    // 将控制界面中的参数 frame 移入弹窗，并连接入口按钮。
    void setupParameterFrameDialogs();
    // 初始化硬件连接状态 UI。
    void initializeConnectionStatusUi();
    // 初始化旧版 GC 控件兼容显示。
    void initializeLegacyGcUiFallbacks();
    // 设置单个连接状态指示标签。
    void setConnectionStatusIndicator(QLabel* label,
                                      const QString& title,
                                      HardwareInterface::ConnectionState state,
                                      const QString& tooltip = QString(),
                                      const QString& stateTextOverride = QString());
    // 刷新控制器、电机和传感器连接状态显示。
    void refreshConnectionStatusUi(bool force = false);
    // 根据机器人状态快照刷新主操作按钮联锁。
    void refreshPrimaryOperationUiState(const RobotStateSnapshot& robotState);
    // 显示主操作反馈文本并设置过期时间。
    void showPrimaryOperationFeedback(const QString& text, const QString& type, int durationMs = 1800);
    // 初始化零位/运行位姿标定 UI。
    void initializeCalibrationUi();
    // 根据标定状态机刷新按钮和说明文本。
    void refreshCalibrationUiState();
    // 返回安装目录下 data 的根目录；安装器只对该目录授予普通用户写权限。
    QString installationDataRootDirPath() const;
    // 首次启动时创建运行数据目录，并迁移旧版本直接放在安装目录根下的数据。
    bool initializeInstallationDataStorage(QString* errorMessage = nullptr);
    // 在资源管理器中打开安装目录下的运行数据目录。
    void openInstallationDataFolder();
    // 返回标定快照存储目录。
    QString calibrationStorageDirPath() const;
    // 返回最新标定快照路径。
    QString latestCalibrationSnapshotPath() const;
    // 返回零位电机参考快照路径。
    QString zeroMotorHomeSnapshotFilePath() const;
    // 返回已知运行位姿快照路径。
    QString knownRuntimePoseSnapshotFilePath() const;
    // 返回直线模组高度参考文件路径。
    QString linearModuleHeightReferenceFilePath() const;
    // 将标定向量格式化为短预览文本。
    QString formatCalibrationVectorPreview(const std::vector<double>& values, int maxCount = 4) const;
    // 保存当前标定快照到指定文件。
    bool saveCalibrationSnapshotToFile(const QString& filePath, bool announce = true);
    // 保存最近零位校准记录和带时间戳历史记录到指定目录。
    bool saveCalibrationSnapshotRecordSetToDir(const QString& dirPath, bool announceSuccess = true);
    // 从指定文件加载标定快照。
    bool loadCalibrationSnapshotFromFile(const QString& filePath, bool announce = true);
    // 保存零位电机和平台位姿参考。
    bool saveZeroMotorHomeSnapshot(const std::vector<double>& motorHome,
                                   bool announce = false,
                                   const std::vector<std::vector<double>>& zeroPlatformPose = std::vector<std::vector<double>>(),
                                   const std::vector<double>& motorHomeEncoder = std::vector<double>(),
                                   const std::vector<qint64>& traceCommandRawPulse = std::vector<qint64>(),
                                   bool liteWinchReference = false);
    // 加载零位电机和平台位姿参考。
    bool loadZeroMotorHomeSnapshot(std::vector<double>& motorHome,
                                   std::vector<std::vector<double>>& zeroPlatformPose,
                                   bool announce = false,
                                   std::vector<double>* motorHomeEncoder = nullptr,
                                   std::vector<qint64>* traceCommandRawPulse = nullptr,
                                   bool* liteWinchReference = nullptr);
    // 连接硬件后自动应用保存的零位参考。
    bool applySavedZeroMotorHomeSnapshotOnConnect(bool announce = false);
    // 启动/重连后使用当前上电读取的电机位置作为本次临时零位。
    // Lite 同时以同一时刻的 8 轴 Trace 建立仅本次连接有效的安全相对位置基准。
    bool applyStartupMotorHomeReference();
    bool latchLiteFullSystemSessionSafetyReference(
            const std::vector<double>& motorHome,
            const QString& operationName);
    // 零位校准1：执行机械回零到零点位姿上方。
    void startZeroCalibrationWorkflow();
    // 零位校准2：开启全绳力控，等待用户预紧确认。
    void startZeroCalibrationPretensionWorkflow();
    // 内部停止当前校准状态。
    void stopZeroCalibrationWorkflow(bool announce = true);
    // 进入预紧平衡阶段。
    bool beginCalibrationPretensionStage(const QString& sourceName);
    // 确认零位标定并保存相关参考。
    bool confirmZeroCalibrationWorkflow();
    // 用当前电机位置确认已知运行位姿。
    bool confirmKnownRuntimePoseFromCurrentMotorPosition(bool keepForceControlThreadRunning = false);
    // Lite 专用：更新动捕后，将当前静止预紧状态确认为绞盘轴向零位、motorHome 和安全行程基准。
    void requestLiteWinchReferenceConfirmation();
    bool finalizeLiteWinchReferenceConfirmation(
            const std::vector<std::vector<double>>& rigidPose);
    // 读取已知运行位姿 UI 输入，单位 mm/deg。
    std::vector<double> knownRuntimePoseInputMmDeg() const;
    // 读取已知运行位姿 UI 输入并转换为 rad。
    std::vector<double> knownRuntimePoseInputRad() const;
    // 按已知位姿重新计算并刷新期望绳力。
    bool refreshExpectedForceForKnownRuntimePose(const std::vector<double>& knownPose,
                                                 const QString& operationName);
    // 构建设计零位姿，可附加 Z 偏移。
    std::vector<double> buildCalibrationDesignHomePose(double zOffsetMm = 0.0) const;
    // 生成机械回零用两段末端轨迹。
    bool buildCalibrationMechanicalHomingTrajectorySegments(
            std::vector<std::vector<std::vector<std::vector<double>>>>& firstSegment,
            std::vector<std::vector<std::vector<std::vector<double>>>>& secondSegment,
            QString& errorMessage) const;
    // 生成机械回零用末端轨迹。
    bool buildCalibrationMechanicalHomingTrajectory(
            std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            QString& errorMessage) const;
    // 将机械回零末端轨迹转换成 PVT 电机命令。
    bool buildCalibrationMechanicalHomingPvtCommand(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            PvtExecutionWorker::PvtCommand& command,
            QString& errorMessage,
            const HybridPoseForceModeConfig* hybridModeConfig = nullptr);
    // 等待机械回零 PVT 段真正到达末点。
    bool waitCalibrationMechanicalHomingPvtComplete(
            const PvtExecutionWorker::PvtCommand& command,
            const QString& segmentName,
            int timeoutMs,
            int sampleMs);
    // 执行机械回零单段 PVT。
    bool executeCalibrationMechanicalHomingSegment(
            const QString& sourceName,
            const QString& segmentName,
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            bool finalSegment,
            const HybridPoseForceModeConfig* hybridModeConfig = nullptr);
    // 执行机械回零 PVT。
    bool executeCalibrationMechanicalHoming(
            const QString& sourceName,
            const HybridPoseForceModeConfig* hybridModeConfig = nullptr);
    // 初始化参数配置 UI。
    void initializeParameterConfigUi();
    // 初始化力传感器 Trace 测试 UI。
    void initializeForceTraceTestUi();
    // 填充力传感器 Trace 默认对象。
    void populateForceTraceDefaultObjects();
    // 根据 UI 参数运行 PDO Trace 探针。
    void runPdoTraceProbeFromUi();
    // 根据 UI 参数运行通用 Trace 探针。
    void runTraceProbeFromUi();
    // 追加力传感器 Trace 测试日志。
    void appendForceTraceTestLog(const QString& text);
    // 返回参数配置文件目录。
    QString parameterConfigStorageDirPath() const;
    // 返回力控参数自动保存文件路径。
    QString forcePidParameterAutoSaveFilePath() const;
    // 返回安装目录 data 下的力控参数自动保存文件路径。
    QString forcePidParameterAutoSaveLocalFilePath() const;
    // 返回力控参数自动保存候选文件路径列表。
    QStringList forcePidParameterAutoSaveCandidateFilePaths() const;
    // 返回 UI 事件日志目录。
    QString uiEventLogDirPath() const;
    // 返回 UI 事件日志文件路径。
    QString uiEventLogFilePath() const;
    // 返回结构化故障日志路径。
    QString structuredFaultLogFilePath() const;
    // 返回软件故障守护日志路径。
    QString softwareFaultGuardLogFilePath() const;
    // 返回软件故障守护启动提示去重标记路径。
    QString softwareFaultGuardNoticeStateFilePath() const;
    // 返回运行诊断报告文件路径。
    QString runtimeDiagnosticsReportFilePath() const;
    // 返回电机 Trace 恢复窗口文件路径。
    QString motorTraceRecoveryWindowFilePath() const;
    // 返回直线模组 Trace 恢复窗口文件路径。
    QString linearModuleTraceRecoveryWindowFilePath() const;
    // 返回位姿仿真绳长导出文件路径。
    QString poseSimulationCableLengthFilePath(const QString& outputDirPath = QString()) const;
    // 返回位姿仿真 PVT 表导出文件路径。
    QString poseSimulationPvtCommandTableFilePath(const QString& outputDirPath = QString()) const;
    // 返回位姿仿真数据和关键计算结果 CSV 导出文件路径。
    QString poseSimulationResultCsvFilePath(const QString& outputDirPath = QString()) const;
    // 返回最近程序控制正运动学实际轨迹 CSV 导出文件路径。
    QString hybridControlActualTrajectoryFilePath(
            const QString& outputDirPath = QString(),
            const QString& startedAtText = QString()) const;
    // 返回 PVT 命令与 Trace 对比文件路径。
    QString posePvtTraceCommandCompareFilePath() const;
    // 返回运行结果快照文件路径。
    QString runtimeResultSnapshotFilePath() const;
    // 返回运行结果快照候选路径列表。
    QStringList runtimeResultSnapshotCandidateFilePaths() const;
    // 返回保存当前位姿候选路径，包含已知位姿和零位历史记录。
    QStringList savedCurrentPoseSnapshotCandidateFilePaths() const;
    // 构建当前参数配置 JSON 快照。
    QJsonObject buildParameterConfigSnapshot() const;
    // 构建力控参数自动保存 JSON 快照。
    QJsonObject buildForcePidParameterAutoSaveSnapshot() const;
    // 构建零位配置 JSON 快照。
    QJsonObject buildZeroPoseConfigSnapshot() const;
    // 构建零位电机参考 JSON 快照。
    QJsonObject buildZeroMotorHomeReferenceSnapshot() const;
    // 构建最近一次运行结果 JSON 快照。
    QJsonObject buildLastRuntimeResultSnapshot() const;
    // 判断是否存在已恢复的运行预紧快照。
    bool hasRestoredRuntimePretensionSnapshot() const;
    // 进入零位预紧状态。
    void enterZeroPosePretensionState(const QString& infoMessage = QString());
    // 进入已恢复运行位姿预紧状态。
    void enterRestoredRuntimePretensionState();
    // 缓存位姿 PVT 运行结果，供后续恢复和诊断。
    bool cachePosePvtRuntimeResult();
    // 将已完成的位姿轨迹末点同步到当前位姿缓存。
    bool syncCompletedPoseTrajectoryEndForRuntimeSnapshot();
    // 清空 PVT 命令与 Trace 对比状态。
    void clearPosePvtTraceCommandComparison();
    // 开始记录 PVT 命令与驱动 Trace 命令/反馈的对比。
    bool beginPosePvtTraceCommandComparison(
            const std::vector<int>& motorIndex,
            const std::vector<std::vector<double>>& motorPosTraj,
            const std::vector<double>& timeStamp,
            const std::vector<qint64>& encoderReferenceRawPulse,
            const std::vector<double>& encoderReferenceValue,
            const QString& encoderReferenceSource,
            bool pvtPositionUsesEncoderZeroReference);
    // 采样一次 PVT 命令与 Trace 对比数据。
    void samplePosePvtTraceCommandComparison();
    // 写出 PVT 命令与 Trace 对比文件。
    bool writePosePvtTraceCommandComparisonFile(const QString& finishReason,
                                                bool announce = true);
    // 保存参数配置到文件。
    bool saveParameterConfigToFile(const QString& filePath, QString* errorMessage = nullptr) const;
    // 退出时自动保存力控参数。
    bool saveForcePidParameterAutoSave(QString* errorMessage = nullptr) const;
    // 保存力控参数到指定文件。
    bool saveForcePidParameterAutoSaveToFile(const QString& filePath,
                                             const QJsonObject& snapshot,
                                             QString* errorMessage = nullptr) const;
    // 初始化力控 PID 参数自动保存。
    void setupForcePidParameterAutoSave();
    // 延迟自动保存力控 PID 参数。
    void scheduleForcePidParameterAutoSave();
    // 立即保存力控 PID 参数。
    void flushForcePidParameterAutoSave();
    // 保存零位配置到文件。
    bool saveZeroPoseConfigToFile(const QString& filePath, QString* errorMessage = nullptr) const;
    // 保存最近一次运行结果快照。
    bool saveLastRuntimeResultSnapshot(bool announce = false);
    // 位姿 PVT 完成后保存运行结果快照。
    bool saveLastRuntimeResultSnapshotAfterPosePvt(bool announce = false);
    // 按节流规则自动保存运行结果快照。
    void maybeAutoSaveLastRuntimeResultSnapshot();
    // 加载最近一次运行结果快照。
    bool loadLastRuntimeResultSnapshot(bool announce = false);
    // 加载保存的当前位姿快照。
    bool loadSavedCurrentPoseSnapshot(bool announce = false,
                                      const QString& preferredFilePath = QString());
    // 从文件加载参数配置。
    bool loadParameterConfigFromFile(const QString& filePath, QString* errorMessage = nullptr);
    // 保存程序初始化完成后的界面参数默认值。
    void captureDefaultParameterConfigSnapshot();
    // 用默认参数补全导入的界面参数，并返回参数校验结果。
    QJsonObject completeParameterConfigSnapshotWithDefaults(
            const QJsonObject& snapshot,
            QString* validationMessage = nullptr) const;
    // 启动时自动读取力控参数。
    bool loadForcePidParameterAutoSave(QString* errorMessage = nullptr);
    // 将内置力控基线作为当前默认参数写入 UI。
    void applyBuiltInForcePidDefaults();
    // 按当前机型收紧实际转矩、力控命令和直接转矩调试的 UI 上限。
    void enforceMachineTorqueLimitsOnUi();
    // 先恢复内置基线，再用参数文件中的合法字段覆盖；返回缺失/非法字段警告。
    void applyForcePidParameterSnapshot(const QJsonObject& snapshot,
                                        QString* warningMessage = nullptr);
    // 将 JSON 参数快照应用到 UI 和内部缓存。
    void applyParameterConfigSnapshot(const QJsonObject& snapshot);
    // 返回当前电机反馈显示单位。
    MotorFeedbackDisplayUnit currentMotorFeedbackDisplayUnit() const;
    // 刷新电机限位单位 UI，可选择同步转换数值。
    void refreshMotorLimitUnitUi(bool convertValues);
    // 应用参数模板。
    void applyParameterTemplate(const QString& templateName);
    // 将模板阻抗参数写入 UI。
    void applyTemplateImpedanceParametersToUi();
    // 追加消息历史记录。
    void appendMessageHistoryEntry(const QString& message, const QString& type);
    // 写入 UI 事件日志。
    void appendUiEventLog(const QString& message, const QString& type);
    // 批量刷写 UI 事件日志，避免 displayInfo 每条消息同步 flush 卡住界面。
    void flushUiEventLog();
    // 独立结构化故障日志线程；停止时以队列屏障保证此前记录先处理。
    void initializeStructuredFaultLogWriter();
    void stopStructuredFaultLogWriter();
    // 写入结构化故障日志。
    void appendStructuredFaultLog(const QJsonObject& record) const;
    // 启动时检查上次运行的软件异常捕获记录并提示一次。
    void reportSoftwareFaultGuardStartupNotice();
    // 执行启动/上电自检并更新自检状态。
    void runStartupSelfCheck(bool announcePass = true, bool logResult = true);
    // 采集当前自检项目结果。
    StartupSelfCheckResult evaluateStartupSelfCheck() const;
    // 运动前确认自检已通过。
    bool ensureStartupSelfCheckReadyForMotion(const QString& actionName);
    // 初始化会话记录 UI。
    void initializeSessionRecordingUi();
    // 刷新会话记录按钮和状态。
    void refreshSessionRecordingUi();
    // 开始采集当前会话数据。
    bool startSessionRecording();
    // 停止会话采集并可提示用户。
    bool stopSessionRecording(bool announce = true);
    // 从控制快照捕获一帧会话记录样本。
    void captureSessionRecordingSample(const ControlWorker::Snapshot& snapshot);
    // 捕获一次实际下发的 PVT 位置控制指令表。
    void captureSessionRecordingPvtPositionCommand(
            const std::vector<int>& motorIndex,
            const std::vector<std::vector<double>>& positionUnit,
            const std::vector<double>& timeStamp,
            const QString& source);
    void captureSessionRecordingPvtControlCycleTiming(int pvtPointCount,
                                                      int pvtAxisCount,
                                                      double pvtGenerationElapsedMs,
                                                      int sourceStartPointIndex = 0);
    void seedSessionRecordingPvtControlCycleRecordsFromLastSuccess();
    void updateLastSuccessfulPvtControlCycleCache(int pvtPointCount,
                                                  int pvtAxisCount);
    bool attachLatestPvtUploadTimingToRecord(SessionRecordingPvtControlCycleRecord& record,
                                             int pvtPointCount,
                                             int pvtAxisCount,
                                             qint64 startMs,
                                             qint64 endMs);
    void attachLatestPvtUploadTimingToSessionRecordingCycle(int pvtPointCount,
                                                             int pvtAxisCount);
    void applyPvtTraceStartDelayMeasurement(qint64 pvtUploadMonotonicUs,
                                            int pointCount,
                                            int axisCount,
                                            quint32 commandStartFrameSequence,
                                            quint32 feedbackStartFrameSequence,
                                            quint64 frameIntervalCount,
                                            int ethercatBusCycleUs,
                                            qint64 delayUs,
                                            int commandStartAxis,
                                            int feedbackStartAxis);
    void refreshPvtTraceStartDelayFromHardwareHistory();
    double pvtControlCycleMaxUs(const SessionRecordingPvtControlCycleRecord& record) const;
    QString latestPvtControlCycleMaxInfoText() const;
    QString appendLatestPvtControlCycleMaxInfoText(const QString& message) const;
    // 构建会话记录导出文本。
    QString buildSessionRecordingExportText() const;
    // 写入会话记录中的原始诊断段。
    bool writeSessionRecordingDiagnosticRawSections(QTextStream& stream,
                                                     qint64 sourceRowIndexOffset,
                                                     QString* errorMessage = nullptr);
    // 返回会话记录无法导出的原因，空字符串表示可导出。
    QString sessionRecordingExportNoDataReason() const;
    // 导出会话记录文件。
    bool writeSessionRecordingExport(QString* outputPath = nullptr,
                                     bool announce = true,
                                     const QString& outputDirPath = QString());
    // 更新运行诊断历史和 UI。
    void updateRuntimeDiagnostics();
    // 重置运行诊断状态，可选择同时重置源统计。
    void resetRuntimeDiagnosticsState(bool resetSources = true);
    // 裁剪运行诊断历史窗口。
    void trimRuntimeDiagnosticsHistory(qint64 nowMs);
    // 汇总当前运行诊断窗口统计。
    RuntimeDiagnosticsSummary buildRuntimeDiagnosticsSummary() const;
    // 刷新运行诊断 UI。
    void refreshRuntimeDiagnosticsUi();
    // 构建运行诊断中的轨迹状态快照。
    QJsonObject buildRuntimeDiagnosticsTrajectoryStatusJson() const;
    // 构建运行诊断中的消息历史快照。
    QJsonObject buildRuntimeDiagnosticsMessageHistoryJson(int maxEntries = 100) const;
    // 构建运行诊断中的 UDP 统计快照。
    QJsonObject buildRuntimeDiagnosticsUdpStatsJson() const;
    // 构建运行诊断中的故障记录快照。
    QJsonObject buildRuntimeDiagnosticsFaultRecordsJson(int maxRecords = 50) const;
    // 构建运行诊断必检条目汇总。
    QJsonObject buildRuntimeDiagnosticsItemSummariesJson() const;
    // 返回运行诊断无法导出的原因，空字符串表示可导出。
    QString runtimeDiagnosticsExportNoDataReason() const;
    // 输出当前电机 Trace 恢复状态提示。
    void announceMotorTraceRecoveryState();
    // 上电后刷新电机 Trace 恢复状态。
    void refreshMotorTraceRecoveryStateAfterPowerOn(bool announce);
    // 启动完成后等待 Trace 首帧就绪，再刷新电机 Trace 恢复状态。
    void scheduleMotorTraceRecoveryStateRefreshAfterStartup(bool announce);
    void runMotorTraceRecoveryStateRefreshAfterStartup(bool announce,
                                                       int attempt,
                                                       int token);
    // 从 UI 触发电机 Trace 恢复 PVT。
    bool restoreMotorTraceRecoveryPvtFromUi();
    // 保存直线模组 Trace 恢复窗口。
    bool saveLinearModuleTraceRecoveryWindow(const QString& reason, bool announce = false);
    // 上电后刷新直线模组 Trace 恢复状态。
    void refreshLinearModuleTraceRecoveryStateAfterPowerOn(bool announce);
    // 从 UI 触发直线模组 Trace 恢复 PVT。
    bool restoreLinearModuleTraceRecoveryPvtFromUi();
    // 构建运行诊断报告 JSON。
    QJsonObject buildRuntimeDiagnosticsReportJson() const;
    // 写出运行诊断报告文件。
    bool writeRuntimeDiagnosticsReport(QString* outputPath = nullptr,
                                       bool announce = false,
                                       const QString& outputDirPath = QString());
    // 返回软件配置项测试证据导出目录。
    QString testEvidenceDirPath() const;
    // 刷新测试/检修 Tab 中的软件、路径和状态摘要。
    void refreshTestMaintenanceInfo();
    // 刷新测试接口模拟按钮状态。
    void refreshTestInterfaceSimulationUi();
    // 切换测试接口不连通模拟状态。
    void setTestInterfaceDisconnectedInjected(bool enabled);
    // 追加测试/检修操作日志。
    void appendTestMaintenanceLog(const QString& text, const QString& type = QString());
    // 运行软件配置项本地自检。
    void runSoftwareConfigurationSelfTest();
    // 按软件配置项测试报告生成步骤级证据矩阵。
    void runSoftwareConfigurationReportStepTests();
    // 将自检记录刷新到表格。
    void refreshSoftwareConfigurationSelfTestTable();
    // 构建测试/检修信息 JSON。
    QJsonObject buildMaintenanceInfoJson() const;
    // 构建自检证据 JSON。
    QJsonObject buildSoftwareConfigurationSelfTestJson() const;
    // 构建自检证据 Markdown。
    QString buildSoftwareConfigurationSelfTestMarkdown() const;
    // 导出自检证据 JSON 和 Markdown。
    bool exportSoftwareConfigurationSelfTestReport(bool announce = true,
                                                  const QString& outputDirPath = QString());
    // 导出检修日志和诊断摘要。
    bool exportMaintenanceLogBundle(bool announce = true,
                                    const QString& outputDirPath = QString());
    // 处理检修界面的本地程序更新包入口。
    void validateUpdatePackageForMaintenance();
    // 从仿真结果构建位姿 PVT 电机命令表。
    bool buildPosePvtCommandTableFromSimulation(
            const HybridPoseForceModeConfig& hybridModeConfig,
            PosePvtCommandTable& table,
            QString& errorMessage,
            bool validateMotorLimits) const;
    // 导出位姿仿真绳长文件。
    bool writePoseSimulationCableLengthFile(QString* outputPath = nullptr,
                                            const QString& outputDirPath = QString());
    // 导出位姿仿真 PVT 命令表。
    bool writePoseSimulationPvtCommandTableFile(QString* outputPath = nullptr,
                                                const QString& outputDirPath = QString());
    // 导出位姿仿真数据和关键计算结果 CSV 长表。
    bool writePoseSimulationResultCsvFile(QString* outputPath = nullptr,
                                          const QString& outputDirPath = QString());
    // 从 UI 输入或浏览选择目录并导出位姿仿真数据和关键计算结果。
    void exportPoseSimulationResultsFromUi();
    // 将机器人状态快照编码为 JSON。
    QJsonObject buildRobotStateSnapshotJson(const RobotStateSnapshot& state) const;
    // 将运行态内部状态编码为 JSON。
    QJsonObject buildRuntimeStateSnapshotJson() const;
    // 将 ControlWorker 快照编码为 JSON。
    QJsonObject buildControlSnapshotJson(const ControlWorker::Snapshot& snapshot) const;
    // 构建结构化故障记录。
    QJsonObject buildStructuredFaultRecord(const QString& eventType,
                                          int level,
                                          int code,
                                          const QString& summary,
                                          const QString& detail,
                                          bool stopActionAttempted,
                                          bool stopActionSucceeded,
                                          const QString& note = QString()) const;
    // 显示消息历史对话框。
    void showMessageHistoryDialog();
    // 打开 UI 事件日志目录。
    bool openUiEventLogDirectory(bool announceFailure = true);
    // 打开 UI 事件日志文件。
    bool openUiEventLogFile(bool announceFailure = true);
    // 安装软件异常/崩溃守护回调。
    void installSoftwareFaultGuards();
    // 卸载软件异常/崩溃守护回调。
    void uninstallSoftwareFaultGuards();

    // 闆疯禌鎺у埗鍗?
    // 批量打开或关闭电机控制器使能。
    bool setMotorControllerEnable(bool enable);// 鍚姩/鍏抽棴鐢垫満浣胯兘
    // 从 UI 切换伺服保持状态。
    bool toggleServoHoldFromUi();
    // 执行人工停机，停止 PVT、力控、单轴和直线模组动作。
    bool motorStop();// 鐢垫満鍋滄
    // 根据当前 PVT 状态切换暂停/恢复。
    bool togglePauseResumeFromUi();
    // 响应外部控制盒停止命令。
    bool stopMotionFromControlBox();
    // 执行电机回零流程。
    bool motor2Home();// 鐢垫満褰掗浂浣?
    // 将所有绳索电机按当前机型的放绳方向移动半圈，用于预紧/调试。
    bool moveAllCableMotorsNegativeHalfTurn();
    // 将指定轴按保存的起点角度返回。
    void returnMotorAxesToStart(const std::vector<int>& motorIndex, const std::vector<double>& targetTheta);
    // 执行位置模式：规划末端轨迹、仿真绳长并准备 PVT。
    bool runPoseMode();// 鎵ц浣嶇疆妯″紡
    // 将位控仿真结果转换为电机 PVT 并下发执行。
    bool startPosePvtTrajectory();// 灏嗕綅鎺т豢鐪熺粨鏋滀笅鍙戣嚦闆疯禌 PVT
    // 位姿仿真线程结束后的统一收尾。
    void handlePoseModeSimulationFinished();
    // 单轴调试：切换选中电机使能。
    void singleMotorEnable();
    // 单轴调试：启动点位运动。
    void singleMotorStart();
    // 单轴调试：停止点位运动。
    void singleMotorStop();
    // 单轴调试：刷新目标/当前位置。
    void singleMotorUpdatePos();
    // 更新单轴实际位置显示。
    void updateSingleMotorActualPos();
    // 根据运行态刷新单轴使能按钮状态。
    void refreshSingleMotorEnableStateUi();
    // 返回单轴点动 UI 选中的轴。
    int selectedSingleMotorIndex() const;
    // 初始化直线模组高度控制 UI。
    void initializeLinearModuleHeightUi();
    // 判断当前模板是否启用直线模组高度模块；G3始终保留，Lite由模板开关控制。
    bool linearModuleHeightModuleEnabled() const;
    // 返回配置为直线模组的轴索引。
    std::vector<int> configuredLinearModuleAxisIndices() const;
    // 加载直线模组高度参考。
    bool loadLinearModuleHeightReference(bool announce = false);
    // 保存直线模组高度参考。
    bool saveLinearModuleHeightReference(double referenceHeightM, bool announce = true);
    // 确保直线模组高度参考存在，必要时建立默认值。
    bool ensureLinearModuleHeightReference(bool allowDefaultReference, bool announce = false);
    // 读取当前直线模组轴、高度和 Trace 原始脉冲。
    bool currentLinearModuleHeights(std::vector<int>& axes,
                                    std::vector<qint64>& traceCommandRawPulse,
                                    std::vector<double>& heightsM,
                                    QString* errorMessage = nullptr,
                                    bool reloadReference = true);
    // 将直线模组 Trace 原始命令脉冲转换为电机单位。
    double linearModuleTraceCommandRawToMotorUnit(int axisIndex,
                                                 qint64 traceCommandRawPulse,
                                                 bool* ok = nullptr) const;
    // 将直线模组 Trace 原始命令脉冲转换为高度米值。
    double linearModuleTraceCommandRawToHeightM(int axisIndex,
                                               qint64 traceCommandRawPulse) const;
    // 捕获当前直线模组高度作为上锚点参考。
    void captureLinearModuleAnchorReference();
    // 用当前直线模组高度同步工作空间高度。
    bool syncWorkspaceHeightWithLinearModuleHeight(const std::vector<double>& heightsM,
                                                   bool announce = false);
    // 用当前直线模组高度同步上锚点起始位置。
    bool syncUpperAnchorStartPosWithLinearModuleHeight(bool announce = false);
    // 用指定高度同步上锚点起始位置。
    bool syncUpperAnchorStartPosWithLinearModuleHeight(const std::vector<double>& heightsM,
                                                       bool announce = false);
    // 刷新直线模组高度 UI。
    bool refreshLinearModuleHeightUi(bool announce = false);
    // 校验直线模组轴当量是否可用于运动。
    bool validateLinearModuleAxisEquivForMotion(const std::vector<int>& axes,
                                                const QString& actionName);
    // 使能直线模组电机。
    bool enableLinearModuleMotors();
    // 失能直线模组电机。
    bool disableLinearModuleMotors(const std::vector<int>& axes, bool announce = false);
    // 启动直线模组高度移动。
    bool startLinearModuleHeightMove();
    // 停止直线模组高度移动。
    bool stopLinearModuleHeightMove();
    // 清除直线模组移动状态。
    void clearLinearModuleHeightMoveState();
    // 刷新直线模组移动完成/超时状态。
    void refreshLinearModuleHeightMoveState();
    // 更新直线模组状态文本。
    void updateLinearModuleHeightStatus(const QString& text, const QString& type = QString());
    // 标记 PVT 保护轴，防止力控同时改写这些轴。
    void setPvtMotionProtection(const std::vector<int>& axisIndex);
    // 清除 PVT 保护轴。
    void clearPvtMotionProtection();
    // 刷新 PVT 保护超时/结束状态。
    void refreshPvtMotionProtectionState();
    // 构建 ControlWorker 使用的受保护轴掩码。
    std::vector<bool> protectedPvtAxisMask(int axisCount) const;
    // PVT 执行前关闭直接力控，避免命令冲突。
    void disableDirectForceControlForPvtMotion();
    // 清除单电机点动状态。
    void clearSingleMotorPointMoveState();
    // 刷新单电机点动完成/超时状态。
    void refreshSingleMotorPointMoveState();
    // 手动电机测试前关闭力控。
    void disableForceControlForManualMotorTest();
    // 校验单轴运动目标和速度是否满足软件限位。
    bool validateMotorCommandLimits(int axisIndex,
                                    double relativeTargetPos,
                                    double targetVel,
                                    const QString& actionName,
                                    QString* errorMessage = nullptr);
    bool buildMotorCommandLimitSnapshots(
            const std::vector<int>& motorIndex,
            const std::vector<double>& motorHome,
            const QString& actionName,
            std::vector<MotorCommandLimitSnapshot>& snapshots,
            QString* errorMessage = nullptr);
    bool validateMotorCommandLimitsWithSnapshot(
            const MotorCommandLimitSnapshot& snapshot,
            double relativeTargetPos,
            double targetVel,
            const QString& actionName,
            QString* errorMessage = nullptr) const;
    // 校验单轴相对位移命令是否满足软件限位。
    bool validateMotorRelativeMoveLimits(int axisIndex,
                                         double relativeMoveDist,
                                         double targetVel,
                                         const QString& actionName,
                                         QString* errorMessage = nullptr);
    // 等待所有电机位置在容差内稳定。
    bool waitMotorPositionStable(int timeoutMs = 800, int sampleMs = 50, double tolerance = 0.002);
    // 等待所有电机轴运动完成。
    bool waitMotorAxesDone(int timeoutMs = 30000, int sampleMs = 50);
    // 同步当前电机零位并刷新显示零点。
    void syncMotorHomeAndDisplayZero(const QString& infoMessage = QString(),
                                     bool forceUseCurrentPosition = false);
    // 将所有电机当前位置设为零位。
    void setAllMotorHomeToCurrentPosition();
    // 清除显示零点，回到硬件零位显示。
    void resetMotorPosDisplayZero();
    // 刷新电机位置显示。
    void refreshMotorPosDisplay();

    void readTrajFile();// 璇诲彇杞ㄨ抗鐐规枃浠?
    bool useTrajFile = false;
    bool trajFilePreserveImportedTimeStep = false;
    std::vector<double> trajFileTrajPx1,trajFileTrajPy1,trajFileTrajPz1,trajFileTrajRx1,trajFileTrajRy1,trajFileTrajRz1;
    std::vector<std::vector<std::vector<std::vector<double>>>> trajFileOfflineTraj;
    struct TrajectoryFileLoadInfo {
        bool attempted = false;
        bool success = false;
        QString source;
        QString path;
        QString message;
        int endNum = 0;
        int pointNum = 0;
        int segmentPointCount = 0;
        int segmentCount = 0;
        double duration = 0.0;
        bool hasTimeStep = false;
        bool uniformTimeStep = false;
        double minTimeStep = 0.0;
        double avgTimeStep = 0.0;
        double maxTimeStep = 0.0;
    };
    TrajectoryFileLoadInfo trajFileLoadInfo;
    std::vector<std::pair<int, int>> trajFileSegmentRanges;
    bool trajectoryFileProgramBatchActive = false;
    int trajectoryFileProgramSegmentIndex = -1;
    std::vector<std::vector<std::vector<std::vector<double>>>> trajectoryFileProgramFullPlannedTraj;
    std::vector<double> trajectoryFileProgramFullPlannedLambda;

    // 参数变化后联动刷新 UI、硬件配置和缓存。
    void paraChange();
    // 将 G3 模板的默认参数写入 UI。
    void applyG3TemplateParameters();
    // 将 Lite 模板的默认参数写入 UI。
    void applyLiteTemplateParameters();
    // 将 UI 参数写入内部缓存和硬件接口。
    void updatePara();
    // 从 UI 应用雷赛硬件配置。
    bool applyLeadshineHardwareConfigFromUi(QStringList* appliedItems = nullptr,
                                            QStringList* skippedItems = nullptr,
                                            bool writeConnectedAxisEquiv = true);
    // 从 UI 应用雷赛轴当量。
    bool applyLeadshineAxisEquivFromUi();
    // 将软件限位同步到硬件接口。
    bool syncMotorSoftwareLimitsToHardware(QStringList* appliedItems = nullptr,
                                           QStringList* skippedItems = nullptr);
    // 返回指定轴配置的雷赛当量。
    double configuredLeadshineAxisEquivForAxis(int axisIndex) const;
    // 简单阻塞延时，供 UI 主线程流程等待硬件状态。
    void delay(unsigned int msec);
    // 停止并释放指定 PositionSimulationModel 线程。
    void stopPoseThread(PositionSimulationModel*& thread);
    // 构造位置仿真轨迹签名，用于确认完成信号属于当前路径/分段。
    QString buildPositionSimulationPathSignature(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            const QString& pathName,
            int segmentIndex = 0,
            int segmentCount = 1) const;
    // 启动仿真并等待指定路径签名的完成信号，支持多段签名全部收齐后返回。
    bool startAndWaitForPositionSimulationPaths(
            PositionSimulationModel& simulation,
            const QStringList& expectedPathSignatures,
            const QString& actionName);
    // 停止动捕线程。
    void stopMotiveThread();
    // 停止力控线程。
    void stopControlThread();
    // 停止并释放末端遥控独立输入监督线程。
    void stopEndpointRemoteInputSupervisorThread();
    // 停止位置仿真线程。
    void stopPositionThread();
    // 启动外部控制盒串口监控。
    void startMonitor();
    // 刷新控制盒串口列表。
    void refreshMonitorComPortList(const QString& preferredPort = QString());
    // 停止外部控制盒串口监控。
    void stopMonitor();
    // 启动安全监控线程。
    void startSafetyMonitor();
    // 停止安全监控线程。
    void stopSafetyMonitor();
    // 初始化 UDP 桥接 worker。
    void initializeUdpBridge();
    // 停止 UDP 桥接 worker。
    void stopUdpBridge();
    // 实时模式下启动 UDP 收发。
    void startUdpForRealtimeMode();
    // 实时模式下停止 UDP 收发。
    void stopUdpForRealtimeMode();
    // 更新 UDP 状态上报载荷。
    void updateUdpStatusPayload(bool forceUpdate = true);
    // 原子切换 UDP 状态更新策略，避免由启用/V9等布尔组合推断调度行为。
    void setUdpRuntimeProfile(UdpRuntimeProfile profile);
    // 构建 UDP 状态摘要文本。
    QString buildUdpStatusSummary() const;
    // 返回 UDP 远端 IP。
    QString udpRealtimeTargetIp() const;
    // 返回配置的传感器采样频率。
    int configuredSensorSampleFrequencyHz() const;
    // 返回配置的力传感器低通截止频率。
    double configuredForceSensorLowPassCutoffHz() const;
    // 判断当前软件估计末端是否处于 V9/UDP 可接收中位。
    bool udpPlatformAtReceiveHomePose() const;
    // 清除 UDP 轨迹后置回中状态。
    void clearUdpReturnHomeState();
    // 记录一段 UDP 轨迹完成后需要执行的内部回中动作。
    bool armUdpReturnHomeAfterTrajectory(const QVector<QVector<double>>& trajectoryPoints);
    // 启动从当前位姿到 UDP 中位的内部直线回中轨迹。
    bool startUdpReturnHomeTrajectory();
    // 处理 comm_platform 二进制平台位姿命令。
    void handleUdpPlatformCommand(const UdpPlatformCommand& command);
    // 清空 comm_platform 二进制平台流式轨迹缓存。
    void resetUdpPlatformTrajectoryCapture();
    // 将当前 comm_platform 二进制平台流式轨迹缓存收束为外部轨迹。
    void finalizeUdpPlatformTrajectoryCapture();
    // 自动导出本次 V9 comm_platform 接收轨迹 CSV，便于核对时间、位移和角位移。
    bool exportUdpPlatformTrajectoryCaptureCsv(const QVector<QVector<double>>& trajectoryPoints,
                                               const QVector<qint64>& receiveTimeMs,
                                               double expectedPeriodSec,
                                               QString* outputPath = nullptr,
                                               QString* errorMessage = nullptr,
                                               const QVector<int>& packetHasSeq = QVector<int>(),
                                               const QVector<quint32>& packetSeq = QVector<quint32>(),
                                               const QVector<quint32>& packetTimestampMs = QVector<quint32>(),
                                               const QString& fileTag = QString());
    // 将完成接收的 UDP 轨迹点转换为程序控制外部轨迹，并复用外部轨迹安全校验、仿真和下发流程。
    bool commitUdpTrajectoryPoints(const QVector<QVector<double>>& rawPoints,
                                   int expectedEndNum,
                                   const QString& sourceLabel);
    // 处理 UDP 实时位姿命令。
    void handleUdpPoseCommand(const UdpPoseCommand& command);
    // 处理 UDP 轨迹分片命令。
    void handleUdpTrajectoryChunk(const UdpTrajectoryChunk& chunk);
    // 初始化安全监控 UI 控件。
    void initializeSafetyUiControls();
    // 初始化故障注入 UI 控件。
    void initializeFaultInjectionUiControls();
    // 节流刷新运行模式 UI 状态。
    void refreshRunModeUiStateThrottled();
    // 刷新安全监控 UI。
    void refreshSafetyUiState();
    // 刷新故障注入控件可用状态。
    void refreshFaultInjectionUiControls();
    // 根据机器人状态快照刷新安全 UI。
    void refreshSafetyUiStateForState(const RobotStateSnapshot& robotState);
    // 更新安全监控主线程心跳。
    void refreshSafetyMonitorHeartbeat(qint64 timestampMs = -1);
    // 估算 PVT 上传期间安全快照超时时间。
    int estimatePvtUploadSnapshotTimeoutMs(int pointCount, int axisCount) const;
    // 判断安全故障是否属于电机位置超限。
    bool isMotorPositionLimitRecoveryFaultCode(int code) const;
    // 构建当前超限恢复轴掩码。
    std::vector<bool> motorPositionLimitRecoveryAxisMask(int axisCount) const;
    // 根据当前位置刷新电机位置超限恢复状态。
    bool refreshMotorPositionLimitRecoveryState(const std::vector<double>& safetyRelativePos,
                                                bool announce);
    // 从硬件读取当前位置并刷新电机位置超限恢复状态。
    bool refreshMotorPositionLimitRecoveryStateFromHardware(bool announce);
    // 将恢复轴列表格式化为界面/日志文本。
    QString motorPositionLimitRecoveryAxisText(const std::vector<bool>& axes) const;
    // 开始硬件独占命令期间的快照超时保护。
    void beginHardwareExclusiveSnapshotTimeout(int timeoutMs);
    // 结束硬件独占命令快照超时保护。
    void endHardwareExclusiveSnapshotTimeout();
    // 将当前配置同步到 SafetyMonitor。
    void updateSafetyMonitorConfig();
    // 按脏标记和降频策略同步 SafetyMonitor 配置。
    bool syncSafetyMonitorConfig(bool forceApply = false,
                                 const ControlWorker::Config* sharedControlConfig = nullptr);
    // 清除安全故障锁存。
    void clearSafetyFaultLatch(bool announce = false,
                               bool allowControlBoxButtonLatchClear = false);
    // 运动前检查安全联锁是否允许动作。
    bool ensureSafetyReadyForMotion(const QString& actionName);
    // 处理 SafetyMonitor 上报的故障。
    void handleSafetyFault(int level, int code, const QString& summary, const QString& detail);
    // 处理工作空间边缘条件解除，清除非锁存预警显示。
    void handleSafetyWarningCleared(int code);
    // 清除当前非锁存预警状态。
    void clearSafetyWarningState(bool announce = false);
    // 返回对外显示的电机名称，逻辑轴按 1-based 绳索/直线模组编号描述。
    QString motorAxisDisplayName(int axisIndex) const;
    // 将对外文本中的 0-based 逻辑轴编号规范化为电机显示名称。
    QString normalizeMotorAxisDisplayText(const QString& text) const;
    // 返回故障信息中使用的电机显示名称，按 1-based 绳索/直线模组编号描述。
    QString faultMotorAxisDisplayName(int axisIndex) const;
    // 将故障摘要/详情中的电机轴编号描述规范化为绳索电机/直线模组电机。
    QString normalizeSafetyFaultMotorAxisText(const QString& text) const;
    // 将故障摘要/详情规范化为真实故障语义，不暴露测试注入入口。
    QString normalizeSafetyFaultDisplayText(const QString& text) const;
    // 根据故障等级执行受控停机/安全停机/急停。
    bool applySafetyStopAction(int level, const QString& summary, const QString& detail);
    // 从 UI 重置安全状态。
    void resetSafetyStateFromUi();
    // 触发软件急停。
    void triggerSoftwareEmergencyStop();
    // 响应控制盒软件急停。
    void triggerControlBoxSoftwareEmergencyStop();
    // 注入断绳故障。
    void triggerInjectedCableBreak();
    // 注入张力采集异常（超时或读数非数值）。
    void triggerInjectedTensionAcquisitionFault();
    // 注入主线程心跳异常。
    void triggerInjectedMainThreadHeartbeatFault();
    // 注入非锁存工作空间边缘预警。
    void triggerInjectedWorkspaceWarning();
    // 清除物理断绳模拟预备状态。
    void clearCableBreakPhysicalSimulationState(bool notifySafetyMonitor = true);
    // 返回物理断绳模拟目标张力通道对应的建模电机轴。
    int cableBreakPhysicalSimulationTargetAxisIndex() const;
    // 根据当前 PVT 时间轴计算物理断绳模拟的松绳与张力置零时刻。
    bool computeCableBreakPhysicalSimulationTiming(
            const std::vector<double>& timeStamp,
            double& holdTimeSec,
            double& forceZeroTimeSec,
            QString* errorMessage = nullptr) const;
    // 在 PVT 表中改写物理断绳模拟目标绳索：中点后反向松绳并保持静止。
    bool applyCableBreakPhysicalSimulationPvtHold(
            const std::vector<int>& controlledMotorIndex,
            const std::vector<double>& timeStamp,
            std::vector<std::vector<double>>& motorPosTraj,
            std::vector<std::vector<double>>& motorVelTraj,
            QString* statusMessage,
            QString* errorMessage) const;
    // 注入电机故障。
    void triggerInjectedMotorFault();
    // 注入 PLC/控制盒通信故障。
    void triggerInjectedPlcCommunicationFault();
    // 从 UI 构建工作空间边界。
    bool buildSafetyWorkspaceBounds(WorkspaceBounds& bounds, QString* errorMessage = nullptr) const;
    // 读取当前实际末端位姿。
    bool currentActualEndPose(std::vector<double>& pose, int maxAgeMs = 500) const;
    // 校验单个位姿是否在工作空间内。
    bool validatePoseWithinWorkspace(const std::vector<double>& pose,
                                     const WorkspaceBounds& bounds,
                                     QString& errorMessage,
                                     bool* nearBoundary = nullptr,
                                     double* overflowAmount = nullptr) const;
    // 校验整条规划轨迹是否在工作空间内。
    bool validateTrajectoryWithinWorkspace(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            QString& errorMessage) const;
    // 从轨迹中提取末端终点位姿。
    bool extractTrajectoryEndpointPose(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            std::vector<double>& pose) const;
    // 从轨迹中提取末端起点位姿。
    bool extractTrajectoryStartPose(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            std::vector<double>& pose) const;
    // 将轨迹起点位姿写回 UI。
    void applyTrajectoryStartPoseToUi(const std::vector<double>& pose);
    // 初始化运行模式控件。
    void initializeRunModeControls();
    // 初始化轨迹模式下拉框。
    void initializeTrajectoryModeBoxes();
    // 从 UI 读取当前运行模式。
    RunMode selectedRunModeFromUi() const;
    // 返回运行模式显示文本。
    QString runModeDisplayName(RunMode mode) const;
    // 返回 UDP 本地监听端口。
    int udpRealtimeListenPort() const;
    // 返回 UDP 远端端口。
    int udpRealtimeTargetPort() const;
    // 返回是否使用 V9-only 平台反馈包回传模式。
    bool udpV9PlatformFeedbackEnabled() const;
    // 返回 V9 轨迹接收完成后是否启用 PCHIP 平滑/重建。
    bool udpV9PchipSmoothingEnabled() const;
    // V9 UDP 无硬件仿真测试：未启动/未连接雷赛时仅模拟 V9 state 和接收仿真链路。
    bool udpV9NoHardwareSimulationTestModeEnabled() const;
    // 将本项目三态映射为 V9 comm_platform 的 state。
    quint32 udpPlatformFeedbackStateForRobotState(const RobotStateSnapshot& robotState) const;
    // 返回 V9 platform state 的中文含义。
    QString udpPlatformStateDisplayText(quint32 state) const;
    // 判断当前是否允许接收 V9 连续位姿点。
    bool udpPlatformCanReceiveTrajectoryPoints() const;
    // 同步 UDP 端口控件显示。
    void syncUdpTargetPortWidgets(int port);
    // 应用 UDP 运行时配置变化。
    void applyUdpRuntimeConfigChange();
    // 刷新 UDP 最近数据包摘要 UI。
    void refreshUdpPacketSummaryUi();
    // 开始记录程序控制 PVT 规划耗时。
    void beginProgramControlPvtPlanningTiming();
    // 结束 PVT 规划耗时记录并显示点数/轴数。
    void finishProgramControlPvtPlanningTiming(int pvtPointCount,
                                               int pvtAxisCount,
                                               int sourceStartPointIndex = 0);
    // 取消本次 PVT 规划耗时记录。
    void cancelProgramControlPvtPlanningTiming(const QString& reason);
    // 设置 PVT 规划耗时标签文本。
    void setProgramControlPvtPlanningTimingLabel(const QString& text,
                                                 const QString& tooltip = QString());
    // 切换运行模式并刷新相关 UI。
    void setRunMode(RunMode mode, bool announce = true);
    // 立即刷新运行模式 UI。
    void refreshRunModeUiState();
    // 判断实时模式是否选择手动位姿输入。
    bool isRealtimeManualPoseInputSelected() const;
    // 判断当前轨迹模式是否来自外部文件。
    bool isExternalPoseTrajectoryModeSelected() const;
    // 判断当前运行应使用轨迹文件输入。
    bool shouldUseTrajectoryFileInput() const;
    // 根据轨迹来源刷新位姿输入 UI。
    void updatePoseInputUiState();
    // 清除轨迹文件选择状态。
    void clearTrajectoryFileSelection(bool clearLoadInfo = true);
    // 刷新轨迹文件信息 UI。
    void updateTrajectoryFileInfoUi();
    // 更新轨迹文件加载结果缓存和 UI。
    void updateTrajectoryFileLoadInfo(const QString& source,
                                      const QString& path,
                                      int endNum,
                                      int pointNum,
                                      int segmentPointCount,
                                      int segmentCount,
                                      double duration,
                                      const std::vector<double>& timeStamp,
                                      bool success,
                                      const QString& message);
    // 重置轨迹文件程序控制批处理状态。
    void resetTrajectoryFileProgramBatchState();
    // 判断是否还有轨迹文件分段待执行。
    bool hasPendingTrajectoryFileProgramSegment() const;
    // 启动下一段轨迹文件程序控制分段。
    bool startNextTrajectoryFileProgramSegment();
    // 安排下一段轨迹文件分段继续执行。
    void scheduleNextTrajectoryFileProgramSegment();
    // 实时模式下用当前位姿同步起点。
    void syncRealtimeStartPoseFromCurrentPose(bool logResult);
    // 启动当前运行模式对应的主动作。
    bool startActiveRunMode(bool fromControlBox = false);
    // 处理主操作按钮。
    bool handleRunModePrimaryAction();
    // 处理次操作按钮。
    bool handleRunModeSecondaryAction();
    // 轨迹模式变化时刷新参数区。
    void onPosTrajModeChanged(const QString& mode);
    // 判断是否请求圆轨迹连续运动。
    bool isCircularContinuousMotionRequested() const;
    // 捕获圆轨迹连续运动参数。
    void captureCircularContinuousMotionParams();
    // 将连续圆轨迹参数写回 UI。
    void applyCircularContinuousMotionParamsToUi();
    // 安排圆轨迹连续重启。
    void scheduleCircularContinuousMotionRestart();
    // 取消圆轨迹连续重启。
    void cancelCircularContinuousMotionRestart();
    // 从 UI 读取位姿轨迹采样周期。
    double poseTrajectoryStepTimeSecFromUi(double fallbackSec = 0.01) const;
    // 开关力控选择 UI。
    void setForceControlSelectionEnabled(bool enabled);
    // 判断是否选择了任何力控轴。
    bool hasForceControlAxisSelected() const;
    // 判断单绳力控调试模式是否激活。
    bool isSingleCableForceDebugModeActive() const;
    // 强制单绳力控只选择一根绳。
    void enforceSingleCableForceSelection(QRadioButton* preferredButton = nullptr);
    // 同步单绳力控调试预紧力。
    void syncSingleCableForceDebugPretensionForce();
    // 刷新单绳力控调试 UI 状态。
    void updateSingleCableForceDebugUiState();
    // 处理控制盒完整状态帧。
    void handleControlBoxStatus(int motionControl,
                                int speedZero,
                                int zeroCalib,
                                int softwareEmergencyStop);
    // 处理控制盒监控串口错误。
    void handleControlBoxSerialCommunicationFault(const QString& detail);
    // 运行态下检查控制盒有效状态帧是否超时。
    void checkControlBoxCommunicationFault();
    // 判断控制盒/PLC监控串口是否已配置。
    bool isControlBoxMonitorComConfigured() const;
    // 处理控制盒运动控制状态。
    void handleControlBoxMotionControl(int state, int speedZeroState = -1);
    // 处理控制盒速度清零状态。
    void handleControlBoxSpeedZero(int state,
                                   int previousState,
                                   bool allowCommandAction = true);
    // 处理控制盒零点标定状态。
    void handleControlBoxZeroCalib(int state);
    // 处理控制盒软件急停状态。
    void handleControlBoxSoftwareEmergencyStop(int state, int previousState);
    // 从编码器位置差分速度更新控制盒安全按钮使用的绳速门限状态。
    void updateControlBoxCableSpeedState(const ControlWorker::Snapshot& snapshot);
    // 返回当前是否存在有效且新鲜的编码器绳速超限，并生成提示详情。
    bool controlBoxCableOverspeedActive(QString* detail = nullptr) const;
    // 运动结束后将超速期间仍未弹起的按钮转为安全锁存。
    void evaluateControlBoxOverspeedButtonSafetyLatch();
    // 返回阻止安全复位的控制盒按钮名称。
    QStringList controlBoxUnreleasedSafetyButtons() const;
    // 判断力控线程是否真的处于运行状态。
    bool isForceControlThreadActuallyRunning() const;
    // 检查控制盒停止速度限制是否允许动作。
    bool controlBoxStopSpeedLimitAllowsAction(QString* failureReason = nullptr);
    // 执行速度清零受控停机。
    bool engageSpeedZeroControlledStop(QString* failureReason = nullptr);
    // 释放速度清零受控停机，并按记录状态回点或续跑。
    bool releaseSpeedZeroControlledStop(QString* failureReason = nullptr);
    // 清除控制盒速度清零 PVT 返回状态。
    void clearControlBoxSpeedZeroPvtReturnState();
    // 读取记录轨迹指定点的位姿。
    bool poseAtRecordedTrajectoryPoint(int pointIndex, std::vector<double>& pose) const;
    // 读取原始记录轨迹起点位姿。
    bool originalRecordedTrajectoryStartPose(std::vector<double>& pose) const;
    // 读取原始记录轨迹终点位姿。
    bool originalRecordedTrajectoryEndPose(std::vector<double>& pose) const;
    // 返回记录轨迹总时长。
    double recordedTrajectoryTotalDurationSec() const;
    // 返回记录轨迹采样周期。
    double recordedTrajectoryStepTimeSec() const;
    // 停机前多次采样 PVT 进度，降低停止点读取抖动。
    ControlBoxPvtProgressSample sampleControlBoxPvtProgressForStopPoint();
    // 使用停前采样结果构造按钮停止链路所需的 PVT 停止点结果。
    HardwareInterface::PvtPauseResult controlBoxPauseResultFromStableStopProgress(
            const ControlBoxPvtProgressSample& stableSample) const;
    // 控制盒 PVT 停止采用界面“停止”按钮同款停轴逻辑。
    bool stopControlBoxPvtWithMainStopLogic(const std::vector<int>& waitAxes,
                                            const QString& actionName,
                                            QString* failureReason = nullptr);
    // 控制盒速度置零/运动控制停止共用：停止当前PVT、记录停止点并启动回到停止点PVT。
    bool stopControlBoxActivePvtAndStartReturn(bool motionControlStop,
                                               QString* failureReason = nullptr);
    // 构建控制盒停止点诊断文本。
    QString controlBoxStopPointDiagnosticText() const;
    // 捕获速度清零停机后的 PVT 返回目标。
    bool captureControlBoxSpeedZeroPvtReturnTarget(
            const HardwareInterface::PvtPauseResult& pauseResult,
            const ControlBoxPvtProgressSample& stableSample,
            bool motionControlStop = false);
    // 控制盒 PVT 边界后输出异常参与轴诊断。
    QString controlBoxPvtAbnormalAxisDiagnosticText(const std::vector<int>& motorIndex);
    // 控制盒 PVT 边界后等待/恢复参与轴到操作使能，允许继续下发后续 PVT。
    bool recoverControlBoxPvtAxesAfterDecStop(const std::vector<int>& motorIndex,
                                              const QString& actionName,
                                              QString* failureReason = nullptr,
                                              const QString& phaseText = QString());
    // 控制盒自动 PVT 下发前确认参与轴仍处于使能状态。
    bool verifyControlBoxPvtAxesEnabled(const std::vector<int>& motorIndex,
                                        const QString& actionName);
    // 控制盒两段 PVT 衔接前确认上一段涉及轴已稳定完成。
    bool waitControlBoxPvtAxesDoneStable(const std::vector<int>& motorIndex,
                                         int timeoutMs,
                                         int sampleMs,
                                         int stableSamples,
                                         QString* failureReason = nullptr);
    // 控制盒自动 PVT 启动失败后的局部停机和 PVT 状态清理。
    void cleanupFailedControlBoxPvtStart(const std::vector<int>& motorIndex,
                                         const QString& actionName);
    // 启动速度清零后的回停机点 PVT。
    bool startControlBoxSpeedZeroPvtReturn();
    // 启动速度清零后的恢复轨迹。
    bool startControlBoxSpeedZeroResumeTrajectory();
    // 启动控制盒运动停止回起点轨迹。
    bool startControlBoxMotionStopHomeTrajectory();
    // 速度清零状态下停止所有电机轴。
    void stopAllMotorAxesForSpeedZero();
    // 周期检查轨迹执行完成并做收尾。
    void checkTrajectoryFinished();
    // 重置控制相关 UI 状态。
    void resetControlUiState();
    // 从 UI 构建每个末端的绳索连接点位置。
    std::vector<std::vector<std::vector<double>>> buildCableContactPointPos() const;
    // 从 UI 构建固定锚点坐标。
    std::vector<std::vector<double>> buildFixedAnchorHome() const;
    // 从 UI 构建绳索电机转换系数。
    std::vector<double> buildCableMotorCof() const;
    // 根据当前状态构建卷绕补偿配置。
    std::vector<WinchCompensation::AxisConfig> buildWinchCompensationConfig() const;
    // 根据指定编码器/Trace 参考构建卷绕补偿配置。
    std::vector<WinchCompensation::AxisConfig> buildWinchCompensationConfig(
            const std::vector<double>& currentMotorEncoderPos,
            const std::vector<qint64>& currentTraceCommandRawPulse = std::vector<qint64>()) const;
    // 构建单轴卷绕补偿配置。
    WinchCompensation::AxisConfig buildWinchCompensationConfigForAxis(int axisIndex) const;
    // 用指定参考构建单轴卷绕补偿配置。
    WinchCompensation::AxisConfig buildWinchCompensationConfigForAxis(
            int axisIndex,
            const std::vector<double>* currentMotorEncoderPos,
            const std::vector<qint64>* currentTraceCommandRawPulse = nullptr) const;
    // 从 UI 读取滑轮半径。
    double buildPulleyRadius() const;
    // 构建固定锚点位移数组。
    std::vector<double> buildFixedAnchorDis() const;
    // 将扁平锚点列表按末端拆分。
    std::vector<std::vector<std::vector<double>>> splitAnchorPositionsByEnd(const std::vector<std::vector<double>>& anchorPosTemp) const;
    // 清空规划期望绳力轨迹。
    void clearPlannedPoseForceTrajectory();
    // 为位姿轨迹逐点规划可行绳力。
    bool planPoseForceTrajectory(const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
                                 QString& errorMessage,
                                 std::vector<double>* failedPose = nullptr,
                                 int* failedStep = nullptr,
                                 double* failedTime = nullptr);
    // 将规划得到的绳力轨迹映射为传感器期望值顺序。
    std::vector<std::vector<double>> mapCableForceTrajectoryToSensorExpected(
        const std::vector<std::vector<double>>& cableForceTraj) const;
    // 判断 UI 是否请求混合位姿-力控模式。
    bool isHybridPoseForceModeRequested() const;
    // 更新混合模式按钮文本。
    void updateHybridPoseForceModeButtonText();
    // 刷新混合模式力控选择 UI。
    void refreshHybridPoseForceSelectionUi();
    // 返回混合模式当前选择数量。
    int hybridPoseForceSelectedCount() const;
    // 返回混合模式选择的传感器索引。
    std::vector<int> selectedHybridPoseForceSensorIndices() const;
    // 返回混合模式选择的力控轴索引。
    std::vector<int> selectedHybridPoseForceAxisIndices() const;
    // 重置混合模式选择。
    void resetHybridPoseForceSelection();
    // 限制混合模式选择数量并保留优先按钮。
    void enforceHybridPoseForceSelectionLimit(QRadioButton* preferredButton = nullptr);
    // 停止混合模式中的力控轴。
    void stopHybridPoseForceAxes(const std::vector<int>& axisIndices);
    // 构建混合位姿-力控运行配置。
    bool buildHybridPoseForceModeConfig(HybridPoseForceModeConfig& out,
                                        QString& errorMessage) const;
    // 将指定轨迹点的期望力应用到力控 worker。
    bool applyHybridPoseForceExpectedForceAtPoint(int pointIndex, QString* errorMessage = nullptr);
    // 随活动轨迹同步计算得到的期望力。
    void syncCalculatedExpectedForceForActiveTrajectory(bool refreshControlConfigAfterApply = false);
    // 轨迹末端保持混合力控轴当前状态。
    void holdHybridPoseForceAtTrajectoryEnd();
    // 清除混合位姿-力控状态。
    void clearHybridPoseForceModeState(bool disableForceThread);

    // 批量切换所有绳索是否参与力控。
    void allUseFC(bool isChecked);// 鍏ㄩ儴缁崇储閮戒娇鐢ㄥ姏鎺?
    // 确认绳索预紧完成并记录回零参考。
    bool setCableHome();// 鐢垫満棰勭揣瀹屾垚
    // 根据期望力计算允许误差。
    double pretensionForceAllowedError(double expectedForce) const;
    // 判断当前绳力是否满足回零确认容差。
    bool isCableForceWithinHomeConfirmTolerance(QString* errorMessage = nullptr) const;
    // 判断零位预紧力是否在容差内。
    bool isZeroPosePretensionForceWithinTolerance(QString* errorMessage = nullptr) const;
    // 判断恢复运行位姿预紧力是否在容差内。
    bool isRestoredRuntimePretensionForceWithinTolerance(QString* errorMessage = nullptr) const;
    // 判断指定运动是否可跳过绳索回零检查。
    bool shouldSkipCableHomeCheckForMotion(const QString& motionName) const;
    // 清除连续轨迹选项。
    void resetContinuousTrajectoryOptions();
    // 运动前检查绳索回零/预紧状态。
    bool ensureCableHomeReadyForMotion(const QString& motionName);
    // 刷新回零确认按钮可用状态。
    void updateCableHomeConfirmEnabled();
    // 重置力控 PID 参数到 UI 当前值。
    void resetFCPIDPara();
    // 请求立即刷新力控配置/期望力。
    void requestImmediateForceControlUpdate();
    OneDimKalmanHandler fcOneDimKalmanHandler;

    std::vector<double> endCurRotHome;
    std::vector<double> motorCurPos, motorCurPosRaw, motorCurAbsPos, motorPosDisplayZero, cableHomePos, forceSensorCurHome;
    // 主线程持有的当前命令零位镜像，供周期 UI 刷新使用，避免同步等待硬件线程。
    std::vector<double> motorHomeUiCache;
    std::vector<QLabel*> curMotorPosTextVec;
    std::vector<LinearModuleHeightReference> linearModuleHeightReferences;
    std::vector<int> linearModuleUpperAnchorAxisIndices;
    std::vector<double> linearModuleUpperAnchorBaseZMm;

    std::vector<QRadioButton*> useFCBtnVec;
    std::vector<QLabel*> mainCableForceLimitSignalVec;
    std::vector<QDoubleSpinBox*> mainForceSensorData,mainForceSensorExp,mainMotorTorqueData;
    std::vector<double> mainForceSensorDataVal,mainForceSensorExpVal;
    QCheckBox* mainAllCableForceDragModeSwitch = nullptr;

    bool forceControlSelectionUiEnabled = true;
    bool suppressSingleCableForceSelectionUpdate = false;
    bool singleCableForceDebugPretensionSaved = false;
    double singleCableForceDebugSavedPretensionForce = 0.0;

    // 返回普通电机轴显示名称。
    QString motorAxisText() const;
    // 返回直线模组轴显示名称。
    QString linearMotorAxisText() const;
    // 判断指定轴是否为电机轴。
    bool isMotorAxis(int axisIndex) const;
    // 判断指定轴是否为直线模组轴。
    bool isLinearMotorAxis(int axisIndex) const;
    // 判断指定轴是否参与绳索/平台建模。
    bool isModeledMotorAxis(int axisIndex) const;
    // 判断指定绳索电机轴是否启用卷绕补偿。
    bool isWinchCompensatedCableMotorAxis(int axisIndex) const;

    // 杞村弬鏁版帶浠跺簭鍒?
    // 收集 UI 中按轴排列的控件指针，便于后续批量读写参数。
    void setUIVec();
    std::vector<QComboBox*> axisTypeVec;
    std::vector<QCheckBox*> axisIsPos2NegVec;
    std::vector<QSpinBox*> axisEndVec,axisSensorIndexVec,axisMotorHardwareAxisVec,axisMotorSlaveIdVec;

    std::vector<QDoubleSpinBox*> axisMotorCofVec,axisSensorCofVec,axisMotorZeroVec,axisSensorZeroVec,// 鐢垫満銆佷紶鎰熷櫒鐨勮浆鎹㈢郴鏁板拰闆剁偣鍊?

    axisCableStartPosXVec,axisCableStartPosYVec,axisCableStartPosZVec,axisCableEndPosXVec,axisCableEndPosYVec,axisCableEndPosZVec,// 缁崇储鍑虹怀鐐瑰拰杩炴帴鐐瑰潗鏍?

    axisCableZeroLenVec,axisMotorMaxVec,axisMotorMinVec,axisMotorVelMaxVec,axisForceMaxVec,// 鍒濆缁抽暱鍜岃繍鍔ㄩ檺鍒?

    axisImqVec,axisFvqVec,axisFcqVec,// 鐢垫満銆佺粸鐩樺姩鍔涘鍙傛暟

    endMassVec,endIxxVec,endIyyVec,endIzzVec,endIxyVec,endIyzVec,endIxzVec,// 鍔ㄥ钩鍙拌川閲忎笌local绯荤殑鎯€х煩闃?

    endImpMdXVec,endImpMdYVec,endImpMdZVec,endImpMdRxVec,endImpMdRyVec,endImpMdRzVec,// 闃绘姉鍙傛暟M,D,K
    endImpDdXVec,endImpDdYVec,endImpDdZVec,endImpDdRxVec,endImpDdRyVec,endImpDdRzVec,
    endImpKdXVec,endImpKdYVec,endImpKdZVec,endImpKdRxVec,endImpKdRyVec,endImpKdRzVec;

    int axisCableNum = 0;
    int axisForceSensorNum = 0;
    QStringList messageHistoryEntries;
    QStringList pendingUiEventLogLines;
    // Lite 普通调试事件只在主线程入队，由现有 250 ms 日志定时器批量追加。
    // 不得在按钮回调中复用完整故障快照或同步重写结构化故障文件。
    QStringList pendingLiteCommissioningEventLogLines;
    // 启动自检属于运行审计而非 SafetyMonitor 故障；无论通过或失败都只
    // 生成轻量追加记录，避免整机已使能后在主线程重写完整故障历史。
    QStringList pendingStartupSelfCheckEventLogLines;
    QTimer* uiEventLogFlushTimer = nullptr;
    std::vector<int> eachEndCableNum;// 璁板綍姣忎釜鏈鎷ユ湁鐨勭怀绱㈡暟

    bool testDyn = false;
};
#endif // MAINWINDOW_H
