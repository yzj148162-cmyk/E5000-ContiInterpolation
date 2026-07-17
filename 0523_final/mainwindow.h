#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
#include "forcesimulationmodel.h"
#include "positionsimulationmodel.h"
#include "motivelocalhandlerthread.h"
#include "controlworker.h"
#include "pvtexecutionworker.h"
#include "simulationworker.h"
#include "hardwareinterface.h"
#include "udppackettypes.h"

#include<QMetaType>// 淇″彿妲藉嚱鏁版湰涓嶆敮鎸乻td::vector,std::string绛夛紝鍥犳闇€瑕佹敞鍐岋紙鏋勯€犲嚱鏁癕ainWindow涔熼渶瑕佹坊鍔犱竴閮ㄥ垎锛?
Q_DECLARE_METATYPE(std::vector<double>)
Q_DECLARE_METATYPE(std::vector<std::vector<double>>)
Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(QVector<QVector3D>)

# pragma execution_character_set("utf-8")

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGridLayout;
class QGroupBox;
class QLabel;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QTextStream;
class QTimer;
class MotorTorqueTestWorker;
class MonitorThread;
class SafetyMonitor;
class SingleMotorTrajectoryWorker;
class UdpCommWorker;

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtDataVisualization;// 浣跨敤DataVisualization缁樺埗涓夌淮妯″瀷(QT6.*鍙敞閲婃湰娈?
#endif

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    enum class PositionMotionState {
        Idle,
        Simulating,
        ExecutingPvt,
        PausedPvt
    };

    enum class ForceMotionState {
        Idle,
        DirectForceControl,
        GravityCompensation
    };

    enum class RunMode {
        ProgramControl,
        RealtimeTrajectory,
        SimulationCalculation
    };

    enum class CableHomeState {
        Unconfirmed,
        WaitingForceStable,
        Confirmed
    };

    enum class CalibrationWorkflowStage {
        Idle,
        MechanicalHoming,
        AwaitingPretensionStart,
        PretensionBalancing,
        CollectingSensorZero,
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
        bool gcRunning = false;
        bool forceControlThreadChecked = false;
        bool forceControlThreadRunning = false;
        bool pvtTrajectoryAvailable = false;
        bool pvtMotionRunning = false;
        bool pvtMotionPaused = false;
        bool pvtMotionFinished = true;
        bool singleMotorTrajectoryActive = false;
        bool anyMotionRunning = false;
        RunMode runMode = RunMode::ProgramControl;
        PositionMotionState positionMotionState = PositionMotionState::Idle;
        ForceMotionState forceMotionState = ForceMotionState::Idle;
    };

    RobotStateSnapshot currentRobotState(bool queryHardware = true) const;

private:
    enum class MotorFeedbackDisplayUnit {
        Degree,
        Revolution
    };
    Ui::MainWindow *ui;
    bool initPara();

    struct RobotRuntimeState {
        bool systemRunning = false;
        bool gcRunning = false;
        bool posModeRunning = false;
        bool pvtCommandActive = false;
        bool hybridPoseForceModeActive = false;
        bool singleCableForceDebugMode = false;
        bool motorTorqueDebugActive = false;
        bool singleMotorTrajectoryActive = false;
        bool servoHoldActive = false;
        RunMode runMode = RunMode::ProgramControl;
        CableHomeState cableHomeState = CableHomeState::Unconfirmed;
        bool autoExecutePoseAfterSimulation = false;
        bool safetyFaultLatched = false;
        int safetyStopLevel = 0;
        int safetyFaultCode = 0;
        QString safetyFaultSummary;
        QString safetyFaultDetail;

        int controlBoxMotionControlState = -1;
        int controlBoxSpeedZeroState = -1;
        int controlBoxZeroCalibState = 0;
        bool controlBoxStartRequiresStop = true;
        bool controlBoxStartInterlockReported = false;
        bool controlBoxPausedPvtMotion = false;
        bool controlBoxPausedForceControl = false;
        bool controlBoxPausedGC = false;
    };

    struct HybridPoseForceModeConfig {
        bool enabled = false;
        std::vector<int> forceAxisIndex;
        std::vector<int> forceSensorIndex;
        std::vector<double> frozenExpectedForce;
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

    struct RuntimeDiagnosticsSample {
        qint64 capturedAtMs = 0;
        ControlWorker::TimingDiagnostics controlTiming;
        HardwareInterface::DiagnosticsSnapshot hardwareTiming;
    };

    struct RuntimeDiagnosticsSummary {
        qint64 windowDurationMs = 0;
        quint64 sensorIntervalCount = 0;
        double sensorAverageHz = 0.0;
        double sensorLatestHz = 0.0;
        quint64 communicationIntervalCount = 0;
        double communicationAverageHz = 0.0;
        double communicationLatestHz = 0.0;
        quint64 motorCommandIntervalCount = 0;
        double motorCommandAveragePeriodMs = 0.0;
        double motorCommandLatestPeriodMs = 0.0;
        quint64 controlLoopIntervalCount = 0;
        double controlLoopAveragePeriodMs = 0.0;
        double controlLoopLatestPeriodMs = 0.0;
    };

    struct SessionRecordingState {
        bool active = false;
        qint64 startedAtMs = 0;
        qint64 endedAtMs = 0;
        QJsonArray samples;
        QJsonObject parameterSnapshot;
        QString lastExportPath;
    };

    RobotRuntimeState runtimeState;
    SessionRecordingState sessionRecordingState;

    void runSwitch();

    // 涓荤嚎绋嬨€備负浠€涔堜娇鐢ㄧ嚎绋嬭€屼笉鐢╭t鐨凲Timer锛堝弬鑰冨姩鎬佹洸绾跨粯鍒剁殑渚嬬▼锛夛紵
    // 鍘熷洜鈶狅細澶氱嚎绋嬪垎鎷呰繍绠楀帇鍔?
    // 鍘熷洜鈶★細QTimer浣跨敤QT鑷甫璁℃椂锛岃€岃嚜宸卞啓鐨勭嚎绋嬪彲浠ョ敤鑷繁鎯崇敤鐨勮鏃舵柟寮忥紝姣斿鍩轰簬绯荤粺鏃堕棿璁℃椂銆傝繖鏍风殑璇濆彲浠ユ湁鏁堥伩鍏嶇嚎绋嬫椂闂存埑涓嶅悓姝ョ殑鎯呭喌
    // 姣斿璁㏑OS绾跨▼涔熺敤绯荤粺鏃堕棿浣滀负鎺у埗鍛ㄦ湡鐨勫熀鍑嗭紝杩欐牱QT鍜孯OS鐢变簬閮界敤绯荤粺鏃堕棿锛屽洜姝ゅ彲鍚屾鏃堕棿鎴?
    // 鍘熷洜鈶細绾跨▼鏄被锛屽熀浜庨潰鍚戝璞＄殑鎬濊矾锛屽啓璧锋潵鏇磋嚜鐢憋紝涓斾笉鐢ㄥ儚QTimer鐢ㄥぇ閲廲onnect鏉ヨ繘琛屾帶鍒跺懆鏈熶笌鎺у埗鎿嶄綔鐨勬槧灏?
    ForceSimulationModel* forceSimulationModel = nullptr;
    ForceSimulationModel* forwardKinematicsHelper = nullptr;
    SimulationWorker* simulationWorker = nullptr;

    QThread* hardwareThread = nullptr;
    ControlWorker* controlWorker = nullptr;
    QThread *ccThread = nullptr;
    MotorTorqueTestWorker* motorTorqueTestWorker = nullptr;
    QThread* motorTorqueTestThread = nullptr;
    SingleMotorTrajectoryWorker* singleMotorTrajectoryWorker = nullptr;
    QThread* singleMotorTrajectoryThread = nullptr;
    QTimer* controlSnapshotTimer = nullptr;
    QTimer* safetyHeartbeatTimer = nullptr;
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
    QWidget* dataVizTab = nullptr;
    QCustomPlot* motorControlPlotHost = nullptr;
    QCustomPlot* endEffectorPosPlotHost = nullptr;
    QCustomPlot* endEffectorVelPlotHost = nullptr;
    QCustomPlot* endEffectorAccPlotHost = nullptr;
    QCustomPlot* cableTensionPlotHost = nullptr;
    QCustomPlot* cableSpeedPlotHost = nullptr;
    QCustomPlot* cableLengthPlotHost = nullptr;
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
    QPushButton* motorTorqueServoSetupButton = nullptr;
    QPushButton* motorTorqueEnableButton = nullptr;
    QPushButton* motorTorqueStartButton = nullptr;
    QPushButton* motorTorqueStopButton = nullptr;
    QLabel* motorTorqueServoSetupStatusLabel = nullptr;
    QLabel* motorTorqueStatusLabel = nullptr;
    bool suppressMotorTorqueLimitUiSync = false;
    QWidget* singleMotorTrajectoryTab = nullptr;
    QComboBox* singleMotorTrajectoryAxisCombo = nullptr;
    QComboBox* singleMotorTrajectoryTypeCombo = nullptr;
    QCheckBox* singleMotorTrajectoryUseCurrentStartCheckBox = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryStartSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryEndSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryCenterSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryAmplitudeSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryCyclesSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryPhaseSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryDurationSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryPlanStepSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryVelocityLimitSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryCurrentPosSpin = nullptr;
    QDoubleSpinBox* singleMotorTrajectoryMaxErrorSpin = nullptr;
    QPushButton* singleMotorTrajectoryEnableButton = nullptr;
    QPushButton* singleMotorTrajectoryStartButton = nullptr;
    QPushButton* singleMotorTrajectoryStopButton = nullptr;
    QPushButton* singleMotorTrajectoryExportButton = nullptr;
    QPushButton* singleMotorTrajectoryClearButton = nullptr;
    QLabel* singleMotorTrajectoryStatusLabel = nullptr;
    QCustomPlot* singleMotorTrajectoryTrackPlotHost = nullptr;
    QCustomPlot* singleMotorTrajectoryErrorPlotHost = nullptr;
    QVector<double> singleMotorTrajectorySampleTime;
    QVector<double> singleMotorTrajectoryExpected;
    QVector<double> singleMotorTrajectoryCommand;
    QVector<double> singleMotorTrajectoryActual;
    QVector<double> singleMotorTrajectoryError;
    QVector<double> singleMotorTrajectoryDriveError;
    double singleMotorTrajectoryDurationSec = 0.0;
    bool singleMotorTrajectoryRunning = false;

    double curTime = 0.0;// 褰撳墠鏃堕棿(s)

    qint64 lastRunModeUiRefreshMs = -1;
    qint64 lastConnectionStatusUiRefreshMs = -1;
    qint64 lastSessionRecordingUiRefreshMs = -1;
    UdpCommStats udpCommStats;
    UdpStatusPayload udpStatusPayload;
    bool udpRealtimeBridgeActive = false;
    HybridPoseForceModeConfig activeHybridPoseForceConfig;
    std::vector<std::vector<double>> activeHybridExpectedForceTraj;
    int activeHybridExpectedForcePointIndex = -1;
    UdpPoseCommand latestUdpPoseCommand;
    UdpTrajectoryChunk latestUdpTrajectoryChunk;
    QVector<QVector<double>> udpTrajectoryPointBuffer;
    quint64 lastUdpPoseSeq = 0;
    qint64 lastUdpPoseTimestampMs = 0;
    quint64 lastUdpTrajectorySeq = 0;
    qint64 lastUdpTrajectoryTimestampMs = 0;
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
    QPushButton* calibrationSaveButton = nullptr;
    QPushButton* calibrationLoadButton = nullptr;
    QPushButton* calibrationRestoreButton = nullptr;
    QString lastCalibrationRecordPath;
    QDateTime lastCalibrationRecordTime;
    QString lastCalibrationRecordSummary;
    CalibrationWorkflowStage calibrationWorkflowStage = CalibrationWorkflowStage::Idle;
    QElapsedTimer visualizationElapsedTimer;
    bool visualizationTimerStarted = false;
    double lastVisualizationPoseTime = -1.0;
    std::vector<double> lastVisualizationPose;
    std::vector<double> lastVisualizationVelocity;
    std::vector<std::vector<double>> lastForwardKinematicsPose;
    qint64 lastForwardKinematicsPoseTimestampMs = -1;

    bool outputInfoSendTrigger();
    bool useCam = true;
    void camLock(bool lockCam);
    void resetMotiveFit();
    bool hasValidMotivePose() const;
    bool hasValidEstimatedEndPose(int maxAgeMs = 500) const;
    bool hasRecentMotivePose(int maxAgeMs = 500) const;
    bool shouldUseForwardKinematicsFallback(int maxAgeMs = 500) const;
    bool currentForwardKinematicsEndPose(std::vector<double>& pose, int maxAgeMs = 1000) const;
    bool currentEstimatedEndPose(std::vector<double>& pose, int maxAgeMs = 500) const;
    void cacheForwardKinematicsPose(const std::vector<std::vector<double>>& platformPose);
    void refreshForwardKinematicsPoseDisplay();
    std::vector<double> configuredCableHomePose() const;
    std::vector<std::vector<double>> configuredCableHomePlatformPose() const;
    bool isCurrentMotorSnapshotAtCableHome(double toleranceMm = 0.5) const;
    bool motiveFitConfirmed = false;

    void updateControlWorkerConfig();
    void applyControlSnapshot();
    void applyMotorPositionSnapshot(const std::vector<double>& absPos,
                                    const std::vector<double>& relRawPos = {});
    void processNoCamCableLengthFromMotorSnapshot();
    bool buildCurrentCableDisplacements(std::vector<std::vector<double>>& cableDisplacement) const;
    bool buildCurrentCableLengths(std::vector<std::vector<double>>& cableLen) const;
    bool buildCurrentCableLengthSnapshot(std::vector<std::vector<double>>& cableLen) const;
    QVector<double> flattenCableLengthsByAxisOrder(const std::vector<std::vector<double>>& cableLen) const;
    QString formatCableLengthVector(const QVector<double>& cableLen) const;
    void logTrajectoryEndCableLengthDiagnostics();
    bool ensurePoseKinematicsHelpersReady(bool forceRebuild = false);
    void updateForwardKinematicsPoseFromMotorSnapshot();
    bool refreshForwardKinematicsPoseFromCurrentMotorPosition();
    ControlWorker::Config buildControlWorkerConfig() const;
    std::vector<double> buildExpectedForceFromUi() const;
    std::vector<double> mapGcCableForceToSensorExpected(const std::vector<double>& gcCableForce) const;
    void setupDataVisualizationTab();
    void setupMotorTorqueTestTab();
    void startMotorTorqueTestThread();
    void stopMotorTorqueTestThread();
    void refreshMotorTorqueAxisOptions();
    int selectedMotorTorqueAxisIndex() const;
    double motorTorqueServoVelocityLimitRpm() const;
    bool applyMotorTorqueServoVelocityLimitFromUi();
    void syncMotorTorqueLimitUiFromAxis(int axisIndex);
    void applyMotorTorqueLimitUiToAxis(int axisIndex);
    void updateMotorTorqueWorkerConfig();
    void startMotorTorqueDebug();
    void stopMotorTorqueDebug();
    void handleMotorTorqueStatus(int axisIndex,
                                 double relativePosition,
                                 double actualTorque,
                                 double actualVelocity,
                                 bool active);
    void reinitializeDataVisualizationPlots();
    void clearVisualizationData();
    double visualizationTimeSeconds();
    void updateVisualizationFromControlSnapshot(const ControlWorker::Snapshot& snapshot);
    void updateVisualizationFromPlatformPose(const std::vector<std::vector<double>>& platformPose);
    void handleVisualizationRigidPose(const std::vector<std::vector<double>>& rigidPose);
    void updateVisualizationPoseSample(double timeSec,
                                       const std::vector<double>& pose,
                                       const std::vector<double>* velocity = nullptr,
                                       const std::vector<double>* acceleration = nullptr);
    QVector<double> mapMotorCommandForPlot(const std::vector<double>& motorCommand) const;
    QVector<double> mapCableTensionForPlot(const std::vector<double>& forceSensorValue) const;
    QVector<double> mapCableSpeedForPlot(const std::vector<double>& motorVelocity) const;
    QVector<double> buildGeometricCableLengthForPlot(const std::vector<std::vector<double>>& platformPose) const;
    double convertMotorFeedbackToCableValue(int axisIndex, double rawValue) const;
    double motorCableAngleScale(int axisIndex) const;
    double motorHardwareDirectionSign(int axisIndex) const;
    void setupSingleMotorTrajectoryTrackingTab();
    void startSingleMotorTrajectoryThread();
    void stopSingleMotorTrajectoryThread();
    void refreshSingleMotorTrajectoryAxisOptions();
    int selectedSingleMotorTrajectoryAxisIndex() const;
    void updateSingleMotorTrajectoryWorkerAxis();
    void refreshSingleMotorTrajectoryUnitUi();
    double singleMotorTrajectoryPositionToDegrees(double position) const;
    double singleMotorTrajectoryRawFollowingErrorToDegrees(int axisIndex, int rawError) const;
    void setupSingleMotorTrajectoryPlots();
    void clearSingleMotorTrajectoryPlots();
    void startSingleMotorTrajectory();
    void stopSingleMotorTrajectory(const QString& source = QStringLiteral("unspecified"));
    void exportSingleMotorTrajectoryData();
    void handleSingleMotorTrajectoryPrepared(int axisIndex,
                                             QVector<double> time,
                                             QVector<double> expectedPosition);
    void handleSingleMotorTrajectorySample(int axisIndex,
                                           double time,
                                           double expectedPosition,
                                           double actualPosition,
                                           double tracePositionError,
                                           double commandPosition,
                                           bool commandPositionValid,
                                           int driveFollowingErrorRaw,
                                           bool driveFollowingErrorValid);
    void handleSingleMotorTrajectoryFeedback(int axisIndex, double relativePosition);
    void handleSingleMotorTrajectoryStateChanged(bool active, const QString& statusText);
    int sensorMinX,sensorMaxX,sensorMinY,sensorMaxY,motorMinX,motorMaxX,motorMinY,motorMaxY;// 鏄剧ず鑼冨洿
    std::vector<std::vector<std::vector<double>>> homeCableVec;
    std::vector<std::vector<double>> homeCableLen;
    std::vector<double> dynPID_P,dynPID_I,dynPID_D,orgPID_P,orgPID_I,orgPID_D;


    std::vector<double> lastTrajEndMotorTheta;// 涓婁竴娆¤建杩圭粓鐐瑰鐢垫満杞锛岀敤浜庤繛缁建杩?
    std::vector<std::vector<double>> lastTrajEndAnchorCoor;// 涓婁竴娆¤建杩圭粓鐐瑰閿氱偣鍧愭爣锛岀敤浜庤繛缁建杩?
    std::vector<double> lastSyncedMotorSoftwareMinPos;
    std::vector<double> lastSyncedMotorSoftwareMaxPos;
    MotorFeedbackDisplayUnit lastMotorFeedbackDisplayUnit = MotorFeedbackDisplayUnit::Revolution;
    bool suppressMotorLimitUnitConversion = false;
    std::vector<std::vector<double>> plannedPoseCableForceTraj;
    std::vector<std::vector<double>> plannedPoseExpectedForceTraj;
    std::vector<double> plannedPoseForceTimeStamp;
    std::vector<int> activePosModeMotorIndex;
    std::vector<double> activePosModeStartMotorTheta;
    std::vector<double> activePosModeContinuousStartMotorTheta;
    std::vector<double> lastPlannedPoseTrajectoryEndPose;
    
    // 杞ㄨ抗瀹屾垚妫€娴嬪畾鏃跺櫒
    QTimer* trajectoryCheckTimer = nullptr;
    bool isFirstTraj = true;

    bool use3DViewer = true;
    bool init3DViewer();
    void update3DViewer(QVector<QVector3D> targetPos, QVector<QVector3D> trajPos, QVector<QVector3D> anchorPos,
                        QVector<QVector3D> cablePos);
    bool is3DViewerReadyForNextInput = false;
    QVector3D to803Frame(QVector3D qtFrame);
    void replay3DViewer();
    void clearPreReplayData();// 娓呯┖涔嬪墠璁板綍鐨勭敤浜庨噸鎾豢鐪熺殑涓夌淮鏁版嵁
    bool isReplaying = false;
    QVector<QVector<QVector3D>> preTargetPos, preTrajPos, preAnchorPos, preCablePos;
    bool isViewShow = false;
    QWidget* main3DViewer;
//    Q3DSurface *graph;
//    QSurface3DSeries *series;
    Q3DScatter *graph;
    QScatter3DSeries *seriesFrame, *seriesTarget, *seriesTraj, *seriesAnchor, *seriesCable, *seriesAnchor2Cable;

    void displayInfo(std::string info, std::string tpye = "normal");
    void clearInfo();
    bool hasReportErr = false;
    void endChange(int areaNum);
    void refreshAxisRuntimeCounts();
    void initializePrimaryOperationUi();
    void initializeConnectionStatusUi();
    void initializeLegacyGcUiFallbacks();
    void setConnectionStatusIndicator(QLabel* label,
                                      const QString& title,
                                      HardwareInterface::ConnectionState state,
                                      const QString& tooltip = QString(),
                                      const QString& stateTextOverride = QString());
    void refreshConnectionStatusUi(bool force = false);
    void refreshPrimaryOperationUiState(const RobotStateSnapshot& robotState);
    void showPrimaryOperationFeedback(const QString& text, const QString& type, int durationMs = 1800);
    void initializeCalibrationUi();
    void refreshCalibrationUiState();
    QString calibrationStorageDirPath() const;
    QString latestCalibrationSnapshotPath() const;
    QString formatCalibrationVectorPreview(const std::vector<double>& values, int maxCount = 4) const;
    bool saveCalibrationSnapshotToFile(const QString& filePath, bool announce = true);
    bool loadCalibrationSnapshotFromFile(const QString& filePath, bool announce = true);
    void startZeroCalibrationWorkflow();
    void stopZeroCalibrationWorkflow(bool announce = true);
    bool beginCalibrationPretensionStage(const QString& sourceName);
    bool confirmZeroCalibrationWorkflow();
    std::vector<double> buildCalibrationDesignHomePose(double zOffsetMm = 0.0) const;
    bool buildCalibrationMechanicalHomingTrajectory(
            std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            QString& errorMessage) const;
    bool buildCalibrationMechanicalHomingPvtCommand(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            PvtExecutionWorker::PvtCommand& command,
            QString& errorMessage);
    bool executeCalibrationMechanicalHoming(const QString& sourceName);
    void initializeParameterConfigUi();
    void initializeForceTraceTestUi();
    void populateForceTraceDefaultObjects();
    void runPdoTraceProbeFromUi();
    void runTraceProbeFromUi();
    void appendForceTraceTestLog(const QString& text);
    QString parameterConfigStorageDirPath() const;
    QString uiEventLogDirPath() const;
    QString uiEventLogFilePath() const;
    QString structuredFaultLogFilePath() const;
    QString softwareFaultGuardLogFilePath() const;
    QString runtimeDiagnosticsReportFilePath() const;
    QString poseSimulationCableLengthFilePath() const;
    QJsonObject buildParameterConfigSnapshot() const;
    QJsonObject buildZeroPoseConfigSnapshot() const;
    bool saveParameterConfigToFile(const QString& filePath, QString* errorMessage = nullptr) const;
    bool saveZeroPoseConfigToFile(const QString& filePath, QString* errorMessage = nullptr) const;
    bool loadParameterConfigFromFile(const QString& filePath, QString* errorMessage = nullptr);
    void applyParameterConfigSnapshot(const QJsonObject& snapshot);
    MotorFeedbackDisplayUnit currentMotorFeedbackDisplayUnit() const;
    void refreshMotorLimitUnitUi(bool convertValues);
    void applyParameterTemplate(const QString& templateName);
    void appendMessageHistoryEntry(const QString& message, const QString& type);
    void appendUiEventLog(const QString& message, const QString& type);
    void appendStructuredFaultLog(const QJsonObject& record) const;
    void initializeSessionRecordingUi();
    void refreshSessionRecordingUi();
    bool startSessionRecording();
    bool stopSessionRecording(bool announce = true);
    void captureSessionRecordingSample(const ControlWorker::Snapshot& snapshot);
    QString buildSessionRecordingExportText() const;
    bool writeSessionRecordingDiagnosticRawSections(QTextStream& stream, QString* errorMessage = nullptr);
    bool writeSessionRecordingExport(QString* outputPath = nullptr, bool announce = true);
    void updateRuntimeDiagnostics();
    void resetRuntimeDiagnosticsState(bool resetSources = true);
    void trimRuntimeDiagnosticsHistory(qint64 nowMs);
    RuntimeDiagnosticsSummary buildRuntimeDiagnosticsSummary() const;
    void refreshRuntimeDiagnosticsUi();
    QJsonObject buildRuntimeDiagnosticsReportJson() const;
    bool writeRuntimeDiagnosticsReport(QString* outputPath = nullptr, bool announce = false);
    bool writePoseSimulationCableLengthFile(QString* outputPath = nullptr);
    QJsonObject buildRobotStateSnapshotJson(const RobotStateSnapshot& state) const;
    QJsonObject buildRuntimeStateSnapshotJson() const;
    QJsonObject buildControlSnapshotJson(const ControlWorker::Snapshot& snapshot) const;
    QJsonObject buildStructuredFaultRecord(const QString& eventType,
                                          int level,
                                          int code,
                                          const QString& summary,
                                          const QString& detail,
                                          bool stopActionAttempted,
                                          bool stopActionSucceeded,
                                          const QString& note = QString()) const;
    void showMessageHistoryDialog();
    bool openUiEventLogDirectory(bool announceFailure = true);
    bool openUiEventLogFile(bool announceFailure = true);
    void installSoftwareFaultGuards();
    void uninstallSoftwareFaultGuards();

    // 闆疯禌鎺у埗鍗?
    bool setMotorControllerEnable(bool enable);// 鍚姩/鍏抽棴鐢垫満浣胯兘
    bool toggleServoHoldFromUi();
    bool motorStop();// 鐢垫満鍋滄
    bool togglePauseResumeFromUi();
    bool stopMotionFromControlBox();
    bool motor2Home();// 鐢垫満褰掗浂浣?
    void returnMotorAxesToStart(const std::vector<int>& motorIndex, const std::vector<double>& targetTheta);
    bool runPoseMode();// 鎵ц浣嶇疆妯″紡
    bool startPosePvtTrajectory();// 灏嗕綅鎺т豢鐪熺粨鏋滀笅鍙戣嚦闆疯禌 PVT
    void handlePoseModeSimulationFinished();
    bool runGCMode();// 鍚姩/闆跺姏鎷栧姩妯″紡

    void singleMotorEnable();
    void singleMotorStart();
    void singleMotorStop();
    void singleMotorUpdatePos();
    void updateSingleMotorActualPos();
    void refreshSingleMotorEnableStateUi();
    int selectedSingleMotorIndex() const;
    void disableForceControlForManualMotorTest();
    bool validateMotorCommandLimits(int axisIndex,
                                    double relativeTargetPos,
                                    double targetVel,
                                    const QString& actionName,
                                    QString* errorMessage = nullptr) const;
    bool waitMotorPositionStable(int timeoutMs = 800, int sampleMs = 50, double tolerance = 0.002);
    bool waitMotorAxesDone(int timeoutMs = 30000, int sampleMs = 50);
    void syncMotorHomeAndDisplayZero(const QString& infoMessage = QString(),
                                     bool forceUseCurrentPosition = false);
    void setAllMotorHomeToCurrentPosition();
    void resetMotorPosDisplayZero();
    void refreshMotorPosDisplay();

    void readTrajFile();// 璇诲彇杞ㄨ抗鐐规枃浠?
    bool useTrajFile = false;
    std::vector<double> trajFileTrajPx1,trajFileTrajPy1,trajFileTrajPz1,trajFileTrajRx1,trajFileTrajRy1,trajFileTrajRz1;
    std::vector<std::vector<std::vector<std::vector<double>>>> trajFileOfflineTraj;

    void paraChange();
    void updatePara();
    bool applyLeadshineHardwareConfigFromUi();
    bool applyLeadshineAxisEquivFromUi();
    void syncMotorSoftwareLimitsToHardware();
    double configuredLeadshineAxisEquivForAxis(int axisIndex) const;
    void delay(unsigned int msec);
    void stopPoseThread(PositionSimulationModel*& thread);
    void stopGravityThread();
    void stopControlThread();
    void stopPositionThread();
    void startMonitor();
    void stopMonitor();
    void startSafetyMonitor();
    void stopSafetyMonitor();
    void initializeUdpBridge();
    void stopUdpBridge();
    void startUdpForRealtimeMode();
    void stopUdpForRealtimeMode();
    void updateUdpStatusPayload();
    QString buildUdpStatusSummary() const;
    QString udpRealtimeTargetIp() const;
    int configuredSensorSampleFrequencyHz() const;
    double configuredForceSensorLowPassCutoffHz() const;
    void handleUdpPoseCommand(const UdpPoseCommand& command);
    void handleUdpTrajectoryChunk(const UdpTrajectoryChunk& chunk);
    void initializeSafetyUiControls();
    void initializeFaultInjectionUiControls();
    void refreshRunModeUiStateThrottled();
    void refreshSafetyUiState();
    void refreshFaultInjectionUiControls();
    void refreshSafetyUiStateForState(const RobotStateSnapshot& robotState);
    void refreshSafetyMonitorHeartbeat(qint64 timestampMs = -1);
    void updateSafetyMonitorConfig();
    void clearSafetyFaultLatch(bool announce = false);
    bool ensureSafetyReadyForMotion(const QString& actionName);
    void handleSafetyFault(int level, int code, const QString& summary, const QString& detail);
    bool applySafetyStopAction(int level, const QString& summary, const QString& detail);
    void resetSafetyStateFromUi();
    void triggerSoftwareEmergencyStop();
    void triggerInjectedCableBreak();
    void triggerInjectedMotorFault();
    void triggerInjectedPlcCommunicationFault();
    bool buildSafetyWorkspaceBounds(WorkspaceBounds& bounds, QString* errorMessage = nullptr) const;
    bool currentActualEndPose(std::vector<double>& pose, int maxAgeMs = 500) const;
    bool validatePoseWithinWorkspace(const std::vector<double>& pose,
                                     const WorkspaceBounds& bounds,
                                     QString& errorMessage,
                                     bool* nearBoundary = nullptr,
                                     double* overflowAmount = nullptr) const;
    bool validateTrajectoryWithinWorkspace(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            QString& errorMessage) const;
    bool extractTrajectoryEndpointPose(
            const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
            std::vector<double>& pose) const;
    void applyTrajectoryStartPoseToUi(const std::vector<double>& pose);
    void initializeRunModeControls();
    void initializeTrajectoryModeBoxes();
    RunMode selectedRunModeFromUi() const;
    QString runModeDisplayName(RunMode mode) const;
    int udpRealtimeListenPort() const;
    int udpRealtimeTargetPort() const;
    void syncUdpTargetPortWidgets(int port);
    void applyUdpRuntimeConfigChange();
    void refreshUdpPacketSummaryUi();
    void setRunMode(RunMode mode, bool announce = true);
    void refreshRunModeUiState();
    bool isRealtimeManualPoseInputSelected() const;
    bool shouldUseTrajectoryFileInput() const;
    void updatePoseInputUiState();
    void clearTrajectoryFileSelection();
    void syncRealtimeStartPoseFromCurrentPose(bool logResult);
    bool startActiveRunMode(bool fromControlBox = false);
    bool handleRunModePrimaryAction();
    bool handleRunModeSecondaryAction();
    void onPosTrajModeChanged(const QString& mode);
    void setForceControlSelectionEnabled(bool enabled);
    bool hasForceControlAxisSelected() const;
    bool isSingleCableForceDebugModeActive() const;
    void enforceSingleCableForceSelection(QRadioButton* preferredButton = nullptr);
    void syncSingleCableForceDebugPretensionForce();
    void updateSingleCableForceDebugUiState();
    void handleControlBoxStatus(int motionControl, int speedZero, int zeroCalib);
    void handleControlBoxMotionControl(int state);
    void handleControlBoxSpeedZero(int state);
    void handleControlBoxZeroCalib(int state);
    bool isForceControlThreadActuallyRunning() const;
    bool engageSpeedZeroControlledStop(QString* failureReason = nullptr);
    void stopAllMotorAxesForSpeedZero();
    void checkTrajectoryFinished();
    void resetControlUiState();
    std::vector<std::vector<std::vector<double>>> buildCableContactPointPos() const;
    std::vector<std::vector<double>> buildFixedAnchorHome() const;
    std::vector<double> buildCableMotorCof() const;
    double buildPulleyRadius() const;
    std::vector<double> buildFixedAnchorDis() const;
    std::vector<std::vector<std::vector<double>>> splitAnchorPositionsByEnd(const std::vector<std::vector<double>>& anchorPosTemp) const;
    void clearPlannedPoseForceTrajectory();
    bool planPoseForceTrajectory(const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
                                 QString& errorMessage,
                                 std::vector<double>* failedPose = nullptr,
                                 int* failedStep = nullptr,
                                 double* failedTime = nullptr);
    std::vector<std::vector<double>> mapCableForceTrajectoryToSensorExpected(
        const std::vector<std::vector<double>>& cableForceTraj) const;
    bool isHybridPoseForceModeRequested() const;
    void updateHybridPoseForceModeButtonText();
    void refreshHybridPoseForceSelectionUi();
    bool buildHybridPoseForceModeConfig(HybridPoseForceModeConfig& out,
                                        QString& errorMessage) const;
    bool applyHybridPoseForceExpectedForceAtPoint(int pointIndex, QString* errorMessage = nullptr);
    void syncHybridPoseForceExpectedForceForActiveTrajectory();
    void clearHybridPoseForceModeState(bool disableForceThread);

    void allUseFC(bool isChecked);// 鍏ㄩ儴缁崇储閮戒娇鐢ㄥ姏鎺?
    bool resetCableForceHome();// 鍔涗紶鎰熷櫒璇绘暟褰掗浂浣?
    bool resetRotHome();// 鍙栧綋鍓嶅Э鎬佷负鏈璧峰濮挎€?
    bool setCableHome();// 鐢垫満棰勭揣瀹屾垚
    bool isCableForceWithinHomeConfirmTolerance() const;
    bool shouldSkipCableHomeCheckForMotion(const QString& motionName) const;
    void resetContinuousTrajectoryOptions();
    bool ensureCableHomeReadyForMotion(const QString& motionName);
    void updateCableHomeConfirmEnabled();
    void resetFCPIDPara();
    void requestImmediateForceControlUpdate();
    OneDimKalmanHandler fcOneDimKalmanHandler;

    std::vector<double> endCurRotHome;
    std::vector<double> motorCurPos, motorCurPosRaw, motorCurAbsPos, motorPosDisplayZero, cableHomePos, forceSensorCurHome;
    std::vector<QLabel*> curMotorPosTextVec;

    void gcDataProcessor(std::vector<double> gcCableForce, std::vector<std::vector<double>> PlatformPose, std::vector<std::vector<double>> PlatformForce);
    void emergeGCData(std::vector<std::vector<double>> PlatformPose);
    void emergeGCDataNoCam(std::vector<std::vector<double> > cableLen);
    std::vector<QRadioButton*> useFCBtnVec;
    std::vector<QDoubleSpinBox*> mainForceSensorData,mainForceSensorExp,mainMotorTorqueData;
    std::vector<QDoubleSpinBox*> mainGCEndWeightVec,mainGCEndXVec,mainGCEndYVec,mainGCEndZVec,mainGCEndMxVec,mainGCEndMyVec,mainGCEndMzVec;
    std::vector<double> mainForceSensorDataVal,mainForceSensorExpVal;

    bool gcResetTime = false;
    bool forceControlSelectionUiEnabled = true;
    bool suppressSingleCableForceSelectionUpdate = false;
    bool singleCableForceDebugPretensionSaved = false;
    double singleCableForceDebugSavedPretensionForce = 0.0;

    QString motorAxisText() const;
    QString linearMotorAxisText() const;
    bool isMotorAxis(int axisIndex) const;
    bool isLinearMotorAxis(int axisIndex) const;
    bool isModeledMotorAxis(int axisIndex) const;

    // 杞村弬鏁版帶浠跺簭鍒?
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
    std::vector<int> eachEndCableNum;// 璁板綍姣忎釜鏈鎷ユ湁鐨勭怀绱㈡暟

    bool testDyn = false;
};
#endif // MAINWINDOW_H
