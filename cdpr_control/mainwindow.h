#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>

#include "cdpr/CdprConfiguration.h"
#include "common/ContiTypes.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QThread;
class QTimer;
class MotionControlWorker;
class CdprCoordinator;
class QChart;
class QLineSeries;
class QValueAxis;
class ZoomableChartView;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

signals:
    void initializeBoardRequested(const ContiTestConfig &config);
    void closeBoardRequested();
    void enableAxesRequested(const ContiTestConfig &config);
    void disableAxesRequested(const ContiTestConfig &config);
    void enableAllAxesRequested();
    void disableAllAxesRequested();
    void startTestRequested(const ContiTestConfig &config);
    void stopTestRequested(bool emergency);
    void enableJogAxisRequested(const SingleAxisJogConfig &config);
    void disableJogAxisRequested(const SingleAxisJogConfig &config);
    void setJogAxisZeroRequested(const SingleAxisJogConfig &config);
    void startPointMoveRequested(const SingleAxisJogConfig &config);
    void stopPointMoveRequested(bool emergency);
    void startVelocityControlRequested(const VelocityControlConfig &config);
    void stopVelocityControlRequested(bool emergency);
    void resetVelocityControllerRequested();
    void writeTorqueVelocityLimitRequested(const TorqueTestConfig &config);
    void startTorqueTestRequested(const TorqueTestConfig &config);
    void updateTorqueCommandRequested(const TorqueTestConfig &config);
    void stopTorqueTestRequested(bool emergency);
    void startTraceDelayCalibrationRequested(const TraceDelayCalibrationConfig &config);
    void stopTraceDelayCalibrationRequested(bool emergency);
    void resetTraceDelayCalibrationAxisRequested(quint16 axis);
    void startTelemetryRecordingRequested();
    void stopTelemetryRecordingRequested();
    void refreshBusCycleRequested();
    void loadCdprConfigurationRequested(const QString &path);
    void validateCdprConfigurationRequested();
    void writeCdprConfigurationTemplateRequested(const QString &path);
    void setCdprInitialPoseSourceRequested(int source);
    void setCdprPresetInitialPoseRequested(
        double x, double y, double z,
        double roll, double pitch, double yaw);
    void connectCdprNokovRequested(const QString &serverAddress);
    void disconnectCdprNokovRequested();
    void captureCdprInitialStateRequested();
    void setCdprForceInputSourceRequested(int source);
    void setCdprSimulatedWrenchRequested(
        double fx, double fy, double fz,
        double mx, double my, double mz);
    void clearCdprSimulatedWrenchRequested();
    void resetCdprDynamicsRequested();
    void advanceCdprDynamicsOnceRequested();
    void prepareCdprOfflinePvtRequested(
        const CdprOfflinePvtRequest &request);
    void startCdprOfflinePvtRequested(
        const CdprOfflinePvtPlan &plan);
    void stopCdprOfflinePvtRequested(bool emergency);
    void startCdprVelocityControlRequested(const CdprOfflinePvtPlan &plan,
                                           const CdprVelocityControlConfig &config);
    void stopCdprVelocityControlRequested(bool emergency);

private slots:
    void onStageChanged(int index);
    void onInitializeClicked();
    void onCloseBoardClicked();
    void onEnableAxesClicked();
    void onDisableAxesClicked();
    void onEnableAllAxesClicked();
    void onDisableAllAxesClicked();
    void onStartClicked();
    void onStopClicked();
    void onEmergencyStopClicked();
    void onCopyLogClicked();
    void onClearLogClicked();
    void onEnableJogAxisClicked();
    void onDisableJogAxisClicked();
    void onSetJogAxisZeroClicked();
    void onStartPointMoveClicked();
    void onStopPointMoveClicked();
    void onEmergencyStopPointMoveClicked();
    void onUseActualPositionClicked();
    void onJogPositionDisplayModeChanged();
    void onStartRecordingClicked();
    void onStopRecordingClicked();
    void onVelocityEnableAxisClicked();
    void onVelocityDisableAxisClicked();
    void onVelocityStartClicked();
    void onVelocityStopClicked();
    void onVelocityEmergencyStopClicked();
    void onVelocityResetClicked();
    void onVelocityClearCurvesClicked();
    void onTorqueEnableAxisClicked();
    void onTorqueDisableAxisClicked();
    void onTorqueWriteOdClicked();
    void onTorqueStartClicked();
    void onTorqueUpdateClicked();
    void onTorqueStopClicked();
    void onTorqueEmergencyStopClicked();
    void onTorqueClearCurvesClicked();
    void onTraceDelayStartClicked();
    void onTraceDelayStopClicked();
    void onTraceDelayEmergencyStopClicked();
    void onTraceDelayResetAxisClicked();
    void onBusCycleSelectionChanged(int index);
    void onProducerPeriodChanged(int periodMs);
    void onCdprLoadConfigurationClicked();
    void onCdprCreateTemplateClicked();
    void updateCdprStatus(const CdprUiStatus &status);
    void updateCdprOfflinePvtPlan(const CdprOfflinePvtPlan &plan);
    void updateCdprOfflinePvtStatus(const CdprOfflinePvtStatus &status);
    void updateCdprVelocityControlStatus(const CdprVelocityControlStatus &status);
    void appendLog(const QString &message);
    void updateStatus(const ContiStatus &status);

private:
    ContiTestConfig collectConfig() const;
    SingleAxisJogConfig collectJogConfig() const;
    VelocityControlConfig collectVelocityConfig() const;
    TorqueTestConfig collectTorqueConfig() const;
    TraceDelayCalibrationConfig collectTraceDelayCalibrationConfig() const;
    CdprOfflinePvtRequest collectCdprOfflinePvtRequest() const;
    CdprVelocityControlConfig collectCdprVelocityControlConfig() const;
    void invalidateCdprOfflinePvtPlan();
    void connectWorker();
    void initializeContiTrajectoryChart();
    void initializeUiRefreshTimer();
    void refreshUiAndCharts();
    void updateContiTrajectoryChart();
    void initializeVelocityControlCharts();
    void updateVelocityControlCharts();
    void clearVelocityControlCharts();
    void initializeTorqueTestCharts();
    void updateTorqueTestCharts();
    void clearTorqueTestCharts();
    void updateTorqueOutputSpeedEquivalent();
    double selectedDegreesPerCardUnit() const;
    void updateGlobalCardUnitUi();
    void updateChartRanges(ZoomableChartView *view,
                           const QList<QLineSeries *> &series, double timeSeconds,
                           double minimumSpan) const;
    int selectedBusCycleUs() const;
    void normalizeProducerPeriodForBusCycle();
    void updateBusPeriodUi();

private:
    Ui::MainWindow *ui_ = nullptr;
    QThread *workerThread_ = nullptr;
    MotionControlWorker *worker_ = nullptr;
    QThread *cdprThread_ = nullptr;
    CdprCoordinator *cdprCoordinator_ = nullptr;
    ContiStatus latestStatus_;
    bool hasLatestStatus_ = false;
    bool statusUiDirty_ = false;
    bool axisStatusRendered_ = false;
    bool lastAxisStatusBoardInitialized_ = false;
    quint16 lastAxisEnabledMask_ = 0;
    int lastDetectedAxisCount_ = -1;
    QTimer *uiRefreshTimer_ = nullptr;
    QChart *contiTrajectoryChart_ = nullptr;
    QValueAxis *contiTrajectoryTimeAxis_ = nullptr;
    QValueAxis *contiTrajectoryValueAxis_ = nullptr;
    QLineSeries *contiExpectedTrajectorySeries_ = nullptr;
    QLineSeries *contiActualTrajectorySeries_ = nullptr;
    quint64 lastContiTrajectoryTraceSequence_ = 0;
    quint64 contiTrajectoryTraceStartTimeUs_ = 0;
    QChart *velocityPositionChart_ = nullptr;
    QChart *velocityErrorChart_ = nullptr;
    QChart *velocitySpeedChart_ = nullptr;
    QLineSeries *velocityPositionSeries_[3] {nullptr, nullptr, nullptr};
    QLineSeries *velocityErrorSeries_[4] {nullptr, nullptr, nullptr, nullptr};
    QLineSeries *velocitySpeedSeries_[4] {nullptr, nullptr, nullptr, nullptr};
    QValueAxis *velocityPositionTimeAxis_ = nullptr;
    QValueAxis *velocityPositionValueAxis_ = nullptr;
    QValueAxis *velocityErrorTimeAxis_ = nullptr;
    QValueAxis *velocityErrorValueAxis_ = nullptr;
    QValueAxis *velocitySpeedTimeAxis_ = nullptr;
    QValueAxis *velocitySpeedValueAxis_ = nullptr;
    QVector<VelocityPlotSample> pendingVelocityPlotSamples_;
    QVector<VelocityPlotSample> velocityPlotBucket_;
    QList<QPointF> velocityPositionDisplayPoints_[3];
    QList<QPointF> velocityErrorDisplayPoints_[4];
    QList<QPointF> velocitySpeedDisplayPoints_[4];
    qint64 velocityPlotBucketIndex_ = -1;
    quint64 lastVelocityRunId_ = 0;
    double lastVelocityPlotTimeS_ = -1.0;
    QChart *torqueValueChart_ = nullptr;
    QChart *torqueMotionChart_ = nullptr;
    QLineSeries *torqueValueSeries_[2] {nullptr, nullptr};
    QLineSeries *torqueMotionSeries_[2] {nullptr, nullptr};
    QValueAxis *torqueValueTimeAxis_ = nullptr;
    QValueAxis *torqueValueAxis_ = nullptr;
    QValueAxis *torqueMotionTimeAxis_ = nullptr;
    QValueAxis *torqueMotionValueAxis_ = nullptr;
    QVector<TorquePlotSample> pendingTorquePlotSamples_;
    QList<QPointF> torqueValueDisplayPoints_[2];
    QList<QPointF> torqueMotionDisplayPoints_[2];
    quint64 lastTorqueRunId_ = 0;
    double lastTorquePlotTimeS_ = -1.0;
    CdprOfflinePvtPlan cdprOfflinePvtPlan_;
    CdprOfflinePvtStatus cdprOfflinePvtStatus_;
    CdprVelocityControlStatus cdprVelocityControlStatus_;
    bool cdprAllMappedAxesEnabled_ = false;
};

#endif // MAINWINDOW_H
