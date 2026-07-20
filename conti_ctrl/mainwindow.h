#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>

#include "common/ContiTypes.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QThread;
class QTimer;
class ContiWorker;
class QChart;
class QLineSeries;
class QValueAxis;

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
    void startTestRequested(const ContiTestConfig &config);
    void stopTestRequested(bool emergency);
    void refreshFeedbackRequested();
    void enableJogAxisRequested(const SingleAxisJogConfig &config);
    void disableJogAxisRequested(const SingleAxisJogConfig &config);
    void setJogAxisZeroRequested(const SingleAxisJogConfig &config);
    void startPointMoveRequested(const SingleAxisJogConfig &config);
    void stopPointMoveRequested(bool emergency);
    void startVelocityControlRequested(const VelocityControlConfig &config);
    void stopVelocityControlRequested(bool emergency);
    void resetVelocityControllerRequested();
    void startTelemetryRecordingRequested();
    void stopTelemetryRecordingRequested();
    void refreshBusCycleRequested();

private slots:
    void onStageChanged(int index);
    void onInitializeClicked();
    void onCloseBoardClicked();
    void onEnableAxesClicked();
    void onDisableAxesClicked();
    void onStartClicked();
    void onStopClicked();
    void onEmergencyStopClicked();
    void onRefreshFeedbackClicked();
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
    void onBusCycleSelectionChanged(int index);
    void onProducerPeriodChanged(int periodMs);
    void appendLog(const QString &message);
    void updateStatus(const ContiStatus &status);

private:
    ContiTestConfig collectConfig() const;
    SingleAxisJogConfig collectJogConfig() const;
    VelocityControlConfig collectVelocityConfig() const;
    void connectWorker();
    void initializeTelemetryCharts();
    void updateTelemetryCharts();
    void updateContiTrajectoryChart();
    void initializeVelocityControlCharts();
    void updateVelocityControlCharts();
    void clearVelocityControlCharts();
    void updateChartRanges(QChart *chart, QValueAxis *timeAxis, QValueAxis *valueAxis,
                           const QList<QLineSeries *> &series, double timeSeconds,
                           double minimumSpan) const;
    int selectedBusCycleUs() const;
    void normalizeProducerPeriodForBusCycle();
    void updateBusPeriodUi();

private:
    Ui::MainWindow *ui_ = nullptr;
    QThread *workerThread_ = nullptr;
    ContiWorker *worker_ = nullptr;
    ContiStatus latestStatus_;
    bool hasLatestStatus_ = false;
    QTimer *telemetryPlotTimer_ = nullptr;
    QChart *positionChart_ = nullptr;
    QChart *followingErrorChart_ = nullptr;
    QChart *contiTrajectoryChart_ = nullptr;
    QLineSeries *positionSeries_[4] {nullptr, nullptr, nullptr, nullptr};
    QLineSeries *followingErrorSeries_[2] {nullptr, nullptr};
    QValueAxis *positionTimeAxis_ = nullptr;
    QValueAxis *positionValueAxis_ = nullptr;
    QValueAxis *errorTimeAxis_ = nullptr;
    QValueAxis *errorValueAxis_ = nullptr;
    QValueAxis *contiTrajectoryTimeAxis_ = nullptr;
    QValueAxis *contiTrajectoryValueAxis_ = nullptr;
    QLineSeries *contiExpectedTrajectorySeries_ = nullptr;
    QLineSeries *contiActualTrajectorySeries_ = nullptr;
    quint64 lastPlottedTraceSequence_ = 0;
    quint64 lastContiTrajectoryTraceSequence_ = 0;
    quint64 contiTrajectoryTraceStartTimeUs_ = 0;
    QChart *velocityPositionChart_ = nullptr;
    QChart *velocityErrorChart_ = nullptr;
    QChart *velocitySpeedChart_ = nullptr;
    QLineSeries *velocityPositionSeries_[3] {nullptr, nullptr, nullptr};
    QLineSeries *velocityErrorSeries_[3] {nullptr, nullptr, nullptr};
    QLineSeries *velocitySpeedSeries_[4] {nullptr, nullptr, nullptr, nullptr};
    QValueAxis *velocityPositionTimeAxis_ = nullptr;
    QValueAxis *velocityPositionValueAxis_ = nullptr;
    QValueAxis *velocityErrorTimeAxis_ = nullptr;
    QValueAxis *velocityErrorValueAxis_ = nullptr;
    QValueAxis *velocitySpeedTimeAxis_ = nullptr;
    QValueAxis *velocitySpeedValueAxis_ = nullptr;
    quint64 lastVelocityRunId_ = 0;
    double lastVelocityPlotTimeS_ = -1.0;
};

#endif // MAINWINDOW_H
