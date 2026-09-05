#include "datavisualizationcontroller.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "MatrixFun.h"
#include "qcustomplot.h"
#include "runtimefeatureswitches.h"

#include <QBoxLayout>
#include <QCheckBox>
#include <QDateTime>
#include <QDir>
#include <QLayout>
#include <QPixmap>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QTimer>
#include <QVector>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace DataVisualizationController;

namespace {

template <typename T>
T* findOptionalUiObject(const QWidget* root, const char* objectName)
{
    return root ? root->findChild<T*>(QString::fromLatin1(objectName)) : nullptr;
}

QCheckBox* ensureCheckBoxInLayout(QWidget* root,
                                  const char* layoutName,
                                  const char* objectName,
                                  const QString& text,
                                  const QString& toolTip,
                                  int insertIndex = -1)
{
    if(!root){
        return nullptr;
    }
    if(QCheckBox* existing = findOptionalUiObject<QCheckBox>(root, objectName)){
        return existing;
    }

    QLayout* layout = findOptionalUiObject<QLayout>(root, layoutName);
    if(!layout){
        return nullptr;
    }

    QWidget* parentWidget = layout->parentWidget() ? layout->parentWidget() : root;
    QCheckBox* checkBox = new QCheckBox(text, parentWidget);
    checkBox->setObjectName(QString::fromLatin1(objectName));
    checkBox->setToolTip(toolTip);
    checkBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    if(QBoxLayout* boxLayout = qobject_cast<QBoxLayout*>(layout)){
        const int boundedIndex = insertIndex < 0 ?
                    boxLayout->count() :
                    std::min(std::max(insertIndex, 0), boxLayout->count());
        boxLayout->insertWidget(boundedIndex, checkBox);
    }
    else{
        layout->addWidget(checkBox);
    }
    return checkBox;
}

QPushButton* ensureButtonInLayout(QWidget* root,
                                  const char* layoutName,
                                  const char* objectName,
                                  const QString& text,
                                  const QString& toolTip,
                                  int insertIndex = -1)
{
    if(!root){
        return nullptr;
    }
    if(QPushButton* existing = findOptionalUiObject<QPushButton>(root, objectName)){
        return existing;
    }

    QLayout* layout = findOptionalUiObject<QLayout>(root, layoutName);
    if(!layout){
        return nullptr;
    }

    QWidget* parentWidget = layout->parentWidget() ? layout->parentWidget() : root;
    QPushButton* button = new QPushButton(text, parentWidget);
    button->setObjectName(QString::fromLatin1(objectName));
    button->setToolTip(toolTip);
    button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    if(QBoxLayout* boxLayout = qobject_cast<QBoxLayout*>(layout)){
        const int boundedIndex = insertIndex < 0 ?
                    boxLayout->count() :
                    std::min(std::max(insertIndex, 0), boxLayout->count());
        boxLayout->insertWidget(boundedIndex, button);
    }
    else{
        layout->addWidget(button);
    }
    return button;
}

bool hasFiniteValues(const std::vector<double>& values, int minSize = 0)
{
    if(static_cast<int>(values.size()) < minSize){
        return false;
    }
    for(double value : values){
        if(!std::isfinite(value)){
            return false;
        }
    }
    return true;
}

bool hasFinitePoseMatrix(const std::vector<std::vector<double>>& values)
{
    if(values.empty() || values.front().size() < 6){
        return false;
    }
    for(const std::vector<double>& row : values){
        if(!hasFiniteValues(row, 6)){
            return false;
        }
    }
    return true;
}

std::vector<std::vector<double>> normalizedPoseMatrix(std::vector<std::vector<double>> values)
{
    if(!hasFinitePoseMatrix(values)){
        return {};
    }
    for(std::vector<double>& row : values){
        row.resize(6);
    }
    return values;
}

int poseTrajectoryPointCountForVisualization(
        const std::vector<std::vector<std::vector<std::vector<double>>>>& trajectory)
{
    if(trajectory.empty() ||
            trajectory.front().empty() ||
            trajectory.front().front().size() < 6 ||
            trajectory.front().front().front().empty()){
        return 0;
    }

    const int pointCount =
            static_cast<int>(trajectory.front().front().front().size());
    for(const auto& endTraj : trajectory){
        if(endTraj.empty() || endTraj.front().size() < 6){
            return 0;
        }
        for(int dim=0; dim<6; ++dim){
            if(static_cast<int>(endTraj.front()[dim].size()) != pointCount){
                return 0;
            }
        }
    }
    return pointCount;
}

std::vector<double> poseTrajectoryTimeAxisForVisualization(
        const std::vector<std::vector<std::vector<std::vector<double>>>>& trajectory,
        int pointCount,
        double fallbackStepSec)
{
    if(pointCount <= 0){
        return {};
    }

    if(!trajectory.empty() &&
            trajectory.front().size() > 3 &&
            !trajectory.front()[3].empty() &&
            static_cast<int>(trajectory.front()[3][0].size()) == pointCount &&
            hasFiniteValues(trajectory.front()[3][0], pointCount)){
        return trajectory.front()[3][0];
    }

    if(fallbackStepSec <= 0.0 || !std::isfinite(fallbackStepSec)){
        fallbackStepSec = 0.01;
    }
    std::vector<double> timeAxis(pointCount, 0.0);
    for(int i=0; i<pointCount; ++i){
        timeAxis[i] = i * fallbackStepSec;
    }
    return timeAxis;
}

int nearestPoseTrajectoryPointIndex(const std::vector<double>& timeAxis,
                                    double trajectoryTimeSec)
{
    if(timeAxis.empty() || !std::isfinite(trajectoryTimeSec)){
        return -1;
    }
    if(trajectoryTimeSec <= timeAxis.front()){
        return 0;
    }
    if(trajectoryTimeSec >= timeAxis.back()){
        return static_cast<int>(timeAxis.size()) - 1;
    }

    auto upperIt = std::lower_bound(timeAxis.begin(), timeAxis.end(), trajectoryTimeSec);
    if(upperIt == timeAxis.begin()){
        return 0;
    }
    if(upperIt == timeAxis.end()){
        return static_cast<int>(timeAxis.size()) - 1;
    }

    const int upperIndex = static_cast<int>(std::distance(timeAxis.begin(), upperIt));
    const int lowerIndex = upperIndex - 1;
    const double lowerError = std::abs(trajectoryTimeSec - timeAxis[lowerIndex]);
    const double upperError = std::abs(timeAxis[upperIndex] - trajectoryTimeSec);
    return upperError < lowerError ? upperIndex : lowerIndex;
}

std::vector<double> smoothVisualizationVector(const std::vector<double>& raw,
                                              std::vector<double>& smoothed,
                                              bool enabled,
                                              double dtSec,
                                              double tauSec)
{
    if(raw.size() < 6){
        smoothed.clear();
        return raw;
    }
    std::vector<double> output(raw.begin(), raw.begin() + 6);
    if(!enabled || !std::isfinite(tauSec) || tauSec <= 0.0){
        smoothed.clear();
        return output;
    }

    if(!std::isfinite(dtSec) || dtSec <= 0.0){
        if(smoothed.size() >= 6){
            return std::vector<double>(smoothed.begin(), smoothed.begin() + 6);
        }
        smoothed = output;
        return output;
    }

    if(smoothed.size() < 6){
        smoothed = output;
        return smoothed;
    }

    const double alpha = std::min(1.0, std::max(0.0, dtSec / (tauSec + dtSec)));
    for(int i=0; i<6; ++i){
        if(!std::isfinite(output[i])){
            output[i] = smoothed[i];
            continue;
        }
        if(!std::isfinite(smoothed[i])){
            smoothed[i] = output[i];
            continue;
        }
        smoothed[i] += alpha * (output[i] - smoothed[i]);
        output[i] = smoothed[i];
    }
    return output;
}

} // namespace

void MainWindow::setupDataVisualizationTab()
{
    if(!ui){
        return;
    }

    dataVizTab = ui->dataVizTab;
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        if(dataVizTab){
            dataVizTab->setEnabled(false);
            dataVizTab->setToolTip(
                        QStringLiteral("数据可视化已由性能开关停用；不采样、不累计曲线数据，也不执行重绘。"));
        }
        if(ui->dataVizClearButton){
            ui->dataVizClearButton->setEnabled(false);
        }
        return;
    }

    motorControlPlotHost = ui->motorControlPlotHost;
    cableTensionPlotHost = ui->cableTensionPlotHost;
    endEffectorPosPlotHost = ui->endEffectorPosPlotHost;
    endEffectorVelPlotHost = ui->endEffectorVelPlotHost;
    endEffectorAccPlotHost = ui->endEffectorAccPlotHost;
    actualEndEffectorPosPlotHost =
            findOptionalUiObject<QCustomPlot>(this, "actualEndEffectorPosPlotHost");
    actualEndEffectorVelPlotHost =
            findOptionalUiObject<QCustomPlot>(this, "actualEndEffectorVelPlotHost");
    actualEndEffectorAccPlotHost =
            findOptionalUiObject<QCustomPlot>(this, "actualEndEffectorAccPlotHost");
    cableSpeedPlotHost = ui->cableSpeedPlotHost;
    cableLengthPlotHost = ui->cableLengthPlotHost;
    dataVizFreezeCurvesCheckBox =
            ensureCheckBoxInLayout(this,
                                   "horizontalLayoutDataVizHeader",
                                   "dataVizFreezeCurvesCheckBox",
                                   QStringLiteral("冻结窗口"),
                                   QStringLiteral("勾选后暂停所有曲线窗口刷新；数据仍继续接收，取消勾选后立即刷新到最新窗口"),
                                   2);
    dataVizScreenshotButton =
            ensureButtonInLayout(this,
                                 "horizontalLayoutDataVizHeader",
                                 "dataVizScreenshotButton",
                                 QStringLiteral("保存截图"),
                                 QStringLiteral("保存当前软件主窗口截图到 data/outputmsg/screenshot"));
    actualEndEffectorVelocitySmoothCheckBox =
            ensureCheckBoxInLayout(this,
                                   "verticalLayoutDataVizActualEndEffectorVel",
                                   "actualEndEffectorVelocitySmoothCheckBox",
                                   QStringLiteral("开启平滑"),
                                   QStringLiteral("勾选后对实际末端速度 X/Y/Z/Rx/Ry/Rz 显示值进行一阶低通平滑；加速度曲线默认使用该速度做差分"),
                                   1);
    actualEndEffectorAccelerationSmoothCheckBox =
            ensureCheckBoxInLayout(this,
                                   "verticalLayoutDataVizActualEndEffectorAcc",
                                   "actualEndEffectorAccelerationSmoothCheckBox",
                                   QStringLiteral("开启平滑"),
                                   QStringLiteral("勾选后对实际末端加速度 X/Y/Z/Rx/Ry/Rz 显示值进行一阶低通平滑；加速度由速度曲线差分得到"),
                                   1);
    if(actualEndEffectorVelocitySmoothCheckBox){
        actualEndEffectorVelocitySmoothCheckBox->setChecked(true);
    }
    if(actualEndEffectorAccelerationSmoothCheckBox){
        actualEndEffectorAccelerationSmoothCheckBox->setChecked(true);
    }

    if(!curveDrawer){
        curveDrawer = new CurveDrawer(this);
    }
    disconnect(curveDrawer,
               &CurveDrawer::plotRefreshTick,
               this,
               &MainWindow::updateVisualizationFromPlannedTrajectory);
    if(!plannedTrajectoryVisualizationTimer){
        plannedTrajectoryVisualizationTimer = new QTimer(this);
        plannedTrajectoryVisualizationTimer->setTimerType(Qt::PreciseTimer);
        connect(plannedTrajectoryVisualizationTimer, &QTimer::timeout, this, [this](){
            updateVisualizationFromPlannedTrajectory(
                        plannedTrajectoryVisualizationTimer ?
                            plannedTrajectoryVisualizationTimer->interval() :
                            poseTrajectoryDisplayTimerIntervalMs());
        });
    }
    if(!motorControlInputVisualizationTimer){
        motorControlInputVisualizationTimer = new QTimer(this);
        motorControlInputVisualizationTimer->setTimerType(Qt::PreciseTimer);
        connect(motorControlInputVisualizationTimer, &QTimer::timeout, this, [this](){
            updateMotorControlInputVisualizationFromPvtCommand(
                        motorControlInputVisualizationTimer ?
                            motorControlInputVisualizationTimer->interval() :
                            poseTrajectoryDisplayTimerIntervalMs());
        });
    }
    if(!simulationDataVisualizationTimer){
        simulationDataVisualizationTimer = new QTimer(this);
        simulationDataVisualizationTimer->setTimerType(Qt::PreciseTimer);
        connect(simulationDataVisualizationTimer, &QTimer::timeout, this, [this](){
            updateSimulationDataVisualizationPlayback(
                        simulationDataVisualizationTimer ?
                            simulationDataVisualizationTimer->interval() :
                            simulationDataVisualizationTimerIntervalMs());
        });
    }

    if(ui->dataVizClearButton){
        disconnect(ui->dataVizClearButton,
                   &QPushButton::clicked,
                   this,
                   &MainWindow::clearVisualizationData);
        connect(ui->dataVizClearButton,
                &QPushButton::clicked,
                this,
                &MainWindow::clearVisualizationData);
    }
    if(dataVizScreenshotButton){
        disconnect(dataVizScreenshotButton,
                   &QPushButton::clicked,
                   this,
                   &MainWindow::saveMainWindowScreenshot);
        connect(dataVizScreenshotButton,
                &QPushButton::clicked,
                this,
                &MainWindow::saveMainWindowScreenshot);
    }

    if(dataVizFreezeCurvesCheckBox){
        disconnect(dataVizFreezeCurvesCheckBox, nullptr, this, nullptr);
        connect(dataVizFreezeCurvesCheckBox,
                &QCheckBox::toggled,
                this,
                [this](bool checked){
            if(!checked && simulationDataVisualizationHoldingFrozenResult){
                simulationDataVisualizationHoldingFrozenResult = false;
                lastVisualizationPoseTime = -1.0;
                lastVisualizationPose.clear();
                lastVisualizationVelocity.clear();
            }
            updateDataVisualizationRefreshEnabled();
        });
    }
    if(actualEndEffectorVelocitySmoothCheckBox){
        disconnect(actualEndEffectorVelocitySmoothCheckBox, nullptr, this, nullptr);
        connect(actualEndEffectorVelocitySmoothCheckBox,
                &QCheckBox::toggled,
                this,
                [this](bool){
            smoothedActualEndEffectorVelocity.clear();
            lastActualEndEffectorVelocity.clear();
            smoothedActualEndEffectorAcceleration.clear();
        });
    }
    if(actualEndEffectorAccelerationSmoothCheckBox){
        disconnect(actualEndEffectorAccelerationSmoothCheckBox, nullptr, this, nullptr);
        connect(actualEndEffectorAccelerationSmoothCheckBox,
                &QCheckBox::toggled,
                this,
                [this](bool){
            smoothedActualEndEffectorAcceleration.clear();
        });
    }

    reinitializeDataVisualizationPlots();
    restartPlannedTrajectoryVisualizationTimer();
    updateDataVisualizationRefreshEnabled();
}

void MainWindow::updateDataVisualizationRefreshEnabled()
{
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        if(curveDrawer){
            curveDrawer->setRefreshEnabled(false);
        }
        return;
    }
    if(!curveDrawer){
        return;
    }
    const bool frozen =
            dataVizFreezeCurvesCheckBox &&
            dataVizFreezeCurvesCheckBox->isChecked();
    curveDrawer->setRefreshEnabled(!forcePidTuningExclusiveRefresh &&
                                   !frozen &&
                                   !simulationDataVisualizationHoldingFrozenResult);
}

void MainWindow::reinitializeDataVisualizationPlots()
{
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        return;
    }
    if(!curveDrawer){
        return;
    }

    curveDrawer->setChannelCounts(8, 8);
    curveDrawer->initPlot(motorControlPlotHost,
                          endEffectorPosPlotHost,
                          endEffectorVelPlotHost,
                          endEffectorAccPlotHost,
                          cableTensionPlotHost,
                          cableSpeedPlotHost,
                          cableLengthPlotHost,
                          actualEndEffectorPosPlotHost,
                          actualEndEffectorVelPlotHost,
                          actualEndEffectorAccPlotHost);
    clearVisualizationData();
}

void MainWindow::clearVisualizationData()
{
    stopPlannedTrajectoryVisualizationTimer();
    resetSimulationDataVisualizationPlayback();
    resetMotorControlInputVisualizationCommand();
    resetCableKinematicVisualizationState();
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        visualizationTimerStarted = false;
        return;
    }
    if(curveDrawer){
        curveDrawer->clearAllDataSignal();
    }
    clearForcePidTuningResponsePlot();
    visualizationTimerStarted = false;
    lastVisualizationPoseTime = -1.0;
    lastVisualizationPose.clear();
    lastVisualizationVelocity.clear();
    lastActualEndEffectorPoseTime = -1.0;
    lastActualEndEffectorPose.clear();
    lastActualEndEffectorVelocity.clear();
    smoothedActualEndEffectorVelocity.clear();
    smoothedActualEndEffectorAcceleration.clear();
    resetPlannedTrajectoryVisualizationCursor();
}

void MainWindow::saveMainWindowScreenshot()
{
    const QString screenshotDirPath =
            QDir(uiEventLogDirPath()).filePath(QStringLiteral("screenshot"));
    QDir screenshotDir(screenshotDirPath);
    if(!screenshotDir.exists() && !screenshotDir.mkpath(QStringLiteral("."))){
        displayInfo(QStringLiteral("错误：无法创建截图目录 %1")
                    .arg(QDir::toNativeSeparators(screenshotDirPath))
                    .toStdString(),
                    "error");
        return;
    }

    const QString fileName = QStringLiteral("screenshot_%1.png")
            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    const QString filePath = screenshotDir.filePath(fileName);
    const QPixmap screenshot = grab();
    if(screenshot.isNull()){
        displayInfo("错误：软件界面截图失败，未生成有效图像", "error");
        return;
    }

    if(!screenshot.save(filePath, "PNG")){
        displayInfo(QStringLiteral("错误：软件界面截图保存失败：%1")
                    .arg(QDir::toNativeSeparators(filePath))
                    .toStdString(),
                    "error");
        return;
    }

    displayInfo(QStringLiteral("软件界面截图已保存：%1")
                .arg(QDir::toNativeSeparators(filePath))
                .toStdString());
}

double MainWindow::visualizationTimeSeconds()
{
    if(!visualizationTimerStarted){
        visualizationElapsedTimer.start();
        visualizationTimerStarted = true;
    }
    return static_cast<double>(visualizationElapsedTimer.elapsed()) / 1000.0;
}

void MainWindow::updateVisualizationFromControlSnapshot(const ControlWorker::Snapshot& snapshot)
{
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        Q_UNUSED(snapshot);
        return;
    }
    if(forcePidTuningExclusiveRefresh && !hybridControlRuntimeRecordingActive){
        return;
    }

    const bool simulationVisualizationLocked =
            simulationDataVisualizationActive ||
            simulationDataVisualizationHoldingFrozenResult;
    const double timeSec = visualizationTimeSeconds();
    if(curveDrawer && !simulationVisualizationLocked && !forcePidTuningExclusiveRefresh){
        curveDrawer->updateCableTensionPlotSignal(timeSec, mapCableTensionForPlot(snapshot.forceSensorValue));
        updateCableKinematicVisualizationFromSnapshot(timeSec, snapshot);
    }
    if((!simulationVisualizationLocked && !forcePidTuningExclusiveRefresh) ||
            hybridControlRuntimeRecordingActive){
        updateActualEndEffectorVisualizationFromSnapshot(timeSec, snapshot);
    }
    if(!forcePidTuningExclusiveRefresh){
        updateForcePidTuningResponsePlot(timeSec,
                                         snapshot.forceSensorValue,
                                         snapshot.expectedForce);
    }
}

void MainWindow::updateVisualizationFromPlatformPose(const std::vector<std::vector<double>>& platformPose)
{
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        Q_UNUSED(platformPose);
        return;
    }
    if(forcePidTuningExclusiveRefresh ||
            simulationDataVisualizationActive ||
            simulationDataVisualizationHoldingFrozenResult ||
            !curveDrawer ||
            platformPose.empty() ||
            platformPose.front().size() < 6){
        return;
    }
    const double timeSec = visualizationTimeSeconds();
    if(shouldUsePlannedTrajectoryVisualization() ||
            isPoseTrajectoryVisualizationWaitingForExecution()){
        return;
    }
    updateVisualizationPoseSample(timeSec, platformPose.front());
}

void MainWindow::handleVisualizationRigidPose(const std::vector<std::vector<double>>& rigidPose)
{
    updateVisualizationFromPlatformPose(rigidPose);
    updateMocapPoseDisplay(rigidPose);
}

void MainWindow::resetPlannedTrajectoryVisualizationCursor()
{
    plannedTrajectoryVisualizationStarted = false;
    plannedTrajectoryVisualizationTimeSec = 0.0;
    plannedTrajectoryVisualizationLastPointIndex = -1;
}

void MainWindow::restartPlannedTrajectoryVisualizationTimer()
{
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        stopPlannedTrajectoryVisualizationTimer();
        return;
    }
    if(!plannedTrajectoryVisualizationTimer || !shouldUsePlannedTrajectoryVisualization()){
        stopPlannedTrajectoryVisualizationTimer();
        return;
    }
    if(activePoseTrajectoryDisplayRunning || activePoseTrajectoryDisplayPointCount() > 0){
        stopPlannedTrajectoryVisualizationTimer();
        return;
    }

    const int intervalMs = poseTrajectoryDisplayTimerIntervalMs();
    if(plannedTrajectoryVisualizationTimer->isActive()){
        plannedTrajectoryVisualizationTimer->stop();
    }
    plannedTrajectoryVisualizationTimer->start(intervalMs);
    updateVisualizationFromPlannedTrajectory(0);
}

void MainWindow::stopPlannedTrajectoryVisualizationTimer()
{
    if(plannedTrajectoryVisualizationTimer && plannedTrajectoryVisualizationTimer->isActive()){
        plannedTrajectoryVisualizationTimer->stop();
    }
}

bool MainWindow::shouldUsePlannedTrajectoryVisualization() const
{
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        return false;
    }
    if(forcePidTuningExclusiveRefresh || !curveDrawer){
        return false;
    }
    return isPoseTrajectoryExecutionVisualizationActive();
}

bool MainWindow::isPoseTrajectoryExecutionVisualizationActive() const
{
    if(activePoseTrajectoryDisplayPointCount() <= 0){
        return false;
    }
    if(activePoseTrajectoryDisplayRunning){
        return true;
    }
    return runtimeState.controlBoxPausedPvtMotion &&
            activePoseTrajectoryDisplayPointIndex >= 0;
}

bool MainWindow::isPoseTrajectoryVisualizationWaitingForExecution() const
{
    if(isPoseTrajectoryExecutionVisualizationActive()){
        return false;
    }
    const bool hasTrajectoryContext =
            poseTrajectoryPointCountForVisualization(activePoseTrajectoryDisplayTraj) > 0 ||
            poseTrajectoryPointCountForVisualization(plannedPoseTrajectoryDisplayTraj) > 0;
    if(!hasTrajectoryContext){
        return false;
    }
    return runtimeState.posModeRunning ||
            runtimeState.pvtCommandActive ||
            runtimeState.pvtMotionProtectedActive;
}

bool MainWindow::sampleActivePoseTrajectoryDisplayPointForVisualization(
        int pointIndex,
        double& trajectoryTimeSec,
        std::vector<double>& pose,
        std::vector<double>& velocity,
        std::vector<double>& acceleration) const
{
    pose.clear();
    velocity.clear();
    acceleration.clear();
    trajectoryTimeSec = 0.0;

    const int pointCount = activePoseTrajectoryDisplayPointCount();
    if(pointCount <= 0 || pointIndex < 0 || pointIndex >= pointCount ||
            activePoseTrajectoryDisplayTraj.empty() ||
            activePoseTrajectoryDisplayTraj.front().empty() ||
            activePoseTrajectoryDisplayTraj.front().front().size() < 6){
        return false;
    }

    std::vector<double> timeAxis;
    if(static_cast<int>(activePoseTrajectoryDisplayTimeStamp.size()) == pointCount &&
            hasFiniteValues(activePoseTrajectoryDisplayTimeStamp, pointCount)){
        timeAxis = activePoseTrajectoryDisplayTimeStamp;
    }
    else{
        timeAxis = poseTrajectoryTimeAxisForVisualization(
                    activePoseTrajectoryDisplayTraj,
                    pointCount,
                    poseTrajectoryStepTimeSecFromUi());
    }
    if(static_cast<int>(timeAxis.size()) != pointCount ||
            !std::isfinite(timeAxis[pointIndex])){
        return false;
    }

    const auto& firstEndTrajectory = activePoseTrajectoryDisplayTraj.front();
    trajectoryTimeSec = timeAxis[pointIndex];
    pose.assign(6, 0.0);
    velocity.assign(6, 0.0);
    acceleration.assign(6, 0.0);
    for(int dim=0; dim<6; ++dim){
        pose[dim] = firstEndTrajectory[0][dim][pointIndex];
        if(firstEndTrajectory.size() > 1 &&
                static_cast<int>(firstEndTrajectory[1].size()) > dim &&
                static_cast<int>(firstEndTrajectory[1][dim].size()) > pointIndex){
            velocity[dim] = firstEndTrajectory[1][dim][pointIndex];
        }
        if(firstEndTrajectory.size() > 2 &&
                static_cast<int>(firstEndTrajectory[2].size()) > dim &&
                static_cast<int>(firstEndTrajectory[2][dim].size()) > pointIndex){
            acceleration[dim] = firstEndTrajectory[2][dim][pointIndex];
        }
    }

    return hasFiniteValues(pose, 6) &&
            hasFiniteValues(velocity, 6) &&
            hasFiniteValues(acceleration, 6);
}

bool MainWindow::updateVisualizationFromCurrentPoseTrajectoryDisplayPoint()
{
    if(forcePidTuningExclusiveRefresh ||
            !curveDrawer ||
            activePoseTrajectoryDisplayPointIndex < 0){
        return false;
    }
    if(activePoseTrajectoryDisplayPointIndex == plannedTrajectoryVisualizationLastPointIndex){
        return true;
    }

    double trajectoryTimeSec = 0.0;
    std::vector<double> pose;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    if(!sampleActivePoseTrajectoryDisplayPointForVisualization(
                activePoseTrajectoryDisplayPointIndex,
                trajectoryTimeSec,
                pose,
                velocity,
                acceleration)){
        return false;
    }

    updateVisualizationPoseSample(trajectoryTimeSec,
                                  pose,
                                  &velocity,
                                  &acceleration);
    plannedTrajectoryVisualizationStarted = true;
    plannedTrajectoryVisualizationTimeSec = trajectoryTimeSec;
    plannedTrajectoryVisualizationLastPointIndex = activePoseTrajectoryDisplayPointIndex;
    return true;
}

bool MainWindow::samplePlannedTrajectoryForVisualization(
        double trajectoryTimeSec,
        std::vector<double>& pose,
        std::vector<double>& velocity,
        std::vector<double>& acceleration) const
{
    pose.clear();
    velocity.clear();
    acceleration.clear();

    const auto* trajectory = &activePoseTrajectoryDisplayTraj;
    int pointCount = poseTrajectoryPointCountForVisualization(*trajectory);
    if(pointCount <= 0){
        trajectory = &plannedPoseTrajectoryDisplayTraj;
        pointCount = poseTrajectoryPointCountForVisualization(*trajectory);
    }
    if(pointCount <= 0 || trajectory->empty()){
        return false;
    }

    const double fallbackStepSec = poseTrajectoryStepTimeSecFromUi();
    std::vector<double> timeAxis;
    if(trajectory == &activePoseTrajectoryDisplayTraj &&
            static_cast<int>(activePoseTrajectoryDisplayTimeStamp.size()) == pointCount &&
            hasFiniteValues(activePoseTrajectoryDisplayTimeStamp, pointCount)){
        timeAxis = activePoseTrajectoryDisplayTimeStamp;
    }
    else{
        timeAxis = poseTrajectoryTimeAxisForVisualization(*trajectory, pointCount, fallbackStepSec);
    }
    const int pointIndex = nearestPoseTrajectoryPointIndex(timeAxis, trajectoryTimeSec);
    if(pointIndex < 0 || pointIndex >= pointCount){
        return false;
    }

    const auto& firstEndTrajectory = trajectory->front();
    pose.assign(6, 0.0);
    velocity.assign(6, 0.0);
    acceleration.assign(6, 0.0);
    for(int dim=0; dim<6; ++dim){
        pose[dim] = firstEndTrajectory[0][dim][pointIndex];
        if(firstEndTrajectory.size() > 1 &&
                static_cast<int>(firstEndTrajectory[1].size()) > dim &&
                static_cast<int>(firstEndTrajectory[1][dim].size()) > pointIndex){
            velocity[dim] = firstEndTrajectory[1][dim][pointIndex];
        }
        if(firstEndTrajectory.size() > 2 &&
                static_cast<int>(firstEndTrajectory[2].size()) > dim &&
                static_cast<int>(firstEndTrajectory[2][dim].size()) > pointIndex){
            acceleration[dim] = firstEndTrajectory[2][dim][pointIndex];
        }
    }

    return hasFiniteValues(pose, 6) &&
            hasFiniteValues(velocity, 6) &&
            hasFiniteValues(acceleration, 6);
}

void MainWindow::updateVisualizationFromPlannedTrajectory(int refreshIntervalMs)
{
    if(!shouldUsePlannedTrajectoryVisualization()){
        if(plannedTrajectoryVisualizationStarted){
            lastVisualizationPoseTime = -1.0;
            lastVisualizationPose.clear();
            lastVisualizationVelocity.clear();
        }
        resetPlannedTrajectoryVisualizationCursor();
        return;
    }
    if(runtimeState.controlBoxPausedPvtMotion){
        return;
    }

    const auto* trajectory = &activePoseTrajectoryDisplayTraj;
    int pointCount = poseTrajectoryPointCountForVisualization(*trajectory);
    if(pointCount <= 0){
        trajectory = &plannedPoseTrajectoryDisplayTraj;
        pointCount = poseTrajectoryPointCountForVisualization(*trajectory);
    }
    if(pointCount <= 0){
        stopPlannedTrajectoryVisualizationTimer();
        resetPlannedTrajectoryVisualizationCursor();
        return;
    }

    const std::vector<double> timeAxis =
            poseTrajectoryTimeAxisForVisualization(*trajectory,
                                                   pointCount,
                                                   poseTrajectoryStepTimeSecFromUi());
    if(static_cast<int>(timeAxis.size()) != pointCount || timeAxis.empty()){
        stopPlannedTrajectoryVisualizationTimer();
        resetPlannedTrajectoryVisualizationCursor();
        return;
    }

    const double intervalSec = static_cast<double>(std::max(0, refreshIntervalMs)) / 1000.0;
    if(!plannedTrajectoryVisualizationStarted){
        plannedTrajectoryVisualizationStarted = true;
        plannedTrajectoryVisualizationTimeSec = timeAxis.front();
    }
    else{
        plannedTrajectoryVisualizationTimeSec += intervalSec;
    }

    if(plannedTrajectoryVisualizationTimeSec > timeAxis.back()){
        plannedTrajectoryVisualizationTimeSec = timeAxis.back();
    }

    const int pointIndex =
            nearestPoseTrajectoryPointIndex(timeAxis, plannedTrajectoryVisualizationTimeSec);
    if(pointIndex < 0){
        return;
    }
    if(pointIndex == plannedTrajectoryVisualizationLastPointIndex &&
            plannedTrajectoryVisualizationTimeSec < timeAxis.back()){
        return;
    }

    std::vector<double> pose;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    if(!samplePlannedTrajectoryForVisualization(plannedTrajectoryVisualizationTimeSec,
                                                pose,
                                                velocity,
                                                acceleration)){
        return;
    }

    updateVisualizationPoseSample(plannedTrajectoryVisualizationTimeSec,
                                  pose,
                                  &velocity,
                                  &acceleration);
    plannedTrajectoryVisualizationLastPointIndex = pointIndex;
    if(plannedTrajectoryVisualizationTimeSec >= timeAxis.back()){
        stopPlannedTrajectoryVisualizationTimer();
    }
}

void MainWindow::updateVisualizationPoseSample(double timeSec,
                                               const std::vector<double>& pose,
                                               const std::vector<double>* velocity,
                                               const std::vector<double>* acceleration)
{
    if(forcePidTuningExclusiveRefresh || !curveDrawer || pose.size() < 6){
        return;
    }

    if(!visualizationTimerStarted){
        visualizationElapsedTimer.start();
        visualizationTimerStarted = true;
    }

    std::vector<double> velocityValue(6, 0.0);
    std::vector<double> accelerationValue(6, 0.0);

    if(velocity && velocity->size() >= 6){
        velocityValue.assign(velocity->begin(), velocity->begin() + 6);
        if(acceleration && acceleration->size() >= 6){
            accelerationValue.assign(acceleration->begin(), acceleration->begin() + 6);
        }
        else if(lastVisualizationPoseTime >= 0.0 &&
                lastVisualizationVelocity.size() >= 6 &&
                timeSec > lastVisualizationPoseTime){
            const double dt = timeSec - lastVisualizationPoseTime;
            for(int i=0; i<6; ++i){
                accelerationValue[i] = (velocityValue[i] - lastVisualizationVelocity[i]) / dt;
            }
        }
    }
    else if(lastVisualizationPoseTime >= 0.0 &&
            lastVisualizationPose.size() >= 6 &&
            timeSec > lastVisualizationPoseTime){
        const double dt = timeSec - lastVisualizationPoseTime;
        for(int i=0; i<6; ++i){
            velocityValue[i] = (pose[i] - lastVisualizationPose[i]) / dt;
        }
        if(lastVisualizationVelocity.size() >= 6){
            for(int i=0; i<6; ++i){
                accelerationValue[i] = (velocityValue[i] - lastVisualizationVelocity[i]) / dt;
            }
        }
    }

    QVector<double> positionVec(3, 0.0);
    QVector<double> orientationVec(3, 0.0);
    QVector<double> velocityVec(3, 0.0);
    QVector<double> angularVelocityVec(3, 0.0);
    QVector<double> accelerationVec(3, 0.0);
    QVector<double> angularAccelerationVec(3, 0.0);
    for(int i=0; i<3; ++i){
        positionVec[i] = pose[i];
        orientationVec[i] = pose[i + 3];
        velocityVec[i] = velocityValue[i];
        angularVelocityVec[i] = velocityValue[i + 3];
        accelerationVec[i] = accelerationValue[i];
        angularAccelerationVec[i] = accelerationValue[i + 3];
    }

    curveDrawer->updateEndEffectorPosPlotSignal(timeSec, positionVec, orientationVec);
    curveDrawer->updateEndEffectorVelPlotSignal(timeSec, velocityVec, angularVelocityVec);
    curveDrawer->updateEndEffectorAccPlotSignal(timeSec, accelerationVec, angularAccelerationVec);

    lastVisualizationPose.assign(pose.begin(), pose.begin() + 6);
    lastVisualizationVelocity = velocityValue;
    lastVisualizationPoseTime = timeSec;
}

void MainWindow::resetSimulationDataVisualizationPlayback()
{
    ++simulationDataVisualizationPlaybackToken;
    if(simulationDataVisualizationTimer &&
            simulationDataVisualizationTimer->isActive()){
        simulationDataVisualizationTimer->stop();
    }
    simulationDataVisualizationActive = false;
    simulationDataVisualizationHoldingFrozenResult = false;
    simulationDataVisualizationStarted = false;
    simulationDataVisualizationTimeSec = 0.0;
    simulationDataVisualizationPointIndex = -1;
    simulationDataVisualizationTrajectory.clear();
    simulationDataVisualizationTimeStamp.clear();
    simulationDataVisualizationCableLengthTraj.clear();
    simulationDataVisualizationCableForceTraj.clear();
    lastSimulationCableLengthVisualizationValues.clear();
    lastSimulationCableKinematicVisualizationTimeSec = -1.0;
}

bool MainWindow::startSimulationDataVisualizationPlayback(
        const std::vector<std::vector<std::vector<std::vector<double>>>>& trajectory,
        const std::vector<double>& timeStamp,
        const std::vector<std::vector<double>>& cableLengthTraj,
        const std::vector<std::vector<double>>& cableForceTraj)
{
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        return true;
    }
    if(!curveDrawer || !simulationDataVisualizationTimer){
        return false;
    }

    const int trajectoryPointCount =
            poseTrajectoryPointCountForVisualization(trajectory);
    if(trajectoryPointCount <= 0 ||
            cableLengthTraj.empty() ||
            cableForceTraj.empty()){
        return false;
    }

    std::vector<double> resolvedTimeAxis;
    if(static_cast<int>(timeStamp.size()) == trajectoryPointCount &&
            hasFiniteValues(timeStamp, trajectoryPointCount)){
        resolvedTimeAxis = timeStamp;
    }
    else{
        resolvedTimeAxis =
                poseTrajectoryTimeAxisForVisualization(
                    trajectory,
                    trajectoryPointCount,
                    poseTrajectoryStepTimeSecFromUi());
    }
    if(static_cast<int>(resolvedTimeAxis.size()) != trajectoryPointCount ||
            resolvedTimeAxis.empty()){
        return false;
    }

    int cableForcePointCount = std::numeric_limits<int>::max();
    for(const std::vector<double>& row : cableForceTraj){
        if(row.empty()){
            return false;
        }
        cableForcePointCount =
                std::min(cableForcePointCount, static_cast<int>(row.size()));
    }
    if(cableForcePointCount == std::numeric_limits<int>::max()){
        return false;
    }

    const int pointCount =
            std::min({trajectoryPointCount,
                      static_cast<int>(resolvedTimeAxis.size()),
                      static_cast<int>(cableLengthTraj.size()),
                      cableForcePointCount});
    if(pointCount <= 0){
        return false;
    }

    auto hasFinitePrefix = [](const std::vector<double>& values, int count) -> bool {
        if(static_cast<int>(values.size()) < count){
            return false;
        }
        for(int i=0; i<count; ++i){
            if(!std::isfinite(values[i])){
                return false;
            }
        }
        return true;
    };

    resolvedTimeAxis.resize(pointCount);
    for(int pointIndex=0; pointIndex<pointCount; ++pointIndex){
        if(!std::isfinite(resolvedTimeAxis[pointIndex]) ||
                (pointIndex > 0 &&
                 resolvedTimeAxis[pointIndex] <= resolvedTimeAxis[pointIndex - 1])){
            return false;
        }
        if(cableLengthTraj[pointIndex].empty() ||
                !hasFiniteValues(cableLengthTraj[pointIndex])){
            return false;
        }
    }
    for(const std::vector<double>& row : cableForceTraj){
        if(!hasFinitePrefix(row, pointCount)){
            return false;
        }
    }

    clearVisualizationData();
    if(dataVizFreezeCurvesCheckBox &&
            dataVizFreezeCurvesCheckBox->isChecked()){
        QSignalBlocker blocker(dataVizFreezeCurvesCheckBox);
        dataVizFreezeCurvesCheckBox->setChecked(false);
    }
    updateDataVisualizationRefreshEnabled();

    simulationDataVisualizationTrajectory = trajectory;
    simulationDataVisualizationTimeStamp = resolvedTimeAxis;
    simulationDataVisualizationCableLengthTraj = cableLengthTraj;
    simulationDataVisualizationCableForceTraj = cableForceTraj;
    simulationDataVisualizationActive = true;
    simulationDataVisualizationHoldingFrozenResult = false;
    simulationDataVisualizationStarted = false;
    simulationDataVisualizationTimeSec = resolvedTimeAxis.front();
    simulationDataVisualizationPointIndex = -1;
    lastSimulationCableLengthVisualizationValues.clear();
    lastSimulationCableKinematicVisualizationTimeSec = -1.0;
    ++simulationDataVisualizationPlaybackToken;

    simulationDataVisualizationTimer->start(simulationDataVisualizationTimerIntervalMs());
    updateSimulationDataVisualizationPlayback(0);
    return true;
}

void MainWindow::updateSimulationDataVisualizationPlayback(int refreshIntervalMs)
{
    if(!curveDrawer){
        resetSimulationDataVisualizationPlayback();
        return;
    }
    if(!simulationDataVisualizationActive){
        if(simulationDataVisualizationTimer &&
                simulationDataVisualizationTimer->isActive()){
            simulationDataVisualizationTimer->stop();
        }
        return;
    }

    int cableForcePointCount = std::numeric_limits<int>::max();
    for(const std::vector<double>& row : simulationDataVisualizationCableForceTraj){
        if(row.empty()){
            cableForcePointCount = 0;
            break;
        }
        cableForcePointCount =
                std::min(cableForcePointCount, static_cast<int>(row.size()));
    }
    if(cableForcePointCount == std::numeric_limits<int>::max()){
        cableForcePointCount = 0;
    }

    const int pointCount =
            std::min({poseTrajectoryPointCountForVisualization(
                          simulationDataVisualizationTrajectory),
                      static_cast<int>(simulationDataVisualizationTimeStamp.size()),
                      static_cast<int>(simulationDataVisualizationCableLengthTraj.size()),
                      cableForcePointCount});
    if(pointCount <= 0){
        resetSimulationDataVisualizationPlayback();
        return;
    }

    const double intervalSec =
            static_cast<double>(std::max(0, refreshIntervalMs)) / 1000.0;
    if(!simulationDataVisualizationStarted){
        simulationDataVisualizationStarted = true;
        simulationDataVisualizationTimeSec =
                simulationDataVisualizationTimeStamp.front();
    }
    else{
        simulationDataVisualizationTimeSec += intervalSec;
    }

    simulationDataVisualizationTimeSec =
            std::max(simulationDataVisualizationTimeStamp.front(),
                     std::min(simulationDataVisualizationTimeSec,
                              simulationDataVisualizationTimeStamp[pointCount - 1]));

    const int pointIndex =
            nearestPoseTrajectoryPointIndex(simulationDataVisualizationTimeStamp,
                                            simulationDataVisualizationTimeSec);
    if(pointIndex < 0 || pointIndex >= pointCount){
        resetSimulationDataVisualizationPlayback();
        return;
    }
    if(pointIndex == simulationDataVisualizationPointIndex &&
            simulationDataVisualizationTimeSec <
            simulationDataVisualizationTimeStamp[pointCount - 1]){
        return;
    }

    std::vector<double> pose;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    QVector<double> cableLength;
    QVector<double> cableSpeed;
    QVector<double> cableTension;
    if(!sampleSimulationDataVisualizationPoint(pointIndex,
                                               pose,
                                               velocity,
                                               acceleration,
                                               cableLength,
                                               cableSpeed,
                                               cableTension)){
        resetSimulationDataVisualizationPlayback();
        displayInfo("警告：仿真数据可视化回放遇到无效数据，已停止回放", "warning");
        return;
    }

    const double timeSec = simulationDataVisualizationTimeStamp[pointIndex];
    updateVisualizationPoseSample(timeSec, pose, &velocity, &acceleration);
    curveDrawer->updateCableLengthPlotSignal(timeSec, cableLength);
    curveDrawer->updateCableSpeedPlotSignal(timeSec, cableSpeed);
    curveDrawer->updateCableTensionPlotSignal(timeSec, cableTension);
    simulationDataVisualizationPointIndex = pointIndex;

    if(timeSec >= simulationDataVisualizationTimeStamp[pointCount - 1] ||
            pointIndex + 1 >= pointCount){
        if(simulationDataVisualizationTimer &&
                simulationDataVisualizationTimer->isActive()){
            simulationDataVisualizationTimer->stop();
        }
        simulationDataVisualizationActive = false;
        simulationDataVisualizationHoldingFrozenResult = true;
        const quint64 token = simulationDataVisualizationPlaybackToken;
        const int freezeDelayMs =
                std::max(20, simulationDataVisualizationTimerIntervalMs());
        QTimer::singleShot(freezeDelayMs, this, [this, token](){
            if(token != simulationDataVisualizationPlaybackToken ||
                    simulationDataVisualizationActive){
                return;
            }
            if(curveDrawer){
                curveDrawer->setRefreshEnabled(true);
            }
            if(dataVizFreezeCurvesCheckBox &&
                    !dataVizFreezeCurvesCheckBox->isChecked()){
                dataVizFreezeCurvesCheckBox->setChecked(true);
            }
            else{
                updateDataVisualizationRefreshEnabled();
            }
        });
    }
}

int MainWindow::simulationDataVisualizationTimerIntervalMs() const
{
    double intervalSec = poseTrajectoryStepTimeSecFromUi(0.0);
    auto applyMinPositiveTimeStep = [&intervalSec](const std::vector<double>& timeAxis){
        if(timeAxis.size() < 2){
            return;
        }
        double minPositiveDt = std::numeric_limits<double>::max();
        for(std::size_t i=1; i<timeAxis.size(); ++i){
            const double dt = timeAxis[i] - timeAxis[i - 1];
            if(dt > 1e-9 && dt < minPositiveDt){
                minPositiveDt = dt;
            }
        }
        if(minPositiveDt < std::numeric_limits<double>::max()){
            intervalSec = minPositiveDt;
        }
    };

    if(simulationDataVisualizationTimeStamp.size() >= 2){
        applyMinPositiveTimeStep(simulationDataVisualizationTimeStamp);
    }
    else{
        const int pointCount =
                poseTrajectoryPointCountForVisualization(
                    simulationDataVisualizationTrajectory);
        if(pointCount > 0){
            applyMinPositiveTimeStep(
                        poseTrajectoryTimeAxisForVisualization(
                            simulationDataVisualizationTrajectory,
                            pointCount,
                            intervalSec));
        }
    }

    if(intervalSec <= 0.0 || !std::isfinite(intervalSec)){
        intervalSec = 0.01;
    }
    return std::max(1, static_cast<int>(std::llround(intervalSec * 1000.0)));
}

bool MainWindow::sampleSimulationDataVisualizationPoint(
        int pointIndex,
        std::vector<double>& pose,
        std::vector<double>& velocity,
        std::vector<double>& acceleration,
        QVector<double>& cableLength,
        QVector<double>& cableSpeed,
        QVector<double>& cableTension)
{
    const int pointCount =
            std::min(static_cast<int>(simulationDataVisualizationTimeStamp.size()),
                     static_cast<int>(simulationDataVisualizationCableLengthTraj.size()));
    if(pointIndex < 0 ||
            pointIndex >= pointCount ||
            simulationDataVisualizationTrajectory.empty() ||
            simulationDataVisualizationTrajectory.front().empty()){
        return false;
    }

    const auto& firstEndTrajectory = simulationDataVisualizationTrajectory.front();
    if(firstEndTrajectory.empty() ||
            firstEndTrajectory.front().size() < 6){
        return false;
    }

    pose.assign(6, 0.0);
    velocity.assign(6, 0.0);
    acceleration.assign(6, 0.0);
    for(int dim=0; dim<6; ++dim){
        if(pointIndex >= static_cast<int>(firstEndTrajectory[0][dim].size())){
            return false;
        }
        pose[dim] = firstEndTrajectory[0][dim][pointIndex];
        if(firstEndTrajectory.size() > 1 &&
                static_cast<int>(firstEndTrajectory[1].size()) > dim &&
                pointIndex < static_cast<int>(firstEndTrajectory[1][dim].size())){
            velocity[dim] = firstEndTrajectory[1][dim][pointIndex];
        }
        if(firstEndTrajectory.size() > 2 &&
                static_cast<int>(firstEndTrajectory[2].size()) > dim &&
                pointIndex < static_cast<int>(firstEndTrajectory[2][dim].size())){
            acceleration[dim] = firstEndTrajectory[2][dim][pointIndex];
        }
    }
    if(!hasFiniteValues(pose, 6) ||
            !hasFiniteValues(velocity, 6) ||
            !hasFiniteValues(acceleration, 6)){
        return false;
    }

    cableLength =
            mapSimulationCablePointForPlot(
                simulationDataVisualizationCableLengthTraj[pointIndex]);
    cableSpeed = QVector<double>(cableLength.size(), 0.0);
    int referencePointIndex = -1;
    double derivativeSign = 1.0;
    if(pointIndex > 0){
        referencePointIndex = pointIndex - 1;
    }
    else if(pointIndex + 1 < pointCount){
        referencePointIndex = pointIndex + 1;
        derivativeSign = -1.0;
    }
    if(referencePointIndex >= 0 &&
            referencePointIndex < pointCount){
        const QVector<double> referenceLength =
                mapSimulationCablePointForPlot(
                    simulationDataVisualizationCableLengthTraj[referencePointIndex]);
        const double dt =
                simulationDataVisualizationTimeStamp[pointIndex] -
                simulationDataVisualizationTimeStamp[referencePointIndex];
        if(std::fabs(dt) > 1e-9 &&
                referenceLength.size() == cableLength.size()){
            for(int i=0; i<cableLength.size(); ++i){
                if(std::isfinite(cableLength[i]) &&
                        std::isfinite(referenceLength[i])){
                    cableSpeed[i] =
                            derivativeSign *
                            (cableLength[i] - referenceLength[i]) /
                            std::fabs(dt);
                }
            }
        }
    }
    cableTension = mapSimulationCableForcePointForPlot(pointIndex);

    lastSimulationCableLengthVisualizationValues = cableLength;
    lastSimulationCableKinematicVisualizationTimeSec =
            simulationDataVisualizationTimeStamp[pointIndex];
    return true;
}

QVector<double> MainWindow::mapSimulationCablePointForPlot(
        const std::vector<double>& cablePoint) const
{
    QVector<double> values(8, 0.0);
    if(!ui || cablePoint.empty()){
        return values;
    }

    const int endCount = std::max(0, ui->devEndNum->value());
    std::vector<int> endSourceBase(endCount, 0);
    int sourceBase = 0;
    for(int endIndex=0; endIndex<endCount; ++endIndex){
        endSourceBase[endIndex] = sourceBase;
        for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
            if(isModeledMotorAxis(axisIndex) &&
                    axisEndVec[axisIndex]->value() == endIndex){
                ++sourceBase;
            }
        }
    }

    std::vector<int> endSourceOffset(endCount, 0);
    int channelIndex = 0;
    for(int axisIndex=0;
        axisIndex<ui->devAxisNum->value() && channelIndex<values.size();
        ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        const int endIndex = axisEndVec[axisIndex]->value();
        int sourceIndex = -1;
        if(endIndex >= 0 && endIndex < endCount){
            sourceIndex = endSourceBase[endIndex] + endSourceOffset[endIndex];
            ++endSourceOffset[endIndex];
        }
        if(sourceIndex >= 0 &&
                sourceIndex < static_cast<int>(cablePoint.size()) &&
                std::isfinite(cablePoint[sourceIndex])){
            values[channelIndex] = cablePoint[sourceIndex];
        }
        ++channelIndex;
    }
    return values;
}

QVector<double> MainWindow::mapSimulationCableForcePointForPlot(int pointIndex) const
{
    std::vector<double> cablePoint;
    cablePoint.reserve(simulationDataVisualizationCableForceTraj.size());
    for(const std::vector<double>& cableForce : simulationDataVisualizationCableForceTraj){
        if(pointIndex >= 0 &&
                pointIndex < static_cast<int>(cableForce.size()) &&
                std::isfinite(cableForce[pointIndex])){
            cablePoint.push_back(cableForce[pointIndex]);
        }
        else{
            cablePoint.push_back(0.0);
        }
    }
    return mapSimulationCablePointForPlot(cablePoint);
}

void MainWindow::updateActualEndEffectorVisualizationFromSnapshot(
        double timeSec,
        const ControlWorker::Snapshot& snapshot)
{
    const bool plotEnabled =
            !forcePidTuningExclusiveRefresh && curveDrawer && std::isfinite(timeSec);
    if(!plotEnabled && !hybridControlRuntimeRecordingActive){
        return;
    }

    const auto captureUnavailableSample = [this, timeSec, &snapshot](){
        if(hybridControlRuntimeRecordingActive){
            captureHybridControlRuntimeRecordSample(
                        timeSec,
                        snapshot,
                        ForwardKinematicsSolver::Result{});
        }
    };

    std::vector<std::vector<double>> cableLength;
    bool hasCableLength = buildCurrentCableLengthSnapshot(cableLength);
    if(!hasCableLength){
        QVector<double> flatCableLength;
        hasCableLength =
                buildMotorPositionCableLengthForVisualization(snapshot.motorAbsPos,
                                                              false,
                                                              flatCableLength);
        if(!hasCableLength && ui && ui->devUseLS && ui->devUseLS->isChecked() &&
                hardwareInterface.isLSConnected()){
            hasCableLength =
                    buildEncoderCableLengthForVisualization(
                        hardwareInterface.getAllMotorEncoderPosUnit(),
                        flatCableLength);
        }
        if(hasCableLength){
            cableLength.assign(ui->devEndNum->value(), {});
            int channelIndex = 0;
            for(int axisIndex=0;
                axisIndex<ui->devAxisNum->value() && channelIndex<flatCableLength.size();
                ++axisIndex){
                if(!isModeledMotorAxis(axisIndex)){
                    continue;
                }
                const int endIndex = axisEndVec[axisIndex]->value();
                if(endIndex < 0 || endIndex >= ui->devEndNum->value()){
                    captureUnavailableSample();
                    return;
                }
                cableLength[endIndex].push_back(flatCableLength[channelIndex]);
                ++channelIndex;
            }
        }
    }
    if(!hasCableLength || cableLength.empty()){
        captureUnavailableSample();
        return;
    }

    const std::vector<std::vector<std::vector<double>>> contactPointByEnd = buildCableContactPointPos();
    const std::vector<std::vector<std::vector<double>>> anchorPosByEnd =
            splitAnchorPositionsByEnd(buildFixedAnchorHome());
    if(contactPointByEnd.empty() || anchorPosByEnd.empty()){
        captureUnavailableSample();
        return;
    }

    const int endIndex = 0;
    if(endIndex >= static_cast<int>(cableLength.size()) ||
            endIndex >= static_cast<int>(contactPointByEnd.size()) ||
            endIndex >= static_cast<int>(anchorPosByEnd.size())){
        captureUnavailableSample();
        return;
    }

    ForwardKinematicsSolver::Request request;
    request.anchorPos = anchorPosByEnd[endIndex];
    request.contactPointLocal = contactPointByEnd[endIndex];
    request.cableLength = cableLength[endIndex];
    request.excludedCableIndices = currentForceControlledCableIndicesForEnd(endIndex);
    request.pulleyRadius = buildPulleyRadius();
    request.initialPose = actualEndEffectorKinematicsSolver.initialPose();
    if(!hasFiniteValues(request.initialPose, 6)){
        std::vector<double> currentPose;
        if(currentForwardKinematicsEndPose(currentPose, 500) && hasFiniteValues(currentPose, 6)){
            request.initialPose = std::vector<double>(currentPose.begin(), currentPose.begin() + 6);
        }
        else{
            request.initialPose = configuredCableHomePose();
        }
    }
    request.keepRotation = true;
    applyForwardKinematicsBoundsForCurrentTemplate(request);

    const ForwardKinematicsSolver::Result result =
            actualEndEffectorKinematicsSolver.solve(request);
    if(hybridControlRuntimeRecordingActive){
        captureHybridControlRuntimeRecordSample(timeSec, snapshot, result);
    }
    if(!result.success || result.pose.size() < 6){
        return;
    }

    if(plotEnabled){
        updateActualEndEffectorVisualizationSample(timeSec, result.pose);
    }
}

void MainWindow::updateActualEndEffectorVisualizationSample(double timeSec,
                                                            const std::vector<double>& pose)
{
    if(!curveDrawer || pose.size() < 6){
        return;
    }

    std::vector<double> velocity(6, 0.0);
    std::vector<double> acceleration(6, 0.0);
    double derivativeDtSec = 0.0;
    bool hasVelocitySample = false;
    if(lastActualEndEffectorPoseTime >= 0.0 &&
            lastActualEndEffectorPose.size() >= 6 &&
            timeSec > lastActualEndEffectorPoseTime){
        const double dt = timeSec - lastActualEndEffectorPoseTime;
        derivativeDtSec = dt;
        hasVelocitySample = true;
        for(int i=0; i<6; ++i){
            velocity[i] = (pose[i] - lastActualEndEffectorPose[i]) / dt;
        }
    }

    const bool smoothVelocity =
            actualEndEffectorVelocitySmoothCheckBox &&
            actualEndEffectorVelocitySmoothCheckBox->isChecked();
    const bool smoothAcceleration =
            actualEndEffectorAccelerationSmoothCheckBox &&
            actualEndEffectorAccelerationSmoothCheckBox->isChecked();
    const std::vector<double> displayVelocity =
            smoothVisualizationVector(velocity,
                                      smoothedActualEndEffectorVelocity,
                                      smoothVelocity,
                                      derivativeDtSec,
                                      kActualEndEffectorVelocitySmoothingTauSec);
    if(hasVelocitySample && lastActualEndEffectorVelocity.size() >= 6){
        for(int i=0; i<6; ++i){
            acceleration[i] = (displayVelocity[i] - lastActualEndEffectorVelocity[i]) /
                    derivativeDtSec;
        }
    }
    const std::vector<double> displayAcceleration =
            smoothVisualizationVector(acceleration,
                                      smoothedActualEndEffectorAcceleration,
                                      smoothAcceleration,
                                      derivativeDtSec,
                                      kActualEndEffectorAccelerationSmoothingTauSec);

    QVector<double> positionVec(3, 0.0);
    QVector<double> orientationVec(3, 0.0);
    QVector<double> velocityVec(3, 0.0);
    QVector<double> angularVelocityVec(3, 0.0);
    QVector<double> accelerationVec(3, 0.0);
    QVector<double> angularAccelerationVec(3, 0.0);
    for(int i=0; i<3; ++i){
        positionVec[i] = pose[i];
        orientationVec[i] = pose[i + 3];
        velocityVec[i] = displayVelocity[i];
        angularVelocityVec[i] = displayVelocity[i + 3];
        accelerationVec[i] = displayAcceleration[i];
        angularAccelerationVec[i] = displayAcceleration[i + 3];
    }

    curveDrawer->updateActualEndEffectorPosPlotSignal(timeSec, positionVec, orientationVec);
    curveDrawer->updateActualEndEffectorVelPlotSignal(timeSec, velocityVec, angularVelocityVec);
    curveDrawer->updateActualEndEffectorAccPlotSignal(timeSec, accelerationVec, angularAccelerationVec);

    lastActualEndEffectorPose.assign(pose.begin(), pose.begin() + 6);
    lastActualEndEffectorVelocity = displayVelocity;
    lastActualEndEffectorPoseTime = timeSec;
}

void MainWindow::resetMotorControlInputVisualizationCommand()
{
    if(motorControlInputVisualizationTimer &&
            motorControlInputVisualizationTimer->isActive()){
        motorControlInputVisualizationTimer->stop();
    }
    motorControlInputVisualizationMotorIndex.clear();
    motorControlInputVisualizationTimeStamp.clear();
    motorControlInputVisualizationPositionUnit.clear();
    motorControlInputVisualizationStarted = false;
    motorControlInputVisualizationTimeSec = 0.0;
    motorControlInputVisualizationPointIndex = -1;
    if(curveDrawer &&
            (!runtimeState.hybridPoseForceModeActive ||
             !activeHybridPoseForceConfig.enabled)){
        curveDrawer->setMotorControlHybridMode(false, QVector<int>());
    }
}

void MainWindow::startMotorControlInputVisualizationFromPvtCommand(
        const std::vector<int>& motorIndex,
        const std::vector<std::vector<double>>& positionUnit,
        const std::vector<double>& timeStamp)
{
    resetMotorControlInputVisualizationCommand();
    if(!RuntimeFeatureSwitches::kDataVisualizationEnabled){
        return;
    }
    if(motorIndex.empty() ||
            positionUnit.empty() ||
            timeStamp.empty() ||
            positionUnit.size() != timeStamp.size()){
        return;
    }

    const int axisCount = static_cast<int>(motorIndex.size());
    for(int pointIndex = 0; pointIndex < static_cast<int>(positionUnit.size()); ++pointIndex){
        if(static_cast<int>(positionUnit[pointIndex].size()) != axisCount ||
                !hasFiniteValues(positionUnit[pointIndex]) ||
                !std::isfinite(timeStamp[pointIndex]) ||
                (pointIndex > 0 && timeStamp[pointIndex] <= timeStamp[pointIndex - 1])){
            return;
        }
    }

    motorControlInputVisualizationMotorIndex = motorIndex;
    motorControlInputVisualizationTimeStamp = timeStamp;
    motorControlInputVisualizationPositionUnit = positionUnit;
    motorControlInputVisualizationStarted = false;
    motorControlInputVisualizationTimeSec = timeStamp.front();
    motorControlInputVisualizationPointIndex = -1;
    if(curveDrawer){
        const QVector<int> forceGraphIndexes =
                motorControlHybridForceGraphIndexesForPlot();
        curveDrawer->setMotorControlHybridMode(!forceGraphIndexes.isEmpty(),
                                               forceGraphIndexes);
    }
    if(motorControlInputVisualizationTimer){
        motorControlInputVisualizationTimer->start(poseTrajectoryDisplayTimerIntervalMs());
    }
    updateMotorControlInputVisualizationFromPvtCommand(0);
}

void MainWindow::updateMotorControlInputVisualizationFromPvtCommand(int refreshIntervalMs)
{
    if(!curveDrawer || !hasActiveMotorControlInputVisualizationCommand()){
        resetMotorControlInputVisualizationCommand();
        return;
    }
    if(isPoseTrajectoryVisualizationWaitingForExecution() ||
            runtimeState.controlBoxPausedPvtMotion){
        return;
    }

    const int pointCount =
            std::min(static_cast<int>(motorControlInputVisualizationTimeStamp.size()),
                     static_cast<int>(motorControlInputVisualizationPositionUnit.size()));

    double trajectoryTimeSec = motorControlInputVisualizationTimeSec;
    int pointIndex = -1;
    double pvtProgressTimeSec = 0.0;
    std::size_t pvtProgressIndex = 0;
    if(hardwareInterface.hasPvtTrajectory() &&
            hardwareInterface.currentPvtProgress(pvtProgressTimeSec, pvtProgressIndex) &&
            std::isfinite(pvtProgressTimeSec)){
        trajectoryTimeSec = pvtProgressTimeSec;
        pointIndex = nearestPoseTrajectoryPointIndex(
                    motorControlInputVisualizationTimeStamp,
                    trajectoryTimeSec);
        if(pointIndex < 0 &&
                pvtProgressIndex < static_cast<std::size_t>(pointCount)){
            pointIndex = static_cast<int>(pvtProgressIndex);
        }
        motorControlInputVisualizationStarted = true;
    }
    else if(!motorControlInputVisualizationStarted){
        motorControlInputVisualizationStarted = true;
        trajectoryTimeSec = motorControlInputVisualizationTimeStamp.front();
        pointIndex = 0;
    }
    else{
        const double intervalSec =
                static_cast<double>(std::max(0, refreshIntervalMs)) / 1000.0;
        trajectoryTimeSec += intervalSec;
        pointIndex = nearestPoseTrajectoryPointIndex(
                    motorControlInputVisualizationTimeStamp,
                    trajectoryTimeSec);
    }

    if(!std::isfinite(trajectoryTimeSec)){
        resetMotorControlInputVisualizationCommand();
        return;
    }
    trajectoryTimeSec = std::max(motorControlInputVisualizationTimeStamp.front(),
                                 std::min(trajectoryTimeSec,
                                          motorControlInputVisualizationTimeStamp.back()));
    motorControlInputVisualizationTimeSec = trajectoryTimeSec;

    if(pointIndex < 0 || pointIndex >= pointCount){
        pointIndex = nearestPoseTrajectoryPointIndex(
                    motorControlInputVisualizationTimeStamp,
                    trajectoryTimeSec);
    }
    if(pointIndex < 0 || pointIndex >= pointCount){
        resetMotorControlInputVisualizationCommand();
        return;
    }
    if(pointIndex == motorControlInputVisualizationPointIndex &&
            trajectoryTimeSec < motorControlInputVisualizationTimeStamp.back()){
        return;
    }

    const std::vector<double>& row =
            motorControlInputVisualizationPositionUnit[pointIndex];
    if(static_cast<int>(row.size()) !=
            static_cast<int>(motorControlInputVisualizationMotorIndex.size()) ||
            !hasFiniteValues(row)){
        resetMotorControlInputVisualizationCommand();
        return;
    }

    const bool hybridMotorControlPlot =
            runtimeState.hybridPoseForceModeActive &&
            activeHybridPoseForceConfig.enabled;
    const QVector<double> plotValues =
            hybridMotorControlPlot ?
                mapHybridMotorControlInputForPlot(
                    motorControlInputVisualizationMotorIndex,
                    row,
                    pointIndex) :
                mapMotorPositionCommandForPlot(
                    motorControlInputVisualizationMotorIndex,
                    row);

    curveDrawer->updateMotorControlPlotSignal(trajectoryTimeSec, plotValues);
    motorControlInputVisualizationPointIndex = pointIndex;

    if(trajectoryTimeSec >= motorControlInputVisualizationTimeStamp.back() ||
            pointIndex + 1 >= pointCount){
        if(motorControlInputVisualizationTimer &&
                motorControlInputVisualizationTimer->isActive()){
            motorControlInputVisualizationTimer->stop();
        }
    }
}

bool MainWindow::hasActiveMotorControlInputVisualizationCommand() const
{
    const int pointCount =
            std::min(static_cast<int>(motorControlInputVisualizationTimeStamp.size()),
                     static_cast<int>(motorControlInputVisualizationPositionUnit.size()));
    return !motorControlInputVisualizationMotorIndex.empty() &&
            pointCount > 0 &&
            motorControlInputVisualizationPointIndex + 1 < pointCount;
}

QVector<double> MainWindow::mapMotorPositionCommandForPlot(
        const std::vector<int>& motorIndex,
        const std::vector<double>& positionUnit) const
{
    QVector<double> values(8, std::numeric_limits<double>::quiet_NaN());
    if(!ui || motorIndex.empty() || positionUnit.empty()){
        return values;
    }

    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        int sourceIndex = -1;
        for(int commandIndex = 0;
            commandIndex < static_cast<int>(motorIndex.size());
            ++commandIndex){
            if(motorIndex[commandIndex] == axisIndex){
                sourceIndex = commandIndex;
                break;
            }
        }
        if(sourceIndex >= 0 && sourceIndex < static_cast<int>(positionUnit.size())){
            values[channelIndex] = positionUnit[sourceIndex];
        }
        ++channelIndex;
    }
    return values;
}

QVector<int> MainWindow::motorControlHybridForceGraphIndexesForPlot() const
{
    QVector<int> graphIndexes;
    if(!ui ||
            !runtimeState.hybridPoseForceModeActive ||
            !activeHybridPoseForceConfig.enabled){
        return graphIndexes;
    }

    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<8; ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(std::find(activeHybridPoseForceConfig.forceAxisIndex.begin(),
                     activeHybridPoseForceConfig.forceAxisIndex.end(),
                     axisIndex) != activeHybridPoseForceConfig.forceAxisIndex.end()){
            graphIndexes.push_back(channelIndex);
        }
        ++channelIndex;
    }
    return graphIndexes;
}

QVector<double> MainWindow::mapHybridMotorControlInputForPlot(
        const std::vector<int>& motorIndex,
        const std::vector<double>& positionUnit,
        int pointIndex) const
{
    QVector<double> values = mapMotorPositionCommandForPlot(motorIndex, positionUnit);
    if(!ui ||
            !runtimeState.hybridPoseForceModeActive ||
            !activeHybridPoseForceConfig.enabled){
        return values;
    }

    auto expectedForceForSensor = [this, pointIndex](int sensorIndex) -> double {
        if(sensorIndex < 0){
            return std::numeric_limits<double>::quiet_NaN();
        }
        if(sensorIndex < static_cast<int>(activeHybridExpectedForceTraj.size())){
            const std::vector<double>& sensorTraj =
                    activeHybridExpectedForceTraj[sensorIndex];
            if(!sensorTraj.empty()){
                const int clampedPointIndex =
                        std::min(std::max(pointIndex, 0),
                                 static_cast<int>(sensorTraj.size()) - 1);
                const double value = sensorTraj[clampedPointIndex];
                if(std::isfinite(value)){
                    return value;
                }
            }
        }
        if(sensorIndex < static_cast<int>(activeHybridPoseForceConfig.frozenExpectedForce.size())){
            const double value =
                    activeHybridPoseForceConfig.frozenExpectedForce[sensorIndex];
            if(std::isfinite(value)){
                return value;
            }
        }
        if(sensorIndex < static_cast<int>(mainForceSensorExpVal.size())){
            const double value = mainForceSensorExpVal[sensorIndex];
            if(std::isfinite(value)){
                return value;
            }
        }
        return std::numeric_limits<double>::quiet_NaN();
    };

    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }

        const auto forceAxisIt =
                std::find(activeHybridPoseForceConfig.forceAxisIndex.begin(),
                          activeHybridPoseForceConfig.forceAxisIndex.end(),
                          axisIndex);
        if(forceAxisIt != activeHybridPoseForceConfig.forceAxisIndex.end()){
            const int forceConfigIndex =
                    static_cast<int>(std::distance(
                                         activeHybridPoseForceConfig.forceAxisIndex.begin(),
                                         forceAxisIt));
            int sensorIndex = -1;
            if(forceConfigIndex >= 0 &&
                    forceConfigIndex < static_cast<int>(activeHybridPoseForceConfig.forceSensorIndex.size())){
                sensorIndex = activeHybridPoseForceConfig.forceSensorIndex[forceConfigIndex];
            }
            else if(axisIndex < static_cast<int>(axisSensorIndexVec.size()) &&
                    axisSensorIndexVec[axisIndex]){
                sensorIndex = axisSensorIndexVec[axisIndex]->value();
            }
            values[channelIndex] = expectedForceForSensor(sensorIndex);
        }
        ++channelIndex;
    }
    return values;
}

void MainWindow::resetCableKinematicVisualizationState()
{
    lastCableLengthVisualizationValues.clear();
    lastCableKinematicVisualizationTimeSec = -1.0;
}

void MainWindow::updateCableKinematicVisualizationFromSnapshot(
        double timeSec,
        const ControlWorker::Snapshot& snapshot)
{
    if(!curveDrawer || !std::isfinite(timeSec)){
        resetCableKinematicVisualizationState();
        return;
    }

    QVector<double> cableLength(8, 0.0);
    std::vector<std::vector<double>> currentCableLengthByEnd;
    bool hasCableLength = buildCurrentCableLengths(currentCableLengthByEnd);
    if(hasCableLength){
        cableLength = flattenCableLengthsByAxisOrder(currentCableLengthByEnd);
    }
    else{
        hasCableLength =
                buildMotorPositionCableLengthForVisualization(snapshot.motorAbsPos,
                                                              false,
                                                              cableLength);
    }

    if(!hasCableLength && ui && ui->devUseLS && ui->devUseLS->isChecked() &&
            hardwareInterface.isLSConnected()){
        hasCableLength =
                buildEncoderCableLengthForVisualization(
                    hardwareInterface.getAllMotorEncoderPosUnit(),
                    cableLength);
    }

    if(!hasCableLength){
        resetCableKinematicVisualizationState();
        return;
    }

    QVector<double> cableSpeed(8, 0.0);
    bool hasSnapshotMotorVelocity = false;
    if(ui && !snapshot.motorVel.empty()){
        for(int axisIndex = 0; axisIndex < ui->devAxisNum->value(); ++axisIndex){
            if(!isModeledMotorAxis(axisIndex)){
                continue;
            }
            if(axisIndex < static_cast<int>(snapshot.motorVel.size()) &&
                    std::isfinite(snapshot.motorVel[axisIndex])){
                hasSnapshotMotorVelocity = true;
                break;
            }
        }
    }

    if(hasSnapshotMotorVelocity){
        cableSpeed = mapCableSpeedForPlot(snapshot.motorVel);
    }
    else if(lastCableKinematicVisualizationTimeSec >= 0.0 &&
            lastCableLengthVisualizationValues.size() == cableLength.size() &&
            timeSec > lastCableKinematicVisualizationTimeSec){
        const double dt = timeSec - lastCableKinematicVisualizationTimeSec;
        if(std::isfinite(dt) && dt > 1e-9){
            for(int channelIndex = 0; channelIndex < cableLength.size(); ++channelIndex){
                const double speed =
                        (cableLength[channelIndex] -
                         lastCableLengthVisualizationValues[channelIndex]) / dt;
                cableSpeed[channelIndex] = std::isfinite(speed) ? speed : 0.0;
            }
        }
    }

    curveDrawer->updateCableLengthPlotSignal(timeSec, cableLength);
    curveDrawer->updateCableSpeedPlotSignal(timeSec, cableSpeed);
    lastCableLengthVisualizationValues = cableLength;
    lastCableKinematicVisualizationTimeSec = timeSec;
}

bool MainWindow::buildEncoderCableLengthForVisualization(
        const std::vector<double>& encoderPosition,
        QVector<double>& cableLength) const
{
    return buildMotorPositionCableLengthForVisualization(encoderPosition,
                                                         true,
                                                         cableLength);
}

bool MainWindow::buildMotorPositionCableLengthForVisualization(
        const std::vector<double>& motorPosition,
        bool useEncoderHome,
        QVector<double>& cableLength) const
{
    cableLength = QVector<double>(8, 0.0);
    if(!ui || motorPosition.empty() || !hasForwardKinematicsCableLengthReference()){
        return false;
    }

    const std::vector<double>* homeMotorPosition = nullptr;
    std::vector<std::vector<double>> referencePose;
    auto hasModeledHomeValues = [this](const std::vector<double>& position) -> bool {
        bool hasModeledAxis = false;
        for(int axisIndex = 0; axisIndex < ui->devAxisNum->value(); ++axisIndex){
            if(!isModeledMotorAxis(axisIndex)){
                continue;
            }
            hasModeledAxis = true;
            if(axisIndex >= static_cast<int>(position.size()) ||
                    !std::isfinite(position[axisIndex])){
                return false;
            }
        }
        return hasModeledAxis;
    };
    if(currentRuntimeMotorHomeReferenceLoaded &&
            hasFinitePoseMatrix(currentRuntimeMotorHomePlatformPose)){
        // 平台位姿与 reference_motor_pos 是同一时刻采集的一对参考。
        // motor_home_pos 在旧G302快照中可能仍是上电零点，不能优先与当前平台位姿混用。
        const std::vector<double>& runtimeReference =
                isLiteTemplateActive() ?
                    (useEncoderHome ?
                         currentRuntimeReferenceMotorEncoderPos :
                         currentRuntimeReferenceMotorPos) :
                    (useEncoderHome ?
                         currentRuntimeMotorHomeEncoderPos :
                         currentRuntimeMotorHomePos);
        if(hasModeledHomeValues(runtimeReference)){
            homeMotorPosition = &runtimeReference;
            referencePose = normalizedPoseMatrix(currentRuntimeMotorHomePlatformPose);
        }
        else if(isLiteTemplateActive()){
            // 兼容尚未包含 reference_motor_pos 字段的旧记录。
            const std::vector<double>& legacyRuntimeHome =
                    useEncoderHome ?
                        currentRuntimeMotorHomeEncoderPos :
                        currentRuntimeMotorHomePos;
            if(hasModeledHomeValues(legacyRuntimeHome)){
                homeMotorPosition = &legacyRuntimeHome;
                referencePose = normalizedPoseMatrix(currentRuntimeMotorHomePlatformPose);
            }
        }
    }

    if(!homeMotorPosition && zeroMotorHomeReferenceLoaded){
        const std::vector<double>& zeroHome =
                useEncoderHome ? zeroMotorHomeEncoderPos : zeroMotorHomePos;
        if(hasModeledHomeValues(zeroHome)){
            homeMotorPosition = &zeroHome;
            referencePose = hasFinitePoseMatrix(zeroMotorHomePlatformPose) ?
                        normalizedPoseMatrix(zeroMotorHomePlatformPose) :
                        normalizedPoseMatrix(configuredCableHomePlatformPose());
        }
    }

    if(!homeMotorPosition){
        return false;
    }

    return buildCableLengthForVisualizationFromReference(motorPosition,
                                                         *homeMotorPosition,
                                                         referencePose,
                                                         cableLength);
}

bool MainWindow::buildCableLengthForVisualizationFromReference(
        const std::vector<double>& motorPosition,
        const std::vector<double>& homeMotorPosition,
        const std::vector<std::vector<double>>& referencePose,
        QVector<double>& cableLength) const
{
    cableLength = QVector<double>(8, 0.0);
    const std::vector<std::vector<double>> normalizedReferencePose =
            normalizedPoseMatrix(referencePose);
    if(!ui ||
            motorPosition.empty() ||
            homeMotorPosition.empty() ||
            !hasFinitePoseMatrix(normalizedReferencePose)){
        return false;
    }

    const QVector<double> referenceCableLength =
            buildGeometricCableLengthForPlot(normalizedReferencePose);
    int channelIndex = 0;
    for(int axisIndex = 0;
        axisIndex < ui->devAxisNum->value() && channelIndex < cableLength.size();
        ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(axisIndex >= static_cast<int>(motorPosition.size()) ||
                axisIndex >= static_cast<int>(homeMotorPosition.size()) ||
                !std::isfinite(motorPosition[axisIndex]) ||
                !std::isfinite(homeMotorPosition[axisIndex]) ||
                channelIndex >= referenceCableLength.size() ||
                !std::isfinite(referenceCableLength[channelIndex])){
            return false;
        }

        const double motorDelta =
                motorPosition[axisIndex] - homeMotorPosition[axisIndex];
        const double cableDelta =
                convertMotorFeedbackToCableValue(axisIndex, motorDelta);
        if(!std::isfinite(cableDelta)){
            return false;
        }

        cableLength[channelIndex] = referenceCableLength[channelIndex] + cableDelta;
        ++channelIndex;
    }

    return channelIndex > 0;
}

QVector<double> MainWindow::mapMotorCommandForPlot(const std::vector<double>& motorCommand) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(axisIndex < static_cast<int>(motorCommand.size())){
            values[channelIndex] = motorCommand[axisIndex];
        }
        ++channelIndex;
    }
    return values;
}

QVector<double> MainWindow::mapCableTensionForPlot(const std::vector<double>& forceSensorValue) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        const int sensorIndex = axisSensorIndexVec[axisIndex]->value();
        if(sensorIndex >= 0 && sensorIndex < static_cast<int>(forceSensorValue.size())){
            values[channelIndex] = forceSensorValue[sensorIndex];
        }
        ++channelIndex;
    }
    return values;
}

double MainWindow::cableForceLimitForSensorIndex(int sensorIndex) const
{
    if(sensorIndex < 0 || !ui){
        return std::numeric_limits<double>::quiet_NaN();
    }

    const int axisCount = std::min(ui->devAxisNum->value(),
                                   static_cast<int>(axisSensorIndexVec.size()));
    for(int axisIndex=0;
        axisIndex<axisCount && axisIndex<static_cast<int>(axisForceMaxVec.size());
        ++axisIndex){
        if(!isModeledMotorAxis(axisIndex) ||
                !axisSensorIndexVec[axisIndex] ||
                !axisForceMaxVec[axisIndex]){
            continue;
        }
        if(axisSensorIndexVec[axisIndex]->value() != sensorIndex){
            continue;
        }

        const double forceLimit = axisForceMaxVec[axisIndex]->value();
        return std::isfinite(forceLimit) && forceLimit > 0.0 ?
                    forceLimit :
                    std::numeric_limits<double>::quiet_NaN();
    }

    return std::numeric_limits<double>::quiet_NaN();
}

QVector<double> MainWindow::mapCableSpeedForPlot(const std::vector<double>& motorVelocity) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(axisIndex < static_cast<int>(motorVelocity.size())){
            values[channelIndex] = convertMotorFeedbackToCableValue(axisIndex, motorVelocity[axisIndex]);
        }
        ++channelIndex;
    }
    return values;
}

QVector<double> MainWindow::buildGeometricCableLengthForPlot(const std::vector<std::vector<double>>& platformPose) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        const int endIndex = axisEndVec[axisIndex]->value();
        if(endIndex >= 0 &&
                endIndex < static_cast<int>(platformPose.size()) &&
                platformPose[endIndex].size() >= 6){
            const std::vector<double> Rx = {
                1.0, 0.0, 0.0,
                0.0, cos(platformPose[endIndex][3]), -sin(platformPose[endIndex][3]),
                0.0, sin(platformPose[endIndex][3]),  cos(platformPose[endIndex][3])
            };
            const std::vector<double> Ry = {
                 cos(platformPose[endIndex][4]), 0.0, sin(platformPose[endIndex][4]),
                 0.0, 1.0, 0.0,
                -sin(platformPose[endIndex][4]), 0.0, cos(platformPose[endIndex][4])
            };
            const std::vector<double> Rz = {
                cos(platformPose[endIndex][5]), -sin(platformPose[endIndex][5]), 0.0,
                sin(platformPose[endIndex][5]),  cos(platformPose[endIndex][5]), 0.0,
                0.0, 0.0, 1.0
            };

            const std::vector<std::vector<double>> rotationX = {
                {Rx[0], Rx[1], Rx[2]},
                {Rx[3], Rx[4], Rx[5]},
                {Rx[6], Rx[7], Rx[8]}
            };
            const std::vector<std::vector<double>> rotationY = {
                {Ry[0], Ry[1], Ry[2]},
                {Ry[3], Ry[4], Ry[5]},
                {Ry[6], Ry[7], Ry[8]}
            };
            const std::vector<std::vector<double>> rotationZ = {
                {Rz[0], Rz[1], Rz[2]},
                {Rz[3], Rz[4], Rz[5]},
                {Rz[6], Rz[7], Rz[8]}
            };
            const std::vector<std::vector<double>> rotation =
                MatrixFun::matrix_mul(MatrixFun::matrix_mul(rotationZ, rotationY), rotationX);
            const std::vector<std::vector<double>> rotatedContactPoint =
                MatrixFun::matrix_mul(rotation,
                                      {{axisCableEndPosXVec[axisIndex]->value()},
                                       {axisCableEndPosYVec[axisIndex]->value()},
                                       {axisCableEndPosZVec[axisIndex]->value()}});

            std::vector<double> globalContactPoint(3, 0.0);
            for(int coordIndex=0; coordIndex<3; ++coordIndex){
                globalContactPoint[coordIndex] =
                    platformPose[endIndex][coordIndex] + rotatedContactPoint[coordIndex][0];
            }

            const std::vector<double> anchorPoint = {
                axisCableStartPosXVec[axisIndex]->value(),
                axisCableStartPosYVec[axisIndex]->value(),
                axisCableStartPosZVec[axisIndex]->value()
            };
            values[channelIndex] =
                MatrixFun::cableLengthCalculate(globalContactPoint, anchorPoint, buildPulleyRadius()).idealLength;
        }
        ++channelIndex;
    }
    return values;
}
