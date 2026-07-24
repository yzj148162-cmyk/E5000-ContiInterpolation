#include "mainwindow.h"

#include "ui_mainwindow.h"

#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QMetaObject>
#include <QPainter>
#include <QPen>
#include <QSignalBlocker>
#include <QStringList>
#include <QThread>
#include <QTimer>
#include <QTableWidgetItem>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>

#include <cmath>
#include <initializer_list>
#include <limits>

#include "hardware/ContiWorker.h"
#include "widgets/ZoomableChartView.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui_(new Ui::MainWindow)
    , workerThread_(new QThread(this))
    , worker_(new ContiWorker)
{
    ui_->setupUi(this);
    normalizeProducerPeriodForBusCycle();
    updateBusPeriodUi();
    initializeTelemetryCharts();
    initializeVelocityControlCharts();
    initializeTraceDelayCalibrationCharts();
    onStageChanged(ui_->stageCombo->currentIndex());

    worker_->moveToThread(workerThread_);
    connectWorker();
    workerThread_->start();
    appendLog(QStringLiteral("程序已启动：请先初始化控制卡，再使能所选测试轴。"));
}

MainWindow::~MainWindow()
{
    if (workerThread_->isRunning()) {
        QMetaObject::invokeMethod(worker_, "shutdownHardware", Qt::BlockingQueuedConnection);
        workerThread_->quit();
        workerThread_->wait();
    }
    delete worker_;
    delete ui_;
}

void MainWindow::connectWorker()
{
    connect(ui_->stageCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &MainWindow::onStageChanged);
    connect(ui_->trajectoryPointModeCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int) { onStageChanged(ui_->stageCombo->currentIndex()); });
    connect(ui_->busCycleCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &MainWindow::onBusCycleSelectionChanged);
    connect(ui_->cardUnitDefinitionCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        const QStringList equivalents {
            QStringLiteral("500.622 pulse/unit（1 unit = 1°；180224 pulse/rev）"),
            QStringLiteral("50.0622 pulse/unit（1 unit = 0.1°；180224 pulse/rev）"),
            QStringLiteral("5.00622 pulse/unit（1 unit = 0.01°；180224 pulse/rev）")
        };
        ui_->equivValueLabel->setText(equivalents.value(index, equivalents.first()));
    });
    connect(ui_->velocityUnitDefinitionCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        const bool custom = index == 3;
        ui_->velocityCustomEquivalentLabel->setEnabled(custom);
        ui_->velocityCustomEquivalentSpin->setEnabled(custom);
    });
    connect(ui_->velocityTrajectoryTypeCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            ui_->velocityTrajectoryParameterStack, &QStackedWidget::setCurrentIndex);
    ui_->velocityTrajectoryParameterStack->setCurrentIndex(
        ui_->velocityTrajectoryTypeCombo->currentIndex());
    connect(ui_->producerPeriodSpin, qOverload<int>(&QSpinBox::valueChanged),
            this, &MainWindow::onProducerPeriodChanged);
    connect(ui_->initializeButton, &QPushButton::clicked, this, &MainWindow::onInitializeClicked);
    connect(ui_->closeBoardButton, &QPushButton::clicked, this, &MainWindow::onCloseBoardClicked);
    connect(ui_->contiEnableAxesButton, &QPushButton::clicked, this, &MainWindow::onEnableAxesClicked);
    connect(ui_->contiDisableAxesButton, &QPushButton::clicked, this, &MainWindow::onDisableAxesClicked);
    connect(ui_->enableAllAxesButton, &QPushButton::clicked,
            this, &MainWindow::onEnableAllAxesClicked);
    connect(ui_->disableAllAxesButton, &QPushButton::clicked,
            this, &MainWindow::onDisableAllAxesClicked);
    connect(ui_->startButton, &QPushButton::clicked, this, &MainWindow::onStartClicked);
    connect(ui_->stopButton, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(ui_->emergencyStopButton, &QPushButton::clicked, this, &MainWindow::onEmergencyStopClicked);
    connect(ui_->refreshFeedbackButton, &QPushButton::clicked, this, &MainWindow::onRefreshFeedbackClicked);
    connect(ui_->copyLogButton, &QPushButton::clicked, this, &MainWindow::onCopyLogClicked);
    connect(ui_->clearLogButton, &QPushButton::clicked, this, &MainWindow::onClearLogClicked);
    connect(ui_->jogEnableButton, &QPushButton::clicked, this, &MainWindow::onEnableJogAxisClicked);
    connect(ui_->jogDisableButton, &QPushButton::clicked, this, &MainWindow::onDisableJogAxisClicked);
    connect(ui_->jogSetCurrentZeroButton, &QPushButton::clicked,
            this, &MainWindow::onSetJogAxisZeroClicked);
    connect(ui_->jogStartButton, &QPushButton::clicked, this, &MainWindow::onStartPointMoveClicked);
    connect(ui_->jogStopButton, &QPushButton::clicked, this, &MainWindow::onStopPointMoveClicked);
    connect(ui_->jogEmergencyStopButton, &QPushButton::clicked,
            this, &MainWindow::onEmergencyStopPointMoveClicked);
    connect(ui_->jogUseActualPositionButton, &QPushButton::clicked,
            this, &MainWindow::onUseActualPositionClicked);
    connect(ui_->jogShowAbsoluteCheck, &QCheckBox::toggled,
            this, &MainWindow::onJogPositionDisplayModeChanged);
    connect(ui_->jogAxisCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &MainWindow::onJogPositionDisplayModeChanged);
    connect(ui_->startRecordingButton, &QPushButton::clicked,
            this, &MainWindow::onStartRecordingClicked);
    connect(ui_->stopRecordingButton, &QPushButton::clicked,
            this, &MainWindow::onStopRecordingClicked);
    connect(ui_->velocityEnableAxisButton, &QPushButton::clicked,
            this, &MainWindow::onVelocityEnableAxisClicked);
    connect(ui_->velocityDisableAxisButton, &QPushButton::clicked,
            this, &MainWindow::onVelocityDisableAxisClicked);
    connect(ui_->velocityStartButton, &QPushButton::clicked,
            this, &MainWindow::onVelocityStartClicked);
    connect(ui_->velocityStopButton, &QPushButton::clicked,
            this, &MainWindow::onVelocityStopClicked);
    connect(ui_->velocityEmergencyStopButton, &QPushButton::clicked,
            this, &MainWindow::onVelocityEmergencyStopClicked);
    connect(ui_->velocityResetButton, &QPushButton::clicked,
            this, &MainWindow::onVelocityResetClicked);
    connect(ui_->velocityClearCurvesButton, &QPushButton::clicked,
            this, &MainWindow::onVelocityClearCurvesClicked);
    connect(ui_->traceDelayUnitCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        const bool custom = index == 3;
        ui_->traceDelayCustomEquivalentLabel->setEnabled(custom);
        ui_->traceDelayCustomEquivalentSpin->setEnabled(custom);
    });
    connect(ui_->traceDelayStartButton, &QPushButton::clicked,
            this, &MainWindow::onTraceDelayStartClicked);
    connect(ui_->traceDelayStopButton, &QPushButton::clicked,
            this, &MainWindow::onTraceDelayStopClicked);
    connect(ui_->traceDelayEmergencyStopButton, &QPushButton::clicked,
            this, &MainWindow::onTraceDelayEmergencyStopClicked);
    connect(ui_->traceDelayResetAxisButton, &QPushButton::clicked,
            this, &MainWindow::onTraceDelayResetAxisClicked);

    connect(this, &MainWindow::initializeBoardRequested, worker_, &ContiWorker::initializeBoard);
    connect(this, &MainWindow::closeBoardRequested, worker_, &ContiWorker::closeBoard);
    connect(this, &MainWindow::enableAxesRequested, worker_, &ContiWorker::enableSelectedAxes);
    connect(this, &MainWindow::disableAxesRequested, worker_, &ContiWorker::disableSelectedAxes);
    connect(this, &MainWindow::enableAllAxesRequested, worker_, &ContiWorker::enableAllDetectedAxes);
    connect(this, &MainWindow::disableAllAxesRequested, worker_, &ContiWorker::disableAllDetectedAxes);
    connect(this, &MainWindow::startTestRequested, worker_, &ContiWorker::startTest);
    connect(this, &MainWindow::stopTestRequested, worker_, &ContiWorker::stopTest);
    connect(this, &MainWindow::refreshFeedbackRequested, worker_, &ContiWorker::refreshFeedback);
    connect(this, &MainWindow::enableJogAxisRequested, worker_, &ContiWorker::enableJogAxis);
    connect(this, &MainWindow::disableJogAxisRequested, worker_, &ContiWorker::disableJogAxis);
    connect(this, &MainWindow::setJogAxisZeroRequested, worker_, &ContiWorker::setJogAxisZero);
    connect(this, &MainWindow::startPointMoveRequested, worker_, &ContiWorker::startPointMove);
    connect(this, &MainWindow::stopPointMoveRequested, worker_, &ContiWorker::stopPointMove);
    connect(this, &MainWindow::startVelocityControlRequested,
            worker_, &ContiWorker::startVelocityControl);
    connect(this, &MainWindow::stopVelocityControlRequested,
            worker_, &ContiWorker::stopVelocityControl);
    connect(this, &MainWindow::resetVelocityControllerRequested,
            worker_, &ContiWorker::resetVelocityController);
    connect(this, &MainWindow::startTraceDelayCalibrationRequested,
            worker_, &ContiWorker::startTraceDelayCalibration);
    connect(this, &MainWindow::stopTraceDelayCalibrationRequested,
            worker_, &ContiWorker::stopTraceDelayCalibration);
    connect(this, &MainWindow::resetTraceDelayCalibrationAxisRequested,
            worker_, &ContiWorker::resetTraceDelayCalibrationAxis);
    connect(this, &MainWindow::startTelemetryRecordingRequested,
            worker_, &ContiWorker::startTelemetryRecording);
    connect(this, &MainWindow::stopTelemetryRecordingRequested,
            worker_, &ContiWorker::stopTelemetryRecording);
    connect(this, &MainWindow::refreshBusCycleRequested,
            worker_, &ContiWorker::refreshBusCycle);
    connect(ui_->readBusCycleButton, &QPushButton::clicked,
            this, [this] { emit refreshBusCycleRequested(); });
    connect(worker_, &ContiWorker::logMessage, this, &MainWindow::appendLog);
    connect(worker_, &ContiWorker::statusChanged, this, [this](const ContiStatus &status) {
        latestStatus_ = status;
        hasLatestStatus_ = true;
        statusUiDirty_ = true;
    });
    connect(worker_, &ContiWorker::velocityPlotSamplesReady,
            this, [this](const QVector<VelocityPlotSample> &samples) {
        constexpr qsizetype kMaximumPendingSamples = 20000;
        pendingVelocityPlotSamples_ += samples;
        const qsizetype overflow = pendingVelocityPlotSamples_.size() - kMaximumPendingSamples;
        if (overflow > 0) {
            pendingVelocityPlotSamples_.remove(0, overflow);
        }
    });
    connect(worker_, &ContiWorker::traceDelayPlotSamplesReady,
            this, [this](const QVector<TraceDelayPlotSample> &samples) {
        constexpr qsizetype kMaximumPendingSamples = 30000;
        pendingTraceDelayPlotSamples_ += samples;
        const qsizetype overflow =
            pendingTraceDelayPlotSamples_.size() - kMaximumPendingSamples;
        if (overflow > 0) {
            pendingTraceDelayPlotSamples_.remove(0, overflow);
        }
    });
}

ContiTestConfig MainWindow::collectConfig() const
{
    ContiTestConfig config;
    config.cardNo = static_cast<quint16>(ui_->cardSpin->value());
    config.crdNo = static_cast<quint16>(ui_->crdSpin->value());
    config.activeAxis = static_cast<quint16>(ui_->activeAxisCombo->currentText().toUInt());
    config.holdAxis = static_cast<quint16>(ui_->holdAxisCombo->currentText().toUInt());
    config.stage = ui_->stageCombo->currentIndex() == 0
        ? TestStage::SingleActiveAxis : TestStage::DualAxis;
    config.trajectoryPointMode = ui_->trajectoryPointModeCombo->currentIndex() == 0
        ? TrajectoryPointMode::QuinticTimeLaw : TrajectoryPointMode::UniformDistance;
    switch (ui_->cardUnitDefinitionCombo->currentIndex()) {
    case 1:
        config.degreesPerCardUnit = 0.1;
        break;
    case 2:
        config.degreesPerCardUnit = 0.01;
        break;
    default:
        config.degreesPerCardUnit = 1.0;
        break;
    }
    config.activeDeltaUnit = ui_->activeDeltaSpin->value();
    config.holdDeltaUnit = ui_->holdDeltaSpin->value();
    config.durationS = ui_->durationSpin->value();
    config.busCycleUs = selectedBusCycleUs();
    config.traceCycle = 1;
    config.producerPeriodMs = ui_->producerPeriodSpin->value();
    config.maxVectorVelocity = ui_->maxVelocitySpin->value();
    config.accelerationTimeS = ui_->accelerationSpin->value();
    config.decelerationTimeS = ui_->decelerationSpin->value();
    config.sCurveTimeS = ui_->sCurveSpin->value();
    config.speedRatio = ui_->speedRatioSpin->value();
    config.lookaheadEnabled = ui_->lookaheadCheck->isChecked();
    config.pathErrorUnit = ui_->pathErrorSpin->value();
    config.preloadAllTrajectoryToCard = ui_->preloadAllTrajectoryCheck->isChecked();
    config.timeSyncEnabled = ui_->timeSyncEnableCheck->isChecked();
    config.startupPreloadMs = ui_->preloadSegmentsSpin->value();
    config.targetBufferMs = ui_->targetBufferSpin->value();
    config.lowBufferMs = ui_->lowBufferSpin->value();
    config.criticalBufferMs = ui_->criticalBufferSpin->value();
    config.executionDelayMs = ui_->executionDelaySpin->value();
    config.ratioUpdatePeriodMs = ui_->ratioPeriodSpin->value();
    config.ratioApiMinIntervalMs = ui_->ratioApiIntervalSpin->value();
    config.ratioSafetyApiIntervalMs = ui_->ratioSafetyApiIntervalSpin->value();
    config.ratioMin = ui_->ratioMinSpin->value();
    config.ratioMax = ui_->speedRatioSpin->value();
    config.phaseGainPerSecond = ui_->phaseGainSpin->value();
    config.phaseDeadbandMs = ui_->phaseDeadbandSpin->value();
    config.bufferGain = ui_->bufferGainSpin->value();
    config.ratioDeadband = ui_->ratioDeadbandSpin->value();
    config.ratioMaxStep = ui_->ratioStepSpin->value();
    config.markOffset = ui_->markOffsetSpin->value();
    return config;
}

SingleAxisJogConfig MainWindow::collectJogConfig() const
{
    SingleAxisJogConfig config;
    config.cardNo = static_cast<quint16>(ui_->cardSpin->value());
    config.axis = static_cast<quint16>(ui_->jogAxisCombo->currentText().toUInt());
    config.targetPositionUnit = ui_->jogTargetPositionSpin->value();
    config.maxVelocityUnitPerSecond = ui_->jogVelocitySpin->value();
    return config;
}

VelocityControlConfig MainWindow::collectVelocityConfig() const
{
    VelocityControlConfig config;
    config.cardNo = static_cast<quint16>(ui_->cardSpin->value());
    config.axis = static_cast<quint16>(ui_->velocityAxisCombo->currentText().toUInt());
    config.trajectoryType = static_cast<VelocityTrajectoryType>(
        ui_->velocityTrajectoryTypeCombo->currentIndex());
    switch (ui_->velocityUnitDefinitionCombo->currentIndex()) {
    case 1:
        config.degreesPerCardUnit = 0.1;
        break;
    case 2:
        config.degreesPerCardUnit = 0.01;
        break;
    case 3:
        config.degreesPerCardUnit = ui_->velocityCustomEquivalentSpin->value()
            / MotorUnit::kPhysicalPulsesPerDegree;
        break;
    default:
        config.degreesPerCardUnit = 1.0;
        break;
    }
    config.relativeDeltaDegree = ui_->velocityDeltaSpin->value();
    config.sineAmplitudeDegree = ui_->velocitySineAmplitudeSpin->value();
    config.sineFrequencyHz = ui_->velocitySineFrequencySpin->value();
    config.chirpAmplitudeDegree = ui_->velocityChirpAmplitudeSpin->value();
    config.chirpStartFrequencyHz = ui_->velocityChirpStartFrequencySpin->value();
    config.chirpEndFrequencyHz = ui_->velocityChirpEndFrequencySpin->value();
    config.durationS = ui_->velocityDurationSpin->value();
    config.controlPeriodMs = ui_->velocityControlPeriodSpin->value();
    config.pidEnabled = ui_->velocityPidEnableCheck->isChecked();
    config.kp = ui_->velocityKpSpin->value();
    config.ki = ui_->velocityKiSpin->value();
    config.kd = ui_->velocityKdSpin->value();
    config.integralLimitDegreeSecond = ui_->velocityIntegralLimitSpin->value();
    config.maxPidCorrectionDegreePerSecond = ui_->velocityMaxCorrectionSpin->value();
    config.velocityFeedforwardEnabled = ui_->velocityFeedforwardCheck->isChecked();
    config.velocityFeedforwardGain = ui_->velocityFeedforwardGainSpin->value();
    config.maxVelocityDegreePerSecond = ui_->velocityMaxSpeedSpin->value();
    config.maxAccelerationDegreePerSecond2 = ui_->velocityMaxAccelerationSpin->value();
    config.onlineChangeTimeS = ui_->velocityChangeTimeSpin->value();
    config.startVelocityThresholdDegreePerSecond = ui_->velocityStartThresholdSpin->value();
    config.positionToleranceDegree = ui_->velocityPositionToleranceSpin->value();
    config.speedToleranceDegreePerSecond = ui_->velocitySpeedToleranceSpin->value();
    config.stableDwellMs = ui_->velocityStableDwellSpin->value();
    config.finishTimeoutMs = ui_->velocityFinishTimeoutSpin->value();
    config.maxFollowingErrorDegree = ui_->velocityMaxFollowingErrorSpin->value();
    config.traceTimeoutMs = ui_->velocityTraceTimeoutSpin->value();
    return config;
}

TraceDelayCalibrationConfig MainWindow::collectTraceDelayCalibrationConfig() const
{
    TraceDelayCalibrationConfig config;
    config.cardNo = static_cast<quint16>(ui_->cardSpin->value());
    config.axis =
        static_cast<quint16>(ui_->traceDelayAxisCombo->currentText().toUInt());
    switch (ui_->traceDelayUnitCombo->currentIndex()) {
    case 1:
        config.degreesPerCardUnit = 0.1;
        break;
    case 2:
        config.degreesPerCardUnit = 0.01;
        break;
    case 3:
        config.degreesPerCardUnit =
            ui_->traceDelayCustomEquivalentSpin->value()
            / MotorUnit::kPhysicalPulsesPerDegree;
        break;
    default:
        config.degreesPerCardUnit = 1.0;
        break;
    }
    config.speedDegreePerSecond = {
        ui_->traceDelaySpeed1Spin->value(),
        ui_->traceDelaySpeed2Spin->value(),
        ui_->traceDelaySpeed3Spin->value()
    };
    config.holdMs = ui_->traceDelayHoldSpin->value();
    config.sampleWindowMs = ui_->traceDelaySampleWindowSpin->value();
    config.restMs = ui_->traceDelayRestSpin->value();
    config.onlineChangeTimeS = ui_->traceDelayChangeTimeSpin->value();
    config.maximumSegmentTravelDegree = ui_->traceDelayTravelLimitSpin->value();
    return config;
}

void MainWindow::onStageChanged(int index)
{
    const bool dualAxis = index == static_cast<int>(TestStage::DualAxis);
    const bool uniformDistance = ui_->trajectoryPointModeCombo->currentIndex() == 1;
    ui_->holdDeltaSpin->setEnabled(dualAxis);
    ui_->holdDeltaLabel->setEnabled(dualAxis);
    if (uniformDistance) {
        ui_->stageHintLabel->setText(dualAxis
            ? QStringLiteral("对照轨迹：两轴按同一线性比例产生等间距小线段，用于排查前瞻连接。")
            : QStringLiteral("对照轨迹：主动轴产生等间距小线段；保持轴不产生位移。"));
    } else {
        ui_->stageHintLabel->setText(dualAxis
            ? QStringLiteral("阶段二：两轴均按同一五次多项式比例运动。")
            : QStringLiteral("阶段一：主动轴运动；保持轴只参与坐标系，不产生位移。"));
    }
}

int MainWindow::selectedBusCycleUs() const
{
    return ui_->busCycleCombo->currentText().section(QLatin1Char(' '), 0, 0).toInt();
}

void MainWindow::normalizeProducerPeriodForBusCycle()
{
    const int busCycleUs = selectedBusCycleUs();
    if (busCycleUs <= 0) {
        return;
    }
    const int minimumMs = (busCycleUs + 999) / 1000;
    ui_->producerPeriodSpin->setMinimum(minimumMs);
    ui_->producerPeriodSpin->setSingleStep(busCycleUs >= 1000 ? minimumMs : 1);

    const int requestedUs = ui_->producerPeriodSpin->value() * 1000;
    const int alignedUs = qMax(busCycleUs,
                                ((requestedUs + busCycleUs / 2) / busCycleUs) * busCycleUs);
    const int alignedMs = (alignedUs + 999) / 1000;
    if (alignedMs != ui_->producerPeriodSpin->value()) {
        const QSignalBlocker blocker(ui_->producerPeriodSpin);
        ui_->producerPeriodSpin->setValue(alignedMs);
    }
}

void MainWindow::updateBusPeriodUi()
{
    const int selectedCycleUs = selectedBusCycleUs();
    const bool boardReady = hasLatestStatus_ && latestStatus_.boardInitialized;
    const int effectiveCycleUs = boardReady ? latestStatus_.busCycleUs : selectedCycleUs;
    const int producerPeriodMs = ui_->producerPeriodSpin->value();
    const bool aligned = effectiveCycleUs > 0
        && (producerPeriodMs * 1000) % effectiveCycleUs == 0;

    if (!boardReady) {
        ui_->busCycleReadValueLabel->setText(QStringLiteral("待初始化"));
        ui_->traceSampleValueLabel->setText(QStringLiteral("待初始化"));
    }
    ui_->contiPlanningAlignmentValueLabel->setText(aligned
        ? QStringLiteral("%1 ms = %2 × %3 us")
              .arg(producerPeriodMs)
              .arg(producerPeriodMs * 1000 / effectiveCycleUs)
              .arg(effectiveCycleUs)
        : QStringLiteral("未与 %1 us 对齐").arg(effectiveCycleUs));
}

void MainWindow::onBusCycleSelectionChanged(int)
{
    normalizeProducerPeriodForBusCycle();
    updateBusPeriodUi();
}

void MainWindow::onProducerPeriodChanged(int)
{
    normalizeProducerPeriodForBusCycle();
    updateBusPeriodUi();
}

void MainWindow::onInitializeClicked()
{
    emit initializeBoardRequested(collectConfig());
}

void MainWindow::onCloseBoardClicked()
{
    emit closeBoardRequested();
}

void MainWindow::onEnableAxesClicked()
{
    emit enableAxesRequested(collectConfig());
}

void MainWindow::onDisableAxesClicked()
{
    emit disableAxesRequested(collectConfig());
}

void MainWindow::onEnableAllAxesClicked()
{
    emit enableAllAxesRequested();
}

void MainWindow::onDisableAllAxesClicked()
{
    emit disableAllAxesRequested();
}

void MainWindow::onStartClicked()
{
    emit startTestRequested(collectConfig());
}

void MainWindow::onStopClicked()
{
    emit stopTestRequested(false);
}

void MainWindow::onEmergencyStopClicked()
{
    emit stopTestRequested(true);
}

void MainWindow::onRefreshFeedbackClicked()
{
    emit refreshFeedbackRequested();
}

void MainWindow::onCopyLogClicked()
{
    const QString logText = ui_->logEdit->toPlainText();
    if (logText.trimmed().isEmpty()) {
        ui_->copyLogButton->setText(QStringLiteral("暂无日志"));
    } else {
        QApplication::clipboard()->setText(logText);
        ui_->copyLogButton->setText(QStringLiteral("已复制"));
    }
    QTimer::singleShot(1200, this, [this] {
        ui_->copyLogButton->setText(QStringLiteral("复制日志"));
    });
}

void MainWindow::onClearLogClicked()
{
    ui_->logEdit->setPlainText(QStringLiteral("日志已清除。"));
}

void MainWindow::onEnableJogAxisClicked()
{
    emit enableJogAxisRequested(collectJogConfig());
}

void MainWindow::onDisableJogAxisClicked()
{
    emit disableJogAxisRequested(collectJogConfig());
}

void MainWindow::onSetJogAxisZeroClicked()
{
    emit setJogAxisZeroRequested(collectJogConfig());
}

void MainWindow::onStartPointMoveClicked()
{
    emit startPointMoveRequested(collectJogConfig());
}

void MainWindow::onStopPointMoveClicked()
{
    emit stopPointMoveRequested(false);
}

void MainWindow::onEmergencyStopPointMoveClicked()
{
    emit stopPointMoveRequested(true);
}

void MainWindow::onUseActualPositionClicked()
{
    // 相对点位模式下，该输入框是“本次移动距离”，不应填入当前位置。
    ui_->jogTargetPositionSpin->setValue(0.0);
}

void MainWindow::onJogPositionDisplayModeChanged()
{
    if (hasLatestStatus_) {
        updateStatus(latestStatus_);
    }
}

void MainWindow::onStartRecordingClicked()
{
    emit startTelemetryRecordingRequested();
}

void MainWindow::onStopRecordingClicked()
{
    emit stopTelemetryRecordingRequested();
}

void MainWindow::onVelocityEnableAxisClicked()
{
    SingleAxisJogConfig config;
    config.cardNo = static_cast<quint16>(ui_->cardSpin->value());
    config.axis = static_cast<quint16>(ui_->velocityAxisCombo->currentText().toUInt());
    emit enableJogAxisRequested(config);
}

void MainWindow::onVelocityDisableAxisClicked()
{
    SingleAxisJogConfig config;
    config.cardNo = static_cast<quint16>(ui_->cardSpin->value());
    config.axis = static_cast<quint16>(ui_->velocityAxisCombo->currentText().toUInt());
    emit disableJogAxisRequested(config);
}

void MainWindow::onVelocityStartClicked()
{
    emit startVelocityControlRequested(collectVelocityConfig());
}

void MainWindow::onVelocityStopClicked()
{
    emit stopVelocityControlRequested(false);
}

void MainWindow::onVelocityEmergencyStopClicked()
{
    emit stopVelocityControlRequested(true);
}

void MainWindow::onVelocityResetClicked()
{
    emit resetVelocityControllerRequested();
}

void MainWindow::onVelocityClearCurvesClicked()
{
    clearVelocityControlCharts();
}

void MainWindow::onTraceDelayStartClicked()
{
    clearTraceDelayCalibrationCharts();
    emit startTraceDelayCalibrationRequested(collectTraceDelayCalibrationConfig());
}

void MainWindow::onTraceDelayStopClicked()
{
    emit stopTraceDelayCalibrationRequested(false);
}

void MainWindow::onTraceDelayEmergencyStopClicked()
{
    emit stopTraceDelayCalibrationRequested(true);
}

void MainWindow::onTraceDelayResetAxisClicked()
{
    emit resetTraceDelayCalibrationAxisRequested(
        static_cast<quint16>(ui_->traceDelayAxisCombo->currentText().toUInt()));
}

void MainWindow::appendLog(const QString &message)
{
    const QString time = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
    ui_->logEdit->appendPlainText(QStringLiteral("[%1] %2").arg(time, message));
}

void MainWindow::updateStatus(const ContiStatus &status)
{
    latestStatus_ = status;
    hasLatestStatus_ = true;
    const bool showAbsolutePosition = ui_->jogShowAbsoluteCheck->isChecked();
    ui_->jogActualPositionLabel->setText(showAbsolutePosition
        ? QStringLiteral("实际绝对位置（Trace）")
        : QStringLiteral("实际相对位置（Trace）"));
    ui_->stateValueLabel->setText(status.stateText);
    ui_->contiRunStateValueLabel->setText(QString::number(status.runState));
    ui_->contiMarkValueLabel->setText(QStringLiteral("%1 / %2").arg(status.currentMark).arg(status.pushedMark));
    ui_->contiBufferValueLabel->setText(QString::number(qMax(0L, status.pushedMark - status.currentMark)));
    ui_->contiSpaceValueLabel->setText(QString::number(status.remainSpace));
    ui_->contiHostQueueValueLabel->setText(QString::number(status.hostQueueSize));
    ui_->busCycleCombo->setEnabled(!status.boardInitialized);
    ui_->cardUnitDefinitionCombo->setEnabled(!status.boardInitialized);
    ui_->readBusCycleButton->setEnabled(status.boardInitialized);
    ui_->busCycleReadValueLabel->setText(status.boardInitialized
        ? QStringLiteral("%1 us").arg(status.busCycleUs)
        : QStringLiteral("待初始化"));
    ui_->traceSampleValueLabel->setText(status.boardInitialized
        ? QStringLiteral("%1 us").arg(status.traceSamplePeriodUs)
        : QStringLiteral("待初始化"));
    const bool planningAligned = status.busCycleUs > 0
        && (status.producerPeriodMs * 1000) % status.busCycleUs == 0;
    ui_->contiPlanningAlignmentValueLabel->setText(status.boardInitialized && planningAligned
        ? QStringLiteral("%1 ms = %2 × %3 us")
              .arg(status.producerPeriodMs)
              .arg(status.producerPeriodMs * 1000 / status.busCycleUs)
              .arg(status.busCycleUs)
        : QStringLiteral("待初始化"));
    ui_->contiExpectedPlanTimeValueLabel->setText(QStringLiteral("%1 / %2 ms")
                                                 .arg(status.expectedPlanTimeS * 1000.0, 0, 'f', 1)
                                                 .arg(status.currentPlanTimeS * 1000.0, 0, 'f', 1));
    ui_->contiPhaseErrorValueLabel->setText(QStringLiteral("%1 ms")
                                            .arg(status.phaseErrorMs, 0, 'f', 1));
    ui_->contiBufferTimeValueLabel->setText(QStringLiteral("%1 ms")
                                            .arg(status.bufferTimeMs, 0, 'f', 1));
    ui_->contiRatioDiagValueLabel->setText(QStringLiteral("%1 / %2 / %3 / %4 / %5")
                                           .arg(status.ratioRef, 0, 'f', 3)
                                           .arg(status.ratioPhase, 0, 'f', 3)
                                           .arg(status.ratioBuffer, 0, 'f', 3)
                                           .arg(status.ratioCommand, 0, 'f', 3)
                                           .arg(status.ratioApplied, 0, 'f', 3));
    ui_->contiRatioApiAgeValueLabel->setText(status.ratioLastApiAgoMs < 0
                                             ? QStringLiteral("-- ms")
                                              : QStringLiteral("%1 ms").arg(status.ratioLastApiAgoMs));
    const int detectedAxisCount = qBound(0, status.detectedAxisCount, 8);
    if (!axisStatusRendered_
        || lastAxisStatusBoardInitialized_ != status.boardInitialized
        || lastAxisEnabledMask_ != status.enabledAxisMask
        || lastDetectedAxisCount_ != detectedAxisCount) {
        const QString unknownStyle = QStringLiteral(
            "QLabel { color: white; background-color: #757575; border-radius: 4px; padding: 4px; }");
        const QString enabledStyle = QStringLiteral(
            "QLabel { color: white; background-color: #2e7d32; border-radius: 4px; padding: 4px; }");
        const QString disabledStyle = QStringLiteral(
            "QLabel { color: white; background-color: #c62828; border-radius: 4px; padding: 4px; }");
        const QList<QLabel *> axisStateLabels {
            ui_->axis0EnableStateLabel, ui_->axis1EnableStateLabel,
            ui_->axis2EnableStateLabel, ui_->axis3EnableStateLabel,
            ui_->axis4EnableStateLabel, ui_->axis5EnableStateLabel,
            ui_->axis6EnableStateLabel, ui_->axis7EnableStateLabel
        };
        for (int axis = 0; axis < axisStateLabels.size(); ++axis) {
            QLabel *label = axisStateLabels.at(axis);
            if (!status.boardInitialized) {
                label->setText(QStringLiteral("轴%1 未知").arg(axis));
                label->setStyleSheet(unknownStyle);
            } else if (axis >= detectedAxisCount) {
                label->setText(QStringLiteral("轴%1 未连接").arg(axis));
                label->setStyleSheet(unknownStyle);
            } else if ((status.enabledAxisMask & static_cast<quint16>(1U << axis)) != 0U) {
                label->setText(QStringLiteral("轴%1 使能").arg(axis));
                label->setStyleSheet(enabledStyle);
            } else {
                label->setText(QStringLiteral("轴%1 失能").arg(axis));
                label->setStyleSheet(disabledStyle);
            }
        }
        axisStatusRendered_ = true;
        lastAxisStatusBoardInitialized_ = status.boardInitialized;
        lastAxisEnabledMask_ = status.enabledAxisMask;
        lastDetectedAxisCount_ = detectedAxisCount;
        ui_->axisEnableStatusGroup->setTitle(status.boardInitialized
            ? QStringLiteral("总线轴使能状态（在线 %1 轴）").arg(detectedAxisCount)
            : QStringLiteral("总线轴使能状态"));
    }
    ui_->enableAllAxesButton->setEnabled(status.boardInitialized && detectedAxisCount > 0);
    ui_->disableAllAxesButton->setEnabled(status.boardInitialized && detectedAxisCount > 0);
    ui_->feedbackSummaryValueLabel->setText(status.boardInitialized
                                                ? QStringLiteral("总线错误码：%1；%2；本次 Trace 帧：%3；API=%4")
                                                      .arg(status.busErrorCode)
                                                      .arg(status.traceStateText)
                                                      .arg(status.traceFramesRead)
                                                      .arg(status.traceLastApiResult)
                                                : QStringLiteral("控制卡未初始化"));
    const VelocityControlStatus &velocity = status.velocityControl;
    ui_->velocityStateValueLabel->setText(QStringLiteral("%1；API=%2，耗时=%3 us")
                                              .arg(velocity.stateText)
                                              .arg(velocity.lastApiResult)
                                              .arg(velocity.lastApiDurationUs));
    ui_->velocityTimingValueLabel->setText(QStringLiteral("%1 s / %2 ms / %3 ms")
                                               .arg(velocity.elapsedS, 0, 'f', 3)
                                               .arg(velocity.controlDtMs, 0, 'f', 3)
                                               .arg(velocity.maximumJitterMs, 0, 'f', 3));
    const QString delayAlignedErrorText =
        velocity.delayAlignedFollowingErrorValid
        ? QStringLiteral("%1°（τ=%2 ms）")
              .arg(velocity.delayAlignedFollowingErrorDegree, 0, 'f', 5)
              .arg(velocity.delayCompensationMs, 0, 'f', 3)
        : QStringLiteral("--（等待延迟历史帧）");
    ui_->velocityPositionStatusValueLabel->setText(
        QStringLiteral("%1 / %2 / %3 / %4 °；延迟对齐=%5")
            .arg(velocity.referencePositionDegree, 0, 'f', 5)
            .arg(velocity.cardCommandPositionDegree, 0, 'f', 5)
            .arg(velocity.actualPositionDegree, 0, 'f', 5)
            .arg(velocity.positionErrorDegree, 0, 'f', 5)
            .arg(delayAlignedErrorText));
    ui_->velocitySpeedStatusValueLabel->setText(QStringLiteral("%1 / %2 / %3 / %4 °/s")
                                                    .arg(velocity.referenceVelocityDegreePerSecond, 0, 'f', 5)
                                                    .arg(velocity.commandVelocityDegreePerSecond, 0, 'f', 5)
                                                    .arg(velocity.cardCommandVelocityDegreePerSecond, 0, 'f', 5)
                                                    .arg(velocity.actualVelocityDegreePerSecond, 0, 'f', 5));
    QStringList velocityFlags;
    if (velocity.velocitySaturated) velocityFlags << QStringLiteral("速度饱和");
    if (velocity.accelerationLimited) velocityFlags << QStringLiteral("加速度限幅");
    if (velocity.integralFrozen) velocityFlags << QStringLiteral("积分冻结");
    if (velocityFlags.isEmpty()) velocityFlags << QStringLiteral("无限幅");
    ui_->velocityPidStatusValueLabel->setText(QStringLiteral("%1 / %2 / %3 / %4 / %5")
                                                  .arg(velocity.feedforwardTermDegreePerSecond, 0, 'f', 5)
                                                  .arg(velocity.pTermDegreePerSecond, 0, 'f', 5)
                                                  .arg(velocity.iTermDegreePerSecond, 0, 'f', 5)
                                                  .arg(velocity.dTermDegreePerSecond, 0, 'f', 5)
                                                  .arg(velocityFlags.join(QStringLiteral("、"))));
    ui_->velocityControlParameterGroup->setEnabled(!velocity.active);
    ui_->velocityStartButton->setEnabled(status.boardInitialized && !velocity.active);
    ui_->velocityEnableAxisButton->setEnabled(status.boardInitialized && !velocity.active);
    ui_->velocityDisableAxisButton->setEnabled(status.boardInitialized && !velocity.active);
    ui_->velocityResetButton->setEnabled(!velocity.active);
    ui_->velocityStopButton->setEnabled(velocity.active);
    ui_->velocityEmergencyStopButton->setEnabled(velocity.active);
    const TraceDelayCalibrationStatus &traceDelay = status.traceDelayCalibration;
    ui_->traceDelayCalibrationParameterGroup->setEnabled(!traceDelay.active);
    ui_->traceDelayStartButton->setEnabled(status.boardInitialized && !traceDelay.active);
    ui_->traceDelayStopButton->setEnabled(traceDelay.active);
    ui_->traceDelayEmergencyStopButton->setEnabled(traceDelay.active);
    ui_->traceDelayResetAxisButton->setEnabled(!traceDelay.active);
    ui_->traceDelayPhaseValueLabel->setText(traceDelay.phaseText);
    ui_->traceDelayProgressBar->setValue(
        qBound(0, traceDelay.progressPercent, 100));
    for (int row = 0; row < ui_->traceDelayResultTable->rowCount(); ++row) {
        TraceDelayAxisResult result;
        result.axis = static_cast<quint16>(row);
        if (row < traceDelay.axisResults.size()) {
            result = traceDelay.axisResults.at(row);
        }
        const QString statusText = result.detail.isEmpty()
            ? (result.calibrated ? QStringLiteral("有效")
                                 : QStringLiteral("待标定"))
            : result.detail;
        const QStringList values {
            QString::number(result.axis),
            QString::number(result.appliedDelayMs, 'f', 4),
            result.measuredDelayMs == 0.0 && !result.calibrated
                ? QStringLiteral("--")
                : QString::number(result.measuredDelayMs, 'f', 4),
            QString::number(result.staticOffsetDegree, 'f', 6),
            result.rSquared == 0.0 && !result.calibrated
                ? QStringLiteral("--")
                : QString::number(result.rSquared, 'f', 5),
            result.source,
            statusText
        };
        for (int column = 0; column < values.size(); ++column) {
            QTableWidgetItem *item =
                ui_->traceDelayResultTable->item(row, column);
            if (item == nullptr) {
                item = new QTableWidgetItem;
                ui_->traceDelayResultTable->setItem(row, column, item);
            }
            item->setText(values.at(column));
            item->setToolTip(result.timestamp.isEmpty()
                                 ? statusText
                                 : QStringLiteral("%1；%2")
                                       .arg(statusText, result.timestamp));
        }
    }
    ui_->recordingStateValueLabel->setText(status.recorder.recording
                                               ? QStringLiteral("记录中（%1 ms Trace）")
                                                     .arg(status.traceSamplePeriodUs / 1000.0, 0, 'f', 1)
                                               : QStringLiteral("未记录"));
    ui_->startRecordingButton->setEnabled(status.boardInitialized && !status.recorder.recording);
    ui_->stopRecordingButton->setEnabled(status.telemetryPlotActive);
    ui_->recordPathValueLabel->setText(status.recorder.outputDirectory.isEmpty()
                                           ? QStringLiteral("开始记录后自动创建 records/run_*")
                                           : status.recorder.outputDirectory);
    ui_->recordStatsValueLabel->setText(QStringLiteral("%1 / %2 / %3")
                                            .arg(status.recorder.writtenFrames)
                                            .arg(status.recorder.queuedFrames)
                                            .arg(status.recorder.droppedFrames));
    if (!status.recorder.errorText.isEmpty()) {
        ui_->recordingStateValueLabel->setText(QStringLiteral("记录错误：%1")
                                                    .arg(status.recorder.errorText));
    }
    for (const AxisFeedback &feedback : status.axisFeedback) {
        const int row = static_cast<int>(feedback.axis);
        if (row < 0 || row >= ui_->axisFeedbackTable->rowCount()) {
            continue;
        }
        const bool hasSoftwareZero = feedback.axis < static_cast<quint16>(status.softwareZeroValid.size())
            && status.softwareZeroValid.at(feedback.axis);
        const double softwareZero = hasSoftwareZero
            ? status.softwareZeroUnit.value(feedback.axis) : 0.0;
        const double displayedCommandPosition = showAbsolutePosition
            ? feedback.commandPositionUnit : feedback.commandPositionUnit - softwareZero;
        const double displayedActualPosition = showAbsolutePosition
            ? feedback.encoderPositionUnit : feedback.encoderPositionUnit - softwareZero;
        const QString delayRemark = !feedback.valid
            ? feedback.errorText
            : (feedback.traceSampleValid
            ? (feedback.delayCompensationValid
                   ? QStringLiteral("延迟补偿 %1 ms（%2）")
                         .arg(feedback.delayCompensationMs, 0, 'f', 3)
                         .arg(feedback.delayCompensationSource)
                   : QStringLiteral("等待 %1 ms 历史帧")
                         .arg(feedback.delayCompensationMs, 0, 'f', 3))
            : feedback.errorText);
        const QStringList values {
            QString::number(feedback.axis),
            feedback.valid ? QStringLiteral("正常") : QStringLiteral("读取失败"),
            QString::number(feedback.stateMachine),
            QString::number(feedback.axisErrorCode),
            QString::number(displayedCommandPosition, 'f', 4),
            QString::number(displayedActualPosition, 'f', 4),
            feedback.delayCompensationValid
                ? QString::number(
                      feedback.delayCompensatedFollowingErrorUnit, 'f', 4)
                : QStringLiteral("--"),
            delayRemark
        };
        for (int column = 0; column < values.size(); ++column) {
            QTableWidgetItem *item = ui_->axisFeedbackTable->item(row, column);
            if (item == nullptr) {
                item = new QTableWidgetItem;
                ui_->axisFeedbackTable->setItem(row, column, item);
            }
            item->setText(values.at(column));
        }
        if (feedback.axis == static_cast<quint16>(ui_->jogAxisCombo->currentText().toUInt())
            && feedback.traceSampleValid) {
            ui_->jogActualPositionSpin->setValue(displayedActualPosition);
        }
    }
}

void MainWindow::initializeTelemetryCharts()
{
    const auto createChart = [](const QString &title, const QString &valueTitle,
                                 ZoomableChartView *view, QChart *&chart,
                                 QValueAxis *&timeAxis, QValueAxis *&valueAxis) {
        chart = new QChart;
        chart->setTitle(title);
        chart->legend()->setVisible(true);
        timeAxis = new QValueAxis(chart);
        timeAxis->setTitleText(QStringLiteral("Trace 时间 (s)"));
        timeAxis->setTitleVisible(true);
        timeAxis->setLabelsVisible(true);
        timeAxis->setLabelFormat(QStringLiteral("%.1f"));
        timeAxis->setTickCount(6);
        timeAxis->setRange(0.0, 30.0);
        valueAxis = new QValueAxis(chart);
        valueAxis->setTitleText(valueTitle);
        valueAxis->setRange(-1.0, 1.0);
        chart->addAxis(timeAxis, Qt::AlignBottom);
        chart->addAxis(valueAxis, Qt::AlignLeft);
        view->setChart(chart);
        view->setRenderHint(QPainter::Antialiasing, false);
        view->setAutomaticRange(0.0, 30.0, -1.0, 1.0);
    };

    createChart(QStringLiteral("两轴 Trace 指令与实际位置"), QStringLiteral("位置 (°)"),
                ui_->positionChartView, positionChart_, positionTimeAxis_, positionValueAxis_);
    createChart(QStringLiteral("两轴 Trace 延迟补偿跟随误差"),
                QStringLiteral("对齐指令 - 实际 (°)"),
                ui_->followingErrorChartView, followingErrorChart_, errorTimeAxis_, errorValueAxis_);
    createChart(QStringLiteral("主动轴：规划期望与 Trace 实际位置"), QStringLiteral("位置 (°)"),
                ui_->contiTrajectoryChartView, contiTrajectoryChart_,
                contiTrajectoryTimeAxis_, contiTrajectoryValueAxis_);

    const QList<QColor> colors {QColor(0, 102, 204), QColor(0, 153, 102),
                                QColor(204, 51, 51), QColor(153, 51, 153)};
    for (int index = 0; index < 4; ++index) {
        positionSeries_[index] = new QLineSeries(positionChart_);
        positionSeries_[index]->setPen(QPen(colors.at(index), 1.4));
        positionSeries_[index]->setName(QStringLiteral("等待 Trace 轴"));
        positionChart_->addSeries(positionSeries_[index]);
        positionSeries_[index]->attachAxis(positionTimeAxis_);
        positionSeries_[index]->attachAxis(positionValueAxis_);
    }
    for (int index = 0; index < 2; ++index) {
        followingErrorSeries_[index] = new QLineSeries(followingErrorChart_);
        followingErrorSeries_[index]->setPen(QPen(colors.at(index), 1.5));
        followingErrorSeries_[index]->setName(QStringLiteral("轴%1 补偿跟随误差").arg(index));
        followingErrorChart_->addSeries(followingErrorSeries_[index]);
        followingErrorSeries_[index]->attachAxis(errorTimeAxis_);
        followingErrorSeries_[index]->attachAxis(errorValueAxis_);
    }

    contiExpectedTrajectorySeries_ = new QLineSeries(contiTrajectoryChart_);
    contiExpectedTrajectorySeries_->setName(QStringLiteral("规划期望"));
    contiExpectedTrajectorySeries_->setPen(QPen(QColor(0, 102, 204), 1.8));
    contiTrajectoryChart_->addSeries(contiExpectedTrajectorySeries_);
    contiExpectedTrajectorySeries_->attachAxis(contiTrajectoryTimeAxis_);
    contiExpectedTrajectorySeries_->attachAxis(contiTrajectoryValueAxis_);
    contiActualTrajectorySeries_ = new QLineSeries(contiTrajectoryChart_);
    contiActualTrajectorySeries_->setName(QStringLiteral("Trace 实际"));
    contiActualTrajectorySeries_->setPen(QPen(QColor(204, 51, 51), 1.5));
    contiTrajectoryChart_->addSeries(contiActualTrajectorySeries_);
    contiActualTrajectorySeries_->attachAxis(contiTrajectoryTimeAxis_);
    contiActualTrajectorySeries_->attachAxis(contiTrajectoryValueAxis_);

    telemetryPlotTimer_ = new QTimer(this);
    telemetryPlotTimer_->setTimerType(Qt::PreciseTimer);
    telemetryPlotTimer_->setInterval(50);
    connect(telemetryPlotTimer_, &QTimer::timeout, this, &MainWindow::updateTelemetryCharts);
    telemetryPlotTimer_->start();
}

void MainWindow::updateTelemetryCharts()
{
    if (statusUiDirty_) {
        statusUiDirty_ = false;
        updateStatus(latestStatus_);
    }
    updateVelocityControlCharts();
    updateTraceDelayCalibrationCharts();
    updateContiTrajectoryChart();
    if (!hasLatestStatus_ || !latestStatus_.telemetryPlotActive) {
        telemetryPlotWasActive_ = false;
        return;
    }
    if (latestStatus_.latestTraceSequence == 0) {
        return;
    }
    if (!telemetryPlotWasActive_) {
        for (QLineSeries *series : positionSeries_) {
            series->clear();
        }
        for (QLineSeries *series : followingErrorSeries_) {
            series->clear();
        }
        lastPlottedTraceSequence_ = 0;
        telemetryPlotStartTimeUs_ = latestStatus_.latestTraceTimeUs;
        ui_->positionChartView->resetAutomaticRangeMode();
        ui_->followingErrorChartView->resetAutomaticRangeMode();
        telemetryPlotWasActive_ = true;
    }
    if (latestStatus_.latestTraceSequence < lastPlottedTraceSequence_) {
        for (QLineSeries *series : positionSeries_) {
            series->clear();
        }
        for (QLineSeries *series : followingErrorSeries_) {
            series->clear();
        }
        lastPlottedTraceSequence_ = 0;
        telemetryPlotStartTimeUs_ = latestStatus_.latestTraceTimeUs;
        ui_->positionChartView->resetAutomaticRangeMode();
        ui_->followingErrorChartView->resetAutomaticRangeMode();
    }
    if (latestStatus_.latestTraceSequence == lastPlottedTraceSequence_) {
        return;
    }

    QList<const AxisFeedback *> traceFeedback;
    for (const AxisFeedback &feedback : latestStatus_.axisFeedback) {
        if (feedback.traceSampleValid) {
            traceFeedback.push_back(&feedback);
        }
        if (traceFeedback.size() == 2) {
            break;
        }
    }
    if (traceFeedback.isEmpty()) {
        return;
    }

    const double timeSeconds = latestStatus_.latestTraceTimeUs >= telemetryPlotStartTimeUs_
        ? static_cast<double>(latestStatus_.latestTraceTimeUs - telemetryPlotStartTimeUs_) / 1000000.0
        : 0.0;
    for (int index = 0; index < traceFeedback.size(); ++index) {
        const AxisFeedback &feedback = *traceFeedback.at(index);
        positionSeries_[index * 2]->setName(QStringLiteral("轴%1 指令").arg(feedback.axis));
        positionSeries_[index * 2 + 1]->setName(QStringLiteral("轴%1 实际").arg(feedback.axis));
        followingErrorSeries_[index]->setName(
            QStringLiteral("轴%1 补偿跟随误差").arg(feedback.axis));
        positionSeries_[index * 2]->append(timeSeconds, feedback.commandPositionUnit);
        positionSeries_[index * 2 + 1]->append(timeSeconds, feedback.encoderPositionUnit);
        if (feedback.delayCompensationValid) {
            followingErrorSeries_[index]->append(
                timeSeconds, feedback.delayCompensatedFollowingErrorUnit);
        }
    }
    lastPlottedTraceSequence_ = latestStatus_.latestTraceSequence;
    updateChartRanges(ui_->positionChartView,
                      {positionSeries_[0], positionSeries_[1], positionSeries_[2], positionSeries_[3]},
                      timeSeconds, 0.1);
    updateChartRanges(ui_->followingErrorChartView,
                      {followingErrorSeries_[0], followingErrorSeries_[1]}, timeSeconds, 0.01);
    ui_->positionChartView->update();
    ui_->followingErrorChartView->update();
}

void MainWindow::initializeVelocityControlCharts()
{
    const auto createChart = [](const QString &title, const QString &valueTitle,
                                 ZoomableChartView *view, QChart *&chart,
                                 QValueAxis *&timeAxis, QValueAxis *&valueAxis) {
        chart = new QChart;
        chart->setTitle(title);
        chart->legend()->setVisible(true);
        timeAxis = new QValueAxis(chart);
        timeAxis->setTitleText(QStringLiteral("运行时间 (s)"));
        timeAxis->setTitleVisible(true);
        timeAxis->setLabelsVisible(true);
        timeAxis->setLabelFormat(QStringLiteral("%.1f"));
        timeAxis->setTickCount(6);
        timeAxis->setRange(0.0, 5.0);
        valueAxis = new QValueAxis(chart);
        valueAxis->setTitleText(valueTitle);
        valueAxis->setRange(-1.0, 1.0);
        chart->addAxis(timeAxis, Qt::AlignBottom);
        chart->addAxis(valueAxis, Qt::AlignLeft);
        view->setChart(chart);
        view->setRenderHint(QPainter::Antialiasing, false);
        view->setAutomaticRange(0.0, 5.0, -1.0, 1.0);
    };
    createChart(QStringLiteral("位置跟踪：规划 / 板卡指令 / Trace实际"),
                QStringLiteral("位置 (°)"), ui_->velocityPositionChartView,
                velocityPositionChart_, velocityPositionTimeAxis_, velocityPositionValueAxis_);
    createChart(QStringLiteral("位置误差：轨迹时间 / 延迟对齐 / 终点容差"),
                QStringLiteral("误差 (°)"),
                 ui_->velocityErrorChartView, velocityErrorChart_,
                 velocityErrorTimeAxis_, velocityErrorValueAxis_);
    createChart(QStringLiteral("速度跟踪：规划 / PID命令 / 板卡指令 / Trace实际"),
                QStringLiteral("速度 (°/s)"), ui_->velocitySpeedChartView,
                velocitySpeedChart_, velocitySpeedTimeAxis_, velocitySpeedValueAxis_);

    const QColor blue(0, 102, 204);
    const QColor purple(128, 64, 160);
    const QColor red(204, 51, 51);
    const QColor green(0, 153, 102);
    const QStringList positionNames {QStringLiteral("规划位置"),
                                     QStringLiteral("板卡指令位置"),
                                     QStringLiteral("Trace实际位置")};
    const QList<QColor> positionColors {blue, purple, red};
    for (int index = 0; index < 3; ++index) {
        velocityPositionSeries_[index] = new QLineSeries(velocityPositionChart_);
        velocityPositionSeries_[index]->setName(positionNames.at(index));
        velocityPositionSeries_[index]->setPen(QPen(positionColors.at(index), 1.5));
        velocityPositionChart_->addSeries(velocityPositionSeries_[index]);
        velocityPositionSeries_[index]->attachAxis(velocityPositionTimeAxis_);
        velocityPositionSeries_[index]->attachAxis(velocityPositionValueAxis_);
    }
    const QStringList errorNames {QStringLiteral("轨迹时间误差"),
                                  QStringLiteral("延迟对齐误差"),
                                  QStringLiteral("正终点容差"),
                                  QStringLiteral("负终点容差")};
    const QList<QColor> errorColors {red, green,
                                     QColor(140, 140, 140),
                                     QColor(140, 140, 140)};
    for (int index = 0; index < 4; ++index) {
        velocityErrorSeries_[index] = new QLineSeries(velocityErrorChart_);
        velocityErrorSeries_[index]->setName(errorNames.at(index));
        velocityErrorSeries_[index]->setPen(
            QPen(errorColors.at(index), index < 2 ? 1.6 : 1.0,
                 index < 2 ? Qt::SolidLine : Qt::DashLine));
        velocityErrorChart_->addSeries(velocityErrorSeries_[index]);
        velocityErrorSeries_[index]->attachAxis(velocityErrorTimeAxis_);
        velocityErrorSeries_[index]->attachAxis(velocityErrorValueAxis_);
    }
    const QStringList speedNames {QStringLiteral("规划速度"), QStringLiteral("PID命令速度"),
                                  QStringLiteral("板卡指令速度"), QStringLiteral("Trace实际速度")};
    const QList<QColor> speedColors {blue, green, purple, red};
    for (int index = 0; index < 4; ++index) {
        velocitySpeedSeries_[index] = new QLineSeries(velocitySpeedChart_);
        velocitySpeedSeries_[index]->setName(speedNames.at(index));
        velocitySpeedSeries_[index]->setPen(QPen(speedColors.at(index), 1.4));
        velocitySpeedChart_->addSeries(velocitySpeedSeries_[index]);
        velocitySpeedSeries_[index]->attachAxis(velocitySpeedTimeAxis_);
        velocitySpeedSeries_[index]->attachAxis(velocitySpeedValueAxis_);
    }
}

void MainWindow::initializeTraceDelayCalibrationCharts()
{
    traceDelayVelocityChart_ = new QChart;
    traceDelayVelocityChart_->setTitle(
        QStringLiteral("标定速度：Trace type03 / type04"));
    traceDelayVelocityTimeAxis_ = new QValueAxis(traceDelayVelocityChart_);
    traceDelayVelocityTimeAxis_->setTitleText(QStringLiteral("Trace 时间 (s)"));
    traceDelayVelocityTimeAxis_->setLabelFormat(QStringLiteral("%.1f"));
    traceDelayVelocityTimeAxis_->setTickCount(6);
    traceDelayVelocityValueAxis_ = new QValueAxis(traceDelayVelocityChart_);
    traceDelayVelocityValueAxis_->setTitleText(QStringLiteral("速度 (°/s)"));
    traceDelayVelocityChart_->addAxis(traceDelayVelocityTimeAxis_, Qt::AlignBottom);
    traceDelayVelocityChart_->addAxis(traceDelayVelocityValueAxis_, Qt::AlignLeft);
    const QStringList velocityNames {QStringLiteral("type03 指令速度"),
                                     QStringLiteral("type04 实际速度")};
    const QList<QColor> velocityColors {QColor(0, 102, 204), QColor(204, 51, 51)};
    for (int index = 0; index < 2; ++index) {
        traceDelayVelocitySeries_[index] =
            new QLineSeries(traceDelayVelocityChart_);
        traceDelayVelocitySeries_[index]->setName(velocityNames.at(index));
        traceDelayVelocitySeries_[index]->setPen(
            QPen(velocityColors.at(index), 1.4));
        traceDelayVelocityChart_->addSeries(traceDelayVelocitySeries_[index]);
        traceDelayVelocitySeries_[index]->attachAxis(traceDelayVelocityTimeAxis_);
        traceDelayVelocitySeries_[index]->attachAxis(traceDelayVelocityValueAxis_);
    }
    ui_->traceDelayVelocityChartView->setChart(traceDelayVelocityChart_);
    ui_->traceDelayVelocityChartView->setRenderHint(QPainter::Antialiasing, false);
    ui_->traceDelayVelocityChartView->setAutomaticRange(
        0.0, 15.0, -120.0, 120.0);

    traceDelayFitChart_ = new QChart;
    traceDelayFitChart_->setTitle(
        QStringLiteral("位置差拟合：e = τv + b"));
    traceDelayFitSpeedAxis_ = new QValueAxis(traceDelayFitChart_);
    traceDelayFitSpeedAxis_->setTitleText(QStringLiteral("稳定段指令速度 (°/s)"));
    traceDelayFitGapAxis_ = new QValueAxis(traceDelayFitChart_);
    traceDelayFitGapAxis_->setTitleText(QStringLiteral("type05-type06 (°)"));
    traceDelayFitChart_->addAxis(traceDelayFitSpeedAxis_, Qt::AlignBottom);
    traceDelayFitChart_->addAxis(traceDelayFitGapAxis_, Qt::AlignLeft);
    traceDelayFitPointSeries_ = new QScatterSeries(traceDelayFitChart_);
    traceDelayFitPointSeries_->setName(QStringLiteral("六段均值"));
    traceDelayFitPointSeries_->setMarkerSize(9.0);
    traceDelayFitLineSeries_ = new QLineSeries(traceDelayFitChart_);
    traceDelayFitLineSeries_->setName(QStringLiteral("线性拟合"));
    traceDelayFitLineSeries_->setPen(QPen(QColor(0, 153, 102), 1.6));
    traceDelayFitChart_->addSeries(traceDelayFitPointSeries_);
    traceDelayFitChart_->addSeries(traceDelayFitLineSeries_);
    traceDelayFitPointSeries_->attachAxis(traceDelayFitSpeedAxis_);
    traceDelayFitPointSeries_->attachAxis(traceDelayFitGapAxis_);
    traceDelayFitLineSeries_->attachAxis(traceDelayFitSpeedAxis_);
    traceDelayFitLineSeries_->attachAxis(traceDelayFitGapAxis_);
    ui_->traceDelayFitChartView->setChart(traceDelayFitChart_);
    ui_->traceDelayFitChartView->setRenderHint(QPainter::Antialiasing, true);
    ui_->traceDelayFitChartView->setAutomaticRange(
        -120.0, 120.0, -1.0, 1.0);
}

void MainWindow::clearTraceDelayCalibrationCharts()
{
    for (QLineSeries *series : traceDelayVelocitySeries_) {
        if (series != nullptr) {
            series->clear();
        }
    }
    if (traceDelayFitPointSeries_ != nullptr) {
        traceDelayFitPointSeries_->clear();
    }
    if (traceDelayFitLineSeries_ != nullptr) {
        traceDelayFitLineSeries_->clear();
    }
    pendingTraceDelayPlotSamples_.clear();
    for (QList<QPointF> &points : traceDelayVelocityDisplayPoints_) {
        points.clear();
    }
    lastTraceDelayPlotTimeS_ = -1.0;
    lastTraceDelayFitSignature_.clear();
    ui_->traceDelayVelocityChartView->resetAutomaticRangeMode();
    ui_->traceDelayFitChartView->resetAutomaticRangeMode();
}

void MainWindow::updateTraceDelayCalibrationCharts()
{
    QVector<TraceDelayPlotSample> samples;
    samples.swap(pendingTraceDelayPlotSamples_);
    if (!samples.isEmpty()) {
        const quint64 newestRunId = samples.constLast().runId;
        if (newestRunId != lastTraceDelayRunId_) {
            clearTraceDelayCalibrationCharts();
            lastTraceDelayRunId_ = newestRunId;
        }
        constexpr double kDisplayIntervalS = 0.005;
        constexpr qsizetype kMaximumPoints = 6000;
        for (const TraceDelayPlotSample &sample : samples) {
            if (sample.runId != newestRunId
                || sample.elapsedS <= lastTraceDelayPlotTimeS_
                || (lastTraceDelayPlotTimeS_ >= 0.0
                    && sample.elapsedS - lastTraceDelayPlotTimeS_
                           < kDisplayIntervalS)) {
                continue;
            }
            traceDelayVelocityDisplayPoints_[0].append(
                QPointF(sample.elapsedS,
                        sample.commandVelocityDegreePerSecond));
            traceDelayVelocityDisplayPoints_[1].append(
                QPointF(sample.elapsedS,
                        sample.actualVelocityDegreePerSecond));
            lastTraceDelayPlotTimeS_ = sample.elapsedS;
        }
        for (QList<QPointF> &points : traceDelayVelocityDisplayPoints_) {
            const qsizetype overflow = points.size() - kMaximumPoints;
            if (overflow > 0) {
                points.remove(0, overflow);
            }
        }
        for (int index = 0; index < 2; ++index) {
            traceDelayVelocitySeries_[index]->replace(
                traceDelayVelocityDisplayPoints_[index]);
        }
        updateChartRanges(ui_->traceDelayVelocityChartView,
                          {traceDelayVelocitySeries_[0],
                           traceDelayVelocitySeries_[1]},
                          lastTraceDelayPlotTimeS_, 1.0);
        ui_->traceDelayVelocityChartView->update();
    }

    if (!hasLatestStatus_) {
        return;
    }
    const TraceDelayCalibrationStatus &status =
        latestStatus_.traceDelayCalibration;
    if (status.fittedSpeedDegreePerSecond.size()
            != status.fittedPositionGapDegree.size()
        || status.fittedSpeedDegreePerSecond.isEmpty()) {
        return;
    }
    QStringList signatureParts;
    for (int index = 0; index < status.fittedSpeedDegreePerSecond.size(); ++index) {
        signatureParts << QStringLiteral("%1:%2")
                              .arg(status.fittedSpeedDegreePerSecond.at(index),
                                   0, 'g', 12)
                              .arg(status.fittedPositionGapDegree.at(index),
                                   0, 'g', 12);
    }
    signatureParts << QString::number(status.fittedSlopeSecond, 'g', 12)
                   << QString::number(status.fittedInterceptDegree, 'g', 12);
    const QString signature = signatureParts.join(QLatin1Char('|'));
    if (signature == lastTraceDelayFitSignature_) {
        return;
    }
    lastTraceDelayFitSignature_ = signature;
    QList<QPointF> points;
    double minimumSpeed = std::numeric_limits<double>::max();
    double maximumSpeed = std::numeric_limits<double>::lowest();
    double minimumGap = std::numeric_limits<double>::max();
    double maximumGap = std::numeric_limits<double>::lowest();
    for (int index = 0; index < status.fittedSpeedDegreePerSecond.size(); ++index) {
        const double speed = status.fittedSpeedDegreePerSecond.at(index);
        const double gap = status.fittedPositionGapDegree.at(index);
        points.append(QPointF(speed, gap));
        minimumSpeed = qMin(minimumSpeed, speed);
        maximumSpeed = qMax(maximumSpeed, speed);
        minimumGap = qMin(minimumGap, gap);
        maximumGap = qMax(maximumGap, gap);
    }
    traceDelayFitPointSeries_->replace(points);
    const double fitMinimum =
        status.fittedSlopeSecond * minimumSpeed
        + status.fittedInterceptDegree;
    const double fitMaximum =
        status.fittedSlopeSecond * maximumSpeed
        + status.fittedInterceptDegree;
    traceDelayFitLineSeries_->replace(
        QList<QPointF> {QPointF(minimumSpeed, fitMinimum),
                        QPointF(maximumSpeed, fitMaximum)});
    minimumGap = qMin(minimumGap, qMin(fitMinimum, fitMaximum));
    maximumGap = qMax(maximumGap, qMax(fitMinimum, fitMaximum));
    const double speedMargin = qMax(1.0, (maximumSpeed - minimumSpeed) * 0.1);
    const double gapMargin = qMax(0.001, (maximumGap - minimumGap) * 0.15);
    ui_->traceDelayFitChartView->setAutomaticRange(
        minimumSpeed - speedMargin, maximumSpeed + speedMargin,
        minimumGap - gapMargin, maximumGap + gapMargin);
    ui_->traceDelayFitChartView->update();
}

void MainWindow::clearVelocityControlCharts()
{
    for (QLineSeries *series : velocityPositionSeries_) {
        if (series != nullptr) series->clear();
    }
    for (QLineSeries *series : velocityErrorSeries_) {
        if (series != nullptr) series->clear();
    }
    for (QLineSeries *series : velocitySpeedSeries_) {
        if (series != nullptr) series->clear();
    }
    pendingVelocityPlotSamples_.clear();
    velocityPlotBucket_.clear();
    for (QList<QPointF> &points : velocityPositionDisplayPoints_) points.clear();
    for (QList<QPointF> &points : velocityErrorDisplayPoints_) points.clear();
    for (QList<QPointF> &points : velocitySpeedDisplayPoints_) points.clear();
    velocityPlotBucketIndex_ = -1;
    lastVelocityPlotTimeS_ = -1.0;
    ui_->velocityPositionChartView->resetAutomaticRangeMode();
    ui_->velocityErrorChartView->resetAutomaticRangeMode();
    ui_->velocitySpeedChartView->resetAutomaticRangeMode();
}

void MainWindow::updateVelocityControlCharts()
{
    QVector<VelocityPlotSample> samples;
    samples.swap(pendingVelocityPlotSamples_);
    const bool controlInactive = hasLatestStatus_ && !latestStatus_.velocityControl.active;
    if (samples.isEmpty() && (velocityPlotBucket_.isEmpty() || !controlInactive)) {
        return;
    }

    const quint64 newestRunId = samples.isEmpty() ? lastVelocityRunId_
                                                   : samples.constLast().runId;
    if (newestRunId != lastVelocityRunId_) {
        clearVelocityControlCharts();
        lastVelocityRunId_ = newestRunId;
    }

    constexpr double kDisplayBucketSeconds = 0.010;
    constexpr double kDisplayHistorySeconds = 20.0;
    constexpr qsizetype kMaximumDisplayPointsPerSeries = 4000;
    bool displayChanged = false;

    const auto appendExtrema = [this](QList<QPointF> &points, const auto &valueOf) {
        const VelocityPlotSample *minimumSample = &velocityPlotBucket_.constFirst();
        const VelocityPlotSample *maximumSample = minimumSample;
        double minimumValue = valueOf(*minimumSample);
        double maximumValue = minimumValue;
        for (const VelocityPlotSample &sample : velocityPlotBucket_) {
            const double value = valueOf(sample);
            if (value < minimumValue) {
                minimumValue = value;
                minimumSample = &sample;
            }
            if (value > maximumValue) {
                maximumValue = value;
                maximumSample = &sample;
            }
        }
        if (minimumSample == maximumSample || qFuzzyCompare(minimumValue, maximumValue)) {
            const VelocityPlotSample &last = velocityPlotBucket_.constLast();
            points.append(QPointF(last.elapsedS, valueOf(last)));
            return;
        }
        if (minimumSample->elapsedS <= maximumSample->elapsedS) {
            points.append(QPointF(minimumSample->elapsedS, minimumValue));
            points.append(QPointF(maximumSample->elapsedS, maximumValue));
        } else {
            points.append(QPointF(maximumSample->elapsedS, maximumValue));
            points.append(QPointF(minimumSample->elapsedS, minimumValue));
        }
    };
    const auto appendValidExtrema = [this](QList<QPointF> &points,
                                            const auto &valueOf,
                                            const auto &isValid) {
        const VelocityPlotSample *minimumSample = nullptr;
        const VelocityPlotSample *maximumSample = nullptr;
        double minimumValue = 0.0;
        double maximumValue = 0.0;
        for (const VelocityPlotSample &sample : velocityPlotBucket_) {
            if (!isValid(sample)) {
                continue;
            }
            const double value = valueOf(sample);
            if (minimumSample == nullptr || value < minimumValue) {
                minimumSample = &sample;
                minimumValue = value;
            }
            if (maximumSample == nullptr || value > maximumValue) {
                maximumSample = &sample;
                maximumValue = value;
            }
        }
        if (minimumSample == nullptr || maximumSample == nullptr) {
            return;
        }
        if (minimumSample == maximumSample || qFuzzyCompare(minimumValue, maximumValue)) {
            points.append(QPointF(maximumSample->elapsedS, maximumValue));
        } else if (minimumSample->elapsedS <= maximumSample->elapsedS) {
            points.append(QPointF(minimumSample->elapsedS, minimumValue));
            points.append(QPointF(maximumSample->elapsedS, maximumValue));
        } else {
            points.append(QPointF(maximumSample->elapsedS, maximumValue));
            points.append(QPointF(minimumSample->elapsedS, minimumValue));
        }
    };
    const auto trimPoints = [kMaximumDisplayPointsPerSeries](QList<QPointF> &points,
                                                             double cutoffTime) {
        qsizetype removeCount = 0;
        while (removeCount < points.size() && points.at(removeCount).x() < cutoffTime) {
            ++removeCount;
        }
        if (removeCount > 0) {
            points.remove(0, removeCount);
        }
        const qsizetype overflow = points.size() - kMaximumDisplayPointsPerSeries;
        if (overflow > 0) {
            points.remove(0, overflow);
        }
    };
    const auto flushBucket = [this, &appendExtrema, &appendValidExtrema,
                              &trimPoints, &displayChanged]() {
        if (velocityPlotBucket_.isEmpty()) {
            return;
        }
        const VelocityPlotSample &last = velocityPlotBucket_.constLast();
        const double time = last.elapsedS;
        velocityPositionDisplayPoints_[0].append(QPointF(time, last.referencePositionDegree));
        velocityPositionDisplayPoints_[1].append(QPointF(time, last.cardCommandPositionDegree));
        appendExtrema(velocityPositionDisplayPoints_[2],
                      [](const VelocityPlotSample &sample) { return sample.actualPositionDegree; });
        appendExtrema(velocityErrorDisplayPoints_[0],
                      [](const VelocityPlotSample &sample) { return sample.positionErrorDegree; });
        appendValidExtrema(
            velocityErrorDisplayPoints_[1],
            [](const VelocityPlotSample &sample) {
                return sample.delayAlignedFollowingErrorDegree;
            },
            [](const VelocityPlotSample &sample) {
                return sample.delayAlignedFollowingErrorValid;
            });
        velocityErrorDisplayPoints_[2].append(QPointF(time, last.positionToleranceDegree));
        velocityErrorDisplayPoints_[3].append(QPointF(time, -last.positionToleranceDegree));
        velocitySpeedDisplayPoints_[0].append(QPointF(time, last.referenceVelocityDegreePerSecond));
        velocitySpeedDisplayPoints_[1].append(QPointF(time, last.commandVelocityDegreePerSecond));
        velocitySpeedDisplayPoints_[2].append(QPointF(time, last.cardCommandVelocityDegreePerSecond));
        appendExtrema(velocitySpeedDisplayPoints_[3],
                      [](const VelocityPlotSample &sample) {
            return sample.actualVelocityDegreePerSecond;
        });

        const double cutoffTime = qMax(0.0, time - kDisplayHistorySeconds);
        for (QList<QPointF> &points : velocityPositionDisplayPoints_) trimPoints(points, cutoffTime);
        for (QList<QPointF> &points : velocityErrorDisplayPoints_) trimPoints(points, cutoffTime);
        for (QList<QPointF> &points : velocitySpeedDisplayPoints_) trimPoints(points, cutoffTime);
        velocityPlotBucket_.clear();
        velocityPlotBucketIndex_ = -1;
        displayChanged = true;
    };

    for (const VelocityPlotSample &sample : samples) {
        if (sample.runId != newestRunId || sample.elapsedS <= lastVelocityPlotTimeS_) {
            continue;
        }
        const qint64 bucketIndex = static_cast<qint64>(
            std::floor((sample.elapsedS + 1e-12) / kDisplayBucketSeconds));
        if (velocityPlotBucketIndex_ >= 0 && bucketIndex != velocityPlotBucketIndex_) {
            flushBucket();
        }
        if (velocityPlotBucketIndex_ < 0) {
            velocityPlotBucketIndex_ = bucketIndex;
        }
        velocityPlotBucket_.push_back(sample);
        lastVelocityPlotTimeS_ = sample.elapsedS;
    }
    if (controlInactive) {
        flushBucket();
    }
    if (!displayChanged) {
        return;
    }

    for (int index = 0; index < 3; ++index) {
        velocityPositionSeries_[index]->replace(velocityPositionDisplayPoints_[index]);
    }
    for (int index = 0; index < 4; ++index) {
        velocityErrorSeries_[index]->replace(velocityErrorDisplayPoints_[index]);
    }
    for (int index = 0; index < 4; ++index) {
        velocitySpeedSeries_[index]->replace(velocitySpeedDisplayPoints_[index]);
    }

    double earliestTime = std::numeric_limits<double>::max();
    double latestTime = 0.0;
    const auto includeTimeRange = [&earliestTime, &latestTime](const QList<QPointF> &points) {
        if (!points.isEmpty()) {
            earliestTime = qMin(earliestTime, points.constFirst().x());
            latestTime = qMax(latestTime, points.constLast().x());
        }
    };
    for (const QList<QPointF> &points : velocityPositionDisplayPoints_) includeTimeRange(points);
    for (const QList<QPointF> &points : velocityErrorDisplayPoints_) includeTimeRange(points);
    for (const QList<QPointF> &points : velocitySpeedDisplayPoints_) includeTimeRange(points);
    if (earliestTime > latestTime) {
        return;
    }
    const double horizontalMinimum = earliestTime <= kDisplayBucketSeconds ? 0.0 : earliestTime;
    const double horizontalMargin = qMax(0.05, (latestTime - horizontalMinimum) * 0.05);
    const double horizontalMaximum = qMax(horizontalMinimum + 5.0, latestTime + horizontalMargin);
    const auto fitAxes = [horizontalMinimum, horizontalMaximum](
                             ZoomableChartView *view,
                             const std::initializer_list<const QList<QPointF> *> &pointSets,
                             double minimumSpan) {
        double minimum = std::numeric_limits<double>::max();
        double maximum = std::numeric_limits<double>::lowest();
        for (const QList<QPointF> *points : pointSets) {
            for (const QPointF &point : *points) {
                minimum = qMin(minimum, point.y());
                maximum = qMax(maximum, point.y());
            }
        }
        if (minimum > maximum) return;
        const double span = qMax(minimumSpan, maximum - minimum);
        const double margin = span * 0.15;
        view->setAutomaticRange(horizontalMinimum, horizontalMaximum,
                                minimum - margin, maximum + margin);
    };
    fitAxes(ui_->velocityPositionChartView,
            {&velocityPositionDisplayPoints_[0], &velocityPositionDisplayPoints_[1],
             &velocityPositionDisplayPoints_[2]}, 0.1);
    fitAxes(ui_->velocityErrorChartView,
            {&velocityErrorDisplayPoints_[0], &velocityErrorDisplayPoints_[1],
             &velocityErrorDisplayPoints_[2], &velocityErrorDisplayPoints_[3]}, 0.01);
    fitAxes(ui_->velocitySpeedChartView,
            {&velocitySpeedDisplayPoints_[0], &velocitySpeedDisplayPoints_[1],
             &velocitySpeedDisplayPoints_[2], &velocitySpeedDisplayPoints_[3]}, 0.1);
    ui_->velocityPositionChartView->update();
    ui_->velocityErrorChartView->update();
    ui_->velocitySpeedChartView->update();
}

void MainWindow::updateContiTrajectoryChart()
{
    if (!hasLatestStatus_ || !latestStatus_.trajectoryComparisonActive) {
        return;
    }
    if (latestStatus_.trajectoryTraceStartTimeUs == 0) {
        // 新一轮测试已启动但还没有取得其首个 Trace 帧，先清空上一次曲线。
        if (contiTrajectoryTraceStartTimeUs_ != 0) {
            contiExpectedTrajectorySeries_->clear();
            contiActualTrajectorySeries_->clear();
            contiTrajectoryTraceStartTimeUs_ = 0;
            lastContiTrajectoryTraceSequence_ = 0;
        }
        return;
    }
    if (contiTrajectoryTraceStartTimeUs_ != latestStatus_.trajectoryTraceStartTimeUs
        || latestStatus_.latestTraceSequence < lastContiTrajectoryTraceSequence_) {
        contiExpectedTrajectorySeries_->clear();
        contiActualTrajectorySeries_->clear();
        contiTrajectoryTraceStartTimeUs_ = latestStatus_.trajectoryTraceStartTimeUs;
        lastContiTrajectoryTraceSequence_ = 0;
        ui_->contiTrajectoryChartView->resetAutomaticRangeMode();
    }
    if (latestStatus_.latestTraceSequence == 0
        || latestStatus_.latestTraceSequence == lastContiTrajectoryTraceSequence_)
    {
        return;
    }

    const AxisFeedback *activeFeedback = nullptr;
    for (const AxisFeedback &feedback : latestStatus_.axisFeedback) {
        if (feedback.axis == latestStatus_.trajectoryActiveAxis && feedback.traceSampleValid) {
            activeFeedback = &feedback;
            break;
        }
    }
    if (activeFeedback == nullptr || latestStatus_.latestTraceTimeUs < contiTrajectoryTraceStartTimeUs_) {
        return;
    }

    const double timeSeconds = static_cast<double>(latestStatus_.latestTraceTimeUs
                                                   - contiTrajectoryTraceStartTimeUs_) / 1000000.0;
    contiExpectedTrajectorySeries_->setName(QStringLiteral("轴%1 规划期望")
                                                  .arg(latestStatus_.trajectoryActiveAxis));
    contiActualTrajectorySeries_->setName(QStringLiteral("轴%1 Trace 实际")
                                                .arg(latestStatus_.trajectoryActiveAxis));
    contiExpectedTrajectorySeries_->append(timeSeconds, latestStatus_.trajectoryExpectedActiveUnit);
    contiActualTrajectorySeries_->append(timeSeconds, activeFeedback->encoderPositionUnit);
    lastContiTrajectoryTraceSequence_ = latestStatus_.latestTraceSequence;
    updateChartRanges(ui_->contiTrajectoryChartView,
                      {contiExpectedTrajectorySeries_, contiActualTrajectorySeries_}, timeSeconds, 0.1);
    ui_->contiTrajectoryChartView->update();
}

void MainWindow::updateChartRanges(ZoomableChartView *view,
                                   const QList<QLineSeries *> &series, double timeSeconds,
                                   double minimumSpan) const
{
    constexpr double kTimeWindowSeconds = 30.0;
    const double left = qMax(0.0, timeSeconds - kTimeWindowSeconds);
    double minimum = std::numeric_limits<double>::max();
    double maximum = std::numeric_limits<double>::lowest();
    for (QLineSeries *line : series) {
        const QList<QPointF> points = line->points();
        int removeCount = 0;
        for (const QPointF &point : points) {
            if (point.x() < left) {
                ++removeCount;
            } else {
                minimum = qMin(minimum, point.y());
                maximum = qMax(maximum, point.y());
            }
        }
        if (removeCount > 0) {
            line->removePoints(0, removeCount);
        }
    }
    if (minimum > maximum) {
        return;
    }
    const double span = qMax(minimumSpan, maximum - minimum);
    const double margin = span * 0.15;
    view->setAutomaticRange(left, qMax(kTimeWindowSeconds, timeSeconds),
                            minimum - margin, maximum + margin);
}
