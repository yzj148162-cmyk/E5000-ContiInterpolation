#include "mainwindow.h"

#include "ui_mainwindow.h"

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
#include <QtCharts/QValueAxis>

#include <limits>

#include "hardware/ContiWorker.h"

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
    connect(ui_->producerPeriodSpin, qOverload<int>(&QSpinBox::valueChanged),
            this, &MainWindow::onProducerPeriodChanged);
    connect(ui_->initializeButton, &QPushButton::clicked, this, &MainWindow::onInitializeClicked);
    connect(ui_->closeBoardButton, &QPushButton::clicked, this, &MainWindow::onCloseBoardClicked);
    connect(ui_->enableAxesButton, &QPushButton::clicked, this, &MainWindow::onEnableAxesClicked);
    connect(ui_->disableAxesButton, &QPushButton::clicked, this, &MainWindow::onDisableAxesClicked);
    connect(ui_->startButton, &QPushButton::clicked, this, &MainWindow::onStartClicked);
    connect(ui_->stopButton, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(ui_->emergencyStopButton, &QPushButton::clicked, this, &MainWindow::onEmergencyStopClicked);
    connect(ui_->refreshFeedbackButton, &QPushButton::clicked, this, &MainWindow::onRefreshFeedbackClicked);
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

    connect(this, &MainWindow::initializeBoardRequested, worker_, &ContiWorker::initializeBoard);
    connect(this, &MainWindow::closeBoardRequested, worker_, &ContiWorker::closeBoard);
    connect(this, &MainWindow::enableAxesRequested, worker_, &ContiWorker::enableSelectedAxes);
    connect(this, &MainWindow::disableAxesRequested, worker_, &ContiWorker::disableSelectedAxes);
    connect(this, &MainWindow::startTestRequested, worker_, &ContiWorker::startTest);
    connect(this, &MainWindow::stopTestRequested, worker_, &ContiWorker::stopTest);
    connect(this, &MainWindow::refreshFeedbackRequested, worker_, &ContiWorker::refreshFeedback);
    connect(this, &MainWindow::enableJogAxisRequested, worker_, &ContiWorker::enableJogAxis);
    connect(this, &MainWindow::disableJogAxisRequested, worker_, &ContiWorker::disableJogAxis);
    connect(this, &MainWindow::setJogAxisZeroRequested, worker_, &ContiWorker::setJogAxisZero);
    connect(this, &MainWindow::startPointMoveRequested, worker_, &ContiWorker::startPointMove);
    connect(this, &MainWindow::stopPointMoveRequested, worker_, &ContiWorker::stopPointMove);
    connect(this, &MainWindow::startTelemetryRecordingRequested,
            worker_, &ContiWorker::startTelemetryRecording);
    connect(this, &MainWindow::stopTelemetryRecordingRequested,
            worker_, &ContiWorker::stopTelemetryRecording);
    connect(this, &MainWindow::refreshBusCycleRequested,
            worker_, &ContiWorker::refreshBusCycle);
    connect(ui_->readBusCycleButton, &QPushButton::clicked,
            this, [this] { emit refreshBusCycleRequested(); });
    connect(worker_, &ContiWorker::logMessage, this, &MainWindow::appendLog);
    connect(worker_, &ContiWorker::statusChanged, this, &MainWindow::updateStatus);
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
    ui_->planningAlignmentValueLabel->setText(aligned
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
    ui_->runStateValueLabel->setText(QString::number(status.runState));
    ui_->markValueLabel->setText(QStringLiteral("%1 / %2").arg(status.currentMark).arg(status.pushedMark));
    ui_->bufferValueLabel->setText(QString::number(qMax(0L, status.pushedMark - status.currentMark)));
    ui_->spaceValueLabel->setText(QString::number(status.remainSpace));
    ui_->hostQueueValueLabel->setText(QString::number(status.hostQueueSize));
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
    ui_->planningAlignmentValueLabel->setText(status.boardInitialized && planningAligned
        ? QStringLiteral("%1 ms = %2 × %3 us")
              .arg(status.producerPeriodMs)
              .arg(status.producerPeriodMs * 1000 / status.busCycleUs)
              .arg(status.busCycleUs)
        : QStringLiteral("待初始化"));
    ui_->expectedPlanTimeValueLabel->setText(QStringLiteral("%1 / %2 ms")
                                                 .arg(status.expectedPlanTimeS * 1000.0, 0, 'f', 1)
                                                 .arg(status.currentPlanTimeS * 1000.0, 0, 'f', 1));
    ui_->phaseErrorValueLabel->setText(QStringLiteral("%1 ms")
                                            .arg(status.phaseErrorMs, 0, 'f', 1));
    ui_->bufferTimeValueLabel->setText(QStringLiteral("%1 ms")
                                            .arg(status.bufferTimeMs, 0, 'f', 1));
    ui_->ratioDiagValueLabel->setText(QStringLiteral("%1 / %2 / %3 / %4 / %5")
                                           .arg(status.ratioRef, 0, 'f', 3)
                                           .arg(status.ratioPhase, 0, 'f', 3)
                                           .arg(status.ratioBuffer, 0, 'f', 3)
                                           .arg(status.ratioCommand, 0, 'f', 3)
                                           .arg(status.ratioApplied, 0, 'f', 3));
    ui_->ratioApiAgeValueLabel->setText(status.ratioLastApiAgoMs < 0
                                             ? QStringLiteral("-- ms")
                                             : QStringLiteral("%1 ms").arg(status.ratioLastApiAgoMs));
    ui_->feedbackSummaryValueLabel->setText(status.boardInitialized
                                                ? QStringLiteral("总线错误码：%1；%2；本次 Trace 帧：%3；API=%4")
                                                      .arg(status.busErrorCode)
                                                      .arg(status.traceStateText)
                                                      .arg(status.traceFramesRead)
                                                      .arg(status.traceLastApiResult)
                                                : QStringLiteral("控制卡未初始化"));
    ui_->recordingStateValueLabel->setText(status.recorder.recording
                                               ? QStringLiteral("记录中（%1 ms Trace）")
                                                     .arg(status.traceSamplePeriodUs / 1000.0, 0, 'f', 1)
                                               : QStringLiteral("未记录"));
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
        const QStringList values {
            QString::number(feedback.axis),
            feedback.valid ? QStringLiteral("正常") : QStringLiteral("读取失败"),
            QString::number(feedback.stateMachine),
            QString::number(feedback.axisErrorCode),
            QString::number(displayedCommandPosition, 'f', 4),
            QString::number(displayedActualPosition, 'f', 4),
            QString::number(displayedCommandPosition - displayedActualPosition, 'f', 4),
            feedback.errorText
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
                                QChartView *view, QChart *&chart,
                                QValueAxis *&timeAxis, QValueAxis *&valueAxis) {
        chart = new QChart;
        chart->setTitle(title);
        chart->legend()->setVisible(true);
        timeAxis = new QValueAxis(chart);
        timeAxis->setTitleText(QStringLiteral("Trace 时间 (s)"));
        timeAxis->setRange(0.0, 30.0);
        valueAxis = new QValueAxis(chart);
        valueAxis->setTitleText(valueTitle);
        valueAxis->setRange(-1.0, 1.0);
        chart->addAxis(timeAxis, Qt::AlignBottom);
        chart->addAxis(valueAxis, Qt::AlignLeft);
        view->setChart(chart);
        view->setRenderHint(QPainter::Antialiasing, false);
    };

    createChart(QStringLiteral("两轴 Trace 指令与实际位置"), QStringLiteral("位置 (°)"),
                ui_->positionChartView, positionChart_, positionTimeAxis_, positionValueAxis_);
    createChart(QStringLiteral("两轴 Trace 跟随误差"), QStringLiteral("指令 - 实际 (°)"),
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
        followingErrorSeries_[index]->setName(QStringLiteral("轴%1 跟随误差").arg(index));
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
    if (!hasLatestStatus_ || latestStatus_.latestTraceSequence == 0) {
        return;
    }
    if (latestStatus_.latestTraceSequence < lastPlottedTraceSequence_) {
        for (QLineSeries *series : positionSeries_) {
            series->clear();
        }
        for (QLineSeries *series : followingErrorSeries_) {
            series->clear();
        }
        lastPlottedTraceSequence_ = 0;
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

    const double timeSeconds = static_cast<double>(latestStatus_.latestTraceTimeUs) / 1000000.0;
    for (int index = 0; index < traceFeedback.size(); ++index) {
        const AxisFeedback &feedback = *traceFeedback.at(index);
        positionSeries_[index * 2]->setName(QStringLiteral("轴%1 指令").arg(feedback.axis));
        positionSeries_[index * 2 + 1]->setName(QStringLiteral("轴%1 实际").arg(feedback.axis));
        followingErrorSeries_[index]->setName(QStringLiteral("轴%1 跟随误差").arg(feedback.axis));
        positionSeries_[index * 2]->append(timeSeconds, feedback.commandPositionUnit);
        positionSeries_[index * 2 + 1]->append(timeSeconds, feedback.encoderPositionUnit);
        followingErrorSeries_[index]->append(timeSeconds,
                                             feedback.commandPositionUnit - feedback.encoderPositionUnit);
    }
    lastPlottedTraceSequence_ = latestStatus_.latestTraceSequence;
    updateChartRanges(positionChart_, positionTimeAxis_, positionValueAxis_,
                      {positionSeries_[0], positionSeries_[1], positionSeries_[2], positionSeries_[3]},
                      timeSeconds, 0.1);
    updateChartRanges(followingErrorChart_, errorTimeAxis_, errorValueAxis_,
                      {followingErrorSeries_[0], followingErrorSeries_[1]}, timeSeconds, 0.01);
    ui_->positionChartView->update();
    ui_->followingErrorChartView->update();
    updateContiTrajectoryChart();
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
    updateChartRanges(contiTrajectoryChart_, contiTrajectoryTimeAxis_, contiTrajectoryValueAxis_,
                      {contiExpectedTrajectorySeries_, contiActualTrajectorySeries_}, timeSeconds, 0.1);
    ui_->contiTrajectoryChartView->update();
}

void MainWindow::updateChartRanges(QChart *, QValueAxis *timeAxis, QValueAxis *valueAxis,
                                   const QList<QLineSeries *> &series, double timeSeconds,
                                   double minimumSpan) const
{
    constexpr double kTimeWindowSeconds = 30.0;
    const double left = qMax(0.0, timeSeconds - kTimeWindowSeconds);
    timeAxis->setRange(left, qMax(kTimeWindowSeconds, timeSeconds));
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
    valueAxis->setRange(minimum - margin, maximum + margin);
}
