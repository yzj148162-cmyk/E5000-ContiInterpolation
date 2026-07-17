#include "mainwindow.h"

#include "ui_mainwindow.h"

#include <QDateTime>
#include <QMetaObject>
#include <QPainter>
#include <QPen>
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
    config.activeDeltaUnit = ui_->activeDeltaSpin->value();
    config.holdDeltaUnit = ui_->holdDeltaSpin->value();
    config.durationS = ui_->durationSpin->value();
    config.producerPeriodMs = ui_->producerPeriodSpin->value();
    config.maxVectorVelocity = ui_->maxVelocitySpin->value();
    config.accelerationTimeS = ui_->accelerationSpin->value();
    config.decelerationTimeS = ui_->decelerationSpin->value();
    config.sCurveTimeS = ui_->sCurveSpin->value();
    config.speedRatio = ui_->speedRatioSpin->value();
    config.lookaheadEnabled = ui_->lookaheadCheck->isChecked();
    config.pathErrorUnit = ui_->pathErrorSpin->value();
    config.preloadSegments = ui_->preloadSegmentsSpin->value();
    config.targetBufferSegments = ui_->targetBufferSpin->value();
    config.lowBufferSegments = ui_->lowBufferSpin->value();
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
    ui_->holdDeltaSpin->setEnabled(dualAxis);
    ui_->holdDeltaLabel->setEnabled(dualAxis);
    ui_->stageHintLabel->setText(dualAxis
        ? QStringLiteral("阶段二：两轴均按同一五次多项式比例运动。")
        : QStringLiteral("阶段一：主动轴运动；保持轴只参与坐标系，不产生位移。"));
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
    ui_->boardValueLabel->setText(status.boardInitialized
                                      ? QStringLiteral("已初始化：%1 张卡，当前卡 %2")
                                            .arg(status.detectedBoardCount)
                                            .arg(status.cardNo)
                                      : QStringLiteral("未初始化"));
    ui_->stateValueLabel->setText(status.stateText);
    ui_->runStateValueLabel->setText(QString::number(status.runState));
    ui_->markValueLabel->setText(QStringLiteral("%1 / %2").arg(status.currentMark).arg(status.pushedMark));
    ui_->bufferValueLabel->setText(QString::number(qMax(0L, status.pushedMark - status.currentMark)));
    ui_->spaceValueLabel->setText(QString::number(status.remainSpace));
    ui_->hostQueueValueLabel->setText(QString::number(status.hostQueueSize));
    ui_->feedbackSummaryValueLabel->setText(status.boardInitialized
                                                ? QStringLiteral("总线错误码：%1；%2；本次 Trace 帧：%3；API=%4")
                                                      .arg(status.busErrorCode)
                                                      .arg(status.traceStateText)
                                                      .arg(status.traceFramesRead)
                                                      .arg(status.traceLastApiResult)
                                                : QStringLiteral("控制卡未初始化"));
    ui_->recordingStateValueLabel->setText(status.recorder.recording
                                               ? QStringLiteral("记录中（500 us Trace）")
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
