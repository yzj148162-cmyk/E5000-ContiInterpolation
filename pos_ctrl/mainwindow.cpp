#include "mainwindow.h"

#include "ui_mainwindow.h"

#include <QDateTime>
#include <QMutexLocker>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QVector>
#include <QtMath>
#include <algorithm>
#include <limits>

namespace {
constexpr double kEps = 1e-6;
constexpr double kPlotWindowS = 15.0;
constexpr double kPlotKeepHistoryS = 120.0;
constexpr int kMinUiRefreshMs = 10;
constexpr int kMaxUiRefreshMs = 500;

double rawToDeg(qint32 rawValue, double pulsePerDeg)
{
    if (std::fabs(pulsePerDeg) < kEps) {
        return 0.0;
    }
    return static_cast<double>(rawValue) / pulsePerDeg;
}

qint64 estimatePvtPointCount(double durationS, qint32 planningPeriodMs)
{
    const double durationMs = std::max(1.0, durationS * 1000.0);
    const int safePeriodMs = std::max(1, planningPeriodMs);
    return std::max<qint64>(2, static_cast<qint64>(std::ceil(durationMs / safePeriodMs)) + 1);
}
}

// UI 线程中的误差曲线小部件。
// 这里只做粗粒度绘图，不参与任何实时控制。
class ErrorPlotWidget : public QWidget
{
public:
    explicit ErrorPlotWidget(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(240);
        setAutoFillBackground(true);
    }

    void clearSamples()
    {
        samples_.clear();
        update();
    }

    void appendSample(double timeS, double errorDeg)
    {
        samples_.push_back(QPointF(timeS, errorDeg));
        while (samples_.size() > 2 && (samples_.back().x() - samples_.front().x()) > kPlotKeepHistoryS) {
            samples_.removeFirst();
        }
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QWidget::paintEvent(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), QColor(248, 249, 250));

        const QRectF plotRect(55, 20, width() - 75, height() - 55);
        painter.setPen(QPen(Qt::black, 1.0));
        painter.drawRect(plotRect);

        if (samples_.isEmpty()) {
            painter.drawText(plotRect, Qt::AlignCenter, QStringLiteral("暂无误差数据"));
            return;
        }

        const double latestX = samples_.back().x();
        const double xMax = std::max(kPlotWindowS, latestX);
        const double xMin = std::max(0.0, xMax - kPlotWindowS);

        double yMin = std::numeric_limits<double>::max();
        double yMax = std::numeric_limits<double>::lowest();
        bool hasVisible = false;
        for (const QPointF &pt : samples_) {
            if (pt.x() + 1e-9 < xMin) {
                continue;
            }
            hasVisible = true;
            yMin = std::min(yMin, pt.y());
            yMax = std::max(yMax, pt.y());
        }

        if (!hasVisible) {
            painter.drawText(plotRect, Qt::AlignCenter, QStringLiteral("暂无可见误差数据"));
            return;
        }

        if (std::fabs(yMax - yMin) < kEps) {
            yMin -= 1.0;
            yMax += 1.0;
        } else {
            const double pad = 0.15 * (yMax - yMin);
            yMin -= pad;
            yMax += pad;
        }

        auto mapPoint = [&](const QPointF &src) {
            const double xRatio = (src.x() - xMin) / std::max(kEps, xMax - xMin);
            const double yRatio = (src.y() - yMin) / std::max(kEps, yMax - yMin);
            return QPointF(plotRect.left() + xRatio * plotRect.width(),
                           plotRect.bottom() - yRatio * plotRect.height());
        };

        painter.setPen(QPen(QColor(210, 210, 210), 1.0, Qt::DashLine));
        painter.drawLine(mapPoint(QPointF(xMin, 0.0)), mapPoint(QPointF(xMax, 0.0)));

        QPainterPath path;
        bool started = false;
        for (const QPointF &pt : samples_) {
            if (pt.x() + 1e-9 < xMin) {
                continue;
            }

            const QPointF mapped = mapPoint(pt);
            if (!started) {
                path.moveTo(mapped);
                started = true;
            } else {
                path.lineTo(mapped);
            }
        }

        painter.setPen(QPen(QColor(220, 53, 69), 2.0));
        painter.drawPath(path);

        painter.setPen(Qt::black);
        painter.drawText(QRectF(5, plotRect.top() - 8, 45, 20), Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(yMax, 'f', 1));
        painter.drawText(QRectF(5, plotRect.bottom() - 10, 45, 20), Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(yMin, 'f', 1));
        painter.drawText(QRectF(plotRect.left(), plotRect.bottom() + 5, 80, 20), Qt::AlignLeft | Qt::AlignTop,
                         QString::number(xMin, 'f', 2) + QStringLiteral(" s"));
        painter.drawText(QRectF(plotRect.right() - 80, plotRect.bottom() + 5, 80, 20), Qt::AlignRight | Qt::AlignTop,
                         QString::number(xMax, 'f', 2) + QStringLiteral(" s"));
        painter.drawText(QRectF(plotRect.left(), 0, plotRect.width(), 20), Qt::AlignCenter,
                         QStringLiteral("位置误差曲线（UI 粗粒度显示，deg）"));
    }

private:
    QVector<QPointF> samples_;
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui_(new Ui::MainWindow)
    , hardwareThread_(new HardwareThread(sharedContext_, systemConfig_, this))
    , plannerThread_(new PlannerThread(sharedContext_, systemConfig_, this))
    , uiRefreshController_(new UiRefreshController(sharedContext_, systemConfig_, this))
{
    ui_->setupUi(this);
    setupRuntimeUi();
    connectSignals();
    startThreads();

    appendLog(QStringLiteral("多线程控制框架已启动：系统规划周期用于上层粗规划，UI 显示周期仅用于界面刷新。"));
}

MainWindow::~MainWindow()
{
    stopThreads();
    delete ui_;
}

void MainWindow::setupRuntimeUi()
{
    ui_->modeCombo->clear();
    ui_->modeCombo->addItem(QStringLiteral("PVT"), static_cast<int>(MotionMode::PVT));
    ui_->modeCombo->addItem(QStringLiteral("CSP"), static_cast<int>(MotionMode::CSP));

    ui_->systemPlanningPeriodMsSpin->setRange(1, 1000);
    ui_->uiRefreshMsSpin->setRange(kMinUiRefreshMs, kMaxUiRefreshMs);
    ui_->uiRefreshMsSpin->setValue(systemConfig_.uiRefreshMs);

    auto *plotLayout = new QVBoxLayout(ui_->errorPlotHost);
    plotLayout->setContentsMargins(0, 0, 0, 0);

    errorPlot_ = new ErrorPlotWidget(ui_->errorPlotHost);
    plotLayout->addWidget(errorPlot_);

    ui_->rawPerDegSpin->setSuffix(QStringLiteral(" raw/deg"));
    ui_->positionLabel->setText(QStringLiteral("-- deg"));
    ui_->errorLabel->setText(QStringLiteral("当前位置误差: -- deg"));
    ui_->tipLabel->setText(QStringLiteral(
        "1. 系统规划周期用于上层粗规划。CSP 会在此基础上细化到 1ms；PVT(PVTS) 会直接按该周期生成整表点列。\n"
        "2. PVT 点数估算约为 ceil(总时长 / 系统规划周期) + 1，当前控制卡上限为 5000 点。\n"
        "3. 如果 PVT 点数接近上限，请增大系统规划周期或缩短总时长，避免整表下发失败。\n"
        "4. CSP 由硬件线程每 1ms 消费共享队列中的一个点；PVT 由硬件线程一次性装表启动，运行期只做监测。\n"
        "5. UI 显示周期只负责界面刷新，不参与实时控制。"));

    applyUiRefreshPeriod();
}

void MainWindow::connectSignals()
{
    connect(ui_->initCardButton, &QPushButton::clicked, this, &MainWindow::onInitCard);
    connect(ui_->closeCardButton, &QPushButton::clicked, this, &MainWindow::onCloseCard);
    connect(ui_->enableAxisButton, &QPushButton::clicked, this, &MainWindow::onEnableAxis);
    connect(ui_->disableAxisButton, &QPushButton::clicked, this, &MainWindow::onDisableAxis);
    connect(ui_->readPosButton, &QPushButton::clicked, this, &MainWindow::onReadPosition);
    connect(ui_->startButton, &QPushButton::clicked, this, &MainWindow::onStartMotion);
    connect(ui_->stopButton, &QPushButton::clicked, this, &MainWindow::onStopMotion);
    connect(ui_->uiRefreshMsSpin, &QSpinBox::valueChanged, this, [this](int) { applyUiRefreshPeriod(); });

    connect(hardwareThread_, &HardwareThread::logMessage, this, &MainWindow::onHardwareLog);
    connect(plannerThread_, &PlannerThread::logMessage, this, &MainWindow::onPlannerLog);
    connect(hardwareThread_, &HardwareThread::boardStateChanged, this, &MainWindow::onBoardStateChanged);
    connect(uiRefreshController_, &UiRefreshController::snapshotReady, this, &MainWindow::updateSnapshot);
}

void MainWindow::startThreads()
{
    sharedContext_.running.storeRelease(1);
    hardwareThread_->start();
    plannerThread_->start();
    uiRefreshController_->start();
}

void MainWindow::stopThreads()
{
    uiRefreshController_->stop();

    sharedContext_.running.storeRelease(0);
    {
        QMutexLocker locker(&sharedContext_.plannerMutex);
        sharedContext_.plannerWakeup = true;
    }
    sharedContext_.plannerWaitCondition.wakeAll();

    if (hardwareThread_->isRunning()) {
        hardwareThread_->wait();
    }
    if (plannerThread_->isRunning()) {
        plannerThread_->wait();
    }
}

void MainWindow::appendLog(const QString &text)
{
    const QString stamp = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    ui_->logEdit->appendPlainText(QStringLiteral("[%1] %2").arg(stamp, text));
}

void MainWindow::applyUiRefreshPeriod()
{
    const int uiRefreshMs = std::clamp(ui_->uiRefreshMsSpin->value(), kMinUiRefreshMs, kMaxUiRefreshMs);
    systemConfig_.uiRefreshMs = uiRefreshMs;
    uiRefreshController_->setRefreshIntervalMs(uiRefreshMs);
}

MotionConfig MainWindow::collectMotionConfig() const
{
    MotionConfig config;
    config.mode = static_cast<MotionMode>(ui_->modeCombo->currentData().toInt());
    config.axis = static_cast<quint16>(ui_->axisSpin->value());
    config.deltaDeg = ui_->deltaAngleSpin->value();
    config.durationS = ui_->durationSpin->value();
    config.systemPlanningPeriodMs = ui_->systemPlanningPeriodMsSpin->value();
    config.minVelDeg = ui_->minVelSpin->value();
    config.maxVelDeg = ui_->maxVelSpin->value();
    config.rawPerDeg = ui_->rawPerDegSpin->value();
    config.ecatPort = static_cast<quint16>(ui_->ecatPortSpin->value());
    config.node = static_cast<quint16>(1000 + ui_->nodeSpin->value());
    config.cspInterpPeriodMs = ui_->cspInterpPeriodMsSpin->value();
    config.cspInterpIndex = ui_->cspInterpIndexSpin->value();
    config.cspRxpdoPosAddr = static_cast<quint16>(ui_->cspRxpdoPosAddrSpin->value());
    config.cspTxpdoStatusAddr = static_cast<quint16>(ui_->cspTxpdoStatusAddrSpin->value());
    config.cspTxpdoModeAddr = static_cast<quint16>(ui_->cspTxpdoModeAddrSpin->value());
    config.cspTxpdoPosAddr = static_cast<quint16>(ui_->cspTxpdoPosAddrSpin->value());
    return config;
}

void MainWindow::updateSnapshot(const UiSnapshot &snapshot)
{
    const double pulsePerDeg = ui_->rawPerDegSpin->value();
    const double actualPosDeg = rawToDeg(snapshot.feedback.actualPosRaw, pulsePerDeg);
    const double errorDeg = rawToDeg(snapshot.feedback.errorRaw, pulsePerDeg);

    ui_->connectionLabel->setText(snapshot.feedback.boardInitialized ? QStringLiteral("已初始化")
                                                                     : QStringLiteral("未初始化"));
    ui_->positionLabel->setText(QString::number(actualPosDeg, 'f', 4) + QStringLiteral(" deg"));
    ui_->stateLabel->setText(snapshot.feedback.motionRunning ? QStringLiteral("运行中")
                                                             : QStringLiteral("空闲/监测"));
    ui_->queueLabel->setText(QString::number(snapshot.queueDepth));
    ui_->cycleLabel->setText(QStringLiteral("已发送=%1, 补队列=%2")
                                 .arg(snapshot.sentPointCount)
                                 .arg(snapshot.queueFillCount));
    ui_->errorLabel->setText(QStringLiteral("当前位置误差: %1 deg").arg(QString::number(errorDeg, 'f', 4)));

    if (errorPlot_ != nullptr) {
        // 只在“本次运动正在记录”时追加采样点。
        // 这样可以避免两类竖线伪影：
        // 1. 设备刚初始化但尚未开始运动时，x 始终为 0，而误差可能从 0 跳到静止位置误差；
        // 2. 运动结束后，x 固定在终点时间，而实际位置仍有最后一点收敛，导致终点被竖直连线。
        const bool appendDuringMotion = curveCaptureActive_ && snapshot.feedback.motionRunning;
        const bool appendTerminalPoint = curveCaptureActive_ && !snapshot.feedback.motionRunning && lastMotionRunning_
                                      && snapshot.feedback.motionTimeS > 0.0;
        if (appendDuringMotion || appendTerminalPoint) {
            errorPlot_->appendSample(snapshot.feedback.motionTimeS, errorDeg);
        }
    }

    if (curveCaptureActive_ && !snapshot.feedback.motionRunning && lastMotionRunning_) {
        curveCaptureActive_ = false;
    }
    lastMotionRunning_ = snapshot.feedback.motionRunning;
}

void MainWindow::onInitCard()
{
    hardwareThread_->enqueueCommand({HardwareCommand::Type::InitializeBoard, collectMotionConfig()});
}

void MainWindow::onCloseCard()
{
    hardwareThread_->enqueueCommand({HardwareCommand::Type::CloseBoard, collectMotionConfig()});
}

void MainWindow::onEnableAxis()
{
    if (!boardInitialized_) {
        appendLog(QStringLiteral("控制卡尚未初始化，无法执行轴使能。"));
        return;
    }

    const MotionConfig config = collectMotionConfig();
    hardwareThread_->enqueueCommand({HardwareCommand::Type::EnableAxis, config});
    appendLog(QStringLiteral("UI 已下发轴使能命令，axis=%1。").arg(config.axis));
}

void MainWindow::onDisableAxis()
{
    if (!boardInitialized_) {
        appendLog(QStringLiteral("控制卡尚未初始化，无法执行轴失能。"));
        return;
    }

    const MotionConfig config = collectMotionConfig();
    hardwareThread_->enqueueCommand({HardwareCommand::Type::DisableAxis, config});
    appendLog(QStringLiteral("UI 已下发轴失能命令，axis=%1。").arg(config.axis));
}

void MainWindow::onStartMotion()
{
    if (!boardInitialized_) {
        appendLog(QStringLiteral("控制卡尚未初始化，无法开始轨迹。"));
        return;
    }

    const MotionConfig config = collectMotionConfig();
    if (config.maxVelDeg < config.minVelDeg || config.maxVelDeg <= 0.0) {
        appendLog(QStringLiteral("参数非法：最大速度必须大于等于最小速度，且必须大于 0。"));
        return;
    }
    if (config.systemPlanningPeriodMs <= 0 || config.systemPlanningPeriodMs > qRound(config.durationS * 1000.0)) {
        appendLog(QStringLiteral("参数非法：系统规划周期必须大于 0，且不能超过总时长。"));
        return;
    }
    if (config.mode == MotionMode::PVT) {
        const qint64 pvtPointCount = estimatePvtPointCount(config.durationS, config.systemPlanningPeriodMs);
        if (pvtPointCount > 5000) {
            appendLog(QStringLiteral("参数非法：PVT 预计生成 %1 个点，已超过控制卡 5000 点上限。请增大系统规划周期或缩短总时长。")
                          .arg(pvtPointCount));
            return;
        }
    }

    {
        QMutexLocker locker(&sharedContext_.requestMutex);
        sharedContext_.motionRequest.config = config;
        sharedContext_.motionRequest.active = true;
        sharedContext_.motionRequest.restartRequested = true;
        ++sharedContext_.motionRequest.sequenceId;
    }

    if (errorPlot_ != nullptr) {
        errorPlot_->clearSamples();
    }
    curveCaptureActive_ = true;
    lastMotionRunning_ = false;

    hardwareThread_->enqueueCommand({HardwareCommand::Type::StartMotion, config});
    appendLog(QStringLiteral("UI 已下发开始命令，mode=%1，系统规划周期=%2ms，UI显示周期=%3ms。")
                  .arg(motionModeToString(config.mode))
                  .arg(config.systemPlanningPeriodMs)
                  .arg(systemConfig_.uiRefreshMs));
}

void MainWindow::onStopMotion()
{
    {
        QMutexLocker locker(&sharedContext_.requestMutex);
        sharedContext_.motionRequest.active = false;
        sharedContext_.motionRequest.restartRequested = false;
    }

    hardwareThread_->enqueueCommand({HardwareCommand::Type::StopMotion, collectMotionConfig()});
    appendLog(QStringLiteral("UI 已下发停止命令。"));
    curveCaptureActive_ = false;
    lastMotionRunning_ = false;
}

void MainWindow::onReadPosition()
{
    if (!boardInitialized_) {
        appendLog(QStringLiteral("控制卡尚未初始化，暂无可读取的反馈快照。"));
        return;
    }

    UiSnapshot snapshot;
    {
        QMutexLocker locker(&sharedContext_.feedbackMutex);
        snapshot.feedback = sharedContext_.feedback;
    }

    const double pulsePerDeg = ui_->rawPerDegSpin->value();
    const double actualDeg = rawToDeg(snapshot.feedback.actualPosRaw, pulsePerDeg);
    const double targetDeg = rawToDeg(snapshot.feedback.targetPosRaw, pulsePerDeg);

    appendLog(QStringLiteral("当前位置快照: actual=%1 deg, target=%2 deg, status=0x%3, modeDisplay=%4")
                  .arg(QString::number(actualDeg, 'f', 4))
                  .arg(QString::number(targetDeg, 'f', 4))
                  .arg(snapshot.feedback.statusWord, 4, 16, QChar('0'))
                  .arg(snapshot.feedback.modeDisplay));
}

void MainWindow::onHardwareLog(const QString &message)
{
    appendLog(QStringLiteral("[硬件线程] %1").arg(message));
}

void MainWindow::onPlannerLog(const QString &message)
{
    appendLog(QStringLiteral("[规划线程] %1").arg(message));
}

void MainWindow::onBoardStateChanged(bool initialized)
{
    boardInitialized_ = initialized;
    ui_->connectionLabel->setText(initialized ? QStringLiteral("已初始化") : QStringLiteral("未初始化"));
}
