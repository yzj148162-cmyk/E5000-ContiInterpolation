#include "hardware/ContiWorker.h"

#include <QTimer>
#include <QThread>
#include <QCoreApplication>
#include <QDir>

#include <algorithm>
#include <cmath>

namespace {
constexpr int kStopPollIntervalMs = 10;
constexpr int kGracefulStopTimeoutMs = 1500;
constexpr int kEmergencyStopTimeoutMs = 250;
constexpr int kContiDiagnosticPeriodMs = 250;
constexpr int kFirstSegmentTimeoutMs = 2000;

qint64 unitToPulse(double unit)
{
    return static_cast<qint64>(std::llround(unit * MotorUnit::kPulsesPerDegree));
}
}

ContiWorker::ContiWorker(QObject *parent)
    : QObject(parent)
    , producerTimer_(new QTimer(this))
    , feedTimer_(new QTimer(this))
    , feedbackTimer_(new QTimer(this))
{
    softwareZeroUnit_.fill(0.0, 8);
    softwareZeroValid_.fill(false, 8);
    pendingJogAutoZeroAxes_.clear();
    producerTimer_->setTimerType(Qt::PreciseTimer);
    feedTimer_->setTimerType(Qt::PreciseTimer);
    feedTimer_->setInterval(5);
    feedbackTimer_->setTimerType(Qt::PreciseTimer);
    feedbackTimer_->setInterval(10);

    connect(producerTimer_, &QTimer::timeout, this, &ContiWorker::produceNextPoint);
    connect(feedTimer_, &QTimer::timeout, this, &ContiWorker::feedCard);
    connect(feedbackTimer_, &QTimer::timeout, this, &ContiWorker::refreshFeedback);
}

void ContiWorker::initializeBoard(const ContiTestConfig &config)
{
    if (boardInitialized_) {
        if (initializedCardNo_ == config.cardNo) {
            emit logMessage(QStringLiteral("控制卡已初始化，基础轴配置已生效。"));
        } else {
            emit logMessage(QStringLiteral("当前已初始化卡号 %1；请先关闭控制卡后再切换卡号。")
                                .arg(initializedCardNo_));
        }
        return;
    }

    QString error;
    if (!card_.initializeBoard(config.cardNo, detectedBoardCount_, error)) {
        enterError(error);
        return;
    }
    if (!configureBaseAxes(config)) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        enterError(QStringLiteral("基础轴配置失败，已关闭控制卡。"));
        return;
    }
    if (!configureFeedbackTrace({config.activeAxis, config.holdAxis}, config.cardNo, error)) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        enterError(QStringLiteral("位置 Trace 配置失败：%1").arg(error));
        return;
    }
    softwareZeroUnit_.fill(0.0, 8);
    softwareZeroValid_.fill(false, 8);
    pendingJogAutoZeroAxes_.clear();
    latestTraceSequence_ = 0;
    latestTraceTimeUs_ = 0;
    boardInitialized_ = true;
    initializedCardNo_ = config.cardNo;
    config_ = config;
    stateText_ = QStringLiteral("控制卡已初始化");
    feedbackTimer_->start();
    emit logMessage(QStringLiteral("控制卡初始化成功：检测到 %1 张卡；仅测试轴 %2、%3 已设置为 %4 pulse/deg。")
                        .arg(detectedBoardCount_)
                        .arg(config.activeAxis)
                        .arg(config.holdAxis)
                        .arg(MotorUnit::kPulsesPerDegree, 0, 'f', 3));
    publishStatus();
}

void ContiWorker::closeBoard()
{
    shutdownHardware();
}

void ContiWorker::startTelemetryRecording()
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡，再开始 Trace 数据记录。"));
        return;
    }
    if (traceAxes_.size() != 2) {
        emit logMessage(QStringLiteral("阶段 A 仅支持两个测试轴的同帧 Trace 记录；当前 Trace 轴数为 %1。")
                            .arg(traceAxes_.size()));
        return;
    }
    TelemetryRunMetadata metadata;
    metadata.cardNo = initializedCardNo_;
    metadata.axes = traceAxes_;
    metadata.traceSamplePeriodUs = card_.traceSamplePeriodUs();
    metadata.rootDirectory = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("records"));
    metadata.description = QStringLiteral("E5000 阶段 A：两轴 Trace 指令(type 5)与实际(type 6)位置记录");
    QString error;
    if (!telemetryRecorder_.start(metadata, error)) {
        emit logMessage(QStringLiteral("开始 Trace 数据记录失败：%1").arg(error));
        publishStatus();
        return;
    }
    emit logMessage(QStringLiteral("Trace 数据记录已开始：%1")
                        .arg(telemetryRecorder_.status().outputDirectory));
    publishStatus();
}

void ContiWorker::stopTelemetryRecording()
{
    const TelemetryRecorderStatus before = telemetryRecorder_.status();
    if (!before.recording) {
        emit logMessage(QStringLiteral("当前没有正在进行的 Trace 数据记录。"));
        return;
    }
    telemetryRecorder_.stop();
    const TelemetryRecorderStatus after = telemetryRecorder_.status();
    emit logMessage(QStringLiteral("Trace 数据记录已停止：写入 %1 帧，丢弃 %2 帧。")
                        .arg(after.writtenFrames)
                        .arg(after.droppedFrames));
    publishStatus();
}

void ContiWorker::shutdownHardware()
{
    // 该槽既由“关闭控制卡”使用，也由主窗口析构时以阻塞方式调用。
    // 关机过程中始终在唯一的硬件线程内访问 LTDMC。
    producerTimer_->stop();
    feedTimer_->stop();
    feedbackTimer_->stop();

    if (!boardInitialized_) {
        return;
    }

    emit logMessage(QStringLiteral("正在执行安全停机：减速停止、超时立即停止、轴失能并关闭控制卡。"));
    safelyStopAllMotionForShutdown();
    if (telemetryRecorder_.status().recording) {
        telemetryRecorder_.appendEvent(QStringLiteral("safe_shutdown"));
        stopTelemetryRecording();
    }
    disableAllEnabledAxesForShutdown();
    traceAxes_.clear();
    latestAxisFeedback_.clear();
    traceFramesRead_ = 0;
    traceStateText_ = QStringLiteral("Trace 已停止");
    latestTraceSequence_ = 0;
    latestTraceTimeUs_ = 0;

    QString error;
    if (!card_.closeBoard(error)) {
        stateText_ = QStringLiteral("控制卡关闭失败（已尝试安全停止和失能）");
        emit logMessage(QStringLiteral("错误：%1").arg(error));
        publishStatus();
        return;
    }
    boardInitialized_ = false;
    detectedBoardCount_ = 0;
    stateText_ = QStringLiteral("控制卡已关闭");
    emit logMessage(stateText_);
    publishStatus();
}

void ContiWorker::enableSelectedAxes(const ContiTestConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_) {
        emit logMessage(QStringLiteral("运动准备或运行期间不能切换轴使能。"));
        return;
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.activeAxis, config.holdAxis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.activeAxis, config.holdAxis}, config.cardNo, traceError)) {
            enterError(QStringLiteral("位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }

    for (const quint16 axis : {config.activeAxis, config.holdAxis}) {
        QString error;
        QString notice;
        if (!card_.setAxisEquivalent(config.cardNo, axis, MotorUnit::kPulsesPerDegree, error)) {
            enterError(QStringLiteral("轴 %1 脉冲当量设置失败：%2").arg(axis).arg(error));
            return;
        }
        if (!card_.clearAxisFaults(config.cardNo, axis, notice, error)) {
            enterError(QStringLiteral("轴 %1 错误清除失败：%2").arg(axis).arg(error));
            return;
        }
        if (!notice.isEmpty()) {
            emit logMessage(QStringLiteral("轴 %1：%2").arg(axis).arg(notice));
        }
        if (!card_.enableAxis(config.cardNo, axis, error)) {
            enterError(QStringLiteral("轴 %1 使能失败：%2").arg(axis).arg(error));
            return;
        }
        enabledAxes_.insert(axis);
        emit logMessage(QStringLiteral("轴 %1 已使能。").arg(axis));
    }
    stateText_ = QStringLiteral("测试轴已使能");
    publishStatus();
}

void ContiWorker::disableSelectedAxes(const ContiTestConfig &config)
{
    if (!boardInitialized_) {
        return;
    }
    if (running_ || preparing_ || pointMoveActive_) {
        stopTest(false);
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.activeAxis, config.holdAxis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.activeAxis, config.holdAxis}, config.cardNo, traceError)) {
            enterError(QStringLiteral("位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }
    for (const quint16 axis : {config.activeAxis, config.holdAxis}) {
        QString error;
        if (!card_.disableAxis(config.cardNo, axis, error)) {
            enterError(QStringLiteral("轴 %1 失能失败：%2").arg(axis).arg(error));
            return;
        }
        enabledAxes_.remove(axis);
        emit logMessage(QStringLiteral("轴 %1 已失能。").arg(axis));
    }
    stateText_ = QStringLiteral("测试轴已失能");
    publishStatus();
}

void ContiWorker::startTest(const ContiTestConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("启动前请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_) {
        emit logMessage(QStringLiteral("已有测试正在准备或运行。"));
        return;
    }
    if (pointMoveActive_) {
        emit logMessage(QStringLiteral("单轴点位运动尚未结束，请先停止该运动后再启动连续插补。"));
        return;
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (config.activeAxis == config.holdAxis) {
        emit logMessage(QStringLiteral("主动轴和保持轴必须不同。"));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.activeAxis, config.holdAxis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.activeAxis, config.holdAxis}, config.cardNo, traceError)) {
            enterError(QStringLiteral("位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }
    if (config.durationS <= 0.0 || config.producerPeriodMs <= 0
        || config.preloadSegments < 2 || config.targetBufferSegments < config.preloadSegments
        || config.lowBufferSegments < 1 || config.lowBufferSegments >= config.targetBufferSegments) {
        emit logMessage(QStringLiteral("测试参数不合法：请检查时长、预压段数和缓冲水位。"));
        return;
    }
    if (!bothAxesEnabled(config)) {
        emit logMessage(QStringLiteral("请先使能当前选择的两个测试轴。"));
        return;
    }
    telemetryRecorder_.appendEvent(QStringLiteral("conti_test_preparing"));

    double activeStart = 0.0;
    double holdStart = 0.0;
    QString error;
    if (!card_.readPositionUnit(config.cardNo, config.activeAxis, activeStart, error)
        || !card_.readPositionUnit(config.cardNo, config.holdAxis, holdStart, error)) {
        enterError(QStringLiteral("读取起始位置失败：%1").arg(error));
        return;
    }

    config_ = config;
    trajectory_.reset(config_, activeStart, holdStart);
    hostQueue_.clear();
    lastQueuedTargetPulse_ = {unitToPulse(activeStart), unitToPulse(holdStart)};
    lastQueuedTargetPulseValid_ = true;
    skippedQuantizedPointCount_ = 0;
    firstEffectivePointTimeS_ = -1.0;
    lastContiDiagnosticMs_ = -1;
    speedRatioNotReadyCount_ = 0;
    nextMark_ = 1;
    lastPushedMark_ = 0;
    listOpen_ = false;
    producerFinished_ = false;
    speedRatioPending_ = false;
    preparing_ = true;
    running_ = false;
    stateText_ = QStringLiteral("正在按 10 ms 周期预生成轨迹");
    producerTimer_->setInterval(config_.producerPeriodMs);

    emit logMessage(QStringLiteral("轨迹已重置：轴 %1 指令起点=%2°，轴 %3 指令起点=%4°；等待预压 %5 段。")
                        .arg(config_.activeAxis).arg(activeStart, 0, 'f', 4)
                        .arg(config_.holdAxis).arg(holdStart, 0, 'f', 4)
                        .arg(config_.preloadSegments));
    const double activeEnd = activeStart + config_.activeDeltaUnit;
    const double holdEnd = holdStart + (config_.stage == TestStage::DualAxis ? config_.holdDeltaUnit : 0.0);
    emit logMessage(QStringLiteral("插补量化诊断：1 pulse=%1°；起点脉冲=(%2,%3)，理论终点脉冲=(%4,%5)。")
                        .arg(1.0 / MotorUnit::kPulsesPerDegree, 0, 'f', 8)
                        .arg(lastQueuedTargetPulse_[0])
                        .arg(lastQueuedTargetPulse_[1])
                        .arg(unitToPulse(activeEnd))
                        .arg(unitToPulse(holdEnd)));
    produceNextPoint(); // 第一个点为 t=0，不必额外等待一个周期。
    producerTimer_->start();
    publishStatus();
}

void ContiWorker::stopTest(bool emergency)
{
    telemetryRecorder_.appendEvent(emergency
                                       ? QStringLiteral("motion_immediate_stop")
                                       : QStringLiteral("motion_decelerated_stop"));
    producerTimer_->stop();
    feedTimer_->stop();

    if (listOpen_) {
        QString error;
        if (!card_.stop(config_, emergency, error)) {
            emit logMessage(QStringLiteral("停止连续插补失败：%1").arg(error));
        }
        if (!card_.closeList(config_, error)) {
            emit logMessage(QStringLiteral("关闭连续插补缓冲区失败：%1").arg(error));
        }
    }

    if (pointMoveActive_) {
        QString error;
        if (!card_.stopAxis(initializedCardNo_, pointMoveAxis_, emergency, error)) {
            emit logMessage(QStringLiteral("停止单轴点位运动失败：%1").arg(error));
        }
        pointMoveActive_ = false;
    }

    listOpen_ = false;
    preparing_ = false;
    running_ = false;
    speedRatioPending_ = false;
    hostQueue_.clear();
    stateText_ = emergency ? QStringLiteral("已立即停止") : QStringLiteral("已减速停止");
    emit logMessage(stateText_);
    publishStatus();
}

void ContiWorker::refreshFeedback()
{
    if (boardInitialized_) {
        const bool traceReadSucceeded = pollTraceFeedback();
        if (!traceReadSucceeded && running_) {
            enterError(QStringLiteral("运行中位置 Trace 读取失败：%1").arg(traceStateText_));
            return;
        }
        if (traceReadSucceeded) {
            const QSet<quint16> pendingAxes = pendingJogAutoZeroAxes_;
            for (const quint16 axis : pendingAxes) {
                if (axis < static_cast<quint16>(latestAxisFeedback_.size())
                    && latestAxisFeedback_.at(axis).traceSampleValid) {
                    softwareZeroUnit_[axis] = latestAxisFeedback_.at(axis).encoderPositionUnit;
                    softwareZeroValid_[axis] = true;
                    pendingJogAutoZeroAxes_.remove(axis);
                    emit logMessage(QStringLiteral("单轴点动：轴 %1 已取得 Trace 首帧，已自动设为软件零位。")
                                        .arg(axis));
                }
            }
        }
        if (pointMoveActive_ && pointMoveDiagnosticPending_ && traceFramesRead_ > 0
            && pointMoveAxis_ < static_cast<quint16>(latestAxisFeedback_.size())) {
            const AxisFeedback &feedback = latestAxisFeedback_.at(pointMoveAxis_);
            if (feedback.traceSampleValid) {
                const QString cardTarget = pointMoveCardTargetValid_
                    ? QString::number(pointMoveCardTargetUnit_, 'f', 4)
                    : QStringLiteral("读取失败");
                const double zero = pointMoveAxis_ < static_cast<quint16>(softwareZeroUnit_.size())
                    ? softwareZeroUnit_.at(pointMoveAxis_) : 0.0;
                emit logMessage(QStringLiteral("点位只读诊断（轴 %1）：请求相对位移=%2°，软件零位（仅显示）=%3°，"
                                               "卡内目标读回=%4°，Trace 绝对指令=%5°，Trace 绝对实际=%6°")
                                    .arg(pointMoveAxis_)
                                    .arg(pointMoveRequestedTargetUnit_, 0, 'f', 4)
                                    .arg(zero, 0, 'f', 4)
                                    .arg(cardTarget)
                                    .arg(feedback.commandPositionUnit, 0, 'f', 4)
                                    .arg(feedback.encoderPositionUnit, 0, 'f', 4));
                pointMoveDiagnosticPending_ = false;
            }
        }
        if (pointMoveActive_) {
            bool done = false;
            QString pointStateError;
            if (!card_.axisMotionDone(pointMoveAxis_, done, pointStateError)) {
                enterError(QStringLiteral("读取单轴点位运动状态失败：%1").arg(pointStateError));
                return;
            }
            if (done) {
            pointMoveActive_ = false;
            stateText_ = QStringLiteral("轴 %1 点位运动完成").arg(pointMoveAxis_);
            emit logMessage(stateText_);
        }
        }
        refreshAxisStates();
        publishStatus();
    }
}

void ContiWorker::enableJogAxis(const SingleAxisJogConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_) {
        emit logMessage(QStringLiteral("连续插补准备、运行或单轴点位运动期间不能切换点动测试轴。"));
        return;
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.axis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.axis}, config.cardNo, traceError)) {
            enterError(QStringLiteral("单轴位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }

    QString error;
    QString notice;
    if (!card_.setAxisEquivalent(config.cardNo, config.axis, MotorUnit::kPulsesPerDegree, error)) {
        enterError(QStringLiteral("轴 %1 脉冲当量设置失败：%2").arg(config.axis).arg(error));
        return;
    }
    if (!card_.clearAxisFaults(config.cardNo, config.axis, notice, error)) {
        enterError(QStringLiteral("轴 %1 错误清除失败：%2").arg(config.axis).arg(error));
        return;
    }
    if (!notice.isEmpty()) {
        emit logMessage(QStringLiteral("轴 %1：%2").arg(config.axis).arg(notice));
    }
    if (!card_.enableAxis(config.cardNo, config.axis, error)) {
        enterError(QStringLiteral("轴 %1 使能失败：%2").arg(config.axis).arg(error));
        return;
    }
    enabledAxes_.insert(config.axis);
    pendingJogAutoZeroAxes_.insert(config.axis);
    // 点动相对位置的初始基准取自使能后的第一帧 Trace 实际位置。
    // 该零位只服务于界面显示，不会写入控制卡，也不参与相对 PMove 下发。
    if (pollTraceFeedback() && config.axis < static_cast<quint16>(latestAxisFeedback_.size())
        && latestAxisFeedback_.at(config.axis).traceSampleValid) {
        softwareZeroUnit_[config.axis] = latestAxisFeedback_.at(config.axis).encoderPositionUnit;
        softwareZeroValid_[config.axis] = true;
        pendingJogAutoZeroAxes_.remove(config.axis);
        emit logMessage(QStringLiteral("单轴点动：轴 %1 已使能，已自动以当前 Trace 实际位置设为软件零位。")
                            .arg(config.axis));
    } else {
        emit logMessage(QStringLiteral("单轴点动：轴 %1 已使能，等待有效 Trace 首帧后自动设置软件零位。")
                            .arg(config.axis));
    }
    stateText_ = QStringLiteral("单轴点动测试轴已使能");
    publishStatus();
}

void ContiWorker::disableJogAxis(const SingleAxisJogConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (running_ || preparing_) {
        emit logMessage(QStringLiteral("连续插补准备或运行期间不能失能点动测试轴。"));
        return;
    }
    if (pointMoveActive_ && pointMoveAxis_ != config.axis) {
        emit logMessage(QStringLiteral("轴 %1 的点位运动尚未结束，不能失能轴 %2。")
                            .arg(pointMoveAxis_)
                            .arg(config.axis));
        return;
    }
    if (pointMoveActive_ && !safelyStopPointAxis(config.axis, QStringLiteral("点动轴失能"))) {
        emit logMessage(QStringLiteral("轴 %1 未确认停止，拒绝失能。") .arg(config.axis));
        return;
    }
    if (!enabledAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("轴 %1 当前未由本程序使能。") .arg(config.axis));
        return;
    }

    QString error;
    if (!card_.disableAxis(initializedCardNo_, config.axis, error)) {
        enterError(QStringLiteral("轴 %1 失能失败：%2").arg(config.axis).arg(error));
        return;
    }
    enabledAxes_.remove(config.axis);
    pendingJogAutoZeroAxes_.remove(config.axis);
    stateText_ = QStringLiteral("单轴点动测试轴已失能");
    emit logMessage(QStringLiteral("单轴点动：轴 %1 已失能。") .arg(config.axis));
    publishStatus();
}

void ContiWorker::setJogAxisZero(const SingleAxisJogConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_) {
        emit logMessage(QStringLiteral("运动准备或运行期间不能修改点动软件零位。"));
        return;
    }
    if (config.cardNo != initializedCardNo_ || !enabledAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("请先使能当前选择的点动测试轴，再设置软件零位。"));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.axis}) {
        QString error;
        if (!configureFeedbackTrace({config.axis}, config.cardNo, error)) {
            emit logMessage(QStringLiteral("设置软件零位前 Trace 重配失败：%1").arg(error));
            return;
        }
    }
    if (!pollTraceFeedback() || config.axis >= static_cast<quint16>(latestAxisFeedback_.size())) {
        emit logMessage(QStringLiteral("设置软件零位失败：尚未获得当前测试轴的有效 Trace 实际位置。"));
        return;
    }
    const AxisFeedback &feedback = latestAxisFeedback_.at(config.axis);
    if (!feedback.traceSampleValid) {
        emit logMessage(QStringLiteral("设置软件零位失败：当前测试轴尚未获得有效 Trace 实际位置。"));
        return;
    }
    softwareZeroUnit_[config.axis] = feedback.encoderPositionUnit;
    softwareZeroValid_[config.axis] = true;
    pendingJogAutoZeroAxes_.remove(config.axis);
    emit logMessage(QStringLiteral("点动软件零位已设置：轴 %1，绝对位置 %2° 现定义为相对 0°；未写入控制卡。")
                        .arg(config.axis)
                        .arg(softwareZeroUnit_.at(config.axis), 0, 'f', 4));
    publishStatus();
}

void ContiWorker::startPointMove(const SingleAxisJogConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_) {
        emit logMessage(QStringLiteral("连续插补准备或运行期间禁止单轴点动。"));
        return;
    }
    if (pointMoveActive_ && config.axis != pointMoveAxis_) {
        emit logMessage(QStringLiteral("轴 %1 的单轴点位运动尚未结束，不能切换到轴 %2。")
                            .arg(pointMoveAxis_)
                            .arg(config.axis));
        return;
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (!enabledAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("请先使能单轴点动测试轴 %1。").arg(config.axis));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.axis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.axis}, config.cardNo, traceError)) {
            enterError(QStringLiteral("单轴位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }
    QString error;
    if (!card_.startPointMove(config, error)) {
        enterError(QStringLiteral("轴 %1 点位运动启动失败：%2").arg(config.axis).arg(error));
        return;
    }
    pointMoveAxis_ = config.axis;
    pointMoveActive_ = true;
    pointMoveDiagnosticPending_ = true;
    pointMoveRequestedTargetUnit_ = config.targetPositionUnit;
    pointMoveCardTargetValid_ = card_.readTargetPositionUnit(config.axis,
                                                              pointMoveCardTargetUnit_, error);
    if (!pointMoveCardTargetValid_) {
        emit logMessage(QStringLiteral("点位只读诊断：读取轴 %1 的卡内目标位置失败：%2")
                            .arg(config.axis)
                            .arg(error));
    }
    stateText_ = QStringLiteral("轴 %1 点位运动中").arg(config.axis);
    emit logMessage(QStringLiteral("单轴相对点动：轴 %1，相对位移=%2°，目标速度 %3°/s。")
                        .arg(config.axis)
                        .arg(config.targetPositionUnit, 0, 'f', 4)
                        .arg(config.maxVelocityUnitPerSecond, 0, 'f', 4));
    publishStatus();
}

void ContiWorker::stopPointMove(bool emergency)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (!pointMoveActive_) {
        emit logMessage(QStringLiteral("当前没有正在执行的单轴点位运动。"));
        return;
    }
    const quint16 axis = pointMoveAxis_;
    if (emergency) {
        QString error;
        if (!card_.stopAxis(initializedCardNo_, axis, true, error)) {
            enterError(QStringLiteral("停止单轴点位运动失败：%1").arg(error));
            return;
        }
        pointMoveActive_ = false;
    } else if (!safelyStopPointAxis(axis, QStringLiteral("单轴点位减速停止"))) {
        enterError(QStringLiteral("轴 %1 停止超时或失败。") .arg(axis));
        return;
    }
    stateText_ = emergency ? QStringLiteral("单轴点位运动已立即停止")
                            : QStringLiteral("单轴点位运动已减速停止");
    emit logMessage(stateText_);
    publishStatus();
}

bool ContiWorker::waitForAxisStop(quint16 axis, int timeoutMs) const
{
    const int attempts = std::max(1, timeoutMs / kStopPollIntervalMs);
    for (int attempt = 0; attempt < attempts; ++attempt) {
        bool done = false;
        QString error;
        if (card_.axisMotionDone(axis, done, error) && done) {
            return true;
        }
        QThread::msleep(kStopPollIntervalMs);
    }
    bool done = false;
    QString error;
    return card_.axisMotionDone(axis, done, error) && done;
}

bool ContiWorker::waitForContiStop(int timeoutMs) const
{
    const int attempts = std::max(1, timeoutMs / kStopPollIntervalMs);
    for (int attempt = 0; attempt < attempts; ++attempt) {
        if (card_.isContiMotionDone(config_)) {
            return true;
        }
        QThread::msleep(kStopPollIntervalMs);
    }
    return card_.isContiMotionDone(config_);
}

bool ContiWorker::safelyStopPointAxis(quint16 axis, const QString &reason)
{
    QString error;
    const bool decelRequested = card_.stopAxis(initializedCardNo_, axis, false, error);
    if (decelRequested && waitForAxisStop(axis, kGracefulStopTimeoutMs)) {
        pointMoveActive_ = false;
        return true;
    }

    emit logMessage(QStringLiteral("%1：轴 %2 减速停止未在 %3 ms 内确认完成%4，改为立即停止。")
                        .arg(reason)
                        .arg(axis)
                        .arg(kGracefulStopTimeoutMs)
                        .arg(error.isEmpty() ? QString() : QStringLiteral("（%1）").arg(error)));
    error.clear();
    if (!card_.stopAxis(initializedCardNo_, axis, true, error)
        || !waitForAxisStop(axis, kEmergencyStopTimeoutMs)) {
        emit logMessage(QStringLiteral("%1：轴 %2 立即停止未确认完成%3。")
                            .arg(reason)
                            .arg(axis)
                            .arg(error.isEmpty() ? QString() : QStringLiteral("（%1）").arg(error)));
        return false;
    }
    pointMoveActive_ = false;
    return true;
}

void ContiWorker::safelyStopAllMotionForShutdown()
{
    producerTimer_->stop();
    feedTimer_->stop();

    if (listOpen_) {
        QString error;
        const bool decelRequested = card_.stop(config_, false, error);
        if (!decelRequested || !waitForContiStop(kGracefulStopTimeoutMs)) {
            emit logMessage(QStringLiteral("连续插补减速停止未在 %1 ms 内确认完成%2，改为立即停止。")
                                .arg(kGracefulStopTimeoutMs)
                                .arg(error.isEmpty() ? QString() : QStringLiteral("（%1）").arg(error)));
            error.clear();
            if (!card_.stop(config_, true, error) || !waitForContiStop(kEmergencyStopTimeoutMs)) {
                emit logMessage(QStringLiteral("连续插补立即停止未确认完成%1。")
                                    .arg(error.isEmpty() ? QString() : QStringLiteral("（%1）").arg(error)));
            }
        }
        error.clear();
        if (!card_.closeList(config_, error)) {
            emit logMessage(QStringLiteral("安全停机时关闭连续插补缓冲区失败：%1").arg(error));
        }
    }

    if (pointMoveActive_) {
        safelyStopPointAxis(pointMoveAxis_, QStringLiteral("安全停机"));
    }

    listOpen_ = false;
    preparing_ = false;
    running_ = false;
    pointMoveActive_ = false;
    hostQueue_.clear();
}

void ContiWorker::disableAllEnabledAxesForShutdown()
{
    const QList<quint16> axes = enabledAxes_.values();
    for (const quint16 axis : axes) {
        QString error;
        if (!card_.disableAxis(initializedCardNo_, axis, error)) {
            emit logMessage(QStringLiteral("安全停机时轴 %1 失能失败：%2").arg(axis).arg(error));
        } else {
            emit logMessage(QStringLiteral("安全停机：轴 %1 已失能。").arg(axis));
        }
    }
    enabledAxes_.clear();
}

void ContiWorker::produceNextPoint()
{
    if (!preparing_ && !running_) {
        return;
    }
    if (trajectory_.hasNext()) {
        const ContiPoint point = trajectory_.nextPoint();
        const std::array<qint64, 2> targetPulse = {
            unitToPulse(point.targetUnit[0]),
            unitToPulse(point.targetUnit[1])
        };

        // 连续插补实际按脉冲坐标执行。跳过与上一已接受目标完全相同的点，
        // 避免五次曲线起步阶段形成零长度首段并卡住 currentMark。
        if (lastQueuedTargetPulseValid_ && targetPulse == lastQueuedTargetPulse_) {
            ++skippedQuantizedPointCount_;
        } else {
            if (firstEffectivePointTimeS_ < 0.0) {
                firstEffectivePointTimeS_ = point.timeS;
                emit logMessage(QStringLiteral("首个有效插补段：规划 t=%1 s，目标=(%2°,%3°)，脉冲=(%4,%5)，相对起点增量=(%6,%7) pulse。")
                                    .arg(point.timeS, 0, 'f', 3)
                                    .arg(point.targetUnit[0], 0, 'f', 6)
                                    .arg(point.targetUnit[1], 0, 'f', 6)
                                    .arg(targetPulse[0])
                                    .arg(targetPulse[1])
                                    .arg(targetPulse[0] - lastQueuedTargetPulse_[0])
                                    .arg(targetPulse[1] - lastQueuedTargetPulse_[1]));
            }
            hostQueue_.enqueue(point);
            lastQueuedTargetPulse_ = targetPulse;
            lastQueuedTargetPulseValid_ = true;
        }
    } else {
        producerFinished_ = true;
        producerTimer_->stop();
        if (preparing_ && hostQueue_.size() < config_.preloadSegments) {
            enterError(QStringLiteral("轨迹生成结束，但脉冲域仅得到 %1 个有效段（预压要求 %2 个，已跳过 %3 个重复点）；请增大位移、缩短轨迹时长或降低预压段数。")
                           .arg(hostQueue_.size())
                           .arg(config_.preloadSegments)
                           .arg(skippedQuantizedPointCount_));
            return;
        }
    }

    if (preparing_ && hostQueue_.size() >= config_.preloadSegments) {
        startAfterPreload();
    }
    publishStatus();
}

bool ContiWorker::startAfterPreload()
{
    QString error;
    if (!card_.configureAndOpen(config_, error)) {
        enterError(error);
        return false;
    }
    listOpen_ = true;

    for (int i = 0; i < config_.preloadSegments; ++i) {
        if (!pushOnePoint()) {
            return false;
        }
    }
    if (!card_.start(config_, error)) {
        enterError(error);
        return false;
    }

    preparing_ = false;
    running_ = true;
    // start_list 成功后控制卡仍可能尚未规划当前段；在线变倍率改由 feedCard()
    // 在坐标系进入运行状态后下发，5049 时保留该标志并重试。
    speedRatioPending_ = true;
    speedRatioNotReadyCount_ = 0;
    contiRunElapsed_.start();
    lastContiDiagnosticMs_ = -1;
    stateText_ = QStringLiteral("连续插补运行中");
    feedTimer_->start();
    emit logMessage(QStringLiteral("已预压 %1 个有效段并启动连续插补；已跳过 %2 个脉冲重复点，首个有效点 t=%3 s，速度倍率=%4。")
                        .arg(config_.preloadSegments)
                        .arg(skippedQuantizedPointCount_)
                        .arg(firstEffectivePointTimeS_, 0, 'f', 3)
                        .arg(config_.speedRatio, 0, 'f', 2));
    return true;
}

bool ContiWorker::pushOnePoint()
{
    if (hostQueue_.isEmpty()) {
        return true;
    }
    QString error;
    const ContiPoint point = hostQueue_.dequeue();
    if (!card_.pushLine(config_, point, nextMark_, error)) {
        enterError(QStringLiteral("压入 mark=%1 失败：%2").arg(nextMark_).arg(error));
        return false;
    }
    lastPushedMark_ = nextMark_;
    ++nextMark_;
    return true;
}

void ContiWorker::feedCard()
{
    if (!running_) {
        return;
    }

    const long current = card_.currentMark(config_);
    long remaining = card_.remainSpace(config_);
    long buffered = std::max(0L, lastPushedMark_ - current);

    if (buffered < config_.lowBufferSegments && hostQueue_.isEmpty() && !producerFinished_) {
        emit logMessage(QStringLiteral("缓冲偏低（%1 段），等待下一周期轨迹点。") .arg(buffered));
    }

    while (buffered < config_.targetBufferSegments && remaining > 0 && !hostQueue_.isEmpty()) {
        if (!pushOnePoint()) {
            return;
        }
        ++buffered;
        --remaining;
    }

    const short state = card_.runState(config_);
    if (state == 3 || state == 4) {
        enterError(QStringLiteral("连续插补意外停止，runState=%1。") .arg(state));
        return;
    }

    if (speedRatioPending_ && state == 0) {
        QString ratioError;
        const ContiSpeedRatioResult result = card_.changeSpeedRatio(config_, ratioError);
        if (result == ContiSpeedRatioResult::Applied) {
            speedRatioPending_ = false;
            emit logMessage(QStringLiteral("连续插补在线速度倍率已生效：%1。")
                                .arg(config_.speedRatio, 0, 'f', 2));
        } else if (result == ContiSpeedRatioResult::Failed) {
            enterError(ratioError);
            return;
        } else {
            ++speedRatioNotReadyCount_;
            if (speedRatioNotReadyCount_ == 1 || speedRatioNotReadyCount_ % 40 == 0) {
                emit logMessage(QStringLiteral("在线速度倍率等待首段规划：返回 5049，已重试 %1 次。")
                                    .arg(speedRatioNotReadyCount_));
            }
        }
    }

    const qint64 elapsedMs = contiRunElapsed_.isValid() ? contiRunElapsed_.elapsed() : 0;
    if (lastContiDiagnosticMs_ < 0 || elapsedMs - lastContiDiagnosticMs_ >= kContiDiagnosticPeriodMs) {
        lastContiDiagnosticMs_ = elapsedMs;
        emit logMessage(QStringLiteral("连续插补诊断：t=%1 ms，runState=%2，currentMark=%3，pushedMark=%4，缓冲余量=%5，上位机队列=%6，重复点=%7。")
                            .arg(elapsedMs)
                            .arg(state)
                            .arg(current)
                            .arg(lastPushedMark_)
                            .arg(remaining)
                            .arg(hostQueue_.size())
                            .arg(skippedQuantizedPointCount_));
    }
    if (current < 1 && elapsedMs >= kFirstSegmentTimeoutMs) {
        enterError(QStringLiteral("启动后 %1 ms 控制卡仍未消费首个有效段（currentMark=%2，pushedMark=%3，缓冲余量=%4）；已安全停止，需检查首段与控制卡插补状态。")
                       .arg(elapsedMs)
                       .arg(current)
                       .arg(lastPushedMark_)
                       .arg(remaining));
        return;
    }

    if (producerFinished_ && hostQueue_.isEmpty()
        && (current >= lastPushedMark_ || state == 2 || state == 4)) {
        finishRun(QStringLiteral("五次多项式轨迹已执行完成。"));
        return;
    }

    publishStatus();
}

void ContiWorker::finishRun(const QString &message)
{
    telemetryRecorder_.appendEvent(QStringLiteral("conti_test_completed"));
    producerTimer_->stop();
    feedTimer_->stop();
    QString error;
    if (listOpen_ && !card_.closeList(config_, error)) {
        emit logMessage(QStringLiteral("轨迹结束后关闭缓冲区失败：%1").arg(error));
    }
    listOpen_ = false;
    preparing_ = false;
    running_ = false;
    speedRatioPending_ = false;
    stateText_ = QStringLiteral("轨迹完成");
    emit logMessage(message);
    publishStatus();
}

void ContiWorker::enterError(const QString &message)
{
    telemetryRecorder_.appendEvent(QStringLiteral("control_error: %1").arg(message));
    producerTimer_->stop();
    feedTimer_->stop();
    if (listOpen_) {
        QString ignored;
        card_.stop(config_, true, ignored);
        card_.closeList(config_, ignored);
    }
    if (pointMoveActive_) {
        QString ignored;
        card_.stopAxis(initializedCardNo_, pointMoveAxis_, true, ignored);
        pointMoveActive_ = false;
    }
    listOpen_ = false;
    preparing_ = false;
    running_ = false;
    speedRatioPending_ = false;
    stateText_ = QStringLiteral("错误");
    emit logMessage(QStringLiteral("错误：%1").arg(message));
    publishStatus();
}

void ContiWorker::publishStatus()
{
    ContiStatus status;
    status.boardInitialized = boardInitialized_;
    status.detectedBoardCount = detectedBoardCount_;
    status.cardNo = initializedCardNo_;
    status.running = running_;
    status.stateText = stateText_;
    status.hostQueueSize = hostQueue_.size();
    status.pushedMark = lastPushedMark_;
    if (listOpen_) {
        status.currentMark = card_.currentMark(config_);
        status.remainSpace = card_.remainSpace(config_);
        status.runState = card_.runState(config_);
    }
    status.traceConfigured = card_.traceConfigured();
    status.traceEverRead = card_.traceEverRead();
    status.traceFramesRead = card_.traceFramesRead();
    status.traceLastApiResult = card_.traceLastApiResult();
    status.traceStateText = card_.traceStateText();
    status.latestTraceSequence = latestTraceSequence_;
    status.latestTraceTimeUs = latestTraceTimeUs_;
    status.traceSamplePeriodUs = card_.traceSamplePeriodUs();
    status.recorder = telemetryRecorder_.status();
    status.softwareZeroUnit = softwareZeroUnit_;
    status.softwareZeroValid = softwareZeroValid_;
    if (boardInitialized_) {
        status.busErrorCode = card_.busErrorCode();
        status.axisFeedback = latestAxisFeedback_;
    }
    emit statusChanged(status);
}

bool ContiWorker::configureBaseAxes(const ContiTestConfig &config)
{
    for (const quint16 axis : {config.activeAxis, config.holdAxis}) {
        QString error;
        if (!card_.setAxisEquivalent(config.cardNo, axis, MotorUnit::kPulsesPerDegree, error)) {
            emit logMessage(QStringLiteral("轴 %1 基础配置失败：%2").arg(axis).arg(error));
            return false;
        }
    }
    return true;
}

#if 0 // Trace PDO 的旧实现：保留到本次重构完成前作对照，实际访问已迁入独占硬件线程。
bool ContiWorker::configureFeedbackTrace(const QVector<quint16> &axes,
                                         quint16 cardNo,
                                         QString &errorMessage)
{
    if (axes.isEmpty() || axes.size() > 8) {
        errorMessage = QStringLiteral("Trace 测试轴数量必须在 1 到 8 之间");
        return false;
    }
    QSet<quint16> uniqueAxes;
    for (const quint16 axis : axes) {
        uniqueAxes.insert(axis);
    }
    if (uniqueAxes.size() != axes.size()) {
        errorMessage = QStringLiteral("Trace 测试轴不能重复");
        return false;
    }

    traceAxes_ = axes;
    latestAxisFeedback_.clear();
    latestAxisFeedback_.reserve(8);
    for (quint16 axis = 0; axis < 8; ++axis) {
        AxisFeedback feedback;
        feedback.axis = axis;
        feedback.errorText = traceAxes_.contains(axis)
            ? QStringLiteral("等待 Trace 首帧")
            : QStringLiteral("未纳入当前测试轴 Trace");
        latestAxisFeedback_.push_back(feedback);
    }

    RuntimeTraceSlaveReader::ReaderConfig traceConfig;
    traceConfig.cardNo = cardNo;
    traceConfig.samplePeriodUs = 500;
    traceConfig.traceBaseCycleUs = 500;
    traceConfig.objects.reserve(traceAxes_.size() * 2);

    // 保持 0523_final 的对象顺序：所有指令位置(type=5)在前，所有实际位置(type=6)在后。
    for (int index = 0; index < traceAxes_.size(); ++index) {
        RuntimeTraceSlaveReader::ObjectConfig object;
        object.logicalIndex = index;
        object.dataType = 5;
        object.dataIndex = traceAxes_.at(index);
        object.dataSubIndex = 0;
        object.slaveId = 0;
        object.apiDataBytes = 4;
        object.valueBytes = 4;
        object.scale = 1.0 / MotorUnit::kPulsesPerDegree;
        traceConfig.objects.push_back(object);
    }
    for (int index = 0; index < traceAxes_.size(); ++index) {
        RuntimeTraceSlaveReader::ObjectConfig object;
        object.logicalIndex = traceAxes_.size() + index;
        object.dataType = 6;
        object.dataIndex = traceAxes_.at(index);
        object.dataSubIndex = 0;
        object.slaveId = 0;
        object.apiDataBytes = 4;
        object.valueBytes = 4;
        object.scale = 1.0 / MotorUnit::kPulsesPerDegree;
        traceConfig.objects.push_back(object);
    }

    if (!feedbackTrace_.configure(traceConfig)) {
        errorMessage = QStringLiteral("dmc_trace_* 返回码=%1")
                           .arg(feedbackTrace_.lastApiResult());
        traceStateText_ = QStringLiteral("Trace 配置失败");
        return false;
    }
    traceFramesRead_ = 0;
    traceStateText_ = QStringLiteral("Trace 已配置：500 us，同帧 type 5/6 位置");
    QStringList axisText;
    for (const quint16 axis : traceAxes_) {
        axisText << QString::number(axis);
    }
    emit logMessage(QStringLiteral("位置 Trace 已配置：轴 %1；500 us 采样，type 5 指令位置 / type 6 实际位置。")
                        .arg(axisText.join(QStringLiteral("、"))));
    return true;
}

bool ContiWorker::pollTraceFeedback()
{
    traceFramesRead_ = feedbackTrace_.readTraceCached();
    if (traceFramesRead_ < 0) {
        traceStateText_ = QStringLiteral("Trace 读取失败，API 返回码=%1")
                              .arg(feedbackTrace_.lastApiResult());
        return false;
    }

    const std::vector<RuntimeTraceSlaveReader::Sample> samples = feedbackTrace_.takeSamples();
    if (samples.empty()) {
        traceStateText_ = feedbackTrace_.hasEverRead()
            ? QStringLiteral("Trace 正常，无新增帧")
            : QStringLiteral("等待 Trace 首帧");
        return true;
    }

    for (const RuntimeTraceSlaveReader::Sample &sample : samples) {
        if (sample.values.size() < static_cast<std::size_t>(traceAxes_.size() * 2)) {
            traceStateText_ = QStringLiteral("Trace 帧对象数量不足");
            return false;
        }
        for (int index = 0; index < traceAxes_.size(); ++index) {
            AxisFeedback &feedback = latestAxisFeedback_[traceAxes_.at(index)];
            feedback.commandPositionUnit = sample.values[static_cast<std::size_t>(index)];
            feedback.encoderPositionUnit = sample.values[static_cast<std::size_t>(traceAxes_.size() + index)];
            feedback.traceSampleValid = true;
        }
    }
    traceStateText_ = QStringLiteral("Trace 正常：本次读取 %1 帧").arg(traceFramesRead_);
    return true;
}

void ContiWorker::refreshAxisStates()
{
    for (const quint16 axis : traceAxes_) {
        AxisFeedback axisState;
        card_.readAxisFeedback(initializedCardNo_, axis, axisState);
        AxisFeedback &feedback = latestAxisFeedback_[axis];
        feedback.stateMachine = axisState.stateMachine;
        feedback.axisErrorCode = axisState.axisErrorCode;
        if (!axisState.valid) {
            feedback.errorText = axisState.errorText;
        } else if (feedback.traceSampleValid) {
            feedback.errorText.clear();
        }
        feedback.valid = axisState.valid && feedback.traceSampleValid;
    }
}

#endif

bool ContiWorker::configureFeedbackTrace(const QVector<quint16> &axes,
                                         quint16,
                                         QString &errorMessage)
{
    if (telemetryRecorder_.status().recording) {
        errorMessage = QStringLiteral("Trace 数据记录进行中，不能切换测试轴或重配 Trace；请先停止记录。");
        return false;
    }
    if (!card_.configureTrace(axes, errorMessage)) {
        return false;
    }
    traceAxes_ = card_.traceAxes();
    latestAxisFeedback_ = card_.axisFeedback();
    traceFramesRead_ = card_.traceFramesRead();
    traceStateText_ = card_.traceStateText();
    QVector<TraceTelemetryFrame> frames = card_.takeTraceTelemetryFrames();
    if (!frames.isEmpty()) {
        const long currentMark = listOpen_ ? card_.currentMark(config_) : -1;
        for (TraceTelemetryFrame &frame : frames) {
            frame.currentMark = static_cast<qint32>(currentMark);
            frame.pushedMark = static_cast<qint32>(lastPushedMark_);
        }
        latestTraceSequence_ = frames.constLast().traceSequence;
        latestTraceTimeUs_ = frames.constLast().traceTimeUs;
        telemetryRecorder_.pushFrames(frames);
    }
    return true;
}

bool ContiWorker::pollTraceFeedback()
{
    QString error;
    if (!card_.pollFeedback(error)) {
        traceFramesRead_ = card_.traceFramesRead();
        traceStateText_ = error;
        return false;
    }
    traceAxes_ = card_.traceAxes();
    latestAxisFeedback_ = card_.axisFeedback();
    traceFramesRead_ = card_.traceFramesRead();
    traceStateText_ = card_.traceStateText();
    return true;
}

void ContiWorker::refreshAxisStates()
{
    latestAxisFeedback_ = card_.axisFeedback();
}

bool ContiWorker::bothAxesEnabled(const ContiTestConfig &config) const
{
    return enabledAxes_.contains(config.activeAxis) && enabledAxes_.contains(config.holdAxis);
}
