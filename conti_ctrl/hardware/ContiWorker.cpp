#include "hardware/ContiWorker.h"

#include <QTimer>
#include <QThread>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>
#include <QStringList>

#include <algorithm>
#include <cmath>

namespace {
constexpr int kStopPollIntervalMs = 10;
constexpr int kGracefulStopTimeoutMs = 1500;
constexpr int kEmergencyStopTimeoutMs = 250;
constexpr int kContiDiagnosticPeriodMs = 250;
constexpr int kFirstSegmentTimeoutMs = 2000;
constexpr int kCompletionStableDwellMs = 150;
constexpr int kVelocityPlotPublishIntervalMs = 50;
constexpr int kTraceDelayPlotPublishIntervalMs = 50;
constexpr int kTorquePlotPublishIntervalMs = 50;
constexpr int kTraceDelayStopTimeoutMs = 2000;
constexpr double kDefaultTracePositionDelayMs = 8.0;
constexpr double kCompletionPositionToleranceDeg = 0.02;
constexpr double kCompletionVelocityToleranceDegPerSecond = 1.0;
constexpr double kDuplicatePositionToleranceDeg = 1e-12;
constexpr double kTwoPi = 6.283185307179586476925286766559;

qint64 degreeToEstimatedPulse(double degree)
{
    return static_cast<qint64>(std::llround(degree * MotorUnit::kPhysicalPulsesPerDegree));
}

bool sameCardUnitDefinition(double left, double right)
{
    return std::abs(left - right) < 1e-12;
}

bool supportedCardUnitDefinition(double degreesPerCardUnit)
{
    return sameCardUnitDefinition(degreesPerCardUnit, 1.0)
        || sameCardUnitDefinition(degreesPerCardUnit, 0.1)
        || sameCardUnitDefinition(degreesPerCardUnit, 0.01);
}

QString velocityTrajectoryName(VelocityTrajectoryType type)
{
    switch (type) {
    case VelocityTrajectoryType::Quintic:
        return QStringLiteral("五次多项式");
    case VelocityTrajectoryType::Sine:
        return QStringLiteral("正弦往复");
    case VelocityTrajectoryType::LinearChirp:
        return QStringLiteral("扫频正弦");
    }
    return QStringLiteral("未知");
}

int traceFrameAxisIndex(const TraceTelemetryFrame &frame, quint16 axis)
{
    for (int index = 0; index < frame.axisCount; ++index) {
        if (frame.axes[index] == axis) {
            return index;
        }
    }
    return -1;
}
}

ContiWorker::ContiWorker(QObject *parent)
    : QObject(parent)
    , producerTimer_(new QTimer(this))
    , monitorTimer_(new QTimer(this))
    , feedbackTimer_(new QTimer(this))
    , velocityControlTimer_(new QTimer(this))
    , torqueTestTimer_(new QTimer(this))
    , traceDelayCalibrationTimer_(new QTimer(this))
{
    softwareZeroUnit_.fill(0.0, 8);
    softwareZeroValid_.fill(false, 8);
    pendingJogAutoZeroAxes_.clear();
    producerTimer_->setTimerType(Qt::PreciseTimer);
    monitorTimer_->setTimerType(Qt::PreciseTimer);
    monitorTimer_->setInterval(5);
    feedbackTimer_->setTimerType(Qt::PreciseTimer);
    feedbackTimer_->setInterval(10);
    velocityControlTimer_->setTimerType(Qt::PreciseTimer);
    velocityControlTimer_->setInterval(10);
    torqueTestTimer_->setTimerType(Qt::PreciseTimer);
    torqueTestTimer_->setInterval(20);
    traceDelayCalibrationTimer_->setTimerType(Qt::PreciseTimer);
    traceDelayCalibrationTimer_->setInterval(5);

    connect(producerTimer_, &QTimer::timeout, this, &ContiWorker::produceNextPoint);
    connect(monitorTimer_, &QTimer::timeout, this, &ContiWorker::monitorContinuousRun);
    connect(feedbackTimer_, &QTimer::timeout, this, &ContiWorker::refreshFeedback);
    connect(velocityControlTimer_, &QTimer::timeout,
            this, &ContiWorker::runVelocityControlCycle);
    connect(torqueTestTimer_, &QTimer::timeout,
            this, &ContiWorker::runTorqueTestCycle);
    connect(traceDelayCalibrationTimer_, &QTimer::timeout,
            this, &ContiWorker::runTraceDelayCalibrationCycle);
    loadTraceDelayCalibrationResults();
}

void ContiWorker::initializeBoard(const ContiTestConfig &config)
{
    if (!supportedCardUnitDefinition(config.degreesPerCardUnit)) {
        emit logMessage(QStringLiteral("板卡 unit 定义无效：仅支持 1、0.1、0.01 °/unit。"));
        return;
    }
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
    int actualBusCycleUs = 0;
    if (!card_.setBusCycle(config.busCycleUs, error)
        || !card_.readBusCycle(actualBusCycleUs, error)) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        enterError(QStringLiteral("EtherCAT 总线周期配置/读取失败：%1").arg(error));
        return;
    }
    if (actualBusCycleUs != config.busCycleUs) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        enterError(QStringLiteral("EtherCAT 总线周期读回值=%1 us，与设置值=%2 us 不一致；已关闭控制卡。")
                       .arg(actualBusCycleUs)
                       .arg(config.busCycleUs));
        return;
    }

    quint16 detectedSlaveCount = 0;
    if (!card_.readEthercatSlaveCount(detectedSlaveCount, error)) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        actualBusCycleUs_ = 0;
        enterError(QStringLiteral("读取 EtherCAT 在线从站数量失败：%1").arg(error));
        return;
    }
    if (detectedSlaveCount == 0 || detectedSlaveCount > 8) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        actualBusCycleUs_ = 0;
        enterError(QStringLiteral("EtherCAT 在线从站数量为 %1；当前程序仅支持 1 至 8 个单轴伺服从站。")
                       .arg(detectedSlaveCount));
        return;
    }
    detectedAxes_.clear();
    detectedAxes_.reserve(detectedSlaveCount);
    for (quint16 axis = 0; axis < detectedSlaveCount; ++axis) {
        detectedAxes_.push_back(axis);
    }
    if (!detectedAxes_.contains(config.activeAxis)
        || !detectedAxes_.contains(config.holdAxis)) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        actualBusCycleUs_ = 0;
        detectedAxes_.clear();
        enterError(QStringLiteral("当前插补测试轴 %1、%2 超出在线轴范围 0-%3；已关闭控制卡。")
                       .arg(config.activeAxis)
                       .arg(config.holdAxis)
                       .arg(detectedSlaveCount - 1));
        return;
    }
    config_ = config;
    config_.busCycleUs = actualBusCycleUs;
    actualBusCycleUs_ = actualBusCycleUs;
    validateLoadedTraceDelayTiming();
    if (!configureBaseAxes(config_)) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        actualBusCycleUs_ = 0;
        detectedAxes_.clear();
        enterError(QStringLiteral("基础轴配置失败，已关闭控制卡。"));
        return;
    }
    if (!configureFeedbackTrace({config_.activeAxis, config_.holdAxis},
                                config_.degreesPerCardUnit, error)) {
        QString closeError;
        card_.closeBoard(closeError);
        detectedBoardCount_ = 0;
        actualBusCycleUs_ = 0;
        detectedAxes_.clear();
        enterError(QStringLiteral("位置 Trace 配置失败：%1").arg(error));
        return;
    }
    softwareZeroUnit_.fill(0.0, 8);
    softwareZeroValid_.fill(false, 8);
    pendingJogAutoZeroAxes_.clear();
    latestTraceSequence_ = 0;
    latestTraceTimeUs_ = 0;
    trajectoryComparisonActive_ = false;
    trajectoryTraceStartTimeUs_ = 0;
    traceVelocityAnchorValid_ = false;
    traceVelocityAxis_ = config.activeAxis;
    traceVelocityValid_ = false;
    traceCommandVelocityDegreePerSecond_ = 0.0;
    traceActualVelocityDegreePerSecond_ = 0.0;
    vectorSpeedReadFailureLogged_ = false;
    boardInitialized_ = true;
    initializedCardNo_ = config.cardNo;
    stateText_ = QStringLiteral("控制卡已初始化");
    feedbackTimer_->start();
    emit logMessage(QStringLiteral("EtherCAT 总线周期已设置并读回：%1 us；Trace=%2 us（每总线周期采样一次）；轨迹规划=%3 ms（%4 个总线周期）。")
                        .arg(actualBusCycleUs_)
                        .arg(card_.traceSamplePeriodUs())
                        .arg(config_.producerPeriodMs)
                        .arg(config_.producerPeriodMs * 1000 / actualBusCycleUs_));
    emit logMessage(QStringLiteral("控制卡初始化成功：检测到 %1 张卡、%2 个 EtherCAT 伺服轴（轴号 0-%3）；全部在线轴已设置为 %4 pulse/unit（1 unit=%5°）；Trace 仍按 %6 pulse/deg 换算。")
                        .arg(detectedBoardCount_)
                        .arg(detectedAxes_.size())
                        .arg(detectedAxes_.size() - 1)
                        .arg(MotorUnit::pulsesPerCardUnit(config_.degreesPerCardUnit), 0, 'f', 5)
                        .arg(config_.degreesPerCardUnit, 0, 'g', 3)
                        .arg(MotorUnit::kPhysicalPulsesPerDegree, 0, 'f', 3));
    publishStatus();
}

void ContiWorker::closeBoard()
{
    shutdownHardware();
}

void ContiWorker::refreshBusCycle()
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡，再读取 EtherCAT 总线周期。"));
        return;
    }
    int cycleUs = 0;
    QString error;
    if (!card_.readBusCycle(cycleUs, error)) {
        emit logMessage(QStringLiteral("读取 EtherCAT 总线周期失败：%1").arg(error));
        return;
    }
    actualBusCycleUs_ = cycleUs;
    if (cycleUs != config_.busCycleUs) {
        emit logMessage(QStringLiteral("警告：EtherCAT 总线周期读回为 %1 us，当前 Trace 配置基准为 %2 us；请安全关闭并重新初始化后再运行。")
                            .arg(cycleUs)
                            .arg(config_.busCycleUs));
    } else {
        emit logMessage(QStringLiteral("EtherCAT 总线周期读回正常：%1 us；Trace=%2 us；规划周期=%3 ms（%4 个总线周期）。")
                            .arg(cycleUs)
                            .arg(card_.traceSamplePeriodUs())
                            .arg(config_.producerPeriodMs)
                            .arg(config_.producerPeriodMs * 1000 / cycleUs));
    }
    publishStatus();
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
    metadata.degreesPerCardUnit = config_.degreesPerCardUnit;
    metadata.rootDirectory = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("records"));
    metadata.description = QStringLiteral("E5000 Trace：type 3/4 指令与实际速度＋type 5/6 指令与实际位置");
    QString error;
    if (!telemetryRecorder_.start(metadata, error)) {
        emit logMessage(QStringLiteral("开始 Trace 数据记录失败：%1").arg(error));
        publishStatus();
        return;
    }
    manualTelemetryRecording_ = true;
    emit logMessage(QStringLiteral("Trace 数据记录已开始：%1")
                        .arg(telemetryRecorder_.status().outputDirectory));
    publishStatus();
}

void ContiWorker::stopTelemetryRecording()
{
    const TelemetryRecorderStatus before = telemetryRecorder_.status();
    if (!before.recording) {
        manualTelemetryRecording_ = false;
        emit logMessage(QStringLiteral("当前没有正在进行的 Trace 数据记录。"));
        publishStatus();
        return;
    }
    telemetryRecorder_.stop();
    manualTelemetryRecording_ = false;
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
    monitorTimer_->stop();
    feedbackTimer_->stop();
    velocityControlTimer_->stop();
    torqueTestTimer_->stop();
    traceDelayCalibrationTimer_->stop();
    resetRunTimingState();

    if (!boardInitialized_) {
        detectedAxes_.clear();
        manualTelemetryRecording_ = false;
        traceDelayCalibrationActive_ = false;
        traceDelayStatus_.active = false;
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
    manualTelemetryRecording_ = false;
    trajectoryComparisonActive_ = false;
    trajectoryTraceStartTimeUs_ = 0;
    traceDelayCalibrationActive_ = false;
    traceDelayMotionStarted_ = false;
    traceDelayStatus_.active = false;
    traceDelayStatus_.phaseText = QStringLiteral("控制卡已关闭");
    resetTraceDelayHistory();

    QString error;
    if (!card_.closeBoard(error)) {
        stateText_ = QStringLiteral("控制卡关闭失败（已尝试安全停止和失能）");
        emit logMessage(QStringLiteral("错误：%1").arg(error));
        publishStatus();
        return;
    }
    boardInitialized_ = false;
    detectedBoardCount_ = 0;
    actualBusCycleUs_ = 0;
    detectedAxes_.clear();
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
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        emit logMessage(QStringLiteral("运动准备或运行期间不能切换轴使能。"));
        return;
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (config.busCycleUs != actualBusCycleUs_) {
        emit logMessage(QStringLiteral("当前控制卡总线周期为 %1 us；界面已改为 %2 us。请先安全关闭控制卡并重新初始化后再测试。")
                            .arg(actualBusCycleUs_)
                            .arg(config.busCycleUs));
        return;
    }
    if (!sameCardUnitDefinition(config.degreesPerCardUnit, config_.degreesPerCardUnit)) {
        emit logMessage(QStringLiteral("界面板卡 unit 定义已改变；请安全关闭并重新初始化控制卡后再使能。"));
        return;
    }
    if (config.producerPeriodMs <= 0
        || (config.producerPeriodMs * 1000) % actualBusCycleUs_ != 0) {
        emit logMessage(QStringLiteral("轨迹规划周期 %1 ms 不是当前总线周期 %2 us 的整数倍。")
                            .arg(config.producerPeriodMs)
                            .arg(actualBusCycleUs_));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.activeAxis, config.holdAxis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.activeAxis, config.holdAxis},
                                    config_.degreesPerCardUnit, traceError)) {
            enterError(QStringLiteral("位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }

    for (const quint16 axis : {config.activeAxis, config.holdAxis}) {
        QString error;
        QString notice;
        if (!card_.setAxisEquivalent(config.cardNo, axis,
                                     MotorUnit::pulsesPerCardUnit(config_.degreesPerCardUnit), error)) {
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
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        stopTest(false);
    }
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿在未关闭控制卡时切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.activeAxis, config.holdAxis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.activeAxis, config.holdAxis},
                                    config_.degreesPerCardUnit, traceError)) {
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

void ContiWorker::enableAllDetectedAxes()
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        emit logMessage(QStringLiteral("运动准备或运行期间不能执行全局轴使能。"));
        return;
    }
    if (detectedAxes_.isEmpty()) {
        emit logMessage(QStringLiteral("未检测到可使能的 EtherCAT 伺服轴。"));
        return;
    }

    QVector<quint16> newlyEnabledAxes;
    for (const quint16 axis : detectedAxes_) {
        if (enabledAxes_.contains(axis)) {
            continue;
        }
        QString notice;
        QString error;
        if (!card_.enableAxis(axis, notice, error)) {
            for (const quint16 rollbackAxis : newlyEnabledAxes) {
                QString ignored;
                card_.disableAxis(rollbackAxis, ignored);
                enabledAxes_.remove(rollbackAxis);
            }
            stateText_ = QStringLiteral("全局轴使能失败");
            emit logMessage(QStringLiteral("错误：轴 %1 全局使能失败：%2；本次已使能的轴已回滚失能。")
                                .arg(axis)
                                .arg(error));
            publishStatus();
            return;
        }
        if (!notice.isEmpty()) {
            emit logMessage(QStringLiteral("轴 %1：%2").arg(axis).arg(notice));
        }
        enabledAxes_.insert(axis);
        newlyEnabledAxes.push_back(axis);
        emit logMessage(QStringLiteral("全局使能：轴 %1 已使能。").arg(axis));
    }
    stateText_ = QStringLiteral("全部在线轴已使能");
    publishStatus();
}

void ContiWorker::disableAllDetectedAxes()
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (velocityControlActive_) {
        emit logMessage(QStringLiteral("全局失能前正在停止速度闭环。"));
        finishVelocityControl(QStringLiteral("速度闭环已因全局失能而减速停止"), false);
    }
    if (traceDelayCalibrationActive_) {
        emit logMessage(QStringLiteral("全局失能前正在停止 Trace 延迟标定。"));
        finishTraceDelayCalibration(QStringLiteral("Trace 延迟标定已因全局失能而停止"),
                                    true, false);
    }
    if (torqueTestActive_) {
        emit logMessage(QStringLiteral("全局失能前正在停止转矩测试。"));
        finishTorqueTest(QStringLiteral("转矩测试已因全局失能而减速停止"), false);
    }
    if (running_ || preparing_ || pointMoveActive_) {
        emit logMessage(QStringLiteral("全局失能前正在执行安全停止。"));
        safelyStopAllMotionForShutdown();
        trajectoryComparisonActive_ = false;
        speedRatioPending_ = false;
        resetRunTimingState();
    }

    QStringList failures;
    for (const quint16 axis : detectedAxes_) {
        QString error;
        if (!card_.disableAxis(axis, error)) {
            failures << QStringLiteral("轴 %1：%2").arg(axis).arg(error);
            continue;
        }
        enabledAxes_.remove(axis);
        emit logMessage(QStringLiteral("全局失能：轴 %1 已失能。").arg(axis));
    }
    if (failures.isEmpty()) {
        stateText_ = QStringLiteral("全部在线轴已失能");
    } else {
        stateText_ = QStringLiteral("部分在线轴失能失败");
        emit logMessage(QStringLiteral("错误：全局失能未全部成功：%1")
                            .arg(failures.join(QStringLiteral("；"))));
    }
    publishStatus();
}

void ContiWorker::startTest(const ContiTestConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("启动前请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
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
    if (!sameCardUnitDefinition(config.degreesPerCardUnit, config_.degreesPerCardUnit)) {
        emit logMessage(QStringLiteral("界面板卡 unit 定义已改变；请安全关闭并重新初始化控制卡后再运行。"));
        return;
    }
    if (config.busCycleUs != actualBusCycleUs_) {
        emit logMessage(QStringLiteral("当前控制卡总线周期为 %1 us；界面已改为 %2 us。请先安全关闭控制卡并重新初始化后再测试。")
                            .arg(actualBusCycleUs_)
                            .arg(config.busCycleUs));
        return;
    }
    if (config.producerPeriodMs <= 0
        || (config.producerPeriodMs * 1000) % actualBusCycleUs_ != 0) {
        emit logMessage(QStringLiteral("轨迹规划周期 %1 ms 不是当前总线周期 %2 us 的整数倍。")
                            .arg(config.producerPeriodMs)
                            .arg(actualBusCycleUs_));
        return;
    }
    if (config.activeAxis == config.holdAxis) {
        emit logMessage(QStringLiteral("主动轴和保持轴必须不同。"));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.activeAxis, config.holdAxis}) {
        QString traceError;
        if (!configureFeedbackTrace({config.activeAxis, config.holdAxis},
                                    config_.degreesPerCardUnit, traceError)) {
            enterError(QStringLiteral("位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }
    if (config.durationS <= 0.0 || config.producerPeriodMs <= 0
        || config.startupPreloadMs < config.producerPeriodMs
        || config.targetBufferMs < config.startupPreloadMs
        || config.lowBufferMs <= config.criticalBufferMs
        || config.criticalBufferMs < 1 || config.executionDelayMs < config.targetBufferMs
        || config.ratioUpdatePeriodMs < 5 || config.ratioApiMinIntervalMs < config.ratioUpdatePeriodMs
        || config.ratioSafetyApiIntervalMs < config.ratioUpdatePeriodMs
        || config.ratioMin <= 0.0 || config.ratioMin > config.ratioMax || config.ratioMax > 2.0
        || config.ratioDeadband < 0.0 || config.ratioMaxStep <= 0.0) {
        emit logMessage(QStringLiteral("测试参数不合法：请检查预压/缓冲时间及倍率控制参数。"));
        return;
    }
    if (!bothAxesEnabled(config)) {
        emit logMessage(QStringLiteral("请先使能当前选择的两个测试轴。"));
        return;
    }
    telemetryRecorder_.appendEvent(QStringLiteral("conti_test_preparing"));

    double activeStartCardUnit = 0.0;
    double holdStartCardUnit = 0.0;
    QString error;
    if (!card_.readPositionUnit(config.cardNo, config.activeAxis, activeStartCardUnit, error)
        || !card_.readPositionUnit(config.cardNo, config.holdAxis, holdStartCardUnit, error)) {
        enterError(QStringLiteral("读取起始位置失败：%1").arg(error));
        return;
    }
    const double activeStart = MotorUnit::cardUnitsToDegrees(
        activeStartCardUnit, config_.degreesPerCardUnit);
    const double holdStart = MotorUnit::cardUnitsToDegrees(
        holdStartCardUnit, config_.degreesPerCardUnit);

    config_ = config;
    config_.busCycleUs = actualBusCycleUs_;
    config_.traceCycle = 1;
    trajectory_.reset(config_, activeStart, holdStart);
    trajectoryStartPoint_.timeS = 0.0;
    trajectoryStartPoint_.targetUnit = {activeStart, holdStart};
    hostQueue_.clear();
    pushedPointsByMark_.clear();
    lastPushedPlanTimeS_ = 0.0;
    latestGeneratedPlanTimeS_ = 0.0;
    currentPlanTimeS_ = 0.0;
    expectedPlanTimeS_ = 0.0;
    phaseErrorMs_ = 0.0;
    bufferTimeMs_ = 0.0;
    ratioRef_ = 0.0;
    ratioPhase_ = 0.0;
    ratioBuffer_ = 0.0;
    resetRunTimingState();
    const double initialRatio = config.timeSyncEnabled ? config.ratioMin : 1.0;
    ratioCommand_ = initialRatio;
    ratioApplied_ = initialRatio;
    config_.speedRatio = initialRatio;
    lastQueuedTargetDegree_ = {activeStart, holdStart};
    lastQueuedTargetDegreeValid_ = true;
    skippedDuplicatePointCount_ = 0;
    firstEffectivePointTimeS_ = -1.0;
    lastContiDiagnosticMs_ = -1;
    trajectoryComparisonActive_ = false;
    trajectoryTraceStartTimeUs_ = 0;
    traceVelocityAnchorValid_ = false;
    traceVelocityAxis_ = config_.activeAxis;
    traceVelocityValid_ = false;
    traceCommandVelocityDegreePerSecond_ = 0.0;
    traceActualVelocityDegreePerSecond_ = 0.0;
    vectorSpeedReadFailureLogged_ = false;
    speedRatioNotReadyCount_ = 0;
    nextMark_ = 1;
    lastPushedMark_ = 0;
    listOpen_ = false;
    producerFinished_ = false;
    speedRatioPending_ = false;
    preparing_ = true;
    running_ = false;
    stateText_ = config_.preloadAllTrajectoryToCard
        ? QStringLiteral("正在生成全部轨迹并准备一次性预装")
        : QStringLiteral("正在按 %1 ms（%2 个总线周期）实时生成延迟轨迹")
              .arg(config_.producerPeriodMs)
              .arg(config_.producerPeriodMs * 1000 / actualBusCycleUs_);
    producerTimer_->setInterval(config_.producerPeriodMs);
    emit logMessage(QStringLiteral("轨迹已重置：轴 %1 指令起点=%2°，轴 %3 指令起点=%4°；%5。")
                        .arg(config_.activeAxis).arg(activeStart, 0, 'f', 4)
                        .arg(config_.holdAxis).arg(holdStart, 0, 'f', 4)
                        .arg(config_.preloadAllTrajectoryToCard
                             ? QStringLiteral("对照模式：将全部有效段一次性预装入卡侧")
                             : QStringLiteral("等待预压 %1 ms").arg(config_.startupPreloadMs)));
    emit logMessage(config_.trajectoryPointMode == TrajectoryPointMode::UniformDistance
        ? QStringLiteral("轨迹产点方式：等间距直线对照；每段几何长度恒定，仅用于检查小线段前瞻和连接平滑性。")
        : QStringLiteral("轨迹产点方式：五次多项式；点间距按时间规律变化，用于正式时间同步测试。"));
    const double activeEnd = activeStart + config_.activeDeltaUnit;
    const double holdEnd = holdStart + (config_.stage == TestStage::DualAxis ? config_.holdDeltaUnit : 0.0);
    emit logMessage(QStringLiteral("插补单位诊断：1 card unit=%1°，equiv=%2 pulse/unit；物理 1 pulse=%3°；理论起点脉冲=(%4,%5)，终点脉冲=(%6,%7)。")
                        .arg(config_.degreesPerCardUnit, 0, 'g', 3)
                        .arg(MotorUnit::pulsesPerCardUnit(config_.degreesPerCardUnit), 0, 'f', 5)
                        .arg(1.0 / MotorUnit::kPhysicalPulsesPerDegree, 0, 'f', 8)
                        .arg(degreeToEstimatedPulse(activeStart))
                        .arg(degreeToEstimatedPulse(holdStart))
                        .arg(degreeToEstimatedPulse(activeEnd))
                        .arg(degreeToEstimatedPulse(holdEnd)));
    emit logMessage(QStringLiteral("时间同步参数：执行延迟=%1 ms，目标缓冲=%2 ms，低水位=%3 ms，危险水位=%4 ms，倍率范围=[%5,%6]。")
                        .arg(config_.executionDelayMs)
                        .arg(config_.targetBufferMs)
                        .arg(config_.lowBufferMs)
                        .arg(config_.criticalBufferMs)
                        .arg(config_.ratioMin, 0, 'f', 2)
                        .arg(config_.ratioMax, 0, 'f', 2));
    if (!config_.timeSyncEnabled) {
        emit logMessage(QStringLiteral("时间同步倍率控制已关闭：公共倍率按 1.000 固定，运行中不调用动态倍率 API。"));
    }
    if (config_.preloadAllTrajectoryToCard) {
        if (!generateAllTrajectoryPoints()) {
            return;
        }
        if (config_.timeSyncEnabled) {
            emit logMessage(QStringLiteral("一次性预装对照仍启用了时间同步倍率控制；若要仅比较流式调度，请另行关闭该控制。"));
        }
        startAfterPreload();
    } else {
        emit logMessage(QStringLiteral("实时流式模式：规划线程每 %1 ms 产生下一点，先建立 %2 ms 执行延迟；启动后继续向独占硬件线程追加轨迹。")
                            .arg(config_.producerPeriodMs)
                            .arg(config_.executionDelayMs));
        produceNextPoint();
        if ((preparing_ || running_) && !producerFinished_) {
            producerTimer_->start();
        }
    }
    publishStatus();
}

void ContiWorker::stopTest(bool emergency)
{
    if (torqueTestActive_) {
        stopTorqueTest(emergency);
        return;
    }
    if (traceDelayCalibrationActive_) {
        stopTraceDelayCalibration(emergency);
        return;
    }
    if (velocityControlActive_) {
        stopVelocityControl(emergency);
        return;
    }
    telemetryRecorder_.appendEvent(emergency
                                       ? QStringLiteral("motion_immediate_stop")
                                       : QStringLiteral("motion_decelerated_stop"));
    producerTimer_->stop();
    monitorTimer_->stop();

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
    trajectoryComparisonActive_ = false;
    speedRatioPending_ = false;
    hostQueue_.clear();
    lastFeedStatus_ = {};
    resetRunTimingState();
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
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
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
        if (!configureFeedbackTrace({config.axis}, config_.degreesPerCardUnit, traceError)) {
            enterError(QStringLiteral("单轴位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }

    QString error;
    QString notice;
    if (!card_.setAxisEquivalent(config.cardNo, config.axis,
                                 MotorUnit::pulsesPerCardUnit(config_.degreesPerCardUnit), error)) {
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
    if (running_ || preparing_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
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
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        emit logMessage(QStringLiteral("运动准备或运行期间不能修改点动软件零位。"));
        return;
    }
    if (config.cardNo != initializedCardNo_ || !enabledAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("请先使能当前选择的点动测试轴，再设置软件零位。"));
        return;
    }
    if (traceAxes_ != QVector<quint16> {config.axis}) {
        QString error;
        if (!configureFeedbackTrace({config.axis}, config_.degreesPerCardUnit, error)) {
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
    if (running_ || preparing_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
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
        if (!configureFeedbackTrace({config.axis}, config_.degreesPerCardUnit, traceError)) {
            enterError(QStringLiteral("单轴位置 Trace 重配失败：%1").arg(traceError));
            return;
        }
    }
    SingleAxisJogConfig cardConfig = config;
    cardConfig.targetPositionUnit = MotorUnit::degreesToCardUnits(
        config.targetPositionUnit, config_.degreesPerCardUnit);
    cardConfig.maxVelocityUnitPerSecond = MotorUnit::degreesToCardUnits(
        config.maxVelocityUnitPerSecond, config_.degreesPerCardUnit);

    QString error;
    if (!card_.startPointMove(cardConfig, error)) {
        enterError(QStringLiteral("轴 %1 点位运动启动失败：%2").arg(config.axis).arg(error));
        return;
    }
    pointMoveAxis_ = config.axis;
    pointMoveActive_ = true;
    pointMoveDiagnosticPending_ = true;
    pointMoveRequestedTargetUnit_ = config.targetPositionUnit;
    double pointMoveCardTargetRawUnit = 0.0;
    pointMoveCardTargetValid_ = card_.readTargetPositionUnit(
        config.axis, pointMoveCardTargetRawUnit, error);
    if (pointMoveCardTargetValid_) {
        pointMoveCardTargetUnit_ = MotorUnit::cardUnitsToDegrees(
            pointMoveCardTargetRawUnit, config_.degreesPerCardUnit);
    }
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

bool ContiWorker::validateVelocityControlConfig(
    const VelocityControlConfig &config,
    QString &errorMessage) const
{
    const bool finite = std::isfinite(config.relativeDeltaDegree)
        && std::isfinite(config.sineAmplitudeDegree)
        && std::isfinite(config.sineFrequencyHz)
        && std::isfinite(config.chirpAmplitudeDegree)
        && std::isfinite(config.chirpStartFrequencyHz)
        && std::isfinite(config.chirpEndFrequencyHz)
        && std::isfinite(config.durationS)
        && std::isfinite(config.degreesPerCardUnit)
        && std::isfinite(config.kp)
        && std::isfinite(config.ki)
        && std::isfinite(config.kd)
        && std::isfinite(config.integralLimitDegreeSecond)
        && std::isfinite(config.maxPidCorrectionDegreePerSecond)
        && std::isfinite(config.velocityFeedforwardGain)
        && std::isfinite(config.maxVelocityDegreePerSecond)
        && std::isfinite(config.maxAccelerationDegreePerSecond2)
        && std::isfinite(config.onlineChangeTimeS)
        && std::isfinite(config.startVelocityThresholdDegreePerSecond)
        && std::isfinite(config.positionToleranceDegree)
        && std::isfinite(config.speedToleranceDegreePerSecond)
        && std::isfinite(config.maxFollowingErrorDegree);
    const double nyquistFrequencyHz = 500.0 / std::max(1, config.controlPeriodMs);
    const bool trajectoryValid =
        (config.trajectoryType == VelocityTrajectoryType::Quintic)
        || (config.trajectoryType == VelocityTrajectoryType::Sine
            && config.sineAmplitudeDegree > 0.0
            && config.sineFrequencyHz > 0.0
            && config.sineFrequencyHz < nyquistFrequencyHz)
        || (config.trajectoryType == VelocityTrajectoryType::LinearChirp
            && config.chirpAmplitudeDegree > 0.0
            && config.chirpStartFrequencyHz > 0.0
            && config.chirpEndFrequencyHz > 0.0
            && config.chirpStartFrequencyHz < nyquistFrequencyHz
            && config.chirpEndFrequencyHz < nyquistFrequencyHz);
    if (!finite || !trajectoryValid || config.axis >= 8 || config.durationS <= 0.0
        || config.degreesPerCardUnit <= 0.0
        || actualBusCycleUs_ <= 0 || config.controlPeriodMs <= 0
        || config.controlPeriodMs * 1000 < actualBusCycleUs_
        || (config.controlPeriodMs * 1000) % actualBusCycleUs_ != 0
        || config.maxVelocityDegreePerSecond <= 0.0
        || config.maxAccelerationDegreePerSecond2 <= 0.0
        || config.maxPidCorrectionDegreePerSecond < 0.0
        || config.integralLimitDegreeSecond < 0.0
        || config.onlineChangeTimeS < 0.0
        || config.startVelocityThresholdDegreePerSecond <= 0.0
        || config.startVelocityThresholdDegreePerSecond >= config.maxVelocityDegreePerSecond
        || config.positionToleranceDegree <= 0.0
        || config.speedToleranceDegreePerSecond <= 0.0
        || config.stableDwellMs < config.controlPeriodMs
        || config.finishTimeoutMs < config.controlPeriodMs
        || config.maxFollowingErrorDegree <= 0.0
        || config.traceTimeoutMs < config.controlPeriodMs) {
        errorMessage = QStringLiteral(
            "速度闭环参数无效：请检查周期、轨迹参数（频率须低于控制周期奈奎斯特频率）、"
            "PID、限幅、完成和安全参数");
        return false;
    }
    return true;
}

void ContiWorker::startVelocityControl(const VelocityControlConfig &requestedConfig)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        emit logMessage(QStringLiteral("已有运动正在准备或运行，不能启动速度闭环。"));
        return;
    }
    VelocityControlConfig config = requestedConfig;
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    QString error;
    if (!validateVelocityControlConfig(config, error)) {
        emit logMessage(error);
        return;
    }
    if (!enabledAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("请先使能速度闭环测试轴 %1。").arg(config.axis));
        return;
    }
    if (telemetryRecorder_.status().recording) {
        telemetryRecorder_.appendEvent(QStringLiteral("recording_stopped_for_velocity_trace_reconfigure"));
        stopTelemetryRecording();
    }
    if (!card_.setAxisEquivalent(config.cardNo, config.axis,
                                 MotorUnit::pulsesPerCardUnit(config.degreesPerCardUnit), error)) {
        enterError(QStringLiteral("轴 %1 脉冲当量设置失败：%2").arg(config.axis).arg(error));
        return;
    }
    if (!configureFeedbackTrace({config.axis}, config.degreesPerCardUnit, error)) {
        enterError(QStringLiteral("速度闭环 Trace 配置失败：%1").arg(error));
        return;
    }

    velocityConfig_ = config;
    ++velocityRunId_;
    velocityStatus_ = {};
    velocityStatus_.active = true;
    velocityStatus_.runId = velocityRunId_;
    velocityStatus_.axis = config.axis;
    velocityStatus_.stateText = QStringLiteral("等待速度/位置 Trace 首帧");
    velocityControlActive_ = true;
    velocityMotionStarted_ = false;
    velocityReferenceInitialized_ = false;
    velocityLastTraceSequence_ = latestTraceSequence_;
    velocityLastDiagnosticMs_ = -1;
    pendingVelocityPlotSamples_.clear();
    velocityPlotPublishClock_.invalidate();
    velocityPlotTraceStartTimeUs_ = 0;
    velocityPlotTraceStartValid_ = false;
    velocityPlotCommandHistory_.clear();
    velocityPlotLastTraceSequence_ = 0;
    velocityBatchAlignedErrorValid_ = false;
    velocityBatchPeakAlignedErrorDegree_ = 0.0;
    velocityPid_.reset();
    velocityRunClock_.invalidate();
    velocityCycleClock_.invalidate();
    velocityCompletionClock_.invalidate();
    velocityTraceFreshClock_.start();
    velocityAlignedErrorFreshClock_.start();

    TelemetryRunMetadata metadata;
    metadata.cardNo = initializedCardNo_;
    metadata.axes = {config.axis};
    metadata.traceSamplePeriodUs = card_.traceSamplePeriodUs();
    metadata.degreesPerCardUnit = config.degreesPerCardUnit;
    metadata.rootDirectory = QDir(QCoreApplication::applicationDirPath())
                                 .filePath(QStringLiteral("records"));
    metadata.description = QStringLiteral("单轴速度模式位置闭环：%1；"
                                          "Trace type 3/4 速度＋type 5/6 位置")
                               .arg(velocityTrajectoryName(config.trajectoryType));
    velocityAutoRecording_ = telemetryRecorder_.start(metadata, error);
    if (!velocityAutoRecording_) {
        emit logMessage(QStringLiteral("警告：速度闭环自动记录未启动：%1").arg(error));
    }

    feedbackTimer_->stop();
    velocityControlTimer_->setInterval(config.controlPeriodMs);
    velocityControlTimer_->start();
    stateText_ = QStringLiteral("速度闭环等待 Trace");
    QString trajectoryParameters;
    switch (config.trajectoryType) {
    case VelocityTrajectoryType::Quintic:
        trajectoryParameters = QStringLiteral("相对位移=%1°")
                                   .arg(config.relativeDeltaDegree, 0, 'f', 4);
        break;
    case VelocityTrajectoryType::Sine:
        trajectoryParameters = QStringLiteral("振幅=%1°，频率=%2 Hz")
                                   .arg(config.sineAmplitudeDegree, 0, 'f', 4)
                                   .arg(config.sineFrequencyHz, 0, 'f', 4);
        break;
    case VelocityTrajectoryType::LinearChirp:
        trajectoryParameters = QStringLiteral("振幅=%1°，频率=%2→%3 Hz")
                                   .arg(config.chirpAmplitudeDegree, 0, 'f', 4)
                                   .arg(config.chirpStartFrequencyHz, 0, 'f', 4)
                                   .arg(config.chirpEndFrequencyHz, 0, 'f', 4);
        break;
    }
    emit logMessage(QStringLiteral("速度闭环已准备：轴 %1，轨迹=%2（%3），"
                                   "轨迹时间=%4 s，控制周期=%5 ms；"
                                   "脉冲当量=%6 pulse/unit（1 unit=%7°）；"
                                   "Trace=type 3/4/5/6，自动记录=%8。")
                        .arg(config.axis)
                        .arg(velocityTrajectoryName(config.trajectoryType))
                        .arg(trajectoryParameters)
                        .arg(config.durationS, 0, 'f', 3)
                        .arg(config.controlPeriodMs)
                        .arg(MotorUnit::pulsesPerCardUnit(config.degreesPerCardUnit), 0, 'f', 6)
                        .arg(config.degreesPerCardUnit, 0, 'g', 12)
                        .arg(velocityAutoRecording_ ? QStringLiteral("是") : QStringLiteral("否")));
    emit logMessage(QStringLiteral("速度闭环参数：前馈=%1×%2，Kp/Ki/Kd=%3/%4/%5，"
                                   "速度上限=%6°/s，加速度上限=%7°/s²，在线变速时间=%8 ms。")
                        .arg(config.velocityFeedforwardEnabled ? QStringLiteral("启用") : QStringLiteral("关闭"))
                        .arg(config.velocityFeedforwardGain, 0, 'f', 3)
                        .arg(config.kp, 0, 'f', 4)
                        .arg(config.ki, 0, 'f', 4)
                        .arg(config.kd, 0, 'f', 4)
                        .arg(config.maxVelocityDegreePerSecond, 0, 'f', 3)
                        .arg(config.maxAccelerationDegreePerSecond2, 0, 'f', 3)
                        .arg(config.onlineChangeTimeS * 1000.0, 0, 'f', 1));
    emit logMessage(QStringLiteral(
                        "跟随误差保护：使用延迟对齐误差 "
                        "type05(t-τ)-type06(t)，上限=%1°；"
                        "PID与终点判定仍使用轨迹时间误差。")
                        .arg(config.maxFollowingErrorDegree, 0, 'f', 4));
    publishStatus();
}

void ContiWorker::evaluateVelocityReference(double elapsedS,
                                             double &positionDegree,
                                             double &velocityDegreePerSecond) const
{
    const double duration = std::max(1e-9, velocityConfig_.durationS);
    const double time = std::clamp(elapsedS, 0.0, duration);
    if (velocityConfig_.trajectoryType == VelocityTrajectoryType::Quintic) {
        const double tau = time / duration;
        const double ratio = 10.0 * std::pow(tau, 3)
            - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
        const double ratioRate = (30.0 * std::pow(tau, 2)
            - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / duration;
        positionDegree = velocityStartPositionDegree_
            + velocityConfig_.relativeDeltaDegree * ratio;
        velocityDegreePerSecond = velocityConfig_.relativeDeltaDegree * ratioRate;
    } else {
        // 固定使用轨迹时长 10%（且不超过 0.5 s）的五次平滑包络，
        // 使往复轨迹在首尾的位置和速度均与静止状态连续。
        const double fadeDuration = std::min(0.5, duration * 0.1);
        double envelope = 1.0;
        double envelopeRate = 0.0;
        auto smoothStep = [](double u) {
            return 10.0 * std::pow(u, 3)
                - 15.0 * std::pow(u, 4) + 6.0 * std::pow(u, 5);
        };
        auto smoothStepRate = [](double u) {
            return 30.0 * std::pow(u, 2)
                - 60.0 * std::pow(u, 3) + 30.0 * std::pow(u, 4);
        };
        if (time < fadeDuration) {
            const double u = time / fadeDuration;
            envelope = smoothStep(u);
            envelopeRate = smoothStepRate(u) / fadeDuration;
        } else if (time > duration - fadeDuration) {
            const double u = (duration - time) / fadeDuration;
            envelope = smoothStep(u);
            envelopeRate = -smoothStepRate(u) / fadeDuration;
        }

        double amplitude = velocityConfig_.sineAmplitudeDegree;
        double phase = kTwoPi * velocityConfig_.sineFrequencyHz * time;
        double phaseRate = kTwoPi * velocityConfig_.sineFrequencyHz;
        if (velocityConfig_.trajectoryType == VelocityTrajectoryType::LinearChirp) {
            amplitude = velocityConfig_.chirpAmplitudeDegree;
            const double sweepRate =
                (velocityConfig_.chirpEndFrequencyHz
                 - velocityConfig_.chirpStartFrequencyHz) / duration;
            phase = kTwoPi * (velocityConfig_.chirpStartFrequencyHz * time
                             + 0.5 * sweepRate * time * time);
            phaseRate = kTwoPi
                * (velocityConfig_.chirpStartFrequencyHz + sweepRate * time);
        }
        positionDegree = velocityStartPositionDegree_
            + amplitude * envelope * std::sin(phase);
        velocityDegreePerSecond = amplitude
            * (envelopeRate * std::sin(phase)
               + envelope * std::cos(phase) * phaseRate);
    }
    if (elapsedS >= duration) {
        positionDegree = velocityFinalPositionDegree_;
        velocityDegreePerSecond = 0.0;
    }
}

void ContiWorker::runVelocityControlCycle()
{
    if (!velocityControlActive_) {
        return;
    }
    if (!pollTraceFeedback()) {
        finishVelocityControl(QStringLiteral("速度闭环 Trace 读取失败：%1").arg(traceStateText_), true);
        return;
    }
    if (latestTraceSequence_ != velocityLastTraceSequence_) {
        velocityLastTraceSequence_ = latestTraceSequence_;
        velocityTraceFreshClock_.restart();
    } else if (velocityTraceFreshClock_.isValid()
               && velocityTraceFreshClock_.elapsed() > velocityConfig_.traceTimeoutMs) {
        finishVelocityControl(QStringLiteral("速度闭环 Trace 超时：%1 ms 内无新帧")
                                  .arg(velocityConfig_.traceTimeoutMs), true);
        return;
    }
    if (velocityConfig_.axis >= static_cast<quint16>(latestAxisFeedback_.size())) {
        finishVelocityControl(QStringLiteral("速度闭环反馈轴索引越界"), true);
        return;
    }
    const AxisFeedback &feedback = latestAxisFeedback_.at(velocityConfig_.axis);
    if (!feedback.valid || !feedback.traceSampleValid) {
        velocityStatus_.stateText = QStringLiteral("等待有效 Trace 反馈");
        publishStatus();
        return;
    }
    if (feedback.stateMachine != 4U || feedback.axisErrorCode != 0U) {
        finishVelocityControl(QStringLiteral("轴状态异常：状态机=%1，轴错误码=%2")
                                  .arg(feedback.stateMachine)
                                  .arg(feedback.axisErrorCode), true);
        return;
    }
    if (feedback.delayCompensationValid) {
        velocityAlignedErrorFreshClock_.restart();
    } else if (velocityMotionStarted_
               && velocityAlignedErrorFreshClock_.isValid()
               && velocityAlignedErrorFreshClock_.elapsed()
                      > velocityConfig_.traceTimeoutMs) {
        finishVelocityControl(
            QStringLiteral("延迟对齐跟随误差无效超时：%1 ms 内无法取得连续历史帧")
                .arg(velocityConfig_.traceTimeoutMs),
            true);
        return;
    }

    if (!velocityReferenceInitialized_) {
        velocityReferenceInitialized_ = true;
        velocityStartPositionDegree_ = feedback.encoderPositionUnit;
        velocityFinalPositionDegree_ = velocityConfig_.trajectoryType
                == VelocityTrajectoryType::Quintic
            ? velocityStartPositionDegree_ + velocityConfig_.relativeDeltaDegree
            : velocityStartPositionDegree_;
        velocityPid_.reset();
        velocityRunClock_.start();
        velocityCycleClock_.start();
        velocityTraceFreshClock_.restart();
        velocityStatus_.actualPositionDegree = feedback.encoderPositionUnit;
        velocityStatus_.referencePositionDegree = velocityStartPositionDegree_;
        velocityStatus_.delayAlignedFollowingErrorDegree =
            feedback.delayCompensatedFollowingErrorUnit;
        velocityStatus_.delayCompensationMs = feedback.delayCompensationMs;
        velocityStatus_.delayAlignedFollowingErrorValid =
            feedback.delayCompensationValid;
        velocityStatus_.stateText = QStringLiteral("闭环运行中");
        stateText_ = QStringLiteral("速度闭环运行中");
        velocityPlotTraceStartTimeUs_ = latestTraceTimeUs_;
        velocityPlotTraceStartValid_ = true;
        velocityPlotPublishClock_.start();
        VelocityPlotSample initialSample;
        initialSample.runId = velocityRunId_;
        initialSample.traceSequence = latestTraceSequence_;
        initialSample.referencePositionDegree = velocityStartPositionDegree_;
        initialSample.cardCommandPositionDegree = feedback.commandPositionUnit;
        initialSample.actualPositionDegree = feedback.encoderPositionUnit;
        initialSample.positionErrorDegree = velocityStartPositionDegree_ - feedback.encoderPositionUnit;
        initialSample.delayAlignedFollowingErrorDegree =
            feedback.delayCompensatedFollowingErrorUnit;
        initialSample.delayAlignedFollowingErrorValid =
            feedback.delayCompensationValid;
        initialSample.positionToleranceDegree = velocityConfig_.positionToleranceDegree;
        initialSample.cardCommandVelocityDegreePerSecond = feedback.commandVelocityUnitPerSecond;
        initialSample.actualVelocityDegreePerSecond = feedback.actualVelocityUnitPerSecond;
        pendingVelocityPlotSamples_.push_back(initialSample);
        emit logMessage(QStringLiteral("速度闭环取得首帧：轴 %1 起点=%2°，结束参考=%3°。")
                            .arg(velocityConfig_.axis)
                            .arg(velocityStartPositionDegree_, 0, 'f', 6)
                            .arg(velocityFinalPositionDegree_, 0, 'f', 6));
        publishStatus();
        return;
    }

    const double elapsedS = velocityRunClock_.elapsed() / 1000.0;
    const double dtSeconds = velocityCycleClock_.isValid()
        ? std::max(1e-6, velocityCycleClock_.restart() / 1000.0)
        : velocityConfig_.controlPeriodMs / 1000.0;
    double referencePosition = 0.0;
    double referenceVelocity = 0.0;
    evaluateVelocityReference(elapsedS, referencePosition, referenceVelocity);
    const double trajectoryTimeError =
        referencePosition - feedback.encoderPositionUnit;
    const bool followingErrorProtectionValid =
        velocityBatchAlignedErrorValid_ || feedback.delayCompensationValid;
    const double followingErrorForProtection =
        velocityBatchAlignedErrorValid_
        ? velocityBatchPeakAlignedErrorDegree_
        : feedback.delayCompensatedFollowingErrorUnit;
    if (followingErrorProtectionValid
        && std::abs(followingErrorForProtection)
               > velocityConfig_.maxFollowingErrorDegree) {
        finishVelocityControl(
            QStringLiteral(
                "延迟对齐跟随误差超限：当前=%1°，上限=%2°，τ=%3 ms；"
                "同时轨迹时间误差=%4°")
                .arg(followingErrorForProtection, 0, 'f', 6)
                .arg(velocityConfig_.maxFollowingErrorDegree, 0, 'f', 6)
                .arg(feedback.delayCompensationMs, 0, 'f', 4)
                .arg(trajectoryTimeError, 0, 'f', 6),
            true);
        return;
    }

    const PositionVelocityPidOutput output = velocityPid_.update(
        velocityConfig_, trajectoryTimeError, referenceVelocity,
        feedback.actualVelocityUnitPerSecond, dtSeconds);
    QElapsedTimer apiClock;
    apiClock.start();
    short apiResult = 0;
    QString error;
    bool apiOk = true;
    if (!velocityMotionStarted_) {
        if (std::abs(output.commandVelocity)
            >= velocityConfig_.startVelocityThresholdDegreePerSecond) {
            apiOk = card_.startVelocityMove(velocityConfig_, output.commandVelocity, error);
            velocityMotionStarted_ = apiOk;
            if (apiOk) {
                emit logMessage(QStringLiteral("速度闭环：轴 %1 已调用 dmc_vmove，启动方向=%2，初始速度=%3°/s。")
                                    .arg(velocityConfig_.axis)
                                    .arg(output.commandVelocity < 0.0
                                             ? QStringLiteral("负向") : QStringLiteral("正向"))
                                    .arg(output.commandVelocity, 0, 'f', 6));
            }
        }
    } else {
        apiOk = card_.changeVelocity(velocityConfig_, output.commandVelocity,
                                     apiResult, error);
    }
    const qint64 apiDurationUs = apiClock.nsecsElapsed() / 1000;
    if (!apiOk) {
        velocityStatus_.lastApiResult = apiResult;
        velocityStatus_.lastApiDurationUs = apiDurationUs;
        finishVelocityControl(QStringLiteral("速度命令执行失败：%1").arg(error), true);
        return;
    }

    velocityStatus_.active = true;
    velocityStatus_.motionStarted = velocityMotionStarted_;
    velocityStatus_.elapsedS = elapsedS;
    velocityStatus_.controlDtMs = dtSeconds * 1000.0;
    velocityStatus_.maximumJitterMs = std::max(
        velocityStatus_.maximumJitterMs,
        std::abs(velocityStatus_.controlDtMs - velocityConfig_.controlPeriodMs));
    velocityStatus_.referencePositionDegree = referencePosition;
    velocityStatus_.cardCommandPositionDegree = feedback.commandPositionUnit;
    velocityStatus_.actualPositionDegree = feedback.encoderPositionUnit;
    velocityStatus_.positionErrorDegree = trajectoryTimeError;
    velocityStatus_.delayAlignedFollowingErrorDegree =
        feedback.delayCompensatedFollowingErrorUnit;
    velocityStatus_.delayCompensationMs = feedback.delayCompensationMs;
    velocityStatus_.delayAlignedFollowingErrorValid =
        feedback.delayCompensationValid;
    velocityStatus_.referenceVelocityDegreePerSecond = referenceVelocity;
    velocityStatus_.commandVelocityDegreePerSecond = output.commandVelocity;
    velocityStatus_.cardCommandVelocityDegreePerSecond = feedback.commandVelocityUnitPerSecond;
    velocityStatus_.actualVelocityDegreePerSecond = feedback.actualVelocityUnitPerSecond;
    velocityStatus_.feedforwardTermDegreePerSecond = output.feedforward;
    velocityStatus_.pTermDegreePerSecond = output.p;
    velocityStatus_.iTermDegreePerSecond = output.i;
    velocityStatus_.dTermDegreePerSecond = output.d;
    velocityStatus_.velocitySaturated = output.velocitySaturated;
    velocityStatus_.accelerationLimited = output.accelerationLimited;
    velocityStatus_.integralFrozen = output.integralFrozen;
    velocityStatus_.lastApiResult = apiResult;
    velocityStatus_.lastApiDurationUs = apiDurationUs;
    velocityStatus_.stateText = elapsedS < velocityConfig_.durationS
        ? QStringLiteral("闭环运行中") : QStringLiteral("终点稳态确认中");

    const bool terminalStable = elapsedS >= velocityConfig_.durationS
        && std::abs(trajectoryTimeError) <= velocityConfig_.positionToleranceDegree
        && std::abs(feedback.actualVelocityUnitPerSecond)
               <= velocityConfig_.speedToleranceDegreePerSecond;
    if (terminalStable) {
        if (!velocityCompletionClock_.isValid()) {
            velocityCompletionClock_.start();
        }
        if (velocityCompletionClock_.elapsed() >= velocityConfig_.stableDwellMs) {
            finishVelocityControl(QStringLiteral("速度闭环轨迹完成：终点误差=%1°，实际速度=%2°/s。")
                                      .arg(trajectoryTimeError, 0, 'f', 6)
                                      .arg(feedback.actualVelocityUnitPerSecond, 0, 'f', 6));
            return;
        }
    } else {
        velocityCompletionClock_.invalidate();
    }
    if (elapsedS * 1000.0 > velocityConfig_.durationS * 1000.0
        + velocityConfig_.finishTimeoutMs) {
        finishVelocityControl(QStringLiteral("速度闭环终点确认超时：误差=%1°，实际速度=%2°/s。")
                                  .arg(trajectoryTimeError, 0, 'f', 6)
                                  .arg(feedback.actualVelocityUnitPerSecond, 0, 'f', 6), true);
        return;
    }

    const qint64 elapsedMs = velocityRunClock_.elapsed();
    if (velocityLastDiagnosticMs_ < 0 || elapsedMs - velocityLastDiagnosticMs_ >= 250) {
        velocityLastDiagnosticMs_ = elapsedMs;
        const QString alignedErrorText = feedback.delayCompensationValid
            ? QStringLiteral("%1°（τ=%2 ms）")
                  .arg(feedback.delayCompensatedFollowingErrorUnit, 0, 'f', 4)
                  .arg(feedback.delayCompensationMs, 0, 'f', 3)
            : QStringLiteral("无效");
        emit logMessage(QStringLiteral(
                            "速度闭环：t=%1 s，位置 ref/act/轨迹误差=%2/%3/%4°，"
                            "延迟对齐跟随误差=%5；"
                            "速度 ref/cmd/card/act=%6/%7/%8/%9°/s，API=%10 us。")
                            .arg(elapsedS, 0, 'f', 3)
                            .arg(referencePosition, 0, 'f', 4)
                            .arg(feedback.encoderPositionUnit, 0, 'f', 4)
                            .arg(trajectoryTimeError, 0, 'f', 4)
                            .arg(alignedErrorText)
                            .arg(referenceVelocity, 0, 'f', 4)
                            .arg(output.commandVelocity, 0, 'f', 4)
                            .arg(feedback.commandVelocityUnitPerSecond, 0, 'f', 4)
                            .arg(feedback.actualVelocityUnitPerSecond, 0, 'f', 4)
                            .arg(apiDurationUs));
    }
    publishStatus();
}

void ContiWorker::finishVelocityControl(const QString &message, bool emergency)
{
    velocityControlTimer_->stop();
    flushVelocityPlotSamples();
    if (velocityMotionStarted_ && boardInitialized_) {
        QString stopError;
        if (!card_.stopAxis(initializedCardNo_, velocityConfig_.axis, emergency, stopError)) {
            emit logMessage(QStringLiteral("速度闭环停止轴失败：%1").arg(stopError));
        }
    }
    velocityControlActive_ = false;
    velocityMotionStarted_ = false;
    velocityReferenceInitialized_ = false;
    velocityStatus_.active = false;
    velocityStatus_.motionStarted = false;
    velocityAlignedErrorFreshClock_.invalidate();
    velocityPlotCommandHistory_.clear();
    velocityPlotLastTraceSequence_ = 0;
    velocityBatchAlignedErrorValid_ = false;
    velocityBatchPeakAlignedErrorDegree_ = 0.0;
    velocityStatus_.stateText = emergency ? QStringLiteral("异常停止") : QStringLiteral("已完成");
    stateText_ = velocityStatus_.stateText;
    telemetryRecorder_.appendEvent(emergency
        ? QStringLiteral("velocity_control_emergency_stop")
        : QStringLiteral("velocity_control_completed"));
    if (velocityAutoRecording_) {
        telemetryRecorder_.stop();
        velocityAutoRecording_ = false;
    }
    emit logMessage(emergency ? QStringLiteral("错误：%1").arg(message) : message);
    if (boardInitialized_) {
        feedbackTimer_->start();
    }
    publishStatus();
}

void ContiWorker::appendVelocityPlotFrames(const QVector<TraceTelemetryFrame> &frames)
{
    velocityBatchAlignedErrorValid_ = false;
    velocityBatchPeakAlignedErrorDegree_ = 0.0;
    if (!velocityControlActive_ || !velocityReferenceInitialized_
        || !velocityPlotTraceStartValid_ || frames.isEmpty()) {
        return;
    }

    constexpr double kPulseToDegree = 1.0 / MotorUnit::kPhysicalPulsesPerDegree;
    for (const TraceTelemetryFrame &frame : frames) {
        if (frame.traceTimeUs < velocityPlotTraceStartTimeUs_) {
            continue;
        }
        int axisIndex = -1;
        for (int index = 0; index < frame.axisCount; ++index) {
            if (frame.axes[index] == velocityConfig_.axis) {
                axisIndex = index;
                break;
            }
        }
        if (axisIndex < 0
            || (frame.validAxisMask & static_cast<quint8>(1U << axisIndex)) == 0U) {
            continue;
        }
        if (velocityPlotLastTraceSequence_ > 0
            && frame.traceSequence != velocityPlotLastTraceSequence_ + 1) {
            velocityPlotCommandHistory_.clear();
        }
        velocityPlotLastTraceSequence_ = frame.traceSequence;

        const double cardCommandPositionDegree =
            frame.commandPulse[axisIndex] * kPulseToDegree;
        velocityPlotCommandHistory_.enqueue(
            {frame.traceTimeUs, cardCommandPositionDegree});
        const double appliedDelayMs =
            velocityConfig_.axis < traceDelayAxisResults_.size()
            ? traceDelayAxisResults_.at(velocityConfig_.axis).appliedDelayMs
            : kDefaultTracePositionDelayMs;
        const quint64 delayUs = static_cast<quint64>(std::llround(
            std::max(0.0, appliedDelayMs) * 1000.0));
        bool delayAlignedErrorValid = false;
        double delayAlignedCommandPositionDegree = 0.0;
        if (frame.traceTimeUs >= delayUs) {
            const quint64 targetTimeUs = frame.traceTimeUs - delayUs;
            while (velocityPlotCommandHistory_.size() >= 2
                   && velocityPlotCommandHistory_.at(1).traceTimeUs <= targetTimeUs) {
                velocityPlotCommandHistory_.dequeue();
            }
            if (!velocityPlotCommandHistory_.isEmpty()
                && velocityPlotCommandHistory_.first().traceTimeUs == targetTimeUs) {
                delayAlignedCommandPositionDegree =
                    velocityPlotCommandHistory_.first().commandPositionDegree;
                delayAlignedErrorValid = true;
            } else if (velocityPlotCommandHistory_.size() >= 2
                       && velocityPlotCommandHistory_.first().traceTimeUs < targetTimeUs
                       && velocityPlotCommandHistory_.at(1).traceTimeUs > targetTimeUs) {
                const TraceCommandHistorySample &left =
                    velocityPlotCommandHistory_.at(0);
                const TraceCommandHistorySample &right =
                    velocityPlotCommandHistory_.at(1);
                const double ratio =
                    (targetTimeUs - left.traceTimeUs)
                    / static_cast<double>(right.traceTimeUs - left.traceTimeUs);
                delayAlignedCommandPositionDegree =
                    left.commandPositionDegree
                    + ratio * (right.commandPositionDegree
                               - left.commandPositionDegree);
                delayAlignedErrorValid = true;
            }
        }

        VelocityPlotSample sample;
        sample.runId = velocityRunId_;
        sample.traceSequence = frame.traceSequence;
        sample.elapsedS = static_cast<double>(frame.traceTimeUs - velocityPlotTraceStartTimeUs_)
            / 1000000.0;
        evaluateVelocityReference(sample.elapsedS, sample.referencePositionDegree,
                                  sample.referenceVelocityDegreePerSecond);
        sample.cardCommandPositionDegree = cardCommandPositionDegree;
        sample.actualPositionDegree = frame.actualPulse[axisIndex] * kPulseToDegree;
        sample.positionErrorDegree = sample.referencePositionDegree - sample.actualPositionDegree;
        sample.delayAlignedFollowingErrorDegree =
            delayAlignedCommandPositionDegree - sample.actualPositionDegree;
        sample.delayAlignedFollowingErrorValid = delayAlignedErrorValid;
        if (delayAlignedErrorValid
            && (!velocityBatchAlignedErrorValid_
                || std::abs(sample.delayAlignedFollowingErrorDegree)
                       > std::abs(velocityBatchPeakAlignedErrorDegree_))) {
            velocityBatchAlignedErrorValid_ = true;
            velocityBatchPeakAlignedErrorDegree_ =
                sample.delayAlignedFollowingErrorDegree;
        }
        sample.positionToleranceDegree = velocityConfig_.positionToleranceDegree;
        sample.commandVelocityDegreePerSecond = velocityStatus_.commandVelocityDegreePerSecond;
        sample.cardCommandVelocityDegreePerSecond =
            frame.commandVelocityPulsePerSecond[axisIndex] * kPulseToDegree;
        sample.actualVelocityDegreePerSecond =
            frame.actualVelocityPulsePerSecond[axisIndex] * kPulseToDegree;
        pendingVelocityPlotSamples_.push_back(sample);
    }

    if (!pendingVelocityPlotSamples_.isEmpty()
        && velocityPlotPublishClock_.isValid()
        && velocityPlotPublishClock_.elapsed() >= kVelocityPlotPublishIntervalMs) {
        flushVelocityPlotSamples();
    }
}

void ContiWorker::flushVelocityPlotSamples()
{
    if (pendingVelocityPlotSamples_.isEmpty()) {
        return;
    }
    QVector<VelocityPlotSample> batch;
    batch.swap(pendingVelocityPlotSamples_);
    emit velocityPlotSamplesReady(batch);
    velocityPlotPublishClock_.restart();
}

void ContiWorker::stopVelocityControl(bool emergency)
{
    if (!velocityControlActive_) {
        emit logMessage(QStringLiteral("当前没有正在运行的速度闭环。"));
        return;
    }
    finishVelocityControl(emergency ? QStringLiteral("速度闭环已立即停止")
                                    : QStringLiteral("速度闭环已减速停止"), emergency);
}

void ContiWorker::resetVelocityController()
{
    if (velocityControlActive_) {
        emit logMessage(QStringLiteral("速度闭环运行中不能重置控制器，请先停止。"));
        return;
    }
    velocityPid_.reset();
    velocityStatus_ = {};
    velocityStatus_.runId = velocityRunId_;
    velocityStatus_.axis = velocityConfig_.axis;
    emit logMessage(QStringLiteral("速度闭环PID状态已重置。"));
    publishStatus();
}

bool ContiWorker::validateTorqueTestConfig(
    const TorqueTestConfig &config, QString &errorMessage) const
{
    const bool finite = std::isfinite(config.degreesPerCardUnit)
        && std::isfinite(config.ratedTorqueNm)
        && std::isfinite(config.targetTorqueNm)
        && std::isfinite(config.maximumCommandTorqueNm)
        && std::isfinite(config.maximumActualTorqueNm)
        && std::isfinite(config.maximumTravelDegree)
        && std::isfinite(config.maximumSpeedDegreePerSecond)
        && std::isfinite(config.velocityLimitRpm);
    if (!finite || config.axis >= 8U || config.degreesPerCardUnit <= 0.0
        || config.ratedTorqueNm <= 0.0
        || config.maximumCommandTorqueNm <= 0.0
        || config.maximumActualTorqueNm <= 0.0
        || std::abs(config.targetTorqueNm) > config.maximumCommandTorqueNm
        || config.maximumTravelDegree <= 0.0
        || config.maximumSpeedDegreePerSecond <= 0.0
        || config.monitorPeriodMs < 5
        || actualBusCycleUs_ <= 0
        || config.monitorPeriodMs * 1000 < actualBusCycleUs_
        || (config.monitorPeriodMs * 1000) % actualBusCycleUs_ != 0
        || config.traceTimeoutMs < config.monitorPeriodMs
        || config.maximumRunTimeMs < config.monitorPeriodMs) {
        errorMessage = QStringLiteral(
            "转矩测试参数无效：请检查额定/目标/限幅转矩、行程、速度、"
            "监测周期、Trace 超时和最长运行时间。");
        return false;
    }
    return true;
}

int ContiWorker::torqueNmToRaw(double torqueNm, double ratedTorqueNm) const
{
    int raw = static_cast<int>(std::llround(torqueNm * 1000.0 / ratedTorqueNm));
    if (raw == 0 && std::abs(torqueNm) > 0.0) {
        raw = torqueNm > 0.0 ? 1 : -1;
    }
    return raw;
}

void ContiWorker::writeTorqueVelocityLimit(const TorqueTestConfig &config)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        emit logMessage(QStringLiteral("存在运动任务时禁止写入转矩模式速度限制。"));
        return;
    }
    if (!detectedAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("转矩测试轴 %1 不在线。").arg(config.axis));
        return;
    }
    long value = 0;
    QString description;
    if (config.velocityLimitOd == TorqueVelocityLimitOd::Vendor220B) {
        if (!std::isfinite(config.velocityLimitRpm) || config.velocityLimitRpm <= 0.0) {
            emit logMessage(QStringLiteral("220Bh 速度限制必须为正数。"));
            return;
        }
        value = static_cast<long>(std::llround(
            config.velocityLimitRpm / 60.0 * MotorUnit::kPulsesPerRevolution));
        description = QStringLiteral("220Bh=%1 rpm（%2 pulse/s）")
                          .arg(config.velocityLimitRpm, 0, 'f', 3).arg(value);
    } else {
        value = config.cia402VelocityLimitRaw;
        if (value <= 0) {
            emit logMessage(QStringLiteral("6080h 驱动器原生值必须大于 0。"));
            return;
        }
        description = QStringLiteral("6080h=%1（驱动器原生单位）").arg(value);
    }

    quint16 node = 0;
    long readback = 0;
    QString error;
    if (!card_.writeTorqueVelocityLimit(config, value, node, readback, error)) {
        emit logMessage(QStringLiteral("错误：转矩速度限制写入/读回失败：%1").arg(error));
        return;
    }
    torqueStatus_.axis = config.axis;
    torqueStatus_.nodeAddress = node;
    torqueStatus_.velocityLimitReadback = readback;
    emit logMessage(QStringLiteral(
        "转矩速度限制已写入并读回：轴 %1，从站 %2，%3。")
                        .arg(config.axis).arg(node).arg(description));
    publishStatus();
}

void ContiWorker::startTorqueTest(const TorqueTestConfig &requestedConfig)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        emit logMessage(QStringLiteral("已有运动正在准备或运行，不能启动转矩测试。"));
        return;
    }
    TorqueTestConfig config = requestedConfig;
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    QString error;
    if (!validateTorqueTestConfig(config, error)) {
        emit logMessage(error);
        return;
    }
    if (!detectedAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("转矩测试轴 %1 不在线。").arg(config.axis));
        return;
    }
    if (!enabledAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("请先使能转矩测试轴 %1。").arg(config.axis));
        return;
    }
    if (std::abs(config.targetTorqueNm) < 1e-12) {
        emit logMessage(QStringLiteral("目标转矩为 0 N·m，转矩模式不会启动。"));
        return;
    }
    if (telemetryRecorder_.status().recording) {
        telemetryRecorder_.appendEvent(
            QStringLiteral("recording_stopped_for_torque_trace_reconfigure"));
        stopTelemetryRecording();
    }
    if (!card_.setAxisEquivalent(config.cardNo, config.axis,
                                 MotorUnit::pulsesPerCardUnit(
                                     config.degreesPerCardUnit), error)) {
        enterError(QStringLiteral("轴 %1 脉冲当量设置失败：%2")
                       .arg(config.axis).arg(error));
        return;
    }
    if (!configureFeedbackTrace({config.axis}, config.degreesPerCardUnit, error)) {
        enterError(QStringLiteral("转矩测试 Trace 配置失败：%1").arg(error));
        return;
    }

    torqueConfig_ = config;
    ++torqueRunId_;
    torqueStatus_ = {};
    torqueStatus_.active = true;
    torqueStatus_.runId = torqueRunId_;
    torqueStatus_.axis = config.axis;
    torqueStatus_.commandTorqueNm = config.targetTorqueNm;
    torqueStatus_.commandTorqueRaw =
        torqueNmToRaw(config.targetTorqueNm, config.ratedTorqueNm);
    torqueStatus_.stateText = QStringLiteral("等待有效 Trace 后启动");
    torqueTestActive_ = true;
    torqueMotionStarted_ = false;
    torqueLastTraceSequence_ = latestTraceSequence_;
    torqueLastDiagnosticMs_ = -1;
    pendingTorquePlotSamples_.clear();
    torqueRunClock_.invalidate();
    torqueTraceFreshClock_.start();
    torquePlotPublishClock_.start();
    feedbackTimer_->stop();
    torqueTestTimer_->setInterval(config.monitorPeriodMs);
    torqueTestTimer_->start();
    stateText_ = QStringLiteral("转矩测试等待 Trace");
    emit logMessage(QStringLiteral(
        "单轴转矩测试已准备：轴 %1，目标=%2 N·m（raw=%3，额定=%4 N·m），"
        "命令限幅=±%5 N·m，行程/速度限制=%6°/%7°/s，监测周期=%8 ms；"
        "启动不会自动使能轴，也不会自动写入速度限制 OD。")
                        .arg(config.axis)
                        .arg(config.targetTorqueNm, 0, 'f', 4)
                        .arg(torqueStatus_.commandTorqueRaw)
                        .arg(config.ratedTorqueNm, 0, 'f', 3)
                        .arg(config.maximumCommandTorqueNm, 0, 'f', 3)
                        .arg(config.maximumTravelDegree, 0, 'f', 3)
                        .arg(config.maximumSpeedDegreePerSecond, 0, 'f', 3)
                        .arg(config.monitorPeriodMs));
    publishStatus();
}

void ContiWorker::updateTorqueCommand(const TorqueTestConfig &config)
{
    if (!torqueTestActive_ || !torqueMotionStarted_) {
        emit logMessage(QStringLiteral("当前没有正在运行的转矩模式，无法在线更新。"));
        return;
    }
    if (config.axis != torqueConfig_.axis
        || std::abs(config.ratedTorqueNm - torqueConfig_.ratedTorqueNm) > 1e-9) {
        emit logMessage(QStringLiteral("运行中不能切换轴或额定转矩。"));
        return;
    }
    if (!std::isfinite(config.targetTorqueNm)
        || std::abs(config.targetTorqueNm) > torqueConfig_.maximumCommandTorqueNm) {
        emit logMessage(QStringLiteral("在线目标转矩超出命令限幅 ±%1 N·m。")
                            .arg(torqueConfig_.maximumCommandTorqueNm, 0, 'f', 3));
        return;
    }
    if (std::abs(config.targetTorqueNm) < 1e-12) {
        finishTorqueTest(QStringLiteral("在线目标转矩为零，转矩测试已减速停止。"));
        return;
    }
    if ((config.targetTorqueNm > 0.0) != (torqueConfig_.targetTorqueNm > 0.0)) {
        emit logMessage(QStringLiteral(
            "硬件位置限位方向在启动时固定；在线反向转矩被拒绝，请先停止后重新启动。"));
        return;
    }
    const int raw = torqueNmToRaw(config.targetTorqueNm,
                                  torqueConfig_.ratedTorqueNm);
    short apiResult = 0;
    QString error;
    QElapsedTimer apiClock;
    apiClock.start();
    if (!card_.changeTorque(torqueConfig_.axis, raw, apiResult, error)) {
        finishTorqueTest(QStringLiteral("在线调整转矩失败：%1").arg(error), true);
        return;
    }
    torqueConfig_.targetTorqueNm = config.targetTorqueNm;
    torqueStatus_.commandTorqueNm = config.targetTorqueNm;
    torqueStatus_.commandTorqueRaw = raw;
    torqueStatus_.lastApiResult = apiResult;
    torqueStatus_.lastApiDurationUs = apiClock.nsecsElapsed() / 1000;
    emit logMessage(QStringLiteral(
        "轴 %1 在线转矩已更新：%2 N·m（raw=%3），API=%4 us。")
                        .arg(torqueConfig_.axis)
                        .arg(config.targetTorqueNm, 0, 'f', 4)
                        .arg(raw).arg(torqueStatus_.lastApiDurationUs));
    publishStatus();
}

void ContiWorker::runTorqueTestCycle()
{
    if (!torqueTestActive_) {
        return;
    }
    if (!pollTraceFeedback()) {
        finishTorqueTest(QStringLiteral("转矩测试 Trace 读取失败：%1")
                             .arg(traceStateText_), true);
        return;
    }
    if (latestTraceSequence_ != torqueLastTraceSequence_) {
        torqueLastTraceSequence_ = latestTraceSequence_;
        torqueTraceFreshClock_.restart();
    } else if (torqueTraceFreshClock_.isValid()
               && torqueTraceFreshClock_.elapsed() > torqueConfig_.traceTimeoutMs) {
        finishTorqueTest(QStringLiteral("转矩测试 Trace 超时：%1 ms 内无新帧")
                             .arg(torqueConfig_.traceTimeoutMs), true);
        return;
    }
    if (torqueConfig_.axis >= static_cast<quint16>(latestAxisFeedback_.size())) {
        finishTorqueTest(QStringLiteral("转矩测试反馈轴索引越界"), true);
        return;
    }
    const AxisFeedback &feedback = latestAxisFeedback_.at(torqueConfig_.axis);
    if (!feedback.valid || !feedback.traceSampleValid) {
        torqueStatus_.stateText = QStringLiteral("等待有效 Trace 反馈");
        publishStatus();
        return;
    }
    if (feedback.stateMachine != 4U || feedback.axisErrorCode != 0U) {
        finishTorqueTest(QStringLiteral("轴状态异常：状态机=%1，轴错误码=%2")
                             .arg(feedback.stateMachine)
                             .arg(feedback.axisErrorCode), true);
        return;
    }

    if (!torqueMotionStarted_) {
        torqueStatus_.startPositionDegree = feedback.encoderPositionUnit;
        const double direction = torqueConfig_.targetTorqueNm > 0.0 ? 1.0 : -1.0;
        torqueStatus_.positionLimitDegree = torqueStatus_.startPositionDegree
            + direction * torqueConfig_.maximumTravelDegree;
        short apiResult = 0;
        QString error;
        QElapsedTimer apiClock;
        apiClock.start();
        if (!card_.startTorqueMove(torqueConfig_, torqueStatus_.commandTorqueRaw,
                                   torqueStatus_.positionLimitDegree,
                                   apiResult, error)) {
            torqueStatus_.lastApiResult = apiResult;
            torqueStatus_.lastApiDurationUs = apiClock.nsecsElapsed() / 1000;
            finishTorqueTest(QStringLiteral("转矩模式启动失败：%1").arg(error), true);
            return;
        }
        torqueStatus_.lastApiResult = apiResult;
        torqueStatus_.lastApiDurationUs = apiClock.nsecsElapsed() / 1000;
        torqueMotionStarted_ = true;
        torqueRunClock_.start();
        torqueStatus_.stateText = QStringLiteral("转矩模式运行中");
        stateText_ = QStringLiteral("单轴转矩模式运行中");
        emit logMessage(QStringLiteral(
            "轴 %1 已调用 nmc_torque_move：目标=%2 N·m（raw=%3），"
            "起点=%4°，%5位置限位=%6°，API=%7 us。")
                            .arg(torqueConfig_.axis)
                            .arg(torqueConfig_.targetTorqueNm, 0, 'f', 4)
                            .arg(torqueStatus_.commandTorqueRaw)
                            .arg(torqueStatus_.startPositionDegree, 0, 'f', 4)
                            .arg(torqueConfig_.hardwarePositionLimitEnabled
                                     ? QStringLiteral("硬件绝对")
                                     : QStringLiteral("仅软件"))
                            .arg(torqueStatus_.positionLimitDegree, 0, 'f', 4)
                            .arg(torqueStatus_.lastApiDurationUs));
    }

    int actualTorqueRaw = 0;
    short torqueApiResult = 0;
    QString torqueError;
    if (!card_.readTorque(torqueConfig_.axis, actualTorqueRaw,
                          torqueApiResult, torqueError)) {
        finishTorqueTest(QStringLiteral("读取实际转矩失败：%1").arg(torqueError), true);
        return;
    }
    const double elapsedS = torqueRunClock_.isValid()
        ? torqueRunClock_.elapsed() / 1000.0 : 0.0;
    const double relativePosition =
        feedback.encoderPositionUnit - torqueStatus_.startPositionDegree;
    torqueStatus_.active = true;
    torqueStatus_.elapsedS = elapsedS;
    torqueStatus_.actualTorqueRaw = actualTorqueRaw;
    torqueStatus_.actualTorqueNm =
        actualTorqueRaw * torqueConfig_.ratedTorqueNm / 1000.0;
    torqueStatus_.actualPositionDegree = feedback.encoderPositionUnit;
    torqueStatus_.actualVelocityDegreePerSecond =
        feedback.actualVelocityUnitPerSecond;
    torqueStatus_.lastApiResult = torqueApiResult;

    TorquePlotSample sample;
    sample.runId = torqueRunId_;
    sample.elapsedS = elapsedS;
    sample.commandTorqueNm = torqueStatus_.commandTorqueNm;
    sample.actualTorqueNm = torqueStatus_.actualTorqueNm;
    sample.relativePositionDegree = relativePosition;
    sample.actualVelocityDegreePerSecond =
        feedback.actualVelocityUnitPerSecond;
    pendingTorquePlotSamples_.push_back(sample);
    if (torquePlotPublishClock_.elapsed() >= kTorquePlotPublishIntervalMs) {
        flushTorquePlotSamples();
    }

    if (std::abs(relativePosition) >= torqueConfig_.maximumTravelDegree) {
        finishTorqueTest(QStringLiteral("转矩测试达到相对行程限制：%1°")
                             .arg(relativePosition, 0, 'f', 4), true);
        return;
    }
    if (std::abs(torqueStatus_.actualTorqueNm)
        > torqueConfig_.maximumActualTorqueNm) {
        finishTorqueTest(QStringLiteral("转矩测试实际转矩超限：%1 N·m > %2 N·m")
                             .arg(torqueStatus_.actualTorqueNm, 0, 'f', 4)
                             .arg(torqueConfig_.maximumActualTorqueNm,
                                  0, 'f', 4), true);
        return;
    }
    if (std::abs(feedback.actualVelocityUnitPerSecond)
        > torqueConfig_.maximumSpeedDegreePerSecond) {
        finishTorqueTest(QStringLiteral("转矩测试实际速度超限：%1°/s > %2°/s")
                             .arg(feedback.actualVelocityUnitPerSecond, 0, 'f', 4)
                             .arg(torqueConfig_.maximumSpeedDegreePerSecond,
                                  0, 'f', 4), true);
        return;
    }
    if (torqueRunClock_.elapsed() >= torqueConfig_.maximumRunTimeMs) {
        finishTorqueTest(QStringLiteral("转矩测试达到最长运行时间 %1 ms，已减速停止。")
                             .arg(torqueConfig_.maximumRunTimeMs));
        return;
    }
    const qint64 elapsedMs = torqueRunClock_.elapsed();
    if (torqueLastDiagnosticMs_ < 0
        || elapsedMs - torqueLastDiagnosticMs_ >= 250) {
        torqueLastDiagnosticMs_ = elapsedMs;
        emit logMessage(QStringLiteral(
            "转矩测试：t=%1 s，目标/实际=%2/%3 N·m（raw=%4/%5），"
            "相对位置=%6°，实际速度=%7°/s。")
                            .arg(elapsedS, 0, 'f', 3)
                            .arg(torqueStatus_.commandTorqueNm, 0, 'f', 4)
                            .arg(torqueStatus_.actualTorqueNm, 0, 'f', 4)
                            .arg(torqueStatus_.commandTorqueRaw)
                            .arg(torqueStatus_.actualTorqueRaw)
                            .arg(relativePosition, 0, 'f', 4)
                            .arg(torqueStatus_.actualVelocityDegreePerSecond,
                                 0, 'f', 4));
    }
    publishStatus();
}

void ContiWorker::flushTorquePlotSamples()
{
    if (pendingTorquePlotSamples_.isEmpty()) {
        return;
    }
    QVector<TorquePlotSample> batch;
    batch.swap(pendingTorquePlotSamples_);
    emit torquePlotSamplesReady(batch);
    torquePlotPublishClock_.restart();
}

void ContiWorker::finishTorqueTest(const QString &message, bool emergency)
{
    torqueTestTimer_->stop();
    flushTorquePlotSamples();
    if (torqueMotionStarted_ && boardInitialized_) {
        QString error;
        if (!card_.stopAxis(initializedCardNo_, torqueConfig_.axis,
                            emergency, error)) {
            emit logMessage(QStringLiteral("转矩测试停止轴失败：%1").arg(error));
        }
    }
    torqueTestActive_ = false;
    torqueMotionStarted_ = false;
    torqueStatus_.active = false;
    torqueStatus_.stateText =
        emergency ? QStringLiteral("异常停止") : QStringLiteral("已停止");
    stateText_ = torqueStatus_.stateText;
    emit logMessage(emergency ? QStringLiteral("错误：%1").arg(message) : message);
    if (boardInitialized_) {
        feedbackTimer_->start();
    }
    publishStatus();
}

void ContiWorker::stopTorqueTest(bool emergency)
{
    if (!torqueTestActive_) {
        emit logMessage(QStringLiteral("当前没有正在运行的转矩测试。"));
        return;
    }
    finishTorqueTest(emergency ? QStringLiteral("转矩测试已立即停止")
                               : QStringLiteral("转矩测试已减速停止"), emergency);
}

bool ContiWorker::validateTraceDelayCalibrationConfig(
    const TraceDelayCalibrationConfig &config,
    QString &errorMessage) const
{
    if (!std::isfinite(config.degreesPerCardUnit)
        || config.degreesPerCardUnit <= 0.0
        || config.axis >= 8U
        || config.holdMs < 200
        || config.sampleWindowMs < 100
        || config.sampleWindowMs > config.holdMs - 100
        || config.restMs < 100
        || !std::isfinite(config.onlineChangeTimeS)
        || config.onlineChangeTimeS < 0.001
        || !std::isfinite(config.maximumSegmentTravelDegree)
        || config.maximumSegmentTravelDegree <= 0.0) {
        errorMessage = QStringLiteral(
            "Trace 延迟标定参数无效：请检查 unit、保持/采样/静止时间、在线变速时间和单段行程。");
        return false;
    }
    double previousSpeed = 0.0;
    for (const double speed : config.speedDegreePerSecond) {
        if (!std::isfinite(speed) || speed <= previousSpeed) {
            errorMessage = QStringLiteral("三档标定速度必须为严格递增的正数。");
            return false;
        }
        if (speed * config.holdMs / 1000.0 > config.maximumSegmentTravelDegree) {
            errorMessage = QStringLiteral(
                "速度 %1°/s 在保持 %2 ms 时的单段位移约 %3°，超过行程上限 %4°。")
                               .arg(speed, 0, 'f', 3)
                               .arg(config.holdMs)
                               .arg(speed * config.holdMs / 1000.0, 0, 'f', 3)
                               .arg(config.maximumSegmentTravelDegree, 0, 'f', 3);
            return false;
        }
        previousSpeed = speed;
    }
    return true;
}

void ContiWorker::startTraceDelayCalibration(
    const TraceDelayCalibrationConfig &requestedConfig)
{
    if (!boardInitialized_) {
        emit logMessage(QStringLiteral("请先初始化控制卡。"));
        return;
    }
    if (running_ || preparing_ || pointMoveActive_ || velocityControlActive_
        || traceDelayCalibrationActive_ || torqueTestActive_) {
        emit logMessage(QStringLiteral("已有运动正在准备或运行，不能启动 Trace 延迟标定。"));
        return;
    }
    TraceDelayCalibrationConfig config = requestedConfig;
    if (config.cardNo != initializedCardNo_) {
        emit logMessage(QStringLiteral("当前已初始化卡号为 %1，请勿切换卡号。")
                            .arg(initializedCardNo_));
        return;
    }
    QString error;
    if (!validateTraceDelayCalibrationConfig(config, error)) {
        emit logMessage(error);
        return;
    }
    if (!detectedAxes_.contains(config.axis) || !enabledAxes_.contains(config.axis)) {
        emit logMessage(QStringLiteral("请先使能在线标定轴 %1。").arg(config.axis));
        return;
    }
    if (telemetryRecorder_.status().recording) {
        emit logMessage(QStringLiteral("请先停止当前 Trace 数据记录，再启动延迟标定。"));
        return;
    }
    if (!card_.setAxisEquivalent(
            config.cardNo, config.axis,
            MotorUnit::pulsesPerCardUnit(config.degreesPerCardUnit), error)) {
        enterError(QStringLiteral("轴 %1 脉冲当量设置失败：%2")
                       .arg(config.axis).arg(error));
        return;
    }
    if (!configureFeedbackTrace({config.axis}, config.degreesPerCardUnit, error)) {
        enterError(QStringLiteral("Trace 延迟标定重配失败：%1").arg(error));
        return;
    }

    traceDelayConfig_ = config;
    traceDelaySegmentTargets_.clear();
    for (const double speed : config.speedDegreePerSecond) {
        traceDelaySegmentTargets_ << speed << -speed;
    }
    traceDelaySegments_.clear();
    traceDelayCurrentSegmentFrames_.clear();
    pendingTraceDelayPlotSamples_.clear();
    resetTraceDelayHistory();
    ++traceDelayRunId_;
    traceDelayCurrentSegmentIndex_ = 0;
    traceDelayPhase_ = TraceDelayPhase::Resting;
    traceDelayCalibrationActive_ = true;
    traceDelayMotionStarted_ = false;
    traceDelayStatus_ = {};
    traceDelayStatus_.active = true;
    traceDelayStatus_.axis = config.axis;
    traceDelayStatus_.totalSegments = traceDelaySegmentTargets_.size();
    traceDelayStatus_.phaseText = QStringLiteral("启动前静止");
    traceDelayStatus_.axisResults = traceDelayAxisResults_;
    traceDelayPhaseClock_.start();
    traceDelayPlotPublishClock_.start();
    traceDelayPlotStartTimeUs_ = 0;

    TelemetryRunMetadata metadata;
    metadata.cardNo = initializedCardNo_;
    metadata.axes = {config.axis};
    metadata.traceSamplePeriodUs = card_.traceSamplePeriodUs();
    metadata.degreesPerCardUnit = config.degreesPerCardUnit;
    metadata.rootDirectory = QDir(QCoreApplication::applicationDirPath())
                                 .filePath(QStringLiteral("records"));
    metadata.description = QStringLiteral(
        "单轴 Trace 位置延迟标定：type 3/4 速度＋type 5/6 位置原始帧");
    traceDelayAutoRecording_ = telemetryRecorder_.start(metadata, error);
    if (!traceDelayAutoRecording_) {
        emit logMessage(QStringLiteral("警告：标定原始 Trace 自动记录未启动：%1").arg(error));
    } else {
        emit logMessage(QStringLiteral("标定原始 Trace 已自动记录到：%1")
                            .arg(telemetryRecorder_.status().outputDirectory));
    }

    feedbackTimer_->stop();
    traceDelayCalibrationTimer_->start();
    stateText_ = QStringLiteral("Trace 延迟标定中");
    emit logMessage(QStringLiteral(
        "Trace 延迟标定已启动：轴 %1，速度序列=±%2/±%3/±%4°/s，"
        "保持=%5 ms，采样窗=%6 ms，段间静止=%7 ms；标定运动不会自动使能轴。")
                        .arg(config.axis)
                        .arg(config.speedDegreePerSecond[0], 0, 'f', 3)
                        .arg(config.speedDegreePerSecond[1], 0, 'f', 3)
                        .arg(config.speedDegreePerSecond[2], 0, 'f', 3)
                        .arg(config.holdMs)
                        .arg(config.sampleWindowMs)
                        .arg(config.restMs));
    publishStatus();
}

bool ContiWorker::startNextTraceDelaySegment(QString &errorMessage)
{
    if (traceDelayCurrentSegmentIndex_ < 0
        || traceDelayCurrentSegmentIndex_ >= traceDelaySegmentTargets_.size()) {
        errorMessage = QStringLiteral("标定段索引越界");
        return false;
    }
    VelocityControlConfig motionConfig;
    motionConfig.cardNo = traceDelayConfig_.cardNo;
    motionConfig.axis = traceDelayConfig_.axis;
    motionConfig.degreesPerCardUnit = traceDelayConfig_.degreesPerCardUnit;
    motionConfig.onlineChangeTimeS = traceDelayConfig_.onlineChangeTimeS;
    motionConfig.startVelocityThresholdDegreePerSecond = 0.001;
    const double target = traceDelaySegmentTargets_.at(traceDelayCurrentSegmentIndex_);
    if (!card_.startVelocityMove(motionConfig, target, errorMessage)) {
        return false;
    }
    traceDelayMotionStarted_ = true;
    traceDelayCurrentSegmentFrames_.clear();
    traceDelayPhase_ = TraceDelayPhase::Moving;
    traceDelayPhaseClock_.restart();
    traceDelayStatus_.currentSegment = traceDelayCurrentSegmentIndex_ + 1;
    traceDelayStatus_.targetSpeedDegreePerSecond = target;
    traceDelayStatus_.phaseText = QStringLiteral("第 %1/%2 段：%3°/s 保持采样")
                                      .arg(traceDelayStatus_.currentSegment)
                                      .arg(traceDelayStatus_.totalSegments)
                                      .arg(target, 0, 'f', 3);
    emit logMessage(QStringLiteral("Trace 延迟标定：开始第 %1/%2 段，目标速度=%3°/s。")
                        .arg(traceDelayStatus_.currentSegment)
                        .arg(traceDelayStatus_.totalSegments)
                        .arg(target, 0, 'f', 3));
    if (traceDelayAutoRecording_) {
        telemetryRecorder_.appendEvent(
            QStringLiteral("trace_delay_segment_start index=%1 target_deg_s=%2")
                .arg(traceDelayStatus_.currentSegment)
                .arg(target, 0, 'f', 6));
    }
    return true;
}

void ContiWorker::runTraceDelayCalibrationCycle()
{
    if (!traceDelayCalibrationActive_) {
        return;
    }
    if (!pollTraceFeedback()) {
        finishTraceDelayCalibration(
            QStringLiteral("Trace 延迟标定读取失败：%1").arg(traceStateText_), true, true);
        return;
    }
    if (traceDelayConfig_.axis >= static_cast<quint16>(latestAxisFeedback_.size())) {
        finishTraceDelayCalibration(QStringLiteral("Trace 延迟标定反馈轴索引越界"),
                                    true, true);
        return;
    }
    const AxisFeedback &feedback = latestAxisFeedback_.at(traceDelayConfig_.axis);
    if (feedback.valid
        && (feedback.stateMachine != 4U || feedback.axisErrorCode != 0U)) {
        finishTraceDelayCalibration(
            QStringLiteral("标定轴状态异常：状态机=%1，轴错误码=%2")
                .arg(feedback.stateMachine).arg(feedback.axisErrorCode),
            true, true);
        return;
    }

    switch (traceDelayPhase_) {
    case TraceDelayPhase::Resting:
        traceDelayStatus_.progressPercent = static_cast<int>(
            100.0 * traceDelayCurrentSegmentIndex_
            / std::max(1, static_cast<int>(traceDelaySegmentTargets_.size())));
        if (traceDelayPhaseClock_.elapsed() >= traceDelayConfig_.restMs) {
            QString error;
            if (!startNextTraceDelaySegment(error)) {
                finishTraceDelayCalibration(
                    QStringLiteral("Trace 延迟标定启动运动失败：%1").arg(error),
                    true, true);
                return;
            }
        }
        break;
    case TraceDelayPhase::Moving:
        traceDelayStatus_.progressPercent = static_cast<int>(
            100.0 * (traceDelayCurrentSegmentIndex_
                     + std::min(1.0, traceDelayPhaseClock_.elapsed()
                                          / static_cast<double>(traceDelayConfig_.holdMs)))
            / std::max(1, static_cast<int>(traceDelaySegmentTargets_.size())));
        if (traceDelayPhaseClock_.elapsed() >= traceDelayConfig_.holdMs) {
            TraceDelaySegmentCapture capture;
            capture.targetSpeedDegreePerSecond =
                traceDelaySegmentTargets_.at(traceDelayCurrentSegmentIndex_);
            capture.frames.swap(traceDelayCurrentSegmentFrames_);
            const quint64 firstSequence = capture.frames.isEmpty()
                ? 0 : capture.frames.constFirst().traceSequence;
            const quint64 lastSequence = capture.frames.isEmpty()
                ? 0 : capture.frames.constLast().traceSequence;
            emit logMessage(QStringLiteral(
                "Trace 延迟标定：第 %1/%2 段采集结束，捕获=%3帧，序号=%4～%5。")
                                .arg(traceDelayCurrentSegmentIndex_ + 1)
                                .arg(traceDelaySegmentTargets_.size())
                                .arg(capture.frames.size())
                                .arg(firstSequence)
                                .arg(lastSequence));
            if (traceDelayAutoRecording_) {
                telemetryRecorder_.appendEvent(
                    QStringLiteral(
                        "trace_delay_segment_end index=%1 target_deg_s=%2 "
                        "captured_frames=%3 first_sequence=%4 last_sequence=%5")
                        .arg(traceDelayCurrentSegmentIndex_ + 1)
                        .arg(capture.targetSpeedDegreePerSecond, 0, 'f', 6)
                        .arg(capture.frames.size())
                        .arg(firstSequence)
                        .arg(lastSequence));
            }
            traceDelaySegments_.push_back(capture);
            QString error;
            if (!card_.stopAxis(initializedCardNo_, traceDelayConfig_.axis, false, error)) {
                finishTraceDelayCalibration(
                    QStringLiteral("Trace 延迟标定减速停止失败：%1").arg(error),
                    true, true);
                return;
            }
            traceDelayPhase_ = TraceDelayPhase::Stopping;
            traceDelayPhaseClock_.restart();
            traceDelayStatus_.phaseText = QStringLiteral("第 %1/%2 段：等待停止")
                                              .arg(traceDelayCurrentSegmentIndex_ + 1)
                                              .arg(traceDelaySegmentTargets_.size());
        }
        break;
    case TraceDelayPhase::Stopping: {
        bool done = false;
        QString error;
        if (!card_.axisMotionDone(traceDelayConfig_.axis, done, error)) {
            finishTraceDelayCalibration(
                QStringLiteral("Trace 延迟标定读取停止状态失败：%1").arg(error),
                true, true);
            return;
        }
        if (!done && traceDelayPhaseClock_.elapsed() > kTraceDelayStopTimeoutMs) {
            finishTraceDelayCalibration(QStringLiteral("Trace 延迟标定段间停止超时"),
                                        true, true);
            return;
        }
        if (done) {
            traceDelayMotionStarted_ = false;
            ++traceDelayCurrentSegmentIndex_;
            if (traceDelayCurrentSegmentIndex_ >= traceDelaySegmentTargets_.size()) {
                analyzeTraceDelayCalibration();
                return;
            }
            traceDelayPhase_ = TraceDelayPhase::Resting;
            traceDelayPhaseClock_.restart();
            traceDelayStatus_.phaseText = QStringLiteral("段间静止");
        }
        break;
    }
    case TraceDelayPhase::Idle:
        break;
    }
    publishStatus();
}

void ContiWorker::analyzeTraceDelayCalibration()
{
    const TraceDelayFitResult fit = TraceDelayCalibrationAnalyzer::analyze(
        traceDelayConfig_, card_.traceSamplePeriodUs(), traceDelaySegments_);
    TraceDelayAxisResult attempt = fit.axisResult;
    attempt.timestamp = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    traceDelayStatus_.fittedSpeedDegreePerSecond =
        fit.measuredSpeedDegreePerSecond;
    traceDelayStatus_.fittedPositionGapDegree =
        fit.measuredPositionGapDegree;
    traceDelayStatus_.fittedSlopeSecond = fit.axisResult.measuredDelayMs / 1000.0;
    traceDelayStatus_.fittedInterceptDegree = fit.axisResult.staticOffsetDegree;
    for (const TraceDelaySegmentDiagnostic &diagnostic : fit.segmentDiagnostics) {
        emit logMessage(QStringLiteral(
            "Trace 标定段 %1：目标=%2°/s，捕获=%3帧，有效轴帧=%4，"
            "指令稳定=%5帧（最长连续=%6），选用=%7帧，段内丢帧=%8，结果=%9。")
                            .arg(diagnostic.segmentNumber)
                            .arg(diagnostic.targetSpeedDegreePerSecond, 0, 'f', 3)
                            .arg(diagnostic.capturedFrames)
                            .arg(diagnostic.validAxisFrames)
                            .arg(diagnostic.commandStableFrames)
                            .arg(diagnostic.longestCommandStableRun)
                            .arg(diagnostic.selectedFrames)
                            .arg(diagnostic.lostFrames)
                            .arg(diagnostic.detail));
        emit logMessage(QStringLiteral(
            "Trace 标定段 %1筛选：type03速度量化步长=%2°/s，"
            "指令稳定容差=±%3°/s。")
                            .arg(diagnostic.segmentNumber)
                            .arg(diagnostic.commandVelocityQuantumDegreePerSecond,
                                 0, 'f', 5)
                            .arg(diagnostic.commandVelocityToleranceDegreePerSecond,
                                 0, 'f', 5));
        if (diagnostic.selectedFrames > 0) {
            emit logMessage(QStringLiteral(
                "Trace 标定段 %1统计：选用序号=%2～%3，指令速度均值=%4°/s，"
                "实际速度均值=%5°/s，实际速度σ=%6°/s，位置差均值=%7°。")
                                .arg(diagnostic.segmentNumber)
                                .arg(diagnostic.selectedFirstSequence)
                                .arg(diagnostic.selectedLastSequence)
                                .arg(diagnostic.commandVelocityMeanDegreePerSecond,
                                     0, 'f', 5)
                                .arg(diagnostic.actualVelocityMeanDegreePerSecond,
                                     0, 'f', 5)
                                .arg(diagnostic.actualVelocityStdDegreePerSecond,
                                     0, 'f', 5)
                                .arg(diagnostic.positionGapMeanDegree,
                                     0, 'f', 6));
        }
    }
    QStringList fitPoints;
    for (int index = 0;
         index < fit.measuredSpeedDegreePerSecond.size()
         && index < fit.measuredPositionGapDegree.size();
         ++index) {
        fitPoints << QStringLiteral("(%1°/s,%2°)")
                         .arg(fit.measuredSpeedDegreePerSecond.at(index),
                              0, 'f', 4)
                         .arg(fit.measuredPositionGapDegree.at(index),
                              0, 'f', 6);
    }
    if (fitPoints.size() == 6) {
        emit logMessage(QStringLiteral(
            "Trace 延迟标定拟合数据：%1；τ=%2 ms，b=%3°，R²=%4，"
            "RMSE=%5°，正反向最大离散=%6 ms，丢帧=%7。")
                            .arg(fitPoints.join(QStringLiteral("、")))
                            .arg(attempt.measuredDelayMs, 0, 'f', 4)
                            .arg(attempt.staticOffsetDegree, 0, 'f', 6)
                            .arg(attempt.rSquared, 0, 'f', 5)
                            .arg(attempt.rmseDegree, 0, 'f', 6)
                            .arg(attempt.pairSpreadMs, 0, 'f', 4)
                            .arg(attempt.lostFrameCount));
    } else {
        emit logMessage(QStringLiteral(
            "Trace 延迟标定未执行最终拟合：有效段=%1/6；原因=%2。")
                            .arg(fitPoints.size())
                            .arg(attempt.detail));
    }

    if (fit.axisResult.valid) {
        traceDelayAxisResults_[traceDelayConfig_.axis] = attempt;
        saveTraceDelayCalibrationResults();
    } else {
        const TraceDelayAxisResult previous =
            traceDelayAxisResults_.at(traceDelayConfig_.axis);
        attempt.appliedDelayMs = previous.appliedDelayMs;
        attempt.source = previous.calibrated
            ? QStringLiteral("沿用实测") : QStringLiteral("默认");
        attempt.detail += QStringLiteral("；本次未覆盖原结果");
    }
    traceDelayStatus_.axisResults = traceDelayAxisResults_;
    traceDelayStatus_.axisResults[traceDelayConfig_.axis] = attempt;
    if (fit.axisResult.valid) {
        finishTraceDelayCalibration(
            QStringLiteral(
                "Trace 延迟标定通过：轴 %1，延迟=%2 ms，静态偏置=%3°，R²=%4，RMSE=%5°。")
                .arg(traceDelayConfig_.axis)
                .arg(attempt.measuredDelayMs, 0, 'f', 4)
                .arg(attempt.staticOffsetDegree, 0, 'f', 6)
                .arg(attempt.rSquared, 0, 'f', 5)
                .arg(attempt.rmseDegree, 0, 'f', 6));
    } else {
        const QString failureMessage = fitPoints.size() == 6
            ? QStringLiteral(
                "Trace 延迟标定未通过：轴 %1，拟合延迟=%2 ms，R²=%3，"
                "丢帧=%4，正反向离散=%5 ms；原因=%6；"
                "诊断继续使用 %7 ms（%8）。")
                  .arg(traceDelayConfig_.axis)
                  .arg(attempt.measuredDelayMs, 0, 'f', 4)
                  .arg(attempt.rSquared, 0, 'f', 5)
                  .arg(attempt.lostFrameCount)
                  .arg(attempt.pairSpreadMs, 0, 'f', 4)
                  .arg(attempt.detail)
                  .arg(attempt.appliedDelayMs, 0, 'f', 4)
                  .arg(attempt.source)
            : QStringLiteral(
                "Trace 延迟标定未通过：轴 %1；原因=%2；"
                "诊断继续使用 %3 ms（%4）。")
                  .arg(traceDelayConfig_.axis)
                  .arg(attempt.detail)
                  .arg(attempt.appliedDelayMs, 0, 'f', 4)
                  .arg(attempt.source);
        finishTraceDelayCalibration(failureMessage, true, false);
    }
}

void ContiWorker::stopTraceDelayCalibration(bool emergency)
{
    if (!traceDelayCalibrationActive_) {
        emit logMessage(QStringLiteral("当前没有正在运行的 Trace 延迟标定。"));
        return;
    }
    finishTraceDelayCalibration(emergency
                                    ? QStringLiteral("Trace 延迟标定已立即停止")
                                    : QStringLiteral("Trace 延迟标定已减速停止"),
                                true, emergency);
}

void ContiWorker::finishTraceDelayCalibration(const QString &message,
                                               bool failed,
                                               bool emergency)
{
    traceDelayCalibrationTimer_->stop();
    if (traceDelayMotionStarted_ && boardInitialized_) {
        QString error;
        bool stopped = card_.stopAxis(initializedCardNo_, traceDelayConfig_.axis,
                                      emergency, error);
        if (stopped && !emergency) {
            stopped = waitForAxisStop(traceDelayConfig_.axis,
                                      kGracefulStopTimeoutMs);
        }
        if (!stopped && !emergency) {
            emit logMessage(QStringLiteral(
                "Trace 延迟标定减速停止未确认，改为立即停止。"));
            error.clear();
            stopped = card_.stopAxis(initializedCardNo_,
                                     traceDelayConfig_.axis, true, error)
                && waitForAxisStop(traceDelayConfig_.axis,
                                   kEmergencyStopTimeoutMs);
        }
        if (!stopped) {
            emit logMessage(QStringLiteral("停止标定轴失败：%1").arg(error));
        }
    }
    traceDelayMotionStarted_ = false;
    flushTraceDelayPlotSamples();
    if (traceDelayAutoRecording_) {
        telemetryRecorder_.appendEvent(failed
            ? QStringLiteral("trace_delay_calibration_failed_or_stopped")
            : QStringLiteral("trace_delay_calibration_completed"));
        telemetryRecorder_.stop();
        traceDelayAutoRecording_ = false;
    }
    traceDelayCalibrationActive_ = false;
    traceDelayPhase_ = TraceDelayPhase::Idle;
    traceDelayStatus_.active = false;
    traceDelayStatus_.progressPercent = failed
        ? traceDelayStatus_.progressPercent : 100;
    traceDelayStatus_.targetSpeedDegreePerSecond = 0.0;
    traceDelayStatus_.phaseText = failed ? QStringLiteral("未通过/已停止")
                                         : QStringLiteral("标定完成");
    stateText_ = failed ? QStringLiteral("Trace 延迟标定未通过/已停止")
                        : QStringLiteral("Trace 延迟标定完成");
    if (boardInitialized_) {
        feedbackTimer_->start();
    }
    emit logMessage(message);
    publishStatus();
}

void ContiWorker::resetTraceDelayCalibrationAxis(quint16 axis)
{
    if (traceDelayCalibrationActive_) {
        emit logMessage(QStringLiteral("标定运行中不能重置结果。"));
        return;
    }
    if (axis >= 8U) {
        emit logMessage(QStringLiteral("Trace 延迟标定轴号无效：%1").arg(axis));
        return;
    }
    TraceDelayAxisResult result;
    result.axis = axis;
    result.appliedDelayMs = kDefaultTracePositionDelayMs;
    result.source = QStringLiteral("默认");
    result.detail = QStringLiteral("已重置为默认延迟");
    traceDelayAxisResults_[axis] = result;
    traceDelayStatus_.axisResults = traceDelayAxisResults_;
    saveTraceDelayCalibrationResults();
    resetTraceDelayHistory();
    emit logMessage(QStringLiteral("轴 %1 的 Trace 延迟结果已重置为默认 %2 ms。")
                        .arg(axis).arg(kDefaultTracePositionDelayMs, 0, 'f', 1));
    publishStatus();
}

void ContiWorker::appendTraceDelayCalibrationFrames(
    const QVector<TraceTelemetryFrame> &frames)
{
    if (!traceDelayCalibrationActive_ || frames.isEmpty()) {
        return;
    }
    for (const TraceTelemetryFrame &frame : frames) {
        const int axisIndex = traceFrameAxisIndex(frame, traceDelayConfig_.axis);
        if (axisIndex < 0
            || (frame.validAxisMask & static_cast<quint8>(1U << axisIndex)) == 0U) {
            continue;
        }
        if (traceDelayPlotStartTimeUs_ == 0) {
            traceDelayPlotStartTimeUs_ = frame.traceTimeUs;
        }
        TraceDelayPlotSample sample;
        sample.runId = traceDelayRunId_;
        sample.elapsedS =
            (frame.traceTimeUs - traceDelayPlotStartTimeUs_) / 1000000.0;
        sample.commandVelocityDegreePerSecond =
            frame.commandVelocityPulsePerSecond[axisIndex]
            / MotorUnit::kPhysicalPulsesPerDegree;
        sample.actualVelocityDegreePerSecond =
            frame.actualVelocityPulsePerSecond[axisIndex]
            / MotorUnit::kPhysicalPulsesPerDegree;
        sample.rawPositionGapDegree =
            (frame.commandPulse[axisIndex] - frame.actualPulse[axisIndex])
            / MotorUnit::kPhysicalPulsesPerDegree;
        pendingTraceDelayPlotSamples_.push_back(sample);
        if (traceDelayPhase_ == TraceDelayPhase::Moving) {
            traceDelayCurrentSegmentFrames_.push_back(frame);
        }
    }
    if (!pendingTraceDelayPlotSamples_.isEmpty()
        && traceDelayPlotPublishClock_.elapsed()
               >= kTraceDelayPlotPublishIntervalMs) {
        flushTraceDelayPlotSamples();
    }
}

void ContiWorker::flushTraceDelayPlotSamples()
{
    if (pendingTraceDelayPlotSamples_.isEmpty()) {
        return;
    }
    QVector<TraceDelayPlotSample> batch;
    batch.swap(pendingTraceDelayPlotSamples_);
    emit traceDelayPlotSamplesReady(batch);
    traceDelayPlotPublishClock_.restart();
}

void ContiWorker::resetTraceDelayHistory()
{
    for (QQueue<TraceCommandHistorySample> &history : traceCommandHistory_) {
        history.clear();
    }
    lastTraceDelaySequence_ = 0;
    for (AxisFeedback &feedback : latestAxisFeedback_) {
        feedback.delayCompensationValid = false;
    }
}

void ContiWorker::applyTraceDelayCompensation(
    const QVector<TraceTelemetryFrame> &frames)
{
    if (frames.isEmpty()) {
        return;
    }
    for (const TraceTelemetryFrame &frame : frames) {
        if (lastTraceDelaySequence_ > 0
            && frame.traceSequence != lastTraceDelaySequence_ + 1) {
            resetTraceDelayHistory();
        }
        lastTraceDelaySequence_ = frame.traceSequence;
        for (int axisIndex = 0; axisIndex < frame.axisCount; ++axisIndex) {
            if ((frame.validAxisMask & static_cast<quint8>(1U << axisIndex)) == 0U) {
                continue;
            }
            const quint16 axis = frame.axes[axisIndex];
            if (axis >= 8U || axis >= static_cast<quint16>(latestAxisFeedback_.size())) {
                continue;
            }
            const double commandDegree =
                frame.commandPulse[axisIndex] / MotorUnit::kPhysicalPulsesPerDegree;
            const double actualDegree =
                frame.actualPulse[axisIndex] / MotorUnit::kPhysicalPulsesPerDegree;
            QQueue<TraceCommandHistorySample> &history = traceCommandHistory_[axis];
            history.enqueue({frame.traceTimeUs, commandDegree});

            const TraceDelayAxisResult &calibration = traceDelayAxisResults_.at(axis);
            const quint64 delayUs = static_cast<quint64>(std::llround(
                std::max(0.0, calibration.appliedDelayMs) * 1000.0));
            AxisFeedback &feedback = latestAxisFeedback_[axis];
            feedback.rawFollowingErrorUnit = commandDegree - actualDegree;
            feedback.delayCompensationMs = calibration.appliedDelayMs;
            feedback.delayCompensationSource = calibration.source;
            feedback.delayCompensationValid = false;

            if (frame.traceTimeUs >= delayUs) {
                const quint64 targetTimeUs = frame.traceTimeUs - delayUs;
                while (history.size() >= 2
                       && history.at(1).traceTimeUs <= targetTimeUs) {
                    history.dequeue();
                }
                if (!history.isEmpty()
                    && history.first().traceTimeUs == targetTimeUs) {
                    feedback.delayAlignedCommandPositionUnit =
                        history.first().commandPositionDegree;
                    feedback.delayCompensationValid = true;
                } else if (history.size() >= 2
                           && history.first().traceTimeUs < targetTimeUs
                           && history.at(1).traceTimeUs > targetTimeUs) {
                    const TraceCommandHistorySample &left = history.at(0);
                    const TraceCommandHistorySample &right = history.at(1);
                    const double ratio =
                        (targetTimeUs - left.traceTimeUs)
                        / static_cast<double>(right.traceTimeUs - left.traceTimeUs);
                    feedback.delayAlignedCommandPositionUnit =
                        left.commandPositionDegree
                        + ratio * (right.commandPositionDegree
                                   - left.commandPositionDegree);
                    feedback.delayCompensationValid = true;
                }
            }
            if (feedback.delayCompensationValid) {
                feedback.delayCompensatedFollowingErrorUnit =
                    feedback.delayAlignedCommandPositionUnit - actualDegree;
            }
        }
    }
}

QString ContiWorker::traceDelayCalibrationFilePath() const
{
    const QString directory =
        QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    return QDir(directory).filePath(QStringLiteral("trace_delay_calibration.json"));
}

void ContiWorker::loadTraceDelayCalibrationResults()
{
    traceDelayAxisResults_.clear();
    traceDelayAxisResults_.reserve(8);
    for (quint16 axis = 0; axis < 8; ++axis) {
        TraceDelayAxisResult result;
        result.axis = axis;
        result.appliedDelayMs = kDefaultTracePositionDelayMs;
        result.source = QStringLiteral("默认");
        result.detail = QStringLiteral("尚未标定");
        traceDelayAxisResults_.push_back(result);
    }
    QFile file(traceDelayCalibrationFilePath());
    if (!file.open(QIODevice::ReadOnly)) {
        traceDelayStatus_.axisResults = traceDelayAxisResults_;
        return;
    }
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject()) {
        traceDelayStatus_.axisResults = traceDelayAxisResults_;
        return;
    }
    const QJsonObject root = document.object();
    savedCalibrationBusCycleUs_ = root.value(QStringLiteral("busCycleUs")).toInt();
    savedCalibrationTracePeriodUs_ =
        root.value(QStringLiteral("tracePeriodUs")).toInt();
    const QJsonArray axes = root.value(QStringLiteral("axes")).toArray();
    for (const QJsonValue &value : axes) {
        const QJsonObject object = value.toObject();
        const int axis = object.value(QStringLiteral("axis")).toInt(-1);
        if (axis < 0 || axis >= traceDelayAxisResults_.size()) {
            continue;
        }
        TraceDelayAxisResult result;
        result.axis = static_cast<quint16>(axis);
        result.calibrated = object.value(QStringLiteral("calibrated")).toBool();
        result.valid = object.value(QStringLiteral("valid")).toBool();
        result.appliedDelayMs =
            object.value(QStringLiteral("appliedDelayMs"))
                .toDouble(kDefaultTracePositionDelayMs);
        result.measuredDelayMs =
            object.value(QStringLiteral("measuredDelayMs")).toDouble();
        result.staticOffsetDegree =
            object.value(QStringLiteral("staticOffsetDegree")).toDouble();
        result.rSquared = object.value(QStringLiteral("rSquared")).toDouble();
        result.rmseDegree = object.value(QStringLiteral("rmseDegree")).toDouble();
        result.pairSpreadMs =
            object.value(QStringLiteral("pairSpreadMs")).toDouble();
        result.lostFrameCount =
            object.value(QStringLiteral("lostFrameCount")).toInt();
        result.source = object.value(QStringLiteral("source"))
                            .toString(QStringLiteral("默认"));
        result.timestamp = object.value(QStringLiteral("timestamp")).toString();
        result.detail = object.value(QStringLiteral("detail")).toString();
        traceDelayAxisResults_[axis] = result;
    }
    traceDelayStatus_.axisResults = traceDelayAxisResults_;
}

void ContiWorker::saveTraceDelayCalibrationResults()
{
    const QString path = traceDelayCalibrationFilePath();
    QDir().mkpath(QFileInfo(path).absolutePath());
    QJsonObject root;
    root.insert(QStringLiteral("version"), 1);
    root.insert(QStringLiteral("busCycleUs"), actualBusCycleUs_);
    root.insert(QStringLiteral("tracePeriodUs"), card_.traceSamplePeriodUs());
    QJsonArray axes;
    for (const TraceDelayAxisResult &result : traceDelayAxisResults_) {
        QJsonObject object;
        object.insert(QStringLiteral("axis"), result.axis);
        object.insert(QStringLiteral("calibrated"), result.calibrated);
        object.insert(QStringLiteral("valid"), result.valid);
        object.insert(QStringLiteral("appliedDelayMs"), result.appliedDelayMs);
        object.insert(QStringLiteral("measuredDelayMs"), result.measuredDelayMs);
        object.insert(QStringLiteral("staticOffsetDegree"),
                      result.staticOffsetDegree);
        object.insert(QStringLiteral("rSquared"), result.rSquared);
        object.insert(QStringLiteral("rmseDegree"), result.rmseDegree);
        object.insert(QStringLiteral("pairSpreadMs"), result.pairSpreadMs);
        object.insert(QStringLiteral("lostFrameCount"), result.lostFrameCount);
        object.insert(QStringLiteral("source"), result.source);
        object.insert(QStringLiteral("timestamp"), result.timestamp);
        object.insert(QStringLiteral("detail"), result.detail);
        axes.append(object);
    }
    root.insert(QStringLiteral("axes"), axes);
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        emit logMessage(QStringLiteral("警告：无法保存 Trace 延迟标定结果：%1")
                            .arg(file.errorString()));
        return;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    savedCalibrationBusCycleUs_ = actualBusCycleUs_;
    savedCalibrationTracePeriodUs_ = card_.traceSamplePeriodUs();
}

void ContiWorker::validateLoadedTraceDelayTiming()
{
    if (savedCalibrationBusCycleUs_ <= 0
        || savedCalibrationTracePeriodUs_ <= 0) {
        return;
    }
    const int currentTracePeriodUs = config_.busCycleUs * config_.traceCycle;
    if (savedCalibrationBusCycleUs_ == actualBusCycleUs_
        && savedCalibrationTracePeriodUs_ == currentTracePeriodUs) {
        return;
    }
    for (TraceDelayAxisResult &result : traceDelayAxisResults_) {
        if (!result.calibrated) {
            continue;
        }
        result.calibrated = false;
        result.valid = false;
        result.appliedDelayMs = kDefaultTracePositionDelayMs;
        result.source = QStringLiteral("默认");
        result.detail = QStringLiteral("总线或 Trace 周期已改变，原标定失效");
    }
    traceDelayStatus_.axisResults = traceDelayAxisResults_;
    emit logMessage(QStringLiteral(
        "提示：已保存的 Trace 延迟标定基于总线/Trace=%1/%2 us，"
        "当前为 %3/%4 us；原结果已失效并回退为默认 %5 ms。")
                        .arg(savedCalibrationBusCycleUs_)
                        .arg(savedCalibrationTracePeriodUs_)
                        .arg(actualBusCycleUs_)
                        .arg(currentTracePeriodUs)
                        .arg(kDefaultTracePositionDelayMs, 0, 'f', 1));
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
    monitorTimer_->stop();
    velocityControlTimer_->stop();
    torqueTestTimer_->stop();
    traceDelayCalibrationTimer_->stop();

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

    if (velocityControlActive_) {
        QString error;
        if (!card_.stopAxis(initializedCardNo_, velocityConfig_.axis, false, error)
            || !waitForAxisStop(velocityConfig_.axis, kGracefulStopTimeoutMs)) {
            emit logMessage(QStringLiteral("速度闭环安全停机：减速停止未确认，改为立即停止。"));
            error.clear();
            card_.stopAxis(initializedCardNo_, velocityConfig_.axis, true, error);
        }
    }
    if (torqueTestActive_) {
        QString error;
        if (!card_.stopAxis(initializedCardNo_, torqueConfig_.axis, false, error)
            || !waitForAxisStop(torqueConfig_.axis, kGracefulStopTimeoutMs)) {
            emit logMessage(QStringLiteral(
                "转矩测试安全停机：减速停止未确认，改为立即停止。"));
            error.clear();
            card_.stopAxis(initializedCardNo_, torqueConfig_.axis, true, error);
        }
    }
    if (traceDelayCalibrationActive_) {
        QString error;
        if (!card_.stopAxis(initializedCardNo_, traceDelayConfig_.axis, false, error)
            || !waitForAxisStop(traceDelayConfig_.axis, kGracefulStopTimeoutMs)) {
            emit logMessage(QStringLiteral(
                "Trace 延迟标定安全停机：减速停止未确认，改为立即停止。"));
            error.clear();
            card_.stopAxis(initializedCardNo_, traceDelayConfig_.axis, true, error);
        }
    }

    listOpen_ = false;
    preparing_ = false;
    running_ = false;
    pointMoveActive_ = false;
    velocityControlActive_ = false;
    velocityMotionStarted_ = false;
    velocityReferenceInitialized_ = false;
    velocityStatus_.active = false;
    velocityStatus_.motionStarted = false;
    torqueTestActive_ = false;
    torqueMotionStarted_ = false;
    torqueStatus_.active = false;
    torqueStatus_.stateText = QStringLiteral("已安全停止");
    traceDelayCalibrationActive_ = false;
    traceDelayMotionStarted_ = false;
    traceDelayStatus_.active = false;
    traceDelayStatus_.phaseText = QStringLiteral("已安全停止");
    hostQueue_.clear();
    lastFeedStatus_ = {};
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
    if ((!preparing_ && !running_) || config_.preloadAllTrajectoryToCard) {
        return;
    }

    if (trajectory_.hasNext()) {
        const ContiPoint point = trajectory_.nextPoint();
        latestGeneratedPlanTimeS_ = point.timeS;
        enqueueTrajectoryPoint(point);
        if (!trajectory_.hasNext()) {
            producerFinished_ = true;
            producerTimer_->stop();
        }
    } else {
        producerFinished_ = true;
        producerTimer_->stop();
    }

    if (preparing_) {
        if (hasExecutionDelayReady()) {
            startAfterPreload();
            return;
        }
        if (producerFinished_) {
            const double availableDelayMs = firstEffectivePointTimeS_ < 0.0
                ? 0.0 : (latestGeneratedPlanTimeS_ - firstEffectivePointTimeS_) * 1000.0;
            enterError(QStringLiteral("实时轨迹已经生成结束，但只能建立 %1 ms 延迟，低于设置的 %2 ms；请减小执行延迟或延长轨迹。")
                           .arg(availableDelayMs, 0, 'f', 1)
                           .arg(config_.executionDelayMs));
        }
        return;
    }

    if (running_) {
        submitGeneratedPoints();
    }
}

void ContiWorker::enqueueTrajectoryPoint(const ContiPoint &point)
{
    // 上位机不再替板卡按脉冲量化过滤。仅丢弃数值上真正重复的坐标，
    // 极小但非零的角度增量仍换算为 card unit 并原样交给 dmc_conti_line_unit。
    const bool duplicate = lastQueuedTargetDegreeValid_
        && std::abs(point.targetUnit[0] - lastQueuedTargetDegree_[0])
               <= kDuplicatePositionToleranceDeg
        && std::abs(point.targetUnit[1] - lastQueuedTargetDegree_[1])
               <= kDuplicatePositionToleranceDeg;
    if (duplicate) {
        ++skippedDuplicatePointCount_;
        return;
    }
    if (firstEffectivePointTimeS_ < 0.0) {
        const std::array<qint64, 2> targetPulse = {
            degreeToEstimatedPulse(point.targetUnit[0]),
            degreeToEstimatedPulse(point.targetUnit[1])
        };
        const std::array<qint64, 2> previousPulse = {
            degreeToEstimatedPulse(lastQueuedTargetDegree_[0]),
            degreeToEstimatedPulse(lastQueuedTargetDegree_[1])
        };
        firstEffectivePointTimeS_ = point.timeS;
        emit logMessage(QStringLiteral("首个有效插补段：规划 t=%1 s，目标=(%2°,%3°)，下发=(%4,%5) card unit；估算物理脉冲=(%6,%7)，相对起点=(%8,%9) pulse（仅诊断，不参与过滤）。")
                            .arg(point.timeS, 0, 'f', 3)
                            .arg(point.targetUnit[0], 0, 'f', 6)
                            .arg(point.targetUnit[1], 0, 'f', 6)
                            .arg(MotorUnit::degreesToCardUnits(
                                     point.targetUnit[0], config_.degreesPerCardUnit), 0, 'f', 8)
                            .arg(MotorUnit::degreesToCardUnits(
                                     point.targetUnit[1], config_.degreesPerCardUnit), 0, 'f', 8)
                            .arg(targetPulse[0])
                            .arg(targetPulse[1])
                            .arg(targetPulse[0] - previousPulse[0])
                            .arg(targetPulse[1] - previousPulse[1]));
    }
    hostQueue_.enqueue(point);
    lastQueuedTargetDegree_ = point.targetUnit;
    lastQueuedTargetDegreeValid_ = true;
}

void ContiWorker::submitGeneratedPoints()
{
    if (!running_ || hostQueue_.isEmpty()) {
        return;
    }
    QVector<ContiFeedItem> feedItems;
    feedItems.reserve(hostQueue_.size());
    while (!hostQueue_.isEmpty()) {
        ContiFeedItem item;
        item.point = hostQueue_.dequeue();
        item.mark = nextMark_++;
        pushedPointsByMark_.insert(item.mark, item.point);
        feedItems.push_back(item);
    }
    card_.appendStreamingPoints(feedItems);
}

bool ContiWorker::hasExecutionDelayReady() const
{
    if (hostQueue_.isEmpty() || firstEffectivePointTimeS_ < 0.0) {
        return false;
    }
    return (latestGeneratedPlanTimeS_ - firstEffectivePointTimeS_) * 1000.0
        >= config_.executionDelayMs;
}

bool ContiWorker::generateAllTrajectoryPoints()
{
    while (trajectory_.hasNext()) {
        const ContiPoint point = trajectory_.nextPoint();
        latestGeneratedPlanTimeS_ = point.timeS;
        enqueueTrajectoryPoint(point);
    }
    producerFinished_ = true;
    if (hostQueue_.isEmpty()) {
        enterError(QStringLiteral("轨迹生成失败：整条轨迹没有非重复插补点；请检查位移与轨迹参数。"));
        return false;
    }
    emit logMessage(QStringLiteral("一次性预装对照：已提前生成完整轨迹，有效段=%1，跳过数值重复点=%2，覆盖至 t=%3 s。")
                        .arg(hostQueue_.size())
                        .arg(skippedDuplicatePointCount_)
                        .arg(latestGeneratedPlanTimeS_, 0, 'f', 3));
    return true;
}

bool ContiWorker::startAfterPreload()
{
    QString error;
    const int queuedPointCount = hostQueue_.size();
    QVector<ContiFeedItem> feedItems;
    feedItems.reserve(queuedPointCount);
    while (!hostQueue_.isEmpty()) {
        ContiFeedItem item;
        item.point = hostQueue_.dequeue();
        item.mark = nextMark_++;
        pushedPointsByMark_.insert(item.mark, item.point);
        feedItems.push_back(item);
    }
    if (!card_.startStreaming(config_, feedItems,
                              config_.preloadAllTrajectoryToCard, error)) {
        enterError(error);
        return false;
    }
    listOpen_ = true;
    lastFeedStatus_ = card_.streamingStatus();
    lastPushedMark_ = lastFeedStatus_.lastPushedMark;
    lastPushedPlanTimeS_ = lastFeedStatus_.lastPushedPlanTimeS;

    ContiCardReadback readback;
    QString readbackError;
    if (card_.readContiCardReadback(config_, readback, readbackError)) {
        emit logMessage(QStringLiteral("板卡参数回读：前瞻=%1，前瞻段数=%2，轨迹误差=%3°，前瞻加速度=%4°/s²；矢量速度(min/max/stop)=%5/%6/%7°/s，加减速=%8/%9 s，S曲线=%10 s。")
                            .arg(readback.lookaheadEnabled ? QStringLiteral("启用")
                                                           : QStringLiteral("禁用"))
                            .arg(readback.lookaheadSegments)
                            .arg(readback.pathErrorDegree, 0, 'f', 6)
                            .arg(readback.lookaheadAccelerationDegreePerSecond2, 0, 'f', 3)
                            .arg(readback.minVectorVelocityDegreePerSecond, 0, 'f', 3)
                            .arg(readback.maxVectorVelocityDegreePerSecond, 0, 'f', 3)
                            .arg(readback.stopVectorVelocityDegreePerSecond, 0, 'f', 3)
                            .arg(readback.accelerationTimeS, 0, 'f', 3)
                            .arg(readback.decelerationTimeS, 0, 'f', 3)
                            .arg(readback.sCurveTimeS, 0, 'f', 3));
    } else {
        emit logMessage(QStringLiteral("警告：连续插补参数只读回读失败：%1；不影响本次运动，但无法确认前瞻和速度参数是否实际生效。")
                            .arg(readbackError));
    }

    preparing_ = false;
    running_ = true;
    trajectoryComparisonActive_ = true;
    trajectoryTraceStartTimeUs_ = 0;
    // 仅在时间同步启用时等待首段规划并下发在线倍率；固定倍率模式不调用该 API。
    speedRatioPending_ = config_.timeSyncEnabled;
    speedRatioNotReadyCount_ = 0;
    contiRunElapsed_.start();
    phaseClockStarted_ = false;
    lastContiDiagnosticMs_ = -1;
    stateText_ = QStringLiteral("连续插补运行中");
    monitorTimer_->start();
    if (config_.preloadAllTrajectoryToCard) {
        emit logMessage(QStringLiteral("一次性预装完成：已写入 %1/%2 个有效段；运行中不再产点或补段；首个有效点 t=%3 s。")
                            .arg(lastPushedMark_)
                            .arg(queuedPointCount)
                            .arg(firstEffectivePointTimeS_, 0, 'f', 3));
    } else {
        emit logMessage(QStringLiteral("独占硬件线程已启动自动补段：初始压入 %1/%2 个有效段（%3 ms），硬件待喂队列=%4；已跳过 %5 个数值重复点，首个有效点 t=%6 s。")
                            .arg(lastPushedMark_)
                            .arg(queuedPointCount)
                            .arg((lastPushedPlanTimeS_ - firstEffectivePointTimeS_) * 1000.0, 0, 'f', 1)
                            .arg(lastFeedStatus_.queuedPointCount)
                            .arg(skippedDuplicatePointCount_)
                            .arg(firstEffectivePointTimeS_, 0, 'f', 3));
    }
    return true;
}

double ContiWorker::planTimeForMark(long currentMark) const
{
    if (pushedPointsByMark_.isEmpty()) {
        return 0.0;
    }

    const long lookupMark = std::max(0L, currentMark + static_cast<long>(config_.markOffset));
    auto iterator = pushedPointsByMark_.upperBound(lookupMark);
    if (iterator == pushedPointsByMark_.begin()) {
        return iterator.value().timeS;
    }
    --iterator;
    return iterator.value().timeS;
}

double ContiWorker::referenceVectorSpeed(double planTimeS) const
{
    if (pushedPointsByMark_.isEmpty()) {
        return 0.0;
    }

    constexpr double kReferenceWindowS = 0.10;
    const double windowBegin = std::max(0.0, planTimeS - kReferenceWindowS / 2.0);
    const double windowEnd = planTimeS + kReferenceWindowS / 2.0;

    ContiPoint previousPoint = trajectoryStartPoint_;
    bool collecting = false;
    double beginTimeS = 0.0;
    double endTimeS = 0.0;
    double pathLength = 0.0;
    for (auto iterator = pushedPointsByMark_.cbegin(); iterator != pushedPointsByMark_.cend(); ++iterator) {
        const ContiPoint &point = iterator.value();
        if (point.timeS < windowBegin) {
            previousPoint = point;
            continue;
        }
        if (!collecting) {
            collecting = true;
            beginTimeS = previousPoint.timeS;
        }
        const double dx = point.targetUnit[0] - previousPoint.targetUnit[0];
        const double dy = point.targetUnit[1] - previousPoint.targetUnit[1];
        pathLength += std::hypot(dx, dy);
        endTimeS = point.timeS;
        previousPoint = point;
        if (point.timeS >= windowEnd) {
            break;
        }
    }

    if (!collecting || endTimeS <= beginTimeS) {
        return 0.0;
    }
    return pathLength / (endTimeS - beginTimeS);
}

double ContiWorker::calculateRatioCommand(long currentMark, double bufferTimeS, qint64 elapsedMs)
{
    currentPlanTimeS_ = planTimeForMark(currentMark);
    const double scheduledPlanTimeS = firstEffectivePointTimeS_ + elapsedMs / 1000.0;
    const double sourceAvailablePlanTimeS = producerFinished_
        ? config_.durationS
        : std::max(firstEffectivePointTimeS_,
            latestGeneratedPlanTimeS_ - config_.executionDelayMs / 1000.0);
    expectedPlanTimeS_ = std::min({scheduledPlanTimeS, sourceAvailablePlanTimeS, config_.durationS});
    phaseErrorMs_ = (expectedPlanTimeS_ - currentPlanTimeS_) * 1000.0;
    bufferTimeMs_ = bufferTimeS * 1000.0;

    if (!config_.timeSyncEnabled) {
        ratioRef_ = 1.0;
        ratioPhase_ = 0.0;
        ratioBuffer_ = 0.0;
        ratioCommand_ = 1.0;
        ratioApplied_ = 1.0;
        config_.speedRatio = 1.0;
        return config_.speedRatio;
    }

    ratioRef_ = std::clamp(referenceVectorSpeed(expectedPlanTimeS_) / config_.maxVectorVelocity,
                            config_.ratioMin, config_.ratioMax);
    ratioPhase_ = std::abs(phaseErrorMs_) <= config_.phaseDeadbandMs
        ? 0.0
        : std::clamp(config_.phaseGainPerSecond * phaseErrorMs_ / 1000.0, -0.15, 0.15);
    const double targetBufferS = config_.targetBufferMs / 1000.0;
    ratioBuffer_ = targetBufferS > 0.0
        ? config_.bufferGain * std::clamp((bufferTimeS - targetBufferS) / targetBufferS, -1.0, 1.0)
        : 0.0;

    ratioCommand_ = std::clamp(ratioRef_ + ratioPhase_ + ratioBuffer_,
                               config_.ratioMin, config_.ratioMax);
    if (bufferTimeMs_ <= config_.criticalBufferMs) {
        ratioCommand_ = config_.ratioMin;
    } else if (bufferTimeMs_ < config_.lowBufferMs) {
        const double safetyFraction = (bufferTimeMs_ - config_.criticalBufferMs)
            / static_cast<double>(config_.lowBufferMs - config_.criticalBufferMs);
        ratioCommand_ = std::min(ratioCommand_,
                                 config_.ratioMin + std::clamp(safetyFraction, 0.0, 1.0) * 0.10);
    }

    config_.speedRatio = std::clamp(ratioCommand_,
                                    ratioApplied_ - config_.ratioMaxStep,
                                    ratioApplied_ + config_.ratioMaxStep);
    return config_.speedRatio;
}

bool ContiWorker::applyRatioCommand(qint64 elapsedMs, QString &errorMessage)
{
    const ContiSpeedRatioResult result = card_.changeSpeedRatio(config_, errorMessage);
    lastRatioApiMs_ = elapsedMs;
    if (result == ContiSpeedRatioResult::Applied) {
        ratioApplied_ = config_.speedRatio;
        speedRatioPending_ = false;
        if (config_.timeSyncEnabled && !phaseClockStarted_) {
            phaseClock_.start();
            phaseClockStarted_ = true;
            emit logMessage(QStringLiteral("时间同步计划时钟已启动：以首段规划确认时刻作为 t=0。"));
        }
        emit logMessage(QStringLiteral("连续插补在线速度倍率已生效：%1。")
                            .arg(ratioApplied_, 0, 'f', 3));
        return true;
    }
    if (result == ContiSpeedRatioResult::Failed) {
        return false;
    }

    speedRatioPending_ = true;
    ++speedRatioNotReadyCount_;
    if (speedRatioNotReadyCount_ == 1 || speedRatioNotReadyCount_ % 10 == 0) {
        emit logMessage(QStringLiteral("在线速度倍率等待首段规划：返回 5049，已重试 %1 次。")
                            .arg(speedRatioNotReadyCount_));
    }
    return true;
}

void ContiWorker::monitorContinuousRun()
{
    if (!running_) {
        return;
    }

    lastFeedStatus_ = card_.streamingStatus();
    if (lastFeedStatus_.failed) {
        enterError(lastFeedStatus_.errorText);
        return;
    }
    long current = lastFeedStatus_.currentMark;
    long remaining = lastFeedStatus_.remainSpace;
    short state = lastFeedStatus_.runState;
    lastPushedMark_ = lastFeedStatus_.lastPushedMark;
    lastPushedPlanTimeS_ = lastFeedStatus_.lastPushedPlanTimeS;
    double bufferedTimeS = std::max(0.0, lastFeedStatus_.lastPushedPlanTimeS
                                         - lastFeedStatus_.currentPlanTimeS);

    // 基准版本保持插补列表打开，直到控制卡真正完成全部运动。
    // 列表未关闭时，部分固件不会及时让 dmc_check_done_multicoor 返回完成，
    // 因此同时使用最终 mark、Trace 最终位置和静止速度组成结束判据。
    const bool allTrajectorySubmitted = producerFinished_
        && hostQueue_.isEmpty()
        && lastFeedStatus_.queuedPointCount == 0;
    const bool cardMotionDone = allTrajectorySubmitted && card_.contiMotionDone(config_);
    const bool finalMarkReached = allTrajectorySubmitted
        && lastPushedMark_ > 0
        // 不同固件可能把 currentMark 定义为“当前段”或“最近完成段”，
        // 自然结束时允许比最后下发 mark 小 1；最终位置与静止速度仍负责防止误判。
        && current >= std::max(0L, lastPushedMark_ - 1);
    const bool traceAtFinalTarget = finalMarkReached && traceReachedFinalTarget();
    if (traceAtFinalTarget) {
        if (!completionStableClock_.isValid()) {
            completionStableClock_.start();
        }
    } else {
        completionStableClock_.invalidate();
    }
    const bool traceStableDone = completionStableClock_.isValid()
        && completionStableClock_.elapsed() >= kCompletionStableDwellMs;
    if (allTrajectorySubmitted && (cardMotionDone || state == 2 || traceStableDone)) {
        const QString trajectoryName = config_.trajectoryPointMode == TrajectoryPointMode::UniformDistance
            ? QStringLiteral("等间距直线对照轨迹") : QStringLiteral("五次多项式轨迹");
        finishRun(QStringLiteral("%1已执行完成：mark=%2/%3，控制卡完成=%4，Trace终点稳态=%5。")
                      .arg(trajectoryName)
                      .arg(current)
                      .arg(lastPushedMark_)
                      .arg(cardMotionDone ? QStringLiteral("是") : QStringLiteral("否"))
                      .arg(traceStableDone ? QStringLiteral("是") : QStringLiteral("否")));
        return;
    }

    if (state == 2 && (!producerFinished_ || lastFeedStatus_.queuedPointCount > 0)) {
        enterError(QStringLiteral("连续插补在实时轨迹尚未完全执行时进入停止状态：mark=%1/%2，硬件待喂=%3；判定为流式断粮或卡侧提前结束。")
                       .arg(current)
                       .arg(lastPushedMark_)
                       .arg(lastFeedStatus_.queuedPointCount));
        return;
    }
    if (state == 3 || state == 4) {
        enterError(QStringLiteral("连续插补意外停止，runState=%1。") .arg(state));
        return;
    }

    const qint64 elapsedMs = contiRunElapsed_.isValid() ? contiRunElapsed_.elapsed() : 0;
    const qint64 phaseElapsedMs = config_.timeSyncEnabled
        ? (phaseClockStarted_ ? phaseClock_.elapsed() : 0)
        : elapsedMs;
    if (state == 0 && (lastRatioControlMs_ < 0
                       || elapsedMs - lastRatioControlMs_ >= config_.ratioUpdatePeriodMs)) {
        calculateRatioCommand(current, bufferedTimeS, phaseElapsedMs);
        const bool safetyOverride = bufferTimeMs_ < config_.lowBufferMs;
        const qint64 minApiIntervalMs = safetyOverride
            ? config_.ratioSafetyApiIntervalMs : config_.ratioApiMinIntervalMs;
        const bool apiIntervalElapsed = lastRatioApiMs_ < 0
            || elapsedMs - lastRatioApiMs_ >= minApiIntervalMs;
        const bool ratioChangeNeeded = std::abs(config_.speedRatio - ratioApplied_) >= config_.ratioDeadband;
        if (config_.timeSyncEnabled
            && apiIntervalElapsed && (speedRatioPending_ || ratioChangeNeeded)) {
            QString ratioError;
            if (!applyRatioCommand(elapsedMs, ratioError)) {
                enterError(ratioError);
                return;
            }
            lastRatioApiMark_ = current;
        }
        lastRatioControlMs_ = elapsedMs;
    }

    if (lastContiDiagnosticMs_ < 0 || elapsedMs - lastContiDiagnosticMs_ >= kContiDiagnosticPeriodMs) {
        lastContiDiagnosticMs_ = elapsedMs;
        const qint64 apiAgoMs = lastRatioApiMs_ < 0 ? -1 : elapsedMs - lastRatioApiMs_;
        emit logMessage(QStringLiteral("连续插补诊断：t=%1 ms，runState=%2，mark=%3/%4，卡侧计划=%5 ms，期望=%6 ms，相位=%7 ms，缓冲=%8 ms，倍率(ref/phase/buffer/cmd/applied)=%9/%10/%11/%12/%13，API距今=%14 ms，硬件待喂=%15，补段调度间隔=%16 us（最大=%17 us）。")
                            .arg(elapsedMs)
                            .arg(state)
                            .arg(current)
                            .arg(lastPushedMark_)
                            .arg(currentPlanTimeS_ * 1000.0, 0, 'f', 1)
                            .arg(expectedPlanTimeS_ * 1000.0, 0, 'f', 1)
                            .arg(phaseErrorMs_, 0, 'f', 1)
                            .arg(bufferTimeMs_, 0, 'f', 1)
                            .arg(ratioRef_, 0, 'f', 3)
                            .arg(ratioPhase_, 0, 'f', 3)
                            .arg(ratioBuffer_, 0, 'f', 3)
                            .arg(ratioCommand_, 0, 'f', 3)
                            .arg(ratioApplied_, 0, 'f', 3)
                            .arg(apiAgoMs)
                            .arg(lastFeedStatus_.queuedPointCount)
                            .arg(lastFeedStatus_.lastServiceGapUs)
                            .arg(lastFeedStatus_.maxServiceGapUs));
        double cardVectorSpeed = 0.0;
        QString vectorSpeedError;
        if (card_.readVectorSpeedDegreePerSecond(config_, cardVectorSpeed, vectorSpeedError)) {
            vectorSpeedReadFailureLogged_ = false;
            const QString traceVelocityText = traceVelocityValid_
                ? QStringLiteral("%1/%2°/s")
                      .arg(traceCommandVelocityDegreePerSecond_, 0, 'f', 3)
                      .arg(traceActualVelocityDegreePerSecond_, 0, 'f', 3)
                : QStringLiteral("等待有效窗口");
            emit logMessage(QStringLiteral("速度诊断：板卡当前矢量速度=%1°/s，主动轴 Trace 10 ms 平均速度(指令/实际)=%2，卡侧未来段=%3。")
                                .arg(cardVectorSpeed, 0, 'f', 3)
                                .arg(traceVelocityText)
                                .arg(std::max(0L, lastPushedMark_ - current)));
        } else if (!vectorSpeedReadFailureLogged_) {
            vectorSpeedReadFailureLogged_ = true;
            emit logMessage(QStringLiteral("警告：读取板卡当前矢量速度失败：%1。")
                                .arg(vectorSpeedError));
        }
    }
    if (current < 1 && elapsedMs >= kFirstSegmentTimeoutMs) {
        enterError(QStringLiteral("启动后 %1 ms 控制卡仍未消费首个有效段（currentMark=%2，pushedMark=%3，缓冲余量=%4）；已安全停止，需检查首段与控制卡插补状态。")
                       .arg(elapsedMs)
                       .arg(current)
                       .arg(lastPushedMark_)
                       .arg(remaining));
        return;
    }

    publishStatus();
}

bool ContiWorker::traceReachedFinalTarget() const
{
    if (pushedPointsByMark_.isEmpty()) {
        return false;
    }

    const ContiPoint &finalPoint = pushedPointsByMark_.last();
    const auto axisAtTarget = [this](quint16 axis, double targetDegree) {
        if (axis >= static_cast<quint16>(latestAxisFeedback_.size())) {
            return false;
        }
        const AxisFeedback &feedback = latestAxisFeedback_.at(axis);
        return feedback.traceSampleValid
            && std::abs(feedback.commandPositionUnit - targetDegree)
                   <= kCompletionPositionToleranceDeg
            && std::abs(feedback.encoderPositionUnit - targetDegree)
                   <= kCompletionPositionToleranceDeg;
    };

    return axisAtTarget(config_.activeAxis, finalPoint.targetUnit[0])
        && axisAtTarget(config_.holdAxis, finalPoint.targetUnit[1])
        && traceVelocityValid_
        && std::abs(traceCommandVelocityDegreePerSecond_)
               <= kCompletionVelocityToleranceDegPerSecond
        && std::abs(traceActualVelocityDegreePerSecond_)
               <= kCompletionVelocityToleranceDegPerSecond;
}

void ContiWorker::finishRun(const QString &message)
{
    telemetryRecorder_.appendEvent(QStringLiteral("conti_test_completed"));
    producerTimer_->stop();
    monitorTimer_->stop();
    QString error;
    if (listOpen_ && !card_.closeList(config_, error)) {
        emit logMessage(QStringLiteral("轨迹结束后关闭缓冲区失败：%1").arg(error));
    }
    listOpen_ = false;
    preparing_ = false;
    running_ = false;
    trajectoryComparisonActive_ = false;
    speedRatioPending_ = false;
    lastFeedStatus_ = {};
    resetRunTimingState();
    stateText_ = QStringLiteral("轨迹完成");
    emit logMessage(message);
    publishStatus();
}

void ContiWorker::enterError(const QString &message)
{
    telemetryRecorder_.appendEvent(QStringLiteral("control_error: %1").arg(message));
    producerTimer_->stop();
    monitorTimer_->stop();
    velocityControlTimer_->stop();
    torqueTestTimer_->stop();
    traceDelayCalibrationTimer_->stop();
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
    if (velocityControlActive_ && boardInitialized_) {
        flushVelocityPlotSamples();
        QString ignored;
        card_.stopAxis(initializedCardNo_, velocityConfig_.axis, true, ignored);
    }
    if (torqueTestActive_ && boardInitialized_) {
        flushTorquePlotSamples();
        QString ignored;
        card_.stopAxis(initializedCardNo_, torqueConfig_.axis, true, ignored);
    }
    if (traceDelayCalibrationActive_ && boardInitialized_) {
        flushTraceDelayPlotSamples();
        QString ignored;
        card_.stopAxis(initializedCardNo_, traceDelayConfig_.axis, true, ignored);
    }
    listOpen_ = false;
    preparing_ = false;
    running_ = false;
    velocityControlActive_ = false;
    velocityMotionStarted_ = false;
    velocityReferenceInitialized_ = false;
    velocityStatus_.active = false;
    velocityStatus_.motionStarted = false;
    velocityStatus_.stateText = QStringLiteral("错误");
    torqueTestActive_ = false;
    torqueMotionStarted_ = false;
    torqueStatus_.active = false;
    torqueStatus_.stateText = QStringLiteral("错误");
    traceDelayCalibrationActive_ = false;
    traceDelayMotionStarted_ = false;
    traceDelayStatus_.active = false;
    traceDelayStatus_.phaseText = QStringLiteral("错误");
    if (velocityAutoRecording_) {
        telemetryRecorder_.stop();
        velocityAutoRecording_ = false;
    }
    if (traceDelayAutoRecording_) {
        telemetryRecorder_.stop();
        traceDelayAutoRecording_ = false;
    }
    trajectoryComparisonActive_ = false;
    speedRatioPending_ = false;
    lastFeedStatus_ = {};
    resetRunTimingState();
    stateText_ = QStringLiteral("错误");
    emit logMessage(QStringLiteral("错误：%1").arg(message));
    publishStatus();
}

void ContiWorker::resetRunTimingState()
{
    contiRunElapsed_.invalidate();
    completionStableClock_.invalidate();
    phaseClock_.invalidate();
    phaseClockStarted_ = false;
    lastRatioControlMs_ = -1;
    lastRatioApiMs_ = -1;
    lastRatioApiMark_ = -1;
    speedRatioNotReadyCount_ = 0;
}

void ContiWorker::publishStatus()
{
    ContiStatus status;
    status.boardInitialized = boardInitialized_;
    for (const quint16 axis : enabledAxes_) {
        if (axis < 8U) {
            status.enabledAxisMask |= static_cast<quint16>(1U << axis);
        }
    }
    status.detectedBoardCount = detectedBoardCount_;
    status.detectedAxisCount = detectedAxes_.size();
    status.cardNo = initializedCardNo_;
    status.busCycleUs = actualBusCycleUs_;
    status.traceCycle = config_.traceCycle;
    status.producerPeriodMs = config_.producerPeriodMs;
    status.running = running_;
    status.stateText = stateText_;
    status.hostQueueSize = hostQueue_.size() + lastFeedStatus_.queuedPointCount;
    status.pushedMark = lastPushedMark_;
    status.currentPlanTimeS = currentPlanTimeS_;
    status.expectedPlanTimeS = expectedPlanTimeS_;
    status.phaseErrorMs = phaseErrorMs_;
    status.bufferTimeMs = bufferTimeMs_;
    status.ratioRef = ratioRef_;
    status.ratioPhase = ratioPhase_;
    status.ratioBuffer = ratioBuffer_;
    status.ratioCommand = ratioCommand_;
    status.ratioApplied = ratioApplied_;
    status.ratioLastApiAgoMs = lastRatioApiMs_ < 0 || !contiRunElapsed_.isValid()
        ? -1 : contiRunElapsed_.elapsed() - lastRatioApiMs_;
    status.trajectoryComparisonActive = trajectoryComparisonActive_;
    status.trajectoryActiveAxis = config_.activeAxis;
    status.trajectoryTraceStartTimeUs = trajectoryTraceStartTimeUs_;
    if (trajectoryComparisonActive_ && contiRunElapsed_.isValid()) {
        status.trajectoryExpectedTimeS = std::clamp(contiRunElapsed_.elapsed() / 1000.0,
                                                     0.0, config_.durationS);
        status.trajectoryExpectedActiveUnit = trajectory_.pointAt(status.trajectoryExpectedTimeS)
                                                   .targetUnit[0];
    }
    if (listOpen_) {
        status.currentMark = lastFeedStatus_.currentMark;
        status.remainSpace = lastFeedStatus_.remainSpace;
        status.runState = lastFeedStatus_.runState;
    }
    status.traceConfigured = card_.traceConfigured();
    status.traceEverRead = card_.traceEverRead();
    status.traceFramesRead = card_.traceFramesRead();
    status.traceLastApiResult = card_.traceLastApiResult();
    status.traceStateText = card_.traceStateText();
    status.latestTraceSequence = latestTraceSequence_;
    status.latestTraceTimeUs = latestTraceTimeUs_;
    status.traceSamplePeriodUs = card_.traceSamplePeriodUs();
    status.telemetryPlotActive = manualTelemetryRecording_;
    status.recorder = telemetryRecorder_.status();
    status.velocityControl = velocityStatus_;
    status.torqueTest = torqueStatus_;
    traceDelayStatus_.axisResults =
        traceDelayStatus_.active ? traceDelayStatus_.axisResults
                                 : (traceDelayStatus_.axisResults.isEmpty()
                                        ? traceDelayAxisResults_
                                        : traceDelayStatus_.axisResults);
    status.traceDelayCalibration = traceDelayStatus_;
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
    if (detectedAxes_.isEmpty()) {
        emit logMessage(QStringLiteral("基础轴配置失败：没有检测到 EtherCAT 伺服轴。"));
        return false;
    }
    for (const quint16 axis : detectedAxes_) {
        QString error;
        if (!card_.setAxisEquivalent(config.cardNo, axis,
                                     MotorUnit::pulsesPerCardUnit(config.degreesPerCardUnit), error)) {
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
    traceConfig.samplePeriodUs = 1000;
    traceConfig.traceBaseCycleUs = 1000;
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
        object.scale = 1.0 / MotorUnit::kPhysicalPulsesPerDegree;
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
        object.scale = 1.0 / MotorUnit::kPhysicalPulsesPerDegree;
        traceConfig.objects.push_back(object);
    }

    if (!feedbackTrace_.configure(traceConfig)) {
        errorMessage = QStringLiteral("dmc_trace_* 返回码=%1")
                           .arg(feedbackTrace_.lastApiResult());
        traceStateText_ = QStringLiteral("Trace 配置失败");
        return false;
    }
    traceFramesRead_ = 0;
    traceStateText_ = QStringLiteral("Trace 已配置：1 ms，同帧 type 5/6 位置");
    QStringList axisText;
    for (const quint16 axis : traceAxes_) {
        axisText << QString::number(axis);
    }
    emit logMessage(QStringLiteral("位置 Trace 已配置：轴 %1；1 ms 采样，type 5 指令位置 / type 6 实际位置。")
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
                                         double degreesPerCardUnit,
                                         QString &errorMessage)
{
    if (telemetryRecorder_.status().recording) {
        errorMessage = QStringLiteral("Trace 数据记录进行中，不能切换测试轴或重配 Trace；请先停止记录。");
        return false;
    }
    const int traceSamplePeriodUs = config_.busCycleUs * config_.traceCycle;
    if (config_.busCycleUs <= 0 || config_.traceCycle <= 0
        || traceSamplePeriodUs % config_.busCycleUs != 0) {
        errorMessage = QStringLiteral("Trace 周期参数无效：总线=%1 us，Trace 倍数=%2")
                           .arg(config_.busCycleUs)
                           .arg(config_.traceCycle);
        return false;
    }
    if (!card_.configureTrace(axes, traceSamplePeriodUs, config_.busCycleUs,
                              degreesPerCardUnit, errorMessage)) {
        return false;
    }
    resetTraceDelayHistory();
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
        applyTraceDelayCompensation(frames);
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
    QVector<TraceTelemetryFrame> frames = card_.takeTraceTelemetryFrames();
    if (!frames.isEmpty()) {
        const long currentMark = listOpen_ ? card_.currentMark(config_) : -1;
        for (TraceTelemetryFrame &frame : frames) {
            frame.currentMark = static_cast<qint32>(currentMark);
            frame.pushedMark = static_cast<qint32>(lastPushedMark_);
        }
        if (trajectoryComparisonActive_ && trajectoryTraceStartTimeUs_ == 0) {
            trajectoryTraceStartTimeUs_ = frames.constFirst().traceTimeUs;
        }
        latestTraceSequence_ = frames.constLast().traceSequence;
        latestTraceTimeUs_ = frames.constLast().traceTimeUs;
        applyTraceDelayCompensation(frames);
        appendTraceDelayCalibrationFrames(frames);
        updateTraceVelocityDiagnostics(frames);
        appendVelocityPlotFrames(frames);
        telemetryRecorder_.pushFrames(frames);
    }
    return true;
}

void ContiWorker::updateTraceVelocityDiagnostics(const QVector<TraceTelemetryFrame> &frames)
{
    constexpr quint64 kVelocityWindowUs = 10000;
    for (const TraceTelemetryFrame &frame : frames) {
        int activeIndex = -1;
        for (int index = 0; index < frame.axisCount; ++index) {
            if (frame.axes[index] == config_.activeAxis) {
                activeIndex = index;
                break;
            }
        }
        if (activeIndex < 0) {
            continue;
        }

        if (!traceVelocityAnchorValid_ || traceVelocityAxis_ != config_.activeAxis
            || frame.traceTimeUs <= traceVelocityAnchorTimeUs_) {
            traceVelocityAnchorValid_ = true;
            traceVelocityAxis_ = config_.activeAxis;
            traceVelocityAnchorTimeUs_ = frame.traceTimeUs;
            traceVelocityAnchorCommandPulse_ = frame.commandPulse[activeIndex];
            traceVelocityAnchorActualPulse_ = frame.actualPulse[activeIndex];
            continue;
        }

        const quint64 elapsedUs = frame.traceTimeUs - traceVelocityAnchorTimeUs_;
        if (elapsedUs < kVelocityWindowUs) {
            continue;
        }
        const double elapsedS = static_cast<double>(elapsedUs) / 1000000.0;
        traceCommandVelocityDegreePerSecond_ =
            (frame.commandPulse[activeIndex] - traceVelocityAnchorCommandPulse_)
            / MotorUnit::kPhysicalPulsesPerDegree / elapsedS;
        traceActualVelocityDegreePerSecond_ =
            (frame.actualPulse[activeIndex] - traceVelocityAnchorActualPulse_)
            / MotorUnit::kPhysicalPulsesPerDegree / elapsedS;
        traceVelocityValid_ = true;
        traceVelocityAnchorTimeUs_ = frame.traceTimeUs;
        traceVelocityAnchorCommandPulse_ = frame.commandPulse[activeIndex];
        traceVelocityAnchorActualPulse_ = frame.actualPulse[activeIndex];
    }
}

void ContiWorker::refreshAxisStates()
{
    latestAxisFeedback_ = card_.axisFeedback();
}

bool ContiWorker::bothAxesEnabled(const ContiTestConfig &config) const
{
    return enabledAxes_.contains(config.activeAxis) && enabledAxes_.contains(config.holdAxis);
}
