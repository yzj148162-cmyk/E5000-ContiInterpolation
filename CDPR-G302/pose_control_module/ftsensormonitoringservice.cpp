#include "ftsensormonitoringservice.h"

#include <QTimer>
#include <QtGlobal>

#include <chrono>

namespace {
qint64 serviceMonotonicUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}
}

FtSensorMonitoringService::FtSensorMonitoringService(
        HardwareInterface* hardware,
        QObject* parent)
    : QObject(parent)
    , hardware_(hardware)
{
}

FtSensorMonitoringService::~FtSensorMonitoringService()
{
    if(pollTimer_){
        pollTimer_->stop();
    }
    if(monitoring_){
        stopMonitoring();
    }
    recorder_.reset();
}

void FtSensorMonitoringService::ensureTimer()
{
    if(pollTimer_){
        return;
    }
    pollTimer_ = new QTimer(this);
    pollTimer_->setTimerType(Qt::PreciseTimer);
    pollTimer_->setInterval(10);
    connect(pollTimer_, &QTimer::timeout, this, [this](){
        consumeAvailableSamples();
    });
}

bool FtSensorMonitoringService::startMonitoring(
        const FtSensorStabilityConfig& config,
        bool previouslyPowered,
        bool automaticZeroEnabled,
        HardwareInterface::RuntimeTraceUsageProfile expectedProfile,
        const QString& recordDirectory,
        QString* recordPath,
        QString* errorMessage)
{
    if(monitoring_){
        if(errorMessage){
            *errorMessage = QStringLiteral("后台F/T监测已经运行");
        }
        return false;
    }
    if(!hardware_){
        if(errorMessage){
            *errorMessage = QStringLiteral("硬件接口不可用");
        }
        return false;
    }

    ensureTimer();
    if(!recorder_){
        // 在本服务所属后台线程中创建QThread对象，避免其QObject线程亲和性
        // 停留在GUI线程；真正写盘仍由记录器自己的低优先级线程执行。
        recorder_ = std::make_unique<FtSensorTraceRecorder>();
    }
    const HardwareInterface::FtSensorTraceBatch stale =
            hardware_->takeFtSensorTraceSamples();
    queueDroppedAtStart_ = stale.queueDroppedTotal;
    queueDroppedLatest_ = stale.queueDroppedTotal;
    expectedProfile_ = expectedProfile;
    traceSamplePeriodUs_ = stale.traceSamplePeriodUs;
    fifoValidNum_ = stale.fifoValidNum;
    lastBatchSize_ = 0;
    traceConfigured_ = false;
    timingReliable_ = false;
    traceLost_ = false;
    automaticZeroEnabled_ = automaticZeroEnabled;
    automaticZeroConfirmed_ = false;
    externalTraceReaderActive_ = false;
    latestSample_ = FtSensorTraceSample{};

    QString path;
    QString recorderError;
    if(!recorder_->begin(recordDirectory, &path, &recorderError)){
        if(errorMessage){
            *errorMessage = recorderError;
        }
        return false;
    }
    recordPath_ = path;
    monitor_.configure(config);
    // 每次新监测都从“本轮尚无零点”开始，避免上一次状态被无提示沿用。
    // 如操作者确认传感器未断电，由GUI在启动成功后显式恢复缓存零漂。
    monitor_.clearZero();
    // 启动时刻必须在本服务线程中取得，不能接收GUI排队前捕获的时间。
    monitor_.start(previouslyPowered, serviceMonotonicUs());
    latestPreheatStatus_ = FtSensorPreheatStatus{};
    monitoring_ = true;
    pollTimer_->start();
    consumeAvailableSamples();
    if(recordPath){
        *recordPath = recordPath_;
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

void FtSensorMonitoringService::consumeAvailableSamples()
{
    if(!monitoring_ || !hardware_){
        return;
    }
    const HardwareInterface::FtSensorTraceBatch batch =
            hardware_->takeFtSensorTraceSamples(!externalTraceReaderActive_);
    traceSamplePeriodUs_ = batch.traceSamplePeriodUs;
    fifoValidNum_ = batch.fifoValidNum;
    lastBatchSize_ = static_cast<int>(batch.samples.size());
    queueDroppedLatest_ = batch.queueDroppedTotal;
    timingReliable_ = batch.timingReliable;
    traceLost_ = batch.traceLost;
    traceConfigured_ = batch.usageProfile == expectedProfile_ &&
            batch.fromTrace && batch.latest.wrenchComplete() &&
            batch.latest.statusValid && batch.latest.sampleCounterValid &&
            batch.latest.temperatureValid;

    if(batch.latest.wrenchComplete()){
        latestSample_ = batch.latest;
    }
    if(!batch.samples.empty()){
        // 原始完整批次先进入有界异步记录器；判稳只做内存计算，不阻塞磁盘。
        recorder_->tryAppend(batch.samples);
        monitor_.ingest(batch.samples);
    }

    const qint64 nowUs = serviceMonotonicUs();
    latestPreheatStatus_ = monitor_.status(nowUs);
    if(automaticZeroEnabled_ && latestPreheatStatus_.stable &&
            !latestPreheatStatus_.zeroValid){
        QString ignored;
        if(monitor_.confirmZero(nowUs, &ignored)){
            automaticZeroConfirmed_ = true;
            latestPreheatStatus_ = monitor_.status(serviceMonotonicUs());
        }
    }
}

FtSensorMonitoringService::Snapshot FtSensorMonitoringService::snapshot() const
{
    Snapshot result;
    result.monitoring = monitoring_;
    result.traceConfigured = traceConfigured_;
    result.timingReliable = timingReliable_;
    result.traceLost = traceLost_;
    result.automaticZeroConfirmed = automaticZeroConfirmed_;
    result.traceSamplePeriodUs = traceSamplePeriodUs_;
    result.fifoValidNum = fifoValidNum_;
    result.lastBatchSize = lastBatchSize_;
    result.queueDroppedSamples = queueDroppedLatest_ >= queueDroppedAtStart_ ?
                queueDroppedLatest_ - queueDroppedAtStart_ : 0;
    result.latestSample = latestSample_;
    // Snapshot只返回后台采集循环已经计算好的只读缓存。GUI刷新频率和
    // BlockingQueuedConnection排队顺序不得推进或重置判稳状态机。
    result.preheat = latestPreheatStatus_;
    result.recordPath = recordPath_;
    return result;
}

void FtSensorMonitoringService::setTraceConsumptionMode(
        HardwareInterface::RuntimeTraceUsageProfile expectedProfile,
        bool externalTraceReaderActive)
{
    expectedProfile_ = expectedProfile;
    externalTraceReaderActive_ = externalTraceReaderActive;
    // profile切换会令板卡Trace帧序号重新从零开始。旧序号只能在旧epoch内
    // 判断连续性；预热资格、统计窗口和软件零点则必须保留。
    monitor_.beginTraceEpoch();
    latestSample_ = FtSensorTraceSample{};
    timingReliable_ = false;
    traceLost_ = false;
    // profile切换后旧的“已配置”结论不能沿用，下一批数据重新建立。
    traceConfigured_ = false;
    if(monitoring_){
        consumeAvailableSamples();
    }
}

bool FtSensorMonitoringService::confirmZero(QString* errorMessage)
{
    // 按钮请求到达服务线程后再取得时间，避免使用GUI排队前的旧时间。
    consumeAvailableSamples();
    const bool confirmed = monitor_.confirmZero(serviceMonotonicUs(), errorMessage);
    latestPreheatStatus_ = monitor_.status(serviceMonotonicUs());
    return confirmed;
}

bool FtSensorMonitoringService::restoreConfirmedZero(
        const std::array<double, kFtSensorWrenchChannelCount>& zero,
        QString* errorMessage)
{
    consumeAvailableSamples();
    const bool restored = monitor_.restoreConfirmedZero(zero, errorMessage);
    latestPreheatStatus_ = monitor_.status(serviceMonotonicUs());
    return restored;
}

void FtSensorMonitoringService::clearZero()
{
    monitor_.clearZero();
    automaticZeroConfirmed_ = false;
    latestPreheatStatus_ = monitor_.status(serviceMonotonicUs());
}

FtSensorMonitoringService::StopReport
FtSensorMonitoringService::stopMonitoring()
{
    StopReport report;
    if(!monitoring_){
        return report;
    }
    if(pollTimer_){
        pollTimer_->stop();
    }
    consumeAvailableSamples();
    report.finalStatus = latestPreheatStatus_;
    monitor_.stop();
    recorder_->finishAndWait();
    report.acceptedForRecording = recorder_->acceptedCount();
    report.written = recorder_->writtenCount();
    report.recorderDropped = recorder_->droppedCount();
    report.queueDropped = queueDroppedLatest_ >= queueDroppedAtStart_ ?
                queueDroppedLatest_ - queueDroppedAtStart_ : 0;
    report.recordPath = recorder_->outputPath();

    // 停止预热采集不等于传感器重新上电。保留已确认的软件零点，
    // 便于后续切换到阶段C；只有显式“清除软件零点”才使其失效。
    monitor_.reset();
    monitoring_ = false;
    traceConfigured_ = false;
    externalTraceReaderActive_ = false;
    automaticZeroConfirmed_ = false;
    latestSample_ = FtSensorTraceSample{};
    return report;
}

bool FtSensorMonitoringService::isMonitoring() const
{
    return monitoring_;
}
