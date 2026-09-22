#ifndef FTSENSORMONITORINGSERVICE_H
#define FTSENSORMONITORINGSERVICE_H

#include "ftsensorpreheatmonitor.h"
#include "ftsensortracerecorder.h"
#include "hardwareinterface.h"

#include <QObject>
#include <QString>

#include <memory>

class QTimer;

// 六维F/T预热与记录后台服务。
//
// 本对象移动到独立QThread后使用：它是F/T逐帧队列的唯一消费者，负责
// 批量取数、判稳、软件零点和异步记录。GUI只能读取Snapshot，不参与
// 采集推进，也不会因为切页或刷新变慢而破坏原始数据连续性。
class FtSensorMonitoringService final : public QObject
{
public:
    struct Snapshot {
        bool monitoring = false;
        bool traceConfigured = false;
        bool timingReliable = false;
        bool traceLost = false;
        bool automaticZeroConfirmed = false;
        int traceSamplePeriodUs = 0;
        int fifoValidNum = 0;
        int lastBatchSize = 0;
        quint64 queueDroppedSamples = 0;
        FtSensorTraceSample latestSample;
        FtSensorPreheatStatus preheat;
        QString recordPath;
    };

    struct StopReport {
        quint64 acceptedForRecording = 0;
        quint64 written = 0;
        quint64 recorderDropped = 0;
        quint64 queueDropped = 0;
        FtSensorPreheatStatus finalStatus;
        QString recordPath;
    };

    explicit FtSensorMonitoringService(HardwareInterface* hardware,
                                       QObject* parent = nullptr);
    ~FtSensorMonitoringService() override;

    bool startMonitoring(
            const FtSensorStabilityConfig& config,
            bool previouslyPowered,
            bool automaticZeroEnabled,
            HardwareInterface::RuntimeTraceUsageProfile expectedProfile,
            const QString& recordDirectory,
            QString* recordPath,
            QString* errorMessage);
    StopReport stopMonitoring();
    Snapshot snapshot() const;
    // 阶段C运行时ControlWorker是唯一Trace推进者；本服务继续消费已解析的
    // F/T队列、判稳和记录，但不再发起第二次板卡Trace读取。
    void setTraceConsumptionMode(
            HardwareInterface::RuntimeTraceUsageProfile expectedProfile,
            bool externalTraceReaderActive);
    bool confirmZero(QString* errorMessage);
    bool restoreConfirmedZero(
            const std::array<double, kFtSensorWrenchChannelCount>& zero,
            QString* errorMessage);
    void clearZero();
    bool isMonitoring() const;

private:
    void ensureTimer();
    void consumeAvailableSamples();

    HardwareInterface* hardware_ = nullptr;
    QTimer* pollTimer_ = nullptr;
    FtSensorPreheatMonitor monitor_;
    FtSensorPreheatStatus latestPreheatStatus_;
    std::unique_ptr<FtSensorTraceRecorder> recorder_;
    FtSensorTraceSample latestSample_;
    HardwareInterface::RuntimeTraceUsageProfile expectedProfile_ =
            HardwareInterface::RuntimeTraceUsageProfile::Base;
    QString recordPath_;
    quint64 queueDroppedAtStart_ = 0;
    quint64 queueDroppedLatest_ = 0;
    int traceSamplePeriodUs_ = 0;
    int fifoValidNum_ = 0;
    int lastBatchSize_ = 0;
    bool monitoring_ = false;
    bool traceConfigured_ = false;
    bool timingReliable_ = false;
    bool traceLost_ = false;
    bool automaticZeroEnabled_ = false;
    bool automaticZeroConfirmed_ = false;
    bool externalTraceReaderActive_ = false;
};

#endif // FTSENSORMONITORINGSERVICE_H
