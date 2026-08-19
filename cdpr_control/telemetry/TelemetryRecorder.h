#ifndef TELEMETRYRECORDER_H
#define TELEMETRYRECORDER_H

#include <atomic>
#include <array>

#include <QMutex>
#include <QJsonObject>
#include <QJsonValue>
#include <QString>
#include <QVector>

#include "common/ContiTypes.h"
#include "cdpr/CdprControlTypes.h"

class QThread;
class TelemetryWriterWorker;

struct ControlCycleTimingSample
{
    quint64 runId = 0;
    quint64 cycleIndex = 0;
    quint64 hostElapsedUs = 0;
    quint32 targetPeriodUs = 0;
    quint32 schedulingIntervalUs = 0;
    quint32 tracePollUs = 0;
    quint32 calculationUs = 0;
    quint32 apiTotalUs = 0;
    quint32 fullCycleUs = 0;
    std::array<quint32, 8> axisApiUs {};
    quint16 slowestAxis = 0;
    quint16 traceFramesRead = 0;
    quint32 estimatedMissedCycles = 0;
};

struct TelemetryRunMetadata
{
    quint16 cardNo = 0;
    QVector<quint16> axes;
    int traceSamplePeriodUs = 1000;
    double degreesPerCardUnit = 1.0;
    TraceFeedbackProfile traceProfile = TraceFeedbackProfile::FullPositionVelocity;
    QString rootDirectory;
    QString description;
};

// 连续力交互每个控制周期的一致性快照。该结构只进入SPSC队列，
// CSV格式化与写盘由TelemetryWriterWorker异步完成。
struct CdprForceControlTelemetrySample
{
    quint64 runId = 0;
    quint64 cycleIndex = 0;
    quint64 hostElapsedUs = 0;
    quint64 traceSequence = 0;
    CdprVector6 sensorWrench {};
    CdprVector6 platformWrench {};
    CdprVector6 desiredPose {};
    CdprVector6 desiredTwist {};
    CdprVector8 desiredCableLengthM {};
    CdprVector8 desiredCableVelocityMps {};
    CdprVector8 targetAxisPositionDegree {};
    CdprVector8 targetAxisVelocityDegreePerSecond {};
    CdprVector8 actualAxisPositionDegree {};
    CdprVector8 actualAxisVelocityDegreePerSecond {};
    double maximumTrackingErrorDegree = 0.0;
    double maximumCableTravelM = 0.0;
};

// 固定容量 SPSC 环形队列：MotionControlWorker 是唯一生产者，写盘线程是唯一消费者。
// 写盘、JSON 与 flush 都不在硬件线程或 MotionControlWorker 的 Trace 读取路径执行。
class TelemetryRecorder
{
public:
    TelemetryRecorder();
    ~TelemetryRecorder();
    TelemetryRecorder(const TelemetryRecorder &) = delete;
    TelemetryRecorder &operator=(const TelemetryRecorder &) = delete;

    bool start(const TelemetryRunMetadata &metadata, QString &errorMessage);
    void stop();
    void pushFrames(const QVector<TraceTelemetryFrame> &frames);
    void pushControlCycleTiming(const ControlCycleTimingSample &sample);
    void pushCdprForceControlSample(
        const CdprForceControlTelemetrySample &sample);
    void appendEvent(const QString &eventText);
    bool initializeRunContext(const QJsonObject &context, QString &errorMessage);
    void updateRunContext(const QString &key, const QJsonValue &value);
    TelemetryRecorderStatus status() const;

private:
    friend class TelemetryWriterWorker;

    int takeBatch(QVector<TraceTelemetryFrame> &batch, int maximum);
    int takeTimingBatch(QVector<ControlCycleTimingSample> &batch, int maximum);
    int takeForceControlBatch(
        QVector<CdprForceControlTelemetrySample> &batch, int maximum);
    void setWriterError(const QString &errorText);
    void setOutputDirectory(const QString &directory);
    void addWrittenFrames(quint64 count);
    void addWrittenTimingSamples(quint64 count);
    void addWrittenForceControlSamples(quint64 count);

    static constexpr quint32 kRingCapacity = 32768;
    QVector<TraceTelemetryFrame> ring_;
    std::atomic<quint32> writeIndex_ {0};
    std::atomic<quint32> readIndex_ {0};
    std::atomic<quint64> writtenFrames_ {0};
    std::atomic<quint64> droppedFrames_ {0};
    static constexpr quint32 kTimingRingCapacity = 32768;
    QVector<ControlCycleTimingSample> timingRing_;
    std::atomic<quint32> timingWriteIndex_ {0};
    std::atomic<quint32> timingReadIndex_ {0};
    std::atomic<quint64> writtenTimingSamples_ {0};
    std::atomic<quint64> droppedTimingSamples_ {0};
    static constexpr quint32 kForceControlRingCapacity = 32768;
    QVector<CdprForceControlTelemetrySample> forceControlRing_;
    std::atomic<quint32> forceControlWriteIndex_ {0};
    std::atomic<quint32> forceControlReadIndex_ {0};
    std::atomic<quint64> writtenForceControlSamples_ {0};
    std::atomic<quint64> droppedForceControlSamples_ {0};
    std::atomic_bool recording_ {false};
    quint64 firstRecordedTraceTimeUs_ = 0;
    bool hasFirstRecordedTraceTime_ = false;
    mutable QMutex statusMutex_;
    QString outputDirectory_;
    QString errorText_;
    QThread *writerThread_ = nullptr;
    TelemetryWriterWorker *writer_ = nullptr;
};

#endif // TELEMETRYRECORDER_H
