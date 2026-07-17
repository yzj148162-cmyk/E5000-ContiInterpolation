#ifndef TELEMETRYRECORDER_H
#define TELEMETRYRECORDER_H

#include <atomic>

#include <QMutex>
#include <QString>
#include <QVector>

#include "common/ContiTypes.h"

class QThread;
class TelemetryWriterWorker;

struct TelemetryRunMetadata
{
    quint16 cardNo = 0;
    QVector<quint16> axes;
    int traceSamplePeriodUs = 1000;
    QString rootDirectory;
    QString description;
};

// 固定容量 SPSC 环形队列：ContiWorker 是唯一生产者，写盘线程是唯一消费者。
// 写盘、JSON 与 flush 都不在硬件线程或 ContiWorker 的 Trace 读取路径执行。
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
    void appendEvent(const QString &eventText);
    TelemetryRecorderStatus status() const;

private:
    friend class TelemetryWriterWorker;

    int takeBatch(QVector<TraceTelemetryFrame> &batch, int maximum);
    void setWriterError(const QString &errorText);
    void setOutputDirectory(const QString &directory);
    void addWrittenFrames(quint64 count);

    static constexpr quint32 kRingCapacity = 32768;
    QVector<TraceTelemetryFrame> ring_;
    std::atomic<quint32> writeIndex_ {0};
    std::atomic<quint32> readIndex_ {0};
    std::atomic<quint64> writtenFrames_ {0};
    std::atomic<quint64> droppedFrames_ {0};
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
