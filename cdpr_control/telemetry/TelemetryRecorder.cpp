#include "telemetry/TelemetryRecorder.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <utility>

namespace {
constexpr int kWriterIntervalMs = 100;
constexpr int kWriterBatchFrames = 4096;
constexpr int kWriterBatchTimingSamples = 4096;

QString createRunDirectory(const QString &rootDirectory, QString &error)
{
    QDir root(rootDirectory);
    if (!root.exists() && !QDir().mkpath(root.absolutePath())) {
        error = QStringLiteral("无法创建记录根目录：%1").arg(root.absolutePath());
        return {};
    }
    const QString baseName = QStringLiteral("run_%1")
                                 .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    for (int suffix = 0; suffix < 1000; ++suffix) {
        const QString name = suffix == 0 ? baseName : QStringLiteral("%1_%2").arg(baseName).arg(suffix);
        if (root.mkdir(name)) {
            return root.filePath(name);
        }
    }
    error = QStringLiteral("无法在记录根目录创建运行目录：%1").arg(root.absolutePath());
    return {};
}
}

class TelemetryWriterWorker : public QObject
{
public:
    explicit TelemetryWriterWorker(TelemetryRecorder *recorder)
        : recorder_(recorder)
    {
    }

    bool begin(const TelemetryRunMetadata &metadata, QString &error)
    {
        finish();
        runDirectory_ = createRunDirectory(metadata.rootDirectory, error);
        if (runDirectory_.isEmpty()) {
            return false;
        }

        QJsonObject root;
        root.insert(QStringLiteral("format"), QStringLiteral("Leadshine Motion Card Trace Telemetry"));
    root.insert(QStringLiteral("formatVersion"), 3);
        root.insert(QStringLiteral("createdAt"), QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
        root.insert(QStringLiteral("cardNo"), static_cast<int>(metadata.cardNo));
        root.insert(QStringLiteral("traceSamplePeriodUs"), metadata.traceSamplePeriodUs);
        root.insert(QStringLiteral("pulsePerDegree"), MotorUnit::kPhysicalPulsesPerDegree);
        root.insert(QStringLiteral("degreesPerCardUnit"), metadata.degreesPerCardUnit);
        root.insert(QStringLiteral("pulsePerCardUnit"),
                    MotorUnit::pulsesPerCardUnit(metadata.degreesPerCardUnit));
        root.insert(QStringLiteral("pulsePerRevolution"), MotorUnit::kPulsesPerRevolution);
        root.insert(QStringLiteral("frameBytes"), static_cast<int>(sizeof(TraceTelemetryFrame)));
        root.insert(QStringLiteral("byteOrder"), QStringLiteral("little-endian"));
        root.insert(QStringLiteral("traceTimeOrigin"), QStringLiteral("first recorded Trace frame"));
        root.insert(QStringLiteral("description"), metadata.description);
        QJsonArray axes;
        for (const quint16 axis : metadata.axes) {
            axes.append(static_cast<int>(axis));
        }
        root.insert(QStringLiteral("traceAxes"), axes);

        QFile metadataFile(QDir(runDirectory_).filePath(QStringLiteral("metadata.json")));
        if (!metadataFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
            || metadataFile.write(QJsonDocument(root).toJson(QJsonDocument::Indented)) < 0) {
            error = QStringLiteral("无法写入 metadata.json：%1").arg(metadataFile.errorString());
            return false;
        }
        metadataFile.close();

        traceFile_.setFileName(QDir(runDirectory_).filePath(QStringLiteral("trace_position.bin")));
        if (!traceFile_.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            error = QStringLiteral("无法打开 trace_position.bin：%1").arg(traceFile_.errorString());
            return false;
        }
        eventFile_.setFileName(QDir(runDirectory_).filePath(QStringLiteral("events.log")));
        if (!eventFile_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            error = QStringLiteral("无法打开 events.log：%1").arg(eventFile_.errorString());
            traceFile_.close();
            return false;
        }
        timingFile_.setFileName(
            QDir(runDirectory_).filePath(QStringLiteral("control_cycle_timing.csv")));
        if (!timingFile_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            error = QStringLiteral("无法打开 control_cycle_timing.csv：%1")
                        .arg(timingFile_.errorString());
            eventFile_.close();
            traceFile_.close();
            return false;
        }
        timingFile_.write(
            "run_id,cycle_index,host_elapsed_us,target_period_us,"
            "scheduling_interval_us,trace_poll_us,calculation_us,api_total_us,"
            "full_cycle_us,cable0_axis_api_us,cable1_axis_api_us,"
            "cable2_axis_api_us,cable3_axis_api_us,cable4_axis_api_us,"
            "cable5_axis_api_us,cable6_axis_api_us,cable7_axis_api_us,slowest_axis,"
            "trace_frames_read,estimated_missed_cycles\n");
        writeEvent(QStringLiteral("recording_started"));
        if (timer_ == nullptr) {
            timer_ = new QTimer(this);
            timer_->setTimerType(Qt::PreciseTimer);
            timer_->setInterval(kWriterIntervalMs);
            connect(timer_, &QTimer::timeout, this, [this] { drain(); });
        }
        timer_->start();
        recorder_->setOutputDirectory(runDirectory_);
        return true;
    }

    void finish()
    {
        if (timer_ != nullptr) {
            timer_->stop();
        }
        while (drain() > 0) {
        }
        if (eventFile_.isOpen()) {
            writeEvent(QStringLiteral("recording_stopped"));
            eventFile_.flush();
            eventFile_.close();
        }
        if (traceFile_.isOpen()) {
            traceFile_.flush();
            traceFile_.close();
        }
        if (timingFile_.isOpen()) {
            timingFile_.flush();
            timingFile_.close();
        }
        runDirectory_.clear();
    }

    int drain()
    {
        return drainTraceFrames() + drainTimingSamples();
    }

    int drainTraceFrames()
    {
        if (!traceFile_.isOpen()) {
            return 0;
        }
        batch_.clear();
        const int count = recorder_->takeBatch(batch_, kWriterBatchFrames);
        if (count <= 0) {
            return 0;
        }
        const qint64 bytes = static_cast<qint64>(count) * static_cast<qint64>(sizeof(TraceTelemetryFrame));
        const qint64 written = traceFile_.write(reinterpret_cast<const char *>(batch_.constData()), bytes);
        if (written != bytes) {
            recorder_->setWriterError(QStringLiteral("写入 trace_position.bin 失败：%1").arg(traceFile_.errorString()));
            return count;
        }
        recorder_->addWrittenFrames(static_cast<quint64>(count));
        return count;
    }

    int drainTimingSamples()
    {
        if (!timingFile_.isOpen()) {
            return 0;
        }
        timingBatch_.clear();
        const int count = recorder_->takeTimingBatch(
            timingBatch_, kWriterBatchTimingSamples);
        if (count <= 0) {
            return 0;
        }
        QByteArray output;
        output.reserve(count * 220);
        for (const ControlCycleTimingSample &sample : std::as_const(timingBatch_)) {
            output.append(QByteArray::number(sample.runId));
            output.append(',');
            output.append(QByteArray::number(sample.cycleIndex));
            output.append(',');
            output.append(QByteArray::number(sample.hostElapsedUs));
            output.append(',');
            output.append(QByteArray::number(sample.targetPeriodUs));
            output.append(',');
            output.append(QByteArray::number(sample.schedulingIntervalUs));
            output.append(',');
            output.append(QByteArray::number(sample.tracePollUs));
            output.append(',');
            output.append(QByteArray::number(sample.calculationUs));
            output.append(',');
            output.append(QByteArray::number(sample.apiTotalUs));
            output.append(',');
            output.append(QByteArray::number(sample.fullCycleUs));
            for (const quint32 axisUs : sample.axisApiUs) {
                output.append(',');
                output.append(QByteArray::number(axisUs));
            }
            output.append(',');
            output.append(QByteArray::number(sample.slowestAxis));
            output.append(',');
            output.append(QByteArray::number(sample.traceFramesRead));
            output.append(',');
            output.append(QByteArray::number(sample.estimatedMissedCycles));
            output.append('\n');
        }
        const qint64 written = timingFile_.write(output);
        if (written != output.size()) {
            recorder_->setWriterError(
                QStringLiteral("写入 control_cycle_timing.csv 失败：%1")
                    .arg(timingFile_.errorString()));
            return count;
        }
        recorder_->addWrittenTimingSamples(static_cast<quint64>(count));
        return count;
    }

    void appendEvent(const QString &eventText)
    {
        if (eventFile_.isOpen()) {
            writeEvent(eventText);
        }
    }

private:
    void writeEvent(const QString &event)
    {
        eventFile_.write(QDateTime::currentDateTime().toString(Qt::ISODateWithMs).toUtf8());
        eventFile_.write(" ");
        eventFile_.write(event.toUtf8());
        eventFile_.write("\n");
    }

    TelemetryRecorder *recorder_ = nullptr;
    QTimer *timer_ = nullptr;
    QFile traceFile_;
    QFile eventFile_;
    QFile timingFile_;
    QString runDirectory_;
    QVector<TraceTelemetryFrame> batch_;
    QVector<ControlCycleTimingSample> timingBatch_;
};

TelemetryRecorder::TelemetryRecorder()
    : ring_(static_cast<qsizetype>(kRingCapacity))
    , timingRing_(static_cast<qsizetype>(kTimingRingCapacity))
    , writerThread_(new QThread)
    , writer_(new TelemetryWriterWorker(this))
{
    writer_->moveToThread(writerThread_);
    writerThread_->start();
}

TelemetryRecorder::~TelemetryRecorder()
{
    stop();
    QMetaObject::invokeMethod(writer_, [writer = writer_] { delete writer; }, Qt::BlockingQueuedConnection);
    writer_ = nullptr;
    writerThread_->quit();
    writerThread_->wait();
    delete writerThread_;
}

bool TelemetryRecorder::start(const TelemetryRunMetadata &metadata, QString &errorMessage)
{
    if (recording_.load(std::memory_order_acquire)) {
        errorMessage = QStringLiteral("当前已在记录，请先停止记录。");
        return false;
    }
    writeIndex_.store(0, std::memory_order_release);
    readIndex_.store(0, std::memory_order_release);
    writtenFrames_.store(0, std::memory_order_release);
    droppedFrames_.store(0, std::memory_order_release);
    timingWriteIndex_.store(0, std::memory_order_release);
    timingReadIndex_.store(0, std::memory_order_release);
    writtenTimingSamples_.store(0, std::memory_order_release);
    droppedTimingSamples_.store(0, std::memory_order_release);
    firstRecordedTraceTimeUs_ = 0;
    hasFirstRecordedTraceTime_ = false;
    {
        QMutexLocker locker(&statusMutex_);
        outputDirectory_.clear();
        errorText_.clear();
    }
    bool writerOpened = false;
    const bool invoked = QMetaObject::invokeMethod(writer_, [&] {
        writerOpened = writer_->begin(metadata, errorMessage);
    }, Qt::BlockingQueuedConnection);
    if (!invoked || !writerOpened) {
        return false;
    }
    recording_.store(true, std::memory_order_release);
    return true;
}

void TelemetryRecorder::stop()
{
    if (!recording_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    QMetaObject::invokeMethod(writer_, [this] { writer_->finish(); }, Qt::BlockingQueuedConnection);
}

void TelemetryRecorder::pushFrames(const QVector<TraceTelemetryFrame> &frames)
{
    if (!recording_.load(std::memory_order_acquire)) {
        return;
    }
    for (const TraceTelemetryFrame &frame : frames) {
        TraceTelemetryFrame recorded = frame;
        if (!hasFirstRecordedTraceTime_) {
            firstRecordedTraceTimeUs_ = recorded.traceTimeUs;
            hasFirstRecordedTraceTime_ = true;
        }
        recorded.traceTimeUs -= firstRecordedTraceTimeUs_;
        const quint32 write = writeIndex_.load(std::memory_order_relaxed);
        const quint32 next = (write + 1U) % kRingCapacity;
        if (next == readIndex_.load(std::memory_order_acquire)) {
            droppedFrames_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        ring_[static_cast<qsizetype>(write)] = recorded;
        writeIndex_.store(next, std::memory_order_release);
    }
}

void TelemetryRecorder::pushControlCycleTiming(
    const ControlCycleTimingSample &sample)
{
    if (!recording_.load(std::memory_order_acquire)) {
        return;
    }
    const quint32 write = timingWriteIndex_.load(std::memory_order_relaxed);
    const quint32 next = (write + 1U) % kTimingRingCapacity;
    if (next == timingReadIndex_.load(std::memory_order_acquire)) {
        droppedTimingSamples_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    timingRing_[static_cast<qsizetype>(write)] = sample;
    timingWriteIndex_.store(next, std::memory_order_release);
}

void TelemetryRecorder::appendEvent(const QString &eventText)
{
    if (!recording_.load(std::memory_order_acquire)) {
        return;
    }
    QMetaObject::invokeMethod(writer_, [this, eventText] {
        writer_->appendEvent(eventText);
    }, Qt::QueuedConnection);
}

TelemetryRecorderStatus TelemetryRecorder::status() const
{
    TelemetryRecorderStatus result;
    result.recording = recording_.load(std::memory_order_acquire);
    const quint32 write = writeIndex_.load(std::memory_order_acquire);
    const quint32 read = readIndex_.load(std::memory_order_acquire);
    result.queuedFrames = write >= read ? write - read : kRingCapacity - read + write;
    result.writtenFrames = writtenFrames_.load(std::memory_order_acquire);
    result.droppedFrames = droppedFrames_.load(std::memory_order_acquire);
    const quint32 timingWrite = timingWriteIndex_.load(std::memory_order_acquire);
    const quint32 timingRead = timingReadIndex_.load(std::memory_order_acquire);
    result.queuedTimingSamples = timingWrite >= timingRead
        ? timingWrite - timingRead
        : kTimingRingCapacity - timingRead + timingWrite;
    result.writtenTimingSamples =
        writtenTimingSamples_.load(std::memory_order_acquire);
    result.droppedTimingSamples =
        droppedTimingSamples_.load(std::memory_order_acquire);
    QMutexLocker locker(&statusMutex_);
    result.outputDirectory = outputDirectory_;
    result.errorText = errorText_;
    return result;
}

int TelemetryRecorder::takeBatch(QVector<TraceTelemetryFrame> &batch, int maximum)
{
    const int limit = std::max(1, maximum);
    batch.reserve(limit);
    quint32 read = readIndex_.load(std::memory_order_relaxed);
    const quint32 write = writeIndex_.load(std::memory_order_acquire);
    while (read != write && batch.size() < limit) {
        batch.push_back(ring_[static_cast<qsizetype>(read)]);
        read = (read + 1U) % kRingCapacity;
    }
    readIndex_.store(read, std::memory_order_release);
    return batch.size();
}

int TelemetryRecorder::takeTimingBatch(
    QVector<ControlCycleTimingSample> &batch, int maximum)
{
    const int limit = std::max(1, maximum);
    batch.reserve(limit);
    quint32 read = timingReadIndex_.load(std::memory_order_relaxed);
    const quint32 write = timingWriteIndex_.load(std::memory_order_acquire);
    while (read != write && batch.size() < limit) {
        batch.push_back(timingRing_[static_cast<qsizetype>(read)]);
        read = (read + 1U) % kTimingRingCapacity;
    }
    timingReadIndex_.store(read, std::memory_order_release);
    return batch.size();
}

void TelemetryRecorder::setWriterError(const QString &errorText)
{
    QMutexLocker locker(&statusMutex_);
    errorText_ = errorText;
}

void TelemetryRecorder::setOutputDirectory(const QString &directory)
{
    QMutexLocker locker(&statusMutex_);
    outputDirectory_ = directory;
}

void TelemetryRecorder::addWrittenFrames(quint64 count)
{
    writtenFrames_.fetch_add(count, std::memory_order_relaxed);
}

void TelemetryRecorder::addWrittenTimingSamples(quint64 count)
{
    writtenTimingSamples_.fetch_add(count, std::memory_order_relaxed);
}
