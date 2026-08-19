#include "telemetry/TelemetryRecorder.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QSaveFile>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <utility>

namespace {
constexpr int kWriterIntervalMs = 100;
constexpr int kWriterBatchFrames = 4096;
constexpr int kWriterBatchTimingSamples = 4096;
constexpr int kWriterBatchForceControlSamples = 1024;

template <size_t Size>
void appendCsvArray(QByteArray &output, const std::array<double, Size> &values)
{
    for (double value : values) {
        output.append(',');
        output.append(QByteArray::number(value, 'g', 17));
    }
}

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
        runContext_ = {};
        runContextDirty_ = false;

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
        QJsonArray traceDataTypes;
        QString traceProfileText;
        if (metadata.traceProfile == TraceFeedbackProfile::VelocityControl) {
            traceProfileText = QStringLiteral("velocity_control_3_4_6");
            traceDataTypes = QJsonArray {3, 4, 6};
        } else if (metadata.traceProfile == TraceFeedbackProfile::PositionControl) {
            traceProfileText = QStringLiteral("position_control_5_6");
            traceDataTypes = QJsonArray {5, 6};
        } else {
            traceProfileText = QStringLiteral("full_3_4_5_6");
            traceDataTypes = QJsonArray {3, 4, 5, 6};
        }
        root.insert(QStringLiteral("traceProfile"), traceProfileText);
        root.insert(QStringLiteral("traceDataTypes"), traceDataTypes);
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
        forceControlFile_.setFileName(
            QDir(runDirectory_).filePath(QStringLiteral("cdpr_force_control.csv")));
        if (!forceControlFile_.open(QIODevice::WriteOnly | QIODevice::Text
                                    | QIODevice::Truncate)) {
            error = QStringLiteral("无法打开 cdpr_force_control.csv：%1")
                        .arg(forceControlFile_.errorString());
            timingFile_.close();
            eventFile_.close();
            traceFile_.close();
            return false;
        }
        QByteArray forceHeader("run_id,cycle_index,host_elapsed_us,trace_sequence");
        const QList<QByteArray> groups {
            "sensor_wrench", "platform_wrench", "desired_pose", "desired_twist"
        };
        for (const QByteArray &group : groups) {
            for (int index = 0; index < 6; ++index) {
                forceHeader.append(',');
                forceHeader.append(group);
                forceHeader.append(QByteArray::number(index));
            }
        }
        const QList<QByteArray> cableGroups {
            "desired_cable_length_m", "desired_cable_velocity_mps",
            "target_axis_position_deg", "target_axis_velocity_deg_s",
            "actual_axis_position_deg", "actual_axis_velocity_deg_s"
        };
        for (const QByteArray &group : cableGroups) {
            for (int index = 0; index < 8; ++index) {
                forceHeader.append(',');
                forceHeader.append(group);
                forceHeader.append(QByteArray::number(index));
            }
        }
        forceHeader.append(",maximum_tracking_error_deg,maximum_cable_travel_m\n");
        forceControlFile_.write(forceHeader);
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
        writeRunContextIfDirty();
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
        if (forceControlFile_.isOpen()) {
            forceControlFile_.flush();
            forceControlFile_.close();
        }
        runDirectory_.clear();
    }

    int drain()
    {
        const int drained = drainTraceFrames() + drainTimingSamples()
            + drainForceControlSamples();
        writeRunContextIfDirty();
        return drained;
    }

    bool initializeRunContext(const QJsonObject &context, QString &error)
    {
        runContext_ = context;
        runContextDirty_ = true;
        return writeRunContextIfDirty(&error);
    }

    void updateRunContext(const QString &key, const QJsonValue &value)
    {
        if (runDirectory_.isEmpty()) {
            return;
        }
        runContext_.insert(key, value);
        runContextDirty_ = true;
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

    int drainForceControlSamples()
    {
        if (!forceControlFile_.isOpen()) {
            return 0;
        }
        forceControlBatch_.clear();
        const int count = recorder_->takeForceControlBatch(
            forceControlBatch_, kWriterBatchForceControlSamples);
        if (count <= 0) {
            return 0;
        }
        QByteArray output;
        output.reserve(count * 1200);
        for (const CdprForceControlTelemetrySample &sample
             : std::as_const(forceControlBatch_)) {
            output.append(QByteArray::number(sample.runId));
            output.append(',');
            output.append(QByteArray::number(sample.cycleIndex));
            output.append(',');
            output.append(QByteArray::number(sample.hostElapsedUs));
            output.append(',');
            output.append(QByteArray::number(sample.traceSequence));
            appendCsvArray(output, sample.sensorWrench);
            appendCsvArray(output, sample.platformWrench);
            appendCsvArray(output, sample.desiredPose);
            appendCsvArray(output, sample.desiredTwist);
            appendCsvArray(output, sample.desiredCableLengthM);
            appendCsvArray(output, sample.desiredCableVelocityMps);
            appendCsvArray(output, sample.targetAxisPositionDegree);
            appendCsvArray(output, sample.targetAxisVelocityDegreePerSecond);
            appendCsvArray(output, sample.actualAxisPositionDegree);
            appendCsvArray(output, sample.actualAxisVelocityDegreePerSecond);
            output.append(',');
            output.append(QByteArray::number(
                sample.maximumTrackingErrorDegree, 'g', 17));
            output.append(',');
            output.append(QByteArray::number(sample.maximumCableTravelM, 'g', 17));
            output.append('\n');
        }
        if (forceControlFile_.write(output) != output.size()) {
            recorder_->setWriterError(
                QStringLiteral("写入 cdpr_force_control.csv 失败：%1")
                    .arg(forceControlFile_.errorString()));
        } else {
            recorder_->addWrittenForceControlSamples(
                static_cast<quint64>(count));
        }
        return count;
    }

    void appendEvent(const QString &eventText)
    {
        if (eventFile_.isOpen()) {
            writeEvent(eventText);
        }
    }

private:
    bool writeRunContextIfDirty(QString *error = nullptr)
    {
        if (!runContextDirty_ || runDirectory_.isEmpty()) {
            return true;
        }
        QSaveFile file(QDir(runDirectory_).filePath(QStringLiteral("run_context.json")));
        const QByteArray payload = QJsonDocument(runContext_).toJson(QJsonDocument::Indented);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)
            || file.write(payload) != payload.size()
            || !file.commit()) {
            const QString message = QStringLiteral("无法异步写入CDPR运行上下文：%1")
                                        .arg(file.errorString());
            recorder_->setWriterError(message);
            if (error != nullptr) {
                *error = message;
            }
            return false;
        }
        runContextDirty_ = false;
        return true;
    }

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
    QFile forceControlFile_;
    QString runDirectory_;
    QJsonObject runContext_;
    bool runContextDirty_ = false;
    QVector<TraceTelemetryFrame> batch_;
    QVector<ControlCycleTimingSample> timingBatch_;
    QVector<CdprForceControlTelemetrySample> forceControlBatch_;
};

TelemetryRecorder::TelemetryRecorder()
    : ring_(static_cast<qsizetype>(kRingCapacity))
    , timingRing_(static_cast<qsizetype>(kTimingRingCapacity))
    , forceControlRing_(static_cast<qsizetype>(kForceControlRingCapacity))
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
    forceControlWriteIndex_.store(0, std::memory_order_release);
    forceControlReadIndex_.store(0, std::memory_order_release);
    writtenForceControlSamples_.store(0, std::memory_order_release);
    droppedForceControlSamples_.store(0, std::memory_order_release);
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

void TelemetryRecorder::pushCdprForceControlSample(
    const CdprForceControlTelemetrySample &sample)
{
    if (!recording_.load(std::memory_order_acquire)) {
        return;
    }
    const quint32 write = forceControlWriteIndex_.load(std::memory_order_relaxed);
    const quint32 next = (write + 1U) % kForceControlRingCapacity;
    if (next == forceControlReadIndex_.load(std::memory_order_acquire)) {
        droppedForceControlSamples_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    forceControlRing_[static_cast<qsizetype>(write)] = sample;
    forceControlWriteIndex_.store(next, std::memory_order_release);
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

bool TelemetryRecorder::initializeRunContext(const QJsonObject &context,
                                             QString &errorMessage)
{
    if (!recording_.load(std::memory_order_acquire)) {
        errorMessage = QStringLiteral("数据记录尚未启动，无法初始化运行上下文。");
        return false;
    }
    bool written = false;
    const bool invoked = QMetaObject::invokeMethod(writer_, [&] {
        written = writer_->initializeRunContext(context, errorMessage);
    }, Qt::BlockingQueuedConnection);
    return invoked && written;
}

void TelemetryRecorder::updateRunContext(const QString &key, const QJsonValue &value)
{
    if (!recording_.load(std::memory_order_acquire)) {
        return;
    }
    QMetaObject::invokeMethod(writer_, [writer = writer_, key, value] {
        writer->updateRunContext(key, value);
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
    const quint32 forceWrite =
        forceControlWriteIndex_.load(std::memory_order_acquire);
    const quint32 forceRead =
        forceControlReadIndex_.load(std::memory_order_acquire);
    result.queuedForceControlSamples = forceWrite >= forceRead
        ? forceWrite - forceRead
        : kForceControlRingCapacity - forceRead + forceWrite;
    result.writtenForceControlSamples =
        writtenForceControlSamples_.load(std::memory_order_acquire);
    result.droppedForceControlSamples =
        droppedForceControlSamples_.load(std::memory_order_acquire);
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

int TelemetryRecorder::takeForceControlBatch(
    QVector<CdprForceControlTelemetrySample> &batch, int maximum)
{
    const int limit = std::max(1, maximum);
    batch.reserve(limit);
    quint32 read = forceControlReadIndex_.load(std::memory_order_relaxed);
    const quint32 write = forceControlWriteIndex_.load(std::memory_order_acquire);
    while (read != write && batch.size() < limit) {
        batch.push_back(forceControlRing_[static_cast<qsizetype>(read)]);
        read = (read + 1U) % kForceControlRingCapacity;
    }
    forceControlReadIndex_.store(read, std::memory_order_release);
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

void TelemetryRecorder::addWrittenForceControlSamples(quint64 count)
{
    writtenForceControlSamples_.fetch_add(count, std::memory_order_relaxed);
}
