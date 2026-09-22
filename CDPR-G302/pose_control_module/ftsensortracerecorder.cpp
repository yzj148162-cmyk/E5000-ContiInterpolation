#include "ftsensortracerecorder.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QStringConverter>
#include <QTextStream>

FtSensorTraceRecorder::FtSensorTraceRecorder(QObject* parent)
    : QThread(parent)
{
    setObjectName(QStringLiteral("FtSensorTraceRecorder"));
}

FtSensorTraceRecorder::~FtSensorTraceRecorder()
{
    finishAndWait();
}

bool FtSensorTraceRecorder::begin(const QString& directory,
                                  QString* outputPath,
                                  QString* errorMessage)
{
    finishAndWait();
    const QString resolved = directory.trimmed().isEmpty() ?
                QDir(QCoreApplication::applicationDirPath()).filePath(
                    QStringLiteral("data/outputmsg/force_interaction_runs")) :
                directory;
    if(!QDir().mkpath(resolved)){
        if(errorMessage) *errorMessage = QStringLiteral("无法创建F/T记录目录：%1").arg(resolved);
        return false;
    }
    outputPath_ = QDir(resolved).filePath(
                QStringLiteral("ft_preheat_%1.csv").arg(
                    QDateTime::currentDateTime().toString(
                        QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    {
        QMutexLocker locker(&mutex_);
        queue_.clear();
        openError_.clear();
    }
    stopRequested_.store(false);
    openSucceeded_.store(false);
    accepted_.store(0);
    written_.store(0);
    dropped_.store(0);
    while(opened_.tryAcquire(1)){
    }
    start(QThread::LowPriority);
    if(!opened_.tryAcquire(1, 1500) || !openSucceeded_.load()){
        finishAndWait();
        if(errorMessage){
            QMutexLocker locker(&mutex_);
            *errorMessage = openError_.isEmpty() ?
                        QStringLiteral("F/T记录线程启动失败") : openError_;
        }
        return false;
    }
    if(outputPath) *outputPath = outputPath_;
    return true;
}

void FtSensorTraceRecorder::tryAppend(
        const std::vector<FtSensorTraceSample>& samples)
{
    if(samples.empty() || !isRunning() || stopRequested_.load()){
        return;
    }
    if(!mutex_.tryLock()){
        dropped_.fetch_add(samples.size());
        return;
    }
    for(const FtSensorTraceSample& sample : samples){
        if(queue_.size() >= kMaximumQueuedSamples){
            dropped_.fetch_add(1);
            continue;
        }
        queue_.push_back(sample);
        accepted_.fetch_add(1);
    }
    mutex_.unlock();
    ready_.wakeOne();
}

void FtSensorTraceRecorder::finishAndWait()
{
    if(!isRunning()){
        return;
    }
    stopRequested_.store(true);
    ready_.wakeOne();
    wait();
}

QString FtSensorTraceRecorder::outputPath() const { return outputPath_; }
quint64 FtSensorTraceRecorder::acceptedCount() const { return accepted_.load(); }
quint64 FtSensorTraceRecorder::writtenCount() const { return written_.load(); }
quint64 FtSensorTraceRecorder::droppedCount() const { return dropped_.load(); }

void FtSensorTraceRecorder::run()
{
    QFile file(outputPath_);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        {
            QMutexLocker locker(&mutex_);
            openError_ = file.errorString();
        }
        openSucceeded_.store(false);
        opened_.release();
        return;
    }
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream << "wall_clock_us,monotonic_us,trace_frame,trace_frame_valid,"
              "sensor_counter,sensor_counter_valid,status_code,status_valid,"
              "temperature_raw,temperature_c,temperature_valid,"
              "fx_raw,fy_raw,fz_raw,mx_raw,my_raw,mz_raw,"
              "fx_n,fy_n,fz_n,mx_nm,my_nm,mz_nm,wrench_complete\n";
    stream.flush();
    openSucceeded_.store(true);
    opened_.release();

    while(true){
        std::deque<FtSensorTraceSample> batch;
        {
            QMutexLocker locker(&mutex_);
            if(queue_.empty() && !stopRequested_.load()){
                ready_.wait(&mutex_, 250);
            }
            batch.swap(queue_);
        }
        for(const FtSensorTraceSample& sample : batch){
            stream << sample.wallClockUs << ',' << sample.monotonicUs << ','
                   << sample.traceFrameSequence << ',' << sample.traceFrameSequenceValid << ','
                   << sample.sampleCounter << ',' << sample.sampleCounterValid << ','
                   << sample.statusCode << ',' << sample.statusValid << ','
                   << sample.temperatureRaw << ',' << sample.temperatureC << ','
                   << sample.temperatureValid;
            for(qint32 raw : sample.raw) stream << ',' << raw;
            for(double value : sample.value) stream << ',' << value;
            stream << ',' << sample.wrenchComplete() << '\n';
            written_.fetch_add(1);
        }
        if(!batch.empty()) stream.flush();
        if(stopRequested_.load()){
            QMutexLocker locker(&mutex_);
            if(queue_.empty()) break;
        }
    }
    stream.flush();
    file.close();
}
