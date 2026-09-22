#ifndef FTSENSORTRACERECORDER_H
#define FTSENSORTRACERECORDER_H

#include "ftsensortypes.h"

#include <atomic>
#include <deque>
#include <vector>

#include <QMutex>
#include <QSemaphore>
#include <QString>
#include <QThread>
#include <QWaitCondition>

// F/T预热专用有界异步记录器。GUI/Trace读取路径只尝试复制一批定长样本，
// 写盘落后时丢弃并计数，不允许磁盘等待反向阻塞Trace读取。
class FtSensorTraceRecorder final : public QThread
{
public:
    explicit FtSensorTraceRecorder(QObject* parent = nullptr);
    ~FtSensorTraceRecorder() override;

    bool begin(const QString& directory,
               QString* outputPath = nullptr,
               QString* errorMessage = nullptr);
    void tryAppend(const std::vector<FtSensorTraceSample>& samples);
    void finishAndWait();
    QString outputPath() const;
    quint64 acceptedCount() const;
    quint64 writtenCount() const;
    quint64 droppedCount() const;

protected:
    void run() override;

private:
    static constexpr std::size_t kMaximumQueuedSamples = 65536;
    QString outputPath_;
    QString openError_;
    mutable QMutex mutex_;
    QWaitCondition ready_;
    QSemaphore opened_;
    std::deque<FtSensorTraceSample> queue_;
    std::atomic_bool stopRequested_{false};
    std::atomic_bool openSucceeded_{false};
    std::atomic<quint64> accepted_{0};
    std::atomic<quint64> written_{0};
    std::atomic<quint64> dropped_{0};
};

#endif // FTSENSORTRACERECORDER_H
