#ifndef RUNTIMETRACESLAVEREADER_H
#define RUNTIMETRACESLAVEREADER_H

#include <deque>
#include <vector>

#include <QElapsedTimer>
#include <QString>
#include <QtGlobal>

// 从 trace_slave_read_extract 迁入：只负责同一张控制卡的 Trace 配置、批量取帧和解码。
class RuntimeTraceSlaveReader
{
public:
    struct ObjectConfig {
        int logicalIndex = -1;
        short dataType = 0;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        // 雷赛手册要求现有 Trace 类型固定传 0，由板卡按 dataType 自动匹配。
        short apiDataBytes = 0;
        int valueBytes = 4;
        double scale = 1.0;
    };
    struct ReaderConfig {
        unsigned short cardNo = 0;
        int samplePeriodUs = 1000;
        int traceBaseCycleUs = 1000;
        int maxBufferBytes = 64 * 1024;
        int maxDrainReads = 16;
        int maxQueuedSamples = 8192;
        std::vector<ObjectConfig> objects;
    };
    struct Sample {
        quint64 sequence = 0;
        std::vector<double> values;
    };

    RuntimeTraceSlaveReader() = default;
    ~RuntimeTraceSlaveReader();

    bool configure(const ReaderConfig &config);
    void reset();
    int readTraceCached();
    std::vector<Sample> takeSamples();
    bool isConfigured() const { return configured_; }
    bool hasEverRead() const { return everRead_; }
    short lastApiResult() const { return lastResult_; }
    QString failureReason() const { return failureReason_; }
    bool timingReliable() const { return timingReliable_; }
    int lastValidFrames() const { return lastValidFrames_; }
    int lastFreeFrames() const { return lastFreeFrames_; }
    int maximumValidFrames() const { return maximumValidFrames_; }
    int minimumFreeFrames() const { return minimumFreeFrames_; }
    quint64 locallyDroppedSamples() const { return locallyDroppedSamples_; }
    int objectTotalBytes() const { return objectTotalBytes_; }
    int objectTotalNum() const { return objectTotalNum_; }
    int frameBytes() const { return frameBytes_; }
    int frameHeaderBytes() const { return frameHeaderBytes_; }
    bool hardwareSequenceAvailable() const { return hardwareSequenceAvailable_; }
    double logicalToHostTimeRatio() const { return logicalToHostTimeRatio_; }
    bool productionRateDiagnosticValid() const { return productionRateDiagnosticValid_; }
    quint64 estimatedGeneratedFrames() const { return estimatedGeneratedFrames_; }
    quint64 sequenceGapFrames() const { return sequenceGapFrames_; }
    int lastReadBytes() const { return lastReadBytes_; }
    int lastReadFrames() const { return lastReadFrames_; }
    quint64 lastReadFirstSequence() const { return lastReadFirstSequence_; }
    quint64 lastReadLastSequence() const { return lastReadLastSequence_; }
    int configReadbackMismatchCount() const { return configReadbackMismatchCount_; }

private:
    struct FrameLayout {
        int frameBytes = 0;
        int valueStart = 0;
        bool hasSequenceHeader = false;
    };
    FrameLayout resolveLayout(int objectTotalBytes, int objectTotalNum) const;
    quint64 decodeSequence(const unsigned char *frameData,
                           bool hasSequenceHeader);
    void updateProductionRateDiagnostic(int currentValidFrames);
    int outputIndex(int objectIndex) const;
    void ensureValues();
    void updateBufferDiagnostics(int validFrames, int freeFrames);

    ReaderConfig config_;
    bool configured_ = false;
    bool everRead_ = false;
    bool timingReliable_ = true;
    short lastResult_ = 0;
    QString failureReason_;
    int lastValidFrames_ = 0;
    int lastFreeFrames_ = 0;
    int maximumValidFrames_ = 0;
    int minimumFreeFrames_ = -1;
    quint64 locallyDroppedSamples_ = 0;
    int objectTotalBytes_ = 0;
    int objectTotalNum_ = 0;
    int frameBytes_ = 0;
    int frameHeaderBytes_ = 0;
    bool hardwareSequenceAvailable_ = false;
    bool hardwareSequenceInitialized_ = false;
    quint32 firstHardwareSequence_ = 0;
    quint32 lastHardwareSequence_ = 0;
    quint64 firstLogicalSequence_ = 0;
    quint64 lastLogicalSequence_ = 0;
    double logicalToHostTimeRatio_ = 1.0;
    bool productionRateDiagnosticValid_ = false;
    qint64 productionBalanceBaseline_ = -1;
    quint64 cumulativeConsumedFrames_ = 0;
    quint64 estimatedGeneratedFrames_ = 0;
    quint64 sequenceGapFrames_ = 0;
    int lastReadBytes_ = 0;
    int lastReadFrames_ = 0;
    quint64 lastReadFirstSequence_ = 0;
    quint64 lastReadLastSequence_ = 0;
    int configReadbackMismatchCount_ = 0;
    QElapsedTimer productionRateClock_;
    std::vector<double> values_;
    std::deque<Sample> samples_;
    quint64 nextSequence_ = 1;
};

#endif // RUNTIMETRACESLAVEREADER_H
