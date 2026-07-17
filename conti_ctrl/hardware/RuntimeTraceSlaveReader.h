#ifndef RUNTIMETRACESLAVEREADER_H
#define RUNTIMETRACESLAVEREADER_H

#include <deque>
#include <vector>

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
        short apiDataBytes = 4;
        int valueBytes = 4;
        double scale = 1.0;
    };
    struct ReaderConfig {
        unsigned short cardNo = 0;
        int samplePeriodUs = 500;
        int traceBaseCycleUs = 500;
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

private:
    struct FrameLayout { int frameBytes = 0; int valueStart = 0; int objectStep = 0; };
    FrameLayout resolveLayout(int objectTotalBytes) const;
    int outputIndex(int objectIndex) const;
    void ensureValues();

    ReaderConfig config_;
    bool configured_ = false;
    bool everRead_ = false;
    short lastResult_ = 0;
    std::vector<double> values_;
    std::deque<Sample> samples_;
    quint64 nextSequence_ = 1;
};

#endif // RUNTIMETRACESLAVEREADER_H
