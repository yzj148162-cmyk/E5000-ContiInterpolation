#ifndef RUNTIME_TRACE_SLAVE_READER_H
#define RUNTIME_TRACE_SLAVE_READER_H

#include <deque>
#include <vector>

class RuntimeTraceSlaveReader
{
public:
    enum class SignMode {
        SignedValue = 0,
        Low16AsSigned
    };

    struct ObjectConfig {
        int logicalIndex = -1;
        short dataType = 19;
        int dataIndex = 0x6000;
        int dataSubIndex = 0;
        short slaveId = 0;
        short apiDataBytes = 0;
        int valueBytes = 2;
        double home = 0.0;
        double scale = 1.0;
        SignMode signMode = SignMode::SignedValue;
    };

    struct ReaderConfig {
        unsigned short cardNo = 0;
        int samplePeriodUs = 500;
        int traceBaseCycleUs = 500;
        short lostHandle = 0;
        short traceType = 0;
        short triggerObjectIndex = 0;
        short triggerType = 0;
        int mask = 0;
        long long condition = 0;
        int maxBufferBytes = 64 * 1024;
        int maxDrainReads = 16;
        int maxQueuedSamples = 8192;
        int retryIntervalUs = 1000 * 1000;
        std::vector<ObjectConfig> objects;
    };

    struct Sample {
        std::vector<double> values;
    };

    RuntimeTraceSlaveReader() = default;
    explicit RuntimeTraceSlaveReader(const ReaderConfig& config);
    ~RuntimeTraceSlaveReader();

    void setConfig(const ReaderConfig& config);
    bool configure();
    void reset();

    int readTraceCached();
    std::vector<double> currentValues() const;
    std::vector<Sample> takeSamples();

    bool isConfigured() const { return traceConfigured; }
    bool hasEverRead() const { return traceEverRead; }
    bool isUnavailable() const { return traceUnavailable; }
    short lastApiResult() const { return lastRet; }

private:
    struct FrameLayout {
        int frameBytes = 0;
        int valueStartOffset = 0;
        int objectStepBytes = 0;
    };

    bool configureTrace();
    void ensureStorage();
    FrameLayout resolveFrameLayout(int objectTotalBytes) const;
    void applyRawValue(int objectIndex, long rawValue);
    int outputIndexForObject(int objectIndex) const;

    ReaderConfig cfg;
    bool traceConfigured = false;
    bool traceUnavailable = false;
    bool traceEverRead = false;
    int consecutiveFailures = 0;
    long long lastRetryUs = 0;
    int lastObjectTotalBytes = 0;
    int lastObjectTotalNum = 0;
    short lastRet = 0;
    std::vector<double> cachedValues;
    std::vector<bool> cacheValid;
    std::deque<Sample> sampleQueue;
};

#endif // RUNTIME_TRACE_SLAVE_READER_H
