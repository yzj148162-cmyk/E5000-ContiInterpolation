#include "runtime_trace_slave_reader.h"

#include "LTDMC.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <thread>
#include <utility>

namespace {

long long monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

long readSignedLittleEndianTraceValue(const unsigned char* raw, int valueBytes)
{
    if(valueBytes >= 4){
        const std::uint32_t u =
                static_cast<std::uint32_t>(raw[0]) |
                (static_cast<std::uint32_t>(raw[1]) << 8) |
                (static_cast<std::uint32_t>(raw[2]) << 16) |
                (static_cast<std::uint32_t>(raw[3]) << 24);
        return static_cast<long>(static_cast<std::int32_t>(u));
    }
    if(valueBytes == 2){
        const std::uint16_t u =
                static_cast<std::uint16_t>(raw[0]) |
                (static_cast<std::uint16_t>(raw[1]) << 8);
        return static_cast<long>(static_cast<std::int16_t>(u));
    }
    return static_cast<long>(static_cast<std::int8_t>(raw[0]));
}

double normalizeRawValue(long rawValue, RuntimeTraceSlaveReader::SignMode signMode)
{
    if(signMode == RuntimeTraceSlaveReader::SignMode::Low16AsSigned){
        return static_cast<double>(static_cast<std::int16_t>(rawValue & 0xFFFF));
    }
    return static_cast<double>(rawValue);
}

} // namespace

RuntimeTraceSlaveReader::RuntimeTraceSlaveReader(const ReaderConfig& config)
{
    setConfig(config);
}

RuntimeTraceSlaveReader::~RuntimeTraceSlaveReader()
{
    reset();
}

void RuntimeTraceSlaveReader::setConfig(const ReaderConfig& config)
{
    reset();
    cfg = config;
    if(cfg.traceBaseCycleUs <= 0){
        cfg.traceBaseCycleUs = 500;
    }
    if(cfg.samplePeriodUs <= 0){
        cfg.samplePeriodUs = cfg.traceBaseCycleUs;
    }
    if(cfg.maxBufferBytes <= 0){
        cfg.maxBufferBytes = 64 * 1024;
    }
    if(cfg.maxDrainReads <= 0){
        cfg.maxDrainReads = 16;
    }
    if(cfg.maxQueuedSamples <= 0){
        cfg.maxQueuedSamples = 8192;
    }
    if(cfg.retryIntervalUs <= 0){
        cfg.retryIntervalUs = 1000 * 1000;
    }
    ensureStorage();
}

bool RuntimeTraceSlaveReader::configure()
{
    return configureTrace();
}

void RuntimeTraceSlaveReader::reset()
{
    if(traceConfigured){
        dmc_trace_data_stop(static_cast<WORD>(cfg.cardNo));
    }
    traceConfigured = false;
    traceUnavailable = false;
    traceEverRead = false;
    consecutiveFailures = 0;
    lastRetryUs = 0;
    lastObjectTotalBytes = 0;
    lastObjectTotalNum = 0;
    lastRet = 0;
    cachedValues.clear();
    cacheValid.clear();
    sampleQueue.clear();
}

bool RuntimeTraceSlaveReader::configureTrace()
{
    if(traceUnavailable || cfg.objects.empty()){
        return false;
    }

    short ret = dmc_trace_data_stop(static_cast<WORD>(cfg.cardNo));
    lastRet = ret;
    ret = dmc_trace_data_reset(static_cast<WORD>(cfg.cardNo));
    lastRet = ret;
    if(ret != 0){
        traceUnavailable = true;
        return false;
    }

    const int traceCycle = std::max(1, cfg.samplePeriodUs / cfg.traceBaseCycleUs);
    ret = dmc_trace_set_config(static_cast<WORD>(cfg.cardNo),
                               static_cast<short>(traceCycle),
                               cfg.lostHandle,
                               cfg.traceType,
                               cfg.triggerObjectIndex,
                               cfg.triggerType,
                               cfg.mask,
                               cfg.condition);
    lastRet = ret;
    if(ret != 0){
        traceUnavailable = true;
        return false;
    }

    ret = dmc_trace_reset_config_object(static_cast<WORD>(cfg.cardNo));
    lastRet = ret;
    if(ret != 0){
        traceUnavailable = true;
        return false;
    }

    for(const ObjectConfig& object : cfg.objects){
        ret = dmc_trace_add_config_object(static_cast<WORD>(cfg.cardNo),
                                          object.dataType,
                                          object.dataIndex,
                                          object.dataSubIndex,
                                          object.slaveId,
                                          object.apiDataBytes);
        lastRet = ret;
        if(ret != 0){
            dmc_trace_data_stop(static_cast<WORD>(cfg.cardNo));
            traceUnavailable = true;
            return false;
        }
    }

    ret = dmc_trace_data_start(static_cast<WORD>(cfg.cardNo));
    lastRet = ret;
    if(ret != 0){
        traceUnavailable = true;
        return false;
    }

    traceConfigured = true;
    traceEverRead = false;
    consecutiveFailures = 0;
    lastRetryUs = 0;
    return true;
}

void RuntimeTraceSlaveReader::ensureStorage()
{
    int outputSize = 0;
    for(int i = 0; i < static_cast<int>(cfg.objects.size()); ++i){
        outputSize = std::max(outputSize, outputIndexForObject(i) + 1);
    }
    if(outputSize < 0){
        outputSize = 0;
    }
    if(static_cast<int>(cachedValues.size()) != outputSize){
        cachedValues.assign(static_cast<std::size_t>(outputSize), 0.0);
        cacheValid.assign(static_cast<std::size_t>(outputSize), false);
    }
}

RuntimeTraceSlaveReader::FrameLayout
RuntimeTraceSlaveReader::resolveFrameLayout(int objectTotalBytes) const
{
    FrameLayout layout;

    int expectedValueBytes = 0;
    int maxValueBytes = 0;
    for(const ObjectConfig& object : cfg.objects){
        const int valueBytes = std::max(1, object.valueBytes);
        expectedValueBytes += valueBytes;
        maxValueBytes = std::max(maxValueBytes, valueBytes);
    }
    layout.frameBytes = std::max(expectedValueBytes, objectTotalBytes);
    if(layout.frameBytes <= 0){
        return layout;
    }

    const int objectCount = static_cast<int>(cfg.objects.size());
    if(objectTotalBytes >= expectedValueBytes && objectCount > 0){
        const int candidateSlotBytes[] = {8, 4, 2, 1};
        int bestHeaderBytes = std::numeric_limits<int>::max();
        int bestSlotBytes = 0;

        for(const int slotBytes : candidateSlotBytes){
            if(slotBytes < maxValueBytes){
                continue;
            }
            const int objectBytes = slotBytes * objectCount;
            if(objectBytes > objectTotalBytes){
                continue;
            }
            const int headerBytes = objectTotalBytes - objectBytes;
            if(headerBytes < bestHeaderBytes){
                bestHeaderBytes = headerBytes;
                bestSlotBytes = slotBytes;
            }
        }

        if(bestSlotBytes > 0){
            layout.valueStartOffset = bestHeaderBytes;
            layout.objectStepBytes = bestSlotBytes;
            return layout;
        }

        layout.valueStartOffset = objectTotalBytes - expectedValueBytes;
    }

    return layout;
}

int RuntimeTraceSlaveReader::readTraceCached()
{
    ensureStorage();

    if(traceUnavailable){
        const long long nowUs = monotonicNowUs();
        if(lastRetryUs > 0 && nowUs - lastRetryUs < cfg.retryIntervalUs){
            return traceEverRead ? 0 : -1;
        }
        lastRetryUs = nowUs;
        traceUnavailable = false;
        traceConfigured = false;
        consecutiveFailures = 0;
    }

    const bool configureNow = !traceConfigured;
    if(configureNow && !configureTrace()){
        return traceEverRead ? 0 : -1;
    }
    if(configureNow){
        const int sleepMs = std::max(2, cfg.samplePeriodUs / 1000 + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }

    int validNum = 0;
    int freeNum = 0;
    int objectTotalBytes = 0;
    int objectTotalNum = 0;
    short ret = dmc_trace_get_state(static_cast<WORD>(cfg.cardNo),
                                    &validNum,
                                    &freeNum,
                                    &objectTotalBytes,
                                    &objectTotalNum);
    (void)freeNum;
    lastRet = ret;
    if(ret != 0){
        ++consecutiveFailures;
        if(consecutiveFailures >= 5){
            reset();
            traceUnavailable = true;
            return -1;
        }
        return traceEverRead ? 0 : -1;
    }
    lastObjectTotalBytes = objectTotalBytes;
    lastObjectTotalNum = objectTotalNum;

    if(validNum <= 0){
        ++consecutiveFailures;
        return traceEverRead ? 0 : -1;
    }
    consecutiveFailures = 0;

    const FrameLayout layout = resolveFrameLayout(objectTotalBytes);
    if(layout.frameBytes <= 0){
        return traceEverRead ? 0 : -1;
    }

    int expectedValueBytes = 0;
    for(const ObjectConfig& object : cfg.objects){
        expectedValueBytes += std::max(1, object.valueBytes);
    }

    int totalCompleteFrameCount = 0;
    int currentValidNum = validNum;
    for(int drainIndex = 0;
        drainIndex < cfg.maxDrainReads && currentValidNum > 0;
        ++drainIndex){
        const int frameCapacity = std::max(1, cfg.maxBufferBytes / layout.frameBytes);
        const int frameCountToRead = std::max(1, std::min(currentValidNum, frameCapacity));
        const int bufferSize = std::max(layout.frameBytes,
                                        layout.frameBytes * frameCountToRead);
        std::vector<unsigned char> buffer(static_cast<std::size_t>(bufferSize), 0);

        int actualReadLength = 0;
        ret = dmc_trace_get_data(static_cast<WORD>(cfg.cardNo),
                                 bufferSize,
                                 buffer.data(),
                                 &actualReadLength);
        lastRet = ret;
        if(ret != 0 || actualReadLength < expectedValueBytes){
            return traceEverRead || totalCompleteFrameCount > 0 ?
                        totalCompleteFrameCount :
                        -1;
        }

        const int completeFrameCount = actualReadLength / layout.frameBytes;
        if(completeFrameCount <= 0){
            return traceEverRead || totalCompleteFrameCount > 0 ?
                        totalCompleteFrameCount :
                        -1;
        }

        for(int frameIndex = 0; frameIndex < completeFrameCount; ++frameIndex){
            const int frameOffset = frameIndex * layout.frameBytes;
            if(frameOffset + layout.valueStartOffset + expectedValueBytes > actualReadLength){
                return traceEverRead || totalCompleteFrameCount > 0 ?
                            totalCompleteFrameCount :
                            -1;
            }

            int objectOffset = layout.valueStartOffset;
            for(int objectIndex = 0; objectIndex < static_cast<int>(cfg.objects.size()); ++objectIndex){
                const ObjectConfig& object = cfg.objects[objectIndex];
                const int valueBytes = std::max(1, object.valueBytes);
                if(objectOffset + valueBytes > layout.frameBytes ||
                        frameOffset + objectOffset + valueBytes > actualReadLength){
                    return traceEverRead || totalCompleteFrameCount > 0 ?
                                totalCompleteFrameCount :
                                -1;
                }

                const unsigned char* raw = buffer.data() + frameOffset + objectOffset;
                applyRawValue(objectIndex,
                              readSignedLittleEndianTraceValue(raw, valueBytes));
                objectOffset += layout.objectStepBytes >= valueBytes ?
                            layout.objectStepBytes :
                            valueBytes;
            }

            Sample sample;
            sample.values = currentValues();
            if(!sample.values.empty()){
                sampleQueue.push_back(std::move(sample));
                while(static_cast<int>(sampleQueue.size()) > cfg.maxQueuedSamples){
                    sampleQueue.pop_front();
                }
            }
        }

        totalCompleteFrameCount += completeFrameCount;

        int nextValidNum = 0;
        int nextFreeNum = 0;
        int nextObjectTotalBytes = 0;
        int nextObjectTotalNum = 0;
        ret = dmc_trace_get_state(static_cast<WORD>(cfg.cardNo),
                                  &nextValidNum,
                                  &nextFreeNum,
                                  &nextObjectTotalBytes,
                                  &nextObjectTotalNum);
        (void)nextFreeNum;
        (void)nextObjectTotalBytes;
        (void)nextObjectTotalNum;
        lastRet = ret;
        if(ret != 0 || nextValidNum <= 0 || completeFrameCount < frameCountToRead){
            break;
        }
        currentValidNum = nextValidNum;
    }

    if(totalCompleteFrameCount <= 0){
        return traceEverRead ? 0 : -1;
    }
    traceEverRead = true;
    return totalCompleteFrameCount;
}

void RuntimeTraceSlaveReader::applyRawValue(int objectIndex, long rawValue)
{
    if(objectIndex < 0 || objectIndex >= static_cast<int>(cfg.objects.size())){
        return;
    }

    const int outputIndex = outputIndexForObject(objectIndex);
    if(outputIndex < 0){
        return;
    }
    if(outputIndex >= static_cast<int>(cachedValues.size())){
        cachedValues.resize(static_cast<std::size_t>(outputIndex + 1), 0.0);
        cacheValid.resize(static_cast<std::size_t>(outputIndex + 1), false);
    }

    const ObjectConfig& object = cfg.objects[objectIndex];
    const double raw = normalizeRawValue(rawValue, object.signMode);
    cachedValues[outputIndex] = (raw - object.home) * object.scale;
    cacheValid[outputIndex] = true;
}

int RuntimeTraceSlaveReader::outputIndexForObject(int objectIndex) const
{
    if(objectIndex < 0 || objectIndex >= static_cast<int>(cfg.objects.size())){
        return -1;
    }
    return cfg.objects[objectIndex].logicalIndex >= 0 ?
                cfg.objects[objectIndex].logicalIndex :
                objectIndex;
}

std::vector<double> RuntimeTraceSlaveReader::currentValues() const
{
    std::vector<double> values = cachedValues;
    for(int i = 0; i < static_cast<int>(values.size()); ++i){
        if(i >= static_cast<int>(cacheValid.size()) || !cacheValid[i]){
            values[i] = 0.0;
        }
    }
    return values;
}

std::vector<RuntimeTraceSlaveReader::Sample>
RuntimeTraceSlaveReader::takeSamples()
{
    std::vector<Sample> result;
    result.reserve(sampleQueue.size());
    while(!sampleQueue.empty()){
        result.push_back(std::move(sampleQueue.front()));
        sampleQueue.pop_front();
    }
    return result;
}
