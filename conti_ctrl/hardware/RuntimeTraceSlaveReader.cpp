#include "hardware/RuntimeTraceSlaveReader.h"

#include "LTDMC.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <thread>

namespace {
long readSignedLittleEndian(const unsigned char *raw, int bytes)
{
    if (bytes >= 4) {
        const std::uint32_t value = static_cast<std::uint32_t>(raw[0])
            | (static_cast<std::uint32_t>(raw[1]) << 8)
            | (static_cast<std::uint32_t>(raw[2]) << 16)
            | (static_cast<std::uint32_t>(raw[3]) << 24);
        return static_cast<long>(static_cast<std::int32_t>(value));
    }
    if (bytes == 2) {
        const std::uint16_t value = static_cast<std::uint16_t>(raw[0])
            | (static_cast<std::uint16_t>(raw[1]) << 8);
        return static_cast<long>(static_cast<std::int16_t>(value));
    }
    return static_cast<long>(static_cast<std::int8_t>(raw[0]));
}
}

RuntimeTraceSlaveReader::~RuntimeTraceSlaveReader()
{
    reset();
}

bool RuntimeTraceSlaveReader::configure(const ReaderConfig &config)
{
    reset();
    config_ = config;
    config_.samplePeriodUs = std::max(1, config_.samplePeriodUs);
    config_.traceBaseCycleUs = std::max(1, config_.traceBaseCycleUs);
    config_.maxBufferBytes = std::max(1024, config_.maxBufferBytes);
    config_.maxDrainReads = std::max(1, config_.maxDrainReads);
    config_.maxQueuedSamples = std::max(1, config_.maxQueuedSamples);
    ensureValues();
    if (config_.objects.empty()) {
        return false;
    }

    const WORD cardNo = static_cast<WORD>(config_.cardNo);
    lastResult_ = dmc_trace_data_stop(cardNo);
    lastResult_ = dmc_trace_data_reset(cardNo);
    if (lastResult_ != 0) {
        return false;
    }
    const short cycle = static_cast<short>(std::max(1, config_.samplePeriodUs / config_.traceBaseCycleUs));
    lastResult_ = dmc_trace_set_config(cardNo, cycle, 0, 0, 0, 0, 0, 0);
    if (lastResult_ != 0) {
        return false;
    }
    lastResult_ = dmc_trace_reset_config_object(cardNo);
    if (lastResult_ != 0) {
        return false;
    }
    for (const ObjectConfig &object : config_.objects) {
        lastResult_ = dmc_trace_add_config_object(cardNo, object.dataType, object.dataIndex,
                                                   object.dataSubIndex, object.slaveId,
                                                   object.apiDataBytes);
        if (lastResult_ != 0) {
            dmc_trace_data_stop(cardNo);
            return false;
        }
    }
    lastResult_ = dmc_trace_data_start(cardNo);
    if (lastResult_ != 0) {
        return false;
    }
    configured_ = true;
    everRead_ = false;
    nextSequence_ = 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(2, config_.samplePeriodUs / 1000 + 1)));
    return true;
}

void RuntimeTraceSlaveReader::reset()
{
    if (configured_) {
        dmc_trace_data_stop(static_cast<WORD>(config_.cardNo));
    }
    configured_ = false;
    everRead_ = false;
    lastResult_ = 0;
    values_.clear();
    samples_.clear();
    nextSequence_ = 1;
}

RuntimeTraceSlaveReader::FrameLayout RuntimeTraceSlaveReader::resolveLayout(int objectTotalBytes) const
{
    FrameLayout layout;
    int valueBytesTotal = 0;
    int maxValueBytes = 0;
    for (const ObjectConfig &object : config_.objects) {
        valueBytesTotal += std::max(1, object.valueBytes);
        maxValueBytes = std::max(maxValueBytes, std::max(1, object.valueBytes));
    }
    layout.frameBytes = std::max(valueBytesTotal, objectTotalBytes);
    if (objectTotalBytes < valueBytesTotal || config_.objects.empty()) {
        return layout;
    }
    int bestHeader = std::numeric_limits<int>::max();
    for (const int slotBytes : {8, 4, 2, 1}) {
        const int bodyBytes = slotBytes * static_cast<int>(config_.objects.size());
        if (slotBytes >= maxValueBytes && bodyBytes <= objectTotalBytes
            && objectTotalBytes - bodyBytes < bestHeader) {
            bestHeader = objectTotalBytes - bodyBytes;
            layout.valueStart = bestHeader;
            layout.objectStep = slotBytes;
        }
    }
    if (layout.objectStep == 0) {
        layout.valueStart = objectTotalBytes - valueBytesTotal;
    }
    return layout;
}

int RuntimeTraceSlaveReader::readTraceCached()
{
    if (!configured_) {
        return -1;
    }
    int validNum = 0, freeNum = 0, objectTotalBytes = 0, objectTotalNum = 0;
    lastResult_ = dmc_trace_get_state(static_cast<WORD>(config_.cardNo), &validNum, &freeNum,
                                      &objectTotalBytes, &objectTotalNum);
    if (lastResult_ != 0 || validNum <= 0) {
        return everRead_ ? 0 : -1;
    }
    const FrameLayout layout = resolveLayout(objectTotalBytes);
    if (layout.frameBytes <= 0) {
        return -1;
    }
    int total = 0;
    for (int drain = 0; drain < config_.maxDrainReads && validNum > 0; ++drain) {
        const int frameCount = std::max(1, std::min(validNum, config_.maxBufferBytes / layout.frameBytes));
        const int bufferSize = std::max(layout.frameBytes, frameCount * layout.frameBytes);
        std::vector<unsigned char> buffer(static_cast<std::size_t>(bufferSize));
        int readBytes = 0;
        lastResult_ = dmc_trace_get_data(static_cast<WORD>(config_.cardNo), bufferSize, buffer.data(), &readBytes);
        if (lastResult_ != 0 || readBytes < layout.frameBytes) {
            return total > 0 ? total : (everRead_ ? 0 : -1);
        }
        const int completeFrames = readBytes / layout.frameBytes;
        for (int frame = 0; frame < completeFrames; ++frame) {
            int offset = layout.valueStart;
            for (int objectIndex = 0; objectIndex < static_cast<int>(config_.objects.size()); ++objectIndex) {
                const ObjectConfig &object = config_.objects[objectIndex];
                const int bytes = std::max(1, object.valueBytes);
                if (offset + bytes > layout.frameBytes) {
                    return total > 0 ? total : -1;
                }
                const int output = outputIndex(objectIndex);
                values_[output] = static_cast<double>(readSignedLittleEndian(
                                      buffer.data() + frame * layout.frameBytes + offset, bytes)) * object.scale;
                offset += layout.objectStep >= bytes ? layout.objectStep : bytes;
            }
            samples_.push_back({nextSequence_++, values_});
            while (static_cast<int>(samples_.size()) > config_.maxQueuedSamples) {
                samples_.pop_front();
            }
        }
        total += completeFrames;
        lastResult_ = dmc_trace_get_state(static_cast<WORD>(config_.cardNo), &validNum, &freeNum,
                                          &objectTotalBytes, &objectTotalNum);
        if (lastResult_ != 0 || validNum <= 0 || completeFrames < frameCount) {
            break;
        }
    }
    everRead_ = everRead_ || total > 0;
    return total;
}

std::vector<RuntimeTraceSlaveReader::Sample> RuntimeTraceSlaveReader::takeSamples()
{
    std::vector<Sample> result;
    result.reserve(samples_.size());
    while (!samples_.empty()) {
        result.push_back(std::move(samples_.front()));
        samples_.pop_front();
    }
    return result;
}

int RuntimeTraceSlaveReader::outputIndex(int objectIndex) const
{
    const int index = config_.objects[objectIndex].logicalIndex;
    return index >= 0 ? index : objectIndex;
}

void RuntimeTraceSlaveReader::ensureValues()
{
    int count = 0;
    for (int index = 0; index < static_cast<int>(config_.objects.size()); ++index) {
        count = std::max(count, outputIndex(index) + 1);
    }
    values_.assign(static_cast<std::size_t>(std::max(0, count)), 0.0);
}
