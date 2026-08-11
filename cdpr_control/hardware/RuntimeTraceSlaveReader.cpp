#include "hardware/RuntimeTraceSlaveReader.h"

#include "LTDMC.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
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

quint32 readUnsignedLittleEndian32(const unsigned char *raw)
{
    return static_cast<quint32>(raw[0])
        | (static_cast<quint32>(raw[1]) << 8)
        | (static_cast<quint32>(raw[2]) << 16)
        | (static_cast<quint32>(raw[3]) << 24);
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
        failureReason_ = QStringLiteral("Trace配置对象为空");
        return false;
    }
    if (config_.samplePeriodUs % config_.traceBaseCycleUs != 0) {
        failureReason_ = QStringLiteral("Trace采样周期不是基础周期的整数倍");
        return false;
    }

    const WORD cardNo = static_cast<WORD>(config_.cardNo);
    lastResult_ = dmc_trace_data_stop(cardNo);
    lastResult_ = dmc_trace_data_reset(cardNo);
    if (lastResult_ != 0) {
        failureReason_ = QStringLiteral("dmc_trace_data_reset失败，错误码=%1")
                             .arg(lastResult_);
        return false;
    }
    const short cycle = static_cast<short>(config_.samplePeriodUs / config_.traceBaseCycleUs);
    lastResult_ = dmc_trace_set_config(cardNo, cycle, 0, 0, 0, 0, 0, 0);
    if (lastResult_ != 0) {
        failureReason_ = QStringLiteral("dmc_trace_set_config失败，错误码=%1")
                             .arg(lastResult_);
        return false;
    }
    lastResult_ = dmc_trace_reset_config_object(cardNo);
    if (lastResult_ != 0) {
        failureReason_ = QStringLiteral("dmc_trace_reset_config_object失败，错误码=%1")
                             .arg(lastResult_);
        return false;
    }
    for (int index = 0; index < static_cast<int>(config_.objects.size()); ++index) {
        const ObjectConfig &object = config_.objects[static_cast<std::size_t>(index)];
        lastResult_ = dmc_trace_add_config_object(cardNo, object.dataType, object.dataIndex,
                                                   object.dataSubIndex, object.slaveId,
                                                   object.apiDataBytes);
        if (lastResult_ != 0) {
            dmc_trace_data_stop(cardNo);
            failureReason_ = QStringLiteral(
                "dmc_trace_add_config_object失败：对象=%1，错误码=%2")
                .arg(index).arg(lastResult_);
            return false;
        }
    }
    // 配置后逐项读回只做诊断。部分运行库会对无关字段归一化，不能将这些
    // 表达差异误判成配置失败；真正的固定帧契约在启动后由get_state校验。
    for (int index = 0; index < static_cast<int>(config_.objects.size()); ++index) {
        short dataType = 0;
        int dataIndex = 0;
        int dataSubIndex = 0;
        short slaveId = 0;
        short dataBytes = 0;
        const short readbackResult = dmc_trace_get_config_object(
            cardNo, static_cast<short>(index), &dataType, &dataIndex,
            &dataSubIndex, &slaveId, &dataBytes);
        const ObjectConfig &expected = config_.objects[static_cast<std::size_t>(index)];
        if (readbackResult != 0 || dataType != expected.dataType
            || dataIndex != expected.dataIndex
            || dataSubIndex != expected.dataSubIndex
            || slaveId != expected.slaveId
            || (dataBytes > 0 && dataBytes != expected.valueBytes)) {
            ++configReadbackMismatchCount_;
        }
    }
    lastResult_ = dmc_trace_data_start(cardNo);
    if (lastResult_ != 0) {
        failureReason_ = QStringLiteral("dmc_trace_data_start失败，错误码=%1")
                             .arg(lastResult_);
        return false;
    }
    configured_ = true;
    everRead_ = false;
    timingReliable_ = true;
    lastValidFrames_ = 0;
    lastFreeFrames_ = 0;
    maximumValidFrames_ = 0;
    minimumFreeFrames_ = -1;
    locallyDroppedSamples_ = 0;
    objectTotalBytes_ = 0;
    objectTotalNum_ = 0;
    frameBytes_ = 0;
    frameHeaderBytes_ = 0;
    hardwareSequenceAvailable_ = false;
    hardwareSequenceInitialized_ = false;
    firstHardwareSequence_ = 0;
    lastHardwareSequence_ = 0;
    firstLogicalSequence_ = 0;
    lastLogicalSequence_ = 0;
    logicalToHostTimeRatio_ = 1.0;
    productionRateDiagnosticValid_ = false;
    productionBalanceBaseline_ = -1;
    cumulativeConsumedFrames_ = 0;
    estimatedGeneratedFrames_ = 0;
    sequenceGapFrames_ = 0;
    lastReadBytes_ = 0;
    lastReadFrames_ = 0;
    lastReadFirstSequence_ = 0;
    lastReadLastSequence_ = 0;
    productionRateClock_.invalidate();
    failureReason_.clear();
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
    timingReliable_ = true;
    lastResult_ = 0;
    lastValidFrames_ = 0;
    lastFreeFrames_ = 0;
    maximumValidFrames_ = 0;
    minimumFreeFrames_ = -1;
    locallyDroppedSamples_ = 0;
    objectTotalBytes_ = 0;
    objectTotalNum_ = 0;
    frameBytes_ = 0;
    frameHeaderBytes_ = 0;
    hardwareSequenceAvailable_ = false;
    hardwareSequenceInitialized_ = false;
    firstHardwareSequence_ = 0;
    lastHardwareSequence_ = 0;
    firstLogicalSequence_ = 0;
    lastLogicalSequence_ = 0;
    logicalToHostTimeRatio_ = 1.0;
    productionRateDiagnosticValid_ = false;
    productionBalanceBaseline_ = -1;
    cumulativeConsumedFrames_ = 0;
    estimatedGeneratedFrames_ = 0;
    sequenceGapFrames_ = 0;
    lastReadBytes_ = 0;
    lastReadFrames_ = 0;
    lastReadFirstSequence_ = 0;
    lastReadLastSequence_ = 0;
    configReadbackMismatchCount_ = 0;
    failureReason_.clear();
    productionRateClock_.invalidate();
    values_.clear();
    samples_.clear();
    nextSequence_ = 1;
}

RuntimeTraceSlaveReader::FrameLayout RuntimeTraceSlaveReader::resolveLayout(
    int objectTotalBytes, int objectTotalNum) const
{
    FrameLayout layout;
    int valueBytesTotal = 0;
    for (const ObjectConfig &object : config_.objects) {
        valueBytesTotal += std::max(1, object.valueBytes);
    }
    // objectTotalBytes/objectTotalNum 是板卡对当前固定对象配置的回读结果。
    // 不再枚举 1/2/4/8 字节槽位猜测帧布局。
    if (config_.objects.empty()
        || objectTotalNum != static_cast<int>(config_.objects.size())
        || objectTotalBytes < valueBytesTotal) {
        return layout;
    }
    layout.frameBytes = objectTotalBytes;
    layout.valueStart = objectTotalBytes - valueBytesTotal;
    layout.hasSequenceHeader = layout.valueStart >= 4;
    return layout;
}

quint64 RuntimeTraceSlaveReader::decodeSequence(
    const unsigned char *frameData, bool hasSequenceHeader)
{
    if (!hasSequenceHeader) {
        hardwareSequenceAvailable_ = false;
        return nextSequence_++;
    }

    hardwareSequenceAvailable_ = true;
    const quint32 rawSequence = readUnsignedLittleEndian32(frameData);
    if (!hardwareSequenceInitialized_) {
        hardwareSequenceInitialized_ = true;
        firstHardwareSequence_ = rawSequence;
        lastHardwareSequence_ = rawSequence;
        nextSequence_ = 2;
        return 1;
    }

    const quint32 increment = rawSequence - lastHardwareSequence_;
    if (increment == 0 || increment > 1000000U) {
        timingReliable_ = false;
        failureReason_ = QStringLiteral(
            "Trace硬件帧序号重复或倒退：上一原始序号=%1，当前=%2")
            .arg(lastHardwareSequence_)
            .arg(rawSequence);
        return nextSequence_++;
    }
    if (increment != 1U) {
        // 保留硬件帧间隙，使逻辑时间不会因漏帧而被压短；同时标记时间轴不可信。
        timingReliable_ = false;
        sequenceGapFrames_ += static_cast<quint64>(increment - 1U);
        failureReason_ = QStringLiteral(
            "Trace硬件帧序号断裂：上一原始序号=%1，当前=%2，缺少=%3帧")
            .arg(lastHardwareSequence_)
            .arg(rawSequence)
            .arg(increment - 1U);
    }
    lastHardwareSequence_ = rawSequence;
    const quint64 sequence = static_cast<quint64>(rawSequence - firstHardwareSequence_) + 1U;
    nextSequence_ = sequence + 1U;
    return sequence;
}

void RuntimeTraceSlaveReader::updateProductionRateDiagnostic(int currentValidFrames)
{
    const qint64 balance = static_cast<qint64>(cumulativeConsumedFrames_)
        + std::max(0, currentValidFrames);
    if (productionBalanceBaseline_ < 0) {
        productionBalanceBaseline_ = balance;
        productionRateClock_.start();
        productionRateDiagnosticValid_ = false;
        logicalToHostTimeRatio_ = 1.0;
        return;
    }
    if (!productionRateClock_.isValid()) {
        productionRateClock_.start();
        productionBalanceBaseline_ = balance;
        return;
    }
    const qint64 hostElapsedUs = productionRateClock_.nsecsElapsed() / 1000;
    const qint64 generatedFrames = balance - productionBalanceBaseline_;
    if (hostElapsedUs < 500000 || generatedFrames < 0) {
        return;
    }
    estimatedGeneratedFrames_ = static_cast<quint64>(generatedFrames);
    logicalToHostTimeRatio_ =
        static_cast<double>(estimatedGeneratedFrames_)
        * static_cast<double>(config_.samplePeriodUs)
        / static_cast<double>(hostElapsedUs);
    productionRateDiagnosticValid_ = true;
}

int RuntimeTraceSlaveReader::readTraceCached()
{
    if (!configured_) {
        failureReason_ = QStringLiteral("Trace尚未配置");
        return -1;
    }
    lastReadBytes_ = 0;
    lastReadFrames_ = 0;
    lastReadFirstSequence_ = 0;
    lastReadLastSequence_ = 0;
    int validNum = 0, freeNum = 0, objectTotalBytes = 0, objectTotalNum = 0;
    lastResult_ = dmc_trace_get_state(static_cast<WORD>(config_.cardNo), &validNum, &freeNum,
                                      &objectTotalBytes, &objectTotalNum);
    if (lastResult_ != 0) {
        timingReliable_ = false;
        failureReason_ = QStringLiteral("dmc_trace_get_state失败，错误码=%1")
                             .arg(lastResult_);
        return everRead_ ? 0 : -1;
    }
    updateBufferDiagnostics(validNum, freeNum);
    updateProductionRateDiagnostic(validNum);
    if (validNum <= 0) {
        return everRead_ ? 0 : -1;
    }
    objectTotalBytes_ = objectTotalBytes;
    objectTotalNum_ = objectTotalNum;
    const FrameLayout layout = resolveLayout(objectTotalBytes, objectTotalNum);
    if (layout.frameBytes <= 0) {
        timingReliable_ = false;
        int expectedValueBytes = 0;
        for (const ObjectConfig &object : config_.objects) {
            expectedValueBytes += std::max(1, object.valueBytes);
        }
        failureReason_ = QStringLiteral(
            "Trace固定帧契约不成立：板卡对象数=%1，期望=%2，"
            "板卡对象总字节=%3，期望数据至少=%4")
            .arg(objectTotalNum)
            .arg(config_.objects.size())
            .arg(objectTotalBytes)
            .arg(expectedValueBytes);
        return -1;
    }
    frameBytes_ = layout.frameBytes;
    frameHeaderBytes_ = layout.valueStart;
    int total = 0;
    for (int drain = 0; drain < config_.maxDrainReads && validNum > 0; ++drain) {
        const int frameCount = std::max(1, std::min(validNum, config_.maxBufferBytes / layout.frameBytes));
        const int bufferSize = std::max(layout.frameBytes, frameCount * layout.frameBytes);
        if (readBuffer_.size() < static_cast<std::size_t>(bufferSize)) {
            readBuffer_.resize(static_cast<std::size_t>(bufferSize));
        }
        int readBytes = 0;
        lastResult_ = dmc_trace_get_data(static_cast<WORD>(config_.cardNo), bufferSize,
                                         readBuffer_.data(), &readBytes);
        if (lastResult_ != 0 || readBytes < layout.frameBytes) {
            timingReliable_ = false;
            failureReason_ = lastResult_ != 0
                ? QStringLiteral("dmc_trace_get_data失败，错误码=%1").arg(lastResult_)
                : QStringLiteral("Trace返回数据不足一帧：返回=%1字节，固定帧宽=%2字节")
                      .arg(readBytes).arg(layout.frameBytes);
            return total > 0 ? total : (everRead_ ? 0 : -1);
        }
        if (readBytes % layout.frameBytes != 0) {
            timingReliable_ = false;
            failureReason_ = QStringLiteral(
                "Trace返回非整帧数据：返回=%1字节，固定帧宽=%2字节，余数=%3")
                .arg(readBytes).arg(layout.frameBytes)
                .arg(readBytes % layout.frameBytes);
        }
        const int completeFrames = readBytes / layout.frameBytes;
        for (int frame = 0; frame < completeFrames; ++frame) {
            const unsigned char *frameData =
                readBuffer_.data() + frame * layout.frameBytes;
            int offset = layout.valueStart;
            for (int objectIndex = 0; objectIndex < static_cast<int>(config_.objects.size()); ++objectIndex) {
                const ObjectConfig &object = config_.objects[objectIndex];
                const int bytes = std::max(1, object.valueBytes);
                if (offset + bytes > layout.frameBytes) {
                    timingReliable_ = false;
                    failureReason_ = QStringLiteral(
                        "Trace对象越过固定帧边界：对象=%1，偏移=%2，字节数=%3，帧宽=%4")
                        .arg(objectIndex)
                        .arg(offset)
                        .arg(bytes)
                        .arg(layout.frameBytes);
                    return total > 0 ? total : -1;
                }
                const int output = outputIndex(objectIndex);
                values_[output] = static_cast<double>(readSignedLittleEndian(
                                      frameData + offset, bytes)) * object.scale;
                offset += bytes;
            }
            const quint64 sequence = decodeSequence(frameData, layout.hasSequenceHeader);
            if (firstLogicalSequence_ == 0) {
                firstLogicalSequence_ = sequence;
            }
            lastLogicalSequence_ = sequence;
            if (lastReadFirstSequence_ == 0) {
                lastReadFirstSequence_ = sequence;
            }
            lastReadLastSequence_ = sequence;
            samples_.push_back({sequence, values_});
            while (static_cast<int>(samples_.size()) > config_.maxQueuedSamples) {
                samples_.pop_front();
                ++locallyDroppedSamples_;
                timingReliable_ = false;
                failureReason_ = QStringLiteral(
                    "Trace本地待处理队列溢出：累计丢弃=%1帧")
                    .arg(locallyDroppedSamples_);
            }
        }
        total += completeFrames;
        lastReadBytes_ += readBytes;
        lastReadFrames_ += completeFrames;
        cumulativeConsumedFrames_ += static_cast<quint64>(completeFrames);
        lastResult_ = dmc_trace_get_state(static_cast<WORD>(config_.cardNo), &validNum, &freeNum,
                                          &objectTotalBytes, &objectTotalNum);
        if (lastResult_ != 0) {
            timingReliable_ = false;
            failureReason_ = QStringLiteral("补读后dmc_trace_get_state失败，错误码=%1")
                                 .arg(lastResult_);
            break;
        }
        updateBufferDiagnostics(validNum, freeNum);
        if (validNum <= 0 || completeFrames < frameCount) {
            break;
        }
    }
    updateProductionRateDiagnostic(validNum);
    if (timingReliable_) {
        failureReason_.clear();
    }
    everRead_ = everRead_ || total > 0;
    return total;
}

void RuntimeTraceSlaveReader::updateBufferDiagnostics(int validFrames, int freeFrames)
{
    lastValidFrames_ = std::max(0, validFrames);
    lastFreeFrames_ = std::max(0, freeFrames);
    maximumValidFrames_ = std::max(maximumValidFrames_, lastValidFrames_);
    if (minimumFreeFrames_ < 0) {
        minimumFreeFrames_ = lastFreeFrames_;
    } else {
        minimumFreeFrames_ = std::min(minimumFreeFrames_, lastFreeFrames_);
    }

    // Trace API不返回每帧硬件时间戳。本程序只能按“实际读到的帧数×采样周期”
    // 重建逻辑时间；一旦卡侧无剩余空间，就无法证明期间没有覆盖旧帧。
    if (lastFreeFrames_ == 0) {
        timingReliable_ = false;
        failureReason_ = QStringLiteral("Trace卡侧缓冲区剩余空间为0，存在覆盖风险");
    }
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
