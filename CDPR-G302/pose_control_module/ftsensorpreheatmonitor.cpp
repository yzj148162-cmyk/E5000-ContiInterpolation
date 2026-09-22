#include "ftsensorpreheatmonitor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr qint64 kUsPerSecond = 1000000;
constexpr std::size_t kMinimumStatisticsSamples = 100;
}

bool FtSensorPreheatMonitor::runTraceEpochSelfChecks(QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    FtSensorStabilityConfig config;
    config.normalMinimumObservationS = 0.0;
    config.previouslyPoweredMinimumObservationS = 0.0;
    config.statisticsWindowS = 1.0;
    config.stableHoldS = 0.0;
    config.forceStdLimitN = 1.0;
    config.momentStdLimitNm = 1.0;
    config.forceSlopeLimitNPerS = 1.0;
    config.momentSlopeLimitNmPerS = 1.0;
    config.forcePeakToPeakLimitN = 1.0;
    config.momentPeakToPeakLimitNm = 1.0;
    config.minimumEffectiveRate = 0.0;
    config.maximumSampleAgeS = 2.0;

    FtSensorPreheatMonitor monitor;
    monitor.configure(config);
    constexpr qint64 startUs = 1000000;
    monitor.start(false, startUs);
    std::vector<FtSensorTraceSample> samples;
    samples.reserve(1001);
    for(quint32 index = 0; index <= 1000; ++index){
        FtSensorTraceSample sample;
        sample.monotonicUs = startUs + static_cast<qint64>(index) * 1000;
        sample.traceFrameSequence = 500000u + index;
        sample.traceFrameSequenceValid = true;
        sample.sampleCounter = 700000u + index;
        sample.sampleCounterValid = true;
        sample.statusCode = 0u;
        sample.statusValid = true;
        sample.temperatureC = 25.0;
        sample.temperatureValid = true;
        sample.channelValid.fill(true);
        samples.push_back(sample);
    }
    monitor.ingest(samples);
    QString zeroError;
    if(!monitor.confirmZero(startUs + 1000000, &zeroError)){
        return fail(QStringLiteral("Trace纪元自检无法建立软件零点：%1")
                    .arg(zeroError));
    }
    const FtSensorPreheatStatus before = monitor.status(startUs + 1000000);
    monitor.beginTraceEpoch();
    FtSensorTraceSample resetFrame = samples.back();
    resetFrame.monotonicUs += 1000;
    resetFrame.traceFrameSequence = 0u;
    resetFrame.sampleCounter += 1u;
    monitor.ingest({resetFrame});
    const FtSensorPreheatStatus after = monitor.status(resetFrame.monotonicUs);
    if(!before.preheatQualified || !before.zeroValid ||
            !after.preheatQualified || !after.zeroValid){
        return fail(QStringLiteral("Trace新纪元错误地撤销了预热资格或软件零点"));
    }
    if(after.acceptedSamples != before.acceptedSamples + 1u ||
            after.rejectedSamples != before.rejectedSamples){
        return fail(QStringLiteral("Trace帧号归零后的首帧未被作为新纪元有效帧接受"));
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

void FtSensorPreheatMonitor::configure(const FtSensorStabilityConfig& config)
{
    config_ = config;
    config_.normalMinimumObservationS = std::max(0.0, config_.normalMinimumObservationS);
    config_.previouslyPoweredMinimumObservationS =
            std::max(0.0, config_.previouslyPoweredMinimumObservationS);
    config_.statisticsWindowS = std::max(1.0, config_.statisticsWindowS);
    config_.stableHoldS = std::max(0.0, config_.stableHoldS);
    config_.minimumEffectiveRate = std::clamp(config_.minimumEffectiveRate, 0.0, 1.0);
    config_.maximumSampleAgeS = std::max(0.1, config_.maximumSampleAgeS);
    config_.candidateInstabilityGraceS =
            std::max(0.0, config_.candidateInstabilityGraceS);
    config_.releaseMultiplier = std::max(1.0, config_.releaseMultiplier);
}

FtSensorStabilityConfig FtSensorPreheatMonitor::configuration() const
{
    return config_;
}

void FtSensorPreheatMonitor::start(bool previouslyPowered, qint64 startMonotonicUs)
{
    reset();
    monitoring_ = true;
    previouslyPowered_ = previouslyPowered;
    startMonotonicUs_ = startMonotonicUs;
}

void FtSensorPreheatMonitor::stop()
{
    monitoring_ = false;
    candidateStableAccumulatedUs_ = 0;
    candidateLastUpdateUs_ = 0;
    candidateInstabilityStartUs_ = 0;
    candidateActive_ = false;
    candidatePaused_ = false;
}

void FtSensorPreheatMonitor::reset()
{
    window_.clear();
    startMonotonicUs_ = 0;
    candidateStableAccumulatedUs_ = 0;
    candidateLastUpdateUs_ = 0;
    candidateInstabilityStartUs_ = 0;
    acceptedSamples_ = 0;
    rejectedSamples_ = 0;
    receivedTraceSamples_ = 0;
    completeTraceSamples_ = 0;
    duplicateSensorSamples_ = 0;
    missingTraceFrames_ = 0;
    lastSampleCounter_ = 0;
    lastSampleCounterValid_ = false;
    lastTraceFrameSequence_ = 0;
    lastTraceFrameSequenceValid_ = false;
    monitoring_ = false;
    previouslyPowered_ = false;
    candidateActive_ = false;
    candidatePaused_ = false;
}

void FtSensorPreheatMonitor::ingest(const std::vector<FtSensorTraceSample>& samples)
{
    if(!monitoring_){
        return;
    }
    for(const FtSensorTraceSample& sample : samples){
        ++receivedTraceSamples_;
        if(!sample.traceFrameSequenceValid){
            ++rejectedSamples_;
            continue;
        }
        if(lastTraceFrameSequenceValid_){
            const quint32 frameIncrement =
                    sample.traceFrameSequence - lastTraceFrameSequence_;
            if(frameIncrement > 1 && frameIncrement < 0x80000000U){
                missingTraceFrames_ += static_cast<quint64>(frameIncrement - 1U);
            }
            else if(frameIncrement == 0 || frameIncrement >= 0x80000000U){
                ++rejectedSamples_;
                continue;
            }
        }
        lastTraceFrameSequence_ = sample.traceFrameSequence;
        lastTraceFrameSequenceValid_ = true;

        const bool completePdoFrame = sample.wrenchComplete() &&
                sample.monotonicUs > 0 &&
                sample.statusValid &&
                sample.sampleCounterValid &&
                sample.temperatureValid;
        if(!completePdoFrame){
            ++rejectedSamples_;
            continue;
        }
        ++completeTraceSamples_;

        if(sample.sampleCounterValid){
            if(lastSampleCounterValid_){
                const quint32 increment = sample.sampleCounter - lastSampleCounter_;
                if(increment == 0){
                    // 传感器PDO更新与Trace采样同频但不同相时会出现一帧保持、
                    // 后一帧追赶的模式。保持帧是完整Trace帧，不是拒绝或丢帧；
                    // 只是不重复加入稳定性统计窗。
                    ++duplicateSensorSamples_;
                    continue;
                }
                if(increment >= 0x80000000U){
                    ++rejectedSamples_;
                    continue;
                }
            }
            lastSampleCounter_ = sample.sampleCounter;
            lastSampleCounterValid_ = true;
        }
        window_.push_back(sample);
        ++acceptedSamples_;
        trim(sample.monotonicUs);
    }
}

void FtSensorPreheatMonitor::beginTraceEpoch()
{
    lastTraceFrameSequence_ = 0;
    lastTraceFrameSequenceValid_ = false;
}

void FtSensorPreheatMonitor::trim(qint64 newestMonotonicUs)
{
    const qint64 retentionUs = static_cast<qint64>(
                std::ceil(config_.statisticsWindowS * kUsPerSecond));
    while(!window_.empty() &&
          newestMonotonicUs - window_.front().monotonicUs > retentionUs){
        window_.pop_front();
    }
}

FtSensorPreheatStatus FtSensorPreheatMonitor::calculateStatus(
        qint64 nowMonotonicUs) const
{
    FtSensorPreheatStatus result;
    result.monitoring = monitoring_;
    result.previouslyPowered = previouslyPowered_;
    result.preheatQualified = preheatQualified_;
    result.zeroValid = zeroValid_;
    result.zero = zero_;
    result.acceptedSamples = acceptedSamples_;
    result.rejectedSamples = rejectedSamples_;
    result.missingSensorSamples = missingTraceFrames_;
    result.receivedTraceSamples = receivedTraceSamples_;
    result.completeTraceSamples = completeTraceSamples_;
    result.duplicateSensorSamples = duplicateSensorSamples_;
    result.missingTraceFrames = missingTraceFrames_;
    result.requiredObservationS = previouslyPowered_ ?
                config_.previouslyPoweredMinimumObservationS :
                config_.normalMinimumObservationS;
    if(startMonotonicUs_ > 0 && nowMonotonicUs >= startMonotonicUs_){
        result.elapsedS = static_cast<double>(nowMonotonicUs - startMonotonicUs_) /
                kUsPerSecond;
    }
    result.minimumObservationSatisfied =
            result.elapsedS >= result.requiredObservationS;
    if(window_.size() < kMinimumStatisticsSamples){
        result.message = QStringLiteral("等待足够的有效Trace样本");
        return result;
    }

    const qint64 firstUs = window_.front().monotonicUs;
    const qint64 lastUs = window_.back().monotonicUs;
    if(lastUs <= firstUs){
        result.message = QStringLiteral("样本时间戳未推进");
        return result;
    }
    result.windowSpanS = static_cast<double>(lastUs - firstUs) / kUsPerSecond;
    // 调用方和Trace解码都使用steady_clock，但跨线程排队时查询时刻可能
    // 比刚解码样本早几毫秒。负帧龄代表样本更新，不是超时，按0处理。
    result.newestSampleAgeS = nowMonotonicUs >= lastUs ?
                static_cast<double>(nowMonotonicUs - lastUs) / kUsPerSecond : 0.0;
    result.currentDataFresh = result.newestSampleAgeS <= config_.maximumSampleAgeS;
    const double minimumWindowS = std::min(config_.statisticsWindowS,
                                           result.requiredObservationS);
    result.statisticsAvailable = result.windowSpanS >=
            std::max(1.0, minimumWindowS * 0.95);

    const double totalObserved = static_cast<double>(completeTraceSamples_ +
                                                     rejectedSamples_ +
                                                     missingTraceFrames_);
    result.effectiveRate = totalObserved > 0.0 ?
                static_cast<double>(completeTraceSamples_) / totalObserved : 0.0;

    const double originS = static_cast<double>(firstUs) / kUsPerSecond;
    double sumT = 0.0;
    double sumTT = 0.0;
    std::array<double, kFtSensorWrenchChannelCount> sum{};
    std::array<double, kFtSensorWrenchChannelCount> sumSquared{};
    std::array<double, kFtSensorWrenchChannelCount> sumTY{};
    std::array<double, kFtSensorWrenchChannelCount> minimum{};
    std::array<double, kFtSensorWrenchChannelCount> maximum{};
    minimum.fill(std::numeric_limits<double>::infinity());
    maximum.fill(-std::numeric_limits<double>::infinity());
    for(const FtSensorTraceSample& sample : window_){
        const double t = static_cast<double>(sample.monotonicUs) /
                kUsPerSecond - originS;
        sumT += t;
        sumTT += t * t;
        for(int channel = 0; channel < kFtSensorWrenchChannelCount; ++channel){
            const double value = sample.value[channel];
            sum[channel] += value;
            sumSquared[channel] += value * value;
            sumTY[channel] += t * value;
            minimum[channel] = std::min(minimum[channel], value);
            maximum[channel] = std::max(maximum[channel], value);
        }
    }
    const double count = static_cast<double>(window_.size());
    const double denominator = count * sumTT - sumT * sumT;
    bool channelsStable = true;
    bool channelsWithinReleaseLimits = true;
    double worstViolationRatio = 1.0;
    QString worstViolationCode;
    QString worstViolationDetail;
    const std::array<QString, kFtSensorWrenchChannelCount> channelNames{{
        QStringLiteral("Fx"), QStringLiteral("Fy"), QStringLiteral("Fz"),
        QStringLiteral("Mx"), QStringLiteral("My"), QStringLiteral("Mz")
    }};
    const auto recordViolation = [&](int channel,
                                     const QString& metricCode,
                                     const QString& metricName,
                                     double value,
                                     double limit,
                                     const QString& unit){
        if(value <= limit || limit <= 0.0){
            return;
        }
        const double ratio = value / limit;
        if(ratio <= worstViolationRatio){
            return;
        }
        worstViolationRatio = ratio;
        worstViolationCode = QStringLiteral("%1_%2")
                .arg(channelNames[channel], metricCode);
        worstViolationDetail = QStringLiteral("%1%2=%3 %4，阈值=%5 %4，硬撤销阈值=%6 %4")
                .arg(channelNames[channel], metricName)
                .arg(value, 0, 'f', channel < 3 ? 5 : 6)
                .arg(unit)
                .arg(limit, 0, 'f', channel < 3 ? 5 : 6)
                .arg(limit * config_.releaseMultiplier, 0, 'f',
                     channel < 3 ? 5 : 6);
    };
    for(int channel = 0; channel < kFtSensorWrenchChannelCount; ++channel){
        FtSensorChannelStatistics& stats = result.channel[channel];
        stats.mean = sum[channel] / count;
        const double variance = std::max(0.0,
                sumSquared[channel] / count - stats.mean * stats.mean);
        stats.standardDeviation = std::sqrt(variance);
        stats.slopePerS = std::abs(denominator) > 1e-12 ?
                    (count * sumTY[channel] - sumT * sum[channel]) /
                        denominator : 0.0;
        stats.peakToPeak = maximum[channel] - minimum[channel];
        const bool forceChannel = channel < 3;
        const double standardDeviationLimit = forceChannel ?
                    config_.forceStdLimitN : config_.momentStdLimitNm;
        const double slopeLimit = forceChannel ?
                    config_.forceSlopeLimitNPerS : config_.momentSlopeLimitNmPerS;
        const double peakToPeakLimit = forceChannel ?
                    config_.forcePeakToPeakLimitN : config_.momentPeakToPeakLimitNm;
        const double absoluteSlope = std::abs(stats.slopePerS);
        const bool channelStable =
                stats.standardDeviation <= standardDeviationLimit &&
                absoluteSlope <= slopeLimit &&
                stats.peakToPeak <= peakToPeakLimit;
        const bool channelWithinReleaseLimits =
                stats.standardDeviation <= standardDeviationLimit * config_.releaseMultiplier &&
                absoluteSlope <= slopeLimit * config_.releaseMultiplier &&
                stats.peakToPeak <= peakToPeakLimit * config_.releaseMultiplier;
        channelsStable = channelsStable && channelStable;
        channelsWithinReleaseLimits = channelsWithinReleaseLimits &&
                channelWithinReleaseLimits;
        recordViolation(channel, QStringLiteral("std"), QStringLiteral("标准差"),
                        stats.standardDeviation, standardDeviationLimit,
                        forceChannel ? QStringLiteral("N") : QStringLiteral("N·m"));
        recordViolation(channel, QStringLiteral("slope"), QStringLiteral("斜率"),
                        absoluteSlope, slopeLimit,
                        forceChannel ? QStringLiteral("N/s") : QStringLiteral("N·m/s"));
        recordViolation(channel, QStringLiteral("ptp"), QStringLiteral("峰峰值"),
                        stats.peakToPeak, peakToPeakLimit,
                        forceChannel ? QStringLiteral("N") : QStringLiteral("N·m"));
    }
    const bool hardConditionsSatisfied = result.statisticsAvailable &&
            result.currentDataFresh &&
            result.effectiveRate >= config_.minimumEffectiveRate;
    result.stabilityConditionsSatisfied = hardConditionsSatisfied && channelsStable;
    result.stabilityWithinReleaseLimits = hardConditionsSatisfied &&
            channelsWithinReleaseLimits;
    result.candidateActive = candidateActive_;
    result.candidatePaused = candidatePaused_;
    result.candidateStableAccumulatedS =
            static_cast<double>(candidateStableAccumulatedUs_) / kUsPerSecond;
    if(candidatePaused_ && candidateInstabilityStartUs_ > 0 &&
            nowMonotonicUs >= candidateInstabilityStartUs_){
        result.candidatePauseElapsedS =
                static_cast<double>(nowMonotonicUs - candidateInstabilityStartUs_) /
                kUsPerSecond;
    }
    result.limitingConditionCode = worstViolationCode;
    result.limitingConditionDetail = worstViolationDetail;
    result.stable = result.minimumObservationSatisfied &&
            result.stabilityConditionsSatisfied &&
            result.candidateActive &&
            !result.candidatePaused &&
            result.candidateStableAccumulatedS >= config_.stableHoldS;
    if(!result.minimumObservationSatisfied){
        result.message = QStringLiteral("预热观察中，还需 %1 s")
                .arg(std::max(0.0, result.requiredObservationS - result.elapsedS),
                     0, 'f', 1);
    }
    else if(!result.statisticsAvailable){
        result.message = QStringLiteral("统计窗口尚未完整，候选稳定计时已撤销");
    }
    else if(!result.currentDataFresh){
        result.message = QStringLiteral("F/T数据已超时，候选稳定计时已撤销");
        result.limitingConditionCode = QStringLiteral("data_stale");
        result.limitingConditionDetail = QStringLiteral("最新有效传感器样本超过%1 s未更新")
                .arg(config_.maximumSampleAgeS, 0, 'f', 1);
    }
    else if(result.effectiveRate < config_.minimumEffectiveRate){
        result.message = QStringLiteral("Trace有效率=%1%，低于%2%，候选稳定计时已撤销")
                .arg(result.effectiveRate * 100.0, 0, 'f', 3)
                .arg(config_.minimumEffectiveRate * 100.0, 0, 'f', 3);
        result.limitingConditionCode = QStringLiteral("effective_rate");
        result.limitingConditionDetail = result.message;
    }
    else if(result.candidatePaused){
        result.message = QStringLiteral("候选判稳暂停：%1；宽限=%2/%3 s，已累计=%4/%5 s")
                .arg(result.limitingConditionDetail)
                .arg(result.candidatePauseElapsedS, 0, 'f', 1)
                .arg(config_.candidateInstabilityGraceS, 0, 'f', 1)
                .arg(result.candidateStableAccumulatedS, 0, 'f', 1)
                .arg(config_.stableHoldS, 0, 'f', 1);
    }
    else if(!result.stabilityConditionsSatisfied){
        result.message = QStringLiteral("滑窗稳定性尚未通过：%1；候选稳定计时已撤销")
                .arg(result.limitingConditionDetail.isEmpty() ?
                         QStringLiteral("统计量超过判稳阈值") :
                         result.limitingConditionDetail);
    }
    else if(!result.stable){
        result.message = QStringLiteral("稳定条件已满足，正在持续确认：%1/%2 s")
                .arg(result.candidateStableAccumulatedS, 0, 'f', 1)
                .arg(config_.stableHoldS, 0, 'f', 1);
    }
    else{
        result.message = QStringLiteral("预热与判稳通过，可以确认软件零点");
    }
    return result;
}

FtSensorPreheatStatus FtSensorPreheatMonitor::status(qint64 nowMonotonicUs) const
{
    FtSensorPreheatStatus result = calculateStatus(nowMonotonicUs);
    if(monitoring_){
        auto* self = const_cast<FtSensorPreheatMonitor*>(this);
        const auto resetCandidate = [self](){
            self->candidateStableAccumulatedUs_ = 0;
            self->candidateLastUpdateUs_ = 0;
            self->candidateInstabilityStartUs_ = 0;
            self->candidateActive_ = false;
            self->candidatePaused_ = false;
        };
        if(result.stabilityConditionsSatisfied){
            if(!self->candidateActive_){
                self->candidateActive_ = true;
                self->candidateStableAccumulatedUs_ = 0;
                self->candidateLastUpdateUs_ = nowMonotonicUs;
            }
            else if(self->candidatePaused_){
                // 暂停期间不累计；恢复后的下一段稳定时间从当前时刻继续。
                self->candidatePaused_ = false;
                self->candidateInstabilityStartUs_ = 0;
                self->candidateLastUpdateUs_ = nowMonotonicUs;
            }
            else if(self->candidateLastUpdateUs_ > 0 &&
                    nowMonotonicUs >= self->candidateLastUpdateUs_){
                self->candidateStableAccumulatedUs_ +=
                        nowMonotonicUs - self->candidateLastUpdateUs_;
                self->candidateLastUpdateUs_ = nowMonotonicUs;
            }
        }
        else if(self->candidateActive_ && result.stabilityWithinReleaseLimits){
            if(!self->candidatePaused_){
                self->candidatePaused_ = true;
                self->candidateInstabilityStartUs_ = nowMonotonicUs;
            }
            self->candidateLastUpdateUs_ = nowMonotonicUs;
            const qint64 graceUs = static_cast<qint64>(
                        config_.candidateInstabilityGraceS * kUsPerSecond);
            if(graceUs <= 0 || nowMonotonicUs -
                    self->candidateInstabilityStartUs_ > graceUs){
                resetCandidate();
            }
        }
        else{
            // 数据超时/有效率不足属于硬异常；统计量超过1.2倍释放阈值也
            // 说明当前不再是可容忍的边界抖动，二者都立即撤销候选。
            resetCandidate();
        }
        result = calculateStatus(nowMonotonicUs);
    }
    return result;
}

bool FtSensorPreheatMonitor::confirmZero(qint64 nowMonotonicUs,
                                         QString* errorMessage)
{
    const FtSensorPreheatStatus current = status(nowMonotonicUs);
    if(!monitoring_){
        if(errorMessage) *errorMessage = QStringLiteral("尚未开始预热监测");
        return false;
    }
    if(!current.stable){
        if(errorMessage){
            *errorMessage = QStringLiteral("预热判稳尚未完成：%1")
                    .arg(current.message.isEmpty() ?
                             QStringLiteral("必须同时满足最短观察、统计窗和连续稳定保持") :
                             current.message);
        }
        return false;
    }
    for(int channel = 0; channel < kFtSensorWrenchChannelCount; ++channel){
        zero_[channel] = current.channel[channel].mean;
    }
    preheatQualified_ = true;
    zeroValid_ = true;
    if(errorMessage) errorMessage->clear();
    return true;
}

bool FtSensorPreheatMonitor::restoreConfirmedZero(
        const std::array<double, kFtSensorWrenchChannelCount>& zero,
        QString* errorMessage)
{
    if(!monitoring_){
        if(errorMessage) *errorMessage = QStringLiteral("尚未开始F/T后台监测");
        return false;
    }
    for(double value : zero){
        if(!std::isfinite(value)){
            if(errorMessage) *errorMessage = QStringLiteral("缓存零漂包含非数值分量");
            return false;
        }
    }
    zero_ = zero;
    preheatQualified_ = true;
    zeroValid_ = true;
    if(errorMessage) errorMessage->clear();
    return true;
}

void FtSensorPreheatMonitor::clearZero()
{
    zero_.fill(0.0);
    preheatQualified_ = false;
    zeroValid_ = false;
}

std::array<double, kFtSensorWrenchChannelCount> FtSensorPreheatMonitor::zero() const
{
    return zero_;
}

std::array<double, kFtSensorWrenchChannelCount>
FtSensorPreheatMonitor::zeroCorrected(const FtSensorTraceSample& sample) const
{
    std::array<double, kFtSensorWrenchChannelCount> result{};
    for(int channel = 0; channel < kFtSensorWrenchChannelCount; ++channel){
        result[channel] = sample.value[channel] - (zeroValid_ ? zero_[channel] : 0.0);
    }
    return result;
}
