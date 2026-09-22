#ifndef FTSENSORPREHEATMONITOR_H
#define FTSENSORPREHEATMONITOR_H

#include "ftsensortypes.h"

#include <array>
#include <deque>
#include <vector>

#include <QString>

struct FtSensorStabilityConfig
{
    double normalMinimumObservationS = 15.0 * 60.0;
    double previouslyPoweredMinimumObservationS = 3.0 * 60.0;
    double statisticsWindowS = 30.0;
    double stableHoldS = 10.0;
    double forceStdLimitN = 0.080;
    double momentStdLimitNm = 0.0020;
    double forceSlopeLimitNPerS = 0.0040;
    double momentSlopeLimitNmPerS = 0.000050;
    double forcePeakToPeakLimitN = 0.80;
    double momentPeakToPeakLimitNm = 0.080;
    double minimumEffectiveRate = 0.99;
    double maximumSampleAgeS = 1.0;
    // 候选判稳已开始后，轻微统计波动只暂停累计；持续超时或超过
    // releaseMultiplier 倍阈值才撤销候选。数据超时等硬异常不使用宽限。
    double candidateInstabilityGraceS = 1.0;
    double releaseMultiplier = 1.2;
};

struct FtSensorChannelStatistics
{
    double mean = 0.0;
    double standardDeviation = 0.0;
    double slopePerS = 0.0;
    double peakToPeak = 0.0;
};

struct FtSensorPreheatStatus
{
    bool monitoring = false;
    bool previouslyPowered = false;
    bool minimumObservationSatisfied = false;
    bool statisticsAvailable = false;
    bool currentDataFresh = false;
    bool stabilityConditionsSatisfied = false;
    bool stabilityWithinReleaseLimits = false;
    bool candidateActive = false;
    bool candidatePaused = false;
    bool stable = false;
    // 只在完整预热判稳后确认零点时锁存。真实施力会使实时
    // 滑窗不再“稳定”，但不应撤销已取得的阶段C预热资格。
    bool preheatQualified = false;
    bool zeroValid = false;
    double elapsedS = 0.0;
    double requiredObservationS = 0.0;
    double windowSpanS = 0.0;
    double effectiveRate = 0.0;
    double newestSampleAgeS = 0.0;
    double candidateStableAccumulatedS = 0.0;
    double candidatePauseElapsedS = 0.0;
    quint64 acceptedSamples = 0;
    quint64 rejectedSamples = 0;
    quint64 missingSensorSamples = 0;
    quint64 receivedTraceSamples = 0;
    quint64 completeTraceSamples = 0;
    quint64 duplicateSensorSamples = 0;
    quint64 missingTraceFrames = 0;
    std::array<FtSensorChannelStatistics, kFtSensorWrenchChannelCount> channel{};
    std::array<double, kFtSensorWrenchChannelCount> zero{};
    QString limitingConditionCode;
    QString limitingConditionDetail;
    QString message;
};

// 六维 F/T 专用的纯计算模块。它不访问 UI、控制卡或电机，只接收已经按
// Trace 帧解码的工程量样本，负责滑窗统计、预热判稳和软件零漂确认。
class FtSensorPreheatMonitor
{
public:
    static bool runTraceEpochSelfChecks(QString* errorMessage = nullptr);
    void configure(const FtSensorStabilityConfig& config);
    FtSensorStabilityConfig configuration() const;
    void start(bool previouslyPowered, qint64 startMonotonicUs);
    void stop();
    void reset();
    void ingest(const std::vector<FtSensorTraceSample>& samples);
    // 板卡重配Trace对象后帧序号会从零重新开始。只重置Trace连续性基准，
    // 保留预热计时、统计窗口、已确认资格和软件零点。
    void beginTraceEpoch();
    FtSensorPreheatStatus status(qint64 nowMonotonicUs) const;
    bool confirmZero(qint64 nowMonotonicUs, QString* errorMessage = nullptr);
    // 仅恢复由操作员在同一程序进程内确认过的软件零漂。恢复后本轮预热
    // 资格一并有效；PDO完整性、实时样本新鲜度和状态码仍由上层单独检查。
    bool restoreConfirmedZero(
            const std::array<double, kFtSensorWrenchChannelCount>& zero,
            QString* errorMessage = nullptr);
    void clearZero();
    std::array<double, kFtSensorWrenchChannelCount> zero() const;
    std::array<double, kFtSensorWrenchChannelCount> zeroCorrected(
            const FtSensorTraceSample& sample) const;

private:
    FtSensorPreheatStatus calculateStatus(qint64 nowMonotonicUs) const;
    void trim(qint64 newestMonotonicUs);

    FtSensorStabilityConfig config_;
    std::deque<FtSensorTraceSample> window_;
    std::array<double, kFtSensorWrenchChannelCount> zero_{};
    qint64 startMonotonicUs_ = 0;
    qint64 candidateStableAccumulatedUs_ = 0;
    qint64 candidateLastUpdateUs_ = 0;
    qint64 candidateInstabilityStartUs_ = 0;
    quint64 acceptedSamples_ = 0;
    quint64 rejectedSamples_ = 0;
    quint64 receivedTraceSamples_ = 0;
    quint64 completeTraceSamples_ = 0;
    quint64 duplicateSensorSamples_ = 0;
    quint64 missingTraceFrames_ = 0;
    quint32 lastSampleCounter_ = 0;
    bool lastSampleCounterValid_ = false;
    quint32 lastTraceFrameSequence_ = 0;
    bool lastTraceFrameSequenceValid_ = false;
    bool monitoring_ = false;
    bool previouslyPowered_ = false;
    bool candidateActive_ = false;
    bool candidatePaused_ = false;
    bool preheatQualified_ = false;
    bool zeroValid_ = false;
};

#endif // FTSENSORPREHEATMONITOR_H
