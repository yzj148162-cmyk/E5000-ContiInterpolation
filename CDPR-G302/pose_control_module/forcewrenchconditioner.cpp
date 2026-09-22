#include "forcewrenchconditioner.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

bool finiteWrench(const ForceInteractionVector6& values)
{
    return std::all_of(values.cbegin(), values.cend(), [](double value){
        return std::isfinite(value);
    });
}
}

bool ForceWrenchConditioningConfig::validate(
        double samplePeriodS, QString* errorMessage) const
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(!std::isfinite(samplePeriodS) || samplePeriodS <= 0.0){
        return fail(QStringLiteral("F/T输入调理采样周期无效"));
    }
    if(!std::isfinite(lowPassCutoffHz) || lowPassCutoffHz < 0.0){
        return fail(QStringLiteral("F/T低通截止频率无效"));
    }
    const double sampleFrequencyHz = 1.0 / samplePeriodS;
    if(lowPassEnabled && (lowPassCutoffHz <= 0.0 ||
                          lowPassCutoffHz > sampleFrequencyHz * 0.45)){
        return fail(QStringLiteral(
                    "F/T低通截止频率必须大于0且不超过控制采样频率的45%%（当前上限%1 Hz）")
                    .arg(sampleFrequencyHz * 0.45, 0, 'f', 3));
    }
    const bool thresholdsFinite =
            std::isfinite(forceStartThresholdN) &&
            std::isfinite(forceReleaseThresholdN) &&
            std::isfinite(torqueStartThresholdNm) &&
            std::isfinite(torqueReleaseThresholdNm);
    if(!thresholdsFinite || forceStartThresholdN < 0.0 ||
            forceReleaseThresholdN < 0.0 ||
            torqueStartThresholdNm < 0.0 ||
            torqueReleaseThresholdNm < 0.0){
        return fail(QStringLiteral("F/T启动/释放阈值必须是有限非负数"));
    }
    if(forceReleaseThresholdN > forceStartThresholdN ||
            torqueReleaseThresholdNm > torqueStartThresholdNm){
        return fail(QStringLiteral("F/T释放阈值不得大于对应启动阈值"));
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool ForceWrenchConditioner::configure(
        const ForceWrenchConditioningConfig& config,
        double samplePeriodS,
        QString* errorMessage)
{
    if(!config.validate(samplePeriodS, errorMessage)){
        configured_ = false;
        return false;
    }
    config_ = config;
    alpha_ = config_.lowPassEnabled ?
                1.0 - std::exp(-2.0 * kPi * config_.lowPassCutoffHz *
                               samplePeriodS) : 1.0;
    alpha_ = std::clamp(alpha_, 0.0, 1.0);
    configured_ = true;
    reset();
    return true;
}

void ForceWrenchConditioner::reset()
{
    filtered_.fill(0.0);
    filterInitialized_ = false;
    forceActive_ = false;
    torqueActive_ = false;
}

double ForceWrenchConditioner::norm3(
        const ForceInteractionVector6& values, int offset)
{
    double squared = 0.0;
    for(int index = 0; index < 3; ++index){
        const double value = values[static_cast<size_t>(offset + index)];
        squared += value * value;
    }
    return std::sqrt(squared);
}

void ForceWrenchConditioner::updateGate(
        double norm, double startThreshold, double releaseThreshold,
        bool& active)
{
    if(startThreshold <= 0.0){
        active = true;
        return;
    }
    if(active){
        if(norm <= releaseThreshold){
            active = false;
        }
    }
    else if(norm >= startThreshold){
        active = true;
    }
}

ForceWrenchConditioningResult ForceWrenchConditioner::process(
        const ForceInteractionVector6& platformWrench)
{
    ForceWrenchConditioningResult result;
    result.unfiltered = platformWrench;
    if(!configured_ || !finiteWrench(platformWrench)){
        return result;
    }
    if(!filterInitialized_){
        filtered_ = platformWrench;
        filterInitialized_ = true;
    }
    else{
        for(size_t index = 0; index < filtered_.size(); ++index){
            filtered_[index] += alpha_ * (platformWrench[index] - filtered_[index]);
        }
    }
    result.filtered = filtered_;
    result.forceNormN = norm3(filtered_, 0);
    result.torqueNormNm = norm3(filtered_, 3);
    updateGate(result.forceNormN,
               config_.forceStartThresholdN,
               config_.forceReleaseThresholdN,
               forceActive_);
    updateGate(result.torqueNormNm,
               config_.torqueStartThresholdNm,
               config_.torqueReleaseThresholdNm,
               torqueActive_);
    result.output = filtered_;
    if(!forceActive_){
        result.output[0] = 0.0;
        result.output[1] = 0.0;
        result.output[2] = 0.0;
    }
    if(!torqueActive_){
        result.output[3] = 0.0;
        result.output[4] = 0.0;
        result.output[5] = 0.0;
    }
    result.forceActive = forceActive_;
    result.torqueActive = torqueActive_;
    result.valid = finiteWrench(result.output);
    return result;
}

bool ForceWrenchConditioner::runSelfChecks(QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    ForceWrenchConditioningConfig config;
    config.lowPassEnabled = false;
    config.forceStartThresholdN = 1.0;
    config.forceReleaseThresholdN = 0.5;
    config.torqueStartThresholdNm = 0.2;
    config.torqueReleaseThresholdNm = 0.1;
    ForceWrenchConditioner conditioner;
    QString configureError;
    if(!conditioner.configure(config, 0.005, &configureError)){
        return fail(QStringLiteral("输入调理自检配置失败：%1").arg(configureError));
    }
    ForceInteractionVector6 wrench{};
    wrench[0] = 0.9;
    auto result = conditioner.process(wrench);
    if(!result.valid || result.forceActive || result.output[0] != 0.0){
        return fail(QStringLiteral("力启动阈值自检失败"));
    }
    wrench[0] = 1.1;
    result = conditioner.process(wrench);
    if(!result.forceActive || std::fabs(result.output[0] - 1.1) > 1.0e-12){
        return fail(QStringLiteral("力启动锁存自检失败"));
    }
    wrench[0] = 0.7;
    result = conditioner.process(wrench);
    if(!result.forceActive){
        return fail(QStringLiteral("力迟滞保持自检失败"));
    }
    wrench[0] = 0.4;
    result = conditioner.process(wrench);
    if(result.forceActive || result.output[0] != 0.0){
        return fail(QStringLiteral("力释放阈值自检失败"));
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}
