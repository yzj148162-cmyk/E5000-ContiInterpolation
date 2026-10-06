#include "forceinteractiontensionshadowworker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {
constexpr int kPoseCount = 6;

double wrappedAngle(double value)
{
    return std::atan2(std::sin(value), std::cos(value));
}
}

qint64 ForceInteractionTensionShadowWorker::nowUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
}

ForceInteractionTensionShadowWorker::ForceInteractionTensionShadowWorker()
    : thread_([this]{ run(); })
{
}

ForceInteractionTensionShadowWorker::~ForceInteractionTensionShadowWorker()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        request_.reset();
    }
    wake_.notify_one();
    if(thread_.joinable()) thread_.join();
}

bool ForceInteractionTensionShadowWorker::configure(
        const ForceInteractionRuntimeConfig& runtimeConfig,
        QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage) *errorMessage = message;
        return false;
    };
    if(runtimeConfig.stage != ForceInteractionRuntimeStage::StageD ||
            runtimeConfig.mechanicalMode !=
                ForceInteractionMechanicalMode::D1PhysicalCabled ||
            !runtimeConfig.translationOnly){
        return fail(QStringLiteral("M2当前仅允许阶段D1平动影子计算"));
    }
    if(!runtimeConfig.tensionControlMocapEnabled){
        return fail(QStringLiteral("M2需要启用Nokov在线张力外环反馈"));
    }
    const double maximumTensionN = *std::min_element(
                runtimeConfig.globalMaximumCableTensionN.cbegin(),
                runtimeConfig.globalMaximumCableTensionN.cend());
    FrozenConfig frozen;
    frozen.shadow = makeDefaultG302TranslationShadowConfig(
                runtimeConfig.rigidBody, runtimeConfig.kinematics,
                runtimeConfig.globalMinimumCableTensionN, maximumTensionN);
    QString validationError;
    if(!frozen.shadow.validate(&validationError)){
        return fail(validationError);
    }
    frozen.mocapTimeoutUs = runtimeConfig.tensionControlMocapTimeoutUs;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(executing_){
            return fail(QStringLiteral("M2上一影子计算尚未退出"));
        }
        ++epoch_;
        frozen.epoch = epoch_;
        config_ = std::move(frozen);
        configured_ = true;
        request_.reset();
        result_.reset();
    }
    if(errorMessage) errorMessage->clear();
    return true;
}

void ForceInteractionTensionShadowWorker::resetSession()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++epoch_;
    configured_ = false;
    request_.reset();
    result_.reset();
}

ForceInteractionTensionShadowWorker::SubmitResult
ForceInteractionTensionShadowWorker::submit(const Request& value)
{
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if(!lock.owns_lock()){
        return SubmitResult::LockContended;
    }
    if(stopping_ || !configured_ || value.epoch != epoch_){
        return SubmitResult::NotReady;
    }
    if(executing_ || request_){
        return SubmitResult::WorkerBusy;
    }
    if(result_){
        return SubmitResult::ResultPending;
    }
    executing_ = true;
    request_ = std::make_shared<Request>(value);
    wake_.notify_one();
    return SubmitResult::Accepted;
}

std::shared_ptr<const ForceInteractionTensionShadowWorker::Result>
ForceInteractionTensionShadowWorker::take()
{
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if(!lock.owns_lock() || !result_) return {};
    return std::move(result_);
}

quint64 ForceInteractionTensionShadowWorker::epoch() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return epoch_;
}

void ForceInteractionTensionShadowWorker::run()
{
    ForceInteractionPlatformState previousObserved;
    qint64 previousMocapUs = 0;
    quint64 previousMocapSequence = 0;
    quint64 activeEpoch = 0;
    ForceInteractionTensionShadow shadow;
    for(;;){
        std::shared_ptr<const Request> request;
        FrozenConfig config;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [this]{ return stopping_ || bool(request_); });
            if(stopping_) return;
            request = std::move(request_);
            config = config_;
        }
        auto result = std::make_shared<Result>();
        result->epoch = request->epoch;
        result->sourceTraceSequence = request->sourceTraceSequence;
        result->sourceTraceUs = request->sourceTraceUs;
        result->mocapSequence = request->mocapSequence;
        result->mocapSourceFrameSequence = request->mocapSourceFrameSequence;
        result->mocapReceivedUs = request->mocapReceivedUs;
        result->mocapAgeUs = request->submittedUs >= request->mocapReceivedUs ?
                    request->submittedUs - request->mocapReceivedUs : -1;
        result->startedUs = nowUs();
        if(activeEpoch != config.epoch){
            activeEpoch = config.epoch;
            previousObserved = {};
            previousMocapUs = 0;
            previousMocapSequence = 0;
            QString error;
            if(!shadow.configure(config.shadow, &error)){
                result->errorMessage = error;
            }
        }
        if(result->errorMessage.isEmpty()){
            if(!request->observed.poseValid || request->mocapSequence == 0 ||
                    request->mocapReceivedUs <= 0 ||
                    result->mocapAgeUs < 0 ||
                    result->mocapAgeUs > config.mocapTimeoutUs){
                result->errorMessage = QStringLiteral(
                            "M2 Nokov位姿无效或超时：帧龄=%1 us，上限=%2 us")
                        .arg(result->mocapAgeUs)
                        .arg(config.mocapTimeoutUs);
            }
            else if(previousMocapSequence > 0 &&
                    request->mocapSequence <= previousMocapSequence){
                result->errorMessage = QStringLiteral("M2 Nokov帧未推进");
            }
            else{
                ForceInteractionTensionShadowInput input;
                input.desired = request->desired;
                input.observed = request->observed;
                input.observed.twistValid = true;
                input.dtS = previousMocapUs > 0 ?
                            double(request->mocapReceivedUs - previousMocapUs) * 1.0e-6 :
                            config.shadow.outerPeriodS;
                if(!std::isfinite(input.dtS) || input.dtS <= 0.0){
                    result->errorMessage = QStringLiteral("M2 Nokov时间戳未递增");
                }
                else if(previousObserved.poseValid){
                    for(int i = 0; i < kPoseCount; ++i){
                        double delta = input.observed.pose[static_cast<size_t>(i)] -
                                previousObserved.pose[static_cast<size_t>(i)];
                        if(i >= 3) delta = wrappedAngle(delta);
                        input.observed.twist[static_cast<size_t>(i)] =
                                delta / input.dtS;
                    }
                }
                if(result->errorMessage.isEmpty()){
                    result->shadow = shadow.evaluate(input);
                    result->valid = result->shadow.valid;
                    result->errorMessage = result->shadow.errorMessage;
                    previousObserved = input.observed;
                    previousMocapUs = request->mocapReceivedUs;
                    previousMocapSequence = request->mocapSequence;
                }
            }
        }
        result->finishedUs = nowUs();
        result->expired = result->finishedUs > request->deadlineUs;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopping_) return;
            result_ = std::move(result);
            executing_ = false;
        }
    }
}
