#include "forceinteractiontensionshadowworker.h"

#include "winchcompensation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {
constexpr int kAxisCount = 8;
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
    frozen.kinematics = runtimeConfig.kinematics;
    frozen.physicalWorkspace = runtimeConfig.physicalWorkspace;
    frozen.motorUnitPerRadian = runtimeConfig.motorUnitPerRadian;
    frozen.initialPoseMmRad.resize(kPoseCount);
    for(int i = 0; i < kPoseCount; ++i){
        frozen.initialPoseMmRad[static_cast<size_t>(i)] = i < 3 ?
                    runtimeConfig.initialState.pose[static_cast<size_t>(i)] * 1000.0 :
                    runtimeConfig.initialState.pose[static_cast<size_t>(i)];
    }
    CompensatedCableKinematics kinematics;
    if(!kinematics.initialize(frozen.kinematics,
                              {frozen.initialPoseMmRad}, {},
                              &validationError)){
        return fail(validationError);
    }
    frozen.referenceCableLengthMm = kinematics.cableLengthsForPose(
                {frozen.initialPoseMmRad}, &validationError);
    if(frozen.referenceCableLengthMm.size() != kAxisCount){
        return fail(validationError.isEmpty() ?
                    QStringLiteral("M2初始参考绳长不是完整八轴数据") :
                    validationError);
    }
    for(double value : frozen.motorUnitPerRadian){
        if(!std::isfinite(value) || std::abs(value) < 1.0e-12){
            return fail(QStringLiteral("M2电机角度换算参数无效"));
        }
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(busy_){
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

bool ForceInteractionTensionShadowWorker::submit(const Request& value)
{
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if(!lock.owns_lock() || stopping_ || !configured_ || busy_ ||
            value.epoch != epoch_){
        return false;
    }
    busy_ = true;
    request_ = std::make_shared<Request>(value);
    wake_.notify_one();
    return true;
}

std::shared_ptr<const ForceInteractionTensionShadowWorker::Result>
ForceInteractionTensionShadowWorker::take()
{
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if(!lock.owns_lock() || !result_) return {};
    auto result = std::move(result_);
    busy_ = false;
    return result;
}

quint64 ForceInteractionTensionShadowWorker::epoch() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return epoch_;
}

void ForceInteractionTensionShadowWorker::run()
{
    ForceInteractionPlatformState previousObserved;
    qint64 previousTraceUs = 0;
    quint64 activeEpoch = 0;
    ForwardKinematicsSolver solver;
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
        result->startedUs = nowUs();
        if(activeEpoch != config.epoch){
            activeEpoch = config.epoch;
            previousObserved = {};
            previousTraceUs = 0;
            solver.setInitialPose(config.initialPoseMmRad);
            QString error;
            if(!shadow.configure(config.shadow, &error)){
                result->errorMessage = error;
            }
        }
        if(result->errorMessage.isEmpty()){
            ForwardKinematicsSolver::Request solveRequest;
            solveRequest.anchorPos = config.kinematics.anchorCableCoordinate;
            solveRequest.contactPointLocal =
                    config.kinematics.endCableContactPos.front();
            solveRequest.pulleyRadius = config.kinematics.pulleyRadiusMm;
            solveRequest.initialPose = solver.initialPose();
            solveRequest.keepRotation = true;
            solveRequest.enforcePhysicalWorkspace = true;
            PhysicalWorkspaceBoundary boundary(config.physicalWorkspace);
            const auto lower = boundary.solverLowerBounds();
            const auto upper = boundary.solverUpperBounds();
            solveRequest.poseLowerBounds.assign(lower.begin(), lower.end());
            solveRequest.poseUpperBounds.assign(upper.begin(), upper.end());
            for(int axis = 0; axis < kAxisCount; ++axis){
                const double relative = request->safetyRelativePosition[axis] -
                        request->actualStartSafetyRelativePosition[axis];
                const double motorTheta = relative /
                        config.motorUnitPerRadian[axis];
                const double scale = std::abs(
                            config.kinematics.cableMotorScaleRadPerMm[axis]);
                const double platformDelta =
                        WinchCompensation::platformDeltaFromMotorTheta(
                            config.kinematics.winchConfig[axis],
                            motorTheta, scale);
                solveRequest.cableLength.push_back(
                            config.referenceCableLengthMm[axis] - platformDelta);
            }
            result->forwardKinematics = solver.solve(solveRequest);
            if(!result->forwardKinematics.success ||
                    result->forwardKinematics.pose.size() < kPoseCount){
                result->errorMessage = QStringLiteral("M2正运动学未收敛");
            }
            else{
                ForceInteractionTensionShadowInput input;
                input.desired = request->desired;
                input.observed.poseValid = true;
                input.observed.twistValid = true;
                for(int i = 0; i < kPoseCount; ++i){
                    input.observed.pose[static_cast<size_t>(i)] = i < 3 ?
                                result->forwardKinematics.pose[i] * 1.0e-3 :
                                result->forwardKinematics.pose[i];
                }
                input.dtS = previousTraceUs > 0 ?
                            double(request->sourceTraceUs - previousTraceUs) * 1.0e-6 :
                            config.shadow.outerPeriodS;
                if(previousObserved.poseValid && input.dtS > 0.0){
                    for(int i = 0; i < kPoseCount; ++i){
                        double delta = input.observed.pose[static_cast<size_t>(i)] -
                                previousObserved.pose[static_cast<size_t>(i)];
                        if(i >= 3) delta = wrappedAngle(delta);
                        input.observed.twist[static_cast<size_t>(i)] =
                                delta / input.dtS;
                    }
                }
                result->shadow = shadow.evaluate(input);
                result->valid = result->shadow.valid;
                result->errorMessage = result->shadow.errorMessage;
                previousObserved = input.observed;
                previousTraceUs = request->sourceTraceUs;
            }
        }
        result->finishedUs = nowUs();
        result->expired = result->finishedUs > request->deadlineUs;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopping_) return;
            result_ = std::move(result);
        }
    }
}
