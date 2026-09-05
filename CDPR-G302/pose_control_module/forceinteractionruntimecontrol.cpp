#include "forceinteractionruntimecontrol.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

bool finiteArray(const OnlineVelocityAxisArray& values)
{
    return std::all_of(values.begin(), values.end(), [](double value){
        return std::isfinite(value);
    });
}

double clampValue(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

double vectorNorm3(const ForceInteractionVector6& values, int offset = 0)
{
    return std::sqrt(values[static_cast<size_t>(offset)] *
                     values[static_cast<size_t>(offset)] +
                     values[static_cast<size_t>(offset + 1)] *
                     values[static_cast<size_t>(offset + 1)] +
                     values[static_cast<size_t>(offset + 2)] *
                     values[static_cast<size_t>(offset + 2)]);
}

bool worldOmegaToZyxEulerRate(const ForceInteractionVector6& pose,
                              const ForceInteractionVector6& twist,
                              ForceInteractionVector3& eulerRate)
{
    const double pitch = pose[4];
    const double yaw = pose[5];
    const double cosinePitch = std::cos(pitch);
    if(std::fabs(cosinePitch) <= 1.0e-6){
        return false;
    }
    const double projected = std::cos(yaw) * twist[3] +
            std::sin(yaw) * twist[4];
    eulerRate[0] = projected / cosinePitch;
    eulerRate[1] = -std::sin(yaw) * twist[3] +
            std::cos(yaw) * twist[4];
    eulerRate[2] = twist[5] + std::tan(pitch) * projected;
    return std::all_of(eulerRate.cbegin(), eulerRate.cend(),
                       [](double value){ return std::isfinite(value); });
}

} // namespace

bool ForceInteractionRuntimeConfig::validate(QString* errorMessage) const
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(machineTemplateName.compare(QStringLiteral("G302"), Qt::CaseInsensitive) != 0){
        return fail(QStringLiteral("阶段B仅允许G302模板"));
    }
    if(periodUs < 1000 || periodUs > 20000){
        return fail(QStringLiteral("控制周期必须位于1~20 ms"));
    }
    if(!initialState.poseValid || rigidBody.massKg <= 0.0){
        return fail(QStringLiteral("初始位姿或刚体质量无效"));
    }
    QString workspaceError;
    if(!physicalWorkspace.validate(&workspaceError) ||
            !workspaceSafety.validate(&workspaceError)){
        return fail(QStringLiteral("物理工作空间配置无效：%1")
                    .arg(workspaceError));
    }
    PhysicalWorkspaceBoundary initialBoundary;
    if(!initialBoundary.configure(physicalWorkspace, &workspaceError)){
        return fail(QStringLiteral("物理工作空间配置无效：%1")
                    .arg(workspaceError));
    }
    std::array<double, 6> initialPoseMmRad{};
    for(int dimension = 0; dimension < 3; ++dimension){
        initialPoseMmRad[static_cast<size_t>(dimension)] =
                initialState.pose[static_cast<size_t>(dimension)] * 1000.0;
    }
    for(int dimension = 3; dimension < 6; ++dimension){
        initialPoseMmRad[static_cast<size_t>(dimension)] =
                initialState.pose[static_cast<size_t>(dimension)];
    }
    const PhysicalWorkspaceBoundaryResult initialWorkspace =
            initialBoundary.evaluatePose(initialPoseMmRad);
    if(initialWorkspace.action != PhysicalWorkspaceAction::Safe){
        return fail(QStringLiteral("初始位姿不满足动平台几何硬边界：%1")
                    .arg(initialWorkspace.reason));
    }
    if(!finiteArray(motorUnitPerRadian) || velocityLimit <= 0.0 ||
            followingErrorLimit <= 0.0 ||
            correctionVelocityLimit < 0.0 || integralLimit < 0.0 ||
            onlineChangeTimeS < 0.0 || traceTimeoutUs <= 0 ||
            brakingStopVelocityMmPerSec < 0.0){
        return fail(QStringLiteral("PID、运动限制或Trace参数无效"));
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(std::fabs(motorUnitPerRadian[axis]) <= 1.0e-12 ||
                motorPositionMinimum[axis] >= motorPositionMaximum[axis]){
            return fail(QStringLiteral("轴%1单位换算或位置边界无效").arg(axis));
        }
    }
    return true;
}

bool ForceInteractionRuntimeControl::prepare(
        const ForceInteractionRuntimeConfig& config,
        QString* errorMessage)
{
    if(isActive()){
        if(errorMessage){
            *errorMessage = QStringLiteral("阶段B正在运行");
        }
        return false;
    }
    QString error;
    if(!config.validate(&error) ||
            !wrenchSource_.configure(config.wrenchProfile,
                                     config.periodUs / 1000000.0,
                                     &error) ||
            !dynamics_.configure(config.rigidBody, config.newmark, &error) ||
            !dynamics_.reset(config.initialState, &error) ||
            !physicalBoundary_.configure(config.physicalWorkspace, &error) ||
            !kinematics_.initialize(config.kinematics,
                                    {{config.initialState.pose[0] * 1000.0,
                                      config.initialState.pose[1] * 1000.0,
                                      config.initialState.pose[2] * 1000.0,
                                      config.initialState.pose[3],
                                      config.initialState.pose[4],
                                      config.initialState.pose[5]}},
                                    {}, &error)){
        if(errorMessage){
            *errorMessage = error;
        }
        return false;
    }
    config_ = config;
    wrenchTransformer_ = std::make_unique<WrenchTransformer>(config.sensorTransform);
    kinematicsState_ = kinematics_.initialState();
    status_ = ForceInteractionRuntimeStatus{};
    status_.state = ForceInteractionRuntimeStatus::State::Prepared;
    status_.message = QStringLiteral("阶段B已准备");
    status_.desiredState = config.initialState;
    actualStartCaptured_ = false;
    previousErrorValid_ = false;
    lastFrameSequenceValid_ = false;
    brakingState_ = ForceInteractionPlatformState{};
    controlledStopReason_.clear();
    integral_.fill(0.0);
    previousError_.fill(0.0);
    return true;
}

bool ForceInteractionRuntimeControl::start(qint64 nowUs, QString* errorMessage)
{
    if(status_.state != ForceInteractionRuntimeStatus::State::Prepared){
        if(errorMessage){
            *errorMessage = QStringLiteral("请先准备阶段B");
        }
        return false;
    }
    recorder_ = std::make_unique<ForceInteractionRunRecorder>();
    ForceInteractionRunMetadata metadata;
    metadata.stage = QStringLiteral("stage_b");
    metadata.sourceName = wrenchSource_.summary();
    metadata.machineTemplateName = config_.machineTemplateName;
    metadata.controlPeriodS = config_.periodUs / 1000000.0;
    metadata.plannedDurationS = config_.maximumTestDurationS;
    QString recordError;
    if(!recorder_->begin(config_.recordingDirectory, metadata,
                         &status_.recordFile, &recordError)){
        recorder_.reset();
        if(errorMessage){
            *errorMessage = QStringLiteral("阶段B记录器启动失败：%1").arg(recordError);
        }
        return false;
    }
    waitStartUs_ = nowUs;
    lastGoodTraceUs_ = 0;
    nextDueUs_ = nowUs;
    status_.state = ForceInteractionRuntimeStatus::State::WaitingForTrace;
    status_.message = QStringLiteral("等待新鲜完整的八轴Trace帧");
    return true;
}

bool ForceInteractionRuntimeControl::feedbackReady(
        const ForceInteractionRuntimeFeedback& feedback) const
{
    return feedback.fromTrace && feedback.frameSequenceValid &&
            feedback.timingReliable && feedback.fifoCaughtUp &&
            !feedback.traceLost && feedback.frameCount > 0 &&
            feedback.newestFrameAgeUs >= 0 &&
            feedback.newestFrameAgeUs <= config_.traceTimeoutUs &&
            finiteArray(feedback.actualPosition) &&
            finiteArray(feedback.actualVelocity);
}

bool ForceInteractionRuntimeControl::requestControlledStop(
        const QString& reason, bool experimentFailure,
        ForceInteractionControlledStopCause cause)
{
    if(status_.state == ForceInteractionRuntimeStatus::State::Braking){
        status_.experimentValid = status_.experimentValid && !experimentFailure;
        return true;
    }
    if(status_.state != ForceInteractionRuntimeStatus::State::Running){
        return false;
    }
    brakingState_ = status_.desiredState;
    controlledStopReason_ = reason.isEmpty() ?
                QStringLiteral("请求受控制动") : reason;
    status_.experimentValid = !experimentFailure;
    status_.safetyStopReason = experimentFailure ? controlledStopReason_ : QString{};
    status_.controlledStopCause = cause;
    status_.state = ForceInteractionRuntimeStatus::State::Braking;
    status_.message = QStringLiteral("协同减速中：%1").arg(controlledStopReason_);
    return true;
}

ForceInteractionPlatformState
ForceInteractionRuntimeControl::advanceBrakingState(
        bool& stopped, QString* errorMessage)
{
    ForceInteractionPlatformState next = brakingState_;
    const double dt = config_.periodUs / 1000000.0;
    const double linearSpeedMPerSec = vectorNorm3(brakingState_.twist);
    const double angularSpeedRadPerSec = vectorNorm3(brakingState_.twist, 3);

    // a 的外部单位为 mm/s^2。转动时用连接点最大半径折算为边缘线速度，
    // 再对六维速度统一缩放，避免各自由度分别截断破坏协同运动方向。
    double maximumRadiusM = 0.0;
    for(const auto& point : config_.physicalWorkspace.platformPointsLocalMm){
        const double radiusMm = std::sqrt(point[0] * point[0] +
                                          point[1] * point[1] +
                                          point[2] * point[2]);
        maximumRadiusM = std::max(maximumRadiusM, radiusMm / 1000.0);
    }
    const double equivalentSpeedMPerSec = linearSpeedMPerSec +
            angularSpeedRadPerSec * maximumRadiusM;
    const double stopThresholdMPerSec =
            config_.brakingStopVelocityMmPerSec / 1000.0;
    const double decelerationMPerSec2 =
            config_.workspaceSafety.stoppingDecelerationMmPerSec2 / 1000.0;
    const double nextEquivalentSpeed = std::max(
                0.0, equivalentSpeedMPerSec - decelerationMPerSec2 * dt);
    const double scale = equivalentSpeedMPerSec > 1.0e-12 ?
                nextEquivalentSpeed / equivalentSpeedMPerSec : 0.0;

    const ForceInteractionVector6 oldTwist = brakingState_.twist;
    for(int dimension = 0; dimension < 3; ++dimension){
        next.twist[static_cast<size_t>(dimension)] =
                oldTwist[static_cast<size_t>(dimension)] * scale;
        next.acceleration[static_cast<size_t>(dimension)] =
                (next.twist[static_cast<size_t>(dimension)] -
                 oldTwist[static_cast<size_t>(dimension)]) / dt;
        next.pose[static_cast<size_t>(dimension)] +=
                0.5 * (oldTwist[static_cast<size_t>(dimension)] +
                       next.twist[static_cast<size_t>(dimension)]) * dt;
    }
    if(config_.translationOnly){
        for(int dimension = 3; dimension < kForceInteractionDofCount; ++dimension){
            next.pose[static_cast<size_t>(dimension)] =
                    brakingState_.pose[static_cast<size_t>(dimension)];
            next.twist[static_cast<size_t>(dimension)] = 0.0;
            next.acceleration[static_cast<size_t>(dimension)] = 0.0;
        }
    }
    else{
        ForceInteractionVector3 oldEulerRate{};
        ForceInteractionVector6 scaledTwist = oldTwist;
        for(int dimension = 3; dimension < 6; ++dimension){
            scaledTwist[static_cast<size_t>(dimension)] =
                    oldTwist[static_cast<size_t>(dimension)] * scale;
        }
        ForceInteractionVector3 newEulerRate{};
        if(!worldOmegaToZyxEulerRate(brakingState_.pose, oldTwist,
                                     oldEulerRate) ||
                !worldOmegaToZyxEulerRate(brakingState_.pose, scaledTwist,
                                          newEulerRate)){
            stopped = false;
            next.poseValid = false;
            if(errorMessage){
                *errorMessage = QStringLiteral(
                            "受控制动接近ZYX欧拉角奇异位姿，无法可靠积分姿态");
            }
            return next;
        }
        for(int dimension = 3; dimension < 6; ++dimension){
            const size_t offset = static_cast<size_t>(dimension);
            next.twist[offset] = scaledTwist[offset];
            next.acceleration[offset] =
                    (scaledTwist[offset] - oldTwist[offset]) / dt;
            next.pose[offset] += 0.5 *
                    (oldEulerRate[static_cast<size_t>(dimension - 3)] +
                     newEulerRate[static_cast<size_t>(dimension - 3)]) * dt;
        }
    }
    next.poseValid = true;
    next.twistValid = true;
    next.accelerationValid = true;
    brakingState_ = next;
    stopped = nextEquivalentSpeed <= stopThresholdMPerSec;
    if(stopped){
        brakingState_.twist.fill(0.0);
        brakingState_.acceleration.fill(0.0);
        next = brakingState_;
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return next;
}

ForceInteractionRuntimeStep ForceInteractionRuntimeControl::step(
        const ForceInteractionRuntimeFeedback& feedback,
        qint64 nowUs)
{
    ForceInteractionRuntimeStep output;
    if(!isActive()){
        return output;
    }
    const bool ready = feedbackReady(feedback);
    const bool fresh = ready &&
            (!lastFrameSequenceValid_ ||
             feedback.logicalFrameSequence > lastFrameSequence_);
    if(fresh){
        lastGoodTraceUs_ = nowUs;
    }
    else{
        const qint64 freshnessAnchorUs = lastGoodTraceUs_ > 0 ?
                    lastGoodTraceUs_ : waitStartUs_;
        if(freshnessAnchorUs > 0 &&
                nowUs - freshnessAnchorUs > config_.traceTimeoutUs){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral(
                        "阶段B可靠Trace超时：fromTrace=%1，序号有效=%2，时序可靠=%3，FIFO已追平=%4，丢帧=%5，帧龄=%6 us，逻辑序号=%7")
                    .arg(feedback.fromTrace ? 1 : 0)
                    .arg(feedback.frameSequenceValid ? 1 : 0)
                    .arg(feedback.timingReliable ? 1 : 0)
                    .arg(feedback.fifoCaughtUp ? 1 : 0)
                    .arg(feedback.traceLost ? 1 : 0)
                    .arg(feedback.newestFrameAgeUs)
                    .arg(feedback.logicalFrameSequence);
        }
        return output;
    }
    if(feedback.traceSamplePeriodUs <= 0 ||
            config_.periodUs % feedback.traceSamplePeriodUs != 0){
        output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
        output.reason = QStringLiteral(
                    "阶段B控制周期%1 us不是Trace采样周期%2 us的整数倍")
                .arg(config_.periodUs)
                .arg(feedback.traceSamplePeriodUs);
        return output;
    }
    if(!actualStartCaptured_){
        // Trace配置及FIFO追平发生在启动请求之后；第一帧可靠反馈才是模型时间零点，
        // 不能把等待Trace的时间误计为控制漏周期。
        nextDueUs_ = nowUs;
    }
    if(nowUs < nextDueUs_){
        return output;
    }
    if(nowUs - nextDueUs_ >= config_.periodUs){
        status_.missedCycleCount += static_cast<quint64>((nowUs - nextDueUs_) / config_.periodUs);
    }
    do{
        nextDueUs_ += config_.periodUs;
    }while(nextDueUs_ <= nowUs);
    lastFrameSequence_ = feedback.logicalFrameSequence;
    lastFrameSequenceValid_ = true;

    if(!actualStartCaptured_){
        actualStartPosition_ = feedback.actualPosition;
        lastReferencePosition_ = actualStartPosition_;
        status_.actualStartPosition = actualStartPosition_;
        actualStartCaptured_ = true;
        status_.state = ForceInteractionRuntimeStatus::State::Running;
        status_.message = QStringLiteral("阶段B运行中");
    }

    const double elapsedS = (status_.stepCount + 1) * config_.periodUs / 1000000.0;
    QElapsedTimer calculationTimer;
    calculationTimer.start();
    ForceInteractionFrameStamp stamp;
    stamp.traceSequence = feedback.logicalFrameSequence;
    stamp.traceTimeUs = static_cast<qint64>(feedback.logicalFrameSequence) *
            std::max(1, feedback.traceSamplePeriodUs);
    stamp.hostMonotonicTimeUs = feedback.monotonicUs;
    stamp.traceValid = true;
    stamp.valid = true;
    ForceInteractionWrenchSample sensorSample;
    ForceInteractionWrenchSample platformSample;
    sensorSample.stamp = stamp;
    platformSample.stamp = stamp;
    CdprDynamicsStepResult dynamicsResult;
    ForceInteractionPlatformState desired;
    bool braking = status_.state == ForceInteractionRuntimeStatus::State::Braking;
    if(!braking && config_.maximumTestDurationS > 0.0 &&
            elapsedS > config_.maximumTestDurationS + 1.0e-12){
        requestControlledStop(
                    QStringLiteral("达到最长模拟调试时间"), false,
                    ForceInteractionControlledStopCause::DurationReached);
        braking = true;
    }

    if(braking){
        bool stopped = false;
        QString brakingError;
        desired = advanceBrakingState(stopped, &brakingError);
        if(!brakingError.isEmpty()){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = brakingError;
            return output;
        }
        if(stopped){
            output.action = ForceInteractionRuntimeStep::Action::NormalStop;
            output.reason = QStringLiteral("%1；末端速度已降至停车阈值")
                    .arg(controlledStopReason_);
            return output;
        }
    }
    else{
        sensorSample = wrenchSource_.sample(stamp, elapsedS);
        const WrenchTransformResult transformed =
                wrenchTransformer_->toPlatformCenterOfMass(sensorSample);
        if(!transformed.sample.valid){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = transformed.errorMessage.isEmpty() ?
                        QStringLiteral("阶段B力旋量转换失败") : transformed.errorMessage;
            return output;
        }
        platformSample = transformed.sample;
        if(config_.translationOnly){
            platformSample.wrench[3] = 0.0;
            platformSample.wrench[4] = 0.0;
            platformSample.wrench[5] = 0.0;
        }
        dynamicsResult = dynamics_.step(platformSample,
                                        config_.periodUs / 1000000.0);
        if(!dynamicsResult.valid){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral("阶段B Newmark失败：%1")
                    .arg(dynamicsResult.errorMessage);
            return output;
        }
        desired = dynamicsResult.state;
        const double accelerationMmPerSec2 =
                vectorNorm3(desired.acceleration) * 1000.0;
        if(accelerationMmPerSec2 >
                config_.workspaceSafety.stoppingDecelerationMmPerSec2){
            requestControlledStop(
                        QStringLiteral("末端响应加速度%1 mm/s²超过上限%2 mm/s²")
                        .arg(accelerationMmPerSec2, 0, 'f', 6)
                        .arg(config_.workspaceSafety.stoppingDecelerationMmPerSec2,
                             0, 'f', 6),
                        true,
                        ForceInteractionControlledStopCause::AccelerationLimit);
            braking = true;
            bool stopped = false;
            QString brakingError;
            desired = advanceBrakingState(stopped, &brakingError);
            if(!brakingError.isEmpty()){
                output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
                output.reason = brakingError;
                return output;
            }
            if(stopped){
                output.action = ForceInteractionRuntimeStep::Action::NormalStop;
                output.reason = QStringLiteral("%1；末端速度已降至停车阈值")
                        .arg(controlledStopReason_);
                return output;
            }
        }
    }

    std::vector<double> poseMmRad(6, 0.0);
    for(int dim = 0; dim < 3; ++dim){
        poseMmRad[dim] = desired.pose[dim] * 1000.0;
    }
    for(int dim = 3; dim < 6; ++dim){
        poseMmRad[dim] = desired.pose[dim];
    }
    std::array<double, 6> desiredPoseArray{};
    std::copy_n(poseMmRad.cbegin(), desiredPoseArray.size(),
                desiredPoseArray.begin());
    PhysicalWorkspaceMotionSample motionSample;
    motionSample.poseMmRad = desiredPoseArray;
    for(int dimension = 0; dimension < 3; ++dimension){
        motionSample.twistMmRadPerSec[static_cast<size_t>(dimension)] =
                desired.twist[static_cast<size_t>(dimension)] * 1000.0;
        motionSample.accelerationMmRadPerSec2[static_cast<size_t>(dimension)] =
                desired.acceleration[static_cast<size_t>(dimension)] * 1000.0;
    }
    for(int dimension = 3; dimension < 6; ++dimension){
        motionSample.twistMmRadPerSec[static_cast<size_t>(dimension)] =
                desired.twist[static_cast<size_t>(dimension)];
        motionSample.accelerationMmRadPerSec2[static_cast<size_t>(dimension)] =
                desired.acceleration[static_cast<size_t>(dimension)];
    }
    PhysicalWorkspaceBoundaryResult workspaceResult =
            physicalBoundary_.evaluateMotion(motionSample,
                                             config_.workspaceSafety);
    if(workspaceResult.action == PhysicalWorkspaceAction::EmergencyStop ||
            workspaceResult.action == PhysicalWorkspaceAction::Invalid){
        output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
        output.reason = QStringLiteral("阶段B末端状态触发物理边界急停：%1")
                .arg(workspaceResult.reason);
        return output;
    }
    if(!braking && workspaceResult.action == PhysicalWorkspaceAction::ControlledStop){
        requestControlledStop(
                    workspaceResult.reason, true,
                    ForceInteractionControlledStopCause::WorkspaceBoundary);
        braking = true;
        bool stopped = false;
        QString brakingError;
        desired = advanceBrakingState(stopped, &brakingError);
        if(!brakingError.isEmpty()){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = brakingError;
            return output;
        }
        if(stopped){
            output.action = ForceInteractionRuntimeStep::Action::NormalStop;
            output.reason = QStringLiteral("%1；末端速度已降至停车阈值")
                    .arg(controlledStopReason_);
            return output;
        }
        for(int dimension = 0; dimension < 3; ++dimension){
            poseMmRad[static_cast<size_t>(dimension)] =
                    desired.pose[static_cast<size_t>(dimension)] * 1000.0;
        }
        for(int dimension = 3; dimension < 6; ++dimension){
            poseMmRad[static_cast<size_t>(dimension)] =
                    desired.pose[static_cast<size_t>(dimension)];
        }
        std::copy_n(poseMmRad.cbegin(), desiredPoseArray.size(),
                    desiredPoseArray.begin());
        motionSample.poseMmRad = desiredPoseArray;
        for(int dimension = 0; dimension < 3; ++dimension){
            motionSample.twistMmRadPerSec[static_cast<size_t>(dimension)] =
                    desired.twist[static_cast<size_t>(dimension)] * 1000.0;
            motionSample.accelerationMmRadPerSec2[static_cast<size_t>(dimension)] =
                    desired.acceleration[static_cast<size_t>(dimension)] * 1000.0;
        }
        for(int dimension = 3; dimension < 6; ++dimension){
            motionSample.twistMmRadPerSec[static_cast<size_t>(dimension)] =
                    desired.twist[static_cast<size_t>(dimension)];
            motionSample.accelerationMmRadPerSec2[static_cast<size_t>(dimension)] =
                    desired.acceleration[static_cast<size_t>(dimension)];
        }
        workspaceResult = physicalBoundary_.evaluateMotion(
                    motionSample, config_.workspaceSafety);
        if(workspaceResult.action == PhysicalWorkspaceAction::EmergencyStop ||
                workspaceResult.action == PhysicalWorkspaceAction::Invalid){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral("受控制动状态已到固定急停线：%1")
                    .arg(workspaceResult.reason);
            return output;
        }
    }
    const CompensatedCableKinematics::Evaluation evaluation =
            kinematics_.evaluatePose({poseMmRad}, kinematicsState_);
    if(!evaluation.valid || evaluation.relativeMotorThetaRad.size() != kOnlineVelocityAxisCount ||
            evaluation.cableLengthMm.size() != kOnlineVelocityAxisCount){
        output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
        output.reason = QStringLiteral("阶段B逆运动学失败：%1").arg(evaluation.errorMessage);
        return output;
    }

    const double dt = config_.periodUs / 1000000.0;
    OnlineVelocityAxisArray reference{};
    OnlineVelocityAxisArray referenceVelocity{};
    OnlineVelocityAxisArray correction{};
    OnlineVelocityAxisArray command{};
    double maximumError = 0.0;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        reference[axis] = actualStartPosition_[axis] +
                evaluation.relativeMotorThetaRad[axis] * config_.motorUnitPerRadian[axis];
        if(reference[axis] < config_.motorPositionMinimum[axis] ||
                reference[axis] > config_.motorPositionMaximum[axis]){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral("阶段B轴%1期望位置越界").arg(axis);
            return output;
        }
        referenceVelocity[axis] = (reference[axis] - lastReferencePosition_[axis]) / dt;
        const double error = reference[axis] - feedback.actualPosition[axis];
        maximumError = std::max(maximumError, std::fabs(error));
        if(std::fabs(error) > config_.followingErrorLimit){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral("阶段B轴%1位置跟随误差%2超限%3")
                    .arg(axis).arg(error, 0, 'f', 6)
                    .arg(config_.followingErrorLimit, 0, 'f', 6);
            return output;
        }
        if(config_.pidEnabled){
            integral_[axis] = clampValue(integral_[axis] + error * dt,
                                         config_.integralLimit);
            const double derivative = previousErrorValid_ ?
                        (error - previousError_[axis]) / dt : 0.0;
            correction[axis] = clampValue(config_.kp * error +
                                           config_.ki * integral_[axis] +
                                           config_.kd * derivative,
                                           config_.correctionVelocityLimit);
            previousError_[axis] = error;
        }
        command[axis] = (config_.feedForwardEnabled ?
                             config_.feedForwardGain * referenceVelocity[axis] : 0.0) +
                correction[axis];
    }
    // 轴速上限仍按共同倍率执行，避免逐轴截断破坏八绳协同关系。
    // 不再限制每周期速度命令增量：六维力交互必须保留 Newmark 给出的
    // 纯惯性时间响应，不能用公共加速度倍率暗中改变等效质量。
    double velocityScale = 1.0;
    for(double value : command){
        if(std::fabs(value) > config_.velocityLimit){
            velocityScale = std::min(velocityScale,
                                     config_.velocityLimit / std::fabs(value));
        }
    }
    for(double& value : command){
        value *= velocityScale;
    }
    previousErrorValid_ = true;
    kinematicsState_ = evaluation.nextState;
    lastReferencePosition_ = reference;

    ForceInteractionRunRecord record;
    record.stepIndex = status_.stepCount + 1;
    record.elapsedS = elapsedS;
    record.stamp = stamp;
    record.availabilityMask = ForceRecordDesiredState |
            ForceRecordCableKinematics | ForceRecordAxisReference |
            ForceRecordAxisCommand | ForceRecordAxisTrace | ForceRecordTiming;
    if(!braking){
        record.availabilityMask |= ForceRecordSensorWrench |
                ForceRecordPlatformWrench;
    }
    record.sensorWrench = sensorSample.wrench;
    record.platformWrench = platformSample.wrench;
    record.desiredState = desired;
    record.interactionSegment = braking ? 1 : 0;
    record.controlledStopCause = static_cast<int>(status_.controlledStopCause);
    record.workspaceAction = static_cast<int>(workspaceResult.action);
    record.workspaceMinimumClearanceMm = workspaceResult.minimumClearanceMm;
    record.workspaceLimitingClearanceMm = workspaceResult.limitingClearanceMm;
    record.workspaceOutwardSpeedMmPerSec =
            workspaceResult.limitingOutwardSpeedMmPerSec;
    record.workspaceOutwardAccelerationMmPerSec2 =
            workspaceResult.limitingOutwardAccelerationMmPerSec2;
    record.workspacePureStoppingDistanceMm =
            workspaceResult.pureStoppingDistanceMm;
    record.workspaceTriggerDistanceMm = workspaceResult.triggerDistanceMm;
    record.workspaceLimitingPoint = workspaceResult.limitingPointIndex;
    record.workspaceLimitingAxis = workspaceResult.limitingAxis;
    record.workspaceLimitingUpperFace = workspaceResult.limitingUpperFace;
    record.workspacePointGlobalMm = workspaceResult.platformPointsGlobalMm;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        record.cableLengthMm[axis] = evaluation.cableLengthMm[axis];
        record.relativeMotorThetaRad[axis] = evaluation.relativeMotorThetaRad[axis];
        record.axisReferencePosition[axis] = reference[axis];
        record.axisReferenceVelocity[axis] = referenceVelocity[axis];
        record.axisPidCorrectionVelocity[axis] = correction[axis];
        record.axisCommandVelocity[axis] = command[axis];
        record.axisTracePosition[axis] = feedback.actualPosition[axis];
        record.axisTraceVelocity[axis] = feedback.actualVelocity[axis];
    }
    record.newmarkIterations = dynamicsResult.iterations;
    record.newmarkResidual = dynamicsResult.residual;
    record.calculationDurationUs = calculationTimer.nsecsElapsed() / 1000;

    output.action = ForceInteractionRuntimeStep::Action::CommandVelocity;
    output.commandVelocity = command;
    output.actualPosition = feedback.actualPosition;
    output.record = record;
    status_.stepCount = record.stepIndex;
    status_.elapsedS = elapsedS;
    status_.latestTraceSequence = feedback.logicalFrameSequence;
    status_.maximumPositionError = std::max(status_.maximumPositionError, maximumError);
    status_.latestCalculationUs = record.calculationDurationUs;
    status_.maximumCalculationUs = std::max(status_.maximumCalculationUs,
                                             record.calculationDurationUs);
    status_.desiredState = desired;
    status_.latestWorkspaceClearanceMm = workspaceResult.minimumClearanceMm;
    status_.minimumWorkspaceClearanceMm = std::min(
                status_.minimumWorkspaceClearanceMm,
                workspaceResult.minimumClearanceMm);
    status_.workspaceTriggerDistanceMm = workspaceResult.triggerDistanceMm;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        status_.desiredCableLengthMm[axis] = evaluation.cableLengthMm[axis];
    }
    status_.referencePosition = reference;
    status_.actualPosition = feedback.actualPosition;
    status_.commandVelocity = command;
    return output;
}

void ForceInteractionRuntimeControl::noteCommandResult(
        const ForceInteractionRuntimeStep& step,
        bool commandOk,
        qint64 apiDurationUs,
        qint64 fullCycleDurationUs)
{
    status_.latestApiUs = apiDurationUs;
    status_.maximumApiUs = std::max(status_.maximumApiUs, apiDurationUs);
    if(!commandOk){
        setTerminal(ForceInteractionRuntimeStatus::State::Fault,
                    step.action == ForceInteractionRuntimeStep::Action::CommandVelocity ?
                        QStringLiteral("阶段B八轴速度API调用失败") :
                        QStringLiteral("阶段B停机API调用失败，已升级立即停止"));
        return;
    }
    if(step.action == ForceInteractionRuntimeStep::Action::CommandVelocity){
        ForceInteractionRunRecord record = step.record;
        record.hardwareApiDurationUs = apiDurationUs;
        record.fullCycleDurationUs = fullCycleDurationUs;
        if(recorder_){
            recorder_->tryAppend(record);
            status_.droppedRecordCount = recorder_->droppedCount();
        }
        ++status_.commandCount;
    }
    else if(step.action == ForceInteractionRuntimeStep::Action::NormalStop){
        setTerminal(ForceInteractionRuntimeStatus::State::Completed, step.reason);
    }
    else if(step.action == ForceInteractionRuntimeStep::Action::EmergencyStop){
        setTerminal(ForceInteractionRuntimeStatus::State::Fault, step.reason);
    }
}

void ForceInteractionRuntimeControl::setTerminal(
        ForceInteractionRuntimeStatus::State state,
        const QString& message)
{
    status_.state = state;
    status_.message = message;
}

void ForceInteractionRuntimeControl::stop(bool fault, const QString& reason)
{
    if(status_.state == ForceInteractionRuntimeStatus::State::Idle){
        return;
    }
    setTerminal(fault ? ForceInteractionRuntimeStatus::State::Fault :
                        ForceInteractionRuntimeStatus::State::Stopped,
                reason);
}

void ForceInteractionRuntimeControl::finishRecording()
{
    if(!recorder_){
        return;
    }
    recorder_->requestFinish();
    recorder_->finishAndWait();
    status_.acceptedRecordCount = recorder_->acceptedCount();
    status_.writtenRecordCount = recorder_->writtenCount();
    status_.droppedRecordCount = recorder_->droppedCount();
    status_.recordingError = recorder_->writerError();
    recorder_.reset();
}

bool ForceInteractionRuntimeControl::isActive() const
{
    return status_.state == ForceInteractionRuntimeStatus::State::WaitingForTrace ||
            status_.state == ForceInteractionRuntimeStatus::State::Running ||
            status_.state == ForceInteractionRuntimeStatus::State::Braking;
}

bool ForceInteractionRuntimeControl::isPrepared() const
{
    return status_.state == ForceInteractionRuntimeStatus::State::Prepared;
}

const ForceInteractionRuntimeConfig& ForceInteractionRuntimeControl::currentConfig() const
{
    return config_;
}

ForceInteractionRuntimeStatus ForceInteractionRuntimeControl::status() const
{
    return status_;
}
