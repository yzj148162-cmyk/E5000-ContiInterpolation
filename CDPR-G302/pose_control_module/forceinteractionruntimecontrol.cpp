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

bool finiteWrench(const ForceInteractionVector6& values)
{
    return std::all_of(values.begin(), values.end(), [](double value){
        return std::isfinite(value);
    });
}

QString runtimeStageName(ForceInteractionWrenchSourceKind source)
{
    return source == ForceInteractionWrenchSourceKind::RealFtTrace ?
                QStringLiteral("阶段C") : QStringLiteral("阶段B");
}

QString runtimeSourceName(ForceInteractionWrenchSourceKind source)
{
    return source == ForceInteractionWrenchSourceKind::RealFtTrace ?
                QStringLiteral("真实F/T Trace（冻结软件零点）") :
                QStringLiteral("模拟六维力");
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

bool validateMotorSafetyRelativeTravel(
        const ForceInteractionRuntimeConfig& config,
        const OnlineVelocityAxisArray& startSafetyRelativePosition,
        const OnlineVelocityAxisArray& actualSafetyRelativePosition,
        const OnlineVelocityAxisArray& relativeCommandPosition,
        OnlineVelocityAxisArray* referenceSafetyRelativePosition,
        QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        const double minimum = config.motorSafetyRelativeMinimum[axis];
        const double maximum = config.motorSafetyRelativeMaximum[axis];
        const double actual = actualSafetyRelativePosition[axis];
        const double reference = startSafetyRelativePosition[axis] +
                relativeCommandPosition[axis];
        if(!std::isfinite(minimum) || !std::isfinite(maximum) ||
                minimum >= maximum || !std::isfinite(actual) ||
                !std::isfinite(reference)){
            return fail(QStringLiteral("轴%1绞盘安全相对位置或边界无效")
                        .arg(axis));
        }
        if(actual < minimum || actual > maximum){
            return fail(QStringLiteral(
                        "轴%1绞盘实际安全相对位置%2已越过[%3, %4]")
                        .arg(axis)
                        .arg(actual, 0, 'f', 6)
                        .arg(minimum, 0, 'f', 6)
                        .arg(maximum, 0, 'f', 6));
        }
        if(reference < minimum || reference > maximum){
            return fail(QStringLiteral(
                        "轴%1绞盘期望安全相对位置%2将越过[%3, %4]")
                        .arg(axis)
                        .arg(reference, 0, 'f', 6)
                        .arg(minimum, 0, 'f', 6)
                        .arg(maximum, 0, 'f', 6));
        }
        if(referenceSafetyRelativePosition){
            (*referenceSafetyRelativePosition)[axis] = reference;
        }
    }
    return true;
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
        return fail(QStringLiteral("六维力交互实机运行仅允许G302模板"));
    }
    if(periodUs < 1000 || periodUs > 20000){
        return fail(QStringLiteral("控制周期必须位于1~20 ms"));
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(!traceDelayValid[axis] || !std::isfinite(traceDelayMs[axis]) ||
                traceDelayMs[axis] < 0.0 || traceDelayMs[axis] > 20.0){
            return fail(QStringLiteral("轴%1缺少与当前硬件模板匹配的有效Trace延迟标定").arg(axis));
        }
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
    if(wrenchSourceKind == ForceInteractionWrenchSourceKind::RealFtTrace){
        if(!finiteWrench(ftSoftwareZero) || ftSampleTimeoutUs <= 0 ||
                !sensorTransform.configured){
            return fail(QStringLiteral("阶段C冻结零点、F/T超时或安装变换无效"));
        }
        QString conditioningError;
        if(!realFtConditioning.validate(periodUs / 1000000.0,
                                        &conditioningError)){
            return fail(QStringLiteral("阶段C真实F/T输入调理参数无效：%1")
                        .arg(conditioningError));
        }
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(std::fabs(motorUnitPerRadian[axis]) <= 1.0e-12 ||
                motorSafetyRelativeMinimum[axis] >=
                    motorSafetyRelativeMaximum[axis]){
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
            *errorMessage = QStringLiteral("六维力交互实机运行正在执行");
        }
        return false;
    }
    QString error;
    if(!config.validate(&error) ||
            (config.wrenchSourceKind ==
                 ForceInteractionWrenchSourceKind::Simulated &&
             !wrenchSource_.configure(config.wrenchProfile,
                                      config.periodUs / 1000000.0,
                                      &error)) ||
            (config.wrenchSourceKind ==
                 ForceInteractionWrenchSourceKind::RealFtTrace &&
             !wrenchConditioner_.configure(config.realFtConditioning,
                                            config.periodUs / 1000000.0,
                                            &error)) ||
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
    if(config.wrenchSourceKind != ForceInteractionWrenchSourceKind::RealFtTrace){
        wrenchConditioner_.reset();
    }
    wrenchTransformer_ = std::make_unique<WrenchTransformer>(config.sensorTransform);
    kinematicsState_ = kinematics_.initialState();
    status_ = ForceInteractionRuntimeStatus{};
    status_.wrenchSourceKind = config.wrenchSourceKind;
    status_.frozenFtSoftwareZero = config.ftSoftwareZero;
    status_.state = ForceInteractionRuntimeStatus::State::Prepared;
    status_.message = QStringLiteral("%1已准备").arg(
                runtimeStageName(config.wrenchSourceKind));
    status_.desiredState = config.initialState;
    actualStartCaptured_ = false;
    previousErrorValid_ = false;
    lastFrameSequenceValid_ = false;
    lastFtSampleCounterValid_ = false;
    lastFtCounterChangeUs_ = 0;
    hostStartUs_ = 0;
    realFtInteractionStartUs_ = 0;
    lastCommandUs_ = 0;
    modelStepCount_ = 0;
    startTraceSequence_ = 0;
    brakingState_ = ForceInteractionPlatformState{};
    controlledStopReason_.clear();
    integral_.fill(0.0);
    previousError_.fill(0.0);
    referenceHistory_.clear();
    return true;
}

bool ForceInteractionRuntimeControl::alignedReferenceAt(
        int axis, quint64 feedbackSequence, int traceSamplePeriodUs,
        double* reference) const
{
    if(!reference || axis < 0 || axis >= kOnlineVelocityAxisCount ||
            traceSamplePeriodUs <= 0 || referenceHistory_.empty() ||
            feedbackSequence < startTraceSequence_) return false;
    // 与 cdpr_control 一致：指令和动力学使用主机运行时钟；这里只借助
    // 启动锚点把 Trace 帧映射到同一运行时间，再扣除该轴标定延迟。
    const double traceElapsedS = static_cast<double>(
                feedbackSequence - startTraceSequence_) *
            static_cast<double>(traceSamplePeriodUs) / 1000000.0;
    const double target = traceElapsedS - config_.traceDelayMs[axis] / 1000.0;
    if(target < referenceHistory_.front().elapsedS) return false;
    for(size_t index = 1; index < referenceHistory_.size(); ++index){
        const auto& left = referenceHistory_[index - 1];
        const auto& right = referenceHistory_[index];
        if(target > right.elapsedS) continue;
        const double span = right.elapsedS - left.elapsedS;
        if(span <= 0.0){
            *reference = right.reference[axis];
            return true;
        }
        const double ratio = std::clamp(
                    (target - left.elapsedS) / span,
                    0.0, 1.0);
        *reference = left.reference[axis] +
                ratio * (right.reference[axis] - left.reference[axis]);
        return std::isfinite(*reference);
    }
    return false;
}

bool ForceInteractionRuntimeControl::start(qint64 nowUs, QString* errorMessage)
{
    if(status_.state != ForceInteractionRuntimeStatus::State::Prepared){
        if(errorMessage){
            *errorMessage = QStringLiteral("请先准备六维力交互实机运行");
        }
        return false;
    }
    recorder_ = std::make_unique<ForceInteractionRunRecorder>();
    ForceInteractionRunMetadata metadata;
    metadata.stage = config_.wrenchSourceKind ==
            ForceInteractionWrenchSourceKind::RealFtTrace ?
                QStringLiteral("stage_c") : QStringLiteral("stage_b");
    metadata.sourceName = config_.wrenchSourceKind ==
            ForceInteractionWrenchSourceKind::RealFtTrace ?
                runtimeSourceName(config_.wrenchSourceKind) :
                wrenchSource_.summary();
    metadata.machineTemplateName = config_.machineTemplateName;
    metadata.controlPeriodS = config_.periodUs / 1000000.0;
    metadata.plannedDurationS = config_.maximumTestDurationS;
    metadata.workspaceReplayEnabled = true;
    metadata.physicalWorkspace = config_.physicalWorkspace;
    metadata.workspaceSafety = config_.workspaceSafety;
    metadata.motorSafetyRelativeBoundsEnabled = true;
    metadata.motorSafetyRelativeMinimum =
            config_.motorSafetyRelativeMinimum;
    metadata.motorSafetyRelativeMaximum =
            config_.motorSafetyRelativeMaximum;
    metadata.realFtConditioningEnabled = config_.wrenchSourceKind ==
            ForceInteractionWrenchSourceKind::RealFtTrace;
    metadata.realFtConditioning = config_.realFtConditioning;
    QString recordError;
    if(!recorder_->begin(config_.recordingDirectory, metadata,
                         &status_.recordFile, &recordError)){
        recorder_.reset();
        if(errorMessage){
            *errorMessage = QStringLiteral("%1记录器启动失败：%2")
                    .arg(runtimeStageName(config_.wrenchSourceKind), recordError);
        }
        return false;
    }
    waitStartUs_ = nowUs;
    lastGoodTraceUs_ = 0;
    nextDueUs_ = nowUs;
    hostStartUs_ = 0;
    realFtInteractionStartUs_ = 0;
    lastCommandUs_ = 0;
    modelStepCount_ = 0;
    startTraceSequence_ = 0;
    lastFtSampleCounterValid_ = false;
    lastFtCounterChangeUs_ = 0;
    status_.state = ForceInteractionRuntimeStatus::State::WaitingForTrace;
    status_.message = config_.wrenchSourceKind ==
            ForceInteractionWrenchSourceKind::RealFtTrace ?
                QStringLiteral("等待新鲜完整的八轴＋F/T同帧Trace") :
                QStringLiteral("等待新鲜完整的八轴Trace帧");
    return true;
}

bool ForceInteractionRuntimeControl::realFtSample(
        const ForceInteractionRuntimeFeedback& feedback,
        qint64 nowUs,
        ForceInteractionWrenchSample& sample,
        qint64& sampleAgeUs,
        QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    const FtSensorTraceSample& ft = feedback.ftSensor;
    if(!feedback.ftRuntimeProfileActive){
        return fail(QStringLiteral("当前不是八轴＋F/T合并Trace profile"));
    }
    if(!ft.wrenchComplete() || !ft.statusValid ||
            !ft.sampleCounterValid || !ft.traceFrameSequenceValid){
        return fail(QStringLiteral(
                        "F/T同帧对象不完整：六维力/状态/计数/序号=%1/%2/%3/%4")
                    .arg(ft.wrenchComplete() ? 1 : 0)
                    .arg(ft.statusValid ? 1 : 0)
                    .arg(ft.sampleCounterValid ? 1 : 0)
                    .arg(ft.traceFrameSequenceValid ? 1 : 0));
    }
    if(ft.traceFrameSequence != feedback.traceFrameSequence ||
            ft.monotonicUs <= 0 || ft.monotonicUs != feedback.monotonicUs){
        return fail(QStringLiteral(
                        "F/T与八轴反馈不是同一Trace帧：F/T=%1/%2，八轴=%3/%4")
                    .arg(ft.traceFrameSequence).arg(ft.monotonicUs)
                    .arg(feedback.traceFrameSequence).arg(feedback.monotonicUs));
    }
    sampleAgeUs = nowUs >= ft.monotonicUs ? nowUs - ft.monotonicUs : 0;
    if(sampleAgeUs > config_.ftSampleTimeoutUs){
        return fail(QStringLiteral("F/T样本帧龄%1 us超过上限%2 us")
                    .arg(sampleAgeUs).arg(config_.ftSampleTimeoutUs));
    }
    if((ft.statusCode & config_.ftStatusMask) !=
            (config_.ftExpectedStatus & config_.ftStatusMask)){
        return fail(QStringLiteral("F/T状态码0x%1不满足期望0x%2（掩码0x%3）")
                    .arg(QString::number(ft.statusCode, 16).toUpper())
                    .arg(QString::number(config_.ftExpectedStatus, 16).toUpper())
                    .arg(QString::number(config_.ftStatusMask, 16).toUpper()));
    }
    if(lastFtSampleCounterValid_){
        const quint32 advance = ft.sampleCounter - lastFtSampleCounter_;
        if(advance == 0){
            if(lastFtCounterChangeUs_ > 0 &&
                    nowUs - lastFtCounterChangeUs_ > config_.ftSampleTimeoutUs){
                return fail(QStringLiteral("F/T SampleCounter=%1已停滞%2 us")
                            .arg(ft.sampleCounter)
                            .arg(nowUs - lastFtCounterChangeUs_));
            }
        }
        else{
            // 无符号差值自然覆盖32位回绕；大于半量程只能解释为倒退。
            if(advance > 0x7fffffffu){
                return fail(QStringLiteral("F/T SampleCounter倒退：%1→%2")
                            .arg(lastFtSampleCounter_).arg(ft.sampleCounter));
            }
            const qint64 elapsedSinceChangeUs = lastFtCounterChangeUs_ > 0 ?
                        std::max<qint64>(0, nowUs - lastFtCounterChangeUs_) : 0;
            const quint32 maximumPlausibleAdvance = static_cast<quint32>(
                        std::max<qint64>(32,
                            elapsedSinceChangeUs /
                                std::max(1, feedback.traceSamplePeriodUs) + 32));
            if(advance > maximumPlausibleAdvance){
                return fail(QStringLiteral(
                                "F/T SampleCounter异常跳变：%1→%2（步进%3，上限%4）")
                            .arg(lastFtSampleCounter_).arg(ft.sampleCounter)
                            .arg(advance).arg(maximumPlausibleAdvance));
            }
            lastFtCounterChangeUs_ = nowUs;
        }
    }
    else{
        lastFtCounterChangeUs_ = nowUs;
        lastFtSampleCounterValid_ = true;
    }
    lastFtSampleCounter_ = ft.sampleCounter;

    sample.stamp.traceSequence = feedback.logicalFrameSequence;
    sample.stamp.traceTimeUs = static_cast<qint64>(
                feedback.logicalFrameSequence) *
            std::max(1, feedback.traceSamplePeriodUs);
    sample.stamp.hostMonotonicTimeUs = ft.monotonicUs;
    sample.stamp.traceValid = true;
    sample.stamp.valid = true;
    sample.coordinate = ForceInteractionWrenchCoordinate::Sensor;
    sample.wrench = ft.value;
    sample.valid = finiteWrench(sample.wrench);
    if(!sample.valid){
        return fail(QStringLiteral("F/T工程量包含非有限数"));
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool ForceInteractionRuntimeControl::feedbackReady(
        const ForceInteractionRuntimeFeedback& feedback) const
{
    const bool motorFeedbackReady = feedback.fromTrace &&
            feedback.frameSequenceValid &&
            feedback.timingReliable && feedback.fifoCaughtUp &&
            !feedback.traceLost && feedback.frameCount > 0 &&
            feedback.newestFrameAgeUs >= 0 &&
            feedback.newestFrameAgeUs <= config_.traceTimeoutUs &&
            finiteArray(feedback.actualPosition) &&
            finiteArray(feedback.safetyRelativePosition) &&
            std::all_of(feedback.safetyRelativePositionFromTrace.cbegin(),
                        feedback.safetyRelativePositionFromTrace.cend(),
                        [](bool valid){ return valid; }) &&
            finiteArray(feedback.actualVelocity) &&
            std::all_of(feedback.motorStateMachine.cbegin(),
                        feedback.motorStateMachine.cend(),
                        [](int state){ return state >= 0; });
    if(!motorFeedbackReady || config_.wrenchSourceKind !=
            ForceInteractionWrenchSourceKind::RealFtTrace){
        return motorFeedbackReady;
    }

    const FtSensorTraceSample& ft = feedback.ftSensor;
    return feedback.ftRuntimeProfileActive && ft.wrenchComplete() &&
            ft.statusValid && ft.sampleCounterValid &&
            ft.traceFrameSequenceValid &&
            ft.traceFrameSequence == feedback.traceFrameSequence &&
            ft.monotonicUs > 0 && ft.monotonicUs == feedback.monotonicUs;
}

bool ForceInteractionRuntimeControl::requestControlledStop(
        const QString& reason, bool experimentFailure,
        ForceInteractionControlledStopCause cause)
{
    if(status_.state == ForceInteractionRuntimeStatus::State::Braking){
        status_.experimentValid = status_.experimentValid && !experimentFailure;
        if(experimentFailure){
            const QString escalatedReason = reason.isEmpty() ?
                        QStringLiteral("协同减速期间出现新的安全失败") : reason;
            status_.safetyStopReason = escalatedReason;
            controlledStopReason_ = escalatedReason;
            status_.controlledStopCause = cause;
            status_.message = QStringLiteral("协同减速中：%1")
                    .arg(escalatedReason);
        }
        return true;
    }
    if(status_.state != ForceInteractionRuntimeStatus::State::Running){
        return false;
    }
    brakingState_ = status_.desiredState;
    controlledStopReason_ = reason.isEmpty() ?
                QStringLiteral("请求受控制动") : reason;
    status_.experimentValid = status_.experimentValid && !experimentFailure;
    if(experimentFailure){
        status_.safetyStopReason = controlledStopReason_;
    }
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
                        "%1可靠Trace超时：fromTrace=%2，序号有效=%3，时序可靠=%4，FIFO已追平=%5，丢帧=%6，帧龄=%7 us，逻辑序号=%8，安全相对位置/状态字完整=%9/%10")
                    .arg(runtimeStageName(config_.wrenchSourceKind))
                    .arg(feedback.fromTrace ? 1 : 0)
                    .arg(feedback.frameSequenceValid ? 1 : 0)
                    .arg(feedback.timingReliable ? 1 : 0)
                    .arg(feedback.fifoCaughtUp ? 1 : 0)
                    .arg(feedback.traceLost ? 1 : 0)
                    .arg(feedback.newestFrameAgeUs)
                    .arg(feedback.logicalFrameSequence)
                    .arg(std::all_of(
                             feedback.safetyRelativePositionFromTrace.cbegin(),
                             feedback.safetyRelativePositionFromTrace.cend(),
                             [](bool valid){ return valid; }) ? 1 : 0)
                    .arg(std::all_of(
                             feedback.motorStateMachine.cbegin(),
                             feedback.motorStateMachine.cend(),
                             [](int state){ return state >= 0; }) ? 1 : 0);
            if(config_.wrenchSourceKind ==
                    ForceInteractionWrenchSourceKind::RealFtTrace){
                output.reason += QStringLiteral("，F/T profile/对象/同帧=%1/%2/%3")
                        .arg(feedback.ftRuntimeProfileActive ? 1 : 0)
                        .arg(feedback.ftSensor.wrenchComplete() &&
                             feedback.ftSensor.statusValid &&
                             feedback.ftSensor.sampleCounterValid &&
                             feedback.ftSensor.traceFrameSequenceValid ? 1 : 0)
                        .arg(feedback.ftSensor.traceFrameSequenceValid &&
                             feedback.ftSensor.traceFrameSequence ==
                                 feedback.traceFrameSequence &&
                             feedback.ftSensor.monotonicUs > 0 &&
                             feedback.ftSensor.monotonicUs ==
                                 feedback.monotonicUs ? 1 : 0);
            }
        }
        return output;
    }
    if(feedback.traceSamplePeriodUs <= 0 ||
            config_.periodUs % feedback.traceSamplePeriodUs != 0){
        output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
        output.reason = QStringLiteral(
                    "%1控制周期%2 us不是Trace采样周期%3 us的整数倍")
                .arg(runtimeStageName(config_.wrenchSourceKind))
                .arg(config_.periodUs)
                .arg(feedback.traceSamplePeriodUs);
        return output;
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        if(feedback.motorStateMachine[axis] != 4){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral(
                        "%1轴%2同帧驱动状态异常：0x6041=0x%3，状态=%4，要求=4(Operation enabled)")
                    .arg(runtimeStageName(config_.wrenchSourceKind))
                    .arg(axis)
                    .arg(QString::number(feedback.motorStatusWord[axis], 16)
                         .rightJustified(4, QLatin1Char('0')).toUpper())
                    .arg(feedback.motorStateMachine[axis]);
            return output;
        }
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
        actualStartSafetyRelativePosition_ = feedback.safetyRelativePosition;
        lastReferencePosition_ = actualStartPosition_;
        referenceHistory_.clear();
        referenceHistory_.push_back(
                    ReferenceHistorySample{0.0,
                                           actualStartPosition_});
        hostStartUs_ = nowUs;
        lastCommandUs_ = 0;
        modelStepCount_ = 0;
        startTraceSequence_ = feedback.logicalFrameSequence;
        status_.actualStartPosition = actualStartPosition_;
        status_.actualStartSafetyRelativePosition =
                actualStartSafetyRelativePosition_;
        actualStartCaptured_ = true;
        status_.state = ForceInteractionRuntimeStatus::State::Running;
        status_.message = config_.wrenchSourceKind ==
                ForceInteractionWrenchSourceKind::RealFtTrace ?
                    QStringLiteral("阶段C运行中，等待首次有效受力（试验计时尚未开始）") :
                    QStringLiteral("%1运行中").arg(
                        runtimeStageName(config_.wrenchSourceKind));
    }

    // 实机链统一使用主机单调时钟。Newmark 保持固定步长，并在调度迟到时
    // 补齐数学子步；Trace 序号只用于反馈时刻映射和延迟对齐。
    const qint64 hostElapsedUs = std::max<qint64>(0, nowUs - hostStartUs_);
    const double elapsedS = hostElapsedUs / 1000000.0;
    const quint64 requiredModelSteps = static_cast<quint64>(
                hostElapsedUs / config_.periodUs);
    const quint64 pendingModelSteps = requiredModelSteps >= modelStepCount_ ?
                requiredModelSteps - modelStepCount_ : 0;
    constexpr quint64 kMaximumCatchUpSteps = 20;
    if(pendingModelSteps > kMaximumCatchUpSteps){
        output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
        output.reason = QStringLiteral(
                    "%1主机调度累计落后过大：需补算%2个Newmark子步（周期%3 us），超过上限%4")
                .arg(runtimeStageName(config_.wrenchSourceKind))
                .arg(pendingModelSteps).arg(config_.periodUs)
                .arg(kMaximumCatchUpSteps);
        return output;
    }
    const int integrationSteps = static_cast<int>(pendingModelSteps);
    const double modelElapsedBeforeS = static_cast<double>(modelStepCount_) *
            config_.periodUs / 1000000.0;
    const double commandDt = lastCommandUs_ > 0 ?
                std::max(1.0e-6, (nowUs - lastCommandUs_) / 1000000.0) :
                config_.periodUs / 1000000.0;
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
    ForceWrenchConditioningResult conditioningResult;
    sensorSample.stamp = stamp;
    platformSample.stamp = stamp;
    CdprDynamicsStepResult dynamicsResult;
    ForceInteractionPlatformState desired = status_.desiredState;
    bool braking = status_.state == ForceInteractionRuntimeStatus::State::Braking;
    qint64 ftSampleAgeUs = -1;
    const bool realFtRuntime = config_.wrenchSourceKind ==
            ForceInteractionWrenchSourceKind::RealFtTrace;
    status_.interactionTriggered = realFtInteractionStartUs_ > 0;
    status_.interactionElapsedS = status_.interactionTriggered ?
                std::max<qint64>(0, nowUs - realFtInteractionStartUs_) /
                    1000000.0 : 0.0;
    const double durationClockS = realFtRuntime ?
                status_.interactionElapsedS : elapsedS;
    if(!braking && config_.maximumTestDurationS > 0.0 &&
            (!realFtRuntime || status_.interactionTriggered) &&
            durationClockS > config_.maximumTestDurationS + 1.0e-12){
        requestControlledStop(
                    realFtRuntime ?
                        QStringLiteral("达到最长有效交互时间") :
                        QStringLiteral("达到最长空载调试时间"),
                    false,
                    ForceInteractionControlledStopCause::DurationReached);
        braking = true;
    }

    if(!braking){
        QString inputError;
        const bool inputValid = config_.wrenchSourceKind ==
                ForceInteractionWrenchSourceKind::RealFtTrace ?
                    realFtSample(feedback, nowUs, sensorSample,
                                 ftSampleAgeUs, &inputError) :
                    ((sensorSample = wrenchSource_.sample(stamp, elapsedS)).valid);
        if(!inputValid){
            if(inputError.isEmpty()){
                inputError = QStringLiteral("六维力输入无效");
            }
            if(actualStartCaptured_ && requestControlledStop(
                        QStringLiteral("%1六维力输入失效：%2")
                            .arg(runtimeStageName(config_.wrenchSourceKind), inputError),
                        true,
                        ForceInteractionControlledStopCause::ForceSensorInput)){
                braking = true;
            }
            else{
                output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
                output.reason = QStringLiteral("%1力输入不可用：%2")
                        .arg(runtimeStageName(config_.wrenchSourceKind), inputError);
                return output;
            }
        }
    }

    if(braking){
        bool stopped = false;
        QString brakingError;
        for(int substep = 0; substep < integrationSteps && !stopped; ++substep){
            desired = advanceBrakingState(stopped, &brakingError);
            if(brakingError.isEmpty()){
                ++modelStepCount_;
            }
        }
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
        const WrenchTransformResult transformed =
                wrenchTransformer_->toPlatformCenterOfMass(sensorSample);
        if(!transformed.sample.valid){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = transformed.errorMessage.isEmpty() ?
                        QStringLiteral("%1力旋量转换失败")
                            .arg(runtimeStageName(config_.wrenchSourceKind)) :
                        transformed.errorMessage;
            return output;
        }
        platformSample = transformed.sample;
        conditioningResult.unfiltered = platformSample.wrench;
        if(config_.translationOnly){
            platformSample.wrench[3] = 0.0;
            platformSample.wrench[4] = 0.0;
            platformSample.wrench[5] = 0.0;
        }
        if(config_.wrenchSourceKind ==
                ForceInteractionWrenchSourceKind::RealFtTrace){
            conditioningResult = wrenchConditioner_.process(platformSample.wrench);
            if(!conditioningResult.valid){
                output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
                output.reason = QStringLiteral("阶段C真实F/T输入调理失败");
                return output;
            }
            platformSample.wrench = conditioningResult.output;
            if(realFtInteractionStartUs_ <= 0 &&
                    (conditioningResult.forceActive ||
                     conditioningResult.torqueActive)){
                // Stage C may wait indefinitely for the operator's first
                // intentional contact. Start its duration budget only after
                // the conditioned wrench has actually opened a gate.
                realFtInteractionStartUs_ = nowUs;
                status_.interactionTriggered = true;
                status_.interactionElapsedS = 0.0;
                status_.message = QStringLiteral(
                            "阶段C已检测到首次有效受力，试验计时开始");
            }
        }
        else{
            conditioningResult.filtered = platformSample.wrench;
            conditioningResult.output = platformSample.wrench;
            conditioningResult.forceNormN = vectorNorm3(platformSample.wrench);
            conditioningResult.torqueNormNm = vectorNorm3(platformSample.wrench, 3);
            conditioningResult.forceActive = true;
            conditioningResult.torqueActive = !config_.translationOnly;
            conditioningResult.valid = true;
        }
        // 当前可用力样本在本次补算窗口内按 ZOH 保持；只补数学状态，
        // 不补发已经过期的速度命令。
        for(int substep = 0; substep < integrationSteps; ++substep){
            dynamicsResult = dynamics_.step(platformSample,
                                            config_.periodUs / 1000000.0);
        if(!dynamicsResult.valid){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral("%1 Newmark失败：%2")
                    .arg(runtimeStageName(config_.wrenchSourceKind),
                         dynamicsResult.errorMessage);
            return output;
        }
            desired = dynamicsResult.state;
            ++modelStepCount_;
        }
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

    const double modelElapsedS = static_cast<double>(modelStepCount_) *
            config_.periodUs / 1000000.0;
    const qint64 modelLagUs = std::max<qint64>(
                0, hostElapsedUs - static_cast<qint64>(modelStepCount_) *
                config_.periodUs);

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
        output.reason = QStringLiteral("%1末端状态触发物理边界急停：%2")
                .arg(runtimeStageName(config_.wrenchSourceKind),
                     workspaceResult.reason);
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
        output.reason = QStringLiteral("%1逆运动学失败：%2")
                .arg(runtimeStageName(config_.wrenchSourceKind),
                     evaluation.errorMessage);
        return output;
    }

    OnlineVelocityAxisArray reference{};
    OnlineVelocityAxisArray referenceVelocity{};
    OnlineVelocityAxisArray correction{};
    OnlineVelocityAxisArray command{};
    OnlineVelocityAxisArray relativeCommandPosition{};
    OnlineVelocityAxisArray safetyRelativeReference{};
    OnlineVelocityAxisArray alignedReferenceForRecord{};
    OnlineVelocityAxisArray alignedErrorForRecord{};
    std::array<int, kOnlineVelocityAxisCount> alignedValidForRecord{};
    alignedReferenceForRecord.fill(std::numeric_limits<double>::quiet_NaN());
    alignedErrorForRecord.fill(std::numeric_limits<double>::quiet_NaN());
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        relativeCommandPosition[axis] =
                evaluation.relativeMotorThetaRad[axis] *
                config_.motorUnitPerRadian[axis];
    }
    QString motorTravelError;
    if(!validateMotorSafetyRelativeTravel(
            config_, actualStartSafetyRelativePosition_,
            feedback.safetyRelativePosition, relativeCommandPosition,
            &safetyRelativeReference, &motorTravelError)){
        output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
        output.reason = QStringLiteral("%1绞盘行程保护：%2")
                .arg(runtimeStageName(config_.wrenchSourceKind),
                     motorTravelError);
        return output;
    }
    const double modelAdvanceS = std::max(
                0.0, modelElapsedS - modelElapsedBeforeS);
    double maximumError = 0.0;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        reference[axis] = actualStartPosition_[axis] +
                relativeCommandPosition[axis];
        referenceVelocity[axis] = modelAdvanceS > 1.0e-12 ?
                    (reference[axis] - lastReferencePosition_[axis]) /
                    modelAdvanceS : 0.0;
    }
    referenceHistory_.push_back(
                ReferenceHistorySample{modelElapsedS, reference});
    while(referenceHistory_.size() > 2 &&
          modelElapsedS - referenceHistory_[1].elapsedS > 5.0){
        referenceHistory_.pop_front();
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        double alignedReference = 0.0;
        const bool aligned = alignedReferenceAt(
                    axis, feedback.logicalFrameSequence,
                    feedback.traceSamplePeriodUs, &alignedReference);
        const double error = aligned
                ? alignedReference - feedback.actualPosition[axis] : 0.0;
        if(aligned){
            alignedReferenceForRecord[axis] = alignedReference;
            alignedErrorForRecord[axis] = error;
            alignedValidForRecord[axis] = 1;
        }
        maximumError = std::max(maximumError, std::fabs(error));
        if(aligned && std::fabs(error) > config_.followingErrorLimit){
            output.action = ForceInteractionRuntimeStep::Action::EmergencyStop;
            output.reason = QStringLiteral("%1轴%2延迟对齐位置跟随误差%3超限%4（标定延迟=%5 ms）")
                    .arg(runtimeStageName(config_.wrenchSourceKind)).arg(axis)
                    .arg(error, 0, 'f', 6)
                    .arg(config_.followingErrorLimit, 0, 'f', 6)
                    .arg(config_.traceDelayMs[axis], 0, 'f', 4);
            return output;
        }
        if(config_.pidEnabled && aligned){
            integral_[axis] = clampValue(integral_[axis] + error * commandDt,
                                         config_.integralLimit);
            const double derivative = previousErrorValid_ ?
                        (error - previousError_[axis]) / commandDt : 0.0;
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
    // 每轴只截断自身超限速度，不改变其余轴的动力学响应。理想参考轨迹保持不变，
    // 限幅造成的执行偏差由延迟对齐跟随误差保护负责检出并停止试验。
    for(double& value : command){
        value = std::clamp(value,
                           -config_.velocityLimit,
                           config_.velocityLimit);
    }
    previousErrorValid_ = true;
    kinematicsState_ = evaluation.nextState;
    lastReferencePosition_ = reference;

    ForceInteractionRunRecord record;
    record.stepIndex = status_.stepCount + 1;
    record.elapsedS = elapsedS;
    record.modelElapsedS = modelElapsedS;
    record.modelLagUs = modelLagUs;
    record.integrationSteps = integrationSteps;
    record.stamp = stamp;
    record.availabilityMask = ForceRecordDesiredState |
            ForceRecordCableKinematics | ForceRecordAxisReference |
            ForceRecordAxisCommand | ForceRecordAxisTrace | ForceRecordTiming;
    if(!braking){
        record.availabilityMask |= ForceRecordSensorWrench |
                ForceRecordPlatformWrench;
    }
    record.sensorWrench = sensorSample.wrench;
    record.platformWrenchUnfiltered = conditioningResult.unfiltered;
    record.platformWrenchFiltered = conditioningResult.filtered;
    record.platformWrench = platformSample.wrench;
    record.forceGateActive = conditioningResult.forceActive;
    record.torqueGateActive = conditioningResult.torqueActive;
    if(config_.wrenchSourceKind ==
            ForceInteractionWrenchSourceKind::RealFtTrace){
        record.ftEngineeringValue = feedback.ftSensor.value;
        record.ftSoftwareZero = config_.ftSoftwareZero;
        for(int channel = 0; channel < kForceInteractionDofCount; ++channel){
            record.ftZeroCorrected[static_cast<size_t>(channel)] =
                    feedback.ftSensor.value[static_cast<size_t>(channel)] -
                    config_.ftSoftwareZero[static_cast<size_t>(channel)];
        }
        record.ftStatusCode = feedback.ftSensor.statusCode;
        record.ftSampleCounter = feedback.ftSensor.sampleCounter;
        record.ftTemperatureC = feedback.ftSensor.temperatureValid ?
                    feedback.ftSensor.temperatureC :
                    std::numeric_limits<double>::quiet_NaN();
        if(ftSampleAgeUs < 0 && feedback.ftSensor.monotonicUs > 0){
            ftSampleAgeUs = nowUs >= feedback.ftSensor.monotonicUs ?
                        nowUs - feedback.ftSensor.monotonicUs : 0;
        }
        record.ftSampleAgeUs = ftSampleAgeUs;
        if(feedback.ftSensor.wrenchComplete() &&
                feedback.ftSensor.statusValid &&
                feedback.ftSensor.sampleCounterValid &&
                feedback.ftSensor.traceFrameSequenceValid){
            record.availabilityMask |= ForceRecordFtDiagnostics;
        }
        status_.latestFtStatusCode = feedback.ftSensor.statusCode;
        status_.latestFtSampleCounter = feedback.ftSensor.sampleCounter;
        status_.latestFtTemperatureC = feedback.ftSensor.temperatureValid ?
                    feedback.ftSensor.temperatureC :
                    std::numeric_limits<double>::quiet_NaN();
        status_.latestFtSampleAgeUs = ftSampleAgeUs;
        status_.latestUnfilteredPlatformWrench =
                conditioningResult.unfiltered;
        status_.latestFilteredPlatformWrench =
                conditioningResult.filtered;
        status_.latestAppliedPlatformWrench =
                conditioningResult.output;
        status_.latestForceGateActive = conditioningResult.forceActive;
        status_.latestTorqueGateActive = conditioningResult.torqueActive;
    }
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
        record.axisAlignedReferencePosition[axis] = alignedReferenceForRecord[axis];
        record.axisAlignedFollowingError[axis] = alignedErrorForRecord[axis];
        record.axisAlignedReferenceValid[axis] = alignedValidForRecord[axis];
        record.axisSafetyRelativeReferencePosition[axis] =
                safetyRelativeReference[axis];
        record.axisReferenceVelocity[axis] = referenceVelocity[axis];
        record.axisPidCorrectionVelocity[axis] = correction[axis];
        record.axisCommandVelocity[axis] = command[axis];
        record.axisTracePosition[axis] = feedback.actualPosition[axis];
        record.axisSafetyRelativeTracePosition[axis] =
                feedback.safetyRelativePosition[axis];
        record.axisTraceVelocity[axis] = feedback.actualVelocity[axis];
        record.axisStatusWord[axis] = feedback.motorStatusWord[axis];
        record.axisStateMachine[axis] = feedback.motorStateMachine[axis];
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
    status_.modelElapsedS = modelElapsedS;
    status_.modelLagUs = modelLagUs;
    status_.latestIntegrationSteps = integrationSteps;
    status_.maximumIntegrationSteps = std::max(
                status_.maximumIntegrationSteps, integrationSteps);
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
    status_.safetyRelativeReferencePosition = safetyRelativeReference;
    status_.safetyRelativeActualPosition = feedback.safetyRelativePosition;
    status_.actualPosition = feedback.actualPosition;
    status_.commandVelocity = command;
    status_.motorStatusWord = feedback.motorStatusWord;
    status_.motorStateMachine = feedback.motorStateMachine;
    lastCommandUs_ = nowUs;
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
                        QStringLiteral("%1八轴速度API调用失败")
                            .arg(runtimeStageName(config_.wrenchSourceKind)) :
                        QStringLiteral("%1停机API调用失败，已升级立即停止")
                            .arg(runtimeStageName(config_.wrenchSourceKind)));
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
    if(state == ForceInteractionRuntimeStatus::State::Fault){
        status_.experimentValid = false;
        if(status_.safetyStopReason.isEmpty()){
            status_.safetyStopReason = message.isEmpty() ?
                        QStringLiteral("%1发生未分类故障")
                            .arg(runtimeStageName(config_.wrenchSourceKind)) : message;
        }
    }
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
    ForceInteractionRunTerminalSummary terminalSummary;
    terminalSummary.present = true;
    terminalSummary.terminalState = static_cast<int>(status_.state);
    terminalSummary.controlledStopCause =
            static_cast<int>(status_.controlledStopCause);
    terminalSummary.experimentValid = status_.experimentValid;
    terminalSummary.finalStepCount = status_.stepCount;
    terminalSummary.finalCommandCount = status_.commandCount;
    terminalSummary.missedCycleCount = status_.missedCycleCount;
    terminalSummary.elapsedS = status_.elapsedS;
    terminalSummary.minimumWorkspaceClearanceMm =
            std::isfinite(status_.minimumWorkspaceClearanceMm) ?
                status_.minimumWorkspaceClearanceMm : 0.0;
    terminalSummary.terminalReason = status_.message;
    terminalSummary.safetyStopReason = status_.safetyStopReason;
    recorder_->setTerminalSummary(terminalSummary);
    recorder_->requestFinish();
    recorder_->finishAndWait();
    status_.acceptedRecordCount = recorder_->acceptedCount();
    status_.writtenRecordCount = recorder_->writtenCount();
    status_.droppedRecordCount = recorder_->droppedCount();
    status_.recordingError = recorder_->writerError();
    recorder_.reset();
}

void ForceInteractionRuntimeControl::resetSession()
{
    if(isActive() || isPrepared()){
        return;
    }
    finishRecording();
    status_ = ForceInteractionRuntimeStatus{};
    config_ = ForceInteractionRuntimeConfig{};
    wrenchTransformer_.reset();
    recorder_.reset();
    actualStartPosition_.fill(0.0);
    actualStartSafetyRelativePosition_.fill(0.0);
    lastReferencePosition_.fill(0.0);
    integral_.fill(0.0);
    previousError_.fill(0.0);
    referenceHistory_.clear();
    actualStartCaptured_ = false;
    previousErrorValid_ = false;
    waitStartUs_ = 0;
    lastGoodTraceUs_ = 0;
    nextDueUs_ = 0;
    hostStartUs_ = 0;
    realFtInteractionStartUs_ = 0;
    lastCommandUs_ = 0;
    modelStepCount_ = 0;
    startTraceSequence_ = 0;
    lastFrameSequence_ = 0;
    lastFrameSequenceValid_ = false;
    lastFtSampleCounter_ = 0;
    lastFtCounterChangeUs_ = 0;
    lastFtSampleCounterValid_ = false;
    brakingState_ = ForceInteractionPlatformState{};
    controlledStopReason_.clear();
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

bool ForceInteractionRuntimeControl::runControlledStopSelfChecks(
        const PhysicalWorkspaceBoundaryConfig& physicalWorkspace,
        QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    QString validationError;
    if(!physicalWorkspace.validate(&validationError)){
        return fail(QStringLiteral("协同制动自检的物理边界无效：%1")
                    .arg(validationError));
    }

    ForceInteractionRuntimeControl control;
    control.config_.periodUs = 5000;
    control.config_.translationOnly = true;
    control.config_.physicalWorkspace = physicalWorkspace;
    control.config_.workspaceSafety.stoppingDecelerationMmPerSec2 = 100.0;
    control.config_.workspaceSafety.additionalSafetyMarginMm = 60.0;
    control.config_.workspaceSafety.emergencyLineMarginMm = 10.0;
    control.config_.brakingStopVelocityMmPerSec = 0.0;

    ForceInteractionPlatformState initial;
    for(int axis = 0; axis < 3; ++axis){
        initial.pose[static_cast<size_t>(axis)] = 0.0005 *
                (physicalWorkspace.frameMinimumMm[axis] +
                 physicalWorkspace.frameMaximumMm[axis]);
    }
    initial.poseValid = true;
    initial.twistValid = true;
    initial.accelerationValid = true;
    initial.twist[0] = 0.1; // 100 mm/s

    const auto arm = [&control, &initial](
            ForceInteractionControlledStopCause cause,
            bool experimentFailure,
            const QString& reason){
        control.status_ = ForceInteractionRuntimeStatus{};
        control.status_.state = ForceInteractionRuntimeStatus::State::Running;
        control.status_.desiredState = initial;
        control.status_.experimentValid = true;
        control.brakingState_ = {};
        control.controlledStopReason_.clear();
        return control.requestControlledStop(reason, experimentFailure, cause);
    };

    const struct StopCase {
        ForceInteractionControlledStopCause cause;
        bool failure;
        const char* name;
    } stopCases[] = {
        {ForceInteractionControlledStopCause::UserRequest, false, "用户停止"},
        {ForceInteractionControlledStopCause::DurationReached, false, "时长到达"},
        {ForceInteractionControlledStopCause::WorkspaceBoundary, true, "动态边界"}
    };
    for(const StopCase& stopCase : stopCases){
        const QString name = QString::fromUtf8(stopCase.name);
        if(!arm(stopCase.cause, stopCase.failure, name) ||
                control.status_.state !=
                    ForceInteractionRuntimeStatus::State::Braking ||
                control.status_.controlledStopCause != stopCase.cause ||
                control.status_.experimentValid == stopCase.failure ||
                (stopCase.failure && control.status_.safetyStopReason != name)){
            return fail(QStringLiteral("%1未正确进入协同制动或试验有效性错误")
                        .arg(name));
        }
    }

    // A safety failure arriving after a benign stop request must replace the
    // benign cause in the final diagnostic; it must never leave “valid=yes”.
    if(!arm(ForceInteractionControlledStopCause::UserRequest, false,
            QStringLiteral("用户停止")) ||
            !control.requestControlledStop(
                QStringLiteral("制动期间到达动态边界"), true,
                ForceInteractionControlledStopCause::WorkspaceBoundary) ||
            control.status_.experimentValid ||
            control.status_.controlledStopCause !=
                ForceInteractionControlledStopCause::WorkspaceBoundary ||
            control.status_.safetyStopReason !=
                QStringLiteral("制动期间到达动态边界")){
        return fail(QStringLiteral("协同制动期间的安全原因升级未锁存"));
    }

    if(!arm(ForceInteractionControlledStopCause::DurationReached, false,
            QStringLiteral("时长到达"))){
        return fail(QStringLiteral("无法进入平动协同制动自检"));
    }
    const double startX = control.brakingState_.pose[0];
    double previousSpeed = vectorNorm3(control.brakingState_.twist);
    bool stopped = false;
    int brakingSteps = 0;
    for(; brakingSteps < 10000 && !stopped; ++brakingSteps){
        QString brakingError;
        const ForceInteractionPlatformState next =
                control.advanceBrakingState(stopped, &brakingError);
        const double speed = vectorNorm3(next.twist);
        const double acceleration = vectorNorm3(next.acceleration);
        if(!brakingError.isEmpty() || !next.poseValid || !next.twistValid ||
                !next.accelerationValid || speed > previousSpeed + 1.0e-12 ||
                next.twist[0] < -1.0e-12 ||
                acceleration > 0.100000001){
            return fail(QStringLiteral("平动协同制动未保持单调减速或超出配置减速度"));
        }
        previousSpeed = speed;
    }
    if(!stopped || brakingSteps != 200 ||
            std::abs(control.brakingState_.pose[0] - startX - 0.05) > 1.0e-10 ||
            vectorNorm3(control.brakingState_.twist) > 1.0e-12){
        return fail(QStringLiteral(
                    "平动协同制动距离/步数错误：步数=%1，位移=%2 m")
                    .arg(brakingSteps)
                    .arg(control.brakingState_.pose[0] - startX, 0, 'g', 12));
    }

    // 绞盘限位必须使用安全相对坐标，而不能把绝对编码器位置与±圈数直接比较。
    // 用非零起点验证“本次运行增量 + 已锁存安全相对起点”的精确关系。
    ForceInteractionRuntimeConfig travelConfig;
    travelConfig.motorSafetyRelativeMinimum.fill(-6.5);
    travelConfig.motorSafetyRelativeMaximum.fill(6.5);
    OnlineVelocityAxisArray safetyStart{};
    OnlineVelocityAxisArray safetyActual{};
    OnlineVelocityAxisArray relativeCommand{};
    OnlineVelocityAxisArray safetyReference{};
    safetyStart.fill(2.0);
    safetyActual.fill(2.0);
    relativeCommand.fill(4.4);
    QString travelError;
    if(!validateMotorSafetyRelativeTravel(
            travelConfig, safetyStart, safetyActual, relativeCommand,
            &safetyReference, &travelError) ||
            std::fabs(safetyReference[0] - 6.4) > 1.0e-12){
        return fail(QStringLiteral("绞盘安全相对位置有效区间自检失败：%1")
                    .arg(travelError));
    }
    relativeCommand[3] = 4.6;
    if(validateMotorSafetyRelativeTravel(
            travelConfig, safetyStart, safetyActual, relativeCommand,
            &safetyReference, &travelError) || !travelError.contains("轴3")){
        return fail(QStringLiteral("绞盘期望安全相对位置越界未被拒绝"));
    }
    relativeCommand.fill(0.0);
    safetyActual[5] = -6.6;
    if(validateMotorSafetyRelativeTravel(
            travelConfig, safetyStart, safetyActual, relativeCommand,
            &safetyReference, &travelError) || !travelError.contains("轴5")){
        return fail(QStringLiteral("绞盘实际安全相对位置越界未被拒绝"));
    }

    // 阶段C真实F/T入口必须与八轴反馈来自同一Trace帧，并在状态异常或
    // SampleCounter停滞时拒绝继续产生动力学输入。此自检不访问任何硬件。
    control.config_.wrenchSourceKind =
            ForceInteractionWrenchSourceKind::RealFtTrace;
    control.config_.ftSampleTimeoutUs = 50000;
    control.config_.ftStatusMask = 0xffffffffu;
    control.config_.ftExpectedStatus = 0u;
    control.lastFtSampleCounterValid_ = false;
    control.lastFtCounterChangeUs_ = 0;
    ForceInteractionRuntimeFeedback ftFeedback;
    ftFeedback.ftRuntimeProfileActive = true;
    ftFeedback.traceFrameSequence = 123u;
    ftFeedback.logicalFrameSequence = 1000u;
    ftFeedback.traceSamplePeriodUs = 1000;
    ftFeedback.monotonicUs = 1000000;
    ftFeedback.ftSensor.traceFrameSequence = 123u;
    ftFeedback.ftSensor.traceFrameSequenceValid = true;
    ftFeedback.ftSensor.monotonicUs = ftFeedback.monotonicUs;
    ftFeedback.ftSensor.statusCode = 0u;
    ftFeedback.ftSensor.statusValid = true;
    ftFeedback.ftSensor.sampleCounter = 55u;
    ftFeedback.ftSensor.sampleCounterValid = true;
    ftFeedback.ftSensor.temperatureC = 25.0;
    ftFeedback.ftSensor.temperatureValid = true;
    for(int channel = 0; channel < kForceInteractionDofCount; ++channel){
        ftFeedback.ftSensor.value[static_cast<size_t>(channel)] =
                static_cast<double>(channel + 1);
        ftFeedback.ftSensor.channelValid[static_cast<size_t>(channel)] = true;
    }
    ForceInteractionWrenchSample ftSample;
    qint64 ftAgeUs = -1;
    QString ftError;
    if(!control.realFtSample(ftFeedback, 1001000, ftSample, ftAgeUs, &ftError) ||
            !ftSample.valid ||
            ftSample.coordinate != ForceInteractionWrenchCoordinate::Sensor ||
            ftAgeUs != 1000 || ftSample.wrench != ftFeedback.ftSensor.value){
        return fail(QStringLiteral("阶段C有效同帧F/T样本未被接受：%1")
                    .arg(ftError));
    }
    ftFeedback.ftSensor.statusCode = 1u;
    if(control.realFtSample(ftFeedback, 1002000, ftSample, ftAgeUs, &ftError) ||
            !ftError.contains(QStringLiteral("状态码"))){
        return fail(QStringLiteral("阶段C非零F/T状态码未被拒绝"));
    }
    ftFeedback.ftSensor.statusCode = 0u;
    ftFeedback.ftSensor.traceFrameSequence = 124u;
    if(control.realFtSample(ftFeedback, 1002000, ftSample, ftAgeUs, &ftError) ||
            !ftError.contains(QStringLiteral("同一Trace帧"))){
        return fail(QStringLiteral("阶段C错帧F/T样本未被拒绝"));
    }
    ftFeedback.ftSensor.traceFrameSequence = ftFeedback.traceFrameSequence;
    ftFeedback.traceFrameSequence = 124u;
    ftFeedback.logicalFrameSequence = 1061u;
    ftFeedback.monotonicUs = 1061000;
    ftFeedback.ftSensor.traceFrameSequence = ftFeedback.traceFrameSequence;
    ftFeedback.ftSensor.monotonicUs = ftFeedback.monotonicUs;
    if(control.realFtSample(ftFeedback, 1062000, ftSample, ftAgeUs, &ftError) ||
            !ftError.contains(QStringLiteral("SampleCounter"))){
        return fail(QStringLiteral("阶段C停滞的F/T SampleCounter未触发超时"));
    }

    // A terminal fault from Trace/API/boundary paths must always invalidate
    // the experiment and retain a useful first failure reason.
    control.status_ = ForceInteractionRuntimeStatus{};
    control.status_.experimentValid = true;
    control.setTerminal(ForceInteractionRuntimeStatus::State::Fault,
                        QStringLiteral("Trace失效自检"));
    if(control.status_.experimentValid ||
            control.status_.safetyStopReason != QStringLiteral("Trace失效自检")){
        return fail(QStringLiteral("阶段B故障未使试验无效或未锁存原因"));
    }

    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}
