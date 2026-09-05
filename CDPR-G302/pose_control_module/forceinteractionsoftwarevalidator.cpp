#include "forceinteractionsoftwarevalidator.h"

#include "forceinteractionrunrecorder.h"
#include "forwardkinematicssolver.h"
#include "wrenchtransformer.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrappedAngleDifference(double left, double right)
{
    return std::remainder(left - right, 2.0 * kPi);
}

double norm3(double x, double y, double z)
{
    return std::sqrt(x * x + y * y + z * z);
}

CompensatedCableKinematics::PoseMatrix toPoseMmRad(
        const ForceInteractionPlatformState& state)
{
    return {{
        state.pose[0] * 1000.0,
        state.pose[1] * 1000.0,
        state.pose[2] * 1000.0,
        state.pose[3], state.pose[4], state.pose[5]
    }};
}

bool runCoreSelfChecks(const ForceInteractionValidationConfig& configuration,
                       QString* errorMessage)
{
    // 自检和正式仿真必须使用同一份冻结安装参数，避免出现“自检使用实测
    // 力臂、正式循环却把模拟量直接当作质心力旋量”的双重语义。
    WrenchTransformer transformer(configuration.sensorTransform);
    ForceInteractionWrenchSample sensor;
    sensor.stamp.valid = true;
    sensor.valid = true;
    sensor.coordinate = ForceInteractionWrenchCoordinate::Sensor;
    sensor.wrench = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    const WrenchTransformResult transformed =
            transformer.toPlatformCenterOfMass(sensor);
    if(!transformed.sample.valid ||
            std::abs(transformed.sample.wrench[0] - 1.0) > 1.0e-12 ||
            std::abs(transformed.sample.wrench[4] -
                     configuration.sensorTransform.sensorOriginInPlatformM[2]) > 1.0e-12){
        if(errorMessage){
            *errorMessage = QStringLiteral("实测F/T力臂的力旋量平移自检失败");
        }
        return false;
    }

    // 使用确定的对角惯量完成解析解自检，避免把用户当前模板参数本身当成
    // 数值基准；用户质量/惯量的合法性仍会在正式仿真配置时单独检查。
    ForceInteractionRigidBodyConfig selfTestRigidBody;
    selfTestRigidBody.massKg = 2.0;
    selfTestRigidBody.inertiaKgM2 = {{
        1.0, 0.0, 0.0,
        0.0, 2.0, 0.0,
        0.0, 0.0, 3.0
    }};
    CdprDynamics dynamics;
    QString error;
    if(!dynamics.configure(selfTestRigidBody, configuration.newmark, &error)){
        if(errorMessage){
            *errorMessage = QStringLiteral("动力学配置自检失败：%1").arg(error);
        }
        return false;
    }
    ForceInteractionPlatformState origin;
    origin.poseValid = true;
    origin.twistValid = true;
    origin.accelerationValid = true;
    if(!dynamics.reset(origin, &error)){
        if(errorMessage){
            *errorMessage = QStringLiteral("动力学复位自检失败：%1").arg(error);
        }
        return false;
    }
    ForceInteractionWrenchSample force;
    force.stamp.valid = true;
    force.valid = true;
    force.coordinate = ForceInteractionWrenchCoordinate::PlatformBodyAtCenterOfMass;
    const CdprDynamicsStepResult zeroStep = dynamics.step(force, 0.001);
    if(!zeroStep.valid || norm3(zeroStep.state.pose[0], zeroStep.state.pose[1],
                                zeroStep.state.pose[2]) > 1.0e-14){
        if(errorMessage){
            *errorMessage = QStringLiteral("零力静止自检失败");
        }
        return false;
    }
    if(!dynamics.reset(origin, &error)){
        if(errorMessage){
            *errorMessage = QStringLiteral("恒力自检复位失败：%1").arg(error);
        }
        return false;
    }
    force.wrench[0] = 1.0;
    constexpr double selfTestDt = 0.001;
    constexpr int selfTestSteps = 100;
    CdprDynamicsStepResult step;
    for(int index = 0; index < selfTestSteps; ++index){
        step = dynamics.step(force, selfTestDt);
        if(!step.valid){
            if(errorMessage){
                *errorMessage = QStringLiteral("恒力Newmark自检失败：%1")
                        .arg(step.errorMessage);
            }
            return false;
        }
    }
    const double elapsedS = selfTestDt * selfTestSteps;
    const double expectedVelocity = elapsedS / selfTestRigidBody.massKg;
    const double expectedPosition = 0.5 * elapsedS * elapsedS /
            selfTestRigidBody.massKg;
    if(std::abs(step.state.twist[0] - expectedVelocity) > 1.0e-9 ||
            std::abs(step.state.pose[0] - expectedPosition) > 1.0e-9){
        if(errorMessage){
            *errorMessage = QStringLiteral("恒力解析解自检失败：x/v=%1/%2，期望=%3/%4")
                    .arg(step.state.pose[0], 0, 'g', 12)
                    .arg(step.state.twist[0], 0, 'g', 12)
                    .arg(expectedPosition, 0, 'g', 12)
                    .arg(expectedVelocity, 0, 'g', 12);
        }
        return false;
    }

    if(!dynamics.reset(origin, &error)){
        if(errorMessage){
            *errorMessage = QStringLiteral("恒力矩自检复位失败：%1").arg(error);
        }
        return false;
    }
    force.wrench = {};
    force.wrench[3] = 1.0;
    for(int index = 0; index < selfTestSteps; ++index){
        step = dynamics.step(force, selfTestDt);
        if(!step.valid){
            if(errorMessage){
                *errorMessage = QStringLiteral("恒力矩Newmark自检失败：%1")
                        .arg(step.errorMessage);
            }
            return false;
        }
    }
    if(std::abs(step.state.twist[3] - elapsedS) > 1.0e-9 ||
            std::abs(step.state.pose[3] - 0.5 * elapsedS * elapsedS) > 1.0e-9){
        if(errorMessage){
            *errorMessage = QStringLiteral("恒力矩解析解自检失败：rx/wx=%1/%2")
                    .arg(step.state.pose[3], 0, 'g', 12)
                    .arg(step.state.twist[3], 0, 'g', 12);
        }
        return false;
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

QString failureSummary(const QString& stage, const QString& error)
{
    return QStringLiteral("阶段A软件验证未通过\n失败环节：%1\n原因：%2\n未调用任何雷赛运动API。")
            .arg(stage, error);
}

} // namespace

ForceInteractionValidationResult ForceInteractionSoftwareValidator::run(
        const ForceInteractionValidationConfig& configuration,
        const std::atomic_bool* cancellationRequested)
{
    ForceInteractionValidationResult result;
    if(!std::isfinite(configuration.controlPeriodS) ||
            configuration.controlPeriodS <= 0.0 ||
            !std::isfinite(configuration.durationS) ||
            configuration.durationS <= 0.0 ||
            configuration.initialPoseMmRad.size() != 1 ||
            configuration.machineTemplateName.trimmed().isEmpty() ||
            !configuration.sensorTransform.configured){
        result.summary = failureSummary(QStringLiteral("输入校验"),
                                        QStringLiteral("G302模板、F/T安装参数、周期、时长或单末端初始位姿无效"));
        return result;
    }

    QString error;
    if(!runCoreSelfChecks(configuration, &error)){
        result.summary = failureSummary(QStringLiteral("核心数学自检"), error);
        return result;
    }

    SimulatedWrenchSource wrenchSource;
    if(!wrenchSource.configure(configuration.wrenchProfile,
                               configuration.controlPeriodS, &error)){
        result.summary = failureSummary(QStringLiteral("模拟力源"), error);
        return result;
    }
    CdprDynamics dynamics;
    if(!dynamics.configure(configuration.rigidBody,
                           configuration.newmark, &error) ||
            !dynamics.reset(configuration.initialState, &error)){
        result.summary = failureSummary(QStringLiteral("Newmark配置/复位"), error);
        return result;
    }
    CompensatedCableKinematics kinematics;
    if(!kinematics.initialize(configuration.kinematics,
                              configuration.initialPoseMmRad, {}, &error)){
        result.summary = failureSummary(QStringLiteral("补偿逆运动学初始化"), error);
        return result;
    }
    if(kinematics.axisCount() != kForceInteractionCableCount ||
            configuration.kinematics.endCableContactPos.size() != 1){
        result.summary = failureSummary(
                    QStringLiteral("运动学配置"),
                    QStringLiteral("阶段A要求单末端、完整8绳配置"));
        return result;
    }

    const int stepCount = std::max(1, static_cast<int>(
            std::ceil(configuration.durationS / configuration.controlPeriodS)));
    CompensatedCableKinematics::State kinematicsState = kinematics.initialState();
    ForwardKinematicsSolver forwardSolver;
    forwardSolver.setInitialPose(configuration.initialPoseMmRad.front());
    WrenchTransformer wrenchTransformer(configuration.sensorTransform);

    ForceInteractionRunMetadata recordingMetadata;
    recordingMetadata.stage = QStringLiteral("stage_a");
    recordingMetadata.sourceName = wrenchSource.summary();
    recordingMetadata.machineTemplateName = configuration.machineTemplateName;
    recordingMetadata.controlPeriodS = configuration.controlPeriodS;
    recordingMetadata.plannedDurationS = configuration.durationS;
    ForceInteractionRunRecorder recorder;
    if(!recorder.begin(configuration.recordingDirectory, recordingMetadata,
                       &result.recordingPath, &error)){
        result.summary = failureSummary(QStringLiteral("逐步数据记录器"), error);
        return result;
    }
    const auto finishRecording = [&]() {
        recorder.finishAndWait();
        result.acceptedRecords = recorder.acceptedCount();
        result.writtenRecords = recorder.writtenCount();
        result.droppedRecords = recorder.droppedCount();
        result.recordingError = recorder.writerError();
    };
    const auto appendRecordingSummary = [&]() {
        result.summary += QStringLiteral(
                    "\n逐步CSV：%1\n记录接受/写入/丢弃=%2/%3/%4%5")
                .arg(result.recordingPath)
                .arg(result.acceptedRecords)
                .arg(result.writtenRecords)
                .arg(result.droppedRecords)
                .arg(result.recordingError.isEmpty() ? QString() :
                     QStringLiteral("；写盘错误=%1").arg(result.recordingError));
    };

    constexpr double translationToleranceMm = 0.1;
    constexpr double orientationToleranceDeg = 0.01;
    constexpr double cableResidualToleranceMm = 0.1;
    QElapsedTimer validationRuntime;
    validationRuntime.start();

    for(int stepIndex = 1; stepIndex <= stepCount; ++stepIndex){
        if(cancellationRequested && cancellationRequested->load()){
            result.cancelled = true;
            result.summary = QStringLiteral("阶段A软件验证已取消；完成%1/%2个数学步，未调用任何雷赛运动API。")
                    .arg(result.completedSteps).arg(stepCount);
            finishRecording();
            appendRecordingSummary();
            return result;
        }
        QElapsedTimer stepTimer;
        stepTimer.start();
        const double elapsedS = std::min(configuration.durationS,
                                         stepIndex * configuration.controlPeriodS);
        ForceInteractionFrameStamp stamp;
        // elapsedS 是可复现的仿真时间；hostMonotonicTimeUs 只记录本次后台
        // 运算真实经过的单调时钟，二者不可混作同一时间基准。
        stamp.hostMonotonicTimeUs = validationRuntime.nsecsElapsed() / 1000;
        stamp.valid = true;
        const ForceInteractionWrenchSample sensorWrench =
                wrenchSource.sample(stamp, elapsedS);
        if(!sensorWrench.valid){
            result.summary = failureSummary(QStringLiteral("模拟力求值"),
                                            QStringLiteral("产生无效力旋量"));
            finishRecording();
            appendRecordingSummary();
            return result;
        }
        WrenchTransformResult transformed =
                wrenchTransformer.toPlatformCenterOfMass(sensorWrench);
        if(!transformed.sample.valid){
            result.summary = failureSummary(
                        QStringLiteral("F/T安装变换"),
                        transformed.errorMessage.isEmpty() ?
                            QStringLiteral("无法得到动平台质心处力旋量") :
                            transformed.errorMessage);
            finishRecording();
            appendRecordingSummary();
            return result;
        }
        if(configuration.translationOnly){
            transformed.sample.wrench[3] = 0.0;
            transformed.sample.wrench[4] = 0.0;
            transformed.sample.wrench[5] = 0.0;
        }
        const CdprDynamicsStepResult dynamicsStep =
                dynamics.step(transformed.sample, configuration.controlPeriodS);
        if(!dynamicsStep.valid){
            result.summary = failureSummary(QStringLiteral("Newmark单步"),
                                            dynamicsStep.errorMessage);
            finishRecording();
            appendRecordingSummary();
            return result;
        }
        result.maximumNewmarkIterations = std::max(
                    result.maximumNewmarkIterations, dynamicsStep.iterations);
        result.maximumNewmarkResidual = std::max(
                    result.maximumNewmarkResidual, dynamicsStep.residual);

        const auto poseMmRad = toPoseMmRad(dynamicsStep.state);
        bool poseOutsideBounds = false;
        if(configuration.poseLowerBoundsMmRad.size() >= 6 &&
                configuration.poseUpperBoundsMmRad.size() >= 6){
            for(int dimension = 0; dimension < 6; ++dimension){
                const double value = poseMmRad[0][static_cast<size_t>(dimension)];
                if(value < configuration.poseLowerBoundsMmRad[static_cast<size_t>(dimension)] ||
                        value > configuration.poseUpperBoundsMmRad[static_cast<size_t>(dimension)]){
                    poseOutsideBounds = true;
                    break;
                }
            }
        }
        if(poseOutsideBounds && result.firstPoseBoundsViolationStep == 0){
            result.firstPoseBoundsViolationStep = stepIndex;
            result.firstPoseBoundsViolationState = dynamicsStep.state;
        }
        const CompensatedCableKinematics::Evaluation evaluation =
                kinematics.evaluatePose(poseMmRad, kinematicsState);
        if(!evaluation.valid ||
                evaluation.relativeMotorThetaRad.size() != kForceInteractionCableCount){
            result.summary = failureSummary(
                        QStringLiteral("补偿逆运动学"),
                        evaluation.errorMessage.isEmpty() ?
                            QStringLiteral("未返回完整8轴结果") : evaluation.errorMessage);
            finishRecording();
            appendRecordingSummary();
            return result;
        }
        kinematicsState = evaluation.nextState;
        for(double angle : evaluation.relativeMotorThetaRad){
            result.maximumRelativeMotorAngleRad = std::max(
                        result.maximumRelativeMotorAngleRad, std::abs(angle));
        }

        {
            ForwardKinematicsSolver::Request request;
            request.anchorPos = configuration.kinematics.anchorCableCoordinate;
            request.contactPointLocal =
                    configuration.kinematics.endCableContactPos.front();
            request.cableLength = evaluation.cableLengthMm;
            request.pulleyRadius = configuration.kinematics.pulleyRadiusMm;
            request.initialPose = forwardSolver.initialPose();
            request.poseLowerBounds = configuration.poseLowerBoundsMmRad;
            request.poseUpperBounds = configuration.poseUpperBoundsMmRad;
            request.keepRotation = true;
            const ForwardKinematicsSolver::Result forward =
                    forwardSolver.solve(request);
            if(!forward.success || forward.pose.size() < 6){
                result.summary = failureSummary(
                            QStringLiteral("正运动学"),
                            QStringLiteral("第%1步未收敛到有限六维位姿").arg(stepIndex));
                finishRecording();
                appendRecordingSummary();
                return result;
            }
            const double translationError = norm3(
                        forward.pose[0] - poseMmRad[0][0],
                        forward.pose[1] - poseMmRad[0][1],
                        forward.pose[2] - poseMmRad[0][2]);
            const double orientationErrorRad = norm3(
                        wrappedAngleDifference(forward.pose[3], poseMmRad[0][3]),
                        wrappedAngleDifference(forward.pose[4], poseMmRad[0][4]),
                        wrappedAngleDifference(forward.pose[5], poseMmRad[0][5]));
            const double orientationErrorDeg = orientationErrorRad * 180.0 / kPi;
            if(translationError > result.maximumTranslationRoundTripErrorMm){
                result.maximumTranslationRoundTripErrorMm = translationError;
                result.maximumTranslationErrorStep = stepIndex;
            }
            if(orientationErrorDeg > result.maximumOrientationRoundTripErrorDeg){
                result.maximumOrientationRoundTripErrorDeg = orientationErrorDeg;
                result.maximumOrientationErrorStep = stepIndex;
            }

            const std::vector<double> reconstructedLengths =
                    kinematics.cableLengthsForPose({forward.pose}, &error);
            if(reconstructedLengths.size() != evaluation.cableLengthMm.size()){
                result.summary = failureSummary(QStringLiteral("正运动学残差"), error);
                finishRecording();
                appendRecordingSummary();
                return result;
            }
            double stepMaximumCableResidualMm = 0.0;
            for(size_t cable = 0; cable < reconstructedLengths.size(); ++cable){
                const double cableResidual =
                        std::abs(reconstructedLengths[cable] -
                                 evaluation.cableLengthMm[cable]);
                stepMaximumCableResidualMm = std::max(
                            stepMaximumCableResidualMm, cableResidual);
                if(cableResidual > result.maximumCableResidualMm){
                    result.maximumCableResidualMm = cableResidual;
                    result.maximumCableResidualStep = stepIndex;
                }
            }
            if(result.firstRoundTripToleranceViolationStep == 0 &&
                    (translationError > translationToleranceMm ||
                     orientationErrorDeg > orientationToleranceDeg ||
                     stepMaximumCableResidualMm > cableResidualToleranceMm)){
                result.firstRoundTripToleranceViolationStep = stepIndex;
                result.firstRoundTripToleranceViolationState = dynamicsStep.state;
            }

            ForceInteractionRunRecord record;
            record.stepIndex = static_cast<quint64>(stepIndex);
            record.elapsedS = elapsedS;
            record.stamp = stamp;
            record.availabilityMask = ForceRecordSensorWrench |
                    ForceRecordPlatformWrench | ForceRecordDesiredState |
                    ForceRecordCableKinematics | ForceRecordForwardKinematics |
                    ForceRecordTiming;
            record.sensorWrench = sensorWrench.wrench;
            record.platformWrench = transformed.sample.wrench;
            record.desiredState = dynamicsStep.state;
            std::copy_n(evaluation.cableLengthMm.begin(),
                        kForceInteractionCableCount,
                        record.cableLengthMm.begin());
            std::copy_n(evaluation.relativeMotorThetaRad.begin(),
                        kForceInteractionCableCount,
                        record.relativeMotorThetaRad.begin());
            std::copy_n(forward.pose.begin(), kForceInteractionDofCount,
                        record.forwardPoseMmRad.begin());
            record.translationRoundTripErrorMm = translationError;
            record.orientationRoundTripErrorDeg = orientationErrorDeg;
            record.maximumCableResidualMm = stepMaximumCableResidualMm;
            record.newmarkIterations = dynamicsStep.iterations;
            record.newmarkResidual = dynamicsStep.residual;
            record.poseBoundsViolation = poseOutsideBounds;
            record.roundTripToleranceViolation =
                    translationError > translationToleranceMm ||
                    orientationErrorDeg > orientationToleranceDeg ||
                    stepMaximumCableResidualMm > cableResidualToleranceMm;
            record.calculationDurationUs = stepTimer.nsecsElapsed() / 1000;
            record.fullCycleDurationUs = record.calculationDurationUs;
            recorder.tryAppend(record);
            ++result.forwardKinematicsChecks;
        }
        result.finalState = dynamicsStep.state;
        result.completedSteps = stepIndex;
    }

    finishRecording();

    result.valid = result.completedSteps == stepCount &&
            result.forwardKinematicsChecks == stepCount &&
            result.firstPoseBoundsViolationStep == 0 &&
            result.maximumTranslationRoundTripErrorMm <= translationToleranceMm &&
            result.maximumOrientationRoundTripErrorDeg <= orientationToleranceDeg &&
            result.maximumCableResidualMm <= cableResidualToleranceMm &&
            result.recordingError.isEmpty() &&
            result.droppedRecords == 0 &&
            result.writtenRecords == static_cast<quint64>(result.completedSteps);
    result.summary = QStringLiteral(
                "阶段A软件验证%1\n"
                "模板：%2（阶段A仅允许G302）\n"
                "输入：%3；周期=%4 ms，时长=%5 s；仅平动=%6\n"
                "F/T安装：r_ES=[%7, %8, %9] m，R_ES=I；模拟量先按传感器原始力旋量转换到质心\n"
                "初始位姿(SI)=[%10, %11, %12 m, %13, %14, %15 rad]\n"
                "刚体质量=%16 kg\n"
                "数学步=%17，逐步运动学往返校验=%18（每步一次）\n"
                "Newmark最大迭代/残差=%19/%20\n"
                "运动学往返最大误差：平移=%21 mm（第%22步），姿态=%23 deg（第%24步），绳长残差=%25 mm（第%26步）\n"
                "首次越出G302正运动学边界：%27\n"
                "首次往返误差超限：%28\n"
                "最大相对电机角=%29 rad\n"
                "末态位姿(SI)=[%30, %31, %32 m, %33, %34, %35 rad]\n"
                "全程未调用任何雷赛运动API。")
            .arg(result.valid ? QStringLiteral("通过") : QStringLiteral("未通过"))
            .arg(configuration.machineTemplateName)
            .arg(wrenchSource.summary())
            .arg(configuration.controlPeriodS * 1000.0, 0, 'f', 3)
            .arg(configuration.durationS, 0, 'f', 3)
            .arg(configuration.translationOnly ? QStringLiteral("是（质心处三维力矩清零）") :
                                                 QStringLiteral("否"))
            .arg(configuration.sensorTransform.sensorOriginInPlatformM[0], 0, 'g', 8)
            .arg(configuration.sensorTransform.sensorOriginInPlatformM[1], 0, 'g', 8)
            .arg(configuration.sensorTransform.sensorOriginInPlatformM[2], 0, 'g', 8)
            .arg(configuration.initialState.pose[0], 0, 'g', 8)
            .arg(configuration.initialState.pose[1], 0, 'g', 8)
            .arg(configuration.initialState.pose[2], 0, 'g', 8)
            .arg(configuration.initialState.pose[3], 0, 'g', 8)
            .arg(configuration.initialState.pose[4], 0, 'g', 8)
            .arg(configuration.initialState.pose[5], 0, 'g', 8)
            .arg(configuration.rigidBody.massKg, 0, 'g', 8)
            .arg(result.completedSteps)
            .arg(result.forwardKinematicsChecks)
            .arg(result.maximumNewmarkIterations)
            .arg(result.maximumNewmarkResidual, 0, 'g', 8)
            .arg(result.maximumTranslationRoundTripErrorMm, 0, 'g', 8)
            .arg(result.maximumTranslationErrorStep)
            .arg(result.maximumOrientationRoundTripErrorDeg, 0, 'g', 8)
            .arg(result.maximumOrientationErrorStep)
            .arg(result.maximumCableResidualMm, 0, 'g', 8)
            .arg(result.maximumCableResidualStep)
            .arg(result.firstPoseBoundsViolationStep > 0 ?
                     QStringLiteral("第%1步，t=%2 ms，位姿=[%3, %4, %5 m, %6, %7, %8 rad]")
                         .arg(result.firstPoseBoundsViolationStep)
                         .arg(result.firstPoseBoundsViolationStep * configuration.controlPeriodS * 1000.0, 0, 'f', 3)
                         .arg(result.firstPoseBoundsViolationState.pose[0], 0, 'g', 8)
                         .arg(result.firstPoseBoundsViolationState.pose[1], 0, 'g', 8)
                         .arg(result.firstPoseBoundsViolationState.pose[2], 0, 'g', 8)
                         .arg(result.firstPoseBoundsViolationState.pose[3], 0, 'g', 8)
                         .arg(result.firstPoseBoundsViolationState.pose[4], 0, 'g', 8)
                         .arg(result.firstPoseBoundsViolationState.pose[5], 0, 'g', 8) :
                     QStringLiteral("无"))
            .arg(result.firstRoundTripToleranceViolationStep > 0 ?
                     QStringLiteral("第%1步，t=%2 ms，位姿=[%3, %4, %5 m, %6, %7, %8 rad]")
                         .arg(result.firstRoundTripToleranceViolationStep)
                         .arg(result.firstRoundTripToleranceViolationStep * configuration.controlPeriodS * 1000.0, 0, 'f', 3)
                         .arg(result.firstRoundTripToleranceViolationState.pose[0], 0, 'g', 8)
                         .arg(result.firstRoundTripToleranceViolationState.pose[1], 0, 'g', 8)
                         .arg(result.firstRoundTripToleranceViolationState.pose[2], 0, 'g', 8)
                         .arg(result.firstRoundTripToleranceViolationState.pose[3], 0, 'g', 8)
                         .arg(result.firstRoundTripToleranceViolationState.pose[4], 0, 'g', 8)
                         .arg(result.firstRoundTripToleranceViolationState.pose[5], 0, 'g', 8) :
                     QStringLiteral("无"))
            .arg(result.maximumRelativeMotorAngleRad, 0, 'g', 8)
            .arg(result.finalState.pose[0], 0, 'g', 8)
            .arg(result.finalState.pose[1], 0, 'g', 8)
            .arg(result.finalState.pose[2], 0, 'g', 8)
            .arg(result.finalState.pose[3], 0, 'g', 8)
            .arg(result.finalState.pose[4], 0, 'g', 8)
            .arg(result.finalState.pose[5], 0, 'g', 8);
    result.summary += QStringLiteral(
                "\n冻结模拟输入幅值/常值=[%1, %2, %3 N, %4, %5, %6 N·m]"
                "\n冻结惯量矩阵=[%7, %8, %9; %10, %11, %12; %13, %14, %15] kg·m²"
                "\n通过阈值：平移≤%16 mm，姿态≤%17 deg，绳长残差≤%18 mm")
            .arg(configuration.wrenchProfile.amplitude[0], 0, 'g', 8)
            .arg(configuration.wrenchProfile.amplitude[1], 0, 'g', 8)
            .arg(configuration.wrenchProfile.amplitude[2], 0, 'g', 8)
            .arg(configuration.wrenchProfile.amplitude[3], 0, 'g', 8)
            .arg(configuration.wrenchProfile.amplitude[4], 0, 'g', 8)
            .arg(configuration.wrenchProfile.amplitude[5], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[0], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[1], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[2], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[3], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[4], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[5], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[6], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[7], 0, 'g', 8)
            .arg(configuration.rigidBody.inertiaKgM2[8], 0, 'g', 8)
            .arg(translationToleranceMm, 0, 'g', 8)
            .arg(orientationToleranceDeg, 0, 'g', 8)
            .arg(cableResidualToleranceMm, 0, 'g', 8);
    if(configuration.wrenchProfile.mode == SimulatedWrenchMode::Pulse){
        result.summary += QStringLiteral("\n脉冲参数：起点=%1 s，持续=%2 s")
                .arg(configuration.wrenchProfile.pulseStartS, 0, 'g', 8)
                .arg(configuration.wrenchProfile.pulseDurationS, 0, 'g', 8);
    }
    else if(configuration.wrenchProfile.mode == SimulatedWrenchMode::Sine){
        result.summary += QStringLiteral("\n正弦参数：频率=%1 Hz，相位=%2 rad")
                .arg(configuration.wrenchProfile.sineFrequencyHz, 0, 'g', 8)
                .arg(configuration.wrenchProfile.sinePhaseRad, 0, 'g', 8);
    }
    else if(configuration.wrenchProfile.mode == SimulatedWrenchMode::Formula){
        result.summary += QStringLiteral(
                    "\n六分量公式=[%1; %2; %3; %4; %5; %6]")
                .arg(configuration.wrenchProfile.expressions[0])
                .arg(configuration.wrenchProfile.expressions[1])
                .arg(configuration.wrenchProfile.expressions[2])
                .arg(configuration.wrenchProfile.expressions[3])
                .arg(configuration.wrenchProfile.expressions[4])
                .arg(configuration.wrenchProfile.expressions[5]);
    }
    if(configuration.poseLowerBoundsMmRad.size() >= 6 &&
            configuration.poseUpperBoundsMmRad.size() >= 6){
        result.summary += QStringLiteral(
                    "\nLite正运动学边界：下界=[%1, %2, %3 mm, %4, %5, %6 rad]，"
                    "上界=[%7, %8, %9 mm, %10, %11, %12 rad]")
                .arg(configuration.poseLowerBoundsMmRad[0], 0, 'g', 8)
                .arg(configuration.poseLowerBoundsMmRad[1], 0, 'g', 8)
                .arg(configuration.poseLowerBoundsMmRad[2], 0, 'g', 8)
                .arg(configuration.poseLowerBoundsMmRad[3], 0, 'g', 8)
                .arg(configuration.poseLowerBoundsMmRad[4], 0, 'g', 8)
                .arg(configuration.poseLowerBoundsMmRad[5], 0, 'g', 8)
                .arg(configuration.poseUpperBoundsMmRad[0], 0, 'g', 8)
                .arg(configuration.poseUpperBoundsMmRad[1], 0, 'g', 8)
                .arg(configuration.poseUpperBoundsMmRad[2], 0, 'g', 8)
                .arg(configuration.poseUpperBoundsMmRad[3], 0, 'g', 8)
                .arg(configuration.poseUpperBoundsMmRad[4], 0, 'g', 8)
                .arg(configuration.poseUpperBoundsMmRad[5], 0, 'g', 8);
    }
    appendRecordingSummary();
    return result;
}

ForceInteractionValidationWorker::ForceInteractionValidationWorker(
        const ForceInteractionValidationConfig& configuration,
        QObject* parent)
    : QThread(parent), configuration_(configuration)
{
    setObjectName(QStringLiteral("ForceInteractionValidationWorker"));
}

void ForceInteractionValidationWorker::requestCancellation()
{
    cancellationRequested_.store(true);
}

void ForceInteractionValidationWorker::run()
{
    const ForceInteractionValidationResult result =
            ForceInteractionSoftwareValidator::run(configuration_,
                                                   &cancellationRequested_);
    emit validationCompleted(result.valid, result.cancelled, result.summary);
}
