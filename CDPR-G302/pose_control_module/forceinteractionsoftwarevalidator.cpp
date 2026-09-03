#include "forceinteractionsoftwarevalidator.h"

#include "forwardkinematicssolver.h"
#include "wrenchtransformer.h"

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
    ForceSensorTransformConfig transform;
    transform.configured = true;
    transform.rotationSensorToPlatform =
            kMeasuredForceSensorToPlatformRotation;
    // 使用实测安装关系 R_ES=I 和实测力臂，联合验证坐标旋转及 r x F 项。
    transform.sensorOriginInPlatformM = kMeasuredForceSensorOriginInPlatformM;
    WrenchTransformer transformer(transform);
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
                     kMeasuredForceSensorOriginInPlatformM[2]) > 1.0e-12){
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
            configuration.initialPoseMmRad.size() != 1){
        result.summary = failureSummary(QStringLiteral("输入校验"),
                                        QStringLiteral("周期、时长或单末端初始位姿无效"));
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

    for(int stepIndex = 1; stepIndex <= stepCount; ++stepIndex){
        if(cancellationRequested && cancellationRequested->load()){
            result.cancelled = true;
            result.summary = QStringLiteral("阶段A软件验证已取消；完成%1/%2个数学步，未调用任何雷赛运动API。")
                    .arg(result.completedSteps).arg(stepCount);
            return result;
        }
        const double elapsedS = std::min(configuration.durationS,
                                         stepIndex * configuration.controlPeriodS);
        ForceInteractionFrameStamp stamp;
        stamp.hostMonotonicTimeUs = static_cast<qint64>(
                    std::llround(elapsedS * 1.0e6));
        stamp.valid = true;
        const ForceInteractionWrenchSample wrench =
                wrenchSource.sample(stamp, elapsedS);
        if(!wrench.valid){
            result.summary = failureSummary(QStringLiteral("模拟力求值"),
                                            QStringLiteral("产生无效力旋量"));
            return result;
        }
        const CdprDynamicsStepResult dynamicsStep =
                dynamics.step(wrench, configuration.controlPeriodS);
        if(!dynamicsStep.valid){
            result.summary = failureSummary(QStringLiteral("Newmark单步"),
                                            dynamicsStep.errorMessage);
            return result;
        }
        result.maximumNewmarkIterations = std::max(
                    result.maximumNewmarkIterations, dynamicsStep.iterations);
        result.maximumNewmarkResidual = std::max(
                    result.maximumNewmarkResidual, dynamicsStep.residual);

        const auto poseMmRad = toPoseMmRad(dynamicsStep.state);
        const CompensatedCableKinematics::Evaluation evaluation =
                kinematics.evaluatePose(poseMmRad, kinematicsState);
        if(!evaluation.valid ||
                evaluation.relativeMotorThetaRad.size() != kForceInteractionCableCount){
            result.summary = failureSummary(
                        QStringLiteral("补偿逆运动学"),
                        evaluation.errorMessage.isEmpty() ?
                            QStringLiteral("未返回完整8轴结果") : evaluation.errorMessage);
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
            result.maximumTranslationRoundTripErrorMm = std::max(
                        result.maximumTranslationRoundTripErrorMm,
                        translationError);
            result.maximumOrientationRoundTripErrorDeg = std::max(
                        result.maximumOrientationRoundTripErrorDeg,
                        orientationErrorRad * 180.0 / kPi);

            const std::vector<double> reconstructedLengths =
                    kinematics.cableLengthsForPose({forward.pose}, &error);
            if(reconstructedLengths.size() != evaluation.cableLengthMm.size()){
                result.summary = failureSummary(QStringLiteral("正运动学残差"), error);
                return result;
            }
            for(size_t cable = 0; cable < reconstructedLengths.size(); ++cable){
                result.maximumCableResidualMm = std::max(
                            result.maximumCableResidualMm,
                            std::abs(reconstructedLengths[cable] -
                                     evaluation.cableLengthMm[cable]));
            }
            ++result.forwardKinematicsChecks;
        }
        result.finalState = dynamicsStep.state;
        result.completedSteps = stepIndex;
    }

    constexpr double translationToleranceMm = 0.1;
    constexpr double orientationToleranceDeg = 0.01;
    constexpr double cableResidualToleranceMm = 0.1;
    result.valid = result.completedSteps == stepCount &&
            result.forwardKinematicsChecks == stepCount &&
            result.maximumTranslationRoundTripErrorMm <= translationToleranceMm &&
            result.maximumOrientationRoundTripErrorDeg <= orientationToleranceDeg &&
            result.maximumCableResidualMm <= cableResidualToleranceMm;
    result.summary = QStringLiteral(
                "阶段A软件验证%1\n"
                "输入：%2；周期=%3 ms，时长=%4 s\n"
                "数学步=%5，逐步运动学往返校验=%6（每步一次）\n"
                "Newmark最大迭代/残差=%7/%8\n"
                "运动学往返最大误差：平移=%9 mm，姿态=%10 deg，绳长残差=%11 mm\n"
                "最大相对电机角=%12 rad\n"
                "末态位姿(SI)=[%13, %14, %15 m, %16, %17, %18 rad]\n"
                "全程未调用任何雷赛运动API。")
            .arg(result.valid ? QStringLiteral("通过") : QStringLiteral("未通过"))
            .arg(wrenchSource.summary())
            .arg(configuration.controlPeriodS * 1000.0, 0, 'f', 3)
            .arg(configuration.durationS, 0, 'f', 3)
            .arg(result.completedSteps)
            .arg(result.forwardKinematicsChecks)
            .arg(result.maximumNewmarkIterations)
            .arg(result.maximumNewmarkResidual, 0, 'g', 8)
            .arg(result.maximumTranslationRoundTripErrorMm, 0, 'g', 8)
            .arg(result.maximumOrientationRoundTripErrorDeg, 0, 'g', 8)
            .arg(result.maximumCableResidualMm, 0, 'g', 8)
            .arg(result.maximumRelativeMotorAngleRad, 0, 'g', 8)
            .arg(result.finalState.pose[0], 0, 'g', 8)
            .arg(result.finalState.pose[1], 0, 'g', 8)
            .arg(result.finalState.pose[2], 0, 'g', 8)
            .arg(result.finalState.pose[3], 0, 'g', 8)
            .arg(result.finalState.pose[4], 0, 'g', 8)
            .arg(result.finalState.pose[5], 0, 'g', 8);
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
