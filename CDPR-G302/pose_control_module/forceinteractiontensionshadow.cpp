#include "forceinteractiontensionshadow.h"

#include "MatrixFun.h"
#include "winchcompensation.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace {

using Controller = TaskSpaceTorqueController;
using Allocator = RedundantTorqueAllocator;

Controller::Vector6d vector6(const ForceInteractionVector6& value)
{
    Controller::Vector6d result;
    for(int i = 0; i < 6; ++i) result[i] = value[static_cast<size_t>(i)];
    return result;
}

Controller::Matrix3d rotationMatrix(const Controller::Vector6d& pose)
{
    return (Eigen::AngleAxisd(pose[5], Controller::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pose[4], Controller::Vector3d::UnitY()) *
            Eigen::AngleAxisd(pose[3], Controller::Vector3d::UnitX()))
            .toRotationMatrix();
}

bool fail(QString* errorMessage, const QString& message)
{
    if(errorMessage) *errorMessage = message;
    return false;
}

} // namespace

bool ForceInteractionTensionShadowConfig::validate(QString* errorMessage) const
{
    std::string error;
    if(!std::isfinite(rigidBody.massKg) || rigidBody.massKg <= 0.0)
        return fail(errorMessage, QStringLiteral("平台质量无效"));
    if(kinematics.anchorCableCoordinate.size() < 8 ||
            kinematics.endCableContactPos.empty() ||
            kinematics.endCableContactPos.front().size() < 8)
        return fail(errorMessage, QStringLiteral("八绳几何参数不完整"));
    if(!TaskSpaceTorqueController::validateConfig(controller, &error))
        return fail(errorMessage, QStringLiteral("任务空间参数无效：%1")
                    .arg(QString::fromStdString(error)));
    if(!RedundantTorqueAllocator::validateConfig(allocator, &error))
        return fail(errorMessage, QStringLiteral("张力分配参数无效：%1")
                    .arg(QString::fromStdString(error)));
    if(!effectiveRadiusMPerRad.allFinite() ||
            (effectiveRadiusMPerRad.array() <= 0.0).any())
        return fail(errorMessage, QStringLiteral("八轴有效卷绕半径无效"));
    if(!std::isfinite(gravityMPerSec2) || gravityMPerSec2 <= 0.0 ||
            !std::isfinite(outerPeriodS) || outerPeriodS <= 0.0)
        return fail(errorMessage, QStringLiteral("重力或外环周期无效"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionShadow::configure(
        const ForceInteractionTensionShadowConfig& config,
        QString* errorMessage)
{
    if(!config.validate(errorMessage)){
        configured_ = false;
        return false;
    }
    config_ = config;
    controller_.reset();
    configured_ = true;
    if(errorMessage) errorMessage->clear();
    return true;
}

void ForceInteractionTensionShadow::reset()
{
    controller_.reset();
}

bool ForceInteractionTensionShadow::buildJacobian(
        const Controller::Vector6d& pose,
        Controller::Matrix8x6d& jacobian,
        QString* errorMessage) const
{
    const auto& contacts = config_.kinematics.endCableContactPos.front();
    const Controller::Matrix3d rotation = rotationMatrix(pose);
    const Controller::Vector3d translation = pose.head<3>();
    for(int cable = 0; cable < 8; ++cable){
        const auto& contactMm = contacts[static_cast<size_t>(cable)];
        const auto& anchorMm = config_.kinematics.anchorCableCoordinate[
                static_cast<size_t>(cable)];
        if(contactMm.size() < 3 || anchorMm.size() < 3)
            return fail(errorMessage, QStringLiteral("绳索%1坐标维数不足").arg(cable + 1));
        const Controller::Vector3d contactLocal(
                    contactMm[0] * 1.0e-3,
                    contactMm[1] * 1.0e-3,
                    contactMm[2] * 1.0e-3);
        const Controller::Vector3d anchor(
                    anchorMm[0] * 1.0e-3,
                    anchorMm[1] * 1.0e-3,
                    anchorMm[2] * 1.0e-3);
        const Controller::Vector3d momentArm = rotation * contactLocal;
        const Controller::Vector3d contactWorld = translation + momentArm;
        const auto cableLength = MatrixFun::cableLengthCalculate(
                    contactWorld, anchor,
                    config_.kinematics.pulleyRadiusMm * 1.0e-3);
        if(!cableLength.valid)
            return fail(errorMessage, QStringLiteral("绳索%1滑轮切点计算失败").arg(cable + 1));
        const Controller::Vector3d cableVector =
                cableLength.tangentPoint - contactWorld;
        const double length = cableVector.norm();
        if(!std::isfinite(length) || length <= 1.0e-9)
            return fail(errorMessage, QStringLiteral("绳索%1方向奇异").arg(cable + 1));
        const Controller::Vector3d direction = cableVector / length;
        jacobian.block<1, 3>(cable, 0) = -direction.transpose();
        jacobian.block<1, 3>(cable, 3) =
                -momentArm.cross(direction).transpose();
    }
    if(!jacobian.allFinite())
        return fail(errorMessage, QStringLiteral("八绳雅可比含非数值"));
    if(errorMessage) errorMessage->clear();
    return true;
}

ForceInteractionTensionShadowResult ForceInteractionTensionShadow::evaluate(
        const ForceInteractionTensionShadowInput& input)
{
    ForceInteractionTensionShadowResult result;
    if(!configured_){
        result.errorMessage = QStringLiteral("影子计算器尚未配置");
        return result;
    }
    if(!input.desired.poseValid || !input.desired.twistValid ||
            !input.desired.accelerationValid || !input.observed.poseValid ||
            !input.observed.twistValid || !std::isfinite(input.dtS)){
        result.errorMessage = QStringLiteral("期望/观测状态或周期无效");
        return result;
    }

    Controller::Reference reference;
    reference.pose = vector6(input.desired.pose);
    reference.velocity = vector6(input.desired.twist);
    reference.acceleration = vector6(input.desired.acceleration);
    Controller::Feedback feedback;
    feedback.pose = vector6(input.observed.pose);
    feedback.velocity = vector6(input.observed.twist);
    if(config_.translationOnly){
        reference.pose.tail<3>() = feedback.pose.tail<3>();
        reference.velocity.tail<3>().setZero();
        reference.acceleration.tail<3>().setZero();
        feedback.velocity.tail<3>().setZero();
    }

    Controller::Matrix8x6d jacobian;
    if(!buildJacobian(feedback.pose, jacobian, &result.errorMessage))
        return result;

    Controller::EffectiveDynamics dynamics;
    dynamics.mass.setZero();
    dynamics.mass.topLeftCorner<3, 3>() =
            config_.rigidBody.massKg * Controller::Matrix3d::Identity();
    Controller::Matrix3d bodyInertia;
    for(int row = 0; row < 3; ++row)
        for(int column = 0; column < 3; ++column)
            bodyInertia(row, column) = config_.rigidBody.inertiaKgM2[
                    static_cast<size_t>(row * 3 + column)];
    const Controller::Matrix3d rotation = rotationMatrix(feedback.pose);
    dynamics.mass.bottomRightCorner<3, 3>() =
            rotation * bodyInertia * rotation.transpose();
    dynamics.generalizedForce.setZero();
    dynamics.generalizedForce[2] =
            config_.rigidBody.massKg * config_.gravityMPerSec2;

    result.control = controller_.update(reference, feedback, dynamics,
                                        input.dtS, config_.controller,
                                        &result.timing);
    if(!result.control.valid){
        result.errorMessage = QStringLiteral("任务空间控制失败：%1")
                .arg(QString::fromStdString(result.control.message));
        return result;
    }

    Allocator::Request allocationRequest;
    allocationRequest.generalizedControl = result.control.generalizedControl;
    allocationRequest.cableJacobian = jacobian;
    allocationRequest.effectiveRadius = config_.effectiveRadiusMPerRad;
    allocationRequest.dtSec = input.dtS;
    allocationRequest.tensionReferenceOnly = true;
    allocationRequest.tensionReferenceSelection =
            Allocator::TensionReferenceSelection::AreaCentroid;
    result.allocation = Allocator().solve(allocationRequest,
                                          config_.allocator,
                                          &result.timing);
    if(!result.allocation.valid){
        controller_.discardPending();
        result.errorMessage = QStringLiteral("冗余张力分配失败：%1")
                .arg(QString::fromStdString(result.allocation.message));
        return result;
    }
    controller_.commit(true, result.allocation.constrainedSolution ||
                       result.allocation.wrenchLimited);
    result.valid = true;
    return result;
}

ForceInteractionTensionShadowConfig makeDefaultG302TranslationShadowConfig(
        const ForceInteractionRigidBodyConfig& rigidBody,
        const CompensatedCableKinematics::Configuration& kinematics,
        double tensionMinimumN,
        double tensionMaximumN,
        double torqueLimitNm)
{
    ForceInteractionTensionShadowConfig config;
    config.rigidBody = rigidBody;
    config.kinematics = kinematics;
    config.translationOnly = true;
    config.outerPeriodS = 0.025;
    config.controller.virtualMass.setConstant(rigidBody.massKg);
    for(int row = 0; row < 3; ++row)
        for(int column = 0; column < 3; ++column)
            config.controller.virtualBodyInertia(row, column) =
                    rigidBody.inertiaKgM2[static_cast<size_t>(row * 3 + column)];
    config.controller.stiffness << 1600.0, 1600.0, 3000.0,
            100.0, 100.0, 25.0;
    config.controller.dampingRatio = 0.95;
    config.controller.integralEnabled = false;
    config.controller.minimumMassEigenvalue = 1.0e-12;

    config.allocator.tensionMinimum.setConstant(tensionMinimumN);
    config.allocator.tensionMaximum.setConstant(tensionMaximumN);
    config.allocator.tensionBias.setConstant(
                0.5 * (tensionMinimumN + tensionMaximumN));
    config.allocator.tensionWeight.setOnes();
    config.allocator.hardwareTorqueMinimum.setConstant(-torqueLimitNm);
    config.allocator.hardwareTorqueMaximum.setZero();
    config.allocator.hardwareDirection.setOnes();
    config.allocator.torqueSlewEnabled = false;

    for(int axis = 0; axis < 8; ++axis){
        double radiusM = 0.0;
        if(axis < static_cast<int>(kinematics.winchConfig.size()) &&
                WinchCompensation::isEnabled(kinematics.winchConfig[axis])){
            radiusM = WinchCompensation::helicalLengthPerRad(
                        kinematics.winchConfig[axis]) * 1.0e-3;
        }
        if((!std::isfinite(radiusM) || radiusM <= 0.0) &&
                axis < static_cast<int>(kinematics.cableMotorScaleRadPerMm.size())){
            const double scale = std::abs(kinematics.cableMotorScaleRadPerMm[axis]);
            if(std::isfinite(scale) && scale > 0.0)
                radiusM = 1.0e-3 / scale;
        }
        config.effectiveRadiusMPerRad[axis] = radiusM;
    }
    return config;
}
