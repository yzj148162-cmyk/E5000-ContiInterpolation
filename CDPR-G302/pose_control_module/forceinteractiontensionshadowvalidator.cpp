#include "forceinteractiontensionshadowvalidator.h"

#include "eightcabletensionfeedback.h"
#include "redundanttorqueallocator.h"
#include "taskspacetorquecontrol.h"

#include <cmath>
#include <limits>

namespace {

using Controller = TaskSpaceTorqueController;
using Allocator = RedundantTorqueAllocator;

bool fail(QString* errorMessage, const QString& message)
{
    if(errorMessage) *errorMessage = message;
    return false;
}

Allocator::Matrix8x6d fullRankSelfTestJacobian()
{
    // A=-J^T.  The eight columns below have rank six and sum to zero, so a
    // strictly-positive equal-tension solution exists for zero wrench.
    Allocator::Matrix6x8d allocation = Allocator::Matrix6x8d::Zero();
    for(int axis = 0; axis < 6; ++axis) allocation(axis, axis) = 1.0;
    allocation(0, 6) = allocation(1, 6) = allocation(2, 6) = -1.0;
    allocation(3, 7) = allocation(4, 7) = allocation(5, 7) = -1.0;
    return -allocation.transpose();
}

Allocator::Config allocatorConfig()
{
    Allocator::Config config;
    config.tensionMinimum.setConstant(1.0);
    config.tensionMaximum.setConstant(100.0);
    config.tensionBias.setConstant(20.0);
    config.tensionWeight.setOnes();
    config.hardwareTorqueMinimum.setConstant(-3.0);
    config.hardwareTorqueMaximum.setZero();
    config.hardwareDirection.setOnes();
    config.torqueSlewEnabled = true;
    config.hardwareTorqueSlewRate.setConstant(20.0);
    return config;
}

} // namespace

bool ForceInteractionTensionShadowValidator::runSelfChecks(QString* errorMessage)
{
    Controller controller;
    Controller::Config controllerConfig;
    controllerConfig.virtualMass.setConstant(2.0);
    controllerConfig.virtualBodyInertia.setIdentity();
    controllerConfig.stiffness << 100.0, 100.0, 100.0, 10.0, 10.0, 10.0;
    controllerConfig.dampingRatio = 0.95;

    Controller::Reference reference;
    Controller::Feedback feedback;
    Controller::EffectiveDynamics dynamics;
    dynamics.mass.setIdentity();
    dynamics.mass.topLeftCorner<3, 3>() *= 2.0;
    dynamics.generalizedForce[2] = 20.0;

    const Controller::Step equilibrium = controller.update(
                reference, feedback, dynamics, 0.025, controllerConfig);
    if(!equilibrium.valid ||
            (equilibrium.generalizedControl - dynamics.generalizedForce)
            .cwiseAbs().maxCoeff() > 1.0e-12){
        return fail(errorMessage, QStringLiteral("M0任务空间静态重力平衡自检失败"));
    }
    controller.commit(true, false);

    reference.pose[0] = 0.01;
    const Controller::Step translated = controller.update(
                reference, feedback, dynamics, 0.025, controllerConfig);
    if(!translated.valid || translated.generalizedControl[0] <= 0.0 ||
            translated.generalizedControl.tail<3>().cwiseAbs().maxCoeff() > 1.0e-12){
        return fail(errorMessage, QStringLiteral("M0仅平动方向/姿态冻结自检失败"));
    }
    controller.discardPending();

    reference.pose[0] = std::numeric_limits<double>::quiet_NaN();
    const Controller::Step nonFinite = controller.update(
                reference, feedback, dynamics, 0.025, controllerConfig);
    if(nonFinite.valid || nonFinite.status != Controller::StatusCode::InvalidReference){
        return fail(errorMessage, QStringLiteral("M0非数值输入拒绝自检失败"));
    }
    reference.pose.setZero();

    const Allocator::Config allocationConfig = allocatorConfig();
    Allocator::Request allocationRequest;
    allocationRequest.cableJacobian = fullRankSelfTestJacobian();
    allocationRequest.effectiveRadius.setConstant(0.02);
    allocationRequest.dtSec = 0.025;
    allocationRequest.tensionReferenceOnly = true;
    allocationRequest.tensionReferenceSelection =
            Allocator::TensionReferenceSelection::AreaCentroid;
    allocationRequest.generalizedControl = dynamics.generalizedForce;
    const Allocator::Result allocation = Allocator().solve(
                allocationRequest, allocationConfig);
    if(!allocation.valid || allocation.allocationRank != 6 ||
            allocation.maximumGeneralizedControlResidual > 1.0e-8 ||
            allocation.minimumTensionMargin < -1.0e-9){
        return fail(errorMessage,
                    QStringLiteral("M0冗余张力可行分配自检失败：%1")
                    .arg(QString::fromStdString(allocation.message)));
    }

    allocationRequest.generalizedControl[0] = 1.0e6;
    const Allocator::Result infeasible = Allocator().solve(
                allocationRequest, allocationConfig);
    if(infeasible.valid || infeasible.status != Allocator::StatusCode::Infeasible){
        return fail(errorMessage, QStringLiteral("M0不可行张力拒绝自检失败"));
    }

    EightCableTensionFeedback::Config feedbackConfig;
    feedbackConfig.enabled = true;
    feedbackConfig.kp.setConstant(0.01);
    feedbackConfig.ki.setZero();
    feedbackConfig.kd.setZero();
    feedbackConfig.deadbandRatio = 0.0;
    feedbackConfig.torqueLimitNm = 3.0;
    feedbackConfig.slewNmPerSec = 6.0;
    feedbackConfig.periodUs = 5000;
    feedbackConfig.parameterSource = "M0 deterministic self-check";
    EightCableTensionFeedback tensionFeedback;
    const auto entryTension = Allocator::Vector8d::Constant(20.0);
    const auto entryTorque = Allocator::Vector8d::Constant(-0.4);
    tensionFeedback.initialize(entryTorque, entryTension, 1000000);
    EightCableTensionFeedback::Diagnostic diagnostic;
    std::string feedbackError;
    const auto higherTarget = Allocator::Vector8d::Constant(21.0);
    if(!tensionFeedback.propose(feedbackConfig, allocationConfig,
                                Allocator::Vector8d::Constant(0.02),
                                higherTarget, entryTension, entryTorque,
                                1005000, 0.0345, diagnostic, feedbackError) ||
            !diagnostic.valid || diagnostic.commandNm.maxCoeff() >= -0.4){
        return fail(errorMessage,
                    QStringLiteral("M0张力反馈方向/斜率自检失败：%1")
                    .arg(QString::fromStdString(feedbackError)));
    }

    if(errorMessage) errorMessage->clear();
    return true;
}
