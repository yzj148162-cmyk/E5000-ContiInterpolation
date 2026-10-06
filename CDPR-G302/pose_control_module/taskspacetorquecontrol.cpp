#include "taskspacetorquecontrol.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace {

using Vector6d = TaskSpaceTorqueController::Vector6d;
using Matrix6d = TaskSpaceTorqueController::Matrix6d;
using Vector3d = TaskSpaceTorqueController::Vector3d;
using Matrix3d = TaskSpaceTorqueController::Matrix3d;

template<typename Derived>
bool isNonNegativeFinite(const Eigen::MatrixBase<Derived>& value)
{
    return value.allFinite() && (value.array() >= 0.0).all();
}

template<typename Derived>
bool isPositiveFinite(const Eigen::MatrixBase<Derived>& value)
{
    return value.allFinite() && (value.array() > 0.0).all();
}

void setError(std::string* errorMessage, const char* message)
{
    if(errorMessage){
        *errorMessage = message;
    }
}

TaskSpaceTorqueController::Step invalidStep(
        TaskSpaceTorqueController::StatusCode status,
        const char* message,
        double dtSec)
{
    TaskSpaceTorqueController::Step step;
    step.status = status;
    step.message = message;
    step.dtSec = dtSec;
    return step;
}

Matrix3d rotationMatrix(const Vector6d& pose)
{
    return (Eigen::AngleAxisd(pose[5], Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pose[4], Vector3d::UnitY()) *
            Eigen::AngleAxisd(pose[3], Vector3d::UnitX())).toRotationMatrix();
}

Matrix3d globalEulerRateMatrix(const Vector6d& pose)
{
    const double pitch = pose[4];
    const double yaw = pose[5];
    Matrix3d result;
    result.col(0) << std::cos(yaw) * std::cos(pitch),
            std::sin(yaw) * std::cos(pitch),
            -std::sin(pitch);
    result.col(1) << -std::sin(yaw), std::cos(yaw), 0.0;
    result.col(2) << 0.0, 0.0, 1.0;
    return result;
}

Matrix3d globalEulerRateMatrixDerivative(const Vector6d& pose,
                                          const Vector3d& eulerRate)
{
    const double pitch = pose[4];
    const double yaw = pose[5];
    const double pitchRate = eulerRate[1];
    const double yawRate = eulerRate[2];
    Matrix3d result = Matrix3d::Zero();
    result.col(0) <<
            -std::sin(yaw) * yawRate * std::cos(pitch) -
                std::cos(yaw) * std::sin(pitch) * pitchRate,
            std::cos(yaw) * yawRate * std::cos(pitch) -
                std::sin(yaw) * std::sin(pitch) * pitchRate,
            -std::cos(pitch) * pitchRate;
    result.col(1) << -std::cos(yaw) * yawRate,
            -std::sin(yaw) * yawRate,
            0.0;
    return result;
}

void convertReferenceKinematics(
        const TaskSpaceTorqueController::Reference& reference,
        Vector6d& twist,
        Vector6d& acceleration)
{
    twist.head<3>() = reference.velocity.head<3>();
    acceleration.head<3>() = reference.acceleration.head<3>();
    const Vector3d eulerRate = reference.velocity.tail<3>();
    const Matrix3d rateMap = globalEulerRateMatrix(reference.pose);
    twist.tail<3>() = rateMap * eulerRate;
    acceleration.tail<3>() =
            rateMap * reference.acceleration.tail<3>() +
            globalEulerRateMatrixDerivative(reference.pose, eulerRate) *
            eulerRate;
}

bool symmetricSquareRoots(const Matrix6d& source,
                          Matrix6d& squareRoot,
                          Matrix6d& inverseSquareRoot)
{
    const Matrix6d symmetric = 0.5 * (source + source.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric);
    if(solver.info() != Eigen::Success ||
            !solver.eigenvalues().allFinite() ||
            solver.eigenvalues().minCoeff() <= 0.0){
        return false;
    }
    const Eigen::Array<double, 6, 1> eigenvalues =
            solver.eigenvalues().array();
    const Matrix6d eigenvectors = solver.eigenvectors();
    squareRoot = eigenvectors * eigenvalues.sqrt().matrix().asDiagonal() *
            eigenvectors.transpose();
    inverseSquareRoot = eigenvectors *
            eigenvalues.sqrt().inverse().matrix().asDiagonal() *
            eigenvectors.transpose();
    squareRoot = 0.5 * (squareRoot + squareRoot.transpose());
    inverseSquareRoot =
            0.5 * (inverseSquareRoot + inverseSquareRoot.transpose());
    return squareRoot.allFinite() && inverseSquareRoot.allFinite();
}

bool modalDampingMatrix(const Matrix6d& virtualTaskInertia,
                        const Vector6d& stiffness,
                        double dampingRatio,
                        Matrix6d& dampingMatrix)
{
    Matrix6d virtualInertiaHalf;
    Matrix6d virtualInertiaInverseHalf;
    if(!symmetricSquareRoots(virtualTaskInertia,
                             virtualInertiaHalf,
                             virtualInertiaInverseHalf)){
        return false;
    }
    const Matrix6d stiffnessMatrix = stiffness.asDiagonal();
    const Matrix6d normalizedStiffness =
            virtualInertiaInverseHalf * stiffnessMatrix *
            virtualInertiaInverseHalf;
    Matrix6d normalizedStiffnessHalf;
    Matrix6d unusedInverseHalf;
    if(!symmetricSquareRoots(normalizedStiffness,
                             normalizedStiffnessHalf,
                             unusedInverseHalf)){
        return false;
    }
    dampingMatrix = 2.0 * dampingRatio * virtualInertiaHalf *
            normalizedStiffnessHalf * virtualInertiaHalf;
    dampingMatrix = 0.5 * (dampingMatrix + dampingMatrix.transpose());
    return dampingMatrix.allFinite();
}

} // namespace

TaskSpaceTorqueController::DynamicsBuildResult
TaskSpaceTorqueController::buildEffectiveDynamics(
        const ModelTerms& model,
        const Vector6d& taskVelocity)
{
    DynamicsBuildResult result;
    if(!taskVelocity.allFinite()){
        result.message = "task velocity contains a non-finite value";
        return result;
    }
    if(!model.platformMass.allFinite() ||
            !model.platformCoriolis.allFinite() ||
            !model.gravity.allFinite() ||
            !model.cableJacobian.allFinite()){
        result.message = "platform or cable model contains a non-finite value";
        return result;
    }
    if(!std::isfinite(model.frictionVelocityScale) ||
            model.frictionVelocityScale <= 0.0){
        result.message =
                "frictionVelocityScale must be finite and positive";
        return result;
    }

    result.cableVelocity = model.cableJacobian * taskVelocity;
    result.smoothCableVelocitySign =
            (result.cableVelocity.array() /
             model.frictionVelocityScale).tanh().matrix();

    // Match cdpr_platform_inverse_dynamics.m: actuator inertia/friction do not
    // enter the rigid-platform wrench. The coordinator puts gravity and the
    // body-frame gyroscopic torque into model.gravity.
    result.dynamics.mass = model.platformMass;
    result.dynamics.coriolis = model.platformCoriolis;
    result.dynamics.generalizedForce = model.gravity;

    if(!result.dynamics.mass.allFinite() ||
            !result.dynamics.coriolis.allFinite() ||
            !result.dynamics.generalizedForce.allFinite()){
        result.message =
                "effective dynamics calculation produced a non-finite value";
        return result;
    }
    result.valid = true;
    return result;
}

bool TaskSpaceTorqueController::validateConfig(
        const Config& config,
        std::string* errorMessage)
{
    if(!isPositiveFinite(config.virtualMass)){
        setError(errorMessage,
                 "virtualMass must contain finite positive values");
        return false;
    }
    if(!config.virtualBodyInertia.allFinite()){
        setError(errorMessage, "virtualBodyInertia contains a non-finite value");
        return false;
    }
    const double virtualInertiaSymmetry =
            (config.virtualBodyInertia -
             config.virtualBodyInertia.transpose()).cwiseAbs().maxCoeff();
    Eigen::SelfAdjointEigenSolver<Matrix3d> virtualInertiaSolver(
                0.5 * (config.virtualBodyInertia +
                       config.virtualBodyInertia.transpose()),
                Eigen::EigenvaluesOnly);
    if(virtualInertiaSymmetry > config.massSymmetryTolerance ||
            virtualInertiaSolver.info() != Eigen::Success ||
            virtualInertiaSolver.eigenvalues().minCoeff() <
                config.minimumMassEigenvalue){
        setError(errorMessage,
                 "virtualBodyInertia must be symmetric positive definite");
        return false;
    }
    if(!std::isfinite(config.dampingRatio) || config.dampingRatio <= 0.0){
        setError(errorMessage,
                 "dampingRatio must be a finite positive scalar");
        return false;
    }
    if(!isPositiveFinite(config.stiffness)){
        setError(errorMessage, "stiffness must contain finite positive gains");
        return false;
    }
    if(!isNonNegativeFinite(config.integralGain)){
        setError(errorMessage,
                 "integralGain must contain finite non-negative gains");
        return false;
    }
    if(!isNonNegativeFinite(config.integralLimit)){
        setError(errorMessage, "integralLimit must contain finite non-negative limits");
        return false;
    }
    if(!std::isfinite(config.minDtSec) || config.minDtSec <= 0.0 ||
            !std::isfinite(config.maxDtSec) ||
            config.maxDtSec < config.minDtSec){
        setError(errorMessage, "time-step bounds are invalid");
        return false;
    }
    if(!std::isfinite(config.massSymmetryTolerance) ||
            config.massSymmetryTolerance < 0.0){
        setError(errorMessage,
                 "massSymmetryTolerance must be finite and non-negative");
        return false;
    }
    if(!std::isfinite(config.minimumMassEigenvalue) ||
            config.minimumMassEigenvalue <= 0.0){
        setError(errorMessage,
                 "minimumMassEigenvalue must be finite and positive");
        return false;
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

TaskSpaceTorqueController::Step TaskSpaceTorqueController::update(
        const Reference& reference,
        const Feedback& feedback,
        const EffectiveDynamics& dynamics,
        double dtSec,
        const Config& config,
        EightCablePreviewTiming* timing)
{
    EightCablePreviewSection validationTiming(timing, EightCablePreviewTiming::ControllerValidation);
    if(hasPendingStep_){
        return invalidStep(StatusCode::PendingStepNotCommitted,
                           "the previous step must be committed or discarded",
                           dtSec);
    }

    std::string configError;
    if(!validateConfig(config, &configError)){
        return invalidStep(StatusCode::InvalidConfiguration,
                           configError.c_str(),
                           dtSec);
    }
    if(!std::isfinite(dtSec) ||
            dtSec < config.minDtSec ||
            dtSec > config.maxDtSec){
        return invalidStep(StatusCode::InvalidTimeStep,
                           "dtSec is outside the configured valid range",
                           dtSec);
    }
    if(!reference.pose.allFinite() ||
            !reference.velocity.allFinite() ||
            !reference.acceleration.allFinite()){
        return invalidStep(StatusCode::InvalidReference,
                           "reference contains a non-finite value",
                           dtSec);
    }
    if(!feedback.pose.allFinite() || !feedback.velocity.allFinite()){
        return invalidStep(StatusCode::InvalidFeedback,
                           "feedback contains a non-finite value",
                           dtSec);
    }
    if(!dynamics.mass.allFinite() ||
            !dynamics.coriolis.allFinite() ||
            !dynamics.generalizedForce.allFinite()){
        return invalidStep(StatusCode::InvalidDynamics,
                           "effective dynamics contain a non-finite value",
                           dtSec);
    }

    validationTiming.finish();
    EightCablePreviewSection massTiming(timing, EightCablePreviewTiming::ControllerMassCheck);
    Step step;
    step.dtSec = dtSec;
    step.integralEnabled = config.integralEnabled;
    const double massScale =
            std::max(1.0, dynamics.mass.cwiseAbs().maxCoeff());
    step.massSymmetryError =
            (dynamics.mass - dynamics.mass.transpose()).cwiseAbs().maxCoeff();
    if(step.massSymmetryError > config.massSymmetryTolerance * massScale){
        return invalidStep(StatusCode::InvalidEffectiveMass,
                           "effective mass is not symmetric within tolerance",
                           dtSec);
    }

    const Matrix6d effectiveMass =
            0.5 * (dynamics.mass + dynamics.mass.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix6d> massSolver(
                effectiveMass, Eigen::EigenvaluesOnly);
    if(massSolver.info() != Eigen::Success ||
            !massSolver.eigenvalues().allFinite()){
        return invalidStep(StatusCode::InvalidEffectiveMass,
                           "effective mass eigenvalue decomposition failed",
                           dtSec);
    }
    step.minimumMassEigenvalue = massSolver.eigenvalues().minCoeff();
    if(step.minimumMassEigenvalue < config.minimumMassEigenvalue){
        return invalidStep(StatusCode::InvalidEffectiveMass,
                           "effective mass is not positive definite",
                           dtSec);
    }

    massTiming.finish();
    step.poseError = calculatePoseError(reference.pose, feedback.pose);
    step.filteredError = step.poseError;
    convertReferenceKinematics(reference,
                               step.referenceVelocity,
                               step.referenceAcceleration);
    step.velocityError = step.referenceVelocity - feedback.velocity;

    step.integralBefore = integral_;
    if(config.integralEnabled){
        // MATLAB outer loop integrates the SO(3) pose error exactly once per
        // outer-loop update. Commit still provides command-aware anti-windup.
        step.integralCandidate = integral_ + dtSec * step.poseError;
        for(int axis = 0; axis < kTaskDof; ++axis){
            const double limit = config.integralLimit(axis);
            const double limited =
                    std::clamp(step.integralCandidate(axis), -limit, limit);
            if(limited != step.integralCandidate(axis)){
                step.integralLimited = true;
            }
            step.integralCandidate(axis) = limited;
        }
    }
    else{
        step.integralBefore.setZero();
        step.integralCandidate.setZero();
    }

    const Matrix3d actualRotation = rotationMatrix(feedback.pose);
    step.virtualTaskInertia.setZero();
    step.virtualTaskInertia.topLeftCorner<3, 3>() =
            config.virtualMass.asDiagonal();
    step.virtualTaskInertia.bottomRightCorner<3, 3>() =
            actualRotation * config.virtualBodyInertia *
            actualRotation.transpose();
    Eigen::LDLT<Matrix6d> virtualMassSolver(step.virtualTaskInertia);
    if(virtualMassSolver.info() != Eigen::Success ||
            !virtualMassSolver.isPositive()){
        return invalidStep(StatusCode::InvalidConfiguration,
                           "virtual task inertia factorization failed",
                           dtSec);
    }

    step.stiffnessTerm =
            config.stiffness.cwiseProduct(step.poseError);
    EightCablePreviewSection dampingTiming(timing, EightCablePreviewTiming::ControllerModalDamping);
    if(!modalDampingMatrix(step.virtualTaskInertia,
                           config.stiffness,
                           config.dampingRatio,
                           step.dampingMatrix)){
        return invalidStep(StatusCode::InvalidConfiguration,
                           "modal damping matrix calculation failed",
                           dtSec);
    }
    dampingTiming.finish();
    step.dampingTerm = step.dampingMatrix * step.velocityError;
    if(config.integralEnabled){
        step.integralTerm =
                config.integralGain.cwiseProduct(step.integralCandidate);
    }
    step.virtualImpedanceWrench =
            step.stiffnessTerm + step.dampingTerm + step.integralTerm;
    step.accelerationFeedback =
            virtualMassSolver.solve(step.virtualImpedanceWrench);
    step.accelerationCommand =
            step.referenceAcceleration + step.accelerationFeedback;

    step.inertiaTerm = effectiveMass * step.accelerationCommand;
    step.coriolisTerm = dynamics.coriolis * feedback.velocity;
    step.modelForceTerm = dynamics.generalizedForce;
    step.generalizedControl =
            step.inertiaTerm +
            step.coriolisTerm +
            step.modelForceTerm;
    step.inertialFeedforwardWrench =
            effectiveMass * step.referenceAcceleration +
            step.coriolisTerm + step.modelForceTerm;
    step.physicalFeedbackWrench =
            step.generalizedControl - step.inertialFeedforwardWrench;

    if(!step.poseError.allFinite() ||
            !step.velocityError.allFinite() ||
            !step.filteredError.allFinite() ||
            !step.referenceVelocity.allFinite() ||
            !step.referenceAcceleration.allFinite() ||
            !step.integralCandidate.allFinite() ||
            !step.virtualTaskInertia.allFinite() ||
            !step.dampingMatrix.allFinite() ||
            !step.virtualImpedanceWrench.allFinite() ||
            !step.accelerationFeedback.allFinite() ||
            !step.accelerationCommand.allFinite() ||
            !step.generalizedControl.allFinite()){
        return invalidStep(StatusCode::NonFiniteOutput,
                           "controller calculation produced a non-finite value",
                           dtSec);
    }

    step.valid = true;
    step.status = StatusCode::Ok;
    pendingStep_ = step;
    hasPendingStep_ = true;
    return step;
}

void TaskSpaceTorqueController::commit(bool commandApplied, bool outputLimited)
{
    if(!hasPendingStep_ || !pendingStep_.valid){
        return;
    }
    if(!pendingStep_.integralEnabled){
        integral_.setZero();
    }
    else if(commandApplied && !outputLimited){
        integral_ = pendingStep_.integralCandidate;
    }
    hasPendingStep_ = false;
    pendingStep_ = Step();
}

void TaskSpaceTorqueController::discardPending()
{
    commit(false, true);
}

void TaskSpaceTorqueController::reset()
{
    integral_.setZero();
    hasPendingStep_ = false;
    pendingStep_ = Step();
}

bool TaskSpaceTorqueController::hasPendingStep() const
{
    return hasPendingStep_;
}

const TaskSpaceTorqueController::Vector6d&
TaskSpaceTorqueController::integralState() const
{
    return integral_;
}

TaskSpaceTorqueController::Vector6d
TaskSpaceTorqueController::calculatePoseError(const Vector6d& desiredPose,
                                               const Vector6d& actualPose)
{
    Vector6d error = Vector6d::Zero();
    error.head<3>() = desiredPose.head<3>() - actualPose.head<3>();
    const Matrix3d rotationError =
            rotationMatrix(desiredPose) * rotationMatrix(actualPose).transpose();
    Eigen::Quaterniond errorQuaternion(rotationError);
    errorQuaternion.normalize();
    if(errorQuaternion.w() < 0.0){
        errorQuaternion.coeffs() *= -1.0;
    }
    const Vector3d vectorPart = errorQuaternion.vec();
    const double vectorNorm = vectorPart.norm();
    if(vectorNorm > 1.0e-12){
        const double angle = 2.0 * std::atan2(vectorNorm,
                                               errorQuaternion.w());
        error.tail<3>() = (angle / vectorNorm) * vectorPart;
    }
    return error;
}

