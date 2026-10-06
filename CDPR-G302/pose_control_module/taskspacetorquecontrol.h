#ifndef TASKSPACETORQUECONTROL_H
#define TASKSPACETORQUECONTROL_H

#include <Eigen/Core>
#include "eightcablepreviewtiming.h"

#include <string>

// Pure 6-DOF task-space controller. Forward kinematics, motor-torque
// allocation, hardware I/O and scheduling belong to separate modules.
// One runtime worker should own one instance; this class is not thread-safe.
class TaskSpaceTorqueController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int kTaskDof = 6;
    static constexpr int kCableCount = 8;
    using Vector6d = Eigen::Matrix<double, kTaskDof, 1>;
    using Matrix6d = Eigen::Matrix<double, kTaskDof, kTaskDof>;
    using Vector3d = Eigen::Vector3d;
    using Matrix3d = Eigen::Matrix3d;
    using Vector8d = Eigen::Matrix<double, kCableCount, 1>;
    using Matrix8x6d = Eigen::Matrix<double, kCableCount, kTaskDof>;

    struct Reference
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector6d pose = Vector6d::Zero();
        // The trajectory interface remains [position; ZYX Euler angle]. The
        // angular entries below are Euler derivatives and are converted to a
        // global angular velocity/acceleration inside update().
        Vector6d velocity = Vector6d::Zero();
        Vector6d acceleration = Vector6d::Zero();
    };

    struct Feedback
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector6d pose = Vector6d::Zero();
        Vector6d velocity = Vector6d::Zero();
    };

    // Physical rigid-platform inverse dynamics:
    // wrench = M * acceleration + C * measured_twist + generalizedForce.
    // The runtime currently fixes the estimated external wrench at zero.
    struct EffectiveDynamics
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Matrix6d mass = Matrix6d::Identity();
        Matrix6d coriolis = Matrix6d::Zero();
        Vector6d generalizedForce = Vector6d::Zero();
    };

    // Pose-dependent physical model. Cable/motor terms are retained in this
    // interface for source compatibility with the first controller, but the
    // MATLAB strategy deliberately excludes actuator inertia and friction from
    // the platform inverse dynamics and open-loop tension-to-torque mapping.
    struct ModelTerms
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Matrix6d platformMass = Matrix6d::Identity();
        Matrix6d platformCoriolis = Matrix6d::Zero();
        Vector6d gravity = Vector6d::Zero();
        Matrix8x6d cableJacobian = Matrix8x6d::Zero();
        Matrix8x6d cableJacobianDerivative = Matrix8x6d::Zero();
        Vector8d effectiveRadius = Vector8d::Ones();
        Vector8d motorInertia = Vector8d::Zero();
        Vector8d viscousFriction = Vector8d::Zero();
        Vector8d coulombFriction = Vector8d::Zero();

        // satSign(v) = tanh(v / frictionVelocityScale). Units must match
        // cableJacobian * taskVelocity.
        double frictionVelocityScale = 1.0e-4;
    };

    struct DynamicsBuildResult
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool valid = false;
        std::string message;
        EffectiveDynamics dynamics;
        Vector8d cableVelocity = Vector8d::Zero();
        Vector8d smoothCableVelocitySign = Vector8d::Zero();
    };

    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Deprecated source-compatibility fields from the former filtered-
        // error controller. The MATLAB controller intentionally ignores them.
        Vector6d lambda = Vector6d::Zero();
        double mu = 1.0;
        bool wrapEulerAngleError = true;

        Vector3d virtualMass = Vector3d::Ones();
        Matrix3d virtualBodyInertia = Matrix3d::Identity();
        // Uniform modal damping ratio. The full pose-dependent 6x6 damping
        // matrix is derived from Md and K on every outer-loop update.
        double dampingRatio = 0.95;
        Vector6d stiffness = Vector6d::Zero();
        Vector6d integralGain = Vector6d::Zero();
        Vector6d integralLimit = Vector6d::Zero();
        bool integralEnabled = false;
        double minDtSec = 1.0e-6;
        double maxDtSec = 0.1;
        double massSymmetryTolerance = 1.0e-8;
        double minimumMassEigenvalue = 1.0e-9;
    };

    enum class StatusCode
    {
        Ok,
        PendingStepNotCommitted,
        InvalidTimeStep,
        InvalidReference,
        InvalidFeedback,
        InvalidConfiguration,
        InvalidDynamics,
        InvalidEffectiveMass,
        NonFiniteOutput
    };

    struct Step
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool valid = false;
        StatusCode status = StatusCode::InvalidConfiguration;
        std::string message;
        double dtSec = 0.0;
        Vector6d poseError = Vector6d::Zero();
        Vector6d velocityError = Vector6d::Zero();
        // Retained diagnostic alias; equals poseError for this controller.
        Vector6d filteredError = Vector6d::Zero();
        // Physical global twist and acceleration after converting the
        // trajectory's ZYX Euler derivatives.
        Vector6d referenceVelocity = Vector6d::Zero();
        Vector6d referenceAcceleration = Vector6d::Zero();
        Vector6d integralBefore = Vector6d::Zero();
        Vector6d integralCandidate = Vector6d::Zero();
        bool integralEnabled = false;
        bool integralLimited = false;
        Matrix6d virtualTaskInertia = Matrix6d::Identity();
        Matrix6d dampingMatrix = Matrix6d::Zero();
        Vector6d stiffnessTerm = Vector6d::Zero();
        Vector6d inertiaTerm = Vector6d::Zero();
        Vector6d coriolisTerm = Vector6d::Zero();
        Vector6d modelForceTerm = Vector6d::Zero();
        Vector6d dampingTerm = Vector6d::Zero();
        Vector6d integralTerm = Vector6d::Zero();
        Vector6d virtualImpedanceWrench = Vector6d::Zero();
        Vector6d accelerationFeedback = Vector6d::Zero();
        Vector6d accelerationCommand = Vector6d::Zero();
        Vector6d inertialFeedforwardWrench = Vector6d::Zero();
        Vector6d physicalFeedbackWrench = Vector6d::Zero();
        Vector6d generalizedControl = Vector6d::Zero();
        double massSymmetryError = 0.0;
        double minimumMassEigenvalue = 0.0;
    };

    // update() creates a pending integral candidate. A second update is
    // rejected until commit() or discardPending() resolves that candidate.
    Step update(const Reference& reference,
                const Feedback& feedback,
                const EffectiveDynamics& dynamics,
                double dtSec,
                const Config& config,
                EightCablePreviewTiming* timing = nullptr);

    // Integral is committed only when the command was applied without limiting.
    // Sample history advances for every valid step to prevent retroactive
    // integration after a frozen cycle.
    void commit(bool commandApplied, bool outputLimited);
    void discardPending();
    void reset();

    bool hasPendingStep() const;
    const Vector6d& integralState() const;
    static Vector6d calculatePoseError(const Vector6d& desiredPose,
                                       const Vector6d& actualPose);
    static DynamicsBuildResult buildEffectiveDynamics(
            const ModelTerms& model,
            const Vector6d& taskVelocity);
    static bool validateConfig(const Config& config,
                               std::string* errorMessage = nullptr);

private:
    Vector6d integral_ = Vector6d::Zero();
    bool hasPendingStep_ = false;
    Step pendingStep_;
};

#endif // TASKSPACETORQUECONTROL_H

