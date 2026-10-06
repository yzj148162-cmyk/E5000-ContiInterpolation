#ifndef REDUNDANTTORQUEALLOCATOR_H
#define REDUNDANTTORQUEALLOCATOR_H

#include "taskspacetorquecontrol.h"

#include <Eigen/Core>
#include "eightcablepreviewtiming.h"

#include <string>

// Solve A*T=w in the two-dimensional tension null space. Direct torque and
// tension-reference callers share geometry and safety bounds; reference callers
// explicitly select the MATLAB area centroid or the legacy closest target.
class RedundantTorqueAllocator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int kTaskDof = TaskSpaceTorqueController::kTaskDof;
    static constexpr int kCableCount = TaskSpaceTorqueController::kCableCount;
    static constexpr int kNullspaceDof = kCableCount - kTaskDof;

    using Vector2d = Eigen::Matrix<double, kNullspaceDof, 1>;
    using Matrix2d = Eigen::Matrix<double, kNullspaceDof, kNullspaceDof>;
    using Vector6d = TaskSpaceTorqueController::Vector6d;
    using Vector8d = TaskSpaceTorqueController::Vector8d;
    using Matrix8x6d = TaskSpaceTorqueController::Matrix8x6d;
    using Matrix6x8d = Eigen::Matrix<double, kTaskDof, kCableCount>;
    using Matrix8x2d = Eigen::Matrix<double, kCableCount, kNullspaceDof>;

    enum class TensionReferenceSelection { ClosestPrevious, AreaCentroid };

    struct Request
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Required by the MATLAB open-loop path.
        Vector6d generalizedControl = Vector6d::Zero();
        Matrix8x6d cableJacobian = Matrix8x6d::Zero();
        Vector8d effectiveRadius = Vector8d::Ones();
        // Optional affine map in RAW hardware torque coordinates:
        // tau = (-hardwareDirection*r + slopeCorrection)*T + offset.
        // Forward/inverse mapping, quantization and all bounds use the same map.
        bool affineTorqueMapping = false;
        Vector8d hardwareTorqueSlopeCorrection = Vector8d::Zero(); // Nm/N
        Vector8d hardwareTorqueOffset = Vector8d::Zero(); // Nm

        // Deprecated source-compatibility inputs; intentionally ignored.
        Matrix8x6d cableJacobianDerivative = Matrix8x6d::Zero();
        Vector6d referenceVelocity = Vector6d::Zero();
        Vector6d referenceAcceleration = Vector6d::Zero();
        Vector8d motorInertia = Vector8d::Zero();
        Vector8d viscousFriction = Vector8d::Zero();
        Vector8d coulombFriction = Vector8d::Zero();
        double frictionVelocityScale = 1.0e-4;
        double dtSec = 0.0;
        bool tensionReferenceOnly = false;
        // Only used by tensionReferenceOnly. Preserve existing 0525 callers.
        TensionReferenceSelection tensionReferenceSelection = TensionReferenceSelection::ClosestPrevious;
        // Ignored by AreaCentroid: neither initial measurement nor history selects the centroid.
        Vector8d previousTargetTension = Vector8d::Zero();
        bool initializationStrategy = false;
        double hardwareTorqueQuantumNm = 0.0; // 0: pure algorithm/unquantized caller.
        bool hasPreviousHardwareTorque = false;
        Vector8d previousHardwareTorque = Vector8d::Zero();
    };

    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector8d tensionMinimum = Vector8d::Zero();
        Vector8d tensionMaximum = Vector8d::Zero();
        Vector8d tensionBias = Vector8d::Zero();
        Vector8d tensionWeight = Vector8d::Ones();
        Vector8d hardwareTorqueMinimum = Vector8d::Zero();
        Vector8d hardwareTorqueMaximum = Vector8d::Zero();

        // physicalTorque -> hardwareTorque. Each value must be +1 or -1.
        Vector8d hardwareDirection = Vector8d::Ones();

        bool torqueSlewEnabled = false;
        Vector8d hardwareTorqueSlewRate = Vector8d::Zero();

        // Deprecated QP compatibility fields. tensionBias remains active only
        // in the coordinator's explicit preload state, not in area-centroid
        // allocation.
        double torqueContinuityWeight = 0.0;
        double nullspaceRegularization = 1.0e-10;

        double minimumDtSec = 1.0e-6;
        double maximumDtSec = 0.1;
        double relativeSingularValueTolerance = 1.0e-9;
        double absoluteSingularValueTolerance = 1.0e-12;
        double linearConstraintTolerance = 1.0e-12;
        double feasibilityTolerance = 1.0e-9;
        double generalizedControlTolerance = 1.0e-8;
    };

    enum class StatusCode
    {
        Ok,
        InvalidRequest,
        InvalidConfiguration,
        InvalidTimeStep,
        RankDeficient,
        Infeasible,
        NumericalFailure,
        GeneralizedControlResidualExceeded
    };

    struct Result
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool valid = false;
        StatusCode status = StatusCode::InvalidConfiguration;
        std::string message;

        Matrix6x8d allocationMatrix = Matrix6x8d::Zero();
        Matrix8x2d nullspaceBasis = Matrix8x2d::Zero();
        Vector2d nullspaceCoordinates = Vector2d::Zero();
        Vector6d singularValues = Vector6d::Zero();
        int allocationRank = 0;
        double allocationConditionNumber = 0.0;

        // nullspaceBasis and coordinates parameterize tension T=Tp+N*z.
        Vector8d particularTension = Vector8d::Zero();
        Vector8d cableReferenceVelocity = Vector8d::Zero();
        Vector8d cableReferenceAcceleration = Vector8d::Zero();
        Vector8d smoothCableVelocitySign = Vector8d::Zero();
        Vector8d motorDynamicTorque = Vector8d::Zero();
        Vector8d particularMotorTorque = Vector8d::Zero();
        Vector8d physicalMotorTorque = Vector8d::Zero();
        Vector8d hardwareMotorTorque = Vector8d::Zero();
        Vector8d predictedTension = Vector8d::Zero();
        Vector8d effectiveHardwareTorqueMinimum = Vector8d::Zero();
        Vector8d effectiveHardwareTorqueMaximum = Vector8d::Zero();

        Vector6d generalizedControlRequested = Vector6d::Zero();
        Vector6d generalizedControlAchieved = Vector6d::Zero();
        Vector6d generalizedControlResidual = Vector6d::Zero();
        double maximumGeneralizedControlResidual = 0.0;

        double objectiveValue = 0.0;
        double feasiblePolygonArea = 0.0;
        int feasibleCandidateCount = 0;
        int activeConstraintCount = 0;
        bool tensionReferenceOnly = false;
        Vector8d previousTargetTension = Vector8d::Zero();
        bool initializationStrategy = false;
        bool exactWrenchFeasible = false;
        bool wrenchLimited = false;
        int solverIterations = 0;
        std::int64_t solveDurationUs = 0;
        double maximumNormalizedWrenchResidual = 0.0;
        Vector6d wrenchNormalization = Vector6d::Ones();
        Vector6d quantizationWrenchAllowance = Vector6d::Zero();
        Vector8d unquantizedHardwareTorque = Vector8d::Zero();
        Vector8d nominalHardwareTorque = Vector8d::Zero();
        Vector8d feedforwardHardwareTorque = Vector8d::Zero();
        Vector8d hardwareTorqueScale = Vector8d::Zero(); // Nm/N
        Vector8d hardwareTorqueOffset = Vector8d::Zero(); // Nm
        bool constrainedSolution = false;
        bool slewConstraintApplied = false;
        double minimumTensionMargin = 0.0;
        double minimumHardwareTorqueMargin = 0.0;
        double minimumHardwareSlewMargin = 0.0;
    };

    Result solve(const Request& request, const Config& config,
                 EightCablePreviewTiming* timing = nullptr) const;

    static bool validateConfig(const Config& config,
                               std::string* errorMessage = nullptr);
};

#endif // REDUNDANTTORQUEALLOCATOR_H

