#include "redundanttorqueallocator.h"

#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/StdVector>

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <cmath>
#include <vector>

namespace {

using Allocator = RedundantTorqueAllocator;
using Vector2d = Allocator::Vector2d;
using Matrix2d = Allocator::Matrix2d;
using Vector8d = Allocator::Vector8d;

struct LinearConstraint
{
    Vector2d normal = Vector2d::Zero();
    double upper = 0.0;
};

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

Allocator::Result invalidResult(Allocator::StatusCode status,
                                const char* message)
{
    Allocator::Result result;
    result.status = status;
    result.message = message;
    return result;
}

bool addConstraint(std::vector<LinearConstraint>& constraints,
                   const Vector2d& normal,
                   double upper,
                   double linearTolerance,
                   double feasibilityTolerance)
{
    const double normalNorm = normal.norm();
    if(!std::isfinite(normalNorm) || !std::isfinite(upper)){
        return false;
    }
    if(normalNorm <= linearTolerance){
        return upper >= -feasibilityTolerance;
    }

    LinearConstraint constraint;
    constraint.normal = normal / normalNorm;
    constraint.upper = upper / normalNorm;
    constraints.push_back(constraint);
    return true;
}

bool isFeasible(const Vector2d& value,
                const std::vector<LinearConstraint>& constraints,
                double tolerance)
{
    if(!value.allFinite()){
        return false;
    }
    for(const LinearConstraint& constraint : constraints){
        if(constraint.normal.dot(value) - constraint.upper > tolerance){
            return false;
        }
    }
    return true;
}

struct PolygonCentroid
{
    bool valid = false;
    Vector2d value = Vector2d::Zero();
    std::vector<Vector2d, Eigen::aligned_allocator<Vector2d>> vertices;
    double area = 0.0;
};

PolygonCentroid feasiblePolygonCentroid(
        const std::vector<LinearConstraint>& constraints,
        double linearTolerance,
        double feasibilityTolerance)
{
    PolygonCentroid result;
    for(std::size_t first = 0; first < constraints.size(); ++first){
        for(std::size_t second = first + 1;
            second < constraints.size();
            ++second){
            Matrix2d boundaryMatrix;
            boundaryMatrix.row(0) = constraints[first].normal.transpose();
            boundaryMatrix.row(1) = constraints[second].normal.transpose();
            const double determinant = boundaryMatrix.determinant();
            if(!std::isfinite(determinant) ||
                    std::fabs(determinant) <= linearTolerance){
                continue;
            }
            Vector2d boundaryUpper;
            boundaryUpper << constraints[first].upper,
                    constraints[second].upper;
            const Vector2d candidate =
                    boundaryMatrix.fullPivLu().solve(boundaryUpper);
            if(!isFeasible(candidate, constraints, feasibilityTolerance)){
                continue;
            }
            const bool duplicate = std::any_of(
                        result.vertices.cbegin(),
                        result.vertices.cend(),
                        [&](const Vector2d& existing){
                const double scale = std::max(
                            1.0,
                            std::max(existing.norm(), candidate.norm()));
                return (existing - candidate).norm() <=
                        feasibilityTolerance * scale;
            });
            if(!duplicate){
                result.vertices.push_back(candidate);
            }
        }
    }
    if(result.vertices.size() < 3){
        return result;
    }

    Vector2d center = Vector2d::Zero();
    for(const Vector2d& vertex : result.vertices){
        center += vertex;
    }
    center /= static_cast<double>(result.vertices.size());
    std::sort(result.vertices.begin(), result.vertices.end(),
              [&](const Vector2d& first, const Vector2d& second){
        return std::atan2(first.y() - center.y(), first.x() - center.x()) <
                std::atan2(second.y() - center.y(), second.x() - center.x());
    });

    double doubleSignedArea = 0.0;
    Vector2d centroidNumerator = Vector2d::Zero();
    for(std::size_t index = 0; index < result.vertices.size(); ++index){
        const Vector2d& current = result.vertices[index];
        const Vector2d& next =
                result.vertices[(index + 1) % result.vertices.size()];
        const double cross = current.x() * next.y() - next.x() * current.y();
        doubleSignedArea += cross;
        centroidNumerator += (current + next) * cross;
    }
    if(!std::isfinite(doubleSignedArea) ||
            std::fabs(doubleSignedArea) <=
                std::max(1.0e-12, linearTolerance * linearTolerance)){
        return result;
    }
    result.value = centroidNumerator / (3.0 * doubleSignedArea);
    result.area = 0.5 * std::fabs(doubleSignedArea);
    result.valid = result.value.allFinite() && std::isfinite(result.area);
    return result;
}

double minimumBoxMargin(const Vector8d& value,
                        const Vector8d& minimum,
                        const Vector8d& maximum)
{
    return (value - minimum).cwiseMin(maximum - value).minCoeff();
}


// The initialization strategy shares the matrix/rank checks with the MATLAB
// path, but chooses a continuous feasible output, including degenerate boxes.
bool closestTension(const Allocator::Result& geometry, const Vector8d& lower,
                    const Vector8d& upper, const Vector8d& preferred,
                    const Vector8d& torqueScale, const Allocator::Config& config,
                    Vector8d& tension, int& candidateCount)
{
    std::vector<LinearConstraint> constraints;
    constraints.reserve(16);
    for(int axis=0;axis<Allocator::kCableCount;++axis){
        const Vector2d row=geometry.nullspaceBasis.row(axis).transpose();
        if(!addConstraint(constraints,row,upper[axis]-geometry.particularTension[axis],
                          config.linearConstraintTolerance,config.feasibilityTolerance) ||
           !addConstraint(constraints,-row,geometry.particularTension[axis]-lower[axis],
                          config.linearConstraintTolerance,config.feasibilityTolerance)){
            return false;
        }
    }
    const Allocator::Matrix8x2d weighted=torqueScale.asDiagonal()*geometry.nullspaceBasis;
    const Matrix2d metric=weighted.transpose()*weighted;
    const Eigen::LDLT<Matrix2d> factor(metric);
    if(factor.info()!=Eigen::Success || !factor.isPositive()){ return false; }
    const Vector2d center=factor.solve(weighted.transpose()*
                torqueScale.cwiseProduct(preferred-geometry.particularTension));
    double best=std::numeric_limits<double>::infinity();
    Vector2d selected=Vector2d::Zero();
    const auto consider=[&](const Vector2d& candidate){
        if(!isFeasible(candidate,constraints,config.feasibilityTolerance)){return;}
        ++candidateCount;
        const Vector2d difference=candidate-center;
        const double objective=difference.dot(metric*difference);
        if(std::isfinite(objective) && objective<best){best=objective;selected=candidate;}
    };
    consider(center);
    // Interior, projections on edges, and all vertices also cover line/point sets.
    for(std::size_t i=0;i<constraints.size();++i){
        const auto& row=constraints[i];
        const Vector2d normal=factor.solve(row.normal);
        const double denominator=row.normal.dot(normal);
        if(denominator>0.0){
            consider(center-normal*((row.normal.dot(center)-row.upper)/denominator));
        }
        for(std::size_t j=i+1;j<constraints.size();++j){
            Matrix2d pair;
            pair.row(0)=row.normal.transpose();
            pair.row(1)=constraints[j].normal.transpose();
            if(std::abs(pair.determinant())<=config.linearConstraintTolerance){continue;}
            consider(pair.fullPivLu().solve(Vector2d(row.upper,constraints[j].upper)));
        }
    }
    if(!std::isfinite(best)){return false;}
    tension=geometry.particularTension+geometry.nullspaceBasis*selected;
    return tension.allFinite();
}

// Bounded-variable least squares. Only used when the desired wrench has a
// physically feasible allocation but the current slew window prevents it.
// No regularizer trades wrench error for continuity: free rank-deficient
// directions use the minimum change from the current feasible point.
bool boundedWrenchLeastSquares(const Allocator::Matrix6x8d& matrix,
                              const Allocator::Vector6d& target,
                              const Vector8d& lower,const Vector8d& upper,
                              Vector8d& tension,int& iterations)
{
    std::array<int,8> active{};
    for(int i=0;i<8;++i){
        tension[i]=std::clamp(tension[i],lower[i],upper[i]);
        if(upper[i]-lower[i]<=1e-12){active[i]=2;}
        else if(tension[i]<=lower[i]+1e-12){active[i]=-1;}
        else if(tension[i]>=upper[i]-1e-12){active[i]=1;}
    }
    const double tolerance=1e-10*(1.0+target.norm());
    for(iterations=1;iterations<=128;++iterations){
        std::array<int,8> freeAxes{};
        int count=0;
        for(int i=0;i<8;++i){if(active[i]==0){freeAxes[count++]=i;}}
        Vector8d candidate=tension;
        if(count){
            Eigen::MatrixXd freeMatrix(6,count);
            for(int j=0;j<count;++j){freeMatrix.col(j)=matrix.col(freeAxes[j]);}
            const Eigen::VectorXd change=freeMatrix.completeOrthogonalDecomposition().solve(
                        target-matrix*tension);
            if(!change.allFinite()){return false;}
            for(int j=0;j<count;++j){candidate[freeAxes[j]]+=change[j];}
            double fraction=1.0;
            int hit=-1,side=0;
            for(int j=0;j<count;++j){
                const int i=freeAxes[j];
                const double delta=candidate[i]-tension[i];
                double allowed=1.0;
                int bound=0;
                if(candidate[i]<lower[i]){allowed=(lower[i]-tension[i])/delta;bound=-1;}
                if(candidate[i]>upper[i]){allowed=(upper[i]-tension[i])/delta;bound=1;}
                if(bound && (hit<0 || allowed<fraction)){fraction=allowed;hit=i;side=bound;}
            }
            if(hit>=0){
                tension+=std::clamp(fraction,0.0,1.0)*(candidate-tension);
                tension=tension.cwiseMax(lower).cwiseMin(upper);
                tension[hit]=side<0 ? lower[hit] : upper[hit];
                active[hit]=side;
                continue;
            }
            tension=candidate;
        }
        const Vector8d gradient=matrix.transpose()*(matrix*tension-target);
        int release=-1;
        double violation=tolerance;
        for(int i=0;i<8;++i){
            const double value=active[i]==-1 ? -gradient[i] :
                               active[i]==1 ? gradient[i] : 0.0;
            if(value>violation){violation=value;release=i;}
        }
        if(release<0){
            for(int i=0;i<8;++i){
                if(active[i]==0 && std::abs(gradient[i])>10.0*tolerance){return false;}
            }
            return tension.allFinite();
        }
        active[release]=0;
    }
    return false;
}

Allocator::Result solveInitializationAllocation(const Allocator::Request& request,
        const Allocator::Config& config,Allocator::Result result,
        EightCablePreviewTiming* timing)
{
    EightCablePreviewSection boundedTiming(timing,EightCablePreviewTiming::AllocatorBoundedPolygon);
    const auto start=std::chrono::steady_clock::now();
    result.initializationStrategy=true;
    const auto finish=[&](Allocator::StatusCode status,const char* message,bool valid){
        result.status=status;result.message=message;result.valid=valid;
        result.solveDurationUs=std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now()-start).count();
        return result;
    };
    if(!request.hasPreviousHardwareTorque || !config.torqueSlewEnabled ||
            !std::isfinite(request.hardwareTorqueQuantumNm) || request.hardwareTorqueQuantumNm<0.0){
        return finish(Allocator::StatusCode::InvalidRequest,
                      "initialization allocation requires committed torque, slew bounds and valid quantum",false);
    }
    const Vector8d scale=result.hardwareTorqueScale;
    const Vector8d offset=result.hardwareTorqueOffset;
    Vector8d hardLower,hardUpper,lower,upper;
    const double quantum=request.hardwareTorqueQuantumNm;
    for(int i=0;i<8;++i){
        double torqueLower=std::max(config.hardwareTorqueMinimum[i],
                std::min(scale[i]*config.tensionMinimum[i]+offset[i],scale[i]*config.tensionMaximum[i]+offset[i]));
        double torqueUpper=std::min(config.hardwareTorqueMaximum[i],
                std::max(scale[i]*config.tensionMinimum[i]+offset[i],scale[i]*config.tensionMaximum[i]+offset[i]));
        const auto snap=[&](double& lo,double& hi){
            if(quantum>0.0){
                lo=std::ceil(lo/quantum-1e-10)*quantum;
                hi=std::floor(hi/quantum+1e-10)*quantum;
            }
            return std::isfinite(lo) && std::isfinite(hi) && lo<=hi;
        };
        // Test physical wrench feasibility before discretization. A valid
        // boundary wrench may need the explicitly reported half-count allowance.
        hardLower[i]=std::min((torqueLower-offset[i])/scale[i],(torqueUpper-offset[i])/scale[i]);
        hardUpper[i]=std::max((torqueLower-offset[i])/scale[i],(torqueUpper-offset[i])/scale[i]);
        if(!snap(torqueLower,torqueUpper)){
            return finish(Allocator::StatusCode::Infeasible,
                          "no representable torque satisfies physical command bounds",false);
        }
        const double step=config.hardwareTorqueSlewRate[i]*request.dtSec;
        torqueLower=std::max(torqueLower,request.previousHardwareTorque[i]-step);
        torqueUpper=std::min(torqueUpper,request.previousHardwareTorque[i]+step);
        if(!snap(torqueLower,torqueUpper)){
            return finish(Allocator::StatusCode::Infeasible,
                          "no representable torque satisfies the committed-output slew window",false);
        }
        result.effectiveHardwareTorqueMinimum[i]=torqueLower;
        result.effectiveHardwareTorqueMaximum[i]=torqueUpper;
        lower[i]=std::min((torqueLower-offset[i])/scale[i],(torqueUpper-offset[i])/scale[i]);
        upper[i]=std::max((torqueLower-offset[i])/scale[i],(torqueUpper-offset[i])/scale[i]);
        // Each wrench component is normalized by its own configured tension
        // authority (N or Nm), rather than adding differently dimensioned errors.
    }
    for(int row=0;row<6;++row){
        result.wrenchNormalization[row]=std::max(1e-9,
                    result.allocationMatrix.row(row).cwiseAbs().dot(
                        config.tensionMinimum.cwiseAbs().cwiseMax(config.tensionMaximum.cwiseAbs())));
    }
    result.slewConstraintApplied=true;
    const Vector8d previous=(request.previousHardwareTorque-offset).cwiseQuotient(scale);
    Vector8d selected;
    const bool exact=closestTension(result,lower,upper,previous,scale,config,
                                   selected,result.feasibleCandidateCount);
    result.exactWrenchFeasible=exact;
    if(!exact){
        Vector8d unrestricted;
        int hardCandidates=0;
        if(!closestTension(result,hardLower,hardUpper,previous,scale,config,
                           unrestricted,hardCandidates)){
            return finish(Allocator::StatusCode::Infeasible,
                          "requested wrench is infeasible even without slew limits",false);
        }
        selected=previous.cwiseMax(lower).cwiseMin(upper);
        const Allocator::Matrix6x8d normalized=
                result.wrenchNormalization.cwiseInverse().asDiagonal()*result.allocationMatrix;
        const Allocator::Vector6d demand=request.generalizedControl.cwiseQuotient(result.wrenchNormalization);
        if(!boundedWrenchLeastSquares(normalized,demand,lower,upper,selected,result.solverIterations)){
            return finish(Allocator::StatusCode::NumericalFailure,
                          "bounded wrench solver did not converge within 128 iterations",false);
        }
    }
    result.unquantizedHardwareTorque=scale.cwiseProduct(selected)+offset;
    result.hardwareMotorTorque=result.unquantizedHardwareTorque;
    if(quantum>0.0){
        for(int i=0;i<8;++i){
            // Exact zero is permitted; the hardware's minimum nonzero raw rule
            // is irrelevant because only integer raw multiples leave this path.
            result.hardwareMotorTorque[i]=std::clamp(
                        std::round(result.hardwareMotorTorque[i]/quantum)*quantum,
                        result.effectiveHardwareTorqueMinimum[i],
                        result.effectiveHardwareTorqueMaximum[i]);
        }
        result.quantizationWrenchAllowance=result.allocationMatrix.cwiseAbs()*
                (Vector8d::Constant(0.5*quantum).cwiseQuotient(scale.cwiseAbs()));
    }
    result.predictedTension=(result.hardwareMotorTorque-offset).cwiseQuotient(scale);
    result.nominalHardwareTorque=-config.hardwareDirection.cwiseProduct(request.effectiveRadius)
            .cwiseProduct(result.predictedTension);
    result.feedforwardHardwareTorque=result.hardwareMotorTorque-result.nominalHardwareTorque;
    result.physicalMotorTorque=config.hardwareDirection.cwiseProduct(result.hardwareMotorTorque);
    result.nullspaceCoordinates=result.nullspaceBasis.transpose()*
            (result.predictedTension-result.particularTension);
    result.generalizedControlAchieved=result.allocationMatrix*result.predictedTension;
    result.generalizedControlResidual=result.generalizedControlAchieved-request.generalizedControl;
    result.maximumGeneralizedControlResidual=result.generalizedControlResidual.cwiseAbs().maxCoeff();
    result.maximumNormalizedWrenchResidual=result.generalizedControlResidual.cwiseAbs().
            cwiseQuotient(result.wrenchNormalization).maxCoeff();
    const double numericAllowance=std::max(1e-8,config.generalizedControlTolerance*
                                           std::max(1.0,request.generalizedControl.cwiseAbs().maxCoeff()));
    result.wrenchLimited=(result.generalizedControlResidual.cwiseAbs().array()>
                          result.quantizationWrenchAllowance.array()+numericAllowance).any();
    result.constrainedSolution=true;
    result.minimumTensionMargin=minimumBoxMargin(result.predictedTension,
                                                 config.tensionMinimum,config.tensionMaximum);
    result.minimumHardwareTorqueMargin=minimumBoxMargin(result.hardwareMotorTorque,
                                            config.hardwareTorqueMinimum,config.hardwareTorqueMaximum);
    result.minimumHardwareSlewMargin=minimumBoxMargin(result.hardwareMotorTorque,
                    result.effectiveHardwareTorqueMinimum,result.effectiveHardwareTorqueMaximum);
    if(!result.hardwareMotorTorque.allFinite() || result.minimumTensionMargin < -1e-8 ||
            result.minimumHardwareTorqueMargin < -1e-8 || result.minimumHardwareSlewMargin < -1e-8){
        return finish(Allocator::StatusCode::NumericalFailure,
                      "initialization allocation failed final quantized bounds",false);
    }
    return finish(Allocator::StatusCode::Ok,result.wrenchLimited ?
                  "initialization A: slew-limited wrench" : "initialization A: wrench realized",true);
}

} // namespace

bool RedundantTorqueAllocator::validateConfig(
        const Config& config,
        std::string* errorMessage)
{
    if(!config.tensionMinimum.allFinite() ||
            !config.tensionMaximum.allFinite() ||
            (config.tensionMaximum.array() <=
             config.tensionMinimum.array()).any()){
        setError(errorMessage, "tension bounds are invalid");
        return false;
    }
    if(!config.tensionBias.allFinite() ||
            (config.tensionBias.array() <
             config.tensionMinimum.array()).any() ||
            (config.tensionBias.array() >
             config.tensionMaximum.array()).any()){
        setError(errorMessage, "tensionBias must lie within tension bounds");
        return false;
    }
    if(!isNonNegativeFinite(config.tensionWeight)){
        setError(errorMessage,
                 "tensionWeight must contain finite non-negative values");
        return false;
    }
    if(!config.hardwareTorqueMinimum.allFinite() ||
            !config.hardwareTorqueMaximum.allFinite() ||
            (config.hardwareTorqueMaximum.array() <
             config.hardwareTorqueMinimum.array()).any()){
        setError(errorMessage, "hardware torque bounds are invalid");
        return false;
    }
    if(!config.hardwareDirection.allFinite()){
        setError(errorMessage, "hardwareDirection contains a non-finite value");
        return false;
    }
    for(int axis = 0; axis < kCableCount; ++axis){
        if(std::fabs(std::fabs(config.hardwareDirection(axis)) - 1.0) >
                1.0e-12){
            setError(errorMessage,
                     "hardwareDirection values must be exactly +1 or -1");
            return false;
        }
    }
    if(!isNonNegativeFinite(config.hardwareTorqueSlewRate)){
        setError(errorMessage,
                 "hardwareTorqueSlewRate must be finite and non-negative");
        return false;
    }
    if(!std::isfinite(config.torqueContinuityWeight) ||
            config.torqueContinuityWeight < 0.0){
        setError(errorMessage,
                 "torqueContinuityWeight must be finite and non-negative");
        return false;
    }
    if(!std::isfinite(config.nullspaceRegularization) ||
            config.nullspaceRegularization <= 0.0){
        setError(errorMessage,
                 "nullspaceRegularization must be finite and positive");
        return false;
    }
    if(!std::isfinite(config.minimumDtSec) || config.minimumDtSec <= 0.0 ||
            !std::isfinite(config.maximumDtSec) ||
            config.maximumDtSec < config.minimumDtSec){
        setError(errorMessage, "time-step bounds are invalid");
        return false;
    }
    if(!std::isfinite(config.relativeSingularValueTolerance) ||
            config.relativeSingularValueTolerance <= 0.0 ||
            !std::isfinite(config.absoluteSingularValueTolerance) ||
            config.absoluteSingularValueTolerance <= 0.0){
        setError(errorMessage, "singular-value tolerances must be positive");
        return false;
    }
    if(!std::isfinite(config.linearConstraintTolerance) ||
            config.linearConstraintTolerance <= 0.0 ||
            !std::isfinite(config.feasibilityTolerance) ||
            config.feasibilityTolerance < 0.0 ||
            !std::isfinite(config.generalizedControlTolerance) ||
            config.generalizedControlTolerance < 0.0){
        setError(errorMessage, "allocation tolerances are invalid");
        return false;
    }

    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

RedundantTorqueAllocator::Result RedundantTorqueAllocator::solve(
        const Request& request,
        const Config& config,
        EightCablePreviewTiming* timing) const
{
    std::string configError;
    if(!validateConfig(config, &configError)){
        return invalidResult(StatusCode::InvalidConfiguration,
                             configError.c_str());
    }
    if(!std::isfinite(request.dtSec) ||
            request.dtSec < config.minimumDtSec ||
            request.dtSec > config.maximumDtSec){
        return invalidResult(StatusCode::InvalidTimeStep,
                             "dtSec is outside the configured valid range");
    }
    if(!request.generalizedControl.allFinite() ||
            !request.cableJacobian.allFinite()){
        return invalidResult(StatusCode::InvalidRequest,
                             "task-space allocation input is non-finite");
    }
    if(!isPositiveFinite(request.effectiveRadius)){
        return invalidResult(StatusCode::InvalidRequest,
                             "effective cable radius is invalid");
    }
    if(request.hasPreviousHardwareTorque &&
            !request.previousHardwareTorque.allFinite()){
        return invalidResult(StatusCode::InvalidRequest,
                             "previous hardware torque is non-finite");
    }

    Result result;
    const auto failed = [&](StatusCode code, const char* message){
        result.valid=false;
        result.status=code;
        result.message=message;
        return result;
    };
    const Vector8d nominalScale = -config.hardwareDirection.cwiseProduct(request.effectiveRadius);
    result.hardwareTorqueScale = nominalScale;
    if(request.affineTorqueMapping){
        if(!request.hardwareTorqueSlopeCorrection.allFinite() || !request.hardwareTorqueOffset.allFinite()){
            return failed(StatusCode::InvalidRequest, "non-finite affine torque map");
        }
        result.hardwareTorqueScale += request.hardwareTorqueSlopeCorrection;
        result.hardwareTorqueOffset = request.hardwareTorqueOffset;
        // Compensation must never reverse the physical force/torque direction
        // or create a nearly singular inverse map / unbounded quantization error.
        for(int axis=0;axis<kCableCount;++axis){
            const double ratio=result.hardwareTorqueScale[axis]/nominalScale[axis];
            if(!std::isfinite(ratio) || ratio<0.5 || ratio>1.5){
                return failed(StatusCode::InvalidRequest, "affine torque slope outside nominal direction/range");
            }
        }
    }
    result.generalizedControlRequested = request.generalizedControl;
    result.allocationMatrix = -request.cableJacobian.transpose();

    EightCablePreviewSection svdTiming(timing, EightCablePreviewTiming::AllocatorSvd);
    Eigen::JacobiSVD<Matrix6x8d> svd(
                result.allocationMatrix,
                Eigen::ComputeFullU | Eigen::ComputeFullV);
    if(svd.info() != Eigen::Success ||
            !svd.singularValues().allFinite() ||
            !svd.matrixV().allFinite()){
        return failed(StatusCode::NumericalFailure,
                             "allocation matrix SVD failed");
    }
    result.singularValues = svd.singularValues();
    const double maximumSingularValue = result.singularValues.maxCoeff();
    const double singularValueThreshold =
            std::max(config.absoluteSingularValueTolerance,
                     config.relativeSingularValueTolerance *
                     maximumSingularValue);
    svd.setThreshold(singularValueThreshold);
    result.allocationRank = static_cast<int>(svd.rank());
    if(result.allocationRank != kTaskDof){
        Result invalid = invalidResult(StatusCode::RankDeficient,
                                       "allocation matrix is not full row rank");
        invalid.allocationMatrix = result.allocationMatrix;
        invalid.singularValues = result.singularValues;
        invalid.allocationRank = result.allocationRank;
        return invalid;
    }
    const double minimumSingularValue =
            result.singularValues(kTaskDof - 1);
    result.allocationConditionNumber =
            maximumSingularValue / minimumSingularValue;
    result.particularTension = svd.solve(request.generalizedControl);
    result.nullspaceBasis =
            svd.matrixV().rightCols<kNullspaceDof>();
    if(!result.particularTension.allFinite() ||
            !result.nullspaceBasis.allFinite()){
        return failed(StatusCode::NumericalFailure,
                             "particular or null-space solution is non-finite");
    }
    result.particularMotorTorque =
            -request.effectiveRadius.cwiseProduct(result.particularTension);

    svdTiming.finish();
    if(request.tensionReferenceOnly){
        // Nominal capability only; nonlinear feedback enforces actual command bounds.
        const bool areaCentroid=request.tensionReferenceSelection==TensionReferenceSelection::AreaCentroid;
        if(request.affineTorqueMapping ||
           (!areaCentroid && (request.tensionReferenceSelection!=TensionReferenceSelection::ClosestPrevious ||
                              !request.previousTargetTension.allFinite()))){
            return failed(StatusCode::InvalidRequest,"invalid tension-reference request");
        }
        Vector8d lower=config.tensionMinimum, upper=config.tensionMaximum;
        for(int i=0;i<kCableCount;++i){
            const double a=nominalScale[i];
            lower[i]=std::max(lower[i],std::min(config.hardwareTorqueMinimum[i]/a,
                                               config.hardwareTorqueMaximum[i]/a));
            upper[i]=std::min(upper[i],std::max(config.hardwareTorqueMinimum[i]/a,
                                               config.hardwareTorqueMaximum[i]/a));
        }
        if((lower.array()>upper.array()).any()){
            return failed(StatusCode::Infeasible,"target wrench is infeasible within tension/nominal capability bounds");
        }
        if(areaCentroid){
            // MATLAB bary_center: enumerate the bounded null-space polygon and
            // use its AREA centroid, not a vertex average or a previous target.
            // Intersect real-machine capability bounds before selecting it.
            std::vector<LinearConstraint> constraints;
            constraints.reserve(2*kCableCount);
            for(int axis=0;axis<kCableCount;++axis){
                const Vector2d row=result.nullspaceBasis.row(axis).transpose();
                if(!addConstraint(constraints,row,upper[axis]-result.particularTension[axis],
                                  config.linearConstraintTolerance,config.feasibilityTolerance) ||
                   !addConstraint(constraints,-row,result.particularTension[axis]-lower[axis],
                                  config.linearConstraintTolerance,config.feasibilityTolerance)){
                    return failed(StatusCode::Infeasible,"area-centroid target has infeasible constant bounds");
                }
            }
            EightCablePreviewSection polygonTiming(timing,EightCablePreviewTiming::AllocatorBoundedPolygon);
            const auto centroid=feasiblePolygonCentroid(constraints,config.linearConstraintTolerance,
                                                        config.feasibilityTolerance);
            result.feasiblePolygonArea=centroid.area;
            result.feasibleCandidateCount=static_cast<int>(centroid.vertices.size());
            if(!centroid.valid || centroid.area<=1.0e-12 ||
               !isFeasible(centroid.value,constraints,config.feasibilityTolerance)){
                return failed(StatusCode::Infeasible,"area-centroid target polygon is empty or has no finite area");
            }
            result.predictedTension=result.particularTension+result.nullspaceBasis*centroid.value;
            if(!result.predictedTension.allFinite()){
                return failed(StatusCode::NumericalFailure,"area-centroid target tension is non-finite");
            }
        } else if(!closestTension(result,lower,upper,request.previousTargetTension,
                                  Vector8d::Ones(),config,result.predictedTension,result.feasibleCandidateCount)){
            return failed(StatusCode::Infeasible,"target wrench is infeasible within tension/nominal capability bounds");
        }
        result.tensionReferenceOnly=true;
        if(!areaCentroid) result.previousTargetTension=request.previousTargetTension;
        result.nominalHardwareTorque=nominalScale.cwiseProduct(result.predictedTension);
        result.hardwareMotorTorque=result.nominalHardwareTorque; // Not an actuator command.
        result.unquantizedHardwareTorque=result.nominalHardwareTorque;
        result.effectiveHardwareTorqueMinimum=config.hardwareTorqueMinimum;
        result.effectiveHardwareTorqueMaximum=config.hardwareTorqueMaximum;
        result.physicalMotorTorque=-request.effectiveRadius.cwiseProduct(result.predictedTension);
        result.generalizedControlAchieved=result.allocationMatrix*result.predictedTension;
        result.generalizedControlResidual=result.generalizedControlAchieved-request.generalizedControl;
        result.maximumGeneralizedControlResidual=result.generalizedControlResidual.cwiseAbs().maxCoeff();
        result.minimumTensionMargin=minimumBoxMargin(result.predictedTension,lower,upper);
        result.minimumHardwareTorqueMargin=minimumBoxMargin(result.nominalHardwareTorque,
                                         config.hardwareTorqueMinimum,config.hardwareTorqueMaximum);
        result.nullspaceCoordinates=result.nullspaceBasis.transpose()*
                                     (result.predictedTension-result.particularTension);
        if(result.minimumTensionMargin < -config.feasibilityTolerance ||
           result.maximumGeneralizedControlResidual > config.generalizedControlTolerance*
                                       std::max(1.0,request.generalizedControl.cwiseAbs().maxCoeff())){
            return failed(StatusCode::GeneralizedControlResidualExceeded,"target allocation residual exceeds tolerance");
        }
        result.exactWrenchFeasible=true; result.valid=true; result.status=StatusCode::Ok;
        result.message=areaCentroid ? "area-centroid target allocated; use measured tension for actual force"
                                    : "target allocated; use measured tension for actual force";
        return result;
    }
    if(request.initializationStrategy){
        return solveInitializationAllocation(request,config,result,timing);
    }
    std::vector<LinearConstraint> tensionConstraints;
    tensionConstraints.reserve(2 * kCableCount);
    bool constraintsValid = true;
    for(int axis = 0; axis < kCableCount; ++axis){
        const Vector2d tensionRow =
                result.nullspaceBasis.row(axis).transpose();
        constraintsValid = constraintsValid && addConstraint(
                    tensionConstraints,
                    tensionRow,
                    config.tensionMaximum(axis) -
                    result.particularTension(axis),
                    config.linearConstraintTolerance,
                    config.feasibilityTolerance);
        constraintsValid = constraintsValid && addConstraint(
                    tensionConstraints,
                    -tensionRow,
                    result.particularTension(axis) -
                    config.tensionMinimum(axis),
                    config.linearConstraintTolerance,
                    config.feasibilityTolerance);
    }
    if(!constraintsValid){
        return failed(StatusCode::Infeasible,
                             "constant tension constraint is infeasible");
    }

    EightCablePreviewSection tensionPolygonTiming(timing, EightCablePreviewTiming::AllocatorTensionPolygon);
    const PolygonCentroid matlabCentroid = feasiblePolygonCentroid(
                tensionConstraints,
                config.linearConstraintTolerance,
                config.feasibilityTolerance);
    tensionPolygonTiming.finish();
    result.feasiblePolygonArea=matlabCentroid.area;
    result.feasibleCandidateCount=static_cast<int>(matlabCentroid.vertices.size());
    if(!matlabCentroid.valid){
        return failed(StatusCode::Infeasible,
                             "tension null-space polygon has no finite area");
    }

    std::vector<LinearConstraint> constraints = tensionConstraints;
    constraints.reserve(4 * kCableCount);
    result.effectiveHardwareTorqueMinimum = config.hardwareTorqueMinimum;
    result.effectiveHardwareTorqueMaximum = config.hardwareTorqueMaximum;
    if(config.torqueSlewEnabled && request.hasPreviousHardwareTorque){
        result.slewConstraintApplied = true;
        const Vector8d maximumStep =
                config.hardwareTorqueSlewRate * request.dtSec;
        result.effectiveHardwareTorqueMinimum =
                result.effectiveHardwareTorqueMinimum.cwiseMax(
                    request.previousHardwareTorque - maximumStep);
        result.effectiveHardwareTorqueMaximum =
                result.effectiveHardwareTorqueMaximum.cwiseMin(
                    request.previousHardwareTorque + maximumStep);
    }
    if((result.effectiveHardwareTorqueMinimum.array() >
        result.effectiveHardwareTorqueMaximum.array() +
        config.feasibilityTolerance).any()){
        return failed(StatusCode::Infeasible,
                             "hardware torque and slew bounds do not intersect");
    }

    for(int axis = 0; axis < kCableCount; ++axis){
        const double torqueScale = result.hardwareTorqueScale(axis);
        const Vector2d hardwareRow =
                torqueScale *
                result.nullspaceBasis.row(axis).transpose();
        const double particularHardwareTorque =
                torqueScale * result.particularTension(axis) + result.hardwareTorqueOffset(axis);
        constraintsValid = constraintsValid && addConstraint(
                    constraints,
                    hardwareRow,
                    result.effectiveHardwareTorqueMaximum(axis) -
                    particularHardwareTorque,
                    config.linearConstraintTolerance,
                    config.feasibilityTolerance);
        constraintsValid = constraintsValid && addConstraint(
                    constraints,
                    -hardwareRow,
                    particularHardwareTorque -
                    result.effectiveHardwareTorqueMinimum(axis),
                    config.linearConstraintTolerance,
                    config.feasibilityTolerance);
    }
    if(!constraintsValid){
        return failed(StatusCode::Infeasible,
                             "constant hardware torque constraint is infeasible");
    }

    EightCablePreviewSection boundedPolygonTiming(timing, EightCablePreviewTiming::AllocatorBoundedPolygon);
    const PolygonCentroid selectedCentroid = feasiblePolygonCentroid(
                constraints,
                config.linearConstraintTolerance,
                config.feasibilityTolerance);
    boundedPolygonTiming.finish();
    if(!selectedCentroid.valid){
        return failed(StatusCode::Infeasible,
                             "no finite-area feasible tension allocation exists");
    }

    result.nullspaceCoordinates = selectedCentroid.value;
    result.feasiblePolygonArea = selectedCentroid.area;
    result.objectiveValue = selectedCentroid.area;
    result.feasibleCandidateCount =
            static_cast<int>(selectedCentroid.vertices.size());
    const double coordinateScale = std::max(
                1.0,
                std::max(matlabCentroid.value.norm(),
                         selectedCentroid.value.norm()));
    result.constrainedSolution =
            (selectedCentroid.value - matlabCentroid.value).norm() >
            config.feasibilityTolerance * coordinateScale;

    result.predictedTension =
            result.particularTension +
            result.nullspaceBasis * result.nullspaceCoordinates;
    result.physicalMotorTorque =
            -request.effectiveRadius.cwiseProduct(result.predictedTension);
    result.nominalHardwareTorque = config.hardwareDirection.cwiseProduct(result.physicalMotorTorque);
    result.hardwareMotorTorque = result.hardwareTorqueScale.cwiseProduct(result.predictedTension)
            + result.hardwareTorqueOffset;
    result.feedforwardHardwareTorque = result.hardwareMotorTorque - result.nominalHardwareTorque;
    result.physicalMotorTorque = config.hardwareDirection.cwiseProduct(result.hardwareMotorTorque);
    result.generalizedControlAchieved =
            result.allocationMatrix * result.predictedTension;
    result.generalizedControlResidual =
            result.generalizedControlAchieved -
            request.generalizedControl;
    result.maximumGeneralizedControlResidual =
            result.generalizedControlResidual.cwiseAbs().maxCoeff();

    for(const LinearConstraint& constraint : constraints){
        if(std::fabs(
                constraint.normal.dot(result.nullspaceCoordinates) -
                constraint.upper) <= config.feasibilityTolerance){
            ++result.activeConstraintCount;
        }
    }

    result.minimumTensionMargin =
            minimumBoxMargin(result.predictedTension,
                             config.tensionMinimum,
                             config.tensionMaximum);
    result.minimumHardwareTorqueMargin =
            minimumBoxMargin(result.hardwareMotorTorque,
                             config.hardwareTorqueMinimum,
                             config.hardwareTorqueMaximum);
    if(result.slewConstraintApplied){
        result.minimumHardwareSlewMargin = minimumBoxMargin(
                    result.hardwareMotorTorque,
                    result.effectiveHardwareTorqueMinimum,
                    result.effectiveHardwareTorqueMaximum);
    }

    const double residualScale =
            std::max(1.0,
                     request.generalizedControl.cwiseAbs().maxCoeff());
    if(result.maximumGeneralizedControlResidual >
            config.generalizedControlTolerance * residualScale){
        result.status = StatusCode::GeneralizedControlResidualExceeded;
        result.message =
                "allocated tension does not reproduce generalized control";
        return result;
    }
    if(result.minimumTensionMargin < -config.feasibilityTolerance ||
            result.minimumHardwareTorqueMargin <
            -config.feasibilityTolerance ||
            (result.slewConstraintApplied &&
             result.minimumHardwareSlewMargin <
             -config.feasibilityTolerance)){
        result.status = StatusCode::NumericalFailure;
        result.message = "selected allocation violates a box constraint";
        return result;
    }

    result.valid = true;
    result.status = StatusCode::Ok;
    result.message.clear();
    return result;
}

