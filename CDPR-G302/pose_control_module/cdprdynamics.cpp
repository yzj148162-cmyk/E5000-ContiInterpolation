#include "cdprdynamics.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace {

using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;

bool finite6(const ForceInteractionVector6& values)
{
    for(double value : values){
        if(!std::isfinite(value)){
            return false;
        }
    }
    return true;
}

Matrix3 matrix3(const ForceInteractionMatrix3& values)
{
    Matrix3 result;
    result << values[0], values[1], values[2],
              values[3], values[4], values[5],
              values[6], values[7], values[8];
    return result;
}

Matrix3 rotationZyx(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    Matrix3 result;
    result << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
              sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
              -sp, cp * sr, cp * cr;
    return result;
}

Matrix3 eulerRateToBodyOmega(double roll, double pitch)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    Matrix3 result;
    result << 1.0, 0.0, -sp,
              0.0, cr, sr * cp,
              0.0, -sr, cr * cp;
    return result;
}

Matrix3 eulerRateMatrixDerivative(double roll, double pitch,
                                  double rollRate, double pitchRate)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    Matrix3 result;
    result << 0.0, 0.0, -cp * pitchRate,
              0.0, -sr * rollRate,
              cr * rollRate * cp - sr * sp * pitchRate,
              0.0, -cr * rollRate,
              -sr * rollRate * cp - cr * sp * pitchRate;
    return result;
}

bool solve3(const Matrix3& matrix, const Vector3& right, Vector3& solution)
{
    const Eigen::FullPivLU<Matrix3> decomposition(matrix);
    if(!decomposition.isInvertible()){
        return false;
    }
    solution = decomposition.solve(right);
    return solution.allFinite();
}

double normDifference(const ForceInteractionVector6& left,
                      const ForceInteractionVector6& right)
{
    double sum = 0.0;
    for(int index = 0; index < kForceInteractionDofCount; ++index){
        const double difference = left[static_cast<size_t>(index)] -
                right[static_cast<size_t>(index)];
        sum += difference * difference;
    }
    return std::sqrt(sum);
}

} // namespace

bool CdprDynamics::configure(const ForceInteractionRigidBodyConfig& rigidBody,
                             const NewmarkBetaConfig& newmark,
                             QString* errorMessage)
{
    configured_ = false;
    initialized_ = false;
    const Matrix3 inertia = matrix3(rigidBody.inertiaKgM2);
    if(!std::isfinite(rigidBody.massKg) || rigidBody.massKg <= 0.0 ||
            !inertia.allFinite() ||
            !inertia.isApprox(inertia.transpose(), 1.0e-9) ||
            Eigen::LLT<Matrix3>(inertia).info() != Eigen::Success){
        if(errorMessage){
            *errorMessage = QStringLiteral("刚体质量必须为正，惯量矩阵必须有限、对称且正定");
        }
        return false;
    }
    if(!std::isfinite(newmark.beta) || !std::isfinite(newmark.gamma) ||
            !std::isfinite(newmark.convergenceTolerance) ||
            newmark.beta <= 0.0 || newmark.gamma <= 0.0 ||
            newmark.convergenceTolerance <= 0.0 || newmark.maximumIterations <= 0){
        if(errorMessage){
            *errorMessage = QStringLiteral("Newmark参数无效");
        }
        return false;
    }
    rigidBody_ = rigidBody;
    newmark_ = newmark;
    configured_ = true;
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool CdprDynamics::reset(const ForceInteractionPlatformState& initialState,
                         QString* errorMessage)
{
    initialized_ = false;
    if(!configured_ || !initialState.poseValid || !finite6(initialState.pose) ||
            (initialState.twistValid && !finite6(initialState.twist)) ||
            (initialState.accelerationValid && !finite6(initialState.acceleration))){
        if(errorMessage){
            *errorMessage = QStringLiteral("Newmark未配置或初始状态无效");
        }
        return false;
    }
    pose_ = initialState.pose;
    generalizedVelocity_ = {};
    generalizedAcceleration_ = {};
    for(int index = 0; index < 3; ++index){
        generalizedVelocity_[static_cast<size_t>(index)] =
                initialState.twistValid ? initialState.twist[static_cast<size_t>(index)] : 0.0;
        generalizedAcceleration_[static_cast<size_t>(index)] =
                initialState.accelerationValid ? initialState.acceleration[static_cast<size_t>(index)] : 0.0;
    }

    const Matrix3 rotation = rotationZyx(pose_[3], pose_[4], pose_[5]);
    const Matrix3 rateMatrix = eulerRateToBodyOmega(pose_[3], pose_[4]);
    const Vector3 worldOmega = initialState.twistValid ?
                Vector3(initialState.twist[3], initialState.twist[4], initialState.twist[5]) :
                Vector3::Zero();
    Vector3 eulerRate;
    if(!solve3(rateMatrix, rotation.transpose() * worldOmega, eulerRate)){
        if(errorMessage){
            *errorMessage = QStringLiteral("初始姿态接近ZYX欧拉角奇异点，无法换算角速度");
        }
        return false;
    }
    generalizedVelocity_[3] = eulerRate.x();
    generalizedVelocity_[4] = eulerRate.y();
    generalizedVelocity_[5] = eulerRate.z();

    if(initialState.accelerationValid){
        const Vector3 worldAlpha(initialState.acceleration[3],
                                 initialState.acceleration[4],
                                 initialState.acceleration[5]);
        const Vector3 bodyAlpha = rotation.transpose() * worldAlpha;
        const Matrix3 derivative = eulerRateMatrixDerivative(
                    pose_[3], pose_[4], eulerRate.x(), eulerRate.y());
        Vector3 eulerAcceleration;
        if(!solve3(rateMatrix, bodyAlpha - derivative * eulerRate,
                   eulerAcceleration)){
            if(errorMessage){
                *errorMessage = QStringLiteral("初始角加速度换算失败");
            }
            return false;
        }
        generalizedAcceleration_[3] = eulerAcceleration.x();
        generalizedAcceleration_[4] = eulerAcceleration.y();
        generalizedAcceleration_[5] = eulerAcceleration.z();
    }
    initialized_ = true;
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

CdprDynamicsStepResult CdprDynamics::step(
        const ForceInteractionWrenchSample& platformWrench,
        double timeStepSecond)
{
    CdprDynamicsStepResult result;
    if(!configured_ || !initialized_){
        result.errorMessage = QStringLiteral("CdprDynamics未配置或未初始化");
        return result;
    }
    if(!platformWrench.valid || !platformWrench.stamp.valid ||
            platformWrench.coordinate !=
                ForceInteractionWrenchCoordinate::PlatformBodyAtCenterOfMass ||
            !finite6(platformWrench.wrench)){
        result.errorMessage = QStringLiteral("Newmark输入必须是平台质心body frame下的有效力旋量帧");
        return result;
    }
    if(!std::isfinite(timeStepSecond) || timeStepSecond <= 0.0){
        result.errorMessage = QStringLiteral("Newmark时间步长必须大于0");
        return result;
    }

    ForceInteractionVector6 currentAcceleration{};
    if(!acceleration(pose_, generalizedVelocity_, platformWrench.wrench,
                     currentAcceleration, &result.errorMessage)){
        return result;
    }
    ForceInteractionVector6 predictedPose{};
    ForceInteractionVector6 predictedVelocity{};
    for(int index = 0; index < kForceInteractionDofCount; ++index){
        const size_t offset = static_cast<size_t>(index);
        predictedPose[offset] = pose_[offset] +
                timeStepSecond * generalizedVelocity_[offset] +
                0.5 * timeStepSecond * timeStepSecond *
                (1.0 - 2.0 * newmark_.beta) * currentAcceleration[offset];
        predictedVelocity[offset] = generalizedVelocity_[offset] +
                timeStepSecond * (1.0 - newmark_.gamma) * currentAcceleration[offset];
    }

    ForceInteractionVector6 accelerationGuess = currentAcceleration;
    ForceInteractionVector6 candidatePose{};
    ForceInteractionVector6 candidateVelocity{};
    ForceInteractionVector6 newAcceleration{};
    for(int iteration = 0; iteration < newmark_.maximumIterations; ++iteration){
        result.iterations = iteration + 1;
        for(int index = 0; index < kForceInteractionDofCount; ++index){
            const size_t offset = static_cast<size_t>(index);
            candidatePose[offset] = predictedPose[offset] +
                    newmark_.beta * timeStepSecond * timeStepSecond *
                    accelerationGuess[offset];
            candidateVelocity[offset] = predictedVelocity[offset] +
                    newmark_.gamma * timeStepSecond * accelerationGuess[offset];
        }
        if(!acceleration(candidatePose, candidateVelocity, platformWrench.wrench,
                         newAcceleration, &result.errorMessage)){
            return result;
        }
        result.residual = normDifference(newAcceleration, accelerationGuess);
        if(result.residual <= newmark_.convergenceTolerance){
            result.converged = true;
            break;
        }
        accelerationGuess = newAcceleration;
    }
    if(!result.converged || !finite6(candidatePose) ||
            !finite6(candidateVelocity) || !finite6(newAcceleration)){
        result.errorMessage = result.converged ?
                    QStringLiteral("Newmark计算产生非有限数") :
                    QStringLiteral("Newmark固定点迭代未收敛：%1次，残差=%2")
                    .arg(newmark_.maximumIterations)
                    .arg(result.residual, 0, 'g', 8);
        return result;
    }

    pose_ = candidatePose;
    generalizedVelocity_ = candidateVelocity;
    generalizedAcceleration_ = newAcceleration;
    result.state = externalState(pose_, generalizedVelocity_, generalizedAcceleration_);
    result.valid = result.state.poseValid && result.state.twistValid &&
            result.state.accelerationValid;
    return result;
}

bool CdprDynamics::configured() const
{
    return configured_;
}

bool CdprDynamics::initialized() const
{
    return initialized_;
}

ForceInteractionPlatformState CdprDynamics::currentState() const
{
    return initialized_ ? externalState(pose_, generalizedVelocity_,
                                        generalizedAcceleration_) :
                          ForceInteractionPlatformState{};
}

bool CdprDynamics::acceleration(
        const ForceInteractionVector6& pose,
        const ForceInteractionVector6& generalizedVelocity,
        const ForceInteractionVector6& bodyWrench,
        ForceInteractionVector6& generalizedAcceleration,
        QString* errorMessage) const
{
    const Matrix3 rotation = rotationZyx(pose[3], pose[4], pose[5]);
    const Vector3 worldForce = rotation *
            Vector3(bodyWrench[0], bodyWrench[1], bodyWrench[2]);
    generalizedAcceleration[0] = worldForce.x() / rigidBody_.massKg;
    generalizedAcceleration[1] = worldForce.y() / rigidBody_.massKg;
    generalizedAcceleration[2] = worldForce.z() / rigidBody_.massKg;

    const Matrix3 rateMatrix = eulerRateToBodyOmega(pose[3], pose[4]);
    const Vector3 eulerRate(generalizedVelocity[3],
                            generalizedVelocity[4],
                            generalizedVelocity[5]);
    const Vector3 bodyOmega = rateMatrix * eulerRate;
    const Matrix3 inertia = matrix3(rigidBody_.inertiaKgM2);
    Vector3 bodyAlpha;
    if(!solve3(inertia,
               Vector3(bodyWrench[3], bodyWrench[4], bodyWrench[5]) -
               bodyOmega.cross(inertia * bodyOmega), bodyAlpha)){
        if(errorMessage){
            *errorMessage = QStringLiteral("刚体惯量矩阵求解失败");
        }
        return false;
    }
    const Matrix3 derivative = eulerRateMatrixDerivative(
                pose[3], pose[4], eulerRate.x(), eulerRate.y());
    Vector3 eulerAcceleration;
    if(!solve3(rateMatrix, bodyAlpha - derivative * eulerRate,
               eulerAcceleration)){
        if(errorMessage){
            *errorMessage = QStringLiteral("姿态接近ZYX欧拉角奇异点，无法求角加速度");
        }
        return false;
    }
    generalizedAcceleration[3] = eulerAcceleration.x();
    generalizedAcceleration[4] = eulerAcceleration.y();
    generalizedAcceleration[5] = eulerAcceleration.z();
    return finite6(generalizedAcceleration);
}

ForceInteractionPlatformState CdprDynamics::externalState(
        const ForceInteractionVector6& pose,
        const ForceInteractionVector6& generalizedVelocity,
        const ForceInteractionVector6& generalizedAcceleration) const
{
    ForceInteractionPlatformState result;
    result.pose = pose;
    for(int index = 0; index < 3; ++index){
        result.twist[static_cast<size_t>(index)] =
                generalizedVelocity[static_cast<size_t>(index)];
        result.acceleration[static_cast<size_t>(index)] =
                generalizedAcceleration[static_cast<size_t>(index)];
    }
    const Matrix3 rotation = rotationZyx(pose[3], pose[4], pose[5]);
    const Matrix3 rateMatrix = eulerRateToBodyOmega(pose[3], pose[4]);
    const Vector3 eulerRate(generalizedVelocity[3],
                            generalizedVelocity[4],
                            generalizedVelocity[5]);
    const Vector3 eulerAcceleration(generalizedAcceleration[3],
                                    generalizedAcceleration[4],
                                    generalizedAcceleration[5]);
    const Vector3 worldOmega = rotation * rateMatrix * eulerRate;
    const Matrix3 derivative = eulerRateMatrixDerivative(
                pose[3], pose[4], eulerRate.x(), eulerRate.y());
    const Vector3 worldAlpha = rotation *
            (rateMatrix * eulerAcceleration + derivative * eulerRate);
    result.twist[3] = worldOmega.x();
    result.twist[4] = worldOmega.y();
    result.twist[5] = worldOmega.z();
    result.acceleration[3] = worldAlpha.x();
    result.acceleration[4] = worldAlpha.y();
    result.acceleration[5] = worldAlpha.z();
    result.poseValid = finite6(result.pose);
    result.twistValid = finite6(result.twist);
    result.accelerationValid = finite6(result.acceleration);
    return result;
}
