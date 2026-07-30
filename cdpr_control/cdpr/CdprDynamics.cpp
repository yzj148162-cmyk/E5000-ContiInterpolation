#include "CdprDynamics.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {
using Matrix3 = std::array<double, 9>;

bool finite6(const CdprVector6 &values)
{
    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}
bool finite9(const Matrix3 &values)
{
    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

CdprVector3 multiply(const Matrix3 &matrix, const CdprVector3 &value)
{
    return {
        matrix[0] * value.x + matrix[1] * value.y + matrix[2] * value.z,
        matrix[3] * value.x + matrix[4] * value.y + matrix[5] * value.z,
        matrix[6] * value.x + matrix[7] * value.y + matrix[8] * value.z
    };
}

CdprVector3 transposeMultiply(const Matrix3 &matrix,
                              const CdprVector3 &value)
{
    return {
        matrix[0] * value.x + matrix[3] * value.y + matrix[6] * value.z,
        matrix[1] * value.x + matrix[4] * value.y + matrix[7] * value.z,
        matrix[2] * value.x + matrix[5] * value.y + matrix[8] * value.z
    };
}

CdprVector3 cross(const CdprVector3 &left, const CdprVector3 &right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

Matrix3 rotationZyx(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return {
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr
    };
}

Matrix3 eulerRateToBodyOmega(double roll, double pitch)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    return {
        1.0, 0.0, -sp,
        0.0, cr, sr * cp,
        0.0, -sr, cr * cp
    };
}

Matrix3 eulerRateMatrixDerivative(double roll, double pitch,
                                  double rollRate, double pitchRate)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    return {
        0.0, 0.0, -cp * pitchRate,
        0.0, -sr * rollRate,
        cr * rollRate * cp - sr * sp * pitchRate,
        0.0, -cr * rollRate,
        -sr * rollRate * cp - cr * sp * pitchRate
    };
}

bool solve3(Matrix3 matrix, CdprVector3 right, CdprVector3 &solution)
{
    std::array<std::array<double, 4>, 3> augmented {{
        {{matrix[0], matrix[1], matrix[2], right.x}},
        {{matrix[3], matrix[4], matrix[5], right.y}},
        {{matrix[6], matrix[7], matrix[8], right.z}}
    }};
    for (int column = 0; column < 3; ++column) {
        int pivot = column;
        for (int row = column + 1; row < 3; ++row) {
            if (std::abs(augmented[static_cast<size_t>(row)]
                                  [static_cast<size_t>(column)])
                > std::abs(augmented[static_cast<size_t>(pivot)]
                                    [static_cast<size_t>(column)])) {
                pivot = row;
            }
        }
        if (std::abs(augmented[static_cast<size_t>(pivot)]
                              [static_cast<size_t>(column)]) < 1.0e-12) {
            return false;
        }
        std::swap(augmented[static_cast<size_t>(pivot)],
                  augmented[static_cast<size_t>(column)]);
        const double divisor =
            augmented[static_cast<size_t>(column)]
                     [static_cast<size_t>(column)];
        for (int item = column; item < 4; ++item) {
            augmented[static_cast<size_t>(column)]
                     [static_cast<size_t>(item)] /= divisor;
        }
        for (int row = 0; row < 3; ++row) {
            if (row == column) {
                continue;
            }
            const double factor =
                augmented[static_cast<size_t>(row)]
                         [static_cast<size_t>(column)];
            for (int item = column; item < 4; ++item) {
                augmented[static_cast<size_t>(row)]
                         [static_cast<size_t>(item)]
                    -= factor * augmented[static_cast<size_t>(column)]
                                         [static_cast<size_t>(item)];
            }
        }
    }
    solution = {augmented[0][3], augmented[1][3], augmented[2][3]};
    return std::isfinite(solution.x) && std::isfinite(solution.y)
        && std::isfinite(solution.z);
}

double normDifference(const CdprVector6 &left, const CdprVector6 &right)
{
    double sum = 0.0;
    for (int index = 0; index < 6; ++index) {
        const double difference =
            left[static_cast<size_t>(index)]
            - right[static_cast<size_t>(index)];
        sum += difference * difference;
    }
    return std::sqrt(sum);
}
}

bool CdprDynamics::configure(const CdprRigidBodyConfig &rigidBody,
                             const CdprNewmarkConfig &newmark,
                             QString *errorText)
{
    configured_ = false;
    initialized_ = false;
    if (!std::isfinite(rigidBody.massKg) || rigidBody.massKg <= 0.0
        || !finite9(rigidBody.inertiaKgM2)) {
        if (errorText) {
            *errorText = QStringLiteral("刚体质量或惯量不是有效有限数。");
        }
        return false;
    }
    constexpr double symmetryTolerance = 1.0e-9;
    if (std::abs(rigidBody.inertiaKgM2[1]
                 - rigidBody.inertiaKgM2[3]) > symmetryTolerance
        || std::abs(rigidBody.inertiaKgM2[2]
                    - rigidBody.inertiaKgM2[6]) > symmetryTolerance
        || std::abs(rigidBody.inertiaKgM2[5]
                    - rigidBody.inertiaKgM2[7]) > symmetryTolerance) {
        if (errorText) {
            *errorText = QStringLiteral("刚体惯量矩阵必须对称。");
        }
        return false;
    }
    const double minor1 = rigidBody.inertiaKgM2[0];
    const double minor2 = rigidBody.inertiaKgM2[0]
            * rigidBody.inertiaKgM2[4]
        - rigidBody.inertiaKgM2[1] * rigidBody.inertiaKgM2[3];
    const double determinant =
        rigidBody.inertiaKgM2[0]
            * (rigidBody.inertiaKgM2[4] * rigidBody.inertiaKgM2[8]
               - rigidBody.inertiaKgM2[5] * rigidBody.inertiaKgM2[7])
        - rigidBody.inertiaKgM2[1]
            * (rigidBody.inertiaKgM2[3] * rigidBody.inertiaKgM2[8]
               - rigidBody.inertiaKgM2[5] * rigidBody.inertiaKgM2[6])
        + rigidBody.inertiaKgM2[2]
            * (rigidBody.inertiaKgM2[3] * rigidBody.inertiaKgM2[7]
               - rigidBody.inertiaKgM2[4] * rigidBody.inertiaKgM2[6]);
    if (minor1 <= 0.0 || minor2 <= 0.0 || determinant <= 0.0) {
        if (errorText) {
            *errorText = QStringLiteral("刚体惯量矩阵必须正定。");
        }
        return false;
    }
    if (!std::isfinite(newmark.beta) || !std::isfinite(newmark.gamma)
        || !std::isfinite(newmark.convergenceTolerance)
        || newmark.beta <= 0.0 || newmark.gamma <= 0.0
        || newmark.convergenceTolerance <= 0.0
        || newmark.maximumIterations <= 0) {
        if (errorText) {
            *errorText = QStringLiteral("Newmark参数无效。");
        }
        return false;
    }
    rigidBody_ = rigidBody;
    newmark_ = newmark;
    configured_ = true;
    if (errorText) {
        errorText->clear();
    }
    return true;
}

bool CdprDynamics::reset(const CdprPlatformState6 &initialState,
                         QString *errorText)
{
    initialized_ = false;
    if (!configured_) {
        if (errorText) {
            *errorText = QStringLiteral("CdprDynamics尚未配置。");
        }
        return false;
    }
    if (!initialState.poseValid || !finite6(initialState.pose)
        || (initialState.twistValid && !finite6(initialState.twist))
        || (initialState.accelerationValid
            && !finite6(initialState.acceleration))) {
        if (errorText) {
            *errorText = QStringLiteral("Newmark初始状态无效。");
        }
        return false;
    }
    pose_ = initialState.pose;
    generalizedVelocity_ = {};
    generalizedAcceleration_ = {};
    generalizedVelocity_[0] = initialState.twist[0];
    generalizedVelocity_[1] = initialState.twist[1];
    generalizedVelocity_[2] = initialState.twist[2];

    const Matrix3 rotation =
        rotationZyx(pose_[3], pose_[4], pose_[5]);
    const Matrix3 rateMatrix =
        eulerRateToBodyOmega(pose_[3], pose_[4]);
    const CdprVector3 worldOmega {
        initialState.twist[3], initialState.twist[4],
        initialState.twist[5]
    };
    const CdprVector3 bodyOmega =
        transposeMultiply(rotation, worldOmega);
    CdprVector3 eulerRate;
    if (!solve3(rateMatrix, bodyOmega, eulerRate)) {
        if (errorText) {
            *errorText = QStringLiteral(
                "初始姿态接近ZYX欧拉角奇异点，无法换算角速度。");
        }
        return false;
    }
    generalizedVelocity_[3] = eulerRate.x;
    generalizedVelocity_[4] = eulerRate.y;
    generalizedVelocity_[5] = eulerRate.z;

    if (initialState.accelerationValid) {
        generalizedAcceleration_[0] = initialState.acceleration[0];
        generalizedAcceleration_[1] = initialState.acceleration[1];
        generalizedAcceleration_[2] = initialState.acceleration[2];
        const CdprVector3 worldAlpha {
            initialState.acceleration[3], initialState.acceleration[4],
            initialState.acceleration[5]
        };
        const CdprVector3 bodyAlpha =
            transposeMultiply(rotation, worldAlpha);
        const Matrix3 rateDerivative =
            eulerRateMatrixDerivative(pose_[3], pose_[4],
                                      eulerRate.x, eulerRate.y);
        const CdprVector3 derivativeTerm =
            multiply(rateDerivative, eulerRate);
        CdprVector3 eulerAcceleration;
        if (!solve3(rateMatrix,
                    {bodyAlpha.x - derivativeTerm.x,
                     bodyAlpha.y - derivativeTerm.y,
                     bodyAlpha.z - derivativeTerm.z},
                    eulerAcceleration)) {
            if (errorText) {
                *errorText = QStringLiteral("初始角加速度换算失败。");
            }
            return false;
        }
        generalizedAcceleration_[3] = eulerAcceleration.x;
        generalizedAcceleration_[4] = eulerAcceleration.y;
        generalizedAcceleration_[5] = eulerAcceleration.z;
    }
    initialized_ = true;
    if (errorText) {
        errorText->clear();
    }
    return true;
}

CdprDynamicsResult CdprDynamics::step(
    const CdprWrenchSample &platformWrench, double timeStepSecond)
{
    CdprDynamicsResult result;
    if (!configured_ || !initialized_) {
        result.errorText = QStringLiteral("CdprDynamics未配置或未初始化。");
        return result;
    }
    if (!platformWrench.valid || !platformWrench.stamp.valid
        || platformWrench.coordinate
            != CdprWrenchCoordinate::PlatformBodyAtCenterOfMass
        || !finite6(platformWrench.wrench)) {
        result.errorText = QStringLiteral(
            "Newmark输入必须是平台质心body frame下的有效力旋量帧。");
        return result;
    }
    if (!std::isfinite(timeStepSecond) || timeStepSecond <= 0.0) {
        result.errorText = QStringLiteral("Newmark时间步长必须大于0。");
        return result;
    }

    CdprVector6 currentAcceleration {};
    if (!acceleration(pose_, generalizedVelocity_,
                      platformWrench.wrench, currentAcceleration,
                      &result.errorText)) {
        return result;
    }
    CdprVector6 predictedPose {};
    CdprVector6 predictedVelocity {};
    for (int index = 0; index < 6; ++index) {
        const size_t offset = static_cast<size_t>(index);
        predictedPose[offset] =
            pose_[offset] + timeStepSecond * generalizedVelocity_[offset]
            + 0.5 * timeStepSecond * timeStepSecond
                * (1.0 - 2.0 * newmark_.beta)
                * currentAcceleration[offset];
        predictedVelocity[offset] =
            generalizedVelocity_[offset]
            + timeStepSecond * (1.0 - newmark_.gamma)
                * currentAcceleration[offset];
    }

    CdprVector6 accelerationGuess = currentAcceleration;
    CdprVector6 candidatePose {};
    CdprVector6 candidateVelocity {};
    CdprVector6 newAcceleration {};
    for (int iteration = 0; iteration < newmark_.maximumIterations;
         ++iteration) {
        result.iterations = iteration + 1;
        for (int index = 0; index < 6; ++index) {
            const size_t offset = static_cast<size_t>(index);
            candidatePose[offset] =
                predictedPose[offset]
                + newmark_.beta * timeStepSecond * timeStepSecond
                    * accelerationGuess[offset];
            candidateVelocity[offset] =
                predictedVelocity[offset]
                + newmark_.gamma * timeStepSecond
                    * accelerationGuess[offset];
        }
        if (!acceleration(candidatePose, candidateVelocity,
                          platformWrench.wrench, newAcceleration,
                          &result.errorText)) {
            return result;
        }
        result.residual =
            normDifference(newAcceleration, accelerationGuess);
        if (result.residual <= newmark_.convergenceTolerance) {
            result.converged = true;
            break;
        }
        accelerationGuess = newAcceleration;
    }
    if (!finite6(candidatePose) || !finite6(candidateVelocity)
        || !finite6(newAcceleration)) {
        result.errorText = QStringLiteral("Newmark计算产生非有限数。");
        return result;
    }
    if (!result.converged) {
        result.errorText = QStringLiteral(
            "Newmark固定点迭代未收敛：%1次，残差=%2。")
            .arg(newmark_.maximumIterations)
            .arg(result.residual, 0, 'g', 6);
        return result;
    }

    pose_ = candidatePose;
    generalizedVelocity_ = candidateVelocity;
    generalizedAcceleration_ = newAcceleration;
    result.state = externalState(
        pose_, generalizedVelocity_, generalizedAcceleration_);
    result.valid = result.state.poseValid && result.state.twistValid
        && result.state.accelerationValid;
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

CdprPlatformState6 CdprDynamics::currentState() const
{
    return initialized_
        ? externalState(pose_, generalizedVelocity_,
                        generalizedAcceleration_)
        : CdprPlatformState6 {};
}

CdprNewmarkConfig CdprDynamics::newmarkConfig() const
{
    return newmark_;
}

bool CdprDynamics::acceleration(
    const CdprVector6 &pose,
    const CdprVector6 &generalizedVelocity,
    const CdprVector6 &bodyWrench,
    CdprVector6 &generalizedAcceleration,
    QString *errorText) const
{
    const Matrix3 rotation =
        rotationZyx(pose[3], pose[4], pose[5]);
    const CdprVector3 bodyForce {
        bodyWrench[0], bodyWrench[1], bodyWrench[2]
    };
    const CdprVector3 worldForce = multiply(rotation, bodyForce);
    generalizedAcceleration[0] = worldForce.x / rigidBody_.massKg;
    generalizedAcceleration[1] = worldForce.y / rigidBody_.massKg;
    generalizedAcceleration[2] = worldForce.z / rigidBody_.massKg;

    const Matrix3 rateMatrix =
        eulerRateToBodyOmega(pose[3], pose[4]);
    const CdprVector3 eulerRate {
        generalizedVelocity[3], generalizedVelocity[4],
        generalizedVelocity[5]
    };
    const CdprVector3 bodyOmega = multiply(rateMatrix, eulerRate);
    const CdprVector3 angularMomentum =
        multiply(rigidBody_.inertiaKgM2, bodyOmega);
    const CdprVector3 gyroscopic =
        cross(bodyOmega, angularMomentum);
    const CdprVector3 bodyMoment {
        bodyWrench[3], bodyWrench[4], bodyWrench[5]
    };
    CdprVector3 bodyAlpha;
    if (!solve3(rigidBody_.inertiaKgM2,
                {bodyMoment.x - gyroscopic.x,
                 bodyMoment.y - gyroscopic.y,
                 bodyMoment.z - gyroscopic.z},
                bodyAlpha)) {
        if (errorText) {
            *errorText = QStringLiteral("刚体惯量矩阵求解失败。");
        }
        return false;
    }
    const Matrix3 rateDerivative =
        eulerRateMatrixDerivative(pose[3], pose[4],
                                  eulerRate.x, eulerRate.y);
    const CdprVector3 derivativeTerm =
        multiply(rateDerivative, eulerRate);
    CdprVector3 eulerAcceleration;
    if (!solve3(rateMatrix,
                {bodyAlpha.x - derivativeTerm.x,
                 bodyAlpha.y - derivativeTerm.y,
                 bodyAlpha.z - derivativeTerm.z},
                eulerAcceleration)) {
        if (errorText) {
            *errorText = QStringLiteral(
                "姿态接近ZYX欧拉角奇异点，无法求角加速度。");
        }
        return false;
    }
    generalizedAcceleration[3] = eulerAcceleration.x;
    generalizedAcceleration[4] = eulerAcceleration.y;
    generalizedAcceleration[5] = eulerAcceleration.z;
    return finite6(generalizedAcceleration);
}

CdprPlatformState6 CdprDynamics::externalState(
    const CdprVector6 &pose,
    const CdprVector6 &generalizedVelocity,
    const CdprVector6 &generalizedAcceleration) const
{
    CdprPlatformState6 result;
    result.pose = pose;
    result.twist[0] = generalizedVelocity[0];
    result.twist[1] = generalizedVelocity[1];
    result.twist[2] = generalizedVelocity[2];
    result.acceleration[0] = generalizedAcceleration[0];
    result.acceleration[1] = generalizedAcceleration[1];
    result.acceleration[2] = generalizedAcceleration[2];

    const Matrix3 rotation =
        rotationZyx(pose[3], pose[4], pose[5]);
    const Matrix3 rateMatrix =
        eulerRateToBodyOmega(pose[3], pose[4]);
    const CdprVector3 eulerRate {
        generalizedVelocity[3], generalizedVelocity[4],
        generalizedVelocity[5]
    };
    const CdprVector3 eulerAcceleration {
        generalizedAcceleration[3], generalizedAcceleration[4],
        generalizedAcceleration[5]
    };
    const CdprVector3 bodyOmega = multiply(rateMatrix, eulerRate);
    const CdprVector3 worldOmega = multiply(rotation, bodyOmega);
    result.twist[3] = worldOmega.x;
    result.twist[4] = worldOmega.y;
    result.twist[5] = worldOmega.z;

    const Matrix3 rateDerivative =
        eulerRateMatrixDerivative(pose[3], pose[4],
                                  eulerRate.x, eulerRate.y);
    const CdprVector3 rateAcceleration =
        multiply(rateMatrix, eulerAcceleration);
    const CdprVector3 rateDerivativeTerm =
        multiply(rateDerivative, eulerRate);
    const CdprVector3 worldAlpha = multiply(
        rotation,
        {rateAcceleration.x + rateDerivativeTerm.x,
         rateAcceleration.y + rateDerivativeTerm.y,
         rateAcceleration.z + rateDerivativeTerm.z});
    result.acceleration[3] = worldAlpha.x;
    result.acceleration[4] = worldAlpha.y;
    result.acceleration[5] = worldAlpha.z;
    result.poseValid = finite6(result.pose);
    result.twistValid = finite6(result.twist);
    result.accelerationValid = finite6(result.acceleration);
    return result;
}
