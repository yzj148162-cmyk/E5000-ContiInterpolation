#include "CdprKinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kMinimumCableLengthM = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

bool finiteVector3(const CdprVector3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

template <size_t Size>
bool finiteArray(const std::array<double, Size> &values)
{
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

CdprVector3 subtract(const CdprVector3 &left, const CdprVector3 &right)
{
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

CdprVector3 cross(const CdprVector3 &left, const CdprVector3 &right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

double norm(const CdprVector3 &value)
{
    return std::sqrt(value.x * value.x + value.y * value.y
                     + value.z * value.z);
}

std::array<double, 9> rotationMatrix(const CdprVector6 &pose)
{
    const double sx = std::sin(pose[3]);
    const double cx = std::cos(pose[3]);
    const double sy = std::sin(pose[4]);
    const double cy = std::cos(pose[4]);
    const double sz = std::sin(pose[5]);
    const double cz = std::cos(pose[5]);
    return {
        cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
        sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
        -sy,     cy * sx,                cy * cx
    };
}

CdprVector3 rotate(const std::array<double, 9> &matrix,
                   const CdprVector3 &value)
{
    return {
        matrix[0] * value.x + matrix[1] * value.y + matrix[2] * value.z,
        matrix[3] * value.x + matrix[4] * value.y + matrix[5] * value.z,
        matrix[6] * value.x + matrix[7] * value.y + matrix[8] * value.z
    };
}

CdprVector3 translated(const CdprVector3 &value, const CdprVector6 &pose)
{
    return {value.x + pose[0], value.y + pose[1], value.z + pose[2]};
}

double dotRow(const std::array<double, kCdprDofCount> &row,
              const CdprVector6 &vector)
{
    double result = 0.0;
    for (int index = 0; index < kCdprDofCount; ++index) {
        result += row[static_cast<size_t>(index)]
            * vector[static_cast<size_t>(index)];
    }
    return result;
}

double normalizedAngle(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

bool solveLinear6(std::array<std::array<double, 6>, 6> matrix,
                  std::array<double, 6> right,
                  std::array<double, 6> &solution)
{
    for (int column = 0; column < 6; ++column) {
        int pivot = column;
        for (int row = column + 1; row < 6; ++row) {
            if (std::abs(matrix[static_cast<size_t>(row)]
                                  [static_cast<size_t>(column)])
                > std::abs(matrix[static_cast<size_t>(pivot)]
                                    [static_cast<size_t>(column)])) {
                pivot = row;
            }
        }
        if (std::abs(matrix[static_cast<size_t>(pivot)]
                           [static_cast<size_t>(column)]) < 1.0e-14) {
            return false;
        }
        if (pivot != column) {
            std::swap(matrix[static_cast<size_t>(pivot)],
                      matrix[static_cast<size_t>(column)]);
            std::swap(right[static_cast<size_t>(pivot)],
                      right[static_cast<size_t>(column)]);
        }

        const double divisor =
            matrix[static_cast<size_t>(column)][static_cast<size_t>(column)];
        for (int index = column; index < 6; ++index) {
            matrix[static_cast<size_t>(column)][static_cast<size_t>(index)]
                /= divisor;
        }
        right[static_cast<size_t>(column)] /= divisor;

        for (int row = 0; row < 6; ++row) {
            if (row == column) {
                continue;
            }
            const double factor =
                matrix[static_cast<size_t>(row)][static_cast<size_t>(column)];
            for (int index = column; index < 6; ++index) {
                matrix[static_cast<size_t>(row)][static_cast<size_t>(index)]
                    -= factor
                    * matrix[static_cast<size_t>(column)]
                            [static_cast<size_t>(index)];
            }
            right[static_cast<size_t>(row)]
                -= factor * right[static_cast<size_t>(column)];
        }
    }
    solution = right;
    return finiteArray(solution);
}

void residualMetrics(const CdprVector8 &predicted,
                     const CdprVector8 &measured,
                     CdprVector8 &residual,
                     double &rms,
                     double &maximum)
{
    double squareSum = 0.0;
    maximum = 0.0;
    for (int index = 0; index < kCdprCableCount; ++index) {
        const size_t offset = static_cast<size_t>(index);
        residual[offset] = predicted[offset] - measured[offset];
        squareSum += residual[offset] * residual[offset];
        maximum = std::max(maximum, std::abs(residual[offset]));
    }
    rms = std::sqrt(squareSum / static_cast<double>(kCdprCableCount));
}
}

CdprKinematics::CdprKinematics(const CdprConfiguration &configuration)
{
    for (int index = 0; index < kCdprCableCount; ++index) {
        const CdprCableAxisConfig &cable =
            configuration.cables[static_cast<size_t>(index)];
        frameAnchorsM_[static_cast<size_t>(index)] = cable.frameAnchorM;
        platformAnchorsM_[static_cast<size_t>(index)] = cable.platformAnchorM;
    }
}

bool CdprKinematics::geometryValid(QString *errorText) const
{
    for (int index = 0; index < kCdprCableCount; ++index) {
        if (!finiteVector3(frameAnchorsM_[static_cast<size_t>(index)])
            || !finiteVector3(platformAnchorsM_[static_cast<size_t>(index)])) {
            if (errorText) {
                *errorText =
                    QStringLiteral("第%1根绳索的连接点坐标不是有限数。")
                        .arg(index);
            }
            return false;
        }
    }
    if (errorText) {
        errorText->clear();
    }
    return true;
}

CdprInverseKinematicsResult CdprKinematics::inverse(
    const CdprPlatformState6 &platform) const
{
    CdprInverseKinematicsResult result;
    if (!platform.poseValid || !finiteArray(platform.pose)) {
        result.errorText = QStringLiteral("末端位姿无效。");
        return result;
    }
    if (platform.twistValid && !finiteArray(platform.twist)) {
        result.errorText = QStringLiteral("末端速度无效。");
        return result;
    }
    if (!geometryValid(&result.errorText)) {
        return result;
    }

    const std::array<double, 9> rotation = rotationMatrix(platform.pose);
    const CdprVector3 platformOrigin {
        platform.pose[0], platform.pose[1], platform.pose[2]
    };

    for (int index = 0; index < kCdprCableCount; ++index) {
        const size_t offset = static_cast<size_t>(index);
        const CdprVector3 radius =
            rotate(rotation, platformAnchorsM_[offset]);
        const CdprVector3 platformPoint =
            translated(radius, platform.pose);
        const CdprVector3 cableVector =
            subtract(frameAnchorsM_[offset], platformPoint);
        const double length = norm(cableVector);
        if (!std::isfinite(length) || length <= kMinimumCableLengthM) {
            result.errorText =
                QStringLiteral("第%1根绳索长度无效或连接点重合。").arg(index);
            return result;
        }
        const CdprVector3 unit {
            cableVector.x / length,
            cableVector.y / length,
            cableVector.z / length
        };
        const CdprVector3 moment = cross(
            subtract(platformPoint, platformOrigin), unit);

        // 绳长增加为正，因此该行是 G302 结构矩阵对应行的负值。
        result.lengthJacobian[offset] = {
            -unit.x, -unit.y, -unit.z,
            -moment.x, -moment.y, -moment.z
        };
        result.platformAnchorWorldM[offset] = platformPoint;
        result.cableUnitVector[offset] = unit;
        result.cables.lengthM[offset] = length;
        if (platform.twistValid) {
            result.cables.velocityMps[offset] =
                dotRow(result.lengthJacobian[offset], platform.twist);
        }
    }

    result.cables.lengthValid = true;
    result.cables.velocityValid = platform.twistValid;
    // Jdot*qdot 尚未进入第一阶段，不能把不完整的加速度标成有效。
    result.cables.accelerationValid = false;
    result.valid = true;
    return result;
}

bool CdprKinematics::evaluateLengths(const CdprVector6 &pose,
                                     CdprVector8 &lengths,
                                     QString *errorText) const
{
    CdprPlatformState6 platform;
    platform.pose = pose;
    platform.poseValid = true;
    const CdprInverseKinematicsResult result = inverse(platform);
    if (!result.valid) {
        if (errorText) {
            *errorText = result.errorText;
        }
        return false;
    }
    lengths = result.cables.lengthM;
    if (errorText) {
        errorText->clear();
    }
    return true;
}

CdprForwardKinematicsResult CdprKinematics::forward(
    const CdprVector8 &measuredLengthsM,
    const CdprVector6 &initialGuess,
    const CdprForwardKinematicsOptions &options) const
{
    CdprForwardKinematicsResult result;
    if (!finiteArray(measuredLengthsM)
        || std::any_of(measuredLengthsM.begin(), measuredLengthsM.end(),
                       [](double value) {
                           return value <= kMinimumCableLengthM;
                       })) {
        result.errorText = QStringLiteral("正运动学输入绳长无效。");
        return result;
    }
    if (!finiteArray(initialGuess)
        || options.maximumIterations <= 0
        || options.residualToleranceM <= 0.0
        || options.translationDifferenceStepM <= 0.0
        || options.angleDifferenceStepRad <= 0.0
        || options.initialDamping <= 0.0) {
        result.errorText = QStringLiteral("正运动学初值或求解参数无效。");
        return result;
    }

    result.valid = true;
    result.pose = initialGuess;
    double damping = options.initialDamping;

    for (int iteration = 0; iteration < options.maximumIterations;
         ++iteration) {
        result.iterations = iteration + 1;
        CdprVector8 predicted {};
        if (!evaluateLengths(result.pose, predicted, &result.errorText)) {
            result.valid = false;
            return result;
        }

        CdprVector8 residual {};
        residualMetrics(predicted, measuredLengthsM, residual,
                        result.rmsResidualM, result.maximumResidualM);
        if (result.maximumResidualM <= options.residualToleranceM) {
            result.converged = true;
            result.errorText.clear();
            return result;
        }

        CdprMatrix8x6 numericalJacobian {};
        for (int dof = 0; dof < kCdprDofCount; ++dof) {
            const double step = dof < 3
                ? options.translationDifferenceStepM
                : options.angleDifferenceStepRad;
            CdprVector6 plusPose = result.pose;
            CdprVector6 minusPose = result.pose;
            plusPose[static_cast<size_t>(dof)] += step;
            minusPose[static_cast<size_t>(dof)] -= step;
            CdprVector8 plusLengths {};
            CdprVector8 minusLengths {};
            if (!evaluateLengths(plusPose, plusLengths, &result.errorText)
                || !evaluateLengths(minusPose, minusLengths,
                                    &result.errorText)) {
                result.valid = false;
                return result;
            }
            for (int cable = 0; cable < kCdprCableCount; ++cable) {
                numericalJacobian[static_cast<size_t>(cable)]
                    [static_cast<size_t>(dof)] =
                    (plusLengths[static_cast<size_t>(cable)]
                     - minusLengths[static_cast<size_t>(cable)])
                    / (2.0 * step);
            }
        }

        std::array<std::array<double, 6>, 6> normal {};
        std::array<double, 6> right {};
        for (int row = 0; row < 6; ++row) {
            for (int column = 0; column < 6; ++column) {
                for (int cable = 0; cable < kCdprCableCount; ++cable) {
                    normal[static_cast<size_t>(row)]
                          [static_cast<size_t>(column)]
                        += numericalJacobian[static_cast<size_t>(cable)]
                                             [static_cast<size_t>(row)]
                         * numericalJacobian[static_cast<size_t>(cable)]
                                             [static_cast<size_t>(column)];
                }
            }
            normal[static_cast<size_t>(row)][static_cast<size_t>(row)]
                += damping;
            for (int cable = 0; cable < kCdprCableCount; ++cable) {
                right[static_cast<size_t>(row)]
                    -= numericalJacobian[static_cast<size_t>(cable)]
                                         [static_cast<size_t>(row)]
                     * residual[static_cast<size_t>(cable)];
            }
        }

        std::array<double, 6> increment {};
        if (!solveLinear6(normal, right, increment)) {
            result.valid = false;
            result.errorText = QStringLiteral("正运动学法方程奇异。");
            return result;
        }

        bool accepted = false;
        double scale = 1.0;
        for (int attempt = 0; attempt < 8; ++attempt) {
            CdprVector6 candidate = result.pose;
            for (int dof = 0; dof < 6; ++dof) {
                candidate[static_cast<size_t>(dof)]
                    += scale * increment[static_cast<size_t>(dof)];
            }
            for (int dof = 3; dof < 6; ++dof) {
                candidate[static_cast<size_t>(dof)] =
                    normalizedAngle(candidate[static_cast<size_t>(dof)]);
            }
            CdprVector8 candidateLengths {};
            if (!evaluateLengths(candidate, candidateLengths,
                                 &result.errorText)) {
                scale *= 0.5;
                continue;
            }
            CdprVector8 candidateResidual {};
            double candidateRms = 0.0;
            double candidateMaximum = 0.0;
            residualMetrics(candidateLengths, measuredLengthsM,
                            candidateResidual, candidateRms,
                            candidateMaximum);
            if (candidateRms < result.rmsResidualM) {
                result.pose = candidate;
                result.rmsResidualM = candidateRms;
                result.maximumResidualM = candidateMaximum;
                accepted = true;
                damping = std::max(options.initialDamping, damping * 0.2);
                break;
            }
            scale *= 0.5;
        }
        if (!accepted) {
            damping *= 10.0;
            if (!std::isfinite(damping) || damping > 1.0e12) {
                result.errorText = QStringLiteral("正运动学迭代未能降低残差。");
                return result;
            }
        }
    }

    CdprVector8 finalLengths {};
    if (evaluateLengths(result.pose, finalLengths, &result.errorText)) {
        CdprVector8 finalResidual {};
        residualMetrics(finalLengths, measuredLengthsM, finalResidual,
                        result.rmsResidualM, result.maximumResidualM);
    }
    result.errorText = QStringLiteral(
        "正运动学在%1次迭代内未收敛，最大绳长残差=%2 m。")
        .arg(options.maximumIterations)
        .arg(result.maximumResidualM, 0, 'g', 6);
    return result;
}
