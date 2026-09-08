#include "physicalworkspaceboundary.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;

bool finite3(const Vector3& values)
{
    return std::all_of(values.begin(), values.end(), [](double value){
        return std::isfinite(value);
    });
}

bool finite6(const std::array<double, 6>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value){
        return std::isfinite(value);
    });
}

Matrix3 rotationZyx(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return {{
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr
    }};
}

Vector3 multiply(const Matrix3& matrix, const Vector3& vector)
{
    return {{
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]
    }};
}

Vector3 cross(const Vector3& left, const Vector3& right)
{
    return {{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]
    }};
}

double norm(const Vector3& value)
{
    return std::sqrt(value[0] * value[0] +
                     value[1] * value[1] +
                     value[2] * value[2]);
}

QString faceName(int axis, bool upper)
{
    static const char* const names[] = {"X", "Y", "Z"};
    return QStringLiteral("%1%2")
            .arg(QString::fromLatin1(names[std::clamp(axis, 0, 2)]),
                 upper ? QStringLiteral("上边界") : QStringLiteral("下边界"));
}

struct PointMotion
{
    Vector3 position{};
    Vector3 velocity{};
    Vector3 acceleration{};
};

PointMotion pointMotion(
        const Vector3& localPoint,
        const Matrix3& rotation,
        const Vector3& centerPosition,
        const Vector3& linearVelocity,
        const Vector3& angularVelocity,
        const Vector3& linearAcceleration,
        const Vector3& angularAcceleration)
{
    const Vector3 offset = multiply(rotation, localPoint);
    const Vector3 omegaCrossOffset = cross(angularVelocity, offset);
    const Vector3 alphaCrossOffset = cross(angularAcceleration, offset);
    const Vector3 centripetal = cross(angularVelocity, omegaCrossOffset);
    PointMotion point;
    for(int axis = 0; axis < 3; ++axis){
        point.position[axis] = centerPosition[axis] + offset[axis];
        point.velocity[axis] = linearVelocity[axis] + omegaCrossOffset[axis];
        point.acceleration[axis] = linearAcceleration[axis] +
                alphaCrossOffset[axis] + centripetal[axis];
    }
    return point;
}

} // namespace

bool PhysicalWorkspaceBoundaryConfig::validate(QString* errorMessage) const
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(!finite3(frameMinimumMm) || !finite3(frameMaximumMm)){
        return fail(QStringLiteral("机架物理边界包含非有限数"));
    }
    for(int axis = 0; axis < 3; ++axis){
        if(frameMinimumMm[axis] >= frameMaximumMm[axis]){
            return fail(QStringLiteral("机架第%1维物理边界无效").arg(axis));
        }
    }
    if(platformPointsLocalMm.empty()){
        return fail(QStringLiteral("缺少动平台局部几何点"));
    }
    for(size_t index = 0; index < platformPointsLocalMm.size(); ++index){
        if(!finite3(platformPointsLocalMm[index])){
            return fail(QStringLiteral("动平台局部几何点%1无效").arg(index + 1));
        }
    }
    if(orientationBoundsEnabled){
        if(!finite3(orientationMinimumRad) || !finite3(orientationMaximumRad)){
            return fail(QStringLiteral("姿态物理边界包含非有限数"));
        }
        for(int axis = 0; axis < 3; ++axis){
            if(orientationMinimumRad[axis] >= orientationMaximumRad[axis]){
                return fail(QStringLiteral("姿态第%1维物理边界无效").arg(axis));
            }
        }
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool DynamicWorkspaceSafetyConfig::validate(QString* errorMessage) const
{
    const bool valid = std::isfinite(stoppingDecelerationMmPerSec2) &&
            stoppingDecelerationMmPerSec2 > 0.0 &&
            std::isfinite(additionalSafetyMarginMm) &&
            additionalSafetyMarginMm >= 0.0 &&
            std::isfinite(emergencyLineMarginMm) &&
            emergencyLineMarginMm >= 0.0 &&
            additionalSafetyMarginMm > emergencyLineMarginMm &&
            std::isfinite(outwardVelocityToleranceMmPerSec) &&
            outwardVelocityToleranceMmPerSec >= 0.0 &&
            std::isfinite(outwardAccelerationToleranceMmPerSec2) &&
            outwardAccelerationToleranceMmPerSec2 >= 0.0;
    if(!valid && errorMessage){
        *errorMessage = QStringLiteral(
                    "动态边界参数无效：末端加速度a必须为正，附加余量必须大于固定急停线距离");
    }
    else if(valid && errorMessage){
        errorMessage->clear();
    }
    return valid;
}

PhysicalWorkspaceBoundary::PhysicalWorkspaceBoundary(
        const PhysicalWorkspaceBoundaryConfig& config)
{
    configure(config);
}

bool PhysicalWorkspaceBoundary::configure(
        const PhysicalWorkspaceBoundaryConfig& config,
        QString* errorMessage)
{
    configured_ = false;
    QString error;
    if(!config.validate(&error)){
        if(errorMessage){
            *errorMessage = error;
        }
        return false;
    }
    config_ = config;
    configured_ = true;
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}

bool PhysicalWorkspaceBoundary::isConfigured() const
{
    return configured_;
}

const PhysicalWorkspaceBoundaryConfig& PhysicalWorkspaceBoundary::config() const
{
    return config_;
}

PhysicalWorkspaceBoundaryResult PhysicalWorkspaceBoundary::evaluatePose(
        const std::array<double, 6>& poseMmRad) const
{
    PhysicalWorkspaceMotionSample sample;
    sample.poseMmRad = poseMmRad;
    DynamicWorkspaceSafetyConfig safety;
    safety.stoppingDecelerationMmPerSec2 = 1.0;
    safety.additionalSafetyMarginMm = 1.0;
    safety.emergencyLineMarginMm = 0.0;
    PhysicalWorkspaceBoundaryResult result = evaluateMotion(sample, safety);
    if(result.action == PhysicalWorkspaceAction::ControlledStop){
        result.action = PhysicalWorkspaceAction::Safe;
    }
    return result;
}

PhysicalWorkspaceBoundaryResult PhysicalWorkspaceBoundary::evaluateMotion(
        const PhysicalWorkspaceMotionSample& sample,
        const DynamicWorkspaceSafetyConfig& safety) const
{
    PhysicalWorkspaceBoundaryResult result;
    QString validationError;
    if(!configured_ || !config_.validate(&validationError) ||
            !safety.validate(&validationError) ||
            !finite6(sample.poseMmRad) || !finite6(sample.twistMmRadPerSec) ||
            !finite6(sample.accelerationMmRadPerSec2)){
        result.reason = validationError.isEmpty() ?
                    QStringLiteral("物理工作空间输入状态无效") : validationError;
        return result;
    }

    result.action = PhysicalWorkspaceAction::Safe;
    result.physicallyInside = true;
    result.orientationInside = true;
    result.minimumClearanceMm = std::numeric_limits<double>::infinity();
    double bestControlledRiskMm = std::numeric_limits<double>::infinity();
    double bestEmergencyClearanceMm = std::numeric_limits<double>::infinity();

    const auto assignFace = [&](size_t pointIndex, int axis, bool upper,
                                const PointMotion& point, double clearance,
                                double outwardSpeed, double outwardAcceleration,
                                double pureStoppingDistance,
                                double triggerDistance){
        result.limitingClearanceMm = clearance;
        result.limitingOutwardSpeedMmPerSec = outwardSpeed;
        result.limitingOutwardAccelerationMmPerSec2 = outwardAcceleration;
        result.pureStoppingDistanceMm = pureStoppingDistance;
        result.triggerDistanceMm = triggerDistance;
        result.limitingPointIndex = static_cast<int>(pointIndex);
        result.limitingAxis = axis;
        result.limitingUpperFace = upper;
        result.limitingPointGlobalMm = point.position;
    };

    const Matrix3 rotation = rotationZyx(sample.poseMmRad[3],
                                         sample.poseMmRad[4],
                                         sample.poseMmRad[5]);
    const Vector3 centerPosition{{sample.poseMmRad[0], sample.poseMmRad[1],
                                  sample.poseMmRad[2]}};
    const Vector3 linearVelocity{{sample.twistMmRadPerSec[0],
                                  sample.twistMmRadPerSec[1],
                                  sample.twistMmRadPerSec[2]}};
    const Vector3 angularVelocity{{sample.twistMmRadPerSec[3],
                                   sample.twistMmRadPerSec[4],
                                   sample.twistMmRadPerSec[5]}};
    const Vector3 linearAcceleration{{sample.accelerationMmRadPerSec2[0],
                                      sample.accelerationMmRadPerSec2[1],
                                      sample.accelerationMmRadPerSec2[2]}};
    const Vector3 angularAcceleration{{sample.accelerationMmRadPerSec2[3],
                                       sample.accelerationMmRadPerSec2[4],
                                       sample.accelerationMmRadPerSec2[5]}};
    double maximumPointRadiusMm = 0.0;
    for(const Vector3& localPoint : config_.platformPointsLocalMm){
        maximumPointRadiusMm = std::max(maximumPointRadiusMm,
                                        norm(localPoint));
    }
    // 协同制动按六维速度整体等比例衰减，其等效速度与运行控制器保持一致。
    // 对某一边界法向，恒定速度方向下的实际滑行量为
    // v_out*v_equivalent/(2*a)，斜向或转动运动不能仍按 v_out^2/(2*a) 估算。
    const double brakingEquivalentSpeedMmPerSec =
            norm(linearVelocity) + norm(angularVelocity) * maximumPointRadiusMm;
    result.platformPointCount = std::min(
                static_cast<int>(config_.platformPointsLocalMm.size()),
                kPhysicalWorkspaceMaximumPlatformPoints);
    for(size_t pointIndex = 0;
        pointIndex < config_.platformPointsLocalMm.size(); ++pointIndex){
        const PointMotion point = pointMotion(
                    config_.platformPointsLocalMm[pointIndex], rotation,
                    centerPosition, linearVelocity, angularVelocity,
                    linearAcceleration, angularAcceleration);
        if(static_cast<int>(pointIndex) < result.platformPointCount){
            result.platformPointsGlobalMm[pointIndex] = point.position;
        }
        for(int axis = 0; axis < 3; ++axis){
            for(int face = 0; face < 2; ++face){
                const bool upper = face == 1;
                const double direction = upper ? 1.0 : -1.0;
                const double clearance = upper ?
                            config_.frameMaximumMm[axis] - point.position[axis] :
                            point.position[axis] - config_.frameMinimumMm[axis];
                const double outwardSpeed = std::max(0.0,
                                                      direction * point.velocity[axis]);
                const double outwardAcceleration =
                        direction * point.acceleration[axis];
                const double pureStoppingDistance = outwardSpeed *
                        brakingEquivalentSpeedMmPerSec /
                        (2.0 * safety.stoppingDecelerationMmPerSec2);
                const double triggerDistance = pureStoppingDistance +
                        safety.additionalSafetyMarginMm;

                if(clearance < result.minimumClearanceMm){
                    result.minimumClearanceMm = clearance;
                    if(result.action == PhysicalWorkspaceAction::Safe){
                        assignFace(pointIndex, axis, upper, point, clearance,
                                   outwardSpeed, outwardAcceleration,
                                   pureStoppingDistance, triggerDistance);
                    }
                }

                if(clearance < 0.0 ||
                        clearance <= safety.emergencyLineMarginMm){
                    result.physicallyInside = result.physicallyInside && clearance >= 0.0;
                    if(result.action != PhysicalWorkspaceAction::EmergencyStop ||
                            clearance < bestEmergencyClearanceMm){
                        bestEmergencyClearanceMm = clearance;
                        assignFace(pointIndex, axis, upper, point, clearance,
                                   outwardSpeed, outwardAcceleration,
                                   pureStoppingDistance, triggerDistance);
                        result.action = PhysicalWorkspaceAction::EmergencyStop;
                        result.reason = clearance < 0.0 ?
                                    QStringLiteral("平台连接点%1越过机架%2，余量=%3 mm")
                                    .arg(pointIndex + 1)
                                    .arg(faceName(axis, upper))
                                    .arg(clearance, 0, 'f', 6) :
                                    QStringLiteral("平台连接点%1到达固定急停线%2，余量=%3 mm")
                                    .arg(pointIndex + 1)
                                    .arg(faceName(axis, upper))
                                    .arg(clearance, 0, 'f', 6);
                    }
                    continue;
                }

                const bool hasOutwardRisk =
                        outwardSpeed > safety.outwardVelocityToleranceMmPerSec ||
                        outwardAcceleration > safety.outwardAccelerationToleranceMmPerSec2;
                const bool clearlyDecelerating =
                        outwardAcceleration < -safety.outwardAccelerationToleranceMmPerSec2;
                const bool brakingDistanceInsufficient =
                        outwardSpeed > safety.outwardVelocityToleranceMmPerSec &&
                        clearance <= pureStoppingDistance;
                if((brakingDistanceInsufficient ||
                        (hasOutwardRisk && clearance <= triggerDistance &&
                         !clearlyDecelerating)) &&
                        result.action != PhysicalWorkspaceAction::EmergencyStop){
                    const double riskMm = clearance - triggerDistance;
                    if(result.action != PhysicalWorkspaceAction::ControlledStop ||
                            riskMm < bestControlledRiskMm){
                        bestControlledRiskMm = riskMm;
                        assignFace(pointIndex, axis, upper, point, clearance,
                                   outwardSpeed, outwardAcceleration,
                                   pureStoppingDistance, triggerDistance);
                        result.action = PhysicalWorkspaceAction::ControlledStop;
                        result.reason = QStringLiteral(
                                "平台连接点%1接近机架%2：余量=%3 mm，朝外速度=%4 mm/s，停车触发距离=%5 mm")
                                .arg(pointIndex + 1)
                                .arg(faceName(axis, upper))
                                .arg(clearance, 0, 'f', 6)
                                .arg(outwardSpeed, 0, 'f', 6)
                                .arg(triggerDistance, 0, 'f', 6);
                    }
                }
            }
        }
    }

    if(config_.orientationBoundsEnabled){
        for(int axis = 0; axis < 3; ++axis){
            const double angle = sample.poseMmRad[axis + 3];
            if(angle < config_.orientationMinimumRad[axis] ||
                    angle > config_.orientationMaximumRad[axis]){
                result.action = PhysicalWorkspaceAction::EmergencyStop;
                result.orientationInside = false;
                result.reason = QStringLiteral("平台姿态第%1维越过机器模板物理限制")
                        .arg(axis);
                return result;
            }
        }
    }

    if(result.action == PhysicalWorkspaceAction::Safe){
        result.reason = QStringLiteral("物理工作空间安全");
    }
    return result;
}

std::array<double, 6> PhysicalWorkspaceBoundary::solverLowerBounds() const
{
    if(!configured_){
        return {};
    }
    return {{config_.frameMinimumMm[0], config_.frameMinimumMm[1],
             config_.frameMinimumMm[2],
             config_.orientationBoundsEnabled ? config_.orientationMinimumRad[0] : -3.14,
             config_.orientationBoundsEnabled ? config_.orientationMinimumRad[1] : -3.14,
             config_.orientationBoundsEnabled ? config_.orientationMinimumRad[2] : -3.14}};
}

std::array<double, 6> PhysicalWorkspaceBoundary::solverUpperBounds() const
{
    if(!configured_){
        return {};
    }
    return {{config_.frameMaximumMm[0], config_.frameMaximumMm[1],
             config_.frameMaximumMm[2],
             config_.orientationBoundsEnabled ? config_.orientationMaximumRad[0] : 3.14,
             config_.orientationBoundsEnabled ? config_.orientationMaximumRad[1] : 3.14,
             config_.orientationBoundsEnabled ? config_.orientationMaximumRad[2] : 3.14}};
}

bool runPhysicalWorkspaceBoundarySelfChecks(
        const PhysicalWorkspaceBoundaryConfig& config,
        QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message){
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    const auto requireAction = [&fail](
            const PhysicalWorkspaceBoundaryResult& result,
            PhysicalWorkspaceAction expected,
            const QString& caseName){
        if(result.action != expected){
            return fail(QStringLiteral("%1：动作不符，实际原因=%2")
                        .arg(caseName, result.reason));
        }
        return true;
    };

    PhysicalWorkspaceBoundary boundary;
    QString error;
    if(!boundary.configure(config, &error)){
        return fail(QStringLiteral("统一物理工作空间配置无效：%1").arg(error));
    }

    std::array<double, 6> centerPose{};
    for(int axis = 0; axis < 3; ++axis){
        centerPose[axis] = 0.5 *
                (config.frameMinimumMm[axis] + config.frameMaximumMm[axis]);
    }
    const PhysicalWorkspaceBoundaryResult centerResult =
            boundary.evaluatePose(centerPose);
    if(!requireAction(centerResult, PhysicalWorkspaceAction::Safe,
                      QStringLiteral("机架中心")) ||
            !centerResult.physicallyInside || !centerResult.orientationInside){
        return fail(QStringLiteral("机架中心的八连接点或姿态未处于物理边界内"));
    }

    Vector3 localMinimum{{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    }};
    Vector3 localMaximum{{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()
    }};
    for(const Vector3& point : config.platformPointsLocalMm){
        for(int axis = 0; axis < 3; ++axis){
            localMinimum[axis] = std::min(localMinimum[axis], point[axis]);
            localMaximum[axis] = std::max(localMaximum[axis], point[axis]);
        }
    }

    // Zero-attitude acceptance matrix: every frame face is checked just
    // inside, exactly on the face, and just outside. Touching the physical
    // frame is an emergency condition even though its signed clearance is 0.
    constexpr double kBoundaryProbeMm = 1.0e-3;
    for(int axis = 0; axis < 3; ++axis){
        for(int face = 0; face < 2; ++face){
            const bool upper = face == 1;
            const double touchingCenter = upper ?
                        config.frameMaximumMm[axis] - localMaximum[axis] :
                        config.frameMinimumMm[axis] - localMinimum[axis];
            const double inwardDirection = upper ? -1.0 : 1.0;
            const QString casePrefix = QStringLiteral("%1%2边界")
                    .arg(QString::fromLatin1(axis == 0 ? "X" : axis == 1 ? "Y" : "Z"),
                         upper ? QStringLiteral("上") : QStringLiteral("下"));

            std::array<double, 6> pose = centerPose;
            pose[axis] = touchingCenter + inwardDirection * kBoundaryProbeMm;
            const PhysicalWorkspaceBoundaryResult inside = boundary.evaluatePose(pose);
            if(!requireAction(inside, PhysicalWorkspaceAction::Safe,
                              casePrefix + QStringLiteral("内侧")) ||
                    !inside.physicallyInside){
                return fail(casePrefix + QStringLiteral("内侧点未被判为安全"));
            }

            pose[axis] = touchingCenter;
            const PhysicalWorkspaceBoundaryResult touching = boundary.evaluatePose(pose);
            if(!requireAction(touching, PhysicalWorkspaceAction::EmergencyStop,
                              casePrefix + QStringLiteral("触边")) ||
                    !touching.physicallyInside || touching.limitingAxis != axis ||
                    touching.limitingUpperFace != upper){
                return fail(casePrefix + QStringLiteral("触边判定或限制面识别错误"));
            }

            pose[axis] = touchingCenter - inwardDirection * kBoundaryProbeMm;
            const PhysicalWorkspaceBoundaryResult outside = boundary.evaluatePose(pose);
            if(!requireAction(outside, PhysicalWorkspaceAction::EmergencyStop,
                              casePrefix + QStringLiteral("越界")) ||
                    outside.physicallyInside || outside.limitingAxis != axis ||
                    outside.limitingUpperFace != upper){
                return fail(casePrefix + QStringLiteral("越界判定或限制面识别错误"));
            }
        }
    }

    // Verify that platform points are transformed by the pose, rather than
    // incorrectly treating their local coordinates as global coordinates.
    constexpr double kPi = 3.14159265358979323846;
    std::array<double, 6> yawPose = centerPose;
    double yaw = 0.5 * kPi;
    if(config.orientationBoundsEnabled &&
            (yaw < config.orientationMinimumRad[2] ||
             yaw > config.orientationMaximumRad[2])){
        yaw = 0.5 * (config.orientationMinimumRad[2] +
                     config.orientationMaximumRad[2]);
        if(std::abs(yaw) < 1.0e-6){
            yaw = 0.5 * config.orientationMaximumRad[2];
        }
    }
    yawPose[5] = yaw;
    const PhysicalWorkspaceBoundaryResult yawResult = boundary.evaluatePose(yawPose);
    if(!requireAction(yawResult, PhysicalWorkspaceAction::Safe,
                      QStringLiteral("非零姿态坐标变换"))){
        return false;
    }
    const int storedPointCount = std::min(
                static_cast<int>(config.platformPointsLocalMm.size()),
                kPhysicalWorkspaceMaximumPlatformPoints);
    if(yawResult.platformPointCount != storedPointCount){
        return fail(QStringLiteral("非零姿态自检返回的连接点数量错误"));
    }
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    for(int index = 0; index < storedPointCount; ++index){
        const Vector3& local = config.platformPointsLocalMm[index];
        const Vector3 expected{{
            centerPose[0] + cosine * local[0] - sine * local[1],
            centerPose[1] + sine * local[0] + cosine * local[1],
            centerPose[2] + local[2]
        }};
        for(int axis = 0; axis < 3; ++axis){
            if(std::abs(yawResult.platformPointsGlobalMm[index][axis] -
                        expected[axis]) > 1.0e-9){
                return fail(QStringLiteral("非零姿态下连接点%1第%2维坐标变换错误")
                            .arg(index + 1).arg(axis));
            }
        }
    }

    if(config.orientationBoundsEnabled){
        for(int axis = 0; axis < 3; ++axis){
            std::array<double, 6> pose = centerPose;
            pose[axis + 3] = config.orientationMaximumRad[axis] + 1.0e-6;
            const PhysicalWorkspaceBoundaryResult outside = boundary.evaluatePose(pose);
            if(outside.action != PhysicalWorkspaceAction::EmergencyStop ||
                    outside.orientationInside){
                return fail(QStringLiteral("姿态第%1维越界未被检出").arg(axis));
            }
        }
    }

    DynamicWorkspaceSafetyConfig safety;
    safety.stoppingDecelerationMmPerSec2 = 100.0;
    safety.additionalSafetyMarginMm = 60.0;
    safety.emergencyLineMarginMm = 10.0;
    const double upperXTouchingCenter =
            config.frameMaximumMm[0] - localMaximum[0];
    const auto upperXSample = [&](double clearanceMm,
                                  double velocityMmPerSec,
                                  double accelerationMmPerSec2){
        PhysicalWorkspaceMotionSample sample;
        sample.poseMmRad = centerPose;
        sample.poseMmRad[0] = upperXTouchingCenter - clearanceMm;
        sample.twistMmRadPerSec[0] = velocityMmPerSec;
        sample.accelerationMmRadPerSec2[0] = accelerationMmPerSec2;
        return sample;
    };

    // v=100 mm/s and a=100 mm/s^2 give a pure stopping distance of 50 mm;
    // adding the 60 mm margin produces a 110 mm trigger distance.
    const PhysicalWorkspaceBoundaryResult approaching =
            boundary.evaluateMotion(upperXSample(100.0, 100.0, 0.0), safety);
    if(!requireAction(approaching, PhysicalWorkspaceAction::ControlledStop,
                      QStringLiteral("动态停车触发")) ||
            std::abs(approaching.pureStoppingDistanceMm - 50.0) > 1.0e-9 ||
            std::abs(approaching.triggerDistanceMm - 110.0) > 1.0e-9){
        return fail(QStringLiteral("动态停车距离计算错误"));
    }

    // X/Y 均为 100 mm/s 时，公共缩放制动下 X 法向滑行量是
    // 100*sqrt(2)*100/(2*100)=70.710678... mm，而不是只看 X 得到的 50 mm。
    PhysicalWorkspaceMotionSample diagonal =
            upperXSample(120.0, 100.0, 0.0);
    diagonal.twistMmRadPerSec[1] = 100.0;
    const PhysicalWorkspaceBoundaryResult diagonalApproaching =
            boundary.evaluateMotion(diagonal, safety);
    const double expectedDiagonalStoppingDistance = 50.0 * std::sqrt(2.0);
    if(!requireAction(diagonalApproaching,
                      PhysicalWorkspaceAction::ControlledStop,
                      QStringLiteral("斜向协同停车触发")) ||
            std::abs(diagonalApproaching.pureStoppingDistanceMm -
                     expectedDiagonalStoppingDistance) > 1.0e-9){
        return fail(QStringLiteral("斜向协同停车距离计算错误"));
    }

    const PhysicalWorkspaceBoundaryResult alreadyDecelerating =
            boundary.evaluateMotion(upperXSample(100.0, 100.0, -1.0), safety);
    if(!requireAction(alreadyDecelerating, PhysicalWorkspaceAction::Safe,
                      QStringLiteral("已主动减速"))){
        return false;
    }

    const PhysicalWorkspaceBoundaryResult insufficientDistance =
            boundary.evaluateMotion(upperXSample(40.0, 100.0, -1.0), safety);
    if(!requireAction(insufficientDistance,
                      PhysicalWorkspaceAction::ControlledStop,
                      QStringLiteral("已减速但纯制动距离不足"))){
        return false;
    }

    const PhysicalWorkspaceBoundaryResult movingInward =
            boundary.evaluateMotion(upperXSample(100.0, -100.0, 0.0), safety);
    if(!requireAction(movingInward, PhysicalWorkspaceAction::Safe,
                      QStringLiteral("靠近上边界但向内运动"))){
        return false;
    }

    const PhysicalWorkspaceBoundaryResult emergencyLine =
            boundary.evaluateMotion(upperXSample(9.999, -100.0, 0.0), safety);
    if(!requireAction(emergencyLine, PhysicalWorkspaceAction::EmergencyStop,
                      QStringLiteral("固定急停线"))){
        return false;
    }

    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}
