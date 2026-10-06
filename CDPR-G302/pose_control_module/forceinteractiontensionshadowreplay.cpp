#include "forceinteractiontensionshadowreplay.h"

#include "forceinteractionkinematicloganalyzer.h"
#include "forceinteractiontensionshadowvalidator.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

std::vector<double> values(const QJsonArray& array)
{
    std::vector<double> result;
    result.reserve(array.size());
    for(const QJsonValue& value : array) result.push_back(value.toDouble());
    return result;
}

std::vector<std::vector<double>> matrix(const QJsonArray& array)
{
    std::vector<std::vector<double>> result;
    result.reserve(array.size());
    for(const QJsonValue& row : array) result.push_back(values(row.toArray()));
    return result;
}

bool fail(ForceInteractionTensionShadowReplayResult& result,
          const QString& message)
{
    result.summary = QStringLiteral("M1离线影子验证失败：%1").arg(message);
    return false;
}

template<typename Derived>
bool readEigenArray(const QJsonArray& array,
                    Eigen::MatrixBase<Derived>& destination)
{
    if(array.size() != destination.size()) return false;
    int index = 0;
    for(Eigen::Index row = 0; row < destination.rows(); ++row){
        for(Eigen::Index column = 0; column < destination.cols(); ++column){
            const double value = array[index++].toDouble(
                        std::numeric_limits<double>::quiet_NaN());
            if(!std::isfinite(value)) return false;
            destination(row, column) = value;
        }
    }
    return true;
}

} // namespace

ForceInteractionTensionShadowReplayResult
ForceInteractionTensionShadowReplay::run(const QString& replayConfigPath)
{
    ForceInteractionTensionShadowReplayResult result;
    QString m0Error;
    if(!ForceInteractionTensionShadowValidator::runSelfChecks(&m0Error)){
        fail(result, QStringLiteral("M0纯算法自检未通过：%1").arg(m0Error));
        return result;
    }
    QFile file(replayConfigPath);
    if(!file.open(QIODevice::ReadOnly)){
        fail(result, QStringLiteral("无法读取%1：%2")
             .arg(QDir::toNativeSeparators(replayConfigPath), file.errorString()));
        return result;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(
                file.readAll(), &parseError);
    if(parseError.error != QJsonParseError::NoError || !document.isObject()){
        fail(result, QStringLiteral("回放JSON格式错误：%1")
             .arg(parseError.errorString()));
        return result;
    }
    const QJsonObject root = document.object();
    const QFileInfo configInfo(replayConfigPath);
    ForceInteractionKinematicLogAnalysisRequest request;
    request.csvPath = configInfo.dir().filePath(root.value("csv_file").toString());
    const QJsonObject geometry = root.value("geometry").toObject();
    request.kinematics.anchorCableCoordinate =
            matrix(geometry.value("base_anchor_global_mm").toArray());
    const QJsonArray ends =
            geometry.value("platform_attachment_local_mm_by_end").toArray();
    for(const QJsonValue& end : ends)
        request.kinematics.endCableContactPos.push_back(matrix(end.toArray()));
    request.kinematics.pulleyRadiusMm =
            geometry.value("pulley_radius_mm").toDouble();
    request.initialPoseMmRad =
            values(geometry.value("initial_pose_mm_rad").toArray());
    request.referenceCableLengthMm =
            values(geometry.value("reference_cable_length_mm").toArray());
    const QJsonArray axes = root.value("axes").toArray();
    if(axes.size() != 8){
        fail(result, QStringLiteral("回放配置不是完整八轴"));
        return result;
    }
    for(int axis = 0; axis < 8; ++axis){
        const QJsonObject item = axes[axis].toObject();
        request.motorUnitPerRadian[axis] =
                item.value("motor_unit_per_radian").toDouble();
        request.actualStartSafetyRelativePosition[axis] =
                item.value("actual_start_safety_relative_position").toDouble();
        request.kinematics.cableMotorScaleRadPerMm.push_back(
                    item.value("fallback_motor_rad_per_mm").toDouble());
        const QJsonObject winch = item.value("winch").toObject();
        WinchCompensation::AxisConfig winchConfig;
        winchConfig.enabled = winch.value("enabled").toBool();
        winchConfig.drumRadiusMm = winch.value("radius_mm").toDouble();
        winchConfig.pitchMmPerRev = winch.value("pitch_mm_per_rev").toDouble();
        winchConfig.projectionMm = winch.value("projection_mm").toDouble();
        winchConfig.initialAxialOffsetValid =
                winch.value("initial_axial_offset_valid").toBool();
        winchConfig.initialAxialOffsetMm =
                winch.value("initial_axial_offset_mm").toDouble();
        request.kinematics.winchConfig.push_back(winchConfig);
    }

    const QJsonObject workspace = root.value("workspace").toObject();
    const auto minimum = values(workspace.value("frame_minimum_mm").toArray());
    const auto maximum = values(workspace.value("frame_maximum_mm").toArray());
    if(minimum.size() != 3 || maximum.size() != 3){
        fail(result, QStringLiteral("工作空间边界缺失"));
        return result;
    }
    std::copy_n(minimum.begin(), 3, request.physicalWorkspace.frameMinimumMm.begin());
    std::copy_n(maximum.begin(), 3, request.physicalWorkspace.frameMaximumMm.begin());
    const auto points = matrix(
                workspace.value("platform_boundary_points_local_mm").toArray());
    for(const auto& point : points){
        if(point.size() >= 3)
            request.physicalWorkspace.platformPointsLocalMm.push_back(
                        {{point[0], point[1], point[2]}});
    }

    // Old v1 sidecars predate the rigid-body fields.  Translation-only replay
    // uses the G302/Lite value that generated those runs; rotational inertia is
    // retained only to keep the 6x6 matrix positive definite and is not excited.
    ForceInteractionRigidBodyConfig rigidBody;
    rigidBody.massKg = root.value("platform_mass_kg").toDouble(9.183);
    rigidBody.inertiaKgM2 = {{
        0.313530372, 0.0, 0.0,
        0.0, 0.313459352, 0.0,
        0.0, 0.0, 0.368548900
    }};
    const auto replayInertia = values(
                root.value("platform_inertia_kg_m2").toArray());
    if(replayInertia.size() == rigidBody.inertiaKgM2.size()){
        std::copy(replayInertia.begin(), replayInertia.end(),
                  rigidBody.inertiaKgM2.begin());
    }
    const double tensionMinimum =
            root.value("minimum_cable_tension_n").toDouble(10.0);
    const double tensionMaximum =
            root.value("maximum_cable_tension_n").toDouble(400.0);
    request.tensionShadow = makeDefaultG302TranslationShadowConfig(
                rigidBody, request.kinematics,
                tensionMinimum, tensionMaximum);
    if(root.value("tension_shadow_enabled").toBool(false)){
        const QJsonObject shadow = root.value("tension_shadow").toObject();
        request.tensionShadow.translationOnly =
                shadow.value("translation_only").toBool(true);
        request.tensionShadow.outerPeriodS =
                shadow.value("outer_period_s").toDouble(
                    request.tensionShadow.outerPeriodS);
        request.tensionShadow.gravityMPerSec2 =
                shadow.value("gravity_m_s2").toDouble(
                    request.tensionShadow.gravityMPerSec2);
        readEigenArray(shadow.value("effective_radius_m_per_rad").toArray(),
                       request.tensionShadow.effectiveRadiusMPerRad);
        const QJsonObject controller = shadow.value("controller").toObject();
        readEigenArray(controller.value("virtual_mass_kg").toArray(),
                       request.tensionShadow.controller.virtualMass);
        readEigenArray(controller.value("virtual_body_inertia_kg_m2").toArray(),
                       request.tensionShadow.controller.virtualBodyInertia);
        request.tensionShadow.controller.dampingRatio =
                controller.value("damping_ratio").toDouble(
                    request.tensionShadow.controller.dampingRatio);
        readEigenArray(controller.value("stiffness").toArray(),
                       request.tensionShadow.controller.stiffness);
        readEigenArray(controller.value("integral_gain").toArray(),
                       request.tensionShadow.controller.integralGain);
        readEigenArray(controller.value("integral_limit").toArray(),
                       request.tensionShadow.controller.integralLimit);
        request.tensionShadow.controller.integralEnabled =
                controller.value("integral_enabled").toBool(
                    request.tensionShadow.controller.integralEnabled);
        const QJsonObject allocator = shadow.value("allocator").toObject();
        readEigenArray(allocator.value("tension_minimum_n").toArray(),
                       request.tensionShadow.allocator.tensionMinimum);
        readEigenArray(allocator.value("tension_maximum_n").toArray(),
                       request.tensionShadow.allocator.tensionMaximum);
        readEigenArray(allocator.value("tension_bias_n").toArray(),
                       request.tensionShadow.allocator.tensionBias);
        readEigenArray(allocator.value("tension_weight").toArray(),
                       request.tensionShadow.allocator.tensionWeight);
        readEigenArray(allocator.value("hardware_torque_minimum_nm").toArray(),
                       request.tensionShadow.allocator.hardwareTorqueMinimum);
        readEigenArray(allocator.value("hardware_torque_maximum_nm").toArray(),
                       request.tensionShadow.allocator.hardwareTorqueMaximum);
        readEigenArray(allocator.value("hardware_direction").toArray(),
                       request.tensionShadow.allocator.hardwareDirection);
        request.tensionShadow.allocator.torqueSlewEnabled =
                allocator.value("torque_slew_enabled").toBool(
                    request.tensionShadow.allocator.torqueSlewEnabled);
        readEigenArray(
                    allocator.value("hardware_torque_slew_rate_nm_s").toArray(),
                    request.tensionShadow.allocator.hardwareTorqueSlewRate);
    }
    request.tensionShadowEnabled = true;

    const ForceInteractionKinematicLogAnalysisResult analysis =
            ForceInteractionKinematicLogAnalyzer::analyze(request);
    result.completed = analysis.completed && analysis.shadowEvaluatedRows > 0 &&
            analysis.shadowValidRows == analysis.shadowEvaluatedRows;
    result.summary = QStringLiteral("M0纯算法自检通过。\n%1")
            .arg(analysis.summary);
    if(!root.contains("platform_mass_kg")){
        result.summary += QStringLiteral(
                    "\n旧版回放配置不含质量字段：本次仅平动复算采用该批D1的G302默认质量9.183 kg；新运行将使用准备时冻结值。");
    }
    return result;
}
