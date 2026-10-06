#include "forceinteractionreplayexporter.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

QJsonArray numberArray(const std::vector<double>& values)
{
    QJsonArray result;
    for(double value : values){
        result.append(std::isfinite(value) ? QJsonValue(value) : QJsonValue());
    }
    return result;
}

template<typename T, std::size_t N>
QJsonArray numberArray(const std::array<T, N>& values)
{
    QJsonArray result;
    for(const T& value : values){
        result.append(static_cast<double>(value));
    }
    return result;
}

QJsonArray matrixArray(const std::vector<std::vector<double>>& matrix)
{
    QJsonArray result;
    for(const auto& row : matrix){
        result.append(numberArray(row));
    }
    return result;
}

QJsonArray nestedPointArray(
        const std::vector<std::vector<std::vector<double>>>& pointsByEnd)
{
    QJsonArray result;
    for(const auto& endPoints : pointsByEnd){
        result.append(matrixArray(endPoints));
    }
    return result;
}

QJsonObject workspaceObject(const PhysicalWorkspaceBoundaryConfig& workspace,
                            const DynamicWorkspaceSafetyConfig& safety)
{
    QJsonObject result;
    result.insert(QStringLiteral("frame_minimum_mm"),
                  numberArray(workspace.frameMinimumMm));
    result.insert(QStringLiteral("frame_maximum_mm"),
                  numberArray(workspace.frameMaximumMm));
    QJsonArray platformPoints;
    for(const auto& point : workspace.platformPointsLocalMm){
        platformPoints.append(numberArray(point));
    }
    result.insert(QStringLiteral("platform_boundary_points_local_mm"),
                  platformPoints);
    result.insert(QStringLiteral("orientation_bounds_enabled"),
                  workspace.orientationBoundsEnabled);
    result.insert(QStringLiteral("orientation_minimum_rad"),
                  numberArray(workspace.orientationMinimumRad));
    result.insert(QStringLiteral("orientation_maximum_rad"),
                  numberArray(workspace.orientationMaximumRad));
    result.insert(QStringLiteral("stopping_deceleration_mm_s2"),
                  safety.stoppingDecelerationMmPerSec2);
    result.insert(QStringLiteral("additional_safety_margin_mm"),
                  safety.additionalSafetyMarginMm);
    result.insert(QStringLiteral("emergency_line_margin_mm"),
                  safety.emergencyLineMarginMm);
    return result;
}

template<typename Derived>
QJsonArray eigenArray(const Eigen::MatrixBase<Derived>& values)
{
    QJsonArray result;
    for(Eigen::Index row = 0; row < values.rows(); ++row){
        for(Eigen::Index column = 0; column < values.cols(); ++column){
            result.append(values(row, column));
        }
    }
    return result;
}

QJsonObject tensionShadowObject(
        const ForceInteractionTensionShadowConfig& config)
{
    QJsonObject result;
    result.insert(QStringLiteral("translation_only"), config.translationOnly);
    result.insert(QStringLiteral("outer_period_s"), config.outerPeriodS);
    result.insert(QStringLiteral("gravity_m_s2"), config.gravityMPerSec2);
    result.insert(QStringLiteral("effective_radius_m_per_rad"),
                  eigenArray(config.effectiveRadiusMPerRad));
    QJsonObject controller;
    controller.insert(QStringLiteral("virtual_mass_kg"),
                      eigenArray(config.controller.virtualMass));
    controller.insert(QStringLiteral("virtual_body_inertia_kg_m2"),
                      eigenArray(config.controller.virtualBodyInertia));
    controller.insert(QStringLiteral("damping_ratio"),
                      config.controller.dampingRatio);
    controller.insert(QStringLiteral("stiffness"),
                      eigenArray(config.controller.stiffness));
    controller.insert(QStringLiteral("integral_gain"),
                      eigenArray(config.controller.integralGain));
    controller.insert(QStringLiteral("integral_limit"),
                      eigenArray(config.controller.integralLimit));
    controller.insert(QStringLiteral("integral_enabled"),
                      config.controller.integralEnabled);
    result.insert(QStringLiteral("controller"), controller);
    QJsonObject allocator;
    allocator.insert(QStringLiteral("tension_minimum_n"),
                     eigenArray(config.allocator.tensionMinimum));
    allocator.insert(QStringLiteral("tension_maximum_n"),
                     eigenArray(config.allocator.tensionMaximum));
    allocator.insert(QStringLiteral("tension_bias_n"),
                     eigenArray(config.allocator.tensionBias));
    allocator.insert(QStringLiteral("tension_weight"),
                     eigenArray(config.allocator.tensionWeight));
    allocator.insert(QStringLiteral("hardware_torque_minimum_nm"),
                     eigenArray(config.allocator.hardwareTorqueMinimum));
    allocator.insert(QStringLiteral("hardware_torque_maximum_nm"),
                     eigenArray(config.allocator.hardwareTorqueMaximum));
    allocator.insert(QStringLiteral("hardware_direction"),
                     eigenArray(config.allocator.hardwareDirection));
    allocator.insert(QStringLiteral("torque_slew_enabled"),
                     config.allocator.torqueSlewEnabled);
    allocator.insert(QStringLiteral("hardware_torque_slew_rate_nm_s"),
                     eigenArray(config.allocator.hardwareTorqueSlewRate));
    result.insert(QStringLiteral("allocator"), allocator);
    return result;
}

} // namespace

QString ForceInteractionReplayExporter::sidecarPathForCsv(const QString& csvPath)
{
    const QFileInfo csvInfo(csvPath);
    return csvInfo.dir().filePath(
                csvInfo.completeBaseName() + QStringLiteral("_replay_config.json"));
}

bool ForceInteractionReplayExporter::writeJson(
        const ForceInteractionReplayExportContext& context,
        QString* outputPath,
        QString* errorMessage)
{
    const QFileInfo csvInfo(context.csvPath);
    if(context.csvPath.trimmed().isEmpty() || !csvInfo.exists()){
        if(errorMessage){
            *errorMessage = QStringLiteral("原始运行CSV不存在：%1")
                    .arg(context.csvPath);
        }
        return false;
    }
    if(context.kinematics.anchorCableCoordinate.size() !=
            kOnlineVelocityAxisCount ||
            context.kinematics.cableMotorScaleRadPerMm.size() !=
            kOnlineVelocityAxisCount ||
            context.referenceCableLengthMm.size() !=
            kOnlineVelocityAxisCount ||
            context.initialPoseMmRad.size() < 6){
        if(errorMessage){
            *errorMessage = QStringLiteral("冻结运动学参数不是完整八轴数据");
        }
        return false;
    }

    QJsonObject root;
    root.insert(QStringLiteral("schema"),
                QStringLiteral("cdpr_g302_matlab_replay_config_v1"));
    root.insert(QStringLiteral("created_at"),
                QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    root.insert(QStringLiteral("run_id"), csvInfo.completeBaseName());
    root.insert(QStringLiteral("csv_file"), csvInfo.fileName());
    root.insert(QStringLiteral("csv_schema"),
                QStringLiteral("force_interaction_run_v13"));
    root.insert(QStringLiteral("stage"), context.stageName);
    root.insert(QStringLiteral("wrench_source"), context.wrenchSourceName);
    root.insert(QStringLiteral("kinematic_template"),
                context.machineTemplateName);
    root.insert(QStringLiteral("actuator_template"),
                context.actuatorTemplateName);
    root.insert(QStringLiteral("mechanical_mode"), context.mechanicalModeName);
    root.insert(QStringLiteral("mocap_mode"), context.mocapModeName);
    root.insert(QStringLiteral("tension_protection_enabled"),
                context.globalTensionSafetyEnabled);
    root.insert(QStringLiteral("minimum_cable_tension_n"),
                context.globalMinimumCableTensionN);
    double strictestMaximumTensionN =
            std::numeric_limits<double>::infinity();
    for(double value : context.globalMaximumCableTensionN){
        strictestMaximumTensionN = std::min(strictestMaximumTensionN, value);
    }
    root.insert(QStringLiteral("maximum_cable_tension_n"),
                std::isfinite(strictestMaximumTensionN) ?
                    strictestMaximumTensionN : 0.0);
    root.insert(QStringLiteral("maximum_cable_tension_n_by_sensor_channel"),
                numberArray(context.globalMaximumCableTensionN));
    root.insert(QStringLiteral("control_period_us"), context.controlPeriodUs);
    root.insert(QStringLiteral("trace_period_us"), context.tracePeriodUs);
    root.insert(QStringLiteral("translation_only"), context.translationOnly);
    root.insert(QStringLiteral("platform_mass_kg"), context.rigidBody.massKg);
    root.insert(QStringLiteral("platform_inertia_kg_m2"),
                numberArray(context.rigidBody.inertiaKgM2));
    root.insert(QStringLiteral("tension_shadow_enabled"),
                context.tensionShadowEnabled);
    if(context.tensionShadowEnabled){
        root.insert(QStringLiteral("tension_shadow"),
                    tensionShadowObject(context.tensionShadow));
    }

    QJsonObject conventions;
    conventions.insert(QStringLiteral("length_unit"), QStringLiteral("mm"));
    conventions.insert(QStringLiteral("angle_unit"), QStringLiteral("rad"));
    conventions.insert(QStringLiteral("pose_order"),
                       QStringLiteral("[x,y,z,rx,ry,rz]"));
    conventions.insert(QStringLiteral("rotation"),
                       QStringLiteral("Rz(rz)*Ry(ry)*Rx(rx)"));
    conventions.insert(QStringLiteral("cable_order"),
                       QStringLiteral("logical cable index 0..7"));
    conventions.insert(QStringLiteral("measured_length_relation"),
                       QStringLiteral("L=L0-platformDelta(theta)"));
    root.insert(QStringLiteral("conventions"), conventions);

    QJsonObject geometry;
    geometry.insert(QStringLiteral("base_anchor_global_mm"),
                    matrixArray(context.kinematics.anchorCableCoordinate));
    geometry.insert(QStringLiteral("platform_attachment_local_mm_by_end"),
                    nestedPointArray(context.kinematics.endCableContactPos));
    geometry.insert(QStringLiteral("pulley_radius_mm"),
                    context.kinematics.pulleyRadiusMm);
    geometry.insert(QStringLiteral("winch_reference_pose_mm_rad_by_end"),
                    matrixArray(context.kinematics.winchReferencePose));
    geometry.insert(QStringLiteral("initial_pose_mm_rad"),
                    numberArray(context.initialPoseMmRad));
    geometry.insert(QStringLiteral("reference_cable_length_mm"),
                    numberArray(context.referenceCableLengthMm));
    root.insert(QStringLiteral("geometry"), geometry);

    QJsonArray axes;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        QJsonObject item;
        item.insert(QStringLiteral("logical_cable"), axis);
        item.insert(QStringLiteral("hardware_axis"), context.hardwareAxis[axis]);
        item.insert(QStringLiteral("slave_id"), context.slaveId[axis]);
        item.insert(QStringLiteral("pulse_per_unit"), context.pulsePerUnit[axis]);
        item.insert(QStringLiteral("hardware_direction_sign"),
                    context.hardwareDirectionSign[axis]);
        item.insert(QStringLiteral("motor_unit_per_radian"),
                    context.motorUnitPerRadian[axis]);
        item.insert(QStringLiteral("fallback_motor_rad_per_mm"),
                    context.kinematics.cableMotorScaleRadPerMm.at(axis));
        item.insert(QStringLiteral("trace_delay_valid"),
                    context.traceDelayValid[axis]);
        item.insert(QStringLiteral("trace_delay_ms"),
                    context.traceDelayMs[axis]);
        item.insert(QStringLiteral("motor_safety_relative_minimum"),
                    context.motorSafetyRelativeMinimum[axis]);
        item.insert(QStringLiteral("motor_safety_relative_maximum"),
                    context.motorSafetyRelativeMaximum[axis]);
        item.insert(QStringLiteral("actual_start_position"),
                    context.actualStartPosition[axis]);
        item.insert(QStringLiteral("actual_start_safety_relative_position"),
                    context.actualStartSafetyRelativePosition[axis]);
        if(axis < static_cast<int>(context.kinematics.winchConfig.size())){
            const auto& winch = context.kinematics.winchConfig[axis];
            QJsonObject winchObject;
            winchObject.insert(QStringLiteral("enabled"), winch.enabled);
            winchObject.insert(QStringLiteral("radius_mm"), winch.drumRadiusMm);
            winchObject.insert(QStringLiteral("pitch_mm_per_rev"),
                               winch.pitchMmPerRev);
            winchObject.insert(QStringLiteral("projection_mm"),
                               winch.projectionMm);
            winchObject.insert(QStringLiteral("initial_axial_offset_valid"),
                               winch.initialAxialOffsetValid);
            winchObject.insert(QStringLiteral("initial_axial_offset_mm"),
                               winch.initialAxialOffsetMm);
            item.insert(QStringLiteral("winch"), winchObject);
        }
        axes.append(item);
    }
    root.insert(QStringLiteral("axes"), axes);
    root.insert(QStringLiteral("workspace"),
                workspaceObject(context.physicalWorkspace,
                                context.workspaceSafety));

    QJsonObject terminal;
    terminal.insert(QStringLiteral("state"), context.terminalState);
    terminal.insert(QStringLiteral("message"), context.terminalMessage);
    terminal.insert(QStringLiteral("safety_stop_reason"),
                    context.safetyStopReason);
    terminal.insert(QStringLiteral("experiment_valid"),
                    context.experimentValid);
    terminal.insert(QStringLiteral("command_count"),
                    static_cast<double>(context.commandCount));
    terminal.insert(QStringLiteral("written_record_count"),
                    static_cast<double>(context.writtenRecordCount));
    terminal.insert(QStringLiteral("dropped_record_count"),
                    static_cast<double>(context.droppedRecordCount));
    root.insert(QStringLiteral("terminal"), terminal);

    const QString jsonPath = sidecarPathForCsv(context.csvPath);
    QSaveFile file(jsonPath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法创建回放配置：%1")
                    .arg(file.errorString());
        }
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if(!file.commit()){
        if(errorMessage){
            *errorMessage = QStringLiteral("回放配置写盘失败：%1")
                    .arg(file.errorString());
        }
        return false;
    }
    if(outputPath){
        *outputPath = jsonPath;
    }
    if(errorMessage){
        errorMessage->clear();
    }
    return true;
}
