#include "forceinteractionreplayexporter.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include <cmath>

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
                QStringLiteral("force_interaction_run_v11"));
    root.insert(QStringLiteral("stage"), context.stageName);
    root.insert(QStringLiteral("wrench_source"), context.wrenchSourceName);
    root.insert(QStringLiteral("kinematic_template"),
                context.machineTemplateName);
    root.insert(QStringLiteral("actuator_template"),
                context.actuatorTemplateName);
    root.insert(QStringLiteral("control_period_us"), context.controlPeriodUs);
    root.insert(QStringLiteral("trace_period_us"), context.tracePeriodUs);
    root.insert(QStringLiteral("translation_only"), context.translationOnly);

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
