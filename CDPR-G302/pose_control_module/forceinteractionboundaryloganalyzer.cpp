#include "forceinteractionboundaryloganalyzer.h"

#include "forceinteractionrunrecorder.h"
#include "physicalworkspaceboundary.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QTextStream>

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr double kNumericTolerance = 1.0e-5;

QString valueAfterEquals(const QString& line, const QString& key)
{
    const QString prefix = QStringLiteral("# ") + key + QLatin1Char('=');
    return line.startsWith(prefix) ? line.mid(prefix.size()).trimmed() : QString{};
}

bool parseDouble(const QString& text, double& value)
{
    bool ok = false;
    value = text.trimmed().toDouble(&ok);
    return ok && std::isfinite(value);
}

bool parseInt(const QString& text, int& value)
{
    bool ok = false;
    value = text.trimmed().toInt(&ok);
    return ok;
}

bool parseUnsigned(const QString& text, quint64& value)
{
    bool ok = false;
    value = text.trimmed().toULongLong(&ok);
    return ok;
}

bool parseVector3(const QString& text, std::array<double, 3>& values)
{
    const QStringList parts = text.split(QLatin1Char(';'));
    if(parts.size() != 3){
        return false;
    }
    for(int index = 0; index < 3; ++index){
        if(!parseDouble(parts[index], values[static_cast<size_t>(index)])){
            return false;
        }
    }
    return true;
}

int requiredColumn(const QHash<QString, int>& columns, const QString& name)
{
    const auto iterator = columns.constFind(name);
    return iterator == columns.cend() ? -1 : iterator.value();
}

bool readDoubleColumn(const QStringList& row, int column, double& value)
{
    return column >= 0 && column < row.size() && parseDouble(row[column], value);
}

bool readIntegerColumn(const QStringList& row, int column, qint64& value)
{
    if(column < 0 || column >= row.size()){
        return false;
    }
    bool ok = false;
    value = row[column].trimmed().toLongLong(&ok);
    return ok;
}

bool nearlyEqual(double left, double right, double* difference = nullptr)
{
    const double delta = std::abs(left - right);
    if(difference){
        *difference = delta;
    }
    const double scale = std::max({1.0, std::abs(left), std::abs(right)});
    return delta <= kNumericTolerance + 1.0e-9 * scale;
}

bool writeReport(const ForceInteractionBoundaryLogAnalysisResult& result,
                 QString* errorMessage)
{
    QJsonObject root;
    root.insert(QStringLiteral("schema"),
                QStringLiteral("force_interaction_boundary_analysis_v1"));
    root.insert(QStringLiteral("generated_at"),
                QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    root.insert(QStringLiteral("csv_path"), result.csvPath);
    root.insert(QStringLiteral("passed"), result.passed);
    root.insert(QStringLiteral("data_rows"), static_cast<double>(result.dataRows));
    root.insert(QStringLiteral("replayed_rows"), static_cast<double>(result.replayedRows));
    root.insert(QStringLiteral("malformed_rows"), static_cast<double>(result.malformedRows));
    root.insert(QStringLiteral("mismatch_rows"), static_cast<double>(result.mismatchRows));
    root.insert(QStringLiteral("action_mismatch_rows"),
                static_cast<double>(result.actionMismatchRows));
    root.insert(QStringLiteral("point_mismatch_rows"),
                static_cast<double>(result.pointMismatchRows));
    root.insert(QStringLiteral("trace_invalid_rows"),
                static_cast<double>(result.traceInvalidRows));
    root.insert(QStringLiteral("trace_non_monotonic_rows"),
                static_cast<double>(result.traceNonMonotonicRows));
    root.insert(QStringLiteral("interaction_rows"),
                static_cast<double>(result.interactionRows));
    root.insert(QStringLiteral("braking_rows"),
                static_cast<double>(result.brakingRows));
    root.insert(QStringLiteral("recorder_accepted_rows"),
                static_cast<double>(result.recorderAcceptedRows));
    root.insert(QStringLiteral("recorder_written_rows"),
                static_cast<double>(result.recorderWrittenRows));
    root.insert(QStringLiteral("recorder_dropped_rows"),
                static_cast<double>(result.recorderDroppedRows));
    root.insert(QStringLiteral("maximum_clearance_difference_mm"),
                result.maximumClearanceDifferenceMm);
    root.insert(QStringLiteral("maximum_trigger_distance_difference_mm"),
                result.maximumTriggerDistanceDifferenceMm);
    root.insert(QStringLiteral("maximum_point_difference_mm"),
                result.maximumPointDifferenceMm);
    root.insert(QStringLiteral("error"), result.errorMessage);
    root.insert(QStringLiteral("summary"), result.summary);

    QSaveFile file(result.reportPath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法创建边界复算报告：%1")
                    .arg(file.errorString());
        }
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if(!file.commit()){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法提交边界复算报告：%1")
                    .arg(file.errorString());
        }
        return false;
    }
    return true;
}

} // namespace

ForceInteractionBoundaryLogAnalysisResult
ForceInteractionBoundaryLogAnalyzer::analyze(const QString& csvPath)
{
    ForceInteractionBoundaryLogAnalysisResult result;
    result.csvPath = QFileInfo(csvPath).absoluteFilePath();
    const QFileInfo csvInfo(result.csvPath);
    result.reportPath = csvInfo.dir().filePath(
                csvInfo.completeBaseName() +
                QStringLiteral("_boundary_analysis.json"));

    QFile file(result.csvPath);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        result.errorMessage = QStringLiteral("无法打开阶段B CSV：%1")
                .arg(file.errorString());
        result.summary = QStringLiteral("阶段B边界离线复算失败：%1")
                .arg(result.errorMessage);
        return result;
    }

    PhysicalWorkspaceBoundaryConfig boundaryConfig;
    DynamicWorkspaceSafetyConfig safetyConfig;
    int pointCount = -1;
    bool replayEnabled = false;
    bool frameMinimumRead = false;
    bool frameMaximumRead = false;
    bool orientationEnabledRead = false;
    bool orientationMinimumRead = false;
    bool orientationMaximumRead = false;
    bool stoppingDecelerationRead = false;
    bool additionalMarginRead = false;
    bool emergencyMarginRead = false;
    bool velocityToleranceRead = false;
    bool accelerationToleranceRead = false;
    bool recorderAcceptedRead = false;
    bool recorderWrittenRead = false;
    bool recorderDroppedRead = false;
    bool schemaV4 = false;
    QStringList header;

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    while(!stream.atEnd()){
        const QString line = stream.readLine();
        if(line.startsWith(QLatin1Char('#'))){
            if(line.trimmed() == QStringLiteral("# schema=force_interaction_run_v4")){
                schemaV4 = true;
            }
            QString value = valueAfterEquals(line,
                                               QStringLiteral("workspace_replay_enabled"));
            if(!value.isEmpty()){
                int enabled = 0;
                replayEnabled = parseInt(value, enabled) && enabled == 1;
            }
            value = valueAfterEquals(line, QStringLiteral("workspace_frame_min_mm"));
            if(!value.isEmpty()){
                frameMinimumRead = parseVector3(value,
                                                 boundaryConfig.frameMinimumMm);
            }
            value = valueAfterEquals(line, QStringLiteral("workspace_frame_max_mm"));
            if(!value.isEmpty()){
                frameMaximumRead = parseVector3(value,
                                                 boundaryConfig.frameMaximumMm);
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("workspace_orientation_enabled"));
            if(!value.isEmpty()){
                int enabled = 0;
                if(parseInt(value, enabled)){
                    boundaryConfig.orientationBoundsEnabled = enabled != 0;
                    orientationEnabledRead = true;
                }
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("workspace_orientation_min_rad"));
            if(!value.isEmpty()){
                orientationMinimumRead = parseVector3(
                            value, boundaryConfig.orientationMinimumRad);
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("workspace_orientation_max_rad"));
            if(!value.isEmpty()){
                orientationMaximumRead = parseVector3(
                            value, boundaryConfig.orientationMaximumRad);
            }
            value = valueAfterEquals(line, QStringLiteral("workspace_point_count"));
            if(!value.isEmpty()){
                parseInt(value, pointCount);
            }
            if(pointCount > 0 && pointCount <=
                    kPhysicalWorkspaceMaximumPlatformPoints){
                for(int point = static_cast<int>(boundaryConfig.platformPointsLocalMm.size());
                    point < pointCount; ++point){
                    value = valueAfterEquals(
                                line,
                                QStringLiteral("workspace_point_local_mm_%1").arg(point));
                    if(value.isEmpty()){
                        break;
                    }
                    std::array<double, 3> coordinates{};
                    if(parseVector3(value, coordinates)){
                        boundaryConfig.platformPointsLocalMm.push_back(coordinates);
                    }
                    break;
                }
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("stopping_deceleration_mm_s2"));
            if(!value.isEmpty()){
                stoppingDecelerationRead = parseDouble(value,
                            safetyConfig.stoppingDecelerationMmPerSec2);
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("additional_safety_margin_mm"));
            if(!value.isEmpty()){
                additionalMarginRead = parseDouble(
                            value, safetyConfig.additionalSafetyMarginMm);
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("emergency_line_margin_mm"));
            if(!value.isEmpty()){
                emergencyMarginRead = parseDouble(
                            value, safetyConfig.emergencyLineMarginMm);
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("outward_velocity_tolerance_mm_s"));
            if(!value.isEmpty()){
                velocityToleranceRead = parseDouble(
                            value, safetyConfig.outwardVelocityToleranceMmPerSec);
            }
            value = valueAfterEquals(line,
                                     QStringLiteral("outward_acceleration_tolerance_mm_s2"));
            if(!value.isEmpty()){
                accelerationToleranceRead = parseDouble(
                            value,
                            safetyConfig.outwardAccelerationToleranceMmPerSec2);
            }
            value = valueAfterEquals(line, QStringLiteral("recorder_accepted"));
            if(!value.isEmpty()){
                recorderAcceptedRead = parseUnsigned(
                            value, result.recorderAcceptedRows);
            }
            value = valueAfterEquals(line, QStringLiteral("recorder_written"));
            if(!value.isEmpty()){
                recorderWrittenRead = parseUnsigned(
                            value, result.recorderWrittenRows);
            }
            value = valueAfterEquals(line, QStringLiteral("recorder_dropped"));
            if(!value.isEmpty()){
                recorderDroppedRead = parseUnsigned(
                            value, result.recorderDroppedRows);
            }
            continue;
        }
        if(line.trimmed().isEmpty()){
            continue;
        }
        if(header.isEmpty()){
            header = line.split(QLatin1Char(','));
        }
    }
    stream.setDevice(nullptr);
    file.close();

    QString configError;
    const bool orientationMetadataComplete =
            !boundaryConfig.orientationBoundsEnabled ||
            (orientationMinimumRead && orientationMaximumRead);
    if(!schemaV4 || !replayEnabled || !frameMinimumRead || !frameMaximumRead ||
            !orientationEnabledRead ||
            pointCount <= 0 || pointCount > kPhysicalWorkspaceMaximumPlatformPoints ||
            static_cast<int>(boundaryConfig.platformPointsLocalMm.size()) != pointCount ||
            !orientationMetadataComplete || !stoppingDecelerationRead ||
            !additionalMarginRead || !emergencyMarginRead ||
            !velocityToleranceRead || !accelerationToleranceRead ||
            !recorderAcceptedRead || !recorderWrittenRead ||
            !recorderDroppedRead ||
            !boundaryConfig.validate(&configError) ||
            !safetyConfig.validate(&configError)){
        result.errorMessage = configError.isEmpty() ?
                    QStringLiteral("CSV不是带完整边界快照的v4阶段B记录") :
                    configError;
        result.summary = QStringLiteral("阶段B边界离线复算失败：%1")
                .arg(result.errorMessage);
        QString reportError;
        writeReport(result, &reportError);
        return result;
    }

    QHash<QString, int> columns;
    for(int index = 0; index < header.size(); ++index){
        columns.insert(header[index].trimmed(), index);
    }
    const int availabilityColumn = requiredColumn(columns,
                                                   QStringLiteral("availability_mask"));
    const int traceSequenceColumn = requiredColumn(columns,
                                                    QStringLiteral("trace_sequence"));
    const int traceValidColumn = requiredColumn(columns,
                                                 QStringLiteral("trace_valid"));
    const int segmentColumn = requiredColumn(columns,
                                              QStringLiteral("interaction_segment"));
    const int actionColumn = requiredColumn(columns,
                                             QStringLiteral("workspace_action"));
    const int minimumClearanceColumn = requiredColumn(
                columns, QStringLiteral("workspace_minimum_clearance_mm"));
    const int limitingClearanceColumn = requiredColumn(
                columns, QStringLiteral("workspace_limiting_clearance_mm"));
    const int outwardSpeedColumn = requiredColumn(
                columns, QStringLiteral("workspace_outward_speed_mm_s"));
    const int outwardAccelerationColumn = requiredColumn(
                columns, QStringLiteral("workspace_outward_acceleration_mm_s2"));
    const int pureStoppingDistanceColumn = requiredColumn(
                columns, QStringLiteral("workspace_pure_stopping_distance_mm"));
    const int triggerDistanceColumn = requiredColumn(
                columns, QStringLiteral("workspace_trigger_distance_mm"));
    const int limitingPointColumn = requiredColumn(
                columns, QStringLiteral("workspace_limiting_point"));
    const int limitingAxisColumn = requiredColumn(
                columns, QStringLiteral("workspace_limiting_axis"));
    const int limitingUpperFaceColumn = requiredColumn(
                columns, QStringLiteral("workspace_limiting_upper_face"));

    std::array<int, 6> poseColumns{};
    std::array<int, 6> twistColumns{};
    std::array<int, 6> accelerationColumns{};
    std::array<std::array<int, 3>, kPhysicalWorkspaceMaximumPlatformPoints>
            pointColumns{};
    bool columnsComplete = availabilityColumn >= 0 && traceSequenceColumn >= 0 &&
            traceValidColumn >= 0 && segmentColumn >= 0 && actionColumn >= 0 &&
            minimumClearanceColumn >= 0 && limitingClearanceColumn >= 0 &&
            outwardSpeedColumn >= 0 && outwardAccelerationColumn >= 0 &&
            pureStoppingDistanceColumn >= 0 && triggerDistanceColumn >= 0 &&
            limitingPointColumn >= 0 && limitingAxisColumn >= 0 &&
            limitingUpperFaceColumn >= 0;
    for(int dimension = 0; dimension < 6; ++dimension){
        poseColumns[static_cast<size_t>(dimension)] = requiredColumn(
                    columns, QStringLiteral("desired_pose_si_%1").arg(dimension));
        twistColumns[static_cast<size_t>(dimension)] = requiredColumn(
                    columns, QStringLiteral("desired_twist_si_%1").arg(dimension));
        accelerationColumns[static_cast<size_t>(dimension)] = requiredColumn(
                    columns,
                    QStringLiteral("desired_acceleration_si_%1").arg(dimension));
        columnsComplete = columnsComplete && poseColumns[dimension] >= 0 &&
                twistColumns[dimension] >= 0 && accelerationColumns[dimension] >= 0;
    }
    static const char* const axes[] = {"x", "y", "z"};
    for(int point = 0; point < pointCount; ++point){
        for(int axis = 0; axis < 3; ++axis){
            pointColumns[static_cast<size_t>(point)][static_cast<size_t>(axis)] =
                    requiredColumn(
                        columns,
                        QStringLiteral("workspace_point_%1_%2_mm")
                        .arg(point)
                        .arg(QString::fromLatin1(axes[axis])));
            columnsComplete = columnsComplete && pointColumns[point][axis] >= 0;
        }
    }
    if(!columnsComplete){
        result.errorMessage = QStringLiteral("CSV缺少边界复算所需字段");
        result.summary = QStringLiteral("阶段B边界离线复算失败：%1")
                .arg(result.errorMessage);
        QString reportError;
        writeReport(result, &reportError);
        return result;
    }

    PhysicalWorkspaceBoundary boundary;
    if(!boundary.configure(boundaryConfig, &configError)){
        result.errorMessage = configError;
        result.summary = QStringLiteral("阶段B边界离线复算失败：%1")
                .arg(result.errorMessage);
        QString reportError;
        writeReport(result, &reportError);
        return result;
    }

    qint64 previousTraceSequence = -1;
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        result.errorMessage = QStringLiteral("无法重新打开阶段B CSV：%1")
                .arg(file.errorString());
        result.summary = QStringLiteral("阶段B边界离线复算失败：%1")
                .arg(result.errorMessage);
        QString reportError;
        writeReport(result, &reportError);
        return result;
    }
    QTextStream dataStream(&file);
    dataStream.setEncoding(QStringConverter::Utf8);
    bool headerSkipped = false;
    while(!dataStream.atEnd()){
        const QString line = dataStream.readLine();
        if(line.startsWith(QLatin1Char('#')) || line.trimmed().isEmpty()){
            continue;
        }
        if(!headerSkipped){
            headerSkipped = true;
            continue;
        }
        const QStringList row = line.split(QLatin1Char(','));
        ++result.dataRows;
        qint64 availability = 0;
        qint64 traceSequence = 0;
        qint64 traceValid = 0;
        qint64 segment = 0;
        qint64 recordedAction = 0;
        qint64 recordedPoint = 0;
        qint64 recordedAxis = 0;
        qint64 recordedUpperFace = 0;
        PhysicalWorkspaceMotionSample sample;
        bool rowValid = readIntegerColumn(row, availabilityColumn, availability) &&
                (availability & ForceRecordDesiredState) != 0 &&
                readIntegerColumn(row, traceSequenceColumn, traceSequence) &&
                readIntegerColumn(row, traceValidColumn, traceValid) &&
                readIntegerColumn(row, segmentColumn, segment) &&
                readIntegerColumn(row, actionColumn, recordedAction) &&
                readIntegerColumn(row, limitingPointColumn, recordedPoint) &&
                readIntegerColumn(row, limitingAxisColumn, recordedAxis) &&
                readIntegerColumn(row, limitingUpperFaceColumn, recordedUpperFace);
        for(int dimension = 0; dimension < 6 && rowValid; ++dimension){
            rowValid = readDoubleColumn(
                        row, poseColumns[dimension], sample.poseMmRad[dimension]) &&
                    readDoubleColumn(
                        row, twistColumns[dimension], sample.twistMmRadPerSec[dimension]) &&
                    readDoubleColumn(
                        row, accelerationColumns[dimension],
                        sample.accelerationMmRadPerSec2[dimension]);
        }
        for(int dimension = 0; dimension < 3 && rowValid; ++dimension){
            sample.poseMmRad[dimension] *= 1000.0;
            sample.twistMmRadPerSec[dimension] *= 1000.0;
            sample.accelerationMmRadPerSec2[dimension] *= 1000.0;
        }
        double recordedMinimum = 0.0;
        double recordedLimiting = 0.0;
        double recordedOutwardSpeed = 0.0;
        double recordedOutwardAcceleration = 0.0;
        double recordedPureDistance = 0.0;
        double recordedTriggerDistance = 0.0;
        rowValid = rowValid &&
                readDoubleColumn(row, minimumClearanceColumn, recordedMinimum) &&
                readDoubleColumn(row, limitingClearanceColumn, recordedLimiting) &&
                readDoubleColumn(row, outwardSpeedColumn, recordedOutwardSpeed) &&
                readDoubleColumn(row, outwardAccelerationColumn,
                                 recordedOutwardAcceleration) &&
                readDoubleColumn(row, pureStoppingDistanceColumn,
                                 recordedPureDistance) &&
                readDoubleColumn(row, triggerDistanceColumn,
                                 recordedTriggerDistance);
        if(!rowValid){
            ++result.malformedRows;
            continue;
        }
        if(traceValid == 0){
            ++result.traceInvalidRows;
        }
        if(previousTraceSequence >= 0 && traceSequence <= previousTraceSequence){
            ++result.traceNonMonotonicRows;
        }
        previousTraceSequence = traceSequence;
        if(segment == 0){
            ++result.interactionRows;
        }
        else{
            ++result.brakingRows;
        }

        const PhysicalWorkspaceBoundaryResult replay =
                boundary.evaluateMotion(sample, safetyConfig);
        ++result.replayedRows;
        bool mismatch = replay.action == PhysicalWorkspaceAction::Invalid;
        if(static_cast<qint64>(replay.action) != recordedAction){
            ++result.actionMismatchRows;
            mismatch = true;
        }
        if(replay.limitingPointIndex != recordedPoint ||
                replay.limitingAxis != recordedAxis ||
                static_cast<qint64>(replay.limitingUpperFace) !=
                    recordedUpperFace){
            mismatch = true;
        }
        double difference = 0.0;
        if(!nearlyEqual(replay.minimumClearanceMm, recordedMinimum,
                        &difference)){
            mismatch = true;
        }
        result.maximumClearanceDifferenceMm = std::max(
                    result.maximumClearanceDifferenceMm, difference);
        if(!nearlyEqual(replay.limitingClearanceMm, recordedLimiting) ||
                !nearlyEqual(replay.limitingOutwardSpeedMmPerSec,
                             recordedOutwardSpeed) ||
                !nearlyEqual(replay.limitingOutwardAccelerationMmPerSec2,
                             recordedOutwardAcceleration) ||
                !nearlyEqual(replay.pureStoppingDistanceMm,
                             recordedPureDistance)){
            mismatch = true;
        }
        if(!nearlyEqual(replay.triggerDistanceMm, recordedTriggerDistance,
                        &difference)){
            mismatch = true;
        }
        result.maximumTriggerDistanceDifferenceMm = std::max(
                    result.maximumTriggerDistanceDifferenceMm, difference);

        bool pointMismatch = false;
        for(int point = 0; point < pointCount; ++point){
            for(int axis = 0; axis < 3; ++axis){
                double recordedCoordinate = 0.0;
                difference = 0.0;
                if(!readDoubleColumn(row, pointColumns[point][axis],
                                     recordedCoordinate)){
                    pointMismatch = true;
                    continue;
                }
                if(!nearlyEqual(replay.platformPointsGlobalMm[point][axis],
                                recordedCoordinate, &difference)){
                    pointMismatch = true;
                }
                result.maximumPointDifferenceMm = std::max(
                            result.maximumPointDifferenceMm, difference);
            }
        }
        if(pointMismatch){
            ++result.pointMismatchRows;
            mismatch = true;
        }
        if(mismatch){
            ++result.mismatchRows;
        }
    }
    file.close();

    result.passed = result.dataRows > 0 &&
            result.replayedRows == result.dataRows &&
            result.malformedRows == 0 && result.mismatchRows == 0 &&
            result.traceInvalidRows == 0 &&
            result.traceNonMonotonicRows == 0 &&
            result.recorderDroppedRows == 0 &&
            result.recorderAcceptedRows == result.recorderWrittenRows &&
            result.recorderWrittenRows == result.dataRows;
    result.summary = QStringLiteral(
                "阶段B边界离线复算%1：复算/数据=%2/%3行，在线结论不一致=%4行（动作=%5，连接点=%6），"
                "格式错误=%7行，Trace无效/非递增=%8/%9行，有效交互/制动=%10/%11行，"
                "记录接受/写入/丢弃=%12/%13/%14，最大余量/触发距离/连接点差=%15/%16/%17 mm。报告=%18")
            .arg(result.passed ? QStringLiteral("通过") : QStringLiteral("未通过"))
            .arg(result.replayedRows)
            .arg(result.dataRows)
            .arg(result.mismatchRows)
            .arg(result.actionMismatchRows)
            .arg(result.pointMismatchRows)
            .arg(result.malformedRows)
            .arg(result.traceInvalidRows)
            .arg(result.traceNonMonotonicRows)
            .arg(result.interactionRows)
            .arg(result.brakingRows)
            .arg(result.recorderAcceptedRows)
            .arg(result.recorderWrittenRows)
            .arg(result.recorderDroppedRows)
            .arg(result.maximumClearanceDifferenceMm, 0, 'g', 8)
            .arg(result.maximumTriggerDistanceDifferenceMm, 0, 'g', 8)
            .arg(result.maximumPointDifferenceMm, 0, 'g', 8)
            .arg(QFileInfo(result.reportPath).absoluteFilePath());

    QString reportError;
    if(!writeReport(result, &reportError)){
        result.passed = false;
        result.errorMessage = reportError;
        result.summary += QStringLiteral("；报告写入失败：%1").arg(reportError);
    }
    return result;
}

ForceInteractionBoundaryLogAnalysisWorker::
ForceInteractionBoundaryLogAnalysisWorker(const QString& csvPath,
                                           QObject* parent)
    : QThread(parent), csvPath_(csvPath)
{
    setObjectName(QStringLiteral("ForceInteractionBoundaryLogAnalysisWorker"));
}

ForceInteractionBoundaryLogAnalysisResult
ForceInteractionBoundaryLogAnalysisWorker::result() const
{
    return result_;
}

void ForceInteractionBoundaryLogAnalysisWorker::run()
{
    result_ = ForceInteractionBoundaryLogAnalyzer::analyze(csvPath_);
}
