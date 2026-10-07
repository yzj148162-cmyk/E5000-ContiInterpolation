#include "forceinteractionkinematicloganalyzer.h"

#include "forwardkinematicssolver.h"
#include "forceinteractionrunrecorder.h"
#include "winchcompensation.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QElapsedTimer>
#include <QTextStream>

namespace {
constexpr int kAxisCount = 8;
constexpr int kPoseCount = 6;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

struct Row {
    quint64 step = 0;
    double modelTimeS = 0.0;
    qint64 hostUs = 0;
    quint64 traceSequence = 0;
    qint64 traceUs = 0;
    bool traceValid = false;
    std::array<double, kPoseCount> desired{}; // m, rad
    std::array<double, kPoseCount> desiredTwist{}; // m/s, rad/s
    std::array<double, kPoseCount> desiredAcceleration{}; // m/s2, rad/s2
    std::array<double, kAxisCount> relativeTrace{};
};

double wrappedAngle(double value)
{
    return std::atan2(std::sin(value), std::cos(value));
}

bool number(const QStringList& values, int index, double* value)
{
    if(index < 0 || index >= values.size()) return false;
    bool ok = false;
    const double parsed = values[index].toDouble(&ok);
    if(!ok || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

bool integer64(const QStringList& values, int index, qint64* value)
{
    if(index < 0 || index >= values.size()) return false;
    bool ok = false;
    const qint64 parsed = values[index].toLongLong(&ok);
    if(!ok) return false;
    *value = parsed;
    return true;
}

struct DesiredStateAtHost
{
    std::array<double, kPoseCount> pose{};
    std::array<double, kPoseCount> twist{};
    std::array<double, kPoseCount> acceleration{};
};

DesiredStateAtHost desiredStateAtHost(
        const std::vector<Row>& rows, qint64 hostUs)
{
    if(rows.empty()) return {};
    const auto upper = std::lower_bound(rows.begin(), rows.end(), hostUs,
        [](const Row& row, qint64 value){ return row.hostUs < value; });
    auto fromRow = [](const Row& row){
        DesiredStateAtHost state;
        state.pose = row.desired;
        state.twist = row.desiredTwist;
        state.acceleration = row.desiredAcceleration;
        return state;
    };
    if(upper == rows.begin()) return fromRow(*upper);
    if(upper == rows.end()) return fromRow(rows.back());
    const Row& b = *upper;
    const Row& a = *(upper - 1);
    const qint64 span = b.hostUs - a.hostUs;
    const double ratio = span > 0 ?
                std::clamp(double(hostUs - a.hostUs) / double(span), 0.0, 1.0) : 0.0;
    DesiredStateAtHost result;
    for(int i = 0; i < kPoseCount; ++i){
        double delta = b.desired[i] - a.desired[i];
        if(i >= 3) delta = wrappedAngle(delta);
        result.pose[i] = a.desired[i] + ratio * delta;
        result.twist[i] = a.desiredTwist[i] +
                ratio * (b.desiredTwist[i] - a.desiredTwist[i]);
        result.acceleration[i] = a.desiredAcceleration[i] +
                ratio * (b.desiredAcceleration[i] - a.desiredAcceleration[i]);
    }
    return result;
}

bool metadataNumber(const QString& line, const QString& key, double* value)
{
    const QString token = key + '=';
    const int begin = line.indexOf(token);
    if(begin < 0) return false;
    const int valueBegin = begin + token.size();
    const int comma = line.indexOf(',', valueBegin);
    bool ok = false;
    const double parsed = line.mid(valueBegin,
                                   comma < 0 ? -1 : comma - valueBegin)
            .trimmed().toDouble(&ok);
    if(!ok || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

QString metadataText(const QString& line, const QString& key)
{
    const QString token = key + '=';
    const int begin = line.indexOf(token);
    if(begin < 0) return {};
    const int valueBegin = begin + token.size();
    const int comma = line.indexOf(',', valueBegin);
    QString value = line.mid(valueBegin, comma < 0 ? -1 : comma - valueBegin)
            .trimmed();
    if(value.size() >= 2 && value.front() == QLatin1Char('"') &&
            value.back() == QLatin1Char('"')){
        value = value.mid(1, value.size() - 2);
        value.replace(QStringLiteral("\"\""), QStringLiteral("\""));
    }
    return value;
}

ForceInteractionKinematicLogAnalysisResult analyzeStaticTensionRecord(
        const QString& csvPath)
{
    ForceInteractionKinematicLogAnalysisResult result;
    result.csvPath = csvPath;
    result.staticTensionMode = true;
    QFile source(csvPath);
    if(!source.open(QIODevice::ReadOnly | QIODevice::Text)){
        result.errorMessage = QStringLiteral("无法读取M3运行记录：%1")
                .arg(source.errorString());
        result.summary = QStringLiteral("M3静态张力验算失败：%1")
                .arg(result.errorMessage);
        return result;
    }

    QTextStream input(&source);
    QStringList header;
    double quantumNm = 0.0;
    double slewNmPerSec = 0.0;
    double terminalPresent = 0.0;
    double terminalExperimentValid = 0.0;
    double recorderAccepted = -1.0;
    double recorderWritten = -1.0;
    double recorderDropped = -1.0;
    bool schemaV16 = false;
    QString parameterVersion;
    QString parameterHash;
    QString configHash;
    QFileInfo sourceInfo(csvPath);
    result.resultCsvPath = sourceInfo.dir().filePath(
                sourceInfo.completeBaseName() +
                QStringLiteral("_static_tension_analysis.csv"));
    QFile output(result.resultCsvPath);
    if(!output.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)){
        result.errorMessage = QStringLiteral("无法创建M3验算结果：%1")
                .arg(output.errorString());
        result.summary = QStringLiteral("M3静态张力验算失败：%1")
                .arg(result.errorMessage);
        return result;
    }
    QTextStream report(&output);
    report.setRealNumberNotation(QTextStream::SmartNotation);
    report.setRealNumberPrecision(12);
    report << "row_index,trace_sequence,batch_ok,error_consistent,quantized,slew_ok,"
              "maximum_abs_tension_error_n,maximum_quantum_residual_nm,"
              "maximum_slew_excess_nm\n";

    bool havePreviousCommand = false;
    qint64 previousHostUs = 0;
    std::array<double, kAxisCount> previousCommand{};
    quint64 rowIndex = 0;
    while(!input.atEnd()){
        const QString line = input.readLine();
        if(line.startsWith('#')){
            if(line.trimmed() == QStringLiteral(
                       "# schema=force_interaction_run_v16")) schemaV16 = true;
            metadataNumber(line, QStringLiteral("static_tension_torque_quantum_nm"),
                           &quantumNm);
            metadataNumber(line, QStringLiteral("static_tension_torque_slew_nm_s"),
                           &slewNmPerSec);
            metadataNumber(line, QStringLiteral("terminal_summary_present"),
                           &terminalPresent);
            metadataNumber(line, QStringLiteral("terminal_experiment_valid"),
                           &terminalExperimentValid);
            metadataNumber(line, QStringLiteral("recorder_accepted"),
                           &recorderAccepted);
            metadataNumber(line, QStringLiteral("recorder_written"),
                           &recorderWritten);
            metadataNumber(line, QStringLiteral("recorder_dropped"),
                           &recorderDropped);
            const QString parsedVersion = metadataText(
                        line, QStringLiteral("static_tension_parameter_version"));
            if(!parsedVersion.isEmpty()) parameterVersion = parsedVersion;
            const QString parsedParameterHash = metadataText(
                        line, QStringLiteral("static_tension_parameter_hash"));
            if(!parsedParameterHash.isEmpty()) parameterHash = parsedParameterHash;
            const QString parsedConfigHash = metadataText(
                        line, QStringLiteral("static_tension_config_hash"));
            if(!parsedConfigHash.isEmpty()) configHash = parsedConfigHash;
            continue;
        }
        if(line.trimmed().isEmpty()) continue;
        if(header.isEmpty()){
            header = line.split(',');
            continue;
        }
        ++result.dataRows;
        ++rowIndex;
        const QStringList values = line.split(',');
        qint64 availability = 0, traceValid = 0, traceSequence = 0, hostUs = 0;
        qint64 proposalCommitted = 0, batchSucceeded = 0;
        qint64 batchPartial = 0, batchDeferred = 0, sameFrame = 0;
        qint64 sourceSequence = 0, sourceUs = 0, sourceAgeUs = -1;
        qint64 queueWaitUs = 0, budgetCheckedUs = 0, budgetRemainingUs = 0;
        qint64 firstCommandUs = 0, lastCommandUs = 0, deadlineUs = 0;
        if(!integer64(values, header.indexOf("availability_mask"), &availability) ||
                !(quint32(availability) & ForceRecordStaticTensionTorque)){
            continue;
        }
        ++result.staticTensionRows;
        bool rowComplete =
                integer64(values, header.indexOf("trace_valid"), &traceValid) &&
                integer64(values, header.indexOf("trace_sequence"), &traceSequence) &&
                integer64(values, header.indexOf("host_monotonic_us"), &hostUs) &&
                integer64(values, header.indexOf(
                              "static_tension_proposal_committed"),
                          &proposalCommitted) &&
                integer64(values, header.indexOf(
                              "static_tension_batch_succeeded"),
                          &batchSucceeded);
        if(schemaV16){
            rowComplete = rowComplete &&
                    integer64(values, header.indexOf(
                                  "static_tension_batch_partial"),
                              &batchPartial) &&
                    integer64(values, header.indexOf(
                                  "static_tension_batch_deferred_before_write"),
                              &batchDeferred) &&
                    integer64(values, header.indexOf(
                                  "static_tension_batch_queue_wait_us"),
                              &queueWaitUs) &&
                    integer64(values, header.indexOf(
                                  "static_tension_batch_budget_checked_us"),
                              &budgetCheckedUs) &&
                    integer64(values, header.indexOf(
                                  "static_tension_batch_budget_remaining_us"),
                              &budgetRemainingUs) &&
                    integer64(values, header.indexOf(
                                  "static_tension_batch_first_command_us"),
                              &firstCommandUs) &&
                    integer64(values, header.indexOf(
                                  "static_tension_batch_last_command_us"),
                              &lastCommandUs) &&
                    integer64(values, header.indexOf(
                                  "static_tension_command_deadline_us"),
                              &deadlineUs) &&
                    integer64(values, header.indexOf(
                                  "static_tension_trace_same_frame"),
                              &sameFrame) &&
                    integer64(values, header.indexOf(
                                  "static_tension_object_source_trace_sequence"),
                              &sourceSequence) &&
                    integer64(values, header.indexOf(
                                  "static_tension_object_source_trace_us"),
                              &sourceUs) &&
                    integer64(values, header.indexOf(
                                  "static_tension_object_age_us"),
                              &sourceAgeUs) &&
                    sameFrame != 0 &&
                    sourceSequence == traceSequence && sourceUs > 0 &&
                    sourceAgeUs >= 0;
        }
        std::array<double, kAxisCount> command{};
        double maximumError = 0.0;
        double maximumQuantumResidual = 0.0;
        double maximumSlewExcess = 0.0;
        bool errorConsistent = true;
        bool quantized = quantumNm > 0.0;
        bool slewOk = slewNmPerSec > 0.0;
        for(int axis = 0; axis < kAxisCount; ++axis){
            double target = 0.0, measured = 0.0, recordedError = 0.0;
            double actualTorque = 0.0;
            double executionTarget = 0.0, entryTorque = 0.0;
            double baseTorque = 0.0, pidCorrection = 0.0;
            double prelimitTorque = 0.0, continuousTorque = 0.0;
            qint64 tensionValid = 0;
            rowComplete = rowComplete &&
                    number(values, header.indexOf(QStringLiteral(
                               "static_tension_target_n_%1").arg(axis)), &target) &&
                    number(values, header.indexOf(QStringLiteral(
                               "static_tension_measured_n_%1").arg(axis)), &measured) &&
                    number(values, header.indexOf(QStringLiteral(
                               "static_tension_error_n_%1").arg(axis)),
                           &recordedError) &&
                    number(values, header.indexOf(QStringLiteral(
                               "static_tension_command_torque_nm_%1").arg(axis)),
                           &command[axis]) &&
                    number(values, header.indexOf(QStringLiteral(
                               "static_tension_actual_torque_nm_%1").arg(axis)),
                           &actualTorque) &&
                    integer64(values, header.indexOf(QStringLiteral(
                               "cable_tension_valid_%1").arg(axis)),
                              &tensionValid) && tensionValid != 0;
            if(schemaV16){
                rowComplete = rowComplete &&
                        number(values, header.indexOf(QStringLiteral(
                                   "static_tension_execution_target_n_%1").arg(axis)),
                               &executionTarget) &&
                        number(values, header.indexOf(QStringLiteral(
                                   "static_tension_entry_torque_nm_%1").arg(axis)),
                               &entryTorque) &&
                        number(values, header.indexOf(QStringLiteral(
                                   "static_tension_base_torque_nm_%1").arg(axis)),
                               &baseTorque) &&
                        number(values, header.indexOf(QStringLiteral(
                                   "static_tension_pid_correction_torque_nm_%1").arg(axis)),
                               &pidCorrection) &&
                        number(values, header.indexOf(QStringLiteral(
                                   "static_tension_prelimit_torque_nm_%1").arg(axis)),
                               &prelimitTorque) &&
                        number(values, header.indexOf(QStringLiteral(
                                   "static_tension_continuous_torque_nm_%1").arg(axis)),
                               &continuousTorque);
            }
            if(!rowComplete) break;
            maximumError = std::max(maximumError, std::abs(recordedError));
            if(std::abs(recordedError - (target - measured)) > 1.0e-6)
                errorConsistent = false;
            if(quantumNm > 0.0){
                const double residual = std::abs(command[axis] -
                        std::round(command[axis] / quantumNm) * quantumNm);
                maximumQuantumResidual = std::max(maximumQuantumResidual, residual);
                if(residual > 1.0e-6) quantized = false;
            }
            if(havePreviousCommand && slewNmPerSec > 0.0 && hostUs > previousHostUs){
                const double allowed = slewNmPerSec *
                        double(hostUs - previousHostUs) * 1.0e-6 + quantumNm + 1.0e-6;
                const double excess = std::abs(command[axis] -
                                                previousCommand[axis]) - allowed;
                maximumSlewExcess = std::max(maximumSlewExcess,
                                              std::max(0.0, excess));
                if(excess > 0.0) slewOk = false;
            }
        }
        if(!rowComplete || traceValid == 0){
            ++result.failedRows;
            continue;
        }
        ++result.validTraceRows;
        if(proposalCommitted != 0) ++result.staticTensionCommittedRows;
        const bool batchOk = batchSucceeded != 0 && batchPartial == 0 &&
                batchDeferred == 0 && queueWaitUs >= 0 &&
                budgetCheckedUs > 0 && budgetRemainingUs >= 0 &&
                firstCommandUs > 0 && lastCommandUs >= firstCommandUs &&
                deadlineUs > 0;
        if(!batchOk) ++result.staticTensionBatchFailureRows;
        if(!errorConsistent) ++result.staticTensionErrorMismatchRows;
        if(!quantized) ++result.staticTensionQuantizationViolationRows;
        if(!slewOk) ++result.staticTensionSlewViolationRows;
        result.staticTensionMaximumAbsErrorN = std::max(
                    result.staticTensionMaximumAbsErrorN, maximumError);
        result.staticTensionMaximumQuantumResidualNm = std::max(
                    result.staticTensionMaximumQuantumResidualNm,
                    maximumQuantumResidual);
        result.staticTensionMaximumSlewExcessNm = std::max(
                    result.staticTensionMaximumSlewExcessNm, maximumSlewExcess);
        report << rowIndex << ',' << traceSequence << ',' << (batchOk ? 1 : 0)
               << ',' << (errorConsistent ? 1 : 0) << ',' << (quantized ? 1 : 0)
               << ',' << (slewOk ? 1 : 0) << ',' << maximumError << ','
               << maximumQuantumResidual << ',' << maximumSlewExcess << '\n';
        previousCommand = command;
        previousHostUs = hostUs;
        havePreviousCommand = true;
    }
    if(quantumNm <= 0.0 || slewNmPerSec <= 0.0){
        result.errorMessage = QStringLiteral("M3记录缺少冻结的转矩量化或斜率参数");
    }
    else if(schemaV16 && (parameterVersion != QStringLiteral("m3_0525_static_v1") ||
                         parameterHash.size() != 64 || configHash.size() != 64)){
        result.errorMessage = QStringLiteral("M3 v16记录缺少冻结参数版本或SHA-256哈希");
    }
    else if(terminalPresent != 1.0 || terminalExperimentValid != 1.0){
        result.errorMessage = QStringLiteral("M3记录缺少有效的终态摘要");
    }
    else if(recorderAccepted < 0.0 || recorderWritten < 0.0 ||
            recorderDropped != 0.0 || recorderAccepted != recorderWritten){
        result.errorMessage = QStringLiteral("M3记录队列不完整");
    }
    else if(result.staticTensionRows == 0){
        result.errorMessage = QStringLiteral("记录中没有M3静态张力数据行");
    }
    else if(result.failedRows || result.staticTensionBatchFailureRows ||
            result.staticTensionErrorMismatchRows ||
            result.staticTensionQuantizationViolationRows ||
            result.staticTensionSlewViolationRows){
        result.errorMessage = QStringLiteral("M3记录存在数据、批次、误差、量化或斜率不一致");
    }
    result.completed = result.errorMessage.isEmpty();
    result.summary = QStringLiteral(
                "M3静态张力验算%1：数据/有效/提交=%2/%3/%4行，"
                "批次失败/误差不一致/量化越界/斜率越界=%5/%6/%7/%8行，"
                "最大|张力误差|=%9 N，最大量化残差=%10 N·m，"
                "最大斜率超额=%11 N·m；报告=%12")
            .arg(result.completed ? QStringLiteral("通过") : QStringLiteral("未通过"))
            .arg(result.staticTensionRows).arg(result.validTraceRows)
            .arg(result.staticTensionCommittedRows)
            .arg(result.staticTensionBatchFailureRows)
            .arg(result.staticTensionErrorMismatchRows)
            .arg(result.staticTensionQuantizationViolationRows)
            .arg(result.staticTensionSlewViolationRows)
            .arg(result.staticTensionMaximumAbsErrorN, 0, 'f', 6)
            .arg(result.staticTensionMaximumQuantumResidualNm, 0, 'g', 6)
            .arg(result.staticTensionMaximumSlewExcessNm, 0, 'g', 6)
            .arg(result.resultCsvPath);
    if(!result.errorMessage.isEmpty())
        result.summary += QStringLiteral("；原因=%1").arg(result.errorMessage);
    return result;
}
}

ForceInteractionKinematicLogAnalysisResult
ForceInteractionKinematicLogAnalyzer::analyze(
        const ForceInteractionKinematicLogAnalysisRequest& request)
{
    ForceInteractionKinematicLogAnalysisResult result;
    result.csvPath = request.csvPath;
    QFile source(request.csvPath);
    if(!source.open(QIODevice::ReadOnly | QIODevice::Text)){
        result.errorMessage = QStringLiteral("无法读取运行记录：%1").arg(source.errorString());
        result.summary = QStringLiteral("六维力交互运动学验算失败：%1").arg(result.errorMessage);
        return result;
    }
    const QByteArray prefix = source.peek(8192);
    if(prefix.contains("execution_mode=static_tension_torque_m3")){
        source.close();
        return analyzeStaticTensionRecord(request.csvPath);
    }
    if(request.referenceCableLengthMm.size() != kAxisCount ||
            request.initialPoseMmRad.size() < kPoseCount ||
            request.kinematics.anchorCableCoordinate.size() < kAxisCount ||
            request.kinematics.endCableContactPos.empty() ||
            request.kinematics.endCableContactPos.front().size() < kAxisCount ||
            request.kinematics.winchConfig.size() < kAxisCount ||
            request.kinematics.cableMotorScaleRadPerMm.size() < kAxisCount){
        result.errorMessage = QStringLiteral("当前运行的冻结运动学参数不完整");
        result.summary = QStringLiteral("六维力交互运动学验算失败：%1").arg(result.errorMessage);
        return result;
    }

    QTextStream input(&source);
    QStringList header;
    std::vector<Row> rows;
    while(!input.atEnd()){
        const QString line = input.readLine();
        if(line.trimmed().isEmpty() || line.startsWith('#')) continue;
        if(header.isEmpty()){
            header = line.split(',');
            continue;
        }
        ++result.dataRows;
        const QStringList values = line.split(',');
        Row row;
        qint64 step = 0, traceSequence = 0, traceValid = 0;
        if(!integer64(values, header.indexOf("step_index"), &step) ||
                !number(values, header.indexOf("model_elapsed_s"), &row.modelTimeS) ||
                !integer64(values, header.indexOf("host_monotonic_us"), &row.hostUs) ||
                !integer64(values, header.indexOf("trace_sequence"), &traceSequence) ||
                !integer64(values, header.indexOf("trace_time_us"), &row.traceUs) ||
                !integer64(values, header.indexOf("trace_valid"), &traceValid)) continue;
        row.step = quint64(std::max<qint64>(0, step));
        row.traceSequence = quint64(std::max<qint64>(0, traceSequence));
        row.traceValid = traceValid != 0;
        bool complete = true;
        for(int i = 0; i < kPoseCount; ++i){
            complete = complete && number(values,
                    header.indexOf(QStringLiteral("desired_pose_si_%1").arg(i)),
                    &row.desired[i]);
            complete = complete && number(values,
                    header.indexOf(QStringLiteral("desired_twist_si_%1").arg(i)),
                    &row.desiredTwist[i]);
            complete = complete && number(values,
                    header.indexOf(QStringLiteral("desired_acceleration_si_%1").arg(i)),
                    &row.desiredAcceleration[i]);
        }
        for(int axis = 0; axis < kAxisCount; ++axis){
            complete = complete && number(values,
                    header.indexOf(QStringLiteral("axis_safety_relative_trace_position_%1").arg(axis)),
                    &row.relativeTrace[axis]);
        }
        if(complete && row.traceValid) rows.push_back(row);
    }
    result.validTraceRows = rows.size();
    if(rows.empty()){
        result.errorMessage = QStringLiteral("记录中没有完整且有效的八轴同帧Trace数据");
        result.summary = QStringLiteral("六维力交互运动学验算失败：%1").arg(result.errorMessage);
        return result;
    }

    qint64 anchorOffset = std::numeric_limits<qint64>::max();
    quint64 previousSequence = 0;
    for(const Row& row : rows){
        anchorOffset = std::min(anchorOffset, row.hostUs - row.traceUs);
        if(previousSequence && row.traceSequence <= previousSequence){
            ++result.nonMonotonicTraceRows;
        }
        previousSequence = row.traceSequence;
    }
    result.traceHostAnchorOffsetUs = anchorOffset;

    QFileInfo sourceInfo(request.csvPath);
    result.resultCsvPath = sourceInfo.dir().filePath(
                sourceInfo.completeBaseName() + QStringLiteral("_kinematic_analysis.csv"));
    QFile output(result.resultCsvPath);
    if(!output.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)){
        result.errorMessage = QStringLiteral("无法创建验算结果：%1").arg(output.errorString());
        result.summary = QStringLiteral("六维力交互运动学验算失败：%1").arg(result.errorMessage);
        return result;
    }
    QTextStream stream(&output);
    stream.setRealNumberNotation(QTextStream::SmartNotation);
    stream.setRealNumberPrecision(12);
    stream << "step_index,trace_sequence,trace_time_us,aligned_model_time_s";
    for(int i=0;i<kPoseCount;++i) stream << ",desired_pose_mm_rad_" << i;
    for(int i=0;i<kPoseCount;++i) stream << ",actual_pose_mm_rad_" << i;
    stream << ",translation_error_mm,orientation_error_deg,rms_cable_residual_mm,maximum_cable_residual_mm,solver_success,solver_termination,solver_iterations"
              ",shadow_evaluated,shadow_valid,shadow_calculation_us,shadow_wrench_residual,shadow_tension_margin_n";
    for(int i=0;i<kAxisCount;++i) stream << ",shadow_target_tension_n_" << i;
    for(int i=0;i<kAxisCount;++i) stream << ",shadow_nominal_torque_nm_" << i;
    stream << ",shadow_error\n";

    ForceInteractionTensionShadow shadow;
    QString shadowConfigurationError;
    const bool shadowConfigured = request.tensionShadowEnabled &&
            shadow.configure(request.tensionShadow, &shadowConfigurationError);
    qint64 previousShadowTraceUs = 0;
    ForceInteractionPlatformState previousObserved;
    const qint64 shadowPeriodUs = request.tensionShadowEnabled ?
                qint64(std::llround(request.tensionShadow.outerPeriodS * 1.0e6)) : 0;

    ForwardKinematicsSolver solver;
    solver.setInitialPose(request.initialPoseMmRad);
    double sumTranslation2 = 0.0, sumOrientation2 = 0.0, sumResidual2 = 0.0;
    for(const Row& row : rows){
        const qint64 alignedHostUs = row.traceUs + anchorOffset;
        const DesiredStateAtHost desired = desiredStateAtHost(rows, alignedHostUs);
        ForwardKinematicsSolver::Request solveRequest;
        solveRequest.anchorPos = request.kinematics.anchorCableCoordinate;
        solveRequest.contactPointLocal = request.kinematics.endCableContactPos.front();
        solveRequest.pulleyRadius = request.kinematics.pulleyRadiusMm;
        solveRequest.initialPose = solver.initialPose();
        if(solveRequest.initialPose.size() < kPoseCount)
            solveRequest.initialPose = request.initialPoseMmRad;
        solveRequest.keepRotation = true;
        solveRequest.enforcePhysicalWorkspace = true;
        solveRequest.physicalWorkspace = request.physicalWorkspace;
        PhysicalWorkspaceBoundary boundary(request.physicalWorkspace);
        const auto lower = boundary.solverLowerBounds();
        const auto upper = boundary.solverUpperBounds();
        solveRequest.poseLowerBounds.assign(lower.begin(), lower.end());
        solveRequest.poseUpperBounds.assign(upper.begin(), upper.end());
        solveRequest.cableLength.reserve(kAxisCount);
        bool cableValid = true;
        for(int axis=0; axis<kAxisCount; ++axis){
            const double unitPerRad = request.motorUnitPerRadian[axis];
            const double scale = std::abs(request.kinematics.cableMotorScaleRadPerMm[axis]);
            if(!std::isfinite(unitPerRad) || std::abs(unitPerRad) < 1e-12 || scale < 1e-12){
                cableValid = false;
                break;
            }
            const double relativeFromRunStart =
                    row.relativeTrace[axis] -
                    request.actualStartSafetyRelativePosition[axis];
            const double motorTheta = relativeFromRunStart / unitPerRad;
            const double platformDelta = WinchCompensation::platformDeltaFromMotorTheta(
                        request.kinematics.winchConfig[axis], motorTheta, scale);
            solveRequest.cableLength.push_back(request.referenceCableLengthMm[axis] - platformDelta);
        }
        ForwardKinematicsSolver::Result solved;
        if(cableValid) solved = solver.solve(solveRequest);
        stream << row.step << ',' << row.traceSequence << ',' << row.traceUs << ','
               << (row.modelTimeS + double(alignedHostUs - row.hostUs) * 1.0e-6);
        for(int i=0;i<kPoseCount;++i)
            stream << ',' << (i < 3 ? desired.pose[i]*1000.0 : desired.pose[i]);
        for(int i=0;i<kPoseCount;++i)
            stream << ',' << (solved.success && i < int(solved.pose.size()) ? solved.pose[i] : std::numeric_limits<double>::quiet_NaN());
        double translation = std::numeric_limits<double>::quiet_NaN();
        double orientation = std::numeric_limits<double>::quiet_NaN();
        if(solved.success && solved.pose.size() >= kPoseCount){
            double translation2 = 0.0, orientation2 = 0.0;
            for(int i=0;i<3;++i){ const double d=solved.pose[i]-desired.pose[i]*1000.0; translation2 += d*d; }
            for(int i=3;i<6;++i){ const double d=wrappedAngle(solved.pose[i]-desired.pose[i]); orientation2 += d*d; }
            translation = std::sqrt(translation2);
            orientation = std::sqrt(orientation2) * kRadToDeg;
            ++result.solvedRows;
            sumTranslation2 += translation*translation;
            sumOrientation2 += orientation*orientation;
            sumResidual2 += solved.rmsCableResidualMm*solved.rmsCableResidualMm;
            result.translationMaximumMm = std::max(result.translationMaximumMm, translation);
            result.orientationMaximumDeg = std::max(result.orientationMaximumDeg, orientation);
            result.cableResidualMaximumMm = std::max(result.cableResidualMaximumMm, solved.maximumCableResidualMm);
        }else{
            ++result.failedRows;
        }
        stream << ',' << translation << ',' << orientation << ','
               << solved.rmsCableResidualMm << ',' << solved.maximumCableResidualMm << ','
               << (solved.success ? 1 : 0) << ',' << solved.terminationType << ','
               << solved.iterationCount;

        const bool shadowDue = shadowConfigured && solved.success &&
                solved.pose.size() >= kPoseCount &&
                (previousShadowTraceUs == 0 ||
                 row.traceUs - previousShadowTraceUs >= shadowPeriodUs);
        bool shadowValid = false;
        qint64 shadowCalculationUs = 0;
        double shadowResidual = std::numeric_limits<double>::quiet_NaN();
        double shadowMargin = std::numeric_limits<double>::quiet_NaN();
        RedundantTorqueAllocator::Vector8d shadowTension =
                RedundantTorqueAllocator::Vector8d::Constant(
                    std::numeric_limits<double>::quiet_NaN());
        RedundantTorqueAllocator::Vector8d shadowTorque = shadowTension;
        QString shadowError;
        if(shadowDue){
            ++result.shadowEvaluatedRows;
            ForceInteractionTensionShadowInput shadowInput;
            for(int i = 0; i < kPoseCount; ++i){
                shadowInput.desired.pose[i] = desired.pose[i];
                shadowInput.desired.twist[i] = desired.twist[i];
                shadowInput.desired.acceleration[i] = desired.acceleration[i];
                shadowInput.observed.pose[i] = i < 3 ?
                            solved.pose[i] * 1.0e-3 : solved.pose[i];
            }
            shadowInput.desired.poseValid = true;
            shadowInput.desired.twistValid = true;
            shadowInput.desired.accelerationValid = true;
            shadowInput.observed.poseValid = true;
            shadowInput.observed.twistValid = true;
            shadowInput.dtS = previousShadowTraceUs > 0 ?
                        double(row.traceUs - previousShadowTraceUs) * 1.0e-6 :
                        request.tensionShadow.outerPeriodS;
            if(previousObserved.poseValid && shadowInput.dtS > 0.0){
                for(int i = 0; i < kPoseCount; ++i){
                    double delta = shadowInput.observed.pose[i] -
                            previousObserved.pose[i];
                    if(i >= 3) delta = wrappedAngle(delta);
                    shadowInput.observed.twist[i] = delta / shadowInput.dtS;
                }
            }
            QElapsedTimer shadowTimer;
            shadowTimer.start();
            const ForceInteractionTensionShadowResult shadowResult =
                    shadow.evaluate(shadowInput);
            shadowCalculationUs = shadowTimer.nsecsElapsed() / 1000;
            result.shadowMaximumCalculationUs = std::max(
                        result.shadowMaximumCalculationUs,
                        shadowCalculationUs);
            if(shadowResult.valid){
                shadowValid = true;
                ++result.shadowValidRows;
                shadowResidual = shadowResult.allocation.maximumGeneralizedControlResidual;
                shadowMargin = shadowResult.allocation.minimumTensionMargin;
                shadowTension = shadowResult.allocation.predictedTension;
                shadowTorque = shadowResult.allocation.hardwareMotorTorque;
                result.shadowMinimumTensionMarginN = std::min(
                            result.shadowMinimumTensionMarginN,
                            shadowResult.allocation.minimumTensionMargin);
                result.shadowMaximumWrenchResidual = std::max(
                            result.shadowMaximumWrenchResidual,
                            shadowResult.allocation.maximumGeneralizedControlResidual);
            }
            else{
                ++result.shadowInfeasibleRows;
                shadowError = shadowResult.errorMessage;
            }
            previousObserved = shadowInput.observed;
            previousShadowTraceUs = row.traceUs;
        }
        stream << ',' << (shadowDue ? 1 : 0)
               << ',' << (shadowValid ? 1 : 0)
               << ',' << shadowCalculationUs
               << ',' << shadowResidual
               << ',' << shadowMargin;
        for(int i=0;i<kAxisCount;++i) stream << ',' << shadowTension[i];
        for(int i=0;i<kAxisCount;++i) stream << ',' << shadowTorque[i];
        stream << ',' << shadowError.replace(',', QStringLiteral("；")) << '\n';
    }
    output.close();
    if(result.solvedRows){
        result.translationRmsMm = std::sqrt(sumTranslation2 / double(result.solvedRows));
        result.orientationRmsDeg = std::sqrt(sumOrientation2 / double(result.solvedRows));
        result.cableResidualRmsMm = std::sqrt(sumResidual2 / double(result.solvedRows));
    }
    result.completed = result.solvedRows > 0 && result.nonMonotonicTraceRows == 0;
    result.summary = QStringLiteral(
                "六维力交互运行后运动学验算完成：同帧Trace有效/记录=%1/%2，正解成功/失败=%3/%4，"
                "末端平移RMS/最大=%5/%6 mm，姿态RMS/最大=%7/%8°，绳长残差RMS/最大=%9/%10 mm，"
                "Trace非递增=%11。结果=%12")
            .arg(result.validTraceRows).arg(result.dataRows)
            .arg(result.solvedRows).arg(result.failedRows)
            .arg(result.translationRmsMm,0,'f',4).arg(result.translationMaximumMm,0,'f',4)
            .arg(result.orientationRmsDeg,0,'f',5).arg(result.orientationMaximumDeg,0,'f',5)
            .arg(result.cableResidualRmsMm,0,'f',6).arg(result.cableResidualMaximumMm,0,'f',6)
            .arg(result.nonMonotonicTraceRows)
            .arg(QDir::toNativeSeparators(result.resultCsvPath));
    if(request.tensionShadowEnabled){
        if(!shadowConfigured){
            result.summary += QStringLiteral(
                        "\nM1在线张力离线影子未运行：冻结配置无效（%1）。")
                    .arg(shadowConfigurationError);
        }
        else{
            result.summary += QStringLiteral(
                        "\nM1在线张力离线影子（仅平动、25 ms外环、无硬件下发）："
                        "计算/有效/不可行=%1/%2/%3，最大计算=%4 us，"
                        "最小张力分配余量=%5 N，最大力旋量残差=%6。")
                    .arg(result.shadowEvaluatedRows)
                    .arg(result.shadowValidRows)
                    .arg(result.shadowInfeasibleRows)
                    .arg(result.shadowMaximumCalculationUs)
                    .arg(result.shadowMinimumTensionMarginN, 0, 'g', 8)
                    .arg(result.shadowMaximumWrenchResidual, 0, 'g', 8);
        }
    }
    return result;
}

ForceInteractionKinematicLogAnalysisWorker::ForceInteractionKinematicLogAnalysisWorker(
        const ForceInteractionKinematicLogAnalysisRequest& request, QObject* parent)
    : QThread(parent), request_(request) {}

ForceInteractionKinematicLogAnalysisResult
ForceInteractionKinematicLogAnalysisWorker::result() const { return result_; }

void ForceInteractionKinematicLogAnalysisWorker::run()
{
    result_ = ForceInteractionKinematicLogAnalyzer::analyze(request_);
}
