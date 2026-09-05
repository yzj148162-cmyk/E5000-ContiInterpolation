#include "forceinteractionrunrecorder.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>

#include <algorithm>
#include <utility>

namespace {

QString csvSafe(QString value)
{
    value.replace('"', QStringLiteral("\"\""));
    return QStringLiteral("\"") + value + QStringLiteral("\"");
}

template<std::size_t Size>
void writeArray(QTextStream& stream, const std::array<double, Size>& values)
{
    for(double value : values){
        stream << ',' << value;
    }
}

void writeGroupHeader(QTextStream& stream, const char* name, int count)
{
    for(int index = 0; index < count; ++index){
        stream << ',' << name << '_' << index;
    }
}

void writeWorkspacePointHeader(QTextStream& stream)
{
    static const char* const axes[] = {"x", "y", "z"};
    for(int point = 0; point < kForceInteractionCableCount; ++point){
        for(const char* axis : axes){
            stream << ",workspace_point_" << point << '_' << axis << "_mm";
        }
    }
}

void writeWorkspacePoints(
        QTextStream& stream,
        const std::array<std::array<double, 3>,
                         kForceInteractionCableCount>& points)
{
    for(const auto& point : points){
        writeArray(stream, point);
    }
}

} // namespace

ForceInteractionRunRecorder::ForceInteractionRunRecorder(QObject* parent)
    : QThread(parent)
{
    setObjectName(QStringLiteral("ForceInteractionRunRecorder"));
}

ForceInteractionRunRecorder::~ForceInteractionRunRecorder()
{
    finishAndWait();
}

bool ForceInteractionRunRecorder::begin(
        const QString& directory,
        const ForceInteractionRunMetadata& metadata,
        QString* outputPath,
        QString* errorMessage)
{
    finishAndWait();
    while(ready_.tryAcquire(1)){
    }

    const QString resolvedDirectory = directory.trimmed().isEmpty() ?
                QDir(QCoreApplication::applicationDirPath())
                    .filePath(QStringLiteral("force_interaction_records")) :
                directory;
    if(!QDir().mkpath(resolvedDirectory)){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法创建六维力交互记录目录：%1")
                    .arg(resolvedDirectory);
        }
        return false;
    }

    QString stageToken = metadata.stage.trimmed().toLower();
    if(stageToken.isEmpty()){
        stageToken = QStringLiteral("unknown");
    }
    stageToken.replace(' ', '_');
    filePath_ = QDir(resolvedDirectory).filePath(
                QStringLiteral("force_interaction_%1_%2.csv")
                .arg(stageToken,
                     QDateTime::currentDateTime().toString(
                         QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    metadata_ = metadata;
    openError_.clear();
    writerError_.clear();
    stopRequested_.store(false);
    openSucceeded_.store(false);
    accepted_.store(0);
    written_.store(0);
    dropped_.store(0);
    {
        QMutexLocker locker(&queueMutex_);
        queue_.clear();
        queue_.resize(kMaximumQueuedRecords);
        queueHead_ = 0;
        queuedRecordCount_ = 0;
    }

    start(QThread::LowPriority);
    if(!ready_.tryAcquire(1, 1500)){
        requestFinish();
        wait();
        if(errorMessage){
            *errorMessage = QStringLiteral("六维力交互记录线程启动超时");
        }
        return false;
    }
    if(!openSucceeded_.load()){
        wait();
        if(errorMessage){
            *errorMessage = openError_;
        }
        return false;
    }
    if(outputPath){
        *outputPath = filePath_;
    }
    return true;
}

void ForceInteractionRunRecorder::tryAppend(
        const ForceInteractionRunRecord& record)
{
    if(!isRunning() || stopRequested_.load()){
        return;
    }
    if(!queueMutex_.tryLock()){
        dropped_.fetch_add(1);
        return;
    }
    if(queuedRecordCount_ >= kMaximumQueuedRecords){
        dropped_.fetch_add(1);
        queueMutex_.unlock();
        return;
    }
    const std::size_t writeIndex =
            (queueHead_ + queuedRecordCount_) % kMaximumQueuedRecords;
    queue_[writeIndex] = record;
    ++queuedRecordCount_;
    accepted_.fetch_add(1);
    queueReady_.wakeOne();
    queueMutex_.unlock();
}

void ForceInteractionRunRecorder::requestFinish()
{
    stopRequested_.store(true);
    queueReady_.wakeAll();
}

void ForceInteractionRunRecorder::finishAndWait()
{
    if(!isRunning()){
        return;
    }
    requestFinish();
    wait();
}

QString ForceInteractionRunRecorder::outputPath() const
{
    return filePath_;
}

QString ForceInteractionRunRecorder::writerError() const
{
    return writerError_;
}

quint64 ForceInteractionRunRecorder::acceptedCount() const
{
    return accepted_.load();
}

quint64 ForceInteractionRunRecorder::writtenCount() const
{
    return written_.load();
}

quint64 ForceInteractionRunRecorder::droppedCount() const
{
    return dropped_.load();
}

void ForceInteractionRunRecorder::run()
{
    QFile file(filePath_);
    openSucceeded_.store(file.open(QIODevice::WriteOnly | QIODevice::Text));
    if(!openSucceeded_.load()){
        openError_ = QStringLiteral("无法打开六维力交互记录文件：%1")
                .arg(file.errorString());
        ready_.release();
        return;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(9);
    stream << "# schema=force_interaction_run_v3\n"
           << "# created="
           << QDateTime::currentDateTime().toString(Qt::ISODateWithMs) << '\n'
           << "# stage=" << csvSafe(metadata_.stage)
           << ",source=" << csvSafe(metadata_.sourceName)
           << ",machine_template=" << csvSafe(metadata_.machineTemplateName)
           << '\n'
           << "# control_period_s=" << metadata_.controlPeriodS
           << ",planned_duration_s=" << metadata_.plannedDurationS << '\n'
           << "# availability_mask:1=sensor_wrench,2=platform_wrench,4=desired_state,"
              "8=cable_kinematics,16=forward_kinematics,32=axis_reference,"
              "64=axis_command,128=axis_trace,256=timing\n";

    stream << "step_index,elapsed_s,availability_mask,host_monotonic_us,"
              "trace_sequence,trace_time_us,trace_valid";
    writeGroupHeader(stream, "sensor_wrench", kForceInteractionDofCount);
    writeGroupHeader(stream, "platform_wrench", kForceInteractionDofCount);
    writeGroupHeader(stream, "desired_pose_si", kForceInteractionDofCount);
    writeGroupHeader(stream, "desired_twist_si", kForceInteractionDofCount);
    writeGroupHeader(stream, "desired_acceleration_si", kForceInteractionDofCount);
    writeGroupHeader(stream, "cable_length_mm", kForceInteractionCableCount);
    writeGroupHeader(stream, "relative_motor_theta_rad", kForceInteractionCableCount);
    writeGroupHeader(stream, "forward_pose_mm_rad", kForceInteractionDofCount);
    stream << ",translation_roundtrip_error_mm,orientation_roundtrip_error_deg,"
              "maximum_cable_residual_mm,newmark_iterations,newmark_residual,"
              "pose_bounds_violation,roundtrip_tolerance_violation,"
              "interaction_segment,controlled_stop_cause,workspace_action,workspace_minimum_clearance_mm,workspace_limiting_clearance_mm,"
              "workspace_outward_speed_mm_s,workspace_outward_acceleration_mm_s2,"
              "workspace_pure_stopping_distance_mm,workspace_trigger_distance_mm,"
              "workspace_limiting_point,workspace_limiting_axis,workspace_limiting_upper_face";
    writeWorkspacePointHeader(stream);
    writeGroupHeader(stream, "axis_reference_position", kForceInteractionCableCount);
    writeGroupHeader(stream, "axis_reference_velocity", kForceInteractionCableCount);
    writeGroupHeader(stream, "axis_pid_correction_velocity", kForceInteractionCableCount);
    writeGroupHeader(stream, "axis_command_velocity", kForceInteractionCableCount);
    writeGroupHeader(stream, "axis_trace_position", kForceInteractionCableCount);
    writeGroupHeader(stream, "axis_trace_velocity", kForceInteractionCableCount);
    stream << ",calculation_us,hardware_api_us,full_cycle_us\n";
    ready_.release();

    for(;;){
        std::vector<ForceInteractionRunRecord> batch;
        batch.reserve(128);
        queueMutex_.lock();
        if(queuedRecordCount_ == 0 && !stopRequested_.load()){
            queueReady_.wait(&queueMutex_, 100);
        }
        const std::size_t count = std::min<std::size_t>(queuedRecordCount_, 128);
        for(std::size_t index = 0; index < count; ++index){
            batch.push_back(std::move(queue_[queueHead_]));
            queue_[queueHead_] = ForceInteractionRunRecord{};
            queueHead_ = (queueHead_ + 1) % kMaximumQueuedRecords;
            --queuedRecordCount_;
        }
        const bool finished = stopRequested_.load() && queuedRecordCount_ == 0;
        queueMutex_.unlock();

        for(const ForceInteractionRunRecord& record : batch){
            stream << record.stepIndex << ',' << record.elapsedS << ','
                   << record.availabilityMask << ','
                   << record.stamp.hostMonotonicTimeUs << ','
                   << record.stamp.traceSequence << ','
                   << record.stamp.traceTimeUs << ','
                   << (record.stamp.traceValid ? 1 : 0);
            writeArray(stream, record.sensorWrench);
            writeArray(stream, record.platformWrench);
            writeArray(stream, record.desiredState.pose);
            writeArray(stream, record.desiredState.twist);
            writeArray(stream, record.desiredState.acceleration);
            writeArray(stream, record.cableLengthMm);
            writeArray(stream, record.relativeMotorThetaRad);
            writeArray(stream, record.forwardPoseMmRad);
            stream << ',' << record.translationRoundTripErrorMm
                   << ',' << record.orientationRoundTripErrorDeg
                   << ',' << record.maximumCableResidualMm
                   << ',' << record.newmarkIterations
                   << ',' << record.newmarkResidual
                   << ',' << (record.poseBoundsViolation ? 1 : 0)
                   << ',' << (record.roundTripToleranceViolation ? 1 : 0)
                   << ',' << record.interactionSegment
                   << ',' << record.controlledStopCause
                   << ',' << record.workspaceAction
                   << ',' << record.workspaceMinimumClearanceMm
                   << ',' << record.workspaceLimitingClearanceMm
                   << ',' << record.workspaceOutwardSpeedMmPerSec
                   << ',' << record.workspaceOutwardAccelerationMmPerSec2
                   << ',' << record.workspacePureStoppingDistanceMm
                   << ',' << record.workspaceTriggerDistanceMm
                   << ',' << record.workspaceLimitingPoint
                   << ',' << record.workspaceLimitingAxis
                   << ',' << (record.workspaceLimitingUpperFace ? 1 : 0);
            writeWorkspacePoints(stream, record.workspacePointGlobalMm);
            writeArray(stream, record.axisReferencePosition);
            writeArray(stream, record.axisReferenceVelocity);
            writeArray(stream, record.axisPidCorrectionVelocity);
            writeArray(stream, record.axisCommandVelocity);
            writeArray(stream, record.axisTracePosition);
            writeArray(stream, record.axisTraceVelocity);
            stream << ',' << record.calculationDurationUs
                   << ',' << record.hardwareApiDurationUs
                   << ',' << record.fullCycleDurationUs << '\n';
            written_.fetch_add(1);
        }
        if(stream.status() != QTextStream::Ok){
            writerError_ = QStringLiteral("六维力交互CSV写入失败");
            break;
        }
        if(finished){
            break;
        }
    }
    stream.flush();
    if(file.error() != QFileDevice::NoError && writerError_.isEmpty()){
        writerError_ = QStringLiteral("六维力交互CSV刷新失败：%1")
                .arg(file.errorString());
    }
    file.close();
}
