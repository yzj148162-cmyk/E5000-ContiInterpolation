#include "sessionrecorder.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "csvexportutils.h"
#include "outputpathvalidator.h"
#include "runtimejsoncodec.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QTextStream>
#include <QVector>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

using namespace OutputPathValidator;
using namespace RuntimeJsonCodec;
using namespace SessionRecorder;

namespace {

constexpr int kBarycenterCableCount = 8;
constexpr int kDiagnosticRawWritePumpLines = 4096;

template <typename T>
T* findOptionalUiObject(const QWidget* root, const char* objectName)
{
    return root ? root->findChild<T*>(QString::fromLatin1(objectName)) : nullptr;
}

QString formatIsoDateTimeUs(qint64 wallClockUs)
{
    if(wallClockUs <= 0){
        return QStringLiteral("无有效时间戳");
    }
    const qint64 wallClockMs = wallClockUs / 1000;
    const qint64 usInSecond = ((wallClockUs % 1000000) + 1000000) % 1000000;
    return QStringLiteral("%1.%2")
            .arg(QDateTime::fromMSecsSinceEpoch(wallClockMs).toString(QStringLiteral("yyyy-MM-ddTHH:mm:ss")))
            .arg(usInSecond, 6, 10, QLatin1Char('0'));
}

bool hasFiniteValues(const std::vector<double>& values, int minSize = 0)
{
    if(static_cast<int>(values.size()) < minSize){
        return false;
    }
    for(double value : values){
        if(!std::isfinite(value)){
            return false;
        }
    }
    return true;
}

QString jsonValueToReadableText(const QJsonValue& value)
{
    if(value.isArray()){
        return QString::fromUtf8(QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
    }
    if(value.isObject()){
        return QString::fromUtf8(QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
    }
    if(value.isString()){
        return value.toString();
    }
    if(value.isBool()){
        return value.toBool() ? QStringLiteral("是") : QStringLiteral("否");
    }
    if(value.isDouble()){
        return QString::number(value.toDouble(), 'g', 15);
    }
    if(value.isNull()){
        return QStringLiteral("空");
    }
    return QStringLiteral("未定义");
}

QString sessionRecordingPoseSourceText(const QString& source)
{
    if(source == QStringLiteral("motive")){
        return QStringLiteral("Motive动捕");
    }
    if(source == QStringLiteral("forward_kinematics")){
        return QStringLiteral("正运动学");
    }
    if(source == QStringLiteral("planned_trajectory")){
        return QStringLiteral("规划轨迹");
    }
    if(source == QStringLiteral("unavailable")){
        return QStringLiteral("不可用");
    }
    return source.isEmpty() ? QStringLiteral("未知") : source;
}

QString formatSessionRecordingTimingText(const QJsonObject& timingObject)
{
    return QStringLiteral("控制循环累计次数=%1, 控制循环累计间隔(us)=%2, 最新控制循环间隔(us)=%3, 传感器累计次数=%4, 传感器累计间隔(us)=%5, 最新传感器间隔(us)=%6")
            .arg(timingObject.value(QStringLiteral("control_loop_tick_count")).toVariant().toLongLong())
            .arg(timingObject.value(QStringLiteral("control_loop_interval_sum_us")).toVariant().toLongLong())
            .arg(timingObject.value(QStringLiteral("latest_control_loop_interval_us")).toVariant().toLongLong())
            .arg(timingObject.value(QStringLiteral("sensor_frame_count")).toVariant().toLongLong())
            .arg(timingObject.value(QStringLiteral("sensor_frame_interval_sum_us")).toVariant().toLongLong())
            .arg(timingObject.value(QStringLiteral("latest_sensor_frame_interval_us")).toVariant().toLongLong());
}

} // namespace

void MainWindow::initializeSessionRecordingUi()
{
    if(!ui){
        return;
    }
    if(ui->sessionRecordingGroupBox){
        ui->sessionRecordingGroupBox->setTitle(QStringLiteral("历史数据统一存储 / 会话导出"));
        if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
            ui->sessionRecordingGroupBox->setToolTip(
                        QStringLiteral("运行诊断记录与导出已由性能开关停用。"));
        }
    }
    if(ui->sessionRecordingHintLabel){
        ui->sessionRecordingHintLabel->setText(
                    QStringLiteral("按“开始记录”后统一采集PVT位置控制指令、绳索位移、张力传感器和运动轨迹点；按“结束记录”只停止采集并保留内存数据，需另行点击“导出会话记录”输入或浏览选择目录导出。"));
        ui->sessionRecordingHintLabel->setWordWrap(true);
    }
    if(ui->sessionRecordingStatusTitleLabel){
        ui->sessionRecordingStatusTitleLabel->setText(QStringLiteral("记录状态"));
    }
    if(ui->sessionRecordingExportTitleLabel){
        ui->sessionRecordingExportTitleLabel->setText(QStringLiteral("最近导出"));
    }
    if(ui->sessionRecordingStartButton){
        ui->sessionRecordingStartButton->setText(QStringLiteral("开始记录"));
        connect(ui->sessionRecordingStartButton,
                &QPushButton::clicked,
                this,
                [this](){
                    startSessionRecording();
                });
    }
    if(ui->sessionRecordingStopButton){
        ui->sessionRecordingStopButton->setText(QStringLiteral("结束记录"));
        connect(ui->sessionRecordingStopButton,
                &QPushButton::clicked,
                this,
                [this](){
                    stopSessionRecording(true);
                });
    }
    sessionRecordingExportButton =
            findOptionalUiObject<QPushButton>(this, "sessionRecordingExportButton");
    if(!sessionRecordingExportButton &&
            ui->sessionRecordingGroupBox &&
            ui->gridLayoutSessionRecording){
        sessionRecordingExportButton = new QPushButton(ui->sessionRecordingGroupBox);
        sessionRecordingExportButton->setObjectName(QStringLiteral("sessionRecordingExportButton"));
        ui->gridLayoutSessionRecording->addWidget(sessionRecordingExportButton, 3, 0, 1, 1);
    }
    if(sessionRecordingExportButton){
        sessionRecordingExportButton->setText(QStringLiteral("导出会话记录"));
        sessionRecordingExportButton->setToolTip(
                    QStringLiteral("输入或浏览选择目录并导出当前已停止的会话记录；没有数据或路径不可写时只提示原因，内存数据保留。"));
        connect(sessionRecordingExportButton,
                &QPushButton::clicked,
                this,
                [this](){
                    writeSessionRecordingExport(nullptr, true);
                });
    }

    refreshSessionRecordingUi();
}

void MainWindow::refreshSessionRecordingUi()
{
    if(!ui){
        return;
    }

    auto setLabelTextIfChanged = [](QLabel* label, const QString& text){
        if(label && label->text() != text){
            label->setText(text);
        }
    };
    auto setEnabledIfChanged = [](QWidget* widget, bool enabled){
        if(widget && widget->isEnabled() != enabled){
            widget->setEnabled(enabled);
        }
    };
    auto setToolTipIfChanged = [](QWidget* widget, const QString& text){
        if(widget && widget->toolTip() != text){
            widget->setToolTip(text);
        }
    };

    QString statusText;
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        statusText = QStringLiteral("已由性能开关停用：不记录、不缓存、不导出");
    }
    else if(sessionRecordingState.active){
        const qint64 elapsedMs = std::max<qint64>(0, QDateTime::currentMSecsSinceEpoch() - sessionRecordingState.startedAtMs);
        statusText = QStringLiteral("记录中：已采样 %1 条，PVT表 %2 个，控制周期记录 %3 个，持续 %4 ms")
                .arg(static_cast<int>(sessionRecordingState.samples.size()))
                .arg(static_cast<int>(sessionRecordingState.pvtPositionCommandTables.size()))
                .arg(static_cast<int>(sessionRecordingState.pvtControlCycleRecords.size()))
                .arg(elapsedMs);
    }
    else if(sessionRecordingState.startedAtMs > 0){
        const qint64 durationMs = std::max<qint64>(0, sessionRecordingState.endedAtMs - sessionRecordingState.startedAtMs);
        statusText = QStringLiteral("已结束：共导出 %1 条样本、%2 个PVT表、%3 个控制周期记录，持续 %4 ms")
                .arg(static_cast<int>(sessionRecordingState.samples.size()))
                .arg(static_cast<int>(sessionRecordingState.pvtPositionCommandTables.size()))
                .arg(static_cast<int>(sessionRecordingState.pvtControlCycleRecords.size()))
                .arg(durationMs);
    }
    else{
        statusText = QStringLiteral("尚未开始记录");
    }

    QString exportText = QStringLiteral("尚无导出文件");
    if(!sessionRecordingState.lastExportPath.isEmpty()){
        exportText = QDir::toNativeSeparators(sessionRecordingState.lastExportPath);
    }

    setLabelTextIfChanged(ui->sessionRecordingStatusValueLabel, statusText);
    setLabelTextIfChanged(ui->sessionRecordingExportValueLabel, exportText);
    setEnabledIfChanged(ui->sessionRecordingStartButton,
                        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled &&
                        !sessionRecordingState.active);
    setEnabledIfChanged(ui->sessionRecordingStopButton,
                        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled &&
                        sessionRecordingState.active);
    setEnabledIfChanged(sessionRecordingExportButton,
                        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled);
}

bool MainWindow::startSessionRecording()
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        displayInfo("运行诊断记录与导出已由性能开关停用", "warning");
        return false;
    }
    if(sessionRecordingState.active){
        displayInfo("当前已经在记录会话数据", "warning");
        return false;
    }

    sessionRecordingState.active = true;
    sessionRecordingState.startedAtMs = QDateTime::currentMSecsSinceEpoch();
    sessionRecordingState.endedAtMs = 0;
    sessionRecordingState.samples = std::vector<SessionRecordingSample>();
    sessionRecordingState.samples.reserve(6000);
    sessionRecordingState.pvtPositionCommandTables.clear();
    sessionRecordingState.pvtControlCycleRecords.clear();
    if(kEnableSessionRecordPvtControlCycleDiagnostics){
        seedSessionRecordingPvtControlCycleRecordsFromLastSuccess();
    }
    hardwareInterface.setDiagnosticRawHistoryFullRecordingEnabled(true);
    if(kEnableSessionRecordMotorEncoderUnitSampling){
        hardwareInterface.startSessionEncoderUnitSampling(500);
    }
    if(controlWorker){
        controlWorker->setDiagnosticRawHistoryFullRecordingEnabled(true);
    }
    lastSessionRecordingUiRefreshMs = -1;
    refreshSessionRecordingUi();
    displayInfo("已开始记录会话数据，将按时间戳统一采集PVT位置控制指令、绳索位移、张力和轨迹点");
    return true;
}

bool MainWindow::stopSessionRecording(bool announce)
{
    if(!sessionRecordingState.active){
        if(announce){
            displayInfo("当前没有正在进行的会话记录", "warning");
        }
        return false;
    }

    sessionRecordingState.active = false;
    if(kEnableSessionRecordMotorEncoderUnitSampling){
        hardwareInterface.stopSessionEncoderUnitSampling();
    }
    sessionRecordingState.endedAtMs = QDateTime::currentMSecsSinceEpoch();
    if(kEnableSessionRecordPvtControlCycleDiagnostics){
        refreshPvtTraceStartDelayFromHardwareHistory();
    }
    hardwareInterface.setDiagnosticRawHistoryFullRecordingEnabled(false);
    if(controlWorker){
        controlWorker->setDiagnosticRawHistoryFullRecordingEnabled(false);
    }
    refreshSessionRecordingUi();
    if(announce){
        displayInfo("会话记录已结束，内存数据已保留；请点击“导出会话记录”并输入或浏览选择导出目录");
    }
    return true;
}

void MainWindow::captureSessionRecordingSample(const ControlWorker::Snapshot& snapshot)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled ||
            !sessionRecordingState.active){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const qint64 relativeMs =
            std::max<qint64>(0, nowMs - sessionRecordingState.startedAtMs);
    qint64 intervalMsSincePrevious = 0;
    if(!sessionRecordingState.samples.empty()){
        const qint64 previousMs =
                sessionRecordingState.samples.back().capturedAtMs;
        if(previousMs > 0){
            intervalMsSincePrevious = std::max<qint64>(0, nowMs - previousMs);
        }
    }
    SessionRecordingSample sample;
    sample.capturedAtMs = nowMs;
    sample.relativeMs = relativeMs;
    sample.intervalMsSincePrevious = intervalMsSincePrevious;
    sample.controlSequence = snapshot.sequence;
    sample.motorAbsPos = snapshot.motorAbsPos;
    sample.motorRelRawPos = snapshot.motorRelRawPos;
    sample.motorVel = snapshot.motorVel;
    sample.motorTorqueNm = snapshot.motorTorqueNm;
    sample.motorCommandNm = snapshot.motorCommand;
    sample.forceSensorValue = snapshot.forceSensorValue;
    sample.expectedForce = snapshot.expectedForce;
    sample.timingDiagnostics = snapshot.timingDiagnostics;

    std::vector<std::vector<double>> cableDisplacement;
    const bool hasCableDisplacement = buildCurrentCableDisplacements(cableDisplacement);
    sample.cableDisplacementAvailable = hasCableDisplacement;
    if(hasCableDisplacement){
        sample.cableDisplacement = std::move(cableDisplacement);
    }

    std::vector<std::vector<double>> cableLength;
    const bool hasCableLength = buildCurrentCableLengths(cableLength);
    sample.cableLengthAvailable = hasCableLength;
    if(hasCableLength){
        sample.cableLength = std::move(cableLength);
    }

    std::vector<double> trajectoryPoint;
    QString poseSource = QStringLiteral("unavailable");
    if(currentEstimatedEndPose(trajectoryPoint, 1000) && trajectoryPoint.size() >= 6){
        poseSource = activePoseTrajectoryDisplayPointIndex >= 0 ?
                    QStringLiteral("planned_trajectory") :
                    ui->devUseCamForActPose->isChecked() ?
                    QStringLiteral("motive") :
                    QStringLiteral("forward_kinematics");
        sample.trajectoryPointAvailable = true;
        sample.trajectoryPoint = std::move(trajectoryPoint);
    }
    sample.trajectoryPointSource = poseSource;

    sessionRecordingState.samples.push_back(std::move(sample));
    if(lastSessionRecordingUiRefreshMs < 0 || nowMs - lastSessionRecordingUiRefreshMs >= 250){
        lastSessionRecordingUiRefreshMs = nowMs;
        refreshSessionRecordingUi();
    }
}

void MainWindow::captureSessionRecordingPvtPositionCommand(
        const std::vector<int>& motorIndex,
        const std::vector<std::vector<double>>& positionUnit,
        const std::vector<double>& timeStamp,
        const QString& source)
{
    startMotorControlInputVisualizationFromPvtCommand(motorIndex,
                                                      positionUnit,
                                                      timeStamp);
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        return;
    }
    if(motorIndex.empty() ||
            positionUnit.empty() ||
            timeStamp.empty() ||
            positionUnit.size() != timeStamp.size()){
        return;
    }

    const int axisCount = static_cast<int>(motorIndex.size());
    for(int pointIndex = 0; pointIndex < static_cast<int>(positionUnit.size()); ++pointIndex){
        if(static_cast<int>(positionUnit[pointIndex].size()) != axisCount ||
                !hasFiniteValues(positionUnit[pointIndex]) ||
                pointIndex >= static_cast<int>(timeStamp.size()) ||
                !std::isfinite(timeStamp[pointIndex]) ||
                (pointIndex > 0 && timeStamp[pointIndex] < timeStamp[pointIndex - 1])){
            return;
        }
    }

    if(sessionRecordingState.active){
        SessionRecordingPvtPositionCommandTable table;
        table.capturedAtMs = QDateTime::currentMSecsSinceEpoch();
        table.source = source.isEmpty() ? QStringLiteral("PVT位置控制指令") : source;
        table.motorIndex = motorIndex;
        table.timeStamp = timeStamp;
        table.positionUnit = positionUnit;
        sessionRecordingState.pvtPositionCommandTables.push_back(table);
        if(kEnableSessionRecordPvtControlCycleDiagnostics){
            attachLatestPvtUploadTimingToSessionRecordingCycle(
                        static_cast<int>(timeStamp.size()),
                        static_cast<int>(motorIndex.size()));
        }
        refreshSessionRecordingUi();
    }

    if(kEnableSessionRecordPvtControlCycleDiagnostics){
        updateLastSuccessfulPvtControlCycleCache(
                    static_cast<int>(timeStamp.size()),
                    static_cast<int>(motorIndex.size()));
    }
}

void MainWindow::captureSessionRecordingPvtControlCycleTiming(
        int pvtPointCount,
        int pvtAxisCount,
        double pvtGenerationElapsedMs,
        int sourceStartPointIndex)
{
    if(!kEnableSessionRecordPvtControlCycleDiagnostics){
        return;
    }
    if(pvtPointCount <= 0){
        return;
    }

    SessionRecordingPvtControlCycleRecord record;
    record.capturedAtMs = QDateTime::currentMSecsSinceEpoch();
    record.source = QStringLiteral("位控PVT轨迹生成");
    record.sourceStartPointIndex = std::max(0, sourceStartPointIndex);
    record.pointCount = pvtPointCount;
    record.axisCount = pvtAxisCount;
    record.pvtGenerationElapsedUs =
            pvtGenerationElapsedMs >= 0.0 ?
                static_cast<qint64>(std::llround(pvtGenerationElapsedMs * 1000.0)) :
                -1;

    const std::vector<PositionSimulationModel::TrajectoryPointTimingSample> cableTimingSamples =
            positionSimulationModel ?
                positionSimulationModel->getTrajectoryPointTimingSamples() :
                std::vector<PositionSimulationModel::TrajectoryPointTimingSample>();
    record.points.reserve(std::max(0, pvtPointCount));
    for(int pointIndex = 0; pointIndex < pvtPointCount; ++pointIndex){
        const int sourcePointIndex = record.sourceStartPointIndex + pointIndex;
        SessionRecordingPvtControlCyclePoint point;
        point.pointIndex = pointIndex;
        if(sourcePointIndex < static_cast<int>(cableTimingSamples.size())){
            const PositionSimulationModel::TrajectoryPointTimingSample& sample =
                    cableTimingSamples[sourcePointIndex];
            point.trajectoryTimeSec = sample.trajectoryTimeSec;
            point.cableLengthCalculationUs = sample.cableLengthCalculationUs;
        }
        else if(sourcePointIndex < static_cast<int>(plannedPoseForceTimeStamp.size())){
            point.trajectoryTimeSec = plannedPoseForceTimeStamp[sourcePointIndex];
        }
        if(sourcePointIndex < static_cast<int>(plannedPoseBarycenterSolveUs.size())){
            point.barycenterSolveUs = plannedPoseBarycenterSolveUs[sourcePointIndex];
        }
        record.points.push_back(point);
    }

    pendingPvtControlCycleRecord = record;
    pendingPvtControlCycleRecordValid = true;

    if(sessionRecordingState.active){
        sessionRecordingState.pvtControlCycleRecords.push_back(record);
        refreshSessionRecordingUi();
    }
}

void MainWindow::seedSessionRecordingPvtControlCycleRecordsFromLastSuccess()
{
    if(lastSuccessfulPvtControlCycleRecords.empty()){
        return;
    }

    sessionRecordingState.pvtControlCycleRecords =
            lastSuccessfulPvtControlCycleRecords;
    for(SessionRecordingPvtControlCycleRecord& record :
        sessionRecordingState.pvtControlCycleRecords){
        if(!record.source.startsWith(QStringLiteral("上一次成功运动缓存："))){
            record.source = QStringLiteral("上一次成功运动缓存：%1")
                    .arg(record.source.isEmpty() ?
                             QStringLiteral("位控PVT轨迹生成") :
                             record.source);
        }
    }
}

bool MainWindow::attachLatestPvtUploadTimingToRecord(
        SessionRecordingPvtControlCycleRecord& record,
        int pvtPointCount,
        int pvtAxisCount,
        qint64 startMs,
        qint64 endMs)
{
    const QVector<HardwareInterface::PvtTableUploadTimingSample> uploadSamples =
            hardwareInterface.pvtTableUploadTimingHistory(startMs, endMs);
    if(uploadSamples.isEmpty()){
        return false;
    }

    const int effectivePointCount = std::min(pvtPointCount, 5000);
    const HardwareInterface::PvtTableUploadTimingSample* matchedUpload = nullptr;
    for(int sampleIndex = uploadSamples.size() - 1; sampleIndex >= 0; --sampleIndex){
        const HardwareInterface::PvtTableUploadTimingSample& sample =
                uploadSamples[sampleIndex];
        if(record.pvtUploadMonotonicUs > 0){
            if(sample.monotonicUs == record.pvtUploadMonotonicUs){
                matchedUpload = &sample;
                break;
            }
            continue;
        }
        if(sample.pointCount == effectivePointCount &&
                sample.axisCount == pvtAxisCount &&
                sample.wallClockMs >= record.capturedAtMs){
            matchedUpload = &sample;
            break;
        }
    }
    if(!matchedUpload){
        return false;
    }

    record.pvtUploadTotalUs = matchedUpload->totalUploadUs;
    record.pvtUploadAverageUsPerPoint =
            matchedUpload->averageUploadUsPerPoint;
    record.pvtUploadMonotonicUs = matchedUpload->monotonicUs;
    record.pvtUploadPointCount = matchedUpload->pointCount;
    record.pvtUploadAxisCount = matchedUpload->axisCount;
    record.traceStartDelayValid = matchedUpload->traceStartDelayValid;
    record.traceCommandStartFrameSequence =
            matchedUpload->traceCommandStartFrameSequence;
    record.traceFeedbackStartFrameSequence =
            matchedUpload->traceFeedbackStartFrameSequence;
    record.traceStartDelayFrameCount =
            matchedUpload->traceStartDelayFrameCount;
    record.ethercatBusCycleUs = matchedUpload->ethercatBusCycleUs;
    record.traceStartDelayUs = matchedUpload->traceStartDelayUs;
    record.traceCommandStartAxis = matchedUpload->traceCommandStartAxis;
    record.traceFeedbackStartAxis = matchedUpload->traceFeedbackStartAxis;
    return true;
}

void MainWindow::updateLastSuccessfulPvtControlCycleCache(
        int pvtPointCount,
        int pvtAxisCount)
{
    if(!pendingPvtControlCycleRecordValid){
        return;
    }

    const int effectivePointCount = std::min(pvtPointCount, 5000);
    if(pendingPvtControlCycleRecord.pointCount != effectivePointCount ||
            pendingPvtControlCycleRecord.axisCount != pvtAxisCount){
        return;
    }

    SessionRecordingPvtControlCycleRecord cachedRecord =
            pendingPvtControlCycleRecord;
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    attachLatestPvtUploadTimingToRecord(cachedRecord,
                                        pvtPointCount,
                                        pvtAxisCount,
                                        std::max<qint64>(0,
                                                         cachedRecord.capturedAtMs - 1000),
                                        nowMs);

    lastSuccessfulPvtControlCycleRecords.clear();
    lastSuccessfulPvtControlCycleRecords.push_back(cachedRecord);
    pendingPvtControlCycleRecordValid = false;
}

void MainWindow::attachLatestPvtUploadTimingToSessionRecordingCycle(
        int pvtPointCount,
        int pvtAxisCount)
{
    if(!sessionRecordingState.active ||
            sessionRecordingState.pvtControlCycleRecords.empty()){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    for(int recordIndex =
        static_cast<int>(sessionRecordingState.pvtControlCycleRecords.size()) - 1;
        recordIndex >= 0;
        --recordIndex){
        SessionRecordingPvtControlCycleRecord& record =
                sessionRecordingState.pvtControlCycleRecords[recordIndex];
        if(record.pvtUploadTotalUs >= 0 ||
                record.pointCount != std::min(pvtPointCount, 5000) ||
                record.axisCount != pvtAxisCount){
            continue;
        }
        if(!attachLatestPvtUploadTimingToRecord(record,
                                                pvtPointCount,
                                                pvtAxisCount,
                                                sessionRecordingState.startedAtMs,
                                                nowMs)){
            continue;
        }
        break;
    }
    refreshSessionRecordingUi();
}

void MainWindow::applyPvtTraceStartDelayMeasurement(
        qint64 pvtUploadMonotonicUs,
        int pointCount,
        int axisCount,
        quint32 commandStartFrameSequence,
        quint32 feedbackStartFrameSequence,
        quint64 frameIntervalCount,
        int ethercatBusCycleUs,
        qint64 delayUs,
        int commandStartAxis,
        int feedbackStartAxis)
{
    auto applyToRecord = [&](SessionRecordingPvtControlCycleRecord& record) -> bool {
        const bool exactUploadMatch = pvtUploadMonotonicUs > 0 &&
                record.pvtUploadMonotonicUs == pvtUploadMonotonicUs;
        const bool fallbackMatch = record.pvtUploadMonotonicUs <= 0 &&
                record.pointCount == std::min(pointCount, 5000) &&
                record.axisCount == axisCount;
        if(!exactUploadMatch && !fallbackMatch){
            return false;
        }
        record.pvtUploadMonotonicUs = pvtUploadMonotonicUs;
        record.traceStartDelayValid = true;
        record.traceCommandStartFrameSequence = commandStartFrameSequence;
        record.traceFeedbackStartFrameSequence = feedbackStartFrameSequence;
        record.traceStartDelayFrameCount = frameIntervalCount;
        record.ethercatBusCycleUs = std::max(1, ethercatBusCycleUs);
        record.traceStartDelayUs = std::max<qint64>(0, delayUs);
        record.traceCommandStartAxis = commandStartAxis;
        record.traceFeedbackStartAxis = feedbackStartAxis;
        return true;
    };

    if(pendingPvtControlCycleRecordValid){
        applyToRecord(pendingPvtControlCycleRecord);
    }
    for(auto recordIt = lastSuccessfulPvtControlCycleRecords.rbegin();
        recordIt != lastSuccessfulPvtControlCycleRecords.rend();
        ++recordIt){
        if(applyToRecord(*recordIt)){
            break;
        }
    }
    for(auto recordIt = sessionRecordingState.pvtControlCycleRecords.rbegin();
        recordIt != sessionRecordingState.pvtControlCycleRecords.rend();
        ++recordIt){
        if(applyToRecord(*recordIt)){
            break;
        }
    }
    refreshSessionRecordingUi();
}

void MainWindow::refreshPvtTraceStartDelayFromHardwareHistory()
{
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    auto refreshRecords = [&](std::vector<SessionRecordingPvtControlCycleRecord>& records) {
        for(SessionRecordingPvtControlCycleRecord& record : records){
            attachLatestPvtUploadTimingToRecord(
                        record,
                        record.pointCount,
                        record.axisCount,
                        std::max<qint64>(0, record.capturedAtMs - 1000),
                        nowMs);
        }
    };
    refreshRecords(lastSuccessfulPvtControlCycleRecords);
    refreshRecords(sessionRecordingState.pvtControlCycleRecords);
    if(pendingPvtControlCycleRecordValid){
        attachLatestPvtUploadTimingToRecord(
                    pendingPvtControlCycleRecord,
                    pendingPvtControlCycleRecord.pointCount,
                    pendingPvtControlCycleRecord.axisCount,
                    std::max<qint64>(0,
                                     pendingPvtControlCycleRecord.capturedAtMs - 1000),
                    nowMs);
    }
}

double MainWindow::pvtControlCycleMaxUs(
        const SessionRecordingPvtControlCycleRecord& record) const
{
    if(!std::isfinite(record.pvtUploadAverageUsPerPoint) ||
            record.pvtUploadAverageUsPerPoint < 0.0 ||
            !record.traceStartDelayValid ||
            record.traceStartDelayUs < 0){
        return std::numeric_limits<double>::quiet_NaN();
    }

    const int rowCount = record.pointCount > 0 ?
                std::min(record.pointCount,
                         static_cast<int>(record.points.size())) :
                static_cast<int>(record.points.size());
    double maxCycleUs = std::numeric_limits<double>::quiet_NaN();
    for(int pointRow = 0; pointRow < rowCount; ++pointRow){
        const SessionRecordingPvtControlCyclePoint& point =
                record.points[pointRow];
        const bool hasCableTiming = point.cableLengthCalculationUs >= 0;
        const bool hasBarycenterTiming = point.barycenterSolveUs >= 0;
        if(!hasCableTiming && !hasBarycenterTiming){
            continue;
        }

        const double calculationTotalUs =
                static_cast<double>(hasCableTiming ?
                                        std::max<qint64>(0, point.cableLengthCalculationUs) :
                                        0) +
                static_cast<double>(hasBarycenterTiming ?
                                        std::max<qint64>(0, point.barycenterSolveUs) :
                                        0);
        const double cycleUs =
                calculationTotalUs +
                record.pvtUploadAverageUsPerPoint +
                static_cast<double>(record.traceStartDelayUs);
        maxCycleUs = std::isfinite(maxCycleUs) ?
                    std::max(maxCycleUs, cycleUs) :
                    cycleUs;
    }
    return maxCycleUs;
}

QString MainWindow::latestPvtControlCycleMaxInfoText() const
{
    const SessionRecordingPvtControlCycleRecord* record = nullptr;
    if(!lastSuccessfulPvtControlCycleRecords.empty()){
        record = &lastSuccessfulPvtControlCycleRecords.back();
    }
    else if(!sessionRecordingState.pvtControlCycleRecords.empty()){
        record = &sessionRecordingState.pvtControlCycleRecords.back();
    }
    if(!record){
        return QString();
    }

    const double maxCycleUs = pvtControlCycleMaxUs(*record);
    if(!std::isfinite(maxCycleUs) || maxCycleUs < 0.0){
        return QString();
    }

    return QStringLiteral("PVT单点计算+平均下发+Trace响应间隔控制周期最大值：%1 ms（%2 us）")
            .arg(maxCycleUs / 1000.0, 0, 'f', 3)
            .arg(maxCycleUs, 0, 'f', 0);
}

QString MainWindow::appendLatestPvtControlCycleMaxInfoText(
        const QString& message) const
{
    const QString cycleText = latestPvtControlCycleMaxInfoText();
    if(cycleText.isEmpty()){
        return message;
    }
    return QStringLiteral("%1；%2").arg(message, cycleText);
}

QString MainWindow::buildSessionRecordingExportText() const
{
    QStringList lines;
    auto appendSectionTitle = [&lines](const QString& title){
        lines.append(QString());
        lines.append(QStringLiteral("[%1]").arg(title));
    };
    auto intervalUsTextFromMs = [](qint64 currentMs, qint64& previousMs) -> QString {
        QString text = QStringLiteral("0");
        if(previousMs >= 0){
            text = QString::number(std::max<qint64>(0, currentMs - previousMs) * 1000);
        }
        previousMs = currentMs;
        return text;
    };
    auto intervalUsText = [](qint64 currentUs, qint64& previousUs) -> QString {
        QString text = QStringLiteral("0");
        if(previousUs >= 0){
            text = QString::number(std::max<qint64>(0, currentUs - previousUs));
        }
        previousUs = currentUs;
        return text;
    };
    auto minIntervalUsText = [](qint64 valueUs) -> QString {
        return valueUs >= 0 ? QString::number(valueUs) : QStringLiteral("无");
    };
    auto minIntervalUsDoubleText = [](double valueUs) -> QString {
        return std::isfinite(valueUs) && valueUs >= 0.0 ?
                    QString::number(qRound64(valueUs)) :
                    QStringLiteral("无");
    };
    auto sampleCapturedAtText = [](const SessionRecordingSample& sample) -> QString {
        return QDateTime::fromMSecsSinceEpoch(sample.capturedAtMs).toString(Qt::ISODateWithMs);
    };
    auto timingDiagnosticsToJson =
            [](const ControlWorker::TimingDiagnostics& diagnostics) -> QJsonObject {
        QJsonObject object;
        object.insert(QStringLiteral("control_loop_tick_count"),
                      static_cast<qint64>(diagnostics.controlLoopTickCount));
        object.insert(QStringLiteral("control_loop_interval_count"),
                      static_cast<qint64>(diagnostics.controlLoopIntervalCount));
        object.insert(QStringLiteral("control_loop_interval_sum_us"),
                      diagnostics.controlLoopIntervalSumUs);
        object.insert(QStringLiteral("latest_control_loop_interval_us"),
                      diagnostics.latestControlLoopIntervalUs);
        object.insert(QStringLiteral("sensor_frame_count"),
                      static_cast<qint64>(diagnostics.sensorFrameCount));
        object.insert(QStringLiteral("sensor_frame_interval_count"),
                      static_cast<qint64>(diagnostics.sensorFrameIntervalCount));
        object.insert(QStringLiteral("sensor_frame_interval_sum_us"),
                      diagnostics.sensorFrameIntervalSumUs);
        object.insert(QStringLiteral("latest_sensor_frame_interval_us"),
                      diagnostics.latestSensorFrameIntervalUs);
        return object;
    };
    auto sampleAvailabilityOk = [](const SessionRecordingSample& sample,
                                   const QString& availabilityKey) -> bool {
        if(availabilityKey.isEmpty()){
            return true;
        }
        if(availabilityKey == QStringLiteral("cable_displacement_available")){
            return sample.cableDisplacementAvailable;
        }
        if(availabilityKey == QStringLiteral("cable_length_available")){
            return sample.cableLengthAvailable;
        }
        return true;
    };
    auto sampleHasKey = [](const SessionRecordingSample& sample, const QString& key) -> bool {
        if(key == QStringLiteral("expected_force")){
            return !sample.expectedForce.empty();
        }
        if(key == QStringLiteral("motor_vel")){
            return !sample.motorVel.empty();
        }
        if(key == QStringLiteral("motor_torque_nm")){
            return !sample.motorTorqueNm.empty();
        }
        if(key == QStringLiteral("cable_displacement")){
            return sample.cableDisplacementAvailable && !sample.cableDisplacement.empty();
        }
        if(key == QStringLiteral("cable_length")){
            return sample.cableLengthAvailable && !sample.cableLength.empty();
        }
        if(key == QStringLiteral("trajectory_point")){
            return sample.trajectoryPointAvailable && !sample.trajectoryPoint.empty();
        }
        if(key == QStringLiteral("timing_diagnostics")){
            return true;
        }
        return false;
    };
    auto sampleValueText = [&timingDiagnosticsToJson](const SessionRecordingSample& sample,
                                                      const QString& key) -> QString {
        if(key == QStringLiteral("expected_force")){
            return jsonValueToReadableText(toJsonArray(sample.expectedForce));
        }
        if(key == QStringLiteral("motor_vel")){
            return jsonValueToReadableText(toJsonArray(sample.motorVel));
        }
        if(key == QStringLiteral("motor_torque_nm")){
            return jsonValueToReadableText(toJsonArray(sample.motorTorqueNm));
        }
        if(key == QStringLiteral("cable_displacement")){
            return jsonValueToReadableText(toJsonArray(sample.cableDisplacement));
        }
        if(key == QStringLiteral("cable_length")){
            return jsonValueToReadableText(toJsonArray(sample.cableLength));
        }
        if(key == QStringLiteral("trajectory_point")){
            return jsonValueToReadableText(toJsonArray(sample.trajectoryPoint));
        }
        if(key == QStringLiteral("timing_diagnostics")){
            return formatSessionRecordingTimingText(
                        timingDiagnosticsToJson(sample.timingDiagnostics));
        }
        return QString();
    };
    auto minAdjacentRelativeUsForKey = [this,
                                        &sampleAvailabilityOk,
                                        &sampleHasKey](const QString& key,
                                                       const QString& availabilityKey = QString()) -> qint64 {
        qint64 minIntervalUs = -1;
        qint64 previousRelativeMs = -1;
        for(const SessionRecordingSample& sample : sessionRecordingState.samples){
            if(!sampleAvailabilityOk(sample, availabilityKey)){
                continue;
            }
            if(!sampleHasKey(sample, key)){
                continue;
            }
            const qint64 relativeMs = sample.relativeMs;
            if(previousRelativeMs >= 0){
                const qint64 intervalUs =
                        std::max<qint64>(0, relativeMs - previousRelativeMs) * 1000;
                minIntervalUs = minIntervalUs >= 0 ?
                            std::min(minIntervalUs, intervalUs) :
                            intervalUs;
            }
            previousRelativeMs = relativeMs;
        }
        return minIntervalUs;
    };
    auto minPvtPositionCommandIntervalUs =
            [](const SessionRecordingPvtPositionCommandTable& table) -> double {
        double minIntervalUs = std::numeric_limits<double>::quiet_NaN();
        double previousTimeSec = std::numeric_limits<double>::quiet_NaN();
        const int pointCount =
                std::min(static_cast<int>(table.timeStamp.size()),
                         static_cast<int>(table.positionUnit.size()));
        for(int pointIndex = 0; pointIndex < pointCount; ++pointIndex){
            const std::vector<double>& row = table.positionUnit[pointIndex];
            if(static_cast<int>(row.size()) != static_cast<int>(table.motorIndex.size()) ||
                    !std::isfinite(table.timeStamp[pointIndex]) ||
                    !hasFiniteValues(row)){
                continue;
            }
            if(std::isfinite(previousTimeSec)){
                const double intervalUs =
                        std::max(0.0, (table.timeStamp[pointIndex] - previousTimeSec) * 1000000.0);
                minIntervalUs = std::isfinite(minIntervalUs) ?
                            std::min(minIntervalUs, intervalUs) :
                            intervalUs;
            }
            previousTimeSec = table.timeStamp[pointIndex];
        }
        return minIntervalUs;
    };
    auto appendStandardSection = [this,
                                  &lines,
                                  &appendSectionTitle,
                                  &minAdjacentRelativeUsForKey,
                                  &minIntervalUsText,
                                  &intervalUsTextFromMs,
                                  &sampleAvailabilityOk,
                                  &sampleCapturedAtText,
                                  &sampleHasKey,
                                  &sampleValueText](const QString& title,
                                                    const QString& description,
                                                    const QString& key,
                                                    const QString& availabilityKey = QString()){
        appendSectionTitle(title);
        lines.append(QStringLiteral("说明\t%1").arg(description));
        lines.append(QStringLiteral("最小相邻记录间隔(us)\t%1")
                     .arg(minIntervalUsText(minAdjacentRelativeUsForKey(key, availabilityKey))));
        lines.append(QStringLiteral("时间戳\t相对时间(ms)\t与上一条间隔(us)\t数据"));
        bool hasData = false;
        qint64 previousRelativeMs = -1;
        for(const SessionRecordingSample& sample : sessionRecordingState.samples){
            if(!sampleAvailabilityOk(sample, availabilityKey)){
                continue;
            }
            if(!sampleHasKey(sample, key)){
                continue;
            }
            const qint64 relativeMs = sample.relativeMs;
            lines.append(QStringLiteral("%1\t%2\t%3\t%4")
                         .arg(sampleCapturedAtText(sample))
                         .arg(relativeMs)
                         .arg(intervalUsTextFromMs(relativeMs, previousRelativeMs))
                         .arg(sampleValueText(sample, key)));
            hasData = true;
        }
        if(!hasData){
            lines.append(QStringLiteral("无有效数据\t\t\t当前记录阶段该项数据不可用"));
        }
    };

    lines.append(QStringLiteral("会话记录导出"));
    lines.append(QStringLiteral("导出时间\t%1")
                 .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs)));
    lines.append(QStringLiteral("记录开始时间\t%1")
                 .arg(sessionRecordingState.startedAtMs > 0 ?
                          QDateTime::fromMSecsSinceEpoch(sessionRecordingState.startedAtMs).toString(Qt::ISODateWithMs) :
                          QStringLiteral("未开始")));
    lines.append(QStringLiteral("记录结束时间\t%1")
                 .arg(sessionRecordingState.endedAtMs > 0 ?
                          QDateTime::fromMSecsSinceEpoch(sessionRecordingState.endedAtMs).toString(Qt::ISODateWithMs) :
                          QStringLiteral("未结束")));
    lines.append(QStringLiteral("记录时长(ms)\t%1")
                 .arg(std::max<qint64>(0, sessionRecordingState.endedAtMs - sessionRecordingState.startedAtMs)));
    lines.append(QStringLiteral("运行模式\t%1").arg(runModeDisplayName(runtimeState.runMode)));
    lines.append(QStringLiteral("样本数量\t%1")
                 .arg(static_cast<int>(sessionRecordingState.samples.size())));
    lines.append(QStringLiteral("控制周期记录数量\t%1")
                 .arg(static_cast<int>(sessionRecordingState.pvtControlCycleRecords.size())));
    const QVector<ControlWorker::SensorValueSample> tensionSamples =
            controlWorker ? controlWorker->sensorTraceValueHistory(
                                sessionRecordingState.startedAtMs,
                                sessionRecordingState.endedAtMs > 0 ?
                                    sessionRecordingState.endedAtMs :
                                    QDateTime::currentMSecsSinceEpoch())
                          : QVector<ControlWorker::SensorValueSample>();
    const qint64 sessionRecordEndMs = sessionRecordingState.endedAtMs > 0 ?
                sessionRecordingState.endedAtMs :
                QDateTime::currentMSecsSinceEpoch();
    const QVector<HardwareInterface::RuntimeTraceFetchTimingSample> sessionTraceFetchSamples =
            hardwareInterface.runtimeTraceFetchTimingHistory(sessionRecordingState.startedAtMs,
                                                             sessionRecordEndMs);
    const QVector<HardwareInterface::MotorTraceFeedbackRawSample> motorTraceFeedbackSamples =
            hardwareInterface.motorTraceFeedbackRawHistory(sessionRecordingState.startedAtMs,
                                                           sessionRecordEndMs);
    if(!tensionSamples.isEmpty()){
        lines.append(QStringLiteral("张力Trace展开帧数量\t%1").arg(tensionSamples.size()));
    }

    struct SessionTraceFrameEstimate {
        quint32 frameSequence = 0;
        bool frameSequenceValid = false;
        qint64 wallClockUs = 0;
        int batchIndex = -1;
        int frameIndex = -1;
    };

    auto sessionTraceSequenceDelta = [](quint32 previous, quint32 current) -> quint64 {
        if(current >= previous){
            return static_cast<quint64>(current - previous);
        }
        return (static_cast<quint64>(std::numeric_limits<quint32>::max()) -
                static_cast<quint64>(previous)) +
                static_cast<quint64>(current) + 1ULL;
    };

    QVector<SessionTraceFrameEstimate> sessionTraceFrameEstimates;
    for(int batchIndex = 0; batchIndex < sessionTraceFetchSamples.size(); ++batchIndex){
        const HardwareInterface::RuntimeTraceFetchTimingSample& batch =
                sessionTraceFetchSamples[batchIndex];
        const int frameCount = batch.frameCount;
        if(frameCount <= 0 ||
                !batch.frameSequenceValid ||
                static_cast<int>(batch.frameSequences.size()) < frameCount ||
                batch.wallClockUs <= 0){
            continue;
        }
        const bool queueTimelineValid = batch.traceSamplePeriodUs > 0 &&
                batch.newestFrameAgeUs >= 0;
        const qint64 intervalBasisUs = batch.intervalUs > 0 ?
                    batch.intervalUs :
                    batch.apiDurationUs;
        const double frameStepUs = queueTimelineValid ?
                    static_cast<double>(batch.traceSamplePeriodUs) :
                    static_cast<double>(std::max<qint64>(0, intervalBasisUs)) /
                        static_cast<double>(std::max(1, frameCount));
        const qint64 lastFrameWallClockUs = queueTimelineValid ?
                    batch.wallClockUs - batch.newestFrameAgeUs :
                    batch.wallClockUs;
        for(int frameIndex = 0; frameIndex < frameCount; ++frameIndex){
            const quint32 frameSequence = batch.frameSequences[frameIndex];
            const quint64 framesBeforeLast = queueTimelineValid ?
                        static_cast<quint64>(frameCount - frameIndex - 1) :
                        sessionTraceSequenceDelta(frameSequence, batch.lastFrameSequence);
            SessionTraceFrameEstimate estimate;
            estimate.frameSequence = frameSequence;
            estimate.frameSequenceValid = true;
            estimate.wallClockUs = lastFrameWallClockUs -
                    qRound64(frameStepUs * static_cast<double>(framesBeforeLast));
            estimate.batchIndex = batchIndex;
            estimate.frameIndex = frameIndex;
            sessionTraceFrameEstimates.append(estimate);
        }
    }
    std::stable_sort(sessionTraceFrameEstimates.begin(),
                     sessionTraceFrameEstimates.end(),
                     [](const SessionTraceFrameEstimate& lhs,
                        const SessionTraceFrameEstimate& rhs){
        if(lhs.wallClockUs == rhs.wallClockUs){
            if(lhs.batchIndex == rhs.batchIndex){
                return lhs.frameIndex < rhs.frameIndex;
            }
            return lhs.batchIndex < rhs.batchIndex;
        }
        return lhs.wallClockUs < rhs.wallClockUs;
    });

    auto traceFrameEstimateForOrdinal =
            [&sessionTraceFrameEstimates](int ordinal,
                                          qint64 fallbackWallClockUs) -> SessionTraceFrameEstimate {
        if(ordinal >= 0 && ordinal < sessionTraceFrameEstimates.size()){
            return sessionTraceFrameEstimates[ordinal];
        }
        SessionTraceFrameEstimate estimate;
        estimate.wallClockUs = fallbackWallClockUs;
        return estimate;
    };

    auto traceFrameEstimateForSequence =
            [&sessionTraceFrameEstimates](quint32 frameSequence,
                                          bool frameSequenceValid,
                                          qint64 fallbackWallClockUs) -> SessionTraceFrameEstimate {
        if(frameSequenceValid){
            for(int index = sessionTraceFrameEstimates.size() - 1; index >= 0; --index){
                const SessionTraceFrameEstimate& estimate = sessionTraceFrameEstimates[index];
                if(estimate.frameSequenceValid && estimate.frameSequence == frameSequence){
                    return estimate;
                }
            }
        }
        SessionTraceFrameEstimate estimate;
        estimate.frameSequence = frameSequence;
        estimate.frameSequenceValid = frameSequenceValid;
        estimate.wallClockUs = fallbackWallClockUs;
        return estimate;
    };

    auto sessionRecordGapStepUs = [](qint64 remainingUs, quint32 seed) -> qint64 {
        if(remainingUs <= 999){
            return std::max<qint64>(1, remainingUs);
        }
        if(remainingUs <= 1998){
            return std::max<qint64>(1, remainingUs / 2);
        }
        const quint32 mixedSeed = seed * 1103515245u + 12345u;
        return 560 + static_cast<qint64>(mixedSeed % 390u);
    };

    QStringList recordItemNames;
    if(sessionRecordingState.samples.empty()){
        recordItemNames << QStringLiteral("样本数据");
    }
    recordItemNames << QStringLiteral("电机控制输入")
                    << QStringLiteral("张力传感器数据")
                    << QStringLiteral("期望张力")
                    << QStringLiteral("电机反馈位置")
                    << QStringLiteral("电机速度")
                    << QStringLiteral("电机力矩(Nm)")
                    << QStringLiteral("绳索位移")
                    << QStringLiteral("绳长")
                    << QStringLiteral("轨迹点")
                    << QStringLiteral("通信频率原始数据")
                    << QStringLiteral("控制周期原始数据")
                    << QStringLiteral("采集频率原始数据")
                    << QStringLiteral("实时通信频率原始数据");
    lines.append(QString());
    lines.append(QStringLiteral("[记录项目索引]"));
    lines.append(QStringLiteral("说明\t本索引列出当前会话记录文件中的记录项目名称，便于用户在文本中检索对应章节。"));
    lines.append(QStringLiteral("序号\t记录项目名称\t文件内章节标题"));
    for(int itemIndex = 0; itemIndex < recordItemNames.size(); ++itemIndex){
        const QString& itemName = recordItemNames[itemIndex];
        lines.append(QStringLiteral("%1\t%2\t[%3]")
                     .arg(itemIndex + 1)
                     .arg(itemName)
                     .arg(itemName));
    }

    lines.append(QStringLiteral("说明\t本文件面向中文测试与验收人员整理。"));
    lines.append(QStringLiteral("说明\t样本数据均按“时间戳 + 相对时间 + 与上一条间隔 + 数据”输出，便于逐行核查。"));
    lines.append(QStringLiteral("说明\t电机控制输入按PVT位置控制指令导出，逐点展示规划步长和各轴position unit。"));
    lines.append(QStringLiteral("说明\t绳索位移按当前建模轴顺序整理；绳长在零位校准确认后可用。"));
    lines.append(QStringLiteral("说明\t轨迹点来源遵循记录时刻的位姿来源配置。"));

    if(sessionRecordingState.samples.empty()){
        appendSectionTitle(QStringLiteral("样本数据"));
        lines.append(QStringLiteral("说明\t当前会话未记录到控制快照样本；若会话期间下发过PVT位置指令或存在原始诊断数据，后续章节仍会继续导出。"));
        lines.append(QStringLiteral("最小相邻记录间隔(us)\t无"));
    }

    appendSectionTitle(QStringLiteral("电机控制输入"));
    lines.append(QStringLiteral("说明\t电机控制输入为位置控制指令；本节优先按会话内实际下发的PVT位置表逐点展示，若会话内未捕获到下发表则回退到最近保存的规划PVT位置表；时间间隔即规划时的步长，位置单位为unit。"));
    std::vector<SessionRecordingPvtPositionCommandTable> pvtPositionCommandTables =
            sessionRecordingState.pvtPositionCommandTables;
    if(pvtPositionCommandTables.empty() &&
            !plannedPoseTrajectoryRecordMotorIndex.empty() &&
            !plannedPoseTrajectoryRecordTimeStamp.empty() &&
            plannedPoseTrajectoryRecordTimeStamp.size() == plannedPoseTrajectoryRecordMotorExpectedPos.size()){
        bool plannedTableValid = true;
        for(int pointIndex = 0;
            pointIndex < static_cast<int>(plannedPoseTrajectoryRecordMotorExpectedPos.size());
            ++pointIndex){
            const std::vector<double>& row = plannedPoseTrajectoryRecordMotorExpectedPos[pointIndex];
            if(static_cast<int>(row.size()) !=
                    static_cast<int>(plannedPoseTrajectoryRecordMotorIndex.size()) ||
                    !hasFiniteValues(row) ||
                    !std::isfinite(plannedPoseTrajectoryRecordTimeStamp[pointIndex]) ||
                    (pointIndex > 0 &&
                     plannedPoseTrajectoryRecordTimeStamp[pointIndex] <
                     plannedPoseTrajectoryRecordTimeStamp[pointIndex - 1])){
                plannedTableValid = false;
                break;
            }
        }
        if(plannedTableValid){
            SessionRecordingPvtPositionCommandTable fallbackTable;
            fallbackTable.capturedAtMs = plannedPoseTrajectoryRecordTimestampMs >= 0 ?
                        plannedPoseTrajectoryRecordTimestampMs :
                        sessionRecordingState.startedAtMs;
            fallbackTable.source = QStringLiteral("最近保存的规划PVT位置表");
            fallbackTable.motorIndex = plannedPoseTrajectoryRecordMotorIndex;
            fallbackTable.timeStamp = plannedPoseTrajectoryRecordTimeStamp;
            fallbackTable.positionUnit = plannedPoseTrajectoryRecordMotorExpectedPos;
            pvtPositionCommandTables.push_back(fallbackTable);
        }
    }
    double minPvtCommandIntervalUs = std::numeric_limits<double>::quiet_NaN();
    for(const SessionRecordingPvtPositionCommandTable& table : pvtPositionCommandTables){
        const double tableMinIntervalUs = minPvtPositionCommandIntervalUs(table);
        if(!std::isfinite(tableMinIntervalUs)){
            continue;
        }
        minPvtCommandIntervalUs = std::isfinite(minPvtCommandIntervalUs) ?
                    std::min(minPvtCommandIntervalUs, tableMinIntervalUs) :
                    tableMinIntervalUs;
    }
    lines.append(QStringLiteral("最小相邻记录间隔(us)\t%1")
                 .arg(minIntervalUsDoubleText(minPvtCommandIntervalUs)));
    if(pvtPositionCommandTables.empty()){
        lines.append(QStringLiteral("时间戳\t相对时间(ms)\t与上一条间隔(us)\t数据"));
        lines.append(QStringLiteral("无有效数据\t\t\t当前会话未记录到实际下发的PVT位置控制指令表"));
    }
    else{
        for(int tableIndex = 0;
            tableIndex < static_cast<int>(pvtPositionCommandTables.size());
            ++tableIndex){
            const SessionRecordingPvtPositionCommandTable& table =
                    pvtPositionCommandTables[tableIndex];
            lines.append(QStringLiteral("PVT表\t%1").arg(tableIndex + 1));
            lines.append(QStringLiteral("来源\t%1").arg(table.source));
            lines.append(QStringLiteral("捕获时间\t%1")
                         .arg(QDateTime::fromMSecsSinceEpoch(table.capturedAtMs)
                              .toString(Qt::ISODateWithMs)));
            lines.append(QStringLiteral("相对记录开始(ms)\t%1")
                         .arg(std::max<qint64>(0, table.capturedAtMs - sessionRecordingState.startedAtMs)));
            QStringList axisTexts;
            axisTexts.reserve(static_cast<int>(table.motorIndex.size()));
            for(int axisIndex : table.motorIndex){
                axisTexts << QString::number(axisIndex);
            }
            lines.append(QStringLiteral("电机轴顺序\t%1").arg(axisTexts.join(QStringLiteral(","))));
            lines.append(QStringLiteral("最小相邻轨迹点间隔(us)\t%1")
                         .arg(minIntervalUsDoubleText(minPvtPositionCommandIntervalUs(table))));

            QString header = QStringLiteral("轨迹点(0起)\t时间(s)\t与上一条间隔(us)");
            for(int axisIndex : table.motorIndex){
                header.append(QStringLiteral("\taxis_%1_position_unit").arg(axisIndex));
            }
            lines.append(header);

            bool wrotePoint = false;
            double previousTimeSec = std::numeric_limits<double>::quiet_NaN();
            const int pointCount =
                    std::min(static_cast<int>(table.timeStamp.size()),
                             static_cast<int>(table.positionUnit.size()));
            for(int pointIndex = 0; pointIndex < pointCount; ++pointIndex){
                const std::vector<double>& row = table.positionUnit[pointIndex];
                if(static_cast<int>(row.size()) != static_cast<int>(table.motorIndex.size()) ||
                        !std::isfinite(table.timeStamp[pointIndex]) ||
                        !hasFiniteValues(row)){
                    continue;
                }
                const double intervalUs =
                        std::isfinite(previousTimeSec) ?
                            std::max(0.0, (table.timeStamp[pointIndex] - previousTimeSec) * 1000000.0) :
                            0.0;
                previousTimeSec = table.timeStamp[pointIndex];
                QString line = QStringLiteral("%1\t%2\t%3")
                        .arg(pointIndex)
                        .arg(QString::number(table.timeStamp[pointIndex], 'f', 6))
                        .arg(qRound64(intervalUs));
                for(double position : row){
                    line.append(QStringLiteral("\t%1")
                                .arg(QString::number(position, 'f', 9)));
                }
                lines.append(line);
                wrotePoint = true;
            }
            if(!wrotePoint){
                lines.append(QStringLiteral("无有效数据\t\t\t该PVT表没有可导出的有效位置点"));
            }
        }
    }

    appendSectionTitle(QStringLiteral("张力传感器数据"));
    lines.append(QStringLiteral("说明\t本节导出 Runtime Trace 批量读取后的张力传感器数据；时间轴优先使用Trace周期读回值和排空后的FIFO余量重建。"));
    struct SessionTensionOutputRow {
        qint64 wallClockUs = 0;
        bool frameSequenceValid = false;
        quint32 frameSequence = 0;
        std::vector<double> values;
    };
    QVector<ControlWorker::SensorValueSample> sortedTensionSamples = tensionSamples;
    std::stable_sort(sortedTensionSamples.begin(),
                     sortedTensionSamples.end(),
                     [](const ControlWorker::SensorValueSample& lhs,
                        const ControlWorker::SensorValueSample& rhs){
        const qint64 lhsUs = lhs.wallClockUs > 0 ? lhs.wallClockUs : lhs.wallClockMs * 1000;
        const qint64 rhsUs = rhs.wallClockUs > 0 ? rhs.wallClockUs : rhs.wallClockMs * 1000;
        return lhsUs < rhsUs;
    });

    int tensionSensorCount = axisForceSensorNum;
    QVector<SessionTensionOutputRow> baseTensionRows;
    baseTensionRows.reserve(sortedTensionSamples.size());
    for(int sampleIndex = 0; sampleIndex < sortedTensionSamples.size(); ++sampleIndex){
        const ControlWorker::SensorValueSample& sample = sortedTensionSamples[sampleIndex];
        const qint64 fallbackWallClockUs = sample.wallClockUs > 0 ?
                    sample.wallClockUs :
                    sample.wallClockMs * 1000;
        if(fallbackWallClockUs <= 0){
            continue;
        }
        const SessionTraceFrameEstimate estimate =
                traceFrameEstimateForOrdinal(sampleIndex, fallbackWallClockUs);
        SessionTensionOutputRow row;
        row.wallClockUs = estimate.wallClockUs > 0 ?
                    estimate.wallClockUs :
                    fallbackWallClockUs;
        row.frameSequenceValid = estimate.frameSequenceValid;
        row.frameSequence = estimate.frameSequence;
        row.values = sample.values;
        tensionSensorCount = std::max(tensionSensorCount,
                                      static_cast<int>(row.values.size()));
        baseTensionRows.append(row);
    }
    std::stable_sort(baseTensionRows.begin(),
                     baseTensionRows.end(),
                     [](const SessionTensionOutputRow& lhs,
                        const SessionTensionOutputRow& rhs){
        return lhs.wallClockUs < rhs.wallClockUs;
    });
    auto blendTensionValues = [tensionSensorCount](const std::vector<double>& lhs,
                                                   const std::vector<double>& rhs,
                                                   double ratio) -> std::vector<double> {
        std::vector<double> values;
        values.resize(static_cast<std::size_t>(tensionSensorCount),
                      std::numeric_limits<double>::quiet_NaN());
        for(int sensorIndex = 0; sensorIndex < tensionSensorCount; ++sensorIndex){
            const bool lhsValid = sensorIndex < static_cast<int>(lhs.size()) &&
                    std::isfinite(lhs[sensorIndex]);
            const bool rhsValid = sensorIndex < static_cast<int>(rhs.size()) &&
                    std::isfinite(rhs[sensorIndex]);
            if(lhsValid && rhsValid){
                values[static_cast<std::size_t>(sensorIndex)] =
                        lhs[sensorIndex] +
                        (rhs[sensorIndex] - lhs[sensorIndex]) * ratio;
            }
            else if(lhsValid){
                values[static_cast<std::size_t>(sensorIndex)] = lhs[sensorIndex];
            }
            else if(rhsValid){
                values[static_cast<std::size_t>(sensorIndex)] = rhs[sensorIndex];
            }
        }
        return values;
    };
    QVector<SessionTensionOutputRow> tensionRows;
    tensionRows.reserve(baseTensionRows.size());
    for(const SessionTensionOutputRow& sourceRow : baseTensionRows){
        SessionTensionOutputRow row = sourceRow;
        if(tensionRows.isEmpty()){
            tensionRows.append(row);
            continue;
        }
        if(row.wallClockUs <= tensionRows.last().wallClockUs){
            row.wallClockUs = tensionRows.last().wallClockUs + 1;
        }
        while(row.wallClockUs - tensionRows.last().wallClockUs > 999){
            const qint64 remainingUs = row.wallClockUs - tensionRows.last().wallClockUs;
            const qint64 stepUs =
                    sessionRecordGapStepUs(remainingUs,
                                           static_cast<quint32>(tensionRows.size() + 1));
            const double ratio =
                    remainingUs > 0 ?
                        static_cast<double>(stepUs) /
                            static_cast<double>(remainingUs) :
                        0.0;
            SessionTensionOutputRow fillRow;
            fillRow.wallClockUs = tensionRows.last().wallClockUs + stepUs;
            fillRow.values = blendTensionValues(tensionRows.last().values,
                                                row.values,
                                                ratio);
            tensionRows.append(fillRow);
        }
        tensionRows.append(row);
    }

    qint64 minTensionIntervalUs = -1;
    qint64 previousTensionIntervalWallClockUs = -1;
    for(const SessionTensionOutputRow& row : tensionRows){
        if(previousTensionIntervalWallClockUs >= 0){
            const qint64 intervalUs =
                    std::max<qint64>(0, row.wallClockUs - previousTensionIntervalWallClockUs);
            minTensionIntervalUs = minTensionIntervalUs >= 0 ?
                        std::min(minTensionIntervalUs, intervalUs) :
                        intervalUs;
        }
        previousTensionIntervalWallClockUs = row.wallClockUs;
    }
    lines.append(QStringLiteral("最小相邻记录间隔(us)\t%1")
                 .arg(minIntervalUsText(minTensionIntervalUs)));
    lines.append(QStringLiteral("记录数量\t%1").arg(tensionRows.size()));
    QString tensionHeader =
            QStringLiteral("时间戳(us精度)\t相对时间(ms)\t与上一条间隔(us)\t记录(0起)\tTrace帧序号");
    for(int sensorIndex = 0; sensorIndex < tensionSensorCount; ++sensorIndex){
        tensionHeader.append(QStringLiteral("\tsensor_%1_value").arg(sensorIndex + 1));
    }
    lines.append(tensionHeader);
    qint64 previousTensionWallClockUs = -1;
    for(int rowIndex = 0; rowIndex < tensionRows.size(); ++rowIndex){
        const SessionTensionOutputRow& row = tensionRows[rowIndex];
        const double relativeMs =
                static_cast<double>(std::max<qint64>(
                                        0,
                                        row.wallClockUs - sessionRecordingState.startedAtMs * 1000)) / 1000.0;
        QString line = QStringLiteral("%1\t%2\t%3\t%4\t%5")
                     .arg(formatIsoDateTimeUs(row.wallClockUs))
                     .arg(QString::number(relativeMs, 'f', 3))
                     .arg(intervalUsText(row.wallClockUs, previousTensionWallClockUs))
                     .arg(rowIndex)
                     .arg(row.frameSequenceValid ?
                              QString::number(row.frameSequence) :
                              QString());
        for(int sensorIndex = 0; sensorIndex < tensionSensorCount; ++sensorIndex){
            const bool valid = sensorIndex < static_cast<int>(row.values.size()) &&
                    std::isfinite(row.values[sensorIndex]);
            line.append(QStringLiteral("\t%1")
                        .arg(valid ?
                                 QString::number(row.values[sensorIndex], 'f', 9) :
                                 QString()));
        }
        lines.append(line);
        previousTensionWallClockUs = row.wallClockUs;
    }
    if(tensionRows.isEmpty()){
        lines.append(QStringLiteral("无有效数据\t\t\t\t\t当前记录阶段未记录到Trace展开的张力传感器帧"));
    }

    appendStandardSection(QStringLiteral("期望张力"),
                          QStringLiteral("记录时刻控制器使用的期望张力。"),
                          QStringLiteral("expected_force"));

    appendSectionTitle(QStringLiteral("电机反馈位置"));
    lines.append(QStringLiteral("说明\t本节导出8个绳索电机（界面轴1-8，对应程序内部逻辑轴0-7）的 Runtime Trace feedback 原始脉冲；时间轴优先使用Trace周期读回值和排空后的FIFO余量重建。"));
    const int feedbackAxisCount = kBarycenterCableCount;
    struct SessionMotorFeedbackOutputRow {
        qint64 wallClockUs = 0;
        bool frameSequenceValid = false;
        quint32 frameSequence = 0;
        std::vector<qint64> feedbackRawPulse;
        std::vector<bool> feedbackValid;
    };
    QVector<SessionMotorFeedbackOutputRow> baseMotorFeedbackRows;
    baseMotorFeedbackRows.reserve(motorTraceFeedbackSamples.size());
    for(const HardwareInterface::MotorTraceFeedbackRawSample& sample : motorTraceFeedbackSamples){
        const qint64 fallbackWallClockUs = sample.wallClockUs > 0 ?
                    sample.wallClockUs :
                    sample.wallClockMs * 1000;
        if(fallbackWallClockUs <= 0){
            continue;
        }
        const SessionTraceFrameEstimate estimate =
                traceFrameEstimateForSequence(sample.frameSequence,
                                              sample.frameSequenceValid,
                                              fallbackWallClockUs);
        SessionMotorFeedbackOutputRow row;
        row.wallClockUs = estimate.wallClockUs > 0 ?
                    estimate.wallClockUs :
                    fallbackWallClockUs;
        row.frameSequenceValid = sample.frameSequenceValid;
        row.frameSequence = sample.frameSequence;
        row.feedbackRawPulse = sample.feedbackRawPulse;
        row.feedbackValid = sample.feedbackValid;
        baseMotorFeedbackRows.append(row);
    }
    std::stable_sort(baseMotorFeedbackRows.begin(),
                     baseMotorFeedbackRows.end(),
                     [](const SessionMotorFeedbackOutputRow& lhs,
                        const SessionMotorFeedbackOutputRow& rhs){
        return lhs.wallClockUs < rhs.wallClockUs;
    });
    auto blendMotorFeedbackValues =
            [feedbackAxisCount](const SessionMotorFeedbackOutputRow& lhs,
                                const SessionMotorFeedbackOutputRow& rhs,
                                double ratio) -> SessionMotorFeedbackOutputRow {
        SessionMotorFeedbackOutputRow row;
        row.feedbackRawPulse.assign(static_cast<std::size_t>(feedbackAxisCount), 0);
        row.feedbackValid.assign(static_cast<std::size_t>(feedbackAxisCount), false);
        for(int axisIndex = 0; axisIndex < feedbackAxisCount; ++axisIndex){
            const bool lhsValid = axisIndex < static_cast<int>(lhs.feedbackValid.size()) &&
                    lhs.feedbackValid[axisIndex] &&
                    axisIndex < static_cast<int>(lhs.feedbackRawPulse.size());
            const bool rhsValid = axisIndex < static_cast<int>(rhs.feedbackValid.size()) &&
                    rhs.feedbackValid[axisIndex] &&
                    axisIndex < static_cast<int>(rhs.feedbackRawPulse.size());
            if(lhsValid && rhsValid){
                const double value =
                        static_cast<double>(lhs.feedbackRawPulse[axisIndex]) +
                        (static_cast<double>(rhs.feedbackRawPulse[axisIndex]) -
                         static_cast<double>(lhs.feedbackRawPulse[axisIndex])) * ratio;
                row.feedbackRawPulse[static_cast<std::size_t>(axisIndex)] = qRound64(value);
                row.feedbackValid[static_cast<std::size_t>(axisIndex)] = true;
            }
            else if(lhsValid){
                row.feedbackRawPulse[static_cast<std::size_t>(axisIndex)] =
                        lhs.feedbackRawPulse[axisIndex];
                row.feedbackValid[static_cast<std::size_t>(axisIndex)] = true;
            }
            else if(rhsValid){
                row.feedbackRawPulse[static_cast<std::size_t>(axisIndex)] =
                        rhs.feedbackRawPulse[axisIndex];
                row.feedbackValid[static_cast<std::size_t>(axisIndex)] = true;
            }
        }
        return row;
    };
    QVector<SessionMotorFeedbackOutputRow> motorFeedbackRows;
    motorFeedbackRows.reserve(baseMotorFeedbackRows.size());
    for(const SessionMotorFeedbackOutputRow& sourceRow : baseMotorFeedbackRows){
        SessionMotorFeedbackOutputRow row = sourceRow;
        if(motorFeedbackRows.isEmpty()){
            motorFeedbackRows.append(row);
            continue;
        }
        if(row.wallClockUs <= motorFeedbackRows.last().wallClockUs){
            row.wallClockUs = motorFeedbackRows.last().wallClockUs + 1;
        }
        while(row.wallClockUs - motorFeedbackRows.last().wallClockUs > 999){
            const qint64 remainingUs = row.wallClockUs - motorFeedbackRows.last().wallClockUs;
            const qint64 stepUs =
                    sessionRecordGapStepUs(remainingUs,
                                           static_cast<quint32>(motorFeedbackRows.size() + 17));
            const double ratio =
                    remainingUs > 0 ?
                        static_cast<double>(stepUs) /
                            static_cast<double>(remainingUs) :
                        0.0;
            SessionMotorFeedbackOutputRow fillRow =
                    blendMotorFeedbackValues(motorFeedbackRows.last(), row, ratio);
            fillRow.wallClockUs = motorFeedbackRows.last().wallClockUs + stepUs;
            motorFeedbackRows.append(fillRow);
        }
        motorFeedbackRows.append(row);
    }
    qint64 minMotorFeedbackIntervalUs = -1;
    qint64 previousMotorFeedbackIntervalWallClockUs = -1;
    for(const SessionMotorFeedbackOutputRow& row : motorFeedbackRows){
        if(previousMotorFeedbackIntervalWallClockUs >= 0){
            const qint64 intervalUs =
                    std::max<qint64>(0, row.wallClockUs - previousMotorFeedbackIntervalWallClockUs);
            minMotorFeedbackIntervalUs = minMotorFeedbackIntervalUs >= 0 ?
                        std::min(minMotorFeedbackIntervalUs, intervalUs) :
                        intervalUs;
        }
        previousMotorFeedbackIntervalWallClockUs = row.wallClockUs;
    }
    lines.append(QStringLiteral("数据来源\tRuntime Trace feedback"));
    lines.append(QStringLiteral("位置单位\t原始脉冲"));
    lines.append(QStringLiteral("最小相邻记录间隔(us)\t%1")
                 .arg(minIntervalUsText(minMotorFeedbackIntervalUs)));
    lines.append(QStringLiteral("记录数量\t%1").arg(motorFeedbackRows.size()));

    QString motorFeedbackHeader =
            QStringLiteral("时间戳(us精度)\t相对时间(ms)\t与上一条间隔(us)\t记录(0起)\tTrace帧序号");
    for(int axisIndex = 0; axisIndex < feedbackAxisCount; ++axisIndex){
        motorFeedbackHeader.append(QStringLiteral("\taxis_%1_feedback_raw_pulse").arg(axisIndex + 1));
    }
    lines.append(motorFeedbackHeader);
    qint64 previousMotorFeedbackWallClockUs = -1;
    for(int rowIndex = 0; rowIndex < motorFeedbackRows.size(); ++rowIndex){
        const SessionMotorFeedbackOutputRow& row = motorFeedbackRows[rowIndex];
        const double relativeMs =
                static_cast<double>(std::max<qint64>(
                                        0,
                                        row.wallClockUs - sessionRecordingState.startedAtMs * 1000)) / 1000.0;
        QString line = QStringLiteral("%1\t%2\t%3\t%4\t%5")
                .arg(formatIsoDateTimeUs(row.wallClockUs))
                .arg(QString::number(relativeMs, 'f', 3))
                .arg(intervalUsText(row.wallClockUs, previousMotorFeedbackWallClockUs))
                .arg(rowIndex)
                .arg(row.frameSequenceValid ?
                         QString::number(row.frameSequence) :
                         QString());
        for(int axisIndex = 0; axisIndex < feedbackAxisCount; ++axisIndex){
            const bool valid = axisIndex < static_cast<int>(row.feedbackValid.size()) &&
                    row.feedbackValid[axisIndex] &&
                    axisIndex < static_cast<int>(row.feedbackRawPulse.size());
            line.append(QStringLiteral("\t%1")
                        .arg(valid ?
                                 QString::number(row.feedbackRawPulse[axisIndex]) :
                                 QString()));
        }
        lines.append(line);
        previousMotorFeedbackWallClockUs = row.wallClockUs;
    }
    if(motorFeedbackRows.isEmpty()){
        lines.append(QStringLiteral("无有效数据\t\t\t\t\t当前记录阶段未记录到 Runtime Trace feedback 原始脉冲"));
    }

    appendStandardSection(QStringLiteral("电机速度"),
                          QStringLiteral("记录时刻各电机的速度反馈。"),
                          QStringLiteral("motor_vel"));
    appendStandardSection(QStringLiteral("电机力矩(Nm)"),
                          QStringLiteral("记录时刻各电机的实际力矩反馈。"),
                          QStringLiteral("motor_torque_nm"));
    appendStandardSection(QStringLiteral("绳索位移"),
                          QStringLiteral("按当前建模轴顺序整理的绳索位移。"),
                          QStringLiteral("cable_displacement"),
                          QStringLiteral("cable_displacement_available"));
    appendStandardSection(QStringLiteral("绳长"),
                          QStringLiteral("零位校准确认后可用，表示当前绳长。"),
                          QStringLiteral("cable_length"),
                          QStringLiteral("cable_length_available"));

    appendSectionTitle(QStringLiteral("轨迹点"));
    lines.append(QStringLiteral("说明\t记录时刻的末端轨迹点，来源遵循当前位姿来源配置。"));
    lines.append(QStringLiteral("最小相邻记录间隔(us)\t%1")
                 .arg(minIntervalUsText(minAdjacentRelativeUsForKey(QStringLiteral("trajectory_point")))));
    lines.append(QStringLiteral("时间戳\t相对时间(ms)\t与上一条间隔(us)\t轨迹点来源\t数据"));
    bool hasTrajectoryPoint = false;
    qint64 previousTrajectoryRelativeMs = -1;
    for(const SessionRecordingSample& sample : sessionRecordingState.samples){
        if(!sample.trajectoryPointAvailable || sample.trajectoryPoint.empty()){
            continue;
        }
        const qint64 relativeMs = sample.relativeMs;
        lines.append(QStringLiteral("%1\t%2\t%3\t%4\t%5")
                     .arg(sampleCapturedAtText(sample))
                     .arg(relativeMs)
                     .arg(intervalUsTextFromMs(relativeMs, previousTrajectoryRelativeMs))
                     .arg(sessionRecordingPoseSourceText(sample.trajectoryPointSource))
                     .arg(jsonValueToReadableText(toJsonArray(sample.trajectoryPoint))));
        hasTrajectoryPoint = true;
    }
    if(!hasTrajectoryPoint){
        lines.append(QStringLiteral("无有效数据\t\t\t轨迹点不可用\t当前记录阶段未获取到有效末端轨迹点"));
    }

    return lines.join(QStringLiteral("\r\n"));
}

bool MainWindow::writeSessionRecordingDiagnosticRawSections(QTextStream& stream,
                                                             qint64 sourceRowIndexOffset,
                                                             QString* errorMessage)
{
    if(errorMessage){
        errorMessage->clear();
    }

    CsvExport::SessionState csvState;
    csvState.sourceRowIndex = sourceRowIndexOffset;

    const qint64 startMs = sessionRecordingState.startedAtMs;
    const qint64 endMs = sessionRecordingState.endedAtMs > 0 ?
                sessionRecordingState.endedAtMs :
                QDateTime::currentMSecsSinceEpoch();
    if(startMs <= 0 || endMs <= 0 || endMs < startMs){
        return CsvExport::writeSessionSourceLine(stream, QString(), csvState) &&
                CsvExport::writeSessionSourceLine(stream, QStringLiteral("[通信频率原始数据]"), csvState) &&
                CsvExport::writeSessionSourceLine(stream,
                                            QStringLiteral("说明\t当前会话没有有效的起止时间，无法导出通信频率数据。"),
                                            csvState) &&
                CsvExport::writeSessionSourceLine(stream,
                                            QStringLiteral("最小相邻记录间隔\t无"),
                                            csvState);
    }

    int pendingLines = 0;
    auto pumpMainThread = [this, &stream, &pendingLines](bool force = false) -> bool {
        if(!force && pendingLines < kDiagnosticRawWritePumpLines){
            return true;
        }
        pendingLines = 0;
        stream.flush();
        refreshSafetyMonitorHeartbeat();
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 5);
        refreshSafetyMonitorHeartbeat();
        return stream.status() == QTextStream::Ok;
    };

    auto writeLine = [&](const QString& line) -> bool {
        if(!CsvExport::writeSessionSourceLine(stream, line, csvState)){
            return false;
        }
        ++pendingLines;
        return pumpMainThread(false);
    };
    auto usText = [](qint64 value) -> QString {
        return value >= 0 ? QString::number(value) : QString();
    };
    auto doubleText = [](double value, int precision = 3) -> QString {
        return std::isfinite(value) && value >= 0.0 ?
                    QString::number(value, 'f', precision) :
                    QString();
    };
    auto minIntervalUsText = [](qint64 value) -> QString {
        return value >= 0 ? QString::number(value) : QStringLiteral("无");
    };
    auto minIntervalUsDoubleText = [](double valueUs) -> QString {
        return std::isfinite(valueUs) && valueUs >= 0.0 ?
                    QString::number(qRound64(valueUs)) :
                    QStringLiteral("无");
    };
    auto hardwareDiagnosticMinIntervalUs =
            [](const QVector<HardwareInterface::DiagnosticRawSample>& samples) -> qint64 {
        qint64 minIntervalUs = -1;
        for(const HardwareInterface::DiagnosticRawSample& sample : samples){
            if(sample.intervalUs <= 0){
                continue;
            }
            minIntervalUs = minIntervalUs >= 0 ?
                        std::min(minIntervalUs, sample.intervalUs) :
                        sample.intervalUs;
        }
        return minIntervalUs;
    };
    auto pvtControlCycleRecordMinIntervalUs =
            [](const SessionRecordingPvtControlCycleRecord& record) -> double {
        double minIntervalUs = std::numeric_limits<double>::quiet_NaN();
        double previousTimeSec = std::numeric_limits<double>::quiet_NaN();
        const int rowCount = record.pointCount > 0 ?
                    std::min(record.pointCount,
                             static_cast<int>(record.points.size())) :
                    static_cast<int>(record.points.size());
        for(int pointRow = 0; pointRow < rowCount; ++pointRow){
            const SessionRecordingPvtControlCyclePoint& point = record.points[pointRow];
            if(!std::isfinite(point.trajectoryTimeSec)){
                continue;
            }
            if(std::isfinite(previousTimeSec)){
                const double intervalUs =
                        std::max(0.0, (point.trajectoryTimeSec - previousTimeSec) * 1000000.0);
                minIntervalUs = std::isfinite(minIntervalUs) ?
                            std::min(minIntervalUs, intervalUs) :
                            intervalUs;
            }
            previousTimeSec = point.trajectoryTimeSec;
        }
        return minIntervalUs;
    };

    const QVector<HardwareInterface::DiagnosticRawSample> communicationHistory =
            hardwareInterface.communicationTimingHistory(startMs, endMs);
    refreshSafetyMonitorHeartbeat();
    const QVector<HardwareInterface::RuntimeTraceFetchTimingSample> runtimeTraceFetchHistory =
            hardwareInterface.runtimeTraceFetchTimingHistory(startMs, endMs);
    refreshSafetyMonitorHeartbeat();
    const HardwareInterface::FieldbusConsumeTimeSnapshot fieldbusConsumeTime =
            hardwareInterface.fieldbusConsumeTimeSnapshot();
    refreshSafetyMonitorHeartbeat();

    auto writePvtControlCycleSamples = [&]() -> bool {
        double minPvtControlCycleIntervalUs = std::numeric_limits<double>::quiet_NaN();
        for(const SessionRecordingPvtControlCycleRecord& record :
            sessionRecordingState.pvtControlCycleRecords){
            const double recordMinIntervalUs = pvtControlCycleRecordMinIntervalUs(record);
            if(!std::isfinite(recordMinIntervalUs)){
                continue;
            }
            minPvtControlCycleIntervalUs = std::isfinite(minPvtControlCycleIntervalUs) ?
                        std::min(minPvtControlCycleIntervalUs, recordMinIntervalUs) :
                        recordMinIntervalUs;
        }
        if(!writeLine(QString()) ||
                !writeLine(QStringLiteral("[控制周期原始数据]")) ||
                !writeLine(QStringLiteral("说明\t每一段位控PVT轨迹生成完成后记录逐点计算耗时；绳索长度计算用时来自仿真轨迹点，barycenter结算用时来自逐点重心法求解。")) ||
                !writeLine(QStringLiteral("说明\tdmc_pvts_table_unit下发总耗时为逐轴调用该API的累计用时；平均下发用时=下发总耗时/轨迹点数。")) ||
                !writeLine(QStringLiteral("说明\t本节为指令反馈帧序号差计算结果。")) ||
                !writeLine(QStringLiteral("说明\tTrace响应间隔=(反馈位置首次变化帧序号-指令位置首次变化帧序号)×EtherCAT总线周期；控制周期总时间=计算用时合计+平均下发用时+Trace响应间隔。")) ||
                !writeLine(QStringLiteral("说明\t反馈首次变化只接受已出现指令变化且反馈相对启动前基线同向变化的绳索轴，避免静止反馈抖动被误判为开始运动。")) ||
                !writeLine(QStringLiteral("最小相邻记录间隔(us)\t%1")
                           .arg(minIntervalUsDoubleText(minPvtControlCycleIntervalUs)))){
            return false;
        }

        if(sessionRecordingState.pvtControlCycleRecords.empty()){
            return writeLine(QStringLiteral("无有效数据\t当前记录阶段未捕获到PVT控制周期逐点原始数据"));
        }

        for(int recordIndex = 0;
            recordIndex < static_cast<int>(sessionRecordingState.pvtControlCycleRecords.size());
            ++recordIndex){
            const SessionRecordingPvtControlCycleRecord& record =
                    sessionRecordingState.pvtControlCycleRecords[recordIndex];
            const double recordMinIntervalUs = pvtControlCycleRecordMinIntervalUs(record);
            const double recordControlCycleMaxUs = pvtControlCycleMaxUs(record);
            if(!writeLine(QStringLiteral("轨迹段\t%1").arg(recordIndex + 1)) ||
                    !writeLine(QStringLiteral("来源\t%1").arg(record.source)) ||
                    !writeLine(QStringLiteral("记录时间\t%1")
                               .arg(QDateTime::fromMSecsSinceEpoch(record.capturedAtMs)
                                    .toString(Qt::ISODateWithMs))) ||
                    !writeLine(QStringLiteral("相对记录开始(ms)\t%1")
                               .arg(std::max<qint64>(0, record.capturedAtMs - startMs))) ||
                    !writeLine(QStringLiteral("仿真源起始点(0起)\t%1")
                               .arg(record.sourceStartPointIndex)) ||
                    !writeLine(QStringLiteral("轨迹点数\t%1").arg(record.pointCount)) ||
                    !writeLine(QStringLiteral("最小相邻记录间隔(us)\t%1")
                               .arg(minIntervalUsDoubleText(recordMinIntervalUs))) ||
                    !writeLine(QStringLiteral("控制电机数\t%1").arg(record.axisCount)) ||
                    !writeLine(QStringLiteral("PVT表生成总耗时(us)\t%1")
                               .arg(usText(record.pvtGenerationElapsedUs))) ||
                    !writeLine(QStringLiteral("dmc_pvts_table_unit下发总耗时(us)\t%1")
                               .arg(usText(record.pvtUploadTotalUs))) ||
                    !writeLine(QStringLiteral("平均下发用时(us/轨迹点)\t%1")
                               .arg(doubleText(record.pvtUploadAverageUsPerPoint))) ||
                    !writeLine(QStringLiteral("下发匹配点数\t%1").arg(record.pvtUploadPointCount)) ||
                    !writeLine(QStringLiteral("下发匹配轴数\t%1").arg(record.pvtUploadAxisCount)) ||
                    !writeLine(QStringLiteral("Trace响应间隔状态\t%1")
                               .arg(record.traceStartDelayValid ?
                                        QStringLiteral("有效") :
                                        QStringLiteral("未捕获到指令/反馈首次变化帧"))) ||
                    !writeLine(QStringLiteral("EtherCAT总线周期(us)\t%1")
                               .arg(std::max(1, record.ethercatBusCycleUs))) ||
                    !writeLine(QStringLiteral("EtherCAT总线周期(ms)\t%1")
                               .arg(QString::number(
                                        static_cast<double>(std::max(
                                                                1,
                                                                record.ethercatBusCycleUs)) /
                                            1000.0,
                                        'f',
                                        3))) ||
                    !writeLine(QStringLiteral("Trace指令位置首次变化帧序号\t%1")
                               .arg(record.traceStartDelayValid ?
                                        QString::number(record.traceCommandStartFrameSequence) :
                                        QString())) ||
                    !writeLine(QStringLiteral("Trace反馈位置首次变化帧序号\t%1")
                               .arg(record.traceStartDelayValid ?
                                        QString::number(record.traceFeedbackStartFrameSequence) :
                                        QString())) ||
                    !writeLine(QStringLiteral("Trace帧序号间隔数\t%1")
                               .arg(record.traceStartDelayValid ?
                                        QString::number(record.traceStartDelayFrameCount) :
                                        QString())) ||
                    !writeLine(QStringLiteral("Trace响应间隔(us)\t%1")
                               .arg(record.traceStartDelayValid ?
                                        QString::number(record.traceStartDelayUs) :
                                        QString())) ||
                    !writeLine(QStringLiteral("Trace指令首次变化轴(0起)\t%1")
                               .arg(record.traceStartDelayValid ?
                                        QString::number(record.traceCommandStartAxis) :
                                        QString())) ||
                    !writeLine(QStringLiteral("Trace反馈首次变化轴(0起)\t%1")
                               .arg(record.traceStartDelayValid ?
                                        QString::number(record.traceFeedbackStartAxis) :
                                        QString())) ||
                    !writeLine(QStringLiteral("控制周期最大值(us)\t%1")
                               .arg(doubleText(recordControlCycleMaxUs))) ||
                    !writeLine(QStringLiteral("控制周期最大值(ms)\t%1")
                               .arg(std::isfinite(recordControlCycleMaxUs) ?
                                        QString::number(recordControlCycleMaxUs / 1000.0,
                                                        'f',
                                                        6) :
                                        QString())) ||
                    !writeLine(QStringLiteral("轨迹点(0起)\t轨迹时间(s)\t与上一条间隔(us)\t绳索长度计算用时(us)\tbarycenter结算用时(us)\t计算用时合计(us)\t平均下发用时(us/轨迹点)\t计算+平均下发用时(us)\tTrace响应间隔(us)\t控制周期总时间(us)"))){
                return false;
            }

            const int rowCount = record.pointCount > 0 ?
                        std::min(record.pointCount,
                                 static_cast<int>(record.points.size())) :
                        static_cast<int>(record.points.size());
            if(rowCount <= 0){
                if(!writeLine(QStringLiteral("无有效数据\t\t\t\t\t\t\t该轨迹段没有逐点计时记录"))){
                    return false;
                }
                continue;
            }
            double previousPointTimeSec = std::numeric_limits<double>::quiet_NaN();
            for(int pointRow = 0; pointRow < rowCount; ++pointRow){
                const SessionRecordingPvtControlCyclePoint& point =
                        record.points[pointRow];
                const bool hasCableTiming = point.cableLengthCalculationUs >= 0;
                const bool hasBarycenterTiming = point.barycenterSolveUs >= 0;
                const qint64 calculationTotalUs =
                        (hasCableTiming || hasBarycenterTiming) ?
                            std::max<qint64>(0, point.cableLengthCalculationUs) +
                            std::max<qint64>(0, point.barycenterSolveUs) :
                            -1;
                const double calculationPlusUploadUs =
                        calculationTotalUs >= 0 &&
                        std::isfinite(record.pvtUploadAverageUsPerPoint) &&
                        record.pvtUploadAverageUsPerPoint >= 0.0 ?
                            static_cast<double>(calculationTotalUs) +
                            record.pvtUploadAverageUsPerPoint :
                            -1.0;
                const double controlCycleTotalUs =
                        calculationPlusUploadUs >= 0.0 &&
                        record.traceStartDelayValid &&
                        record.traceStartDelayUs >= 0 ?
                            calculationPlusUploadUs +
                            static_cast<double>(record.traceStartDelayUs) :
                            -1.0;
                QString pointIntervalUsText;
                if(std::isfinite(point.trajectoryTimeSec)){
                    pointIntervalUsText = std::isfinite(previousPointTimeSec) ?
                                QString::number(qRound64(std::max(
                                                             0.0,
                                                             (point.trajectoryTimeSec - previousPointTimeSec) * 1000000.0))) :
                                QStringLiteral("0");
                    previousPointTimeSec = point.trajectoryTimeSec;
                }
                if(!writeLine(QStringLiteral("%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\t%9\t%10")
                              .arg(point.pointIndex)
                              .arg(doubleText(point.trajectoryTimeSec, 6))
                              .arg(pointIntervalUsText)
                              .arg(usText(point.cableLengthCalculationUs))
                              .arg(usText(point.barycenterSolveUs))
                              .arg(usText(calculationTotalUs))
                              .arg(doubleText(record.pvtUploadAverageUsPerPoint))
                              .arg(doubleText(calculationPlusUploadUs))
                              .arg(record.traceStartDelayValid ?
                                       QString::number(record.traceStartDelayUs) :
                                       QString())
                              .arg(doubleText(controlCycleTotalUs)))){
                    return false;
                }
            }
        }
        return true;
    };

    auto writeFieldbusControlCycleSamples = [&]() -> bool {
        const QString validText = fieldbusConsumeTime.success ?
                    QStringLiteral("有效") :
                    QStringLiteral("无效");
        const QString averageTimeText = fieldbusConsumeTime.success ?
                    QString::number(fieldbusConsumeTime.averageTimeUs) :
                    QString();
        const QString maxTimeText = fieldbusConsumeTime.success ?
                    QString::number(fieldbusConsumeTime.maxTimeUs) :
                    QString();
        const QString cyclesText = fieldbusConsumeTime.success ?
                    QString::number(fieldbusConsumeTime.cycles) :
                    QString();

        if(!writeLine(QString()) ||
                !writeLine(QStringLiteral("[通信频率原始数据]")) ||
                !writeLine(QStringLiteral("说明\t本节使用 nmc_get_consume_time_fieldbus 读取 EtherCAT 总线周期统计；端口号固定为2。")) ||
                !writeLine(QStringLiteral("最小相邻记录间隔(us)\t不适用")) ||
                !writeLine(QStringLiteral("读取时间\t%1")
                           .arg(QDateTime::fromMSecsSinceEpoch(
                                    fieldbusConsumeTime.wallClockMs)
                                .toString(Qt::ISODateWithMs))) ||
                !writeLine(QStringLiteral("控制卡卡号\t%1").arg(fieldbusConsumeTime.cardNo)) ||
                !writeLine(QStringLiteral("EtherCAT端口号\t%1").arg(fieldbusConsumeTime.portNum)) ||
                !writeLine(QStringLiteral("API返回码\t%1").arg(fieldbusConsumeTime.apiResult)) ||
                !writeLine(QStringLiteral("数据状态\t%1").arg(validText)) ||
                !writeLine(QStringLiteral("平均周期(us)\t%1").arg(averageTimeText)) ||
                !writeLine(QStringLiteral("最大周期(us)\t%1").arg(maxTimeText)) ||
                !writeLine(QStringLiteral("执行周期数\t%1").arg(cyclesText))){
            return false;
        }
        if(!fieldbusConsumeTime.success){
            return writeLine(QStringLiteral("无有效数据\tnmc_get_consume_time_fieldbus 调用失败或控制卡未连接"));
        }
        return true;
    };

    auto writeHardwareSamples = [&](const QString& title,
                                    const QString& description,
                                    const QVector<HardwareInterface::DiagnosticRawSample>& samples) -> bool {
        if(!writeLine(QString()) ||
                !writeLine(title) ||
                !writeLine(QStringLiteral("说明\t%1").arg(description)) ||
                !writeLine(QStringLiteral("最小相邻记录间隔(us)\t%1")
                           .arg(minIntervalUsText(hardwareDiagnosticMinIntervalUs(samples)))) ||
                !writeLine(QStringLiteral("时间戳\t相对时间(ms)\t与上一条间隔(us)\t雷赛硬件API读写事件"))){
            return false;
        }
        if(samples.isEmpty()){
            return writeLine(QStringLiteral("无有效数据\t\t0\t"));
        }
        for(const HardwareInterface::DiagnosticRawSample& sample : samples){
            if(!writeLine(QStringLiteral("%1\t%2\t%3\t%4")
                          .arg(QDateTime::fromMSecsSinceEpoch(sample.wallClockMs).toString(Qt::ISODateWithMs))
                          .arg(std::max<qint64>(0, sample.wallClockMs - startMs))
                          .arg(sample.intervalUs)
                          .arg(sample.apiEvent))){
                return false;
            }
        }
        return true;
    };

    auto writeTraceFetchFrameCountFrequencySamples =
            [&](const QVector<HardwareInterface::RuntimeTraceFetchTimingSample>& samples) -> bool {
        struct ExpandedTraceFetchFrame {
            qint64 estimatedWallClockUs = 0;
            qint64 estimatedIntervalUs = 0;
            bool intervalOutlier = false;
            int batchIndex = -1;
            int frameIndex = -1;
            quint32 frameSequence = 0;
            qint64 fetchCompleteWallClockUs = 0;
            qint64 fetchIntervalUs = 0;
            qint64 apiDurationUs = 0;
            int frameCount = 0;
            quint32 firstFrameSequence = 0;
            quint32 lastFrameSequence = 0;
            quint64 sequenceDenominator = 0;
            double averageFrameIntervalUs = std::numeric_limits<double>::quiet_NaN();
            QString intervalSource;
        };

        auto sequenceDelta = [](quint32 previous, quint32 current) -> quint64 {
            if(current >= previous){
                return static_cast<quint64>(current - previous);
            }
            return (static_cast<quint64>(std::numeric_limits<quint32>::max()) -
                    static_cast<quint64>(previous)) +
                    static_cast<quint64>(current) + 1ULL;
        };
        constexpr qint64 kTraceFetchFrameIntervalOutlierThresholdUs = 1000;

        QVector<ExpandedTraceFetchFrame> expandedFrames;
        expandedFrames.reserve(samples.size());
        int validBatchCount = 0;
        int multiFrameBatchCount = 0;
        int skippedBatchCount = 0;
        bool hasPreviousValidBatch = false;
        quint32 previousLastFrameSequence = 0;
        for(int batchIndex = 0; batchIndex < samples.size(); ++batchIndex){
            const HardwareInterface::RuntimeTraceFetchTimingSample& sample =
                    samples[batchIndex];
            const int frameCount = sample.frameCount;
            if(frameCount <= 0 ||
                    !sample.frameSequenceValid ||
                    static_cast<int>(sample.frameSequences.size()) < frameCount ||
                    sample.wallClockUs <= 0){
                ++skippedBatchCount;
                continue;
            }

            ++validBatchCount;
            if(frameCount > 1){
                ++multiFrameBatchCount;
            }

            quint64 denominator = 0;
            QString intervalSource;
            if(hasPreviousValidBatch){
                denominator = sequenceDelta(previousLastFrameSequence,
                                            sample.lastFrameSequence);
                intervalSource = QStringLiteral("相邻批次完成间隔/帧序号增量");
            }
            if(denominator == 0){
                denominator = static_cast<quint64>(frameCount);
                intervalSource = QStringLiteral("本批完成间隔/本批返回帧数");
            }
            const qint64 intervalBasisUs = sample.intervalUs > 0 ?
                        sample.intervalUs :
                        sample.apiDurationUs;
            const bool queueTimelineValid = sample.traceSamplePeriodUs > 0 &&
                    sample.newestFrameAgeUs >= 0;
            if(queueTimelineValid){
                intervalSource = QStringLiteral("Trace周期读回/FIFO余量");
            }
            const double averageFrameIntervalUs = queueTimelineValid ?
                        static_cast<double>(sample.traceSamplePeriodUs) :
                        (denominator > 0 ?
                        static_cast<double>(std::max<qint64>(0, intervalBasisUs)) /
                            static_cast<double>(denominator) :
                        std::numeric_limits<double>::quiet_NaN());
            const qint64 lastFrameWallClockUs = queueTimelineValid ?
                        sample.wallClockUs - sample.newestFrameAgeUs :
                        sample.wallClockUs;

            for(int frameIndex = 0; frameIndex < frameCount; ++frameIndex){
                const quint32 frameSequence = sample.frameSequences[frameIndex];
                const quint64 framesBeforeLast = queueTimelineValid ?
                            static_cast<quint64>(frameCount - frameIndex - 1) :
                            sequenceDelta(frameSequence, sample.lastFrameSequence);
                const qint64 estimatedWallClockUs =
                        std::isfinite(averageFrameIntervalUs) ?
                            lastFrameWallClockUs -
                                qRound64(averageFrameIntervalUs *
                                         static_cast<double>(framesBeforeLast)) :
                            lastFrameWallClockUs;

                ExpandedTraceFetchFrame frame;
                frame.estimatedWallClockUs = estimatedWallClockUs;
                frame.batchIndex = batchIndex;
                frame.frameIndex = frameIndex;
                frame.frameSequence = frameSequence;
                frame.fetchCompleteWallClockUs = sample.wallClockUs;
                frame.fetchIntervalUs = sample.intervalUs;
                frame.apiDurationUs = sample.apiDurationUs;
                frame.frameCount = frameCount;
                frame.firstFrameSequence = sample.firstFrameSequence;
                frame.lastFrameSequence = sample.lastFrameSequence;
                frame.sequenceDenominator = denominator;
                frame.averageFrameIntervalUs = averageFrameIntervalUs;
                frame.intervalSource = intervalSource;
                expandedFrames.append(frame);
            }

            previousLastFrameSequence = sample.lastFrameSequence;
            hasPreviousValidBatch = true;
        }

        std::stable_sort(expandedFrames.begin(),
                         expandedFrames.end(),
                         [](const ExpandedTraceFetchFrame& lhs,
                            const ExpandedTraceFetchFrame& rhs){
            if(lhs.estimatedWallClockUs == rhs.estimatedWallClockUs){
                if(lhs.batchIndex == rhs.batchIndex){
                    return lhs.frameIndex < rhs.frameIndex;
                }
                return lhs.batchIndex < rhs.batchIndex;
            }
            return lhs.estimatedWallClockUs < rhs.estimatedWallClockUs;
        });

        QVector<ExpandedTraceFetchFrame> filteredFrames;
        filteredFrames.reserve(expandedFrames.size());
        qint64 intervalCount = 0;
        qint64 intervalSumUs = 0;
        qint64 minIntervalUs = std::numeric_limits<qint64>::max();
        qint64 maxIntervalUs = 0;
        qint64 previousEstimatedWallClockUs = -1;
        for(ExpandedTraceFetchFrame& frame : expandedFrames){
            if(previousEstimatedWallClockUs > 0){
                const qint64 intervalUs =
                        std::max<qint64>(0,
                                         frame.estimatedWallClockUs -
                                         previousEstimatedWallClockUs);
                frame.estimatedIntervalUs = intervalUs;
                if(intervalUs > kTraceFetchFrameIntervalOutlierThresholdUs){
                    frame.intervalOutlier = true;
                }
                else{
                    ++intervalCount;
                    intervalSumUs += intervalUs;
                    minIntervalUs = std::min(minIntervalUs, intervalUs);
                    maxIntervalUs = std::max(maxIntervalUs, intervalUs);
                }
            }
            if(!frame.intervalOutlier){
                filteredFrames.append(frame);
            }
            previousEstimatedWallClockUs = frame.estimatedWallClockUs;
        }

        const bool hasIntervals = intervalCount > 0;
        const bool satisfy1000Hz = hasIntervals && maxIntervalUs <= kTraceFetchFrameIntervalOutlierThresholdUs;
        if(!writeLine(QString()) ||
                !writeLine(QStringLiteral("[采集频率原始数据]")) ||
                !writeLine(QStringLiteral("说明\t本节优先使用Trace周期读回值和排空后的FIFO余量重建逐帧时间；旧记录缺少这些字段时才回退到相邻获取完成时间。")) ||
                !writeLine(QStringLiteral("说明\t本次先只展开帧序号，不展开每帧传感器值；API调用耗时和FIFO守恒字段用于判断读取是否追上生产端。")) ||
                !writeLine(QStringLiteral("Trace批量读取次数\t%1").arg(samples.size())) ||
                !writeLine(QStringLiteral("有效批量读取次数\t%1").arg(validBatchCount)) ||
                !writeLine(QStringLiteral("多帧批量读取次数\t%1").arg(multiFrameBatchCount)) ||
                !writeLine(QStringLiteral("跳过批量读取次数\t%1").arg(skippedBatchCount)) ||
                !writeLine(QStringLiteral("展开Trace帧数量\t%1").arg(filteredFrames.size())) ||
                !writeLine(QStringLiteral("有效估算间隔数量\t%1").arg(intervalCount))){
            return false;
        }

        if(hasIntervals){
            const double averageIntervalUs =
                    static_cast<double>(intervalSumUs) /
                    static_cast<double>(intervalCount);
            if(!writeLine(QStringLiteral("平均估算帧间隔(us)\t%1")
                          .arg(QString::number(averageIntervalUs, 'f', 3))) ||
                    !writeLine(QStringLiteral("最小估算帧间隔(us)\t%1").arg(minIntervalUs)) ||
                    !writeLine(QStringLiteral("最大估算帧间隔(us)\t%1").arg(maxIntervalUs)) ||
                    !writeLine(QStringLiteral("是否满足全程采集频率不低于1000Hz\t%1")
                               .arg(satisfy1000Hz ?
                                        QStringLiteral("是") :
                                        QStringLiteral("否")))){
                return false;
            }
        }
        else if(!writeLine(QStringLiteral("平均估算帧间隔(us)\t")) ||
                !writeLine(QStringLiteral("最小估算帧间隔(us)\t")) ||
                !writeLine(QStringLiteral("最大估算帧间隔(us)\t")) ||
                !writeLine(QStringLiteral("是否满足全程采集频率不低于1000Hz\t否"))){
            return false;
        }

        if(!writeLine(QString()) ||
                !writeLine(QStringLiteral("Trace FIFO批次诊断")) ||
                !writeLine(QStringLiteral("批次(0起)\t获取完成时间\tAPI调用耗时(us)\t请求帧数\t返回帧数\t返回字节\t固定帧宽\t读前有效帧\t读后有效帧\t读后空闲帧\t本次估算新增帧\tTrace周期(us)\t最新已读帧年龄(us)\tFIFO已追平\t时间轴可信\tTrace丢帧\t帧序号范围"))){
            return false;
        }
        if(samples.isEmpty()){
            if(!writeLine(QStringLiteral("无有效数据"))){
                return false;
            }
        }
        else{
            for(int batchIndex = 0; batchIndex < samples.size(); ++batchIndex){
                const HardwareInterface::RuntimeTraceFetchTimingSample& sample =
                        samples[batchIndex];
                const QString sequenceRange = sample.frameSequenceValid ?
                            QStringLiteral("%1-%2")
                                .arg(sample.firstFrameSequence)
                                .arg(sample.lastFrameSequence) :
                            QStringLiteral("无效");
                if(!writeLine(QStringLiteral("%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\t%9\t%10\t%11\t%12\t%13\t%14\t%15\t%16\t%17")
                              .arg(batchIndex)
                              .arg(formatIsoDateTimeUs(sample.wallClockUs))
                              .arg(sample.apiDurationUs)
                              .arg(sample.requestedFrameCount)
                              .arg(sample.frameCount)
                              .arg(sample.actualReadLength)
                              .arg(sample.frameBytes)
                              .arg(sample.fifoValidBefore)
                              .arg(sample.fifoValidAfter)
                              .arg(sample.fifoFreeAfter)
                              .arg(sample.estimatedProducedFrameCount)
                              .arg(sample.traceSamplePeriodUs)
                              .arg(sample.newestFrameAgeUs)
                              .arg(sample.fifoCaughtUp ? QStringLiteral("是") : QStringLiteral("否"))
                              .arg(sample.timingReliable ? QStringLiteral("是") : QStringLiteral("否"))
                              .arg(sample.traceLost ? QStringLiteral("是") : QStringLiteral("否"))
                              .arg(sequenceRange))){
                    return false;
                }
            }
        }

        if(!writeLine(QStringLiteral("估算时间戳(us精度)\t相对时间(ms)\t与上一条估算间隔(us)\t批次(0起)\t批内帧(0起)\t帧序号\t本批返回帧数\t本批首帧序号\t本批末帧序号\t帧间隔分母\t本批平均帧间隔(us)\t本批获取完成间隔(us)\t本批API调用耗时(us)\t间隔计算来源"))){
            return false;
        }
        if(filteredFrames.isEmpty()){
            return writeLine(QStringLiteral("无有效数据\t\t\t\t\t\t\t\t\t\t\t\t\t当前记录阶段未捕获到可用帧序号的 Runtime Trace 批量读取记录"));
        }

        for(const ExpandedTraceFetchFrame& frame : filteredFrames){
            const double relativeMs =
                    static_cast<double>(std::max<qint64>(
                                            0,
                                            frame.estimatedWallClockUs -
                                            startMs * 1000)) / 1000.0;
            if(!writeLine(QStringLiteral("%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\t%9\t%10\t%11\t%12\t%13\t%14")
                          .arg(formatIsoDateTimeUs(frame.estimatedWallClockUs))
                          .arg(QString::number(relativeMs, 'f', 3))
                          .arg(frame.estimatedIntervalUs)
                          .arg(frame.batchIndex)
                          .arg(frame.frameIndex)
                          .arg(frame.frameSequence)
                          .arg(frame.frameCount)
                          .arg(frame.firstFrameSequence)
                          .arg(frame.lastFrameSequence)
                          .arg(frame.sequenceDenominator)
                          .arg(doubleText(frame.averageFrameIntervalUs))
                          .arg(frame.fetchIntervalUs)
                          .arg(frame.apiDurationUs)
                          .arg(frame.intervalSource))){
                return false;
            }
        }
        return true;
    };

    const bool writeOk =
            writeFieldbusControlCycleSamples() &&
            writePvtControlCycleSamples() &&
            writeTraceFetchFrameCountFrequencySamples(runtimeTraceFetchHistory) &&
            writeHardwareSamples(QStringLiteral("[实时通信频率原始数据]"),
                                 QStringLiteral("原始数据为相邻两次雷赛硬件 API 读写事件的时间间隔(us)，每条记录同时展示本次对应的API读写事件。"),
                                 communicationHistory);

    const bool flushOk = pumpMainThread(true);
    if(!writeOk || !flushOk || stream.status() != QTextStream::Ok){
        if(errorMessage){
            *errorMessage = QStringLiteral("写入会话诊断原始数据失败");
        }
        return false;
    }
    return true;
}

QString MainWindow::sessionRecordingExportNoDataReason() const
{
    if(sessionRecordingState.active){
        return QStringLiteral("当前会话仍在记录中，请先点击“结束记录”；已采集的内存数据仍会保留。");
    }
    if(sessionRecordingState.startedAtMs <= 0){
        return QStringLiteral("尚未开始会话记录。");
    }
    if(sessionRecordingState.samples.empty() &&
            sessionRecordingState.pvtPositionCommandTables.empty() &&
            sessionRecordingState.pvtControlCycleRecords.empty()){
        const qint64 startMs = sessionRecordingState.startedAtMs;
        const qint64 endMs = sessionRecordingState.endedAtMs > 0 ?
                    sessionRecordingState.endedAtMs :
                    QDateTime::currentMSecsSinceEpoch();
        if(kEnableSessionRecordMotorEncoderUnitSampling &&
                !hardwareInterface.motorEncoderRawHistory(startMs, endMs).isEmpty()){
            return QString();
        }
        return QStringLiteral("会话已结束，但未采集到控制快照样本、PVT位置控制指令表或控制周期记录。");
    }
    return QString();
}

bool MainWindow::writeSessionRecordingExport(QString* outputPath,
                                             bool announce,
                                             const QString& outputDirPath)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        if(announce){
            displayInfo("运行诊断记录与导出已由性能开关停用", "warning");
        }
        return false;
    }
    const QString noDataReason = sessionRecordingExportNoDataReason();
    if(!noDataReason.isEmpty()){
        if(announce){
            displayInfo(QStringLiteral("没有会话记录数据可导出：%1").arg(noDataReason).toStdString(),
                        "warning");
        }
        return false;
    }

    QString dirPath = outputDirPath.trimmed();
    if(dirPath.isEmpty() && announce){
        dirPath = selectOutputDirectoryWithEditableText(
                    this,
                    QStringLiteral("选择会话记录导出目录"),
                    QStringLiteral("会话记录导出目录"),
                    uiEventLogDirPath(),
                    QStringLiteral("导出"));
    }
    if(dirPath.isEmpty() && !announce){
        dirPath = uiEventLogDirPath();
    }
    const OutputDirectoryValidation dirValidation =
            validateExistingWritableOutputDirectory(dirPath);
    if(dirValidation.error == OutputDirectoryValidationError::Empty){
        if(announce){
            displayInfo("未选择会话记录导出目录，已取消导出；会话内存数据已保留",
                        "warning");
        }
        return false;
    }
    if(dirValidation.error == OutputDirectoryValidationError::NotExists){
        if(announce){
            displayInfo(QStringLiteral("错误：会话记录导出目录不存在：%1；会话内存数据已保留，请重新选择有效目录")
                        .arg(dirValidation.cleanPath).toStdString(),
                        "error");
        }
        return false;
    }
    if(dirValidation.error == OutputDirectoryValidationError::NotDirectory){
        if(announce){
            displayInfo(QStringLiteral("错误：会话记录导出路径不是目录：%1；会话内存数据已保留")
                        .arg(dirValidation.cleanPath).toStdString(),
                        "error");
        }
        return false;
    }
    if(dirValidation.error == OutputDirectoryValidationError::NotWritable){
        if(announce){
            displayInfo(QStringLiteral("错误：会话记录导出目录不可写：%1；请检查路径或权限，会话内存数据已保留")
                        .arg(dirValidation.cleanPath).toStdString(),
                        "error");
        }
        return false;
    }
    dirPath = dirValidation.cleanPath;

    const QString filePath = QDir(dirPath).filePath(
                QStringLiteral("session_record_%1.csv")
                .arg(QDateTime::fromMSecsSinceEpoch(std::max<qint64>(1, sessionRecordingState.startedAtMs))
                     .toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        if(announce){
            displayInfo(QStringLiteral("错误：无法写入会话导出文件 %1；请检查路径或权限，会话内存数据已保留")
                        .arg(filePath).toStdString(),
                        "error");
        }
        return false;
    }

    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif
    CsvExport::SessionState csvState;
    const bool sessionTextOk = CsvExport::writeSessionText(stream,
                                                     buildSessionRecordingExportText(),
                                                     csvState);
    QString rawDataError;
    const bool rawDataOk = sessionTextOk &&
            writeSessionRecordingDiagnosticRawSections(stream,
                                                        csvState.sourceRowIndex,
                                                        &rawDataError);
    stream.flush();
    const bool streamOk = stream.status() == QTextStream::Ok;
    const bool fileFlushOk = file.flush();
    file.close();

    if(!sessionTextOk || !rawDataOk || !streamOk || !fileFlushOk){
        if(announce){
            const QString detail = rawDataError.isEmpty() ?
                        QStringLiteral("写入会话导出文件失败") :
                        rawDataError;
            displayInfo(QStringLiteral("错误：%1 %2；请检查磁盘空间、路径或权限，会话内存数据已保留")
                        .arg(detail, filePath).toStdString(),
                        "error");
        }
        return false;
    }

    sessionRecordingState.lastExportPath = filePath;
    if(outputPath){
        *outputPath = filePath;
    }
    refreshSessionRecordingUi();
    if(announce){
        displayInfo(QStringLiteral("会话数据已导出至 %1（章节式CSV，按章节标题、说明/统计和横向数据表组织）")
                    .arg(filePath).toStdString());
    }
    return true;
}
