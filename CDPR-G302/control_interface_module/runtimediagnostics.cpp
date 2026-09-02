#include "runtimediagnostics.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "csvexportutils.h"
#include "outputpathvalidator.h"
#include "runtimejsoncodec.h"
#include "safetymonitor.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QList>
#include <QPushButton>
#include <QSaveFile>
#include <QTextStream>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace OutputPathValidator;
using namespace RuntimeDiagnostics;
using namespace RuntimeJsonCodec;

namespace {

constexpr int kUdpRealtimeJsonSendIntervalMs = 200;
constexpr int kUdpRealtimeV9FeedbackIntervalMs = 10;

int udpRealtimeSendIntervalMsForMode(bool v9FeedbackEnabled)
{
    return v9FeedbackEnabled ?
                kUdpRealtimeV9FeedbackIntervalMs :
                kUdpRealtimeJsonSendIntervalMs;
}

template <typename T>
T* findOptionalUiObject(const QWidget* root, const char* objectName)
{
    return root ? root->findChild<T*>(QString::fromLatin1(objectName)) : nullptr;
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

double frequencyHzFromIntervalUs(double intervalUs)
{
    if(!std::isfinite(intervalUs) || intervalUs <= 0.0){
        return 0.0;
    }
    return 1000000.0 / intervalUs;
}

void insertIntervalFrequencySummary(QJsonObject& object,
                                    qint64 minIntervalUs,
                                    qint64 maxIntervalUs,
                                    double averageIntervalUs,
                                    qint64 latestIntervalUs)
{
    object.insert(QStringLiteral("frequency_from_min_interval_hz"),
                  frequencyHzFromIntervalUs(static_cast<double>(minIntervalUs)));
    object.insert(QStringLiteral("frequency_from_max_interval_hz"),
                  frequencyHzFromIntervalUs(static_cast<double>(maxIntervalUs)));
    object.insert(QStringLiteral("average_frequency_hz"),
                  frequencyHzFromIntervalUs(averageIntervalUs));
    object.insert(QStringLiteral("latest_frequency_hz"),
                  frequencyHzFromIntervalUs(static_cast<double>(latestIntervalUs)));
}

QJsonObject buildIntervalCounterSummaryJson(quint64 intervalCount,
                                            qint64 intervalSumUs,
                                            qint64 latestIntervalUs,
                                            const QString& source)
{
    QJsonObject object;
    object.insert(QStringLiteral("source"), source);
    object.insert(QStringLiteral("interval_count"), jsonIntFromQuint64(intervalCount));
    object.insert(QStringLiteral("interval_sum_us"), intervalSumUs);
    object.insert(QStringLiteral("latest_interval_us"), latestIntervalUs);
    object.insert(QStringLiteral("has_interval_data"), intervalCount > 0);
    object.insert(QStringLiteral("min_max_available"), false);
    object.insert(QStringLiteral("note"),
                  intervalCount > 0 ?
                      QStringLiteral("当前窗口无原始间隔样本，使用累计计数/累计时长/最新间隔给出退化汇总。") :
                      QStringLiteral("当前窗口无有效间隔数据。"));
    const double averageIntervalUs = intervalCount > 0 ?
                static_cast<double>(intervalSumUs) / static_cast<double>(intervalCount) :
                0.0;
    object.insert(QStringLiteral("average_interval_us"), averageIntervalUs);
    insertIntervalFrequencySummary(object, 0, 0, averageIntervalUs, latestIntervalUs);
    return object;
}

template <typename Sample>
QJsonObject buildIntervalSampleSummaryJson(const QVector<Sample>& samples,
                                           qint64 windowStartMs,
                                           qint64 windowEndMs,
                                           const QString& source)
{
    QJsonObject object;
    object.insert(QStringLiteral("source"), source);
    object.insert(QStringLiteral("window_start_ms"), windowStartMs);
    object.insert(QStringLiteral("window_end_ms"), windowEndMs);
    object.insert(QStringLiteral("window_start"),
                  windowStartMs > 0 ?
                      QDateTime::fromMSecsSinceEpoch(windowStartMs).toString(Qt::ISODateWithMs) :
                      QString());
    object.insert(QStringLiteral("window_end"),
                  windowEndMs > 0 ?
                      QDateTime::fromMSecsSinceEpoch(windowEndMs).toString(Qt::ISODateWithMs) :
                      QString());
    object.insert(QStringLiteral("sample_count"), samples.size());

    qint64 minIntervalUs = std::numeric_limits<qint64>::max();
    qint64 maxIntervalUs = 0;
    qint64 intervalSumUs = 0;
    qint64 latestIntervalUs = 0;
    int intervalCount = 0;
    qint64 firstSampleMs = 0;
    qint64 lastSampleMs = 0;
    for(const Sample& sample : samples){
        if(firstSampleMs <= 0){
            firstSampleMs = sample.wallClockMs;
        }
        lastSampleMs = sample.wallClockMs;
        if(sample.intervalUs <= 0){
            continue;
        }
        minIntervalUs = std::min(minIntervalUs, sample.intervalUs);
        maxIntervalUs = std::max(maxIntervalUs, sample.intervalUs);
        intervalSumUs += sample.intervalUs;
        latestIntervalUs = sample.intervalUs;
        ++intervalCount;
    }

    object.insert(QStringLiteral("first_sample_ms"), firstSampleMs);
    object.insert(QStringLiteral("last_sample_ms"), lastSampleMs);
    object.insert(QStringLiteral("first_sample_at"),
                  firstSampleMs > 0 ?
                      QDateTime::fromMSecsSinceEpoch(firstSampleMs).toString(Qt::ISODateWithMs) :
                      QString());
    object.insert(QStringLiteral("last_sample_at"),
                  lastSampleMs > 0 ?
                      QDateTime::fromMSecsSinceEpoch(lastSampleMs).toString(Qt::ISODateWithMs) :
                      QString());
    object.insert(QStringLiteral("interval_count"), intervalCount);
    object.insert(QStringLiteral("has_interval_data"), intervalCount > 0);
    object.insert(QStringLiteral("min_max_available"), intervalCount > 0);
    if(intervalCount <= 0){
        object.insert(QStringLiteral("note"),
                      QStringLiteral("当前窗口无有效原始间隔样本；原始明细由 session record 导出承担。"));
        insertIntervalFrequencySummary(object, 0, 0, 0.0, 0);
        return object;
    }

    const double averageIntervalUs =
            static_cast<double>(intervalSumUs) / static_cast<double>(intervalCount);
    object.insert(QStringLiteral("min_interval_us"), minIntervalUs);
    object.insert(QStringLiteral("max_interval_us"), maxIntervalUs);
    object.insert(QStringLiteral("average_interval_us"), averageIntervalUs);
    object.insert(QStringLiteral("latest_interval_us"), latestIntervalUs);
    object.insert(QStringLiteral("interval_sum_us"), intervalSumUs);
    insertIntervalFrequencySummary(object,
                                   minIntervalUs,
                                   maxIntervalUs,
                                   averageIntervalUs,
                                   latestIntervalUs);
    return object;
}

QJsonObject buildVectorValueSummaryJson(const std::vector<double>& values,
                                        const QString& valuesKey)
{
    QJsonObject object;
    object.insert(QStringLiteral("value_count"), static_cast<int>(values.size()));
    object.insert(valuesKey, toJsonArray(values));

    double minValue = 0.0;
    double maxValue = 0.0;
    double sumValue = 0.0;
    int validCount = 0;
    for(double value : values){
        if(!std::isfinite(value)){
            continue;
        }
        if(validCount == 0){
            minValue = value;
            maxValue = value;
        }
        else{
            minValue = std::min(minValue, value);
            maxValue = std::max(maxValue, value);
        }
        sumValue += value;
        ++validCount;
    }

    object.insert(QStringLiteral("valid_value_count"), validCount);
    object.insert(QStringLiteral("has_valid_values"), validCount > 0);
    if(validCount > 0){
        object.insert(QStringLiteral("min_value"), minValue);
        object.insert(QStringLiteral("max_value"), maxValue);
        object.insert(QStringLiteral("average_value"), sumValue / static_cast<double>(validCount));
    }
    return object;
}

QJsonObject buildSensorValueHistorySummaryJson(const QVector<ControlWorker::SensorValueSample>& samples,
                                               qint64 windowStartMs,
                                               qint64 windowEndMs)
{
    QJsonObject object;
    object.insert(QStringLiteral("source"), QStringLiteral("ControlWorker::sensorValueHistory"));
    object.insert(QStringLiteral("window_start_ms"), windowStartMs);
    object.insert(QStringLiteral("window_end_ms"), windowEndMs);
    object.insert(QStringLiteral("sample_count"), samples.size());

    qint64 firstSampleMs = 0;
    qint64 lastSampleMs = 0;
    int traceSampleCount = 0;
    int expandedTraceSampleCount = 0;
    int validValueCount = 0;
    double minValue = 0.0;
    double maxValue = 0.0;
    double sumValue = 0.0;
    std::vector<double> latestValues;
    for(const ControlWorker::SensorValueSample& sample : samples){
        if(firstSampleMs <= 0){
            firstSampleMs = sample.wallClockMs;
        }
        lastSampleMs = sample.wallClockMs;
        if(sample.fromTrace){
            ++traceSampleCount;
        }
        if(sample.expandedTraceFrame){
            ++expandedTraceSampleCount;
        }
        latestValues = sample.values;
        for(double value : sample.values){
            if(!std::isfinite(value)){
                continue;
            }
            if(validValueCount == 0){
                minValue = value;
                maxValue = value;
            }
            else{
                minValue = std::min(minValue, value);
                maxValue = std::max(maxValue, value);
            }
            sumValue += value;
            ++validValueCount;
        }
    }

    object.insert(QStringLiteral("first_sample_ms"), firstSampleMs);
    object.insert(QStringLiteral("last_sample_ms"), lastSampleMs);
    object.insert(QStringLiteral("first_sample_at"),
                  firstSampleMs > 0 ?
                      QDateTime::fromMSecsSinceEpoch(firstSampleMs).toString(Qt::ISODateWithMs) :
                      QString());
    object.insert(QStringLiteral("last_sample_at"),
                  lastSampleMs > 0 ?
                      QDateTime::fromMSecsSinceEpoch(lastSampleMs).toString(Qt::ISODateWithMs) :
                      QString());
    object.insert(QStringLiteral("trace_sample_count"), traceSampleCount);
    object.insert(QStringLiteral("expanded_trace_sample_count"), expandedTraceSampleCount);
    object.insert(QStringLiteral("valid_value_count"), validValueCount);
    object.insert(QStringLiteral("has_valid_values"), validValueCount > 0);
    if(validValueCount > 0){
        object.insert(QStringLiteral("min_value"), minValue);
        object.insert(QStringLiteral("max_value"), maxValue);
        object.insert(QStringLiteral("average_value"), sumValue / static_cast<double>(validValueCount));
    }
    object.insert(QStringLiteral("latest_values"), toJsonArray(latestValues));
    return object;
}

QString safetyFaultCodeName(int faultCode)
{
    switch(static_cast<SafetyMonitor::FaultCode>(faultCode)){
    case SafetyMonitor::FaultCode::None:
        return QStringLiteral("none");
    case SafetyMonitor::FaultCode::SnapshotTimeout:
        return QStringLiteral("snapshot_timeout");
    case SafetyMonitor::FaultCode::HardwareDisconnected:
        return QStringLiteral("hardware_disconnected");
    case SafetyMonitor::FaultCode::CableForceLow:
        return QStringLiteral("cable_force_low");
    case SafetyMonitor::FaultCode::CableForceHigh:
        return QStringLiteral("cable_force_high");
    case SafetyMonitor::FaultCode::CableBreak:
        return QStringLiteral("cable_break");
    case SafetyMonitor::FaultCode::MotorRangeExceeded:
        return QStringLiteral("motor_range_exceeded");
    case SafetyMonitor::FaultCode::MotorOverspeed:
        return QStringLiteral("motor_overspeed");
    case SafetyMonitor::FaultCode::SensorInvalid:
        return QStringLiteral("sensor_invalid");
    case SafetyMonitor::FaultCode::WorkspaceExceeded:
        return QStringLiteral("workspace_exceeded");
    case SafetyMonitor::FaultCode::SoftwareHang:
        return QStringLiteral("software_hang");
    case SafetyMonitor::FaultCode::MotorTorqueExceeded:
        return QStringLiteral("motor_torque_exceeded");
    case SafetyMonitor::FaultCode::MotorFault:
        return QStringLiteral("motor_fault");
    case SafetyMonitor::FaultCode::PlcCommunicationFault:
        return QStringLiteral("plc_communication_fault");
    case SafetyMonitor::FaultCode::StartupSelfCheckFailed:
        return QStringLiteral("startup_self_check_failed");
    case SafetyMonitor::FaultCode::ControlBoxButtonNotReset:
        return QStringLiteral("control_box_button_not_reset");
    }
    return QStringLiteral("unknown");
}

QString safetyFaultDisplayName(int faultCode)
{
    switch(static_cast<SafetyMonitor::FaultCode>(faultCode)){
    case SafetyMonitor::FaultCode::None:
        return QStringLiteral("无故障/人工请求");
    case SafetyMonitor::FaultCode::SnapshotTimeout:
        return QStringLiteral("控制快照超时");
    case SafetyMonitor::FaultCode::HardwareDisconnected:
        return QStringLiteral("硬件或驱动通信异常");
    case SafetyMonitor::FaultCode::CableForceLow:
        return QStringLiteral("绳索张力低于下限");
    case SafetyMonitor::FaultCode::CableForceHigh:
        return QStringLiteral("绳索张力超过上限");
    case SafetyMonitor::FaultCode::CableBreak:
        return QStringLiteral("断绳/断崖式失张");
    case SafetyMonitor::FaultCode::MotorRangeExceeded:
        return QStringLiteral("电机位置越界");
    case SafetyMonitor::FaultCode::MotorOverspeed:
        return QStringLiteral("电机速度超限");
    case SafetyMonitor::FaultCode::SensorInvalid:
        return QStringLiteral("传感器或位姿反馈无效");
    case SafetyMonitor::FaultCode::WorkspaceExceeded:
        return QStringLiteral("工作空间越界");
    case SafetyMonitor::FaultCode::SoftwareHang:
        return QStringLiteral("软件卡死/看门狗超时");
    case SafetyMonitor::FaultCode::MotorTorqueExceeded:
        return QStringLiteral("电机实际力矩超限");
    case SafetyMonitor::FaultCode::MotorFault:
        return QStringLiteral("电机故障/运动参与电机失能");
    case SafetyMonitor::FaultCode::PlcCommunicationFault:
        return QStringLiteral("工控机与PLC通信断开");
    case SafetyMonitor::FaultCode::StartupSelfCheckFailed:
        return QStringLiteral("系统自检未通过");
    case SafetyMonitor::FaultCode::ControlBoxButtonNotReset:
        return QStringLiteral("手柄控制盒安全按钮未复位");
    }
    return QStringLiteral("未知故障");
}


} // namespace

QString MainWindow::runtimeDiagnosticsReportFilePath() const
{
    const QString exportTimestamp =
            QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    return QDir(uiEventLogDirPath()).filePath(
                QStringLiteral("runtime_diagnostics_report_%1.csv").arg(exportTimestamp));
}

void MainWindow::trimRuntimeDiagnosticsHistory(qint64 nowMs)
{
    const qint64 cutoffMs = nowMs - kRuntimeDiagnosticsWindowMs;
    while(runtimeDiagnosticsHistory.size() > 1 &&
          runtimeDiagnosticsHistory.at(1).capturedAtMs <= cutoffMs){
        runtimeDiagnosticsHistory.remove(0);
    }
}

void MainWindow::resetRuntimeDiagnosticsState(bool resetSources)
{
    if(!kEnableRuntimeDiagnosticsRecording){
        Q_UNUSED(resetSources);
        runtimeDiagnosticsHistory.clear();
        lastRuntimeDiagnosticsReportPath.clear();
        lastRuntimeDiagnosticsAutoWriteMs = 0;
        runtimeDiagnosticsReportWriting = false;
        refreshRuntimeDiagnosticsUi();
        return;
    }

    if(resetSources){
        hardwareInterface.resetDiagnostics();
        if(controlWorker){
            controlWorker->resetTimingDiagnostics();
        }
    }

    runtimeDiagnosticsHistory.clear();
    lastRuntimeDiagnosticsReportPath.clear();
    lastRuntimeDiagnosticsAutoWriteMs = 0;
    refreshRuntimeDiagnosticsUi();
}

MainWindow::RuntimeDiagnosticsSummary MainWindow::buildRuntimeDiagnosticsSummary() const
{
    RuntimeDiagnosticsSummary summary;
    if(runtimeDiagnosticsHistory.isEmpty()){
        return summary;
    }

    const RuntimeDiagnosticsSample& first = runtimeDiagnosticsHistory.first();
    const RuntimeDiagnosticsSample& last = runtimeDiagnosticsHistory.last();
    summary.windowDurationMs = std::max<qint64>(0, last.capturedAtMs - first.capturedAtMs);

    auto avgHzFromDiff = [](quint64 count, qint64 sumUs) -> double {
        if(count == 0 || sumUs <= 0){
            return 0.0;
        }
        return (static_cast<double>(count) * 1000000.0) / static_cast<double>(sumUs);
    };
    auto avgMsFromDiff = [](quint64 count, qint64 sumUs) -> double {
        if(count == 0){
            return 0.0;
        }
        return static_cast<double>(sumUs) / static_cast<double>(count) / 1000.0;
    };
    auto latestHzFromUs = [](qint64 dtUs) -> double {
        if(dtUs <= 0){
            return 0.0;
        }
        return 1000000.0 / static_cast<double>(dtUs);
    };
    auto latestMsFromUs = [](qint64 dtUs) -> double {
        if(dtUs <= 0){
            return 0.0;
        }
        return static_cast<double>(dtUs) / 1000.0;
    };

    const quint64 sensorIntervalCount =
            last.controlTiming.sensorFrameIntervalCount - first.controlTiming.sensorFrameIntervalCount;
    const qint64 sensorIntervalSumUs =
            last.controlTiming.sensorFrameIntervalSumUs - first.controlTiming.sensorFrameIntervalSumUs;
    summary.sensorIntervalCount = sensorIntervalCount;
    summary.sensorAverageHz = avgHzFromDiff(sensorIntervalCount, sensorIntervalSumUs);
    summary.sensorLatestHz = latestHzFromUs(last.controlTiming.latestSensorFrameIntervalUs);

    const quint64 controlIntervalCount =
            last.controlTiming.controlLoopIntervalCount - first.controlTiming.controlLoopIntervalCount;
    const qint64 controlIntervalSumUs =
            last.controlTiming.controlLoopIntervalSumUs - first.controlTiming.controlLoopIntervalSumUs;
    summary.controlLoopIntervalCount = controlIntervalCount;
    summary.controlLoopAveragePeriodMs = avgMsFromDiff(controlIntervalCount, controlIntervalSumUs);
    summary.controlLoopLatestPeriodMs = latestMsFromUs(last.controlTiming.latestControlLoopIntervalUs);

    const quint64 communicationIntervalCount =
            last.hardwareTiming.communicationIntervalCount - first.hardwareTiming.communicationIntervalCount;
    const qint64 communicationIntervalSumUs =
            last.hardwareTiming.communicationIntervalSumUs - first.hardwareTiming.communicationIntervalSumUs;
    summary.communicationIntervalCount = communicationIntervalCount;
    summary.communicationAverageHz = avgHzFromDiff(communicationIntervalCount, communicationIntervalSumUs);
    summary.communicationLatestHz = latestHzFromUs(last.hardwareTiming.latestCommunicationIntervalUs);

    const quint64 motorCommandIntervalCount =
            last.hardwareTiming.motorCommandIntervalCount - first.hardwareTiming.motorCommandIntervalCount;
    const qint64 motorCommandIntervalSumUs =
            last.hardwareTiming.motorCommandIntervalSumUs - first.hardwareTiming.motorCommandIntervalSumUs;
    summary.motorCommandIntervalCount = motorCommandIntervalCount;
    summary.motorCommandAveragePeriodMs = avgMsFromDiff(motorCommandIntervalCount, motorCommandIntervalSumUs);
    summary.motorCommandLatestPeriodMs = latestMsFromUs(last.hardwareTiming.latestMotorCommandIntervalUs);

    return summary;
}

void MainWindow::refreshRuntimeDiagnosticsUi()
{
    if(!ui){
        return;
    }

    auto setLabelTextIfChanged = [](QLabel* label, const QString& text){
        if(label && label->text() != text){
            label->setText(text);
        }
    };
    auto setButtonTextIfChanged = [](QPushButton* button, const QString& text){
        if(button && button->text() != text){
            button->setText(text);
        }
    };
    auto setWidgetEnabledIfChanged = [](QWidget* widget, bool enabled){
        if(widget && widget->isEnabled() != enabled){
            widget->setEnabled(enabled);
        }
    };
    auto ensureRuntimeDiagnosticsLabel =
            [this](const char* objectName, int row, int column, int columnSpan, bool wordWrap) -> QLabel* {
        QLabel* label = findOptionalUiObject<QLabel>(this, objectName);
        if(!label && ui && ui->runtimeDiagnosticsGroupBox && ui->gridLayoutRuntimeDiagnostics){
            label = new QLabel(ui->runtimeDiagnosticsGroupBox);
            label->setObjectName(QString::fromLatin1(objectName));
            label->setWordWrap(wordWrap);
            ui->gridLayoutRuntimeDiagnostics->addWidget(label, row, column, 1, columnSpan);
        }
        return label;
    };

    QLabel* udpStatsTitleLabel = ensureRuntimeDiagnosticsLabel(
                "runtimeDiagnosticsUdpStatsTitleLabel", 4, 0, 1, false);
    QLabel* udpStatsValueLabel = ensureRuntimeDiagnosticsLabel(
                "runtimeDiagnosticsUdpStatsValueLabel", 4, 1, 2, true);
    QLabel* faultRecordsTitleLabel = ensureRuntimeDiagnosticsLabel(
                "runtimeDiagnosticsFaultRecordsTitleLabel", 5, 0, 1, false);
    QLabel* faultRecordsValueLabel = ensureRuntimeDiagnosticsLabel(
                "runtimeDiagnosticsFaultRecordsValueLabel", 5, 1, 2, true);
    if(ui->gridLayoutRuntimeDiagnostics){
        if(ui->runtimeDiagnosticsExportButton){
            ui->gridLayoutRuntimeDiagnostics->addWidget(ui->runtimeDiagnosticsExportButton, 6, 1, 1, 1);
        }
        if(ui->runtimeDiagnosticsReportStatusLabel){
            ui->gridLayoutRuntimeDiagnostics->addWidget(ui->runtimeDiagnosticsReportStatusLabel, 6, 2, 1, 1);
        }
    }

    if(ui->runtimeDiagnosticsGroupBox){
        ui->runtimeDiagnosticsGroupBox->setTitle(QStringLiteral("运行诊断导出"));
    }
    setLabelTextIfChanged(ui->runtimeSensorFreqTitleLabel, QStringLiteral("参数快照"));
    setLabelTextIfChanged(ui->runtimeCommFreqTitleLabel, QStringLiteral("控制快照"));
    setLabelTextIfChanged(ui->runtimeMotorCycleTitleLabel, QStringLiteral("轨迹状态"));
    setLabelTextIfChanged(ui->runtimeDiagnosticsWindowTitleLabel, QStringLiteral("消息历史"));
    setLabelTextIfChanged(udpStatsTitleLabel, QStringLiteral("UDP 统计"));
    setLabelTextIfChanged(faultRecordsTitleLabel, QStringLiteral("故障记录"));

    setLabelTextIfChanged(
                ui->runtimeSensorFreqValueLabel,
                QStringLiteral("导出时写入当前 UI 参数快照；配置目录：%1")
                .arg(QDir::toNativeSeparators(parameterConfigStorageDirPath())));

    const bool hasControlSnapshot = controlWorker != nullptr;
    const ControlWorker::Snapshot controlSnapshot =
            hasControlSnapshot ? controlWorker->latestSnapshot() : ControlWorker::Snapshot{};
    const QString controlText = hasControlSnapshot ?
                QStringLiteral("ControlWorker seq=%1，力控线程=%2，电机位置%3项，张力%4项")
                .arg(static_cast<qulonglong>(controlSnapshot.sequence))
                .arg(controlSnapshot.forceThreadRunning ? QStringLiteral("运行") : QStringLiteral("停止"))
                .arg(static_cast<int>(controlSnapshot.motorRelRawPos.size()))
                .arg(static_cast<int>(controlSnapshot.forceSensorValue.size())) :
                QStringLiteral("控制线程尚未初始化，导出时会记录为 unavailable。");
    setLabelTextIfChanged(ui->runtimeCommFreqValueLabel, controlText);

    const RobotStateSnapshot robotState = currentRobotState(false);
    QString pvtStateText = QStringLiteral("空闲");
    if(robotState.pvtMotionPaused){
        pvtStateText = QStringLiteral("暂停");
    }
    else if(robotState.pvtMotionRunning){
        pvtStateText = QStringLiteral("运行中");
    }
    else if(robotState.pvtTrajectoryAvailable){
        pvtStateText = QStringLiteral("已加载");
    }
    const int plannedPointCount = static_cast<int>(plannedPoseTrajectoryRecordPose.size());
    const int activePointCount = activePoseTrajectoryDisplayPointCount();
    QString trajectoryText = QStringLiteral("运行模式=%1，PVT=%2，规划点=%3，显示点=%4")
            .arg(runModeDisplayName(robotState.runMode),
                 pvtStateText)
            .arg(plannedPointCount)
            .arg(activePointCount);
    if(trajFileLoadInfo.attempted){
        trajectoryText.append(QStringLiteral("，外部轨迹=%1(%2点)")
                              .arg(trajFileLoadInfo.success ? QStringLiteral("已加载") : QStringLiteral("失败"))
                              .arg(trajFileLoadInfo.pointNum));
    }
    setLabelTextIfChanged(ui->runtimeMotorCycleValueLabel, trajectoryText);

    setLabelTextIfChanged(
                ui->runtimeDiagnosticsWindowValueLabel,
                QStringLiteral("内存消息 %1 条，待写入日志 %2 条；导出时保留最近 100 条。")
                .arg(messageHistoryEntries.size())
                .arg(pendingUiEventLogLines.size()));
    setLabelTextIfChanged(udpStatsValueLabel, buildUdpStatusSummary());

    const QString faultText = runtimeState.safetyFaultLatched ?
                QStringLiteral("已锁存：%1；日志：%2")
                .arg(runtimeState.safetyFaultSummary.isEmpty() ?
                         QStringLiteral("未提供摘要") :
                         runtimeState.safetyFaultSummary,
                     QDir::toNativeSeparators(structuredFaultLogFilePath())) :
                QStringLiteral("当前无锁存故障；结构化故障日志：%1")
                .arg(QDir::toNativeSeparators(structuredFaultLogFilePath()));
    setLabelTextIfChanged(faultRecordsValueLabel, faultText);

    QString reportStatus = !RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled ?
                QStringLiteral("已由性能开关停用") :
                runtimeDiagnosticsReportWriting ?
                    QStringLiteral("写入中...") :
                    QStringLiteral("尚未导出");
    if(RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled &&
            !lastRuntimeDiagnosticsReportPath.isEmpty()){
        reportStatus = QFileInfo(lastRuntimeDiagnosticsReportPath).fileName();
    }
    setLabelTextIfChanged(ui->runtimeDiagnosticsReportStatusLabel, reportStatus);
    setButtonTextIfChanged(ui->runtimeDiagnosticsExportButton, QStringLiteral("导出运行诊断"));
    if(ui->runtimeDiagnosticsExportButton){
        const QString tooltip = QStringLiteral(
                    "导出当前运行诊断快照到 data/outputmsg/runtime_diagnostics_report_时间戳.csv，内容包含参数快照、控制快照、诊断条目汇总、轨迹状态、消息历史、UDP统计和故障记录。");
        if(ui->runtimeDiagnosticsExportButton->toolTip() != tooltip){
            ui->runtimeDiagnosticsExportButton->setToolTip(tooltip);
        }
    }
    setWidgetEnabledIfChanged(ui->runtimeDiagnosticsExportButton,
                              RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled &&
                              !runtimeDiagnosticsReportWriting);
}

QJsonObject MainWindow::buildRuntimeDiagnosticsTrajectoryStatusJson() const
{
    const RobotStateSnapshot robotState = currentRobotState(false);
    QJsonObject object;
    object.insert(QStringLiteral("robot_state"), buildRobotStateSnapshotJson(robotState));
    object.insert(QStringLiteral("runtime_state"), buildRuntimeStateSnapshotJson());

    QJsonObject planned;
    const int plannedPointCount = static_cast<int>(plannedPoseTrajectoryRecordPose.size());
    const bool plannedAvailable =
            plannedPointCount > 0 &&
            static_cast<int>(plannedPoseTrajectoryRecordTimeStamp.size()) == plannedPointCount;
    planned.insert(QStringLiteral("available"), plannedAvailable);
    planned.insert(QStringLiteral("point_count"), plannedPointCount);
    planned.insert(QStringLiteral("motor_axis_count"),
                   static_cast<int>(plannedPoseTrajectoryRecordMotorIndex.size()));
    planned.insert(QStringLiteral("motor_index"),
                   toJsonArray(plannedPoseTrajectoryRecordMotorIndex));
    if(plannedPoseTrajectoryRecordTimestampMs >= 0){
        planned.insert(QStringLiteral("updated_at_ms"), plannedPoseTrajectoryRecordTimestampMs);
        planned.insert(QStringLiteral("updated_at"),
                       QDateTime::fromMSecsSinceEpoch(plannedPoseTrajectoryRecordTimestampMs)
                       .toString(Qt::ISODateWithMs));
    }
    if(plannedAvailable && !plannedPoseTrajectoryRecordTimeStamp.empty()){
        planned.insert(QStringLiteral("start_time_sec"), plannedPoseTrajectoryRecordTimeStamp.front());
        planned.insert(QStringLiteral("end_time_sec"), plannedPoseTrajectoryRecordTimeStamp.back());
        planned.insert(QStringLiteral("duration_sec"),
                       std::max(0.0,
                                plannedPoseTrajectoryRecordTimeStamp.back() -
                                plannedPoseTrajectoryRecordTimeStamp.front()));
    }
    if(plannedPointCount > 0 && hasFiniteValues(plannedPoseTrajectoryRecordPose.front(), 6)){
        planned.insert(QStringLiteral("start_pose"), toJsonArray(plannedPoseTrajectoryRecordPose.front()));
    }
    if(plannedPointCount > 0 && hasFiniteValues(plannedPoseTrajectoryRecordPose.back(), 6)){
        planned.insert(QStringLiteral("end_pose"), toJsonArray(plannedPoseTrajectoryRecordPose.back()));
    }
    if(hasFiniteValues(lastPlannedPoseTrajectoryEndPose, 6)){
        planned.insert(QStringLiteral("last_planned_end_pose"), toJsonArray(lastPlannedPoseTrajectoryEndPose));
    }
    object.insert(QStringLiteral("planned_pose_trajectory"), planned);

    QJsonObject display;
    display.insert(QStringLiteral("running"), activePoseTrajectoryDisplayRunning);
    display.insert(QStringLiteral("point_index"), activePoseTrajectoryDisplayPointIndex);
    display.insert(QStringLiteral("point_count"), activePoseTrajectoryDisplayPointCount());
    display.insert(QStringLiteral("time_stamp_count"),
                   static_cast<int>(activePoseTrajectoryDisplayTimeStamp.size()));
    display.insert(QStringLiteral("lambda_count"),
                   static_cast<int>(activePoseTrajectoryDisplayLambda.size()));
    object.insert(QStringLiteral("active_trajectory_display"), display);

    QJsonObject trajectoryFile;
    trajectoryFile.insert(QStringLiteral("use_trajectory_file"), useTrajFile);
    trajectoryFile.insert(QStringLiteral("attempted"), trajFileLoadInfo.attempted);
    trajectoryFile.insert(QStringLiteral("success"), trajFileLoadInfo.success);
    trajectoryFile.insert(QStringLiteral("source"), trajFileLoadInfo.source);
    trajectoryFile.insert(QStringLiteral("path"), QDir::toNativeSeparators(trajFileLoadInfo.path));
    trajectoryFile.insert(QStringLiteral("message"), trajFileLoadInfo.message);
    trajectoryFile.insert(QStringLiteral("end_num"), trajFileLoadInfo.endNum);
    trajectoryFile.insert(QStringLiteral("point_num"), trajFileLoadInfo.pointNum);
    trajectoryFile.insert(QStringLiteral("segment_point_count"), trajFileLoadInfo.segmentPointCount);
    trajectoryFile.insert(QStringLiteral("segment_count"), trajFileLoadInfo.segmentCount);
    trajectoryFile.insert(QStringLiteral("duration_sec"), trajFileLoadInfo.duration);
    trajectoryFile.insert(QStringLiteral("has_time_step"), trajFileLoadInfo.hasTimeStep);
    trajectoryFile.insert(QStringLiteral("uniform_time_step"), trajFileLoadInfo.uniformTimeStep);
    trajectoryFile.insert(QStringLiteral("min_time_step_sec"), trajFileLoadInfo.minTimeStep);
    trajectoryFile.insert(QStringLiteral("avg_time_step_sec"), trajFileLoadInfo.avgTimeStep);
    trajectoryFile.insert(QStringLiteral("max_time_step_sec"), trajFileLoadInfo.maxTimeStep);
    trajectoryFile.insert(QStringLiteral("program_batch_active"), trajectoryFileProgramBatchActive);
    trajectoryFile.insert(QStringLiteral("program_segment_index"), trajectoryFileProgramSegmentIndex);
    trajectoryFile.insert(QStringLiteral("program_segment_count"),
                          static_cast<int>(trajFileSegmentRanges.size()));
    object.insert(QStringLiteral("trajectory_file"), trajectoryFile);

    QJsonObject udpTrajectory;
    udpTrajectory.insert(QStringLiteral("trajectory_point_buffer_count"),
                         udpTrajectoryPointBuffer.size());
    udpTrajectory.insert(QStringLiteral("platform_trajectory_point_buffer_count"),
                         udpPlatformTrajectoryPointBuffer.size());
    udpTrajectory.insert(QStringLiteral("platform_capture_armed"),
                         udpPlatformTrajectoryCaptureArmed);
    udpTrajectory.insert(QStringLiteral("program_control_external_trajectory_active"),
                         udpProgramControlExternalTrajectoryActive);
    udpTrajectory.insert(QStringLiteral("program_control_auto_execute_after_simulation"),
                         udpProgramControlExternalTrajectoryActive &&
                         runtimeState.autoExecutePoseAfterSimulation);
    udpTrajectory.insert(QStringLiteral("return_home_pending"), udpReturnHomePending);
    udpTrajectory.insert(QStringLiteral("return_home_in_progress"), udpReturnHomeInProgress);
    udpTrajectory.insert(QStringLiteral("latest_pose_seq"),
                         static_cast<qint64>(std::min<quint64>(latestUdpPoseCommand.seq,
                                                               static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    udpTrajectory.insert(QStringLiteral("latest_pose_timestamp_ms"), latestUdpPoseCommand.timestampMs);
    udpTrajectory.insert(QStringLiteral("latest_pose_duration_sec"), latestUdpPoseCommand.duration);
    udpTrajectory.insert(QStringLiteral("latest_pose_step_ms"), latestUdpPoseCommand.stepMs);
    udpTrajectory.insert(QStringLiteral("latest_trajectory_seq"),
                         static_cast<qint64>(std::min<quint64>(latestUdpTrajectoryChunk.seq,
                                                               static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    udpTrajectory.insert(QStringLiteral("latest_trajectory_timestamp_ms"),
                         latestUdpTrajectoryChunk.timestampMs);
    udpTrajectory.insert(QStringLiteral("latest_platform_command_ms"),
                         lastUdpPlatformCommandTimeMs);
    object.insert(QStringLiteral("udp_trajectory_state"), udpTrajectory);

    return object;
}

QJsonObject MainWindow::buildRuntimeDiagnosticsMessageHistoryJson(int maxEntries) const
{
    const int boundedMaxEntries = std::max(0, maxEntries);
    const int totalCount = messageHistoryEntries.size();
    const int startIndex = std::max(0, totalCount - boundedMaxEntries);

    QJsonArray recentEntries;
    for(int index = startIndex; index < totalCount; ++index){
        recentEntries.append(messageHistoryEntries.at(index));
    }

    QJsonObject object;
    object.insert(QStringLiteral("total_count"), totalCount);
    object.insert(QStringLiteral("exported_recent_count"), recentEntries.size());
    object.insert(QStringLiteral("pending_log_line_count"), pendingUiEventLogLines.size());
    object.insert(QStringLiteral("memory_history_limit"), 500);
    object.insert(QStringLiteral("text_log_path"), QDir::toNativeSeparators(uiEventLogFilePath()));
    object.insert(QStringLiteral("recent_entries"), recentEntries);
    return object;
}

QJsonObject MainWindow::buildRuntimeDiagnosticsUdpStatsJson() const
{
    auto timestampObject = [](qint64 timestampMs) -> QJsonObject {
        QJsonObject object;
        object.insert(QStringLiteral("timestamp_ms"), timestampMs);
        object.insert(QStringLiteral("timestamp"),
                      timestampMs > 0 ?
                          QDateTime::fromMSecsSinceEpoch(timestampMs).toString(Qt::ISODateWithMs) :
                          QString());
        object.insert(QStringLiteral("age_ms"),
                      timestampMs > 0 ?
                          std::max<qint64>(0, QDateTime::currentMSecsSinceEpoch() - timestampMs) :
                          -1);
        return object;
    };

    QJsonObject stats;
    stats.insert(QStringLiteral("bridge_initialized"), udpCommWorker != nullptr);
    stats.insert(QStringLiteral("bridge_active"), udpRealtimeBridgeActive);
    stats.insert(QStringLiteral("listen_port"), udpRealtimeListenPort());
    stats.insert(QStringLiteral("target_ip"), udpRealtimeTargetIp());
    stats.insert(QStringLiteral("target_port"), udpRealtimeTargetPort());
    stats.insert(QStringLiteral("v9_platform_feedback_enabled"), udpV9PlatformFeedbackEnabled());
    stats.insert(QStringLiteral("send_interval_ms"),
                 udpRealtimeSendIntervalMsForMode(udpV9PlatformFeedbackEnabled()));
    stats.insert(QStringLiteral("summary"), buildUdpStatusSummary());
    stats.insert(QStringLiteral("rx_count"),
                 static_cast<qint64>(std::min<quint64>(udpCommStats.rxCount,
                                                       static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    stats.insert(QStringLiteral("tx_count"),
                 static_cast<qint64>(std::min<quint64>(udpCommStats.txCount,
                                                       static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    stats.insert(QStringLiteral("parse_error_count"),
                 static_cast<qint64>(std::min<quint64>(udpCommStats.parseErrorCount,
                                                       static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    stats.insert(QStringLiteral("last_rx"), timestampObject(udpCommStats.lastRxTimeMs));
    stats.insert(QStringLiteral("last_tx"), timestampObject(udpCommStats.lastTxTimeMs));
    stats.insert(QStringLiteral("receive_status"), udpCommStats.receiveStatus);
    stats.insert(QStringLiteral("parse_status"), udpCommStats.parseStatus);
    stats.insert(QStringLiteral("send_status"), udpCommStats.sendStatus);
    stats.insert(QStringLiteral("last_error"), udpCommStats.lastError);
    stats.insert(QStringLiteral("last_received_packet_preview"), udpCommStats.lastReceivedPacket);
    stats.insert(QStringLiteral("last_packet_summary"), udpLastPacketSummaryText);
    stats.insert(QStringLiteral("last_packet_action"), udpLastPacketActionText);

    QJsonObject payload;
    payload.insert(QStringLiteral("seq"),
                   static_cast<qint64>(std::min<quint64>(udpStatusPayload.seq,
                                                         static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    payload.insert(QStringLiteral("system_running"), udpStatusPayload.systemRunning);
    payload.insert(QStringLiteral("pvt_running"), udpStatusPayload.pvtRunning);
    payload.insert(QStringLiteral("pvt_paused"), udpStatusPayload.pvtPaused);
    payload.insert(QStringLiteral("platform_state"),
                   static_cast<int>(udpStatusPayload.platformFeedback.state));
    payload.insert(QStringLiteral("platform_state_text"),
                   udpPlatformStateDisplayText(udpStatusPayload.platformFeedback.state));
    payload.insert(QStringLiteral("platform_feedback_timestamp_ms"),
                   udpStatusPayload.platformFeedback.timestampMs);
    payload.insert(QStringLiteral("forward_kinematics_end_pose_valid"),
                   udpStatusPayload.forwardKinematicsEndPoseValid);
    payload.insert(QStringLiteral("forward_kinematics_end_pose"),
                   UdpPacketTypes::vectorToJsonArray(udpStatusPayload.forwardKinematicsEndPose));
    payload.insert(QStringLiteral("forward_kinematics_end_pose_timestamp_ms"),
                   udpStatusPayload.forwardKinematicsEndPoseTimestampMs);
    payload.insert(QStringLiteral("forward_kinematics_end_pose_equation_count"),
                   udpStatusPayload.forwardKinematicsEndPoseEquationCount);
    stats.insert(QStringLiteral("latest_status_payload"), payload);
    return stats;
}

QJsonObject MainWindow::buildRuntimeDiagnosticsFaultRecordsJson(int maxRecords) const
{
    const QString filePath = structuredFaultLogFilePath();
    QFileInfo fileInfo(filePath);
    QJsonObject object;
    object.insert(QStringLiteral("structured_log_path"), QDir::toNativeSeparators(filePath));
    object.insert(QStringLiteral("text_log_path"), QDir::toNativeSeparators(uiEventLogFilePath()));
    object.insert(QStringLiteral("software_fault_guard_log_path"),
                  QDir::toNativeSeparators(softwareFaultGuardLogFilePath()));
    object.insert(QStringLiteral("log_exists"), fileInfo.exists());
    object.insert(QStringLiteral("latched_fault"), runtimeState.safetyFaultLatched);
    object.insert(QStringLiteral("current_fault_code"), runtimeState.safetyFaultCode);
    object.insert(QStringLiteral("current_fault_code_name"),
                  safetyFaultCodeName(runtimeState.safetyFaultCode));
    object.insert(QStringLiteral("current_fault_type"),
                  safetyFaultDisplayName(runtimeState.safetyFaultCode));
    object.insert(QStringLiteral("current_fault_summary"), runtimeState.safetyFaultSummary);
    object.insert(QStringLiteral("current_fault_detail"), runtimeState.safetyFaultDetail);
    object.insert(QStringLiteral("current_fault_occurred_at_ms"),
                  runtimeState.safetyFaultOccurredMs);
    object.insert(QStringLiteral("current_fault_occurred_at"),
                  runtimeState.safetyFaultOccurredMs > 0 ?
                      QDateTime::fromMSecsSinceEpoch(runtimeState.safetyFaultOccurredMs)
                      .toString(Qt::ISODateWithMs) :
                      QString());

    QJsonArray records;
    QFile file(filePath);
    if(file.exists() && file.open(QIODevice::ReadOnly | QIODevice::Text)){
        const QByteArray bytes = file.readAll();
        QJsonParseError parseError;
        const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
        const bool legacyContainer =
                parseError.error == QJsonParseError::NoError &&
                document.isObject() &&
                document.object().value(QStringLiteral("records")).isArray();
        if(legacyContainer){
            const QJsonObject rootObject = document.object();
            records = rootObject.value(QStringLiteral("records")).toArray();
            object.insert(QStringLiteral("log_updated_at"),
                          rootObject.value(QStringLiteral("updated_at")).toString());
            object.insert(QStringLiteral("latest_event_type"),
                          rootObject.value(QStringLiteral("latest_event_type")).toString());
            object.insert(QStringLiteral("latest_event_type_text"),
                          rootObject.value(QStringLiteral("latest_event_type_text")).toString());
            object.insert(QStringLiteral("latest_fault_type"),
                          rootObject.value(QStringLiteral("latest_fault_type")).toString());
            object.insert(QStringLiteral("latest_fault_type_text"),
                          rootObject.value(QStringLiteral("latest_fault_type_text")).toString());
            object.insert(QStringLiteral("latest_fault_occurred_at"),
                          rootObject.value(QStringLiteral("latest_fault_occurred_at")).toString());
        }
        else{
            const QList<QByteArray> lines = bytes.split('\n');
            for(const QByteArray& line : lines){
                const QByteArray trimmed = line.trimmed();
                if(trimmed.isEmpty()){
                    continue;
                }
                QJsonParseError lineParseError;
                const QJsonDocument lineDocument = QJsonDocument::fromJson(trimmed, &lineParseError);
                if(lineParseError.error == QJsonParseError::NoError && lineDocument.isObject()){
                    records.append(lineDocument.object());
                }
            }
        }
    }

    const int totalCount = records.size();
    const int boundedMaxRecords = std::max(0, maxRecords);
    const int startIndex = std::max(0, totalCount - boundedMaxRecords);
    QJsonArray recentRecords;
    for(int index = startIndex; index < totalCount; ++index){
        recentRecords.append(records.at(index));
    }
    object.insert(QStringLiteral("record_count"), totalCount);
    object.insert(QStringLiteral("exported_recent_count"), recentRecords.size());
    object.insert(QStringLiteral("recent_records"), recentRecords);
    return object;
}

QJsonObject MainWindow::buildRuntimeDiagnosticsItemSummariesJson() const
{
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    qint64 windowStartMs = nowMs - kRuntimeDiagnosticsWindowMs;
    qint64 windowEndMs = nowMs;
    QString windowSource = QStringLiteral("recent_runtime_window");
    if(sessionRecordingState.active && sessionRecordingState.startedAtMs > 0){
        windowStartMs = sessionRecordingState.startedAtMs;
        windowEndMs = nowMs;
        windowSource = QStringLiteral("active_session_record_window");
    }
    else if(sessionRecordingState.startedAtMs > 0 && sessionRecordingState.endedAtMs > 0){
        windowStartMs = sessionRecordingState.startedAtMs;
        windowEndMs = sessionRecordingState.endedAtMs;
        windowSource = QStringLiteral("last_session_record_window");
    }
    else if(!runtimeDiagnosticsHistory.isEmpty()){
        windowStartMs = runtimeDiagnosticsHistory.first().capturedAtMs;
        windowEndMs = std::max(runtimeDiagnosticsHistory.last().capturedAtMs, windowStartMs);
        windowSource = QStringLiteral("runtime_diagnostics_history_window");
    }
    if(windowEndMs < windowStartMs){
        windowEndMs = windowStartMs;
    }

    QJsonObject root;
    root.insert(QStringLiteral("说明"),
                QStringLiteral("本节逐条覆盖运行诊断导出要求；原始明细继续由 session record 导出，本报告只保留当前窗口汇总。"));
    root.insert(QStringLiteral("window_source"), windowSource);
    root.insert(QStringLiteral("window_start_ms"), windowStartMs);
    root.insert(QStringLiteral("window_end_ms"), windowEndMs);
    root.insert(QStringLiteral("window_duration_ms"),
                std::max<qint64>(0, windowEndMs - windowStartMs));
    root.insert(QStringLiteral("window_start"),
                QDateTime::fromMSecsSinceEpoch(windowStartMs).toString(Qt::ISODateWithMs));
    root.insert(QStringLiteral("window_end"),
                QDateTime::fromMSecsSinceEpoch(windowEndMs).toString(Qt::ISODateWithMs));
    QJsonArray requiredItems;
    requiredItems.append(QStringLiteral("软件采集张力传感器数据"));
    requiredItems.append(QStringLiteral("硬件通信间隔"));
    requiredItems.append(QStringLiteral("电机指令"));
    requiredItems.append(QStringLiteral("控制循环周期"));
    requiredItems.append(QStringLiteral("UDP 统计"));
    requiredItems.append(QStringLiteral("运行状态"));
    root.insert(QStringLiteral("required_items"), requiredItems);
    root.insert(QStringLiteral("all_required_sections_present"), true);

    ControlWorker::Snapshot controlSnapshot;
    const bool controlAvailable = controlWorker != nullptr;
    if(controlAvailable){
        controlSnapshot = controlWorker->latestSnapshot();
    }
    const HardwareInterface::DiagnosticsSnapshot hardwareDiagnostics =
            hardwareInterface.diagnosticsSnapshot();

    QJsonObject tensionItem;
    tensionItem.insert(QStringLiteral("section_present"), true);
    tensionItem.insert(QStringLiteral("item_name"), QStringLiteral("软件采集张力传感器数据"));
    tensionItem.insert(QStringLiteral("raw_data_path"),
                       QStringLiteral("session record: [张力传感器数据]"));
    if(controlAvailable){
        const QVector<ControlWorker::SensorValueSample> sensorValueHistory =
                controlWorker->sensorValueHistory(windowStartMs, windowEndMs);
        const QVector<ControlWorker::SensorValueSample> sensorTraceValueHistory =
                controlWorker->sensorTraceValueHistory(windowStartMs, windowEndMs);
        const QVector<ControlWorker::DiagnosticRawSample> sensorFrameHistory =
                controlWorker->sensorTimingHistory(windowStartMs, windowEndMs);
        const QVector<ControlWorker::DiagnosticRawSample> sensorTraceReadHistory =
                controlWorker->sensorTraceReadTimingHistory(windowStartMs, windowEndMs);
        QJsonObject sensorFrameSummary =
                buildIntervalSampleSummaryJson(sensorFrameHistory,
                                               windowStartMs,
                                               windowEndMs,
                                               QStringLiteral("ControlWorker::sensorTimingHistory"));
        if(!sensorFrameSummary.value(QStringLiteral("has_interval_data")).toBool()){
            sensorFrameSummary.insert(
                        QStringLiteral("counter_fallback"),
                        buildIntervalCounterSummaryJson(
                            controlSnapshot.timingDiagnostics.sensorFrameIntervalCount,
                            controlSnapshot.timingDiagnostics.sensorFrameIntervalSumUs,
                            controlSnapshot.timingDiagnostics.latestSensorFrameIntervalUs,
                            QStringLiteral("ControlWorker::TimingDiagnostics.sensorFrame")));
        }
        QJsonObject traceReadSummary =
                buildIntervalSampleSummaryJson(sensorTraceReadHistory,
                                               windowStartMs,
                                               windowEndMs,
                                               QStringLiteral("ControlWorker::sensorTraceReadTimingHistory"));
        const bool hasTensionData =
                !controlSnapshot.forceSensorValue.empty() ||
                !sensorValueHistory.isEmpty() ||
                controlSnapshot.timingDiagnostics.sensorFrameCount > 0;
        tensionItem.insert(QStringLiteral("available"), hasTensionData);
        tensionItem.insert(QStringLiteral("latest_tension_summary"),
                           buildVectorValueSummaryJson(controlSnapshot.forceSensorValue,
                                                       QStringLiteral("latest_values")));
        tensionItem.insert(QStringLiteral("value_history_summary"),
                           buildSensorValueHistorySummaryJson(sensorValueHistory,
                                                              windowStartMs,
                                                              windowEndMs));
        tensionItem.insert(QStringLiteral("trace_expanded_value_sample_count"),
                           sensorTraceValueHistory.size());
        tensionItem.insert(QStringLiteral("sensor_frame_interval_summary"), sensorFrameSummary);
        tensionItem.insert(QStringLiteral("trace_read_interval_summary"), traceReadSummary);
    }
    else{
        tensionItem.insert(QStringLiteral("available"), false);
        tensionItem.insert(QStringLiteral("note"),
                           QStringLiteral("控制线程未初始化，无法读取软件采集张力缓存。"));
    }
    root.insert(QStringLiteral("软件采集张力传感器数据"), tensionItem);

    const QVector<HardwareInterface::DiagnosticRawSample> communicationHistory =
            hardwareInterface.communicationTimingHistory(windowStartMs, windowEndMs);
    QJsonObject communicationItem =
            buildIntervalSampleSummaryJson(communicationHistory,
                                           windowStartMs,
                                           windowEndMs,
                                           QStringLiteral("HardwareInterface::communicationTimingHistory"));
    communicationItem.insert(QStringLiteral("section_present"), true);
    communicationItem.insert(QStringLiteral("item_name"), QStringLiteral("硬件通信间隔"));
    communicationItem.insert(QStringLiteral("raw_data_path"),
                             QStringLiteral("session record: [实时通信频率原始数据]/[通信频率原始数据]"));
    communicationItem.insert(QStringLiteral("available"),
                             !communicationHistory.isEmpty() ||
                             hardwareDiagnostics.communicationIntervalCount > 0);
    communicationItem.insert(QStringLiteral("event_count_total"),
                             jsonIntFromQuint64(hardwareDiagnostics.communicationEventCount));
    if(!communicationItem.value(QStringLiteral("has_interval_data")).toBool()){
        communicationItem.insert(
                    QStringLiteral("counter_fallback"),
                    buildIntervalCounterSummaryJson(
                        hardwareDiagnostics.communicationIntervalCount,
                        hardwareDiagnostics.communicationIntervalSumUs,
                        hardwareDiagnostics.latestCommunicationIntervalUs,
                        QStringLiteral("HardwareInterface::DiagnosticsSnapshot.communication")));
    }
    root.insert(QStringLiteral("硬件通信间隔"), communicationItem);

    const QVector<HardwareInterface::DiagnosticRawSample> motorCommandHistory =
            hardwareInterface.motorCommandTimingHistory(windowStartMs, windowEndMs);
    QJsonObject motorCommandItem =
            buildIntervalSampleSummaryJson(motorCommandHistory,
                                           windowStartMs,
                                           windowEndMs,
                                           QStringLiteral("HardwareInterface::motorCommandTimingHistory"));
    motorCommandItem.insert(QStringLiteral("section_present"), true);
    motorCommandItem.insert(QStringLiteral("item_name"), QStringLiteral("电机指令"));
    motorCommandItem.insert(QStringLiteral("raw_data_path"),
                            QStringLiteral("session record: 电机指令间隔原始导出已停用"));
    motorCommandItem.insert(QStringLiteral("available"),
                            !motorCommandHistory.isEmpty() ||
                            hardwareDiagnostics.motorCommandIntervalCount > 0 ||
                            (controlAvailable && !controlSnapshot.motorCommand.empty()));
    motorCommandItem.insert(QStringLiteral("event_count_total"),
                            jsonIntFromQuint64(hardwareDiagnostics.motorCommandEventCount));
    if(controlAvailable){
        motorCommandItem.insert(QStringLiteral("latest_motor_command_summary"),
                                buildVectorValueSummaryJson(controlSnapshot.motorCommand,
                                                            QStringLiteral("latest_values")));
    }
    if(!motorCommandItem.value(QStringLiteral("has_interval_data")).toBool()){
        motorCommandItem.insert(
                    QStringLiteral("counter_fallback"),
                    buildIntervalCounterSummaryJson(
                        hardwareDiagnostics.motorCommandIntervalCount,
                        hardwareDiagnostics.motorCommandIntervalSumUs,
                        hardwareDiagnostics.latestMotorCommandIntervalUs,
                        QStringLiteral("HardwareInterface::DiagnosticsSnapshot.motorCommand")));
    }
    root.insert(QStringLiteral("电机指令"), motorCommandItem);

    QJsonObject controlLoopItem;
    controlLoopItem.insert(QStringLiteral("section_present"), true);
    controlLoopItem.insert(QStringLiteral("item_name"), QStringLiteral("控制循环周期"));
    controlLoopItem.insert(QStringLiteral("raw_data_path"),
                           QStringLiteral("session record: 控制循环周期原始导出已停用"));
    if(controlAvailable){
        const QVector<ControlWorker::DiagnosticRawSample> controlLoopHistory =
                controlWorker->controlLoopTimingHistory(windowStartMs, windowEndMs);
        controlLoopItem =
                buildIntervalSampleSummaryJson(controlLoopHistory,
                                               windowStartMs,
                                               windowEndMs,
                                               QStringLiteral("ControlWorker::controlLoopTimingHistory"));
        controlLoopItem.insert(QStringLiteral("section_present"), true);
        controlLoopItem.insert(QStringLiteral("item_name"), QStringLiteral("控制循环周期"));
        controlLoopItem.insert(QStringLiteral("raw_data_path"),
                               QStringLiteral("session record: 控制循环周期原始导出已停用"));
        controlLoopItem.insert(QStringLiteral("available"),
                               !controlLoopHistory.isEmpty() ||
                               controlSnapshot.timingDiagnostics.controlLoopIntervalCount > 0);
        controlLoopItem.insert(QStringLiteral("tick_count_total"),
                               jsonIntFromQuint64(controlSnapshot.timingDiagnostics.controlLoopTickCount));
        if(!controlLoopItem.value(QStringLiteral("has_interval_data")).toBool()){
            controlLoopItem.insert(
                        QStringLiteral("counter_fallback"),
                        buildIntervalCounterSummaryJson(
                            controlSnapshot.timingDiagnostics.controlLoopIntervalCount,
                            controlSnapshot.timingDiagnostics.controlLoopIntervalSumUs,
                            controlSnapshot.timingDiagnostics.latestControlLoopIntervalUs,
                            QStringLiteral("ControlWorker::TimingDiagnostics.controlLoop")));
        }
    }
    else{
        controlLoopItem.insert(QStringLiteral("available"), false);
        controlLoopItem.insert(QStringLiteral("note"),
                               QStringLiteral("控制线程未初始化，无法读取控制循环周期。"));
    }
    root.insert(QStringLiteral("控制循环周期"), controlLoopItem);

    QJsonObject udpItem = buildRuntimeDiagnosticsUdpStatsJson();
    udpItem.insert(QStringLiteral("section_present"), true);
    udpItem.insert(QStringLiteral("item_name"), QStringLiteral("UDP 统计"));
    udpItem.insert(QStringLiteral("available"),
                   udpCommWorker != nullptr ||
                   udpRealtimeBridgeActive ||
                   udpCommStats.rxCount > 0 ||
                   udpCommStats.txCount > 0 ||
                   udpCommStats.parseErrorCount > 0 ||
                   !udpCommStats.lastError.isEmpty());
    udpItem.insert(QStringLiteral("summary_info"),
                   QStringLiteral("包含桥接状态、端口、收发计数、解析错误计数、最近收发时间和最新平台状态载荷。"));
    root.insert(QStringLiteral("UDP 统计"), udpItem);

    const RobotStateSnapshot robotState = currentRobotState(false);
    QJsonObject runtimeItem;
    runtimeItem.insert(QStringLiteral("section_present"), true);
    runtimeItem.insert(QStringLiteral("item_name"), QStringLiteral("运行状态"));
    runtimeItem.insert(QStringLiteral("available"), true);
    runtimeItem.insert(QStringLiteral("summary"),
                       QStringLiteral("run_mode=%1, system_running=%2, any_motion_running=%3, safety_fault_latched=%4")
                       .arg(runModeDisplayName(robotState.runMode),
                            robotState.systemRunning ? QStringLiteral("true") : QStringLiteral("false"),
                            robotState.anyMotionRunning ? QStringLiteral("true") : QStringLiteral("false"),
                            runtimeState.safetyFaultLatched ? QStringLiteral("true") : QStringLiteral("false")));
    runtimeItem.insert(QStringLiteral("robot_state"), buildRobotStateSnapshotJson(robotState));
    runtimeItem.insert(QStringLiteral("runtime_state"), buildRuntimeStateSnapshotJson());
    root.insert(QStringLiteral("运行状态"), runtimeItem);

    return root;
}

QString MainWindow::runtimeDiagnosticsExportNoDataReason() const
{
    const QJsonObject parameterWidgets =
            buildParameterConfigSnapshot().value(QStringLiteral("widgets")).toObject();
    if(!parameterWidgets.isEmpty()){
        return QString();
    }
    if(controlWorker){
        return QString();
    }
    if(!plannedPoseTrajectoryRecordPose.empty() ||
            activePoseTrajectoryDisplayPointCount() > 0 ||
            trajFileLoadInfo.attempted){
        return QString();
    }
    if(!messageHistoryEntries.isEmpty() || !pendingUiEventLogLines.isEmpty()){
        return QString();
    }
    if(udpRealtimeBridgeActive ||
            udpCommStats.rxCount > 0 ||
            udpCommStats.txCount > 0 ||
            udpCommStats.parseErrorCount > 0 ||
            !udpCommStats.lastError.isEmpty() ||
            !udpLastPacketSummaryText.isEmpty() ||
            !udpLastPacketActionText.isEmpty()){
        return QString();
    }
    if(runtimeState.safetyFaultLatched ||
            QFileInfo(structuredFaultLogFilePath()).exists() ||
            QFileInfo(uiEventLogFilePath()).exists()){
        return QString();
    }

    return QStringLiteral("当前没有可导出的运行诊断数据：参数快照为空，控制线程未初始化，轨迹状态、消息历史、UDP统计和故障记录均为空。");
}

QJsonObject MainWindow::buildRuntimeDiagnosticsReportJson() const
{
    {
        QJsonObject root;
        root.insert(QStringLiteral("报告名称"), QStringLiteral("运行诊断快照报告"));
        root.insert(QStringLiteral("报告版本"), 2);
        root.insert(QStringLiteral("schema_version"), 2);
        root.insert(QStringLiteral("生成时间"),
                    QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
        root.insert(QStringLiteral("software_name"),
                    QCoreApplication::applicationName().isEmpty() ?
                        QStringLiteral("G302") :
                        QCoreApplication::applicationName());
        root.insert(QStringLiteral("build_date"), QString::fromLatin1(__DATE__ " " __TIME__));
        root.insert(QStringLiteral("qt_version"), QStringLiteral(QT_VERSION_STR));
        root.insert(QStringLiteral("source_dir"), QStringLiteral(G302_SOURCE_DIR));
        root.insert(QStringLiteral("installation_data_root"), installationDataRootDirPath());
        root.insert(QStringLiteral("calibration_dir"), calibrationStorageDirPath());
        root.insert(QStringLiteral("parameter_config_dir"), parameterConfigStorageDirPath());
        root.insert(QStringLiteral("log_dir"), uiEventLogDirPath());
        root.insert(QStringLiteral("是否有可导出数据"), true);
        root.insert(QStringLiteral("导出说明"),
                    QStringLiteral("运行诊断导出当前运行现场快照；频率统计和实时值原始数据由会话记录导出承担，避免重复导出。"));

        QJsonArray exportedSections;
        exportedSections.append(QStringLiteral("参数快照"));
        exportedSections.append(QStringLiteral("控制快照"));
        exportedSections.append(QStringLiteral("诊断条目汇总"));
        exportedSections.append(QStringLiteral("轨迹状态"));
        exportedSections.append(QStringLiteral("消息历史"));
        exportedSections.append(QStringLiteral("UDP统计"));
        exportedSections.append(QStringLiteral("故障记录"));
        root.insert(QStringLiteral("导出项"), exportedSections);

        root.insert(QStringLiteral("参数快照"), buildParameterConfigSnapshot());

        QJsonObject controlSnapshotObject;
        controlSnapshotObject.insert(QStringLiteral("control_worker_available"), controlWorker != nullptr);
        if(controlWorker){
            controlSnapshotObject.insert(QStringLiteral("control_worker_snapshot"),
                                         buildControlSnapshotJson(controlWorker->latestSnapshot()));
        }
        const HardwareInterface::DiagnosticsSnapshot hardwareDiagnostics =
                hardwareInterface.diagnosticsSnapshot();
        QJsonObject hardwareDiagnosticsObject;
        hardwareDiagnosticsObject.insert(
                    QStringLiteral("communication_event_count"),
                    static_cast<qint64>(std::min<quint64>(
                                            hardwareDiagnostics.communicationEventCount,
                                            static_cast<quint64>(std::numeric_limits<qint64>::max()))));
        hardwareDiagnosticsObject.insert(
                    QStringLiteral("communication_interval_count"),
                    static_cast<qint64>(std::min<quint64>(
                                            hardwareDiagnostics.communicationIntervalCount,
                                            static_cast<quint64>(std::numeric_limits<qint64>::max()))));
        hardwareDiagnosticsObject.insert(QStringLiteral("communication_interval_sum_us"),
                                         hardwareDiagnostics.communicationIntervalSumUs);
        hardwareDiagnosticsObject.insert(QStringLiteral("latest_communication_interval_us"),
                                         hardwareDiagnostics.latestCommunicationIntervalUs);
        hardwareDiagnosticsObject.insert(
                    QStringLiteral("motor_command_event_count"),
                    static_cast<qint64>(std::min<quint64>(
                                            hardwareDiagnostics.motorCommandEventCount,
                                            static_cast<quint64>(std::numeric_limits<qint64>::max()))));
        hardwareDiagnosticsObject.insert(
                    QStringLiteral("motor_command_interval_count"),
                    static_cast<qint64>(std::min<quint64>(
                                            hardwareDiagnostics.motorCommandIntervalCount,
                                            static_cast<quint64>(std::numeric_limits<qint64>::max()))));
        hardwareDiagnosticsObject.insert(QStringLiteral("motor_command_interval_sum_us"),
                                         hardwareDiagnostics.motorCommandIntervalSumUs);
        hardwareDiagnosticsObject.insert(QStringLiteral("latest_motor_command_interval_us"),
                                         hardwareDiagnostics.latestMotorCommandIntervalUs);
        controlSnapshotObject.insert(QStringLiteral("hardware_diagnostics_snapshot"),
                                     hardwareDiagnosticsObject);
        root.insert(QStringLiteral("控制快照"), controlSnapshotObject);

        root.insert(QStringLiteral("诊断条目汇总"), buildRuntimeDiagnosticsItemSummariesJson());
        root.insert(QStringLiteral("轨迹状态"), buildRuntimeDiagnosticsTrajectoryStatusJson());
        root.insert(QStringLiteral("消息历史"), buildRuntimeDiagnosticsMessageHistoryJson(100));
        root.insert(QStringLiteral("UDP统计"), buildRuntimeDiagnosticsUdpStatsJson());
        root.insert(QStringLiteral("故障记录"), buildRuntimeDiagnosticsFaultRecordsJson(50));
        return root;
    }

}

bool MainWindow::writeRuntimeDiagnosticsReport(QString* outputPath,
                                               bool announce,
                                               const QString& outputDirPath)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        if(announce){
            displayInfo("运行诊断记录与导出已由性能开关停用", "warning");
        }
        return false;
    }
    if(runtimeDiagnosticsReportWriting){
        if(announce){
            displayInfo("诊断报告正在写入，请稍后再试", "warning");
        }
        return false;
    }

    const QString noDataReason = runtimeDiagnosticsExportNoDataReason();
    if(!noDataReason.isEmpty()){
        if(announce){
            displayInfo(QStringLiteral("没有运行诊断数据可导出：%1").arg(noDataReason).toStdString(),
                        "warning");
        }
        return false;
    }

    QString dirPath = outputDirPath.trimmed();
    if(dirPath.isEmpty() && announce){
        dirPath = selectOutputDirectoryWithEditableText(
                    this,
                    QStringLiteral("选择运行诊断导出目录"),
                    QStringLiteral("运行诊断导出目录"),
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
            displayInfo("未选择运行诊断导出目录，已取消导出；内存诊断数据已保留",
                        "warning");
        }
        return false;
    }
    if(dirValidation.error == OutputDirectoryValidationError::NotExists){
        if(announce){
            displayInfo(QStringLiteral("错误：运行诊断导出目录不存在：%1；内存诊断数据已保留，请重新选择有效目录")
                        .arg(dirValidation.cleanPath).toStdString(),
                        "error");
        }
        return false;
    }
    if(dirValidation.error == OutputDirectoryValidationError::NotDirectory){
        if(announce){
            displayInfo(QStringLiteral("错误：运行诊断导出路径不是目录：%1；内存诊断数据已保留")
                        .arg(dirValidation.cleanPath).toStdString(),
                        "error");
        }
        return false;
    }
    if(dirValidation.error == OutputDirectoryValidationError::NotWritable){
        if(announce){
            displayInfo(QStringLiteral("错误：运行诊断导出目录不可写：%1；请检查路径或权限，内存诊断数据已保留")
                        .arg(dirValidation.cleanPath).toStdString(),
                        "error");
        }
        return false;
    }
    dirPath = dirValidation.cleanPath;

    runtimeDiagnosticsReportWriting = true;
    struct ReportWritingGuard {
        bool& flag;
        ~ReportWritingGuard(){ flag = false; }
    } reportWritingGuard{runtimeDiagnosticsReportWriting};

    refreshSafetyMonitorHeartbeat();
    const QJsonObject report = buildRuntimeDiagnosticsReportJson();

    const QString reportFilePath =
            QDir(dirPath).filePath(QFileInfo(runtimeDiagnosticsReportFilePath()).fileName());
    QSaveFile reportFile(reportFilePath);
    if(!reportFile.open(QIODevice::WriteOnly | QIODevice::Text)){
        if(announce){
            displayInfo(QStringLiteral("错误：无法写入运行诊断报告 %1；请检查路径或权限，内存诊断数据已保留")
                        .arg(reportFilePath).toStdString(),
                        "error");
        }
        return false;
    }

    bool csvWriteOk = false;
    {
        QTextStream stream(&reportFile);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
        stream.setCodec("UTF-8");
#endif
        csvWriteOk = CsvExport::writeJsonLongTable(stream, QJsonValue(report));
        stream.flush();
        csvWriteOk = csvWriteOk && stream.status() == QTextStream::Ok;
    }
    if(!csvWriteOk){
        reportFile.cancelWriting();
        if(announce){
            displayInfo(QStringLiteral("错误：写入运行诊断 CSV 失败 %1；请检查磁盘空间、路径或权限，内存诊断数据已保留")
                        .arg(reportFilePath).toStdString(),
                        "error");
        }
        return false;
    }
    if(!reportFile.commit()){
        if(announce){
            displayInfo(QStringLiteral("错误：提交运行诊断报告失败 %1；请检查路径或权限，内存诊断数据已保留")
                        .arg(reportFilePath).toStdString(),
                        "error");
        }
        return false;
    }
    refreshSafetyMonitorHeartbeat();

    lastRuntimeDiagnosticsReportPath = reportFilePath;
    lastRuntimeDiagnosticsAutoWriteMs = QDateTime::currentMSecsSinceEpoch();
    refreshRuntimeDiagnosticsUi();

    if(outputPath){
        *outputPath = reportFilePath;
    }
    if(announce){
        displayInfo(QStringLiteral("运行诊断快照已导出至 %1")
                    .arg(reportFilePath).toStdString());
    }
    return true;
}

void MainWindow::updateRuntimeDiagnostics()
{
    if(!kEnableRuntimeDiagnosticsRecording){
        return;
    }
    if(!controlWorker){
        return;
    }

    static qint64 lastDiagnosticsUpdateMs = 0;
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(lastDiagnosticsUpdateMs > 0 && nowMs - lastDiagnosticsUpdateMs < 500){
        return;
    }
    lastDiagnosticsUpdateMs = nowMs;

    RuntimeDiagnosticsSample sample;
    sample.capturedAtMs = nowMs;
    sample.controlTiming = controlWorker->latestSnapshot().timingDiagnostics;
    sample.hardwareTiming = hardwareInterface.diagnosticsSnapshot();

    runtimeDiagnosticsHistory.append(sample);
    trimRuntimeDiagnosticsHistory(sample.capturedAtMs);
    refreshRuntimeDiagnosticsUi();

    const RuntimeDiagnosticsSummary summary = buildRuntimeDiagnosticsSummary();
    if(kEnableRuntimeDiagnosticsAutoReport &&
            !runtimeDiagnosticsReportWriting &&
            summary.windowDurationMs >= kRuntimeDiagnosticsWindowMs &&
            (lastRuntimeDiagnosticsAutoWriteMs <= 0 ||
             sample.capturedAtMs - lastRuntimeDiagnosticsAutoWriteMs >= kRuntimeDiagnosticsAutoWriteIntervalMs)){
        if(!writeRuntimeDiagnosticsReport(nullptr, false)){
            lastRuntimeDiagnosticsAutoWriteMs = sample.capturedAtMs;
        }
    }
}
