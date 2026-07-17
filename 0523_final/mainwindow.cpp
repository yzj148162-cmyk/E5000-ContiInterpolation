#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "monitorthread.h"
#include "motortorquetestworker.h"
#include "safetymonitor.h"
#include "singlemotortrajectoryworker.h"
#include "udpcommworker.h"
#include "nokovrigidbodyframe.h"
#include "trajectoryplanner.h"
#include "barycenter.h"
#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDesktopServices>
#include <QDialog>
#include <QElapsedTimer>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMutex>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QSpinBox>
#include <QTabBar>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QTimer>
#include <QUrl>
#include <QVariant>
#include <QVBoxLayout>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <eh.h>
#include <exception>
#include <functional>
#include <limits>
#include <cmath>
#include <string>

static QFile* gLogFile = nullptr;
static QMutex gLogMutex;
static bool gIsLogging = false;
static QtMessageHandler gPrevHandler = nullptr;
static std::atomic<HardwareInterface*> gSoftwareFaultGuardHardware{nullptr};
static std::atomic_bool gSoftwareFaultGuardInstalled{false};
static std::atomic_bool gSoftwareFaultGuardHandling{false};
static std::wstring gSoftwareFaultGuardLogPath;

namespace {
constexpr int kBarycenterCableCount = 8;
constexpr double kBarycenterForceMinN = 20.0;
constexpr double kBarycenterForceMaxN = 997.0;

const QString kRunModeProgram = QStringLiteral("程序控制");
const QString kRunModeRealtime = QStringLiteral("实时运动轨迹控制");
const QString kRunModeSimulation = QStringLiteral("仿真计算");
const QString kRealtimeInputManual = QStringLiteral("手动位姿指令");
const QString kRealtimeInputExternal = QStringLiteral("外部轨迹文件");

const QString kPoseSimButtonText = QStringLiteral("轨迹仿真");
const QString kPoseSimButtonBusyText = QStringLiteral("轨迹仿真进行中...");
constexpr bool kPoseSimulationTestBypass = false;

const QString kPoseTrajStraight = QStringLiteral("直线轨迹");
const QString kPoseTrajCircular = QStringLiteral("圆形轨迹");
const QString kPoseTrajLineAccel = QStringLiteral("匀加减速直线轨迹");
const QString kPoseTrajEightShape = QStringLiteral("变姿态8字形轨迹");
const QString kPoseTrajStepAccel = QStringLiteral("阶跃变加速轨迹");
const QString kPoseTrajSineAccel = QStringLiteral("正弦变加速轨迹");
const QString kPoseTrajCubic = QStringLiteral("三次多项式轨迹");
const QString kParameterTemplateG3 = QStringLiteral("G3 标准模板");
const QString kParameterTemplateCurrent = QStringLiteral("当前界面配置");
constexpr int kUdpRealtimeLocalPort = 5000;
constexpr int kUdpRealtimeDefaultRemotePort = 5001;
constexpr int kUdpRealtimeSendIntervalMs = 200;
const QString kUdpRealtimeRemoteIp = QStringLiteral("127.0.0.1");
constexpr double kDefaultAxisForceMaxN = 997.0;
constexpr double kDefaultCablePretensionN = 20.0;
constexpr double kSafetyWorkspaceZMinMm = -40.0;
constexpr double kDefaultMotorVelLimitRevPerSec = 10.0;
constexpr double kTorqueServoVelocityLimitRpm = 600.01;
constexpr double kCalibrationHomeLiftMm = 500.0;
constexpr double kCalibrationHomeApproachOffsetMm = 20.0;
constexpr double kCalibrationLinearSpeedMmPerSec = 120.0;
constexpr double kCalibrationAngularSpeedRadPerSec = 0.35;
constexpr double kCalibrationMinSegmentDurationSec = 1.5;
constexpr double kCalibrationSegmentDurationPaddingSec = 0.5;
// 全局初始位姿：位置使用 mm，姿态由用户给定的 deg 预先转换为程序内部使用的 rad。
constexpr double kDegToRad = 0.01745329251994329576923690768489;
constexpr double kGlobalInitialPosePxMm = -224.706310264221;
constexpr double kGlobalInitialPosePyMm = -85.8319471399350;
constexpr double kGlobalInitialPosePzMm = -30.1080677007462;
constexpr double kGlobalInitialPoseRxRad = -0.546779263599080 * kDegToRad;
constexpr double kGlobalInitialPoseRyRad = 0.506976217762942 * kDegToRad;
constexpr double kGlobalInitialPoseRzRad = 0.373864325999395 * kDegToRad;
constexpr int kConnectionStatusRefreshIntervalMs = 1000;
constexpr qint64 kRuntimeDiagnosticsWindowMs = 5 * 60 * 1000;
constexpr qint64 kRuntimeDiagnosticsAutoWriteIntervalMs = 5 * 60 * 1000;
constexpr int kDiagnosticRawWritePumpLines = 4096;
constexpr int kLeadshineHardwareAxisMin = 0;
constexpr int kLeadshineHardwareAxisMax = 11;

template <typename T>
T* findOptionalUiObject(const QWidget* root, const char* objectName)
{
    return root ? root->findChild<T*>(QString::fromLatin1(objectName)) : nullptr;
}

bool optionalCheckBoxChecked(const QWidget* root, const char* objectName, bool fallback = false)
{
    if(QCheckBox* checkBox = findOptionalUiObject<QCheckBox>(root, objectName)){
        return checkBox->isChecked();
    }
    return fallback;
}

void optionalCheckBoxSetChecked(QWidget* root, const char* objectName, bool checked)
{
    if(QCheckBox* checkBox = findOptionalUiObject<QCheckBox>(root, objectName)){
        checkBox->setChecked(checked);
    }
}

double optionalSpinBoxValue(const QWidget* root, const char* objectName, double fallback = 0.0)
{
    if(QDoubleSpinBox* spinBox = findOptionalUiObject<QDoubleSpinBox>(root, objectName)){
        return spinBox->value();
    }
    return fallback;
}

void optionalSpinBoxSetValue(QWidget* root, const char* objectName, double value)
{
    if(QDoubleSpinBox* spinBox = findOptionalUiObject<QDoubleSpinBox>(root, objectName)){
        spinBox->setValue(value);
    }
}

QString optionalComboCurrentText(const QWidget* root, const char* objectName, const QString& fallback = QString())
{
    if(QComboBox* comboBox = findOptionalUiObject<QComboBox>(root, objectName)){
        return comboBox->currentText();
    }
    return fallback;
}

int optionalComboCurrentDataInt(const QWidget* root, const char* objectName, int fallback = 0)
{
    if(QComboBox* comboBox = findOptionalUiObject<QComboBox>(root, objectName)){
        const QVariant data = comboBox->currentData();
        if(data.isValid()){
            return data.toInt();
        }
        return comboBox->currentIndex();
    }
    return fallback;
}

void optionalButtonSetEnabled(QWidget* root, const char* objectName, bool enabled)
{
    if(QAbstractButton* button = findOptionalUiObject<QAbstractButton>(root, objectName)){
        button->setEnabled(enabled);
    }
}

void optionalButtonSetText(QWidget* root, const char* objectName, const QString& text)
{
    if(QAbstractButton* button = findOptionalUiObject<QAbstractButton>(root, objectName)){
        button->setText(text);
    }
}

std::string buildSoftwareFaultGuardRecordLine(const char* eventType,
                                              const char* source,
                                              const char* summary,
                                              const char* detail,
                                              bool stopAttempted,
                                              bool stopSucceeded,
                                              unsigned long extraCode = 0)
{
    SYSTEMTIME now;
    GetLocalTime(&now);

    char timestamp[64] = {};
    std::snprintf(timestamp,
                  sizeof(timestamp),
                  "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
                  now.wYear,
                  now.wMonth,
                  now.wDay,
                  now.wHour,
                  now.wMinute,
                  now.wSecond,
                  now.wMilliseconds);

    char buffer[2048] = {};
    if(extraCode != 0){
        std::snprintf(buffer,
                      sizeof(buffer),
                      "{\"schema_version\":1,\"logged_at\":\"%s\",\"event_type\":\"%s\","
                      "\"source\":\"%s\",\"summary\":\"%s\",\"detail\":\"%s\","
                      "\"stop_action_attempted\":%s,\"stop_action_succeeded\":%s,"
                      "\"exception_code\":\"0x%08lX\"}\n",
                      timestamp,
                      eventType,
                      source,
                      summary,
                      detail,
                      stopAttempted ? "true" : "false",
                      stopSucceeded ? "true" : "false",
                      extraCode);
    }
    else{
        std::snprintf(buffer,
                      sizeof(buffer),
                      "{\"schema_version\":1,\"logged_at\":\"%s\",\"event_type\":\"%s\","
                      "\"source\":\"%s\",\"summary\":\"%s\",\"detail\":\"%s\","
                      "\"stop_action_attempted\":%s,\"stop_action_succeeded\":%s}\n",
                      timestamp,
                      eventType,
                      source,
                      summary,
                      detail,
                      stopAttempted ? "true" : "false",
                      stopSucceeded ? "true" : "false");
    }
    return std::string(buffer);
}

void appendSoftwareFaultGuardRecord(const char* eventType,
                                    const char* source,
                                    const char* summary,
                                    const char* detail,
                                    bool stopAttempted,
                                    bool stopSucceeded,
                                    unsigned long extraCode = 0)
{
    if(gSoftwareFaultGuardLogPath.empty()){
        return;
    }

    HANDLE fileHandle = CreateFileW(gSoftwareFaultGuardLogPath.c_str(),
                                    FILE_APPEND_DATA,
                                    FILE_SHARE_READ | FILE_SHARE_WRITE,
                                    nullptr,
                                    OPEN_ALWAYS,
                                    FILE_ATTRIBUTE_NORMAL,
                                    nullptr);
    if(fileHandle == INVALID_HANDLE_VALUE){
        return;
    }

    const std::string line = buildSoftwareFaultGuardRecordLine(eventType,
                                                               source,
                                                               summary,
                                                               detail,
                                                               stopAttempted,
                                                               stopSucceeded,
                                                               extraCode);
    DWORD written = 0;
    WriteFile(fileHandle,
              line.data(),
              static_cast<DWORD>(line.size()),
              &written,
              nullptr);
    CloseHandle(fileHandle);
}

bool attemptSoftwareFaultGuardEmergencyStop()
{
    HardwareInterface* hardware = gSoftwareFaultGuardHardware.load();
    if(!hardware){
        return false;
    }
    return hardware->emergencyStopAll();
}

LONG WINAPI softwareFaultGuardUnhandledExceptionFilter(EXCEPTION_POINTERS* exceptionInfo)
{
    if(!gSoftwareFaultGuardHandling.exchange(true)){
        const unsigned long exceptionCode =
                exceptionInfo && exceptionInfo->ExceptionRecord ?
                    exceptionInfo->ExceptionRecord->ExceptionCode : 0UL;
        const bool stopOk = attemptSoftwareFaultGuardEmergencyStop();
        appendSoftwareFaultGuardRecord("process_unhandled_exception",
                                       "SetUnhandledExceptionFilter",
                                       "software_crash_guard_triggered",
                                       "Unhandled exception captured; best-effort emergency stop issued before process termination",
                                       true,
                                       stopOk,
                                       exceptionCode);
    }
    return EXCEPTION_EXECUTE_HANDLER;
}

void softwareFaultGuardTerminateHandler()
{
    if(!gSoftwareFaultGuardHandling.exchange(true)){
        const bool stopOk = attemptSoftwareFaultGuardEmergencyStop();
        appendSoftwareFaultGuardRecord("process_terminate",
                                       "std::terminate",
                                       "software_crash_guard_triggered",
                                       "Unhandled terminate reached; best-effort emergency stop issued before process termination",
                                       true,
                                       stopOk);
    }
    TerminateProcess(GetCurrentProcess(), 3);
}

void softwareFaultGuardSignalHandler(int signalNumber)
{
    if(!gSoftwareFaultGuardHandling.exchange(true)){
        char detail[128] = {};
        std::snprintf(detail,
                      sizeof(detail),
                      "Fatal C signal %d captured; best-effort emergency stop issued before process termination",
                      signalNumber);
        const bool stopOk = attemptSoftwareFaultGuardEmergencyStop();
        appendSoftwareFaultGuardRecord("process_signal",
                                       "std::signal",
                                       "software_crash_guard_triggered",
                                       detail,
                                       true,
                                       stopOk,
                                       static_cast<unsigned long>(signalNumber));
    }
    TerminateProcess(GetCurrentProcess(), static_cast<UINT>(128 + signalNumber));
}

QString messageTypeDisplayName(const QString& type)
{
    if(type == QStringLiteral("error")){
        return QStringLiteral("错误");
    }
    if(type == QStringLiteral("warning")){
        return QStringLiteral("警告");
    }
    if(type == QStringLiteral("reset")){
        return QStringLiteral("重置");
    }
    return QStringLiteral("信息");
}

QString connectionStateDisplayText(HardwareInterface::ConnectionState state)
{
    switch(state){
    case HardwareInterface::ConnectionState::Connected:
        return QStringLiteral("连接");
    case HardwareInterface::ConnectionState::Disabled:
        return QStringLiteral("未使能");
    case HardwareInterface::ConnectionState::Fault:
        return QStringLiteral("异常");
    case HardwareInterface::ConnectionState::Disconnected:
    default:
        return QStringLiteral("断开");
    }
}

QString connectionStateStyle(HardwareInterface::ConnectionState state)
{
    switch(state){
    case HardwareInterface::ConnectionState::Connected:
        return QStringLiteral(
                    "QLabel { color: #166534; background: #dcfce7; border: 1px solid #86efac; "
                    "border-radius: 6px; padding: 4px 6px; font-weight: 700; }");
    case HardwareInterface::ConnectionState::Disabled:
        return QStringLiteral(
                    "QLabel { color: #9a3412; background: #ffedd5; border: 1px solid #fdba74; "
                    "border-radius: 6px; padding: 4px 6px; font-weight: 700; }");
    case HardwareInterface::ConnectionState::Fault:
        return QStringLiteral(
                    "QLabel { color: #991b1b; background: #fee2e2; border: 1px solid #fca5a5; "
                    "border-radius: 6px; padding: 4px 6px; font-weight: 700; }");
    case HardwareInterface::ConnectionState::Disconnected:
    default:
        return QStringLiteral(
                    "QLabel { color: #475569; background: #e2e8f0; border: 1px solid #cbd5e1; "
                    "border-radius: 6px; padding: 4px 6px; font-weight: 600; }");
    }
}

bool shouldPersistParameterWidget(const QWidget* widget)
{
    if(!widget){
        return false;
    }
    if(widget->objectName().isEmpty() || widget->objectName().startsWith(QStringLiteral("qt_"))){
        return false;
    }
    if(widget->property("skipConfig").toString() == QStringLiteral("true")){
        return false;
    }
    return qobject_cast<const QSpinBox*>(widget) ||
            qobject_cast<const QDoubleSpinBox*>(widget) ||
            qobject_cast<const QCheckBox*>(widget) ||
            qobject_cast<const QRadioButton*>(widget) ||
            qobject_cast<const QComboBox*>(widget);
}

QJsonArray toJsonArray(const std::vector<double>& values)
{
    QJsonArray array;
    for(double value : values){
        array.append(value);
    }
    return array;
}

QJsonArray toJsonArray(const std::vector<std::vector<double>>& values)
{
    QJsonArray array;
    for(const std::vector<double>& row : values){
        array.append(toJsonArray(row));
    }
    return array;
}

std::vector<double> fromJsonArray(const QJsonArray& array)
{
    std::vector<double> values;
    values.reserve(array.size());
    for(const QJsonValue& value : array){
        values.push_back(value.toDouble());
    }
    return values;
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

QString normalizePoseTrajectoryModeName(const QString& mode){
    if(mode == QStringLiteral("直线")){
        return kPoseTrajStraight;
    }
    if(mode == QStringLiteral("圆弧")){
        return kPoseTrajCircular;
    }
    if(mode == QStringLiteral("S曲线")){
        return kPoseTrajLineAccel;
    }
    if(mode == QStringLiteral("八字形")){
        return kPoseTrajEightShape;
    }
    if(mode == QStringLiteral("阶跃加速度")){
        return kPoseTrajStepAccel;
    }
    if(mode == QStringLiteral("正弦加速度")){
        return kPoseTrajSineAccel;
    }
    if(mode == QStringLiteral("三次多项式直线")){
        return kPoseTrajCubic;
    }
    return mode;
}

QStringList builtInPoseTrajectoryModes(){
    return {
        kPoseTrajStraight,
        kPoseTrajCircular,
        kPoseTrajLineAccel,
        kPoseTrajEightShape,
        kPoseTrajStepAccel,
        kPoseTrajSineAccel,
        kPoseTrajCubic
    };
}

void setPoseSimulationIdleText(Ui::MainWindow* ui){
    if(ui && ui->mainPosModeSwitch){
        ui->mainPosModeSwitch->setText(kPoseSimButtonText);
    }
}

void setPoseSimulationBusyText(Ui::MainWindow* ui){
    if(ui && ui->mainPosModeSwitch){
        ui->mainPosModeSwitch->setText(kPoseSimButtonBusyText);
    }
}

std::vector<double> estimateUdpTrajectoryDerivative(const std::vector<double>& samples,
                                                    const std::vector<double>& timeAxis)
{
    std::vector<double> derivative(samples.size(), 0.0);
    if(samples.size() != timeAxis.size() || samples.size() < 2){
        return derivative;
    }

    for(size_t index=0; index<samples.size(); ++index){
        if(index == 0){
            const double dt = timeAxis[1] - timeAxis[0];
            derivative[index] = dt > 1e-9 ? (samples[1] - samples[0]) / dt : 0.0;
        }
        else if(index + 1 == samples.size()){
            const double dt = timeAxis[index] - timeAxis[index - 1];
            derivative[index] = dt > 1e-9 ? (samples[index] - samples[index - 1]) / dt : 0.0;
        }
        else{
            const double dt = timeAxis[index + 1] - timeAxis[index - 1];
            derivative[index] = dt > 1e-9 ? (samples[index + 1] - samples[index - 1]) / dt : 0.0;
        }
    }

    return derivative;
}

bool buildUdpOfflineTrajectory(const QVector<QVector<double>>& points,
                               int expectedEndNum,
                               TrajectoryPlanner::MultiEndTrajectory& outTraj,
                               double& outDuration,
                               std::vector<double>& outStartPose,
                               QString& errorMessage)
{
    if(expectedEndNum != 1){
        errorMessage = QStringLiteral("当前 UDP 轨迹接入仅支持单末端，与界面配置不匹配");
        return false;
    }
    if(points.size() < 2){
        errorMessage = QStringLiteral("UDP 轨迹点数量至少应为 2");
        return false;
    }

    std::vector<double> timeAxis;
    timeAxis.reserve(points.size());
    std::vector<std::vector<double>> poseSamples(6);
    for(auto& samples : poseSamples){
        samples.reserve(points.size());
    }

    double previousTime = -1.0;
    for(int pointIndex=0; pointIndex<points.size(); ++pointIndex){
        const QVector<double>& point = points[pointIndex];
        if(point.size() != 7){
            errorMessage = QStringLiteral("UDP 轨迹点格式错误，第 %1 个点不是 [t, x, y, z, rx, ry, rz]").arg(pointIndex + 1);
            return false;
        }

        const double timeValue = point[0];
        if(!std::isfinite(timeValue)){
            errorMessage = QStringLiteral("UDP 轨迹点时间戳无效，第 %1 个点不是有限数值").arg(pointIndex + 1);
            return false;
        }
        if(pointIndex > 0 && timeValue <= previousTime){
            errorMessage = QStringLiteral("UDP 轨迹时间戳必须严格递增，第 %1 个点时间无效").arg(pointIndex + 1);
            return false;
        }
        previousTime = timeValue;
        timeAxis.push_back(timeValue);

        for(int dim=0; dim<6; ++dim){
            const double sample = point[dim + 1];
            if(!std::isfinite(sample)){
                errorMessage = QStringLiteral("UDP 轨迹点位姿无效，第 %1 个点第 %2 维不是有限数值")
                        .arg(pointIndex + 1)
                        .arg(dim + 1);
                return false;
            }
            poseSamples[dim].push_back(sample);
        }
    }

    outTraj.clear();
    outTraj.resize(1);
    outTraj[0].resize(4);
    outTraj[0][0].resize(6);
    outTraj[0][1].resize(6);
    outTraj[0][2].resize(6);
    outTraj[0][3].resize(6);

    outStartPose.assign(6, 0.0);
    for(int dim=0; dim<6; ++dim){
        outTraj[0][0][dim] = poseSamples[dim];
        outTraj[0][1][dim] = estimateUdpTrajectoryDerivative(outTraj[0][0][dim], timeAxis);
        outTraj[0][2][dim] = estimateUdpTrajectoryDerivative(outTraj[0][1][dim], timeAxis);
        outTraj[0][3][dim] = timeAxis;
        outStartPose[dim] = outTraj[0][0][dim].front();
    }

    outDuration = timeAxis.back();
    return true;
}

void invokeOnObjectThreadWithHeartbeat(QObject* receiver,
                                       const std::function<void()>& work,
                                       const std::function<void()>& heartbeat)
{
    if(!receiver){
        return;
    }

    QThread* receiverThread = receiver->thread();
    if(!receiverThread ||
            !receiverThread->isRunning() ||
            QThread::currentThread() == receiverThread){
        work();
        if(heartbeat){
            heartbeat();
        }
        return;
    }

    std::atomic_bool done{false};
    const bool posted = QMetaObject::invokeMethod(receiver, [&work, &done](){
        work();
        done.store(true, std::memory_order_release);
    }, Qt::QueuedConnection);
    if(!posted){
        work();
        if(heartbeat){
            heartbeat();
        }
        return;
    }

    while(!done.load(std::memory_order_acquire)){
        if(heartbeat){
            heartbeat();
        }
        QThread::msleep(10);
    }
    if(heartbeat){
        heartbeat();
    }
}

void rescaleCustomPlotYAxis(QCustomPlot* plot)
{
    if(!plot){
        return;
    }

    bool hasData = false;
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    for(int i=0; i<plot->graphCount(); ++i){
        QCPGraph* graph = plot->graph(i);
        if(!graph || graph->dataCount() == 0){
            continue;
        }
        bool foundRange = false;
        const QCPRange range = graph->getValueRange(foundRange, QCP::sdBoth);
        if(!foundRange){
            continue;
        }
        minValue = std::min(minValue, range.lower);
        maxValue = std::max(maxValue, range.upper);
        hasData = true;
    }

    if(!hasData){
        plot->yAxis->setRange(-1.0, 1.0);
        return;
    }
    if(qFuzzyCompare(minValue, maxValue)){
        minValue -= 1.0;
        maxValue += 1.0;
    }
    else{
        const double padding = std::max(1e-6, (maxValue - minValue) * 0.1);
        minValue -= padding;
        maxValue += padding;
    }
    plot->yAxis->setRange(minValue, maxValue);
}
}

static void msgOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg){
    if(gPrevHandler)
        gPrevHandler(type, context, msg);

    if(!gIsLogging || !gLogFile)
        return;

    QByteArray localMsg = msg.toLocal8Bit();
    const char *file = context.file ? context.file : "";
    const char *function = context.function ? context.function : "";

    QString strOutStream;
    switch (type) {
    case QtDebugMsg:
        strOutStream = QString("%1 %2 %3 %4 [Debug] %5\n").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz")).arg(file).arg(context.line).arg(function).arg(QString(localMsg));
        break;
    case QtInfoMsg:
        strOutStream = QString("%1 %2 %3 %4 [Info] %5\n").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz")).arg(file).arg(context.line).arg(function).arg(QString(localMsg));
        break;
    case QtWarningMsg:
        strOutStream = QString("%1 %2 %3 %4 [Warning] %5\n").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz")).arg(file).arg(context.line).arg(function).arg(QString(localMsg));
        break;
    case QtCriticalMsg:
        strOutStream = QString("%1 %2 %3 %4 [Critical] %5\n").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz")).arg(file).arg(context.line).arg(function).arg(QString(localMsg));
        break;
    case QtFatalMsg:
        strOutStream = QString("%1 %2 %3 %4 [Fatal] %5\n").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz")).arg(file).arg(context.line).arg(function).arg(QString(localMsg));
        break;
    }

    QMutexLocker locker(&gLogMutex);
    if(gLogFile->isOpen()){
        QTextStream stream(gLogFile);
        stream << strOutStream;
        stream.flush();
    }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initializeLegacyGcUiFallbacks();
    
    qRegisterMetaType<std::vector<double>>("std::vector<double>&");
    qRegisterMetaType<std::vector<std::vector<double>>>("std::vector<std::vector<double>>&");
    qRegisterMetaType<std::string>("std::string&");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<QVector<QVector3D>>("QVector<QVector3D>&");

    qRegisterMetaType<QVector<double>>("QVector<double>");// 注册double向量，以实现关于该类型变量的信号和槽的映射
    qRegisterMetaType<UdpPoseCommand>("UdpPoseCommand");
    qRegisterMetaType<UdpTrajectoryChunk>("UdpTrajectoryChunk");
    qRegisterMetaType<UdpCommStats>("UdpCommStats");
    qRegisterMetaType<UdpStatusPayload>("UdpStatusPayload");
    gPrevHandler = qInstallMessageHandler(msgOutput);
    setUIVec();
    simulationWorker = new SimulationWorker(this);
    hardwareThread = new QThread(this);
    hardwareThread->setObjectName(QStringLiteral("HardwareThread"));
    hardwareInterface.moveToThread(hardwareThread);
    hardwareThread->start(QThread::HighPriority);
    installSoftwareFaultGuards();
    if(initPara()){
        displayInfo("初始化完成");
    }
}

MainWindow::~MainWindow()
{
    uninstallSoftwareFaultGuards();
    if(trajectoryCheckTimer){
        trajectoryCheckTimer->stop();
    }
    if(controlSnapshotTimer){
        controlSnapshotTimer->stop();
    }
    if(safetyHeartbeatTimer){
        safetyHeartbeatTimer->stop();
    }
    stopSingleMotorTrajectoryThread();
    stopMotorTorqueTestThread();
    if(simulationWorker){
        simulationWorker->stopPositionSimulation();
        positionSimulationModel = nullptr;
        simulationWorker->stopForceSimulation();
        forceSimulationModel = nullptr;
        delete simulationWorker;
        simulationWorker = nullptr;
    }
    stopSafetyMonitor();
    stopUdpBridge();
    stopPoseThread(positionSimulationHelper);
    if(forwardKinematicsHelper){
        delete forwardKinematicsHelper;
        forwardKinematicsHelper = nullptr;
    }
    stopControlThread();
    stopPositionThread();
    if(controlWorker){
        delete controlWorker;
        controlWorker = nullptr;
    }
    if(ccThread){
        delete ccThread;
        ccThread = nullptr;
    }
    if(pvtExecutionWorker){
        delete pvtExecutionWorker;
        pvtExecutionWorker = nullptr;
    }
    if(positionThread){
        delete positionThread;
        positionThread = nullptr;
    }
    if(hardwareThread){
        if(hardwareThread->isRunning()){
            hardwareInterface.disconnectLS();
            QThread* mainThread = QCoreApplication::instance()->thread();
            QMetaObject::invokeMethod(&hardwareInterface, [&](){
                hardwareInterface.moveToThread(mainThread);
            }, Qt::BlockingQueuedConnection);
            hardwareThread->quit();
            if(!hardwareThread->wait(1000)){
                hardwareThread->terminate();
                hardwareThread->wait(500);
            }
        }
        delete hardwareThread;
        hardwareThread = nullptr;
    }
    gIsLogging = false;
    if(gLogFile){
        gLogFile->close();
        delete gLogFile;
        gLogFile = nullptr;
    }
    stopMonitor();
    delete ui;
}

MainWindow::RobotStateSnapshot MainWindow::currentRobotState(bool queryHardware) const{
    RobotStateSnapshot state;
    state.systemRunning = runtimeState.systemRunning;
    state.lsConnected = ui->devUseLS->isChecked() &&
            (queryHardware ? hardwareInterface.isLSConnected() : runtimeState.systemRunning);
    state.controlThreadRunning = ccThread && ccThread->isRunning();
    state.cableHomeState = runtimeState.cableHomeState;
    state.runMode = runtimeState.runMode;
    state.posModeRunning = runtimeState.posModeRunning;
    state.gcRunning = runtimeState.gcRunning;
    state.forceControlThreadChecked = ui->mainFCThread->isChecked();

    bool hasForceControlAxis = runtimeState.hybridPoseForceModeActive;
    if(!hasForceControlAxis){
        for(QRadioButton* btn : useFCBtnVec){
            if(btn && btn->isChecked()){
                hasForceControlAxis = true;
                break;
            }
        }
    }
    state.forceControlThreadRunning = state.systemRunning &&
            state.controlThreadRunning &&
            state.forceControlThreadChecked &&
            hasForceControlAxis;
    if(controlWorker && controlWorker->latestSnapshot().forceThreadRunning){
        state.forceControlThreadRunning = true;
    }

    if(state.systemRunning && state.lsConnected && queryHardware){
        state.pvtTrajectoryAvailable = hardwareInterface.hasPvtTrajectory();
        if(state.pvtTrajectoryAvailable){
            state.pvtMotionPaused = hardwareInterface.isPvtMotionPausedState();
            state.pvtMotionRunning = hardwareInterface.isPvtMotionRunning();
            state.pvtMotionFinished = hardwareInterface.isPvtMotionFinished();
        }
        else{
            state.pvtMotionRunning = false;
            state.pvtMotionPaused = false;
            state.pvtMotionFinished = true;
        }
    }
    else{
        state.pvtTrajectoryAvailable = !queryHardware && runtimeState.pvtCommandActive;
        state.pvtMotionPaused = !queryHardware &&
                runtimeState.pvtCommandActive &&
                runtimeState.controlBoxPausedPvtMotion;
        state.pvtMotionRunning = !queryHardware &&
                runtimeState.pvtCommandActive &&
                !state.pvtMotionPaused;
        state.pvtMotionFinished = !runtimeState.pvtCommandActive;
    }

    if(state.pvtMotionPaused){
        state.positionMotionState = PositionMotionState::PausedPvt;
    }
    else if(state.pvtMotionRunning){
        state.positionMotionState = PositionMotionState::ExecutingPvt;
    }
    else if(state.posModeRunning){
        state.positionMotionState = PositionMotionState::Simulating;
    }
    else{
        state.positionMotionState = PositionMotionState::Idle;
    }

    if(state.gcRunning){
        state.forceMotionState = ForceMotionState::GravityCompensation;
    }
    else if(state.forceControlThreadRunning){
        state.forceMotionState = ForceMotionState::DirectForceControl;
    }
    else{
        state.forceMotionState = ForceMotionState::Idle;
    }

    state.singleMotorTrajectoryActive = runtimeState.singleMotorTrajectoryActive;
    state.anyMotionRunning = runtimeState.motorTorqueDebugActive ||
            state.singleMotorTrajectoryActive ||
            state.positionMotionState != PositionMotionState::Idle ||
            state.forceMotionState != ForceMotionState::Idle;
    return state;
}

void MainWindow::initializeLegacyGcUiFallbacks()
{
    auto markLegacyControl = [](QWidget* widget) {
        if(!widget){
            return;
        }
        widget->setProperty("skipConfig", true);
        widget->hide();
    };

    auto ensureCheckBox = [this, &markLegacyControl](const char* objectName, bool checked) {
        if(findOptionalUiObject<QCheckBox>(this, objectName)){
            return;
        }
        QCheckBox* checkBox = new QCheckBox(this);
        checkBox->setObjectName(QString::fromLatin1(objectName));
        checkBox->setChecked(checked);
        markLegacyControl(checkBox);
    };

    auto ensureSpinBox = [this, &markLegacyControl](const char* objectName, double value) {
        if(findOptionalUiObject<QDoubleSpinBox>(this, objectName)){
            return;
        }
        QDoubleSpinBox* spinBox = new QDoubleSpinBox(this);
        spinBox->setObjectName(QString::fromLatin1(objectName));
        spinBox->setDecimals(3);
        spinBox->setMinimum(-99999.99);
        spinBox->setMaximum(99999.99);
        spinBox->setValue(value);
        markLegacyControl(spinBox);
    };

    auto ensureComboBox = [this, &markLegacyControl](const char* objectName, const QStringList& items, int currentIndex) {
        if(findOptionalUiObject<QComboBox>(this, objectName)){
            return;
        }
        QComboBox* comboBox = new QComboBox(this);
        comboBox->setObjectName(QString::fromLatin1(objectName));
        comboBox->addItems(items);
        comboBox->setCurrentIndex(std::clamp(currentIndex, 0, comboBox->count() - 1));
        markLegacyControl(comboBox);
    };

    auto ensureIntButton = [this, &markLegacyControl](const char* objectName, const QString& text) {
        if(findOptionalUiObject<IntBtn>(this, objectName)){
            return;
        }
        IntBtn* button = new IntBtn(this);
        button->setObjectName(QString::fromLatin1(objectName));
        button->setText(text);
        markLegacyControl(button);
    };

    ensureIntButton("mainGCModeSwitch", QStringLiteral("启动零力拖动"));
    ensureIntButton("mainGCResetRotHome", QStringLiteral("更新姿态零点"));

    ensureCheckBox("Forcecontinuetraj", false);
    ensureCheckBox("mainGCUseRot", true);
    ensureCheckBox("mainCompTor", true);
    ensureCheckBox("devGCUseImpCtrl", true);
    ensureCheckBox("mainGCSim", false);

    ensureComboBox("ForcetrajMode", QStringList{QStringLiteral("直线"), QStringLiteral("圆弧")}, 0);

    ensureSpinBox("mainGCCableOptForce", 0.0);
    ensureSpinBox("mainGCEndX", 0.0);
    ensureSpinBox("mainGCEndY", 0.0);
    ensureSpinBox("mainGCEndZ", 10.482);
    ensureSpinBox("mainGCEndMx", 0.0);
    ensureSpinBox("mainGCEndMy", 0.0);
    ensureSpinBox("mainGCEndMz", 0.0);
}

void MainWindow::refreshAxisRuntimeCounts()
{
    const int endCount = std::max(1, ui ? ui->devEndNum->value() : 0);
    axisCableNum = 0;
    eachEndCableNum.assign(endCount, 0);

    const int axisCount = std::min({ui ? ui->devAxisNum->value() : 0,
                                    static_cast<int>(axisTypeVec.size()),
                                    static_cast<int>(axisEndVec.size()),
                                    static_cast<int>(axisSensorIndexVec.size())});
    int maxSensorIndex = -1;
    for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
        const int endIndex = axisEndVec[axisIndex] ? axisEndVec[axisIndex]->value() : -1;
        if(isMotorAxis(axisIndex) && endIndex >= 0 && endIndex < endCount){
            axisCableNum++;
            eachEndCableNum[endIndex]++;
        }

        const int sensorIndex = axisSensorIndexVec[axisIndex] ? axisSensorIndexVec[axisIndex]->value() : -1;
        if(sensorIndex >= 0){
            maxSensorIndex = std::max(maxSensorIndex, sensorIndex);
        }
    }

    axisForceSensorNum = std::max(0, maxSensorIndex + 1);
    if(static_cast<int>(forceSensorCurHome.size()) < axisForceSensorNum){
        forceSensorCurHome.resize(axisForceSensorNum, 0.0);
    }
}

void MainWindow::showPrimaryOperationFeedback(const QString& text, const QString& type, int durationMs)
{
    QString style = QStringLiteral(
                "QLabel { color: #334155; background: #f8fafc; border: 1px solid #cbd5e1; "
                "border-radius: 4px; padding: 6px 8px; font-weight: 600; }");
    if(type == QStringLiteral("error")){
        style = QStringLiteral(
                    "QLabel { color: #991b1b; background: #fee2e2; border: 1px solid #fca5a5; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(type == QStringLiteral("warning")){
        style = QStringLiteral(
                    "QLabel { color: #92400e; background: #fef3c7; border: 1px solid #fcd34d; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(type == QStringLiteral("info")){
        style = QStringLiteral(
                    "QLabel { color: #1d4ed8; background: #dbeafe; border: 1px solid #93c5fd; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }

    primaryOperationFeedbackText = text;
    primaryOperationFeedbackStyle = style;
    primaryOperationFeedbackExpireMs = QDateTime::currentMSecsSinceEpoch() + std::max(0, durationMs);

    if(ui && ui->mainMotionStateLabel){
        ui->mainMotionStateLabel->setText(text);
        ui->mainMotionStateLabel->setStyleSheet(style);
    }

    displayInfo(text.toStdString(), type.toStdString());
}

void MainWindow::initializePrimaryOperationUi()
{
    if(ui->mainSwitch){
        ui->mainSwitch->setStyleSheet(
                    QStringLiteral("QPushButton { background: #dcfce7; color: #166534; font-weight: 600; }"));
    }
    if(ui->mainPauseSwitch){
        ui->mainPauseSwitch->setText(QStringLiteral("暂停"));
        ui->mainPauseSwitch->setStyleSheet(
                    QStringLiteral("QPushButton { background: #fef3c7; color: #92400e; font-weight: 600; }"));
        connect(ui->mainPauseSwitch, &IntBtn::sendInt, this, [this](int){
            togglePauseResumeFromUi();
        });
    }
    if(ui->mainEmergencyStopSwitch){
        ui->mainEmergencyStopSwitch->setText(QStringLiteral("急停"));
        ui->mainEmergencyStopSwitch->setStyleSheet(
                    QStringLiteral("QPushButton { background: #fee2e2; color: #991b1b; font-weight: 700; }"));
        connect(ui->mainEmergencyStopSwitch, &IntBtn::sendInt, this, [this](int){
            triggerSoftwareEmergencyStop();
        });
    }
    if(ui->mainStopSwitch){
        ui->mainStopSwitch->setStyleSheet(
                    QStringLiteral("QPushButton { background: #e2e8f0; color: #334155; font-weight: 600; }"));
    }
    if(ui->mainServoHoldSwitch){
        ui->mainServoHoldSwitch->setText(QStringLiteral("抱闸"));
        ui->mainServoHoldSwitch->setStyleSheet(
                    QStringLiteral("QPushButton { background: #e0f2fe; color: #075985; font-weight: 600; }"));
        ui->mainServoHoldSwitch->setToolTip(
                    QStringLiteral("软件保持功能，不是真实机械抱闸。点击后会先停止并等待停稳，再失能所有轴；再次点击可重新使能。紧急情况请使用“急停”。"));
        connect(ui->mainServoHoldSwitch, &IntBtn::sendInt, this, [this](int){
            toggleServoHoldFromUi();
        });
    }
    if(ui->mainMotionStateLabel){
        ui->mainMotionStateLabel->setText(QStringLiteral("主控状态：待命"));
        ui->mainMotionStateLabel->setStyleSheet(
                    QStringLiteral("QLabel { color: #334155; background: #f8fafc; border: 1px solid #cbd5e1; "
                                   "border-radius: 4px; padding: 6px 8px; font-weight: 600; }"));
    }
}

void MainWindow::initializeConnectionStatusUi()
{
    if(connectionStatusGroupBox || !ui){
        return;
    }

    connectionStatusGroupBox = ui->connectionStatusGroupBox;
    connectionControllerStatusLabel = ui->connectionControllerStatusLabel;
    connectionAxisStatusLabels = {
        ui->connectionAxisStatusLabel_1,
        ui->connectionAxisStatusLabel_2,
        ui->connectionAxisStatusLabel_3,
        ui->connectionAxisStatusLabel_4,
        ui->connectionAxisStatusLabel_5,
        ui->connectionAxisStatusLabel_6,
        ui->connectionAxisStatusLabel_7,
        ui->connectionAxisStatusLabel_8,
        ui->connectionAxisStatusLabel_9,
        ui->connectionAxisStatusLabel_10,
        ui->connectionAxisStatusLabel_11,
        ui->connectionAxisStatusLabel_12
    };
    connectionSensorStatusLabels = {
        ui->connectionSensorStatusLabel_1,
        ui->connectionSensorStatusLabel_2
    };

    if(!connectionStatusGroupBox || !connectionControllerStatusLabel){
        connectionAxisStatusLabels.clear();
        connectionSensorStatusLabels.clear();
        return;
    }

    if(QLabel* legendLabel = ui->connectionStatusLegendLabel){
        legendLabel->setText(QStringLiteral("绿=连接且使能  黄=连接且失能  灰=断开  红=异常"));
    }

    for(QLabel* label : connectionAxisStatusLabels){
        if(!label){
            connectionAxisStatusLabels.clear();
            connectionSensorStatusLabels.clear();
            return;
        }
    }
    for(QLabel* label : connectionSensorStatusLabels){
        if(!label){
            connectionAxisStatusLabels.clear();
            connectionSensorStatusLabels.clear();
            return;
        }
    }

    refreshConnectionStatusUi(true);
}

void MainWindow::setConnectionStatusIndicator(QLabel* label,
                                              const QString& title,
                                              HardwareInterface::ConnectionState state,
                                              const QString& tooltip,
                                              const QString& stateTextOverride)
{
    if(!label){
        return;
    }

    const QString stateText = stateTextOverride.isEmpty() ?
                connectionStateDisplayText(state) :
                stateTextOverride;
    const QString text = QStringLiteral("%1\n%2").arg(title, stateText);
    const QString style = connectionStateStyle(state);

    if(label->text() != text){
        label->setText(text);
    }
    if(label->styleSheet() != style){
        label->setStyleSheet(style);
    }
    if(label->toolTip() != tooltip){
        label->setToolTip(tooltip);
    }
}

void MainWindow::refreshConnectionStatusUi(bool force)
{
    if(!connectionStatusGroupBox || !ui ||
            !connectionControllerStatusLabel ||
            connectionAxisStatusLabels.empty() ||
            connectionSensorStatusLabels.empty()){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(!force &&
            lastConnectionStatusUiRefreshMs >= 0 &&
            (nowMs - lastConnectionStatusUiRefreshMs) < kConnectionStatusRefreshIntervalMs){
        return;
    }

    refreshAxisRuntimeCounts();

    const int axisCount = static_cast<int>(connectionAxisStatusLabels.size());
    const int sensorCount = static_cast<int>(connectionSensorStatusLabels.size());

    const HardwareInterface::ConnectionDiagnostics diagnostics = hardwareInterface.connectionDiagnostics();

    QString controllerTooltip = QStringLiteral("主站状态: %1\n错误码: %2")
            .arg(diagnostics.controller.busState)
            .arg(diagnostics.controller.errorCode);
    if(!ui->devUseLS->isChecked() &&
            diagnostics.controller.state == HardwareInterface::ConnectionState::Disconnected){
        controllerTooltip = QStringLiteral("当前未启用雷赛控制卡。\n%1").arg(controllerTooltip);
    }
    if(diagnostics.controller.apiResult != 0){
        controllerTooltip += QStringLiteral("\n接口返回: %1").arg(diagnostics.controller.apiResult);
    }
    setConnectionStatusIndicator(connectionControllerStatusLabel,
                                 QStringLiteral("控制卡"),
                                 diagnostics.controller.state,
                                 controllerTooltip);

    for(int i = 0; i < axisCount; ++i){
        HardwareInterface::ConnectionState state = HardwareInterface::ConnectionState::Disconnected;
        QString tooltip = QStringLiteral("逻辑轴%1当前未配置为雷赛电机轴。").arg(i);
        QString stateText = QStringLiteral("断开");

        if(i < static_cast<int>(diagnostics.motorAxes.size())){
            const HardwareInterface::ConnectionItemDiagnostics& axis = diagnostics.motorAxes[i];
            state = axis.state;

            QStringList details;
            details << QStringLiteral("逻辑轴: %1").arg(i);
            if(axis.hardwareAxis >= 0){
                details << QStringLiteral("控制卡轴: %1").arg(axis.hardwareAxis);
            }
            if(axis.slaveAddress > 0){
                details << QStringLiteral("从站地址: %1").arg(axis.slaveAddress);
            }
            if(axis.subSlaveAddress > 0){
                details << QStringLiteral("子从站: %1").arg(axis.subSlaveAddress);
            }
            if(axis.busState >= 0){
                details << QStringLiteral("从站状态: %1").arg(axis.busState);
            }
            if(axis.stateMachine >= 0){
                details << QStringLiteral("状态机: %1").arg(axis.stateMachine);
                details << QStringLiteral("伺服使能: %1")
                           .arg(axis.stateMachine == 4 ? QStringLiteral("是") : QStringLiteral("否"));
            }
            details << QStringLiteral("状态字: 0x%1")
                       .arg(QString::number(static_cast<quint32>(axis.statusWord), 16).toUpper());
            details << QStringLiteral("停止原因: %1").arg(axis.stopReason);
            if(axis.stopReasonApiResult != 0){
                details << QStringLiteral("停止原因读取返回: %1").arg(axis.stopReasonApiResult);
            }
            details << QStringLiteral("错误码: %1").arg(axis.errorCode);
            if(axis.apiResult != 0){
                details << QStringLiteral("接口返回: %1").arg(axis.apiResult);
            }
            tooltip = details.join(QLatin1Char('\n'));

            if(state == HardwareInterface::ConnectionState::Connected){
                stateText = QStringLiteral("连接且使能");
            }
            else if(state == HardwareInterface::ConnectionState::Disabled){
                stateText = QStringLiteral("连接且失能");
            }
            else if(state == HardwareInterface::ConnectionState::Fault){
                stateText = QStringLiteral("异常");
            }
        }

        setConnectionStatusIndicator(connectionAxisStatusLabels[i],
                                     QStringLiteral("轴%1").arg(i),
                                     state,
                                     tooltip,
                                     stateText);
    }

    for(int i = 0; i < sensorCount; ++i){
        HardwareInterface::ConnectionState state = HardwareInterface::ConnectionState::Disconnected;
        QString tooltip = QStringLiteral("张力通道%1当前未配置。").arg(i + 1);

        if(i < static_cast<int>(diagnostics.forceSensors.size())){
            const HardwareInterface::ConnectionItemDiagnostics& sensor = diagnostics.forceSensors[i];
            state = sensor.state;

            QStringList details;
            details << QStringLiteral("传感器通道: %1").arg(i + 1);
            if(sensor.apiResult != 0){
                details << QStringLiteral("接口返回: %1").arg(sensor.apiResult);
            }
            else{
                details << QStringLiteral("PDO读取正常");
            }
            tooltip = details.join(QLatin1Char('\n'));
        }

        setConnectionStatusIndicator(connectionSensorStatusLabels[i],
                                     QStringLiteral("力%1").arg(i + 1),
                                     state,
                                     tooltip);
    }

    refreshSingleMotorEnableStateUi();
    lastConnectionStatusUiRefreshMs = nowMs;
}

void MainWindow::refreshPrimaryOperationUiState(const RobotStateSnapshot& robotState)
{
    auto setTextIfChanged = [](QAbstractButton* button, const QString& text){
        if(button && button->text() != text){
            button->setText(text);
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
    auto setLabelTextIfChanged = [](QLabel* label, const QString& text){
        if(label && label->text() != text){
            label->setText(text);
        }
    };
    auto setLabelStyleIfChanged = [](QLabel* label, const QString& style){
        if(label && label->styleSheet() != style){
            label->setStyleSheet(style);
        }
    };

    QString pauseButtonText = QStringLiteral("暂停");
    bool pauseEnabled = true;
    QString servoHoldButtonText = runtimeState.servoHoldActive ?
                QStringLiteral("解除抱闸") :
                QStringLiteral("抱闸");
    QString servoHoldToolTip = runtimeState.servoHoldActive ?
                QStringLiteral("当前处于软件抱闸保持状态（伺服失能，不是真实机械抱闸）。点击后将重新使能所有轴。") :
                QStringLiteral("软件保持功能，不是真实机械抱闸。点击后会先停止并等待停稳，再失能所有轴；再次点击可重新使能。紧急情况请使用“急停”。");
    bool servoHoldEnabled = true;
    QString stateText = QStringLiteral("主控状态：待命");
    QString stateStyle = QStringLiteral(
                "QLabel { color: #334155; background: #f8fafc; border: 1px solid #cbd5e1; "
                "border-radius: 4px; padding: 6px 8px; font-weight: 600; }");

    if(runtimeState.safetyFaultLatched){
        stateText = QStringLiteral("主控状态：安全锁存");
        stateStyle = QStringLiteral(
                    "QLabel { color: #991b1b; background: #fee2e2; border: 1px solid #fca5a5; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(robotState.positionMotionState == PositionMotionState::PausedPvt){
        pauseButtonText = QStringLiteral("继续");
        pauseEnabled = true;
        stateText = QStringLiteral("主控状态：位置轨迹已暂停");
        stateStyle = QStringLiteral(
                    "QLabel { color: #92400e; background: #fef3c7; border: 1px solid #fcd34d; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(robotState.positionMotionState == PositionMotionState::ExecutingPvt){
        pauseEnabled = true;
        stateText = QStringLiteral("主控状态：位置轨迹运行中");
        stateStyle = QStringLiteral(
                    "QLabel { color: #166534; background: #dcfce7; border: 1px solid #86efac; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(robotState.positionMotionState == PositionMotionState::Simulating){
        stateText = QStringLiteral("主控状态：轨迹仿真中");
        stateStyle = QStringLiteral(
                    "QLabel { color: #1d4ed8; background: #dbeafe; border: 1px solid #93c5fd; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(robotState.gcRunning || robotState.forceControlThreadRunning){
        stateText = QStringLiteral("主控状态：力控/零力拖动运行中");
        stateStyle = QStringLiteral(
                    "QLabel { color: #166534; background: #dcfce7; border: 1px solid #86efac; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(runtimeState.servoHoldActive){
        pauseEnabled = false;
        stateText = QStringLiteral("主控状态：软件抱闸保持中（伺服失能）");
        stateStyle = QStringLiteral(
                    "QLabel { color: #075985; background: #e0f2fe; border: 1px solid #7dd3fc; "
                    "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    }
    else if(robotState.systemRunning){
        stateText = QStringLiteral("主控状态：系统就绪");
    }
    else if(robotState.lsConnected){
        stateText = QStringLiteral("主控状态：已连接待上电");
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(primaryOperationFeedbackExpireMs > nowMs && !primaryOperationFeedbackText.isEmpty()){
        stateText = primaryOperationFeedbackText;
        stateStyle = primaryOperationFeedbackStyle;
    }
    else if(primaryOperationFeedbackExpireMs > 0 && nowMs >= primaryOperationFeedbackExpireMs){
        primaryOperationFeedbackExpireMs = 0;
        primaryOperationFeedbackText.clear();
        primaryOperationFeedbackStyle.clear();
    }

    setTextIfChanged(ui->mainPauseSwitch, pauseButtonText);
    setEnabledIfChanged(ui->mainPauseSwitch, pauseEnabled);
    setTextIfChanged(ui->mainServoHoldSwitch, servoHoldButtonText);
    setEnabledIfChanged(ui->mainServoHoldSwitch, servoHoldEnabled);
    setToolTipIfChanged(ui->mainServoHoldSwitch, servoHoldToolTip);
    setLabelTextIfChanged(ui->mainMotionStateLabel, stateText);
    setLabelStyleIfChanged(ui->mainMotionStateLabel, stateStyle);
}

QString MainWindow::calibrationStorageDirPath() const
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("calibration_records"));
}

QString MainWindow::latestCalibrationSnapshotPath() const
{
    return QDir(calibrationStorageDirPath()).filePath(QStringLiteral("last_zero_calibration.json"));
}

QString MainWindow::formatCalibrationVectorPreview(const std::vector<double>& values, int maxCount) const
{
    if(values.empty()){
        return QStringLiteral("[]");
    }

    QStringList parts;
    const int count = std::min(maxCount, static_cast<int>(values.size()));
    for(int i=0; i<count; ++i){
        parts << QString::number(values[i], 'f', 3);
    }
    if(static_cast<int>(values.size()) > count){
        parts << QStringLiteral("...");
    }
    return QStringLiteral("[%1]").arg(parts.join(QStringLiteral(", ")));
}

bool MainWindow::saveCalibrationSnapshotToFile(const QString& filePath, bool announce)
{
    if(filePath.isEmpty()){
        return false;
    }

    refreshAxisRuntimeCounts();

    QFileInfo fileInfo(filePath);
    QDir dir;
    if(!dir.mkpath(fileInfo.absolutePath())){
        displayInfo(QStringLiteral("错误：无法创建零位校准记录目录 %1")
                        .arg(fileInfo.absolutePath()).toStdString(),
                    "error");
        return false;
    }

    std::vector<double> motorHome = hardwareInterface.getAllMotorHome();
    if(motorHome.empty()){
        motorHome = motorCurAbsPos;
    }

    std::vector<double> devEndHome = {
        ui->devEndHomeX->value(),
        ui->devEndHomeY->value(),
        ui->devEndHomeZ->value(),
        ui->devEndHomeRx->value(),
        ui->devEndHomeRy->value(),
        ui->devEndHomeRz->value()
    };

    QString cableHomeStateText = QStringLiteral("unconfirmed");
    if(runtimeState.cableHomeState == CableHomeState::WaitingForceStable){
        cableHomeStateText = QStringLiteral("waiting_force_stable");
    }
    else if(runtimeState.cableHomeState == CableHomeState::Confirmed){
        cableHomeStateText = QStringLiteral("confirmed");
    }

    const QDateTime savedAt = QDateTime::currentDateTime();
    QJsonObject root;
    root.insert(QStringLiteral("saved_at"), savedAt.toString(Qt::ISODate));
    root.insert(QStringLiteral("axis_count"), ui->devAxisNum->value());
    root.insert(QStringLiteral("sensor_count"), axisForceSensorNum);
    root.insert(QStringLiteral("motive_fit_confirmed"), motiveFitConfirmed);
    root.insert(QStringLiteral("cable_home_state"), cableHomeStateText);
    root.insert(QStringLiteral("motor_home"), toJsonArray(motorHome));
    root.insert(QStringLiteral("force_sensor_home"), toJsonArray(forceSensorCurHome));
    root.insert(QStringLiteral("end_cur_rot_home"), toJsonArray(endCurRotHome));
    root.insert(QStringLiteral("dev_end_home"), toJsonArray(devEndHome));
    root.insert(QStringLiteral("summary"), calibrationStatusLabel ? calibrationStatusLabel->text() : QString());

    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        displayInfo(QStringLiteral("错误：无法写入零位校准记录 %1")
                        .arg(filePath).toStdString(),
                    "error");
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();

    lastCalibrationRecordPath = filePath;
    lastCalibrationRecordTime = savedAt;
    lastCalibrationRecordSummary = root.value(QStringLiteral("summary")).toString();
    refreshCalibrationUiState();

    if(announce){
        displayInfo(QStringLiteral("零位校准记录已保存至 %1").arg(filePath).toStdString());
    }
    return true;
}

bool MainWindow::loadCalibrationSnapshotFromFile(const QString& filePath, bool announce)
{
    if(filePath.isEmpty()){
        return false;
    }

    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly)){
        displayInfo(QStringLiteral("错误：无法读取零位校准记录 %1")
                        .arg(filePath).toStdString(),
                    "error");
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    file.close();
    if(parseError.error != QJsonParseError::NoError || !document.isObject()){
        displayInfo(QStringLiteral("错误：零位校准记录解析失败：%1")
                        .arg(parseError.errorString()).toStdString(),
                    "error");
        return false;
    }

    const QJsonObject root = document.object();
    std::vector<double> motorHome = fromJsonArray(root.value(QStringLiteral("motor_home")).toArray());
    std::vector<double> sensorHome = fromJsonArray(root.value(QStringLiteral("force_sensor_home")).toArray());
    std::vector<double> rotHome = fromJsonArray(root.value(QStringLiteral("end_cur_rot_home")).toArray());
    std::vector<double> devEndHome = fromJsonArray(root.value(QStringLiteral("dev_end_home")).toArray());

    if(motorHome.empty()){
        displayInfo("错误：零位校准记录缺少电机零位数据", "error");
        return false;
    }

    refreshAxisRuntimeCounts();
    if(static_cast<int>(motorHome.size()) < ui->devAxisNum->value()){
        motorHome.resize(ui->devAxisNum->value(), 0.0);
    }
    if(static_cast<int>(sensorHome.size()) < axisForceSensorNum){
        sensorHome.resize(axisForceSensorNum, 0.0);
    }
    if(rotHome.size() < 3){
        rotHome.resize(3, 0.0);
    }

    hardwareInterface.setMotorHome(motorHome);
    hardwareInterface.setForceSensorHome(sensorHome);
    forceSensorCurHome = sensorHome;
    endCurRotHome = rotHome;
    motorPosDisplayZero.clear();

    if(devEndHome.size() >= 6){
        ui->devEndHomeX->setValue(devEndHome[0]);
        ui->devEndHomeY->setValue(devEndHome[1]);
        ui->devEndHomeZ->setValue(devEndHome[2]);
        ui->devEndHomeRx->setValue(devEndHome[3]);
        ui->devEndHomeRy->setValue(devEndHome[4]);
        ui->devEndHomeRz->setValue(devEndHome[5]);
    }

    motiveFitConfirmed = root.value(QStringLiteral("motive_fit_confirmed")).toBool(motiveFitConfirmed);
    const QString cableHomeStateText = root.value(QStringLiteral("cable_home_state")).toString();
    if(cableHomeStateText == QStringLiteral("confirmed")){
        runtimeState.cableHomeState = CableHomeState::Confirmed;
    }
    else if(cableHomeStateText == QStringLiteral("waiting_force_stable")){
        runtimeState.cableHomeState = CableHomeState::WaitingForceStable;
    }
    else{
        runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    }

    ui->mainResetCableForceHomeSwitch->setEnabled(true);
    if(runtimeState.cableHomeState == CableHomeState::Confirmed){
        ui->mainSetCableHome->setEnabled(false);
        ui->mainPosModeSwitch->setEnabled(true);
        optionalButtonSetEnabled(this, "mainGCModeSwitch", true);
    }
    else{
        updateCableHomeConfirmEnabled();
    }

    updateControlWorkerConfig();
    refreshMotorPosDisplay();
    lastCalibrationRecordPath = filePath;
    lastCalibrationRecordSummary = root.value(QStringLiteral("summary")).toString();
    lastCalibrationRecordTime = QDateTime::fromString(root.value(QStringLiteral("saved_at")).toString(), Qt::ISODate);
    calibrationWorkflowStage = runtimeState.cableHomeState == CableHomeState::Confirmed ?
                CalibrationWorkflowStage::Completed :
                CalibrationWorkflowStage::Idle;
    refreshCalibrationUiState();
    refreshRunModeUiState();

    if(announce){
        displayInfo(QStringLiteral("已加载零位校准记录 %1").arg(filePath).toStdString());
    }
    return true;
}

void MainWindow::initializeCalibrationUi()
{
    if(calibrationStatusGroupBox || !ui){
        return;
    }

    calibrationStatusGroupBox = ui->calibrationStatusGroupBox;
    calibrationStatusLabel = ui->calibrationStatusLabel;
    calibrationActionLabel = ui->calibrationActionLabel;
    calibrationStepLabel = ui->calibrationStepLabel;
    calibrationResultLabel = ui->calibrationResultLabel;
    calibrationStartButton = ui->calibrationStartButton;
    calibrationStopButton = ui->calibrationStopButton;
    calibrationConfirmButton = ui->calibrationConfirmButton;
    calibrationSaveButton = ui->calibrationSaveButton;
    calibrationLoadButton = ui->calibrationLoadButton;
    calibrationRestoreButton = ui->calibrationRestoreButton;

    if(!calibrationStatusGroupBox || !calibrationStatusLabel || !calibrationActionLabel
            || !calibrationStepLabel || !calibrationResultLabel
            || !calibrationStartButton || !calibrationStopButton || !calibrationConfirmButton
            || !calibrationSaveButton
            || !calibrationLoadButton || !calibrationRestoreButton){
        return;
    }

    connect(calibrationStartButton, &QPushButton::clicked, this, [this](){
        startZeroCalibrationWorkflow();
    });
    connect(calibrationStopButton, &QPushButton::clicked, this, [this](){
        stopZeroCalibrationWorkflow(true);
    });
    connect(calibrationConfirmButton, &QPushButton::clicked, this, [this](){
        confirmZeroCalibrationWorkflow();
    });

    connect(calibrationSaveButton, &QPushButton::clicked, this, [this](){
        const QString dirPath = calibrationStorageDirPath();
        const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
        const QString latestPath = latestCalibrationSnapshotPath();
        const QString historyPath = QDir(dirPath).filePath(
                    QStringLiteral("zero_calibration_%1.json").arg(stamp));
        const bool saveLatestOk = saveCalibrationSnapshotToFile(latestPath, false);
        const bool saveHistoryOk = saveCalibrationSnapshotToFile(historyPath, false);
        if(saveLatestOk && saveHistoryOk){
            lastCalibrationRecordPath = historyPath;
            lastCalibrationRecordTime = QDateTime::currentDateTime();
            refreshCalibrationUiState();
            displayInfo(QStringLiteral("零位校准记录已保存：最近记录 %1")
                            .arg(historyPath).toStdString());
        }
    });
    connect(calibrationLoadButton, &QPushButton::clicked, this, [this](){
        const QString filePath = QFileDialog::getOpenFileName(
                    this,
                    QStringLiteral("加载零位校准记录"),
                    calibrationStorageDirPath(),
                    QStringLiteral("Zero Calibration (*.json)"));
        if(filePath.isEmpty()){
            return;
        }
        loadCalibrationSnapshotFromFile(filePath, true);
    });
    connect(calibrationRestoreButton, &QPushButton::clicked, this, [this](){
        const QString latestPath = latestCalibrationSnapshotPath();
        if(!QFileInfo::exists(latestPath)){
            displayInfo("警告：当前没有可恢复的最近零位校准记录", "warning");
            return;
        }
        loadCalibrationSnapshotFromFile(latestPath, true);
    });

    refreshCalibrationUiState();
}

bool MainWindow::beginCalibrationPretensionStage(const QString& sourceName)
{
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        displayInfo(QStringLiteral("%1失败：雷赛控制卡未连接").arg(sourceName).toStdString(), "error");
        return false;
    }
    if(!setMotorControllerEnable(true)){
        displayInfo(QStringLiteral("%1失败：电机使能失败").arg(sourceName).toStdString(), "error");
        return false;
    }

    waitMotorPositionStable();
    runtimeState.cableHomeState = CableHomeState::WaitingForceStable;
    ui->mainAllUseFC->setChecked(true);
    allUseFC(true);
    ui->mainFCThread->setChecked(true);
    requestImmediateForceControlUpdate();
    updateCableHomeConfirmEnabled();
    displayInfo(QStringLiteral("%1：已启用完全力控开始绳索预紧与张力均衡")
                    .arg(sourceName).toStdString());
    return true;
}

bool MainWindow::confirmZeroCalibrationWorkflow()
{
    if(calibrationWorkflowStage == CalibrationWorkflowStage::AwaitingPretensionStart){
        displayInfo("错误：当前仅完成机械回零，请先再次点击“开始零位校准”进入预紧阶段", "error");
        return false;
    }

    const bool requiresCalibrationFinalize =
            calibrationWorkflowStage == CalibrationWorkflowStage::AwaitingConfirm ||
            calibrationWorkflowStage == CalibrationWorkflowStage::CollectingSensorZero ||
            calibrationWorkflowStage == CalibrationWorkflowStage::UpdatingSpatialBaseline;
    if(!requiresCalibrationFinalize){
        return setCableHome();
    }

    calibrationWorkflowStage = CalibrationWorkflowStage::CollectingSensorZero;
    refreshCalibrationUiState();
    if(!resetCableForceHome()){
        calibrationWorkflowStage = CalibrationWorkflowStage::AwaitingConfirm;
        refreshCalibrationUiState();
        return false;
    }

    if(ui->devUseCamForActPose->isChecked()){
        calibrationWorkflowStage = CalibrationWorkflowStage::UpdatingSpatialBaseline;
        refreshCalibrationUiState();
        resetMotiveFit();
        if(!motiveFitConfirmed){
            calibrationWorkflowStage = CalibrationWorkflowStage::AwaitingConfirm;
            refreshCalibrationUiState();
            return false;
        }
    }

    calibrationWorkflowStage = CalibrationWorkflowStage::AwaitingConfirm;
    refreshCalibrationUiState();
    return setCableHome();
}

std::vector<double> MainWindow::buildCalibrationDesignHomePose(double zOffsetMm) const
{
    return {
        ui->devEndHomeX->value(),
        ui->devEndHomeY->value(),
        ui->devEndHomeZ->value() + zOffsetMm,
        ui->devEndHomeRx->value(),
        ui->devEndHomeRy->value(),
        ui->devEndHomeRz->value()
    };
}

bool MainWindow::buildCalibrationMechanicalHomingTrajectory(
        TrajectoryPlanner::MultiEndTrajectory& plannedEndTraj,
        QString& errorMessage) const
{
    plannedEndTraj.clear();

    std::vector<double> currentPose;
    if(!currentEstimatedEndPose(currentPose, 1000) || currentPose.size() < 6){
        errorMessage = ui->devUseCamForActPose->isChecked() ?
                    QStringLiteral("机械回零失败：当前未获取到有效动捕位姿，无法规划回零轨迹") :
                    QStringLiteral("机械回零失败：当前未获取到有效正运动学位姿，无法规划回零轨迹");
        return false;
    }

    const std::vector<double> aboveHomePose = buildCalibrationDesignHomePose(kCalibrationHomeLiftMm);
    const std::vector<double> approachHomePose = buildCalibrationDesignHomePose(kCalibrationHomeApproachOffsetMm);
    auto estimateSegmentDuration = [](const std::vector<double>& startPose,
                                      const std::vector<double>& endPose) -> double {
        const double dx = endPose[0] - startPose[0];
        const double dy = endPose[1] - startPose[1];
        const double dz = endPose[2] - startPose[2];
        const double linearDistanceMm = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double drx = endPose[3] - startPose[3];
        const double dry = endPose[4] - startPose[4];
        const double drz = endPose[5] - startPose[5];
        const double angularDistanceRad = std::sqrt(drx * drx + dry * dry + drz * drz);
        const double linearDuration = linearDistanceMm / kCalibrationLinearSpeedMmPerSec;
        const double angularDuration = angularDistanceRad / kCalibrationAngularSpeedRadPerSec;
        return std::max(kCalibrationMinSegmentDurationSec,
                        std::max(linearDuration, angularDuration) + kCalibrationSegmentDurationPaddingSec);
    };

    double stepTime = ui->mainPosModeTargetStepTime->value();
    if(stepTime <= 1e-6){
        stepTime = std::max(0.01, ui->devCtrlCycleMs->value() / 1000.0);
    }

    auto buildStraightSegment = [&](const std::vector<double>& startPose,
                                    const std::vector<double>& endPose) -> TrajectoryPlanner::MultiEndTrajectory {
        TrajectoryPlanner::EndpointRequest request;
        request.type = TrajectoryPlanner::TrajType::Quintic;
        request.q_param = {
            startPose,
            {0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0},
            endPose,
            {0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0},
            estimateSegmentDuration(startPose, endPose),
            stepTime
        };
        return TrajectoryPlanner::buildLineTrajectory({request}, request.type);
    };

    const TrajectoryPlanner::MultiEndTrajectory firstSegment =
            buildStraightSegment(currentPose, aboveHomePose);
    const TrajectoryPlanner::MultiEndTrajectory secondSegment =
            buildStraightSegment(aboveHomePose, approachHomePose);
    if(firstSegment.size() != 1 || secondSegment.size() != 1 ||
            firstSegment[0].size() < 4 || secondSegment[0].size() < 4 ||
            firstSegment[0][0].size() < 6 || secondSegment[0][0].size() < 6 ||
            firstSegment[0][0][0].empty() || secondSegment[0][0][0].empty()){
        errorMessage = QStringLiteral("机械回零失败：两段直线轨迹规划结果无效");
        return false;
    }

    plannedEndTraj.resize(1);
    plannedEndTraj[0].resize(4);
    for(int blockIndex=0; blockIndex<4; ++blockIndex){
        plannedEndTraj[0][blockIndex].resize(6);
        for(int dim=0; dim<6; ++dim){
            plannedEndTraj[0][blockIndex][dim] = firstSegment[0][blockIndex][dim];
        }
    }

    const double timeOffset = plannedEndTraj[0][3][0].empty() ? 0.0 : plannedEndTraj[0][3][0].back();
    for(int blockIndex=0; blockIndex<4; ++blockIndex){
        for(int dim=0; dim<6; ++dim){
            const std::vector<double>& source = secondSegment[0][blockIndex][dim];
            if(source.size() <= 1){
                continue;
            }
            for(std::size_t pointIndex=1; pointIndex<source.size(); ++pointIndex){
                const double value = (blockIndex == 3) ? (source[pointIndex] + timeOffset) : source[pointIndex];
                plannedEndTraj[0][blockIndex][dim].push_back(value);
            }
        }
    }

    if(plannedEndTraj[0][0][0].size() < 2){
        errorMessage = QStringLiteral("机械回零失败：拼接后的轨迹点数不足");
        return false;
    }

    if(!validateTrajectoryWithinWorkspace(plannedEndTraj, errorMessage)){
        errorMessage = QStringLiteral("机械回零失败：%1").arg(errorMessage);
        return false;
    }
    return true;
}

bool MainWindow::buildCalibrationMechanicalHomingPvtCommand(
        const TrajectoryPlanner::MultiEndTrajectory& plannedEndTraj,
        PvtExecutionWorker::PvtCommand& command,
        QString& errorMessage)
{
    command = PvtExecutionWorker::PvtCommand();
    if(plannedEndTraj.size() != 1 ||
            plannedEndTraj[0].size() < 4 ||
            plannedEndTraj[0][0].size() < 6 ||
            plannedEndTraj[0][0][0].size() < 2){
        errorMessage = QStringLiteral("机械回零失败：待下发轨迹无效");
        return false;
    }

    const std::size_t pointCount = plannedEndTraj[0][0][0].size();
    const double stepTimeFallback = std::max(0.01, ui->devCtrlCycleMs->value() / 1000.0);

    std::vector<int> cableMotorIndex;
    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
            if(isModeledMotorAxis(axisIndex) && axisEndVec[axisIndex]->value() == endIndex){
                cableMotorIndex.push_back(axisIndex);
            }
        }
    }
    if(cableMotorIndex.empty()){
        errorMessage = QStringLiteral("机械回零失败：当前没有可参与位控的建模电机轴");
        return false;
    }

    const std::vector<double> absMotorPos = hardwareInterface.getAllMotorPos();
    if(absMotorPos.empty()){
        errorMessage = QStringLiteral("机械回零失败：无法读取当前电机位置");
        return false;
    }

    const double pulleyRadius = buildPulleyRadius();
    std::vector<std::vector<double>> relativeMotorTheta(cableMotorIndex.size(),
                                                        std::vector<double>(pointCount, 0.0));
    std::vector<double> timeStamp(pointCount, 0.0);
    std::vector<double> firstCableLen(cableMotorIndex.size(), 0.0);
    int channelIndex = 0;
    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        if(endIndex >= static_cast<int>(plannedEndTraj.size()) ||
                plannedEndTraj[endIndex].size() < 4 ||
                plannedEndTraj[endIndex][0].size() < 6){
            errorMessage = QStringLiteral("机械回零失败：轨迹末端数量与轴参数配置不一致");
            return false;
        }

        for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
            if(!isModeledMotorAxis(axisIndex) || axisEndVec[axisIndex]->value() != endIndex){
                continue;
            }
            if(channelIndex >= static_cast<int>(cableMotorIndex.size())){
                errorMessage = QStringLiteral("机械回零失败：建模轴通道数量异常");
                return false;
            }

            for(std::size_t pointIndex=0; pointIndex<pointCount; ++pointIndex){
                const double px = plannedEndTraj[endIndex][0][0][pointIndex];
                const double py = plannedEndTraj[endIndex][0][1][pointIndex];
                const double pz = plannedEndTraj[endIndex][0][2][pointIndex];
                const double rx = plannedEndTraj[endIndex][0][3][pointIndex];
                const double ry = plannedEndTraj[endIndex][0][4][pointIndex];
                const double rz = plannedEndTraj[endIndex][0][5][pointIndex];

                const std::vector<double> Rx = {
                    1.0, 0.0, 0.0,
                    0.0, std::cos(rx), -std::sin(rx),
                    0.0, std::sin(rx),  std::cos(rx)
                };
                const std::vector<double> Ry = {
                    std::cos(ry), 0.0, std::sin(ry),
                    0.0, 1.0, 0.0,
                    -std::sin(ry), 0.0, std::cos(ry)
                };
                const std::vector<double> Rz = {
                    std::cos(rz), -std::sin(rz), 0.0,
                    std::sin(rz),  std::cos(rz), 0.0,
                    0.0, 0.0, 1.0
                };
                const std::vector<std::vector<double>> rotationX = {
                    {Rx[0], Rx[1], Rx[2]},
                    {Rx[3], Rx[4], Rx[5]},
                    {Rx[6], Rx[7], Rx[8]}
                };
                const std::vector<std::vector<double>> rotationY = {
                    {Ry[0], Ry[1], Ry[2]},
                    {Ry[3], Ry[4], Ry[5]},
                    {Ry[6], Ry[7], Ry[8]}
                };
                const std::vector<std::vector<double>> rotationZ = {
                    {Rz[0], Rz[1], Rz[2]},
                    {Rz[3], Rz[4], Rz[5]},
                    {Rz[6], Rz[7], Rz[8]}
                };
                const std::vector<std::vector<double>> rotation =
                        MatrixFun::matrix_mul(MatrixFun::matrix_mul(rotationZ, rotationY), rotationX);
                const std::vector<std::vector<double>> rotatedContactPoint =
                        MatrixFun::matrix_mul(rotation,
                                              {{axisCableEndPosXVec[axisIndex]->value()},
                                               {axisCableEndPosYVec[axisIndex]->value()},
                                               {axisCableEndPosZVec[axisIndex]->value()}});

                const std::vector<double> globalContactPoint = {
                    px + rotatedContactPoint[0][0],
                    py + rotatedContactPoint[1][0],
                    pz + rotatedContactPoint[2][0]
                };
                const std::vector<double> anchorPoint = {
                    axisCableStartPosXVec[axisIndex]->value(),
                    axisCableStartPosYVec[axisIndex]->value(),
                    axisCableStartPosZVec[axisIndex]->value()
                };
                const double cableLen =
                        MatrixFun::cableLengthCalculate(globalContactPoint, anchorPoint, pulleyRadius).idealLength;
                if(pointIndex == 0){
                    firstCableLen[channelIndex] = cableLen;
                }

                relativeMotorTheta[channelIndex][pointIndex] =
                        (firstCableLen[channelIndex] - cableLen) * motorCableAngleScale(axisIndex);
                if(channelIndex == 0){
                    if(plannedEndTraj[endIndex].size() > 3 &&
                            !plannedEndTraj[endIndex][3].empty() &&
                            !plannedEndTraj[endIndex][3][0].empty() &&
                            pointIndex < plannedEndTraj[endIndex][3][0].size()){
                        timeStamp[pointIndex] = plannedEndTraj[endIndex][3][0][pointIndex];
                    }
                    else{
                        timeStamp[pointIndex] = pointIndex * stepTimeFallback;
                    }
                }
            }
            ++channelIndex;
        }
    }

    double feedbackUnitScale = 1.0;
    if(ui->devMotorFeedbackIsRd->isChecked()){
        feedbackUnitScale = 1.0 / (2.0 * M_PI);
    }
    else if(ui->devMotorFeedbackIsTheta->isChecked()){
        feedbackUnitScale = 180.0 / M_PI;
    }

    command.motorIndex = cableMotorIndex;
    command.timeStamp = timeStamp;
    command.motorPosTraj.assign(pointCount, std::vector<double>(cableMotorIndex.size(), 0.0));
    command.motorVelTraj.assign(pointCount, std::vector<double>(cableMotorIndex.size(), 0.0));
    for(std::size_t pointIndex=0; pointIndex<pointCount; ++pointIndex){
        for(std::size_t motorColumn=0; motorColumn<cableMotorIndex.size(); ++motorColumn){
            const int axisIndex = cableMotorIndex[motorColumn];
            if(axisIndex < 0 || axisIndex >= static_cast<int>(absMotorPos.size())){
                errorMessage = QStringLiteral("机械回零失败：读取到的电机位置数量与轴配置不一致");
                return false;
            }

            const double hardwareDirection = motorHardwareDirectionSign(axisIndex);

            command.motorPosTraj[pointIndex][motorColumn] =
                    absMotorPos[axisIndex] +
                    hardwareDirection * feedbackUnitScale * relativeMotorTheta[motorColumn][pointIndex];
            if(pointIndex == 0){
                command.motorVelTraj[pointIndex][motorColumn] = 0.0;
            }
            else{
                const double dt = std::max(1e-6, timeStamp[pointIndex] - timeStamp[pointIndex - 1]);
                command.motorVelTraj[pointIndex][motorColumn] =
                        hardwareDirection * feedbackUnitScale *
                        (relativeMotorTheta[motorColumn][pointIndex] - relativeMotorTheta[motorColumn][pointIndex - 1]) / dt;
            }

            if(command.motorPosTraj[pointIndex][motorColumn] > axisMotorMaxVec[axisIndex]->value()){
                errorMessage = QStringLiteral("机械回零轨迹不会下发：电机%1在轨迹点%2处的目标位置%3会导致电机超出软件位置上限%4")
                        .arg(axisIndex)
                        .arg(static_cast<int>(pointIndex))
                        .arg(command.motorPosTraj[pointIndex][motorColumn], 0, 'f', 6)
                        .arg(axisMotorMaxVec[axisIndex]->value(), 0, 'f', 6);
                return false;
            }
            if(command.motorPosTraj[pointIndex][motorColumn] < axisMotorMinVec[axisIndex]->value()){
                errorMessage = QStringLiteral("机械回零轨迹不会下发：电机%1在轨迹点%2处的目标位置%3会导致电机超出软件位置下限%4")
                        .arg(axisIndex)
                        .arg(static_cast<int>(pointIndex))
                        .arg(command.motorPosTraj[pointIndex][motorColumn], 0, 'f', 6)
                        .arg(axisMotorMinVec[axisIndex]->value(), 0, 'f', 6);
                return false;
            }
            if(std::abs(command.motorVelTraj[pointIndex][motorColumn]) > axisMotorVelMaxVec[axisIndex]->value()){
                errorMessage = QStringLiteral("机械回零失败：电机%1在轨迹点%2处的目标速度%3超过安全上限%4")
                        .arg(axisIndex)
                        .arg(static_cast<int>(pointIndex))
                        .arg(command.motorVelTraj[pointIndex][motorColumn], 0, 'f', 6)
                        .arg(axisMotorVelMaxVec[axisIndex]->value(), 0, 'f', 6);
                return false;
            }
        }
    }
    return true;
}

bool MainWindow::executeCalibrationMechanicalHoming(const QString& sourceName)
{
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        displayInfo(QStringLiteral("%1失败：雷赛控制卡未连接").arg(sourceName).toStdString(), "error");
        return false;
    }
    if(!currentRobotState().systemRunning){
        displayInfo(QStringLiteral("%1失败：请先完成启动上电后再执行机械回零").arg(sourceName).toStdString(), "error");
        return false;
    }
    if(!pvtExecutionWorker){
        displayInfo(QStringLiteral("%1失败：PVT 执行线程未初始化").arg(sourceName).toStdString(), "error");
        return false;
    }

    TrajectoryPlanner::MultiEndTrajectory plannedEndTraj;
    QString errorMessage;
    if(!buildCalibrationMechanicalHomingTrajectory(plannedEndTraj, errorMessage)){
        displayInfo(errorMessage.toStdString(), "error");
        return false;
    }

    PvtExecutionWorker::PvtCommand pvtCommand;
    if(!buildCalibrationMechanicalHomingPvtCommand(plannedEndTraj, pvtCommand, errorMessage)){
        displayInfo(errorMessage.toStdString(), "error");
        return false;
    }

    displayInfo(QStringLiteral("%1：已规划两段直线机械回零轨迹，先运动至零点位姿正上方500mm，再竖直下移到零点位姿上方20mm")
                    .arg(sourceName).toStdString());
    bool pvtStarted = false;
    bool pvtAvailable = false;
    invokeOnObjectThreadWithHeartbeat(pvtExecutionWorker, [&](){
        pvtStarted = pvtExecutionWorker->startPvtTrajectory(pvtCommand);
        pvtAvailable = hardwareInterface.hasPvtTrajectory();
    }, [this](){
        refreshSafetyMonitorHeartbeat();
    });
    if(!pvtStarted || !pvtAvailable){
        runtimeState.pvtCommandActive = false;
        displayInfo(QStringLiteral("%1失败：机械回零轨迹下发失败").arg(sourceName).toStdString(), "error");
        return false;
    }
    runtimeState.pvtCommandActive = true;

    const int timeoutMs = std::max(5000,
                                   static_cast<int>((pvtCommand.timeStamp.empty() ? 0.0 : pvtCommand.timeStamp.back()) * 1000.0) + 5000);
    const bool done = waitMotorAxesDone(timeoutMs, 20);
    hardwareInterface.clearPvtTrajectoryState();
    runtimeState.pvtCommandActive = false;
    if(!done){
        displayInfo(QStringLiteral("%1失败：机械回零轨迹未在等待时间内执行完成").arg(sourceName).toStdString(), "error");
        return false;
    }

    if(ui->Poscontinuetraj->isChecked()){
        std::vector<double> finalPose;
        if(extractTrajectoryEndpointPose(plannedEndTraj, finalPose) && finalPose.size() >= 6){
            lastPlannedPoseTrajectoryEndPose = finalPose;
            applyTrajectoryStartPoseToUi(finalPose);
        }
        else{
            displayInfo(QStringLiteral("%1：连续轨迹模式下未能自动提取机械回零终点位姿，请手动确认下一段起点")
                        .arg(sourceName).toStdString(), "warning");
        }

        if(!refreshForwardKinematicsPoseFromCurrentMotorPosition() && !ui->devUseCamForActPose->isChecked()){
            displayInfo(QStringLiteral("%1：机械回零已完成，但未能立即刷新正运动学位姿，请检查电机位置反馈和预紧零点状态")
                        .arg(sourceName).toStdString(), "warning");
        }
    }

    displayInfo(QStringLiteral("%1：两段直线机械回零轨迹执行完成，末端已到达零点位姿上方20mm")
                .arg(sourceName).toStdString());
    return true;
}

void MainWindow::startZeroCalibrationWorkflow()
{
    refreshAxisRuntimeCounts();
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        displayInfo("错误：零位校准启动失败，请先连接并上电雷赛控制卡", "error");
        stopZeroCalibrationWorkflow(false);
        return;
    }

    const RobotStateSnapshot robotState = currentRobotState(false);
    if(robotState.positionMotionState != PositionMotionState::Idle ||
            robotState.forceMotionState != ForceMotionState::Idle){
        displayInfo("错误：当前存在运行中的位控或力控任务，请先停止后再开始零位校准", "error");
        stopZeroCalibrationWorkflow(false);
        return;
    }

    if(calibrationWorkflowStage == CalibrationWorkflowStage::AwaitingPretensionStart){
        calibrationWorkflowStage = CalibrationWorkflowStage::PretensionBalancing;
        refreshCalibrationUiState();
        if(!beginCalibrationPretensionStage(QStringLiteral("零位校准流程"))){
            stopZeroCalibrationWorkflow(false);
            return;
        }
        calibrationWorkflowStage = CalibrationWorkflowStage::AwaitingConfirm;
        refreshCalibrationUiState();
        displayInfo("零位校准流程：预紧已启动，请等待张力稳定后点击“确认并同步”完成零位校准");
        return;
    }

    calibrationWorkflowStage = CalibrationWorkflowStage::MechanicalHoming;
    refreshCalibrationUiState();
    displayInfo("零位校准流程启动：开始执行两段直线机械回零");
    if(!motor2Home()){
        stopZeroCalibrationWorkflow(false);
        return;
    }
    setMotorControllerEnable(false);
    calibrationWorkflowStage = CalibrationWorkflowStage::AwaitingPretensionStart;
    refreshCalibrationUiState();
    displayInfo("零位校准流程：机械回零已完成，末端停在零点位姿上方20mm且电机已自动失能；请确认位置后再次点击“开始零位校准”进入预紧");
}

void MainWindow::stopZeroCalibrationWorkflow(bool announce)
{
    calibrationWorkflowStage = CalibrationWorkflowStage::Stopped;
    runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    updateCableHomeConfirmEnabled();
    ui->mainFCThread->setChecked(false);
    ui->mainAllUseFC->setChecked(false);
    allUseFC(false);
    setForceControlSelectionEnabled(true);
    if(ui->devUseLS->isChecked() && hardwareInterface.isLSConnected()){
        for(int i=0; i<ui->devAxisNum->value(); ++i){
            if(isMotorAxis(i)){
                hardwareInterface.motorStop(i);
            }
        }
    }
    refreshCalibrationUiState();
    refreshRunModeUiState();
    if(announce){
        displayInfo("零位校准流程已停止，新的零位基准尚未确认同步", "warning");
    }
}

void MainWindow::initializeParameterConfigUi()
{
    if(!ui || !ui->devReadConfigBtn || ui->devReadConfigBtn->property("paramUiInitialized").toBool()){
        return;
    }
    if(!ui->devWriteConfigBtn || !ui->devApplyTemplateBtn || !ui->devTemplatePresetCombo){
        return;
    }
    ui->devReadConfigBtn->setProperty("paramUiInitialized", true);

    if(ui->devTemplatePresetCombo){
        ui->devTemplatePresetCombo->setCurrentText(kParameterTemplateG3);
    }
    if(ui->devLowerMachineProtocolCombo){
        ui->devLowerMachineProtocolCombo->setCurrentText(QStringLiteral("UDP"));
        ui->devLowerMachineProtocolCombo->setToolTip(
                    QStringLiteral("当前实时通讯链路固定使用 UDP；下方需分别配置本机命令监听端口和状态回传目标地址。"));
    }
    if(ui->devLowerMachineListenPort){
        ui->devLowerMachineListenPort->setToolTip(
                    QStringLiteral("程序本机用于接收外部 UDP 命令的监听端口。"));
        if(ui->devLowerMachineListenPort->value() <= 0){
            ui->devLowerMachineListenPort->setValue(kUdpRealtimeLocalPort);
        }
    }
    if(ui->devSensorSampleFreqHz){
        ui->devSensorSampleFreqHz->setToolTip(
                    QStringLiteral("张力传感器采集频率，范围 10~2000 Hz；Trace读取时 1000 Hz 对应1000us，2000 Hz 对应500us。"));
        ui->devSensorSampleFreqHz->setMaximum(2000);
        if(ui->devSensorSampleFreqHz->value() < 10 || ui->devSensorSampleFreqHz->value() > 2000){
            ui->devSensorSampleFreqHz->setValue(1000);
        }
    }
    if(ui->devForceSensorLowPassCutoffHz){
        ui->devForceSensorLowPassCutoffHz->setToolTip(
                    QStringLiteral("张力传感器一阶低通滤波截止频率。0 Hz 表示关闭滤波；超过采集频率 45% 时运行时自动按上限处理。"));
        if(ui->devForceSensorLowPassCutoffHz->value() < 0.0 || ui->devForceSensorLowPassCutoffHz->value() > 500.0){
            ui->devForceSensorLowPassCutoffHz->setValue(0.0);
        }
    }
    if(ui->devLowerMachineIpCombo){
        ui->devLowerMachineIpCombo->setToolTip(
                    QStringLiteral("实时模式下，软件会把 UDP 状态包回传到这里设置的目标 IP。"));
        if(ui->devLowerMachineIpCombo->currentText().trimmed().isEmpty()){
            ui->devLowerMachineIpCombo->setCurrentText(kUdpRealtimeRemoteIp);
        }
    }
    if(ui->devLowerMachinePort){
        ui->devLowerMachinePort->setToolTip(
                    QStringLiteral("实时模式下，软件会把 UDP 状态包回传到这里设置的目标端口。"));
        if(ui->devLowerMachinePort->value() <= 0){
            ui->devLowerMachinePort->setValue(kUdpRealtimeDefaultRemotePort);
        }
    }
    syncUdpTargetPortWidgets(udpRealtimeTargetPort());

    connect(ui->devApplyTemplateBtn, &IntBtn::sendInt, this, [this](int){
        applyParameterTemplate(ui->devTemplatePresetCombo ?
                                   ui->devTemplatePresetCombo->currentText() :
                                   QString());
    });
    connect(ui->devReadConfigBtn, &IntBtn::sendInt, this, [this](int){
        const QString filePath = QFileDialog::getOpenFileName(
                    this,
                    QStringLiteral("读取参数配置文件"),
                    parameterConfigStorageDirPath(),
                    QStringLiteral("Parameter Config (*.json)"));
        if(filePath.isEmpty()){
            return;
        }
        QString errorMessage;
        if(!loadParameterConfigFromFile(filePath, &errorMessage)){
            displayInfo(QStringLiteral("错误：读取参数配置失败，%1").arg(errorMessage).toStdString(), "error");
            return;
        }
        displayInfo(QStringLiteral("已读取参数配置文件 %1，界面参数已更新，点击“更新并应用参数”使底层配置生效")
                        .arg(filePath).toStdString());
    });
    connect(ui->devWriteConfigBtn, &IntBtn::sendInt, this, [this](int){
        const QString defaultName = QStringLiteral("parameter_config_%1.json")
                                        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
        const QString filePath = QFileDialog::getSaveFileName(
                    this,
                    QStringLiteral("导出参数配置文件"),
                    QDir(parameterConfigStorageDirPath()).filePath(defaultName),
                    QStringLiteral("Parameter Config (*.json)"));
        if(filePath.isEmpty()){
            return;
        }
        QString errorMessage;
        if(!saveParameterConfigToFile(filePath, &errorMessage)){
            displayInfo(QStringLiteral("错误：导出参数配置失败，%1").arg(errorMessage).toStdString(), "error");
            return;
        }
        displayInfo(QStringLiteral("参数配置已导出至 %1").arg(filePath).toStdString());
    });
    if(ui->devLowerMachineProtocolCombo){
        connect(ui->devLowerMachineProtocolCombo,
                &QComboBox::currentTextChanged,
                this,
                [this](const QString& text){
                    if(text != QStringLiteral("UDP")){
                        const QSignalBlocker blocker(ui->devLowerMachineProtocolCombo);
                        ui->devLowerMachineProtocolCombo->setCurrentText(QStringLiteral("UDP"));
                    }
                    applyUdpRuntimeConfigChange();
                });
    }
    if(ui->devLowerMachineListenPort){
        connect(ui->devLowerMachineListenPort,
                QOverload<int>::of(&QSpinBox::valueChanged),
                this,
                [this](int){
                    applyUdpRuntimeConfigChange();
                });
    }
    if(ui->devSensorSampleFreqHz){
        connect(ui->devSensorSampleFreqHz,
                QOverload<int>::of(&QSpinBox::valueChanged),
                this,
                [this](int){
                    updateControlWorkerConfig();
                    refreshRuntimeDiagnosticsUi();
                });
    }
    if(ui->devForceSensorLowPassCutoffHz){
        connect(ui->devForceSensorLowPassCutoffHz,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [this](double){
                    updateControlWorkerConfig();
                    refreshRuntimeDiagnosticsUi();
                });
    }
    if(ui->devLowerMachineIpCombo){
        connect(ui->devLowerMachineIpCombo,
                &QComboBox::currentTextChanged,
                this,
                [this](const QString&){
                    applyUdpRuntimeConfigChange();
                });
    }
    if(ui->devLowerMachinePort){
        connect(ui->devLowerMachinePort,
                QOverload<int>::of(&QSpinBox::valueChanged),
                this,
                [this](int value){
                    syncUdpTargetPortWidgets(value);
                    applyUdpRuntimeConfigChange();
                });
    }
    if(ui->devUpdateZeroPoseFromMotiveBtn){
        disconnect(ui->devUpdateZeroPoseFromMotiveBtn,
                   &QPushButton::clicked,
                   this,
                   &MainWindow::resetMotiveFit);
        connect(ui->devUpdateZeroPoseFromMotiveBtn,
                &QPushButton::clicked,
                this,
                &MainWindow::resetMotiveFit);
    }
    if(ui->devSaveZeroPoseBtn){
        connect(ui->devSaveZeroPoseBtn, &QPushButton::clicked, this, [this](){
            const QString defaultName = QStringLiteral("zero_pose_%1.json")
                                            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
            const QString filePath = QFileDialog::getSaveFileName(
                        this,
                        QStringLiteral("保存零点位姿文件"),
                        QDir(parameterConfigStorageDirPath()).filePath(defaultName),
                        QStringLiteral("Zero Pose (*.json)"));
            if(filePath.isEmpty()){
                return;
            }
            QString errorMessage;
            if(!saveZeroPoseConfigToFile(filePath, &errorMessage)){
                displayInfo(QStringLiteral("错误：保存零点位姿失败，%1").arg(errorMessage).toStdString(), "error");
                return;
            }
            displayInfo(QStringLiteral("零点位姿已保存至 %1；后续可通过“读取参数配置文件”载入，并点击“更新并应用参数”使其生效")
                            .arg(filePath).toStdString());
        });
    }
    if(ui->infoHistoryBtn){
        connect(ui->infoHistoryBtn, &QPushButton::clicked, this, &MainWindow::showMessageHistoryDialog);
    }
    if(ui->infoLogDirBtn){
        connect(ui->infoLogDirBtn, &QPushButton::clicked, this, [this](){
            openUiEventLogDirectory(true);
        });
    }
    if(ui->infoFaultLogBtn){
        connect(ui->infoFaultLogBtn, &QPushButton::clicked, this, [this](){
            openUiEventLogFile(true);
        });
    }
}

void MainWindow::initializeForceTraceTestUi()
{
    QTableWidget* table = ui->forceTraceObjectTable;
    table->setColumnCount(6);
    table->setHorizontalHeaderLabels(QStringList()
                                     << QStringLiteral("从站地址")
                                     << QStringLiteral("变量名")
                                     << QStringLiteral("Index/地址")
                                     << QStringLiteral("SubIndex")
                                     << QStringLiteral("数据字节")
                                     << QStringLiteral("Trace dataType"));
    table->horizontalHeader()->setStretchLastSection(true);
    table->verticalHeader()->setVisible(false);
    table->setAlternatingRowColors(true);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);

    populateForceTraceDefaultObjects();

    connect(ui->forceTraceLoadDefaultButton, &QPushButton::clicked,
            this, &MainWindow::populateForceTraceDefaultObjects);
    connect(ui->forceTraceAddRowButton, &QPushButton::clicked, this, [this](){
        QTableWidget* objectTable = ui->forceTraceObjectTable;
        const int row = objectTable->rowCount();
        objectTable->insertRow(row);
        const QStringList defaults = QStringList()
                << QString::number(ui->forceTracePdoSlaveAddrSpin->value())
                << QStringLiteral("W")
                << QStringLiteral("0x6000")
                << QStringLiteral("0")
                << QStringLiteral("0")
                << QStringLiteral("19");
        for(int column = 0; column < defaults.size(); ++column){
            objectTable->setItem(row, column, new QTableWidgetItem(defaults.at(column)));
        }
    });
    connect(ui->forceTraceRemoveRowButton, &QPushButton::clicked, this, [this](){
        QTableWidget* objectTable = ui->forceTraceObjectTable;
        const QModelIndexList selection = objectTable->selectionModel()->selectedRows();
        if(selection.isEmpty()){
            return;
        }
        QVector<int> rows;
        rows.reserve(selection.size());
        for(const QModelIndex& index : selection){
            rows.append(index.row());
        }
        std::sort(rows.begin(), rows.end(), std::greater<int>());
        for(int row : rows){
            objectTable->removeRow(row);
        }
    });
    connect(ui->forceTraceClearLogButton, &QPushButton::clicked,
            ui->forceTraceLogEdit, &QPlainTextEdit::clear);
    connect(ui->forceTraceRunPdoButton, &QPushButton::clicked,
            this, &MainWindow::runPdoTraceProbeFromUi);
    connect(ui->forceTraceRunTraceButton, &QPushButton::clicked,
            this, &MainWindow::runTraceProbeFromUi);
}

void MainWindow::populateForceTraceDefaultObjects()
{
    struct DefaultTraceObject {
        int slave = 0;
        const char* name = "";
        int index = 0x6000;
        int subIndex = 0;
        int dataBytes = 0;
        int dataType = 19;
    };

    const DefaultTraceObject defaults[] = {
        {1004, "W1-S1", 0x6000, 1, 0, 19},
        {1004, "W2-S2", 0x6000, 2, 0, 19},
        {1004, "W3-S3", 0x6000, 3, 0, 19},
        {1004, "W4-S4", 0x6000, 4, 0, 19},
        {1011, "W5-S5", 0x6000, 1, 0, 19},
        {1011, "W6-S6", 0x6000, 2, 0, 19},
        {1011, "W7-S7", 0x6000, 3, 0, 19},
        {1011, "W8-S8", 0x6000, 4, 0, 19}
    };

    QTableWidget* table = ui->forceTraceObjectTable;
    table->setRowCount(0);
    for(const DefaultTraceObject& object : defaults){
        const int row = table->rowCount();
        table->insertRow(row);
        table->setItem(row, 0, new QTableWidgetItem(QString::number(object.slave)));
        table->setItem(row, 1, new QTableWidgetItem(QString::fromLatin1(object.name)));
        table->setItem(row, 2, new QTableWidgetItem(QStringLiteral("0x%1").arg(object.index, 0, 16)));
        table->setItem(row, 3, new QTableWidgetItem(QString::number(object.subIndex)));
        table->setItem(row, 4, new QTableWidgetItem(QString::number(object.dataBytes)));
        table->setItem(row, 5, new QTableWidgetItem(QString::number(object.dataType)));
    }
    table->resizeColumnsToContents();
}

void MainWindow::appendForceTraceTestLog(const QString& text)
{
    ui->forceTraceLogEdit->appendPlainText(
                QStringLiteral("[%1] %2")
                .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")),
                     text));
}

void MainWindow::runPdoTraceProbeFromUi()
{
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        appendForceTraceTestLog(QStringLiteral("PDO Trace测试取消：雷赛控制卡未连接。"));
        displayInfo("PDO Trace测试失败：雷赛控制卡未连接", "error");
        return;
    }

    HardwareInterface::PdoTraceProbeConfig config;
    config.channel = static_cast<WORD>(ui->forceTracePdoChannelSpin->value());
    config.slaveAddress = static_cast<WORD>(ui->forceTracePdoSlaveAddrSpin->value());
    config.traceLength = static_cast<DWORD>(ui->forceTracePdoTraceLengthSpin->value());
    config.readStartAddress = 0;
    config.readLengthBytes = static_cast<DWORD>(ui->forceTraceReadLengthSpin->value());
    config.maxReadLengthBytes = static_cast<DWORD>(ui->forceTraceMaxReadLengthSpin->value());
    config.waitTimeoutMs = ui->forceTraceTimeoutSpin->value();
    config.pollIntervalMs = ui->forceTracePollIntervalSpin->value();

    QTableWidget* table = ui->forceTraceObjectTable;
    for(int row = 0; row < table->rowCount(); ++row){
        bool okSlave = false;
        bool okIndex = false;
        bool okSubIndex = false;
        const int slave = table->item(row, 0) ? table->item(row, 0)->text().toInt(&okSlave, 0) : 0;
        const int index = table->item(row, 2) ? table->item(row, 2)->text().toInt(&okIndex, 0) : 0;
        const int subIndex = table->item(row, 3) ? table->item(row, 3)->text().toInt(&okSubIndex, 0) : 0;
        if(!okSlave || !okIndex || !okSubIndex){
            appendForceTraceTestLog(QStringLiteral("PDO Trace跳过第%1行：数值格式无效。").arg(row + 1));
            continue;
        }
        if(slave != config.slaveAddress){
            continue;
        }
        if(index < 0 || index > 0xFFFF || subIndex < 0 || subIndex > 0xFFFF){
            appendForceTraceTestLog(QStringLiteral("PDO Trace跳过第%1行：Index/SubIndex超出WORD范围。").arg(row + 1));
            continue;
        }
        config.objects.push_back({
            static_cast<WORD>(index),
            static_cast<WORD>(subIndex)
        });
    }

    if(config.objects.empty()){
        appendForceTraceTestLog(QStringLiteral("PDO Trace测试取消：表格中没有从站%1的对象。")
                                .arg(config.slaveAddress));
        return;
    }

    appendForceTraceTestLog(QStringLiteral("启动PDO Trace：channel=%1 slave=%2 objects=%3 traceLen=%4")
                            .arg(config.channel)
                            .arg(config.slaveAddress)
                            .arg(static_cast<int>(config.objects.size()))
                            .arg(config.traceLength));
    const HardwareInterface::PdoTraceProbeResult result =
            hardwareInterface.runPdoTraceForceSensorProbe(config);

    appendForceTraceTestLog(QStringLiteral("PDO Trace结果：success=%1 timeout=%2 start=%3 getNum=%4 read=%5 stop=%6 polls=%7 dataNum=%8 packetBytes=%9 readBytes=%10 state=%11")
                            .arg(result.success)
                            .arg(result.timedOut)
                            .arg(result.startResult)
                            .arg(result.getNumResult)
                            .arg(result.readResult)
                            .arg(result.stopResult)
                            .arg(result.pollCount)
                            .arg(result.dataNum)
                            .arg(result.sizeOfEachPacket)
                            .arg(result.actualReadLength)
                            .arg(result.traceState));
    for(const QString& message : result.messages){
        appendForceTraceTestLog(QStringLiteral("PDO Trace信息：%1").arg(message));
    }
    if(!result.data.isEmpty()){
        appendForceTraceTestLog(QStringLiteral("PDO Trace前64字节：%1")
                                .arg(QString::fromLatin1(result.data.left(64).toHex(' '))));
    }
}

void MainWindow::runTraceProbeFromUi()
{
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        appendForceTraceTestLog(QStringLiteral("通用Trace测试取消：雷赛控制卡未连接。"));
        displayInfo("通用Trace测试失败：雷赛控制卡未连接", "error");
        return;
    }

    HardwareInterface::TraceProbeConfig config;
    config.configureSource = ui->forceTraceSetSourceCheckBox->isChecked();
    config.source = static_cast<WORD>(ui->forceTraceSourceSpin->value());
    config.traceCycle = static_cast<short>(ui->forceTraceCycleSpin->value());
    config.lostHandle = static_cast<short>(ui->forceTraceLostHandleSpin->value());
    config.traceType = static_cast<short>(ui->forceTraceTypeSpin->value());
    config.triggerObjectIndex = static_cast<short>(ui->forceTraceTriggerObjectSpin->value());
    config.triggerType = static_cast<short>(ui->forceTraceTriggerTypeSpin->value());
    config.mask = ui->forceTraceMaskSpin->value();
    config.condition = ui->forceTraceConditionSpin->value();
    config.bufferSizeBytes = ui->forceTraceBufferSizeSpin->value();
    config.maxBufferSizeBytes = ui->forceTraceMaxReadLengthSpin->value();
    config.waitTimeoutMs = ui->forceTraceTimeoutSpin->value();
    config.pollIntervalMs = ui->forceTracePollIntervalSpin->value();

    QTableWidget* table = ui->forceTraceObjectTable;
    for(int row = 0; row < table->rowCount(); ++row){
        bool okSlave = false;
        bool okIndex = false;
        bool okSubIndex = false;
        bool okDataBytes = false;
        bool okDataType = false;
        const int slave = table->item(row, 0) ? table->item(row, 0)->text().toInt(&okSlave, 0) : 0;
        const int index = table->item(row, 2) ? table->item(row, 2)->text().toInt(&okIndex, 0) : 0;
        const int subIndex = table->item(row, 3) ? table->item(row, 3)->text().toInt(&okSubIndex, 0) : 0;
        const int dataBytes = table->item(row, 4) ? table->item(row, 4)->text().toInt(&okDataBytes, 0) : 0;
        const int dataType = table->item(row, 5) ? table->item(row, 5)->text().toInt(&okDataType, 0) : 0;
        if(!okSlave || !okIndex || !okSubIndex || !okDataBytes || !okDataType){
            appendForceTraceTestLog(QStringLiteral("通用Trace跳过第%1行：数值格式无效。").arg(row + 1));
            continue;
        }
        if(dataBytes < 0 || dataBytes > 8){
            appendForceTraceTestLog(QStringLiteral("通用Trace跳过第%1行：数据字节数无效。").arg(row + 1));
            continue;
        }
        config.objects.push_back({
            static_cast<short>(dataType),
            index,
            subIndex,
            static_cast<short>(slave),
            static_cast<short>(dataBytes)
        });
    }

    if(config.objects.empty()){
        appendForceTraceTestLog(QStringLiteral("通用Trace测试取消：对象表为空或全部无效。"));
        return;
    }

    appendForceTraceTestLog(QStringLiteral("启动通用Trace：objects=%1 cycle=%2 buffer=%3")
                            .arg(static_cast<int>(config.objects.size()))
                            .arg(config.traceCycle)
                            .arg(config.bufferSizeBytes));
    const HardwareInterface::TraceProbeResult result =
            hardwareInterface.runTraceForceSensorProbe(config);

    appendForceTraceTestLog(QStringLiteral("通用Trace结果：success=%1 timeout=%2 setConfig=%3 addObject=%4 start=%5 getState=%6 getData=%7 stop=%8 polls=%9 valid=%10 free=%11 objectBytes=%12 objectNum=%13 readBytes=%14 flags=%15/%16/%17")
                            .arg(result.success)
                            .arg(result.timedOut)
                            .arg(result.setConfigResult)
                            .arg(result.addConfigObjectResult)
                            .arg(result.dataStartResult)
                            .arg(result.getStateResult)
                            .arg(result.getDataResult)
                            .arg(result.dataStopResult)
                            .arg(result.pollCount)
                            .arg(result.validNum)
                            .arg(result.freeNum)
                            .arg(result.objectTotalBytes)
                            .arg(result.objectTotalNum)
                            .arg(result.actualReadLength)
                            .arg(result.startFlag)
                            .arg(result.triggeredFlag)
                            .arg(result.lostFlag));
    for(const QString& message : result.messages){
        appendForceTraceTestLog(QStringLiteral("通用Trace信息：%1").arg(message));
    }
    if(!result.data.isEmpty()){
        appendForceTraceTestLog(QStringLiteral("通用Trace前64字节：%1")
                                .arg(QString::fromLatin1(result.data.left(64).toHex(' '))));
    }
}

QString MainWindow::parameterConfigStorageDirPath() const
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config_profiles"));
}

QString MainWindow::uiEventLogDirPath() const
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("outputmsg"));
}

QString MainWindow::uiEventLogFilePath() const
{
    return QDir(uiEventLogDirPath()).filePath(QStringLiteral("ui_fault_events.log"));
}

QString MainWindow::structuredFaultLogFilePath() const
{
    return QDir(uiEventLogDirPath()).filePath(QStringLiteral("fault_log.jsonl"));
}

QString MainWindow::softwareFaultGuardLogFilePath() const
{
    return QDir(uiEventLogDirPath()).filePath(QStringLiteral("software_fault_guard.jsonl"));
}

QString MainWindow::runtimeDiagnosticsReportFilePath() const
{
    return QDir(uiEventLogDirPath()).filePath(QStringLiteral("runtime_diagnostics_report.json"));
}

QString MainWindow::poseSimulationCableLengthFilePath() const
{
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    return QDir(uiEventLogDirPath()).filePath(
                QStringLiteral("pose_simulation_cable_lengths_%1.txt").arg(timestamp));
}

void MainWindow::installSoftwareFaultGuards()
{
    const QString logPath = softwareFaultGuardLogFilePath();
    QDir().mkpath(QFileInfo(logPath).absolutePath());
    gSoftwareFaultGuardLogPath = QDir::toNativeSeparators(logPath).toStdWString();
    gSoftwareFaultGuardHardware.store(&hardwareInterface);
    gSoftwareFaultGuardHandling.store(false);

    if(gSoftwareFaultGuardInstalled.exchange(true)){
        return;
    }

    SetUnhandledExceptionFilter(softwareFaultGuardUnhandledExceptionFilter);
    std::set_terminate(softwareFaultGuardTerminateHandler);
    std::signal(SIGABRT, softwareFaultGuardSignalHandler);
    std::signal(SIGFPE, softwareFaultGuardSignalHandler);
    std::signal(SIGILL, softwareFaultGuardSignalHandler);
    std::signal(SIGSEGV, softwareFaultGuardSignalHandler);
}

void MainWindow::uninstallSoftwareFaultGuards()
{
    gSoftwareFaultGuardHardware.store(nullptr);
}

QJsonObject MainWindow::buildParameterConfigSnapshot() const
{
    QJsonObject widgetValues;
    const QList<QWidget*> widgets = findChildren<QWidget*>();
    for(QWidget* widget : widgets){
        if(!shouldPersistParameterWidget(widget)){
            continue;
        }

        const QString name = widget->objectName();
        if(auto spinBox = qobject_cast<QSpinBox*>(widget)){
            widgetValues.insert(name, spinBox->value());
        }
        else if(auto doubleSpinBox = qobject_cast<QDoubleSpinBox*>(widget)){
            widgetValues.insert(name, doubleSpinBox->value());
        }
        else if(auto checkBox = qobject_cast<QCheckBox*>(widget)){
            widgetValues.insert(name, checkBox->isChecked());
        }
        else if(auto radioButton = qobject_cast<QRadioButton*>(widget)){
            widgetValues.insert(name, radioButton->isChecked());
        }
        else if(auto comboBox = qobject_cast<QComboBox*>(widget)){
            widgetValues.insert(name, comboBox->currentText());
        }
    }

    QJsonObject root;
    root.insert(QStringLiteral("schema_version"), 1);
    root.insert(QStringLiteral("saved_at"), QDateTime::currentDateTime().toString(Qt::ISODate));
    root.insert(QStringLiteral("widgets"), widgetValues);
    return root;
}

QJsonObject MainWindow::buildZeroPoseConfigSnapshot() const
{
    QJsonObject widgetValues;
    widgetValues.insert(QStringLiteral("devEndHomeX"), ui->devEndHomeX->value());
    widgetValues.insert(QStringLiteral("devEndHomeY"), ui->devEndHomeY->value());
    widgetValues.insert(QStringLiteral("devEndHomeZ"), ui->devEndHomeZ->value());
    widgetValues.insert(QStringLiteral("devEndHomeRx"), ui->devEndHomeRx->value());
    widgetValues.insert(QStringLiteral("devEndHomeRy"), ui->devEndHomeRy->value());
    widgetValues.insert(QStringLiteral("devEndHomeRz"), ui->devEndHomeRz->value());

    QJsonObject root;
    root.insert(QStringLiteral("schema_version"), 1);
    root.insert(QStringLiteral("profile_type"), QStringLiteral("zero_pose"));
    root.insert(QStringLiteral("saved_at"), QDateTime::currentDateTime().toString(Qt::ISODate));
    root.insert(QStringLiteral("widgets"), widgetValues);
    root.insert(QStringLiteral("dev_end_home"),
                toJsonArray({
                                ui->devEndHomeX->value(),
                                ui->devEndHomeY->value(),
                                ui->devEndHomeZ->value(),
                                ui->devEndHomeRx->value(),
                                ui->devEndHomeRy->value(),
                                ui->devEndHomeRz->value()
                            }));
    return root;
}

bool MainWindow::saveParameterConfigToFile(const QString& filePath, QString* errorMessage) const
{
    QFileInfo fileInfo(filePath);
    if(!QDir().mkpath(fileInfo.absolutePath())){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法创建目录 %1").arg(fileInfo.absolutePath());
        }
        return false;
    }

    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法写入文件 %1").arg(filePath);
        }
        return false;
    }

    file.write(QJsonDocument(buildParameterConfigSnapshot()).toJson(QJsonDocument::Indented));
    file.close();
    return true;
}

bool MainWindow::saveZeroPoseConfigToFile(const QString& filePath, QString* errorMessage) const
{
    QFileInfo fileInfo(filePath);
    if(!QDir().mkpath(fileInfo.absolutePath())){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法创建目录 %1").arg(fileInfo.absolutePath());
        }
        return false;
    }

    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法写入文件 %1").arg(filePath);
        }
        return false;
    }

    file.write(QJsonDocument(buildZeroPoseConfigSnapshot()).toJson(QJsonDocument::Indented));
    file.close();
    return true;
}

QJsonObject MainWindow::buildRobotStateSnapshotJson(const RobotStateSnapshot& state) const
{
    auto cableHomeStateName = [](CableHomeState stateValue) -> QString {
        switch(stateValue){
        case CableHomeState::Unconfirmed:
            return QStringLiteral("unconfirmed");
        case CableHomeState::WaitingForceStable:
            return QStringLiteral("waiting_force_stable");
        case CableHomeState::Confirmed:
            return QStringLiteral("confirmed");
        }
        return QStringLiteral("unknown");
    };

    auto positionMotionStateName = [](PositionMotionState stateValue) -> QString {
        switch(stateValue){
        case PositionMotionState::Idle:
            return QStringLiteral("idle");
        case PositionMotionState::Simulating:
            return QStringLiteral("simulating");
        case PositionMotionState::ExecutingPvt:
            return QStringLiteral("executing_pvt");
        case PositionMotionState::PausedPvt:
            return QStringLiteral("paused_pvt");
        }
        return QStringLiteral("unknown");
    };

    auto forceMotionStateName = [](ForceMotionState stateValue) -> QString {
        switch(stateValue){
        case ForceMotionState::Idle:
            return QStringLiteral("idle");
        case ForceMotionState::DirectForceControl:
            return QStringLiteral("direct_force_control");
        case ForceMotionState::GravityCompensation:
            return QStringLiteral("gravity_compensation");
        }
        return QStringLiteral("unknown");
    };

    QJsonObject object;
    object.insert(QStringLiteral("system_running"), state.systemRunning);
    object.insert(QStringLiteral("ls_connected"), state.lsConnected);
    object.insert(QStringLiteral("control_thread_running"), state.controlThreadRunning);
    object.insert(QStringLiteral("cable_home_state"), cableHomeStateName(state.cableHomeState));
    object.insert(QStringLiteral("pos_mode_running"), state.posModeRunning);
    object.insert(QStringLiteral("gc_running"), state.gcRunning);
    object.insert(QStringLiteral("force_control_thread_checked"), state.forceControlThreadChecked);
    object.insert(QStringLiteral("force_control_thread_running"), state.forceControlThreadRunning);
    object.insert(QStringLiteral("pvt_trajectory_available"), state.pvtTrajectoryAvailable);
    object.insert(QStringLiteral("pvt_motion_running"), state.pvtMotionRunning);
    object.insert(QStringLiteral("pvt_motion_paused"), state.pvtMotionPaused);
    object.insert(QStringLiteral("pvt_motion_finished"), state.pvtMotionFinished);
    object.insert(QStringLiteral("single_motor_trajectory_active"),
                  state.singleMotorTrajectoryActive);
    object.insert(QStringLiteral("any_motion_running"), state.anyMotionRunning);
    object.insert(QStringLiteral("run_mode"), runModeDisplayName(state.runMode));
    object.insert(QStringLiteral("position_motion_state"),
                  positionMotionStateName(state.positionMotionState));
    object.insert(QStringLiteral("force_motion_state"),
                  forceMotionStateName(state.forceMotionState));
    return object;
}

QJsonObject MainWindow::buildRuntimeStateSnapshotJson() const
{
    auto cableHomeStateName = [](CableHomeState stateValue) -> QString {
        switch(stateValue){
        case CableHomeState::Unconfirmed:
            return QStringLiteral("unconfirmed");
        case CableHomeState::WaitingForceStable:
            return QStringLiteral("waiting_force_stable");
        case CableHomeState::Confirmed:
            return QStringLiteral("confirmed");
        }
        return QStringLiteral("unknown");
    };

    QJsonObject object;
    object.insert(QStringLiteral("system_running"), runtimeState.systemRunning);
    object.insert(QStringLiteral("gc_running"), runtimeState.gcRunning);
    object.insert(QStringLiteral("pos_mode_running"), runtimeState.posModeRunning);
    object.insert(QStringLiteral("hybrid_pose_force_mode_active"),
                  runtimeState.hybridPoseForceModeActive);
    object.insert(QStringLiteral("single_cable_force_debug_mode"),
                  isSingleCableForceDebugModeActive());
    object.insert(QStringLiteral("motor_torque_debug_active"),
                  runtimeState.motorTorqueDebugActive);
    object.insert(QStringLiteral("single_motor_trajectory_active"),
                  runtimeState.singleMotorTrajectoryActive);
    object.insert(QStringLiteral("run_mode"), runModeDisplayName(runtimeState.runMode));
    object.insert(QStringLiteral("cable_home_state"),
                  cableHomeStateName(runtimeState.cableHomeState));
    object.insert(QStringLiteral("auto_execute_pose_after_simulation"),
                  runtimeState.autoExecutePoseAfterSimulation);
    object.insert(QStringLiteral("safety_fault_latched"), runtimeState.safetyFaultLatched);
    object.insert(QStringLiteral("safety_stop_level"), runtimeState.safetyStopLevel);
    object.insert(QStringLiteral("safety_fault_code"), runtimeState.safetyFaultCode);
    object.insert(QStringLiteral("safety_fault_summary"), runtimeState.safetyFaultSummary);
    object.insert(QStringLiteral("safety_fault_detail"), runtimeState.safetyFaultDetail);
    object.insert(QStringLiteral("control_box_motion_control_state"),
                  runtimeState.controlBoxMotionControlState);
    object.insert(QStringLiteral("control_box_speed_zero_state"),
                  runtimeState.controlBoxSpeedZeroState);
    object.insert(QStringLiteral("control_box_zero_calib_state"),
                  runtimeState.controlBoxZeroCalibState);
    object.insert(QStringLiteral("control_box_start_requires_stop"),
                  runtimeState.controlBoxStartRequiresStop);
    object.insert(QStringLiteral("control_box_start_interlock_reported"),
                  runtimeState.controlBoxStartInterlockReported);
    object.insert(QStringLiteral("control_box_paused_pvt_motion"),
                  runtimeState.controlBoxPausedPvtMotion);
    object.insert(QStringLiteral("control_box_paused_force_control"),
                  runtimeState.controlBoxPausedForceControl);
    object.insert(QStringLiteral("control_box_paused_gc"),
                  runtimeState.controlBoxPausedGC);
    return object;
}

QJsonObject MainWindow::buildControlSnapshotJson(const ControlWorker::Snapshot& snapshot) const
{
    QJsonObject object;
    object.insert(QStringLiteral("sequence"),
                  static_cast<qint64>(std::min<quint64>(snapshot.sequence,
                                                        static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    object.insert(QStringLiteral("force_thread_running"), snapshot.forceThreadRunning);
    object.insert(QStringLiteral("force_expected_from_external"),
                  snapshot.forceExpectedFromExternal);
    object.insert(QStringLiteral("force_pid_output_mode"),
                  snapshot.forcePidOutputMode == ControlWorker::ForcePidOutputMode::Torque ?
                      QStringLiteral("torque") :
                      QStringLiteral("velocity"));
    object.insert(QStringLiteral("motor_abs_pos"), toJsonArray(snapshot.motorAbsPos));
    object.insert(QStringLiteral("motor_rel_raw_pos"), toJsonArray(snapshot.motorRelRawPos));
    object.insert(QStringLiteral("motor_vel"), toJsonArray(snapshot.motorVel));
    object.insert(QStringLiteral("motor_torque_nm"), toJsonArray(snapshot.motorTorqueNm));
    object.insert(QStringLiteral("motor_command"), toJsonArray(snapshot.motorCommand));
    object.insert(QStringLiteral("force_sensor_value"), toJsonArray(snapshot.forceSensorValue));
    object.insert(QStringLiteral("expected_force"), toJsonArray(snapshot.expectedForce));
    QJsonObject timingObject;
    timingObject.insert(QStringLiteral("control_loop_tick_count"),
                        static_cast<qint64>(snapshot.timingDiagnostics.controlLoopTickCount));
    timingObject.insert(QStringLiteral("control_loop_interval_count"),
                        static_cast<qint64>(snapshot.timingDiagnostics.controlLoopIntervalCount));
    timingObject.insert(QStringLiteral("control_loop_interval_sum_us"),
                        snapshot.timingDiagnostics.controlLoopIntervalSumUs);
    timingObject.insert(QStringLiteral("latest_control_loop_interval_us"),
                        snapshot.timingDiagnostics.latestControlLoopIntervalUs);
    timingObject.insert(QStringLiteral("sensor_frame_count"),
                        static_cast<qint64>(snapshot.timingDiagnostics.sensorFrameCount));
    timingObject.insert(QStringLiteral("sensor_frame_interval_count"),
                        static_cast<qint64>(snapshot.timingDiagnostics.sensorFrameIntervalCount));
    timingObject.insert(QStringLiteral("sensor_frame_interval_sum_us"),
                        snapshot.timingDiagnostics.sensorFrameIntervalSumUs);
    timingObject.insert(QStringLiteral("latest_sensor_frame_interval_us"),
                        snapshot.timingDiagnostics.latestSensorFrameIntervalUs);
    object.insert(QStringLiteral("timing_diagnostics"), timingObject);
    return object;
}

QJsonObject MainWindow::buildStructuredFaultRecord(const QString& eventType,
                                                   int level,
                                                   int code,
                                                   const QString& summary,
                                                   const QString& detail,
                                                   bool stopActionAttempted,
                                                   bool stopActionSucceeded,
                                                   const QString& note) const
{
    auto stopLevelName = [](int stopLevel) -> QString {
        switch(static_cast<SafetyMonitor::StopLevel>(stopLevel)){
        case SafetyMonitor::StopLevel::Warning:
            return QStringLiteral("warning");
        case SafetyMonitor::StopLevel::ControlledStop:
            return QStringLiteral("controlled_stop");
        case SafetyMonitor::StopLevel::SafetyStop:
            return QStringLiteral("safety_stop");
        case SafetyMonitor::StopLevel::EmergencyStop:
            return QStringLiteral("emergency_stop");
        }
        return QStringLiteral("none");
    };

    auto faultCodeName = [](int faultCode) -> QString {
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
        }
        return QStringLiteral("unknown");
    };

    QJsonObject object;
    object.insert(QStringLiteral("schema_version"), 1);
    object.insert(QStringLiteral("logged_at"),
                  QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    object.insert(QStringLiteral("event_type"), eventType);
    object.insert(QStringLiteral("stop_level"), stopLevelName(level));
    object.insert(QStringLiteral("stop_level_value"), level);
    object.insert(QStringLiteral("fault_code"), faultCodeName(code));
    object.insert(QStringLiteral("fault_code_value"), code);
    object.insert(QStringLiteral("summary"), summary);
    object.insert(QStringLiteral("detail"), detail);
    object.insert(QStringLiteral("stop_action_attempted"), stopActionAttempted);
    object.insert(QStringLiteral("stop_action_succeeded"), stopActionSucceeded);
    object.insert(QStringLiteral("leadshine_enabled_in_ui"),
                  ui && ui->devUseLS ? ui->devUseLS->isChecked() : false);
    object.insert(QStringLiteral("structured_log_path"),
                  QDir::toNativeSeparators(structuredFaultLogFilePath()));
    object.insert(QStringLiteral("text_log_path"),
                  QDir::toNativeSeparators(uiEventLogFilePath()));
    object.insert(QStringLiteral("software_fault_guard_log_path"),
                  QDir::toNativeSeparators(softwareFaultGuardLogFilePath()));
    if(!note.isEmpty()){
        object.insert(QStringLiteral("note"), note);
    }

    const RobotStateSnapshot robotState = currentRobotState(false);
    object.insert(QStringLiteral("robot_state"), buildRobotStateSnapshotJson(robotState));
    object.insert(QStringLiteral("runtime_state"), buildRuntimeStateSnapshotJson());

    if(controlWorker){
        object.insert(QStringLiteral("control_snapshot"),
                      buildControlSnapshotJson(controlWorker->latestSnapshot()));
    }

    std::vector<double> actualPose;
    if(currentEstimatedEndPose(actualPose, 1000) && !actualPose.empty()){
        object.insert(QStringLiteral("actual_pose"), toJsonArray(actualPose));
    }

    WorkspaceBounds bounds;
    if(buildSafetyWorkspaceBounds(bounds)){
        QJsonObject workspaceObject;
        workspaceObject.insert(QStringLiteral("x_min"), bounds.xMin);
        workspaceObject.insert(QStringLiteral("x_max"), bounds.xMax);
        workspaceObject.insert(QStringLiteral("y_min"), bounds.yMin);
        workspaceObject.insert(QStringLiteral("y_max"), bounds.yMax);
        workspaceObject.insert(QStringLiteral("z_min"), bounds.zMin);
        workspaceObject.insert(QStringLiteral("z_max"), bounds.zMax);
        workspaceObject.insert(QStringLiteral("warning_margin"), bounds.warningMargin);
        workspaceObject.insert(QStringLiteral("severe_overflow"), bounds.severeOverflow);
        object.insert(QStringLiteral("workspace_bounds"), workspaceObject);
    }

    object.insert(QStringLiteral("parameter_snapshot"), buildParameterConfigSnapshot());
    return object;
}

bool MainWindow::loadParameterConfigFromFile(const QString& filePath, QString* errorMessage)
{
    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        if(errorMessage){
            *errorMessage = QStringLiteral("无法读取文件 %1").arg(filePath);
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    file.close();
    if(parseError.error != QJsonParseError::NoError || !document.isObject()){
        if(errorMessage){
            *errorMessage = QStringLiteral("JSON 解析失败：%1").arg(parseError.errorString());
        }
        return false;
    }

    const QJsonObject snapshot = document.object().value(QStringLiteral("widgets")).toObject();
    if(snapshot.isEmpty()){
        if(errorMessage){
            *errorMessage = QStringLiteral("配置文件中未找到 widgets 配置项");
        }
        return false;
    }

    applyParameterConfigSnapshot(snapshot);
    return true;
}

void MainWindow::applyParameterConfigSnapshot(const QJsonObject& snapshot)
{
    suppressMotorLimitUnitConversion = true;
    for(auto it = snapshot.constBegin(); it != snapshot.constEnd(); ++it){
        QWidget* widget = findChild<QWidget*>(it.key());
        if(!shouldPersistParameterWidget(widget)){
            continue;
        }

        if(auto spinBox = qobject_cast<QSpinBox*>(widget)){
            spinBox->setValue(it.value().toInt(spinBox->value()));
        }
        else if(auto doubleSpinBox = qobject_cast<QDoubleSpinBox*>(widget)){
            doubleSpinBox->setValue(it.value().toDouble(doubleSpinBox->value()));
        }
        else if(auto checkBox = qobject_cast<QCheckBox*>(widget)){
            checkBox->setChecked(it.value().toBool(checkBox->isChecked()));
        }
        else if(auto radioButton = qobject_cast<QRadioButton*>(widget)){
            radioButton->setChecked(it.value().toBool(radioButton->isChecked()));
        }
        else if(auto comboBox = qobject_cast<QComboBox*>(widget)){
            const QString comboValue = it.value().toString(comboBox->currentText());
            if(comboBox->objectName().startsWith(QStringLiteral("axisType"))){
                if(comboValue.trimmed().isEmpty() || comboValue.trimmed() == QStringLiteral("/")){
                    comboBox->setCurrentIndex(0);
                }
                else if(comboValue.contains(QStringLiteral("直线模组"))){
                    comboBox->setCurrentText(linearMotorAxisText());
                }
                else{
                    comboBox->setCurrentText(motorAxisText());
                }
            }
            else{
                comboBox->setCurrentText(comboValue);
            }
        }
    }
    suppressMotorLimitUnitConversion = false;

    lastMotorFeedbackDisplayUnit = currentMotorFeedbackDisplayUnit();
    refreshMotorLimitUnitUi(false);
    endChange(ui->devEndNum->value());
    refreshAxisRuntimeCounts();
    onPosTrajModeChanged(ui->PostrajMode->currentText());
    refreshCalibrationUiState();
    refreshRunModeUiState();
    refreshMotorPosDisplay();
}

void MainWindow::applyParameterTemplate(const QString& templateName)
{
    if(templateName == kParameterTemplateG3){
        ui->devUseG3->setChecked(true);
        paraChange();
        refreshAxisRuntimeCounts();
        onPosTrajModeChanged(ui->PostrajMode->currentText());
        refreshCalibrationUiState();
        refreshRunModeUiState();
        displayInfo("已将 G3 标准模板加载到参数界面，点击“更新并应用参数”后同步到底层模块");
        return;
    }

    displayInfo("当前选择为“当前界面配置”，未执行模板覆盖操作");
}

void MainWindow::appendMessageHistoryEntry(const QString& message, const QString& type)
{
    if(message.isEmpty()){
        return;
    }

    messageHistoryEntries.append(
                QStringLiteral("[%1] [%2] %3")
                .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")))
                .arg(messageTypeDisplayName(type))
                .arg(message));
    constexpr int kMaxMessageHistoryEntries = 500;
    while(messageHistoryEntries.size() > kMaxMessageHistoryEntries){
        messageHistoryEntries.removeFirst();
    }
}

void MainWindow::appendUiEventLog(const QString& message, const QString& type)
{
    if(message.isEmpty()){
        return;
    }

    const QString dirPath = uiEventLogDirPath();
    if(!QDir().mkpath(dirPath)){
        return;
    }

    QFile file(uiEventLogFilePath());
    if(!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)){
        return;
    }

    QTextStream stream(&file);
    stream << QStringLiteral("[%1] [%2] %3\n")
              .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")))
              .arg(messageTypeDisplayName(type))
              .arg(message);
    stream.flush();
    file.close();
}

void MainWindow::appendStructuredFaultLog(const QJsonObject& record) const
{
    if(record.isEmpty()){
        return;
    }

    const QString dirPath = uiEventLogDirPath();
    if(!QDir().mkpath(dirPath)){
        return;
    }

    const QString filePath = structuredFaultLogFilePath();
    QJsonObject rootObject;
    QJsonArray recordsArray;

    QFile readFile(filePath);
    if(readFile.exists() && readFile.open(QIODevice::ReadOnly | QIODevice::Text)){
        QJsonParseError parseError;
        const QJsonDocument existingDocument =
                QJsonDocument::fromJson(readFile.readAll(), &parseError);
        readFile.close();
        if(parseError.error == QJsonParseError::NoError && existingDocument.isObject()){
            rootObject = existingDocument.object();
            recordsArray = rootObject.value(QStringLiteral("records")).toArray();
        }
    }

    rootObject.insert(QStringLiteral("schema_version"), 1);
    rootObject.insert(QStringLiteral("updated_at"),
                      QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    recordsArray.append(record);
    rootObject.insert(QStringLiteral("record_count"), recordsArray.size());
    rootObject.insert(QStringLiteral("records"), recordsArray);

    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        return;
    }

    file.write(QJsonDocument(rootObject).toJson(QJsonDocument::Indented));
    file.flush();
    file.close();
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

    const RuntimeDiagnosticsSummary summary = buildRuntimeDiagnosticsSummary();
    setLabelTextIfChanged(ui->runtimeSensorFreqTitleLabel,
                          QStringLiteral("采集频率（目标 %1 Hz）").arg(configuredSensorSampleFrequencyHz()));
    auto formatHzText = [](double latestHz, double avgHz, quint64 intervalCount) -> QString {
        if(intervalCount == 0 && latestHz <= 0.0){
            return QStringLiteral("暂无有效数据");
        }
        if(intervalCount == 0){
            return QStringLiteral("最新 %1 Hz | 5分钟均值等待更多样本")
                    .arg(latestHz, 0, 'f', latestHz >= 1000.0 ? 0 : 1);
        }
        return QStringLiteral("最新 %1 Hz | 5分钟均值 %2 Hz")
                .arg(latestHz, 0, 'f', latestHz >= 1000.0 ? 0 : 1)
                .arg(avgHz, 0, 'f', avgHz >= 1000.0 ? 0 : 1);
    };
    auto formatPeriodText = [](double latestMs, double avgMs, quint64 intervalCount) -> QString {
        if(intervalCount == 0 && latestMs <= 0.0){
            return QStringLiteral("暂无有效数据");
        }
        if(intervalCount == 0){
            return QStringLiteral("最新 %1 ms | 5分钟均值等待更多样本")
                    .arg(latestMs, 0, 'f', 3);
        }
        return QStringLiteral("最新 %1 ms | 5分钟均值 %2 ms")
                .arg(latestMs, 0, 'f', 3)
                .arg(avgMs, 0, 'f', 3);
    };
    auto formatWindowText = [](qint64 durationMs, const QString& path) -> QString {
        const qint64 totalSeconds = std::max<qint64>(0, durationMs / 1000);
        const qint64 minutes = totalSeconds / 60;
        const qint64 seconds = totalSeconds % 60;
        const QString durationText = QStringLiteral("%1:%2")
                .arg(minutes, 2, 10, QLatin1Char('0'))
                .arg(seconds, 2, 10, QLatin1Char('0'));
        if(durationMs >= kRuntimeDiagnosticsWindowMs){
            if(path.isEmpty()){
                return QStringLiteral("窗口已覆盖最近 5 分钟：%1，等待自动或手动导出报告").arg(durationText);
            }
            return QStringLiteral("窗口已覆盖最近 5 分钟：%1，最近报告 %2")
                    .arg(durationText, QDir::toNativeSeparators(path));
        }
        return QStringLiteral("当前累计窗口 %1 / 05:00，尚未形成完整 5 分钟报告").arg(durationText);
    };

    setLabelTextIfChanged(ui->runtimeSensorFreqValueLabel,
                          formatHzText(summary.sensorLatestHz,
                                       summary.sensorAverageHz,
                                       summary.sensorIntervalCount));
    setLabelTextIfChanged(ui->runtimeCommFreqValueLabel,
                          formatHzText(summary.communicationLatestHz,
                                       summary.communicationAverageHz,
                                       summary.communicationIntervalCount));
    setLabelTextIfChanged(ui->runtimeMotorCycleValueLabel,
                          formatPeriodText(summary.motorCommandLatestPeriodMs,
                                           summary.motorCommandAveragePeriodMs,
                                           summary.motorCommandIntervalCount));
    setLabelTextIfChanged(ui->runtimeDiagnosticsWindowValueLabel,
                          formatWindowText(summary.windowDurationMs, lastRuntimeDiagnosticsReportPath));

    QString reportStatus = QStringLiteral("等待导出");
    if(!lastRuntimeDiagnosticsReportPath.isEmpty()){
        reportStatus = QFileInfo(lastRuntimeDiagnosticsReportPath).fileName();
    }
    setLabelTextIfChanged(ui->runtimeDiagnosticsReportStatusLabel, reportStatus);
}

QJsonObject MainWindow::buildRuntimeDiagnosticsReportJson() const
{
    QJsonObject root;
    root.insert(QStringLiteral("报告名称"), QStringLiteral("运行诊断5分钟报告"));
    root.insert(QStringLiteral("报告版本"), 1);
    root.insert(QStringLiteral("生成时间"),
                QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    root.insert(QStringLiteral("目标统计窗口(ms)"), kRuntimeDiagnosticsWindowMs);
    root.insert(QStringLiteral("原始数据导出方式"),
                QStringLiteral("5分钟诊断报告不再生成原始数据附件；频率和周期证明用原始间隔数据随 session_record_*.txt 会话记录导出。"));

    if(runtimeDiagnosticsHistory.isEmpty()){
        root.insert(QStringLiteral("是否有可导出数据"), false);
        root.insert(QStringLiteral("说明"),
                    QStringLiteral("当前运行诊断窗口内没有可导出的统计数据。"));
        return root;
    }

    const RuntimeDiagnosticsSample& first = runtimeDiagnosticsHistory.first();
    const RuntimeDiagnosticsSample& last = runtimeDiagnosticsHistory.last();
    const RuntimeDiagnosticsSummary summary = buildRuntimeDiagnosticsSummary();
    const auto passText = [](bool pass) -> QString {
        return pass ? QStringLiteral("通过") : QStringLiteral("未通过");
    };

    root.insert(QStringLiteral("是否有可导出数据"), true);

    QJsonObject window;
    window.insert(QStringLiteral("开始时间"),
                  QDateTime::fromMSecsSinceEpoch(first.capturedAtMs).toString(Qt::ISODateWithMs));
    window.insert(QStringLiteral("结束时间"),
                  QDateTime::fromMSecsSinceEpoch(last.capturedAtMs).toString(Qt::ISODateWithMs));
    window.insert(QStringLiteral("窗口时长(ms)"), summary.windowDurationMs);
    window.insert(QStringLiteral("是否覆盖完整5分钟"), summary.windowDurationMs >= kRuntimeDiagnosticsWindowMs);
    window.insert(QStringLiteral("窗口完整性说明"),
                  summary.windowDurationMs >= kRuntimeDiagnosticsWindowMs ?
                      QStringLiteral("已覆盖完整5分钟统计窗口。") :
                      QStringLiteral("当前尚未覆盖完整5分钟，报告基于现有累计窗口生成。"));
    root.insert(QStringLiteral("统计窗口"), window);

    QJsonObject thresholds;
    thresholds.insert(QStringLiteral("采集频率下限(Hz)"), 1000.0);
    thresholds.insert(QStringLiteral("实时通信频率下限(Hz)"), 2000.0);
    thresholds.insert(QStringLiteral("电机控制周期上限(ms)"), 2.0);
    thresholds.insert(QStringLiteral("位置控制周期上限(ms)"), 4.0);
    root.insert(QStringLiteral("验收阈值"), thresholds);

    QJsonArray notes;
    notes.append(QStringLiteral("本报告面向中文测试与验收人员，所有指标均按软件侧实际可观测事件统计。"));
    notes.append(QStringLiteral("采集频率基于 ControlWorker 软件侧张力读取完成次数统计；通用Trace一次读取到多个缓冲帧时，仍只按一次读取计数。"));
    notes.append(QStringLiteral("实时通信频率基于软件可观测到的雷赛硬件 API 读写活动时间间隔统计，不等同于控制卡内部原始 EtherCAT 帧计数。"));
    notes.append(QStringLiteral("电机控制周期基于软件侧电机控制指令发送活动的相邻时间间隔统计。"));
    notes.append(QStringLiteral("位置控制周期基于 ControlWorker 控制循环执行的相邻时间间隔统计。"));
    notes.append(QStringLiteral("为降低主线程和磁盘负担，本5分钟报告只输出汇总结论；原始间隔值按“时间戳 + 相对时间 + 原始间隔(us)”输出到会话记录。"));
    root.insert(QStringLiteral("报告说明"), notes);

    QJsonObject calculation;
    calculation.insert(QStringLiteral("平均频率计算公式"),
                       QStringLiteral("平均频率(Hz) = 区间样本数 × 1000000 / 区间总间隔(us)"));
    calculation.insert(QStringLiteral("最新频率计算公式"),
                       QStringLiteral("最新频率(Hz) = 1000000 / 最新一次间隔(us)"));
    calculation.insert(QStringLiteral("平均周期计算公式"),
                       QStringLiteral("平均周期(ms) = 区间总间隔(us) / 区间样本数 / 1000"));
    calculation.insert(QStringLiteral("最新周期计算公式"),
                       QStringLiteral("最新周期(ms) = 最新一次间隔(us) / 1000"));
    root.insert(QStringLiteral("计算方法"), calculation);

    QJsonObject config;
    const int sensorTargetHz = configuredSensorSampleFrequencyHz();
    config.insert(QStringLiteral("张力传感器采样目标频率(Hz)"),
                  sensorTargetHz);
    config.insert(QStringLiteral("张力软件读取目标周期(us)"),
                  std::max(500, static_cast<int>(std::round(1000000.0 / sensorTargetHz))));
    config.insert(QStringLiteral("张力Trace采样周期(us)"),
                  sensorTargetHz > 1000 ? 500 : 1000);
    config.insert(QStringLiteral("张力传感器低通截止频率(Hz)"),
                  configuredForceSensorLowPassCutoffHz());
    config.insert(QStringLiteral("原始数据记录出口"),
                  QStringLiteral("session_record_*.txt"));
    root.insert(QStringLiteral("当前配置"), config);

    const bool sensorPass =
            summary.sensorIntervalCount > 0 && summary.sensorAverageHz >= 1000.0;
    const bool communicationPass =
            summary.communicationIntervalCount > 0 && summary.communicationAverageHz >= 2000.0;
    const bool motorPass =
            summary.motorCommandIntervalCount > 0 &&
            summary.motorCommandAveragePeriodMs > 0.0 &&
            summary.motorCommandAveragePeriodMs <= 2.0;
    const bool controlLoopPass =
            summary.controlLoopIntervalCount > 0 &&
            summary.controlLoopAveragePeriodMs > 0.0 &&
            summary.controlLoopAveragePeriodMs < 4.0;

    QJsonObject metrics;

    QJsonObject sensorMetric;
    sensorMetric.insert(QStringLiteral("指标含义"),
                        QStringLiteral("ControlWorker 软件侧张力读取完成频率；Trace一次读取多个缓冲帧只计一次。"));
    sensorMetric.insert(QStringLiteral("原始数据说明"),
                        QStringLiteral("原始数据为相邻两次软件侧张力读取完成的时间间隔(us)；一次Trace读取多个缓冲帧不展开。"));
    sensorMetric.insert(QStringLiteral("区间样本数"),
                        static_cast<qint64>(summary.sensorIntervalCount));
    sensorMetric.insert(QStringLiteral("最新值(Hz)"), summary.sensorLatestHz);
    sensorMetric.insert(QStringLiteral("5分钟平均值(Hz)"), summary.sensorAverageHz);
    sensorMetric.insert(QStringLiteral("判定结果"), passText(sensorPass));
    sensorMetric.insert(QStringLiteral("是否通过"), sensorPass);
    metrics.insert(QStringLiteral("采集频率"), sensorMetric);

    QJsonObject communicationMetric;
    communicationMetric.insert(QStringLiteral("指标含义"),
                               QStringLiteral("软件侧可观测的雷赛硬件 API 读写活动频率。"));
    communicationMetric.insert(QStringLiteral("原始数据说明"),
                               QStringLiteral("原始数据为相邻两次雷赛硬件 API 读写事件的时间间隔(us)。"));
    communicationMetric.insert(QStringLiteral("区间样本数"),
                               static_cast<qint64>(summary.communicationIntervalCount));
    communicationMetric.insert(QStringLiteral("最新值(Hz)"), summary.communicationLatestHz);
    communicationMetric.insert(QStringLiteral("5分钟平均值(Hz)"), summary.communicationAverageHz);
    communicationMetric.insert(QStringLiteral("判定结果"), passText(communicationPass));
    communicationMetric.insert(QStringLiteral("是否通过"), communicationPass);
    metrics.insert(QStringLiteral("实时通信频率"), communicationMetric);

    QJsonObject motorMetric;
    motorMetric.insert(QStringLiteral("指标含义"),
                       QStringLiteral("软件侧连续两次电机控制指令发送之间的控制周期。"));
    motorMetric.insert(QStringLiteral("原始数据说明"),
                       QStringLiteral("原始数据为相邻两次电机控制指令发送事件的时间间隔(us)。"));
    motorMetric.insert(QStringLiteral("区间样本数"),
                       static_cast<qint64>(summary.motorCommandIntervalCount));
    motorMetric.insert(QStringLiteral("最新值(ms)"), summary.motorCommandLatestPeriodMs);
    motorMetric.insert(QStringLiteral("5分钟平均值(ms)"), summary.motorCommandAveragePeriodMs);
    motorMetric.insert(QStringLiteral("判定结果"), passText(motorPass));
    motorMetric.insert(QStringLiteral("是否通过"), motorPass);
    metrics.insert(QStringLiteral("电机控制周期"), motorMetric);

    QJsonObject controlLoopMetric;
    controlLoopMetric.insert(QStringLiteral("指标含义"),
                             QStringLiteral("ControlWorker 控制循环相邻两次执行之间的位置控制周期。"));
    controlLoopMetric.insert(QStringLiteral("原始数据说明"),
                             QStringLiteral("原始数据为相邻两次 ControlWorker 控制循环执行的时间间隔(us)。"));
    controlLoopMetric.insert(QStringLiteral("区间样本数"),
                             static_cast<qint64>(summary.controlLoopIntervalCount));
    controlLoopMetric.insert(QStringLiteral("最新值(ms)"), summary.controlLoopLatestPeriodMs);
    controlLoopMetric.insert(QStringLiteral("5分钟平均值(ms)"), summary.controlLoopAveragePeriodMs);
    controlLoopMetric.insert(QStringLiteral("判定结果"), passText(controlLoopPass));
    controlLoopMetric.insert(QStringLiteral("是否通过"), controlLoopPass);
    metrics.insert(QStringLiteral("位置控制周期"), controlLoopMetric);

    root.insert(QStringLiteral("统计结果"), metrics);

    QJsonObject conclusion;
    conclusion.insert(QStringLiteral("采集频率"), passText(sensorPass));
    conclusion.insert(QStringLiteral("实时通信频率"), passText(communicationPass));
    conclusion.insert(QStringLiteral("电机控制周期"), passText(motorPass));
    conclusion.insert(QStringLiteral("位置控制周期"), passText(controlLoopPass));
    root.insert(QStringLiteral("结论汇总"), conclusion);

    return root;
}

bool MainWindow::writeRuntimeDiagnosticsReport(QString* outputPath, bool announce)
{
    if(runtimeDiagnosticsReportWriting){
        if(announce){
            displayInfo("诊断报告正在写入，请稍后再试", "warning");
        }
        return false;
    }

    runtimeDiagnosticsReportWriting = true;
    struct ReportWritingGuard {
        bool& flag;
        ~ReportWritingGuard(){ flag = false; }
    } reportWritingGuard{runtimeDiagnosticsReportWriting};

    refreshSafetyMonitorHeartbeat();
    const QJsonObject report = buildRuntimeDiagnosticsReportJson();
    if(!report.value(QStringLiteral("是否有可导出数据")).toBool(false)){
        if(announce){
            displayInfo("当前没有可导出的诊断统计数据", "warning");
        }
        return false;
    }

    const QString dirPath = uiEventLogDirPath();
    if(!QDir().mkpath(dirPath)){
        if(announce){
            displayInfo(QStringLiteral("错误：无法创建诊断报告目录 %1").arg(dirPath).toStdString(), "error");
        }
        return false;
    }

    const QString reportFilePath = runtimeDiagnosticsReportFilePath();
    QFile reportFile(reportFilePath);
    if(!reportFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        if(announce){
            displayInfo(QStringLiteral("错误：无法写入诊断报告 %1").arg(reportFilePath).toStdString(), "error");
        }
        return false;
    }

    const QByteArray reportBytes = QJsonDocument(report).toJson(QJsonDocument::Indented);
    if(reportFile.write(reportBytes) != reportBytes.size()){
        reportFile.close();
        if(announce){
            displayInfo(QStringLiteral("错误：写入诊断报告失败 %1").arg(reportFilePath).toStdString(), "error");
        }
        return false;
    }
    reportFile.flush();
    reportFile.close();
    refreshSafetyMonitorHeartbeat();

    lastRuntimeDiagnosticsReportPath = reportFilePath;
    lastRuntimeDiagnosticsAutoWriteMs = QDateTime::currentMSecsSinceEpoch();
    refreshRuntimeDiagnosticsUi();

    if(outputPath){
        *outputPath = reportFilePath;
    }
    if(announce){
        displayInfo(QStringLiteral("5分钟诊断报告已导出至 %1；原始间隔数据请通过会话记录导出")
                    .arg(reportFilePath).toStdString());
    }
    return true;
}

bool MainWindow::writePoseSimulationCableLengthFile(QString* outputPath)
{
    if(!positionSimulationModel || !positionSimulationModel->isTrajectorySimulationComplete()){
        return false;
    }

    const std::vector<std::vector<double>> cableLengthTraj =
            positionSimulationModel->getCableLengthTraj();
    if(cableLengthTraj.empty()){
        return false;
    }

    const std::vector<double> timeStamps =
            positionSimulationModel->getCableLengthTimeStamps();
    const QString dirPath = uiEventLogDirPath();
    if(!QDir().mkpath(dirPath)){
        displayInfo(QStringLiteral("错误：无法创建仿真绳长导出目录 %1").arg(dirPath).toStdString(), "error");
        return false;
    }

    const QString filePath = poseSimulationCableLengthFilePath();
    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        displayInfo(QStringLiteral("错误：无法写入仿真绳长文件 %1").arg(filePath).toStdString(), "error");
        return false;
    }

    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif
    const int cableCount = static_cast<int>(cableLengthTraj.front().size());
    stream << "# pose simulation cable lengths\n";
    stream << "# generated_at=" << QDateTime::currentDateTime().toString(Qt::ISODateWithMs) << "\n";
    stream << "# unit=mm\n";
    stream << "# row_count=" << static_cast<int>(cableLengthTraj.size()) << "\n";
    stream << "# cable_count=" << cableCount << "\n";
    stream << "point_index\ttime_s";
    for(int cableIndex = 0; cableIndex < cableCount; ++cableIndex){
        stream << "\tcable_" << (cableIndex + 1) << "_mm";
    }
    stream << "\n";

    for(int pointIndex = 0; pointIndex < static_cast<int>(cableLengthTraj.size()); ++pointIndex){
        stream << pointIndex;
        if(pointIndex < static_cast<int>(timeStamps.size())){
            stream << "\t" << QString::number(timeStamps[pointIndex], 'f', 6);
        }
        else{
            stream << "\t";
        }
        for(int cableIndex = 0; cableIndex < cableCount; ++cableIndex){
            if(cableIndex < static_cast<int>(cableLengthTraj[pointIndex].size())){
                stream << "\t" << QString::number(cableLengthTraj[pointIndex][cableIndex], 'f', 6);
            }
            else{
                stream << "\t";
            }
        }
        stream << "\n";
    }

    stream.flush();
    file.close();

    if(stream.status() != QTextStream::Ok){
        displayInfo(QStringLiteral("错误：写入仿真绳长文件失败 %1").arg(filePath).toStdString(), "error");
        return false;
    }

    if(outputPath){
        *outputPath = filePath;
    }
    return true;
}

void MainWindow::initializeSessionRecordingUi()
{
    if(!ui){
        return;
    }
    if(ui->sessionRecordingGroupBox){
        ui->sessionRecordingGroupBox->setTitle(QStringLiteral("历史数据统一存储 / 会话导出"));
    }
    if(ui->sessionRecordingHintLabel){
        ui->sessionRecordingHintLabel->setText(
                    QStringLiteral("按“开始记录”后，统一采集电机控制输入、绳索位移、张力传感器、运动轨迹点，并在导出时附加频率/周期证明用的原始间隔数据；按“结束记录”后自动导出中文可读的会话记录文本。"));
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
        ui->sessionRecordingStopButton->setText(QStringLiteral("结束记录并导出"));
        connect(ui->sessionRecordingStopButton,
                &QPushButton::clicked,
                this,
                [this](){
                    stopSessionRecording(true);
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
    if(sessionRecordingState.active){
        const qint64 elapsedMs = std::max<qint64>(0, QDateTime::currentMSecsSinceEpoch() - sessionRecordingState.startedAtMs);
        statusText = QStringLiteral("记录中：已采样 %1 条，持续 %2 ms")
                .arg(sessionRecordingState.samples.size())
                .arg(elapsedMs);
    }
    else if(sessionRecordingState.startedAtMs > 0){
        const qint64 durationMs = std::max<qint64>(0, sessionRecordingState.endedAtMs - sessionRecordingState.startedAtMs);
        statusText = QStringLiteral("已结束：共导出 %1 条样本，持续 %2 ms")
                .arg(sessionRecordingState.samples.size())
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
    setEnabledIfChanged(ui->sessionRecordingStartButton, !sessionRecordingState.active);
    setEnabledIfChanged(ui->sessionRecordingStopButton, sessionRecordingState.active);
}

bool MainWindow::startSessionRecording()
{
    if(sessionRecordingState.active){
        displayInfo("当前已经在记录会话数据", "warning");
        return false;
    }

    sessionRecordingState.active = true;
    sessionRecordingState.startedAtMs = QDateTime::currentMSecsSinceEpoch();
    sessionRecordingState.endedAtMs = 0;
    sessionRecordingState.samples = QJsonArray();
    sessionRecordingState.parameterSnapshot = buildParameterConfigSnapshot();
    lastSessionRecordingUiRefreshMs = -1;
    refreshSessionRecordingUi();
    displayInfo("已开始记录会话数据，将按时间戳统一采集电机控制输入、绳索位移、张力和轨迹点");
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
    sessionRecordingState.endedAtMs = QDateTime::currentMSecsSinceEpoch();
    const bool writeOk = writeSessionRecordingExport(nullptr, announce);
    refreshSessionRecordingUi();
    return writeOk;
}

void MainWindow::captureSessionRecordingSample(const ControlWorker::Snapshot& snapshot)
{
    if(!sessionRecordingState.active){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    QJsonObject sample;
    sample.insert(QStringLiteral("captured_at"),
                  QDateTime::fromMSecsSinceEpoch(nowMs).toString(Qt::ISODateWithMs));
    sample.insert(QStringLiteral("relative_ms"),
                  std::max<qint64>(0, nowMs - sessionRecordingState.startedAtMs));
    sample.insert(QStringLiteral("control_snapshot"), buildControlSnapshotJson(snapshot));
    sample.insert(QStringLiteral("control_sequence"),
                  static_cast<qint64>(std::min<quint64>(snapshot.sequence,
                                                        static_cast<quint64>(std::numeric_limits<qint64>::max()))));
    sample.insert(QStringLiteral("motor_abs_pos"), toJsonArray(snapshot.motorAbsPos));
    sample.insert(QStringLiteral("motor_rel_raw_pos"), toJsonArray(snapshot.motorRelRawPos));
    sample.insert(QStringLiteral("motor_vel"), toJsonArray(snapshot.motorVel));
    sample.insert(QStringLiteral("motor_torque_nm"), toJsonArray(snapshot.motorTorqueNm));
    sample.insert(QStringLiteral("motor_control_input"), toJsonArray(snapshot.motorCommand));
    sample.insert(QStringLiteral("motor_control_input_mode"),
                  snapshot.forcePidOutputMode == ControlWorker::ForcePidOutputMode::Torque ?
                      QStringLiteral("torque_nm") :
                      QStringLiteral("velocity"));
    sample.insert(QStringLiteral("tension_sensor"), toJsonArray(snapshot.forceSensorValue));
    sample.insert(QStringLiteral("expected_force"), toJsonArray(snapshot.expectedForce));

    QJsonObject timingDiagnostics;
    timingDiagnostics.insert(QStringLiteral("control_loop_tick_count"),
                             static_cast<qint64>(snapshot.timingDiagnostics.controlLoopTickCount));
    timingDiagnostics.insert(QStringLiteral("control_loop_interval_count"),
                             static_cast<qint64>(snapshot.timingDiagnostics.controlLoopIntervalCount));
    timingDiagnostics.insert(QStringLiteral("control_loop_interval_sum_us"),
                             snapshot.timingDiagnostics.controlLoopIntervalSumUs);
    timingDiagnostics.insert(QStringLiteral("latest_control_loop_interval_us"),
                             snapshot.timingDiagnostics.latestControlLoopIntervalUs);
    timingDiagnostics.insert(QStringLiteral("sensor_frame_count"),
                             static_cast<qint64>(snapshot.timingDiagnostics.sensorFrameCount));
    timingDiagnostics.insert(QStringLiteral("sensor_frame_interval_count"),
                             static_cast<qint64>(snapshot.timingDiagnostics.sensorFrameIntervalCount));
    timingDiagnostics.insert(QStringLiteral("sensor_frame_interval_sum_us"),
                             snapshot.timingDiagnostics.sensorFrameIntervalSumUs);
    timingDiagnostics.insert(QStringLiteral("latest_sensor_frame_interval_us"),
                             snapshot.timingDiagnostics.latestSensorFrameIntervalUs);
    sample.insert(QStringLiteral("timing_diagnostics"), timingDiagnostics);

    std::vector<std::vector<double>> cableDisplacement;
    const bool hasCableDisplacement = buildCurrentCableDisplacements(cableDisplacement);
    sample.insert(QStringLiteral("cable_displacement_available"), hasCableDisplacement);
    if(hasCableDisplacement){
        sample.insert(QStringLiteral("cable_displacement"), toJsonArray(cableDisplacement));
    }

    std::vector<std::vector<double>> cableLength;
    const bool hasCableLength = buildCurrentCableLengths(cableLength);
    sample.insert(QStringLiteral("cable_length_available"), hasCableLength);
    if(hasCableLength){
        sample.insert(QStringLiteral("cable_length"), toJsonArray(cableLength));
    }

    std::vector<double> trajectoryPoint;
    QString poseSource = QStringLiteral("unavailable");
    if(currentEstimatedEndPose(trajectoryPoint, 1000) && trajectoryPoint.size() >= 6){
        poseSource = ui->devUseCamForActPose->isChecked() ?
                    QStringLiteral("motive") :
                    QStringLiteral("forward_kinematics");
        sample.insert(QStringLiteral("trajectory_point"), toJsonArray(trajectoryPoint));
    }
    sample.insert(QStringLiteral("trajectory_point_source"), poseSource);

    sessionRecordingState.samples.append(sample);
    if(lastSessionRecordingUiRefreshMs < 0 || nowMs - lastSessionRecordingUiRefreshMs >= 250){
        lastSessionRecordingUiRefreshMs = nowMs;
        refreshSessionRecordingUi();
    }
}

QString MainWindow::buildSessionRecordingExportText() const
{
    QStringList lines;
    auto appendSectionTitle = [&lines](const QString& title){
        lines.append(QString());
        lines.append(QStringLiteral("[%1]").arg(title));
    };
    auto appendStandardSection = [this, &lines, &appendSectionTitle](const QString& title,
                                                                     const QString& description,
                                                                     const QString& key,
                                                                     const QString& availabilityKey = QString()){
        appendSectionTitle(title);
        lines.append(QStringLiteral("说明\t%1").arg(description));
        lines.append(QStringLiteral("时间戳\t相对时间(ms)\t数据"));
        bool hasData = false;
        for(const QJsonValue& value : sessionRecordingState.samples){
            const QJsonObject sample = value.toObject();
            if(!availabilityKey.isEmpty() && !sample.value(availabilityKey).toBool(false)){
                continue;
            }
            if(!sample.contains(key)){
                continue;
            }
            lines.append(QStringLiteral("%1\t%2\t%3")
                         .arg(sample.value(QStringLiteral("captured_at")).toString())
                         .arg(sample.value(QStringLiteral("relative_ms")).toVariant().toLongLong())
                         .arg(jsonValueToReadableText(sample.value(key))));
            hasData = true;
        }
        if(!hasData){
            lines.append(QStringLiteral("无有效数据\t\t当前记录阶段该项数据不可用"));
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
    lines.append(QStringLiteral("样本数量\t%1").arg(sessionRecordingState.samples.size()));
    const QVector<ControlWorker::SensorValueSample> tensionSamples =
            controlWorker ? controlWorker->sensorValueHistory(
                                sessionRecordingState.startedAtMs,
                                sessionRecordingState.endedAtMs > 0 ?
                                    sessionRecordingState.endedAtMs :
                                    QDateTime::currentMSecsSinceEpoch())
                          : QVector<ControlWorker::SensorValueSample>();
    if(!tensionSamples.isEmpty()){
        lines.append(QStringLiteral("张力采样数量\t%1").arg(tensionSamples.size()));
    }
    lines.append(QStringLiteral("说明\t本文件面向中文测试与验收人员整理。"));
    lines.append(QStringLiteral("说明\t样本数据均按“时间戳 + 相对时间 + 数据”输出，便于逐行核查。"));
    lines.append(QStringLiteral("说明\t电机控制输入为控制线程输出的电机控制指令。"));
    lines.append(QStringLiteral("说明\t绳索位移按当前建模轴顺序整理；绳长在零位校准确认后可用。"));
    lines.append(QStringLiteral("说明\t轨迹点来源遵循记录时刻的位姿来源配置。"));

    appendSectionTitle(QStringLiteral("开始记录时的参数快照"));
    lines.append(QStringLiteral("说明\t以下参数快照记录开始按下“开始记录”时的参数值。参数键名沿用程序内部对象名，便于程序追溯。"));
    if(sessionRecordingState.parameterSnapshot.contains(QStringLiteral("saved_at"))){
        lines.append(QStringLiteral("参数快照保存时间\t%1")
                     .arg(sessionRecordingState.parameterSnapshot.value(QStringLiteral("saved_at")).toString()));
    }
    const QJsonObject parameterWidgets =
            sessionRecordingState.parameterSnapshot.value(QStringLiteral("widgets")).toObject();
    if(parameterWidgets.isEmpty()){
        lines.append(QStringLiteral("参数键名\t参数值"));
        lines.append(QStringLiteral("无参数快照\t空"));
    }
    else{
        QStringList keys = parameterWidgets.keys();
        std::sort(keys.begin(), keys.end());
        lines.append(QStringLiteral("参数键名\t参数值"));
        for(const QString& key : keys){
            lines.append(QStringLiteral("%1\t%2")
                         .arg(key, jsonValueToReadableText(parameterWidgets.value(key))));
        }
    }

    if(sessionRecordingState.samples.isEmpty()){
        appendSectionTitle(QStringLiteral("样本数据"));
        lines.append(QStringLiteral("说明\t当前会话未记录到任何样本数据。"));
        return lines.join(QStringLiteral("\r\n"));
    }

    appendStandardSection(QStringLiteral("电机控制输入"),
                          QStringLiteral("控制线程输出的电机控制输入，数组顺序与当前电机轴顺序一致。"),
                          QStringLiteral("motor_control_input"));

    appendSectionTitle(QStringLiteral("张力传感器数据"));
    lines.append(QStringLiteral("说明\t张力采样线程每次采样完成后的当前数值；若采样线程历史不可用，则回退到控制快照记录。"));
    lines.append(QStringLiteral("时间戳\t相对时间(ms)\t数据"));
    bool hasTensionSamples = false;
    for(const ControlWorker::SensorValueSample& sample : tensionSamples){
        const qint64 sampleWallClockUs = sample.wallClockUs > 0 ?
                    sample.wallClockUs :
                    sample.wallClockMs * 1000;
        const double relativeMs =
                static_cast<double>(std::max<qint64>(
                                        0,
                                        sampleWallClockUs - sessionRecordingState.startedAtMs * 1000)) / 1000.0;
        lines.append(QStringLiteral("%1\t%2\t%3")
                     .arg(QDateTime::fromMSecsSinceEpoch(sample.wallClockMs).toString(Qt::ISODateWithMs))
                     .arg(QString::number(relativeMs, 'f', 3))
                     .arg(jsonValueToReadableText(toJsonArray(sample.values))));
        hasTensionSamples = true;
    }
    if(!hasTensionSamples){
        for(const QJsonValue& value : sessionRecordingState.samples){
            const QJsonObject sample = value.toObject();
            if(!sample.contains(QStringLiteral("tension_sensor"))){
                continue;
            }
            lines.append(QStringLiteral("%1\t%2\t%3")
                         .arg(sample.value(QStringLiteral("captured_at")).toString())
                         .arg(sample.value(QStringLiteral("relative_ms")).toVariant().toLongLong())
                         .arg(jsonValueToReadableText(sample.value(QStringLiteral("tension_sensor")))));
            hasTensionSamples = true;
        }
    }
    if(!hasTensionSamples){
        lines.append(QStringLiteral("无有效数据\t\t当前记录阶段该项数据不可用"));
    }

    appendStandardSection(QStringLiteral("期望张力"),
                          QStringLiteral("记录时刻控制器使用的期望张力。"),
                          QStringLiteral("expected_force"));
    appendStandardSection(QStringLiteral("电机绝对位置"),
                          QStringLiteral("记录时刻各电机的绝对位置反馈。"),
                          QStringLiteral("motor_abs_pos"));
    appendStandardSection(QStringLiteral("电机相对原始位置"),
                          QStringLiteral("记录时刻扣除电机零位后的相对原始位置。"),
                          QStringLiteral("motor_rel_raw_pos"));
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
    lines.append(QStringLiteral("时间戳\t相对时间(ms)\t轨迹点来源\t数据"));
    bool hasTrajectoryPoint = false;
    for(const QJsonValue& value : sessionRecordingState.samples){
        const QJsonObject sample = value.toObject();
        if(!sample.contains(QStringLiteral("trajectory_point"))){
            continue;
        }
        lines.append(QStringLiteral("%1\t%2\t%3\t%4")
                     .arg(sample.value(QStringLiteral("captured_at")).toString())
                     .arg(sample.value(QStringLiteral("relative_ms")).toVariant().toLongLong())
                     .arg(sessionRecordingPoseSourceText(sample.value(QStringLiteral("trajectory_point_source")).toString()))
                     .arg(jsonValueToReadableText(sample.value(QStringLiteral("trajectory_point")))));
        hasTrajectoryPoint = true;
    }
    if(!hasTrajectoryPoint){
        lines.append(QStringLiteral("无有效数据\t\t轨迹点不可用\t当前记录阶段未获取到有效末端轨迹点"));
    }

    appendSectionTitle(QStringLiteral("控制时序诊断"));
    lines.append(QStringLiteral("说明\t记录每个采样时刻对应的控制循环与传感器采样时序累计信息，便于分析记录期间控制状态。"));
    lines.append(QStringLiteral("时间戳\t相对时间(ms)\t数据"));
    bool hasTimingDiagnostics = false;
    for(const QJsonValue& value : sessionRecordingState.samples){
        const QJsonObject sample = value.toObject();
        const QJsonObject timingObject = sample.value(QStringLiteral("timing_diagnostics")).toObject();
        if(timingObject.isEmpty()){
            continue;
        }
        lines.append(QStringLiteral("%1\t%2\t%3")
                     .arg(sample.value(QStringLiteral("captured_at")).toString())
                     .arg(sample.value(QStringLiteral("relative_ms")).toVariant().toLongLong())
                     .arg(formatSessionRecordingTimingText(timingObject)));
        hasTimingDiagnostics = true;
    }
    if(!hasTimingDiagnostics){
        lines.append(QStringLiteral("无有效数据\t\t当前记录阶段没有可用的控制时序诊断信息"));
    }

    return lines.join(QStringLiteral("\r\n"));
}

bool MainWindow::writeSessionRecordingDiagnosticRawSections(QTextStream& stream, QString* errorMessage)
{
    if(errorMessage){
        errorMessage->clear();
    }

    const qint64 startMs = sessionRecordingState.startedAtMs;
    const qint64 endMs = sessionRecordingState.endedAtMs > 0 ?
                sessionRecordingState.endedAtMs :
                QDateTime::currentMSecsSinceEpoch();
    if(startMs <= 0 || endMs <= 0 || endMs < startMs){
        stream << "\n[频率周期原始诊断数据]\n";
        stream << "说明\t当前会话没有有效的起止时间，无法导出频率周期原始间隔。\n";
        return stream.status() == QTextStream::Ok;
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
        stream << line << "\n";
        ++pendingLines;
        return pumpMainThread(false);
    };

    refreshSafetyMonitorHeartbeat();
    const QVector<ControlWorker::DiagnosticRawSample> sensorHistory =
            controlWorker ? controlWorker->sensorTimingHistory(startMs, endMs)
                          : QVector<ControlWorker::DiagnosticRawSample>();
    refreshSafetyMonitorHeartbeat();
    const QVector<ControlWorker::DiagnosticRawSample> controlLoopHistory =
            controlWorker ? controlWorker->controlLoopTimingHistory(startMs, endMs)
                          : QVector<ControlWorker::DiagnosticRawSample>();
    refreshSafetyMonitorHeartbeat();
    const QVector<HardwareInterface::DiagnosticRawSample> communicationHistory =
            hardwareInterface.communicationTimingHistory(startMs, endMs);
    refreshSafetyMonitorHeartbeat();
    const QVector<HardwareInterface::DiagnosticRawSample> motorCommandHistory =
            hardwareInterface.motorCommandTimingHistory(startMs, endMs);
    refreshSafetyMonitorHeartbeat();

    if(!writeLine(QString()) ||
            !writeLine(QStringLiteral("[频率周期原始诊断数据]")) ||
            !writeLine(QStringLiteral("说明\t本节承接原5分钟诊断报告附件用途，用于证明采集频率、实时通信频率、电机控制周期和位置控制周期。")) ||
            !writeLine(QStringLiteral("说明\t为降低自动5分钟报告负担，原始间隔不再随5分钟报告单独保存，而随会话记录统一导出。")) ||
            !writeLine(QStringLiteral("统计窗口开始\t%1").arg(QDateTime::fromMSecsSinceEpoch(startMs).toString(Qt::ISODateWithMs))) ||
            !writeLine(QStringLiteral("统计窗口结束\t%1").arg(QDateTime::fromMSecsSinceEpoch(endMs).toString(Qt::ISODateWithMs))) ||
            !writeLine(QStringLiteral("统计窗口时长(ms)\t%1").arg(std::max<qint64>(0, endMs - startMs)))){
        if(errorMessage){
            *errorMessage = QStringLiteral("写入会话诊断原始数据标题失败");
        }
        return false;
    }

    auto writeControlSamples = [&](const QString& title,
                                   const QString& description,
                                   const QVector<ControlWorker::DiagnosticRawSample>& samples) -> bool {
        if(!writeLine(QString()) ||
                !writeLine(title) ||
                !writeLine(QStringLiteral("说明\t%1").arg(description)) ||
                !writeLine(QStringLiteral("时间戳\t相对时间(ms)\t原始间隔(us)"))){
            return false;
        }
        if(samples.isEmpty()){
            return writeLine(QStringLiteral("无有效数据\t\t0"));
        }
        for(const ControlWorker::DiagnosticRawSample& sample : samples){
            const qint64 sampleWallClockUs = sample.wallClockUs > 0 ?
                        sample.wallClockUs :
                        sample.wallClockMs * 1000;
            const double relativeMs =
                    static_cast<double>(std::max<qint64>(0, sampleWallClockUs - startMs * 1000)) / 1000.0;
            if(!writeLine(QStringLiteral("%1\t%2\t%3")
                          .arg(QDateTime::fromMSecsSinceEpoch(sample.wallClockMs).toString(Qt::ISODateWithMs))
                          .arg(QString::number(relativeMs, 'f', 3))
                          .arg(sample.intervalUs))){
                return false;
            }
        }
        return true;
    };

    auto writeHardwareSamples = [&](const QString& title,
                                    const QString& description,
                                    const QVector<HardwareInterface::DiagnosticRawSample>& samples) -> bool {
        if(!writeLine(QString()) ||
                !writeLine(title) ||
                !writeLine(QStringLiteral("说明\t%1").arg(description)) ||
                !writeLine(QStringLiteral("时间戳\t相对时间(ms)\t原始间隔(us)"))){
            return false;
        }
        if(samples.isEmpty()){
            return writeLine(QStringLiteral("无有效数据\t\t0"));
        }
        for(const HardwareInterface::DiagnosticRawSample& sample : samples){
            if(!writeLine(QStringLiteral("%1\t%2\t%3")
                          .arg(QDateTime::fromMSecsSinceEpoch(sample.wallClockMs).toString(Qt::ISODateWithMs))
                          .arg(std::max<qint64>(0, sample.wallClockMs - startMs))
                          .arg(sample.intervalUs))){
                return false;
            }
        }
        return true;
    };

    const bool writeOk =
            writeControlSamples(QStringLiteral("[采集频率原始数据]"),
                                QStringLiteral("原始数据为相邻两组张力Trace帧的时间间隔(us)；Trace缓冲帧按采样周期展开。"),
                                sensorHistory) &&
            writeHardwareSamples(QStringLiteral("[实时通信频率原始数据]"),
                                 QStringLiteral("原始数据为相邻两次雷赛硬件 API 读写事件的时间间隔(us)。"),
                                 communicationHistory) &&
            writeHardwareSamples(QStringLiteral("[电机控制周期原始数据]"),
                                 QStringLiteral("原始数据为相邻两次电机控制指令发送事件的时间间隔(us)。"),
                                 motorCommandHistory) &&
            writeControlSamples(QStringLiteral("[位置控制周期原始数据]"),
                                QStringLiteral("原始数据为相邻两次 ControlWorker 控制循环执行的时间间隔(us)。"),
                                controlLoopHistory);

    const bool flushOk = pumpMainThread(true);
    if(!writeOk || !flushOk || stream.status() != QTextStream::Ok){
        if(errorMessage){
            *errorMessage = QStringLiteral("写入会话诊断原始数据失败");
        }
        return false;
    }
    return true;
}

bool MainWindow::writeSessionRecordingExport(QString* outputPath, bool announce)
{
    const QString dirPath = uiEventLogDirPath();
    if(!QDir().mkpath(dirPath)){
        if(announce){
            displayInfo(QStringLiteral("错误：无法创建会话导出目录 %1").arg(dirPath).toStdString(), "error");
        }
        return false;
    }

    const QString filePath = QDir(dirPath).filePath(
                QStringLiteral("session_record_%1.txt")
                .arg(QDateTime::fromMSecsSinceEpoch(std::max<qint64>(1, sessionRecordingState.startedAtMs))
                     .toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        if(announce){
            displayInfo(QStringLiteral("错误：无法写入会话导出文件 %1").arg(filePath).toStdString(), "error");
        }
        return false;
    }

    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif
    stream << buildSessionRecordingExportText();
    QString rawDataError;
    const bool rawDataOk = writeSessionRecordingDiagnosticRawSections(stream, &rawDataError);
    stream.flush();
    const bool streamOk = stream.status() == QTextStream::Ok;
    const bool fileFlushOk = file.flush();
    file.close();

    if(!rawDataOk || !streamOk || !fileFlushOk){
        if(announce){
            const QString detail = rawDataError.isEmpty() ?
                        QStringLiteral("写入会话导出文件失败") :
                        rawDataError;
            displayInfo(QStringLiteral("错误：%1 %2").arg(detail, filePath).toStdString(), "error");
        }
        return false;
    }

    sessionRecordingState.lastExportPath = filePath;
    if(outputPath){
        *outputPath = filePath;
    }
    refreshSessionRecordingUi();
    if(announce){
        displayInfo(QStringLiteral("会话数据已导出至 %1（中文文本，按时间戳+数据呈现）").arg(filePath).toStdString());
    }
    return true;
}

void MainWindow::updateRuntimeDiagnostics()
{
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
    if(!runtimeDiagnosticsReportWriting &&
            summary.windowDurationMs >= kRuntimeDiagnosticsWindowMs &&
            (lastRuntimeDiagnosticsAutoWriteMs <= 0 ||
             sample.capturedAtMs - lastRuntimeDiagnosticsAutoWriteMs >= kRuntimeDiagnosticsAutoWriteIntervalMs)){
        if(!writeRuntimeDiagnosticsReport(nullptr, false)){
            lastRuntimeDiagnosticsAutoWriteMs = sample.capturedAtMs;
        }
    }
}

void MainWindow::showMessageHistoryDialog()
{
    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("消息历史"));
    dialog.resize(760, 420);

    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    QTextEdit* historyView = new QTextEdit(&dialog);
    historyView->setReadOnly(true);
    historyView->setPlainText(messageHistoryEntries.isEmpty() ?
                                  QStringLiteral("当前暂无历史消息。") :
                                  messageHistoryEntries.join(QLatin1Char('\n')));
    layout->addWidget(historyView);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch();
    QPushButton* clearButton = new QPushButton(QStringLiteral("清空历史"), &dialog);
    QPushButton* closeButton = new QPushButton(QStringLiteral("关闭"), &dialog);
    actionLayout->addWidget(clearButton);
    actionLayout->addWidget(closeButton);
    layout->addLayout(actionLayout);

    connect(clearButton, &QPushButton::clicked, &dialog, [this, historyView](){
        messageHistoryEntries.clear();
        historyView->setPlainText(QStringLiteral("当前暂无历史消息。"));
    });
    connect(closeButton, &QPushButton::clicked, &dialog, &QDialog::accept);

    dialog.exec();
}

bool MainWindow::openUiEventLogDirectory(bool announceFailure)
{
    const QString dirPath = uiEventLogDirPath();
    if(!QDir().mkpath(dirPath)){
        if(announceFailure){
            displayInfo(QStringLiteral("错误：无法创建日志目录 %1").arg(dirPath).toStdString(), "error");
        }
        return false;
    }
    if(!QDesktopServices::openUrl(QUrl::fromLocalFile(dirPath))){
        if(announceFailure){
            displayInfo(QStringLiteral("错误：无法打开日志目录 %1").arg(dirPath).toStdString(), "error");
        }
        return false;
    }
    return true;
}

bool MainWindow::openUiEventLogFile(bool announceFailure)
{
    const QString dirPath = uiEventLogDirPath();
    const QString filePath = uiEventLogFilePath();
    if(!QDir().mkpath(dirPath)){
        if(announceFailure){
            displayInfo(QStringLiteral("错误：无法创建日志目录 %1").arg(dirPath).toStdString(), "error");
        }
        return false;
    }

    QFile file(filePath);
    if(!file.exists()){
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
            if(announceFailure){
                displayInfo(QStringLiteral("错误：无法创建故障日志文件 %1").arg(filePath).toStdString(), "error");
            }
            return false;
        }
        file.close();
    }

    if(!QDesktopServices::openUrl(QUrl::fromLocalFile(filePath))){
        if(announceFailure){
            displayInfo(QStringLiteral("错误：无法打开故障日志文件 %1").arg(filePath).toStdString(), "error");
        }
        return false;
    }
    return true;
}

void MainWindow::refreshCalibrationUiState()
{
    if(!calibrationStatusGroupBox || !calibrationStatusLabel || !calibrationActionLabel
            || !calibrationStepLabel || !calibrationResultLabel){
        return;
    }
    refreshAxisRuntimeCounts();

    auto setLabelTextIfChanged = [](QLabel* label, const QString& text){
        if(label && label->text() != text){
            label->setText(text);
        }
    };
    auto setLabelStyleIfChanged = [](QLabel* label, const QString& style){
        if(label && label->styleSheet() != style){
            label->setStyleSheet(style);
        }
    };
    auto setEnabledIfChanged = [](QWidget* widget, bool enabled){
        if(widget && widget->isEnabled() != enabled){
            widget->setEnabled(enabled);
        }
    };

    const QString neutralStyle = QStringLiteral(
                "QLabel { color: #334155; background: #f8fafc; border: 1px solid #cbd5e1; "
                "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    const QString warningStyle = QStringLiteral(
                "QLabel { color: #92400e; background: #fef3c7; border: 1px solid #fcd34d; "
                "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    const QString successStyle = QStringLiteral(
                "QLabel { color: #166534; background: #dcfce7; border: 1px solid #86efac; "
                "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");
    const QString errorStyle = QStringLiteral(
                "QLabel { color: #991b1b; background: #fee2e2; border: 1px solid #fca5a5; "
                "border-radius: 4px; padding: 6px 8px; font-weight: 700; }");

    QString statusText = QStringLiteral("校准状态：待开始");
    QString statusStyle = neutralStyle;
    QString actionText = QStringLiteral("操作：先点击“开始零位校准”执行机械回零，回零完成后末端停在零点位姿上方20mm；确认位置后，再次点击“开始预紧”进入预紧与确认流程。");
    QString stageText = QStringLiteral("待开始");

    switch(calibrationWorkflowStage){
    case CalibrationWorkflowStage::MechanicalHoming:
        statusText = QStringLiteral("校准状态：机械回零中");
        statusStyle = warningStyle;
        actionText = QStringLiteral("操作：系统正在执行机械回零，请等待各轴完成归零。");
        stageText = QStringLiteral("机械回零");
        break;
    case CalibrationWorkflowStage::AwaitingPretensionStart:
        statusText = QStringLiteral("校准状态：回零完成，待开始预紧");
        statusStyle = warningStyle;
        actionText = QStringLiteral("操作：确认末端停在零点位姿上方20mm并固定后，点击“开始预紧”进入绳索预紧与张力均衡。");
        stageText = QStringLiteral("等待开始预紧");
        break;
    case CalibrationWorkflowStage::PretensionBalancing:
        statusText = QStringLiteral("校准状态：绳索预紧与均衡中");
        statusStyle = warningStyle;
        actionText = QStringLiteral("操作：系统已进入完全力控，正在执行绳索预紧与张力均衡。");
        stageText = QStringLiteral("绳索预紧/均衡");
        break;
    case CalibrationWorkflowStage::CollectingSensorZero:
        statusText = QStringLiteral("校准状态：采集张力零点中");
        statusStyle = warningStyle;
        actionText = QStringLiteral("操作：系统正在采集当前张力传感器读数作为零点基准。");
        stageText = QStringLiteral("采集张力零点");
        break;
    case CalibrationWorkflowStage::UpdatingSpatialBaseline:
        statusText = QStringLiteral("校准状态：更新空间基准中");
        statusStyle = warningStyle;
        actionText = QStringLiteral("操作：系统正在采集当前末端位姿并更新空间坐标基准。");
        stageText = QStringLiteral("更新空间基准");
        break;
    case CalibrationWorkflowStage::AwaitingConfirm:
        statusText = QStringLiteral("校准状态：待确认并同步");
        statusStyle = warningStyle;
        actionText = QStringLiteral("操作：请在张力稳定后点击“确认并同步”，将新的零位参数写入位姿控制基准。");
        stageText = QStringLiteral("等待确认并同步");
        break;
    case CalibrationWorkflowStage::Completed:
        statusText = QStringLiteral("校准状态：零位校准已完成");
        statusStyle = successStyle;
        actionText = QStringLiteral("操作：零位参数已同步至位姿控制模块，可保存记录、加载历史或直接开始运行。");
        stageText = QStringLiteral("校准完成");
        break;
    case CalibrationWorkflowStage::Stopped:
        statusText = QStringLiteral("校准状态：流程已停止");
        statusStyle = errorStyle;
        actionText = QStringLiteral("操作：当前校准已停止，需重新开始零位校准或加载历史基准后再运行。");
        stageText = QStringLiteral("已停止");
        break;
    case CalibrationWorkflowStage::Idle:
    default:
        if(runtimeState.cableHomeState == CableHomeState::Confirmed){
            statusText = QStringLiteral("校准状态：零位校准已完成");
            statusStyle = successStyle;
            actionText = QStringLiteral("操作：当前零位基准已生效，可重新校准、保存记录或直接开始运行。");
            stageText = QStringLiteral("已完成");
        }
        else if(runtimeState.cableHomeState == CableHomeState::WaitingForceStable){
            statusText = QStringLiteral("校准状态：待张力稳定并确认预紧");
            statusStyle = warningStyle;
            actionText = QStringLiteral("操作：请等待绳索预紧稳定后点击“确认并同步”，或重新开始零位校准。");
            stageText = QStringLiteral("等待预紧确认");
        }
        else if(ui->devUseCamForActPose->isChecked() && !motiveFitConfirmed){
            statusText = QStringLiteral("校准状态：待更新空间基准");
            statusStyle = warningStyle;
            actionText = QStringLiteral("操作：当前启用了动捕位姿，请先更新空间基准，再执行零位校准。");
            stageText = QStringLiteral("待更新空间基准");
        }
        break;
    }

    const QString motiveState = ui->devUseCamForActPose->isChecked() ?
                (motiveFitConfirmed ? QStringLiteral("空间基准：已更新")
                                    : QStringLiteral("空间基准：待更新")) :
                QStringLiteral("空间基准：未启用动捕，使用配置零位");
    const QString cableState = runtimeState.cableHomeState == CableHomeState::Confirmed ?
                QStringLiteral("绳索零位：已确认") :
                (runtimeState.cableHomeState == CableHomeState::WaitingForceStable ?
                     QStringLiteral("绳索零位：待预紧确认") :
                     QStringLiteral("绳索零位：未确认"));

    std::vector<double> motorHome = hardwareInterface.getAllMotorHome();
    if(motorHome.empty()){
        motorHome = motorCurAbsPos;
    }
    const QString motorState = motorHome.empty() ?
                QStringLiteral("电机零位：暂无记录") :
                QStringLiteral("电机零位：%1 轴 %2")
                    .arg(motorHome.size())
                    .arg(formatCalibrationVectorPreview(motorHome));
    const QString sensorState = axisForceSensorNum <= 0 ?
                QStringLiteral("张力零点：未配置传感器") :
                QStringLiteral("张力零点：%1 通道 %2")
                    .arg(axisForceSensorNum)
                    .arg(formatCalibrationVectorPreview(forceSensorCurHome));
    const std::vector<double> spatialHome = {
        ui->devEndHomeX->value(),
        ui->devEndHomeY->value(),
        ui->devEndHomeZ->value(),
        ui->devEndHomeRx->value(),
        ui->devEndHomeRy->value(),
        ui->devEndHomeRz->value()
    };
    const QString spatialState = QStringLiteral("空间零位：%1")
            .arg(formatCalibrationVectorPreview(spatialHome, 6));

    const QString stepText = QStringLiteral(
                "流程：机械回零 -> 开始预紧 -> 采集张力零点 -> 更新空间基准 -> 确认并同步\n"
                "当前环节：%1\n%2\n%3\n%4\n%5")
            .arg(stageText, motorState, cableState, sensorState, motiveState);

    QString resultText = QStringLiteral("%1\n%2\n%3")
            .arg(spatialState, motorState, sensorState);
    if(lastCalibrationRecordTime.isValid()){
        resultText.append(QStringLiteral("\n最近记录：%1")
                          .arg(lastCalibrationRecordTime.toString(QStringLiteral("yyyy-MM-dd hh:mm:ss"))));
        if(!lastCalibrationRecordPath.isEmpty()){
            resultText.append(QStringLiteral("\n记录文件：%1")
                              .arg(QFileInfo(lastCalibrationRecordPath).fileName()));
        }
        if(!lastCalibrationRecordSummary.isEmpty()){
            resultText.append(QStringLiteral("\n摘要：%1").arg(lastCalibrationRecordSummary));
        }
    }
    else{
        resultText.append(QStringLiteral("\n最近记录：尚未保存或加载零位校准结果"));
    }

    setLabelTextIfChanged(calibrationStatusLabel, statusText);
    setLabelStyleIfChanged(calibrationStatusLabel, statusStyle);
    setLabelTextIfChanged(calibrationActionLabel, actionText);
    setLabelTextIfChanged(calibrationStepLabel, stepText);
    setLabelTextIfChanged(calibrationResultLabel, resultText);

    const bool canConfirm = ui->mainSetCableHome && ui->mainSetCableHome->isEnabled();
    const bool workflowRunning = calibrationWorkflowStage != CalibrationWorkflowStage::Idle &&
            calibrationWorkflowStage != CalibrationWorkflowStage::Completed &&
            calibrationWorkflowStage != CalibrationWorkflowStage::Stopped;
    const bool canStartWorkflow = !workflowRunning || calibrationWorkflowStage == CalibrationWorkflowStage::AwaitingPretensionStart;
    setEnabledIfChanged(calibrationStartButton, canStartWorkflow);
    if(calibrationStartButton){
        const QString startButtonText = calibrationWorkflowStage == CalibrationWorkflowStage::AwaitingPretensionStart ?
                    QStringLiteral("开始预紧") :
                    QStringLiteral("开始零位校准");
        if(calibrationStartButton->text() != startButtonText){
            calibrationStartButton->setText(startButtonText);
        }
    }
    setEnabledIfChanged(calibrationStopButton,
                        workflowRunning ||
                        calibrationWorkflowStage == CalibrationWorkflowStage::AwaitingPretensionStart ||
                        calibrationWorkflowStage == CalibrationWorkflowStage::AwaitingConfirm);
    setEnabledIfChanged(calibrationConfirmButton, canConfirm);
    if(calibrationRestoreButton){
        calibrationRestoreButton->setEnabled(QFileInfo::exists(latestCalibrationSnapshotPath()));
    }
}

void MainWindow::initializeRunModeControls(){
    if(ui->runModeGroupBox){
        ui->runModeGroupBox->setTitle(QStringLiteral("运行模式选择"));
    }
    if(ui->runModeProgramRadio){
        ui->runModeProgramRadio->setText(kRunModeProgram);
    }
    if(ui->runModeRealtimeRadio){
        ui->runModeRealtimeRadio->setText(kRunModeRealtime);
    }
    if(ui->runModeSimulationRadio){
        ui->runModeSimulationRadio->setText(kRunModeSimulation);
    }
    if(ui->runModeRealtimeSourceLabel){
        ui->runModeRealtimeSourceLabel->setText(QStringLiteral("实时轨迹来源"));
    }
    if(ui->runModeRealtimeSourceCombo){
        const QSignalBlocker blocker(ui->runModeRealtimeSourceCombo);
        ui->runModeRealtimeSourceCombo->clear();
        ui->runModeRealtimeSourceCombo->addItem(kRealtimeInputManual);
        ui->runModeRealtimeSourceCombo->addItem(kRealtimeInputExternal);
        ui->runModeRealtimeSourceCombo->setCurrentIndex(0);
    }
    if(ui->runModeUdpTargetPortLabel){
        ui->runModeUdpTargetPortLabel->setText(QStringLiteral("UDP状态回传端口"));
    }
    if(ui->runModeUdpTargetPortSpinBox){
        ui->runModeUdpTargetPortSpinBox->setRange(1, 65535);
        ui->runModeUdpTargetPortSpinBox->setProperty("skipConfig", true);
        ui->runModeUdpTargetPortSpinBox->setToolTip(
                    QStringLiteral("实时模式下，软件会把 UDP 状态包回传到参数配置页设置的下位机 IP 的该端口"));
    }
    syncUdpTargetPortWidgets(udpRealtimeTargetPort());
    if(ui->udpPacketBriefTitleLabel){
        ui->udpPacketBriefTitleLabel->setText(QStringLiteral("最近收包"));
    }
    if(ui->udpPacketActionTitleLabel){
        ui->udpPacketActionTitleLabel->setText(QStringLiteral("动作结果"));
    }
    if(ui->runtimeSensorFreqTitleLabel){
        ui->runtimeSensorFreqTitleLabel->setText(
                    QStringLiteral("采集频率（目标 %1 Hz）").arg(configuredSensorSampleFrequencyHz()));
    }
    if(ui->runtimeCommFreqTitleLabel){
        ui->runtimeCommFreqTitleLabel->setText(QStringLiteral("实时通信频率"));
    }
    if(ui->runtimeMotorCycleTitleLabel){
        ui->runtimeMotorCycleTitleLabel->setText(QStringLiteral("电机控制周期"));
    }
    if(ui->runtimeDiagnosticsWindowTitleLabel){
        ui->runtimeDiagnosticsWindowTitleLabel->setText(QStringLiteral("5分钟窗口"));
    }
    if(ui->runtimeDiagnosticsExportButton){
        ui->runtimeDiagnosticsExportButton->setText(QStringLiteral("导出5分钟报告"));
        ui->runtimeDiagnosticsExportButton->setToolTip(
                    QStringLiteral("导出当前滚动统计窗口的 5 分钟诊断汇总报告到 outputmsg/runtime_diagnostics_report.json；原始间隔数据通过会话记录导出。"));
    }
    if(ui->runtimeDiagnosticsReportStatusLabel){
        ui->runtimeDiagnosticsReportStatusLabel->setText(QStringLiteral("尚未导出"));
    }

    connect(ui->runModeProgramRadio, &QRadioButton::toggled, this, [this](bool checked){
        if(checked){
            setRunMode(RunMode::ProgramControl);
        }
    });
    connect(ui->runModeRealtimeRadio, &QRadioButton::toggled, this, [this](bool checked){
        if(checked){
            setRunMode(RunMode::RealtimeTrajectory);
        }
    });
    connect(ui->runModeSimulationRadio, &QRadioButton::toggled, this, [this](bool checked){
        if(checked){
            setRunMode(RunMode::SimulationCalculation);
        }
    });
    connect(ui->runModeRealtimeSourceCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int){
                refreshRunModeUiState();
            });
    connect(ui->runModeUdpTargetPortSpinBox,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this,
            [this](int value){
                syncUdpTargetPortWidgets(value);
                applyUdpRuntimeConfigChange();
            });
    connect(ui->runtimeDiagnosticsExportButton,
            &QPushButton::clicked,
            this,
            [this](){
                writeRuntimeDiagnosticsReport(nullptr, true);
            });
    {
        const QSignalBlocker blockerProgram(ui->runModeProgramRadio);
        const QSignalBlocker blockerRealtime(ui->runModeRealtimeRadio);
        const QSignalBlocker blockerSimulation(ui->runModeSimulationRadio);
        ui->runModeProgramRadio->setChecked(true);
    }
    runtimeState.runMode = RunMode::ProgramControl;
    refreshUdpPacketSummaryUi();
    refreshRuntimeDiagnosticsUi();
    refreshRunModeUiState();
}

void MainWindow::initializeSafetyUiControls()
{
    if(ui->safetyStatusGroupBox){
        ui->safetyStatusGroupBox->setTitle(QStringLiteral("安全状态"));
    }
    if(ui->safetyStatusTitleLabel){
        ui->safetyStatusTitleLabel->setText(QStringLiteral("监控状态"));
    }
    if(ui->safetyFaultSummaryTitleLabel){
        ui->safetyFaultSummaryTitleLabel->setText(QStringLiteral("故障摘要"));
    }
    if(ui->safetyFaultDetailTitleLabel){
        ui->safetyFaultDetailTitleLabel->setText(QStringLiteral("处理提示"));
    }
    if(ui->safetyResetButton){
        ui->safetyResetButton->setText(QStringLiteral("安全复位"));
        ui->safetyResetButton->setToolTip(QStringLiteral("清除安全锁存；若控制卡已连接，会先清除所有电机轴错误码"));
        connect(ui->safetyResetButton, &QPushButton::clicked,
                this, &MainWindow::resetSafetyStateFromUi);
    }
    if(ui->safetyEmergencyButton){
        ui->safetyEmergencyButton->setText(QStringLiteral("软件急停"));
        ui->safetyEmergencyButton->setToolTip(QStringLiteral("立即执行软件侧停机，并向执行机构请求急停"));
        ui->safetyEmergencyButton->setStyleSheet(
                    QStringLiteral("QPushButton { color: #991b1b; font-weight: 600; }"));
        ui->safetyEmergencyButton->setVisible(false);
    }

    refreshSafetyUiState();
}

void MainWindow::initializeFaultInjectionUiControls()
{
    if(!ui){
        return;
    }

    if(ui->faultInjectionGroupBox){
        ui->faultInjectionGroupBox->setTitle(QStringLiteral("故障注入"));
    }
    if(ui->faultInjectionHintLabel){
        ui->faultInjectionHintLabel->setText(
                    QStringLiteral("仅用于联调/验收。触发后会直接进入统一安全停机链路，并形成安全锁存。"));
    }
    if(ui->faultInjectionCableLabel){
        ui->faultInjectionCableLabel->setText(QStringLiteral("断绳通道"));
    }
    if(ui->faultInjectionMotorLabel){
        ui->faultInjectionMotorLabel->setText(QStringLiteral("电机轴"));
    }
    if(ui->faultInjectionPlcCommLabel){
        ui->faultInjectionPlcCommLabel->setText(QStringLiteral("通信故障"));
    }
    if(ui->faultInjectionCableButton){
        ui->faultInjectionCableButton->setText(QStringLiteral("模拟断绳"));
        ui->faultInjectionCableButton->setStyleSheet(
                    QStringLiteral("QPushButton { color: #991b1b; font-weight: 600; }"));
        connect(ui->faultInjectionCableButton, &QPushButton::clicked,
                this, &MainWindow::triggerInjectedCableBreak);
    }
    if(ui->faultInjectionMotorButton){
        ui->faultInjectionMotorButton->setText(QStringLiteral("模拟电机故障"));
        ui->faultInjectionMotorButton->setStyleSheet(
                    QStringLiteral("QPushButton { color: #9a3412; font-weight: 600; }"));
        connect(ui->faultInjectionMotorButton, &QPushButton::clicked,
                this, &MainWindow::triggerInjectedMotorFault);
    }
    if(ui->faultInjectionPlcCommButton){
        ui->faultInjectionPlcCommButton->setText(QStringLiteral("模拟工控机与PLC断开通信"));
        ui->faultInjectionPlcCommButton->setStyleSheet(
                    QStringLiteral("QPushButton { color: #7c2d12; font-weight: 600; }"));
        connect(ui->faultInjectionPlcCommButton, &QPushButton::clicked,
                this, &MainWindow::triggerInjectedPlcCommunicationFault);
    }

    refreshFaultInjectionUiControls();
}

MainWindow::RunMode MainWindow::selectedRunModeFromUi() const{
    if(ui->runModeRealtimeRadio && ui->runModeRealtimeRadio->isChecked()){
        return RunMode::RealtimeTrajectory;
    }
    if(ui->runModeSimulationRadio && ui->runModeSimulationRadio->isChecked()){
        return RunMode::SimulationCalculation;
    }
    return RunMode::ProgramControl;
}

QString MainWindow::runModeDisplayName(RunMode mode) const{
    switch(mode){
    case RunMode::ProgramControl:
        return kRunModeProgram;
    case RunMode::RealtimeTrajectory:
        return kRunModeRealtime;
    case RunMode::SimulationCalculation:
        return kRunModeSimulation;
    }
    return kRunModeProgram;
}

int MainWindow::udpRealtimeTargetPort() const
{
    if(ui && ui->devLowerMachinePort){
        return qBound(1, ui->devLowerMachinePort->value(), 65535);
    }
    if(ui && ui->runModeUdpTargetPortSpinBox){
        return qBound(1, ui->runModeUdpTargetPortSpinBox->value(), 65535);
    }
    return kUdpRealtimeDefaultRemotePort;
}

int MainWindow::udpRealtimeListenPort() const
{
    if(ui && ui->devLowerMachineListenPort){
        return qBound(1, ui->devLowerMachineListenPort->value(), 65535);
    }
    return kUdpRealtimeLocalPort;
}

int MainWindow::configuredSensorSampleFrequencyHz() const
{
    if(ui && ui->devSensorSampleFreqHz){
        return qBound(10, ui->devSensorSampleFreqHz->value(), 2000);
    }
    return 1000;
}

double MainWindow::configuredForceSensorLowPassCutoffHz() const
{
    if(ui && ui->devForceSensorLowPassCutoffHz){
        return qBound(0.0, ui->devForceSensorLowPassCutoffHz->value(), 500.0);
    }
    return 0.0;
}

QString MainWindow::udpRealtimeTargetIp() const
{
    if(ui && ui->devLowerMachineIpCombo){
        const QString configuredIp = ui->devLowerMachineIpCombo->currentText().trimmed();
        if(!configuredIp.isEmpty()){
            return configuredIp;
        }
    }
    return kUdpRealtimeRemoteIp;
}

void MainWindow::syncUdpTargetPortWidgets(int port)
{
    const int boundedPort = qBound(1, port, 65535);
    if(ui && ui->devLowerMachinePort && ui->devLowerMachinePort->value() != boundedPort){
        const QSignalBlocker blocker(ui->devLowerMachinePort);
        ui->devLowerMachinePort->setValue(boundedPort);
    }
    if(ui && ui->runModeUdpTargetPortSpinBox && ui->runModeUdpTargetPortSpinBox->value() != boundedPort){
        const QSignalBlocker blocker(ui->runModeUdpTargetPortSpinBox);
        ui->runModeUdpTargetPortSpinBox->setValue(boundedPort);
    }
}

void MainWindow::applyUdpRuntimeConfigChange()
{
    if(ui && ui->devLowerMachineProtocolCombo &&
            ui->devLowerMachineProtocolCombo->currentText() != QStringLiteral("UDP")){
        const QSignalBlocker blocker(ui->devLowerMachineProtocolCombo);
        ui->devLowerMachineProtocolCombo->setCurrentText(QStringLiteral("UDP"));
    }

    if(udpRealtimeBridgeActive){
        stopUdpForRealtimeMode();
        startUdpForRealtimeMode();
    }
    else{
        refreshRunModeUiState();
    }
}

void MainWindow::refreshUdpPacketSummaryUi()
{
    if(!ui){
        return;
    }

    auto setLabelTextIfChanged = [](QLabel* label, const QString& text){
        if(label && label->text() != text){
            label->setText(text);
        }
    };

    const QString packetSummary = udpLastPacketSummaryText.isEmpty() ?
                QStringLiteral("尚未收到 UDP 控制包") :
                udpLastPacketSummaryText;
    const QString actionSummary = udpLastPacketActionText.isEmpty() ?
                QStringLiteral("等待 UDP 控制包。空闲时，pose_command 会走实时手动位姿链路，trajectory_chunk 会走外部轨迹链路。") :
                udpLastPacketActionText;

    setLabelTextIfChanged(ui->udpPacketBriefValueLabel, packetSummary);
    setLabelTextIfChanged(ui->udpPacketActionValueLabel, actionSummary);
}

void MainWindow::setRunMode(RunMode mode, bool announce){
    if(runtimeState.runMode == mode){
        refreshRunModeUiState();
        return;
    }

    const RobotStateSnapshot robotState = currentRobotState();
    if(robotState.anyMotionRunning){
        {
            const QSignalBlocker blockerProgram(ui->runModeProgramRadio);
            const QSignalBlocker blockerRealtime(ui->runModeRealtimeRadio);
            const QSignalBlocker blockerSimulation(ui->runModeSimulationRadio);
            ui->runModeProgramRadio->setChecked(runtimeState.runMode == RunMode::ProgramControl);
            ui->runModeRealtimeRadio->setChecked(runtimeState.runMode == RunMode::RealtimeTrajectory);
            ui->runModeSimulationRadio->setChecked(runtimeState.runMode == RunMode::SimulationCalculation);
        }
        displayInfo("当前存在运行中的运动任务，请先停止后再切换运行模式", "warning");
        return;
    }

    const RunMode previousMode = runtimeState.runMode;
    runtimeState.runMode = mode;
    runtimeState.autoExecutePoseAfterSimulation = false;
    if(previousMode == RunMode::RealtimeTrajectory && mode != RunMode::RealtimeTrajectory){
        stopUdpForRealtimeMode();
    }
    else if(previousMode != RunMode::RealtimeTrajectory && mode == RunMode::RealtimeTrajectory){
        startUdpForRealtimeMode();
    }
    if(announce){
        displayInfo(QStringLiteral("已切换到%1")
                    .arg(runModeDisplayName(mode)).toStdString());
    }
    refreshRunModeUiState();
}

bool MainWindow::isRealtimeManualPoseInputSelected() const{
    return runtimeState.runMode == RunMode::RealtimeTrajectory &&
            (!ui->runModeRealtimeSourceCombo ||
             ui->runModeRealtimeSourceCombo->currentText() == kRealtimeInputManual);
}

bool MainWindow::shouldUseTrajectoryFileInput() const{
    if(!useTrajFile){
        return false;
    }

    switch(runtimeState.runMode){
    case RunMode::ProgramControl:
        return false;
    case RunMode::RealtimeTrajectory:
        return ui->runModeRealtimeSourceCombo &&
                ui->runModeRealtimeSourceCombo->currentText() == kRealtimeInputExternal;
    case RunMode::SimulationCalculation:
        return true;
    }
    return false;
}

void MainWindow::updatePoseInputUiState(){
    bool needEndPose = true;
    bool needRunTime = true;

    if(runtimeState.runMode != RunMode::RealtimeTrajectory){
        const QString normalizedMode = normalizePoseTrajectoryModeName(ui->PostrajMode->currentText());
        if(normalizedMode == kPoseTrajCircular){
            needEndPose = false;
        }
        else if(normalizedMode == kPoseTrajEightShape){
            needEndPose = false;
        }
        else if(normalizedMode == kPoseTrajStepAccel){
            needEndPose = false;
            needRunTime = false;
        }
    }

    const bool fileInputActive = shouldUseTrajectoryFileInput();
    const bool realtimeManualInput = isRealtimeManualPoseInputSelected();
    const bool allowStartPose = !fileInputActive &&
            !(realtimeManualInput && ui->devUseCamForActPose->isChecked());
    const bool allowEndPose = !fileInputActive && needEndPose;
    const bool allowRunTime = !fileInputActive && needRunTime;

    ui->mainPosModeTargetStartPx->setEnabled(allowStartPose);
    ui->mainPosModeTargetStartPy->setEnabled(allowStartPose);
    ui->mainPosModeTargetStartPz->setEnabled(allowStartPose);
    ui->mainPosModeTargetStartRx->setEnabled(allowStartPose);
    ui->mainPosModeTargetStartRy->setEnabled(allowStartPose);
    ui->mainPosModeTargetStartRz->setEnabled(allowStartPose);

    ui->mainPosModeTargetEndPx->setEnabled(allowEndPose);
    ui->mainPosModeTargetEndPy->setEnabled(allowEndPose);
    ui->mainPosModeTargetEndPz->setEnabled(allowEndPose);
    ui->mainPosModeTargetEndRx->setEnabled(allowEndPose);
    ui->mainPosModeTargetEndRy->setEnabled(allowEndPose);
    ui->mainPosModeTargetEndRz->setEnabled(allowEndPose);

    ui->mainPosModeTargetRunTime->setEnabled(allowRunTime);
    ui->mainPosModeTargetStepTime->setEnabled(true);
}

void MainWindow::clearTrajectoryFileSelection(){
    useTrajFile = false;
    trajFileOfflineTraj.clear();
    trajFileTrajPx1.clear();
    trajFileTrajPy1.clear();
    trajFileTrajPz1.clear();
    trajFileTrajRx1.clear();
    trajFileTrajRy1.clear();
    trajFileTrajRz1.clear();
    udpTrajectoryPointBuffer.clear();
}

void MainWindow::syncRealtimeStartPoseFromCurrentPose(bool logResult){
    if(!isRealtimeManualPoseInputSelected()){
        return;
    }

    std::vector<double> currentPose;
    if(currentEstimatedEndPose(currentPose, 1000) && currentPose.size() >= 6){
        ui->mainPosModeTargetStartPx->setValue(currentPose[0]);
        ui->mainPosModeTargetStartPy->setValue(currentPose[1]);
        ui->mainPosModeTargetStartPz->setValue(currentPose[2]);
        ui->mainPosModeTargetStartRx->setValue(currentPose[3]);
        ui->mainPosModeTargetStartRy->setValue(currentPose[4]);
        ui->mainPosModeTargetStartRz->setValue(currentPose[5]);
        if(logResult){
            displayInfo("实时轨迹模式：已将起点同步为当前实际末端位姿");
        }
        updatePoseInputUiState();
        return;
    }

    if(logResult){
        displayInfo("实时轨迹模式：未获取到当前实际末端位姿，继续使用界面中的起点参数", "warning");
    }
    updatePoseInputUiState();
}

bool MainWindow::startActiveRunMode(bool fromControlBox){
    if(!ensureSafetyReadyForMotion(QStringLiteral("运行模式启动"))){
        return false;
    }

    if(runtimeState.runMode == RunMode::SimulationCalculation && fromControlBox){
        displayInfo("控制盒启动仅支持程序控制或实时运动轨迹控制模式", "warning");
        return false;
    }

    if(runtimeState.runMode == RunMode::RealtimeTrajectory){
        if(ui->runModeRealtimeSourceCombo &&
                ui->runModeRealtimeSourceCombo->currentText() == kRealtimeInputExternal &&
                !useTrajFile){
            displayInfo("实时轨迹模式：请先导入外部轨迹文件，或切换到手动位姿指令来源", "warning");
            return false;
        }
        if(isRealtimeManualPoseInputSelected()){
            syncRealtimeStartPoseFromCurrentPose(!fromControlBox);
        }
    }

    runtimeState.autoExecutePoseAfterSimulation =
            runtimeState.runMode != RunMode::SimulationCalculation;
    return runPoseMode();
}

bool MainWindow::handleRunModePrimaryAction(){
    return startActiveRunMode(false);
}

bool MainWindow::handleRunModeSecondaryAction(){
    if(runtimeState.runMode == RunMode::SimulationCalculation){
        displayInfo("仿真计算模式仅执行上位机仿真，不向执行机构下发轨迹", "warning");
        return false;
    }
    if(runtimeState.runMode == RunMode::ProgramControl){
        displayInfo("程序控制模式会在轨迹规划完成后自动执行；如需手动分步执行，请切换到实时运动轨迹控制模式", "warning");
        return false;
    }

    runtimeState.autoExecutePoseAfterSimulation = false;
    if(isRealtimeManualPoseInputSelected()){
        syncRealtimeStartPoseFromCurrentPose(true);
    }
    return startPosePvtTrajectory();
}

void MainWindow::refreshRunModeUiStateThrottled()
{
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if(lastRunModeUiRefreshMs >= 0 && (nowMs - lastRunModeUiRefreshMs) < 200){
        return;
    }
    refreshRunModeUiState();
}

void MainWindow::refreshRunModeUiState(){
    if(!ui){
        return;
    }
    lastRunModeUiRefreshMs = QDateTime::currentMSecsSinceEpoch();

    auto setLabelTextIfChanged = [](QLabel* label, const QString& text){
        if(label && label->text() != text){
            label->setText(text);
        }
    };
    auto setButtonTextIfChanged = [](QAbstractButton* button, const QString& text){
        if(button && button->text() != text){
            button->setText(text);
        }
    };
    auto setVisibleIfChanged = [](QWidget* widget, bool visible){
        if(widget && widget->isHidden() == visible){
            widget->setVisible(visible);
        }
    };
    auto setEnabledIfChanged = [](QWidget* widget, bool enabled){
        if(widget && widget->isEnabled() != enabled){
            widget->setEnabled(enabled);
        }
    };

    {
        const QSignalBlocker blockerProgram(ui->runModeProgramRadio);
        const QSignalBlocker blockerRealtime(ui->runModeRealtimeRadio);
        const QSignalBlocker blockerSimulation(ui->runModeSimulationRadio);
        if(ui->runModeProgramRadio->isChecked() != (runtimeState.runMode == RunMode::ProgramControl)){
            ui->runModeProgramRadio->setChecked(runtimeState.runMode == RunMode::ProgramControl);
        }
        if(ui->runModeRealtimeRadio->isChecked() != (runtimeState.runMode == RunMode::RealtimeTrajectory)){
            ui->runModeRealtimeRadio->setChecked(runtimeState.runMode == RunMode::RealtimeTrajectory);
        }
        if(ui->runModeSimulationRadio->isChecked() != (runtimeState.runMode == RunMode::SimulationCalculation)){
            ui->runModeSimulationRadio->setChecked(runtimeState.runMode == RunMode::SimulationCalculation);
        }
    }

    const RobotStateSnapshot robotState = currentRobotState(false);
    const bool isProgramMode = runtimeState.runMode == RunMode::ProgramControl;
    const bool isRealtimeMode = runtimeState.runMode == RunMode::RealtimeTrajectory;
    const bool isSimulationMode = runtimeState.runMode == RunMode::SimulationCalculation;
    const bool isRealtimeFileMode = isRealtimeMode &&
            ui->runModeRealtimeSourceCombo &&
            ui->runModeRealtimeSourceCombo->currentText() == kRealtimeInputExternal;

    setVisibleIfChanged(ui->runModeRealtimeSourceLabel, isRealtimeMode);
    setEnabledIfChanged(ui->runModeRealtimeSourceLabel, !robotState.anyMotionRunning);
    setVisibleIfChanged(ui->runModeRealtimeSourceCombo, isRealtimeMode);
    setEnabledIfChanged(ui->runModeRealtimeSourceCombo, !robotState.anyMotionRunning);
    setVisibleIfChanged(ui->runModeUdpTargetPortLabel, isRealtimeMode);
    setEnabledIfChanged(ui->runModeUdpTargetPortLabel, !robotState.anyMotionRunning);
    setVisibleIfChanged(ui->runModeUdpTargetPortSpinBox, isRealtimeMode);
    setEnabledIfChanged(ui->runModeUdpTargetPortSpinBox, !robotState.anyMotionRunning);
    setVisibleIfChanged(ui->udpPacketBriefTitleLabel, isRealtimeMode);
    setVisibleIfChanged(ui->udpPacketBriefValueLabel, isRealtimeMode);
    setVisibleIfChanged(ui->udpPacketActionTitleLabel, isRealtimeMode);
    setVisibleIfChanged(ui->udpPacketActionValueLabel, isRealtimeMode);
    setEnabledIfChanged(ui->runtimeDiagnosticsExportButton, !runtimeDiagnosticsHistory.isEmpty());

    setEnabledIfChanged(ui->runModeProgramRadio, !robotState.anyMotionRunning);
    setEnabledIfChanged(ui->runModeRealtimeRadio, !robotState.anyMotionRunning);
    setEnabledIfChanged(ui->runModeSimulationRadio, !robotState.anyMotionRunning);

    if(ui->runModeSummaryLabel){
        QString summary;
        if(isProgramMode){
            summary = QStringLiteral("默认模式。选择预设轨迹与参数后，系统按“轨迹规划/仿真校验/闭环执行”统一流程完成标准测试任务。");
        }
        else if(isRealtimeMode){
            summary = QStringLiteral("在线调试模式。支持手动位姿指令与外部轨迹文件两种输入方式，并开放实时调试/阻抗操作面板。");
            summary.append(QStringLiteral(" "));
            summary.append(buildUdpStatusSummary());
        }
        else{
            summary = QStringLiteral("仿真计算模式。仅在上位机完成轨迹规划、仿真运算与结果可视化，不向执行机构下发运动指令。");
        }
        if(robotState.anyMotionRunning){
            summary.append(QStringLiteral(" 当前任务运行中，模式选择已互锁。"));
        }
        if(runtimeState.safetyFaultLatched){
            summary.append(QStringLiteral(" 当前存在安全锁存故障，需执行安全复位后方可恢复运行。"));
        }
        setLabelTextIfChanged(ui->runModeSummaryLabel, summary);
    }

    refreshRuntimeDiagnosticsUi();

    setVisibleIfChanged(ui->label_21, !isRealtimeMode);
    setVisibleIfChanged(ui->PostrajMode, !isRealtimeMode);
    setVisibleIfChanged(ui->posTrajParamTabWidget, !isRealtimeMode);
    if(ui->frame_53){
        setVisibleIfChanged(ui->frame_53, true);
    }
    if(QFrame* frame54 = findOptionalUiObject<QFrame>(this, "frame_54")){
        setVisibleIfChanged(frame54, isRealtimeMode);
    }
    setVisibleIfChanged(ui->mainPosModeSend, isRealtimeMode);
    setVisibleIfChanged(ui->mainReadTrajFileTrigger, isSimulationMode || isRealtimeFileMode);
    if(ui->mainHybridPoseForceModeSwitch){
        const bool showHybridUi = !isSimulationMode;
        setVisibleIfChanged(ui->mainHybridPoseForceModeSwitch, showHybridUi);
        if(ui->labelHybridForceCableA){
            setVisibleIfChanged(ui->labelHybridForceCableA, showHybridUi);
        }
        if(ui->mainHybridPoseForceCableA){
            setVisibleIfChanged(ui->mainHybridPoseForceCableA, showHybridUi);
        }
        if(ui->labelHybridForceCableB){
            setVisibleIfChanged(ui->labelHybridForceCableB, showHybridUi);
        }
        if(ui->mainHybridPoseForceCableB){
            setVisibleIfChanged(ui->mainHybridPoseForceCableB, showHybridUi);
        }
    }

    if(ui->mainReadTrajFileTrigger){
        QString fileButtonText;
        if(useTrajFile){
            fileButtonText = isRealtimeMode ?
                QStringLiteral("外部轨迹文件已加载") :
                QStringLiteral("仿真轨迹文件已加载");
        }
        else{
            fileButtonText = isRealtimeMode ?
                QStringLiteral("导入外部轨迹文件") :
                QStringLiteral("导入仿真轨迹文件");
        }
        setButtonTextIfChanged(ui->mainReadTrajFileTrigger, fileButtonText);
    }

    if(ui->label_21){
        setLabelTextIfChanged(ui->label_21,
                              isSimulationMode ?
                                  QStringLiteral("仿真轨迹类型") :
                                  QStringLiteral("程序轨迹类型"));
    }
    if(ui->label_76){
        setLabelTextIfChanged(ui->label_76,
                              isRealtimeMode &&
                              isRealtimeManualPoseInputSelected() &&
                              ui->devUseCamForActPose->isChecked() ?
                                  QStringLiteral("当前位姿(mm)") :
                                  QStringLiteral("起点位置(mm)"));
    }
    if(ui->label_77){
        setLabelTextIfChanged(ui->label_77,
                              isRealtimeMode &&
                              isRealtimeManualPoseInputSelected() &&
                              ui->devUseCamForActPose->isChecked() ?
                                  QStringLiteral("当前姿态(rad)") :
                                  QStringLiteral("起点姿态(rad)"));
    }
    if(ui->label_70){
        setLabelTextIfChanged(ui->label_70,
                              isRealtimeMode ?
                                  QStringLiteral("目标位置(mm)") :
                                  QStringLiteral("终点位置(mm)"));
    }
    if(ui->label_74){
        setLabelTextIfChanged(ui->label_74,
                              isRealtimeMode ?
                                  QStringLiteral("目标姿态(rad)") :
                                  QStringLiteral("终点姿态(rad)"));
    }
    if(ui->label_75){
        setLabelTextIfChanged(ui->label_75, QStringLiteral("运行时间(s)"));
    }
    if(ui->label_202){
        setLabelTextIfChanged(ui->label_202, QStringLiteral("时间步长(s)"));
    }
    if(QLabel* label30 = findOptionalUiObject<QLabel>(this, "label_30")){
        setLabelTextIfChanged(label30, QStringLiteral("阻抗轨迹模式"));
    }

    QString primaryActionText;
    if(isProgramMode){
        if(robotState.positionMotionState == PositionMotionState::Simulating){
            primaryActionText = QStringLiteral("程序控制规划中...");
        }
        else if(robotState.positionMotionState == PositionMotionState::PausedPvt){
            primaryActionText = QStringLiteral("程序控制已暂停");
        }
        else if(robotState.positionMotionState == PositionMotionState::ExecutingPvt){
            primaryActionText = QStringLiteral("程序控制执行中...");
        }
        else{
            primaryActionText = QStringLiteral("启动程序控制");
        }
    }
    else if(isRealtimeMode){
        if(robotState.positionMotionState == PositionMotionState::Simulating){
            primaryActionText = QStringLiteral("实时轨迹规划中...");
        }
        else if(robotState.positionMotionState == PositionMotionState::PausedPvt){
            primaryActionText = QStringLiteral("实时轨迹已暂停");
        }
        else if(robotState.positionMotionState == PositionMotionState::ExecutingPvt){
            primaryActionText = QStringLiteral("实时轨迹执行中...");
        }
        else{
            primaryActionText = QStringLiteral("启动实时轨迹");
        }
    }
    else{
        primaryActionText = robotState.positionMotionState == PositionMotionState::Simulating ?
                    QStringLiteral("仿真计算进行中...") :
                    QStringLiteral("启动仿真计算");
    }
    setButtonTextIfChanged(ui->mainPosModeSwitch, primaryActionText);
    setButtonTextIfChanged(ui->mainPosModeSend, QStringLiteral("执行最近规划"));

    if(ui->mainHybridPoseForceModeSwitch){
        setEnabledIfChanged(ui->mainHybridPoseForceModeSwitch,
                            !isSimulationMode && !robotState.anyMotionRunning);
    }
    if(ui->mainHybridPoseForceCableA){
        setEnabledIfChanged(ui->mainHybridPoseForceCableA,
                            !isSimulationMode && !robotState.anyMotionRunning);
    }
    if(ui->mainHybridPoseForceCableB){
        setEnabledIfChanged(ui->mainHybridPoseForceCableB,
                            !isSimulationMode && !robotState.anyMotionRunning);
    }

    updatePoseInputUiState();
    refreshPrimaryOperationUiState(robotState);
    refreshUdpPacketSummaryUi();
    refreshCalibrationUiState();
    refreshSafetyUiStateForState(robotState);
    refreshConnectionStatusUi();
}

void MainWindow::refreshSafetyUiState()
{
    if(!ui){
        return;
    }
    refreshSafetyUiStateForState(currentRobotState(false));
}

void MainWindow::refreshSafetyUiStateForState(const RobotStateSnapshot& robotState)
{
    if(!ui){
        return;
    }

    auto setLabelTextIfChanged = [](QLabel* label, const QString& text){
        if(label && label->text() != text){
            label->setText(text);
        }
    };
    auto setLabelStyleIfChanged = [](QLabel* label, const QString& style){
        if(label && label->styleSheet() != style){
            label->setStyleSheet(style);
        }
    };
    auto setEnabledIfChanged = [](QWidget* widget, bool enabled){
        if(widget && widget->isEnabled() != enabled){
            widget->setEnabled(enabled);
        }
    };

    QString statusText;
    QString summaryText;
    QString detailText;
    QString statusStyle;

    if(runtimeState.safetyFaultLatched){
        if(runtimeState.safetyStopLevel >= static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop)){
            statusText = QStringLiteral("安全急停锁存");
            statusStyle = QStringLiteral(
                        "QLabel { color: #991b1b; background: #fee2e2; border: 1px solid #fca5a5; "
                        "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        }
        else if(runtimeState.safetyStopLevel >= static_cast<int>(SafetyMonitor::StopLevel::SafetyStop)){
            statusText = QStringLiteral("安全停机锁存");
            statusStyle = QStringLiteral(
                        "QLabel { color: #9a3412; background: #ffedd5; border: 1px solid #fdba74; "
                        "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        }
        else{
            statusText = QStringLiteral("受控停机锁存");
            statusStyle = QStringLiteral(
                        "QLabel { color: #92400e; background: #fef3c7; border: 1px solid #fcd34d; "
                        "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        }

        summaryText = runtimeState.safetyFaultSummary.isEmpty() ?
                    QStringLiteral("存在未复位的安全故障锁存") :
                    runtimeState.safetyFaultSummary;
        const QString detailPrefix = runtimeState.safetyFaultDetail.isEmpty() ?
                    QStringLiteral("请先检查张力、电机反馈、末端位姿和工作空间边界。") :
                    runtimeState.safetyFaultDetail;
        detailText = QStringLiteral("%1 请确认现场安全且运动已停止，再执行“安全复位”；若控制卡已连接，系统会先清除所有电机轴错误码。")
                .arg(detailPrefix);
    }
    else if(!ui->devUseLS->isChecked()){
        statusText = QStringLiteral("监控待机");
        statusStyle = QStringLiteral(
                    "QLabel { color: #475569; background: #e2e8f0; border: 1px solid #cbd5e1; "
                    "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        summaryText = QStringLiteral("当前未启用执行机构链路，未发现活动安全故障。");
        detailText = QStringLiteral("如需进行硬件闭环测试，请先启用执行机构并连接控制卡。");
    }
    else if(!robotState.lsConnected){
        statusText = QStringLiteral("待连接");
        statusStyle = QStringLiteral(
                    "QLabel { color: #1d4ed8; background: #dbeafe; border: 1px solid #93c5fd; "
                    "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        summaryText = QStringLiteral("尚未连接执行机构，未发现活动安全故障。");
        detailText = QStringLiteral("连接控制卡并上电后，安全监控将开始跟踪电机、张力和末端位姿。");
    }
    else if(!robotState.systemRunning){
        statusText = QStringLiteral("待上电");
        statusStyle = QStringLiteral(
                    "QLabel { color: #1d4ed8; background: #dbeafe; border: 1px solid #93c5fd; "
                    "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        summaryText = QStringLiteral("执行机构已连接，等待系统上电。");
        detailText = QStringLiteral("完成上电后可进入程序控制、实时轨迹或仿真链路。");
    }
    else if(robotState.anyMotionRunning){
        statusText = QStringLiteral("监控中");
        statusStyle = QStringLiteral(
                    "QLabel { color: #166534; background: #dcfce7; border: 1px solid #86efac; "
                    "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        summaryText = QStringLiteral("安全监控运行中，正在持续检查张力、电机、末端位姿与工作空间。");
        detailText = QStringLiteral("如发现故障将自动停机并锁存；如需人工中止，请使用“软件急停”。");
    }
    else{
        statusText = QStringLiteral("待命");
        statusStyle = QStringLiteral(
                    "QLabel { color: #166534; background: #dcfce7; border: 1px solid #86efac; "
                    "border-radius: 4px; padding: 4px 8px; font-weight: 600; }");
        summaryText = QStringLiteral("未发现活动安全故障，可开始规划、仿真或执行。");
        detailText = QStringLiteral("当前支持从此处查看锁存故障摘要，并执行安全复位或软件急停。");
    }

    if(ui->safetyStatusValueLabel){
        setLabelTextIfChanged(ui->safetyStatusValueLabel, statusText);
        setLabelStyleIfChanged(ui->safetyStatusValueLabel, statusStyle);
    }
    if(ui->safetyFaultSummaryLabel){
        setLabelTextIfChanged(ui->safetyFaultSummaryLabel, summaryText);
    }
    if(ui->safetyFaultDetailLabel){
        setLabelTextIfChanged(ui->safetyFaultDetailLabel, detailText);
    }
    if(ui->safetyResetButton){
        setEnabledIfChanged(ui->safetyResetButton, runtimeState.safetyFaultLatched);
    }
    if(ui->safetyEmergencyButton){
        setEnabledIfChanged(ui->safetyEmergencyButton, true);
    }
    refreshFaultInjectionUiControls();
}

void MainWindow::refreshFaultInjectionUiControls()
{
    if(!ui || !ui->faultInjectionGroupBox || !ui->faultInjectionCableCombo
            || !ui->faultInjectionMotorCombo || !ui->faultInjectionCableButton
            || !ui->faultInjectionMotorButton || !ui->faultInjectionPlcCommButton){
        return;
    }

    refreshAxisRuntimeCounts();
    const int axisCount = ui->devAxisNum ? std::max(0, ui->devAxisNum->value()) : 0;
    const int sensorCount = std::max(0, axisForceSensorNum);

    auto rebuildCombo = [](QComboBox* combo,
                           int count,
                           const QString& prefix,
                           const QString& emptyText,
                           int displayStartIndex){
        const QVariant previousData = combo->currentData();
        const int previousIndex = previousData.isValid() ? previousData.toInt() : -1;
        const QSignalBlocker blocker(combo);
        combo->clear();
        if(count <= 0){
            combo->addItem(emptyText, QVariant());
            return;
        }
        for(int i=0; i<count; ++i){
            combo->addItem(QStringLiteral("%1%2").arg(prefix).arg(displayStartIndex + i), i);
        }
        const int restoredIndex = combo->findData(previousIndex);
        combo->setCurrentIndex(restoredIndex >= 0 ? restoredIndex : 0);
    };

    if(axisCount != faultInjectionCachedAxisCount){
        rebuildCombo(ui->faultInjectionMotorCombo,
                     axisCount,
                     QStringLiteral("电机轴"),
                     QStringLiteral("未配置电机轴"),
                     0);
        faultInjectionCachedAxisCount = axisCount;
    }
    if(sensorCount != faultInjectionCachedSensorCount){
        rebuildCombo(ui->faultInjectionCableCombo,
                     sensorCount,
                     QStringLiteral("绳索/张力通道"),
                     QStringLiteral("未配置张力通道"),
                     1);
        faultInjectionCachedSensorCount = sensorCount;
    }

    const bool canInject = !runtimeState.safetyFaultLatched;
    ui->faultInjectionCableCombo->setEnabled(sensorCount > 0 && canInject);
    ui->faultInjectionMotorCombo->setEnabled(axisCount > 0 && canInject);
    ui->faultInjectionCableButton->setEnabled(sensorCount > 0 && canInject);
    ui->faultInjectionMotorButton->setEnabled(axisCount > 0 && canInject);
    ui->faultInjectionPlcCommButton->setEnabled(canInject);
}

std::vector<double> MainWindow::buildExpectedForceFromUi() const{
    std::vector<double> expected(axisForceSensorNum, 0.0);
    for(int i=0; i<axisForceSensorNum && i<static_cast<int>(mainForceSensorExp.size()); ++i){
        expected[i] = mainForceSensorExp[i]->value();
    }
    return expected;
}

std::vector<double> MainWindow::mapGcCableForceToSensorExpected(const std::vector<double>& gcCableForce) const{
    std::vector<double> expected = buildExpectedForceFromUi();
    if(static_cast<int>(expected.size()) < axisForceSensorNum){
        expected.resize(axisForceSensorNum, 0.0);
    }

    int tmp = 0;
    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
            if(axisSensorIndexVec[axisIndex]->value() >= 0 && axisEndVec[axisIndex]->value() == endIndex){
                const int sensorIndex = axisSensorIndexVec[axisIndex]->value();
                if(tmp < static_cast<int>(gcCableForce.size()) && sensorIndex < axisForceSensorNum){
                    expected[sensorIndex] = gcCableForce[tmp];
                }
                tmp++;
            }
        }
    }
    return expected;
}

ControlWorker::Config MainWindow::buildControlWorkerConfig() const{
    ControlWorker::Config config;
    config.axisCount = ui->devAxisNum->value();
    config.sensorCount = axisForceSensorNum;
    config.ctrlCycleMs = ui->devCtrlCycleMs->value();
    config.sensorSampleHz = configuredSensorSampleFrequencyHz();
    config.forceSensorLowPassCutoffHz = configuredForceSensorLowPassCutoffHz();
    config.systemRunning = runtimeState.systemRunning;
    config.useLeadshine = ui->devUseLS->isChecked();
    config.forceThreadEnabled = ui->mainFCThread->isChecked();
    config.usePid = ui->devUsePID->isChecked();
    config.forcePidOutputMode =
            optionalComboCurrentDataInt(this, "devForceControlOutputMode", 0) == 1 ?
                ControlWorker::ForcePidOutputMode::Torque :
                ControlWorker::ForcePidOutputMode::Velocity;
    config.pvtActiveOrPaused = runtimeState.pvtCommandActive;
    config.actualTorqueRealtimeEnabled = ui->mainMotorTorqueRealtimeButton->isChecked();
    config.actualTorqueLimitEnabled = ui->motorTorqueLimitEnable->isChecked();
    config.actualTorqueLimitNm = ui->motorTorqueLimitNm->value();
    config.initForce = ui->devFCInitForce->value();
    config.forcePidDeadbandRatio = ui->devFCDeadbandPercent->value() / 100.0;
    config.forceTorqueCommandLimitNm = optionalSpinBoxValue(this, "devFCTorqueLimitNm", 2.0);
    config.forceTorqueCommandSlewRateNmPerSec =
            optionalSpinBoxValue(this, "devFCTorqueSlewNmPerSec", 20.0);
    config.forceTorqueVelocityDampingNmPerVelocity =
            optionalSpinBoxValue(this, "devFCTorqueDampingNmPerVelocity", 0.0);
    config.forceSensorHome = forceSensorCurHome;
    config.expectedForce = buildExpectedForceFromUi();
    config.pidP = dynPID_P.empty() ? orgPID_P : dynPID_P;
    config.pidI = dynPID_I.empty() ? orgPID_I : dynPID_I;
    config.pidD = dynPID_D.empty() ? orgPID_D : dynPID_D;
    config.axes.resize(config.axisCount);
    HybridPoseForceModeConfig hybridModeConfig;
    QString hybridModeError;
    const bool hybridForceControlActive = runtimeState.hybridPoseForceModeActive &&
            buildHybridPoseForceModeConfig(hybridModeConfig, hybridModeError) &&
            hybridModeConfig.enabled;

    for(int axisIndex=0; axisIndex<config.axisCount; ++axisIndex){
        ControlWorker::AxisConfig axis;
        axis.isMotorAxis = isMotorAxis(axisIndex);
        axis.sensorIndex = axisSensorIndexVec[axisIndex]->value();
        if(hybridForceControlActive){
            axis.forceControlEnabled = std::find(hybridModeConfig.forceAxisIndex.begin(),
                                                 hybridModeConfig.forceAxisIndex.end(),
                                                 axisIndex) != hybridModeConfig.forceAxisIndex.end();
        }
        else{
            axis.forceControlEnabled = axis.sensorIndex >= 0 &&
                    axis.sensorIndex < static_cast<int>(useFCBtnVec.size()) &&
                    useFCBtnVec[axis.sensorIndex]->isChecked();
        }
        axis.motorCof = axisMotorCofVec[axisIndex]->value();
        axis.forceMax = axisForceMaxVec[axisIndex]->value();
        axis.motorMin = axisMotorMinVec[axisIndex]->value();
        axis.motorMax = axisMotorMaxVec[axisIndex]->value();
        axis.motorVelMax = axisMotorVelMaxVec[axisIndex]->value();
        config.axes[axisIndex] = axis;
    }

    return config;
}

void MainWindow::updateControlWorkerConfig(){
    refreshHybridPoseForceSelectionUi();
    syncHybridPoseForceExpectedForceForActiveTrajectory();
    syncSingleCableForceDebugPretensionForce();
    syncMotorSoftwareLimitsToHardware();
    const int forceSensorTraceSamplePeriodUs = 500;
    if(forceSensorTraceSamplePeriodUs != lastAppliedForceSensorTraceSamplePeriodUs){
        hardwareInterface.setForceSensorTraceSamplePeriodUs(forceSensorTraceSamplePeriodUs);
        lastAppliedForceSensorTraceSamplePeriodUs = forceSensorTraceSamplePeriodUs;
    }
    if(controlWorker){
        controlWorker->setConfig(buildControlWorkerConfig());
    }
}

void MainWindow::applyMotorPositionSnapshot(const std::vector<double>& absPos,
                                            const std::vector<double>& relRawPos){
    const int axisCount = std::min(static_cast<int>(absPos.size()), ui->devAxisNum->value());
    std::vector<double> motorHome;
    if(static_cast<int>(relRawPos.size()) < axisCount){
        motorHome = hardwareInterface.getAllMotorHome();
    }
    motorCurAbsPos.assign(absPos.begin(), absPos.begin() + axisCount);
    motorCurPosRaw.clear();
    motorCurPos.clear();
    motorCurPosRaw.reserve(axisCount);
    motorCurPos.reserve(axisCount);

    for(int i=0; i<axisCount; ++i){
        double relPos = i < static_cast<int>(relRawPos.size()) ? relRawPos[i] : absPos[i];
        if(i >= static_cast<int>(relRawPos.size()) && i < static_cast<int>(motorHome.size())){
            relPos -= motorHome[i];
        }
        motorCurPosRaw.push_back(relPos);
        motorCurPos.push_back(relPos);
    }

    for(int i=0; i<axisCount; ++i){
        motorCurPos[i] = convertMotorFeedbackToCableValue(i, motorCurPos[i]);
    }

    const int singleMotorIndex = selectedSingleMotorIndex();
    if(singleMotorIndex >= 0 && singleMotorIndex < static_cast<int>(motorCurPosRaw.size())){
        ui->SinglemotorActpos->setValue(motorCurPosRaw[singleMotorIndex]);
    }
}

void MainWindow::applyControlSnapshot(){
    if(!controlWorker){
        return;
    }

    static quint64 lastSequence = 0;
    const ControlWorker::Snapshot snapshot = controlWorker->latestSnapshot();
    if(snapshot.sequence == 0 || snapshot.sequence == lastSequence){
        return;
    }
    lastSequence = snapshot.sequence;

    if(!snapshot.motorAbsPos.empty()){
        applyMotorPositionSnapshot(snapshot.motorAbsPos, snapshot.motorRelRawPos);
        refreshMotorPosDisplay();
        processNoCamCableLengthFromMotorSnapshot();
        updateForwardKinematicsPoseFromMotorSnapshot();
    }

    if(!snapshot.forceSensorValue.empty()){
        const int sensorCount = std::min(axisForceSensorNum, static_cast<int>(snapshot.forceSensorValue.size()));
        for(int i=0; i<sensorCount; ++i){
            mainForceSensorDataVal[i] = snapshot.forceSensorValue[i];
            if(i < static_cast<int>(mainForceSensorData.size())){
                mainForceSensorData[i]->setValue(snapshot.forceSensorValue[i]);
            }
        }
    }

    if(snapshot.forceExpectedFromExternal || snapshot.forceThreadRunning){
        const int sensorCount = std::min(axisForceSensorNum, static_cast<int>(snapshot.expectedForce.size()));
        for(int i=0; i<sensorCount && i<static_cast<int>(mainForceSensorExp.size()); ++i){
            mainForceSensorExp[i]->setValue(snapshot.expectedForce[i]);
        }
    }

    if(ui->mainMotorTorqueRealtimeButton->isChecked() && !snapshot.motorTorqueNm.empty()){
        const int sensorDisplayCount = std::min(axisForceSensorNum, static_cast<int>(mainMotorTorqueData.size()));
        for(int sensorIndex=0; sensorIndex<sensorDisplayCount; ++sensorIndex){
            int mappedAxisIndex = -1;
            for(int axisIndex=0; axisIndex<ui->devAxisNum->value() &&
                axisIndex<static_cast<int>(axisSensorIndexVec.size()); ++axisIndex){
                if(isMotorAxis(axisIndex) &&
                        axisSensorIndexVec[axisIndex] &&
                        axisSensorIndexVec[axisIndex]->value() == sensorIndex){
                    mappedAxisIndex = axisIndex;
                    break;
                }
            }
            if(mappedAxisIndex < 0 && sensorIndex < static_cast<int>(snapshot.motorTorqueNm.size())){
                mappedAxisIndex = sensorIndex;
            }

            double torqueNm = 0.0;
            if(mappedAxisIndex >= 0 && mappedAxisIndex < static_cast<int>(snapshot.motorTorqueNm.size()) &&
                    std::isfinite(snapshot.motorTorqueNm[mappedAxisIndex])){
                torqueNm = snapshot.motorTorqueNm[mappedAxisIndex];
            }
            mainMotorTorqueData[sensorIndex]->setValue(torqueNm);
        }
    }

    updateVisualizationFromControlSnapshot(snapshot);
    captureSessionRecordingSample(snapshot);
}

void MainWindow::setupDataVisualizationTab()
{
    if(!ui){
        return;
    }

    dataVizTab = ui->dataVizTab;
    motorControlPlotHost = ui->motorControlPlotHost;
    cableTensionPlotHost = ui->cableTensionPlotHost;
    endEffectorPosPlotHost = ui->endEffectorPosPlotHost;
    endEffectorVelPlotHost = ui->endEffectorVelPlotHost;
    endEffectorAccPlotHost = ui->endEffectorAccPlotHost;
    cableSpeedPlotHost = ui->cableSpeedPlotHost;
    cableLengthPlotHost = ui->cableLengthPlotHost;

    if(!curveDrawer){
        curveDrawer = new CurveDrawer(this);
    }

    if(ui->dataVizClearButton){
        disconnect(ui->dataVizClearButton,
                   &QPushButton::clicked,
                   this,
                   &MainWindow::clearVisualizationData);
        connect(ui->dataVizClearButton,
                &QPushButton::clicked,
                this,
                &MainWindow::clearVisualizationData);
    }

    reinitializeDataVisualizationPlots();
}

void MainWindow::setupSingleMotorTrajectoryTrackingTab()
{
    if(!ui || singleMotorTrajectoryTab){
        return;
    }

    singleMotorTrajectoryTab = findOptionalUiObject<QWidget>(this, "singleMotorTrajectoryTrackingTab");
    if(!singleMotorTrajectoryTab){
        return;
    }

    singleMotorTrajectoryAxisCombo = findOptionalUiObject<QComboBox>(this, "singleMotorTrajAxisCombo");
    singleMotorTrajectoryTypeCombo = findOptionalUiObject<QComboBox>(this, "singleMotorTrajTypeCombo");
    singleMotorTrajectoryUseCurrentStartCheckBox = findOptionalUiObject<QCheckBox>(this, "singleMotorTrajUseCurrentStartCheckBox");
    singleMotorTrajectoryStartSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajStartPosSpin");
    singleMotorTrajectoryEndSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajEndPosSpin");
    singleMotorTrajectoryCenterSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajCenterPosSpin");
    singleMotorTrajectoryAmplitudeSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajAmplitudeSpin");
    singleMotorTrajectoryCyclesSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajCyclesSpin");
    singleMotorTrajectoryPhaseSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajPhaseSpin");
    singleMotorTrajectoryDurationSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajDurationSpin");
    singleMotorTrajectoryPlanStepSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajPlanStepSpin");
    singleMotorTrajectoryVelocityLimitSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajVelocityLimitSpin");
    singleMotorTrajectoryCurrentPosSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajCurrentPosSpin");
    singleMotorTrajectoryMaxErrorSpin = findOptionalUiObject<QDoubleSpinBox>(this, "singleMotorTrajMaxErrorSpin");
    singleMotorTrajectoryEnableButton = findOptionalUiObject<QPushButton>(this, "singleMotorTrajEnableButton");
    singleMotorTrajectoryStartButton = findOptionalUiObject<QPushButton>(this, "singleMotorTrajStartButton");
    singleMotorTrajectoryStopButton = findOptionalUiObject<QPushButton>(this, "singleMotorTrajStopButton");
    singleMotorTrajectoryExportButton = findOptionalUiObject<QPushButton>(this, "singleMotorTrajExportButton");
    singleMotorTrajectoryClearButton = findOptionalUiObject<QPushButton>(this, "singleMotorTrajClearButton");
    singleMotorTrajectoryStatusLabel = findOptionalUiObject<QLabel>(this, "singleMotorTrajStatusLabel");
    singleMotorTrajectoryTrackPlotHost = findOptionalUiObject<QCustomPlot>(this, "singleMotorTrajTrackPlotHost");
    singleMotorTrajectoryErrorPlotHost = findOptionalUiObject<QCustomPlot>(this, "singleMotorTrajErrorPlotHost");

    if(!singleMotorTrajectoryAxisCombo ||
            !singleMotorTrajectoryTypeCombo ||
            !singleMotorTrajectoryStartSpin ||
            !singleMotorTrajectoryEndSpin ||
            !singleMotorTrajectoryDurationSpin ||
            !singleMotorTrajectoryPlanStepSpin ||
            !singleMotorTrajectoryStartButton ||
            !singleMotorTrajectoryStopButton){
        displayInfo("错误：单电机轨迹精度UI未完整加载", "error");
        return;
    }

    if(singleMotorTrajectoryTypeCombo){
        QSignalBlocker blocker(singleMotorTrajectoryTypeCombo);
        singleMotorTrajectoryTypeCombo->clear();
        singleMotorTrajectoryTypeCombo->addItem(QStringLiteral("直线"),
                                                static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Line));
        singleMotorTrajectoryTypeCombo->addItem(QStringLiteral("正弦"),
                                                static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Sine));
        singleMotorTrajectoryTypeCombo->addItem(QStringLiteral("三角波"),
                                                static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Triangle));
        singleMotorTrajectoryTypeCombo->addItem(QStringLiteral("阶跃"),
                                                static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Step));
    }

    const auto setWideRange = [](QDoubleSpinBox* spin){
        if(!spin){
            return;
        }
        spin->setRange(-100000000.0, 100000000.0);
        spin->setDecimals(6);
        spin->setKeyboardTracking(false);
    };
    setWideRange(singleMotorTrajectoryStartSpin);
    setWideRange(singleMotorTrajectoryEndSpin);
    setWideRange(singleMotorTrajectoryCenterSpin);
    setWideRange(singleMotorTrajectoryAmplitudeSpin);
    setWideRange(singleMotorTrajectoryCurrentPosSpin);
    setWideRange(singleMotorTrajectoryMaxErrorSpin);
    if(singleMotorTrajectoryCurrentPosSpin){
        singleMotorTrajectoryCurrentPosSpin->setReadOnly(true);
        singleMotorTrajectoryCurrentPosSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    }
    if(singleMotorTrajectoryMaxErrorSpin){
        singleMotorTrajectoryMaxErrorSpin->setReadOnly(true);
        singleMotorTrajectoryMaxErrorSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    }
    if(singleMotorTrajectoryPlanStepSpin){
        singleMotorTrajectoryPlanStepSpin->setDecimals(3);
        singleMotorTrajectoryPlanStepSpin->setRange(1.0, 1000.0);
        singleMotorTrajectoryPlanStepSpin->setSuffix(QStringLiteral(" ms"));
        singleMotorTrajectoryPlanStepSpin->setKeyboardTracking(false);
    }

    const auto refreshParameterState = [this](){
        const int typeData = singleMotorTrajectoryTypeCombo ?
                    singleMotorTrajectoryTypeCombo->currentData(Qt::UserRole).toInt() :
                    static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Line);
        const bool lineOrStep =
                typeData == static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Line) ||
                typeData == static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Step);
        const bool periodic =
                typeData == static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Sine) ||
                typeData == static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Triangle);
        const bool useCurrentStart =
                singleMotorTrajectoryUseCurrentStartCheckBox &&
                singleMotorTrajectoryUseCurrentStartCheckBox->isChecked();
        if(singleMotorTrajectoryStartSpin){
            singleMotorTrajectoryStartSpin->setEnabled(lineOrStep && !useCurrentStart);
        }
        if(singleMotorTrajectoryEndSpin){
            singleMotorTrajectoryEndSpin->setEnabled(lineOrStep);
        }
        if(singleMotorTrajectoryCenterSpin){
            singleMotorTrajectoryCenterSpin->setEnabled(periodic && !useCurrentStart);
        }
        if(singleMotorTrajectoryAmplitudeSpin){
            singleMotorTrajectoryAmplitudeSpin->setEnabled(periodic);
        }
        if(singleMotorTrajectoryCyclesSpin){
            singleMotorTrajectoryCyclesSpin->setEnabled(periodic);
        }
        if(singleMotorTrajectoryPhaseSpin){
            singleMotorTrajectoryPhaseSpin->setEnabled(periodic);
        }
    };

    connect(singleMotorTrajectoryTypeCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [refreshParameterState](int){ refreshParameterState(); });
    if(singleMotorTrajectoryUseCurrentStartCheckBox){
        connect(singleMotorTrajectoryUseCurrentStartCheckBox,
                &QCheckBox::toggled,
                this,
                [refreshParameterState](bool){ refreshParameterState(); });
    }

    connect(singleMotorTrajectoryAxisCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int){
        updateSingleMotorTrajectoryWorkerAxis();
    });
    if(singleMotorTrajectoryEnableButton){
        connect(singleMotorTrajectoryEnableButton, &QPushButton::clicked, this, [this](){
            const int axisIndex = selectedSingleMotorTrajectoryAxisIndex();
            if(axisIndex < 0){
                displayInfo("错误：单电机轨迹测试电机轴号无效", "error");
                return;
            }
            if(!ui->devUseLS->isChecked()){
                displayInfo("错误：当前仅支持雷赛单电机轨迹测试", "error");
                return;
            }
            if(!hardwareInterface.isLSConnected()){
                displayInfo("错误：请先连接雷赛控制卡", "error");
                return;
            }
            if(!applyLeadshineHardwareConfigFromUi()){
                displayInfo("错误：单电机轨迹测试使能失败，雷赛轴参数未正确配置", "error");
                return;
            }
            disableForceControlForManualMotorTest();
            updateControlWorkerConfig();
            if(!hardwareInterface.setMotorEnable(axisIndex, true)){
                displayInfo("错误：单电机轨迹测试电机使能失败", "error");
                return;
            }
            displayInfo(QString("单电机轨迹测试：电机轴%1已使能").arg(axisIndex).toStdString());
        });
    }
    disconnect(singleMotorTrajectoryStartButton,
               &QPushButton::clicked,
               this,
               &MainWindow::startSingleMotorTrajectory);
    connect(singleMotorTrajectoryStartButton,
            &QPushButton::clicked,
            this,
            &MainWindow::startSingleMotorTrajectory);
    connect(singleMotorTrajectoryStopButton, &QPushButton::clicked, this, [this](){
        stopSingleMotorTrajectory(QStringLiteral("single_motor_trajectory_stop_button"));
    });
    singleMotorTrajectoryStopButton->setEnabled(false);
    if(singleMotorTrajectoryExportButton){
        disconnect(singleMotorTrajectoryExportButton,
                   &QPushButton::clicked,
                   this,
                   &MainWindow::exportSingleMotorTrajectoryData);
        connect(singleMotorTrajectoryExportButton,
                &QPushButton::clicked,
                this,
                &MainWindow::exportSingleMotorTrajectoryData);
    }
    if(singleMotorTrajectoryClearButton){
        disconnect(singleMotorTrajectoryClearButton,
                   &QPushButton::clicked,
                   this,
                   &MainWindow::clearSingleMotorTrajectoryPlots);
        connect(singleMotorTrajectoryClearButton,
                &QPushButton::clicked,
                this,
                &MainWindow::clearSingleMotorTrajectoryPlots);
    }
    for(QComboBox* axisType : axisTypeVec){
        if(axisType){
            connect(axisType,
                    QOverload<int>::of(&QComboBox::currentIndexChanged),
                    this,
                    [this](int){
                refreshSingleMotorTrajectoryAxisOptions();
            });
        }
    }
    for(QSpinBox* hardwareAxisSpin : axisMotorHardwareAxisVec){
        if(hardwareAxisSpin){
            connect(hardwareAxisSpin,
                    QOverload<int>::of(&QSpinBox::valueChanged),
                    this,
                    [this](int){
                refreshSingleMotorTrajectoryAxisOptions();
            });
        }
    }
    for(QSpinBox* slaveIdSpin : axisMotorSlaveIdVec){
        if(slaveIdSpin){
            connect(slaveIdSpin,
                    QOverload<int>::of(&QSpinBox::valueChanged),
                    this,
                    [this](int){
                refreshSingleMotorTrajectoryAxisOptions();
            });
        }
    }

    setupSingleMotorTrajectoryPlots();
    refreshSingleMotorTrajectoryUnitUi();
    refreshSingleMotorTrajectoryAxisOptions();
    refreshParameterState();
    startSingleMotorTrajectoryThread();
    updateSingleMotorTrajectoryWorkerAxis();
}

void MainWindow::startSingleMotorTrajectoryThread()
{
    if(singleMotorTrajectoryThread || singleMotorTrajectoryWorker){
        return;
    }

    singleMotorTrajectoryThread = new QThread(this);
    singleMotorTrajectoryThread->setObjectName(QStringLiteral("SingleMotorTrajectoryThread"));
    singleMotorTrajectoryWorker = new SingleMotorTrajectoryWorker(&hardwareInterface);
    singleMotorTrajectoryWorker->moveToThread(singleMotorTrajectoryThread);
    connect(singleMotorTrajectoryThread, &QThread::started, singleMotorTrajectoryWorker, &SingleMotorTrajectoryWorker::start);
    connect(singleMotorTrajectoryThread, &QThread::finished, singleMotorTrajectoryWorker, &QObject::deleteLater);
    connect(singleMotorTrajectoryWorker, &SingleMotorTrajectoryWorker::displayInfoSignal, this, &MainWindow::displayInfo);
    connect(singleMotorTrajectoryWorker,
            &SingleMotorTrajectoryWorker::feedbackUpdated,
            this,
            &MainWindow::handleSingleMotorTrajectoryFeedback,
            Qt::QueuedConnection);
    connect(singleMotorTrajectoryWorker,
            &SingleMotorTrajectoryWorker::trajectoryPrepared,
            this,
            &MainWindow::handleSingleMotorTrajectoryPrepared,
            Qt::QueuedConnection);
    connect(singleMotorTrajectoryWorker,
            &SingleMotorTrajectoryWorker::sampleUpdated,
            this,
            &MainWindow::handleSingleMotorTrajectorySample,
            Qt::QueuedConnection);
    connect(singleMotorTrajectoryWorker,
            &SingleMotorTrajectoryWorker::trajectoryStateChanged,
            this,
            &MainWindow::handleSingleMotorTrajectoryStateChanged,
            Qt::QueuedConnection);
    singleMotorTrajectoryThread->start(QThread::HighPriority);
}

void MainWindow::stopSingleMotorTrajectoryThread()
{
    runtimeState.singleMotorTrajectoryActive = false;
    singleMotorTrajectoryRunning = false;
    if(singleMotorTrajectoryWorker){
        if(singleMotorTrajectoryThread && singleMotorTrajectoryThread->isRunning()){
            QMetaObject::invokeMethod(singleMotorTrajectoryWorker,
                                      "stop",
                                      Qt::BlockingQueuedConnection);
        }
        else{
            singleMotorTrajectoryWorker->stop();
        }
    }
    if(singleMotorTrajectoryThread){
        singleMotorTrajectoryThread->quit();
        if(!singleMotorTrajectoryThread->wait(1000)){
            singleMotorTrajectoryThread->terminate();
            singleMotorTrajectoryThread->wait(500);
        }
        delete singleMotorTrajectoryThread;
        singleMotorTrajectoryThread = nullptr;
    }
    singleMotorTrajectoryWorker = nullptr;
}

void MainWindow::refreshSingleMotorTrajectoryAxisOptions()
{
    if(!singleMotorTrajectoryAxisCombo){
        return;
    }

    const int previousAxis = selectedSingleMotorTrajectoryAxisIndex();
    QSignalBlocker blocker(singleMotorTrajectoryAxisCombo);
    singleMotorTrajectoryAxisCombo->clear();
    const int axisCount = ui ? ui->devAxisNum->value() : 0;
    for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
        if(!isMotorAxis(axisIndex)){
            continue;
        }
        QString axisText = QStringLiteral("逻辑轴%1").arg(axisIndex);
        if(axisIndex < static_cast<int>(axisMotorHardwareAxisVec.size()) &&
                axisMotorHardwareAxisVec[axisIndex]){
            axisText += QStringLiteral(" / 控制卡轴%1").arg(axisMotorHardwareAxisVec[axisIndex]->value());
        }
        if(axisIndex < static_cast<int>(axisMotorSlaveIdVec.size()) &&
                axisMotorSlaveIdVec[axisIndex] &&
                axisMotorSlaveIdVec[axisIndex]->value() > 0){
            axisText += QStringLiteral(" / 从站%1").arg(axisMotorSlaveIdVec[axisIndex]->value());
        }
        singleMotorTrajectoryAxisCombo->addItem(axisText, axisIndex);
    }

    int targetIndex = -1;
    if(previousAxis >= 0){
        targetIndex = singleMotorTrajectoryAxisCombo->findData(previousAxis, Qt::UserRole);
    }
    if(targetIndex < 0 && singleMotorTrajectoryAxisCombo->count() > 0){
        targetIndex = 0;
    }
    singleMotorTrajectoryAxisCombo->setCurrentIndex(targetIndex);
    const bool hasAxis = targetIndex >= 0;
    singleMotorTrajectoryAxisCombo->setEnabled(hasAxis);
    if(singleMotorTrajectoryEnableButton){
        singleMotorTrajectoryEnableButton->setEnabled(hasAxis);
    }
    if(singleMotorTrajectoryStartButton){
        singleMotorTrajectoryStartButton->setEnabled(hasAxis && !singleMotorTrajectoryRunning);
    }
    if(singleMotorTrajectoryExportButton){
        singleMotorTrajectoryExportButton->setEnabled(hasAxis);
    }
    updateSingleMotorTrajectoryWorkerAxis();
}

int MainWindow::selectedSingleMotorTrajectoryAxisIndex() const
{
    if(!singleMotorTrajectoryAxisCombo || singleMotorTrajectoryAxisCombo->currentIndex() < 0){
        return -1;
    }
    return singleMotorTrajectoryAxisCombo->currentData(Qt::UserRole).toInt();
}

void MainWindow::updateSingleMotorTrajectoryWorkerAxis()
{
    if(!singleMotorTrajectoryWorker){
        return;
    }
    const int axisIndex = selectedSingleMotorTrajectoryAxisIndex();
    QMetaObject::invokeMethod(singleMotorTrajectoryWorker, [worker = singleMotorTrajectoryWorker, axisIndex](){
        worker->setSelectedAxis(axisIndex);
    }, Qt::QueuedConnection);
}

void MainWindow::refreshSingleMotorTrajectoryUnitUi()
{
    const MotorFeedbackDisplayUnit unit = currentMotorFeedbackDisplayUnit();
    const QString positionSuffix = unit == MotorFeedbackDisplayUnit::Degree ?
                QStringLiteral(" deg") :
                QStringLiteral(" rev");
    const QString velocitySuffix = unit == MotorFeedbackDisplayUnit::Degree ?
                QStringLiteral(" deg/s") :
                QStringLiteral(" rev/s");
    for(QDoubleSpinBox* spin : {
        singleMotorTrajectoryStartSpin,
        singleMotorTrajectoryEndSpin,
        singleMotorTrajectoryCenterSpin,
        singleMotorTrajectoryAmplitudeSpin,
        singleMotorTrajectoryCurrentPosSpin
    }){
        if(spin){
            spin->setSuffix(positionSuffix);
        }
    }
    if(singleMotorTrajectoryMaxErrorSpin){
        singleMotorTrajectoryMaxErrorSpin->setSuffix(QStringLiteral(" deg"));
    }
    if(singleMotorTrajectoryVelocityLimitSpin){
        singleMotorTrajectoryVelocityLimitSpin->setSuffix(velocitySuffix);
    }
}

double MainWindow::singleMotorTrajectoryPositionToDegrees(double position) const
{
    if(!std::isfinite(position)){
        return std::numeric_limits<double>::quiet_NaN();
    }
    return currentMotorFeedbackDisplayUnit() == MotorFeedbackDisplayUnit::Degree ?
                position :
                position * 360.0;
}

double MainWindow::singleMotorTrajectoryRawFollowingErrorToDegrees(int axisIndex, int rawError) const
{
    const double equiv = configuredLeadshineAxisEquivForAxis(axisIndex);
    if(equiv <= 1e-12 || !std::isfinite(equiv)){
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double valueInCurrentUnit = static_cast<double>(rawError) / equiv;
    return singleMotorTrajectoryPositionToDegrees(valueInCurrentUnit);
}

void MainWindow::setupSingleMotorTrajectoryPlots()
{
    if(singleMotorTrajectoryTrackPlotHost){
        singleMotorTrajectoryTrackPlotHost->clearGraphs();
        singleMotorTrajectoryTrackPlotHost->legend->setVisible(true);
        singleMotorTrajectoryTrackPlotHost->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
        singleMotorTrajectoryTrackPlotHost->axisRect()->setupFullAxesBox();
        singleMotorTrajectoryTrackPlotHost->xAxis->setLabel(QStringLiteral("时间 (s)"));
        singleMotorTrajectoryTrackPlotHost->yAxis->setLabel(QStringLiteral("位置 (deg)"));
        singleMotorTrajectoryTrackPlotHost->addGraph();
        singleMotorTrajectoryTrackPlotHost->graph(0)->setName(QStringLiteral("电机接收指令位置"));
        singleMotorTrajectoryTrackPlotHost->graph(0)->setPen(QPen(QColor(232, 142, 0), 1.5, Qt::DashLine));
        singleMotorTrajectoryTrackPlotHost->addGraph();
        singleMotorTrajectoryTrackPlotHost->graph(1)->setName(QStringLiteral("实际反馈"));
        singleMotorTrajectoryTrackPlotHost->graph(1)->setPen(QPen(QColor(204, 51, 51), 1.5));
        singleMotorTrajectoryTrackPlotHost->xAxis->setRange(0.0, 10.0);
        singleMotorTrajectoryTrackPlotHost->yAxis->setRange(-1.0, 1.0);
    }
    if(singleMotorTrajectoryErrorPlotHost){
        singleMotorTrajectoryErrorPlotHost->clearGraphs();
        singleMotorTrajectoryErrorPlotHost->legend->setVisible(true);
        singleMotorTrajectoryErrorPlotHost->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
        singleMotorTrajectoryErrorPlotHost->axisRect()->setupFullAxesBox();
        singleMotorTrajectoryErrorPlotHost->xAxis->setLabel(QStringLiteral("时间 (s)"));
        singleMotorTrajectoryErrorPlotHost->yAxis->setLabel(QStringLiteral("误差 (deg)"));
        singleMotorTrajectoryErrorPlotHost->addGraph();
        singleMotorTrajectoryErrorPlotHost->graph(0)->setName(QStringLiteral("Trace指令-实际"));
        singleMotorTrajectoryErrorPlotHost->graph(0)->setPen(QPen(QColor(0, 153, 102), 1.5));
        singleMotorTrajectoryErrorPlotHost->addGraph();
        singleMotorTrajectoryErrorPlotHost->graph(1)->setName(QStringLiteral("SDO 60F4跟踪误差"));
        singleMotorTrajectoryErrorPlotHost->graph(1)->setPen(QPen(QColor(126, 87, 194), 1.5, Qt::DashLine));
        singleMotorTrajectoryErrorPlotHost->xAxis->setRange(0.0, 10.0);
        singleMotorTrajectoryErrorPlotHost->yAxis->setRange(-1.0, 1.0);
    }
}

void MainWindow::clearSingleMotorTrajectoryPlots()
{
    if(singleMotorTrajectoryTrackPlotHost){
        for(int i=0; i<singleMotorTrajectoryTrackPlotHost->graphCount(); ++i){
            singleMotorTrajectoryTrackPlotHost->graph(i)->data()->clear();
        }
        singleMotorTrajectoryTrackPlotHost->xAxis->setRange(0.0, std::max(1.0, singleMotorTrajectoryDurationSec));
        singleMotorTrajectoryTrackPlotHost->yAxis->setRange(-1.0, 1.0);
        singleMotorTrajectoryTrackPlotHost->replot();
    }
    if(singleMotorTrajectoryErrorPlotHost){
        for(int i=0; i<singleMotorTrajectoryErrorPlotHost->graphCount(); ++i){
            singleMotorTrajectoryErrorPlotHost->graph(i)->data()->clear();
        }
        singleMotorTrajectoryErrorPlotHost->xAxis->setRange(0.0, std::max(1.0, singleMotorTrajectoryDurationSec));
        singleMotorTrajectoryErrorPlotHost->yAxis->setRange(-1.0, 1.0);
        singleMotorTrajectoryErrorPlotHost->replot();
    }
    singleMotorTrajectorySampleTime.clear();
    singleMotorTrajectoryExpected.clear();
    singleMotorTrajectoryCommand.clear();
    singleMotorTrajectoryActual.clear();
    singleMotorTrajectoryError.clear();
    singleMotorTrajectoryDriveError.clear();
    if(singleMotorTrajectoryMaxErrorSpin){
        QSignalBlocker blocker(singleMotorTrajectoryMaxErrorSpin);
        singleMotorTrajectoryMaxErrorSpin->setValue(0.0);
    }
}

void MainWindow::startSingleMotorTrajectory()
{
    const int axisIndex = selectedSingleMotorTrajectoryAxisIndex();
    if(axisIndex < 0){
        displayInfo("错误：单电机轨迹测试电机轴号无效", "error");
        return;
    }
    if(!ui->devUseLS->isChecked()){
        displayInfo("错误：当前仅支持雷赛单电机轨迹测试", "error");
        return;
    }
    if(!hardwareInterface.isLSConnected()){
        displayInfo("错误：请先连接雷赛控制卡", "error");
        return;
    }
    if(!applyLeadshineHardwareConfigFromUi()){
        displayInfo("错误：单电机轨迹测试启动失败，雷赛轴参数未正确配置", "error");
        return;
    }
    if(!hardwareInterface.isMotorEnabled(axisIndex)){
        displayInfo("错误：请先使能当前电机，再启动轨迹测试", "error");
        return;
    }
    const RobotStateSnapshot robotState = currentRobotState();
    if(robotState.anyMotionRunning && !runtimeState.singleMotorTrajectoryActive){
        displayInfo("错误：当前已有运动任务运行，不能启动单电机轨迹测试", "error");
        return;
    }
    if(!singleMotorTrajectoryWorker){
        startSingleMotorTrajectoryThread();
    }
    if(!singleMotorTrajectoryWorker){
        displayInfo("错误：单电机轨迹测试线程未创建", "error");
        return;
    }

    const bool useCurrentStart =
            singleMotorTrajectoryUseCurrentStartCheckBox &&
            singleMotorTrajectoryUseCurrentStartCheckBox->isChecked();
    double axisMin = -100000000.0;
    double axisMax = 100000000.0;
    double axisVelMax = 0.0;
    if(axisIndex < static_cast<int>(axisMotorMinVec.size()) && axisMotorMinVec[axisIndex]){
        axisMin = axisMotorMinVec[axisIndex]->value();
    }
    if(axisIndex < static_cast<int>(axisMotorMaxVec.size()) && axisMotorMaxVec[axisIndex]){
        axisMax = axisMotorMaxVec[axisIndex]->value();
    }
    if(axisIndex < static_cast<int>(axisMotorVelMaxVec.size()) && axisMotorVelMaxVec[axisIndex]){
        axisVelMax = axisMotorVelMaxVec[axisIndex]->value();
    }
    double uiVelMax = singleMotorTrajectoryVelocityLimitSpin ?
                singleMotorTrajectoryVelocityLimitSpin->value() :
                axisVelMax;
    if(uiVelMax <= 1e-9){
        uiVelMax = axisVelMax;
    }
    double commandVelMax = uiVelMax;
    if(axisVelMax > 1e-9){
        commandVelMax = commandVelMax > 1e-9 ? std::min(commandVelMax, axisVelMax) : axisVelMax;
    }

    SingleMotorTrajectoryWorker::Command command;
    command.axisIndex = axisIndex;
    command.type = static_cast<SingleMotorTrajectoryWorker::TrajectoryType>(
                singleMotorTrajectoryTypeCombo ?
                    singleMotorTrajectoryTypeCombo->currentData(Qt::UserRole).toInt() :
                    static_cast<int>(SingleMotorTrajectoryWorker::TrajectoryType::Line));
    command.startPosition = singleMotorTrajectoryStartSpin ? singleMotorTrajectoryStartSpin->value() : 0.0;
    command.endPosition = singleMotorTrajectoryEndSpin ? singleMotorTrajectoryEndSpin->value() : 0.0;
    command.centerPosition = singleMotorTrajectoryCenterSpin ? singleMotorTrajectoryCenterSpin->value() : 0.0;
    command.amplitude = singleMotorTrajectoryAmplitudeSpin ? singleMotorTrajectoryAmplitudeSpin->value() : 0.0;
    command.cycles = singleMotorTrajectoryCyclesSpin ? singleMotorTrajectoryCyclesSpin->value() : 1.0;
    command.phaseDeg = singleMotorTrajectoryPhaseSpin ? singleMotorTrajectoryPhaseSpin->value() : 0.0;
    command.durationSec = singleMotorTrajectoryDurationSpin ? singleMotorTrajectoryDurationSpin->value() : 5.0;
    command.planStepMs = singleMotorTrajectoryPlanStepSpin ? singleMotorTrajectoryPlanStepSpin->value() : 20.0;
    command.minPosition = axisMin;
    command.maxPosition = axisMax;
    command.maxVelocity = commandVelMax;
    command.useCurrentStart = useCurrentStart;

    disableForceControlForManualMotorTest();
    updateControlWorkerConfig();
    clearSingleMotorTrajectoryPlots();
    if(singleMotorTrajectoryStatusLabel){
        singleMotorTrajectoryStatusLabel->setText(QStringLiteral("启动中"));
    }
    if(singleMotorTrajectoryStartButton){
        singleMotorTrajectoryStartButton->setEnabled(false);
    }
    if(singleMotorTrajectoryStopButton){
        singleMotorTrajectoryStopButton->setEnabled(true);
    }

    QMetaObject::invokeMethod(singleMotorTrajectoryWorker,
                              [worker = singleMotorTrajectoryWorker, command](){
        worker->startTrajectory(command);
    }, Qt::QueuedConnection);
}

void MainWindow::stopSingleMotorTrajectory(const QString& source)
{
    Q_UNUSED(source)
    if(singleMotorTrajectoryWorker){
        QMetaObject::invokeMethod(singleMotorTrajectoryWorker,
                                  "requestStopTrajectory",
                                  Qt::QueuedConnection);
    }
    runtimeState.singleMotorTrajectoryActive = false;
    singleMotorTrajectoryRunning = false;
    updateSafetyMonitorConfig();
    refreshRunModeUiStateThrottled();
    if(singleMotorTrajectoryStartButton){
        singleMotorTrajectoryStartButton->setEnabled(true);
    }
    if(singleMotorTrajectoryStopButton){
        singleMotorTrajectoryStopButton->setEnabled(false);
    }
    if(singleMotorTrajectoryStatusLabel){
        singleMotorTrajectoryStatusLabel->setText(QStringLiteral("停止中"));
    }
}

void MainWindow::exportSingleMotorTrajectoryData()
{
    if(singleMotorTrajectorySampleTime.isEmpty()){
        displayInfo("错误：当前没有可导出的单电机轨迹测试数据", "error");
        return;
    }

    const QString defaultName = QStringLiteral("single_motor_trajectory_%1.csv")
            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_hhmmss")));
    const QString filePath = QFileDialog::getSaveFileName(this,
                                                          QStringLiteral("导出单电机轨迹数据"),
                                                          defaultName,
                                                          QStringLiteral("CSV Files (*.csv);;All Files (*)"));
    if(filePath.isEmpty()){
        return;
    }

    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        displayInfo((QStringLiteral("错误：无法写入单电机轨迹数据文件 ") + filePath).toStdString(), "error");
        return;
    }

    const auto csvNumber = [](double value) -> QString {
        return std::isfinite(value) ? QString::number(value, 'f', 9) : QString();
    };

    QTextStream stream(&file);
    stream << "time_s,qt_expected_deg,command_position_deg,actual_position_deg,command_minus_actual_deg,drive_60f4_error_deg\n";
    const int count = std::min({singleMotorTrajectorySampleTime.size(),
                                singleMotorTrajectoryExpected.size(),
                                singleMotorTrajectoryCommand.size(),
                                singleMotorTrajectoryActual.size(),
                                singleMotorTrajectoryError.size(),
                                singleMotorTrajectoryDriveError.size()});
    for(int i=0; i<count; ++i){
        stream << QString::number(singleMotorTrajectorySampleTime[i], 'f', 6) << ','
               << csvNumber(singleMotorTrajectoryExpected[i]) << ','
               << csvNumber(singleMotorTrajectoryCommand[i]) << ','
               << csvNumber(singleMotorTrajectoryActual[i]) << ','
               << csvNumber(singleMotorTrajectoryError[i]) << ','
               << csvNumber(singleMotorTrajectoryDriveError[i]) << '\n';
    }
    file.close();
    displayInfo((QStringLiteral("单电机轨迹测试数据已导出：") + filePath).toStdString());
}

void MainWindow::handleSingleMotorTrajectoryPrepared(int,
                                                     QVector<double> time,
                                                     QVector<double>)
{
    clearSingleMotorTrajectoryPlots();
    singleMotorTrajectoryDurationSec = time.isEmpty() ? 0.0 : time.back();
    if(singleMotorTrajectoryTrackPlotHost && singleMotorTrajectoryTrackPlotHost->graphCount() >= 2){
        singleMotorTrajectoryTrackPlotHost->graph(0)->data()->clear();
        singleMotorTrajectoryTrackPlotHost->graph(1)->data()->clear();
        singleMotorTrajectoryTrackPlotHost->xAxis->setRange(0.0, std::max(1.0, singleMotorTrajectoryDurationSec));
        singleMotorTrajectoryTrackPlotHost->yAxis->setRange(-1.0, 1.0);
        singleMotorTrajectoryTrackPlotHost->replot(QCustomPlot::rpQueuedReplot);
    }
    if(singleMotorTrajectoryErrorPlotHost && singleMotorTrajectoryErrorPlotHost->graphCount() >= 2){
        singleMotorTrajectoryErrorPlotHost->graph(0)->data()->clear();
        singleMotorTrajectoryErrorPlotHost->graph(1)->data()->clear();
        singleMotorTrajectoryErrorPlotHost->xAxis->setRange(0.0, std::max(1.0, singleMotorTrajectoryDurationSec));
        singleMotorTrajectoryErrorPlotHost->yAxis->setRange(-1.0, 1.0);
        singleMotorTrajectoryErrorPlotHost->replot(QCustomPlot::rpQueuedReplot);
    }
}

void MainWindow::handleSingleMotorTrajectorySample(int axisIndex,
                                                   double time,
                                                   double expectedPosition,
                                                   double actualPosition,
                                                   double tracePositionError,
                                                   double commandPosition,
                                                   bool commandPositionValid,
                                                   int driveFollowingErrorRaw,
                                                   bool driveFollowingErrorValid)
{
    const double expectedDeg = singleMotorTrajectoryPositionToDegrees(expectedPosition);
    const double actualDeg = singleMotorTrajectoryPositionToDegrees(actualPosition);
    const double commandDeg = commandPositionValid ?
                singleMotorTrajectoryPositionToDegrees(commandPosition) :
                std::numeric_limits<double>::quiet_NaN();
    const double errorDeg = singleMotorTrajectoryPositionToDegrees(tracePositionError);
    const double driveErrorDeg = driveFollowingErrorValid ?
                singleMotorTrajectoryRawFollowingErrorToDegrees(axisIndex, driveFollowingErrorRaw) :
                std::numeric_limits<double>::quiet_NaN();

    singleMotorTrajectorySampleTime.push_back(time);
    singleMotorTrajectoryExpected.push_back(expectedDeg);
    singleMotorTrajectoryCommand.push_back(commandDeg);
    singleMotorTrajectoryActual.push_back(actualDeg);
    singleMotorTrajectoryError.push_back(errorDeg);
    singleMotorTrajectoryDriveError.push_back(driveErrorDeg);

    if(singleMotorTrajectoryTrackPlotHost && singleMotorTrajectoryTrackPlotHost->graphCount() >= 2){
        if(std::isfinite(commandDeg)){
            singleMotorTrajectoryTrackPlotHost->graph(0)->addData(time, commandDeg);
        }
        if(std::isfinite(actualDeg)){
            singleMotorTrajectoryTrackPlotHost->graph(1)->addData(time, actualDeg);
        }
        singleMotorTrajectoryTrackPlotHost->xAxis->setRange(0.0, std::max({1.0, singleMotorTrajectoryDurationSec, time}));
        rescaleCustomPlotYAxis(singleMotorTrajectoryTrackPlotHost);
        singleMotorTrajectoryTrackPlotHost->replot(QCustomPlot::rpQueuedReplot);
    }
    if(singleMotorTrajectoryErrorPlotHost && singleMotorTrajectoryErrorPlotHost->graphCount() >= 2){
        if(std::isfinite(errorDeg)){
            singleMotorTrajectoryErrorPlotHost->graph(0)->addData(time, errorDeg);
        }
        if(std::isfinite(driveErrorDeg)){
            singleMotorTrajectoryErrorPlotHost->graph(1)->addData(time, driveErrorDeg);
        }
        singleMotorTrajectoryErrorPlotHost->xAxis->setRange(0.0, std::max({1.0, singleMotorTrajectoryDurationSec, time}));
        rescaleCustomPlotYAxis(singleMotorTrajectoryErrorPlotHost);
        singleMotorTrajectoryErrorPlotHost->replot(QCustomPlot::rpQueuedReplot);
    }
    if(singleMotorTrajectoryMaxErrorSpin){
        double maxAbsError = 0.0;
        for(double value : singleMotorTrajectoryError){
            if(std::isfinite(value)){
                maxAbsError = std::max(maxAbsError, std::fabs(value));
            }
        }
        for(double value : singleMotorTrajectoryDriveError){
            if(std::isfinite(value)){
                maxAbsError = std::max(maxAbsError, std::fabs(value));
            }
        }
        QSignalBlocker blocker(singleMotorTrajectoryMaxErrorSpin);
        singleMotorTrajectoryMaxErrorSpin->setValue(maxAbsError);
    }
}

void MainWindow::handleSingleMotorTrajectoryFeedback(int axisIndex, double relativePosition)
{
    if(axisIndex != selectedSingleMotorTrajectoryAxisIndex()){
        return;
    }
    if(singleMotorTrajectoryCurrentPosSpin){
        QSignalBlocker blocker(singleMotorTrajectoryCurrentPosSpin);
        singleMotorTrajectoryCurrentPosSpin->setValue(std::isfinite(relativePosition) ? relativePosition : 0.0);
    }
}

void MainWindow::handleSingleMotorTrajectoryStateChanged(bool active, const QString& statusText)
{
    runtimeState.singleMotorTrajectoryActive = active;
    singleMotorTrajectoryRunning = active;
    if(singleMotorTrajectoryStartButton){
        singleMotorTrajectoryStartButton->setEnabled(!active && selectedSingleMotorTrajectoryAxisIndex() >= 0);
    }
    if(singleMotorTrajectoryStopButton){
        singleMotorTrajectoryStopButton->setEnabled(active);
    }
    if(singleMotorTrajectoryStatusLabel){
        singleMotorTrajectoryStatusLabel->setText(statusText);
    }
    updateSafetyMonitorConfig();
    refreshRunModeUiStateThrottled();
}

void MainWindow::setupMotorTorqueTestTab()
{
    if(!ui || motorTorqueTestTab){
        return;
    }

    motorTorqueTestTab = findOptionalUiObject<QWidget>(this, "motorTorqueTestTab");
    motorTorqueServoVelocityLimitSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorqueServoVelocityLimitSpin");
    motorTorqueAxisCombo = findOptionalUiObject<QComboBox>(this, "motorTorqueAxisCombo");
    motorTorqueTargetSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorqueTargetSpin");
    motorTorquePosMinSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorquePosMinSpin");
    motorTorquePosMaxSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorquePosMaxSpin");
    motorTorqueVelMaxSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorqueVelMaxSpin");
    motorTorqueActualPosSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorqueActualPosSpin");
    motorTorqueActualVelSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorqueActualVelSpin");
    motorTorqueActualTorqueSpin = findOptionalUiObject<QDoubleSpinBox>(this, "motorTorqueActualTorqueSpin");
    motorTorqueServoSetupButton = findOptionalUiObject<QPushButton>(this, "motorTorqueServoSetupButton");
    motorTorqueEnableButton = findOptionalUiObject<QPushButton>(this, "motorTorqueEnableButton");
    motorTorqueStartButton = findOptionalUiObject<QPushButton>(this, "motorTorqueStartButton");
    motorTorqueStopButton = findOptionalUiObject<QPushButton>(this, "motorTorqueStopButton");
    motorTorqueServoSetupStatusLabel = findOptionalUiObject<QLabel>(this, "motorTorqueServoSetupStatusLabel");
    motorTorqueStatusLabel = findOptionalUiObject<QLabel>(this, "motorTorqueStatusLabel");

    if(!motorTorqueTestTab ||
            !motorTorqueAxisCombo ||
            !motorTorqueTargetSpin ||
            !motorTorquePosMinSpin ||
            !motorTorquePosMaxSpin ||
            !motorTorqueVelMaxSpin ||
            !motorTorqueActualPosSpin ||
            !motorTorqueActualVelSpin ||
            !motorTorqueActualTorqueSpin ||
            !motorTorqueEnableButton ||
            !motorTorqueStartButton ||
            !motorTorqueStopButton ||
            !motorTorqueStatusLabel){
        displayInfo("错误：转矩模式调试UI未完整加载", "error");
        return;
    }

    if(motorTorqueServoVelocityLimitSpin){
        motorTorqueServoVelocityLimitSpin->setRange(0.01, 1000000.0);
        motorTorqueServoVelocityLimitSpin->setDecimals(2);
        motorTorqueServoVelocityLimitSpin->setSingleStep(0.01);
        motorTorqueServoVelocityLimitSpin->setValue(kTorqueServoVelocityLimitRpm);
        motorTorqueServoVelocityLimitSpin->setKeyboardTracking(false);
    }
    motorTorqueTargetSpin->setRange(-200000.0, 200000.0);
    motorTorqueTargetSpin->setDecimals(2);
    motorTorqueTargetSpin->setSingleStep(0.01);
    motorTorqueTargetSpin->setSuffix(QStringLiteral(" N·m"));
    motorTorqueTargetSpin->setKeyboardTracking(false);

    const auto initLimitSpin = [](QDoubleSpinBox* spin){
        spin->setRange(-100000000.0, 100000000.0);
        spin->setDecimals(6);
        spin->setSingleStep(0.1);
        spin->setKeyboardTracking(false);
    };
    initLimitSpin(motorTorquePosMinSpin);
    initLimitSpin(motorTorquePosMaxSpin);
    initLimitSpin(motorTorqueVelMaxSpin);
    motorTorqueVelMaxSpin->setRange(0.0, 100000000.0);

    motorTorqueStopButton->setEnabled(false);

    const auto initReadSpin = [](QDoubleSpinBox* spin){
        spin->setRange(-100000000.0, 100000000.0);
        spin->setDecimals(6);
        spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spin->setReadOnly(true);
    };
    initReadSpin(motorTorqueActualPosSpin);
    initReadSpin(motorTorqueActualVelSpin);
    initReadSpin(motorTorqueActualTorqueSpin);
    motorTorqueActualTorqueSpin->setSuffix(QStringLiteral(" N·m"));
    motorTorqueStatusLabel->setFrameShape(QFrame::StyledPanel);
    motorTorqueStatusLabel->setMinimumHeight(24);

    if(motorTorqueServoSetupButton){
        connect(motorTorqueServoSetupButton, &QPushButton::clicked, this, [this](){
            applyMotorTorqueServoVelocityLimitFromUi();
            displayInfo("转矩模式220Bh速度限制参数已更新，将在下次连接雷赛控制卡时按pulse/s写入", "normal");
        });
    }
    connect(motorTorqueAxisCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int){
        syncMotorTorqueLimitUiFromAxis(selectedMotorTorqueAxisIndex());
        updateMotorTorqueWorkerConfig();
    });
    connect(motorTorqueTargetSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this](double){
        updateMotorTorqueWorkerConfig();
    });
    const auto connectLimitSpin = [this](QDoubleSpinBox* spin){
        connect(spin,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [this](double){
            if(suppressMotorTorqueLimitUiSync){
                return;
            }
            const int axisIndex = selectedMotorTorqueAxisIndex();
            applyMotorTorqueLimitUiToAxis(axisIndex);
            updateMotorTorqueWorkerConfig();
        });
    };
    connectLimitSpin(motorTorquePosMinSpin);
    connectLimitSpin(motorTorquePosMaxSpin);
    connectLimitSpin(motorTorqueVelMaxSpin);
    connect(motorTorqueEnableButton, &QPushButton::clicked, this, [this](){
        const int axisIndex = selectedMotorTorqueAxisIndex();
        if(axisIndex < 0){
            displayInfo("错误：转矩调试电机轴号无效", "error");
            return;
        }
        if(!ui->devUseLS->isChecked()){
            displayInfo("错误：当前仅支持雷赛转矩模式调试", "error");
            return;
        }
        if(!hardwareInterface.isLSConnected()){
            displayInfo("错误：请先连接雷赛控制卡", "error");
            return;
        }
        if(!applyLeadshineHardwareConfigFromUi()){
            displayInfo("错误：转矩调试使能失败，雷赛轴参数未正确配置", "error");
            return;
        }
        disableForceControlForManualMotorTest();
        updateControlWorkerConfig();
        if(!hardwareInterface.setMotorEnable(axisIndex, true)){
            displayInfo("错误：转矩调试电机使能失败", "error");
            return;
        }
        if(ccThread && !ccThread->isRunning()){
            ccThread->start(QThread::HighPriority);
        }
        displayInfo(QString("转矩调试：电机轴%1已使能").arg(axisIndex).toStdString());
    });
    connect(motorTorqueStartButton, &QPushButton::clicked, this, &MainWindow::startMotorTorqueDebug);
    connect(motorTorqueStopButton, &QPushButton::clicked, this, &MainWindow::stopMotorTorqueDebug);

    for(QComboBox* axisType : axisTypeVec){
        if(axisType){
            connect(axisType,
                    QOverload<int>::of(&QComboBox::currentIndexChanged),
                    this,
                    [this](int){
                refreshMotorTorqueAxisOptions();
            });
        }
    }
    for(QSpinBox* hardwareAxisSpin : axisMotorHardwareAxisVec){
        if(hardwareAxisSpin){
            connect(hardwareAxisSpin,
                    QOverload<int>::of(&QSpinBox::valueChanged),
                    this,
                    [this](int){
                refreshMotorTorqueAxisOptions();
            });
        }
    }
    for(QSpinBox* slaveIdSpin : axisMotorSlaveIdVec){
        if(slaveIdSpin){
            connect(slaveIdSpin,
                    QOverload<int>::of(&QSpinBox::valueChanged),
                    this,
                    [this](int){
                refreshMotorTorqueAxisOptions();
            });
        }
    }

    refreshMotorTorqueAxisOptions();
    startMotorTorqueTestThread();
    updateMotorTorqueWorkerConfig();
}

void MainWindow::startMotorTorqueTestThread()
{
    if(motorTorqueTestThread || motorTorqueTestWorker){
        return;
    }

    motorTorqueTestThread = new QThread(this);
    motorTorqueTestThread->setObjectName(QStringLiteral("MotorTorqueTestThread"));
    motorTorqueTestWorker = new MotorTorqueTestWorker(&hardwareInterface);
    motorTorqueTestWorker->moveToThread(motorTorqueTestThread);
    connect(motorTorqueTestThread, &QThread::started, motorTorqueTestWorker, &MotorTorqueTestWorker::start);
    connect(motorTorqueTestThread, &QThread::finished, motorTorqueTestWorker, &QObject::deleteLater);
    connect(motorTorqueTestWorker, &MotorTorqueTestWorker::displayInfoSignal, this, &MainWindow::displayInfo);
    connect(motorTorqueTestWorker,
            &MotorTorqueTestWorker::statusUpdated,
            this,
            &MainWindow::handleMotorTorqueStatus,
            Qt::QueuedConnection);
    connect(motorTorqueTestWorker,
            &MotorTorqueTestWorker::activeChanged,
            this,
            [this](bool active){
        runtimeState.motorTorqueDebugActive = active;
        if(motorTorqueStartButton){
            motorTorqueStartButton->setEnabled(!active);
        }
        if(motorTorqueStopButton){
            motorTorqueStopButton->setEnabled(active);
        }
        if(motorTorqueStatusLabel){
            motorTorqueStatusLabel->setText(active ? QStringLiteral("运行中") : QStringLiteral("已停止"));
        }
        refreshRunModeUiStateThrottled();
        updateSafetyMonitorConfig();
    }, Qt::QueuedConnection);
    motorTorqueTestThread->start(QThread::HighPriority);
}

void MainWindow::stopMotorTorqueTestThread()
{
    runtimeState.motorTorqueDebugActive = false;
    if(motorTorqueTestWorker){
        if(motorTorqueTestThread && motorTorqueTestThread->isRunning()){
            QMetaObject::invokeMethod(motorTorqueTestWorker,
                                      "stop",
                                      Qt::BlockingQueuedConnection);
        }
        else{
            motorTorqueTestWorker->stop();
        }
    }
    if(motorTorqueTestThread){
        motorTorqueTestThread->quit();
        if(!motorTorqueTestThread->wait(1000)){
            motorTorqueTestThread->terminate();
            motorTorqueTestThread->wait(500);
        }
        delete motorTorqueTestThread;
        motorTorqueTestThread = nullptr;
    }
    motorTorqueTestWorker = nullptr;
}

int MainWindow::selectedMotorTorqueAxisIndex() const
{
    if(!motorTorqueAxisCombo || motorTorqueAxisCombo->currentIndex() < 0){
        return -1;
    }
    return motorTorqueAxisCombo->currentData(Qt::UserRole).toInt();
}

double MainWindow::motorTorqueServoVelocityLimitRpm() const
{
    if(!motorTorqueServoVelocityLimitSpin){
        return kTorqueServoVelocityLimitRpm;
    }
    return std::max(0.01, motorTorqueServoVelocityLimitSpin->value());
}

bool MainWindow::applyMotorTorqueServoVelocityLimitFromUi()
{
    const int axisCount = ui->devAxisNum->value();
    std::vector<int> motorSlaveIdVec(axisCount, 0);
    std::vector<bool> motorTorqueVelocityLimitEnabled(axisCount, false);
    for(int i = 0; i < axisCount; ++i){
        if(!isMotorAxis(i)){
            continue;
        }
        motorTorqueVelocityLimitEnabled[i] = !isLinearMotorAxis(i);
        if(i < static_cast<int>(axisMotorSlaveIdVec.size()) && axisMotorSlaveIdVec[i]){
            motorSlaveIdVec[i] = axisMotorSlaveIdVec[i]->value();
        }
    }
    hardwareInterface.setMotorSlaveIds(motorSlaveIdVec);
    hardwareInterface.setMotorTorqueVelocityLimitEnabled(motorTorqueVelocityLimitEnabled);
    const double velocityLimit = motorTorqueServoVelocityLimitRpm();
    hardwareInterface.setLeadshineTorqueVelocityLimitRpm(velocityLimit);

    if(motorTorqueServoSetupStatusLabel){
        motorTorqueServoSetupStatusLabel->setText(QStringLiteral("连接时写入"));
    }
    return true;
}

void MainWindow::refreshMotorTorqueAxisOptions()
{
    if(!motorTorqueAxisCombo){
        return;
    }

    const int previousAxis = selectedMotorTorqueAxisIndex();
    QSignalBlocker blocker(motorTorqueAxisCombo);
    motorTorqueAxisCombo->clear();
    const int axisCount = ui ? ui->devAxisNum->value() : 0;
    for(int axisIndex=0; axisIndex<axisCount; ++axisIndex){
        if(!isMotorAxis(axisIndex)){
            continue;
        }
        QString axisText = QStringLiteral("逻辑轴%1").arg(axisIndex);
        if(axisIndex < static_cast<int>(axisMotorHardwareAxisVec.size()) &&
                axisMotorHardwareAxisVec[axisIndex]){
            axisText += QStringLiteral(" / 控制卡轴%1").arg(axisMotorHardwareAxisVec[axisIndex]->value());
        }
        if(axisIndex < static_cast<int>(axisMotorSlaveIdVec.size()) &&
                axisMotorSlaveIdVec[axisIndex] &&
                axisMotorSlaveIdVec[axisIndex]->value() > 0){
            axisText += QStringLiteral(" / 从站%1").arg(axisMotorSlaveIdVec[axisIndex]->value());
        }
        motorTorqueAxisCombo->addItem(axisText, axisIndex);
    }

    int targetIndex = -1;
    if(previousAxis >= 0){
        targetIndex = motorTorqueAxisCombo->findData(previousAxis, Qt::UserRole);
    }
    if(targetIndex < 0 && motorTorqueAxisCombo->count() > 0){
        targetIndex = 0;
    }
    motorTorqueAxisCombo->setCurrentIndex(targetIndex);
    const bool hasAxis = targetIndex >= 0;
    motorTorqueAxisCombo->setEnabled(hasAxis);
    if(motorTorqueEnableButton){
        motorTorqueEnableButton->setEnabled(hasAxis);
    }
    if(motorTorqueStartButton){
        motorTorqueStartButton->setEnabled(hasAxis);
    }
    syncMotorTorqueLimitUiFromAxis(selectedMotorTorqueAxisIndex());
}

void MainWindow::syncMotorTorqueLimitUiFromAxis(int axisIndex)
{
    if(!motorTorquePosMinSpin || !motorTorquePosMaxSpin || !motorTorqueVelMaxSpin){
        return;
    }

    suppressMotorTorqueLimitUiSync = true;
    const bool validAxis = axisIndex >= 0 &&
            axisIndex < static_cast<int>(axisMotorMinVec.size()) &&
            axisIndex < static_cast<int>(axisMotorMaxVec.size()) &&
            axisIndex < static_cast<int>(axisMotorVelMaxVec.size()) &&
            axisMotorMinVec[axisIndex] &&
            axisMotorMaxVec[axisIndex] &&
            axisMotorVelMaxVec[axisIndex];
    if(validAxis){
        motorTorquePosMinSpin->setValue(axisMotorMinVec[axisIndex]->value());
        motorTorquePosMaxSpin->setValue(axisMotorMaxVec[axisIndex]->value());
        motorTorqueVelMaxSpin->setValue(axisMotorVelMaxVec[axisIndex]->value());
    }
    else{
        motorTorquePosMinSpin->setValue(0.0);
        motorTorquePosMaxSpin->setValue(0.0);
        motorTorqueVelMaxSpin->setValue(0.0);
    }
    motorTorquePosMinSpin->setEnabled(validAxis);
    motorTorquePosMaxSpin->setEnabled(validAxis);
    motorTorqueVelMaxSpin->setEnabled(validAxis);
    suppressMotorTorqueLimitUiSync = false;
}

void MainWindow::applyMotorTorqueLimitUiToAxis(int axisIndex)
{
    if(axisIndex < 0 ||
            axisIndex >= static_cast<int>(axisMotorMinVec.size()) ||
            axisIndex >= static_cast<int>(axisMotorMaxVec.size()) ||
            axisIndex >= static_cast<int>(axisMotorVelMaxVec.size()) ||
            !axisMotorMinVec[axisIndex] ||
            !axisMotorMaxVec[axisIndex] ||
            !axisMotorVelMaxVec[axisIndex]){
        return;
    }

    const QSignalBlocker minBlocker(axisMotorMinVec[axisIndex]);
    const QSignalBlocker maxBlocker(axisMotorMaxVec[axisIndex]);
    const QSignalBlocker velBlocker(axisMotorVelMaxVec[axisIndex]);
    axisMotorMinVec[axisIndex]->setValue(motorTorquePosMinSpin->value());
    axisMotorMaxVec[axisIndex]->setValue(motorTorquePosMaxSpin->value());
    axisMotorVelMaxVec[axisIndex]->setValue(motorTorqueVelMaxSpin->value());
    syncMotorSoftwareLimitsToHardware();
    updateControlWorkerConfig();
}

void MainWindow::updateMotorTorqueWorkerConfig()
{
    if(!motorTorqueTestWorker ||
            !motorTorqueAxisCombo ||
            !motorTorqueTargetSpin ||
            !motorTorquePosMinSpin ||
            !motorTorquePosMaxSpin ||
            !motorTorqueVelMaxSpin){
        return;
    }

    const int axisIndex = selectedMotorTorqueAxisIndex();
    QMetaObject::invokeMethod(motorTorqueTestWorker,
                              "setConfig",
                              Qt::QueuedConnection,
                              Q_ARG(int, axisIndex),
                              Q_ARG(double, motorTorqueTargetSpin->value()),
                              Q_ARG(double, motorTorquePosMinSpin->value()),
                              Q_ARG(double, motorTorquePosMaxSpin->value()),
                              Q_ARG(double, motorTorqueVelMaxSpin->value()));
}

void MainWindow::startMotorTorqueDebug()
{
    const int axisIndex = selectedMotorTorqueAxisIndex();
    if(axisIndex < 0){
        displayInfo("错误：转矩调试电机轴号无效", "error");
        return;
    }
    if(!ui->devUseLS->isChecked()){
        displayInfo("错误：当前仅支持雷赛转矩模式调试", "error");
        return;
    }
    if(!hardwareInterface.isLSConnected()){
        displayInfo("错误：请先连接雷赛控制卡", "error");
        return;
    }
    if(!hardwareInterface.isMotorEnabled(axisIndex)){
        displayInfo("错误：请先使能当前电机，再启动转矩模式", "error");
        return;
    }
    if(motorTorqueTargetSpin && qFuzzyIsNull(motorTorqueTargetSpin->value())){
        displayInfo("错误：目标力矩为0，转矩模式不会启动", "error");
        return;
    }
    const RobotStateSnapshot robotState = currentRobotState(false);
    if(robotState.anyMotionRunning){
        displayInfo("错误：当前已有运动任务运行，不能启动转矩模式调试", "error");
        return;
    }

    disableForceControlForManualMotorTest();
    applyMotorTorqueLimitUiToAxis(axisIndex);
    updateMotorTorqueWorkerConfig();
    if(ccThread && !ccThread->isRunning()){
        ccThread->start(QThread::HighPriority);
    }
    if(motorTorqueTestWorker){
        QMetaObject::invokeMethod(motorTorqueTestWorker,
                                  "setTorqueActive",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool, true));
    }
    runtimeState.motorTorqueDebugActive = true;
    updateSafetyMonitorConfig();
    refreshRunModeUiStateThrottled();
    if(motorTorqueStartButton){
        motorTorqueStartButton->setEnabled(false);
    }
    if(motorTorqueStopButton){
        motorTorqueStopButton->setEnabled(true);
    }
    if(motorTorqueStatusLabel){
        motorTorqueStatusLabel->setText(QStringLiteral("启动中"));
    }
}

void MainWindow::stopMotorTorqueDebug()
{
    if(motorTorqueTestWorker){
        QMetaObject::invokeMethod(motorTorqueTestWorker,
                                  "requestStopMotion",
                                  Qt::QueuedConnection);
    }
    runtimeState.motorTorqueDebugActive = false;
    updateSafetyMonitorConfig();
    refreshRunModeUiStateThrottled();
    if(motorTorqueStartButton){
        motorTorqueStartButton->setEnabled(true);
    }
    if(motorTorqueStopButton){
        motorTorqueStopButton->setEnabled(false);
    }
    if(motorTorqueStatusLabel){
        motorTorqueStatusLabel->setText(QStringLiteral("停止中"));
    }
}

void MainWindow::handleMotorTorqueStatus(int,
                                         double relativePosition,
                                         double actualTorque,
                                         double actualVelocity,
                                         bool active)
{
    const auto setSpinValue = [](QDoubleSpinBox* spin, double value){
        if(!spin){
            return;
        }
        QSignalBlocker blocker(spin);
        spin->setValue(std::isfinite(value) ? value : 0.0);
    };
    setSpinValue(motorTorqueActualPosSpin, relativePosition);
    setSpinValue(motorTorqueActualVelSpin, actualVelocity);
    setSpinValue(motorTorqueActualTorqueSpin, actualTorque);
    if(motorTorqueStartButton){
        motorTorqueStartButton->setEnabled(!active);
    }
    if(motorTorqueStopButton){
        motorTorqueStopButton->setEnabled(active);
    }
    if(motorTorqueStatusLabel){
        motorTorqueStatusLabel->setText(active ? QStringLiteral("运行中") : QStringLiteral("未启动"));
    }
}

void MainWindow::reinitializeDataVisualizationPlots()
{
    if(!curveDrawer){
        return;
    }

    curveDrawer->setChannelCounts(8, 8);
    curveDrawer->initPlot(motorControlPlotHost,
                          endEffectorPosPlotHost,
                          endEffectorVelPlotHost,
                          endEffectorAccPlotHost,
                          cableTensionPlotHost,
                          cableSpeedPlotHost,
                          cableLengthPlotHost);
    clearVisualizationData();
}

void MainWindow::clearVisualizationData()
{
    if(curveDrawer){
        curveDrawer->clearAllDataSignal();
    }
    visualizationTimerStarted = false;
    lastVisualizationPoseTime = -1.0;
    lastVisualizationPose.clear();
    lastVisualizationVelocity.clear();
}

double MainWindow::visualizationTimeSeconds()
{
    if(!visualizationTimerStarted){
        visualizationElapsedTimer.start();
        visualizationTimerStarted = true;
    }
    return static_cast<double>(visualizationElapsedTimer.elapsed()) / 1000.0;
}

void MainWindow::updateVisualizationFromControlSnapshot(const ControlWorker::Snapshot& snapshot)
{
    if(!curveDrawer){
        return;
    }
    const double timeSec = visualizationTimeSeconds();
    curveDrawer->updateMotorControlPlotSignal(timeSec, mapMotorCommandForPlot(snapshot.motorCommand));
    curveDrawer->updateCableTensionPlotSignal(timeSec, mapCableTensionForPlot(snapshot.forceSensorValue));
    curveDrawer->updateCableSpeedPlotSignal(timeSec, mapCableSpeedForPlot(snapshot.motorVel));
}

void MainWindow::updateVisualizationFromPlatformPose(const std::vector<std::vector<double>>& platformPose)
{
    if(!curveDrawer || platformPose.empty() || platformPose.front().size() < 6){
        return;
    }
    const double timeSec = visualizationTimeSeconds();
    curveDrawer->updateCableLengthPlotSignal(timeSec, buildGeometricCableLengthForPlot(platformPose));
    updateVisualizationPoseSample(timeSec, platformPose.front());
}

void MainWindow::handleVisualizationRigidPose(const std::vector<std::vector<double>>& rigidPose)
{
    updateVisualizationFromPlatformPose(rigidPose);
}

void MainWindow::updateVisualizationPoseSample(double timeSec,
                                               const std::vector<double>& pose,
                                               const std::vector<double>* velocity,
                                               const std::vector<double>* acceleration)
{
    if(!curveDrawer || pose.size() < 6){
        return;
    }

    if(!visualizationTimerStarted){
        visualizationElapsedTimer.start();
        visualizationTimerStarted = true;
    }

    std::vector<double> velocityValue(6, 0.0);
    std::vector<double> accelerationValue(6, 0.0);

    if(velocity && velocity->size() >= 6){
        velocityValue.assign(velocity->begin(), velocity->begin() + 6);
        if(acceleration && acceleration->size() >= 6){
            accelerationValue.assign(acceleration->begin(), acceleration->begin() + 6);
        }
        else if(lastVisualizationPoseTime >= 0.0 &&
                lastVisualizationVelocity.size() >= 6 &&
                timeSec > lastVisualizationPoseTime){
            const double dt = timeSec - lastVisualizationPoseTime;
            for(int i=0; i<6; ++i){
                accelerationValue[i] = (velocityValue[i] - lastVisualizationVelocity[i]) / dt;
            }
        }
    }
    else if(lastVisualizationPoseTime >= 0.0 &&
            lastVisualizationPose.size() >= 6 &&
            timeSec > lastVisualizationPoseTime){
        const double dt = timeSec - lastVisualizationPoseTime;
        for(int i=0; i<6; ++i){
            velocityValue[i] = (pose[i] - lastVisualizationPose[i]) / dt;
        }
        if(lastVisualizationVelocity.size() >= 6){
            for(int i=0; i<6; ++i){
                accelerationValue[i] = (velocityValue[i] - lastVisualizationVelocity[i]) / dt;
            }
        }
    }

    QVector<double> positionVec(3, 0.0);
    QVector<double> orientationVec(3, 0.0);
    QVector<double> velocityVec(3, 0.0);
    QVector<double> angularVelocityVec(3, 0.0);
    QVector<double> accelerationVec(3, 0.0);
    QVector<double> angularAccelerationVec(3, 0.0);
    for(int i=0; i<3; ++i){
        positionVec[i] = pose[i];
        orientationVec[i] = pose[i + 3];
        velocityVec[i] = velocityValue[i];
        angularVelocityVec[i] = velocityValue[i + 3];
        accelerationVec[i] = accelerationValue[i];
        angularAccelerationVec[i] = accelerationValue[i + 3];
    }

    curveDrawer->updateEndEffectorPosPlotSignal(timeSec, positionVec, orientationVec);
    curveDrawer->updateEndEffectorVelPlotSignal(timeSec, velocityVec, angularVelocityVec);
    curveDrawer->updateEndEffectorAccPlotSignal(timeSec, accelerationVec, angularAccelerationVec);

    lastVisualizationPose.assign(pose.begin(), pose.begin() + 6);
    lastVisualizationVelocity = velocityValue;
    lastVisualizationPoseTime = timeSec;
}

QVector<double> MainWindow::mapMotorCommandForPlot(const std::vector<double>& motorCommand) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(axisIndex < static_cast<int>(motorCommand.size())){
            values[channelIndex] = motorCommand[axisIndex];
        }
        ++channelIndex;
    }
    return values;
}

QVector<double> MainWindow::mapCableTensionForPlot(const std::vector<double>& forceSensorValue) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        const int sensorIndex = axisSensorIndexVec[axisIndex]->value();
        if(sensorIndex >= 0 && sensorIndex < static_cast<int>(forceSensorValue.size())){
            values[channelIndex] = forceSensorValue[sensorIndex];
        }
        ++channelIndex;
    }
    return values;
}

QVector<double> MainWindow::mapCableSpeedForPlot(const std::vector<double>& motorVelocity) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(axisIndex < static_cast<int>(motorVelocity.size())){
            values[channelIndex] = convertMotorFeedbackToCableValue(axisIndex, motorVelocity[axisIndex]);
        }
        ++channelIndex;
    }
    return values;
}

QVector<double> MainWindow::buildGeometricCableLengthForPlot(const std::vector<std::vector<double>>& platformPose) const
{
    QVector<double> values(8, 0.0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        const int endIndex = axisEndVec[axisIndex]->value();
        if(endIndex >= 0 &&
                endIndex < static_cast<int>(platformPose.size()) &&
                platformPose[endIndex].size() >= 6){
            const std::vector<double> Rx = {
                1.0, 0.0, 0.0,
                0.0, cos(platformPose[endIndex][3]), -sin(platformPose[endIndex][3]),
                0.0, sin(platformPose[endIndex][3]),  cos(platformPose[endIndex][3])
            };
            const std::vector<double> Ry = {
                 cos(platformPose[endIndex][4]), 0.0, sin(platformPose[endIndex][4]),
                 0.0, 1.0, 0.0,
                -sin(platformPose[endIndex][4]), 0.0, cos(platformPose[endIndex][4])
            };
            const std::vector<double> Rz = {
                cos(platformPose[endIndex][5]), -sin(platformPose[endIndex][5]), 0.0,
                sin(platformPose[endIndex][5]),  cos(platformPose[endIndex][5]), 0.0,
                0.0, 0.0, 1.0
            };

            const std::vector<std::vector<double>> rotationX = {
                {Rx[0], Rx[1], Rx[2]},
                {Rx[3], Rx[4], Rx[5]},
                {Rx[6], Rx[7], Rx[8]}
            };
            const std::vector<std::vector<double>> rotationY = {
                {Ry[0], Ry[1], Ry[2]},
                {Ry[3], Ry[4], Ry[5]},
                {Ry[6], Ry[7], Ry[8]}
            };
            const std::vector<std::vector<double>> rotationZ = {
                {Rz[0], Rz[1], Rz[2]},
                {Rz[3], Rz[4], Rz[5]},
                {Rz[6], Rz[7], Rz[8]}
            };
            const std::vector<std::vector<double>> rotation =
                MatrixFun::matrix_mul(MatrixFun::matrix_mul(rotationZ, rotationY), rotationX);
            const std::vector<std::vector<double>> rotatedContactPoint =
                MatrixFun::matrix_mul(rotation,
                                      {{axisCableEndPosXVec[axisIndex]->value()},
                                       {axisCableEndPosYVec[axisIndex]->value()},
                                       {axisCableEndPosZVec[axisIndex]->value()}});

            std::vector<double> globalContactPoint(3, 0.0);
            for(int coordIndex=0; coordIndex<3; ++coordIndex){
                globalContactPoint[coordIndex] =
                    platformPose[endIndex][coordIndex] + rotatedContactPoint[coordIndex][0];
            }

            const std::vector<double> anchorPoint = {
                axisCableStartPosXVec[axisIndex]->value(),
                axisCableStartPosYVec[axisIndex]->value(),
                axisCableStartPosZVec[axisIndex]->value()
            };
            values[channelIndex] =
                MatrixFun::cableLengthCalculate(globalContactPoint, anchorPoint, buildPulleyRadius()).idealLength;
        }
        ++channelIndex;
    }
    return values;
}

QVector<double> MainWindow::flattenCableLengthsByAxisOrder(const std::vector<std::vector<double>>& cableLen) const
{
    QVector<double> values(8, 0.0);
    std::vector<int> channelOffset(ui->devEndNum->value(), 0);
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value() && channelIndex<values.size(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }

        const int endIndex = axisEndVec[axisIndex]->value();
        if(endIndex >= 0 && endIndex < static_cast<int>(cableLen.size())){
            const int cableIndex = channelOffset[endIndex];
            if(cableIndex >= 0 && cableIndex < static_cast<int>(cableLen[endIndex].size())){
                values[channelIndex] = cableLen[endIndex][cableIndex];
            }
            channelOffset[endIndex]++;
        }
        ++channelIndex;
    }
    return values;
}

QString MainWindow::formatCableLengthVector(const QVector<double>& cableLen) const
{
    QStringList parts;
    parts.reserve(cableLen.size());
    for(int i=0; i<cableLen.size(); ++i){
        parts << QStringLiteral("L%1=%2").arg(i + 1).arg(cableLen[i], 0, 'f', 6);
    }
    return parts.join(QStringLiteral(", "));
}

void MainWindow::logTrajectoryEndCableLengthDiagnostics()
{
    std::vector<double> targetPose = lastPlannedPoseTrajectoryEndPose;
    QString targetSource = QStringLiteral("规划终点");
    if(targetPose.size() < 6){
        targetPose = {
            ui->mainPosModeTargetEndPx->value(),
            ui->mainPosModeTargetEndPy->value(),
            ui->mainPosModeTargetEndPz->value(),
            ui->mainPosModeTargetEndRx->value(),
            ui->mainPosModeTargetEndRy->value(),
            ui->mainPosModeTargetEndRz->value()
        };
        targetSource = QStringLiteral("界面终点");
    }
    if(targetPose.size() < 6){
        displayInfo("轨迹终点绳长诊断：无法获取终点目标位姿", "warning");
        return;
    }

    if(ui->devUseLS->isChecked() && hardwareInterface.isLSConnected()){
        const std::vector<double> absPos = hardwareInterface.getAllMotorPos();
        if(!absPos.empty()){
            applyMotorPositionSnapshot(absPos);
        }
    }
    else if(controlWorker){
        const ControlWorker::Snapshot snapshot = controlWorker->latestSnapshot();
        if(!snapshot.motorAbsPos.empty()){
            applyMotorPositionSnapshot(snapshot.motorAbsPos, snapshot.motorRelRawPos);
        }
    }

    const QVector<double> theoreticalCableLen = buildGeometricCableLengthForPlot({targetPose});
    displayInfo(QStringLiteral("轨迹终点绳长诊断：%1位姿=(%2, %3, %4, %5, %6, %7)")
                .arg(targetSource)
                .arg(targetPose[0], 0, 'f', 6)
                .arg(targetPose[1], 0, 'f', 6)
                .arg(targetPose[2], 0, 'f', 6)
                .arg(targetPose[3], 0, 'f', 6)
                .arg(targetPose[4], 0, 'f', 6)
                .arg(targetPose[5], 0, 'f', 6)
                .toStdString());
    displayInfo(QStringLiteral("轨迹终点理论8根绳长(mm)：%1")
                .arg(formatCableLengthVector(theoreticalCableLen))
                .toStdString());

    std::vector<std::vector<double>> currentCableLenByEnd;
    if(!buildCurrentCableLengths(currentCableLenByEnd)){
        displayInfo("轨迹终点当前8根绳长(mm)：无法生成，请检查预紧确认、homeCableLen/cableHomePos 和电机位置反馈", "warning");
        return;
    }

    const QVector<double> currentCableLen = flattenCableLengthsByAxisOrder(currentCableLenByEnd);
    QVector<double> deltaCableLen(8, 0.0);
    double maxAbsDelta = 0.0;
    for(int i=0; i<deltaCableLen.size(); ++i){
        deltaCableLen[i] = currentCableLen[i] - theoreticalCableLen[i];
        maxAbsDelta = std::max(maxAbsDelta, std::abs(deltaCableLen[i]));
    }

    displayInfo(QStringLiteral("轨迹终点当前8根绳长(mm)：%1")
                .arg(formatCableLengthVector(currentCableLen))
                .toStdString());
    displayInfo(QStringLiteral("轨迹终点绳长差 当前-理论(mm)：%1；最大绝对差=%2")
                .arg(formatCableLengthVector(deltaCableLen))
                .arg(maxAbsDelta, 0, 'f', 6)
                .toStdString());
}

double MainWindow::convertMotorFeedbackToCableValue(int axisIndex, double rawValue) const
{
    if(axisIndex < 0 || axisIndex >= static_cast<int>(axisMotorCofVec.size()) || !isModeledMotorAxis(axisIndex)){
        return rawValue;
    }

    const double angleScale = motorCableAngleScale(axisIndex);
    if(std::abs(angleScale) < 1e-9){
        return rawValue;
    }
    const double hardwareDirection = motorHardwareDirectionSign(axisIndex);

    if(ui->devMotorFeedbackIsRd->isChecked()){
        return -hardwareDirection * rawValue * 2.0 * PI / angleScale;
    }
    if(ui->devMotorFeedbackIsTheta->isChecked()){
        return -hardwareDirection * rawValue * PI / (180.0 * angleScale);
    }
    return rawValue;
}

double MainWindow::motorCableAngleScale(int axisIndex) const
{
    if(axisIndex < 0 || axisIndex >= static_cast<int>(axisMotorCofVec.size()) || !axisMotorCofVec[axisIndex]){
        return 0.0;
    }
    const double configuredValue = std::abs(axisMotorCofVec[axisIndex]->value());
    if(configuredValue < 1e-9){
        return 0.0;
    }

    // 兼容旧配置：<=1 按 rad/mm；>1 按卷筒半径 mm。
    if(configuredValue > 1.0){
        return 1.0 / configuredValue;
    }
    return configuredValue;
}

double MainWindow::motorHardwareDirectionSign(int axisIndex) const
{
    if(axisIndex < 0 || axisIndex >= static_cast<int>(axisMotorCofVec.size()) || !axisMotorCofVec[axisIndex]){
        return 1.0;
    }
    return axisMotorCofVec[axisIndex]->value() < 0.0 ? -1.0 : 1.0;
}

//此函数用于设定系统初始参数：框架尺寸，轴数，传感器零漂等，以及连接信号与槽函数。返回值表示是否初始化成功，成功则返回true，失败则返回false。
bool MainWindow::initPara(){
    connect(ui->clearMsgBtn,&IntBtn::sendInt,this,&MainWindow::clearInfo);
    ui->tabWidget->setCurrentIndex(0);
    setupDataVisualizationTab();
    setupSingleMotorTrajectoryTrackingTab();
    setupMotorTorqueTestTab();
    initializeParameterConfigUi();
    initializeForceTraceTestUi();

    connect(ui->devUpdateParaBtn,&IntBtn::sendInt,this,&MainWindow::init3DViewer);// 更新模块数量后，模型也需要更新
    connect(ui->devUpdateParaBtn,&IntBtn::sendInt,this,&MainWindow::updatePara);
    connect(ui->devUseG3,&QRadioButton::clicked,this,&MainWindow::paraChange);
    connect(ui->devUseCamForActPose,&QCheckBox::toggled,this,&MainWindow::camLock);

    connect(ui->devFCKp,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,&MainWindow::resetFCPIDPara);
    connect(ui->devFCKi,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,&MainWindow::resetFCPIDPara);
    connect(ui->devFCKd,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,&MainWindow::resetFCPIDPara);
    connect(ui->devFCKp,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,&MainWindow::requestImmediateForceControlUpdate);
    connect(ui->devFCKi,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,&MainWindow::requestImmediateForceControlUpdate);
    connect(ui->devFCKd,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,&MainWindow::requestImmediateForceControlUpdate);
    connect(ui->devUsePID,&QRadioButton::toggled,this,[this](bool){
        requestImmediateForceControlUpdate();
    });
    connect(ui->devFCDeadbandPercent,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,&MainWindow::requestImmediateForceControlUpdate);
    if(QComboBox* forcePidOutputMode = findOptionalUiObject<QComboBox>(this, "devForceControlOutputMode")){
        QSignalBlocker blocker(forcePidOutputMode);
        forcePidOutputMode->clear();
        forcePidOutputMode->addItem(QStringLiteral("速度PID"), 0);
        forcePidOutputMode->addItem(QStringLiteral("力矩PID"), 1);
        forcePidOutputMode->setCurrentIndex(0);
        connect(forcePidOutputMode,
                QOverload<int>::of(&QComboBox::currentIndexChanged),
                this,
                [this](int){
            requestImmediateForceControlUpdate();
        });
    }
    if(QDoubleSpinBox* torqueLimit = findOptionalUiObject<QDoubleSpinBox>(this, "devFCTorqueLimitNm")){
        torqueLimit->setRange(0.0, 45.0);
        torqueLimit->setDecimals(3);
        torqueLimit->setSingleStep(0.1);
        torqueLimit->setValue(2.0);
        connect(torqueLimit,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [this](double){
            requestImmediateForceControlUpdate();
        });
    }
    if(QDoubleSpinBox* torqueSlew = findOptionalUiObject<QDoubleSpinBox>(this, "devFCTorqueSlewNmPerSec")){
        torqueSlew->setRange(0.0, 1000.0);
        torqueSlew->setDecimals(3);
        torqueSlew->setSingleStep(1.0);
        torqueSlew->setValue(20.0);
        connect(torqueSlew,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [this](double){
            requestImmediateForceControlUpdate();
        });
    }
    if(QDoubleSpinBox* torqueDamping =
            findOptionalUiObject<QDoubleSpinBox>(this, "devFCTorqueDampingNmPerVelocity")){
        torqueDamping->setRange(0.0, 1000.0);
        torqueDamping->setDecimals(4);
        torqueDamping->setSingleStep(0.01);
        torqueDamping->setValue(0.0);
        connect(torqueDamping,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [this](double){
            requestImmediateForceControlUpdate();
        });
    }


    // 参数设置
    ui->devCtrlCycleMs->setValue(10);
    hardwareInterface.threadCtrlCycleMs = (double)ui->devCtrlCycleMs->value();

    ui->devFrameL->setValue(6370.0);
    ui->devFrameW->setValue(6370.0);
    ui->devFrameH->setValue(5520.0);
    ui->devPulleyRadius->setValue(18.5);
    // ======

    ui->devEndNum->setRange(1,1);
    ui->devEndNum->setValue(1);
    ui->devEndNum->setEnabled(false);
    ui->devAxisNum->setRange(1, static_cast<int>(axisTypeVec.size()));
    ui->devAxisNum->setValue(12);// 默认包含8根绳索电机和4个直线模组电机
    ui->SinglemotorNum->setRange(0, std::max(0, ui->devAxisNum->value() - 1));
    ui->SinglemotorNum->setValue(0);
    for(QSpinBox* hardwareAxisSpinBox : axisMotorHardwareAxisVec){
        if(hardwareAxisSpinBox){
            hardwareAxisSpinBox->setRange(kLeadshineHardwareAxisMin, kLeadshineHardwareAxisMax);
        }
    }
    for(QDoubleSpinBox* forceMaxSpinBox : axisForceMaxVec){
        if(forceMaxSpinBox){
            forceMaxSpinBox->setValue(kDefaultAxisForceMaxN);
        }
    }
    for(QDoubleSpinBox* motorVelMaxSpinBox : axisMotorVelMaxVec){
        if(motorVelMaxSpinBox){
            motorVelMaxSpinBox->setValue(kDefaultMotorVelLimitRevPerSec);
        }
    }
    ui->devFCInitForce->setValue(kDefaultCablePretensionN);
    for(QDoubleSpinBox* forceExpectSpinBox : mainForceSensorExp){
        if(forceExpectSpinBox){
            forceExpectSpinBox->setValue(kDefaultCablePretensionN);
        }
    }
    connect(ui->devAxisNum,QOverload<int>::of(&QSpinBox::valueChanged),this,[this](int axisCount){
        ui->SinglemotorNum->setRange(0, std::max(0, axisCount - 1));
        updateSingleMotorActualPos();
        reinitializeDataVisualizationPlots();
        refreshSingleMotorTrajectoryAxisOptions();
        refreshMotorTorqueAxisOptions();
    });

    // ***添加传感器零漂固定值
    ui->axisIsStaticSensorHome->setChecked(false); // 设置为默认不勾选
    // 1.3 0.5 1.2 0.7 1.4 0.7 1.0 1.3
    // 0.9 0.2 0.8 0.2 0.9 0.3 0.7 1.0
    // 0.6 0.0 0.4 0.0 0.4 0.0 0.9 0.8
    // 0.2 -0.1 0.3 -0.2 0.1 -0.1 0.0 0.6
    // 0.2 -0.1 0.3 -0.2 0.1 -0.2 0.0 0.5
    // 0.7 0.0 0.5 0.0 0.3 0.0 0.1 0.7
    // 0.6 0.0 0.5 -0.1 0.2 0.0 0.1 0.6
    std::vector<double> sensorZerovalues = {0.5, 0.0, 0.4, 0.0, 0.3, 0.0, 0.1, 0.6};
    for(int i=0;i<sensorZerovalues.size();++i)
    {
        axisSensorZeroVec[i]->setValue(sensorZerovalues[i]);
    }

    connect(&hardwareInterface,&HardwareInterface::displayInfoSignal,this,&MainWindow::displayInfo);

    // === Main ===
    connect(ui->mainSwitch,&IntBtn::sendInt,this,&MainWindow::runSwitch);
    connect(ui->mainStopSwitch,&IntBtn::sendInt,this,&MainWindow::motorStop);
    connect(ui->mainMotor2Home,&IntBtn::sendInt,this,&MainWindow::motor2Home);
    connect(ui->mainSetCurrentMotorZeroButton, &QPushButton::clicked,
            this, &MainWindow::setAllMotorHomeToCurrentPosition);
    connect(ui->mainResetCableForceHomeSwitch,&IntBtn::sendInt,this,&MainWindow::resetCableForceHome);
    connect(ui->mainSetCableHome,&IntBtn::sendInt,this,[this](int){
        confirmZeroCalibrationWorkflow();
    });
    connect(ui->mainReplay3DViewer,&IntBtn::sendInt,this,&MainWindow::replay3DViewer);
    connect(ui->mainReadTrajFileTrigger,&IntBtn::sendInt,this,&MainWindow::readTrajFile);
    connect(ui->mainPosModeSwitch,&IntBtn::sendInt,this,&MainWindow::handleRunModePrimaryAction);
    if(IntBtn* gcModeSwitch = findOptionalUiObject<IntBtn>(this, "mainGCModeSwitch")){
        connect(gcModeSwitch,&IntBtn::sendInt,this,&MainWindow::runGCMode);
    }
    connect(ui->mainPosModeSend,&IntBtn::sendInt,this,&MainWindow::handleRunModeSecondaryAction);
    connect(ui->mainHybridPoseForceModeSwitch,&QPushButton::toggled,this,[this](bool checked){
        updateHybridPoseForceModeButtonText();
        if(checked){
            const int cableA = ui->mainHybridPoseForceCableA ? ui->mainHybridPoseForceCableA->value() : 7;
            const int cableB = ui->mainHybridPoseForceCableB ? ui->mainHybridPoseForceCableB->value() : 8;
            displayInfo(QStringLiteral("已切换到力位混合控制模式：执行时将使用第%1、第%2根绳索做力控")
                        .arg(cableA)
                        .arg(cableB)
                        .toStdString());
        }
        else{
            displayInfo("已退出力位混合控制模式");
        }
    });
    updateSingleCableForceDebugUiState();
    connect(ui->mainSingleCableForceDebugSwitch,&QPushButton::toggled,this,[this](bool checked){
        updateSingleCableForceDebugUiState();
        requestImmediateForceControlUpdate();
        updateSafetyMonitorConfig();
        displayInfo(checked ?
                    QStringLiteral("已进入单绳力控调试状态：最多允许一个力控绳索，已旁路松弛和位姿反馈安全触发")
                        .toStdString() :
                    QStringLiteral("已退出单绳力控调试状态，恢复常规力控选择和安全监控")
                        .toStdString());
    });
    auto updateMotorTorqueRealtimeUi = [this](bool checked){
        ui->mainMotorTorqueRealtimeButton->setText(checked ?
                    QStringLiteral("实时转矩：开") :
                    QStringLiteral("实时转矩：关"));
        for(QDoubleSpinBox* torqueData : mainMotorTorqueData){
            if(torqueData){
                torqueData->setEnabled(checked);
            }
        }
    };
    updateMotorTorqueRealtimeUi(ui->mainMotorTorqueRealtimeButton->isChecked());
    connect(ui->mainMotorTorqueRealtimeButton, &QPushButton::toggled, this,
            [this, updateMotorTorqueRealtimeUi](bool checked){
        updateMotorTorqueRealtimeUi(checked);
        updateControlWorkerConfig();
    });
    connect(ui->motorTorqueLimitEnable, &QCheckBox::toggled, this, [this](bool){
        updateControlWorkerConfig();
    });
    connect(ui->motorTorqueLimitNm, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double){
        updateControlWorkerConfig();
    });
    ui->Poscontinuetraj->setChecked(false);
    optionalCheckBoxSetChecked(this, "Forcecontinuetraj", false);
    connect(ui->mainAllUseFC,&QRadioButton::clicked,this,[this](bool checked){
        if(checked && isSingleCableForceDebugModeActive()){
            ui->mainAllUseFC->setChecked(false);
            enforceSingleCableForceSelection();
            displayInfo("单绳力控调试状态下不允许完全力控，请最多选择一根绳索", "warning");
            requestImmediateForceControlUpdate();
            return;
        }
        if(checked && runtimeState.cableHomeState != CableHomeState::Confirmed){
            runtimeState.cableHomeState = CableHomeState::WaitingForceStable;
            updateCableHomeConfirmEnabled();
        }
        allUseFC(checked);
    });
    for(QRadioButton* forceControlButton : useFCBtnVec){
        connect(forceControlButton,&QRadioButton::toggled,this,[this, forceControlButton](bool){
            if(suppressSingleCableForceSelectionUpdate){
                return;
            }
            if(isSingleCableForceDebugModeActive()){
                enforceSingleCableForceSelection(forceControlButton);
                syncSingleCableForceDebugPretensionForce();
            }
            requestImmediateForceControlUpdate();
        });
    }
    for(QDoubleSpinBox* expectedForceSpinBox : mainForceSensorExp){
        if(!expectedForceSpinBox){
            continue;
        }
        connect(expectedForceSpinBox,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this,
                [this](double){
            if(isSingleCableForceDebugModeActive()){
                syncSingleCableForceDebugPretensionForce();
                requestImmediateForceControlUpdate();
            }
        });
    }
    connect(ui->mainFCThread,&QCheckBox::toggled,this,[this](bool checked){
        if(checked){
            if(runtimeState.cableHomeState != CableHomeState::Confirmed){
                runtimeState.cableHomeState = CableHomeState::WaitingForceStable;
                updateCableHomeConfirmEnabled();
            }
        }
        requestImmediateForceControlUpdate();
    });
    connect(ui->mainDevInfoOutputTrigger,&IntBtn::sendInt,this,&MainWindow::outputInfoSendTrigger);
    if(IntBtn* gcResetRotHome = findOptionalUiObject<IntBtn>(this, "mainGCResetRotHome")){
        connect(gcResetRotHome,&IntBtn::sendInt,this,&MainWindow::resetRotHome);
    }
    connect(ui->motorPosZeronow,&QPushButton::clicked,this,&MainWindow::resetMotorPosDisplayZero);
    connect(ui->Absmotorpos,&QCheckBox::toggled,this,&MainWindow::refreshMotorPosDisplay);
    connect(ui->SinglemotorEnable,&QPushButton::clicked,this,&MainWindow::singleMotorEnable);
    connect(ui->SinglemotorStart,&QPushButton::clicked,this,&MainWindow::singleMotorStart);
    connect(ui->SinglemotorStop,&QPushButton::clicked,this,&MainWindow::singleMotorStop);
    connect(ui->SinglemotorUpdatePos,&QPushButton::clicked,this,&MainWindow::singleMotorUpdatePos);
    connect(ui->SinglemotorNum,QOverload<int>::of(&QSpinBox::valueChanged),this,[this](int){
        updateSingleMotorActualPos();
        refreshSingleMotorEnableStateUi();
    });
    refreshSingleMotorEnableStateUi();
    connect(ui->devMotorFeedbackIsTheta,&QRadioButton::toggled,this,[this](bool checked){
        if(checked){
            if(suppressMotorLimitUnitConversion){
                lastMotorFeedbackDisplayUnit = currentMotorFeedbackDisplayUnit();
                return;
            }
            refreshMotorLimitUnitUi(true);
            if(applyLeadshineAxisEquivFromUi()){
                updateControlWorkerConfig();
            }
        }
    });
    connect(ui->devMotorFeedbackIsRd,&QRadioButton::toggled,this,[this](bool checked){
        if(checked){
            if(suppressMotorLimitUnitConversion){
                lastMotorFeedbackDisplayUnit = currentMotorFeedbackDisplayUnit();
                return;
            }
            refreshMotorLimitUnitUi(true);
            if(applyLeadshineAxisEquivFromUi()){
                updateControlWorkerConfig();
            }
        }
    });
    refreshMotorLimitUnitUi(false);
    initializePrimaryOperationUi();
    initializeConnectionStatusUi();
    initializeCalibrationUi();
    initializeTrajectoryModeBoxes();
    initializeRunModeControls();
    initializeSessionRecordingUi();
    initializeSafetyUiControls();
    initializeFaultInjectionUiControls();
    initializeUdpBridge();
    refreshHybridPoseForceSelectionUi();
    updateHybridPoseForceModeButtonText();
    connect(ui->PostrajMode, QOverload<const QString&>::of(&QComboBox::currentTextChanged), this, &MainWindow::onPosTrajModeChanged);
    updateCableHomeConfirmEnabled();

    ccThread = new QThread(this);
    controlWorker = new ControlWorker(&hardwareInterface);
    controlWorker->moveToThread(ccThread);
    connect(ccThread, &QThread::started, controlWorker, &ControlWorker::start);
    connect(ccThread, &QThread::finished, controlWorker, &ControlWorker::stop);
    connect(controlWorker, &ControlWorker::displayInfoSignal, this, &MainWindow::displayInfo);
    connect(controlWorker, &ControlWorker::actualTorqueLimitExceeded, this,
            [this](int axisIndex, double actualTorqueNm, double limitNm){
        const QString summary = QStringLiteral("电机%1实际转矩达到限幅").arg(axisIndex);
        const QString detail = QStringLiteral(
                    "ControlWorker通过nmc_get_torque持续读取电机实际转矩，检测到电机%1实际转矩%2 N·m达到或超过限幅%3 N·m（额定转矩45 N·m），已触发软件急停。")
                .arg(axisIndex)
                .arg(actualTorqueNm, 0, 'f', 3)
                .arg(limitNm, 0, 'f', 3);
        handleSafetyFault(static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop),
                          static_cast<int>(SafetyMonitor::FaultCode::MotorTorqueExceeded),
                          summary,
                          detail);
    }, Qt::QueuedConnection);

    controlSnapshotTimer = new QTimer(this);
    controlSnapshotTimer->setInterval(50);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::updateControlWorkerConfig);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::applyControlSnapshot);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::refreshForwardKinematicsPoseDisplay);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::updateCableHomeConfirmEnabled);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::refreshRunModeUiStateThrottled);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::updateSafetyMonitorConfig);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::updateUdpStatusPayload);
    connect(controlSnapshotTimer, &QTimer::timeout, this, &MainWindow::updateRuntimeDiagnostics);
    controlSnapshotTimer->start();

    safetyHeartbeatTimer = new QTimer(this);
    safetyHeartbeatTimer->setTimerType(Qt::PreciseTimer);
    safetyHeartbeatTimer->setInterval(100);
    connect(safetyHeartbeatTimer, &QTimer::timeout, this, [this](){
        refreshSafetyMonitorHeartbeat();
    });
    safetyHeartbeatTimer->start();

    resetRuntimeDiagnosticsState(true);

    startSafetyMonitor();

    positionThread = new QThread(this);
    pvtExecutionWorker = new PvtExecutionWorker(&hardwareInterface);
    pvtExecutionWorker->moveToThread(positionThread);
    connect(pvtExecutionWorker, &PvtExecutionWorker::displayInfoSignal, this, &MainWindow::displayInfo);
    positionThread->start();

    ui->mainPosModeTargetStartPx->setValue(kGlobalInitialPosePxMm);
    ui->mainPosModeTargetStartPy->setValue(kGlobalInitialPosePyMm);
    ui->mainPosModeTargetStartPz->setValue(kGlobalInitialPosePzMm);
    ui->mainPosModeTargetStartRx->setValue(kGlobalInitialPoseRxRad);
    ui->mainPosModeTargetStartRy->setValue(kGlobalInitialPoseRyRad);
    ui->mainPosModeTargetStartRz->setValue(kGlobalInitialPoseRzRad);
    ui->mainPosModeTargetEndPx->setValue(-1450);// 200
    ui->mainPosModeTargetEndPy->setValue(0);// 200
    ui->mainPosModeTargetEndPz->setValue(250);// 600
    ui->mainPosModeTargetEndRz->setValue(0.0);// 10.0*PI/180.0
    ui->mainPosModeTargetRunTime->setValue(8);
    ui->mainPosModeTargetStepTime->setValue(0.1);

    optionalSpinBoxSetValue(this, "mainGCEndZ", 10.482);// 20.482



    ui->devUseG3->setChecked(true);
    paraChange();
    refreshAxisRuntimeCounts();

    if(!init3DViewer())
        return false;
    startMonitor();
    
    // 轨迹完成检测定时器
    trajectoryCheckTimer = new QTimer(this);
    connect(trajectoryCheckTimer, &QTimer::timeout, this, &MainWindow::checkTrajectoryFinished);
    
    return true;
}

QString MainWindow::motorAxisText() const{
    return QStringLiteral("绳索电机");
}

QString MainWindow::linearMotorAxisText() const{
    return QStringLiteral("直线模组电机");
}

MainWindow::MotorFeedbackDisplayUnit MainWindow::currentMotorFeedbackDisplayUnit() const
{
    if(ui->devMotorFeedbackIsTheta && ui->devMotorFeedbackIsTheta->isChecked()){
        return MotorFeedbackDisplayUnit::Degree;
    }
    return MotorFeedbackDisplayUnit::Revolution;
}

void MainWindow::refreshMotorLimitUnitUi(bool convertValues)
{
    const MotorFeedbackDisplayUnit currentUnit = currentMotorFeedbackDisplayUnit();
    if(convertValues &&
            !suppressMotorLimitUnitConversion &&
            currentUnit != lastMotorFeedbackDisplayUnit){
        const double scale = currentUnit == MotorFeedbackDisplayUnit::Degree ? 360.0 : (1.0 / 360.0);
        auto rescaleSpinBoxes = [scale](const std::vector<QDoubleSpinBox*>& spinBoxes){
            for(QDoubleSpinBox* spinBox : spinBoxes){
                if(!spinBox){
                    continue;
                }
                spinBox->setValue(spinBox->value() * scale);
            }
        };
        rescaleSpinBoxes(axisMotorMinVec);
        rescaleSpinBoxes(axisMotorMaxVec);
        rescaleSpinBoxes(axisMotorVelMaxVec);
    }

    lastMotorFeedbackDisplayUnit = currentUnit;
    const QString positionUnitText = currentUnit == MotorFeedbackDisplayUnit::Degree ?
                QStringLiteral("deg") : QStringLiteral("rev");
    const QString velocityUnitText = currentUnit == MotorFeedbackDisplayUnit::Degree ?
                QStringLiteral("deg/s") : QStringLiteral("rev/s");
    if(ui->label_355){
        ui->label_355->setText(QStringLiteral("位置上限(input, %1)").arg(positionUnitText));
    }
    if(ui->label_356){
        ui->label_356->setText(QStringLiteral("位置下限(input, %1)").arg(positionUnitText));
    }
    if(ui->label_357){
        ui->label_357->setText(QStringLiteral("速度上限(input, %1)").arg(velocityUnitText));
    }
    if(motorTorqueTestTab){
        syncMotorTorqueLimitUiFromAxis(selectedMotorTorqueAxisIndex());
        updateMotorTorqueWorkerConfig();
    }
    if(singleMotorTrajectoryTab){
        refreshSingleMotorTrajectoryUnitUi();
    }
}

bool MainWindow::isMotorAxis(int axisIndex) const{
    if(axisIndex < 0 || axisIndex >= static_cast<int>(axisTypeVec.size()) || !axisTypeVec[axisIndex]){
        return false;
    }

    const QString axisTypeText = axisTypeVec[axisIndex]->currentText().trimmed();
    if(!axisTypeText.isEmpty()){
        if(axisTypeText == QStringLiteral("/")){
            return false;
        }
        if(axisTypeText == motorAxisText() ||
                axisTypeText == linearMotorAxisText() ||
                axisTypeText.contains(QStringLiteral("电机"))){
            return true;
        }
        if(axisTypeVec[axisIndex]->isEditable()){
            return false;
        }
    }

    const QVariant axisTypeData = axisTypeVec[axisIndex]->currentData(Qt::UserRole);
    return axisTypeData.isValid() && axisTypeData.toInt() == COM_EC_LS;
}

bool MainWindow::isLinearMotorAxis(int axisIndex) const{
    if(axisIndex < 0 || axisIndex >= static_cast<int>(axisTypeVec.size()) || !axisTypeVec[axisIndex]){
        return false;
    }

    const QString axisTypeText = axisTypeVec[axisIndex]->currentText().trimmed();
    return axisTypeText == linearMotorAxisText() ||
            axisTypeText.contains(QStringLiteral("直线模组"));
}

bool MainWindow::isModeledMotorAxis(int axisIndex) const{
    return isMotorAxis(axisIndex) &&
            !isLinearMotorAxis(axisIndex) &&
            axisEndVec[axisIndex]->value() >= 0;
}

bool MainWindow::outputInfoSendTrigger(){
    static bool trigger = false;
    if(trigger){
        gIsLogging = false;
        if(gLogFile){
            gLogFile->close();
            delete gLogFile;
            gLogFile = nullptr;
        }
        ui->mainDevInfoOutputTrigger->setText("开始输出调试数据");
        trigger = false;
        displayInfo("调试数据输出已停止");
    }
    else{
        QDir().mkpath("D:/outputmsg");
        QString logPath = QString("D:/outputmsg/%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
        gLogFile = new QFile(logPath);
        if(!gLogFile->open(QIODevice::WriteOnly | QIODevice::Text)){
            displayInfo("错误：无法创建调试日志文件 " + logPath.toStdString(), "error");
            delete gLogFile;
            gLogFile = nullptr;
            return false;
        }
        gIsLogging = true;
        ui->mainDevInfoOutputTrigger->setText("停止输出调试数据");
        trigger = true;
        displayInfo("调试数据输出至 " + logPath.toStdString());
    }

    if(motiveLocalHandlerThread && motiveLocalHandlerThread->isInit)
        motiveLocalHandlerThread->detailInfo = trigger;

    return true;
}

void MainWindow::resetMotiveFit(){
    if(!motiveLocalHandlerThread || !motiveLocalHandlerThread->isInit){
        motiveFitConfirmed = false;
        displayInfo("错误：当前未连接有效动捕，无法更新零点位姿", "error");
        refreshCalibrationUiState();
        return;
    }

    delay(100);
    if(!motiveLocalHandlerThread->hasRecentRigidBody(500)){
        motiveFitConfirmed = false;
        displayInfo(QString("错误：最近 500ms 内无有效动捕刚体，当前检测到 %1 个点，需要 %2 个有效点")
                        .arg(motiveLocalHandlerThread->lastMarkerCount())
                        .arg(NokovRigidBodyFrame::REQUIRED_POINT_COUNT)
                        .toStdString(),
                    "error");
        refreshCalibrationUiState();
        return;
    }
    std::vector<std::vector<double>> rigidPose = motiveLocalHandlerThread->getRigidPose();
    if(rigidPose.empty() || rigidPose[0].size() < 6){
        motiveFitConfirmed = false;
        displayInfo("错误：当前动捕刚体位姿无效，无法更新动捕零位", "error");
        refreshCalibrationUiState();
        return;
    }
    std::vector<std::vector<QDoubleSpinBox*>> pos = {{ui->mainPosModeTargetStartPx,ui->mainPosModeTargetStartPy,ui->mainPosModeTargetStartPz}};
    std::vector<std::vector<QDoubleSpinBox*>> endPos = {{ui->mainPosModeTargetEndPx,ui->mainPosModeTargetEndPy,ui->mainPosModeTargetEndPz}};
    std::vector<std::vector<QDoubleSpinBox*>> rot = {{ui->mainPosModeTargetStartRx,ui->mainPosModeTargetStartRy,ui->mainPosModeTargetStartRz}};
    std::vector<std::vector<QDoubleSpinBox*>> endRot = {{ui->mainPosModeTargetEndRx,ui->mainPosModeTargetEndRy,ui->mainPosModeTargetEndRz}};

    while(rigidPose.size() > ui->devEndNum->value()){// 当动捕获取的刚体位姿数量比设定的值大时，剔除多余的（排序靠后的）以防报错
        rigidPose.pop_back();
    }

    for(int i=0;i<rigidPose.size();++i){
        for(int ii=0;ii<6;++ii){
            if(ii<3){
                pos[i][ii]->setValue(rigidPose[i][ii]);
                endPos[i][ii]->setValue(rigidPose[i][ii]);
            }
            else{
                rot[i][ii-3]->setValue(rigidPose[i][ii]);
                endRot[i][ii-3]->setValue(rigidPose[i][ii]);
            }
        }
    }
    ui->devEndHomeX->setValue(rigidPose[0][0]);
    ui->devEndHomeY->setValue(rigidPose[0][1]);
    ui->devEndHomeZ->setValue(rigidPose[0][2]);
    ui->devEndHomeRx->setValue(rigidPose[0][3]);
    ui->devEndHomeRy->setValue(rigidPose[0][4]);
    ui->devEndHomeRz->setValue(rigidPose[0][5]);
    endCurRotHome = {rigidPose[0][3], rigidPose[0][4], rigidPose[0][5]};
    motiveFitConfirmed = !rigidPose.empty();
    if(motiveFitConfirmed){
        updateControlWorkerConfig();
        displayInfo("零位校准：空间基准已更新，当前位姿已同步到起点/终点参考和空间零位");
    }
    refreshCalibrationUiState();
}

bool MainWindow::hasValidMotivePose() const{
    if(!ui->devUseCamForActPose->isChecked()){
        return true;
    }
    return hasRecentMotivePose(500);
}

bool MainWindow::hasValidEstimatedEndPose(int maxAgeMs) const
{
    std::vector<double> actualPose;
    return currentEstimatedEndPose(actualPose, maxAgeMs);
}

bool MainWindow::hasRecentMotivePose(int maxAgeMs) const
{
    if(!ui->devUseCamForActPose->isChecked()){
        return false;
    }
    if(!motiveLocalHandlerThread || !motiveLocalHandlerThread->isInit){
        return false;
    }
    if(maxAgeMs >= 0 && !motiveLocalHandlerThread->hasRecentRigidBody(maxAgeMs)){
        return false;
    }

    const std::vector<std::vector<double>> rigidPose = motiveLocalHandlerThread->getRigidPose();
    return !rigidPose.empty() && rigidPose[0].size() >= 6;
}

bool MainWindow::shouldUseForwardKinematicsFallback(int maxAgeMs) const
{
    Q_UNUSED(maxAgeMs);
    if(!ui->devUseCamForActPose->isChecked()){
        return true;
    }
    return false;
}

bool MainWindow::currentForwardKinematicsEndPose(std::vector<double>& pose, int maxAgeMs) const
{
    pose.clear();
    if(lastForwardKinematicsPose.empty() || lastForwardKinematicsPose.front().size() < 6){
        return false;
    }
    if(maxAgeMs >= 0 && lastForwardKinematicsPoseTimestampMs >= 0){
        const qint64 ageMs = QDateTime::currentMSecsSinceEpoch() - lastForwardKinematicsPoseTimestampMs;
        if(ageMs > maxAgeMs){
            return false;
        }
    }

    pose.assign(lastForwardKinematicsPose.front().begin(),
                lastForwardKinematicsPose.front().begin() + 6);
    return true;
}

void MainWindow::cacheForwardKinematicsPose(const std::vector<std::vector<double>>& platformPose)
{
    if(platformPose.empty() || platformPose.front().size() < 6){
        return;
    }
    lastForwardKinematicsPose = platformPose;
    lastForwardKinematicsPoseTimestampMs = QDateTime::currentMSecsSinceEpoch();
    refreshForwardKinematicsPoseDisplay();
}

void MainWindow::refreshForwardKinematicsPoseDisplay()
{
    const char* spinBoxNames[6] = {
        "mainForwardKinematicsPosePx",
        "mainForwardKinematicsPosePy",
        "mainForwardKinematicsPosePz",
        "mainForwardKinematicsPoseRx",
        "mainForwardKinematicsPoseRy",
        "mainForwardKinematicsPoseRz"
    };

    std::vector<double> pose;
    const bool hasPose = currentForwardKinematicsEndPose(pose, -1) && pose.size() >= 6;
    if(hasPose){
        for(int i=0; i<6; ++i){
            optionalSpinBoxSetValue(this, spinBoxNames[i], pose[i]);
        }
    }

    QLabel* statusLabel = findOptionalUiObject<QLabel>(this, "mainForwardKinematicsPoseStatusLabel");
    if(!statusLabel){
        return;
    }

    if(!hasPose){
        statusLabel->setText(QStringLiteral("未生成正运动学位姿"));
        return;
    }

    if(lastForwardKinematicsPoseTimestampMs < 0){
        statusLabel->setText(QStringLiteral("有效"));
        return;
    }

    const qint64 ageMs = QDateTime::currentMSecsSinceEpoch() - lastForwardKinematicsPoseTimestampMs;
    statusLabel->setText(ageMs <= 1000 ?
                         QStringLiteral("有效，%1 ms 前更新").arg(ageMs) :
                         QStringLiteral("缓存超时，%1 ms 前更新").arg(ageMs));
}

std::vector<double> MainWindow::configuredCableHomePose() const
{
    return {
        ui->devEndHomeX->value(),
        ui->devEndHomeY->value(),
        ui->devEndHomeZ->value(),
        ui->devEndHomeRx->value(),
        ui->devEndHomeRy->value(),
        ui->devEndHomeRz->value()
    };
}

std::vector<std::vector<double>> MainWindow::configuredCableHomePlatformPose() const
{
    return {configuredCableHomePose()};
}

bool MainWindow::isCurrentMotorSnapshotAtCableHome(double toleranceMm) const
{
    if(runtimeState.cableHomeState != CableHomeState::Confirmed ||
            cableHomePos.empty() ||
            motorCurPos.empty()){
        return false;
    }

    int modeledCableIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(axisIndex >= static_cast<int>(motorCurPos.size()) ||
                modeledCableIndex >= static_cast<int>(cableHomePos.size())){
            return false;
        }
        if(std::abs(motorCurPos[axisIndex] - cableHomePos[modeledCableIndex]) > toleranceMm){
            return false;
        }
        ++modeledCableIndex;
    }

    return modeledCableIndex > 0;
}

bool MainWindow::buildSafetyWorkspaceBounds(WorkspaceBounds& bounds, QString* errorMessage) const
{
    const double frameL = ui->devFrameL->value();
    const double frameW = ui->devFrameW->value();
    const double frameH = ui->devFrameH->value();
    if(frameL <= 0.0 || frameW <= 0.0 || frameH <= 0.0){
        if(errorMessage){
            *errorMessage = QStringLiteral("安全工作空间配置无效：机架长宽高必须大于0");
        }
        return false;
    }

    // The frame zero is 40 mm above the actual lower safe workspace boundary.
    bounds.xMin = -frameL / 2.0;
    bounds.xMax = frameL / 2.0;
    bounds.yMin = -frameW / 2.0;
    bounds.yMax = frameW / 2.0;
    bounds.zMin = kSafetyWorkspaceZMinMm;
    bounds.zMax = frameH;

    const double baseMargin = std::min({frameL, frameW, frameH}) * 0.05;
    bounds.warningMargin = std::max(20.0, std::min(100.0, baseMargin));
    bounds.severeOverflow = std::max(60.0, bounds.warningMargin);
    return true;
}

bool MainWindow::currentEstimatedEndPose(std::vector<double>& pose, int maxAgeMs) const
{
    pose.clear();
    if(ui->devUseCamForActPose->isChecked()){
        if(!hasRecentMotivePose(maxAgeMs)){
            return false;
        }
        const std::vector<std::vector<double>> rigidPose = motiveLocalHandlerThread->getRigidPose();
        if(!rigidPose.empty() && rigidPose[0].size() >= 6){
            pose = rigidPose[0];
            return true;
        }
        return false;
    }
    return currentForwardKinematicsEndPose(pose, maxAgeMs);
}

bool MainWindow::currentActualEndPose(std::vector<double>& pose, int maxAgeMs) const
{
    return currentEstimatedEndPose(pose, maxAgeMs);
}

bool MainWindow::validatePoseWithinWorkspace(const std::vector<double>& pose,
                                             const WorkspaceBounds& bounds,
                                             QString& errorMessage,
                                             bool* nearBoundary,
                                             double* overflowAmount) const
{
    if(nearBoundary){
        *nearBoundary = false;
    }
    if(overflowAmount){
        *overflowAmount = 0.0;
    }

    if(pose.size() < 3){
        errorMessage = QStringLiteral("末端位姿数据不完整，无法进行工作空间校验");
        return false;
    }

    const double px = pose[0];
    const double py = pose[1];
    const double pz = pose[2];
    if(!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)){
        errorMessage = QStringLiteral("末端位姿数据无效，存在非数值坐标");
        return false;
    }

    const double overflowX = std::max(bounds.xMin - px, px - bounds.xMax);
    const double overflowY = std::max(bounds.yMin - py, py - bounds.yMax);
    const double overflowZ = std::max(bounds.zMin - pz, pz - bounds.zMax);
    const double maxOverflow = std::max({0.0, overflowX, overflowY, overflowZ});
    if(overflowAmount){
        *overflowAmount = maxOverflow;
    }

    if(maxOverflow > 0.0){
        errorMessage = QStringLiteral(
                    "末端位姿超出安全工作空间：px=%1, py=%2, pz=%3，不在 X[%4, %5], Y[%6, %7], Z[%8, %9] 内")
                .arg(px, 0, 'f', 3)
                .arg(py, 0, 'f', 3)
                .arg(pz, 0, 'f', 3)
                .arg(bounds.xMin, 0, 'f', 3)
                .arg(bounds.xMax, 0, 'f', 3)
                .arg(bounds.yMin, 0, 'f', 3)
                .arg(bounds.yMax, 0, 'f', 3)
                .arg(bounds.zMin, 0, 'f', 3)
                .arg(bounds.zMax, 0, 'f', 3);
        return false;
    }

    const double marginX = std::min(px - bounds.xMin, bounds.xMax - px);
    const double marginY = std::min(py - bounds.yMin, bounds.yMax - py);
    const double marginZ = std::min(pz - bounds.zMin, bounds.zMax - pz);
    if(nearBoundary){
        *nearBoundary = bounds.warningMargin > 0.0 &&
                std::min({marginX, marginY, marginZ}) <= bounds.warningMargin;
    }
    errorMessage.clear();
    return true;
}

bool MainWindow::validateTrajectoryWithinWorkspace(
        const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
        QString& errorMessage) const
{
    WorkspaceBounds bounds;
    if(!buildSafetyWorkspaceBounds(bounds, &errorMessage)){
        return false;
    }

    if(plannedEndTraj.size() != 1 ||
            plannedEndTraj[0].empty() ||
            plannedEndTraj[0][0].size() < 3 ||
            plannedEndTraj[0][0][0].empty()){
        errorMessage = QStringLiteral("工作空间校验失败：轨迹数据格式不完整");
        return false;
    }

    const std::vector<std::vector<double>>& poseTraj = plannedEndTraj[0][0];
    const int pointCount = static_cast<int>(poseTraj[0].size());
    const std::vector<double>* timeAxis = nullptr;
    if(plannedEndTraj[0].size() > 3 &&
            !plannedEndTraj[0][3].empty() &&
            static_cast<int>(plannedEndTraj[0][3][0].size()) == pointCount){
        timeAxis = &plannedEndTraj[0][3][0];
    }

    for(int dim=1; dim<3; ++dim){
        if(static_cast<int>(poseTraj[dim].size()) != pointCount){
            errorMessage = QStringLiteral("工作空间校验失败：轨迹点数量不一致");
            return false;
        }
    }

    for(int pointIndex=0; pointIndex<pointCount; ++pointIndex){
        std::vector<double> pose(6, 0.0);
        for(int dim=0; dim<6 && dim<static_cast<int>(poseTraj.size()); ++dim){
            if(pointIndex >= static_cast<int>(poseTraj[dim].size())){
                errorMessage = QStringLiteral("工作空间校验失败：轨迹维度长度不一致");
                return false;
            }
            pose[dim] = poseTraj[dim][pointIndex];
        }

        QString poseError;
        if(!validatePoseWithinWorkspace(pose, bounds, poseError)){
            if(timeAxis && pointIndex < static_cast<int>(timeAxis->size())){
                errorMessage = QStringLiteral("轨迹第%1个点（t=%2 s）越界：%3")
                        .arg(pointIndex + 1)
                        .arg((*timeAxis)[pointIndex], 0, 'f', 3)
                        .arg(poseError);
            }
            else{
                errorMessage = QStringLiteral("轨迹第%1个点越界：%2")
                        .arg(pointIndex + 1)
                        .arg(poseError);
            }
            return false;
        }
    }

    errorMessage.clear();
    return true;
}

bool MainWindow::extractTrajectoryEndpointPose(
        const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
        std::vector<double>& pose) const
{
    pose.clear();
    if(plannedEndTraj.size() != 1 ||
            plannedEndTraj[0].empty() ||
            plannedEndTraj[0][0].size() < 6 ||
            plannedEndTraj[0][0][0].empty()){
        return false;
    }

    const std::vector<std::vector<double>>& poseTraj = plannedEndTraj[0][0];
    const int lastPointIndex = static_cast<int>(poseTraj[0].size()) - 1;
    if(lastPointIndex < 0){
        return false;
    }

    pose.assign(6, 0.0);
    for(int dim=0; dim<6; ++dim){
        if(lastPointIndex >= static_cast<int>(poseTraj[dim].size()) ||
                !std::isfinite(poseTraj[dim][lastPointIndex])){
            pose.clear();
            return false;
        }
        pose[dim] = poseTraj[dim][lastPointIndex];
    }
    return true;
}

void MainWindow::applyTrajectoryStartPoseToUi(const std::vector<double>& pose)
{
    if(pose.size() < 6){
        return;
    }

    ui->mainPosModeTargetStartPx->setValue(pose[0]);
    ui->mainPosModeTargetStartPy->setValue(pose[1]);
    ui->mainPosModeTargetStartPz->setValue(pose[2]);
    ui->mainPosModeTargetStartRx->setValue(pose[3]);
    ui->mainPosModeTargetStartRy->setValue(pose[4]);
    ui->mainPosModeTargetStartRz->setValue(pose[5]);
}

void MainWindow::update3DViewer(QVector<QVector3D> targetPos, QVector<QVector3D> trajPos, QVector<QVector3D> anchorPos,
                                QVector<QVector3D> cablePos){
    if(use3DViewer && main3DViewer){
        bool shouldActivateViewer = false;
        if(!main3DViewer->isVisible()){
            main3DViewer->show();
            isViewShow = true;
            shouldActivateViewer = true;
        }
        else if(main3DViewer->isMinimized()){
            main3DViewer->showNormal();
            shouldActivateViewer = true;
        }
        if(shouldActivateViewer){
            main3DViewer->raise();
            main3DViewer->activateWindow();
        }
    }

    if(is3DViewerReadyForNextInput){
        if(!use3DViewer)
            return;
        is3DViewerReadyForNextInput = false;

    //    SYSTEMTIME systemTime;
    //    GetLocalTime(&systemTime);
    //    static double startTimeMs = (double)systemTime.wMilliseconds + (double)systemTime.wSecond*1000.0;
    //    double curTimeMs = (double)systemTime.wMilliseconds + (double)systemTime.wSecond*1000.0;
    //    if(curTimeMs-startTimeMs<1000.0)
    //        return;

        float anchor2CableCof = 100.0f;// 绳索绘制密度系数，越大越密
        int anchor2CableCountSum = 0.0;
        std::vector<int> anchor2CableCount(anchorPos.count());
        for(int i=0;i<anchorPos.count();++i){// 根据各锚点座和绳索接点之间的距离(m)规划绘制
            anchor2CableCount[i] = (int)(anchor2CableCof*MatrixFun::norm2(MatrixFun::vecDiff({anchorPos[i].x(),anchorPos[i].y(),anchorPos[i].z()},
                                                                                       {cablePos[i].x(),cablePos[i].y(),cablePos[i].z()})));
            anchor2CableCountSum += anchor2CableCount[i];
        }

        QScatterDataArray *dataArrayTarget = new QScatterDataArray();// 散点数据（储存绘图数据）
        QScatterDataArray *dataArrayTraj = new QScatterDataArray();
        QScatterDataArray *dataArrayAnchor = new QScatterDataArray();
        QScatterDataArray *dataArrayCable = new QScatterDataArray();
        QScatterDataArray *dataArrayAnchor2Cable = new QScatterDataArray();
        dataArrayTarget->resize(targetPos.count());
        dataArrayTraj->resize(trajPos.count());
        dataArrayAnchor->resize(anchorPos.count());
        dataArrayCable->resize(cablePos.count());
        dataArrayAnchor2Cable->resize(std::max(1,anchor2CableCountSum));

        QScatterDataItem *ptrToDataArrayTarget = &dataArrayTarget->first();// 散点数据指针（进行绘图操作）
        QScatterDataItem *ptrToDataArrayTraj = &dataArrayTraj->first();
        QScatterDataItem *ptrToDataArrayAnchor = &dataArrayAnchor->first();
        QScatterDataItem *ptrToDataArrayCable = &dataArrayCable->first();
        QScatterDataItem *ptrToDataArrayAnchor2Cable = &dataArrayAnchor2Cable->first();
    //    ptrToDataArrayTarget->setPosition(targetPos);
    //    ptrToDataArrayTraj->setPosition(trajPos);
    //    ptrToDataArrayAnchor->setPosition(anchorPos);
    //    ptrToDataArrayCable->setPosition(cablePos);
        for (int i = 0;i < targetPos.count();i++ ){
            ptrToDataArrayTarget->setPosition(to803Frame(targetPos[i]));
            ptrToDataArrayTarget++;
        }
        for (int i = 0;i < trajPos.count();i++ ){
            ptrToDataArrayTraj->setPosition(to803Frame(trajPos[i]));
            ptrToDataArrayTraj++;
        }
        for (int i = 0;i < anchorPos.count();i++ ){
            ptrToDataArrayAnchor->setPosition(to803Frame(anchorPos[i]));
            ptrToDataArrayAnchor++;
        }
        for (int i = 0;i < cablePos.count();i++ ){
            ptrToDataArrayCable->setPosition(to803Frame(cablePos[i]));
            ptrToDataArrayCable++;
        }
        for(int i = 0;i < anchorPos.count();i++){
            for(int ii=0;ii<anchor2CableCount[i];++ii){
                QVector3D tempCoor = anchorPos[i] + (cablePos[i]-anchorPos[i])*ii/anchor2CableCount[i];
                ptrToDataArrayAnchor2Cable->setPosition(to803Frame(tempCoor));
                ptrToDataArrayAnchor2Cable++;
            }
        }
        seriesTarget->dataProxy()->resetArray(dataArrayTarget);//更新散点
        seriesTraj->dataProxy()->resetArray(dataArrayTraj);
        seriesAnchor->dataProxy()->resetArray(dataArrayAnchor);
        seriesCable->dataProxy()->resetArray(dataArrayCable);
        seriesAnchor2Cable->dataProxy()->resetArray(dataArrayAnchor2Cable);

        if(!isReplaying){// 防止记录重播的操作
            preTargetPos.push_back(targetPos);// 记录，用于重播
            preTrajPos.push_back(trajPos);
            preAnchorPos.push_back(anchorPos);
            preCablePos.push_back(cablePos);
        }

        //    QVector<QVector3D> pointsFrame = {{2.0,2.0,2.0},{4.0,4.0,4.0}};
        //    QScatterDataArray *dataArrayFrame = new QScatterDataArray();
        //    dataArrayFrame->resize(pointsFrame.count());
        //    QScatterDataItem *ptrToDataArrayFrame = &dataArrayFrame->first();
        //    for (int i = 0;i < pointsFrame.count();i++ )
        //    {
        //        ptrToDataArrayFrame->setPosition(pointsFrame[i]);
        //        ptrToDataArrayFrame++;
        //    }
        //    seriesFrame->dataProxy()->resetArray(dataArrayFrame);//更新散点

        is3DViewerReadyForNextInput = true;
    }
}

bool MainWindow::init3DViewer(){
    if(runtimeState.systemRunning && main3DViewer){
        displayInfo("系统运行中已跳过3D视图重建；如需修改模块数量或机架尺寸，请先断连后再更新参数。", "warning");
        return true;
    }

    graph = new Q3DScatter();
    main3DViewer = QWidget::createWindowContainer(graph);
    main3DViewer->setWindowTitle(QStringLiteral("3D仿真视图"));
    main3DViewer->resize(960, 720);
    QScatterDataProxy *proxyFrame = new QScatterDataProxy(); //数据代理
    QScatterDataProxy *proxyTarget = new QScatterDataProxy();
    QScatterDataProxy *proxyTraj = new QScatterDataProxy();
    QScatterDataProxy *proxyAnchor = new QScatterDataProxy();
    QScatterDataProxy *proxyCable = new QScatterDataProxy();
    QScatterDataProxy *proxyAnchor2Cable = new QScatterDataProxy();

    double frameL(ui->devFrameL->value()/1000.0),
            frameW(ui->devFrameW->value()/1000.0),
            frameH(ui->devFrameH->value()/1000.0);
//    double range = std::max(std::max(frameL,frameW),frameH);

    //创建序列
    seriesFrame = new QScatter3DSeries(proxyFrame);
    seriesFrame->setMeshSmooth(true);
    seriesTarget = new QScatter3DSeries(proxyTarget);
    seriesTarget->setMeshSmooth(true);
    seriesTraj = new QScatter3DSeries(proxyTraj);
    seriesTraj->setMeshSmooth(true);
    seriesAnchor = new QScatter3DSeries(proxyAnchor);
    seriesAnchor->setMeshSmooth(true);
    seriesCable = new QScatter3DSeries(proxyCable);
    seriesCable->setMeshSmooth(true);
    seriesAnchor2Cable = new QScatter3DSeries(proxyAnchor2Cable);
    seriesAnchor2Cable->setMeshSmooth(true);
    graph->addSeries(seriesFrame);
    graph->addSeries(seriesTarget);
    graph->addSeries(seriesTraj);
    graph->addSeries(seriesAnchor);
    graph->addSeries(seriesCable);
    graph->addSeries(seriesAnchor2Cable);

    //创建坐标轴 AxisOrientation
    // 由于在默认视角中，QT坐标系的Y轴朝上，而真实坐标系是Z轴朝上，所以此处做一定更改
    // 同时，为了能让坐标正确显示，以下setPosition都需要搭配函数to803Frame，以适配这一更改
    // 但点的坐标还是按真实坐标系下的坐标输入
    graph->axisX()->setTitle("axis X");
    graph->axisY()->setTitle("axis Z");
    graph->axisZ()->setTitle("axis Y");
//    graph->axisZ()->setReversed(true);
    graph->axisX()->setTitleVisible(true);
    graph->axisY()->setTitleVisible(true);
    graph->axisZ()->setTitleVisible(true);
    graph->axisX()->setRange(-((float)((int)frameL+1))/2.0,((float)((int)frameL+1))/2.0);// 规整坐标轴范围
    graph->axisX()->setSegmentCount(((int)frameL+1)/0.5);// 以0.5m为单位分割坐标轴
    graph->axisY()->setRange(0.0,(float)((int)frameH+1));
    graph->axisY()->setSegmentCount(((int)frameH+1)/0.5);
    graph->axisZ()->setRange(-((float)((int)frameW+1))/2.0,((float)((int)frameW+1))/2.0);
    graph->axisZ()->setSegmentCount(((int)frameW+1)/0.5);
    graph->axisX()->setAutoAdjustRange(false);
    graph->axisY()->setAutoAdjustRange(false);
    graph->axisZ()->setAutoAdjustRange(false);

    graph->activeTheme()->setLabelBackgroundEnabled(false);
    graph->activeTheme()->setBackgroundColor(QColor(90,90,90));//设置背景色

    //数据点
    // 边框
    seriesFrame->setMesh(QAbstract3DSeries::MeshBar);//数据点为柱
    seriesFrame->setSingleHighlightColor(QColor(0,0,255));//设置点选中时的高亮颜色
    seriesFrame->setBaseColor(QColor(0,255,255));//设置点的颜色
    seriesFrame->setItemSize(0.25f);//设置点的大小 max:1.0f
    // 牵引物
    seriesTarget->setMesh(QAbstract3DSeries::MeshBar);
    seriesTarget->setSingleHighlightColor(QColor(255,0,255));
    seriesTarget->setBaseColor(QColor(255,0,0));
    seriesTarget->setItemSize(0.2f);
    // 轨迹
    seriesTraj->setMesh(QAbstract3DSeries::MeshBar);
    seriesTraj->setSingleHighlightColor(QColor(255,255,0));
    seriesTraj->setBaseColor(QColor(0,255,0));
    seriesTraj->setItemSize(0.2f);
    // 动锚点
    seriesAnchor->setMesh(QAbstract3DSeries::MeshBar);
    seriesAnchor->setSingleHighlightColor(QColor(255,0,0));
    seriesAnchor->setBaseColor(QColor(255,255,0));
    seriesAnchor->setItemSize(0.3f);
    // 绳索接点
    seriesCable->setMesh(QAbstract3DSeries::MeshBar);
    seriesCable->setSingleHighlightColor(QColor(0,0,0));
    seriesCable->setBaseColor(QColor(125,125,125));
    seriesCable->setItemSize(0.1f);// QH:0.2
    // 绳索
    seriesAnchor2Cable->setMesh(QAbstract3DSeries::MeshBar);
    seriesAnchor2Cable->setSingleHighlightColor(QColor(255,0,0));
    seriesAnchor2Cable->setBaseColor(QColor(255,125,0));
    seriesAnchor2Cable->setItemSize(0.02f);// QH:0.05

    // 模拟边框
    QVector<QVector3D> pointsFrame;
    float pDisX(seriesFrame->itemSize()*((float)frameL)/(25.0*0.25)), pDisY(seriesFrame->itemSize()*(float)frameW/(25.0*0.25)),
            pDisZ(seriesFrame->itemSize()*(float)frameH/(25.0*0.25));
    for(float xIndex = -frameL/2.0f; xIndex < frameL/2.0f; xIndex+=pDisX){
        pointsFrame.push_back({xIndex,(float)-frameW/2.0f,0.0f});
        pointsFrame.push_back({xIndex,(float)frameW/2.0f,0.0f});
        pointsFrame.push_back({xIndex,(float)-frameW/2.0f,(float)frameH});
        pointsFrame.push_back({xIndex,(float)frameW/2.0f,(float)frameH});
    }
    for(float yIndex = -frameW/2.0f; yIndex < frameW/2.0f; yIndex+=pDisY){
        pointsFrame.push_back({(float)-frameL/2.0f,yIndex,0.0f});
        pointsFrame.push_back({(float)frameL/2.0f,yIndex,0.0f});
        pointsFrame.push_back({(float)-frameL/2.0f,yIndex,(float)frameH});
        pointsFrame.push_back({(float)frameL/2.0f,yIndex,(float)frameH});
    }
    for(float zIndex = 0.0f; zIndex < frameH; zIndex+=pDisZ){
        pointsFrame.push_back({(float)-frameL/2.0f,(float)-frameW/2.0f,zIndex});
        pointsFrame.push_back({(float)-frameL/2.0f,(float)frameW/2.0f,zIndex});
        pointsFrame.push_back({(float)frameL/2.0f,(float)-frameW/2.0f,zIndex});
        pointsFrame.push_back({(float)frameL/2.0f,(float)frameW/2.0f,zIndex});
    }
    QScatterDataArray *dataArrayFrame = new QScatterDataArray();// 散点数据（储存绘图数据）
    dataArrayFrame->resize(pointsFrame.count());
    QScatterDataItem *ptrToDataArrayFrame = &dataArrayFrame->first();// 散点数据指针（进行绘图操作）

    // 绘制
    for (int i = 0;i < pointsFrame.count();i++ ){
        ptrToDataArrayFrame->setPosition(to803Frame(pointsFrame[i]));// 设置散点坐标
        ptrToDataArrayFrame++;
    }
    seriesFrame->dataProxy()->resetArray(dataArrayFrame);//更新散点

    // ====== TEST ======
    // 牵引物
    QVector<QVector3D> pointsTarget = {{0.0f,0.0f,1.0f}};
    QScatterDataArray *dataArrayTarget = new QScatterDataArray();
    dataArrayTarget->resize(pointsTarget.count());
    QScatterDataItem *ptrToDataArrayTarget = &dataArrayTarget->first();
    // 轨迹
    QVector<QVector3D> pointsTraj = {{0.0f,0.0f,1.0f},{-0.15f,-0.1f,1.1f},{-0.3f,-0.2f,1.2f},{-0.45f,-0.3f,1.3f},{-0.6f,-0.4f,1.4f},
                                    {-0.75f,-0.5f,1.5f},{-0.9f,-0.6f,1.6f},{-1.05f,-0.7f,1.7f},{-1.2f,-0.8f,1.8f},{-1.35f,-0.9f,1.9f}};
    QScatterDataArray *dataArrayTraj = new QScatterDataArray();
    dataArrayTraj->resize(pointsTraj.count());
    QScatterDataItem *ptrToDataArrayTraj = &dataArrayTraj->first();
    // 动锚点
    QVector<QVector3D> pointsAnchor = {{(float)frameL/4.0f,(float)frameW/4.0f,(float)frameH/2.0f},{(float)-frameL/4.0f,(float)-frameW/4.0f,0.0f}};
    QScatterDataArray *dataArrayAnchor = new QScatterDataArray();
    dataArrayAnchor->resize(pointsAnchor.count());
    QScatterDataItem *ptrToDataArrayAnchor = &dataArrayAnchor->first();
    // 绳索接点
    QVector<QVector3D> pointsCable = {{(float)frameL/8.0f,(float)frameW/8.0f,(float)frameH/4.0f},{(float)-frameL/8.0f,(float)-frameW/8.0f,0.0f}};
    QScatterDataArray *dataArrayCable = new QScatterDataArray();
    dataArrayCable->resize(pointsCable.count());
    QScatterDataItem *ptrToDataArrayCable = &dataArrayCable->first();

    // 绘制
    for (int i = 0;i < pointsTarget.count();i++ ){
        ptrToDataArrayTarget->setPosition(to803Frame(pointsTarget[i]));
        ptrToDataArrayTarget++;
    }
    for (int i = 0;i < pointsTraj.count();i++ ){
        ptrToDataArrayTraj->setPosition(to803Frame(pointsTraj[i]));
        ptrToDataArrayTraj++;
    }
    for (int i = 0;i < pointsAnchor.count();i++ ){
        ptrToDataArrayAnchor->setPosition(to803Frame(pointsAnchor[i]));
        ptrToDataArrayAnchor++;
    }
    for (int i = 0;i < pointsCable.count();i++ ){
        ptrToDataArrayCable->setPosition(to803Frame(pointsCable[i]));
        ptrToDataArrayCable++;
    }
    seriesTarget->dataProxy()->resetArray(dataArrayTarget);
    seriesTraj->dataProxy()->resetArray(dataArrayTraj);
    seriesAnchor->dataProxy()->resetArray(dataArrayAnchor);
    seriesCable->dataProxy()->resetArray(dataArrayCable);
    // ============

    is3DViewerReadyForNextInput = true;
    return true;
}

QVector3D MainWindow::to803Frame(QVector3D qtFrame){
    QVector3D result = qtFrame;
    result[1] = qtFrame[2];// y、z坐标互换
    result[2] = qtFrame[1];
    return result;
}

void MainWindow::displayInfo(std::string info, std::string type){
    const QString message = QString::fromStdString(info);
    const QString messageType = QString::fromStdString(type);

    if(messageType != QStringLiteral("reset")){
        appendMessageHistoryEntry(message, messageType);
        appendUiEventLog(message, messageType);
    }

    if(type == "reset")// 重置信息窗口
        hasReportErr = false;
    if(!hasReportErr || type == "error" || type == "reset"){// 仅没有任何程序发出过错误提示时才执行，防止错误提示被覆盖
        QPalette red,black;
        red.setColor(QPalette::Text, QColor(255, 0, 0));
        black.setColor(QPalette::Text, QColor(0, 0, 0));
        if(type == "normal")
            ui->infoDisplay->setPalette(black);
        else if(type == "warning")
            ui->infoDisplay->setPalette(black);
        else if(type == "error"){
            hasReportErr = true;
            ui->infoDisplay->setPalette(red);
        }
        else if(type == "reset"){
            ui->infoDisplay->setPalette(black);
        }
        ui->infoDisplay->setText(message);
    }
}

void MainWindow::clearInfo(){
    displayInfo("","reset");
}

void MainWindow::stopPoseThread(PositionSimulationModel*& thread){
    if(!thread){
        return;
    }

    thread->stopThread();
    if(thread->QThread::isRunning() && !thread->wait(500)){
        thread->terminate();
        thread->wait(500);
    }
    delete thread;
    thread = nullptr;
}

void MainWindow::stopGravityThread(){
    if(simulationWorker){
        simulationWorker->stopForceSimulation();
    }
    forceSimulationModel = nullptr;
    runtimeState.gcRunning = false;
    clearHybridPoseForceModeState(false);
}

void MainWindow::stopControlThread(){
    if(ccThread && ccThread->isRunning()){
        if(controlWorker){
            QMetaObject::invokeMethod(controlWorker,"stop",Qt::BlockingQueuedConnection);
        }
        ccThread->quit();
        if(!ccThread->wait(1000)){
            ccThread->terminate();
            ccThread->wait(500);
        }
    }
}

void MainWindow::setForceControlSelectionEnabled(bool enabled){
    forceControlSelectionUiEnabled = enabled;
    const bool singleCableDebug = isSingleCableForceDebugModeActive();
    ui->mainAllUseFC->setEnabled(enabled && !singleCableDebug);
    ui->mainFCThread->setEnabled(enabled);
    if(ui->mainSingleCableForceDebugSwitch){
        ui->mainSingleCableForceDebugSwitch->setEnabled(enabled);
    }
    if(ui->mainHybridPoseForceModeSwitch){
        ui->mainHybridPoseForceModeSwitch->setEnabled(enabled && !singleCableDebug);
    }
    if(ui->mainHybridPoseForceCableA){
        ui->mainHybridPoseForceCableA->setEnabled(enabled && !singleCableDebug);
    }
    if(ui->mainHybridPoseForceCableB){
        ui->mainHybridPoseForceCableB->setEnabled(enabled && !singleCableDebug);
    }
    for(QRadioButton* btn : useFCBtnVec){
        if(btn){
            btn->setEnabled(enabled);
        }
    }
}

void MainWindow::initializeTrajectoryModeBoxes(){
    const QString previousPoseMode = normalizePoseTrajectoryModeName(ui->PostrajMode->currentText());
    {
        const QSignalBlocker blocker(ui->PostrajMode);
        ui->PostrajMode->clear();
        ui->PostrajMode->addItems(builtInPoseTrajectoryModes());
        const int targetIndex = ui->PostrajMode->findText(previousPoseMode);
        ui->PostrajMode->setCurrentIndex(targetIndex >= 0 ? targetIndex : 0);
    }

    ui->posTrajParamTabWidget->tabBar()->hide();
    ui->posTrajParamTabWidget->setTabText(ui->posTrajParamTabWidget->indexOf(ui->tabQuintic), kPoseTrajStraight);
    ui->posTrajParamTabWidget->setTabText(ui->posTrajParamTabWidget->indexOf(ui->tabCircular), kPoseTrajCircular);
    ui->posTrajParamTabWidget->setTabText(ui->posTrajParamTabWidget->indexOf(ui->tabSCurve), kPoseTrajLineAccel);
    ui->posTrajParamTabWidget->setTabText(ui->posTrajParamTabWidget->indexOf(ui->tabEightShape), kPoseTrajEightShape);
    ui->posTrajParamTabWidget->setTabText(ui->posTrajParamTabWidget->indexOf(ui->tabStepAccel), kPoseTrajStepAccel);
    ui->posTrajParamTabWidget->setTabText(ui->posTrajParamTabWidget->indexOf(ui->tabSine), kPoseTrajSineAccel);
    ui->posTrajParamTabWidget->setTabText(ui->posTrajParamTabWidget->indexOf(ui->tabCubic), kPoseTrajCubic);
    setPoseSimulationIdleText(ui);
    onPosTrajModeChanged(ui->PostrajMode->currentText());

    if(QComboBox* forceTrajMode = findOptionalUiObject<QComboBox>(this, "ForcetrajMode")){
        if(forceTrajMode->count() == 0){
            forceTrajMode->addItem(QStringLiteral("直线"));
            forceTrajMode->addItem(QStringLiteral("圆弧"));
        }
    }
}

void MainWindow::onPosTrajModeChanged(const QString& mode){
    const QString normalizedMode = normalizePoseTrajectoryModeName(mode);
    bool needEndPose = true;
    bool needRunTime = true;
    
    if(normalizedMode == kPoseTrajStraight){
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabQuintic);
    } else if(normalizedMode == kPoseTrajCircular){
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabCircular);
        needEndPose = false;
    } else if(normalizedMode == kPoseTrajLineAccel){
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabSCurve);
    } else if(normalizedMode == kPoseTrajEightShape){
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabEightShape);
        needEndPose = false;
    } else if(normalizedMode == kPoseTrajStepAccel){
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabStepAccel);
        needEndPose = false;
        needRunTime = false;
    } else if(normalizedMode == kPoseTrajSineAccel){
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabSine);
    } else if(normalizedMode == kPoseTrajCubic){
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabCubic);
    } else {
        ui->posTrajParamTabWidget->setCurrentWidget(ui->tabCircular);
        needEndPose = false;
    }
    
    Q_UNUSED(needEndPose);
    Q_UNUSED(needRunTime);
    updatePoseInputUiState();
}

bool MainWindow::hasForceControlAxisSelected() const{
    for(QRadioButton* btn : useFCBtnVec){
        if(btn && btn->isChecked()){
            return true;
        }
    }
    return false;
}

bool MainWindow::isSingleCableForceDebugModeActive() const{
    return runtimeState.singleCableForceDebugMode ||
            (ui && ui->mainSingleCableForceDebugSwitch &&
             ui->mainSingleCableForceDebugSwitch->isChecked());
}

void MainWindow::enforceSingleCableForceSelection(QRadioButton* preferredButton){
    if(!isSingleCableForceDebugModeActive() ||
            suppressSingleCableForceSelectionUpdate ||
            useFCBtnVec.empty()){
        return;
    }

    suppressSingleCableForceSelectionUpdate = true;
    QRadioButton* selectedButton = nullptr;
    if(preferredButton &&
            useFCBtnVec.end() != std::find(useFCBtnVec.begin(), useFCBtnVec.end(), preferredButton) &&
            preferredButton->isChecked()){
        selectedButton = preferredButton;
    }
    if(!selectedButton){
        for(QRadioButton* btn : useFCBtnVec){
            if(btn && btn->isChecked()){
                selectedButton = btn;
                break;
            }
        }
    }

    for(QRadioButton* btn : useFCBtnVec){
        if(!btn){
            continue;
        }
        const bool shouldCheck = btn == selectedButton;
        if(btn->isChecked() != shouldCheck){
            btn->setChecked(shouldCheck);
        }
    }
    suppressSingleCableForceSelectionUpdate = false;
}

void MainWindow::syncSingleCableForceDebugPretensionForce(){
    if(!ui || !ui->devFCInitForce){
        return;
    }

    if(!isSingleCableForceDebugModeActive()){
        if(singleCableForceDebugPretensionSaved){
            const QSignalBlocker blocker(ui->devFCInitForce);
            ui->devFCInitForce->setValue(singleCableForceDebugSavedPretensionForce);
            singleCableForceDebugPretensionSaved = false;
        }
        return;
    }

    if(!singleCableForceDebugPretensionSaved){
        singleCableForceDebugSavedPretensionForce = ui->devFCInitForce->value();
        singleCableForceDebugPretensionSaved = true;
    }

    int selectedSensorIndex = -1;
    for(int i=0; i<static_cast<int>(useFCBtnVec.size()); ++i){
        if(useFCBtnVec[i] && useFCBtnVec[i]->isChecked()){
            selectedSensorIndex = i;
            break;
        }
    }
    if(selectedSensorIndex < 0 ||
            selectedSensorIndex >= static_cast<int>(mainForceSensorExp.size()) ||
            !mainForceSensorExp[selectedSensorIndex]){
        return;
    }

    const double expectedForce = mainForceSensorExp[selectedSensorIndex]->value();
    if(!std::isfinite(expectedForce)){
        return;
    }
    if(std::abs(ui->devFCInitForce->value() - expectedForce) <= 1e-9){
        return;
    }

    const QSignalBlocker blocker(ui->devFCInitForce);
    ui->devFCInitForce->setValue(expectedForce);
}

void MainWindow::updateSingleCableForceDebugUiState(){
    runtimeState.singleCableForceDebugMode =
            ui->mainSingleCableForceDebugSwitch &&
            ui->mainSingleCableForceDebugSwitch->isChecked();

    if(ui->mainSingleCableForceDebugSwitch){
        ui->mainSingleCableForceDebugSwitch->setText(
                    runtimeState.singleCableForceDebugMode ?
                        QStringLiteral("单绳力控调试：开") :
                        QStringLiteral("单绳力控调试：关"));
    }
    if(runtimeState.singleCableForceDebugMode){
        if(ui->mainAllUseFC->isChecked()){
            ui->mainAllUseFC->setChecked(false);
        }
        if(ui->mainHybridPoseForceModeSwitch && ui->mainHybridPoseForceModeSwitch->isChecked()){
            ui->mainHybridPoseForceModeSwitch->setChecked(false);
        }
        enforceSingleCableForceSelection();
    }
    syncSingleCableForceDebugPretensionForce();
    setForceControlSelectionEnabled(forceControlSelectionUiEnabled);
    refreshHybridPoseForceSelectionUi();
}

void MainWindow::startMonitor(){
    if(monitorThread || monitorQThread){
        return;
    }

    const QString alertPortName = ui->MonitorCOMnum->currentText().trimmed();
    if(alertPortName.isEmpty() || alertPortName == QStringLiteral("/")){
        displayInfo("监视串口未启用：MonitorCOMnum 为 /");
        return;
    }

    bool baudOk = false;
    const int alertBaudRate = ui->MonitorCOMbaudrate->currentText().toInt(&baudOk);
    if(!baudOk || alertBaudRate <= 0){
        displayInfo("监视波特率配置无效", "error");
        return;
    }

    displayInfo(QString("监视串口按配置连接：%1 @ %2")
                .arg(alertPortName)
                .arg(alertBaudRate).toStdString());

    runtimeState.controlBoxMotionControlState = -1;
    runtimeState.controlBoxSpeedZeroState = -1;
    runtimeState.controlBoxZeroCalibState = 0;

    monitorThread = new MonitorThread();
    monitorQThread = new QThread(this);
    monitorThread->setSerialConfig(alertPortName, alertBaudRate);
    monitorThread->moveToThread(monitorQThread);

    connect(monitorQThread, &QThread::started,
            monitorThread, &MonitorThread::startMonitoring);
    connect(monitorQThread, &QThread::finished,
            monitorThread, &QObject::deleteLater);
    connect(monitorThread, &MonitorThread::displayInfoSignal,
            this, [this](const QString& info, const QString& type){
                displayInfo(info.toStdString(), type.toStdString());
            }, Qt::QueuedConnection);
    connect(monitorThread, &MonitorThread::controlBoxStatusUpdated,
            this, &MainWindow::handleControlBoxStatus, Qt::QueuedConnection);

    monitorQThread->start();
}

void MainWindow::stopMonitor(){
    MonitorThread* worker = monitorThread;
    QThread* thread = monitorQThread;
    monitorThread = nullptr;
    monitorQThread = nullptr;

    if(worker && thread && thread->isRunning()){
        QMetaObject::invokeMethod(worker, "stopMonitoring", Qt::BlockingQueuedConnection);
        thread->quit();
        if(!thread->wait(1000)){
            thread->terminate();
            thread->wait(500);
        }
    }

    if(thread){
        delete thread;
    }
}

void MainWindow::startSafetyMonitor()
{
    if(safetyMonitor || safetyMonitorThread){
        updateSafetyMonitorConfig();
        return;
    }

    safetyMonitor = new SafetyMonitor();
    safetyMonitor->setControlWorker(controlWorker);
    safetyMonitor->setHardwareInterface(&hardwareInterface);
    safetyMonitorThread = new QThread(this);
    safetyMonitor->moveToThread(safetyMonitorThread);

    connect(safetyMonitorThread, &QThread::started,
            safetyMonitor, &SafetyMonitor::start);
    connect(safetyMonitorThread, &QThread::finished,
            safetyMonitor, &QObject::deleteLater);
    connect(safetyMonitor, &SafetyMonitor::faultDetected,
            this, &MainWindow::handleSafetyFault, Qt::QueuedConnection);

    updateSafetyMonitorConfig();
    safetyMonitorThread->start();
}

void MainWindow::stopSafetyMonitor()
{
    SafetyMonitor* worker = safetyMonitor;
    QThread* thread = safetyMonitorThread;
    safetyMonitor = nullptr;
    safetyMonitorThread = nullptr;

    if(worker && thread && thread->isRunning()){
        QMetaObject::invokeMethod(worker, "stop", Qt::BlockingQueuedConnection);
        thread->quit();
        if(!thread->wait(1000)){
            thread->terminate();
            thread->wait(500);
        }
    }

    if(thread){
        delete thread;
    }
}

void MainWindow::initializeUdpBridge()
{
    if(udpCommWorker || udpThread){
        return;
    }

    udpCommWorker = new UdpCommWorker();
    udpThread = new QThread(this);
    udpCommWorker->moveToThread(udpThread);

    connect(udpThread, &QThread::finished,
            udpCommWorker, &QObject::deleteLater);
    connect(udpCommWorker, &UdpCommWorker::statsUpdated,
            this, [this](const UdpCommStats& stats){
                udpCommStats = stats;
                refreshRunModeUiStateThrottled();
            }, Qt::QueuedConnection);
    connect(udpCommWorker, &UdpCommWorker::poseCommandReceived,
            this, &MainWindow::handleUdpPoseCommand, Qt::QueuedConnection);
    connect(udpCommWorker, &UdpCommWorker::trajectoryChunkReceived,
            this, &MainWindow::handleUdpTrajectoryChunk, Qt::QueuedConnection);
    connect(udpCommWorker, &UdpCommWorker::displayInfoSignal,
            this, [this](const QString& info, const QString& type){
                displayInfo(info.toStdString(), type.toStdString());
            }, Qt::QueuedConnection);

    udpThread->start();

    if(runtimeState.runMode == RunMode::RealtimeTrajectory){
        startUdpForRealtimeMode();
    }
}

void MainWindow::stopUdpBridge()
{
    UdpCommWorker* worker = udpCommWorker;
    QThread* thread = udpThread;
    udpCommWorker = nullptr;
    udpThread = nullptr;
    udpRealtimeBridgeActive = false;

    if(worker && thread && thread->isRunning()){
        QMetaObject::invokeMethod(worker, "stopListening", Qt::BlockingQueuedConnection);
        thread->quit();
        if(!thread->wait(1000)){
            thread->terminate();
            thread->wait(500);
        }
    }

    if(thread){
        delete thread;
    }
}

void MainWindow::startUdpForRealtimeMode()
{
    if(!udpCommWorker || !udpThread || !udpThread->isRunning()){
        return;
    }
    if(udpRealtimeBridgeActive){
        updateUdpStatusPayload();
        return;
    }

    udpRealtimeBridgeActive = true;
    updateUdpStatusPayload();
    const int listenPort = udpRealtimeListenPort();
    const QString targetIp = udpRealtimeTargetIp();
    const int targetPort = udpRealtimeTargetPort();

    QMetaObject::invokeMethod(udpCommWorker,
                              "setSendIntervalMs",
                              Qt::QueuedConnection,
                              Q_ARG(int, kUdpRealtimeSendIntervalMs));
    QMetaObject::invokeMethod(udpCommWorker,
                              "startListening",
                              Qt::QueuedConnection,
                              Q_ARG(int, listenPort),
                              Q_ARG(QString, targetIp),
                              Q_ARG(int, targetPort));
    QMetaObject::invokeMethod(udpCommWorker,
                              "setPeriodicSendEnabled",
                              Qt::QueuedConnection,
                              Q_ARG(bool, true));
}

void MainWindow::stopUdpForRealtimeMode()
{
    if(!udpCommWorker || !udpRealtimeBridgeActive){
        return;
    }

    udpRealtimeBridgeActive = false;
    QMetaObject::invokeMethod(udpCommWorker,
                              "setPeriodicSendEnabled",
                              Qt::QueuedConnection,
                              Q_ARG(bool, false));
    QMetaObject::invokeMethod(udpCommWorker, "stopListening", Qt::QueuedConnection);
    refreshRunModeUiStateThrottled();
}

void MainWindow::updateUdpStatusPayload()
{
    if(!udpCommWorker){
        return;
    }

    static quint64 udpStatusSeq = 0;
    const RobotStateSnapshot robotState = currentRobotState(false);
    const ControlWorker::Snapshot snapshot = controlWorker ?
                controlWorker->latestSnapshot() :
                ControlWorker::Snapshot{};

    auto toQVector = [](const std::vector<double>& values){
        QVector<double> converted;
        converted.reserve(static_cast<int>(values.size()));
        for(double value : values){
            converted.push_back(value);
        }
        return converted;
    };

    UdpStatusPayload payload;
    payload.seq = ++udpStatusSeq;
    payload.systemRunning = robotState.systemRunning;
    payload.pvtRunning = robotState.pvtMotionRunning;
    payload.pvtPaused = robotState.pvtMotionPaused;
    payload.motorPos = toQVector(snapshot.motorRelRawPos);
    payload.motorVel = toQVector(snapshot.motorVel);
    payload.force = toQVector(snapshot.forceSensorValue);
    payload.expectedForce = toQVector(snapshot.expectedForce.empty() ?
                                          buildExpectedForceFromUi() :
                                          snapshot.expectedForce);

    udpStatusPayload = payload;
    QMetaObject::invokeMethod(udpCommWorker,
                              "updateStatusPayload",
                              Qt::QueuedConnection,
                              Q_ARG(UdpStatusPayload, payload));
}

QString MainWindow::buildUdpStatusSummary() const
{
    if(!udpCommWorker){
        return QStringLiteral("UDP 旁路未初始化。");
    }

    if(!udpRealtimeBridgeActive){
        return QStringLiteral("UDP 旁路未启用。");
    }

    const QString receiveStatus = udpCommStats.receiveStatus.isEmpty() ?
                QStringLiteral("待启动") :
                udpCommStats.receiveStatus;
    const QString sendStatus = udpCommStats.sendStatus.isEmpty() ?
                QStringLiteral("待发送") :
                udpCommStats.sendStatus;
    const QString targetIp = udpRealtimeTargetIp();
    QString summary = QStringLiteral("UDP 旁路：监听 0.0.0.0:%1，回传 %2:%3，接收状态=%4，发送状态=%5，收=%6，发=%7，解析错=%8。")
            .arg(udpRealtimeListenPort())
            .arg(targetIp)
            .arg(udpRealtimeTargetPort())
            .arg(receiveStatus)
            .arg(sendStatus)
            .arg(udpCommStats.rxCount)
            .arg(udpCommStats.txCount)
            .arg(udpCommStats.parseErrorCount);

    if(udpCommStats.lastRxTimeMs > 0){
        const qint64 ageMs = std::max<qint64>(0, QDateTime::currentMSecsSinceEpoch() - udpCommStats.lastRxTimeMs);
        summary.append(QStringLiteral(" 最近接收距今 %1 ms。").arg(ageMs));
    }
    else{
        summary.append(QStringLiteral(" 尚未接收到外部 UDP 指令。"));
    }

    if(!udpCommStats.lastError.isEmpty()){
        summary.append(QStringLiteral(" 最近错误：%1。").arg(udpCommStats.lastError));
    }

    return summary;
}

void MainWindow::handleUdpPoseCommand(const UdpPoseCommand& command)
{
    if(command.seq == lastUdpPoseSeq &&
            command.timestampMs == lastUdpPoseTimestampMs){
        udpLastPacketSummaryText = QStringLiteral("pose_command seq=%1 timestamp=%2，重复包")
                .arg(command.seq)
                .arg(command.timestampMs);
        udpLastPacketActionText = QStringLiteral("已忽略重复包；如需重复测试，请修改 seq 或 timestamp_ms。");
        displayInfo(QStringLiteral("UDP pose_command 重复包已忽略：seq=%1").arg(command.seq).toStdString(),
                    "warning");
        refreshUdpPacketSummaryUi();
        return;
    }
    lastUdpPoseSeq = command.seq;
    lastUdpPoseTimestampMs = command.timestampMs;
    latestUdpPoseCommand = command;

    if(command.pose.size() < 6){
        udpLastPacketSummaryText = QStringLiteral("pose_command seq=%1，位姿维度不足").arg(command.seq);
        udpLastPacketActionText = QStringLiteral("包格式无效，未更新目标位姿。");
        displayInfo("错误：收到的 UDP pose_command 位姿维度不足，已忽略", "error");
        refreshUdpPacketSummaryUi();
        return;
    }
    udpLastPacketSummaryText = QStringLiteral(
                "pose_command seq=%1，duration=%2s，目标位姿=[%3, %4, %5, %6, %7, %8]")
            .arg(command.seq)
            .arg(command.duration, 0, 'f', 3)
            .arg(command.pose[0], 0, 'f', 3)
            .arg(command.pose[1], 0, 'f', 3)
            .arg(command.pose[2], 0, 'f', 3)
            .arg(command.pose[3], 0, 'f', 3)
            .arg(command.pose[4], 0, 'f', 3)
            .arg(command.pose[5], 0, 'f', 3);
    if(runtimeState.runMode != RunMode::RealtimeTrajectory){
        udpLastPacketActionText = QStringLiteral("当前不在实时运动轨迹控制模式，已忽略自动执行。");
        displayInfo(QStringLiteral("收到 UDP pose_command seq=%1，但当前不在实时运动轨迹控制模式，已忽略自动执行")
                    .arg(command.seq).toStdString(),
                    "warning");
        refreshUdpPacketSummaryUi();
        return;
    }

    if(ui->runModeRealtimeSourceCombo &&
            ui->runModeRealtimeSourceCombo->currentText() != kRealtimeInputManual){
        ui->runModeRealtimeSourceCombo->setCurrentText(kRealtimeInputManual);
    }

    std::vector<double> actualPose;
    if(currentEstimatedEndPose(actualPose) && actualPose.size() >= 6){
        ui->mainPosModeTargetStartPx->setValue(actualPose[0]);
        ui->mainPosModeTargetStartPy->setValue(actualPose[1]);
        ui->mainPosModeTargetStartPz->setValue(actualPose[2]);
        ui->mainPosModeTargetStartRx->setValue(actualPose[3]);
        ui->mainPosModeTargetStartRy->setValue(actualPose[4]);
        ui->mainPosModeTargetStartRz->setValue(actualPose[5]);
    }

    ui->mainPosModeTargetEndPx->setValue(command.pose[0]);
    ui->mainPosModeTargetEndPy->setValue(command.pose[1]);
    ui->mainPosModeTargetEndPz->setValue(command.pose[2]);
    ui->mainPosModeTargetEndRx->setValue(command.pose[3]);
    ui->mainPosModeTargetEndRy->setValue(command.pose[4]);
    ui->mainPosModeTargetEndRz->setValue(command.pose[5]);
    if(command.duration > 0.0){
        ui->mainPosModeTargetRunTime->setValue(command.duration);
    }

    const RobotStateSnapshot robotState = currentRobotState();
    if(robotState.anyMotionRunning){
        udpLastPacketActionText = QStringLiteral("当前任务运行中：已更新实时手动位姿目标，但本次不自动启动。");
        displayInfo(QStringLiteral("收到 UDP pose_command seq=%1，已更新实时手动位姿目标；当前任务运行中，本次不自动启动")
                    .arg(command.seq).toStdString(),
                    "warning");
        refreshRunModeUiState();
        refreshUdpPacketSummaryUi();
        return;
    }

    udpLastPacketActionText =
                QStringLiteral("已更新实时手动位姿目标，并启动实时位姿规划/执行链路。");
    displayInfo(QStringLiteral("收到 UDP pose_command seq=%1，按实时手动位姿链路启动规划与执行")
                .arg(command.seq).toStdString());
    startActiveRunMode(false);
    refreshUdpPacketSummaryUi();
}

void MainWindow::handleUdpTrajectoryChunk(const UdpTrajectoryChunk& chunk)
{
    const quint64 previousSeq = lastUdpTrajectorySeq;
    const qint64 previousTimestampMs = lastUdpTrajectoryTimestampMs;
    if(chunk.seq == lastUdpTrajectorySeq &&
            chunk.timestampMs == lastUdpTrajectoryTimestampMs){
        udpLastPacketSummaryText = QStringLiteral("trajectory_chunk seq=%1 timestamp=%2，重复包")
                .arg(chunk.seq)
                .arg(chunk.timestampMs);
        udpLastPacketActionText = QStringLiteral("已忽略重复分片；如需重复测试，请修改 seq 或 timestamp_ms。");
        displayInfo(QStringLiteral("UDP trajectory_chunk 重复包已忽略：seq=%1").arg(chunk.seq).toStdString(),
                    "warning");
        refreshUdpPacketSummaryUi();
        return;
    }
    lastUdpTrajectorySeq = chunk.seq;
    lastUdpTrajectoryTimestampMs = chunk.timestampMs;
    latestUdpTrajectoryChunk = chunk;
    udpLastPacketSummaryText = QStringLiteral("trajectory_chunk seq=%1，points=%2，final=%3，end_num=%4")
            .arg(chunk.seq)
            .arg(static_cast<int>(chunk.points.size()))
            .arg(chunk.final ? QStringLiteral("true") : QStringLiteral("false"))
            .arg(chunk.endNum);

    if(runtimeState.runMode != RunMode::RealtimeTrajectory){
        udpLastPacketActionText = QStringLiteral("当前不在实时运动轨迹控制模式，已忽略自动执行。");
        displayInfo(QStringLiteral("收到 UDP trajectory_chunk seq=%1，但当前不在实时运动轨迹控制模式，已忽略自动执行")
                    .arg(chunk.seq).toStdString(),
                    "warning");
        refreshUdpPacketSummaryUi();
        return;
    }

    const int expectedEndNum = ui ? ui->devEndNum->value() : 1;
    if(chunk.endNum > 0 && chunk.endNum != expectedEndNum){
        udpLastPacketActionText = QStringLiteral("末端数量与界面配置不一致，已清空缓存并拒绝接入。");
        displayInfo(QStringLiteral("错误：收到的 UDP 轨迹末端数量为 %1，与当前界面配置 %2 不一致")
                    .arg(chunk.endNum)
                    .arg(expectedEndNum)
                    .toStdString(),
                    "error");
        udpTrajectoryPointBuffer.clear();
        refreshUdpPacketSummaryUi();
        return;
    }

    if(!udpTrajectoryPointBuffer.isEmpty() &&
            (chunk.seq < previousSeq ||
             (chunk.seq == previousSeq && chunk.timestampMs < previousTimestampMs))){
        // A non-monotonic seq generally indicates a new stream; restart the buffer.
        udpTrajectoryPointBuffer.clear();
    }
    for(const QVector<double>& point : chunk.points){
        udpTrajectoryPointBuffer.push_back(point);
    }

    if(!chunk.final){
        udpLastPacketActionText = QStringLiteral("已缓存轨迹分片，当前累计 %1 个点，等待 final=true。")
                .arg(static_cast<int>(udpTrajectoryPointBuffer.size()));
        displayInfo(QStringLiteral("收到 UDP trajectory_chunk seq=%1，已缓存 %2 个轨迹点，等待 final=true")
                    .arg(chunk.seq)
                    .arg(static_cast<int>(udpTrajectoryPointBuffer.size()))
                    .toStdString());
        refreshUdpPacketSummaryUi();
        return;
    }

    TrajectoryPlanner::MultiEndTrajectory offlineTraj;
    double duration = 0.0;
    std::vector<double> startPose;
    QString errorMessage;
    if(!buildUdpOfflineTrajectory(udpTrajectoryPointBuffer,
                                  expectedEndNum,
                                  offlineTraj,
                                  duration,
                                  startPose,
                                  errorMessage)){
        udpTrajectoryPointBuffer.clear();
        udpLastPacketActionText = QStringLiteral("轨迹转换失败：%1").arg(errorMessage);
        displayInfo(QStringLiteral("错误：UDP 轨迹转换失败，%1").arg(errorMessage).toStdString(), "error");
        refreshUdpPacketSummaryUi();
        return;
    }

    udpTrajectoryPointBuffer.clear();
    trajFileOfflineTraj = offlineTraj;
    useTrajFile = true;
    trajFileTrajPx1 = trajFileOfflineTraj[0][0][0];
    trajFileTrajPy1 = trajFileOfflineTraj[0][0][1];
    trajFileTrajPz1 = trajFileOfflineTraj[0][0][2];
    trajFileTrajRx1 = trajFileOfflineTraj[0][0][3];
    trajFileTrajRy1 = trajFileOfflineTraj[0][0][4];
    trajFileTrajRz1 = trajFileOfflineTraj[0][0][5];

    ui->mainPosModeTargetRunTime->setValue(duration);
    if(!trajFileTrajPx1.empty()){
        ui->mainPosModeTargetStartPx->setValue(trajFileTrajPx1.front());
        ui->mainPosModeTargetStartPy->setValue(trajFileTrajPy1.front());
        ui->mainPosModeTargetStartPz->setValue(trajFileTrajPz1.front());
        ui->mainPosModeTargetStartRx->setValue(trajFileTrajRx1.front());
        ui->mainPosModeTargetStartRy->setValue(trajFileTrajRy1.front());
        ui->mainPosModeTargetStartRz->setValue(trajFileTrajRz1.front());
        ui->mainPosModeTargetEndPx->setValue(trajFileTrajPx1.back());
        ui->mainPosModeTargetEndPy->setValue(trajFileTrajPy1.back());
        ui->mainPosModeTargetEndPz->setValue(trajFileTrajPz1.back());
        ui->mainPosModeTargetEndRx->setValue(trajFileTrajRx1.back());
        ui->mainPosModeTargetEndRy->setValue(trajFileTrajRy1.back());
        ui->mainPosModeTargetEndRz->setValue(trajFileTrajRz1.back());
    }

    if(ui->runModeRealtimeSourceCombo &&
            ui->runModeRealtimeSourceCombo->currentText() != kRealtimeInputExternal){
        ui->runModeRealtimeSourceCombo->setCurrentText(kRealtimeInputExternal);
    }

    const RobotStateSnapshot robotState = currentRobotState();
    if(robotState.anyMotionRunning){
        udpLastPacketActionText = QStringLiteral("已转换并缓存外部轨迹，共 %1 个点；当前任务运行中，本次不自动启动。")
                .arg(static_cast<int>(trajFileTrajPx1.size()));
        displayInfo(QStringLiteral("收到 UDP 轨迹 final=true，已转换并缓存 %1 个轨迹点；当前任务运行中，本次不自动启动")
                    .arg(static_cast<int>(trajFileTrajPx1.size()))
                    .toStdString(),
                    "warning");
        refreshRunModeUiState();
        refreshUdpPacketSummaryUi();
        return;
    }

    udpLastPacketActionText =
                QStringLiteral("已转换为外部轨迹缓存，并启动外部轨迹规划/执行链路。");
    displayInfo(QStringLiteral("收到 UDP 轨迹 final=true，已按外部轨迹输入缓存 %1 个轨迹点，并启动规划与执行")
                .arg(static_cast<int>(trajFileTrajPx1.size()))
                .toStdString());
    startActiveRunMode(false);
    refreshUdpPacketSummaryUi();
}

void MainWindow::refreshSafetyMonitorHeartbeat(qint64 timestampMs)
{
    if(!safetyMonitor){
        return;
    }
    if(timestampMs <= 0){
        timestampMs = QDateTime::currentMSecsSinceEpoch();
    }
    safetyMonitor->updateMainThreadHeartbeat(timestampMs);
}

void MainWindow::updateSafetyMonitorConfig()
{
    if(!safetyMonitor){
        return;
    }

    const qint64 heartbeatMs = QDateTime::currentMSecsSinceEpoch();
    refreshSafetyMonitorHeartbeat(heartbeatMs);

    const ControlWorker::Config controlConfig = buildControlWorkerConfig();

    SafetyMonitor::Config config;
    config.axisCount = controlConfig.axisCount;
    config.sensorCount = controlConfig.sensorCount;
    config.cycleMs = std::max(1.0, controlConfig.ctrlCycleMs);
    config.monitorEnabled = ui->devUseLS->isChecked();
    config.systemRunning = runtimeState.systemRunning;
    config.hardwareConnected = ui->devUseLS->isChecked();
    config.forceThreadRunning = isForceControlThreadActuallyRunning();
    config.singleCableForceDebugMode = isSingleCableForceDebugModeActive();
    config.motionActive = runtimeState.pvtCommandActive ||
            config.forceThreadRunning ||
            runtimeState.gcRunning ||
            runtimeState.motorTorqueDebugActive ||
            runtimeState.singleMotorTrajectoryActive;
    config.snapshotTimeoutMs = std::max(300, static_cast<int>(std::round(config.cycleMs * 8.0)));
    config.persistentFaultCycles = std::max(3, static_cast<int>(std::round(250.0 / config.cycleMs)));
    config.poseTimeoutCycles = std::max(3, static_cast<int>(std::round(300.0 / config.cycleMs)));
    config.cableBreakControllabilityEnabled = true;
    config.poseEndIndex = 0;
    config.cableBreakControllableRankThreshold = 6;
    config.forceJacobianRankTolerance = 1e-5;
    config.pulleyRadius = buildPulleyRadius();
    const int heartbeatBaseMs = controlSnapshotTimer ? std::max(1, controlSnapshotTimer->interval()) : 50;
    config.softwareWatchdogEnabled = true;
    config.mainThreadHeartbeatMs = heartbeatMs;
    config.mainThreadHeartbeatTimeoutMs = std::max(1500, heartbeatBaseMs * 20);
    config.mainThreadHeartbeatGraceMs = std::max(250, heartbeatBaseMs * 5);
    config.mainThreadHeartbeatTimeoutFaultCycles = 3;
    config.watchdogLogFilePath = softwareFaultGuardLogFilePath();
    config.axes.resize(controlConfig.axes.size());

    // In the current UI there is no dedicated per-rope lower tension threshold yet.
    // Use half of the configured pretension as the software lower safety threshold.
    const double forceMinThreshold = std::max(0.0, controlConfig.initForce * 0.5);
    for(int axisIndex=0; axisIndex<static_cast<int>(controlConfig.axes.size()); ++axisIndex){
        const ControlWorker::AxisConfig& axis = controlConfig.axes[axisIndex];
        SafetyMonitor::AxisConfig monitorAxis;
        monitorAxis.monitored = axis.isMotorAxis;
        monitorAxis.modeledForForceJacobian = isModeledMotorAxis(axisIndex);
        monitorAxis.sensorIndex = axis.sensorIndex;
        monitorAxis.endIndex = axisEndVec[axisIndex] ? axisEndVec[axisIndex]->value() : -1;
        monitorAxis.forceMin = forceMinThreshold;
        monitorAxis.forceMax = axis.forceMax;
        monitorAxis.motorMin = axis.motorMin;
        monitorAxis.motorMax = axis.motorMax;
        monitorAxis.motorVelMax = axis.motorVelMax;
        if(axisIndex < static_cast<int>(axisCableStartPosXVec.size())){
            monitorAxis.anchorPoint = {
                axisCableStartPosXVec[axisIndex]->value(),
                axisCableStartPosYVec[axisIndex]->value(),
                axisCableStartPosZVec[axisIndex]->value()
            };
        }
        if(axisIndex < static_cast<int>(axisCableEndPosXVec.size())){
            monitorAxis.contactPointLocal = {
                axisCableEndPosXVec[axisIndex]->value(),
                axisCableEndPosYVec[axisIndex]->value(),
                axisCableEndPosZVec[axisIndex]->value()
            };
        }
        config.axes[axisIndex] = monitorAxis;
    }

    WorkspaceBounds workspaceBounds;
    if(buildSafetyWorkspaceBounds(workspaceBounds)){
        config.workspaceMonitorEnabled = true;
        config.workspaceXMin = workspaceBounds.xMin;
        config.workspaceXMax = workspaceBounds.xMax;
        config.workspaceYMin = workspaceBounds.yMin;
        config.workspaceYMax = workspaceBounds.yMax;
        config.workspaceZMin = workspaceBounds.zMin;
        config.workspaceZMax = workspaceBounds.zMax;
        config.workspaceWarningMargin = workspaceBounds.warningMargin;
        config.workspaceSevereOverflow = workspaceBounds.severeOverflow;
    }

    std::vector<double> actualPose;
    config.hasActualPose = currentEstimatedEndPose(actualPose);
    if(config.hasActualPose){
        config.actualPose = actualPose;
    }
    else{
        config.actualPose.clear();
    }

    safetyMonitor->setConfig(config);
}

void MainWindow::clearSafetyFaultLatch(bool announce)
{
    const bool hadLatchedFault = runtimeState.safetyFaultLatched;
    const int previousStopLevel = runtimeState.safetyStopLevel;
    const int previousFaultCode = runtimeState.safetyFaultCode;
    const QString previousSummary = runtimeState.safetyFaultSummary;
    const QString previousDetail = runtimeState.safetyFaultDetail;
    runtimeState.safetyFaultLatched = false;
    runtimeState.safetyStopLevel = 0;
    runtimeState.safetyFaultCode = 0;
    runtimeState.safetyFaultSummary.clear();
    runtimeState.safetyFaultDetail.clear();

    if(safetyMonitor){
        QMetaObject::invokeMethod(safetyMonitor, "clearFaultLatch", Qt::QueuedConnection);
    }

    if(hadLatchedFault){
        appendStructuredFaultLog(
                    buildStructuredFaultRecord(QStringLiteral("fault_latch_cleared"),
                                               previousStopLevel,
                                               previousFaultCode,
                                               previousSummary.isEmpty() ?
                                                   QStringLiteral("安全锁存已清除") :
                                                   previousSummary,
                                               previousDetail,
                                               false,
                                               true,
                                               announce ?
                                                   QStringLiteral("安全锁存由界面复位操作清除") :
                                                   QStringLiteral("安全锁存由系统流程清除")));
    }

    if(announce && hadLatchedFault){
        displayInfo("安全锁存已清除");
    }

    refreshRunModeUiState();
}

bool MainWindow::ensureSafetyReadyForMotion(const QString& actionName)
{
    if(runtimeState.servoHoldActive){
        displayInfo(QStringLiteral("错误：%1失败，当前处于抱闸保持状态，请先解除抱闸并重新使能所有轴。")
                    .arg(actionName).toStdString(),
                    "error");
        return false;
    }

    if(!runtimeState.safetyFaultLatched){
        return true;
    }

    const QString summary = runtimeState.safetyFaultSummary.isEmpty() ?
                QStringLiteral("存在未复位的安全故障锁存") :
                runtimeState.safetyFaultSummary;
    displayInfo(QStringLiteral("错误：%1失败，%2。请先执行安全复位。")
                    .arg(actionName, summary).toStdString(),
                "error");
    return false;
}

bool MainWindow::applySafetyStopAction(int level, const QString& summary, const QString& detail)
{
    Q_UNUSED(summary);
    Q_UNUSED(detail);

    if(trajectoryCheckTimer){
        trajectoryCheckTimer->stop();
    }
    runtimeState.autoExecutePoseAfterSimulation = false;
    runtimeState.servoHoldActive = false;
    if(runtimeState.motorTorqueDebugActive){
        stopMotorTorqueDebug();
    }
    if(runtimeState.singleMotorTrajectoryActive){
        stopSingleMotorTrajectory(QStringLiteral("main_stop_button"));
    }

    stopGravityThread();
    optionalButtonSetEnabled(this, "mainGCModeSwitch", true);
    optionalButtonSetText(this, "mainGCModeSwitch", QStringLiteral("启动零力拖动"));

    ui->mainFCThread->setChecked(false);
    for(int i=0;i<useFCBtnVec.size();++i){
        useFCBtnVec[i]->setChecked(false);
    }
    clearHybridPoseForceModeState(false);

    if(simulationWorker){
        simulationWorker->stopPositionSimulation();
    }
    positionSimulationModel = nullptr;
    runtimeState.posModeRunning = false;
    runtimeState.pvtCommandActive = false;
    runtimeState.singleMotorTrajectoryActive = false;
    runtimeState.gcRunning = false;
    runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    lastTrajEndMotorTheta.clear();
    activePosModeMotorIndex.clear();
    activePosModeStartMotorTheta.clear();
    activePosModeContinuousStartMotorTheta.clear();
    runtimeState.controlBoxMotionControlState = -1;
    runtimeState.controlBoxSpeedZeroState = 0;
    runtimeState.controlBoxPausedPvtMotion = false;
    runtimeState.controlBoxPausedForceControl = false;
    runtimeState.controlBoxPausedGC = false;
    runtimeState.controlBoxStartRequiresStop = true;
    runtimeState.controlBoxStartInterlockReported = false;
    hardwareInterface.clearPvtTrajectoryState();
    runtimeState.pvtCommandActive = false;

    ui->mainPosModeSwitch->setEnabled(true);
    setPoseSimulationIdleText(ui);
    ui->mainPosModeSend->setEnabled(true);
    ui->mainReplay3DViewer->setEnabled(true);
    setForceControlSelectionEnabled(true);
    updateCableHomeConfirmEnabled();
    ui->mainStopSwitch->setText("停止");

    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        return false;
    }

    if(level >= static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop)){
        return hardwareInterface.emergencyStopAll();
    }

    bool stopOk = true;
    for(int i=0;i<ui->devAxisNum->value();++i){
        if(isMotorAxis(i)){
            stopOk = hardwareInterface.motorStop(i) && stopOk;
        }
    }
    return stopOk;
}

void MainWindow::handleSafetyFault(int level, int code, const QString& summary, const QString& detail)
{
    const SafetyMonitor::StopLevel stopLevel = static_cast<SafetyMonitor::StopLevel>(level);
    const QString messageType = stopLevel == SafetyMonitor::StopLevel::EmergencyStop ?
                QStringLiteral("error") :
                QStringLiteral("warning");

    if(stopLevel == SafetyMonitor::StopLevel::Warning){
        displayInfo(QStringLiteral("安全预警：%1").arg(summary).toStdString(),
                    messageType.toStdString());
        if(!detail.isEmpty()){
            displayInfo(detail.toStdString(), messageType.toStdString());
        }
        appendStructuredFaultLog(
                    buildStructuredFaultRecord(QStringLiteral("fault_warning"),
                                               level,
                                               code,
                                               summary,
                                               detail,
                                               false,
                                               true));
        return;
    }

    if(runtimeState.safetyFaultLatched &&
            level <= runtimeState.safetyStopLevel &&
            runtimeState.safetyFaultSummary == summary){
        return;
    }

    runtimeState.safetyFaultLatched = true;
    runtimeState.safetyStopLevel = level;
    runtimeState.safetyFaultCode = code;
    runtimeState.safetyFaultSummary = summary;
    runtimeState.safetyFaultDetail = detail;

    QString actionPrefix;
    if(stopLevel == SafetyMonitor::StopLevel::ControlledStop){
        actionPrefix = QStringLiteral("安全受控停机");
    }
    else if(stopLevel == SafetyMonitor::StopLevel::SafetyStop){
        actionPrefix = QStringLiteral("安全停机");
    }
    else{
        actionPrefix = QStringLiteral("安全急停");
    }

    displayInfo(QStringLiteral("%1：%2").arg(actionPrefix, summary).toStdString(),
                messageType.toStdString());
    if(!detail.isEmpty()){
        displayInfo(detail.toStdString(), messageType.toStdString());
    }

    int executedLevel = level;
    QString structuredNote;
    QString executedActionPrefix = actionPrefix;
    bool stopOk = false;
    if(stopLevel == SafetyMonitor::StopLevel::ControlledStop &&
            code == static_cast<int>(SafetyMonitor::FaultCode::CableBreak)){
        QString failureReason;
        stopOk = engageSpeedZeroControlledStop(&failureReason);
        if(stopOk){
            structuredNote = QStringLiteral("断绳可控性评估通过，已执行控制盒速度置零链路缓停，并在 SafetyMonitor 中持续复评力雅可比秩。");
        }
        else{
            executedLevel = static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop);
            executedActionPrefix = QStringLiteral("安全急停");
            runtimeState.safetyStopLevel = executedLevel;
            displayInfo(QStringLiteral("断绳缓停链路执行失败：%1，已回退为安全急停")
                        .arg(failureReason.isEmpty() ?
                                 QStringLiteral("当前没有可暂停的运动源") :
                                 failureReason)
                        .toStdString(),
                        "warning");
            stopOk = applySafetyStopAction(executedLevel, summary, detail);
            structuredNote = failureReason.isEmpty() ?
                        QStringLiteral("断绳可控性评估通过，但缓停链路执行失败，已回退为安全急停。") :
                        QStringLiteral("断绳可控性评估通过，但缓停链路执行失败：%1，已回退为安全急停。")
                        .arg(failureReason);
        }
    }
    else{
        stopOk = applySafetyStopAction(level, summary, detail);
    }
    appendStructuredFaultLog(
                buildStructuredFaultRecord(QStringLiteral("fault_triggered"),
                                           executedLevel,
                                           code,
                                           summary,
                                           detail,
                                           true,
                                           stopOk,
                                           stopOk ?
                                               structuredNote :
                                               (structuredNote.isEmpty() ?
                                                    QStringLiteral("软件侧停机清理已执行，但硬件停机动作返回失败或当前未连接驱动") :
                                                    structuredNote)));
    if(!stopOk){
        displayInfo(QStringLiteral("%1已执行软件侧停机清理，但硬件停机动作返回失败，请检查驱动连接")
                        .arg(executedActionPrefix).toStdString(),
                    "warning");
    }
    refreshRunModeUiState();
    updateSafetyMonitorConfig();
}

void MainWindow::resetSafetyStateFromUi()
{
    const RobotStateSnapshot robotState = currentRobotState();
    if(!runtimeState.safetyFaultLatched){
        displayInfo("当前没有需要复位的安全锁存", "warning");
        appendStructuredFaultLog(
                    buildStructuredFaultRecord(QStringLiteral("fault_reset_rejected"),
                                               static_cast<int>(SafetyMonitor::StopLevel::Warning),
                                               static_cast<int>(SafetyMonitor::FaultCode::None),
                                               QStringLiteral("安全复位请求被拒绝"),
                                               QStringLiteral("当前没有需要复位的安全锁存"),
                                               false,
                                               false));
        refreshRunModeUiState();
        return;
    }

    if(robotState.anyMotionRunning){
        displayInfo("安全复位前请先停止当前运动任务，确认设备已停稳", "warning");
        appendStructuredFaultLog(
                    buildStructuredFaultRecord(QStringLiteral("fault_reset_rejected"),
                                               runtimeState.safetyStopLevel,
                                               runtimeState.safetyFaultCode,
                                               runtimeState.safetyFaultSummary.isEmpty() ?
                                                   QStringLiteral("安全复位请求被拒绝") :
                                                   runtimeState.safetyFaultSummary,
                                               QStringLiteral("安全复位前请先停止当前运动任务，确认设备已停稳"),
                                               false,
                                               false));
        refreshRunModeUiState();
        return;
    }

    const bool emergencyStopLatched =
            runtimeState.safetyStopLevel >= static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop);
    if(robotState.lsConnected && !hardwareInterface.clearAllLeadshineAxisErrorCodes()){
        displayInfo("安全复位失败：电机轴错误码清除未成功，请检查控制卡与驱动状态", "error");
        appendStructuredFaultLog(
                    buildStructuredFaultRecord(QStringLiteral("fault_reset_rejected"),
                                               runtimeState.safetyStopLevel,
                                               runtimeState.safetyFaultCode,
                                               runtimeState.safetyFaultSummary.isEmpty() ?
                                                   QStringLiteral("安全复位请求被拒绝") :
                                                   runtimeState.safetyFaultSummary,
                                               QStringLiteral("安全复位清除电机轴错误码失败"),
                                               false,
                                               false));
        refreshRunModeUiState();
        return;
    }

    if(emergencyStopLatched){
        runtimeState.systemRunning = false;
        if(ui->mainSwitch){
            ui->mainSwitch->setText(QStringLiteral("启动"));
        }
        updateControlWorkerConfig();
    }

    clearSafetyFaultLatch(true);
}

void MainWindow::triggerSoftwareEmergencyStop()
{
    const RobotStateSnapshot robotState = currentRobotState();
    const QString summary = QStringLiteral("界面触发软件急停");
    const QString detail = QStringLiteral(
                "用户通过主界面“软件急停”按钮中止当前链路，系统已执行软件侧停机，并向执行机构请求急停。");

    if(safetyMonitor){
        QMetaObject::invokeMethod(safetyMonitor,
                                  "requestEmergencyStop",
                                  Qt::QueuedConnection,
                                  Q_ARG(QString, summary));
    }

    if(!robotState.systemRunning &&
            !robotState.lsConnected &&
            !robotState.anyMotionRunning){
        const bool stopOk = applySafetyStopAction(static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop),
                                                  summary,
                                                  detail);
        appendStructuredFaultLog(
                    buildStructuredFaultRecord(QStringLiteral("manual_emergency_request"),
                                               static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop),
                                               static_cast<int>(SafetyMonitor::FaultCode::None),
                                               summary,
                                               detail,
                                               true,
                                               stopOk,
                                               QStringLiteral("当前未处于运行态，仅执行软件侧急停清理")));
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：急停按钮已按下，当前未连接硬件，仅完成软件侧急停清理"),
                                     QStringLiteral("error"));
        refreshRunModeUiState();
        return;
    }

    handleSafetyFault(static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop),
                      static_cast<int>(SafetyMonitor::FaultCode::None),
                      summary,
                      detail);
}

void MainWindow::triggerInjectedCableBreak()
{
    if(!ui || !ui->faultInjectionCableCombo){
        return;
    }

    if(runtimeState.safetyFaultLatched){
        displayInfo("当前存在安全锁存故障，请先复位后再执行新的故障注入", "warning");
        refreshFaultInjectionUiControls();
        return;
    }

    const QVariant selectedChannel = ui->faultInjectionCableCombo->currentData();
    if(!selectedChannel.isValid()){
        displayInfo("错误：当前没有可用于断绳模拟的张力通道", "error");
        refreshFaultInjectionUiControls();
        return;
    }

    const int sensorIndex = selectedChannel.toInt();
    const QString summary = QStringLiteral("故障注入：模拟绳索%1断裂").arg(sensorIndex + 1);
    const QString detail = QStringLiteral(
                "用户通过故障注入入口触发断绳模拟。系统将先进行断绳后力雅可比秩评估，可控则缓停，不可控则急停。");

    displayInfo(QStringLiteral("已触发断绳模拟：绳索/张力通道 %1").arg(sensorIndex + 1).toStdString(),
                "warning");
    if(safetyMonitor){
        QMetaObject::invokeMethod(safetyMonitor,
                                  "requestInjectedCableBreak",
                                  Qt::QueuedConnection,
                                  Q_ARG(int, sensorIndex));
    }
    else{
        handleSafetyFault(static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop),
                          static_cast<int>(SafetyMonitor::FaultCode::CableBreak),
                          summary,
                          detail + QStringLiteral(" 当前安全监控线程未运行，已直接执行安全急停。"));
    }
}

void MainWindow::triggerInjectedMotorFault()
{
    if(!ui || !ui->faultInjectionMotorCombo){
        return;
    }

    if(runtimeState.safetyFaultLatched){
        displayInfo("当前存在安全锁存故障，请先复位后再执行新的故障注入", "warning");
        refreshFaultInjectionUiControls();
        return;
    }

    const QVariant selectedAxis = ui->faultInjectionMotorCombo->currentData();
    if(!selectedAxis.isValid()){
        displayInfo("错误：当前没有可用于电机故障模拟的电机轴", "error");
        refreshFaultInjectionUiControls();
        return;
    }

    const int axisIndex = selectedAxis.toInt();
    const QString summary = QStringLiteral("故障注入：模拟电机轴%1故障").arg(axisIndex);
    const QString detail = QStringLiteral(
                "用户通过故障注入入口触发电机故障模拟。系统应立即执行统一安全急停链路，并锁存该故障用于 6.5-3 验证。");

    displayInfo(QStringLiteral("已触发电机故障模拟：电机轴 %1").arg(axisIndex).toStdString(),
                "warning");
    handleSafetyFault(static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop),
                      static_cast<int>(SafetyMonitor::FaultCode::HardwareDisconnected),
                      summary,
                      detail);
}

void MainWindow::triggerInjectedPlcCommunicationFault()
{
    if(runtimeState.safetyFaultLatched){
        displayInfo("当前存在安全锁存故障，请先复位后再执行新的故障注入", "warning");
        refreshFaultInjectionUiControls();
        return;
    }

    const QString summary = QStringLiteral("故障注入：模拟工控机与PLC断开通信");
    const QString detail = QStringLiteral(
                "用户通过故障注入入口触发工控机与PLC通信断开模拟。系统应立即执行统一安全急停链路，并锁存该故障用于通信故障联调与验收验证。");

    displayInfo("已触发通信断开模拟：工控机与PLC通信断开", "warning");
    handleSafetyFault(static_cast<int>(SafetyMonitor::StopLevel::EmergencyStop),
                      static_cast<int>(SafetyMonitor::FaultCode::HardwareDisconnected),
                      summary,
                      detail);
}

void MainWindow::handleControlBoxStatus(int motionControl, int speedZero, int zeroCalib){
    if(zeroCalib == 0){
        handleControlBoxMotionControl(motionControl);
    }
    handleControlBoxSpeedZero(speedZero);
    handleControlBoxZeroCalib(zeroCalib);
}

void MainWindow::handleControlBoxMotionControl(int state){
    if(runtimeState.controlBoxStartRequiresStop){
        if(state == 1){
            if(!runtimeState.controlBoxStartInterlockReported){
                displayInfo("未将启动停止开关放置到停止状态。", "error");
                runtimeState.controlBoxStartInterlockReported = true;
            }
            runtimeState.controlBoxMotionControlState = state;
            return;
        }

        if(state == 0){
            runtimeState.controlBoxStartRequiresStop = false;
            const bool hadReportedInterlock = runtimeState.controlBoxStartInterlockReported;
            runtimeState.controlBoxStartInterlockReported = false;
            runtimeState.controlBoxMotionControlState = state;
            hasReportErr = false;
            if(hadReportedInterlock){
                displayInfo("控制盒启动互锁已解除，请重新拨到启动位执行轨迹");
            }
            return;
        }
    }

    if(state == runtimeState.controlBoxMotionControlState){
        return;
    }
    runtimeState.controlBoxMotionControlState = state;

    if(state == 1){
        const RobotStateSnapshot robotState = currentRobotState();
        if(!robotState.systemRunning){
            displayInfo("控制盒：启动失败，请先打开总开关 mainSwitch", "error");
        }
        else if(!ensureCableHomeReadyForMotion(QStringLiteral("控制盒运动控制"))){
            return;
        }
        else if(!robotState.posModeRunning &&
                !robotState.pvtMotionRunning &&
                !robotState.pvtMotionPaused){
            displayInfo("控制盒：运动控制切到启动，按当前运行模式开始任务");
            startActiveRunMode(true);
        }
        else{
            displayInfo("控制盒：当前已有位控任务正在仿真、执行或暂停，忽略重复启动", "warning");
        }
        return;
    }

    if(state == 0){
        displayInfo("控制盒：运动控制切到停止，中止轨迹并回到本段起点", "warning");
        runtimeState.controlBoxSpeedZeroState = 0;
        runtimeState.controlBoxPausedPvtMotion = false;
        runtimeState.controlBoxPausedForceControl = false;
        runtimeState.controlBoxPausedGC = false;
        stopMotionFromControlBox();
    }
}

bool MainWindow::engageSpeedZeroControlledStop(QString* failureReason)
{
    if(failureReason){
        failureReason->clear();
    }

    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        if(failureReason){
            *failureReason = QStringLiteral("雷赛控制卡未连接");
        }
        return false;
    }

    const RobotStateSnapshot robotState = currentRobotState();
    const bool pvtRunning = robotState.pvtMotionRunning;
    const bool forceControlRunning = robotState.forceControlThreadRunning;
    const bool gcRunning = robotState.gcRunning;
    if(!robotState.anyMotionRunning){
        if(failureReason){
            *failureReason = QStringLiteral("当前没有实际运行的位置轨迹、力控线程或零力拖动模式");
        }
        return false;
    }

    bool pausedSomething = false;
    if(pvtRunning){
        bool paused = false;
        if(pvtExecutionWorker){
            invokeOnObjectThreadWithHeartbeat(pvtExecutionWorker, [&](){
                paused = pvtExecutionWorker->pausePvtMotion();
            }, [this](){
                refreshSafetyMonitorHeartbeat();
            });
        }
        runtimeState.controlBoxPausedPvtMotion = paused;
        pausedSomething = runtimeState.controlBoxPausedPvtMotion || pausedSomething;
    }

    if(forceControlRunning){
        ui->mainFCThread->setChecked(false);
        requestImmediateForceControlUpdate();
        runtimeState.controlBoxPausedForceControl = true;
        pausedSomething = true;
    }

    if(gcRunning){
        runtimeState.controlBoxPausedGC = true;
        runGCMode();
        pausedSomething = true;
    }

    const bool continuousControlPaused = runtimeState.controlBoxPausedForceControl ||
            runtimeState.controlBoxPausedGC;
    if(continuousControlPaused && !runtimeState.controlBoxPausedPvtMotion){
        stopAllMotorAxesForSpeedZero();
    }

    if(!pausedSomething && failureReason){
        *failureReason = QStringLiteral("缓停链路未成功暂停任何运动源");
    }
    return pausedSomething;
}

void MainWindow::handleControlBoxSpeedZero(int state){
    const int previousState = runtimeState.controlBoxSpeedZeroState;
    if(state == previousState){
        return;
    }
    runtimeState.controlBoxSpeedZeroState = state;
    if(previousState < 0 && state == 0){
        return;
    }

    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        if(state == 1){
            displayInfo("控制盒：速度置零无效，雷赛控制卡未连接", "error");
        }
        return;
    }

    const RobotStateSnapshot robotState = currentRobotState();
    const bool pvtPaused = robotState.pvtMotionPaused;

    if(state == 1){
        QString failureReason;
        if(engageSpeedZeroControlledStop(&failureReason)){
            displayInfo("控制盒：速度置零已执行，运动源已暂停且电机速度已置零", "warning");
        }
        else{
            displayInfo(QStringLiteral("控制盒：速度置零失败，%1")
                        .arg(failureReason.isEmpty() ?
                                 QStringLiteral("当前没有可暂停的运动源") :
                                 failureReason)
                        .toStdString(),
                        "warning");
        }
        return;
    }

    if(state == 0){
        bool resumedSomething = false;

        if(runtimeState.controlBoxPausedPvtMotion || pvtPaused){
            bool resumed = false;
            if(pvtExecutionWorker){
                invokeOnObjectThreadWithHeartbeat(pvtExecutionWorker, [&](){
                    resumed = pvtExecutionWorker->resumePvtMotion();
                }, [this](){
                    refreshSafetyMonitorHeartbeat();
                });
            }
            resumedSomething = resumed || resumedSomething;
            runtimeState.controlBoxPausedPvtMotion = false;
        }

        if(runtimeState.controlBoxPausedForceControl){
            ui->mainFCThread->setChecked(true);
            requestImmediateForceControlUpdate();
            runtimeState.controlBoxPausedForceControl = false;
            resumedSomething = true;
        }

        if(runtimeState.controlBoxPausedGC){
            if(!runtimeState.gcRunning){
                resumedSomething = runGCMode() || resumedSomething;
            }
            runtimeState.controlBoxPausedGC = false;
        }

        if(resumedSomething){
            displayInfo("控制盒：速度置零已释放，已恢复由控制盒暂停的运动源");
        }
        else{
            displayInfo("控制盒：速度置零复位失败，当前没有由控制盒暂停的运动源", "warning");
        }
    }
}

bool MainWindow::isForceControlThreadActuallyRunning() const{
    if(!runtimeState.systemRunning || !ccThread || !ccThread->isRunning() || !ui->mainFCThread->isChecked()){
        return false;
    }
    if(controlWorker && controlWorker->latestSnapshot().forceThreadRunning){
        return true;
    }

    if(runtimeState.hybridPoseForceModeActive){
        return true;
    }

    for(QRadioButton* btn : useFCBtnVec){
        if(btn && btn->isChecked()){
            return true;
        }
    }
    return false;
}

void MainWindow::stopAllMotorAxesForSpeedZero(){
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        return;
    }

    for(int i=0;i<ui->devAxisNum->value();++i){
        if(isMotorAxis(i)){
            hardwareInterface.motorStop(i);
        }
    }
}

void MainWindow::handleControlBoxZeroCalib(int state){
    if(state == runtimeState.controlBoxZeroCalibState){
        return;
    }
    runtimeState.controlBoxZeroCalibState = state;

    if(state == 0){
        if(calibrationWorkflowStage != CalibrationWorkflowStage::Completed &&
                calibrationWorkflowStage != CalibrationWorkflowStage::AwaitingPretensionStart &&
                calibrationWorkflowStage != CalibrationWorkflowStage::AwaitingConfirm){
            calibrationWorkflowStage = CalibrationWorkflowStage::Idle;
            refreshCalibrationUiState();
        }
        return;
    }

    if(state == 1){
        if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
            displayInfo("控制盒：零位校准1失败，雷赛控制卡未连接", "error");
            return;
        }
        calibrationWorkflowStage = CalibrationWorkflowStage::MechanicalHoming;
        refreshCalibrationUiState();
        runtimeState.cableHomeState = CableHomeState::Unconfirmed;
        updateCableHomeConfirmEnabled();
        displayInfo("控制盒：零位校准1，执行两段直线机械回零；回零后请人工确认末端停在零点位姿上方20mm", "warning");
        if(motor2Home()){
            setMotorControllerEnable(false);
            runtimeState.cableHomeState = CableHomeState::Unconfirmed;
            updateCableHomeConfirmEnabled();
            calibrationWorkflowStage = CalibrationWorkflowStage::AwaitingPretensionStart;
            displayInfo("控制盒：零位校准1回零完成，末端停在零点位姿上方20mm且电机已自动失能；请确认位置后进入预紧");
        }
        else{
            calibrationWorkflowStage = CalibrationWorkflowStage::Stopped;
            displayInfo("控制盒：零位校准1回零未确认完成，未自动失能", "warning");
        }
        refreshCalibrationUiState();
        return;
    }

    if(state == 2){
        calibrationWorkflowStage = CalibrationWorkflowStage::PretensionBalancing;
        refreshCalibrationUiState();
        if(!beginCalibrationPretensionStage(QStringLiteral("控制盒：零位校准2"))){
            calibrationWorkflowStage = CalibrationWorkflowStage::Stopped;
            refreshCalibrationUiState();
            return;
        }
        calibrationWorkflowStage = CalibrationWorkflowStage::AwaitingConfirm;
        refreshCalibrationUiState();
        displayInfo("控制盒：零位校准2，张紧稳定后点击“确认并同步”完成零位校准");
    }
}

void MainWindow::checkTrajectoryFinished(){
    if(!trajectoryCheckTimer){
        return;
    }

    bool hasPvtTrajectory = false;
    bool pvtMotionPaused = false;
    bool pvtMotionFinished = true;
    invokeOnObjectThreadWithHeartbeat(&hardwareInterface, [&](){
        hasPvtTrajectory = hardwareInterface.hasPvtTrajectory();
        if(hasPvtTrajectory){
            pvtMotionPaused = hardwareInterface.isPvtMotionPausedState();
            pvtMotionFinished = hardwareInterface.isPvtMotionFinished();
        }
    }, [this](){
        refreshSafetyMonitorHeartbeat();
    });

    if(!hasPvtTrajectory){
        trajectoryCheckTimer->stop();
        runtimeState.pvtCommandActive = false;
        return;
    }

    if(pvtMotionPaused){
        return;
    }

    if(!pvtMotionFinished){
        return;
    }

    trajectoryCheckTimer->stop();
    invokeOnObjectThreadWithHeartbeat(&hardwareInterface, [&](){
        hardwareInterface.clearPvtTrajectoryState();
    }, [this](){
        refreshSafetyMonitorHeartbeat();
    });

    runtimeState.posModeRunning = false;
    runtimeState.pvtCommandActive = false;
    const bool hybridWasActive = runtimeState.hybridPoseForceModeActive;
    const bool keepCableHomeForContinuousTrajectory = ui->Poscontinuetraj->isChecked();
    logTrajectoryEndCableLengthDiagnostics();
    bool refreshedForwardKinematicsForContinuousTrajectory = false;
    bool syncedStartPoseForContinuousTrajectory = false;
    if(keepCableHomeForContinuousTrajectory){
        runtimeState.cableHomeState = CableHomeState::Confirmed;
        refreshedForwardKinematicsForContinuousTrajectory =
                refreshForwardKinematicsPoseFromCurrentMotorPosition();
        if(lastPlannedPoseTrajectoryEndPose.size() >= 6){
            applyTrajectoryStartPoseToUi(lastPlannedPoseTrajectoryEndPose);
            syncedStartPoseForContinuousTrajectory = true;
        }
        else{
            std::vector<double> currentPose;
            if(currentEstimatedEndPose(currentPose, -1) && currentPose.size() >= 6){
                applyTrajectoryStartPoseToUi(currentPose);
                syncedStartPoseForContinuousTrajectory = true;
            }
        }
    }
    else{
        runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    }
    activePosModeMotorIndex.clear();
    activePosModeStartMotorTheta.clear();
    activePosModeContinuousStartMotorTheta.clear();
    runtimeState.controlBoxMotionControlState = -1;
    runtimeState.controlBoxStartRequiresStop = true;
    runtimeState.controlBoxStartInterlockReported = false;
    runtimeState.controlBoxPausedPvtMotion = false;

    ui->mainStopSwitch->setText("停止");
    ui->mainPosModeSwitch->setEnabled(true);
    setPoseSimulationIdleText(ui);
    ui->mainPosModeSend->setEnabled(true);
    ui->mainReplay3DViewer->setEnabled(true);
    updateCableHomeConfirmEnabled();
    setForceControlSelectionEnabled(true);

    if(keepCableHomeForContinuousTrajectory){
        if(hybridWasActive){
            clearHybridPoseForceModeState(true);
            displayInfo("位置轨迹执行完成，已结束2绳恒张力保持；连续轨迹已保持预紧零点和正运动学位姿刷新，已将上一段终点填入新的起点");
        }
        else{
            displayInfo("位置轨迹执行完成；连续轨迹已保持预紧零点和正运动学位姿刷新，已将上一段终点填入新的起点");
        }
        if(!refreshedForwardKinematicsForContinuousTrajectory){
            displayInfo("警告：连续轨迹已保持预紧零点，但本次完成时未能立即刷新正运动学位姿；请检查电机位置反馈和绳长基准",
                        "warning");
        }
        if(!syncedStartPoseForContinuousTrajectory){
            displayInfo("警告：连续轨迹未能自动同步下一段起点位姿，请手动确认起点参数",
                        "warning");
        }
    }
    else if(hybridWasActive){
        clearHybridPoseForceModeState(true);
        displayInfo("位置轨迹执行完成，已结束2绳恒张力保持；预紧确认已失效；下次运行前请重新预紧并确认");
    }
    else{
        displayInfo("位置轨迹执行完成，预紧确认已失效；下次运行前请重新预紧并确认");
    }
}

void MainWindow::resetControlUiState(){
    if(trajectoryCheckTimer){
        trajectoryCheckTimer->stop();
    }
    runtimeState.autoExecutePoseAfterSimulation = false;
    runtimeState.posModeRunning = false;
    runtimeState.pvtCommandActive = false;
    runtimeState.gcRunning = false;
    lastTrajEndMotorTheta.clear();
    activePosModeMotorIndex.clear();
    activePosModeStartMotorTheta.clear();
    activePosModeContinuousStartMotorTheta.clear();
    runtimeState.controlBoxPausedPvtMotion = false;
    runtimeState.controlBoxPausedForceControl = false;
    runtimeState.controlBoxPausedGC = false;
    runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    clearPlannedPoseForceTrajectory();
    clearHybridPoseForceModeState(false);

    ui->mainStopSwitch->setText("停止");
    ui->mainPosModeSwitch->setEnabled(true);
    setPoseSimulationIdleText(ui);
    ui->mainPosModeSend->setEnabled(true);
    ui->mainReplay3DViewer->setEnabled(true);
    optionalButtonSetEnabled(this, "mainGCModeSwitch", true);
    optionalButtonSetText(this, "mainGCModeSwitch", QStringLiteral("启动零力拖动"));
    ui->mainResetCableForceHomeSwitch->setEnabled(true);
    updateCableHomeConfirmEnabled();
    ui->devUseCamForActPose->setEnabled(true);
    setForceControlSelectionEnabled(true);
}

std::vector<std::vector<std::vector<double>>> MainWindow::buildCableContactPointPos() const{
    std::vector<std::vector<std::vector<double>>> cableContactPointPos(ui->devEndNum->value());
    for(int i=0;i<ui->devAxisNum->value();++i){
        if(isModeledMotorAxis(i)){
            cableContactPointPos[axisEndVec[i]->value()].push_back(
                        {axisCableEndPosXVec[i]->value(),axisCableEndPosYVec[i]->value(),axisCableEndPosZVec[i]->value()});
        }
    }
    return cableContactPointPos;
}

std::vector<std::vector<double>> MainWindow::buildFixedAnchorHome() const{
    std::vector<std::vector<double>> anchorCableCoorHome;
    for(int endIndex=0;endIndex<ui->devEndNum->value();++endIndex){
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isModeledMotorAxis(i) && axisEndVec[i]->value() == endIndex){
                anchorCableCoorHome.push_back(
                            {axisCableStartPosXVec[i]->value(),axisCableStartPosYVec[i]->value(),axisCableStartPosZVec[i]->value()});
            }
        }
    }
    return anchorCableCoorHome;
}

std::vector<double> MainWindow::buildCableMotorCof() const{
    std::vector<double> cableMotorCof;
    for(int endIndex=0;endIndex<ui->devEndNum->value();++endIndex){
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isModeledMotorAxis(i) && axisEndVec[i]->value() == endIndex){
                cableMotorCof.push_back(motorCableAngleScale(i));
            }
        }
    }
    return cableMotorCof;
}

double MainWindow::buildPulleyRadius() const{
    return ui->devPulleyRadius->value();
}

std::vector<double> MainWindow::buildFixedAnchorDis() const{
    return std::vector<double>(buildFixedAnchorHome().size(),0.0);
}

std::vector<std::vector<std::vector<double>>> MainWindow::splitAnchorPositionsByEnd(const std::vector<std::vector<double>>& anchorPosTemp) const{
    std::vector<std::vector<std::vector<double>>> curAnchorPos(ui->devEndNum->value());
    int tmp = 0;
    for(int i=0;i<ui->devAxisNum->value();++i){
        if(isModeledMotorAxis(i)){
            if(tmp < anchorPosTemp.size()){
                curAnchorPos[axisEndVec[i]->value()].push_back(anchorPosTemp[tmp]);
            }
            tmp++;
        }
    }
    return curAnchorPos;
}

void MainWindow::clearPlannedPoseForceTrajectory(){
    plannedPoseCableForceTraj.clear();
    plannedPoseExpectedForceTraj.clear();
    plannedPoseForceTimeStamp.clear();
}

std::vector<std::vector<double>> MainWindow::mapCableForceTrajectoryToSensorExpected(
    const std::vector<std::vector<double>>& cableForceTraj) const
{
    const int pointCount = cableForceTraj.empty() ? 0 : static_cast<int>(cableForceTraj.front().size());
    const std::vector<double> defaultExpected = buildExpectedForceFromUi();

    std::vector<std::vector<double>> expectedTraj(axisForceSensorNum, std::vector<double>(pointCount, 0.0));
    for(int sensorIndex=0; sensorIndex<axisForceSensorNum; ++sensorIndex){
        const double defaultValue = sensorIndex < static_cast<int>(defaultExpected.size()) ? defaultExpected[sensorIndex] : 0.0;
        std::fill(expectedTraj[sensorIndex].begin(), expectedTraj[sensorIndex].end(), defaultValue);
    }

    int cableIndex = 0;
    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
            if(!isModeledMotorAxis(axisIndex) || axisEndVec[axisIndex]->value() != endIndex){
                continue;
            }

            const int sensorIndex = axisSensorIndexVec[axisIndex]->value();
            if(sensorIndex >= 0 &&
               sensorIndex < axisForceSensorNum &&
               cableIndex < static_cast<int>(cableForceTraj.size())){
                const std::vector<double>& cableForce = cableForceTraj[cableIndex];
                if(static_cast<int>(cableForce.size()) == pointCount){
                    expectedTraj[sensorIndex] = cableForce;
                }
            }
            ++cableIndex;
        }
    }

    return expectedTraj;
}

bool MainWindow::isHybridPoseForceModeRequested() const
{
    return runtimeState.runMode != RunMode::SimulationCalculation &&
            ui->mainHybridPoseForceModeSwitch &&
            ui->mainHybridPoseForceModeSwitch->isChecked();
}

void MainWindow::updateHybridPoseForceModeButtonText()
{
    if(!ui->mainHybridPoseForceModeSwitch){
        return;
    }
    ui->mainHybridPoseForceModeSwitch->setText(
                ui->mainHybridPoseForceModeSwitch->isChecked() ?
                    QStringLiteral("力位混合控制：开") :
                    QStringLiteral("力位混合控制：关"));
}

void MainWindow::refreshHybridPoseForceSelectionUi()
{
    if(!ui || !ui->mainHybridPoseForceCableA || !ui->mainHybridPoseForceCableB){
        return;
    }

    int modeledCableCount = 0;
    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
            if(isModeledMotorAxis(axisIndex) && axisEndVec[axisIndex]->value() == endIndex){
                ++modeledCableCount;
            }
        }
    }

    const int maxCableCount = std::max(1, modeledCableCount);
    if(ui->mainHybridPoseForceCableA->maximum() != maxCableCount){
        ui->mainHybridPoseForceCableA->setMaximum(maxCableCount);
    }
    if(ui->mainHybridPoseForceCableB->maximum() != maxCableCount){
        ui->mainHybridPoseForceCableB->setMaximum(maxCableCount);
    }
    if(ui->mainHybridPoseForceCableA->value() > maxCableCount){
        ui->mainHybridPoseForceCableA->setValue(maxCableCount);
    }
    if(ui->mainHybridPoseForceCableB->value() > maxCableCount){
        ui->mainHybridPoseForceCableB->setValue(maxCableCount);
    }
}

bool MainWindow::buildHybridPoseForceModeConfig(HybridPoseForceModeConfig& out,
                                                QString& errorMessage) const
{
    out = HybridPoseForceModeConfig{};
    if(!isHybridPoseForceModeRequested()){
        return true;
    }

    std::vector<int> modeledAxisIndex;
    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
            if(!isModeledMotorAxis(axisIndex) || axisEndVec[axisIndex]->value() != endIndex){
                continue;
            }
            modeledAxisIndex.push_back(axisIndex);
        }
    }

    if(static_cast<int>(modeledAxisIndex.size()) != 8){
        errorMessage = QStringLiteral("力位混合执行失败：当前模式要求8根建模绳索，实际为%1根")
                .arg(modeledAxisIndex.size());
        return false;
    }

    const int cableOrderA = ui->mainHybridPoseForceCableA ? ui->mainHybridPoseForceCableA->value() - 1 : 6;
    const int cableOrderB = ui->mainHybridPoseForceCableB ? ui->mainHybridPoseForceCableB->value() - 1 : 7;
    if(cableOrderA == cableOrderB){
        errorMessage = QStringLiteral("力位混合执行失败：两根力控绳索不能选择同一根");
        return false;
    }

    const std::vector<int> selectedCableOrder = {cableOrderA, cableOrderB};
    std::vector<bool> sensorSelected(axisForceSensorNum, false);
    std::vector<int> selectedForceAxisIndex;
    std::vector<int> selectedForceSensorIndex;
    for(int cableOrder : selectedCableOrder){
        if(cableOrder < 0 || cableOrder >= static_cast<int>(modeledAxisIndex.size())){
            errorMessage = QStringLiteral("力位混合执行失败：所选力控绳索序号超出当前建模范围");
            return false;
        }

        const int axisIndex = modeledAxisIndex[cableOrder];
        const int sensorIndex = axisSensorIndexVec[axisIndex]->value();
        if(sensorIndex < 0 || sensorIndex >= axisForceSensorNum){
            errorMessage = QStringLiteral("力位混合执行失败：所选力控绳索未正确映射到力传感器");
            return false;
        }
        if(sensorSelected[sensorIndex]){
            errorMessage = QStringLiteral("力位混合执行失败：所选力控绳索映射到了重复的力传感器通道");
            return false;
        }

        sensorSelected[sensorIndex] = true;
        selectedForceAxisIndex.push_back(axisIndex);
        selectedForceSensorIndex.push_back(sensorIndex);
    }

    if(static_cast<int>(selectedForceAxisIndex.size()) != 2){
        errorMessage = QStringLiteral("力位混合执行失败：所选力控绳索数量不是2根");
        return false;
    }

    out.enabled = true;
    out.forceAxisIndex = selectedForceAxisIndex;
    out.forceSensorIndex = selectedForceSensorIndex;
    out.frozenExpectedForce = buildExpectedForceFromUi();
    if(static_cast<int>(out.frozenExpectedForce.size()) < axisForceSensorNum){
        out.frozenExpectedForce.resize(axisForceSensorNum, 0.0);
    }
    return true;
}

bool MainWindow::applyHybridPoseForceExpectedForceAtPoint(int pointIndex, QString* errorMessage)
{
    if(errorMessage){
        errorMessage->clear();
    }
    if(!controlWorker){
        if(errorMessage){
            *errorMessage = QStringLiteral("力位混合执行失败：力控线程未初始化");
        }
        return false;
    }
    if(!activeHybridPoseForceConfig.enabled){
        if(errorMessage){
            *errorMessage = QStringLiteral("力位混合执行失败：当前未激活力位混合控制配置");
        }
        return false;
    }
    if(activeHybridExpectedForceTraj.empty()){
        if(errorMessage){
            *errorMessage = QStringLiteral("力位混合执行失败：当前没有可用的轨迹点期望绳力");
        }
        return false;
    }

    const int sensorCount = std::max(axisForceSensorNum, static_cast<int>(activeHybridPoseForceConfig.frozenExpectedForce.size()));
    std::vector<double> expectedForce = activeHybridPoseForceConfig.frozenExpectedForce;
    if(static_cast<int>(expectedForce.size()) < sensorCount){
        expectedForce.resize(sensorCount, 0.0);
    }

    for(int sensorIndex : activeHybridPoseForceConfig.forceSensorIndex){
        if(sensorIndex < 0 || sensorIndex >= static_cast<int>(activeHybridExpectedForceTraj.size())){
            if(errorMessage){
                *errorMessage = QStringLiteral("力位混合执行失败：力控绳索未找到对应的期望绳力轨迹");
            }
            return false;
        }
        const std::vector<double>& sensorTraj = activeHybridExpectedForceTraj[sensorIndex];
        if(pointIndex < 0 || pointIndex >= static_cast<int>(sensorTraj.size())){
            if(errorMessage){
                *errorMessage = QStringLiteral("力位混合执行失败：轨迹点%1超出期望绳力范围").arg(pointIndex + 1);
            }
            return false;
        }
        expectedForce[sensorIndex] = sensorTraj[pointIndex];
    }

    mainForceSensorExpVal = expectedForce;
    for(int sensorIndex=0; sensorIndex < static_cast<int>(expectedForce.size()) &&
                            sensorIndex < static_cast<int>(mainForceSensorExp.size());
        ++sensorIndex){
        if(mainForceSensorExp[sensorIndex]){
            mainForceSensorExp[sensorIndex]->setValue(expectedForce[sensorIndex]);
        }
    }
    controlWorker->setExternalExpectedForce(expectedForce);
    activeHybridExpectedForcePointIndex = pointIndex;
    return true;
}

void MainWindow::syncHybridPoseForceExpectedForceForActiveTrajectory()
{
    if(!runtimeState.hybridPoseForceModeActive || activeHybridExpectedForceTraj.empty()){
        return;
    }
    if(!hardwareInterface.hasPvtTrajectory()){
        return;
    }

    double currentTrajectoryTime = 0.0;
    std::size_t currentIndex = 0;
    if(!hardwareInterface.currentPvtProgress(currentTrajectoryTime, currentIndex)){
        return;
    }

    const int pointCount = static_cast<int>(activeHybridExpectedForceTraj.front().size());
    if(pointCount <= 0){
        return;
    }

    const int clampedIndex = std::min(std::max(static_cast<int>(currentIndex), 0), pointCount - 1);
    if(clampedIndex == activeHybridExpectedForcePointIndex){
        return;
    }

    QString errorMessage;
    if(!applyHybridPoseForceExpectedForceAtPoint(clampedIndex, &errorMessage) && !errorMessage.isEmpty()){
        displayInfo(errorMessage.toStdString(), "error");
    }
}

void MainWindow::clearHybridPoseForceModeState(bool disableForceThread)
{
    runtimeState.hybridPoseForceModeActive = false;
    activeHybridPoseForceConfig = HybridPoseForceModeConfig{};
    activeHybridExpectedForceTraj.clear();
    activeHybridExpectedForcePointIndex = -1;
    if(controlWorker){
        controlWorker->clearExternalExpectedForce();
    }
    if(disableForceThread){
        ui->mainFCThread->setChecked(false);
    }
    requestImmediateForceControlUpdate();
}

bool MainWindow::planPoseForceTrajectory(
    const std::vector<std::vector<std::vector<std::vector<double>>>>& plannedEndTraj,
    QString& errorMessage,
    std::vector<double>* failedPose,
    int* failedStep,
    double* failedTime)
{
    clearPlannedPoseForceTrajectory();
    if(failedPose){
        failedPose->clear();
    }
    if(failedStep){
        *failedStep = -1;
    }
    if(failedTime){
        *failedTime = -1.0;
    }

    if(plannedEndTraj.size() != 1){
        errorMessage = QStringLiteral("当前轨迹力分配校验仅支持单末端轨迹");
        return false;
    }
    if(plannedEndTraj[0].empty() ||
       plannedEndTraj[0][0].size() < 6 ||
       plannedEndTraj[0][0][0].empty()){
        errorMessage = QStringLiteral("轨迹力分配校验失败：轨迹数据格式不完整");
        return false;
    }

    std::vector<std::vector<std::vector<double>>> forceTraj = plannedEndTraj[0];
    if(forceTraj.size() < 4){
        forceTraj.resize(4);
    }
    for(int layer=1; layer<4; ++layer){
        if(forceTraj[layer].size() < 6){
            forceTraj[layer].resize(6);
        }
    }

    const std::vector<std::vector<double>> rawPoseTraj = forceTraj[0];
    const int pointCount = static_cast<int>(forceTraj[0][0].size());
    for(int dim=0; dim<6; ++dim){
        if(static_cast<int>(forceTraj[0][dim].size()) != pointCount){
            errorMessage = QStringLiteral("轨迹力分配校验失败：轨迹点数量不一致");
            return false;
        }
    }

    auto buildPoseDescription = [&rawPoseTraj, pointCount](int pointIndex) -> QString {
        if(pointIndex < 0 || pointIndex >= pointCount || rawPoseTraj.size() < 6){
            return QString();
        }
        return QStringLiteral("，对应位姿(mm/rad)：px=%1, py=%2, pz=%3, rx=%4, ry=%5, rz=%6")
                .arg(rawPoseTraj[0][pointIndex], 0, 'f', 3)
                .arg(rawPoseTraj[1][pointIndex], 0, 'f', 3)
                .arg(rawPoseTraj[2][pointIndex], 0, 'f', 3)
                .arg(rawPoseTraj[3][pointIndex], 0, 'f', 6)
                .arg(rawPoseTraj[4][pointIndex], 0, 'f', 6)
                .arg(rawPoseTraj[5][pointIndex], 0, 'f', 6);
    };

    auto estimateDerivative = [](const std::vector<double>& data,
                                 const std::vector<double>& timeStamp) -> std::vector<double> {
        const int count = static_cast<int>(data.size());
        std::vector<double> derivative(count, 0.0);
        if(count <= 1 || static_cast<int>(timeStamp.size()) != count){
            return derivative;
        }

        for(int i=0; i<count; ++i){
            if(i == 0){
                const double dt = timeStamp[1] - timeStamp[0];
                derivative[i] = std::abs(dt) > 1e-9 ? (data[1] - data[0]) / dt : 0.0;
            }
            else if(i == count - 1){
                const double dt = timeStamp[count - 1] - timeStamp[count - 2];
                derivative[i] = std::abs(dt) > 1e-9 ? (data[count - 1] - data[count - 2]) / dt : 0.0;
            }
            else{
                const double dt = timeStamp[i + 1] - timeStamp[i - 1];
                derivative[i] = std::abs(dt) > 1e-9 ? (data[i + 1] - data[i - 1]) / dt : 0.0;
            }
        }
        return derivative;
    };

    const double fallbackStepTime = ui->mainPosModeTargetStepTime->value();
    if(fallbackStepTime <= 0.0){
        errorMessage = QStringLiteral("轨迹力分配校验失败：轨迹步长必须大于0");
        return false;
    }

    bool hasTimeAxis = !forceTraj[3].empty() && static_cast<int>(forceTraj[3][0].size()) == pointCount;
    if(!hasTimeAxis){
        for(int dim=0; dim<6; ++dim){
            forceTraj[3][dim].resize(pointCount, 0.0);
        }
        for(int pointIndex=0; pointIndex<pointCount; ++pointIndex){
            const double t = pointIndex * fallbackStepTime;
            for(int dim=0; dim<6; ++dim){
                forceTraj[3][dim][pointIndex] = t;
            }
        }
    }
    else{
        for(int dim=1; dim<6; ++dim){
            if(static_cast<int>(forceTraj[3][dim].size()) != pointCount){
                forceTraj[3][dim] = forceTraj[3][0];
            }
        }
    }

    const std::vector<double> timeStamp = forceTraj[3][0];
    for(int dim=0; dim<6; ++dim){
        if(static_cast<int>(forceTraj[1][dim].size()) != pointCount){
            forceTraj[1][dim] = estimateDerivative(forceTraj[0][dim], timeStamp);
        }
        if(static_cast<int>(forceTraj[2][dim].size()) != pointCount){
            forceTraj[2][dim] = estimateDerivative(forceTraj[1][dim], timeStamp);
        }
    }

    const std::vector<std::vector<std::vector<double>>> cableContactPointPos = buildCableContactPointPos();
    const std::vector<std::vector<double>> anchorCableCoorHome = buildFixedAnchorHome();
    if(cableContactPointPos.empty()){
        errorMessage = QStringLiteral("轨迹力分配校验失败：未配置绳索接点");
        return false;
    }
    if(cableContactPointPos[0].size() != anchorCableCoorHome.size()){
        errorMessage = QStringLiteral("轨迹力分配校验失败：锚点数量与接点数量不一致");
        return false;
    }

    const int cableCount = static_cast<int>(anchorCableCoorHome.size());
    if(cableCount != kBarycenterCableCount){
        errorMessage = QStringLiteral("轨迹力分配校验当前仅支持%1根绳索，实际配置为%2根")
                .arg(kBarycenterCableCount).arg(cableCount);
        return false;
    }

    Eigen::Matrix<double, 3, 8> basePoints;
    Eigen::Matrix<double, 3, 8> attachPoints;
    for(int cableIndex=0; cableIndex<cableCount; ++cableIndex){
        if(anchorCableCoorHome[cableIndex].size() < 3 || cableContactPointPos[0][cableIndex].size() < 3){
            errorMessage = QStringLiteral("轨迹力分配校验失败：锚点或接点坐标维度不足");
            return false;
        }

        basePoints.col(cableIndex) << anchorCableCoorHome[cableIndex][0] / 1000.0,
                                       anchorCableCoorHome[cableIndex][1] / 1000.0,
                                       anchorCableCoorHome[cableIndex][2] / 1000.0;
        attachPoints.col(cableIndex) << cableContactPointPos[0][cableIndex][0] / 1000.0,
                                         cableContactPointPos[0][cableIndex][1] / 1000.0,
                                         cableContactPointPos[0][cableIndex][2] / 1000.0;
    }

    if(endMassVec.empty() || endIxxVec.empty() || endIyyVec.empty() || endIzzVec.empty() ||
       endIxyVec.empty() || endIyzVec.empty() || endIxzVec.empty()){
        errorMessage = QStringLiteral("轨迹力分配校验失败：动力学参数未初始化");
        return false;
    }

    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    inertia(0,0) = endIxxVec[0]->value();
    inertia(1,1) = endIyyVec[0]->value();
    inertia(2,2) = endIzzVec[0]->value();
    inertia(0,1) = inertia(1,0) = endIxyVec[0]->value();
    inertia(1,2) = inertia(2,1) = endIyzVec[0]->value();
    inertia(0,2) = inertia(2,0) = endIxzVec[0]->value();

    const double barycenterForceMin = kBarycenterForceMinN;
    const double barycenterForceMax = kBarycenterForceMaxN;
    if(barycenterForceMax <= barycenterForceMin){
        errorMessage = QStringLiteral("轨迹力分配校验失败：barycenter最大拉力上限%1不大于最小拉力下限%2")
                .arg(barycenterForceMax, 0, 'f', 3)
                .arg(barycenterForceMin, 0, 'f', 3);
        return false;
    }

    Eigen::VectorXd forceMin = Eigen::VectorXd::Constant(cableCount, barycenterForceMin);
    Eigen::VectorXd forceMax = Eigen::VectorXd::Constant(cableCount, barycenterForceMax);
    int cableIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex) || axisEndVec[axisIndex]->value() != 0){
            continue;
        }
        ++cableIndex;
    }

    if(cableIndex != cableCount){
        errorMessage = QStringLiteral("轨迹力分配校验失败：建模电机轴数量与绳索数量不一致");
        return false;
    }

    for(int layer=0; layer<3; ++layer){
        for(int dim=0; dim<3; ++dim){
            if(static_cast<int>(forceTraj[layer][dim].size()) != pointCount){
                errorMessage = QStringLiteral("轨迹力分配校验失败：轨迹点数量不一致");
                return false;
            }
            for(double& value : forceTraj[layer][dim]){
                value /= 1000.0;
            }
        }
    }

    BarycenterEigen barycenter;
    barycenter.setParams(endMassVec[0]->value(), inertia, basePoints, attachPoints, forceMin, forceMax,
                         buildPulleyRadius() / 1000.0);
    const BarycenterEigen::SolveResult solveResult = barycenter.solveTrajectory(forceTraj);
    if(!solveResult.is_valid){
        if(solveResult.failed_step >= 0){
            if(failedStep){
                *failedStep = solveResult.failed_step;
            }
            if(failedTime){
                *failedTime = solveResult.failed_time;
            }
            if(failedPose && rawPoseTraj.size() >= 6){
                failedPose->resize(6, 0.0);
                for(int dim=0; dim<6; ++dim){
                    if(solveResult.failed_step < static_cast<int>(rawPoseTraj[dim].size())){
                        (*failedPose)[dim] = rawPoseTraj[dim][solveResult.failed_step];
                    }
                }
            }
            const QString poseDescription = buildPoseDescription(solveResult.failed_step);
            if(solveResult.failed_time >= 0.0){
                errorMessage = QStringLiteral("轨迹力分配不可行：第%1个轨迹点（t=%2 s）无可行绳索力解%3")
                        .arg(solveResult.failed_step + 1)
                        .arg(solveResult.failed_time, 0, 'f', 3)
                        .arg(poseDescription);
            }
            else{
                errorMessage = QStringLiteral("轨迹力分配不可行：第%1个轨迹点无可行绳索力解%2")
                        .arg(solveResult.failed_step + 1)
                        .arg(poseDescription);
            }
        }
        else{
            errorMessage = QStringLiteral("轨迹力分配不可行：未找到满足上下限的绳索力解");
        }
        return false;
    }

    plannedPoseCableForceTraj = solveResult.cable_force;
    plannedPoseExpectedForceTraj = mapCableForceTrajectoryToSensorExpected(solveResult.cable_force);
    plannedPoseForceTimeStamp = forceTraj[3][0];
    return true;
}

void MainWindow::runSwitch(){
    if(ui->devUseCamForActPose->isChecked() && !motiveFitConfirmed){
        displayInfo("错误：启用动捕位姿时，请先执行 mainResetMotiveFit", "error");
        return;
    }

    refreshAxisRuntimeCounts();
    if(!ui->devUseLS->isChecked()){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：启动按钮已按下，但当前未启用雷赛控制卡，无法执行硬件启动"),
                                     QStringLiteral("warning"));
        return;
    }

    if(hardwareInterface.isLSConnected()){
        if(!runtimeState.systemRunning){
            clearSafetyFaultLatch(false);
            if(!setMotorControllerEnable(true)){
                showPrimaryOperationFeedback(QStringLiteral("主控反馈：启动失败，电机使能失败"),
                                             QStringLiteral("error"));
                return;
            }
            runtimeState.autoExecutePoseAfterSimulation = false;
            runtimeState.servoHoldActive = false;
            runtimeState.cableHomeState = CableHomeState::Unconfirmed;
            updateCableHomeConfirmEnabled();
            waitMotorPositionStable();
            syncMotorHomeAndDisplayZero("启动电机零点和显示已同步");
            updateControlWorkerConfig();
            if(ccThread && !ccThread->isRunning()){
                ccThread->start(QThread::HighPriority);
            }
            if(!isViewShow){
                main3DViewer->show();
                isViewShow = true;
            }
            resetRuntimeDiagnosticsState(true);
            runtimeState.systemRunning = true;
            ui->mainSwitch->setText("断连");
            ui->devUseCamForActPose->setEnabled(false);
            runtimeState.controlBoxStartRequiresStop = true;
            runtimeState.controlBoxStartInterlockReported = false;
            runtimeState.controlBoxMotionControlState = -1;
            updateSafetyMonitorConfig();
            return;
        }

        ui->mainFCThread->setChecked(false);
        for(int i=0;i<useFCBtnVec.size();++i){
            useFCBtnVec[i]->setChecked(false);
        }
        if(simulationWorker){
            simulationWorker->stopPositionSimulation();
        }
        positionSimulationModel = nullptr;
        stopGravityThread();
        stopControlThread();
        runtimeState.systemRunning = false;
        updateSafetyMonitorConfig();
        setMotorControllerEnable(false);
        hardwareInterface.disconnectLS();
        runtimeState.autoExecutePoseAfterSimulation = false;
        runtimeState.servoHoldActive = false;
        if(runtimeState.motorTorqueDebugActive){
            stopMotorTorqueDebug();
        }
        resetRuntimeDiagnosticsState(true);
        clearSafetyFaultLatch(false);
        resetControlUiState();
        ui->mainSwitch->setText("启动");
        updateSafetyMonitorConfig();
    }
    else{
        clearSafetyFaultLatch(false);
        if(!applyLeadshineHardwareConfigFromUi()){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：启动失败，雷赛轴参数未正确配置"),
                                         QStringLiteral("error"));
            return;
        }
        hardwareInterface.useStaticMotorHome = ui->axisIsStaticMotorHome->isChecked();
        if(hardwareInterface.useStaticMotorHome){
            std::vector<double> tmp(ui->devAxisNum->value());
            for(int i=0;i<ui->devAxisNum->value();++i){
                tmp[i] = axisMotorZeroVec[i]->value();
            }
            hardwareInterface.setMotorHome(tmp);
            motorPosDisplayZero.clear();
        }
        hardwareInterface.useStaticSensorHome = ui->axisIsStaticSensorHome->isChecked();
        std::vector<double> sensorHome(axisForceSensorNum, 0.0);
        if(hardwareInterface.useStaticSensorHome){// 不自动读取传感器零点数字量，而使用手动输入值（前提是传感器没零飘）
            for(int i=0;i<ui->devAxisNum->value();++i){
                const int sensorIndex = axisSensorIndexVec[i]->value();
                if(sensorIndex >= 0 && sensorIndex < static_cast<int>(sensorHome.size())){
                    sensorHome[sensorIndex] = axisSensorZeroVec[i]->value();
                }
            }
        }
        forceSensorCurHome = sensorHome;
        hardwareInterface.setForceSensorHome(sensorHome);
        if(!hardwareInterface.connectLS()){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：启动失败，未连接雷赛控制卡或驱动未上电"),
                                         QStringLiteral("error"));
            return;
        }
        if(motorTorqueServoSetupStatusLabel){
            motorTorqueServoSetupStatusLabel->setText(QStringLiteral("已随连接写入"));
        }
        if(!setMotorControllerEnable(true)){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：启动失败，电机使能失败"),
                                         QStringLiteral("error"));
            hardwareInterface.disconnectLS();
            return;
        }
        runtimeState.autoExecutePoseAfterSimulation = false;
        runtimeState.servoHoldActive = false;
        runtimeState.cableHomeState = CableHomeState::Unconfirmed;
        updateCableHomeConfirmEnabled();
        waitMotorPositionStable();
        syncMotorHomeAndDisplayZero("启动电机零点和显示已同步");
        updateControlWorkerConfig();
        if(ccThread && !ccThread->isRunning()){
            ccThread->start(QThread::HighPriority);
        }

        if(!isViewShow){// 打开仿真窗口
            main3DViewer->show();
            isViewShow = true;
        }

        resetRuntimeDiagnosticsState(true);
        runtimeState.systemRunning = true;
        ui->mainSwitch->setText("断连");
        ui->devUseCamForActPose->setEnabled(false);
        runtimeState.controlBoxStartRequiresStop = true;
        runtimeState.controlBoxStartInterlockReported = false;
        runtimeState.controlBoxMotionControlState = -1;
        updateSafetyMonitorConfig();
    }
}

void MainWindow::resetFCPIDPara(){// qDebug() << "TTTTTTTTTTTTTTTT";
    dynPID_P = std::vector<double>(axisForceSensorNum,ui->devFCKp->value());
    dynPID_I = std::vector<double>(axisForceSensorNum,ui->devFCKi->value());
    dynPID_D = std::vector<double>(axisForceSensorNum,ui->devFCKd->value());
    orgPID_P = dynPID_P;
    orgPID_I = dynPID_I;
    orgPID_D = dynPID_D;
}

void MainWindow::requestImmediateForceControlUpdate(){
    updateControlWorkerConfig();
}

bool MainWindow::setMotorControllerEnable(bool enable){
    if(!ui->devUseLS->isChecked()){
        return true;
    }
    bool ok = false;
    invokeOnObjectThreadWithHeartbeat(&hardwareInterface, [&](){
        ok = hardwareInterface.isLSConnected() &&
                hardwareInterface.setAllMotorEnable(enable);
    }, [this](){
        refreshSafetyMonitorHeartbeat();
    });
    if(ok){
        refreshSingleMotorEnableStateUi();
    }
    return ok;
}

bool MainWindow::toggleServoHoldFromUi()
{
    const RobotStateSnapshot robotState = currentRobotState();
    if(!ui->devUseLS->isChecked() || !robotState.lsConnected || !robotState.systemRunning){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：软件抱闸无效，当前未连接并上电雷赛控制卡"),
                                     QStringLiteral("warning"));
        return false;
    }

    if(runtimeState.safetyFaultLatched){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：当前存在安全锁存，软件抱闸不可执行。请先完成安全复位；紧急情况请使用急停"),
                                     QStringLiteral("error"));
        return false;
    }

    if(runtimeState.servoHoldActive){
        if(!setMotorControllerEnable(true)){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：解除软件抱闸失败，所有轴重新使能失败"),
                                         QStringLiteral("error"));
            return false;
        }
        runtimeState.servoHoldActive = false;
        waitMotorPositionStable();
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：已解除软件抱闸保持，所有轴重新使能。请确认末端安全后再启动运动"),
                                     QStringLiteral("info"));
        refreshPrimaryOperationUiState(currentRobotState());
        return true;
    }

    if(robotState.anyMotionRunning){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：软件抱闸请求已收到。当前正在运动，将先执行停止并等待停稳，再失能所有轴；紧急情况请使用急停"),
                                     QStringLiteral("warning"));
        if(!motorStop()){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：软件抱闸失败，停止链路执行失败，已拒绝直接失能所有轴"),
                                         QStringLiteral("error"));
            return false;
        }
    }

    if(!waitMotorAxesDone(5000, 20)){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：软件抱闸失败，电机未在限定时间内停稳。为避免运动中失能造成损坏，已拒绝执行"),
                                     QStringLiteral("error"));
        return false;
    }

    if(!setMotorControllerEnable(false)){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：软件抱闸失败，停稳后所有轴失能未成功"),
                                     QStringLiteral("error"));
        return false;
    }

    runtimeState.servoHoldActive = true;
    showPrimaryOperationFeedback(QStringLiteral("主控反馈：已停止并失能所有轴，进入软件抱闸保持状态（不是真实机械抱闸）"),
                                 QStringLiteral("warning"));
    refreshPrimaryOperationUiState(currentRobotState());
    return true;
}

int MainWindow::selectedSingleMotorIndex() const{
    const int axisIndex = ui->SinglemotorNum->value();
    if(axisIndex < 0 || axisIndex >= ui->devAxisNum->value()){
        return -1;
    }
    return axisIndex;
}

void MainWindow::disableForceControlForManualMotorTest(){
    const bool forceThreadWasChecked = ui->mainFCThread->isChecked();
    ui->mainFCThread->setChecked(false);
    for(int i=0;i<useFCBtnVec.size();++i){
        useFCBtnVec[i]->setChecked(false);
    }
    setForceControlSelectionEnabled(true);

    if(forceThreadWasChecked && ccThread && ccThread->isRunning()){
        delay(std::max(20, ui->devCtrlCycleMs->value() * 2));
    }
}

bool MainWindow::validateMotorCommandLimits(int axisIndex,
                                            double relativeTargetPos,
                                            double targetVel,
                                            const QString& actionName,
                                            QString* errorMessage) const
{
    if(errorMessage){
        errorMessage->clear();
    }

    if(axisIndex < 0 ||
            axisIndex >= static_cast<int>(axisMotorMinVec.size()) ||
            axisIndex >= static_cast<int>(axisMotorMaxVec.size()) ||
            axisIndex >= static_cast<int>(axisMotorVelMaxVec.size()) ||
            !axisMotorMinVec[axisIndex] ||
            !axisMotorMaxVec[axisIndex] ||
            !axisMotorVelMaxVec[axisIndex]){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1失败，电机轴%2限位配置不存在")
                    .arg(actionName)
                    .arg(axisIndex);
        }
        return false;
    }

    const double minPos = axisMotorMinVec[axisIndex]->value();
    const double maxPos = axisMotorMaxVec[axisIndex]->value();
    const double maxVel = axisMotorVelMaxVec[axisIndex]->value();
    if(!std::isfinite(relativeTargetPos) ||
            !std::isfinite(targetVel) ||
            !std::isfinite(minPos) ||
            !std::isfinite(maxPos) ||
            !std::isfinite(maxVel) ||
            maxPos <= minPos){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1失败，电机轴%2限位配置无效，位置范围[%3, %4]，速度上限%5")
                    .arg(actionName)
                    .arg(axisIndex)
                    .arg(minPos, 0, 'f', 6)
                    .arg(maxPos, 0, 'f', 6)
                    .arg(maxVel, 0, 'f', 6);
        }
        return false;
    }

    if(relativeTargetPos > maxPos || relativeTargetPos < minPos){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1不会下发，电机轴%2目标位置%3会导致电机超出软件位置限位[%4, %5]")
                    .arg(actionName)
                    .arg(axisIndex)
                    .arg(relativeTargetPos, 0, 'f', 6)
                    .arg(minPos, 0, 'f', 6)
                    .arg(maxPos, 0, 'f', 6);
        }
        return false;
    }

    if(maxVel > 1e-9 && std::fabs(targetVel) > maxVel){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1失败，电机轴%2目标速度%3超过软件速度上限%4")
                    .arg(actionName)
                    .arg(axisIndex)
                    .arg(std::fabs(targetVel), 0, 'f', 6)
                    .arg(maxVel, 0, 'f', 6);
        }
        return false;
    }

    return true;
}

bool MainWindow::waitMotorPositionStable(int timeoutMs, int sampleMs, double tolerance){
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        return false;
    }

    std::vector<double> lastPos = hardwareInterface.getAllMotorPos();
    if(lastPos.empty()){
        return false;
    }

    const int axisCount = std::min(static_cast<int>(lastPos.size()), ui->devAxisNum->value());
    int stableCount = 0;
    QElapsedTimer timer;
    timer.start();
    while(timer.elapsed() < timeoutMs){
        delay(sampleMs);
        const std::vector<double> curPos = hardwareInterface.getAllMotorPos();
        if(static_cast<int>(curPos.size()) < axisCount){
            return false;
        }

        double maxDelta = 0.0;
        for(int i=0;i<axisCount;++i){
            if(isMotorAxis(i)){
                maxDelta = std::max(maxDelta, std::fabs(curPos[i] - lastPos[i]));
            }
        }

        if(maxDelta <= tolerance){
            ++stableCount;
            if(stableCount >= 3){
                return true;
            }
        }
        else{
            stableCount = 0;
        }
        lastPos = curPos;
    }

    displayInfo("电机使能后位置未在等待时间内完全稳定，仍按当前读取值同步零点", "warning");
    return false;
}

bool MainWindow::waitMotorAxesDone(int timeoutMs, int sampleMs){
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        return false;
    }

    QElapsedTimer timer;
    timer.start();
    while(timer.elapsed() < timeoutMs){
        bool allDone = true;
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isMotorAxis(i) && !hardwareInterface.isMotorDone(i)){
                allDone = false;
                break;
            }
        }
        if(allDone){
            return true;
        }
        delay(sampleMs);
    }
    return false;
}

void MainWindow::syncMotorHomeAndDisplayZero(const QString& infoMessage,
                                             bool forceUseCurrentPosition){
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        return;
    }

    const bool useStaticMotorHome = !forceUseCurrentPosition && ui->axisIsStaticMotorHome->isChecked();
    hardwareInterface.useStaticMotorHome = useStaticMotorHome;
    std::vector<double> absPos = hardwareInterface.getAllMotorPos();
    if(absPos.empty()){
        displayInfo("错误：电机当前位置读取为空，无法同步电机零点", "error");
        return;
    }

    if(useStaticMotorHome){
        std::vector<double> staticHome(ui->devAxisNum->value(),0.0);
        for(int i=0;i<ui->devAxisNum->value();++i){
            staticHome[i] = axisMotorZeroVec[i]->value();
        }
        hardwareInterface.setMotorHome(staticHome);
    }
    else{
        hardwareInterface.setMotorHome(absPos);
    }

    const std::vector<double> motorHome = hardwareInterface.getAllMotorHome();
    const int axisCount = std::min(static_cast<int>(absPos.size()), ui->devAxisNum->value());
    motorCurAbsPos.assign(absPos.begin(), absPos.begin() + axisCount);
    motorCurPosRaw.clear();
    motorCurPos.clear();
    motorCurPosRaw.reserve(axisCount);
    motorCurPos.reserve(axisCount);

    for(int i=0;i<axisCount;++i){
        double relPos = absPos[i];
        if(i < static_cast<int>(motorHome.size())){
            relPos -= motorHome[i];
        }
        if(!useStaticMotorHome){
            relPos = 0.0;
        }
        motorCurPosRaw.push_back(relPos);
        motorCurPos.push_back(relPos);
    }

    motorPosDisplayZero.clear();
    updateSingleMotorActualPos();
    refreshMotorPosDisplay();
    if(!infoMessage.isEmpty()){
        displayInfo(infoMessage.toStdString());
    }
}

void MainWindow::setAllMotorHomeToCurrentPosition(){
    if(!ui->devUseLS->isChecked()){
        displayInfo("错误：当前仅支持雷赛电机位置清零", "error");
        return;
    }
    if(!hardwareInterface.isLSConnected()){
        displayInfo("错误：请先连接雷赛控制卡", "error");
        return;
    }

    syncMotorHomeAndDisplayZero(QStringLiteral("已将所有电机当前位置设为零位"), true);
}

void MainWindow::updateSingleMotorActualPos(){
    const int axisIndex = selectedSingleMotorIndex();
    if(axisIndex < 0){
        return;
    }

    double actualPos = 0.0;
    if(hardwareInterface.isLSConnected()){
        actualPos = hardwareInterface.readMotorCurPos(axisIndex);
        std::vector<double> motorHome = hardwareInterface.getAllMotorHome();
        if(axisIndex < motorHome.size()){
            actualPos -= motorHome[axisIndex];
        }
    }
    ui->SinglemotorActpos->setValue(actualPos);
}

void MainWindow::refreshSingleMotorEnableStateUi(){
    if(!ui || !ui->SinglemotorEnable){
        return;
    }

    const int axisIndex = selectedSingleMotorIndex();
    bool enabled = false;
    if(axisIndex >= 0 &&
            ui->devUseLS->isChecked() &&
            hardwareInterface.isLSConnected() &&
            isMotorAxis(axisIndex)){
        enabled = hardwareInterface.isMotorEnabled(axisIndex);
    }

    const QString text = enabled ?
                QStringLiteral("电机已使能") :
                QStringLiteral("使能电机");
    if(ui->SinglemotorEnable->text() != text){
        ui->SinglemotorEnable->setText(text);
    }
}
//单电机点位运动
void MainWindow::singleMotorEnable(){
    const int axisIndex = selectedSingleMotorIndex();
    if(axisIndex < 0){
        displayInfo("错误：电机轴号超出范围","error");
        return;
    }
    if(!ui->devUseLS->isChecked()){
        displayInfo("错误：当前仅支持雷赛单电机调试","error");
        return;
    }
    if(!hardwareInterface.isLSConnected()){
        displayInfo("错误：请先连接雷赛控制卡","error");
        return;
    }
    if(!applyLeadshineHardwareConfigFromUi()){
        displayInfo("错误：单电机使能失败，雷赛轴参数未正确配置","error");
        return;
    }

    disableForceControlForManualMotorTest();
    updateControlWorkerConfig();
    if(!hardwareInterface.setMotorEnable(axisIndex, true)){
        displayInfo("错误：单电机使能失败","error");
        return;
    }
    if(ccThread && !ccThread->isRunning()){
        ccThread->start(QThread::HighPriority);
    }
    updateSingleMotorActualPos();
    refreshSingleMotorEnableStateUi();
    displayInfo(QString("电机轴%1已使能").arg(axisIndex).toStdString());
}

void MainWindow::singleMotorStart(){
    const int axisIndex = selectedSingleMotorIndex();
    if(axisIndex < 0){
        displayInfo("错误：电机轴号超出范围","error");
        return;
    }
    if(!ui->devUseLS->isChecked()){
        displayInfo("错误：当前仅支持雷赛单电机调试","error");
        return;
    }
    if(!hardwareInterface.isLSConnected()){
        displayInfo("错误：请先连接雷赛控制卡","error");
        return;
    }

    disableForceControlForManualMotorTest();
    if(!hardwareInterface.isMotorEnabled(axisIndex)){
        refreshSingleMotorEnableStateUi();
        displayInfo("错误：请先使能当前电机，再开始运动","error");
        return;
    }

    if(ccThread && !ccThread->isRunning()){
        ccThread->start(QThread::HighPriority);
    }

    const double relativeTargetPos = ui->SinglemotorPos->value();
    const double targetVel = std::fabs(ui->SinglemotorVel->value());
    QString limitError;
    if(!validateMotorCommandLimits(axisIndex,
                                   relativeTargetPos,
                                   targetVel,
                                   QStringLiteral("单电机点位启动"),
                                   &limitError)){
        hardwareInterface.motorStop(axisIndex);
        displayInfo(limitError.toStdString(), "error");
        return;
    }

    updateControlWorkerConfig();
    double targetPos = relativeTargetPos;
    std::vector<double> motorHome = hardwareInterface.getAllMotorHome();
    if(axisIndex < motorHome.size()){
        targetPos += motorHome[axisIndex];
    }

    if(!hardwareInterface.motorAbsPos(axisIndex, targetPos, targetVel)){
        displayInfo("错误：单电机点位启动失败","error");
        return;
    }

    updateSingleMotorActualPos();
    refreshSingleMotorEnableStateUi();
    displayInfo(QString("电机轴%1开始运动").arg(axisIndex).toStdString());
}

void MainWindow::singleMotorStop(){
    const int axisIndex = selectedSingleMotorIndex();
    if(axisIndex < 0){
        displayInfo("错误：电机轴号超出范围","error");
        return;
    }
    if(!ui->devUseLS->isChecked()){
        displayInfo("错误：当前仅支持雷赛单电机调试","error");
        return;
    }
    if(!hardwareInterface.isLSConnected()){
        displayInfo("错误：请先连接雷赛控制卡","error");
        return;
    }

    if(!hardwareInterface.motorStop(axisIndex)){
        displayInfo("错误：单电机停止失败","error");
        return;
    }

    updateSingleMotorActualPos();
    refreshSingleMotorEnableStateUi();
    displayInfo(QString("电机轴%1已停止，保持使能状态").arg(axisIndex).toStdString());
}

void MainWindow::singleMotorUpdatePos(){
    updateSingleMotorActualPos();
    refreshSingleMotorEnableStateUi();
    ui->SinglemotorPos->setValue(ui->SinglemotorActpos->value());
}

void MainWindow::resetMotorPosDisplayZero(){
    if(!ui->devUseLS->isChecked()){
        displayInfo("错误：当前仅支持雷赛电机位置清零", "error");
        return;
    }
    if(!hardwareInterface.isLSConnected()){
        displayInfo("错误：请先连接雷赛控制卡", "error");
        return;
    }

    syncMotorHomeAndDisplayZero(ui->axisIsStaticMotorHome->isChecked()
                                ? QStringLiteral("已按静态电机零点重置电机位置")
                                : QStringLiteral("已按当前位置重置电机位置"));
}

void MainWindow::refreshMotorPosDisplay(){
    const bool showAbs = ui->Absmotorpos->isChecked();
    const int axisCount = std::min((int)curMotorPosTextVec.size(), ui->devAxisNum->value());
    for(int i=0;i<axisCount;++i){
        double value = 0.0;
        if(showAbs){
            if(i < motorCurAbsPos.size()){
                value = motorCurAbsPos[i];
            }
        }
        else if(i < motorPosDisplayZero.size() && i < motorCurAbsPos.size()){
            value = motorCurAbsPos[i] - motorPosDisplayZero[i];
        }
        else if(i < motorCurPosRaw.size()){
            value = motorCurPosRaw[i];
        }
        curMotorPosTextVec[i]->setText(QString::number(value, 'f', 3));
    }
}

bool MainWindow::runPoseMode(){
    const RobotStateSnapshot robotState = currentRobotState();
    const bool poseSimulationTestBypass = kPoseSimulationTestBypass;
    const bool useTrajectoryFileInput = shouldUseTrajectoryFileInput();
    const bool realtimeManualInput = isRealtimeManualPoseInputSelected();
    const bool simulationOnlyMode = runtimeState.runMode == RunMode::SimulationCalculation;
    const bool requireExecutionPrecheck =
            !poseSimulationTestBypass &&
            !simulationOnlyMode;
    if(requireExecutionPrecheck && !robotState.systemRunning){
        displayInfo("错误：位置模式启动失败，请先启动电机使能","error");
        return false;
    }
    bool cableHomeCheckedForThisStart = false;
    bool forwardKinematicsRefreshedForThisStart = true;
    if(requireExecutionPrecheck && !ui->devUseCamForActPose->isChecked()){
        if(!ensureCableHomeReadyForMotion(QStringLiteral("位置模式"))){
            return false;
        }
        cableHomeCheckedForThisStart = true;
        forwardKinematicsRefreshedForThisStart = refreshForwardKinematicsPoseFromCurrentMotorPosition();
    }
    if(requireExecutionPrecheck && !hasValidEstimatedEndPose()){
        displayInfo(!ui->devUseCamForActPose->isChecked() && !forwardKinematicsRefreshedForThisStart ?
                    "错误：位置模式启动失败，无法根据当前电机位置刷新正运动学位姿，请检查轴类型、末端编号、参数配置和电机位置反馈" :
                    ui->devUseCamForActPose->isChecked() ?
                    "错误：位置模式启动失败，尚未收到有效实际位姿更新" :
                    "错误：位置模式启动失败，尚未生成有效正运动学位姿，请确认预紧确认已完成且电机位置反馈正常",
                    "error");
        return false;
    }
    if(robotState.gcRunning){
        displayInfo("错误：位置模式启动失败，零力拖动/动力学阻抗正在运行，请先停止力控轨迹","error");
        return false;
    }
    if(robotState.positionMotionState == PositionMotionState::ExecutingPvt ||
            robotState.positionMotionState == PositionMotionState::PausedPvt){
        displayInfo("错误：位置模式启动失败，当前已有位置轨迹正在执行或暂停","error");
        return false;
    }
    if(requireExecutionPrecheck && !cableHomeCheckedForThisStart &&
            !ensureCableHomeReadyForMotion(QStringLiteral("位置模式"))){
        return false;
    }

    //这是防止轨迹初始点和当前位置不一致做的判断，避免了更复杂的自动移动到起始点
    if(requireExecutionPrecheck){
        std::vector<double> currentPose;
        if(currentEstimatedEndPose(currentPose) && currentPose.size() >= 6){
            const std::vector<double> startPose = {
                ui->mainPosModeTargetStartPx->value(),
                ui->mainPosModeTargetStartPy->value(),
                ui->mainPosModeTargetStartPz->value(),
                ui->mainPosModeTargetStartRx->value(),
                ui->mainPosModeTargetStartRy->value(),
                ui->mainPosModeTargetStartRz->value()
            };
            bool poseMismatch = false;
            for(int i=0;i<6;++i){
                if(std::abs(currentPose[i] - startPose[i]) > 1e-3){
                    poseMismatch = true;
                    break;
                }
            }
            if(poseMismatch){
                displayInfo("警告：轨迹起始点与当前实际位姿不一致，请先移动到起始点","warning");
                return false;
            }
        }
    }

    if(robotState.posModeRunning){
        if(simulationWorker){
            simulationWorker->stopPositionSimulation();
        }
        positionSimulationModel = nullptr;
        runtimeState.autoExecutePoseAfterSimulation = false;
        clearPlannedPoseForceTrajectory();
        runtimeState.posModeRunning = false;
        runtimeState.pvtCommandActive = false;
        ui->mainPosModeSwitch->setEnabled(true);
        ui->mainPosModeSend->setEnabled(true);
        //ui->mainGCModeSwitch->setEnabled(true);
        ui->mainReplay3DViewer->setEnabled(true);
        setPoseSimulationIdleText(ui);
    }
    else{
        if(poseSimulationTestBypass){
            displayInfo("测试模式：已绕过电机使能、有效动捕、绳索零点和起始位姿一致性前提，仅用于3D仿真与离线轨迹文件测试","warning");
        }
        clearPlannedPoseForceTrajectory();
        clearPreReplayData();
        const QString posTrajMode = realtimeManualInput ?
                    kPoseTrajStraight :
                    normalizePoseTrajectoryModeName(ui->PostrajMode->currentText());
        TrajectoryPlanner::MultiEndTrajectory plannedEndTraj;

        if(useTrajectoryFileInput){
            if(trajFileOfflineTraj.empty()){
                displayInfo("错误：位置轨迹文件数据为空，请重新读取轨迹文件", "error");
                return false;
            }
            plannedEndTraj = trajFileOfflineTraj;
            TrajectoryPlanner::resampleTrajectory(plannedEndTraj, ui->mainPosModeTargetStepTime->value());
        }
        else{
            if(posTrajMode == kPoseTrajStraight){
                TrajectoryPlanner::EndpointRequest request;
                request.type = TrajectoryPlanner::TrajType::Quintic;
                request.q_param = {
                    {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                     ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()},
                    {0,0,0,0,0,0},
                    {0,0,0,0,0,0},
                    {ui->mainPosModeTargetEndPx->value(),ui->mainPosModeTargetEndPy->value(),ui->mainPosModeTargetEndPz->value(),
                     ui->mainPosModeTargetEndRx->value(),ui->mainPosModeTargetEndRy->value(),ui->mainPosModeTargetEndRz->value()},
                    {0,0,0,0,0,0},
                    {0,0,0,0,0,0},
                    ui->mainPosModeTargetRunTime->value(),
                    ui->mainPosModeTargetStepTime->value()
                };
                plannedEndTraj = TrajectoryPlanner::buildLineTrajectory({request}, request.type);
                if(plannedEndTraj.empty()){
                    displayInfo("错误：直线轨迹规划失败，请检查运行时间和步长", "error");
                    return false;
                }
            }
            else if(posTrajMode == kPoseTrajCircular){
                TrajectoryPlanner::EndpointRequest request;
                request.type = TrajectoryPlanner::TrajType::Circular;
                request.cir_param.startPose = {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                                               ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()};
                request.cir_param.center = {ui->posTrajCircularCenterX->value(),ui->posTrajCircularCenterY->value(),ui->posTrajCircularCenterZ->value()};
                request.cir_param.radius = ui->posTrajCircularRadius->value();
                request.cir_param.omega = ui->posTrajCircularOmega->value();
                request.cir_param.direction = ui->posTrajCircularDirection->currentIndex() == 0 ? -1 : 1;
                request.cir_param.stepTime = ui->mainPosModeTargetStepTime->value();
                plannedEndTraj = TrajectoryPlanner::buildLineTrajectory({request},request.type);
                if(plannedEndTraj.empty()){
                    displayInfo("错误：圆形轨迹规划失败，请检查圆心、半径、角速度和步长", "error");
                    return false;
                }
            }
            else if(posTrajMode == kPoseTrajLineAccel){
                TrajectoryPlanner::EndpointRequest request;
                request.type = TrajectoryPlanner::TrajType::SCurve;
                request.s_param.startPose = {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                                             ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()};
                request.s_param.endPose = {ui->mainPosModeTargetEndPx->value(),ui->mainPosModeTargetEndPy->value(),ui->mainPosModeTargetEndPz->value(),
                                             ui->mainPosModeTargetEndRx->value(),ui->mainPosModeTargetEndRy->value(),ui->mainPosModeTargetEndRz->value()};
                request.s_param.vmax = ui->posTrajSCurveVmax->value();
                request.s_param.acc = ui->posTrajSCurveAcc->value();
                request.s_param.dec = ui->posTrajSCurveDec->value();
                request.s_param.stepTime = ui->mainPosModeTargetStepTime->value();
                plannedEndTraj = TrajectoryPlanner::buildLineTrajectory({request},request.type);
                if(plannedEndTraj.empty()){
                    displayInfo("错误：匀加减速直线轨迹规划失败，请检查最大速度、加速度和减速度", "error");
                    return false;
                }
            }
            else if(posTrajMode == kPoseTrajEightShape){
                TrajectoryPlanner::EndpointRequest request;
                request.type = TrajectoryPlanner::TrajType::EightShape;
                request.e_param.startPose = {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                                             ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()};
                request.e_param.normal = {ui->posTrajEightShapeNormalX->value(),ui->posTrajEightShapeNormalY->value(),ui->posTrajEightShapeNormalZ->value()};
                request.e_param.R = ui->posTrajEightShapeR->value();
                request.e_param.range = ui->posTrajEightShapeRange->value();
                request.e_param.duration = ui->mainPosModeTargetRunTime->value();
                request.e_param.stepTime = ui->mainPosModeTargetStepTime->value();
                plannedEndTraj = TrajectoryPlanner::buildLineTrajectory({request},request.type);
                if(plannedEndTraj.empty()){
                    displayInfo("错误：变姿态8字形轨迹规划失败，请检查法向量、半径和姿态变化幅度", "error");
                    return false;
                }
            }
            else if(posTrajMode == kPoseTrajSineAccel){
                TrajectoryPlanner::EndpointRequest request;
                request.type = TrajectoryPlanner::TrajType::Sine;
                request.sin_param.startPose = {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                                             ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()};
                request.sin_param.endPose = {ui->mainPosModeTargetEndPx->value(),ui->mainPosModeTargetEndPy->value(),ui->mainPosModeTargetEndPz->value(),
                                          ui->mainPosModeTargetEndRx->value(),ui->mainPosModeTargetEndRy->value(),ui->mainPosModeTargetEndRz->value()};
                request.sin_param.A = ui->posTrajSineA->value();
                request.sin_param.w = ui->posTrajSineW->value();
                request.sin_param.phi = ui->posTrajSinePhi->value();
                request.sin_param.duration = ui->mainPosModeTargetRunTime->value();
                request.sin_param.stepTime = ui->mainPosModeTargetStepTime->value();
                plannedEndTraj = TrajectoryPlanner::buildLineTrajectory({request},request.type);
                if(plannedEndTraj.empty()){
                    displayInfo("错误：正弦变加速轨迹规划失败，请检查振幅、角频率和初相位", "error");
                    return false;
                }
            }
            else if(posTrajMode == kPoseTrajCubic){
                TrajectoryPlanner::EndpointRequest request;
                request.type = TrajectoryPlanner::TrajType::Cubic;
                request.cub_param = {
                    {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                     ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()},
                    {ui->posTrajCubicStartVelX->value(),ui->posTrajCubicStartVelY->value(),ui->posTrajCubicStartVelZ->value(),
                     ui->posTrajCubicStartVelRx->value(),ui->posTrajCubicStartVelRy->value(),ui->posTrajCubicStartVelRz->value()},
                    {ui->mainPosModeTargetEndPx->value(),ui->mainPosModeTargetEndPy->value(),ui->mainPosModeTargetEndPz->value(),
                     ui->mainPosModeTargetEndRx->value(),ui->mainPosModeTargetEndRy->value(),ui->mainPosModeTargetEndRz->value()},
                    {ui->posTrajCubicEndVelX->value(),ui->posTrajCubicEndVelY->value(),ui->posTrajCubicEndVelZ->value(),
                     ui->posTrajCubicEndVelRx->value(),ui->posTrajCubicEndVelRy->value(),ui->posTrajCubicEndVelRz->value()},
                    ui->mainPosModeTargetRunTime->value(),
                    ui->mainPosModeTargetStepTime->value()
                };
                plannedEndTraj = TrajectoryPlanner::buildLineTrajectory({request},request.type);
                if(plannedEndTraj.empty()){
                    displayInfo("错误：三次多项式轨迹规划失败，请检查运行时间和步长", "error");
                    return false;
                }
            }
            else if(posTrajMode == kPoseTrajStepAccel){
                TrajectoryPlanner::EndpointRequest request;
                request.type = TrajectoryPlanner::TrajType::StepAccel;
                request.stepAccel_param.startPose = {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                                                     ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()};
                request.stepAccel_param.dir = {ui->posTrajStepAccelDirX->value(),ui->posTrajStepAccelDirY->value(),ui->posTrajStepAccelDirZ->value()};
                request.stepAccel_param.a_before = ui->posTrajStepAccelABefore->value();
                request.stepAccel_param.a_after = ui->posTrajStepAccelAAfter->value();
                request.stepAccel_param.t_step = ui->posTrajStepAccelTStep->value();
                request.stepAccel_param.stepTime = ui->mainPosModeTargetStepTime->value();
                plannedEndTraj = TrajectoryPlanner::buildLineTrajectory({request},request.type);
                if(plannedEndTraj.empty()){
                    displayInfo("错误：阶跃变加速轨迹规划失败，请检查运动方向和阶跃参数", "error");
                    return false;
                }
            }
            else{
                displayInfo(QStringLiteral("错误：未识别的位置轨迹类型：%1").arg(posTrajMode).toStdString(), "error");
                return false;
            }
        }

        QString workspaceCheckError;
        if(!validateTrajectoryWithinWorkspace(plannedEndTraj, workspaceCheckError)){
            if(simulationOnlyMode){
                displayInfo(QStringLiteral("警告：%1；当前为仿真计算模式，继续保留轨迹仿真用于分析")
                            .arg(workspaceCheckError).toStdString(),
                            "warning");
            }
            else{
                displayInfo(QStringLiteral("错误：%1").arg(workspaceCheckError).toStdString(), "error");
                return false;
            }
        }

        lastPlannedPoseTrajectoryEndPose.clear();
        if(!extractTrajectoryEndpointPose(plannedEndTraj, lastPlannedPoseTrajectoryEndPose)){
            displayInfo("警告：未能记录本段轨迹终点位姿，连续轨迹完成后将尝试使用当前正运动学位姿同步下一段起点",
                        "warning");
        }

        HybridPoseForceModeConfig hybridModeConfig;
        QString hybridModeError;
        if(!buildHybridPoseForceModeConfig(hybridModeConfig, hybridModeError)){
            displayInfo(hybridModeError.toStdString(), "error");
            return false;
        }

        QString forcePlanError;
        if(!planPoseForceTrajectory(plannedEndTraj, forcePlanError)){
            if(hybridModeConfig.enabled){
                clearPlannedPoseForceTrajectory();
                displayInfo(QStringLiteral("警告：位置轨迹规划通过，但全8绳力分配校验失败。%1；当前按“6绳位控 + 2绳恒张力保持”模式继续仿真")
                            .arg(forcePlanError).toStdString(), "warning");
            }
            else{
                displayInfo(QStringLiteral("错误：位置轨迹规划通过，但力分配校验失败。%1").arg(forcePlanError).toStdString(), "error");
                return false;
            }
        }

        std::vector<std::vector<std::vector<double>>> cableContactPointPos = buildCableContactPointPos();
        std::vector<std::vector<double>> anchorCableCoorHome = buildFixedAnchorHome();
        std::vector<double> cableMotorCof = buildCableMotorCof();
        std::vector<double> zeroAnchorDis(anchorCableCoorHome.size(),0.0);
        if(simulationWorker){
            simulationWorker->stopPositionSimulation();
        }
        positionSimulationModel = nullptr;
        lastTrajEndAnchorCoor = anchorCableCoorHome;
        qDebug() << "lastTrajEndAnchorCoor:" << lastTrajEndAnchorCoor;

        positionSimulationModel = new PositionSimulationModel(ui->devCtrlCycleMs->value(), ui->devFrameL->value(),ui->devFrameW->value(),
                                                              zeroAnchorDis,cableMotorCof,cableContactPointPos,&lastTrajEndAnchorCoor,
                                                              zeroAnchorDis,true,
                                                              buildPulleyRadius());
        positionSimulationModel->setStaticAnchor(true);
        connect(positionSimulationModel,&PositionSimulationModel::displayInfoSignal,this,&MainWindow::displayInfo);
        connect(positionSimulationModel,&PositionSimulationModel::update3DViewerSignal,this,&MainWindow::update3DViewer);
        connect(positionSimulationModel,&PositionSimulationModel::poseCtrlStartSignal,this,&MainWindow::clearPreReplayData);
        connect(positionSimulationModel,&PositionSimulationModel::poseCtrlEndSignal,this,&MainWindow::handlePoseModeSimulationFinished);
        positionSimulationModel->setOfflineEndTraj(plannedEndTraj);
        if(simulationWorker){
            simulationWorker->setPositionSimulationModel(positionSimulationModel);
            simulationWorker->startPositionSimulation();
        }

        runtimeState.posModeRunning = true;
        setPoseSimulationBusyText(ui);
        ui->mainPosModeSwitch->setEnabled(false);
        ui->mainPosModeSend->setEnabled(false);
        ui->mainReplay3DViewer->setEnabled(false);

        // 在执行了位控后，和重力补偿冲突，同时也不宜更新零点
        ui->mainResetCableForceHomeSwitch->setEnabled(false);
//        ui->mainGCModeSwitch->setEnabled(false);

        // test
//        qDebug() << "TEST:" << positionSimulationModel->anchorMoveDis2AnchorCableCoor(std::vector<double>(8,0.0));
    }
    return true;
}

void MainWindow::handlePoseModeSimulationFinished(){
    if(!positionSimulationModel){
        return;
    }
    if(positionSimulationModel->QThread::isRunning()){
        positionSimulationModel->wait(500);
    }

    runtimeState.posModeRunning = false;
    runtimeState.pvtCommandActive = false;
    ui->mainPosModeSwitch->setEnabled(true);
    setPoseSimulationIdleText(ui);
    ui->mainPosModeSend->setEnabled(true);
    ui->mainReplay3DViewer->setEnabled(true);
    displayInfo("轨迹仿真完成");

    if(runtimeState.runMode == RunMode::SimulationCalculation){
        QString cableLengthFilePath;
        if(writePoseSimulationCableLengthFile(&cableLengthFilePath)){
            displayInfo(QStringLiteral("仿真轨迹绳长已导出至 %1")
                        .arg(cableLengthFilePath).toStdString());
        }
        else{
            displayInfo("警告：仿真轨迹绳长未导出，请确认仿真已完整完成且绳长数据非空", "warning");
        }
    }

    if(kPoseSimulationTestBypass){
        runtimeState.autoExecutePoseAfterSimulation = false;
        displayInfo("测试模式：已阻止仿真结束后的自动位控下发，仅保留仿真验证链路","warning");
        return;
    }

    const bool shouldAutoExecute = runtimeState.autoExecutePoseAfterSimulation;
    runtimeState.autoExecutePoseAfterSimulation = false;
    if(shouldAutoExecute){
        startPosePvtTrajectory();
    }
}

void MainWindow::readTrajFile(){
    if(!useTrajFile){
        QStringList filesPath = QFileDialog::getOpenFileNames(this,"选择轨迹点文件","","(*.txt)");
        if(filesPath.isEmpty()){// 若未设置路径
            return;
        }

        TrajectoryPlanner::FileTrajectory fileTraj;
        QString errorMessage;
        if(!TrajectoryPlanner::loadPoseFile(filesPath[0], ui->devEndNum->value(), fileTraj, errorMessage)){
            displayInfo(errorMessage.toStdString(), "error");
            return;
        }

        trajFileOfflineTraj = fileTraj.offlineTraj;
        ui->mainPosModeTargetRunTime->setValue(fileTraj.duration);

        trajFileTrajPx1 = trajFileOfflineTraj[0][0][0];
        trajFileTrajPy1 = trajFileOfflineTraj[0][0][1];
        trajFileTrajPz1 = trajFileOfflineTraj[0][0][2];
        trajFileTrajRx1 = trajFileOfflineTraj[0][0][3];
        trajFileTrajRy1 = trajFileOfflineTraj[0][0][4];
        trajFileTrajRz1 = trajFileOfflineTraj[0][0][5];

        std::vector<double> uiStartPose = {ui->mainPosModeTargetStartPx->value(),ui->mainPosModeTargetStartPy->value(),ui->mainPosModeTargetStartPz->value(),
                                           ui->mainPosModeTargetStartRx->value(),ui->mainPosModeTargetStartRy->value(),ui->mainPosModeTargetStartRz->value()};
        bool startPoseMismatch = false;
        for(int i=0;i<6;++i){
            if(std::abs(fileTraj.firstEndStartPose[i] - uiStartPose[i]) > 1e-3){
                startPoseMismatch = true;
                break;
            }
        }
        if(startPoseMismatch){
            displayInfo("警告：轨迹文件起始点与当前位姿不一致，已自动更新起始位姿","warning");
            ui->mainPosModeTargetStartPx->setValue(trajFileTrajPx1[0]);
            ui->mainPosModeTargetStartPy->setValue(trajFileTrajPy1[0]);
            ui->mainPosModeTargetStartPz->setValue(trajFileTrajPz1[0]);
            ui->mainPosModeTargetStartRx->setValue(trajFileTrajRx1[0]);
            ui->mainPosModeTargetStartRy->setValue(trajFileTrajRy1[0]);
            ui->mainPosModeTargetStartRz->setValue(trajFileTrajRz1[0]);
        }

        useTrajFile = true;
    }
    else{
        clearTrajectoryFileSelection();
    }
    refreshRunModeUiState();
}

bool MainWindow::startPosePvtTrajectory(){
    const RobotStateSnapshot robotState = currentRobotState();
    if(!ensureSafetyReadyForMotion(QStringLiteral("位控轨迹下发"))){
        return false;
    }
    if(runtimeState.runMode == RunMode::SimulationCalculation){
        displayInfo("仿真计算模式仅支持上位机仿真，不向执行机构下发位控轨迹", "warning");
        return false;
    }
    if(robotState.gcRunning){
        displayInfo("错误：位控轨迹下发失败，零力拖动/动力学阻抗正在运行，请先停止力控轨迹","error");
        return false;
    }
    if(!ensureCableHomeReadyForMotion(QStringLiteral("位控轨迹"))){
        return false;
    }

    if(!positionSimulationModel){
        displayInfo("错误：发送的位控信息为空，请先进行仿真。","error");
        return false;
    }

    HybridPoseForceModeConfig hybridModeConfig;
    QString hybridModeError;
    if(!buildHybridPoseForceModeConfig(hybridModeConfig, hybridModeError)){
        displayInfo(hybridModeError.toStdString(), "error");
        return false;
    }
    runtimeState.hybridPoseForceModeActive = false;
    if(!hybridModeConfig.enabled){
        clearHybridPoseForceModeState(false);
    }

    // 输出弧度rad（根据ui里的电机系数计算）
    // 输出向量内元素下标与轴对应关系是：前几个是第一个末端的电机，接着是第二个末端，第三个末端...
    // 第一个末端内电机的顺序和输入量的电机顺序一致，即在“轴参数”页面从上到下排，跳过那些不是第一个末端的。第二个末端内电机排序同理
    std::vector<std::vector<double>> cableMotorThetaTrajVec = positionSimulationModel->getCableMotorThetaTraj();
    std::vector<std::vector<double>> anchorTimeStampTrajVec = positionSimulationModel->getAnchorTimeStampTraj();

    if(cableMotorThetaTrajVec.empty() || anchorTimeStampTrajVec.empty()){
        displayInfo("错误：发送的位控信息为空，请先进行仿真。","error");
        return false;
    }

    qDebug() << "====== Sending pose mode data to Leadshine PVT ======";
    qDebug() << "cableMotorThetaTraj size:" << cableMotorThetaTrajVec.size() << "cableMotorThetaTraj[0] size:" << cableMotorThetaTrajVec[0].size();
    qDebug() << "anchorTimeStampTraj size:" << anchorTimeStampTrajVec.size() << "anchorTimeStampTraj[0] size:" << anchorTimeStampTrajVec[0].size();
    for(int i=0;i<cableMotorThetaTrajVec.size();++i){
        qDebug() << "cableMotorThetaTraj" << i << cableMotorThetaTrajVec[i];
        qDebug() << "anchorTimeStampTraj" << i << anchorTimeStampTrajVec[i];
    }

    int trajNum = static_cast<int>(cableMotorThetaTrajVec[0].size());

    if(ui->devUseLS->isChecked()){
        double tmpCof = 1.0;
        if(ui->devMotorFeedbackIsRd->isChecked()){// 弧度转圈数
            tmpCof = 1.0/(2.0*M_PI);
        }
        else if(ui->devMotorFeedbackIsTheta->isChecked()){// 弧度转度
            tmpCof = 180.0/M_PI;
        }

        std::vector<int> cableMotorIndex,anchorMotorIndex,cableForceSensorIndex;
        std::vector<double> cableMotorHardwareDirection;
        for(int endIndex=0;endIndex<ui->devEndNum->value();++endIndex){
            for(int i=0;i<ui->devAxisNum->value();++i){
                if(isModeledMotorAxis(i) && axisEndVec[i]->value() == endIndex){
                    cableMotorIndex.push_back(i);// 记录需要控制的轴号，并且向量内元素排序和位控模块输出一致
                    cableMotorHardwareDirection.push_back(motorHardwareDirectionSign(i));
                    // 找出对应的力传感器，用于后续位控+力控时排除力控绳索的判断。没有力传感器就是-1
                    cableForceSensorIndex.push_back(axisSensorIndexVec[i]->value());
                }
            }
        }
        if(lastTrajEndMotorTheta.size() != cableMotorIndex.size()){
            lastTrajEndMotorTheta.assign(cableMotorIndex.size(), 0.0);
        }

        std::vector<std::vector<double>> allTrajForPos(trajNum,std::vector<double>(cableMotorIndex.size(),0.0));
        std::vector<std::vector<double>> allTrajVelForPos(trajNum,std::vector<double>(cableMotorIndex.size(),0.0));
        for(int trajIndex=0;trajIndex<cableMotorThetaTrajVec[0].size();++trajIndex){
            for(int srcAxis=0;srcAxis<cableMotorIndex.size();++srcAxis){
                const double hardwareDirection =
                        srcAxis < static_cast<int>(cableMotorHardwareDirection.size()) ?
                            cableMotorHardwareDirection[srcAxis] :
                            1.0;
                allTrajForPos[trajIndex][srcAxis] =
                        tmpCof * hardwareDirection * cableMotorThetaTrajVec[srcAxis][trajIndex] +
                        lastTrajEndMotorTheta[srcAxis];
                if(trajIndex == 0){
                    allTrajVelForPos[trajIndex][srcAxis] = 0.0;
                }
                else{
                    allTrajVelForPos[trajIndex][srcAxis] =
                            tmpCof * hardwareDirection *
                            (cableMotorThetaTrajVec[srcAxis][trajIndex] - cableMotorThetaTrajVec[srcAxis][trajIndex-1]) /
                            (anchorTimeStampTrajVec[srcAxis][trajIndex] - anchorTimeStampTrajVec[srcAxis][trajIndex-1]);
                }
            }
        }

        if(hybridModeConfig.enabled){
            runtimeState.hybridPoseForceModeActive = true;
            activeHybridPoseForceConfig = hybridModeConfig;
            activeHybridExpectedForceTraj.clear();
            activeHybridExpectedForcePointIndex = -1;

            const bool usePlannedExpectedForceTraj = runtimeState.runMode == RunMode::ProgramControl;
            if(usePlannedExpectedForceTraj){
                if(plannedPoseExpectedForceTraj.empty()){
                    clearHybridPoseForceModeState(false);
                    displayInfo("力位混合执行失败：程序控制模式下未生成轨迹点期望绳力，请先完成轨迹力分配计算", "error");
                    return false;
                }

                const int expectedPointCount = static_cast<int>(plannedPoseExpectedForceTraj.front().size());
                if(expectedPointCount != trajNum){
                    clearHybridPoseForceModeState(false);
                    displayInfo(QStringLiteral("力位混合执行失败：期望绳力轨迹点数%1与位控轨迹点数%2不一致")
                                .arg(expectedPointCount)
                                .arg(trajNum)
                                .toStdString(),
                                "error");
                    return false;
                }

                for(int sensorIndex : hybridModeConfig.forceSensorIndex){
                    if(sensorIndex < 0 ||
                       sensorIndex >= static_cast<int>(plannedPoseExpectedForceTraj.size()) ||
                       static_cast<int>(plannedPoseExpectedForceTraj[sensorIndex].size()) != expectedPointCount){
                        clearHybridPoseForceModeState(false);
                        displayInfo("力位混合执行失败：所选力控绳索缺少完整的期望绳力轨迹", "error");
                        return false;
                    }
                }

                activeHybridExpectedForceTraj = plannedPoseExpectedForceTraj;
                QString hybridExpectedForceError;
                if(!applyHybridPoseForceExpectedForceAtPoint(0, &hybridExpectedForceError)){
                    clearHybridPoseForceModeState(false);
                    displayInfo(hybridExpectedForceError.toStdString(), "error");
                    return false;
                }
                displayInfo("力位混合执行：程序控制模式将按每个轨迹点计算绳力，实时更新2根力控绳索期望值");
            }
            else{
                if(controlWorker){
                    controlWorker->setExternalExpectedForce(hybridModeConfig.frozenExpectedForce);
                }
                mainForceSensorExpVal = hybridModeConfig.frozenExpectedForce;
                displayInfo("力位混合执行：实时模式将按6根绳索位置控制、2根绳索固定期望力控保持执行");
            }

            ui->mainFCThread->setChecked(true);
            requestImmediateForceControlUpdate();
        }

        std::vector<int> controlledMotorIndex;
        std::vector<int> controlledAxisColumn;
        for(int i=0;i<cableForceSensorIndex.size();++i){
            const bool forceCtrlEnabled = hybridModeConfig.enabled ?
                        (std::find(hybridModeConfig.forceAxisIndex.begin(),
                                   hybridModeConfig.forceAxisIndex.end(),
                                   cableMotorIndex[i]) != hybridModeConfig.forceAxisIndex.end()) :
                        (cableForceSensorIndex[i] >= 0 &&
                         cableForceSensorIndex[i] < useFCBtnVec.size() &&
                         useFCBtnVec[cableForceSensorIndex[i]]->isChecked() &&
                         ui->mainFCThread->isChecked());
            if(forceCtrlEnabled){
                continue;
            }
            controlledMotorIndex.push_back(cableMotorIndex[i]);
            controlledAxisColumn.push_back(i);
        }

        if(controlledMotorIndex.empty()){
            clearHybridPoseForceModeState(false);
            displayInfo("错误：位控轨迹没有可控制电机轴","error");
            return false;
        }

        std::vector<std::vector<double>> controlledTrajForPos(trajNum,std::vector<double>(controlledAxisColumn.size(),0.0));
        std::vector<std::vector<double>> controlledTrajVelForPos(trajNum,std::vector<double>(controlledAxisColumn.size(),0.0));
        for(int trajIndex=0;trajIndex<allTrajForPos.size();++trajIndex){
            for(int cIdx=0;cIdx<controlledAxisColumn.size();++cIdx){
                controlledTrajForPos[trajIndex][cIdx] = allTrajForPos[trajIndex][controlledAxisColumn[cIdx]];
            }
        }
        for(int trajIndex=0;trajIndex<allTrajVelForPos.size();++trajIndex){
            for(int cIdx=0;cIdx<controlledAxisColumn.size();++cIdx){
                controlledTrajVelForPos[trajIndex][cIdx] = allTrajVelForPos[trajIndex][controlledAxisColumn[cIdx]];
            }
        }

        for(int i=0;i<controlledTrajForPos.size();++i){
            for(int ii=0;ii<controlledTrajForPos[i].size();++ii){
                if(controlledTrajForPos[i][ii]>axisMotorMaxVec[controlledMotorIndex[ii]]->value()){
                    clearHybridPoseForceModeState(false);
                    displayInfo(QString("位控轨迹不会下发：电机%1在轨迹点%2处的目标位置%3会导致电机超出软件位置上限%4").arg(controlledMotorIndex[ii]).arg(i).arg(controlledTrajForPos[i][ii])
                                .arg(axisMotorMaxVec[controlledMotorIndex[ii]]->value()).toStdString(),"error");
                    return false;
                }
                else if(controlledTrajForPos[i][ii]<axisMotorMinVec[controlledMotorIndex[ii]]->value()){
                    clearHybridPoseForceModeState(false);
                    displayInfo(QString("位控轨迹不会下发：电机%1在轨迹点%2处的目标位置%3会导致电机超出软件位置下限%4").arg(controlledMotorIndex[ii]).arg(i).arg(controlledTrajForPos[i][ii])
                                .arg(axisMotorMinVec[controlledMotorIndex[ii]]->value()).toStdString(),"error");
                    return false;
                }
            }
        }
        for(int i=0;i<controlledTrajVelForPos.size();++i){
            for(int ii=0;ii<controlledTrajVelForPos[i].size();++ii){
                if(fabs(controlledTrajVelForPos[i][ii])>axisMotorVelMaxVec[controlledMotorIndex[ii]]->value()){
                    clearHybridPoseForceModeState(false);
                    displayInfo(QString("位控错误：电机%1在轨迹点%2处的期望转速%3大于设定的安全转速%4").arg(controlledMotorIndex[ii]).arg(i).arg(controlledTrajVelForPos[i][ii])
                                .arg(axisMotorVelMaxVec[controlledMotorIndex[ii]]->value()).toStdString(),"error");
                    return false;
                }
            }
        }

        std::vector<double> allTrajVelTimeStampForPos(anchorTimeStampTrajVec[0].begin(),anchorTimeStampTrajVec[0].end());
        const std::vector<double> plannedFullTrajEndMotorTheta = allTrajForPos[allTrajForPos.size()-1];
        const std::vector<double> plannedFullTrajStartMotorTheta = allTrajForPos.front();

        activePosModeMotorIndex = controlledMotorIndex;
        activePosModeStartMotorTheta.clear();
        const std::vector<double> absMotorPosBeforePvt = hardwareInterface.getAllMotorPos();
        for(int i=0;i<controlledMotorIndex.size();++i){
            const int axisIndex = controlledMotorIndex[i];
            if(axisIndex >= 0 && axisIndex < static_cast<int>(absMotorPosBeforePvt.size())){
                activePosModeStartMotorTheta.push_back(absMotorPosBeforePvt[axisIndex]);
            }
            else{
                activePosModeStartMotorTheta.push_back(controlledTrajForPos.front()[i]);
            }
        }
        activePosModeContinuousStartMotorTheta = plannedFullTrajStartMotorTheta;

        PvtExecutionWorker::PvtCommand pvtCommand;
        pvtCommand.motorIndex = controlledMotorIndex;
        pvtCommand.motorPosTraj = controlledTrajForPos;
        pvtCommand.motorVelTraj = controlledTrajVelForPos;
        pvtCommand.timeStamp = allTrajVelTimeStampForPos;
        bool pvtStarted = false;
        bool pvtAvailable = false;
        if(pvtExecutionWorker){
            invokeOnObjectThreadWithHeartbeat(pvtExecutionWorker, [&](){
                pvtStarted = pvtExecutionWorker->startPvtTrajectory(pvtCommand);
                pvtAvailable = hardwareInterface.hasPvtTrajectory();
            }, [this](){
                refreshSafetyMonitorHeartbeat();
            });
        }
        if(!pvtStarted || !pvtAvailable){
            runtimeState.pvtCommandActive = false;
            activePosModeMotorIndex.clear();
            activePosModeStartMotorTheta.clear();
            activePosModeContinuousStartMotorTheta.clear();
            clearHybridPoseForceModeState(false);
            return false;
        }
        runtimeState.pvtCommandActive = true;
        lastTrajEndMotorTheta = plannedFullTrajEndMotorTheta;
        // 运动期间正运动学仍需要预紧零点作为绳长基准；轨迹结束或停止时再使预紧确认失效。
        updateCableHomeConfirmEnabled();
        setForceControlSelectionEnabled(false);
        qDebug() << allTrajVelTimeStampForPos.size() << allTrajVelTimeStampForPos;
        qDebug() << controlledMotorIndex;
        qDebug() << controlledTrajForPos.size() << controlledTrajForPos;
        qDebug() << controlledTrajVelForPos.size() << controlledTrajVelForPos;

        if(trajectoryCheckTimer && pvtAvailable){
            trajectoryCheckTimer->start(100);
        }
    }

    return true;
}

bool MainWindow::runGCMode(){
    if(!ensureSafetyReadyForMotion(QStringLiteral("零力拖动/阻抗调试"))){
        return false;
    }

    if(runtimeState.runMode != RunMode::RealtimeTrajectory){
        displayInfo("零力拖动/阻抗调试仅在实时运动轨迹控制模式下开放", "warning");
        return false;
    }

    const RobotStateSnapshot robotState = currentRobotState();
    if(!robotState.systemRunning){
        displayInfo("错误：零力拖动模式启动失败，请先启动电机使能","error");
        return false;
    }

    if(robotState.gcRunning){
        if(simulationWorker){
            simulationWorker->stopForceSimulation();
        }
        forceSimulationModel = nullptr;
        qDebug() << "Stop forceSimulationModel";

        runtimeState.gcRunning = false;
        runtimeState.autoExecutePoseAfterSimulation = false;
        if(!runtimeState.controlBoxPausedGC){
            runtimeState.cableHomeState = CableHomeState::Unconfirmed;
            updateCableHomeConfirmEnabled();
        }
        clearHybridPoseForceModeState(false);
        ui->mainReplay3DViewer->setEnabled(true);
        //ui->mainPosModeSend->setEnabled(true);
        optionalButtonSetText(this, "mainGCModeSwitch", QStringLiteral("启动零力拖动"));
        if(hardwareInterface.hasPvtTrajectory() &&
                (hardwareInterface.isPvtMotionRunning() || hardwareInterface.isPvtMotionPausedState())){
            setForceControlSelectionEnabled(false);
        }
        else{
            setForceControlSelectionEnabled(true);
        }

        if(ui->devUseLS->isChecked()){// edit
            hardwareInterface.setMotorChangeSpdTime(0.01);// 电机变速时间复位
        }
    }
    else{
        runtimeState.autoExecutePoseAfterSimulation = false;
        if(!ensureCableHomeReadyForMotion(QStringLiteral("零力拖动/动力学阻抗"))){
            return false;
        }
        if((robotState.positionMotionState == PositionMotionState::ExecutingPvt ||
            robotState.positionMotionState == PositionMotionState::PausedPvt) &&
                !hasForceControlAxisSelected()){
            displayInfo("错误：位置轨迹正在运行；如需同时启动力控/阻抗，请先在轨迹开始前选择力控绳索", "error");
            return false;
        }
        if(optionalComboCurrentText(this, "ForcetrajMode") == QStringLiteral("圆弧")){
            displayInfo("提示：当前无力控圆弧轨迹实现，已按直线/实时力控逻辑执行", "warning");
        }

        if(motiveLocalHandlerThread && optionalCheckBoxChecked(this, "mainGCUseRot", true)){
            resetRotHome();
        }

        std::vector<std::vector<std::vector<double>>> gcCableContactPointCoor(ui->devEndNum->value());
        std::vector<std::vector<double>> maxF(ui->devEndNum->value());
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isModeledMotorAxis(i)){// 接点局部坐标。第一层各个末端，第二层各个锚点座，第三层位置
                gcCableContactPointCoor[axisEndVec[i]->value()].push_back({axisCableEndPosXVec[i]->value(),axisCableEndPosYVec[i]->value(),axisCableEndPosZVec[i]->value(),1.0});
                maxF[axisEndVec[i]->value()].push_back(axisForceMaxVec[i]->value());
            }
        }

        std::string gcSta;
        if(ui->devGCUseEquCtrl->isChecked()){
            gcSta = "equ";
        }
        else if(ui->devGCUseObjCtrl->isChecked()){
            gcSta = "obj";
        }

        if(simulationWorker){
            simulationWorker->stopForceSimulation();
        }
        forceSimulationModel = nullptr;
        qDebug() << "Delete last forceSimulationModel";

        forceSimulationModel = new ForceSimulationModel(ui->devCtrlCycleMs->value(),gcCableContactPointCoor, ui->devUseCamForActPose->isChecked(),gcSta);
        forceSimulationModel->setPulleyRadius(buildPulleyRadius());
        clearHybridPoseForceModeState(false);
        forceSimulationModel->setCableForceMinMax(ui->devFCInitForce->value(),maxF);

        forceSimulationModel->compTor = optionalCheckBoxChecked(this, "mainCompTor", true);

        std::vector<double> tmpWeight;
        for(int i=0;i<ui->devEndNum->value();++i){
            tmpWeight.push_back(endMassVec[i]->value()*9.8);
        }
        forceSimulationModel->weight = tmpWeight;
        forceSimulationModel->optCableForce = optionalSpinBoxValue(this, "mainGCCableOptForce", 0.0);

        // 动力学
        if(optionalCheckBoxChecked(this, "devGCUseImpCtrl", true)){
            std::vector<double> endMass(ui->devEndNum->value(), 0.0);
            std::vector<double> auxiliaryMass(ui->devEndNum->value(), 0.0);
            std::vector<std::vector<double>> endNr(ui->devEndNum->value()),endImq(ui->devEndNum->value()),endFvq(ui->devEndNum->value()),endFcq(ui->devEndNum->value()),
                    endMd,endDd,endKd;
            std::vector<std::vector<std::vector<double>>> inertiaLocal,agentInertiaLocal,anchorsGlobal(ui->devEndNum->value()),contactPointsLocal(ui->devEndNum->value());
            for(int i=0;i<ui->devEndNum->value();++i){
                endMass[i] = endMassVec[i]->value();

                // 动平台惯性矩阵
                inertiaLocal.push_back(std::vector<std::vector<double>>(3,std::vector<double>(3,0.0)));// 3x3
                inertiaLocal[i][0][0] = endIxxVec[i]->value();
                inertiaLocal[i][1][1] = endIyyVec[i]->value();
                inertiaLocal[i][2][2] = endIzzVec[i]->value();
                inertiaLocal[i][0][1] = inertiaLocal[i][1][0] = endIxyVec[i]->value();
                inertiaLocal[i][1][2] = inertiaLocal[i][2][1] = endIyzVec[i]->value();
                inertiaLocal[i][0][2] = inertiaLocal[i][2][0] = endIxzVec[i]->value();

                agentInertiaLocal.push_back(std::vector<std::vector<double>>(3,std::vector<double>(3,0.0)));// 当前无悬吊目标，保持为0

                for(int ii=0;ii<ui->devAxisNum->value();++ii){
                    if(isModeledMotorAxis(ii) && axisEndVec[ii]->value() == i){
                        // 所有锚点座起始位置
                        anchorsGlobal[i].push_back({axisCableStartPosXVec[ii]->value()/1000.0,axisCableStartPosYVec[ii]->value()/1000.0,axisCableStartPosZVec[ii]->value()/1000.0});
                        // 接点局部坐标
                        contactPointsLocal[i].push_back({axisCableEndPosXVec[ii]->value()/1000.0,axisCableEndPosYVec[ii]->value()/1000.0,axisCableEndPosZVec[ii]->value()/1000.0});
                        // 转换系数(m/rad)
                        endNr[i].push_back((1.0 / motorCableAngleScale(ii)) / 1000.0);// mm/rad -> m/rad
                        // 绞盘惯性、粘性摩擦、库仑摩擦
                        endImq[i].push_back(axisImqVec[ii]->value());
                        endFvq[i].push_back(axisFvqVec[ii]->value());
                        endFcq[i].push_back(axisFcqVec[ii]->value());
                    }
                }

                endMd.push_back({endImpMdXVec[i]->value(),endImpMdYVec[i]->value(),endImpMdZVec[i]->value(),
                                 endImpMdRxVec[i]->value(),endImpMdRyVec[i]->value(),endImpMdRzVec[i]->value()});
                endDd.push_back({endImpDdXVec[i]->value(),endImpDdYVec[i]->value(),endImpDdZVec[i]->value(),
                                 endImpDdRxVec[i]->value(),endImpDdRyVec[i]->value(),endImpDdRzVec[i]->value()});
                endKd.push_back({endImpKdXVec[i]->value(),endImpKdYVec[i]->value(),endImpKdZVec[i]->value(),
                                 endImpKdRxVec[i]->value(),endImpKdRyVec[i]->value(),endImpKdRzVec[i]->value()});
            }
            forceSimulationModel->setDynPara(endMass, inertiaLocal, anchorsGlobal, contactPointsLocal, endNr,
                                             auxiliaryMass, agentInertiaLocal, endImq, endFvq, endFcq,
                                             endMd, endDd, endKd, gcSta);
        }

        connect(forceSimulationModel,&ForceSimulationModel::displayInfoSignal,this,&MainWindow::displayInfo);
        connect(forceSimulationModel,&ForceSimulationModel::gcStartSignal,this,&MainWindow::clearPreReplayData);
        connect(forceSimulationModel,&ForceSimulationModel::gcCableForceResultSignal,this,&MainWindow::gcDataProcessor);
        if(motiveLocalHandlerThread && ui->devUseCamForActPose->isChecked()){
            disconnect(motiveLocalHandlerThread,
                       &MotiveLocalHandlerThread::dataUpdateSignal,
                       this,
                       &MainWindow::emergeGCData);
            connect(motiveLocalHandlerThread,
                    &MotiveLocalHandlerThread::dataUpdateSignal,
                    this,
                    &MainWindow::emergeGCData);
        }
        else{
        }

        forceSimulationModel->useSim = optionalCheckBoxChecked(this, "mainGCSim", false);
        if(optionalCheckBoxChecked(this, "mainGCSim", false)){// 仿真。需要分别给出锚点座位置（下面给初始值，gcDataProcessor更新），悬吊目标位姿（线程内部根据a=F/m计算），期望力（下面给初始值，gcDataProcessor更新）
            connect(forceSimulationModel,&ForceSimulationModel::update3DViewerSignal,this,&MainWindow::update3DViewer);
            ui->mainFCThread->setChecked(false);
            for(int i=0;i<useFCBtnVec.size();++i){
                useFCBtnVec[i]->setChecked(false);
            }

            // 初始化
            // 锚点座位置
            if(positionSimulationHelper){
                std::vector<double> anchorMotorCurDisTemp;
                for(int i=0;i<motorCurPos.size();++i){
                    if(isModeledMotorAxis(i)){
                        anchorMotorCurDisTemp.push_back(0.0);
                    }
                }

                std::vector<std::vector<double>> curAnchorPosTemp = positionSimulationHelper->anchorMoveDis2AnchorCableCoor(anchorMotorCurDisTemp);
                std::vector<std::vector<std::vector<double>>> curAnchorPos(ui->devEndNum->value());
                int tmp = 0;
                for(int i=0;i<ui->devAxisNum->value();++i){
                    if(isModeledMotorAxis(i)){
                        curAnchorPos[axisEndVec[i]->value()].push_back(curAnchorPosTemp[tmp]);
                        tmp++;
                    }
                }

                forceSimulationModel->curAnchorPosTemp = curAnchorPos;
            }
            // 期望力
            std::vector<std::vector<double>> expPlatformForce;
            for(int i=0;i<ui->devEndNum->value();++i)
                expPlatformForce.push_back(std::vector<double>({mainGCEndXVec[i]->value(),mainGCEndYVec[i]->value(),mainGCEndZVec[i]->value(),
                                                            mainGCEndMxVec[i]->value(),mainGCEndMyVec[i]->value(),mainGCEndMzVec[i]->value()}));
            forceSimulationModel->expPlatformForce = expPlatformForce;
            // 实际力（仿真中的实际力用期望力代替，相当于绳索能瞬间到达期望力，非常理想但不可能的情况）
            std::vector<std::vector<double>> curCableForce(ui->devEndNum->value());
            for(int i=0;i<ui->devAxisNum->value();++i){
                if(axisSensorIndexVec[i]->value()>=0){
                    mainForceSensorExp[axisSensorIndexVec[i]->value()]->setValue(ui->devFCInitForce->value());// 先重置，否则会代入上一次最后的值
                    curCableForce[axisEndVec[i]->value()].push_back(mainForceSensorExp[axisSensorIndexVec[i]->value()]->value());
                }
            }
            forceSimulationModel->updateCurCableForce(curCableForce);

            forceSimulationModel->isUpdate = true;
        }
        else{
            disconnect(forceSimulationModel,&ForceSimulationModel::update3DViewerSignal,this,&MainWindow::update3DViewer);
        }

        forceSimulationModel->isFirst = true;
        if(simulationWorker){
            simulationWorker->setForceSimulationModel(forceSimulationModel);
            simulationWorker->startForceSimulation();
        }

        gcResetTime = true;
        runtimeState.gcRunning = true;
        optionalButtonSetText(this, "mainGCModeSwitch", QStringLiteral("停止零力拖动"));
        ui->mainReplay3DViewer->setEnabled(false);

        // 在执行了重力补偿后，和位控冲突，同时也不宜更新零点，或者中途取消或增加力控对象
        ui->mainResetCableForceHomeSwitch->setEnabled(false);
//        ui->mainPosModeSwitch->setEnabled(false);
        ui->mainPosModeSend->setEnabled(false);
        setForceControlSelectionEnabled(false);

        if(ui->devUseLS->isChecked()){// edit
            hardwareInterface.setMotorChangeSpdTime(0.01);// 修改电机变速时间，减少振荡，但可能会造成迟滞
        }
    }
    return true;
}

void MainWindow::emergeGCDataNoCam(std::vector<std::vector<double>> cableLen){
    std::vector<std::vector<double>> curCableForce(ui->devEndNum->value());
    if(!runtimeState.gcRunning || optionalCheckBoxChecked(this, "mainGCSim", false)){
        return;
    }

    // 无论是清华的末端动平台，还是803的机械臂，其都需要动平台/平台的位姿信息。这不是因为需要进行静力学计算，而是为了分配绳索力
    std::vector<std::vector<std::vector<double>>> curAnchorPos(ui->devEndNum->value());
    if(motorCurPos.empty()){
        qDebug() << "Error:motorCurPos is empty, using zero point instead. If this is not intended, please shutdown the programm and check connection.";
        displayInfo("Error:motorCurPos is empty, using zero point instead. If this is not intended, please shutdown the programm and check connection.","error");
        motorCurPos.resize(ui->devAxisNum->value(),0.0);
    }

    std::vector<double> anchorMotorCurDisTemp;
    if(positionSimulationHelper){
        for(int i=0;i<motorCurPos.size();++i){
            if(isModeledMotorAxis(i)){
                anchorMotorCurDisTemp.push_back(0.0);
            }
        }
        std::vector<std::vector<double>> curAnchorPosTemp = positionSimulationHelper->anchorMoveDis2AnchorCableCoor(anchorMotorCurDisTemp);
        int tmp = 0;
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isModeledMotorAxis(i)){
                curAnchorPos[axisEndVec[i]->value()].push_back(curAnchorPosTemp[tmp]);
                tmp++;
            }
        }
    }

    std::vector<std::vector<double>> expPlatformForce;
    for(int i=0;i<ui->devEndNum->value();++i)
        expPlatformForce.push_back(std::vector<double>({mainGCEndXVec[i]->value(),mainGCEndYVec[i]->value(),mainGCEndZVec[i]->value(),
                                                    mainGCEndMxVec[i]->value(),mainGCEndMyVec[i]->value(),mainGCEndMzVec[i]->value()}));

    static SYSTEMTIME systemTime;
    static double startTimeS;
    GetLocalTime(&systemTime);
    double curTimeS = (double)systemTime.wMilliseconds/1000.0 + (double)systemTime.wSecond +
            (double)systemTime.wMinute*60.0 + (double)systemTime.wHour*60.0*60.0;
    if(gcResetTime){// 重置计时器
        gcResetTime = false;
        startTimeS = curTimeS;
    }

    for(int i=0;i<ui->devAxisNum->value();++i){
        if(axisSensorIndexVec[i]->value()>=0){
            curCableForce[axisEndVec[i]->value()].push_back(mainForceSensorDataVal[axisSensorIndexVec[i]->value()]);
        }
    }

    std::vector<std::vector<double>> curPlatformForce(ui->devEndNum->value(), std::vector<double>(6, 0.0));

    if(forceSimulationModel){// 防止力补偿在计算过程中，变量被更新
        forceSimulationModel->updateCurCableForce(curCableForce);
        forceSimulationModel->updateNoCam(curAnchorPos,cableLen,expPlatformForce,curPlatformForce,curTimeS-startTimeS);
        forceSimulationModel->optCableForce = optionalSpinBoxValue(this, "mainGCCableOptForce", 0.0);
    }
}

void MainWindow::emergeGCData(std::vector<std::vector<double> > PlatformPose){
    std::vector<std::vector<double>> curCableForce(ui->devEndNum->value());
    if(!runtimeState.gcRunning || optionalCheckBoxChecked(this, "mainGCSim", false) || PlatformPose.empty()){
        return;
    }

    // 无论是清华的末端动平台，还是803的机械臂，其都需要动平台/平台的位姿信息。这不是因为需要进行静力学计算，而是为了分配绳索力
    std::vector<std::vector<std::vector<double>>> curAnchorPos(ui->devEndNum->value());
    if(motorCurPos.empty()){
        qDebug() << "Error:motorCurPos is empty, using zero point instead. If this is not intended, please shutdown the programm and check connection.";
        displayInfo("Error:motorCurPos is empty, using zero point instead. If this is not intended, please shutdown the programm and check connection.","error");
        motorCurPos.resize(ui->devAxisNum->value(),0.0);
    }

    std::vector<double> anchorMotorCurDisTemp;
    if(positionSimulationHelper){
        for(int i=0;i<motorCurPos.size();++i){
            if(isModeledMotorAxis(i)){
                anchorMotorCurDisTemp.push_back(0.0);
            }
        }
        std::vector<std::vector<double>> curAnchorPosTemp = positionSimulationHelper->anchorMoveDis2AnchorCableCoor(anchorMotorCurDisTemp);
        int tmp = 0;
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isModeledMotorAxis(i)){
                curAnchorPos[axisEndVec[i]->value()].push_back(curAnchorPosTemp[tmp]);
                tmp++;
            }
        }
    }

    std::vector<std::vector<double>> expPlatformForce;
    for(int i=0;i<ui->devEndNum->value();++i)
        expPlatformForce.push_back(std::vector<double>({mainGCEndXVec[i]->value(),mainGCEndYVec[i]->value(),mainGCEndZVec[i]->value(),
                                                    mainGCEndMxVec[i]->value(),mainGCEndMyVec[i]->value(),mainGCEndMzVec[i]->value()}));

    while(PlatformPose.size() > ui->devEndNum->value())// 当动捕获取的刚体位姿数量比设定的值大时，剔除多余的（排序靠后的）以防报错
        PlatformPose.pop_back();

    for(int i=0;i<PlatformPose.size();++i){
        for(int ii=0;ii<3;++ii){
            if(optionalCheckBoxChecked(this, "mainGCUseRot", true))
                PlatformPose[i][ii+3] -= endCurRotHome[ii];
            else
                PlatformPose[i][ii+3] = 0.0;
        }
    }

    static SYSTEMTIME systemTime;
    static double startTimeS;
    GetLocalTime(&systemTime);
    double curTimeS = (double)systemTime.wMilliseconds/1000.0 + (double)systemTime.wSecond +
            (double)systemTime.wMinute*60.0 + (double)systemTime.wHour*60.0*60.0;
    if(gcResetTime){// 重置计时器
        gcResetTime = false;
        startTimeS = curTimeS;
    }

    for(int i=0;i<ui->devAxisNum->value();++i){
        if(axisSensorIndexVec[i]->value()>=0){
            curCableForce[axisEndVec[i]->value()].push_back(mainForceSensorDataVal[axisSensorIndexVec[i]->value()]);
        }
    }

    std::vector<std::vector<double>> curPlatformForce(ui->devEndNum->value(), std::vector<double>(6, 0.0));

    if(forceSimulationModel){// 防止力补偿在计算过程中，变量被更新
        forceSimulationModel->updateCurCableForce(curCableForce);
        forceSimulationModel->update(curAnchorPos,PlatformPose,expPlatformForce,curPlatformForce,curTimeS-startTimeS);
        forceSimulationModel->optCableForce = optionalSpinBoxValue(this, "mainGCCableOptForce", 0.0);
    }
}

void MainWindow::gcDataProcessor(std::vector<double> gcCableForce, std::vector<std::vector<double>> PlatformPose, std::vector<std::vector<double>> PlatformForce){
    Q_UNUSED(PlatformForce);
    const bool useForwardKinematicsPose =
            optionalCheckBoxChecked(this, "mainGCSim", false) || shouldUseForwardKinematicsFallback(500);
    if(useForwardKinematicsPose && !PlatformPose.empty()){
        cacheForwardKinematicsPose(PlatformPose);
    }
    if(useForwardKinematicsPose){
        updateVisualizationFromPlatformPose(PlatformPose);
    }
    if(forceSimulationModel && !forceSimulationModel->optErr){// 当出现错误时，保持为上一次的计算结果。只有正常计算才会更新输出
        const std::vector<double> expectedForce = mapGcCableForceToSensorExpected(gcCableForce);
        mainForceSensorExpVal = expectedForce;
        if(controlWorker){
            controlWorker->setExternalExpectedForce(expectedForce);
        }
    }
    else{// 遇到错误时，进行下一次输入量更新
        if(forceSimulationModel){
            qDebug() << "GC Error:" << forceSimulationModel->optErr;
        }
    }

    if(optionalCheckBoxChecked(this, "mainGCSim", false) && forceSimulationModel){
        // 锚点座位置
        if(positionSimulationHelper){
            std::vector<double> anchorMotorCurDisTemp;
            for(int i=0;i<motorCurPos.size();++i){
                if(isModeledMotorAxis(i)){
                    anchorMotorCurDisTemp.push_back(0.0);
                }
            }
            std::vector<std::vector<double>> curAnchorPosTemp = positionSimulationHelper->anchorMoveDis2AnchorCableCoor(anchorMotorCurDisTemp);
            std::vector<std::vector<std::vector<double>>> curAnchorPos(ui->devEndNum->value());
            int tmp = 0;
            for(int i=0;i<ui->devAxisNum->value();++i){
                if(isModeledMotorAxis(i)){
                    curAnchorPos[axisEndVec[i]->value()].push_back(curAnchorPosTemp[tmp]);
                    tmp++;
                }
            }
            forceSimulationModel->curAnchorPosTemp = curAnchorPos;
        }
        // 期望力
        std::vector<std::vector<double>> expPlatformForce;
        for(int i=0;i<ui->devEndNum->value();++i)
            expPlatformForce.push_back(std::vector<double>({mainGCEndXVec[i]->value(),mainGCEndYVec[i]->value(),mainGCEndZVec[i]->value(),
                                                        mainGCEndMxVec[i]->value(),mainGCEndMyVec[i]->value(),mainGCEndMzVec[i]->value()}));
        forceSimulationModel->expPlatformForce = expPlatformForce;
        // 实际力（仿真中的实际力用期望力代替，相当于绳索能瞬间到达期望力，非常理想但不可能的情况）
        std::vector<std::vector<double>> curCableForce(ui->devEndNum->value());
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(axisSensorIndexVec[i]->value()>=0){
                curCableForce[axisEndVec[i]->value()].push_back(mainForceSensorExp[axisSensorIndexVec[i]->value()]->value());
            }
        }
        forceSimulationModel->updateCurCableForce(curCableForce);

        forceSimulationModel->optCableForce = optionalSpinBoxValue(this, "mainGCCableOptForce", 0.0);

        forceSimulationModel->isUpdate = true;
    }
}

void MainWindow::endChange(int areaNum){
//    if(areaNum != 1 && areaNum != 2){
//        displayInfo("错误：接点区域数量只支持 1 或 2 ","error");
//        return;
//    }
    Q_UNUSED(areaNum);

    std::vector<std::vector<double>> initDataA,initDataB;
    std::vector<std::vector<double>> anchorCableCoorHomeInitDataA,anchorCableCoorHomeInitDataB;

    if(ui->devUseG3->isChecked()){
        // 末端坐标系下绳索结点
        // 若为机械臂构造：此处的接线框架坐标系，是指在机械臂伸直时（此时下述平台坐标系与世界坐标系平行），以平台圆心作为原点，从基座到末端方向的平台轴方向为x正方向，向右（末端到基座的视角）为y正方向，向上为z正方向的坐标系。
        // 若为框架构造：此处的接线框架坐标系，是指与世界坐标系平行，以框架中心为原点的坐标系 ***

        // G302室内的0323动平台坐标
        axisCableEndPosXVec[0]->setValue(-482.133);axisCableEndPosYVec[0]->setValue(156.7446);axisCableEndPosZVec[0]->setValue(-253.582);
        axisCableEndPosXVec[1]->setValue(-298.034);axisCableEndPosYVec[1]->setValue(410.3261);axisCableEndPosZVec[1]->setValue(253.5815);
        axisCableEndPosXVec[2]->setValue(482.1332);axisCableEndPosYVec[2]->setValue(156.7446);axisCableEndPosZVec[2]->setValue(-253.582);
        axisCableEndPosXVec[3]->setValue(298.0335);axisCableEndPosYVec[3]->setValue(410.3261);axisCableEndPosZVec[3]->setValue(253.5815);
        axisCableEndPosXVec[4]->setValue(298.0335);axisCableEndPosYVec[4]->setValue(-410.326);axisCableEndPosZVec[4]->setValue(-253.582);
        axisCableEndPosXVec[5]->setValue(482.1332);axisCableEndPosYVec[5]->setValue(-156.745);axisCableEndPosZVec[5]->setValue(253.5815);
        axisCableEndPosXVec[6]->setValue(-298.034);axisCableEndPosYVec[6]->setValue(-410.326);axisCableEndPosZVec[6]->setValue(-253.582);
        axisCableEndPosXVec[7]->setValue(-482.133);axisCableEndPosYVec[7]->setValue(-156.745);axisCableEndPosZVec[7]->setValue(253.5815);


        // 滑轮座出绳点（开环误差来源之一，需要加滑轮误差补偿算法）
        // 填写顺序应当与参数设置标签页中的绳索接点位置的填写顺序对应
//        anchorCableCoorHomeInitDataA = {{1055.6,-935.5,2000.0},{1055.6,935.5,2000.0},{-1055.6,935.5,2000.0},{-1055.6,-935.5,2000.0},
//                                        {801.0,-965.0,2195.8},{801.0,965.0,2164.6},{-801.0,965.0,2195.8},{-801.0,-965.0,2164.6}};

        // G302 当前零点出绳点坐标，按 B1~B8 对应绳索电机 1~8 填写
        axisCableStartPosXVec[0]->setValue(-2955.55); axisCableStartPosYVec[0]->setValue(2614.386); axisCableStartPosZVec[0]->setValue(2478.5);
        axisCableStartPosXVec[1]->setValue(-2740.67); axisCableStartPosYVec[1]->setValue(2953.898); axisCableStartPosZVec[1]->setValue(20.01662);
        axisCableStartPosXVec[2]->setValue(2957.778); axisCableStartPosYVec[2]->setValue(2619.487); axisCableStartPosZVec[2]->setValue(2478.5);
        axisCableStartPosXVec[3]->setValue(2741.135); axisCableStartPosYVec[3]->setValue(2954.938); axisCableStartPosZVec[3]->setValue(22.82642);
        axisCableStartPosXVec[4]->setValue(2946.743); axisCableStartPosYVec[4]->setValue(-2615.77); axisCableStartPosZVec[4]->setValue(2478.5);
        axisCableStartPosXVec[5]->setValue(2742.466); axisCableStartPosYVec[5]->setValue(-2955.33); axisCableStartPosZVec[5]->setValue(19.43142);
        axisCableStartPosXVec[6]->setValue(-2963.8);  axisCableStartPosYVec[6]->setValue(-2610.35); axisCableStartPosZVec[6]->setValue(2478.5);
        axisCableStartPosXVec[7]->setValue(-2742.94); axisCableStartPosYVec[7]->setValue(-2953.44); axisCableStartPosZVec[7]->setValue(23.57075);


        endCurRotHome = std::vector<double>(3, 0.0);
    }
}

bool MainWindow::ensurePoseKinematicsHelpersReady(bool forceRebuild)
{
    if(!forceRebuild && positionSimulationHelper && forwardKinematicsHelper){
        return true;
    }

    refreshAxisRuntimeCounts();
    if(axisCableNum <= 0){
        return false;
    }

    std::vector<std::vector<std::vector<double>>> cableContactPointPos = buildCableContactPointPos();
    static std::vector<std::vector<double>> anchorCableCoorHome;
    anchorCableCoorHome = buildFixedAnchorHome();
    const std::vector<double> cableMotorCof = buildCableMotorCof();
    if(anchorCableCoorHome.empty() || cableMotorCof.empty()){
        return false;
    }

    std::vector<double> anchorMotorCof(anchorCableCoorHome.size(), 0.0);
    std::vector<double> anchorStepTimeMaxDis(anchorCableCoorHome.size(), 0.0);
    lastTrajEndAnchorCoor = anchorCableCoorHome;

    if(positionSimulationHelper){
        delete positionSimulationHelper;
        positionSimulationHelper = nullptr;
    }
    if(forwardKinematicsHelper){
        delete forwardKinematicsHelper;
        forwardKinematicsHelper = nullptr;
    }
    lastForwardKinematicsPose.clear();
    lastForwardKinematicsPoseTimestampMs = -1;

    positionSimulationHelper = new PositionSimulationModel(ui->devCtrlCycleMs->value(),
                                                           ui->devFrameL->value(),
                                                           ui->devFrameW->value(),
                                                           anchorMotorCof,
                                                           cableMotorCof,
                                                           cableContactPointPos,
                                                           &anchorCableCoorHome,
                                                           anchorStepTimeMaxDis,
                                                           useTrajFile,
                                                           buildPulleyRadius());
    positionSimulationHelper->frameCalMethod = "fixed";

    forwardKinematicsHelper = new ForceSimulationModel(ui->devCtrlCycleMs->value(),
                                                       cableContactPointPos,
                                                       false,
                                                       "equ");
    forwardKinematicsHelper->setPulleyRadius(buildPulleyRadius());
    return positionSimulationHelper && forwardKinematicsHelper;
}

void MainWindow::processNoCamCableLengthFromMotorSnapshot(){
    // 计算初始绳长，并记录绳索电机预紧后的转角。
    const bool useForwardKinematicsPose = shouldUseForwardKinematicsFallback(500);
    const bool needCableHomeBaseline = homeCableVec.empty() ||
            homeCableLen.empty() ||
            cableHomePos.empty();
    if(runtimeState.cableHomeState == CableHomeState::Confirmed &&
            needCableHomeBaseline &&
            runtimeState.systemRunning){
        if(!ensurePoseKinematicsHelpersReady()){
            return;
        }
        cableHomePos.clear();
        homeCableVec.assign(ui->devEndNum->value(), {});
        homeCableLen.assign(ui->devEndNum->value(), {});
        std::vector<double> anchorMotorCurDisTemp;
        std::vector<std::vector<std::vector<double>>> curAnchorPos(ui->devEndNum->value());
        for(int i=0; i<static_cast<int>(motorCurPos.size()); ++i){
            if(isModeledMotorAxis(i)){
                anchorMotorCurDisTemp.push_back(0.0);
                cableHomePos.push_back(motorCurPos[i]);
            }
        }

        std::vector<std::vector<double>> curAnchorPosTemp = positionSimulationHelper->anchorMoveDis2AnchorCableCoor(anchorMotorCurDisTemp);
        int tmp = 0;
        for(int i=0; i<ui->devAxisNum->value(); ++i){
            if(isModeledMotorAxis(i)){
                curAnchorPos[axisEndVec[i]->value()].push_back(curAnchorPosTemp[tmp]);
                tmp++;
            }
        }

        std::vector<std::vector<std::vector<std::vector<double>>>> endCableContactPosAfterRot(ui->devEndNum->value());
        std::vector<std::vector<QDoubleSpinBox*>> devEndHomeBox = {{ui->devEndHomeX,ui->devEndHomeY,ui->devEndHomeZ,ui->devEndHomeRx,ui->devEndHomeRy,ui->devEndHomeRz}};
        std::vector<std::vector<std::vector<double>>> trajR(ui->devEndNum->value());
        for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
            std::vector<std::vector<double>> Rx = {{1.0,0.0,0.0},{0.0,cos(devEndHomeBox[endIndex][3]->value()),-sin(devEndHomeBox[endIndex][3]->value())},{0.0,sin(devEndHomeBox[endIndex][3]->value()),cos(devEndHomeBox[endIndex][3]->value())}};
            std::vector<std::vector<double>> Ry = {{cos(devEndHomeBox[endIndex][4]->value()),0.0,sin(devEndHomeBox[endIndex][4]->value())},{0.0,1.0,0.0},{-sin(devEndHomeBox[endIndex][4]->value()),0.0,cos(devEndHomeBox[endIndex][4]->value())}};
            std::vector<std::vector<double>> Rz = {{cos(devEndHomeBox[endIndex][5]->value()),-sin(devEndHomeBox[endIndex][5]->value()),0.0},{sin(devEndHomeBox[endIndex][5]->value()),cos(devEndHomeBox[endIndex][5]->value()),0.0},{0.0,0.0,1.0}};
            trajR[endIndex] = MatrixFun::matrix_mul(MatrixFun::matrix_mul(Rz,Ry),Rx);
            homeCableVec[endIndex].resize(eachEndCableNum[endIndex]);
        }
        for(int i=0; i<ui->devAxisNum->value(); ++i){
            if(isModeledMotorAxis(i)){
                endCableContactPosAfterRot[axisEndVec[i]->value()].push_back(
                            MatrixFun::matrix_mul(trajR[axisEndVec[i]->value()],{{axisCableEndPosXVec[i]->value()},
                                                                                 {axisCableEndPosYVec[i]->value()},
                                                                                 {axisCableEndPosZVec[i]->value()}}));
            }
        }
        const double pulleyRadius = buildPulleyRadius();
        for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
            for(int i=0; i<eachEndCableNum[endIndex]; ++i){
                std::vector<double> globalContactPoint(3, 0.0);
                for(int ii=0; ii<3; ++ii){
                    globalContactPoint[ii] = devEndHomeBox[endIndex][ii]->value() + endCableContactPosAfterRot[endIndex][i][ii][0];
                }
                const MatrixFun::CableLengthResult cableGeometry =
                        MatrixFun::cableLengthCalculate(globalContactPoint, curAnchorPos[endIndex][i], pulleyRadius);
                for(int ii=0; ii<3; ++ii){
                    homeCableVec[endIndex][i].push_back(cableGeometry.tangentPoint[ii] - globalContactPoint[ii]);
                }
                homeCableLen[endIndex].push_back(cableGeometry.idealLength);
            }
        }
    }

    std::vector<std::vector<double>> cableLen;
    if(runtimeState.gcRunning && useForwardKinematicsPose &&
            buildCurrentCableLengths(cableLen)){
        emergeGCDataNoCam(cableLen);
    }
}

bool MainWindow::buildCurrentCableDisplacements(std::vector<std::vector<double>>& cableDisplacement) const
{
    cableDisplacement.assign(ui->devEndNum->value(), {});
    if(motorCurPos.empty()){
        return false;
    }

    for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex) || axisIndex >= static_cast<int>(motorCurPos.size())){
            continue;
        }

        const int endIndex = axisEndVec[axisIndex]->value();
        if(endIndex < 0 || endIndex >= ui->devEndNum->value()){
            return false;
        }
        cableDisplacement[endIndex].push_back(motorCurPos[axisIndex]);
    }

    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        if(cableDisplacement[endIndex].empty()){
            return false;
        }
    }
    return true;
}

bool MainWindow::buildCurrentCableLengths(std::vector<std::vector<double>>& cableLen) const
{
    cableLen.assign(ui->devEndNum->value(), {});
    if(runtimeState.cableHomeState != CableHomeState::Confirmed ||
            homeCableLen.empty() ||
            cableHomePos.empty() ||
            motorCurPos.empty()){
        return false;
    }

    std::vector<int> channelOffset(ui->devEndNum->value(), 0);
    int modeledCableIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        if(axisIndex >= static_cast<int>(motorCurPos.size()) ||
                modeledCableIndex >= static_cast<int>(cableHomePos.size())){
            return false;
        }

        const int endIndex = axisEndVec[axisIndex]->value();
        if(endIndex < 0 || endIndex >= ui->devEndNum->value()){
            return false;
        }
        const int cableIndex = channelOffset[endIndex];
        if(endIndex >= static_cast<int>(homeCableLen.size()) ||
                cableIndex >= static_cast<int>(homeCableLen[endIndex].size())){
            return false;
        }

        cableLen[endIndex].push_back(motorCurPos[axisIndex] - cableHomePos[modeledCableIndex] +
                                     homeCableLen[endIndex][cableIndex]);
        channelOffset[endIndex]++;
        modeledCableIndex++;
    }

    for(int endIndex=0; endIndex<ui->devEndNum->value(); ++endIndex){
        if(cableLen[endIndex].empty()){
            return false;
        }
    }
    return true;
}

bool MainWindow::buildCurrentCableLengthSnapshot(std::vector<std::vector<double>>& cableLen) const
{
    return buildCurrentCableLengths(cableLen);
}

void MainWindow::updateForwardKinematicsPoseFromMotorSnapshot()
{
    if(!runtimeState.systemRunning){
        return;
    }
    if(!ensurePoseKinematicsHelpersReady()){
        return;
    }

    if(isCurrentMotorSnapshotAtCableHome()){
        cacheForwardKinematicsPose(configuredCableHomePlatformPose());
        return;
    }

    std::vector<std::vector<double>> cableLen;
    if(!buildCurrentCableLengths(cableLen)){
        return;
    }

    std::vector<double> anchorMotorCurDisTemp;
    for(int axisIndex=0; axisIndex<static_cast<int>(motorCurPos.size()); ++axisIndex){
        if(isModeledMotorAxis(axisIndex)){
            anchorMotorCurDisTemp.push_back(0.0);
        }
    }

    const std::vector<std::vector<double>> curAnchorPosTemp =
            positionSimulationHelper->anchorMoveDis2AnchorCableCoor(anchorMotorCurDisTemp);
    std::vector<std::vector<std::vector<double>>> curAnchorPos(ui->devEndNum->value());
    int channelIndex = 0;
    for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }
        const int endIndex = axisEndVec[axisIndex]->value();
        if(endIndex < 0 || endIndex >= ui->devEndNum->value() ||
                channelIndex >= static_cast<int>(curAnchorPosTemp.size())){
            return;
        }
        curAnchorPos[endIndex].push_back(curAnchorPosTemp[channelIndex]);
        ++channelIndex;
    }

    const std::vector<std::vector<double>> fkPose =
            forwardKinematicsHelper->solvePoseFromCableLengths(curAnchorPos,
                                                               cableLen,
                                                               optionalCheckBoxChecked(this, "mainGCUseRot", true));
    if(!fkPose.empty() && fkPose.front().size() >= 6){
        cacheForwardKinematicsPose(fkPose);
    }
}

bool MainWindow::refreshForwardKinematicsPoseFromCurrentMotorPosition()
{
    if(ui->devUseCamForActPose->isChecked()){
        return hasRecentMotivePose(500);
    }

    if(runtimeState.cableHomeState != CableHomeState::Confirmed){
        return false;
    }

    if(!ensurePoseKinematicsHelpersReady()){
        return false;
    }

    bool hasMotorPosition = false;
    if(ui->devUseLS->isChecked() && hardwareInterface.isLSConnected()){
        const std::vector<double> absPos = hardwareInterface.getAllMotorPos();
        if(!absPos.empty()){
            applyMotorPositionSnapshot(absPos);
            hasMotorPosition = true;
        }
    }

    if(!hasMotorPosition && controlWorker){
        const ControlWorker::Snapshot snapshot = controlWorker->latestSnapshot();
        if(!snapshot.motorAbsPos.empty()){
            applyMotorPositionSnapshot(snapshot.motorAbsPos, snapshot.motorRelRawPos);
            hasMotorPosition = true;
        }
    }

    if(!hasMotorPosition){
        hasMotorPosition = !motorCurPos.empty();
    }

    if(!hasMotorPosition){
        return false;
    }

    processNoCamCableLengthFromMotorSnapshot();
    updateForwardKinematicsPoseFromMotorSnapshot();

    std::vector<double> pose;
    return currentForwardKinematicsEndPose(pose, 500);
}

void MainWindow::stopPositionThread(){
    if(positionThread && positionThread->isRunning()){
        positionThread->quit();
        if(!positionThread->wait(1000)){
            positionThread->terminate();
            positionThread->wait(500);
        }
    }
}

void MainWindow::replay3DViewer(){
    if(!use3DViewer)
        return;
    for(int i=0;i<preTargetPos.size();++i){
        isReplaying = true;
        ui->mainPosModeSwitch->setEnabled(false);
        optionalButtonSetEnabled(this, "mainGCModeSwitch", false);
        ui->mainReplay3DViewer->setEnabled(false);
        ui->mainPosModeSend->setEnabled(false);
        update3DViewer(preTargetPos[i], preTrajPos[i], preAnchorPos[i], preCablePos[i]);
        delay(100);
    }
    isReplaying = false;
    ui->mainPosModeSwitch->setEnabled(true);
    optionalButtonSetEnabled(this, "mainGCModeSwitch", true);
    ui->mainReplay3DViewer->setEnabled(true);
    ui->mainPosModeSend->setEnabled(true);
}

void MainWindow::clearPreReplayData(){
    preTargetPos.resize(0);
    preTrajPos.resize(0);
    preAnchorPos.resize(0);
    preCablePos.resize(0);
    if(seriesTarget){
        seriesTarget->dataProxy()->resetArray(new QScatterDataArray());
    }
    if(seriesTraj){
        seriesTraj->dataProxy()->resetArray(new QScatterDataArray());
    }
    if(seriesAnchor){
        seriesAnchor->dataProxy()->resetArray(new QScatterDataArray());
    }
    if(seriesCable){
        seriesCable->dataProxy()->resetArray(new QScatterDataArray());
    }
    if(seriesAnchor2Cable){
        seriesAnchor2Cable->dataProxy()->resetArray(new QScatterDataArray());
    }
    is3DViewerReadyForNextInput = true;
    clearVisualizationData();
}

// 该函数在devUseG3的状态改变时被调用，用于根据G3的使用情况自动调整相关参数，现在为默认参数设置
void MainWindow::paraChange(){
    if(ui->devUseG3->isChecked()){
        ui->devUseLS->setEnabled(true);
        ui->devUseLS->setChecked(true);

        ui->devCamTypeBox->setCurrentText("nokov");
        ui->devEndNum->setValue(1);
        ui->devFSPortNum->setValue(2);
        ui->devMonitorCycleMs->setValue(10);
        ui->MonitorCOMnum->setCurrentText("COM12");
        ui->MonitorCOMbaudrate->setCurrentText("115200");

        ui->devFSDataLen->setValue(2);// SBT908E中的PDO长度为int32
        // 张力通道顺序：1004:0x6000:1-4 -> 绳索1-4，1011:0x6000:1-4 -> 绳索5-8。
        std::vector<double> devForceSensorCofVec = {0.1, 0.1, 0.1, 0.1,
                                                    0.1, 0.1, 0.1, 0.1};

        // 用于anchorMoveDis2AnchorCableCoor判断位置。通常包络锚点座出绳点所处位置即可
        ui->devFrameL->setValue(6370.0);// x轴平行的边，单位 mm
        ui->devFrameW->setValue(6370.0);// y轴平行的边，单位 mm
        ui->devFrameH->setValue(5520.0);// z轴高度，单位 mm
        ui->devPulleyRadius->setValue(18.5);// 滑轮半径(mm)

        ui->devAxisNum->setValue(12);// 8根绳索电机 + 4个直线模组电机
        ui->devMotorFeedbackIsTheta->setChecked(false);
        ui->devMotorFeedbackIsRd->setChecked(true);

        ui->axisIsForceSensorSigned->setChecked(false);
        ui->axisIsStaticSensorHome->setChecked(false);

        // 根据最终接线顺序修正
        // 由于所有传感器当前都是绳力越大，数字量越小，所以负号
        // std::vector<double> devForceSensorCofVec = {-0.007034782,-0.006645996,
        //                                             -0.006670229,-0.007030674,
        //                                             -0.0067061,-0.007107018,
        //                                             -0.007043724,-0.006928195};//
        // 逻辑轴顺序：绳索电机1~8、直线模组1~4；控制卡轴号按 Motion 轴映射后的0-based顺序填写。
        const std::vector<int> defaultHardwareAxisNo = {0, 1, 5, 4, 6, 7, 11, 10, 2, 3, 8, 9};
        const std::vector<int> defaultSlaveId = {1001, 1002, 1007, 1006, 1008, 1009, 1014, 1013, 1003, 1005, 1010, 1012};
        const int modeledCableAxisCount = 8;
        const double defaultCableMotorPosLimit = 14.0;
        const double defaultCableMotorVelLimit = kDefaultMotorVelLimitRevPerSec;
        const double defaultOtherMotorPosLimit = 14.0;
        const double defaultOtherMotorVelLimit = kDefaultMotorVelLimitRevPerSec;

        for(int i=0;i<axisTypeVec.size();++i){
            axisForceMaxVec[i]->setValue(kDefaultAxisForceMaxN);
            if(i<ui->devAxisNum->value()){
                axisTypeVec[i]->setCurrentText(i < modeledCableAxisCount ?
                                                   motorAxisText() :
                                                   linearMotorAxisText());
                axisMotorHardwareAxisVec[i]->setValue(defaultHardwareAxisNo[i]);
                axisMotorSlaveIdVec[i]->setValue(defaultSlaveId[i]);
                axisMotorVelMaxVec[i]->setValue(
                            i < modeledCableAxisCount ? defaultCableMotorVelLimit
                                                      : defaultOtherMotorVelLimit);

                if(i < modeledCableAxisCount){
                    axisEndVec[i]->setValue(0);
                    axisSensorIndexVec[i]->setValue(i);
                    axisSensorCofVec[i]->setValue(devForceSensorCofVec[i]);
                }
                else{
                    axisEndVec[i]->setValue(-1);
                    axisSensorIndexVec[i]->setValue(-1);
                    axisSensorCofVec[i]->setValue(0.0);
                }
            }
            else{
                axisTypeVec[i]->setCurrentIndex(0);
                axisEndVec[i]->setValue(-1);
                axisSensorIndexVec[i]->setValue(-1);
                axisMotorHardwareAxisVec[i]->setValue(i);
                axisMotorSlaveIdVec[i]->setValue(0);
                axisSensorCofVec[i]->setValue(0.0);
            }
        }

        for(int i=0;i<axisTypeVec.size();++i){
            if(isModeledMotorAxis(i)){
                axisMotorCofVec[i]->setValue(43.0);// 卷筒半径43mm
            }
            else if(isMotorAxis(i)){
                axisMotorCofVec[i]->setValue(0.0);
            }
            axisIsPos2NegVec[i]->setChecked(false);
        }

        for(int i=0;i<axisTypeVec.size();++i){
            if(i < modeledCableAxisCount){
                axisMotorMaxVec[i]->setValue(defaultCableMotorPosLimit);
                axisMotorMinVec[i]->setValue(-defaultCableMotorPosLimit);// 度
            }
            else{
                axisMotorMaxVec[i]->setValue(defaultOtherMotorPosLimit);
                axisMotorMinVec[i]->setValue(-defaultOtherMotorPosLimit);// 度
            }
        }

        ui->devUseCamForActPose->setChecked(false);
        ui->devEndHomeX->setValue(kGlobalInitialPosePxMm);
        ui->devEndHomeY->setValue(kGlobalInitialPosePyMm);
        ui->devEndHomeZ->setValue(kGlobalInitialPosePzMm);
        ui->devEndHomeRx->setValue(kGlobalInitialPoseRxRad);
        ui->devEndHomeRy->setValue(kGlobalInitialPoseRyRad);
        ui->devEndHomeRz->setValue(kGlobalInitialPoseRzRad);

        ui->mainResetMotiveFit->setEnabled(true);
        ui->mainPosModeTargetStartPx->setValue(kGlobalInitialPosePxMm);
        ui->mainPosModeTargetStartPy->setValue(kGlobalInitialPosePyMm);
        ui->mainPosModeTargetStartPz->setValue(kGlobalInitialPosePzMm);
        ui->mainPosModeTargetStartRx->setValue(kGlobalInitialPoseRxRad);
        ui->mainPosModeTargetStartRy->setValue(kGlobalInitialPoseRyRad);
        ui->mainPosModeTargetStartRz->setValue(kGlobalInitialPoseRzRad);
        ui->mainPosModeTargetEndPx->setValue(0);// 200
        ui->mainPosModeTargetEndPy->setValue(0);// 200
        ui->mainPosModeTargetEndPz->setValue(655);// 600
        ui->mainPosModeTargetEndRz->setValue(0.0);// 10.0*PI/180.0
        ui->mainPosModeTargetRunTime->setValue(20.0);

        // 这里假设没有悬吊目标，即微重力补偿对象即为动平台
        // endMassVec[0]->setValue(2.2);// ***
        // endIxxVec[0]->setValue(2705.4*1e-6);
        // endIyyVec[0]->setValue(2705.4*1e-6);
        // endIzzVec[0]->setValue(2183.9*1e-6);

        // 室内0305动平台的质量***
        // endMassVec[0]->setValue(3.365);
        // ui->mainGCEndZ->setValue(endMassVec[0]->value()*9.8);
        // endIxxVec[0]->setValue(26571.939*1e-6); endIyyVec[0]->setValue(29099.969*1e-6); endIzzVec[0]->setValue(28289.252*1e-6);
        // endIxyVec[0]->setValue(-8.368*1e-6);    endIyzVec[0]->setValue(-13.403*1e-6);   endIxzVec[0]->setValue(16.134*1e-6);

        // 室内0323动平台的质量***
        endMassVec[0]->setValue(2.594);
        optionalSpinBoxSetValue(this, "mainGCEndZ", endMassVec[0]->value()*9.8);
        endIxxVec[0]->setValue(14589.685*1e-6); endIyyVec[0]->setValue(16873.391*1e-6); endIzzVec[0]->setValue(18983.031*1e-6);
        endIxyVec[0]->setValue(-8.551*1e-6);   endIyzVec[0]->setValue(-34.619*1e-6);   endIxzVec[0]->setValue(33.140*1e-6);

        if(testDyn)
            endMassVec[0]->setValue(100.0);
        for(int i=0;i<axisTypeVec.size();++i){
            if(i<8){
                axisImqVec[i]->setValue(0.0);
                axisFvqVec[i]->setValue(0.0);
                axisFcqVec[i]->setValue(0.0);
            }
        }

        // 此处为阻抗参数的初始值，后续可以改成于ui中dev可设置的量
        endImpMdXVec[0]->setValue(50);endImpMdYVec[0]->setValue(50);endImpMdZVec[0]->setValue(50);// 5000，1000，500
        endImpMdRxVec[0]->setValue(4000);endImpMdRyVec[0]->setValue(4000);endImpMdRzVec[0]->setValue(4000);// 25000，8000
        endImpDdXVec[0]->setValue(1500);endImpDdYVec[0]->setValue(1500);endImpDdZVec[0]->setValue(1500);// 9000，15000，4500
        endImpDdRxVec[0]->setValue(3000);endImpDdRyVec[0]->setValue(3000);endImpDdRzVec[0]->setValue(3000);// 1800
        endImpKdXVec[0]->setValue(10000);endImpKdYVec[0]->setValue(10000);endImpKdZVec[0]->setValue(10000);// 4000，3000，6000
        endImpKdRxVec[0]->setValue(1000);endImpKdRyVec[0]->setValue(1000);endImpKdRzVec[0]->setValue(1000);//800
        // 此处为力控制PID参数的初始值，后续可以改成于ui中dev可设置的量
        ui->devUsePID->setChecked(true);
        ui->devFCKp->setValue(1.5);// 0.004 0.005 0.09 0.015
        ui->devFCKi->setValue(0.01);// 0.05
        ui->devFCKd->setValue(0.05);// 0.0004 0.005 0.015 0.005
        ui->devFCInitForce->setValue(kDefaultCablePretensionN);
        ui->devFCDeadbandPercent->setValue(10.0);

        for(int i=0;i<mainForceSensorExp.size();++i){
            mainForceSensorExp[i]->setValue(ui->devFCInitForce->value());
        }
        optionalCheckBoxSetChecked(this, "mainCompTor", true);
        optionalCheckBoxSetChecked(this, "mainGCUseRot", true);

    }
    endChange(ui->devEndNum->value());

    ui->devGCUseEquCtrl->setChecked(true);
//    updatePara();
    //emit(ui->devEndNum->valueChanged(ui->devEndNum->value()));// 也更新一下锚点座数据
}

void MainWindow::updatePara(){
    if(runtimeState.systemRunning){
        refreshAxisRuntimeCounts();
        resetFCPIDPara();
        hardwareInterface.threadCtrlCycleMs = (double)ui->devCtrlCycleMs->value();
        if(motiveLocalHandlerThread){
            motiveLocalHandlerThread->ctrlCycleMs = (double)(ui->devCtrlCycleMs->value());
        }
        updateControlWorkerConfig();
        updateSafetyMonitorConfig();
        refreshSafetyMonitorHeartbeat();
        displayInfo("系统运行中已跳过完整参数重建和雷赛硬件参数下发；运行参数已同步。需要修改轴号、脉冲当量、动捕或模块数量时，请先断连后再更新参数。", "warning");
        return;
    }

    refreshAxisRuntimeCounts();
    forceSensorCurHome = std::vector<double>(axisForceSensorNum,0.0);
    resetFCPIDPara();

    // 动捕
    if(ui->devCamTypeBox->currentText().toStdString() == "/")
        useCam = false;
    else
        useCam = true;

    if(mlhThread){
        mlhThread->quit();
        mlhThread->wait();
        delete mlhThread;
        mlhThread = nullptr;
    }
    if(motiveLocalHandlerThread){
        delete motiveLocalHandlerThread;
        motiveLocalHandlerThread = nullptr;
    }

    if(useCam){
        ui->devUpdateParaBtn->setEnabled(false);
        delay(100);
        motiveLocalHandlerThread = new MotiveLocalHandlerThread(ui->devCtrlCycleMs->value());
        if(motiveLocalHandlerThread->isInit){
            mlhThread = new QThread(this);
            motiveLocalHandlerThread->moveToThread(mlhThread);
            connect(mlhThread, &QThread::started, motiveLocalHandlerThread, &MotiveLocalHandlerThread::startTimer);
            connect(mlhThread, &QThread::finished, motiveLocalHandlerThread, &MotiveLocalHandlerThread::stopTimer);

            disconnect(ui->mainResetMotiveFit,
                       &IntBtn::sendInt,
                       this,
                       &MainWindow::resetMotiveFit);
            connect(ui->mainResetMotiveFit,
                    &IntBtn::sendInt,
                    this,
                    &MainWindow::resetMotiveFit);
            disconnect(motiveLocalHandlerThread,
                       &MotiveLocalHandlerThread::dataUpdateSignal,
                       this,
                       &MainWindow::handleVisualizationRigidPose);
            connect(motiveLocalHandlerThread,
                    &MotiveLocalHandlerThread::dataUpdateSignal,
                    this,
                    &MainWindow::handleVisualizationRigidPose);
        }
        else{
            displayInfo("Failed to connect to the camera server, please restart the system and check connection.","error");
            delete motiveLocalHandlerThread;
            motiveLocalHandlerThread = nullptr;
        }
        ui->devUpdateParaBtn->setEnabled(true);
    }

    // 硬件参数
    if(ui->devUseLS->isChecked()){
        if(!applyLeadshineHardwareConfigFromUi()){
            return;
        }
    }

    fcOneDimKalmanHandler = OneDimKalmanHandler(axisForceSensorNum);
    fcOneDimKalmanHandler.setCov(std::vector<double>(axisForceSensorNum,0.0),std::vector<double>(axisForceSensorNum,0.002),
                                 std::vector<double>(axisForceSensorNum,0.05));// 0.002 50.0; 0.002 0.05

    hardwareInterface.threadCtrlCycleMs = (double)ui->devCtrlCycleMs->value();
    if(motiveLocalHandlerThread)
        motiveLocalHandlerThread->ctrlCycleMs = (double)(ui->devCtrlCycleMs->value());
    updateControlWorkerConfig();

    ensurePoseKinematicsHelpersReady(true);

    // 启动线程
    if(ccThread && !ccThread->isRunning()){
        ccThread->start(QThread::HighPriority);
    }
    if(motiveLocalHandlerThread && motiveLocalHandlerThread->isInit){
        mlhThread->start();
    }
    stopMonitor();
    startMonitor();
    updateSafetyMonitorConfig();
}

bool MainWindow::applyLeadshineHardwareConfigFromUi()
{
    if(!ui->devUseLS->isChecked()){
        return true;
    }

    std::vector<int> sensorAdr(axisForceSensorNum);
    std::vector<int> sensorDataAdr(axisForceSensorNum);
    std::vector<double> sensorCof(axisForceSensorNum);

    for(int i=0;i<ui->devAxisNum->value();++i){
        if(axisSensorIndexVec[i]->value()>=0){
            const int sensorIndex = axisSensorIndexVec[i]->value();
            sensorAdr[sensorIndex] = sensorIndex * 2;
            sensorDataAdr[sensorIndex] = sensorIndex;
            sensorCof[sensorIndex] = axisSensorCofVec[i]->value();
        }
    }
    hardwareInterface.setSensorPara(std::vector<int>(axisForceSensorNum,COM_EC_LS_SBT),
                                    std::vector<int>(axisForceSensorNum,ui->devFSPortNum->value()),
                                    sensorAdr,sensorDataAdr,std::vector<int>(axisForceSensorNum,ui->devFSDataLen->value()),
                                    sensorCof);
    hardwareInterface.setForceSensorIsSigned(ui->axisIsForceSensorSigned->isChecked());

    std::vector<unsigned int> motorIDVec(ui->devAxisNum->value(), 0U);
    std::vector<int> comType(ui->devAxisNum->value(), 0);
    std::vector<int> motorSlaveIdVec(ui->devAxisNum->value(), 0);
    std::vector<bool> motorTorqueVelocityLimitEnabled(ui->devAxisNum->value(), false);
    QSet<int> usedHardwareAxis;
    bool hasLeadshineAxis = false;
    for(int i=0;i<static_cast<int>(motorIDVec.size());++i){
        const bool motorAxis = isMotorAxis(i);
        const int configuredHardwareAxisNo = axisMotorHardwareAxisVec[i]->value();
        const int configuredSlaveId = axisMotorSlaveIdVec[i]->value();
        if(i == 0 || configuredHardwareAxisNo == 0){
            const QVariant axisTypeData = axisTypeVec[i]->currentData(Qt::UserRole);
            displayInfo(QStringLiteral("雷赛配置调试：逻辑轴%1，类型=\"%2\"，类型数据=%3，isMotor=%4，控制卡轴号=%5")
                        .arg(i)
                        .arg(axisTypeVec[i]->currentText().trimmed())
                        .arg(axisTypeData.isValid() ? QString::number(axisTypeData.toInt()) : QStringLiteral("invalid"))
                        .arg(motorAxis ? QStringLiteral("true") : QStringLiteral("false"))
                        .arg(configuredHardwareAxisNo)
                        .toStdString(),
                        motorAxis ? "info" : "warning");
        }
        if(!motorAxis){
            continue;
        }

        hasLeadshineAxis = true;
        if(configuredHardwareAxisNo < kLeadshineHardwareAxisMin ||
                configuredHardwareAxisNo > kLeadshineHardwareAxisMax){
            displayInfo(QStringLiteral("错误：逻辑电机轴%1的控制卡轴号必须设置为0到11").arg(i).toStdString(), "error");
            return false;
        }

        const int hardwareAxisIndex = configuredHardwareAxisNo;
        if(usedHardwareAxis.contains(hardwareAxisIndex)){
            displayInfo(QStringLiteral("错误：逻辑电机轴配置了重复的控制卡轴号 %1").arg(configuredHardwareAxisNo).toStdString(), "error");
            return false;
        }

        usedHardwareAxis.insert(hardwareAxisIndex);
        motorIDVec[i] = static_cast<unsigned int>(hardwareAxisIndex);
        motorSlaveIdVec[i] = configuredSlaveId;
        motorTorqueVelocityLimitEnabled[i] = !isLinearMotorAxis(i);
        comType[i] = COM_EC_LS;
    }

    if(!hasLeadshineAxis){
        displayInfo("错误：当前未配置任何雷赛电机轴", "error");
        return false;
    }

    hardwareInterface.setMotorPara(motorIDVec, comType, {}, {}, {}, motorSlaveIdVec,
                                   motorTorqueVelocityLimitEnabled);
    hardwareInterface.setLeadshineTorqueVelocityLimitRpm(motorTorqueServoVelocityLimitRpm());
    syncMotorSoftwareLimitsToHardware();
    if(!applyLeadshineAxisEquivFromUi()){
        return false;
    }
    return true;
}

void MainWindow::syncMotorSoftwareLimitsToHardware()
{
    if(runtimeState.systemRunning){
        return;
    }

    const int axisCount = ui->devAxisNum->value();
    std::vector<double> minPos(axisCount, 0.0);
    std::vector<double> maxPos(axisCount, 0.0);
    for(int i=0; i<axisCount; ++i){
        if(i < static_cast<int>(axisMotorMinVec.size()) && axisMotorMinVec[i]){
            minPos[i] = axisMotorMinVec[i]->value();
        }
        if(i < static_cast<int>(axisMotorMaxVec.size()) && axisMotorMaxVec[i]){
            maxPos[i] = axisMotorMaxVec[i]->value();
        }
    }

    if(minPos == lastSyncedMotorSoftwareMinPos &&
            maxPos == lastSyncedMotorSoftwareMaxPos){
        return;
    }

    hardwareInterface.setMotorSoftwareLimits(minPos, maxPos);
    lastSyncedMotorSoftwareMinPos = minPos;
    lastSyncedMotorSoftwareMaxPos = maxPos;
}

double MainWindow::configuredLeadshineAxisEquivForAxis(int axisIndex) const
{
    if(!isMotorAxis(axisIndex)){
        return 0.0;
    }

    double baseEquiv = isLinearMotorAxis(axisIndex) ? 728.178 : 1000.0;
    //728.178四舍五入，多圈之后有误差
    if(ui->devMotorFeedbackIsRd->isChecked()){
        baseEquiv *= 360.0;
    }
    return baseEquiv;
}

bool MainWindow::applyLeadshineAxisEquivFromUi()
{
    if(!ui->devUseLS->isChecked()){
        return true;
    }

    std::vector<double> motorEquivVec(ui->devAxisNum->value(), 0.0);
    bool hasLeadshineAxis = false;
    for(int i=0;i<ui->devAxisNum->value();++i){
        if(!isMotorAxis(i)){
            continue;
        }

        hasLeadshineAxis = true;
        const double axisEquiv = configuredLeadshineAxisEquivForAxis(i);
        if(axisEquiv <= 0.0){
            displayInfo(QStringLiteral("错误：逻辑电机轴%1未生成有效脉冲当量").arg(i).toStdString(), "error");
            return false;
        }
        motorEquivVec[i] = axisEquiv;
    }

    if(!hasLeadshineAxis){
        return true;
    }

    hardwareInterface.setLeadshineAxisEquiv(motorEquivVec);
    if(hardwareInterface.isLSConnected() && !hardwareInterface.applyLeadshineAxisEquiv()){
        displayInfo("错误：更新雷赛轴脉冲当量失败", "error");
        return false;
    }
    return true;
}

void MainWindow::delay(unsigned int msec){
    QTime _Timer = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < _Timer ){
        refreshSafetyMonitorHeartbeat();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);// 处理本线程的事件循环，处理事件循环最多100ms必须返回本语句。可以防止线程阻塞
    }
    refreshSafetyMonitorHeartbeat();
}

bool MainWindow::togglePauseResumeFromUi()
{
    const RobotStateSnapshot robotState = currentRobotState();
    if(runtimeState.safetyFaultLatched && robotState.positionMotionState != PositionMotionState::PausedPvt){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：暂停按钮已按下，但当前存在安全锁存故障，请先复位"),
                                     QStringLiteral("warning"));
        refreshRunModeUiState();
        return false;
    }

    if(robotState.positionMotionState == PositionMotionState::PausedPvt){
        if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：继续失败，当前未连接雷赛控制卡"),
                                         QStringLiteral("error"));
            return false;
        }

        bool resumed = false;
        if(pvtExecutionWorker){
            invokeOnObjectThreadWithHeartbeat(pvtExecutionWorker, [&](){
                resumed = pvtExecutionWorker->resumePvtMotion();
            }, [this](){
                refreshSafetyMonitorHeartbeat();
            });
        }
        if(resumed){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：已从暂停点继续位置轨迹"),
                                         QStringLiteral("info"));
            refreshRunModeUiState();
        }
        return resumed;
    }

    if(robotState.positionMotionState == PositionMotionState::ExecutingPvt){
        if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：暂停失败，当前未连接雷赛控制卡"),
                                         QStringLiteral("error"));
            return false;
        }

        bool paused = false;
        if(pvtExecutionWorker){
            invokeOnObjectThreadWithHeartbeat(pvtExecutionWorker, [&](){
                paused = pvtExecutionWorker->pausePvtMotion();
            }, [this](){
                refreshSafetyMonitorHeartbeat();
            });
        }
        if(paused){
            showPrimaryOperationFeedback(QStringLiteral("主控反馈：已平滑暂停当前位置轨迹，可点击“继续”恢复运行"),
                                         QStringLiteral("info"));
            refreshRunModeUiState();
        }
        return paused;
    }

    if(robotState.positionMotionState == PositionMotionState::Simulating){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：暂停按钮已按下，但当前轨迹仿真阶段暂不支持暂停/继续"),
                                     QStringLiteral("warning"));
        return false;
    }

    if(robotState.gcRunning || robotState.forceControlThreadRunning){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：暂停按钮已按下，但当前仅支持位置轨迹暂停/继续"),
                                     QStringLiteral("warning"));
        return false;
    }

    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        showPrimaryOperationFeedback(QStringLiteral("主控反馈：暂停按钮已按下，但当前未连接雷赛控制卡，也没有可暂停轨迹"),
                                     QStringLiteral("warning"));
        return false;
    }

    showPrimaryOperationFeedback(QStringLiteral("主控反馈：暂停按钮已按下，但当前没有可暂停或继续的运动任务"),
                                 QStringLiteral("warning"));
    return false;
}

bool MainWindow::motorStop(){
    if(trajectoryCheckTimer){
        trajectoryCheckTimer->stop();
    }
    runtimeState.autoExecutePoseAfterSimulation = false;
    if(runtimeState.motorTorqueDebugActive){
        stopMotorTorqueDebug();
    }

    stopGravityThread();
    optionalButtonSetEnabled(this, "mainGCModeSwitch", true);
    optionalButtonSetText(this, "mainGCModeSwitch", QStringLiteral("启动零力拖动"));

    ui->mainFCThread->setChecked(false);
    for(int i=0;i<useFCBtnVec.size();++i){
        useFCBtnVec[i]->setChecked(false);
    }
    clearHybridPoseForceModeState(false);

    if(simulationWorker){
        simulationWorker->stopPositionSimulation();
    }
    positionSimulationModel = nullptr;
    runtimeState.posModeRunning = false;
    runtimeState.pvtCommandActive = false;
    runtimeState.singleMotorTrajectoryActive = false;
    runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    lastTrajEndMotorTheta.clear();
    activePosModeMotorIndex.clear();
    activePosModeStartMotorTheta.clear();
    activePosModeContinuousStartMotorTheta.clear();
    runtimeState.controlBoxPausedPvtMotion = false;
    runtimeState.controlBoxPausedForceControl = false;
    runtimeState.controlBoxPausedGC = false;
    hardwareInterface.clearPvtTrajectoryState();
    runtimeState.pvtCommandActive = false;

    ui->mainPosModeSwitch->setEnabled(true);
    setPoseSimulationIdleText(ui);
    ui->mainPosModeSend->setEnabled(true);
    ui->mainReplay3DViewer->setEnabled(true);
    setForceControlSelectionEnabled(true);
    updateCableHomeConfirmEnabled();
    ui->mainStopSwitch->setText("停止");

    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        displayInfo("错误：雷赛控制卡未连接，无法下发电机停止指令", "error");
        return false;
    }

    bool stopOk = true;
    for(int i=0;i<ui->devAxisNum->value();++i){
        if(isMotorAxis(i)){
            stopOk = hardwareInterface.motorStop(i) && stopOk;
        }
    }
    if(stopOk){
        displayInfo("按钮停止：已停止所有电机并清理运行状态");
    }
    return stopOk;
}

bool MainWindow::stopMotionFromControlBox(){
    const RobotStateSnapshot robotState = currentRobotState();
    runtimeState.autoExecutePoseAfterSimulation = false;
    const bool hasPositionTrajectory = robotState.posModeRunning ||
            robotState.pvtMotionRunning ||
            robotState.pvtMotionPaused ||
            (robotState.pvtTrajectoryAvailable && !robotState.pvtMotionFinished);
    const bool hasForceTrajectory = robotState.gcRunning;
    const bool hasTorqueDebug = runtimeState.motorTorqueDebugActive;
    const bool hasSingleMotorTrajectory = runtimeState.singleMotorTrajectoryActive;

    if(!hasPositionTrajectory && !hasForceTrajectory && !hasTorqueDebug && !hasSingleMotorTrajectory){
        displayInfo("当前没有运行中的位置轨迹或力控轨迹，停止指令不执行");
        return true;
    }

    if(trajectoryCheckTimer){
        trajectoryCheckTimer->stop();
    }

    if(hasForceTrajectory){
        stopGravityThread();
        optionalButtonSetEnabled(this, "mainGCModeSwitch", true);
        optionalButtonSetText(this, "mainGCModeSwitch", QStringLiteral("启动零力拖动"));
    }
    if(hasTorqueDebug){
        stopMotorTorqueDebug();
    }
    if(hasSingleMotorTrajectory){
        stopSingleMotorTrajectory(QStringLiteral("control_box_stop"));
    }

    ui->mainFCThread->setChecked(false);
    for(int i=0;i<useFCBtnVec.size();++i){
        useFCBtnVec[i]->setChecked(false);
    }
    clearHybridPoseForceModeState(false);

    if(simulationWorker){
        simulationWorker->stopPositionSimulation();
    }
    positionSimulationModel = nullptr;
    runtimeState.posModeRunning = false;
    runtimeState.pvtCommandActive = false;
    runtimeState.singleMotorTrajectoryActive = false;
    ui->mainPosModeSwitch->setEnabled(true);
    setPoseSimulationIdleText(ui);
    ui->mainPosModeSend->setEnabled(true);
    ui->mainReplay3DViewer->setEnabled(true);

    bool stopOk = true;
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        displayInfo("警告：雷赛控制卡未连接，仅清理软件轨迹状态，无法下发电机停止指令", "warning");
    }
    else if(hasPositionTrajectory && !hasSingleMotorTrajectory){
        stopOk = false;
        if(pvtExecutionWorker){
            invokeOnObjectThreadWithHeartbeat(pvtExecutionWorker, [&](){
                stopOk = pvtExecutionWorker->pausePvtMotion();
                pvtExecutionWorker->clearPvtState();
            }, [this](){
                refreshSafetyMonitorHeartbeat();
            });
        }
    }
    else if(hasForceTrajectory){
        stopAllMotorAxesForSpeedZero();
    }
    lastTrajEndMotorTheta.clear();
    std::vector<double> savedStartTheta = activePosModeStartMotorTheta;
    std::vector<int> savedMotorIndex = activePosModeMotorIndex;
    activePosModeStartMotorTheta.clear();
    activePosModeContinuousStartMotorTheta.clear();
    runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    runtimeState.controlBoxPausedPvtMotion = false;
    runtimeState.controlBoxPausedForceControl = false;
    runtimeState.controlBoxPausedGC = false;
    setForceControlSelectionEnabled(true);
    updateCableHomeConfirmEnabled();
    ui->mainStopSwitch->setText("停止");

    if(!stopOk){
        return false;
    }

    if(hasPositionTrajectory && !hasSingleMotorTrajectory && !savedStartTheta.empty() && !savedMotorIndex.empty() && ui->devUseLS->isChecked() && hardwareInterface.isLSConnected()){
        displayInfo("轨迹已暂停，电机将缓慢返回本段轨迹起始点...");
        QTimer::singleShot(100, this, [this, savedStartTheta, savedMotorIndex](){
            returnMotorAxesToStart(savedMotorIndex, savedStartTheta);
        });
    }

    return true;
}

void MainWindow::returnMotorAxesToStart(const std::vector<int>& motorIndex, const std::vector<double>& targetTheta){
    if(motorIndex.size() != targetTheta.size() || motorIndex.empty()){
        displayInfo("错误：回轨迹起始点参数无效", "error");
        return;
    }
    if(!ui->devUseLS->isChecked() || !hardwareInterface.isLSConnected()){
        displayInfo("警告：雷赛控制卡未连接，无法执行回轨迹起始点", "warning");
        return;
    }
    delay(100);
    const std::vector<double> motorHome = hardwareInterface.getAllMotorHome();
    for(size_t i=0;i<motorIndex.size();++i){
        const int idx = motorIndex[i];
        double relativeTarget = targetTheta[i];
        if(idx >= 0 && idx < static_cast<int>(motorHome.size())){
            relativeTarget -= motorHome[idx];
        }
        const double curPos = hardwareInterface.readMotorCurPos(idx);
        const double vel = std::max(std::fabs(targetTheta[i] - curPos) / 15.0, 1e-5);
        QString limitError;
        if(!validateMotorCommandLimits(idx,
                                       relativeTarget,
                                       vel,
                                       QStringLiteral("回轨迹起始点"),
                                       &limitError)){
            hardwareInterface.motorStop(idx);
            displayInfo(limitError.toStdString(), "error");
            return;
        }
    }

    bool moveOk = true;
    for(size_t i=0;i<motorIndex.size();++i){
        int idx = motorIndex[i];
        double curPos = hardwareInterface.readMotorCurPos(idx);
        double vel = std::fabs(targetTheta[i] - curPos) / 15.0;
        vel = std::max(std::fabs(vel), 1e-5);
        moveOk = hardwareInterface.motorAbsPos(idx, targetTheta[i], vel) && moveOk;
    }
    if(!moveOk){
        displayInfo("错误：回轨迹起始点时存在电机指令下发失败", "error");
        return;
    }
    displayInfo("电机正在返回本段轨迹起始点...");
}

bool MainWindow::motor2Home(){
    if(trajectoryCheckTimer){
        trajectoryCheckTimer->stop();
    }
    if(runtimeState.singleMotorTrajectoryActive){
        stopSingleMotorTrajectory(QStringLiteral("motor_home"));
    }
    const bool keepPositionContinuousForHoming =
            ui->Poscontinuetraj->isChecked() &&
            calibrationWorkflowStage != CalibrationWorkflowStage::MechanicalHoming;
    const bool keepCableHomeForContinuousHoming =
            keepPositionContinuousForHoming &&
            runtimeState.cableHomeState == CableHomeState::Confirmed;
    if(keepPositionContinuousForHoming){
        optionalCheckBoxSetChecked(this, "Forcecontinuetraj", false);
    }
    else{
        resetContinuousTrajectoryOptions();
    }
    if(!keepCableHomeForContinuousHoming){
        runtimeState.cableHomeState = CableHomeState::Unconfirmed;
    }
    updateCableHomeConfirmEnabled();
    hardwareInterface.clearPvtTrajectoryState();
    runtimeState.pvtCommandActive = false;
    if(simulationWorker){
        simulationWorker->stopPositionSimulation();
    }
    positionSimulationModel = nullptr;
    runtimeState.posModeRunning = false;
    runtimeState.pvtCommandActive = false;
    runtimeState.singleMotorTrajectoryActive = false;
    lastTrajEndMotorTheta.clear();
    activePosModeMotorIndex.clear();
    activePosModeStartMotorTheta.clear();
    activePosModeContinuousStartMotorTheta.clear();
    ui->mainPosModeSwitch->setEnabled(true);
    setPoseSimulationIdleText(ui);
    ui->mainPosModeSend->setEnabled(true);
    ui->mainReplay3DViewer->setEnabled(true);

    ui->mainFCThread->setChecked(false);
    for(int i=0;i<useFCBtnVec.size();++i){
        useFCBtnVec[i]->setChecked(false);// 先取消所有力控
    }
    clearHybridPoseForceModeState(false);
    setForceControlSelectionEnabled(true);
    delay(100);// 等待力控速度指令停止
    if(keepCableHomeForContinuousHoming){
        displayInfo("机械回零：已保留连续轨迹选项和预紧零点，使用当前正运动学位姿作为回零轨迹起点");
    }

    if(ui->devUseLS->isChecked() && hardwareInterface.isLSConnected()){
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isMotorAxis(i)){
                hardwareInterface.motorStop(i);
            }
        }
        delay(50);
        if(!executeCalibrationMechanicalHoming(QStringLiteral("机械回零"))){
            return false;
        }
    }
    else{
        displayInfo("错误：雷赛控制卡未连接，无法执行电机回零位", "error");
        return false;
    }
    return true;
}

bool MainWindow::resetCableForceHome(){
    refreshAxisRuntimeCounts();
    if(axisForceSensorNum <= 0){
        displayInfo("错误：当前未配置张力传感器，无法采集张力零点", "error");
        refreshCalibrationUiState();
        return false;
    }
    if(static_cast<int>(mainForceSensorDataVal.size()) < axisForceSensorNum){
        displayInfo("错误：当前张力传感器数据不足，无法采集张力零点", "error");
        refreshCalibrationUiState();
        return false;
    }
    for(int i=0;i<axisForceSensorNum;++i){
        forceSensorCurHome[i] = mainForceSensorDataVal[i];
    }
    hardwareInterface.setForceSensorHome(forceSensorCurHome);
    if(runtimeState.cableHomeState != CableHomeState::Confirmed){
        runtimeState.cableHomeState = CableHomeState::WaitingForceStable;
    }
    updateCableHomeConfirmEnabled();
    displayInfo(QStringLiteral("零位校准：已采集当前张力读数作为零点基准 %1")
                    .arg(formatCalibrationVectorPreview(forceSensorCurHome)).toStdString());
    refreshCalibrationUiState();
    return true;
}

bool MainWindow::isCableForceWithinHomeConfirmTolerance() const{
    std::vector<bool> checkedSensor(axisForceSensorNum,false);
    bool hasValidSensor = false;

    for(int axisIndex=0; axisIndex<ui->devAxisNum->value(); ++axisIndex){
        if(!isModeledMotorAxis(axisIndex)){
            continue;
        }

        const int sensorIndex = axisSensorIndexVec[axisIndex]->value();
        if(sensorIndex < 0 || sensorIndex >= axisForceSensorNum || checkedSensor[sensorIndex]){
            continue;
        }
        if(sensorIndex >= static_cast<int>(mainForceSensorDataVal.size()) ||
                sensorIndex >= static_cast<int>(mainForceSensorExp.size())){
            return false;
        }

        checkedSensor[sensorIndex] = true;
        hasValidSensor = true;

        const double curForce = mainForceSensorDataVal[sensorIndex];
        const double expForce = mainForceSensorExp[sensorIndex]->value();
        const double allowedErr = std::max(std::fabs(expForce) * 2.0, 0.1);
        if(std::fabs(curForce - expForce) > allowedErr){
            return false;
        }
    }

    return hasValidSensor;
}

bool MainWindow::shouldSkipCableHomeCheckForMotion(const QString& motionName) const{
    if(motionName == QStringLiteral("零力拖动/动力学阻抗")){
        return optionalCheckBoxChecked(this, "Forcecontinuetraj", false);
    }

    return ui->Poscontinuetraj->isChecked();
}

void MainWindow::resetContinuousTrajectoryOptions(){
    ui->Poscontinuetraj->setChecked(false);
    optionalCheckBoxSetChecked(this, "Forcecontinuetraj", false);
}

bool MainWindow::ensureCableHomeReadyForMotion(const QString& motionName){
    if(runtimeState.cableHomeState == CableHomeState::Confirmed){
        return true;
    }

    if(shouldSkipCableHomeCheckForMotion(motionName)){
        return true;
    }

    runtimeState.cableHomeState = CableHomeState::WaitingForceStable;
    updateCableHomeConfirmEnabled();
    displayInfo(QString("错误：%1启动失败，请先完成预紧并点击“预紧确认”")
                    .arg(motionName)
                    .toStdString(), "error");
    return false;
}

void MainWindow::updateCableHomeConfirmEnabled(){
    const bool motionBlockingConfirm =
            runtimeState.posModeRunning ||
            runtimeState.pvtCommandActive ||
            runtimeState.gcRunning ||
            calibrationWorkflowStage == CalibrationWorkflowStage::MechanicalHoming;
    const bool canConfirm = runtimeState.cableHomeState != CableHomeState::Confirmed &&
            !motionBlockingConfirm;
    ui->mainSetCableHome->setEnabled(canConfirm);
    refreshCalibrationUiState();
}

bool MainWindow::resetRotHome(){// 目前暂时只有动捕+零力用到了
    if(motiveLocalHandlerThread){
        std::vector<std::vector<double>> tmp = motiveLocalHandlerThread->getRigidPose();
        if(!tmp.empty()){
            for(int ii=0;ii<3;++ii){
                endCurRotHome[ii] = tmp[0][ii+3];
            }
        }
    }
    return true;
}

bool MainWindow::setCableHome(){
    if(!isCableForceWithinHomeConfirmTolerance()){
        runtimeState.cableHomeState = CableHomeState::WaitingForceStable;
        updateCableHomeConfirmEnabled();
        displayInfo("错误：预紧确认失败，当前绳索力与期望力误差未全部小于10%", "error");
        refreshCalibrationUiState();
        return false;
    }

    runtimeState.cableHomeState = CableHomeState::Confirmed;
    const std::vector<double> cableHomeAbsPos = hardwareInterface.getAllMotorPos();
    hardwareInterface.setMotorHome(cableHomeAbsPos);
    if(!cableHomeAbsPos.empty()){
        applyMotorPositionSnapshot(cableHomeAbsPos);
    }
    motorPosDisplayZero.clear();
    homeCableVec.clear();
    homeCableLen.clear();
    cableHomePos.clear();
    lastForwardKinematicsPose.clear();
    lastForwardKinematicsPoseTimestampMs = -1;
    if(!ui->devUseCamForActPose->isChecked()){
        cacheForwardKinematicsPose(configuredCableHomePlatformPose());
    }

    ui->mainFCThread->setChecked(false);
    ui->mainAllUseFC->setChecked(false);
    allUseFC(false);
    setForceControlSelectionEnabled(true);
    if(ui->devUseLS->isChecked() && hardwareInterface.isLSConnected()){
        for(int i=0;i<ui->devAxisNum->value();++i){
            if(isMotorAxis(i)){
                hardwareInterface.motorStop(i);
            }
        }
    }

    lastTrajEndMotorTheta.clear();
    activePosModeMotorIndex.clear();
    activePosModeStartMotorTheta.clear();
    activePosModeContinuousStartMotorTheta.clear();
    runtimeState.controlBoxPausedPvtMotion = false;
    runtimeState.controlBoxPausedForceControl = false;
    runtimeState.controlBoxPausedGC = false;

    ui->mainSetCableHome->setEnabled(false);
    ui->mainPosModeSwitch->setEnabled(true);
    optionalButtonSetEnabled(this, "mainGCModeSwitch", true);
    ui->mainResetCableForceHomeSwitch->setEnabled(true);
    calibrationWorkflowStage = CalibrationWorkflowStage::Completed;
    updateControlWorkerConfig();
    if(!ui->devUseCamForActPose->isChecked()){
        if(!refreshForwardKinematicsPoseFromCurrentMotorPosition()){
            displayInfo("警告：预紧确认已记录，但当前未能生成正运动学位姿；位置模式启动前会再次读取电机位置反馈，请检查轴类型、末端编号、参数配置和电机位置反馈",
                        "warning");
        }
    }
    saveCalibrationSnapshotToFile(latestCalibrationSnapshotPath(), false);
    displayInfo("零位校准完成：已记录当前电机位置为绳索/轨迹零点，力控已关闭，系统处于可运行状态");
    refreshCalibrationUiState();
    refreshRunModeUiState();
    return true;
}

void MainWindow::allUseFC(bool isChecked){
    if(isSingleCableForceDebugModeActive()){
        if(ui->mainAllUseFC->isChecked()){
            ui->mainAllUseFC->setChecked(false);
        }
        enforceSingleCableForceSelection();
        return;
    }
    for(int i=0;i<useFCBtnVec.size();++i)
        useFCBtnVec[i]->setChecked(isChecked);
}



void MainWindow::camLock(bool lockCam){
    ui->mainResetMotiveFit->setEnabled(lockCam);
    motiveFitConfirmed = !lockCam;
    refreshCalibrationUiState();
}
//设置UI的信号槽连接
void MainWindow::setUIVec(){
    axisTypeVec = {ui->axisType,ui->axisType_2,ui->axisType_3,ui->axisType_4,ui->axisType_5,ui->axisType_6,ui->axisType_7,ui->axisType_8,
                   ui->axisType_9,ui->axisType_10,ui->axisType_11,ui->axisType_12};
    axisEndVec = {ui->axisEnd,ui->axisEnd_2,ui->axisEnd_3,ui->axisEnd_4,ui->axisEnd_5,ui->axisEnd_6,ui->axisEnd_7,ui->axisEnd_8,
                  ui->axisEnd_9,ui->axisEnd_10,ui->axisEnd_11,ui->axisEnd_12};
    axisIsPos2NegVec = {ui->axisIsPos2Neg,ui->axisIsPos2Neg_2,ui->axisIsPos2Neg_3,ui->axisIsPos2Neg_4,ui->axisIsPos2Neg_5,ui->axisIsPos2Neg_6,ui->axisIsPos2Neg_7,ui->axisIsPos2Neg_8,
                        ui->axisIsPos2Neg_9,ui->axisIsPos2Neg_10,ui->axisIsPos2Neg_11,ui->axisIsPos2Neg_12};
    axisSensorIndexVec = {ui->axisSensorIndex,ui->axisSensorIndex_2,ui->axisSensorIndex_3,ui->axisSensorIndex_4,ui->axisSensorIndex_5,ui->axisSensorIndex_6,ui->axisSensorIndex_7,ui->axisSensorIndex_8,
                          ui->axisSensorIndex_9,ui->axisSensorIndex_10,ui->axisSensorIndex_11,ui->axisSensorIndex_12};
    axisMotorHardwareAxisVec = {ui->axisSensorAdr,ui->axisSensorAdr_2,ui->axisSensorAdr_3,ui->axisSensorAdr_4,ui->axisSensorAdr_5,ui->axisSensorAdr_6,ui->axisSensorAdr_7,ui->axisSensorAdr_8,
                                ui->axisSensorAdr_9,ui->axisSensorAdr_10,ui->axisSensorAdr_11,ui->axisSensorAdr_12};
    axisMotorSlaveIdVec = {ui->axisSensorDataAdr,ui->axisSensorDataAdr_2,ui->axisSensorDataAdr_3,ui->axisSensorDataAdr_4,ui->axisSensorDataAdr_5,ui->axisSensorDataAdr_6,ui->axisSensorDataAdr_7,ui->axisSensorDataAdr_8,
                           ui->axisSensorDataAdr_9,ui->axisSensorDataAdr_10,ui->axisSensorDataAdr_11,ui->axisSensorDataAdr_12};
    axisMotorCofVec = {ui->axisMotorCof,ui->axisMotorCof_2,ui->axisMotorCof_3,ui->axisMotorCof_4,ui->axisMotorCof_5,ui->axisMotorCof_6,ui->axisMotorCof_7,ui->axisMotorCof_8,
                       ui->axisMotorCof_9,ui->axisMotorCof_10,ui->axisMotorCof_11,ui->axisMotorCof_12};
    axisSensorCofVec = {ui->axisSensorCof,ui->axisSensorCof_2,ui->axisSensorCof_3,ui->axisSensorCof_4,ui->axisSensorCof_5,ui->axisSensorCof_6,ui->axisSensorCof_7,ui->axisSensorCof_8,
                        ui->axisSensorCof_9,ui->axisSensorCof_10,ui->axisSensorCof_11,ui->axisSensorCof_12};
    axisMotorZeroVec = {ui->axisMotorZero,ui->axisMotorZero_2,ui->axisMotorZero_3,ui->axisMotorZero_4,ui->axisMotorZero_5,ui->axisMotorZero_6,ui->axisMotorZero_7,ui->axisMotorZero_8,
                        ui->axisMotorZero_9,ui->axisMotorZero_10,ui->axisMotorZero_11,ui->axisMotorZero_12};
    axisSensorZeroVec = {ui->axisSensorZero,ui->axisSensorZero_2,ui->axisSensorZero_3,ui->axisSensorZero_4,ui->axisSensorZero_5,ui->axisSensorZero_6,ui->axisSensorZero_7,ui->axisSensorZero_8,
                         ui->axisSensorZero_9,ui->axisSensorZero_10,ui->axisSensorZero_11,ui->axisSensorZero_12};
    curMotorPosTextVec = {ui->Curmotorpos_1, ui->Curmotorpos_2, ui->Curmotorpos_3, ui->Curmotorpos_4,
                          ui->Curmotorpos_5, ui->Curmotorpos_6, ui->Curmotorpos_7, ui->Curmotorpos_8,
                          ui->Curmotorpos_9, ui->Curmotorpos_10, ui->Curmotorpos_11, ui->Curmotorpos_12};
    axisCableStartPosXVec = {ui->axisCableStartPosX,ui->axisCableStartPosX_2,ui->axisCableStartPosX_3,ui->axisCableStartPosX_4,ui->axisCableStartPosX_5,ui->axisCableStartPosX_6,ui->axisCableStartPosX_7,ui->axisCableStartPosX_8,
                             ui->axisCableStartPosX_9,ui->axisCableStartPosX_10,ui->axisCableStartPosX_11,ui->axisCableStartPosX_12};
    axisCableStartPosYVec = {ui->axisCableStartPosY,ui->axisCableStartPosY_2,ui->axisCableStartPosY_3,ui->axisCableStartPosY_4,ui->axisCableStartPosY_5,ui->axisCableStartPosY_6,ui->axisCableStartPosY_7,ui->axisCableStartPosY_8,
                             ui->axisCableStartPosY_9,ui->axisCableStartPosY_10,ui->axisCableStartPosY_11,ui->axisCableStartPosY_12};
    axisCableStartPosZVec = {ui->axisCableStartPosZ,ui->axisCableStartPosZ_2,ui->axisCableStartPosZ_3,ui->axisCableStartPosZ_4,ui->axisCableStartPosZ_5,ui->axisCableStartPosZ_6,ui->axisCableStartPosZ_7,ui->axisCableStartPosZ_8,
                             ui->axisCableStartPosZ_9,ui->axisCableStartPosZ_10,ui->axisCableStartPosZ_11,ui->axisCableStartPosZ_12};
    axisCableEndPosXVec = {ui->axisCableEndPosX,ui->axisCableEndPosX_2,ui->axisCableEndPosX_3,ui->axisCableEndPosX_4,ui->axisCableEndPosX_5,ui->axisCableEndPosX_6,ui->axisCableEndPosX_7,ui->axisCableEndPosX_8,
                           ui->axisCableEndPosX_9,ui->axisCableEndPosX_10,ui->axisCableEndPosX_11,ui->axisCableEndPosX_12};
    axisCableEndPosYVec = {ui->axisCableEndPosY,ui->axisCableEndPosY_2,ui->axisCableEndPosY_3,ui->axisCableEndPosY_4,ui->axisCableEndPosY_5,ui->axisCableEndPosY_6,ui->axisCableEndPosY_7,ui->axisCableEndPosY_8,
                           ui->axisCableEndPosY_9,ui->axisCableEndPosY_10,ui->axisCableEndPosY_11,ui->axisCableEndPosY_12};
    axisCableEndPosZVec = {ui->axisCableEndPosZ,ui->axisCableEndPosZ_2,ui->axisCableEndPosZ_3,ui->axisCableEndPosZ_4,ui->axisCableEndPosZ_5,ui->axisCableEndPosZ_6,ui->axisCableEndPosZ_7,ui->axisCableEndPosZ_8,
                           ui->axisCableEndPosZ_9,ui->axisCableEndPosZ_10,ui->axisCableEndPosZ_11,ui->axisCableEndPosZ_12};
    axisCableZeroLenVec = {ui->axisCableZeroLen,ui->axisCableZeroLen_2,ui->axisCableZeroLen_3,ui->axisCableZeroLen_4,ui->axisCableZeroLen_5,ui->axisCableZeroLen_6,ui->axisCableZeroLen_7,ui->axisCableZeroLen_8,
                           ui->axisCableZeroLen_9,ui->axisCableZeroLen_10,ui->axisCableZeroLen_11,ui->axisCableZeroLen_12};
    axisMotorMaxVec = {ui->axisMotorMax,ui->axisMotorMax_2,ui->axisMotorMax_3,ui->axisMotorMax_4,ui->axisMotorMax_5,ui->axisMotorMax_6,ui->axisMotorMax_7,ui->axisMotorMax_8,
                       ui->axisMotorMax_9,ui->axisMotorMax_10,ui->axisMotorMax_11,ui->axisMotorMax_12};
    axisMotorMinVec = {ui->axisMotorMin,ui->axisMotorMin_2,ui->axisMotorMin_3,ui->axisMotorMin_4,ui->axisMotorMin_5,ui->axisMotorMin_6,ui->axisMotorMin_7,ui->axisMotorMin_8,
                       ui->axisMotorMin_9,ui->axisMotorMin_10,ui->axisMotorMin_11,ui->axisMotorMin_12};
    axisMotorVelMaxVec = {ui->axisMotorVelMax,ui->axisMotorVelMax_2,ui->axisMotorVelMax_3,ui->axisMotorVelMax_4,ui->axisMotorVelMax_5,ui->axisMotorVelMax_6,ui->axisMotorVelMax_7,ui->axisMotorVelMax_8,
                          ui->axisMotorVelMax_9,ui->axisMotorVelMax_10,ui->axisMotorVelMax_11,ui->axisMotorVelMax_12};
    axisForceMaxVec = {ui->axisForceMax,ui->axisForceMax_2,ui->axisForceMax_3,ui->axisForceMax_4,ui->axisForceMax_5,ui->axisForceMax_6,ui->axisForceMax_7,ui->axisForceMax_8,
                       ui->axisForceMax_9,ui->axisForceMax_10,ui->axisForceMax_11,ui->axisForceMax_12};
    axisImqVec = {ui->axisImq,ui->axisImq_2,ui->axisImq_3,ui->axisImq_4,ui->axisImq_5,ui->axisImq_6,ui->axisImq_7,ui->axisImq_8,
                  ui->axisImq_9,ui->axisImq_10,ui->axisImq_11,ui->axisImq_12};
    axisFvqVec = {ui->axisFvq,ui->axisFvq_2,ui->axisFvq_3,ui->axisFvq_4,ui->axisFvq_5,ui->axisFvq_6,ui->axisFvq_7,ui->axisFvq_8,
                  ui->axisFvq_9,ui->axisFvq_10,ui->axisFvq_11,ui->axisFvq_12};
    axisFcqVec = {ui->axisFcq,ui->axisFcq_2,ui->axisFcq_3,ui->axisFcq_4,ui->axisFcq_5,ui->axisFcq_6,ui->axisFcq_7,ui->axisFcq_8,
                  ui->axisFcq_9,ui->axisFcq_10,ui->axisFcq_11,ui->axisFcq_12};

    QStringList typeList;
    typeList << QStringLiteral("/") << motorAxisText() << linearMotorAxisText();
    for(int i=0;i<axisTypeVec.size();++i){
        axisTypeVec[i]->clear();
        axisTypeVec[i]->addItem(typeList[0], 0);
        axisTypeVec[i]->addItem(typeList[1], COM_EC_LS);
        axisTypeVec[i]->addItem(typeList[2], COM_EC_LS);
        if(axisMotorHardwareAxisVec[i]){
            axisMotorHardwareAxisVec[i]->setRange(kLeadshineHardwareAxisMin, kLeadshineHardwareAxisMax);
        }
        if(axisMotorSlaveIdVec[i]){
            axisMotorSlaveIdVec[i]->setRange(0, 99999);
        }
    }

    endMassVec = {ui->endMass};
    endIxxVec = {ui->endIxx};endIyyVec = {ui->endIyy};endIzzVec = {ui->endIzz};
    endIxyVec = {ui->endIxy};endIxzVec = {ui->endIxz};endIyzVec = {ui->endIyz};
    endImpMdXVec = {ui->endImpMdX};
    endImpMdYVec = {ui->endImpMdY};
    endImpMdZVec = {ui->endImpMdZ};
    endImpMdRxVec = {ui->endImpMdRx};
    endImpMdRyVec = {ui->endImpMdRy};
    endImpMdRzVec = {ui->endImpMdRz};
    endImpDdXVec = {ui->endImpDdX};
    endImpDdYVec = {ui->endImpDdY};
    endImpDdZVec = {ui->endImpDdZ};
    endImpDdRxVec = {ui->endImpDdRx};
    endImpDdRyVec = {ui->endImpDdRy};
    endImpDdRzVec = {ui->endImpDdRz};
    endImpKdXVec = {ui->endImpKdX};
    endImpKdYVec = {ui->endImpKdY};
    endImpKdZVec = {ui->endImpKdZ};
    endImpKdRxVec = {ui->endImpKdRx};
    endImpKdRyVec = {ui->endImpKdRy};
    endImpKdRzVec = {ui->endImpKdRz};

    mainGCEndXVec = {findOptionalUiObject<QDoubleSpinBox>(this, "mainGCEndX")};
    mainGCEndYVec = {findOptionalUiObject<QDoubleSpinBox>(this, "mainGCEndY")};
    mainGCEndZVec = {findOptionalUiObject<QDoubleSpinBox>(this, "mainGCEndZ")};
    mainGCEndMxVec = {findOptionalUiObject<QDoubleSpinBox>(this, "mainGCEndMx")};
    mainGCEndMyVec = {findOptionalUiObject<QDoubleSpinBox>(this, "mainGCEndMy")};
    mainGCEndMzVec = {findOptionalUiObject<QDoubleSpinBox>(this, "mainGCEndMz")};

    useFCBtnVec = {ui->mainUseFC,ui->mainUseFC_2,ui->mainUseFC_3,ui->mainUseFC_4,ui->mainUseFC_5,ui->mainUseFC_6,
                   ui->mainUseFC_7,ui->mainUseFC_8};
    mainForceSensorData = {ui->mainForceSensorData,ui->mainForceSensorData_2,ui->mainForceSensorData_3,ui->mainForceSensorData_4,
                             ui->mainForceSensorData_5,ui->mainForceSensorData_6,ui->mainForceSensorData_7,ui->mainForceSensorData_8};
    mainForceSensorDataVal.resize(mainForceSensorData.size());
    mainForceSensorExp = {ui->mainForceSensorExp,ui->mainForceSensorExp_2,ui->mainForceSensorExp_3,ui->mainForceSensorExp_4,
                          ui->mainForceSensorExp_5,ui->mainForceSensorExp_6,ui->mainForceSensorExp_7,ui->mainForceSensorExp_8};
    mainForceSensorExpVal.resize(mainForceSensorExp.size());
    mainMotorTorqueData = {ui->mainMotorTorqueData,ui->mainMotorTorqueData_2,ui->mainMotorTorqueData_3,ui->mainMotorTorqueData_4,
                           ui->mainMotorTorqueData_5,ui->mainMotorTorqueData_6,ui->mainMotorTorqueData_7,ui->mainMotorTorqueData_8};
}
