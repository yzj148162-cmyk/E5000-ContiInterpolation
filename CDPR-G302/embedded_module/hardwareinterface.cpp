/*
 * 文件总览：
 * - HardwareInterface 的实现文件，集中处理雷赛 SDK 调用、单位换算、Trace 数据解析、PVT 下发和故障/停机动作。
 * - 复杂处重点关注三类转换：逻辑轴到硬件轴、原始脉冲/寄存器到工程单位、板卡 Trace 缓冲到带时间戳的反馈样本。
 * - 这里的函数多直接触碰设备状态，调用前通常需要 MainWindow 或 SafetyMonitor 已完成运行状态与互斥检查。
 */

#include "hardwareinterface.h"
#include "runtimefeatureswitches.h"
#include "runtimepathutils.h"
#include "trajectoryplanner.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDevice>
#include <QFileInfo>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <utility>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace {
// nmc_get_axis_errcode 返回驱动器 WORD 故障码。按手册通用的十六进制格式输出，
// 例如十进制 25472 显示为 0x6380。
QString formatLeadshineDriveErrorCodeHex(WORD errorCode)
{
    const QString hexText = QString::number(static_cast<quint16>(errorCode), 16)
            .rightJustified(4, QLatin1Char('0'))
            .toUpper();
    return QStringLiteral("0x%1").arg(hexText);
}

double convertRawToSigned(long rawVal, bool isDirectSigned)
{
    if (isDirectSigned) {
        return static_cast<double>(rawVal);
    }
    return static_cast<double>(static_cast<int16_t>(rawVal & 0xFFFF));
}

long readSignedLittleEndianTraceValue(const unsigned char* raw, int valueBytes)
{
    if(valueBytes >= 4){
        const quint32 u =
                static_cast<quint32>(raw[0]) |
                (static_cast<quint32>(raw[1]) << 8) |
                (static_cast<quint32>(raw[2]) << 16) |
                (static_cast<quint32>(raw[3]) << 24);
        return static_cast<long>(static_cast<qint32>(u));
    }
    if(valueBytes == 2){
        const quint16 u =
                static_cast<quint16>(raw[0]) |
                (static_cast<quint16>(raw[1]) << 8);
        return static_cast<long>(static_cast<qint16>(u));
    }
    return static_cast<long>(static_cast<qint8>(raw[0]));
}

quint16 readUnsignedLittleEndianTraceWord(const unsigned char* raw)
{
    return static_cast<quint16>(raw[0]) |
            (static_cast<quint16>(raw[1]) << 8);
}

int decodeCia402StateMachine(quint16 statusWord)
{
    if((statusWord & 0x004FU) == 0x0000U){
        return 0; // Not ready to switch on.
    }
    if((statusWord & 0x004FU) == 0x0040U){
        return 1; // Switch on disabled.
    }
    if((statusWord & 0x006FU) == 0x0021U){
        return 2; // Ready to switch on.
    }
    if((statusWord & 0x006FU) == 0x0023U){
        return 3; // Switched on.
    }
    if((statusWord & 0x006FU) == 0x0027U){
        return 4; // Operation enabled.
    }
    if((statusWord & 0x006FU) == 0x0007U){
        return 5; // Quick stop active.
    }
    if((statusWord & 0x004FU) == 0x000FU){
        return 6; // Fault reaction active.
    }
    if((statusWord & 0x004FU) == 0x0008U){
        return 7; // Fault.
    }
    return -1;
}

quint32 readTraceFrameSequence(const unsigned char* raw)
{
    return static_cast<quint32>(raw[0]) |
            (static_cast<quint32>(raw[1]) << 8) |
            (static_cast<quint32>(raw[2]) << 16) |
            (static_cast<quint32>(raw[3]) << 24);
}

qint64 monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

qint64 wallClockNowUs()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

void updateAtomicMaximum(std::atomic<qint64>& target, qint64 value)
{
    qint64 observed = target.load(std::memory_order_relaxed);
    while(value > observed &&
          !target.compare_exchange_weak(observed,
                                        value,
                                        std::memory_order_relaxed,
                                        std::memory_order_relaxed)){
    }
}

constexpr qint64 kDiagnosticRawDefaultRetentionMs = 30 * 1000;
constexpr qint64 kDiagnosticRawTrimIntervalMs = 1000;
constexpr qint64 kDiagnosticRawDefaultSampleIntervalUs = 50 * 1000;
constexpr int kDiagnosticRawDefaultMaxSamples = 30000;
constexpr int kDiagnosticTraceFeedbackDefaultMaxSamples = 6000;
constexpr int kForceSensorTraceBaseCycleUs = 500;
constexpr int kForceSensorTraceMinPeriodUs = 500;
constexpr int kForceSensorTraceMaxPeriodUs = 1000;
constexpr std::size_t kMaxMotorTracePositionSamplesPerAxis = 8192;
constexpr std::size_t kMaxForceSensorTraceSamples = 8192;
constexpr qint64 kMotorTorqueTraceFreshTimeoutUs = 50 * 1000;
constexpr qint64 kMotorPositionTraceFreshTimeoutUs = 50 * 1000;
constexpr qint64 kForceSensorTraceDiagnosticsFreshTimeoutUs = 5 * 1000 * 1000;
constexpr qint64 kForceSensorTraceDiagnosticsPollIntervalUs = 5 * 1000 * 1000;
constexpr qint64 kMotorTracePositionWindowRetentionUs = 100 * 1000;
constexpr double kRuntimeTracePositionLimitGuardUnit = 0.25;
constexpr qint64 kRuntimeTracePositionRejectWarnIntervalUs = 1000 * 1000;
constexpr qint64 kRuntimeTraceLayoutRejectWarnIntervalUs = 1000 * 1000;
constexpr int kPvtPreStartTraceCatchUpTimeoutMs = 1000;
constexpr int kPvtPreStartTraceCatchUpPollMs = 1;
constexpr bool kEnablePvtHandshakeDiagnostics = false;
constexpr WORD kLeadshineEtherCatPort = 2;
constexpr WORD kLeadshineTorqueVelocityLimitIndex = 0x220B;
constexpr WORD kLeadshineTorqueVelocityLimitSubIndex = 0;
constexpr WORD kLeadshineTorqueVelocityLimitBitLength = 32;
constexpr WORD kLeadshineModeOfOperationIndex = 0x6060;
constexpr WORD kLeadshineModeOfOperationSubIndex = 0;
constexpr WORD kLeadshineModeOfOperationBitLength = 8;
constexpr WORD kLeadshineFollowingErrorActualIndex = 0x60F4;
constexpr WORD kLeadshineFollowingErrorActualSubIndex = 0;
constexpr WORD kLeadshineFollowingErrorActualBitLength = 32;
constexpr short kLeadshineTraceDataTypeCommandPosition = 5;
constexpr short kLeadshineTraceDataTypeActualPosition = 6;
constexpr short kLeadshineTraceDataTypeCommandVelocity = 3;
constexpr short kLeadshineTraceDataTypeActualVelocity = 4;
constexpr short kLeadshineTraceDataTypeFeedbackTorque = 8;
constexpr short kLeadshineTraceDataTypeGenericPdo = 19;
constexpr int kLeadshineStatusWordIndex = 0x6041;
constexpr int kLeadshineStatusWordSubIndex = 0;
// 雷赛内置 Trace 类型 3/4 的数据宽度由板卡根据类型自动匹配；旧工程
// RuntimeTraceSlaveReader 也固定向 dmc_trace_add_config_object() 传 0。
// 主机侧帧解析仍按返回的 4 字节有符号速度值处理。
constexpr short kLeadshineTraceAutomaticDataBytes = 0;
constexpr short kLeadshineTracePositionDataBytes = 4;
constexpr short kLeadshineTraceTorqueDataBytes = 4;
constexpr short kLeadshineTraceStatusWordDataBytes = 2;
constexpr int kCableMotorTraceLogicalAxisCount = 8;
constexpr int kDefaultEthercatBusCycleUs = 500;
constexpr int kRuntimeTraceTimestampFutureToleranceFrames = 4;
constexpr int kRuntimeTraceMaxDrainReads = 8;
constexpr int kRuntimeTraceMinimumBacklogDrainReads = 2;
constexpr qint64 kRuntimeTraceDrainBudgetUs = 4 * 1000;
constexpr quint32 kRuntimeTraceMaximumSequenceIncrement = 1000000U;
constexpr double kLeadshineTorqueVelocityLimitPulsesPerRev = 360000.0;
constexpr double kDefaultLeadshineRatedMotorTorqueNm = 45.0;
constexpr double kLeadshineTorqueRawPerRatedTorque = 1000.0;
constexpr double kLiteMotorTorqueCommandLimitNm = 40.0;
constexpr WORD kTorquePositionLimitDisabled = 0;
constexpr double kTorquePositionLimitValueUnused = 0.0;
constexpr WORD kTorqueAbsolutePositionMode = 1;
constexpr DWORD kLeadshinePvtTableCapacity = 5000;
constexpr int kPvtUploadAckRetryCount = 6;
constexpr int kPvtUploadAckRetryDelayMs = 5;
constexpr int kPvtStartAckTimeoutMs = 200;
constexpr int kPvtStartAckPollMs = 5;
constexpr bool kEnablePvtControlCycleDiagnostics =
        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;

double normalizedLeadshineRatedMotorTorqueNm(double ratedTorqueNm)
{
    if(std::isfinite(ratedTorqueNm) && ratedTorqueNm > 0.0){
        return ratedTorqueNm;
    }
    return kDefaultLeadshineRatedMotorTorqueNm;
}

int leadshineTorqueNmToRaw(double torqueNm, double ratedTorqueNm)
{
    // 这里只做 Nm<->额定转矩原始值的比例换算并保留符号。G302 的绳索
    // 正/负方向由上层控制策略处理：正电机力矩=放绳，负电机力矩=收绳。
    const double ratedTorque = normalizedLeadshineRatedMotorTorqueNm(ratedTorqueNm);
    const double rawTorque = torqueNm *
            kLeadshineTorqueRawPerRatedTorque /
            ratedTorque;
    long raw = std::lround(rawTorque);
    if(raw == 0 && torqueNm != 0.0){
        raw = torqueNm > 0.0 ? 1 : -1;
    }
    raw = std::clamp(raw,
                     static_cast<long>(std::numeric_limits<int>::min()),
                     static_cast<long>(std::numeric_limits<int>::max()));
    return static_cast<int>(raw);
}

double leadshineTorqueRawToNm(long rawValue, double ratedTorqueNm)
{
    // Trace 反馈保持控制器原始正负号，不能在硬件层乘机型方向符号。
    const double ratedTorque = normalizedLeadshineRatedMotorTorqueNm(ratedTorqueNm);
    return static_cast<double>(rawValue) *
            ratedTorque /
            kLeadshineTorqueRawPerRatedTorque;
}

template<typename Sample>
void trimRawHistory(QVector<Sample>& history, qint64 cutoffMs)
{
    int removeCount = 0;
    while(removeCount < history.size() && history.at(removeCount).wallClockMs < cutoffMs){
        ++removeCount;
    }
    if(removeCount > 0){
        history.erase(history.begin(), history.begin() + removeCount);
    }
}

template<typename Sample>
void trimRawHistoryToMaxSamples(QVector<Sample>& history, int maxSamples)
{
    if(maxSamples <= 0 || history.size() <= maxSamples){
        return;
    }
    const int removeCount = history.size() - maxSamples;
    history.erase(history.begin(), history.begin() + removeCount);
}

template<typename Sample>
void trimRawHistoryForMode(QVector<Sample>& history,
                           qint64 nowMs,
                           bool fullRecording,
                           int defaultMaxSamples)
{
    if(fullRecording){
        return;
    }
    trimRawHistory(history, nowMs - kDiagnosticRawDefaultRetentionMs);
    trimRawHistoryToMaxSamples(history, defaultMaxSamples);
}

bool shouldAppendDiagnosticRawSample(bool fullRecording,
                                     qint64 sampleWallClockUs,
                                     qint64& lastAppendUs)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        return false;
    }
    if(fullRecording){
        return true;
    }
    if(sampleWallClockUs <= 0){
        return false;
    }
    if(lastAppendUs <= 0 ||
            sampleWallClockUs - lastAppendUs >= kDiagnosticRawDefaultSampleIntervalUs){
        lastAppendUs = sampleWallClockUs;
        return true;
    }
    return false;
}
}

HardwareInterface::HardwareInterface() = default;

HardwareInterface::HardwareInterface(double _threadCtrlCycleMs)
    : threadCtrlCycleMs(_threadCtrlCycleMs) {
}

HardwareInterface::~HardwareInterface()
{
    stopSessionEncoderUnitSampling();
}

void HardwareInterface::setMotorEnableQueryTimingEnabled(bool enabled)
{
    if(enabled){
        motorEnableQueryCount.store(0, std::memory_order_relaxed);
        motorEnableQueryTotalQueueWaitUs.store(0, std::memory_order_relaxed);
        motorEnableQueryLatestQueueWaitUs.store(0, std::memory_order_relaxed);
        motorEnableQueryMaximumQueueWaitUs.store(0, std::memory_order_relaxed);
        motorEnableQueryTotalApiDurationUs.store(0, std::memory_order_relaxed);
        motorEnableQueryLatestApiDurationUs.store(0, std::memory_order_relaxed);
        motorEnableQueryMaximumApiDurationUs.store(0, std::memory_order_relaxed);
        motorEnableQueryTotalCallDurationUs.store(0, std::memory_order_relaxed);
        motorEnableQueryLatestCallDurationUs.store(0, std::memory_order_relaxed);
        motorEnableQueryMaximumCallDurationUs.store(0, std::memory_order_relaxed);
    }
    motorEnableQueryTimingEnabled.store(enabled, std::memory_order_release);
}

HardwareInterface::MotorEnableQueryTimingSnapshot
HardwareInterface::motorEnableQueryTimingSnapshot() const
{
    MotorEnableQueryTimingSnapshot snapshot;
    snapshot.queryCount = motorEnableQueryCount.load(std::memory_order_relaxed);
    snapshot.totalQueueWaitUs =
            motorEnableQueryTotalQueueWaitUs.load(std::memory_order_relaxed);
    snapshot.latestQueueWaitUs =
            motorEnableQueryLatestQueueWaitUs.load(std::memory_order_relaxed);
    snapshot.maximumQueueWaitUs =
            motorEnableQueryMaximumQueueWaitUs.load(std::memory_order_relaxed);
    snapshot.totalApiDurationUs =
            motorEnableQueryTotalApiDurationUs.load(std::memory_order_relaxed);
    snapshot.latestApiDurationUs =
            motorEnableQueryLatestApiDurationUs.load(std::memory_order_relaxed);
    snapshot.maximumApiDurationUs =
            motorEnableQueryMaximumApiDurationUs.load(std::memory_order_relaxed);
    snapshot.totalCallDurationUs =
            motorEnableQueryTotalCallDurationUs.load(std::memory_order_relaxed);
    snapshot.latestCallDurationUs =
            motorEnableQueryLatestCallDurationUs.load(std::memory_order_relaxed);
    snapshot.maximumCallDurationUs =
            motorEnableQueryMaximumCallDurationUs.load(std::memory_order_relaxed);
    return snapshot;
}

void HardwareInterface::setDiagnosticRawHistoryFullRecordingEnabled(bool enabled)
{
    enabled = enabled && RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;
    QMutexLocker locker(&diagnosticsMutex);
    if(diagnosticRawHistoryFullRecordingEnabled == enabled){
        return;
    }
    diagnosticRawHistoryFullRecordingEnabled = enabled;
    if(!enabled){
        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
        trimRawHistoryForMode(communicationRawHistory,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
        trimRawHistoryForMode(motorCommandRawHistory,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
        trimRawHistoryForMode(motorPositionRawSamples,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
        trimRawHistoryForMode(motorEncoderRawSamples,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
        trimRawHistoryForMode(motorTraceFeedbackRawSamples,
                              nowMs,
                              false,
                              kDiagnosticTraceFeedbackDefaultMaxSamples);
        trimRawHistoryForMode(runtimeTraceFetchTimingSamples,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
        trimRawHistoryForMode(pvtTableUploadTimingSamples,
                              nowMs,
                              false,
                              kDiagnosticRawDefaultMaxSamples);
    }
}

int HardwareInterface::resolveLeadshineAxisIndex(int logicalIndex) const
{
    if(logicalIndex < 0 ||
       logicalIndex >= static_cast<int>(motorComType.size()) ||
       logicalIndex >= static_cast<int>(motorIdVec.size()) ||
       motorComType[logicalIndex] != COM_EC_LS){
        return -1;
    }
    return static_cast<int>(motorIdVec[logicalIndex]);
}

double HardwareInterface::resolveLeadshineAxisEquiv(int logicalIndex) const
{
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorEquivVec.size())){
        return 0.0;
    }
    return motorEquivVec[logicalIndex];
}

QString HardwareInterface::axisDisplayName(int logicalIndex) const
{
    QString displayName;
    if(logicalIndex >= 0 && logicalIndex < 8){
        displayName = QStringLiteral("绳索电机%1").arg(logicalIndex + 1);
    }
    else if(logicalIndex >= 8 && logicalIndex < 12){
        displayName = QStringLiteral("直线模组电机%1").arg(logicalIndex - 7);
    }
    else{
        displayName = QStringLiteral("电机%1").arg(logicalIndex + 1);
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if(hardwareAxis >= 0){
        return QStringLiteral("%1(控制卡轴%2)")
                .arg(displayName)
                .arg(hardwareAxis);
    }
    return displayName;
}

bool HardwareInterface::hasValidMotorSoftwareLimit(int logicalIndex) const
{
    return logicalIndex >= 0 &&
            logicalIndex < static_cast<int>(motorSoftwareMinPos.size()) &&
            logicalIndex < static_cast<int>(motorSoftwareMaxPos.size()) &&
            std::isfinite(motorSoftwareMinPos[logicalIndex]) &&
            std::isfinite(motorSoftwareMaxPos[logicalIndex]) &&
            motorSoftwareMaxPos[logicalIndex] > motorSoftwareMinPos[logicalIndex];
}

double HardwareInterface::relativeMotorPosition(int logicalIndex, double absolutePosition) const
{
    double relativePosition = absolutePosition;
    if(logicalIndex >= 0 && logicalIndex < static_cast<int>(motorHomePos.size())){
        relativePosition -= motorHomePos[logicalIndex];
    }
    return relativePosition;
}

bool HardwareInterface::hasValidMotorSafetyHome(int logicalIndex) const
{
    return hasValidMotorSessionSafetyTraceHome(logicalIndex) ||
            (logicalIndex >= 0 &&
             logicalIndex < static_cast<int>(motorSafetyHomeTraceCommandRawPulse.size()) &&
             motorSafetyHomeTraceCommandRawPulse[logicalIndex] != 0);
}

bool HardwareInterface::hasValidMotorSessionSafetyTraceHome(int logicalIndex) const
{
    return logicalIndex >= 0 &&
            logicalIndex < static_cast<int>(motorSessionSafetyHomeTraceRawPulse.size()) &&
            logicalIndex < static_cast<int>(motorSessionSafetyHomeTraceValid.size()) &&
            logicalIndex < static_cast<int>(motorSessionSafetyHomeTraceUsesFeedback.size()) &&
            motorSessionSafetyHomeTraceValid[logicalIndex];
}

bool HardwareInterface::hasValidMotorSafetyEncoderHome(int logicalIndex) const
{
    return logicalIndex >= 0 &&
            logicalIndex < static_cast<int>(motorSafetyHomeEncoderUnit.size()) &&
            std::isfinite(motorSafetyHomeEncoderUnit[logicalIndex]);
}

double HardwareInterface::motorSafetyHomeUnit(int logicalIndex) const
{
    if(hasValidMotorSafetyEncoderHome(logicalIndex)){
        return motorSafetyHomeEncoderUnit[logicalIndex];
    }
    if(logicalIndex >= 0 && logicalIndex < static_cast<int>(motorHomePos.size())){
        return motorHomePos[logicalIndex];
    }
    return std::numeric_limits<double>::quiet_NaN();
}

double HardwareInterface::safetyRelativeMotorPosition(int logicalIndex, double absolutePosition) const
{
    const double safetyHome = motorSafetyHomeUnit(logicalIndex);
    if(std::isfinite(safetyHome)){
        return absolutePosition - safetyHome;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool HardwareInterface::motorSafetyRelativeFromTraceFrame(
        int logicalIndex,
        const MotorTracePositionWindowFrame& frame,
        double& relativePosition,
        MotorSafetyRelativePositionSource* source) const
{
    relativePosition = std::numeric_limits<double>::quiet_NaN();
    if(source){
        *source = MotorSafetyRelativePositionSource::Invalid;
    }
    if(!hasValidMotorSafetyHome(logicalIndex)){
        return false;
    }

    auto readRaw = [&](const std::vector<qint64>& rawPulse,
                       const std::vector<bool>& valid,
                       qint64& rawValue) -> bool {
        if(logicalIndex < static_cast<int>(rawPulse.size()) &&
                logicalIndex < static_cast<int>(valid.size()) &&
                valid[logicalIndex]){
            rawValue = rawPulse[logicalIndex];
            return true;
        }
        return false;
    };

    const bool sessionHome = hasValidMotorSessionSafetyTraceHome(logicalIndex);
    if(sessionHome){
        const qint64 nowUs = monotonicNowUs();
        const qint64 futureToleranceUs = std::max<qint64>(
                    2 * 1000,
                    static_cast<qint64>(runtimeTraceSamplePeriodUs) *
                        kRuntimeTraceTimestampFutureToleranceFrames);
        const bool frameTimestampFresh = frame.monotonicUs > 0 &&
                nowUs + futureToleranceUs >= frame.monotonicUs &&
                (nowUs < frame.monotonicUs ||
                 nowUs - frame.monotonicUs <= kMotorPositionTraceFreshTimeoutUs);
        if(!runtimeTraceConfigReadbackValid ||
                runtimeTraceNewestFrameAgeUs < 0 ||
                runtimeTraceNewestFrameAgeUs > kMotorPositionTraceFreshTimeoutUs ||
                !frameTimestampFresh){
            return false;
        }
    }

    qint64 currentRaw = 0;
    bool ok = false;
    qint64 homeRaw = 0;
    if(sessionHome){
        const bool useFeedback = motorSessionSafetyHomeTraceUsesFeedback[logicalIndex];
        ok = useFeedback ?
                    readRaw(frame.feedbackRawPulse, frame.feedbackValid, currentRaw) :
                    readRaw(frame.commandRawPulse, frame.commandValid, currentRaw);
        homeRaw = motorSessionSafetyHomeTraceRawPulse[logicalIndex];
        if(ok && source){
            *source = useFeedback ?
                        MotorSafetyRelativePositionSource::TraceFeedbackSessionHome :
                        MotorSafetyRelativePositionSource::TraceCommandSessionHome;
        }
    }
    else{
        // 持久化零位明确保存的是 command 原始脉冲，因此运行时也只允许
        // 使用 command Trace，不能在 command/feedback 之间自动切换。
        ok = readRaw(frame.commandRawPulse, frame.commandValid, currentRaw);
        homeRaw = motorSafetyHomeTraceCommandRawPulse[logicalIndex];
        if(ok && source){
            *source = MotorSafetyRelativePositionSource::TraceCommandPersistentHome;
        }
    }
    if(!ok || (!sessionHome && currentRaw == 0)){
        if(source){
            *source = MotorSafetyRelativePositionSource::Invalid;
        }
        return false;
    }

    const double axisEquiv = resolveLeadshineAxisEquiv(logicalIndex);
    if(!std::isfinite(axisEquiv) || axisEquiv <= 0.0){
        if(source){
            *source = MotorSafetyRelativePositionSource::Invalid;
        }
        return false;
    }
    const qint64 rawDelta = currentRaw - homeRaw;
    relativePosition = static_cast<double>(rawDelta) / axisEquiv;
    if(!std::isfinite(relativePosition)){
        if(source){
            *source = MotorSafetyRelativePositionSource::Invalid;
        }
        return false;
    }
    return true;
}

bool HardwareInterface::safetyRelativeMotorTargetFromAbsoluteDirect(
        int logicalIndex,
        double absolutePosition,
        double& relativePosition)
{
    relativePosition = std::numeric_limits<double>::quiet_NaN();

    double currentAbsolutePosition = 0.0;
    double currentSafetyRelativePosition = 0.0;
    if(readMotorPositionUnitDirect(logicalIndex, currentAbsolutePosition, false) &&
            readMotorSafetyRelativePositionDirect(logicalIndex, currentSafetyRelativePosition) &&
            std::isfinite(currentAbsolutePosition) &&
            std::isfinite(currentSafetyRelativePosition) &&
            std::isfinite(absolutePosition)){
        relativePosition =
                currentSafetyRelativePosition + (absolutePosition - currentAbsolutePosition);
        return std::isfinite(relativePosition);
    }

    // G302 会话零点建立后，安全位置只能来自锁存时选定的同一 Trace
    // 通道；Trace 失效时不允许退回另一种位置量继续下发命令。
    if(hasValidMotorSessionSafetyTraceHome(logicalIndex)){
        return false;
    }

    relativePosition = safetyRelativeMotorPosition(logicalIndex, absolutePosition);
    return std::isfinite(relativePosition);
}

double HardwareInterface::tracePulseToMotorUnit(int logicalIndex, qint64 rawPulse) const
{
    const double axisEquiv = resolveLeadshineAxisEquiv(logicalIndex);
    if(std::isfinite(axisEquiv) && axisEquiv > 0.0){
        return static_cast<double>(rawPulse) / axisEquiv;
    }
    return static_cast<double>(rawPulse);
}

bool HardwareInterface::readMotorPositionUnitDirect(int logicalIndex, double& position, bool updateCache)
{
    position = 0.0;
    if(!isConnectLS ||
            logicalIndex < 0 ||
            logicalIndex >= static_cast<int>(motorComType.size()) ||
            motorComType[logicalIndex] != COM_EC_LS){
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if(hardwareAxis < 0){
        return false;
    }

    double unitPosition = 0.0;
    const short ret = dmc_get_position_unit(0, static_cast<WORD>(hardwareAxis), &unitPosition);
    recordCommunicationEvent(false, QStringLiteral("dmc_get_position_unit"));
    if(ret != 0 || !std::isfinite(unitPosition)){
        return false;
    }

    if(updateCache){
        if(motorCurPos.size() != motorIdVec.size()){
            motorCurPos.assign(motorIdVec.size(), 0.0);
        }
        if(logicalIndex < static_cast<int>(motorCurPos.size())){
            motorCurPos[logicalIndex] = unitPosition;
        }
    }
    position = unitPosition;
    return true;
}

bool HardwareInterface::readMotorEncoderUnitDirect(int logicalIndex, double& position)
{
    position = 0.0;
    if(!isConnectLS ||
            logicalIndex < 0 ||
            logicalIndex >= static_cast<int>(motorComType.size()) ||
            motorComType[logicalIndex] != COM_EC_LS){
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if(hardwareAxis < 0){
        return false;
    }

    double unitPosition = 0.0;
    const short ret = dmc_get_encoder_unit(0, static_cast<WORD>(hardwareAxis), &unitPosition);
    recordCommunicationEvent(false, QStringLiteral("dmc_get_encoder_unit"));
    if(ret != 0 || !std::isfinite(unitPosition)){
        return false;
    }

    position = unitPosition;
    return true;
}

bool HardwareInterface::readMotorSafetyRelativePositionDirect(int logicalIndex, double& relativePosition)
{
    relativePosition = std::numeric_limits<double>::quiet_NaN();

    const bool sessionTraceHome =
            hasValidMotorSessionSafetyTraceHome(logicalIndex);
    if(hasValidMotorSafetyHome(logicalIndex)){
        const int frameCount = readRuntimeTraceCached();
        if((frameCount >= 0 || runtimeTraceEverRead) &&
                latestMotorTracePositionFrameValid &&
                motorSafetyRelativeFromTraceFrame(logicalIndex,
                                                  latestMotorTracePositionFrame,
                                                  relativePosition)){
            return true;
        }
    }

    if(sessionTraceHome){
        return false;
    }

    double encoderPosition = 0.0;
    if(hasValidMotorSafetyEncoderHome(logicalIndex) &&
            readMotorEncoderUnitDirect(logicalIndex, encoderPosition)){
        relativePosition = encoderPosition - motorSafetyHomeEncoderUnit[logicalIndex];
        return std::isfinite(relativePosition);
    }

    double unitPosition = 0.0;
    if(readMotorPositionUnitDirect(logicalIndex, unitPosition, false)){
        relativePosition = relativeMotorPosition(logicalIndex, unitPosition);
        return std::isfinite(relativePosition);
    }

    return false;
}

void HardwareInterface::resetMotorTracePositionOffsets()
{
    motorCommandTraceOffsetUnit.assign(motorIdVec.size(), 0.0);
    motorActualTraceOffsetUnit.assign(motorIdVec.size(), 0.0);
    motorCommandTraceOffsetValid.assign(motorIdVec.size(), false);
    motorActualTraceOffsetValid.assign(motorIdVec.size(), false);
    motorTracePositionSampleQueues.assign(motorIdVec.size(),
                                          std::deque<MotorTracePositionSample>());
}

void HardwareInterface::ensureMotorTracePositionOffsetStorage()
{
    const std::size_t axisCount = motorIdVec.size();
    if(motorCommandTraceOffsetUnit.size() != axisCount ||
            motorActualTraceOffsetUnit.size() != axisCount ||
            motorCommandTraceOffsetValid.size() != axisCount ||
            motorActualTraceOffsetValid.size() != axisCount){
        resetMotorTracePositionOffsets();
    }
}

bool HardwareInterface::ensureMotorTracePositionOffsets(int logicalIndex)
{
    ensureMotorTracePositionOffsetStorage();
    if(logicalIndex < 0 ||
            logicalIndex >= static_cast<int>(motorIdVec.size()) ||
            logicalIndex >= static_cast<int>(motorComType.size()) ||
            logicalIndex >= static_cast<int>(motorCommandPos.size()) ||
            logicalIndex >= static_cast<int>(motorTraceActualPos.size()) ||
            motorComType[logicalIndex] != COM_EC_LS ||
            !std::isfinite(motorCommandPos[logicalIndex]) ||
            !std::isfinite(motorTraceActualPos[logicalIndex])){
        return false;
    }

    if(motorCommandTraceOffsetValid[logicalIndex] &&
            motorActualTraceOffsetValid[logicalIndex]){
        return true;
    }

    double directUnitPosition = 0.0;
    if(!readMotorPositionUnitDirect(logicalIndex, directUnitPosition, false)){
        return false;
    }

    if(!motorCommandTraceOffsetValid[logicalIndex]){
        motorCommandTraceOffsetUnit[logicalIndex] =
                motorCommandPos[logicalIndex] - directUnitPosition;
        motorCommandTraceOffsetValid[logicalIndex] = true;
    }
    if(!motorActualTraceOffsetValid[logicalIndex]){
        motorActualTraceOffsetUnit[logicalIndex] =
                motorTraceActualPos[logicalIndex] - directUnitPosition;
        motorActualTraceOffsetValid[logicalIndex] = true;
    }
    if(logicalIndex < static_cast<int>(motorTracePositionSampleQueues.size())){
        motorTracePositionSampleQueues[logicalIndex].clear();
    }
    return true;
}

double HardwareInterface::traceAlignedRelativePosition(
        int logicalIndex,
        double traceUnitPosition,
        const std::vector<double>& traceOffsetUnit,
        const std::vector<bool>& traceOffsetValid) const
{
    double alignedAbsolutePosition = traceUnitPosition;
    if(logicalIndex >= 0 &&
            logicalIndex < static_cast<int>(traceOffsetUnit.size()) &&
            logicalIndex < static_cast<int>(traceOffsetValid.size()) &&
            traceOffsetValid[logicalIndex]){
        alignedAbsolutePosition -= traceOffsetUnit[logicalIndex];
    }
    return relativeMotorPosition(logicalIndex, alignedAbsolutePosition);
}

bool HardwareInterface::validateRelativeMotorSoftwareLimit(int logicalIndex,
                                                           double relativePosition,
                                                           const QString& commandName,
                                                           QString* errorMessage) const
{
    if(errorMessage){
        errorMessage->clear();
    }
    if(!hasValidMotorSoftwareLimit(logicalIndex)){
        return true;
    }

    const double minPos = motorSoftwareMinPos[logicalIndex];
    const double maxPos = motorSoftwareMaxPos[logicalIndex];
    if(!std::isfinite(relativePosition) ||
            relativePosition < minPos ||
            relativePosition > maxPos){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1不会下发，%2目标位置%3会导致电机超出软件位置限位[%4, %5]")
                    .arg(commandName)
                    .arg(axisDisplayName(logicalIndex))
                    .arg(relativePosition, 0, 'f', 6)
                    .arg(minPos, 0, 'f', 6)
                    .arg(maxPos, 0, 'f', 6);
        }
        return false;
    }

    return true;
}

bool HardwareInterface::validateSafetyRelativeMotorSoftwareLimit(int logicalIndex,
                                                                 double relativePosition,
                                                                 const QString& commandName,
                                                                 QString* errorMessage) const
{
    if(errorMessage){
        errorMessage->clear();
    }
    if(!hasValidMotorSoftwareLimit(logicalIndex)){
        return true;
    }

    const double minPos = motorSoftwareMinPos[logicalIndex];
    const double maxPos = motorSoftwareMaxPos[logicalIndex];
    if(!std::isfinite(relativePosition) ||
            relativePosition < minPos ||
            relativePosition > maxPos){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1不会下发，%2安全相对位置%3会导致电机超出软件位置限位[%4, %5]")
                    .arg(commandName)
                    .arg(axisDisplayName(logicalIndex))
                    .arg(relativePosition, 0, 'f', 6)
                    .arg(minPos, 0, 'f', 6)
                    .arg(maxPos, 0, 'f', 6);
        }
        return false;
    }

    return true;
}

bool HardwareInterface::validateCurrentMotorSafetyLimitForAutomaticMotion(
        int logicalIndex,
        const QString& commandName,
        QString* errorMessage)
{
    if(errorMessage){
        errorMessage->clear();
    }
    if(!hasValidMotorSoftwareLimit(logicalIndex)){
        return true;
    }

    double currentSafetyRelative = std::numeric_limits<double>::quiet_NaN();
    if(!readMotorSafetyRelativePositionDirect(logicalIndex, currentSafetyRelative)){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1不会下发，%2无法读取安全相对位置")
                    .arg(commandName)
                    .arg(axisDisplayName(logicalIndex));
        }
        return false;
    }

    return validateSafetyRelativeMotorSoftwareLimit(logicalIndex,
                                                    currentSafetyRelative,
                                                    commandName,
                                                    errorMessage);
}

bool HardwareInterface::validateAbsoluteMotorSoftwareLimit(int logicalIndex,
                                                           double absolutePosition,
                                                           const QString& commandName,
                                                           QString* errorMessage)
{
    double safetyRelativeTarget = std::numeric_limits<double>::quiet_NaN();
    safetyRelativeMotorTargetFromAbsoluteDirect(logicalIndex,
                                                absolutePosition,
                                                safetyRelativeTarget);
    return validateSafetyRelativeMotorSoftwareLimit(
                logicalIndex,
                safetyRelativeTarget,
                commandName,
                errorMessage);
}

bool HardwareInterface::validateVelocityMotorSoftwareLimit(int logicalIndex,
                                                           double currentAbsolutePosition,
                                                           double velocity,
                                                           const QString& commandName,
                                                           QString* errorMessage)
{
    if(errorMessage){
        errorMessage->clear();
    }
    if(!hasValidMotorSoftwareLimit(logicalIndex) || std::fabs(velocity) <= 1e-12){
        return true;
    }

    double relativePosition = std::numeric_limits<double>::quiet_NaN();
    if(!readMotorSafetyRelativePositionDirect(logicalIndex, relativePosition)){
        relativePosition = safetyRelativeMotorPosition(logicalIndex, currentAbsolutePosition);
    }
    return validateVelocityMotorSoftwareLimitFromSnapshot(logicalIndex,
                                                          relativePosition,
                                                          velocity,
                                                          commandName,
                                                          errorMessage);
}

bool HardwareInterface::validateVelocityMotorSoftwareLimitFromSnapshot(
        int logicalIndex,
        double relativePosition,
        double velocity,
        const QString& commandName,
        QString* errorMessage) const
{
    if(errorMessage){
        errorMessage->clear();
    }
    if(!hasValidMotorSoftwareLimit(logicalIndex) || std::fabs(velocity) <= 1e-12){
        return true;
    }
    const double minPos = motorSoftwareMinPos[logicalIndex];
    const double maxPos = motorSoftwareMaxPos[logicalIndex];
    const bool outsideLimit = relativePosition > maxPos || relativePosition < minPos;
    const bool movingPastUpperLimit = velocity > 0.0 && relativePosition >= maxPos;
    const bool movingPastLowerLimit = velocity < 0.0 && relativePosition <= minPos;
    if(!std::isfinite(relativePosition) ||
            !std::isfinite(velocity) ||
            outsideLimit ||
            movingPastUpperLimit ||
            movingPastLowerLimit){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1不会下发，%2当前安全相对位置%3不允许按速度%4运动，软件位置限位[%5, %6]")
                    .arg(commandName)
                    .arg(axisDisplayName(logicalIndex))
                    .arg(relativePosition, 0, 'f', 6)
                    .arg(velocity, 0, 'f', 6)
                    .arg(minPos, 0, 'f', 6)
                    .arg(maxPos, 0, 'f', 6);
        }
        return false;
    }

    return true;
}

void HardwareInterface::recordCommunicationEvent(bool motorCommandEvent,
                                                 const QString& apiEvent)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        return;
    }
    const qint64 nowUs = monotonicNowUs();
    const qint64 nowWallClockMs = QDateTime::currentMSecsSinceEpoch();
    const QString eventName = apiEvent.isEmpty() ?
                QStringLiteral("未标注雷赛硬件API事件") :
                apiEvent;
    QMutexLocker locker(&diagnosticsMutex);
    diagnostics.communicationEventCount++;
    if(lastCommunicationEventUs > 0){
        const qint64 dtUs = std::max<qint64>(0, nowUs - lastCommunicationEventUs);
        diagnostics.communicationIntervalCount++;
        diagnostics.communicationIntervalSumUs += dtUs;
        diagnostics.latestCommunicationIntervalUs = dtUs;
        if(shouldAppendDiagnosticRawSample(diagnosticRawHistoryFullRecordingEnabled,
                                           nowWallClockMs * 1000,
                                           lastCommunicationRawHistoryAppendUs)){
            communicationRawHistory.append({nowWallClockMs, dtUs, eventName});
            if(nowWallClockMs - lastCommunicationHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistoryForMode(communicationRawHistory,
                                      nowWallClockMs,
                                      diagnosticRawHistoryFullRecordingEnabled,
                                      kDiagnosticRawDefaultMaxSamples);
                lastCommunicationHistoryTrimMs = nowWallClockMs;
            }
        }
    }
    lastCommunicationEventUs = nowUs;

    if(motorCommandEvent){
        diagnostics.motorCommandEventCount++;
        if(lastMotorCommandEventUs > 0){
            const qint64 dtUs = std::max<qint64>(0, nowUs - lastMotorCommandEventUs);
            diagnostics.motorCommandIntervalCount++;
            diagnostics.motorCommandIntervalSumUs += dtUs;
            diagnostics.latestMotorCommandIntervalUs = dtUs;
            if(shouldAppendDiagnosticRawSample(diagnosticRawHistoryFullRecordingEnabled,
                                               nowWallClockMs * 1000,
                                               lastMotorCommandRawHistoryAppendUs)){
                motorCommandRawHistory.append({nowWallClockMs, dtUs, eventName});
                if(nowWallClockMs - lastMotorCommandHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                    trimRawHistoryForMode(motorCommandRawHistory,
                                          nowWallClockMs,
                                          diagnosticRawHistoryFullRecordingEnabled,
                                          kDiagnosticRawDefaultMaxSamples);
                    lastMotorCommandHistoryTrimMs = nowWallClockMs;
                }
            }
        }
        lastMotorCommandEventUs = nowUs;
    }
}

void HardwareInterface::recordMotorPositionRawSample(const std::vector<double>& positions, const QString& source)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        return;
    }
    const qint64 nowUs = monotonicNowUs();
    const qint64 nowWallClockMs = QDateTime::currentMSecsSinceEpoch();
    QMutexLocker locker(&diagnosticsMutex);
    const qint64 intervalUs = lastMotorPositionReadUs > 0 ?
                std::max<qint64>(0, nowUs - lastMotorPositionReadUs) :
                0;
    lastMotorPositionReadUs = nowUs;
    if(shouldAppendDiagnosticRawSample(diagnosticRawHistoryFullRecordingEnabled,
                                       nowWallClockMs * 1000,
                                       lastMotorPositionRawHistoryAppendUs)){
        motorPositionRawSamples.append({nowWallClockMs, nowUs, intervalUs, source, positions});
        if(nowWallClockMs - lastMotorPositionRawHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
            trimRawHistoryForMode(motorPositionRawSamples,
                                  nowWallClockMs,
                                  diagnosticRawHistoryFullRecordingEnabled,
                                  kDiagnosticRawDefaultMaxSamples);
            lastMotorPositionRawHistoryTrimMs = nowWallClockMs;
        }
    }
}

void HardwareInterface::recordMotorEncoderRawSample(const std::vector<double>& positions,
                                                    const QString& source,
                                                    qint64 durationUs,
                                                    qint64 sampleWallClockUs,
                                                    qint64 sampleMonotonicUs)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        return;
    }
    const qint64 nowUs = sampleMonotonicUs > 0 ? sampleMonotonicUs : monotonicNowUs();
    const qint64 nowWallClockUs = sampleWallClockUs > 0 ? sampleWallClockUs : wallClockNowUs();
    const qint64 nowWallClockMs = nowWallClockUs / 1000;
    QMutexLocker locker(&diagnosticsMutex);
    const qint64 intervalUs = lastMotorEncoderReadUs > 0 ?
                std::max<qint64>(0, nowUs - lastMotorEncoderReadUs) :
                0;
    lastMotorEncoderReadUs = nowUs;
    if(shouldAppendDiagnosticRawSample(diagnosticRawHistoryFullRecordingEnabled,
                                       nowWallClockUs,
                                       lastMotorEncoderRawHistoryAppendUs)){
        MotorPositionRawSample sample;
        sample.wallClockMs = nowWallClockMs;
        sample.monotonicUs = nowUs;
        sample.intervalUs = intervalUs;
        sample.source = source;
        sample.positions = positions;
        sample.wallClockUs = nowWallClockUs;
        sample.durationUs = std::max<qint64>(0, durationUs);
        motorEncoderRawSamples.append(sample);
        if(nowWallClockMs - lastMotorEncoderRawHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
            trimRawHistoryForMode(motorEncoderRawSamples,
                                  nowWallClockMs,
                                  diagnosticRawHistoryFullRecordingEnabled,
                                  kDiagnosticRawDefaultMaxSamples);
            lastMotorEncoderRawHistoryTrimMs = nowWallClockMs;
        }
    }
}

void HardwareInterface::recordMotorTraceFeedbackRawSample(
        qint64 frameWallClockUs,
        qint64 frameMonotonicUs,
        quint32 frameSequence,
        bool frameSequenceValid,
        const std::vector<qint64>& feedbackRawPulse,
        const std::vector<bool>& feedbackValid)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        return;
    }
    if(frameWallClockUs <= 0 ||
            frameMonotonicUs <= 0 ||
            feedbackRawPulse.empty() ||
            feedbackValid.empty()){
        return;
    }

    bool hasValidFeedback = false;
    for(bool valid : feedbackValid){
        if(valid){
            hasValidFeedback = true;
            break;
        }
    }
    if(!hasValidFeedback){
        return;
    }

    QMutexLocker locker(&diagnosticsMutex);
    if(lastMotorTraceFeedbackRawUs > 0 &&
            frameMonotonicUs <= lastMotorTraceFeedbackRawUs){
        return;
    }
    const qint64 intervalUs = lastMotorTraceFeedbackRawUs > 0 ?
                std::max<qint64>(0, frameMonotonicUs - lastMotorTraceFeedbackRawUs) :
                0;
    lastMotorTraceFeedbackRawUs = frameMonotonicUs;

    MotorTraceFeedbackRawSample sample;
    sample.wallClockMs = frameWallClockUs / 1000;
    sample.wallClockUs = frameWallClockUs;
    sample.monotonicUs = frameMonotonicUs;
    sample.intervalUs = intervalUs;
    sample.frameSequence = frameSequence;
    sample.frameSequenceValid = frameSequenceValid;
    sample.feedbackRawPulse = feedbackRawPulse;
    sample.feedbackValid = feedbackValid;
    if(shouldAppendDiagnosticRawSample(diagnosticRawHistoryFullRecordingEnabled,
                                       sample.wallClockUs,
                                       lastMotorTraceFeedbackRawHistoryAppendUs)){
        motorTraceFeedbackRawSamples.append(sample);
        if(sample.wallClockMs - lastMotorTraceFeedbackRawHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
            trimRawHistoryForMode(motorTraceFeedbackRawSamples,
                                  sample.wallClockMs,
                                  diagnosticRawHistoryFullRecordingEnabled,
                                  kDiagnosticTraceFeedbackDefaultMaxSamples);
            lastMotorTraceFeedbackRawHistoryTrimMs = sample.wallClockMs;
        }
    }
}

void HardwareInterface::recordRuntimeTraceFetchTimingSample(
        qint64 readEndWallClockUs,
        qint64 readEndMonotonicUs,
        qint64 apiDurationUs,
        int actualReadLength,
        int frameBytes,
        int frameCount,
        int requestedFrameCount,
        int fifoValidBefore,
        int fifoValidAfter,
        int fifoFreeAfter,
        int estimatedProducedFrameCount,
        int traceSamplePeriodUs,
        qint64 newestFrameAgeUs,
        bool latestOnly,
        bool fifoCaughtUp,
        bool timingReliable,
        bool traceLost,
        bool frameSequenceValid,
        const std::vector<quint32>& frameSequences)
{
    if(!RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        return;
    }
    if(readEndWallClockUs <= 0 ||
            readEndMonotonicUs <= 0 ||
            frameCount <= 0 ||
            frameBytes <= 0){
        return;
    }

    RuntimeTraceFetchTimingSample sample;
    sample.wallClockMs = readEndWallClockUs / 1000;
    sample.wallClockUs = readEndWallClockUs;
    sample.monotonicUs = readEndMonotonicUs;
    sample.apiDurationUs = std::max<qint64>(0, apiDurationUs);
    sample.actualReadLength = actualReadLength;
    sample.frameBytes = frameBytes;
    sample.frameCount = frameCount;
    sample.requestedFrameCount = requestedFrameCount;
    sample.fifoValidBefore = std::max(0, fifoValidBefore);
    sample.fifoValidAfter = std::max(0, fifoValidAfter);
    sample.fifoFreeAfter = std::max(0, fifoFreeAfter);
    sample.estimatedProducedFrameCount = std::max(0, estimatedProducedFrameCount);
    sample.traceSamplePeriodUs = std::max(1, traceSamplePeriodUs);
    sample.newestFrameAgeUs = newestFrameAgeUs;
    sample.latestOnly = latestOnly;
    sample.fifoCaughtUp = fifoCaughtUp;
    sample.timingReliable = timingReliable;
    sample.traceLost = traceLost;
    sample.frameSequences = frameSequences;
    sample.frameSequenceValid =
            frameSequenceValid &&
            static_cast<int>(frameSequences.size()) == frameCount &&
            !frameSequences.empty();
    if(sample.frameSequenceValid){
        sample.firstFrameSequence = frameSequences.front();
        sample.lastFrameSequence = frameSequences.back();
    }

    QMutexLocker locker(&diagnosticsMutex);
    sample.intervalUs = lastRuntimeTraceFetchTimingUs > 0 ?
                std::max<qint64>(0, readEndMonotonicUs - lastRuntimeTraceFetchTimingUs) :
                0;
    lastRuntimeTraceFetchTimingUs = readEndMonotonicUs;
    if(shouldAppendDiagnosticRawSample(diagnosticRawHistoryFullRecordingEnabled,
                                       sample.wallClockUs,
                                       lastRuntimeTraceFetchTimingHistoryAppendUs)){
        runtimeTraceFetchTimingSamples.append(sample);
        if(sample.wallClockMs - lastRuntimeTraceFetchTimingHistoryTrimMs >=
                kDiagnosticRawTrimIntervalMs){
            trimRawHistoryForMode(runtimeTraceFetchTimingSamples,
                                  sample.wallClockMs,
                                  diagnosticRawHistoryFullRecordingEnabled,
                                  kDiagnosticRawDefaultMaxSamples);
            lastRuntimeTraceFetchTimingHistoryTrimMs = sample.wallClockMs;
        }
    }
}

void HardwareInterface::recordCurrentMotorEncoderUnitsForAxes(const std::vector<int>& logicalAxes)
{
    if(logicalAxes.empty()){
        return;
    }

    const qint64 scanStartUs = monotonicNowUs();
    std::vector<double> encoderPositions(motorIdVec.size(),
                                         std::numeric_limits<double>::quiet_NaN());
    bool hasAnyEncoder = false;
    for(const int axis : logicalAxes){
        if(axis < 0 ||
                axis >= static_cast<int>(motorIdVec.size()) ||
                axis >= static_cast<int>(motorComType.size()) ||
                motorComType[axis] != COM_EC_LS){
            continue;
        }
        double encoderUnit = 0.0;
        if(!readMotorEncoderUnitDirect(axis, encoderUnit)){
            continue;
        }
        encoderPositions[axis] = encoderUnit;
        hasAnyEncoder = true;
    }
    if(hasAnyEncoder){
        const qint64 scanEndUs = monotonicNowUs();
        recordMotorEncoderRawSample(encoderPositions,
                                    QStringLiteral("dmc_get_encoder_unit"),
                                    std::max<qint64>(0, scanEndUs - scanStartUs),
                                    wallClockNowUs(),
                                    scanEndUs);
    }
}

std::vector<int> HardwareInterface::defaultSessionEncoderUnitSamplingAxes() const
{
    std::vector<int> axes;
    const int axisLimit = std::min(kCableMotorTraceLogicalAxisCount,
                                   static_cast<int>(motorIdVec.size()));
    axes.reserve(axisLimit);
    for(int axis = 0; axis < axisLimit; ++axis){
        if(axis < static_cast<int>(motorComType.size()) &&
                motorComType[axis] == COM_EC_LS &&
                resolveLeadshineAxisIndex(axis) >= 0){
            axes.push_back(axis);
        }
    }
    return axes;
}

std::vector<int> HardwareInterface::sanitizeSessionEncoderUnitSamplingAxes(
        const std::vector<int>& logicalAxes) const
{
    const std::vector<int> requestedAxes =
            logicalAxes.empty() ? defaultSessionEncoderUnitSamplingAxes() : logicalAxes;
    std::vector<int> axes;
    axes.reserve(requestedAxes.size());
    for(const int axis : requestedAxes){
        if(axis < 0 ||
                axis >= kCableMotorTraceLogicalAxisCount ||
                axis >= static_cast<int>(motorIdVec.size()) ||
                axis >= static_cast<int>(motorComType.size()) ||
                motorComType[axis] != COM_EC_LS ||
                resolveLeadshineAxisIndex(axis) < 0){
            continue;
        }
        if(std::find(axes.begin(), axes.end(), axis) == axes.end()){
            axes.push_back(axis);
        }
    }
    return axes;
}

void HardwareInterface::sessionEncoderUnitSamplingLoop(std::vector<int> logicalAxes,
                                                       int intervalUs)
{
    using Clock = std::chrono::steady_clock;
    const auto interval =
            std::chrono::microseconds(std::max(100, intervalUs));
    auto nextWake = Clock::now();

    while(sessionEncoderUnitSamplingActive.load(std::memory_order_acquire)){
        runOnHardwareThread([&]() {
            recordCurrentMotorEncoderUnitsForAxes(logicalAxes);
        });

        nextWake += interval;
        const auto now = Clock::now();
        if(nextWake > now){
            std::this_thread::sleep_until(nextWake);
        }
        else{
            nextWake = now;
        }
    }
}

void HardwareInterface::startSessionEncoderUnitSampling(int intervalUs,
                                                        std::vector<int> logicalAxes)
{
    stopSessionEncoderUnitSampling();

    const int clampedIntervalUs = std::max(100, intervalUs);
    const std::vector<int> axes = runOnHardwareThread([&]() -> std::vector<int> {
        return sanitizeSessionEncoderUnitSamplingAxes(logicalAxes);
    });
    if(axes.empty()){
        return;
    }

    {
        QMutexLocker diagnosticsLocker(&diagnosticsMutex);
        lastMotorEncoderReadUs = 0;
        lastMotorEncoderRawHistoryAppendUs = 0;
    }

    QMutexLocker locker(&sessionEncoderUnitSamplerMutex);
    if(sessionEncoderUnitSamplingActive.load(std::memory_order_acquire)){
        return;
    }
    sessionEncoderUnitSamplingActive.store(true, std::memory_order_release);
    sessionEncoderUnitSamplerThread =
            std::thread(&HardwareInterface::sessionEncoderUnitSamplingLoop,
                        this,
                        axes,
                        clampedIntervalUs);
}

void HardwareInterface::stopSessionEncoderUnitSampling()
{
    std::thread threadToJoin;
    {
        QMutexLocker locker(&sessionEncoderUnitSamplerMutex);
        sessionEncoderUnitSamplingActive.store(false, std::memory_order_release);
        if(sessionEncoderUnitSamplerThread.joinable()){
            threadToJoin = std::move(sessionEncoderUnitSamplerThread);
        }
    }

    if(threadToJoin.joinable()){
        if(threadToJoin.get_id() == std::this_thread::get_id()){
            threadToJoin.detach();
        }
        else{
            threadToJoin.join();
        }
    }
}

void HardwareInterface::unsupportedFeature(const QString& featureName) {
    emit displayInfoSignal((QString("未接入功能: %1").arg(featureName)).toStdString(), "warning");
}

void HardwareInterface::setMotorPara(std::vector<unsigned int> idVec, std::vector<int> comType, std::vector<QString> portInfo,
                                     std::vector<double> posRaw2dataCof, std::vector<double> velRaw2dataCof,
                                     std::vector<int> slaveIdVec,
                                     std::vector<bool> torqueVelocityLimitEnabled) {
    return runOnHardwareThread([&]() {
    motorIdVec = std::move(idVec);
    motorComType = std::move(comType);
    motorPortInfo = std::move(portInfo);
    motorPosRaw2dataCof = std::move(posRaw2dataCof);
    motorVelRaw2dataCof = std::move(velRaw2dataCof);
    motorSlaveIdVec = std::move(slaveIdVec);
    motorTorqueVelocityLimitEnabled = std::move(torqueVelocityLimitEnabled);
    if(motorSlaveIdVec.size() < motorIdVec.size()){
        motorSlaveIdVec.resize(motorIdVec.size(), 0);
    }
    if(motorTorqueVelocityLimitEnabled.empty()){
        motorTorqueVelocityLimitEnabled.assign(motorIdVec.size(), true);
    }
    else if(motorTorqueVelocityLimitEnabled.size() < motorIdVec.size()){
        motorTorqueVelocityLimitEnabled.resize(motorIdVec.size(), false);
    }

    motorCurState.assign(motorIdVec.size(), false);
    motorCurPos.assign(motorIdVec.size(), 0.0);
    motorCommandPos.assign(motorIdVec.size(), 0.0);
    motorTraceActualPos.assign(motorIdVec.size(), 0.0);
    motorTraceCommandVelocity.assign(motorIdVec.size(), 0.0);
    motorTraceActualVelocity.assign(motorIdVec.size(), 0.0);
    motorTraceCommandVelocityValid.assign(motorIdVec.size(), false);
    motorTraceActualVelocityValid.assign(motorIdVec.size(), false);
    motorTraceTorqueNm.assign(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
    motorTraceTorqueValid.assign(motorIdVec.size(), false);
    motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);
    motorCurVel.assign(motorIdVec.size(), 0.0);
    motorEquivVec.resize(motorIdVec.size(), 0.0);
    motorSessionSafetyHomeTraceRawPulse.assign(motorIdVec.size(), 0);
    motorSessionSafetyHomeTraceValid.assign(motorIdVec.size(), false);
    motorSessionSafetyHomeTraceUsesFeedback.assign(motorIdVec.size(), false);
    resetMotorPositionTraceState();
    });
}

void HardwareInterface::setMotorSlaveIds(std::vector<int> slaveIdVec)
{
    return runOnHardwareThread([&]() {
    motorSlaveIdVec = std::move(slaveIdVec);
    if(motorSlaveIdVec.size() < motorIdVec.size()){
        motorSlaveIdVec.resize(motorIdVec.size(), 0);
    }
    std::fill(motorSessionSafetyHomeTraceValid.begin(),
              motorSessionSafetyHomeTraceValid.end(),
              false);
    resetRuntimeTraceState();
    });
}

void HardwareInterface::setMotorTorqueVelocityLimitEnabled(std::vector<bool> enabled)
{
    return runOnHardwareThread([&]() {
    motorTorqueVelocityLimitEnabled = std::move(enabled);
    if(motorTorqueVelocityLimitEnabled.empty()){
        motorTorqueVelocityLimitEnabled.assign(motorIdVec.size(), true);
    }
    else if(motorTorqueVelocityLimitEnabled.size() < motorIdVec.size()){
        motorTorqueVelocityLimitEnabled.resize(motorIdVec.size(), false);
    }
    });
}

void HardwareInterface::setMotorChangeSpdTime(double s) {
    return runOnHardwareThread([&]() {
    velChangeSpd = s;
    });
}

void HardwareInterface::setMotorHome(std::vector<double> homeValue) {
    return runOnHardwareThread([&]() {
    motorHomePos = std::move(homeValue);
    for(auto& samples : motorTracePositionSampleQueues){
        samples.clear();
    }
    });
}

void HardwareInterface::setMotorSafetyHomeTraceCommandRawPulse(std::vector<qint64> rawPulse)
{
    return runOnHardwareThread([&]() {
    motorSafetyHomeTraceCommandRawPulse = std::move(rawPulse);
    if(!motorIdVec.empty() &&
            motorSafetyHomeTraceCommandRawPulse.size() > motorIdVec.size()){
        motorSafetyHomeTraceCommandRawPulse.resize(motorIdVec.size());
    }
    // 切换到完整系统的持久化 command 零位语义时，退出 G302 会话零点。
    motorSessionSafetyHomeTraceRawPulse.assign(motorIdVec.size(), 0);
    motorSessionSafetyHomeTraceValid.assign(motorIdVec.size(), false);
    motorSessionSafetyHomeTraceUsesFeedback.assign(motorIdVec.size(), false);
    for(auto& samples : motorTracePositionSampleQueues){
        samples.clear();
    }
    });
}

void HardwareInterface::setMotorSafetyHomeEncoderUnit(std::vector<double> encoderUnit)
{
    return runOnHardwareThread([&]() {
    motorSafetyHomeEncoderUnit = std::move(encoderUnit);
    if(!motorIdVec.empty() &&
            motorSafetyHomeEncoderUnit.size() > motorIdVec.size()){
        motorSafetyHomeEncoderUnit.resize(motorIdVec.size());
    }
    for(auto& samples : motorTracePositionSampleQueues){
        samples.clear();
    }
    });
}

void HardwareInterface::setLeadshineAxisEquiv(std::vector<double> equivValue) {
    return runOnHardwareThread([&]() {
    motorEquivVec = std::move(equivValue);
    if(motorEquivVec.size() < motorIdVec.size()){
        motorEquivVec.resize(motorIdVec.size(), 0.0);
    }
    std::fill(motorSessionSafetyHomeTraceValid.begin(),
              motorSessionSafetyHomeTraceValid.end(),
              false);
    resetMotorPositionTraceState();
    });
}

bool HardwareInterface::setMotorHomeForAxis(int logicalIndex,
                                            double homeValue,
                                            bool* usesFeedback,
                                            qint64* rawPulse)
{
    if(usesFeedback){
        *usesFeedback = false;
    }
    if(rawPulse){
        *rawPulse = 0;
    }

    std::vector<bool> selectedUsesFeedback;
    std::vector<qint64> selectedRawPulse;
    if(!setMotorHomesForAxes({logicalIndex},
                             {homeValue},
                             &selectedUsesFeedback,
                             &selectedRawPulse)){
        return false;
    }
    if(usesFeedback && !selectedUsesFeedback.empty()){
        *usesFeedback = selectedUsesFeedback.front();
    }
    if(rawPulse && !selectedRawPulse.empty()){
        *rawPulse = selectedRawPulse.front();
    }
    return true;
}

bool HardwareInterface::setMotorHomesForAxes(const std::vector<int>& logicalIndices,
                                             const std::vector<double>& homeValues,
                                             std::vector<bool>* usesFeedback,
                                             std::vector<qint64>* rawPulse,
                                             std::vector<qint64>* commandRawPulse)
{
    return runOnHardwareThread([&]() -> bool {
    if(usesFeedback){
        usesFeedback->clear();
    }
    if(rawPulse){
        rawPulse->clear();
    }
    if(commandRawPulse){
        commandRawPulse->clear();
    }
    if(logicalIndices.empty() || logicalIndices.size() != homeValues.size()){
        return false;
    }

    std::vector<bool> seenAxis(motorIdVec.size(), false);
    for(std::size_t i = 0; i < logicalIndices.size(); ++i){
        const int logicalIndex = logicalIndices[i];
        if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorIdVec.size()) ||
                logicalIndex >= static_cast<int>(motorComType.size()) ||
                motorComType[logicalIndex] != COM_EC_LS ||
                seenAxis[logicalIndex] ||
                !std::isfinite(homeValues[i])){
            return false;
        }
        const double axisEquiv = resolveLeadshineAxisEquiv(logicalIndex);
        if(!std::isfinite(axisEquiv) || axisEquiv <= 0.0){
            return false;
        }
        seenAxis[logicalIndex] = true;
    }

    readRuntimeTraceCached(false);
    const qint64 nowUs = monotonicNowUs();
    const qint64 futureToleranceUs = std::max<qint64>(
                2 * 1000,
                static_cast<qint64>(runtimeTraceSamplePeriodUs) *
                    kRuntimeTraceTimestampFutureToleranceFrames);
    const bool freshFrame = runtimeTraceConfigReadbackValid &&
            runtimeTraceTimingReliable &&
            runtimeTraceFifoCaughtUp &&
            !runtimeTraceLost &&
            runtimeTraceNewestFrameAgeUs >= 0 &&
            runtimeTraceNewestFrameAgeUs <= kMotorPositionTraceFreshTimeoutUs &&
            latestMotorTracePositionFrameValid &&
            latestMotorTracePositionFrame.monotonicUs > 0 &&
            nowUs + futureToleranceUs >= latestMotorTracePositionFrame.monotonicUs &&
            (nowUs < latestMotorTracePositionFrame.monotonicUs ||
             nowUs - latestMotorTracePositionFrame.monotonicUs <=
                kMotorPositionTraceFreshTimeoutUs);
    if(!freshFrame){
        return false;
    }

    std::vector<bool> selectedUsesFeedback(logicalIndices.size(), false);
    std::vector<qint64> selectedRawPulse(logicalIndices.size(), 0);
    std::vector<qint64> selectedCommandRawPulse(logicalIndices.size(), 0);
    for(std::size_t i = 0; i < logicalIndices.size(); ++i){
        const int logicalIndex = logicalIndices[i];
        const bool feedbackValid =
                logicalIndex < static_cast<int>(latestMotorTracePositionFrame.feedbackRawPulse.size()) &&
                logicalIndex < static_cast<int>(latestMotorTracePositionFrame.feedbackValid.size()) &&
                latestMotorTracePositionFrame.feedbackValid[logicalIndex];
        const bool commandValid =
                logicalIndex < static_cast<int>(latestMotorTracePositionFrame.commandRawPulse.size()) &&
                logicalIndex < static_cast<int>(latestMotorTracePositionFrame.commandValid.size()) &&
                latestMotorTracePositionFrame.commandValid[logicalIndex];
        if(!feedbackValid && !commandValid){
            return false;
        }
        // 调用方请求 command 原始脉冲时，将它视作本次原子提交的必要字段。
        // G302 绞盘基准必须与 motorHome/安全基准来自同一帧，不能提交后再读另一帧补齐。
        if(commandRawPulse &&
                (!commandValid ||
                 latestMotorTracePositionFrame.commandRawPulse[logicalIndex] == 0)){
            return false;
        }

        // 优先锁存实际反馈；若当前只提供 command，则明确锁存 command。
        // 后续相对位置始终读取此处选定的同一通道，不做自动回退。
        selectedUsesFeedback[i] = feedbackValid;
        selectedRawPulse[i] = feedbackValid ?
                    latestMotorTracePositionFrame.feedbackRawPulse[logicalIndex] :
                    latestMotorTracePositionFrame.commandRawPulse[logicalIndex];
        if(commandValid){
            selectedCommandRawPulse[i] =
                    latestMotorTracePositionFrame.commandRawPulse[logicalIndex];
        }
    }

    // 所有轴的前置检查通过后再整体提交，避免整机安全基准只更新一部分。
    if(motorHomePos.size() < motorIdVec.size()){
        motorHomePos.resize(motorIdVec.size(), 0.0);
    }
    motorSessionSafetyHomeTraceRawPulse.resize(motorIdVec.size(), 0);
    motorSessionSafetyHomeTraceValid.resize(motorIdVec.size(), false);
    motorSessionSafetyHomeTraceUsesFeedback.resize(motorIdVec.size(), false);
    ensureMotorTracePositionOffsetStorage();
    for(std::size_t i = 0; i < logicalIndices.size(); ++i){
        const int logicalIndex = logicalIndices[i];
        motorHomePos[logicalIndex] = homeValues[i];
        motorSessionSafetyHomeTraceRawPulse[logicalIndex] = selectedRawPulse[i];
        motorSessionSafetyHomeTraceUsesFeedback[logicalIndex] = selectedUsesFeedback[i];
        motorSessionSafetyHomeTraceValid[logicalIndex] = true;
        if(logicalIndex < static_cast<int>(motorCommandTraceOffsetValid.size())){
            motorCommandTraceOffsetValid[logicalIndex] = false;
        }
        if(logicalIndex < static_cast<int>(motorActualTraceOffsetValid.size())){
            motorActualTraceOffsetValid[logicalIndex] = false;
        }
        if(logicalIndex < static_cast<int>(motorTracePositionSampleQueues.size())){
            motorTracePositionSampleQueues[logicalIndex].clear();
        }
    }
    if(usesFeedback){
        *usesFeedback = selectedUsesFeedback;
    }
    if(rawPulse){
        *rawPulse = selectedRawPulse;
    }
    if(commandRawPulse){
        *commandRawPulse = selectedCommandRawPulse;
    }
    return true;
    });
}

void HardwareInterface::setLeadshineRatedMotorTorqueNm(double ratedTorqueNm)
{
    return runOnHardwareThread([&]() {
    const double normalizedRatedTorque =
            normalizedLeadshineRatedMotorTorqueNm(ratedTorqueNm);
    if(std::fabs(leadshineRatedMotorTorqueNm - normalizedRatedTorque) <= 1e-9){
        return;
    }
    leadshineRatedMotorTorqueNm = normalizedRatedTorque;
    motorTraceTorqueNm.assign(motorIdVec.size(),
                              std::numeric_limits<double>::quiet_NaN());
    motorTraceTorqueValid.assign(motorIdVec.size(), false);
    motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);
    });
}

void HardwareInterface::setMotorSoftwareLimits(std::vector<double> minPos,
                                               std::vector<double> maxPos,
                                               std::vector<double> maxVel)
{
    return runOnHardwareThread([&]() {
    motorSoftwareMinPos = std::move(minPos);
    motorSoftwareMaxPos = std::move(maxPos);
    motorSoftwareMaxVel = std::move(maxVel);
    const std::size_t axisCount = motorIdVec.size();
    if(motorSoftwareMinPos.size() < axisCount){
        motorSoftwareMinPos.resize(axisCount, 0.0);
    }
    if(motorSoftwareMaxPos.size() < axisCount){
        motorSoftwareMaxPos.resize(axisCount, 0.0);
    }
    if(motorSoftwareMaxVel.size() < axisCount){
        motorSoftwareMaxVel.resize(axisCount, 0.0);
    }
    });
}

void HardwareInterface::setSensorPara(std::vector<unsigned int> idVec, std::vector<int> comType, std::vector<QString> portInfo,
                                      std::vector<double> raw2dataCof) {
    return runOnHardwareThread([&]() {
    sensorIdVec = std::move(idVec);
    sensorComType = std::move(comType);
    sensorPortInfo = std::move(portInfo);
    sensorRaw2DataCof = std::move(raw2dataCof);
    forceSensorCachedValue.assign(sensorComType.size(), 0.0);
    forceSensorCacheValid.assign(sensorComType.size(), false);
    forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
    nextForceSensorPollIndex = 0;
    resetForceSensorTraceState();

    rodCurForce.clear();
    jointCurTheta.clear();
    jointCurThetaID.clear();
    for (int i = 0; i < static_cast<int>(sensorIdVec.size()); ++i) {
        if (sensorComType[i] == COM_485_SBT) {
            rodCurForce.push_back(0.0);
        } 
    }
    });
}

void HardwareInterface::setSensorPara(std::vector<int> comType, std::vector<int> _sensorPort, std::vector<int> _sensorAdr,
                                      std::vector<int> _sensorDataAdr, std::vector<int> _sensorDataLen,
                                      std::vector<double> raw2dataCof) {
    return runOnHardwareThread([&]() {
    sensorComType = std::move(comType);
    sensorPort = std::move(_sensorPort);
    sensorAdr = std::move(_sensorAdr);
    sensorDataAdr = std::move(_sensorDataAdr);
    sensorDataLen = std::move(_sensorDataLen);
    sensorRaw2DataCof = std::move(raw2dataCof);
    sensorHomeValue.resize(sensorComType.size(), 0.0);
    forceSensorCachedValue.assign(sensorComType.size(), 0.0);
    forceSensorCacheValid.assign(sensorComType.size(), false);
    forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
    nextForceSensorPollIndex = 0;
    resetForceSensorTraceState();
    });
}

void HardwareInterface::setForceSensorHome(std::vector<double> homeValue) {
    return runOnHardwareThread([&]() {
    sensorHomeValue = std::move(homeValue);
    sensorHomeValue.resize(sensorComType.size(), 0.0);
    forceSensorCachedValue.assign(sensorComType.size(), 0.0);
    forceSensorCacheValid.assign(sensorComType.size(), false);
    forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
    nextForceSensorPollIndex = 0;
    resetForceSensorTraceState();
    });
}

void HardwareInterface::setForceSensorIsSigned(bool isSigned) {
    return runOnHardwareThread([&]() {
    forceSensorIsSigned = isSigned;
    forceSensorCachedValue.assign(sensorComType.size(), 0.0);
    forceSensorCacheValid.assign(sensorComType.size(), false);
    forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
    nextForceSensorPollIndex = 0;
    resetForceSensorTraceState();
    });
}

bool HardwareInterface::setMotorSoftwareLimitForAxis(int logicalIndex,
                                                       double minPos,
                                                       double maxPos,
                                                       double maxVel)
{
    return runOnHardwareThread([&]() -> bool {
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorIdVec.size()) ||
            !std::isfinite(minPos) || !std::isfinite(maxPos) ||
            !std::isfinite(maxVel) || minPos >= maxPos || maxVel <= 0.0){
        return false;
    }
    motorSoftwareMinPos.resize(motorIdVec.size(), 0.0);
    motorSoftwareMaxPos.resize(motorIdVec.size(), 0.0);
    motorSoftwareMaxVel.resize(motorIdVec.size(), 0.0);
    motorSoftwareMinPos[logicalIndex] = minPos;
    motorSoftwareMaxPos[logicalIndex] = maxPos;
    motorSoftwareMaxVel[logicalIndex] = maxVel;
    return true;
    });
}

void HardwareInterface::setRuntimeTraceConfigType(RuntimeTraceConfigType type)
{
    return runOnHardwareThread([&]() {
    if(activeRuntimeTraceConfigType == type){
        return;
    }
    activeRuntimeTraceConfigType = type;
    resetRuntimeTraceState();
    });
}

HardwareInterface::RuntimeTraceConfigType HardwareInterface::runtimeTraceConfigType() const
{
    return runOnHardwareThread([&]() -> RuntimeTraceConfigType {
    return activeRuntimeTraceConfigType;
    });
}

bool HardwareInterface::runtimeTraceUsageProfileIncludesVelocitySignals(
        RuntimeTraceUsageProfile profile) const
{
    return profile != RuntimeTraceUsageProfile::Base;
}

bool HardwareInterface::runtimeTraceUsageProfileIncludesForceSensors(
        RuntimeTraceUsageProfile profile) const
{
    if(profile == RuntimeTraceUsageProfile::Base){
        return baseRuntimeTraceForceSensorEnabled;
    }
    return RuntimeFeatureSwitches::kOnlineVelocityForceSensorTraceEnabled;
}

void HardwareInterface::resetEndpointRemoteRuntimeTraceStatusFault()
{
    endpointRemoteTraceStatusFaultLatched = false;
    endpointRemoteTraceStatusFaultAxis = -1;
    endpointRemoteTraceStatusFaultWord = 0;
    endpointRemoteTraceStatusFaultStateMachine = -1;
    endpointRemoteTraceStatusFaultLogicalFrameSequence = 0;
}

bool HardwareInterface::setRuntimeTraceUsageProfile(
        RuntimeTraceUsageProfile profile,
        quint64 endpointRemoteSessionToken)
{
    return runOnHardwareThread([&]() -> bool {
    const bool endpointRemoteProfile =
            profile == RuntimeTraceUsageProfile::EndpointRemoteTransition ||
            profile == RuntimeTraceUsageProfile::EndpointRemoteRunning;
    if(endpointRemoteProfile != (endpointRemoteSessionToken != 0)){
        return false;
    }
    if(activeRuntimeTraceUsageProfile == profile &&
            runtimeTraceEndpointRemoteSessionToken == endpointRemoteSessionToken){
        return true;
    }

    const RuntimeTraceUsageProfile previousProfile =
            activeRuntimeTraceUsageProfile;
    const quint64 previousSessionToken = runtimeTraceEndpointRemoteSessionToken;
    const bool profileConfigurationChanged =
            runtimeTraceUsageProfileIncludesVelocitySignals(previousProfile) !=
                runtimeTraceUsageProfileIncludesVelocitySignals(profile) ||
            runtimeTraceUsageProfileIncludesForceSensors(previousProfile) !=
                runtimeTraceUsageProfileIncludesForceSensors(profile);

    activeRuntimeTraceUsageProfile = profile;
    runtimeTraceEndpointRemoteSessionToken = endpointRemoteSessionToken;
    resetEndpointRemoteRuntimeTraceStatusFault();
    if(!profileConfigurationChanged || !isConnectLS){
        ++runtimeTraceUsageProfileGeneration;
        return true;
    }

    resetRuntimeTraceState();
    if(configureRuntimeTraceRead()){
        ++runtimeTraceUsageProfileGeneration;
        return true;
    }

    // 配置失败时恢复完整的上一profile语义及其会话令牌。回滚重配即使
    // 失败也不会授权新profile；调用方据此保持停机。
    activeRuntimeTraceUsageProfile = previousProfile;
    runtimeTraceEndpointRemoteSessionToken = previousSessionToken;
    resetEndpointRemoteRuntimeTraceStatusFault();
    resetRuntimeTraceState();
    configureRuntimeTraceRead();
    return false;
    });
}

HardwareInterface::RuntimeTraceUsageProfile
HardwareInterface::runtimeTraceUsageProfile() const
{
    return runOnHardwareThread([&]() -> RuntimeTraceUsageProfile {
    return activeRuntimeTraceUsageProfile;
    });
}

void HardwareInterface::setLiteRuntimeTraceTopology(
        LiteRuntimeTraceTopology topology)
{
    return runOnHardwareThread([&]() {
    if(activeLiteRuntimeTraceTopology == topology){
        return;
    }
    activeLiteRuntimeTraceTopology = topology;
    resetRuntimeTraceState();
    });
}

HardwareInterface::LiteRuntimeTraceTopology
HardwareInterface::liteRuntimeTraceTopology() const
{
    return runOnHardwareThread([&]() -> LiteRuntimeTraceTopology {
    return activeLiteRuntimeTraceTopology;
    });
}

void HardwareInterface::setRuntimeTraceCommissioningSelection(int logicalAxis,
                                                               int sensorIndex)
{
    return runOnHardwareThread([&]() {
    if(runtimeTraceCommissioningAxis == logicalAxis &&
            runtimeTraceCommissioningSensor == sensorIndex){
        return;
    }
    runtimeTraceCommissioningAxis = logicalAxis;
    runtimeTraceCommissioningSensor = sensorIndex;
    std::fill(motorSessionSafetyHomeTraceValid.begin(),
              motorSessionSafetyHomeTraceValid.end(),
              false);
    resetRuntimeTraceState();
    });
}

void HardwareInterface::clearRuntimeTraceCommissioningSelection()
{
    setRuntimeTraceCommissioningSelection(-1, -1);
}

void HardwareInterface::setForceSensorTraceReadEnabled(bool enabled)
{
    return runOnHardwareThread([&]() {
    if(baseRuntimeTraceForceSensorEnabled == enabled){
        return;
    }
    const bool previousProfileIncludesForce =
            runtimeTraceUsageProfileIncludesForceSensors(
                activeRuntimeTraceUsageProfile);
    baseRuntimeTraceForceSensorEnabled = enabled;
    const bool currentProfileIncludesForce =
            runtimeTraceUsageProfileIncludesForceSensors(
                activeRuntimeTraceUsageProfile);
    if(previousProfileIncludesForce != currentProfileIncludesForce){
        resetForceSensorTraceState();
    }
    });
}

void HardwareInterface::setForceSensorTraceSamplePeriodUs(int periodUs)
{
    return runOnHardwareThread([&]() {
    const int periodSwitchMidpointUs =
            (kForceSensorTraceMinPeriodUs + kForceSensorTraceMaxPeriodUs) / 2;
    const int boundedPeriodUs =
            periodUs < periodSwitchMidpointUs ?
                kForceSensorTraceMinPeriodUs : kForceSensorTraceMaxPeriodUs;
    if(forceSensorTraceSamplePeriodUs == boundedPeriodUs &&
            motorPositionTraceSamplePeriodUs == boundedPeriodUs){
        return;
    }

    forceSensorTraceSamplePeriodUs = boundedPeriodUs;
    motorPositionTraceSamplePeriodUs = boundedPeriodUs;
    resetForceSensorTraceState();
    });
}

// 只打开雷赛控制卡。调试模式使用该入口验证控制卡通信和读取总线/轴诊断，
// 故意不写脉冲当量、不读写运行零位、不清错、不使能，也不写转矩参数。
bool HardwareInterface::connectLSControllerOnly()
{
    return runOnHardwareThread([&]() -> bool {
    if (isConnectLS) {
        return true;
    }

    const short boardCount = dmc_board_init();
    recordCommunicationEvent(false, QStringLiteral("dmc_board_init"));
    if (boardCount <= 0) {
        isConnectLS = false;
        emit displayInfoSignal("Leadshine control card not detected.", "error");
        return false;
    }

    isConnectLS = true;
    resetRuntimeTraceState();
    std::fill(motorCurState.begin(), motorCurState.end(), false);
    emit displayInfoSignal("Leadshine controller communication opened; no axis configuration or enable command was issued.",
                           "normal");
    return true;
    });
}

bool HardwareInterface::setRuntimeTraceVelocitySignalsEnabled(bool enabled)
{
    return setOnlineVelocityRuntimeTraceProfileEnabled(enabled);
}

bool HardwareInterface::setOnlineVelocityRuntimeTraceProfileEnabled(bool enabled)
{
    return setRuntimeTraceUsageProfile(
                enabled ? RuntimeTraceUsageProfile::PresetOnlineVelocity :
                          RuntimeTraceUsageProfile::Base);
}

// 完整整机启动入口：在控制卡连接成功后配置全部已建模雷赛轴、读取位置并建立整机运行零位。
bool HardwareInterface::connectLS() {
    if(!connectLSControllerOnly()){
        return false;
    }

    return runOnHardwareThread([&]() -> bool {
    auto closeBoardOnConnectFailure = [this]() {
        resetRuntimeTraceState();
        dmc_board_close();
        isConnectLS = false;
    };

    // 逐逻辑轴确认其硬件轴号和脉冲当量有效；任一轴失败都关闭板卡，避免半初始化状态继续运行。
    std::vector<double> tmpMotorVec(motorIdVec.size(), 0.0);
    bool hasLeadshineAxis = false;
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (motorComType[i] == COM_EC_LS) {
            hasLeadshineAxis = true;
            const int hardwareAxis = resolveLeadshineAxisIndex(i);
            if (hardwareAxis < 0) {
                closeBoardOnConnectFailure();
                emit displayInfoSignal(QString("%1 has no valid controller axis.")
                                           .arg(axisDisplayName(i))
                                           .toStdString(),
                                       "error");
                return false;
            }
            const double axisEquiv = resolveLeadshineAxisEquiv(i);
            if (axisEquiv <= 0.0) {
                closeBoardOnConnectFailure();
                emit displayInfoSignal(QString("%1未配置有效的脉冲当量").arg(axisDisplayName(i)).toStdString(),
                                       "error");
                return false;
            }
            const short equivRet = dmc_set_equiv(0, static_cast<WORD>(hardwareAxis), axisEquiv);
            recordCommunicationEvent(false, QStringLiteral("dmc_set_equiv"));
            if (equivRet != 0) {
                closeBoardOnConnectFailure();
                emit displayInfoSignal(
                    QString("雷赛控制卡初始化失败：%1脉冲当量设置失败，错误码%2")
                        .arg(axisDisplayName(i))
                        .arg(equivRet)
                        .toStdString(),
                    "error");
                return false;
            }
            double tmp = 0.0;
            if (!readMotorPositionUnitDirect(i, tmp)) {
                closeBoardOnConnectFailure();
                emit displayInfoSignal(
                    QString("Leadshine handshake failed: %1 position read did not succeed.")
                        .arg(axisDisplayName(i))
                        .toStdString(),
                    "error");
                return false;
            }
            tmpMotorVec[i] = tmp;
            motorCurState[i] = false;
        }
    }
    if (!hasLeadshineAxis) {
        closeBoardOnConnectFailure();
        emit displayInfoSignal("No Leadshine motor axis is configured.", "error");
        return false;
    }
    if (!useStaticMotorHome) {
        motorHomePos = tmpMotorVec;
    }

    if (!useStaticSensorHome) {
        sensorHomeValue.assign(sensorComType.size(), 0.0);
    }
    else {
        sensorHomeValue.resize(sensorComType.size(), 0.0);
    }

    if(!applyLeadshineTorqueVelocityLimit(leadshineTorqueVelocityLimitRpm)){
        closeBoardOnConnectFailure();
        return false;
    }
    return true;
    });
}

bool HardwareInterface::applyLeadshineAxisEquiv() {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        return true;
    }

    bool hasLeadshineAxis = false;
    bool allOk = true;
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (motorComType[i] != COM_EC_LS) {
            continue;
        }

        hasLeadshineAxis = true;
        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        const double axisEquiv = resolveLeadshineAxisEquiv(i);
        if (hardwareAxis < 0 || axisEquiv <= 0.0) {
            allOk = false;
            continue;
        }

        const short ret = dmc_set_equiv(0, static_cast<WORD>(hardwareAxis), axisEquiv);
        recordCommunicationEvent(false, QStringLiteral("dmc_set_equiv"));
        if (ret != 0) {
            emit displayInfoSignal(QString("雷赛轴脉冲当量更新失败：%1，错误码%2")
                                       .arg(axisDisplayName(i))
                                       .arg(ret)
                                       .toStdString(),
                                   "error");
            allOk = false;
        }
    }
    if(hasLeadshineAxis && allOk){
        resetRuntimeTraceState();
    }
    return hasLeadshineAxis && allOk;
    });
}

bool HardwareInterface::applyLeadshineAxisEquiv(int logicalIndex)
{
    return runOnHardwareThread([&]() -> bool {
    if(!isConnectLS){
        emit displayInfoSignal("Leadshine card is not connected; cannot write axis equivalent.", "error");
        return false;
    }
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorComType.size()) ||
            motorComType[logicalIndex] != COM_EC_LS){
        emit displayInfoSignal("Invalid Leadshine logical axis for equivalent write.", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    const double axisEquiv = resolveLeadshineAxisEquiv(logicalIndex);
    if(hardwareAxis < 0 || !std::isfinite(axisEquiv) || axisEquiv <= 0.0){
        emit displayInfoSignal(QString("%1 has invalid hardware-axis/equivalent configuration.")
                               .arg(axisDisplayName(logicalIndex)).toStdString(),
                               "error");
        return false;
    }

    const short ret = dmc_set_equiv(0, static_cast<WORD>(hardwareAxis), axisEquiv);
    recordCommunicationEvent(false, QStringLiteral("dmc_set_equiv"));
    if(ret != 0){
        emit displayInfoSignal(QString("%1 axis equivalent write failed, code %2.")
                               .arg(axisDisplayName(logicalIndex))
                               .arg(ret)
                               .toStdString(),
                               "error");
        return false;
    }
    resetRuntimeTraceState();
    return true;
    });
}

void HardwareInterface::setLeadshineTorqueVelocityLimitRpm(double velocityLimitRpm)
{
    return runOnHardwareThread([&]() {
    if(std::isfinite(velocityLimitRpm) && velocityLimitRpm > 0.0){
        leadshineTorqueVelocityLimitRpm = velocityLimitRpm;
    }
    });
}

bool HardwareInterface::applyLeadshineTorqueVelocityLimit(double velocityLimitRpm)
{
    return runOnHardwareThread([&]() -> bool {
    if(!isConnectLS){
        emit displayInfoSignal("错误：雷赛控制卡未连接，无法写入转矩模式速度限制", "error");
        return false;
    }
    if(!std::isfinite(velocityLimitRpm) || velocityLimitRpm <= 0.0){
        emit displayInfoSignal("错误：转矩模式速度限制必须大于0 rpm", "error");
        return false;
    }
    const long velocityLimitPulsePerSecond =
            static_cast<long>(std::lround(velocityLimitRpm / 60.0 *
                                          kLeadshineTorqueVelocityLimitPulsesPerRev));

    bool hasWritableAxis = false;
    bool allOk = true;
    std::vector<WORD> writtenNodes;

    for(int i = 0; i < static_cast<int>(motorIdVec.size()); ++i){
        if(i >= static_cast<int>(motorComType.size()) || motorComType[i] != COM_EC_LS){
            continue;
        }
        if(i < static_cast<int>(motorTorqueVelocityLimitEnabled.size()) &&
                !motorTorqueVelocityLimitEnabled[i]){
            continue;
        }

        hasWritableAxis = true;
        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        if(hardwareAxis < 0){
            emit displayInfoSignal(QString("错误：%1未配置有效控制卡轴号，无法写入220Bh转矩模式速度限制")
                                       .arg(axisDisplayName(i))
                                       .toStdString(),
                                   "error");
            allOk = false;
            continue;
        }

        WORD slaveAddress = 0;
        if(i < static_cast<int>(motorSlaveIdVec.size()) && motorSlaveIdVec[i] > 0){
            slaveAddress = static_cast<WORD>(motorSlaveIdVec[i]);
        }
        else{
            WORD subSlaveAddress = 0;
            const short addrRet = nmc_get_axis_node_address(0,
                                                            static_cast<WORD>(hardwareAxis),
                                                            &slaveAddress,
                                                            &subSlaveAddress);
            recordCommunicationEvent(false, QStringLiteral("nmc_get_axis_node_address"));
            if(addrRet != 0 || slaveAddress == 0){
                emit displayInfoSignal(QString("错误：%1从站地址读取失败，无法写入220Bh转矩模式速度限制，错误码%2")
                                           .arg(axisDisplayName(i))
                                           .arg(addrRet)
                                           .toStdString(),
                                       "error");
                allOk = false;
                continue;
            }
        }

        if(std::find(writtenNodes.begin(), writtenNodes.end(), slaveAddress) != writtenNodes.end()){
            continue;
        }

        const short ret = nmc_set_node_od(0,
                                          kLeadshineEtherCatPort,
                                          slaveAddress,
                                          kLeadshineTorqueVelocityLimitIndex,
                                          kLeadshineTorqueVelocityLimitSubIndex,
                                          kLeadshineTorqueVelocityLimitBitLength,
                                          velocityLimitPulsePerSecond);
        recordCommunicationEvent(false, QStringLiteral("nmc_set_node_od"));
        if(ret != 0){
            emit displayInfoSignal(QString("错误：电机从站%1写入220Bh转矩模式速度限制%2 rpm失败，写入值%3 pulse/s，错误码%4")
                                       .arg(slaveAddress)
                                       .arg(velocityLimitRpm, 0, 'f', 2)
                                       .arg(velocityLimitPulsePerSecond)
                                       .arg(ret)
                                       .toStdString(),
                                   "error");
            allOk = false;
            continue;
        }

        writtenNodes.push_back(slaveAddress);
    }

    if(!hasWritableAxis){
        emit displayInfoSignal("No torque-mode velocity limit write is required.", "normal");
        return true;
    }
    if(allOk){
        emit displayInfoSignal(QString("转矩模式伺服速度限制已写入非直线模组电机：220Bh = %1 rpm，写入值%2 pulse/s")
                                   .arg(velocityLimitRpm, 0, 'f', 2)
                                   .arg(velocityLimitPulsePerSecond)
                                   .toStdString(),
                               "normal");
    }
    return allOk;
    });
}

bool HardwareInterface::applyLeadshineTorqueVelocityLimit(int logicalIndex,
                                                           double velocityLimitRpm)
{
    return runOnHardwareThread([&]() -> bool {
    if(!isConnectLS){
        emit displayInfoSignal("Leadshine card is not connected; cannot write torque velocity limit.", "error");
        return false;
    }
    if(!std::isfinite(velocityLimitRpm) || velocityLimitRpm <= 0.0){
        emit displayInfoSignal("Torque velocity limit must be greater than zero.", "error");
        return false;
    }
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorComType.size()) ||
            motorComType[logicalIndex] != COM_EC_LS){
        emit displayInfoSignal("Invalid Leadshine logical axis for torque velocity limit write.", "error");
        return false;
    }
    if(logicalIndex < static_cast<int>(motorTorqueVelocityLimitEnabled.size()) &&
            !motorTorqueVelocityLimitEnabled[logicalIndex]){
        return true;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if(hardwareAxis < 0){
        emit displayInfoSignal(QString("%1 has no valid controller axis.")
                               .arg(axisDisplayName(logicalIndex)).toStdString(),
                               "error");
        return false;
    }

    WORD slaveAddress = 0;
    if(logicalIndex < static_cast<int>(motorSlaveIdVec.size()) &&
            motorSlaveIdVec[logicalIndex] > 0){
        slaveAddress = static_cast<WORD>(motorSlaveIdVec[logicalIndex]);
    }
    else{
        WORD subSlaveAddress = 0;
        const short addrRet = nmc_get_axis_node_address(0,
                                                        static_cast<WORD>(hardwareAxis),
                                                        &slaveAddress,
                                                        &subSlaveAddress);
        recordCommunicationEvent(false, QStringLiteral("nmc_get_axis_node_address"));
        if(addrRet != 0 || slaveAddress == 0){
            emit displayInfoSignal(QString("%1 slave address read failed, code %2.")
                                   .arg(axisDisplayName(logicalIndex))
                                   .arg(addrRet)
                                   .toStdString(),
                                   "error");
            return false;
        }
    }

    const long velocityLimitPulsePerSecond =
            static_cast<long>(std::lround(velocityLimitRpm / 60.0 *
                                          kLeadshineTorqueVelocityLimitPulsesPerRev));
    const short ret = nmc_set_node_od(0,
                                      kLeadshineEtherCatPort,
                                      slaveAddress,
                                      kLeadshineTorqueVelocityLimitIndex,
                                      kLeadshineTorqueVelocityLimitSubIndex,
                                      kLeadshineTorqueVelocityLimitBitLength,
                                      velocityLimitPulsePerSecond);
    recordCommunicationEvent(false, QStringLiteral("nmc_set_node_od"));
    if(ret != 0){
        emit displayInfoSignal(QString("%1 torque velocity limit write failed, code %2.")
                               .arg(axisDisplayName(logicalIndex))
                               .arg(ret)
                               .toStdString(),
                               "error");
        return false;
    }
    return true;
    });
}

bool HardwareInterface::configureLeadshineAxisForCommissioning(int logicalIndex)
{
    if(!applyLeadshineAxisEquiv(logicalIndex)){
        return false;
    }

    const bool positionReadable = runOnHardwareThread([&]() -> bool {
        double currentPosition = 0.0;
        return readMotorPositionUnitDirect(logicalIndex, currentPosition);
    });
    if(!positionReadable){
        emit displayInfoSignal(QString("%1 position read failed during commissioning setup.")
                               .arg(axisDisplayName(logicalIndex)).toStdString(),
                               "error");
        return false;
    }

    return applyLeadshineTorqueVelocityLimit(logicalIndex,
                                              leadshineTorqueVelocityLimitRpm);
}

bool HardwareInterface::disconnectLS() {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        std::fill(motorSessionSafetyHomeTraceValid.begin(),
                  motorSessionSafetyHomeTraceValid.end(),
                  false);
        return true;
    }
    exportMotorTracePositionWindowForEvent(QStringLiteral("disconnect"));
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtMotorVelMax.clear();
    activePvtTimeStamp.clear();
    activePvtStartMonotonicUs = 0;

    // Only issue disable commands for axes that this process observed as
    // enabled. A G302 commissioning session may intentionally have absent
    // axes, so disconnect must not turn into an all-axis communication test.
    for(int logicalIndex = 0;
        logicalIndex < static_cast<int>(motorCurState.size());
        ++logicalIndex){
        if(motorCurState[logicalIndex]){
            applyLeadshineAxisEnableState(logicalIndex, false, false);
        }
    }
    resetForceSensorTraceState();
    std::fill(motorSessionSafetyHomeTraceValid.begin(),
              motorSessionSafetyHomeTraceValid.end(),
              false);
    dmc_board_close();
    isConnectLS = false;
    return true;
    });
}

bool HardwareInterface::applyLeadshineAxisEnableState(int logicalIndex, bool enable, bool emitErrors)
{
    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if (hardwareAxis < 0) {
        if (emitErrors) {
            emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(logicalIndex)).toStdString(), "error");
        }
        return false;
    }

    if (!enable) {
        const short ret = nmc_set_axis_disable(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true, QStringLiteral("nmc_set_axis_disable"));
        if (ret == 0) {
            motorCurState[logicalIndex] = false;
            if(logicalIndex < static_cast<int>(motorSessionSafetyHomeTraceValid.size())){
                motorSessionSafetyHomeTraceValid[logicalIndex] = false;
            }
            return true;
        }
        // Keep the conservative enabled cache on a failed disable so a later
        // disconnect/emergency path retries this axis instead of silently
        // assuming it is safe.
        motorCurState[logicalIndex] = true;
        if (emitErrors) {
            emit displayInfoSignal(QString("%1失能失败，返回码%2")
                                       .arg(axisDisplayName(logicalIndex))
                                       .arg(ret)
                                       .toStdString(),
                                   "error");
        }
        return false;
    }

    short lastEnableRet = 0;
    short lastStateRet = 0;
    WORD lastStateMachine = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
        lastEnableRet = nmc_set_axis_enable(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true, QStringLiteral("nmc_set_axis_enable"));
        lastStateRet = nmc_get_axis_state_machine(0, static_cast<WORD>(hardwareAxis), &lastStateMachine);
        recordCommunicationEvent(true, QStringLiteral("nmc_get_axis_state_machine"));
        if (lastEnableRet == 0 && lastStateRet == 0 && lastStateMachine == 4U) {
            motorCurState[logicalIndex] = true;
            return true;
        }
        QThread::msleep(50);
    }

    motorCurState[logicalIndex] = false;
    if (emitErrors) {
        emit displayInfoSignal(QString("%1使能未进入操作使能状态，enable返回码%2，状态读取返回码%3，状态机%4")
                                   .arg(axisDisplayName(logicalIndex))
                                   .arg(lastEnableRet)
                                   .arg(lastStateRet)
                                   .arg(lastStateMachine)
                                   .toStdString(),
                               "error");
    }
    return false;
}

bool HardwareInterface::refreshLeadshineMotorEnableState(int logicalIndex,
                                                         qint64* apiDurationUs)
{
    if(apiDurationUs){
        *apiDurationUs = 0;
    }
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorCurState.size())){
        return false;
    }

    bool enabled = false;
    if(isConnectLS &&
            logicalIndex < static_cast<int>(motorComType.size()) &&
            motorComType[logicalIndex] == COM_EC_LS){
        const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
        if(hardwareAxis >= 0){
            WORD stateMachine = 0;
            const qint64 apiStartUs = apiDurationUs ? monotonicNowUs() : 0;
            const short ret =
                    nmc_get_axis_state_machine(0, static_cast<WORD>(hardwareAxis), &stateMachine);
            if(apiDurationUs){
                *apiDurationUs = std::max<qint64>(0, monotonicNowUs() - apiStartUs);
            }
            recordCommunicationEvent(false, QStringLiteral("nmc_get_axis_state_machine"));
            enabled = (ret == 0 && stateMachine == 4U);
        }
    }

    motorCurState[logicalIndex] = enabled;
    return enabled;
}

bool HardwareInterface::setAllMotorEnable(bool enable) {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        return false;
    }

    if(!enable){
        if(motorJogVelocityFastActive.size() != motorIdVec.size()){
            motorJogVelocityFastActive.assign(motorIdVec.size(), false);
        }
        else{
            std::fill(motorJogVelocityFastActive.begin(),
                      motorJogVelocityFastActive.end(),
                      false);
        }
        const bool freezePvtWindow =
                motorTracePositionWindowRecordingEnabled || hasActivePvtTrajectory;
        if(freezePvtWindow){
            readRuntimeTraceCached();
            freezeMotorTracePositionWindowRecording();
        }
        exportMotorTracePositionWindowForEvent(QStringLiteral("motor_disable"));
    }

    bool hasLeadshineAxis = false;
    bool allOk = true;
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (motorComType[i] != COM_EC_LS) {
            continue;
        }
        hasLeadshineAxis = true;
        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        if (hardwareAxis < 0) {
            motorCurState[i] = false;
            allOk = false;
            continue;
        }

        if (!applyLeadshineAxisEnableState(i, enable, true)) {
            allOk = false;
        }
    }
    if(hasLeadshineAxis && allOk){
        resetMotorPositionTraceState();
    }
    return hasLeadshineAxis && allOk;
    });
}


bool HardwareInterface::setMotorEnable(int index, bool enable) {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        emit displayInfoSignal("当前未连接雷赛控制卡", "error");
        return false;
    }
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        emit displayInfoSignal("电机编号超出范围", "error");
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("当前电机未配置为雷赛电机", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }

    if(!enable){
        const bool freezePvtWindow =
                motorTracePositionWindowRecordingEnabled || hasActivePvtTrajectory;
        if(freezePvtWindow){
            readRuntimeTraceCached();
            freezeMotorTracePositionWindowRecording();
        }
        exportMotorTracePositionWindowForEvent(QStringLiteral("motor_disable"));
    }

    return applyLeadshineAxisEnableState(index, enable, true);
    });
}

bool HardwareInterface::emergencyStopAll() {
    std::vector<int> logicalAxes;
    logicalAxes.reserve(motorComType.size());
    for(int logicalIndex = 0;
        logicalIndex < static_cast<int>(motorComType.size());
        ++logicalIndex){
        if(motorComType[logicalIndex] == COM_EC_LS){
            logicalAxes.push_back(logicalIndex);
        }
    }
    return emergencyStopAxes(logicalAxes);
}

bool HardwareInterface::emergencyStopAxes(const std::vector<int>& logicalAxes) {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        return false;
    }

    const bool exportTraceWindowAfterStop =
            motorTracePositionWindowRecordingEnabled || hasActivePvtTrajectory;

    bool allOk = true;
    for(const int logicalIndex : logicalAxes){
        if(logicalIndex < 0 ||
                logicalIndex >= static_cast<int>(motorComType.size()) ||
                motorComType[logicalIndex] != COM_EC_LS){
            allOk = false;
            continue;
        }
        const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
        if (hardwareAxis < 0) {
            allOk = false;
            if(logicalIndex < static_cast<int>(motorCurState.size())){
                motorCurState[logicalIndex] = false;
            }
            continue;
        }
        const short stopRet = dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_stop"));
        const short disableRet =
                nmc_set_axis_disable(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true, QStringLiteral("nmc_set_axis_disable"));
        allOk = stopRet == 0 && disableRet == 0 && allOk;
        if(logicalIndex < static_cast<int>(motorCurState.size())){
            motorCurState[logicalIndex] = false;
        }
        if(logicalIndex < static_cast<int>(motorSessionSafetyHomeTraceValid.size())){
            motorSessionSafetyHomeTraceValid[logicalIndex] = false;
        }
    }

    if(exportTraceWindowAfterStop){
        readRuntimeTraceCached();
        freezeMotorTracePositionWindowRecording();
    }
    exportMotorTracePositionWindowForEvent(QStringLiteral("emergency_stop"));

    pvtTraceStartDelayState.active = false;
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtTimeStamp.clear();
    activePvtStartMonotonicUs = 0;

    return allOk;
    });
}

bool HardwareInterface::clearLeadshineAxisErrorCode(int logicalIndex, bool emitErrors)
{
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        if (emitErrors) {
            emit displayInfoSignal("Leadshine card is not connected; cannot clear motor axis error.", "error");
        }
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if (hardwareAxis < 0) {
        if (emitErrors) {
            emit displayInfoSignal(QString("%1 has no valid controller axis; cannot clear axis error.")
                                       .arg(axisDisplayName(logicalIndex))
                                       .toStdString(),
                                   "error");
        }
        return false;
    }

    const short ret = nmc_clear_axis_errcode(0, static_cast<WORD>(hardwareAxis));
    recordCommunicationEvent(true, QStringLiteral("nmc_clear_axis_errcode"));
    QThread::msleep(20);
    refreshLeadshineMotorEnableState(logicalIndex);
    if (ret != 0 && emitErrors) {
        emit displayInfoSignal(QString("%1错误码清除失败，返回码%2")
                                   .arg(axisDisplayName(logicalIndex))
                                   .arg(ret)
                                   .toStdString(),
                               "error");
    }
    return ret == 0;
    });
}

bool HardwareInterface::clearLeadshineBusErrorCode(bool emitErrors)
{
    return runOnHardwareThread([&]() -> bool {
    if(!isConnectLS){
        if(emitErrors){
            emit displayInfoSignal("Leadshine card is not connected; cannot clear bus error.", "error");
        }
        return false;
    }

    WORD busErrCode = 0;
    const short readRet = nmc_get_errcode(0, kLeadshineEtherCatPort, &busErrCode);
    recordCommunicationEvent(true, QStringLiteral("nmc_get_errcode"));
    if(readRet != 0){
        if(emitErrors){
            emit displayInfoSignal(QString("Leadshine bus error read failed, code %1.")
                                   .arg(readRet).toStdString(),
                                   "error");
        }
        return false;
    }
    if(busErrCode == 0){
        if(emitErrors){
            emit displayInfoSignal("Leadshine bus has no error code to clear.", "normal");
        }
        return true;
    }

    const short clearRet = nmc_clear_errcode(0, kLeadshineEtherCatPort);
    recordCommunicationEvent(true, QStringLiteral("nmc_clear_errcode"));
    QThread::msleep(20);
    if(emitErrors){
        emit displayInfoSignal(QString("Leadshine bus error %1 clear returned %2.")
                               .arg(busErrCode)
                               .arg(clearRet)
                               .toStdString(),
                               clearRet == 0 ? "warning" : "error");
    }
    return clearRet == 0;
    });
}

bool HardwareInterface::clearAllLeadshineAxisErrorCodes()
{
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        emit displayInfoSignal("Leadshine card is not connected; cannot clear motor axis errors.", "error");
        return false;
    }

    bool hasLeadshineAxis = false;
    bool allOk = true;
    int clearedAxisCount = 0;
    const int axisConfigCount = std::min(static_cast<int>(motorComType.size()),
                                         static_cast<int>(motorIdVec.size()));
    for (int i = 0; i < axisConfigCount; ++i) {
        if (motorComType[i] != COM_EC_LS) {
            continue;
        }

        hasLeadshineAxis = true;
        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        if (hardwareAxis < 0) {
            emit displayInfoSignal(QString("%1未配置有效的控制卡轴号，无法清除轴错误码")
                                       .arg(axisDisplayName(i))
                                       .toStdString(),
                                   "error");
            allOk = false;
            continue;
        }

        const short ret = nmc_clear_axis_errcode(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true, QStringLiteral("nmc_clear_axis_errcode"));
        if (ret != 0) {
            emit displayInfoSignal(QString("%1错误码清除失败，返回码%2")
                                       .arg(axisDisplayName(i))
                                       .arg(ret)
                                       .toStdString(),
                                   "error");
            allOk = false;
            continue;
        }

        ++clearedAxisCount;
    }

    if (!hasLeadshineAxis) {
        emit displayInfoSignal("No Leadshine motor axis is configured; cannot clear axis errors.", "error");
        return false;
    }

    if (allOk) {
        emit displayInfoSignal(QString("Safety reset cleared %1 Leadshine motor axis errors.")
                                   .arg(clearedAxisCount)
                                   .toStdString(),
                               "info");
    }

    return allOk;
    });
}

bool HardwareInterface::isLSConnected() const {
    return isConnectLS.load(std::memory_order_acquire);
}

bool HardwareInterface::isMotorDone(int index) const {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS || index < 0 || index >= static_cast<int>(motorComType.size())) {
        return true;
    }
    if (motorComType[index] != COM_EC_LS) {
        return true;
    }
    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        return true;
    }
    return dmc_check_done(0, static_cast<WORD>(hardwareAxis)) != 0;
    });
}

bool HardwareInterface::hasPvtTrajectory() const {
    return runOnHardwareThread([&]() -> bool {
    return hasActivePvtTrajectory && !activePvtMotorIndex.empty() && !activePvtMotorPosTraj.empty();
    });
}

bool HardwareInterface::isPvtMotionRunning() const {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS || !hasPvtTrajectory() || isPvtMotionPaused) {
        return false;
    }

    for (int axis : activePvtMotorIndex) {
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (hardwareAxis < 0) {
            continue;
        }
        if (dmc_check_done(0, static_cast<WORD>(hardwareAxis)) == 0) {
            return true;
        }
    }
    return false;
    });
}

bool HardwareInterface::isPvtMotionPausedState() const {
    return runOnHardwareThread([&]() -> bool {
    return isPvtMotionPaused;
    });
}

bool HardwareInterface::isPvtMotionFinished() const {
    return runOnHardwareThread([&]() -> bool {
    if (!hasPvtTrajectory()) {
        return true;
    }

    if (isPvtMotionPaused) {
        return false;
    }

    for (int axis : activePvtMotorIndex) {
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (hardwareAxis < 0) {
            continue;
        }
        if (dmc_check_done(0, static_cast<WORD>(hardwareAxis)) == 0) {
                return false;
        }
    }

    return true;
    });
}

bool HardwareInterface::currentPvtProgress(double& currentTrajectoryTime, std::size_t& currentIndex) const {
    return runOnHardwareThread([&]() -> bool {
        return getPvtCurrentProgress(currentTrajectoryTime, currentIndex);
    });
}

qint64 HardwareInterface::activePvtStartTimeMonotonicUs() const {
    return runOnHardwareThread([&]() -> qint64 {
        return activePvtStartMonotonicUs;
    });
}

void HardwareInterface::clearPvtTrajectoryState() {
    return runOnHardwareThread([&]() {
    freezeMotorTracePositionWindowRecording();
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtMotorVelMax.clear();
    activePvtTimeStamp.clear();
    activePvtStartMonotonicUs = 0;
    });
}

bool HardwareInterface::pausePvtMotion() {
    return runOnHardwareThread([&]() -> bool {
    return smoothPausePvtMotion(0.2);
    });
}

bool HardwareInterface::resumePvtMotion() {
    return runOnHardwareThread([&]() -> bool {
    return smoothResumePvtMotion(0.5);
    });
}

HardwareInterface::PvtPauseResult HardwareInterface::lastPvtPauseResult() const {
    return runOnHardwareThread([&]() -> PvtPauseResult {
        return lastPvtPauseResultData;
    });
}

bool HardwareInterface::smoothPausePvtMotion(double transitionTimeSec) {
    return runOnHardwareThread([&]() -> bool {
    lastPvtPauseResultData = PvtPauseResult();
    if (!isConnectLS) {
        emit displayInfoSignal("当前未连接雷赛控制卡", "error");
        return false;
    }
    if (!hasPvtTrajectory()) {
        emit displayInfoSignal("当前没有可暂停的 PVT 位置轨迹", "warning");
        return false;
    }
    if (isPvtMotionPaused) {
        emit displayInfoSignal("PVT position trajectory is already paused.", "warning");
        return true;
    }
    if (!isPvtMotionRunning()) {
        hasActivePvtTrajectory = false;
        pausedPvtResumeIndex = 0;
        pausedPvtResumeTime = 0.0;
        activePvtMotorIndex.clear();
        activePvtMotorPosTraj.clear();
        activePvtTimeStamp.clear();
        activePvtStartMonotonicUs = 0;
        emit displayInfoSignal("当前 PVT 位置轨迹未在运动，不能执行速度置零", "warning");
        return false;
    }

    double currentTrajectoryTime = 0.0;
    std::size_t currentIndex = 0;
    if (!getPvtCurrentProgress(currentTrajectoryTime, currentIndex)) {
        emit displayInfoSignal("PVT pause failed: cannot read current trajectory progress.", "error");
        return false;
    }
    lastPvtPauseResultData.beforeProgressValid = true;
    lastPvtPauseResultData.beforeTrajectoryTime = currentTrajectoryTime;
    lastPvtPauseResultData.beforeIndex = static_cast<int>(currentIndex);

    struct DecStopAxis {
        int logicalAxis = -1;
        int hardwareAxis = -1;
    };
    std::vector<DecStopAxis> decStopAxes;
    decStopAxes.reserve(activePvtMotorIndex.size());
    for (int axis : activePvtMotorIndex) {
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (hardwareAxis < 0) {
            continue;
        }
        decStopAxes.push_back({axis, hardwareAxis});
    }
    if (decStopAxes.empty()) {
        emit displayInfoSignal("PVT pause failed: no valid Leadshine axis for deceleration stop.", "error");
        return false;
    }

    const WORD cardNo = 0;
    const double stopDecTimeSec = transitionTimeSec > 1e-6 ? transitionTimeSec : 0.2;
    constexpr double kDefaultDecStopTimeSec = 0.001;
    bool decStopSetOk = true;
    for (const DecStopAxis& axis : decStopAxes) {
        const short err = dmc_set_dec_stop_time(cardNo,
                                                static_cast<WORD>(axis.hardwareAxis),
                                                stopDecTimeSec);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_dec_stop_time"));
        if (err != 0) {
            decStopSetOk = false;
            emit displayInfoSignal(QString("PVT 减速停止时间设置失败，%1，错误码%2")
                                       .arg(axisDisplayName(axis.logicalAxis))
                                       .arg(err)
                                       .toStdString(), "error");
        }
    }
    if (!decStopSetOk) {
        for (const DecStopAxis& axis : decStopAxes) {
            dmc_set_dec_stop_time(cardNo,
                                  static_cast<WORD>(axis.hardwareAxis),
                                  kDefaultDecStopTimeSec);
            recordCommunicationEvent(true, QStringLiteral("dmc_set_dec_stop_time"));
        }
        return false;
    }

    auto restoreDecStopTime = [&]() {
        for (const DecStopAxis& axis : decStopAxes) {
            const short err = dmc_set_dec_stop_time(cardNo,
                                                    static_cast<WORD>(axis.hardwareAxis),
                                                    kDefaultDecStopTimeSec);
            recordCommunicationEvent(true, QStringLiteral("dmc_set_dec_stop_time"));
            if (err != 0) {
                emit displayInfoSignal(QString("PVT 停止后恢复减速停止时间失败，%1，错误码%2")
                                           .arg(axisDisplayName(axis.logicalAxis))
                                           .arg(err)
                                           .toStdString(), "warning");
            }
        }
    };

    const int stopWaitMs = static_cast<int>(std::ceil(std::max(transitionTimeSec, 0.1) * 1000.0)) + 500;
    if (!stopActivePvtAxesAndWait(stopWaitMs)) {
        restoreDecStopTime();
        emit displayInfoSignal("PVT 轨迹暂停失败：原 PVT 轨迹未能可靠停止", "error");
        return false;
    }
    restoreDecStopTime();

    double stoppedTrajectoryTime = currentTrajectoryTime;
    std::size_t stoppedIndex = currentIndex;
    if (getPvtCurrentProgress(stoppedTrajectoryTime, stoppedIndex)) {
        lastPvtPauseResultData.afterProgressValid = true;
        lastPvtPauseResultData.afterTrajectoryTime = stoppedTrajectoryTime;
        lastPvtPauseResultData.afterIndex = static_cast<int>(stoppedIndex);
        currentTrajectoryTime = stoppedTrajectoryTime;
        currentIndex = stoppedIndex;
    }

    hasActivePvtTrajectory = true;
    pausedPvtResumeTime = currentTrajectoryTime;
    auto resumeIt = std::lower_bound(activePvtTimeStamp.begin(), activePvtTimeStamp.end(), pausedPvtResumeTime);
    pausedPvtResumeIndex = static_cast<std::size_t>(std::distance(activePvtTimeStamp.begin(), resumeIt));
    if (pausedPvtResumeIndex >= activePvtMotorPosTraj.size()) {
        hasActivePvtTrajectory = false;
        isPvtMotionPaused = false;
        pausedPvtResumeIndex = 0;
        pausedPvtResumeTime = 0.0;
        emit displayInfoSignal("PVT 位置轨迹已接近终点，无剩余轨迹可复位继续", "warning");
        return false;
    }

    isPvtMotionPaused = true;
    lastPvtPauseResultData.success = true;
    emit displayInfoSignal(QString("PVT 位置轨迹已按控制卡减速停，续跑时间%1s，续跑点索引%2")
                               .arg(pausedPvtResumeTime, 0, 'f', 3)
                               .arg(static_cast<int>(pausedPvtResumeIndex))
                               .toStdString(), "warning");
    return true;
    });
}

bool HardwareInterface::smoothResumePvtMotion(double transitionTimeSec) {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        emit displayInfoSignal("当前未连接雷赛控制卡", "error");
        return false;
    }
    if (!isPvtMotionPaused || activePvtMotorIndex.empty() || activePvtMotorPosTraj.empty()) {
        emit displayInfoSignal("当前没有已暂停的 PVT 位置轨迹", "warning");
        return false;
    }
    if (activePvtMotorVelTraj.size() < activePvtMotorPosTraj.size()) {
        emit displayInfoSignal("PVT velocity trajectory data is invalid; cannot continue.", "error");
        return false;
    }

    const std::vector<int> sourceMotorIndex = activePvtMotorIndex;
    const std::vector<std::vector<double>> sourcePosTraj = activePvtMotorPosTraj;
    const std::vector<std::vector<double>> sourceVelTraj = activePvtMotorVelTraj;
    const std::vector<double> sourceVelMax = activePvtMotorVelMax;
    const std::vector<double> sourceTimeStamp = activePvtTimeStamp;

    const double resumeTime = pausedPvtResumeTime > 0.0 ? pausedPvtResumeTime :
                              sourceTimeStamp[std::min(pausedPvtResumeIndex, sourceTimeStamp.size() - 1)];
    if (resumeTime >= sourceTimeStamp.back() - 1e-9) {
        hasActivePvtTrajectory = false;
        isPvtMotionPaused = false;
        pausedPvtResumeIndex = 0;
        pausedPvtResumeTime = 0.0;
        emit displayInfoSignal("PVT 位置轨迹没有剩余点可继续执行", "warning");
        return false;
    }

    const std::vector<double> currentPos = readMotorPositions(sourceMotorIndex);
    if (currentPos.size() != sourceMotorIndex.size()) {
        emit displayInfoSignal("PVT resume failed: cannot read current motor position.", "error");
        return false;
    }

    const int resumeIndex = pausedPvtResumeIndex;
    const int remainingCount = static_cast<int>(sourceTimeStamp.size()) - resumeIndex;
    if (remainingCount <= 0) {
        emit displayInfoSignal("PVT 位置轨迹没有剩余点可继续执行", "warning");
        return false;
    }

    std::vector<std::vector<double>> remainPosTraj(remainingCount);
    std::vector<std::vector<double>> remainVelTraj(remainingCount);
    std::vector<double> remainTimeStamp(remainingCount);
    const double timeBase = sourceTimeStamp[resumeIndex];
    for (int i = 0; i < remainingCount; ++i) {
        remainPosTraj[i] = sourcePosTraj[resumeIndex + i];
        remainVelTraj[i] = sourceVelTraj[resumeIndex + i];
        remainTimeStamp[i] = sourceTimeStamp[resumeIndex + i] - timeBase;
    }
    remainTimeStamp[0] = 0.0;

    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    const std::vector<double> beginVel(sourceMotorIndex.size(), 0.0);
    const std::vector<double> endVel(sourceMotorIndex.size(), 0.0);
    return startPvtTable(sourceMotorIndex,
                         remainPosTraj,
                         remainVelTraj,
                         sourceVelMax,
                         remainTimeStamp,
                         beginVel,
                         endVel,
                         true,
                         QStringLiteral("PVT position trajectory resumed"));
    });
}

std::vector<double> HardwareInterface::readMotorPositions(const std::vector<int>& motorIndex) const {
    std::vector<double> result;
    result.reserve(motorIndex.size());
    for (const int axis : motorIndex) {
        double pos = 0.0;
        if(!const_cast<HardwareInterface*>(this)->readMotorPositionUnitDirect(axis, pos)){
            return {};
        }
        result.push_back(pos);
    }
    return result;
}

std::vector<double> HardwareInterface::readMotorSpeeds(const std::vector<int>& motorIndex) const {
    std::vector<double> result(motorIndex.size(), 0.0);
    for (int i = 0; i < static_cast<int>(motorIndex.size()); ++i) {
        const int axis = motorIndex[i];
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (hardwareAxis < 0) {
            return {};
        }
        dmc_read_current_speed_unit(0, static_cast<WORD>(hardwareAxis), &result[i]);
        const_cast<HardwareInterface*>(this)->recordCommunicationEvent(
                    false,
                    QStringLiteral("dmc_read_current_speed_unit"));
    }
    return result;
}

bool HardwareInterface::getPvtCurrentProgress(double& currentTrajectoryTime, std::size_t& currentIndex) const {
    if (!hasPvtTrajectory() || activePvtTimeStamp.empty()) {
        return false;
    }

    const WORD cardNo = 0;
    DWORD minRunIndex = static_cast<DWORD>(activePvtTimeStamp.size() - 1);
    bool hasRunIndex = false;
    for (int axis : activePvtMotorIndex) {
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (hardwareAxis < 0) {
            continue;
        }
        DWORD runIndex = 0;
        const short indexErr = dmc_pvt_get_run_index(cardNo, static_cast<WORD>(hardwareAxis), &runIndex);
        const_cast<HardwareInterface*>(this)->recordCommunicationEvent(
                    false,
                    QStringLiteral("dmc_pvt_get_run_index"));
        if (indexErr == 0) {
            minRunIndex = hasRunIndex ? std::min(minRunIndex, runIndex) : runIndex;
            hasRunIndex = true;
        }
    }

    if (!hasRunIndex) {
        return false;
    }

    currentIndex = std::min<std::size_t>(static_cast<std::size_t>(minRunIndex), activePvtTimeStamp.size() - 1);
    currentTrajectoryTime = activePvtTimeStamp[currentIndex];
    return true;
}

bool HardwareInterface::waitPvtAxesDone(int timeoutMs) {
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        bool allDone = true;
        for (int axis : activePvtMotorIndex) {
            const int hardwareAxis = resolveLeadshineAxisIndex(axis);
            if (hardwareAxis < 0) {
                continue;
            }
            if (dmc_check_done(0, static_cast<WORD>(hardwareAxis)) == 0) {
                const_cast<HardwareInterface*>(this)->recordCommunicationEvent(
                            false,
                            QStringLiteral("dmc_check_done"));
                allDone = false;
                break;
            }
            const_cast<HardwareInterface*>(this)->recordCommunicationEvent(
                        false,
                        QStringLiteral("dmc_check_done"));
        }
        if (allDone) {
            return true;
        }
        delay(10);
    }
    return false;
}

bool HardwareInterface::stopActivePvtAxesAndWait(int timeoutMs) {
    if (activePvtMotorIndex.empty()) {
        return false;
    }

    bool stopCmdOk = true;
    for (int axis : activePvtMotorIndex) {
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (hardwareAxis < 0) {
            continue;
        }
        const short err = dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_stop"));
        if (err != 0) {
            stopCmdOk = false;
            emit displayInfoSignal(QString("PVT 原轨迹停止失败，%1，错误码%2")
                                       .arg(axisDisplayName(axis))
                                       .arg(err)
                                       .toStdString(), "error");
        }
    }

    if (!stopCmdOk) {
        return false;
    }
    return waitPvtAxesDone(timeoutMs);
}

// 下发并启动多轴 PVT 表。函数内分为数据校验、逐轴写表、启动命令、启动后握手诊断和活动轨迹缓存五个阶段。
bool HardwareInterface::startPvtTable(const std::vector<int>& motorIndex,
                                      const std::vector<std::vector<double>>& motorPosTraj,
                                      const std::vector<std::vector<double>>& motorVelTraj,
                                      const std::vector<double>& motorVelMax,
                                      const std::vector<double>& timeStamp,
                                      const std::vector<double>& beginVel,
                                      const std::vector<double>& endVel,
                                      bool updateActiveTrajectory,
                                      const QString& successMessage) {
    if (!isConnectLS) {
        emit displayInfoSignal("当前未连接雷赛控制卡", "error");
        return false;
    }
    if (motorIndex.empty() || motorPosTraj.size() < 2 || timeStamp.size() < 2) {
        emit displayInfoSignal("PVT position trajectory is empty or has insufficient points.", "error");
        return false;
    }
    if (motorPosTraj.size() != timeStamp.size()) {
        emit displayInfoSignal("PVT position point count does not match timestamp count.", "error");
        return false;
    }
    if(motorPosTraj.size() > kLeadshinePvtTableCapacity){
        emit displayInfoSignal(QString("PVT position trajectory not sent: point count %1 exceeds controller table capacity %2. Split the trajectory before sending.")
                               .arg(static_cast<qulonglong>(motorPosTraj.size()))
                               .arg(static_cast<qulonglong>(kLeadshinePvtTableCapacity))
                               .toStdString(),
                               "error");
        return false;
    }
    if (motorVelTraj.size() < motorPosTraj.size()) {
        emit displayInfoSignal("PVT 速度轨迹数据不足", "error");
        return false;
    }

    // 第一阶段：统一检查点数、维度、时间戳和软件限位，所有问题都在写硬件前返回。
    const DWORD pointCount = static_cast<DWORD>(
                std::min<std::size_t>(motorPosTraj.size(), kLeadshinePvtTableCapacity));
    for (DWORD pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
        if (motorPosTraj[pointIndex].size() < motorIndex.size()) {
            emit displayInfoSignal("PVT position dimension does not match motor count.", "error");
            return false;
        }
        if (motorVelTraj[pointIndex].size() < motorIndex.size()) {
            emit displayInfoSignal("PVT velocity dimension does not match motor count.", "error");
            return false;
        }
        if (pointIndex > 0 && timeStamp[pointIndex] <= timeStamp[pointIndex - 1]) {
            emit displayInfoSignal("PVT 时间戳必须严格递增", "error");
            return false;
        }

    }

    for(int axis = 0; axis < static_cast<int>(motorComType.size()); ++axis){
        if(motorComType[axis] != COM_EC_LS){
            continue;
        }
        QString currentLimitError;
        if(!validateCurrentMotorSafetyLimitForAutomaticMotion(
                    axis,
                    QStringLiteral("PVT位置轨迹"),
                    &currentLimitError)){
            emit displayInfoSignal(currentLimitError.toStdString(), "error");
            return false;
        }
    }

    for (int axisVecIndex = 0; axisVecIndex < static_cast<int>(motorIndex.size()); ++axisVecIndex) {
        const int axis = motorIndex[axisVecIndex];
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (axis < 0 || axis >= static_cast<int>(motorComType.size()) || axis >= static_cast<int>(motorHomePos.size())) {
            emit displayInfoSignal("PVT position trajectory not sent: motor axis is out of range.", "error");
            return false;
        }
        if (motorComType[axis] != COM_EC_LS) {
            emit displayInfoSignal("PVT 位置轨迹不会下发，当前仅支持雷赛电机控制", "error");
            return false;
        }
        if (hardwareAxis < 0) {
            emit displayInfoSignal(QString("PVT position trajectory not sent: %1 has no valid controller axis.")
                                       .arg(axisDisplayName(axis))
                                       .toStdString(), "error");
            return false;
        }
        double currentAbsoluteForLimit = std::numeric_limits<double>::quiet_NaN();
        double currentSafetyRelativeForLimit = std::numeric_limits<double>::quiet_NaN();
        const bool hasSafetyTargetBase =
                readMotorPositionUnitDirect(axis, currentAbsoluteForLimit, false) &&
                readMotorSafetyRelativePositionDirect(axis, currentSafetyRelativeForLimit) &&
                std::isfinite(currentAbsoluteForLimit) &&
                std::isfinite(currentSafetyRelativeForLimit);
        for (DWORD pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
            QString limitError;
            const double absoluteTarget =
                    motorHomePos[axis] + motorPosTraj[pointIndex][axisVecIndex];
            const bool limitOk = hasSafetyTargetBase ?
                        validateSafetyRelativeMotorSoftwareLimit(
                            axis,
                            currentSafetyRelativeForLimit +
                                (absoluteTarget - currentAbsoluteForLimit),
                            QStringLiteral("PVT浣嶇疆杞ㄨ抗"),
                            &limitError) :
                        validateAbsoluteMotorSoftwareLimit(
                            axis,
                            absoluteTarget,
                        QStringLiteral("PVT位置轨迹"),
                            &limitError);
            if(!limitOk){
                emit displayInfoSignal(QString("%1，轨迹点%2")
                                           .arg(limitError)
                                           .arg(static_cast<int>(pointIndex))
                                           .toStdString(),
                                       "error");
                return false;
            }
            const double pvtVelocity = motorVelTraj[pointIndex][axisVecIndex];
            if(!std::isfinite(pvtVelocity)){
                emit displayInfoSignal(QString("PVT 轨迹不会下发：%1在轨迹点%2处的PVT表速度无效，请重新规划")
                                           .arg(axisDisplayName(axis))
                                           .arg(static_cast<int>(pointIndex))
                                           .toStdString(),
                                       "error");
                return false;
            }
            double maxVelocity = 0.0;
            if(axisVecIndex < static_cast<int>(motorVelMax.size()) &&
                    std::isfinite(motorVelMax[axisVecIndex]) &&
                    motorVelMax[axisVecIndex] > 0.0){
                maxVelocity = motorVelMax[axisVecIndex];
            }
            else if(axis < static_cast<int>(motorSoftwareMaxVel.size())){
                maxVelocity = motorSoftwareMaxVel[axis];
            }
            if(std::isfinite(maxVelocity) &&
                    maxVelocity > 1e-9 &&
                    std::fabs(pvtVelocity) > maxVelocity){
                emit displayInfoSignal(QString("警告：PVT 轨迹不会下发，PVT表中%1在轨迹点%2处的电机速度%3 unit/s超过嵌入式模块速度上限%4 unit/s，请重新规划")
                                           .arg(axisDisplayName(axis))
                                           .arg(static_cast<int>(pointIndex))
                                           .arg(pvtVelocity, 0, 'f', 6)
                                           .arg(maxVelocity, 0, 'f', 6)
                                           .toStdString(),
                                       "error");
                return false;
            }
        }
    }

    // 第二阶段：逐轴写入 PVT 表，并通过剩余空间做轻量 ACK 诊断。ACK 异常只告警，不阻断已经被板卡接受的启动流程。
    const WORD cardNo = 0;
    std::vector<WORD> axisList;
    axisList.reserve(motorIndex.size());
    struct PvtUploadAck {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        short remainBefore = -1;
        short remainAfter = -1;
        short expectedRemain = -1;
    };
    std::vector<PvtUploadAck> uploadAcks;
    uploadAcks.reserve(motorIndex.size());
    bool uploadAckAllOk = true;
    qint64 pvtTableUploadTotalUs = 0;
    qint64 pvtTableUploadLastMonotonicUs = 0;

    auto queryPvtRemainSpace = [&](int logicalAxis,
                                   int hardwareAxis,
                                   const QString& phase,
                                   short& remainSpace) -> bool {
        remainSpace = -1;
        for(int retryIndex = 0; retryIndex < kPvtUploadAckRetryCount; ++retryIndex){
            const short value =
                    dmc_pvt_get_remain_space(cardNo, static_cast<WORD>(hardwareAxis));
            recordCommunicationEvent(false, QStringLiteral("dmc_pvt_get_remain_space"));
            if(value >= 0 &&
                    value <= static_cast<short>(kLeadshinePvtTableCapacity)){
                remainSpace = value;
                return true;
            }
            if(retryIndex + 1 < kPvtUploadAckRetryCount){
                delay(kPvtUploadAckRetryDelayMs);
            }
        }

        emit displayInfoSignal(QString("PVT 握手诊断：%1在%2阶段无法读取控制卡PVT表剩余空间，本次仅记录告警，不阻止启动")
                                   .arg(axisDisplayName(logicalAxis))
                                   .arg(phase)
                                   .toStdString(),
                               "warning");
        return false;
    };

    for (int axisVecIndex = 0; axisVecIndex < static_cast<int>(motorIndex.size()); ++axisVecIndex) {
        const int axis = motorIndex[axisVecIndex];
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (axis < 0 || axis >= static_cast<int>(motorComType.size()) || axis >= static_cast<int>(motorHomePos.size())) {
            emit displayInfoSignal("PVT 电机编号超出范围", "error");
            return false;
        }
        if (motorComType[axis] != COM_EC_LS) {
            emit displayInfoSignal("Only Leadshine motor control is supported here.", "error");
            return false;
        }

        if (hardwareAxis < 0) {
            emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(axis)).toStdString(), "error");
            return false;
        }

        PvtUploadAck ack;
        ack.logicalAxis = axis;
        ack.hardwareAxis = hardwareAxis;
        if(kEnablePvtHandshakeDiagnostics &&
                !queryPvtRemainSpace(axis,
                                     hardwareAxis,
                                     QStringLiteral("下发前"),
                                     ack.remainBefore)){
            uploadAckAllOk = false;
        }

        // 雷赛 PVT 表使用相对时间和相对位置，所以每根轴都以该表首点为局部零点。
        std::vector<double> pTime(pointCount, 0.0);
        std::vector<double> pPos(pointCount, 0.0);
        const double timeBase = timeStamp[0];
        const double posBase = motorPosTraj[0][axisVecIndex];
        for (DWORD pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
            pTime[pointIndex] = timeStamp[pointIndex] - timeBase;
            pPos[pointIndex] = motorPosTraj[pointIndex][axisVecIndex] - posBase;
        }
        pTime[0] = 0.0;
        pPos[0] = 0.0;

        const double axisBeginVel = axisVecIndex < static_cast<int>(beginVel.size()) ? beginVel[axisVecIndex] : 0.0;
        const double axisEndVel = axisVecIndex < static_cast<int>(endVel.size()) ? endVel[axisVecIndex] : 0.0;
        const qint64 uploadCallStartUs = monotonicNowUs();
        const short err = dmc_pvts_table_unit(cardNo,
                                             static_cast<WORD>(hardwareAxis),
                                             pointCount,
                                             pTime.data(),
                                             pPos.data(),
                                             axisBeginVel,
                                             axisEndVel);
        const qint64 uploadCallElapsedUs =
                std::max<qint64>(0, monotonicNowUs() - uploadCallStartUs);
        pvtTableUploadTotalUs += uploadCallElapsedUs;
        pvtTableUploadLastMonotonicUs = uploadCallStartUs + uploadCallElapsedUs;
        recordCommunicationEvent(true, QStringLiteral("dmc_pvts_table_unit"));
        if (err != 0) {
            emit displayInfoSignal(QString("PVT 数据表下发失败，%1，错误码%2，首点pos=%3，首点time=%4")
                                       .arg(axisDisplayName(axis))
                                       .arg(err)
                                       .arg(pPos[0])
                                       .arg(pTime[0])
                                       .toStdString(), "error");
            return false;
        }
        if(kEnablePvtHandshakeDiagnostics){
            ack.expectedRemain = static_cast<short>(
                        kLeadshinePvtTableCapacity > pointCount ?
                            kLeadshinePvtTableCapacity - pointCount :
                            0);
            bool uploadAckOk = false;
            for(int retryIndex = 0; retryIndex < kPvtUploadAckRetryCount; ++retryIndex){
                if(!queryPvtRemainSpace(axis,
                                        hardwareAxis,
                                        QStringLiteral("下发后"),
                                        ack.remainAfter)){
                    uploadAckAllOk = false;
                    break;
                }
                if(ack.remainAfter == ack.expectedRemain){
                    uploadAckOk = true;
                    break;
                }
                if(retryIndex + 1 < kPvtUploadAckRetryCount){
                    delay(kPvtUploadAckRetryDelayMs);
                }
            }
            if(!uploadAckOk){
                emit displayInfoSignal(QString("PVT 握手诊断：%1数据表点数ACK不一致，发送点数%2，期望剩余空间%3，实际剩余空间%4，下发前剩余空间%5。本次仅记录告警，不阻止启动。")
                                           .arg(axisDisplayName(axis))
                                           .arg(static_cast<qulonglong>(pointCount))
                                           .arg(ack.expectedRemain)
                                           .arg(ack.remainAfter)
                                           .arg(ack.remainBefore)
                                           .toStdString(),
                                       "warning");
                uploadAckAllOk = false;
            }

            uploadAcks.push_back(ack);
        }
        axisList.push_back(static_cast<WORD>(hardwareAxis));
    }

    if (axisList.empty()) {
        emit displayInfoSignal("PVT axis list is empty; motion not started.", "error");
        return false;
    }

    const bool requiresFreshSessionTrace =
            std::any_of(motorIndex.begin(),
                        motorIndex.end(),
                        [&](int axis) {
        return hasValidMotorSessionSafetyTraceHome(axis);
    });
    if(requiresFreshSessionTrace){
        QElapsedTimer traceCatchUpTimer;
        traceCatchUpTimer.start();
        bool sessionTraceReady = false;
        do{
            readRuntimeTraceCached(false);
            const qint64 nowUs = monotonicNowUs();
            const qint64 futureToleranceUs = std::max<qint64>(
                        2 * 1000,
                        static_cast<qint64>(runtimeTraceSamplePeriodUs) *
                            kRuntimeTraceTimestampFutureToleranceFrames);
            const bool latestFrameFresh =
                    runtimeTraceConfigReadbackValid &&
                    runtimeTraceTimingReliable &&
                    runtimeTraceFifoCaughtUp &&
                    !runtimeTraceLost &&
                    runtimeTraceNewestFrameAgeUs >= 0 &&
                    runtimeTraceNewestFrameAgeUs <=
                        kMotorPositionTraceFreshTimeoutUs &&
                    latestMotorTracePositionFrameValid &&
                    latestMotorTracePositionFrame.monotonicUs > 0 &&
                    nowUs + futureToleranceUs >=
                        latestMotorTracePositionFrame.monotonicUs &&
                    (nowUs < latestMotorTracePositionFrame.monotonicUs ||
                     nowUs - latestMotorTracePositionFrame.monotonicUs <=
                        kMotorPositionTraceFreshTimeoutUs);
            sessionTraceReady = latestFrameFresh;
            if(sessionTraceReady){
                for(const int axis : motorIndex){
                    if(!hasValidMotorSessionSafetyTraceHome(axis)){
                        continue;
                    }
                    double relativePosition =
                            std::numeric_limits<double>::quiet_NaN();
                    if(!motorSafetyRelativeFromTraceFrame(
                                axis,
                                latestMotorTracePositionFrame,
                                relativePosition) ||
                            !std::isfinite(relativePosition)){
                        sessionTraceReady = false;
                        break;
                    }
                }
            }
            if(!sessionTraceReady &&
                    traceCatchUpTimer.elapsed() <
                        kPvtPreStartTraceCatchUpTimeoutMs){
                delay(kPvtPreStartTraceCatchUpPollMs);
            }
        } while(!sessionTraceReady &&
                traceCatchUpTimer.elapsed() <
                    kPvtPreStartTraceCatchUpTimeoutMs);

        if(!sessionTraceReady){
            const qint64 nowUs = monotonicNowUs();
            const qint64 frameDeltaUs =
                    latestMotorTracePositionFrame.monotonicUs > 0 ?
                        nowUs - latestMotorTracePositionFrame.monotonicUs :
                        -1;
            emit displayInfoSignal(
                        QString("PVT position trajectory not started: Runtime Trace did not catch up after table upload; fifo_valid=%1 newest_age_us=%2 frame_delta_us=%3 caught_up=%4 timing_reliable=%5 trace_lost=%6.")
                        .arg(runtimeTraceLastFifoValidNum)
                        .arg(runtimeTraceNewestFrameAgeUs)
                        .arg(frameDeltaUs)
                        .arg(runtimeTraceFifoCaughtUp ? 1 : 0)
                        .arg(runtimeTraceTimingReliable ? 1 : 0)
                        .arg(runtimeTraceLost ? 1 : 0)
                        .toStdString(),
                        "error");
            return false;
        }
    }

    // 所有轴的 PVT 表均已写入控制卡；该正常消息经 MainWindow::displayInfo
    // 同时进入信息栏、消息历史和 UI 事件日志。
    emit displayInfoSignal("电机运动指令生成成功", "info");

    // 第三阶段：所有轴表写完后统一启动，确保多轴同时进入同一段轨迹。
    if(kEnablePvtControlCycleDiagnostics &&
            pointCount > 0 &&
            pvtTableUploadLastMonotonicUs > 0){
        PvtTableUploadTimingSample timingSample;
        timingSample.wallClockUs = QDateTime::currentMSecsSinceEpoch() * 1000;
        timingSample.wallClockMs = timingSample.wallClockUs / 1000;
        timingSample.monotonicUs = pvtTableUploadLastMonotonicUs;
        timingSample.totalUploadUs = pvtTableUploadTotalUs;
        timingSample.averageUploadUsPerPoint =
                static_cast<double>(pvtTableUploadTotalUs) /
                static_cast<double>(pointCount);
        timingSample.pointCount = static_cast<int>(pointCount);
        timingSample.axisCount = static_cast<int>(axisList.size());
        timingSample.motorIndex = motorIndex;
        timingSample.source = successMessage;
        QMutexLocker locker(&diagnosticsMutex);
        if(shouldAppendDiagnosticRawSample(diagnosticRawHistoryFullRecordingEnabled,
                                           timingSample.wallClockUs,
                                           lastPvtTableUploadTimingHistoryAppendUs)){
            pvtTableUploadTimingSamples.append(timingSample);
            if(timingSample.wallClockMs - lastPvtTableUploadTimingHistoryTrimMs >=
                    kDiagnosticRawTrimIntervalMs){
                trimRawHistoryForMode(pvtTableUploadTimingSamples,
                                      timingSample.wallClockMs,
                                      diagnosticRawHistoryFullRecordingEnabled,
                                      kDiagnosticRawDefaultMaxSamples);
                lastPvtTableUploadTimingHistoryTrimMs = timingSample.wallClockMs;
            }
        }
    }

    // 在启动命令前刷新一次Trace，固定本次8个绳索轴的command/feedback基线。
    // 该读取不改变既有trace_raw_command缓存的用途，只为后续首次变化帧判定提供起点。
    pvtTraceStartDelayState = PvtTraceStartDelayState{};
    if(kEnablePvtControlCycleDiagnostics){
        readRuntimeTraceCached(false);
    }

    const short err = dmc_pvt_move(cardNo, static_cast<WORD>(axisList.size()), axisList.data());
    recordCommunicationEvent(true, QStringLiteral("dmc_pvt_move"));
    if (err != 0) {
        emit displayInfoSignal(QString("PVT 运动启动失败，错误码%1").arg(err).toStdString(), "error");
        return false;
    }

    if(kEnablePvtControlCycleDiagnostics){
        armPvtTraceStartDelayMeasurement(motorIndex,
                                         static_cast<int>(pointCount),
                                         pvtTableUploadLastMonotonicUs);
    }

    const qint64 pvtStartMonotonicUs = monotonicNowUs();
    beginMotorTracePositionWindowRecording();
    readRuntimeTraceCached(false);

    bool startAckOk = true;
    if(kEnablePvtHandshakeDiagnostics){
    struct PvtStartAck {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        DWORD runIndex = 0;
        short runIndexRet = -1;
        short doneState = -1;
    };
    std::vector<PvtStartAck> startAcks(motorIndex.size());
    auto readPvtStartAck = [&]() -> bool {
        bool allStarted = true;
        for(int axisVecIndex = 0; axisVecIndex < static_cast<int>(motorIndex.size()); ++axisVecIndex){
            const int axis = motorIndex[axisVecIndex];
            const int hardwareAxis = resolveLeadshineAxisIndex(axis);
            PvtStartAck ack;
            ack.logicalAxis = axis;
            ack.hardwareAxis = hardwareAxis;
            if(hardwareAxis < 0){
                allStarted = false;
                startAcks[axisVecIndex] = ack;
                continue;
            }

            ack.runIndexRet = dmc_pvt_get_run_index(cardNo,
                                                    static_cast<WORD>(hardwareAxis),
                                                    &ack.runIndex);
            recordCommunicationEvent(false, QStringLiteral("dmc_pvt_get_run_index"));
            ack.doneState = dmc_check_done(cardNo, static_cast<WORD>(hardwareAxis));
            recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
            startAcks[axisVecIndex] = ack;

            const bool runIndexReadable = ack.runIndexRet == 0;
            const bool motionStartedOrFinished =
                    ack.doneState == 0 ||
                    ack.runIndex > 0 ||
                    (ack.doneState == 1 && ack.runIndex >= pointCount - 1);
            if(!runIndexReadable || !motionStartedOrFinished){
                allStarted = false;
            }
        }
        return allStarted;
    };

    // 第四阶段：启动后短时间轮询 run index/done，帮助判断“板卡返回成功但轴未实际动”的异常。
    QElapsedTimer startAckTimer;
    startAckTimer.start();
    startAckOk = false;
    while(startAckTimer.elapsed() <= kPvtStartAckTimeoutMs){
        if(readPvtStartAck()){
            startAckOk = true;
            break;
        }
        delay(kPvtStartAckPollMs);
    }
    if(!startAckOk){
        QStringList axisStates;
        for(const PvtStartAck& ack : startAcks){
            axisStates << QString("%1(hw%2): runRet=%3 runIndex=%4 done=%5")
                          .arg(axisDisplayName(ack.logicalAxis))
                          .arg(ack.hardwareAxis)
                          .arg(ack.runIndexRet)
                          .arg(static_cast<qulonglong>(ack.runIndex))
                          .arg(ack.doneState);
        }
        emit displayInfoSignal(QString("PVT 握手诊断：控制卡已返回启动成功，但%1 ms内未确认各轴开始执行PVT表。本次不阻止启动，状态：%2")
                                   .arg(kPvtStartAckTimeoutMs)
                                   .arg(axisStates.join(QStringLiteral("; ")))
                                   .toStdString(),
                               "warning");
    }
    }

    if (updateActiveTrajectory) {
        // 第五阶段：缓存活动轨迹，供暂停、恢复、进度显示和安全停机后的回零逻辑使用。
        activePvtMotorIndex = motorIndex;
        activePvtMotorPosTraj.assign(motorPosTraj.begin(), motorPosTraj.begin() + static_cast<std::ptrdiff_t>(pointCount));
        activePvtMotorVelTraj.assign(motorVelTraj.begin(), motorVelTraj.begin() + static_cast<std::ptrdiff_t>(pointCount));
        activePvtMotorVelMax.assign(motorIndex.size(), 0.0);
        for(int axisVecIndex = 0; axisVecIndex < static_cast<int>(motorIndex.size()); ++axisVecIndex){
            const int axis = motorIndex[axisVecIndex];
            if(axisVecIndex < static_cast<int>(motorVelMax.size()) &&
                    std::isfinite(motorVelMax[axisVecIndex]) &&
                    motorVelMax[axisVecIndex] > 0.0){
                activePvtMotorVelMax[axisVecIndex] = motorVelMax[axisVecIndex];
            }
            else if(axis >= 0 && axis < static_cast<int>(motorSoftwareMaxVel.size())){
                activePvtMotorVelMax[axisVecIndex] = motorSoftwareMaxVel[axis];
            }
        }
        activePvtTimeStamp.assign(timeStamp.begin(), timeStamp.begin() + static_cast<std::ptrdiff_t>(pointCount));
        activePvtStartMonotonicUs = pvtStartMonotonicUs;
        hasActivePvtTrajectory = true;
        isPvtMotionPaused = false;
        pausedPvtResumeIndex = 0;
        pausedPvtResumeTime = 0.0;
    }

    if(kEnablePvtHandshakeDiagnostics){
        QStringList uploadAckSummary;
        for(const PvtUploadAck& ack : uploadAcks){
            uploadAckSummary << QString("%1(hw%2): before=%3 after=%4 expected=%5")
                                .arg(axisDisplayName(ack.logicalAxis))
                                .arg(ack.hardwareAxis)
                                .arg(ack.remainBefore)
                                .arg(ack.remainAfter)
                                .arg(ack.expectedRemain);
        }
        const bool handshakeClean = uploadAckAllOk && startAckOk;
        emit displayInfoSignal(QString(handshakeClean ?
                                       "PVT 表握手通过：%1" :
                                       "PVT 表握手诊断完成但存在告警：%1")
                                   .arg(uploadAckSummary.join(QStringLiteral("; ")))
                                   .toStdString(),
                               handshakeClean ? "info" : "warning");
    }
    emit displayInfoSignal(successMessage.toStdString(), "info");
    return true;
}

bool HardwareInterface::motorStop(int index) {
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor control is supported here.", "error");
        return false;
    }
    // Hybrid force axes can be any selected logical axes; do not clear unrelated PVT axes.
    const bool stoppedAxisInActivePvt =
            std::find(activePvtMotorIndex.begin(), activePvtMotorIndex.end(), index) != activePvtMotorIndex.end();
    if(stoppedAxisInActivePvt){
        pvtTraceStartDelayState.active = false;
        hasActivePvtTrajectory = false;
        isPvtMotionPaused = false;
        pausedPvtResumeIndex = 0;
        pausedPvtResumeTime = 0.0;
        activePvtMotorIndex.clear();
        activePvtMotorPosTraj.clear();
        activePvtMotorVelTraj.clear();
        activePvtMotorVelMax.clear();
        activePvtTimeStamp.clear();
        activePvtStartMonotonicUs = 0;
    }
    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }
    const short stopErr = dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
    recordCommunicationEvent(true, QStringLiteral("dmc_stop"));
    if(index >= 0 && index < static_cast<int>(motorJogVelocityFastActive.size())){
        motorJogVelocityFastActive[index] = false;
    }
    if(stopErr != 0){
        emit displayInfoSignal(QString("Motor stop failed on %1, error %2.")
                                   .arg(axisDisplayName(index))
                                   .arg(stopErr)
                                   .toStdString(),
                               "error");
        return false;
    }
    return true;
    });
}

bool HardwareInterface::motorStopAxes(const std::vector<int>& motorIndex)
{
    return runOnHardwareThread([&]() -> bool {
        bool allOk = true;
        for(const int axisIndex : motorIndex){
            // 已经处于HardwareThread时，motorStop会直接执行，不会为每根轴
            // 再创建一个可被周期诊断穿插的BlockingQueuedConnection。
            allOk = motorStop(axisIndex) && allOk;
        }
        return allOk;
    });
}

bool HardwareInterface::motorHome(int index) {
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size()) || index >= static_cast<int>(motorHomePos.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor control is supported here.", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }

    double pos = 0.0;
    if(!readMotorPositionUnitDirect(index, pos)){
        emit displayInfoSignal(QString("Unit position read failed for %1.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(),
                               "error");
        return false;
    }
    double vel = std::fabs(pos - motorHomePos[index]) / 15.0;

    QString limitError;
    if(!validateAbsoluteMotorSoftwareLimit(index,
                                           motorHomePos[index],
                                           QStringLiteral("电机回零指令"),
                                           &limitError)){
        emit displayInfoSignal(limitError.toStdString(), "error");
        return false;
    }

    dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, std::max(std::fabs(vel), 1e-5), 0.1, 0.1, 0);
    recordCommunicationEvent(true, QStringLiteral("dmc_set_profile_unit"));
    dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
    recordCommunicationEvent(true, QStringLiteral("dmc_set_s_profile"));
    dmc_pmove_unit(0, static_cast<WORD>(hardwareAxis), motorHomePos[index], 1);
    recordCommunicationEvent(true, QStringLiteral("dmc_pmove_unit"));
    return true;
    });
}


bool HardwareInterface::motorAbsPos(int index, double pos, double vel) {
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor control is supported here.", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }

    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtMotorVelMax.clear();
    activePvtTimeStamp.clear();
    activePvtStartMonotonicUs = 0;

    QString limitError;
    if(!validateAbsoluteMotorSoftwareLimit(index,
                                           pos,
                                           QStringLiteral("电机点位运动指令"),
                                           &limitError)){
        emit displayInfoSignal(limitError.toStdString(), "error");
        return false;
    }

    const double safeVel = std::max(std::fabs(vel), 1e-5);
    const short doneState = dmc_check_done(0, static_cast<WORD>(hardwareAxis));
    recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
    if (doneState == 0) {
        const short stopErr = dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_stop"));
        if (stopErr != 0) {
            emit displayInfoSignal(QString("Motor axis %1 failed to stop before point move, error %2")
                                       .arg(index)
                                       .arg(stopErr)
                                       .toStdString(), "error");
            return false;
        }
        bool stopped = false;
        for (int retry = 0; retry < 100; ++retry) {
            const short stoppedState = dmc_check_done(0, static_cast<WORD>(hardwareAxis));
            recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
            if (stoppedState != 0) {
                stopped = true;
                break;
            }
            QThread::msleep(10);
        }
        if (!stopped) {
            emit displayInfoSignal(QString("Motor axis %1 did not stop before point move.")
                                       .arg(index)
                                       .toStdString(), "error");
            return false;
        }
    }
    {
        const short clearStopReasonErr = dmc_clear_stop_reason(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true, QStringLiteral("dmc_clear_stop_reason"));
        const short profileErr = dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, safeVel, 0.1, 0.1, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_profile_unit"));
        const short sErr = dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_s_profile"));
        const short moveErr = dmc_pmove_unit(0, static_cast<WORD>(hardwareAxis), pos, 1);
        recordCommunicationEvent(true, QStringLiteral("dmc_pmove_unit"));
        if (clearStopReasonErr != 0 || profileErr != 0 || sErr != 0 || moveErr != 0) {
            emit displayInfoSignal(QString("%1点位运动启动失败，错误码 clear=%2 profile=%3 s=%4 move=%5")
                                       .arg(axisDisplayName(index))
                                       .arg(clearStopReasonErr)
                                       .arg(profileErr)
                                       .arg(sErr)
                                       .arg(moveErr)
                                       .toStdString(), "error");
            return false;
        }
    }
    return true;
    });
}

bool HardwareInterface::motorRelativePos(int index, double dist, double vel) {
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor control is supported here.", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }
    if(!std::isfinite(dist)){
        emit displayInfoSignal(QString("%1 relative distance is invalid.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(), "error");
        return false;
    }

    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtMotorVelMax.clear();
    activePvtTimeStamp.clear();
    activePvtStartMonotonicUs = 0;

    double currentPos = 0.0;
    if(!readMotorPositionUnitDirect(index, currentPos)){
        emit displayInfoSignal(QString("Unit position read failed for %1.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(),
                               "error");
        return false;
    }

    QString limitError;
    if(!validateAbsoluteMotorSoftwareLimit(index,
                                           currentPos + dist,
                                           QStringLiteral("relative point move command"),
                                           &limitError)){
        emit displayInfoSignal(limitError.toStdString(), "error");
        return false;
    }

    const double safeVel = std::max(std::fabs(vel), 1e-5);
    const short doneState = dmc_check_done(0, static_cast<WORD>(hardwareAxis));
    recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
    if (doneState == 0) {
        const short stopErr = dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_stop"));
        if (stopErr != 0) {
            emit displayInfoSignal(QString("Motor axis %1 failed to stop before relative point move, error %2")
                                       .arg(index)
                                       .arg(stopErr)
                                       .toStdString(), "error");
            return false;
        }
        bool stopped = false;
        for (int retry = 0; retry < 100; ++retry) {
            const short stoppedState = dmc_check_done(0, static_cast<WORD>(hardwareAxis));
            recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
            if (stoppedState != 0) {
                stopped = true;
                break;
            }
            QThread::msleep(10);
        }
        if (!stopped) {
            emit displayInfoSignal(QString("Motor axis %1 did not stop before relative point move.")
                                       .arg(index)
                                       .toStdString(), "error");
            return false;
        }
    }
    {
        const short clearStopReasonErr = dmc_clear_stop_reason(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true, QStringLiteral("dmc_clear_stop_reason"));
        const short profileErr = dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, safeVel, 0.1, 0.1, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_profile_unit"));
        const short sErr = dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_s_profile"));
        const short moveErr = dmc_pmove_unit(0, static_cast<WORD>(hardwareAxis), dist, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_pmove_unit"));
        if (clearStopReasonErr != 0 || profileErr != 0 || sErr != 0 || moveErr != 0) {
            emit displayInfoSignal(QString("Motor axis %1 relative point move failed, error clear=%2 profile=%3 s=%4 move=%5")
                                       .arg(index)
                                       .arg(clearStopReasonErr)
                                       .arg(profileErr)
                                       .arg(sErr)
                                       .arg(moveErr)
                                       .toStdString(), "error");
            return false;
        }
    }
    return true;
    });
}


bool HardwareInterface::motorVel(int index, double vel) {
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor control is supported here.", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }

    double currentPos = 0.0;
    if(!readMotorPositionUnitDirect(index, currentPos)){
        emit displayInfoSignal(QString("Unit position read failed for %1.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(),
                               "error");
        return false;
    }
    QString limitError;
    if(!validateVelocityMotorSoftwareLimit(index,
                                           currentPos,
                                           vel,
                                           QStringLiteral("电机速度运动指令"),
                                           &limitError)){
        emit displayInfoSignal(limitError.toStdString(), "error");
        return false;
    }

    if (dmc_check_done(0, static_cast<WORD>(hardwareAxis))) {
        recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
        dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, std::max(std::fabs(vel), 1e-5), 0.1, 0.1, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_profile_unit"));
        dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_s_profile"));
        dmc_vmove(0, static_cast<WORD>(hardwareAxis), vel >= 0 ? 1 : 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_vmove"));
    } else {
        dmc_change_speed_unit(0, static_cast<WORD>(hardwareAxis), vel, velChangeSpd);
        recordCommunicationEvent(true, QStringLiteral("dmc_change_speed_unit"));
    }
    return true;
    });
}
// Batch JOG/continuous velocity command for online speed following.
bool HardwareInterface::motorVelBatch(const std::vector<int>& motorIndex,
                                      const std::vector<double>& velocity,
                                      double changeTimeSec)
{
    return runOnHardwareThread([&]() -> bool {
    if(motorIndex.size() != velocity.size()){
        emit displayInfoSignal(QString("Batch JOG velocity command size mismatch: axes=%1 velocities=%2.")
                                   .arg(static_cast<int>(motorIndex.size()))
                                   .arg(static_cast<int>(velocity.size()))
                                   .toStdString(),
                               "error");
        return false;
    }
    if(motorIndex.empty()){
        return true;
    }
    if(!isConnectLS){
        emit displayInfoSignal("Leadshine controller is not connected for batch JOG velocity command.", "error");
        return false;
    }

    struct BatchVelocityCommand {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        double velocity = 0.0;
        short doneState = 1;
    };

    std::vector<BatchVelocityCommand> commands;
    commands.reserve(motorIndex.size());
    for(int axisColumn = 0; axisColumn < static_cast<int>(motorIndex.size()); ++axisColumn){
        const int logicalAxis = motorIndex[axisColumn];
        const double signedVelocity = velocity[axisColumn];
        if(std::find(motorIndex.begin(), motorIndex.begin() + axisColumn, logicalAxis) !=
                motorIndex.begin() + axisColumn){
            emit displayInfoSignal(QString("Batch JOG velocity command has duplicate %1.")
                                       .arg(axisDisplayName(logicalAxis))
                                       .toStdString(),
                                   "error");
            return false;
        }
        if(!std::isfinite(signedVelocity)){
            emit displayInfoSignal(QString("Batch JOG velocity command for %1 is not finite.")
                                       .arg(axisDisplayName(logicalAxis))
                                       .toStdString(),
                                   "error");
            return false;
        }
        if(logicalAxis < 0 || logicalAxis >= static_cast<int>(motorComType.size())){
            emit displayInfoSignal(QString("Batch JOG velocity command has invalid %1.")
                                       .arg(axisDisplayName(logicalAxis))
                                       .toStdString(),
                                   "error");
            return false;
        }
        if(motorComType[logicalAxis] != COM_EC_LS){
            emit displayInfoSignal("Only Leadshine motor control is supported for batch JOG velocity command.",
                                   "error");
            return false;
        }

        const int hardwareAxis = resolveLeadshineAxisIndex(logicalAxis);
        if(hardwareAxis < 0){
            emit displayInfoSignal(QString("%1 has no valid controller axis.")
                                       .arg(axisDisplayName(logicalAxis))
                                       .toStdString(),
                                   "error");
            return false;
        }
        if(!refreshLeadshineMotorEnableState(logicalAxis)){
            emit displayInfoSignal(QString("Batch JOG velocity command rejected: %1 is not enabled.")
                                       .arg(axisDisplayName(logicalAxis))
                                       .toStdString(),
                                   "error");
            return false;
        }

        double currentPos = 0.0;
        if(!readMotorPositionUnitDirect(logicalAxis, currentPos)){
            emit displayInfoSignal(QString("Unit position read failed for %1 in batch JOG velocity command.")
                                       .arg(axisDisplayName(logicalAxis))
                                       .toStdString(),
                                   "error");
            return false;
        }
        QString limitError;
        if(!validateVelocityMotorSoftwareLimit(logicalAxis,
                                               currentPos,
                                               signedVelocity,
                                               QStringLiteral("batch JOG velocity command"),
                                               &limitError)){
            emit displayInfoSignal(limitError.toStdString(), "error");
            return false;
        }

        const short doneState = dmc_check_done(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
        BatchVelocityCommand command;
        command.logicalAxis = logicalAxis;
        command.hardwareAxis = hardwareAxis;
        command.velocity = signedVelocity;
        command.doneState = doneState;
        commands.push_back(command);
    }

    const double taccdec =
            (std::isfinite(changeTimeSec) && changeTimeSec >= 0.0) ?
                changeTimeSec :
                velChangeSpd;
    if(!std::isfinite(taccdec) || taccdec < 0.0){
        emit displayInfoSignal(QString("Batch JOG velocity command has invalid change time %1.")
                                   .arg(taccdec, 0, 'f', 6)
                                   .toStdString(),
                               "error");
        return false;
    }
    bool ok = true;
    for(const BatchVelocityCommand& command : commands){
        const WORD hardwareAxis = static_cast<WORD>(command.hardwareAxis);
        const double signedVelocity = command.velocity;
        if(std::fabs(signedVelocity) <= 1e-9){
            if(command.doneState == 0){
                const short stopErr = dmc_stop(0, hardwareAxis, 0);
                recordCommunicationEvent(true, QStringLiteral("dmc_stop"));
                if(stopErr != 0){
                    emit displayInfoSignal(QString("Batch JOG velocity stop failed on %1, error %2.")
                                               .arg(axisDisplayName(command.logicalAxis))
                                               .arg(stopErr)
                                               .toStdString(),
                                           "error");
                    ok = false;
                }
            }
            continue;
        }

        if(command.doneState != 0){
            const short clearStopReasonErr = dmc_clear_stop_reason(0, hardwareAxis);
            recordCommunicationEvent(true, QStringLiteral("dmc_clear_stop_reason"));
            const short profileErr = dmc_set_profile_unit(0,
                                                          hardwareAxis,
                                                          0.0,
                                                          std::max(std::fabs(signedVelocity), 1e-5),
                                                          0.1,
                                                          0.1,
                                                          0);
            recordCommunicationEvent(true, QStringLiteral("dmc_set_profile_unit"));
            const short sErr = dmc_set_s_profile(0, hardwareAxis, 0, 0);
            recordCommunicationEvent(true, QStringLiteral("dmc_set_s_profile"));
            const short moveErr = dmc_vmove(0, hardwareAxis, signedVelocity >= 0.0 ? 1 : 0);
            recordCommunicationEvent(true, QStringLiteral("dmc_vmove"));
            if(clearStopReasonErr != 0 || profileErr != 0 || sErr != 0 || moveErr != 0){
                WORD stateMachine = 0;
                const short stateRet = nmc_get_axis_state_machine(0, hardwareAxis, &stateMachine);
                recordCommunicationEvent(false, QStringLiteral("nmc_get_axis_state_machine"));
                emit displayInfoSignal(QString("Batch JOG velocity start failed on %1 hardware axis %2, error clear=%3 profile=%4 s=%5 move=%6 state_ret=%7 state=%8.")
                                           .arg(axisDisplayName(command.logicalAxis))
                                           .arg(command.hardwareAxis)
                                           .arg(clearStopReasonErr)
                                           .arg(profileErr)
                                           .arg(sErr)
                                           .arg(moveErr)
                                           .arg(stateRet)
                                           .arg(stateMachine)
                                           .toStdString(),
                                       "error");
                ok = false;
            }
        }
        else{
            const short speedErr = dmc_change_speed_unit(0, hardwareAxis, signedVelocity, taccdec);
            recordCommunicationEvent(true, QStringLiteral("dmc_change_speed_unit"));
            if(speedErr != 0){
                emit displayInfoSignal(QString("Batch JOG online speed change failed on %1, error %2.")
                                           .arg(axisDisplayName(command.logicalAxis))
                                           .arg(speedErr)
                                           .toStdString(),
                                       "error");
                ok = false;
            }
        }
    }
    return ok;
    });
}

bool HardwareInterface::motorTorqueStart(int index, double torqueNm)
{
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor torque mode debugging is supported here.", "error");
        return false;
    }
    if(!isConnectLS){
        emit displayInfoSignal("错误：雷赛控制卡未连接，无法启动转矩模式", "error");
        return false;
    }
    if(!std::isfinite(torqueNm)){
        emit displayInfoSignal("错误：目标转矩无效，转矩模式不会启动", "error");
        return false;
    }

    const double limitedTorqueNm =
            activeRuntimeTraceConfigType == RuntimeTraceConfigType::G302 ?
                std::clamp(torqueNm,
                           -kLiteMotorTorqueCommandLimitNm,
                           kLiteMotorTorqueCommandLimitNm) :
                torqueNm;
    const int torqueRaw = leadshineTorqueNmToRaw(limitedTorqueNm,
                                                 leadshineRatedMotorTorqueNm);
    if(torqueRaw == 0){
        emit displayInfoSignal("Target torque is zero; torque mode will not start.", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(), "error");
        return false;
    }

    // Hybrid force axes can be any selected logical axes; do not clear unrelated PVT axes.
    const bool torqueAxisInActivePvt =
            std::find(activePvtMotorIndex.begin(), activePvtMotorIndex.end(), index) != activePvtMotorIndex.end();
    if(torqueAxisInActivePvt){
        hasActivePvtTrajectory = false;
        isPvtMotionPaused = false;
        pausedPvtResumeIndex = 0;
        pausedPvtResumeTime = 0.0;
        activePvtMotorIndex.clear();
        activePvtMotorPosTraj.clear();
        activePvtMotorVelTraj.clear();
        activePvtMotorVelMax.clear();
        activePvtTimeStamp.clear();
        activePvtStartMonotonicUs = 0;
    }

    const short torqueErr = nmc_torque_move(0,
                                            static_cast<WORD>(hardwareAxis),
                                            torqueRaw,
                                            kTorquePositionLimitDisabled,
                                            kTorquePositionLimitValueUnused,
                                            kTorqueAbsolutePositionMode);
    recordCommunicationEvent(true, QStringLiteral("nmc_torque_move"));
    if(torqueErr != 0){
        emit displayInfoSignal(QString("错误：%1转矩模式启动失败，错误码%2")
                                   .arg(axisDisplayName(index))
                                   .arg(torqueErr)
                                   .toStdString(), "error");
        return false;
    }
    return true;
    });
}

bool HardwareInterface::motorTorqueChange(int index, double torqueNm)
{
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor torque mode debugging is supported here.", "error");
        return false;
    }
    if(!isConnectLS){
        emit displayInfoSignal("错误：雷赛控制卡未连接，无法调整转矩", "error");
        return false;
    }
    if(!std::isfinite(torqueNm)){
        emit displayInfoSignal("错误：目标转矩无效，无法调整转矩", "error");
        return false;
    }

    const double limitedTorqueNm =
            activeRuntimeTraceConfigType == RuntimeTraceConfigType::G302 ?
                std::clamp(torqueNm,
                           -kLiteMotorTorqueCommandLimitNm,
                           kLiteMotorTorqueCommandLimitNm) :
                torqueNm;
    const int torqueRaw = leadshineTorqueNmToRaw(limitedTorqueNm,
                                                 leadshineRatedMotorTorqueNm);

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(), "error");
        return false;
    }

    const short torqueErr = nmc_change_torque(0, static_cast<WORD>(hardwareAxis), torqueRaw);
    recordCommunicationEvent(true, QStringLiteral("nmc_change_torque"));
    if(torqueErr != 0){
        emit displayInfoSignal(QString("错误：%1在线调整转矩失败，错误码%2")
                                   .arg(axisDisplayName(index))
                                   .arg(torqueErr)
                                   .toStdString(), "error");
        return false;
    }
    return true;
    });
}

bool HardwareInterface::motorVelWithTargetPosAndStopVel(int index, double vel, double targetPos, double stopVel) {
    return runOnHardwareThread([&]() -> bool {
    if (index < 0 || index >= static_cast<int>(motorComType.size())) {
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("Only Leadshine motor control is supported here.", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }

    double pos = 0.0;
    if(!readMotorPositionUnitDirect(index, pos)){
        emit displayInfoSignal(QString("Unit position read failed for %1.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(),
                               "error");
        return false;
    }
    bool crossed = (vel > 0 && pos > targetPos) || (vel < 0 && pos < targetPos);

    QString limitError;
    if(!validateAbsoluteMotorSoftwareLimit(index,
                                           targetPos,
                                           QStringLiteral("电机带目标速度运动指令"),
                                           &limitError)){
        emit displayInfoSignal(limitError.toStdString(), "error");
        return false;
    }
    if(!validateVelocityMotorSoftwareLimit(index,
                                           pos,
                                           crossed ? stopVel : vel,
                                           QStringLiteral("电机带目标速度运动指令"),
                                           &limitError)){
        emit displayInfoSignal(limitError.toStdString(), "error");
        return false;
    }

    if (dmc_check_done(0, static_cast<WORD>(hardwareAxis))) {
        recordCommunicationEvent(false, QStringLiteral("dmc_check_done"));
        if (crossed) {
            if (std::fabs(stopVel) < 1e-5) {
                return motorStop(index);
            }
            dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, std::max(std::fabs(stopVel), 1e-5), 0.1, 0.1, 0);
            recordCommunicationEvent(true, QStringLiteral("dmc_set_profile_unit"));
        } else {
            dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, std::max(std::fabs(vel), 1e-5), 0.1, 0.1, 0);
            recordCommunicationEvent(true, QStringLiteral("dmc_set_profile_unit"));
        }

        dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_set_s_profile"));
        dmc_vmove(0, static_cast<WORD>(hardwareAxis), vel >= 0 ? 1 : 0);
        recordCommunicationEvent(true, QStringLiteral("dmc_vmove"));
    } else {
        if (crossed) {
            if (std::fabs(stopVel) < 1e-5) {
                return motorStop(index);
            }
            dmc_change_speed_unit(0, static_cast<WORD>(hardwareAxis), stopVel, velChangeSpd);
            recordCommunicationEvent(true, QStringLiteral("dmc_change_speed_unit"));
        } else {
            dmc_change_speed_unit(0, static_cast<WORD>(hardwareAxis), vel, velChangeSpd);
            recordCommunicationEvent(true, QStringLiteral("dmc_change_speed_unit"));
        }
    }
    return true;
    });
}

double HardwareInterface::readMotorCurPos(int index) {
    return runOnHardwareThread([&]() -> double {
    double result = 0.0;
    if (index >= 0 && index < static_cast<int>(motorComType.size()) && motorComType[index] == COM_EC_LS) {
        readMotorPositionUnitDirect(index, result);
    }
    return result;
    });
}

bool HardwareInterface::readLeadshinePositionUnitTraceDiagnostic(int index,
                                                                 double& directUnitPosition,
                                                                 double& encoderUnitPosition,
                                                                 double& traceCommandUnitPosition,
                                                                 double& traceActualUnitPosition,
                                                                 double& axisEquiv,
                                                                 double& commandUnitDiff,
                                                                 double& commandPulseDiffEstimate,
                                                                 double& actualZeroOffsetUnit,
                                                                 double& actualZeroOffsetPulse,
                                                                 double& encoderVsPositionUnitDiff,
                                                                 double& encoderVsPositionPulseDiff,
                                                                 double& traceActualEncoderUnitDiff,
                                                                 double& traceActualEncoderPulseDiff,
                                                                 QString* errorMessage)
{
    return runOnHardwareThread([&]() -> bool {
    directUnitPosition = std::numeric_limits<double>::quiet_NaN();
    encoderUnitPosition = std::numeric_limits<double>::quiet_NaN();
    traceCommandUnitPosition = std::numeric_limits<double>::quiet_NaN();
    traceActualUnitPosition = std::numeric_limits<double>::quiet_NaN();
    axisEquiv = std::numeric_limits<double>::quiet_NaN();
    commandUnitDiff = std::numeric_limits<double>::quiet_NaN();
    commandPulseDiffEstimate = std::numeric_limits<double>::quiet_NaN();
    actualZeroOffsetUnit = std::numeric_limits<double>::quiet_NaN();
    actualZeroOffsetPulse = std::numeric_limits<double>::quiet_NaN();
    encoderVsPositionUnitDiff = std::numeric_limits<double>::quiet_NaN();
    encoderVsPositionPulseDiff = std::numeric_limits<double>::quiet_NaN();
    traceActualEncoderUnitDiff = std::numeric_limits<double>::quiet_NaN();
    traceActualEncoderPulseDiff = std::numeric_limits<double>::quiet_NaN();
    if(errorMessage){
        errorMessage->clear();
    }

    auto fail = [&](const QString& message) -> bool {
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };

    if(!isConnectLS){
        return fail(QStringLiteral("Leadshine card is not connected"));
    }
    if(index < 0 || index >= static_cast<int>(motorComType.size())){
        return fail(QStringLiteral("motor axis index is out of range"));
    }
    if(motorComType[index] != COM_EC_LS){
        return fail(QStringLiteral("axis is not a Leadshine motor axis"));
    }
    if(resolveLeadshineAxisIndex(index) < 0){
        return fail(QStringLiteral("axis has no valid controller axis mapping"));
    }

    axisEquiv = resolveLeadshineAxisEquiv(index);
    if(!std::isfinite(axisEquiv) || axisEquiv <= 0.0){
        return fail(QStringLiteral("axisEquiv is invalid"));
    }

    if(!readMotorPositionUnitDirect(index, directUnitPosition, false)){
        return fail(QStringLiteral("dmc_get_position_unit failed"));
    }
    if(!readMotorEncoderUnitDirect(index, encoderUnitPosition)){
        return fail(QStringLiteral("dmc_get_encoder_unit failed"));
    }

    bool traceOk = false;
    for(int retry = 0; retry < 3; ++retry){
        const int frameCount = readRuntimeTraceCached();
        if((frameCount >= 0 || runtimeTraceEverRead) &&
                index < static_cast<int>(motorCommandPos.size()) &&
                index < static_cast<int>(motorTraceActualPos.size()) &&
                std::isfinite(motorCommandPos[index]) &&
                std::isfinite(motorTraceActualPos[index])){
            traceCommandUnitPosition = motorCommandPos[index];
            traceActualUnitPosition = motorTraceActualPos[index];
            traceOk = true;
            break;
        }
        QThread::msleep(2);
    }
    if(!traceOk){
        return fail(QStringLiteral("Trace position read failed; Trace may not be configured or no valid frame has arrived"));
    }
    if(!std::isfinite(traceCommandUnitPosition) || !std::isfinite(traceActualUnitPosition)){
        return fail(QStringLiteral("Trace converted position is invalid"));
    }

    commandUnitDiff = traceCommandUnitPosition - directUnitPosition;
    commandPulseDiffEstimate = commandUnitDiff * axisEquiv;
    actualZeroOffsetUnit = traceActualUnitPosition - directUnitPosition;
    actualZeroOffsetPulse = actualZeroOffsetUnit * axisEquiv;
    encoderVsPositionUnitDiff = encoderUnitPosition - directUnitPosition;
    encoderVsPositionPulseDiff = encoderVsPositionUnitDiff * axisEquiv;
    traceActualEncoderUnitDiff = traceActualUnitPosition - encoderUnitPosition;
    traceActualEncoderPulseDiff = traceActualEncoderUnitDiff * axisEquiv;
    return std::isfinite(commandUnitDiff) &&
            std::isfinite(commandPulseDiffEstimate) &&
            std::isfinite(actualZeroOffsetUnit) &&
            std::isfinite(actualZeroOffsetPulse) &&
            std::isfinite(encoderVsPositionUnitDiff) &&
            std::isfinite(encoderVsPositionPulseDiff) &&
            std::isfinite(traceActualEncoderUnitDiff) &&
            std::isfinite(traceActualEncoderPulseDiff);
    });
}

bool HardwareInterface::readMotorRelativeCurPos(int index, double& relativePosition)
{
    double commandRelativePosition = 0.0;
    return readMotorRelativeTracePositions(index, commandRelativePosition, relativePosition);
}

bool HardwareInterface::readMotorSafetyRelativeCurPos(int index, double& relativePosition)
{
    return runOnHardwareThread([&]() -> bool {
    return readMotorSafetyRelativePositionDirect(index, relativePosition);
    });
}

bool HardwareInterface::readMotorCurrentSpeedUnit(int index, double& velocity)
{
    return runOnHardwareThread([&]() -> bool {
    velocity = std::numeric_limits<double>::quiet_NaN();
    if(!isConnectLS ||
            index < 0 ||
            index >= static_cast<int>(motorComType.size()) ||
            motorComType[index] != COM_EC_LS){
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if(hardwareAxis < 0){
        return false;
    }

    double currentVelocity = 0.0;
    const short ret = dmc_read_current_speed_unit(
                0,
                static_cast<WORD>(hardwareAxis),
                &currentVelocity);
    recordCommunicationEvent(false, QStringLiteral("dmc_read_current_speed_unit"));
    if(ret != 0 || !std::isfinite(currentVelocity)){
        return false;
    }

    if(motorCurVel.size() != motorIdVec.size()){
        motorCurVel.assign(motorIdVec.size(), 0.0);
    }
    if(index < static_cast<int>(motorCurVel.size())){
        motorCurVel[index] = currentVelocity;
    }
    velocity = currentVelocity;
    return true;
    });
}

bool HardwareInterface::readMotorTorqueNmTraceCached(int index, double& torqueNm)
{
    return runOnHardwareThread([&]() -> bool {
    torqueNm = std::numeric_limits<double>::quiet_NaN();
    if(!isConnectLS ||
            index < 0 ||
            index >= static_cast<int>(motorComType.size()) ||
            motorComType[index] != COM_EC_LS){
        return false;
    }

    // 安全相对位置读取已经在本轮刷新 Runtime Trace；这里只访问同一
    // Trace 帧留下的指定轴缓存，避免再次读取或遍历其他轴。
    const std::vector<double> cachedTorque = currentMotorTorqueTraceCachedValues();
    if(index >= static_cast<int>(cachedTorque.size()) ||
            !std::isfinite(cachedTorque[index])){
        return false;
    }
    torqueNm = cachedTorque[index];
    return true;
    });
}

bool HardwareInterface::readMotorRelativeTracePositions(int index,
                                                        double& commandRelativePosition,
                                                        double& feedbackRelativePosition)
{
    return runOnHardwareThread([&]() -> bool {
    commandRelativePosition = 0.0;
    feedbackRelativePosition = 0.0;
    if(index < 0 ||
            index >= static_cast<int>(motorComType.size()) ||
            motorComType[index] != COM_EC_LS ||
            resolveLeadshineAxisIndex(index) < 0){
        return false;
    }
    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    if(motorTraceActualPos.size() != motorIdVec.size()){
        motorTraceActualPos.assign(motorIdVec.size(), 0.0);
    }
    if(motorCommandPos.size() != motorIdVec.size()){
        motorCommandPos.assign(motorIdVec.size(), 0.0);
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return false;
    }
    if(index >= static_cast<int>(motorCurPos.size()) ||
            index >= static_cast<int>(motorCommandPos.size()) ||
            index >= static_cast<int>(motorTraceActualPos.size()) ||
            !std::isfinite(motorTraceActualPos[index]) ||
            !std::isfinite(motorCommandPos[index])){
        return false;
    }
    if(!ensureMotorTracePositionOffsets(index)){
        return false;
    }

    commandRelativePosition =
            traceAlignedRelativePosition(index,
                                         motorCommandPos[index],
                                         motorCommandTraceOffsetUnit,
                                         motorCommandTraceOffsetValid);
    feedbackRelativePosition =
            traceAlignedRelativePosition(index,
                                         motorTraceActualPos[index],
                                         motorActualTraceOffsetUnit,
                                         motorActualTraceOffsetValid);
    return std::isfinite(commandRelativePosition) && std::isfinite(feedbackRelativePosition);
    });
}

std::vector<HardwareInterface::MotorTracePositionSample>
HardwareInterface::readMotorRelativeTracePositionSamples(int index)
{
    return runOnHardwareThread([&]() -> std::vector<MotorTracePositionSample> {
    if(index < 0 ||
            index >= static_cast<int>(motorComType.size()) ||
            motorComType[index] != COM_EC_LS ||
            resolveLeadshineAxisIndex(index) < 0){
        return {};
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return {};
    }
    if(motorTracePositionSampleQueues.size() != motorIdVec.size()){
        motorTracePositionSampleQueues.assign(motorIdVec.size(),
                                              std::deque<MotorTracePositionSample>());
    }
    if(index >= static_cast<int>(motorTracePositionSampleQueues.size())){
        return {};
    }

    auto& queuedSamples = motorTracePositionSampleQueues[index];
    std::vector<MotorTracePositionSample> result;
    result.reserve(queuedSamples.size());
    while(!queuedSamples.empty()){
        result.push_back(queuedSamples.front());
        queuedSamples.pop_front();
    }
    return result;
    });
}

std::vector<std::vector<HardwareInterface::MotorTracePositionSample>>
HardwareInterface::readMotorRelativeTracePositionSamples(const std::vector<int>& motorIndex)
{
    return runOnHardwareThread([&]() -> std::vector<std::vector<MotorTracePositionSample>> {
    std::vector<std::vector<MotorTracePositionSample>> result(motorIndex.size());
    if(motorIndex.empty()){
        return result;
    }

    for(const int axis : motorIndex){
        if(axis < 0 ||
                axis >= static_cast<int>(motorComType.size()) ||
                motorComType[axis] != COM_EC_LS ||
                resolveLeadshineAxisIndex(axis) < 0){
            return {};
        }
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return {};
    }
    if(motorTracePositionSampleQueues.size() != motorIdVec.size()){
        motorTracePositionSampleQueues.assign(motorIdVec.size(),
                                              std::deque<MotorTracePositionSample>());
    }

    for(int index = 0; index < static_cast<int>(motorIndex.size()); ++index){
        const int axis = motorIndex[index];
        if(axis >= static_cast<int>(motorTracePositionSampleQueues.size())){
            return {};
        }

        auto& queuedSamples = motorTracePositionSampleQueues[axis];
        result[index].reserve(queuedSamples.size());
        while(!queuedSamples.empty()){
            result[index].push_back(queuedSamples.front());
            queuedSamples.pop_front();
        }
    }
    return result;
    });
}

bool HardwareInterface::readMotorTraceCommandRawPulseSnapshot(
        const std::vector<int>& motorIndex,
        std::vector<qint64>& rawPulse)
{
    return runOnHardwareThread([&]() -> bool {
    rawPulse.assign(motorIdVec.size(), 0);
    if(motorIndex.empty() || !isConnectLS){
        return false;
    }

    for(const int axis : motorIndex){
        if(axis < 0 ||
                axis >= static_cast<int>(motorComType.size()) ||
                axis >= static_cast<int>(motorIdVec.size()) ||
                motorComType[axis] != COM_EC_LS ||
                resolveLeadshineAxisIndex(axis) < 0){
            return false;
        }
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return false;
    }
    if(!latestMotorTracePositionFrameValid){
        return false;
    }

    const MotorTracePositionWindowFrame& frame = latestMotorTracePositionFrame;
    for(const int axis : motorIndex){
        if(axis >= static_cast<int>(frame.commandValid.size()) ||
                axis >= static_cast<int>(frame.commandRawPulse.size()) ||
                !frame.commandValid[axis]){
            return false;
        }
        rawPulse[axis] = frame.commandRawPulse[axis];
    }
    return true;
    });
}

bool HardwareInterface::readFreshMotorTraceCommandRawPulseSnapshot(
        const std::vector<int>& motorIndex,
        std::vector<qint64>& rawPulse,
        QString* failureReason,
        bool* fifoNeedsDrain)
{
    return runOnHardwareThread([&]() -> bool {
    rawPulse.clear();
    if(failureReason){
        failureReason->clear();
    }
    if(fifoNeedsDrain){
        *fifoNeedsDrain = false;
    }

    const auto fail = [&](const QString& reason) -> bool {
        if(failureReason){
            *failureReason = reason;
        }
        rawPulse.clear();
        return false;
    };

    if(motorIndex.empty()){
        return fail(QStringLiteral("未指定需要校验的逻辑轴"));
    }
    if(!isConnectLS){
        return fail(QStringLiteral("雷赛控制卡未连接"));
    }

    for(const int axis : motorIndex){
        if(axis < 0 ||
                axis >= static_cast<int>(motorComType.size()) ||
                axis >= static_cast<int>(motorIdVec.size()) ||
                motorComType[axis] != COM_EC_LS ||
                resolveLeadshineAxisIndex(axis) < 0){
            return fail(QStringLiteral("逻辑轴%1不是有效的雷赛轴").arg(axis + 1));
        }
    }

    const int frameCount = readRuntimeTraceCached(false);
    if(fifoNeedsDrain){
        // fifoCaughtUp 表示上次批量读取开始前已经存在的旧帧已全部消费。
        // API阻塞期间新产生的帧由newestFrameAgeUs体现，不需要在这里忙等追读。
        *fifoNeedsDrain = !runtimeTraceFifoCaughtUp;
    }
    const qint64 nowUs = monotonicNowUs();
    const qint64 futureToleranceUs = std::max<qint64>(
                2 * 1000,
                static_cast<qint64>(runtimeTraceSamplePeriodUs) *
                    kRuntimeTraceTimestampFutureToleranceFrames);
    const qint64 frameDeltaUs =
            latestMotorTracePositionFrame.monotonicUs > 0 ?
                nowUs - latestMotorTracePositionFrame.monotonicUs :
                -1;
    const bool frameTimestampFresh = latestMotorTracePositionFrameValid &&
            latestMotorTracePositionFrame.monotonicUs > 0 &&
            nowUs + futureToleranceUs >= latestMotorTracePositionFrame.monotonicUs &&
            (nowUs < latestMotorTracePositionFrame.monotonicUs ||
             frameDeltaUs <= kMotorPositionTraceFreshTimeoutUs);
    const bool freshFrame = runtimeTraceConfigReadbackValid &&
            runtimeTraceTimingReliable &&
            runtimeTraceFifoCaughtUp &&
            !runtimeTraceLost &&
            runtimeTraceNewestFrameAgeUs >= 0 &&
            runtimeTraceNewestFrameAgeUs <= kMotorPositionTraceFreshTimeoutUs &&
            frameTimestampFresh;
    if(!freshFrame){
        return fail(QStringLiteral(
            "frames=%1, configured=%2, unavailable=%3, readback=%4, "
            "timing_reliable=%5, fifo_caught_up=%6, trace_lost=%7, "
            "fifo_valid=%8, newest_age_us=%9, frame_valid=%10, frame_delta_us=%11")
                    .arg(frameCount)
                    .arg(runtimeTraceConfigured ? 1 : 0)
                    .arg(runtimeTraceUnavailable ? 1 : 0)
                    .arg(runtimeTraceConfigReadbackValid ? 1 : 0)
                    .arg(runtimeTraceTimingReliable ? 1 : 0)
                    .arg(runtimeTraceFifoCaughtUp ? 1 : 0)
                    .arg(runtimeTraceLost ? 1 : 0)
                    .arg(runtimeTraceLastFifoValidNum)
                    .arg(runtimeTraceNewestFrameAgeUs)
                    .arg(latestMotorTracePositionFrameValid ? 1 : 0)
                    .arg(frameDeltaUs));
    }

    const MotorTracePositionWindowFrame& frame = latestMotorTracePositionFrame;
    QStringList invalidAxes;
    rawPulse.reserve(motorIndex.size());
    for(const int axis : motorIndex){
        const bool commandValid =
                axis < static_cast<int>(frame.commandValid.size()) &&
                axis < static_cast<int>(frame.commandRawPulse.size()) &&
                frame.commandValid[axis];
        if(!commandValid || frame.commandRawPulse[axis] == 0){
            invalidAxes.push_back(axisDisplayName(axis));
            continue;
        }
        rawPulse.push_back(frame.commandRawPulse[axis]);
    }
    if(!invalidAxes.empty() || rawPulse.size() != motorIndex.size()){
        return fail(QStringLiteral("新鲜Trace帧缺少有效非零command_raw：%1")
                    .arg(invalidAxes.join(QStringLiteral(", "))));
    }
    return true;
    });
}

bool HardwareInterface::restartRuntimeTraceForFreshSnapshot(QString* failureReason)
{
    return runOnHardwareThread([&]() -> bool {
    if(failureReason){
        failureReason->clear();
    }
    if(!isConnectLS){
        if(failureReason){
            *failureReason = QStringLiteral("雷赛控制卡未连接");
        }
        return false;
    }

    // 启动基准只需要重启后的当前硬件快照。此时整机尚未进入运行态，
    // 可以安全丢弃连接诊断等先行读取留下的旧帧，重新从FIFO头部开始。
    resetRuntimeTraceState();
    if(configureRuntimeTraceRead()){
        return true;
    }
    if(failureReason){
        *failureReason = QStringLiteral(
                    "Runtime Trace停止、清空或按当前配置重新启动失败");
    }
    return false;
    });
}

bool HardwareInterface::readMotorCommandPosition(int index, double& commandPosition)
{
    return runOnHardwareThread([&]() -> bool {
    double actualPosition = 0.0;
    return readMotorRelativeTracePositions(index, commandPosition, actualPosition);
    });
}

bool HardwareInterface::configureMotorPositionTrace(int index)
{
    return runOnHardwareThread([&]() -> bool {
    Q_UNUSED(index);
    return configureMotorPositionTraceRead();
    });
}

bool HardwareInterface::readMotorTracePositions(int index, double& commandPosition, double& actualPosition)
{
    return runOnHardwareThread([&]() -> bool {
    return readMotorRelativeTracePositions(index, commandPosition, actualPosition);
    });
}

void HardwareInterface::stopMotorPositionTrace()
{
    return runOnHardwareThread([&]() {
    resetMotorPositionTraceState();
    });
}

bool HardwareInterface::readLeadshineModeOfOperation(int index, qint8& modeOfOperation)
{
    return runOnHardwareThread([&]() -> bool {
    modeOfOperation = 0;
    if (!isConnectLS ||
            index < 0 ||
            index >= static_cast<int>(motorComType.size()) ||
            motorComType[index] != COM_EC_LS) {
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        return false;
    }

    WORD slaveAddress = 0;
    if (index < static_cast<int>(motorSlaveIdVec.size()) && motorSlaveIdVec[index] > 0) {
        slaveAddress = static_cast<WORD>(motorSlaveIdVec[index]);
    } else {
        WORD subSlaveAddress = 0;
        const short addrRet = nmc_get_axis_node_address(0,
                                                        static_cast<WORD>(hardwareAxis),
                                                        &slaveAddress,
                                                        &subSlaveAddress);
        recordCommunicationEvent(false, QStringLiteral("nmc_get_axis_node_address"));
        if (addrRet != 0 || slaveAddress == 0) {
            return false;
        }
    }

    long value = 0;
    const short ret = nmc_get_node_od(0,
                                      kLeadshineEtherCatPort,
                                      slaveAddress,
                                      kLeadshineModeOfOperationIndex,
                                      kLeadshineModeOfOperationSubIndex,
                                      kLeadshineModeOfOperationBitLength,
                                      &value);
    recordCommunicationEvent(false, QStringLiteral("nmc_get_node_od"));
    if (ret != 0) {
        return false;
    }

    modeOfOperation = static_cast<qint8>(value & 0xFF);
    return true;
    });
}

bool HardwareInterface::readLeadshineFollowingErrorRaw(int index, int& followingErrorRaw)
{
    return runOnHardwareThread([&]() -> bool {
    followingErrorRaw = 0;
    if (!isConnectLS ||
            index < 0 ||
            index >= static_cast<int>(motorComType.size()) ||
            motorComType[index] != COM_EC_LS ||
            index >= static_cast<int>(motorSlaveIdVec.size())) {
        return false;
    }

    const int slaveId = motorSlaveIdVec[index];
    if (slaveId <= 0) {
        return false;
    }

    long value = 0;
    const short ret = nmc_get_node_od(0,
                                      kLeadshineEtherCatPort,
                                      static_cast<WORD>(slaveId),
                                      kLeadshineFollowingErrorActualIndex,
                                      kLeadshineFollowingErrorActualSubIndex,
                                      kLeadshineFollowingErrorActualBitLength,
                                      &value);
    recordCommunicationEvent(false, QStringLiteral("nmc_get_node_od"));
    if (ret != 0) {
        return false;
    }

    followingErrorRaw = static_cast<int>(static_cast<qint32>(value));
    return true;
    });
}

void HardwareInterface::canPortInfoProcessor(const QString, int &deviceIndex, int &canPortIndex) {
    deviceIndex = 0;
    canPortIndex = 0;
}

void HardwareInterface::canPortInfoProcessor(const std::vector<QString>, std::vector<int> &deviceIndex, std::vector<int> &canPortIndex) {
    deviceIndex.clear();
    canPortIndex.clear();
}

void HardwareInterface::resetRuntimeTraceState()
{
    if(isConnectLS && runtimeTraceConfigured){
        dmc_trace_data_stop(0);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_stop"));
    }
    runtimeTraceConfigured = false;
    runtimeTraceUnavailable = false;
    runtimeTraceEverRead = false;
    runtimeTraceConfigReadbackValid = false;
    runtimeTraceTimingReliable = false;
    runtimeTraceFifoCaughtUp = false;
    runtimeTraceLost = false;
    runtimeTraceConsecutiveFailures = 0;
    runtimeTraceObjectTotalBytes = 0;
    runtimeTraceObjectTotalNum = 0;
    runtimeTraceEthercatBusCycleUs = kDefaultEthercatBusCycleUs;
    runtimeTraceConfiguredCycle = 1;
    runtimeTraceSamplePeriodUs = kForceSensorTraceBaseCycleUs;
    runtimeTraceLastFifoValidNum = 0;
    runtimeTraceLastFifoFreeNum = 0;
    runtimeTraceLastRetryUs = 0;
    runtimeTraceLastFrameWallClockUs = 0;
    runtimeTraceLastFrameMonotonicUs = 0;
    runtimeTraceNewestFrameAgeUs = -1;
    runtimeTraceLastFrameSequenceValid = false;
    runtimeTraceLastFrameSequence = 0;
    runtimeTraceSequenceInitialized = false;
    runtimeTraceLastRawSequence = 0;
    runtimeTraceLastLogicalSequence = 0;
    runtimeTraceHostTimeAnchorValid = false;
    runtimeTraceHostTimeAnchorSequence = 0;
    runtimeTraceHostTimeAnchorWallClockUs = 0;
    runtimeTraceHostTimeAnchorMonotonicUs = 0;
    runtimeTracePositionRejectLastWarnUs = 0;
    runtimeTraceLayoutRejectLastWarnUs = 0;
    latestMotorTracePositionFrame = MotorTracePositionWindowFrame{};
    latestMotorTracePositionFrameValid = false;
    forceSensorDiagnosticsLastTraceReadUs = 0;
    resetEndpointRemoteRuntimeTraceStatusFault();
    pvtTraceStartDelayState = PvtTraceStartDelayState{};
    runtimeTraceObjects.clear();
    motorCommandPositionTraceObjects.clear();
    motorPositionTraceObjects.clear();
    motorCommandVelocityTraceObjects.clear();
    motorActualVelocityTraceObjects.clear();
    motorStatusWordTraceObjects.clear();
    motorTorqueTraceObjects.clear();
    motorTraceCommandVelocity.assign(motorIdVec.size(), 0.0);
    motorTraceActualVelocity.assign(motorIdVec.size(), 0.0);
    motorTraceCommandVelocityValid.assign(motorIdVec.size(), false);
    motorTraceActualVelocityValid.assign(motorIdVec.size(), false);
    motorTraceStatusWord.assign(motorIdVec.size(), 0);
    motorTraceStatusWordValid.assign(motorIdVec.size(), false);
    motorTraceStatusWordMonotonicUs.assign(motorIdVec.size(), 0);
    motorTraceTorqueNm.assign(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
    motorTraceTorqueValid.assign(motorIdVec.size(), false);
    motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);
    forceSensorTraceObjects.clear();
    forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
    resetMotorTracePositionOffsets();
    forceSensorTraceSampleQueue.clear();
}

void HardwareInterface::resetForceSensorTraceState()
{
    resetRuntimeTraceState();
}

void HardwareInterface::resetMotorPositionTraceState()
{
    resetRuntimeTraceState();
}

void HardwareInterface::applyMotorCommandPositionTraceRawValue(int logicalIndex, long rawValue)
{
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorComType.size())){
        return;
    }
    if(motorCommandPos.size() != motorIdVec.size()){
        motorCommandPos.assign(motorIdVec.size(), 0.0);
    }
    if(logicalIndex >= static_cast<int>(motorCommandPos.size())){
        return;
    }

    motorCommandPos[logicalIndex] = tracePulseToMotorUnit(logicalIndex, rawValue);
}

void HardwareInterface::applyMotorPositionTraceRawValue(int logicalIndex, long rawValue)
{
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorComType.size())){
        return;
    }
    if(motorTraceActualPos.size() != motorIdVec.size()){
        motorTraceActualPos.assign(motorIdVec.size(), 0.0);
    }
    if(logicalIndex >= static_cast<int>(motorTraceActualPos.size())){
        return;
    }

    motorTraceActualPos[logicalIndex] = tracePulseToMotorUnit(logicalIndex, rawValue);
}

short HardwareInterface::resolveLeadshineTraceSlaveId(int logicalIndex, int hardwareAxis) const
{
    if(logicalIndex >= 0 &&
            logicalIndex < static_cast<int>(motorSlaveIdVec.size()) &&
            motorSlaveIdVec[logicalIndex] > 0){
        return static_cast<short>(motorSlaveIdVec[logicalIndex]);
    }
    if(hardwareAxis >= 0){
        return static_cast<short>(1001 + hardwareAxis);
    }
    return 0;
}

void HardwareInterface::applyMotorTorqueTraceRawValue(int logicalIndex,
                                                      long rawValue,
                                                      qint64 traceMonotonicUs)
{
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorComType.size())){
        return;
    }
    if(motorTraceTorqueNm.size() != motorIdVec.size()){
        motorTraceTorqueNm.assign(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
        motorTraceTorqueValid.assign(motorIdVec.size(), false);
        motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);
    }
    if(motorTraceTorqueValid.size() != motorIdVec.size()){
        motorTraceTorqueValid.assign(motorIdVec.size(), false);
    }
    if(motorTraceTorqueMonotonicUs.size() != motorIdVec.size()){
        motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);
    }
    if(logicalIndex >= static_cast<int>(motorTraceTorqueNm.size())){
        return;
    }

    motorTraceTorqueNm[logicalIndex] = leadshineTorqueRawToNm(rawValue, leadshineRatedMotorTorqueNm);
    motorTraceTorqueValid[logicalIndex] = true;
    motorTraceTorqueMonotonicUs[logicalIndex] = traceMonotonicUs;
}

std::vector<double> HardwareInterface::currentMotorTorqueTraceCachedValues() const
{
    std::vector<double> torqueNm(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
    const qint64 nowUs = monotonicNowUs();
    const int axisCount = std::min({static_cast<int>(motorIdVec.size()),
                                    static_cast<int>(motorTraceTorqueNm.size()),
                                    static_cast<int>(motorTraceTorqueValid.size()),
                                    static_cast<int>(motorTraceTorqueMonotonicUs.size())});
    for(int axis = 0; axis < axisCount; ++axis){
        const qint64 sampleUs = motorTraceTorqueMonotonicUs[axis];
        const bool fresh =
                sampleUs > 0 &&
                (nowUs < sampleUs ||
                 nowUs - sampleUs <= kMotorTorqueTraceFreshTimeoutUs);
        if(motorTraceTorqueValid[axis] && fresh){
            torqueNm[axis] = motorTraceTorqueNm[axis];
        }
    }
    return torqueNm;
}

std::vector<double> HardwareInterface::readMotorTorqueTraceCachedDirect()
{
    if(!isConnectLS){
        return std::vector<double>(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return std::vector<double>(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
    }
    return currentMotorTorqueTraceCachedValues();
}

void HardwareInterface::beginMotorTracePositionWindowRecording()
{
    motorTracePositionWindow.clear();
    motorTracePositionWindowRecordingEnabled = true;
    motorTracePositionWindowFrozenAfterPvt = false;
}

void HardwareInterface::freezeMotorTracePositionWindowRecording()
{
    return runOnHardwareThread([&]() {
    const bool hasPvtWindowToFreeze =
            motorTracePositionWindowRecordingEnabled || hasActivePvtTrajectory;
    motorTracePositionWindowRecordingEnabled = false;
    if(hasPvtWindowToFreeze){
        motorTracePositionWindowFrozenAfterPvt = true;
    }
    });
}

bool HardwareInterface::activePvtAxesDoneDirect() const
{
    if(!hasActivePvtTrajectory || isPvtMotionPaused || activePvtMotorIndex.empty()){
        return false;
    }
    for(const int axis : activePvtMotorIndex){
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if(hardwareAxis < 0){
            continue;
        }
        if(dmc_check_done(0, static_cast<WORD>(hardwareAxis)) == 0){
            return false;
        }
    }
    return true;
}

void HardwareInterface::appendMotorTracePositionWindowFrame(const MotorTracePositionWindowFrame& frame)
{
    const std::size_t axisCount = frame.commandValid.size();
    bool hasAnyValidAxis = false;
    for(std::size_t axis = 0; axis < axisCount; ++axis){
        const bool commandValid =
                axis < frame.commandValid.size() && frame.commandValid[axis];
        const bool feedbackValid =
                axis < frame.feedbackValid.size() && frame.feedbackValid[axis];
        if(commandValid || feedbackValid){
            hasAnyValidAxis = true;
            break;
        }
    }
    if(!hasAnyValidAxis || frame.monotonicUs <= 0){
        return;
    }

    motorTracePositionWindow.push_back(frame);
    trimMotorTracePositionWindow(frame.monotonicUs);
}

void HardwareInterface::exportMotorTracePositionWindowForEvent(const QString& reason)
{
    if(!motorTracePositionWindowFrozenAfterPvt){
        constexpr int kTraceWindowExportReadAttempts = 8;
        constexpr int kTraceWindowExportRetryDelayMs = 5;
        for(int attempt = 0;
            attempt < kTraceWindowExportReadAttempts && motorTracePositionWindow.empty();
            ++attempt){
            readRuntimeTraceCached();
            if(motorTracePositionWindow.empty()){
                delay(kTraceWindowExportRetryDelayMs);
            }
        }
        if(!motorTracePositionWindow.empty()){
            readRuntimeTraceCached();
        }
    }

    QString traceWindowPath;
    QString traceWindowError;
    if(exportMotorTracePositionWindow(reason,
                                      &traceWindowPath,
                                      &traceWindowError)){
        emit displayInfoSignal(QString("Trace位置恢复窗口已保存至 %1")
                                   .arg(traceWindowPath)
                                   .toStdString(),
                               "info");
    }
    else if(!traceWindowError.isEmpty()){
        emit displayInfoSignal(QString("Trace位置恢复窗口未保存：%1")
                                   .arg(traceWindowError)
                                   .toStdString(),
                               "warning");
    }
}

void HardwareInterface::trimMotorTracePositionWindow(qint64 latestMonotonicUs)
{
    if(latestMonotonicUs <= 0){
        return;
    }

    const qint64 cutoffUs = latestMonotonicUs - kMotorTracePositionWindowRetentionUs;
    while(!motorTracePositionWindow.empty() &&
          motorTracePositionWindow.front().monotonicUs < cutoffUs){
        motorTracePositionWindow.pop_front();
    }
}

void HardwareInterface::setMotorTracePositionWindowFilePath(const QString& filePath)
{
    return runOnHardwareThread([&]() {
    motorTracePositionWindowOutputPath = filePath;
    });
}

QString HardwareInterface::motorTracePositionWindowFilePath() const
{
    return runOnHardwareThread([&]() -> QString {
    if(!motorTracePositionWindowOutputPath.isEmpty()){
        return motorTracePositionWindowOutputPath;
    }
    return QDir(RuntimePathUtils::dataPath(QStringLiteral("outputmsg")))
            .filePath(QStringLiteral("motor_trace_position_window_latest.tsv"));
    });
}

bool HardwareInterface::exportMotorTracePositionWindow(const QString& reason,
                                                       QString* outputPath,
                                                       QString* errorMessage)
{
    return runOnHardwareThread([&]() -> bool {
    if(errorMessage){
        errorMessage->clear();
    }

    if(motorTracePositionWindow.empty()){
        if(errorMessage){
            *errorMessage = QStringLiteral("Trace position window is empty.");
        }
        return false;
    }

    const QString filePath = motorTracePositionWindowFilePath();
    if(outputPath){
        *outputPath = filePath;
    }
    const QFileInfo fileInfo(filePath);
    const QString dirPath = fileInfo.dir().absolutePath();
    if(!QDir().mkpath(dirPath)){
        if(errorMessage){
            *errorMessage = QStringLiteral("Cannot create output directory %1.").arg(dirPath);
        }
        return false;
    }

    QFile file(filePath);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)){
        if(errorMessage){
            *errorMessage = QStringLiteral("Cannot write trace position window file %1.").arg(filePath);
        }
        return false;
    }

    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif
    auto number = [](double value, int precision = 9) -> QString {
        return std::isfinite(value) ? QString::number(value, 'f', precision) : QString();
    };
    auto boolText = [](bool value) -> QString {
        return value ? QStringLiteral("1") : QStringLiteral("0");
    };

    stream << "# motor trace position sliding window\n";
    stream << "# generated_at\t" << QDateTime::currentDateTime().toString(Qt::ISODateWithMs) << "\n";
    stream << "# reason\t" << reason << "\n";
    stream << "# retention_s\t0.100\n";
    stream << "# frame_count\t" << static_cast<qulonglong>(motorTracePositionWindow.size()) << "\n";
    stream << "# axis_count\t" << static_cast<int>(motorIdVec.size()) << "\n";
    stream << "# note\tcommand_raw and feedback_raw are the raw trace pulse values. command_delta_unit for recovery is raw_delta / axis_equiv.\n";
    stream << "sample_index\twall_clock_us\tmonotonic_us\tlogical_axis\thardware_axis\taxis_equiv"
           << "\tcommand_unit\tfeedback_unit\tcommand_relative_unit\tfeedback_relative_unit"
           << "\tcommand_raw\tfeedback_raw\tcommand_valid\tfeedback_valid\n";

    int sampleIndex = 0;
    for(const MotorTracePositionWindowFrame& frame : motorTracePositionWindow){
        const int axisCount = static_cast<int>(
                    std::max<std::size_t>(frame.commandValid.size(),
                                          frame.feedbackValid.size()));
        for(int axis = 0; axis < axisCount; ++axis){
            const bool commandValid =
                    axis < static_cast<int>(frame.commandValid.size()) &&
                    frame.commandValid[axis];
            const bool feedbackValid =
                    axis < static_cast<int>(frame.feedbackValid.size()) &&
                    frame.feedbackValid[axis];
            if(!commandValid && !feedbackValid){
                continue;
            }

            stream << sampleIndex
                   << "\t" << frame.wallClockUs
                   << "\t" << frame.monotonicUs
                   << "\t" << axis
                   << "\t" << resolveLeadshineAxisIndex(axis)
                   << "\t" << number(resolveLeadshineAxisEquiv(axis), 6)
                   << "\t" << number(axis < static_cast<int>(frame.commandUnitPosition.size()) ?
                                        frame.commandUnitPosition[axis] :
                                        std::numeric_limits<double>::quiet_NaN())
                   << "\t" << number(axis < static_cast<int>(frame.feedbackUnitPosition.size()) ?
                                        frame.feedbackUnitPosition[axis] :
                                        std::numeric_limits<double>::quiet_NaN())
                   << "\t" << number(axis < static_cast<int>(frame.commandRelativePosition.size()) ?
                                        frame.commandRelativePosition[axis] :
                                        std::numeric_limits<double>::quiet_NaN())
                   << "\t" << number(axis < static_cast<int>(frame.feedbackRelativePosition.size()) ?
                                        frame.feedbackRelativePosition[axis] :
                                        std::numeric_limits<double>::quiet_NaN())
                   << "\t" << (commandValid &&
                               axis < static_cast<int>(frame.commandRawPulse.size()) ?
                                   QString::number(frame.commandRawPulse[axis]) :
                                   QString())
                   << "\t" << (feedbackValid &&
                               axis < static_cast<int>(frame.feedbackRawPulse.size()) ?
                                   QString::number(frame.feedbackRawPulse[axis]) :
                                   QString())
                   << "\t" << boolText(commandValid)
                   << "\t" << boolText(feedbackValid)
                   << "\n";
        }
        ++sampleIndex;
    }

    file.close();
    if(file.error() != QFileDevice::NoError){
        if(errorMessage){
            *errorMessage = QStringLiteral("Trace position window file write failed %1.")
                    .arg(filePath);
        }
        return false;
    }

    return true;
    });
}

bool HardwareInterface::loadMotorTraceRecoveryFrameFromFile(
        const QString& filePath,
        MotorTracePositionWindowFrame& frame,
        QString* errorMessage) const
{
    if(errorMessage){
        errorMessage->clear();
    }
    frame = MotorTracePositionWindowFrame();

    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        if(errorMessage){
            *errorMessage = QStringLiteral("Cannot open trace position window file %1.").arg(filePath);
        }
        return false;
    }

    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif

    auto ensureAxisStorage = [](MotorTracePositionWindowFrame& target, int axis) {
        const int requiredSize = axis + 1;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        if(static_cast<int>(target.commandUnitPosition.size()) < requiredSize){
            target.commandUnitPosition.resize(requiredSize, nan);
            target.feedbackUnitPosition.resize(requiredSize, nan);
            target.commandRelativePosition.resize(requiredSize, nan);
            target.feedbackRelativePosition.resize(requiredSize, nan);
            target.commandRawPulse.resize(requiredSize, 0);
            target.feedbackRawPulse.resize(requiredSize, 0);
            target.commandValid.resize(requiredSize, false);
            target.feedbackValid.resize(requiredSize, false);
        }
    };

    int latestSampleIndex = -1;
    bool hasAnyRow = false;
    while(!stream.atEnd()){
        const QString line = stream.readLine();
        if(line.isEmpty() ||
                line.startsWith(QLatin1Char('#')) ||
                line.startsWith(QStringLiteral("sample_index"))){
            continue;
        }

        const QStringList fields = line.split(QLatin1Char('\t'));
        if(fields.size() < 14){
            continue;
        }

        bool ok = false;
        const int sampleIndex = fields[0].toInt(&ok);
        if(!ok){
            continue;
        }
        const int logicalAxis = fields[3].toInt(&ok);
        if(!ok || logicalAxis < 0){
            continue;
        }

        if(sampleIndex > latestSampleIndex){
            frame = MotorTracePositionWindowFrame();
            latestSampleIndex = sampleIndex;
            hasAnyRow = false;
        }
        if(sampleIndex != latestSampleIndex){
            continue;
        }

        ensureAxisStorage(frame, logicalAxis);
        frame.wallClockUs = fields[1].toLongLong();
        frame.monotonicUs = fields[2].toLongLong();
        frame.commandUnitPosition[logicalAxis] = fields[6].toDouble();
        frame.feedbackUnitPosition[logicalAxis] = fields[7].toDouble();
        frame.commandRelativePosition[logicalAxis] = fields[8].toDouble();
        frame.feedbackRelativePosition[logicalAxis] = fields[9].toDouble();
        const bool commandValid = fields[12].toInt() != 0;
        const bool feedbackValid = fields[13].toInt() != 0;
        frame.commandValid[logicalAxis] = commandValid;
        frame.feedbackValid[logicalAxis] = feedbackValid;
        if(commandValid){
            frame.commandRawPulse[logicalAxis] = fields[10].toLongLong();
        }
        if(feedbackValid){
            frame.feedbackRawPulse[logicalAxis] = fields[11].toLongLong();
        }
        hasAnyRow = true;
    }

    if(!hasAnyRow){
        if(errorMessage){
            *errorMessage = QStringLiteral("Trace position window file has no sample rows.");
        }
        return false;
    }
    return true;
}

bool HardwareInterface::readCurrentMotorTraceRecoveryFrame(
        const std::vector<int>& logicalAxes,
        const MotorTracePositionWindowFrame& savedFrame,
        bool requireSavedFeedbackChannels,
        MotorTracePositionWindowFrame& frame,
        QString* errorMessage)
{
    if(errorMessage){
        errorMessage->clear();
    }
    frame = MotorTracePositionWindowFrame();
    if(logicalAxes.empty()){
        if(errorMessage){
            *errorMessage = QStringLiteral("No logical axes require trace recovery comparison.");
        }
        return false;
    }
    if(!isConnectLS){
        if(errorMessage){
            *errorMessage = QStringLiteral("Leadshine controller is not connected.");
        }
        return false;
    }

    auto validAt = [](const std::vector<bool>& valid, int axis) -> bool {
        return axis >= 0 &&
                axis < static_cast<int>(valid.size()) &&
                valid[axis];
    };

    auto frameHasAxes = [&](const MotorTracePositionWindowFrame& candidate,
                            QStringList* missingDetails = nullptr) -> bool {
        bool hasAllRequiredChannels = true;
        for(const int axis : logicalAxes){
            const bool needCommand = validAt(savedFrame.commandValid, axis);
            const bool needFeedback =
                    requireSavedFeedbackChannels &&
                    validAt(savedFrame.feedbackValid, axis);
            if(axis < 0 || (!needCommand && !needFeedback)){
                continue;
            }

            QStringList missingChannels;
            if(needCommand && !validAt(candidate.commandValid, axis)){
                hasAllRequiredChannels = false;
                missingChannels << QStringLiteral("command");
            }
            if(needFeedback && !validAt(candidate.feedbackValid, axis)){
                hasAllRequiredChannels = false;
                missingChannels << QStringLiteral("feedback");
            }
            if(missingDetails && !missingChannels.isEmpty()){
                missingDetails->append(QStringLiteral("%1(%2)")
                                       .arg(axisDisplayName(axis),
                                            missingChannels.join(QLatin1Char('+'))));
            }
        }
        return hasAllRequiredChannels;
    };

    constexpr int kTraceRecoveryReadAttempts = 25;
    constexpr int kTraceRecoveryReadDelayMs = 10;
    const qint64 readStartUs = monotonicNowUs();
    QStringList latestMissingDetails;
    for(int attempt = 0; attempt < kTraceRecoveryReadAttempts; ++attempt){
        readRuntimeTraceCached();
        if(latestMotorTracePositionFrameValid){
            const MotorTracePositionWindowFrame& candidate = latestMotorTracePositionFrame;
            if(candidate.monotonicUs >= readStartUs - kMotorTracePositionWindowRetentionUs &&
                    frameHasAxes(candidate)){
                frame = candidate;
                return true;
            }
            latestMissingDetails.clear();
            frameHasAxes(candidate, &latestMissingDetails);
        }
        delay(kTraceRecoveryReadDelayMs);
    }

    if(errorMessage){
        *errorMessage = latestMissingDetails.isEmpty() ?
                    QStringLiteral("Cannot read current trace command/feedback positions for recovery comparison.") :
                    QStringLiteral("Cannot read current trace channels for recovery comparison: %1.")
                    .arg(latestMissingDetails.join(QStringLiteral(", ")));
    }
    return false;
}

HardwareInterface::MotorTraceRecoveryState
HardwareInterface::refreshMotorTraceRecoveryState(const QString& filePath,
                                                  std::vector<int> logicalAxes,
                                                  bool requireSavedFeedbackChannels)
{
    return runOnHardwareThread([&]() -> MotorTraceRecoveryState {
    MotorTraceRecoveryState state;
    state.filePath = filePath.isEmpty() ? motorTracePositionWindowFilePath() : filePath;

    QString errorMessage;
    MotorTracePositionWindowFrame savedFrame;
    if(!loadMotorTraceRecoveryFrameFromFile(state.filePath, savedFrame, &errorMessage)){
        state.message = errorMessage;
        cachedMotorTraceRecoveryStateData = state;
        return state;
    }
    state.fileLoaded = true;
    state.savedWallClockUs = savedFrame.wallClockUs;
    state.savedMonotonicUs = savedFrame.monotonicUs;

    std::sort(logicalAxes.begin(), logicalAxes.end());
    logicalAxes.erase(std::unique(logicalAxes.begin(), logicalAxes.end()),
                      logicalAxes.end());
    std::vector<int> axesToCompare;
    const int savedAxisCount = static_cast<int>(
                std::max<std::size_t>(savedFrame.commandValid.size(),
                                      savedFrame.feedbackValid.size()));
    for(int axis = 0; axis < savedAxisCount; ++axis){
        if(!logicalAxes.empty() &&
                !std::binary_search(logicalAxes.begin(), logicalAxes.end(), axis)){
            continue;
        }
        const bool savedCommandValid =
                axis < static_cast<int>(savedFrame.commandValid.size()) &&
                savedFrame.commandValid[axis];
        const bool savedFeedbackValid =
                axis < static_cast<int>(savedFrame.feedbackValid.size()) &&
                savedFrame.feedbackValid[axis];
        const bool hasRequiredSavedTrace =
                savedCommandValid ||
                (requireSavedFeedbackChannels && savedFeedbackValid);
        if(!hasRequiredSavedTrace){
            continue;
        }
        if(axis >= static_cast<int>(motorComType.size()) ||
                motorComType[axis] != COM_EC_LS ||
                resolveLeadshineAxisIndex(axis) < 0){
            continue;
        }
        axesToCompare.push_back(axis);
    }

    if(axesToCompare.empty()){
        state.message = QStringLiteral("Trace recovery file contains no configured Leadshine axes.");
        cachedMotorTraceRecoveryStateData = state;
        return state;
    }

    MotorTracePositionWindowFrame currentFrame;
    if(!readCurrentMotorTraceRecoveryFrame(axesToCompare,
                                           savedFrame,
                                           requireSavedFeedbackChannels,
                                           currentFrame,
                                           &errorMessage)){
        state.message = errorMessage;
        cachedMotorTraceRecoveryStateData = state;
        return state;
    }
    state.currentTraceRead = true;
    state.currentWallClockUs = currentFrame.wallClockUs;
    state.currentMonotonicUs = currentFrame.monotonicUs;

    for(const int axis : axesToCompare){
        MotorTraceRecoveryAxisState axisState;
        axisState.logicalAxis = axis;
        axisState.hardwareAxis = resolveLeadshineAxisIndex(axis);
        axisState.axisEquiv = resolveLeadshineAxisEquiv(axis);
        axisState.savedCommandValid =
                axis < static_cast<int>(savedFrame.commandValid.size()) &&
                savedFrame.commandValid[axis];
        axisState.savedFeedbackValid =
                axis < static_cast<int>(savedFrame.feedbackValid.size()) &&
                savedFrame.feedbackValid[axis];
        axisState.currentCommandValid =
                axis < static_cast<int>(currentFrame.commandValid.size()) &&
                currentFrame.commandValid[axis];
        axisState.currentFeedbackValid =
                axis < static_cast<int>(currentFrame.feedbackValid.size()) &&
                currentFrame.feedbackValid[axis];

        if(axisState.savedCommandValid && axis < static_cast<int>(savedFrame.commandRawPulse.size())){
            axisState.savedCommandRawPulse = savedFrame.commandRawPulse[axis];
        }
        if(axisState.savedFeedbackValid && axis < static_cast<int>(savedFrame.feedbackRawPulse.size())){
            axisState.savedFeedbackRawPulse = savedFrame.feedbackRawPulse[axis];
        }
        if(axisState.currentCommandValid && axis < static_cast<int>(currentFrame.commandRawPulse.size())){
            axisState.currentCommandRawPulse = currentFrame.commandRawPulse[axis];
        }
        if(axisState.currentFeedbackValid && axis < static_cast<int>(currentFrame.feedbackRawPulse.size())){
            axisState.currentFeedbackRawPulse = currentFrame.feedbackRawPulse[axis];
        }
        if(axis < static_cast<int>(savedFrame.commandUnitPosition.size())){
            axisState.savedCommandUnitPosition = savedFrame.commandUnitPosition[axis];
        }
        if(axis < static_cast<int>(savedFrame.feedbackUnitPosition.size())){
            axisState.savedFeedbackUnitPosition = savedFrame.feedbackUnitPosition[axis];
        }
        if(axis < static_cast<int>(currentFrame.commandUnitPosition.size())){
            axisState.currentCommandUnitPosition = currentFrame.commandUnitPosition[axis];
        }
        if(axis < static_cast<int>(currentFrame.feedbackUnitPosition.size())){
            axisState.currentFeedbackUnitPosition = currentFrame.feedbackUnitPosition[axis];
        }

        axisState.commandMismatch =
                axisState.savedCommandValid &&
                axisState.currentCommandValid &&
                axisState.savedCommandRawPulse != axisState.currentCommandRawPulse;
        axisState.feedbackMismatch =
                axisState.savedFeedbackValid &&
                axisState.currentFeedbackValid &&
                axisState.savedFeedbackRawPulse != axisState.currentFeedbackRawPulse;
        if(std::isfinite(axisState.axisEquiv) && axisState.axisEquiv > 0.0){
            if(axisState.savedCommandValid && axisState.currentCommandValid){
                axisState.commandDeltaUnit =
                        static_cast<double>(axisState.savedCommandRawPulse -
                                            axisState.currentCommandRawPulse) /
                        axisState.axisEquiv;
            }
            if(axisState.savedFeedbackValid && axisState.currentFeedbackValid){
                axisState.feedbackDeltaUnit =
                        static_cast<double>(axisState.savedFeedbackRawPulse -
                                            axisState.currentFeedbackRawPulse) /
                        axisState.axisEquiv;
            }
            axisState.restoreAvailable =
                    axisState.savedCommandValid &&
                    axisState.currentCommandValid &&
                    std::isfinite(axisState.commandDeltaUnit);
        }

        state.hasMismatch = state.hasMismatch ||
                axisState.commandMismatch ||
                axisState.feedbackMismatch;
        state.axes.push_back(axisState);
    }

    if(state.hasMismatch){
        state.message = QStringLiteral("Trace recovery comparison found command/feedback mismatch.");
    }
    else{
        state.message = QStringLiteral("Trace recovery comparison matched the saved trace window.");
    }
    cachedMotorTraceRecoveryStateData = state;
    return state;
    });
}

HardwareInterface::MotorTraceRecoveryState HardwareInterface::cachedMotorTraceRecoveryState() const
{
    return runOnHardwareThread([&]() -> MotorTraceRecoveryState {
    return cachedMotorTraceRecoveryStateData;
    });
}

std::vector<double> HardwareInterface::currentMotorPositionCachedValues(const std::vector<int>& motorIndex) const
{
    std::vector<double> result;
    result.reserve(motorIndex.size());
    for(const int axis : motorIndex){
        if(axis < 0 ||
                axis >= static_cast<int>(motorComType.size()) ||
                axis >= static_cast<int>(motorTraceActualPos.size()) ||
                motorComType[axis] != COM_EC_LS){
            return {};
        }
        double position = motorTraceActualPos[axis];
        if(axis < static_cast<int>(motorActualTraceOffsetUnit.size()) &&
                axis < static_cast<int>(motorActualTraceOffsetValid.size()) &&
                motorActualTraceOffsetValid[axis]){
            position -= motorActualTraceOffsetUnit[axis];
        }
        result.push_back(position);
    }
    return result;
}

bool HardwareInterface::configureMotorPositionTraceRead()
{
    return configureRuntimeTraceRead();
}

bool HardwareInterface::configureRuntimeTraceRead()
{
    if(!isConnectLS || runtimeTraceUnavailable){
        return false;
    }

    runtimeTraceConfigReadbackValid = false;
    runtimeTraceTimingReliable = false;
    runtimeTraceFifoCaughtUp = false;
    runtimeTraceLost = false;
    runtimeTraceHostTimeAnchorValid = false;

    runtimeTraceObjects.clear();
    motorCommandPositionTraceObjects.clear();
    motorPositionTraceObjects.clear();
    motorCommandVelocityTraceObjects.clear();
    motorActualVelocityTraceObjects.clear();
    motorStatusWordTraceObjects.clear();
    motorTorqueTraceObjects.clear();
    motorTraceCommandVelocity.assign(motorIdVec.size(), 0.0);
    motorTraceActualVelocity.assign(motorIdVec.size(), 0.0);
    motorTraceCommandVelocityValid.assign(motorIdVec.size(), false);
    motorTraceActualVelocityValid.assign(motorIdVec.size(), false);
    motorTraceStatusWord.assign(motorIdVec.size(), 0);
    motorTraceStatusWordValid.assign(motorIdVec.size(), false);
    motorTraceStatusWordMonotonicUs.assign(motorIdVec.size(), 0);
    motorTraceTorqueNm.assign(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
    motorTraceTorqueValid.assign(motorIdVec.size(), false);
    motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);

    struct RuntimeTraceAxis {
        int logicalAxis = -1;
        int hardwareAxis = -1;
    };
    std::vector<RuntimeTraceAxis> traceAxes;
    traceAxes.reserve(motorIdVec.size());
    for(int i = 0; i < static_cast<int>(motorIdVec.size()); ++i){
        if(i >= static_cast<int>(motorComType.size()) ||
                motorComType[i] != COM_EC_LS){
            continue;
        }
        if(runtimeTraceCommissioningAxis >= 0 &&
                i != runtimeTraceCommissioningAxis){
            continue;
        }
        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        if(hardwareAxis < 0){
            continue;
        }
        if(activeRuntimeTraceConfigType == RuntimeTraceConfigType::G302 &&
                activeLiteRuntimeTraceTopology ==
                    LiteRuntimeTraceTopology::TemporarySevenAxisSensorSlave1008 &&
                hardwareAxis == 7){
            continue;
        }
        traceAxes.push_back({i, hardwareAxis});
    }

    struct DefaultTraceMap {
        int sensorIndex = -1;
        short slaveId = 0;
        int subIndex = 0;
        short apiDataBytes = 0;
        int valueBytes = 2;
    };
    struct RuntimeTraceProfile {
        int feedbackAndTorqueLogicalAxisCount = kCableMotorTraceLogicalAxisCount;
        std::vector<int> traceDataIndexByLogicalAxis;
        std::vector<short> torqueTraceSlaveIdByLogicalAxis;
        std::vector<DefaultTraceMap> forceSensorTraceMap;
    };
    const auto g3RuntimeTraceProfile = []() {
        RuntimeTraceProfile profile;
        profile.feedbackAndTorqueLogicalAxisCount = kCableMotorTraceLogicalAxisCount;
        profile.forceSensorTraceMap = {
            {0, 1004, 1},
            {1, 1004, 2},
            {2, 1004, 3},
            {3, 1004, 4},
            {4, 1011, 1},
            {5, 1011, 2},
            {6, 1011, 3},
            {7, 1011, 4}
        };
        return profile;
    };
    const auto liteRuntimeTraceProfile = [&]() {
        RuntimeTraceProfile profile = g3RuntimeTraceProfile();
        // Standard G302 hardware axes 0..7 map to slaves 1001..1008.  In the
        // temporary seven-axis topology hardware axis 7 is filtered above and
        // the force transmitter occupies the now-vacant slave address 1008.
        profile.traceDataIndexByLogicalAxis = {
            4,
            5,
            7,
            6,
            0,
            1,
            3,
            2
        };
        profile.torqueTraceSlaveIdByLogicalAxis = {
            1005,
            1006,
            1008,
            1007,
            1001,
            1002,
            1004,
            1003
        };
        const short forceSensorSlaveId =
                activeLiteRuntimeTraceTopology ==
                    LiteRuntimeTraceTopology::TemporarySevenAxisSensorSlave1008 ?
                    1008 : 1009;
        profile.forceSensorTraceMap = {
            {0, forceSensorSlaveId, 1, 4, 4},
            {1, forceSensorSlaveId, 2, 4, 4},
            {2, forceSensorSlaveId, 3, 4, 4},
            {3, forceSensorSlaveId, 4, 4, 4},
            {4, forceSensorSlaveId, 5, 4, 4},
            {5, forceSensorSlaveId, 6, 4, 4},
            {6, forceSensorSlaveId, 7, 4, 4},
            {7, forceSensorSlaveId, 8, 4, 4}
        };
        return profile;
    };
    const RuntimeTraceProfile traceProfile =
            activeRuntimeTraceConfigType == RuntimeTraceConfigType::G302 ?
                liteRuntimeTraceProfile() :
                g3RuntimeTraceProfile();
    const auto traceDataIndexForAxis = [&](const RuntimeTraceAxis& axis) -> int {
        if(axis.logicalAxis >= 0 &&
                axis.logicalAxis < static_cast<int>(traceProfile.traceDataIndexByLogicalAxis.size()) &&
                traceProfile.traceDataIndexByLogicalAxis[axis.logicalAxis] >= 0){
            return traceProfile.traceDataIndexByLogicalAxis[axis.logicalAxis];
        }
        return axis.hardwareAxis;
    };
    const auto torqueTraceSlaveIdForAxis = [&](const RuntimeTraceAxis& axis) -> short {
        if(axis.logicalAxis >= 0 &&
                axis.logicalAxis < static_cast<int>(traceProfile.torqueTraceSlaveIdByLogicalAxis.size()) &&
                traceProfile.torqueTraceSlaveIdByLogicalAxis[axis.logicalAxis] > 0){
            return traceProfile.torqueTraceSlaveIdByLogicalAxis[axis.logicalAxis];
        }
        return resolveLeadshineTraceSlaveId(axis.logicalAxis, axis.hardwareAxis);
    };

    for(const RuntimeTraceAxis& axis : traceAxes){
        MotorCommandPositionTraceObject object;
        object.logicalAxis = axis.logicalAxis;
        object.hardwareAxis = axis.hardwareAxis;
        object.dataType = kLeadshineTraceDataTypeCommandPosition;
        object.dataIndex = traceDataIndexForAxis(axis);
        object.dataSubIndex = 0;
        object.slaveId = 0;
        object.apiDataBytes = kLeadshineTracePositionDataBytes;
        object.valueBytes = kLeadshineTracePositionDataBytes;
        motorCommandPositionTraceObjects.push_back(object);

        RuntimeTraceObject runtimeObject;
        runtimeObject.kind = RuntimeTraceObjectKind::MotorCommandPosition;
        runtimeObject.objectIndex = static_cast<int>(motorCommandPositionTraceObjects.size()) - 1;
        runtimeObject.valueBytes = object.valueBytes;
        runtimeTraceObjects.push_back(runtimeObject);
    }

    for(const RuntimeTraceAxis& axis : traceAxes){
        if(axis.logicalAxis < 0 ||
                axis.logicalAxis >= traceProfile.feedbackAndTorqueLogicalAxisCount){
            continue;
        }
        MotorPositionTraceObject object;
        object.logicalAxis = axis.logicalAxis;
        object.hardwareAxis = axis.hardwareAxis;
        object.dataType = kLeadshineTraceDataTypeActualPosition;
        object.dataIndex = traceDataIndexForAxis(axis);
        object.dataSubIndex = 0;
        object.slaveId = 0;
        object.apiDataBytes = kLeadshineTracePositionDataBytes;
        object.valueBytes = kLeadshineTracePositionDataBytes;
        motorPositionTraceObjects.push_back(object);

        RuntimeTraceObject runtimeObject;
        runtimeObject.kind = RuntimeTraceObjectKind::MotorPosition;
        runtimeObject.objectIndex = static_cast<int>(motorPositionTraceObjects.size()) - 1;
        runtimeObject.valueBytes = object.valueBytes;
        runtimeTraceObjects.push_back(runtimeObject);
    }

    if(runtimeTraceUsageProfileIncludesVelocitySignals(
                activeRuntimeTraceUsageProfile)){
        for(const RuntimeTraceAxis& axis : traceAxes){
            if(axis.logicalAxis < 0 ||
                    axis.logicalAxis >= traceProfile.feedbackAndTorqueLogicalAxisCount){
                continue;
            }
            MotorVelocityTraceObject object;
            object.logicalAxis = axis.logicalAxis;
            object.hardwareAxis = axis.hardwareAxis;
            object.dataType = kLeadshineTraceDataTypeCommandVelocity;
            object.dataIndex = traceDataIndexForAxis(axis);
            object.dataSubIndex = 0;
            object.slaveId = 0;
            object.apiDataBytes = kLeadshineTraceAutomaticDataBytes;
            object.valueBytes = kLeadshineTracePositionDataBytes;
            motorCommandVelocityTraceObjects.push_back(object);

            RuntimeTraceObject runtimeObject;
            runtimeObject.kind = RuntimeTraceObjectKind::MotorCommandVelocity;
            runtimeObject.objectIndex =
                    static_cast<int>(motorCommandVelocityTraceObjects.size()) - 1;
            runtimeObject.valueBytes = object.valueBytes;
            runtimeTraceObjects.push_back(runtimeObject);
        }

        for(const RuntimeTraceAxis& axis : traceAxes){
            if(axis.logicalAxis < 0 ||
                    axis.logicalAxis >= traceProfile.feedbackAndTorqueLogicalAxisCount){
                continue;
            }
            MotorVelocityTraceObject object;
            object.logicalAxis = axis.logicalAxis;
            object.hardwareAxis = axis.hardwareAxis;
            object.dataType = kLeadshineTraceDataTypeActualVelocity;
            object.dataIndex = traceDataIndexForAxis(axis);
            object.dataSubIndex = 0;
            object.slaveId = 0;
            object.apiDataBytes = kLeadshineTraceAutomaticDataBytes;
            object.valueBytes = kLeadshineTracePositionDataBytes;
            motorActualVelocityTraceObjects.push_back(object);

            RuntimeTraceObject runtimeObject;
            runtimeObject.kind = RuntimeTraceObjectKind::MotorActualVelocity;
            runtimeObject.objectIndex =
                    static_cast<int>(motorActualVelocityTraceObjects.size()) - 1;
            runtimeObject.valueBytes = object.valueBytes;
            runtimeTraceObjects.push_back(runtimeObject);
        }

        // Read CiA 402 statusword through the generic slave PDO channel.  The
        // object is deliberately part of the same 500 us Trace frame as the
        // position and velocity feedback used by online control.
        for(const RuntimeTraceAxis& axis : traceAxes){
            if(axis.logicalAxis < 0 ||
                    axis.logicalAxis >= traceProfile.feedbackAndTorqueLogicalAxisCount){
                continue;
            }
            MotorStatusWordTraceObject object;
            object.logicalAxis = axis.logicalAxis;
            object.hardwareAxis = axis.hardwareAxis;
            object.dataType = kLeadshineTraceDataTypeGenericPdo;
            object.dataIndex = kLeadshineStatusWordIndex;
            object.dataSubIndex = kLeadshineStatusWordSubIndex;
            object.slaveId = resolveLeadshineTraceSlaveId(
                        axis.logicalAxis,
                        axis.hardwareAxis);
            object.apiDataBytes = kLeadshineTraceStatusWordDataBytes;
            object.valueBytes = kLeadshineTraceStatusWordDataBytes;
            motorStatusWordTraceObjects.push_back(object);

            RuntimeTraceObject runtimeObject;
            runtimeObject.kind = RuntimeTraceObjectKind::MotorStatusWord;
            runtimeObject.objectIndex =
                    static_cast<int>(motorStatusWordTraceObjects.size()) - 1;
            runtimeObject.valueBytes = object.valueBytes;
            runtimeTraceObjects.push_back(runtimeObject);
        }
    }

    for(const RuntimeTraceAxis& axis : traceAxes){
        if(axis.logicalAxis < 0 ||
                axis.logicalAxis >= traceProfile.feedbackAndTorqueLogicalAxisCount){
            continue;
        }
        MotorTorqueTraceObject object;
        object.logicalAxis = axis.logicalAxis;
        object.hardwareAxis = axis.hardwareAxis;
        object.dataType = kLeadshineTraceDataTypeFeedbackTorque;
        object.dataIndex = traceDataIndexForAxis(axis);
        object.dataSubIndex = 0;
        object.slaveId = torqueTraceSlaveIdForAxis(axis);
        object.apiDataBytes = kLeadshineTraceTorqueDataBytes;
        object.valueBytes = kLeadshineTraceTorqueDataBytes;
        motorTorqueTraceObjects.push_back(object);

        RuntimeTraceObject runtimeObject;
        runtimeObject.kind = RuntimeTraceObjectKind::MotorTorque;
        runtimeObject.objectIndex = static_cast<int>(motorTorqueTraceObjects.size()) - 1;
        runtimeObject.valueBytes = object.valueBytes;
        runtimeTraceObjects.push_back(runtimeObject);
    }

    forceSensorTraceObjects.clear();
    if(runtimeTraceUsageProfileIncludesForceSensors(
                activeRuntimeTraceUsageProfile)){
        for(const DefaultTraceMap& map : traceProfile.forceSensorTraceMap){
            if(map.sensorIndex < 0 ||
                    map.sensorIndex >= static_cast<int>(sensorComType.size()) ||
                    sensorComType[map.sensorIndex] != COM_EC_LS_SBT){
                continue;
            }
            if(runtimeTraceCommissioningAxis >= 0 &&
                    map.sensorIndex != runtimeTraceCommissioningSensor){
                continue;
            }
            ForceSensorTraceObject object;
            object.sensorIndex = map.sensorIndex;
            object.dataType = 19;
            object.dataIndex = 0x6000;
            object.dataSubIndex = map.subIndex;
            object.slaveId = map.slaveId;
            object.apiDataBytes = map.apiDataBytes;
            object.valueBytes = map.valueBytes;
            forceSensorTraceObjects.push_back(object);

            RuntimeTraceObject runtimeObject;
            runtimeObject.kind = RuntimeTraceObjectKind::ForceSensor;
            runtimeObject.objectIndex = static_cast<int>(forceSensorTraceObjects.size()) - 1;
            runtimeObject.valueBytes = object.valueBytes;
            runtimeTraceObjects.push_back(runtimeObject);
        }
    }

    if(runtimeTraceObjects.empty()){
        runtimeTraceUnavailable = true;
        return false;
    }

    const short stopRet = dmc_trace_data_stop(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_stop"));
    const short resetRet = dmc_trace_data_reset(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_reset"));
    if(resetRet != 0){
        runtimeTraceUnavailable = true;
        runtimeTraceConfigured = false;
        runtimeTraceConfigReadbackValid = false;
        emit displayInfoSignal(
                    QStringLiteral("Runtime Trace重配置前清空FIFO失败：stop返回%1，reset返回%2")
                    .arg(stopRet)
                    .arg(resetRet)
                    .toStdString(),
                    "error");
        return false;
    }

    // 板卡FIFO已经清空，主机侧也必须丢弃上一Trace会话的末帧和时间锚点，
    // 避免重连后的新鲜度校验暂时看到旧会话缓存。
    latestMotorTracePositionFrame = MotorTracePositionWindowFrame{};
    latestMotorTracePositionFrameValid = false;
    runtimeTraceHostTimeAnchorValid = false;
    runtimeTraceLastFrameWallClockUs = 0;
    runtimeTraceLastFrameMonotonicUs = 0;
    runtimeTraceNewestFrameAgeUs = -1;

    int tracePeriodUs = motorPositionTraceObjects.empty() ?
                forceSensorTraceSamplePeriodUs :
                motorPositionTraceSamplePeriodUs;
    if(!motorPositionTraceObjects.empty() && !forceSensorTraceObjects.empty()){
        tracePeriodUs = std::min(motorPositionTraceSamplePeriodUs, forceSensorTraceSamplePeriodUs);
    }
    DWORD ethercatBusCycleUs = 0;
    short ret = nmc_get_cycletime(0,
                            kLeadshineEtherCatPort,
                            &ethercatBusCycleUs);
    recordCommunicationEvent(false, QStringLiteral("nmc_get_cycletime"));
    if(ret != 0 || ethercatBusCycleUs == 0 ||
            ethercatBusCycleUs > static_cast<DWORD>(std::numeric_limits<int>::max())){
        runtimeTraceUnavailable = true;
        runtimeTraceConfigReadbackValid = false;
        return false;
    }
    runtimeTraceEthercatBusCycleUs = static_cast<int>(ethercatBusCycleUs);
    const int traceCycle = std::max(
                1,
                (tracePeriodUs + runtimeTraceEthercatBusCycleUs - 1) /
                    runtimeTraceEthercatBusCycleUs);
    if(traceCycle > std::numeric_limits<short>::max()){
        runtimeTraceUnavailable = true;
        runtimeTraceConfigReadbackValid = false;
        return false;
    }
    ret = dmc_trace_set_config(0,
                               static_cast<short>(traceCycle),
                               0,
                               0,
                               0,
                               0,
                               0,
                               0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_set_config"));
    if(ret != 0){
        runtimeTraceUnavailable = true;
        runtimeTraceConfigReadbackValid = false;
        return false;
    }

    short readbackTraceCycle = 0;
    short readbackLostHandle = 0;
    short readbackTraceType = 0;
    short readbackTriggerObjectIndex = 0;
    short readbackTriggerType = 0;
    int readbackMask = 0;
    long long readbackCondition = 0;
    ret = dmc_trace_get_config(0,
                               &readbackTraceCycle,
                               &readbackLostHandle,
                               &readbackTraceType,
                               &readbackTriggerObjectIndex,
                               &readbackTriggerType,
                               &readbackMask,
                               &readbackCondition);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_config"));
    if(ret != 0 || readbackTraceCycle != traceCycle || readbackTraceCycle <= 0){
        runtimeTraceUnavailable = true;
        runtimeTraceConfigReadbackValid = false;
        return false;
    }
    runtimeTraceConfiguredCycle = readbackTraceCycle;
    runtimeTraceSamplePeriodUs = runtimeTraceEthercatBusCycleUs *
            runtimeTraceConfiguredCycle;
    runtimeTraceConfigReadbackValid = true;

    ret = dmc_trace_reset_config_object(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_reset_config_object"));
    if(ret != 0){
        runtimeTraceUnavailable = true;
        return false;
    }

    const auto addTraceConfigObject = [&](short dataType,
                                          int dataIndex,
                                          int dataSubIndex,
                                          short slaveId,
                                          short apiDataBytes) -> bool {
        ret = dmc_trace_add_config_object(0,
                                          dataType,
                                          dataIndex,
                                          dataSubIndex,
                                          slaveId,
                                          apiDataBytes);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_add_config_object"));
        if(ret != 0){
            dmc_trace_data_stop(0);
            recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_stop"));
            runtimeTraceUnavailable = true;
            return false;
        }
        return true;
    };

    for(const RuntimeTraceObject& runtimeObject : runtimeTraceObjects){
        bool addOk = false;
        if(runtimeObject.kind == RuntimeTraceObjectKind::MotorCommandPosition){
            if(runtimeObject.objectIndex >= 0 &&
                    runtimeObject.objectIndex < static_cast<int>(motorCommandPositionTraceObjects.size())){
                const MotorCommandPositionTraceObject& object =
                        motorCommandPositionTraceObjects[runtimeObject.objectIndex];
                addOk = addTraceConfigObject(object.dataType,
                                             object.dataIndex,
                                             object.dataSubIndex,
                                             object.slaveId,
                                             object.apiDataBytes);
            }
        }
        else if(runtimeObject.kind == RuntimeTraceObjectKind::MotorPosition){
            if(runtimeObject.objectIndex >= 0 &&
                    runtimeObject.objectIndex < static_cast<int>(motorPositionTraceObjects.size())){
                const MotorPositionTraceObject& object =
                        motorPositionTraceObjects[runtimeObject.objectIndex];
                addOk = addTraceConfigObject(object.dataType,
                                             object.dataIndex,
                                             object.dataSubIndex,
                                             object.slaveId,
                                             object.apiDataBytes);
            }
        }
        else if(runtimeObject.kind == RuntimeTraceObjectKind::MotorCommandVelocity){
            if(runtimeObject.objectIndex >= 0 &&
                    runtimeObject.objectIndex < static_cast<int>(motorCommandVelocityTraceObjects.size())){
                const MotorVelocityTraceObject& object =
                        motorCommandVelocityTraceObjects[runtimeObject.objectIndex];
                addOk = addTraceConfigObject(object.dataType,
                                             object.dataIndex,
                                             object.dataSubIndex,
                                             object.slaveId,
                                             object.apiDataBytes);
            }
        }
        else if(runtimeObject.kind == RuntimeTraceObjectKind::MotorActualVelocity){
            if(runtimeObject.objectIndex >= 0 &&
                    runtimeObject.objectIndex < static_cast<int>(motorActualVelocityTraceObjects.size())){
                const MotorVelocityTraceObject& object =
                        motorActualVelocityTraceObjects[runtimeObject.objectIndex];
                addOk = addTraceConfigObject(object.dataType,
                                             object.dataIndex,
                                             object.dataSubIndex,
                                             object.slaveId,
                                             object.apiDataBytes);
            }
        }
        else if(runtimeObject.kind == RuntimeTraceObjectKind::MotorStatusWord){
            if(runtimeObject.objectIndex >= 0 &&
                    runtimeObject.objectIndex < static_cast<int>(motorStatusWordTraceObjects.size())){
                const MotorStatusWordTraceObject& object =
                        motorStatusWordTraceObjects[runtimeObject.objectIndex];
                addOk = addTraceConfigObject(object.dataType,
                                             object.dataIndex,
                                             object.dataSubIndex,
                                             object.slaveId,
                                             object.apiDataBytes);
            }
        }
        else if(runtimeObject.kind == RuntimeTraceObjectKind::MotorTorque){
            if(runtimeObject.objectIndex >= 0 &&
                    runtimeObject.objectIndex < static_cast<int>(motorTorqueTraceObjects.size())){
                const MotorTorqueTraceObject& object =
                        motorTorqueTraceObjects[runtimeObject.objectIndex];
                addOk = addTraceConfigObject(object.dataType,
                                             object.dataIndex,
                                             object.dataSubIndex,
                                             object.slaveId,
                                             object.apiDataBytes);
            }
        }
        else if(runtimeObject.kind == RuntimeTraceObjectKind::ForceSensor){
            if(runtimeObject.objectIndex >= 0 &&
                    runtimeObject.objectIndex < static_cast<int>(forceSensorTraceObjects.size())){
                const ForceSensorTraceObject& object =
                        forceSensorTraceObjects[runtimeObject.objectIndex];
                addOk = addTraceConfigObject(object.dataType,
                                             object.dataIndex,
                                             object.dataSubIndex,
                                             object.slaveId,
                                             object.apiDataBytes);
            }
        }
        if(!addOk){
            runtimeTraceUnavailable = true;
            return false;
        }
    }

    ret = dmc_trace_data_start(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_start"));
    if(ret != 0){
        runtimeTraceUnavailable = true;
        return false;
    }

    runtimeTraceConfigured = true;
    runtimeTraceEverRead = false;
    runtimeTraceTimingReliable = false;
    runtimeTraceFifoCaughtUp = false;
    runtimeTraceLost = false;
    runtimeTraceConsecutiveFailures = 0;
    runtimeTraceObjectTotalBytes = 0;
    runtimeTraceObjectTotalNum = 0;
    runtimeTraceLastFifoValidNum = 0;
    runtimeTraceLastFifoFreeNum = 0;
    runtimeTraceLastRetryUs = 0;
    runtimeTraceLastFrameWallClockUs = 0;
    runtimeTraceLastFrameMonotonicUs = 0;
    runtimeTraceNewestFrameAgeUs = -1;
    runtimeTraceLastFrameSequenceValid = false;
    runtimeTraceLastFrameSequence = 0;
    runtimeTraceSequenceInitialized = false;
    runtimeTraceLastRawSequence = 0;
    runtimeTraceLastLogicalSequence = 0;
    runtimeTraceHostTimeAnchorValid = false;
    runtimeTraceHostTimeAnchorSequence = 0;
    runtimeTraceHostTimeAnchorWallClockUs = 0;
    runtimeTraceHostTimeAnchorMonotonicUs = 0;
    ++runtimeTraceConfigurationGeneration;
    return true;
}

void HardwareInterface::armPvtTraceStartDelayMeasurement(
        const std::vector<int>& motorIndex,
        int pointCount,
        qint64 pvtUploadMonotonicUs)
{
    pvtTraceStartDelayState = PvtTraceStartDelayState{};
    if(!latestMotorTracePositionFrameValid ||
            motorIndex.empty() ||
            pointCount <= 0 ||
            pvtUploadMonotonicUs <= 0){
        return;
    }

    const MotorTracePositionWindowFrame& baseline =
            latestMotorTracePositionFrame;
    PvtTraceStartDelayState state;
    state.pvtUploadMonotonicUs = pvtUploadMonotonicUs;
    state.pointCount = pointCount;
    state.axisCount = static_cast<int>(motorIndex.size());
    state.ethercatBusCycleUs = std::max(1, runtimeTraceEthercatBusCycleUs);
    state.commandBaselineRawPulse.assign(motorIdVec.size(), 0);
    state.feedbackBaselineRawPulse.assign(motorIdVec.size(), 0);
    state.commandBaselineValid.assign(motorIdVec.size(), false);
    state.feedbackBaselineValid.assign(motorIdVec.size(), false);

    for(const int axis : motorIndex){
        if(axis < 0 ||
                axis >= kCableMotorTraceLogicalAxisCount ||
                axis >= static_cast<int>(motorIdVec.size()) ||
                axis >= static_cast<int>(baseline.commandRawPulse.size()) ||
                axis >= static_cast<int>(baseline.feedbackRawPulse.size()) ||
                axis >= static_cast<int>(baseline.commandValid.size()) ||
                axis >= static_cast<int>(baseline.feedbackValid.size()) ||
                !baseline.commandValid[axis] ||
                !baseline.feedbackValid[axis]){
            continue;
        }
        state.motorIndex.push_back(axis);
        state.commandBaselineRawPulse[axis] = baseline.commandRawPulse[axis];
        state.feedbackBaselineRawPulse[axis] = baseline.feedbackRawPulse[axis];
        state.commandBaselineValid[axis] = true;
        state.feedbackBaselineValid[axis] = true;
    }
    state.active = !state.motorIndex.empty();
    pvtTraceStartDelayState = std::move(state);
}

void HardwareInterface::observePvtTraceStartDelayFrame(
        quint32 frameSequence,
        const std::vector<qint64>& commandRawPulse,
        const std::vector<bool>& commandValid,
        const std::vector<qint64>& feedbackRawPulse,
        const std::vector<bool>& feedbackValid)
{
    if(!pvtTraceStartDelayState.active){
        return;
    }

    if(!pvtTraceStartDelayState.commandStartFound){
        for(const int axis : pvtTraceStartDelayState.motorIndex){
            if(axis < 0 ||
                    axis >= static_cast<int>(commandRawPulse.size()) ||
                    axis >= static_cast<int>(commandValid.size()) ||
                    axis >= static_cast<int>(pvtTraceStartDelayState.commandBaselineRawPulse.size()) ||
                    axis >= static_cast<int>(pvtTraceStartDelayState.commandBaselineValid.size()) ||
                    !commandValid[axis] ||
                    !pvtTraceStartDelayState.commandBaselineValid[axis] ||
                    commandRawPulse[axis] ==
                        pvtTraceStartDelayState.commandBaselineRawPulse[axis]){
                continue;
            }
            pvtTraceStartDelayState.commandStartFound = true;
            pvtTraceStartDelayState.commandStartFrameSequence = frameSequence;
            pvtTraceStartDelayState.commandStartAxis = axis;
            break;
        }
    }

    if(!pvtTraceStartDelayState.commandStartFound){
        return;
    }

    int feedbackStartAxis = -1;
    for(const int axis : pvtTraceStartDelayState.motorIndex){
        if(axis < 0 ||
                axis >= static_cast<int>(commandRawPulse.size()) ||
                axis >= static_cast<int>(commandValid.size()) ||
                axis >= static_cast<int>(feedbackRawPulse.size()) ||
                axis >= static_cast<int>(feedbackValid.size()) ||
                axis >= static_cast<int>(pvtTraceStartDelayState.commandBaselineRawPulse.size()) ||
                axis >= static_cast<int>(pvtTraceStartDelayState.commandBaselineValid.size()) ||
                axis >= static_cast<int>(pvtTraceStartDelayState.feedbackBaselineRawPulse.size()) ||
                axis >= static_cast<int>(pvtTraceStartDelayState.feedbackBaselineValid.size()) ||
                !commandValid[axis] ||
                !pvtTraceStartDelayState.commandBaselineValid[axis] ||
                !feedbackValid[axis] ||
                !pvtTraceStartDelayState.feedbackBaselineValid[axis]){
            continue;
        }
        const qint64 commandDelta = commandRawPulse[axis] -
                pvtTraceStartDelayState.commandBaselineRawPulse[axis];
        const qint64 feedbackDelta = feedbackRawPulse[axis] -
                pvtTraceStartDelayState.feedbackBaselineRawPulse[axis];
        if(commandDelta == 0 ||
                feedbackDelta == 0 ||
                (commandDelta > 0) != (feedbackDelta > 0)){
            continue;
        }
        feedbackStartAxis = axis;
        break;
    }
    if(feedbackStartAxis < 0){
        return;
    }

    const quint32 commandStartFrameSequence =
            pvtTraceStartDelayState.commandStartFrameSequence;
    const quint32 feedbackStartFrameSequence = frameSequence;
    const quint64 frameIntervalCount = static_cast<quint32>(
                feedbackStartFrameSequence - commandStartFrameSequence);
    const int ethercatBusCycleUs =
            std::max(1, pvtTraceStartDelayState.ethercatBusCycleUs);
    const qint64 delayUs = static_cast<qint64>(frameIntervalCount) *
            static_cast<qint64>(ethercatBusCycleUs);
    const qint64 pvtUploadMonotonicUs =
            pvtTraceStartDelayState.pvtUploadMonotonicUs;
    const int pointCount = pvtTraceStartDelayState.pointCount;
    const int axisCount = pvtTraceStartDelayState.axisCount;
    const int commandStartAxis = pvtTraceStartDelayState.commandStartAxis;
    pvtTraceStartDelayState.active = false;

    {
        QMutexLocker locker(&diagnosticsMutex);
        for(int sampleIndex = pvtTableUploadTimingSamples.size() - 1;
            sampleIndex >= 0;
            --sampleIndex){
            PvtTableUploadTimingSample& sample =
                    pvtTableUploadTimingSamples[sampleIndex];
            if(sample.monotonicUs != pvtUploadMonotonicUs){
                continue;
            }
            sample.traceStartDelayValid = true;
            sample.traceCommandStartFrameSequence = commandStartFrameSequence;
            sample.traceFeedbackStartFrameSequence = feedbackStartFrameSequence;
            sample.traceStartDelayFrameCount = frameIntervalCount;
            sample.ethercatBusCycleUs = ethercatBusCycleUs;
            sample.traceStartDelayUs = delayUs;
            sample.traceCommandStartAxis = commandStartAxis;
            sample.traceFeedbackStartAxis = feedbackStartAxis;
            break;
        }
    }

    emit pvtTraceStartDelayMeasured(pvtUploadMonotonicUs,
                                    pointCount,
                                    axisCount,
                                    commandStartFrameSequence,
                                    feedbackStartFrameSequence,
                                    frameIntervalCount,
                                    ethercatBusCycleUs,
                                    delayUs,
                                    commandStartAxis,
                                    feedbackStartAxis);
}

bool HardwareInterface::decodeRuntimeTraceFrame(
        const unsigned char* frameData,
        int frameBytes,
        int objectValueStartOffset,
        qint64 frameWallClockUs,
        qint64 frameMonotonicUs,
        quint32 frameSequence,
        bool frameSequenceValid,
        bool appendHistorySamples,
        bool recordRawDiagnostic)
{
    if(frameData == nullptr || frameBytes <= 0 ||
            objectValueStartOffset < 0 ||
            objectValueStartOffset > frameBytes){
        return false;
    }

    runtimeTraceLastFrameWallClockUs = frameWallClockUs;
    runtimeTraceLastFrameMonotonicUs = frameMonotonicUs;
    runtimeTraceLastFrameSequence = frameSequence;
    runtimeTraceLastFrameSequenceValid = frameSequenceValid;

    std::vector<qint64> frameCommandRawPulse(motorIdVec.size(), 0);
    std::vector<qint64> frameFeedbackRawPulse(motorIdVec.size(), 0);
    std::vector<bool> frameCommandRawPulseValid(motorIdVec.size(), false);
    std::vector<bool> frameFeedbackRawPulseValid(motorIdVec.size(), false);

    int objectOffset = objectValueStartOffset;
    for(const RuntimeTraceObject& object : runtimeTraceObjects){
        const int valueBytes = std::max(1, object.valueBytes);
        if(objectOffset + valueBytes > frameBytes){
            return false;
        }
        const unsigned char* raw = frameData + objectOffset;
        const long rawValue = readSignedLittleEndianTraceValue(raw, valueBytes);
        if(object.kind == RuntimeTraceObjectKind::MotorCommandPosition){
            if(object.objectIndex >= 0 &&
                    object.objectIndex < static_cast<int>(motorCommandPositionTraceObjects.size())){
                const int logicalAxis =
                        motorCommandPositionTraceObjects[object.objectIndex].logicalAxis;
                applyMotorCommandPositionTraceRawValue(logicalAxis, rawValue);
                if(logicalAxis >= 0 &&
                        logicalAxis < static_cast<int>(frameCommandRawPulse.size())){
                    frameCommandRawPulse[logicalAxis] = static_cast<qint64>(rawValue);
                    frameCommandRawPulseValid[logicalAxis] = true;
                }
            }
        }
        else if(object.kind == RuntimeTraceObjectKind::MotorPosition){
            if(object.objectIndex >= 0 &&
                    object.objectIndex < static_cast<int>(motorPositionTraceObjects.size())){
                const int logicalAxis =
                        motorPositionTraceObjects[object.objectIndex].logicalAxis;
                applyMotorPositionTraceRawValue(logicalAxis, rawValue);
                if(logicalAxis >= 0 &&
                        logicalAxis < static_cast<int>(frameFeedbackRawPulse.size())){
                    frameFeedbackRawPulse[logicalAxis] = static_cast<qint64>(rawValue);
                    frameFeedbackRawPulseValid[logicalAxis] = true;
                }
            }
        }
        else if(object.kind == RuntimeTraceObjectKind::MotorCommandVelocity){
            if(object.objectIndex >= 0 &&
                    object.objectIndex < static_cast<int>(motorCommandVelocityTraceObjects.size())){
                const int logicalAxis =
                        motorCommandVelocityTraceObjects[object.objectIndex].logicalAxis;
                applyMotorCommandVelocityTraceRawValue(logicalAxis, rawValue);
            }
        }
        else if(object.kind == RuntimeTraceObjectKind::MotorActualVelocity){
            if(object.objectIndex >= 0 &&
                    object.objectIndex < static_cast<int>(motorActualVelocityTraceObjects.size())){
                const int logicalAxis =
                        motorActualVelocityTraceObjects[object.objectIndex].logicalAxis;
                applyMotorActualVelocityTraceRawValue(logicalAxis, rawValue);
            }
        }
        else if(object.kind == RuntimeTraceObjectKind::MotorStatusWord){
            if(object.objectIndex >= 0 &&
                    object.objectIndex < static_cast<int>(motorStatusWordTraceObjects.size())){
                const int logicalAxis =
                        motorStatusWordTraceObjects[object.objectIndex].logicalAxis;
                if(logicalAxis >= 0 &&
                        logicalAxis < static_cast<int>(motorTraceStatusWord.size()) &&
                        valueBytes == kLeadshineTraceStatusWordDataBytes){
                    motorTraceStatusWord[logicalAxis] =
                            readUnsignedLittleEndianTraceWord(raw);
                    motorTraceStatusWordValid[logicalAxis] = true;
                    motorTraceStatusWordMonotonicUs[logicalAxis] =
                            frameMonotonicUs;
                }
            }
        }
        else if(object.kind == RuntimeTraceObjectKind::MotorTorque){
            if(object.objectIndex >= 0 &&
                    object.objectIndex < static_cast<int>(motorTorqueTraceObjects.size())){
                const int logicalAxis =
                        motorTorqueTraceObjects[object.objectIndex].logicalAxis;
                applyMotorTorqueTraceRawValue(logicalAxis,
                                              rawValue,
                                              frameMonotonicUs);
            }
        }
        else if(object.kind == RuntimeTraceObjectKind::ForceSensor){
            if(object.objectIndex >= 0 &&
                    object.objectIndex < static_cast<int>(forceSensorTraceObjects.size())){
                applyForceSensorRawValue(
                            forceSensorTraceObjects[object.objectIndex].sensorIndex,
                            rawValue,
                            frameMonotonicUs);
            }
        }
        objectOffset += valueBytes;
    }

    if(frameSequenceValid){
        observePvtTraceStartDelayFrame(frameSequence,
                                       frameCommandRawPulse,
                                       frameCommandRawPulseValid,
                                       frameFeedbackRawPulse,
                                       frameFeedbackRawPulseValid);
    }
    if(recordRawDiagnostic){
        recordMotorTraceFeedbackRawSample(frameWallClockUs,
                                          frameMonotonicUs,
                                          frameSequence,
                                          frameSequenceValid,
                                          frameFeedbackRawPulse,
                                          frameFeedbackRawPulseValid);
    }

    if(appendHistorySamples &&
            !motorCommandPositionTraceObjects.empty() &&
            !motorPositionTraceObjects.empty()){
        MotorTracePositionWindowFrame windowFrame;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        windowFrame.wallClockUs = frameWallClockUs;
        windowFrame.monotonicUs = frameMonotonicUs;
        windowFrame.commandUnitPosition.assign(motorIdVec.size(), nan);
        windowFrame.feedbackUnitPosition.assign(motorIdVec.size(), nan);
        windowFrame.commandRelativePosition.assign(motorIdVec.size(), nan);
        windowFrame.feedbackRelativePosition.assign(motorIdVec.size(), nan);
        windowFrame.commandRawPulse.assign(motorIdVec.size(), 0);
        windowFrame.feedbackRawPulse.assign(motorIdVec.size(), 0);
        windowFrame.commandValid.assign(motorIdVec.size(), false);
        windowFrame.feedbackValid.assign(motorIdVec.size(), false);
        bool hasWindowSample = false;
        for(int axis = 0; axis < static_cast<int>(motorIdVec.size()); ++axis){
            if(axis < static_cast<int>(frameCommandRawPulseValid.size()) &&
                    frameCommandRawPulseValid[axis]){
                windowFrame.commandRawPulse[axis] = frameCommandRawPulse[axis];
                windowFrame.commandUnitPosition[axis] =
                        tracePulseToMotorUnit(axis, frameCommandRawPulse[axis]);
                windowFrame.commandValid[axis] = true;
            }
            if(axis < static_cast<int>(frameFeedbackRawPulseValid.size()) &&
                    frameFeedbackRawPulseValid[axis]){
                windowFrame.feedbackRawPulse[axis] = frameFeedbackRawPulse[axis];
                windowFrame.feedbackUnitPosition[axis] =
                        tracePulseToMotorUnit(axis, frameFeedbackRawPulse[axis]);
                windowFrame.feedbackValid[axis] = true;
            }
        }

        for(const MotorPositionTraceObject& object : motorPositionTraceObjects){
            const int logicalAxis = object.logicalAxis;
            if(logicalAxis < 0 ||
                    logicalAxis >= static_cast<int>(motorComType.size()) ||
                    logicalAxis >= static_cast<int>(motorCommandPos.size()) ||
                    logicalAxis >= static_cast<int>(motorTraceActualPos.size()) ||
                    logicalAxis >= static_cast<int>(motorTracePositionSampleQueues.size()) ||
                    motorComType[logicalAxis] != COM_EC_LS ||
                    !std::isfinite(motorCommandPos[logicalAxis]) ||
                    !std::isfinite(motorTraceActualPos[logicalAxis])){
                continue;
            }
            if(!ensureMotorTracePositionOffsets(logicalAxis)){
                continue;
            }

            MotorTracePositionSample sample;
            sample.commandRelativePosition =
                    traceAlignedRelativePosition(logicalAxis,
                                                 motorCommandPos[logicalAxis],
                                                 motorCommandTraceOffsetUnit,
                                                 motorCommandTraceOffsetValid);
            sample.feedbackRelativePosition =
                    traceAlignedRelativePosition(logicalAxis,
                                                 motorTraceActualPos[logicalAxis],
                                                 motorActualTraceOffsetUnit,
                                                 motorActualTraceOffsetValid);
            sample.wallClockUs = frameWallClockUs;
            sample.monotonicUs = frameMonotonicUs;
            if(logicalAxis < static_cast<int>(frameCommandRawPulseValid.size()) &&
                    frameCommandRawPulseValid[logicalAxis]){
                sample.commandRawPulse = frameCommandRawPulse[logicalAxis];
                sample.commandRawPulseValid = true;
            }
            if(logicalAxis < static_cast<int>(frameFeedbackRawPulseValid.size()) &&
                    frameFeedbackRawPulseValid[logicalAxis]){
                sample.feedbackRawPulse = frameFeedbackRawPulse[logicalAxis];
                sample.feedbackRawPulseValid = true;
            }
            auto& samples = motorTracePositionSampleQueues[logicalAxis];
            samples.push_back(sample);
            while(samples.size() > kMaxMotorTracePositionSamplesPerAxis){
                samples.pop_front();
            }
            windowFrame.commandRelativePosition[logicalAxis] =
                    sample.commandRelativePosition;
            windowFrame.feedbackRelativePosition[logicalAxis] =
                    sample.feedbackRelativePosition;
            hasWindowSample = true;
        }
        if(hasWindowSample){
            latestMotorTracePositionFrame = windowFrame;
            latestMotorTracePositionFrameValid = true;
            const bool shouldAppendRecoveryFrame =
                    motorTracePositionWindowRecordingEnabled ||
                    (!motorTracePositionWindowFrozenAfterPvt && !hasActivePvtTrajectory);
            if(shouldAppendRecoveryFrame){
                appendMotorTracePositionWindowFrame(windowFrame);
            }
            if(motorTracePositionWindowRecordingEnabled && activePvtAxesDoneDirect()){
                motorTracePositionWindowRecordingEnabled = false;
                motorTracePositionWindowFrozenAfterPvt = true;
                pvtTraceStartDelayState.active = false;
            }
        }
    }

    if(appendHistorySamples && !forceSensorTraceObjects.empty()){
        ForceSensorTraceSample sample;
        sample.values = currentForceSensorCachedValues();
        sample.wallClockUs = frameWallClockUs;
        sample.monotonicUs = frameMonotonicUs;
        sample.frameSequence = frameSequence;
        sample.frameSequenceValid = frameSequenceValid;
        if(!sample.values.empty()){
            forceSensorTraceSampleQueue.push_back(std::move(sample));
            while(forceSensorTraceSampleQueue.size() > kMaxForceSensorTraceSamples){
                forceSensorTraceSampleQueue.pop_front();
            }
        }
    }
    return true;
}

void HardwareInterface::observeEndpointRemoteRuntimeTraceStatusWords(
        const unsigned char* frameData,
        int frameBytes,
        int objectValueStartOffset,
        quint64 logicalFrameSequence)
{
    if(activeRuntimeTraceUsageProfile !=
            RuntimeTraceUsageProfile::EndpointRemoteRunning ||
            endpointRemoteTraceStatusFaultLatched ||
            frameData == nullptr || frameBytes <= 0 ||
            objectValueStartOffset < 0 ||
            objectValueStartOffset > frameBytes){
        return;
    }

    int objectOffset = objectValueStartOffset;
    for(const RuntimeTraceObject& object : runtimeTraceObjects){
        const int valueBytes = std::max(1, object.valueBytes);
        if(objectOffset + valueBytes > frameBytes){
            return;
        }
        if(object.kind == RuntimeTraceObjectKind::MotorStatusWord &&
                object.objectIndex >= 0 &&
                object.objectIndex < static_cast<int>(
                    motorStatusWordTraceObjects.size()) &&
                valueBytes == kLeadshineTraceStatusWordDataBytes){
            const int logicalAxis =
                    motorStatusWordTraceObjects[object.objectIndex].logicalAxis;
            const quint16 statusWord = readUnsignedLittleEndianTraceWord(
                        frameData + objectOffset);
            const int stateMachine = decodeCia402StateMachine(statusWord);
            if(stateMachine != 4){
                endpointRemoteTraceStatusFaultLatched = true;
                endpointRemoteTraceStatusFaultAxis = logicalAxis;
                endpointRemoteTraceStatusFaultWord = statusWord;
                endpointRemoteTraceStatusFaultStateMachine = stateMachine;
                endpointRemoteTraceStatusFaultLogicalFrameSequence =
                        logicalFrameSequence;
                return;
            }
        }
        objectOffset += valueBytes;
    }
}

int HardwareInterface::readRuntimeTraceCached(bool latestOnly)
{
    runtimeTraceLastDataApiDurationUs = 0;
    const bool captureAttributionTiming =
            motorEnableQueryTimingEnabled.load(std::memory_order_acquire);
    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    if(motorTraceActualPos.size() != motorIdVec.size()){
        motorTraceActualPos.assign(motorIdVec.size(), 0.0);
    }
    if(motorTraceStatusWord.size() != motorIdVec.size()){
        motorTraceStatusWord.assign(motorIdVec.size(), 0);
    }
    if(motorTraceStatusWordValid.size() != motorIdVec.size()){
        motorTraceStatusWordValid.assign(motorIdVec.size(), false);
    }
    if(motorTraceStatusWordMonotonicUs.size() != motorIdVec.size()){
        motorTraceStatusWordMonotonicUs.assign(motorIdVec.size(), 0);
    }
    if(motorTraceTorqueNm.size() != motorIdVec.size()){
        motorTraceTorqueNm.assign(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
        motorTraceTorqueValid.assign(motorIdVec.size(), false);
        motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);
    }
    if(motorTraceTorqueValid.size() != motorIdVec.size()){
        motorTraceTorqueValid.assign(motorIdVec.size(), false);
    }
    if(motorTraceTorqueMonotonicUs.size() != motorIdVec.size()){
        motorTraceTorqueMonotonicUs.assign(motorIdVec.size(), 0);
    }
    if(motorCommandPos.size() != motorIdVec.size()){
        motorCommandPos.assign(motorIdVec.size(), 0.0);
    }
    if(motorTracePositionSampleQueues.size() != motorIdVec.size()){
        motorTracePositionSampleQueues.assign(motorIdVec.size(),
                                              std::deque<MotorTracePositionSample>());
    }
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
        forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
        nextForceSensorPollIndex = 0;
    }

    if(runtimeTraceUnavailable){
        const qint64 nowUs = monotonicNowUs();
        if(runtimeTraceLastRetryUs > 0 &&
                nowUs - runtimeTraceLastRetryUs < 1000 * 1000){
            return runtimeTraceEverRead ? 0 : -1;
        }
        runtimeTraceLastRetryUs = nowUs;
        runtimeTraceUnavailable = false;
        runtimeTraceConfigured = false;
        runtimeTraceConsecutiveFailures = 0;
    }

    const bool configureNow = !runtimeTraceConfigured;
    if(configureNow && !configureRuntimeTraceRead()){
        return runtimeTraceEverRead ? 0 : -1;
    }
    if(configureNow){
        delay(static_cast<unsigned int>(std::max(2, motorPositionTraceSamplePeriodUs / 1000 + 1)));
    }

    int validNum = 0;
    int freeNum = 0;
    int objectTotalBytes = 0;
    int objectTotalNum = 0;
    const short stateRet = dmc_trace_get_state(0,
                                               &validNum,
                                               &freeNum,
                                               &objectTotalBytes,
                                               &objectTotalNum);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_state"));
    if(stateRet != 0){
        ++runtimeTraceConsecutiveFailures;
        if(runtimeTraceConsecutiveFailures >= 5){
            resetRuntimeTraceState();
            runtimeTraceUnavailable = true;
            return -1;
        }
        return runtimeTraceEverRead ? 0 : -1;
    }
    runtimeTraceObjectTotalBytes = objectTotalBytes;
    runtimeTraceObjectTotalNum = objectTotalNum;
    runtimeTraceLastFifoValidNum = std::max(0, validNum);
    runtimeTraceLastFifoFreeNum = std::max(0, freeNum);
    if(freeNum <= 0){
        runtimeTraceTimingReliable = false;
        runtimeTraceHostTimeAnchorValid = false;
    }

    short traceStartFlag = 0;
    short traceTriggeredFlag = 0;
    short traceLostFlag = 0;
    const short traceFlagRet = dmc_trace_get_flag(0,
                                                   &traceStartFlag,
                                                   &traceTriggeredFlag,
                                                   &traceLostFlag);
    Q_UNUSED(traceTriggeredFlag);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_flag"));
    runtimeTraceLost = traceFlagRet != 0 || traceLostFlag != 0;
    if(traceFlagRet != 0 || traceStartFlag == 0 || runtimeTraceLost){
        runtimeTraceTimingReliable = false;
        runtimeTraceHostTimeAnchorValid = false;
    }
    if(traceFlagRet == 0 && traceLostFlag != 0){
        dmc_trace_reset_lost_flag(0);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_reset_lost_flag"));
    }
    if(validNum <= 0){
        ++runtimeTraceConsecutiveFailures;
        runtimeTraceFifoCaughtUp = true;
        return runtimeTraceEverRead ? 0 : -1;
    }
    runtimeTraceConsecutiveFailures = 0;

    int expectedValueBytes = 0;
    for(const RuntimeTraceObject& object : runtimeTraceObjects){
        const int valueBytes = std::max(1, object.valueBytes);
        expectedValueBytes += valueBytes;
    }
    const int configuredObjectCount = static_cast<int>(runtimeTraceObjects.size());
    const bool frameLayoutValid = configuredObjectCount > 0 &&
            objectTotalNum == configuredObjectCount &&
            objectTotalBytes >= expectedValueBytes;
    if(!frameLayoutValid){
        runtimeTraceTimingReliable = false;
        runtimeTraceFifoCaughtUp = false;
        runtimeTraceHostTimeAnchorValid = false;
        const qint64 nowUs = monotonicNowUs();
        if(runtimeTraceLayoutRejectLastWarnUs <= 0 ||
                nowUs - runtimeTraceLayoutRejectLastWarnUs >=
                    kRuntimeTraceLayoutRejectWarnIntervalUs){
            runtimeTraceLayoutRejectLastWarnUs = nowUs;
            emit displayInfoSignal(
                        QString("Runtime Trace frame layout rejected: configured_objects=%1 card_objects=%2 value_bytes=%3 frame_bytes=%4.")
                        .arg(configuredObjectCount)
                        .arg(objectTotalNum)
                        .arg(expectedValueBytes)
                        .arg(objectTotalBytes)
                        .toStdString(),
                        "error");
        }
        return runtimeTraceEverRead ? 0 : -1;
    }

    // The controller reports a fixed frame width. Object payloads are packed in
    // configuration order at the end of that frame; any leading bytes are the
    // controller header (including the hardware sequence when present).
    const int frameBytes = objectTotalBytes;
    const int objectValueStartOffset = objectTotalBytes - expectedValueBytes;

    // Only the explicit endpoint-remote running profile accepts reduced history density. FIFO
    // draining, every frame-header sequence check, timing anchoring and the
    // independent 5 ms freshness gate remain unchanged. Force/PVT,
    // commissioning, recovery and explicit latest-only readers keep the
    // original per-frame full decode path.
    const bool decodeOnlyNewestFrameForEndpointRemote =
            !latestOnly &&
            activeRuntimeTraceUsageProfile ==
                RuntimeTraceUsageProfile::EndpointRemoteRunning;
    std::vector<unsigned char> newestOnlineVelocityFrame;
    qint64 newestOnlineVelocityFrameWallClockUs = 0;
    qint64 newestOnlineVelocityFrameMonotonicUs = 0;
    quint32 newestOnlineVelocityFrameSequence = 0;
    bool newestOnlineVelocityFrameSequenceValid = false;

    int totalCompleteFrameCount = 0;
    int currentValidNum = validNum;
    // 只跟踪本次函数进入时已经存在的旧帧。后续SDK调用期间新产生的帧
    // 不能在下一批读取时再次升级为“历史积压”，否则会重新追逐移动尾部。
    int remainingPreExistingFrameCount = std::max(0, validNum);
    const qint64 drainStartMonotonicUs = monotonicNowUs();
    const int maxTraceDrainReads = latestOnly ? 1 : kRuntimeTraceMaxDrainReads;
    for(int drainIndex = 0; drainIndex < maxTraceDrainReads && currentValidNum > 0; ++drainIndex){
        const int maxBufferBytes = latestOnly ? 128 * 1024 : 64 * 1024;
        const int frameCapacity = std::max(1, maxBufferBytes / frameBytes);
        const int frameCountToRead = std::max(1, std::min(currentValidNum, frameCapacity));
        const int bufferSize = std::max(frameBytes, frameBytes * frameCountToRead);
        std::vector<unsigned char> buffer(static_cast<std::size_t>(bufferSize), 0);
        int actualReadLength = 0;
        const qint64 dataCallStartMonotonicUs = monotonicNowUs();
        const short dataRet = dmc_trace_get_data(0,
                                                 bufferSize,
                                                 buffer.data(),
                                                 &actualReadLength);
        const qint64 dataCallEndMonotonicUs = monotonicNowUs();
        if(captureAttributionTiming){
            runtimeTraceLastDataApiDurationUs += std::max<qint64>(
                        0, dataCallEndMonotonicUs - dataCallStartMonotonicUs);
        }
        const qint64 dataCallEndWallClockUs = wallClockNowUs();
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_data"));
        if(dataRet != 0 || actualReadLength < expectedValueBytes){
            return runtimeTraceEverRead || totalCompleteFrameCount > 0 ? totalCompleteFrameCount : -1;
        }

        const int completeFrameCount = actualReadLength / frameBytes;
        if(completeFrameCount <= 0){
            return runtimeTraceEverRead || totalCompleteFrameCount > 0 ? totalCompleteFrameCount : -1;
        }
        if(actualReadLength % frameBytes != 0){
            runtimeTraceTimingReliable = false;
            runtimeTraceHostTimeAnchorValid = false;
        }

        int nextValidNum = 0;
        int nextFreeNum = 0;
        int nextObjectTotalBytes = 0;
        int nextObjectTotalNum = 0;
        const short nextStateRet = dmc_trace_get_state(0,
                                                       &nextValidNum,
                                                       &nextFreeNum,
                                                       &nextObjectTotalBytes,
                                                       &nextObjectTotalNum);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_state"));
        const bool nextStateValid = nextStateRet == 0 &&
                nextObjectTotalBytes == frameBytes &&
                nextObjectTotalNum == configuredObjectCount;
        if(!nextStateValid){
            nextValidNum = std::max(0, currentValidNum - completeFrameCount);
            nextFreeNum = 0;
            runtimeTraceTimingReliable = false;
            runtimeTraceHostTimeAnchorValid = false;
        }
        else{
            runtimeTraceObjectTotalBytes = nextObjectTotalBytes;
            runtimeTraceObjectTotalNum = nextObjectTotalNum;
        }
        runtimeTraceLastFifoValidNum = std::max(0, nextValidNum);
        runtimeTraceLastFifoFreeNum = std::max(0, nextFreeNum);
        // FIFO按时间顺序返回帧，因此本批读取先消费函数进入时已有的旧帧。
        // API调用期间产生的新帧只参与帧年龄计算，不进入旧积压计数。
        const int remainingCurrentBatchFrameCount = std::max(
                    0,
                    currentValidNum - completeFrameCount);
        remainingPreExistingFrameCount = std::max(
                    0,
                    remainingPreExistingFrameCount - completeFrameCount);
        const int estimatedProducedFrameCount = std::max(
                    0,
                    nextValidNum - remainingCurrentBatchFrameCount);
        const bool batchFifoCaughtUp = nextStateValid &&
                nextFreeNum > 0 &&
                remainingPreExistingFrameCount == 0;

        std::vector<quint32> fetchFrameSequences;
        std::vector<quint64> fetchLogicalSequences;
        fetchFrameSequences.reserve(static_cast<std::size_t>(completeFrameCount));
        fetchLogicalSequences.reserve(static_cast<std::size_t>(completeFrameCount));
        bool fetchFrameSequenceValid = objectValueStartOffset >= 4;
        bool fetchFrameSequenceContinuous = fetchFrameSequenceValid;
        for(int frameIndex = 0; frameIndex < completeFrameCount; ++frameIndex){
            const int frameOffset = frameIndex * frameBytes;
            if(frameOffset + 4 <= actualReadLength && fetchFrameSequenceValid){
                const quint32 frameSequence =
                        readTraceFrameSequence(buffer.data() + frameOffset);
                fetchFrameSequences.push_back(frameSequence);
                if(!runtimeTraceSequenceInitialized){
                    runtimeTraceSequenceInitialized = true;
                    runtimeTraceLastRawSequence = frameSequence;
                    runtimeTraceLastLogicalSequence = 1;
                }
                else{
                    const quint32 sequenceIncrement =
                            frameSequence - runtimeTraceLastRawSequence;
                    const quint32 expectedIncrement = static_cast<quint32>(
                                std::max(1, runtimeTraceConfiguredCycle));
                    if(sequenceIncrement == 0 ||
                            sequenceIncrement > kRuntimeTraceMaximumSequenceIncrement){
                        fetchFrameSequenceContinuous = false;
                        ++runtimeTraceLastLogicalSequence;
                    }
                    else{
                        runtimeTraceLastLogicalSequence += sequenceIncrement;
                        if(sequenceIncrement != expectedIncrement){
                            fetchFrameSequenceContinuous = false;
                        }
                    }
                    runtimeTraceLastRawSequence = frameSequence;
                }
                fetchLogicalSequences.push_back(runtimeTraceLastLogicalSequence);
            }
            else{
                fetchFrameSequenceValid = false;
                fetchFrameSequenceContinuous = false;
                fetchFrameSequences.clear();
                fetchLogicalSequences.clear();
                break;
            }
        }

        const qint64 framePeriodUs = std::max<qint64>(1, runtimeTraceSamplePeriodUs);
        const qint64 newestFrameAgeUs = nextStateValid ?
                    static_cast<qint64>(std::max(0, nextValidNum)) * framePeriodUs :
                    -1;
        const bool batchTimingCandidate =
                runtimeTraceConfigReadbackValid &&
                nextStateValid &&
                nextFreeNum > 0 &&
                !runtimeTraceLost &&
                fetchFrameSequenceValid &&
                fetchFrameSequenceContinuous &&
                static_cast<int>(fetchLogicalSequences.size()) == completeFrameCount;
        if(!batchTimingCandidate){
            runtimeTraceTimingReliable = false;
            runtimeTraceHostTimeAnchorValid = false;
        }
        if(batchFifoCaughtUp && batchTimingCandidate){
            runtimeTraceHostTimeAnchorValid = true;
            runtimeTraceHostTimeAnchorSequence = fetchLogicalSequences.back();
            runtimeTraceHostTimeAnchorWallClockUs =
                    dataCallEndWallClockUs - newestFrameAgeUs;
            runtimeTraceHostTimeAnchorMonotonicUs =
                    dataCallEndMonotonicUs - newestFrameAgeUs;
        }
        runtimeTraceTimingReliable = batchTimingCandidate &&
                runtimeTraceHostTimeAnchorValid;
        runtimeTraceFifoCaughtUp = batchFifoCaughtUp;
        runtimeTraceNewestFrameAgeUs = newestFrameAgeUs;
        recordRuntimeTraceFetchTimingSample(
                    dataCallEndWallClockUs,
                    dataCallEndMonotonicUs,
                    std::max<qint64>(0,
                                     dataCallEndMonotonicUs -
                                     dataCallStartMonotonicUs),
                    actualReadLength,
                    frameBytes,
                    completeFrameCount,
                    frameCountToRead,
                    currentValidNum,
                    nextValidNum,
                    nextFreeNum,
                    estimatedProducedFrameCount,
                    runtimeTraceSamplePeriodUs,
                    newestFrameAgeUs,
                    latestOnly,
                    batchFifoCaughtUp,
                    runtimeTraceTimingReliable,
                    runtimeTraceLost,
                    fetchFrameSequenceValid,
                    fetchFrameSequences);

        // 末端遥控运行期即使只完整解析最新帧，也轻量扫描本批每一帧的
        // 0x6041。任何中间帧离开Operation enabled都会锁存，专用速度
        // 入口在发命令前失败关闭；不把异常瞬态藏在“仅解析最新帧”后面。
        if(activeRuntimeTraceUsageProfile ==
                RuntimeTraceUsageProfile::EndpointRemoteRunning){
            for(int frameIndex = 0; frameIndex < completeFrameCount; ++frameIndex){
                const int frameOffset = frameIndex * frameBytes;
                if(frameOffset + objectValueStartOffset + expectedValueBytes >
                        actualReadLength){
                    break;
                }
                const quint64 logicalSequence =
                        frameIndex < static_cast<int>(fetchLogicalSequences.size()) ?
                            fetchLogicalSequences[frameIndex] : 0;
                observeEndpointRemoteRuntimeTraceStatusWords(
                            buffer.data() + frameOffset,
                            frameBytes,
                            objectValueStartOffset,
                            logicalSequence);
            }
        }

        // 延迟测量期间必须逐帧解码，避免latestOnly跳过命令或反馈首次变化帧。
        const bool decodeAllFramesForPvtDelay = pvtTraceStartDelayState.active;
        const int firstFrameIndex =
                decodeOnlyNewestFrameForEndpointRemote ||
                (latestOnly && !decodeAllFramesForPvtDelay) ?
                    completeFrameCount - 1 :
                    0;
        for(int frameIndex = firstFrameIndex; frameIndex < completeFrameCount; ++frameIndex){
            const int frameOffset = frameIndex * frameBytes;
            if(frameOffset + objectValueStartOffset + expectedValueBytes > actualReadLength){
                return runtimeTraceEverRead || totalCompleteFrameCount > 0 ? totalCompleteFrameCount : -1;
            }

            const qint64 frameAgeUs =
                    static_cast<qint64>(std::max(0, nextValidNum) +
                                        completeFrameCount - frameIndex - 1) *
                    framePeriodUs;
            const bool frameSequenceValid = objectValueStartOffset >= 4 &&
                    frameOffset + 4 <= actualReadLength &&
                    frameIndex < static_cast<int>(fetchLogicalSequences.size());
            const quint32 frameSequence = frameSequenceValid ?
                        readTraceFrameSequence(buffer.data() + frameOffset) :
                        0;
            qint64 frameWallClockUs = dataCallEndWallClockUs - frameAgeUs;
            qint64 frameMonotonicUs = dataCallEndMonotonicUs - frameAgeUs;
            if(runtimeTraceHostTimeAnchorValid && frameSequenceValid){
                const quint64 logicalSequence = fetchLogicalSequences[frameIndex];
                const qint64 anchorDelta = logicalSequence >= runtimeTraceHostTimeAnchorSequence ?
                            static_cast<qint64>(logicalSequence -
                                                runtimeTraceHostTimeAnchorSequence) :
                            -static_cast<qint64>(runtimeTraceHostTimeAnchorSequence -
                                                 logicalSequence);
                frameWallClockUs = runtimeTraceHostTimeAnchorWallClockUs +
                        anchorDelta * runtimeTraceEthercatBusCycleUs;
                frameMonotonicUs = runtimeTraceHostTimeAnchorMonotonicUs +
                        anchorDelta * runtimeTraceEthercatBusCycleUs;
            }
            if(decodeOnlyNewestFrameForEndpointRemote){
                const unsigned char* frameData = buffer.data() + frameOffset;
                newestOnlineVelocityFrame.assign(frameData,
                                                 frameData + frameBytes);
                newestOnlineVelocityFrameWallClockUs = frameWallClockUs;
                newestOnlineVelocityFrameMonotonicUs = frameMonotonicUs;
                newestOnlineVelocityFrameSequence = frameSequence;
                newestOnlineVelocityFrameSequenceValid = frameSequenceValid;
                continue;
            }
            runtimeTraceLastFrameWallClockUs = frameWallClockUs;
            runtimeTraceLastFrameMonotonicUs = frameMonotonicUs;
            runtimeTraceLastFrameSequence = frameSequence;
            runtimeTraceLastFrameSequenceValid = frameSequenceValid;
            std::vector<qint64> frameCommandRawPulse(motorIdVec.size(), 0);
            std::vector<qint64> frameFeedbackRawPulse(motorIdVec.size(), 0);
            std::vector<bool> frameCommandRawPulseValid(motorIdVec.size(), false);
            std::vector<bool> frameFeedbackRawPulseValid(motorIdVec.size(), false);

            int objectOffset = objectValueStartOffset;
            for(const RuntimeTraceObject& object : runtimeTraceObjects){
                const int valueBytes = std::max(1, object.valueBytes);
                if(objectOffset + valueBytes > frameBytes ||
                        frameOffset + objectOffset + valueBytes > actualReadLength){
                    return runtimeTraceEverRead || totalCompleteFrameCount > 0 ? totalCompleteFrameCount : -1;
                }
                const unsigned char* raw = buffer.data() + frameOffset + objectOffset;
                const long rawValue = readSignedLittleEndianTraceValue(raw, valueBytes);
                if(object.kind == RuntimeTraceObjectKind::MotorCommandPosition){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(motorCommandPositionTraceObjects.size())){
                        const int logicalAxis =
                                motorCommandPositionTraceObjects[object.objectIndex].logicalAxis;
                        applyMotorCommandPositionTraceRawValue(
                                    logicalAxis,
                                    rawValue);
                        if(logicalAxis >= 0 &&
                                logicalAxis < static_cast<int>(frameCommandRawPulse.size())){
                            frameCommandRawPulse[logicalAxis] = static_cast<qint64>(rawValue);
                            frameCommandRawPulseValid[logicalAxis] = true;
                        }
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::MotorPosition){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(motorPositionTraceObjects.size())){
                        const int logicalAxis =
                                motorPositionTraceObjects[object.objectIndex].logicalAxis;
                        applyMotorPositionTraceRawValue(logicalAxis,
                                                        rawValue);
                        if(logicalAxis >= 0 &&
                                logicalAxis < static_cast<int>(frameFeedbackRawPulse.size())){
                            frameFeedbackRawPulse[logicalAxis] = static_cast<qint64>(rawValue);
                            frameFeedbackRawPulseValid[logicalAxis] = true;
                        }
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::MotorCommandVelocity){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(motorCommandVelocityTraceObjects.size())){
                        const int logicalAxis =
                                motorCommandVelocityTraceObjects[object.objectIndex].logicalAxis;
                        applyMotorCommandVelocityTraceRawValue(logicalAxis, rawValue);
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::MotorActualVelocity){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(motorActualVelocityTraceObjects.size())){
                        const int logicalAxis =
                                motorActualVelocityTraceObjects[object.objectIndex].logicalAxis;
                        applyMotorActualVelocityTraceRawValue(logicalAxis, rawValue);
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::MotorStatusWord){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(motorStatusWordTraceObjects.size())){
                        const int logicalAxis =
                                motorStatusWordTraceObjects[object.objectIndex].logicalAxis;
                        if(logicalAxis >= 0 &&
                                logicalAxis < static_cast<int>(motorTraceStatusWord.size()) &&
                                valueBytes == kLeadshineTraceStatusWordDataBytes){
                            motorTraceStatusWord[logicalAxis] =
                                    readUnsignedLittleEndianTraceWord(raw);
                            motorTraceStatusWordValid[logicalAxis] = true;
                            motorTraceStatusWordMonotonicUs[logicalAxis] =
                                    frameMonotonicUs;
                        }
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::MotorTorque){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(motorTorqueTraceObjects.size())){
                        const int logicalAxis =
                                motorTorqueTraceObjects[object.objectIndex].logicalAxis;
                        applyMotorTorqueTraceRawValue(logicalAxis,
                                                      rawValue,
                                                      frameMonotonicUs);
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::ForceSensor){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(forceSensorTraceObjects.size())){
                        applyForceSensorRawValue(forceSensorTraceObjects[object.objectIndex].sensorIndex,
                                                 rawValue,
                                                 frameMonotonicUs);
                    }
                }
                objectOffset += valueBytes;
            }

            if(frameSequenceValid){
                observePvtTraceStartDelayFrame(frameSequence,
                                               frameCommandRawPulse,
                                               frameCommandRawPulseValid,
                                               frameFeedbackRawPulse,
                                               frameFeedbackRawPulseValid);
            }
            recordMotorTraceFeedbackRawSample(frameWallClockUs,
                                              frameMonotonicUs,
                                              frameSequence,
                                              frameSequenceValid,
                                              frameFeedbackRawPulse,
                                              frameFeedbackRawPulseValid);

            if(!latestOnly && !motorCommandPositionTraceObjects.empty() && !motorPositionTraceObjects.empty()){
                MotorTracePositionWindowFrame windowFrame;
                const double nan = std::numeric_limits<double>::quiet_NaN();
                windowFrame.wallClockUs = frameWallClockUs;
                windowFrame.monotonicUs = frameMonotonicUs;
                windowFrame.commandUnitPosition.assign(motorIdVec.size(), nan);
                windowFrame.feedbackUnitPosition.assign(motorIdVec.size(), nan);
                windowFrame.commandRelativePosition.assign(motorIdVec.size(), nan);
                windowFrame.feedbackRelativePosition.assign(motorIdVec.size(), nan);
                windowFrame.commandRawPulse.assign(motorIdVec.size(), 0);
                windowFrame.feedbackRawPulse.assign(motorIdVec.size(), 0);
                windowFrame.commandValid.assign(motorIdVec.size(), false);
                windowFrame.feedbackValid.assign(motorIdVec.size(), false);
                bool hasWindowSample = false;
                for(int axis = 0; axis < static_cast<int>(motorIdVec.size()); ++axis){
                    if(axis < static_cast<int>(frameCommandRawPulseValid.size()) &&
                            frameCommandRawPulseValid[axis]){
                        windowFrame.commandRawPulse[axis] = frameCommandRawPulse[axis];
                        windowFrame.commandUnitPosition[axis] =
                                tracePulseToMotorUnit(axis, frameCommandRawPulse[axis]);
                        windowFrame.commandValid[axis] = true;
                    }
                    if(axis < static_cast<int>(frameFeedbackRawPulseValid.size()) &&
                            frameFeedbackRawPulseValid[axis]){
                        windowFrame.feedbackRawPulse[axis] = frameFeedbackRawPulse[axis];
                        windowFrame.feedbackUnitPosition[axis] =
                                tracePulseToMotorUnit(axis, frameFeedbackRawPulse[axis]);
                        windowFrame.feedbackValid[axis] = true;
                    }
                }

                for(const MotorPositionTraceObject& object : motorPositionTraceObjects){
                    const int logicalAxis = object.logicalAxis;
                    if(logicalAxis < 0 ||
                            logicalAxis >= static_cast<int>(motorComType.size()) ||
                            logicalAxis >= static_cast<int>(motorCommandPos.size()) ||
                            logicalAxis >= static_cast<int>(motorTraceActualPos.size()) ||
                            logicalAxis >= static_cast<int>(motorTracePositionSampleQueues.size()) ||
                            motorComType[logicalAxis] != COM_EC_LS ||
                            !std::isfinite(motorCommandPos[logicalAxis]) ||
                            !std::isfinite(motorTraceActualPos[logicalAxis])){
                        continue;
                    }
                    if(!ensureMotorTracePositionOffsets(logicalAxis)){
                        continue;
                    }

                    MotorTracePositionSample sample;
                    sample.commandRelativePosition =
                            traceAlignedRelativePosition(logicalAxis,
                                                         motorCommandPos[logicalAxis],
                                                         motorCommandTraceOffsetUnit,
                                                         motorCommandTraceOffsetValid);
                    sample.feedbackRelativePosition =
                            traceAlignedRelativePosition(logicalAxis,
                                                         motorTraceActualPos[logicalAxis],
                                                         motorActualTraceOffsetUnit,
                                                         motorActualTraceOffsetValid);
                    sample.wallClockUs = frameWallClockUs;
                    sample.monotonicUs = frameMonotonicUs;
                    if(logicalAxis < static_cast<int>(frameCommandRawPulseValid.size()) &&
                            frameCommandRawPulseValid[logicalAxis]){
                        sample.commandRawPulse = frameCommandRawPulse[logicalAxis];
                        sample.commandRawPulseValid = true;
                    }
                    if(logicalAxis < static_cast<int>(frameFeedbackRawPulseValid.size()) &&
                            frameFeedbackRawPulseValid[logicalAxis]){
                        sample.feedbackRawPulse = frameFeedbackRawPulse[logicalAxis];
                        sample.feedbackRawPulseValid = true;
                    }
                    auto& samples = motorTracePositionSampleQueues[logicalAxis];
                    samples.push_back(sample);
                    while(samples.size() > kMaxMotorTracePositionSamplesPerAxis){
                        samples.pop_front();
                    }
                    windowFrame.commandRelativePosition[logicalAxis] =
                            sample.commandRelativePosition;
                    windowFrame.feedbackRelativePosition[logicalAxis] =
                            sample.feedbackRelativePosition;
                    hasWindowSample = true;
                }
                if(hasWindowSample){
                    latestMotorTracePositionFrame = windowFrame;
                    latestMotorTracePositionFrameValid = true;
                    const bool shouldAppendRecoveryFrame =
                            motorTracePositionWindowRecordingEnabled ||
                            (!motorTracePositionWindowFrozenAfterPvt && !hasActivePvtTrajectory);
                    if(shouldAppendRecoveryFrame){
                        appendMotorTracePositionWindowFrame(windowFrame);
                    }
                    if(motorTracePositionWindowRecordingEnabled && activePvtAxesDoneDirect()){
                        motorTracePositionWindowRecordingEnabled = false;
                        motorTracePositionWindowFrozenAfterPvt = true;
                        pvtTraceStartDelayState.active = false;
                    }
                }
            }
            if(!latestOnly && !forceSensorTraceObjects.empty()){
                ForceSensorTraceSample sample;
                sample.values = currentForceSensorCachedValues();
                sample.wallClockUs = frameWallClockUs;
                sample.monotonicUs = frameMonotonicUs;
                sample.frameSequence = frameSequence;
                sample.frameSequenceValid = frameSequenceValid;
                if(!sample.values.empty()){
                    forceSensorTraceSampleQueue.push_back(std::move(sample));
                    while(forceSensorTraceSampleQueue.size() > kMaxForceSensorTraceSamples){
                        forceSensorTraceSampleQueue.pop_front();
                    }
                }
            }
        }

        totalCompleteFrameCount += latestOnly ? 1 : completeFrameCount;
        if(latestOnly){
            break;
        }

        if(!nextStateValid || batchFifoCaughtUp || nextValidNum <= 0){
            break;
        }
        // 若首批SDK读取发生短读但仍留有真正旧帧，至少允许一次补充读取。
        // 这不是放宽反馈年龄；最终快照仍必须通过独立的5 ms年龄上限。
        const int completedDrainReadCount = drainIndex + 1;
        if(completedDrainReadCount >= kRuntimeTraceMinimumBacklogDrainReads &&
                monotonicNowUs() - drainStartMonotonicUs >=
                    kRuntimeTraceDrainBudgetUs){
            runtimeTraceFifoCaughtUp = false;
            break;
        }
        currentValidNum = nextValidNum;
    }

    if(totalCompleteFrameCount <= 0){
        return runtimeTraceEverRead ? 0 : -1;
    }
    if(decodeOnlyNewestFrameForEndpointRemote){
        if(newestOnlineVelocityFrame.size() !=
                static_cast<std::size_t>(frameBytes) ||
                !decodeRuntimeTraceFrame(
                    newestOnlineVelocityFrame.data(),
                    frameBytes,
                    objectValueStartOffset,
                    newestOnlineVelocityFrameWallClockUs,
                    newestOnlineVelocityFrameMonotonicUs,
                    newestOnlineVelocityFrameSequence,
                    newestOnlineVelocityFrameSequenceValid,
                    true,
                    false)){
            runtimeTraceTimingReliable = false;
            runtimeTraceFifoCaughtUp = false;
            runtimeTraceHostTimeAnchorValid = false;
            return runtimeTraceEverRead ? 0 : -1;
        }
    }
    runtimeTraceEverRead = true;
    return totalCompleteFrameCount;
}

std::vector<double> HardwareInterface::readMotorPositionsTraceCached(const std::vector<int>& motorIndex)
{
    if(motorIndex.empty()){
        return {};
    }
    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    if(motorTraceActualPos.size() != motorIdVec.size()){
        motorTraceActualPos.assign(motorIdVec.size(), 0.0);
    }
    for(const int axis : motorIndex){
        if(axis < 0 ||
                axis >= static_cast<int>(motorComType.size()) ||
                motorComType[axis] != COM_EC_LS ||
                resolveLeadshineAxisIndex(axis) < 0){
            return {};
        }
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return {};
    }
    for(const int axis : motorIndex){
        if(!ensureMotorTracePositionOffsets(axis)){
            return {};
        }
    }
    return currentMotorPositionCachedValues(motorIndex);
}

bool HardwareInterface::readMotorPositionTraceCached(int logicalIndex, double& position)
{
    const std::vector<double> values = readMotorPositionsTraceCached({logicalIndex});
    if(values.size() != 1){
        return false;
    }
    position = values.front();
    return true;
}

HardwareInterface::RuntimeTraceSnapshot HardwareInterface::readRuntimeTraceLatestSnapshot()
{
    const bool captureAttributionTiming =
            motorEnableQueryTimingEnabled.load(std::memory_order_acquire);
    const qint64 requestStartUs = captureAttributionTiming ? monotonicNowUs() : 0;
    qint64 hardwareEntryUs = 0;
    qint64 hardwareExitUs = 0;
    qint64 dataApiDurationUs = 0;
    RuntimeTraceSnapshot result = runOnHardwareThread([&]() -> RuntimeTraceSnapshot {
    if(captureAttributionTiming){
        hardwareEntryUs = monotonicNowUs();
    }
    RuntimeTraceSnapshot snapshot;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    snapshot.motorPosition.assign(motorIdVec.size(), nan);
    snapshot.motorSafetyRelativePosition.assign(motorIdVec.size(), nan);
    snapshot.motorSafetyRelativePositionSource.assign(
                motorIdVec.size(),
                MotorSafetyRelativePositionSource::Invalid);
    snapshot.motorStatusWord.assign(motorIdVec.size(), 0);
    snapshot.motorStateMachine.assign(motorIdVec.size(), -1);
    snapshot.motorCommandVelocity.assign(motorIdVec.size(), nan);
    snapshot.motorActualVelocity.assign(motorIdVec.size(), nan);
    snapshot.motorTorqueNm.assign(motorIdVec.size(), nan);
    snapshot.usageProfile = activeRuntimeTraceUsageProfile;
    snapshot.usageProfileGeneration = runtimeTraceUsageProfileGeneration;
    snapshot.configurationGeneration = runtimeTraceConfigurationGeneration;
    snapshot.endpointRemoteSessionToken =
            runtimeTraceEndpointRemoteSessionToken;
    EndpointRemoteVelocitySafetyContext& endpointRemoteSafety =
            snapshot.endpointRemoteVelocitySafety;
    endpointRemoteSafety.motorPosition.assign(motorIdVec.size(), nan);
    endpointRemoteSafety.motorSafetyRelativePosition.assign(
                motorIdVec.size(), nan);
    endpointRemoteSafety.motorSafetyRelativePositionSource.assign(
                motorIdVec.size(),
                MotorSafetyRelativePositionSource::Invalid);
    endpointRemoteSafety.motorStatusWord.assign(motorIdVec.size(), 0);
    endpointRemoteSafety.motorStateMachine.assign(motorIdVec.size(), -1);
    endpointRemoteSafety.usageProfile = activeRuntimeTraceUsageProfile;
    endpointRemoteSafety.usageProfileGeneration =
            runtimeTraceUsageProfileGeneration;
    endpointRemoteSafety.configurationGeneration =
            runtimeTraceConfigurationGeneration;
    endpointRemoteSafety.sessionToken = runtimeTraceEndpointRemoteSessionToken;
    if(!isConnectLS){
        if(captureAttributionTiming){
            hardwareExitUs = monotonicNowUs();
        }
        return snapshot;
    }

    const int frameCount = readRuntimeTraceCached(false);
    dataApiDurationUs = runtimeTraceLastDataApiDurationUs;
    if(frameCount < 0 && !runtimeTraceEverRead){
        if(captureAttributionTiming){
            hardwareExitUs = monotonicNowUs();
        }
        return snapshot;
    }

    std::vector<int> leadshineAxes;
    leadshineAxes.reserve(motorIdVec.size());
    for(int axis = 0; axis < static_cast<int>(motorIdVec.size()); ++axis){
        if(axis < static_cast<int>(motorComType.size()) &&
                motorComType[axis] == COM_EC_LS &&
                resolveLeadshineAxisIndex(axis) >= 0){
            const int hardwareAxis = resolveLeadshineAxisIndex(axis);
            if(activeRuntimeTraceConfigType == RuntimeTraceConfigType::G302 &&
                    activeLiteRuntimeTraceTopology ==
                        LiteRuntimeTraceTopology::TemporarySevenAxisSensorSlave1008 &&
                    hardwareAxis == 7){
                continue;
            }
            if(runtimeTraceCommissioningAxis >= 0 &&
                    axis != runtimeTraceCommissioningAxis){
                continue;
            }
            leadshineAxes.push_back(axis);
        }
    }

    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    const auto invalidateTracePositionOffsets = [&](int axis) {
        if(axis >= 0 && axis < static_cast<int>(motorActualTraceOffsetValid.size())){
            motorActualTraceOffsetValid[axis] = false;
        }
        if(axis >= 0 && axis < static_cast<int>(motorCommandTraceOffsetValid.size())){
            motorCommandTraceOffsetValid[axis] = false;
        }
    };
    const auto warnRejectedTracePosition =
            [&](int axis,
                double tracePosition,
                double directPosition,
                double traceRelativePosition,
                double directRelativePosition,
                double minPosition,
                double maxPosition) {
        const qint64 nowUs = monotonicNowUs();
        if(runtimeTracePositionRejectLastWarnUs > 0 &&
                nowUs - runtimeTracePositionRejectLastWarnUs <
                kRuntimeTracePositionRejectWarnIntervalUs){
            return;
        }
        runtimeTracePositionRejectLastWarnUs = nowUs;
        const qint64 frameDeltaUs = latestMotorTracePositionFrame.monotonicUs > 0 ?
                    nowUs - latestMotorTracePositionFrame.monotonicUs :
                    std::numeric_limits<qint64>::min();
        emit displayInfoSignal(
                    QString("Runtime Trace motor position rejected: %1 trace=%2 direct=%3 rel_trace=%4 rel_direct=%5 limit=[%6, %7] frame_delta_us=%8 newest_age_us=%9 fifo_valid=%10 caught_up=%11 timing_reliable=%12 trace_lost=%13.")
                    .arg(axisDisplayName(axis))
                    .arg(tracePosition, 0, 'f', 6)
                    .arg(directPosition, 0, 'f', 6)
                    .arg(traceRelativePosition, 0, 'f', 6)
                    .arg(directRelativePosition, 0, 'f', 6)
                    .arg(minPosition, 0, 'f', 6)
                    .arg(maxPosition, 0, 'f', 6)
                    .arg(frameDeltaUs)
                    .arg(runtimeTraceNewestFrameAgeUs)
                    .arg(runtimeTraceLastFifoValidNum)
                    .arg(runtimeTraceFifoCaughtUp ? 1 : 0)
                    .arg(runtimeTraceTimingReliable ? 1 : 0)
                    .arg(runtimeTraceLost ? 1 : 0)
                    .toStdString(),
                    "warning");
    };
    for(const int axis : leadshineAxes){
        if(axis < 0 || axis >= static_cast<int>(snapshot.motorPosition.size())){
            continue;
        }
        if(!ensureMotorTracePositionOffsets(axis)){
            continue;
        }

        const std::vector<double> tracePositions =
                currentMotorPositionCachedValues(std::vector<int>{axis});
        if(tracePositions.size() != 1){
            continue;
        }

        const double position = tracePositions.front();
        bool acceptPosition = std::isfinite(position);
        bool rejectedByDirectCheck = false;
        double relativePosition = nan;
        bool relativePositionOk = false;
        double directPosition = nan;
        double directRelativePosition = nan;
        double minPosition = nan;
        double maxPosition = nan;

        MotorSafetyRelativePositionSource traceSafetySource =
                MotorSafetyRelativePositionSource::Invalid;
        const bool exactLatestPositionFrame =
                latestMotorTracePositionFrameValid &&
                latestMotorTracePositionFrame.monotonicUs > 0 &&
                latestMotorTracePositionFrame.monotonicUs ==
                    runtimeTraceLastFrameMonotonicUs;
        relativePositionOk = exactLatestPositionFrame &&
                motorSafetyRelativeFromTraceFrame(axis,
                                                  latestMotorTracePositionFrame,
                                                  relativePosition,
                                                  &traceSafetySource);
        if(relativePositionOk &&
                axis < static_cast<int>(
                    endpointRemoteSafety.motorSafetyRelativePosition.size())){
            endpointRemoteSafety.motorSafetyRelativePosition[axis] =
                    relativePosition;
            endpointRemoteSafety.motorSafetyRelativePositionSource[axis] =
                    traceSafetySource;
        }
        const bool sessionTraceHome =
                hasValidMotorSessionSafetyTraceHome(axis);
        if(!relativePositionOk && !sessionTraceHome &&
                hasValidMotorSafetyEncoderHome(axis)){
            double encoderPosition = 0.0;
            if(readMotorEncoderUnitDirect(axis, encoderPosition)){
                relativePosition = encoderPosition - motorSafetyHomeEncoderUnit[axis];
                relativePositionOk = std::isfinite(relativePosition);
                if(relativePositionOk){
                    traceSafetySource =
                            MotorSafetyRelativePositionSource::EncoderFallback;
                }
            }
        }
        if(!relativePositionOk && !sessionTraceHome){
            relativePosition = relativeMotorPosition(axis, position);
            relativePositionOk = std::isfinite(relativePosition);
            if(relativePositionOk){
                traceSafetySource =
                        MotorSafetyRelativePositionSource::PositionFallback;
            }
        }
        if(relativePositionOk &&
                axis < static_cast<int>(snapshot.motorSafetyRelativePosition.size())){
            snapshot.motorSafetyRelativePosition[axis] = relativePosition;
            snapshot.motorSafetyRelativePositionSource[axis] =
                    traceSafetySource;
        }

        if(acceptPosition && hasValidMotorSoftwareLimit(axis)){
            minPosition = motorSoftwareMinPos[axis];
            maxPosition = motorSoftwareMaxPos[axis];
            const bool traceOutsideLimit = relativePositionOk &&
                    (relativePosition < minPosition ||
                     relativePosition > maxPosition);
            if(traceOutsideLimit){
                const bool directReadOk =
                        readMotorPositionUnitDirect(axis, directPosition, false) &&
                        std::isfinite(directPosition);
                if(directReadOk){
                    if(hasValidMotorSafetyEncoderHome(axis)){
                        double encoderPosition = 0.0;
                        if(readMotorEncoderUnitDirect(axis, encoderPosition)){
                            directRelativePosition =
                                    encoderPosition - motorSafetyHomeEncoderUnit[axis];
                        }
                    }
                    if(!std::isfinite(directRelativePosition)){
                        directRelativePosition = relativeMotorPosition(axis, directPosition);
                    }
                    const bool directInsideOrNearLimit =
                            std::isfinite(directRelativePosition) &&
                            directRelativePosition >=
                            minPosition - kRuntimeTracePositionLimitGuardUnit &&
                            directRelativePosition <=
                            maxPosition + kRuntimeTracePositionLimitGuardUnit;
                    if(directInsideOrNearLimit){
                        acceptPosition = false;
                        rejectedByDirectCheck = true;
                    }
                }
                else{
                    acceptPosition = false;
                    rejectedByDirectCheck = true;
                }
            }
        }

        if(!acceptPosition){
            invalidateTracePositionOffsets(axis);
            if(rejectedByDirectCheck){
                warnRejectedTracePosition(axis,
                                          position,
                                          directPosition,
                                          relativePosition,
                                          directRelativePosition,
                                          minPosition,
                                          maxPosition);
            }
            continue;
        }

        snapshot.motorPosition[axis] = position;
        endpointRemoteSafety.motorPosition[axis] = position;
        if(axis < static_cast<int>(motorCurPos.size())){
            motorCurPos[axis] = position;
        }
    }

    snapshot.motorTorqueNm = currentMotorTorqueTraceCachedValues();
    const int commandVelocityCount = std::min(
                static_cast<int>(snapshot.motorCommandVelocity.size()),
                static_cast<int>(motorTraceCommandVelocity.size()));
    for(int axis = 0; axis < commandVelocityCount; ++axis){
        if(axis < static_cast<int>(motorTraceCommandVelocityValid.size()) &&
                motorTraceCommandVelocityValid[axis]){
            snapshot.motorCommandVelocity[axis] = motorTraceCommandVelocity[axis];
        }
    }
    const int actualVelocityCount = std::min(
                static_cast<int>(snapshot.motorActualVelocity.size()),
                static_cast<int>(motorTraceActualVelocity.size()));
    for(int axis = 0; axis < actualVelocityCount; ++axis){
        if(axis < static_cast<int>(motorTraceActualVelocityValid.size()) &&
                motorTraceActualVelocityValid[axis]){
            snapshot.motorActualVelocity[axis] = motorTraceActualVelocity[axis];
        }
    }
    const int statusWordCount = std::min(
                static_cast<int>(snapshot.motorStatusWord.size()),
                static_cast<int>(motorTraceStatusWord.size()));
    for(int axis = 0; axis < statusWordCount; ++axis){
        const bool sameLatestFrame =
                runtimeTraceLastFrameMonotonicUs > 0 &&
                axis < static_cast<int>(motorTraceStatusWordValid.size()) &&
                motorTraceStatusWordValid[axis] &&
                axis < static_cast<int>(motorTraceStatusWordMonotonicUs.size()) &&
                motorTraceStatusWordMonotonicUs[axis] ==
                    runtimeTraceLastFrameMonotonicUs;
        if(!sameLatestFrame){
            continue;
        }
        const quint16 statusWord = motorTraceStatusWord[axis];
        snapshot.motorStatusWord[axis] = statusWord;
        snapshot.motorStateMachine[axis] =
                decodeCia402StateMachine(statusWord);
        endpointRemoteSafety.motorStatusWord[axis] = statusWord;
        endpointRemoteSafety.motorStateMachine[axis] =
                snapshot.motorStateMachine[axis];
    }
    snapshot.forceSensorValue = currentForceSensorCachedValues();
    snapshot.forceSensorFrameMonotonicUs = forceSensorTraceValueMonotonicUs;
    snapshot.wallClockUs = runtimeTraceLastFrameWallClockUs;
    snapshot.monotonicUs = runtimeTraceLastFrameMonotonicUs;
    snapshot.newestFrameAgeUs = runtimeTraceNewestFrameAgeUs;
    if(snapshot.monotonicUs > 0){
        const qint64 snapshotNowUs = monotonicNowUs();
        if(snapshotNowUs >= snapshot.monotonicUs){
            snapshot.newestFrameAgeUs = std::max(
                        snapshot.newestFrameAgeUs,
                        snapshotNowUs - snapshot.monotonicUs);
        }
    }
    snapshot.frameCount = std::max(0, frameCount);
    snapshot.fifoValidNum = runtimeTraceLastFifoValidNum;
    snapshot.fifoFreeNum = runtimeTraceLastFifoFreeNum;
    snapshot.traceSamplePeriodUs = runtimeTraceSamplePeriodUs;
    snapshot.frameSequence = runtimeTraceLastFrameSequence;
    snapshot.logicalFrameSequence = runtimeTraceLastLogicalSequence;
    snapshot.fromTrace = runtimeTraceEverRead;
    snapshot.frameSequenceValid = runtimeTraceLastFrameSequenceValid;
    snapshot.timingReliable = runtimeTraceTimingReliable &&
            runtimeTraceConfigReadbackValid &&
            runtimeTraceHostTimeAnchorValid;
    snapshot.fifoCaughtUp = runtimeTraceFifoCaughtUp;
    snapshot.traceLost = runtimeTraceLost;
    endpointRemoteSafety.monotonicUs = snapshot.monotonicUs;
    endpointRemoteSafety.newestFrameAgeUs = snapshot.newestFrameAgeUs;
    endpointRemoteSafety.fifoValidNum = snapshot.fifoValidNum;
    endpointRemoteSafety.fifoFreeNum = snapshot.fifoFreeNum;
    endpointRemoteSafety.traceSamplePeriodUs = snapshot.traceSamplePeriodUs;
    endpointRemoteSafety.logicalFrameSequence =
            snapshot.logicalFrameSequence;
    endpointRemoteSafety.fromTrace = snapshot.fromTrace;
    endpointRemoteSafety.frameSequenceValid = snapshot.frameSequenceValid;
    endpointRemoteSafety.timingReliable = snapshot.timingReliable;
    endpointRemoteSafety.fifoCaughtUp = snapshot.fifoCaughtUp;
    endpointRemoteSafety.traceLost = snapshot.traceLost;
    endpointRemoteSafety.statusFaultLatched =
            endpointRemoteTraceStatusFaultLatched;
    endpointRemoteSafety.statusFaultAxis =
            endpointRemoteTraceStatusFaultAxis;
    endpointRemoteSafety.statusFaultWord =
            endpointRemoteTraceStatusFaultWord;
    endpointRemoteSafety.statusFaultStateMachine =
            endpointRemoteTraceStatusFaultStateMachine;
    endpointRemoteSafety.statusFaultLogicalFrameSequence =
            endpointRemoteTraceStatusFaultLogicalFrameSequence;
    // readRuntimeTraceCached()可能在本次调用内完成首次配置并推进generation，
    // 因此在返回前以HardwareThread内的最终profile状态覆盖入口初值。
    snapshot.usageProfile = activeRuntimeTraceUsageProfile;
    snapshot.usageProfileGeneration = runtimeTraceUsageProfileGeneration;
    snapshot.configurationGeneration = runtimeTraceConfigurationGeneration;
    snapshot.endpointRemoteSessionToken =
            runtimeTraceEndpointRemoteSessionToken;
    endpointRemoteSafety.usageProfile = activeRuntimeTraceUsageProfile;
    endpointRemoteSafety.usageProfileGeneration =
            runtimeTraceUsageProfileGeneration;
    endpointRemoteSafety.configurationGeneration =
            runtimeTraceConfigurationGeneration;
    endpointRemoteSafety.sessionToken = runtimeTraceEndpointRemoteSessionToken;
    snapshot.forceSensorTraceSamples.reserve(forceSensorTraceSampleQueue.size());
    while(!forceSensorTraceSampleQueue.empty()){
        snapshot.forceSensorTraceSamples.push_back(std::move(forceSensorTraceSampleQueue.front()));
        forceSensorTraceSampleQueue.pop_front();
    }
    if(captureAttributionTiming){
        hardwareExitUs = monotonicNowUs();
    }
    return snapshot;
    });
    if(captureAttributionTiming){
        const qint64 callCompleteUs = monotonicNowUs();
        result.hardwareThreadQueueWaitUs = hardwareEntryUs > requestStartUs ?
                    hardwareEntryUs - requestStartUs : 0;
        result.hardwareThreadExecutionUs = hardwareExitUs > hardwareEntryUs ?
                    hardwareExitUs - hardwareEntryUs : 0;
        result.dataApiDurationUs = std::max<qint64>(0, dataApiDurationUs);
        result.totalReadCallUs = std::max<qint64>(
                    0, callCompleteUs - requestStartUs);
    }
    return result;
}

void HardwareInterface::applyForceSensorRawValue(int sensorIndex, long rawValue, qint64 traceMonotonicUs)
{
    if(sensorIndex < 0 || sensorIndex >= static_cast<int>(sensorComType.size())){
        return;
    }
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
        forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
        nextForceSensorPollIndex = 0;
    }
    if(forceSensorTraceValueMonotonicUs.size() != sensorComType.size()){
        forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
    }

    const double currentRaw = convertRawToSigned(rawValue, forceSensorIsSigned);
    const double homeValue = sensorIndex < static_cast<int>(sensorHomeValue.size()) ?
                sensorHomeValue[sensorIndex] :
                0.0;
    const double raw2DataCof = sensorIndex < static_cast<int>(sensorRaw2DataCof.size()) ?
                sensorRaw2DataCof[sensorIndex] :
                1.0;
    forceSensorCachedValue[sensorIndex] = (currentRaw - homeValue) * raw2DataCof;
    forceSensorCacheValid[sensorIndex] = true;
    if(traceMonotonicUs > 0){
        forceSensorTraceValueMonotonicUs[sensorIndex] = traceMonotonicUs;
    }
}

std::vector<double> HardwareInterface::currentForceSensorCachedValues() const
{
    std::vector<double> value;
    value.reserve(sensorComType.size());
    for(int i = 0; i < static_cast<int>(sensorComType.size()); ++i){
        if(sensorComType[i] != COM_EC_LS_SBT){
            continue;
        }
        value.push_back(i < static_cast<int>(forceSensorCacheValid.size()) &&
                        forceSensorCacheValid[i] ?
                            forceSensorCachedValue[i] :
                            0.0);
    }
    return value;
}

short HardwareInterface::readForceSensorTxpdoExtra(int sensorIndex, int* rawValue) const
{
    if(sensorIndex < 0 || !rawValue){
        return -1;
    }

    int port = sensorIndex < static_cast<int>(sensorPort.size()) ?
                sensorPort[sensorIndex] :
                2;
    int address = sensorIndex < static_cast<int>(sensorAdr.size()) ?
                sensorAdr[sensorIndex] :
                sensorIndex * 2;
    int dataLen = sensorIndex < static_cast<int>(sensorDataLen.size()) ?
                sensorDataLen[sensorIndex] :
                2;

    if(port < 0){
        port = 2;
    }
    if(address < 0){
        address = sensorIndex * 2;
    }
    if(dataLen <= 0){
        dataLen = 2;
    }

    return nmc_read_txpdo_extra(0,
                                static_cast<WORD>(port),
                                static_cast<WORD>(address),
                                static_cast<WORD>(dataLen),
                                rawValue);
}

std::vector<double> HardwareInterface::readForceSensorDataCachedDirect(int maxChannelsToRead)
{
    return readForceSensorDataCachedDirectResult(maxChannelsToRead).values;
}

HardwareInterface::ForceSensorReadResult HardwareInterface::readForceSensorDataCachedDirectResult(int maxChannelsToRead)
{
    if(!isConnectLS){
        return {};
    }

    const int sensorCount = static_cast<int>(sensorComType.size());
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
        forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
        nextForceSensorPollIndex = 0;
    }

    const bool hasAnyCachedValue = std::any_of(forceSensorCacheValid.begin(),
                                               forceSensorCacheValid.end(),
                                               [](bool valid){ return valid; });
    const int effectiveMaxChannelsToRead = hasAnyCachedValue ? maxChannelsToRead : sensorCount;
    const int readLimit = effectiveMaxChannelsToRead <= 0 ?
                sensorCount :
                std::min(effectiveMaxChannelsToRead, sensorCount);
    int readCount = 0;
    int visitedCount = 0;
    while(sensorCount > 0 && readCount < readLimit && visitedCount < sensorCount){
        const int sensorIndex = nextForceSensorPollIndex % sensorCount;
        nextForceSensorPollIndex = (sensorIndex + 1) % sensorCount;
        ++visitedCount;

        if(sensorComType[sensorIndex] != COM_EC_LS_SBT){
            continue;
        }

        int tmpValue = 0;
        const short readRet = readForceSensorTxpdoExtra(sensorIndex, &tmpValue);
        recordCommunicationEvent(false, QStringLiteral("nmc_read_txpdo_extra"));
        ++readCount;
        if(readRet != 0){
            continue;
        }

        applyForceSensorRawValue(sensorIndex, static_cast<long>(tmpValue));
    }

    ForceSensorReadResult result;
    result.values = currentForceSensorCachedValues();
    result.frameCount = result.values.empty() ? 0 : 1;
    result.fromTrace = false;
    return result;
}

bool HardwareInterface::configureForceSensorTraceRead()
{
    if(!runtimeTraceUsageProfileIncludesForceSensors(
                activeRuntimeTraceUsageProfile)){
        return false;
    }
    return configureRuntimeTraceRead();
}

std::vector<double> HardwareInterface::readForceSensorDataTraceCached()
{
    return readForceSensorDataTraceCachedResult().values;
}

HardwareInterface::ForceSensorReadResult HardwareInterface::readForceSensorDataTraceCachedResult()
{
    if(!runtimeTraceUsageProfileIncludesForceSensors(
                activeRuntimeTraceUsageProfile)){
        return {};
    }
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
        forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
        nextForceSensorPollIndex = 0;
    }
    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return {};
    }
    ForceSensorReadResult result;
    result.values = currentForceSensorCachedValues();
    result.frameCount = std::max(0, frameCount);
    result.fromTrace = true;
    return result;
}

std::vector<double> HardwareInterface::readForceSensorDataSnapshot() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    if(!isConnectLS){
        return std::vector<double>();
    }
    std::vector<double> value;
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
        forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
        nextForceSensorPollIndex = 0;
    }
    for (int i = 0; i < static_cast<int>(sensorComType.size()); ++i) {
        if (sensorComType[i] == COM_EC_LS_SBT) {
            int tmpValue = 0;
            const short readRet = readForceSensorTxpdoExtra(i, &tmpValue);
            recordCommunicationEvent(false, QStringLiteral("nmc_read_txpdo_extra"));
            if(readRet == 0){
                applyForceSensorRawValue(i, static_cast<long>(tmpValue));
            }
            value.push_back(i < static_cast<int>(forceSensorCacheValid.size()) &&
                            forceSensorCacheValid[i] ?
                                forceSensorCachedValue[i] :
                                0.0);
        }
    }
    return value;
    });
}

std::vector<double> HardwareInterface::readForceSensorDataCached(int maxChannelsToRead) {
    return readForceSensorDataCachedResult(maxChannelsToRead).values;
}

std::vector<HardwareInterface::ForceSensorTraceSample>
HardwareInterface::readForceSensorDataTraceSamples()
{
    return runOnHardwareThread([&]() -> std::vector<ForceSensorTraceSample> {
    if(!runtimeTraceUsageProfileIncludesForceSensors(
                activeRuntimeTraceUsageProfile)){
        return {};
    }
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
        forceSensorTraceValueMonotonicUs.assign(sensorComType.size(), 0);
        nextForceSensorPollIndex = 0;
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return {};
    }

    std::vector<ForceSensorTraceSample> result;
    result.reserve(forceSensorTraceSampleQueue.size());
    while(!forceSensorTraceSampleQueue.empty()){
        result.push_back(std::move(forceSensorTraceSampleQueue.front()));
        forceSensorTraceSampleQueue.pop_front();
    }
    return result;
    });
}

HardwareInterface::ForceSensorReadResult HardwareInterface::readForceSensorDataCachedResult(int maxChannelsToRead) {
    return runOnHardwareThread([&]() -> ForceSensorReadResult {
    ForceSensorReadResult traceResult = readForceSensorDataTraceCachedResult();
    if(!traceResult.values.empty() && traceResult.frameCount > 0){
        return traceResult;
    }
    ForceSensorReadResult directResult = readForceSensorDataCachedDirectResult(maxChannelsToRead);
    if(!directResult.values.empty()){
        return directResult;
    }
    return traceResult;
    });
}

HardwareInterface::PdoTraceProbeResult HardwareInterface::runPdoTraceForceSensorProbe(const PdoTraceProbeConfig& config)
{
    const PdoTraceProbeConfig request = config;
    return runOnHardwareThread([this, request]() -> PdoTraceProbeResult {
    PdoTraceProbeResult result;
    if(!isConnectLS){
        result.messages << QStringLiteral("Leadshine controller is not connected.");
        return result;
    }
    if(request.slaveAddress == 0){
        result.messages << QStringLiteral("PDO trace slaveAddress is not configured.");
        return result;
    }
    if(request.objects.empty()){
        result.messages << QStringLiteral("PDO trace object list is empty.");
        return result;
    }
    if(request.traceLength == 0){
        result.messages << QStringLiteral("PDO traceLength must be greater than 0.");
        return result;
    }

    std::vector<WORD> indexes;
    std::vector<WORD> subIndexes;
    indexes.reserve(request.objects.size());
    subIndexes.reserve(request.objects.size());
    for(const PdoTraceObjectConfig& object : request.objects){
        indexes.push_back(object.index);
        subIndexes.push_back(object.subIndex);
    }

    result.preStopResult = nmc_stop_pdo_trace(0, request.channel, request.slaveAddress);
    recordCommunicationEvent(false, QStringLiteral("nmc_stop_pdo_trace"));
    result.clearBeforeStartResult = nmc_clear_pdo_trace_data(0, request.channel, request.slaveAddress);
    recordCommunicationEvent(false, QStringLiteral("nmc_clear_pdo_trace_data"));
    if(result.clearBeforeStartResult != 0){
        result.messages << QStringLiteral("nmc_clear_pdo_trace_data returned %1.")
                           .arg(result.clearBeforeStartResult);
    }

    result.startResult = nmc_start_pdo_trace(0,
                                             request.channel,
                                             request.slaveAddress,
                                             static_cast<WORD>(indexes.size()),
                                             request.traceLength,
                                             indexes.data(),
                                             subIndexes.data());
    recordCommunicationEvent(false, QStringLiteral("nmc_start_pdo_trace"));
    if(result.startResult != 0){
        result.messages << QStringLiteral("nmc_start_pdo_trace returned %1.")
                           .arg(result.startResult);
        return result;
    }

    const int timeoutMs = std::max(0, request.waitTimeoutMs);
    const int pollIntervalMs = std::max(1, request.pollIntervalMs);
    QElapsedTimer timer;
    timer.start();
    while(true){
        ++result.pollCount;
        result.getNumResult = nmc_get_pdo_trace_num(0,
                                                    request.channel,
                                                    request.slaveAddress,
                                                    &result.dataNum,
                                                    &result.sizeOfEachPacket);
        recordCommunicationEvent(false, QStringLiteral("nmc_get_pdo_trace_num"));
        result.stateResult = nmc_get_pdo_trace_state(0,
                                                     request.channel,
                                                     request.slaveAddress,
                                                     &result.traceState);
        recordCommunicationEvent(false, QStringLiteral("nmc_get_pdo_trace_state"));
        if(result.getNumResult == 0 && result.dataNum > 0 && result.sizeOfEachPacket > 0){
            break;
        }
        if(timer.elapsed() >= timeoutMs){
            result.timedOut = true;
            break;
        }
        QThread::msleep(static_cast<unsigned long>(pollIntervalMs));
    }

    if(result.timedOut){
        result.messages << QStringLiteral("PDO trace wait timed out after %1 ms.")
                           .arg(timeoutMs);
    }
    if(result.getNumResult != 0){
        result.messages << QStringLiteral("nmc_get_pdo_trace_num returned %1.")
                           .arg(result.getNumResult);
    }
    if(result.stateResult != 0){
        result.messages << QStringLiteral("nmc_get_pdo_trace_state returned %1.")
                           .arg(result.stateResult);
    }

    DWORD readLength = request.readLengthBytes;
    if(readLength == 0 && result.dataNum > 0 && result.sizeOfEachPacket > 0){
        const quint64 estimatedLength = static_cast<quint64>(result.dataNum) *
                static_cast<quint64>(result.sizeOfEachPacket);
        readLength = static_cast<DWORD>(std::min<quint64>(
                                            estimatedLength,
                                            static_cast<quint64>(request.maxReadLengthBytes)));
    }
    if(readLength == 0){
        readLength = std::min<DWORD>(4096, request.maxReadLengthBytes);
    }
    if(request.maxReadLengthBytes > 0){
        readLength = std::min<DWORD>(readLength, request.maxReadLengthBytes);
    }
    result.requestedReadLength = readLength;

    if(readLength > 0){
        std::vector<BYTE> buffer(static_cast<std::size_t>(readLength), 0);
        result.readResult = nmc_read_pdo_trace_data(0,
                                                    request.channel,
                                                    request.slaveAddress,
                                                    request.readStartAddress,
                                                    readLength,
                                                    &result.actualReadLength,
                                                    buffer.data());
        recordCommunicationEvent(false, QStringLiteral("nmc_read_pdo_trace_data"));
        if(result.readResult == 0 && result.actualReadLength > 0){
            const int byteCount = static_cast<int>(std::min<DWORD>(
                                                       result.actualReadLength,
                                                       readLength));
            result.data = QByteArray(reinterpret_cast<const char*>(buffer.data()), byteCount);
        }
        else if(result.readResult != 0){
            result.messages << QStringLiteral("nmc_read_pdo_trace_data returned %1.")
                               .arg(result.readResult);
        }
    }

    result.stopResult = nmc_stop_pdo_trace(0, request.channel, request.slaveAddress);
    recordCommunicationEvent(false, QStringLiteral("nmc_stop_pdo_trace"));
    if(result.stopResult != 0){
        result.messages << QStringLiteral("nmc_stop_pdo_trace returned %1.")
                           .arg(result.stopResult);
    }

    result.success = result.startResult == 0 &&
            result.getNumResult == 0 &&
            result.readResult == 0 &&
            result.actualReadLength > 0 &&
            !result.timedOut;
    return result;
    });
}

HardwareInterface::TraceProbeResult HardwareInterface::runTraceForceSensorProbe(const TraceProbeConfig& config)
{
    const TraceProbeConfig request = config;
    return runOnHardwareThread([this, request]() -> TraceProbeResult {
    TraceProbeResult result;
    if(!isConnectLS){
        result.messages << QStringLiteral("Leadshine controller is not connected.");
        return result;
    }
    if(request.objects.empty()){
        result.messages << QStringLiteral("Trace object list is empty.");
        return result;
    }

    if(request.configureSource){
        result.setSourceResult = dmc_trace_set_source(0, request.source);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_set_source"));
        if(result.setSourceResult != 0){
            result.messages << QStringLiteral("dmc_trace_set_source returned %1.")
                               .arg(result.setSourceResult);
        }
    }

    result.stopBeforeConfigResult = dmc_trace_data_stop(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_stop"));
    result.dataResetBeforeConfigResult = dmc_trace_data_reset(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_reset"));

    result.setConfigResult = dmc_trace_set_config(0,
                                                  request.traceCycle,
                                                  request.lostHandle,
                                                  request.traceType,
                                                  request.triggerObjectIndex,
                                                  request.triggerType,
                                                  request.mask,
                                                  request.condition);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_set_config"));
    if(result.setConfigResult != 0){
        result.messages << QStringLiteral("dmc_trace_set_config returned %1.")
                           .arg(result.setConfigResult);
        return result;
    }

    result.resetConfigObjectResult = dmc_trace_reset_config_object(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_reset_config_object"));
    if(result.resetConfigObjectResult != 0){
        result.messages << QStringLiteral("dmc_trace_reset_config_object returned %1.")
                           .arg(result.resetConfigObjectResult);
        return result;
    }

    for(const TraceObjectConfig& object : request.objects){
        result.addConfigObjectResult = dmc_trace_add_config_object(0,
                                                                   object.dataType,
                                                                   object.dataIndex,
                                                                   object.dataSubIndex,
                                                                   object.slaveId,
                                                                   object.dataBytes);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_add_config_object"));
        if(result.addConfigObjectResult != 0){
            result.messages << QStringLiteral("dmc_trace_add_config_object returned %1.")
                               .arg(result.addConfigObjectResult);
            return result;
        }
    }

    result.dataStartResult = dmc_trace_data_start(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_start"));
    if(result.dataStartResult != 0){
        result.messages << QStringLiteral("dmc_trace_data_start returned %1.")
                           .arg(result.dataStartResult);
        return result;
    }

    const int timeoutMs = std::max(0, request.waitTimeoutMs);
    const int pollIntervalMs = std::max(1, request.pollIntervalMs);
    QElapsedTimer timer;
    timer.start();
    while(true){
        ++result.pollCount;
        result.getFlagResult = dmc_trace_get_flag(0,
                                                  &result.startFlag,
                                                  &result.triggeredFlag,
                                                  &result.lostFlag);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_flag"));
        result.getStateResult = dmc_trace_get_state(0,
                                                    &result.validNum,
                                                    &result.freeNum,
                                                    &result.objectTotalBytes,
                                                    &result.objectTotalNum);
        recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_state"));
        if(result.getStateResult == 0 && result.validNum > 0){
            break;
        }
        if(timer.elapsed() >= timeoutMs){
            result.timedOut = true;
            break;
        }
        QThread::msleep(static_cast<unsigned long>(pollIntervalMs));
    }

    if(result.timedOut){
        result.messages << QStringLiteral("Trace wait timed out after %1 ms.")
                           .arg(timeoutMs);
    }
    if(result.getFlagResult != 0){
        result.messages << QStringLiteral("dmc_trace_get_flag returned %1.")
                           .arg(result.getFlagResult);
    }
    if(result.getStateResult != 0){
        result.messages << QStringLiteral("dmc_trace_get_state returned %1.")
                           .arg(result.getStateResult);
    }

    int readLength = std::max(1, request.bufferSizeBytes);
    if(request.maxBufferSizeBytes > 0){
        readLength = std::min(readLength, request.maxBufferSizeBytes);
    }
    std::vector<unsigned char> buffer(static_cast<std::size_t>(readLength), 0);
    result.getDataResult = dmc_trace_get_data(0,
                                              readLength,
                                              buffer.data(),
                                              &result.actualReadLength);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_get_data"));
    if(result.getDataResult == 0 && result.actualReadLength > 0){
        const int byteCount = std::min(result.actualReadLength, readLength);
        result.data = QByteArray(reinterpret_cast<const char*>(buffer.data()), byteCount);
    }
    else if(result.getDataResult != 0){
        result.messages << QStringLiteral("dmc_trace_get_data returned %1.")
                           .arg(result.getDataResult);
    }

    result.dataStopResult = dmc_trace_data_stop(0);
    recordCommunicationEvent(false, QStringLiteral("dmc_trace_data_stop"));
    if(result.dataStopResult != 0){
        result.messages << QStringLiteral("dmc_trace_data_stop returned %1.")
                           .arg(result.dataStopResult);
    }

    result.success = result.dataStartResult == 0 &&
            result.getStateResult == 0 &&
            result.getDataResult == 0 &&
            result.actualReadLength > 0 &&
            !result.timedOut;
    return result;
    });
}

HardwareInterface::DiagnosticsSnapshot HardwareInterface::diagnosticsSnapshot() const
{
    QMutexLocker locker(&diagnosticsMutex);
    return diagnostics;
}

HardwareInterface::FieldbusConsumeTimeSnapshot HardwareInterface::fieldbusConsumeTimeSnapshot() const
{
    return runOnHardwareThread([&]() -> FieldbusConsumeTimeSnapshot {
        FieldbusConsumeTimeSnapshot snapshot;
        snapshot.wallClockMs = QDateTime::currentMSecsSinceEpoch();
        snapshot.cardNo = 0;
        snapshot.portNum = kLeadshineEtherCatPort;
        if(!isConnectLS){
            snapshot.apiResult = -1;
            return snapshot;
        }

        DWORD averageTime = 0;
        DWORD maxTime = 0;
        uint64 cycles = 0;
        const short ret = nmc_get_consume_time_fieldbus(snapshot.cardNo,
                                                       snapshot.portNum,
                                                       &averageTime,
                                                       &maxTime,
                                                       &cycles);
        const_cast<HardwareInterface*>(this)->recordCommunicationEvent(
                    false,
                    QStringLiteral("nmc_get_consume_time_fieldbus"));
        snapshot.apiResult = ret;
        snapshot.averageTimeUs = averageTime;
        snapshot.maxTimeUs = maxTime;
        snapshot.cycles = static_cast<quint64>(cycles);
        snapshot.success = ret == 0;
        return snapshot;
    });
}

QVector<HardwareInterface::DiagnosticRawSample> HardwareInterface::communicationTimingHistory(qint64 startWallClockMs,
                                                                                                qint64 endWallClockMs) const
{
    QMutexLocker locker(&diagnosticsMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(communicationRawHistory.size());
    for(const DiagnosticRawSample& sample : communicationRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<HardwareInterface::DiagnosticRawSample> HardwareInterface::motorCommandTimingHistory(qint64 startWallClockMs,
                                                                                              qint64 endWallClockMs) const
{
    QMutexLocker locker(&diagnosticsMutex);
    QVector<DiagnosticRawSample> filtered;
    filtered.reserve(motorCommandRawHistory.size());
    for(const DiagnosticRawSample& sample : motorCommandRawHistory){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<HardwareInterface::MotorPositionRawSample> HardwareInterface::motorPositionRawHistory(qint64 startWallClockMs,
                                                                                              qint64 endWallClockMs) const
{
    QMutexLocker locker(&diagnosticsMutex);
    QVector<MotorPositionRawSample> filtered;
    filtered.reserve(motorPositionRawSamples.size());
    for(const MotorPositionRawSample& sample : motorPositionRawSamples){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<HardwareInterface::MotorPositionRawSample> HardwareInterface::motorEncoderRawHistory(qint64 startWallClockMs,
                                                                                             qint64 endWallClockMs) const
{
    QMutexLocker locker(&diagnosticsMutex);
    QVector<MotorPositionRawSample> filtered;
    filtered.reserve(motorEncoderRawSamples.size());
    for(const MotorPositionRawSample& sample : motorEncoderRawSamples){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<HardwareInterface::MotorTraceFeedbackRawSample> HardwareInterface::motorTraceFeedbackRawHistory(
        qint64 startWallClockMs,
        qint64 endWallClockMs) const
{
    QMutexLocker locker(&diagnosticsMutex);
    QVector<MotorTraceFeedbackRawSample> filtered;
    filtered.reserve(motorTraceFeedbackRawSamples.size());
    for(const MotorTraceFeedbackRawSample& sample : motorTraceFeedbackRawSamples){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<HardwareInterface::RuntimeTraceFetchTimingSample> HardwareInterface::runtimeTraceFetchTimingHistory(
        qint64 startWallClockMs,
        qint64 endWallClockMs) const
{
    QMutexLocker locker(&diagnosticsMutex);
    QVector<RuntimeTraceFetchTimingSample> filtered;
    filtered.reserve(runtimeTraceFetchTimingSamples.size());
    for(const RuntimeTraceFetchTimingSample& sample : runtimeTraceFetchTimingSamples){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

QVector<HardwareInterface::PvtTableUploadTimingSample> HardwareInterface::pvtTableUploadTimingHistory(
        qint64 startWallClockMs,
        qint64 endWallClockMs) const
{
    QMutexLocker locker(&diagnosticsMutex);
    QVector<PvtTableUploadTimingSample> filtered;
    filtered.reserve(pvtTableUploadTimingSamples.size());
    for(const PvtTableUploadTimingSample& sample : pvtTableUploadTimingSamples){
        if(sample.wallClockMs < startWallClockMs || sample.wallClockMs > endWallClockMs){
            continue;
        }
        filtered.append(sample);
    }
    return filtered;
}

HardwareInterface::ConnectionDiagnostics HardwareInterface::connectionDiagnostics() const
{
    QThread* targetThread = thread();
    if(!targetThread || !targetThread->isRunning() || QThread::currentThread() == targetThread){
        const ConnectionDiagnostics snapshot = queryConnectionDiagnosticsDirect();
        storeConnectionDiagnosticsSnapshot(snapshot);
        return snapshot;
    }

    requestConnectionDiagnosticsRefresh();
    QMutexLocker locker(&connectionDiagnosticsMutex);
    return cachedConnectionDiagnostics;
}

void HardwareInterface::storeConnectionDiagnosticsSnapshot(const ConnectionDiagnostics& snapshot) const
{
    QMutexLocker locker(&connectionDiagnosticsMutex);
    cachedConnectionDiagnostics = snapshot;
    connectionDiagnosticsRefreshPending = false;
}

void HardwareInterface::requestConnectionDiagnosticsRefresh() const
{
    {
        QMutexLocker locker(&connectionDiagnosticsMutex);
        if(connectionDiagnosticsRefreshPending){
            return;
        }
        connectionDiagnosticsRefreshPending = true;
    }

    const bool posted = QMetaObject::invokeMethod(
                const_cast<HardwareInterface*>(this),
                [this](){
        const ConnectionDiagnostics snapshot = queryConnectionDiagnosticsDirect();
        storeConnectionDiagnosticsSnapshot(snapshot);
    }, Qt::QueuedConnection);

    if(!posted){
        QMutexLocker locker(&connectionDiagnosticsMutex);
        connectionDiagnosticsRefreshPending = false;
    }
}

HardwareInterface::ConnectionItemDiagnostics HardwareInterface::motorAxisDiagnostics(int logicalIndex) const
{
    return runOnHardwareThread([&]() -> ConnectionItemDiagnostics {
        return queryMotorAxisDiagnosticsDirect(logicalIndex);
    });
}

HardwareInterface::ConnectionItemDiagnostics HardwareInterface::controllerDiagnostics() const
{
    return runOnHardwareThread([&]() -> ConnectionItemDiagnostics {
        return queryControllerDiagnosticsDirect();
    });
}

void HardwareInterface::applyMotorCommandVelocityTraceRawValue(int logicalIndex, long rawValue)
{
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorIdVec.size())){
        return;
    }
    if(motorTraceCommandVelocity.size() != motorIdVec.size()){
        motorTraceCommandVelocity.assign(motorIdVec.size(), 0.0);
        motorTraceCommandVelocityValid.assign(motorIdVec.size(), false);
    }
    // 雷赛 Trace type 3 返回的是按当前脉冲当量换算后的板卡工程单位速度
    // （unit/s），不是原始 pulse/s；不能再除以 axisEquiv。
    motorTraceCommandVelocity[logicalIndex] = static_cast<double>(rawValue);
    motorTraceCommandVelocityValid[logicalIndex] =
            std::isfinite(motorTraceCommandVelocity[logicalIndex]);
}

void HardwareInterface::applyMotorActualVelocityTraceRawValue(int logicalIndex, long rawValue)
{
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorIdVec.size())){
        return;
    }
    if(motorTraceActualVelocity.size() != motorIdVec.size()){
        motorTraceActualVelocity.assign(motorIdVec.size(), 0.0);
        motorTraceActualVelocityValid.assign(motorIdVec.size(), false);
    }
    // 雷赛 Trace type 4 与 type 3 一致，返回值已经是当前板卡 unit/s。
    // type 5/6 位置仍通过 tracePulseToMotorUnit() 做 pulse -> unit 换算。
    motorTraceActualVelocity[logicalIndex] = static_cast<double>(rawValue);
    motorTraceActualVelocityValid[logicalIndex] =
            std::isfinite(motorTraceActualVelocity[logicalIndex]);
}

HardwareInterface::ConnectionItemDiagnostics
HardwareInterface::forceSensorDiagnostics(int sensorIndex) const
{
    return runOnHardwareThread([&]() -> ConnectionItemDiagnostics {
        ConnectionItemDiagnostics sensor;
        if(!isConnectLS){
            return sensor;
        }
        if(sensorIndex < 0 ||
                sensorIndex >= static_cast<int>(sensorComType.size()) ||
                sensorComType[sensorIndex] != COM_EC_LS_SBT){
            sensor.state = ConnectionState::Fault;
            sensor.apiResult = -1;
            return sensor;
        }
        if(!runtimeTraceUsageProfileIncludesForceSensors(
                    activeRuntimeTraceUsageProfile)){
            // 在线速度 Trace 配置会主动省略力传感器对象；这是性能配置，不能作为掉线故障上报。
            sensor.state = ConnectionState::Disabled;
            sensor.apiResult = 0;
            return sensor;
        }

        const bool hasTraceObject =
                std::any_of(forceSensorTraceObjects.cbegin(),
                            forceSensorTraceObjects.cend(),
                            [sensorIndex](const ForceSensorTraceObject& object){
            return object.sensorIndex == sensorIndex;
        });
        auto traceIsFresh = [this, sensorIndex]() -> bool {
            const qint64 nowUs = monotonicNowUs();
            const qint64 lastUs =
                    sensorIndex < static_cast<int>(forceSensorTraceValueMonotonicUs.size()) ?
                        forceSensorTraceValueMonotonicUs[sensorIndex] : 0;
            return lastUs > 0 &&
                    (nowUs < lastUs ||
                     nowUs - lastUs <= kForceSensorTraceDiagnosticsFreshTimeoutUs);
        };

        int traceReadResult = 0;
        if(runtimeTraceUsageProfileIncludesForceSensors(
                    activeRuntimeTraceUsageProfile) &&
                hasTraceObject && !traceIsFresh()){
            traceReadResult =
                    const_cast<HardwareInterface*>(this)->readRuntimeTraceCached();
        }
        if(traceIsFresh()){
            sensor.state = ConnectionState::Connected;
            sensor.apiResult = 0;
            return sensor;
        }

        sensor.state = ConnectionState::Fault;
        sensor.apiResult = !runtimeTraceUsageProfileIncludesForceSensors(
                    activeRuntimeTraceUsageProfile) ? -2 :
                           (!hasTraceObject ? -3 :
                            (traceReadResult < 0 ? traceReadResult : -4));
        return sensor;
    });
}

HardwareInterface::ConnectionItemDiagnostics HardwareInterface::queryControllerDiagnosticsDirect() const
{
    ConnectionItemDiagnostics controller;
    if(!isConnectLS){
        return controller;
    }

    uint32 masterState = 0;
    WORD cardErrCode = 0;
    const short masterRet = nmc_get_master_state(0, &masterState);
    const short cardErrRet = nmc_get_card_errcode(0, &cardErrCode);
    controller.busState = static_cast<int>(masterState);
    controller.errorCode = static_cast<int>(cardErrCode);
    controller.apiResult = masterRet != 0 ? masterRet : cardErrRet;
    if(masterRet != 0 || cardErrRet != 0 || cardErrCode != 0){
        controller.state = ConnectionState::Fault;
    }
    else if(masterState == 8U){
        controller.state = ConnectionState::Connected;
    }
    else if(masterState == 0U){
        controller.state = ConnectionState::Disconnected;
    }
    else{
        controller.state = ConnectionState::Fault;
    }
    return controller;
}

HardwareInterface::ConnectionItemDiagnostics HardwareInterface::queryMotorAxisDiagnosticsDirect(int logicalIndex) const
{
    ConnectionItemDiagnostics axis;
    if(!isConnectLS){
        return axis;
    }
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorComType.size())){
        axis.state = ConnectionState::Fault;
        axis.apiResult = -1;
        return axis;
    }
    if(motorComType[logicalIndex] != COM_EC_LS){
        return axis;
    }

    axis.hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if(axis.hardwareAxis < 0){
        axis.state = ConnectionState::Fault;
        axis.apiResult = -2;
        return axis;
    }

    WORD axisStateMachine = 0;
    long statusWord = 0;
    long stopReason = 0;
    WORD axisErrCode = 0;
    WORD slaveAddress = 0;
    WORD subSlaveAddress = 0;
    WORD slaveState = 0;

    const short stateMachineRet =
            nmc_get_axis_state_machine(0, static_cast<WORD>(axis.hardwareAxis), &axisStateMachine);
    const short statusWordRet =
            nmc_get_axis_statusword(0, static_cast<WORD>(axis.hardwareAxis), &statusWord);
    const short errCodeRet =
            nmc_get_axis_errcode(0, static_cast<WORD>(axis.hardwareAxis), &axisErrCode);
    const short stopReasonRet =
            dmc_get_stop_reason(0, static_cast<WORD>(axis.hardwareAxis), &stopReason);
    const short slaveAddressRet =
            nmc_get_axis_node_address(0,
                                      static_cast<WORD>(axis.hardwareAxis),
                                      &slaveAddress,
                                      &subSlaveAddress);

    axis.stateMachine = static_cast<int>(axisStateMachine);
    axis.statusWord = statusWord;
    axis.stopReason = stopReason;
    axis.stopReasonApiResult = stopReasonRet;
    axis.errorCode = static_cast<int>(axisErrCode);
    axis.slaveAddress = static_cast<int>(slaveAddress);
    axis.subSlaveAddress = static_cast<int>(subSlaveAddress);

    short slaveStateRet = 0;
    bool hasSlaveState = false;
    if(slaveAddressRet == 0 && slaveAddress > 0){
        slaveStateRet = nmc_get_slave_state(0, slaveAddress, &slaveState);
        if(slaveStateRet == 0){
            hasSlaveState = true;
            axis.busState = static_cast<int>(slaveState);
        }
    }

    if(stateMachineRet != 0 || statusWordRet != 0 || errCodeRet != 0 || slaveAddressRet != 0){
        axis.state = ConnectionState::Fault;
        axis.apiResult = stateMachineRet != 0 ? stateMachineRet :
                         (statusWordRet != 0 ? statusWordRet :
                          (errCodeRet != 0 ? errCodeRet : slaveAddressRet));
        return axis;
    }

    if(slaveAddress > 0 && slaveStateRet != 0){
        axis.state = ConnectionState::Fault;
        axis.apiResult = slaveStateRet;
        return axis;
    }

    if(axisErrCode != 0){
        axis.state = ConnectionState::Fault;
        return axis;
    }

    const bool axisEnabled = axisStateMachine == 4U;
    if(hasSlaveState){
        if(slaveState == 8U){
            axis.state = axisEnabled ?
                        ConnectionState::Connected :
                        ConnectionState::Disabled;
        }
        else if(slaveState == 0U){
            axis.state = ConnectionState::Disconnected;
        }
        else{
            axis.state = ConnectionState::Fault;
        }
        return axis;
    }

    axis.state = axisEnabled ?
                ConnectionState::Connected :
                ConnectionState::Disabled;
    return axis;
}

HardwareInterface::ConnectionDiagnostics HardwareInterface::queryConnectionDiagnosticsDirect() const
{
    ConnectionDiagnostics snapshot;
    snapshot.motorAxes.resize(motorComType.size());
    snapshot.forceSensors.resize(sensorComType.size());

    if(!isConnectLS){
        return snapshot;
    }

    snapshot.controller = queryControllerDiagnosticsDirect();

    for(int i = 0; i < static_cast<int>(motorComType.size()); ++i){
        ConnectionItemDiagnostics& axis = snapshot.motorAxes[i];
        if(motorComType[i] != COM_EC_LS){
            continue;
        }

        axis.hardwareAxis = resolveLeadshineAxisIndex(i);
        if(axis.hardwareAxis < 0){
            axis.state = ConnectionState::Fault;
            axis.apiResult = -1;
            continue;
        }

        WORD axisStateMachine = 0;
        long statusWord = 0;
        long stopReason = 0;
        WORD axisErrCode = 0;
        WORD slaveAddress = 0;
        WORD subSlaveAddress = 0;
        WORD slaveState = 0;

        const short stateMachineRet =
                nmc_get_axis_state_machine(0, static_cast<WORD>(axis.hardwareAxis), &axisStateMachine);
        const short statusWordRet =
                nmc_get_axis_statusword(0, static_cast<WORD>(axis.hardwareAxis), &statusWord);
        const short errCodeRet =
                nmc_get_axis_errcode(0, static_cast<WORD>(axis.hardwareAxis), &axisErrCode);
        const short stopReasonRet =
                dmc_get_stop_reason(0, static_cast<WORD>(axis.hardwareAxis), &stopReason);
        const short slaveAddressRet =
                nmc_get_axis_node_address(0,
                                          static_cast<WORD>(axis.hardwareAxis),
                                          &slaveAddress,
                                          &subSlaveAddress);

        axis.stateMachine = static_cast<int>(axisStateMachine);
        axis.statusWord = statusWord;
        axis.stopReason = stopReason;
        axis.stopReasonApiResult = stopReasonRet;
        axis.errorCode = static_cast<int>(axisErrCode);
        axis.slaveAddress = static_cast<int>(slaveAddress);
        axis.subSlaveAddress = static_cast<int>(subSlaveAddress);

        short slaveStateRet = 0;
        bool hasSlaveState = false;
        if(slaveAddressRet == 0 && slaveAddress > 0){
            slaveStateRet = nmc_get_slave_state(0, slaveAddress, &slaveState);
            if(slaveStateRet == 0){
                hasSlaveState = true;
                axis.busState = static_cast<int>(slaveState);
            }
        }

        if(stateMachineRet != 0 || statusWordRet != 0 || errCodeRet != 0 || slaveAddressRet != 0){
            axis.state = ConnectionState::Fault;
            axis.apiResult = stateMachineRet != 0 ? stateMachineRet :
                             (statusWordRet != 0 ? statusWordRet :
                              (errCodeRet != 0 ? errCodeRet : slaveAddressRet));
            continue;
        }

        if(slaveAddress > 0 && slaveStateRet != 0){
            axis.state = ConnectionState::Fault;
            axis.apiResult = slaveStateRet;
            continue;
        }

        if(axisErrCode != 0){
            axis.state = ConnectionState::Fault;
            continue;
        }

        const bool axisEnabled = axisStateMachine == 4U;
        if(hasSlaveState){
            if(slaveState == 8U){
                axis.state = axisEnabled ?
                            ConnectionState::Connected :
                            ConnectionState::Disabled;
            }
            else if(slaveState == 0U){
                axis.state = ConnectionState::Disconnected;
            }
            else{
                axis.state = ConnectionState::Fault;
            }
            continue;
        }

        axis.state = axisEnabled ?
                    ConnectionState::Connected :
                    ConnectionState::Disabled;
    }

    qint64 diagnosticsNowUs = monotonicNowUs();
    auto forceSensorTraceValueFresh = [this, diagnosticsNowUs](int sensorIndex) -> bool {
        const qint64 lastTraceValueUs =
                sensorIndex < static_cast<int>(forceSensorTraceValueMonotonicUs.size()) ?
                    forceSensorTraceValueMonotonicUs[sensorIndex] :
                    0;
        return lastTraceValueUs > 0 &&
                (diagnosticsNowUs < lastTraceValueUs ||
                 diagnosticsNowUs - lastTraceValueUs <= kForceSensorTraceDiagnosticsFreshTimeoutUs);
    };

    bool needsForceTraceRefresh = false;
    for(int i = 0; i < static_cast<int>(sensorComType.size()); ++i){
        if(sensorComType[i] == COM_EC_LS_SBT && !forceSensorTraceValueFresh(i)){
            needsForceTraceRefresh = true;
            break;
        }
    }

    int forceTraceReadResult = 0;
    if(runtimeTraceUsageProfileIncludesForceSensors(
                activeRuntimeTraceUsageProfile) && needsForceTraceRefresh){
        const bool shouldPollTrace =
                forceSensorDiagnosticsLastTraceReadUs <= 0 ||
                diagnosticsNowUs - forceSensorDiagnosticsLastTraceReadUs >=
                    kForceSensorTraceDiagnosticsPollIntervalUs;
        if(shouldPollTrace){
            forceTraceReadResult = const_cast<HardwareInterface*>(this)->readRuntimeTraceCached();
            forceSensorDiagnosticsLastTraceReadUs = monotonicNowUs();
            diagnosticsNowUs = forceSensorDiagnosticsLastTraceReadUs;
        }
    }

    for(int i = 0; i < static_cast<int>(sensorComType.size()); ++i){
        ConnectionItemDiagnostics& sensor = snapshot.forceSensors[i];
        if(sensorComType[i] != COM_EC_LS_SBT){
            continue;
        }
        if(!runtimeTraceUsageProfileIncludesForceSensors(
                    activeRuntimeTraceUsageProfile)){
            // 在线速度模式下该通道由配置主动停用，不参与连接/安全故障判定。
            sensor.state = ConnectionState::Disabled;
            sensor.apiResult = 0;
            continue;
        }

        const bool hasTraceObject =
                std::any_of(forceSensorTraceObjects.begin(),
                            forceSensorTraceObjects.end(),
                            [i](const ForceSensorTraceObject& object){
            return object.sensorIndex == i;
        });
        const qint64 lastTraceValueUs =
                i < static_cast<int>(forceSensorTraceValueMonotonicUs.size()) ?
                    forceSensorTraceValueMonotonicUs[i] :
                    0;
        const bool traceValueFresh =
                lastTraceValueUs > 0 &&
                (diagnosticsNowUs < lastTraceValueUs ||
                 diagnosticsNowUs - lastTraceValueUs <= kForceSensorTraceDiagnosticsFreshTimeoutUs);

        if(traceValueFresh){
            sensor.apiResult = 0;
            sensor.state = ConnectionState::Connected;
        }
        else{
            sensor.state = ConnectionState::Fault;
            if(!runtimeTraceUsageProfileIncludesForceSensors(
                        activeRuntimeTraceUsageProfile)){
                sensor.apiResult = -2;
            }
            else if(!hasTraceObject){
                sensor.apiResult = -3;
            }
            else if(forceTraceReadResult < 0){
                sensor.apiResult = forceTraceReadResult;
            }
            else{
                sensor.apiResult = -4;
            }
        }
    }

    return snapshot;
}

void HardwareInterface::resetDiagnostics()
{
    QMutexLocker locker(&diagnosticsMutex);
    diagnostics = DiagnosticsSnapshot{};
    lastCommunicationEventUs = 0;
    lastMotorCommandEventUs = 0;
    communicationRawHistory.clear();
    motorCommandRawHistory.clear();
    motorPositionRawSamples.clear();
    motorEncoderRawSamples.clear();
    motorTraceFeedbackRawSamples.clear();
    runtimeTraceFetchTimingSamples.clear();
    pvtTableUploadTimingSamples.clear();
    lastCommunicationHistoryTrimMs = 0;
    lastMotorCommandHistoryTrimMs = 0;
    lastMotorPositionReadUs = 0;
    lastMotorPositionRawHistoryTrimMs = 0;
    lastMotorEncoderReadUs = 0;
    lastMotorEncoderRawHistoryTrimMs = 0;
    lastMotorTraceFeedbackRawUs = 0;
    lastMotorTraceFeedbackRawHistoryTrimMs = 0;
    lastRuntimeTraceFetchTimingUs = 0;
    lastRuntimeTraceFetchTimingHistoryTrimMs = 0;
    lastPvtTableUploadTimingHistoryTrimMs = 0;
    lastCommunicationRawHistoryAppendUs = 0;
    lastMotorCommandRawHistoryAppendUs = 0;
    lastMotorPositionRawHistoryAppendUs = 0;
    lastMotorEncoderRawHistoryAppendUs = 0;
    lastMotorTraceFeedbackRawHistoryAppendUs = 0;
    lastRuntimeTraceFetchTimingHistoryAppendUs = 0;
    lastPvtTableUploadTimingHistoryAppendUs = 0;
}

void HardwareInterface::checkJointSensorDataManual() {
}

void HardwareInterface::checkAllMotorState() {
    return runOnHardwareThread([&]() {
    if(motorCurState.size() != motorIdVec.size()){
        motorCurState.assign(motorIdVec.size(), false);
    }
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        refreshLeadshineMotorEnableState(i);
    }

    });
}

void HardwareInterface::checkAllMotorPos() {
    return runOnHardwareThread([&]() {
    getAllMotorPos();
    emit motorCurPosUpdateSignal(motorCurPos);
    });
}

void HardwareInterface::checkAllMotorVel() {
    return runOnHardwareThread([&]() {
    emit motorCurVelUpdateSignal(motorCurVel);
    });
}

std::vector<bool> HardwareInterface::getAllMotorState() {
    return runOnHardwareThread([&]() -> std::vector<bool> {
    if(motorCurState.size() != motorIdVec.size()){
        motorCurState.assign(motorIdVec.size(), false);
    }
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        refreshLeadshineMotorEnableState(i);
    }
    return motorCurState;
    });
}

bool HardwareInterface::isMotorEnabled(int index) {
    if(!motorEnableQueryTimingEnabled.load(std::memory_order_acquire)){
        return runOnHardwareThread([&]() -> bool {
        if(motorCurState.size() != motorIdVec.size()){
            motorCurState.assign(motorIdVec.size(), false);
        }
        return refreshLeadshineMotorEnableState(index);
        });
    }

    const qint64 requestStartUs = monotonicNowUs();
    qint64 hardwareEntryUs = 0;
    qint64 apiDurationUs = 0;
    const bool enabled = runOnHardwareThread([&]() -> bool {
        hardwareEntryUs = monotonicNowUs();
        if(motorCurState.size() != motorIdVec.size()){
            motorCurState.assign(motorIdVec.size(), false);
        }
        return refreshLeadshineMotorEnableState(index, &apiDurationUs);
    });
    const qint64 callCompleteUs = monotonicNowUs();
    const qint64 queueWaitUs = hardwareEntryUs > requestStartUs ?
                hardwareEntryUs - requestStartUs : 0;
    const qint64 callDurationUs = std::max<qint64>(0,
                                                   callCompleteUs - requestStartUs);
    motorEnableQueryCount.fetch_add(1, std::memory_order_relaxed);
    motorEnableQueryTotalQueueWaitUs.fetch_add(queueWaitUs,
                                                std::memory_order_relaxed);
    motorEnableQueryLatestQueueWaitUs.store(queueWaitUs,
                                             std::memory_order_relaxed);
    updateAtomicMaximum(motorEnableQueryMaximumQueueWaitUs, queueWaitUs);
    motorEnableQueryTotalApiDurationUs.fetch_add(apiDurationUs,
                                                  std::memory_order_relaxed);
    motorEnableQueryLatestApiDurationUs.store(apiDurationUs,
                                               std::memory_order_relaxed);
    updateAtomicMaximum(motorEnableQueryMaximumApiDurationUs, apiDurationUs);
    motorEnableQueryTotalCallDurationUs.fetch_add(callDurationUs,
                                                   std::memory_order_relaxed);
    motorEnableQueryLatestCallDurationUs.store(callDurationUs,
                                                std::memory_order_relaxed);
    updateAtomicMaximum(motorEnableQueryMaximumCallDurationUs, callDurationUs);
    return enabled;
}

std::vector<double> HardwareInterface::getAllMotorPos() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    std::vector<int> leadshineAxes;
    leadshineAxes.reserve(motorIdVec.size());
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (i >= static_cast<int>(motorComType.size()) || motorComType[i] != COM_EC_LS) {
            continue;
        }
        leadshineAxes.push_back(i);
    }
    if(leadshineAxes.empty()){
        return std::vector<double>{};
    }

    recordCurrentMotorEncoderUnitsForAxes(leadshineAxes);

    const std::vector<double> tracePositions = readMotorPositionsTraceCached(leadshineAxes);
    if(tracePositions.size() == leadshineAxes.size()){
        for(int i = 0; i < static_cast<int>(leadshineAxes.size()); ++i){
            const int axis = leadshineAxes[i];
            if(axis >= 0 && axis < static_cast<int>(motorCurPos.size())){
                motorCurPos[axis] = tracePositions[i];
            }
        }
        recordMotorPositionRawSample(motorCurPos, QStringLiteral("trace_actual_aligned"));
        return motorCurPos;
    }

    for (const int i : leadshineAxes) {
        double unitPos = 0.0;
        if(!readMotorPositionUnitDirect(i, unitPos)){
            return std::vector<double>{};
        }
    }
    recordMotorPositionRawSample(motorCurPos, QStringLiteral("dmc_get_position_unit_fallback"));
    return motorCurPos;
    });
}

std::vector<double> HardwareInterface::getAllMotorPosUnit() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    bool hasLeadshineAxis = false;
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (i >= static_cast<int>(motorComType.size()) || motorComType[i] != COM_EC_LS) {
            continue;
        }
        hasLeadshineAxis = true;
        double unitPos = 0.0;
        if(!readMotorPositionUnitDirect(i, unitPos)){
            return std::vector<double>{};
        }
    }
    if(!hasLeadshineAxis){
        return std::vector<double>{};
    }
    return motorCurPos;
    });
}

std::vector<double> HardwareInterface::getAllMotorEncoderPosUnit() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    std::vector<double> encoderPos(motorIdVec.size(),
                                   std::numeric_limits<double>::quiet_NaN());
    bool hasLeadshineAxis = false;
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (i >= static_cast<int>(motorComType.size()) || motorComType[i] != COM_EC_LS) {
            continue;
        }
        hasLeadshineAxis = true;
        double unitPos = 0.0;
        if(!readMotorEncoderUnitDirect(i, unitPos)){
            return std::vector<double>{};
        }
        encoderPos[i] = unitPos;
    }
    if(!hasLeadshineAxis){
        return std::vector<double>{};
    }
    recordMotorEncoderRawSample(encoderPos, QStringLiteral("dmc_get_encoder_unit_snapshot"));
    return encoderPos;
    });
}

std::vector<double> HardwareInterface::getAllMotorVel() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (motorComType[i] == COM_EC_LS) {
            const int hardwareAxis = resolveLeadshineAxisIndex(i);
            if (hardwareAxis >= 0) {
                dmc_read_current_speed_unit(0, static_cast<WORD>(hardwareAxis), &motorCurVel[i]);
                recordCommunicationEvent(false, QStringLiteral("dmc_read_current_speed_unit"));
            }
        }
    }
    return motorCurVel;
    });
}

std::vector<double> HardwareInterface::getAllMotorTorqueNmTraceCached()
{
    return runOnHardwareThread([&]() -> std::vector<double> {
        return readMotorTorqueTraceCachedDirect();
    });
}

std::vector<double> HardwareInterface::getAllMotorTorqueNm() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    return readMotorTorqueTraceCachedDirect();
    });
}

std::vector<double> HardwareInterface::getAllMotorHome() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    return motorHomePos;
    });
}

std::vector<qint64> HardwareInterface::getAllMotorSafetyHomeTraceCommandRawPulse()
{
    return runOnHardwareThread([&]() -> std::vector<qint64> {
    return motorSafetyHomeTraceCommandRawPulse;
    });
}

std::vector<double> HardwareInterface::getAllMotorSafetyHomeUnit()
{
    return runOnHardwareThread([&]() -> std::vector<double> {
    std::vector<double> safetyHome(motorIdVec.size(),
                                   std::numeric_limits<double>::quiet_NaN());
    for(int i = 0; i < static_cast<int>(motorIdVec.size()); ++i){
        safetyHome[i] = motorSafetyHomeUnit(i);
    }
    return safetyHome;
    });
}

std::vector<double> HardwareInterface::getAllMotorSafetyRelativePosUnit()
{
    return runOnHardwareThread([&]() -> std::vector<double> {
    std::vector<double> relativePos(motorIdVec.size(),
                                    std::numeric_limits<double>::quiet_NaN());
    bool hasLeadshineAxis = false;
    for(int i = 0; i < static_cast<int>(motorIdVec.size()); ++i){
        if(i >= static_cast<int>(motorComType.size()) || motorComType[i] != COM_EC_LS){
            continue;
        }
        hasLeadshineAxis = true;
        double relative = std::numeric_limits<double>::quiet_NaN();
        if(!readMotorSafetyRelativePositionDirect(i, relative)){
            return std::vector<double>{};
        }
        relativePos[i] = relative;
    }
    return hasLeadshineAxis ? relativePos : std::vector<double>{};
    });
}

void HardwareInterface::motorPosTraj(std::vector<int> motorIndex, std::vector<std::vector<double>> motorPosTraj,
                                     std::vector<std::vector<double>> motorVel, std::vector<double> motorVelMax,
                                     std::vector<double> timeStamp) {
    return runOnHardwareThread([&]() {
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtMotorVelMax.clear();
    activePvtTimeStamp.clear();
    activePvtStartMonotonicUs = 0;
    const std::vector<double> beginVel(motorIndex.size(), 0.0);
    const std::vector<double> endVel(motorIndex.size(), 0.0);
    startPvtTable(motorIndex,
                  motorPosTraj,
                  motorVel,
                  motorVelMax,
                  timeStamp,
                  beginVel,
                  endVel,
                  true,
                  QStringLiteral("PVT position trajectory started"));
    });
}

void HardwareInterface::delay(unsigned int msec) {
    QThread::msleep(msec);
}
