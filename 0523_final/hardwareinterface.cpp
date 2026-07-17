#include "hardwareinterface.h"
#include "trajectoryplanner.h"

#include <QDateTime>
#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <utility>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace {
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

qint64 monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

constexpr qint64 kDiagnosticRawRetentionMs = 6 * 60 * 1000;
constexpr qint64 kDiagnosticRawTrimIntervalMs = 1000;
constexpr int kForceSensorTraceBaseCycleUs = 500;
constexpr int kForceSensorTraceMinPeriodUs = 500;
constexpr int kForceSensorTraceMaxPeriodUs = 1000;
constexpr std::size_t kMaxMotorTracePositionSamplesPerAxis = 8192;
constexpr std::size_t kMaxForceSensorTraceSamples = 8192;
constexpr WORD kLeadshineEtherCatPort = 2;
constexpr WORD kLeadshineTorqueVelocityLimitIndex = 0x220B;
constexpr WORD kLeadshineTorqueVelocityLimitSubIndex = 0;
constexpr WORD kLeadshineTorqueVelocityLimitBitLength = 32;
constexpr WORD kLeadshineFollowingErrorActualIndex = 0x60F4;
constexpr WORD kLeadshineFollowingErrorActualSubIndex = 0;
constexpr WORD kLeadshineFollowingErrorActualBitLength = 32;
constexpr short kLeadshineTraceDataTypeCommandPosition = 5;
constexpr short kLeadshineTraceDataTypeActualPosition = 6;
constexpr short kLeadshineTracePositionDataBytes = 4;
constexpr double kLeadshineTorqueVelocityLimitPulsesPerRev = 360000.0;
constexpr double kLeadshineRatedMotorTorqueNm = 45.0;
constexpr double kLeadshineTorqueFeedbackRawPerRatedTorque = 1000.0;
constexpr WORD kTorquePositionLimitDisabled = 0;
constexpr double kTorquePositionLimitValueUnused = 0.0;
constexpr WORD kTorqueAbsolutePositionMode = 1;

int leadshineTorqueNmToRaw(double torqueNm)
{
    const double rawTorque = torqueNm *
            kLeadshineTorqueFeedbackRawPerRatedTorque /
            kLeadshineRatedMotorTorqueNm;
    long raw = std::lround(rawTorque);
    if(raw == 0 && torqueNm != 0.0){
        raw = torqueNm > 0.0 ? 1 : -1;
    }
    raw = std::clamp(raw,
                     static_cast<long>(std::numeric_limits<int>::min()),
                     static_cast<long>(std::numeric_limits<int>::max()));
    return static_cast<int>(raw);
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
}

HardwareInterface::HardwareInterface() = default;

HardwareInterface::HardwareInterface(double _threadCtrlCycleMs)
    : threadCtrlCycleMs(_threadCtrlCycleMs) {
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
    const int hardwareAxis = resolveLeadshineAxisIndex(logicalIndex);
    if(hardwareAxis >= 0){
        return QStringLiteral("逻辑轴%1(控制卡轴%2)")
                .arg(logicalIndex)
                .arg(hardwareAxis);
    }
    return QStringLiteral("逻辑轴%1").arg(logicalIndex);
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
            *errorMessage = QStringLiteral("错误：%1不会下发，逻辑轴%2目标位置%3会导致电机超出软件位置限位[%4, %5]")
                    .arg(commandName)
                    .arg(logicalIndex)
                    .arg(relativePosition, 0, 'f', 6)
                    .arg(minPos, 0, 'f', 6)
                    .arg(maxPos, 0, 'f', 6);
        }
        return false;
    }

    return true;
}

bool HardwareInterface::validateAbsoluteMotorSoftwareLimit(int logicalIndex,
                                                           double absolutePosition,
                                                           const QString& commandName,
                                                           QString* errorMessage) const
{
    return validateRelativeMotorSoftwareLimit(logicalIndex,
                                              relativeMotorPosition(logicalIndex, absolutePosition),
                                              commandName,
                                              errorMessage);
}

bool HardwareInterface::validateVelocityMotorSoftwareLimit(int logicalIndex,
                                                           double currentAbsolutePosition,
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

    const double relativePosition = relativeMotorPosition(logicalIndex, currentAbsolutePosition);
    const double minPos = motorSoftwareMinPos[logicalIndex];
    const double maxPos = motorSoftwareMaxPos[logicalIndex];
    const bool movingPastUpperLimit = velocity > 0.0 && relativePosition >= maxPos;
    const bool movingPastLowerLimit = velocity < 0.0 && relativePosition <= minPos;
    if(!std::isfinite(relativePosition) ||
            !std::isfinite(velocity) ||
            movingPastUpperLimit ||
            movingPastLowerLimit){
        if(errorMessage){
            *errorMessage = QStringLiteral("错误：%1不会下发，逻辑轴%2当前位置%3继续按速度%4运动会导致电机超出软件位置限位[%5, %6]")
                    .arg(commandName)
                    .arg(logicalIndex)
                    .arg(relativePosition, 0, 'f', 6)
                    .arg(velocity, 0, 'f', 6)
                    .arg(minPos, 0, 'f', 6)
                    .arg(maxPos, 0, 'f', 6);
        }
        return false;
    }

    return true;
}

void HardwareInterface::recordCommunicationEvent(bool motorCommandEvent)
{
    const qint64 nowUs = monotonicNowUs();
    const qint64 nowWallClockMs = QDateTime::currentMSecsSinceEpoch();
    QMutexLocker locker(&diagnosticsMutex);
    diagnostics.communicationEventCount++;
    if(lastCommunicationEventUs > 0){
        const qint64 dtUs = std::max<qint64>(0, nowUs - lastCommunicationEventUs);
        diagnostics.communicationIntervalCount++;
        diagnostics.communicationIntervalSumUs += dtUs;
        diagnostics.latestCommunicationIntervalUs = dtUs;
        communicationRawHistory.append({nowWallClockMs, dtUs});
        if(nowWallClockMs - lastCommunicationHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
            trimRawHistory(communicationRawHistory, nowWallClockMs - kDiagnosticRawRetentionMs);
            lastCommunicationHistoryTrimMs = nowWallClockMs;
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
            motorCommandRawHistory.append({nowWallClockMs, dtUs});
            if(nowWallClockMs - lastMotorCommandHistoryTrimMs >= kDiagnosticRawTrimIntervalMs){
                trimRawHistory(motorCommandRawHistory, nowWallClockMs - kDiagnosticRawRetentionMs);
                lastMotorCommandHistoryTrimMs = nowWallClockMs;
            }
        }
        lastMotorCommandEventUs = nowUs;
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
    motorCurVel.assign(motorIdVec.size(), 0.0);
    motorEquivVec.resize(motorIdVec.size(), 0.0);
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
    });
}

void HardwareInterface::setLeadshineAxisEquiv(std::vector<double> equivValue) {
    return runOnHardwareThread([&]() {
    motorEquivVec = std::move(equivValue);
    if(motorEquivVec.size() < motorIdVec.size()){
        motorEquivVec.resize(motorIdVec.size(), 0.0);
    }
    resetMotorPositionTraceState();
    });
}

void HardwareInterface::setMotorSoftwareLimits(std::vector<double> minPos,
                                               std::vector<double> maxPos)
{
    return runOnHardwareThread([&]() {
    motorSoftwareMinPos = std::move(minPos);
    motorSoftwareMaxPos = std::move(maxPos);
    const std::size_t axisCount = motorIdVec.size();
    if(motorSoftwareMinPos.size() < axisCount){
        motorSoftwareMinPos.resize(axisCount, 0.0);
    }
    if(motorSoftwareMaxPos.size() < axisCount){
        motorSoftwareMaxPos.resize(axisCount, 0.0);
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
    nextForceSensorPollIndex = 0;
    resetForceSensorTraceState();
    });
}

void HardwareInterface::setForceSensorIsSigned(bool isSigned) {
    return runOnHardwareThread([&]() {
    forceSensorIsSigned = isSigned;
    forceSensorCachedValue.assign(sensorComType.size(), 0.0);
    forceSensorCacheValid.assign(sensorComType.size(), false);
    nextForceSensorPollIndex = 0;
    resetForceSensorTraceState();
    });
}

void HardwareInterface::setForceSensorTraceReadEnabled(bool enabled)
{
    return runOnHardwareThread([&]() {
    if(forceSensorTraceReadEnabled == enabled){
        return;
    }
    forceSensorTraceReadEnabled = enabled;
    resetForceSensorTraceState();
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
    if(forceSensorTraceSamplePeriodUs == boundedPeriodUs){
        return;
    }

    forceSensorTraceSamplePeriodUs = boundedPeriodUs;
    resetForceSensorTraceState();
    });
}

bool HardwareInterface::connectLS() {
    return runOnHardwareThread([&]() -> bool {
    if (isConnectLS) {
        return true;
    }

    const short boardCount = dmc_board_init();
    recordCommunicationEvent(false);
    if (boardCount <= 0) {
        isConnectLS = false;
        emit displayInfoSignal("Leadshine control card not detected.", "error");
        return false;
    }

    isConnectLS = true;
    resetRuntimeTraceState();
    auto closeBoardOnConnectFailure = [this]() {
        resetRuntimeTraceState();
        dmc_board_close();
        isConnectLS = false;
    };

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
            recordCommunicationEvent(false);
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
            if(!runtimeTraceConfigured){
                configureRuntimeTraceRead();
                delay(static_cast<unsigned int>(std::max(2, motorPositionTraceSamplePeriodUs / 1000 + 1)));
            }
            const short posRet = readMotorPositionTraceCached(i, tmp) ? 0 : -1;
            if (posRet != 0) {
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
        recordCommunicationEvent(false);
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
            recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
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

bool HardwareInterface::disconnectLS() {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        return true;
    }
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtTimeStamp.clear();

    setAllMotorEnable(false);
    resetForceSensorTraceState();
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

    WORD busErrCode = 0;
    const short busErrRet = nmc_get_errcode(0, 2, &busErrCode);
    recordCommunicationEvent(true);
    if (busErrRet == 0 && busErrCode != 0) {
        const short clearBusRet = nmc_clear_errcode(0, 2);
        recordCommunicationEvent(true);
        if (emitErrors) {
            emit displayInfoSignal(QString("雷赛总线存在错误码%1，已尝试清除，清除返回码%2")
                                       .arg(busErrCode)
                                       .arg(clearBusRet)
                                       .toStdString(),
                                   clearBusRet == 0 ? "warning" : "error");
        }
        QThread::msleep(20);
    }

    WORD axisErrCode = 0;
    const short axisErrRet = nmc_get_axis_errcode(0, static_cast<WORD>(hardwareAxis), &axisErrCode);
    recordCommunicationEvent(true);
    if (axisErrRet == 0 && axisErrCode != 0) {
        const short clearAxisRet = nmc_clear_axis_errcode(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true);
        if (emitErrors) {
            emit displayInfoSignal(QString("%1存在轴错误码%2，已尝试清除，清除返回码%3")
                                       .arg(axisDisplayName(logicalIndex))
                                       .arg(axisErrCode)
                                       .arg(clearAxisRet)
                                       .toStdString(),
                                   clearAxisRet == 0 ? "warning" : "error");
        }
        QThread::msleep(20);
    }

    if (!enable) {
        const short ret = nmc_set_axis_disable(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true);
        if (ret == 0) {
            motorCurState[logicalIndex] = false;
            return true;
        }
        motorCurState[logicalIndex] = false;
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
        recordCommunicationEvent(true);
        lastStateRet = nmc_get_axis_state_machine(0, static_cast<WORD>(hardwareAxis), &lastStateMachine);
        recordCommunicationEvent(true);
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

bool HardwareInterface::refreshLeadshineMotorEnableState(int logicalIndex)
{
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
            const short ret =
                    nmc_get_axis_state_machine(0, static_cast<WORD>(hardwareAxis), &stateMachine);
            recordCommunicationEvent(false);
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
        emit displayInfoSignal("电机轴号超出范围", "error");
        return false;
    }
    if (motorComType[index] != COM_EC_LS) {
        emit displayInfoSignal("当前轴未配置为雷赛电机轴", "error");
        return false;
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }

    return applyLeadshineAxisEnableState(index, enable, true);
    });
}

bool HardwareInterface::emergencyStopAll() {
    return runOnHardwareThread([&]() -> bool {
    if (!isConnectLS) {
        return false;
    }

    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtTimeStamp.clear();

    for (int i = 0; i < static_cast<int>(motorComType.size()); ++i) {
        if (motorComType[i] != COM_EC_LS) {
            continue;
        }
        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        if (hardwareAxis < 0) {
            motorCurState[i] = false;
            continue;
        }
        dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
        recordCommunicationEvent(true);
        nmc_set_axis_disable(0, static_cast<WORD>(hardwareAxis));
        recordCommunicationEvent(true);
        motorCurState[i] = false;
    }

    return true;
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
        recordCommunicationEvent(true);
        if (ret != 0) {
            emit displayInfoSignal(QString("%1轴错误码清除失败，返回码%2")
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
    return runOnHardwareThread([&]() -> bool {
    return isConnectLS;
    });
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

void HardwareInterface::clearPvtTrajectoryState() {
    return runOnHardwareThread([&]() {
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtTimeStamp.clear();
    });
}

bool HardwareInterface::pausePvtMotion() {
    return runOnHardwareThread([&]() -> bool {
    return smoothPausePvtMotion(0.5);
    });
}

bool HardwareInterface::resumePvtMotion() {
    return runOnHardwareThread([&]() -> bool {
    return smoothResumePvtMotion(0.5);
    });
}

bool HardwareInterface::smoothPausePvtMotion(double transitionTimeSec) {
    return runOnHardwareThread([&]() -> bool {
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
        emit displayInfoSignal("当前 PVT 位置轨迹未在运动，不能执行速度置零", "warning");
        return false;
    }

    double currentTrajectoryTime = 0.0;
    std::size_t currentIndex = 0;
    if (!getPvtCurrentProgress(currentTrajectoryTime, currentIndex)) {
        emit displayInfoSignal("PVT pause failed: cannot read current trajectory progress.", "error");
        return false;
    }

    const int stopWaitMs = static_cast<int>(std::ceil(std::max(transitionTimeSec, 0.1) * 1000.0)) + 500;
    if (!stopActivePvtAxesAndWait(stopWaitMs)) {
        emit displayInfoSignal("PVT 轨迹暂停失败：原 PVT 轨迹未能可靠停止", "error");
        return false;
    }

    double stoppedTrajectoryTime = currentTrajectoryTime;
    std::size_t stoppedIndex = currentIndex;
    if (getPvtCurrentProgress(stoppedTrajectoryTime, stoppedIndex)) {
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
                         remainTimeStamp,
                         beginVel,
                         endVel,
                         true,
                         QStringLiteral("PVT position trajectory resumed"));
    });
}

std::vector<double> HardwareInterface::readMotorPositions(const std::vector<int>& motorIndex) const {
    return const_cast<HardwareInterface*>(this)->readMotorPositionsTraceCached(motorIndex);
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
        const_cast<HardwareInterface*>(this)->recordCommunicationEvent(false);
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
        const_cast<HardwareInterface*>(this)->recordCommunicationEvent(false);
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
                const_cast<HardwareInterface*>(this)->recordCommunicationEvent(false);
                allDone = false;
                break;
            }
            const_cast<HardwareInterface*>(this)->recordCommunicationEvent(false);
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
        recordCommunicationEvent(true);
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

bool HardwareInterface::startPvtTable(const std::vector<int>& motorIndex,
                                      const std::vector<std::vector<double>>& motorPosTraj,
                                      const std::vector<std::vector<double>>& motorVelTraj,
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
    if (motorVelTraj.size() < motorPosTraj.size()) {
        emit displayInfoSignal("PVT 速度轨迹数据不足", "error");
        return false;
    }

    const DWORD pointCount = static_cast<DWORD>(std::min<std::size_t>(motorPosTraj.size(), 5000));
    for (DWORD pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
        if (motorPosTraj[pointIndex].size() < motorIndex.size()) {
            emit displayInfoSignal("PVT position dimension does not match motor count.", "error");
            return false;
        }
        if (pointIndex > 0 && timeStamp[pointIndex] <= timeStamp[pointIndex - 1]) {
            emit displayInfoSignal("PVT 时间戳必须严格递增", "error");
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

        for (DWORD pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
            QString limitError;
            if(!validateRelativeMotorSoftwareLimit(
                        axis,
                        motorPosTraj[pointIndex][axisVecIndex],
                        QStringLiteral("PVT位置轨迹"),
                        &limitError)){
                emit displayInfoSignal(QString("%1，轨迹点%2")
                                           .arg(limitError)
                                           .arg(static_cast<int>(pointIndex))
                                           .toStdString(),
                                       "error");
                return false;
            }
        }
    }

    const WORD cardNo = 0;
    std::vector<WORD> axisList;
    axisList.reserve(motorIndex.size());

    for (int axisVecIndex = 0; axisVecIndex < static_cast<int>(motorIndex.size()); ++axisVecIndex) {
        const int axis = motorIndex[axisVecIndex];
        const int hardwareAxis = resolveLeadshineAxisIndex(axis);
        if (axis < 0 || axis >= static_cast<int>(motorComType.size()) || axis >= static_cast<int>(motorHomePos.size())) {
            emit displayInfoSignal("PVT 电机轴号超出范围", "error");
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
        const short err = dmc_pvts_table_unit(cardNo,
                                             static_cast<WORD>(hardwareAxis),
                                             pointCount,
                                             pTime.data(),
                                             pPos.data(),
                                             axisBeginVel,
                                             axisEndVel);
        recordCommunicationEvent(true);
        if (err != 0) {
            emit displayInfoSignal(QString("PVT 数据表下发失败，%1，错误码%2，首点pos=%3，首点time=%4")
                                       .arg(axisDisplayName(axis))
                                       .arg(err)
                                       .arg(pPos[0])
                                       .arg(pTime[0])
                                       .toStdString(), "error");
            return false;
        }
        axisList.push_back(static_cast<WORD>(hardwareAxis));
    }

    if (axisList.empty()) {
        emit displayInfoSignal("PVT axis list is empty; motion not started.", "error");
        return false;
    }

    const short err = dmc_pvt_move(cardNo, static_cast<WORD>(axisList.size()), axisList.data());
    recordCommunicationEvent(true);
    if (err != 0) {
        emit displayInfoSignal(QString("PVT 运动启动失败，错误码%1").arg(err).toStdString(), "error");
        return false;
    }

    if (updateActiveTrajectory) {
        activePvtMotorIndex = motorIndex;
        activePvtMotorPosTraj.assign(motorPosTraj.begin(), motorPosTraj.begin() + static_cast<std::ptrdiff_t>(pointCount));
        activePvtMotorVelTraj.assign(motorVelTraj.begin(), motorVelTraj.begin() + static_cast<std::ptrdiff_t>(pointCount));
        activePvtTimeStamp.assign(timeStamp.begin(), timeStamp.begin() + static_cast<std::ptrdiff_t>(pointCount));
        hasActivePvtTrajectory = true;
        isPvtMotionPaused = false;
        pausedPvtResumeIndex = 0;
        pausedPvtResumeTime = 0.0;
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
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtTimeStamp.clear();
    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.").arg(axisDisplayName(index)).toStdString(), "error");
        return false;
    }
    dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
    recordCommunicationEvent(true);
    return true;
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
    if(!readMotorPositionTraceCached(index, pos)){
        emit displayInfoSignal(QString("Trace feedback position read failed for motor axis %1.")
                                   .arg(index)
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
    recordCommunicationEvent(true);
    dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
    recordCommunicationEvent(true);
    dmc_pmove_unit(0, static_cast<WORD>(hardwareAxis), motorHomePos[index], 1);
    recordCommunicationEvent(true);
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
    activePvtTimeStamp.clear();

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
    recordCommunicationEvent(false);
    if (doneState == 0) {
        const short stopErr = dmc_stop(0, static_cast<WORD>(hardwareAxis), 0);
        recordCommunicationEvent(true);
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
            recordCommunicationEvent(false);
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
        recordCommunicationEvent(true);
        const short profileErr = dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, safeVel, 0.1, 0.1, 0);
        recordCommunicationEvent(true);
        const short sErr = dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
        recordCommunicationEvent(true);
        const short moveErr = dmc_pmove_unit(0, static_cast<WORD>(hardwareAxis), pos, 1);
        recordCommunicationEvent(true);
        if (clearStopReasonErr != 0 || profileErr != 0 || sErr != 0 || moveErr != 0) {
            emit displayInfoSignal(QString("电机轴%1点位运动启动失败，错误码 clear=%2 profile=%3 s=%4 move=%5")
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
    if(!readMotorPositionTraceCached(index, currentPos)){
        emit displayInfoSignal(QString("Trace feedback position read failed for motor axis %1.")
                                   .arg(index)
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
        recordCommunicationEvent(false);
        dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, std::max(std::fabs(vel), 1e-5), 0.1, 0.1, 0);
        recordCommunicationEvent(true);
        dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
        recordCommunicationEvent(true);
        dmc_vmove(0, static_cast<WORD>(hardwareAxis), vel >= 0 ? 1 : 0);
        recordCommunicationEvent(true);
    } else {
        dmc_change_speed_unit(0, static_cast<WORD>(hardwareAxis), vel, velChangeSpd);
        recordCommunicationEvent(true);
    }
    return true;
    });
}
//到达目标位置后切换到指定速度
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

    const int torqueRaw = leadshineTorqueNmToRaw(torqueNm);
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

    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtTimeStamp.clear();

    const short torqueErr = nmc_torque_move(0,
                                            static_cast<WORD>(hardwareAxis),
                                            torqueRaw,
                                            kTorquePositionLimitDisabled,
                                            kTorquePositionLimitValueUnused,
                                            kTorqueAbsolutePositionMode);
    recordCommunicationEvent(true);
    if(torqueErr != 0){
        emit displayInfoSignal(QString("错误：电机轴%1转矩模式启动失败，错误码%2")
                                   .arg(index)
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

    const int torqueRaw = leadshineTorqueNmToRaw(torqueNm);
    if(torqueRaw == 0){
        return motorStop(index);
    }

    const int hardwareAxis = resolveLeadshineAxisIndex(index);
    if (hardwareAxis < 0) {
        emit displayInfoSignal(QString("%1 has no valid controller axis.")
                                   .arg(axisDisplayName(index))
                                   .toStdString(), "error");
        return false;
    }

    const short torqueErr = nmc_change_torque(0, static_cast<WORD>(hardwareAxis), torqueRaw);
    recordCommunicationEvent(true);
    if(torqueErr != 0){
        emit displayInfoSignal(QString("错误：电机轴%1在线调整转矩失败，错误码%2")
                                   .arg(index)
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
    if(!readMotorPositionTraceCached(index, pos)){
        emit displayInfoSignal(QString("Trace feedback position read failed for motor axis %1.")
                                   .arg(index)
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
        recordCommunicationEvent(false);
        if (crossed) {
            if (std::fabs(stopVel) < 1e-5) {
                return motorStop(index);
            }
            dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, std::max(std::fabs(stopVel), 1e-5), 0.1, 0.1, 0);
            recordCommunicationEvent(true);
        } else {
            dmc_set_profile_unit(0, static_cast<WORD>(hardwareAxis), 0.0, std::max(std::fabs(vel), 1e-5), 0.1, 0.1, 0);
            recordCommunicationEvent(true);
        }

        dmc_set_s_profile(0, static_cast<WORD>(hardwareAxis), 0, 0);
        recordCommunicationEvent(true);
        dmc_vmove(0, static_cast<WORD>(hardwareAxis), vel >= 0 ? 1 : 0);
        recordCommunicationEvent(true);
    } else {
        if (crossed) {
            if (std::fabs(stopVel) < 1e-5) {
                return motorStop(index);
            }
            dmc_change_speed_unit(0, static_cast<WORD>(hardwareAxis), stopVel, velChangeSpd);
            recordCommunicationEvent(true);
        } else {
            dmc_change_speed_unit(0, static_cast<WORD>(hardwareAxis), vel, velChangeSpd);
            recordCommunicationEvent(true);
        }
    }
    return true;
    });
}

double HardwareInterface::readMotorCurPos(int index) {
    return runOnHardwareThread([&]() -> double {
    double result = 0.0;
    if (index >= 0 && index < static_cast<int>(motorComType.size()) && motorComType[index] == COM_EC_LS) {
        readMotorPositionTraceCached(index, result);
    }
    return result;
    });
}

bool HardwareInterface::readMotorRelativeCurPos(int index, double& relativePosition)
{
    double commandRelativePosition = 0.0;
    return readMotorRelativeTracePositions(index, commandRelativePosition, relativePosition);
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
    if(motorCommandPos.size() != motorIdVec.size()){
        motorCommandPos.assign(motorIdVec.size(), 0.0);
    }

    const int frameCount = readRuntimeTraceCached();
    if(frameCount < 0 && !runtimeTraceEverRead){
        return false;
    }
    if(index >= static_cast<int>(motorCurPos.size()) ||
            index >= static_cast<int>(motorCommandPos.size()) ||
            !std::isfinite(motorCurPos[index]) ||
            !std::isfinite(motorCommandPos[index])){
        return false;
    }

    commandRelativePosition = relativeMotorPosition(index, motorCommandPos[index]);
    feedbackRelativePosition = relativeMotorPosition(index, motorCurPos[index]);
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
    recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
    }
    runtimeTraceConfigured = false;
    runtimeTraceUnavailable = false;
    runtimeTraceEverRead = false;
    runtimeTraceConsecutiveFailures = 0;
    runtimeTraceObjectTotalBytes = 0;
    runtimeTraceObjectTotalNum = 0;
    runtimeTraceLastRetryUs = 0;
    runtimeTraceObjects.clear();
    motorCommandPositionTraceObjects.clear();
    motorPositionTraceObjects.clear();
    forceSensorTraceObjects.clear();
    motorTracePositionSampleQueues.clear();
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

    const double axisEquiv = resolveLeadshineAxisEquiv(logicalIndex);
    motorCommandPos[logicalIndex] = axisEquiv > 0.0 ?
                static_cast<double>(rawValue) / axisEquiv :
                static_cast<double>(rawValue);
}

void HardwareInterface::applyMotorPositionTraceRawValue(int logicalIndex, long rawValue)
{
    if(logicalIndex < 0 || logicalIndex >= static_cast<int>(motorComType.size())){
        return;
    }
    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    if(logicalIndex >= static_cast<int>(motorCurPos.size())){
        return;
    }

    const double axisEquiv = resolveLeadshineAxisEquiv(logicalIndex);
    motorCurPos[logicalIndex] = axisEquiv > 0.0 ?
                static_cast<double>(rawValue) / axisEquiv :
                static_cast<double>(rawValue);
}

std::vector<double> HardwareInterface::currentMotorPositionCachedValues(const std::vector<int>& motorIndex) const
{
    std::vector<double> result;
    result.reserve(motorIndex.size());
    for(const int axis : motorIndex){
        if(axis < 0 ||
                axis >= static_cast<int>(motorComType.size()) ||
                axis >= static_cast<int>(motorCurPos.size()) ||
                motorComType[axis] != COM_EC_LS){
            return {};
        }
        result.push_back(motorCurPos[axis]);
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

    runtimeTraceObjects.clear();
    motorCommandPositionTraceObjects.clear();
    motorPositionTraceObjects.clear();

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
        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        if(hardwareAxis < 0){
            continue;
        }
        traceAxes.push_back({i, hardwareAxis});
    }

    for(const RuntimeTraceAxis& axis : traceAxes){
        MotorCommandPositionTraceObject object;
        object.logicalAxis = axis.logicalAxis;
        object.hardwareAxis = axis.hardwareAxis;
        object.dataType = kLeadshineTraceDataTypeCommandPosition;
        object.dataIndex = axis.hardwareAxis;
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
        MotorPositionTraceObject object;
        object.logicalAxis = axis.logicalAxis;
        object.hardwareAxis = axis.hardwareAxis;
        object.dataType = kLeadshineTraceDataTypeActualPosition;
        object.dataIndex = axis.hardwareAxis;
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

    forceSensorTraceObjects.clear();
    if(forceSensorTraceReadEnabled){
        struct DefaultTraceMap {
            int sensorIndex = -1;
            short slaveId = 0;
            int subIndex = 0;
        };
        const DefaultTraceMap traceMap[] = {
            {0, 1004, 1},
            {1, 1004, 2},
            {2, 1004, 3},
            {3, 1004, 4},
            {4, 1011, 1},
            {5, 1011, 2},
            {6, 1011, 3},
            {7, 1011, 4}
        };

        for(const DefaultTraceMap& map : traceMap){
            if(map.sensorIndex < 0 ||
                    map.sensorIndex >= static_cast<int>(sensorComType.size()) ||
                    sensorComType[map.sensorIndex] != COM_EC_LS_SBT){
                continue;
            }
            ForceSensorTraceObject object;
            object.sensorIndex = map.sensorIndex;
            object.dataType = 19;
            object.dataIndex = 0x6000;
            object.dataSubIndex = map.subIndex;
            object.slaveId = map.slaveId;
            object.apiDataBytes = 0;
            object.valueBytes = 2;
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

    short ret = dmc_trace_data_stop(0);
    recordCommunicationEvent(false);
    ret = dmc_trace_data_reset(0);
    recordCommunicationEvent(false);
    if(ret != 0){
        runtimeTraceUnavailable = true;
        return false;
    }

    int tracePeriodUs = motorPositionTraceObjects.empty() ?
                forceSensorTraceSamplePeriodUs :
                motorPositionTraceSamplePeriodUs;
    if(!motorPositionTraceObjects.empty() && !forceSensorTraceObjects.empty()){
        tracePeriodUs = std::min(motorPositionTraceSamplePeriodUs, forceSensorTraceSamplePeriodUs);
    }
    const int traceCycle = std::max(1, tracePeriodUs / kForceSensorTraceBaseCycleUs);
    ret = dmc_trace_set_config(0,
                               static_cast<short>(traceCycle),
                               0,
                               0,
                               0,
                               0,
                               0,
                               0);
    recordCommunicationEvent(false);
    if(ret != 0){
        runtimeTraceUnavailable = true;
        return false;
    }

    ret = dmc_trace_reset_config_object(0);
    recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
        if(ret != 0){
            dmc_trace_data_stop(0);
            recordCommunicationEvent(false);
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
    recordCommunicationEvent(false);
    if(ret != 0){
        runtimeTraceUnavailable = true;
        return false;
    }

    runtimeTraceConfigured = true;
    runtimeTraceEverRead = false;
    runtimeTraceConsecutiveFailures = 0;
    runtimeTraceObjectTotalBytes = 0;
    runtimeTraceObjectTotalNum = 0;
    runtimeTraceLastRetryUs = 0;
    return true;
}

int HardwareInterface::readRuntimeTraceCached()
{
    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
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
    Q_UNUSED(freeNum);
    recordCommunicationEvent(false);
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
    if(validNum <= 0){
        ++runtimeTraceConsecutiveFailures;
        return runtimeTraceEverRead ? 0 : -1;
    }
    runtimeTraceConsecutiveFailures = 0;

    int expectedValueBytes = 0;
    int maxValueBytes = 0;
    for(const RuntimeTraceObject& object : runtimeTraceObjects){
        const int valueBytes = std::max(1, object.valueBytes);
        expectedValueBytes += valueBytes;
        maxValueBytes = std::max(maxValueBytes, valueBytes);
    }
    const int frameBytes = std::max(expectedValueBytes, objectTotalBytes);
    if(frameBytes <= 0){
        return runtimeTraceEverRead ? 0 : -1;
    }

    int objectValueStartOffset = 0;
    int objectStepBytes = 0;
    const int configuredObjectCount = static_cast<int>(runtimeTraceObjects.size());
    if(objectTotalBytes >= expectedValueBytes && configuredObjectCount > 0){
        const int candidateSlotBytes[] = {8, 4, 2, 1};
        int bestHeaderBytes = std::numeric_limits<int>::max();
        int bestSlotBytes = 0;
        for(const int slotBytes : candidateSlotBytes){
            if(slotBytes < maxValueBytes){
                continue;
            }
            const int objectBytes = slotBytes * configuredObjectCount;
            if(objectBytes > objectTotalBytes){
                continue;
            }
            const int headerBytes = objectTotalBytes - objectBytes;
            if(headerBytes < bestHeaderBytes){
                bestHeaderBytes = headerBytes;
                bestSlotBytes = slotBytes;
            }
        }

        if(bestSlotBytes > 0){
            objectValueStartOffset = bestHeaderBytes;
            objectStepBytes = bestSlotBytes;
        }
        else{
            objectValueStartOffset = objectTotalBytes - expectedValueBytes;
        }
    }

    int totalCompleteFrameCount = 0;
    int currentValidNum = validNum;
    constexpr int kMaxTraceDrainReads = 16;
    for(int drainIndex = 0; drainIndex < kMaxTraceDrainReads && currentValidNum > 0; ++drainIndex){
        const int maxBufferBytes = 64 * 1024;
        const int frameCapacity = std::max(1, maxBufferBytes / frameBytes);
        const int frameCountToRead = std::max(1, std::min(currentValidNum, frameCapacity));
        const int bufferSize = std::max(frameBytes, frameBytes * frameCountToRead);
        std::vector<unsigned char> buffer(static_cast<std::size_t>(bufferSize), 0);
        int actualReadLength = 0;
        const short dataRet = dmc_trace_get_data(0,
                                                 bufferSize,
                                                 buffer.data(),
                                                 &actualReadLength);
        recordCommunicationEvent(false);
        if(dataRet != 0 || actualReadLength < expectedValueBytes){
            return runtimeTraceEverRead || totalCompleteFrameCount > 0 ? totalCompleteFrameCount : -1;
        }

        const int completeFrameCount = actualReadLength / frameBytes;
        if(completeFrameCount <= 0){
            return runtimeTraceEverRead || totalCompleteFrameCount > 0 ? totalCompleteFrameCount : -1;
        }

        for(int frameIndex = 0; frameIndex < completeFrameCount; ++frameIndex){
            const int frameOffset = frameIndex * frameBytes;
            if(frameOffset + objectValueStartOffset + expectedValueBytes > actualReadLength){
                return runtimeTraceEverRead || totalCompleteFrameCount > 0 ? totalCompleteFrameCount : -1;
            }

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
                        applyMotorCommandPositionTraceRawValue(
                                    motorCommandPositionTraceObjects[object.objectIndex].logicalAxis,
                                    rawValue);
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::MotorPosition){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(motorPositionTraceObjects.size())){
                        applyMotorPositionTraceRawValue(motorPositionTraceObjects[object.objectIndex].logicalAxis,
                                                        rawValue);
                    }
                }
                else if(object.kind == RuntimeTraceObjectKind::ForceSensor){
                    if(object.objectIndex >= 0 &&
                            object.objectIndex < static_cast<int>(forceSensorTraceObjects.size())){
                        applyForceSensorRawValue(forceSensorTraceObjects[object.objectIndex].sensorIndex,
                                                 rawValue);
                    }
                }
                objectOffset += objectStepBytes >= valueBytes ? objectStepBytes : valueBytes;
            }

            if(!motorCommandPositionTraceObjects.empty() && !motorPositionTraceObjects.empty()){
                for(const MotorPositionTraceObject& object : motorPositionTraceObjects){
                    const int logicalAxis = object.logicalAxis;
                    if(logicalAxis < 0 ||
                            logicalAxis >= static_cast<int>(motorComType.size()) ||
                            logicalAxis >= static_cast<int>(motorCommandPos.size()) ||
                            logicalAxis >= static_cast<int>(motorCurPos.size()) ||
                            logicalAxis >= static_cast<int>(motorTracePositionSampleQueues.size()) ||
                            motorComType[logicalAxis] != COM_EC_LS ||
                            !std::isfinite(motorCommandPos[logicalAxis]) ||
                            !std::isfinite(motorCurPos[logicalAxis])){
                        continue;
                    }

                    MotorTracePositionSample sample;
                    sample.commandRelativePosition =
                            relativeMotorPosition(logicalAxis, motorCommandPos[logicalAxis]);
                    sample.feedbackRelativePosition =
                            relativeMotorPosition(logicalAxis, motorCurPos[logicalAxis]);
                    auto& samples = motorTracePositionSampleQueues[logicalAxis];
                    samples.push_back(sample);
                    while(samples.size() > kMaxMotorTracePositionSamplesPerAxis){
                        samples.pop_front();
                    }
                }
            }
            if(!forceSensorTraceObjects.empty()){
                ForceSensorTraceSample sample;
                sample.values = currentForceSensorCachedValues();
                if(!sample.values.empty()){
                    forceSensorTraceSampleQueue.push_back(std::move(sample));
                    while(forceSensorTraceSampleQueue.size() > kMaxForceSensorTraceSamples){
                        forceSensorTraceSampleQueue.pop_front();
                    }
                }
            }
        }

        totalCompleteFrameCount += completeFrameCount;

        int nextValidNum = 0;
        int nextFreeNum = 0;
        int nextObjectTotalBytes = 0;
        int nextObjectTotalNum = 0;
        const short nextStateRet = dmc_trace_get_state(0,
                                                       &nextValidNum,
                                                       &nextFreeNum,
                                                       &nextObjectTotalBytes,
                                                       &nextObjectTotalNum);
        Q_UNUSED(nextFreeNum);
        Q_UNUSED(nextObjectTotalBytes);
        Q_UNUSED(nextObjectTotalNum);
        recordCommunicationEvent(false);
        if(nextStateRet != 0 || nextValidNum <= 0 || completeFrameCount < frameCountToRead){
            break;
        }
        currentValidNum = nextValidNum;
    }

    if(totalCompleteFrameCount <= 0){
        return runtimeTraceEverRead ? 0 : -1;
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

void HardwareInterface::applyForceSensorRawValue(int sensorIndex, long rawValue)
{
    if(sensorIndex < 0 || sensorIndex >= static_cast<int>(sensorComType.size())){
        return;
    }
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
        nextForceSensorPollIndex = 0;
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
        recordCommunicationEvent(false);
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
    if(!forceSensorTraceReadEnabled){
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
    if(!forceSensorTraceReadEnabled){
        return {};
    }
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
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
        nextForceSensorPollIndex = 0;
    }
    for (int i = 0; i < static_cast<int>(sensorComType.size()); ++i) {
        if (sensorComType[i] == COM_EC_LS_SBT) {
            int tmpValue = 0;
            const short readRet = readForceSensorTxpdoExtra(i, &tmpValue);
            recordCommunicationEvent(false);
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
    if(!forceSensorTraceReadEnabled){
        return {};
    }
    if(forceSensorCachedValue.size() != sensorComType.size()){
        forceSensorCachedValue.assign(sensorComType.size(), 0.0);
        forceSensorCacheValid.assign(sensorComType.size(), false);
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
    if(!traceResult.values.empty()){
        return traceResult;
    }
    return readForceSensorDataCachedDirectResult(maxChannelsToRead);
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
    recordCommunicationEvent(false);
    result.clearBeforeStartResult = nmc_clear_pdo_trace_data(0, request.channel, request.slaveAddress);
    recordCommunicationEvent(false);
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
    recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
        result.stateResult = nmc_get_pdo_trace_state(0,
                                                     request.channel,
                                                     request.slaveAddress,
                                                     &result.traceState);
        recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
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
    recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
        if(result.setSourceResult != 0){
            result.messages << QStringLiteral("dmc_trace_set_source returned %1.")
                               .arg(result.setSourceResult);
        }
    }

    result.stopBeforeConfigResult = dmc_trace_data_stop(0);
    recordCommunicationEvent(false);
    result.dataResetBeforeConfigResult = dmc_trace_data_reset(0);
    recordCommunicationEvent(false);

    result.setConfigResult = dmc_trace_set_config(0,
                                                  request.traceCycle,
                                                  request.lostHandle,
                                                  request.traceType,
                                                  request.triggerObjectIndex,
                                                  request.triggerType,
                                                  request.mask,
                                                  request.condition);
    recordCommunicationEvent(false);
    if(result.setConfigResult != 0){
        result.messages << QStringLiteral("dmc_trace_set_config returned %1.")
                           .arg(result.setConfigResult);
        return result;
    }

    result.resetConfigObjectResult = dmc_trace_reset_config_object(0);
    recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
        if(result.addConfigObjectResult != 0){
            result.messages << QStringLiteral("dmc_trace_add_config_object returned %1.")
                               .arg(result.addConfigObjectResult);
            return result;
        }
    }

    result.dataStartResult = dmc_trace_data_start(0);
    recordCommunicationEvent(false);
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
        recordCommunicationEvent(false);
        result.getStateResult = dmc_trace_get_state(0,
                                                    &result.validNum,
                                                    &result.freeNum,
                                                    &result.objectTotalBytes,
                                                    &result.objectTotalNum);
        recordCommunicationEvent(false);
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
    recordCommunicationEvent(false);
    if(result.getDataResult == 0 && result.actualReadLength > 0){
        const int byteCount = std::min(result.actualReadLength, readLength);
        result.data = QByteArray(reinterpret_cast<const char*>(buffer.data()), byteCount);
    }
    else if(result.getDataResult != 0){
        result.messages << QStringLiteral("dmc_trace_get_data returned %1.")
                           .arg(result.getDataResult);
    }

    result.dataStopResult = dmc_trace_data_stop(0);
    recordCommunicationEvent(false);
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

    uint32 masterState = 0;
    WORD cardErrCode = 0;
    const short masterRet = nmc_get_master_state(0, &masterState);
    const short cardErrRet = nmc_get_card_errcode(0, &cardErrCode);
    snapshot.controller.busState = static_cast<int>(masterState);
    snapshot.controller.errorCode = static_cast<int>(cardErrCode);
    snapshot.controller.apiResult = masterRet != 0 ? masterRet : cardErrRet;
    if(masterRet != 0 || cardErrRet != 0 || cardErrCode != 0){
        snapshot.controller.state = ConnectionState::Fault;
    }
    else if(masterState == 8U){
        snapshot.controller.state = ConnectionState::Connected;
    }
    else if(masterState == 0U){
        snapshot.controller.state = ConnectionState::Disconnected;
    }
    else{
        snapshot.controller.state = ConnectionState::Fault;
    }

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

    for(int i = 0; i < static_cast<int>(sensorComType.size()); ++i){
        ConnectionItemDiagnostics& sensor = snapshot.forceSensors[i];
        if(sensorComType[i] != COM_EC_LS_SBT){
            continue;
        }

        int rawValue = 0;
        const short readRet = readForceSensorTxpdoExtra(i, &rawValue);
        sensor.apiResult = readRet;
        sensor.state = readRet == 0 ? ConnectionState::Connected : ConnectionState::Fault;
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
    lastCommunicationHistoryTrimMs = 0;
    lastMotorCommandHistoryTrimMs = 0;
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
    return runOnHardwareThread([&]() -> bool {
    if(motorCurState.size() != motorIdVec.size()){
        motorCurState.assign(motorIdVec.size(), false);
    }
    return refreshLeadshineMotorEnableState(index);
    });
}

std::vector<double> HardwareInterface::getAllMotorPos() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    std::vector<int> leadshineAxes;
    leadshineAxes.reserve(motorIdVec.size());
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (i < static_cast<int>(motorComType.size()) && motorComType[i] == COM_EC_LS) {
            leadshineAxes.push_back(i);
        }
    }
    readMotorPositionsTraceCached(leadshineAxes);
    return motorCurPos;
    });
}

std::vector<double> HardwareInterface::getAllMotorVel() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    for (int i = 0; i < static_cast<int>(motorIdVec.size()); ++i) {
        if (motorComType[i] == COM_EC_LS) {
            const int hardwareAxis = resolveLeadshineAxisIndex(i);
            if (hardwareAxis >= 0) {
                dmc_read_current_speed_unit(0, static_cast<WORD>(hardwareAxis), &motorCurVel[i]);
                recordCommunicationEvent(false);
            }
        }
    }
    return motorCurVel;
    });
}

std::vector<double> HardwareInterface::getAllMotorTorqueNm() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    std::vector<double> torqueNm(motorIdVec.size(), std::numeric_limits<double>::quiet_NaN());
    if(!isConnectLS){
        return torqueNm;
    }

    const int axisConfigCount = std::min(static_cast<int>(motorIdVec.size()),
                                         static_cast<int>(motorComType.size()));
    for(int i = 0; i < axisConfigCount; ++i){
        if(motorComType[i] != COM_EC_LS){
            continue;
        }

        const int hardwareAxis = resolveLeadshineAxisIndex(i);
        if(hardwareAxis < 0){
            continue;
        }

        int torque = 0;
        const short ret = nmc_get_torque(0, static_cast<WORD>(hardwareAxis), &torque);
        recordCommunicationEvent(false);
        if(ret == 0){
            // nmc_get_torque returns thousandths of rated torque; expose N*m to callers.
            torqueNm[i] = static_cast<double>(torque) *
                    kLeadshineRatedMotorTorqueNm /
                    kLeadshineTorqueFeedbackRawPerRatedTorque;
        }
    }

    return torqueNm;
    });
}

std::vector<double> HardwareInterface::getAllMotorHome() {
    return runOnHardwareThread([&]() -> std::vector<double> {
    return motorHomePos;
    });
}

void HardwareInterface::motorPosTraj(std::vector<int> motorIndex, std::vector<std::vector<double>> motorPosTraj,
                                     std::vector<std::vector<double>> motorVel, std::vector<double> timeStamp) {
    return runOnHardwareThread([&]() {
    hasActivePvtTrajectory = false;
    isPvtMotionPaused = false;
    pausedPvtResumeIndex = 0;
    pausedPvtResumeTime = 0.0;
    activePvtMotorIndex.clear();
    activePvtMotorPosTraj.clear();
    activePvtMotorVelTraj.clear();
    activePvtTimeStamp.clear();
    const std::vector<double> beginVel(motorIndex.size(), 0.0);
    const std::vector<double> endVel(motorIndex.size(), 0.0);
    startPvtTable(motorIndex,
                  motorPosTraj,
                  motorVel,
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
