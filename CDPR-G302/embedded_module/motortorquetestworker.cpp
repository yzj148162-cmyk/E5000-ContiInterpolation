/*
 * 文件总览：
 * - MotorTorqueTestWorker 的实现文件，定时读取单轴反馈并在边界内维持目标力矩命令。
 * - 一旦位置、速度或硬件状态不满足条件，会主动停止力矩运动并向 UI 报告原因。
 */

#include "motortorquetestworker.h"

#include "hardwareinterface.h"

#include <QMutexLocker>
#include <QStringList>
#include <QTimer>

#include <cmath>
#include <limits>

namespace {

constexpr double kTorqueRecoveryVelocityEpsilon = 1e-6;
constexpr double kTorqueRecoveryInsideGuardUnit = 0.01;

QString torqueMotorDisplayName(int axisIndex)
{
    if(axisIndex >= 0 && axisIndex < 8){
        return QStringLiteral("绳索电机%1").arg(axisIndex + 1);
    }
    if(axisIndex >= 8 && axisIndex < 12){
        return QStringLiteral("直线模组电机%1").arg(axisIndex - 7);
    }
    return QStringLiteral("电机%1").arg(axisIndex + 1);
}

QString torqueConnectionStateText(HardwareInterface::ConnectionState state)
{
    switch(state){
    case HardwareInterface::ConnectionState::Connected:
        return QStringLiteral("已连接/已使能");
    case HardwareInterface::ConnectionState::Disabled:
        return QStringLiteral("已连接/未使能");
    case HardwareInterface::ConnectionState::Fault:
        return QStringLiteral("故障");
    case HardwareInterface::ConnectionState::Disconnected:
    default:
        return QStringLiteral("未连接");
    }
}

QString torqueStatusWordFlags(long statusWord)
{
    const unsigned long word = static_cast<unsigned long>(statusWord);
    if(word == 0){
        return QStringLiteral("状态字为0");
    }

    QStringList flags;
    if(word & (1UL << 3)){
        flags << QStringLiteral("故障");
    }
    if((word & (1UL << 5)) == 0){
        flags << QStringLiteral("快速停止有效");
    }
    if(word & (1UL << 6)){
        flags << QStringLiteral("禁止上电");
    }
    if(word & (1UL << 7)){
        flags << QStringLiteral("警告");
    }
    if(word & (1UL << 10)){
        flags << QStringLiteral("目标到达");
    }
    if(word & (1UL << 11)){
        flags << QStringLiteral("内部限位");
    }
    if(flags.isEmpty()){
        return QStringLiteral("无明显停机标志");
    }
    return flags.join(QStringLiteral("/"));
}

QString formatTorqueAxisDiagnostics(const HardwareInterface::ConnectionItemDiagnostics& diagnostics)
{
    const QString statusWordHex = QStringLiteral("0x%1")
            .arg(static_cast<qulonglong>(static_cast<unsigned long>(diagnostics.statusWord)), 0, 16)
            .toUpper();
    QString detail = QStringLiteral("诊断：状态=%1，控制卡轴=%2，状态机=%3，状态字=%4(%5)，轴错误码=%6，停止原因=%7")
            .arg(torqueConnectionStateText(diagnostics.state))
            .arg(diagnostics.hardwareAxis)
            .arg(diagnostics.stateMachine)
            .arg(statusWordHex)
            .arg(torqueStatusWordFlags(diagnostics.statusWord))
            .arg(diagnostics.errorCode)
            .arg(diagnostics.stopReason);

    if(diagnostics.stopReasonApiResult != 0){
        detail += QStringLiteral("，停止原因读取返回=%1").arg(diagnostics.stopReasonApiResult);
    }
    if(diagnostics.slaveAddress > 0){
        detail += QStringLiteral("，从站地址=%1").arg(diagnostics.slaveAddress);
    }
    if(diagnostics.busState >= 0){
        detail += QStringLiteral("，从站状态=%1").arg(diagnostics.busState);
    }
    if(diagnostics.apiResult != 0){
        detail += QStringLiteral("，API返回=%1").arg(diagnostics.apiResult);
    }
    return detail;
}

}

MotorTorqueTestWorker::MotorTorqueTestWorker(HardwareInterface* hardware, QObject* parent)
    : QObject(parent),
      hardwareInterface(hardware)
{
}

void MotorTorqueTestWorker::start()
{
    if(timer){
        timer->start();
        return;
    }

    timer = new QTimer(this);
    timer->setTimerType(Qt::PreciseTimer);
    timer->setInterval(50);
    connect(timer, &QTimer::timeout, this, &MotorTorqueTestWorker::loop);
    timer->start();
}

void MotorTorqueTestWorker::stop()
{
    requestStopMotion();
    if(timer){
        timer->stop();
    }
}

int MotorTorqueTestWorker::torqueSign(double torque)
{
    if(torque > 0){
        return 1;
    }
    if(torque < 0){
        return -1;
    }
    return 0;
}

void MotorTorqueTestWorker::setConfig(int axisIndex,
                                      double targetTorque,
                                      double relativeMinPos,
                                      double relativeMaxPos,
                                      double velocityLimit,
                                      bool allowMoveOutsideSoftwareLimit)
{
    int stopAxis = -1;
    {
        QMutexLocker locker(&configMutex);
        const bool commandLimitChanged =
                axisIndex != config.axisIndex ||
                torqueSign(targetTorque) != torqueSign(config.targetTorque);
        if(config.active && commandStarted && commandLimitChanged){
            stopAxis = lastCommandAxis;
            commandStarted = false;
            lastCommandAxis = -1;
            lastCommandTorque = 0.0;
            recoveryFromSoftwareLimitActive = false;
        }

        config.axisIndex = axisIndex;
        config.targetTorque = targetTorque;
        config.relativeMinPos = relativeMinPos;
        config.relativeMaxPos = relativeMaxPos;
        config.velocityLimit = velocityLimit;
        config.allowMoveOutsideSoftwareLimit = allowMoveOutsideSoftwareLimit;
    }

    if(stopAxis >= 0 && hardwareInterface){
        hardwareInterface->motorStop(stopAxis);
    }
}

void MotorTorqueTestWorker::setTorqueActive(bool active)
{
    int stopAxis = -1;
    bool changed = false;
    {
        QMutexLocker locker(&configMutex);
        if(config.active == active){
            return;
        }
        config.active = active;
        changed = true;
        if(!active && commandStarted){
            stopAxis = lastCommandAxis;
            commandStarted = false;
            lastCommandAxis = -1;
            lastCommandTorque = 0.0;
        }
        if(!active){
            recoveryFromSoftwareLimitActive = false;
        }
    }

    if(stopAxis >= 0 && hardwareInterface){
        hardwareInterface->motorStop(stopAxis);
    }
    if(changed){
        emit activeChanged(active);
    }
}

void MotorTorqueTestWorker::requestStopMotion()
{
    stopTorqueMotion(QString(), "normal");
}

MotorTorqueTestWorker::Config MotorTorqueTestWorker::currentConfig() const
{
    QMutexLocker locker(&configMutex);
    return config;
}

void MotorTorqueTestWorker::stopTorqueMotion(const QString& reason, const std::string& type)
{
    int stopAxis = -1;
    bool wasActive = false;
    {
        QMutexLocker locker(&configMutex);
        wasActive = config.active;
        config.active = false;
        if(commandStarted){
            stopAxis = lastCommandAxis;
            commandStarted = false;
            lastCommandAxis = -1;
            lastCommandTorque = 0.0;
        }
        recoveryFromSoftwareLimitActive = false;
    }

    if(stopAxis >= 0 && hardwareInterface){
        hardwareInterface->motorStop(stopAxis);
    }
    if(!reason.isEmpty()){
        emit displayInfoSignal(reason.toStdString(), type);
    }
    if(wasActive){
        emit activeChanged(false);
    }
}

bool MotorTorqueTestWorker::ensureTorqueCommand(const Config& cfg)
{
    if(!hardwareInterface){
        return false;
    }

    if(!std::isfinite(cfg.targetTorque) || cfg.targetTorque == 0.0){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：目标转矩为0"), "warning");
        return false;
    }

    bool shouldStart = false;
    bool shouldChange = false;
    {
        QMutexLocker locker(&configMutex);
        shouldStart = !commandStarted ||
                lastCommandAxis != cfg.axisIndex ||
                torqueSign(lastCommandTorque) != torqueSign(cfg.targetTorque);
        shouldChange = commandStarted &&
                !shouldStart &&
                std::fabs(lastCommandTorque - cfg.targetTorque) > 1e-9;
    }

    if(shouldStart){
        if(!hardwareInterface->motorTorqueStart(cfg.axisIndex,
                                                cfg.targetTorque)){
            stopTorqueMotion(QStringLiteral("转矩模式调试已停止：转矩模式启动失败"), "error");
            return false;
        }
        QMutexLocker locker(&configMutex);
        commandStarted = true;
        lastCommandAxis = cfg.axisIndex;
        lastCommandTorque = cfg.targetTorque;
        return true;
    }

    if(shouldChange){
        if(!hardwareInterface->motorTorqueChange(cfg.axisIndex, cfg.targetTorque)){
            stopTorqueMotion(QStringLiteral("转矩模式调试已停止：在线调整目标转矩失败"), "error");
            return false;
        }
        QMutexLocker locker(&configMutex);
        lastCommandTorque = cfg.targetTorque;
    }
    return true;
}

void MotorTorqueTestWorker::loop()
{
    const Config cfg = currentConfig();
    const double invalidValue = std::numeric_limits<double>::quiet_NaN();
    double relativePosition = invalidValue;
    double actualVelocity = invalidValue;
    double actualTorque = invalidValue;

    if(!hardwareInterface || !hardwareInterface->isLSConnected()){
        emit statusUpdated(cfg.axisIndex, relativePosition, actualTorque, actualVelocity, false);
        if(cfg.active){
            stopTorqueMotion(QStringLiteral("转矩模式调试已停止：雷赛控制卡未连接"), "error");
        }
        return;
    }

    const bool validSelectedAxis = cfg.axisIndex >= 0;
    const bool relativePositionValid = validSelectedAxis &&
            hardwareInterface->readMotorSafetyRelativeCurPos(
                cfg.axisIndex,
                relativePosition);
    const bool actualVelocityValid = validSelectedAxis &&
            hardwareInterface->readMotorCurrentSpeedUnit(
                cfg.axisIndex,
                actualVelocity);
    const bool actualTorqueValid = validSelectedAxis &&
            hardwareInterface->readMotorTorqueNmTraceCached(
                cfg.axisIndex,
                actualTorque);

    emit statusUpdated(cfg.axisIndex, relativePosition, actualTorque, actualVelocity, cfg.active);

    if(!cfg.active){
        return;
    }
    if(!validSelectedAxis){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：电机编号无效"), "error");
        return;
    }
    if(!relativePositionValid || !std::isfinite(relativePosition)){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：当前逻辑轴的 Trace 安全相对位置无效或超时"), "error");
        return;
    }
    if(!actualVelocityValid || !std::isfinite(actualVelocity)){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：当前逻辑轴的速度反馈无效"), "error");
        return;
    }
    if(!actualTorqueValid || !std::isfinite(actualTorque)){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：当前逻辑轴的 Trace 实际转矩反馈无效或超时"), "error");
        return;
    }

    bool commandWasStarted = false;
    {
        QMutexLocker locker(&configMutex);
        commandWasStarted = commandStarted;
    }
    if(commandWasStarted){
        const HardwareInterface::ConnectionItemDiagnostics diagnostics =
                hardwareInterface->motorAxisDiagnostics(cfg.axisIndex);
        if(diagnostics.state != HardwareInterface::ConnectionState::Connected ||
                diagnostics.errorCode != 0){
            stopTorqueMotion(QStringLiteral("转矩模式调试已停止：%1退出使能或驱动状态异常。%2")
                             .arg(torqueMotorDisplayName(cfg.axisIndex))
                             .arg(formatTorqueAxisDiagnostics(diagnostics)),
                             "error");
            return;
        }
    }

    const bool hasPositionLimits =
            std::isfinite(cfg.relativeMinPos) &&
            std::isfinite(cfg.relativeMaxPos) &&
            cfg.relativeMaxPos > cfg.relativeMinPos;
    if(hasPositionLimits){
        const bool aboveLimit = relativePosition > cfg.relativeMaxPos;
        const bool belowLimit = relativePosition < cfg.relativeMinPos;
        if(aboveLimit || belowLimit){
            QMutexLocker locker(&configMutex);
            recoveryFromSoftwareLimitActive = true;
        }

        if(!cfg.allowMoveOutsideSoftwareLimit && std::isfinite(actualVelocity)){
            const bool movingOutUpper =
                    relativePosition >= cfg.relativeMaxPos &&
                    actualVelocity > kTorqueRecoveryVelocityEpsilon;
            const bool movingOutLower =
                    relativePosition <= cfg.relativeMinPos &&
                    actualVelocity < -kTorqueRecoveryVelocityEpsilon;
            if(movingOutUpper || movingOutLower){
                stopTorqueMotion(QStringLiteral("转矩模式调试已停止：安全相对位置仍在向软件限位外运动"), "error");
                return;
            }
        }

        bool shouldStopAfterRecovery = false;
        {
            QMutexLocker locker(&configMutex);
            shouldStopAfterRecovery =
                    recoveryFromSoftwareLimitActive &&
                    relativePosition > cfg.relativeMinPos + kTorqueRecoveryInsideGuardUnit &&
                    relativePosition < cfg.relativeMaxPos - kTorqueRecoveryInsideGuardUnit;
        }
        if(shouldStopAfterRecovery){
            stopTorqueMotion(QStringLiteral("转矩模式恢复已停止：电机已回到软件位置限位内"), "normal");
            return;
        }
    }

    if(std::isfinite(cfg.velocityLimit) &&
            cfg.velocityLimit > 1e-9 &&
            std::isfinite(actualVelocity) &&
            std::fabs(actualVelocity) > cfg.velocityLimit){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：实际速度超过速度上限"), "error");
        return;
    }

    ensureTorqueCommand(cfg);
}
