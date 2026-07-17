#include "motortorquetestworker.h"

#include "hardwareinterface.h"

#include <QMutexLocker>
#include <QStringList>
#include <QTimer>

#include <cmath>
#include <limits>
#include <vector>

namespace {

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
                                      double velocityLimit)
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
        }

        config.axisIndex = axisIndex;
        config.targetTorque = targetTorque;
        config.relativeMinPos = relativeMinPos;
        config.relativeMaxPos = relativeMaxPos;
        config.velocityLimit = velocityLimit;
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

    const std::vector<double> motorAbsPos = hardwareInterface->getAllMotorPos();
    const std::vector<double> motorHome = hardwareInterface->getAllMotorHome();
    const std::vector<double> motorVel = hardwareInterface->getAllMotorVel();
    const std::vector<double> motorTorque = hardwareInterface->getAllMotorTorqueNm();
    if(cfg.axisIndex >= 0 && cfg.axisIndex < static_cast<int>(motorAbsPos.size())){
        relativePosition = motorAbsPos[cfg.axisIndex];
        if(cfg.axisIndex < static_cast<int>(motorHome.size())){
            relativePosition -= motorHome[cfg.axisIndex];
        }
    }
    if(cfg.axisIndex >= 0 && cfg.axisIndex < static_cast<int>(motorVel.size())){
        actualVelocity = motorVel[cfg.axisIndex];
    }
    if(cfg.axisIndex >= 0 && cfg.axisIndex < static_cast<int>(motorTorque.size())){
        actualTorque = motorTorque[cfg.axisIndex];
    }

    emit statusUpdated(cfg.axisIndex, relativePosition, actualTorque, actualVelocity, cfg.active);

    if(!cfg.active){
        return;
    }
    if(cfg.axisIndex < 0 || cfg.axisIndex >= static_cast<int>(motorAbsPos.size())){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：电机轴号无效"), "error");
        return;
    }
    if(!std::isfinite(relativePosition)){
        stopTorqueMotion(QStringLiteral("转矩模式调试已停止：当前位置反馈无效"), "error");
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
            stopTorqueMotion(QStringLiteral("转矩模式调试已停止：电机轴%1退出使能或驱动状态异常。%2")
                             .arg(cfg.axisIndex)
                             .arg(formatTorqueAxisDiagnostics(diagnostics)),
                             "error");
            return;
        }
    }

    ensureTorqueCommand(cfg);
}
