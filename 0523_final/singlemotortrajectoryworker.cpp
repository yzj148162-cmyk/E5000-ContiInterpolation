#include "singlemotortrajectoryworker.h"

#include <QDateTime>
#include <QTimer>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace {
constexpr int kMaxPvtPointCount = 10000;
constexpr qint64 kTraceReadBasePeriodUs = 500;
constexpr double kTwoPi = 6.28318530717958647692;

qint64 monotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

bool isFinite(double value)
{
    return std::isfinite(value);
}

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

double quinticBlend(double u)
{
    const double x = clamp01(u);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

double quinticBlendDerivative(double u)
{
    const double x = clamp01(u);
    return 30.0 * x * x * (1.0 - x) * (1.0 - x);
}

double triangleWave(double cyclePosition)
{
    const double frac = cyclePosition - std::floor(cyclePosition);
    if(frac < 0.25){
        return 4.0 * frac;
    }
    if(frac < 0.75){
        return 2.0 - 4.0 * frac;
    }
    return -4.0 + 4.0 * frac;
}

double triangleWaveDerivative(double cyclePosition)
{
    const double frac = cyclePosition - std::floor(cyclePosition);
    if(frac < 0.25){
        return 4.0;
    }
    if(frac < 0.75){
        return -4.0;
    }
    return 4.0;
}
}

SingleMotorTrajectoryWorker::SingleMotorTrajectoryWorker(HardwareInterface* hardware, QObject* parent)
    : QObject(parent),
      hardwareInterface(hardware)
{
}

void SingleMotorTrajectoryWorker::start()
{
    nextTraceReadDueUs = monotonicNowUs();
    if(timer){
        timer->start();
        return;
    }

    timer = new QTimer(this);
    timer->setTimerType(Qt::PreciseTimer);
    timer->setInterval(0);
    connect(timer, &QTimer::timeout, this, &SingleMotorTrajectoryWorker::loop);
    timer->start();
}

void SingleMotorTrajectoryWorker::stop()
{
    requestStopTrajectory();
    nextTraceReadDueUs = 0;
    if(timer){
        timer->stop();
    }
}

void SingleMotorTrajectoryWorker::setSelectedAxis(int axisIndex)
{
    selectedAxis = axisIndex;
    double position = 0.0;
    if(readRelativePosition(selectedAxis, position)){
        emit feedbackUpdated(selectedAxis, position);
    }
}

void SingleMotorTrajectoryWorker::startTrajectory(const SingleMotorTrajectoryWorker::Command& command)
{
    if(!hardwareInterface || !hardwareInterface->isLSConnected()){
        finishActiveTrajectory(QStringLiteral("雷赛控制卡未连接"), "error");
        return;
    }
    if(command.axisIndex < 0){
        finishActiveTrajectory(QStringLiteral("电机轴号无效"), "error");
        return;
    }

    selectedAxis = command.axisIndex;
    if(active && activeAxis >= 0){
        hardwareInterface->motorStop(activeAxis);
    }
    active = false;
    activeAxis = -1;
    activeStartMs = 0;
    activeSampleElapsedUs = 0;
    lastDriveFollowingErrorReadMs = -1;
    activeTime.clear();
    activeExpectedPosition.clear();

    double currentPosition = 0.0;
    if(!readRelativePosition(command.axisIndex, currentPosition)){
        finishActiveTrajectory(QStringLiteral("当前电机位置反馈无效"), "error");
        return;
    }

    Command effectiveCommand = command;
    if(effectiveCommand.useCurrentStart){
        effectiveCommand.startPosition = currentPosition;
        if(effectiveCommand.type == TrajectoryType::Line ||
                effectiveCommand.type == TrajectoryType::Step){
            effectiveCommand.centerPosition =
                    0.5 * (effectiveCommand.startPosition + effectiveCommand.endPosition);
        }
        else if(effectiveCommand.type == TrajectoryType::Sine){
            const double phaseRad = effectiveCommand.phaseDeg * kTwoPi / 360.0;
            effectiveCommand.centerPosition =
                    currentPosition - effectiveCommand.amplitude * std::sin(phaseRad);
        }
        else if(effectiveCommand.type == TrajectoryType::Triangle){
            const double phaseCycle = effectiveCommand.phaseDeg / 360.0;
            effectiveCommand.centerPosition =
                    currentPosition - effectiveCommand.amplitude * triangleWave(phaseCycle);
        }
    }
    else{
        const double expectedStart = initialPositionForCommand(effectiveCommand);
        if(std::fabs(currentPosition - expectedStart) > 0.01){
            emit displayInfoSignal(QStringLiteral("Single motor trajectory test: current position %1 differs from trajectory start %2; trajectory not sent.")
                                   .arg(currentPosition, 0, 'f', 6)
                                   .arg(expectedStart, 0, 'f', 6)
                                   .toStdString(),
                                   "error");
            finishActiveTrajectory(QStringLiteral("current position differs from trajectory start"), "error");
            return;
        }
    }

    QVector<double> time;
    QVector<double> position;
    QVector<double> velocity;
    QString errorMessage;
    if(!buildTrajectory(effectiveCommand, time, position, velocity, errorMessage)){
        finishActiveTrajectory(errorMessage, "error");
        return;
    }

    std::vector<int> motorIndex = {effectiveCommand.axisIndex};
    std::vector<std::vector<double>> motorPosTraj;
    std::vector<std::vector<double>> motorVelTraj;
    std::vector<double> timeStamp;
    motorPosTraj.reserve(position.size());
    motorVelTraj.reserve(position.size());
    timeStamp.reserve(position.size());
    for(int i=0; i<position.size(); ++i){
        motorPosTraj.push_back({position[i]});
        motorVelTraj.push_back({i < velocity.size() ? velocity[i] : 0.0});
        timeStamp.push_back(time[i]);
    }

    activeAxis = effectiveCommand.axisIndex;
    activeTime = time;
    activeExpectedPosition = position;
    activeStartMs = QDateTime::currentMSecsSinceEpoch();
    activeSampleElapsedUs = 0;
    lastDriveFollowingErrorReadMs = -1;
    readRelativeTracePositionSamples(activeAxis);
    active = true;

    emit trajectoryPrepared(activeAxis, activeTime, activeExpectedPosition);
    emit trajectoryStateChanged(true, QStringLiteral("running"));

    hardwareInterface->motorPosTraj(motorIndex, motorPosTraj, motorVelTraj, timeStamp);
    if(!hardwareInterface->hasPvtTrajectory()){
        active = false;
        activeAxis = -1;
        activeStartMs = 0;
        activeSampleElapsedUs = 0;
        emit trajectoryStateChanged(false, QStringLiteral("轨迹下发失败"));
        return;
    }

    emit displayInfoSignal(QStringLiteral("单电机轨迹精度测试：轴%1轨迹已下发，点数%2，时长%3 s")
                           .arg(effectiveCommand.axisIndex)
                           .arg(position.size())
                           .arg(effectiveCommand.durationSec, 0, 'f', 3)
                           .toStdString(),
                           "info");
}

void SingleMotorTrajectoryWorker::requestStopTrajectory()
{
    if(active && activeAxis >= 0 && hardwareInterface && hardwareInterface->isLSConnected()){
        hardwareInterface->motorStop(activeAxis);
    }
    if(active){
        finishActiveTrajectory(QStringLiteral("stopped"), "warning");
    }
    else{
        emit trajectoryStateChanged(false, QStringLiteral("not started"));
    }
}

void SingleMotorTrajectoryWorker::loop()
{
    const qint64 loopEntryUs = monotonicNowUs();
    if(nextTraceReadDueUs <= 0){
        nextTraceReadDueUs = loopEntryUs;
    }
    if(loopEntryUs < nextTraceReadDueUs){
        return;
    }
    while(nextTraceReadDueUs <= loopEntryUs){
        nextTraceReadDueUs += kTraceReadBasePeriodUs;
    }

    double selectedPosition = 0.0;
    bool selectedPositionValid = false;
    if(selectedAxis >= 0 && (!active || selectedAxis != activeAxis)){
        double selectedCommandPosition = 0.0;
        selectedPositionValid = readRelativeTracePositions(selectedAxis,
                                                           selectedCommandPosition,
                                                           selectedPosition);
        if(selectedPositionValid){
            emit feedbackUpdated(selectedAxis, selectedPosition);
        }
    }

    if(!active){
        return;
    }
    if(!hardwareInterface || !hardwareInterface->isLSConnected()){
        finishActiveTrajectory(QStringLiteral("雷赛控制卡断开"), "error");
        return;
    }
    if(activeAxis < 0 || activeTime.isEmpty() || activeExpectedPosition.isEmpty()){
        finishActiveTrajectory(QStringLiteral("轨迹数据无效"), "error");
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const double wallElapsedSec = std::max<qint64>(0, nowMs - activeStartMs) / 1000.0;

    const std::vector<HardwareInterface::MotorTracePositionSample> traceSamples =
            readRelativeTracePositionSamples(activeAxis);
    if(traceSamples.empty()){
        const double finalTime = activeTime.back();
        if(wallElapsedSec <= finalTime + 2.0){
            return;
        }
        hardwareInterface->motorStop(activeAxis);
        finishActiveTrajectory(QStringLiteral("trajectory timeout"), "warning");
        return;
    }
    int driveFollowingErrorRaw = 0;
    bool driveFollowingErrorValid = false;
    if(lastDriveFollowingErrorReadMs < 0 || nowMs - lastDriveFollowingErrorReadMs >= 250){
        driveFollowingErrorValid =
                hardwareInterface->readLeadshineFollowingErrorRaw(activeAxis, driveFollowingErrorRaw);
        lastDriveFollowingErrorReadMs = nowMs;
    }

    for(const HardwareInterface::MotorTracePositionSample& sample : traceSamples){
        const double sampleElapsedSec =
                static_cast<double>(activeSampleElapsedUs) / 1000000.0;
        const double expectedPosition = interpolateExpected(sampleElapsedSec);
        const double tracePositionError =
                sample.commandRelativePosition - sample.feedbackRelativePosition;

        emit sampleUpdated(activeAxis,
                           sampleElapsedSec,
                           expectedPosition,
                           sample.feedbackRelativePosition,
                           tracePositionError,
                           sample.commandRelativePosition,
                           true,
                           driveFollowingErrorRaw,
                           driveFollowingErrorValid);
        activeSampleElapsedUs += kTraceReadBasePeriodUs;
    }

    if(selectedAxis == activeAxis){
        emit feedbackUpdated(activeAxis, traceSamples.back().feedbackRelativePosition);
    }

    const double finalTime = activeTime.back();
    if(wallElapsedSec >= finalTime && hardwareInterface->isMotorDone(activeAxis)){
        finishActiveTrajectory(QStringLiteral("completed"), "info");
    }
    else if(wallElapsedSec > finalTime + 2.0){
        hardwareInterface->motorStop(activeAxis);
        finishActiveTrajectory(QStringLiteral("轨迹超时结束"), "warning");
    }
}

bool SingleMotorTrajectoryWorker::buildTrajectory(const Command& command,
                                                  QVector<double>& time,
                                                  QVector<double>& position,
                                                  QVector<double>& velocity,
                                                  QString& errorMessage) const
{
    if(!isFinite(command.durationSec) || command.durationSec <= 0.0){
        errorMessage = QStringLiteral("轨迹时长无效");
        return false;
    }
    if(!isFinite(command.planStepMs) || command.planStepMs <= 0.0){
        errorMessage = QStringLiteral("规划时间步长无效");
        return false;
    }
    if(!isFinite(command.minPosition) ||
            !isFinite(command.maxPosition) ||
            command.maxPosition <= command.minPosition){
        errorMessage = QStringLiteral("电机位置限位无效");
        return false;
    }
    if((command.type == TrajectoryType::Sine || command.type == TrajectoryType::Triangle) &&
            (!isFinite(command.cycles) || command.cycles <= 0.0)){
        errorMessage = QStringLiteral("周期数必须大于0");
        return false;
    }

    const double planStepSec = command.planStepMs / 1000.0;
    const int segmentCount = static_cast<int>(std::ceil(command.durationSec / planStepSec));
    const int pointCount = segmentCount + 1;
    if(pointCount < 2 || pointCount > kMaxPvtPointCount){
        errorMessage = QStringLiteral("trajectory point count %1 exceeds the current PVT path limit %2")
                .arg(pointCount)
                .arg(kMaxPvtPointCount);
        return false;
    }

    time.resize(pointCount);
    position.resize(pointCount);
    velocity.resize(pointCount);

    const double phaseRad = command.phaseDeg * kTwoPi / 360.0;
    const double phaseCycle = command.phaseDeg / 360.0;
    for(int i=0; i<pointCount; ++i){
        const double t = (i == pointCount - 1) ?
                    command.durationSec :
                    std::min(command.durationSec, i * planStepSec);
        const double u = command.durationSec > 1e-12 ? t / command.durationSec : 0.0;
        const double lambda = quinticBlend(u);
        const double lambdaDot = quinticBlendDerivative(u) / command.durationSec;

        double target = command.startPosition;
        double targetVelocity = 0.0;
        switch(command.type){
        case TrajectoryType::Line: {
            const double delta = command.endPosition - command.startPosition;
            target = command.startPosition + delta * lambda;
            targetVelocity = delta * lambdaDot;
            break;
        }
        case TrajectoryType::Sine: {
            const double phaseSpan = kTwoPi * command.cycles;
            const double waveParameter = phaseRad + phaseSpan * lambda;
            const double waveParameterDot = phaseSpan * lambdaDot;
            target = command.centerPosition +
                    command.amplitude * std::sin(waveParameter);
            targetVelocity = command.amplitude * std::cos(waveParameter) * waveParameterDot;
            break;
        }
        case TrajectoryType::Triangle: {
            const double cyclePosition = phaseCycle + command.cycles * lambda;
            const double cyclePositionDot = command.cycles * lambdaDot;
            target = command.centerPosition +
                    command.amplitude * triangleWave(cyclePosition);
            targetVelocity = command.amplitude *
                    triangleWaveDerivative(cyclePosition) *
                    cyclePositionDot;
            break;
        }
        case TrajectoryType::Step: {
            const double delta = command.endPosition - command.startPosition;
            if(lambda <= 0.5){
                target = command.startPosition;
                targetVelocity = 0.0;
            }
            else{
                const double local = clamp01((lambda - 0.5) * 2.0);
                const double localBlend = quinticBlend(local);
                const double localBlendDerivative = quinticBlendDerivative(local);
                target = command.startPosition + delta * localBlend;
                targetVelocity = delta * localBlendDerivative * 2.0 * lambdaDot;
            }
            break;
        }
        }

        if(i == 0 || i == pointCount - 1){
            targetVelocity = 0.0;
        }
        if(!isFinite(target) || !isFinite(targetVelocity)){
            errorMessage = QStringLiteral("trajectory point %1 is not finite").arg(i);
            return false;
        }
        if(target < command.minPosition || target > command.maxPosition){
            errorMessage = QStringLiteral("轨迹点%1目标位置%2超出软件限位[%3, %4]")
                    .arg(i)
                    .arg(target, 0, 'f', 6)
                    .arg(command.minPosition, 0, 'f', 6)
                    .arg(command.maxPosition, 0, 'f', 6);
            return false;
        }
        if(command.maxVelocity > 1e-9 && std::fabs(targetVelocity) > command.maxVelocity){
            errorMessage = QStringLiteral("轨迹点%1期望速度%2超过速度上限%3")
                    .arg(i)
                    .arg(std::fabs(targetVelocity), 0, 'f', 6)
                    .arg(command.maxVelocity, 0, 'f', 6);
            return false;
        }

        time[i] = t;
        position[i] = target;
        velocity[i] = targetVelocity;
    }

    for(int i=1; i<pointCount; ++i){
        const double dt = time[i] - time[i - 1];
        if(dt <= 0.0){
            errorMessage = QStringLiteral("轨迹时间戳必须严格递增");
            return false;
        }
        if(command.maxVelocity > 1e-9){
            const double averageVelocity = (position[i] - position[i - 1]) / dt;
            if(std::fabs(averageVelocity) > command.maxVelocity){
                errorMessage = QStringLiteral("轨迹段%1平均速度%2超过速度上限%3")
                        .arg(i)
                        .arg(std::fabs(averageVelocity), 0, 'f', 6)
                        .arg(command.maxVelocity, 0, 'f', 6);
                return false;
            }
        }
    }

    velocity[0] = 0.0;
    velocity[pointCount - 1] = 0.0;
    return true;
}

bool SingleMotorTrajectoryWorker::readRelativeTracePositions(int axisIndex,
                                                             double& commandPosition,
                                                             double& feedbackPosition) const
{
    if(!hardwareInterface || !hardwareInterface->isLSConnected() || axisIndex < 0){
        return false;
    }

    double command = 0.0;
    double feedback = 0.0;
    if(!hardwareInterface->readMotorRelativeTracePositions(axisIndex, command, feedback)){
        return false;
    }
    if(!std::isfinite(command) || !std::isfinite(feedback)){
        return false;
    }
    commandPosition = command;
    feedbackPosition = feedback;
    return true;
}

std::vector<HardwareInterface::MotorTracePositionSample>
SingleMotorTrajectoryWorker::readRelativeTracePositionSamples(int axisIndex) const
{
    if(!hardwareInterface || !hardwareInterface->isLSConnected() || axisIndex < 0){
        return {};
    }
    return hardwareInterface->readMotorRelativeTracePositionSamples(axisIndex);
}

bool SingleMotorTrajectoryWorker::readRelativePosition(int axisIndex, double& relativePosition) const
{
    double commandPosition = 0.0;
    if(!readRelativeTracePositions(axisIndex, commandPosition, relativePosition)){
        return false;
    }
    return true;
}

double SingleMotorTrajectoryWorker::initialPositionForCommand(const Command& command) const
{
    switch(command.type){
    case TrajectoryType::Line:
    case TrajectoryType::Step:
        return command.startPosition;
    case TrajectoryType::Sine:
        return command.centerPosition +
                command.amplitude * std::sin(command.phaseDeg * kTwoPi / 360.0);
    case TrajectoryType::Triangle:
        return command.centerPosition +
                command.amplitude * triangleWave(command.phaseDeg / 360.0);
    }
    return command.startPosition;
}

double SingleMotorTrajectoryWorker::interpolateExpected(double elapsedSec) const
{
    if(activeTime.isEmpty() || activeExpectedPosition.isEmpty()){
        return std::numeric_limits<double>::quiet_NaN();
    }
    if(elapsedSec <= activeTime.front()){
        return activeExpectedPosition.front();
    }
    if(elapsedSec >= activeTime.back()){
        return activeExpectedPosition.back();
    }

    const auto upperIt = std::upper_bound(activeTime.begin(), activeTime.end(), elapsedSec);
    const int upperIndex = static_cast<int>(std::distance(activeTime.begin(), upperIt));
    const int lowerIndex = std::max(0, upperIndex - 1);
    const double t0 = activeTime[lowerIndex];
    const double t1 = activeTime[upperIndex];
    const double p0 = activeExpectedPosition[lowerIndex];
    const double p1 = activeExpectedPosition[upperIndex];
    if(t1 <= t0){
        return p0;
    }
    const double u = (elapsedSec - t0) / (t1 - t0);
    return p0 + (p1 - p0) * u;
}

void SingleMotorTrajectoryWorker::finishActiveTrajectory(const QString& statusText, const std::string& type)
{
    active = false;
    activeAxis = -1;
    activeStartMs = 0;
    activeSampleElapsedUs = 0;
    lastDriveFollowingErrorReadMs = -1;
    emit trajectoryStateChanged(false, statusText);
    if(!statusText.isEmpty()){
        emit displayInfoSignal(QStringLiteral("单电机轨迹精度测试：%1")
                               .arg(statusText)
                               .toStdString(),
                               type);
    }
}
