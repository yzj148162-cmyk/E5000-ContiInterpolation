#include "pvtexecutionworker.h"

#include <QElapsedTimer>
#include <QThread>
#include <algorithm>
#include <cmath>

PvtExecutionWorker::PvtExecutionWorker(HardwareInterface* hardware, QObject* parent)
    : QObject(parent),
      hardwareInterface(hardware)
{
}

bool PvtExecutionWorker::startPvtTrajectory(const PvtCommand& command)
{
    if(!hardwareInterface || !hardwareInterface->isLSConnected()){
        emit displayInfoSignal("错误：雷赛控制卡未连接，无法启动 PVT 位置轨迹", "error");
        return false;
    }

    hardwareInterface->motorPosTraj(command.motorIndex,
                                    command.motorPosTraj,
                                    command.motorVelTraj,
                                    command.timeStamp);
    if(!hardwareInterface->hasPvtTrajectory()){
        emit displayInfoSignal("错误：PVT 位置轨迹启动失败", "error");
        return false;
    }
    return true;
}

bool PvtExecutionWorker::pausePvtMotion()
{
    if(!hardwareInterface){
        return false;
    }
    return hardwareInterface->pausePvtMotion();
}

bool PvtExecutionWorker::resumePvtMotion()
{
    if(!hardwareInterface){
        return false;
    }
    return hardwareInterface->resumePvtMotion();
}

void PvtExecutionWorker::clearPvtState()
{
    if(hardwareInterface){
        hardwareInterface->clearPvtTrajectoryState();
    }
}

bool PvtExecutionWorker::stopAndReturnHome(const StopReturnCommand& command)
{
    if(!hardwareInterface || !hardwareInterface->isLSConnected()){
        emit displayInfoSignal("错误：雷赛控制卡未连接，无法停止位置轨迹", "error");
        return false;
    }

    if(command.smoothPauseActivePvt &&
            hardwareInterface->hasPvtTrajectory() &&
            hardwareInterface->isPvtMotionRunning()){
        hardwareInterface->smoothPausePvtMotion(command.smoothPauseTimeSec);
    }

    std::vector<int> stopMotorIndex = command.motorIndex;
    if(stopMotorIndex.empty()){
        const std::vector<double> allMotorPos = hardwareInterface->getAllMotorPos();
        stopMotorIndex.reserve(allMotorPos.size());
        for(int i=0; i<static_cast<int>(allMotorPos.size()); ++i){
            stopMotorIndex.push_back(i);
        }
    }

    for(int axisIndex : stopMotorIndex){
        hardwareInterface->motorStop(axisIndex);
    }

    if(!waitAxesDone(stopMotorIndex, command.stopWaitTimeoutMs)){
        emit displayInfoSignal("警告：电机停止等待超时，仍继续下发慢速回零指令", "warning");
    }

    hardwareInterface->clearPvtTrajectoryState();
    const std::vector<double> motorHome = hardwareInterface->getAllMotorHome();
    if(motorHome.empty()){
        emit displayInfoSignal("当前没有有效电机零点，已仅执行电机停止", "warning");
        return true;
    }

    bool returnCmdOk = true;
    for(int axisIndex : stopMotorIndex){
        if(axisIndex < 0 || axisIndex >= static_cast<int>(motorHome.size())){
            returnCmdOk = false;
            continue;
        }

        const double curPos = hardwareInterface->readMotorCurPos(axisIndex);
        double returnVel = std::max(std::fabs(curPos - motorHome[axisIndex]) / command.returnDurationSec,
                                    command.minReturnVel);
        if(axisIndex < static_cast<int>(command.motorVelMax.size()) && command.motorVelMax[axisIndex] > 1e-5){
            returnVel = std::min(returnVel, command.motorVelMax[axisIndex]);
        }
        returnCmdOk = hardwareInterface->motorAbsPos(axisIndex, motorHome[axisIndex], returnVel) && returnCmdOk;
    }

    if(!returnCmdOk){
        emit displayInfoSignal("错误：部分电机慢速回零指令下发失败", "error");
        return false;
    }

    emit displayInfoSignal("平台轨迹已中止，电机已下发慢速回零指令", "info");
    return true;
}

bool PvtExecutionWorker::waitAxesDone(const std::vector<int>& motorIndex, int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();
    while(timer.elapsed() < timeoutMs){
        bool allDone = true;
        for(int axisIndex : motorIndex){
            if(!hardwareInterface->isMotorDone(axisIndex)){
                allDone = false;
                break;
            }
        }
        if(allDone){
            return true;
        }
        QThread::msleep(10);
    }
    return false;
}
