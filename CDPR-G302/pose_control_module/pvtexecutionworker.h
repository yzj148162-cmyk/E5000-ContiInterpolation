/*
 * 文件总览：
 * - PvtExecutionWorker 封装 PVT 轨迹执行、暂停、恢复、停止并回零等硬件动作。
 * - MainWindow 负责做状态与安全检查，本类聚焦把已生成的电机轨迹下发给 HardwareInterface。
 */

#ifndef PVTEXECUTIONWORKER_H
#define PVTEXECUTIONWORKER_H

#include <QObject>
#include <vector>

#include "hardwareinterface.h"

class PvtExecutionWorker : public QObject
{
    Q_OBJECT
public:
    struct PvtCommand {
        std::vector<int> motorIndex;
        std::vector<std::vector<double>> motorPosTraj;
        std::vector<std::vector<double>> motorVelTraj;
        std::vector<double> motorVelMax;
        std::vector<double> timeStamp;
    };

    struct StopReturnCommand {
        std::vector<int> motorIndex;
        std::vector<double> motorVelMax;
        bool smoothPauseActivePvt = true;
        double smoothPauseTimeSec = 0.2;
        int stopWaitTimeoutMs = 3000;
        double returnDurationSec = 15.0;
        double minReturnVel = 0.05;
    };

    // 绑定硬件接口；实际 PVT 下发和停机动作都委托给 HardwareInterface。
    explicit PvtExecutionWorker(HardwareInterface* hardware, QObject* parent = nullptr);

    // 下发完整 PVT 轨迹并启动执行。
    bool startPvtTrajectory(const PvtCommand& command);
    // 暂停当前 PVT 轨迹，保留可恢复状态。
    bool pausePvtMotion();
    // 从暂停点继续执行 PVT 轨迹。
    bool resumePvtMotion();
    // 平滑停止当前轨迹并按命令回到起点/零位。
    bool stopAndReturnHome(const StopReturnCommand& command);
    // 清除 PVT 运行状态缓存。
    void clearPvtState();

signals:
    void displayInfoSignal(std::string info, std::string type);

private:
    // 等待指定电机轴运动完成，用于停止回零流程的同步。
    bool waitAxesDone(const std::vector<int>& motorIndex, int timeoutMs);

    HardwareInterface* hardwareInterface = nullptr;
};

#endif // PVTEXECUTIONWORKER_H
