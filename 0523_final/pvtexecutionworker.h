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
        std::vector<double> timeStamp;
    };

    struct StopReturnCommand {
        std::vector<int> motorIndex;
        std::vector<double> motorVelMax;
        bool smoothPauseActivePvt = true;
        double smoothPauseTimeSec = 0.5;
        int stopWaitTimeoutMs = 3000;
        double returnDurationSec = 15.0;
        double minReturnVel = 0.05;
    };

    explicit PvtExecutionWorker(HardwareInterface* hardware, QObject* parent = nullptr);

    bool startPvtTrajectory(const PvtCommand& command);
    bool pausePvtMotion();
    bool resumePvtMotion();
    bool stopAndReturnHome(const StopReturnCommand& command);
    void clearPvtState();

signals:
    void displayInfoSignal(std::string info, std::string type);

private:
    bool waitAxesDone(const std::vector<int>& motorIndex, int timeoutMs);

    HardwareInterface* hardwareInterface = nullptr;
};

#endif // PVTEXECUTIONWORKER_H
