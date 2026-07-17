#ifndef TRAJECTORYPOINT_H
#define TRAJECTORYPOINT_H

#include <QtGlobal>

// 规划线程产出、硬件线程消费的最小轨迹点单元。
// 当前先保留目标位置/速度/加速度三个字段，便于后续扩展。
struct TrajectoryPoint
{
    double timeS = 0.0;
    qint32 targetPosRaw = 0;
    qint32 targetVelRaw = 0;
    qint32 targetAccRaw = 0;
};

#endif // TRAJECTORYPOINT_H
