#ifndef ETHERCATINTERFACE_H
#define ETHERCATINTERFACE_H

#include <QString>
#include <QVector>

#include "common/FeedbackData.h"
#include "common/TrajectoryPoint.h"
#include "config/MotionConfig.h"

// 对雷赛 EtherCAT 总线卡接口的最小封装。
// 当前负责三类核心动作：
// 1. 控制卡初始化、轴使能/失能、停止轴
// 2. CSP 模式下的 PDO 读写
// 3. PVT(PVTS) 模式下的整表装载与启动
class EthercatInterface
{
public:
    bool initializeBoard(QString &errorMessage) const;
    bool closeBoard(QString &errorMessage) const;
    bool stopAxis(const MotionConfig &config, QString &errorMessage) const;
    bool enableAxis(const MotionConfig &config, QString &errorMessage) const;
    bool disableAxis(const MotionConfig &config, QString &errorMessage) const;

    bool configureCspMode(const MotionConfig &config, QString &errorMessage) const;
    bool writeCspTargetPositionRaw(const MotionConfig &config, qint32 targetPosRaw, QString &errorMessage) const;
    bool startPvtsMotion(const MotionConfig &config,
                         const QVector<TrajectoryPoint> &trajectory,
                         QString &errorMessage) const;
    bool readPvtRunIndex(const MotionConfig &config, quint32 &runIndex, QString &errorMessage) const;
    bool isAxisMotionDone(const MotionConfig &config, bool &done, QString &errorMessage) const;

    bool readFeedback(const MotionConfig &config, FeedbackData &feedback, QString &errorMessage) const;
    bool readStartPositionRaw(const MotionConfig &config, qint32 &startPosRaw, QString &errorMessage) const;

};

#endif // ETHERCATINTERFACE_H
