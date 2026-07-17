#ifndef E5000CONTIINTERFACE_H
#define E5000CONTIINTERFACE_H

#include <QString>

#include "LTDMC.h"
#include "common/ContiTypes.h"

// 所有 LTDMC 调用集中在这里，避免 UI 和轨迹代码直接依赖控制卡 API。
class E5000ContiInterface
{
public:
    bool initializeBoard(WORD requestedCardNo, short &boardCount, QString &errorMessage) const;
    bool closeBoard(QString &errorMessage) const;
    bool setAxisEquivalent(WORD cardNo, WORD axis, double pulsePerUnit, QString &errorMessage) const;
    bool clearAxisFaults(WORD cardNo, WORD axis, QString &noticeMessage, QString &errorMessage) const;
    bool enableAxis(WORD cardNo, WORD axis, QString &errorMessage) const;
    bool disableAxis(WORD cardNo, WORD axis, QString &errorMessage) const;
    bool readPositionUnit(WORD cardNo, WORD axis, double &position, QString &errorMessage) const;
    bool readTargetPositionUnit(WORD cardNo, WORD axis, double &position, QString &errorMessage) const;
    bool readAxisFeedback(WORD cardNo, WORD axis, AxisFeedback &feedback) const;
    bool startPointMove(const SingleAxisJogConfig &config, QString &errorMessage) const;
    bool stopAxis(WORD cardNo, WORD axis, bool emergency, QString &errorMessage) const;
    bool isAxisMotionDone(WORD cardNo, WORD axis) const;
    bool axisMotionDone(WORD cardNo, WORD axis, bool &done, QString &errorMessage) const;
    bool axisRunMode(WORD cardNo, WORD axis, quint16 &mode, QString &errorMessage) const;
    bool isContiMotionDone(const ContiTestConfig &config) const;

    bool configureAndOpen(const ContiTestConfig &config, QString &errorMessage) const;
    bool pushLine(const ContiTestConfig &config, const ContiPoint &point, long mark, QString &errorMessage) const;
    bool start(const ContiTestConfig &config, QString &errorMessage) const;
    ContiSpeedRatioResult changeSpeedRatio(const ContiTestConfig &config, QString &errorMessage) const;
    bool stop(const ContiTestConfig &config, bool emergency, QString &errorMessage) const;
    bool closeList(const ContiTestConfig &config, QString &errorMessage) const;

    long currentMark(const ContiTestConfig &config) const;
    long remainSpace(const ContiTestConfig &config) const;
    short runState(const ContiTestConfig &config) const;
};

#endif // E5000CONTIINTERFACE_H
