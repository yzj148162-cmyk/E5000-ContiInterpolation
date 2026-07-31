#ifndef MOTIONCARDHARDWAREINTERFACE_H
#define MOTIONCARDHARDWAREINTERFACE_H

#include <QVector>

#include "common/ContiTypes.h"
class QThread;

// 唯一的板卡访问边界。该服务内部独占一个 QThread；所有实现 LTDMC 的对象
// 都只在该线程内执行。控制层只能调用本类的同步代理方法，不能直接访问 SDK。
class MotionCardHardwareInterface
{
public:
    MotionCardHardwareInterface();
    ~MotionCardHardwareInterface();
    MotionCardHardwareInterface(const MotionCardHardwareInterface &) = delete;
    MotionCardHardwareInterface &operator=(const MotionCardHardwareInterface &) = delete;

    bool initializeBoard(quint16 cardNo, short &boardCount, QString &errorMessage);
    bool closeBoard(QString &errorMessage);
    bool setBusCycle(int cycleUs, QString &errorMessage);
    bool readBusCycle(int &cycleUs, QString &errorMessage) const;
    bool readControllerWorkMode(quint16 &workMode, QString &errorMessage) const;
    bool readEthercatBusState(quint16 &busErrorCode, quint32 &masterState,
                              QString &errorMessage) const;
    bool readEthercatSlaveCount(quint16 &slaveCount, QString &errorMessage) const;

    bool configureTrace(const QVector<quint16> &axes, int samplePeriodUs, int traceBaseCycleUs,
                        double degreesPerCardUnit, QString &errorMessage);
    bool enableAxis(quint16 axis, QString &noticeMessage, QString &errorMessage);
    bool disableAxis(quint16 axis, QString &errorMessage);

    bool readCommandPosition(quint16 axis, double &position, QString &errorMessage) const;
    bool readTargetPositionUnit(quint16 axis, double &position, QString &errorMessage) const;
    bool startPointMove(const SingleAxisJogConfig &config, QString &errorMessage);
    bool startVelocityMove(const VelocityControlConfig &config,
                           double initialVelocityDegreePerSecond,
                           QString &errorMessage);
    bool changeVelocity(const VelocityControlConfig &config,
                        double velocityDegreePerSecond,
                        short &apiResult,
                        QString &errorMessage);
    bool startPvtMotion(
        const QVector<quint16> &axes,
        const QVector<double> &timeS,
        const QVector<QVector<double>> &axisPositionDegree,
        double degreesPerCardUnit,
        QString &errorMessage);
    bool readPvtMotionStatus(const QVector<quint16> &axes,
                             int &minimumRunIndex,
                             bool &allAxesDone,
                             QString &errorMessage) const;
    bool startTorqueMove(const TorqueTestConfig &config, int torqueRaw,
                         double absolutePositionLimitDegree,
                         short &apiResult, QString &errorMessage);
    bool changeTorque(quint16 axis, int torqueRaw, short &apiResult,
                      QString &errorMessage);
    bool readTorque(quint16 axis, int &torqueRaw, short &apiResult,
                    QString &errorMessage) const;
    bool writeTorqueVelocityLimit(const TorqueTestConfig &config, long value,
                                  quint16 &nodeAddress, long &readback,
                                  QString &errorMessage);
    bool readTorqueDriveDiagnostic(quint16 axis,
                                   TorqueDriveDiagnostic &diagnostic,
                                   QString &errorMessage) const;
    bool stopAxis(quint16 axis, bool emergency, QString &errorMessage) const;
    bool stopAxis(quint16 cardNo, quint16 axis, bool emergency, QString &errorMessage) const;
    bool axisMotionDone(quint16 axis, bool &done, QString &errorMessage) const;
    bool axisRunMode(quint16 axis, quint16 &mode, QString &errorMessage) const;

    bool configureAndOpen(const ContiTestConfig &config, QString &errorMessage) const;
    bool pushLine(const ContiTestConfig &config, const ContiPoint &point, long mark, QString &errorMessage) const;
    bool start(const ContiTestConfig &config, QString &errorMessage) const;
    bool startStreaming(const ContiTestConfig &config,
                        const QVector<ContiFeedItem> &points,
                        bool preloadAllToCard,
                        QString &errorMessage) const;
    void appendStreamingPoints(const QVector<ContiFeedItem> &points) const;
    ContiFeedStatus streamingStatus() const;
    ContiSpeedRatioResult changeSpeedRatio(const ContiTestConfig &config, QString &errorMessage) const;
    bool stop(const ContiTestConfig &config, bool emergency, QString &errorMessage) const;
    bool closeList(const ContiTestConfig &config, QString &errorMessage) const;
    bool contiMotionDone(const ContiTestConfig &config) const;
    bool readContiCardReadback(const ContiTestConfig &config,
                               ContiCardReadback &readback,
                               QString &errorMessage) const;
    bool readVectorSpeedDegreePerSecond(const ContiTestConfig &config,
                                        double &speedDegreePerSecond,
                                        QString &errorMessage) const;
    long currentMark(const ContiTestConfig &config) const;
    long remainSpace(const ContiTestConfig &config) const;
    short runState(const ContiTestConfig &config) const;

    bool pollFeedback(QString &errorMessage);
    QVector<AxisFeedback> axisFeedback() const;
    QVector<quint16> traceAxes() const;
    bool traceConfigured() const;
    bool traceEverRead() const;
    int traceFramesRead() const;
    int traceSamplePeriodUs() const;
    TraceReadDiagnostics traceReadDiagnostics() const;
    QVector<TraceTelemetryFrame> takeTraceTelemetryFrames();
    short traceLastApiResult() const;
    QString traceStateText() const;
    quint16 busErrorCode() const;

    // 兼容控制层已有的调用形状。cardNo 由本服务在 initializeBoard 后固定，
    // 因而这些参数仅用于保持控制层不直接接触 SDK 类型。
    bool setAxisEquivalent(quint16 cardNo, quint16 axis, double pulsePerUnit, QString &errorMessage) const;
    bool clearAxisFaults(quint16 cardNo, quint16 axis, QString &noticeMessage, QString &errorMessage) const;
    bool enableAxis(quint16 cardNo, quint16 axis, QString &errorMessage) const;
    bool disableAxis(quint16 cardNo, quint16 axis, QString &errorMessage) const;
    bool readPositionUnit(quint16 cardNo, quint16 axis, double &position, QString &errorMessage) const;
    bool isAxisMotionDone(quint16 cardNo, quint16 axis) const;
    bool isContiMotionDone(const ContiTestConfig &config) const;

private:
    class Backend;
    Backend *backend_ = nullptr;
    QThread *hardwareThread_ = nullptr;
};

#endif // MOTIONCARDHARDWAREINTERFACE_H
