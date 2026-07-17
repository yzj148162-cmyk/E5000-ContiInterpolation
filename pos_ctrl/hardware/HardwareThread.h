#ifndef HARDWARETHREAD_H
#define HARDWARETHREAD_H

#include <QMutex>
#include <QQueue>
#include <QThread>
#include <QVector>

#include "common/SharedContext.h"
#include "config/SystemConfig.h"
#include "hardware/EthercatInterface.h"

// 硬件线程命令。
// UI 线程只投递命令，不直接操作控制卡。
struct HardwareCommand
{
    enum class Type {
        InitializeBoard,
        CloseBoard,
        EnableAxis,
        DisableAxis,
        StartMotion,
        StopMotion
    };

    Type type = Type::InitializeBoard;
    MotionConfig config;
};

// 硬件线程。
// 这是唯一按照严格 1ms 节拍运行的线程，负责轨迹点消费和 PDO 收发。
class HardwareThread : public QThread
{
    Q_OBJECT

public:
    HardwareThread(SharedContext &sharedContext, const SystemConfig &systemConfig, QObject *parent = nullptr);
    ~HardwareThread() override = default;

    void enqueueCommand(const HardwareCommand &command);

protected:
    void run() override;

signals:
    void logMessage(const QString &message);
    void boardStateChanged(bool initialized);

private:
    PlanningOutputType currentOutputType() const;
    PlannerWakePolicy currentWakePolicy() const;
    bool shouldWakePlannerPeriodically() const;
    void wakePlannerForEvent();
    void processPendingCommands();
    void handleInitializeBoard();
    void handleCloseBoard();
    void handleEnableAxis(const MotionConfig &config);
    void handleDisableAxis(const MotionConfig &config);
    void handleStartMotion(const MotionConfig &config);
    void handleStopMotion();
    void publishFeedback(const FeedbackData &feedback);

private:
    SharedContext &sharedContext_;
    SystemConfig systemConfig_;
    EthercatInterface ethercat_;

    QMutex commandMutex_;
    QQueue<HardwareCommand> commands_;

    MotionConfig activeConfig_;
    bool boardInitialized_ = false;
    bool motionActive_ = false;
    TrajectoryPoint lastPoint_;
    QVector<TrajectoryPoint> activePvtTrajectory_;
};

#endif // HARDWARETHREAD_H
