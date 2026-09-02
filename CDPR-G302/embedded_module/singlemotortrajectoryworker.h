/*
 * 文件总览：
 * - SingleMotorTrajectoryWorker 用于单轴调试，生成线性/正弦/三角/阶跃位置轨迹并读取实际跟随情况。
 * - 它直接依赖 HardwareInterface，但只控制选定轴，适合调试电机、编码器和驱动跟随误差。
 */

#ifndef SINGLEMOTORTRAJECTORYWORKER_H
#define SINGLEMOTORTRAJECTORYWORKER_H

#include <QObject>
#include <QString>
#include <QVector>

#include <string>
#include <vector>

#include "hardwareinterface.h"

class QTimer;

class SingleMotorTrajectoryWorker : public QObject
{
    Q_OBJECT
public:
    enum class TrajectoryType {
        Line = 0,
        Sine = 1,
        Triangle = 2,
        Step = 3
    };

    struct Command {
        int axisIndex = -1;
        TrajectoryType type = TrajectoryType::Line;
        double startPosition = 0.0;
        double endPosition = 0.0;
        double centerPosition = 0.0;
        double amplitude = 0.0;
        double cycles = 1.0;
        double phaseDeg = 0.0;
        double durationSec = 5.0;
        double planStepMs = 20.0;
        double minPosition = 0.0;
        double maxPosition = 0.0;
        double maxVelocity = 0.0;
        bool useCurrentStart = true;
    };

    // 绑定硬件接口；该 worker 只控制选中的单个轴。
    explicit SingleMotorTrajectoryWorker(HardwareInterface* hardware, QObject* parent = nullptr);

public slots:
    // 启动周期循环，持续读取当前选中轴反馈。
    void start();
    // 停止周期循环并结束任何正在执行的单轴轨迹。
    void stop();
    // 更新当前调试轴，未执行轨迹时用于实时反馈显示。
    void setSelectedAxis(int axisIndex);
    // 校验并生成轨迹，然后按采样点向硬件下发位置命令。
    void startTrajectory(const SingleMotorTrajectoryWorker::Command& command);
    // 请求中止当前轨迹并通知 UI 状态变化。
    void requestStopTrajectory();

signals:
    void feedbackUpdated(int axisIndex, double relativePosition);
    void trajectoryPrepared(int axisIndex, QVector<double> time, QVector<double> expectedPosition);
    void sampleUpdated(int axisIndex,
                       double time,
                       double expectedPosition,
                       double actualPosition,
                       double tracePositionError,
                       double commandPosition,
                       bool commandPositionValid,
                       int driveFollowingErrorRaw,
                       bool driveFollowingErrorValid);
    void trajectoryStateChanged(bool active, QString statusText);
    void displayInfoSignal(std::string info, std::string type);

private slots:
    // 周期任务：读取跟随误差/反馈位置，并推进正在执行的单轴轨迹。
    void loop();

private:
    // 根据命令类型生成期望位置、速度和时间序列。
    bool buildTrajectory(const Command& command,
                         QVector<double>& time,
                         QVector<double>& position,
                         QVector<double>& velocity,
                         QString& errorMessage) const;
    // 从 Trace 读取命令位置和反馈位置，用于更精确的跟随误差显示。
    bool readRelativeTracePositions(int axisIndex,
                                    double& commandPosition,
                                    double& feedbackPosition) const;
    // 读取当前轴 Trace 缓存中的一批相对位置样本。
    std::vector<HardwareInterface::MotorTracePositionSample>
    readRelativeTracePositionSamples(int axisIndex) const;
    // 读取当前轴相对零位的位置反馈。
    bool readRelativePosition(int axisIndex, double& relativePosition) const;
    // 解析轨迹起点；可使用当前反馈位置或命令中的固定起点。
    double initialPositionForCommand(const Command& command) const;
    // 按已生成轨迹对给定运行时间做线性插值。
    double interpolateExpected(double elapsedSec) const;
    // 统一收尾：停止硬件命令、清状态并发出 UI 状态文本。
    void finishActiveTrajectory(const QString& statusText, const std::string& type);

    HardwareInterface* hardwareInterface = nullptr;
    QTimer* timer = nullptr;
    qint64 nextTraceReadDueUs = 0;
    int selectedAxis = -1;
    bool active = false;
    int activeAxis = -1;
    qint64 activeStartMs = 0;
    qint64 activeSampleElapsedUs = 0;
    qint64 lastDriveFollowingErrorReadMs = -1;
    QVector<double> activeTime;
    QVector<double> activeExpectedPosition;
};

#endif // SINGLEMOTORTRAJECTORYWORKER_H
