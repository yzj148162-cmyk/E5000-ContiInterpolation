/*
 * 文件总览：
 * - MotorTorqueTestWorker 用于单轴力矩模式测试，按配置的目标力矩和位置/速度边界循环下发命令。
 * - 该 worker 主要服务调试，不参与正常多轴 PVT 或实时力控流程。
 */

#ifndef MOTORTORQUETESTWORKER_H
#define MOTORTORQUETESTWORKER_H

#include <QObject>
#include <QMutex>
#include <QString>

#include <string>

class QTimer;
class HardwareInterface;

class MotorTorqueTestWorker : public QObject
{
    Q_OBJECT
public:
    // 绑定硬件接口；worker 由 MainWindow 放入独立线程后周期执行。
    explicit MotorTorqueTestWorker(HardwareInterface* hardware, QObject* parent = nullptr);

public slots:
    // 启动定时循环，开始读取反馈并按配置下发力矩。
    void start();
    // 停止定时循环并撤销当前力矩命令。
    void stop();
    // 更新单轴力矩测试参数，包括目标力矩、位置边界和速度限制。
    void setConfig(int axisIndex,
                   double targetTorque,
                   double relativeMinPos,
                   double relativeMaxPos,
                   double velocityLimit,
                   bool allowMoveOutsideSoftwareLimit);
    // 切换力矩输出是否真正生效，保留参数但可临时暂停命令。
    void setTorqueActive(bool active);
    // 请求停止当前运动，供 UI 或安全逻辑跨线程调用。
    void requestStopMotion();

signals:
    void statusUpdated(int axisIndex,
                       double relativePosition,
                       double actualTorque,
                       double actualVelocity,
                       bool active);
    void activeChanged(bool active);
    void displayInfoSignal(std::string info, std::string type);

private slots:
    // 周期任务：检查位置边界、刷新反馈并维持/停止力矩命令。
    void loop();

private:
    struct Config {
        int axisIndex = -1;
        double targetTorque = 0.0;
        double relativeMinPos = 0.0;
        double relativeMaxPos = 0.0;
        double velocityLimit = 0.0;
        // 仅在主界面已确认该轴处于位置超限恢复状态时允许继续向限位外运动。
        bool allowMoveOutsideSoftwareLimit = false;
        bool active = false;
    };

    // 线程安全地复制当前配置，避免循环中长时间持锁。
    Config currentConfig() const;
    // 停止力矩模式并发出提示信息。
    void stopTorqueMotion(const QString& reason, const std::string& type = "warning");
    // 确保硬件侧已按当前配置进入目标力矩输出状态。
    bool ensureTorqueCommand(const Config& config);
    // 计算目标力矩方向，用于边界保护和命令状态判断。
    static int torqueSign(double torque);

    HardwareInterface* hardwareInterface = nullptr;
    QTimer* timer = nullptr;
    mutable QMutex configMutex;
    Config config;
    bool commandStarted = false;
    int lastCommandAxis = -1;
    double lastCommandTorque = 0.0;
    bool recoveryFromSoftwareLimitActive = false;
};

#endif // MOTORTORQUETESTWORKER_H
