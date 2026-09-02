/*
 * 文件总览：
 * - SimulationWorker 是位置仿真模型线程管理器，负责启动、停止并释放 PositionSimulationModel。
 * - PositionSimulationModel 自身继承 QThread，因此这里不再额外创建工作线程。
 */

#ifndef SIMULATIONWORKER_H
#define SIMULATIONWORKER_H

#include <QObject>

class QThread;
class PositionSimulationModel;

class SimulationWorker : public QObject
{
    Q_OBJECT
public:
    // 创建仿真管理器；具体仿真模型稍后由 MainWindow 注入。
    explicit SimulationWorker(QObject* parent = nullptr);
    // 析构时停止并释放当前仿真模型。
    ~SimulationWorker();

    // 返回当前托管的 PositionSimulationModel 指针。
    PositionSimulationModel* positionSimulationModel() const;

    // 替换托管模型；调用方负责先停止旧模型或交给本类清理。
    void setPositionSimulationModel(PositionSimulationModel* model);
    // 启动 PositionSimulationModel 线程执行轨迹仿真。
    bool startPositionSimulation();
    // 请求仿真线程停止并等待退出。
    void stopPositionSimulation();

private:
    PositionSimulationModel* m_positionSimulationModel = nullptr;
};

#endif // SIMULATIONWORKER_H
