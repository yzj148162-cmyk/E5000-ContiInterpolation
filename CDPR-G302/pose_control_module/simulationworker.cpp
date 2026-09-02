/*
 * 文件总览：
 * - SimulationWorker 的实现文件，处理位置仿真对象生命周期、启动和停止。
 * - MainWindow 通过本类统一释放 PositionSimulationModel，避免 UI 层重复管理 QThread 状态。
 */

#include "simulationworker.h"

#include "positionsimulationmodel.h"

#include <QThread>

SimulationWorker::SimulationWorker(QObject* parent)
    : QObject(parent)
{
}

SimulationWorker::~SimulationWorker()
{
    stopPositionSimulation();
}

PositionSimulationModel* SimulationWorker::positionSimulationModel() const
{
    return m_positionSimulationModel;
}

void SimulationWorker::setPositionSimulationModel(PositionSimulationModel* model)
{
    if (m_positionSimulationModel == model) {
        return;
    }
    stopPositionSimulation();
    m_positionSimulationModel = model;
}

bool SimulationWorker::startPositionSimulation()
{
    if (!m_positionSimulationModel) {
        return false;
    }
    if (!m_positionSimulationModel->QThread::isRunning()) {
        m_positionSimulationModel->start();
    }
    return true;
}

void SimulationWorker::stopPositionSimulation()
{
    if (!m_positionSimulationModel) {
        return;
    }

    m_positionSimulationModel->stopThread();
    if (m_positionSimulationModel->QThread::isRunning() && !m_positionSimulationModel->wait(500)) {
        m_positionSimulationModel->terminate();
        m_positionSimulationModel->wait(500);
    }
    delete m_positionSimulationModel;
    m_positionSimulationModel = nullptr;
}
