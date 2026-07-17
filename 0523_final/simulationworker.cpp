#include "simulationworker.h"

#include "forcesimulationmodel.h"
#include "positionsimulationmodel.h"

#include <QMetaObject>
#include <QThread>

SimulationWorker::SimulationWorker(QObject* parent)
    : QObject(parent)
{
}

SimulationWorker::~SimulationWorker()
{
    stopPositionSimulation();
    stopForceSimulation();
}

PositionSimulationModel* SimulationWorker::positionSimulationModel() const
{
    return m_positionSimulationModel;
}

ForceSimulationModel* SimulationWorker::forceSimulationModel() const
{
    return m_forceSimulationModel;
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

void SimulationWorker::setForceSimulationModel(ForceSimulationModel* model)
{
    if (m_forceSimulationModel == model) {
        return;
    }

    stopForceSimulation();
    m_forceSimulationModel = model;
    if (!m_forceSimulationModel) {
        return;
    }

    m_forceSimulationThread = new QThread(this);
    m_forceSimulationModel->moveToThread(m_forceSimulationThread);
    connect(m_forceSimulationThread, &QThread::started, m_forceSimulationModel, &ForceSimulationModel::startTimer);
    connect(m_forceSimulationThread, &QThread::finished, m_forceSimulationModel, &ForceSimulationModel::stopTimer);
}

bool SimulationWorker::startForceSimulation()
{
    if (!m_forceSimulationModel || !m_forceSimulationThread) {
        return false;
    }
    if (!m_forceSimulationThread->isRunning()) {
        m_forceSimulationThread->start();
    }
    return true;
}

void SimulationWorker::stopForceSimulation()
{
    if (m_forceSimulationThread) {
        if (m_forceSimulationThread->isRunning()) {
            QMetaObject::invokeMethod(m_forceSimulationModel, [this]() {
                if (m_forceSimulationModel) {
                    m_forceSimulationModel->stopTimer();
                }
            }, Qt::BlockingQueuedConnection);
            m_forceSimulationThread->quit();
            if (!m_forceSimulationThread->wait(1000)) {
                m_forceSimulationThread->terminate();
                m_forceSimulationThread->wait(500);
            }
        }
        delete m_forceSimulationThread;
        m_forceSimulationThread = nullptr;
    }

    if (m_forceSimulationModel) {
        delete m_forceSimulationModel;
        m_forceSimulationModel = nullptr;
    }
}
