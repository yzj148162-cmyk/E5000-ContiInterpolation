#ifndef SIMULATIONWORKER_H
#define SIMULATIONWORKER_H

#include <QObject>

class QThread;
class PositionSimulationModel;
class ForceSimulationModel;

class SimulationWorker : public QObject
{
    Q_OBJECT
public:
    explicit SimulationWorker(QObject* parent = nullptr);
    ~SimulationWorker();

    PositionSimulationModel* positionSimulationModel() const;
    ForceSimulationModel* forceSimulationModel() const;

    void setPositionSimulationModel(PositionSimulationModel* model);
    bool startPositionSimulation();
    void stopPositionSimulation();

    void setForceSimulationModel(ForceSimulationModel* model);
    bool startForceSimulation();
    void stopForceSimulation();

private:
    PositionSimulationModel* m_positionSimulationModel = nullptr;
    ForceSimulationModel* m_forceSimulationModel = nullptr;
    QThread* m_forceSimulationThread = nullptr;
};

#endif // SIMULATIONWORKER_H
