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

    explicit SingleMotorTrajectoryWorker(HardwareInterface* hardware, QObject* parent = nullptr);

public slots:
    void start();
    void stop();
    void setSelectedAxis(int axisIndex);
    void startTrajectory(const SingleMotorTrajectoryWorker::Command& command);
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
    void loop();

private:
    bool buildTrajectory(const Command& command,
                         QVector<double>& time,
                         QVector<double>& position,
                         QVector<double>& velocity,
                         QString& errorMessage) const;
    bool readRelativeTracePositions(int axisIndex,
                                    double& commandPosition,
                                    double& feedbackPosition) const;
    std::vector<HardwareInterface::MotorTracePositionSample>
    readRelativeTracePositionSamples(int axisIndex) const;
    bool readRelativePosition(int axisIndex, double& relativePosition) const;
    double initialPositionForCommand(const Command& command) const;
    double interpolateExpected(double elapsedSec) const;
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
