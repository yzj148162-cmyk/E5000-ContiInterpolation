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
    explicit MotorTorqueTestWorker(HardwareInterface* hardware, QObject* parent = nullptr);

public slots:
    void start();
    void stop();
    void setConfig(int axisIndex,
                   double targetTorque,
                   double relativeMinPos,
                   double relativeMaxPos,
                   double velocityLimit);
    void setTorqueActive(bool active);
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
    void loop();

private:
    struct Config {
        int axisIndex = -1;
        double targetTorque = 0.0;
        double relativeMinPos = 0.0;
        double relativeMaxPos = 0.0;
        double velocityLimit = 0.0;
        bool active = false;
    };

    Config currentConfig() const;
    void stopTorqueMotion(const QString& reason, const std::string& type = "warning");
    bool ensureTorqueCommand(const Config& config);
    static int torqueSign(double torque);

    HardwareInterface* hardwareInterface = nullptr;
    QTimer* timer = nullptr;
    mutable QMutex configMutex;
    Config config;
    bool commandStarted = false;
    int lastCommandAxis = -1;
    double lastCommandTorque = 0.0;
};

#endif // MOTORTORQUETESTWORKER_H
