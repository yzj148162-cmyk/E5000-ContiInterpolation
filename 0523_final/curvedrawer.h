#ifndef CURVEDRAWER_H
#define CURVEDRAWER_H

#include <QObject>
#include <QStringList>
#include <QTimer>
#include <QVector>

#include "qcustomplot.h"

class QWidget;

class CurveDrawer : public QObject
{
    Q_OBJECT

public:
    explicit CurveDrawer(QObject *parent = nullptr);
    ~CurveDrawer() override = default;

    void setChannelCounts(int motorControlChannelCount, int cableChannelCount);
    void initPlot(QCustomPlot *motorControlPlotWidget,
                  QCustomPlot *endEffectorPosPlotWidget,
                  QCustomPlot *endEffectorVelPlotWidget,
                  QCustomPlot *endEffectorAccPlotWidget,
                  QCustomPlot *cableTensionPlotWidget,
                  QCustomPlot *cableSpeedPlotWidget,
                  QCustomPlot *cableLengthPlotWidget);

signals:
    void updateMotorControlPlotSignal(double time, QVector<double> values);
    void updateEndEffectorPosPlotSignal(double time, QVector<double> position, QVector<double> orientation);
    void updateEndEffectorVelPlotSignal(double time, QVector<double> velocity, QVector<double> angularVelocity);
    void updateEndEffectorAccPlotSignal(double time, QVector<double> acceleration, QVector<double> angularAcceleration);
    void updateCableTensionPlotSignal(double time, QVector<double> tensions);
    void updateCableSpeedPlotSignal(double time, QVector<double> speeds);
    void updateCableLengthPlotSignal(double time, QVector<double> lengths);
    void clearAllDataSignal();

private slots:
    void handleUpdateMotorControlPlot(double time, QVector<double> values);
    void handleUpdateEndEffectorPosPlot(double time, QVector<double> position, QVector<double> orientation);
    void handleUpdateEndEffectorVelPlot(double time, QVector<double> velocity, QVector<double> angularVelocity);
    void handleUpdateEndEffectorAccPlot(double time, QVector<double> acceleration, QVector<double> angularAcceleration);
    void handleUpdateCableTensionPlot(double time, QVector<double> tensions);
    void handleUpdateCableSpeedPlot(double time, QVector<double> speeds);
    void handleUpdateCableLengthPlot(double time, QVector<double> lengths);
    void handleClearAllData();

private:
    void initConnections();
    void setupPlot(QCustomPlot *plot,
                   int graphCount,
                   const QString &yLabel,
                   const QStringList &graphNames);
    void rescalePlotYAxis(QCustomPlot *plot) const;

    static QStringList buildAxisNames(const QString &prefix, int count, int startIndex = 1);
    static QStringList buildPoseAxisNames();

    QCustomPlot *motorControlPlot = nullptr;
    QCustomPlot *endEffectorPosPlot = nullptr;
    QCustomPlot *endEffectorVelPlot = nullptr;
    QCustomPlot *endEffectorAccPlot = nullptr;
    QCustomPlot *cableTensionPlot = nullptr;
    QCustomPlot *cableSpeedPlot = nullptr;
    QCustomPlot *cableLengthPlot = nullptr;

    QTimer *plotTimer = nullptr;
    int motorControlChannelCount = 8;
    int cableChannelCount = 8;
    double timeWindow = 10.0;
};

#endif // CURVEDRAWER_H
