#ifndef MOTIVELOCALHANDLERTHREAD_H
#define MOTIVELOCALHANDLERTHREAD_H

#include <QObject>
#include <QTimer>
#include <QString>
#include <QtGlobal>

#include <vector>
#include <string>

#include "NokovMinimalClient.h"

#pragma execution_character_set("utf-8")

class MotiveLocalHandlerThread : public QObject
{
    Q_OBJECT

public:
    MotiveLocalHandlerThread();
    MotiveLocalHandlerThread(double _ctrlCycleMs, QString nokovIP = "10.1.1.198");
    ~MotiveLocalHandlerThread();

    void startTimer();
    void stopTimer();

    double ctrlCycleMs = 0.0;

    std::vector<std::vector<double>> getRigidPose();
    std::vector<std::vector<double>> calCableStartPos();
    bool hasCurrentRigidBody() const;
    bool hasRecentRigidBody(int maxAgeMs) const;
    int lastMarkerCount() const;

    bool isInit = false;
    bool extraInfo = false;
    bool detailInfo = false;

private:
    QTimer* timer = nullptr;
    bool isFirstLoop = true;
    bool isFirst = true;

    void threadLoop();
    void dataProcessor();

    // Nokov
    NokovMinimalClient* m_client = nullptr;
    bool m_isConnected = false;
    bool m_isPolling = false;
    qint64 m_lastPollTime = 0;
    int m_frameCount = 0;
    double m_averageInterval = 0.0;
    bool m_lastRigidBodyValid = false;
    int m_lastMarkerCount = 0;
    qint64 m_lastValidRigidBodyTimestampMs = -1;

    std::vector<std::vector<double>> rigidPose;
    std::vector<std::vector<double>> tempRigidPose;

signals:
    void displayInfoSignal(std::string info, std::string type);
    void dataUpdateSignal(std::vector<std::vector<double>> rigidPose);
};

#endif // MOTIVELOCALHANDLERTHREAD_H
