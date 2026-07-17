#ifndef UDPPACKETTYPES_H
#define UDPPACKETTYPES_H

#include <QDateTime>
#include <QJsonArray>
#include <QJsonObject>
#include <QMetaType>
#include <QString>
#include <QVector>

struct UdpPoseCommand
{
    quint64 seq = 0;
    qint64 timestampMs = 0;
    // Project pose units: x/y/z are millimeters, rx/ry/rz are radians.
    QVector<double> pose;
    double duration = 0.0;
};

struct UdpTrajectoryChunk
{
    quint64 seq = 0;
    qint64 timestampMs = 0;
    int endNum = 0;
    // Each point uses project units: [t(s), x(mm), y(mm), z(mm), rx(rad), ry(rad), rz(rad)].
    QVector<QVector<double>> points;
    bool final = false;
};

struct UdpCommStats
{
    quint64 rxCount = 0;
    quint64 txCount = 0;
    quint64 parseErrorCount = 0;
    qint64 lastRxTimeMs = 0;
    qint64 lastTxTimeMs = 0;
    QString lastError;
    QString lastReceivedPacket;
    QString receiveStatus;
    QString parseStatus;
    QString sendStatus;
};

struct UdpStatusPayload
{
    quint64 seq = 0;
    bool systemRunning = false;
    bool pvtRunning = false;
    bool pvtPaused = false;
    QVector<double> motorPos;
    QVector<double> motorVel;
    QVector<double> force;
    QVector<double> expectedForce;
};

Q_DECLARE_METATYPE(UdpPoseCommand)
Q_DECLARE_METATYPE(UdpTrajectoryChunk)
Q_DECLARE_METATYPE(UdpCommStats)
Q_DECLARE_METATYPE(UdpStatusPayload)

namespace UdpPacketTypes {

inline qint64 nowMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}

inline QJsonArray vectorToJsonArray(const QVector<double>& values)
{
    QJsonArray array;
    for(double value : values){
        array.append(value);
    }
    return array;
}

} // namespace UdpPacketTypes

#endif // UDPPACKETTYPES_H
