/*
 * 文件总览：
 * - 定义 UDP 通信使用的轻量数据结构：位姿命令、轨迹分片、通信统计和状态上报载荷。
 * - 所有结构体都注册为 Qt metatype，便于跨线程信号槽传递。
 * - 注释中的单位约定非常关键：位置为 mm，姿态为 rad，轨迹点首列为时间 s。
 */

#ifndef UDPPACKETTYPES_H
#define UDPPACKETTYPES_H

#include <QDateTime>
#include <QJsonArray>
#include <QJsonObject>
#include <QMetaType>
#include <QString>
#include <QVector>

struct UdpPlatformCommand
{
    qint64 receivedAtMs = 0;
    quint32 mark = 0x6001;
    quint32 cmd = 0;
    bool hasSeqTimestamp = false;
    quint32 seq = 0;
    quint32 timestampMs = 0;
    float surge = 0.0f;
    float sway = 0.0f;
    float heave = 0.0f;
    float velX = 0.0f;
    float velY = 0.0f;
    float velZ = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float angVelX = 0.0f;
    float angVelY = 0.0f;
    float angVelZ = 0.0f;
};

struct UdpPlatformFeedback
{
    qint64 timestampMs = 0;
    quint32 mark = 0x6001;
    quint32 state = 3;
    float surge = 0.0f;
    float sway = 0.0f;
    float heave = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float l1 = 0.0f;
    float l2 = 0.0f;
    float l3 = 0.0f;
    float l4 = 0.0f;
    float l5 = 0.0f;
    float l6 = 0.0f;
};

struct UdpPoseCommand
{
    quint64 seq = 0;
    qint64 timestampMs = 0;
    // Project pose units: x/y/z are millimeters, rx/ry/rz are radians.
    QVector<double> pose;
    double duration = 0.0;
    double stepMs = 0.0;
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
    QVector<double> forwardKinematicsEndPose;
    bool forwardKinematicsEndPoseValid = false;
    qint64 forwardKinematicsEndPoseTimestampMs = 0;
    int forwardKinematicsEndPoseEquationCount = 0;
    UdpPlatformFeedback platformFeedback;
};

Q_DECLARE_METATYPE(UdpPlatformCommand)
Q_DECLARE_METATYPE(UdpPlatformFeedback)
Q_DECLARE_METATYPE(UdpPoseCommand)
Q_DECLARE_METATYPE(UdpTrajectoryChunk)
Q_DECLARE_METATYPE(UdpCommStats)
Q_DECLARE_METATYPE(UdpStatusPayload)

namespace UdpPacketTypes {

constexpr quint32 kPlatformPacketMark = 0x6001;
constexpr int kPlatformPacketSize = 56;
constexpr int kPlatformCommandPacketSize = 64;

inline bool isPlatformCommandPacketSize(int size)
{
    return size == kPlatformPacketSize || size == kPlatformCommandPacketSize;
}

inline qint64 nowMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}

// 将 QVector<double> 转成 JSON 数组，供 UDP 状态和命令序列化复用。
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
