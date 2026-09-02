#include "udpfeedbacksender.h"

#include <QJsonDocument>
#include <QTimer>
#include <QUdpSocket>

#include <cstring>

#pragma execution_character_set("utf-8")

namespace {

constexpr int kV9FeedbackMinSendIntervalMs = 8;

void writeU32(QByteArray& datagram, int offset, quint32 value)
{
    std::memcpy(datagram.data() + offset, &value, sizeof(value));
}

void writeFloat(QByteArray& datagram, int offset, float value)
{
    std::memcpy(datagram.data() + offset, &value, sizeof(value));
}

} // namespace

UdpFeedbackSender::UdpFeedbackSender(QObject* parent)
    : QObject(parent)
{
    sendTimer = new QTimer(this);
    sendTimer->setTimerType(Qt::PreciseTimer);
    sendTimer->setInterval(sendIntervalMs);
    connect(sendTimer, &QTimer::timeout, this, &UdpFeedbackSender::sendPeriodicNow);
}

UdpFeedbackSender::~UdpFeedbackSender()
{
    stop();
}

void UdpFeedbackSender::ensureSocket()
{
    if(socket){
        return;
    }
    socket = new QUdpSocket(this);
}

void UdpFeedbackSender::configureRemote(QString remoteIp, int targetRemotePort)
{
    QHostAddress parsedRemote(remoteIp.trimmed());
    if(parsedRemote.isNull()){
        parsedRemote = QHostAddress::LocalHost;
    }

    remoteAddress = parsedRemote;
    remotePort = static_cast<quint16>(qBound(1, targetRemotePort, 65535));
    ensureSocket();
}

void UdpFeedbackSender::setPeriodicSendEnabled(bool enabled)
{
    periodicSendEnabled = enabled;
    if(!sendTimer){
        return;
    }

    if(periodicSendEnabled){
        sendTimer->start(sendIntervalMs);
    }
    else{
        sendTimer->stop();
    }
}

void UdpFeedbackSender::setSendIntervalMs(int intervalMs)
{
    sendIntervalMs = qMax(1, intervalMs);
    if(sendTimer){
        sendTimer->setInterval(sendIntervalMs);
        if(periodicSendEnabled && sendTimer->isActive()){
            sendTimer->start(sendIntervalMs);
        }
    }
}

void UdpFeedbackSender::setPlatformFeedbackEnabled(bool enabled)
{
    platformFeedbackEnabled = enabled;
    prioritySendPending = false;
    lastSendTimeMs = 0;
}

void UdpFeedbackSender::updateStatusPayload(UdpStatusPayload payload)
{
    statusPayload = payload;
}

void UdpFeedbackSender::sendNow()
{
    if(platformFeedbackEnabled){
        prioritySendPending = true;
        return;
    }

    sendDatagram(true);
}

void UdpFeedbackSender::sendPeriodicNow()
{
    if(!periodicSendEnabled){
        return;
    }

    const bool sent = sendDatagram(false);
    if(sent){
        prioritySendPending = false;
    }
}

bool UdpFeedbackSender::sendDatagram(bool force)
{
    ensureSocket();

    const qint64 nowMs = UdpPacketTypes::nowMs();
    if(platformFeedbackEnabled &&
            !force &&
            lastSendTimeMs > 0 &&
            nowMs - lastSendTimeMs < kV9FeedbackMinSendIntervalMs){
        return false;
    }

    const QByteArray datagram = platformFeedbackEnabled ?
                buildPlatformFeedbackDatagram() :
                QJsonDocument(buildStatusObject()).toJson(QJsonDocument::Compact);
    const qint64 sent = socket->writeDatagram(datagram, remoteAddress, remotePort);
    if(sent < 0){
        const QString error = platformFeedbackEnabled ?
                    QStringLiteral("UDP V9反馈发送失败：%1").arg(socket->errorString()) :
                    QStringLiteral("UDP JSON状态包发送失败：%1").arg(socket->errorString());
        const QString status = platformFeedbackEnabled ?
                    QStringLiteral("V9反馈发送失败") :
                    QStringLiteral("JSON状态包发送失败");
        emit sendResult(false, status, error, nowMs);
        return false;
    }

    lastSendTimeMs = nowMs;
    const QString status = platformFeedbackEnabled ?
                (prioritySendPending ?
                     QStringLiteral("V9反馈周期发送中，有待发送的即时更新") :
                     QStringLiteral("V9反馈周期发送中")) :
                QStringLiteral("JSON状态包已发送");
    emit sendResult(true, status, QString(), nowMs);
    return true;
}

void UdpFeedbackSender::stop()
{
    periodicSendEnabled = false;
    prioritySendPending = false;
    lastSendTimeMs = 0;
    if(sendTimer){
        sendTimer->stop();
    }
    if(socket){
        socket->close();
        socket->deleteLater();
        socket = nullptr;
    }
}

QByteArray UdpFeedbackSender::buildPlatformFeedbackDatagram() const
{
    const UdpPlatformFeedback& feedback = statusPayload.platformFeedback;
    QByteArray datagram;
    datagram.resize(UdpPacketTypes::kPlatformPacketSize);
    datagram.fill(char(0));
    writeU32(datagram, 0, UdpPacketTypes::kPlatformPacketMark);
    writeU32(datagram, 4, feedback.state);
    writeFloat(datagram, 8, feedback.surge);
    writeFloat(datagram, 12, feedback.sway);
    writeFloat(datagram, 16, feedback.heave);
    writeFloat(datagram, 20, feedback.roll);
    writeFloat(datagram, 24, feedback.pitch);
    writeFloat(datagram, 28, feedback.yaw);
    writeFloat(datagram, 32, feedback.l1);
    writeFloat(datagram, 36, feedback.l2);
    writeFloat(datagram, 40, feedback.l3);
    writeFloat(datagram, 44, feedback.l4);
    writeFloat(datagram, 48, feedback.l5);
    writeFloat(datagram, 52, feedback.l6);
    return datagram;
}

QJsonObject UdpFeedbackSender::buildStatusObject() const
{
    QJsonObject object;
    object.insert(QStringLiteral("type"), QStringLiteral("status"));
    object.insert(QStringLiteral("seq"), static_cast<double>(statusPayload.seq));
    object.insert(QStringLiteral("timestamp_ms"), static_cast<double>(UdpPacketTypes::nowMs()));
    object.insert(QStringLiteral("system_running"), statusPayload.systemRunning);
    object.insert(QStringLiteral("pvt_running"), statusPayload.pvtRunning);
    object.insert(QStringLiteral("pvt_paused"), statusPayload.pvtPaused);
    object.insert(QStringLiteral("motor_pos"), UdpPacketTypes::vectorToJsonArray(statusPayload.motorPos));
    object.insert(QStringLiteral("motor_vel"), UdpPacketTypes::vectorToJsonArray(statusPayload.motorVel));
    object.insert(QStringLiteral("force"), UdpPacketTypes::vectorToJsonArray(statusPayload.force));
    object.insert(QStringLiteral("expected_force"), UdpPacketTypes::vectorToJsonArray(statusPayload.expectedForce));
    object.insert(QStringLiteral("forward_kinematics_end_pose"),
                  UdpPacketTypes::vectorToJsonArray(statusPayload.forwardKinematicsEndPose));
    object.insert(QStringLiteral("forward_kinematics_end_pose_valid"),
                  statusPayload.forwardKinematicsEndPoseValid);
    object.insert(QStringLiteral("forward_kinematics_end_pose_timestamp_ms"),
                  static_cast<double>(statusPayload.forwardKinematicsEndPoseTimestampMs));
    object.insert(QStringLiteral("forward_kinematics_end_pose_equation_count"),
                  statusPayload.forwardKinematicsEndPoseEquationCount);
    return object;
}
