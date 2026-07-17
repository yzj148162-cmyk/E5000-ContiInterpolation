#include "udpcommworker.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QTimer>
#include <QUdpSocket>

#pragma execution_character_set("utf-8")

UdpCommWorker::UdpCommWorker(QObject* parent)
    : QObject(parent)
{
    sendTimer = new QTimer(this);
    sendTimer->setInterval(1000);
    connect(sendTimer, &QTimer::timeout, this, &UdpCommWorker::sendStatusNow);
    stats.receiveStatus = QStringLiteral("未启动");
    stats.parseStatus = QStringLiteral("未接收");
    stats.sendStatus = QStringLiteral("未发送");
}

UdpCommWorker::~UdpCommWorker()
{
    stopListening();
}

void UdpCommWorker::ensureSocket()
{
    if(socket){
        return;
    }
    socket = new QUdpSocket(this);
    connect(socket, &QUdpSocket::readyRead, this, &UdpCommWorker::handleReadyRead);
    connect(socket, &QUdpSocket::errorOccurred, this, &UdpCommWorker::handleSocketError);
}

void UdpCommWorker::startListening(int localPort, QString remoteIp, int targetRemotePort)
{
    ensureSocket();
    if(socket->isOpen()){
        socket->close();
    }

    QHostAddress parsedRemote(remoteIp.trimmed());
    if(parsedRemote.isNull()){
        parsedRemote = QHostAddress::LocalHost;
        stats.lastError = QStringLiteral("远端 IP 无效，已回退为 127.0.0.1");
    }
    remoteAddress = parsedRemote;
    remotePort = static_cast<quint16>(qBound(1, targetRemotePort, 65535));

    if(!socket->bind(QHostAddress::AnyIPv4,
                     static_cast<quint16>(qBound(1, localPort, 65535)),
                     QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)){
        setError(QStringLiteral("UDP 监听失败：%1").arg(socket->errorString()));
        stats.receiveStatus = QStringLiteral("监听失败");
        emitStats();
        return;
    }

    stats.receiveStatus = QStringLiteral("监听中");
    stats.parseStatus = QStringLiteral("等待数据");
    emitStats();
    emit displayInfoSignal(QStringLiteral("UDP 监听已启动：0.0.0.0:%1").arg(localPort),
                           QStringLiteral("normal"));
    emit displayInfoSignal(QStringLiteral("UDP 状态回传目标 = %1:%2")
                           .arg(remoteAddress.toString())
                           .arg(remotePort),
                           QStringLiteral("normal"));
}

void UdpCommWorker::stopListening()
{
    if(sendTimer){
        sendTimer->stop();
    }
    if(socket){
        socket->close();
        socket->deleteLater();
        socket = nullptr;
    }
    stats.receiveStatus = QStringLiteral("已停止");
    stats.sendStatus = QStringLiteral("已停止");
    emitStats();
}

void UdpCommWorker::setPeriodicSendEnabled(bool enabled)
{
    if(!sendTimer){
        return;
    }
    if(enabled){
        sendTimer->start();
        stats.sendStatus = QStringLiteral("周期发送中");
    }
    else{
        sendTimer->stop();
        stats.sendStatus = QStringLiteral("周期发送已关闭");
    }
    emitStats();
}

void UdpCommWorker::setSendIntervalMs(int intervalMs)
{
    if(sendTimer){
        sendTimer->setInterval(qMax(20, intervalMs));
    }
}

void UdpCommWorker::updateStatusPayload(UdpStatusPayload payload)
{
    statusPayload = payload;
}

void UdpCommWorker::sendStatusNow()
{
    ensureSocket();
    const QJsonDocument document(buildStatusObject());
    const QByteArray datagram = document.toJson(QJsonDocument::Compact);
    const qint64 sent = socket->writeDatagram(datagram, remoteAddress, remotePort);
    if(sent < 0){
        setError(QStringLiteral("UDP 状态发送失败：%1").arg(socket->errorString()));
        stats.sendStatus = QStringLiteral("发送失败");
    }
    else{
        ++stats.txCount;
        stats.lastTxTimeMs = UdpPacketTypes::nowMs();
        stats.sendStatus = QStringLiteral("发送成功");
    }
    emitStats();
}

void UdpCommWorker::handleReadyRead()
{
    if(!socket){
        return;
    }

    while(socket->hasPendingDatagrams()){
        QByteArray datagram;
        datagram.resize(static_cast<int>(socket->pendingDatagramSize()));
        QHostAddress sender;
        quint16 senderPort = 0;
        socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        ++stats.rxCount;
        stats.lastRxTimeMs = UdpPacketTypes::nowMs();
        stats.receiveStatus = QStringLiteral("已接收");
        stats.lastReceivedPacket = QString::fromUtf8(datagram.left(512));

        QString errorMessage;
        if(parseDatagram(datagram, errorMessage)){
            stats.parseStatus = QStringLiteral("解析成功");
            stats.lastError.clear();
        }
        else{
            ++stats.parseErrorCount;
            stats.parseStatus = QStringLiteral("解析失败");
            setError(errorMessage);
        }
        emitStats();
    }
}

void UdpCommWorker::handleSocketError()
{
    if(socket){
        setError(QStringLiteral("UDP socket 错误：%1").arg(socket->errorString()));
        emitStats();
    }
}

bool UdpCommWorker::parseDatagram(const QByteArray& datagram, QString& errorMessage)
{
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(datagram, &parseError);
    if(parseError.error != QJsonParseError::NoError || !document.isObject()){
        errorMessage = QStringLiteral("JSON 格式错误：%1").arg(parseError.errorString());
        return false;
    }

    const QJsonObject object = document.object();
    const QString type = object.value(QStringLiteral("type")).toString();
    if(type == QStringLiteral("pose_command")){
        UdpPoseCommand command;
        if(!parsePoseCommand(object, command, errorMessage)){
            return false;
        }
        emit poseCommandReceived(command);
        return true;
    }
    if(type == QStringLiteral("trajectory_chunk")){
        UdpTrajectoryChunk chunk;
        if(!parseTrajectoryChunk(object, chunk, errorMessage)){
            return false;
        }
        emit trajectoryChunkReceived(chunk);
        return true;
    }

    errorMessage = QStringLiteral("未知 UDP 包类型：%1").arg(type);
    return false;
}

bool UdpCommWorker::parsePoseCommand(const QJsonObject& object,
                                     UdpPoseCommand& command,
                                     QString& errorMessage) const
{
    const QJsonArray poseArray = object.value(QStringLiteral("pose")).toArray();
    if(poseArray.size() != 6){
        errorMessage = QStringLiteral("pose_command.pose 必须是 6 维数组，单位为 [mm, mm, mm, rad, rad, rad]");
        return false;
    }

    command.seq = static_cast<quint64>(object.value(QStringLiteral("seq")).toDouble(0.0));
    command.timestampMs = static_cast<qint64>(object.value(QStringLiteral("timestamp_ms")).toDouble(0.0));
    command.duration = object.value(QStringLiteral("duration")).toDouble(0.0);
    command.pose.clear();
    for(const QJsonValue& value : poseArray){
        if(!value.isDouble()){
            errorMessage = QStringLiteral("pose_command.pose 包含非数值元素");
            return false;
        }
        command.pose.push_back(value.toDouble());
    }
    return true;
}

bool UdpCommWorker::parseTrajectoryChunk(const QJsonObject& object,
                                         UdpTrajectoryChunk& chunk,
                                         QString& errorMessage) const
{
    const QJsonArray pointsArray = object.value(QStringLiteral("points")).toArray();
    if(pointsArray.isEmpty()){
        errorMessage = QStringLiteral("trajectory_chunk.points 不能为空");
        return false;
    }

    chunk.seq = static_cast<quint64>(object.value(QStringLiteral("seq")).toDouble(0.0));
    chunk.timestampMs = static_cast<qint64>(object.value(QStringLiteral("timestamp_ms")).toDouble(0.0));
    chunk.endNum = object.value(QStringLiteral("end_num")).toInt(0);
    chunk.final = object.value(QStringLiteral("final")).toBool(false);
    chunk.points.clear();

    for(const QJsonValue& pointValue : pointsArray){
        const QJsonArray pointArray = pointValue.toArray();
        if(pointArray.size() != 7){
            errorMessage = QStringLiteral("trajectory_chunk.points 每个点必须为 [t, x, y, z, rx, ry, rz]");
            return false;
        }
        QVector<double> point;
        for(const QJsonValue& value : pointArray){
            if(!value.isDouble()){
                errorMessage = QStringLiteral("trajectory_chunk.points 包含非数值元素");
                return false;
            }
            point.push_back(value.toDouble());
        }
        chunk.points.push_back(point);
    }
    return true;
}

QJsonObject UdpCommWorker::buildStatusObject() const
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
    object.insert(QStringLiteral("rx_count"), static_cast<double>(stats.rxCount));
    object.insert(QStringLiteral("tx_count"), static_cast<double>(stats.txCount));
    object.insert(QStringLiteral("parse_error_count"), static_cast<double>(stats.parseErrorCount));
    object.insert(QStringLiteral("last_error"), stats.lastError);
    return object;
}

void UdpCommWorker::emitStats()
{
    emit statsUpdated(stats);
}

void UdpCommWorker::setError(const QString& errorMessage)
{
    stats.lastError = errorMessage;
    emit displayInfoSignal(errorMessage, QStringLiteral("error"));
}
