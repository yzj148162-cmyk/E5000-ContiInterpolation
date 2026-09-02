/*
 * 文件总览：
 * - UdpCommWorker 的实现文件，负责 UDP socket 生命周期、JSON 数据包解析、状态包组装和错误统计。
 * - 接收流程为 datagram -> JSON -> pose/trajectory 结构体 -> 信号；发送流程为状态缓存 -> JSON -> remote 地址。
 */

#include "udpcommworker.h"
#include "udpfeedbacksender.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QThread>
#include <QUdpSocket>

#include <cmath>
#include <cstring>
#include <limits>

#pragma execution_character_set("utf-8")

namespace {

constexpr int kUdpStatsEmitMinIntervalMs = 100;
constexpr int kUdpPacketPreviewMinIntervalMs = 100;
constexpr int kUdpErrorSignalMinIntervalMs = 1000;

quint32 readU32(const QByteArray& datagram, int offset)
{
    quint32 value = 0;
    std::memcpy(&value, datagram.constData() + offset, sizeof(value));
    return value;
}

float readFloat(const QByteArray& datagram, int offset)
{
    float value = 0.0f;
    std::memcpy(&value, datagram.constData() + offset, sizeof(value));
    return value;
}

bool parsePositiveIntegerField(const QJsonObject& object,
                               const QString& key,
                               qint64& value,
                               QString& errorMessage)
{
    const QJsonValue jsonValue = object.value(key);
    if(jsonValue.isUndefined() || jsonValue.isNull()){
        errorMessage = QStringLiteral("UDP JSON 字段缺失：%1").arg(key);
        return false;
    }

    if(jsonValue.isDouble()){
        const double number = jsonValue.toDouble();
        if(!std::isfinite(number) ||
                number < 1.0 ||
                std::floor(number) != number ||
                number > static_cast<double>(std::numeric_limits<qint64>::max())){
            errorMessage = QStringLiteral("UDP JSON 字段 %1 必须为正整数").arg(key);
            return false;
        }
        value = static_cast<qint64>(number);
        return true;
    }

    if(jsonValue.isString()){
        bool ok = false;
        const qint64 parsed = jsonValue.toString().toLongLong(&ok);
        if(ok && parsed > 0){
            value = parsed;
            return true;
        }
    }

    errorMessage = QStringLiteral("UDP JSON 字段 %1 必须为正整数").arg(key);
    return false;
}

bool parseFiniteDoubleValue(const QJsonValue& jsonValue,
                            const QString& fieldName,
                            double& value,
                            QString& errorMessage)
{
    if(!jsonValue.isDouble()){
        errorMessage = QStringLiteral("UDP JSON 字段 %1 必须为数值").arg(fieldName);
        return false;
    }

    value = jsonValue.toDouble();
    if(!std::isfinite(value)){
        errorMessage = QStringLiteral("UDP JSON 字段 %1 不是有限数值").arg(fieldName);
        return false;
    }
    return true;
}

} // namespace

UdpCommWorker::UdpCommWorker(QObject* parent)
    : QObject(parent)
{
    stats.receiveStatus = QStringLiteral("未启动");
    stats.parseStatus = QStringLiteral("未接收");
    stats.sendStatus = QStringLiteral("未发送");
}

UdpCommWorker::~UdpCommWorker()
{
    stopListening();
    stopFeedbackSenderThread();
}

void UdpCommWorker::startFeedbackSenderThread()
{
    if(feedbackSenderThread || feedbackSender){
        return;
    }

    feedbackSenderThread = new QThread();
    feedbackSender = new UdpFeedbackSender();
    feedbackSender->moveToThread(feedbackSenderThread);

    connect(feedbackSenderThread, &QThread::finished,
            feedbackSender, &QObject::deleteLater);
    connect(this, &UdpCommWorker::configureFeedbackSender,
            feedbackSender, &UdpFeedbackSender::configureRemote, Qt::QueuedConnection);
    connect(this, &UdpCommWorker::feedbackPeriodicSendEnabledChanged,
            feedbackSender, &UdpFeedbackSender::setPeriodicSendEnabled, Qt::QueuedConnection);
    connect(this, &UdpCommWorker::feedbackSendIntervalChanged,
            feedbackSender, &UdpFeedbackSender::setSendIntervalMs, Qt::QueuedConnection);
    connect(this, &UdpCommWorker::feedbackPlatformFeedbackEnabledChanged,
            feedbackSender, &UdpFeedbackSender::setPlatformFeedbackEnabled, Qt::QueuedConnection);
    connect(this, &UdpCommWorker::feedbackStatusPayloadUpdated,
            feedbackSender, &UdpFeedbackSender::updateStatusPayload, Qt::QueuedConnection);
    connect(this, &UdpCommWorker::feedbackSendNowRequested,
            feedbackSender, &UdpFeedbackSender::sendNow, Qt::QueuedConnection);
    connect(this, &UdpCommWorker::feedbackStopRequested,
            feedbackSender, &UdpFeedbackSender::stop, Qt::QueuedConnection);
    connect(feedbackSender, &UdpFeedbackSender::sendResult,
            this, &UdpCommWorker::handleFeedbackSendResult, Qt::QueuedConnection);

    feedbackSenderThread->start();
}

void UdpCommWorker::stopFeedbackSenderThread()
{
    UdpFeedbackSender* sender = feedbackSender;
    QThread* senderThread = feedbackSenderThread;
    feedbackSender = nullptr;
    feedbackSenderThread = nullptr;

    if(sender && senderThread && senderThread->isRunning()){
        QMetaObject::invokeMethod(sender, "stop", Qt::BlockingQueuedConnection);
        senderThread->quit();
        if(!senderThread->wait(1000)){
            senderThread->terminate();
            senderThread->wait(500);
        }
    }
    delete senderThread;
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
    startFeedbackSenderThread();
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
    hasLastJsonPoseCommand = false;
    lastJsonPoseSeq = 0;
    lastJsonPoseTimestampMs = 0;
    hasLastJsonTrajectoryChunk = false;
    lastJsonTrajectorySeq = 0;
    lastJsonTrajectoryTimestampMs = 0;
    lastJsonTrajectoryChunkWasFinal = false;
    emit configureFeedbackSender(remoteAddress.toString(), remotePort);
    emit feedbackSendIntervalChanged(sendIntervalMs);
    emit feedbackPlatformFeedbackEnabledChanged(platformFeedbackEnabled);
    emit feedbackStatusPayloadUpdated(statusPayload);

    if(!socket->bind(QHostAddress::AnyIPv4,
                     static_cast<quint16>(qBound(1, localPort, 65535)),
                     QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)){
        setError(QStringLiteral("UDP 监听失败：%1").arg(socket->errorString()));
        stats.receiveStatus = QStringLiteral("监听失败");
        emitStats(true);
        return;
    }

    stats.receiveStatus = QStringLiteral("监听中");
    stats.parseStatus = QStringLiteral("等待数据");
    emitStats(true);
    emit displayInfoSignal(QStringLiteral("UDP 监听已启动：0.0.0.0:%1").arg(localPort),
                           QStringLiteral("normal"));
    emit displayInfoSignal(QStringLiteral("UDP 状态回传目标 = %1:%2")
                           .arg(remoteAddress.toString())
                           .arg(remotePort),
                           QStringLiteral("normal"));
}

void UdpCommWorker::stopListening()
{
    periodicSendEnabled = false;
    if(feedbackSender){
        emit feedbackPeriodicSendEnabledChanged(false);
        emit feedbackStopRequested();
    }
    if(socket){
        socket->close();
        socket->deleteLater();
        socket = nullptr;
    }
    platformProtocolDetected = false;
    hasLastJsonPoseCommand = false;
    lastJsonPoseSeq = 0;
    lastJsonPoseTimestampMs = 0;
    hasLastJsonTrajectoryChunk = false;
    lastJsonTrajectorySeq = 0;
    lastJsonTrajectoryTimestampMs = 0;
    lastJsonTrajectoryChunkWasFinal = false;
    stats.receiveStatus = QStringLiteral("已停止");
    stats.sendStatus = QStringLiteral("已停止");
    emitStats(true);
}

void UdpCommWorker::setPeriodicSendEnabled(bool enabled)
{
    periodicSendEnabled = enabled;
    if(feedbackSender){
        emit feedbackPeriodicSendEnabledChanged(enabled);
    }
    if(enabled){
        stats.sendStatus = platformFeedbackEnabled ?
                    QStringLiteral("V9反馈周期发送中") :
                    QStringLiteral("JSON状态周期发送中");
    }
    else{
        stats.sendStatus = QStringLiteral("周期发送已关闭");
    }
    emitStats(true);
}

void UdpCommWorker::setSendIntervalMs(int intervalMs)
{
    sendIntervalMs = qMax(1, intervalMs);
    if(feedbackSender){
        emit feedbackSendIntervalChanged(sendIntervalMs);
    }
}

void UdpCommWorker::updateStatusPayload(UdpStatusPayload payload)
{
    statusPayload = payload;
    if(feedbackSender){
        emit feedbackStatusPayloadUpdated(statusPayload);
    }
}

void UdpCommWorker::setPlatformFeedbackEnabled(bool enabled)
{
    platformFeedbackEnabled = enabled;
    if(feedbackSender){
        emit feedbackPlatformFeedbackEnabledChanged(enabled);
    }
    stats.sendStatus = enabled ?
                QStringLiteral("仅发送V9反馈包") :
                QStringLiteral("发送JSON状态包");
    emitStats(true);
}

void UdpCommWorker::sendStatusNow()
{
    if(feedbackSender){
        emit feedbackSendNowRequested();
    }
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
        const qint64 nowMs = UdpPacketTypes::nowMs();
        stats.lastRxTimeMs = nowMs;
        stats.receiveStatus = QStringLiteral("已接收");
        const bool updatePacketPreview =
                lastPacketPreviewUpdateMs <= 0 ||
                (nowMs - lastPacketPreviewUpdateMs) >= kUdpPacketPreviewMinIntervalMs;
        if(updatePacketPreview){
            lastPacketPreviewUpdateMs = nowMs;
            stats.lastReceivedPacket = QString::fromUtf8(datagram.left(512));
        }

        QString errorMessage;
        if(parseDatagram(datagram, errorMessage, updatePacketPreview)){
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
        emitStats(true);
    }
}

bool UdpCommWorker::parseDatagram(const QByteArray& datagram,
                                  QString& errorMessage,
                                  bool updatePacketPreview)
{
    UdpPlatformCommand platformCommand;
    if(parsePlatformCommandDatagram(datagram, platformCommand, errorMessage)){
        platformProtocolDetected = true;
        if(updatePacketPreview){
            stats.lastReceivedPacket = QString::fromLatin1(datagram.toHex(' ').left(512));
        }
        emit platformCommandReceived(platformCommand);
        return true;
    }
    if(UdpPacketTypes::isPlatformCommandPacketSize(datagram.size())){
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(datagram, &parseError);
    if(parseError.error != QJsonParseError::NoError || !document.isObject()){
        errorMessage = QStringLiteral("JSON 格式错误：%1").arg(parseError.errorString());
        return false;
    }

    const QJsonObject object = document.object();
    const QJsonValue typeValue = object.value(QStringLiteral("type"));
    if(!typeValue.isString() || typeValue.toString().trimmed().isEmpty()){
        errorMessage = QStringLiteral("UDP JSON 类型字段缺失或不是字符串");
        return false;
    }

    const QString type = typeValue.toString().trimmed();
    if(type == QStringLiteral("pose_command")){
        UdpPoseCommand command;
        if(!parsePoseCommand(object, command, errorMessage)){
            return false;
        }
        if(hasLastJsonPoseCommand){
            if(command.seq == lastJsonPoseSeq &&
                    command.timestampMs == lastJsonPoseTimestampMs){
                errorMessage = QStringLiteral("位姿命令包重复，已拒绝：序号=%1，时间戳=%2")
                        .arg(command.seq)
                        .arg(command.timestampMs);
                return false;
            }
            if(command.timestampMs <= lastJsonPoseTimestampMs){
                errorMessage = QStringLiteral("位姿命令包时间戳必须递增，当前=%1，上次=%2")
                        .arg(command.timestampMs)
                        .arg(lastJsonPoseTimestampMs);
                return false;
            }
            if(command.seq <= lastJsonPoseSeq){
                errorMessage = QStringLiteral("位姿命令包序号必须递增，当前=%1，上次=%2")
                        .arg(command.seq)
                        .arg(lastJsonPoseSeq);
                return false;
            }
        }
        hasLastJsonPoseCommand = true;
        lastJsonPoseSeq = command.seq;
        lastJsonPoseTimestampMs = command.timestampMs;
        emit poseCommandReceived(command);
        return true;
    }
    if(type == QStringLiteral("trajectory_chunk")){
        UdpTrajectoryChunk chunk;
        if(!parseTrajectoryChunk(object, chunk, errorMessage)){
            return false;
        }
        if(hasLastJsonTrajectoryChunk){
            if(chunk.seq == lastJsonTrajectorySeq &&
                    chunk.timestampMs == lastJsonTrajectoryTimestampMs){
                errorMessage = QStringLiteral("轨迹分片包重复，已拒绝：序号=%1，时间戳=%2")
                        .arg(chunk.seq)
                        .arg(chunk.timestampMs);
                return false;
            }
            if(chunk.timestampMs <= lastJsonTrajectoryTimestampMs){
                errorMessage = QStringLiteral("轨迹分片包时间戳必须递增，当前=%1，上次=%2")
                        .arg(chunk.timestampMs)
                        .arg(lastJsonTrajectoryTimestampMs);
                return false;
            }
            if(!lastJsonTrajectoryChunkWasFinal && chunk.seq <= lastJsonTrajectorySeq){
                errorMessage = QStringLiteral("轨迹分片包序号必须递增，当前=%1，上次=%2")
                        .arg(chunk.seq)
                        .arg(lastJsonTrajectorySeq);
                return false;
            }
        }
        hasLastJsonTrajectoryChunk = true;
        lastJsonTrajectorySeq = chunk.seq;
        lastJsonTrajectoryTimestampMs = chunk.timestampMs;
        lastJsonTrajectoryChunkWasFinal = chunk.final;
        emit trajectoryChunkReceived(chunk);
        return true;
    }

    errorMessage = QStringLiteral("未知 UDP 包类型：%1").arg(type);
    return false;
}

bool UdpCommWorker::parsePlatformCommandDatagram(const QByteArray& datagram,
                                                 UdpPlatformCommand& command,
                                                 QString& errorMessage) const
{
    if(!UdpPacketTypes::isPlatformCommandPacketSize(datagram.size())){
        return false;
    }

    const quint32 mark = readU32(datagram, 0);
    if(mark != UdpPacketTypes::kPlatformPacketMark){
        errorMessage = QStringLiteral("平台二进制包头错误：0x%1")
                .arg(QString::number(mark, 16));
        return false;
    }

    command.receivedAtMs = UdpPacketTypes::nowMs();
    command.mark = mark;
    command.cmd = readU32(datagram, 4);
    int payloadOffset = 8;
    command.hasSeqTimestamp = datagram.size() == UdpPacketTypes::kPlatformCommandPacketSize;
    if(command.hasSeqTimestamp){
        command.seq = readU32(datagram, 8);
        command.timestampMs = readU32(datagram, 12);
        payloadOffset = 16;
    }
    command.surge = readFloat(datagram, payloadOffset + 0);
    command.sway = readFloat(datagram, payloadOffset + 4);
    command.heave = readFloat(datagram, payloadOffset + 8);
    command.velX = readFloat(datagram, payloadOffset + 12);
    command.velY = readFloat(datagram, payloadOffset + 16);
    command.velZ = readFloat(datagram, payloadOffset + 20);
    command.roll = readFloat(datagram, payloadOffset + 24);
    command.pitch = readFloat(datagram, payloadOffset + 28);
    command.yaw = readFloat(datagram, payloadOffset + 32);
    command.angVelX = readFloat(datagram, payloadOffset + 36);
    command.angVelY = readFloat(datagram, payloadOffset + 40);
    command.angVelZ = readFloat(datagram, payloadOffset + 44);
    return true;
}

bool UdpCommWorker::parsePoseCommand(const QJsonObject& object,
                                     UdpPoseCommand& command,
                                     QString& errorMessage) const
{
    qint64 seq = 0;
    if(!parsePositiveIntegerField(object, QStringLiteral("seq"), seq, errorMessage)){
        return false;
    }
    qint64 timestampMs = 0;
    if(!parsePositiveIntegerField(object, QStringLiteral("timestamp_ms"), timestampMs, errorMessage)){
        return false;
    }

    const QJsonArray poseArray = object.value(QStringLiteral("pose")).toArray();
    if(poseArray.size() != 6){
        errorMessage = QStringLiteral("位姿命令包的位姿数组（pose）必须是 6 维数组，单位为 [mm, mm, mm, rad, rad, rad]");
        return false;
    }

    const QJsonValue durationValue = object.value(QStringLiteral("duration"));
    if(durationValue.isUndefined() || durationValue.isNull()){
        errorMessage = QStringLiteral("位姿命令包的时长字段（duration）为必填字段，单位为 s");
        return false;
    }
    double duration = 0.0;
    if(!parseFiniteDoubleValue(durationValue,
                               QStringLiteral("duration"),
                               duration,
                               errorMessage)){
        return false;
    }
    if(duration <= 0.0){
        errorMessage = QStringLiteral("位姿命令包的时长字段（duration）必须大于 0，单位为 s");
        return false;
    }

    const QJsonValue stepMsValue = object.value(QStringLiteral("step_ms"));
    if(stepMsValue.isUndefined() || stepMsValue.isNull()){
        errorMessage = QStringLiteral("位姿命令包的步长字段（step_ms）为必填字段，单位为 ms");
        return false;
    }
    double stepMs = 0.0;
    if(!parseFiniteDoubleValue(stepMsValue,
                               QStringLiteral("step_ms"),
                               stepMs,
                               errorMessage)){
        return false;
    }
    if(stepMs < 1.0){
        errorMessage = QStringLiteral("位姿命令包的步长字段（step_ms）必须大于等于 1.0 ms");
        return false;
    }

    command.seq = static_cast<quint64>(seq);
    command.timestampMs = timestampMs;
    command.duration = duration;
    command.stepMs = stepMs;
    command.pose.clear();
    command.pose.reserve(6);
    int index = 0;
    for(const QJsonValue& value : poseArray){
        double parsed = 0.0;
        if(!parseFiniteDoubleValue(value,
                                   QStringLiteral("pose[%1]").arg(index),
                                   parsed,
                                   errorMessage)){
            return false;
        }
        command.pose.push_back(parsed);
        ++index;
    }
    return true;
}

bool UdpCommWorker::parseTrajectoryChunk(const QJsonObject& object,
                                         UdpTrajectoryChunk& chunk,
                                         QString& errorMessage) const
{
    qint64 seq = 0;
    if(!parsePositiveIntegerField(object, QStringLiteral("seq"), seq, errorMessage)){
        return false;
    }
    qint64 timestampMs = 0;
    if(!parsePositiveIntegerField(object, QStringLiteral("timestamp_ms"), timestampMs, errorMessage)){
        return false;
    }

    const QJsonArray pointsArray = object.value(QStringLiteral("points")).toArray();
    if(pointsArray.isEmpty()){
        errorMessage = QStringLiteral("轨迹分片包的轨迹点数组（points）不能为空");
        return false;
    }

    int endNum = 0;
    const QJsonValue endNumValue = object.value(QStringLiteral("end_num"));
    if(!endNumValue.isUndefined() && !endNumValue.isNull()){
        qint64 parsedEndNum = 0;
        if(!parsePositiveIntegerField(object, QStringLiteral("end_num"), parsedEndNum, errorMessage)){
            return false;
        }
        if(parsedEndNum > std::numeric_limits<int>::max()){
            errorMessage = QStringLiteral("轨迹分片包的末端数字段（end_num）超出整数范围");
            return false;
        }
        endNum = static_cast<int>(parsedEndNum);
    }

    const QJsonValue finalValue = object.value(QStringLiteral("final"));
    if(!finalValue.isUndefined() && !finalValue.isNull() && !finalValue.isBool()){
        errorMessage = QStringLiteral("轨迹分片包的结束标记（final）必须为布尔值");
        return false;
    }

    chunk.seq = static_cast<quint64>(seq);
    chunk.timestampMs = timestampMs;
    chunk.endNum = endNum;
    chunk.final = finalValue.toBool(false);
    chunk.points.clear();
    chunk.points.reserve(pointsArray.size());

    double previousTimeSec = -std::numeric_limits<double>::infinity();
    int pointIndex = 0;
    for(const QJsonValue& pointValue : pointsArray){
        const QJsonArray pointArray = pointValue.toArray();
        if(pointArray.size() != 7){
            errorMessage = QStringLiteral("轨迹分片包的轨迹点数组（points）中每个点必须为 [t, x, y, z, rx, ry, rz]");
            return false;
        }
        QVector<double> point;
        point.reserve(7);
        int valueIndex = 0;
        for(const QJsonValue& value : pointArray){
            double parsed = 0.0;
            if(!parseFiniteDoubleValue(value,
                                       QStringLiteral("points[%1][%2]").arg(pointIndex).arg(valueIndex),
                                       parsed,
                                       errorMessage)){
                return false;
            }
            point.push_back(parsed);
            ++valueIndex;
        }
        if(point[0] <= previousTimeSec){
            errorMessage = QStringLiteral("轨迹分片包的轨迹点时间必须严格递增，第 %1 个点时间=%2，上一个时间=%3")
                    .arg(pointIndex + 1)
                    .arg(point[0], 0, 'f', 6)
                    .arg(previousTimeSec, 0, 'f', 6);
            return false;
        }
        previousTimeSec = point[0];
        chunk.points.push_back(point);
        ++pointIndex;
    }
    return true;
}

void UdpCommWorker::handleFeedbackSendResult(bool success,
                                             QString status,
                                             QString errorMessage,
                                             qint64 txTimeMs)
{
    stats.sendStatus = status;
    if(success){
        ++stats.txCount;
        stats.lastTxTimeMs = txTimeMs;
    }
    else{
        setError(errorMessage);
    }
    emitStats();
}

void UdpCommWorker::emitStats(bool force)
{
    const qint64 nowMs = UdpPacketTypes::nowMs();
    if(!force &&
            lastStatsEmitMs > 0 &&
            (nowMs - lastStatsEmitMs) < kUdpStatsEmitMinIntervalMs){
        return;
    }
    lastStatsEmitMs = nowMs;
    emit statsUpdated(stats);
}

void UdpCommWorker::setError(const QString& errorMessage)
{
    stats.lastError = errorMessage;
    const qint64 nowMs = UdpPacketTypes::nowMs();
    if(lastErrorSignalMs > 0 &&
            (nowMs - lastErrorSignalMs) < kUdpErrorSignalMinIntervalMs){
        if(errorMessage != lastErrorSignalMessage){
            lastErrorSignalMessage = errorMessage;
        }
        return;
    }
    lastErrorSignalMs = nowMs;
    lastErrorSignalMessage = errorMessage;
    emit displayInfoSignal(errorMessage, QStringLiteral("error"));
}
