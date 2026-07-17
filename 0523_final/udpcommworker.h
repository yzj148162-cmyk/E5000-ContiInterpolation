#ifndef UDPCOMMWORKER_H
#define UDPCOMMWORKER_H

#include <QObject>
#include <QHostAddress>

#include "udppackettypes.h"

class QTimer;
class QUdpSocket;

class UdpCommWorker : public QObject
{
    Q_OBJECT
public:
    explicit UdpCommWorker(QObject* parent = nullptr);
    ~UdpCommWorker() override;

public slots:
    void startListening(int localPort, QString remoteIp, int remotePort);
    void stopListening();
    void setPeriodicSendEnabled(bool enabled);
    void setSendIntervalMs(int intervalMs);
    void updateStatusPayload(UdpStatusPayload payload);
    void sendStatusNow();

signals:
    void statsUpdated(UdpCommStats stats);
    void poseCommandReceived(UdpPoseCommand command);
    void trajectoryChunkReceived(UdpTrajectoryChunk chunk);
    void displayInfoSignal(QString info, QString type);

private slots:
    void handleReadyRead();
    void handleSocketError();

private:
    bool parseDatagram(const QByteArray& datagram, QString& errorMessage);
    bool parsePoseCommand(const QJsonObject& object, UdpPoseCommand& command, QString& errorMessage) const;
    bool parseTrajectoryChunk(const QJsonObject& object, UdpTrajectoryChunk& chunk, QString& errorMessage) const;
    void emitStats();
    void setError(const QString& errorMessage);
    void ensureSocket();
    QJsonObject buildStatusObject() const;

    QUdpSocket* socket = nullptr;
    QTimer* sendTimer = nullptr;
    QHostAddress remoteAddress = QHostAddress::LocalHost;
    quint16 remotePort = 5001;
    UdpCommStats stats;
    UdpStatusPayload statusPayload;
};

#endif // UDPCOMMWORKER_H
