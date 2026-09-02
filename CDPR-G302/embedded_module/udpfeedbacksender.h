#ifndef UDPFEEDBACKSENDER_H
#define UDPFEEDBACKSENDER_H

#include <QObject>
#include <QHostAddress>

#include "udppackettypes.h"

class QTimer;
class QUdpSocket;

class UdpFeedbackSender : public QObject
{
    Q_OBJECT
public:
    explicit UdpFeedbackSender(QObject* parent = nullptr);
    ~UdpFeedbackSender() override;

public slots:
    void configureRemote(QString remoteIp, int remotePort);
    void setPeriodicSendEnabled(bool enabled);
    void setSendIntervalMs(int intervalMs);
    void setPlatformFeedbackEnabled(bool enabled);
    void updateStatusPayload(UdpStatusPayload payload);
    void sendNow();
    void stop();

signals:
    void sendResult(bool success, QString status, QString errorMessage, qint64 txTimeMs);

private:
    void ensureSocket();
    void sendPeriodicNow();
    bool sendDatagram(bool force);
    QJsonObject buildStatusObject() const;
    QByteArray buildPlatformFeedbackDatagram() const;

    QUdpSocket* socket = nullptr;
    QTimer* sendTimer = nullptr;
    QHostAddress remoteAddress = QHostAddress::LocalHost;
    quint16 remotePort = 10093;
    int sendIntervalMs = 200;
    bool periodicSendEnabled = false;
    bool platformFeedbackEnabled = false;
    bool prioritySendPending = false;
    qint64 lastSendTimeMs = 0;
    UdpStatusPayload statusPayload;
};

#endif // UDPFEEDBACKSENDER_H
