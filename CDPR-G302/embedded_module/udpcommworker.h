/*
 * 文件总览：
 * - UdpCommWorker 封装 UDP 收发，接收外部位姿/轨迹命令，并周期性发送当前系统状态。
 * - 数据包解析结果通过 Qt 信号交给 MainWindow，通信统计用于 UI 显示和排查网络/格式问题。
 */

#ifndef UDPCOMMWORKER_H
#define UDPCOMMWORKER_H

#include <QObject>
#include <QHostAddress>

#include "udppackettypes.h"

class QUdpSocket;
class QThread;
class UdpFeedbackSender;

class UdpCommWorker : public QObject
{
    Q_OBJECT
public:
    // 创建 UDP worker 并初始化定时发送器。
    explicit UdpCommWorker(QObject* parent = nullptr);
    // 释放 socket 和定时器资源。
    ~UdpCommWorker() override;

public slots:
    // 绑定本地端口并设置远端地址，开始接收外部位姿/轨迹命令。
    void startListening(int localPort, QString remoteIp, int remotePort);
    // 停止监听并关闭 socket。
    void stopListening();
    // 启停周期性状态上报。
    void setPeriodicSendEnabled(bool enabled);
    // 设置状态上报间隔。
    void setSendIntervalMs(int intervalMs);
    // 更新下一次要发送的系统状态载荷。
    void updateStatusPayload(UdpStatusPayload payload);
    // 请求尽快发送当前状态；V9反馈模式下只标记下一周期优先发送，不立即发包。
    void sendStatusNow();

    void setPlatformFeedbackEnabled(bool enabled);

signals:
    void statsUpdated(UdpCommStats stats);
    void platformCommandReceived(UdpPlatformCommand command);
    void poseCommandReceived(UdpPoseCommand command);
    void trajectoryChunkReceived(UdpTrajectoryChunk chunk);
    void displayInfoSignal(QString info, QString type);

private slots:
    // 处理 socket 中所有待读数据报，并分派到 JSON 解析流程。
    void handleReadyRead();
    // 将 socket 错误转换为统计状态和提示信息。
    void handleSocketError();
    void handleFeedbackSendResult(bool success,
                                  QString status,
                                  QString errorMessage,
                                  qint64 txTimeMs);

signals:
    void configureFeedbackSender(QString remoteIp, int remotePort);
    void feedbackPeriodicSendEnabledChanged(bool enabled);
    void feedbackSendIntervalChanged(int intervalMs);
    void feedbackPlatformFeedbackEnabledChanged(bool enabled);
    void feedbackStatusPayloadUpdated(UdpStatusPayload payload);
    void feedbackSendNowRequested();
    void feedbackStopRequested();

private:
    // 解析单个 UDP 数据报，按 type 分派为位姿命令或轨迹分片。
    bool parseDatagram(const QByteArray& datagram,
                       QString& errorMessage,
                       bool updatePacketPreview);
    // 从 JSON 对象解析实时位姿命令。
    bool parsePoseCommand(const QJsonObject& object, UdpPoseCommand& command, QString& errorMessage) const;
    // 从 JSON 对象解析离散轨迹分片。
    bool parseTrajectoryChunk(const QJsonObject& object, UdpTrajectoryChunk& chunk, QString& errorMessage) const;
    // 向 UI 发出最新通信统计。
    void emitStats(bool force = false);
    // 记录解析/通信错误并更新统计。
    void setError(const QString& errorMessage);
    // 延迟创建 QUdpSocket 并连接信号槽。
    void ensureSocket();
    void startFeedbackSenderThread();
    void stopFeedbackSenderThread();

    bool parsePlatformCommandDatagram(const QByteArray& datagram,
                                      UdpPlatformCommand& command,
                                      QString& errorMessage) const;

    QUdpSocket* socket = nullptr;
    QThread* feedbackSenderThread = nullptr;
    UdpFeedbackSender* feedbackSender = nullptr;
    QHostAddress remoteAddress = QHostAddress::LocalHost;
    quint16 remotePort = 10093;
    bool platformProtocolDetected = false;
    bool platformFeedbackEnabled = false;
    bool periodicSendEnabled = false;
    bool hasLastJsonPoseCommand = false;
    quint64 lastJsonPoseSeq = 0;
    qint64 lastJsonPoseTimestampMs = 0;
    bool hasLastJsonTrajectoryChunk = false;
    quint64 lastJsonTrajectorySeq = 0;
    qint64 lastJsonTrajectoryTimestampMs = 0;
    bool lastJsonTrajectoryChunkWasFinal = false;
    int sendIntervalMs = 200;
    qint64 lastStatsEmitMs = 0;
    qint64 lastPacketPreviewUpdateMs = 0;
    qint64 lastErrorSignalMs = 0;
    QString lastErrorSignalMessage;
    UdpCommStats stats;
    UdpStatusPayload statusPayload;
};

#endif // UDPCOMMWORKER_H
