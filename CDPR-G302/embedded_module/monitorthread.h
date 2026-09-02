/*
 * 文件总览：
 * - MonitorThread 读取外部控制盒串口数据，解析运动控制、速度清零、零点标定和软件急停状态。
 * - 解析后的状态通过信号交给 MainWindow，用于与 UI 按钮和运行状态联锁。
 */

#ifndef MONITORTHREAD_H
#define MONITORTHREAD_H

#include <QObject>
#include <QByteArray>
#include <QString>

class QSerialPort;

#pragma execution_character_set("utf-8")

class MonitorThread : public QObject
{
    Q_OBJECT

public:
    // 创建控制盒串口监控对象，默认端口和波特率可后续覆盖。
    explicit MonitorThread(QObject *parent = nullptr);
    // 停止串口并释放资源。
    ~MonitorThread();
    // 设置控制盒串口端口名和波特率。
    void setSerialConfig(const QString& portName, int baudRate);

public slots:
    // 打开串口并开始监听控制盒帧。
    void startMonitoring();
    // 停止监听并关闭串口。
    void stopMonitoring();

private slots:
    // 串口可读时追加到缓冲区并尝试解析完整帧。
    void handleReadyRead();

private:
    // 从接收缓冲区中拆帧并发出控制盒状态信号。
    void parseControlBoxFrames();
    // 校验单帧格式和校验位是否符合控制盒协议。
    bool isValidControlBoxFrame(const QByteArray& frame) const;

    QSerialPort* serialPort = nullptr;

    QString serialPortName = QStringLiteral("COM3");
    int serialBaudRate = 115200;

    QByteArray rxBuffer;

signals:
    void controlBoxStatusUpdated(int motionControl,
                                 int speedZero,
                                 int zeroCalib,
                                 int softwareEmergencyStop);
    void displayInfoSignal(QString info, QString type);
    void serialCommunicationFault(QString detail);
};

#endif // MONITORTHREAD_H
