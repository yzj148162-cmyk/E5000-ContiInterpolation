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
    explicit MonitorThread(QObject *parent = nullptr);
    ~MonitorThread();
    void setSerialConfig(const QString& portName, int baudRate);

public slots:
    void startMonitoring();
    void stopMonitoring();

private slots:
    void handleReadyRead();

private:
    void parseControlBoxFrames();
    bool isValidControlBoxFrame(const QByteArray& frame) const;

    QSerialPort* serialPort = nullptr;

    QString serialPortName = QStringLiteral("COM3");
    int serialBaudRate = 115200;

    QByteArray rxBuffer;

signals:
    void controlBoxStatusUpdated(int motionControl, int speedZero, int zeroCalib);
    void displayInfoSignal(QString info, QString type);
};

#endif // MONITORTHREAD_H
