#include "monitorthread.h"

#include <QSerialPort>

#pragma execution_character_set("utf-8")

MonitorThread::MonitorThread(QObject *parent)
    : QObject(parent)
{
}

MonitorThread::~MonitorThread()
{
    stopMonitoring();
}

void MonitorThread::setSerialConfig(const QString& portName, int baudRate)
{
    serialPortName = portName;
    serialBaudRate = baudRate;
}

void MonitorThread::startMonitoring()
{
    if (serialPort) {
        return;
    }

    serialPort = new QSerialPort(this);
    serialPort->setPortName(serialPortName);
    serialPort->setBaudRate(serialBaudRate);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);
    connect(serialPort, &QSerialPort::readyRead, this, &MonitorThread::handleReadyRead);
    connect(serialPort, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error){
        if (error != QSerialPort::NoError && serialPort) {
            emit displayInfoSignal(QString("监视串口错误：%1").arg(serialPort->errorString()), "error");
        }
    });

    if (!serialPort->open(QIODevice::ReadOnly)) {
        emit displayInfoSignal(QString("监视串口监听未启动：无法打开 %1，%2")
                                   .arg(serialPortName, serialPort->errorString()), "warning");
    } else {
        emit displayInfoSignal(QString("监视串口监听已启动：%1 @ %2")
                                   .arg(serialPortName)
                                   .arg(serialBaudRate), "normal");
    }

}

void MonitorThread::stopMonitoring()
{
    if (serialPort) {
        if (serialPort->isOpen()) {
            serialPort->close();
        }
        serialPort->deleteLater();
        serialPort = nullptr;
    }

    rxBuffer.clear();
}

void MonitorThread::handleReadyRead()
{
    if (!serialPort) {
        return;
    }

    const QByteArray data = serialPort->readAll();
    if (data.isEmpty()) {
        return;
    }

    rxBuffer.append(data);
    parseControlBoxFrames();

    if (rxBuffer.size() > 256) {
        emit displayInfoSignal("控制盒串口数据超过 256 字节且未解析，已清空缓存", "warning");
        rxBuffer.clear();
    }
}

void MonitorThread::parseControlBoxFrames()
{
    // 控制盒协议固定为二进制 5 字节帧：
    // [0]cmd=0x01, [1]运动控制, [2]速度置零, [3]零位校准, [4]低8位累加校验。
    while (true) {
        const int cmdIndex = rxBuffer.indexOf(char(0x01));
        if (cmdIndex < 0) {
            return;
        }
        if (rxBuffer.size() < cmdIndex + 5) {
            return;
        }

        const QByteArray frame = rxBuffer.mid(cmdIndex, 5);
        if (!isValidControlBoxFrame(frame)) {
            rxBuffer.remove(cmdIndex, 1);
            emit displayInfoSignal("控制盒状态帧校验失败或字段非法，已丢弃一个同步字节", "warning");
            continue;
        }

        const int motionControl = static_cast<unsigned char>(frame[1]);
        const int speedZero = static_cast<unsigned char>(frame[2]);
        const int zeroCalib = static_cast<unsigned char>(frame[3]);
        rxBuffer.remove(cmdIndex, 5);
        emit controlBoxStatusUpdated(motionControl, speedZero, zeroCalib);
    }
}

bool MonitorThread::isValidControlBoxFrame(const QByteArray& frame) const
{
    if (frame.size() != 5 || static_cast<unsigned char>(frame[0]) != 0x01) {
        return false;
    }

    const unsigned char motionControl = static_cast<unsigned char>(frame[1]);
    const unsigned char speedZero = static_cast<unsigned char>(frame[2]);
    const unsigned char zeroCalib = static_cast<unsigned char>(frame[3]);
    const unsigned char checksum = static_cast<unsigned char>(frame[4]);

    if (motionControl > 1 || speedZero > 1 || zeroCalib > 2) {
        return false;
    }

    const unsigned char expectedChecksum = static_cast<unsigned char>(
        (static_cast<unsigned char>(frame[0]) +
         motionControl +
         speedZero +
         zeroCalib) & 0xFF);
    return checksum == expectedChecksum;
}
