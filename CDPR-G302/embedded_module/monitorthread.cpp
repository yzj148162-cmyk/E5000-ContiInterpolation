/*
 * 文件总览：
 * - MonitorThread 的实现文件，管理串口打开/关闭、接收缓冲区和控制盒协议帧解析。
 * - parseControlBoxFrames 会从连续字节流中提取完整帧，避免半包/粘包影响状态判断。
 */

#include "monitorthread.h"

#include <QSerialPort>

#pragma execution_character_set("utf-8")

namespace {
bool isControlBoxIdleHeartbeatFrame(const QByteArray& frame)
{
    return frame.size() == 6 &&
            static_cast<unsigned char>(frame[0]) == 0x01 &&
            static_cast<unsigned char>(frame[1]) == 0x00 &&
            static_cast<unsigned char>(frame[2]) == 0x00 &&
            static_cast<unsigned char>(frame[3]) == 0x00 &&
            static_cast<unsigned char>(frame[4]) == 0x01 &&
            static_cast<unsigned char>(frame[5]) == 0x02;
}
}

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
            const QString detail = QStringLiteral("监视串口错误：%1").arg(serialPort->errorString());
            emit displayInfoSignal(detail, "error");
            emit serialCommunicationFault(detail);
        }
    });

    if (!serialPort->open(QIODevice::ReadOnly)) {
        const QString detail = QStringLiteral("监视串口监听未启动：无法打开 %1，%2")
                .arg(serialPortName, serialPort->errorString());
        emit displayInfoSignal(detail, "warning");
        emit serialCommunicationFault(detail);
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
    // Control box protocol is a fixed 6-byte binary frame:
    // [0]cmd=0x01, [1]motion, [2]speed zero, [3]zero calib,
    // [4]software emergency stop raw input (PA7: 0=pressed, 1=released),
    // [5]low-8-bit additive checksum.
    while (true) {
        const int cmdIndex = rxBuffer.indexOf(char(0x01));
        if (cmdIndex < 0) {
            return;
        }
        if (rxBuffer.size() < cmdIndex + 6) {
            return;
        }

        const QByteArray frame = rxBuffer.mid(cmdIndex, 6);
        if (!isValidControlBoxFrame(frame)) {
            rxBuffer.remove(cmdIndex, 1);
            emit displayInfoSignal("控制盒状态帧校验失败或字段非法，已丢弃一个同步字节", "warning");
            continue;
        }

        const int motionControl = static_cast<unsigned char>(frame[1]);
        const int speedZero = static_cast<unsigned char>(frame[2]);
        const int zeroCalib = static_cast<unsigned char>(frame[3]);
        const int softwareEmergencyStopRaw = static_cast<unsigned char>(frame[4]);
        const int softwareEmergencyStop = (softwareEmergencyStopRaw == 0) ? 1 : 0;
        rxBuffer.remove(cmdIndex, 6);
        emit controlBoxStatusUpdated(motionControl,
                                     speedZero,
                                     zeroCalib,
                                     softwareEmergencyStop);
    }
}

bool MonitorThread::isValidControlBoxFrame(const QByteArray& frame) const
{
    if (frame.size() != 6 || static_cast<unsigned char>(frame[0]) != 0x01) {
        return false;
    }
    if (isControlBoxIdleHeartbeatFrame(frame)) {
        return true;
    }

    const unsigned char motionControl = static_cast<unsigned char>(frame[1]);
    const unsigned char speedZero = static_cast<unsigned char>(frame[2]);
    const unsigned char zeroCalib = static_cast<unsigned char>(frame[3]);
    const unsigned char softwareEmergencyStopRaw = static_cast<unsigned char>(frame[4]);
    const unsigned char checksum = static_cast<unsigned char>(frame[5]);

    if (motionControl > 1 || speedZero > 1 || zeroCalib > 2 || softwareEmergencyStopRaw > 1) {
        return false;
    }

    const unsigned char expectedChecksum = static_cast<unsigned char>(
        (static_cast<unsigned char>(frame[0]) +
         motionControl +
         speedZero +
         zeroCalib +
         softwareEmergencyStopRaw) & 0xFF);
    return checksum == expectedChecksum;
}
