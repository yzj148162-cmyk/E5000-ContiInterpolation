#ifndef FEEDBACKDATA_H
#define FEEDBACKDATA_H

#include <QtGlobal>

// 硬件线程每个周期写入的一份反馈快照。
// UI 线程和规划线程都只读取这份结构，不直接访问底层驱动。
struct FeedbackData
{
    qint32 actualPosRaw = 0;
    qint32 targetPosRaw = 0;
    quint16 statusWord = 0;
    qint8 modeDisplay = 0;
    qint32 errorRaw = 0;
    // 统一给 UI 提供“当前运动时间轴”。
    // CSP 按已发送 1ms 点数累计，PVT 按 runIndex 对应的轨迹时间写入。
    double motionTimeS = 0.0;
    bool motionRunning = false;
    bool fault = false;
    bool boardInitialized = false;
};

#endif // FEEDBACKDATA_H
