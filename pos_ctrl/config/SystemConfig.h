#ifndef SYSTEMCONFIG_H
#define SYSTEMCONFIG_H

#include <QtGlobal>

// 系统级固定参数统一收口在这里，避免魔法数字散落在各个线程里。
// 其中 UI 刷新周期支持在运行时通过界面修改。
struct SystemConfig
{
    qint32 hardwareCycleUs = 1000;
    qint32 uiRefreshMs = 50;
    qint32 plannerBatchSize = 16;
    qint32 plannerQueueLowWatermark = 24;
};

#endif // SYSTEMCONFIG_H
