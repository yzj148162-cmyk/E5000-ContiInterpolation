#ifndef PDOACCESS_H
#define PDOACCESS_H

#include <QtGlobal>

#include "config/MotionConfig.h"

// 把手册里的 PDO 地址整理成一个小结构，便于 EtherCAT 接口统一引用。
struct PdoAccess
{
    quint16 rxTargetPosAddr = 1;
    quint16 txStatusAddr = 0;
    quint16 txModeAddr = 3;
    quint16 txActualPosAddr = 1;

    static PdoAccess fromMotionConfig(const MotionConfig &config);
};

#endif // PDOACCESS_H
