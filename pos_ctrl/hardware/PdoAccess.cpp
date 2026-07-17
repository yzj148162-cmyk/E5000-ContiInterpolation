#include "hardware/PdoAccess.h"

PdoAccess PdoAccess::fromMotionConfig(const MotionConfig &config)
{
    PdoAccess access;
    access.rxTargetPosAddr = config.cspRxpdoPosAddr;
    access.txStatusAddr = config.cspTxpdoStatusAddr;
    access.txModeAddr = config.cspTxpdoModeAddr;
    access.txActualPosAddr = config.cspTxpdoPosAddr;
    return access;
}
