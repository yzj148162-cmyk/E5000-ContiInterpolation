#ifndef FORCEINTERACTIONTORQUEHARDWARE_H
#define FORCEINTERACTIONTORQUEHARDWARE_H

#include "forceinteractiontypes.h"

#include <vector>

class HardwareInterface;

// 2026-10-07: Narrow M3 hardware coordinator.  It knows only the eight-axis
// transaction lifecycle; it never reads UI, F/T, Nokov, Newmark or PID state.
class ForceInteractionTorqueHardware
{
public:
    explicit ForceInteractionTorqueHardware(HardwareInterface* hardware = nullptr);
    void setHardware(HardwareInterface* hardware);

    ForceInteractionTorqueBatchReport start(
            const std::vector<int>& axes,
            const std::vector<double>& torqueNm,
            qint64 deadlineUs,
            qint64 executionBudgetUs) const;
    ForceInteractionTorqueBatchReport update(
            const std::vector<int>& axes,
            const std::vector<double>& torqueNm,
            qint64 deadlineUs,
            qint64 executionBudgetUs) const;
    bool returnToPositionHold(const std::vector<int>& axes) const;

private:
    HardwareInterface* hardware_ = nullptr;
};

#endif // FORCEINTERACTIONTORQUEHARDWARE_H
