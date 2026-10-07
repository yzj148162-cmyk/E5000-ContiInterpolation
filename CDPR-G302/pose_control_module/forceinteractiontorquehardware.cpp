#include "forceinteractiontorquehardware.h"

#include "hardwareinterface.h"

namespace {
ForceInteractionTorqueBatchReport convert(
        const HardwareInterface::TorqueBatchResult& source)
{
    ForceInteractionTorqueBatchReport result;
    result.success = source.success;
    result.partialCommand = source.partialCommand;
    result.deferredBeforeWrite = source.deferredBeforeWrite;
    result.hardwareQueueWaitUs = source.hardwareQueueWaitUs;
    result.budgetCheckedUs = source.budgetCheckedUs;
    result.budgetRemainingUs = source.budgetRemainingUs;
    result.failedLogicalAxis = source.failedLogicalAxis;
    result.firstCommandMonotonicUs = source.firstCommandMonotonicUs;
    result.lastCommandMonotonicUs = source.lastCommandMonotonicUs;
    result.apiDurationUs = source.apiDurationUs;
    result.message = source.message;
    return result;
}
}

ForceInteractionTorqueHardware::ForceInteractionTorqueHardware(
        HardwareInterface* hardware)
    : hardware_(hardware)
{
}

void ForceInteractionTorqueHardware::setHardware(HardwareInterface* hardware)
{
    hardware_ = hardware;
}

ForceInteractionTorqueBatchReport ForceInteractionTorqueHardware::start(
        const std::vector<int>& axes, const std::vector<double>& torqueNm,
        qint64 deadlineUs, qint64 executionBudgetUs) const
{
    if(!hardware_){
        ForceInteractionTorqueBatchReport result;
        result.message = QStringLiteral("M3硬件协调器未绑定HardwareInterface");
        return result;
    }
    return convert(hardware_->motorTorqueStartBatchFast(
                       axes, torqueNm, deadlineUs, executionBudgetUs));
}

ForceInteractionTorqueBatchReport ForceInteractionTorqueHardware::update(
        const std::vector<int>& axes, const std::vector<double>& torqueNm,
        qint64 deadlineUs, qint64 executionBudgetUs) const
{
    if(!hardware_){
        ForceInteractionTorqueBatchReport result;
        result.message = QStringLiteral("M3硬件协调器未绑定HardwareInterface");
        return result;
    }
    return convert(hardware_->motorTorqueChangeBatchFast(
                       axes, torqueNm, deadlineUs, executionBudgetUs));
}

bool ForceInteractionTorqueHardware::returnToPositionHold(
        const std::vector<int>& axes) const
{
    return hardware_ && hardware_->motorTorqueReturnToPositionHold(axes);
}
