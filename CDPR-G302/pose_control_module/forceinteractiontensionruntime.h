#ifndef FORCEINTERACTIONTENSIONRUNTIME_H
#define FORCEINTERACTIONTENSIONRUNTIME_H

#include "eightcabletensionfeedback.h"
#include "forceinteractiontypes.h"

#include <QString>

// 2026-10-07：M3静态张力保持的纯状态层。该类不访问HardwareInterface、
// Trace、Nokov或UI；硬件协调器必须在批量命令确认成功后才调用commitProposal()。
enum class ForceInteractionTensionState : quint8
{
    Idle = 0,
    Prepared,
    AcquiringReliableTrace,
    EntryQualified,
    TorqueStarting,
    TargetTransition,
    StaticHolding,
    CorrectionUnloading,
    ReturningPositionHold,
    Completed,
    Fault
};

struct ForceInteractionTensionRuntimeConfig
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EightCableTensionFeedback::Config feedback;
    RedundantTorqueAllocator::Config bounds;
    RedundantTorqueAllocator::Vector8d effectiveRadiusM =
            RedundantTorqueAllocator::Vector8d::Zero();
    RedundantTorqueAllocator::Vector8d frozenTargetTensionN =
            RedundantTorqueAllocator::Vector8d::Zero();
    // 实测安全边界来自D1/SafetyMonitor冻结快照；与分配器目标边界分开。
    RedundantTorqueAllocator::Vector8d measuredSafetyMinimumN =
            RedundantTorqueAllocator::Vector8d::Zero();
    RedundantTorqueAllocator::Vector8d measuredSafetyMaximumN =
            RedundantTorqueAllocator::Vector8d::Zero();
    double hardwareTorqueQuantumNm = 0.0345;
    double maximumEntryTargetDifferenceN = 100.0;
    double maximumEntryAbsVelocityUnitPerSec = 2.0;
    qint64 innerPeriodUs = 5000;

    bool validate(QString* errorMessage = nullptr) const;
};

struct ForceInteractionTensionEntrySnapshot
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RedundantTorqueAllocator::Vector8d actualPositionUnit =
            RedundantTorqueAllocator::Vector8d::Zero();
    RedundantTorqueAllocator::Vector8d actualVelocityUnitPerSec =
            RedundantTorqueAllocator::Vector8d::Zero();
    RedundantTorqueAllocator::Vector8d measuredTensionN =
            RedundantTorqueAllocator::Vector8d::Zero();
    RedundantTorqueAllocator::Vector8d actualTorqueNm =
            RedundantTorqueAllocator::Vector8d::Zero();
    std::array<bool, kForceInteractionCableCount> operationEnabled{};
    quint64 traceSequence = 0;
    qint64 sampleUs = 0;
    bool sameFrame = false;
    bool reliable = false;
};

struct ForceInteractionTensionRuntimeDiagnostic
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ForceInteractionTensionState state = ForceInteractionTensionState::Idle;
    quint64 proposalSequence = 0;
    bool proposalPending = false;
    bool proposalValid = false;
    EightCableTensionFeedback::Diagnostic feedback;
    QString reason;
};

class ForceInteractionTensionRuntime
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Vector8d = RedundantTorqueAllocator::Vector8d;

    bool prepare(const ForceInteractionTensionRuntimeConfig& config,
                 QString* errorMessage = nullptr);
    bool beginTraceAcquisition(QString* errorMessage = nullptr);
    bool qualifyEntry(const ForceInteractionTensionEntrySnapshot& snapshot,
                      QString* errorMessage = nullptr);
    bool beginTorqueStart(QString* errorMessage = nullptr);
    bool confirmTorqueStarted(QString* errorMessage = nullptr);

    // Produces a candidate only. Internal PID/slew state advances on a later
    // successful commit, so a rejected or deferred hardware batch is reversible.
    bool propose(qint64 sampleUs, const Vector8d& measuredTensionN,
                 const Vector8d& lastCommittedTorqueNm,
                 quint64 targetVersion,
                 ForceInteractionTensionRuntimeDiagnostic& diagnostic,
                 QString* errorMessage = nullptr);
    bool commitProposal(bool hardwareBatchSucceeded,
                        QString* errorMessage = nullptr);

    bool requestControlledStop(QString* errorMessage = nullptr);
    bool markCorrectionUnloaded(QString* errorMessage = nullptr);
    bool markPositionHoldRestored(QString* errorMessage = nullptr);
    void fail(const QString& reason);
    void reset();

    ForceInteractionTensionState state() const { return state_; }
    const ForceInteractionTensionRuntimeConfig& config() const { return config_; }
    const ForceInteractionTensionEntrySnapshot& entrySnapshot() const { return entry_; }
    const ForceInteractionTensionRuntimeDiagnostic& diagnostic() const { return diagnostic_; }

    static bool runSelfChecks(QString* errorMessage = nullptr);

private:
    bool reject(QString* errorMessage, const QString& message) const;
    void updateDiagnostic(const QString& reason = QString());

    ForceInteractionTensionState state_ = ForceInteractionTensionState::Idle;
    ForceInteractionTensionRuntimeConfig config_;
    ForceInteractionTensionEntrySnapshot entry_;
    EightCableTensionFeedback feedback_;
    EightCableTensionFeedback pendingFeedback_;
    ForceInteractionTensionRuntimeDiagnostic diagnostic_;
    bool proposalPending_ = false;
    quint64 proposalSequence_ = 0;
};

#endif // FORCEINTERACTIONTENSIONRUNTIME_H
