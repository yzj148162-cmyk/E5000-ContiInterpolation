#include "forceinteractiontensionruntime.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace {

using Vector8d = RedundantTorqueAllocator::Vector8d;

bool allOperationEnabled(const std::array<bool, kForceInteractionCableCount>& values)
{
    return std::all_of(values.begin(), values.end(), [](bool value){ return value; });
}

bool within(const Vector8d& value, const Vector8d& lower, const Vector8d& upper,
            double tolerance = 0.0)
{
    return value.allFinite() &&
            (value.array() >= lower.array() - tolerance).all() &&
            (value.array() <= upper.array() + tolerance).all();
}

} // namespace

bool ForceInteractionTensionRuntimeConfig::validate(QString* errorMessage) const
{
    std::string feedbackError;
    std::string boundsError;
    const bool valid = feedback.enabled &&
            EightCableTensionFeedback::validate(feedback, &feedbackError) &&
            RedundantTorqueAllocator::validateConfig(bounds, &boundsError) &&
            effectiveRadiusM.allFinite() && (effectiveRadiusM.array() > 0.0).all() &&
            within(frozenTargetTensionN, bounds.tensionMinimum,
                   bounds.tensionMaximum, bounds.feasibilityTolerance) &&
            measuredSafetyMinimumN.allFinite() &&
            measuredSafetyMaximumN.allFinite() &&
            (measuredSafetyMinimumN.array() >= 0.0).all() &&
            (measuredSafetyMaximumN.array() <= 400.0).all() &&
            (measuredSafetyMinimumN.array() < measuredSafetyMaximumN.array()).all() &&
            (bounds.tensionMinimum.array() >= measuredSafetyMinimumN.array()).all() &&
            (bounds.tensionMaximum.array() <= measuredSafetyMaximumN.array()).all() &&
            std::isfinite(hardwareTorqueQuantumNm) &&
            std::abs(hardwareTorqueQuantumNm - 0.0345) <= 1.0e-9 &&
            std::isfinite(maximumEntryTargetDifferenceN) &&
            maximumEntryTargetDifferenceN > 0.0 &&
            innerPeriodUs == feedback.periodUs && innerPeriodUs == 5000;
    if(!valid && errorMessage){
        *errorMessage = QStringLiteral("M3配置无效：0525=%1，分配边界=%2；"
                                      "首版要求5 ms、0.0345 Nm量化、正有效半径，"
                                      "且分配边界位于实测安全边界内")
                .arg(QString::fromStdString(feedbackError),
                     QString::fromStdString(boundsError));
    }else if(errorMessage){
        errorMessage->clear();
    }
    return valid;
}

bool ForceInteractionTensionRuntime::reject(QString* errorMessage,
                                            const QString& message) const
{
    if(errorMessage) *errorMessage = message;
    return false;
}

void ForceInteractionTensionRuntime::updateDiagnostic(const QString& reason)
{
    diagnostic_.state = state_;
    diagnostic_.proposalSequence = proposalSequence_;
    diagnostic_.proposalPending = proposalPending_;
    if(!reason.isNull()) diagnostic_.reason = reason;
}

bool ForceInteractionTensionRuntime::prepare(
        const ForceInteractionTensionRuntimeConfig& config,
        QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::Idle &&
            state_ != ForceInteractionTensionState::Completed &&
            state_ != ForceInteractionTensionState::Fault){
        return reject(errorMessage, QStringLiteral("M3仅允许从空闲或终态重新准备"));
    }
    QString validationError;
    if(!config.validate(&validationError)) return reject(errorMessage, validationError);
    reset();
    config_ = config;
    state_ = ForceInteractionTensionState::Prepared;
    updateDiagnostic(QStringLiteral("M3配置与冻结目标已准备，尚未改变电机模式"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::beginTraceAcquisition(QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::Prepared){
        return reject(errorMessage, QStringLiteral("M3专用Trace只能在已准备状态申请"));
    }
    state_ = ForceInteractionTensionState::AcquiringReliableTrace;
    updateDiagnostic(QStringLiteral("等待M3同帧位置/状态/转矩/张力Trace"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::qualifyEntry(
        const ForceInteractionTensionEntrySnapshot& snapshot,
        QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::AcquiringReliableTrace){
        return reject(errorMessage, QStringLiteral("M3进入快照只能在Trace采集状态确认"));
    }
    if(!snapshot.reliable || !snapshot.sameFrame || snapshot.traceSequence == 0 ||
            snapshot.sampleUs <= 0 || !snapshot.actualPositionUnit.allFinite() ||
            !snapshot.actualTorqueNm.allFinite() ||
            !allOperationEnabled(snapshot.operationEnabled)){
        return reject(errorMessage, QStringLiteral("M3进入快照不可靠、不完整或驱动未全部使能"));
    }
    if(!within(snapshot.measuredTensionN, config_.measuredSafetyMinimumN,
               config_.measuredSafetyMaximumN)){
        return reject(errorMessage, QStringLiteral("M3进入张力超出冻结的全局准入边界"));
    }
    if(!within(snapshot.measuredTensionN, config_.bounds.tensionMinimum,
               config_.bounds.tensionMaximum)){
        return reject(errorMessage, QStringLiteral("M3进入张力不在静态分配器工作边界内"));
    }
    if(!within(snapshot.actualTorqueNm, config_.bounds.hardwareTorqueMinimum,
               config_.bounds.hardwareTorqueMaximum,
               config_.bounds.feasibilityTolerance)){
        return reject(errorMessage, QStringLiteral("M3进入实际转矩超出硬件边界"));
    }
    if((config_.frozenTargetTensionN - snapshot.measuredTensionN)
            .cwiseAbs().maxCoeff() > config_.maximumEntryTargetDifferenceN){
        return reject(errorMessage, QStringLiteral("M3冻结目标与进入张力差超过准入阈值"));
    }
    entry_ = snapshot;
    feedback_.initialize(snapshot.actualTorqueNm, snapshot.measuredTensionN,
                         snapshot.sampleUs);
    if(!feedback_.initialized()){
        return reject(errorMessage, QStringLiteral("M3张力反馈初始状态无效"));
    }
    state_ = ForceInteractionTensionState::EntryQualified;
    updateDiagnostic(QStringLiteral("M3进入快照已冻结，尚未下发转矩"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::beginTorqueStart(QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::EntryQualified){
        return reject(errorMessage, QStringLiteral("M3转矩启动要求进入快照已确认"));
    }
    state_ = ForceInteractionTensionState::TorqueStarting;
    updateDiagnostic(QStringLiteral("等待八轴事务式转矩启动结果"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::confirmTorqueStarted(QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::TorqueStarting){
        return reject(errorMessage, QStringLiteral("M3转矩启动确认状态不匹配"));
    }
    state_ = ForceInteractionTensionState::TargetTransition;
    updateDiagnostic(QStringLiteral("八轴转矩已启动，开始冻结目标平滑过渡"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::propose(
        qint64 sampleUs, const Vector8d& measuredTensionN,
        const Vector8d& lastCommittedTorqueNm, quint64 targetVersion,
        ForceInteractionTensionRuntimeDiagnostic& diagnostic,
        QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::TargetTransition &&
            state_ != ForceInteractionTensionState::StaticHolding){
        return reject(errorMessage, QStringLiteral("M3当前状态不允许生成转矩候选"));
    }
    if(proposalPending_){
        return reject(errorMessage, QStringLiteral("M3上一批转矩候选尚未提交或丢弃"));
    }
    pendingFeedback_ = feedback_;
    EightCableTensionFeedback::Diagnostic feedbackDiagnostic;
    std::string proposalError;
    if(!pendingFeedback_.propose(config_.feedback, config_.bounds,
                                 config_.effectiveRadiusM,
                                 config_.frozenTargetTensionN,
                                 measuredTensionN, lastCommittedTorqueNm,
                                 sampleUs, config_.hardwareTorqueQuantumNm,
                                 feedbackDiagnostic, proposalError,
                                 25000, targetVersion)){
        return reject(errorMessage, QStringLiteral("M3张力候选无效：%1")
                      .arg(QString::fromStdString(proposalError)));
    }
    proposalPending_ = true;
    ++proposalSequence_;
    diagnostic_.proposalValid = true;
    diagnostic_.feedback = feedbackDiagnostic;
    updateDiagnostic(QStringLiteral("M3转矩候选待硬件批次确认"));
    diagnostic = diagnostic_;
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::commitProposal(bool hardwareBatchSucceeded,
                                                     QString* errorMessage)
{
    if(!proposalPending_){
        return reject(errorMessage, QStringLiteral("M3没有待确认的转矩候选"));
    }
    proposalPending_ = false;
    if(!hardwareBatchSucceeded){
        diagnostic_.proposalValid = false;
        updateDiagnostic(QStringLiteral("M3硬件批次未提交，纯算法状态已丢弃"));
        if(errorMessage) errorMessage->clear();
        return true;
    }
    feedback_ = pendingFeedback_;
    diagnostic_.feedback.committed = true;
    diagnostic_.proposalValid = true;
    if(state_ == ForceInteractionTensionState::TargetTransition &&
            diagnostic_.feedback.targetBaseRemainingUs == 0){
        state_ = ForceInteractionTensionState::StaticHolding;
    }
    updateDiagnostic(state_ == ForceInteractionTensionState::StaticHolding ?
                     QStringLiteral("M3冻结目标静态保持") :
                     QStringLiteral("M3冻结目标过渡中"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::requestControlledStop(QString* errorMessage)
{
    if(proposalPending_){
        return reject(errorMessage, QStringLiteral("M3存在待确认批次，不能开始受控停止"));
    }
    if(state_ != ForceInteractionTensionState::TargetTransition &&
            state_ != ForceInteractionTensionState::StaticHolding){
        return reject(errorMessage, QStringLiteral("M3当前状态不能开始受控停止"));
    }
    state_ = ForceInteractionTensionState::CorrectionUnloading;
    updateDiagnostic(QStringLiteral("M3等待硬件层受控撤除张力修正"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::markCorrectionUnloaded(QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::CorrectionUnloading){
        return reject(errorMessage, QStringLiteral("M3尚未进入修正卸载状态"));
    }
    state_ = ForceInteractionTensionState::ReturningPositionHold;
    updateDiagnostic(QStringLiteral("M3等待以新鲜实际位置恢复位置保持"));
    if(errorMessage) errorMessage->clear();
    return true;
}

bool ForceInteractionTensionRuntime::markPositionHoldRestored(QString* errorMessage)
{
    if(state_ != ForceInteractionTensionState::ReturningPositionHold){
        return reject(errorMessage, QStringLiteral("M3尚未进入位置保持回退状态"));
    }
    state_ = ForceInteractionTensionState::Completed;
    updateDiagnostic(QStringLiteral("M3已恢复位置保持并完成"));
    if(errorMessage) errorMessage->clear();
    return true;
}

void ForceInteractionTensionRuntime::fail(const QString& reason)
{
    proposalPending_ = false;
    state_ = ForceInteractionTensionState::Fault;
    updateDiagnostic(reason.isEmpty() ? QStringLiteral("M3故障停止") : reason);
}

void ForceInteractionTensionRuntime::reset()
{
    state_ = ForceInteractionTensionState::Idle;
    config_ = ForceInteractionTensionRuntimeConfig{};
    entry_ = ForceInteractionTensionEntrySnapshot{};
    feedback_ = EightCableTensionFeedback{};
    pendingFeedback_ = EightCableTensionFeedback{};
    diagnostic_ = ForceInteractionTensionRuntimeDiagnostic{};
    proposalPending_ = false;
    proposalSequence_ = 0;
}

bool ForceInteractionTensionRuntime::runSelfChecks(QString* errorMessage)
{
    const auto failSelfCheck = [errorMessage](const QString& message){
        if(errorMessage) *errorMessage = message;
        return false;
    };
    ForceInteractionTensionRuntimeConfig config;
    config.feedback.enabled = true;
    config.feedback.independentTargetFeedforward = true;
    config.feedback.sharedExecutionReference = true;
    config.feedback.kp.setConstant(0.01);
    config.feedback.ki.setZero();
    config.feedback.kd.setZero();
    config.feedback.deadbandRatio = 0.0;
    config.feedback.torqueLimitNm = 3.0;
    config.feedback.slewNmPerSec = 6.0;
    config.feedback.periodUs = 5000;
    config.feedback.parameterSource = "M3 deterministic self-check";
    config.bounds.tensionMinimum.setConstant(10.0);
    config.bounds.tensionMaximum.setConstant(400.0);
    config.bounds.tensionBias.setConstant(20.0);
    config.bounds.tensionWeight.setOnes();
    config.bounds.hardwareTorqueMinimum.setConstant(-3.0);
    config.bounds.hardwareTorqueMaximum.setZero();
    config.bounds.hardwareDirection.setOnes();
    config.bounds.torqueSlewEnabled = true;
    config.bounds.hardwareTorqueSlewRate.setConstant(20.0);
    config.effectiveRadiusM.setConstant(0.02);
    config.frozenTargetTensionN.setConstant(21.0);
    config.measuredSafetyMinimumN.setConstant(5.0);
    config.measuredSafetyMaximumN.setConstant(400.0);

    ForceInteractionTensionRuntime runtime;
    QString error;
    if(!runtime.prepare(config, &error) ||
            !runtime.beginTraceAcquisition(&error)){
        return failSelfCheck(QStringLiteral("M3准备/Trace状态自检失败：%1").arg(error));
    }
    ForceInteractionTensionEntrySnapshot entry;
    entry.actualPositionUnit.setZero();
    entry.measuredTensionN.setConstant(20.0);
    entry.actualTorqueNm.setConstant(-0.4);
    entry.operationEnabled.fill(true);
    entry.traceSequence = 10;
    entry.sampleUs = 1000000;
    entry.sameFrame = true;
    entry.reliable = true;
    if(!runtime.qualifyEntry(entry, &error) ||
            !runtime.beginTorqueStart(&error) ||
            !runtime.confirmTorqueStarted(&error)){
        return failSelfCheck(QStringLiteral("M3进入/交接状态自检失败：%1").arg(error));
    }

    ForceInteractionTensionRuntimeDiagnostic proposal;
    const Vector8d committed = Vector8d::Constant(-0.4);
    if(!runtime.propose(1005000, entry.measuredTensionN, committed, 1,
                        proposal, &error) ||
            !proposal.proposalPending || !proposal.proposalValid){
        return failSelfCheck(QStringLiteral("M3候选生成自检失败：%1").arg(error));
    }
    if(!runtime.commitProposal(false, &error) ||
            runtime.diagnostic().feedback.committed){
        return failSelfCheck(QStringLiteral("M3失败批次回滚自检失败：%1").arg(error));
    }
    if(!runtime.propose(1005000, entry.measuredTensionN, committed, 1,
                        proposal, &error) ||
            !runtime.commitProposal(true, &error) ||
            !runtime.diagnostic().feedback.committed){
        return failSelfCheck(QStringLiteral("M3成功批次提交自检失败：%1").arg(error));
    }
    if(!runtime.requestControlledStop(&error) ||
            !runtime.markCorrectionUnloaded(&error) ||
            !runtime.markPositionHoldRestored(&error) ||
            runtime.state() != ForceInteractionTensionState::Completed){
        return failSelfCheck(QStringLiteral("M3受控停止状态自检失败：%1").arg(error));
    }

    ForceInteractionTensionRuntime invalid;
    config.innerPeriodUs = 6000;
    if(invalid.prepare(config, &error)){
        return failSelfCheck(QStringLiteral("M3非法周期未被拒绝"));
    }
    if(errorMessage) errorMessage->clear();
    return true;
}
