#include "hardwareinterface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {
constexpr double kRuntimeTracePositionLimitGuardUnitLocal = 0.25;

qint64 jogFastMonotonicNowUs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}
}

void HardwareInterface::resetMotorVelBatchFastState(const std::vector<int>& motorIndex)
{
    return runOnHardwareThread([&]() {
    if(motorJogVelocityFastActive.size() != motorIdVec.size()){
        motorJogVelocityFastActive.assign(motorIdVec.size(), false);
    }
    if(motorIndex.empty()){
        std::fill(motorJogVelocityFastActive.begin(),
                  motorJogVelocityFastActive.end(),
                  false);
        return;
    }
    for(const int axis : motorIndex){
        if(axis >= 0 && axis < static_cast<int>(motorJogVelocityFastActive.size())){
            motorJogVelocityFastActive[axis] = false;
        }
    }
    });
}

bool HardwareInterface::motorVelBatchFast(const std::vector<int>& motorIndex,
                                           const std::vector<double>& velocity,
                                           double changeTimeSec,
                                           std::vector<double> currentAbsolutePosition)
{
    const bool ok = runOnHardwareThread([&]() -> bool {
    return motorVelBatchFastDirect(motorIndex,
                                   velocity,
                                   changeTimeSec,
                                   currentAbsolutePosition,
                                   nullptr,
                                   0,
                                   0,
                                   nullptr);
    });
    return ok;
}

bool HardwareInterface::motorVelBatchFastEndpointRemote(
        const std::vector<int>& motorIndex,
        const std::vector<double>& velocity,
        double changeTimeSec,
        const EndpointRemoteVelocitySafetyContext& safetyContext,
        qint64 maximumFeedbackAgeUs,
        quint64 sessionToken,
        EndpointRemoteVelocityCommandReport* commandReport)
{
    if(commandReport){
        commandReport->outcome =
                EndpointRemoteVelocityCommandOutcome::NotAttempted;
        commandReport->failureReason.clear();
        commandReport->hardwareThreadTaskStartUs = 0;
        commandReport->sdkCallStartUs = 0;
        commandReport->sdkCallEndUs = 0;
        commandReport->hardwareThreadTaskEndUs = 0;
    }
    const bool ok = runOnHardwareThread([&]() -> bool {
    if(commandReport){
        commandReport->hardwareThreadTaskStartUs = jogFastMonotonicNowUs();
    }
    std::vector<double> currentAbsolutePosition;
    currentAbsolutePosition.reserve(motorIndex.size());
    for(const int logicalAxis : motorIndex){
        currentAbsolutePosition.push_back(
                    logicalAxis >= 0 &&
                    logicalAxis < static_cast<int>(
                        safetyContext.motorPosition.size()) ?
                        safetyContext.motorPosition[logicalAxis] :
                        std::numeric_limits<double>::quiet_NaN());
    }
    const bool directOk = motorVelBatchFastDirect(motorIndex,
                                                  velocity,
                                                  changeTimeSec,
                                                  currentAbsolutePosition,
                                                  &safetyContext,
                                                  maximumFeedbackAgeUs,
                                                  sessionToken,
                                                  commandReport);
    if(commandReport){
        commandReport->hardwareThreadTaskEndUs = jogFastMonotonicNowUs();
        if(directOk && commandReport->outcome ==
                EndpointRemoteVelocityCommandOutcome::NotAttempted){
            commandReport->outcome =
                    EndpointRemoteVelocityCommandOutcome::Succeeded;
        }
    }
    return directOk;
    });
    if(commandReport && commandReport->outcome ==
            EndpointRemoteVelocityCommandOutcome::NotAttempted){
        const bool taskStarted = commandReport->hardwareThreadTaskStartUs > 0;
        commandReport->outcome = taskStarted ?
                    EndpointRemoteVelocityCommandOutcome::InternalFailure :
                    EndpointRemoteVelocityCommandOutcome::HardwareThreadDispatchFailed;
        commandReport->failureReason = taskStarted ?
                    QStringLiteral("末端遥控八轴速度命令失败，但HardwareThread任务未返回明确原因") :
                    QStringLiteral("末端遥控八轴速度命令未能进入HardwareThread执行");
        emit displayInfoSignal(commandReport->failureReason.toStdString(), "error");
    }
    return ok;
}

HardwareInterface::EndpointRemoteTraceCommandResult
HardwareInterface::readRuntimeTraceAndMotorVelBatchFastEndpointRemote(
        const std::vector<int>& motorIndex,
        const std::vector<double>& velocity,
        double changeTimeSec,
        qint64 maximumFeedbackAgeUs,
        quint64 sessionToken,
        quint64 minimumLogicalFrameSequenceExclusive,
        EndpointRemoteCommandAdmissionProfile admissionProfile,
        qint64 planningStartedUs,
        qint64 planningCompletedUs,
        int actuationProfile)
{
    EndpointRemoteTraceCommandResult initial;
    EndpointRemoteVelocityCommandReport& initialReport =
            initial.commandReport;
    initialReport.beforeSubmitUs = jogFastMonotonicNowUs();
    initialReport.planningStartedUs = planningStartedUs;
    initialReport.planningCompletedUs = planningCompletedUs;
    initialReport.actuationProfile = actuationProfile;
    double commandSquaredNorm = 0.0;
    for(const double value : velocity){
        if(std::isfinite(value)){
            commandSquaredNorm += value * value;
            initialReport.commandMaximumAbsVelocity = std::max(
                        initialReport.commandMaximumAbsVelocity,
                        std::fabs(value));
        }
    }
    initialReport.commandL2Norm = std::sqrt(commandSquaredNorm);

    EndpointRemoteTraceCommandResult result = runOnHardwareThread(
                [&]() -> EndpointRemoteTraceCommandResult {
        EndpointRemoteTraceCommandResult taskResult = initial;
        EndpointRemoteVelocityCommandReport& report = taskResult.commandReport;
        report.compositeTaskStartUs = jogFastMonotonicNowUs();

        // 当前已经位于HardwareThread；readRuntimeTraceLatestSnapshot()内部的
        // runOnHardwareThread会直接执行，因此这里只发生一次Trace SDK读取且不嵌套排队。
        const qint64 traceCallStartUs = jogFastMonotonicNowUs();
        taskResult.traceSnapshot = readRuntimeTraceLatestSnapshot();
        report.traceReadCompletedUs = jogFastMonotonicNowUs();
        report.traceValidationCompletedUs = report.traceReadCompletedUs;
        report.hardwareThreadTaskStartUs = report.traceReadCompletedUs;
        taskResult.traceSnapshot.hardwareThreadQueueWaitUs =
                report.compositeTaskStartUs > report.beforeSubmitUs ?
                    report.compositeTaskStartUs - report.beforeSubmitUs : 0;
        if(taskResult.traceSnapshot.totalReadCallUs <= 0){
            taskResult.traceSnapshot.totalReadCallUs = std::max<qint64>(
                        0, report.traceReadCompletedUs - traceCallStartUs);
            taskResult.traceSnapshot.hardwareThreadExecutionUs =
                    taskResult.traceSnapshot.totalReadCallUs;
        }

        constexpr qint64 kEndpointRemoteMaximumFeedbackAgeUs = 5 * 1000;
        const qint64 permittedFeedbackAgeUs = std::min(
                    maximumFeedbackAgeUs,
                    kEndpointRemoteMaximumFeedbackAgeUs);
        const EndpointRemoteVelocitySafetyContext& context =
                taskResult.traceSnapshot.endpointRemoteVelocitySafety;
        if(context.monotonicUs > 0 && permittedFeedbackAgeUs > 0){
            report.commandDeadlineUs =
                    context.monotonicUs + permittedFeedbackAgeUs;
            report.entryFrameAgeUs = report.traceReadCompletedUs >=
                    context.monotonicUs ?
                        report.traceReadCompletedUs - context.monotonicUs : 0;
            report.remainingDeadlineBudgetUs =
                    report.commandDeadlineUs - report.traceReadCompletedUs;
        }

        const bool identityValid = sessionToken != 0 &&
                sessionToken == runtimeTraceEndpointRemoteSessionToken &&
                context.sessionToken == sessionToken &&
                activeRuntimeTraceUsageProfile ==
                    RuntimeTraceUsageProfile::EndpointRemoteRunning &&
                context.usageProfile ==
                    RuntimeTraceUsageProfile::EndpointRemoteRunning &&
                context.usageProfileGeneration ==
                    runtimeTraceUsageProfileGeneration &&
                context.configurationGeneration ==
                    runtimeTraceConfigurationGeneration;
        const qint64 futureToleranceUs = std::max<qint64>(
                    2 * 1000,
                    static_cast<qint64>(context.traceSamplePeriodUs) * 4);
        const bool freshnessValid = permittedFeedbackAgeUs > 0 &&
                context.fromTrace &&
                context.frameSequenceValid &&
                context.timingReliable &&
                context.fifoCaughtUp &&
                !context.traceLost &&
                context.traceSamplePeriodUs == 500 &&
                context.monotonicUs > 0 &&
                report.traceReadCompletedUs + futureToleranceUs >=
                    context.monotonicUs &&
                context.newestFrameAgeUs >= 0 &&
                context.newestFrameAgeUs <= permittedFeedbackAgeUs &&
                report.entryFrameAgeUs >= 0 &&
                report.entryFrameAgeUs <= permittedFeedbackAgeUs &&
                context.logicalFrameSequence >
                    minimumLogicalFrameSequenceExclusive;
        if(identityValid && !context.statusFaultLatched && !freshnessValid){
            const QString reason = QStringLiteral(
                        "末端遥控复合提交未取得新的5 ms内同帧Trace：周期=%1 us，入口帧龄=%2 us，上限=%3 us，逻辑序号=%4，上次规划帧=%5，FIFO追平=%6")
                    .arg(context.traceSamplePeriodUs)
                    .arg(report.entryFrameAgeUs)
                    .arg(permittedFeedbackAgeUs)
                    .arg(context.logicalFrameSequence)
                    .arg(minimumLogicalFrameSequenceExclusive)
                    .arg(context.fifoCaughtUp ? 1 : 0);
            report.failureReason = reason;
            report.hardwareThreadTaskEndUs = jogFastMonotonicNowUs();
            if(admissionProfile ==
                    EndpointRemoteCommandAdmissionProfile::
                        StartFromConfirmedZero){
                report.outcome =
                        EndpointRemoteVelocityCommandOutcome::FreshFrameDeferred;
                return taskResult;
            }
            report.outcome =
                    EndpointRemoteVelocityCommandOutcome::SafetyContextRejected;
            emit displayInfoSignal(reason.toStdString(), "error");
            return taskResult;
        }

        std::vector<double> currentAbsolutePosition;
        currentAbsolutePosition.reserve(motorIndex.size());
        for(const int logicalAxis : motorIndex){
            currentAbsolutePosition.push_back(
                        logicalAxis >= 0 &&
                        logicalAxis < static_cast<int>(
                            context.motorPosition.size()) ?
                            context.motorPosition[logicalAxis] :
                            std::numeric_limits<double>::quiet_NaN());
        }
        const bool commandSucceeded = motorVelBatchFastDirect(
                    motorIndex,
                    velocity,
                    changeTimeSec,
                    currentAbsolutePosition,
                    &context,
                    maximumFeedbackAgeUs,
                    sessionToken,
                    &report);
        report.sdkCalled = report.sdkCallStartUs > 0;
        report.hardwareThreadTaskEndUs = jogFastMonotonicNowUs();
        if(commandSucceeded && report.outcome ==
                EndpointRemoteVelocityCommandOutcome::NotAttempted){
            report.outcome = EndpointRemoteVelocityCommandOutcome::Succeeded;
        }
        return taskResult;
    });

    if(result.commandReport.compositeTaskStartUs <= 0){
        result.commandReport.outcome =
                EndpointRemoteVelocityCommandOutcome::
                    HardwareThreadDispatchFailed;
        result.commandReport.failureReason = QStringLiteral(
                    "末端遥控复合Trace/速度任务未能进入HardwareThread执行");
        emit displayInfoSignal(
                    result.commandReport.failureReason.toStdString(), "error");
    }
    return result;
}

bool HardwareInterface::motorVelBatchFastDirect(
        const std::vector<int>& motorIndex,
        const std::vector<double>& velocity,
        double changeTimeSec,
        const std::vector<double>& currentAbsolutePosition,
        const EndpointRemoteVelocitySafetyContext* endpointRemoteSafetyContext,
        qint64 maximumFeedbackAgeUs,
        quint64 endpointRemoteSessionToken,
        EndpointRemoteVelocityCommandReport* endpointRemoteCommandReport)
{
    const bool endpointRemoteCommand = endpointRemoteSafetyContext != nullptr;
    const auto failCommand = [&](const QString& message,
                                 EndpointRemoteVelocityCommandOutcome outcome) -> bool {
        const QString resolvedMessage = message.isEmpty() ?
                    QStringLiteral("末端遥控八轴速度命令失败，但底层原因为空") :
                    message;
        if(endpointRemoteCommand && endpointRemoteCommandReport &&
                endpointRemoteCommandReport->outcome ==
                    EndpointRemoteVelocityCommandOutcome::NotAttempted){
            endpointRemoteCommandReport->outcome = outcome;
            endpointRemoteCommandReport->failureReason = resolvedMessage;
        }
        emit displayInfoSignal(resolvedMessage.toStdString(), "error");
        return false;
    };
    const auto rejectEndpointRemoteContext = [&](const QString& reason) -> bool {
        return failCommand(
                    QStringLiteral("末端遥控八轴速度命令在下发前被拒绝：%1")
                        .arg(reason),
                    EndpointRemoteVelocityCommandOutcome::SafetyContextRejected);
    };

    if(endpointRemoteCommand){
        constexpr qint64 kEndpointRemoteMaximumFeedbackAgeUs = 5 * 1000;
        const EndpointRemoteVelocitySafetyContext& context =
                *endpointRemoteSafetyContext;
        const qint64 permittedFeedbackAgeUs = std::min(
                    maximumFeedbackAgeUs,
                    kEndpointRemoteMaximumFeedbackAgeUs);
        if(endpointRemoteSessionToken == 0 ||
                endpointRemoteSessionToken !=
                    runtimeTraceEndpointRemoteSessionToken ||
                context.sessionToken != endpointRemoteSessionToken){
            return rejectEndpointRemoteContext(QStringLiteral(
                        "Trace会话令牌与当前遥控会话不一致"));
        }
        if(activeRuntimeTraceUsageProfile !=
                RuntimeTraceUsageProfile::EndpointRemoteRunning ||
                context.usageProfile !=
                    RuntimeTraceUsageProfile::EndpointRemoteRunning){
            return rejectEndpointRemoteContext(QStringLiteral(
                        "Runtime Trace profile不是EndpointRemoteRunning"));
        }
        if(context.usageProfileGeneration !=
                    runtimeTraceUsageProfileGeneration ||
                context.configurationGeneration !=
                    runtimeTraceConfigurationGeneration){
            return rejectEndpointRemoteContext(QStringLiteral(
                        "Runtime Trace profile/config generation已变化"));
        }
        if(!runtimeTraceConfigured ||
                !runtimeTraceConfigReadbackValid ||
                permittedFeedbackAgeUs <= 0 ||
                !context.fromTrace ||
                !context.frameSequenceValid ||
                !context.timingReliable ||
                !context.fifoCaughtUp ||
                context.traceLost ||
                context.traceSamplePeriodUs != 500 ||
                context.newestFrameAgeUs < 0 ||
                context.newestFrameAgeUs > permittedFeedbackAgeUs ||
                context.monotonicUs <= 0){
            return rejectEndpointRemoteContext(QStringLiteral(
                        "同帧Trace来源/序号/时序/FIFO/丢帧/500us周期/年龄校验失败"
                        "（周期=%1 us，年龄=%2 us，上限=%3 us，逻辑序号=%4）")
                    .arg(context.traceSamplePeriodUs)
                    .arg(context.newestFrameAgeUs)
                    .arg(permittedFeedbackAgeUs)
                    .arg(context.logicalFrameSequence));
        }
        const qint64 nowUs = jogFastMonotonicNowUs();
        const qint64 futureToleranceUs = std::max<qint64>(
                    2 * 1000,
                    static_cast<qint64>(context.traceSamplePeriodUs) * 4);
        if(nowUs + futureToleranceUs < context.monotonicUs ||
                (nowUs >= context.monotonicUs &&
                 nowUs - context.monotonicUs > permittedFeedbackAgeUs)){
            return rejectEndpointRemoteContext(QStringLiteral(
                        "同帧Trace在HardwareThread命令入口已超过%1 us"
                        "（当前帧差=%2 us）")
                    .arg(permittedFeedbackAgeUs)
                    .arg(nowUs - context.monotonicUs));
        }
        if(context.statusFaultLatched){
            return rejectEndpointRemoteContext(QStringLiteral(
                        "排空帧中已锁存驱动状态异常：轴%1，0x6041=0x%2，"
                        "状态=%3，逻辑序号=%4")
                    .arg(context.statusFaultAxis + 1)
                    .arg(QString::number(context.statusFaultWord, 16)
                         .rightJustified(4, QLatin1Char('0')).toUpper())
                    .arg(context.statusFaultStateMachine)
                    .arg(context.statusFaultLogicalFrameSequence));
        }
    }

    if(motorIndex.size() != velocity.size()){
        return failCommand(
                    QString("Fast batch JOG velocity size mismatch: axes=%1 velocities=%2.")
                        .arg(static_cast<int>(motorIndex.size()))
                        .arg(static_cast<int>(velocity.size())),
                    EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
    }
    if(currentAbsolutePosition.size() != motorIndex.size()){
        return failCommand(
                    QString("Fast batch JOG position size mismatch: axes=%1 positions=%2.")
                        .arg(static_cast<int>(motorIndex.size()))
                        .arg(static_cast<int>(currentAbsolutePosition.size())),
                    EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
    }
    if(motorIndex.empty()){
        return true;
    }
    if(!isConnectLS){
        return failCommand(
                    QStringLiteral("Leadshine controller is not connected for fast batch JOG velocity command."),
                    EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
    }
    if(motorJogVelocityFastActive.size() != motorIdVec.size()){
        motorJogVelocityFastActive.assign(motorIdVec.size(), false);
    }

    struct BatchVelocityCommand {
        int logicalAxis = -1;
        int hardwareAxis = -1;
        double velocity = 0.0;
    };

    std::vector<BatchVelocityCommand> commands;
    commands.reserve(motorIndex.size());
    for(int axisColumn = 0; axisColumn < static_cast<int>(motorIndex.size()); ++axisColumn){
        const int logicalAxis = motorIndex[axisColumn];
        const double signedVelocity = velocity[axisColumn];
        const double currentPosition = currentAbsolutePosition[axisColumn];
        if(std::find(motorIndex.begin(), motorIndex.begin() + axisColumn, logicalAxis) !=
                motorIndex.begin() + axisColumn){
            return failCommand(
                        QString("Fast batch JOG command has duplicate logical axis %1.")
                            .arg(logicalAxis),
                        EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
        }
        if(!std::isfinite(signedVelocity) || !std::isfinite(currentPosition)){
            return failCommand(
                        QString("Fast batch JOG command for logical axis %1 has invalid velocity or position.")
                            .arg(logicalAxis),
                        EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
        }
        if(logicalAxis < 0 || logicalAxis >= static_cast<int>(motorComType.size())){
            return failCommand(
                        QString("Fast batch JOG command has invalid logical axis %1.")
                            .arg(logicalAxis),
                        EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
        }
        if(motorComType[logicalAxis] != COM_EC_LS){
            return failCommand(
                        QStringLiteral("Only Leadshine motor control is supported for fast batch JOG velocity command."),
                        EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
        }
        if(logicalAxis >= static_cast<int>(motorCurState.size()) ||
                !motorCurState[logicalAxis]){
            return failCommand(
                        QString("Fast batch JOG command rejected: %1 is not enabled in cache.")
                            .arg(axisDisplayName(logicalAxis)),
                        EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
        }

        const int hardwareAxis = resolveLeadshineAxisIndex(logicalAxis);
        if(hardwareAxis < 0){
            return failCommand(
                        QString("%1 has no valid controller axis.")
                            .arg(axisDisplayName(logicalAxis)),
                        EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
        }

        QString limitError;
        bool limitValid = false;
        if(endpointRemoteCommand){
            const EndpointRemoteVelocitySafetyContext& context =
                    *endpointRemoteSafetyContext;
            const bool contextAxisValid =
                    logicalAxis < static_cast<int>(
                        context.motorSafetyRelativePosition.size()) &&
                    logicalAxis < static_cast<int>(
                        context.motorSafetyRelativePositionSource.size()) &&
                    logicalAxis < static_cast<int>(context.motorStatusWord.size()) &&
                    logicalAxis < static_cast<int>(context.motorStateMachine.size());
            if(!contextAxisValid){
                return rejectEndpointRemoteContext(
                            QStringLiteral("轴%1同帧安全上下文字段不完整")
                            .arg(logicalAxis + 1));
            }
            const MotorSafetyRelativePositionSource positionSource =
                    context.motorSafetyRelativePositionSource[logicalAxis];
            const bool traceSafetySource =
                    positionSource ==
                        MotorSafetyRelativePositionSource::TraceCommandPersistentHome ||
                    positionSource ==
                        MotorSafetyRelativePositionSource::TraceCommandSessionHome ||
                    positionSource ==
                        MotorSafetyRelativePositionSource::TraceFeedbackSessionHome;
            if(!traceSafetySource ||
                    !std::isfinite(
                        context.motorSafetyRelativePosition[logicalAxis])){
                return rejectEndpointRemoteContext(
                            QStringLiteral("轴%1安全相对位置不是同帧Trace来源")
                            .arg(logicalAxis + 1));
            }
            if(context.motorStateMachine[logicalAxis] != 4){
                return rejectEndpointRemoteContext(
                            QStringLiteral("轴%1同帧0x6041=0x%2，状态=%3，要求=4")
                            .arg(logicalAxis + 1)
                            .arg(QString::number(
                                     context.motorStatusWord[logicalAxis], 16)
                                 .rightJustified(4, QLatin1Char('0')).toUpper())
                            .arg(context.motorStateMachine[logicalAxis]));
            }
            limitValid = validateVelocityMotorSoftwareLimitFromSnapshot(
                        logicalAxis,
                        context.motorSafetyRelativePosition[logicalAxis],
                        signedVelocity,
                        QStringLiteral("endpoint remote fast batch JOG velocity command"),
                        &limitError);
        }
        else{
            // 通用预设轨迹/点动路径保留原有主动取安全位置行为。
            limitValid = validateVelocityMotorSoftwareLimit(
                        logicalAxis,
                        currentPosition,
                        signedVelocity,
                        QStringLiteral("fast batch JOG velocity command"),
                        &limitError);
        }
        if(!limitValid){
            return failCommand(
                        limitError,
                        EndpointRemoteVelocityCommandOutcome::SoftwareLimitRejected);
        }

        BatchVelocityCommand command;
        command.logicalAxis = logicalAxis;
        command.hardwareAxis = hardwareAxis;
        command.velocity = signedVelocity;
        commands.push_back(command);
    }

    const double taccdec =
            (std::isfinite(changeTimeSec) && changeTimeSec >= 0.0) ?
                changeTimeSec :
                velChangeSpd;
    if(!std::isfinite(taccdec) || taccdec < 0.0){
        return failCommand(
                    QString("Fast batch JOG velocity command has invalid change time %1.")
                        .arg(taccdec, 0, 'f', 6),
                    EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
    }

    // 只在现有速度命令第一次真正调用板卡 SDK 前取一次时间，并在整批命令
    // 完成后取一次结束时间；不逐轴增加计时和硬件访问。
    const auto markSdkCallStarted = [&]() {
        if(endpointRemoteCommand && endpointRemoteCommandReport &&
                endpointRemoteCommandReport->sdkCallStartUs <= 0){
            endpointRemoteCommandReport->sdkCallStartUs =
                    jogFastMonotonicNowUs();
        }
    };
    auto startVelocityMove = [&](const BatchVelocityCommand& command) -> bool {
        const WORD hardwareAxis = static_cast<WORD>(command.hardwareAxis);
        const double signedVelocity = command.velocity;
        const double startTransitionTime = std::max(0.001, taccdec);
        markSdkCallStarted();
        const short clearStopReasonErr = dmc_clear_stop_reason(0, hardwareAxis);
        recordCommunicationEvent(true);
        const short profileErr = dmc_set_profile_unit(0,
                                                      hardwareAxis,
                                                      0.0,
                                                      std::max(std::fabs(signedVelocity), 1e-5),
                                                      startTransitionTime,
                                                      startTransitionTime,
                                                      0);
        recordCommunicationEvent(true);
        const short sErr = dmc_set_s_profile(0, hardwareAxis, 0, 0);
        recordCommunicationEvent(true);
        const short moveErr = dmc_vmove(0, hardwareAxis, signedVelocity >= 0.0 ? 1 : 0);
        recordCommunicationEvent(true);
        const bool ok = clearStopReasonErr == 0 && profileErr == 0 && sErr == 0 && moveErr == 0;
        if(!ok){
            failCommand(
                        QString("Fast batch JOG start failed on logical axis %1 hardware axis %2, error clear=%3 profile=%4 s=%5 move=%6.")
                            .arg(command.logicalAxis)
                            .arg(command.hardwareAxis)
                            .arg(clearStopReasonErr)
                            .arg(profileErr)
                            .arg(sErr)
                            .arg(moveErr),
                        EndpointRemoteVelocityCommandOutcome::SdkFailure);
        }
        return ok;
    };

    bool ok = true;
    for(const BatchVelocityCommand& command : commands){
        const int logicalAxis = command.logicalAxis;
        const WORD hardwareAxis = static_cast<WORD>(command.hardwareAxis);
        const double signedVelocity = command.velocity;
        if(logicalAxis < 0 || logicalAxis >= static_cast<int>(motorJogVelocityFastActive.size())){
            failCommand(
                        QString("Fast batch JOG command loop has invalid logical axis %1.")
                            .arg(logicalAxis),
                        EndpointRemoteVelocityCommandOutcome::CommandValidationRejected);
            ok = false;
            continue;
        }

        if(std::fabs(signedVelocity) <= 1e-9){
            if(motorJogVelocityFastActive[logicalAxis]){
                // 已进入速度模式后，过零仍保持同一 JOG 会话并在线改速；
                // 显式终止由 motorStop/急停入口负责。这样反向轨迹不会在
                // 每次速度过零时退出并重新 dmc_vmove。
                markSdkCallStarted();
                const short stopErr = dmc_change_speed_unit(0,
                                                             hardwareAxis,
                                                             0.0,
                                                             taccdec);
                recordCommunicationEvent(true);
                if(stopErr != 0){
                    const short doneState = dmc_check_done(0, hardwareAxis);
                    recordCommunicationEvent(false);
                    if(doneState != 0){
                        motorJogVelocityFastActive[logicalAxis] = false;
                    }
                    else{
                        failCommand(
                                    QString("Fast batch JOG zero-speed change failed on logical axis %1, error %2.")
                                        .arg(logicalAxis)
                                        .arg(stopErr),
                                    EndpointRemoteVelocityCommandOutcome::SdkFailure);
                        ok = false;
                    }
                }
            }
            continue;
        }

        if(!motorJogVelocityFastActive[logicalAxis]){
            const bool started = startVelocityMove(command);
            motorJogVelocityFastActive[logicalAxis] = started;
            ok = ok && started;
            continue;
        }

        markSdkCallStarted();
        const short speedErr = dmc_change_speed_unit(0, hardwareAxis, signedVelocity, taccdec);
        recordCommunicationEvent(true);
        if(speedErr != 0){
            const short doneState = dmc_check_done(0, hardwareAxis);
            recordCommunicationEvent(false);
            if(doneState != 0){
                motorJogVelocityFastActive[logicalAxis] = false;
                const bool restarted = startVelocityMove(command);
                motorJogVelocityFastActive[logicalAxis] = restarted;
                ok = ok && restarted;
            }
            else{
                failCommand(
                            QString("Fast batch JOG online speed change failed on logical axis %1, error %2.")
                                .arg(logicalAxis)
                                .arg(speedErr),
                            EndpointRemoteVelocityCommandOutcome::SdkFailure);
                ok = false;
            }
        }
    }
    if(endpointRemoteCommand && endpointRemoteCommandReport &&
            endpointRemoteCommandReport->sdkCallStartUs > 0){
        endpointRemoteCommandReport->sdkCallEndUs = jogFastMonotonicNowUs();
    }
    if(ok && endpointRemoteCommand && endpointRemoteCommandReport){
        endpointRemoteCommandReport->outcome =
                EndpointRemoteVelocityCommandOutcome::Succeeded;
    }
    return ok;
}

bool HardwareInterface::readMotorTracePositionUnit(int index, double& position)
{
    return runOnHardwareThread([&]() -> bool {
    position = std::numeric_limits<double>::quiet_NaN();
    if(!isConnectLS ||
            index < 0 ||
            index >= static_cast<int>(motorComType.size()) ||
            motorComType[index] != COM_EC_LS ||
            resolveLeadshineAxisIndex(index) < 0){
        return false;
    }

    const int frameCount = readRuntimeTraceCached(true);
    if(frameCount < 0 && !runtimeTraceEverRead){
        return false;
    }
    if(!ensureMotorTracePositionOffsets(index)){
        return false;
    }

    const std::vector<double> positions =
            currentMotorPositionCachedValues(std::vector<int>{index});
    if(positions.size() != 1 || !std::isfinite(positions.front())){
        return false;
    }

    double tracePosition = positions.front();
    if(hasValidMotorSoftwareLimit(index)){
        const double relativePosition = relativeMotorPosition(index, tracePosition);
        const double minPosition = motorSoftwareMinPos[index];
        const double maxPosition = motorSoftwareMaxPos[index];
        const bool traceOutsideLimit =
                !std::isfinite(relativePosition) ||
                relativePosition < minPosition ||
                relativePosition > maxPosition;
        if(traceOutsideLimit){
            double directPosition = 0.0;
            const bool directReadOk =
                    readMotorPositionUnitDirect(index, directPosition, false) &&
                    std::isfinite(directPosition);
            if(directReadOk){
                const double directRelativePosition = relativeMotorPosition(index, directPosition);
                const bool directInsideOrNearLimit =
                        std::isfinite(directRelativePosition) &&
                        directRelativePosition >=
                        minPosition - kRuntimeTracePositionLimitGuardUnitLocal &&
                        directRelativePosition <=
                        maxPosition + kRuntimeTracePositionLimitGuardUnitLocal;
                if(directInsideOrNearLimit){
                    if(index < static_cast<int>(motorActualTraceOffsetValid.size())){
                        motorActualTraceOffsetValid[index] = false;
                    }
                    if(index < static_cast<int>(motorCommandTraceOffsetValid.size())){
                        motorCommandTraceOffsetValid[index] = false;
                    }
                    return false;
                }
            }
            else{
                if(index < static_cast<int>(motorActualTraceOffsetValid.size())){
                    motorActualTraceOffsetValid[index] = false;
                }
                if(index < static_cast<int>(motorCommandTraceOffsetValid.size())){
                    motorCommandTraceOffsetValid[index] = false;
                }
                return false;
            }
        }
    }

    if(motorCurPos.size() != motorIdVec.size()){
        motorCurPos.assign(motorIdVec.size(), 0.0);
    }
    if(index < static_cast<int>(motorCurPos.size())){
        motorCurPos[index] = tracePosition;
    }
    position = tracePosition;
    return true;
    });
}
