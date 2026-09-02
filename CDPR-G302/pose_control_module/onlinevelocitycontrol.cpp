#include "onlinevelocitycontrol.h"

#include "runtimefeatureswitches.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QSemaphore>
#include <QTextStream>
#include <QThread>
#include <QWaitCondition>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace {

constexpr qint64 kOnlineVelocityMaximumTraceFeedbackDelayUs = 5 * 1000;

double clampSymmetric(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

bool finiteAxisArray(const OnlineVelocityAxisArray& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

int finiteAxisCount(const OnlineVelocityAxisArray& values)
{
    return static_cast<int>(std::count_if(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    }));
}

struct OnlineVelocityRecord {
    OnlineVelocityStep step;
    qint64 apiDurationUs = 0;
    qint64 fullCycleDurationUs = 0;
    bool commandOk = false;
};

}

class OnlineVelocityCsvRecorder : public QThread
{
public:
    ~OnlineVelocityCsvRecorder() override
    {
        finishAndWait();
    }

    bool begin(const QString& directory,
               const OnlineVelocityPlan& plan,
               const OnlineVelocityConfig& config,
               QString* outputPath,
               QString* errorMessage)
    {
        finishAndWait();
        while(ready.tryAcquire(1)){
        }
        const QString resolvedDirectory = directory.trimmed().isEmpty() ?
                    QDir(QCoreApplication::applicationDirPath())
                        .filePath(QStringLiteral("online_velocity_logs")) :
                    directory;
        if(!QDir().mkpath(resolvedDirectory)){
            if(errorMessage){
                *errorMessage = QStringLiteral("无法创建在线速度记录目录：%1").arg(resolvedDirectory);
            }
            return false;
        }

        filePath = QDir(resolvedDirectory).filePath(
                    QStringLiteral("online_velocity_%1.csv")
                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
        sourceName = plan.sourceName;
        sourceName.replace('\r', ' ');
        sourceName.replace('\n', ' ');
        sourceName.replace(',', ';');
        planDurationSec = plan.durationSec();
        planSamplePeriodUs = plan.samplePeriodUs;
        runConfig = config;
        stopRequested = false;
        openSucceeded = false;
        openError.clear();
        dropped.store(0);
        {
            QMutexLocker locker(&queueMutex);
            queue.clear();
            queue.resize(kMaximumQueuedRecords);
            queueHead = 0;
            queuedRecordCount = 0;
        }
        start(QThread::LowPriority);
        if(!ready.tryAcquire(1, 1500)){
            requestFinish();
            wait();
            if(errorMessage){
                *errorMessage = QStringLiteral("在线速度记录线程启动超时");
            }
            return false;
        }
        if(!openSucceeded){
            wait();
            if(errorMessage){
                *errorMessage = openError;
            }
            return false;
        }
        if(outputPath){
            *outputPath = filePath;
        }
        return true;
    }

    void tryAppend(const OnlineVelocityRecord& record)
    {
        if(!isRunning() || stopRequested.load()){
            return;
        }
        if(!queueMutex.tryLock()){
            dropped.fetch_add(1);
            return;
        }
        if(queuedRecordCount >= kMaximumQueuedRecords){
            dropped.fetch_add(1);
            queueMutex.unlock();
            return;
        }
        const std::size_t writeIndex =
                (queueHead + queuedRecordCount) % kMaximumQueuedRecords;
        queue[writeIndex] = record;
        queuedRecordCount++;
        queueReady.wakeOne();
        queueMutex.unlock();
    }

    quint64 droppedCount() const
    {
        return dropped.load();
    }

    void requestFinish()
    {
        stopRequested.store(true);
        queueReady.wakeAll();
    }

    void finishAndWait()
    {
        if(!isRunning()){
            return;
        }
        requestFinish();
        wait();
    }

protected:
    void run() override
    {
        QFile file(filePath);
        openSucceeded = file.open(QIODevice::WriteOnly | QIODevice::Text);
        if(!openSucceeded){
            openError = QStringLiteral("无法打开在线速度记录文件：%1").arg(file.errorString());
            ready.release();
            return;
        }

        QTextStream stream(&file);
        stream.setRealNumberNotation(QTextStream::FixedNotation);
        stream.setRealNumberPrecision(9);
        stream << "# source_name=" << sourceName << '\n'
               << "# sample_period_us=" << planSamplePeriodUs
               << ",duration_s=" << planDurationSec << '\n'
               << "# feedforward_enabled=" << (runConfig.feedForwardEnabled ? 1 : 0)
               << ",feedforward_gain=" << runConfig.feedForwardGain
               << ",pid_enabled=" << (runConfig.pidEnabled ? 1 : 0)
               << ",kp=" << runConfig.kp
               << ",ki=" << runConfig.ki
               << ",kd=" << runConfig.kd << '\n'
               << "# integral_limit=" << runConfig.integralLimit
               << ",correction_velocity_limit=" << runConfig.maxCorrectionVelocity
               << ",maximum_velocity=" << runConfig.maxVelocity
               << ",maximum_acceleration=" << runConfig.maxAcceleration
               << ",online_change_time_s=" << runConfig.onlineChangeTimeSec
               << ",trace_timeout_us=" << runConfig.traceTimeoutUs << '\n'
               << "# trace_delay_calibration=excluded,delay_aligned_following_error_protection=excluded\n";
        stream << "wall_clock_us,monotonic_us,logical_frame,elapsed_s,api_us,full_cycle_us,command_ok";
        const char* groups[] = {"ref_pos", "act_pos", "ref_vel", "act_vel", "trace_cmd_vel",
                                "pos_error", "ff", "p", "i", "d", "cmd_vel"};
        for(const char* group : groups){
            for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
                stream << ',' << group << '_' << axis;
            }
        }
        stream << '\n';
        ready.release();

        for(;;){
            std::vector<OnlineVelocityRecord> batch;
            batch.reserve(128);
            queueMutex.lock();
            if(queuedRecordCount == 0 && !stopRequested.load()){
                queueReady.wait(&queueMutex, 100);
            }
            const std::size_t count = std::min<std::size_t>(queuedRecordCount, 128);
            for(std::size_t index = 0; index < count; ++index){
                batch.push_back(std::move(queue[queueHead]));
                queue[queueHead] = OnlineVelocityRecord{};
                queueHead = (queueHead + 1) % kMaximumQueuedRecords;
                queuedRecordCount--;
            }
            const bool finished = stopRequested.load() && queuedRecordCount == 0;
            queueMutex.unlock();

            for(const OnlineVelocityRecord& record : batch){
                const OnlineVelocityStep& step = record.step;
                stream << step.wallClockUs << ','
                       << step.monotonicUs << ','
                       << step.logicalFrameSequence << ','
                       << step.elapsedSec << ','
                       << record.apiDurationUs << ','
                       << record.fullCycleDurationUs << ','
                       << (record.commandOk ? 1 : 0);
                const OnlineVelocityAxisArray* values[] = {
                    &step.referencePosition, &step.actualPosition,
                    &step.referenceVelocity, &step.actualVelocity,
                    &step.tracedCommandVelocity, &step.positionError,
                    &step.feedForwardTerm, &step.pTerm, &step.iTerm,
                    &step.dTerm, &step.commandVelocity
                };
                for(const OnlineVelocityAxisArray* array : values){
                    for(double value : *array){
                        stream << ',' << value;
                    }
                }
                stream << '\n';
            }
            if(finished){
                break;
            }
        }
        stream.flush();
        file.close();
    }

private:
    static constexpr std::size_t kMaximumQueuedRecords = 4096;
    QString filePath;
    QString openError;
    QString sourceName;
    double planDurationSec = 0.0;
    int planSamplePeriodUs = 0;
    OnlineVelocityConfig runConfig;
    QMutex queueMutex;
    QWaitCondition queueReady;
    QSemaphore ready;
    std::vector<OnlineVelocityRecord> queue;
    std::size_t queueHead = 0;
    std::size_t queuedRecordCount = 0;
    std::atomic_bool stopRequested{false};
    std::atomic_bool openSucceeded{false};
    std::atomic<quint64> dropped{0};
};

bool OnlineVelocityPlan::validate(QString* errorMessage) const
{
    const auto fail = [&](const QString& message) {
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(timeSec.size() < 2 || position.size() != timeSec.size() || velocity.size() != timeSec.size()){
        return fail(QStringLiteral("在线速度轨迹的时间、位置和速度点数不一致，或轨迹点少于 2"));
    }
    if(!isSupportedOnlineVelocityPeriodUs(samplePeriodUs)){
        return fail(QStringLiteral("在线速度轨迹未记录有效的 1、2、5、10 或 20 ms 采样周期"));
    }
    if(!std::isfinite(timeSec.front()) || std::fabs(timeSec.front()) > 1.0e-9){
        return fail(QStringLiteral("在线速度轨迹必须从 0 秒开始"));
    }
    std::array<bool, kOnlineVelocityAxisCount> seen{};
    for(int column = 0; column < kOnlineVelocityAxisCount; ++column){
        const int axis = axes[column];
        if(axis != column || seen[axis]){
            return fail(QStringLiteral("在线速度轨迹必须按 0~7 顺序包含八个电机轴"));
        }
        seen[axis] = true;
    }
    const double expectedPeriodSec = static_cast<double>(samplePeriodUs) / 1000000.0;
    for(std::size_t point = 0; point < timeSec.size(); ++point){
        if(!std::isfinite(timeSec[point]) ||
                (point > 0 && timeSec[point] <= timeSec[point - 1])){
            return fail(QStringLiteral("在线速度轨迹时间戳必须严格递增"));
        }
        if(point > 0){
            const double intervalSec = timeSec[point] - timeSec[point - 1];
            const bool finalShortInterval = point + 1 == timeSec.size() &&
                    intervalSec <= expectedPeriodSec + 1.0e-9;
            if(!finalShortInterval &&
                    std::fabs(intervalSec - expectedPeriodSec) > 1.0e-9){
                return fail(QStringLiteral("在线速度轨迹时间间隔与记录的控制周期不一致"));
            }
        }
        if(!finiteAxisArray(position[point]) || !finiteAxisArray(velocity[point])){
            return fail(QStringLiteral("在线速度轨迹包含非有限位置或速度"));
        }
    }
    return true;
}

double OnlineVelocityPlan::durationSec() const
{
    return timeSec.empty() ? 0.0 : timeSec.back();
}

bool OnlineVelocityConfig::validate(QString* errorMessage) const
{
    const auto fail = [&](const QString& message) {
        if(errorMessage){
            *errorMessage = message;
        }
        return false;
    };
    if(!isSupportedOnlineVelocityPeriodUs(periodUs)){
        return fail(QStringLiteral("控制周期仅支持 1、2、5、10 或 20 ms"));
    }
    const double values[] = {feedForwardGain, kp, ki, kd, integralLimit,
                             maxCorrectionVelocity, maxVelocity, maxAcceleration,
                             onlineChangeTimeSec, startVelocityThreshold, endpointSettleSec};
    if(!std::all_of(std::begin(values), std::end(values), [](double value) {
        return std::isfinite(value) && value >= 0.0;
    })){
        return fail(QStringLiteral("在线速度控制参数必须为有限非负数"));
    }
    if(maxVelocity <= 0.0 || maxAcceleration <= 0.0 ||
            traceTimeoutUs <= 0 || initialTraceWaitTimeoutUs <= 0){
        return fail(QStringLiteral("最大速度、最大加速度和 Trace 超时必须大于 0"));
    }
    if(onlineChangeTimeSec < 0.001){
        return fail(QStringLiteral("在线改速时间不能小于 0.001 s"));
    }
    return true;
}

qint64 OnlineVelocityConfig::traceFeedbackDelayLimitUs() const
{
    return std::min({static_cast<qint64>(periodUs),
                     traceTimeoutUs,
                     kOnlineVelocityMaximumTraceFeedbackDelayUs});
}

OnlineVelocityControl::OnlineVelocityControl()
    : recorder(new OnlineVelocityCsvRecorder)
{
}

OnlineVelocityControl::~OnlineVelocityControl() = default;

bool OnlineVelocityControl::prepare(const OnlineVelocityPlan& newPlan,
                                    const OnlineVelocityConfig& newConfig,
                                    QString* errorMessage)
{
    if(isActive()){
        if(errorMessage){
            *errorMessage = QStringLiteral("在线速度控制仍在运行，不能覆盖轨迹");
        }
        return false;
    }
    if(!newPlan.validate(errorMessage) || !newConfig.validate(errorMessage)){
        return false;
    }
    if(newPlan.samplePeriodUs != newConfig.periodUs){
        if(errorMessage){
            *errorMessage = QStringLiteral("控制周期已改变，请按当前周期重新生成在线速度轨迹");
        }
        return false;
    }
    recorder->finishAndWait();
    plan = newPlan;
    config = newConfig;
    currentStatus = OnlineVelocityStatus{};
    currentStatus.state = OnlineVelocityStatus::State::Prepared;
    currentStatus.message = QStringLiteral("轨迹已准备，等待启动");
    currentStatus.durationSec = plan.durationSec();
    integral.fill(0.0);
    lastCommandVelocity.fill(0.0);
    motionStarted.fill(false);
    lastFeedbackMonotonicUs = 0;
    controlIntervalCount = 0;
    controlIntervalSumUs = 0;
    return true;
}

bool OnlineVelocityControl::start(qint64 nowUs, QString* errorMessage)
{
    if(currentStatus.state != OnlineVelocityStatus::State::Prepared){
        if(errorMessage){
            *errorMessage = QStringLiteral("请先准备在线速度轨迹");
        }
        return false;
    }
    QString recordPath;
    if(RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
        if(!recorder->begin(config.recordDirectory,
                            plan,
                            config,
                            &recordPath,
                            errorMessage)){
            return false;
        }
    }
    else{
        recorder->finishAndWait();
    }
    currentStatus.recordFile = recordPath;
    currentStatus.state = OnlineVelocityStatus::State::WaitingForTrace;
    currentStatus.message = QStringLiteral("等待第一帧可靠的八轴 Trace");
    waitStartUs = nowUs;
    trajectoryStartUs = 0;
    lastCommandUs = 0;
    lastFeedbackMonotonicUs = 0;
    lastGoodTraceUs = 0;
    nextDueUs = nowUs;
    lastUsedFrameSequence = 0;
    lastUsedFrameSequenceValid = false;
    integral.fill(0.0);
    lastCommandVelocity.fill(0.0);
    motionStarted.fill(false);
    return true;
}

void OnlineVelocityControl::resetTraceWaitClock(qint64 nowUs)
{
    if(currentStatus.state != OnlineVelocityStatus::State::WaitingForTrace){
        return;
    }
    waitStartUs = nowUs;
    lastGoodTraceUs = 0;
    nextDueUs = nowUs;
}

bool OnlineVelocityControl::feedbackReady(const OnlineVelocityFeedback& feedback) const
{
    const qint64 feedbackDelayLimitUs = config.traceFeedbackDelayLimitUs();
    return feedback.fromTrace &&
            feedback.frameSequenceValid &&
            feedback.timingReliable &&
            feedback.fifoCaughtUp &&
            !feedback.traceLost &&
            feedback.monotonicUs > 0 &&
            feedbackDelayLimitUs > 0 &&
            feedback.newestFrameAgeUs >= 0 &&
            feedback.newestFrameAgeUs <= feedbackDelayLimitUs &&
            finiteAxisArray(feedback.actualPosition) &&
            finiteAxisArray(feedback.actualVelocity) &&
            finiteAxisArray(feedback.tracedCommandVelocity);
}

void OnlineVelocityControl::interpolate(double time,
                                        OnlineVelocityAxisArray& positionOut,
                                        OnlineVelocityAxisArray& velocityOut) const
{
    if(time <= 0.0){
        positionOut = plan.position.front();
        velocityOut = plan.velocity.front();
        return;
    }
    if(time >= plan.timeSec.back()){
        positionOut = plan.position.back();
        velocityOut = plan.velocity.back();
        return;
    }
    const auto upper = std::upper_bound(plan.timeSec.begin(), plan.timeSec.end(), time);
    const std::size_t right = static_cast<std::size_t>(upper - plan.timeSec.begin());
    const std::size_t left = right - 1;
    const double span = plan.timeSec[right] - plan.timeSec[left];
    const double ratio = span > 0.0 ? (time - plan.timeSec[left]) / span : 0.0;
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        positionOut[axis] = plan.position[left][axis] +
                ratio * (plan.position[right][axis] - plan.position[left][axis]);
        velocityOut[axis] = plan.velocity[left][axis] +
                ratio * (plan.velocity[right][axis] - plan.velocity[left][axis]);
    }
}

OnlineVelocityStep OnlineVelocityControl::step(const OnlineVelocityFeedback& feedback,
                                               qint64 nowUs)
{
    OnlineVelocityStep result;
    if(!isActive()){
        return result;
    }
    if(nowUs < nextDueUs){
        return result;
    }
    while(nextDueUs <= nowUs){
        nextDueUs += config.periodUs;
    }

    const bool traceMetadataAvailable = feedback.fromTrace &&
            feedback.frameSequenceValid &&
            feedback.timingReliable &&
            feedback.fifoCaughtUp &&
            !feedback.traceLost;
    if(traceMetadataAvailable &&
            (feedback.traceSamplePeriodUs <= 0 ||
             config.periodUs % feedback.traceSamplePeriodUs != 0)){
        result.action = OnlineVelocityStep::Action::EmergencyStop;
        result.reason = QStringLiteral(
                    "在线速度控制周期必须是 Runtime Trace 采样周期的整数倍");
        setTerminalState(OnlineVelocityStatus::State::Fault, result.reason);
        recorder->requestFinish();
        return result;
    }

    const bool knownTraceIntegrityFault =
            currentStatus.state != OnlineVelocityStatus::State::WaitingForTrace &&
            feedback.fromTrace &&
            (!feedback.frameSequenceValid ||
             !feedback.timingReliable ||
             feedback.traceLost);
    if(knownTraceIntegrityFault){
        result.action = OnlineVelocityStep::Action::EmergencyStop;
        result.reason = QStringLiteral("Runtime Trace 时间轴不可信或已报告丢帧");
        setTerminalState(OnlineVelocityStatus::State::Fault, result.reason);
        recorder->requestFinish();
        return result;
    }

    const bool ready = feedbackReady(feedback);
    const bool feedbackFresh = ready &&
            (!lastUsedFrameSequenceValid ||
             feedback.logicalFrameSequence != lastUsedFrameSequence);
    if(feedbackFresh){
        lastGoodTraceUs = nowUs;
        lastUsedFrameSequence = feedback.logicalFrameSequence;
        lastUsedFrameSequenceValid = true;
    }
    else{
        const qint64 referenceUs = lastGoodTraceUs > 0 ? lastGoodTraceUs : waitStartUs;
        const qint64 feedbackTimeoutUs =
                currentStatus.state == OnlineVelocityStatus::State::WaitingForTrace ?
                    config.initialTraceWaitTimeoutUs : config.traceTimeoutUs;
        if(referenceUs > 0 && nowUs - referenceUs > feedbackTimeoutUs){
            result.action = OnlineVelocityStep::Action::EmergencyStop;
            result.reason = QStringLiteral(
                        "Runtime Trace等待超时：fromTrace=%1，序号有效=%2，时序可靠=%3，FIFO已追平=%4，丢帧=%5，帧年龄=%6 us，反馈时延允许上限=%7 us，逻辑序号=%8，位置/命令速度/实际速度有效轴=%9/%10/%11（各需8轴）")
                    .arg(feedback.fromTrace ? 1 : 0)
                    .arg(feedback.frameSequenceValid ? 1 : 0)
                    .arg(feedback.timingReliable ? 1 : 0)
                    .arg(feedback.fifoCaughtUp ? 1 : 0)
                    .arg(feedback.traceLost ? 1 : 0)
                    .arg(feedback.newestFrameAgeUs)
                    .arg(config.traceFeedbackDelayLimitUs())
                    .arg(feedback.logicalFrameSequence)
                    .arg(finiteAxisCount(feedback.actualPosition))
                    .arg(finiteAxisCount(feedback.tracedCommandVelocity))
                    .arg(finiteAxisCount(feedback.actualVelocity));
            setTerminalState(OnlineVelocityStatus::State::Fault, result.reason);
            recorder->requestFinish();
        }
        if(result.action == OnlineVelocityStep::Action::EmergencyStop ||
                currentStatus.state == OnlineVelocityStatus::State::WaitingForTrace ||
                !ready){
            return result;
        }
    }

    if(currentStatus.state == OnlineVelocityStatus::State::WaitingForTrace){
        actualStartPosition = feedback.actualPosition;
        trajectoryStartUs = nowUs;
        lastCommandUs = nowUs;
        currentStatus.state = OnlineVelocityStatus::State::Running;
        currentStatus.message = QStringLiteral("在线速度轨迹运行中");
    }

    const double elapsedSec = std::max(0.0,
            static_cast<double>(nowUs - trajectoryStartUs) / 1000000.0);
    if(elapsedSec >= plan.durationSec() + config.endpointSettleSec){
        result.action = OnlineVelocityStep::Action::NormalStop;
        result.reason = QStringLiteral("轨迹终点稳定时间已完成");
        setTerminalState(OnlineVelocityStatus::State::Completed, result.reason);
        recorder->requestFinish();
        return result;
    }

    result.action = OnlineVelocityStep::Action::CommandVelocity;
    result.wallClockUs = feedback.wallClockUs;
    result.monotonicUs = feedback.monotonicUs;
    result.logicalFrameSequence = feedback.logicalFrameSequence;
    result.elapsedSec = elapsedSec;
    result.actualPosition = feedback.actualPosition;
    result.actualVelocity = feedback.actualVelocity;
    result.tracedCommandVelocity = feedback.tracedCommandVelocity;
    const bool settling = elapsedSec >= plan.durationSec();
    interpolate(settling ? plan.durationSec() : elapsedSec,
                result.referencePosition,
                result.referenceVelocity);
    if(settling){
        result.referenceVelocity.fill(0.0);
        currentStatus.state = OnlineVelocityStatus::State::Settling;
        currentStatus.message = QStringLiteral("终点稳定中");
    }

    const double commandDtSec = std::max(
                static_cast<double>(config.periodUs) / 1000000.0,
                static_cast<double>(nowUs - lastCommandUs) / 1000000.0);
    result.controlIntervalUs = lastCommandUs > 0 ?
                std::max<qint64>(0, nowUs - lastCommandUs) : 0;
    double feedbackDtSec = feedback.traceSamplePeriodUs > 0 ?
                static_cast<double>(feedback.traceSamplePeriodUs) / 1000000.0 :
                commandDtSec;
    if(feedbackFresh && lastFeedbackMonotonicUs > 0 &&
            feedback.monotonicUs > lastFeedbackMonotonicUs){
        feedbackDtSec = static_cast<double>(
                    feedback.monotonicUs - lastFeedbackMonotonicUs) / 1000000.0;
    }
    if(feedbackFresh){
        lastFeedbackMonotonicUs = feedback.monotonicUs;
    }
    for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
        result.referencePosition[axis] = actualStartPosition[axis] +
                (result.referencePosition[axis] - plan.position.front()[axis]);
        const double error = result.referencePosition[axis] - result.actualPosition[axis];
        result.positionError[axis] = error;
        result.feedForwardTerm[axis] = config.feedForwardEnabled ?
                    config.feedForwardGain * result.referenceVelocity[axis] : 0.0;

        const double previousIntegral = integral[axis];
        if(config.pidEnabled){
            if(feedbackFresh){
                integral[axis] = clampSymmetric(integral[axis] + error * feedbackDtSec,
                                                config.integralLimit);
            }
            result.pTerm[axis] = config.kp * error;
            result.iTerm[axis] = config.ki * integral[axis];
            result.dTerm[axis] = config.kd *
                    (result.referenceVelocity[axis] - result.actualVelocity[axis]);
        }
        double correction = result.pTerm[axis] + result.iTerm[axis] + result.dTerm[axis];
        correction = clampSymmetric(correction, config.maxCorrectionVelocity);
        const double rawCommand = result.feedForwardTerm[axis] + correction;
        double command = clampSymmetric(rawCommand, config.maxVelocity);
        if(config.pidEnabled && command != rawCommand && error * rawCommand > 0.0){
            integral[axis] = previousIntegral;
            result.iTerm[axis] = config.ki * integral[axis];
            correction = clampSymmetric(result.pTerm[axis] + result.iTerm[axis] +
                                        result.dTerm[axis],
                                        config.maxCorrectionVelocity);
            command = clampSymmetric(result.feedForwardTerm[axis] + correction,
                                     config.maxVelocity);
        }
        const double maximumStep = config.maxAcceleration * commandDtSec;
        command = std::max(lastCommandVelocity[axis] - maximumStep,
                           std::min(lastCommandVelocity[axis] + maximumStep, command));
        if(!motionStarted[axis] &&
                std::fabs(command) < config.startVelocityThreshold){
            command = 0.0;
        }
        result.commandVelocity[axis] = command;
    }
    lastCommandVelocity = result.commandVelocity;
    lastCommandUs = nowUs;
    updateStatusFromStep(result);
    return result;
}

void OnlineVelocityControl::noteCommandResult(const OnlineVelocityStep& stepResult,
                                              bool commandOk,
                                              qint64 apiDurationUs,
                                              qint64 fullCycleDurationUs)
{
    if(stepResult.action == OnlineVelocityStep::Action::CommandVelocity){
        currentStatus.latestCommandApiUs = apiDurationUs;
        currentStatus.maximumCommandApiUs =
                std::max(currentStatus.maximumCommandApiUs, apiDurationUs);
        currentStatus.commandCount++;
        if(commandOk){
            for(int axis = 0; axis < kOnlineVelocityAxisCount; ++axis){
                if(std::fabs(stepResult.commandVelocity[axis]) >=
                        config.startVelocityThreshold){
                    motionStarted[axis] = true;
                }
            }
        }
        currentStatus.latestFullCycleUs = fullCycleDurationUs;
        currentStatus.maximumFullCycleUs = std::max(
                    currentStatus.maximumFullCycleUs,
                    fullCycleDurationUs);
        currentStatus.averageFullCycleUs +=
                (static_cast<double>(fullCycleDurationUs) -
                 currentStatus.averageFullCycleUs) /
                static_cast<double>(currentStatus.commandCount);
        if(fullCycleDurationUs > config.periodUs){
            currentStatus.executionOverrunCount++;
        }
        if(RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled){
            OnlineVelocityRecord record;
            record.step = stepResult;
            record.apiDurationUs = apiDurationUs;
            record.fullCycleDurationUs = fullCycleDurationUs;
            record.commandOk = commandOk;
            recorder->tryAppend(record);
            currentStatus.droppedRecordCount = recorder->droppedCount();
        }
    }
    if(!commandOk){
        setTerminalState(OnlineVelocityStatus::State::Fault,
                         QStringLiteral("八轴速度命令下发失败"));
        recorder->requestFinish();
    }
}

void OnlineVelocityControl::stop(bool fault, const QString& reason)
{
    setTerminalState(fault ? OnlineVelocityStatus::State::Fault :
                             OnlineVelocityStatus::State::Stopped,
                     reason);
    recorder->requestFinish();
}

void OnlineVelocityControl::finishRecording()
{
    recorder->finishAndWait();
    currentStatus.droppedRecordCount =
            RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled ?
                recorder->droppedCount() : 0;
}

bool OnlineVelocityControl::isActive() const
{
    return currentStatus.state == OnlineVelocityStatus::State::WaitingForTrace ||
            currentStatus.state == OnlineVelocityStatus::State::Running ||
            currentStatus.state == OnlineVelocityStatus::State::Settling;
}

bool OnlineVelocityControl::isPrepared() const
{
    return currentStatus.state == OnlineVelocityStatus::State::Prepared;
}

const OnlineVelocityPlan& OnlineVelocityControl::preparedPlan() const
{
    return plan;
}

const OnlineVelocityConfig& OnlineVelocityControl::currentConfig() const
{
    return config;
}

OnlineVelocityStatus OnlineVelocityControl::status() const
{
    OnlineVelocityStatus result = currentStatus;
    result.droppedRecordCount = RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled ?
                recorder->droppedCount() : 0;
    return result;
}

void OnlineVelocityControl::updateStatusFromStep(const OnlineVelocityStep& stepResult)
{
    currentStatus.elapsedSec = stepResult.elapsedSec;
    currentStatus.latestLogicalFrameSequence = stepResult.logicalFrameSequence;
    currentStatus.referencePosition = stepResult.referencePosition;
    currentStatus.actualPosition = stepResult.actualPosition;
    currentStatus.referenceVelocity = stepResult.referenceVelocity;
    currentStatus.actualVelocity = stepResult.actualVelocity;
    currentStatus.commandVelocity = stepResult.commandVelocity;
    currentStatus.rawPositionError = stepResult.positionError;
    if(stepResult.controlIntervalUs > 0){
        currentStatus.latestControlIntervalUs = stepResult.controlIntervalUs;
        currentStatus.maximumControlIntervalUs = std::max(
                    currentStatus.maximumControlIntervalUs,
                    stepResult.controlIntervalUs);
        controlIntervalCount++;
        controlIntervalSumUs += stepResult.controlIntervalUs;
        currentStatus.averageControlIntervalUs =
                static_cast<double>(controlIntervalSumUs) /
                static_cast<double>(controlIntervalCount);
        if(stepResult.controlIntervalUs >
                static_cast<qint64>(config.periodUs) * 3 / 2){
            currentStatus.schedulingOverrunCount++;
            const quint64 elapsedPeriods = static_cast<quint64>(
                        (stepResult.controlIntervalUs + config.periodUs / 2) /
                        config.periodUs);
            if(elapsedPeriods > 1){
                currentStatus.missedCycleCount += elapsedPeriods - 1;
            }
        }
    }
    for(double error : stepResult.positionError){
        currentStatus.maxRawPositionError =
                std::max(currentStatus.maxRawPositionError, std::fabs(error));
    }
}

void OnlineVelocityControl::setTerminalState(OnlineVelocityStatus::State state,
                                             const QString& message)
{
    currentStatus.state = state;
    currentStatus.message = message;
    nextDueUs = 0;
}
