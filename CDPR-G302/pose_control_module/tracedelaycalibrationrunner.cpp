#include "tracedelaycalibrationrunner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>
#include <QTextStream>

namespace {

QString formatAxisDiagnostics(
        const HardwareInterface::ConnectionItemDiagnostics& diagnostics)
{
    return QStringLiteral("控制卡轴%1：状态机=%2，状态字=0x%3，轴错误码=%4，停止原因=%5"
                          "（诊断返回=%6，停止原因返回=%7）")
            .arg(diagnostics.hardwareAxis)
            .arg(diagnostics.stateMachine)
            .arg(static_cast<qulonglong>(diagnostics.statusWord), 0, 16)
            .arg(diagnostics.errorCode)
            .arg(diagnostics.stopReason)
            .arg(diagnostics.apiResult)
            .arg(diagnostics.stopReasonApiResult);
}
int profileSlot(const QString& key)
{
    return key == QStringLiteral("generic_incremental_8_axis") ? 1 : 0;
}

bool writeCalibrationCsv(const TraceDelayCalibrationConfig& config,
                         int axis,
                         const std::vector<TraceDelayCalibrationSegment>& segments,
                         QString* outputPath,
                         QString* errorMessage)
{
    QString directory = config.recordingDirectory;
    if(directory.isEmpty()){
        directory = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)
                + QStringLiteral("/trace_delay_calibration");
    }
    if(!QDir().mkpath(directory)){
        if(errorMessage) *errorMessage = QStringLiteral("无法创建标定记录目录");
        return false;
    }
    const QString path = QDir(directory).filePath(
                QStringLiteral("trace_delay_%1_axis%2_%3.csv")
                .arg(config.actuatorProfileKey).arg(axis)
                .arg(QDateTime::currentDateTime().toString(
                         QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    QFile file(path);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        if(errorMessage) *errorMessage = file.errorString();
        return false;
    }
    QTextStream out(&file);
    out << "axis,segment,target_velocity,frame_sequence,monotonic_us,command_position,actual_position,command_velocity,actual_velocity,valid\n";
    for(size_t segment = 0; segment < segments.size(); ++segment){
        for(const auto& sample : segments[segment].samples){
            out << axis << ',' << segment + 1 << ','
                << QString::number(segments[segment].targetVelocityUnitPerSec, 'g', 16) << ','
                << sample.frameSequence << ',' << sample.monotonicUs << ','
                << QString::number(sample.commandPositionUnit, 'g', 16) << ','
                << QString::number(sample.actualPositionUnit, 'g', 16) << ','
                << QString::number(sample.commandVelocityUnitPerSec, 'g', 16) << ','
                << QString::number(sample.actualVelocityUnitPerSec, 'g', 16) << ','
                << (sample.valid ? 1 : 0) << '\n';
        }
    }
    file.close();
    if(file.error() != QFileDevice::NoError){
        if(errorMessage) *errorMessage = file.errorString();
        return false;
    }
    if(outputPath) *outputPath = path;
    return true;
}

bool writeCalibrationResults(
        const std::array<QString, 2>& profileKeys,
        const std::array<double, 2>& equivalents,
        const std::array<int, 2>& tracePeriodsUs,
        const std::array<std::array<TraceDelayAxisResult, 8>, 2>& results,
        QString* errorMessage)
{
    const QString directory =
            QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    if(!QDir().mkpath(directory)){
        if(errorMessage) *errorMessage = QStringLiteral("无法创建标定结果目录");
        return false;
    }
    QJsonObject root;
    root.insert(QStringLiteral("version"), 2);
    QJsonArray profiles;
    for(int slot = 0; slot < 2; ++slot){
        QJsonObject profile;
        profile.insert(QStringLiteral("key"), profileKeys[slot]);
        profile.insert(QStringLiteral("axis_equivalent"), equivalents[slot]);
        profile.insert(QStringLiteral("trace_period_us"), tracePeriodsUs[slot]);
        QJsonArray axes;
        for(const auto& result : results[slot]){
            QJsonObject item;
            item.insert(QStringLiteral("axis"), result.axis);
            item.insert(QStringLiteral("calibrated"), result.calibrated);
            item.insert(QStringLiteral("valid"), result.valid);
            item.insert(QStringLiteral("delay_ms"), result.measuredDelayMs);
            item.insert(QStringLiteral("offset"), result.staticOffsetUnit);
            item.insert(QStringLiteral("r2"), result.rSquared);
            item.insert(QStringLiteral("rmse"), result.rmseUnit);
            item.insert(QStringLiteral("pair_spread_ms"), result.pairSpreadMs);
            item.insert(QStringLiteral("lost_frames"), result.lostFrameCount);
            item.insert(QStringLiteral("timestamp"), result.timestamp);
            item.insert(QStringLiteral("source"), result.source);
            item.insert(QStringLiteral("detail"), result.detail);
            axes.append(item);
        }
        profile.insert(QStringLiteral("axes"), axes);
        profiles.append(profile);
    }
    root.insert(QStringLiteral("profiles"), profiles);
    QFile file(QDir(directory).filePath(
                   QStringLiteral("force_interaction_trace_delay.json")));
    if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if(errorMessage) *errorMessage = file.errorString();
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    if(file.error() != QFileDevice::NoError){
        if(errorMessage) *errorMessage = file.errorString();
        return false;
    }
    return true;
}
}

TraceDelayCalibrationRunner::TraceDelayCalibrationRunner(HardwareInterface* hardware)
    : hardware_(hardware)
{
    for(int slot = 0; slot < 2; ++slot){
        for(int axis = 0; axis < 8; ++axis) storedResults_[slot][axis].axis = axis;
    }
    loadResults();
}

bool TraceDelayCalibrationRunner::start(const TraceDelayCalibrationConfig& config,
                                        QString* errorMessage)
{
    QMutexLocker locker(&mutex_);
    if(status_.active || finalizationPending_){
        if(errorMessage) *errorMessage = QStringLiteral("Trace 延迟标定正在运行");
        return false;
    }
    if(!hardware_ || !hardware_->isLSConnected()){
        if(errorMessage) *errorMessage = QStringLiteral("控制卡未连接");
        return false;
    }
    if(config.axis < 0 || config.axis >= 8 || config.actuatorProfileKey.isEmpty() ||
       config.axisEquivalentPulsePerUnit <= 0.0 || config.holdMs <= 0 ||
       config.sampleWindowMs <= 0 || config.sampleWindowMs >= config.holdMs ||
       config.restMs < 100 || config.onlineChangeTimeS < 0.0){
        if(errorMessage) *errorMessage = QStringLiteral("Trace 延迟标定参数无效");
        return false;
    }
    for(double speed : config.speeds){
        if(!std::isfinite(speed) || speed <= 0.0 ||
           speed * config.holdMs / 1000.0 > config.maximumSegmentTravelUnit){
            if(errorMessage) *errorMessage = QStringLiteral("标定速度或单段行程超限");
            return false;
        }
    }

    config_ = config;
    axes_.clear();
    if(config.allAxes){
        for(int axis = 0; axis < 8; ++axis) axes_.push_back(axis);
    } else {
        axes_.push_back(config.axis);
    }
    for(int axis : axes_){
        if(!hardware_->isMotorEnabled(axis)){
            if(errorMessage) *errorMessage = QStringLiteral("轴 %1 尚未使能").arg(axis);
            return false;
        }
    }
    targets_ = {{config.speeds[0], -config.speeds[0],
                 config.speeds[1], -config.speeds[1],
                 config.speeds[2], -config.speeds[2]}};
    status_ = TraceDelayCalibrationStatus();
    finalizationWarnings_.clear();
    status_.active = true;
    status_.state = TraceDelayCalibrationStatus::State::Configuring;
    status_.allAxes = config.allAxes;
    status_.totalAxes = static_cast<int>(axes_.size());
    status_.currentAxisOrdinal = 1;
    status_.axis = axes_.front();
    status_.profileKey = config.actuatorProfileKey;
    status_.phaseText = QStringLiteral("配置 Trace");
    status_.axisResults = storedResults_[profileSlot(config.actuatorProfileKey)];
    segments_.clear();
    traceSamplePeriodUs_ = 0;
    phaseStartUs_ = 0;
    axisStartUs_ = 0;
    QString configureError;
    if(!configureCurrentAxis(&configureError)){
        finish(true, configureError);
        if(errorMessage) *errorMessage = configureError;
        return false;
    }
    return true;
}

bool TraceDelayCalibrationRunner::configureCurrentAxis(QString* errorMessage)
{
    const int axis = axes_[status_.currentAxisOrdinal - 1];
    // 轴选择改变时必须真正重建对象表；同一 Preset profile 的重复设置会被
    // HardwareInterface 判为无变化，因此先退回 Base 再选择下一轴。
    hardware_->setOnlineVelocityRuntimeTraceProfileEnabled(false);
    hardware_->setRuntimeTraceCommissioningSelection(axis, -1);
    if(!hardware_->setOnlineVelocityRuntimeTraceProfileEnabled(true)){
        if(errorMessage) *errorMessage = QStringLiteral("轴 %1 速度/位置 Trace 配置失败").arg(axis);
        return false;
    }
    hardware_->readMotorRelativeTracePositionSamples(axis);
    status_.axis = axis;
    status_.currentSegment = 0;
    status_.targetVelocityUnitPerSec = 0.0;
    status_.state = TraceDelayCalibrationStatus::State::Resting;
    status_.phaseText = QStringLiteral("段间静止");
    status_.message = QStringLiteral("轴 %1 等待首段标定").arg(axis);
    segments_.clear();
    phaseStartUs_ = 0;
    axisStartUs_ = 0;
    return true;
}

bool TraceDelayCalibrationRunner::beginCurrentSegment(QString* errorMessage)
{
    const int axis = status_.axis;
    const double target = targets_[status_.currentSegment];
    const auto beforeCommand = hardware_->motorAxisDiagnostics(axis);
    const std::vector<int> axes{axis};
    const std::vector<double> velocity{target};
    if(!hardware_->motorVelBatch(axes, velocity, config_.onlineChangeTimeS)){
        const auto rejected = hardware_->motorAxisDiagnostics(axis);
        if(errorMessage){
            *errorMessage = QStringLiteral("轴 %1 下发标定速度 %2 失败；下发前%3；拒绝后%4")
                    .arg(axis).arg(target, 0, 'f', 4)
                    .arg(formatAxisDiagnostics(beforeCommand))
                    .arg(formatAxisDiagnostics(rejected));
        }
        return false;
    }
    TraceDelayCalibrationSegment segment;
    segment.targetVelocityUnitPerSec = target;
    segments_.push_back(std::move(segment));
    status_.targetVelocityUnitPerSec = target;
    status_.state = TraceDelayCalibrationStatus::State::Moving;
    status_.phaseText = QStringLiteral("采集第 %1/6 段").arg(status_.currentSegment + 1);
    status_.message = QStringLiteral("下发前%1")
            .arg(formatAxisDiagnostics(beforeCommand));
    return true;
}

void TraceDelayCalibrationRunner::drainSamples()
{
    if(!hardware_ || status_.axis < 0) return;
    auto samples = hardware_->readMotorRelativeTracePositionSamples(status_.axis);
    if(status_.state != TraceDelayCalibrationStatus::State::Moving || segments_.empty()) return;
    auto& destination = segments_.back().samples;
    destination.reserve(destination.size() + samples.size());
    for(const auto& source : samples){
        TraceDelayCalibrationSample sample;
        sample.frameSequence = source.frameSequence;
        sample.monotonicUs = source.monotonicUs;
        sample.commandPositionUnit = source.commandRelativePosition;
        sample.actualPositionUnit = source.feedbackRelativePosition;
        sample.commandVelocityUnitPerSec = source.commandVelocityUnitPerSec;
        sample.actualVelocityUnitPerSec = source.actualVelocityUnitPerSec;
        sample.valid = source.frameSequenceValid && source.commandRawPulseValid &&
                source.feedbackRawPulseValid && source.commandVelocityValid &&
                source.actualVelocityValid &&
                std::isfinite(sample.commandPositionUnit) &&
                std::isfinite(sample.actualPositionUnit) &&
                std::isfinite(sample.commandVelocityUnitPerSec) &&
                std::isfinite(sample.actualVelocityUnitPerSec);
        destination.push_back(sample);
    }
}

void TraceDelayCalibrationRunner::tick(qint64 nowUs, int traceSamplePeriodUs)
{
    QMutexLocker locker(&mutex_);
    if(finalizationPending_){
        if(finalizationFuture_.wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready){
            return;
        }
        collectCurrentAxisFinalization();
        return;
    }
    if(!status_.active) return;
    if(traceSamplePeriodUs > 0) traceSamplePeriodUs_ = traceSamplePeriodUs;
    drainSamples();
    if(phaseStartUs_ <= 0){
        phaseStartUs_ = nowUs;
        axisStartUs_ = nowUs;
        return;
    }
    const qint64 elapsedUs = nowUs - phaseStartUs_;
    if(status_.state == TraceDelayCalibrationStatus::State::Resting){
        if(elapsedUs < static_cast<qint64>(config_.restMs) * 1000) return;
        hardware_->readMotorRelativeTracePositionSamples(status_.axis);
        QString error;
        if(!beginCurrentSegment(&error)){
            finish(true, error);
            return;
        }
        phaseStartUs_ = nowUs;
    } else if(status_.state == TraceDelayCalibrationStatus::State::Moving){
        if(elapsedUs < static_cast<qint64>(config_.holdMs) * 1000) return;
        drainSamples();
        const auto beforeStop = hardware_->motorAxisDiagnostics(status_.axis);
        if(!hardware_->motorStop(status_.axis)){
            const auto failedStop = hardware_->motorAxisDiagnostics(status_.axis);
            finish(true, QStringLiteral("轴 %1 停止失败；停止前%2；失败后%3")
                   .arg(status_.axis)
                   .arg(formatAxisDiagnostics(beforeStop))
                   .arg(formatAxisDiagnostics(failedStop)));
            return;
        }
        const auto afterStop = hardware_->motorAxisDiagnostics(status_.axis);
        status_.state = TraceDelayCalibrationStatus::State::Stopping;
        status_.phaseText = QStringLiteral("等待静止");
        status_.targetVelocityUnitPerSec = 0.0;
        status_.message = QStringLiteral("停止前%1；停止后%2")
                .arg(formatAxisDiagnostics(beforeStop))
                .arg(formatAxisDiagnostics(afterStop));
        phaseStartUs_ = nowUs;
    } else if(status_.state == TraceDelayCalibrationStatus::State::Stopping){
        if(elapsedUs < static_cast<qint64>(config_.restMs) * 1000) return;
        ++status_.currentSegment;
        if(status_.currentSegment < 6){
            status_.state = TraceDelayCalibrationStatus::State::Resting;
            status_.phaseText = QStringLiteral("段间静止");
            phaseStartUs_ = nowUs - static_cast<qint64>(config_.restMs) * 1000;
        } else {
            completeCurrentAxis();
        }
    }
    const int completedAxes = status_.currentAxisOrdinal - 1;
    const int segmentProgress = std::min(6, status_.currentSegment);
    status_.progressPercent = std::clamp(
        (completedAxes * 6 + segmentProgress) * 100 /
        std::max(1, status_.totalAxes * 6), 0, 100);
}

void TraceDelayCalibrationRunner::completeCurrentAxis()
{
    TraceDelayCalibrationConfig axisConfig = config_;
    axisConfig.axis = status_.axis;
    const int traceSamplePeriodUs = traceSamplePeriodUs_;
    const int axis = status_.axis;
    auto resultsForPersistence = storedResults_;
    const int persistenceSlot = profileSlot(config_.actuatorProfileKey);
    resultsForPersistence[persistenceSlot][axis].axis = axis;
    auto equivalentsForPersistence = storedEquivalent_;
    equivalentsForPersistence[persistenceSlot] = config_.axisEquivalentPulsePerUnit;
    auto tracePeriodsForPersistence = storedTracePeriodUs_;
    tracePeriodsForPersistence[persistenceSlot] = traceSamplePeriodUs;
    const auto profileKeysForPersistence = storedProfileKeys_;
    std::vector<TraceDelayCalibrationSegment> jobSegments = std::move(segments_);
    status_.state = TraceDelayCalibrationStatus::State::Finalizing;
    status_.phaseText = QStringLiteral("后台拟合与写盘");
    status_.targetVelocityUnitPerSec = 0.0;
    status_.message = QStringLiteral("控制循环继续运行，正在后台处理标定数据");
    finalizationPending_ = true;
    finalizationFuture_ = std::async(
                std::launch::async,
                [axisConfig, traceSamplePeriodUs, axis,
                 data = std::move(jobSegments),
                 resultsForPersistence,
                 equivalentsForPersistence,
                 tracePeriodsForPersistence,
                 profileKeysForPersistence]() mutable {
        FinalizationResult result;
        QElapsedTimer timer;
        timer.start();
        result.fit = TraceDelayCalibrationAnalyzer::analyze(
                    axisConfig, traceSamplePeriodUs, data);
        result.fit.axisResult.timestamp =
                QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
        result.fitDurationUs = timer.nsecsElapsed() / 1000;
        timer.restart();
        writeCalibrationCsv(axisConfig, axis, data,
                            &result.rawDataFile, &result.errorMessage);
        result.csvWriteDurationUs = timer.nsecsElapsed() / 1000;
        resultsForPersistence[profileSlot(axisConfig.actuatorProfileKey)][axis] =
                result.fit.axisResult;
        timer.restart();
        QString persistenceError;
        writeCalibrationResults(profileKeysForPersistence,
                                equivalentsForPersistence,
                                tracePeriodsForPersistence,
                                resultsForPersistence,
                                &persistenceError);
        result.persistenceDurationUs = timer.nsecsElapsed() / 1000;
        if(!persistenceError.isEmpty()){
            if(!result.errorMessage.isEmpty()) result.errorMessage += QStringLiteral("；");
            result.errorMessage += QStringLiteral("结果保存失败：%1").arg(persistenceError);
        }
        result.segments = std::move(data);
        return result;
    });
}

void TraceDelayCalibrationRunner::collectCurrentAxisFinalization()
{
    FinalizationResult finalized;
    try{
        finalized = finalizationFuture_.get();
    }
    catch(const std::exception& exception){
        finalizationPending_ = false;
        finish(true, QStringLiteral("标定后台处理异常：%1")
               .arg(QString::fromLocal8Bit(exception.what())));
        return;
    }
    catch(...){
        finalizationPending_ = false;
        finish(true, QStringLiteral("标定后台处理发生未知异常"));
        return;
    }
    finalizationPending_ = false;
    if(!status_.active){
        return;
    }
    status_.fitDurationUs = finalized.fitDurationUs;
    status_.csvWriteDurationUs = finalized.csvWriteDurationUs;
    status_.persistenceDurationUs = finalized.persistenceDurationUs;
    status_.rawDataFile = finalized.rawDataFile;
    status_.axisResults[status_.axis] = finalized.fit.axisResult;
    const int slot = profileSlot(config_.actuatorProfileKey);
    storedResults_[slot][status_.axis] = finalized.fit.axisResult;
    storedEquivalent_[slot] = config_.axisEquivalentPulsePerUnit;
    storedTracePeriodUs_[slot] = traceSamplePeriodUs_;
    lastConfig_ = config_;
    lastConfig_.axis = status_.axis;
    lastSegments_ = std::move(finalized.segments);
    lastTraceSamplePeriodUs_ = traceSamplePeriodUs_;
    if(!finalized.errorMessage.isEmpty()){
        if(!finalizationWarnings_.isEmpty()) finalizationWarnings_ += QStringLiteral("；");
        finalizationWarnings_ += finalized.errorMessage;
        status_.message = QStringLiteral("标定计算完成，但记录存在问题：%1")
                .arg(finalizationWarnings_);
    }
    if(status_.currentAxisOrdinal < status_.totalAxes){
        ++status_.currentAxisOrdinal;
        QString error;
        if(!configureCurrentAxis(&error)) finish(true, error);
        return;
    }
    int passed = 0;
    for(int axis : axes_){
        if(status_.axisResults[axis].valid) ++passed;
    }
    QString completionMessage = QStringLiteral("Trace 延迟标定采集完成：%1/%2轴通过")
            .arg(passed).arg(status_.totalAxes);
    if(!finalizationWarnings_.isEmpty()){
        completionMessage += QStringLiteral("；记录警告：%1")
                .arg(finalizationWarnings_);
    }
    finish(false, completionMessage);
}

void TraceDelayCalibrationRunner::finish(bool fault, const QString& message)
{
    QElapsedTimer restoreTimer;
    restoreTimer.start();
    if(hardware_){
        if(status_.axis >= 0) hardware_->motorStop(status_.axis);
        hardware_->setOnlineVelocityRuntimeTraceProfileEnabled(false);
        hardware_->clearRuntimeTraceCommissioningSelection();
    }
    status_.traceRestoreDurationUs = restoreTimer.nsecsElapsed() / 1000;
    status_.active = false;
    status_.state = fault ? TraceDelayCalibrationStatus::State::Fault
                          : TraceDelayCalibrationStatus::State::Completed;
    status_.phaseText = fault ? QStringLiteral("故障") : QStringLiteral("完成");
    status_.message = message +
            QStringLiteral("；耗时：拟合=%1 ms，CSV=%2 ms，结果保存=%3 ms，Trace恢复=%4 ms")
            .arg(status_.fitDurationUs / 1000.0, 0, 'f', 3)
            .arg(status_.csvWriteDurationUs / 1000.0, 0, 'f', 3)
            .arg(status_.persistenceDurationUs / 1000.0, 0, 'f', 3)
            .arg(status_.traceRestoreDurationUs / 1000.0, 0, 'f', 3);
    if(!fault) status_.progressPercent = 100;
}

void TraceDelayCalibrationRunner::stop(bool emergency, const QString& reason)
{
    QMutexLocker locker(&mutex_);
    if(!status_.active) return;
    if(emergency && hardware_) hardware_->emergencyStopAxes({status_.axis});
    writeRawCsv(nullptr);
    finish(emergency, reason.isEmpty() ? QStringLiteral("用户停止标定") : reason);
}

void TraceDelayCalibrationRunner::resetSession()
{
    QMutexLocker locker(&mutex_);
    if(status_.active){
        return;
    }
    status_ = TraceDelayCalibrationStatus{};
    config_ = TraceDelayCalibrationConfig{};
    axes_.clear();
    segments_.clear();
    phaseStartUs_ = 0;
    axisStartUs_ = 0;
    traceSamplePeriodUs_ = 0;
    finalizationWarnings_.clear();
}

bool TraceDelayCalibrationRunner::isActive() const
{
    QMutexLocker locker(&mutex_);
    return status_.active;
}

TraceDelayCalibrationStatus TraceDelayCalibrationRunner::status() const
{
    QMutexLocker locker(&mutex_);
    return status_;
}

std::array<TraceDelayAxisResult, 8> TraceDelayCalibrationRunner::resultsForProfile(
        const QString& profileKey, double equivalent, int tracePeriodUs) const
{
    QMutexLocker locker(&mutex_);
    const int slot = profileSlot(profileKey);
    auto results = storedResults_[slot];
    const bool stale = storedEquivalent_[slot] <= 0.0 || storedTracePeriodUs_[slot] <= 0 ||
            std::fabs(storedEquivalent_[slot] - equivalent) > 1.0e-6 ||
            storedTracePeriodUs_[slot] != tracePeriodUs;
    for(auto& result : results){
        result.stale = result.calibrated && stale;
        if(result.stale) result.valid = false;
    }
    return results;
}

bool TraceDelayCalibrationRunner::recalculateLast(QString* errorMessage)
{
    QMutexLocker locker(&mutex_);
    if(lastSegments_.size() != 6U || lastTraceSamplePeriodUs_ <= 0){
        if(errorMessage) *errorMessage = QStringLiteral("本次启动后尚无完整原始标定数据");
        return false;
    }
    TraceDelayFitResult fit = TraceDelayCalibrationAnalyzer::analyze(
                lastConfig_, lastTraceSamplePeriodUs_, lastSegments_);
    fit.axisResult.timestamp = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    const int slot = profileSlot(lastConfig_.actuatorProfileKey);
    storedResults_[slot][lastConfig_.axis] = fit.axisResult;
    status_.axisResults = storedResults_[slot];
    saveResults();
    return fit.axisResult.valid;
}

bool TraceDelayCalibrationRunner::writeRawCsv(QString* errorMessage)
{
    if(lastSegments_.empty() && segments_.empty()) return false;
    QString directory = config_.recordingDirectory;
    if(directory.isEmpty()){
        directory = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)
                + QStringLiteral("/trace_delay_calibration");
    }
    if(!QDir().mkpath(directory)){
        if(errorMessage) *errorMessage = QStringLiteral("无法创建标定记录目录");
        return false;
    }
    const QString path = QDir(directory).filePath(QStringLiteral("trace_delay_%1_axis%2_%3.csv")
        .arg(config_.actuatorProfileKey).arg(status_.axis)
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    QFile file(path);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        if(errorMessage) *errorMessage = file.errorString();
        return false;
    }
    QTextStream out(&file);
    out << "axis,segment,target_velocity,frame_sequence,monotonic_us,command_position,actual_position,command_velocity,actual_velocity,valid\n";
    const auto& data = segments_.empty() ? lastSegments_ : segments_;
    for(size_t segment = 0; segment < data.size(); ++segment){
        for(const auto& sample : data[segment].samples){
            out << status_.axis << ',' << segment + 1 << ','
                << QString::number(data[segment].targetVelocityUnitPerSec, 'g', 16) << ','
                << sample.frameSequence << ',' << sample.monotonicUs << ','
                << QString::number(sample.commandPositionUnit, 'g', 16) << ','
                << QString::number(sample.actualPositionUnit, 'g', 16) << ','
                << QString::number(sample.commandVelocityUnitPerSec, 'g', 16) << ','
                << QString::number(sample.actualVelocityUnitPerSec, 'g', 16) << ','
                << (sample.valid ? 1 : 0) << '\n';
        }
    }
    status_.rawDataFile = path;
    return true;
}

QString TraceDelayCalibrationRunner::persistencePath() const
{
    const QString directory = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    QDir().mkpath(directory);
    return QDir(directory).filePath(QStringLiteral("force_interaction_trace_delay.json"));
}

void TraceDelayCalibrationRunner::saveResults() const
{
    writeCalibrationResults(storedProfileKeys_, storedEquivalent_, storedTracePeriodUs_,
                            storedResults_, nullptr);
}

void TraceDelayCalibrationRunner::loadResults()
{
    QFile file(persistencePath());
    if(!file.open(QIODevice::ReadOnly)) return;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    for(const auto profileValue : document.object().value(QStringLiteral("profiles")).toArray()){
        const QJsonObject profile = profileValue.toObject();
        const int slot = profileSlot(profile.value(QStringLiteral("key")).toString());
        storedEquivalent_[slot] = profile.value(QStringLiteral("axis_equivalent")).toDouble();
        storedTracePeriodUs_[slot] = profile.value(QStringLiteral("trace_period_us")).toInt();
        for(const auto axisValue : profile.value(QStringLiteral("axes")).toArray()){
            const QJsonObject item = axisValue.toObject();
            const int axis = item.value(QStringLiteral("axis")).toInt(-1);
            if(axis < 0 || axis >= 8) continue;
            auto& result = storedResults_[slot][axis];
            result.axis = axis;
            result.valid = item.value(QStringLiteral("valid")).toBool();
            result.calibrated = item.contains(QStringLiteral("calibrated"))
                    ? item.value(QStringLiteral("calibrated")).toBool()
                    : result.valid;
            result.measuredDelayMs = item.value(QStringLiteral("delay_ms")).toDouble();
            result.appliedDelayMs = result.valid ? result.measuredDelayMs : 8.0;
            result.staticOffsetUnit = item.value(QStringLiteral("offset")).toDouble();
            result.rSquared = item.value(QStringLiteral("r2")).toDouble();
            result.rmseUnit = item.value(QStringLiteral("rmse")).toDouble();
            result.pairSpreadMs = item.value(QStringLiteral("pair_spread_ms")).toDouble();
            result.lostFrameCount = item.value(QStringLiteral("lost_frames")).toInt();
            result.timestamp = item.value(QStringLiteral("timestamp")).toString();
            result.source = item.value(QStringLiteral("source")).toString(
                        result.valid ? QStringLiteral("实测") : QStringLiteral("默认"));
            result.detail = item.value(QStringLiteral("detail")).toString(
                        result.valid ? QStringLiteral("已加载标定结果") :
                                       (result.calibrated ? QStringLiteral("标定未通过") :
                                                            QStringLiteral("尚未标定")));
        }
    }
}
