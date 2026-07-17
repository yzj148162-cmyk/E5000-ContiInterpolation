#include "hardware/E5000HardwareInterface.h"

#include <QMetaObject>
#include <QSet>
#include <QStringList>
#include <QThread>

#include <cmath>
#include <algorithm>

#include "hardware/E5000ContiInterface.h"
#include "hardware/RuntimeTraceSlaveReader.h"

class E5000HardwareInterface::Backend : public QObject
{
public:
    bool initialize(quint16 cardNo, short &boardCount, QString &error)
    {
        if (!card_.initializeBoard(cardNo, boardCount, error)) {
            return false;
        }
        cardNo_ = cardNo;
        return true;
    }

    bool close(QString &error)
    {
        feedbackTrace_.reset();
        traceAxes_.clear();
        latestAxisFeedback_.clear();
        pendingTelemetryFrames_.clear();
        latestTraceSequence_ = 0;
        traceFramesRead_ = 0;
        traceStateText_ = QStringLiteral("Trace 已停止");
        return card_.closeBoard(error);
    }

    bool configureAxes(const QVector<quint16> &axes, QString &error)
    {
        QSet<quint16> uniqueAxes;
        for (const quint16 axis : axes) {
            if (!uniqueAxes.contains(axis)
                && !card_.setAxisEquivalent(cardNo_, axis, MotorUnit::kPulsesPerDegree, error)) {
                return false;
            }
            uniqueAxes.insert(axis);
        }
        return true;
    }

    bool configureTrace(const QVector<quint16> &axes, QString &error)
    {
        QSet<quint16> uniqueAxes;
        for (const quint16 axis : axes) {
            uniqueAxes.insert(axis);
        }
        if (axes.isEmpty() || axes.size() > 8 || uniqueAxes.size() != axes.size()) {
            error = QStringLiteral("Trace 测试轴必须为 1 至 8 个互不重复的轴");
            return false;
        }
        traceAxes_ = axes;
        latestAxisFeedback_.clear();
        pendingTelemetryFrames_.clear();
        latestTraceSequence_ = 0;
        latestAxisFeedback_.reserve(8);
        for (quint16 axis = 0; axis < 8; ++axis) {
            AxisFeedback feedback;
            feedback.axis = axis;
            feedback.errorText = traceAxes_.contains(axis)
                ? QStringLiteral("等待 Trace 首帧") : QStringLiteral("未纳入当前测试轴 Trace");
            latestAxisFeedback_.push_back(feedback);
        }

        RuntimeTraceSlaveReader::ReaderConfig traceConfig;
        traceConfig.cardNo = cardNo_;
        traceConfig.samplePeriodUs = 1000;
        traceConfig.traceBaseCycleUs = 1000;
        traceConfig.objects.reserve(traceAxes_.size() * 2);
        const auto addObject = [&traceConfig](int logicalIndex, short dataType, quint16 axis) {
            RuntimeTraceSlaveReader::ObjectConfig object;
            object.logicalIndex = logicalIndex;
            object.dataType = dataType;
            object.dataIndex = axis;
            object.dataSubIndex = 0;
            object.slaveId = 0;
            object.apiDataBytes = 4;
            object.valueBytes = 4;
            object.scale = 1.0 / MotorUnit::kPulsesPerDegree;
            traceConfig.objects.push_back(object);
        };
        for (int index = 0; index < traceAxes_.size(); ++index) {
            addObject(index, 5, traceAxes_.at(index));
        }
        for (int index = 0; index < traceAxes_.size(); ++index) {
            addObject(traceAxes_.size() + index, 6, traceAxes_.at(index));
        }
        if (!feedbackTrace_.configure(traceConfig)) {
            error = QStringLiteral("dmc_trace_* 返回码=%1").arg(feedbackTrace_.lastApiResult());
            traceStateText_ = QStringLiteral("Trace 配置失败");
            return false;
        }
        traceFramesRead_ = 0;
        traceStateText_ = QStringLiteral("Trace 已配置：1 ms，同帧 type 5/6 位置");
        return true;
    }

    bool enable(quint16 axis, QString &notice, QString &error)
    {
        if (!card_.setAxisEquivalent(cardNo_, axis, MotorUnit::kPulsesPerDegree, error)
            || !card_.clearAxisFaults(cardNo_, axis, notice, error)
            || !card_.enableAxis(cardNo_, axis, error)) {
            return false;
        }
        return true;
    }

    bool poll(QString &error)
    {
        traceFramesRead_ = feedbackTrace_.readTraceCached();
        if (traceFramesRead_ < 0) {
            traceStateText_ = QStringLiteral("Trace 读取失败，API 返回码=%1")
                                  .arg(feedbackTrace_.lastApiResult());
            error = traceStateText_;
            return false;
        }
        const std::vector<RuntimeTraceSlaveReader::Sample> samples = feedbackTrace_.takeSamples();
        if (samples.empty()) {
            traceStateText_ = feedbackTrace_.hasEverRead()
                ? QStringLiteral("Trace 正常，无新增帧") : QStringLiteral("等待 Trace 首帧");
        } else {
            for (const RuntimeTraceSlaveReader::Sample &sample : samples) {
                if (sample.values.size() < static_cast<std::size_t>(traceAxes_.size() * 2)) {
                    error = QStringLiteral("Trace 帧对象数量不足");
                    traceStateText_ = error;
                    return false;
                }
                for (int index = 0; index < traceAxes_.size(); ++index) {
                    AxisFeedback &feedback = latestAxisFeedback_[traceAxes_.at(index)];
                    feedback.commandPositionUnit = sample.values[static_cast<std::size_t>(index)];
                    feedback.encoderPositionUnit = sample.values[static_cast<std::size_t>(traceAxes_.size() + index)];
                    feedback.traceSampleValid = true;
                }
            }
            traceStateText_ = QStringLiteral("Trace 正常：本次读取 %1 帧").arg(traceFramesRead_);
        }
        for (const quint16 axis : traceAxes_) {
            AxisFeedback state;
            card_.readAxisFeedback(cardNo_, axis, state);
            AxisFeedback &feedback = latestAxisFeedback_[axis];
            feedback.stateMachine = state.stateMachine;
            feedback.axisErrorCode = state.axisErrorCode;
            feedback.valid = state.valid && feedback.traceSampleValid;
            feedback.errorText = !state.valid ? state.errorText
                                               : (feedback.traceSampleValid ? QString() : feedback.errorText);
        }
        pendingTelemetryFrames_.clear();
        pendingTelemetryFrames_.reserve(static_cast<qsizetype>(samples.size()));
        for (const RuntimeTraceSlaveReader::Sample &sample : samples) {
            TraceTelemetryFrame frame;
            frame.traceSequence = sample.sequence;
            frame.traceTimeUs = (sample.sequence > 0 ? sample.sequence - 1 : 0) * 1000ULL;
            frame.axisCount = static_cast<quint8>(std::min(2, static_cast<int>(traceAxes_.size())));
            for (int index = 0; index < frame.axisCount; ++index) {
                const quint16 axis = traceAxes_.at(index);
                const AxisFeedback &feedback = latestAxisFeedback_.at(axis);
                frame.axes[index] = axis;
                frame.commandPulse[index] = static_cast<qint32>(std::llround(
                    sample.values[static_cast<std::size_t>(index)] * MotorUnit::kPulsesPerDegree));
                frame.actualPulse[index] = static_cast<qint32>(std::llround(
                    sample.values[static_cast<std::size_t>(traceAxes_.size() + index)] * MotorUnit::kPulsesPerDegree));
                frame.axisState[index] = feedback.stateMachine;
                frame.axisError[index] = feedback.axisErrorCode;
                if (feedback.valid) {
                    frame.validAxisMask |= static_cast<quint8>(1U << index);
                }
            }
            pendingTelemetryFrames_.push_back(frame);
            latestTraceSequence_ = frame.traceSequence;
        }
        return true;
    }

    E5000ContiInterface card_;
    RuntimeTraceSlaveReader feedbackTrace_;
    quint16 cardNo_ = 0;
    QVector<quint16> traceAxes_;
    QVector<AxisFeedback> latestAxisFeedback_;
    int traceFramesRead_ = 0;
    QVector<TraceTelemetryFrame> pendingTelemetryFrames_;
    quint64 latestTraceSequence_ = 0;
    QString traceStateText_ = QStringLiteral("Trace 未配置");
};

namespace {
template <typename Function>
auto invokeHardware(QObject *target, Function &&function)
{
    using Result = decltype(function());
    Result result {};
    QMetaObject::invokeMethod(target, [&result, &function] { result = function(); }, Qt::BlockingQueuedConnection);
    return result;
}
}

E5000HardwareInterface::E5000HardwareInterface()
    : backend_(new Backend)
    , hardwareThread_(new QThread)
{
    backend_->moveToThread(hardwareThread_);
    hardwareThread_->start();
}

E5000HardwareInterface::~E5000HardwareInterface()
{
    if (backend_ != nullptr) {
        QMetaObject::invokeMethod(backend_, [backend = backend_] { delete backend; }, Qt::BlockingQueuedConnection);
        backend_ = nullptr;
    }
    hardwareThread_->quit();
    hardwareThread_->wait();
    delete hardwareThread_;
}

bool E5000HardwareInterface::initializeBoard(quint16 cardNo, short &boardCount, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->initialize(cardNo, boardCount, error); }); }
bool E5000HardwareInterface::closeBoard(QString &error)
{ return invokeHardware(backend_, [&] { return backend_->close(error); }); }
bool E5000HardwareInterface::configureAxes(const QVector<quint16> &axes, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->configureAxes(axes, error); }); }
bool E5000HardwareInterface::configureTrace(const QVector<quint16> &axes, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->configureTrace(axes, error); }); }
bool E5000HardwareInterface::enableAxis(quint16 axis, QString &notice, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->enable(axis, notice, error); }); }
bool E5000HardwareInterface::disableAxis(quint16 axis, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->card_.disableAxis(backend_->cardNo_, axis, error); }); }
bool E5000HardwareInterface::readCommandPosition(quint16 axis, double &position, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.readPositionUnit(backend_->cardNo_, axis, position, error); }); }
bool E5000HardwareInterface::readTargetPositionUnit(quint16 axis, double &position, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.readTargetPositionUnit(backend_->cardNo_, axis, position, error); }); }
bool E5000HardwareInterface::startPointMove(const SingleAxisJogConfig &config, QString &error)
{
    return invokeHardware(backend_, [&] {
        bool previouslyDone = false;
        if (!backend_->card_.axisMotionDone(backend_->cardNo_, config.axis, previouslyDone, error)) {
            return false;
        }
        Q_UNUSED(previouslyDone);
        if (!backend_->card_.startPointMove(config, error)) {
            return false;
        }
        QThread::msleep(10);
        bool done = false;
        quint16 runMode = 0;
        if (!backend_->card_.axisMotionDone(backend_->cardNo_, config.axis, done, error)
            || !backend_->card_.axisRunMode(backend_->cardNo_, config.axis, runMode, error)) {
            return false;
        }
        if (!done && runMode != 1U) {
            QString ignored;
            backend_->card_.stopAxis(backend_->cardNo_, config.axis, true, ignored);
            error = QStringLiteral("点位命令未进入定长模式：运行模式=%1（期望 1），已请求立即停止")
                        .arg(runMode);
            return false;
        }
        return true;
    });
}
bool E5000HardwareInterface::stopAxis(quint16 axis, bool emergency, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.stopAxis(backend_->cardNo_, axis, emergency, error); }); }
bool E5000HardwareInterface::stopAxis(quint16, quint16 axis, bool emergency, QString &error) const
{ return stopAxis(axis, emergency, error); }
bool E5000HardwareInterface::axisMotionDone(quint16 axis, bool &done, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.axisMotionDone(backend_->cardNo_, axis, done, error); }); }
bool E5000HardwareInterface::axisRunMode(quint16 axis, quint16 &mode, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.axisRunMode(backend_->cardNo_, axis, mode, error); }); }
bool E5000HardwareInterface::configureAndOpen(const ContiTestConfig &config, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.configureAndOpen(config, error); }); }
bool E5000HardwareInterface::pushLine(const ContiTestConfig &config, const ContiPoint &point, long mark, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.pushLine(config, point, mark, error); }); }
bool E5000HardwareInterface::start(const ContiTestConfig &config, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.start(config, error); }); }
ContiSpeedRatioResult E5000HardwareInterface::changeSpeedRatio(const ContiTestConfig &config,
                                                                QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.changeSpeedRatio(config, error); }); }
bool E5000HardwareInterface::stop(const ContiTestConfig &config, bool emergency, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.stop(config, emergency, error); }); }
bool E5000HardwareInterface::closeList(const ContiTestConfig &config, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.closeList(config, error); }); }
bool E5000HardwareInterface::contiMotionDone(const ContiTestConfig &config) const
{ return invokeHardware(backend_, [&] { return backend_->card_.isContiMotionDone(config); }); }
long E5000HardwareInterface::currentMark(const ContiTestConfig &config) const
{ return invokeHardware(backend_, [&] { return backend_->card_.currentMark(config); }); }
long E5000HardwareInterface::remainSpace(const ContiTestConfig &config) const
{ return invokeHardware(backend_, [&] { return backend_->card_.remainSpace(config); }); }
short E5000HardwareInterface::runState(const ContiTestConfig &config) const
{ return invokeHardware(backend_, [&] { return backend_->card_.runState(config); }); }
bool E5000HardwareInterface::pollFeedback(QString &error)
{ return invokeHardware(backend_, [&] { return backend_->poll(error); }); }
QVector<AxisFeedback> E5000HardwareInterface::axisFeedback() const
{ return invokeHardware(backend_, [&] { return backend_->latestAxisFeedback_; }); }
QVector<quint16> E5000HardwareInterface::traceAxes() const
{ return invokeHardware(backend_, [&] { return backend_->traceAxes_; }); }
bool E5000HardwareInterface::traceConfigured() const
{ return invokeHardware(backend_, [&] { return backend_->feedbackTrace_.isConfigured(); }); }
bool E5000HardwareInterface::traceEverRead() const
{ return invokeHardware(backend_, [&] { return backend_->feedbackTrace_.hasEverRead(); }); }
int E5000HardwareInterface::traceFramesRead() const
{ return invokeHardware(backend_, [&] { return backend_->traceFramesRead_; }); }
int E5000HardwareInterface::traceSamplePeriodUs() const
{ return 1000; }
QVector<TraceTelemetryFrame> E5000HardwareInterface::takeTraceTelemetryFrames()
{
    return invokeHardware(backend_, [&] {
        QVector<TraceTelemetryFrame> frames = std::move(backend_->pendingTelemetryFrames_);
        backend_->pendingTelemetryFrames_.clear();
        return frames;
    });
}
short E5000HardwareInterface::traceLastApiResult() const
{ return invokeHardware(backend_, [&] { return backend_->feedbackTrace_.lastApiResult(); }); }
QString E5000HardwareInterface::traceStateText() const
{ return invokeHardware(backend_, [&] { return backend_->traceStateText_; }); }
quint16 E5000HardwareInterface::busErrorCode() const
{ return invokeHardware(backend_, [&] { WORD code = 0; nmc_get_errcode(backend_->cardNo_, 2, &code); return static_cast<quint16>(code); }); }
bool E5000HardwareInterface::setAxisEquivalent(quint16, quint16 axis, double pulsePerUnit, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.setAxisEquivalent(backend_->cardNo_, axis, pulsePerUnit, error); }); }
bool E5000HardwareInterface::clearAxisFaults(quint16, quint16 axis, QString &notice, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.clearAxisFaults(backend_->cardNo_, axis, notice, error); }); }
bool E5000HardwareInterface::enableAxis(quint16, quint16 axis, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.enableAxis(backend_->cardNo_, axis, error); }); }
bool E5000HardwareInterface::disableAxis(quint16, quint16 axis, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.disableAxis(backend_->cardNo_, axis, error); }); }
bool E5000HardwareInterface::readPositionUnit(quint16, quint16 axis, double &position, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->card_.readPositionUnit(backend_->cardNo_, axis, position, error); }); }
bool E5000HardwareInterface::isAxisMotionDone(quint16, quint16 axis) const
{
    return invokeHardware(backend_, [&] {
        bool done = false;
        QString ignored;
        return backend_->card_.axisMotionDone(backend_->cardNo_, axis, done, ignored) && done;
    });
}
bool E5000HardwareInterface::isContiMotionDone(const ContiTestConfig &config) const
{ return contiMotionDone(config); }
