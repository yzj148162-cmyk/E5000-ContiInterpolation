#include "hardware/E5000HardwareInterface.h"

#include <QMetaObject>
#include <QElapsedTimer>
#include <QMap>
#include <QQueue>
#include <QSet>
#include <QStringList>
#include <QThread>
#include <QTimer>

#include <cmath>
#include <algorithm>

#include "hardware/E5000ContiInterface.h"
#include "hardware/RuntimeTraceSlaveReader.h"

class E5000HardwareInterface::Backend : public QObject
{
public:
    void stopFeedScheduler(bool clearQueue = true)
    {
        if (feedTimer_ != nullptr) {
            feedTimer_->stop();
        }
        feedStatus_.active = false;
        streamListOpen_ = false;
        if (clearQueue) {
            feedQueue_.clear();
            planTimeByMark_.clear();
            feedStatus_.queuedPointCount = 0;
        }
    }

    double planTimeForMark(long mark) const
    {
        if (planTimeByMark_.isEmpty()) {
            return 0.0;
        }
        auto iterator = planTimeByMark_.upperBound(std::max(0L, mark));
        if (iterator == planTimeByMark_.begin()) {
            return iterator.value();
        }
        --iterator;
        return iterator.value();
    }

    bool pushQueuedPoints(bool pushAll, double targetBufferS, QString &error)
    {
        feedStatus_.currentMark = card_.currentMark(streamConfig_);
        feedStatus_.currentPlanTimeS = planTimeForMark(feedStatus_.currentMark);
        feedStatus_.remainSpace = card_.remainSpace(streamConfig_);

        while (!feedQueue_.isEmpty() && feedStatus_.remainSpace > 0) {
            const double bufferedTimeS = std::max(0.0,
                feedStatus_.lastPushedPlanTimeS - feedStatus_.currentPlanTimeS);
            if (!pushAll && feedStatus_.lastPushedMark > 0
                && bufferedTimeS >= targetBufferS) {
                break;
            }
            const ContiFeedItem item = feedQueue_.dequeue();
            if (!card_.pushLine(streamConfig_, item.point, item.mark, error)) {
                return false;
            }
            feedStatus_.lastPushedMark = item.mark;
            feedStatus_.lastPushedPlanTimeS = item.point.timeS;
            --feedStatus_.remainSpace;
        }
        feedStatus_.queuedPointCount = feedQueue_.size();
        return true;
    }

    void serviceFeedQueue()
    {
        if (!feedStatus_.active || !streamListOpen_ || feedStatus_.failed) {
            return;
        }

        if (feedServiceClock_.isValid()) {
            const qint64 nowUs = feedServiceClock_.nsecsElapsed() / 1000;
            if (lastFeedServiceUs_ >= 0) {
                feedStatus_.lastServiceGapUs = nowUs - lastFeedServiceUs_;
                feedStatus_.maxServiceGapUs = std::max(feedStatus_.maxServiceGapUs,
                                                       feedStatus_.lastServiceGapUs);
            }
            lastFeedServiceUs_ = nowUs;
        }

        QString error;
        if (!pushQueuedPoints(false, streamConfig_.targetBufferMs / 1000.0, error)) {
            feedStatus_.failed = true;
            feedStatus_.active = false;
            feedStatus_.errorText = QStringLiteral("硬件线程自动补段失败：%1").arg(error);
            if (feedTimer_ != nullptr) {
                feedTimer_->stop();
            }
            QString ignored;
            card_.stop(streamConfig_, true, ignored);
            return;
        }
        feedStatus_.runState = card_.runState(streamConfig_);
    }

    bool startStreaming(const ContiTestConfig &config,
                        const QVector<ContiFeedItem> &points,
                        bool preloadAllToCard,
                        QString &error)
    {
        stopFeedScheduler(true);
        feedStatus_ = {};
        streamConfig_ = config;
        lastAcceptedMark_ = 0;
        if (points.isEmpty()) {
            error = QStringLiteral("硬件线程未收到有效轨迹段");
            return false;
        }
        if (!card_.configureAndOpen(streamConfig_, error)) {
            return false;
        }
        streamListOpen_ = true;
        for (const ContiFeedItem &item : points) {
            feedQueue_.enqueue(item);
            planTimeByMark_.insert(item.mark, item.point.timeS);
            lastAcceptedMark_ = item.mark;
        }
        feedStatus_.queuedPointCount = feedQueue_.size();

        const long availableSlots = card_.remainSpace(streamConfig_);
        if (preloadAllToCard && availableSlots < feedQueue_.size()) {
            error = QStringLiteral("一次性预装失败：有效段=%1，但控制卡当前可用段位=%2")
                        .arg(feedQueue_.size())
                        .arg(availableSlots);
            QString ignored;
            card_.closeList(streamConfig_, ignored);
            stopFeedScheduler(true);
            return false;
        }

        const double initialBufferS = std::max(config.startupPreloadMs,
                                               config.targetBufferMs) / 1000.0;
        if (!pushQueuedPoints(preloadAllToCard, initialBufferS, error)) {
            QString ignored;
            card_.closeList(streamConfig_, ignored);
            stopFeedScheduler(true);
            return false;
        }
        const double initialBufferedS = feedStatus_.lastPushedPlanTimeS
            - feedStatus_.currentPlanTimeS;
        if (!preloadAllToCard
            && initialBufferedS * 1000.0 + 0.001 < config.startupPreloadMs) {
            error = QStringLiteral("启动预压不足：实际=%1 ms，要求=%2 ms")
                        .arg(initialBufferedS * 1000.0, 0, 'f', 1)
                        .arg(config.startupPreloadMs);
            QString ignored;
            card_.closeList(streamConfig_, ignored);
            stopFeedScheduler(true);
            return false;
        }
        if (!card_.start(streamConfig_, error)) {
            QString ignored;
            card_.closeList(streamConfig_, ignored);
            stopFeedScheduler(true);
            return false;
        }

        if (feedTimer_ == nullptr) {
            feedTimer_ = new QTimer(this);
            feedTimer_->setTimerType(Qt::PreciseTimer);
            feedTimer_->setInterval(5);
            connect(feedTimer_, &QTimer::timeout, this, [this] { serviceFeedQueue(); });
        }
        feedStatus_.active = true;
        feedStatus_.failed = false;
        feedStatus_.errorText.clear();
        feedServiceClock_.start();
        lastFeedServiceUs_ = -1;
        feedTimer_->start();
        serviceFeedQueue();
        return true;
    }

    void appendStreamingPoints(const QVector<ContiFeedItem> &points)
    {
        if (points.isEmpty() || feedStatus_.failed) {
            return;
        }
        if (!feedStatus_.active || !streamListOpen_) {
            feedStatus_.failed = true;
            feedStatus_.errorText = QStringLiteral("硬件线程收到轨迹点时连续插补未处于运行状态");
            return;
        }
        for (const ContiFeedItem &item : points) {
            if (item.mark <= lastAcceptedMark_) {
                feedStatus_.failed = true;
                feedStatus_.active = false;
                feedStatus_.errorText = QStringLiteral("轨迹 mark 未严格递增：收到 %1，上次为 %2")
                                            .arg(item.mark)
                                            .arg(lastAcceptedMark_);
                if (feedTimer_ != nullptr) {
                    feedTimer_->stop();
                }
                QString ignored;
                card_.stop(streamConfig_, true, ignored);
                return;
            }
            feedQueue_.enqueue(item);
            planTimeByMark_.insert(item.mark, item.point.timeS);
            lastAcceptedMark_ = item.mark;
        }
        feedStatus_.queuedPointCount = feedQueue_.size();
        serviceFeedQueue();
    }

    ContiFeedStatus streamingStatus()
    {
        serviceFeedQueue();
        return feedStatus_;
    }

    bool initialize(quint16 cardNo, short &boardCount, QString &error)
    {
        if (!card_.initializeBoard(cardNo, boardCount, error)) {
            return false;
        }
        cardNo_ = cardNo;
        return true;
    }

    bool setBusCycle(int cycleUs, QString &error)
    {
        if (cycleUs != 250 && cycleUs != 500 && cycleUs != 1000 && cycleUs != 2000) {
            error = QStringLiteral("EtherCAT 总线周期必须为 250/500/1000/2000 us");
            return false;
        }
        const short result = nmc_set_cycletime(cardNo_, kEthercatPort,
                                               static_cast<DWORD>(cycleUs));
        if (result != 0) {
            error = QStringLiteral("nmc_set_cycletime(cycle=%1 us) 失败，错误码=%2")
                        .arg(cycleUs)
                        .arg(result);
            return false;
        }
        return true;
    }

    bool readBusCycle(int &cycleUs, QString &error) const
    {
        DWORD value = 0;
        const short result = nmc_get_cycletime(cardNo_, kEthercatPort, &value);
        if (result != 0) {
            error = QStringLiteral("nmc_get_cycletime 失败，错误码=%1").arg(result);
            return false;
        }
        cycleUs = static_cast<int>(value);
        return true;
    }

    bool close(QString &error)
    {
        stopFeedScheduler(true);
        feedbackTrace_.reset();
        traceAxes_.clear();
        latestAxisFeedback_.clear();
        pendingTelemetryFrames_.clear();
        latestTraceSequence_ = 0;
        traceFramesRead_ = 0;
        traceSamplePeriodUs_ = 0;
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

    bool configureTrace(const QVector<quint16> &axes, int samplePeriodUs,
                        int traceBaseCycleUs, QString &error)
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
        traceConfig.samplePeriodUs = samplePeriodUs;
        traceConfig.traceBaseCycleUs = traceBaseCycleUs;
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
        traceSamplePeriodUs_ = samplePeriodUs;
        traceStateText_ = QStringLiteral("Trace 已配置：%1 us，同帧 type 5/6 位置")
                              .arg(traceSamplePeriodUs_);
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
        // Trace 读取可能批量搬运多帧；读取前先补段，确保硬件线程内补段优先。
        serviceFeedQueue();
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
            frame.traceTimeUs = (sample.sequence > 0 ? sample.sequence - 1 : 0)
                * static_cast<quint64>(traceSamplePeriodUs_);
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
    static constexpr WORD kEthercatPort = 2;
    quint16 cardNo_ = 0;
    QVector<quint16> traceAxes_;
    QVector<AxisFeedback> latestAxisFeedback_;
    int traceFramesRead_ = 0;
    int traceSamplePeriodUs_ = 0;
    QVector<TraceTelemetryFrame> pendingTelemetryFrames_;
    quint64 latestTraceSequence_ = 0;
    QString traceStateText_ = QStringLiteral("Trace 未配置");
    QTimer *feedTimer_ = nullptr;
    QQueue<ContiFeedItem> feedQueue_;
    QMap<long, double> planTimeByMark_;
    ContiTestConfig streamConfig_;
    ContiFeedStatus feedStatus_;
    QElapsedTimer feedServiceClock_;
    qint64 lastFeedServiceUs_ = -1;
    bool streamListOpen_ = false;
    long lastAcceptedMark_ = 0;
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
bool E5000HardwareInterface::setBusCycle(int cycleUs, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->setBusCycle(cycleUs, error); }); }
bool E5000HardwareInterface::readBusCycle(int &cycleUs, QString &error) const
{ return invokeHardware(backend_, [&] { return backend_->readBusCycle(cycleUs, error); }); }
bool E5000HardwareInterface::configureAxes(const QVector<quint16> &axes, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->configureAxes(axes, error); }); }
bool E5000HardwareInterface::configureTrace(const QVector<quint16> &axes, int samplePeriodUs,
                                             int traceBaseCycleUs, QString &error)
{ return invokeHardware(backend_, [&] { return backend_->configureTrace(axes, samplePeriodUs, traceBaseCycleUs, error); }); }
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
bool E5000HardwareInterface::startStreaming(const ContiTestConfig &config,
                                            const QVector<ContiFeedItem> &points,
                                            bool preloadAllToCard,
                                            QString &error) const
{
    return invokeHardware(backend_, [&] {
        return backend_->startStreaming(config, points, preloadAllToCard, error);
    });
}
void E5000HardwareInterface::appendStreamingPoints(const QVector<ContiFeedItem> &points) const
{
    if (points.isEmpty()) {
        return;
    }
    QMetaObject::invokeMethod(backend_,
                              [backend = backend_, points] {
                                  backend->appendStreamingPoints(points);
                              },
                              Qt::QueuedConnection);
}
ContiFeedStatus E5000HardwareInterface::streamingStatus() const
{ return invokeHardware(backend_, [&] { return backend_->streamingStatus(); }); }
ContiSpeedRatioResult E5000HardwareInterface::changeSpeedRatio(const ContiTestConfig &config,
                                                                QString &error) const
{
    return invokeHardware(backend_, [&] {
        backend_->serviceFeedQueue();
        return backend_->card_.changeSpeedRatio(config, error);
    });
}
bool E5000HardwareInterface::stop(const ContiTestConfig &config, bool emergency, QString &error) const
{
    return invokeHardware(backend_, [&] {
        backend_->stopFeedScheduler(false);
        return backend_->card_.stop(config, emergency, error);
    });
}
bool E5000HardwareInterface::closeList(const ContiTestConfig &config, QString &error) const
{
    return invokeHardware(backend_, [&] {
        backend_->stopFeedScheduler(true);
        return backend_->card_.closeList(config, error);
    });
}
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
{ return invokeHardware(backend_, [&] { return backend_->traceSamplePeriodUs_; }); }
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
