/*
 * 文件总览：
 * - CurveDrawer 的实现文件，集中配置 QCustomPlot 的曲线数量、名称、交互行为和数据窗口。
 * - 各 handleUpdate 函数只追加数据，定时器统一 replot，避免高频信号导致 UI 频繁重绘。
 */

#include "curvedrawer.h"

#include <QDateTime>
#include <QtGlobal>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr int kPlotRefreshIntervalMs = 50;
constexpr int kPlotYAxisRescaleIntervalMs = 200;
constexpr int kPlotDataTrimIntervalMs = 200;
constexpr double kPlotYAxisOverflowRatio = 0.05;

QVector<QColor> plotColors()
{
    return {
        QColor(0, 102, 204),
        QColor(0, 153, 102),
        QColor(204, 51, 51),
        QColor(153, 102, 0),
        QColor(102, 51, 153),
        QColor(0, 153, 153),
        QColor(204, 102, 0),
        QColor(102, 102, 102),
        QColor(153, 51, 102),
        QColor(51, 102, 51),
        QColor(51, 51, 153),
        QColor(153, 0, 51)
    };
}

void updateFiniteRange(double value, double& minValue, double& maxValue)
{
    if(!std::isfinite(value)){
        return;
    }
    minValue = std::min(minValue, value);
    maxValue = std::max(maxValue, value);
}

} // namespace

CurveDrawer::CurveDrawer(QObject *parent)
    : QObject(parent)
{
    initConnections();

    plotTimer = new QTimer(this);
    plotTimer->setInterval(kPlotRefreshIntervalMs);
    connect(plotTimer, &QTimer::timeout, this, &CurveDrawer::handlePlotTimerTick);
    plotTimer->start();
}

void CurveDrawer::setChannelCounts(int motorControlChannelCount, int cableChannelCount)
{
    this->motorControlChannelCount = std::max(1, motorControlChannelCount);
    this->cableChannelCount = std::max(1, cableChannelCount);
}

void CurveDrawer::setRefreshEnabled(bool enabled)
{
    if(!plotTimer){
        return;
    }
    if(enabled){
        if(!plotTimer->isActive()){
            plotTimer->start();
        }
        handlePlotTimerTick();
        return;
    }
    if(plotTimer->isActive()){
        plotTimer->stop();
    }
}

void CurveDrawer::setMotorControlHybridMode(bool enabled, const QVector<int>& forceGraphIndexes)
{
    QVector<int> normalizedForceGraphIndexes;
    if(enabled){
        normalizedForceGraphIndexes.reserve(forceGraphIndexes.size());
        for(int graphIndex : forceGraphIndexes){
            if(graphIndex < 0 ||
                    graphIndex >= motorControlChannelCount ||
                    normalizedForceGraphIndexes.contains(graphIndex)){
                continue;
            }
            normalizedForceGraphIndexes.push_back(graphIndex);
        }
        std::sort(normalizedForceGraphIndexes.begin(), normalizedForceGraphIndexes.end());
    }

    const bool useHybridMode = enabled && !normalizedForceGraphIndexes.empty();
    const bool changed =
            motorControlHybridMode != useHybridMode ||
            motorControlHybridForceGraphIndexes != normalizedForceGraphIndexes;
    motorControlHybridMode = useHybridMode;
    motorControlHybridForceGraphIndexes = normalizedForceGraphIndexes;
    applyMotorControlPlotMode(changed);
}

void CurveDrawer::initPlot(QCustomPlot *motorControlPlotWidget,
                           QCustomPlot *endEffectorPosPlotWidget,
                           QCustomPlot *endEffectorVelPlotWidget,
                           QCustomPlot *endEffectorAccPlotWidget,
                           QCustomPlot *cableTensionPlotWidget,
                           QCustomPlot *cableSpeedPlotWidget,
                           QCustomPlot *cableLengthPlotWidget,
                           QCustomPlot *actualEndEffectorPosPlotWidget,
                           QCustomPlot *actualEndEffectorVelPlotWidget,
                           QCustomPlot *actualEndEffectorAccPlotWidget)
{
    motorControlPlot = motorControlPlotWidget;
    endEffectorPosPlot = endEffectorPosPlotWidget;
    endEffectorVelPlot = endEffectorVelPlotWidget;
    endEffectorAccPlot = endEffectorAccPlotWidget;
    actualEndEffectorPosPlot = actualEndEffectorPosPlotWidget;
    actualEndEffectorVelPlot = actualEndEffectorVelPlotWidget;
    actualEndEffectorAccPlot = actualEndEffectorAccPlotWidget;
    cableTensionPlot = cableTensionPlotWidget;
    cableSpeedPlot = cableSpeedPlotWidget;
    cableLengthPlot = cableLengthPlotWidget;

    setupPlot(motorControlPlot,
              motorControlChannelCount,
              QStringLiteral("控制输入 (unit)"),
              buildAxisNames(QStringLiteral("轴"), motorControlChannelCount));
    motorControlHybridMode = false;
    motorControlHybridForceGraphIndexes.clear();
    applyMotorControlPlotMode(false);
    setupPlot(endEffectorPosPlot,
              6,
              QStringLiteral("位置 (mm)"),
              buildPoseAxisNames(),
              true,
              QStringLiteral("姿态 (rad)"));
    setupPlot(endEffectorVelPlot,
              6,
              QStringLiteral("速度 (mm/s)"),
              buildPoseAxisNames(),
              true,
              QStringLiteral("角速度 (rad/s)"));
    setupPlot(endEffectorAccPlot,
              6,
              QStringLiteral("加速度 (mm/s^2)"),
              buildPoseAxisNames(),
              true,
              QStringLiteral("角加速度 (rad/s^2)"));
    setupPlot(actualEndEffectorPosPlot,
              6,
              QStringLiteral("位置 (mm)"),
              buildPoseAxisNames(),
              true,
              QStringLiteral("姿态 (rad)"));
    setupPlot(actualEndEffectorVelPlot,
              6,
              QStringLiteral("速度 (mm/s)"),
              buildPoseAxisNames(),
              true,
              QStringLiteral("角速度 (rad/s)"));
    setupPlot(actualEndEffectorAccPlot,
              6,
              QStringLiteral("加速度 (mm/s^2)"),
              buildPoseAxisNames(),
              true,
              QStringLiteral("角加速度 (rad/s^2)"));
    setupPlot(cableTensionPlot,
              cableChannelCount,
              QStringLiteral("绳索张力 (N)"),
              buildAxisNames(QStringLiteral("轴"), cableChannelCount));
    setupPlot(cableSpeedPlot,
              cableChannelCount,
              QStringLiteral("绳索速度 (mm/s)"),
              buildAxisNames(QStringLiteral("轴"), cableChannelCount));
    setupPlot(cableLengthPlot,
              cableChannelCount,
              QStringLiteral("绳索长度 (mm)"),
              buildAxisNames(QStringLiteral("轴"), cableChannelCount));
}

void CurveDrawer::handleUpdateMotorControlPlot(double time, QVector<double> values)
{
    if (!motorControlPlot) {
        return;
    }

    double leftMinSample = std::numeric_limits<double>::infinity();
    double leftMaxSample = -std::numeric_limits<double>::infinity();
    double rightMinSample = std::numeric_limits<double>::infinity();
    double rightMaxSample = -std::numeric_limits<double>::infinity();
    bool hasSample = false;
    const int graphCount = std::min(static_cast<int>(values.size()), motorControlPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        if(!std::isfinite(values[i])){
            continue;
        }
        motorControlPlot->graph(i)->addData(time, values[i]);
        if(motorControlHybridMode && isMotorControlForceGraph(i)){
            updateFiniteRange(values[i], rightMinSample, rightMaxSample);
        }
        else{
            updateFiniteRange(values[i], leftMinSample, leftMaxSample);
        }
        hasSample = true;
    }
    if(!hasSample){
        return;
    }
    if(motorControlHybridMode){
        updateDualAxisPlotAfterAppend(motorControlPlot,
                                      time,
                                      leftMinSample,
                                      leftMaxSample,
                                      rightMinSample,
                                      rightMaxSample);
    }
    else{
        updatePlotAfterAppend(motorControlPlot, time, leftMinSample, leftMaxSample);
    }
}

void CurveDrawer::handleUpdateEndEffectorPosPlot(double time, QVector<double> position, QVector<double> orientation)
{
    if (!endEffectorPosPlot) {
        return;
    }

    double leftMinSample = std::numeric_limits<double>::infinity();
    double leftMaxSample = -std::numeric_limits<double>::infinity();
    double rightMinSample = std::numeric_limits<double>::infinity();
    double rightMaxSample = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
        if (i < position.size()) {
            endEffectorPosPlot->graph(i)->addData(time, position[i]);
            updateFiniteRange(position[i], leftMinSample, leftMaxSample);
        }
        if (i < orientation.size()) {
            endEffectorPosPlot->graph(i + 3)->addData(time, orientation[i]);
            updateFiniteRange(orientation[i], rightMinSample, rightMaxSample);
        }
    }
    updateDualAxisPlotAfterAppend(endEffectorPosPlot,
                                  time,
                                  leftMinSample,
                                  leftMaxSample,
                                  rightMinSample,
                                  rightMaxSample);
}

void CurveDrawer::handleUpdateEndEffectorVelPlot(double time, QVector<double> velocity, QVector<double> angularVelocity)
{
    if (!endEffectorVelPlot) {
        return;
    }

    double leftMinSample = std::numeric_limits<double>::infinity();
    double leftMaxSample = -std::numeric_limits<double>::infinity();
    double rightMinSample = std::numeric_limits<double>::infinity();
    double rightMaxSample = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
        if (i < velocity.size()) {
            endEffectorVelPlot->graph(i)->addData(time, velocity[i]);
            updateFiniteRange(velocity[i], leftMinSample, leftMaxSample);
        }
        if (i < angularVelocity.size()) {
            endEffectorVelPlot->graph(i + 3)->addData(time, angularVelocity[i]);
            updateFiniteRange(angularVelocity[i], rightMinSample, rightMaxSample);
        }
    }
    updateDualAxisPlotAfterAppend(endEffectorVelPlot,
                                  time,
                                  leftMinSample,
                                  leftMaxSample,
                                  rightMinSample,
                                  rightMaxSample);
}

void CurveDrawer::handleUpdateEndEffectorAccPlot(double time,
                                                 QVector<double> acceleration,
                                                 QVector<double> angularAcceleration)
{
    if (!endEffectorAccPlot) {
        return;
    }

    double leftMinSample = std::numeric_limits<double>::infinity();
    double leftMaxSample = -std::numeric_limits<double>::infinity();
    double rightMinSample = std::numeric_limits<double>::infinity();
    double rightMaxSample = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
        if (i < acceleration.size()) {
            endEffectorAccPlot->graph(i)->addData(time, acceleration[i]);
            updateFiniteRange(acceleration[i], leftMinSample, leftMaxSample);
        }
        if (i < angularAcceleration.size()) {
            endEffectorAccPlot->graph(i + 3)->addData(time, angularAcceleration[i]);
            updateFiniteRange(angularAcceleration[i], rightMinSample, rightMaxSample);
        }
    }
    updateDualAxisPlotAfterAppend(endEffectorAccPlot,
                                  time,
                                  leftMinSample,
                                  leftMaxSample,
                                  rightMinSample,
                                  rightMaxSample);
}

void CurveDrawer::handleUpdateActualEndEffectorPosPlot(double time, QVector<double> position, QVector<double> orientation)
{
    if(!actualEndEffectorPosPlot){
        return;
    }

    double leftMinSample = std::numeric_limits<double>::infinity();
    double leftMaxSample = -std::numeric_limits<double>::infinity();
    double rightMinSample = std::numeric_limits<double>::infinity();
    double rightMaxSample = -std::numeric_limits<double>::infinity();
    for(int i=0; i<3; ++i){
        if(i < position.size()){
            actualEndEffectorPosPlot->graph(i)->addData(time, position[i]);
            updateFiniteRange(position[i], leftMinSample, leftMaxSample);
        }
        if(i < orientation.size()){
            actualEndEffectorPosPlot->graph(i + 3)->addData(time, orientation[i]);
            updateFiniteRange(orientation[i], rightMinSample, rightMaxSample);
        }
    }
    updateDualAxisPlotAfterAppend(actualEndEffectorPosPlot,
                                  time,
                                  leftMinSample,
                                  leftMaxSample,
                                  rightMinSample,
                                  rightMaxSample);
}

void CurveDrawer::handleUpdateActualEndEffectorVelPlot(double time, QVector<double> velocity, QVector<double> angularVelocity)
{
    if(!actualEndEffectorVelPlot){
        return;
    }

    double leftMinSample = std::numeric_limits<double>::infinity();
    double leftMaxSample = -std::numeric_limits<double>::infinity();
    double rightMinSample = std::numeric_limits<double>::infinity();
    double rightMaxSample = -std::numeric_limits<double>::infinity();
    for(int i=0; i<3; ++i){
        if(i < velocity.size()){
            actualEndEffectorVelPlot->graph(i)->addData(time, velocity[i]);
            updateFiniteRange(velocity[i], leftMinSample, leftMaxSample);
        }
        if(i < angularVelocity.size()){
            actualEndEffectorVelPlot->graph(i + 3)->addData(time, angularVelocity[i]);
            updateFiniteRange(angularVelocity[i], rightMinSample, rightMaxSample);
        }
    }
    updateDualAxisPlotAfterAppend(actualEndEffectorVelPlot,
                                  time,
                                  leftMinSample,
                                  leftMaxSample,
                                  rightMinSample,
                                  rightMaxSample);
}

void CurveDrawer::handleUpdateActualEndEffectorAccPlot(double time,
                                                       QVector<double> acceleration,
                                                       QVector<double> angularAcceleration)
{
    if(!actualEndEffectorAccPlot){
        return;
    }

    double leftMinSample = std::numeric_limits<double>::infinity();
    double leftMaxSample = -std::numeric_limits<double>::infinity();
    double rightMinSample = std::numeric_limits<double>::infinity();
    double rightMaxSample = -std::numeric_limits<double>::infinity();
    for(int i=0; i<3; ++i){
        if(i < acceleration.size()){
            actualEndEffectorAccPlot->graph(i)->addData(time, acceleration[i]);
            updateFiniteRange(acceleration[i], leftMinSample, leftMaxSample);
        }
        if(i < angularAcceleration.size()){
            actualEndEffectorAccPlot->graph(i + 3)->addData(time, angularAcceleration[i]);
            updateFiniteRange(angularAcceleration[i], rightMinSample, rightMaxSample);
        }
    }
    updateDualAxisPlotAfterAppend(actualEndEffectorAccPlot,
                                  time,
                                  leftMinSample,
                                  leftMaxSample,
                                  rightMinSample,
                                  rightMaxSample);
}

void CurveDrawer::handleUpdateCableTensionPlot(double time, QVector<double> tensions)
{
    if (!cableTensionPlot) {
        return;
    }

    double minSample = std::numeric_limits<double>::infinity();
    double maxSample = -std::numeric_limits<double>::infinity();
    const int graphCount = std::min(static_cast<int>(tensions.size()), cableTensionPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        cableTensionPlot->graph(i)->addData(time, tensions[i]);
        updateFiniteRange(tensions[i], minSample, maxSample);
    }
    updatePlotAfterAppend(cableTensionPlot, time, minSample, maxSample);
}

void CurveDrawer::handleUpdateCableSpeedPlot(double time, QVector<double> speeds)
{
    if (!cableSpeedPlot) {
        return;
    }

    double minSample = std::numeric_limits<double>::infinity();
    double maxSample = -std::numeric_limits<double>::infinity();
    const int graphCount = std::min(static_cast<int>(speeds.size()), cableSpeedPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        cableSpeedPlot->graph(i)->addData(time, speeds[i]);
        updateFiniteRange(speeds[i], minSample, maxSample);
    }
    updatePlotAfterAppend(cableSpeedPlot, time, minSample, maxSample);
}

void CurveDrawer::handleUpdateCableLengthPlot(double time, QVector<double> lengths)
{
    if (!cableLengthPlot) {
        return;
    }

    double minSample = std::numeric_limits<double>::infinity();
    double maxSample = -std::numeric_limits<double>::infinity();
    const int graphCount = std::min(static_cast<int>(lengths.size()), cableLengthPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        cableLengthPlot->graph(i)->addData(time, lengths[i]);
        updateFiniteRange(lengths[i], minSample, maxSample);
    }
    updatePlotAfterAppend(cableLengthPlot, time, minSample, maxSample);
}

void CurveDrawer::handleClearAllData()
{
    const auto clearPlot = [this](QCustomPlot *plot) {
        if (!plot) {
            return;
        }

        for (int i = 0; i < plot->graphCount(); ++i) {
            plot->graph(i)->data()->clear();
        }
        plot->xAxis->setRange(0.0, timeWindow);
        plot->yAxis->setRange(-1.0, 1.0);
        plot->yAxis2->setRange(-1.0, 1.0);
        if(PlotRefreshState* state = refreshStateForPlot(plot)){
            *state = PlotRefreshState{};
        }
        if(isPlotVisibleForRefresh(plot)){
            plot->replot();
        }
        else if(PlotRefreshState* state = refreshStateForPlot(plot)){
            state->dirty = true;
        }
    };

    clearPlot(motorControlPlot);
    clearPlot(endEffectorPosPlot);
    clearPlot(endEffectorVelPlot);
    clearPlot(endEffectorAccPlot);
    clearPlot(actualEndEffectorPosPlot);
    clearPlot(actualEndEffectorVelPlot);
    clearPlot(actualEndEffectorAccPlot);
    clearPlot(cableTensionPlot);
    clearPlot(cableSpeedPlot);
    clearPlot(cableLengthPlot);
}

void CurveDrawer::initConnections()
{
    connect(this,
            &CurveDrawer::updateMotorControlPlotSignal,
            this,
            &CurveDrawer::handleUpdateMotorControlPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateEndEffectorPosPlotSignal,
            this,
            &CurveDrawer::handleUpdateEndEffectorPosPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateEndEffectorVelPlotSignal,
            this,
            &CurveDrawer::handleUpdateEndEffectorVelPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateEndEffectorAccPlotSignal,
            this,
            &CurveDrawer::handleUpdateEndEffectorAccPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateActualEndEffectorPosPlotSignal,
            this,
            &CurveDrawer::handleUpdateActualEndEffectorPosPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateActualEndEffectorVelPlotSignal,
            this,
            &CurveDrawer::handleUpdateActualEndEffectorVelPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateActualEndEffectorAccPlotSignal,
            this,
            &CurveDrawer::handleUpdateActualEndEffectorAccPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateCableTensionPlotSignal,
            this,
            &CurveDrawer::handleUpdateCableTensionPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateCableSpeedPlotSignal,
            this,
            &CurveDrawer::handleUpdateCableSpeedPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::updateCableLengthPlotSignal,
            this,
            &CurveDrawer::handleUpdateCableLengthPlot,
            Qt::QueuedConnection);
    connect(this,
            &CurveDrawer::clearAllDataSignal,
            this,
            &CurveDrawer::handleClearAllData,
            Qt::QueuedConnection);
}

void CurveDrawer::handlePlotTimerTick()
{
    emit plotRefreshTick(plotTimer ? plotTimer->interval() : kPlotRefreshIntervalMs);
    refreshDirtyPlot(motorControlPlot, motorControlPlotState);
    refreshDirtyPlot(endEffectorPosPlot, endEffectorPosPlotState);
    refreshDirtyPlot(endEffectorVelPlot, endEffectorVelPlotState);
    refreshDirtyPlot(endEffectorAccPlot, endEffectorAccPlotState);
    refreshDirtyPlot(actualEndEffectorPosPlot, actualEndEffectorPosPlotState);
    refreshDirtyPlot(actualEndEffectorVelPlot, actualEndEffectorVelPlotState);
    refreshDirtyPlot(actualEndEffectorAccPlot, actualEndEffectorAccPlotState);
    refreshDirtyPlot(cableTensionPlot, cableTensionPlotState);
    refreshDirtyPlot(cableSpeedPlot, cableSpeedPlotState);
    refreshDirtyPlot(cableLengthPlot, cableLengthPlotState);
}

CurveDrawer::PlotRefreshState* CurveDrawer::refreshStateForPlot(QCustomPlot *plot)
{
    if(!plot){
        return nullptr;
    }
    if(plot == motorControlPlot){
        return &motorControlPlotState;
    }
    if(plot == endEffectorPosPlot){
        return &endEffectorPosPlotState;
    }
    if(plot == endEffectorVelPlot){
        return &endEffectorVelPlotState;
    }
    if(plot == endEffectorAccPlot){
        return &endEffectorAccPlotState;
    }
    if(plot == actualEndEffectorPosPlot){
        return &actualEndEffectorPosPlotState;
    }
    if(plot == actualEndEffectorVelPlot){
        return &actualEndEffectorVelPlotState;
    }
    if(plot == actualEndEffectorAccPlot){
        return &actualEndEffectorAccPlotState;
    }
    if(plot == cableTensionPlot){
        return &cableTensionPlotState;
    }
    if(plot == cableSpeedPlot){
        return &cableSpeedPlotState;
    }
    if(plot == cableLengthPlot){
        return &cableLengthPlotState;
    }
    return nullptr;
}

void CurveDrawer::refreshDirtyPlot(QCustomPlot *plot, PlotRefreshState& state)
{
    if(!plot || !state.dirty || !isPlotVisibleForRefresh(plot)){
        return;
    }

    if(state.forceRescale){
        rescalePlotYAxis(plot);
        state.lastRescaleMs = QDateTime::currentMSecsSinceEpoch();
        state.forceRescale = false;
    }
    plot->replot(QCustomPlot::rpQueuedReplot);
    state.dirty = false;
}

void CurveDrawer::updatePlotAfterAppend(QCustomPlot *plot,
                                        double time,
                                        double minSample,
                                        double maxSample)
{
    if(!plot){
        return;
    }

    PlotRefreshState* state = refreshStateForPlot(plot);
    if(!state){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    trimPlotDataIfDue(plot, *state, time, nowMs);
    plot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
    state->dirty = true;

    if(!isPlotVisibleForRefresh(plot)){
        state->forceRescale = true;
        return;
    }
    maybeRescalePlotYAxis(plot, *state, minSample, maxSample, nowMs);
}

void CurveDrawer::updateDualAxisPlotAfterAppend(QCustomPlot *plot,
                                                double time,
                                                double leftMinSample,
                                                double leftMaxSample,
                                                double rightMinSample,
                                                double rightMaxSample)
{
    if(!plot){
        return;
    }

    PlotRefreshState* state = refreshStateForPlot(plot);
    if(!state){
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    trimPlotDataIfDue(plot, *state, time, nowMs);
    plot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
    state->dirty = true;

    if(!isPlotVisibleForRefresh(plot)){
        state->forceRescale = true;
        return;
    }
    maybeRescaleDualYAxis(plot,
                          *state,
                          leftMinSample,
                          leftMaxSample,
                          rightMinSample,
                          rightMaxSample,
                          nowMs);
}

void CurveDrawer::trimPlotDataIfDue(QCustomPlot *plot,
                                    PlotRefreshState& state,
                                    double time,
                                    qint64 nowMs)
{
    if(!plot ||
            (state.lastTrimMs > 0 &&
             nowMs - state.lastTrimMs < kPlotDataTrimIntervalMs)){
        return;
    }

    for(int i = 0; i < plot->graphCount(); ++i){
        if(plot->graph(i)){
            plot->graph(i)->data()->removeBefore(time - timeWindow);
        }
    }
    state.lastTrimMs = nowMs;
}

void CurveDrawer::maybeRescalePlotYAxis(QCustomPlot *plot,
                                        PlotRefreshState& state,
                                        double minSample,
                                        double maxSample,
                                        qint64 nowMs)
{
    if(!plot){
        return;
    }

    const bool rescaleDue =
            state.lastRescaleMs <= 0 ||
            nowMs - state.lastRescaleMs >= kPlotYAxisRescaleIntervalMs;
    const bool sampleOutOfRange =
            sampleOutsideAxisRange(plot ? plot->yAxis : nullptr, minSample, maxSample);
    if(!state.forceRescale && !rescaleDue && !sampleOutOfRange){
        return;
    }

    rescalePlotYAxis(plot);
    state.lastRescaleMs = nowMs;
    state.forceRescale = false;
}

void CurveDrawer::maybeRescaleDualYAxis(QCustomPlot *plot,
                                        PlotRefreshState& state,
                                        double leftMinSample,
                                        double leftMaxSample,
                                        double rightMinSample,
                                        double rightMaxSample,
                                        qint64 nowMs)
{
    if(!plot){
        return;
    }

    const bool rescaleDue =
            state.lastRescaleMs <= 0 ||
            nowMs - state.lastRescaleMs >= kPlotYAxisRescaleIntervalMs;
    const bool sampleOutOfRange =
            sampleOutsideAxisRange(plot->yAxis, leftMinSample, leftMaxSample) ||
            sampleOutsideAxisRange(plot->yAxis2, rightMinSample, rightMaxSample);
    if(!state.forceRescale && !rescaleDue && !sampleOutOfRange){
        return;
    }

    rescalePlotYAxis(plot);
    state.lastRescaleMs = nowMs;
    state.forceRescale = false;
}

bool CurveDrawer::isPlotVisibleForRefresh(QCustomPlot *plot) const
{
    if(!plot || !plot->isVisible()){
        return false;
    }

    QWidget* window = plot->window();
    return !window || !window->isMinimized();
}

bool CurveDrawer::sampleOutsideAxisRange(QCPAxis *axis,
                                         double minSample,
                                         double maxSample) const
{
    if(!axis ||
            !std::isfinite(minSample) ||
            !std::isfinite(maxSample)){
        return false;
    }

    const QCPRange range = axis->range();
    if(!std::isfinite(range.lower) ||
            !std::isfinite(range.upper) ||
            range.upper <= range.lower){
        return true;
    }
    const double overflowPadding =
            std::max(1e-6, range.size() * kPlotYAxisOverflowRatio);
    return minSample < range.lower - overflowPadding ||
            maxSample > range.upper + overflowPadding;
}

bool CurveDrawer::isDualYAxisPosePlot(QCustomPlot *plot) const
{
    return plot == endEffectorPosPlot ||
            plot == endEffectorVelPlot ||
            plot == endEffectorAccPlot ||
            plot == actualEndEffectorPosPlot ||
            plot == actualEndEffectorVelPlot ||
            plot == actualEndEffectorAccPlot;
}

bool CurveDrawer::isMotorControlForceGraph(int graphIndex) const
{
    return motorControlHybridMode &&
            motorControlHybridForceGraphIndexes.contains(graphIndex);
}

void CurveDrawer::applyMotorControlPlotMode(bool clearData)
{
    if(!motorControlPlot){
        return;
    }

    motorControlPlot->yAxis->setLabel(
                motorControlHybridMode ?
                    QStringLiteral("位置控制输入 (unit)") :
                    QStringLiteral("控制输入 (unit)"));
    motorControlPlot->yAxis2->setVisible(motorControlHybridMode);
    motorControlPlot->yAxis2->setTickLabels(motorControlHybridMode);
    motorControlPlot->yAxis2->setLabel(
                motorControlHybridMode ?
                    QStringLiteral("期望力 (N)") :
                    QString());

    const QVector<QColor> colors = plotColors();
    for(int i = 0; i < motorControlPlot->graphCount(); ++i){
        QCPGraph* graph = motorControlPlot->graph(i);
        if(!graph){
            continue;
        }

        const bool forceGraph = isMotorControlForceGraph(i);
        graph->setValueAxis(forceGraph ? motorControlPlot->yAxis2 : motorControlPlot->yAxis);
        QPen pen(colors[i % colors.size()], 1.5);
        pen.setStyle(forceGraph ? Qt::DashLine : Qt::SolidLine);
        graph->setPen(pen);
        graph->setName(QStringLiteral("轴%1").arg(i + 1));
        if(clearData){
            graph->data()->clear();
        }
    }

    if(clearData){
        motorControlPlot->xAxis->setRange(0.0, timeWindow);
        motorControlPlot->yAxis->setRange(-1.0, 1.0);
        motorControlPlot->yAxis2->setRange(-1.0, 1.0);
    }

    if(PlotRefreshState* state = refreshStateForPlot(motorControlPlot)){
        if(clearData){
            *state = PlotRefreshState{};
        }
        state->dirty = true;
        state->forceRescale = true;
    }
}

void CurveDrawer::setupPlot(QCustomPlot *plot,
                            int graphCount,
                            const QString &yLabel,
                            const QStringList &graphNames,
                            bool dualYAxis,
                            const QString &rightYLabel)
{
    if (!plot) {
        return;
    }

    plot->clearGraphs();

    plot->legend->setVisible(true);
    plot->legend->setBrush(QColor(255, 255, 255, 210));
    plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    plot->axisRect()->setupFullAxesBox();
    plot->xAxis->setLabel(QStringLiteral("时间 (s)"));
    plot->yAxis->setLabel(yLabel);
    plot->xAxis2->setVisible(true);
    plot->xAxis2->setTickLabels(false);
    plot->yAxis2->setVisible(dualYAxis);
    plot->yAxis2->setTickLabels(dualYAxis);
    plot->yAxis2->setLabel(dualYAxis ? rightYLabel : QString());
    plot->xAxis->setRange(0.0, timeWindow);
    plot->yAxis->setRange(-1.0, 1.0);
    plot->yAxis2->setRange(-1.0, 1.0);

    const QVector<QColor> colors = plotColors();
    for (int i = 0; i < graphCount; ++i) {
        plot->addGraph();
        QCPGraph *graph = plot->graph(i);
        graph->setValueAxis(dualYAxis && i >= 3 ? plot->yAxis2 : plot->yAxis);
        QPen pen(colors[i % colors.size()], 1.5);
        if(dualYAxis && i >= 3){
            pen.setStyle(Qt::DashLine);
        }
        graph->setPen(pen);
        if (i < graphNames.size()) {
            graph->setName(graphNames[i]);
        } else {
            graph->setName(QStringLiteral("通道%1").arg(i + 1));
        }
    }
    if(PlotRefreshState* state = refreshStateForPlot(plot)){
        *state = PlotRefreshState{};
        state->dirty = true;
        state->forceRescale = true;
    }
}

void CurveDrawer::rescalePlotYAxis(QCustomPlot *plot) const
{
    if (!plot) {
        return;
    }

    if(plot == motorControlPlot && motorControlHybridMode){
        rescaleAxisForMotorControlHybridGraphs(plot->yAxis, false);
        rescaleAxisForMotorControlHybridGraphs(plot->yAxis2, true);
        return;
    }

    if(isDualYAxisPosePlot(plot)){
        rescaleAxisForGraphRange(plot, plot->yAxis, 0, 3);
        rescaleAxisForGraphRange(plot, plot->yAxis2, 3, 3);
        return;
    }

    rescaleAxisForGraphRange(plot, plot->yAxis, 0, plot->graphCount());
}

void CurveDrawer::rescaleAxisForGraphRange(QCustomPlot *plot,
                                           QCPAxis *axis,
                                           int firstGraph,
                                           int graphCount) const
{
    if(!plot || !axis || firstGraph < 0 || graphCount <= 0){
        return;
    }

    bool hasData = false;
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    const int endGraph = std::min(plot->graphCount(), firstGraph + graphCount);
    for (int i = firstGraph; i < endGraph; ++i) {
        if (!plot->graph(i) || plot->graph(i)->dataCount() == 0) {
            continue;
        }
        bool foundRange = false;
        const QCPRange valueRange = plot->graph(i)->getValueRange(foundRange, QCP::sdBoth);
        if (!foundRange) {
            continue;
        }
        minValue = std::min(minValue, valueRange.lower);
        maxValue = std::max(maxValue, valueRange.upper);
        hasData = true;
    }

    if (!hasData) {
        axis->setRange(-1.0, 1.0);
        return;
    }

    QCPRange yRange(minValue, maxValue);
    if (qFuzzyIsNull(yRange.size())) {
        yRange.lower -= 1.0;
        yRange.upper += 1.0;
    } else {
        const double padding = std::max(1e-6, yRange.size() * 0.1);
        yRange.lower -= padding;
        yRange.upper += padding;
    }
    axis->setRange(yRange);
}

void CurveDrawer::rescaleAxisForMotorControlHybridGraphs(QCPAxis *axis, bool forceGraphs) const
{
    if(!motorControlPlot || !axis){
        return;
    }

    bool hasData = false;
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    for(int i = 0; i < motorControlPlot->graphCount(); ++i){
        const bool graphIsForce =
                motorControlHybridForceGraphIndexes.contains(i);
        if(graphIsForce != forceGraphs ||
                !motorControlPlot->graph(i) ||
                motorControlPlot->graph(i)->dataCount() == 0){
            continue;
        }

        bool foundRange = false;
        const QCPRange valueRange =
                motorControlPlot->graph(i)->getValueRange(foundRange, QCP::sdBoth);
        if(!foundRange){
            continue;
        }
        minValue = std::min(minValue, valueRange.lower);
        maxValue = std::max(maxValue, valueRange.upper);
        hasData = true;
    }

    if(!hasData){
        axis->setRange(-1.0, 1.0);
        return;
    }

    QCPRange yRange(minValue, maxValue);
    if(qFuzzyIsNull(yRange.size())){
        yRange.lower -= 1.0;
        yRange.upper += 1.0;
    }
    else{
        const double padding = std::max(1e-6, yRange.size() * 0.1);
        yRange.lower -= padding;
        yRange.upper += padding;
    }
    axis->setRange(yRange);
}

QStringList CurveDrawer::buildAxisNames(const QString &prefix, int count, int startIndex)
{
    QStringList names;
    names.reserve(count);
    for (int i = 0; i < count; ++i) {
        names.push_back(QStringLiteral("%1%2").arg(prefix).arg(startIndex + i));
    }
    return names;
}

QStringList CurveDrawer::buildPoseAxisNames()
{
    return {
        QStringLiteral("X"),
        QStringLiteral("Y"),
        QStringLiteral("Z"),
        QStringLiteral("Rx"),
        QStringLiteral("Ry"),
        QStringLiteral("Rz")
    };
}
