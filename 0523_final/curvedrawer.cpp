#include "curvedrawer.h"

#include <QtGlobal>

#include <algorithm>
#include <limits>

namespace {

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

} // namespace

CurveDrawer::CurveDrawer(QObject *parent)
    : QObject(parent)
{
    initConnections();

    plotTimer = new QTimer(this);
    plotTimer->setInterval(20);
    connect(plotTimer, &QTimer::timeout, this, [this]() {
        if (motorControlPlot) {
            motorControlPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (endEffectorPosPlot) {
            endEffectorPosPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (endEffectorVelPlot) {
            endEffectorVelPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (endEffectorAccPlot) {
            endEffectorAccPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (cableTensionPlot) {
            cableTensionPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (cableSpeedPlot) {
            cableSpeedPlot->replot(QCustomPlot::rpQueuedReplot);
        }
        if (cableLengthPlot) {
            cableLengthPlot->replot(QCustomPlot::rpQueuedReplot);
        }
    });
    plotTimer->start();
}

void CurveDrawer::setChannelCounts(int motorControlChannelCount, int cableChannelCount)
{
    this->motorControlChannelCount = std::max(1, motorControlChannelCount);
    this->cableChannelCount = std::max(1, cableChannelCount);
}

void CurveDrawer::initPlot(QCustomPlot *motorControlPlotWidget,
                           QCustomPlot *endEffectorPosPlotWidget,
                           QCustomPlot *endEffectorVelPlotWidget,
                           QCustomPlot *endEffectorAccPlotWidget,
                           QCustomPlot *cableTensionPlotWidget,
                           QCustomPlot *cableSpeedPlotWidget,
                           QCustomPlot *cableLengthPlotWidget)
{
    motorControlPlot = motorControlPlotWidget;
    endEffectorPosPlot = endEffectorPosPlotWidget;
    endEffectorVelPlot = endEffectorVelPlotWidget;
    endEffectorAccPlot = endEffectorAccPlotWidget;
    cableTensionPlot = cableTensionPlotWidget;
    cableSpeedPlot = cableSpeedPlotWidget;
    cableLengthPlot = cableLengthPlotWidget;

    setupPlot(motorControlPlot,
              motorControlChannelCount,
              QStringLiteral("控制输入"),
              buildAxisNames(QStringLiteral("轴"), motorControlChannelCount, 0));
    setupPlot(endEffectorPosPlot,
              6,
              QStringLiteral("位姿"),
              buildPoseAxisNames());
    setupPlot(endEffectorVelPlot,
              6,
              QStringLiteral("速度"),
              buildPoseAxisNames());
    setupPlot(endEffectorAccPlot,
              6,
              QStringLiteral("加速度"),
              buildPoseAxisNames());
    setupPlot(cableTensionPlot,
              cableChannelCount,
              QStringLiteral("绳索张力 (N)"),
              buildAxisNames(QStringLiteral("绳"), cableChannelCount));
    setupPlot(cableSpeedPlot,
              cableChannelCount,
              QStringLiteral("绳索速度 (mm/s)"),
              buildAxisNames(QStringLiteral("绳"), cableChannelCount));
    setupPlot(cableLengthPlot,
              cableChannelCount,
              QStringLiteral("绳索长度"),
              buildAxisNames(QStringLiteral("绳"), cableChannelCount));
}

void CurveDrawer::handleUpdateMotorControlPlot(double time, QVector<double> values)
{
    if (!motorControlPlot) {
        return;
    }

    const int graphCount = std::min(static_cast<int>(values.size()), motorControlPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        motorControlPlot->graph(i)->addData(time, values[i]);
        motorControlPlot->graph(i)->data()->removeBefore(time - timeWindow);
    }
    rescalePlotYAxis(motorControlPlot);
    motorControlPlot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
}

void CurveDrawer::handleUpdateEndEffectorPosPlot(double time, QVector<double> position, QVector<double> orientation)
{
    if (!endEffectorPosPlot) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        if (i < position.size()) {
            endEffectorPosPlot->graph(i)->addData(time, position[i]);
            endEffectorPosPlot->graph(i)->data()->removeBefore(time - timeWindow);
        }
        if (i < orientation.size()) {
            endEffectorPosPlot->graph(i + 3)->addData(time, orientation[i]);
            endEffectorPosPlot->graph(i + 3)->data()->removeBefore(time - timeWindow);
        }
    }
    rescalePlotYAxis(endEffectorPosPlot);
    endEffectorPosPlot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
}

void CurveDrawer::handleUpdateEndEffectorVelPlot(double time, QVector<double> velocity, QVector<double> angularVelocity)
{
    if (!endEffectorVelPlot) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        if (i < velocity.size()) {
            endEffectorVelPlot->graph(i)->addData(time, velocity[i]);
            endEffectorVelPlot->graph(i)->data()->removeBefore(time - timeWindow);
        }
        if (i < angularVelocity.size()) {
            endEffectorVelPlot->graph(i + 3)->addData(time, angularVelocity[i]);
            endEffectorVelPlot->graph(i + 3)->data()->removeBefore(time - timeWindow);
        }
    }
    rescalePlotYAxis(endEffectorVelPlot);
    endEffectorVelPlot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
}

void CurveDrawer::handleUpdateEndEffectorAccPlot(double time,
                                                 QVector<double> acceleration,
                                                 QVector<double> angularAcceleration)
{
    if (!endEffectorAccPlot) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        if (i < acceleration.size()) {
            endEffectorAccPlot->graph(i)->addData(time, acceleration[i]);
            endEffectorAccPlot->graph(i)->data()->removeBefore(time - timeWindow);
        }
        if (i < angularAcceleration.size()) {
            endEffectorAccPlot->graph(i + 3)->addData(time, angularAcceleration[i]);
            endEffectorAccPlot->graph(i + 3)->data()->removeBefore(time - timeWindow);
        }
    }
    rescalePlotYAxis(endEffectorAccPlot);
    endEffectorAccPlot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
}

void CurveDrawer::handleUpdateCableTensionPlot(double time, QVector<double> tensions)
{
    if (!cableTensionPlot) {
        return;
    }

    const int graphCount = std::min(static_cast<int>(tensions.size()), cableTensionPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        cableTensionPlot->graph(i)->addData(time, tensions[i]);
        cableTensionPlot->graph(i)->data()->removeBefore(time - timeWindow);
    }
    rescalePlotYAxis(cableTensionPlot);
    cableTensionPlot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
}

void CurveDrawer::handleUpdateCableSpeedPlot(double time, QVector<double> speeds)
{
    if (!cableSpeedPlot) {
        return;
    }

    const int graphCount = std::min(static_cast<int>(speeds.size()), cableSpeedPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        cableSpeedPlot->graph(i)->addData(time, speeds[i]);
        cableSpeedPlot->graph(i)->data()->removeBefore(time - timeWindow);
    }
    rescalePlotYAxis(cableSpeedPlot);
    cableSpeedPlot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
}

void CurveDrawer::handleUpdateCableLengthPlot(double time, QVector<double> lengths)
{
    if (!cableLengthPlot) {
        return;
    }

    const int graphCount = std::min(static_cast<int>(lengths.size()), cableLengthPlot->graphCount());
    for (int i = 0; i < graphCount; ++i) {
        cableLengthPlot->graph(i)->addData(time, lengths[i]);
        cableLengthPlot->graph(i)->data()->removeBefore(time - timeWindow);
    }
    rescalePlotYAxis(cableLengthPlot);
    cableLengthPlot->xAxis->setRange(std::max(time, timeWindow), timeWindow, Qt::AlignRight);
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
        plot->replot();
    };

    clearPlot(motorControlPlot);
    clearPlot(endEffectorPosPlot);
    clearPlot(endEffectorVelPlot);
    clearPlot(endEffectorAccPlot);
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

void CurveDrawer::setupPlot(QCustomPlot *plot,
                            int graphCount,
                            const QString &yLabel,
                            const QStringList &graphNames)
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
    plot->xAxis->setRange(0.0, timeWindow);
    plot->yAxis->setRange(-1.0, 1.0);

    const QVector<QColor> colors = plotColors();
    for (int i = 0; i < graphCount; ++i) {
        plot->addGraph();
        QCPGraph *graph = plot->graph(i);
        graph->setPen(QPen(colors[i % colors.size()], 1.5));
        if (i < graphNames.size()) {
            graph->setName(graphNames[i]);
        } else {
            graph->setName(QStringLiteral("通道%1").arg(i + 1));
        }
    }
}

void CurveDrawer::rescalePlotYAxis(QCustomPlot *plot) const
{
    if (!plot) {
        return;
    }

    bool hasData = false;
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < plot->graphCount(); ++i) {
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
        plot->yAxis->setRange(-1.0, 1.0);
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
    plot->yAxis->setRange(yRange);
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
