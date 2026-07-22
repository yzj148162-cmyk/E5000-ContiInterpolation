#include "ZoomableChartView.h"

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtMath>

#include <cmath>

namespace {
constexpr double kZoomStep = 1.15;
constexpr double kMinimumAxisSpan = 1.0e-9;
constexpr double kMaximumAxisSpan = 1.0e12;
}

ZoomableChartView::ZoomableChartView(QWidget *parent)
    : QChartView(parent)
{
    setMouseTracking(true);
}

void ZoomableChartView::setAutomaticRange(double horizontalMinimum, double horizontalMaximum,
                                          double verticalMinimum, double verticalMaximum)
{
    automaticHorizontalMinimum_ = horizontalMinimum;
    automaticHorizontalMaximum_ = horizontalMaximum;
    automaticVerticalMinimum_ = verticalMinimum;
    automaticVerticalMaximum_ = verticalMaximum;
    automaticRangeValid_ = true;
    if (!manualZoomActive_) {
        applyStoredAutomaticRange();
    }
}

void ZoomableChartView::resetAutomaticRangeMode()
{
    manualZoomActive_ = false;
    applyStoredAutomaticRange();
}

bool ZoomableChartView::isManualZoomActive() const
{
    return manualZoomActive_;
}

void ZoomableChartView::wheelEvent(QWheelEvent *event)
{
    if (chart() == nullptr || event->angleDelta().y() == 0) {
        QChartView::wheelEvent(event);
        return;
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QPoint viewPoint = event->position().toPoint();
#else
    const QPoint viewPoint = event->pos();
#endif
    QPointF chartPoint;
    if (!pointInPlotArea(viewPoint, &chartPoint)) {
        QChartView::wheelEvent(event);
        return;
    }

    QValueAxis *horizontalAxis = nullptr;
    QValueAxis *verticalAxis = nullptr;
    for (QAbstractAxis *axis : chart()->axes(Qt::Horizontal)) {
        if ((horizontalAxis = qobject_cast<QValueAxis *>(axis)) != nullptr) {
            break;
        }
    }
    for (QAbstractAxis *axis : chart()->axes(Qt::Vertical)) {
        if ((verticalAxis = qobject_cast<QValueAxis *>(axis)) != nullptr) {
            break;
        }
    }
    if (horizontalAxis == nullptr || verticalAxis == nullptr) {
        QChartView::wheelEvent(event);
        return;
    }

    const double horizontalSpan = horizontalAxis->max() - horizontalAxis->min();
    const double verticalSpan = verticalAxis->max() - verticalAxis->min();
    const double wheelSteps = static_cast<double>(event->angleDelta().y()) / 120.0;
    const double scaleFactor = std::pow(kZoomStep, -wheelSteps);
    const double newHorizontalSpan = horizontalSpan * scaleFactor;
    const double newVerticalSpan = verticalSpan * scaleFactor;
    if (newHorizontalSpan < kMinimumAxisSpan || newVerticalSpan < kMinimumAxisSpan
        || newHorizontalSpan > kMaximumAxisSpan || newVerticalSpan > kMaximumAxisSpan) {
        event->accept();
        return;
    }

    const QRectF plotArea = chart()->plotArea();
    const double horizontalRatio = qBound(0.0,
                                          (chartPoint.x() - plotArea.left()) / plotArea.width(),
                                          1.0);
    const double verticalRatio = qBound(0.0,
                                        (plotArea.bottom() - chartPoint.y()) / plotArea.height(),
                                        1.0);
    const double horizontalAnchor = horizontalAxis->min() + horizontalRatio * horizontalSpan;
    const double verticalAnchor = verticalAxis->min() + verticalRatio * verticalSpan;

    horizontalAxis->setRange(horizontalAnchor - horizontalRatio * newHorizontalSpan,
                             horizontalAnchor + (1.0 - horizontalRatio) * newHorizontalSpan);
    verticalAxis->setRange(verticalAnchor - verticalRatio * newVerticalSpan,
                           verticalAnchor + (1.0 - verticalRatio) * newVerticalSpan);
    manualZoomActive_ = true;
    event->accept();
}

void ZoomableChartView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && pointInPlotArea(event->pos())) {
        panning_ = true;
        lastPanPosition_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QChartView::mousePressEvent(event);
}

void ZoomableChartView::mouseMoveEvent(QMouseEvent *event)
{
    if (!panning_) {
        setCursor(pointInPlotArea(event->pos()) ? Qt::OpenHandCursor : Qt::ArrowCursor);
        QChartView::mouseMoveEvent(event);
        return;
    }

    if (chart() == nullptr || !(event->buttons() & Qt::LeftButton)) {
        panning_ = false;
        setCursor(Qt::OpenHandCursor);
        QChartView::mouseMoveEvent(event);
        return;
    }

    QValueAxis *horizontalAxis = nullptr;
    QValueAxis *verticalAxis = nullptr;
    for (QAbstractAxis *axis : chart()->axes(Qt::Horizontal)) {
        if ((horizontalAxis = qobject_cast<QValueAxis *>(axis)) != nullptr) {
            break;
        }
    }
    for (QAbstractAxis *axis : chart()->axes(Qt::Vertical)) {
        if ((verticalAxis = qobject_cast<QValueAxis *>(axis)) != nullptr) {
            break;
        }
    }

    const QRectF plotArea = chart()->plotArea();
    if (horizontalAxis == nullptr || verticalAxis == nullptr
        || plotArea.width() <= 0.0 || plotArea.height() <= 0.0) {
        event->accept();
        return;
    }

    const QPoint delta = event->pos() - lastPanPosition_;
    lastPanPosition_ = event->pos();
    if (delta.isNull()) {
        event->accept();
        return;
    }

    const double horizontalSpan = horizontalAxis->max() - horizontalAxis->min();
    const double verticalSpan = verticalAxis->max() - verticalAxis->min();
    double horizontalMinimum = horizontalAxis->min()
        - static_cast<double>(delta.x()) * horizontalSpan / plotArea.width();
    double horizontalMaximum = horizontalMinimum + horizontalSpan;
    double verticalMinimum = verticalAxis->min()
        + static_cast<double>(delta.y()) * verticalSpan / plotArea.height();
    double verticalMaximum = verticalMinimum + verticalSpan;

    const auto clampToAutomaticRange = [](double &minimum, double &maximum,
                                          double currentMinimum, double currentMaximum,
                                          double automaticMinimum, double automaticMaximum) {
        const double span = maximum - minimum;
        const double automaticSpan = automaticMaximum - automaticMinimum;
        if (automaticSpan <= 0.0) {
            return;
        }
        if (span >= automaticSpan) {
            minimum = currentMinimum;
            maximum = currentMaximum;
            return;
        }
        if (minimum < automaticMinimum) {
            minimum = automaticMinimum;
            maximum = minimum + span;
        }
        if (maximum > automaticMaximum) {
            maximum = automaticMaximum;
            minimum = maximum - span;
        }
    };
    if (automaticRangeValid_) {
        clampToAutomaticRange(horizontalMinimum, horizontalMaximum,
                              horizontalAxis->min(), horizontalAxis->max(),
                              automaticHorizontalMinimum_, automaticHorizontalMaximum_);
        clampToAutomaticRange(verticalMinimum, verticalMaximum,
                              verticalAxis->min(), verticalAxis->max(),
                              automaticVerticalMinimum_, automaticVerticalMaximum_);
    }

    horizontalAxis->setRange(horizontalMinimum, horizontalMaximum);
    verticalAxis->setRange(verticalMinimum, verticalMaximum);
    manualZoomActive_ = true;
    event->accept();
}

void ZoomableChartView::mouseReleaseEvent(QMouseEvent *event)
{
    if (panning_ && event->button() == Qt::LeftButton) {
        panning_ = false;
        setCursor(pointInPlotArea(event->pos()) ? Qt::OpenHandCursor : Qt::ArrowCursor);
        event->accept();
        return;
    }
    QChartView::mouseReleaseEvent(event);
}

void ZoomableChartView::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (pointInPlotArea(event->pos())) {
        resetAutomaticRangeMode();
        event->accept();
        return;
    }
    QChartView::mouseDoubleClickEvent(event);
}

bool ZoomableChartView::pointInPlotArea(const QPoint &viewPoint, QPointF *chartPoint) const
{
    if (chart() == nullptr || chart()->plotArea().isEmpty()) {
        return false;
    }
    const QPointF mappedPoint = chart()->mapFromScene(mapToScene(viewPoint));
    if (chartPoint != nullptr) {
        *chartPoint = mappedPoint;
    }
    return chart()->plotArea().contains(mappedPoint);
}

void ZoomableChartView::applyStoredAutomaticRange()
{
    if (!automaticRangeValid_ || chart() == nullptr) {
        return;
    }
    for (QAbstractAxis *axis : chart()->axes(Qt::Horizontal)) {
        if (QValueAxis *valueAxis = qobject_cast<QValueAxis *>(axis)) {
            valueAxis->setRange(automaticHorizontalMinimum_, automaticHorizontalMaximum_);
            break;
        }
    }
    for (QAbstractAxis *axis : chart()->axes(Qt::Vertical)) {
        if (QValueAxis *valueAxis = qobject_cast<QValueAxis *>(axis)) {
            valueAxis->setRange(automaticVerticalMinimum_, automaticVerticalMaximum_);
            break;
        }
    }
}
