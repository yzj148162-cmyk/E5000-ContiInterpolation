#ifndef ZOOMABLECHARTVIEW_H
#define ZOOMABLECHARTVIEW_H

#include <QtCharts/QChartView>

class QMouseEvent;
class QWheelEvent;

class ZoomableChartView : public QChartView
{
public:
    explicit ZoomableChartView(QWidget *parent = nullptr);

    void setAutomaticRange(double horizontalMinimum, double horizontalMaximum,
                           double verticalMinimum, double verticalMaximum);
    void resetAutomaticRangeMode();
    bool isManualZoomActive() const;

protected:
    void wheelEvent(QWheelEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    bool pointInPlotArea(const QPoint &viewPoint, QPointF *chartPoint = nullptr) const;
    void applyStoredAutomaticRange();

private:
    bool manualZoomActive_ = false;
    bool automaticRangeValid_ = false;
    double automaticHorizontalMinimum_ = 0.0;
    double automaticHorizontalMaximum_ = 1.0;
    double automaticVerticalMinimum_ = 0.0;
    double automaticVerticalMaximum_ = 1.0;
};

#endif // ZOOMABLECHARTVIEW_H
