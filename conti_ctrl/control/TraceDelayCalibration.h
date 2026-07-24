#ifndef TRACEDELAYCALIBRATION_H
#define TRACEDELAYCALIBRATION_H

#include <QVector>

#include "common/ContiTypes.h"

struct TraceDelaySegmentCapture
{
    double targetSpeedDegreePerSecond = 0.0;
    QVector<TraceTelemetryFrame> frames;
};

struct TraceDelayFitResult
{
    TraceDelayAxisResult axisResult;
    QVector<double> measuredSpeedDegreePerSecond;
    QVector<double> measuredPositionGapDegree;
};

class TraceDelayCalibrationAnalyzer
{
public:
    static TraceDelayFitResult analyze(
        const TraceDelayCalibrationConfig &config,
        int traceSamplePeriodUs,
        const QVector<TraceDelaySegmentCapture> &segments);
};

#endif // TRACEDELAYCALIBRATION_H
