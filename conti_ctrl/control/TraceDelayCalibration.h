#ifndef TRACEDELAYCALIBRATION_H
#define TRACEDELAYCALIBRATION_H

#include <QVector>

#include "common/ContiTypes.h"

struct TraceDelaySegmentCapture
{
    double targetSpeedDegreePerSecond = 0.0;
    QVector<TraceTelemetryFrame> frames;
};

struct TraceDelaySegmentDiagnostic
{
    int segmentNumber = 0;
    double targetSpeedDegreePerSecond = 0.0;
    int capturedFrames = 0;
    int validAxisFrames = 0;
    int commandStableFrames = 0;
    int longestCommandStableRun = 0;
    int selectedFrames = 0;
    int lostFrames = 0;
    quint64 firstSequence = 0;
    quint64 lastSequence = 0;
    quint64 selectedFirstSequence = 0;
    quint64 selectedLastSequence = 0;
    double commandVelocityMeanDegreePerSecond = 0.0;
    double actualVelocityMeanDegreePerSecond = 0.0;
    double actualVelocityStdDegreePerSecond = 0.0;
    double positionGapMeanDegree = 0.0;
    bool accepted = false;
    QString detail;
};

struct TraceDelayFitResult
{
    TraceDelayAxisResult axisResult;
    QVector<double> measuredSpeedDegreePerSecond;
    QVector<double> measuredPositionGapDegree;
    QVector<TraceDelaySegmentDiagnostic> segmentDiagnostics;
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
