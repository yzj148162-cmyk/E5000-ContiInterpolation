#ifndef TRACEDELAYCALIBRATION_H
#define TRACEDELAYCALIBRATION_H

#include <array>
#include <vector>
#include <QString>
#include <QtGlobal>

struct TraceDelayCalibrationSample {
    quint32 frameSequence = 0;
    qint64 monotonicUs = 0;
    double commandPositionUnit = 0.0;
    double actualPositionUnit = 0.0;
    double commandVelocityUnitPerSec = 0.0;
    double actualVelocityUnitPerSec = 0.0;
    bool valid = false;
};

struct TraceDelayCalibrationSegment {
    double targetVelocityUnitPerSec = 0.0;
    std::vector<TraceDelayCalibrationSample> samples;
};

struct TraceDelayCalibrationConfig {
    int axis = 0;
    bool allAxes = false;
    QString actuatorProfileKey;
    double axisEquivalentPulsePerUnit = 0.0;
    std::array<double, 3> speeds{{30.0, 60.0, 100.0}};
    int holdMs = 1500;
    int sampleWindowMs = 500;
    int restMs = 500;
    double onlineChangeTimeS = 0.001;
    double maximumSegmentTravelUnit = 180.0;
    QString recordingDirectory;
};

struct TraceDelayAxisResult {
    int axis = 0;
    bool calibrated = false;
    bool valid = false;
    bool stale = false;
    double appliedDelayMs = 8.0;
    double measuredDelayMs = 0.0;
    double staticOffsetUnit = 0.0;
    double rSquared = 0.0;
    double rmseUnit = 0.0;
    double pairSpreadMs = 0.0;
    int lostFrameCount = 0;
    QString source = QStringLiteral("默认");
    QString timestamp;
    QString detail = QStringLiteral("尚未标定");
};

struct TraceDelaySegmentDiagnostic {
    int segmentNumber = 0;
    double targetVelocityUnitPerSec = 0.0;
    int capturedFrames = 0;
    int stableFrames = 0;
    int selectedFrames = 0;
    int lostFrames = 0;
    quint32 selectedFirstSequence = 0;
    quint32 selectedLastSequence = 0;
    double commandVelocityMean = 0.0;
    double actualVelocityMean = 0.0;
    double actualVelocityStd = 0.0;
    double positionGapMean = 0.0;
    bool accepted = false;
    QString detail;
};

struct TraceDelayFitResult {
    TraceDelayAxisResult axisResult;
    std::vector<double> fittedVelocity;
    std::vector<double> fittedPositionGap;
    std::vector<TraceDelaySegmentDiagnostic> segmentDiagnostics;
};

struct TraceDelayCalibrationStatus {
    enum class State { Idle, Configuring, Resting, Moving, Stopping, Finalizing, Completed, Fault };
    State state = State::Idle;
    bool active = false;
    bool allAxes = false;
    int axis = -1;
    int currentAxisOrdinal = 0;
    int totalAxes = 0;
    int currentSegment = 0;
    int totalSegments = 6;
    int progressPercent = 0;
    double targetVelocityUnitPerSec = 0.0;
    QString profileKey;
    QString phaseText = QStringLiteral("未运行");
    QString message;
    QString rawDataFile;
    qint64 fitDurationUs = 0;
    qint64 csvWriteDurationUs = 0;
    qint64 persistenceDurationUs = 0;
    qint64 traceRestoreDurationUs = 0;
    std::array<TraceDelayAxisResult, 8> axisResults{};
};

class TraceDelayCalibrationAnalyzer {
public:
    static TraceDelayFitResult analyze(const TraceDelayCalibrationConfig& config,
                                       int traceSamplePeriodUs,
                                       const std::vector<TraceDelayCalibrationSegment>& segments);
};

#endif
