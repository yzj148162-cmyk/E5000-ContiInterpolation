#ifndef SESSIONRECORDER_H
#define SESSIONRECORDER_H

#include "controlworker.h"
#include "runtimefeatureswitches.h"

#include <QString>
#include <QtGlobal>

#include <vector>

// 会话记录的数据模型和功能开关。采样时机、UI 与硬件访问仍由 MainWindow 编排。
namespace SessionRecorder {

inline constexpr bool kEnableSessionRecordPvtControlCycleDiagnostics =
        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;
inline constexpr bool kEnableSessionRecordMotorEncoderUnitSampling =
        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;

struct PvtPositionCommandTable {
    qint64 capturedAtMs = 0;
    QString source;
    std::vector<int> motorIndex;
    std::vector<double> timeStamp;
    std::vector<std::vector<double>> positionUnit;
};

struct PvtControlCyclePoint {
    int pointIndex = 0;
    double trajectoryTimeSec = -1.0;
    qint64 cableLengthCalculationUs = -1;
    qint64 barycenterSolveUs = -1;
};

struct PvtControlCycleRecord {
    qint64 capturedAtMs = 0;
    QString source;
    int sourceStartPointIndex = 0;
    int pointCount = 0;
    int axisCount = 0;
    qint64 pvtGenerationElapsedUs = -1;
    qint64 pvtUploadTotalUs = -1;
    double pvtUploadAverageUsPerPoint = -1.0;
    qint64 pvtUploadMonotonicUs = -1;
    int pvtUploadPointCount = 0;
    int pvtUploadAxisCount = 0;
    bool traceStartDelayValid = false;
    quint32 traceCommandStartFrameSequence = 0;
    quint32 traceFeedbackStartFrameSequence = 0;
    quint64 traceStartDelayFrameCount = 0;
    int ethercatBusCycleUs = 500;
    qint64 traceStartDelayUs = -1;
    int traceCommandStartAxis = -1;
    int traceFeedbackStartAxis = -1;
    std::vector<PvtControlCyclePoint> points;
};

struct Sample {
    qint64 capturedAtMs = 0;
    qint64 relativeMs = 0;
    qint64 intervalMsSincePrevious = 0;
    quint64 controlSequence = 0;
    std::vector<double> motorAbsPos;
    std::vector<double> motorRelRawPos;
    std::vector<double> motorVel;
    std::vector<double> motorTorqueNm;
    std::vector<double> motorCommandNm;
    std::vector<double> forceSensorValue;
    std::vector<double> expectedForce;
    ControlWorker::TimingDiagnostics timingDiagnostics;
    bool cableDisplacementAvailable = false;
    std::vector<std::vector<double>> cableDisplacement;
    bool cableLengthAvailable = false;
    std::vector<std::vector<double>> cableLength;
    bool trajectoryPointAvailable = false;
    std::vector<double> trajectoryPoint;
    QString trajectoryPointSource;
};

struct State {
    bool active = false;
    qint64 startedAtMs = 0;
    qint64 endedAtMs = 0;
    std::vector<Sample> samples;
    std::vector<PvtPositionCommandTable> pvtPositionCommandTables;
    std::vector<PvtControlCycleRecord> pvtControlCycleRecords;
    QString lastExportPath;
};

} // namespace SessionRecorder

#endif // SESSIONRECORDER_H
