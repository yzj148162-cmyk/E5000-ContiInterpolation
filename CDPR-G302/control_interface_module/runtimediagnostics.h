#ifndef RUNTIMEDIAGNOSTICS_H
#define RUNTIMEDIAGNOSTICS_H

#include "controlworker.h"
#include "hardwareinterface.h"
#include "runtimefeatureswitches.h"

#include <QtGlobal>

// 运行诊断的采样模型和窗口配置。报告生成仍读取 MainWindow 当前运行快照。
namespace RuntimeDiagnostics {

inline constexpr bool kEnableRuntimeDiagnosticsRecording =
        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;
inline constexpr bool kEnableRuntimeDiagnosticsAutoReport =
        RuntimeFeatureSwitches::kRuntimeDiagnosticsEnabled;
inline constexpr qint64 kRuntimeDiagnosticsWindowMs = 5 * 60 * 1000;
inline constexpr qint64 kRuntimeDiagnosticsAutoWriteIntervalMs = 5 * 60 * 1000;

struct Sample {
    qint64 capturedAtMs = 0;
    ControlWorker::TimingDiagnostics controlTiming;
    HardwareInterface::DiagnosticsSnapshot hardwareTiming;
};

struct Summary {
    qint64 windowDurationMs = 0;
    quint64 sensorIntervalCount = 0;
    double sensorAverageHz = 0.0;
    double sensorLatestHz = 0.0;
    quint64 communicationIntervalCount = 0;
    double communicationAverageHz = 0.0;
    double communicationLatestHz = 0.0;
    quint64 motorCommandIntervalCount = 0;
    double motorCommandAveragePeriodMs = 0.0;
    double motorCommandLatestPeriodMs = 0.0;
    quint64 controlLoopIntervalCount = 0;
    double controlLoopAveragePeriodMs = 0.0;
    double controlLoopLatestPeriodMs = 0.0;
};

} // namespace RuntimeDiagnostics

#endif // RUNTIMEDIAGNOSTICS_H
