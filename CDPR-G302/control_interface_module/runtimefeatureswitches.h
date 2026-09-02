#ifndef RUNTIMEFEATURESWITCHES_H
#define RUNTIMEFEATURESWITCHES_H

// Central performance switches. Change these values to restore individual
// observation features without touching the control and safety paths.
namespace RuntimeFeatureSwitches {

// Disable all data-visualization sampling, curve accumulation and repainting.
inline constexpr bool kDataVisualizationEnabled = false;

// Disable runtime/session/online-velocity diagnostic recording and exporting.
// Safety fault logging is intentionally independent and remains enabled.
inline constexpr bool kRuntimeDiagnosticsEnabled = false;

// Keep one narrowly scoped attribution probe available for endpoint remote
// commissioning. It only accumulates atomic timing counters while that mode
// is active and emits one summary after stop/fault; it does not restore the
// high-rate runtime/session histories disabled above.
inline constexpr bool kEndpointRemoteAttributionDiagnosticsEnabled = true;

// Online velocity uses motor positions, velocities and feedback torque only.
// Force-sensor Trace objects are restored when leaving that run mode.
inline constexpr bool kOnlineVelocityForceSensorTraceEnabled = false;

// Only applies while waiting for the first reliable frame after a Trace profile
// switch. Normal running still uses the shorter continuous-feedback timeout.
inline constexpr long long kOnlineVelocityInitialTraceWaitTimeoutUs =
        3500LL * 1000LL;

} // namespace RuntimeFeatureSwitches

#endif // RUNTIMEFEATURESWITCHES_H
