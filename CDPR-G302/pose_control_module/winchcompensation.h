/*
 * 文件总览：
 * - WinchCompensation 描述绞盘螺旋卷绕补偿，把平台绳长变化、收放绳量和电机转角联系起来。
 * - AxisConfig 保存每根轴的卷筒半径、螺距、投影长度和初始轴向偏移。
 * - 该工具主要被 PositionSimulationModel 用于生成更接近实际卷绕几何的电机角度轨迹。
 */

#ifndef WINCHCOMPENSATION_H
#define WINCHCOMPENSATION_H

#include <limits>

namespace WinchCompensation {

struct AxisConfig {
    bool enabled = false;
    double drumRadiusMm = 0.0;
    double pitchMmPerRev = 0.0;
    double projectionMm = 0.0;
    bool initialAxialOffsetValid = false;
    double initialAxialOffsetMm = 0.0;
};

struct SolveResult {
    double takeupMm = 0.0;
    bool valid = false;
};

// 判断某轴是否配置了有效的螺旋卷绕补偿参数。
bool isEnabled(const AxisConfig& config);
// 计算电机转过 1 rad 时绳索沿螺旋线实际收放的长度。
double helicalLengthPerRad(const AxisConfig& config);

// 根据平台侧绳长变化反解卷筒收放量；有多解时优先贴近上一时刻收放量。
SolveResult solveTakeupFromPlatformDelta(
        const AxisConfig& config,
        double platformDeltaMm,
        double previousTakeupMm = std::numeric_limits<double>::quiet_NaN());

// 计算考虑螺旋卷绕后的电机侧长度变化，可同时返回选中的收放量。
double lengthDeltaFromPlatformDelta(
        const AxisConfig& config,
        double platformDeltaMm,
        double previousTakeupMm = std::numeric_limits<double>::quiet_NaN(),
        double* selectedTakeupMm = nullptr);

// 将卷筒收放量转换为电机转角；无补偿参数时使用线性比例兜底。
double motorThetaFromTakeup(
        const AxisConfig& config,
        double takeupMm,
        double fallbackAngleScaleRadPerMm);

// 从平台侧绳长变化直接计算电机转角，并输出本次选用的收放量。
double motorThetaFromPlatformDelta(
        const AxisConfig& config,
        double platformDeltaMm,
        double fallbackAngleScaleRadPerMm,
        double previousTakeupMm,
        double* selectedTakeupMm = nullptr);

// 将电机转角反算为平台侧绳长变化，用于显示和恢复逻辑。
double platformDeltaFromMotorTheta(
        const AxisConfig& config,
        double motorThetaRad,
        double fallbackAngleScaleRadPerMm);

} // namespace WinchCompensation

#endif // WINCHCOMPENSATION_H
