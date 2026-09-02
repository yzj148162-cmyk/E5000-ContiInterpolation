/*
 * 文件总览：
 * - WinchCompensation 的实现文件，通过解析几何方程选择合理收放绳解，并在禁用配置时回退到线性角度比例。
 * - 核心难点是平台长度变化可能对应多个数学解，因此会利用 previousTakeupMm 做连续性选择。
 */

#include "winchcompensation.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double helicalLengthPerRev(const WinchCompensation::AxisConfig& config)
{
    const double circumference = 2.0 * kPi * config.drumRadiusMm;
    return std::hypot(circumference, config.pitchMmPerRev);
}

double axialRatioPerTakeup(const WinchCompensation::AxisConfig& config)
{
    const double lengthPerRev = helicalLengthPerRev(config);
    if (lengthPerRev <= kEps) {
        return 0.0;
    }
    return config.pitchMmPerRev / lengthPerRev;
}

double platformDeltaFromTakeup(
        const WinchCompensation::AxisConfig& config,
        double takeupMm)
{
    const double ratio = axialRatioPerTakeup(config);
    const double initialOffset = config.initialAxialOffsetValid &&
            std::isfinite(config.initialAxialOffsetMm) ?
                config.initialAxialOffsetMm :
                0.0;
    const double baselineLineLength = std::hypot(config.projectionMm, initialOffset);
    return takeupMm +
            std::hypot(config.projectionMm, initialOffset + ratio * takeupMm) -
            baselineLineLength;
}

bool isFinite(double value)
{
    return std::isfinite(value);
}

} // namespace

namespace WinchCompensation {

bool isEnabled(const AxisConfig& config)
{
    return config.enabled &&
           isFinite(config.drumRadiusMm) &&
           isFinite(config.pitchMmPerRev) &&
           isFinite(config.projectionMm) &&
           (!config.initialAxialOffsetValid || isFinite(config.initialAxialOffsetMm)) &&
           config.drumRadiusMm > kEps &&
           config.pitchMmPerRev > kEps &&
           config.projectionMm > kEps;
}

double helicalLengthPerRad(const AxisConfig& config)
{
    if (!isEnabled(config)) {
        return 0.0;
    }
    return helicalLengthPerRev(config) / (2.0 * kPi);
}

SolveResult solveTakeupFromPlatformDelta(
        const AxisConfig& config,
        double platformDeltaMm,
        double previousTakeupMm)
{
    if (!isFinite(platformDeltaMm)) {
        return {};
    }

    if (!isEnabled(config)) {
        return {platformDeltaMm, true};
    }

    const double cRatio = axialRatioPerTakeup(config);
    const double initialOffset = config.initialAxialOffsetValid &&
            isFinite(config.initialAxialOffsetMm) ?
                config.initialAxialOffsetMm :
                0.0;
    const double baselineLineLength =
            std::hypot(config.projectionMm, initialOffset);
    const double k = platformDeltaMm + baselineLineLength;
    const double a = cRatio * cRatio - 1.0;
    const double b = 2.0 * (k + initialOffset * cRatio);
    const double c = baselineLineLength * baselineLineLength - k * k;

    std::array<double, 2> roots = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()
    };
    int rootCount = 0;

    if (std::abs(a) <= kEps) {
        if (std::abs(b) <= kEps) {
            return {previousTakeupMm, isFinite(previousTakeupMm)};
        }
        roots[rootCount++] = -c / b;
    } else {
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < -1e-7) {
            return {previousTakeupMm, isFinite(previousTakeupMm)};
        }
        const double rootDiscriminant = std::sqrt(std::max(0.0, discriminant));
        roots[rootCount++] = (-b + rootDiscriminant) / (2.0 * a);
        roots[rootCount++] = (-b - rootDiscriminant) / (2.0 * a);
    }

    std::array<double, 2> validRoots = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()
    };
    int validCount = 0;
    for (int index = 0; index < rootCount; ++index) {
        const double root = roots[index];
        if (!isFinite(root)) {
            continue;
        }
        const double actualLineLength = k - root;
        if (actualLineLength >= config.projectionMm - 1e-7) {
            validRoots[validCount++] = root;
        }
    }

    if (validCount == 0) {
        double selected = roots[0];
        double bestError = std::numeric_limits<double>::infinity();
        for (int index = 0; index < rootCount; ++index) {
            const double root = roots[index];
            if (!isFinite(root)) {
                continue;
            }
            const double expectedLineLength =
                    std::hypot(config.projectionMm,
                               initialOffset + cRatio * root);
            const double error = std::abs((k - root) - expectedLineLength);
            if (error < bestError) {
                bestError = error;
                selected = root;
            }
        }
        return {selected, isFinite(selected)};
    }

    double selected = validRoots[0];
    if (validCount > 1) {
        if (isFinite(previousTakeupMm)) {
            const double firstDiff = std::abs(validRoots[0] - previousTakeupMm);
            const double secondDiff = std::abs(validRoots[1] - previousTakeupMm);
            selected = firstDiff <= secondDiff ? validRoots[0] : validRoots[1];
        } else {
            const double firstDiff = std::abs(validRoots[0] - platformDeltaMm);
            const double secondDiff = std::abs(validRoots[1] - platformDeltaMm);
            selected = firstDiff <= secondDiff ? validRoots[0] : validRoots[1];
        }
    }

    return {selected, isFinite(selected)};
}

double lengthDeltaFromPlatformDelta(
        const AxisConfig& config,
        double platformDeltaMm,
        double previousTakeupMm,
        double* selectedTakeupMm)
{
    const SolveResult result =
            solveTakeupFromPlatformDelta(config, platformDeltaMm, previousTakeupMm);
    if (selectedTakeupMm) {
        *selectedTakeupMm = result.takeupMm;
    }
    if (!result.valid || !isFinite(platformDeltaMm) || !isFinite(result.takeupMm)) {
        return 0.0;
    }
    return platformDeltaMm - result.takeupMm;
}

double motorThetaFromTakeup(
        const AxisConfig& config,
        double takeupMm,
        double fallbackAngleScaleRadPerMm)
{
    if (!isFinite(takeupMm)) {
        return 0.0;
    }
    if (!isEnabled(config)) {
        return std::isfinite(fallbackAngleScaleRadPerMm)
                ? takeupMm * fallbackAngleScaleRadPerMm
                : 0.0;
    }

    const double lengthPerRad = helicalLengthPerRad(config);
    if (lengthPerRad <= kEps) {
        return 0.0;
    }
    return takeupMm / lengthPerRad;
}

double motorThetaFromPlatformDelta(
        const AxisConfig& config,
        double platformDeltaMm,
        double fallbackAngleScaleRadPerMm,
        double previousTakeupMm,
        double* selectedTakeupMm)
{
    const SolveResult result =
            solveTakeupFromPlatformDelta(config, platformDeltaMm, previousTakeupMm);
    if (selectedTakeupMm) {
        *selectedTakeupMm = result.takeupMm;
    }
    return motorThetaFromTakeup(config, result.takeupMm, fallbackAngleScaleRadPerMm);
}

double platformDeltaFromMotorTheta(
        const AxisConfig& config,
        double motorThetaRad,
        double fallbackAngleScaleRadPerMm)
{
    if (!isFinite(motorThetaRad)) {
        return 0.0;
    }
    if (!isEnabled(config)) {
        if (!std::isfinite(fallbackAngleScaleRadPerMm) ||
                std::abs(fallbackAngleScaleRadPerMm) <= kEps) {
            return 0.0;
        }
        return motorThetaRad / fallbackAngleScaleRadPerMm;
    }

    const double lengthPerRad = helicalLengthPerRad(config);
    if (lengthPerRad <= kEps) {
        return 0.0;
    }
    return platformDeltaFromTakeup(config, motorThetaRad * lengthPerRad);
}

} // namespace WinchCompensation
