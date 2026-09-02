/*
 * 文件总览：
 * - RopeElasticCompensation 的实现文件，按胡克定律估算绳索在张力下的实际长度与规划长度差。
 * - 若配置未启用或物理参数无效，函数会返回原始规划长度，避免把异常参数传播到执行轨迹。
 */

#include "ropeelasticcompensation.h"

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-12;

// User override for rope elastic compensation inside hybrid pose-force control.
// Non-hybrid position trajectories never apply rope elastic compensation.
constexpr bool kRopeElasticCompensationEnabled = true;

// Lengths are in mm, tension is in N, so 120 GPa is 120000 N/mm^2.
constexpr double kRopeElasticModulusNPerMm2 = 120000.0;
constexpr double kRopeRadiusMm = 1.5;

// Fixed full-rope lengths outside the pulley tangent point to end exit point.
// Fill these eight values with the measured L0 for each cable.
constexpr double kFixedRopeLengthL0Cable1Mm = 6972.18;
constexpr double kFixedRopeLengthL0Cable2Mm = 2865.71;
constexpr double kFixedRopeLengthL0Cable3Mm = 6972.18;
constexpr double kFixedRopeLengthL0Cable4Mm = 2865.71;
constexpr double kFixedRopeLengthL0Cable5Mm = 6972.18;
constexpr double kFixedRopeLengthL0Cable6Mm = 2865.71;
constexpr double kFixedRopeLengthL0Cable7Mm = 6972.18;
constexpr double kFixedRopeLengthL0Cable8Mm = 2865.71;
constexpr std::array<double, RopeElasticCompensation::kCableCount> kFixedRopeLengthL0Mm = {
    kFixedRopeLengthL0Cable1Mm,
    kFixedRopeLengthL0Cable2Mm,
    kFixedRopeLengthL0Cable3Mm,
    kFixedRopeLengthL0Cable4Mm,
    kFixedRopeLengthL0Cable5Mm,
    kFixedRopeLengthL0Cable6Mm,
    kFixedRopeLengthL0Cable7Mm,
    kFixedRopeLengthL0Cable8Mm
};

bool isFinite(double value)
{
    return std::isfinite(value);
}

} // namespace

namespace RopeElasticCompensation {

Config defaultConfig()
{
    Config config;
    config.enabled = kRopeElasticCompensationEnabled;
    config.fixedLengthL0Mm = kFixedRopeLengthL0Mm;
    config.elasticModulusNPerMm2 = kRopeElasticModulusNPerMm2;
    config.ropeRadiusMm = kRopeRadiusMm;
    return config;
}

double areaMm2(const Config& config)
{
    if (!isFinite(config.ropeRadiusMm) || config.ropeRadiusMm <= kEps) {
        return 0.0;
    }
    return kPi * config.ropeRadiusMm * config.ropeRadiusMm;
}

bool isEnabled(const Config& config)
{
    return config.enabled &&
           isFinite(config.elasticModulusNPerMm2) &&
           config.elasticModulusNPerMm2 > kEps &&
           areaMm2(config) > kEps;
}

double actualDynamicLengthMm(const Config& config,
                             int cableIndex,
                             double plannedDynamicLengthMm,
                             double tensionN)
{
    if (!isEnabled(config) ||
            cableIndex < 0 ||
            cableIndex >= kCableCount ||
            !isFinite(plannedDynamicLengthMm) ||
            !isFinite(tensionN)) {
        return plannedDynamicLengthMm;
    }

    const double ea = config.elasticModulusNPerMm2 * areaMm2(config);
    if (ea <= kEps) {
        return plannedDynamicLengthMm;
    }

    const double forceRatio = tensionN / ea;
    const double denominator = 1.0 + forceRatio;
    if (std::abs(denominator) <= kEps) {
        return plannedDynamicLengthMm;
    }

    return (plannedDynamicLengthMm - forceRatio * config.fixedLengthL0Mm[cableIndex]) /
            denominator;
}

double lengthDeltaMm(const Config& config,
                     int cableIndex,
                     double plannedDynamicLengthMm,
                     double tensionN)
{
    return actualDynamicLengthMm(config,
                                 cableIndex,
                                 plannedDynamicLengthMm,
                                 tensionN) -
            plannedDynamicLengthMm;
}

} // namespace RopeElasticCompensation
