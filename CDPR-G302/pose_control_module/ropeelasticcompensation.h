/*
 * 文件总览：
 * - RopeElasticCompensation 提供绳索弹性伸长补偿，根据绳索材料刚度、半径、固定长度和当前张力修正动态绳长。
 * - 接口保持无状态，便于在轨迹仿真和离线计算中按点调用。
 */

#ifndef ROPEELASTICCOMPENSATION_H
#define ROPEELASTICCOMPENSATION_H

#include <array>

namespace RopeElasticCompensation {

constexpr int kCableCount = 8;

struct Config {
    bool enabled = false;
    std::array<double, kCableCount> fixedLengthL0Mm = {};
    double elasticModulusNPerMm2 = 0.0;
    double ropeRadiusMm = 0.0;
};

// 返回 8 根绳索的默认禁用配置。
Config defaultConfig();
// 判断弹性补偿参数是否完整且启用。
bool isEnabled(const Config& config);
// 根据绳半径计算截面积，供胡克定律伸长量计算使用。
double areaMm2(const Config& config);
// 根据张力和固定长度修正动态绳长，得到实际应下发/校验的长度。
double actualDynamicLengthMm(const Config& config,
                             int cableIndex,
                             double plannedDynamicLengthMm,
                             double tensionN);
// 只返回弹性伸长修正量，便于诊断或叠加到其它补偿项。
double lengthDeltaMm(const Config& config,
                     int cableIndex,
                     double plannedDynamicLengthMm,
                     double tensionN);

} // namespace RopeElasticCompensation

#endif // ROPEELASTICCOMPENSATION_H
