#ifndef DATAVISUALIZATIONCONTROLLER_H
#define DATAVISUALIZATIONCONTROLLER_H

// 数据可视化刷新和实际末端导数显示所用的固定平滑参数。
namespace DataVisualizationController {

inline constexpr double kActualEndEffectorVelocitySmoothingTauSec = 0.15;
inline constexpr double kActualEndEffectorAccelerationSmoothingTauSec = 0.35;

} // namespace DataVisualizationController

#endif // DATAVISUALIZATIONCONTROLLER_H
