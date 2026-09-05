#ifndef PHYSICALWORKSPACEBOUNDARY_H
#define PHYSICALWORKSPACEBOUNDARY_H

#include <array>
#include <vector>

#include <QString>

constexpr int kPhysicalWorkspaceMaximumPlatformPoints = 8;

// 机器模板唯一的物理工作空间定义。长度统一使用 mm，姿态使用 rad。
// platformPointsLocalMm 是动平台几何边缘的代表点；当前使用八个绳索连接点。
struct PhysicalWorkspaceBoundaryConfig
{
    std::array<double, 3> frameMinimumMm{{0.0, 0.0, 0.0}};
    std::array<double, 3> frameMaximumMm{{0.0, 0.0, 0.0}};
    std::vector<std::array<double, 3>> platformPointsLocalMm;
    bool orientationBoundsEnabled = false;
    std::array<double, 3> orientationMinimumRad{{0.0, 0.0, 0.0}};
    std::array<double, 3> orientationMaximumRad{{0.0, 0.0, 0.0}};

    bool validate(QString* errorMessage = nullptr) const;
};

// 末端状态的外部语义：位置 mm、线速度 mm/s、线加速度 mm/s^2；
// ZYX 欧拉角 rad，角速度/角加速度均在全局坐标系中表达。
struct PhysicalWorkspaceMotionSample
{
    std::array<double, 6> poseMmRad{};
    std::array<double, 6> twistMmRadPerSec{};
    std::array<double, 6> accelerationMmRadPerSec2{};
};

enum class PhysicalWorkspaceAction
{
    Safe,
    ControlledStop,
    EmergencyStop,
    Invalid
};

struct PhysicalWorkspaceBoundaryResult
{
    PhysicalWorkspaceAction action = PhysicalWorkspaceAction::Invalid;
    bool physicallyInside = false;
    bool orientationInside = false;
    double minimumClearanceMm = 0.0;
    double limitingClearanceMm = 0.0;
    double limitingOutwardSpeedMmPerSec = 0.0;
    double limitingOutwardAccelerationMmPerSec2 = 0.0;
    double pureStoppingDistanceMm = 0.0;
    double triggerDistanceMm = 0.0;
    int limitingPointIndex = -1;
    int limitingAxis = -1;
    bool limitingUpperFace = false;
    std::array<double, 3> limitingPointGlobalMm{};
    int platformPointCount = 0;
    std::array<std::array<double, 3>,
               kPhysicalWorkspaceMaximumPlatformPoints> platformPointsGlobalMm{};
    QString reason;
};

struct DynamicWorkspaceSafetyConfig
{
    // 不使用单独总延迟项：d_trigger=v_out^2/(2*a)+margin。
    double stoppingDecelerationMmPerSec2 = 500.0;
    double additionalSafetyMarginMm = 50.0;
    double emergencyLineMarginMm = 10.0;
    double outwardVelocityToleranceMmPerSec = 1.0e-6;
    double outwardAccelerationToleranceMmPerSec2 = 1.0e-6;

    bool validate(QString* errorMessage = nullptr) const;
};

class PhysicalWorkspaceBoundary
{
public:
    explicit PhysicalWorkspaceBoundary(
            const PhysicalWorkspaceBoundaryConfig& config = {});

    bool configure(const PhysicalWorkspaceBoundaryConfig& config,
                   QString* errorMessage = nullptr);
    bool isConfigured() const;
    const PhysicalWorkspaceBoundaryConfig& config() const;

    // 只检查几何/姿态是否位于物理工作空间，供阶段A和正运动学结果验收。
    PhysicalWorkspaceBoundaryResult evaluatePose(
            const std::array<double, 6>& poseMmRad) const;

    // 同时计算连接点法向速度、加速度和动态停车触发距离。
    PhysicalWorkspaceBoundaryResult evaluateMotion(
            const PhysicalWorkspaceMotionSample& sample,
            const DynamicWorkspaceSafetyConfig& safety) const;

    // 正运动学数值搜索的质心盒边界。最终解仍必须通过 evaluatePose。
    std::array<double, 6> solverLowerBounds() const;
    std::array<double, 6> solverUpperBounds() const;

private:
    PhysicalWorkspaceBoundaryConfig config_;
    bool configured_ = false;
};

#endif // PHYSICALWORKSPACEBOUNDARY_H
