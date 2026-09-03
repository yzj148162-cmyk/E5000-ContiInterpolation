#ifndef FORCEINTERACTIONTYPES_H
#define FORCEINTERACTIONTYPES_H

#include <array>

#include <QString>
#include <QtGlobal>

constexpr int kForceInteractionDofCount = 6;
constexpr int kForceInteractionCableCount = 8;

using ForceInteractionVector3 = std::array<double, 3>;
using ForceInteractionVector6 = std::array<double, kForceInteractionDofCount>;
using ForceInteractionMatrix3 = std::array<double, 9>;

// 实测六维力受力/测量参考点位于动平台局部坐标系原点（正二十面体几何中心）
// 正上方 325.48 mm。动力学和力旋量变换统一使用 SI，因此此处保存为 m。
// 当前把该实测点作为传感器坐标系 S 的原点；若传感器手册另行定义测量原点，
// 应在实物标定时修正本向量。
inline constexpr ForceInteractionVector3 kMeasuredForceSensorOriginInPlatformM{{
    0.0, 0.0, 0.32548
}};

// 实测安装关系：传感器坐标系 S 的三轴方向与动平台局部坐标系 E 完全一致，
// 因而从 S 到 E 的旋转矩阵 R_ES 为单位阵。
inline constexpr ForceInteractionMatrix3 kMeasuredForceSensorToPlatformRotation{{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
}};

// Trace 时刻和主机接收时刻职责不同。纯软件验证不伪造 Trace 序号，
// 只填写主机单调时间；后续实机阶段再由同帧 Trace 数据填充完整标记。
struct ForceInteractionFrameStamp
{
    quint64 traceSequence = 0;
    qint64 traceTimeUs = 0;
    qint64 hostMonotonicTimeUs = 0;
    bool traceValid = false;
    bool valid = false;
};

enum class ForceInteractionWrenchCoordinate : quint8
{
    Sensor = 0,
    PlatformBodyAtCenterOfMass
};

// wrench = [Fx,Fy,Fz,Mx,My,Mz]，单位依次为 N 和 N·m。
struct ForceInteractionWrenchSample
{
    ForceInteractionFrameStamp stamp;
    ForceInteractionVector6 wrench{};
    ForceInteractionWrenchCoordinate coordinate =
            ForceInteractionWrenchCoordinate::PlatformBodyAtCenterOfMass;
    bool valid = false;
};

// 动力学内部统一采用 SI：平移 m，姿态 rad，速度 m/s、rad/s，
// 加速度 m/s^2、rad/s^2。进入 G302 运动学前仅将平移量换算为 mm。
struct ForceInteractionPlatformState
{
    ForceInteractionVector6 pose{};
    ForceInteractionVector6 twist{};
    ForceInteractionVector6 acceleration{};
    bool poseValid = false;
    bool twistValid = false;
    bool accelerationValid = false;
};

struct ForceInteractionRigidBodyConfig
{
    double massKg = 0.0;
    ForceInteractionMatrix3 inertiaKgM2{};
};

struct ForceSensorTransformConfig
{
    bool configured = false;
    ForceInteractionMatrix3 rotationSensorToPlatform =
            kMeasuredForceSensorToPlatformRotation;
    // 从动平台质心/局部原点指向传感器受力/测量参考点，在平台局部系表达。
    // 安装位置和坐标轴方向均已实测；configured 仍须等 Trace 通道映射和
    // 力/力矩正负方向全部确认后才能置 true。
    ForceInteractionVector3 sensorOriginInPlatformM =
            kMeasuredForceSensorOriginInPlatformM;
    ForceInteractionVector6 channelScale{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
    ForceInteractionVector6 channelBias{};
    int wrenchReactionSign = 1;
};

enum class SimulatedWrenchMode : quint8
{
    Constant = 0,
    Pulse,
    Sine,
    Formula
};

// 模拟量直接定义在平台质心 body frame，不依赖尚未标定的传感器安装关系。
struct SimulatedWrenchProfile
{
    SimulatedWrenchMode mode = SimulatedWrenchMode::Constant;
    ForceInteractionVector6 amplitude{};
    ForceInteractionVector6 bias{};
    double pulseStartS = 0.0;
    double pulseDurationS = 0.5;
    double sineFrequencyHz = 0.5;
    double sinePhaseRad = 0.0;
    std::array<QString, kForceInteractionDofCount> expressions{{
        QStringLiteral("0"), QStringLiteral("0"), QStringLiteral("0"),
        QStringLiteral("0"), QStringLiteral("0"), QStringLiteral("0")
    }};
};

#endif // FORCEINTERACTIONTYPES_H
