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

// 2026-10-05：姿态调试按“传感器测量参考点等效位于动平台质心”建模。
// 因而平台质心到传感器原点的平移为零；动力学和力旋量变换统一使用 SI。
// 历史 +Z 偏移值只保留在设计文档中，不再参与当前计算。
inline constexpr ForceInteractionVector3 kMeasuredForceSensorOriginInPlatformM{{
    0.0, 0.0, 0.0
}};

// 当前安装关系：传感器坐标系 S 的三轴方向与动平台局部坐标系 E 完全一致，
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

// 阶段B/C共用同一运行内核，只在动力学入口选择力旋量来源。
enum class ForceInteractionWrenchSourceKind : quint8
{
    Simulated = 0,
    RealFtTrace
};

// 2026-10-07：阶段D执行后端在“准备阶段D”时冻结。默认值必须继续是
// 已验证的在线速度链；M3静态张力后端只在专用准入流程中显式选择。
enum class ForceInteractionExecutionMode : quint8
{
    OnlineVelocity = 0,
    StaticTensionTorqueExperimental
};

// Hardware-independent batch result shared by the M3 coordinator, runtime
// recorder and ControlWorker.  No SDK type crosses this boundary.
struct ForceInteractionTorqueBatchReport
{
    bool success = false;
    bool partialCommand = false;
    bool deferredBeforeWrite = false;
    qint64 hardwareQueueWaitUs = 0;
    qint64 budgetCheckedUs = 0;
    qint64 budgetRemainingUs = 0;
    int failedLogicalAxis = -1;
    qint64 firstCommandMonotonicUs = 0;
    qint64 lastCommandMonotonicUs = 0;
    qint64 apiDurationUs = 0;
    QString message;
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

// 模拟量与真实F/T Trace保持相同语义：传感器测量参考点、传感器坐标系S下
// 的原始六维力。进入动力学前统一经过WrenchTransformer转换到平台质心。
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
