#ifndef CDPRCONTROLTYPES_H
#define CDPRCONTROLTYPES_H

#include <array>

#include <QMetaType>
#include <QString>
#include <QtGlobal>
#include <QVector>

constexpr int kCdprDofCount = 6;
constexpr int kCdprCableCount = 8;

using CdprVector6 = std::array<double, kCdprDofCount>;
using CdprVector8 = std::array<double, kCdprCableCount>;
using CdprMatrix8x6 =
    std::array<std::array<double, kCdprDofCount>, kCdprCableCount>;

struct CdprVector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 全部时间戳均使用同一单调时钟的微秒值。序号和时间戳属于整帧，
// 防止把不同控制周期的期望、命令和反馈拼成一帧。
struct CdprFrameStamp
{
    quint64 sequence = 0;
    qint64 monotonicTimeUs = 0;
    bool valid = false;
};

enum class CdprInitialPoseSource : quint8
{
    Preset = 0,
    NokovMarkers
};

enum class CdprForceInputSource : quint8
{
    Simulated = 0,
    TraceFtSensor
};

enum class CdprWrenchCoordinate : quint8
{
    Sensor = 0,
    PlatformBodyAtCenterOfMass
};

// wrench = [Fx, Fy, Fz, Mx, My, Mz]，单位依次为 N 和 N·m。
struct CdprWrenchSample
{
    CdprFrameStamp stamp;
    CdprVector6 wrench {};
    CdprWrenchCoordinate coordinate = CdprWrenchCoordinate::Sensor;
    bool valid = false;
};

// pose = [x, y, z, rx, ry, rz]，姿态为 Rz*Ry*Rx，角度单位 rad。
// twist/acceleration 的后三项分别为世界坐标系角速度和角加速度。
struct CdprPlatformState6
{
    CdprVector6 pose {};
    CdprVector6 twist {};
    CdprVector6 acceleration {};
    bool poseValid = false;
    bool twistValid = false;
    bool accelerationValid = false;
};

// 绳长增加为正；长度、速度、加速度单位依次为 m、m/s、m/s^2。
struct CdprCableState8
{
    CdprVector8 lengthM {};
    CdprVector8 velocityMps {};
    CdprVector8 accelerationMps2 {};
    bool lengthValid = false;
    bool velocityValid = false;
    bool accelerationValid = false;
};

struct CdprAxisCommandFrame8
{
    CdprFrameStamp stamp;
    CdprVector8 targetPositionDegree {};
    CdprVector8 targetVelocityDegreePerSecond {};
    quint16 validAxisMask = 0;
};

struct CdprAxisFeedbackFrame8
{
    CdprFrameStamp stamp;
    CdprVector8 commandPositionDegree {};
    CdprVector8 actualPositionDegree {};
    CdprVector8 actualVelocityDegreePerSecond {};
    quint16 validAxisMask = 0;
};

// 每次实机启动重新建立的基准，不能由配置文件中的参考位姿替代。
struct CdprStartupState
{
    CdprFrameStamp stamp;
    CdprInitialPoseSource poseSource = CdprInitialPoseSource::Preset;
    CdprPlatformState6 initialPlatform;
    CdprCableState8 initialCables;
    CdprVector8 initialAxisPositionDegree {};
    quint16 validAxisMask = 0;
    bool poseStable = false;
    bool valid = false;
};

enum class CdprRunState : quint8
{
    Unconfigured = 0,
    Configured,
    Ready,
    Running,
    Stopping,
    FaultLatched
};

// CDPR控制链每周期只发布一份完整快照。协调器是唯一写入者，
// UI、绘图和记录模块只消费快照。
struct CdprRobotState
{
    CdprFrameStamp stamp;
    CdprRunState runState = CdprRunState::Unconfigured;
    CdprPlatformState6 desiredPlatform;
    CdprPlatformState6 actualPlatform;
    CdprCableState8 desiredCables;
    CdprCableState8 actualCables;
    CdprWrenchSample rawWrench;
    CdprWrenchSample platformWrench;
    CdprAxisCommandFrame8 axisCommand;
    CdprAxisFeedbackFrame8 axisFeedback;
    quint32 safetyFlags = 0;
    bool safetyLatched = false;
};

// 已知末端测试轨迹的公共请求。离线PVT和八轴实时速度闭环分别调用独立
// 生成入口；只有离线PVT入口受板卡装表点数策略限制。
struct CdprOfflinePvtRequest
{
    // UI用于丢弃参数变化前仍在后台生成的旧速度闭环轨迹；不参与运动计算。
    quint64 requestId = 0;
    CdprVector6 relativePose {};
    double durationS = 5.0;
    int samplePeriodMs = 10;
    double winchRadiusM = 0.08;
    double maximumAxisVelocityDegreePerSecond = 720.0;
    double degreesPerCardUnit = 1.0;
};

// 公共参考轨迹数据结构。离线PVT将其装入板卡表，八轴速度闭环只在主机端
// 按控制周期查询它，不向PVT表装点，因此不继承PVT点数上限。
struct CdprOfflinePvtPlan
{
    bool valid = false;
    QString errorText;
    QString summary;
    CdprOfflinePvtRequest request;
    QVector<double> timeS;
    // 与timeS严格同一时间轴的末端期望位姿。离线虚拟运动学分析直接
    // 使用它，避免在运行结束后依赖已变化的UI或配置重新生成轨迹。
    QVector<CdprVector6> platformPose;
    // 本次规划所用结构配置的完整JSON快照。运行记录会原样写入
    // configuration_snapshot.json，保证旧记录可独立复算。
    QString configurationSnapshotJson;
    // 八轴速度闭环的预生成轨迹缓存。离线PVT仍在每次运行目录中保存独立副本。
    QString planId;
    QString expectedTrajectoryPath;
    QString expectedTrajectorySha256;
    std::array<quint16, kCdprCableCount> axes {};
    std::array<int, kCdprCableCount> directions {};
    std::array<QVector<double>, kCdprCableCount> axisPositionDegree;
    // 与位置表同一时间轴的解析速度。离线 PVT 不使用该字段，
    // 但八轴实时速度闭环直接以它作为速度前馈。
    std::array<QVector<double>, kCdprCableCount> axisVelocityDegreePerSecond;
    CdprCableState8 initialCables;
    CdprCableState8 finalCables;
    CdprVector8 finalAxisDisplacementDegree {};
    CdprVector8 peakAxisVelocityDegreePerSecond {};
};

// 八轴实时速度模式位置闭环。轨迹仍由 CDPR 运动学预检生成；执行阶段不装 PVT
// 表，而是在每个控制周期针对八根轴在线更新速度命令。
struct CdprVelocityControlConfig
{
    int controlPeriodMs = 1;
    double degreesPerCardUnit = 1.0;
    bool pidEnabled = true;
    bool velocityFeedforwardEnabled = true;
    double velocityFeedforwardGain = 1.0;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integralLimitDegreeSecond = 10.0;
    double maxPidCorrectionDegreePerSecond = 20.0;
    double maxVelocityDegreePerSecond = 720.0;
    double maxAccelerationDegreePerSecond2 = 2000.0;
    double onlineChangeTimeS = 0.001;
    double startVelocityThresholdDegreePerSecond = 0.001;
    double maxFollowingErrorDegree = 2.0;
    int traceTimeoutMs = 100;
};

struct CdprVelocityControlStatus
{
    bool active = false;
    bool motionStarted = false;
    quint64 runId = 0;
    double elapsedS = 0.0;
    double controlDtMs = 0.0;
    quint64 timingSampleCount = 0;
    double averageControlDtMs = 0.0;
    double maximumControlDtMs = 0.0;
    double currentFullCycleMs = 0.0;
    double averageFullCycleMs = 0.0;
    double maximumFullCycleMs = 0.0;
    double averageTracePollMs = 0.0;
    double maximumTracePollMs = 0.0;
    double averageCalculationMs = 0.0;
    double maximumCalculationMs = 0.0;
    double averageApiTotalMs = 0.0;
    double maximumApiTotalMs = 0.0;
    int slowestAxis = -1;
    double maximumSingleAxisApiMs = 0.0;
    int latestTraceFramesRead = 0;
    quint64 executionOverrunCount = 0;
    quint64 schedulingOverrunCount = 0;
    quint64 estimatedMissedCycles = 0;
    quint16 activeAxisMask = 0;
    double maximumTrackingErrorDegree = 0.0;
    QString stateText = QStringLiteral("未运行");
};

enum class CdprOfflinePvtRunState : quint8
{
    Idle = 0,
    Ready,
    Running,
    Completed,
    Stopped,
    Fault
};

struct CdprOfflinePvtStatus
{
    CdprOfflinePvtRunState state = CdprOfflinePvtRunState::Idle;
    bool active = false;
    int currentIndex = 0;
    int totalPointCount = 0;
    double elapsedS = 0.0;
    double durationS = 0.0;
    QString stateText = QStringLiteral("未生成轨迹");
};

Q_DECLARE_METATYPE(CdprOfflinePvtRequest)
Q_DECLARE_METATYPE(CdprOfflinePvtPlan)
Q_DECLARE_METATYPE(CdprOfflinePvtStatus)
Q_DECLARE_METATYPE(CdprVelocityControlConfig)
Q_DECLARE_METATYPE(CdprVelocityControlStatus)

#endif // CDPRCONTROLTYPES_H
