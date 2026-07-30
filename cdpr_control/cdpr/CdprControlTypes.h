#ifndef CDPRCONTROLTYPES_H
#define CDPRCONTROLTYPES_H

#include <array>

#include <QtGlobal>

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
    CdprAxisCommandFrame8 axisCommand;
    CdprAxisFeedbackFrame8 axisFeedback;
    quint32 safetyFlags = 0;
    bool safetyLatched = false;
};

#endif // CDPRCONTROLTYPES_H
