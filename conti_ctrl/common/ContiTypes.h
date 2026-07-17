#ifndef CONTITYPES_H
#define CONTITYPES_H

#include <array>

#include <QString>
#include <QMetaType>
#include <QtGlobal>
#include <QVector>

namespace MotorUnit
{
constexpr double kPulsesPerDegree = 500.622;
constexpr int kPulsesPerRevolution = 180224;
}

enum class TestStage : quint8
{
    SingleActiveAxis = 0, // 两轴坐标系：第一轴运动，第二轴保持当前位置
    DualAxis = 1
};

// 在线倍率请求的执行结果。5049 表示控制卡首段尚未规划完成，属于可重试状态。
enum class ContiSpeedRatioResult : quint8
{
    Applied,
    NotReady,
    Failed
};

struct ContiTestConfig
{
    quint16 cardNo = 0;
    quint16 crdNo = 0;
    quint16 activeAxis = 0;
    quint16 holdAxis = 1;
    TestStage stage = TestStage::SingleActiveAxis;

    double activeDeltaUnit = 1.0;
    double holdDeltaUnit = 1.0;
    double durationS = 2.0;
    int producerPeriodMs = 10;

    double maxVectorVelocity = 10.0;
    double accelerationTimeS = 0.05;
    double decelerationTimeS = 0.05;
    double sCurveTimeS = 0.02;
    // speedRatio 是即将下发给控制卡的当前倍率；时间同步运行中由控制器更新。
    double speedRatio = 1.0;
    bool lookaheadEnabled = true;
    double pathErrorUnit = 0.0;

    bool timeSyncEnabled = true;
    int startupPreloadMs = 200;
    int targetBufferMs = 200;
    int lowBufferMs = 100;
    int criticalBufferMs = 50;
    int executionDelayMs = 250;
    int ratioUpdatePeriodMs = 10;
    int ratioApiMinIntervalMs = 100;
    int ratioSafetyApiIntervalMs = 50;
    double ratioMin = 0.20;
    double ratioMax = 1.00;
    double phaseGainPerSecond = 0.80;
    int phaseDeadbandMs = 20;
    double bufferGain = 0.10;
    double ratioDeadband = 0.01;
    double ratioMaxStep = 0.02;
    int markOffset = 0;
};

struct ContiPoint
{
    double timeS = 0.0;
    std::array<double, 2> targetUnit {};
};

// 第二页单电机测试使用的相对点位运动参数，单位与界面一致，均为角度制。
struct SingleAxisJogConfig
{
    quint16 cardNo = 0;
    quint16 axis = 0;
    double targetPositionUnit = 0.0;
    double maxVelocityUnitPerSecond = 10.0;
};

// 一条轴反馈同时保留控制卡指令位置与编码器位置，二者的差值用于观察跟随情况。
struct AxisFeedback
{
    quint16 axis = 0;
    bool valid = false;
    quint16 stateMachine = 0;
    quint16 axisErrorCode = 0;
    double commandPositionUnit = 0.0;
    double encoderPositionUnit = 0.0;
    bool traceSampleValid = false;
    QString errorText;
};

// 阶段 A 只记录当前 Trace 中的两个测试轴。位置保存为原始脉冲，时间由
// traceSequence 和 tracePeriodUs 重建；不以主机收到批量数据的时刻替代采样时刻。
struct TraceTelemetryFrame
{
    quint64 traceSequence = 0;
    quint64 traceTimeUs = 0;
    quint16 axes[2] {0, 0};
    qint32 commandPulse[2] {0, 0};
    qint32 actualPulse[2] {0, 0};
    quint16 axisState[2] {0, 0};
    quint16 axisError[2] {0, 0};
    quint8 axisCount = 0;
    quint8 validAxisMask = 0;
    qint32 currentMark = -1;
    qint32 pushedMark = -1;
};

struct TelemetryRecorderStatus
{
    bool recording = false;
    quint64 queuedFrames = 0;
    quint64 writtenFrames = 0;
    quint64 droppedFrames = 0;
    QString outputDirectory;
    QString errorText;
};

struct ContiStatus
{
    bool boardInitialized = false;
    int detectedBoardCount = 0;
    quint16 cardNo = 0;
    bool running = false;
    int runState = 4;
    long currentMark = 0;
    long pushedMark = 0;
    long remainSpace = 0;
    int hostQueueSize = 0;
    double currentPlanTimeS = 0.0;
    double expectedPlanTimeS = 0.0;
    double phaseErrorMs = 0.0;
    double bufferTimeMs = 0.0;
    double ratioRef = 0.0;
    double ratioPhase = 0.0;
    double ratioBuffer = 0.0;
    double ratioCommand = 0.0;
    double ratioApplied = 0.0;
    qint64 ratioLastApiAgoMs = -1;
    // 连续插补页低频对比图使用的理论轨迹快照。实际位置仍直接来自同一时刻附近的 Trace 最新帧。
    bool trajectoryComparisonActive = false;
    quint16 trajectoryActiveAxis = 0;
    quint64 trajectoryTraceStartTimeUs = 0;
    double trajectoryExpectedTimeS = 0.0;
    double trajectoryExpectedActiveUnit = 0.0;
    quint16 busErrorCode = 0;
    bool traceConfigured = false;
    bool traceEverRead = false;
    int traceFramesRead = 0;
    short traceLastApiResult = 0;
    QString traceStateText;
    QVector<AxisFeedback> axisFeedback;
    QVector<double> softwareZeroUnit;
    QVector<bool> softwareZeroValid;
    quint64 latestTraceSequence = 0;
    quint64 latestTraceTimeUs = 0;
    int traceSamplePeriodUs = 1000;
    TelemetryRecorderStatus recorder;
    QString stateText = QStringLiteral("未初始化");
};

Q_DECLARE_METATYPE(AxisFeedback)
Q_DECLARE_METATYPE(ContiStatus)
Q_DECLARE_METATYPE(ContiTestConfig)
Q_DECLARE_METATYPE(SingleAxisJogConfig)

#endif // CONTITYPES_H
