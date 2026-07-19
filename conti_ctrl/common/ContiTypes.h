#ifndef CONTITYPES_H
#define CONTITYPES_H

#include <array>

#include <QString>
#include <QMetaType>
#include <QtGlobal>
#include <QVector>

namespace MotorUnit
{
// 编码器的物理分辨率固定为 500.622 pulse/deg。板卡 unit 的定义可在测试前选择，
// 但 Trace 原始脉冲与物理角度之间始终使用该常量换算。
constexpr double kPhysicalPulsesPerDegree = 500.622;
constexpr int kPulsesPerRevolution = 180224;

inline double pulsesPerCardUnit(double degreesPerCardUnit)
{
    return kPhysicalPulsesPerDegree * degreesPerCardUnit;
}

inline double degreesToCardUnits(double degrees, double degreesPerCardUnit)
{
    return degrees / degreesPerCardUnit;
}

inline double cardUnitsToDegrees(double cardUnits, double degreesPerCardUnit)
{
    return cardUnits * degreesPerCardUnit;
}
}

enum class TestStage : quint8
{
    SingleActiveAxis = 0, // 两轴坐标系：第一轴运动，第二轴保持当前位置
    DualAxis = 1
};

// 五次轨迹用于正式时间历程测试；等间距轨迹只用于隔离“小线段连接/前瞻”问题。
enum class TrajectoryPointMode : quint8
{
    QuinticTimeLaw = 0,
    UniformDistance = 1
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
    TrajectoryPointMode trajectoryPointMode = TrajectoryPointMode::QuinticTimeLaw;

    // 上层规划与界面始终使用角度；仅在雷赛 unit API 边界按此值换算。
    // 支持 1、0.1、0.01 deg/unit，对应 500.622、50.0622、5.00622 pulse/unit。
    double degreesPerCardUnit = 1.0;

    double activeDeltaUnit = 1.0;
    double holdDeltaUnit = 1.0;
    double durationS = 2.0;
    // EtherCAT 周期由 nmc_set_cycletime 在初始化阶段设置。手册支持
    // 250/500/1000/2000 us；Trace 固定每个总线周期采样一次。
    int busCycleUs = 1000;
    int traceCycle = 1;
    int producerPeriodMs = 10;

    double maxVectorVelocity = 10.0;
    double accelerationTimeS = 0.05;
    double decelerationTimeS = 0.05;
    double sCurveTimeS = 0.02;
    // speedRatio 是即将下发给控制卡的当前倍率；时间同步运行中由控制器更新。
    double speedRatio = 1.0;
    bool lookaheadEnabled = true;
    double pathErrorUnit = 0.0;
    // 对照模式：启动前将整条有效轨迹一次性压入卡侧，不进行运行中产点/补段。
    bool preloadAllTrajectoryToCard = false;

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

// 上层规划线程与独占硬件线程之间传递的已编号轨迹段。
// mark 在进入硬件线程前固定，便于上层将时间历程与卡侧执行序号对应起来。
struct ContiFeedItem
{
    ContiPoint point;
    long mark = 0;
};

// 独占硬件线程中的补段调度快照。上层只读取该快照，不参与定时补段。
struct ContiFeedStatus
{
    bool active = false;
    bool failed = false;
    // start_list 后先保留实时新增点，待板卡真正输出首段速度后再开放流式写入。
    bool motionStarted = false;
    bool runtimeFeedReleased = false;
    int runtimeFeedBatchPointCount = 0;
    QString errorText;
    long currentMark = 0;
    long lastPushedMark = 0;
    long remainSpace = 0;
    short runState = 4;
    int queuedPointCount = 0;
    double currentPlanTimeS = 0.0;
    double lastPushedPlanTimeS = 0.0;
    qint64 lastServiceGapUs = 0;
    qint64 maxServiceGapUs = 0;
    int lastRuntimePushBatchCount = 0;
    int maxRuntimePushBatchCount = 0;
    quint64 runtimePushedPointCount = 0;
    long lastPushSpaceBefore = 0;
    long lastPushSpaceAfter = 0;
};

// 控制卡实际生效的连续插补规划参数。所有量在 SDK 边界转换回角度制。
struct ContiCardReadback
{
    bool lookaheadEnabled = false;
    long lookaheadSegments = 0;
    double pathErrorDegree = 0.0;
    double lookaheadAccelerationDegreePerSecond2 = 0.0;
    double minVectorVelocityDegreePerSecond = 0.0;
    double maxVectorVelocityDegreePerSecond = 0.0;
    double accelerationTimeS = 0.0;
    double decelerationTimeS = 0.0;
    double stopVectorVelocityDegreePerSecond = 0.0;
    double sCurveTimeS = 0.0;
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
    int busCycleUs = 0;
    int traceCycle = 0;
    int producerPeriodMs = 0;
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
