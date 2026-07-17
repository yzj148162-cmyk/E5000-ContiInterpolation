#ifndef MOTIONCONFIG_H
#define MOTIONCONFIG_H

#include <QString>
#include <QtGlobal>

// 当前骨架只保留 PVT 和 CSP 两种模式。
enum class MotionMode {
    PVT = 0,
    CSP
};

// 上层结果向硬件线程交付时的三种典型形态。
// 这层抽象描述的是“结果形态本身”，不是某个运动模式的专属定义。
enum class PlanningOutputType {
    // 整条轨迹在启动前一次性准备完成，交给硬件线程整表下发。
    PrecomputedTable = 0,
    // 未来轨迹点按批次补入共享队列，由硬件线程逐点消费。
    QueuedSequence,
    // 每次只提供当前时刻或下一时刻控制量，供硬件线程直接读取。
    RealtimeCommand
};

// 规划线程的唤醒策略。
// 这层抽象描述的是“规划线程如何被驱动”，不是模式名字的别名。
enum class PlannerWakePolicy {
    EventDriven = 0,
    Periodic,
    PeriodicWithBuffer
};

inline QString motionModeToString(MotionMode mode)
{
    return mode == MotionMode::CSP ? QStringLiteral("CSP") : QStringLiteral("PVT");
}

inline PlanningOutputType planningOutputTypeForMode(MotionMode mode)
{
    return mode == MotionMode::CSP ? PlanningOutputType::QueuedSequence
                                   : PlanningOutputType::PrecomputedTable;
}

inline PlannerWakePolicy plannerWakePolicyForOutputType(PlanningOutputType outputType)
{
    switch (outputType) {
    case PlanningOutputType::PrecomputedTable:
        return PlannerWakePolicy::EventDriven;
    case PlanningOutputType::QueuedSequence:
        return PlannerWakePolicy::PeriodicWithBuffer;
    case PlanningOutputType::RealtimeCommand:
        return PlannerWakePolicy::Periodic;
    }

    return PlannerWakePolicy::EventDriven;
}

inline PlannerWakePolicy plannerWakePolicyForMode(MotionMode mode)
{
    return plannerWakePolicyForOutputType(planningOutputTypeForMode(mode));
}

// UI 下发到规划线程和硬件线程的一组运动参数。
struct MotionConfig
{
    MotionMode mode = MotionMode::PVT;
    quint16 axis = 0;
    double deltaDeg = 0.0;
    double durationS = 5.0;

    // 系统规划周期：
    // - CSP：先按该周期生成粗点，再细化到 1ms 供硬件线程逐点消费
    // - PVT(PVTS)：直接按该周期生成整表点列，交给控制卡内部完成段间恢复
    qint32 systemPlanningPeriodMs = 10;

    double minVelDeg = 0.0;
    double maxVelDeg = 60.0;
    double rawPerDeg = 500.622;

    quint16 ecatPort = 2;
    quint16 node = 1001;
    qint32 cspInterpPeriodMs = 1;
    qint32 cspInterpIndex = -3;
    quint16 cspRxpdoPosAddr = 1;
    quint16 cspTxpdoStatusAddr = 0;
    quint16 cspTxpdoModeAddr = 3;
    quint16 cspTxpdoPosAddr = 1;
};

#endif // MOTIONCONFIG_H
