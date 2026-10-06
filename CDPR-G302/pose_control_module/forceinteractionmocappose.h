#ifndef FORCEINTERACTIONMOCAPPOSE_H
#define FORCEINTERACTIONMOCAPPOSE_H

#include <QtGlobal>

#include <array>
#include <mutex>

// 2026-10-06: 六维力在线张力链专用的Nokov值快照。
// 该结构只传递已经由NokovPoseCalculator换算完成的平台位姿，不持有SDK帧、
// UI控件或动捕线程对象，避免M2/M3与采集实现及界面生命周期耦合。
struct ForceInteractionMocapPose
{
    bool valid = false;
    quint64 sequence = 0;
    int sourceFrameSequence = -1;
    quint64 connectionGeneration = 0;
    qint64 receivedMonotonicUs = 0;
    qint64 receivedWallClockMs = 0;
    std::array<double, 6> poseMmRad{};
};

class ForceInteractionMocapPoseStore
{
public:
    void publish(const ForceInteractionMocapPose& value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = value;
        latest_.sequence = ++sequence_;
    }

    void invalidate()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = {};
        latest_.sequence = ++sequence_;
    }

    ForceInteractionMocapPose latest() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

private:
    mutable std::mutex mutex_;
    ForceInteractionMocapPose latest_;
    quint64 sequence_ = 0;
};

#endif // FORCEINTERACTIONMOCAPPOSE_H
