#ifndef FORCEINTERACTIONTENSIONSHADOWWORKER_H
#define FORCEINTERACTIONTENSIONSHADOWWORKER_H

#include "forceinteractionruntimecontrol.h"
#include "forceinteractiontensionshadow.h"
#include "forwardkinematicssolver.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

// 2026-10-05: M2 online shadow executor.  It owns no hardware and has one
// bounded request/result slot, so a slow FK/allocation pass can never build a
// queue or delay the existing 5 ms online-velocity controller.
class ForceInteractionTensionShadowWorker
{
public:
    struct Request {
        quint64 epoch = 0;
        quint64 sourceTraceSequence = 0;
        qint64 sourceTraceUs = 0;
        qint64 submittedUs = 0;
        qint64 deadlineUs = 0;
        ForceInteractionPlatformState desired;
        OnlineVelocityAxisArray safetyRelativePosition{};
        OnlineVelocityAxisArray actualStartSafetyRelativePosition{};
    };

    struct Result {
        quint64 epoch = 0;
        quint64 sourceTraceSequence = 0;
        qint64 sourceTraceUs = 0;
        qint64 startedUs = 0;
        qint64 finishedUs = 0;
        bool valid = false;
        bool expired = false;
        QString errorMessage;
        ForwardKinematicsSolver::Result forwardKinematics;
        ForceInteractionTensionShadowResult shadow;
    };

    ForceInteractionTensionShadowWorker();
    ~ForceInteractionTensionShadowWorker();
    ForceInteractionTensionShadowWorker(
            const ForceInteractionTensionShadowWorker&) = delete;
    ForceInteractionTensionShadowWorker& operator=(
            const ForceInteractionTensionShadowWorker&) = delete;

    bool configure(const ForceInteractionRuntimeConfig& runtimeConfig,
                   QString* errorMessage = nullptr);
    void resetSession();
    bool submit(const Request& request);
    std::shared_ptr<const Result> take();
    quint64 epoch() const;

private:
    struct FrozenConfig {
        quint64 epoch = 0;
        ForceInteractionTensionShadowConfig shadow;
        CompensatedCableKinematics::Configuration kinematics;
        PhysicalWorkspaceBoundaryConfig physicalWorkspace;
        std::vector<double> initialPoseMmRad;
        std::vector<double> referenceCableLengthMm;
        OnlineVelocityAxisArray motorUnitPerRadian{};
    };

    static qint64 nowUs();
    void run();

    mutable std::mutex mutex_;
    std::condition_variable wake_;
    FrozenConfig config_;
    std::shared_ptr<const Request> request_;
    std::shared_ptr<const Result> result_;
    bool configured_ = false;
    bool busy_ = false;
    bool stopping_ = false;
    quint64 epoch_ = 0;
    std::thread thread_;
};

#endif // FORCEINTERACTIONTENSIONSHADOWWORKER_H
