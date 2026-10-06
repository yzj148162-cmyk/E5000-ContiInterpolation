#ifndef FORCEINTERACTIONTENSIONSHADOWWORKER_H
#define FORCEINTERACTIONTENSIONSHADOWWORKER_H

#include "forceinteractionruntimecontrol.h"
#include "forceinteractiontensionshadow.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

// 2026-10-06: M2 online shadow executor. It owns no hardware and consumes an
// immutable Nokov pose snapshot. Encoder-to-cable-length FK is deliberately
// outside M2/M3+, so task-space feedback has the same source as the reference
// online-tension controller.
class ForceInteractionTensionShadowWorker
{
public:
    // 2026-10-06: Separate a real worker overrun from mailbox lock contention
    // and from a completed result that has not yet been consumed.
    enum class SubmitResult {
        Accepted,
        WorkerBusy,
        ResultPending,
        LockContended,
        NotReady
    };

    struct Request {
        quint64 epoch = 0;
        quint64 sourceTraceSequence = 0;
        qint64 sourceTraceUs = 0;
        qint64 submittedUs = 0;
        qint64 deadlineUs = 0;
        quint64 mocapSequence = 0;
        int mocapSourceFrameSequence = -1;
        qint64 mocapReceivedUs = 0;
        ForceInteractionPlatformState desired;
        ForceInteractionPlatformState observed;
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
        quint64 mocapSequence = 0;
        int mocapSourceFrameSequence = -1;
        qint64 mocapReceivedUs = 0;
        qint64 mocapAgeUs = -1;
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
    SubmitResult submit(const Request& request);
    std::shared_ptr<const Result> take();
    quint64 epoch() const;

private:
    struct FrozenConfig {
        quint64 epoch = 0;
        ForceInteractionTensionShadowConfig shadow;
        qint64 mocapTimeoutUs = 30000;
    };

    static qint64 nowUs();
    void run();

    mutable std::mutex mutex_;
    std::condition_variable wake_;
    FrozenConfig config_;
    std::shared_ptr<const Request> request_;
    std::shared_ptr<const Result> result_;
    bool configured_ = false;
    bool executing_ = false;
    bool stopping_ = false;
    quint64 epoch_ = 0;
    std::thread thread_;
};

#endif // FORCEINTERACTIONTENSIONSHADOWWORKER_H
