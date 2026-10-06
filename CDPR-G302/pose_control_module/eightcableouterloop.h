#ifndef EIGHTCABLEOUTERLOOP_H
#define EIGHTCABLEOUTERLOOP_H
#include "taskspacetorquecontrol.h"
#include "redundanttorqueallocator.h"
#include <QtGlobal>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

// Calculation only. No HardwareInterface, SDK or continuous Trace acquisition.
// A single bounded job/result slot; a stale result never writes coordinator state.
class EightCableOuterLoop
{
public:
    struct Request {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        quint64 epoch=0, tick=0, sourceSequence=0;
        qint64 dueUs=0, sourceUs=0, mocapSourceUs=0, deadlineUs=0;
        qint64 computeBudgetUs=0;
        double dtSec=0;
        // Sensor values frozen with this outer observation, never a later inner frame.
        TaskSpaceTorqueController::Vector8d measuredTensionN=TaskSpaceTorqueController::Vector8d::Zero();
        bool measuredTensionValid=false;
        qint64 tensionSourceUs=0;
        quint32 tensionSourceSequence=0;
        bool tensionSourceSequenceValid=false;
        TaskSpaceTorqueController controller;
        TaskSpaceTorqueController::Config controllerConfig;
        TaskSpaceTorqueController::ModelTerms model;
        TaskSpaceTorqueController::Reference reference;
        TaskSpaceTorqueController::Feedback feedback;
        RedundantTorqueAllocator::Request allocation;
        RedundantTorqueAllocator::Config allocationConfig;
    };
    struct Result {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::shared_ptr<const Request> request;
        bool valid=false, expired=false;
        std::string error;
        qint64 startedUs=0, finishedUs=0;
        TaskSpaceTorqueController controller;
        TaskSpaceTorqueController::Step control;
        RedundantTorqueAllocator::Result allocation;
    };
    EightCableOuterLoop();
    ~EightCableOuterLoop();
    EightCableOuterLoop(const EightCableOuterLoop&)=delete;
    EightCableOuterLoop& operator=(const EightCableOuterLoop&)=delete;
    bool submit(std::shared_ptr<const Request> request);
    std::shared_ptr<const Result> take();
    static qint64 nowUs();
private:
    void run();
    std::mutex mutex_;
    std::condition_variable wake_;
    std::shared_ptr<const Request> request_;
    std::shared_ptr<const Result> result_;
    bool busy_=false, stopping_=false;
    std::thread thread_;
};
#endif

