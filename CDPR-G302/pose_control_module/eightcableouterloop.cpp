#include "eightcableouterloop.h"
#include <chrono>
#include <exception>

qint64 EightCableOuterLoop::nowUs(){
    return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
}
EightCableOuterLoop::EightCableOuterLoop():thread_([this]{run();}){}
EightCableOuterLoop::~EightCableOuterLoop(){
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;request_.reset();}
    wake_.notify_one();
    if(thread_.joinable()){thread_.join();}
}
bool EightCableOuterLoop::submit(std::shared_ptr<const Request> request){
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock() || stopping_ || busy_ || !request){return false;}
    busy_=true;request_=std::move(request);wake_.notify_one();return true;
}
std::shared_ptr<const EightCableOuterLoop::Result> EightCableOuterLoop::take(){
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock() || !result_){return {};}
    auto result=std::move(result_);busy_=false;return result;
}
void EightCableOuterLoop::run(){
    for(;;){
        std::shared_ptr<const Request> request;
        {std::unique_lock<std::mutex> lock(mutex_);
         wake_.wait(lock,[this]{return stopping_ || bool(request_);});
         if(stopping_){return;}request=std::move(request_);}
        auto result=std::make_shared<Result>();result->request=request;
        result->startedUs=nowUs();result->controller=request->controller;
        try{
            const auto dynamics=TaskSpaceTorqueController::buildEffectiveDynamics(request->model,request->feedback.velocity);
            if(!dynamics.valid){result->error=dynamics.message;}
            else{
                result->control=result->controller.update(request->reference,request->feedback,
                        dynamics.dynamics,request->dtSec,request->controllerConfig);
                if(!result->control.valid){result->error=result->control.message;}
                else{
                    auto allocation=request->allocation;
                    allocation.generalizedControl=result->control.generalizedControl;
                    const qint64 allocationStart=nowUs();
                    RedundantTorqueAllocator solver;
                    result->allocation=solver.solve(allocation,request->allocationConfig);
                    result->allocation.solveDurationUs=nowUs()-allocationStart;
                    result->valid=result->allocation.valid;
                    result->error=result->allocation.message;
                    // This state belongs to a published target, not to a motor batch.
                    result->controller.commit(result->valid,result->allocation.wrenchLimited);
                }
            }
        }catch(const std::exception& error){result->error=error.what();}
         catch(...){result->error="outer calculation raised an unknown exception";}
        result->finishedUs=nowUs();
        result->expired=result->finishedUs>request->deadlineUs || result->finishedUs-result->startedUs>request->computeBudgetUs;
        {std::lock_guard<std::mutex> lock(mutex_);
         if(stopping_){return;}result_=std::move(result);}
    }
}

