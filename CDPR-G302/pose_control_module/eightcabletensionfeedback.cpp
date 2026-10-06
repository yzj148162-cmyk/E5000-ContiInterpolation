#include "eightcabletensionfeedback.h"
#include "forcepidtorquelimits.h"
#include "eightcabletimingprofile.h"
#include <algorithm>
#include <cmath>

bool EightCableTensionFeedback::validate(const Config& c, std::string* error)
{
    if(c.sharedExecutionReference && (!c.enabled || !c.independentTargetFeedforward)){
        if(error){*error="0525 shared execution reference requires enabled tension feedback and independent target feedforward";}
        return false;
    }
    if(!c.enabled){ return true; }
    const bool valid = c.kp.allFinite() && (c.kp.array()>0.0).all() &&
            c.ki.allFinite() && c.kd.allFinite() && c.ki.isZero(0) && c.kd.isZero(0) &&
            std::isfinite(c.deadbandRatio) && c.deadbandRatio>=0 && c.deadbandRatio<1 &&
            std::isfinite(c.torqueLimitNm) && c.torqueLimitNm>0 && c.torqueLimitNm<=34.5 &&
            std::isfinite(c.slewNmPerSec) && c.slewNmPerSec>0 &&
            EightCableTimingProfile::supportsInner(c.periodUs) && !c.parameterSource.empty();
    if(!valid && error){ *error="0525 tension feedback requires frozen Kp>0, Ki=Kd=0, valid deadband/slew, torque <=34.5 Nm and 5/6/8/10 ms period"; }
    return valid;
}

void EightCableTensionFeedback::initialize(const Vector8d& torque, const Vector8d& tension, qint64 sampleUs)
{
    pid_.resetAll();
    entryTorque_=torque; entryTension_=tension; continuousTorque_=torque;
    targetBaseAnchored_=false; targetBaseVersion_=0;
    appliedBase_=torque; targetBaseGoal_=torque; targetBaseRemainingSec_=0.0;
    lastSampleUs_=sampleUs;
    initialized_=torque.allFinite() && tension.allFinite() && sampleUs>0;
}

bool EightCableTensionFeedback::propose(const Config& c, const RedundantTorqueAllocator::Config& bounds,
        const Vector8d& radius, const Vector8d& target, const Vector8d& measured,
        const Vector8d& committedTorque, qint64 sampleUs, double quantum,
        Diagnostic& d, std::string& error, qint64 outerPeriodUs, quint64 targetVersion)
{
    d.evaluated=true; d.sampleUs=sampleUs; d.dtUs=sampleUs-lastSampleUs_;
    if(!validate(c,&error) || !c.enabled || !initialized_ || d.dtUs<=0 ||
            !target.allFinite() || !measured.allFinite() || !committedTorque.allFinite() ||
            !radius.allFinite() || (radius.array()<=0).any() ||
            !std::isfinite(quantum) || std::abs(quantum-0.0345)>1e-9){
        error="invalid initialized 0525 tension feedback input or Lite torque quantum"; return false;
    }
    const double dt=double(d.dtUs)*1e-6;
    if(dt<bounds.minimumDtSec || dt>bounds.maximumDtSec || !bounds.torqueSlewEnabled){
        error="0525 feedback time step or hardware slew configuration is invalid"; return false;
    }
    if(c.independentTargetFeedforward &&
            (!EightCableTimingProfile::lookup(c.periodUs,outerPeriodUs).valid() ||
             !bounds.hardwareDirection.allFinite() ||
             (bounds.hardwareDirection.array().abs()-1.0).abs().maxCoeff()>1e-12 ||
             !bounds.hardwareTorqueSlewRate.allFinite() ||
             (bounds.hardwareTorqueSlewRate.array()<=0.0).any() ||
             (targetBaseAnchored_ && targetVersion<targetBaseVersion_))){
        error="0525 target feedforward requires a frozen loop profile, signed hardware direction, positive hard slew and causal target version";
        return false;
    }
    if((target.array()<bounds.tensionMinimum.array()-bounds.feasibilityTolerance).any() ||
       (target.array()>bounds.tensionMaximum.array()+bounds.feasibilityTolerance).any() ||
       (measured.array()<bounds.tensionMinimum.array()).any() ||
       (measured.array()>bounds.tensionMaximum.array()).any() ||
       (bounds.tensionMaximum.array()>400.0).any()){
        error="0525 tension reference or raw measured load is outside Lite bounds"; return false;
    }
    d.entryTorqueNm=entryTorque_; d.entryTensionN=entryTension_;
    d.targetN=target; d.measuredN=measured; d.errorN=target-measured;
    d.independentTargetFeedforward=c.independentTargetFeedforward;
    d.feedforwardStepNm.setZero();
    for(int i=0;i<8;++i){
        d.baseNm[i]=entryTorque_[i]-bounds.hardwareDirection[i]*radius[i]*(target[i]-entryTension_[i]);
    }
    if(!d.baseNm.allFinite()){
        error="0525 nominal target torque is nonfinite"; return false;
    }
    if(c.independentTargetFeedforward && targetBaseAnchored_ && targetVersion==targetBaseVersion_ &&
            (d.baseNm-targetBaseGoal_).cwiseAbs().maxCoeff()>1e-10){
        error="0525 allocated target changed without a new version"; return false;
    }
    d.appliedBaseNm=d.baseNm;
    if(c.independentTargetFeedforward && targetVersion>0){
        if(!targetBaseAnchored_){
            // Version 0 is the handover's measured-tension placeholder.
            // First allocation changes coordinates, NOT the output: retain
            // the continuous torque and let the original slow correction act.
            appliedBase_=d.baseNm; targetBaseGoal_=d.baseNm;
            targetBaseRemainingSec_=0.0; targetBaseAnchored_=true;
        }else if(targetVersion!=targetBaseVersion_ &&
                 (d.baseNm-targetBaseGoal_).cwiseAbs().maxCoeff()>1e-12){
            targetBaseGoal_=d.baseNm;
            targetBaseRemainingSec_=double(outerPeriodUs)*1e-6;
            // Common progress preserves nominal eight-axis increment proportions;
            // it does not assert that the physical wrench is preserved.
            for(int i=0;i<8;++i){
                const double rate=std::min(targetBaseSlewLimitNmPerSec,bounds.hardwareTorqueSlewRate[i]);
                targetBaseRemainingSec_=std::max(targetBaseRemainingSec_,
                        std::abs(targetBaseGoal_[i]-appliedBase_[i])/rate);
            }
        }
        targetBaseVersion_=targetVersion;
        if(targetBaseRemainingSec_>0.0){
            // No larger feedforward step to catch up a missed slot.
            const double advance=std::min(dt,double(c.periodUs)*1e-6);
            const double fraction=std::min(1.0,advance/targetBaseRemainingSec_);
            d.feedforwardStepNm=(targetBaseGoal_-appliedBase_)*fraction;
            appliedBase_+=d.feedforwardStepNm;
            targetBaseRemainingSec_=std::max(0.0,targetBaseRemainingSec_-advance);
        }
        d.appliedBaseNm=appliedBase_;
    }
    d.targetBaseAnchored=targetBaseAnchored_;
    d.targetBaseRemainingUs=qint64(std::llround(targetBaseRemainingSec_*1e6));
    // Derive the execution reference from the NOMINAL ramp only. Feedback,
    // command quantization and hard clipping must never redefine the target.
    // At the first real allocation and after the ramp, use the exact target.
    d.executionTargetN=target;
    if(c.independentTargetFeedforward && targetBaseAnchored_ && targetBaseRemainingSec_>0.0){
        for(int i=0;i<8;++i){
            d.executionTargetN[i]=entryTension_[i]+(entryTorque_[i]-d.appliedBaseNm[i])/
                    (bounds.hardwareDirection[i]*radius[i]);
        }
    }
    if(!d.executionTargetN.allFinite() ||
       (d.executionTargetN.array()<bounds.tensionMinimum.array()-bounds.feasibilityTolerance).any() ||
       (d.executionTargetN.array()>bounds.tensionMaximum.array()+bounds.feasibilityTolerance).any()){
        error="0525 nominal ramp execution reference is outside Lite bounds"; return false;
    }
    d.sharedExecutionReference=c.sharedExecutionReference;
    d.feedbackReferenceN=c.sharedExecutionReference ? d.executionTargetN : target;
    d.feedbackErrorN=d.feedbackReferenceN-measured;
    const auto values=[](const Vector8d& v){return std::vector<double>(v.data(),v.data()+8);};
    pid_.updatePara(values(c.kp),values(c.ki),values(c.kd),dt*1000.0);
    pid_.updateTustinPara(std::vector<double>(8,0),std::vector<double>(8,0),std::vector<double>(8,0),
                         std::vector<double>(8,-c.torqueLimitNm),std::vector<double>(8,c.torqueLimitNm));
    const auto output=pid_.updateWithRelativeDeadband(values(measured),values(d.feedbackReferenceN),c.deadbandRatio,
                                                     std::vector<int>(8,1));
    if(output.size()!=8){error="0525 core rejected channel dimensions";return false;}
    for(int i=0;i<8;++i){
        // allocator direction is opposite the original 0525 tightening sign.
        const double tightening=-bounds.hardwareDirection[i];
        d.pNm[i]=pid_.debugPTerm()[i];
        d.correctionNm[i]=tightening*output[i]; // Already Nm. Never multiply by radius again.
        d.pidClipped[i]=std::abs(d.pNm[i])>c.torqueLimitNm;
        d.requestNm[i]=d.appliedBaseNm[i]+d.correctionNm[i];
        const double lo=std::max(bounds.hardwareTorqueMinimum[i],-c.torqueLimitNm);
        const double hi=std::min(bounds.hardwareTorqueMaximum[i], c.torqueLimitNm);
        if(lo>hi || !std::isfinite(d.requestNm[i])){error="0525 torque bounds do not intersect";return false;}
        const double bounded=forcePidTorqueClamp(d.requestNm[i],lo,hi);
        // Move the prior output with the base, then apply the unchanged 0525
        // slew to feedback. Zero base step is the old constant-target slew.
        const double translated=continuousTorque_[i]+d.feedforwardStepNm[i];
        double signal=forcePidTorqueSlew(bounded,translated,c.slewNmPerSec,dt);
        d.signalSlewLimited[i]=std::abs(signal-bounded)>1e-12;
        d.feedbackStepNm[i]=signal-translated;
        const double safeSignal=forcePidTorqueClamp(signal,lo,hi);
        const bool totalClipped=safeSignal!=signal;
        signal=safeSignal;
        const double step=bounds.hardwareTorqueSlewRate[i]*dt;
        const double rawLo=std::ceil(std::max(lo,committedTorque[i]-step)/quantum-1e-10);
        const double rawHi=std::floor(std::min(hi,committedTorque[i]+step)/quantum+1e-10);
        if(rawLo>rawHi){error="no raw command satisfies final torque/slew bounds";return false;}
        const double rounded=std::round(signal/quantum);
        const double raw=forcePidTorqueClamp(rounded,rawLo,rawHi);
        d.commandNm[i]=raw*quantum;
        d.hardwareLimited[i]=std::abs(bounded-d.requestNm[i])>1e-12 || totalClipped || raw!=rounded;
        // Keep fractional slew progress when raw is unchanged; only a HARD
        // limiter feeds back its realizable command, avoiding virtual runaway.
        if(totalClipped || raw!=rounded){ signal=d.commandNm[i]; }
        continuousTorque_[i]=signal; d.continuousNm[i]=signal;
        d.feedbackStateNm[i]=signal-d.appliedBaseNm[i];
    }
    lastSampleUs_=sampleUs; d.valid=true;
    return true;
}

