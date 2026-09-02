#include "forcepid0525.h"

ForcePid0525::ForcePid0525(){
}

ForcePid0525::ForcePid0525(std::vector<double> _kp,
                           std::vector<double> _ki,
                           std::vector<double> _kd,
                           double _Tms)
    : kp(_kp), ki(_ki), kd(_kd), T(_Tms / 1000.0){
}

ForcePid0525::~ForcePid0525(){

}

std::vector<double> ForcePid0525::update(std::vector<double> act,
                                         std::vector<double> exp,
                                         const std::vector<int>& freezeIntegral){
    std::vector<double> delta(act.size());// qDebug() << act.size() << exp.size() << kp.size() << ki.size() << kd.size();
    if(act.size() != exp.size() || act.size() !=  kp.size() || kp.size() != ki.size() || ki.size() != kd.size() || act.size() == 0)
        return {};

    size_t n = act.size();
    if(lastErr.empty())
        lastErr = std::vector<double>(n,0.0);
    if(tolErr.empty())
        tolErr = std::vector<double>(n,0.0);
    if(lastDer.empty())
        lastDer.assign(n, 0.0);
    if(der_tau.empty())
        der_tau.assign(n, 0.0);  // 0.01, driver filtering is already used in the original 0525 PID.
    if(output_min.empty())
        output_min.assign(n, -720.0);
    if(output_max.empty())
        output_max.assign(n, 720.0);

    if(integ_min.empty())
        integ_min.assign(n, -50000.0);
    if(integ_max.empty())
        integ_max.assign(n, 50000.0);

    debugErr.assign(n, 0.0);
    debugP.assign(n, 0.0);
    debugI.assign(n, 0.0);
    debugD.assign(n, 0.0);
    debugOutputNm.assign(n, 0.0);

    for(int i=0;i<n;++i){
        double curErr = exp[i]-act[i];

        double I_new = tolErr[i];
        const bool integralFrozen =
                i < static_cast<int>(freezeIntegral.size()) && freezeIntegral[i] != 0;
        if(!integralFrozen){
            I_new = tolErr[i] + 0.5 * T * (curErr + lastErr[i]);
            I_new = std::max(integ_min[i], std::min(integ_max[i], I_new));
            tolErr[i] = I_new;
        }

        double rawD = (curErr - lastErr[i]);
        double alpha = der_tau[i] / (der_tau[i] + T);
        double D_new = alpha * lastDer[i] + (1.0 - alpha) * rawD;
        lastDer[i] = D_new;

        const double pTerm = kp[i] * curErr;
        const double iTerm = kp[i] * ki[i] * tolErr[i];
        const double dTerm = kp[i] * kd[i] * D_new;
        double u = pTerm + iTerm + dTerm;

        u = std::max(output_min[i], std::min(output_max[i], u));
        delta[i] = u;
        debugErr[i] = curErr;
        debugP[i] = pTerm;
        debugI[i] = iTerm;
        debugD[i] = dTerm;
        debugOutputNm[i] = u;

        lastErr[i] = curErr;
    }

    return delta;
}

std::vector<double> ForcePid0525::updateWithRelativeDeadband(std::vector<double> act,
                                                             std::vector<double> exp,
                                                             double deadbandRatio,
                                                             const std::vector<int>& freezeIntegral){
    std::vector<double> delta = update(act, exp, freezeIntegral);
    if(delta.empty() || !std::isfinite(deadbandRatio) || deadbandRatio <= 0.0){
        return delta;
    }

    const size_t n = std::min({delta.size(), act.size(), exp.size()});
    for(size_t i=0; i<n; ++i){
        const double err = exp[i] - act[i];
        const double deadbandAbs = std::abs(exp[i]) * deadbandRatio;
        if(std::isfinite(err) &&
                std::isfinite(deadbandAbs) &&
                std::abs(err) < deadbandAbs){
            delta[i] = 0.0;
            resetChannel(i);
            if(i < debugErr.size()){
                debugErr[i] = err;
            }
            if(i < debugOutputNm.size()){
                debugOutputNm[i] = 0.0;
            }
            if(i < debugP.size()){
                debugP[i] = 0.0;
            }
            if(i < debugI.size()){
                debugI[i] = 0.0;
            }
            if(i < debugD.size()){
                debugD[i] = 0.0;
            }
        }
    }

    return delta;
}

void ForcePid0525::updatePara(std::vector<double> _kp,
                              std::vector<double> _ki,
                              std::vector<double> _kd,
                              double _Tms){// qDebug() << "TTTTTTTTTT" << _kp << _ki << _kd;
//    if(kp.size() != _kp.size() || ki.size() != _ki.size() || kd.size() != _kd.size())
//        return;
    kp = _kp;
    ki = _ki;
    kd = _kd;
    T = _Tms/1000.0;
}

void ForcePid0525::resetTolErr(){
    tolErr.resize(0);

    integrator.resize(0);
    debugErr.resize(0);
    debugP.resize(0);
    debugI.resize(0);
    debugD.resize(0);
    debugOutputNm.resize(0);
}

void ForcePid0525::resetIntegral(size_t index){
    if(index < tolErr.size()){
        tolErr[index] = 0.0;
    }
    if(index < integrator.size()){
        integrator[index] = 0.0;
    }
    if(index < debugI.size()){
        debugI[index] = 0.0;
    }
}

void ForcePid0525::resetChannel(size_t index){
    if(index < tolErr.size()){
        tolErr[index] = 0.0;
    }
    if(index < integrator.size()){
        integrator[index] = 0.0;
    }
    if(index < lastErr.size()){
        lastErr[index] = 0.0;
    }
    if(index < lastDer.size()){
        lastDer[index] = 0.0;
    }
    if(index < debugErr.size()){
        debugErr[index] = 0.0;
    }
    if(index < debugP.size()){
        debugP[index] = 0.0;
    }
    if(index < debugI.size()){
        debugI[index] = 0.0;
    }
    if(index < debugD.size()){
        debugD[index] = 0.0;
    }
    if(index < debugOutputNm.size()){
        debugOutputNm[index] = 0.0;
    }
}

void ForcePid0525::resetAll(){
    tolErr.resize(0);
    lastErr.resize(0);

    integrator.resize(0);
    lastDer.resize(0);
    debugErr.resize(0);
    debugP.resize(0);
    debugI.resize(0);
    debugD.resize(0);
    debugOutputNm.resize(0);
}

void ForcePid0525::updateTustinPara(std::vector<double> _der_tau,
                                    std::vector<double> _integ_min,
                                    std::vector<double> _integ_max,
                                    std::vector<double> _output_min,
                                    std::vector<double> _output_max){
    der_tau = _der_tau;
    integ_min = _integ_min;
    integ_max = _integ_max;
    if(!_output_min.empty())
        output_min = _output_min;
    if(!_output_max.empty())
        output_max = _output_max;
}
