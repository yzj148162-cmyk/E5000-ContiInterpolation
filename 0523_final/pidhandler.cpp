#include "pidhandler.h"

PIDHandler::PIDHandler(){
}

PIDHandler::PIDHandler(std::vector<double> _kp, std::vector<double> _ki, std::vector<double> _kd, double _Tms)
    : kp(_kp),ki(_ki),kd(_kd),T(_Tms/1000.0){
}

PIDHandler::~PIDHandler(){

}

std::vector<double> PIDHandler::update(std::vector<double> act, std::vector<double> exp){
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
        der_tau.assign(n, 0.0);  // 0.01，由于变速器自带滤波所以这里先不滤波
    if(output_min.empty())
        output_min.assign(n, -720.0);  // 默认输出下限
    if(output_max.empty())
        output_max.assign(n, 720.0);   // 默认输出上限

    // 当前使用±10000.0的保守值，因为output_min/(kp*ki)是向量运算需要每次循环
    if(integ_min.empty())
        integ_min.assign(n, -50000.0);  // 默认积分下限（保守值）
    if(integ_max.empty())
        integ_max.assign(n, 50000.0);   // 默认积分上限（保守值）
    
    for(int i=0;i<n;++i){
        double curErr = exp[i]-act[i];
        
        // 1. 积分抗饱和
        // double I_new = tolErr[i] + curErr;// 前向欧拉法
        double I_new = tolErr[i] + 0.5 * T * (curErr + lastErr[i]);// 梯形法

        I_new = std::max(integ_min[i], std::min(integ_max[i], I_new));
        tolErr[i] = I_new;
        
        // 2. 微分滤波（一阶低通）
        double rawD = (curErr - lastErr[i]);
        double alpha = der_tau[i] / (der_tau[i] + T);
        double D_new = alpha * lastDer[i] + (1.0 - alpha) * rawD;
        lastDer[i] = D_new;
        
        // 3. PID计算（使用滤波后的微分值）
        double u = kp[i]*(curErr + ki[i]*tolErr[i] + kd[i]*D_new);
        
        // 4. 输出限幅
        u = std::max(output_min[i], std::min(output_max[i], u));
        delta[i] = u;
        
        lastErr[i] = curErr;
    }

    return delta;
}

std::vector<double> PIDHandler::updateWithRelativeDeadband(std::vector<double> act, std::vector<double> exp, double deadbandRatio){
    std::vector<double> delta = update(act, exp);
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
        }
    }

    return delta;
}

void PIDHandler::updatePara(std::vector<double> _kp, std::vector<double> _ki, std::vector<double> _kd, double _Tms){// qDebug() << "TTTTTTTTTT" << _kp << _ki << _kd;
//    if(kp.size() != _kp.size() || ki.size() != _ki.size() || kd.size() != _kd.size())
//        return;
    kp = _kp;
    ki = _ki;
    kd = _kd;
    T = _Tms/1000.0;
}

void PIDHandler::resetTolErr(){
    tolErr.resize(0);

    integrator.resize(0);
}

void PIDHandler::resetChannel(size_t index){
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
}

void PIDHandler::resetAll(){
    tolErr.resize(0);
    lastErr.resize(0);

    integrator.resize(0);
    lastDer.resize(0);
}

void PIDHandler::updateTustinPara(std::vector<double> _der_tau, std::vector<double> _integ_min, std::vector<double> _integ_max, std::vector<double> _output_min, std::vector<double> _output_max){
    der_tau = _der_tau;
    integ_min = _integ_min;
    integ_max = _integ_max;
    if(!_output_min.empty())
        output_min = _output_min;
    if(!_output_max.empty())
        output_max = _output_max;
}
