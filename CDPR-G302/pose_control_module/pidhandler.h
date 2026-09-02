#ifndef PIDHANDLER_H
#define PIDHANDLER_H

/*
 * 文件总览：
 * - PIDHandler 是多通道 PID 工具类，每个向量元素对应一个独立控制通道。
 * - 支持基础前向欧拉 PID、相对死区、Tustin/梯形积分、微分滤波和输出/积分限幅。
 */

#include <cmath>
#include <cstddef>
#include <vector>
#include <algorithm>
#include <cassert>

#include <QDebug>

class PIDHandler
{
public:   
    // 构造空 PID，后续可通过 updatePara 配置通道参数。
    PIDHandler();
    // kp,ki,kd的向量size决定该pid控制器需要控制多少个变量，向量元素决定各个变量的pid参数。在整个过程中，各个变量之间互相独立
    PIDHandler(std::vector<double> _kp, std::vector<double> _ki, std::vector<double> _kd, double _Tms);
    // 释放 PID 对象；成员均为值类型容器，无额外资源。
    ~PIDHandler();

    // 采用前向欧拉法离散化PID，输出为相对值，即每次零点都为上一次的结果
    std::vector<double> update(std::vector<double> act, std::vector<double> exp);
    // 带相对死区的 PID 更新，小误差时抑制输出以降低抖动。
    std::vector<double> updateWithRelativeDeadband(std::vector<double> act, std::vector<double> exp, double deadbandRatio);

    // 在运行过程中修改参数（不会重置其它成员变量）
    void updatePara(std::vector<double> _kp, std::vector<double> _ki, std::vector<double> _kd, double _Tms);
    // 配置 Tustin 积分、微分滤波以及积分/输出限幅参数。
    void updateTustinPara(std::vector<double> _der_tau, std::vector<double> _integ_min, std::vector<double> _integ_max, std::vector<double> _output_min = std::vector<double>(), std::vector<double> _output_max = std::vector<double>());

    // 重置累计误差
    void resetTolErr();
    // 重置指定通道的误差、积分和微分历史。
    void resetChannel(size_t index);
    // 重置全部（不含exp kp ki kd）
    void resetAll();
private:
    double T;// 控制周期(s)
    std::vector<double> kp,ki,kd;
    std::vector<double> lastErr,tolErr;

    // 梯形法中间量
    std::vector<double> integrator;   // I[k]

    // 上次的滤波后微分值
    std::vector<double> lastDer;
    // 微分滤波时间常数 tau (每通道)，若为空则默认为 0 (无滤波)。tau 在 0.01*T 到 5*T 之间可试。若测量噪声大，增大 tau（更平滑）；若需要更敏捷响应，减小 tau
    std::vector<double> der_tau;
    // 积分上下限（抗饱和），若为空将使用大范围
    std::vector<double> integ_min, integ_max;
    // 输出限幅范围
    std::vector<double> output_min, output_max;
};

#endif // PIDHANDLER_H
