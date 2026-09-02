#ifndef KALMANHANDLER_H
#define KALMANHANDLER_H

/*
 * 文件总览：
 * - OneDimKalmanHandler 是多通道一维 Kalman 滤波工具，用于对传感器测量值做简单状态估计。
 * - 每个通道共享同样的线性模型结构，但可配置独立的 P/Q/R 协方差参数。
 */

#include <vector>

class OneDimKalmanHandler
{
public:
    // 构造空滤波器，后续可用带 size 的构造函数初始化通道。
    OneDimKalmanHandler();
    // 按通道数量初始化一维 Kalman 滤波器数组。
    OneDimKalmanHandler(int size);
    // 析构滤波器；成员容器自动释放。
    ~OneDimKalmanHandler();

    // 设置每个通道的 P/Q/R 协方差；不调用时使用默认参数。
    void setCov(std::vector<double> _P, std::vector<double> _Q, std::vector<double> _R);// 若不设置，则采用默认参数
    // 输入控制量 u 和测量值 z，返回各通道滤波后的估计值。
    std::vector<double> update(std::vector<double> u, std::vector<double> z);// u：控制向量，即外界控制量  z：测量向量，即传感器数据分布的均值
private:
    int size = -1;
    std::vector<double> x,F,B,P,Q,R,K,H;
    bool isFirst = true;
};

#endif // KALMANHANDLER_H
