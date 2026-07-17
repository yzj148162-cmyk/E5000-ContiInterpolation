#ifndef KALMANHANDLER_H
#define KALMANHANDLER_H

#include <vector>

class OneDimKalmanHandler
{
public:
    OneDimKalmanHandler();
    OneDimKalmanHandler(int size);
    ~OneDimKalmanHandler();

    void setCov(std::vector<double> _P, std::vector<double> _Q, std::vector<double> _R);// 若不设置，则采用默认参数
    std::vector<double> update(std::vector<double> u, std::vector<double> z);// u：控制向量，即外界控制量  z：测量向量，即传感器数据分布的均值
private:
    int size = -1;
    std::vector<double> x,F,B,P,Q,R,K,H;
    bool isFirst = true;
};

#endif // KALMANHANDLER_H
