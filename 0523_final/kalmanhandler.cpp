#include "kalmanhandler.h"

OneDimKalmanHandler::OneDimKalmanHandler(){}

OneDimKalmanHandler::OneDimKalmanHandler(int _size)
    :size(_size){
    x.resize(size,0.0);// state, x(k) = a * x(k-1) + b * u(k)
    F.resize(size,1.0);// state transfer matrix
    B.resize(size,0.0);// control matrix
    H.resize(size,1.0);// observer matrix
    P.resize(size,0.0);// estimate cov
    Q.resize(size,0.002);// process cov
    R.resize(size,0.05);// observer cov
    K.resize(size,0.0);// kalman gain
}

OneDimKalmanHandler::~OneDimKalmanHandler(){
}

void OneDimKalmanHandler::setCov(std::vector<double> _P, std::vector<double> _Q, std::vector<double> _R){
    if(_P.size() != size || _Q.size() != size || _R.size() != size)
        return;
    P = _P;
    Q = _Q;
    R = _R;
}

std::vector<double> OneDimKalmanHandler::update(std::vector<double> u, std::vector<double> z){
    if(isFirst){
        for(int i=0;i<size;++i){
            x[i] = z[i];// x起点需要和原始数据的第一个数据匹配，不然x会从0开始，并花非常长的时间才能到起始值
        }
        isFirst = false;
    }

    for(int i=0;i<size;++i){
        /* predict, time update */
        x[i] = F[i] * x[i] + B[i] * u[i];
        P[i] = F[i] * P[i] * F[i] + Q[i];

        /* correct, measurement update */
        K[i] = P[i] * H[i] / (H[i] * P[i] * H[i] + R[i]);
        x[i] = x[i] + K[i] * (z[i] - H[i] * x[i]);
        P[i] = (1 - K[i] * H[i]) * P[i];
    }

    return x;
}
