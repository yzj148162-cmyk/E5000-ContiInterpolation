#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <vector>
#include <QDebug>

class TrajectoryGenerator
{
public:
    enum TrajectoryType {
        QUINTIC_POLYNOMIAL,
        CIRCULAR,
        LINEAR_S_CURVE,
        EIGHT_SHAPE,
        SINE_ACCELERATION,
        CUBIC_POLYNOMIAL
    };

    TrajectoryGenerator();
    ~TrajectoryGenerator();

    // 设置轨迹类型
    void setTrajectoryType(TrajectoryType type);

    // 圆形轨迹参数
    void setCircularParams(double centerX, double centerY, double centerZ, double radius, double angularSpeed, bool clockwise);

    // 直线S曲线参数
    void setLinearSCurveParams(double maxVel, double acc, double dec);

    // 8字形参数
    void setEightShapeParams(double sizeX, double sizeY, double freq, double attAmp);

    // 正弦参数
    void setSineParams(double freq, double amp, double phase);

    // 设置轨迹
    void setTraj(const std::vector<std::vector<double>>& p0, 
                 const std::vector<std::vector<double>>& v0, 
                 const std::vector<std::vector<double>>& a0, 
                 const std::vector<std::vector<double>>& pt, 
                 const std::vector<std::vector<double>>& vt, 
                 const std::vector<std::vector<double>>& at, 
                 double runTime, double stepTime);

    // 生成轨迹
    // std::vector<std::vector<std::vector<std::vector<double>>>> generateTrajectory();

private:
    TrajectoryType trajectoryType;

    // 圆形轨迹参数
    double circCenterX, circCenterY, circCenterZ;
    double circRadius;
    double circAngularSpeed;
    bool circClockwise;

    // 直线S曲线参数
    double sMaxVel, sAcc, sDec;

    // 8字形参数
    double eightSizeX, eightSizeY, eightFreq, eightAttAmp;

    // 正弦参数
    double sineFreq, sineAmp, sinePhase;

    // 轨迹基本参数
    std::vector<std::vector<double>> p0, v0, a0, pt, vt, at;
    double runTime, stepTime;

    // 生成五次多项式轨迹
    std::vector<std::vector<std::vector<double>>> generateQuinticPolynomial(std::vector<double> p0, std::vector<double> v0, std::vector<double> a0,
                                                                            std::vector<double> pt, std::vector<double> vt, std::vector<double> at,
                                                                            double t, double dt);

    // 生成圆形轨迹
    std::vector<std::vector<std::vector<double>>> generateCircular(
        std::vector<double> center, // [x,y,z]
        double radius,
        double z,
        double omega, // 角速度
        double t,
        double dt,
        int direction// 1逆时针 -1顺时针
        );

    // 生成直线S曲线轨迹
    std::vector<std::vector<std::vector<double>>> generateLinearSCurve(
        std::vector<double> p0,
        std::vector<double> pt,
        double vmax,
        double acc,
        double dec,
        double dt);

    // 生成8字形轨迹
    std::vector<std::vector<std::vector<double>>> generateEightShape(
        double t_start,
        double t_end,
        double dt,
        std::vector<double> p_start,   // [x y z roll pitch yaw]
        double R,
        std::vector<double> vec        // 法向量
        );

    // 生成正弦变加速轨迹
    std::vector<std::vector<std::vector<double>>> generateSineAcceleration(
        std::vector<double> p0,
        std::vector<double> dir,
        double A, double w, double phi,
        double t, double dt);

    // 生成三次多项式轨迹
    std::vector<std::vector<std::vector<double>>> generateCubicPolynomial(
        std::vector<double> p0,
        std::vector<double> v0,
        std::vector<double> pt,
        std::vector<double> vt,
        double t,
        double dt);

    void resampleTrajectory(
        std::vector<std::vector<std::vector<std::vector<double>>>>& traj,
        double maxStep);

    
};

#endif // TRAJECTORYGENERATOR_H
