/*
 * 文件总览：
 * - TrajectoryGenerator 是较早的轨迹生成类，保留了多种基础轨迹算法实现。
 * - 当前主流程更多使用 TrajectoryPlanner，本类可作为算法参考或兼容旧调用。
 * - 输出结构同样按位置、速度、加速度和时间组织。
 */

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

    // 生成五次多项式轨迹，满足起止位置、速度和加速度约束。
    std::vector<std::vector<std::vector<double>>> generateQuinticPolynomial(
        std::vector<double> p0,
        std::vector<double> v0,
        std::vector<double> a0,
        std::vector<double> pt,
        std::vector<double> vt,
        std::vector<double> at,
        double t,
        double dt);

    // 生成圆形轨迹
    // 根据圆心、半径、高度和角速度生成水平圆轨迹。
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
    // 生成直线 S 曲线轨迹，自动处理加速、匀速和减速段。
    std::vector<std::vector<std::vector<double>>> generateLinearSCurve(
        std::vector<double> p0,
        std::vector<double> pt,
        double vmax,
        double acc,
        double dec,
        double dt);

    // 生成8字形轨迹
    // 生成 8 字形空间轨迹，并按法向量确定运动平面。
    std::vector<std::vector<std::vector<double>>> generateEightShape(
        double t_start,
        double t_end,
        double dt,
        std::vector<double> p_start,   // [x y z roll pitch yaw]
        double R,
        std::vector<double> vec        // 法向量
        );

    // 生成正弦变加速轨迹
    // 按正弦加速度模型沿指定方向积分生成轨迹。
    std::vector<std::vector<std::vector<double>>> generateSineAcceleration(
        std::vector<double> p0,
        std::vector<double> dir,
        double A, double w, double phi,
        double t, double dt);

    // 生成三次多项式轨迹
    // 生成三次多项式轨迹，满足起止位置和起止速度约束。
    std::vector<std::vector<std::vector<double>>> generateCubicPolynomial(
        std::vector<double> p0,
        std::vector<double> v0,
        std::vector<double> pt,
        std::vector<double> vt,
        double t,
        double dt);

    // 对轨迹按最大时间步长重采样，避免下游控制周期过粗。
    void resampleTrajectory(
        std::vector<std::vector<std::vector<std::vector<double>>>>& traj,
        double maxStep);

    
};

#endif // TRAJECTORYGENERATOR_H
