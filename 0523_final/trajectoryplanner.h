#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <QString>

#include <vector>

class TrajectoryPlanner
{
public:
    using TrajectoryBlock = std::vector<std::vector<std::vector<double>>>;
    using MultiEndTrajectory = std::vector<std::vector<std::vector<std::vector<double>>>>;

    enum class TrajType {
    Quintic,
    Circular,
    SCurve,
    EightShape,
    Sine,
    Cubic,
    StepAccel
};

    // 五次多项式
    struct QuinticParam
    {
        std::vector<double> startPose;
        std::vector<double> startVel;
        std::vector<double> startAcc;
        std::vector<double> endPose;
        std::vector<double> endVel;
        std::vector<double> endAcc;
        double duration = 0.0;
        double stepTime = 0.0;
    };

    // 圆轨迹
    struct CircularParam
    {
        std::vector<double> startPose;
        std::vector<double> center; // [x,y,z]
        double radius;
        double omega;
        int direction;
        double stepTime;
    };

    // S曲线
    struct SCurveParam
    {
        std::vector<double> startPose, endPose;
        double vmax;
        double acc;
        double dec;
        double stepTime;
    };

    // 8字
    struct EightShapeParam
    {
        std::vector<double> startPose,normal;
        double R;
        double range;
        double duration;
        double stepTime;
        //还需要有运动频率和姿态变化幅度
    };

    // 正弦加速度
    struct SineParam
    {
        std::vector<double> startPose;
        std::vector<double> endPose;
        double A;
        double w;
        double phi;
        double duration;
        double stepTime;
    };

    //阶跃加速度
    struct StepAccelParam
    {
        std::vector<double> startPose;     // 初始位置
        std::vector<double> dir;    // 运动方向（单位向量）

        double a_before;            // 阶跃前加速度
        double a_after;             // 阶跃后加速度

        double t_step;              // 阶跃发生时刻
        double stepTime;            // 采样时间);
    };

    // 三次多项式
    struct CubicParam
    {
        std::vector<double> startPose;     // 初始位置
        std::vector<double> startVel;     // 初始速度
        std::vector<double> endPose;     // 初始位置
        std::vector<double> endVel;     // 初始速度
        double duration;
        double stepTime;
    };

    struct EndpointRequest {
        TrajType type;
        QuinticParam q_param;
        CircularParam cir_param;
        SCurveParam s_param;
        EightShapeParam e_param;
        SineParam sin_param;
        CubicParam cub_param;
        StepAccelParam stepAccel_param;
    };

    struct FileTrajectory {
        int endNum = 0;
        int pointNum = 0;
        double duration = 0.0;
        MultiEndTrajectory offlineTraj;
        std::vector<double> firstEndStartPose;
    };

    struct PointTrajectoryTransition {
        std::vector<std::vector<double>> positionTraj;
        std::vector<double> timeStamp;
        double resumeTime = 0.0;
        std::vector<double> referencePosition;
    };

    struct PointTrajectoryTransitionRequest {
        std::vector<std::vector<double>> sourcePositionTraj;
        std::vector<double> sourceTimeStamp;
        std::vector<double> currentPosition;
        double currentTrajectoryTime = 0.0;
        double transitionTime = 0.5;
        double sampleTime = 0.001;
    };

    static TrajectoryBlock quintic(const std::vector<double>& startPose,
                                   const std::vector<double>& startVel,
                                   const std::vector<double>& startAcc,
                                   const std::vector<double>& endPose,
                                   const std::vector<double>& endVel,
                                   const std::vector<double>& endAcc,
                                   double duration,
                                   double stepTime);

    static TrajectoryBlock circular(
        const std::vector<double>& startPose,     
        const std::vector<double>& center, // [x,y,z]
        const double radius,
        const double omega,
        const int direction,
        double stepTime);


    static TrajectoryBlock scurve(        
        const std::vector<double>& startPose,
        const std::vector<double>& endPose,
        const double vmax,
        const double acc,
        const double dec,
        double stepTime);

    static TrajectoryBlock eightShape(        
        const std::vector<double>& startPose,
        const std::vector<double>& normal,
        const double R,
        const double range,
        double duration,
        double stepTime);

    static TrajectoryBlock sineshape(        
        const std::vector<double>& startPose,
        const std::vector<double>& endPose,
        const double A,
        const double w,
        const double phi,
        double duration,
        double stepTime);


    static TrajectoryBlock stepAccel(        
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& dir,    // 运动方向（单位向量）

        const double a_before,            // 阶跃前加速度
        const double a_after,             // 阶跃后加速度

        const double t_step,              // 阶跃发生时刻
        double stepTime);            // 采样时间);

    static TrajectoryBlock cubicline(        
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& startVel,     // 初始速度
        const std::vector<double>& endPose,     // 初始位置
        const std::vector<double>& endVel,     // 初始速度
        double duration,            // 总时长
        double stepTime);            // 采样时间);


    static MultiEndTrajectory buildLineTrajectory(const std::vector<EndpointRequest>& requests, const TrajectoryPlanner::TrajType& type);
    static MultiEndTrajectory buildSingleEndOfflinePoseTrajectory(const std::vector<std::vector<double>>& poseRows);

    static std::vector<double> interpolatePointTrajectory(const std::vector<std::vector<double>>& positionTraj,
                                                          const std::vector<double>& timeStamp,
                                                          double targetTime);
    static std::vector<double> estimatePointTrajectoryVelocity(const std::vector<std::vector<double>>& positionTraj,
                                                               const std::vector<double>& timeStamp,
                                                               double targetTime);
    static bool buildStopTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                              PointTrajectoryTransition& out,
                                              QString& errorMessage);
                                              
    static bool buildResumeTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                                PointTrajectoryTransition& out,
                                                QString& errorMessage);

    static bool loadPoseFile(const QString& path,
                             int expectedEndNum,
                             FileTrajectory& out,
                             QString& errorMessage);

    static void resampleTrajectory(
    std::vector<std::vector<std::vector<std::vector<double>>>>& traj,
    double maxStep);
};

#endif // TRAJECTORYPLANNER_H
