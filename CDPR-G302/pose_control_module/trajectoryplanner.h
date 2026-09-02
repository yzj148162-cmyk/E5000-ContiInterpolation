/*
 * 文件总览：
 * - TrajectoryPlanner 是静态轨迹生成与文件轨迹处理工具，输出统一的多末端轨迹数据结构。
 * - 支持五次多项式、圆轨迹、S 曲线、8 字、正弦、三次多项式、阶跃加速度以及暂停/恢复过渡轨迹。
 * - MainWindow 负责把 UI 参数组装成 EndpointRequest，本类只做数学生成、插值、切片、重采样和文件解析。
 */

#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <QString>

#include <Eigen/Dense>

#include <cstddef>
#include <utility>
#include <vector>

class TrajectoryPlanner
{
public:
    using TrajectoryBlock = std::vector<std::vector<std::vector<double>>>;
    using MultiEndTrajectory = std::vector<std::vector<std::vector<std::vector<double>>>>;

    // 轨迹的前四层保持既有约定：位置、速度、加速度、时间。
    // SO(3) 数据只附加在直线五次轨迹上，避免把显示欧拉角导数和真实角速度混用。
    static constexpr std::size_t kTrajectoryPoseLayer = 0;
    static constexpr std::size_t kTrajectoryVelocityLayer = 1;
    static constexpr std::size_t kTrajectoryAccelerationLayer = 2;
    static constexpr std::size_t kTrajectoryTimeLayer = 3;
    static constexpr std::size_t kSO3RotationLayer = 4;       // 9 行，按行存储 R_global_from_body
    static constexpr std::size_t kSO3OmegaBodyLayer = 5;      // 3 行，机体系角速度
    static constexpr std::size_t kSO3AlphaBodyLayer = 6;      // 3 行，机体系角加速度
    static constexpr std::size_t kSO3OmegaGlobalLayer = 7;    // 3 行，全局系角速度
    static constexpr std::size_t kSO3AlphaGlobalLayer = 8;    // 3 行，全局系角加速度

    struct SO3AttitudeSample
    {
        Eigen::Matrix3d rotationGlobalFromBody = Eigen::Matrix3d::Identity();
        Eigen::Vector3d omegaBody = Eigen::Vector3d::Zero();
        Eigen::Vector3d alphaBody = Eigen::Vector3d::Zero();
        Eigen::Vector3d omegaGlobal = Eigen::Vector3d::Zero();
        Eigen::Vector3d alphaGlobal = Eigen::Vector3d::Zero();
    };

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
        double duration;
        int direction;
        double stepTime;
        double startLambda = 0.0;
        double endLambda = 1.0;
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
        double startLambda = 0.0;
        double endLambda = 1.0;
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
        bool preserveImportedTimeStep = false;
        MultiEndTrajectory offlineTraj;
        std::vector<double> firstEndStartPose;
        std::vector<int> segmentFlags;
        std::vector<std::pair<int, int>> segmentRanges;
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

    // 生成满足起止位置/速度/加速度约束的五次多项式轨迹块。
    static TrajectoryBlock quintic(const std::vector<double>& startPose,
                                   const std::vector<double>& startVel,
                                   const std::vector<double>& startAcc,
                                   const std::vector<double>& endPose,
                                   const std::vector<double>& endVel,
                                   const std::vector<double>& endAcc,
                                   double duration,
                                   double stepTime);
    // 对单个标量参数生成五次多项式进度曲线，常用于 lambda 插值。
    static std::vector<double> quinticParameterProfile(double startValue,
                                                        double endValue,
                                                        double duration,
                                                        double stepTime);

    // 为已生成的“直线五次、静止起止”轨迹附加 SO(3) 姿态。
    // position[3:5] 仅保留连续显示用的 ZYX 欧拉角；后续几何和动力学
    // 应通过 readSO3AttitudeSample() 使用旋转矩阵及真实角速度/角加速度。
    static bool applySO3QuinticAttitude(TrajectoryBlock& trajectory,
                                        const std::vector<double>& startEuler,
                                        const std::vector<double>& endEuler,
                                        QString* errorMessage = nullptr);
    // 为姿态恒定的轨迹附加 SO(3) 数据（圆形、阶跃等）。
    static bool applySO3ConstantAttitude(TrajectoryBlock& trajectory,
                                         const std::vector<double>& euler,
                                         QString* errorMessage = nullptr);
    // 沿既有六维标量进度将端点姿态改为 SO(3) 最短旋转（S 曲线、正弦）。
    static bool applySO3EndpointProgressAttitude(TrajectoryBlock& trajectory,
                                                 const std::vector<double>& startPose,
                                                 const std::vector<double>& endPose,
                                                 QString* errorMessage = nullptr);
    // SO(3) 三次 Hermite 姿态，保留原三次轨迹设置的起止角速度。
    static bool applySO3CubicHermiteAttitude(TrajectoryBlock& trajectory,
                                             const std::vector<double>& startEuler,
                                             const std::vector<double>& endEuler,
                                             QString* errorMessage = nullptr);
    // 保留既有欧拉角波形作为显示/定义，但使用 SO(3) 矩阵和真实角运动学。
    static bool applySO3EulerWaveformAttitude(TrajectoryBlock& trajectory,
                                              QString* errorMessage = nullptr);
    static bool hasSO3AttitudeData(const TrajectoryBlock& trajectory);
    static bool readSO3AttitudeSample(const TrajectoryBlock& trajectory,
                                      int pointIndex,
                                      SO3AttitudeSample& sample);

    // 生成圆轨迹；start/end lambda 支持从轨迹中间暂停后重规划。
    static TrajectoryBlock circular(
        const std::vector<double>& startPose,     
        const std::vector<double>& center, // [x,y,z]
        const double radius,
        const double duration,
        const int direction,
        double stepTime,
        double startLambda = 0.0,
        double endLambda = 1.0);


    // 生成直线 S 曲线轨迹，自动按最大速度、加速度和减速度分段。
    static TrajectoryBlock scurve(        
        const std::vector<double>& startPose,
        const std::vector<double>& endPose,
        const double vmax,
        const double acc,
        const double dec,
        double stepTime);

    // 生成变姿态 8 字轨迹，normal 决定运动平面方向。
    static TrajectoryBlock eightShape(        
        const std::vector<double>& startPose,
        const std::vector<double>& normal,
        const double R,
        const double range,
        double duration,
        double stepTime,
        double startLambda = 0.0,
        double endLambda = 1.0);

    // 生成正弦加速度直线轨迹，适合测试加速度扰动响应。
    static TrajectoryBlock sineshape(        
        const std::vector<double>& startPose,
        const std::vector<double>& endPose,
        const double A,
        const double w,
        const double phi,
        double duration,
        double stepTime);


    // 生成加速度阶跃轨迹，用于测试控制器对突变加速度的响应。
    static TrajectoryBlock stepAccel(        
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& dir,    // 运动方向（单位向量）

        const double a_before,            // 阶跃前加速度
        const double a_after,             // 阶跃后加速度

        const double t_step,              // 阶跃发生时刻
        double stepTime);            // 采样时间);

    // 生成满足起止位置和速度约束的三次多项式轨迹。
    static TrajectoryBlock cubicline(        
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& startVel,     // 初始速度
        const std::vector<double>& endPose,     // 初始位置
        const std::vector<double>& endVel,     // 初始速度
        double duration,            // 总时长
        double stepTime);            // 采样时间);


    // 根据每个末端的请求批量生成多末端轨迹；MainWindow 的规划入口主要调用这里。
    static MultiEndTrajectory buildLineTrajectory(const std::vector<EndpointRequest>& requests, const TrajectoryPlanner::TrajType& type);
    // 将外部文件中的单末端位姿行转换为统一的多末端轨迹结构。
    static MultiEndTrajectory buildSingleEndOfflinePoseTrajectory(const std::vector<std::vector<double>>& poseRows);

    // 按时间戳对点轨迹插值，返回 targetTime 对应的位置。
    static std::vector<double> interpolatePointTrajectory(const std::vector<std::vector<double>>& positionTraj,
                                                          const std::vector<double>& timeStamp,
                                                          double targetTime);
    // 按邻近采样点估计 targetTime 处速度，用于暂停/恢复平滑过渡。
    static std::vector<double> estimatePointTrajectoryVelocity(const std::vector<std::vector<double>>& positionTraj,
                                                               const std::vector<double>& timeStamp,
                                                               double targetTime);
    // 从当前轨迹点平滑过渡到停止点，生成暂停段轨迹。
    static bool buildStopTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                              PointTrajectoryTransition& out,
                                              QString& errorMessage);
                                              
    // 从停止点平滑过渡回原轨迹，生成恢复段轨迹。
    static bool buildResumeTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                                PointTrajectoryTransition& out,
                                                QString& errorMessage);

    // 读取外部位姿轨迹文件，并解析分段标记、时间戳和首末端起点。
    // 分段位姿文件的起终点按位置模式 UI 坐标解释；Lite 传 0，G3 传既有 15° Rx 偏置。
    static bool loadPoseFile(const QString& path,
                             int expectedEndNum,
                             double positionModeUiRxOffsetRad,
                             FileTrajectory& out,
                             QString& errorMessage);

    // 截取多末端轨迹的指定点区间，用于 PVT 部分重发或恢复。
    static MultiEndTrajectory sliceTrajectory(const MultiEndTrajectory& traj,
                                              int startPointIndex,
                                              int endPointIndex);

    // 按最大步长重新采样轨迹时间轴，防止文件轨迹采样过稀。
    static void resampleTrajectory(
    std::vector<std::vector<std::vector<std::vector<double>>>>& traj,
    double maxStep);
};

#endif // TRAJECTORYPLANNER_H
