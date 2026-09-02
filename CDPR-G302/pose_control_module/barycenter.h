/*
 * 文件总览：
 * - BarycenterEigen 使用重心法为 8 根绳索分配张力，主要用于离线轨迹可行性检查和绳力预计算。
 * - 输入是平台质量/惯量、锚点、接点、绳力上下限以及轨迹点；输出为每个时刻的绳力和失败位置。
 * - 复杂算法集中在零空间顶点计算、顶点排序和多边形重心选择。
 */

#ifndef BARYCENTER_EIGEN_H
#define BARYCENTER_EIGEN_H

#include <Eigen/Dense>
#include <functional>
#include <vector>

class BarycenterEigen
{
public:

    struct SolveResult
    {
        std::vector<std::vector<double>> cable_force; // [8][T]
        std::vector<long long> barycenter_solve_us;
        bool is_valid = true;
        int failed_step = -1;
        double failed_time = -1.0;
    };

    // 配置重心法求解所需的刚体参数、锚点/连接点几何和每根绳索的张力上下限。
    void setParams(
        double mass,
        const Eigen::Matrix3d& Iee,
        const Eigen::Matrix<double, 3, 8>& base_points,
        const Eigen::Matrix<double, 3, 8>& attach_points,
        const Eigen::VectorXd& force_min,
        const Eigen::VectorXd& force_max,
        double pulley_radius = 0.0
        );

    // 按轨迹逐点求解 8 根绳索张力；任一点不可行时返回失败步和失败时间。
    SolveResult solveTrajectory(
        const std::vector<std::vector<std::vector<double>>>& traj
        );
    // 带进度回调的轨迹张力求解，供 UI 或耗时任务取消/显示进度使用。
    SolveResult solveTrajectory(
        const std::vector<std::vector<std::vector<double>>>& traj,
        const std::function<bool(int)>& progressCallback
        );



    // void setParams(
    //     double mass,
    //     Eigen::Matrix3d Iee,
    //     Eigen::Matrix<double, 3, 8> base_points,
    //     Eigen::Matrix<double, 3, 8> attach_points,
    //     double force_min, double force_max
    // );

    // /**
    //  * @brief 求解绳力分配（barycenter方法）
    //  *
    //  * @param force_ee   末端力 (3x1)
    //  * @param moment_ee  末端力矩 (3x1)
    //  * @param J          雅可比矩阵 (6 x n)
    //  * @param fmin       最小张力 (n x 1)
    //  * @param fmax       最大张力 (n x 1)
    //  * @return Eigen::VectorXd (n x 1) 绳力
    //  */
    // static Eigen::VectorXd solve(
    //     const Eigen::Vector3d& force_ee,
    //     const Eigen::Vector3d& moment_ee,
    //     const Eigen::MatrixXd& J,
    //     const Eigen::VectorXd& fmin,
    //     const Eigen::VectorXd& fmax
    // );

    // struct Params {
    //     double mass;
    //     Eigen::Matrix3d Iee;

    //     Eigen::Matrix<double, 3, 8> base_points;   // b_i
    //     Eigen::Matrix<double, 3, 8> attach_points; // a_i (EE系)

    //     double force_min;
    //     double force_max;
    // };

    // std::vector<std::vector<double>> cable_force;
    // bool is_valid;
    //总之后续通过is_valid是否为真来判断轨迹是否可行，为后续提供操作空间

    // static std::vector<std::vector<double>> solveTrajectory(
    //     const Params& param,
    //     const std::vector<std::vector<std::vector<double>>>& traj);

private:
    // ===== 参数 =====
    double mass;
    Eigen::Matrix3d Iee;
    Eigen::Matrix<double, 3, 8> base_points;
    Eigen::Matrix<double, 3, 8> attach_points;
    Eigen::VectorXd force_min, force_max;
    double pulley_radius = 0.0;


    // ===== 核心算法 =====
    // 单个轨迹点的核心求解：由末端力/力矩和雅可比矩阵计算满足约束的绳力。
    static Eigen::VectorXd solveStep(
        const Eigen::Vector3d& force_ee,
        const Eigen::Vector3d& moment_ee,
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& fmin,
        const Eigen::VectorXd& fmax
        );

    // 在零空间平面中枚举张力约束多边形的顶点。
    static std::vector<Eigen::Vector2d> computeVertices(
        const Eigen::VectorXd& tp,
        const Eigen::MatrixXd& N,
        const Eigen::VectorXd& fmin,
        const Eigen::VectorXd& fmax
        );

    // 将约束多边形顶点按极角排序，便于后续求重心。
    static std::vector<Eigen::Vector2d> sortVertices(
        const std::vector<Eigen::Vector2d>& vertices
        );

    // 计算约束多边形重心，作为重心法选取的零空间修正量。
    static Eigen::Vector2d computeCentroid(
        const std::vector<Eigen::Vector2d>& vertices
        );

    // ===== 运动学工具 =====
    // 将 ZYX 欧拉角及其导数转换为角速度。
    static Eigen::Vector3d eul2omega(
        const Eigen::Vector3d& eul,
        const Eigen::Vector3d& deul);

    // 根据 ZYX 欧拉角、一阶导和二阶导计算角加速度。
    static Eigen::Vector3d eulZYXddot2alpha(
        const Eigen::Vector3d& eul,
        const Eigen::Vector3d& deul,
        const Eigen::Vector3d& ddeul);

    // 将刚体坐标系下的角速度/角加速度转换到全局坐标系。
    static void bodyToGlobal(
        const Eigen::Vector3d& eul,
        const Eigen::Vector3d& omega_body,
        const Eigen::Vector3d& alpha_body,
        Eigen::Vector3d& omega_global,
        Eigen::Vector3d& alpha_global);
};

#endif
