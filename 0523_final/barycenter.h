#ifndef BARYCENTER_EIGEN_H
#define BARYCENTER_EIGEN_H

#include <Eigen/Dense>
#include <vector>

class BarycenterEigen
{
public:

    struct SolveResult
    {
        std::vector<std::vector<double>> cable_force; // [8][T]
        bool is_valid = true;
        int failed_step = -1;
        double failed_time = -1.0;
    };

    void setParams(
        double mass,
        const Eigen::Matrix3d& Iee,
        const Eigen::Matrix<double, 3, 8>& base_points,
        const Eigen::Matrix<double, 3, 8>& attach_points,
        const Eigen::VectorXd& force_min,
        const Eigen::VectorXd& force_max,
        double pulley_radius = 0.0
        );

    SolveResult solveTrajectory(
        const std::vector<std::vector<std::vector<double>>>& traj
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
    static Eigen::VectorXd solveStep(
        const Eigen::Vector3d& force_ee,
        const Eigen::Vector3d& moment_ee,
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& fmin,
        const Eigen::VectorXd& fmax
        );

    static std::vector<Eigen::Vector2d> computeVertices(
        const Eigen::VectorXd& tp,
        const Eigen::MatrixXd& N,
        const Eigen::VectorXd& fmin,
        const Eigen::VectorXd& fmax
        );

    static std::vector<Eigen::Vector2d> sortVertices(
        const std::vector<Eigen::Vector2d>& vertices
        );

    static Eigen::Vector2d computeCentroid(
        const std::vector<Eigen::Vector2d>& vertices
        );

    // ===== 运动学工具 =====
    static Eigen::Vector3d eul2omega(
        const Eigen::Vector3d& eul,
        const Eigen::Vector3d& deul);

    static Eigen::Vector3d eulZYXddot2alpha(
        const Eigen::Vector3d& eul,
        const Eigen::Vector3d& deul,
        const Eigen::Vector3d& ddeul);

    static void bodyToGlobal(
        const Eigen::Vector3d& eul,
        const Eigen::Vector3d& omega_body,
        const Eigen::Vector3d& alpha_body,
        Eigen::Vector3d& omega_global,
        Eigen::Vector3d& alpha_global);
};

#endif
