#ifndef CDPRDYNMODEL_H
#define CDPRDYNMODEL_H

#include <vector>
#include <QDebug>
#include <cmath>
#include <Eigen/Dense>

#include <MatrixFun.h>
#include <optimization.h>
#include <nlopt.h>
#include <nlopt.hpp>

using namespace Eigen;

class CdprDynModel {
public:
    CdprDynModel();
    CdprDynModel(int _dof, int _cableNum);

    // ==========================================
    // API: 设定系统物理参数(前面的质量、惯量是包含动平台及其悬吊的智能体的。agent代表动平台所悬吊的智能体。g取正代表z轴向上，否则z轴向下)
    // ==========================================
    void setPhysicalParams(double mass, const Matrix3d& inertia_body_local,
                           const std::vector<Vector3d>& anchors_global,
                           const std::vector<Vector3d>& points_local,// 局部吊点坐标
                           const VectorXd& nr_coeffs, // L = L0+nr*q，电机转角到绳长的转换系数
                           double mass_agent, const Matrix3d& inertia_agent_local,
                           double gravity_acc = 9.8);
    void setPhysicalParams(double mass, const Matrix3d& inertia_body_local,
                           const std::vector<Vector3d>& anchors_global,
                           const std::vector<Vector3d>& points_local,// 局部吊点坐标
                           const VectorXd& nr_coeffs, // L = L0+nr*q，电机转角到绳长的转换系数
                           double mass_agent, const Matrix3d& inertia_agent_local,
                           const Vector3d& p_sensor_local, // 六维力传感器在动平台局部坐标系下的位置
                           const Matrix3d& R_sensor_local, // 六维力传感器相对于动平台的旋转矩阵
                           const Vector3d& p_robot_local,  // 智能体质心在动平台局部坐标系下的位置
                           double gravity_acc = 9.8);

    // ==========================================
    // API: 设定电机/绞盘动力学参数
    // 注意：是Imq，而非Im（少了nr）。Fvq同理
    // ==========================================
    void setWinchParams(const MatrixXd& Im_q, const MatrixXd& Fv_q, const MatrixXd& Fc_q);

    // ==========================================
    // API: 设定阻抗控制参数 Md, Dd, Kd
    // ==========================================
    void setImpedanceGains(const MatrixXd& M_des, const MatrixXd& D_des, const MatrixXd& K_des);

    // ==========================================
    // API: 设定优化算法
    // ==========================================
    void setOptParams(std::string _sta, double fmin, std::vector<double> fmax);

    // ==========================================
    // API: 设定期望轨迹 (Xd, Xd_dot, Xd_ddot)
    // 如果用updateMicrogravityTrajectory，根据交互力实时计算期望轨迹，则第一次用该函数设置起点，之后就不用该函数
    // ==========================================
    void setDesiredTrajectory(const VectorXd& pos, const VectorXd& vel, const VectorXd& acc);

    // ==========================================================
    // 实时计算微重力环境下的期望轨迹
    // 对应公式: M_r * acc_r + Coriolis = F_r_ext
    // 输入:
    //   F_interaction: 智能体受到的交互力 (6维: Fx, Fy, Fz, Tx, Ty, Tz)，需在全局坐标系下
    //   dt: 采样/积分时间步长
    // 输出: 无 (直接更新类内部的 X_d, dX_d, ddX_d)
    // ==========================================================
    void updateMicrogravityTrajectory(const VectorXd& F_interaction, double dt);

    // ==========================================
    // 基于传感器读数和动力学，计算整体外部交互力 F_ext。对应公式: (20) - (25)
    // 需要用第二种setPhysicalParams设置参数
    // 输入:
    //   F_sensor_reading: 六维力传感器读数 [fx, fy, fz, mx, my, mz]
    //   X_curr, dX_curr, ddX_curr: 当前动平台的运动状态（用于计算惯性力）（注意：不是智能体的）
    // 输出:
    //   F_ext: 作用在动平台整体参考点上的广义交互力 [Fx, Fy, Fz, Mx, My, Mz]
    // ==========================================
    VectorXd calculateTotalExternalForce(const VectorXd& F_sensor_reading,
                                         const VectorXd& X_curr,
                                         const VectorXd& dX_curr,
                                         const VectorXd& ddX_curr);

    // ==========================================
    // 计算输出量 tau_m (对应公式 18)
    // 输入: 当前状态 X, dX, 外部力 F_ext, 采样时间 dt（姿态为动轴ZYX/定轴XYZ欧拉角）
    // ==========================================
    VectorXd calculateCableForce(const VectorXd& X_curr, const VectorXd& dX_curr, const VectorXd& F_ext, double dt);

    // 默认参数
    void setTestParams();
    void setPulleyRadius(double radius);
    void setMotorTorqueLimitNm(double limitNm);
    double getMotorTorqueLimitNm() const;
    bool isMotorTorqueLimitExceeded() const;
    int getMotorTorqueLimitExceededIndex() const;
    double getMotorTorqueLimitExceededValueNm() const;

    double optCableForce = 0.0;
    bool compTor = true;// 是否补偿力矩（若不补偿，则完全不考虑力矩）
    nlopt_result resultCode = NLOPT_SUCCESS;
private:
    int DOF = 6;      // 自由度
    int CABLE_NUM = 8; // 绳索数量 (示例设定为8)

    VectorXd X_d, dX_d, ddX_d;
    MatrixXd Md_inv, Dd, Kd;
    double m_T;// 整体质量（动平台+智能体）
    Matrix3d I_T_local;// 整体局部惯量
    std::vector<Vector3d> B_points;
    std::vector<Vector3d> A_points;
    VectorXd nr;
    MatrixXd nr_inv;
    double g;
    MatrixXd I_m_q, F_v_q, F_c_q;
    VectorXd tau;

    Vector3d p_s_local; // 六维力传感器局部位置
    Matrix3d R_s_local; // 六维力传感器相对于动平台的旋转矩阵

    // --- (智能体) ---
    double m_r;// 智能体质量
    Matrix3d I_r_local;// 智能体局部惯量（相对于自身质心）
    Vector3d p_r_local; // 智能体质心局部位置

    std::string sta = "equ";
    double force_min;
    std::vector<double> force_max;
    int errIndex = 0;
    double motorTorqueLimitNm = 45.0;
    double pulleyRadius = 0.0;
    bool motorTorqueLimitExceeded = false;
    int motorTorqueLimitExceededIndex = -1;
    double motorTorqueLimitExceededValueNm = 0.0;

    MatrixXd J_dot;
    MatrixXd J_analytical_prev;
    bool is_first_run;
    MatrixXd current_J,current_J_breve;

    MatrixXd testJ_dot;
    bool testUseDefineJDot = false;

    double deg2rad(double degrees) {
        return degrees * 3.14159265358979323846 / 180.0;
    }

    // Matrix3d computeRotationMatrix(double a, double b, double c);// 基于动轴ZYX欧拉角计算旋转矩阵
    Matrix3d computeH(double alpha, double beta, double gamma);// 基于动ZYX欧拉角，计算欧拉角速度到旋转角速度的变化矩阵（对应H）
    MatrixXd computeH_breve(double a, double b, double c);// 基于动ZYX欧拉角，计算进行位置增广后的欧拉角速度到旋转角速度的变化矩阵（对应带短音符的H）
    MatrixXd computeH_breve_dot(double a, double b, double c, const Vector3d& rates);// 计算求导后的带短音符的H
    MatrixXd computeGeometricJacobian(const Vector3d& p, const Matrix3d& R);// 计算CDPR的雅可比矩阵（对应带短音符的J）

    // ==========================================================
    // 计算电机力矩 tau_m
    // 公式: tau = nr^-1 * J^T * tau_m
    // ==========================================================
    VectorXd computeMotorTorques(const VectorXd& taux);
    void applyMotorTorqueLimit(VectorXd& motorTorques);

    // ==========================================================
    // 考虑绞盘动力学损耗，计算实际绳索拉力
    // 基于公式 (12) 变形: F = Nr^-1 * (tau_m - tau_loss)
    
    // 输入:
    //   tau_m_input: 电机力矩
    //   dX_curr:     当前末端速度 (用于计算 q_dot)
    //   ddX_cmd:     期望/指令末端加速度 (用于估算 q_ddot)
    // ==========================================================
    VectorXd calculateActualCableTensions(const VectorXd& tau_m_input,const VectorXd& dX_curr,const VectorXd& ddX_cmd);
    VectorXd calculateActualCableTensionsOpt(const VectorXd& tau_m_input,const VectorXd& dX_curr,const VectorXd& ddX_cmd);

    MatrixXd tmpJTNrInv;
    std::vector<double> lastResult;
    static double myfunc(unsigned n, const double *x, double *grad, void *my_func_data);
    static void myconstraint(unsigned m, double *result,unsigned n, const double *x,
                             double *gradient,void *func_data);
    static double objFunNlopt(unsigned n, const double *x, double *grad, void *my_func_data);
    typedef struct
    {
        double a, b;
    }my_constraint_data;
};

#endif // CDPRDYNMODEL_H
