#include "cdprdynmodel.h"

#include <algorithm>

CdprDynModel::CdprDynModel(){

}

CdprDynModel::CdprDynModel(int _dof, int _cableNum)
    : DOF(_dof),CABLE_NUM(_cableNum){
    // 初始化变量尺寸
    X_d = VectorXd::Zero(DOF);
    dX_d = VectorXd::Zero(DOF);
    ddX_d = VectorXd::Zero(DOF);

    // 默认增益
    Md_inv = MatrixXd::Identity(DOF, DOF);
    Dd = MatrixXd::Identity(DOF, DOF);
    Kd = MatrixXd::Identity(DOF, DOF);

    // 初始化上一时刻的雅可比用于数值微分
    J_analytical_prev = MatrixXd::Zero(CABLE_NUM, DOF);
    is_first_run = true;
}

void CdprDynModel::setMotorTorqueLimitNm(double limitNm){
    if(std::isfinite(limitNm) && limitNm > 0.0){
        motorTorqueLimitNm = limitNm;
    }
}

double CdprDynModel::getMotorTorqueLimitNm() const{
    return motorTorqueLimitNm;
}

void CdprDynModel::setPulleyRadius(double radius){
    pulleyRadius = radius;
}

bool CdprDynModel::isMotorTorqueLimitExceeded() const{
    return motorTorqueLimitExceeded;
}

int CdprDynModel::getMotorTorqueLimitExceededIndex() const{
    return motorTorqueLimitExceededIndex;
}

double CdprDynModel::getMotorTorqueLimitExceededValueNm() const{
    return motorTorqueLimitExceededValueNm;
}

void CdprDynModel::setPhysicalParams(double mass, const Matrix3d& inertia_body_local,
                                     const std::vector<Vector3d>& anchors_global,
                                     const std::vector<Vector3d>& points_local,
                                     const VectorXd& nr_coeffs,
                                     double mass_agent, const Matrix3d& inertia_agent_local,
                                     double gravity_acc) {
    m_T = mass;
    I_T_local = inertia_body_local;
    B_points = anchors_global;
    A_points = points_local;
    nr = nr_coeffs;
    g = gravity_acc;

    nr_inv = MatrixXd::Zero(CABLE_NUM, CABLE_NUM);
    for(int i=0; i<CABLE_NUM; ++i) {
        nr_inv(i, i) = 1.0 / nr_coeffs(i);
    }

    m_r = mass_agent;
    I_r_local = inertia_agent_local;
}

void CdprDynModel::setPhysicalParams(double mass, const Matrix3d& inertia_body_local,
                                     const std::vector<Vector3d>& anchors_global,
                                     const std::vector<Vector3d>& points_local,
                                     const VectorXd& nr_coeffs,
                                     double mass_agent, const Matrix3d& inertia_agent_local,
                                     const Vector3d& p_sensor_local, const Matrix3d& R_sensor_local,
                                     const Vector3d& p_robot_local,
                                     double gravity_acc) {
    m_T = mass;
    I_T_local = inertia_body_local;
    B_points = anchors_global;
    A_points = points_local;
    nr = nr_coeffs;
    g = gravity_acc;

    nr_inv = MatrixXd::Zero(CABLE_NUM, CABLE_NUM);
    for(int i=0; i<CABLE_NUM; ++i) {
        nr_inv(i, i) = 1.0 / nr_coeffs(i);
    }

    m_r = mass_agent;
    I_r_local = inertia_agent_local;

    p_s_local = p_sensor_local;
    R_s_local = R_sensor_local;
    p_r_local = p_robot_local;
}

void CdprDynModel::setWinchParams(const MatrixXd& Im_q, const MatrixXd& Fv_q, const MatrixXd& Fc_q) {
    // 存储原始参数，计算时再处理 nr 系数
    I_m_q = Im_q;
    F_v_q = Fv_q;
    F_c_q = Fc_q;
}

void CdprDynModel::setImpedanceGains(const MatrixXd& M_des, const MatrixXd& D_des, const MatrixXd& K_des) {
    Md_inv = M_des.inverse();
    Dd = D_des;
    Kd = K_des;
}

void CdprDynModel::setOptParams(std::string _sta, double fmin, std::vector<double> fmax){
    sta = _sta;
    force_min = fmin;
    force_max = fmax;
}

void CdprDynModel::setDesiredTrajectory(const VectorXd& pos, const VectorXd& vel, const VectorXd& acc) {
    X_d = pos;
    dX_d = vel;
    ddX_d = acc;
}

void CdprDynModel::updateMicrogravityTrajectory(const VectorXd& F_interaction, double dt) {
    // 状态量定义:
    // X_d: [x, y, z, alpha, beta, gamma]
    // dX_d: [vx, vy, vz, d_alpha, d_beta, d_gamma] (注意：后三个是欧拉角速度，不是角速度 omega)

    // 1. 提取当前仿真状态 (上一时刻的期望值作为当前状态)
    Vector3d p_r = X_d.head(3);
    Vector3d euler = X_d.tail(3); // alpha, beta, gamma
    Vector3d v_r = dX_d.head(3);
    Vector3d euler_rate = dX_d.tail(3);

    // 2. 计算运动学矩阵 R 和 H
    // Matrix3d R = computeRotationMatrix(euler(0), euler(1), euler(2));
    Matrix3d R = MatrixFun::vecToMatrix(MatrixFun::getRots(std::vector<double>({0,0,0,euler(0), euler(1), euler(2)})));
    Matrix3d H = computeH(euler(0), euler(1), euler(2)); // H矩阵将欧拉角速度映射到角速度

    // 3. 计算当前角速度 omega (在全局坐标系表达，或者根据公式定义)
    // 公式中的 I_r 是 ^G I_r (全局惯量)，omega_r 也是全局角速度
    // 关系: omega = H * euler_rate
    Vector3d omega_r = H * euler_rate;

    // 4. 计算智能体在全局下的惯性矩阵 (图片2公式)
    // ^G I_r = R * ^e I_r * R^T
    Matrix3d I_r_global = R * I_r_local * R.transpose();

    // 5. 求解动力学方程
    // 线性部分: m_r * p_ddot = F_ext_force
    // 旋转部分: I_r_global * omega_dot + omega x (I_r_global * omega) = F_ext_torque
    Vector3d F_force = F_interaction.head(3);
    Vector3d F_torque = F_interaction.tail(3);

    // 5.1 计算线加速度 p_ddot
    // 模拟微重力，无重力项 mg
    Vector3d p_ddot = F_force / m_r;// qDebug() << F_interaction(2) << m_r;

    // 5.2 计算角加速度 omega_dot
    // omega_dot = (I_r_global)^-1 * (F_torque - omega x (I_r_global * omega))
    Vector3d coriolis_torque = omega_r.cross(I_r_global * omega_r); // 对应公式中的 ~omega * I * omega
    Vector3d net_torque = F_torque - coriolis_torque;

    // 使用 Eigen 的求解器或求逆来解 I * alpha = net_torque
    Vector3d omega_dot = I_r_global.inverse() * net_torque;
    // 也可以用 I_r_global.llt().solve(net_torque) 更快

    // 6. 数值积分 (更新状态)
    // 采用简单的欧拉积分 (Euler Integration)

    // 6.1 速度积分
    Vector3d v_r_next = v_r + p_ddot * dt;
    Vector3d omega_r_next = omega_r + omega_dot * dt;

    // 6.2 位置积分
    Vector3d p_r_next = p_r + v_r_next * dt;

    // 6.3 姿态积分 (难点: 需将 omega_next 转回 euler_rate_next)
    // 关系: euler_rate = H^-1 * omega
    // 注意: H 矩阵随姿态变化，这里用更新前的 H 近似，或者用预测后的姿态重算 H (Heun's method)，此处用简单近似
    Matrix3d H_inv = H.inverse(); // H 矩阵在 beta=90度时奇异，需注意
    Vector3d euler_rate_next = H_inv * omega_r_next;
    Vector3d euler_next = euler + euler_rate_next * dt;

    // 7. 更新类成员变量 (期望轨迹)
    X_d.head(3) = p_r_next;
    X_d.tail(3) = euler_next;

    dX_d.head(3) = v_r_next;
    dX_d.tail(3) = euler_rate_next;

    // 这里的 ddX_d 用于后续控制律的前馈。注意控制律需要的是 [p_ddot; euler_ddot]，而不是 [p_ddot; omega_dot]
    // 需计算 euler_ddot: d(H^-1 * omega)/dt = H^-1 * omega_dot + (H^-1)_dot * omega。这是一个繁琐的导数项。
    // 工程简化: 直接用差分计算 ddX_d，或者忽略 (H^-1)_dot 项
    // 简化方案: euler_ddot ≈ (euler_rate_next - euler_rate) / dt
    Vector3d euler_ddot = (euler_rate_next - euler_rate) / dt;
    ddX_d.head(3) = p_ddot;
    ddX_d.tail(3) = euler_ddot;
}


VectorXd CdprDynModel::calculateCableForce(const VectorXd& X_curr, const VectorXd& dX_curr, const VectorXd& F_ext, double dt) {
    // 1. 状态解包
    Vector3d p = X_curr.head(3);
    Vector3d euler = X_curr.tail(3); // alpha, beta, gamma
    Vector3d euler_rate = dX_curr.tail(3);

    // 2. 基础运动学矩阵计算
    // Matrix3d R = computeRotationMatrix(euler(0), euler(1), euler(2));
    Matrix3d R = MatrixFun::vecToMatrix(MatrixFun::getRots(std::vector<double>({0,0,0,euler(0), euler(1), euler(2)})));
    MatrixXd H_breve = computeH_breve(euler(0), euler(1), euler(2));
    MatrixXd H_breve_dot = computeH_breve_dot(euler(0), euler(1), euler(2), euler_rate);
    Matrix3d H = H_breve.block(3,3,3,3); // 提取 3x3 的 H 矩阵(H矩阵将欧拉角速度映射到角速度)

    // 3. 计算雅可比
    MatrixXd J_breve = computeGeometricJacobian(p, R);
    MatrixXd J = J_breve * H_breve; // 解析雅可比
    current_J = J;
    current_J_breve = J_breve;

    // 数值微分计算 J_dot
    // 解析求导极其复杂，此处采用数值微分
    J_dot = MatrixXd::Zero(CABLE_NUM, DOF);
    if(testUseDefineJDot){
        J_dot = testJ_dot;
    }
    else{
        if (!is_first_run) {
            J_dot = (J - J_analytical_prev) / dt;
        }
        J_analytical_prev = J;
        is_first_run = false;
    }

    // 4. 计算刚体动力学矩阵 M, C, G
    // 全局惯量: ^G I_T = R * ^e I_T * R^T
    Matrix3d I_T_global = R * I_T_local * R.transpose();

    // 刚体质量矩阵 M_body_diag (6x6)
    MatrixXd M_body_diag = MatrixXd::Zero(6, 6);
    M_body_diag.block(0,0,3,3) = m_T * Matrix3d::Identity();
    M_body_diag.block(3,3,3,3) = I_T_global;
    MatrixXd M = H_breve.transpose() * M_body_diag * H_breve;

    // C Term 1: H_breve^T * M_body * H_breve_dot
    MatrixXd C_term1 = H_breve.transpose() * M_body_diag * H_breve_dot;

    // C Term 2: H_breve^T * [0 0; 0 (H*phi_dot)x * I_T * H]
    // 计算角速度 omega = H * phi_dot
    Vector3d omega = H * euler_rate;
    // 计算反对称矩阵 S(omega)
    Matrix3d omega_skew;
    omega_skew << 0, -omega(2), omega(1),
            omega(2), 0, -omega(0),
            -omega(1), omega(0), 0;

    MatrixXd C_inner = MatrixXd::Zero(6, 6);
    C_inner.block(3,3,3,3) = omega_skew * I_T_global * H;// 已知向量v=[x1,y1,z1]，构造其反对称矩阵A(3x3)。设向量u=[x2,y2,z2]，则cross(v,u) == A*u
    MatrixXd C_term2 = H_breve.transpose() * C_inner;
    MatrixXd C = C_term1 + C_term2;

    // G 矩阵。G = H_breve^T * [-mt*g]
    VectorXd gravity_vec(6);
    gravity_vec << 0, 0, -m_T * g, 0, 0, 0;
    VectorXd G = H_breve.transpose() * (-1.0 * gravity_vec);

    // 5. 计算系统完整动力学矩阵 Me, Ce, Fe (图6 公式16)
    MatrixXd I_m = nr_inv * I_m_q;
    MatrixXd F_v = nr_inv * F_v_q;
    MatrixXd F_c = F_c_q;

    MatrixXd Me = M + J.transpose() * nr_inv * I_m * J;
    MatrixXd Ce = C + J.transpose() * nr_inv * I_m * J_dot;

    VectorXd dL = J * dX_curr;
    VectorXd sign_dL(CABLE_NUM);
    for(int i=0; i<CABLE_NUM; ++i) {
        if(dL(i) > 1e-6) sign_dL(i) = 1.0;
        else if(dL(i) < -1e-6) sign_dL(i) = -1.0;
        else sign_dL(i) = 0.0;
    }

    VectorXd Fe_term1 = J.transpose() * nr_inv * F_v * dL;
    VectorXd Fe_term2 = J.transpose() * nr_inv * F_c * sign_dL;
    VectorXd Fe = Fe_term1 + Fe_term2 + G;

//    qDebug() << "Fv" << F_v(0,0) << F_v(1,1) <<  F_v(2,2) <<  F_v(3,3) <<  F_v(4,4) <<  F_v(5,5);
//    qDebug() << "dL" << dL(0) << dL(1) << dL(2) << dL(3) << dL(4) << dL(5);
//    qDebug() << "dX_curr" << dX_curr(0) << dX_curr(1) << dX_curr(2) << dX_curr(3) << dX_curr(4) << dX_curr(5);
//    qDebug() << "Fe_term1" << Fe_term1(0) << Fe_term1(1) << Fe_term1(2) << Fe_term1(3) << Fe_term1(4) << Fe_term1(5);
//    qDebug() << "Fe_term2" << Fe_term2(0) << Fe_term2(1) << Fe_term2(2) << Fe_term2(3) << Fe_term2(4) << Fe_term2(5);
//    qDebug() << "G" << G(0) << G(1) << G(2) << G(3) << G(4) << G(5);

    // 检查向量中的每一个元素是否为 NaN 值，如果是则将其替换为 0
    X_d = (X_d.array().isNaN()).select(Eigen::VectorXd::Zero(X_d.size()), X_d);
    dX_d = (dX_d.array().isNaN()).select(Eigen::VectorXd::Zero(dX_d.size()), dX_d);
    ddX_d = (ddX_d.array().isNaN()).select(Eigen::VectorXd::Zero(ddX_d.size()), ddX_d);


    // 6. 计算 tau (公式 18)
    VectorXd e = X_d - X_curr;
    VectorXd e_dot = dX_d - dX_curr;

    // test
    // VectorXd testXc,testdXc;
    // testXc << 0.0422336,-0.0831488,0.0596024,0.620158,0.274722,-0.021534;
    // testdXc << -0.000254822,0.000988007,0.000615883,0.00164783,-0.0178598,0.0019723;
    // X_d = testXc;
    // e = X_d - X_curr;
    // e_dot = dX_d - dX_curr;

    VectorXd impedance_force = Dd * e_dot + Kd * e - F_ext;
    VectorXd acc_cmd = ddX_d + Md_inv * impedance_force;

    VectorXd dynamics_feedforward = Me * acc_cmd + Ce * dX_curr + Fe;
    VectorXd ext_force_mapping = H_breve.transpose() * F_ext;

//        qDebug() << "M" << (Me * acc_cmd)(0) << (Me * acc_cmd)(1) << (Me * acc_cmd)(2) <<
//                    (Me * acc_cmd)(3) << (Me * acc_cmd)(4) << (Me * acc_cmd)(5);
//        qDebug() << "C" << (Ce * dX_curr)(0) << (Ce * dX_curr)(1) << (Ce * dX_curr)(2) <<
//                    (Ce * dX_curr)(3) << (Ce * dX_curr)(4) << (Ce * dX_curr)(5);
//        qDebug() << "Fe" << Fe(0) << Fe(1) << Fe(2) <<
//                    Fe(3) << Fe(4) << Fe(5);
    qDebug() << "Fext" << F_ext(0) << F_ext(1) << F_ext(2) << F_ext(3) << F_ext(4) << F_ext(5);
    // qDebug() << "ext_force_mapping" << ext_force_mapping(0) << ext_force_mapping(1) << ext_force_mapping(2) << ext_force_mapping(3) << ext_force_mapping(4) << ext_force_mapping(5);

    tau = dynamics_feedforward - ext_force_mapping;

    // if(!compTor){
    //     tau(3)=0.0;tau(4)=0.0;tau(5)=0.0;
    // }

    qDebug() << "Xc" << X_curr(0) << X_curr(1) << X_curr(2) << X_curr(3) << X_curr(4) << X_curr(5);
    qDebug() << "dXc" << dX_curr(0) << dX_curr(1) << dX_curr(2) << dX_curr(3) << dX_curr(4) << dX_curr(5);
    qDebug() << "Xd" << X_d(0) << X_d(1) << X_d(2) << X_d(3) << X_d(4) << X_d(5);
    qDebug() << "dXd" << dX_d(0) << dX_d(1) << dX_d(2) << dX_d(3) << dX_d(4) << dX_d(5);
    qDebug() << "ddXd" << ddX_d(0) << ddX_d(1) << ddX_d(2) << ddX_d(3) << ddX_d(4) << ddX_d(5);
    qDebug() << "tau" << tau(0) << tau(1) << tau(2) << tau(3) << tau(4) << tau(5);

    return calculateActualCableTensions(computeMotorTorques(tau),dX_curr,ddX_d);
    // return calculateActualCableTensionsOpt(tau,dX_curr,ddX_d);
}

void CdprDynModel::setTestParams(){
    // 1. 质量计算
    double mass_ee = 100.0;// 总质量
    double mass_robot = 60.0;

    // 2. 全局锚点 (Base Anchors) - 对应 MATLAB b1_g 到 b8_g
    // MATLAB: [-8.9; 3.9; 5.9; 1] -> C++ Vector3d
    std::vector<Vector3d> anchors(CABLE_NUM);
    anchors[0] = Vector3d(-8.9,  3.9, 5.9);
    anchors[1] = Vector3d(-8.9, -3.9, 5.9);
    anchors[2] = Vector3d( 8.9, -3.9, 5.9);
    anchors[3] = Vector3d( 8.9,  3.9, 5.9);
    anchors[4] = Vector3d(-8.9,  3.9, 5.1);
    anchors[5] = Vector3d(-8.9, -3.9, 5.1);
    anchors[6] = Vector3d( 8.9, -3.9, 5.1);
    anchors[7] = Vector3d( 8.9,  3.9, 5.1);

    // 3. 局部连接点 (Attachment Points) - 对应 MATLAB 计算过程
    double lx = 0.640;
    double rad54 = deg2rad(54.0);
    double rad72 = deg2rad(72.0);
    double la = lx / (cos(rad54) + sin(rad72));
    double s72 = sin(rad72);
    double c72 = cos(rad72);

    // 计算原始坐标 (交错分布)
    std::vector<Vector3d> body_points(CABLE_NUM);
    body_points[0] = Vector3d(lx/2 - la*s72, -la*(0.5+c72),  0.16);
    body_points[1] = Vector3d(lx/2,          -la/2,          0.16);
    body_points[2] = Vector3d(lx/2,           la/2,          0.16);
    body_points[3] = Vector3d(lx/2 - la*s72,  la*(0.5+c72),  0.16);
    body_points[4] = Vector3d(-lx/2 + la*s72, la*(0.5+c72), -0.16);
    body_points[5] = Vector3d(-lx/2,          la/2,         -0.16);
    body_points[6] = Vector3d(-lx/2,         -la/2,         -0.16);
    body_points[7] = Vector3d(-lx/2 + la*s72, -la*(0.5+c72),-0.16);

    // 转换坐标原点到质心 (CoM Offset)
    // MATLAB: CoM_z = 0, 所以 offset 是 [0, 0, 0]. 代码里实际上没有位移，但保留逻辑
    Vector3d CoM_offset(0.0, 0.0, 0.0);
    for(auto& p : body_points) {
        p -= CoM_offset;
    }

    // 4. 惯性矩阵计算
    double ax = lx;
    double ay = 2 * la * (0.5 + c72);
    double az = 0.32;

    // I_ee (末端平台惯量)
    double Ixx_ee = (1.0/12.0) * mass_ee * (ay*ay + az*az);
    double Iyy_ee = (1.0/12.0) * mass_ee * (ax*ax + az*az);
    double Izz_ee = (1.0/12.0) * mass_ee * (ay*ay + ax*ax);
    Matrix3d I_ee;
    I_ee << Ixx_ee, 0, 0,
            0, Iyy_ee, 0,
            0, 0, Izz_ee;
    qDebug() << "Ixx,Iyy,Izz" << Ixx_ee << Iyy_ee << Izz_ee;

    // I_robot (智能体惯量)
    Matrix3d I_robot;
    I_robot << 0.23, 0, 0,
            0, 0.26, 0,
            0, 0, 0.35;

    // 5. 其他参数
    VectorXd nr_vec(8);
    nr_vec << 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06;
    double g_val = 9.8;

//    Vector3d p_sensor_local; // 六维力传感器在动平台局部坐标系下的位置
//    p_sensor_local << 0,0,0;
//    Matrix3d R_sensor_local; // 六维力传感器相对于动平台的旋转矩阵
//    R_sensor_local << 1, 0, 0,
//            0, 1, 0,
//            0, 0, 1;
//    Vector3d p_robot_local; // 智能体质心在动平台局部坐标系下的位置
//    p_robot_local << 0,0,0;

    // === 调用 API 设置物理参数 ===
    setPhysicalParams(mass_ee, I_ee, anchors, body_points, nr_vec, mass_robot, I_robot, g_val);
//    setPhysicalParams(mass_ee, I_ee, anchors, body_points, nr_vec, mass_robot, I_robot, p_sensor_local, R_sensor_local, p_robot_local, g_val);

    // 6. 电机与绞盘参数（注意，这里是Imq,Fvq,Fcq。simulink里的是Im,Fv，是乘过nr-1的）
    MatrixXd Imq = (nr_vec * 0.0019).asDiagonal();
    MatrixXd Fvq = (nr_vec * 0.1578).asDiagonal();
    MatrixXd Fcq = MatrixXd::Identity(8, 8) * 0.2570;
//    MatrixXd Imq = MatrixXd::Identity(8, 8) * 0.0019;
//    MatrixXd Fvq = MatrixXd::Identity(8, 8) * 0.1578;
//    MatrixXd Fcq = MatrixXd::Identity(8, 8) * 0.2570;
    qDebug() << "Imq,Fvq,Fcq" << nr_vec(0) * 0.0019 << nr_vec(0) * 0.1578 << 0.2570;

    setWinchParams(Imq, Fvq, Fcq);

    // 7. 设置 Force Min/Max (如果需要存入类中，需新增 API，这里暂存外部或传入计算函数)
    sta = "equ";
    force_min = 10.0;
    force_max = std::vector<double>(8,1000.0);

    // 阻抗参数
    Eigen::VectorXd impM(6), impD(6), impK(6);
    impM << 1000,1000,1000,800,800,800;
    impD << 90000,90000,90000,90000,90000,90000;
    impK << 400000,400000,400000,8000,8000,8000;
    setImpedanceGains(impM.asDiagonal(),impD.asDiagonal(),impK.asDiagonal());

    // 期望轨迹
    VectorXd posd(6),veld(6),accd(6);
    posd << -1.460480000000000,0.0,1.0,0.0,0.0,0.0; veld << 1.382284764802552,0.0,0.0,0.0,0.0,0.0; accd << 0.230611165719344,0.0,0.0,0.0,0.0,0.0;
    setDesiredTrajectory(posd,veld,accd);

    double xc = -1.460479053629516;double yc = -1.703088517640053e-12;double zc = 1.000000000060142;
    double mxc = 2.968431088906092e-10;double myc = 1.878283595921353e-09;double mzc = -1.961565184268136e-10;
    VectorXd posc(6),velc(6),fexd(6);
    posc << xc,yc,zc,mxc,myc,mzc;
    velc << 1.382285106493228,1.184897187127425e-12,-1.842970220877963e-11,-1.209265969085051e-09,-2.635536539531028e-09,4.195661636098623e-10;
    fexd << 0.012652105448478,1.777023220583228e-07,1.264265847567003e-08,-3.630176621165348e-09,2.530420149973241e-04,7.642507770420161e-12;

    testUseDefineJDot = true;
    testJ_dot = MatrixXd::Zero(CABLE_NUM, DOF);
    testJ_dot <<    0.0607,    0.0468,    0.0524,   -0.0251,    0.0137,    0.0169,
                    0.0528,   -0.0419,    0.0538,   -0.0045,   -0.0088,   -0.0024,
                    0.0328,    0.0344,   -0.0396,   -0.0137,    0.0179,    0.0042,
                    0.0281,   -0.0297,   -0.0395,   -0.0085,    0.0015,   -0.0072,
                    0.0523,    0.0454,    0.0542,    0.0255,   -0.0125,   -0.0142,
                    0.0610,   -0.0510,    0.0528,    0.0028,    0.0071,   0.0036,
                    0.0249,    0.0309,   -0.0357,    0.0124,   -0.0154,   -0.0047,
                    0.0295,   -0.0356,   -0.0358,    0.0064,   -0.0020,    0.0072;
    VectorXd result = calculateCableForce(posc, velc, fexd, 0.001);
    qDebug() << "Cable Force" << result(0) << result(1) << result(2) << result(3) << result(4) << result(5) << result(6) << result(7);
    testUseDefineJDot = false;
}

// Matrix3d CdprDynModel::computeRotationMatrix(double a, double b, double c) {
//     Matrix3d R;
//     R = AngleAxisd(a, Vector3d::UnitZ()) *
//             AngleAxisd(b, Vector3d::UnitY()) *
//             AngleAxisd(c, Vector3d::UnitX());
//     return R;
// }

Matrix3d CdprDynModel::computeH(double alpha, double beta, double gamma) {
    // 对应图片2中的 H 矩阵定义 (Z-Y-X 欧拉角速度映射)
    // [omega] = [H] * [euler_rate]
    // 注意：图片2给出的是 H_breve，右下角 3x3 块即为 H
    double cb = cos(beta), sb = sin(beta);
    double cg = cos(gamma), sg = sin(gamma);

    Matrix3d H;
    H << cb*cg, -sg, 0,
            cb*sg,  cg, 0,
            -sb,     0, 1;
    return H;
}

MatrixXd CdprDynModel::computeH_breve(double a, double b, double c) {
    double cb = cos(b), sb = sin(b);
    double cg = cos(c), sg = sin(c);
    MatrixXd H_breve = MatrixXd::Zero(6, 6);
    H_breve.block(0,0,3,3) = Matrix3d::Identity();
    Matrix3d H;
    H << cb*cg, -sg, 0,
            cb*sg,  cg, 0,
            -sb,     0, 1;
    H_breve.block(3,3,3,3) = H;
    return H_breve;
}

MatrixXd CdprDynModel::computeH_breve_dot(double a, double b, double c, const Vector3d& rates) {
    double da = rates(0), db = rates(1), dc = rates(2);
    double cb = cos(b), sb = sin(b);
    double cg = cos(c), sg = sin(c);
    MatrixXd H_breve_dot = MatrixXd::Zero(6, 6);
    Matrix3d H_dot;
    H_dot(0,0) = -sb*cg*db - cb*sg*dc; H_dot(0,1) = -cg*dc; H_dot(0,2) = 0;
    H_dot(1,0) = -sb*sg*db + cb*cg*dc; H_dot(1,1) = -sg*dc; H_dot(1,2) = 0;
    H_dot(2,0) = -cb*db;               H_dot(2,1) = 0;      H_dot(2,2) = 0;
    H_breve_dot.block(3,3,3,3) = H_dot;
    return H_breve_dot;
}

MatrixXd CdprDynModel::computeGeometricJacobian(const Vector3d& p, const Matrix3d& R) {
    MatrixXd J_breve = MatrixXd::Zero(CABLE_NUM, 6);
    for(int j=0; j<CABLE_NUM; ++j) {
        Vector3d Bj = B_points[j];
        Vector3d Aj = A_points[j];
        Vector3d RAj = R * Aj;
        const Vector3d contactPoint = p + RAj;
        const MatrixFun::CableLengthResult cableGeometry =
                MatrixFun::cableLengthCalculate(contactPoint, Bj, pulleyRadius);
        Vector3d vec = contactPoint - cableGeometry.tangentPoint;
        double len = cableGeometry.idealLength;
        if(!std::isfinite(len) || len <= 1e-12){
            continue;
        }
        Vector3d u = vec / len;
        J_breve.row(j).head(3) = u.transpose();
        J_breve.row(j).tail(3) = RAj.cross(u).transpose();

        // Vector3d test = p + RAj;qDebug() << j << test[0] << test[1] << test[2];qDebug() << Bj[0] << Bj[1] << Bj[2];
    }
    return J_breve;
}

void CdprDynModel::applyMotorTorqueLimit(VectorXd& motorTorques) {
    motorTorqueLimitExceeded = false;
    motorTorqueLimitExceededIndex = -1;
    motorTorqueLimitExceededValueNm = 0.0;

    if(!std::isfinite(motorTorqueLimitNm) || motorTorqueLimitNm <= 0.0){
        return;
    }

    const double tripTolerance = 1e-6;
    for(int i=0; i<motorTorques.size(); ++i){
        const double torque = motorTorques(i);
        if(!std::isfinite(torque)){
            continue;
        }

        const double absTorque = std::fabs(torque);
        if(absTorque >= motorTorqueLimitNm - tripTolerance){
            if(!motorTorqueLimitExceeded || absTorque > std::fabs(motorTorqueLimitExceededValueNm)){
                motorTorqueLimitExceeded = true;
                motorTorqueLimitExceededIndex = i;
                motorTorqueLimitExceededValueNm = torque;
            }
        }

        motorTorques(i) = std::clamp(torque, -motorTorqueLimitNm, motorTorqueLimitNm);
    }
}

VectorXd CdprDynModel::computeMotorTorques(const VectorXd& taux) {
    // 构建系数矩阵 A
    // 根据公式: tau = (nr^-1 * J^T) * tau_m
    // 令 A = nr^-1 * J^T，此时方程为: tau = A * tau_m
    // 为了进一步提高控制频率（机器人控制通常要求 <1ms），把矩阵计算移出nlopt约束函数
    MatrixXd J_T = current_J.transpose(); // 尺寸 (6, 8)
    tmpJTNrInv = J_T * nr_inv;            // 尺寸 (6, 8)

    // 优化算法
    double* ub = new double[CABLE_NUM];// 上限
    double* lb = new double[CABLE_NUM];// 下限
    double* x = new double[CABLE_NUM];// 初值
    for (int i = 0; i < CABLE_NUM; ++i){
        ub[i] = force_max[i]/nr_inv(i,i);
        lb[i] = force_min/nr_inv(i,i);
        if(lb[i] > ub[i]){
            std::swap(lb[i], ub[i]);
        }
        if(std::isfinite(motorTorqueLimitNm) && motorTorqueLimitNm > 0.0){
            const double originalLb = lb[i];
            lb[i] = std::max(lb[i], -motorTorqueLimitNm);
            ub[i] = std::min(ub[i], motorTorqueLimitNm);
            if(lb[i] > ub[i]){
                const double fallback = std::clamp(originalLb, -motorTorqueLimitNm, motorTorqueLimitNm);
                lb[i] = fallback;
                ub[i] = fallback;
            }
        }
        if(lastResult.empty())
            x[i] = lb[i];// 初值取下限，让绳索力尽量小
        else{
            x[i] = lastResult[i];// qDebug() << "startVal " << i << x[i];
        }
        x[i] = std::clamp(x[i], lb[i], ub[i]);
        // qDebug() << i << lb[i] << ub[i] << nr_inv(i,i);
    }

    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LD_SLSQP, CABLE_NUM);
    nlopt_set_upper_bounds(opt, ub);
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_maxtime(opt, 0.2);// 至多运行0.2s
    nlopt_set_min_objective(opt, &CdprDynModel::myfunc, this);
    my_constraint_data data[2] = { { 0, 0 }, { 0, 0 } };

    // 绳索力最大值约束（不等式约束）
    double* cableInequalityTol = new double[CABLE_NUM];// 不等式右端的值的数列
    for(int i=0;i<CABLE_NUM;++i){
        if(i<3)
            cableInequalityTol[i] = 1e-1;// 1e-6 即<=0。别改成别的值，因为不等式也是以是否在0附近为判据的
        else
            cableInequalityTol[i] = 1.0;// 1e-2 NM
    }

    // 绳索力抵消末端平台自身力和力矩，以实现零力状态（等式约束）
    double* cableEqualityTol = new double[6];// 不等式右端的值的数列
    for(int i=0;i<6;++i){
        if(i<3)
            cableEqualityTol[i] = 1e-1;// 1e-6 即<=0。别改成别的值，因为不等式也是以是否在0附近为判据的
        else
            cableEqualityTol[i] = 1e-2;// NM
    }

    int resultNum = 6;
    if(!compTor)
        resultNum = 3;// 不补偿力矩

    // 根据实测，使用等式进行力补偿，能在位姿是常量时求出唯一解；而使用最小二乘法，则在位姿是常量时求出的解浮动较大
    if(sta == "equ")
        nlopt_add_equality_mconstraint(opt, resultNum, &CdprDynModel::myconstraint, this, cableEqualityTol);// 补偿力和力矩
    else if(sta == "obj")
        nlopt_set_min_objective(opt, &CdprDynModel::objFunNlopt, this);// 补偿力和力矩
    else{
        qDebug() << "GravityCompensationThread warning: unknown sta, use equ instead.";
        nlopt_add_equality_mconstraint(opt, resultNum, &CdprDynModel::myconstraint, this, cableEqualityTol);// 补偿力和力矩
    }

    nlopt_set_xtol_rel(opt, 1e-6);
    double minf;
    resultCode = nlopt_optimize(opt, x, &minf);

    VectorXd tau_m;
    if (resultCode < 0){
        errIndex = 1;
        if(lastResult.empty()){
            tau_m = Eigen::Map<const VectorXd>(lb, CABLE_NUM);
        }
        else{
            tau_m = MatrixFun::vecToVxd(lastResult);
        }
        qDebug() << "GravityCompensationThread error: nlopt optimization fail." << resultCode;
    }
    else if(resultCode == NLOPT_MAXTIME_REACHED){
        errIndex = 2;
        if(lastResult.empty()){
            tau_m = Eigen::Map<const VectorXd>(lb, CABLE_NUM);
        }
        else{
            tau_m = MatrixFun::vecToVxd(lastResult);
        }
        qDebug() << "GravityCompensationThread warning: nlopt optimization stop because max time is reached.";
    }
    else{
        errIndex = 0;
        tau_m = Eigen::Map<const VectorXd>(x, CABLE_NUM);
        applyMotorTorqueLimit(tau_m);

        lastResult.resize(0);
        for(int i=0;i<CABLE_NUM;++i){
            lastResult.push_back(tau_m(i));
        }
    }// qDebug() << resultCode;

    if(resultCode < 0 || resultCode == NLOPT_MAXTIME_REACHED){
        applyMotorTorqueLimit(tau_m);
    }

    nlopt_destroy(opt);
    delete[] ub;
    delete[] lb;
    delete[] x;
    delete[] cableInequalityTol;
    delete[] cableEqualityTol;

    return tau_m;
}

VectorXd CdprDynModel::calculateActualCableTensions(const VectorXd& tau_m_input,
                                                    const VectorXd& dX_curr,
                                                    const VectorXd& ddX_cmd){
    // 1. 计算绳长变化速度 dL 和 加速度 ddL
    // dL = J * dX
    VectorXd dL = current_J * dX_curr;

    // ddL = J * ddX + J_dot * dX
    VectorXd ddL = current_J * ddX_cmd + J_dot * dX_curr;

//        qDebug() << "dL" << dL(0) << dL(1) << dL(2) << dL(3) << dL(4) << dL(5) << dL(6) << dL(07);
//        qDebug() << "ddL" << ddL(0) << ddL(1) << ddL(2) << ddL(3) << ddL(4) << ddL(5) << ddL(6) << ddL(07);

    // 2. 映射到电机空间 (关节空间)
    // q_dot = Nr^-1 * dL
    // q_ddot = Nr^-1 * ddL
    VectorXd q_dot = nr_inv * dL;
    VectorXd q_ddot = nr_inv * ddL;

    // 3. 计算绞盘内部损耗力矩 (Friction + Inertia)
    // tau_loss = Im_q * q_ddot + Fv_q * q_dot + Fc_q * sign(q_dot)

    VectorXd sign_q_dot(CABLE_NUM);
    for(int i=0; i<CABLE_NUM; ++i) {
        if(q_dot(i) > 1e-5) sign_q_dot(i) = 1.0;
        else if(q_dot(i) < -1e-5) sign_q_dot(i) = -1.0;
        else sign_q_dot(i) = 0.0;
    }

    VectorXd tau_inertia = I_m_q * q_ddot;
    VectorXd tau_viscous = F_v_q * q_dot;
    VectorXd tau_coulomb = F_c_q * sign_q_dot;

    VectorXd tau_loss = tau_inertia + tau_viscous + tau_coulomb;

    // 4. 计算净输出力矩 (用于产生拉力)
    // tau_net = tau_m - tau_loss
    VectorXd tau_net = tau_m_input - tau_loss;

    // 5. 转换为拉力 F
    // F = Nr^-1 * tau_net
    VectorXd F_actual = nr_inv * tau_net;

    // test
    // F_actual << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0;
    // for(int i=0;i<8;++i){
    //     qDebug() << i << B_points[i][0] << B_points[i][1] << B_points[i][2];
    // }

    // 验算/仿真
    std::vector<double> tmpCableF;
    for(int i=0;i<CABLE_NUM;++i){
        tmpCableF.push_back(F_actual(i));
    }
    VectorXd cablesF = -1.0*current_J_breve.transpose()*F_actual;// 绳索合力
    qDebug() << "Each Cable F" << tmpCableF;
    qDebug() << "Cables F" << cablesF(0) << cablesF(1) << cablesF(2) << cablesF(3) << cablesF(4) << cablesF(5);

    return F_actual;
}

VectorXd CdprDynModel::calculateActualCableTensionsOpt(const VectorXd& tau_x,
                                                       const VectorXd& dX_curr,
                                                       const VectorXd& ddX_cmd){
    MatrixXd J_T = current_J.transpose(); // 尺寸 (6, 8)
    tmpJTNrInv = J_T * nr_inv;            // 尺寸 (6, 8)
    VectorXd tau_m_input = tmpJTNrInv.completeOrthogonalDecomposition().solve(tau_x);// 伪逆

    // 1. 计算绳长变化速度 dL 和 加速度 ddL
    // dL = J * dX
    VectorXd dL = current_J * dX_curr;

    // ddL = J * ddX + J_dot * dX
    VectorXd ddL = current_J * ddX_cmd + J_dot * dX_curr;

    //        qDebug() << "dL" << dL(0) << dL(1) << dL(2) << dL(3) << dL(4) << dL(5) << dL(6) << dL(07);
    //        qDebug() << "ddL" << ddL(0) << ddL(1) << ddL(2) << ddL(3) << ddL(4) << ddL(5) << ddL(6) << ddL(07);

    // 2. 映射到电机空间 (关节空间)
    // q_dot = Nr^-1 * dL
    // q_ddot = Nr^-1 * ddL
    VectorXd q_dot = nr_inv * dL;
    VectorXd q_ddot = nr_inv * ddL;

    // 3. 计算绞盘内部损耗力矩 (Friction + Inertia)
    // tau_loss = Im_q * q_ddot + Fv_q * q_dot + Fc_q * sign(q_dot)

    VectorXd sign_q_dot(CABLE_NUM);
    for(int i=0; i<CABLE_NUM; ++i) {
        if(q_dot(i) > 1e-5) sign_q_dot(i) = 1.0;
        else if(q_dot(i) < -1e-5) sign_q_dot(i) = -1.0;
        else sign_q_dot(i) = 0.0;
    }

    VectorXd tau_inertia = I_m_q * q_ddot;
    VectorXd tau_viscous = F_v_q * q_dot;
    VectorXd tau_coulomb = F_c_q * sign_q_dot;

    VectorXd tau_loss = tau_inertia + tau_viscous + tau_coulomb;

    // 4. 计算净输出力矩 (用于产生拉力)
    // tau_net = tau_m - tau_loss
    VectorXd tau_net = tau_m_input - tau_loss;

    // 5. 转换为拉力 F
    // F = Nr^-1 * tau_net
    VectorXd F_actual = nr_inv * tau_net;

    // test
    // F_actual << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0;
    // for(int i=0;i<8;++i){
    //     qDebug() << i << B_points[i][0] << B_points[i][1] << B_points[i][2];
    // }

    // 验算/仿真
    std::vector<double> tmpCableF;
    for(int i=0;i<CABLE_NUM;++i){
        tmpCableF.push_back(F_actual(i));
    }
    VectorXd cablesF = -1.0*current_J_breve.transpose()*F_actual;// 绳索合力
    qDebug() << "Each Cable F" << tmpCableF;
    qDebug() << "Cables F" << cablesF(0) << cablesF(1) << cablesF(2) << cablesF(3) << cablesF(4) << cablesF(5);

    return F_actual;
}


double CdprDynModel::myfunc(unsigned n, const double *x, double *grad, void *my_func_data){
    CdprDynModel* pThis = reinterpret_cast<CdprDynModel*>(my_func_data);
    double res = 0.0;

    // (fmin+fmax)/n（效果：会尽量靠近设定值，但若不满足等式约束，会导致部分绳力很小而另一部分很大）
    for(unsigned i = 0; i < n; ++i) {
        // double target = (pThis->force_min/pThis->nr_inv(i,i) + pThis->force_max[i]/pThis->nr_inv(i,i)) / 2.0;

        double diff = x[i] - pThis->optCableForce/pThis->nr_inv(i,i);
        res += pow(diff, 2);

        // 梯度计算：f = (x - target)^2  =>  f' = 2 * (x - target)
        if (grad) {
            grad[i] = 2.0 * diff;
        }
    }

    // 最小范数（效果：各绳绳力会尽可能相近）
   // for(unsigned i = 0; i < n; ++i) {
   //     res += pow(x[i], 2);

   //     if (grad) {// 计算梯度
   //         grad[i] = 2.0 * x[i]; // f' = 2x
   //     }
   // }
    return res;
}

void CdprDynModel::myconstraint(unsigned m, double *result,unsigned n, const double *x,
                  double *gradient,void *func_data){
    // 将 func_data 强制转换为类的指针，从而访问成员变量
    CdprDynModel* pThis = reinterpret_cast<CdprDynModel*>(func_data);

    Eigen::Map<const VectorXd> taum(x, n);
    VectorXd tauOpt = -1.0*pThis->tmpJTNrInv*taum;
    for(int j=0;j<m;++j){// qDebug() << j << pThis->tau[j] << tauOpt[j];
        result[j] = pThis->tau[j] - tauOpt[j];
    }

    // 计算梯度
    // 约束 h_j(x) 对 x_i 的偏导数就是矩阵 A 的第 j 行第 i 列元素
    if (gradient) {
        for (unsigned j = 0; j < m; ++j) {
            for (unsigned i = 0; i < n; ++i) {
                // gradient 是一维数组，按行存储 (Row-Major)
                // grandient[j*n + i] 对应 d(result[j]) / d(x[i])
                gradient[j * n + i] = -1.0*-1.0*pThis->tmpJTNrInv(j, i);
            }
        }
    }
}

double CdprDynModel::objFunNlopt(unsigned n, const double *x, double *grad, void *my_func_data){
    // 将 func_data 强制转换为类的指针，从而访问成员变量
    CdprDynModel* pThis = reinterpret_cast<CdprDynModel*>(my_func_data);
    double result = 0.0;

    Eigen::Map<const VectorXd> taum(x, n);
    VectorXd tauOpt = -1.0*pThis->tmpJTNrInv*taum;
    std::vector<double> temp(6,0.0);
    for(int j=0;j<6;++j){
        temp[j] = pThis->tau[j] - tauOpt[j];
        result += pow(temp[j],2.0);
    }

    return result;
}

VectorXd CdprDynModel::calculateTotalExternalForce(const VectorXd& F_sensor_reading,
                                                   const VectorXd& X_curr,
                                                   const VectorXd& dX_curr,
                                                   const VectorXd& ddX_curr) {
    // -----------------------------------------------------
    // 步骤 0: 状态解包与运动学计算
    // -----------------------------------------------------
    Vector3d p = X_curr.head(3);       // 动平台参考点位置
    Vector3d euler = X_curr.tail(3);   // 姿态
    Vector3d v = dX_curr.head(3);      // 线速度
    Vector3d euler_rate = dX_curr.tail(3); // 欧拉角速度
    Vector3d a_lin = ddX_curr.head(3); // 线加速度
    Vector3d euler_acc = ddX_curr.tail(3); // 欧拉角加速度

    // 计算旋转矩阵 R 和角速度变换矩阵 H
    // Matrix3d R = computeRotationMatrix(euler(0), euler(1), euler(2));
    Matrix3d R = MatrixFun::vecToMatrix(MatrixFun::getRots(std::vector<double>({0,0,0,euler(0), euler(1), euler(2)})));
    Matrix3d H = computeH(euler(0), euler(1), euler(2));

    // 计算角速度 omega 和 角加速度 omega_dot
    Vector3d omega = H * euler_rate;

    // 计算 H_dot 用于求角加速度: alpha = H * euler_dd + H_dot * euler_d
    // 这里为了精确，我们需要 computeH_dot (类中已有 computeH_breve_dot，提取右下角即可)
    MatrixXd H_breve_dot = computeH_breve_dot(euler(0), euler(1), euler(2), euler_rate);
    Matrix3d H_dot = H_breve_dot.block(3,3,3,3);
    Vector3d omega_dot = H * euler_acc + H_dot * euler_rate;

    // -----------------------------------------------------
    // 步骤 1: 计算智能体的运动状态 (公式 21 相关)
    // -----------------------------------------------------
    // 智能体质心在全局坐标系下的位置 (虽然后续计算力矩主要用矢量差，但算出全局位置更清晰)
    // p_r = p + R * p_r_local

    // 智能体质心的线加速度 a_r (刚体运动学公式)
    // a_r = a_lin + alpha x r + omega x (omega x r)
    // 其中 r = R * p_r_local (从动平台原点指向智能体质心的向量)
    Vector3d r_vec = R * p_r_local;
    Vector3d term1 = omega_dot.cross(r_vec);
    Vector3d term2 = omega.cross(omega.cross(r_vec));
    Vector3d a_r = a_lin + term1 + term2;// a_r = a_lin;仅在 刚体没有旋转 或 智能体质心和动平台参考点位置重合 时才成立，否则还需要term1和term2。即动平台的角加速度会影响智能体线加速度

    // 智能体在全局下的惯性矩阵 (公式 22)
    Matrix3d I_r_global = R * I_r_local * R.transpose();

    // -----------------------------------------------------
    // 步骤 2: 处理传感器力 (公式 23, 24)
    // -----------------------------------------------------
    // 将传感器读数转换到全局坐标系方向（如果按照文档里假设六维力传感器坐标系与末端坐标系姿态相同，则R_s_local为单位矩阵，但依旧有R）
    // 例：如果机器人向右倾斜 90 度，此时重力垂直向下。对于传感器来说，它感受到的是“侧面受力”（它的 Y 轴或 X 轴受力），而不是 Z 轴受力。
    Matrix3d R_sensor_global = R * R_s_local;
    Vector3d f_sensor_global = R_sensor_global * F_sensor_reading.head(3);
    Vector3d m_sensor_global = R_sensor_global * F_sensor_reading.tail(3);

    // 计算作用在智能体质心 Oz 上的传感器广义力 ^r F_sensor (公式 24)
    // 公式24中: ^r F_sensor = F_sensor + [0; r_RS x f_sensor]
    // 其中 r_RS 是 "智能体指向传感器" 的向量 (全局系下)
    // 向量计算: r_RS = (Pos_Sensor - Pos_Agent)
    Vector3d r_RS = R * (p_s_local - p_r_local);

    Vector3d force_on_agent_from_sensor = f_sensor_global;
    Vector3d moment_on_agent_from_sensor = m_sensor_global + r_RS.cross(f_sensor_global);

    // -----------------------------------------------------
    // 步骤 3: 智能体动力学反解，求 ^r F_ext (公式 20)
    // -----------------------------------------------------
    // 公式 20: [Mass_Matrix] * [Acc] + [Coriolis] + [Gravity] = ^r F_sensor + ^r F_ext
    // 移项得: ^r F_ext = ([M]*[A] + [C] + [G]) - ^r F_sensor

    // 3.1 动力学项 (左边三项)
    // 惯性项: m_r * a_r
    Vector3d dyn_force = m_r * a_r;

    // 旋转动力学项: I * alpha + omega x (I * omega)
    Vector3d dyn_moment = I_r_global * omega_dot + omega.cross(I_r_global * omega);

    // 重力项: [-m_r * g] (注意公式20左边是 +[-mg]，移项后在右边也是+mg? 不，公式是 Dyn + G_vec = Forces)
    // 所以 Forces = Dyn + G_vec
    // 公式20中的重力向量是 [-m_r g] (假设z轴向上g为正，重力向下为负)
    // 注意: setPhysicalParams 里 gravity_acc 默认为 9.8 (正数)。
    // 代码中 G 向量通常定义为 [0, 0, -m*g]。
    Vector3d gravity_vec_agent;
    gravity_vec_agent << 0, 0, -m_r * g;

    // 3.2 计算 ^r F_ext (智能体受到的外部交互力)
    // 线性部分: f_ext_r = m*a - f_sensor - (-m*g)  (注意符号，公式20中 G项在左边)
    // 公式20: m*a + (-mg) = f_sensor + f_ext
    // => f_ext = m*a - mg - f_sensor
    Vector3d f_ext_r = dyn_force + gravity_vec_agent - force_on_agent_from_sensor;

    // 旋转部分: I*alpha + w*Iw = m_sensor + m_ext
    // => m_ext = (I*alpha + w*Iw) - m_sensor
    Vector3d m_ext_r = dyn_moment - moment_on_agent_from_sensor;

    // -----------------------------------------------------
    // 步骤 4: 等效变换到动平台整体参考点 (公式 25)
    // -----------------------------------------------------
    // 我们需要输出的是 F_ext (对整体动平台+智能体系统的作用力)
    // 参考点是 X 定义的原点 (Platform Overall CoM / Body Frame Origin)
    // 公式25: F_ext = ^r F_ext + [0; r_TR x f_ext_r]
    // r_TR: "动平台整体质心指向智能体质心" 的向量 (全局系下)
    // 这里对应代码中的 r_vec = R * p_r_local
    Vector3d r_TR = r_vec;// 注意！！！这里有问题：只有我们的局部坐标系原点(X_curr的位置)就是整体质心(Total CoM)，才成立

    Vector3d F_ext_total_force = f_ext_r;
    Vector3d F_ext_total_moment = m_ext_r + r_TR.cross(f_ext_r);

    // 组装结果
    VectorXd F_ext_total(6);
    F_ext_total << F_ext_total_force, F_ext_total_moment;

    return F_ext_total;
}
