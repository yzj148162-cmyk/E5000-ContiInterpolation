#include "barycenter.h"
#include "MatrixFun.h"
#include <algorithm>
#include <cmath>
#include <Eigen/SVD>

// ================= 参数设置 =================
void BarycenterEigen::setParams(
    double mass,
    const Eigen::Matrix3d& Iee,
    const Eigen::Matrix<double, 3, 8>& base_points,
    const Eigen::Matrix<double, 3, 8>& attach_points,
    const Eigen::VectorXd& force_min,
    const Eigen::VectorXd& force_max,
    double pulley_radius)
{
    this->mass = mass;
    this->Iee = Iee;
    this->base_points = base_points;
    this->attach_points = attach_points;
    this->force_min = force_min;
    this->force_max = force_max;
    this->pulley_radius = pulley_radius;
}

// ================= 轨迹求解 =================
BarycenterEigen::SolveResult BarycenterEigen::solveTrajectory(
    const std::vector<std::vector<std::vector<double>>>& traj)
{
    SolveResult result;

    if (traj.size() < 4 ||
        traj[0].size() < 6 ||
        traj[1].size() < 6 ||
        traj[2].size() < 6 ||
        traj[3].empty() ||
        traj[0][0].empty() ||
        force_min.size() != 8 ||
        force_max.size() != 8)
    {
        result.is_valid = false;
        return result;
    }

    int steps = static_cast<int>(traj[0][0].size());
    result.cable_force.assign(8, std::vector<double>(steps, 0.0));

    const double g = 9.8;
    const std::vector<double>& time_vec = traj[3][0];
    if (static_cast<int>(time_vec.size()) != steps)
    {
        result.is_valid = false;
        return result;
    }
    for (int i = 0; i < 6; ++i)
    {
        if (static_cast<int>(traj[0][i].size()) != steps ||
            static_cast<int>(traj[1][i].size()) != steps ||
            static_cast<int>(traj[2][i].size()) != steps)
        {
            result.is_valid = false;
            return result;
        }
    }

    for (int k = 0; k < steps; ++k)
    {
        Eigen::VectorXd pose(6), vel(6), acc(6);

        for (int i = 0; i < 6; ++i)
        {
            pose(i) = traj[0][i][k];
            vel(i)  = traj[1][i][k];
            acc(i)  = traj[2][i][k];
        }

        double x = pose(0), y = pose(1), z = pose(2);
        double ax = pose(3), ay = pose(4), az = pose(5);

        // ===== 构造位姿 =====
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        Eigen::Matrix4d Txyz = Eigen::Matrix4d::Identity();
        Txyz(0,3)=x; Txyz(1,3)=y; Txyz(2,3)=z;

        Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
        Rz(0,0)=cos(az); Rz(0,1)=-sin(az);
        Rz(1,0)=sin(az); Rz(1,1)=cos(az);

        Eigen::Matrix4d Ry = Eigen::Matrix4d::Identity();
        Ry(0,0)=cos(ay); Ry(0,2)=sin(ay);
        Ry(2,0)=-sin(ay); Ry(2,2)=cos(ay);

        Eigen::Matrix4d Rx = Eigen::Matrix4d::Identity();
        Rx(1,1)=cos(ax); Rx(1,2)=-sin(ax);
        Rx(2,1)=sin(ax); Rx(2,2)=cos(ax);

        T = Txyz * Rz * Ry * Rx;
        Eigen::Matrix3d R = T.block<3,3>(0,0);

        // ===== attach → global =====
        Eigen::Matrix<double, 3, 8> a_g;
        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector4d p;
            p << attach_points.col(i), 1.0;
            a_g.col(i) = (T * p).head<3>();
        }

        Eigen::Vector3d ao_g(x,y,z);

        // ===== Jacobian =====
        Eigen::MatrixXd J(6,8);

        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector3d b = base_points.col(i);
            Eigen::Vector3d a = a_g.col(i);
            const MatrixFun::CableLengthResult cable_geometry =
                    MatrixFun::cableLengthCalculate(a, b, pulley_radius);
            Eigen::Vector3d e = cable_geometry.tangentPoint;

            Eigen::Vector3d cable = e - a;
            double L = cable_geometry.idealLength;
            if (L <= 1e-9)
            {
                result.is_valid = false;
                result.failed_step = k;
                if (k < static_cast<int>(time_vec.size()))
                    result.failed_time = time_vec[k];
                return result;
            }
            Eigen::Vector3d u = cable / L;

            Eigen::Vector3d r = a - ao_g;

            J.block<3,1>(0,i) = u;
            J.block<3,1>(3,i) = r.cross(u);
        }

        Eigen::Vector3d force_ee;
        force_ee << 0, 0, -mass * g;
        force_ee += -acc.head<3>() * mass;

        Eigen::Vector3d eul = pose.tail<3>();
        Eigen::Vector3d deul = vel.tail<3>();
        Eigen::Vector3d ddeul = acc.tail<3>();

        // 1. 机体系角速度
        Eigen::Vector3d omega_body = eul2omega(eul, deul);

        // 2. 机体系角加速度
        Eigen::Vector3d alpha_body = eulZYXddot2alpha(eul, deul, ddeul);

        // 3. 转到全局系
        Eigen::Vector3d omega, alpha;
        bodyToGlobal(eul, omega_body, alpha_body, omega, alpha);


        Eigen::Matrix3d skew;
        skew <<     0, -omega(2), omega(1),
            omega(2), 0, -omega(0),
            -omega(1), omega(0), 0;

        Eigen::Vector3d moment_ee =
            -R * Iee * alpha
            -R * skew * Iee * omega;

        // =========================
        // 6. 张力上下限
        // =========================
        Eigen::VectorXd fmin = force_min;
        Eigen::VectorXd fmax = force_max;

        // =========================
        // 7. barycenter 求解
        // =========================
        Eigen::VectorXd f = solveStep(force_ee, moment_ee, J, fmin, fmax);

        // ❗ 不可行判断
        if (f.size() == 0)
        {
            result.is_valid = false;
            result.failed_step = k;
            if (k < static_cast<int>(time_vec.size()))
                result.failed_time = time_vec[k];
            return result;
        }

        for (int i = 0; i < 8; ++i)
            result.cable_force[i][k] = f(i);
    }

    return result;
}

// ================= barycenter核心 =================
Eigen::VectorXd BarycenterEigen::solveStep(
    const Eigen::Vector3d& force_ee,
    const Eigen::Vector3d& moment_ee,
    const Eigen::MatrixXd& J,
    const Eigen::VectorXd& fmin,
    const Eigen::VectorXd& fmax)
{
    const int rows = static_cast<int>(J.rows());
    const int n = static_cast<int>(J.cols());
    if (rows <= 0 || n <= 0 || fmin.size() != n || fmax.size() != n)
        return Eigen::VectorXd();

    Eigen::VectorXd w(6);
    w << force_ee, moment_ee;
    if (w.size() != rows)
        return Eigen::VectorXd();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J, Eigen::ComputeFullU | Eigen::ComputeFullV);

    if (svd.rank() != rows)
        return Eigen::VectorXd(); // Jacobian 非满行秩，当前 barycenter 前提不成立

    Eigen::VectorXd tp = -svd.solve(w);

    const int null_dim = n - rows;

    if (null_dim != 2)
        return Eigen::VectorXd(); // ❗失败

    Eigen::MatrixXd N = svd.matrixV().rightCols(null_dim);

    auto vertices = computeVertices(tp, N, fmin, fmax);

    if (vertices.size() < 3)
        return Eigen::VectorXd(); // ❗失败

    vertices = sortVertices(vertices);
    Eigen::Vector2d bc = computeCentroid(vertices);

    return tp + N * bc;
}

// ================= 几何函数 =================
std::vector<Eigen::Vector2d> BarycenterEigen::computeVertices(
    const Eigen::VectorXd& tp,
    const Eigen::MatrixXd& N,
    const Eigen::VectorXd& fmin,
    const Eigen::VectorXd& fmax)
{
    int n = tp.size();
    std::vector<Eigen::Vector2d> vertices;

    for (int i = 0; i < 2*n; ++i)
    {
        for (int j = i+1; j < 2*n; ++j)
        {
            int idx1 = i % n;
            int idx2 = j % n;

            double b1 = (i < n ? fmin(idx1) : fmax(idx1)) - tp(idx1);
            double b2 = (j < n ? fmin(idx2) : fmax(idx2)) - tp(idx2);

            Eigen::Matrix2d A;
            A << N(idx1,0), N(idx1,1),
                N(idx2,0), N(idx2,1);

            if (fabs(A.determinant()) < 1e-8)
                continue;

            Eigen::Vector2d x = A.inverse() * Eigen::Vector2d(b1,b2);
            Eigen::VectorXd f = tp + N * x;

            if ((f.array() >= fmin.array()-1e-5).all() &&
                (f.array() <= fmax.array()+1e-5).all())
            {
                vertices.push_back(x);
            }
        }
    }

    return vertices;
}

std::vector<Eigen::Vector2d> BarycenterEigen::sortVertices(
    const std::vector<Eigen::Vector2d>& vertices)
{
    Eigen::Vector2d center(0,0);
    for (auto& v : vertices) center += v;
    center /= vertices.size();

    auto sorted = vertices;

    std::sort(sorted.begin(), sorted.end(),
              [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
              {
                  return atan2(a.y()-center.y(), a.x()-center.x()) <
                         atan2(b.y()-center.y(), b.x()-center.x());
              });

    return sorted;
}

Eigen::Vector2d BarycenterEigen::computeCentroid(
    const std::vector<Eigen::Vector2d>& vertices)
{
    double A = 0;
    Eigen::Vector2d C(0,0);

    int n = vertices.size();

    for (int i = 0; i < n; ++i)
    {
        auto& p1 = vertices[i];
        auto& p2 = vertices[(i+1)%n];

        double cross = p1.x()*p2.y() - p2.x()*p1.y();

        A += cross;
        C += (p1+p2)*cross;
    }

    A *= 0.5;
    C /= (6*A);

    return C;
}

Eigen::Vector3d BarycenterEigen::eul2omega(
    const Eigen::Vector3d& eul,
    const Eigen::Vector3d& deul)
{
    double phi = eul(0);
    double theta = eul(1);

    double dphi = deul(0);
    double dtheta = deul(1);
    double dpsi = deul(2);

    Eigen::Vector3d omega;

    omega(0) = dphi - sin(theta) * dpsi;
    omega(1) = cos(phi) * dtheta + sin(phi) * cos(theta) * dpsi;
    omega(2) = -sin(phi) * dtheta + cos(phi) * cos(theta) * dpsi;

    return omega;
}

Eigen::Vector3d BarycenterEigen::eulZYXddot2alpha(
    const Eigen::Vector3d& eul,
    const Eigen::Vector3d& deul,
    const Eigen::Vector3d& ddeul)
{
    double phi = eul(0);
    double theta = eul(1);

    double dphi = deul(0);
    double dtheta = deul(1);
    double dpsi = deul(2);

    Eigen::Matrix3d J;
    J << 1, 0, -sin(theta),
        0, cos(phi), sin(phi)*cos(theta),
        0, -sin(phi), cos(phi)*cos(theta);

    Eigen::Matrix3d dJ;

    dJ << 0, 0, -cos(theta)*dtheta,
        0, -sin(phi)*dphi,
        cos(phi)*dphi*cos(theta) - sin(phi)*sin(theta)*dtheta,
        0, -cos(phi)*dphi,
        -sin(phi)*dphi*cos(theta) - cos(phi)*sin(theta)*dtheta;

    Eigen::Vector3d alpha = J * ddeul + dJ * deul;

    return alpha;
}

void BarycenterEigen::bodyToGlobal(
    const Eigen::Vector3d& eul,
    const Eigen::Vector3d& omega_body,
    const Eigen::Vector3d& alpha_body,
    Eigen::Vector3d& omega_global,
    Eigen::Vector3d& alpha_global)
{
    double phi = eul(0);
    double theta = eul(1);
    double psi = eul(2);

    Eigen::Matrix3d R;

    R << cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta),
        sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),
        sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),
        sin(phi)*cos(theta),
        cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi),
        cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi),
        cos(phi)*cos(theta);

    omega_global = R.transpose() * omega_body;
    alpha_global = R.transpose() * alpha_body;
}
