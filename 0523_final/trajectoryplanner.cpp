#include "trajectoryplanner.h"

#include <QFile>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace {

// 校验“离散点轨迹”的合法性
bool validatePointTrajectory(const std::vector<std::vector<double>>& positionTraj,
                             const std::vector<double>& timeStamp,
                             QString& errorMessage)
{
    // 至少要有两个点，且时间与轨迹长度一致
    if (positionTraj.size() < 2 || timeStamp.size() < 2 || positionTraj.size() != timeStamp.size()) {
        errorMessage = QStringLiteral("轨迹点或时间戳数量不足");
        return false;
    }

    // 获取维度（例如 xyz 或 xyz+rpy）
    const size_t dim = positionTraj.front().size();
    if (dim == 0) {
        errorMessage = QStringLiteral("轨迹维度为空");
        return false;
    }

    // 检查每个点维度一致，且时间严格递增
    for (size_t i = 0; i < positionTraj.size(); ++i) {
        if (positionTraj[i].size() != dim) {
            errorMessage = QStringLiteral("轨迹各点维度不一致");
            return false;
        }
        if (i > 0 && timeStamp[i] <= timeStamp[i - 1]) {
            errorMessage = QStringLiteral("轨迹时间戳必须严格递增");
            return false;
        }
    }

    return true;
}

// 时间裁剪到合法区间
double clampTime(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

bool parseDoubleLine(const QString& line,
                     int expectedCount,
                     std::vector<double>& values)
{
    const QStringList tokens = line.simplified().split(' ', Qt::SkipEmptyParts);
    if (expectedCount > 0 && tokens.size() != expectedCount) {
        return false;
    }

    values.clear();
    values.reserve(tokens.size());
    for (const QString& token : tokens) {
        bool ok = false;
        const double value = token.toDouble(&ok);
        if (!ok || !std::isfinite(value)) {
            return false;
        }
        values.push_back(value);
    }
    return true;
}

std::vector<double> buildUniformTimeAxis(int pointNum, double stepTime)
{
    std::vector<double> timeStamp;
    if (pointNum <= 0 || stepTime <= 0.0) {
        return timeStamp;
    }

    timeStamp.reserve(pointNum);
    for (int index = 0; index < pointNum; ++index) {
        timeStamp.push_back(index * stepTime);
    }
    return timeStamp;
}

std::vector<double> estimateTrajectoryDerivative(const std::vector<double>& samples,
                                                 const std::vector<double>& timeStamp)
{
    std::vector<double> derivative(samples.size(), 0.0);
    if (samples.size() != timeStamp.size() || samples.size() < 2) {
        return derivative;
    }

    for (size_t index = 0; index < samples.size(); ++index) {
        if (index == 0) {
            const double dt = timeStamp[1] - timeStamp[0];
            derivative[index] = dt > 1e-9 ? (samples[1] - samples[0]) / dt : 0.0;
        } else if (index + 1 == samples.size()) {
            const double dt = timeStamp[index] - timeStamp[index - 1];
            derivative[index] = dt > 1e-9 ? (samples[index] - samples[index - 1]) / dt : 0.0;
        } else {
            const double dt = timeStamp[index + 1] - timeStamp[index - 1];
            derivative[index] = dt > 1e-9 ? (samples[index + 1] - samples[index - 1]) / dt : 0.0;
        }
    }

    return derivative;
}

} // namespace


// ========================= 五次多项式轨迹 =========================
TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::quintic(
    const std::vector<double>& startPose,
    const std::vector<double>& startVel,
    const std::vector<double>& startAcc,
    const std::vector<double>& endPose,
    const std::vector<double>& endVel,
    const std::vector<double>& endAcc,
    double duration,
    double stepTime)
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    // 参数合法性检查
    if (startPose.size() != startVel.size() ||
        startPose.size() != startAcc.size() ||
        startPose.size() != endPose.size() ||
        startPose.size() != endVel.size() ||
        startPose.size() != endAcc.size() ||
        std::abs(duration) < 1e-9 ||
        std::abs(stepTime) < 1e-9) {
        return {};
    }

    // 分配维度
    for (auto& block : result) {
        block.resize(startPose.size());
    }

    // 五次多项式系数 c3 c4 c5
    std::vector<double> c3;
    std::vector<double> c4;
    std::vector<double> c5;
    c3.reserve(startPose.size());
    c4.reserve(startPose.size());
    c5.reserve(startPose.size());

    // 逐维计算多项式系数
    for (size_t i = 0; i < startPose.size(); ++i) {
        c3.push_back((20.0 * (endPose[i] - startPose[i]) - (8.0 * endVel[i] + 12.0 * startVel[i]) * duration +
                      (endAcc[i] - 3.0 * startAcc[i]) * std::pow(duration, 2.0)) /
                     (2.0 * std::pow(duration, 3.0)));

        c4.push_back((-30.0 * (endPose[i] - startPose[i]) + (14.0 * endVel[i] + 16.0 * startVel[i]) * duration -
                      (2.0 * endAcc[i] - 3.0 * startAcc[i]) * std::pow(duration, 2.0)) /
                     (2.0 * std::pow(duration, 4.0)));

        c5.push_back((12.0 * (endPose[i] - startPose[i]) - (6.0 * endVel[i] + 6.0 * startVel[i]) * duration +
                      (endAcc[i] - startAcc[i]) * std::pow(duration, 2.0)) /
                     (2.0 * std::pow(duration, 5.0)));
    }

    double currentTime = 0.0;

    // 离散步数
    const int maxStep = static_cast<int>(std::floor(duration / stepTime + 1e-9));

    // 轨迹生成
    for (int step = 0; step <= maxStep; ++step) {
        for (size_t dim = 0; dim < startPose.size(); ++dim) {

            // 时间
            result[3][dim].push_back(currentTime);

            // 位置
            result[0][dim].push_back(startPose[dim] + startVel[dim] * currentTime + startAcc[dim] * std::pow(currentTime, 2.0) +
                                     c3[dim] * std::pow(currentTime, 3.0) +
                                     c4[dim] * std::pow(currentTime, 4.0) +
                                     c5[dim] * std::pow(currentTime, 5.0));

            // 速度
            result[1][dim].push_back(startVel[dim] + 2.0 * startAcc[dim] * currentTime +
                                     3.0 * c3[dim] * std::pow(currentTime, 2.0) +
                                     4.0 * c4[dim] * std::pow(currentTime, 3.0) +
                                     5.0 * c5[dim] * std::pow(currentTime, 4.0));

            // 加速度
            result[2][dim].push_back(2.0 * startAcc[dim] +
                                     6.0 * c3[dim] * currentTime +
                                     12.0 * c4[dim] * std::pow(currentTime, 2.0) +
                                     20.0 * c5[dim] * std::pow(currentTime, 3.0));
        }

        currentTime += stepTime;
    }

    // 末端补点（保证精确到终点）
    if (result[3][0].empty() || std::abs(result[3][0].back() - duration) > 1e-9) {
        for (size_t dim = 0; dim < startPose.size(); ++dim) {
            result[3][dim].push_back(duration);
            result[0][dim].push_back(endPose[dim]);
            result[1][dim].push_back(endVel[dim]);
            result[2][dim].push_back(endAcc[dim]);
        }
    }

    return result;
}

TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::circular(
        const std::vector<double>& startPose,        
        const std::vector<double>& center, // [x,y,z]
        const double radius,
        const double omega,
        const int direction,
        double stepTime)
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    // ===== 参数检查 =====
    if (startPose.size() != 6 ||
        center.size() != 3 ||
        radius <= 0 ||
        omega <= 0 ||
        std::abs(stepTime) < 1e-9)
    {
        return {};
    }

    // 分配维度（6维：xyz + rpy）
    for (auto& block : result) {
        block.resize(6);
    }

    double duration = 2 * 3.1415926 / omega;

    // ===== 初始角度（由 startPose 决定）=====
    double dx0 = startPose[0] - center[0];
    double dy0 = startPose[1] - center[1];
    double theta0 = atan2(dy0, dx0);

    double time = 0.0;
    const int maxStep = static_cast<int>(std::floor(duration / stepTime + 1e-9));

    for (int step = 0; step <= maxStep; ++step)
    {
        double theta = theta0 + direction * omega * time;

        double cos_t = cos(theta);
        double sin_t = sin(theta);

        // ===== 位置 =====
        double x = center[0] + radius * cos_t;
        double y = center[1] + radius * sin_t;
        double z = startPose[2]; // 保持高度

        // ===== 速度 =====
        double vx = -radius * omega * sin_t * direction;
        double vy =  radius * omega * cos_t * direction;
        double vz = 0.0;

        // ===== 加速度 =====
        double ax = -radius * omega * omega * cos_t;
        double ay = -radius * omega * omega * sin_t;
        double az = 0.0;

        // ===== 写入 =====
        // 位置
        result[0][0].push_back(x);
        result[0][1].push_back(y);
        result[0][2].push_back(z);

        // 姿态（保持不变）
        result[0][3].push_back(startPose[3]);
        result[0][4].push_back(startPose[4]);
        result[0][5].push_back(startPose[5]);

        // 速度
        result[1][0].push_back(vx);
        result[1][1].push_back(vy);
        result[1][2].push_back(vz);

        result[1][3].push_back(0.0);
        result[1][4].push_back(0.0);
        result[1][5].push_back(0.0);

        // 加速度
        result[2][0].push_back(ax);
        result[2][1].push_back(ay);
        result[2][2].push_back(az);

        result[2][3].push_back(0.0);
        result[2][4].push_back(0.0);
        result[2][5].push_back(0.0);

        // 时间（每个维度都一样）
        for (int d = 0; d < 6; ++d) {
            result[3][d].push_back(time);
        }

        time += stepTime;
    }

    return result;
}


TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::scurve(
    const std::vector<double>& startPose,
    const std::vector<double>& endPose,
    const double vmax,
    const double acc,
    const double dec,
    double stepTime)
{
    TrajectoryBlock result(4);

    // ===== 参数检查 =====
    if (startPose.size() != endPose.size() ||
        vmax <= 0 || acc <= 0 || dec <= 0 ||
        stepTime <= 1e-9) {
        return {};
    }

    int dim = startPose.size();

    for (auto& block : result) {
        block.resize(dim);
    }

    // ===== 计算方向 & 距离 =====
    std::vector<double> dir(dim);
    double dist = 0.0;

    for (int i = 0; i < dim; ++i) {
        dir[i] = endPose[i] - startPose[i];
        dist += dir[i] * dir[i];
    }

    dist = std::sqrt(dist);
    if (dist < 1e-9) return {};

    for (int i = 0; i < dim; ++i) {
        dir[i] /= dist;
    }

    // ===== 时间参数 =====
    double ta = vmax / acc;
    double td = vmax / dec;

    double da = 0.5 * acc * ta * ta;
    double dd = 0.5 * dec * td * td;

    double tc = 0.0;

    // ===== 判断是否能达到 vmax =====
    if (dist > da + dd) {
        tc = (dist - da - dd) / vmax;
    } else {
        // 三角速度情况：重新计算 vmax
        // ta = std::sqrt(dist / (0.5 * acc * (1 + acc / dec)));
        // td = ta * acc / dec;
        // vmax = acc * ta;

        // da = 0.5 * acc * ta * ta;
        // dd = 0.5 * dec * td * td;
        // tc = 0.0;
        return {};
    }

    double totalTime = ta + tc + td;

    // ===== 轨迹生成 =====
    double time = 0.0;

    while (time <= totalTime + 1e-9) {

        double s = 0.0;
        double v = 0.0;
        double a = 0.0;

        if (time < ta) {
            // 加速段
            a = acc;
            v = acc * time;
            s = 0.5 * acc * time * time;
        }
        else if (time < ta + tc) {
            // 匀速段
            a = 0.0;
            v = vmax;
            s = da + vmax * (time - ta);
        }
        else {
            // 减速段
            double t2 = time - ta - tc;
            a = -dec;
            v = vmax - dec * t2;
            s = da + vmax * tc + vmax * t2 - 0.5 * dec * t2 * t2;
        }

        // ===== 写入各维 =====
        for (int i = 0; i < dim; ++i) {
            result[0][i].push_back(startPose[i] + dir[i] * s);
            result[1][i].push_back(dir[i] * v);
            result[2][i].push_back(dir[i] * a);
            result[3][i].push_back(time);
        }

        time += stepTime;
    }

    // ===== 末点修正 =====
    for (int i = 0; i < dim; ++i) {
        result[0][i].back() = endPose[i];
        result[1][i].back() = 0.0;
        result[2][i].back() = 0.0;
        result[3][i].back() = totalTime;
    }

    return result;


}

TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::eightShape(
        const std::vector<double>& startPose,
        const std::vector<double>& normal,
        const double R,
        const double range,
        double duration,
        double stepTime)
{
    TrajectoryBlock result(4);

    // ===== 参数检查 =====
    if (startPose.size() != 6 ||
        normal.size() != 3 ||
        R <= 0 ||
        duration <= 1e-9 ||
        stepTime <= 1e-9)
    {
        return {};
    }

    int dim = 6;

    for (auto& block : result) {
        block.resize(dim);
    }

    // ===== 初始位姿 =====
    std::vector<double> p0(3);
    std::vector<double> eul0(3);

    for (int i = 0; i < 3; ++i) {
        p0[i] = startPose[i];
        eul0[i] = startPose[i + 3];
    }

    // ===== 法向量单位化 =====
    double norm = std::sqrt(normal[0]*normal[0] +
                            normal[1]*normal[1] +
                            normal[2]*normal[2]);

    if (norm < 1e-9) return {};

    std::vector<double> n_vec = {
        normal[0]/norm,
        normal[1]/norm,
        normal[2]/norm
    };

    // ===== 构造平面基 u v =====
    std::vector<double> tmp =
        (std::fabs(n_vec[0]) < 0.9) ?
        std::vector<double>{1,0,0} :
        std::vector<double>{0,1,0};

    // u = n × tmp
    std::vector<double> u = {
        n_vec[1]*tmp[2] - n_vec[2]*tmp[1],
        n_vec[2]*tmp[0] - n_vec[0]*tmp[2],
        n_vec[0]*tmp[1] - n_vec[1]*tmp[0]
    };

    double u_norm = std::sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    for (auto &v : u) v /= u_norm;

    // v = n × u
    std::vector<double> v = {
        n_vec[1]*u[2] - n_vec[2]*u[1],
        n_vec[2]*u[0] - n_vec[0]*u[2],
        n_vec[0]*u[1] - n_vec[1]*u[0]
    };

    // ===== 轨迹生成 =====
    double time = 0.0;

    while (time <= duration + 1e-9)
    {
        double tau = time / duration;

        // ===== 五次时间参数 =====
        double s   = 10*pow(tau,3) - 15*pow(tau,4) + 6*pow(tau,5);
        double ds  = (30*pow(tau,2) - 60*pow(tau,3) + 30*pow(tau,4)) / duration;
        double dds = (60*tau - 180*pow(tau,2) + 120*pow(tau,3)) / (duration*duration);

        double theta   = 2*M_PI*s;
        double dtheta  = 2*M_PI*ds;
        double ddtheta = 2*M_PI*dds;

        // ===== 8字形 =====
        double x  = R*sin(theta);
        double y  = R*sin(theta)*cos(theta);

        double dx = R*cos(theta)*dtheta;
        double dy = R*cos(2*theta)*dtheta;

        double ddx = -R*sin(theta)*dtheta*dtheta + R*cos(theta)*ddtheta;
        double ddy = -2*R*sin(2*theta)*dtheta*dtheta + R*cos(2*theta)*ddtheta;

        // ===== 空间映射 =====
        std::vector<double> pos(3), vel(3), acc(3);

        for (int i = 0; i < 3; ++i) {
            pos[i] = p0[i] + x*u[i] + y*v[i];
            vel[i] = dx*u[i] + dy*v[i];
            acc[i] = ddx*u[i] + ddy*v[i];
        }

        // ===== 姿态 =====
        double roll  = eul0[0] + range*sin(theta);
        double pitch = eul0[1] + range*cos(theta);
        double yaw   = 0;

        double droll  = range*cos(theta)*dtheta;
        double dpitch = -range*sin(theta)*dtheta;
        double dyaw   = 0;

        double ddroll  = -range*sin(theta)*dtheta*dtheta + range*cos(theta)*ddtheta;
        double ddpitch = -range*cos(theta)*dtheta*dtheta - range*sin(theta)*ddtheta;
        double ddyaw   = 0;

        // ===== 写入 =====
        for (int i = 0; i < 3; ++i) {
            result[0][i].push_back(pos[i]);
            result[1][i].push_back(vel[i]);
            result[2][i].push_back(acc[i]);
            result[3][i].push_back(time);
        }

        result[0][3].push_back(roll);
        result[0][4].push_back(pitch);
        result[0][5].push_back(yaw);

        result[1][3].push_back(droll);
        result[1][4].push_back(dpitch);
        result[1][5].push_back(dyaw);

        result[2][3].push_back(ddroll);
        result[2][4].push_back(ddpitch);
        result[2][5].push_back(ddyaw);

        result[3][3].push_back(time);
        result[3][4].push_back(time);
        result[3][5].push_back(time);

        time += stepTime;
    }

    return result;
}

TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::sineshape(
        const std::vector<double>& startPose,
        const std::vector<double>& endPose,
        const double A,
        const double w,
        const double phi,
        double duration,
        double stepTime)
{
    TrajectoryBlock result(4);

    // ===== 参数检查 =====
    if (startPose.size() != endPose.size() ||
        w <= 0 ||
        duration <= 1e-9 ||
        stepTime <= 1e-9)
    {
        return {};
    }

    int dim = startPose.size();

    for (auto& block : result) {
        block.resize(dim);
    }

    // ===== 方向单位向量 =====
    std::vector<double> dir(dim);
    double dist = 0.0;

    for (int i = 0; i < dim; ++i) {
        dir[i] = endPose[i] - startPose[i];
        dist += dir[i] * dir[i];
    }

    dist = std::sqrt(dist);
    if (dist < 1e-9) return {};

    for (int i = 0; i < dim; ++i) {
        dir[i] /= dist;
    }

    // ===== 状态变量 =====
    double time = 0.0;
    double v = 0.0;
    double s = 0.0;

    // ===== 主循环 =====
    while (time <= duration + 1e-9)
    {
        double a = A * std::sin(w * time + phi);

        // 数值积分（欧拉）
        v += a * stepTime;
        s += v * stepTime;

        for (int i = 0; i < dim; ++i) {
            result[0][i].push_back(startPose[i] + dir[i] * s);
            result[1][i].push_back(dir[i] * v);
            result[2][i].push_back(dir[i] * a);
            result[3][i].push_back(time);
        }

        time += stepTime;
    }

    return result;
}


TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::stepAccel(
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& dir,    // 运动方向（单位向量）

        const double a_before,            // 阶跃前加速度
        const double a_after,             // 阶跃后加速度

        const double t_step,              // 阶跃发生时刻
        double stepTime)            // 采样时间);
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    int dim = startPose.size();

    // ===== 参数检查 =====
    if (dim < 3 ||
        dir.size() != 3 ||
        stepTime <= 1e-9 ||
        t_step < 0 ||
        std::abs(a_after) <= 1e-9)
    {
        return {};
    }
    const double dirNorm = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (dirNorm <= 1e-9) {
        return {};
    }

    // ===== 分配空间 =====
    for (auto& block : result)
        block.resize(dim);

    // ===== 时间推进 =====
    double time = 0.0;

    double v_scalar = 0.0; // 标量速度（沿dir）
    double s_scalar = 0.0; // 标量位移
    double duration = a_before * t_step / a_after + t_step;


    while (time <= duration + 1e-9)
    {
        // ===== 判断当前加速度 =====
        double a_scalar = (time < t_step) ? a_before : a_after;

        // ===== 积分（欧拉法）=====
        v_scalar += a_scalar * stepTime;
        s_scalar += v_scalar * stepTime;

        // ===== 写入各维 =====
        for (int i = 0; i < dim; ++i)
        {
            double pos = startPose[i];
            double vel = 0.0;
            double acc = 0.0;
            if (i < 3) {
                pos += dir[i] * s_scalar;
                vel = dir[i] * v_scalar;
                acc = dir[i] * a_scalar;
            }

            result[0][i].push_back(pos);
            result[1][i].push_back(vel);
            result[2][i].push_back(acc);
            result[3][i].push_back(time);
        }

        time += stepTime;
    }

    return result;
}


TrajectoryPlanner::TrajectoryBlock TrajectoryPlanner::cubicline(
        const std::vector<double>& startPose,     // 初始位置
        const std::vector<double>& startVel,     // 初始速度
        const std::vector<double>& endPose,     // 初始位置
        const std::vector<double>& endVel,     // 初始速度
        double duration,            // 总时长
        double stepTime)            // 采样时间);
{
    TrajectoryBlock result(4); // 0位置 1速度 2加速度 3时间

    int dim = startPose.size();

    // ===== 参数检查 =====
    if (dim == 0 ||
        startVel.size() != dim ||
        endPose.size() != dim ||
        endVel.size() != dim ||
        duration <= 1e-9 ||
        stepTime <= 1e-9)
    {
        return {};
    }

    // ===== 分配空间 =====
    for (auto& block : result)
        block.resize(dim);

    // ===== 系数 =====
    std::vector<double> a0 = startPose;
    std::vector<double> a1 = startVel;
    std::vector<double> a2(dim);
    std::vector<double> a3(dim);

    for (int i = 0; i < dim; ++i)
    {
        a2[i] = (3*(endPose[i] - startPose[i]) - (2*startVel[i] + endVel[i]) * duration)
                / (duration * duration);

        a3[i] = (2*(startPose[i] - endPose[i]) + (startVel[i] + endVel[i]) * duration)
                / (duration * duration * duration);
    }

    // ===== 轨迹生成 =====
    double time = 0.0;
    const int maxStep = static_cast<int>(std::floor(duration / stepTime + 1e-9));

    for (int step = 0; step <= maxStep; ++step)
    {
        for (int i = 0; i < dim; ++i)
        {
            double t = time;

            double p = a0[i] + a1[i]*t + a2[i]*t*t + a3[i]*t*t*t;
            double v = a1[i] + 2*a2[i]*t + 3*a3[i]*t*t;
            double a = 2*a2[i] + 6*a3[i]*t;

            result[0][i].push_back(p);
            result[1][i].push_back(v);
            result[2][i].push_back(a);
            result[3][i].push_back(time);
        }

        time += stepTime;
    }

    // ===== 终点补偿（防止浮点误差）=====
    if (result[3][0].empty() || std::abs(result[3][0].back() - duration) > 1e-9)
    {
        for (int i = 0; i < dim; ++i)
        {
            result[3][i].push_back(duration);
            result[0][i].push_back(endPose[i]);
            result[1][i].push_back(endVel[i]);

            // 加速度用公式算（更一致）
            double a_end = 2*a2[i] + 6*a3[i]*duration;
            result[2][i].push_back(a_end);
        }
    }

    return result;
}

// ========================= 多末端轨迹拼接 =========================
TrajectoryPlanner::MultiEndTrajectory
TrajectoryPlanner::buildLineTrajectory(const std::vector<EndpointRequest>& requests, const TrajectoryPlanner::TrajType& type)
{
    MultiEndTrajectory result;
    result.reserve(requests.size());
    switch (type)
    {
    case TrajectoryPlanner::TrajType::Quintic:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = quintic(
            request.q_param.startPose,
            request.q_param.startVel,
            request.q_param.startAcc,
            request.q_param.endPose,
            request.q_param.endVel,
            request.q_param.endAcc,
            request.q_param.duration,
            request.q_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::Circular:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = circular(
            request.cir_param.startPose,
            request.cir_param.center,
            request.cir_param.radius,
            request.cir_param.omega,
            request.cir_param.direction,
            request.cir_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::SCurve:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = scurve(
            request.s_param.startPose,
            request.s_param.endPose,
            request.s_param.vmax,
            request.s_param.acc,
            request.s_param.dec,
            request.s_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::EightShape:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = eightShape(
            request.e_param.startPose,
            request.e_param.normal,
            request.e_param.R,
            request.e_param.range,
            request.e_param.duration,
            request.e_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::Cubic:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = cubicline(
            request.cub_param.startPose,
            request.cub_param.startVel,
            request.cub_param.endPose,
            request.cub_param.endVel,
            request.cub_param.duration,
            request.cub_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::StepAccel:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = stepAccel(
            request.stepAccel_param.startPose,
            request.stepAccel_param.dir,
            request.stepAccel_param.a_before,
            request.stepAccel_param.a_after,
            request.stepAccel_param.t_step,
            request.stepAccel_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    case TrajectoryPlanner::TrajType::Sine:
        for (const EndpointRequest& request : requests) {
        TrajectoryBlock block = sineshape(
            request.sin_param.startPose,
            request.sin_param.endPose,
            request.sin_param.A,
            request.sin_param.w,
            request.sin_param.phi,
            request.sin_param.duration,
            request.sin_param.stepTime);

        if (block.empty()) {
            return {};
        }

        result.push_back(block);
        }
        break;
    default:
    return {};
    }
    // 每个末端独立生成轨迹

    return result;
}


// ========================= 离线轨迹直接装载 =========================
TrajectoryPlanner::MultiEndTrajectory
TrajectoryPlanner::buildSingleEndOfflinePoseTrajectory(
    const std::vector<std::vector<double>>& poseRows)
{
    if (poseRows.size() < 6) {
        return {};
    }

    MultiEndTrajectory result(1);
    result[0].resize(4);
    result[0][0].resize(6);

    // 直接填充位置（无速度加速度）
    for (int dim = 0; dim < 6; ++dim) {
        result[0][0][dim] = poseRows[dim];
    }

    return result;
}


// ========================= 插值 =========================
std::vector<double>
TrajectoryPlanner::interpolatePointTrajectory(
    const std::vector<std::vector<double>>& positionTraj,
    const std::vector<double>& timeStamp,
    double targetTime)
{
    QString errorMessage;

    if (!validatePointTrajectory(positionTraj, timeStamp, errorMessage)) {
        return {};
    }

    // 边界处理
    if (targetTime <= timeStamp.front()) return positionTraj.front();
    if (targetTime >= timeStamp.back())  return positionTraj.back();

    // 找区间
    auto upperIt = std::upper_bound(timeStamp.begin(), timeStamp.end(), targetTime);
    size_t upperIndex = std::distance(timeStamp.begin(), upperIt);
    size_t lowerIndex = upperIndex - 1;

    double t0 = timeStamp[lowerIndex];
    double t1 = timeStamp[upperIndex];
    double ratio = (targetTime - t0) / (t1 - t0);

    std::vector<double> result(positionTraj.front().size(), 0.0);

    // 线性插值
    for (size_t dim = 0; dim < result.size(); ++dim) {
        result[dim] = positionTraj[lowerIndex][dim] +
                      ratio * (positionTraj[upperIndex][dim] - positionTraj[lowerIndex][dim]);
    }

    return result;
}


// ========================= 速度估计 =========================
std::vector<double>
TrajectoryPlanner::estimatePointTrajectoryVelocity(
    const std::vector<std::vector<double>>& positionTraj,
    const std::vector<double>& timeStamp,
    double targetTime)
{
    QString errorMessage;

    if (!validatePointTrajectory(positionTraj, timeStamp, errorMessage)) {
        return {};
    }

    double t = clampTime(targetTime, timeStamp.front(), timeStamp.back());

    auto upperIt = std::upper_bound(timeStamp.begin(), timeStamp.end(), t);
    size_t upperIndex = std::distance(timeStamp.begin(), upperIt);

    if (upperIndex == 0) upperIndex = 1;
    if (upperIndex >= timeStamp.size()) upperIndex = timeStamp.size() - 1;

    size_t lowerIndex = upperIndex - 1;

    double dt = timeStamp[upperIndex] - timeStamp[lowerIndex];
    if (dt <= 1e-9) {
        return std::vector<double>(positionTraj.front().size(), 0.0);
    }

    std::vector<double> velocity(positionTraj.front().size(), 0.0);

    // 差分法
    for (size_t dim = 0; dim < velocity.size(); ++dim) {
        velocity[dim] = (positionTraj[upperIndex][dim] - positionTraj[lowerIndex][dim]) / dt;
    }

    return velocity;
}

bool TrajectoryPlanner::loadPoseFile(const QString& path,
                                     int expectedEndNum,
                                     FileTrajectory& out,
                                     QString& errorMessage)
{
    QFile trajFile(path);
    if (!trajFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        errorMessage = QStringLiteral("错误：轨迹文件打开失败");
        return false;
    }

    QStringList data;
    QTextStream stream(&trajFile);
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (!line.isEmpty() && !line.startsWith('#')) {
            data.append(line);
        }
    }

    if (data.size() < 8) {
        errorMessage = QStringLiteral("错误：轨迹文件数据不足");
        return false;
    }

    out = FileTrajectory{};
    bool ok = false;
    out.endNum = data.at(0).toInt(&ok);
    if (!ok || out.endNum <= 0) {
        errorMessage = QStringLiteral("错误：轨迹文件中的末端数量无效");
        return false;
    }
    if (expectedEndNum > 0 && out.endNum != expectedEndNum) {
        errorMessage = QStringLiteral("错误：读取的末端数量与界面配置不匹配");
        return false;
    }

    out.pointNum = data.at(1).toInt(&ok);
    if (!ok || out.pointNum < 2) {
        errorMessage = QStringLiteral("错误：轨迹文件中的轨迹点数量至少应为 2");
        return false;
    }

    const bool hasTimeStampLine = data.size() == (3 + out.endNum * 6);
    const bool hasLegacyPoseOnlyFormat = data.size() == (2 + out.endNum * 6);
    if (!hasTimeStampLine && !hasLegacyPoseOnlyFormat) {
        errorMessage = QStringLiteral("错误：轨迹文件格式不合法，应为“末端数 + 点数 + 时间戳行 + 6 行位姿/末端”");
        return false;
    }

    std::vector<double> fileTimeStamp;
    if (hasTimeStampLine) {
        if (!parseDoubleLine(data.at(2), out.pointNum, fileTimeStamp)) {
            errorMessage = QStringLiteral("错误：轨迹文件时间戳行格式不合法");
            return false;
        }
    } else {
        fileTimeStamp = buildUniformTimeAxis(out.pointNum, 1.0);
    }

    for (int index = 1; index < static_cast<int>(fileTimeStamp.size()); ++index) {
        if (fileTimeStamp[index] <= fileTimeStamp[index - 1]) {
            errorMessage = QStringLiteral("错误：轨迹文件时间戳必须严格递增");
            return false;
        }
    }

    const int poseLineBaseIndex = hasTimeStampLine ? 3 : 2;
    out.duration = fileTimeStamp.empty() ? 0.0 : fileTimeStamp.back();

    out.offlineTraj.resize(out.endNum);
    for (int endIndex = 0; endIndex < out.endNum; ++endIndex) {
        out.offlineTraj[endIndex].resize(4);
        out.offlineTraj[endIndex][0].resize(6);
        out.offlineTraj[endIndex][1].resize(6);
        out.offlineTraj[endIndex][2].resize(6);
        out.offlineTraj[endIndex][3].resize(6);
        for (int dim = 0; dim < 6; ++dim) {
            const int lineIndex = poseLineBaseIndex + endIndex * 6 + dim;
            if (!parseDoubleLine(data.at(lineIndex), out.pointNum, out.offlineTraj[endIndex][0][dim])) {
                errorMessage = QStringLiteral("错误：轨迹文件第 %1 行位姿数据格式不合法").arg(lineIndex + 1);
                return false;
            }
            out.offlineTraj[endIndex][3][dim] = fileTimeStamp;
            out.offlineTraj[endIndex][1][dim] =
                estimateTrajectoryDerivative(out.offlineTraj[endIndex][0][dim], fileTimeStamp);
            out.offlineTraj[endIndex][2][dim] =
                estimateTrajectoryDerivative(out.offlineTraj[endIndex][1][dim], fileTimeStamp);
        }
    }

    out.firstEndStartPose.resize(6);
    for (int dim = 0; dim < 6; ++dim) {
        out.firstEndStartPose[dim] = out.offlineTraj[0][0][dim][0];
    }

    return true;
}

void TrajectoryPlanner::resampleTrajectory(
    std::vector<std::vector<std::vector<std::vector<double>>>>& traj,
    double maxStep)
{
    if (maxStep <= 0.0) {
        return;
    }

    for (size_t endIndex = 0; endIndex < traj.size(); ++endIndex)
    {
        auto& tVec = traj[endIndex][3][0]; // 时间（所有dim相同）

        int dimNum = traj[endIndex][0].size();

        std::vector<std::vector<double>> newPos(dimNum);
        std::vector<std::vector<double>> newVel(dimNum);
        std::vector<std::vector<double>> newAcc(dimNum);
        std::vector<double> newTime;

        for (size_t k = 0; k < tVec.size() - 1; ++k)
        {
            double t0 = tVec[k];
            double t1 = tVec[k + 1];
            double dt = t1 - t0;

            // 原始点直接加入
            newTime.push_back(t0);
            for (int d = 0; d < dimNum; ++d) {
                newPos[d].push_back(traj[endIndex][0][d][k]);
                newVel[d].push_back(traj[endIndex][1][d][k]);
                newAcc[d].push_back(traj[endIndex][2][d][k]);
            }

            if (dt <= maxStep) continue;

            // ===== 需要插值 =====
            int insertNum = static_cast<int>(floor(dt / maxStep));

            for (int i = 1; i <= insertNum; ++i)
            {
                double ti = t0 + i * maxStep;
                if (ti >= t1) break;

                newTime.push_back(ti);

                double tau = ti - t0;

                for (int d = 0; d < dimNum; ++d)
                {
                    double p0 = traj[endIndex][0][d][k];
                    double v0 = traj[endIndex][1][d][k];
                    double p1 = traj[endIndex][0][d][k + 1];
                    double v1 = traj[endIndex][1][d][k + 1];

                    double T = dt;

                    double a0 = p0;
                    double a1 = v0;
                    double a2 = (3*(p1-p0) - (2*v0+v1)*T)/(T*T);
                    double a3 = (2*(p0-p1) + (v0+v1)*T)/(T*T*T);

                    double p = a0 + a1*tau + a2*tau*tau + a3*tau*tau*tau;
                    double v = a1 + 2*a2*tau + 3*a3*tau*tau;
                    double a = 2*a2 + 6*a3*tau;

                    newPos[d].push_back(p);
                    newVel[d].push_back(v);
                    newAcc[d].push_back(a);
                }
            }
        }

        // 最后一个点补上
        int last = tVec.size() - 1;
        newTime.push_back(tVec[last]);
        for (int d = 0; d < dimNum; ++d) {
            newPos[d].push_back(traj[endIndex][0][d][last]);
            newVel[d].push_back(traj[endIndex][1][d][last]);
            newAcc[d].push_back(traj[endIndex][2][d][last]);
        }

        // ===== 写回 =====
        traj[endIndex][0] = newPos;
        traj[endIndex][1] = newVel;
        traj[endIndex][2] = newAcc;

        for (int d = 0; d < dimNum; ++d)
            traj[endIndex][3][d] = newTime;
    }
}

bool TrajectoryPlanner::buildStopTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                                      PointTrajectoryTransition& out,
                                                      QString& errorMessage)
{
    if (!validatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, errorMessage)) {
        return false;
    }
    if (request.currentPosition.size() != request.sourcePositionTraj.front().size()) {
        errorMessage = QStringLiteral("当前点维度与轨迹维度不一致");
        return false;
    }

    const double safeSampleTime = std::max(request.sampleTime, 0.001);
    const double safeTransitionTime = std::max(request.transitionTime, safeSampleTime);
    const double startTime = clampTime(request.currentTrajectoryTime,
                                      request.sourceTimeStamp.front(),
                                      request.sourceTimeStamp.back());
    const double stopTime = clampTime(startTime + safeTransitionTime,
                                      request.sourceTimeStamp.front(),
                                      request.sourceTimeStamp.back());
    const double actualDuration = std::max(stopTime - startTime, safeSampleTime);
    const int stepCount = std::max(2, static_cast<int>(std::ceil(actualDuration / safeSampleTime)) + 1);

    out = PointTrajectoryTransition{};
    out.positionTraj.reserve(stepCount);
    out.timeStamp.reserve(stepCount);
    out.positionTraj.push_back(request.currentPosition);
    out.timeStamp.push_back(0.0);

    for (int step = 1; step < stepCount; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(stepCount - 1);
        const double t = startTime + (stopTime - startTime) * ratio;
        out.positionTraj.push_back(interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, t));
        out.timeStamp.push_back(actualDuration * ratio);
    }

    out.resumeTime = stopTime;
    out.referencePosition = interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, stopTime);
    return true;
}

bool TrajectoryPlanner::buildResumeTransitionTrajectory(const PointTrajectoryTransitionRequest& request,
                                                        PointTrajectoryTransition& out,
                                                        QString& errorMessage)
{
    if (!validatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, errorMessage)) {
        return false;
    }
    if (request.currentPosition.size() != request.sourcePositionTraj.front().size()) {
        errorMessage = QStringLiteral("当前点维度与轨迹维度不一致");
        return false;
    }

    const double startTime = clampTime(request.currentTrajectoryTime,
                                      request.sourceTimeStamp.front(),
                                      request.sourceTimeStamp.back());
    if (startTime >= request.sourceTimeStamp.back() - 1e-9) {
        errorMessage = QStringLiteral("轨迹已接近终点，无剩余轨迹可缓启");
        return false;
    }

    const double safeSampleTime = std::max(request.sampleTime, 0.001);
    const double safeTransitionTime = std::max(request.transitionTime, safeSampleTime);
    const double rampEndTime = clampTime(startTime + safeTransitionTime,
                                         request.sourceTimeStamp.front(),
                                         request.sourceTimeStamp.back());
    const double rampDuration = std::max(rampEndTime - startTime, safeSampleTime);
    const int rampStepCount = std::max(2, static_cast<int>(std::ceil(rampDuration / safeSampleTime)) + 1);
    out = PointTrajectoryTransition{};
    out.positionTraj.push_back(request.currentPosition);
    out.timeStamp.push_back(0.0);

    for (int step = 1; step < rampStepCount; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(rampStepCount - 1);
        const double t = startTime + (rampEndTime - startTime) * ratio;
        out.positionTraj.push_back(interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, t));
        out.timeStamp.push_back(rampDuration * ratio);
    }

    for (size_t pointIndex = 0; pointIndex < request.sourceTimeStamp.size(); ++pointIndex) {
        if (request.sourceTimeStamp[pointIndex] <= rampEndTime + 1e-9) {
            continue;
        }
        const std::vector<double>& point = request.sourcePositionTraj[pointIndex];
        const double relativeTime = request.sourceTimeStamp[pointIndex] - startTime;
        if (relativeTime > out.timeStamp.back() + 1e-9) {
            out.positionTraj.push_back(point);
            out.timeStamp.push_back(relativeTime);
        }
    }

    out.resumeTime = rampEndTime;
    out.referencePosition = interpolatePointTrajectory(request.sourcePositionTraj, request.sourceTimeStamp, rampEndTime);
    return out.positionTraj.size() >= 2;
}