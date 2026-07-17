#include "trajectorygenerator.h"

TrajectoryGenerator::TrajectoryGenerator()
    : trajectoryType(QUINTIC_POLYNOMIAL),
      circCenterX(0.0), circCenterY(0.0), circCenterZ(0.0),
      circRadius(100.0), circAngularSpeed(1.0), circClockwise(false),
      sMaxVel(200.0), sAcc(100.0), sDec(100.0),
      eightSizeX(500.0), eightSizeY(500.0), eightFreq(0.5), eightAttAmp(10.0),
      sineFreq(2.0), sineAmp(50.0), sinePhase(0.0),
      runTime(8.0), stepTime(0.1)
{}

TrajectoryGenerator::~TrajectoryGenerator()
{}

void TrajectoryGenerator::setTrajectoryType(TrajectoryType type)
{
    trajectoryType = type;
}

void TrajectoryGenerator::setCircularParams(double centerX, double centerY, double centerZ, double radius, double angularSpeed, bool clockwise)
{
    circCenterX = centerX;
    circCenterY = centerY;
    circCenterZ = centerZ;
    circRadius = radius;
    circAngularSpeed = angularSpeed;
    circClockwise = clockwise;
}

void TrajectoryGenerator::setLinearSCurveParams(double maxVel, double acc, double dec)
{
    sMaxVel = maxVel;
    sAcc = acc;
    sDec = dec;
}

void TrajectoryGenerator::setEightShapeParams(double sizeX, double sizeY, double freq, double attAmp)
{
    eightSizeX = sizeX;
    eightSizeY = sizeY;
    eightFreq = freq;
    eightAttAmp = attAmp;
}

void TrajectoryGenerator::setSineParams(double freq, double amp, double phase)
{
    sineFreq = freq;
    sineAmp = amp;
    sinePhase = phase;
}

void TrajectoryGenerator::setTraj(const std::vector<std::vector<double>>& p0, 
                                 const std::vector<std::vector<double>>& v0, 
                                 const std::vector<std::vector<double>>& a0, 
                                 const std::vector<std::vector<double>>& pt, 
                                 const std::vector<std::vector<double>>& vt, 
                                 const std::vector<std::vector<double>>& at, 
                                 double runTime, double stepTime)
{
    this->p0 = p0;
    this->v0 = v0;
    this->a0 = a0;
    this->pt = pt;
    this->vt = vt;
    this->at = at;
    this->runTime = runTime;
    this->stepTime = stepTime;
}

// std::vector<std::vector<std::vector<std::vector<double>>>> TrajectoryGenerator::generateTrajectory()
// {
//     switch (trajectoryType) {
//     case QUINTIC_POLYNOMIAL:
//         return generateQuinticPolynomial();
//     case CIRCULAR:
//         return generateCircular();
//     case LINEAR_S_CURVE:
//         return generateLinearSCurve();
//     case EIGHT_SHAPE:
//         return generateEightShape();
//     case SINE_ACCELERATION:
//         return generateSineAcceleration();
//     case CUBIC_POLYNOMIAL:
//         return generateCubicPolynomial();
//     default:
//         return generateQuinticPolynomial();
//     }
// }

std::vector<std::vector<std::vector<double>>> TrajectoryGenerator::generateQuinticPolynomial(std::vector<double> p0, std::vector<double> v0, std::vector<double> a0,
                                                                            std::vector<double> pt, std::vector<double> vt, std::vector<double> at,
                                                                            double t, double dt)
{
    std::vector<std::vector<std::vector<double>>> result(4);
    if (p0.size() != v0.size() || p0.size() != a0.size() || p0.size() != pt.size() ||
        p0.size() != vt.size() || p0.size() != at.size() || std::abs(t) < 1e-9 || std::abs(dt) < 1e-9) {
        return {};
    }

    for (auto& block : result) {
        block.resize(p0.size());
    }

    std::vector<double> c0(p0);
    std::vector<double> c1(v0);
    std::vector<double> c2(a0);
    std::vector<double> c3;
    std::vector<double> c4;
    std::vector<double> c5;
    c3.reserve(p0.size());
    c4.reserve(p0.size());
    c5.reserve(p0.size());

    for (size_t i = 0; i < p0.size(); ++i) {
        c3.push_back((20.0 * (pt[i] - p0[i]) - (8.0 * vt[i] + 12.0 * v0[i]) * t + (at[i] - 3.0 * a0[i]) * std::pow(t, 2.0)) /
                     (2.0 * std::pow(t, 3.0)));
        c4.push_back((-30.0 * (pt[i] - p0[i]) + (14.0 * vt[i] + 16.0 * v0[i]) * t - (2.0 * at[i] - 3.0 * a0[i]) * std::pow(t, 2.0)) /
                     (2.0 * std::pow(t, 4.0)));
        c5.push_back((12.0 * (pt[i] - p0[i]) - (6.0 * vt[i] + 6.0 * v0[i]) * t + (at[i] - a0[i]) * std::pow(t, 2.0)) /
                     (2.0 * std::pow(t, 5.0)));
    }

    double currentTime = 0.0;
    const int maxStep = static_cast<int>(std::floor(t / dt + 1e-9));
    for (int step = 0; step <= maxStep; ++step) {
        for (size_t dim = 0; dim < p0.size(); ++dim) {
            result[3][dim].push_back(currentTime);
            result[0][dim].push_back(c0[dim] + c1[dim] * currentTime + c2[dim] * std::pow(currentTime, 2.0) +
                                     c3[dim] * std::pow(currentTime, 3.0) + c4[dim] * std::pow(currentTime, 4.0) +
                                     c5[dim] * std::pow(currentTime, 5.0));
            result[1][dim].push_back(c1[dim] + 2.0 * c2[dim] * currentTime + 3.0 * c3[dim] * std::pow(currentTime, 2.0) +
                                     4.0 * c4[dim] * std::pow(currentTime, 3.0) + 5.0 * c5[dim] * std::pow(currentTime, 4.0));
            result[2][dim].push_back(2.0 * c2[dim] + 6.0 * c3[dim] * currentTime + 12.0 * c4[dim] * std::pow(currentTime, 2.0) +
                                     20.0 * c5[dim] * std::pow(currentTime, 3.0));
        }
        currentTime += dt;
    }

    if (result[3][0].empty() || std::abs(result[3][0].back() - t) > 1e-9) {
        for (size_t dim = 0; dim < p0.size(); ++dim) {
            result[3][dim].push_back(t);
            result[0][dim].push_back(pt[dim]);
            result[1][dim].push_back(vt[dim]);
            result[2][dim].push_back(at[dim]);
        }
    }

    return result;
}

std::vector<std::vector<std::vector<double>>> TrajectoryGenerator::generateCircular
(
    std::vector<double> center, // [x,y,z]
    double radius,
    double z,
    double omega, // 角速度
    double t,
    double dt,
    int direction = 1 // 1逆时针 -1顺时针
)
{
    std::vector<std::vector<std::vector<double>>> result(4);
    int dim = 3;

    for (auto &block : result)
        block.resize(dim);

    double time = 0.0;
    while (time <= t + 1e-9)
    {
        double theta = direction * omega * time;

        double x = center[0] + radius * cos(theta);
        double y = center[1] + radius * sin(theta);

        double vx = -radius * omega * sin(theta) * direction;
        double vy =  radius * omega * cos(theta) * direction;

        double ax = -radius * omega * omega * cos(theta);
        double ay = -radius * omega * omega * sin(theta);

        result[0][0].push_back(x);
        result[0][1].push_back(y);
        result[0][2].push_back(z);

        result[1][0].push_back(vx);
        result[1][1].push_back(vy);
        result[1][2].push_back(0);

        result[2][0].push_back(ax);
        result[2][1].push_back(ay);
        result[2][2].push_back(0);

        for(int d=0;d<dim;++d)
            result[3][d].push_back(time);

        time += dt;
    }

    return result;
}

std::vector<std::vector<std::vector<double>>> TrajectoryGenerator::generateLinearSCurve(
    std::vector<double> p0,
    std::vector<double> pt,
    double vmax,
    double acc,
    double dec,
    double dt)
{
    int dim = p0.size();
    std::vector<std::vector<std::vector<double>>> result(4);
    for (auto &b : result) b.resize(dim);

    std::vector<double> dir(dim);
    double dist = 0;
    for(int i=0;i<dim;++i){
        dir[i]=pt[i]-p0[i];
        dist+=dir[i]*dir[i];
    }
    dist = sqrt(dist);
    for(int i=0;i<dim;++i) dir[i]/=dist;

    double ta = vmax/acc;
    double td = vmax/dec;
    double da = 0.5*acc*ta*ta;
    double dd = 0.5*dec*td*td;

    double tc = (dist - da - dd)/vmax;
    if(tc<0) tc=0;

    double total = ta+tc+td;
    double time=0;

    while(time<=total+1e-9){
        double s=0,v=0,a=0;

        if(time<ta){
            a=acc;
            v=acc*time;
            s=0.5*acc*time*time;
        }
        else if(time<ta+tc){
            a=0;
            v=vmax;
            s=da+vmax*(time-ta);
        }
        else{
            double t2=time-ta-tc;
            a=-dec;
            v=vmax-dec*t2;
            s=da+vmax*tc+vmax*t2-0.5*dec*t2*t2;
        }

        for(int i=0;i<dim;++i){
            result[0][i].push_back(p0[i]+dir[i]*s);
            result[1][i].push_back(dir[i]*v);
            result[2][i].push_back(dir[i]*a);
            result[3][i].push_back(time);
        }

        time+=dt;
    }

    return result;
}

std::vector<std::vector<std::vector<double>>> TrajectoryGenerator::generateEightShape(
    double t_start,
    double t_end,
    double dt,
    std::vector<double> p_start,   // [x y z roll pitch yaw]
    double R,
    std::vector<double> vec        // 法向量
)
{
    std::vector<std::vector<std::vector<double>>> result(4);

    int dim = 6; // xyz + rpy
    for (auto &b : result) b.resize(dim);

    double T = t_end - t_start;
    if (T <= 1e-9 || dt <= 1e-9) return {};

    // ===== 初始位姿 =====
    std::vector<double> p0(3);
    std::vector<double> eul0(3);
    for (int i = 0; i < 3; ++i) {
        p0[i] = p_start[i];
        eul0[i] = p_start[i + 3];
    }

    // ===== 法向量单位化 =====
    double norm = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
    std::vector<double> n_vec = {vec[0]/norm, vec[1]/norm, vec[2]/norm};

    // ===== 构造平面基 u v =====
    std::vector<double> tmp = (fabs(n_vec[0]) < 0.9) ? 
                             std::vector<double>{1,0,0} : 
                             std::vector<double>{0,1,0};

    // u = n × tmp
    std::vector<double> u = {
        n_vec[1]*tmp[2] - n_vec[2]*tmp[1],
        n_vec[2]*tmp[0] - n_vec[0]*tmp[2],
        n_vec[0]*tmp[1] - n_vec[1]*tmp[0]
    };

    double u_norm = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    for (auto &v : u) v /= u_norm;

    // v = n × u
    std::vector<double> v = {
        n_vec[1]*u[2] - n_vec[2]*u[1],
        n_vec[2]*u[0] - n_vec[0]*u[2],
        n_vec[0]*u[1] - n_vec[1]*u[0]
    };

    double time = t_start;

    while (time <= t_end + 1e-9)
    {
        double tau = (time - t_start) / T;

        // ===== 五次多项式 =====
        double s   = 10*pow(tau,3) - 15*pow(tau,4) + 6*pow(tau,5);
        double ds  = (30*pow(tau,2) - 60*pow(tau,3) + 30*pow(tau,4)) / T;
        double dds = (60*tau - 180*pow(tau,2) + 120*pow(tau,3)) / (T*T);

        double theta  = 2*M_PI*s;
        double dtheta = 2*M_PI*ds;
        double ddtheta= 2*M_PI*dds;

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
        double roll  = eul0[0] + 0.1*sin(theta);
        double pitch = eul0[1] + 0.1*cos(theta);
        double yaw   = 0;

        double droll  = 0.1*cos(theta)*dtheta;
        double dpitch = -0.1*sin(theta)*dtheta;
        double dyaw   = 0;

        double ddroll  = -0.1*sin(theta)*dtheta*dtheta + 0.1*cos(theta)*ddtheta;
        double ddpitch = -0.1*cos(theta)*dtheta*dtheta - 0.1*sin(theta)*ddtheta;
        double ddyaw   = 0;

        // ===== 存储 =====
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

        time += dt;
    }

    return result;
}

std::vector<std::vector<std::vector<double>>> TrajectoryGenerator::generateSineAcceleration(
    std::vector<double> p0,
    std::vector<double> dir,
    double A, double w, double phi,
    double t, double dt)
{
    int dim=p0.size();
    std::vector<std::vector<std::vector<double>>> result(4);
    for(auto &b:result) b.resize(dim);

    double time=0;
    double v=0,s=0;

    while(time<=t+1e-9){
        double a = A*sin(w*time+phi);
        v += a*dt;
        s += v*dt;

        for(int i=0;i<dim;++i){
            result[0][i].push_back(p0[i]+dir[i]*s);
            result[1][i].push_back(dir[i]*v);
            result[2][i].push_back(dir[i]*a);
            result[3][i].push_back(time);
        }

        time+=dt;
    }

    return result;
}

std::vector<std::vector<std::vector<double>>> TrajectoryGenerator::generateCubicPolynomial(
    std::vector<double> p0,
    std::vector<double> v0,
    std::vector<double> pt,
    std::vector<double> vt,
    double t,
    double dt)
{
    int dim=p0.size();
    std::vector<std::vector<std::vector<double>>> result(4);
    for(auto &b:result) b.resize(dim);

    std::vector<double> a0=p0,a1=v0,a2(dim),a3(dim);

    for(int i=0;i<dim;++i){
        a2[i]=(3*(pt[i]-p0[i]) - (2*v0[i]+vt[i])*t)/(t*t);
        a3[i]=(2*(p0[i]-pt[i]) + (v0[i]+vt[i])*t)/(t*t*t);
    }

    double time=0;
    while(time<=t+1e-9){
        for(int i=0;i<dim;++i){
            double p=a0[i]+a1[i]*time+a2[i]*time*time+a3[i]*time*time*time;
            double v=a1[i]+2*a2[i]*time+3*a3[i]*time*time;
            double a=2*a2[i]+6*a3[i]*time;

            result[0][i].push_back(p);
            result[1][i].push_back(v);
            result[2][i].push_back(a);
            result[3][i].push_back(time);
        }
        time+=dt;
    }

    return result;
}

void TrajectoryGenerator::resampleTrajectory(
    std::vector<std::vector<std::vector<std::vector<double>>>>& traj,
    double maxStep)
{
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

/*
使用说明
if (useOfflineTraj)
{
    ...
    
    // 👉 插值修复
    double step = ui->mainPosModeTargetStepTime->value();
    resampleTrajectory(traj, step);
}

*/

