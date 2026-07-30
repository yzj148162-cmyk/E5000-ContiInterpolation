function [pose_trj, v_trj, a_trj, t_vec_G] = plan_trj_curve_xiaoyuan( ...
    t_start, t_step, t_end, p_start, p_end, R, vec)

%% ================= 时间轴 =================
t_vec_G = t_start : t_step : t_end;
n = length(t_vec_G);
T = t_end - t_start;
tau = (t_vec_G - t_start) / T;   % 归一化时间

%% ============ 五次多项式时间标量 ============
s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
sd  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T;
sdd = (60*tau - 180*tau.^2 + 120*tau.^3) / T^2;

%% ================= 圆弧平面构造 =================
n_vec = vec(:);
n_vec = n_vec / norm(n_vec);   % 单位法向量

p0 = p_start(1:3);
p1 = p_end(1:3);
d = norm(p1 - p0);

if d > 2*R
    error('起点与终点距离大于直径，无法构造圆弧');
end

% 在圆弧平面内构造正交基 {e1, e2}
e1 = (p0 - p1);
e1 = e1 - dot(e1, n_vec) * n_vec;
e1 = e1 / norm(e1);

e2 = cross(n_vec, e1);

%% ================= 圆心计算 =================
mid = (p0 + p1) / 2;
h = sqrt(R^2 - (d/2)^2);

% 圆心取唯一方向（由 e2 决定）
center = mid + h * e2;

%% ================= 圆弧角参数 =================
v0 = p0 - center;
v1 = p1 - center;

theta0 = atan2(dot(v0,e2), dot(v0,e1));
theta1 = atan2(dot(v1,e2), dot(v1,e1));

% 最短圆弧
if theta1 - theta0 > pi
    theta1 = theta1 - 2*pi;
elseif theta1 - theta0 < -pi
    theta1 = theta1 + 2*pi;
end

theta   = theta0 + s   * (theta1 - theta0);
thetad  = sd  * (theta1 - theta0);
thetadd = sdd * (theta1 - theta0);

%% ================= 位置轨迹 =================
pos = center ...
    + R * (cos(theta).*e1 + sin(theta).*e2);

vel = R * ( ...
    -sin(theta).*thetad .* e1 ...
    + cos(theta).*thetad .* e2 );

acc = R * ( ...
    -cos(theta).*thetad.^2 .* e1 ...
    -sin(theta).*thetadd .* e1 ...
    -sin(theta).*thetad.^2 .* e2 ...
    +cos(theta).*thetadd .* e2 );

%% ================= 姿态轨迹（ZYX 欧拉角） =================
eul0 = p_start(4:6);
eul1 = p_end(4:6);

eul   = eul0 + (eul1 - eul0) .* s;
euld  = (eul1 - eul0) .* sd;
euldd = (eul1 - eul0) .* sdd;

%% ================= 输出 =================
pose_trj = [pos; eul];
v_trj    = [vel; euld];
a_trj    = [acc; euldd];

end
