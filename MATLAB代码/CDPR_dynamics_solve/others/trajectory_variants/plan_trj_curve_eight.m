function [pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_curve_eight(t_start1, t_step, t_end1, p_start1, p_end1, R, vec)

% ==========================================
% 变姿态 8 字形轨迹
% 五次多项式时间参数
% ==========================================

%% 时间
t_vec_G1 = t_start1:t_step:t_end1;
n = length(t_vec_G1);
T = t_end1 - t_start1;

%% 初始位姿
p0 = p_start1(1:3);
eul0 = p_start1(4:6);

%% 法向量单位化
n_vec = vec(:)/norm(vec);

%% 构造平面基
if abs(dot(n_vec,[1;0;0])) < 0.9
    tmp = [1;0;0];
else
    tmp = [0;1;0];
end

u = cross(n_vec,tmp); 
u = u/norm(u);
v = cross(n_vec,u);

%% 初始化
pose_trj1 = zeros(6,n);
v_trj1    = zeros(6,n);
a_trj1    = zeros(6,n);

for i = 1:n
    
    tau = (t_vec_G1(i)-t_start1)/T;
    
    % 五次多项式
    s   = 10*tau^3 - 15*tau^4 + 6*tau^5;
    ds  = (30*tau^2 - 60*tau^3 + 30*tau^4)/T;
    dds = (60*tau - 180*tau^2 + 120*tau^3)/T^2;
    
    theta  = 2*pi*s;
    dtheta = 2*pi*ds;
    ddtheta= 2*pi*dds;
    
    %% ===== 8字形位置 =====
    x  = R*sin(theta);
    y  = R*sin(theta)*cos(theta);
    
    dx = R*cos(theta)*dtheta;
    dy = R*(cos(2*theta))*dtheta;
    
    ddx = -R*sin(theta)*dtheta^2 + R*cos(theta)*ddtheta;
    ddy = -2*R*sin(2*theta)*dtheta^2 + R*cos(2*theta)*ddtheta;
    
    pos = p0 + x*u + y*v;
    vel = dx*u + dy*v;
    acc = ddx*u + ddy*v;
    
    %% ===== 姿态变化 =====
    % 切向方向
    % yaw = atan2(dy,dx);
    
    roll  = eul0(1) + 0.1*sin(theta);
    pitch = eul0(2) + 0.1*cos(theta);
    % yaw   = eul0(3) + yaw;
    yaw = 0;
    
    pose_trj1(1:3,i) = pos;
    pose_trj1(4:6,i) = [roll; pitch; yaw];
    
    v_trj1(1:3,i) = vel;
    a_trj1(1:3,i) = acc;
    
end

%% 欧拉角速度/加速度用数值微分
v_trj1(4:6,:) = gradient(pose_trj1(4:6,:), t_step);
a_trj1(4:6,:) = gradient(v_trj1(4:6,:), t_step);

end