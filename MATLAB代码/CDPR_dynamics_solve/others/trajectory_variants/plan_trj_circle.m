function [pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_circle(t_start1, t_step, t_end1, p_start1, p_end1, R, vec)

% ==========================================
% 五次多项式整圆轨迹规划
% 首尾速度加速度为0
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

%% 构造平面基向量
if abs(dot(n_vec,[1;0;0])) < 0.9
    tmp = [1;0;0];
else
    tmp = [0;1;0];
end

u = cross(n_vec, tmp);
u = u / norm(u);
v = cross(n_vec, u);

%% 圆心
center = p0 - R*u;

%% 初始化
pose_trj1 = zeros(6,n);
v_trj1    = zeros(6,n);
a_trj1    = zeros(6,n);

%% 五次多项式角度规划
for i = 1:n
    
    tau = (t_vec_G1(i) - t_start1)/T;
    
    % s(tau)
    s   = 10*tau^3 - 15*tau^4 + 6*tau^5;
    ds  = (30*tau^2 - 60*tau^3 + 30*tau^4)/T;
    dds = (60*tau - 180*tau^2 + 120*tau^3)/T^2;
    
    theta  = 2*pi*s;
    dtheta = 2*pi*ds;
    ddtheta= 2*pi*dds;
    
    % 位置
    pos = center + R*cos(theta)*u + R*sin(theta)*v;
    
    % 速度
    vel = -R*sin(theta)*dtheta*u + ...
           R*cos(theta)*dtheta*v;
       
    % 加速度
    acc = -R*cos(theta)*dtheta^2*u ...
          -R*sin(theta)*dtheta^2*u*0 ... % 保持结构清晰
          -R*sin(theta)*dtheta^2*v ...
          +(-R*sin(theta)*ddtheta)*u ...
          +( R*cos(theta)*ddtheta)*v;
    
    % 上面展开略复杂，等价标准写法如下（更清晰）:
    acc = -R*cos(theta)*dtheta^2*u ...
          -R*sin(theta)*dtheta^2*v ...
          -R*sin(theta)*ddtheta*u ...
          +R*cos(theta)*ddtheta*v;
    
    pose_trj1(1:3,i) = pos;
    v_trj1(1:3,i) = vel;
    a_trj1(1:3,i) = acc;
    
end

%% 姿态保持不变
pose_trj1(4:6,:) = repmat(eul0,1,n);
v_trj1(4:6,:)    = zeros(3,n);
a_trj1(4:6,:)    = zeros(3,n);

end