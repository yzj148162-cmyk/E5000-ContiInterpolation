function cfg = rigid_body_problem()
%RIGID_BODY_PROBLEM Newmark 与 RK4 共用的算例配置。
% 在这里统一修改物理参数、初始条件、时间设置和外力/力矩定义。

% 物理参数
cfg.m = 10;
cfg.g = 9.81;
cfg.I_body = diag([1, 2, 3]);

% 时间设置
cfg.dt = 0.01;
cfg.t_end = 10;

% 初始条件
cfg.r0 = [0; 0; 0];
cfg.eul0 = [0; 0; 0];
% cfg.eul0 = [0.1; 0.2; 0.3];        % [phi; theta; psi]，ZYX 欧拉角
cfg.v0 = [1; 0; 0];                % 全局坐标系平动速度
% cfg.omega_body0 = [0.1; 0.2; 0.3]; % 体坐标系角速度
% cfg.omega_body0 = [0.5; 0.6; 0.2];
cfg.omega_body0 = [0; 0; 0];

% Newmark-beta 参数
cfg.newmark.beta = 0.25;
cfg.newmark.gamma = 0.5;
cfg.newmark.tol = 1e-8;
cfg.newmark.max_iter = 50;

% RK4 输入重构阈值
cfg.rk4.F_th = inf;
cfg.rk4.M_th = inf;

% 外力/力矩定义
% F_world 表达在全局坐标系，M_body 表达在体坐标系。
cfg.force_world_fun = @default_force_world;
cfg.moment_body_fun = @default_moment_body;
end

function F_world = default_force_world(t, q, q_dot, cfg) %#ok<INUSD>
step = double(t >= 3.0);
F_var = [1.2 * cos(2*pi*0.5*t);
         0.8 * cos(2*pi*0.8*t + pi/4);
         0.6 * cos(2*pi*0.3*t + pi/2)];
F_step = [0.8; -0.6; 1] * step;
% F_step = [14; -13; 15] * step;

% 保留重力：z 向始终包含 -m*g。
F_world = F_var + F_step + [0; 0; -cfg.m * cfg.g];
% F_world = F_var + [0; 0; -cfg.m * cfg.g];
end

function M_body = default_moment_body(t, q, q_dot, cfg) %#ok<INUSD>
step = double(t >= 3.0);
M_var = [0.05 * cos(2*pi*0.4*t + pi/6);
         0.04 * cos(2*pi*0.7*t + pi/3);
         0.03 * cos(2*pi*0.9*t + pi/2)];
M_step = [0.04; -0.03; 0.02] * step;
% M_step = [0.5; -0.8; 0.3] * step;

M_body = M_var + M_step;
% M_body = M_var;
end
