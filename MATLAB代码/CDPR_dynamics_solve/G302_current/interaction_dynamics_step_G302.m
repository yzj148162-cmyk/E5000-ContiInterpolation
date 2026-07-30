function [state_curr, state_next, dyn_out] = interaction_dynamics_step_G302( ...
    state_curr, wrench_sensor_interval_S, para_dyn, sensor_cfg, t_step, integrator_cfg)
%INTERACTION_DYNAMICS_STEP_G302 单刚体动力学正解，内部使用四元数RK4积分
%   输入/输出姿态使用 R_WB，四元数 q_WB 只在本函数内部用于数值积分。

if nargin < 6 || isempty(integrator_cfg)
    integrator_cfg.input_hold = 'zoh';
end

pose = state_curr.pose(:);
vel = state_curr.vel(:);
omega_b = state_curr.omega_E(:);

mass_ee = para_dyn.mass_ee;
I_b = para_dyn.Iee;
I_b_inv = para_dyn.Iee_inv;

if isfield(state_curr, 'R_WB')
    R_WB = state_curr.R_WB;
else
    R_WB = rotm_zyx_local_to_global(pose(4:6));
end

wrench_left_b = transform_sensor_wrench_to_ee( ...
    wrench_sensor_interval_S(:,1), sensor_cfg.R_ES, sensor_cfg.r_ES_E, sensor_cfg.sensor_sign);
if size(wrench_sensor_interval_S, 2) >= 2
    wrench_right_b = transform_sensor_wrench_to_ee( ...
        wrench_sensor_interval_S(:,2), sensor_cfg.R_ES, sensor_cfg.r_ES_E, sensor_cfg.sensor_sign);
else
    wrench_right_b = wrench_left_b;
end

q_WB = rotm_to_quat_scalar_first(R_WB);
x0 = [pose(1:3); vel(1:3); q_WB; omega_b];

[x_next, k1, input_info] = rk4_step(x0, wrench_left_b, wrench_right_b, ...
    mass_ee, I_b, I_b_inv, t_step, integrator_cfg);

p_next = x_next(1:3);
v_next = x_next(4:6);
q_next = normalize_quat(x_next(7:10));
omega_b_next = x_next(11:13);
R_WB_next = quat_to_rotm_scalar_first(q_next);
eul_next = rotm_to_eulZYX(R_WB_next);

% 当前时刻输出量，用于主循环中的几何、绳速、绳力平衡和绘图。
R_WB_curr = quat_to_rotm_scalar_first(q_WB);
F_b = wrench_left_b(1:3);
M_b = wrench_left_b(4:6);
F_W = R_WB_curr * F_b;
M_W = R_WB_curr * M_b;
a_W = k1(4:6);
alpha_b = k1(11:13);
omega_W = R_WB_curr * omega_b;
alpha_W = R_WB_curr * alpha_b;

inertial_force_W = -mass_ee * a_W;
inertial_moment_W = -R_WB_curr * (I_b * alpha_b + Skew_F(omega_b) * I_b * omega_b);

state_curr.pose = [pose(1:3); rotm_to_eulZYX(R_WB_curr)];
state_curr.vel = [vel(1:3); omega_W];
state_curr.omega_E = omega_b;
state_curr.R_WB = R_WB_curr;

state_next.pose = [p_next; eul_next];
state_next.vel = [v_next; R_WB_next * omega_b_next];
state_next.omega_E = omega_b_next;
state_next.R_WB = R_WB_next;

dyn_out.wrench_E = wrench_left_b;
dyn_out.force_E = F_b;
dyn_out.moment_E = M_b;
dyn_out.force_G = F_W;
dyn_out.moment_G = M_W;
dyn_out.inertial_force_G = inertial_force_W;
dyn_out.inertial_moment_G = inertial_moment_W;
dyn_out.inertial_wrench_G = [inertial_force_W; inertial_moment_W];
dyn_out.acc_G6 = [a_W; alpha_W];
dyn_out.alpha_E = alpha_b;
dyn_out.omega_G = omega_W;
dyn_out.alpha_G = alpha_W;
dyn_out.R_WB = R_WB_curr;
dyn_out.R_WB_next = R_WB_next;
dyn_out.input_mode = input_info.mode;
dyn_out.input_force_delta_norm = input_info.force_delta_norm;
dyn_out.input_moment_delta_norm = input_info.moment_delta_norm;
end

function [x_next, k1, input_info] = rk4_step(x, wrench_left_b, wrench_right_b, ...
    mass_ee, I_b, I_b_inv, h, integrator_cfg)
[wrench_k1, wrench_k2, wrench_k3, wrench_k4, input_info] = rk4_wrench_stages( ...
    wrench_left_b, wrench_right_b, integrator_cfg);

k1 = rigid_body_rhs(x, wrench_k1, mass_ee, I_b, I_b_inv);
k2 = rigid_body_rhs(x + 0.5*h*k1, wrench_k2, mass_ee, I_b, I_b_inv);
k3 = rigid_body_rhs(x + 0.5*h*k2, wrench_k3, mass_ee, I_b, I_b_inv);
k4 = rigid_body_rhs(x + h*k3, wrench_k4, mass_ee, I_b, I_b_inv);

x_next = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
x_next(7:10) = normalize_quat(x_next(7:10));
end

function [wrench_k1, wrench_k2, wrench_k3, wrench_k4, input_info] = rk4_wrench_stages( ...
    wrench_left_b, wrench_right_b, integrator_cfg)
force_delta_norm = norm(wrench_right_b(1:3) - wrench_left_b(1:3));
moment_delta_norm = norm(wrench_right_b(4:6) - wrench_left_b(4:6));

switch lower(integrator_cfg.input_hold)
    case 'zoh'
        wrench_k1 = wrench_left_b;
        wrench_k2 = wrench_left_b;
        wrench_k3 = wrench_left_b;
        wrench_k4 = wrench_left_b;
        mode = 'ZOH';
    case 'foh'
        wrench_mid_b = 0.5 * (wrench_left_b + wrench_right_b);
        wrench_k1 = wrench_left_b;
        wrench_k2 = wrench_mid_b;
        wrench_k3 = wrench_mid_b;
        wrench_k4 = wrench_right_b;
        mode = 'FOH';
    case 'foh_jump'
        force_jump_threshold = get_cfg_scalar(integrator_cfg, 'force_jump_threshold', inf);
        moment_jump_threshold = get_cfg_scalar(integrator_cfg, 'moment_jump_threshold', inf);
        is_smooth = force_delta_norm <= force_jump_threshold && ...
            moment_delta_norm <= moment_jump_threshold;
        if is_smooth
            wrench_mid_b = 0.5 * (wrench_left_b + wrench_right_b);
            wrench_k1 = wrench_left_b;
            wrench_k2 = wrench_mid_b;
            wrench_k3 = wrench_mid_b;
            wrench_k4 = wrench_right_b;
            mode = 'FOH';
        else
            wrench_k1 = wrench_left_b;
            wrench_k2 = wrench_left_b;
            wrench_k3 = wrench_left_b;
            wrench_k4 = wrench_left_b;
            mode = 'JUMP_ZOH';
        end
    otherwise
        error('未知的输入保持方式: %s', integrator_cfg.input_hold);
end

input_info.mode = mode;
input_info.force_delta_norm = force_delta_norm;
input_info.moment_delta_norm = moment_delta_norm;
end

function value = get_cfg_scalar(cfg, field_name, default_value)
if isfield(cfg, field_name) && ~isempty(cfg.(field_name))
    value = cfg.(field_name);
else
    value = default_value;
end
end

function x_dot = rigid_body_rhs(x, wrench_b, mass_ee, I_b, I_b_inv)
p_W = x(1:3); %#ok<NASGU>
v_W = x(4:6);
q_WB = normalize_quat(x(7:10));
omega_b = x(11:13);

f_b = wrench_b(1:3);
tau_b = wrench_b(4:6);

R_WB = quat_to_rotm_scalar_first(q_WB);
p_dot = v_W;
v_dot = R_WB * f_b / mass_ee;
q_dot = 0.5 * quat_multiply(q_WB, [0; omega_b]);
omega_dot = I_b_inv * (tau_b - Skew_F(omega_b) * I_b * omega_b);

x_dot = [p_dot; v_dot; q_dot; omega_dot];
end

function q = normalize_quat(q)
q = q(:);
nq = norm(q);
if nq < eps
    error('四元数范数过小，无法归一化');
end
q = q / nq;
if q(1) < 0
    q = -q;
end
end

function qp = quat_multiply(q, p)
q = q(:);
p = p(:);
q0 = q(1);
qv = q(2:4);
p0 = p(1);
pv = p(2:4);

qp = [q0*p0 - qv.'*pv; ...
      q0*pv + p0*qv + Skew_F(qv)*pv];
end

function R = quat_to_rotm_scalar_first(q)
q = normalize_quat(q);
q0 = q(1);
qv = q(2:4);

R = (q0^2 - qv.'*qv) * eye(3) + 2 * (qv*qv.') + 2 * q0 * Skew_F(qv);
end

function q = rotm_to_quat_scalar_first(R)
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1.0) * 2;
    q0 = 0.25 * s;
    q1 = (R(3,2) - R(2,3)) / s;
    q2 = (R(1,3) - R(3,1)) / s;
    q3 = (R(2,1) - R(1,2)) / s;
else
    if R(1,1) > R(2,2) && R(1,1) > R(3,3)
        s = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
        q0 = (R(3,2) - R(2,3)) / s;
        q1 = 0.25 * s;
        q2 = (R(1,2) + R(2,1)) / s;
        q3 = (R(1,3) + R(3,1)) / s;
    elseif R(2,2) > R(3,3)
        s = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
        q0 = (R(1,3) - R(3,1)) / s;
        q1 = (R(1,2) + R(2,1)) / s;
        q2 = 0.25 * s;
        q3 = (R(2,3) + R(3,2)) / s;
    else
        s = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
        q0 = (R(2,1) - R(1,2)) / s;
        q1 = (R(1,3) + R(3,1)) / s;
        q2 = (R(2,3) + R(3,2)) / s;
        q3 = 0.25 * s;
    end
end

q = normalize_quat([q0; q1; q2; q3]);
end

function R_WB = rotm_zyx_local_to_global(eul)
phi = eul(1);
theta = eul(2);
psi = eul(3);

% 与 G302.m 中 Tr_matrix 的旋转部分保持同一 ZYX 顺序。
R_WB = [cos(psi), -sin(psi), 0; ...
        sin(psi),  cos(psi), 0; ...
               0,         0, 1] * ...
       [ cos(theta), 0, sin(theta); ...
                  0, 1,          0; ...
        -sin(theta), 0, cos(theta)] * ...
       [1,        0,         0; ...
        0, cos(phi), -sin(phi); ...
        0, sin(phi),  cos(phi)];
end

function eul = rotm_to_eulZYX(R)
theta = asin(max(-1, min(1, -R(3,1))));
ct = cos(theta);

if abs(ct) > 1e-8
    phi = atan2(R(3,2), R(3,3));
    psi = atan2(R(2,1), R(1,1));
else
    phi = 0;
    psi = atan2(-R(1,2), R(2,2));
end

eul = [phi; theta; psi];
end
