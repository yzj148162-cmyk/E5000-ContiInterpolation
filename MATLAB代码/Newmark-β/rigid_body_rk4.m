if ~exist('run_as_subsolver', 'var') || ~run_as_subsolver
    clear; clc; close all;
end
solver_timer = tic;

% 6-DOF rigid body dynamics: quaternion state + RK4 method.
% Shared physical parameters, initial conditions, and wrench definition are
% stored in rigid_body_problem.m.

cfg = rigid_body_problem();
global m I_body
m = cfg.m;
I_body = cfg.I_body;
I_body_inv = inv(I_body);

dt = cfg.dt;
t_end = cfg.t_end;
t = 0:dt:t_end;
N = length(t);

F_th = cfg.rk4.F_th;
M_th = cfg.rk4.M_th;

r0 = cfg.r0;
eul0 = cfg.eul0;
q0 = [r0; eul0];
v0 = cfg.v0;
omega_body0 = cfg.omega_body0;

B0 = B_matrix(eul0(1), eul0(2));
eul_dot0 = B0 \ omega_body0;
q_dot0 = [v0; eul_dot0];

quat0 = eul_zyx_to_quat(eul0(1), eul0(2), eul0(3));
x0 = [r0; v0; quat0; omega_body0];

X = zeros(13, N);       % [p_W; v_W; quat_WB; omega_body]
Q = zeros(6, N);
Q_dot = zeros(6, N);
A = zeros(6, N);
mode_log = strings(1, N-1);

X(:,1) = x0;
Q(:,1) = q0;
Q_dot(:,1) = q_dot0;
A(:,1) = compute_acc(q0, q_dot0, 0);

x_n = x0;

for i = 1:N-1
    tn = t(i);
    h = t(i+1) - tn;

    [q_node, q_dot_node] = output_from_state(x_n);

    U_k = sample_wrench(tn, q_node, q_dot_node, cfg);
    U_k1 = sample_wrench(tn + h, q_node, q_dot_node, cfg);

    [x_next, mode_log(i)] = rk4_step_with_input_reconstruction( ...
        x_n, U_k, U_k1, m, I_body, I_body_inv, h, F_th, M_th);

    x_n = x_next;
    X(:,i+1) = x_n;

    [Q(:,i+1), Q_dot(:,i+1)] = output_from_state(x_n);
    A(:,i+1) = compute_acc(Q(:,i+1), Q_dot(:,i+1), t(i+1));
end

energy = zeros(1, N);
for i = 1:N
    [~, ~, T, V] = compute_energy(Q(:,i), Q_dot(:,i));
    energy(i) = T + V;
end

omega_world = zeros(3, N);
alpha_world = zeros(3, N);

for i = 1:N
    quat = X(7:10,i);
    omega_body = X(11:13,i);
    R = quat_to_rotm(quat);

    phi = Q(4,i);
    theta = Q(5,i);
    eul_dot = Q_dot(4:6,i);
    eul_ddot = A(4:6,i);

    B = B_matrix(phi, theta);
    Bdot = B_dot_matrix(phi, theta, eul_dot(1), eul_dot(2));
    alpha_body = B * eul_ddot + Bdot * eul_dot;

    omega_world(:,i) = R * omega_body;
    alpha_world(:,i) = R * alpha_body;
end

if ~exist('suppress_solver_plots', 'var') || ~suppress_solver_plots
figure('Name', 'RK4 刚体运动状态与角运动');
subplot(3,2,1);
plot(t, Q(1:3,:));
xlabel('Time (s)');
ylabel('Position (m)');
legend('x','y','z');
grid on;

subplot(3,2,2);
plot(t, Q(4:6,:));
xlabel('Time (s)');
ylabel('Euler angle (rad)');
legend('\phi','\theta','\psi');
grid on;

subplot(3,2,3);
plot(t, Q_dot(1:3,:));
xlabel('Time (s)');
ylabel('Linear velocity (m/s)');
legend('v_x','v_y','v_z');
grid on;

subplot(3,2,4);
plot(t, energy);
xlabel('Time (s)');
ylabel('Total energy (J)');
title('RK4 mechanical energy check');
grid on;

subplot(3,2,5);
plot(t, omega_world);
xlabel('Time (s)');
ylabel('\omega_{world} (rad/s)');
legend('\omega_x','\omega_y','\omega_z');
grid on;

subplot(3,2,6);
plot(t, alpha_world);
xlabel('Time (s)');
ylabel('\alpha_{world} (rad/s^2)');
legend('\alpha_x','\alpha_y','\alpha_z');
grid on;
end
fprintf('RK4 程序耗时: %.6f s\n', toc(solver_timer));

function U = sample_wrench(t, q, q_dot, cfg)
    [F_world, M_body] = rigid_body_wrench(t, q, q_dot, cfg);
    U = [F_world; M_body];
end

function [x_next, mode] = rk4_step_with_input_reconstruction( ...
    x_k, U_k, U_k1, mass, I_b, I_b_inv, h, F_th, M_th)

    [U1, U2, U3, U4, mode] = select_stage_wrench(U_k, U_k1, F_th, M_th);

    K1 = rigid_body_rhs(x_k,             U1, mass, I_b, I_b_inv);
    K2 = rigid_body_rhs(x_k + 0.5*h*K1,  U2, mass, I_b, I_b_inv);
    K3 = rigid_body_rhs(x_k + 0.5*h*K2,  U3, mass, I_b, I_b_inv);
    K4 = rigid_body_rhs(x_k + h*K3,      U4, mass, I_b, I_b_inv);

    x_next = x_k + h/6 * (K1 + 2*K2 + 2*K3 + K4);
    x_next(7:10) = normalize_quat(x_next(7:10));
end

function [U1, U2, U3, U4, mode] = select_stage_wrench(U_k, U_k1, F_th, M_th)
    dU = U_k1 - U_k;
    df = norm(dU(1:3));
    dm = norm(dU(4:6));

    if df <= F_th && dm <= M_th
        U_mid = 0.5 * (U_k + U_k1);
        U1 = U_k;
        U2 = U_mid;
        U3 = U_mid;
        U4 = U_k1;
        mode = "FOH";
    else
        U1 = U_k;
        U2 = U_k;
        U3 = U_k;
        U4 = U_k;
        mode = "JUMP_ZOH";
    end
end

function x_dot = rigid_body_rhs(x, U, mass, I_b, I_b_inv)
    p_dot = x(4:6);
    quat = normalize_quat(x(7:10));
    omega_body = x(11:13);

    F_world = U(1:3);
    M_body = U(4:6);

    v_dot = F_world / mass;
    quat_dot = quat_derivative(quat, omega_body);
    omega_dot = I_b_inv * (M_body - cross(omega_body, I_b * omega_body));

    x_dot = [p_dot; v_dot; quat_dot; omega_dot];
end

function [q, q_dot] = output_from_state(x)
    p = x(1:3);
    v = x(4:6);
    quat = normalize_quat(x(7:10));
    omega_body = x(11:13);

    eul = quat_to_eul_zyx(quat);
    B = B_matrix(eul(1), eul(2));
    eul_dot = B \ omega_body;

    q = [p; eul];
    q_dot = [v; eul_dot];
end

function quat = eul_zyx_to_quat(phi, theta, psi)
    cphi = cos(phi/2);
    sphi = sin(phi/2);
    ctheta = cos(theta/2);
    stheta = sin(theta/2);
    cpsi = cos(psi/2);
    spsi = sin(psi/2);

    quat = [cpsi*ctheta*cphi + spsi*stheta*sphi;
            cpsi*ctheta*sphi - spsi*stheta*cphi;
            cpsi*stheta*cphi + spsi*ctheta*sphi;
            spsi*ctheta*cphi - cpsi*stheta*sphi];
    quat = normalize_quat(quat);
end

function eul = quat_to_eul_zyx(quat)
    R = quat_to_rotm(quat);
    theta = asin(max(-1, min(1, -R(3,1))));
    phi = atan2(R(3,2), R(3,3));
    psi = atan2(R(2,1), R(1,1));
    eul = [phi; theta; psi];
end

function R = quat_to_rotm(quat)
    quat = normalize_quat(quat);
    q0 = quat(1);
    qv = quat(2:4);
    qx = qv(1);
    qy = qv(2);
    qz = qv(3);

    qv_cross = [0, -qz, qy;
                qz, 0, -qx;
                -qy, qx, 0];

    R = (q0^2 - qv.'*qv) * eye(3) + 2 * (qv * qv.') + 2 * q0 * qv_cross;
end

function quat_dot = quat_derivative(quat, omega_body)
    quat = normalize_quat(quat);
    q0 = quat(1);
    qv = quat(2:4);

    quat_dot = 0.5 * [-qv.' * omega_body;
                      q0 * omega_body + cross(qv, omega_body)];
end

function quat = normalize_quat(quat)
    n = norm(quat);
    if n < eps
        quat = [1; 0; 0; 0];
    else
        quat = quat / n;
    end
end
