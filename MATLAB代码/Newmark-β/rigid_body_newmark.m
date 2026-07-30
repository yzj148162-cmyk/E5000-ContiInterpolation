if ~exist('run_as_subsolver', 'var') || ~run_as_subsolver
    clear; clc; 
    % close all;
end
solver_timer = tic;

% 6-DOF rigid body dynamics: ZYX Euler angles + Newmark-beta method.
% Shared physical parameters, initial conditions, and wrench definition are
% stored in rigid_body_problem.m.

cfg = rigid_body_problem();
global m I_body
m = cfg.m;
I_body = cfg.I_body;

dt = cfg.dt;
t_end = cfg.t_end;
t = 0:dt:t_end;
N = length(t);

beta = cfg.newmark.beta;
gamma = cfg.newmark.gamma;
tol = cfg.newmark.tol;
max_iter = cfg.newmark.max_iter;

r0 = cfg.r0;
eul0 = cfg.eul0;
q0 = [r0; eul0];
v0 = cfg.v0;
omega_body0 = cfg.omega_body0;

B0 = B_matrix(eul0(1), eul0(2));
eul_dot0 = B0 \ omega_body0;
q_dot0 = [v0; eul_dot0];
a0 = compute_acc(q0, q_dot0, 0);

Q = zeros(6, N);
Q_dot = zeros(6, N);
A = zeros(6, N);
Q(:,1) = q0;
Q_dot(:,1) = q_dot0;
A(:,1) = a0;

q_n = q0;
q_dot_n = q_dot0;
a_n = a0;

for i = 1:N-1
    tn = t(i);
    h = t(i+1) - tn;

    q_p = q_n + h*q_dot_n + 0.5*h^2*(1 - 2*beta)*a_n;
    q_dot_p = q_dot_n + h*(1 - gamma)*a_n;

    a_guess = a_n;
    converged = false;

    for iter = 1:max_iter
        q = q_p + beta*h^2 * a_guess;
        q_dot = q_dot_p + gamma*h * a_guess;

        a_new = compute_acc(q, q_dot, tn + h);
        residual = norm(a_new - a_guess);

        if residual < tol
            converged = true;
            break;
        end

        a_guess = a_new;
    end

    if ~converged
        q = q_p + beta*h^2 * a_new;
        q_dot = q_dot_p + gamma*h * a_new;
        warning('Newmark step %d did not converge, residual = %e', i, residual);
    end

    q_n = q;
    q_dot_n = q_dot;
    a_n = a_new;

    Q(:, i+1) = q_n;
    Q_dot(:, i+1) = q_dot_n;
    A(:, i+1) = a_n;
end

energy = zeros(1, N);
for i = 1:N
    [~, ~, T, V] = compute_energy(Q(:,i), Q_dot(:,i));
    energy(i) = T + V;
end

omega_world = zeros(3, N);
alpha_world = zeros(3, N);

for i = 1:N
    phi = Q(4,i);
    theta = Q(5,i);
    psi = Q(6,i);
    eul_dot = Q_dot(4:6,i);
    eul_ddot = A(4:6,i);

    B = B_matrix(phi, theta);
    Bdot = B_dot_matrix(phi, theta, eul_dot(1), eul_dot(2));
    R = rotm_zyx(phi, theta, psi);

    omega_body = B * eul_dot;
    alpha_body = B * eul_ddot + Bdot * eul_dot;

    omega_world(:,i) = R * omega_body;
    alpha_world(:,i) = R * alpha_body;
end

if ~exist('suppress_solver_plots', 'var') || ~suppress_solver_plots
figure('Name', 'Newmark-beta 刚体运动状态与角运动');
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
title('Newmark-beta mechanical energy check');
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
fprintf('Newmark-beta 程序耗时: %.6f s\n', toc(solver_timer));
