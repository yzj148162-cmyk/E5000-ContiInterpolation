function [F_world, M_body, T, V] = compute_energy(q, q_dot)
%COMPUTE_ENERGY Kinetic and potential energy for the shared problem setup.
global m I_body

cfg = rigid_body_problem();
if isempty(m)
    m = cfg.m;
end
if isempty(I_body)
    I_body = cfg.I_body;
end

[F_world, M_body] = rigid_body_wrench(0, q, q_dot, cfg);

v = q_dot(1:3);
phi = q(4);
theta = q(5);
B = B_matrix(phi, theta);
omega_body = B * q_dot(4:6);

T_trans = 0.5 * m * (v' * v);
T_rot = 0.5 * omega_body' * I_body * omega_body;
T = T_trans + T_rot;
V = m * cfg.g * q(3);
end
