function q_ddot = compute_acc(q, q_dot, t)
%COMPUTE_ACC Generalized acceleration for q = [p_W; eul_zyx].
% Translational dynamics are evaluated in world frame.
% Rotational dynamics are evaluated in body frame.
global m I_body

cfg = rigid_body_problem();
if isempty(m)
    m = cfg.m;
end
if isempty(I_body)
    I_body = cfg.I_body;
end

[F_world, M_body] = rigid_body_wrench(t, q, q_dot, cfg);
a_trans = F_world / m;

phi = q(4);
theta = q(5);
eul_dot = q_dot(4:6);

B = B_matrix(phi, theta);
omega_body = B * eul_dot;
Bdot = B_dot_matrix(phi, theta, eul_dot(1), eul_dot(2));

omega_dot_body = I_body \ (M_body - cross(omega_body, I_body * omega_body));
eul_ddot = B \ (omega_dot_body - Bdot * eul_dot);

q_ddot = [a_trans; eul_ddot];
end
