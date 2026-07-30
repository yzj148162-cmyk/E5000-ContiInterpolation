function [F_world, M_body] = rigid_body_wrench(t, q, q_dot, cfg)
%RIGID_BODY_WRENCH External force and moment used by both solvers.
% F_world is expressed in world frame; M_body is expressed in body frame.
if nargin < 4 || isempty(cfg)
    cfg = rigid_body_problem();
end

F_world = cfg.force_world_fun(t, q, q_dot, cfg);
M_body = cfg.moment_body_fun(t, q, q_dot, cfg);
end
