function dyn_out = interaction_dynamics_sample_output_G302(state_curr, wrench_sensor_raw_S, para_dyn, sensor_cfg)
%INTERACTION_DYNAMICS_SAMPLE_OUTPUT_G302 单个采样点的交互力输出整理
%   用于最后一个状态点绘图，不执行动力学积分。

pose = state_curr.pose(:);
omega_b = state_curr.omega_E(:);
mass_ee = para_dyn.mass_ee; %#ok<NASGU>
I_b = para_dyn.Iee;

if isfield(state_curr, 'R_WB')
    R_WB = state_curr.R_WB;
else
    R_WB = rotm_zyx_local_to_global_sample(pose(4:6));
end

wrench_b = transform_sensor_wrench_to_ee( ...
    wrench_sensor_raw_S, sensor_cfg.R_ES, sensor_cfg.r_ES_E, sensor_cfg.sensor_sign);

F_b = wrench_b(1:3);
M_b = wrench_b(4:6);
F_W = R_WB * F_b;
M_W = R_WB * M_b;

dyn_out.wrench_E = wrench_b;
dyn_out.force_E = F_b;
dyn_out.moment_E = M_b;
dyn_out.force_G = F_W;
dyn_out.moment_G = M_W;
dyn_out.inertial_moment_G = -R_WB * (Skew_F(omega_b) * I_b * omega_b);
dyn_out.R_WB = R_WB;
end

function R_WB = rotm_zyx_local_to_global_sample(eul)
phi = eul(1);
theta = eul(2);
psi = eul(3);

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
