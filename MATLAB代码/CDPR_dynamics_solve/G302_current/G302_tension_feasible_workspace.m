clc; clear; close all;

%% --------------------- Parameters of CDPR ---------------------------- %%
para_cdpr = fun_cdpr_params_G302;

mass_ee = para_cdpr.mass_ee;
base_g = para_cdpr.base_g;
attach_e = para_cdpr.attach_e;
ao_e = para_cdpr.ao_e;

%% --------------------- Workspace Search Settings --------------------- %%
z_range = [0.5, 2.0];
x_range = [-1.0, 1.0];
y_range = [-1.0, 1.0];
xy_step = 0.020;
num_monte_carlo = 40000; % 蒙特卡洛随机位姿点数量

% 姿态遍历范围，单位为deg；内部计算时会转换为rad。
% ax_range_deg = [-10, 10]; ax_step_deg = 5;
% ay_range_deg = [-10, 10]; ay_step_deg = 5;
% az_range_deg = [-10, 10]; az_step_deg = 5;
ax_range_deg = [-5, 5]; ax_step_deg = 5;
ay_range_deg = [-5, 5]; ay_step_deg = 5;
az_range_deg = [-45, 45]; az_step_deg = 5;

ax_range = deg2rad(ax_range_deg);
ay_range = deg2rad(ay_range_deg);
az_range = deg2rad(az_range_deg);

force_min = 10;
force_max = 500;

% 静态干涉阈值，沿用当前工程中的量级。
check_cfg.rope_clearance = 0.006;      % 绳索-绳索最小距离阈值(m)
check_cfg.platform_clearance = 0.08;   % 绳索-平台中心轴最小距离阈值(m)

use_parallel = true;

%% --------------------- Candidate Pose Sampling ------------------------ %%
% 规则网格遍历版本，后续需要时可切回。
% z_vals = [0.5, 0.8, 1.1, 1.4, 1.7, 2.0];
% x_vals = x_range(1):xy_step:x_range(2);
% y_vals = y_range(1):xy_step:y_range(2);
% ax_vals = deg2rad(ax_range_deg(1):ax_step_deg:ax_range_deg(2));
% ay_vals = deg2rad(ay_range_deg(1):ay_step_deg:ay_range_deg(2));
% az_vals = deg2rad(az_range_deg(1):az_step_deg:az_range_deg(2));
% [X_grid, Y_grid, Z_grid, AX_grid, AY_grid, AZ_grid] = ndgrid( ...
%     x_vals, y_vals, z_vals, ax_vals, ay_vals, az_vals);
% candidate_pose = [X_grid(:), Y_grid(:), Z_grid(:), ...
%     AX_grid(:), AY_grid(:), AZ_grid(:)];

rng(1); % 固定随机种子，便于复现实验结果。
candidate_pose = zeros(num_monte_carlo, 6);
candidate_pose(:,1) = x_range(1) + diff(x_range) * rand(num_monte_carlo, 1);
candidate_pose(:,2) = y_range(1) + diff(y_range) * rand(num_monte_carlo, 1);
candidate_pose(:,3) = z_range(1) + diff(z_range) * rand(num_monte_carlo, 1);
candidate_pose(:,4) = ax_range(1) + diff(ax_range) * rand(num_monte_carlo, 1);
candidate_pose(:,5) = ay_range(1) + diff(ay_range) * rand(num_monte_carlo, 1);
candidate_pose(:,6) = az_range(1) + diff(az_range) * rand(num_monte_carlo, 1);
num_candidate = size(candidate_pose, 1);

fprintf('待检查位姿点数量: %d\n', num_candidate);
fprintf('蒙特卡洛采样: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f]\n', ...
    x_range(1), x_range(2), y_range(1), y_range(2), z_range(1), z_range(2));
fprintf('姿态采样范围(deg): ax=[%.3f, %.3f], ay=[%.3f, %.3f], az=[%.3f, %.3f]\n', ...
    ax_range_deg(1), ax_range_deg(2), ay_range_deg(1), ay_range_deg(2), ...
    az_range_deg(1), az_range_deg(2));

%% --------------------- Parallel Pool --------------------------------- %%
if use_parallel
    try
        if isempty(gcp('nocreate'))
            parpool;
        end
    catch ME
        warning('G302Workspace:ParallelPoolFailed', ...
            '并行池启动失败，改用串行循环。原因: %s', ME.message);
        use_parallel = false;
    end
end

%% --------------------- Workspace Evaluation -------------------------- %%
feasible_mask = false(num_candidate, 1);
fail_code = zeros(num_candidate, 1);

tic;
if use_parallel
    parfor idx = 1:num_candidate
        pose = candidate_pose(idx, :).';
        [feasible_mask(idx), fail_code(idx)] = check_tension_feasible_pose_G302( ...
            pose, para_cdpr, mass_ee, force_min, force_max, check_cfg);
    end
else
    for idx = 1:num_candidate
        pose = candidate_pose(idx, :).';
        [feasible_mask(idx), fail_code(idx)] = check_tension_feasible_pose_G302( ...
            pose, para_cdpr, mass_ee, force_min, force_max, check_cfg);
    end
end
elapsed_time = toc;

feasible_pose = candidate_pose(feasible_mask, :);
feasible_pos = feasible_pose(:,1:3);
success_rate = size(feasible_pos, 1) / num_candidate;

fprintf('检查完成，用时 %.2f s。\n', elapsed_time);
fprintf('张力可行位姿点数量: %d / %d\n', size(feasible_pos, 1), num_candidate);
fprintf('静态位姿可行率: %.2f%%\n', success_rate * 100);
fprintf('失败统计: 绳长异常=%d, 绳索干涉=%d, 平台干涉=%d, 张力不可行=%d\n', ...
    sum(fail_code == 1), sum(fail_code == 2), sum(fail_code == 3), sum(fail_code == 4));

workspace_result.candidate_pose = candidate_pose;
workspace_result.candidate_pos = candidate_pose(:,1:3);
workspace_result.feasible_pose = feasible_pose;
workspace_result.feasible_pos = feasible_pos;
workspace_result.feasible_mask = feasible_mask;
workspace_result.fail_code = fail_code;
workspace_result.success_rate = success_rate;
workspace_result.z_range = z_range;
workspace_result.xy_step = xy_step;
workspace_result.num_monte_carlo = num_monte_carlo;
workspace_result.ax_range_deg = ax_range_deg;
workspace_result.ay_range_deg = ay_range_deg;
workspace_result.az_range_deg = az_range_deg;
workspace_result.ax_range_rad = ax_range;
workspace_result.ay_range_rad = ay_range;
workspace_result.az_range_rad = az_range;
workspace_result.force_min = force_min;
workspace_result.force_max = force_max;
workspace_result.check_cfg = check_cfg;

%% --------------------- Plotting -------------------------------------- %%
figure('Name','G302 张力可行工作空间','Color','w','Position',[100,100,980,760]);
hold on; grid on; axis equal;

if ~isempty(feasible_pos)
    scatter3(feasible_pos(:,1), feasible_pos(:,2), feasible_pos(:,3), ...
        24, feasible_pos(:,3), 'filled', 'MarkerEdgeColor', 'none');
    colormap(turbo);
    cb = colorbar;
    cb.Label.String = 'z (m)';
end

% plot_frame_box_G302(base_g);
% scatter3(base_g(1,:), base_g(2,:), base_g(3,:), 95, 'k', 'filled', ...
%     'MarkerEdgeColor', 'none', 'HandleVisibility', 'off');
% text(base_g(1,:), base_g(2,:), base_g(3,:), compose(' b%d', 1:8), ...
%     'Color', 'k', 'FontSize', 9, 'HandleVisibility', 'off');

xlim([min(base_g(1,:))-0.1, max(base_g(1,:))+0.1]);
ylim([min(base_g(2,:))-0.1, max(base_g(2,:))+0.1]);
zlim([0, max(base_g(3,:))+0.1]);
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('CDPR 张力可行工作空间');
view(45, 25);

info_text = build_workspace_info_text_G302(candidate_pose, success_rate, num_monte_carlo);
annotation('textbox', [0.58, 0.72, 0.32, 0.17], ...
    'String', info_text, ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', [0.25 0.25 0.25], ...
    'FontSize', 10, ...
    'LineWidth', 0.8);

%% --------------------- Local Functions ------------------------------- %%
function info_text = build_workspace_info_text_G302(candidate_pose, feasible_rate, num_monte_carlo)
pose_min = min(candidate_pose, [], 1);
pose_max = max(candidate_pose, [], 1);
pose_min(4:6) = rad2deg(pose_min(4:6));
pose_max(4:6) = rad2deg(pose_max(4:6));

info_text = sprintf(['随机取点个数: %d\n', ...
    '静态位姿可行率: %.2f%%\n', ...
    '六维位姿范围:\n', ...
    'x [%.2f, %.2f] m, y [%.2f, %.2f] m\n', ...
    'z [%.2f, %.2f] m\n', ...
    'ax [%.1f, %.1f] deg, ay [%.1f, %.1f] deg\n', ...
    'az [%.1f, %.1f] deg'], ...
    num_monte_carlo, feasible_rate * 100, ...
    pose_min(1), pose_max(1), pose_min(2), pose_max(2), ...
    pose_min(3), pose_max(3), ...
    pose_min(4), pose_max(4), pose_min(5), pose_max(5), ...
    pose_min(6), pose_max(6));
end

function plot_frame_box_G302(base_g)
x_min = min(base_g(1,:));
x_max = max(base_g(1,:));
y_min = min(base_g(2,:));
y_max = max(base_g(2,:));
z_min = 0;
z_max = max(base_g(3,:));

corner = [x_min, y_min, z_min;
          x_max, y_min, z_min;
          x_max, y_max, z_min;
          x_min, y_max, z_min;
          x_min, y_min, z_max;
          x_max, y_min, z_max;
          x_max, y_max, z_max;
          x_min, y_max, z_max];
edge_idx = [1 2; 2 3; 3 4; 4 1; ...
            5 6; 6 7; 7 8; 8 5; ...
            1 5; 2 6; 3 7; 4 8];

for i = 1:size(edge_idx,1)
    p1 = corner(edge_idx(i,1),:);
    p2 = corner(edge_idx(i,2),:);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
        'k-', 'LineWidth', 1.8, 'HandleVisibility', 'off');
end
end

function [is_feasible, fail_code] = check_tension_feasible_pose_G302( ...
    pose, para_cdpr, mass_ee, force_min, force_max, check_cfg)
% fail_code:
% 0 可行；1 绳长异常；2 绳索间静态干涉；3 绳索与平台干涉；4 张力不可行。

is_feasible = false;
fail_code = 0;

[segments, jaco, cable_lengths, platform_axis_seg] = build_pose_geometry_G302(pose, para_cdpr);

if any(~isfinite(cable_lengths)) || any(cable_lengths < 1e-8)
    fail_code = 1;
    return;
end

if has_static_rope_interference_G302(segments, check_cfg.rope_clearance)
    fail_code = 2;
    return;
end

if has_platform_interference_G302(segments, platform_axis_seg, check_cfg.platform_clearance)
    fail_code = 3;
    return;
end

force_ee = [0; 0; -mass_ee * 9.8];
moment_ee = zeros(3, 1);

try
    cable_force = bary_center(force_ee, moment_ee, jaco, force_min, force_max);
    tension_ok = all(isfinite(cable_force)) && ...
        all(cable_force >= force_min - 1e-6) && ...
        all(cable_force <= force_max + 1e-6);
catch
    [cable_force, exitflag] = pseudo_inverse_new(force_ee, moment_ee, jaco, force_min, force_max);
    tension_ok = exitflag ~= -1 && ~isempty(cable_force) && ...
        all(cable_force >= force_min - 1e-6) && ...
        all(cable_force <= force_max + 1e-6);
end

if ~tension_ok
    fail_code = 4;
    return;
end

is_feasible = true;
end

function [segments, jaco, cable_lengths, platform_axis_seg] = build_pose_geometry_G302(pose, para_cdpr)
base_g = para_cdpr.base_g;
attach_e = para_cdpr.attach_e;
ao_e = para_cdpr.ao_e;

Tr_matrix = make_transform_zyx_G302(pose);
attach_g = Tr_matrix * attach_e;
ao_g = Tr_matrix * ao_e;

vector_ee = attach_g(1:3,:) - ao_g(1:3);
vector_cable = base_g(1:3,:) - attach_g(1:3,:);
cable_lengths = vecnorm(vector_cable, 2, 1);
unit_vector_cable = vector_cable ./ cable_lengths;

jaco_trans = zeros(6, 8);
for i = 1:8
    jaco_trans(:,i) = [unit_vector_cable(:,i); ...
        Skew_F(vector_ee(:,i)) * unit_vector_cable(:,i)];
end
jaco = jaco_trans.';

segments = [attach_g(1:3,:).', base_g(1:3,:).'];

z_min_e = min(attach_e(3,:));
z_max_e = max(attach_e(3,:));
platform_down_g = Tr_matrix * [0; 0; z_min_e; 1];
platform_up_g = Tr_matrix * [0; 0; z_max_e; 1];
platform_axis_seg = [platform_down_g(1:3).', platform_up_g(1:3).'];
end

function Tr_matrix = make_transform_zyx_G302(pose)
x_step = pose(1);
y_step = pose(2);
z_step = pose(3);
ax_step = pose(4);
ay_step = pose(5);
az_step = pose(6);

Tr_matrix = [1, 0, 0, x_step; ...
             0, 1, 0, y_step; ...
             0, 0, 1, z_step; ...
             0, 0, 0,      1] * ...
            [cos(az_step), -sin(az_step), 0, 0; ...
             sin(az_step),  cos(az_step), 0, 0; ...
                        0,             0, 1, 0; ...
                        0,             0, 0, 1] * ...
            [ cos(ay_step), 0, sin(ay_step), 0; ...
                         0, 1,            0, 0; ...
             -sin(ay_step), 0, cos(ay_step), 0; ...
                         0, 0,            0, 1] * ...
            [1,            0,             0, 0; ...
             0, cos(ax_step), -sin(ax_step), 0; ...
             0, sin(ax_step),  cos(ax_step), 0; ...
             0,            0,             0, 1];
end

function has_interference = has_static_rope_interference_G302(segments, clearance)
has_interference = false;
n = size(segments, 1);
for i = 1:n-1
    for j = i+1:n
        dist_ij = segment_distance_G302(segments(i,:), segments(j,:));
        if dist_ij < clearance
            has_interference = true;
            return;
        end
    end
end
end

function has_interference = has_platform_interference_G302(segments, platform_axis_seg, clearance)
has_interference = false;
n = size(segments, 1);
for i = 1:n
    dist_i = segment_distance_G302(segments(i,:), platform_axis_seg);
    if dist_i < clearance
        has_interference = true;
        return;
    end
end
end

function d = segment_distance_G302(seg1, seg2)
P1 = seg1(1:3).';
Q1 = seg1(4:6).';
P2 = seg2(1:3).';
Q2 = seg2(4:6).';

u = Q1 - P1;
v = Q2 - P2;
w = P1 - P2;

a = dot(u,u);
b = dot(u,v);
c = dot(v,v);
d_dot = dot(u,w);
e = dot(v,w);
D = a*c - b*b;

sD = D;
tD = D;

if D < 1e-8
    sN = 0;
    sD = 1;
    tN = e;
    tD = c;
else
    sN = b*e - c*d_dot;
    tN = a*e - b*d_dot;
    if sN < 0
        sN = 0;
        tN = e;
        tD = c;
    elseif sN > sD
        sN = sD;
        tN = e + b;
        tD = c;
    end
end

if tN < 0
    tN = 0;
    if -d_dot < 0
        sN = 0;
    elseif -d_dot > a
        sN = sD;
    else
        sN = -d_dot;
        sD = a;
    end
elseif tN > tD
    tN = tD;
    if (-d_dot + b) < 0
        sN = 0;
    elseif (-d_dot + b) > a
        sN = sD;
    else
        sN = -d_dot + b;
        sD = a;
    end
end

if abs(sN) < 1e-8
    sc = 0;
else
    sc = sN / sD;
end

if abs(tN) < 1e-8
    tc = 0;
else
    tc = tN / tD;
end

dP = w + sc*u - tc*v;
d = norm(dP);
end
