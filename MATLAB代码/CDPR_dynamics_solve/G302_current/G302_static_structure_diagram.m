clc; clear; close all;

%% --------------------- Parameters ------------------------------------ %%
para_cdpr = fun_cdpr_params_G302;
base_g = para_cdpr.base_g;
attach_e = para_cdpr.attach_e;

% 动平台放在框架中心偏下的位置，仅用于静态结构示意。
frame_center = mean(base_g(1:3,:), 2);
platform_pos = [frame_center(1); frame_center(2); 1.0];
platform_eul = [0; 0; 0];
pose_ee = [platform_pos; platform_eul];

Tr_matrix = make_transform_zyx_G302(pose_ee);
attach_g = Tr_matrix * attach_e;
a_down_e = [0; 0; -0.15 - (-0.35*2/3); 1];
a_up_e = [0; 0; 0.15 - (-0.35*2/3); 1];
a_down_g = Tr_matrix * a_down_e;
a_up_g = Tr_matrix * a_up_e;

%% --------------------- Plot ------------------------------------------ %%
figure('Name','G302框架-动平台-绳索静态示意图', ...
    'Color','w','Position',[120,80,980,760]);
hold on; axis equal;

plot_frame_box_G302(base_g);
plot_platform_G302(attach_g, a_down_g, a_up_g);
plot_cables_G302(base_g, attach_g);

scatter3(base_g(1,:), base_g(2,:), base_g(3,:), 32, ...
    [0.05 0.05 0.05], 'filled', 'MarkerEdgeColor', 'none');
scatter3(attach_g(1,:), attach_g(2,:), attach_g(3,:), 30, ...
    [0.05 0.35 0.85], 'filled', 'MarkerEdgeColor', 'none');

xlim([min(base_g(1,:))-0.15, max(base_g(1,:))+0.15]);
ylim([min(base_g(2,:))-0.15, max(base_g(2,:))+0.15]);
zlim([0, max(base_g(3,:))+0.15]);
view(42, 24);
camproj perspective;
axis off;

%% --------------------- Local Functions -------------------------------- %%
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
        '-', 'Color', [0.05 0.05 0.05], 'LineWidth', 2.0);
end
end

function plot_platform_G302(attach_g, a_down_g, a_up_g)
edge_idx = [1 3; 3 5; 5 7; 7 1; ...
            2 4; 4 6; 6 8; 8 2];

for i = 1:size(edge_idx,1)
    p1 = attach_g(1:3, edge_idx(i,1));
    p2 = attach_g(1:3, edge_idx(i,2));
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
        '-', 'Color', [0.05 0.35 0.85], 'LineWidth', 1.8);
end

plot_cylinder_between(gca, a_down_g(1:3), a_up_g(1:3), 0.1, ...
    'FaceColor', 'c', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

function plot_cables_G302(base_g, attach_g)
for i = 1:8
    plot3([attach_g(1,i), base_g(1,i)], ...
          [attach_g(2,i), base_g(2,i)], ...
          [attach_g(3,i), base_g(3,i)], ...
          '-', 'Color', [0.85 0.05 0.05], 'LineWidth', 1.3);
end
end
