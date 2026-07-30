function plot_update_planarTrj(planar_plot, pose_ee, pos_cdpr, pose_ee_trj, pos_cdpr_trj, para_cdpr)
% 读取绘图句柄
trj_circle_ee_plot = planar_plot.trj_circle_ee_plot;
trj_pose_ee_plot = planar_plot.trj_pose_ee_plot;
trj_circle_cdpr_plot = planar_plot.trj_circle_cdpr_plot;
trj_pos_cdpr_plot = planar_plot.trj_pos_cdpr_plot;
trj_cable_plot = planar_plot.trj_cable_plot;

% 更新末端轨迹
r_ee = para_cdpr.r_ee; %末端圆平台的半径
alpha = linspace(0, 2*pi, 100);
set(trj_circle_ee_plot, 'XData', pose_ee(1)+r_ee*cos(alpha), 'YData', pose_ee(2)+r_ee*sin(alpha));
set(trj_pose_ee_plot, 'XData', pose_ee_trj(1,:), 'YData', pose_ee_trj(2,:));

% 更新锚点座轨迹
r_cdpr = para_cdpr.r_cdpr; %锚点座分布的圆半径
alpha = linspace(0, 2*pi, 100);
set(trj_circle_cdpr_plot, 'XData', pos_cdpr(1)+r_cdpr*cos(alpha), 'YData', pos_cdpr(2)+r_cdpr*sin(alpha));
set(trj_pos_cdpr_plot, 'XData', pos_cdpr_trj(1,:), 'YData', pos_cdpr_trj(2,:));

% 更新绳索
bp = para_cdpr.bp; 
ep = para_cdpr.ep;

% 将锚点座、出绳点坐标转到世界坐标系下
bp = bp(1:2,:) + pos_cdpr(1:2,1).*ones(2,6);
ep = ep(1:2,:) + pose_ee(1:2,1).*ones(2,6);

for j = 1:6
    set(trj_cable_plot(j), 'XData', [bp(1,j),ep(1,j)], 'YData', [bp(2,j),ep(2,j)]);
end

end