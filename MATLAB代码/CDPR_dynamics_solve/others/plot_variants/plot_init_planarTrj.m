function planar_plot = plot_init_planarTrj(ax, pose_ee, pos_cdpr, para_cdpr, para_track)
%% 绘制轨道网格
% 读取轨道参数
d = para_track.track_dist;
Lx = para_track.Lx;
Ly = para_track.Ly;
% 网格线颜色
grid_gray = [211 211 211] / 256;

% 横向网格线（y固定）
for y = -Ly/2:d:Ly/2
    line(ax, [-Lx/2, Lx/2], [y, y], 'Color', grid_gray, 'LineStyle', '-', 'LineWidth', 0.8);
end

% 纵向网格线（x固定）
for x = -Lx/2:d:Lx/2
    line(ax, [x, x], [-Ly/2, Ly/2], 'Color', grid_gray, 'LineStyle', '-', 'LineWidth', 0.8);
end

hold(ax, 'on');

%% 绘制末端轨迹
r_ee = para_cdpr.r_ee; %末端圆平台的半径

% 绘制末端动平台（圆形）
alpha = linspace(0, 2*pi, 100);
trj_circle_ee_plot = plot(ax, pose_ee(1)+r_ee*cos(alpha), pose_ee(2)+r_ee*sin(alpha),'k','LineWidth',1.5);

% 绘制末端动平台中心轨迹
trj_pose_ee_plot = plot(ax, pose_ee(1), pose_ee(2),'Color',"#00CED1",'LineWidth',1.5);

%% 绘制锚点座轨迹
r_cdpr = para_cdpr.r_cdpr; %锚点座分布的圆半径

% 绘制锚点座分布圆
alpha = linspace(0, 2*pi, 100);
trj_circle_cdpr_plot = plot(ax, pos_cdpr(1)+r_cdpr*cos(alpha), pos_cdpr(2)+r_cdpr*sin(alpha),'k--');

% 绘制锚点座中心移动轨迹
trj_pos_cdpr_plot = plot(ax, pos_cdpr(1), pos_cdpr(2),'Color',"#DC143C",'LineWidth',1.5);

%% 绘制绳索
bp = para_cdpr.bp; 
ep = para_cdpr.ep;

% 将锚点座、出绳点坐标转到世界坐标系下
bp = bp(1:2,:) + pos_cdpr(1:2,1).*ones(2,6);
ep = ep(1:2,:) + pose_ee(1:2,1).*ones(2,6);

% 绘制绳索
trj_cable_plot(1) = plot(ax,[bp(1,1),ep(1,1)],[bp(2,1),ep(2,1)],'Color',"#0072BD",'LineWidth',1.2);
trj_cable_plot(2) = plot(ax,[bp(1,2),ep(1,2)],[bp(2,2),ep(2,2)],'Color',"#D95319",'LineWidth',1.2); 
trj_cable_plot(3) = plot(ax,[bp(1,3),ep(1,3)],[bp(2,3),ep(2,3)],'Color',"#EDB120",'LineWidth',1.2); 
trj_cable_plot(4) = plot(ax,[bp(1,4),ep(1,4)],[bp(2,4),ep(2,4)],'Color',"#7E2F8E",'LineWidth',1.2); 
trj_cable_plot(5) = plot(ax,[bp(1,5),ep(1,5)],[bp(2,5),ep(2,5)],'Color',"#77AC30",'LineWidth',1.2); 
trj_cable_plot(6) = plot(ax,[bp(1,6),ep(1,6)],[bp(2,6),ep(2,6)],'Color',"#4DBEEE",'LineWidth',1.2); 


%% 打包绘图句柄
planar_plot.trj_circle_ee_plot = trj_circle_ee_plot;
planar_plot.trj_pose_ee_plot = trj_pose_ee_plot;
planar_plot.trj_circle_cdpr_plot = trj_circle_cdpr_plot;
planar_plot.trj_pos_cdpr_plot = trj_pos_cdpr_plot;
planar_plot.trj_cable_plot = trj_cable_plot;

end