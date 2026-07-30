function space_plot = plot_init_spaceTrj_G302(ax, pose_ee, base_g,ideal_a_g)
%% 绘制轨道网格
% 读取轨道参数
% d = para_track.track_dist;
% Lx = para_track.Lx;
% Ly = para_track.Ly;
% Lz = para_track.Lz;
% 网格线颜色
% grid_gray = [211 211 211] / 256;

% 横向网格线（y固定）
% for y = -Ly/2:d:Ly/2
%     line(ax, [-Lx/2, Lx/2], [y, y], [Lz, Lz], 'Color', grid_gray, 'LineStyle', '-', 'LineWidth', 0.8);
% end
% 
% % 纵向网格线（x固定）
% for x = -Lx/2:d:Lx/2
%     line(ax, [x, x], [-Ly/2, Ly/2], [Lz, Lz], 'Color', grid_gray, 'LineStyle', '-', 'LineWidth', 0.8);
% end
% 
hold(ax, 'on');
% view(ax, -62.8858, 21.8875);
% view(ax, 50.6285,12.4709);
% view(ax, -63.8971,15.1796);
view(ax, -38.7451, 21.2386);



%% 绘制末端轨迹
% r_ee = para_cdpr.r_ee; %末端圆平台的半径

% 绘制末端动平台
i1=1;
j1=3;
trj_circle_ee_plot(1) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
i1=3;
j1=5;
trj_circle_ee_plot(2) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
i1=5;
j1=7;
trj_circle_ee_plot(3) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
i1=7;
j1=1;
trj_circle_ee_plot(4) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
% i1=3;
% j1=8;
% trj_circle_ee_plot(5) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
% i1=2;
% j1=5;
% trj_circle_ee_plot(6) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
% i1=4;
% j1=7;
% trj_circle_ee_plot(7) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
% i1=1;
% j1=6;
% trj_circle_ee_plot(6) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);

trj_circle_ee_plot(5) = plot_cylinder_between(ax,ideal_a_g(41:43), ideal_a_g(45:47), 0.1,  'FaceColor', 'c', ...
    'FaceAlpha', 0.5, ...    % 不透明
    'EdgeColor', 'none');


i1=2;
j1=4;
trj_circle_ee_plot(6) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
i1=4;
j1=6;
trj_circle_ee_plot(7) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
i1=6;
j1=8;
trj_circle_ee_plot(8) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);
i1=8;
j1=2;
trj_circle_ee_plot(9) = plot3(ax, [ideal_a_g(4*i1-3),ideal_a_g(4*j1-3)],[ideal_a_g(4*i1-2),ideal_a_g(4*j1-2)],[ideal_a_g(4*i1-1),ideal_a_g(4*j1-1)] , 'k','LineWidth',1.5);


% trj_circle_ee_plot(25) = plot3(ax, [ideal_a_g(41),ideal_a_g(45)],[ideal_a_g(42),ideal_a_g(46)],[ideal_a_g(43),ideal_a_g(47)] , 'r','LineWidth',1.5);hold on
% 计算箭头的方向和起点（箭头从直线的终点开始）
% arrow_start = [ideal_a_g(41), ideal_a_g(41), ideal_a_g(41)];
% arrow_direction = [x(2)-x(1), y(2)-y(1), z(2)-z(1)];

% 使用 quiver3 绘制箭头
trj_circle_ee_plot(10) = quiver3(ax,ideal_a_g(33), ideal_a_g(34), ideal_a_g(35), ...
        ideal_a_g(37)-ideal_a_g(33), ideal_a_g(38)-ideal_a_g(34), ideal_a_g(39)-ideal_a_g(35), ...
        1, 'MaxHeadSize', 100, 'Color', 'r','LineWidth',1.5); % 0.1 是箭头的缩放比例
% trj_circle_ee_plot(32) = quiver3(ax,ideal_a_g(57), ideal_a_g(58), ideal_a_g(59), ...
%         ideal_a_g(57)-ideal_a_g(41), ideal_a_g(58)-ideal_a_g(42), ideal_a_g(59)-ideal_a_g(43), ...
%         1, 'MaxHeadSize', 100, 'Color', 'r','LineWidth',1.5); % 0.1 是箭头的缩放比例
% trj_circle_ee_plot(33) = quiver3(ax,ideal_a_g(61), ideal_a_g(62), ideal_a_g(63), ...
%         ideal_a_g(61)-ideal_a_g(41), ideal_a_g(62)-ideal_a_g(42), ideal_a_g(63)-ideal_a_g(43), ...
%         1, 'MaxHeadSize', 100, 'Color', 'r','LineWidth',1.5); % 0.1 是箭头的缩放比例

% trj_circle_ee_plot(25) = arrow(ax,[ideal_a_g(41),ideal_a_g(42),ideal_a_g(43)],[ideal_a_g(45),ideal_a_g(46),ideal_a_g(47)],'color', 'r','LineWidth',1.5)

% alpha = linspace(0, 2*pi, 100);
% trj_circle_ee_plot = plot3(ax, pose_ee(1)+r_ee*cos(alpha), pose_ee(2)+r_ee*sin(alpha), pose_ee(3)*ones(1,length(alpha)), 'k','LineWidth',1.5);

% 绘制末端动平台中心轨迹
trj_pose_ee_plot = plot3(ax, pose_ee(1), pose_ee(2), pose_ee(3),'--','Color',"#00CED1",'LineWidth',1.5);

%% 绘制锚点座轨迹
%{
                b1 ---------------- b4
               / |                 / |
              / b5                / b8
             /   |               /   |
            b2 ---------------- b3   |
            |    |               |   |
            b6   ·------------- b7 --·
            |   /     Zg| Yg     |  /
            |  /        |/_Xg    | /
            | /                  |/
            ·--------------------·
%}

% trj_circle_cdpr_plot(1) = plot3(ax, [base_g(1,1),base_g(1,2)],[base_g(2,1),base_g(2,2)],[base_g(3,1),base_g(3,2)] , 'k-');
% trj_circle_cdpr_plot(2) = plot3(ax, [base_g(1,1),base_g(1,4)],[base_g(2,1),base_g(2,4)],[base_g(3,1),base_g(3,4)] , 'k-');
% trj_circle_cdpr_plot(3) = plot3(ax, [base_g(1,1),base_g(1,5)],[base_g(2,1),base_g(2,5)],[base_g(3,1),base_g(3,5)] , 'k-');
% trj_circle_cdpr_plot(4) = plot3(ax, [base_g(1,5),base_g(1,6)],[base_g(2,5),base_g(2,6)],[base_g(3,5),base_g(3,6)] , 'k-');
% trj_circle_cdpr_plot(5) = plot3(ax, [base_g(1,5),base_g(1,8)],[base_g(2,5),base_g(2,8)],[base_g(3,5),base_g(3,8)] , 'k-');
% trj_circle_cdpr_plot(6) = plot3(ax, [base_g(1,2),base_g(1,3)],[base_g(2,2),base_g(2,3)],[base_g(3,2),base_g(3,3)] , 'k-');
% trj_circle_cdpr_plot(7) = plot3(ax, [base_g(1,2),base_g(1,6)],[base_g(2,2),base_g(2,6)],[base_g(3,2),base_g(3,6)] , 'k-');
% trj_circle_cdpr_plot(8) = plot3(ax, [base_g(1,6),base_g(1,7)],[base_g(2,6),base_g(2,7)],[base_g(3,6),base_g(3,7)] , 'k-');
% trj_circle_cdpr_plot(9) = plot3(ax, [base_g(1,3),base_g(1,7)],[base_g(2,3),base_g(2,7)],[base_g(3,3),base_g(3,7)] , 'k-');
% trj_circle_cdpr_plot(10) = plot3(ax, [base_g(1,3),base_g(1,4)],[base_g(2,3),base_g(2,4)],[base_g(3,3),base_g(3,4)] , 'k-');
% trj_circle_cdpr_plot(11) = plot3(ax, [base_g(1,4),base_g(1,8)],[base_g(2,4),base_g(2,8)],[base_g(3,4),base_g(3,8)] , 'k-');
% trj_circle_cdpr_plot(12) = plot3(ax, [base_g(1,7),base_g(1,8)],[base_g(2,7),base_g(2,8)],[base_g(3,7),base_g(3,8)] , 'k-');
% Lx = 4*2;
% Ly = 3.55*2;
% Lz = 3.8;

Lx = 2.72;
Ly = 2.72;
Lz = 2.6;

bg1 = [-Lx/2;-Ly/2;0];
bg2 = [Lx/2;Ly/2;Lz];
bg3 = [Lx/2;-Ly/2;0];
bg4 = [-Lx/2;Ly/2;Lz];
bg5 = [Lx/2;Ly/2;0];
bg6 = [-Lx/2;-Ly/2;Lz];
bg7 = [-Lx/2;Ly/2;0];
bg8 = [Lx/2;-Ly/2;Lz];
bg=[bg1,bg2,bg3,bg4,bg5,bg6,bg7,bg8];


trj_circle_cdpr_plot(1) = plot3(ax, [bg(1,1),bg(1,3)],[bg(2,1),bg(2,3)],[bg(3,1),bg(3,3)] , 'k-');
trj_circle_cdpr_plot(2) = plot3(ax, [bg(1,3),bg(1,5)],[bg(2,3),bg(2,5)],[bg(3,3),bg(3,5)] , 'k-');
trj_circle_cdpr_plot(3) = plot3(ax, [bg(1,5),bg(1,7)],[bg(2,5),bg(2,7)],[bg(3,5),bg(3,7)] , 'k-');
trj_circle_cdpr_plot(4) = plot3(ax, [bg(1,7),bg(1,1)],[bg(2,7),bg(2,1)],[bg(3,7),bg(3,1)] , 'k-');
trj_circle_cdpr_plot(5) = plot3(ax, [bg(1,3),bg(1,8)],[bg(2,3),bg(2,8)],[bg(3,3),bg(3,8)] , 'k-');
trj_circle_cdpr_plot(6) = plot3(ax, [bg(1,2),bg(1,5)],[bg(2,2),bg(2,5)],[bg(3,2),bg(3,5)] , 'k-');
trj_circle_cdpr_plot(7) = plot3(ax, [bg(1,4),bg(1,7)],[bg(2,4),bg(2,7)],[bg(3,4),bg(3,7)] , 'k-');
trj_circle_cdpr_plot(8) = plot3(ax, [bg(1,1),bg(1,6)],[bg(2,1),bg(2,6)],[bg(3,1),bg(3,6)] , 'k-');
trj_circle_cdpr_plot(9) = plot3(ax, [bg(1,2),bg(1,4)],[bg(2,2),bg(2,4)],[bg(3,2),bg(3,4)] , 'k-');
trj_circle_cdpr_plot(10) = plot3(ax, [bg(1,6),bg(1,4)],[bg(2,6),bg(2,4)],[bg(3,6),bg(3,4)] , 'k-');
trj_circle_cdpr_plot(11) = plot3(ax, [bg(1,6),bg(1,8)],[bg(2,6),bg(2,8)],[bg(3,6),bg(3,8)] , 'k-');
trj_circle_cdpr_plot(12) = plot3(ax, [bg(1,2),bg(1,8)],[bg(2,2),bg(2,8)],[bg(3,2),bg(3,8)] , 'k-');

% 绘制锚点座中心移动轨迹
% trj_pos_cdpr_plot = plot3(ax, pos_cdpr(1), pos_cdpr(2), Lz, 'Color',"#DC143C",'LineWidth',1.5);

%% 绘制绳索

% 
% % 绘制绳索
trj_cable_plot(1) = plot3(ax,[base_g(1,1),ideal_a_g(1)],[base_g(2,1),ideal_a_g(2)],[base_g(3,1),ideal_a_g(3)],'LineWidth',1.2);%,'Color',"#0072BD"
trj_cable_plot(2) = plot3(ax,[base_g(1,2),ideal_a_g(5)],[base_g(2,2),ideal_a_g(6)],[base_g(3,2),ideal_a_g(7)],'LineWidth',1.2);% ,'Color',"#D95319"
trj_cable_plot(3) = plot3(ax,[base_g(1,3),ideal_a_g(9)],[base_g(2,3),ideal_a_g(10)],[base_g(3,3),ideal_a_g(11)],'LineWidth',1.2); %,'Color',"#EDB120"
trj_cable_plot(4) = plot3(ax,[base_g(1,4),ideal_a_g(13)],[base_g(2,4),ideal_a_g(14)],[base_g(3,4),ideal_a_g(15)],'LineWidth',1.2); %,'Color',"#7E2F8E"
trj_cable_plot(5) = plot3(ax,[base_g(1,5),ideal_a_g(17)],[base_g(2,5),ideal_a_g(18)],[base_g(3,5),ideal_a_g(19)],'LineWidth',1.2); %,'Color',"#77AC30"
trj_cable_plot(6) = plot3(ax,[base_g(1,6),ideal_a_g(21)],[base_g(2,6),ideal_a_g(22)],[base_g(3,6),ideal_a_g(23)],'LineWidth',1.2); %,'Color',"#4DBEEE"
trj_cable_plot(7) = plot3(ax,[base_g(1,7),ideal_a_g(25)],[base_g(2,7),ideal_a_g(26)],[base_g(3,7),ideal_a_g(27)],'LineWidth',1.2); %,'Color',"#77AC30"
trj_cable_plot(8) = plot3(ax,[base_g(1,8),ideal_a_g(29)],[base_g(2,8),ideal_a_g(30)],[base_g(3,8),ideal_a_g(31)],'LineWidth',1.2); %,'Color',"#4DBEEE"

% trj_cable_plot(1) = plot3(ax,[base_g(1,1),ideal_a_g(9)],[base_g(2,1),ideal_a_g(10)],[base_g(3,1),ideal_a_g(11)],'LineWidth',1.2);%,'Color',"#0072BD"
% trj_cable_plot(2) = plot3(ax,[base_g(1,2),ideal_a_g(49)],[base_g(2,2),ideal_a_g(50)],[base_g(3,2),ideal_a_g(51)],'LineWidth',1.2);% ,'Color',"#D95319"
% trj_cable_plot(3) = plot3(ax,[base_g(1,3),ideal_a_g(17)],[base_g(2,3),ideal_a_g(18)],[base_g(3,3),ideal_a_g(19)],'LineWidth',1.2); %,'Color',"#EDB120"
% trj_cable_plot(4) = plot3(ax,[base_g(1,4),ideal_a_g(5)],[base_g(2,4),ideal_a_g(6)],[base_g(3,4),ideal_a_g(7)],'LineWidth',1.2); %,'Color',"#7E2F8E"
% trj_cable_plot(5) = plot3(ax,[base_g(1,5),ideal_a_g(25)],[base_g(2,5),ideal_a_g(26)],[base_g(3,5),ideal_a_g(27)],'LineWidth',1.2); %,'Color',"#77AC30"
% trj_cable_plot(6) = plot3(ax,[base_g(1,6),ideal_a_g(13)],[base_g(2,6),ideal_a_g(14)],[base_g(3,6),ideal_a_g(15)],'LineWidth',1.2); %,'Color',"#4DBEEE"
% trj_cable_plot(7) = plot3(ax,[base_g(1,7),ideal_a_g(53)],[base_g(2,7),ideal_a_g(54)],[base_g(3,7),ideal_a_g(55)],'LineWidth',1.2); %,'Color',"#77AC30"
% trj_cable_plot(8) = plot3(ax,[base_g(1,8),ideal_a_g(21)],[base_g(2,8),ideal_a_g(22)],[base_g(3,8),ideal_a_g(23)],'LineWidth',1.2); %,'Color',"#4DBEEE"

% [a12_down_g;a4_g;a1_g;a6_g;a3_g;a8_g;a5_g;a11_top_g;a9_top_g;a10_down_g;ao_g;ao_z_g;a2_g;a7_g];

% trj_cable_plot(1) = plot3(ax,[base_g(1,1),ideal_a_g(9)],[base_g(2,1),ideal_a_g(10)],[base_g(3,1),ideal_a_g(11)],'LineWidth',1.2);%,'Color',"#0072BD"
% trj_cable_plot(2) = plot3(ax,[base_g(1,2),ideal_a_g(29)],[base_g(2,2),ideal_a_g(30)],[base_g(3,2),ideal_a_g(31)],'LineWidth',1.2);% ,'Color',"#D95319"
% trj_cable_plot(3) = plot3(ax,[base_g(1,3),ideal_a_g(17)],[base_g(2,3),ideal_a_g(18)],[base_g(3,3),ideal_a_g(19)],'LineWidth',1.2); %,'Color',"#EDB120"
% trj_cable_plot(4) = plot3(ax,[base_g(1,4),ideal_a_g(5)],[base_g(2,4),ideal_a_g(6)],[base_g(3,4),ideal_a_g(7)],'LineWidth',1.2); %,'Color',"#7E2F8E"
% trj_cable_plot(5) = plot3(ax,[base_g(1,5),ideal_a_g(25)],[base_g(2,5),ideal_a_g(26)],[base_g(3,5),ideal_a_g(27)],'LineWidth',1.2); %,'Color',"#77AC30"
% trj_cable_plot(6) = plot3(ax,[base_g(1,6),ideal_a_g(13)],[base_g(2,6),ideal_a_g(14)],[base_g(3,6),ideal_a_g(15)],'LineWidth',1.2); %,'Color',"#4DBEEE"
% trj_cable_plot(7) = plot3(ax,[base_g(1,7),ideal_a_g(1)],[base_g(2,7),ideal_a_g(2)],[base_g(3,7),ideal_a_g(3)],'LineWidth',1.2); %,'Color',"#77AC30"
% trj_cable_plot(8) = plot3(ax,[base_g(1,8),ideal_a_g(21)],[base_g(2,8),ideal_a_g(22)],[base_g(3,8),ideal_a_g(23)],'LineWidth',1.2); %,'Color',"#4DBEEE"
% [a7_e;a4_g;a1_g;a6_g;a3_g;a8_g;a5_g;a2_e;a9_top_g;a10_down_g;ao_g;ao_z_g;a11_top_e;a12_down_e]
 

%% 打包绘图句柄
space_plot.trj_circle_ee_plot = trj_circle_ee_plot;
space_plot.trj_pose_ee_plot = trj_pose_ee_plot;
space_plot.trj_circle_cdpr_plot = trj_circle_cdpr_plot;
% space_plot.trj_pos_cdpr_plot = trj_pos_cdpr_plot;
space_plot.trj_cable_plot = trj_cable_plot;

end
