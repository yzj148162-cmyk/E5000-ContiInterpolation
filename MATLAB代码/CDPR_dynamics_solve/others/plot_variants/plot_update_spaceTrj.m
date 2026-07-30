function plot_update_spaceTrj(space_plot, pose_ee, base_g,ideal_a_g,pose_ee_trj)
% 读取绘图句柄
trj_circle_ee_plot = space_plot.trj_circle_ee_plot;
trj_pose_ee_plot = space_plot.trj_pose_ee_plot;
trj_circle_cdpr_plot = space_plot.trj_circle_cdpr_plot;
% trj_pos_cdpr_plot = space_plot.trj_pos_cdpr_plot;
trj_cable_plot = space_plot.trj_cable_plot;

% 更新末端轨迹
% r_ee = para_cdpr.r_ee; %末端圆平台的半径
% alpha = linspace(0, 2*pi, 100);
% set(trj_circle_ee_plot, 'XData', pose_ee(1)+r_ee*cos(alpha), 'YData', pose_ee(2)+r_ee*sin(alpha), 'ZData', pose_ee(3)*ones(1,length(alpha)));
% set(trj_pose_ee_plot, 'XData', pose_ee_trj(1,:), 'YData', pose_ee_trj(2,:), 'ZData', pose_ee_trj(3,:));


set(trj_circle_ee_plot(1),'XData',[ideal_a_g(1),ideal_a_g(5)],'YData',[ideal_a_g(2),ideal_a_g(6)],'ZData',[ideal_a_g(3),ideal_a_g(7)]);
% set(trj_circle_ee_plot(2),'XData',[ideal_a_g(1),ideal_a_g(9)],'YData',[ideal_a_g(2),ideal_a_g(10)],'ZData',[ideal_a_g(3),ideal_a_g(11)]);
set(trj_circle_ee_plot(2),'XData',[ideal_a_g(1),ideal_a_g(25)],'YData',[ideal_a_g(2),ideal_a_g(26)],'ZData',[ideal_a_g(3),ideal_a_g(27)]);
set(trj_circle_ee_plot(3),'XData',[ideal_a_g(17),ideal_a_g(25)],'YData',[ideal_a_g(18),ideal_a_g(26)],'ZData',[ideal_a_g(19),ideal_a_g(27)]);
set(trj_circle_ee_plot(4),'XData',[ideal_a_g(17),ideal_a_g(9)],'YData',[ideal_a_g(18),ideal_a_g(10)],'ZData',[ideal_a_g(19),ideal_a_g(11)]);
set(trj_circle_ee_plot(5),'XData',[ideal_a_g(17),ideal_a_g(21)],'YData',[ideal_a_g(18),ideal_a_g(22)],'ZData',[ideal_a_g(19),ideal_a_g(23)]);
set(trj_circle_ee_plot(6),'XData',[ideal_a_g(25),ideal_a_g(29)],'YData',[ideal_a_g(26),ideal_a_g(30)],'ZData',[ideal_a_g(27),ideal_a_g(31)]);
set(trj_circle_ee_plot(7),'XData',[ideal_a_g(9),ideal_a_g(13)],'YData',[ideal_a_g(10),ideal_a_g(14)],'ZData',[ideal_a_g(11),ideal_a_g(15)]);
set(trj_circle_ee_plot(8),'XData',[ideal_a_g(5),ideal_a_g(13)],'YData',[ideal_a_g(6),ideal_a_g(14)],'ZData',[ideal_a_g(7),ideal_a_g(15)]);
set(trj_circle_ee_plot(9),'XData',[ideal_a_g(5),ideal_a_g(29)],'YData',[ideal_a_g(6),ideal_a_g(30)],'ZData',[ideal_a_g(7),ideal_a_g(31)]);
% set(trj_circle_ee_plot(11),'XData',[ideal_a_g(21),ideal_a_g(29)],'YData',[ideal_a_g(22),ideal_a_g(30)],'ZData',[ideal_a_g(23),ideal_a_g(31)]);
set(trj_circle_ee_plot(10),'XData',[ideal_a_g(21),ideal_a_g(13)],'YData',[ideal_a_g(22),ideal_a_g(14)],'ZData',[ideal_a_g(23),ideal_a_g(15)]);
set(trj_circle_ee_plot(11),'XData',[ideal_a_g(29),ideal_a_g(49)],'YData',[ideal_a_g(30),ideal_a_g(50)],'ZData',[ideal_a_g(31),ideal_a_g(51)]);
set(trj_circle_ee_plot(12),'XData',[ideal_a_g(21),ideal_a_g(49)],'YData',[ideal_a_g(22),ideal_a_g(50)],'ZData',[ideal_a_g(23),ideal_a_g(51)]);
set(trj_circle_ee_plot(13),'XData',[ideal_a_g(1),ideal_a_g(53)],'YData',[ideal_a_g(2),ideal_a_g(54)],'ZData',[ideal_a_g(3),ideal_a_g(55)]);
set(trj_circle_ee_plot(14),'XData',[ideal_a_g(9),ideal_a_g(53)],'YData',[ideal_a_g(10),ideal_a_g(54)],'ZData',[ideal_a_g(11),ideal_a_g(55)]);
set(trj_circle_ee_plot(15),'XData',[ideal_a_g(33),ideal_a_g(49)],'YData',[ideal_a_g(34),ideal_a_g(50)],'ZData',[ideal_a_g(35),ideal_a_g(51)]);
set(trj_circle_ee_plot(16),'XData',[ideal_a_g(37),ideal_a_g(53)],'YData',[ideal_a_g(38),ideal_a_g(54)],'ZData',[ideal_a_g(39),ideal_a_g(55)]);
set(trj_circle_ee_plot(17),'XData',[ideal_a_g(25),ideal_a_g(49)],'YData',[ideal_a_g(26),ideal_a_g(50)],'ZData',[ideal_a_g(27),ideal_a_g(51)]);
set(trj_circle_ee_plot(18),'XData',[ideal_a_g(17),ideal_a_g(49)],'YData',[ideal_a_g(18),ideal_a_g(50)],'ZData',[ideal_a_g(19),ideal_a_g(51)]);
set(trj_circle_ee_plot(19),'XData',[ideal_a_g(13),ideal_a_g(53)],'YData',[ideal_a_g(14),ideal_a_g(54)],'ZData',[ideal_a_g(15),ideal_a_g(55)]);
set(trj_circle_ee_plot(20),'XData',[ideal_a_g(5),ideal_a_g(53)],'YData',[ideal_a_g(6),ideal_a_g(54)],'ZData',[ideal_a_g(7),ideal_a_g(55)] );


set(trj_circle_ee_plot(21),'XData',[ideal_a_g(1),ideal_a_g(29)],'YData',[ideal_a_g(2),ideal_a_g(30)],'ZData',[ideal_a_g(3),ideal_a_g(31)]);
set(trj_circle_ee_plot(22),'XData',[ideal_a_g(9),ideal_a_g(21)],'YData',[ideal_a_g(10),ideal_a_g(22)],'ZData',[ideal_a_g(11),ideal_a_g(23)]);
% set(trj_circle_ee_plot(15),'XData',[ideal_a_g(25),ideal_a_g(21)],'YData',[ideal_a_g(26),ideal_a_g(22)],'ZData',[ideal_a_g(27),ideal_a_g(23)] );
% set(trj_circle_ee_plot(16),'XData',[ideal_a_g(1),ideal_a_g(13)],'YData',[ideal_a_g(2),ideal_a_g(14)],'ZData',[ideal_a_g(3),ideal_a_g(15)]);
set(trj_circle_ee_plot(23),'XData',[ideal_a_g(33),ideal_a_g(5)],'YData',[ideal_a_g(34),ideal_a_g(6)],'ZData',[ideal_a_g(35),ideal_a_g(7)]);
set(trj_circle_ee_plot(24),'XData',[ideal_a_g(33),ideal_a_g(13)],'YData',[ideal_a_g(34),ideal_a_g(14)],'ZData',[ideal_a_g(35),ideal_a_g(15)]);
set(trj_circle_ee_plot(25),'XData',[ideal_a_g(33),ideal_a_g(21)],'YData',[ideal_a_g(34),ideal_a_g(22)],'ZData',[ideal_a_g(35),ideal_a_g(23)]);
set(trj_circle_ee_plot(26),'XData',[ideal_a_g(33),ideal_a_g(29)],'YData',[ideal_a_g(34),ideal_a_g(30)],'ZData',[ideal_a_g(35),ideal_a_g(31)]);
set(trj_circle_ee_plot(27),'XData',[ideal_a_g(37),ideal_a_g(9)],'YData',[ideal_a_g(38),ideal_a_g(10)],'ZData',[ideal_a_g(39),ideal_a_g(11)]);
set(trj_circle_ee_plot(28),'XData',[ideal_a_g(37),ideal_a_g(17)],'YData',[ideal_a_g(38),ideal_a_g(18)],'ZData',[ideal_a_g(39),ideal_a_g(19)]);
set(trj_circle_ee_plot(29),'XData',[ideal_a_g(37),ideal_a_g(25)],'YData',[ideal_a_g(38),ideal_a_g(26)],'ZData',[ideal_a_g(39),ideal_a_g(27)]);
set(trj_circle_ee_plot(30),'XData',[ideal_a_g(37),ideal_a_g(1)],'YData',[ideal_a_g(38),ideal_a_g(2)],'ZData',[ideal_a_g(39),ideal_a_g(3)]);
set(trj_circle_ee_plot(31),'XData',ideal_a_g(41),'YData',ideal_a_g(42),'ZData',ideal_a_g(43),...
    'UData', ideal_a_g(45)-ideal_a_g(41), 'VData', ideal_a_g(46)-ideal_a_g(42), 'WData', ideal_a_g(47)-ideal_a_g(43));


set(trj_pose_ee_plot, 'XData', pose_ee_trj(1,:), 'YData', pose_ee_trj(2,:), 'ZData', pose_ee_trj(3,:));

% 更新锚点座轨迹
% r_cdpr = para_cdpr.r_cdpr; %锚点座分布的圆半径
% Lz = para_track.Lz;
% alpha = linspace(0, 2*pi, 100);
% set(trj_circle_cdpr_plot, 'XData', pos_cdpr(1)+r_cdpr*cos(alpha), 'YData', pos_cdpr(2)+r_cdpr*sin(alpha), 'ZData', Lz*ones(1,length(alpha)));
% set(trj_pos_cdpr_plot, 'XData', pos_cdpr_trj(1,:), 'YData', pos_cdpr_trj(2,:), 'ZData', Lz*ones(1,size(pos_cdpr_trj,2)));

% 更新绳索
% bp = para_cdpr.bp; 
% ep = para_cdpr.ep;

% 将锚点座、出绳点坐标转到世界坐标系下
% bp = bp + [pos_cdpr(1:2,1);0].*ones(3,6);
% ep = ep + pose_ee(1:3,1).*ones(3,6);

for j = 1:8
    set(trj_cable_plot(j), 'XData', [base_g(1,j),ideal_a_g(4*j-3)], 'YData', [base_g(2,j),ideal_a_g(4*j-2)], 'ZData', [base_g(3,j),ideal_a_g(4*j-1)]);
end
% set(trj_cable_plot(1), 'XData',[base_g(1,1),ideal_a_g(9)] , 'YData', [base_g(2,1),ideal_a_g(10)], 'ZData',[base_g(3,1),ideal_a_g(11)] );
% set(trj_cable_plot(2), 'XData', [base_g(1,2),ideal_a_g(49)], 'YData', [base_g(2,2),ideal_a_g(50)], 'ZData',[base_g(3,2),ideal_a_g(51)] );
% set(trj_cable_plot(3), 'XData', [base_g(1,3),ideal_a_g(17)], 'YData',[base_g(2,3),ideal_a_g(18)] , 'ZData',[base_g(3,3),ideal_a_g(19)] );
% set(trj_cable_plot(4), 'XData',[base_g(1,4),ideal_a_g(5)] , 'YData',[base_g(2,4),ideal_a_g(6)] , 'ZData',[base_g(3,4),ideal_a_g(7)] );
% set(trj_cable_plot(5), 'XData',[base_g(1,5),ideal_a_g(25)] , 'YData',[base_g(2,5),ideal_a_g(26)], 'ZData',[base_g(3,5),ideal_a_g(27)] );
% set(trj_cable_plot(6), 'XData',[base_g(1,6),ideal_a_g(13)] , 'YData',[base_g(2,6),ideal_a_g(14)] , 'ZData',[base_g(3,6),ideal_a_g(15)] );
% set(trj_cable_plot(7), 'XData',[base_g(1,7),ideal_a_g(53)] , 'YData',[base_g(2,7),ideal_a_g(54)] , 'ZData',[base_g(3,7),ideal_a_g(55)] );
% set(trj_cable_plot(8), 'XData',[base_g(1,8),ideal_a_g(21)] , 'YData',[base_g(2,8),ideal_a_g(22)] , 'ZData',[base_g(3,8),ideal_a_g(23)] );


% trj_cable_plot(1) = plot3(ax,[base_g(1,1),ideal_a_g(1)],[base_g(2,1),ideal_a_g(2)],[base_g(3,1),ideal_a_g(3)],'LineWidth',1.2);%,'Color',"#0072BD"
% trj_cable_plot(2) = plot3(ax,[base_g(1,2),ideal_a_g(5)],[base_g(2,2),ideal_a_g(6)],[base_g(3,2),ideal_a_g(7)],'LineWidth',1.2);% ,'Color',"#D95319"
% trj_cable_plot(3) = plot3(ax,[base_g(1,3),ideal_a_g(9)],[base_g(2,3),ideal_a_g(10)],[base_g(3,3),ideal_a_g(11)],'LineWidth',1.2); %,'Color',"#EDB120"
% trj_cable_plot(4) = plot3(ax,[base_g(1,4),ideal_a_g(13)],[base_g(2,4),ideal_a_g(14)],[base_g(3,4),ideal_a_g(15)],'LineWidth',1.2); %,'Color',"#7E2F8E"
% trj_cable_plot(5) = plot3(ax,[base_g(1,5),ideal_a_g(17)],[base_g(2,5),ideal_a_g(18)],[base_g(3,5),ideal_a_g(19)],'LineWidth',1.2); %,'Color',"#77AC30"
% trj_cable_plot(6) = plot3(ax,[base_g(1,6),ideal_a_g(21)],[base_g(2,6),ideal_a_g(22)],[base_g(3,6),ideal_a_g(23)],'LineWidth',1.2); %,'Color',"#4DBEEE"
% trj_cable_plot(7) = plot3(ax,[base_g(1,7),ideal_a_g(25)],[base_g(2,7),ideal_a_g(26)],[base_g(3,7),ideal_a_g(27)],'LineWidth',1.2); %,'Color',"#77AC30"
% trj_cable_plot(8) = plot3(ax,[base_g(1,8),ideal_a_g(29)],[base_g(2,8),ideal_a_g(30)],[base_g(3,8),ideal_a_g(31)],'LineWidth',1.2); %,'Color',"#4DBEEE"