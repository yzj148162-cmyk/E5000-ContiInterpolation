function plot_trajectory(result_trj_ee, base_g, result_trj_cable,ideal_a_g)
%% 读取绘图参数
% 读取末端轨迹
t_vec = result_trj_ee.t_vec_G;
t_step = result_trj_ee.t_step;
pose_ee = result_trj_ee.pose_ee;
vel_ee = result_trj_ee.vel_ee;
acc_ee = result_trj_ee.acc_ee;
% pose_ee(4,:) = -pose_ee(4,:);
% vel_ee(4,:) = -vel_ee(4,:);
% acc_ee(4,:) = -acc_ee(4,:);
% 读取CDPR轨迹
% pos_cdpr = result_trj_cdpr.pos_cdpr;
% vel_cdpr = result_trj_cdpr.vel_cdpr;
% acc_cdpr = result_trj_cdpr.acc_cdpr;

% 读取绳索长度、绳索速度、绳索张力
cl = result_trj_cable.cl_trj;
cv = result_trj_cable.cv_trj;
cf = result_trj_cable.cf_trj;


%% 创建图窗
% 创建5x6的图窗
fig = figure('Position', [100, 100, 1600, 800], 'Color', 'w');
tiled = tiledlayout(3, 6, 'TileSpacing', 'loose', 'Padding', 'loose');

% 创建子图
ax_space = nexttile(tiled, [3,3]);    %空间轨迹
ax_pose_ee1 = nexttile(tiled, [1,1]);  %末端位姿

% ax_pos_cdpr = nexttile(tiled, [1,1]); %CDPR位置
ax_vel_ee1 = nexttile(tiled, [1,1]);   %末端速度

% ax_vel_cdpr = nexttile(tiled, [1,1]); %CDPR速度
ax_acc_ee1 = nexttile(tiled, [1,1]);   %末端加速
ax_pose_ee2 = nexttile(tiled, [1,1]);  %末端位姿
ax_vel_ee2 = nexttile(tiled, [1,1]);   %末端速度
ax_acc_ee2 = nexttile(tiled, [1,1]);   %末端加速
% ax_acc_cdpr = nexttile(tiled, [1,1]); %CDPR加速度
% ax_planar = nexttile(tiled, [3,3]);   %平面轨迹
ax_cl = nexttile(tiled, [1,1]);       %绳索长度
ax_cv = nexttile(tiled, [1,1]);       %绳索速度
ax_cf = nexttile(tiled, [1,1]);       %绳索张力

%% 初始化绘图
% 空间轨迹
space_plot = plot_init_spaceTrj(ax_space, pose_ee, base_g,ideal_a_g);
% arrow([ideal_a_g(41),ideal_a_g(42),ideal_a_g(43)],[ideal_a_g(45),ideal_a_g(46),ideal_a_g(47)],'color', 'r','LineWidth',1.5)
% 平面轨迹
% planar_plot = plot_init_planarTrj(ax_planar, pose_ee, base_g, para_cdpr, para_track);

% 末端位姿
pose_ee_plot1 = plot(ax_pose_ee1, t_vec(1), pose_ee(1:3,1),'LineWidth', 1.5);

% 末端速度
vel_ee_plot1 = plot(ax_vel_ee1, t_vec(1), vel_ee(1:3,1),'LineWidth', 1.5);

% 末端加速度
acc_ee_plot1 = plot(ax_acc_ee1, t_vec(1), acc_ee(1:3,1),'LineWidth', 1.5);

% 末端位姿
pose_ee_plot2 = plot(ax_pose_ee2, t_vec(1), pose_ee(4:6,1)*180/pi,'LineWidth', 1.5);

% 末端速度
vel_ee_plot2 = plot(ax_vel_ee2, t_vec(1), vel_ee(4:6,1)*180/pi,'LineWidth', 1.5);

% 末端加速度
acc_ee_plot2 = plot(ax_acc_ee2, t_vec(1), acc_ee(4:6,1)*180/pi,'LineWidth', 1.5);

% CDPR位置
% pos_cdpr_plot = plot(ax_pos_cdpr, t_vec(1), base_g(:,1),'LineWidth', 1.5);

% CDPR速度
% vel_cdpr_plot = plot(ax_vel_cdpr, t_vec(1), vel_cdpr(1),'LineWidth', 1.5);

% CDPR加速度
% acc_cdpr_plot = plot(ax_acc_cdpr, t_vec(1), acc_cdpr(1),'LineWidth', 1.5);

% 绳索长度
cl_plot = plot(ax_cl, t_vec(1), cl(:,1),'LineWidth', 1.5);

% 绳索速度
cv_plot = plot(ax_cv, t_vec(1), cv(:,1),'LineWidth', 1.5);

% 绳索张力
cf_plot = plot(ax_cf, t_vec(1), cf(:,1),'LineWidth', 1.5);


%% 设置子图格式
% 空间轨迹
box(ax_space, 'off'); axis(ax_space, 'equal')
xlabel(ax_space, 'X(m)'); ylabel(ax_space, 'Y(m)'); zlabel(ax_space, 'Z(m)');
set(ax_space, 'XLim', [-10.5 10.5], 'YLim', [-10.5 10.5], 'ZLim', [-0.5 11]);
% set(ax_space, 'XLim', [-3 3], 'YLim', [-3 3], 'ZLim', [-0.7 5.6]);

% 平面轨迹
% box(ax_planar, 'on'); axis(ax_planar, 'equal')
% xlabel(ax_planar, 'X(m)'); ylabel(ax_planar, 'Y(m)');

% 末端位姿
grid(ax_pose_ee1, 'on'); box(ax_pose_ee1, 'on');
title(ax_pose_ee1, '末端位置');
ylabel(ax_pose_ee1, '位置(m)');
% legend(ax_pose_ee, {'X','Y','Z'},'Location','southeast','FontSize', 6);
legend(ax_pose_ee1, {'X','Y','Z'},'Position', [0.941333333333333,0.884555555555555,0.04,0.04],'FontSize', 6);%

% 末端速度
grid(ax_vel_ee1, 'on'); box(ax_vel_ee1, 'on');
title(ax_vel_ee1, '末端速度');
ylabel(ax_vel_ee1, '速度(m/s)');

% 末端加速度
grid(ax_acc_ee1, 'on'); box(ax_acc_ee1, 'on');
title(ax_acc_ee1, '末端加速度');
xlabel(ax_acc_ee1, '时间(s)');
ylabel(ax_acc_ee1, '加速度(m/s^2)');

% 末端位姿
grid(ax_pose_ee2, 'on'); box(ax_pose_ee2, 'on');
title(ax_pose_ee2, '末端姿态');
ylabel(ax_pose_ee2, '角度(°)');
% legend(ax_pose_ee, {'X','Y','Z'},'Location','southeast','FontSize', 6);
% legend(ax_pose_ee1, {'X','Y','Z'},'Position', [0.941333333333333,0.884555555555555,0.04,0.04],'FontSize', 6);%

% 末端速度
grid(ax_vel_ee2, 'on'); box(ax_vel_ee2, 'on');
title(ax_vel_ee2, '末端角速度');
ylabel(ax_vel_ee2, '角速度(°/s)');

% 末端加速度
grid(ax_acc_ee2, 'on'); box(ax_acc_ee2, 'on');
title(ax_acc_ee2, '末端角加速度');
xlabel(ax_acc_ee2, '时间(s)');
ylabel(ax_acc_ee2, '角加速度(°/s^2)');

% CDPR位置
% grid(ax_pos_cdpr, 'on'); box(ax_pos_cdpr, 'on');
% title(ax_pos_cdpr, 'CDPR位置');
% ylabel(ax_pos_cdpr, '位置(m)');
% legend(ax_pos_cdpr, {'X','Y'},'Location','southeast','FontSize', 6);
% legend(ax_pos_cdpr, {'X','Y'}, 'Position', [0.85 0.849 0.05 0.03],'FontSize', 6);

% CDPR沿轨道的速度
% grid(ax_vel_cdpr, 'on'); box(ax_vel_cdpr, 'on');
% title(ax_vel_cdpr, 'CDPR沿轨道速度');
% ylabel(ax_vel_cdpr, '速度(m/s)');

% CDPR沿轨道的加速度
% grid(ax_acc_cdpr, 'on'); box(ax_acc_cdpr, 'on');
% title(ax_acc_cdpr, 'CDPR沿轨道加速度');
% xlabel(ax_acc_cdpr, '时间(s)');
% ylabel(ax_acc_cdpr, '加速度(m/s^2)');

% 绳索长度图
grid(ax_cl, 'on'); box(ax_cl, 'on');
ylabel(ax_cl, '长度(m)');
title(ax_cl, '绳索长度');
legend(ax_cl, '1号绳索', '2号绳索', '3号绳索', '4号绳索','5号绳索', '6号绳索', '7号绳索', '8号绳索','Position', [0.927985556356112,0.212500002980232,0.061333332518737,0.10722221924199]);

% 绳索速度图
grid(ax_cv, 'on'); box(ax_cv, 'on');
ylabel(ax_cv, '速度(m/s)');
title(ax_cv, '绳索速度');

% 绳索张力图
grid(ax_cf, 'on'); box(ax_cf, 'on');
xlabel(ax_cf, '时间(s)'); ylabel(ax_cf, '张力(N)');
title(ax_cf, '绳索张力');

%% 设置子图的坐标轴范围
% 空间轨迹图
% Lx = para_track.Lx; Ly = para_track.Ly; Lz = para_track.Lz;
% set(ax_space, 'XLim', [-Lx/2-0.2 Lx/2+0.2], 'YLim', [-Ly/2-0.2 Ly/2+0.2], 'ZLim', [0 Lz+0.5]);

% 平面轨迹图
% set(ax_planar, 'XLim', [-Lx/2 Lx/2], 'YLim', [-Ly/2 Ly/2]);

% 计算每一曲线图的最值，作为上下界
min_pose_ee = min(pose_ee,[],'all'); max_pose_ee = max(pose_ee,[],'all');
min_vel_ee = min(vel_ee,[],'all'); max_vel_ee = max(vel_ee,[],'all');
min_acc_ee = min(acc_ee,[],'all'); max_acc_ee = max(acc_ee,[],'all');
min_pos = min(pose_ee([4:6],:),[],'all'); max_pos = max(pose_ee([4:6],:),[],'all');
min_vel = min(vel_ee([4:6],:),[],'all'); max_vel = max(vel_ee([4:6],:),[],'all');
min_acc = min(acc_ee([4:6],:),[],'all'); max_acc = max(acc_ee([4:6],:),[],'all');
min_cl = min(cl,[],'all'); max_cl = max(cl,[],'all');
min_cv = min(cv,[],'all'); max_cv = max(cv,[],'all');
min_cf = min(cf,[],'all'); max_cf = max(cf,[],'all');

% 设置曲线图的坐标轴范围
set(ax_pose_ee1, 'XLim', [0, t_vec(end)], 'YLim', [min_pose_ee max_pose_ee]); 
set(ax_vel_ee1, 'XLim', [0, t_vec(end)], 'YLim', [min_vel_ee max_vel_ee]);
set(ax_acc_ee1, 'XLim', [0, t_vec(end)], 'YLim', [min_acc_ee max_acc_ee]);
set(ax_pose_ee2, 'XLim', [0, t_vec(end)],'YLim', [min_pos max_pos+0.01]*180/pi);
set(ax_vel_ee2, 'XLim', [0, t_vec(end)], 'YLim', [min_vel max_vel+0.01]*180/pi);
set(ax_acc_ee2, 'XLim', [0, t_vec(end)], 'YLim', [min_acc max_acc+0.01]*180/pi);
% set(ax_acc_cdpr, 'XLim', [0, t_vec(end)], 'YLim', [min_acc_cdpr max_acc_cdpr]);
set(ax_cl, 'XLim', [0, t_vec(end)], 'YLim', [min_cl max_cl]);
set(ax_cv, 'XLim', [0, t_vec(end)], 'YLim', [min_cv max_cv]);
set(ax_cf, 'XLim', [0, t_vec(end)], 'YLim', [min_cf max_cf]);




%% 创建视频文件
videoFile = 'trajectory_video';
v = VideoWriter(videoFile, 'MPEG-4'); %MPEG-4 (Motion JPEG AVI)
v.FrameRate = round(1/t_step); %视频帧率
open(v);

%% 更新图窗
for i = 1:size(t_vec,2)
    % 更新空间轨迹
    plot_update_spaceTrj(space_plot, pose_ee(:,i), base_g, ideal_a_g(:,i),pose_ee(:,1:i));

    % pose_ee, base_g,ideal_a_g
    
    % 更新平面轨迹
    % plot_update_planarTrj(planar_plot, pose_ee(:,i), pos_cdpr(:,i), ...
    %                       pose_ee(:,1:i), pos_cdpr(:,1:i), para_cdpr);
    
    % 更新末端位姿、速度、加速度
    for j = 1:3
        set(pose_ee_plot1(j), 'XData', t_vec(1:i), 'YData', pose_ee(j,1:i));
        set(vel_ee_plot1(j), 'XData', t_vec(1:i), 'YData', vel_ee(j,1:i));
        set(acc_ee_plot1(j), 'XData', t_vec(1:i), 'YData', acc_ee(j,1:i));
    end
    for j = 1:3
        set(pose_ee_plot2(j), 'XData', t_vec(1:i), 'YData', pose_ee(j+3,1:i)*180/pi);
        set(vel_ee_plot2(j), 'XData', t_vec(1:i), 'YData', vel_ee(j+3,1:i)*180/pi);
        set(acc_ee_plot2(j), 'XData', t_vec(1:i), 'YData', acc_ee(j+3,1:i)*180/pi);
    end
    
    % % 更新CDPR位置、速度、加速度
    % for j = 1:2
    %     set(pos_cdpr_plot(j), 'XData', t_vec(1:i), 'YData', pos_cdpr(j,1:i));
    % end
    % 
    % set(vel_cdpr_plot, 'XData', t_vec(1:i), 'YData', vel_cdpr(1:i));
    % set(acc_cdpr_plot, 'XData', t_vec(1:i), 'YData', acc_cdpr(1:i));
    
    % 更新绳索长度、张力
    for j = 1:8
        set(cl_plot(j), 'XData', t_vec(1:i), 'YData', cl(j,1:i));
        set(cv_plot(j), 'XData', t_vec(1:i), 'YData', cv(j,1:i));
        set(cf_plot(j), 'XData', t_vec(1:i), 'YData', cf(j,1:i));
    end
    
    % 捕获帧并写入视频
    frame = getframe(fig);
    writeVideo(v, frame);
    
    % 适当暂停以控制实时显示速度（不影响视频生成）
    pause(0.00001);
    
end

% 关闭视频文件
close(v);
disp(['视频已保存为: ' videoFile]);

end