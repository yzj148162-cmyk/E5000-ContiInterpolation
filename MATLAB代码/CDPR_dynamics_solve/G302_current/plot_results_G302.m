function plot_results_G302(result_trj_ee, base_g, result_trj_cable, ideal_a_g, result_wrench)
%PLOT_RESULTS_G302 统一管理G302仿真绘图与视频输出

%% 读取数据
t_vec_G = result_trj_ee.t_vec_G;
ideal_cable_force = result_trj_cable.cf_trj;
ideal_cable_v = result_trj_cable.cv_trj;
interaction_wrench_G = result_wrench.interaction_wrench_G;

%% 动画与轨迹综合图
plot_trajectory_G302(result_trj_ee, base_g, result_trj_cable, ideal_a_g);

%% Global Frame下末端质心六维交互力
figure();
subplot(2,1,1);
plot(t_vec_G, interaction_wrench_G(1:3,:), 'linewidth', 1);
legend('Fx', 'Fy', 'Fz','FontSize',12);
xlabel('时间(s)','FontSize',12);
ylabel('力(N)','FontSize',12);
title('global frame下末端质心交互力','FontSize',12);
grid on;

subplot(2,1,2);
plot(t_vec_G, interaction_wrench_G(4:6,:), 'linewidth', 1);
legend('Mx', 'My', 'Mz','FontSize',12);
xlabel('时间(s)','FontSize',12);
ylabel('力矩(N·m)','FontSize',12);
title('global frame下末端质心交互力矩','FontSize',12);
grid on;

%% 绳索功率
P = ideal_cable_force .* abs(ideal_cable_v);
figure();
plot(t_vec_G, P , 'linewidth', 1);
legend('1号绳索', '2号绳索', '3号绳索', '4号绳索', ...
       '5号绳索', '6号绳索', '7号绳索', '8号绳索','FontSize',12);
xlabel('时间(s)','FontSize',12);
ylabel('功率(W)','FontSize',12);
grid on;

%% 总功率
P_tol = sum(P, 1);
figure();
plot(t_vec_G, P_tol ,'k', 'linewidth', 1);
xlabel('时间(s)','FontSize',12);
ylabel('总功率(W)','FontSize',12);
grid on;
end
