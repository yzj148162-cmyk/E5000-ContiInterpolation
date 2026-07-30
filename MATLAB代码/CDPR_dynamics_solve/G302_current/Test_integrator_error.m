clear; clc; close all;

%% 测试目的
% 单独验证一维外部力 F(t)=F0*cos(Omega*t) 对质点运动状态的影响。
% 重点比较：
% 1) 连续余弦力的解析解；
% 2) 传感器离散采样后，在每个积分区间内零阶保持(ZOH)并用RK4积分；
% 3) 延迟一拍的FOH-RK4结果。
%
% 方法3的真实含义：
% 在线系统在收到t_n采样值后，使用t_{n-1}和t_n两个采样值，
% 对上一段[t_{n-1}, t_n]做一阶保持(FOH)插值并完成动力学正解。
% 因此它不是用已知公式预测未来[t_n, t_{n+1}]，而是牺牲一拍实时性，
% 换取对上一积分区间输入力的更准确近似。

%% 参数设置
m = 1.0;                 % 质量(kg)
F0 = 8.0;                % 力幅值(N)
freq = 0.2;              % 频率(Hz)，周期为5s
Omega = 2*pi*freq;       % 角频率(rad/s)
t_end = 15.0;            % 15s = 3个完整周期
dt_list = [0.1, 0.05, 0.02, 0.01, 0.005];
dt_plot = 0.05;          % 用于绘图展示的时间步长

%% 不同步长下的终值误差对比
fprintf('dt\t\tZOH_x_end\t\tZOH_v_end\t\tFOH_x_end\t\tFOH_v_end\t\tExact_x_end\t\tExact_v_end\n');

for dt = dt_list
    result = simulate_one_dt(dt, t_end, m, F0, Omega);

    fprintf('%.5f\t% .12e\t% .12e\t% .12e\t% .12e\t% .12e\t% .12e\n', ...
        dt, ...
        result.x_zoh(end), result.v_zoh(end), ...
        result.x_foh(end), result.v_foh(end), ...
        result.x_exact(end), result.v_exact(end));
end

%% 绘制指定步长下的位置、速度、加速度曲线
result_plot = simulate_one_dt(dt_plot, t_end, m, F0, Omega);

figure('Name','一维外力积分误差测试','Color','w','Position',[100,100,1200,760]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

% 位置
nexttile;
plot(result_plot.t_state, result_plot.x_exact, 'k-', 'LineWidth', 1.5); hold on;
plot(result_plot.t_state, result_plot.x_zoh, 'b--', 'LineWidth', 1.2);
plot(result_plot.t_state, result_plot.x_foh, 'r-.', 'LineWidth', 1.2);
grid on;
xlabel('时间(s)');
ylabel('位置(m)');
title(sprintf('位置响应对比 dt = %.4f s', dt_plot));
legend('连续解析解', 'ZOH-RK4', '延迟一拍FOH-RK4', 'Location','best');

% 速度
nexttile;
plot(result_plot.t_state, result_plot.v_exact, 'k-', 'LineWidth', 1.5); hold on;
plot(result_plot.t_state, result_plot.v_zoh, 'b--', 'LineWidth', 1.2);
plot(result_plot.t_state, result_plot.v_foh, 'r-.', 'LineWidth', 1.2);
grid on;
xlabel('时间(s)');
ylabel('速度(m/s)');
title('速度响应对比');
legend('连续解析解', 'ZOH-RK4', '延迟一拍FOH-RK4', 'Location','best');

% 加速度
nexttile;
plot(result_plot.t_state, result_plot.a_exact, 'k-', 'LineWidth', 1.5); hold on;
stairs(result_plot.t_state, result_plot.a_zoh_plot, 'b--', 'LineWidth', 1.2);
plot(result_plot.t_state, result_plot.a_foh_sample, 'r-.', 'LineWidth', 1.2);
grid on;
xlabel('时间(s)');
ylabel('加速度(m/s^2)');
title('加速度输入对比');
legend('连续解析加速度', 'ZOH采样保持加速度', 'FOH端点采样加速度', 'Location','best');

%% 单步长仿真函数
function result = simulate_one_dt(dt, t_end, m, F0, Omega)
    N_total = round(t_end / dt);       % 积分区间数
    t_state = (0:N_total) * dt;        % 状态时间点，长度N_total+1
    t_input = (0:N_total-1) * dt;      % ZOH输入时间点，长度N_total

    % 连续余弦力解析解，初始条件x(0)=0, v(0)=0
    a_exact = (F0/m) * cos(Omega * t_state);
    v_exact = (F0/m) / Omega * sin(Omega * t_state);
    x_exact = (F0/m) / Omega^2 * (1 - cos(Omega * t_state));

    % 方法1：离散采样输入 + 零阶保持 + RK4。
    % 注意：在这个一维测试里，ZOH使每个区间内加速度为常数。
    % 因此RK4会退化成常加速度精确积分，结果与
    % x_next = x + v*dt + 0.5*a*dt^2, v_next = v + a*dt 完全一致。
    x_zoh = zeros(1, N_total+1);
    v_zoh = zeros(1, N_total+1);
    a_zoh_input = (F0/m) * cos(Omega * t_input);

    for k = 1:N_total
        y = [x_zoh(k); v_zoh(k)];
        a_k = a_zoh_input(k);

        k1 = rhs_1d_zoh(y, a_k);
        k2 = rhs_1d_zoh(y + 0.5*dt*k1, a_k);
        k3 = rhs_1d_zoh(y + 0.5*dt*k2, a_k);
        k4 = rhs_1d_zoh(y + dt*k3, a_k);

        y_next = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4);

        x_zoh(k+1) = y_next(1);
        v_zoh(k+1) = y_next(2);
    end

    % 方法2：延迟一拍FOH-RK4。
    % 对第k段[t_{k-1}, t_k]，需要等t_k采样值到达后，
    % 才用左右端点采样a_{k-1}, a_k完成该段积分。
    x_foh = zeros(1, N_total+1);
    v_foh = zeros(1, N_total+1);
    a_foh_sample = (F0/m) * cos(Omega * t_state);

    for k = 1:N_total
        y = [x_foh(k); v_foh(k)];
        a_left = a_foh_sample(k);
        a_right = a_foh_sample(k+1);

        k1 = rhs_1d_foh(0, y, a_left, a_right, dt);
        k2 = rhs_1d_foh(0.5*dt, y + 0.5*dt*k1, a_left, a_right, dt);
        k3 = rhs_1d_foh(0.5*dt, y + 0.5*dt*k2, a_left, a_right, dt);
        k4 = rhs_1d_foh(dt, y + dt*k3, a_left, a_right, dt);

        y_next = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4);

        x_foh(k+1) = y_next(1);
        v_foh(k+1) = y_next(2);
    end

    result.t_state = t_state;
    result.t_input = [t_input, t_state(end)];
    result.x_exact = x_exact;
    result.v_exact = v_exact;
    result.a_exact = a_exact;
    result.x_zoh = x_zoh;
    result.v_zoh = v_zoh;
    result.a_zoh_plot = [a_zoh_input, a_zoh_input(end)];
    result.x_foh = x_foh;
    result.v_foh = v_foh;
    result.a_foh_sample = a_foh_sample;
end

%% 一维ZOH动力学右端
function y_dot = rhs_1d_zoh(y, a_const)
    v = y(2);
    y_dot = [v; a_const];
end

%% 一维FOH动力学右端
function y_dot = rhs_1d_foh(tau, y, a_left, a_right, dt)
    v = y(2);
    a = a_left + (a_right - a_left) * tau / dt;
    y_dot = [v; a];
end
