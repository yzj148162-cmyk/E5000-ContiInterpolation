function [pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_3_line(t_start1, t_step, t_end1, p_start1, p_end1)

% ===================================================
% 三次多项式直线轨迹
% 起止速度=0
% ===================================================

%% 时间
t_vec_G1 = t_start1:t_step:t_end1;
n = length(t_vec_G1);
T = t_end1 - t_start1;

%% 初始化
pose_trj1 = zeros(6,n);
v_trj1    = zeros(6,n);
a_trj1    = zeros(6,n);

%% 位移
Dp = p_end1 - p_start1;

for i = 1:n
    
    t = t_vec_G1(i) - t_start1;
    tau = t / T;
    
    % 位置
    s = 3*tau^2 - 2*tau^3;
    pose_trj1(:,i) = p_start1 + Dp * s;
    
    % 速度
    ds = (6*tau - 6*tau^2)/T;
    v_trj1(:,i) = Dp * ds;
    
    % 加速度
    dds = (6 - 12*tau)/T^2;
    a_trj1(:,i) = Dp * dds;
    
end

end