function [pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_sin_line(t_start1, t_step, t_end1, p_start1, p_end1)

% ===================================================
% 正弦加速度平滑直线轨迹
% 起止速度=0
% 起止加速度=0
% 加速度正弦变化
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
    
    ratio = t/T;
    omega = 2*pi/T;
    
    % 位置
    pose_trj1(:,i) = p_start1 + ...
        Dp*( ratio - (1/(2*pi))*sin(2*pi*ratio) );
    
    % 速度
    v_trj1(:,i) = (Dp/T)*( 1 - cos(2*pi*ratio) );
    
    % 加速度
    a_trj1(:,i) = (2*pi*Dp/T^2)*sin(2*pi*ratio);
    
end

end