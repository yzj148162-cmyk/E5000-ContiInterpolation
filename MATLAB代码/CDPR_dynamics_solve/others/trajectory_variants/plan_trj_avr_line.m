function [pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_avr_line(t_start1, t_step, t_end1, p_start1, p_end1)

% ==========================================
% 直线轨迹
% 前半段匀加速，后半段匀减速
% 起终速度为0
% ==========================================

%% 时间
t_vec_G1 = t_start1:t_step:t_end1;
n = length(t_vec_G1);
T = t_end1 - t_start1;

%% 初始化
pose_trj1 = zeros(6,n);
v_trj1    = zeros(6,n);
a_trj1    = zeros(6,n);

%% 总位移
Dp = p_end1 - p_start1;

%% 对称加速度
a_const = 4*Dp / T^2;

for i = 1:n
    
    t = t_vec_G1(i) - t_start1;
    
    if t <= T/2
        % 前半段 匀加速
        pose_trj1(:,i) = p_start1 + 0.5*a_const*t^2;
        v_trj1(:,i)    = a_const*t;
        a_trj1(:,i)    = a_const;
        
    else
        % 后半段 匀减速
        t2 = t - T/2;
        
        v_max = a_const*(T/2);
        p_mid = p_start1 + 0.5*a_const*(T/2)^2;
        
        pose_trj1(:,i) = p_mid + v_max*t2 - 0.5*a_const*t2^2;
        v_trj1(:,i)    = v_max - a_const*t2;
        a_trj1(:,i)    = -a_const;
    end
    
end

end