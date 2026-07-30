function [pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_step_line(t_start1, t_step, t_end1, p_start1, p_end1)

% ===================================================
% 四段式阶跃变加速轨迹
% +a → +2a → -a → -2a
% 起止速度为0
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

%% 基础加速度
a_const = 8*Dp/(5*T^2);

t1 = T/4;
t2 = T/2;
t3 = 3*T/4;

for i = 1:n
    
    t = t_vec_G1(i) - t_start1;
    
    if t <= t1
        % 第一段 +a
        pose_trj1(:,i) = p_start1 + 0.5*a_const*t^2;
        v_trj1(:,i)    = a_const*t;
        a_trj1(:,i)    = a_const;
        
    elseif t <= t2
        % 第二段 +2a
        p1 = p_start1 + 0.5*a_const*t1^2;
        v1 = a_const*t1;
        dt = t - t1;
        
        pose_trj1(:,i) = p1 + v1*dt + 0.5*(2*a_const)*dt^2;
        v_trj1(:,i)    = v1 + 2*a_const*dt;
        a_trj1(:,i)    = 2*a_const;
        
    elseif t <= t3
        % 第三段 -a
        % 先算到t2的状态
        p2 = p_start1 ...
             + 0.5*a_const*t1^2 ...
             + (a_const*t1)*(t2 - t1) ...
             + 0.5*(2*a_const)*(t2 - t1)^2;
             
        v2 = a_const*t1 + 2*a_const*(t2 - t1);
        
        dt = t - t2;
        
        pose_trj1(:,i) = p2 + v2*dt - 0.5*a_const*dt^2;
        v_trj1(:,i)    = v2 - a_const*dt;
        a_trj1(:,i)    = -a_const;
        
    else
        % 第四段 -2a
        % 先算到t3的状态
        p2 = p_start1 ...
             + 0.5*a_const*t1^2 ...
             + (a_const*t1)*(t2 - t1) ...
             + 0.5*(2*a_const)*(t2 - t1)^2;
        v2 = a_const*t1 + 2*a_const*(t2 - t1);
        
        p3 = p2 + v2*(t3 - t2) - 0.5*a_const*(t3 - t2)^2;
        v3 = v2 - a_const*(t3 - t2);
        
        dt = t - t3;
        
        pose_trj1(:,i) = p3 + v3*dt - 0.5*(2*a_const)*dt^2;
        v_trj1(:,i)    = v3 - 2*a_const*dt;
        a_trj1(:,i)    = -2*a_const;
    end
    
end

end