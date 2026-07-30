function [pose_trj2, v_trj2, a_trj2, t_vec_G2] = plan_trj_curve2_xiaoyuan(t_start2, t_step, t_end2, p_start2, p_end2, R, vec, O_position)
% PLAN_TRJ_CURVE2_XIAOYUAN 生成空间 1/4 圆弧轨迹 (指定半径与法向量)
%
% 输入:
%   t_start2, t_end2 - 起止时间
%   t_step           - 时间步长
%   p_start2         - 起点 [x; y; z; roll; pitch; yaw]
%   p_end2           - 终点 [x; y; z; roll; pitch; yaw] 
%                      (注: 算法会优先保证 Start, R 和 vec。如果 End 与 1/4圆弧几何不符，
%                       End 可能会被微调以符合严格的圆弧方程)
%   R                - 圆弧半径
%   vec              - 圆弧所在平面的法向量 [vx; vy; vz]
%   O_position       - 目标注视点 [x; y; z]
%
% 输出:
%   pose_trj2, v_trj2, a_trj2, t_vec_G2

    %% 1. 初始化
    p_start2 = p_start2(:);
    p_end2   = p_end2(:);
    vec      = vec(:) / norm(vec); % 归一化法向量
    O_position = O_position(:);
    
    t_vec_G2 = t_start2 : t_step : t_end2;
    N = length(t_vec_G2);
    T = t_end2 - t_start2;
    
    pos_res = zeros(3, N);
    eul_res = zeros(3, N);
    
    %% 2. 几何计算 (确定圆心与基向量)
    % 目标：找到圆心 C，使得 Start->End 构成 90度圆弧
    
    % 2.1 弦向量 (Start -> End)
    chord_vec = p_end2(1:3) - p_start2(1:3);
    
    % 2.2 几何一致性检查 (可选)
    % 理论上 1/4 圆弧的弦长应为 R * sqrt(2)
    % 这里我们不报错，而是根据 Start 和 vec 重新构建准确的几何
    
    % 2.3 计算圆心 C
    % 逻辑：在平面内，圆心位于弦的中垂线上。
    % 对于 1/4 圆，圆心 C 到弦中点 M 的距离等于 弦长/2。
    % 更简单的向量构建法：
    %   C = Midpoint + 0.5 * (vec x chord_vec) 
    %   这会构建一个正方形关系 (C, Start, End, M_opp)
    
    midpoint = (p_start2(1:3) + p_end2(1:3)) / 2;
    cross_term = cross(vec, chord_vec);
    
    % 计算圆心 (假设 p_end2 位置大致正确，利用叉乘确定弯曲方向)
    Center = midpoint + 0.5 * cross_term;
    
    % 2.4 构建圆弧的局部基向量 (从圆心出发)
    % u_vec: Center -> Start
    vec_CS = p_start2(1:3) - Center;
    % 强制修正半径为 R (消除输入误差)
    vec_CS = vec_CS / norm(vec_CS) * R; 
    Center = p_start2(1:3) - vec_CS; % 反推精确的 Center
    
    u_vec = vec_CS;           % 对应 cos(0)
    
    % v_vec: Center -> End (方向)
    % v 必须垂直于 u，且在平面内
    % v = vec x u
    v_vec = cross(vec, u_vec); % 对应 sin(theta)
    
    % 此时，轨迹方程 P(theta) = Center + u*cos(theta) + v*sin(theta)
    % 当 theta=0, P=Start; 当 theta=pi/2, P=End(修正后);

    %% 3. 姿态 Offset 计算 (保持初始相对角度)
    % 3.1 初始实际姿态
    yaw_start_act   = p_start2(6);
    pitch_start_act = p_start2(5);
    roll_start_act  = p_start2(4);
    
    % 3.2 初始理想姿态 (Start -> O)
    vec_look_0 = O_position(1:3) - p_start2(1:3);
    yaw_ideal_0   = atan2(vec_look_0(2), vec_look_0(1));
    pitch_ideal_0 = atan2(vec_look_0(3), norm(vec_look_0(1:2)));
    
    % 3.3 计算偏差
    offset_yaw   = yaw_start_act - yaw_ideal_0;
    offset_pitch = pitch_start_act - pitch_ideal_0;
    
    % 归一化偏差到 [-pi, pi]
    offset_yaw = atan2(sin(offset_yaw), cos(offset_yaw));

    %% 4. 轨迹规划循环
    for i = 1:N
        tau = t_vec_G2(i) - t_start2;
        
        % 4.1 五次多项式 s: 0 -> 1
        [s, s_dot, s_ddot] = quintic_scaling(tau, T);
        
        % =======================
        % Part A: 位置 (1/4 圆弧)
        % =======================
        theta = s * (pi / 2); % 0 -> 90度
        
        % 这里的 u_vec 和 v_vec 模长均为 R
        pos_curr = Center + u_vec * cos(theta) + v_vec * sin(theta);
        pos_res(:, i) = pos_curr;
        
        % =======================
        % Part B: 姿态 (Look-At + Offset)
        % =======================
        vec_look = O_position(1:3) - pos_curr;
        
        % 理想角度
        yaw_ideal   = atan2(vec_look(2), vec_look(1));
        pitch_ideal = atan2(vec_look(3), norm(vec_look(1:2)));
        
        % 叠加偏差
        curr_yaw   = yaw_ideal + offset_yaw;
        curr_pitch = pitch_ideal + offset_pitch;
        curr_roll  = roll_start_act; % Roll 保持初始
        
        eul_res(:, i) = [curr_roll; curr_pitch; curr_yaw];
    end
    
    %% 5. 速度与加速度计算
    
    % 5.1 位置微分 (解析法，精度高)
    % dP/dt = (-u*sin + v*cos) * theta_dot
    % d2P/dt2 = (-u*cos - v*sin)*theta_dot^2 + (-u*sin + v*cos)*theta_ddot
    
    vel_res = zeros(3, N);
    acc_res = zeros(3, N);
    
    for i = 1:N
        tau = t_vec_G2(i) - t_start2;
        [s, s_dot, s_ddot] = quintic_scaling(tau, T);
        theta = s * pi/2;
        theta_dot = s_dot * pi/2;
        theta_ddot = s_ddot * pi/2;
        
        v_vec_t = (-u_vec * sin(theta) + v_vec * cos(theta)) * theta_dot;
        a_vec_t = (-u_vec * cos(theta) - v_vec * sin(theta)) * theta_dot^2 + ...
                  (-u_vec * sin(theta) + v_vec * cos(theta)) * theta_ddot;
              
        vel_res(:, i) = v_vec_t;
        acc_res(:, i) = a_vec_t;
    end
    
    % 5.2 姿态微分 (数值法)
    eul_res(3,:) = unwrap(eul_res(3,:));
    eul_res(2,:) = unwrap(eul_res(2,:));
    
    dt = t_step;
    vel_ang = gradient(eul_res, dt);
    acc_ang = gradient(vel_ang, dt);
    
    %% 6. 组合输出
    pose_trj2 = [pos_res; eul_res];
    v_trj2    = [vel_res; vel_ang];
    a_trj2    = [acc_res; acc_ang];
end

function [s, s_dot, s_ddot] = quintic_scaling(t, T)
    x = t / T;
    if x < 0, x = 0; end; if x > 1, x = 1; end
    s = 10*x^3 - 15*x^4 + 6*x^5;
    s_dot = (30*x^2 - 60*x^3 + 30*x^4) / T;
    s_ddot = (60*x - 180*x^2 + 120*x^3) / (T^2);
end