function [pose_trj1, v_trj1, a_trj1, t_vec_G1] = plan_trj_oval_xiaoyuan(t_start1, t_step, t_end1, p_start1, p_end1, O_position)
% PLAN_TRJ_OVAL_XIAOYUAN 生成指向点O的 1/4 椭圆轨迹
%
% 几何约束:
%   1. 轨迹形状: XY平面为 1/4 椭圆，Z轴线性上升。
%   2. 初始速度方向: 沿 Y 轴 (通过椭圆圆心几何位置保证)。
%   3. 姿态约束 (Look-At): 在运动过程中，物体的"正前方"(Body X轴)始终指向 O_position。
%
% 输入:
%   p_start1     - [x; y; z; ...] (后三位角度将被忽略，由O点决定)
%   p_end1       - [x; y; z; ...] (后三位角度将被忽略，由O点决定)
%   O_position   - [x; y; z] 目标注视点坐标
%
% 输出:
%   pose_trj1 - [x, y, z, roll, pitch, yaw]
%   v_trj1    - [vx, vy, vz, wx, wy, wz]
%   a_trj1    - [ax, ay, az, alpadx, alphay, alphaz]

    %% 1. 初始化
    p_start1 = p_start1(:);
    p_end1   = p_end1(:);
    O_position = O_position(:); % 确保是列向量 (3x1)
    
    t_vec_G1 = t_start1 : t_step : t_end1;
    N = length(t_vec_G1);
    T = t_end1 - t_start1;
    
    % 预分配空间
    pos_res = zeros(3, N); % 仅位置
    vel_res = zeros(3, N);
    acc_res = zeros(3, N);
    
    eul_res = zeros(3, N); % 仅姿态 (Rad)

    %% 2. 位置轨迹规划 (XY 1/4 椭圆, 初始速度沿Y轴)
    
    % 几何构建:
    % 初始速度沿 Y -> Start点相对于Center必须在 X 轴方向上。
    % 结合 1/4 椭圆特性 -> Center 坐标为 (End.x, Start.y)
    center_xy = [p_end1(1); p_start1(2)];
    
    % 向量 A (Center -> Start): 纯 X 方向分量
    vec_A = [p_start1(1) - center_xy(1); 0; 0];
    
    % 向量 B (Center -> End): 纯 Y 方向分量
    vec_B = [0; p_end1(2) - center_xy(2); 0];
    
    % Z 轴高度
    z_start = p_start1(3);
    z_diff  = p_end1(3) - z_start;

    %% 3. 主循环：计算位置 & 计算 Look-At 姿态
    for i = 1:N
        tau = t_vec_G1(i) - t_start1;
        
        % 3.1 五次多项式时间缩放
        [s, s_dot, s_ddot] = quintic_scaling(tau, T);
        
        % 3.2 计算位置 (Position)
        % theta: 0 -> pi/2
        theta      = s * (pi / 2);
        theta_dot  = s_dot * (pi / 2);
        theta_ddot = s_ddot * (pi / 2);
        
        % XY (椭圆)
        pos_xy = [center_xy; 0] + vec_A * cos(theta) + vec_B * sin(theta);
        vel_xy = (-vec_A * sin(theta) + vec_B * cos(theta)) * theta_dot;
        
        acc_term1 = (-vec_A * cos(theta) - vec_B * sin(theta)) * (theta_dot^2);
        acc_term2 = (-vec_A * sin(theta) + vec_B * cos(theta)) * theta_ddot;
        acc_xy    = acc_term1 + acc_term2;
        
        % Z (线性插值)
        pos_z = z_start + z_diff * s;
        vel_z = z_diff * s_dot;
        acc_z = z_diff * s_ddot;
        
        % 存储位置、线速度、线加速度
        P_curr = [pos_xy(1); pos_xy(2); pos_z];
        pos_res(:, i) = P_curr;
        vel_res(:, i) = [vel_xy(1); vel_xy(2); vel_z];
        acc_res(:, i) = [acc_xy(1); acc_xy(2); acc_z];
        
        % 3.3 计算姿态 (Orientation - Look At O)
        % 向量：从当前位置 P 指向 O
        vec_look = O_position(1:3) - P_curr;
        
        % 计算 Yaw (绕 Z 轴旋转): atan2(dy, dx)
        % dx, dy 是 vec_look 在水平面的投影
        yaw_angle = atan2(vec_look(2), vec_look(1)) + pi/3.4;%
        % yaw_angle = 0;
        
        % 计算 Pitch (绕 Y 轴旋转): atan2(dz, horizontal_distance)
        % 注意：这里假设标准的航空坐标定义，抬头为正还是负取决于具体定义。
        % 此处定义：Z轴向上，抬头(看向高处的O) Pitch > 0
        horiz_dist = norm(vec_look(1:2));
        pitch_angle = -atan2(vec_look(3), horiz_dist);
        % pitch_angle = 0;
        % 计算 Roll: 默认保持水平 (0)
        roll_angle = 0;
        
        eul_res(:, i) = [roll_angle; pitch_angle; yaw_angle];
    end
    
    %% 4. 后处理：计算角速度和角加速度
    % 由于 Look-At 使得角度变化是非线性的，且涉及反三角函数，
    % 使用数值微分 (gradient) 是最稳健的方法。
    
    % 4.1 角度去卷绕 (Unwrap) 
    % 防止 -179 度跳变到 179 度导致微分产生巨大尖峰
    eul_res(3, :) = unwrap(eul_res(3, :)); 
    eul_res(2, :) = unwrap(eul_res(2, :));
    
    % 4.2 数值微分计算角速度 (rad/s)
    % gradient 计算相邻点的差分，除以 dt
    dt = t_step;
    omega_roll  = gradient(eul_res(1, :), dt);
    omega_pitch = gradient(eul_res(2, :), dt);
    omega_yaw   = gradient(eul_res(3, :), dt);
    
    vel_ang = [omega_roll; omega_pitch; omega_yaw];
    
    % 4.3 数值微分计算角加速度 (rad/s^2)
    alpha_roll  = gradient(omega_roll, dt);
    alpha_pitch = gradient(omega_pitch, dt);
    alpha_yaw   = gradient(omega_yaw, dt);
    
    acc_ang = [alpha_roll; alpha_pitch; alpha_yaw];
    
    %% 5. 组合最终输出
    pose_trj1 = [pos_res; eul_res];
    v_trj1    = [vel_res; vel_ang];
    a_trj1    = [acc_res; acc_ang];
    
end

function [s, s_dot, s_ddot] = quintic_scaling(t, T)
    x = t / T;
    if x < 0, x = 0; end; if x > 1, x = 1; end
    s = 10*x^3 - 15*x^4 + 6*x^5;
    s_dot = (30*x^2 - 60*x^3 + 30*x^4) / T;
    s_ddot = (60*x - 180*x^2 + 120*x^3) / (T^2);
end