function [pose_trj, v_trj, a_trj, t_vec_G] = plan_trj_curve(t_start, t_step, t_end, p_start, p_end, R)
    % 三维圆弧轨迹规划函数
    % 输入:
    %   t_start: 开始时间
    %   t_step: 时间步长
    %   t_end: 结束时间
    %   p_start: 6x1矩阵 [x0; y0; z0; roll0; pitch0; yaw0]
    %   p_end: 6x1矩阵 [xf; yf; zf; rollf; pitchf; yawf]
    %   R: 圆弧半径
    % 输出:
    %   pose_trj: 位姿轨迹 [x, y, z, roll, pitch, yaw]
    %   v_trj: 速度和角速度轨迹 [vx, vy, vz, v_roll, v_pitch, v_yaw]
    %   a_trj: 加速度和角加速度轨迹 [ax, ay, az, a_roll, a_pitch, a_yaw]
    %   t_vec_G: 时间序列

    % 生成时间序列
    t_vec_G = t_start:t_step:t_end;
    n_points = length(t_vec_G);
    T = t_end - t_start;
    
    % 提取起始和结束位置
    pos_start = p_start(1:3);
    pos_end = p_end(1:3);
    
    % 提取起始和结束姿态（ZYX欧拉角）
    eul_start = p_start(4:6);
    eul_end = p_end(4:6);
    
    % 计算三维圆弧参数
    [arc_center, arc_plane, theta_start, theta_end] = calculate_3d_arc(pos_start, pos_end, R);
    
    % 计算圆弧长度
    arc_length = R * abs(theta_end - theta_start);
    
    % 使用五次多项式规划圆弧路径参数
    s_traj = fifth_order_polynomial_traj(0, arc_length, T, t_vec_G - t_start);
    
    % 生成三维圆弧轨迹
    pos_traj = generate_3d_arc_trajectory(arc_center, arc_plane, R, theta_start, theta_end, s_traj, arc_length);
    
    % 强制设置起点和终点
    pos_traj(1, :) = pos_start';
    pos_traj(end, :) = pos_end';
    
    % 生成姿态轨迹（使用五次多项式插值）
    eul_traj = generate_orientation_trajectory(eul_start, eul_end, T, t_vec_G - t_start);
    
    % 组合位置和姿态轨迹
    pose_trj = [pos_traj, eul_traj];
    
    % 计算速度和加速度
    [v_trj, a_trj] = calculate_derivatives(pose_trj, t_step);
    
    % 确保起始和结束时刻静止
    v_trj(1, :) = 0;
    v_trj(end, :) = 0;
    a_trj(1, :) = 0;
    a_trj(end, :) = 0;
    pose_trj = pose_trj';
    v_trj = v_trj';
    a_trj = a_trj';

end

function [center, plane, theta_start, theta_end] = calculate_3d_arc(start, end_, R)
    % 计算三维空间中的圆弧参数
    
    % 确保输入是列向量
    start = start(:);
    end_ = end_(:);
    
    % 计算弦向量和长度
    chord = end_ - start;
    chord_length = norm(chord);
    
    % 检查半径有效性
    if R < chord_length / 2
        error('半径R太小。最小半径应为: %.6f', chord_length / 2);
    end
    
    % 计算弦中点
    mid_point = (start + end_) / 2;
    
    % 计算圆心到弦的垂直距离
    h = sqrt(R^2 - (chord_length/2)^2);
    
    % 计算圆弧平面法向量
    % 使用默认方向，但确保与弦不平行
    default_normal = [0; 0; 1];
    if abs(dot(chord/norm(chord), default_normal)) > 0.99
        default_normal = [0; 1; 0]; % 如果弦与Z轴平行，使用Y轴
    end
    
    % 计算垂直于弦的方向（在圆弧平面内）
    perp = cross(chord, default_normal);
    perp = perp / norm(perp);
    
    % 计算圆心
    center = mid_point + h * perp;
    
    % 定义圆弧平面坐标系
    % u: 从圆心到起点的单位向量
    u = (start - center) / R;
    
    % v: 圆弧平面内垂直于u的单位向量
    v = cross(cross(u, chord), u);
    v = v / norm(v);
    
    % 平面法向量
    plane_normal = cross(u, v);
    plane = struct('u', u, 'v', v, 'normal', plane_normal, 'center', center);
    
    % 计算起点和终点的角度
    vec_start = start - center;
    vec_end = end_ - center;
    
    % 在圆弧平面坐标系中计算角度
    theta_start = 0; % 起点角度设为0
    theta_end = atan2(dot(vec_end, v), dot(vec_end, u));
    
    % 确保角度差在合理范围内
    if theta_end < 0
        theta_end = theta_end + 2*pi;
    end
    
    % 验证半径
    R_start = norm(vec_start);
    R_end = norm(vec_end);
    
    fprintf('三维圆弧参数:\n');
    fprintf('  圆心: [%.6f, %.6f, %.6f]\n', center);
    fprintf('  平面法向量: [%.6f, %.6f, %.6f]\n', plane_normal);
    fprintf('  起点角度: %.6f rad\n', theta_start);
    fprintf('  终点角度: %.6f rad\n', theta_end);
    fprintf('  起点半径: %.6f\n', R_start);
    fprintf('  终点半径: %.6f\n', R_end);
    fprintf('  弦长: %.6f\n', chord_length);
    
    if abs(R_start - R) > 1e-6 || abs(R_end - R) > 1e-6
        warning('半径不匹配: 期望半径=%.6f, 起点半径=%.6f, 终点半径=%.6f', R, R_start, R_end);
    end
end

function pos_traj = generate_3d_arc_trajectory(center, plane, R, theta_start, theta_end, s_traj, arc_length)
    % 生成三维圆弧轨迹
    
    n_points = length(s_traj);
    pos_traj = zeros(n_points, 3);
    
    % 计算总角度
    total_theta = theta_end - theta_start;
    
    % 生成轨迹点
    for i = 1:n_points
        s = s_traj(i);
        
        % 计算当前角度
        theta = theta_start + (s / arc_length) * total_theta;
        
        % 在圆弧平面坐标系中计算点坐标
        point_local = R * (cos(theta) * plane.u + sin(theta) * plane.v);
        
        % 转换到全局坐标系
        pos_traj(i, :) = (center + point_local)';
    end
    
    % 验证起点和终点
    start_error = norm(pos_traj(1,:)' - plane.center - R * plane.u);
    end_error = norm(pos_traj(end,:)' - plane.center - R * (cos(theta_end) * plane.u + sin(theta_end) * plane.v));
    
    fprintf('三维圆弧验证:\n');
    fprintf('  起点误差: %.10f\n', start_error);
    fprintf('  终点误差: %.10f\n', end_error);
    
    if start_error > 1e-10 || end_error > 1e-10
        warning('起点或终点位置不精确!');
    end
end

function s_traj = fifth_order_polynomial_traj(s0, sf, T, t_vec)
    % 五次多项式轨迹规划
    t = t_vec;
    s_traj = zeros(size(t));
    
    for i = 1:length(t)
        tau = t(i) / T;
        if tau < 0
            s_traj(i) = s0;
        elseif tau > 1
            s_traj(i) = sf;
        else
            % 五次多项式确保起点和终点速度加速度为零
            s_traj(i) = s0 + (sf - s0) * (10*tau^3 - 15*tau^4 + 6*tau^5);
        end
    end
end

function eul_traj = generate_orientation_trajectory(eul_start, eul_end, T, t_vec)
    % 生成姿态轨迹（五次多项式插值）
    n_points = length(t_vec);
    eul_traj = zeros(n_points, 3);
    
    for i = 1:3
        e0 = eul_start(i);
        ef = eul_end(i);
        
        % 处理角度跳变
        angle_diff = ef - e0;
        if angle_diff > pi
            ef = ef - 2*pi;
        elseif angle_diff < -pi
            ef = ef + 2*pi;
        end
        
        % 五次多项式插值
        for j = 1:n_points
            tau = t_vec(j) / T;
            if tau < 0
                eul_traj(j, i) = e0;
            elseif tau > 1
                eul_traj(j, i) = ef;
            else
                eul_traj(j, i) = e0 + (ef - e0) * (10*tau^3 - 15*tau^4 + 6*tau^5);
            end
        end
    end
end

function [v_trj, a_trj] = calculate_derivatives(pose_trj, dt)
    % 计算速度和加速度
    n_points = size(pose_trj, 1);
    v_trj = zeros(n_points, 6);
    a_trj = zeros(n_points, 6);
    
    % 使用中心差分法计算导数
    for i = 2:n_points-1
        % 速度
        v_trj(i, :) = (pose_trj(i+1, :) - pose_trj(i-1, :)) / (2 * dt);
        
        % 加速度
        a_trj(i, :) = (pose_trj(i+1, :) - 2 * pose_trj(i, :) + pose_trj(i-1, :)) / (dt^2);
    end
    
    % 边界处理（使用前向/后向差分）
    v_trj(1, :) = (pose_trj(2, :) - pose_trj(1, :)) / dt;
    v_trj(end, :) = (pose_trj(end, :) - pose_trj(end-1, :)) / dt;
    
    a_trj(1, :) = (pose_trj(3, :) - 2 * pose_trj(2, :) + pose_trj(1, :)) / (dt^2);
    a_trj(end, :) = (pose_trj(end, :) - 2 * pose_trj(end-1, :) + pose_trj(end-2, :)) / (dt^2);
end