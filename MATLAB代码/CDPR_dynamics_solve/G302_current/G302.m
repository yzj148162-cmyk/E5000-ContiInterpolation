clc; clear; close all;
%% --------------------- Parameters of CDPR ---------------------------- %%
para_cdpr = fun_cdpr_params_G302;

mass_ee = para_cdpr.mass_ee;
M0_ee = para_cdpr.M0_ee;
Iee = M0_ee(4:6,4:6);
base_g = para_cdpr.base_g;
attach_e = para_cdpr.attach_e;
ao_e = para_cdpr.ao_e;

b1_g = base_g(:,1);b2_g = base_g(:,2);b3_g = base_g(:,3);b4_g = base_g(:,4);
b5_g = base_g(:,5);b6_g = base_g(:,6);b7_g = base_g(:,7);b8_g = base_g(:,8);

a1_e = attach_e(:,1);a2_e = attach_e(:,2);a3_e = attach_e(:,3);a4_e = attach_e(:,4);
a5_e = attach_e(:,5);a6_e = attach_e(:,6);a7_e = attach_e(:,7);a8_e = attach_e(:,8);

% ao_e = [0; 0; 0; 1]; 
ao_z_e = [1*cos(pi/3.4); -1*sin(pi/3.4); 0; 1];

% ao_z_e = [0; 2; 0; 1];
% ao_z_e = [1; 0; 0; 1];
ao_x_e = [0.5; 0; 0; 1];
ao_y_e = [0; 0.5; 0; 1];


%% --------------------- Trajectory Generation ------------------------- %%
% t_step = 0.1; %------时间步长
% 
% %-------用五次多项式进行轨迹规划
% % [pose_trj_smart, v_trj_eul, a_trj_eul, t_vec_G,t_end] =  plan_trj_G302_case2(t_step);
% [pose_trj_smart, v_trj_eul, a_trj_eul, t_vec_G,t_end] =  plan_trj_G302_case1(t_step);
% 
% 
% %-------轨迹规划后的欧拉角导数和二阶导数转换为角速度和角加速度
% omega_local = zeros(3,size(t_vec_G,2));
% alpha_local = zeros(3,size(t_vec_G,2));
% pose_trj = zeros(6,size(t_vec_G,2));
% v_trj = zeros(6,size(t_vec_G,2));
% a_trj = zeros(6,size(t_vec_G,2));
% % v_trj(1:3,:) = v_trj_eul(1:3,:);
% % a_trj(1:3,:) = a_trj_eul(1:3,:);
% pose_trj(4:6,:) = pose_trj_smart(4:6,:);
% for i = 1:size(t_vec_G,2)
%     omega_local(:,i) = eul2omega(pose_trj_smart(4:6,i),v_trj_eul(4:6,i));
%     alpha_local(:,i) = eulZYXddot2alpha(pose_trj_smart(4:6,i),v_trj_eul(4:6,i),a_trj_eul(4:6,i));
%     [v_trj(4:6,i),a_trj(4:6,i)] = body_to_global(pose_trj_smart(4:6,i), omega_local(:,i),alpha_local(:,i));
% 
% 
%     r_vec_local = [0;0;0.35/3];
%     [r_vec_global,~] = body_to_global(pose_trj_smart(4:6,i), r_vec_local,r_vec_local);
%     pose_trj(1:3,i) = pose_trj_smart(1:3,i) + r_vec_global;
%     v_trj(1:3,i) = v_trj_eul(1:3,i) + Skew_F(v_trj(4:6))*r_vec_global;
%     a_trj(1:3,i) = a_trj_eul(1:3,i) + Skew_F(a_trj(4:6))*r_vec_global + Skew_F(v_trj(4:6))*(Skew_F(v_trj(4:6))*r_vec_global);    
% end

%% --------------------- Interaction wrenches Generation ------------------------- %%
t_step = 0.05; %------时间步长
t_end = 15;
N_total = round(t_end / t_step);  % 积分区间数
t_vec_G = (0:N_total) * t_step;   % 状态时间点，共 N_total+1 个
step_num = N_total + 1;           % 状态点数量

%-------生成假定六维力传感器原始数据，F_sensorRaw = [f_s; tau_s]
sensor_data = generate_interaction_wrench_G302(t_step, N_total, 'default');
integrator_cfg.input_hold = 'foh_jump'; % 可选 'zoh'、'foh' 或 'foh_jump'
integrator_cfg.force_jump_threshold = 1.0;    % 突变力判断阈值(N)
integrator_cfg.moment_jump_threshold = 0.01;  % 突变力矩判断阈值(N*m)
sensor_cfg.R_ES = para_cdpr.sensor_R_ES;
sensor_cfg.r_ES_E = para_cdpr.sensor_origin_e(1:3);
sensor_cfg.sensor_sign = para_cdpr.sensor_sign;

%-------交互力驱动动力学递推参数
para_dyn.mass_ee = mass_ee;
para_dyn.Iee = Iee;
para_dyn.Iee_inv = inv(Iee);

pose_trj = zeros(6,step_num);
v_trj = zeros(6,step_num);
a_trj = zeros(6,step_num);
omega_local = zeros(3,step_num);
alpha_local = zeros(3,step_num);
R_trj = zeros(3,3,step_num);
interaction_wrench_E = zeros(6,step_num);
interaction_wrench_G = zeros(6,step_num);
inertial_wrench_G = zeros(6,step_num);
integrator_mode_log = strings(1,N_total);

%-------动平台质心初始状态
pose_trj(:,1) = [0; -0.8; 0.4 + 0.35/3; 0; 0; 0];
v_trj(:,1) = zeros(6,1);
omega_local(:,1) = zeros(3,1);
init_eul = pose_trj(4:6,1);
R_trj(:,:,1) = [cos(init_eul(3)), -sin(init_eul(3)), 0; ...
                sin(init_eul(3)),  cos(init_eul(3)), 0; ...
                               0,                 0, 1] * ...
               [ cos(init_eul(2)), 0, sin(init_eul(2)); ...
                                0, 1,                0; ...
                -sin(init_eul(2)), 0, cos(init_eul(2))] * ...
               [1,                0,                 0; ...
                0, cos(init_eul(1)), -sin(init_eul(1)); ...
                0, sin(init_eul(1)),  cos(init_eul(1))];

%% ------------------------ Dynamics Solve ----------------------------- %%
force_min = 0;
force_max = 500;

num_vertex = zeros(1,step_num);
ideal_workspace = zeros(6,step_num);
ideal_cable_length = zeros(8,step_num);
ideal_cable_force = zeros(8,step_num);
ideal_cable_v = zeros(8,step_num);
ideal_cable_a = zeros(8,step_num);
ideal_a_g = zeros(48,step_num);

i_num = 0;
i_ideal = 0;
[a1_g0,a2_g0,a3_g0,a4_g0,a5_g0,a6_g0,a7_g0,a8_g0] = initial_A(pose_trj,attach_e);

a_down_e = [0;0;-0.15-(-0.35*2/3);1];
a_up_e = [0;0;0.15-(-0.35*2/3);1];

for i_step = 1:step_num
    state_curr.pose = pose_trj(:,i_step);
    state_curr.vel = v_trj(:,i_step);
    state_curr.omega_E = omega_local(:,i_step);
    state_curr.R_WB = R_trj(:,:,i_step);

    if i_step <= N_total
        wrench_interval_S = sensor_data.wrench_raw_S(:,i_step:i_step+1);
        [state_curr, state_next, dyn_out] = interaction_dynamics_step_G302( ...
            state_curr, wrench_interval_S, para_dyn, sensor_cfg, t_step, integrator_cfg);
    
        pose_trj(:,i_step) = state_curr.pose;
        v_trj(:,i_step) = state_curr.vel;
        a_trj(:,i_step) = dyn_out.acc_G6;
        omega_local(:,i_step) = state_curr.omega_E;
        alpha_local(:,i_step) = dyn_out.alpha_E;
        R_trj(:,:,i_step) = state_curr.R_WB;
        interaction_wrench_E(:,i_step) = dyn_out.wrench_E;
        interaction_wrench_G(:,i_step) = [dyn_out.force_G; dyn_out.moment_G];
        inertial_wrench_G(:,i_step) = dyn_out.inertial_wrench_G;
        integrator_mode_log(i_step) = dyn_out.input_mode;
    
        pose_trj(:,i_step+1) = state_next.pose;
        v_trj(:,i_step+1) = state_next.vel;
        omega_local(:,i_step+1) = state_next.omega_E;
        R_trj(:,:,i_step+1) = state_next.R_WB;
    else
        % 最后一个状态点不再重复使用末段输入力积分，只补齐绘图/后续计算所需数据。
        dyn_out = interaction_dynamics_sample_output_G302(state_curr, ...
            sensor_data.wrench_raw_S(:,i_step), para_dyn, sensor_cfg);
        pose_trj(:,i_step) = state_curr.pose;
        v_trj(:,i_step) = state_curr.vel;
        a_trj(:,i_step) = a_trj(:,i_step-1);
        omega_local(:,i_step) = state_curr.omega_E;
        alpha_local(:,i_step) = alpha_local(:,i_step-1);
        R_trj(:,:,i_step) = state_curr.R_WB;
        interaction_wrench_E(:,i_step) = dyn_out.wrench_E;
        interaction_wrench_G(:,i_step) = [dyn_out.force_G; dyn_out.moment_G];
        inertial_wrench_G(:,i_step) = inertial_wrench_G(:,i_step-1);
    end

    pose_ee = pose_trj(:,i_step);
    x_step = pose_ee(1);
    y_step = pose_ee(2);
    z_step = pose_ee(3);
    ax_step = pose_ee(4);
    ay_step = pose_ee(5);
    az_step = pose_ee(6);

    % Calculate homogeneous transformation matrix
    Tr_matrix = [1, 0, 0, x_step;...
                 0, 1, 0, y_step;...
                 0, 0, 1, z_step;...
                 0, 0, 0,       1] * ...
                [cos(az_step), -sin(az_step), 0, 0;...
                 sin(az_step),  cos(az_step), 0, 0;...
                            0,             0, 1, 0;...
                            0,             0, 0, 1] * ...
                [ cos(ay_step), 0, sin(ay_step), 0;...
                             0, 1,            0, 0;...
                 -sin(ay_step), 0, cos(ay_step), 0;...
                             0, 0,            0, 1] * ...
                [1,            0,             0, 0;...
                 0, cos(ax_step), -sin(ax_step), 0;...
                 0, sin(ax_step),  cos(ax_step), 0;...
                 0,            0,             0, 1];

    % Coordinates of attachment points converted to the global frame
    a1_g = Tr_matrix * a1_e;
    a2_g = Tr_matrix * a2_e;
    a3_g = Tr_matrix * a3_e;
    a4_g = Tr_matrix * a4_e;
    a5_g = Tr_matrix * a5_e;
    a6_g = Tr_matrix * a6_e;
    a7_g = Tr_matrix * a7_e;
    a8_g = Tr_matrix * a8_e;

    a_down_g = Tr_matrix * a_down_e;
    a_up_g = Tr_matrix * a_up_e;

    ao_g = Tr_matrix * ao_e;
    ao_z_g = Tr_matrix * ao_z_e;
    ao_x_g = Tr_matrix * ao_x_e;
    ao_y_g = Tr_matrix * ao_y_e;
    % a_smart_mess_g = Tr_matrix * a_smart_mess_e;

    % Vectors of EE in global frame (from the center to attachment points)
    vector_ee1 = a1_g - ao_g; vector_ee1(4) = [];
    vector_ee2 = a2_g - ao_g; vector_ee2(4) = [];
    vector_ee3 = a3_g - ao_g; vector_ee3(4) = [];
    vector_ee4 = a4_g - ao_g; vector_ee4(4) = [];
    vector_ee5 = a5_g - ao_g; vector_ee5(4) = [];
    vector_ee6 = a6_g - ao_g; vector_ee6(4) = [];
    vector_ee7 = a7_g - ao_g; vector_ee7(4) = [];
    vector_ee8 = a8_g - ao_g; vector_ee8(4) = [];

    % Cable vectors of corresponding attachment points (from EE to the base)
    vector_cable1 = b1_g - a1_g; vector_cable1(4) = [];
    vector_cable2 = b2_g - a2_g; vector_cable2(4) = [];
    vector_cable3 = b3_g - a3_g; vector_cable3(4) = [];
    vector_cable4 = b4_g - a4_g; vector_cable4(4) = [];
    vector_cable5 = b5_g - a5_g; vector_cable5(4) = [];
    vector_cable6 = b6_g - a6_g; vector_cable6(4) = [];
    vector_cable7 = b7_g - a7_g; vector_cable7(4) = [];
    vector_cable8 = b8_g - a8_g; vector_cable8(4) = [];

    %--------- 判断是否干涉（8条线段，3D空间中的端点坐标）
    segments_curr = [
        [a1_g(1:3)', b1_g(1:3)']; ...
        [a2_g(1:3)', b2_g(1:3)']; ...
        [a3_g(1:3)', b3_g(1:3)']; ...
        [a4_g(1:3)', b4_g(1:3)']; ...
        [a5_g(1:3)', b5_g(1:3)']; ...
        [a6_g(1:3)', b6_g(1:3)']; ...
        [a7_g(1:3)', b7_g(1:3)']; ...
        [a8_g(1:3)', b8_g(1:3)']];

    segments_prev = [
        [a1_g0(1:3)', b1_g(1:3)']; ...
        [a2_g0(1:3)', b2_g(1:3)']; ...
        [a3_g0(1:3)', b3_g(1:3)']; ...
        [a4_g0(1:3)', b4_g(1:3)']; ...
        [a5_g0(1:3)', b5_g(1:3)']; ...
        [a6_g0(1:3)', b6_g(1:3)']; ...
        [a7_g0(1:3)', b7_g(1:3)']; ...
        [a8_g0(1:3)', b8_g(1:3)']];


    %-------最短距离小于容许值检查
    % hasInterference = checkLineSegmentInterference(segments_prev,segments_curr);
    


    %-------绳索干涉检查
    [hasInterference, info] = checkCableInterferenceFull(segments_prev, segments_curr);


    %-------加一个绳索与圆柱体干涉检查
    for ii = 1 : 2 : 8
        vector = [segments_curr(ii,:);
                 a_down_g(1:3)', a_up_g(1:3)'];
        [hasInterference2, info2] = checkCableplatInterference(vector);
    end

    

% %------------保存上一步的绳索位置关系
    a1_g0 = a1_g;a2_g0 = a2_g;a3_g0 = a3_g;a4_g0 = a4_g;
    a5_g0 = a5_g;a6_g0 = a6_g;a7_g0 = a7_g;a8_g0 = a8_g;
    

    % Ideal cable lengths
    ideal_cl1 = norm(vector_cable1);
    ideal_cl2 = norm(vector_cable2);
    ideal_cl3 = norm(vector_cable3);
    ideal_cl4 = norm(vector_cable4);
    ideal_cl5 = norm(vector_cable5);
    ideal_cl6 = norm(vector_cable6);
    ideal_cl7 = norm(vector_cable7);
    ideal_cl8 = norm(vector_cable8);

    % Unit cable vectors
    unit_vector_cable1 = vector_cable1 / ideal_cl1;
    unit_vector_cable2 = vector_cable2 / ideal_cl2;
    unit_vector_cable3 = vector_cable3 / ideal_cl3;
    unit_vector_cable4 = vector_cable4 / ideal_cl4;
    unit_vector_cable5 = vector_cable5 / ideal_cl5;
    unit_vector_cable6 = vector_cable6 / ideal_cl6;
    unit_vector_cable7 = vector_cable7 / ideal_cl7;
    unit_vector_cable8 = vector_cable8 / ideal_cl8;

    % Jacobian matrix
    jaco_trans = [unit_vector_cable1, unit_vector_cable2, unit_vector_cable3, ...
                  unit_vector_cable4, unit_vector_cable5, unit_vector_cable6, ...
                  unit_vector_cable7, unit_vector_cable8; ...
                  Skew_F(vector_ee1)*unit_vector_cable1, ...
                  Skew_F(vector_ee2)*unit_vector_cable2, ...
                  Skew_F(vector_ee3)*unit_vector_cable3, ...
                  Skew_F(vector_ee4)*unit_vector_cable4, ...
                  Skew_F(vector_ee5)*unit_vector_cable5, ...
                  Skew_F(vector_ee6)*unit_vector_cable6, ...
                  Skew_F(vector_ee7)*unit_vector_cable7, ...
                  Skew_F(vector_ee8)*unit_vector_cable8;];
    jaco = jaco_trans';

    % cable velocity, acceleration and length
    ideal_cable_v(:,i_step) = jaco*v_trj(:, i_step);
    if i_step == 1
        ideal_cable_a(:,i_step) = zeros(8,1);
    else
        ideal_cable_a(:,i_step) = (ideal_cable_v(:,i_step) - ideal_cable_v(:,i_step-1)) / t_step;
    end

    ideal_cl = [ideal_cl1; ideal_cl2; ideal_cl3; ideal_cl4; ...
                ideal_cl5; ideal_cl6; ideal_cl7; ideal_cl8;];

    % Newton-Euler dynamic equation
    interaction_force_G = interaction_wrench_G(1:3,i_step);
    interaction_moment_G = interaction_wrench_G(4:6,i_step);
    inertial_force_G = inertial_wrench_G(1:3,i_step);
    inertial_moment_G = inertial_wrench_G(4:6,i_step);
    % 此处的牛顿欧拉方程中，惯性力项和交互力项可以抵消掉，等式两边只剩绳索合力与重力
    % force_ee = [0;0;-mass_ee*9.8] + interaction_force_G + inertial_force_G; %external force
    % moment_ee = interaction_moment_G + inertial_moment_G; %external moment
    force_ee = [0;0;-mass_ee*9.8];
    moment_ee = interaction_moment_G;

    
                     % % 4-norm method
                    % 
                    % % use 4-norm method by Gosselin to solve the force distribution problem
                    % ideal_cf_lb = force_min * ones(8,1);
                    % ideal_cf_ub = force_max * ones(8,1);
                    % force_average = (force_min+force_max)/2 * ones(8,1);
                    % 
                    % % initial iterative point of the optimizer
                    % if (i_ideal == 0)
                    %     ideal_cf_x0 = force_average;
                    % else
                    %     ideal_cf_x0 = ideal_cf; %use last solved 'cf' as the initial point
                    % end
                    % 
                    % % linear constrains
                    % ideal_cf_aeq = jaco_trans;
                    % ideal_cf_beq = -[force_ee;moment_ee];
                    % options = optimset('Display','off','TolFun',1e-6,'TolX',1e-6,...
                    %                    'MaxIter',800,'MaxFunEvals',9*500);
                    % [ideal_cf,ideal_cf_fval,ideal_cf_exitflag]= ...
                    %    fmincon(@(x)sum((x).^4),ideal_cf_x0,[],[],...
                    %            ideal_cf_aeq,ideal_cf_beq,ideal_cf_lb,ideal_cf_ub,[],options);%-force_average
                    % i_num = i_num + 1;
                    % % ideal_cf_fval_fin(i_num) = ideal_cf_fval;
                    % % ideal_cf_exitflag_fin(i_num) = ideal_cf_exitflag;
                    % 
                    % % if fmincon can find the cable force, the position is in the workspace
                    % if (ideal_cf_exitflag >= 0)
                    %     i_ideal=i_ideal+1;
                    %     % workspace, cable length and force are put in the cells
                    %     ideal_workspace(:,i_ideal) = [x_step; y_step; z_step; ...
                    %                                ax_step; ay_step; az_step];
                    %     ideal_cable_length(:,i_ideal) = ideal_cl;
                    %     ideal_cable_force(:,i_ideal) = ideal_cf;
                    %     ideal_a_g(:,i_ideal) = [a1_g;a2_g;a3_g;a4_g;a5_g;a6_g;a7_g;a8_g;a9_top_g;a10_down_g;ao_g;ao_z_g;a11_top_g;a12_down_g];
                    % else
                    %     continue
                    % end

    % barycenter method
    try
        [ideal_cf, num_v] = bary_center(force_ee, moment_ee, jaco, force_min, force_max);
    catch
        [ideal_cf, ideal_cf_exitflag] = pseudo_inverse_new(force_ee, moment_ee, jaco, force_min, force_max);
        num_v = 0;
        if ideal_cf_exitflag == -1
            warning('第 %d 个时间步绳力求解失败，跳过该步。', i_step);
            continue
        end
    end


    % if each cable force is bigger than 0, the position is in the workspace
    if ((ideal_cf(1)>0) && (ideal_cf(2)>0) && (ideal_cf(3)>0) && ...
        (ideal_cf(4)>0) && (ideal_cf(5)>0) && (ideal_cf(6)>0) && ...
        (ideal_cf(7)>0) && (ideal_cf(8)>0))
        i_ideal = i_ideal + 1;
        % workspace, cable length and force are put in the cells
        ideal_workspace(:,i_ideal) = [x_step; y_step; z_step; ...
                                   ax_step; ay_step; az_step];
        ideal_cable_length(:,i_ideal) = ideal_cl;
        ideal_cable_force(:,i_ideal) = ideal_cf;
        ideal_a_g(:,i_ideal) = [a1_g;a2_g;a3_g;a4_g;a5_g;a6_g;a7_g;a8_g;ao_g;ao_z_g;a_down_g;a_up_g];
    else
        continue
    end
end

%% ------------------------- Plotting ---------------------------------- %%
result_trj_ee.t_vec_G = t_vec_G;
result_trj_ee.t_step = t_step;
result_trj_ee.pose_ee = pose_trj;
result_trj_ee.vel_ee = v_trj;
result_trj_ee.acc_ee = a_trj;

result_trj_cable.cl_trj = ideal_cable_length;
result_trj_cable.cv_trj = ideal_cable_v;
result_trj_cable.ca_trj = ideal_cable_a;
result_trj_cable.cf_trj = ideal_cable_force;

result_wrench.interaction_wrench_E = interaction_wrench_E;
result_wrench.interaction_wrench_G = interaction_wrench_G;
result_wrench.inertial_wrench_G = inertial_wrench_G;
result_wrench.integrator_mode_log = integrator_mode_log;

plot_results_G302(result_trj_ee, base_g, result_trj_cable, ideal_a_g, result_wrench);
