function [pose_trj, v_trj, a_trj, t_vec_G] = ...
    plan_trj_line(t_start, t_step, t_end, p_start, p_end)
%% 读取轨迹参数
t_vec_G = t_start:t_step:t_end;
x_start = p_start(1);
y_start = p_start(2);
z_start = p_start(3);
alpha_start = p_start(4);
beta_start  = p_start(5);
gamma_start = p_start(6);

x_end = p_end(1);
y_end = p_end(2);
z_end = p_end(3);
alpha_end = p_end(4);
beta_end  = p_end(5);
gamma_end = p_end(6);

t_vec_mid = t_start:t_step:t_end;
t_vec = t_vec_mid-t_start*ones(1,length(t_vec_mid));

t = t_end - t_start;
% t_vec = 0:t_step:t;

%% 五次多项式生成直线轨迹
dx = x_end - x_start;
dy = y_end - y_start;
dz = z_end - z_start;
da = alpha_end - alpha_start;
db = beta_end  - beta_start;
dr = gamma_end - gamma_start;

% 多项式系数计算（约束条件：起点和终点的速度和加速度为0）
a0_x = x_start; a1_x = 0; a2_x = 0; a3_x = 10*dx/t.^3; a4_x = -15*dx/t.^4; a5_x = 6*dx/t.^5;
a0_y = y_start; a1_y = 0; a2_y = 0; a3_y = 10*dy/t.^3; a4_y = -15*dy/t.^4; a5_y = 6*dy/t.^5;
a0_z = z_start; a1_z = 0; a2_z = 0; a3_z = 10*dz/t.^3; a4_z = -15*dz/t.^4; a5_z = 6*dz/t.^5;
a0_a = alpha_start; a1_a = 0; a2_a = 0; a3_a = 10*da/t.^3; a4_a = -15*da/t.^4; a5_a = 6*da/t.^5;
a0_b = beta_start;  a1_b = 0; a2_b = 0; a3_b = 10*db/t.^3; a4_b = -15*db/t.^4; a5_b = 6*db/t.^5;
a0_r = gamma_start; a1_r = 0; a2_r = 0; a3_r = 10*dr/t.^3; a4_r = -15*dr/t.^4; a5_r = 6*dr/t.^5;

% 轨迹点计算
x_trj = a0_x + a1_x*t_vec + a2_x*t_vec.^2 + a3_x*t_vec.^3 + a4_x*t_vec.^4 + a5_x*t_vec.^5;
y_trj = a0_y + a1_y*t_vec + a2_y*t_vec.^2 + a3_y*t_vec.^3 + a4_y*t_vec.^4 + a5_y*t_vec.^5;
z_trj = a0_z + a1_z*t_vec + a2_z*t_vec.^2 + a3_z*t_vec.^3 + a4_z*t_vec.^4 + a5_z*t_vec.^5;
alpha_trj = a0_a + a1_a*t_vec + a2_a*t_vec.^2 + a3_a*t_vec.^3 + a4_a*t_vec.^4 + a5_a*t_vec.^5;
beta_trj  = a0_b + a1_b*t_vec + a2_b*t_vec.^2 + a3_b*t_vec.^3 + a4_b*t_vec.^4 + a5_b*t_vec.^5;
gamma_trj = a0_r + a1_r*t_vec + a2_r*t_vec.^2 + a3_r*t_vec.^3 + a4_r*t_vec.^4 + a5_r*t_vec.^5;

% 速度计算
vx_trj = a1_x + 2*a2_x*t_vec + 3*a3_x*t_vec.^2 + 4*a4_x*t_vec.^3 + 5*a5_x*t_vec.^4;
vy_trj = a1_y + 2*a2_y*t_vec + 3*a3_y*t_vec.^2 + 4*a4_y*t_vec.^3 + 5*a5_y*t_vec.^4;
vz_trj = a1_z + 2*a2_z*t_vec + 3*a3_z*t_vec.^2 + 4*a4_z*t_vec.^3 + 5*a5_z*t_vec.^4;
va_trj = a1_a + 2*a2_a*t_vec + 3*a3_a*t_vec.^2 + 4*a4_a*t_vec.^3 + 5*a5_a*t_vec.^4;
vb_trj = a1_b + 2*a2_b*t_vec + 3*a3_b*t_vec.^2 + 4*a4_b*t_vec.^3 + 5*a5_b*t_vec.^4;
vr_trj = a1_r + 2*a2_r*t_vec + 3*a3_r*t_vec.^2 + 4*a4_r*t_vec.^3 + 5*a5_r*t_vec.^4;

% 加速度计算
ax_trj = 2*a2_x + 6*a3_x*t_vec + 12*a4_x*t_vec.^2 + 20*a5_x*t_vec.^3;
ay_trj = 2*a2_y + 6*a3_y*t_vec + 12*a4_y*t_vec.^2 + 20*a5_y*t_vec.^3;
az_trj = 2*a2_z + 6*a3_z*t_vec + 12*a4_z*t_vec.^2 + 20*a5_z*t_vec.^3;
aa_trj = 2*a2_a + 6*a3_a*t_vec + 12*a4_a*t_vec.^2 + 20*a5_a*t_vec.^3;
ab_trj = 2*a2_b + 6*a3_b*t_vec + 12*a4_b*t_vec.^2 + 20*a5_b*t_vec.^3;
ar_trj = 2*a2_r + 6*a3_r*t_vec + 12*a4_r*t_vec.^2 + 20*a5_r*t_vec.^3;

pose_trj = [x_trj; y_trj; z_trj; alpha_trj; beta_trj; gamma_trj];
v_trj = [vx_trj; vy_trj; vz_trj; va_trj; vb_trj; vr_trj];
a_trj = [ax_trj; ay_trj; az_trj; aa_trj; ab_trj; ar_trj];




