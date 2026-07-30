# G302 Interaction Wrench Driven Dynamics Refactor Plan

## 1. Goal

当前 `G302.m` 的主流程是：

1. 先由轨迹规划函数生成完整的 `pose_trj_smart / v_trj_eul / a_trj_eul`。
2. 再将欧拉角导数转换为角速度、角加速度。
3. 在 `Dynamics Solve` 循环中逐帧读取既定轨迹状态 `pose_trj(:,i_step)`、`v_trj(:,i_step)`、`a_trj(:,i_step)`。
4. 基于该状态计算绳长、绳速、Jacobian、末端惯性项、绳力分配和绘图数据。

新的目标是把第 1 步的“预先给定轨迹”替换为“由假定六维力传感器交互力递推动平台状态”：

1. 在 `Interaction wrenches Generation` 模块生成假定 F/T sensor 原始六维力。
2. 将 sensor frame 下的原始 wrench 转换到动平台质心 local frame。
3. 在 `Dynamics Solve` 每个时间步调用一个新的逆动力学/状态更新函数。
4. 该函数根据当前状态、当前交互 wrench 和平台惯量参数，输出当前时刻作用在质心的力/力矩、加速度、角速度/角加速度，并积分得到下一时刻速度和位姿。
5. 主循环继续使用本时刻生成的 `pose_ee / v_ee / a_ee` 计算干涉检查、绳长、绳速、绳力分配、功率和绘图。

## 2. Key Design Choice

建议保留 `G302.m` 后半段现有的绳索几何、Jacobian、绳力分配和绘图逻辑，只替换状态来源。

也就是说，原来的：

```matlab
pose_ee = pose_trj(:,i_step);
...
ideal_cable_v(:,i_step) = jaco*v_trj(:, i_step);
Fa = -M0_ee*a_trj(:,i_step);
omega = omega_local(:, i_step);
alpha = alpha_local(:, i_step);
```

改造后应变成：

```matlab
pose_ee = pose_trj(:,i_step);
v_ee = v_trj(:,i_step);
a_ee = a_trj(:,i_step);
omega = omega_local(:,i_step);
alpha = alpha_local(:,i_step);
```

其中这些数组不再来自轨迹规划，而是在循环中由交互 wrench 递推填充。

## 3. Coordinate Frames

建议明确三个坐标系：

- `G`: global frame，当前 `base_g` 和主循环中 `Tr_matrix` 所在坐标系。
- `E`: 动平台质心 local frame，`ao_e = [0;0;0;1]`，惯量矩阵 `Iee` 默认在这个坐标系表达。
- `S`: F/T sensor frame，六维力传感器原始输出所在坐标系。

传感器原始输出定义为：

```matlab
wrench_sensor_raw_S = [Fx_S; Fy_S; Fz_S; Mx_S; My_S; Mz_S];
```

需要转换为动平台质心 local frame：

```matlab
wrench_interaction_E = transform_sensor_wrench_to_ee( ...
    wrench_sensor_raw_S, R_ES, r_ES_E);
```

其中：

- `R_ES`: sensor frame 到 EE local frame 的旋转矩阵。
- `r_ES_E`: sensor 原点相对 EE 质心的位置，使用 EE local frame 表达。

wrench 平移转换建议使用：

```matlab
F_E = R_ES * F_S;
M_E_at_sensor = R_ES * M_S;
M_E_at_com = M_E_at_sensor + Skew_F(r_ES_E) * F_E;
wrench_E = [F_E; M_E_at_com];
```

注意这里假设传感器测得的是“环境作用在平台上的力”。如果传感器符号定义是“平台作用在环境上的力”，则最终需要取负号。这个符号建议单独放一个参数，例如：

```matlab
sensor_sign = 1;   % or -1
```

## 4. Interaction Wrench Generation

新增函数建议：

```matlab
function sensor_data = generate_interaction_wrench_G302(t_vec_G, scenario)
```

输出结构体：

```matlab
sensor_data.t_vec_G
sensor_data.wrench_raw_S      % 6 x step_num
sensor_data.R_ES              % 3 x 3
sensor_data.r_ES_E            % 3 x 1
sensor_data.sensor_sign
```

初期可用简单可控的假设力，例如：

```matlab
Fx = 5*sin(2*pi*0.2*t);
Fy = zeros(size(t));
Fz = zeros(size(t));
Mx = zeros(size(t));
My = zeros(size(t));
Mz = 0.2*sin(2*pi*0.2*t);
```

后续再换成真实实验数据或更复杂的接触模型。

## 5. New Dynamics Function

建议新增函数：

```matlab
function [state_curr, state_next, dyn_out] = interaction_dynamics_step_G302( ...
    state_curr, wrench_sensor_raw_S, para_dyn, sensor_cfg, t_step)
```

输入：

```matlab
state_curr.pose      % 6 x 1, [x;y;z;roll;pitch;yaw]
state_curr.vel       % 6 x 1, [vx;vy;vz; wx_G;wy_G;wz_G] or angular velocity convention chosen below
state_curr.omega_E   % 3 x 1, body/local angular velocity
```

建议内部采用：

- 平动速度 `v_G` 用 global frame 表达。
- 姿态仍使用 ZYX 欧拉角 `[roll; pitch; yaw]`。
- 角速度和角加速度动力学计算优先用 EE local frame，即 `omega_E / alpha_E`。
- 输出给当前主循环的 `v_trj(4:6,i)`、`a_trj(4:6,i)` 继续保持 global angular velocity / global angular acceleration，以匹配当前 `jaco*v_trj` 的使用方式。

动力学核心：

```matlab
wrench_E = transform_sensor_wrench_to_ee(...);
F_E = wrench_E(1:3);
M_E = wrench_E(4:6);

R_GE = ...                            % 使用 G302.m 当前同样的 ZYX 旋转矩阵
F_G = R_GE * F_E;

a_G = F_G / mass_ee;
alpha_E = Iee \ (M_E - Skew_F(omega_E)*Iee*omega_E);
```

状态递推只由交互力产生；重力不进入 `interaction_dynamics_step_G302`，而是在 `G302.m` 的绳力分配外载中显式加入。

积分建议先用半隐式 Euler，简单稳定：

```matlab
v_G_next = v_G + a_G*t_step;
pos_next = pos + v_G_next*t_step;

omega_E_next = omega_E + alpha_E*t_step;
phi = eul(1);
theta = eul(2);
eul_dot_next = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                0, cos(phi),           -sin(phi);
                0, sin(phi)/cos(theta), cos(phi)/cos(theta)] * omega_E_next;
eul_next = eul + eul_dot_next*t_step;
```

如果你希望更平滑，下一版可以改为 RK4 或 `ode45`，但第一版先保持和主循环结构一致。

## 6. Required Helper Functions

建议新增这些小函数，避免把主脚本继续写长：

1. `generate_interaction_wrench_G302.m`

   生成假定传感器原始 wrench。

2. `transform_sensor_wrench_to_ee.m`

   将 sensor frame 的 wrench 转换到 EE local frame 质心处。

3. `interaction_dynamics_step_G302.m`

   单步动力学计算和积分。

注意：计算中涉及叉乘时，不使用 MATLAB 内置 `cross` 函数，统一使用当前工程已有的 `Skew_F.m`：

```matlab
cross(a,b)  ->  Skew_F(a) * b
```

例如 wrench 平移和刚体转动项应写成：

```matlab
M_E_at_com = M_E_at_sensor + Skew_F(r_ES_E) * F_E;
gyro_E = Skew_F(omega_E) * Iee * omega_E;
```

## 7. Changes in G302.m

### 7.1 Initialization

保留：

```matlab
t_step = 0.1;
```

新增：

```matlab
t_end = 15;                       % or interaction scenario duration
t_vec_G = 0:t_step:t_end;
step_num = size(t_vec_G,2);

pose_trj = zeros(6,step_num);
v_trj = zeros(6,step_num);
a_trj = zeros(6,step_num);
omega_local = zeros(3,step_num);
alpha_local = zeros(3,step_num);
interaction_wrench_E = zeros(6,step_num);

pose_trj(:,1) = initial_pose_ee;
v_trj(:,1) = initial_vel_ee;
omega_local(:,1) = initial_omega_E;
```

注意：`pose_trj_smart` 在新流程中可以不再使用。绘图处建议改回：

```matlab
result_trj_ee.pose_ee = pose_trj;
```

### 7.2 Interaction Wrench Block

```matlab
sensor_data = generate_interaction_wrench_G302(t_vec_G, scenario);
sensor_cfg.R_ES = sensor_data.R_ES;
sensor_cfg.r_ES_E = sensor_data.r_ES_E;
sensor_cfg.sensor_sign = sensor_data.sensor_sign;
```

### 7.3 Dynamics Solve Loop Entry

在每次 `for i_step = 1:step_num` 的开始，先从当前数组取状态：

```matlab
pose_ee = pose_trj(:,i_step);
v_ee = v_trj(:,i_step);
a_ee = a_trj(:,i_step);
omega = omega_local(:,i_step);
alpha = alpha_local(:,i_step);
```

然后进入当前已有的几何和绳索计算。

### 7.4 State Propagation

建议在每步循环末尾、完成本时刻绳力等计算之后，递推下一步：

```matlab
if i_step < step_num
    state_curr.pose = pose_trj(:,i_step);
    state_curr.vel = v_trj(:,i_step);
    state_curr.omega_E = omega_local(:,i_step);

    [~, state_next, dyn_out] = interaction_dynamics_step_G302( ...
        state_curr, ...
        sensor_data.wrench_raw_S(:,i_step), ...
        para_dyn, sensor_cfg, t_step);

    pose_trj(:,i_step+1) = state_next.pose;
    v_trj(:,i_step+1) = state_next.vel;
    a_trj(:,i_step) = dyn_out.acc_G6;
    omega_local(:,i_step+1) = state_next.omega_E;
    alpha_local(:,i_step) = dyn_out.alpha_E;
    interaction_wrench_E(:,i_step) = dyn_out.wrench_E;
end
```

最后一个时刻的 `a_trj(:,end)` 和 `alpha_local(:,end)` 可以复制倒数第二个值，或在循环中为最后一步只计算不积分。

## 8. Force Used for Cable Force Distribution

当前代码中绳力分配使用：

```matlab
Fa = -M0_ee*a_trj(:,i_step);
force_ee = [0;0;-mass_ee*9.8] + Fa(1:3);
moment_ee = -R*Iee*alpha - R*Skew_F(omega)*Iee*omega;
[ideal_cf, num_v] = bary_center(force_ee, moment_ee, jaco, force_min, force_max);
```

引入交互力后，需要明确绳索需要平衡的 wrench。

推荐表达为：

```matlab
W_required_G = - (W_inertia_G + W_gravity_G + W_interaction_G);
```

其中 `bary_center` 当前接口等价于求：

```matlab
jaco_trans * cable_force + [force_ee; moment_ee] = 0
```

因此可将：

```matlab
force_ee
moment_ee
```

理解为“除绳索外，作用在平台上的外部 wrench 加惯性等效项”。交互力加入后应变为：

```matlab
force_ee = gravity_force_G + interaction_force_G + inertial_force_G;
moment_ee = interaction_moment_G + inertial_moment_G;
```

需要注意：

- `interaction_moment_G = R_GE * interaction_moment_E`
- `inertial_moment_G` 当前代码已用 `-R*Iee*alpha - R*Skew_F(omega)*Iee*omega`
- `omega` 在陀螺项中应使用 local/body angular velocity；当前代码这里是 `omega_local(:,i_step)`，这一点应保留。

## 9. Validation Plan

第一步建议使用简单场景验证：

1. `R_ES = eye(3)`，`r_ES_E = [0;0;0]`。
2. 只施加 `Fx = constant` 或 `Fx = sin(t)`，其他力和力矩为 0。
3. 初始姿态为 0，初始角速度为 0。
4. 关闭或忽略姿态变化，先验证 `x` 方向运动是否满足 `a = F/m`。
5. 再加入 `Mz`，验证 yaw 方向角加速度是否约等于 `Mz/Izz`。
6. 最后加入非零 `r_ES_E`，验证力臂产生的附加力矩方向是否正确。

## 10. Implementation Order

建议分三步做，便于定位问题：

1. 新增传感器 wrench 生成和坐标转换函数，只打印/绘制转换后的 `wrench_E`，不接入动力学。
2. 新增 `interaction_dynamics_step_G302`，用一个最小测试脚本或 `G302.m` 的短时间仿真验证状态积分。
3. 接入 `G302.m` 主循环，替换轨迹输入，同时保留绳长、绳速、绳力分配和绘图。

## 11. Open Questions Before Coding

实现前建议确认以下问题：

1. F/T sensor 的安装位姿：`R_ES` 和 `r_ES_E` 的具体数值是什么？
2. 传感器原始力的符号：原始输出表示“环境作用于动平台”，还是“动平台作用于环境”？
3. 新动力学中是否考虑重力？若考虑，绳力分配中也要统一避免重复。
4. 初始位姿、初始速度、初始角速度采用什么值？
5. 是否需要保留 `pose_trj_smart` 这个变量用于表示传感器/接触点轨迹，还是完全改为 `pose_trj`？

## 12. Optional Optimizations

以下三项先不作为第一轮实现内容，等主流程跑通后再考虑：

1. `eulZYX_to_rotm.m`

   统一 `G302.m` 中反复手写的 ZYX 旋转矩阵。

2. `omega_to_eulZYXdot.m`

   `eul2omega` 的反变换，用于从 body angular velocity 积分欧拉角。第一版也可以先在动力学函数内部直接写公式。

3. `make_transform_G302.m`

   把 `G302.m` 和 `initial_A.m` 中重复的齐次变换矩阵生成逻辑抽出来。
