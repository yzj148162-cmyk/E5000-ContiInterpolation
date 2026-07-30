# G302中加入ZOH/FOH-RK4可选积分方式的修改规划

## 1. 修改目标

当前 `G302.m` 中的动力学正解采用的是 ZOH-RK4 思路：每个积分区间内，RK4 的四个阶段都使用同一个六维传感器力输入。

本次计划在保留原 ZOH-RK4 的基础上，新增 FOH-RK4 作为可选项：

- ZOH-RK4：每个积分区间内输入力保持为左端采样值。
- FOH-RK4：每个积分区间内输入力由左右端采样值线性插值。

FOH-RK4 的使用方式定义为延迟一拍响应：收到 `t_{i+1}` 的传感器采样后，使用 `t_i` 和 `t_{i+1}` 两个采样值回算 `[t_i, t_{i+1}]` 区间内的状态变化。它不是外推预测未来输入。

## 2. 统一时间与数组长度约定

继续使用 `N_total` 作为唯一主计时量：

```matlab
N_total = round(t_end / t_step);  % 积分区间数
t_vec_G = (0:N_total) * t_step;   % 状态/采样时间点
step_num = N_total + 1;           % 状态点数量
```

数组长度约定：

- `pose_trj`, `v_trj`, `a_trj`, `R_trj`：长度为 `step_num = N_total + 1`。
- `sensor_data.wrench_raw_S`：长度改为 `step_num = N_total + 1`。
- 主动力学积分循环：只对 `i_step = 1:N_total` 执行。

这样每个积分区间 `[t_i, t_{i+1}]` 都能同时拿到左右端传感器采样：

```matlab
wrench_left_S  = sensor_data.wrench_raw_S(:, i_step);
wrench_right_S = sensor_data.wrench_raw_S(:, i_step+1);
```

## 3. ZOH与FOH的RK4阶段输入

设当前积分区间为 `[t_i, t_{i+1}]`，步长为 `h`，传感器力经坐标变换后得到：

```matlab
W_i     = wrench_E_left;
W_ip1   = wrench_E_right;
W_mid   = 0.5 * (W_i + W_ip1);
```

### 3.1 ZOH-RK4

区间内输入保持为左端采样值：

```text
k1: W_i
k2: W_i
k3: W_i
k4: W_i
```

### 3.2 FOH-RK4

区间内输入按一阶保持线性变化：

```text
k1: W_i
k2: 0.5 * (W_i + W_{i+1})
k3: 0.5 * (W_i + W_{i+1})
k4: W_{i+1}
```

等价连续表达为：

```matlab
W(t_i + tau) = W_i + tau / h * (W_ip1 - W_i);
```

## 4. 坐标系转换策略

当前传感器原始力 `wrench_raw_S` 在 sensor frame 下表达，而动力学正解中使用动平台质心 local frame 下的力。

FOH 插值有两种理论等价方式：

1. 先在 sensor frame 下插值，再转换到 local frame。
2. 先把左右端点力都转换到 local frame，再在 local frame 下插值。

由于当前传感器到质心 local frame 的 wrench 变换是线性的，二者等价。

计划采用方式2：

```matlab
wrench_left_E  = transform_sensor_wrench_to_ee(wrench_left_S, sensor_cfg);
wrench_right_E = transform_sensor_wrench_to_ee(wrench_right_S, sensor_cfg);
```

然后在 RK4 阶段内部根据 `input_hold` 选择使用左端、右端或中点力。

## 5. 绘图时间轴处理

FOH-RK4 虽然在实时系统中延迟一拍输出，但绘图中建议按物理时间对齐，而不是整体向后平移一拍。

即：

```text
用 W_i 和 W_{i+1} 回算 [t_i, t_{i+1}]
得到的 state_{i+1} 仍然画在 t_{i+1}
```

原因：

- 当前主要目的是比较 ZOH 和 FOH 对同一物理积分区间的积分精度。
- 如果把 FOH 曲线整体延迟绘制，会混入实时输出延迟造成的相位差，不利于判断积分漂移。

设计文档中需要明确区分：

- 物理时间：状态属于实际运动发生的时间点。
- 可获得时间：FOH 结果在实时实现中要晚一个采样周期才能获得。

当前工程中的主绘图按物理时间绘制，不平移 FOH 曲线。

## 6. 需要修改的文件与步骤

### 6.1 `G302.m`

新增积分配置：

```matlab
integrator_cfg.input_hold = 'zoh';  % 可选 'zoh' 或 'foh'
```

主循环中每个积分区间传入两个端点的传感器采样：

```matlab
wrench_interval_S = sensor_data.wrench_raw_S(:, i_step:i_step+1);
```

调用动力学步进函数时增加 `integrator_cfg`：

```matlab
[state_curr, state_next, dyn_out] = interaction_dynamics_step_G302( ...
    state_curr, wrench_interval_S, para_dyn, sensor_cfg, t_step, integrator_cfg);
```

最后一个状态点仍然不再执行积分。最后一帧绘图用的 `interaction_wrench_E/G` 建议使用最后一个传感器采样点转换得到，而不是简单复制上一帧。

### 6.2 `generate_interaction_wrench_G302.m`

当前输出为 `6 x N_total`，需要改为 `6 x (N_total+1)`，表示真实传感器采样点序列：

```matlab
t_sample = (0:N_total) * t_step;
wrench_raw_S = zeros(6, N_total+1);
```

例如余弦输入应改为：

```matlab
wrench_raw_S(1,:) = 8 * cos(2*pi*0.2*t_sample);
```

输出字段建议：

```matlab
sensor_data.N_total = N_total;
sensor_data.t_sample = t_sample;
sensor_data.wrench_raw_S = wrench_raw_S;
sensor_data.scenario = scenario;
```

原来的 `t_input` 字段可以删除，或改名为 `t_sample`。

### 6.3 `interaction_dynamics_step_G302.m`

函数接口从单个输入力改为区间输入力：

```matlab
function [state_curr, state_next, dyn_out] = interaction_dynamics_step_G302( ...
    state_curr, wrench_sensor_interval_S, para_dyn, sensor_cfg, t_step, integrator_cfg)
```

内部先转换左右端点力：

```matlab
wrench_left_E = transform_sensor_wrench_to_ee(wrench_sensor_interval_S(:,1), sensor_cfg);
wrench_right_E = transform_sensor_wrench_to_ee(wrench_sensor_interval_S(:,2), sensor_cfg);
```

再传给 RK4：

```matlab
[x_next, k1] = rk4_step( ...
    x0, wrench_left_E, wrench_right_E, mass_ee, I_b, I_b_inv, t_step, integrator_cfg);
```

`dyn_out.wrench_E` 建议保持为当前采样点 `t_i` 的力，即 `wrench_left_E`。这样绘图含义仍然是当前采样点的传感器力。

### 6.4 `rk4_step` 局部函数

`rk4_step` 增加左右端点 wrench 和配置输入：

```matlab
function [x_next, k1] = rk4_step( ...
    x0, wrench_left_E, wrench_right_E, mass_ee, I_b, I_b_inv, t_step, integrator_cfg)
```

根据 `integrator_cfg.input_hold` 选择四个阶段的输入：

```matlab
switch lower(integrator_cfg.input_hold)
    case 'zoh'
        wrench_k1 = wrench_left_E;
        wrench_k2 = wrench_left_E;
        wrench_k3 = wrench_left_E;
        wrench_k4 = wrench_left_E;
    case 'foh'
        wrench_mid = 0.5 * (wrench_left_E + wrench_right_E);
        wrench_k1 = wrench_left_E;
        wrench_k2 = wrench_mid;
        wrench_k3 = wrench_mid;
        wrench_k4 = wrench_right_E;
    otherwise
        error('未知的输入保持方式: %s', integrator_cfg.input_hold);
end
```

然后分别传入 `rigid_body_rhs`。

### 6.5 `单刚体动力学正解_RK4设计文档.md`

需要同步新增或更新以下内容：

- 数组长度约定：`N_total`、`step_num`、`wrench_raw_S`。
- ZOH-RK4 与 FOH-RK4 的定义。
- FOH 延迟一拍的真实含义。
- FOH 的坐标系转换策略。
- 绘图按物理时间对齐，不按可获得时间整体平移。

## 7. 验证计划

修改完成后按以下顺序验证：

1. `checkcode('G302.m','generate_interaction_wrench_G302.m','interaction_dynamics_step_G302.m')`
2. 运行：

```matlab
integrator_cfg.input_hold = 'zoh';
G302
```

确认 ZOH 结果和当前版本趋势一致。

3. 切换：

```matlab
integrator_cfg.input_hold = 'foh';
G302
```

确认无索引越界，绘图正常。

4. 对比 `pose_trj`、`v_trj`、`a_trj` 和 `interaction_wrench_G` 曲线，观察 FOH 是否减小由 ZOH 引入的积分漂移。

5. 检查最后一个状态点：

- 不应再次执行积分。
- 应使用最后一个传感器采样值补齐绘图用交互力。

## 8. 当前暂不处理的优化项

以下内容本次不作为必要修改：

- 增加实时输出时间轴 `t_available = t_vec_G + t_step`。
- 为 FOH 中点力单独绘图。
- 增加更高阶输入重建方法，例如 SOH 或 Hermite 插值。
- 对真实传感器噪声做滤波或去偏置。
