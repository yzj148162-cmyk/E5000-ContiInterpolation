# CDPR 正运动学函数求解设计文档

## 1. 文档目的

本文档用于指导后续编写 CDPR（Cable-Driven Parallel Robot，绳驱并联机器人）正运动学 MATLAB 函数。

当前仿真阶段的控制量只有 **绳长**，因此本文档暂不考虑：

- 卷筒半径；
- 电机转角；
- 编码器脉冲；
- 绳索层绕；
- 卷筒排绳误差；
- 绳索弹性补偿。

本阶段只研究：

> 已知各根绳索长度，求末端动平台的位置和姿态。

---

## 2. 问题定义

CDPR 正运动学问题为：

\[
\text{已知 } L_i \Rightarrow \text{求末端位姿 } (p, R)
\]

其中：

- \(L_i\)：第 \(i\) 根绳索的长度；
- \(p \in \mathbb{R}^3\)：末端动平台参考点在世界坐标系下的位置；
- \(R \in SO(3)\)：动平台相对于世界坐标系的旋转矩阵。

对于 3D 空间 6 自由度 CDPR，末端位姿可以写成：

\[
x =
\begin{bmatrix}
p_x & p_y & p_z & \phi & \theta & \psi
\end{bmatrix}^T
\]

其中：

- \(p_x,p_y,p_z\)：末端位置；
- \(\phi\)：roll，绕 \(x\) 轴转角；
- \(\theta\)：pitch，绕 \(y\) 轴转角；
- \(\psi\)：yaw，绕 \(z\) 轴转角。

---

## 3. 坐标系与几何参数

### 3.1 世界坐标系

世界坐标系记为：

\[
\{W\}
\]

机架上的出绳点或固定点均在世界坐标系下表达。

第 \(i\) 根绳索的机架固定点为：

\[
A_i =
\begin{bmatrix}
A_{ix} \\
A_{iy} \\
A_{iz}
\end{bmatrix}
\]

MATLAB 中建议使用矩阵：

```matlab
A = [A1, A2, ..., Am];   % 3×m
```

其中第 \(i\) 列为第 \(i\) 根绳索的固定端点：

```matlab
Ai = A(:, i);
```

---

### 3.2 动平台本体坐标系

动平台本体坐标系记为：

\[
\{B\}
\]

第 \(i\) 根绳索在动平台上的连接点为：

\[
B_i =
\begin{bmatrix}
B_{ix} \\
B_{iy} \\
B_{iz}
\end{bmatrix}
\]

注意，\(B_i\) 是在 **动平台本体坐标系** 下表达的，不是世界坐标。

MATLAB 中建议使用矩阵：

```matlab
B = [B1, B2, ..., Bm];   % 3×m
```

其中第 \(i\) 列为第 \(i\) 根绳索在动平台上的连接点：

```matlab
Bi = B(:, i);
```

---

### 3.3 动平台位姿

动平台参考点在世界坐标系中的位置为：

\[
p =
\begin{bmatrix}
p_x \\
p_y \\
p_z
\end{bmatrix}
\]

动平台姿态用旋转矩阵表示：

\[
R = R_z(\psi) R_y(\theta) R_x(\phi)
\]

其中：

- \(\phi\)：roll；
- \(\theta\)：pitch；
- \(\psi\)：yaw。

第 \(i\) 个动平台连接点在世界坐标系下的位置为：

\[
P_i = p + R B_i
\]

---

## 4. 绳长几何方程

第 \(i\) 根绳索的理论长度为：

\[
\hat{L}_i = \| A_i - (p + R B_i) \|
\]

也可以写成：

\[
\hat{L}_i = \| A_i - P_i \|
\]

其中：

\[
P_i = p + R B_i
\]

实际仿真中已知测量绳长：

\[
L_i
\]

因此正运动学要求：

\[
\hat{L}_i = L_i
\]

即：

\[
\| A_i - (p + R B_i) \| - L_i = 0
\]

对于 \(m\) 根绳索，有：

\[
f_i(x) = \| A_i - (p + R B_i) \| - L_i = 0,
\quad i = 1,2,\dots,m
\]

---

## 5. 优化问题形式

由于该方程是非线性的，通常不直接求解析解，而是转化为非线性最小二乘问题：

\[
\min_x J(x)
\]

其中：

\[
J(x) = \sum_{i=1}^{m}
\left(
\| A_i - (p + R B_i) \| - L_i
\right)^2
\]

定义残差：

\[
r_i(x) =
\| A_i - (p + R B_i) \| - L_i
\]

残差向量为：

\[
r(x) =
\begin{bmatrix}
r_1(x) \\
r_2(x) \\
\vdots \\
r_m(x)
\end{bmatrix}
\]

则优化问题为：

\[
\min_x \|r(x)\|^2
\]

---

## 6. 函数接口设计

建议主函数命名为：

```matlab
cdpr_forward_kinematics.m
```

函数接口：

```matlab
function [x_sol, info] = cdpr_forward_kinematics(A, B, L, x0, options)
```

---

### 6.1 输入参数

#### 1. `A`

```matlab
A: 3×m double
```

含义：

- 世界坐标系下的机架固定点；
- 每一列对应一根绳索。

示例：

```matlab
A(:,1) = [xA1; yA1; zA1];
A(:,2) = [xA2; yA2; zA2];
```

---

#### 2. `B`

```matlab
B: 3×m double
```

含义：

- 动平台本体系下的绳索连接点；
- 每一列对应一根绳索。

示例：

```matlab
B(:,1) = [xB1; yB1; zB1];
B(:,2) = [xB2; yB2; zB2];
```

---

#### 3. `L`

```matlab
L: m×1 double
```

含义：

- 当前时刻各根绳索长度；
- 本阶段仿真中直接给出；
- 暂不考虑由电机转角换算绳长。

示例：

```matlab
L = [L1; L2; L3; L4; L5; L6; L7; L8];
```

---

#### 4. `x0`

```matlab
x0: 6×1 double
```

含义：

- 正运动学求解的初始猜测；
- 建议格式为：

```matlab
x0 = [px0; py0; pz0; roll0; pitch0; yaw0];
```

其中角度单位为 **rad**。

实时仿真时推荐使用上一时刻求解结果作为初始值：

```matlab
x0 = x_last;
```

---

#### 5. `options`

```matlab
options: struct，可选
```

用于设置求解器参数，例如：

```matlab
options.Display = 'off';
options.MaxIterations = 100;
options.FunctionTolerance = 1e-12;
options.StepTolerance = 1e-12;
```

如果调用时不输入 `options`，函数内部应使用默认参数。

---

### 6.2 输出参数

#### 1. `x_sol`

```matlab
x_sol: 6×1 double
```

求解得到的末端位姿：

```matlab
x_sol = [px; py; pz; roll; pitch; yaw];
```

---

#### 2. `info`

```matlab
info: struct
```

建议包含以下字段：

```matlab
info.residual        % 最终残差向量
info.resnorm         % 残差平方和
info.exitflag        % 求解器退出标志
info.output          % 求解器输出信息
info.L_pred          % 根据求解位姿反算出的绳长
info.length_error    % L_pred - L
info.R               % 求解得到的旋转矩阵
info.p               % 求解得到的位置
```

---

## 7. 残差函数设计

建议单独编写残差函数：

```matlab
cdpr_length_residual.m
```

函数接口：

```matlab
function r = cdpr_length_residual(x, A, B, L)
```

---

### 7.1 输入

```matlab
x: 6×1
A: 3×m
B: 3×m
L: m×1
```

---

### 7.2 输出

```matlab
r: m×1
```

其中：

\[
r_i = \hat{L}_i - L_i
\]

---

### 7.3 计算流程

对于每一根绳：

1. 从 `x` 中取出位置和欧拉角：

```matlab
p = x(1:3);
eul = x(4:6);
```

2. 计算旋转矩阵：

```matlab
R = eulZYX_to_R(eul);
```

3. 计算动平台连接点在世界系中的坐标：

```matlab
Pi = p + R * B(:, i);
```

4. 计算预测绳长：

```matlab
L_pred_i = norm(A(:, i) - Pi);
```

5. 计算残差：

```matlab
r(i) = L_pred_i - L(i);
```

---

## 8. 欧拉角转旋转矩阵函数

建议单独编写：

```matlab
eulZYX_to_R.m
```

函数接口：

```matlab
function R = eulZYX_to_R(eul)
```

其中：

```matlab
eul = [roll; pitch; yaw];
```

采用 ZYX 旋转顺序：

\[
R = R_z(yaw) R_y(pitch) R_x(roll)
\]

即：

\[
R = R_z(\psi) R_y(\theta) R_x(\phi)
\]

---

## 9. 主求解流程

主函数 `cdpr_forward_kinematics` 的内部流程如下。

### Step 1：输入检查

检查：

```matlab
size(A,1) == 3
size(B,1) == 3
size(A,2) == size(B,2)
length(L) == size(A,2)
length(x0) == 6
```

如果不满足，直接报错。

---

### Step 2：整理输入维度

将 `L` 和 `x0` 强制整理为列向量：

```matlab
L = L(:);
x0 = x0(:);
```

---

### Step 3：定义残差函数句柄

```matlab
fun = @(x) cdpr_length_residual(x, A, B, L);
```

---

### Step 4：设置优化器参数

推荐优先使用 `lsqnonlin`：

```matlab
optim_options = optimoptions('lsqnonlin', ...
    'Display', options.Display, ...
    'Algorithm', 'levenberg-marquardt', ...
    'MaxIterations', options.MaxIterations, ...
    'FunctionTolerance', options.FunctionTolerance, ...
    'StepTolerance', options.StepTolerance);
```

---

### Step 5：调用非线性最小二乘求解

```matlab
[x_sol, resnorm, residual, exitflag, output] = ...
    lsqnonlin(fun, x0, [], [], optim_options);
```

---

### Step 6：计算输出信息

根据 `x_sol` 重新计算：

```matlab
p_sol = x_sol(1:3);
R_sol = eulZYX_to_R(x_sol(4:6));
L_pred = cdpr_predict_length(x_sol, A, B);
length_error = L_pred - L;
```

并打包到 `info`：

```matlab
info.p = p_sol;
info.R = R_sol;
info.residual = residual;
info.resnorm = resnorm;
info.exitflag = exitflag;
info.output = output;
info.L_pred = L_pred;
info.length_error = length_error;
```

---

## 10. 可选：预测绳长函数

建议编写一个独立函数：

```matlab
cdpr_predict_length.m
```

函数接口：

```matlab
function L_pred = cdpr_predict_length(x, A, B)
```

作用：

> 已知位姿，计算各根绳索的理论长度。

这个函数既可以用于正运动学残差计算，也可以用于验证结果。

计算公式：

\[
\hat{L}_i = \| A_i - (p + R B_i) \|
\]

---

## 11. 推荐的文件结构

建议组织为：

```text
cdpr_forward_kinematics/
│
├── main_test_fk.m
│
├── cdpr_forward_kinematics.m
├── cdpr_length_residual.m
├── cdpr_predict_length.m
├── eulZYX_to_R.m
│
└── README.md
```

其中：

- `main_test_fk.m`：测试脚本；
- `cdpr_forward_kinematics.m`：正运动学主函数；
- `cdpr_length_residual.m`：绳长残差函数；
- `cdpr_predict_length.m`：位姿到绳长的预测函数；
- `eulZYX_to_R.m`：欧拉角转旋转矩阵函数；
- `README.md`：工程说明。

---

## 12. 测试脚本设计

测试脚本 `main_test_fk.m` 建议按以下流程写。

### Step 1：定义几何参数

```matlab
A = [...];   % 3×m
B = [...];   % 3×m
```

---

### Step 2：设定一个真实位姿

```matlab
p_true = [0.2; -0.1; 0.1];
eul_true = [10; 5; -8] * pi / 180;
x_true = [p_true; eul_true];
```

---

### Step 3：用真实位姿生成绳长

```matlab
L = cdpr_predict_length(x_true, A, B);
```

此时得到的 `L` 相当于仿真中的测量绳长。

---

### Step 4：设置初值

```matlab
x0 = [0; 0; 0; 0; 0; 0];
```

---

### Step 5：调用正运动学

```matlab
[x_sol, info] = cdpr_forward_kinematics(A, B, L, x0);
```

---

### Step 6：比较结果

```matlab
disp('真实位姿：');
disp(x_true);

disp('求解位姿：');
disp(x_sol);

disp('位姿误差：');
disp(x_sol - x_true);

disp('绳长残差：');
disp(info.length_error);
```

---

## 13. 实时仿真中的调用方式

如果仿真中每个周期都给出绳长：

```matlab
L_now = 当前时刻绳长;
```

则可以写成：

```matlab
x_last = 初始位姿;

for k = 1:N

    L_now = L_history(:, k);

    [x_now, info] = cdpr_forward_kinematics(A, B, L_now, x_last);

    pose_history(:, k) = x_now;

    x_last = x_now;

end
```

核心思想：

> 当前时刻的初值使用上一时刻求出的位姿。

这样可以提高收敛速度和稳定性。

---

## 14. 初值设计原则

正运动学求解对初值非常敏感。

推荐顺序如下：

### 14.1 仿真第一帧

第一帧可以使用：

```matlab
x0 = [0; 0; 0; 0; 0; 0];
```

前提是动平台初始位姿确实接近世界坐标系原点和零姿态。

如果初始位置已知，应使用真实初始位姿附近的值。

---

### 14.2 后续帧

后续每一帧推荐使用：

```matlab
x0 = x_last;
```

其中 `x_last` 是上一帧正运动学求得的位姿。

---

### 14.3 初值过差的风险

如果初值过差，可能出现：

- 不收敛；
- 收敛到错误构型；
- 姿态角跳变；
- 得到镜像解；
- 残差较小但实际姿态不对。

因此，实时正运动学不要每一帧都从零开始求。

---

## 15. 求解器选择

### 15.1 推荐方案：`lsqnonlin`

MATLAB 中推荐使用：

```matlab
lsqnonlin
```

原因：

- 直接适合非线性最小二乘问题；
- 不需要手动写目标函数平方和；
- 可以直接返回残差；
- 工程上调试方便。

---

### 15.2 备选方案：`fminunc`

如果没有 Optimization Toolbox，可以用 `fminunc`，将目标函数写成：

\[
J(x)=r(x)^T r(x)
\]

MATLAB 形式：

```matlab
cost = @(x) sum(cdpr_length_residual(x, A, B, L).^2);
x_sol = fminunc(cost, x0);
```

但是 `fminunc` 不如 `lsqnonlin` 直接。

---

### 15.3 备选方案：手写 Gauss-Newton

后续如果要移植到 C++ 或 TwinCAT C++，可以考虑手写迭代：

\[
\Delta x = -(J^T J)^{-1}J^T r
\]

或者更稳地写成：

\[
(J^T J + \lambda I)\Delta x = -J^T r
\]

这就是 Levenberg-Marquardt 的思想。

当前 MATLAB 仿真阶段不必手写，先用 `lsqnonlin` 验证模型正确性。

---

## 16. 收敛判断

求解结束后，至少检查以下量。

### 16.1 残差平方和

```matlab
info.resnorm
```

越接近 0，说明求出的位姿越能匹配当前绳长。

---

### 16.2 各根绳长误差

```matlab
info.length_error
```

即：

\[
\hat{L}_i - L_i
\]

应在允许误差范围内。

---

### 16.3 退出标志

```matlab
info.exitflag
```

如果 `exitflag <= 0`，说明求解可能失败，需要检查：

- 初值是否合理；
- 绳长是否与机构几何匹配；
- A、B 点是否设置错误；
- 单位是否统一；
- 姿态角单位是否使用 rad。

---

## 17. 单位约定

为了避免错误，建议统一：

- 长度单位：m；
- 角度单位：rad；
- 矩阵 `A`、`B`、`p`、`L` 全部使用 m；
- 欧拉角输入输出全部使用 rad。

如果需要显示角度，再转换为 deg：

```matlab
eul_deg = eul_rad * 180 / pi;
```

---

## 18. 常见错误

### 18.1 A 和 B 的坐标系混用

错误情况：

```matlab
B(:,i)
```

本来应该是动平台本体系下的点，却误填成了世界坐标。

正确理解：

\[
P_i = p + R B_i
\]

只有经过 \(p + RB_i\) 后，动平台连接点才被转换到世界坐标系。

---

### 18.2 绳长顺序不一致

必须保证：

```matlab
A(:,i), B(:,i), L(i)
```

对应的是同一根绳。

如果第 1 根绳的固定点、动平台连接点、绳长对不上，求解结果会错误。

---

### 18.3 角度单位错误

MATLAB 三角函数使用 rad。

错误：

```matlab
eul = [10; 5; -8];   % 这会被当成rad，不是deg
```

正确：

```matlab
eul = [10; 5; -8] * pi / 180;
```

---

### 18.4 初值每次都设为零

实时仿真中不推荐：

```matlab
x0 = zeros(6,1);
```

每个周期都从零开始容易导致收敛慢或跳解。

推荐：

```matlab
x0 = x_last;
```

---

### 18.5 欧拉角奇异

如果 pitch 接近 \(\pm 90^\circ\)，ZYX 欧拉角会出现奇异。

当前阶段可以先使用欧拉角，因为实现简单。

后续如果姿态变化范围较大，建议改成：

- 四元数；
- 或旋转向量；
- 或 \(SE(3)\) 上的李代数更新。

---

## 19. 当前阶段不考虑的内容

本设计文档暂不处理：

1. 电机编码器值到绳长的转换；
2. 卷筒半径；
3. 卷筒层绕；
4. 绳索弹性；
5. 张力约束；
6. 松绳检测；
7. 绳索与结构干涉；
8. 雅可比矩阵解析推导；
9. 四元数姿态优化；
10. 动力学正解。

这些内容可以在正运动学主流程跑通后逐步加入。

---

## 20. 后续可扩展方向

### 20.1 增加解析 Jacobian

当前可以先让 `lsqnonlin` 使用数值差分 Jacobian。

后续为了提高速度，可以推导并加入解析 Jacobian。

绳长对位姿的小扰动关系近似为：

\[
\delta L_i =
u_i^T \delta p
+
(RB_i \times u_i)^T \delta \theta
\]

其中：

\[
u_i =
\frac{p + RB_i - A_i}
{\|p + RB_i - A_i\|}
\]

因此第 \(i\) 行 Jacobian 可写为：

\[
J_i =
\begin{bmatrix}
u_i^T & (RB_i \times u_i)^T
\end{bmatrix}
\]

该形式适合后续手写 Gauss-Newton 或 Levenberg-Marquardt。

---

### 20.2 改为四元数姿态

欧拉角版本适合快速验证。

后续如果需要更稳定的姿态计算，可以将状态改成：

\[
x =
\begin{bmatrix}
p \\
q
\end{bmatrix}
\]

其中 \(q\) 为单位四元数。

但需要额外处理：

\[
\|q\| = 1
\]

通常需要每次迭代后归一化，或者用最小三参数扰动更新四元数。

---

### 20.3 加入多传感器融合

实际 CDPR 中，仅靠绳长正运动学可能不够稳定。

后续可加入：

- IMU；
- 动捕系统；
- 视觉测量；
- 张力传感器；

构成状态估计问题，例如：

- EKF；
- UKF；
- 非线性优化估计。

当前 MATLAB 仿真阶段先不引入。

---

## 21. 设计总结

当前 CDPR 正运动学函数的核心思路是：

1. 建立绳长几何模型：

\[
\hat{L}_i = \| A_i - (p + R B_i) \|
\]

2. 用当前位姿预测绳长；

3. 将预测绳长与已知绳长作差，得到残差：

\[
r_i = \hat{L}_i - L_i
\]

4. 构造非线性最小二乘问题：

\[
\min_x \|r(x)\|^2
\]

5. 使用 MATLAB 的 `lsqnonlin` 求解；

6. 输出末端位姿：

\[
x =
\begin{bmatrix}
p_x & p_y & p_z & roll & pitch & yaw
\end{bmatrix}^T
\]

7. 实时仿真中，使用上一时刻位姿作为当前求解初值。

一句话总结：

> CDPR 正运动学不是直接代公式算出位姿，而是通过绳长几何约束构造非线性最小二乘问题，再迭代求出最符合当前绳长的一组末端位姿。
