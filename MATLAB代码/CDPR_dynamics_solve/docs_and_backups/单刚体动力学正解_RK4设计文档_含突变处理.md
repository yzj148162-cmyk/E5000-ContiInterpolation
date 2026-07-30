# 单刚体动力学正解数值求解设计文档（含 FOH/ZOH 突变力处理）

## 1. 问题设定

本设计文档用于实现单刚体动平台在外部六维力传感器作用下的动力学正解数值求解。

先将六维力传感器输出从 sensor frame 变换至位于刚体质心的 body frame 下表达：

\[
wrench_b=
\begin{bmatrix}
f_b\\
\tau_b
\end{bmatrix}
\]

其中：

\[
f_b\in \mathbb{R}^3
\]

为 body frame 下的外力；

\[
\tau_b\in \mathbb{R}^3
\]

为 body frame 下、绕刚体质心的外力矩。

采样周期为：

\[
h=t_{k+1}-t_k
\]

第 \(k\) 个积分区间定义为：

\[
I_k=[t_k,t_{k+1}]
\]

区间两端的传感器采样为：

\[
U_k=wrench_{b,k},\qquad U_{k+1}=wrench_{b,k+1}
\]

当前工程支持三种输入处理方式：

1. **ZOH：零阶保持**，整个区间使用左端采样 \(U_k\)；
2. **FOH：一阶保持**，平滑力段使用 \(U_k\) 与 \(U_{k+1}\) 对区间 \(I_k\) 做线性重构；
3. **Jump-ZOH：突变力段零阶保持**，当 \(U_k\rightarrow U_{k+1}\) 被判断为突变时，不使用 FOH，而是整个区间 \(I_k\) 仍使用旧值 \(U_k\)。

FOH 不是外推预测未来输入，而是等 \(t_{k+1}\) 的传感器采样到达后，使用 \(t_k\) 和 \(t_{k+1}\) 两端采样值回算上一段 \([t_k,t_{k+1}]\)。因此如果用于实时执行，会天然产生一个采样周期的延迟。

工程数组长度约定为：

- \(N_{total}\)：积分区间数；
- `step_num = N_total + 1`：状态点数量；
- `wrench_raw_S`：传感器采样序列，长度为 `N_total + 1`；
- 第 \(k\) 个积分区间 \([t_k,t_{k+1}]\) 使用 `wrench_raw_S(:,k)` 和 `wrench_raw_S(:,k+1)` 两个端点采样。

---

## 2. 坐标系和状态变量

采用如下状态变量：

\[
x=
\begin{bmatrix}
p_W\\
v_W\\
q_{WB}\\
\omega_b
\end{bmatrix}
\]

其中：

- \(p_W\)：刚体质心位置，world frame 表达；
- \(v_W\)：刚体质心线速度，world frame 表达；
- \(q_{WB}\)：body frame 到 world frame 的姿态四元数；
- \(\omega_b\)：刚体角速度，body frame 表达。

四元数采用标量在前的形式：

\[
q=
\begin{bmatrix}
q_0\\
q_v
\end{bmatrix}
=
\begin{bmatrix}
q_0\\
q_1\\
q_2\\
q_3
\end{bmatrix}
\]

并满足单位四元数约束：

\[
\|q\|=1
\]

旋转矩阵由四元数得到：

\[
R_{WB}=R(q_{WB})
\]

用于将 body frame 下的向量转换到 world frame：

\[
a_W=R_{WB}a_b
\]

---

## 3. 连续动力学方程

### 3.1 平动方程

由于外力 \(f_b\) 在 body frame 下表达，先通过姿态转换到 world frame：

\[
f_W=R(q_{WB})f_b
\]

平动方程为：

\[
\dot p_W=v_W
\]

\[
\dot v_W=\frac{1}{m}R(q_{WB})f_b
\]

### 3.2 姿态四元数方程

角速度 \(\omega_b\) 写成纯四元数：

\[
\bar\omega_b=
\begin{bmatrix}
0\\
\omega_b
\end{bmatrix}
\]

四元数姿态微分方程为：

\[
\dot q_{WB}=\frac{1}{2}q_{WB}\otimes \bar\omega_b
\]

四元数乘法采用 Hamilton 乘法。若：

\[
q=\begin{bmatrix}q_0\\q_v\end{bmatrix},
\qquad
p=\begin{bmatrix}p_0\\p_v\end{bmatrix}
\]

则：

\[
q\otimes p=
\begin{bmatrix}
q_0p_0-q_v^Tp_v\\
q_0p_v+p_0q_v+q_v\times p_v
\end{bmatrix}
\]

因此：

\[
\dot q_{WB}
=\frac{1}{2}
\begin{bmatrix}
-q_v^T\omega_b\\
q_0\omega_b+q_v\times\omega_b
\end{bmatrix}
\]

### 3.3 转动欧拉方程

在 body frame 下建立欧拉方程：

\[
I_b\dot\omega_b+\omega_b\times I_b\omega_b=\tau_b
\]

整理为：

\[
\dot\omega_b=I_b^{-1}\left(\tau_b-\omega_b\times I_b\omega_b\right)
\]

其中 \(I_b\) 是 body frame 下绕质心的转动惯量矩阵。

实现中建议提前计算：

\[
I_{b,inv}=I_b^{-1}
\]

运行时使用：

\[
\dot\omega_b=I_{b,inv}\left(\tau_b-\omega_b\times I_b\omega_b\right)
\]

---

## 4. 状态方程统一形式

定义右端函数：

\[
\dot x=F(x,wrench_b)
\]

其中：

\[
F(x,wrench_b)=
\begin{bmatrix}
\dot p_W\\
\dot v_W\\
\dot q_{WB}\\
\dot\omega_b
\end{bmatrix}
=
\begin{bmatrix}
v_W\\[4pt]
\frac{1}{m}R(q_{WB})f_b\\[4pt]
\frac{1}{2}q_{WB}\otimes
\begin{bmatrix}
0\\
\omega_b
\end{bmatrix}\\[10pt]
I_{b,inv}\left(\tau_b-\omega_b\times I_b\omega_b\right)
\end{bmatrix}
\]

---

## 5. 输入重构方法

第 \(k\) 个积分区间为：

\[
I_k=[t_k,t_{k+1}]
\]

左右端点输入为：

\[
U_k=wrench_{b,k}
\]

\[
U_{k+1}=wrench_{b,k+1}
\]

在计算 RK4 之前，先根据 \(U_k\rightarrow U_{k+1}\) 的变化情况选择四个 RK4 阶段使用的输入：

\[
U^{(1)},\ U^{(2)},\ U^{(3)},\ U^{(4)}
\]

其中：

\[
U^{(i)}=
\begin{bmatrix}
f_b^{(i)}\\
\tau_b^{(i)}
\end{bmatrix}
\]

### 5.1 ZOH 输入

ZOH 情况下，整个区间使用左端采样：

\[
U^{(1)}=U^{(2)}=U^{(3)}=U^{(4)}=U_k
\]

即：

\[
wrench_b(t)=U_k,\qquad t\in[t_k,t_{k+1}]
\]

### 5.2 平滑力段 FOH 输入

如果 \(U_k\rightarrow U_{k+1}\) 被判断为平滑变化，则使用 FOH：

\[
U(t_k+\tau)=U_k+\frac{\tau}{h}(U_{k+1}-U_k),\qquad \tau\in[0,h]
\]

RK4 四个阶段输入为：

\[
U^{(1)}=U_k
\]

\[
U^{(2)}=U^{(3)}=\frac{1}{2}(U_k+U_{k+1})
\]

\[
U^{(4)}=U_{k+1}
\]

### 5.3 突变力段 Jump-ZOH 输入

如果 \(U_k\rightarrow U_{k+1}\) 被判断为突变，则不使用 FOH。

原因是 FOH 会把突变力在整个区间 \([t_k,t_{k+1}]\) 内抹成斜坡，相当于让系统在突变真正被检测到之前就提前受到了新力。

此时采用 Jump-ZOH，即该区间全部使用突变前的旧值：

\[
U^{(1)}=U^{(2)}=U^{(3)}=U^{(4)}=U_k
\]

这等价于保守地认为突变发生在区间末端 \(t_{k+1}\) 附近，而不是在整个区间内线性变化。

### 5.4 平滑/突变判断

定义：

\[
\Delta f_k=f_{b,k+1}-f_{b,k}
\]

\[
\Delta \tau_k=\tau_{b,k+1}-\tau_{b,k}
\]

若：

\[
\|\Delta f_k\|\leq F_{th}
\]

且：

\[
\|\Delta \tau_k\|\leq M_{th}
\]

则认为该区间为平滑力段，使用 FOH。

若：

\[
\|\Delta f_k\|>F_{th}
\]

或：

\[
\|\Delta \tau_k\|>M_{th}
\]

则认为该区间为突变力段，使用 Jump-ZOH。

其中 \(F_{th}\) 和 \(M_{th}\) 为工程阈值，需要根据传感器量程、滤波效果、期望微重力模拟敏感度和实验安全性调试。

---

## 6. 一拍延迟的采样、重构与执行关系

FOH 和 Jump-ZOH 都属于**延迟一拍的输入重构**。

在 \(t_{k+1}\) 时刻拿到 \(U_{k+1}\) 后，才能使用 \(U_k\) 和 \(U_{k+1}\) 判断并重构区间：

\[
I_k=[t_k,t_{k+1}]
\]

然后用 RK4 计算：

\[
X_k\rightarrow X_{k+1}
\]

如果用于实时执行，该重构结果通常在下一执行时段：

\[
I_{k+1}=[t_{k+1},t_{k+2}]
\]

中使用。因此该方法提高的是自由漂浮状态重构精度，但代价是响应延迟一个采样周期。

### 6.1 四个采样点示例

考虑采样点：

\[
t_{k-1},\ t_k,\ t_{k+1},\ t_{k+2}
\]

对应输入：

\[
U_{k-1},\ U_k,\ U_{k+1},\ U_{k+2}
\]

三个时段：

\[
I_{k-1}=[t_{k-1},t_k]
\]

\[
I_k=[t_k,t_{k+1}]
\]

\[
I_{k+1}=[t_{k+1},t_{k+2}]
\]

若突变发生在 \(I_k\)，即 \(U_k\rightarrow U_{k+1}\) 被判断为突变，则关系如下：

| 采样到达时刻 | 已知端点输入 | 被重构的时段 | 重构方法 | 该重构结果用于执行的时段 |
|---|---|---|---|---|
| \(t_k\) | \(U_{k-1},U_k\) | \(I_{k-1}\) | 平滑则 FOH，突变则 Jump-ZOH | \(I_k\) |
| \(t_{k+1}\) | \(U_k,U_{k+1}\) | \(I_k\) | 若检测为突变，使用 Jump-ZOH，即全段用 \(U_k\) | \(I_{k+1}\) |
| \(t_{k+2}\) | \(U_{k+1},U_{k+2}\) | \(I_{k+1}\) | 根据 \(U_{k+1}\rightarrow U_{k+2}\) 重新判断：平滑则 FOH，突变则 Jump-ZOH | \(I_{k+2}\) |

注意：不会用 \(U_k,U_{k+1}\) 去处理 \([t_{k+1},t_{k+2}]\)。每个区间只使用该区间自己的左右端点。

在仿真或离线重建绘图时，\(X_{k+1}\) 仍可画在物理时间 \(t_{k+1}\)，用于分析积分误差；但若在实时系统中将其作为执行轨迹，则它相对实际外力响应延迟一拍。

---

## 7. RK4 固定步长积分流程

第 \(k\) 个积分区间已知：

\[
x_k=
\begin{bmatrix}
p_{W,k}\\
v_{W,k}\\
q_{WB,k}\\
\omega_{b,k}
\end{bmatrix}
\]

以及阶段输入：

\[
U^{(1)},\ U^{(2)},\ U^{(3)},\ U^{(4)}
\]

计算四个 RK4 斜率：

\[
K_1=F(x_k,U^{(1)})
\]

\[
K_2=F\left(x_k+\frac{h}{2}K_1,U^{(2)}\right)
\]

\[
K_3=F\left(x_k+\frac{h}{2}K_2,U^{(3)}\right)
\]

\[
K_4=F\left(x_k+hK_3,U^{(4)}\right)
\]

然后更新状态：

\[
x_{k+1}=x_k+\frac{h}{6}\left(K_1+2K_2+2K_3+K_4\right)
\]

四元数更新后必须归一化：

\[
q_{WB,k+1}\leftarrow\frac{q_{WB,k+1}}{\|q_{WB,k+1}\|}
\]

---

## 8. RK4 阶段内部计算顺序

每个 RK4 阶段先构造该阶段临时状态：

\[
x_i=
\begin{bmatrix}
p_i\\
v_i\\
q_i\\
\omega_i
\end{bmatrix}
\]

然后取该阶段输入：

\[
U^{(i)}=
\begin{bmatrix}
f_b^{(i)}\\
\tau_b^{(i)}
\end{bmatrix}
\]

再按依赖关系计算该阶段导数。

### 8.1 四元数预处理

使用阶段四元数 \(q_i\) 计算旋转矩阵前，建议先进行临时归一化：

\[
q_i\leftarrow\frac{q_i}{\|q_i\|}
\]

该归一化用于当前阶段计算，不改变已经保存的主状态。

### 8.2 计算旋转矩阵

\[
R_i=R(q_i)
\]

四元数转旋转矩阵公式：

\[
R(q)=
(q_0^2-q_v^Tq_v)I_3
+2q_vq_v^T
+2q_0[q_v]_\times
\]

其中：

\[
[q_v]_\times=
\begin{bmatrix}
0&-q_3&q_2\\
q_3&0&-q_1\\
-q_2&q_1&0
\end{bmatrix}
\]

### 8.3 计算角速度导数

\[
K_{\omega,i}=I_{b,inv}\left(\tau_b^{(i)}-\omega_i\times I_b\omega_i\right)
\]

### 8.4 计算四元数导数

\[
K_{q,i}
=\frac{1}{2}q_i\otimes
\begin{bmatrix}
0\\
\omega_i
\end{bmatrix}
\]

展开为：

\[
K_{q,i}
=\frac{1}{2}
\begin{bmatrix}
-q_{v,i}^T\omega_i\\
q_{0,i}\omega_i+q_{v,i}\times\omega_i
\end{bmatrix}
\]

### 8.5 计算线速度导数

\[
K_{v,i}=\frac{1}{m}R_i f_b^{(i)}
\]

### 8.6 计算位置导数

\[
K_{p,i}=v_i
\]

因此：

\[
K_i=
\begin{bmatrix}
K_{p,i}\\
K_{v,i}\\
K_{q,i}\\
K_{\omega,i}
\end{bmatrix}
\]

---

## 9. 四个阶段展开

### 第一阶段

\[
x_1=x_k
\]

\[
K_1=F(x_1,U^{(1)})
\]

### 第二阶段

\[
x_2=x_k+\frac{h}{2}K_1
\]

即：

\[
p_2=p_k+\frac{h}{2}K_{p,1}
\]

\[
v_2=v_k+\frac{h}{2}K_{v,1}
\]

\[
q_2=q_k+\frac{h}{2}K_{q,1}
\]

\[
\omega_2=\omega_k+\frac{h}{2}K_{\omega,1}
\]

然后：

\[
K_2=F(x_2,U^{(2)})
\]

### 第三阶段

\[
x_3=x_k+\frac{h}{2}K_2
\]

即：

\[
p_3=p_k+\frac{h}{2}K_{p,2}
\]

\[
v_3=v_k+\frac{h}{2}K_{v,2}
\]

\[
q_3=q_k+\frac{h}{2}K_{q,2}
\]

\[
\omega_3=\omega_k+\frac{h}{2}K_{\omega,2}
\]

然后：

\[
K_3=F(x_3,U^{(3)})
\]

### 第四阶段

\[
x_4=x_k+hK_3
\]

即：

\[
p_4=p_k+hK_{p,3}
\]

\[
v_4=v_k+hK_{v,3}
\]

\[
q_4=q_k+hK_{q,3}
\]

\[
\omega_4=\omega_k+hK_{\omega,3}
\]

然后：

\[
K_4=F(x_4,U^{(4)})
\]

---

## 10. 状态更新展开式

\[
p_{W,k+1}
=p_{W,k}+\frac{h}{6}
\left(K_{p,1}+2K_{p,2}+2K_{p,3}+K_{p,4}\right)
\]

\[
v_{W,k+1}
=v_{W,k}+\frac{h}{6}
\left(K_{v,1}+2K_{v,2}+2K_{v,3}+K_{v,4}\right)
\]

\[
q_{WB,k+1}
=q_{WB,k}+\frac{h}{6}
\left(K_{q,1}+2K_{q,2}+2K_{q,3}+K_{q,4}\right)
\]

\[
\omega_{b,k+1}
=\omega_{b,k}+\frac{h}{6}
\left(K_{\omega,1}+2K_{\omega,2}+2K_{\omega,3}+K_{\omega,4}\right)
\]

归一化四元数：

\[
q_{WB,k+1}\leftarrow\frac{q_{WB,k+1}}{\|q_{WB,k+1}\|}
\]

如后续代码需要旋转矩阵：

\[
R_{WB,k+1}=R(q_{WB,k+1})
\]

如需要 world frame 下角速度：

\[
\omega_{W,k+1}=R_{WB,k+1}\omega_{b,k+1}
\]

---

## 11. MATLAB 风格伪代码

### 11.1 选择 RK4 阶段输入

```matlab
function [U1, U2, U3, U4, mode] = select_stage_wrench(Uk, Uk1, F_th, M_th)
    dU = Uk1 - Uk;
    df = norm(dU(1:3));
    dm = norm(dU(4:6));

    if df <= F_th && dm <= M_th
        % 平滑段：FOH
        Umid = 0.5 * (Uk + Uk1);
        U1 = Uk;
        U2 = Umid;
        U3 = Umid;
        U4 = Uk1;
        mode = "FOH";
    else
        % 突变段：Jump-ZOH，整段使用旧值 Uk
        U1 = Uk;
        U2 = Uk;
        U3 = Uk;
        U4 = Uk;
        mode = "JUMP_ZOH";
    end
end
```

### 11.2 一个区间的 RK4 积分

```matlab
function [x_next, mode] = rk4_step_with_jump_handling( ...
    x_k, U_k, U_k1, mass_ee, I_b, I_b_inv, h, F_th, M_th)

    [U1, U2, U3, U4, mode] = select_stage_wrench(U_k, U_k1, F_th, M_th);

    K1 = rigid_body_rhs(x_k,             U1, mass_ee, I_b, I_b_inv);
    K2 = rigid_body_rhs(x_k + 0.5*h*K1,  U2, mass_ee, I_b, I_b_inv);
    K3 = rigid_body_rhs(x_k + 0.5*h*K2,  U3, mass_ee, I_b, I_b_inv);
    K4 = rigid_body_rhs(x_k + h*K3,      U4, mass_ee, I_b, I_b_inv);

    x_next = x_k + h/6 * (K1 + 2*K2 + 2*K3 + K4);
    x_next(7:10) = normalize_quat(x_next(7:10));
end
```

### 11.3 主循环时间关系

```matlab
% 状态点数量 step_num = N_total + 1
% 第 k 个区间 [t_k, t_{k+1}] 使用 wrench(:,k) 和 wrench(:,k+1)

for k = 1:N_total
    U_k  = wrench_b_log(:, k);
    U_k1 = wrench_b_log(:, k+1);

    [x_next, mode_k] = rk4_step_with_jump_handling( ...
        x_curr, U_k, U_k1, mass_ee, I_b, I_b_inv, h, F_th, M_th);

    x_log(:, k+1) = x_next;
    mode_log(k) = mode_k;

    x_curr = x_next;
end
```

如果该算法用于实时执行，则第 \(k\) 段的重构结果会在第 \(k+1\) 段执行；如果用于仿真或离线误差分析，则 \(x_{k+1}\) 仍记录在物理时刻 \(t_{k+1}\)。

---

## 12. 实现要点

1. \(I_b\) 和 \(I_{b,inv}\) 在进入实时循环前提前准备好。
2. 不单独积分旋转矩阵 \(R\)，主姿态状态使用四元数 \(q_{WB}\)。
3. 每个 RK4 阶段需要由阶段四元数 \(q_i\) 计算阶段旋转矩阵 \(R_i\)。
4. 每个周期结束后必须归一化 \(q_{WB,k+1}\)。
5. 后续控制算法需要 \(R\) 时，由最新四元数实时转换：

\[
R_{WB}=R(q_{WB})
\]

6. 平滑段使用 FOH，提高连续输入的积分精度。
7. 突变段使用 Jump-ZOH，避免把真实突变力错误重构成斜坡。
8. Jump-ZOH 对突变段采用旧值 \(U_k\)，因此响应更保守；若需要更快的突变响应，需要另行设计实时预测或执行端限幅策略。
9. 本文仅处理动平台端动力学正解的输入重构问题，绳力、绳速、绳长和驱动端限幅不在本文范围内。
