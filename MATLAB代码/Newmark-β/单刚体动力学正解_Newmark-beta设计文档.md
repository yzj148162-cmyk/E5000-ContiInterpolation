# 单刚体动力学正解数值求解设计文档（Newmark-beta 方法）

## 1. 问题设定

本文档用于说明当前工程中 `rigid_body_newmark.m` 使用的单刚体动力学正解算法。

该方法采用：

- 位置 + ZYX 欧拉角作为广义坐标；
- Newmark-beta 方法进行二阶时间积分；
- 固定点迭代求解隐式的下一步广义加速度；
- 平动方程在 world frame 下建立；
- 转动欧拉方程在 body frame 下建立。

当前工程的公共算例参数由：

```matlab
rigid_body_problem.m
```

统一提供，包括质量、惯量、初始条件、时间步长、Newmark 参数以及外力/力矩定义。

外力/力矩统一由：

```matlab
rigid_body_wrench.m
```

返回：

\[
F_W(t,q,\dot q)
\]

和：

\[
M_B(t,q,\dot q)
\]

其中：

- \(F_W\)：world frame 下的外力；
- \(M_B\)：body frame 下、绕质心的外力矩。

---

## 2. 状态变量

Newmark 主程序采用广义坐标：

\[
q=
\begin{bmatrix}
x\\y\\z\\\phi\\\theta\\\psi
\end{bmatrix}
\]

其中：

- \(x,y,z\)：刚体质心在 world frame 下的位置；
- \(\phi,\theta,\psi\)：ZYX 欧拉角；
- \(\phi\)：roll；
- \(\theta\)：pitch；
- \(\psi\)：yaw。

广义速度为：

\[
\dot q=
\begin{bmatrix}
\dot x\\\dot y\\\dot z\\\dot\phi\\\dot\theta\\\dot\psi
\end{bmatrix}
=
\begin{bmatrix}
v_W\\\dot e
\end{bmatrix}
\]

其中：

\[
\dot e=
\begin{bmatrix}
\dot\phi\\\dot\theta\\\dot\psi
\end{bmatrix}
\]

广义加速度为：

\[
\ddot q=
\begin{bmatrix}
\ddot x\\\ddot y\\\ddot z\\\ddot\phi\\\ddot\theta\\\ddot\psi
\end{bmatrix}
\]

主程序中对应的数组为：

```matlab
Q      % q
Q_dot  % q_dot
A      % q_ddot
```

---

## 3. 坐标系和欧拉角运动学

### 3.1 欧拉角速度到体角速度

代码中不直接把欧拉角速度当作刚体真实角速度，而是通过矩阵 \(B(\phi,\theta)\) 转换：

\[
\omega_B = B(\phi,\theta)\dot e
\]

其中：

\[
B=
\begin{bmatrix}
1 & 0 & -\sin\theta\\
0 & \cos\phi & \sin\phi\cos\theta\\
0 & -\sin\phi & \cos\phi\cos\theta
\end{bmatrix}
\]

对应文件：

```matlab
B_matrix.m
```

该关系表示：

- \(\dot e\)：欧拉角速度；
- \(\omega_B\)：body frame 下的真实角速度。

### 3.2 角加速度关系

由：

\[
\omega_B = B\dot e
\]

对时间求导：

\[
\dot\omega_B = B\ddot e + \dot B\dot e
\]

因此：

\[
\ddot e = B^{-1}\left(\dot\omega_B-\dot B\dot e\right)
\]

代码中对应：

```matlab
Bdot = B_dot_matrix(phi, theta, eul_dot(1), eul_dot(2));
eul_ddot = B \ (omega_dot_body - Bdot * eul_dot);
```

---

## 4. 连续动力学方程

动力学右端由：

```matlab
compute_acc(q, q_dot, t)
```

计算。

### 4.1 平动方程

平动在 world frame 下计算：

\[
m\ddot r_W = F_W
\]

因此：

\[
\ddot r_W = \frac{F_W}{m}
\]

代码中：

```matlab
[F_world, M_body] = rigid_body_wrench(t, q, q_dot, cfg);
a_trans = F_world / m;
```

### 4.2 转动欧拉方程

转动在 body frame 下计算：

\[
I_B\dot\omega_B+\omega_B\times I_B\omega_B=M_B
\]

整理得：

\[
\dot\omega_B =
I_B^{-1}
\left(
M_B-\omega_B\times I_B\omega_B
\right)
\]

代码中：

```matlab
omega_dot_body = I_body \ (M_body - cross(omega_body, I_body * omega_body));
```

然后通过欧拉角运动学关系得到：

\[
\ddot e = B^{-1}\left(\dot\omega_B-\dot B\dot e\right)
\]

最终：

\[
\ddot q=
\begin{bmatrix}
\ddot r_W\\
\ddot e
\end{bmatrix}
\]

---

## 5. Newmark-beta 时间积分格式

第 \(n\) 个时间点已知：

\[
q_n,\quad \dot q_n,\quad \ddot q_n
\]

时间步长为：

\[
h=t_{n+1}-t_n
\]

Newmark-beta 格式为：

\[
q_{n+1}
=q_n+h\dot q_n
h^2
\left[
\left(\frac{1}{2}-\beta\right)\ddot q_n
+\beta\ddot q_{n+1}
\right]
\]

\[
\dot q_{n+1}
=\dot q_n
+h
\left[
(1-\gamma)\ddot q_n
+\gamma\ddot q_{n+1}
\right]
\]

当前工程使用：

```matlab
beta = 0.25;
gamma = 0.5;
```

这对应经典的平均加速度 Newmark 方法。

对于线性系统，该参数组合常用于获得良好的稳定性；对于当前非线性刚体问题，仍需要通过迭代使下一步加速度和动力学方程一致。

---

## 6. 预测-校正形式

为了实现方便，代码将 Newmark 公式拆成预测部分和校正部分。

### 6.1 预测项

先计算不含 \(\ddot q_{n+1}\) 的部分：

\[
q_p
=q_n+h\dot q_n
\frac{1}{2}h^2(1-2\beta)\ddot q_n
\]

\[
\dot q_p
=\dot q_n+h(1-\gamma)\ddot q_n
\]

代码中：

```matlab
q_p = q_n + h*q_dot_n + 0.5*h^2*(1 - 2*beta)*a_n;
q_dot_p = q_dot_n + h*(1 - gamma)*a_n;
```

### 6.2 校正项

如果给定一个下一步加速度猜测：

\[
a_{guess}\approx \ddot q_{n+1}
\]

则有：

\[
q_{n+1}
=q_p+\beta h^2 a_{guess}
\]

\[
\dot q_{n+1}
=\dot q_p+\gamma h a_{guess}
\]

代码中：

```matlab
q = q_p + beta*h^2 * a_guess;
q_dot = q_dot_p + gamma*h * a_guess;
```

---

## 7. 隐式加速度求解

由于：

\[
\ddot q_{n+1}=f(q_{n+1},\dot q_{n+1},t_{n+1})
\]

而 \(q_{n+1}\) 和 \(\dot q_{n+1}\) 又依赖 \(\ddot q_{n+1}\)，所以这是一个隐式问题。

当前工程使用固定点迭代：

1. 初始猜测：

\[
a_{guess}^{(0)}=\ddot q_n
\]

2. 根据当前加速度猜测计算：

\[
q^{(j)} = q_p+\beta h^2 a_{guess}^{(j)}
\]

\[
\dot q^{(j)} = \dot q_p+\gamma h a_{guess}^{(j)}
\]

3. 调用动力学右端：

\[
a_{new}^{(j)}
=f(q^{(j)},\dot q^{(j)},t_{n+1})
\]

4. 计算残差：

\[
r^{(j)}=
\left\|
a_{new}^{(j)}-a_{guess}^{(j)}
\right\|
\]

5. 若：

\[
r^{(j)}<tol
\]

则认为收敛。

6. 否则更新：

\[
a_{guess}^{(j+1)}=a_{new}^{(j)}
\]

继续迭代。

代码中：

```matlab
a_guess = a_n;
converged = false;

for iter = 1:max_iter
    q = q_p + beta*h^2 * a_guess;
    q_dot = q_dot_p + gamma*h * a_guess;

    a_new = compute_acc(q, q_dot, tn + h);
    residual = norm(a_new - a_guess);

    if residual < tol
        converged = true;
        break;
    end

    a_guess = a_new;
end
```

若未收敛，程序使用最后一次 \(a_{new}\) 重新同步 \(q\) 和 \(\dot q\)，并给出 warning：

```matlab
q = q_p + beta*h^2 * a_new;
q_dot = q_dot_p + gamma*h * a_new;
warning('Newmark step %d did not converge, residual = %e', i, residual);
```

---

## 8. 一个时间步的完整流程

第 \(n\) 步到第 \(n+1\) 步的计算流程如下：

1. 已知：

\[
q_n,\quad \dot q_n,\quad a_n
\]

2. 计算时间步长：

\[
h=t_{n+1}-t_n
\]

3. 预测：

\[
q_p,\quad \dot q_p
\]

4. 令：

\[
a_{guess}=a_n
\]

5. 固定点迭代：

   - 由 \(a_{guess}\) 得到 \(q,\dot q\)；
   - 调用 `compute_acc` 得到 \(a_{new}\)；
   - 检查 \(\|a_{new}-a_{guess}\|\)；
   - 若不收敛，令 \(a_{guess}=a_{new}\)。

6. 收敛或达到最大迭代次数后，更新：

\[
q_{n+1}=q
\]

\[
\dot q_{n+1}=\dot q
\]

\[
a_{n+1}=a_{new}
\]

7. 保存到：

```matlab
Q(:, i+1)
Q_dot(:, i+1)
A(:, i+1)
```

---

## 9. MATLAB 风格伪代码

```matlab
q_n = q0;
q_dot_n = q_dot0;
a_n = compute_acc(q0, q_dot0, 0);

for i = 1:N-1
    tn = t(i);
    h = t(i+1) - tn;

    q_p = q_n + h*q_dot_n + 0.5*h^2*(1 - 2*beta)*a_n;
    q_dot_p = q_dot_n + h*(1 - gamma)*a_n;

    a_guess = a_n;

    for iter = 1:max_iter
        q = q_p + beta*h^2 * a_guess;
        q_dot = q_dot_p + gamma*h * a_guess;

        a_new = compute_acc(q, q_dot, tn + h);
        residual = norm(a_new - a_guess);

        if residual < tol
            break;
        end

        a_guess = a_new;
    end

    q_n = q;
    q_dot_n = q_dot;
    a_n = a_new;

    Q(:, i+1) = q_n;
    Q_dot(:, i+1) = q_dot_n;
    A(:, i+1) = a_n;
end
```

---

## 10. 后处理量

### 10.1 能量

程序计算：

\[
E=T+V_g
\]

其中：

\[
T=
\frac{1}{2}m v_W^T v_W
+
\frac{1}{2}\omega_B^T I_B\omega_B
\]

\[
V_g=mgz
\]

注意：当前算例中存在时变外力和时变力矩，因此 \(T+V_g\) 不一定守恒。

如果 \(z<0\)，则 \(V_g=mgz<0\)，总量 \(T+V_g\) 也可能为负。这不代表动能计算错误，而是势能零点选在 \(z=0\)。

### 10.2 全局坐标系角速度

先由欧拉角速度计算体角速度：

\[
\omega_B=B\dot e
\]

再用姿态矩阵转到 world frame：

\[
\omega_W=R_{WB}\omega_B
\]

代码中：

```matlab
omega_body = B * eul_dot;
omega_world(:,i) = R * omega_body;
```

### 10.3 全局坐标系角加速度

先计算 body frame 下角加速度：

\[
\alpha_B = \dot\omega_B = B\ddot e+\dot B\dot e
\]

再转到 world frame：

\[
\alpha_W=R_{WB}\alpha_B
\]

代码中：

```matlab
alpha_body = B * eul_ddot + Bdot * eul_dot;
alpha_world(:,i) = R * alpha_body;
```

---

## 11. 与 RK4 方法的主要区别

### 11.1 状态变量不同

Newmark-beta 方法当前使用：

\[
q=[p_W;e]
\]

作为二阶系统状态，直接积分：

\[
q,\quad \dot q,\quad \ddot q
\]

RK4 方法使用：

\[
x=[p_W;v_W;q_{WB};\omega_B]
\]

作为一阶系统状态，其中姿态由四元数表示。

### 11.2 积分思想不同

Newmark-beta：

- 面向二阶动力学；
- 通过预测-校正公式推进 \(q,\dot q\)；
- 当前实现中需要迭代求 \(\ddot q_{n+1}\)。

RK4：

- 面向一阶状态方程；
- 通过四个阶段斜率 \(K_1,K_2,K_3,K_4\) 推进状态；
- 姿态四元数每步归一化。

### 11.3 姿态表达不同

Newmark-beta：

- 主状态直接使用 ZYX 欧拉角；
- 需要 \(B\) 和 \(\dot B\) 处理欧拉角速度与体角速度之间的关系；
- 可能遇到欧拉角奇异性。

RK4：

- 主状态使用四元数；
- 避免欧拉角奇异性；
- 输出绘图时再转换成欧拉角。

---

## 12. 实现注意事项

1. 当前 Newmark 实现使用固定点迭代，不是 Newton-Raphson 迭代。
2. 若外力、力矩或姿态耦合较强，固定点迭代可能收敛变慢甚至不收敛。
3. `beta=0.25`、`gamma=0.5` 对线性系统常见，但非线性刚体系统仍要关注步长和迭代收敛。
4. ZYX 欧拉角在 \(\cos\theta\approx 0\) 时会接近奇异。
5. 当前外力 \(F_W\) 已经在 world frame 下表达，因此平动方程没有再做 body-to-world 转换。
6. 当前力矩 \(M_B\) 在 body frame 下表达，因此转动方程直接使用体坐标欧拉方程。
7. 若未来将外力改为 body frame 表达，则需要先通过 \(R_{WB}\) 转换：

\[
F_W=R_{WB}F_B
\]

8. 若要进行严格能量校验，需要把非保守外力和外力矩做功也纳入能量平衡。

