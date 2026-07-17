# CDPR 基于雷赛 E5000 连续插补的实时运动下发方案

> 适用对象：8 绳 / 8 电机 CDPR，自由漂浮效果模拟；上位机根据末端外力计算动平台运动响应，再将电机位置序列下发给雷赛 E5000 控制卡执行。  
> 当前方案偏向“运动自然、顺滑、响应人感不明显延迟”，而不是严格复现每一个动力学积分点的时间历程。

---

## 1. 方案背景与目标

### 1.1 控制目标

系统需要实现的核心过程是：

```text
末端动平台受到外部力旋量
        ↓
上位机动力学求解，例如 Newmark-β 法
        ↓
得到下一时刻末端运动状态
        ↓
CDPR 逆运动学
        ↓
得到各电机期望位置
        ↓
通过雷赛 E5000 控制卡驱动电机运动
        ↓
动平台表现出类似自由漂浮的运动效果
```

其中，控制卡直接控制的是电机轴，而不是机器人末端。因此，对于雷赛控制卡来说，最终下发的数据应该是：

```text
8 个电机轴的位置序列 Q_0, Q_1, Q_2, ...
```

而不是末端的 `x, y, z, 姿态`。

---

## 2. 对雷赛连续插补功能的基本理解

### 2.1 连续插补不是 PVT

连续插补函数 `dmc_conti_line_unit` 的核心输入是：

```cpp
Target_Pos
```

它表示参与插补的各个轴的目标位置数组，单位为 `unit`。

也就是说，连续插补缓冲区中压入的是：

```text
一条条“轨迹段指令”
```

而不是：

```text
位置 + 速度 + 时间戳
```

这与 PVT 运动不同。PVT 更接近“按时间表执行轨迹”；连续插补更接近“给控制卡一串轨迹段端点，让控制卡根据速度曲线和前瞻算法连续执行”。

因此，在连续插补方案中，如果希望电机按一定时间尺度运行，不能只给位置，还需要设置和调整速度参数。

---

### 2.2 连续插补执行的是轴空间轨迹

对于 CDPR 来说，连续插补并不知道机器人结构、绳长模型、动平台姿态，也不知道张力分配。

控制卡只知道：

```text
轴 0 到什么位置
轴 1 到什么位置
...
轴 7 到什么位置
```

因此，`dmc_conti_line_unit` 的作用可以理解为：

```text
在 8 轴电机位置空间中，从当前点 Q_i 运动到下一点 Q_{i+1}
```

它不保证 CDPR 末端在笛卡尔空间中走直线或圆弧。若希望末端轨迹符合规划，需要由上位机完成：

```text
末端轨迹规划
        ↓
末端轨迹离散
        ↓
CDPR 逆运动学
        ↓
转换为 8 轴电机位置序列
```

然后再交给控制卡执行。

---

### 2.3 连续插补点与 EtherCAT 总线周期的关系

手册没有明确说明 `dmc_conti_line_unit` 中相邻两个 `Target_Pos` 点之间的固定时间间隔。

合理理解是：

```text
上位机压入的点：轨迹段端点
控制卡内部插补点：控制卡根据速度曲线进一步细分
EtherCAT 总线周期：控制卡向驱动器刷新目标值的底层节拍
```

因此，若 EtherCAT 总线周期设为 1 ms，可以认为控制卡最终对驱动器的目标刷新受 1 ms 总线周期约束。但是，缓冲区中“一条连续插补指令”并不自动等于 1 ms。它的实际执行时间与以下因素有关：

```text
轨迹段长度
基础矢量速度
速度倍率
加减速时间
S 曲线平滑时间
是否开启前瞻
路径连接情况
```

所以，若希望“每条轨迹段大约代表 1 ms”，需要通过轨迹细分和速度参数设置，使控制卡的消费速度接近这个目标。

---

## 3. 为什么不采用“每算出一个点就压入一个点”

若上位机周期为 10 ms，每次只算出 `t_{k+1}` 的一个电机位置点，然后立即压入控制卡，会有以下风险：

```text
1. 缓冲区余量很少；
2. Windows / Qt 上位机线程一旦抖动，控制卡缓冲区可能断粮；
3. 小线段前瞻没有足够后续轨迹可看；
4. 控制卡可能因缓冲区执行完而自动结束连续插补；
5. 若速度参数偏快，控制卡消费轨迹段的速度可能超过上位机补点速度。
```

因此，必须保证控制卡缓冲区中始终有一定“余粮”。

这里的余粮不应只理解为“还有多少条指令”，而应尽量换算成“还有多少毫秒的轨迹”。

---

## 4. 本方案的核心思想

### 4.1 总体思路

采用：

```text
50 ms 延迟执行队列
+ 1 ms 末端插值
+ 连续插补小线段前瞻
+ 速度参数约束
+ 缓冲区水位反馈调速
```

也就是说，上位机实时采集外力并计算运动响应，但不把最新计算结果立即交给控制卡执行，而是先放入一个延迟队列。控制卡始终执行队列中稍早的一段轨迹。

这样既能使用真实采样到的外力数据，又能为控制卡提供足够的缓冲余量和前瞻轨迹。

---

### 4.2 时间结构

建议第一版采用：

```text
EtherCAT 总线周期：1 ms
上位机动力学周期：10 ms
末端插值周期：1 ms
固定执行延迟：50 ms
控制卡缓冲目标余量：30~50 ms
```

实际时间关系可以理解为：

```text
真实时刻 t
上位机正在根据当前外力计算最新轨迹

控制卡实际执行的是 t - 50ms 左右的轨迹
```

这会带来约 50 ms 的固定响应延迟，但可以换来更稳定的轨迹缓存和更平滑的运动。

对于“人眼或人手感觉不出明显延迟”的自由漂浮演示，50 ms 可作为初始测试值。后续如果感觉响应迟钝，可尝试降到 30 ms。

---

## 5. 数据生成流程

### 5.1 上位机 10 ms 动力学更新

每个上位机周期执行：

```text
1. 采集当前末端外力旋量 F_k；
2. 使用 Newmark-β 法，由当前状态 X_k 计算下一状态 X_{k+1}；
3. 得到末端在 t_{k+1} 时刻的位置、速度、姿态等状态；
4. 记录 X_k 到 X_{k+1} 这一段末端运动。
```

这里的 `X_k` 可以包含：

```text
位置 p
速度 v
姿态 R / 四元数 / 欧拉角
角速度 ω
```

---

### 5.2 末端空间 1 ms 插值

若上位机周期为 10 ms，总线周期为 1 ms，则把 `X_k -> X_{k+1}` 细分为 10 个 1 ms 末端轨迹点。

例如：

```text
X_k
X_{k+0.1}
X_{k+0.2}
...
X_{k+1}
```

这里的插值是在末端空间中完成，而不是直接在 8 轴电机空间插值。

原因是：

```text
CDPR 的末端位姿与绳长/电机位置之间是非线性关系。
```

先在末端空间插值，再做逆运动学，更符合机器人末端轨迹需求。

第一版可以先采用简单方法：

```text
位置：线性插值
姿态：若变化很小，可先用欧拉角线性插值；后续可改为四元数球面插值 SLERP
```

---

### 5.3 逆运动学得到电机位置

对每一个 1 ms 末端轨迹点，执行 CDPR 逆运动学：

```text
末端位姿 X_i
        ↓
计算 8 根绳长 L_i
        ↓
绳长转换为电机 unit
        ↓
得到 8 轴电机位置 Q_i
```

其中：

```text
Q_i = [q0_i, q1_i, q2_i, q3_i, q4_i, q5_i, q6_i, q7_i]
```

最终进入控制卡的是 `Q_i`，不是 `X_i`。

---

## 6. 两级缓冲结构

### 6.1 上位机延迟队列

上位机每 10 ms 生成 10 个 1 ms 电机位置点，并将它们放入延迟队列。

延迟队列的作用是：

```text
让计算结果先积累一段时间，不立即执行；
使控制卡执行的轨迹始终滞后真实计算约 50 ms；
为控制卡持续补点提供稳定来源。
```

可以维护一个队列：

```cpp
std::deque<MotorPoint> hostDelayQueue;
```

其中每个 `MotorPoint` 包含：

```cpp
struct MotorPoint
{
    double pos[8];   // 8 个电机位置，单位 unit
    double t;        // 该点对应的理论时间，可用于调试
};
```

---

### 6.2 控制卡连续插补缓冲区

控制卡连续插补缓冲区中存放的是已经压入板卡、等待执行的连续插补指令。

每压入一个 1 ms 电机位置点，就调用一次：

```cpp
dmc_conti_line_unit(...)
```

若每条指令都代表 1 ms 小段，则可以把控制卡内未执行段数近似换算为时间余量：

```text
板卡内时间余量 ≈ 未执行段数 × 1 ms
```

---

## 7. `mark` 与缓冲区水位监控

### 7.1 使用 mark 标记每条指令

每次调用 `dmc_conti_line_unit` 时，给 `mark` 传入一个递增编号：

```cpp
long pushedMark = 0;

dmc_conti_line_unit(card, crd, 8, axisList, q.pos, 1, pushedMark);
pushedMark++;
```

这样，每条连续插补指令都对应一个编号。

---

### 7.2 读取当前执行段号

运行中读取：

```cpp
long currentMark = dmc_conti_read_current_mark(card, crd);
```

它表示控制卡当前执行到的插补段号。

于是可以估算：

```text
未执行段数 ≈ pushedMark - currentMark
```

若每段为 1 ms，则：

```text
板卡轨迹余量 ms ≈ pushedMark - currentMark
```

---

### 7.3 辅助读取剩余空间

还可以读取：

```cpp
long remainSpace = dmc_conti_remain_space(card, crd);
```

它表示连续插补缓冲区剩余空间。

`remainSpace` 用于防止缓冲区写满；`currentMark` 用于估算控制卡执行进度。

两者共同使用：

```text
currentMark：判断板卡实际执行到哪里
pushedMark - currentMark：判断还有多少时间余量
remainSpace：判断还能不能继续压入更多指令
```

---

## 8. 速度参数设置方案

### 8.1 为什么需要速度参数

连续插补不是 PVT，没有每个点的时间戳。

如果我们希望每条 1 ms 小段大约用 1 ms 执行完，就必须让控制卡的矢量速度与轨迹段长度匹配。

对于第 `i` 段：

```text
Q_i → Q_{i+1}
```

轴空间长度为：

```text
L_i = ||Q_{i+1} - Q_i||
```

若希望该段约 1 ms 执行完：

```text
V_i = L_i / 0.001
```

这给出了该段的参考矢量速度。

---

### 8.2 基础速度曲线

初始化时设置连续插补的基础速度曲线：

```cpp
dmc_set_vector_profile_unit(
    card,
    crd,
    0,          // Min_Vel
    baseMaxVel, // Max_Vel
    Tacc,
    Tdec,
    0           // Stop_Vel
);
```

建议初始：

```text
baseMaxVel：根据最大允许绳速/电机速度换算，先保守设置
Tacc：0.03~0.08 s
Tdec：0.03~0.08 s
```

这里的 `Max_Vel` 是插补坐标系的矢量速度上限，不是某一个电机轴的速度。

---

### 8.3 S 曲线平滑

为减少加减速冲击，可设置 S 曲线平滑：

```cpp
dmc_set_vector_s_profile(card, crd, 0, s_para);
```

建议初始：

```text
s_para = 0.01~0.03 s
```

若设置过大，运动会变钝；若设置过小，加减速可能较硬。

---

### 8.4 速度倍率调节

运行中可用速度倍率进行整体调速：

```cpp
dmc_conti_change_speed_ratio(card, crd, ratio_cmd);
```

其中：

```text
ratio_cmd ∈ [0, 1]
```

若 `baseMaxVel = 10000 unit/s`：

```text
ratio_cmd = 1.0 约等于 10000 unit/s
ratio_cmd = 0.5 约等于 5000 unit/s
ratio_cmd = 0.2 约等于 2000 unit/s
```

---

## 9. 速度设置采用 C：轨迹速度估计 + 缓冲区水位反馈

### 9.1 轨迹速度估计

对最近若干个 1 ms 小段计算平均参考速度：

```text
V_ref_i = ||Q_{i+1} - Q_i|| / 0.001
V_ref_avg = mean(V_ref_i)
```

得到基础倍率：

```text
ratio_ref = V_ref_avg / baseMaxVel
```

---

### 9.2 缓冲区水位反馈

设定目标板卡缓冲时间：

```text
targetBufferTime = 40 ms
```

当前板卡缓冲时间估计为：

```text
bufferTime = (pushedMark - currentMark) × 1 ms
```

若：

```text
bufferTime < 30 ms
```

说明控制卡消费偏快或上位机补点不足，应适当降低速度倍率。

若：

```text
bufferTime > 50 ms
```

说明控制卡消费偏慢，可适当提高速度倍率。

---

### 9.3 综合速度倍率

最终倍率可以采用：

```text
ratio_cmd = ratio_ref × ratio_buffer
```

再进行限幅和平滑：

```text
ratio_cmd ∈ [0.2, 1.0]
ratio_cmd = 0.9 × ratio_last + 0.1 × ratio_cmd
```

这样可以避免外力噪声或轨迹微小波动导致电机速度忽快忽慢。

---

### 9.4 倍率更新周期与变化死区

缓冲水位控制器应在每个缓冲监控周期计算一次 `ratio_cmd`。第一版可采用与补点线程一致或更慢的周期，例如 `5~10 ms`；计算输入包括最新的 `currentMark`、`pushedMark`、轨迹参考速度和当前缓冲水位。

但“每周期计算”不等于“每周期都调用 `dmc_conti_change_speed_ratio`”。为避免水位估计量化、轨迹微小波动或通信抖动导致倍率频繁来回变化，应记录最后一次已下发的倍率 `ratio_applied`，设置倍率变化死区：

```text
ratioDeadband = 0.01
if |ratio_cmd - ratio_applied| < ratioDeadband：
    保持当前倍率，不下发调整命令
else：
    调用 dmc_conti_change_speed_ratio 并更新 ratio_applied
```

例如当前倍率为 `0.80`，死区为 `0.01`：计算得到 `0.805` 时保持 `0.80`；计算得到 `0.815` 时才下发新的倍率。这样既保留了连续的水位反馈控制，又能防止倍率抖动。

缓冲区进入危险区时应绕过死区：当 `bufferTime < 30 ms` 时立即执行降速策略；当缓冲区接近耗尽、Trace/通信异常或补点明显跟不上时，应优先执行减速停止或立即停止等安全策略，而不等待死区条件满足。

---

## 10. 前瞻功能设置

### 10.1 为什么需要前瞻

你的轨迹最终会被细分为大量 1 ms 小线段。

若不开前瞻，控制卡可能把各小段处理得比较独立，段间速度衔接不够平滑，可能出现：

```text
速度波动
段间顿挫
电机声音不连续
绳索张力波动
```

前瞻的作用是：

```text
控制卡提前查看缓冲区中的后续小线段，优化段间速度过渡。
```

对于你的 1 ms 小线段方案，前瞻是有价值的。

---

### 10.2 推荐设置

初始化时，在打开连续插补缓冲区之前设置前瞻：

```cpp
dmc_conti_set_lookahead_mode(
    card,
    crd,
    1,      // enable，1 表示使能前瞻
    0,      // LookaheadSegments，手册中为保留参数
    0,      // PathError，先设为 0
    0       // LookaheadAcc，手册中为保留参数
);
```

第一版建议：

```text
开启前瞻，但 PathError = 0
```

这样先利用前瞻改善速度连接，但不让控制卡为了圆弧过渡主动偏离轨迹点。

---

## 11. 实验路线

### 11.1 阶段一：单电机连续插补框架测试

目的：验证连续插补缓冲机制能跑通。

由于严格的直线插补通常按 2 轴及以上理解，单电机测试建议使用：

```cpp
dmc_conti_pmove_unit(...)
```

测试内容：

```text
1. 能否打开连续插补缓冲区；
2. 能否压入单轴连续插补运动指令；
3. 能否启动；
4. 能否读取 currentMark；
5. 能否读取 remainSpace；
6. 能否正常停止和关闭；
7. 速度倍率调整是否有效。
```

此阶段不验证多轴同步，只验证连续插补机制。

---

### 11.2 阶段二：两个电机连续直线插补测试

目的：验证 `dmc_conti_line_unit` 的多轴直线插补功能。

配置：

```cpp
WORD axisNum = 2;
WORD axisList[2] = {0, 1};
```

压入多条简单轨迹：

```cpp
double target[2] = {q0, q1};
dmc_conti_line_unit(card, crd, axisNum, axisList, target, 1, mark);
```

测试内容：

```text
1. 两轴是否能同步运动；
2. 多条小线段是否能连续执行；
3. 前瞻开启后是否更平滑；
4. currentMark 是否按 mark 递增；
5. 速度倍率是否能影响执行快慢；
6. 缓冲区水位反馈是否能维持余量。
```

---

### 11.3 阶段三：8 电机 CDPR 轴空间轨迹测试

目的：验证最终 8 轴位置序列下发。

配置：

```cpp
WORD axisNum = 8;
WORD axisList[8] = {0,1,2,3,4,5,6,7};
```

每个 1 ms 点：

```cpp
double target[8] = {
    q0, q1, q2, q3,
    q4, q5, q6, q7
};
```

调用：

```cpp
dmc_conti_line_unit(card, crd, 8, axisList, target, 1, mark);
```

测试内容：

```text
1. 8 轴能否同步连续执行；
2. 绳长变化是否平滑；
3. 末端平台是否按期望轨迹运动；
4. 50 ms 延迟是否可接受；
5. 缓冲区余量是否稳定在 30~50 ms；
6. 电机是否有异常振动、啸叫或过速；
7. 张力是否有明显波动。
```

---

## 12. 初始化与运行伪代码

### 12.1 初始化

```cpp
const WORD card = 0;
const WORD crd  = 0;
const WORD axisNum = 8;
WORD axisList[8] = {0,1,2,3,4,5,6,7};

// 1. 设置前瞻，必须在 open_list 前
// 第一版 PathError 设为 0，避免轨迹圆弧过渡偏离
dmc_conti_set_lookahead_mode(card, crd, 1, 0, 0, 0);

// 2. 设置插补速度曲线
dmc_set_vector_profile_unit(card, crd,
                            0,
                            baseMaxVel,
                            0.05,
                            0.05,
                            0);

// 3. 设置 S 曲线平滑
dmc_set_vector_s_profile(card, crd, 0, 0.02);

// 4. 打开连续插补缓冲区
dmc_conti_open_list(card, crd, axisNum, axisList);
```

---

### 12.2 上位机 10 ms 控制循环

```cpp
void controlLoop10ms()
{
    // 1. 采集外力
    Wrench Fk = readForceSensor();

    // 2. Newmark-β 动力学积分，得到下一状态
    EndState Xk1 = newmarkStep(Xk, Fk, 0.010);

    // 3. 将 Xk 到 Xk1 细分成 10 个 1ms 末端点
    for (int i = 1; i <= 10; ++i)
    {
        double alpha = i / 10.0;

        EndPose pose_i = interpolateEndPose(Xk.pose, Xk1.pose, alpha);

        // 4. CDPR 逆运动学：末端位姿 -> 8 轴电机位置
        MotorPoint Qi = cdprInverseKinematics(pose_i);

        // 5. 进入上位机延迟队列
        hostDelayQueue.push(Qi);
    }

    Xk = Xk1;
}
```

---

### 12.3 板卡补点循环

```cpp
void feedCardLoop()
{
    long currentMark = dmc_conti_read_current_mark(card, crd);

    int cardBufferedSegments = pushedMark - currentMark;
    double cardBufferTimeMs = cardBufferedSegments * 1.0;

    // 目标保持约 40ms 板卡余量
    while (cardBufferTimeMs < 40.0 && hostDelayQueue.hasDelayedData(50.0))
    {
        MotorPoint q = hostDelayQueue.popDelayedPoint(50.0);

        dmc_conti_line_unit(card,
                            crd,
                            axisNum,
                            axisList,
                            q.pos,
                            1,          // 绝对模式
                            pushedMark);

        pushedMark++;
        cardBufferedSegments++;
        cardBufferTimeMs = cardBufferedSegments * 1.0;
    }

    updateSpeedRatioByTrajectoryAndBuffer();
}
```

---

### 12.4 启动流程

```cpp
// 1. 先运行上位机控制循环，让 hostDelayQueue 积累至少 50ms 数据
waitUntilHostQueueHas(50);

// 2. 再向板卡预压约 40ms 轨迹
for (int i = 0; i < 40; ++i)
{
    MotorPoint q = hostDelayQueue.popDelayedPoint(50.0);

    dmc_conti_line_unit(card,
                        crd,
                        axisNum,
                        axisList,
                        q.pos,
                        1,
                        pushedMark);

    pushedMark++;
}

// 3. 启动连续插补
dmc_conti_start_list(card, crd);
```

---

### 12.5 停止流程

```cpp
// 减速停止
dmc_conti_stop_list(card, crd, 0);

// 关闭连续插补缓冲区
dmc_conti_close_list(card, crd);
```

运行过程中不要频繁执行：

```text
stop → close → open → start
```

除非出现异常或需要重新初始化。正常运行应通过持续补点和速度调节维持运动连续性。

---

## 13. 关键参数建议

| 参数 | 初始建议 | 说明 |
|---|---:|---|
| EtherCAT 总线周期 | 1 ms | 先用常规稳定值 |
| 上位机动力学周期 | 10 ms | 先降低上位机实时压力 |
| 末端插值周期 | 1 ms | 与总线周期对齐 |
| 固定执行延迟 | 50 ms | 稳定优先，后续可减小 |
| 板卡缓冲目标余量 | 40 ms | 即约 40 条 1ms 小段 |
| 板卡缓冲下限 | 30 ms | 低于则降速/加快补点 |
| 板卡缓冲上限 | 50 ms | 高于则可适当提速 |
| Tacc/Tdec | 0.03~0.08 s | 先保守 |
| S 平滑时间 | 0.01~0.03 s | 减少冲击 |
| PathError | 0 | 第一版避免轨迹偏离 |
| 速度倍率范围 | 0.2~1.0 | 防止过慢或过快 |

---

## 14. 需要重点验证的现象

实验中重点观察：

```text
1. dmc_conti_read_current_mark 是否稳定递增；
2. pushedMark - currentMark 是否稳定在 30~50 左右；
3. dmc_conti_remain_space 是否不会接近 0；
4. 控制卡是否会因缓冲区空而自动结束连续插补；
5. 两轴和 8 轴连续插补时是否同步；
6. 前瞻开启后运动是否更平滑；
7. 速度倍率调节是否会引入明显速度波动；
8. 50 ms 延迟是否能被人感觉出来；
9. 绳索张力是否出现明显尖峰；
10. 末端运动是否符合自由漂浮直觉。
```

---

## 15. 方案优点与局限

### 15.1 优点

```text
1. 避免 Windows/Qt 上位机必须严格 1ms 写目标位置；
2. 控制卡负责底层连续插补和 EtherCAT 周期刷新；
3. 通过 50ms 延迟队列保证缓冲区余粮；
4. 可使用前瞻改善小线段轨迹平滑性；
5. 后续真实外力采样仍会进入队列，不是用一次旧力预测很远；
6. 适合“自然、顺滑、自由漂浮感”的演示目标。
```

### 15.2 局限

```text
1. 连续插补不是 PVT，没有严格时间戳；
2. 速度只能通过速度曲线和倍率近似约束；
3. 执行存在固定延迟；
4. 已压入板卡的轨迹不适合频繁修改；
5. 若速度设置过快，板卡可能消费完缓冲区；
6. 若速度设置过慢，运动响应会变钝；
7. 前瞻参数不当可能改变轨迹连接方式，需要谨慎调试。
```

---

## 16. 最终推荐执行顺序

```text
第一步：用 dmc_conti_pmove_unit 做单电机连续插补框架测试。

第二步：用 dmc_conti_line_unit 做双电机连续直线插补测试。

第三步：验证 mark、remain_space、速度倍率、水位反馈。

第四步：引入 10ms Newmark-β + 1ms 末端插值 + 50ms 延迟队列。

第五步：扩展到 8 电机 CDPR 轴空间位置序列执行。

第六步：根据实际效果调整：
    固定延迟 50ms → 30ms；
    板卡余量 40ms → 30ms；
    S 平滑时间；
    速度倍率滤波系数；
    PathError 是否仍为 0。
```

---

## 17. 一句话总结

本方案不是让雷赛控制卡直接理解 CDPR 末端轨迹，而是：

```text
上位机负责：外力采样、Newmark-β 动力学、末端插值、逆运动学、电机位置序列生成。

雷赛 E5000 负责：多轴连续插补、缓冲执行、前瞻平滑、底层 EtherCAT 周期目标刷新。
```

通过 50 ms 延迟执行队列和 1 ms 小线段轨迹，可以在保证控制卡缓冲区有余粮的同时，尽量保留真实外力采样对自由漂浮运动的影响。
