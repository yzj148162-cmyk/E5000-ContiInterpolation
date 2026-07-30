# CDPR 基于雷赛 E5000 连续插补的软件设计文档

> 本文档是在《CDPR 基于雷赛 E5000 连续插补的实时运动下发方案》的基础上，进一步面向后续控制程序开发整理的软件设计说明。  
> 目标不是直接给出完整可运行代码，而是说明软件整体框架、模块职责、数据流、状态机、关键接口、缓冲策略和核心伪代码，使后续编程实现有明确依据。

---

## 1. 设计目标

### 1.1 系统目标

本软件用于实现 8 绳 / 8 电机 CDPR 的自由漂浮效果模拟。系统在运行过程中：

```text
采集末端动平台外部力旋量
        ↓
上位机动力学求解，第一版采用 Newmark-β 法
        ↓
得到末端未来一个上位机周期后的运动状态
        ↓
对末端运动段进行 1 ms 插值
        ↓
CDPR 逆运动学得到 8 个电机的位置序列
        ↓
通过雷赛 E5000 连续插补缓冲区下发给控制卡
        ↓
控制卡前瞻 + 连续插补 + EtherCAT 总线周期刷新
        ↓
伺服电机驱动 CDPR 动平台运动
```

最终希望动平台在外力作用下表现出较自然、顺滑、可感知延迟不明显的自由漂浮运动效果。

---

### 1.2 软件设计核心思想

本方案不采用 Windows 上位机严格 1 ms 周期直接写伺服目标位置的方式，而采用：

```text
上位机 10 ms 级动力学计算
+ 1 ms 末端空间插值
+ 50 ms 延迟执行队列
+ 雷赛 E5000 连续插补缓冲区
+ 前瞻模式
+ mark 水位监控
+ 速度倍率反馈调节
```

核心原因是：

```text
1. Windows / Qt 线程不适合承担严格 1 ms 实时下发；
2. 雷赛 E5000 连续插补适合执行多段小线段轨迹；
3. CDPR 末端轨迹需要由上位机计算，控制卡只执行 8 轴电机位置序列；
4. 50 ms 延迟队列可以保证板卡缓冲区有余粮，避免连续插补断粮自动结束；
5. 1 ms 小线段可与 EtherCAT 总线周期形成较自然的执行粒度；
6. 前瞻可改善大量小线段之间的速度连接。
```

---

## 2. 关键约定

### 2.1 时间参数

第一版建议采用以下参数：

| 参数 | 建议值 | 说明 |
|---|---:|---|
| EtherCAT 总线周期 | 1 ms | 控制卡向伺服刷新目标的底层节拍 |
| 上位机动力学周期 | 10 ms | Newmark-β 积分步长，可后续改为 1 ms |
| 末端插值周期 | 1 ms | 将 10 ms 末端状态段细分为 10 个点 |
| 固定执行延迟 | 50 ms | 控制卡执行约 50 ms 之前算好的轨迹 |
| 板卡缓冲目标余量 | 40 ms | 板卡内尽量保持约 40 条 1 ms 小段 |
| 板卡缓冲下限 | 30 ms | 低于该值认为余量偏低 |
| 板卡缓冲上限 | 50 ms | 高于该值认为余量偏高 |

---

### 2.2 坐标与变量约定

| 符号 | 含义 |
|---|---|
| `F_k` | 第 k 个上位机周期采集到的末端外部力旋量 |
| `X_k` | 第 k 个周期的末端状态，包含位置、速度、姿态、角速度等 |
| `X_{k+1}` | Newmark-β 法求得的下一周期末端状态 |
| `Pose_i` | 1 ms 插值得到的中间末端位姿点 |
| `Q_i` | 由末端位姿逆运动学得到的 8 轴电机位置 |
| `Q_i.pos[8]` | 8 个电机的目标位置，单位为雷赛 `unit` |
| `mark` | 每条压入板卡的连续插补指令编号 |

---

### 2.3 绝对模式与相对模式选择

第一版建议使用 `dmc_conti_line_unit` 的绝对模式：

```cpp
posi_mode = 1;
```

原因：

```text
1. 每个 Q_i 表示 8 个电机在该 1 ms 轨迹点的绝对目标位置；
2. 不容易因为累加误差导致位置漂移；
3. 更方便和逆运动学输出结果对应；
4. 调试时容易比对理论电机位置与控制卡目标位置。
```

---

## 3. 软件总体架构

### 3.1 推荐模块划分

软件建议分为以下模块：

```text
MainController
├── ForceSensorInterface        外力传感器接口
├── StateEstimator              状态维护与反馈处理
├── DynamicsSolver              Newmark-β 动力学求解器
├── EndTrajectoryInterpolator   末端 1 ms 插值器
├── CDPRKinematics              CDPR 逆运动学模块
├── HostTrajectoryQueue         上位机 50 ms 延迟轨迹队列
├── E5000ContiInterface         雷赛 E5000 连续插补接口封装
├── BufferLevelController       板卡缓冲区水位与速度倍率控制
├── SafetyMonitor               安全监控与异常处理
└── Logger                      数据记录与调试输出
```

各模块不一定都要独立成类，但建议软件逻辑上保持这些边界，避免后续代码混乱。

---

### 3.2 总体数据流

```text
ForceSensorInterface
        ↓ F_k
DynamicsSolver
        ↓ X_{k+1}
EndTrajectoryInterpolator
        ↓ Pose_1ms
CDPRKinematics
        ↓ Q_1ms
HostTrajectoryQueue
        ↓ 延迟 50 ms 后输出 Q
E5000ContiInterface
        ↓ dmc_conti_line_unit
雷赛 E5000 控制卡
        ↓ EtherCAT
伺服驱动器 / 电机
```

---

### 3.3 线程建议

第一版可以采用 3 个主要线程或定时任务。

#### 线程 1：动力学计算线程

周期：10 ms。

职责：

```text
1. 采集外力；
2. 执行 Newmark-β 动力学积分；
3. 生成 10 个 1 ms 末端插值点；
4. 对每个点做逆运动学；
5. 将 10 个 MotorPoint 推入上位机延迟队列。
```

#### 线程 2：板卡补点线程

周期：建议 2~5 ms。

职责：

```text
1. 读取 currentMark；
2. 估算板卡内未执行时间余量；
3. 若余量不足，从延迟队列取点并压入板卡；
4. 更新 pushedMark；
5. 调用速度水位控制模块调整速度倍率。
```

#### 线程 3：状态与安全监控线程

周期：10~50 ms。

职责：

```text
1. 查询连续插补运行状态；
2. 查询轴状态、报警、限位、急停；
3. 记录日志；
4. 必要时触发减速停止。
```

如果第一版程序较简单，也可以将线程 2 和线程 3 合并，但建议将动力学计算和板卡补点解耦。

---

## 4. 主要数据结构设计

### 4.1 末端状态结构

```cpp
struct EndState
{
    double t;              // 理论时间，单位 s
    Vector3 p;             // 末端位置，世界坐标系
    Vector3 v;             // 末端线速度
    Quaternion q;          // 末端姿态，第一版也可以先用欧拉角
    Vector3 omega;         // 末端角速度
};
```

说明：

```text
1. 若第一版姿态变化很小，可先使用欧拉角，但软件接口建议预留四元数；
2. 动力学内部状态与插值状态应保持一致；
3. 后续如果改用 RK4 / 四元数积分，可直接替换 DynamicsSolver。
```

---

### 4.2 末端位姿结构

```cpp
struct EndPose
{
    double t;
    Vector3 p;
    Quaternion q;
};
```

`EndPose` 只表示几何位姿，用于插值和逆运动学，不包含速度、加速度等动力学变量。

---

### 4.3 电机位置点结构

```cpp
struct MotorPoint
{
    double t;              // 理论轨迹时间
    double pos[8];         // 8 个电机绝对目标位置，单位 unit
    long localIndex;        // 上位机生成序号，可用于调试
};
```

说明：

```text
1. pos[8] 的顺序必须与 axisList[8] 一一对应；
2. t 用于调试和日志，不直接传入 dmc_conti_line_unit；
3. localIndex 用于确认队列、压入、执行之间是否丢点。
```

---

### 4.4 上位机延迟队列

```cpp
class HostTrajectoryQueue
{
public:
    void push(const MotorPoint& p);
    bool hasDelay(double delay_s, double now_s) const;
    bool popDelayedPoint(double delay_s, double now_s, MotorPoint& out);
    size_t size() const;
    double timeSpan() const;
};
```

队列逻辑：

```text
1. 动力学线程不断向队尾写入新生成的 1 ms 电机位置点；
2. 板卡补点线程从队头读取已经满足 50 ms 延迟条件的点；
3. 队列既是时间延迟器，也是上位机侧轨迹缓存。
```

注意：

```text
实际实现中要加互斥锁或无锁队列，避免多线程读写冲突。
```

---

## 5. 主要模块设计

## 5.1 ForceSensorInterface

### 职责

```text
1. 读取六维力传感器数据；
2. 完成传感器坐标系到动平台坐标系 / 世界坐标系的转换；
3. 做零偏补偿和简单滤波；
4. 输出动力学求解所需的外力旋量。
```

### 关键接口

```cpp
class ForceSensorInterface
{
public:
    bool initialize();
    bool readWrench(Wrench& wrench_out);
    void setZeroOffset(const Wrench& zero);
    Wrench transformToBodyOrWorld(const Wrench& raw);
};
```

### 输出

```cpp
struct Wrench
{
    double force[3];   // Fx, Fy, Fz
    double torque[3];  // Tx, Ty, Tz
};
```

### 注意事项

```text
1. 力传感器采样最好带时间戳；
2. 外力噪声会直接影响自由漂浮效果，建议低通滤波；
3. 过滤不能太重，否则人手推动会有明显延迟；
4. 坐标系方向和符号必须在实验前验证。
```

---

## 5.2 DynamicsSolver

### 职责

```text
使用 Newmark-β 法，根据当前末端状态和当前外力，计算下一个上位机周期的末端状态。
```

### 关键接口

```cpp
class DynamicsSolver
{
public:
    void setTimeStep(double dt);
    void setParameters(double beta, double gamma);
    EndState step(const EndState& Xk, const Wrench& Fk);
};
```

### 第一版参数

```text
β = 0.25
γ = 0.5
dt = 0.010 s
```

### 伪代码

```cpp
EndState DynamicsSolver::step(const EndState& Xk, const Wrench& Fk)
{
    // 1. 根据当前状态和外力计算等效加速度
    // 2. 使用 Newmark-β 预测/校正位置和速度
    // 3. 姿态部分第一版可小角度近似，后续改为四元数积分
    // 4. 输出 X_{k+1}
}
```

### 注意事项

```text
1. 如果上位机周期最终改为 1 ms，Newmark 步长也可改为 1 ms；
2. 但第一版建议先用 10 ms，降低上位机实时压力；
3. 速度不为 0 时，即使外力为 0，动平台也应继续运动；
4. 动力学状态不能只由当前外力决定，还必须继承上一周期速度和姿态状态。
```

---

## 5.3 EndTrajectoryInterpolator

### 职责

```text
将 X_k 到 X_{k+1} 的 10 ms 末端运动段细分为 10 个 1 ms 末端位姿点。
```

### 关键接口

```cpp
class EndTrajectoryInterpolator
{
public:
    std::vector<EndPose> interpolate(
        const EndState& Xk,
        const EndState& Xk1,
        double interp_dt
    );
};
```

### 第一版插值方法

```text
位置：线性插值
姿态：若姿态变化很小，可先用欧拉角线性插值；更推荐预留四元数 SLERP 接口
```

### 伪代码

```cpp
std::vector<EndPose> interpolate(const EndState& Xk,
                                 const EndState& Xk1,
                                 double interp_dt)
{
    std::vector<EndPose> result;
    int N = round((Xk1.t - Xk.t) / interp_dt);  // 10ms / 1ms = 10

    for (int i = 1; i <= N; ++i)
    {
        double alpha = double(i) / double(N);

        EndPose p;
        p.t = Xk.t + i * interp_dt;
        p.p = (1 - alpha) * Xk.p + alpha * Xk1.p;
        p.q = slerp(Xk.q, Xk1.q, alpha);  // 第一版也可简化

        result.push_back(p);
    }

    return result;
}
```

---

## 5.4 CDPRKinematics

### 职责

```text
将每个 1 ms 末端位姿点转换为 8 个电机的绝对位置 unit。
```

### 关键接口

```cpp
class CDPRKinematics
{
public:
    MotorPoint inverseToMotorPoint(const EndPose& pose);
};
```

### 处理流程

```text
EndPose
    ↓
计算动平台上 8 个绳索连接点的世界坐标
    ↓
计算每根绳长
    ↓
考虑卷筒半径、减速比、编码器分辨率、脉冲当量
    ↓
转换成雷赛 unit
    ↓
输出 MotorPoint.pos[8]
```

### 注意事项

```text
1. pos[8] 顺序必须和 axisList[8] 保持一致；
2. 绳长增加对应电机正转还是反转，需要通过实验标定；
3. 如果控制卡使用绝对模式，必须保证电机当前位置和软件内部 Q 当前值一致；
4. 逆运动学输出应做限幅检查，避免超出电机软限位。
```

---

## 5.5 E5000ContiInterface

### 职责

封装雷赛 E5000 连续插补相关库函数，隔离底层 API，使上层软件不直接散乱调用控制卡函数。

### 关键接口

```cpp
class E5000ContiInterface
{
public:
    bool initializeCard();
    bool enableAxes();
    bool configureConti();
    bool openList();
    bool pushLinePoint(const MotorPoint& p, long mark);
    bool start();
    bool stop(bool emergency);
    bool closeList();

    long readCurrentMark();
    long readRemainSpace();
    short readRunState();
    bool changeSpeedRatio(double ratio);
};
```

### 内部参数

```cpp
WORD card = 0;
WORD crd = 0;
WORD axisNum = 8;
WORD axisList[8] = {0,1,2,3,4,5,6,7};
```

### 初始化顺序

```cpp
bool E5000ContiInterface::configureConti()
{
    // 1. 设置前瞻，必须在 open_list 前
    dmc_conti_set_lookahead_mode(card, crd, 1, 0, 0, 0);

    // 2. 设置矢量速度曲线
    dmc_set_vector_profile_unit(card, crd, 0, baseMaxVel, Tacc, Tdec, 0);

    // 3. 设置 S 曲线平滑
    dmc_set_vector_s_profile(card, crd, 0, s_para);

    // 4. 打开连续插补缓冲区
    dmc_conti_open_list(card, crd, axisNum, axisList);

    return true;
}
```

### 压点接口

```cpp
bool E5000ContiInterface::pushLinePoint(const MotorPoint& p, long mark)
{
    short ret = dmc_conti_line_unit(
        card,
        crd,
        axisNum,
        axisList,
        const_cast<double*>(p.pos),
        1,      // 绝对模式
        mark
    );

    return ret == 0;
}
```

---

## 5.6 BufferLevelController

### 职责

```text
1. 根据 pushedMark 和 currentMark 估算板卡内轨迹余量；
2. 根据轨迹点间距估计参考速度倍率；
3. 根据缓冲区水位调整速度倍率；
4. 调用 dmc_conti_change_speed_ratio 实现运行中整体调速。
```

### 关键接口

```cpp
class BufferLevelController
{
public:
    double computeBufferTimeMs(long pushedMark, long currentMark);
    double computeTrajectoryRatio(const std::deque<MotorPoint>& recentPoints);
    double computeBufferCorrection(double bufferTimeMs);
    double updateRatio(double ratio_ref, double bufferTimeMs);
};
```

### 速度控制逻辑

```text
ratio_ref：由轨迹段长度估计出的参考倍率
ratio_buffer：由板卡缓冲水位得出的修正倍率
ratio_cmd = ratio_ref × ratio_buffer
ratio_cmd 再经过限幅和低通滤波
```

### 伪代码

```cpp
double BufferLevelController::updateRatio(double ratio_ref, double bufferTimeMs)
{
    double ratio_buffer = 1.0;

    if (bufferTimeMs < 30.0)
    {
        // 余量偏低，控制卡消费偏快，适当降速
        ratio_buffer = 0.85;
    }
    else if (bufferTimeMs > 50.0)
    {
        // 余量偏高，控制卡消费偏慢，可略微提速
        ratio_buffer = 1.05;
    }

    double ratio = ratio_ref * ratio_buffer;

    ratio = clamp(ratio, 0.2, 1.0);

    // 平滑，避免速度倍率跳变
    ratio = 0.9 * lastRatio + 0.1 * ratio;
    lastRatio = ratio;

    return ratio;
}
```

### 说明

```text
1. 第一版可以先固定 ratio = 0.5 跑通流程；
2. 跑通后再启用轨迹速度估计；
3. 最后再加入水位反馈；
4. 不建议一开始就把速度闭环做得很激进。
```

---

## 5.7 SafetyMonitor

### 职责

```text
1. 监控控制卡运行状态；
2. 监控轴报警、限位、急停；
3. 监控缓冲区是否即将断粮；
4. 监控电机目标位置、速度估计是否超限；
5. 触发安全停机。
```

### 关键异常

```text
1. dmc_conti_get_run_state 显示连续插补已停止，但软件仍处于 Running；
2. pushedMark - currentMark 低于安全下限；
3. dmc_conti_remain_space 接近 0，说明缓冲区可能写满；
4. 电机目标位置超过软限位；
5. 逆运动学失败或绳长不可行；
6. 外力传感器数据异常；
7. 伺服报警或急停。
```

### 停止策略

正常停止：

```cpp
dmc_conti_stop_list(card, crd, 0);  // 减速停止
dmc_conti_close_list(card, crd);
```

异常急停：

```cpp
dmc_conti_stop_list(card, crd, 1);  // 立即停止，具体以手册 stop_mode 定义为准
```

注意：

```text
运行过程中不要频繁执行 stop → close → open → start。
只有异常或重新初始化时才使用该流程。
```

---

## 6. 软件状态机设计

### 6.1 状态定义

```cpp
enum class SystemState
{
    Idle,           // 空闲
    Initializing,   // 初始化控制卡和参数
    PreFilling,     // 预填充上位机队列和板卡缓冲区
    Ready,          // 已准备好，可启动
    Running,        // 正在运行
    Stopping,       // 正在停止
    Error           // 异常状态
};
```

---

### 6.2 状态转移

```text
Idle
  ↓ start command
Initializing
  ↓ 初始化成功
PreFilling
  ↓ 上位机队列 ≥ 50 ms，板卡预压 ≥ 40 ms
Ready
  ↓ dmc_conti_start_list 成功
Running
  ↓ stop command
Stopping
  ↓ 停止完成
Idle
```

异常情况下：

```text
任意状态
  ↓ error detected
Error
  ↓ reset command
Idle
```

---

### 6.3 启动流程伪代码

```cpp
bool MainController::start()
{
    state = SystemState::Initializing;

    if (!forceSensor.initialize()) return enterError();
    if (!e5000.initializeCard()) return enterError();
    if (!e5000.enableAxes()) return enterError();
    if (!e5000.configureConti()) return enterError();

    state = SystemState::PreFilling;

    // 先让动力学线程运行，积累 50 ms 延迟队列
    while (hostQueue.timeSpan() < 0.050)
    {
        runOneDynamicsStep();
    }

    // 再向板卡预压 40 ms 轨迹
    long pushed = 0;
    for (int i = 0; i < 40; ++i)
    {
        MotorPoint p;
        if (!hostQueue.popDelayedPoint(0.050, getTime(), p)) return enterError();
        if (!e5000.pushLinePoint(p, pushed)) return enterError();
        pushed++;
    }

    pushedMark = pushed;

    state = SystemState::Ready;

    if (!e5000.start()) return enterError();

    state = SystemState::Running;
    return true;
}
```

---

## 7. 主循环设计

### 7.1 动力学计算循环，10 ms

```cpp
void MainController::dynamicsLoop10ms()
{
    if (state != SystemState::Running && state != SystemState::PreFilling)
        return;

    Wrench Fk;
    if (!forceSensor.readWrench(Fk))
    {
        safety.reportForceSensorError();
        return;
    }

    EndState Xk1 = dynamics.step(currentState, Fk);

    std::vector<EndPose> poses = interpolator.interpolate(currentState, Xk1, 0.001);

    for (const EndPose& pose : poses)
    {
        MotorPoint q = kinematics.inverseToMotorPoint(pose);
        hostQueue.push(q);
    }

    currentState = Xk1;
}
```

---

### 7.2 板卡补点循环，2~5 ms

```cpp
void MainController::feedLoop()
{
    if (state != SystemState::Running)
        return;

    long currentMark = e5000.readCurrentMark();
    long remainSpace = e5000.readRemainSpace();

    double bufferTimeMs = (pushedMark - currentMark) * 1.0;

    while (bufferTimeMs < 40.0 && remainSpace > 100)
    {
        MotorPoint p;
        if (!hostQueue.popDelayedPoint(0.050, getTime(), p))
        {
            // 上位机队列暂时没有满足延迟的数据，不强行压点
            break;
        }

        bool ok = e5000.pushLinePoint(p, pushedMark);
        if (!ok)
        {
            safety.reportCardPushError();
            break;
        }

        pushedMark++;
        bufferTimeMs += 1.0;
        remainSpace--;
    }

    double ratio_ref = estimateRatioFromRecentTrajectory();
    double ratio_cmd = bufferController.updateRatio(ratio_ref, bufferTimeMs);
    e5000.changeSpeedRatio(ratio_cmd);
}
```

---

### 7.3 安全监控循环，10~50 ms

```cpp
void MainController::monitorLoop()
{
    short runState = e5000.readRunState();
    long currentMark = e5000.readCurrentMark();
    long remainSpace = e5000.readRemainSpace();

    double bufferTimeMs = (pushedMark - currentMark) * 1.0;

    logger.record(runState, currentMark, pushedMark, remainSpace, bufferTimeMs);

    if (state == SystemState::Running)
    {
        if (bufferTimeMs < 10.0)
        {
            safety.reportBufferAlmostEmpty();
        }

        if (runState indicates stopped unexpectedly)
        {
            safety.reportUnexpectedStop();
        }
    }
}
```

其中 `runState indicates stopped unexpectedly` 需要根据雷赛函数实际返回值确认后实现。

---

## 8. 速度参数设置细化

### 8.1 基础速度参数

```cpp
struct ContiSpeedConfig
{
    double baseMaxVel;   // 插补坐标系最大矢量速度，unit/s
    double tAcc;         // 加速时间，s
    double tDec;         // 减速时间，s
    double sPara;        // S 曲线平滑时间，s
};
```

第一版建议：

```text
baseMaxVel：先按最大允许绳速保守换算
Tacc：0.05 s
Tdec：0.05 s
sPara：0.02 s
```

---

### 8.2 由轨迹估计速度倍率

```cpp
double estimateRatioFromRecentTrajectory()
{
    // 取最近 M 个准备下发或已经下发的点
    // 计算每段 8 轴空间长度 L_i
    // V_i = L_i / 0.001
    // ratio_ref = mean(V_i) / baseMaxVel
}
```

建议：

```text
1. M 可取 5~20；
2. ratio_ref 要限幅，比如 [0.2, 1.0]；
3. 轨迹速度估计只是辅助，不要求每段严格 1 ms。
```

---

### 8.3 由缓冲水位修正速度

```text
目标：bufferTimeMs 稳定在 40 ms 左右。
```

简单规则：

```text
bufferTimeMs < 30 ms：降低速度倍率
30 ms ≤ bufferTimeMs ≤ 50 ms：保持正常
bufferTimeMs > 50 ms：略微提高速度倍率
```

后续可改成比例控制：

```text
error = bufferTimeMs - targetBufferTimeMs
ratio_buffer = 1 + Kp * error
```

注意：

```text
Kp 不宜过大，否则速度会随缓冲区波动而振荡。
```

---

## 9. 实验阶段对应的软件配置

### 9.1 阶段一：单电机连续插补框架测试

目标：验证控制卡连续插补基本流程，而不是验证多轴直线插补。

使用函数：

```cpp
dmc_conti_pmove_unit(...)
```

测试内容：

```text
1. open_list 是否成功；
2. pmove 指令是否能压入；
3. start 是否成功；
4. currentMark 是否可读；
5. remainSpace 是否可读；
6. stop 和 close 是否正常；
7. speed ratio 是否有效。
```

此阶段可不接入 Newmark-β 和 CDPR 逆运动学，只生成简单的单轴位置序列。

---

### 9.2 阶段二：双电机连续直线插补测试

目标：验证 `dmc_conti_line_unit` 的 2 轴插补。

配置：

```cpp
WORD axisNum = 2;
WORD axisList[2] = {0, 1};
```

测试轨迹：

```text
两轴同步匀速小步运动，例如每 1 ms 一个目标点；
先用离线生成的简单轨迹，不接入力传感器。
```

测试内容：

```text
1. 两轴是否同步启动和同步运动；
2. 多条 1 ms 小段能否连续执行；
3. 前瞻开启后是否平滑；
4. mark 是否递增；
5. 水位反馈是否能稳定余量。
```

---

### 9.3 阶段三：8 电机 CDPR 运行测试

目标：验证完整自由漂浮软件链路。

逐步打开功能：

```text
1. 8 轴离线轨迹连续插补；
2. 8 轴末端轨迹 → 逆运动学 → 电机位置连续插补；
3. 加入 Newmark-β 动力学；
4. 加入力传感器实时采样；
5. 加入 50 ms 延迟队列；
6. 加入速度水位反馈；
7. 调整延迟和前瞻参数。
```

---

## 10. 日志与调试数据设计

建议记录以下数据：

| 数据 | 说明 |
|---|---|
| `time` | 上位机时间 |
| `F_k` | 外力旋量 |
| `X_k` | 末端动力学状态 |
| `Q_i[8]` | 8 轴目标位置 |
| `pushedMark` | 已压入板卡的最大 mark |
| `currentMark` | 板卡当前执行 mark |
| `bufferTimeMs` | 估算板卡轨迹余量 |
| `remainSpace` | 连续插补缓冲区剩余空间 |
| `ratio_cmd` | 当前速度倍率 |
| `runState` | 连续插补运行状态 |
| `errorCode` | 控制卡函数返回值 |

日志用途：

```text
1. 判断板卡是否跑快或跑慢；
2. 判断 50 ms 延迟是否稳定；
3. 判断外力变化到末端响应的实际延迟；
4. 判断速度倍率是否振荡；
5. 复盘异常停机原因。
```

---

## 11. 异常处理策略

### 11.1 队列不足

现象：

```text
hostDelayQueue 中没有满足 50 ms 延迟的数据。
```

处理：

```text
1. 不强行压入无效点；
2. 若板卡 bufferTimeMs 仍足够，则等待下一周期；
3. 若 bufferTimeMs 低于安全下限，则降低速度倍率；
4. 若继续下降，则减速停止。
```

---

### 11.2 板卡余量过低

现象：

```text
pushedMark - currentMark < 10
```

处理：

```text
1. 立即降低速度倍率；
2. 优先补点；
3. 若无法补点，触发安全减速停止；
4. 不建议压入零位移点保活。
```

---

### 11.3 逆运动学失败

现象：

```text
某个末端位姿不可达、绳长超限或计算异常。
```

处理：

```text
1. 不压入该点；
2. 可将末端轨迹限幅到可达范围；
3. 若连续失败，减速停止；
4. 记录失败位姿和原因。
```

---

### 11.4 控制卡意外停止

现象：

```text
软件状态为 Running，但 dmc_conti_get_run_state 显示已停止。
```

处理：

```text
1. 立即进入 Error 状态；
2. 停止动力学继续下发；
3. 查询错误码和轴状态；
4. 人工确认后再重新 open/start。
```

---

## 12. 后续可扩展方向

### 12.1 上位机周期从 10 ms 改为 1 ms

若后续计算压力允许，可将 Newmark-β 积分周期改为 1 ms。此时：

```text
1. 末端插值模块可以取消或只用于姿态平滑；
2. 每个上位机周期生成 1 个 MotorPoint；
3. 仍然保留 50 ms 延迟队列；
4. 板卡缓冲水位逻辑不变。
```

---

### 12.2 姿态插值升级

第一版若姿态变化很小，可简化处理。后续建议：

```text
1. 内部姿态统一用四元数；
2. 末端姿态插值使用 SLERP；
3. 逆运动学中使用旋转矩阵计算动平台连接点位置。
```

---

### 12.3 前瞻 PathError 调整

第一版：

```text
PathError = 0
```

后续若小线段段间仍有顿挫，可尝试非常小的 `PathError`，观察是否改善平滑性，同时检查末端轨迹误差和张力变化。

---

## 13. 最小开发顺序建议

```text
1. 封装 E5000ContiInterface，先能 open/start/stop/close；
2. 完成 dmc_conti_pmove_unit 单轴测试；
3. 完成 dmc_conti_line_unit 双轴测试；
4. 实现 mark 递增、currentMark 读取和 bufferTimeMs 估算；
5. 实现 HostTrajectoryQueue；
6. 实现 50 ms 延迟队列 + 离线轨迹补点；
7. 实现 8 轴离线轨迹运行；
8. 接入 CDPR 逆运动学；
9. 接入 Newmark-β 动力学；
10. 接入力传感器；
11. 加入速度水位反馈；
12. 做整机自由漂浮效果测试。
```

---

## 14. 一句话总结

本软件设计的本质是：

```text
上位机负责实时计算“应该怎样运动”，包括外力采样、动力学积分、末端插值和逆运动学；
雷赛 E5000 负责把已经算好的 8 轴电机位置小线段连续、平滑、同步地执行出来。
```

通过 **50 ms 延迟执行队列** 和 **mark 水位反馈**，软件可以在不依赖 Windows 严格 1 ms 实时性的前提下，尽量稳定地向控制卡提供连续轨迹，从而实现 CDPR 自由漂浮效果的工程验证。
