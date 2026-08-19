# CDPR 力交互控制平台

这是基于 Qt Widgets、雷赛 E3000 EtherCAT 控制卡的 8 绳 6 自由度 CDPR 控制工程。当前保留单轴点动、速度模式位置闭环、转矩测试、Trace 延迟标定及早期连续插补对照功能，并已接通模拟六维力驱动8台空载电机的首版最小实时闭环。

## 当前软件分层

- `MotionControlWorker`：控制状态机、规划、反馈处理和安全判定；
- `MotionCardHardwareInterface`：独占硬件线程，作为唯一板卡访问边界；
- `LeadshineMotionCard`：集中封装 LTDMC/nmc 系列雷赛 SDK；
- `CdprCoordinator`：唯一CDPR状态协调器、配置校验和初始数学自检；
- `CdprControlTypes`：6维平台、8绳、8轴命令/反馈和机器人完整状态帧；
- `CdprKinematics`：无静态共享状态的直线绳段正逆运动学和绳速雅可比；
- `CdprDynamics`：纯惯性自由刚体Newmark-beta软件单步；
- `CdprForceInput`：定值/脉冲/正弦/公式力生成器、F/T同帧Trace适配接口及传感器到平台质心的力旋量变换；
- `NokovMarkerProvider`：仅采集全部标记点，并向样机几何位姿重建接口提供数据；
- `TelemetryRecorder`：异步数据记录。

## CDPR 样机参数约定

- 初始 8 绳长度不写死，由初始末端位姿和 8 组连接点通过逆运动学计算；
- 当前 Newmark 纯惯性模型使用动平台总成的质量与惯量，不重复维护“虚拟刚体”参数；
- 绞盘直径为 160 mm；
- 每根绳相对启动位置允许收绳和放绳各 6.5 圈，即单方向最大绳长变化约 3.2673 m；
- 电机方向与收放绳对应关系必须逐轴完成实物标定后才能进入 8 轴运行。

## 构建

工程自带雷赛 SDK 头文件、导入库和运行时 DLL。Qt安装必须包含 Widgets、
Concurrent 和 Charts 模块。在 Qt 6.8.3 MSVC 2022 x64 环境中运行：

```bat
qmake cdpr_control.pro
nmake
```

生成的程序目标名为 `cdpr_control`。

运动学离线测试位于 `tests/cdpr_kinematics_tests.pro`；Newmark与六维力变换
测试位于 `tests/cdpr_dynamics_tests.pro`。当前CDPR页包含结构参数、初始状态、
力输入、预设轨迹测试、离线验证、力交互控制和8绳监视七个子页。离线PVT已接通
“末端五次轨迹—逆运动学—虚拟绞盘—8轴PVT装表与同步启动”链路。
Nokov几何位姿重建和真实F/T Trace对象尚未接通；模拟力交互启动时会从八轴
第一组同帧Trace建立编码器基准。

当前可用8台空载电机验证三条互斥执行链：

- 模拟六维力、固定5 ms Newmark、8个独立PID和虚拟绞盘映射组成的无固定终点
  实时速度闭环；
- 当前已知五次轨迹测试使用独立主机参考轨迹缓存，逐周期调用
  `dmc_change_speed_unit`，不装PVT表且不受PVT点数上限限制；
- 整条轨迹预先已知、使用
  `dmc_pvts_table_unit + dmc_pvt_move` 调用链的离线PVT位置控制（已实现首版，
  待8台空载电机实测方向、单位和同步完成判定）。

虚拟绞盘首版统一假设电机正方向对应放绳，实物安装后再逐轴修改方向。
真实F/T数据同样由Trace采集，首版直接把六通道原始值作为N和N·m使用。

模拟力交互的基本操作顺序为：加载并校验配置、在“初始状态”建立预设位姿基准、
在“力输入”选择并校验模拟公式、初始化控制卡并使能8个映射轴，最后在
“力交互控制”点击开始。“离线验证”可先用相同公式按5 ms执行Newmark和
8绳逆运动学，但不会读取Trace或下发电机命令。PID、
速度/加速度限幅和跟随误差阈值复用“预设轨迹测试”的8轴速度闭环参数。运行
没有自动终点，需手动停止；每次运行会异步保存原始Trace、周期耗时以及
`cdpr_force_control.csv`。
