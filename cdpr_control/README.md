# CDPR 力交互控制平台

这是基于 Qt Widgets、雷赛 E3000 EtherCAT 控制卡的 8 绳 6 自由度 CDPR 控制工程。当前保留单轴点动、速度模式位置闭环、转矩测试、Trace 延迟标定及早期连续插补对照功能，并正在接入 Newmark-beta 动力学、正逆运动学和 8 轴力交互控制。

## 当前软件分层

- `MotionControlWorker`：控制状态机、规划、反馈处理和安全判定；
- `MotionCardHardwareInterface`：独占硬件线程，作为唯一板卡访问边界；
- `LeadshineMotionCard`：集中封装 LTDMC/nmc 系列雷赛 SDK；
- `CdprCoordinator`：唯一CDPR状态协调器、配置校验和初始数学自检；
- `CdprControlTypes`：6维平台、8绳、8轴命令/反馈和机器人完整状态帧；
- `CdprKinematics`：无静态共享状态的直线绳段正逆运动学和绳速雅可比；
- `CdprDynamics`：纯惯性自由刚体Newmark-beta软件单步；
- `CdprForceInput`：模拟六维力、F/T Trace占位及传感器到平台质心的力旋量变换；
- `NokovMarkerProvider`：仅采集全部标记点，并向样机几何位姿重建接口提供数据；
- `TelemetryRecorder`：异步数据记录。

## CDPR 样机参数约定

- 初始 8 绳长度不写死，由初始末端位姿和 8 组连接点通过逆运动学计算；
- 当前 Newmark 纯惯性模型使用动平台总成的质量与惯量，不重复维护“虚拟刚体”参数；
- 卷筒直径为 160 mm；
- 每根绳相对启动位置允许收绳和放绳各 6.5 圈，即单方向最大绳长变化约 3.2673 m；
- 电机方向与收放绳对应关系必须逐轴完成实物标定后才能进入 8 轴运行。

## 构建

工程自带雷赛 SDK 头文件、导入库和运行时 DLL。在 Qt 6.8.3 MSVC 2022 x64 环境中运行：

```bat
qmake cdpr_control.pro
nmake
```

生成的程序目标名为 `cdpr_control`。

运动学离线测试位于 `tests/cdpr_kinematics_tests.pro`；Newmark与六维力变换
测试位于 `tests/cdpr_dynamics_tests.pro`。当前CDPR页包含结构参数、初始化与
输入、控制状态和8绳监视四个子页。Nokov几何位姿重建、真实F/T Trace对象、
8轴编码器启动基准和实机控制链尚未接通，因此8轴实机控制入口仍保持禁用。
