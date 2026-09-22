# CDPR-G302 六维力交互离线回放

本目录是独立 MATLAB 工程。Qt 只导出运行 CSV 和同名
`*_replay_config.json`，不会启动、调用或依赖 MATLAB。

## 使用方法

1. 在 Qt 的“六维力交互”页完成阶段 B/C 空转。运行结束时程序会自动在
   CSV 同目录生成回放 JSON，也可点击“导出回放配置”重新生成。
2. 启动 MATLAB，将本目录设为当前文件夹。
3. 执行：

   ```matlab
   result = replay_force_interaction_run;
   ```

   文件选择框会自动打开 Qt 运行记录目录并选中最新的
   `*_replay_config.json`；需要分析其他批次时再改选对应文件。
4. 逐帧结果会写到 JSON 所在目录的同名 `*_matlab_replay` 子目录，
   包括 CSV、MAT、摘要 JSON 和轨迹图。动画默认只显示有效交互段。
   同一窗口左侧同步显示完整正二十面体（12顶点、20面、30边）、8根
   绳索及末端轨迹，右侧滚动显示进入动力学模型的全局系三维力，以及
   XYZ期望位置和Trace正运动学实际位置。

可选参数示例：

```matlab
result = replay_force_interaction_run( ...
    'D:\records\run_replay_config.json', ...
    'Animate', true, ...
    'IncludeBraking', true, ...
    'WriteVideo', true, ...
    'FrameStride', 2, ...
    'MaxFrames', inf);
```

## 数据原则

- JSON 是样机几何、绞盘、轴映射和单位换算的唯一参数源。
- CSV 中每行八轴 `axis_safety_relative_trace_position_*` 必须来自同一
  Trace 帧；不按八根轴各自延迟错开后拼成虚拟平台。
- Trace 延迟只用于 Qt 在线跟随误差诊断，不用于改变正运动学所需的
  同时刻八轴实际状态。
- 正运动学按全部有效帧计算，动画抽帧仅影响显示速度，不影响分析结果。
- 三维力曲线读取 `platform_wrench_0..2`，即坐标变换、滤波和门限处理后
  真正进入动力学求解的全局系平台力，不是传感器坐标系原始值。
- 现有回放JSON只含8个绳索连接点；动画按G302正二十面体几何恢复另外
  4个非连接顶点并生成完整30条等长边。未来若JSON提供
  `platform_body_vertices_local_mm`，则优先使用显式完整顶点。
- 根目录旧《MATLAB代码》仅是开发时的算法参考，本目录不 `addpath`、
  不调用其中任何函数。

需要 Optimization Toolbox 时优先使用 `lsqnonlin`；没有该工具箱时会
自动退回 `fminsearch`，但速度和边界收敛性会弱一些。

安装或修改 MATLAB 后可运行 `test_forward_kinematics_smoke`，它不读取
实机日志，只检查坐标变换和八绳正运动学是否能回到一个已知位姿。
`test_replay_pipeline_smoke` 还会在临时目录构造一份最小 CSV/JSON，检查
导入、逐帧复算和结果文件生成的整条离线链路。
