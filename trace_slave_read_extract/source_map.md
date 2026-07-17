# Source Map

本目录的模块化代码来自以下项目源码逻辑：

| 源文件 | 行号 | 内容 |
| --- | --- | --- |
| `hardwareinterface.cpp` | 29-46 | Trace 原始字节按小端有符号数解码。 |
| `hardwareinterface.cpp` | 57-61 | 500 us Trace 基准周期、样本队列上限等常量。 |
| `hardwareinterface.h` | 155-163 | 张力传感器读取结果和 Trace 样本结构。 |
| `hardwareinterface.h` | 339-399 | Trace 状态、对象配置、缓存队列等成员。 |
| `hardwareinterface.cpp` | 2053-2082 | Trace 状态重置和停止。 |
| `hardwareinterface.cpp` | 2141-2370 | 运行期 Trace 对象配置、`dmc_trace_set_config()`、`dmc_trace_add_config_object()`、`dmc_trace_data_start()`。 |
| `hardwareinterface.cpp` | 2372-2609 | `dmc_trace_get_state()`、`dmc_trace_get_data()`、帧布局推断、逐对象解码和样本入队。 |
| `hardwareinterface.cpp` | 2645-2681 | 张力传感器原始值换算、缓存读取。 |
| `hardwareinterface.cpp` | 2783-2863 | Trace 缓存读取结果和样本队列取出。 |
| `controlworker.cpp` | 630-705 | 上层控制线程按 Trace 样本队列处理 500 us 传感器帧。 |

未纳入模块的相关诊断代码：

| 源文件 | 行号 | 内容 |
| --- | --- | --- |
| `hardwareinterface.cpp` | 2875-3021 | `nmc_*_pdo_trace_*` 一次性 PDO Trace 探测。 |
| `hardwareinterface.cpp` | 3023-3170 | 通用 `dmc_trace_*` 一次性探测。 |
| `mainwindow.cpp` | 2539-2680 | UI 触发 PDO Trace / 通用 Trace 测试。 |

`runtime_trace_slave_reader.*` 聚焦实时读取路径，不包含 UI、Qt 线程派发、诊断日志、历史记录和运动控制业务状态。
