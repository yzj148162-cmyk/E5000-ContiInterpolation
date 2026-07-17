# Trace 模块说明

本文档说明 `trace_slave_read_extract` 中摘录出的 Trace 读取模块。该模块面向雷赛 EtherCAT 控制卡，通过 `dmc_trace_*` 接口批量读取从站变量，适合迁移到与本项目硬件结构相近的工程。

## 模块目标

`RuntimeTraceSlaveReader` 的目标是把原项目中分散在 `HardwareInterface` 内的实时 Trace 读取逻辑抽离为独立模块：

1. 统一配置多个从站变量。
2. 启动雷赛控制卡 Trace 缓冲。
3. 周期性批量读取 Trace 帧。
4. 将原始字节按小端有符号数解析。
5. 根据零点和比例系数换算为工程量。
6. 维护当前缓存值和按帧样本队列。

该模块只负责 Trace 采集，不包含 UI、Qt 线程派发、滤波、诊断图表、运动控制和安全策略。

## 文件关系

- `runtime_trace_slave_reader.h`：模块公开接口和配置结构。
- `runtime_trace_slave_reader.cpp`：雷赛 Trace API 调用、帧布局推断、数据解码和缓存队列。
- `README.md`：快速迁移示例。
- `source_map.md`：与原项目源码位置的对应关系。

## 核心数据结构

### `ObjectConfig`

每个 `ObjectConfig` 对应一个要 Trace 的从站变量。

| 字段 | 含义 |
| --- | --- |
| `logicalIndex` | 输出通道下标。小于 0 时按对象顺序输出。 |
| `dataType` | 雷赛 Trace 对象类型。本项目张力传感器使用 `19`。 |
| `dataIndex` | 对象索引。本项目张力传感器使用 `0x6000`。 |
| `dataSubIndex` | 子索引。本项目对应张力传感器通道号。 |
| `slaveId` | EtherCAT 从站地址。 |
| `apiDataBytes` | 传给 `dmc_trace_add_config_object()` 的数据字节参数。 |
| `valueBytes` | 实际解析该变量时使用的字节数。 |
| `home` | 零点原始值。 |
| `scale` | 原始值到工程量的比例系数。 |
| `signMode` | 原始值符号解释方式。 |

### `ReaderConfig`

`ReaderConfig` 是整个 Trace Reader 的运行配置。

| 字段 | 含义 |
| --- | --- |
| `cardNo` | 雷赛控制卡号，原项目使用 `0`。 |
| `samplePeriodUs` | 目标采样周期，原项目张力 Trace 使用 `500 us`。 |
| `traceBaseCycleUs` | Trace 基准周期，原项目按 `500 us` 换算。 |
| `lostHandle` | 传给 `dmc_trace_set_config()` 的丢帧处理参数。 |
| `traceType` | Trace 类型参数。原项目实时读取使用 `0`。 |
| `triggerObjectIndex` | 触发对象序号。实时连续采集通常为 `0`。 |
| `triggerType` | 触发类型。实时连续采集通常为 `0`。 |
| `mask` | 触发掩码。实时连续采集通常为 `0`。 |
| `condition` | 触发条件。实时连续采集通常为 `0`。 |
| `maxBufferBytes` | 单次 `dmc_trace_get_data()` 的最大缓冲区。 |
| `maxDrainReads` | 每次读取最多连续抽空 Trace 缓冲的次数。 |
| `maxQueuedSamples` | 内部样本队列最大帧数。 |
| `retryIntervalUs` | Trace 不可用后的自动重试间隔。 |
| `objects` | 需要读取的对象列表。 |

### `Sample`

`Sample.values` 是一帧中所有已配置变量换算后的工程量数组。数组下标由 `logicalIndex` 决定。

## 运行流程

1. 构造 `ReaderConfig`，填入从站变量列表。
2. 构造 `RuntimeTraceSlaveReader reader(config)` 或调用 `setConfig(config)`。
3. 调用 `configure()`，内部执行：
   - `dmc_trace_data_stop()`
   - `dmc_trace_data_reset()`
   - `dmc_trace_set_config()`
   - `dmc_trace_reset_config_object()`
   - `dmc_trace_add_config_object()`
   - `dmc_trace_data_start()`
4. 在控制循环或采集线程中周期调用 `readTraceCached()`。
5. `readTraceCached()` 内部执行：
   - `dmc_trace_get_state()` 获取有效帧数和帧大小。
   - 推断每帧中有效数据起始偏移和对象槽位宽度。
   - `dmc_trace_get_data()` 批量取出 Trace 数据。
   - 按对象顺序解析原始值。
   - 写入当前缓存并追加 `Sample` 队列。
6. 上层调用 `takeSamples()` 取走所有新增样本。
7. 如果只需要最新值，可调用 `currentValues()`。

## 帧解析策略

雷赛 Trace 返回的 `objectTotalBytes` 可能大于各对象实际字节数总和。原项目为兼容这种情况，按 `{8, 4, 2, 1}` 估算每个对象槽位宽度，并把多出的字节视为帧头。

模块保留了这个逻辑：

- 先计算对象实际值字节数总和 `expectedValueBytes`。
- 如果 `objectTotalBytes >= expectedValueBytes`，尝试寻找最合适的对象槽位宽度。
- 如果找到槽位宽度，则从推断出的帧头后开始逐对象解析。
- 如果找不到，则默认有效值位于帧尾。

这种处理来自原项目的实时读取路径，可兼容本项目 SBT908E 张力传感器和电机位置 Trace 数据。

## 本项目张力传感器映射

本项目默认读取 8 路张力传感器，配置如下：

| 输出通道 | 从站地址 | 对象索引 | 子索引 | 字节数 |
| --- | --- | --- | --- | --- |
| 0 | `1004` | `0x6000` | `1` | `2` |
| 1 | `1004` | `0x6000` | `2` | `2` |
| 2 | `1004` | `0x6000` | `3` | `2` |
| 3 | `1004` | `0x6000` | `4` | `2` |
| 4 | `1011` | `0x6000` | `1` | `2` |
| 5 | `1011` | `0x6000` | `2` | `2` |
| 6 | `1011` | `0x6000` | `3` | `2` |
| 7 | `1011` | `0x6000` | `4` | `2` |

换算公式：

```cpp
value = (raw - home) * scale;
```

其中 `home` 对应原项目 `sensorHomeValue`，`scale` 对应原项目 `sensorRaw2DataCof`。

## 接入新项目的步骤

1. 把 `runtime_trace_slave_reader.h/.cpp` 复制到新项目。
2. 确认新项目能包含雷赛 `LTDMC.h`，并链接 `LTDMC` 库。
3. 根据 EtherCAT 从站地址和对象字典填写 `ObjectConfig`。
4. 把采集调用放在硬件访问线程中，避免多个线程同时操作同一控制卡的 Trace API。
5. 在周期任务中调用 `readTraceCached()`。
6. 用 `takeSamples()` 取出新帧，接入滤波、记录、显示或控制环。
7. 停止采集或重新配置前调用 `reset()`。

## 错误和重试

模块保留原项目的容错策略：

- `dmc_trace_get_state()` 连续失败达到 5 次后，会重置 Trace 状态并标记为不可用。
- 不可用后不会每个周期都立即重配，而是等待 `retryIntervalUs` 后再尝试。
- 如果曾经成功读取过数据，短时无新帧时 `readTraceCached()` 返回 `0`，上层可以继续使用缓存值。
- 如果从未成功读取且配置或读取失败，`readTraceCached()` 返回 `-1`。

## 使用限制

- 同一张控制卡通常只有一套 Trace 缓冲。不要让多个模块同时调用 `dmc_trace_set_config()` 或 `dmc_trace_get_data()`。
- 本模块不是线程安全类。如果跨线程使用，需要由上层加锁或限定在单一硬件线程中调用。
- 模块不负责控制卡连接和断开。调用前应确保 `dmc_board_init()` 已成功。
- 模块不负责 PDO 映射配置。需要先保证从站对象已经能被控制卡 Trace 访问。
- 如果新项目读取的数据不是小端有符号整数，需要调整 `readSignedLittleEndianTraceValue()` 或新增解码策略。

## 与原项目上层控制的边界

原项目 `ControlWorker` 在 500 us 节奏下取 `readForceSensorDataTraceSamples()` 的样本队列，并补充时间戳、滤波、历史记录和控制逻辑。迁移时建议保持同样分层：

- `RuntimeTraceSlaveReader`：只负责读取和解码。
- 硬件接口层：负责连接控制卡、线程串行化、对象配置。
- 控制层：负责采样时间戳、滤波、诊断、控制律和安全处理。
