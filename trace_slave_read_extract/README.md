# Trace 读取从站变量摘录

本目录把本项目中通过雷赛 `dmc_trace_*` 读取从站变量的核心逻辑独立出来，便于在结构相近的项目中复用。主代码是 `RuntimeTraceSlaveReader`，不依赖 Qt，只依赖雷赛 `LTDMC.h` 和 `LTDMC` 库。

## 文件

- `runtime_trace_slave_reader.h`
- `runtime_trace_slave_reader.cpp`
- `trace_module_description.md`
- `source_map.md`

## 原项目逻辑链路

1. 组织 Trace 对象列表：每个对象包含 `dataType`、`dataIndex`、`dataSubIndex`、`slaveId`、API 数据字节数、实际值字节数和输出通道号。
2. 调用 `dmc_trace_data_stop()`、`dmc_trace_data_reset()` 清理旧 Trace 状态。
3. 按采样周期换算 `traceCycle = samplePeriodUs / traceBaseCycleUs`，调用 `dmc_trace_set_config()`。
4. 调用 `dmc_trace_reset_config_object()` 后，对每个变量调用 `dmc_trace_add_config_object()`。
5. 调用 `dmc_trace_data_start()` 启动 Trace。
6. 周期调用 `dmc_trace_get_state()` 查询 `validNum`、`objectTotalBytes`、`objectTotalNum`。
7. 调用 `dmc_trace_get_data()` 批量取帧，按对象顺序解析每帧。
8. 每个变量按小端有符号数解码，再执行 `(raw - home) * scale`，写入缓存并形成样本队列。
9. 上层控制循环调用 `takeSamples()` 一次性取走队列中的所有样本。

## 本项目张力传感器配置示例

本项目默认把 8 路张力传感器分在两个从站：

```cpp
RuntimeTraceSlaveReader::ReaderConfig cfg;
cfg.samplePeriodUs = 500;
cfg.traceBaseCycleUs = 500;

auto addForceSensor = [&](int channel, short slaveId, int subIndex,
                          double home, double scale) {
    RuntimeTraceSlaveReader::ObjectConfig object;
    object.logicalIndex = channel;
    object.dataType = 19;
    object.dataIndex = 0x6000;
    object.dataSubIndex = subIndex;
    object.slaveId = slaveId;
    object.apiDataBytes = 0;
    object.valueBytes = 2;
    object.home = home;
    object.scale = scale;
    object.signMode = RuntimeTraceSlaveReader::SignMode::SignedValue;
    cfg.objects.push_back(object);
};

addForceSensor(0, 1004, 1, sensorHome[0], sensorScale[0]);
addForceSensor(1, 1004, 2, sensorHome[1], sensorScale[1]);
addForceSensor(2, 1004, 3, sensorHome[2], sensorScale[2]);
addForceSensor(3, 1004, 4, sensorHome[3], sensorScale[3]);
addForceSensor(4, 1011, 1, sensorHome[4], sensorScale[4]);
addForceSensor(5, 1011, 2, sensorHome[5], sensorScale[5]);
addForceSensor(6, 1011, 3, sensorHome[6], sensorScale[6]);
addForceSensor(7, 1011, 4, sensorHome[7], sensorScale[7]);

RuntimeTraceSlaveReader reader(cfg);
reader.configure();
```

读取时：

```cpp
const int frames = reader.readTraceCached();
if(frames > 0 || reader.hasEverRead()){
    const std::vector<RuntimeTraceSlaveReader::Sample> samples = reader.takeSamples();
    for(const RuntimeTraceSlaveReader::Sample& sample : samples){
        const std::vector<double>& forceValues = sample.values;
        // 在这里接入滤波、显示、记录或控制环
    }
}
```

## 迁移要点

- `dataType = 19, dataIndex = 0x6000, slaveId = 从站地址, dataSubIndex = 通道号` 是本项目 SBT908E 张力传感器的配置方式。
- 如果新项目读取电机命令位置/实际位置，可参考原项目 `dataType = 5/6` 的写法，`dataIndex` 使用雷赛硬件轴号，`slaveId = 0`，`valueBytes = apiDataBytes = 4`。
- `RuntimeTraceSlaveReader` 默认按对象顺序输出；设置 `logicalIndex` 后可把不同从站和通道映射到固定输出下标。
- 原项目解析帧时会兼容 `objectTotalBytes` 大于实际对象字节数的情况，自动估计帧头和每对象槽位宽度；这个逻辑已保留。
- 上层调用应放在同一个硬件访问线程或自行加锁，避免和其他 `dmc_trace_*` 调用并发操作同一张控制卡。
