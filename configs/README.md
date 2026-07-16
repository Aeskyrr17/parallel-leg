# 配置参数传入说明

`embedded_framework` 的配置入口位于本目录，CMake 配置阶段会读取 `params.json`、`robot.json` 和 `board/board.ioc`，生成 `generated/config.hpp` 与 `generated/robot_config.hpp`。修改 JSON 后重新运行 CMake configure 或 build，即可刷新生成头文件。

生成脚本由顶层 `CMakeLists.txt` 传入以下 CMake 变量：`IOC` 指向 `board/board.ioc`，`PARAMS` 指向 `configs/params.json`，`ROBOT_CONFIG` 指向 `configs/robot.json`，`OUT_DIR` 指向 `configs/generated`。通常不需要手动传这些变量。

## params.json

`params.json` 描述构建开关、外设绑定和任务参数。

| 路径 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `build.usbx` | bool | `false` | 是否编译 USBX/USB CDC 相关 BSP 与功能。 |
| `build.motors.dji` | bool | `true` | 是否启用 DJI 电机相关构建开关。 |
| `build.motors.dm` | bool | `true` | 是否启用达妙电机相关构建开关。 |
| `build.motors.lk` | bool | `false` | 是否启用 LK 电机相关构建开关。 |
| `bindings.remoter_uart` | string | `uart5` | 遥控器串口，需要与 `board.ioc` 中存在且有 RX DMA 的 UART 名称一致。 |
| `bindings.referee_uart` | string | `usart1` | 裁判系统串口，需要与 `board.ioc` 中存在的 UART 名称一致。 |
| `can.<fdcan>.id_type` | string | IOC 推导 | 指定某路 CAN 接收过滤 ID 类型，可选 `standard` 或 `extended`。例如 `can.fdcan1.id_type`。手动配置优先于 IOC 推导。 |
| `ahrs.imu_offset_x` | number | `0.0` | IMU X 轴安装偏置。 |
| `ahrs.imu_thread_priority` | number | `3` | AHRS/IMU 线程优先级。 |
| `ahrs.temp_thread_priority` | number | `4` | IMU 温控线程优先级。 |
| `ahrs.target_temp` | number | `45.0` | IMU 目标温度。 |
| `remoter.source` | string | 空 | 遥控器来源，例如 `dr16`。 |
| `remoter.thread_priority` | number | `2` | 遥控器线程优先级。 |
| `remoter.rx_timeout_ticks` | number | `100` | 遥控器接收超时 tick 数。 |
| `referee.thread_priority` | number | `8` | 裁判系统线程优先级。 |
| `test.thread_priority` | number | `10` | demo/test 线程优先级。 |
| `test.report_uart` | string | `uart7` | 测试报告串口，不能与当前遥控器 UART 冲突。 |
| `test.auto_run_on_boot` | bool | `true` | 是否上电自动运行测试/demo。 |
| `usb.read_thread_priority` | number | `5` | USB CDC 读线程优先级。 |
| `usb.write_thread_priority` | number | `5` | USB CDC 写线程优先级。 |
| `usb.period_ticks` | number | `2` | USB CDC 周期 tick 数。 |

注意：`ahrs`、`remoter`、`referee`、`test`、`usb` 这些分组在生成脚本中按“整组缺省”补默认值。如果某个分组里只填写一部分字段，未填写的字段不会生成，使用时应保持同组字段完整。

CAN 的 `id_type` 只表示标准帧 ID 或扩展帧 ID，不表示 CAN Classic 或 CAN FD。CAN Classic/FD 仍由 `board.ioc` 的 `FDCANx.FrameFormat` 推导；`can.<fdcan>.id_type` 只控制生成到 `config::can::filter_id_types` 的标准/扩展过滤类型。

## robot.json

`robot.json` 描述机器人设备树，目前主要包含电机配置。

### `devices.motors.dm`

| 路径 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `id_base` | string/number | `0x01` | 达妙电机反馈表的电机 ID 起始值。 |
| `master_id_base` | string/number | `0x05` | 达妙反馈帧 master ID 起始值。 |
| `max_motors` | number | `4` | 单个 CAN 总线登记的达妙电机最大数量。 |

### `devices.motors.list[]`

每个电机对象支持以下字段：

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `name` | string | 是 | 生成到 `robot::motors` 命名空间中的 C++ 标识符。 |
| `model` | string | 是 | 电机型号。当前 motor demo 支持 `dji_m2006`、`dji_m3508`、`dji_gm6020`、`dji_xroll`、`dm_dm4310`、`dm_dm8009p`。 |
| `can_bus` | string | 是 | CAN 外设名称，例如 `fdcan1`、`fdcan2`，必须存在于 `board.ioc`。 |
| `can_type` | string | 是 | CAN 帧类型：`classic` 或 `fd`。 |
| `can_id` | string/number | 是 | 电机基础 CAN ID，建议十六进制字符串，如 `0x01`。 |
| `control_mode` | string | 否 | 电机初始控制模式，默认 `relax`；可选 `relax`、`torque`、`mit`、`pos_speed`、`speed`、`multi`。`velocity` 可作为 `speed` 的别名，`position_speed` 可作为 `pos_speed` 的别名。 |

示例：

```json
{
  "name": "motor2",
  "model": "dm_dm4310",
  "can_bus": "fdcan1",
  "can_type": "fd",
  "can_id": "0x01",
  "control_mode": "mit"
}
```

`control_mode` 会写入生成的 `motors::config`，电机对象构造时即设置到 `control_mode`。对于达妙电机，enable/disable/save-zero/clear-error 的控制帧 ID 会根据该模式选择基础 ID、`+0x100`、`+0x200` 或 `0x300`，因此需要在 enable 前通过配置确定。

`model` 会生成 `<name>_model` 常量。例如 `name` 为 `motor2` 时会生成 `robot::motors::motor2_model`。demo 可以用这个常量在编译期选择具体 C++ 电机类。
