# PS2 遥控器实机测试 Demo

本 demo 用于验证 PS2 UART 接收器、协议解析以及统一 remoter 服务。测试结果通过调试器中的 `demo_debug_instance.ps2_unit` 展开观察，不依赖串口打印。

## 启用配置

在 `configs/params.json` 中选择 PS2，并绑定实际接收器串口：

```json
{
  "bindings": {
    "remoter_uart": "usart10"
  },
  "remoter": {
    "source": "ps2",
    "thread_priority": 2,
    "rx_timeout_ticks": 100,
    "ps2_offline_timeout_ticks": 600,
    "ps2_frame_timeout_ticks": 20,
    "ps2_deadzone": 0.08
  }
}
```

对应 UART 必须在 `board.ioc` 中启用 RX DMA，并配置好 RX/TX 引脚。驱动启动时会将串口切换为 9600 8N1。

重新运行 CMake configure 和 build 后，`demo/app.cpp` 会根据 `config::feature::enable_ps2` 自动选择本 demo。

## 调试字段

连接调试器后观察：

- `link`：`0` 已连接，`1` 接收器在线但手柄未连接，`2` 接收器离线。
- `buttons`：当前 16 位按键位图。
- `pressed` / `released`：最新一帧的按下沿与释放沿，只保持一个约 40 ms 的协议更新周期，低频 Live Watch 可能看不到。
- `last_pressed` / `last_released`：最近一次非零边沿，持续保留，适合 Live Watch。
- `pressed_seen_mask` / `released_seen_mask`：本次启动以来观察到过的按键边沿累计位图。
- `press_event_count` / `release_event_count`：按下沿和释放沿事件帧计数。
- `raw_left_x`～`raw_right_y`：未经归一化的四轴原始值。
- `left_x`～`right_y`：统一 remoter 输出，已归一化并应用死区，Y 轴向上为正。
- `last_frame_period_ticks`：相邻正常帧周期，标称约 40 ms 对应的 tick 数。
- `min_frame_period_ticks` / `max_frame_period_ticks`：观测到的帧周期范围。
- `connected_frame_count`：收到的正常帧数量。
- `remote_disconnected_count` / `receiver_offline_count`：链路状态切换次数。
- `passed`：已收到正常帧、统一 remoter 来源为 PS2 且当前在线。

## 建议测试步骤

1. 接收器上电但手柄未连接，确认 `link == 1`。
2. 打开手柄，确认 `link == 0`、`connected_frame_count` 持续增加。
3. 逐个按键，核对 `buttons`、`pressed`、`released`。
4. 分别把四个摇杆推到极限，核对原始范围和归一化方向。
5. 关闭手柄，确认返回 `link == 1`。
6. 断开接收器供电或 UART，超过超时后确认 `link == 2`。
