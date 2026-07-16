# USB CDC demo

## 作用

本 demo 用于验证 USBX CDC ACM 的 USB 虚拟串口收发链路。

- 主机通过 USB CDC COM 口访问设备。
- 主机发送固定 24 字节 `host_packet`。
- 板端校验 magic 与 checksum，通过后返回 32 字节 `device_packet`。
- 固件侧不使用 print，状态通过 `demo_debug_instance.usb` 展开观察。

## 调试流程

1. 枚举检查：
   - Windows 能识别 USB CDC COM 口。
   - `started = true`
   - `ready = true`
   - `connected = true`
2. 坏校验包检查：
   - 主机读不到正常响应。
   - `error_count` 增加。
3. 正常包检查：
   - `rx_count` 增加。
   - `tx_count` 增加。
   - `last_seq/last_counter/last_value/last_flag` 与主机包一致。
4. BSP 细节检查：
   - `bsp_last_read_len = 24` 表示 OUT 收到完整 host 包。
   - `bsp_last_write_requested = 32` 表示准备发送完整 device 包。
   - `bsp_last_write_actual = 32` 表示 USBX 实际写出完整响应。
   - `bsp_write_count` 与 demo `tx_count` 应同步增长。

曾定位过的关键问题：

- USBX 需要启动 DCD 并 `HAL_PCD_Start()`，否则 debug 可 ready 但 PC 不枚举。
- RX/TX 线程之间不能依赖裸 pending 变量，当前使用 mutex 保护单槽响应。
- USB OTG HS 需要配置 Rx/Tx FIFO，否则 CDC IN 写可能阻塞；当前对齐 `gimbal_2026` 的 FIFO 设置。

## 测试项

- 坏 checksum 包被拒绝。
- 连续 10 个正常包均收到响应。
- 响应 magic 为 `RSB1`。
- 响应 seq、value、flag 与请求一致。
- `rx_count` 与 `tx_count` 同步递增。
- 多次重复运行时计数连续增长，`error_count` 只随坏校验包增长。

## 运行

先使用 Ninja + `arm-none-eabi` 构建并烧录：

```powershell
cmake --build .agents\build\embedded_framework_elf_arm
```

烧录：

```text
.agents/build/embedded_framework_elf_arm/embedded_framework.elf
```

查看 USB CDC 端口：

```powershell
cd embedded_framework\demo\tools
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
```

主机侧运行：

```powershell
.\.venv\Scripts\python.exe .\run_usb_demo.py --port COM5 --bad-checksum
```

端口号按实际 USB CDC COM 口修改。
