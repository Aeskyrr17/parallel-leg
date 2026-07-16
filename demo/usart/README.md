# USART demo

## 作用

本 demo 用于验证 `bsp::usart` 的 USART1 DMA 收发链路。

- 板端使用 `app::uart::usart1`。
- 主机发送固定 24 字节 `host_packet`。
- 板端校验 magic 与 checksum，通过后返回 32 字节 `device_packet`。
- 固件侧不使用 print，状态通过 `demo_debug_instance.usart` 展开观察。

## 调试流程

1. 启动后确认：
   - `started = true`
   - `ready = true`
   - `last_status = 0`
2. 未发送数据前，`rx_count/tx_count/error_count` 不应异常增长。
3. 发送坏校验包后：
   - 不返回正常响应。
   - `error_count` 增加。
4. 发送正常包后：
   - `rx_count` 增加。
   - `tx_count` 增加。
   - `last_seq/last_counter/last_value/last_flag` 与主机包一致。

曾定位过的关键问题：

- USART 状态对象不能放在 NOLOAD 未初始化区，否则 `initialized` 可能上电即为脏值。
- Circular DMA 下不能同时把 TC 和 IDLE 都当作完整包处理，否则同一包会被重复计数。

## 测试项

- 坏 checksum 包被拒绝。
- 连续 10 个正常包均收到响应。
- 响应 magic 为 `RUS1`。
- 响应 seq、value、flag 与请求一致。
- `rx_count` 与 `tx_count` 同步递增。

## 运行

先使用 Ninja + `arm-none-eabi` 构建并烧录：

```powershell
cmake --build .agents\build\embedded_framework_elf_arm
```

烧录：

```text
.agents/build/embedded_framework_elf_arm/embedded_framework.elf
```

主机侧运行：

```powershell
cd embedded_framework\demo\tools
.\.venv\Scripts\python.exe .\run_usart_demo.py --port COM35 --baud 115200 --bad-checksum
```

端口号按实际 USART 转串口设备修改。
