# STM32F103RCT6 + CJ-702 + LoRa 集成测试结果

## 单元测试

### 主机单元测试

运行环境：WSL (Ubuntu on Windows)，gcc 13.x
命令：
```bash
cd test/stm32_cj702_lora_hal/Tests/host
make
```

结果：
```
cj702: 11/11 passed
aggregator: 5/5 passed
lora_frame: 10/10 passed
```

覆盖内容：
- CJ-702 正温度、负温度、错误校验和帧解析
- Aggregator 30 样本平均与最小样本数不足分支
- LoRa 数据帧/错误帧打包长度、帧头、消息类型、payload 长度字段

## 固件构建

命令：
```bash
cd test/stm32_cj702_lora_hal
make clean && make
```

结果：成功生成
- `build/stm32_cj702_lora_hal.elf`
- `build/stm32_cj702_lora_hal.hex`
- `build/stm32_cj702_lora_hal.bin`

Size：
```
text    data    bss     dec     hex
8980    12      1892    10884   2a84
```

## 烧录验证

命令：
```bash
STM32_Programmer_CLI.exe -c port=SWD -w build/stm32_cj702_lora_hal.hex -v -g
```

结果：烧录并校验成功。

探测信息：
- ST-LINK SN: 37FF71064E573436B0D01143
- ST-LINK FW: V2J37S7
- Device ID: 0x414
- Device name: STM32F101/F103 High-density
- NVM size: 256 KBytes
- Device CPU: Cortex-M3

## 硬件集成测试（2026-06-21）

已按建议接线表完成接线，并通过 ST-Link 在线观察运行状态。

### 观测方法

使用 probe-rs / embedded-debugger 连接 STM32F103RCTx，读取关键变量与寄存器：
- `g_app_fsm` @ `0x200000b8`
- `g_aggregator` @ `0x200000cc`
- `g_tx_buf` @ `0x20000104`
- `GPIOC_ODR` @ `0x4001100c`

### 观测结果（第一次，GND 未共地）

- 程序正常运行，PC 在 Flash 区域（`0x080009C2` 附近）。
- `GPIOC_ODR = 0x05`，即 PC0=1、PC1=0、PC2=1，**绿灯亮**，与 `LED_STATE_NORMAL` 一致。
- `g_app_fsm.seconds` 持续递增并回零，状态机循环正常。
- `g_aggregator.count = 0`，`g_new_sample_ready = 0`，**USART2 RX 未收到任何 CJ-702 数据**。
- `USART2_SR = 0xC0`，RXNE=0，确认 USART2 硬件层无数据。
- `g_tx_buf` 中已打包错误帧：`AA 55 01 FF 01 01 C7`（超时错误码 0x01，CRC8=0xC7），说明 LoRa 帧打包与发送路径已被触发。

### 观测结果（第二次，GND 共地后）

- 共地后初见好转：某个 60 秒周期内 LoRa 错误帧从 `0x01`（超时）变成了 `0x02`（数据不完整），说明期间收到了少量有效 CJ-702 帧，但数量不足 10 个，未能形成平均值。
- 继续观测一个完整 60 秒周期后，`g_tx_buf` 又变回 `AA 55 01 FF 01 01 C7`（超时错误），`g_aggregator.count = 0`，`s_cj702_idx = 0`。
- 结论：GND 共地是必要条件，但 **TXD → PA3 的连接仍不稳定或传感器输出断续**。

### 结论与下一步

- 固件状态机、LoRa 帧打包、TX 发送、状态灯均正常。
- **CJ-702 传感器数据未稳定到达 STM32 PA3**，需继续检查：
  1. CJ-702 是否已上电（5V/GND 电压是否稳定）。
  2. CJ-702 的 TXD 是否可靠接到 STM32 的 PA3（USART2_RX），杜邦线/排针是否接触不良。
  3. 传感器是否已开始输出（部分模块上电后需要数十秒预热）。
  4. 尝试用 USB 转串口直接接 CJ-702 TXD，确认传感器每 2 秒输出 17 字节 9600bps 数据。

排查传感器侧后，重新上电并等待 60 秒，应能在 USART3/PB10 侧看到 20 字节数据帧：`AA 55 01 01 0E [14 bytes payload] [CRC8]`。

## 代码问题定位与修复（2026-06-25）

### 根因

`Core/Src/usart.c` 中 `MX_USART2_UART_Init()` 与 `MX_USART3_UART_Init()` 没有设置 `Init.Mode` 字段。由于 `huart2`/`huart3` 是全局变量，零初始化后 `Mode = 0`，导致 `HAL_UART_Init()` 在配置 `USART_CR1` 时未置位 `TE` 和 `RE`。

- USART2 的 `RE=0`：RXNE 中断永不产生，STM32 收不到 CJ-702 数据。
- USART3 的 `TE=0`：PB10 上可能没有实际的 LoRa 数据输出（之前只观察到 `g_tx_buf` 被打包）。

CubeMX 配置文件 `stm32_cj702_lora_hal.ioc` 中本来配置的是 `USART2.Mode=UART_MODE_TX_RX` 等字段，但生成的 `usart.c` 里这些赋值缺失。

### 修复内容

在 `MX_USART2_UART_Init()` 和 `MX_USART3_UART_Init()` 中补全：

```c
huartx.Init.WordLength = UART_WORDLENGTH_8B;
huartx.Init.StopBits   = UART_STOPBITS_1;
huartx.Init.Parity     = UART_PARITY_NONE;
huartx.Init.Mode       = UART_MODE_TX_RX;
```

### 验证

- 本地 `make clean && make` 通过，生成 `build/stm32_cj702_lora_hal.hex`。
- 固件尺寸：`text=9000 data=12 bss=1892`。
- 主机单元测试需在 WSL 下运行；当前 Windows bash 环境无法直接执行 `./host_tests`。

## 烧录与在线验证（2026-06-25）

使用 ST-Link V2 烧录修复后的固件，通过 probe-rs 读取关键变量：

- `g_new_sample_ready = 1`：RX 中断已触发并成功解析 CJ-702 帧。
- `g_aggregator.count = 17`：约 34 秒内累计收到 17 个有效样本。
- `s_cj702_raw` 前两个字节为 `3C 02`，与 CJ-702 帧头一致。
- `g_tx_buf` 内容：
  ```
  AA 55 01 01 0E 02 07 00 0B 00 4A 00 0A 00 0C 08 FE 19 01 64
  ```
  帧类型为数据帧（`01`），payload 长度 `0E`。解包后的平均值为：
  - CO2：519 ppm
  - HCHO：11
  - TVOC：74 ppb
  - PM2.5：10 μg/m³
  - PM10：12 μg/m³
  - 温度：23.02 °C
  - 湿度：64.01 %RH

结论：**修复后 CJ-702 数据已能稳定到达 STM32，LoRa 数据帧也已正常打包。** 后续只需在 USART3/PB10 侧接 LoRa 模块验证 RF 发送即可。

## LoRa 模块发送验证（2026-06-25）

连接 E22-400T30S（M0=0、M1=0 透传模式）后，再次在线观测：

- `g_app_fsm.seconds` 每 60 秒回零，状态机循环正常。
- 每次 60 秒周期结束时 `g_tx_buf` 都会被更新为一帧新的数据帧，例如：
  ```
  AA 55 01 01 0E 02 38 00 11 00 69 00 09 00 0B 09 11 18 39 05
  ```
- `USART3_CR1 = 0x200C`：UE=1、TE=1、RE=1，发送器已使能。
- `USART3_SR = 0xC0`：TXE=1、TC=1，上一帧已完整移出 USART3 移位寄存器。
- `USART3_BRR = 0x0EA6`（3750），对应 9600 bps @ 36 MHz PCLK1，波特率正确。

固件层面已确认：**STM32 每 60 秒通过 PB10 向 LoRa 模块发送 20 字节数据帧，且 USART3 发送已完成。** 是否真正通过 LoRa 射频发出，需要用以下任一方式在另一端验证：

1. 另一块 LoRa 模块 + USB 转串口接收；
2. 在 PB10 引脚直接用 USB 转串口监听 9600 bps，应能看到 `AA 55 01 01 0E ...`；
3. 观察 LoRa 模块发送指示灯（如有）。

## 原始硬件集成测试清单

需要实际接线后验证：

1. CJ-702 → USART2 (PA3) 数据接收：
   - 每 2 秒收到 17 字节 CJ-702 帧
   - 校验通过后加入 aggregator

2. LoRa 模块 → USART3 (PB10) 数据发送：
   - 每 60 秒输出 20 字节传感器数据帧：`AA 55 01 01 0E [14 bytes payload] [CRC8]`
   - 断开传感器 60 秒后输出 7 字节错误帧：`AA 55 01 FF 01 01 [CRC8]` 或 `AA 55 01 FF 01 02 [CRC8]`

3. RGB LED 状态：
   - 正常：绿灯亮
   - 发送瞬间：蓝灯闪烁
   - 传感器异常：红灯亮

### 建议接线

| STM32 | CJ-702 | LoRa (E22-400T30S) |
|---|---|---|
| 5V    | 5V     | 5V                 |
| GND   | GND    | GND                |
| PA3   | TXD    | —                  |
| PB10  | —      | RX                 |
| PB11  | —      | TX                 |

> 注意：LoRa 模块默认透传模式需要 M0=0、M1=0（接地）。

## RDK X5 LoRa 接收集成测试（2026-06-28）

### 测试环境

- 接收端：E22-400TBH-SC → USB CDC → RDK X5 (/dev/ttyACM0, 9600 bps)
- 发送端：STM32F103RCT6 + CJ-702 + E22-400T30S（work mode=0 透传）

### 测试步骤

1. RDK 侧 `colcon build --packages-select sentry_interfaces sentry_sensors` 成功
2. 单元测试 12/12 通过
3. `ros2 launch sentry_sensors lora_bridge.launch.py` 启动节点
4. 订阅 `/sensor/environment_fixed` 等待 125 秒（覆盖 2 个发送周期）

### 测试结果

收到 2 条 `Environment` 消息，间隔约 60 秒：

```
#1: co2=527 hcho=12 tvoc=80 pm25=24 pm10=29 airT=25.1C airH=54.6%
#2: co2=520 hcho=12 tvoc=75 pm25=23 pm10=28 airT=25.1C airH=54.5%
```

- `data_source` = `FIXED_LORA`
- 土壤/叶面字段为 0.0（当前固件仅发送 CJ702 空气传感器数据）
- 节点串口断开后会自动重连（每 3 秒重试）

### 问题记录

| 问题 | 原因 | 解决 |
|---|---|---|
| E22-400TBH-SC 默认在 work mode 2（配置模式） | 模块之前通过按钮设置过 | 通过按键设为 mode 0（透传模式） |
| USB CDC 设备在模式切换时断开 | 模块重启导致 USB 枚举变化 | 节点加入 OSError 捕获与自动重连 |
