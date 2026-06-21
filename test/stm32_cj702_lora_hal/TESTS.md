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
