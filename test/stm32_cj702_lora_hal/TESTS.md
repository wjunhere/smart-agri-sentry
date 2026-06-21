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

## 待完成硬件集成测试

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
