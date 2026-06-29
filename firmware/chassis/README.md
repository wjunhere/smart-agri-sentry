# firmware/chassis

STM32F407ZGT6 履带式底盘控制器。

## 硬件

- **MCU**: STM32F407ZGT6（最小系统板，当前运行在 HSI 16 MHz）
- **电机**: MG540 直流减速电机 ×2
- **编码器**: 13 PPR，减速比 1:30，四倍频 → 每米 11035 脉冲
- **底盘**: 履带式，轮距 0.23 m

## 接线

| 功能 | STM32 引脚 | 连接目标 |
|---|---|---|
| 调试串口 TX/RX | USART1 PA9/PA10 | ST-Link VCP，9600 baud |
| RDK X5 协议 | USART2 PA2/PA3 | RDK X5 UART2，115200 baud |
| 左电机 PWM | TIM1 CH1 PE9 | 左电机驱动 PWM |
| 右电机 PWM | TIM1 CH2 PE11 | 右电机驱动 PWM |
| 左电机方向 | PE13 / PE14 | 左电机驱动方向 |
| 右电机方向 | PG6 / PG7 | 右电机驱动方向 |
| 左编码器 | TIM2 PA5 / PB3 | 左编码器 A/B 相 |
| 右编码器 | TIM3 PA6 / PC7 | 右编码器 A/B 相 |
| 电池电压 | ADC1 CH7 PA7 | 电池分压 |
| 心跳 LED | PA8 | 状态指示灯 |

## 主循环周期

| 任务 | 周期 | 说明 |
|---|---|---|
| Protocol_Process | 主循环 | 解析 RDK X5 运动控制帧 |
| PID 闭环 | 10 ms | 编码器 → PID → PWM |
| 底盘状态帧 TX | 100 ms | DMA 发送 25-byte 状态帧 |
| 心跳 LED | 500 ms | PA8 翻转 |
| 状态打印 | 5000 ms | USART1 摘要 |
| 看门狗 | 100 ms | IWDG 刷新 |

## 编译与烧录

1. 使用 Keil µVision 打开 `MDK-ARM/Intelligent_Agricultural_Robot_Car.uvprojx` 编译。
2. 或使用 OpenOCD / ST-Link 烧录 `MDK-ARM/.../Intelligent_Agricultural_Robot_Car.hex`。
3. 上电后 USART1 9600 输出 boot marker `012345...`。

## 协议速查

### RDK X5 → STM32（运动控制帧，TYPE=0x81）

```
AA 55 81 04 <left_mm/s:int16le> <right_mm/s:int16le> <CRC16:big-endian>
```

- `left_mm/s` / `right_mm/s`：目标左右轮速度，单位 mm/s。
- STM32 内部按 11035 pulses/m 转换为 pulses/10ms；左右电机驱动通道在硬件上交叉。

### STM32 → RDK X5（底盘状态帧，TYPE=0x03）

```
AA 55 03 13 <left_speed:int16le> <right_speed:int16le> <battery:int16le> <alarm:uint8> <left_pulse:int32le> <right_pulse:int32le> <timestamp:uint32le> <CRC16:big-endian>
```

- `left_speed` / `right_speed`：mm/s
- `battery`：电压 ×100，单位 0.01 V
- `alarm`：报警位，bit 2 (0x04) = 通信错误
- `left_pulse` / `right_pulse`：有符号累计脉冲
- `timestamp`：STM32 开机毫秒数

### CRC16

- 算法：CRC16-CCITT（多项式 0x1021，初始值 0xFFFF）
- 范围：TYPE + LEN + DATA
- 传输：big-endian（高字节在前）

## 调试命令（USART1）

```text
LR 100 100    # 左右轮目标 100 pulses/10ms
STOP          # 停止电机
HELP          # 显示帮助
```
