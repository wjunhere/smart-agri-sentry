# USART2 通信测试报告

**日期**: 2026-06-27  
**芯片**: STM32F407ZGTx (Cortex-M4)  
**时钟**: HSI 16 MHz  
**调试器**: ST-Link V2 (OpenOCD 0.12.0)

---

## 1. 硬件接线

| 信号 | STM32 引脚 | 外部设备 |
|------|-----------|---------|
| USART2 TX | PA2 | USB-TTL RX |
| USART2 RX | PA3 | USB-TTL TX |
| GND | GND | USB-TTL GND |

- ST-Link VCP → USART1 (PA9/PA10)，9600 baud，调试控制台
- USB-TTL → USART2 (PA2/PA3)，115200 baud，RDK X5 协议
- **COM9 = USART2**

---

## 2. 编码器参数

| 参数 | 值 |
|------|-----|
| 电机型号 | MG540 |
| 编码器 PPR | 13 |
| 减速比 | 1:30 |
| 轮径 | 4.5 cm |
| 轮周长 | 0.1414 m |
| 每轮圈脉冲 | 1560 (= 13 × 4 × 30) |
| 每米脉冲 | **11035** |

---

## 3. 协议帧格式

### 速度控制帧 (RDK X5 → STM32)
```
AA 01 04 <left_L> <left_H> <right_L> <right_H> <CRC>
```
- 长度: 9 字节
- left/right: `int16_t` little-endian（脉冲/10ms，滤波后的目标值）
- 硬件通道交叉：固件内部左右交换

### 遥测帧 (STM32 → RDK X5)
```
AA 02 08 <encR[4]> <encL[4]> <CRC>
```
- 长度: 12 字节
- encL/encR: `int32_t` little-endian（累积脉冲，永不清零）
- 同样左右交换以匹配硬件交叉
- 频率: 10 Hz
- CRC: 8 位累加和（byte 0-10 的和 mod 256）

### 示例

停止状态遥测帧：
```
AA 02 08 00 00 00 00 00 00 00 00 B4
```
CRC 验证: `AA+02+08+00+00+00+00+00+00+00+00 = B4` ✓

左轮速度=10 控制帧：
```
AA 01 04 0A 00 00 00 B9
```
CRC 验证: `AA+01+04+0A+00+00+00 = B9` ✓

---

## 4. 编译 & 烧录

```
编译器: ARMCC V5.06 update 5 (build 528)
编译结果: 0 Error(s), 0 Warning(s)
固件大小: Code=26636 RO-data=1056 RW-data=100 ZI-data=2908
烧录工具: OpenOCD + ST-Link V2
烧录结果: 32768 bytes written in 1.19s (26.84 KiB/s) ✓
```

---

## 5. 测试结果

| 测试项 | 方法 | 结果 |
|--------|------|------|
| 固件编译 | Keil UV4 命令行 | ✅ 0 error |
| 固件烧录 | OpenOCD STLink | ✅ 32KB 写入 |
| 遥测接收 | COM9 115200 读取 | ✅ 12 字节帧，10Hz 稳定 |
| 帧格式 | 解析 header/cmd/len/crc | ✅ 全部正确 |
| CRC 校验 | 8-bit additive checksum | ✅ 全部通过 |
| 速度控制 | 发送 `AA 01 04 0A 00 00 00 B9` | ✅ 电机转动 |
| 累积脉冲 | 电机转动后遥测值增长 | ✅ 观测到累积值增加 |
| STOP 指令 | 发送 `AA 01 04 00 00 00 00 AF` | ✅ 电机停止 |

---

## 6. 改动文件清单

| 文件 | 改动说明 |
|------|---------|
| `Users/bsp_encoder.h` | 新增 `ENCODER_PULSES_PER_METER` 常量、`Encoder_Get_Left/Right_Accum()`、`Encoder_Get_Left/Right_Distance_M()` |
| `Users/bsp_encoder.c` | 新增 `accum_left/right` 变量（永不清零），在 `Get_Speed` 中累加原始脉冲 |
| `Users/bsp_protocol.h` | `Protocol_Send_Telemetry` 参数 `int16_t` → `int32_t` |
| `Users/bsp_protocol.c` | 遥测帧扩容：4 字节 payload → 8 字节（2× int32 LE），帧长 8→12 |
| `Core/Src/main.c` | 遥测调用改为发累积值；5 秒状态行增加 Accum + 距离显示 |
| `Users/bsp_debug.c` | `STATUS` 命令新增 Accum 原始脉冲 + 米数 |

---

## 7. 数据流示意

```
Encoder_Get_Left_Speed()           Encoder_Get_Right_Speed()
  ├─ 读 TIM2 → 清零                  ├─ 读 TIM3 → 清零
  ├─ accum_left += raw (不清零)     ├─ accum_right += raw (不清零)
  ├─ LPF 滤波                       ├─ LPF 滤波
  └─ return → PID                   └─ return → PID
         ↓                                  ↓
    PID_Calc()                         PID_Calc()
         ↓                                  ↓
    Motor_Set_PWM()                          
                                             
Encoder_Get_Left_Accum()            Encoder_Get_Right_Accum()
  └─ return accum_left               └─ return accum_right
         ↓                                  ↓
    Protocol_Send_Telemetry(int32, int32)
         ↓
    USART2 TX @10Hz (12 字节帧)
         ↓
    RDK X5 收到 int32 累积脉冲
```

---

## 8. 调试命令 (USART1 控制台)

| 命令 | 说明 |
|------|------|
| `STATUS` | 显示速度/累积脉冲/距离/电池等信息 |
| `L10` | 左轮目标=10 |
| `R10` | 右轮目标=10 |
| `LR 10 20` | 左=10 右=20 |
| `STOP` | 停止并复位 PID |
| `HELP` | 帮助 |

## 9. Python 测试脚本

```python
import serial

# USART2 = COM9, 115200 baud
ser = serial.Serial('COM9', 115200, timeout=0.5)

# 发送速度控制: 左轮=10, 右轮=0
# Frame: AA 01 04 <L_L><L_H><R_L><R_H> <CRC>
left, right = 10, 0
frame = bytes([
    0xAA, 0x01, 0x04,
    left & 0xFF, (left >> 8) & 0xFF,
    right & 0xFF, (right >> 8) & 0xFF,
    0x00  # placeholder
])
crc = sum(frame[:7]) & 0xFF
frame = frame[:7] + bytes([crc])
ser.write(frame)

# 读取遥测
data = ser.read(60)
for i in range(len(data) - 11):
    if data[i] == 0xAA and data[i+1] == 0x02 and data[i+2] == 0x08:
        r_acc = int.from_bytes(data[i+3:i+7], 'little', signed=True)
        l_acc = int.from_bytes(data[i+7:i+11], 'little', signed=True)
        print(f"L_acc={l_acc}, R_acc={r_acc}, CRC={'OK' if sum(data[i:i+11]) & 0xFF == data[i+11] else 'BAD'}")

ser.close()
```
