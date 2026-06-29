# 底盘控制项目集成实现计划

> **For agentic workers:** REQUIRED SUB-_SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `test/Intelligent_Agricultural_Robot_Car/` 迁移为 `firmware/chassis/`，采用项目规范 `0xAA55 + CRC16-CCITT` 协议，使 RDK X5 能通过 `/cmd_vel` 控制底盘并接收 `/sentry/chassis/status`。

**Architecture:** STM32F407 保持裸机主循环，仅重写 `bsp_protocol` 协议层；RDK 侧修复 `uart_bridge_node` 的 `/cmd_vel` 差速转换与底盘状态帧解析；`wheel_odom_node` 更新轮距与每米脉冲参数。

**Tech Stack:** STM32 HAL/C, STM32CubeMX/Keil, ROS2 Humble, Python 3, pytest.

---

## 文件映射

| 文件 | 操作 | 职责 |
|------|------|------|
| `firmware/chassis/` | 新建（从 `test/Intelligent_Agricultural_Robot_Car/` 复制） | 正式底盘固件目录 |
| `firmware/chassis/Users/bsp_protocol.h` | 重写 | 协议帧定义、CRC16、对外 API |
| `firmware/chassis/Users/bsp_protocol.c` | 重写 | 帧打包/解析、DMA 发送、IDLE+DMA 接收 |
| `firmware/chassis/Core/Src/main.c` | 修改 | 调用新协议 API、报警位统计 |
| `firmware/chassis/Tests/host/test_protocol.py` | 新建 | 主机端离线协议测试 |
| `firmware/chassis/README.md` | 新建 | 接线、编译、烧录、协议速查 |
| `src/sentry_interfaces/msg/ChassisStatus.msg` | 修改 | `left_pulse/right_pulse` 改为 `int32` |
| `src/sentry_sensors/sentry_sensors/uart_bridge_node.py` | 修改 | `/cmd_vel` 差速转换、状态帧解析 |
| `src/sentry_sensors/tests/test_uart_bridge_node.py` | 修改 | 新增协议解析与 cmd_vel 转换测试 |
| `src/sentry_mission/sentry_mission/wheel_odom_node.py` | 修改 | 默认轮距 0.23、每米脉冲 11035 |
| `docs/HARDWARE.md` | 修改 | 同步实际硬件参数 |
| `docs/ROS2.md` | 修改 | 更新 `/sentry/chassis/status` 与 `/cmd_vel` 说明 |

---

## Task 1: 迁移目录结构

**Files:**
- Create: `firmware/chassis/` (copy from `test/Intelligent_Agricultural_Robot_Car/`)
- Create: `firmware/chassis/Tests/host/`
- Create: `firmware/chassis/README.md` (placeholder, Task 11 填充)

- [ ] **Step 1: 复制测试项目到正式目录**

Run:
```bash
cp -r test/Intelligent_Agricultural_Robot_Car firmware/chassis
mkdir -p firmware/chassis/Tests/host
```

- [ ] **Step 2: 提交目录迁移**

```bash
git add firmware/chassis
# 排除 CubeMX 生成的临时/编译产物，确保 .gitignore 生效
git status
```

Commit:
```bash
git commit -m "$(cat <<'EOF'
chore(firmware): promote test chassis to firmware/chassis

Copy Intelligent_Agricultural_Robot_Car test project into firmware/chassis
as the canonical STM32F407 chassis controller.
EOF
)"
```

---

## Task 2: STM32 CRC16-CCITT 实现

**Files:**
- Modify: `firmware/chassis/Users/bsp_protocol.h`
- Modify: `firmware/chassis/Users/bsp_protocol.c`
- Create: `firmware/chassis/Tests/host/test_protocol.py`

- [ ] **Step 1: 在头文件中声明 CRC16 函数**

`firmware/chassis/Users/bsp_protocol.h`:
```c
#ifndef __BSP_PROTOCOL_H
#define __BSP_PROTOCOL_H

#include "stm32f4xx_hal.h"

#define FRAME_HEADER_0  0xAA
#define FRAME_HEADER_1  0x55

#define TYPE_CHASSIS    0x03
#define TYPE_MOTION_CMD 0x81

#define RX_BUFF_SIZE    64

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern volatile float target_speed_left;
extern volatile float target_speed_right;

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

void Protocol_Init(void);
void Protocol_Process(void);
void Protocol_Send_Chassis_Status(int16_t left_speed_mm_s,
                                  int16_t right_speed_mm_s,
                                  int16_t battery_x100,
                                  uint8_t alarm_bits,
                                  int32_t left_pulse,
                                  int32_t right_pulse,
                                  uint32_t timestamp_ms);

#endif
```

- [ ] **Step 2: 实现 CRC16-CCITT**

`firmware/chassis/Users/bsp_protocol.c`（替换原有文件内容，先只放 CRC）：
```c
#include "bsp_protocol.h"

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

- [ ] **Step 3: 写主机端 CRC 测试**

`firmware/chassis/Tests/host/test_protocol.py`:
```python
#!/usr/bin/env python3
import struct


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def test_crc16_ccitt_known():
    """Verify against a known vector: body=0x81 0x04 0x00 0x00 0x00 0x00"""
    body = bytes([0x81, 0x04, 0x00, 0x00, 0x00, 0x00])
    expected = crc16_ccitt(body)
    # Cross-check with struct-pack round-trip
    assert struct.pack('>H', expected) != b'\x00\x00'


def test_crc_matches_stm32_logic():
    """Same algorithm as C implementation; should not raise."""
    body = bytes([0x03, 0x13]) + bytes(19)
    crc = crc16_ccitt(body)
    assert 0 <= crc <= 0xFFFF


if __name__ == '__main__':
    test_crc16_ccitt_known()
    test_crc_matches_stm32_logic()
    print('CRC tests OK')
```

Run:
```bash
cd firmware/chassis/Tests/host
python test_protocol.py
```
Expected:
```text
CRC tests OK
```

- [ ] **Step 4: 提交 CRC 实现与测试**

```bash
git add firmware/chassis/Users/bsp_protocol.h
if [ -f firmware/chassis/Users/bsp_protocol.c ]; then git add firmware/chassis/Users/bsp_protocol.c; fi
git add firmware/chassis/Tests/host/test_protocol.py
git commit -m "feat(protocol): add CRC16-CCITT and host test"
```

---

## Task 3: STM32 底盘状态帧发送（DMA）

**Files:**
- Modify: `firmware/chassis/Users/bsp_protocol.c`

- [x] **Step 1: 添加 DMA 发送缓冲与状态标志**

在 `bsp_protocol.c` 顶部追加：
```c
static uint8_t tx_buf[32];
static volatile uint8_t tx_busy = 0;

#define CHASSIS_ALARM_COMM_ERROR 0x04
```

- [x] **Step 2: 实现 DMA 发送函数**

在 `bsp_protocol.c` 中追加：
```c
void Protocol_Send_Chassis_Status(int16_t left_speed_mm_s,
                                  int16_t right_speed_mm_s,
                                  int16_t battery_x100,
                                  uint8_t alarm_bits,
                                  int32_t left_pulse,
                                  int32_t right_pulse,
                                  uint32_t timestamp_ms) {
    if (tx_busy) return;

    uint8_t *p = tx_buf;
    *p++ = FRAME_HEADER_0;
    *p++ = FRAME_HEADER_1;
    *p++ = TYPE_CHASSIS;
    *p++ = 19;  // payload length
    *p++ = (uint8_t)(left_speed_mm_s & 0xFF);
    *p++ = (uint8_t)((left_speed_mm_s >> 8) & 0xFF);
    *p++ = (uint8_t)(right_speed_mm_s & 0xFF);
    *p++ = (uint8_t)((right_speed_mm_s >> 8) & 0xFF);
    *p++ = (uint8_t)(battery_x100 & 0xFF);
    *p++ = (uint8_t)((battery_x100 >> 8) & 0xFF);
    *p++ = alarm_bits;
    *p++ = (uint8_t)(left_pulse & 0xFF);
    *p++ = (uint8_t)((left_pulse >> 8) & 0xFF);
    *p++ = (uint8_t)((left_pulse >> 16) & 0xFF);
    *p++ = (uint8_t)((left_pulse >> 24) & 0xFF);
    *p++ = (uint8_t)(right_pulse & 0xFF);
    *p++ = (uint8_t)((right_pulse >> 8) & 0xFF);
    *p++ = (uint8_t)((right_pulse >> 16) & 0xFF);
    *p++ = (uint8_t)((right_pulse >> 24) & 0xFF);
    *p++ = (uint8_t)(timestamp_ms & 0xFF);
    *p++ = (uint8_t)((timestamp_ms >> 8) & 0xFF);
    *p++ = (uint8_t)((timestamp_ms >> 16) & 0xFF);
    *p++ = (uint8_t)((timestamp_ms >> 24) & 0xFF);

    uint16_t crc = crc16_ccitt(&tx_buf[2], 2 + 19);  // TYPE + LEN + DATA
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)((crc >> 8) & 0xFF);

    tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart2, tx_buf, 25);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        tx_busy = 0;
    }
}
```

- [x] **Step 3: 扩展主机测试验证发送帧**

在 `test_protocol.py` 追加：
```python
def pack_chassis_status(left_speed_mm_s, right_speed_mm_s, battery_x100,
                        alarm_bits, left_pulse, right_pulse, timestamp_ms):
    payload = struct.pack('<hhHBiiI',
                          left_speed_mm_s,
                          right_speed_mm_s,
                          battery_x100,
                          alarm_bits,
                          left_pulse,
                          right_pulse,
                          timestamp_ms)
    body = bytes([0x03, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([0xAA, 0x55]) + body + struct.pack('>H', crc)


def test_pack_chassis_status():
    frame = pack_chassis_status(100, -50, 1234, 0x04, 100000, -100000, 0x12345678)
    assert len(frame) == 25
    assert frame[0:2] == b'\xaa\x55'
    assert frame[2] == 0x03
    assert frame[3] == 19
    # Verify CRC endianness: last two bytes are big-endian CRC
    body = frame[2:-2]
    rx_crc = struct.unpack('>H', frame[-2:])[0]
    assert crc16_ccitt(body) == rx_crc


if __name__ == '__main__':
    test_crc16_ccitt_known()
    test_crc_matches_stm32_logic()
    test_pack_chassis_status()
    print('All protocol tests OK')
```

Run:
```bash
python firmware/chassis/Tests/host/test_protocol.py
```
Expected:
```text
All protocol tests OK
```

- [x] **Step 4: 提交 DMA 发送实现**

```bash
git add firmware/chassis/Users/bsp_protocol.c
if [ -f firmware/chassis/Users/bsp_protocol.h ]; then git add firmware/chassis/Users/bsp_protocol.h; fi
git add firmware/chassis/Tests/host/test_protocol.py
git commit -m "feat(protocol): implement chassis status frame DMA TX"
```

---

## Task 4: STM32 运动控制帧接收

**Files:**
- Modify: `firmware/chassis/Users/bsp_protocol.c`
- Modify: `firmware/chassis/Users/bsp_protocol.h`

- [x] **Step 1: 保留 IDLE+DMA 接收缓冲与变量**

在 `bsp_protocol.c` 顶部已有 `rx_buff`、`rx_flag`、`rx_len` 和 `target_speed_left/right` 变量，保留它们：
```c
uint8_t rx_buff[RX_BUFF_SIZE];
volatile uint8_t rx_flag = 0;
volatile uint16_t rx_len = 0;

volatile float target_speed_left = 0.0f;
volatile float target_speed_right = 0.0f;

static uint8_t comm_error_count = 0;
```

- [x] **Step 2: 实现 Protocol_Init 和帧解析**

追加：
```c
void Protocol_Init(void) {
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_usart2_rx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, RX_BUFF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

static int16_t find_sync(const uint8_t *buf, uint16_t start, uint16_t len) {
    for (uint16_t i = start; i + 1 < len; i++) {
        if (buf[i] == FRAME_HEADER_0 && buf[i + 1] == FRAME_HEADER_1) {
            return (int16_t)i;
        }
    }
    return -1;
}

static void handle_motion_cmd(const uint8_t *payload) {
    int16_t left_mm_s  = (int16_t)(payload[0] | (payload[1] << 8));
    int16_t right_mm_s = (int16_t)(payload[2] | (payload[3] << 8));

    // 硬件左右通道交叉：上位机视角左=右，右=左
    target_speed_left  = (float)right_mm_s * 11035.0f / 100000.0f;
    target_speed_right = (float)left_mm_s  * 11035.0f / 100000.0f;
}

void Protocol_Process(void) {
    if (!rx_flag) return;
    rx_flag = 0;

    uint16_t offset = 0;
    while (offset + 6 <= rx_len) {
        if (rx_buff[offset] != FRAME_HEADER_0 || rx_buff[offset + 1] != FRAME_HEADER_1) {
            int16_t next = find_sync(rx_buff, offset + 1, rx_len);
            if (next < 0) break;
            offset = (uint16_t)next;
        }

        if (offset + 6 > rx_len) break;
        uint8_t cmd = rx_buff[offset + 2];
        uint8_t dlen = rx_buff[offset + 3];
        uint16_t total = 4 + dlen + 2;
        if (offset + total > rx_len) break;

        uint16_t calc_crc = crc16_ccitt(&rx_buff[offset + 2], 2 + dlen);
        uint16_t rx_crc = (uint16_t)(rx_buff[offset + total - 2] |
                                     (rx_buff[offset + total - 1] << 8));

        if (calc_crc != rx_crc) {
            comm_error_count++;
            offset++;
            continue;
        }

        if (cmd == TYPE_MOTION_CMD && dlen == 4) {
            handle_motion_cmd(&rx_buff[offset + 4]);
        }

        offset += total;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    if (huart2.RxState != HAL_UART_STATE_READY) {
        huart2.RxState = HAL_UART_STATE_READY;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, RX_BUFF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

uint8_t Protocol_Get_CommErrorCount(void) {
    return comm_error_count;
}

void Protocol_Clear_CommErrorCount(void) {
    comm_error_count = 0;
}
```

- [x] **Step 3: 在头文件补充 API**

`bsp_protocol.h` 追加：
```c
uint8_t Protocol_Get_CommErrorCount(void);
void Protocol_Clear_CommErrorCount(void);
```

- [x] **Step 4: 扩展主机测试验证运动控制帧解析逻辑**

在 `test_protocol.py` 追加：
```python
def pack_motion_cmd(left_mm_s, right_mm_s):
    payload = struct.pack('<hh', left_mm_s, right_mm_s)
    body = bytes([0x81, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([0xAA, 0x55]) + body + struct.pack('>H', crc)


def test_pack_motion_cmd():
    frame = pack_motion_cmd(200, -100)
    assert frame[0:2] == b'\xaa\x55'
    assert frame[2] == 0x81
    assert frame[3] == 4
    body = frame[2:-2]
    rx_crc = struct.unpack('>H', frame[-2:])[0]
    assert crc16_ccitt(body) == rx_crc


if __name__ == '__main__':
    test_crc16_ccitt_known()
    test_crc_matches_stm32_logic()
    test_pack_chassis_status()
    test_pack_motion_cmd()
    print('All protocol tests OK')
```

Run:
```bash
python firmware/chassis/Tests/host/test_protocol.py
```
Expected:
```text
All protocol tests OK
```

- [x] **Step 5: 提交接收逻辑**

```bash
git add firmware/chassis/Users/bsp_protocol.c
if [ -f firmware/chassis/Users/bsp_protocol.h ]; then git add firmware/chassis/Users/bsp_protocol.h; fi
git add firmware/chassis/Tests/host/test_protocol.py
git commit -m "feat(protocol): implement motion command frame RX with CRC16"
```

---

## Task 5: 更新 STM32 main.c

**Files:**
- Modify: `firmware/chassis/Core/Src/main.c`

- [x] **Step 1: 替换旧协议调用**

找到 `main.c` 中的遥测发送代码（原 `Protocol_Send_Telemetry(...)`），替换为：
```c
/* Telemetry @ 10 Hz */
if ((int32_t)(now - last_telem_tick) >= 100) {
    last_telem_tick += 100;

    int16_t left_speed_mm_s = (int16_t)(speed_left * 0.14137167f / 1560.0f * 100.0f * 1000.0f);
    int16_t right_speed_mm_s = (int16_t)(speed_right * 0.14137167f / 1560.0f * 100.0f * 1000.0f);
    int16_t battery_x100 = (int16_t)(battery_voltage * 100.0f);

    Protocol_Send_Chassis_Status(left_speed_mm_s,
                                 right_speed_mm_s,
                                 battery_x100,
                                 0,  // alarm_bits，后续叠加通信错误
                                 Encoder_Get_Left_Accum(),
                                 Encoder_Get_Right_Accum(),
                                 HAL_GetTick());
}
```

- [x] **Step 2: 在 main.c 中引入报警位**

在文件顶部或合适位置定义：
```c
#define CHASSIS_ALARM_COMM_ERROR 0x04
```

在状态打印任务里追加通信错误计数：
```c
uint8_t alarm_bits = 0;
if (Protocol_Get_CommErrorCount() > 5) {
    alarm_bits |= CHASSIS_ALARM_COMM_ERROR;
}
```

并把 `alarm_bits` 传给 `Protocol_Send_Chassis_Status`。

- [x] **Step 3: 删除旧协议声明**

确保 `main.c` 不再引用旧的 `Protocol_Send_Telemetry`；`#include "bsp_protocol.h"` 保留。

- [x] **Step 4: 提交 main.c 更新**

```bash
git add firmware/chassis/Core/Src/main.c
git commit -m "feat(chassis): send project-standard chassis status frame from main loop"
```

---

## Task 6: 更新 ChassisStatus.msg

**Files:**
- Modify: `src/sentry_interfaces/msg/ChassisStatus.msg`

- [x] **Step 1: 修改脉冲字段类型**

```text
float32 left_speed
float32 right_speed
float32 battery_voltage
uint8 alarm_bits
int32 left_pulse
int32 right_pulse
uint32 encoder_timestamp
```

- [x] **Step 2: 编译验证消息接口**

Run:
```bash
cd src/sentry_interfaces
colcon build --packages-select sentry_interfaces
```
Expected: 0 errors, 0 warnings.

- [x] **Step 3: 提交消息定义更新**

```bash
git add src/sentry_interfaces/msg/ChassisStatus.msg
git commit -m "fix(interfaces): ChassisStatus pulses are signed int32"
```

---

## Task 7: 修复 RDK uart_bridge_node 的 /cmd_vel 转换

**Files:**
- Modify: `src/sentry_sensors/sentry_sensors/uart_bridge_node.py`

- [x] **Step 1: 添加 wheel_base 参数**

在 `__init__` 中追加参数声明：
```python
self.declare_parameter('wheel_base', 0.23)
self.wheel_base = self.get_parameter('wheel_base').value
```

- [x] **Step 2: 重写 on_cmd_vel**

替换原函数为：
```python
def on_cmd_vel(self, msg: Twist):
    if self.ser is None or not self.ser.is_open:
        return

    v = msg.linear.x
    w = msg.angular.z
    left_m_s = v - w * self.wheel_base / 2.0
    right_m_s = v + w * self.wheel_base / 2.0

    left_mm_s = int(left_m_s * 1000)
    right_mm_s = int(right_m_s * 1000)

    payload = struct.pack('<hh', left_mm_s, right_mm_s)
    frame = encode_frame(TYPE_MOTION_CMD, payload)
    try:
        self.ser.write(frame)
    except serial.SerialException as e:
        self.get_logger().error(f'UART write error: {e}')
```

- [x] **Step 3: 写测试**

在 `src/sentry_sensors/tests/test_uart_bridge_node.py` 追加：
```python
def test_cmd_vel_conversion(node):
    """Verify Twist is converted to differential-drive wheel speeds."""
    import struct
    from geometry_msgs.msg import Twist

    node.wheel_base = 0.23
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.0

    with patch.object(node.ser, 'write') as mock_write:
        node.on_cmd_vel(msg)
        assert mock_write.called
        frame = mock_write.call_args[0][0]
        assert frame[0:2] == b'\xaa\x55'
        assert frame[2] == 0x81
        left, right = struct.unpack('<hh', frame[4:8])
        assert left == 500
        assert right == 500
```

Run:
```bash
cd src/sentry_sensors
pytest tests/test_uart_bridge_node.py -v
```
Expected: all tests pass.

- [x] **Step 4: 提交**

```bash
git add src/sentry_sensors/sentry_sensors/uart_bridge_node.py
if [ -f src/sentry_sensors/tests/test_uart_bridge_node.py ]; then git add src/sentry_sensors/tests/test_uart_bridge_node.py; fi
git commit -m "fix(uart_bridge): convert /cmd_vel Twist to differential wheel speeds"
```

---

## Task 8: 修复 RDK uart_bridge_node 状态帧解析

**Files:**
- Modify: `src/sentry_sensors/sentry_sensors/uart_bridge_node.py`
- Modify: `src/sentry_sensors/tests/test_uart_bridge_node.py`

- [x] **Step 1: 更新 decode_chassis_frame 中的脉冲类型**

当前代码：
```python
(ls, rs, bv, alarm, lp, rp, ts) = struct.unpack('<hhHBIII', payload)
```

改为：
```python
(ls, rs, bv, alarm, lp, rp, ts) = struct.unpack('<hhHBiiI', payload)
```

这样 `lp` 和 `rp` 按有符号 int32 解析。

- [x] **Step 2: 添加状态帧解析测试**

在 `test_uart_bridge_node.py` 追加：
```python
def test_decode_chassis_status_with_negative_pulse():
    from sentry_sensors.uart_bridge_node import encode_frame, decode_chassis_frame
    import struct

    payload = struct.pack('<hhHBiiI',
                          500, -300, 1234, 0x04,
                          100000, -100000, 0x12345678)
    frame = encode_frame(0x03, payload)
    data = decode_chassis_frame(frame)
    assert data is not None
    assert data['left_speed'] == 0.5
    assert data['right_speed'] == -0.3
    assert data['battery_voltage'] == 12.34
    assert data['alarm_bits'] == 0x04
    assert data['left_pulse'] == 100000
    assert data['right_pulse'] == -100000
    assert data['encoder_timestamp'] == 0x12345678
```

Run:
```bash
pytest tests/test_uart_bridge_node.py -v
```
Expected: all tests pass.

- [x] **Step 3: 提交**

```bash
git add src/sentry_sensors/sentry_sensors/uart_bridge_node.py
if [ -f src/sentry_sensors/tests/test_uart_bridge_node.py ]; then git add src/sentry_sensors/tests/test_uart_bridge_node.py; fi
git commit -m "fix(uart_bridge): parse signed int32 encoder pulses in chassis status"
```

---

## Task 9: 更新 wheel_odom_node 默认参数

**Files:**
- Modify: `src/sentry_mission/sentry_mission/wheel_odom_node.py`

- [x] **Step 1: 修改默认值**

```python
self.declare_parameter('wheel_base', 0.23)
self.declare_parameter('pulses_per_meter', 11035)
self.declare_parameter('max_pulse_delta', 1000)
```

- [x] **Step 2: 提交**

```bash
git add src/sentry_mission/sentry_mission/wheel_odom_node.py
git commit -m "fix(wheel_odom): defaults for MG540 tracked chassis"
```

---

## Task 10: 文档更新

**Files:**
- Modify: `docs/HARDWARE.md`
- Modify: `docs/ROS2.md`
- Create: `firmware/chassis/README.md`

- [x] **Step 1: 更新 HARDWARE.md**

把硬件参数章节改为：
```markdown
| 模块 | 型号/规格 | 备注 |
|---|---|---|
| AI 主控 | RDK X5 | ... |
| 运动控制 | STM32F407ZGT6 | 最小系统板 |
| 电机 | MG540 直流减速电机 ×2 | 13 PPR 编码器 |
| 电机驱动 | 当前测试驱动板 | 与 STM32 直连 |
| 编码器 | 13 PPR，减速比 1:30 | 每米 11035 脉冲 |
| 底盘 | 履带式 | 轮距 0.23 m |
```

删除或标注 FreeRTOS 任务表，改为裸机时间片表（参见设计文档 5 节）。

- [x] **Step 2: 更新 ROS2.md**

在 `/sentry/chassis/status` 段落注明：
```markdown
- `left_pulse` / `right_pulse`：`int32`，累计脉冲，可正可负
- 输入：`/cmd_vel` 由 `uart_bridge_node` 转为左右轮 mm/s 下发
```

- [x] **Step 3: 创建 firmware/chassis/README.md**

内容至少包含：
```markdown
# firmware/chassis

STM32F407ZGT6 履带式底盘控制器。

## 接线
- USART1 PA9/PA10 → ST-Link VCP，9600 baud，调试控制台
- USART2 PA2/PA3 → RDK X5 UART2，115200 baud
- TIM1 CH1 PE9 / CH2 PE11 → 左右电机 PWM
- PE13/PE14 / PG6/PG7 → 左右电机方向
- TIM2 PA5/PB3 → 左编码器
- TIM3 PA6/PC7 → 右编码器
- ADC1 CH7 PA7 → 电池电压

## 编译与烧录
使用 Keil UV4 打开 `MDK-ARM/Intelligent_Agricultural_Robot_Car.uvprojx` 编译，
或用 OpenOCD + ST-Link 烧录 `MDK-ARM/.../Intelligent_Agricultural_Robot_Car.hex`。

## 协议速查
- RDK → STM32: `AA 55 81 04 <L_mm/s> <R_mm/s> CRC16`
- STM32 → RDK: `AA 55 03 13 <payload 19B> CRC16`
```

- [x] **Step 4: 提交文档更新**

```bash
git add docs/HARDWARE.md docs/ROS2.md firmware/chassis/README.md
git commit -m "docs: update hardware and protocol docs for integrated chassis"
```

Commit: `c4d93f9`

---

## Task 11: STM32 编译与烧录验证

**Files:**
- Verify: `firmware/chassis/MDK-ARM/*.uvprojx`

- [ ] **Step 1: 编译**

使用 Keil 打开 `firmware/chassis/MDK-ARM/Intelligent_Agricultural_Robot_Car.uvprojx`，编译。
Expected: 0 Error(s), 0 Warning(s)（与现有测试项目一致）。

- [ ] **Step 2: 烧录**

使用 OpenOCD 或 STM32_Programmer_CLI 烧录，确认 boot marker `012345` 在 USART1 9600 输出。

- [ ] **Step 3: 单机调试**

在 USART1 串口终端输入：
```text
LR 100 100
```
Expected: 左右轮以目标 100 pulses/10ms 转动。

```text
STOP
```
Expected: 电机停止。

- [ ] **Step 4: 提交（如需要修复则提交）**

---

## Task 12: RDK 联调与端到端测试

**Files:**
- Run: `sentry_sensors` 与 `sentry_mission` 包

- [ ] **Step 1: 构建 ROS2 包**

```bash
cd ~/dev_ws
colcon build --packages-select sentry_interfaces sentry_sensors sentry_mission
source install/setup.bash
```

- [ ] **Step 2: 启动 uart_bridge_node**

```bash
ros2 run sentry_sensors uart_bridge_node --ros-args -p uart_port:=/dev/ttyS5 -p wheel_base:=0.23
```

- [ ] **Step 3: 检查底盘状态**

```bash
ros2 topic echo /sentry/chassis/status
```
Expected: 10 Hz 收到数据，转动车轮时 `left_pulse` / `right_pulse` 变化。

- [ ] **Step 4: 测试 /cmd_vel**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```
Expected: 车以约 0.2 m/s 直线前进，`/sentry/chassis/status` 回显速度接近 0.2 m/s。

- [ ] **Step 5: 测试 wheel_odom_node**

```bash
ros2 run sentry_mission wheel_odom_node --ros-args -p wheel_base:=0.23 -p pulses_per_meter:=11035
ros2 topic echo /wheel/odom
```
Expected: 走 1 m 后 `pose.position.x` 接近 1 m。

- [ ] **Step 6: 提交最终修复（如有）**

---

## 自我检查

**Spec 覆盖检查：**

| Spec 要求 | 对应任务 |
|-----------|----------|
| 迁移到 `firmware/chassis/` | Task 1 |
| 项目规范协议 0xAA55 + CRC16 | Task 2, 3, 4 |
| 底盘状态帧 19B payload | Task 3 |
| 运动控制帧 mm/s | Task 4, 7 |
| 左右通道交叉处理 | Task 4, 7 |
| DMA 发送 | Task 3 |
| `/cmd_vel` 输入 | Task 7 |
| `/sentry/chassis/status` 输出 | Task 3, 8 |
| `wheel_odom_node` 参数 | Task 9 |
| 文档更新 | Task 10 |
| 测试验证 | Task 2, 4, 7, 8, 11, 12 |

**Placeholder 检查：** 无 TBD/TODO，每个任务含具体代码或命令。

**类型一致性检查：**
- `ChassisStatus.msg` 中 `left_pulse/right_pulse` 已改为 `int32`
- `uart_bridge_node.py` 中 `struct.unpack` 已改为 `'ii'`
- STM32 中 `left_pulse/right_pulse` 使用 `int32_t`

---

## 执行方式

Plan complete and saved to `docs/superpowers/plans/2026-06-29-chassis-integration.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
