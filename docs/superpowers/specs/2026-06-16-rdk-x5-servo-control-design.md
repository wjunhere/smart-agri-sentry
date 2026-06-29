# RDK X5 云台舵机直接驱动设计

> 设计日期：2026-06-16
> 对应需求：在 RDK X5 上直接驱动 32/33 脚的 PWM 舵机，支持键盘控制

---

## 1. 背景与目标

项目已有 `/sentry/servo_cmd` → `uart_bridge_node` → STM32 的云台控制链路。本设计实现 **RDK X5 直接通过 40-pin 接口驱动两路舵机**，用于：

1. 脱离 STM32 快速验证云台硬件；
2. 为后续 mission_control 增加一条可选的本地 PWM 控制路径。

## 2. 硬件确认

- 舵机型号：幻尔 LFD-01M（参数见 `docs/hardware_refs/steeringengine/产品参数图.md`）
  - 工作电压：4.8–6 V
  - 转动范围：0–180°
  - PWM 脉宽：500–2500 µs，对应 0–180°
  - 控制频率：50 Hz（周期 20 ms）
- 接线：
  - 32 脚 → 水平舵机（yaw）
  - 33 脚 → 俯仰舵机（pitch）
- RDK X5 实际 PWM 映射（SSH 已验证）：

| 物理引脚 | 官方 PWM 名 | sysfs 路径 | 用途 |
|---------|------------|-----------|------|
| 32 脚 | pwm6 | `/sys/class/pwm/pwmchip0/pwm0` | yaw（水平） |
| 33 脚 | pwm7 | `/sys/class/pwm/pwmchip0/pwm1` | pitch（俯仰） |

> 官方文档把该控制器标为 `pwmchip3`，但在当前内核枚举中显示为 `pwmchip0`。如后续内核升级后枚举变化，可通过 YAML 配置调整。

- 权限：`sunrise` 用户已在 `gpio` 组，可直接读写 PWM sysfs，无需每次 sudo。
- 若 32/33 脚默认仍为 I2C 功能，需启用 PWM overlay：
  ```bash
  sudo srpi-config
  # Interface Options → PWM → 启用 PWM3
  # 或在 /boot/config.txt 中加入：dtoverlay=dtoverlay_pwm3
  sudo reboot
  ```

## 3. 方案选择

采用 **方案 B：公共驱动模块 + 两个入口**：

- `servo_driver.py`：单一驱动实现，避免 PWM 初始化、角度换算、限位逻辑重复。
- `servo_keyboard.py`：零依赖 ROS2 的独立脚本，用于硬件验证。
- `servo_driver_node.py`：ROS2 节点，复用现有 `ServoCmd` 消息，方便被 `mission_control` 调用。

## 4. 文件布局

```
src/sentry_servo/
├── package.xml
├── setup.py
├── config/
│   └── servo_config.yaml       # PWM chip/channel、限位、步长
├── sentry_servo/
│   ├── __init__.py
│   ├── servo_driver.py         # Servo 类
│   ├── servo_keyboard.py       # 独立键盘脚本
│   └── servo_driver_node.py    # ROS2 节点
└── tests/
    └── test_servo_driver.py    # 角度映射单元测试
```

## 5. 组件设计

### 5.1 `servo_driver.py`

```python
class Servo:
    def __init__(self, channel: int, chip: int = 0,
                 freq_hz: int = 50,
                 min_us: int = 500, max_us: int = 2500,
                 min_angle: int = 0, max_angle: int = 180,
                 name: str = "servo"):
        ...

    def set_angle(self, angle: float) -> None:
        """限位、换算并写入 duty_cycle。"""

    def enable(self) -> None:
        """导出 PWM 通道、设置周期、使能输出。"""

    def disable(self) -> None:
        """禁用并取消导出 PWM 通道。"""
```

换算公式：
```
duty_us = 500 + (angle / 180.0) * 2000
duty_ns = duty_us * 1000
period_ns = 20_000_000
```

### 5.2 `servo_keyboard.py`

- 加载 `config/servo_config.yaml`。
- 初始化两个 `Servo` 实例：
  - yaw：channel 0（32 脚）
  - pitch：channel 1（33 脚）
- 默认角度 90°/90°，启动后自动归中。
- 键位：

| 按键 | 动作 |
|------|------|
| `←` | yaw -5° |
| `→` | yaw +5° |
| `↑` | pitch +5° |
| `↓` | pitch -5° |
| `r` | 复位到 90°/90° |
| `q` / `Esc` | 退出并释放 PWM |

- 使用 `termios` + `select` 读取单键，不依赖 `pynput`。

### 5.3 `servo_driver_node.py`

- ROS2 节点名：`servo_driver_node`
- 订阅 `/sentry/servo_cmd`（类型 `sentry_interfaces/msg/ServoCmd`）
  - `pitch` → 俯仰舵机
  - `yaw`   → 水平舵机
- 通过参数或 YAML 加载 PWM 配置。
- 节点销毁时调用 `Servo.disable()`。

## 6. 配置示例

`config/servo_config.yaml`：

```yaml
pwm:
  chip: 0
  frequency_hz: 50
  min_pulse_us: 500
  max_pulse_us: 2500

servos:
  yaw:
    channel: 0
    min_angle: 0
    max_angle: 180
    initial_angle: 90
    step_deg: 5
  pitch:
    channel: 1
    min_angle: 30
    max_angle: 150
    initial_angle: 90
    step_deg: 5
```

## 7. 数据流

独立脚本：
```
键盘 → servo_keyboard.py → Servo.set_angle() → /sys/class/pwm/pwmchip0/pwm{0,1}
```

ROS2 节点：
```
/sentry/servo_cmd → servo_driver_node.py → Servo.set_angle() → /sys/class/pwm/pwmchip0/pwm{0,1}
```

## 8. 错误处理与安全

- 启动时检测 `/sys/class/pwm/pwmchip{N}` 是否存在；不存在时打印明确诊断信息。
- 角度超出 `[min_angle, max_angle]` 时自动钳位，并打印 warning。
- `SIGINT` / `KeyboardInterrupt` / 节点销毁时，自动 `disable()` 两路 PWM。
- 若导出通道已被占用（`Device or resource busy`），视为已导出，继续初始化。

## 9. 测试计划

| 测试 | 方式 | 通过标准 |
|------|------|---------|
| 角度映射单元测试 | `pytest tests/test_servo_driver.py` | 0°→500us，90°→1500us，180°→2500us |
| 独立脚本硬件测试 | RDK X5 上运行 `python3 servo_keyboard.py` | 按键后对应舵机转动，复位键回到中位 |
| ROS2 节点测试 | `ros2 topic pub /sentry/servo_cmd ...` | 节点收到消息后舵机到达目标角度 |
| 边界保护测试 | 持续按方向键至极限 | 角度被钳位在限位内，不报错 |

## 10. 与现有系统的集成

- 不改动现有 `uart_bridge_node` 和 `mission_control_node`。
- 后续如需 mission_control 使用本地 PWM，只需新增一个发布者：`pub_servo_local = create_publisher(ServoCmd, '/sentry/servo_cmd', 10)`，复用同一话题。

## 11. 后续步骤

设计确认后，将按 `superpowers:writing-plans` 生成实现计划，并执行 TDD + code review。
