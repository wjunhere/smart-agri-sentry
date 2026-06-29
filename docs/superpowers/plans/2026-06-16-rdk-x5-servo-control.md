# RDK X5 云台舵机直接驱动实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 RDK X5 上通过 40-pin PWM 直接驱动两路舵机，交付独立键盘脚本和 ROS2 节点两个入口。

**Architecture:** 公共 `Servo` 类封装 Linux sysfs PWM；`servo_keyboard.py` 作为零 ROS 依赖的独立脚本；`servo_driver_node.py` 作为 ROS2 节点订阅 `/sentry/servo_cmd`；配置全部收敛到 `servo_config.yaml`。

**Tech Stack:** Python 3 + ROS2 Humble (`rclpy` / `ament_python`) + `sentry_interfaces` + `PyYAML`

---

## 文件结构总览

```text
src/sentry_servo/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── servo_config.yaml
├── resource/
│   └── sentry_servo
├── sentry_servo/
│   ├── __init__.py
│   ├── servo_driver.py
│   ├── servo_keyboard.py
│   └── servo_driver_node.py
└── tests/
    ├── test_servo_driver.py
    └── test_servo_driver_node.py
```

---

## Task 0：更新根目录 `PLAN.md`

**Files:**
- Modify: `PLAN.md`

- [ ] **Step 1：写入任务追踪清单**

```markdown
# RDK X5 云台舵机直接驱动追踪

- [ ] Task 1：创建 `sentry_servo` ROS2 Python 包骨架
- [ ] Task 2：TDD 实现 `servo_driver.py`
- [ ] Task 3：实现独立键盘脚本 `servo_keyboard.py`
- [ ] Task 4：实现并测试 ROS2 节点 `servo_driver_node.py`
- [ ] Task 5：`colcon build` 与 `colcon test`
- [ ] Task 6：最终审查与提交
```

- [ ] **Step 2：提交 `PLAN.md`（仅当用户明确同意提交时）**

---

## Task 1：创建 `sentry_servo` 包骨架

**Files:**
- Create: `src/sentry_servo/setup.py`
- Create: `src/sentry_servo/setup.cfg`
- Create: `src/sentry_servo/package.xml`
- Create: `src/sentry_servo/resource/sentry_servo`
- Create: `src/sentry_servo/sentry_servo/__init__.py`
- Create: `src/sentry_servo/config/servo_config.yaml`

- [ ] **Step 1：创建 `src/sentry_servo/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'sentry_servo'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/servo_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Direct PWM servo driver for Smart Agri Sentry on RDK X5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_keyboard = sentry_servo.servo_keyboard:main',
            'servo_driver_node = sentry_servo.servo_driver_node:main',
        ],
    },
)
```

- [ ] **Step 2：创建 `src/sentry_servo/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/sentry_servo
[install]
install_scripts=$base/lib/sentry_servo
```

- [ ] **Step 3：创建 `src/sentry_servo/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_servo</name>
  <version>0.2.0</version>
  <description>Direct PWM servo driver for Smart Agri Sentry on RDK X5</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sentry_interfaces</depend>
  <depend>python3-yaml</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4：创建空资源文件 `src/sentry_servo/resource/sentry_servo`**

内容为空文件即可。

- [ ] **Step 5：创建 `src/sentry_servo/sentry_servo/__init__.py`**

```python
# sentry_servo package
```

- [ ] **Step 6：创建 `src/sentry_servo/config/servo_config.yaml`**

```yaml
servo_driver_node:
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

- [ ] **Step 7：提交骨架（仅当用户明确同意提交时）**

```bash
git add src/sentry_servo
git commit -m "chore(sentry_servo): add package skeleton and config"
```

---

## Task 2：TDD 实现 `servo_driver.py`

**Files:**
- Create: `src/sentry_servo/sentry_servo/servo_driver.py`
- Create: `src/sentry_servo/tests/test_servo_driver.py`

### 2.1 写失败测试

- [ ] **Step 1：创建 `src/sentry_servo/tests/test_servo_driver.py`**

```python
import pytest

from sentry_servo.servo_driver import Servo, ServoError


def test_angle_to_duty_ns_at_limits():
    servo = Servo(channel=0, min_angle=0, max_angle=180)
    assert servo.angle_to_duty_ns(0) == 500_000
    assert servo.angle_to_duty_ns(90) == 1_500_000
    assert servo.angle_to_duty_ns(180) == 2_500_000


def test_angle_clamping():
    servo = Servo(channel=0, min_angle=30, max_angle=150)
    assert servo.angle_to_duty_ns(0) == servo.angle_to_duty_ns(30)
    assert servo.angle_to_duty_ns(200) == servo.angle_to_duty_ns(150)


def test_custom_pulse_range():
    servo = Servo(channel=0, min_us=1000, max_us=2000)
    assert servo.angle_to_duty_ns(0) == 1_000_000
    assert servo.angle_to_duty_ns(180) == 2_000_000


def test_sysfs_path_building():
    servo = Servo(channel=1, chip=0)
    assert servo._base == '/sys/class/pwm/pwmchip0'
    assert servo._path == '/sys/class/pwm/pwmchip0/pwm1'
```

- [ ] **Step 2：运行测试确认失败**

```bash
cd src/sentry_servo
python -m pytest tests/test_servo_driver.py -v
```

Expected: FAIL（`Servo` 未定义）

### 2.2 实现驱动

- [ ] **Step 3：创建 `src/sentry_servo/sentry_servo/servo_driver.py`**

```python
import errno
import os


class ServoError(Exception):
    """Raised when PWM sysfs operations fail."""


class Servo:
    """Linux sysfs PWM servo driver."""

    def __init__(self, channel: int, chip: int = 0,
                 freq_hz: int = 50,
                 min_us: int = 500, max_us: int = 2500,
                 min_angle: float = 0.0, max_angle: float = 180.0,
                 name: str = 'servo'):
        self.channel = int(channel)
        self.chip = int(chip)
        self.freq_hz = int(freq_hz)
        self.period_ns = int(1_000_000_000 / self.freq_hz)
        self.min_us = int(min_us)
        self.max_us = int(max_us)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.name = name
        self.last_angle = self.min_angle

        self._base = f'/sys/class/pwm/pwmchip{self.chip}'
        self._path = f'{self._base}/pwm{self.channel}'
        self._enabled = False

    def _write(self, name: str, value: int) -> None:
        path = os.path.join(self._path, name)
        try:
            with open(path, 'w') as f:
                f.write(str(value))
        except OSError as exc:
            raise ServoError(
                f'Failed to write {value} to {path}: {exc}') from exc

    def _export(self) -> None:
        if os.path.exists(self._path):
            return
        export_path = os.path.join(self._base, 'export')
        try:
            with open(export_path, 'w') as f:
                f.write(str(self.channel))
        except OSError as exc:
            if exc.errno != errno.EBUSY:
                raise ServoError(
                    f'Failed to export PWM {self.chip}/{self.channel}: {exc}') from exc

    def enable(self) -> None:
        """Export, set period and enable the PWM channel."""
        self._export()
        self._write('period', self.period_ns)
        self._write('duty_cycle', 0)
        self._write('enable', 1)
        self._enabled = True

    def disable(self) -> None:
        """Disable and unexport the PWM channel."""
        if not os.path.exists(self._path):
            self._enabled = False
            return
        try:
            self._write('enable', 0)
        except ServoError:
            pass
        try:
            with open(os.path.join(self._base, 'unexport'), 'w') as f:
                f.write(str(self.channel))
        except OSError:
            pass
        self._enabled = False

    def angle_to_duty_ns(self, angle: float) -> int:
        """Map an angle in degrees to duty cycle in nanoseconds."""
        clamped = max(self.min_angle, min(self.max_angle, float(angle)))
        pulse_us = self.min_us + (clamped / 180.0) * (self.max_us - self.min_us)
        return int(pulse_us * 1000)

    def set_angle(self, angle: float) -> None:
        """Move servo to the requested angle (clamped to limits)."""
        if not self._enabled:
            self.enable()
        clamped = max(self.min_angle, min(self.max_angle, float(angle)))
        self.last_angle = clamped
        duty_ns = self.angle_to_duty_ns(clamped)
        self._write('duty_cycle', duty_ns)
```

- [ ] **Step 4：运行测试确认通过**

```bash
cd src/sentry_servo
python -m pytest tests/test_servo_driver.py -v
```

Expected: PASS

- [ ] **Step 5：提交驱动实现（仅当用户明确同意提交时）**

```bash
git add src/sentry_servo/sentry_servo/servo_driver.py src/sentry_servo/tests/test_servo_driver.py
git commit -m "feat(sentry_servo): add sysfs PWM driver with angle mapping"
```

---

## Task 3：实现独立键盘脚本 `servo_keyboard.py`

**Files:**
- Create: `src/sentry_servo/sentry_servo/servo_keyboard.py`

- [ ] **Step 1：创建 `src/sentry_servo/sentry_servo/servo_keyboard.py`**

```python
#!/usr/bin/env python3
"""Standalone keyboard servo controller for RDK X5."""

import argparse
import os
import select
import sys
import termios
import tty

import yaml

from sentry_servo.servo_driver import Servo


def _default_config():
    return {
        'pwm': {
            'chip': 0,
            'frequency_hz': 50,
            'min_pulse_us': 500,
            'max_pulse_us': 2500,
        },
        'servos': {
            'yaw': {
                'channel': 0,
                'min_angle': 0,
                'max_angle': 180,
                'initial_angle': 90,
                'step_deg': 5,
            },
            'pitch': {
                'channel': 1,
                'min_angle': 30,
                'max_angle': 150,
                'initial_angle': 90,
                'step_deg': 5,
            },
        },
    }


def _load_config(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _merge_config(user):
    cfg = _default_config()
    if not user:
        return cfg
    pwm = user.get('pwm', {})
    for key in cfg['pwm']:
        cfg['pwm'][key] = pwm.get(key, cfg['pwm'][key])
    for name in ('yaw', 'pitch'):
        servo = user.get('servos', {}).get(name, {})
        cfg['servos'][name].update(servo)
    return cfg


def _make_servo(cfg, name):
    pwm = cfg['pwm']
    servo = cfg['servos'][name]
    return Servo(
        channel=servo['channel'],
        chip=pwm['chip'],
        freq_hz=pwm['frequency_hz'],
        min_us=pwm['min_pulse_us'],
        max_us=pwm['max_pulse_us'],
        min_angle=servo['min_angle'],
        max_angle=servo['max_angle'],
        name=name,
    )


def _getch(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    parser = argparse.ArgumentParser(
        description='Keyboard control for RDK X5 PWM servos')
    parser.add_argument(
        '--config', default='config/servo_config.yaml',
        help='Path to servo_config.yaml')
    args = parser.parse_args()

    cfg = _merge_config(_load_config(args.config))

    yaw = _make_servo(cfg, 'yaw')
    pitch = _make_servo(cfg, 'pitch')

    yaw_cfg = cfg['servos']['yaw']
    pitch_cfg = cfg['servos']['pitch']

    yaw.set_angle(yaw_cfg['initial_angle'])
    pitch.set_angle(pitch_cfg['initial_angle'])

    print('Controls: ←/→ yaw  ↑/↓ pitch  r=reset  q/ESC=quit')

    try:
        while True:
            ch = _getch()
            if ch is None:
                continue
            if ch == '\x1b':
                bracket = _getch(0.05)
                if bracket == '[':
                    key = _getch(0.05)
                    if key == 'C':
                        yaw.set_angle(yaw.last_angle + yaw_cfg['step_deg'])
                    elif key == 'D':
                        yaw.set_angle(yaw.last_angle - yaw_cfg['step_deg'])
                    elif key == 'A':
                        pitch.set_angle(pitch.last_angle + pitch_cfg['step_deg'])
                    elif key == 'B':
                        pitch.set_angle(pitch.last_angle - pitch_cfg['step_deg'])
            elif ch.lower() == 'r':
                yaw.set_angle(yaw_cfg['initial_angle'])
                pitch.set_angle(pitch_cfg['initial_angle'])
            elif ch.lower() == 'q' or ch == '\x03':
                break
    finally:
        yaw.disable()
        pitch.disable()
        print('Servos disabled.')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2：语法检查**

```bash
python -m py_compile src/sentry_servo/sentry_servo/servo_keyboard.py
```

Expected: 无输出（编译成功）

- [ ] **Step 3：RDK X5 硬件验证（手动）**

在 RDK X5 终端（项目根目录）执行：

```bash
source install/setup.bash
ros2 run sentry_servo servo_keyboard --config install/sentry_servo/share/sentry_servo/config/servo_config.yaml
```

验证：
- 按 `←/→` 水平舵机左右动；
- 按 `↑/↓` 俯仰舵机上下动；
- 按 `r` 双舵机回到中位；
- 按 `q` 退出后 `/sys/class/pwm/pwmchip0/pwm0` 和 `pwm1` 被释放。

- [ ] **Step 4：提交键盘脚本（仅当用户明确同意提交时）**

```bash
git add src/sentry_servo/sentry_servo/servo_keyboard.py
git commit -m "feat(sentry_servo): add standalone keyboard servo controller"
```

---

## Task 4：实现并测试 ROS2 节点 `servo_driver_node.py`

**Files:**
- Create: `src/sentry_servo/sentry_servo/servo_driver_node.py`
- Create: `src/sentry_servo/tests/test_servo_driver_node.py`

### 4.1 写失败测试

- [ ] **Step 1：创建 `src/sentry_servo/tests/test_servo_driver_node.py`**

```python
import pytest
import rclpy
from unittest.mock import MagicMock, patch

from sentry_servo.servo_driver_node import ServoDriverNode
from sentry_interfaces.msg import ServoCmd


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    with patch('sentry_servo.servo_driver_node.Servo') as MockServo:
        MockServo.return_value = MagicMock()
        n = ServoDriverNode()
        yield n
        n.destroy_node()


def test_servo_cmd_sets_pitch_and_yaw(node):
    msg = ServoCmd()
    msg.pitch = 60
    msg.yaw = 120

    node.on_servo_cmd(msg)

    assert node.pitch.set_angle.call_args[0][0] == 60
    assert node.yaw.set_angle.call_args[0][0] == 120


def test_initial_angles_applied():
    with patch('sentry_servo.servo_driver_node.Servo') as MockServo:
        mock = MagicMock()
        MockServo.return_value = mock
        node = ServoDriverNode()
        assert mock.set_angle.call_count == 2
        node.destroy_node()
```

- [ ] **Step 2：运行测试确认失败**

```bash
cd src/sentry_servo
python -m pytest tests/test_servo_driver_node.py -v
```

Expected: FAIL（`ServoDriverNode` 未定义）

### 4.2 实现 ROS2 节点

- [ ] **Step 3：创建 `src/sentry_servo/sentry_servo/servo_driver_node.py`**

```python
import os

import rclpy
from rclpy.node import Node
import yaml

from sentry_interfaces.msg import ServoCmd
from sentry_servo.servo_driver import Servo


class ServoDriverNode(Node):
    """ROS2 node that drives PWM servos from /sentry/servo_cmd."""

    def __init__(self):
        super().__init__('servo_driver_node')
        self.declare_parameter('config_path', '')

        cfg = self._load_config(self.get_parameter('config_path').value)
        pwm_cfg = cfg.get('pwm', {})
        servos_cfg = cfg.get('servos', {})

        self.yaw = self._create_servo(servos_cfg.get('yaw', {}), pwm_cfg)
        self.pitch = self._create_servo(servos_cfg.get('pitch', {}), pwm_cfg)

        self.sub = self.create_subscription(
            ServoCmd, '/sentry/servo_cmd', self.on_servo_cmd, 10)

        self.yaw.set_angle(servos_cfg.get('yaw', {}).get('initial_angle', 90))
        self.pitch.set_angle(
            servos_cfg.get('pitch', {}).get('initial_angle', 90))

        self.get_logger().info('Servo driver node ready')

    def _default_config(self):
        return {
            'pwm': {
                'chip': 0,
                'frequency_hz': 50,
                'min_pulse_us': 500,
                'max_pulse_us': 2500,
            },
            'servos': {
                'yaw': {
                    'channel': 0,
                    'min_angle': 0,
                    'max_angle': 180,
                    'initial_angle': 90,
                },
                'pitch': {
                    'channel': 1,
                    'min_angle': 30,
                    'max_angle': 150,
                    'initial_angle': 90,
                },
            },
        }

    def _load_config(self, path):
        if not path:
            return self._default_config()
        if not os.path.isabs(path):
            candidates = [
                path,
                os.path.join(os.getcwd(), path),
                os.path.join(
                    os.path.dirname(__file__), '..', '..', '..', path),
            ]
            for c in candidates:
                if os.path.exists(c):
                    path = c
                    break
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        self.get_logger().warn(f'Config not found: {path}, using defaults')
        return self._default_config()

    def _create_servo(self, servo_cfg, pwm_cfg):
        return Servo(
            channel=servo_cfg.get('channel', 0),
            chip=pwm_cfg.get('chip', 0),
            freq_hz=pwm_cfg.get('frequency_hz', 50),
            min_us=pwm_cfg.get('min_pulse_us', 500),
            max_us=pwm_cfg.get('max_pulse_us', 2500),
            min_angle=servo_cfg.get('min_angle', 0),
            max_angle=servo_cfg.get('max_angle', 180),
            name=servo_cfg.get('name', 'servo'),
        )

    def on_servo_cmd(self, msg: ServoCmd):
        self.pitch.set_angle(float(msg.pitch))
        self.yaw.set_angle(float(msg.yaw))

    def destroy_node(self):
        self.yaw.disable()
        self.pitch.disable()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4：运行测试确认通过**

```bash
cd src/sentry_servo
python -m pytest tests/test_servo_driver_node.py -v
```

Expected: PASS

- [ ] **Step 5：提交 ROS2 节点（仅当用户明确同意提交时）**

```bash
git add src/sentry_servo/sentry_servo/servo_driver_node.py src/sentry_servo/tests/test_servo_driver_node.py
git commit -m "feat(sentry_servo): add ROS2 servo driver node for /sentry/servo_cmd"
```

---

## Task 5：`colcon build` 与 `colcon test`

- [ ] **Step 1：编译新包**

```bash
cd E:/smart_agri_sentry
colcon build --packages-select sentry_servo --symlink-install
```

Expected: `sentry_servo` 包 `Finished` 成功。

- [ ] **Step 2：运行测试**

```bash
colcon test --packages-select sentry_servo
```

Expected: 所有测试 `passed`。

- [ ] **Step 3：查看测试结果**

```bash
colcon test-result --verbose
```

Expected: 无失败用例。

- [ ] **Step 4：ROS2 节点硬件验证（手动）**

```bash
source install/setup.bash
ros2 run sentry_servo servo_driver_node --ros-args -p config_path:=install/sentry_servo/share/sentry_servo/config/servo_config.yaml
```

另开终端：

```bash
ros2 topic pub /sentry/servo_cmd sentry_interfaces/msg/ServoCmd "{pitch: 120, yaw: 45}"
```

验证：俯仰舵机转到约 120°，水平舵机转到约 45°。

---

## Task 6：最终审查与提交

- [ ] **Step 1：运行 lint 检查**

```bash
cd src/sentry_servo
python -m flake8 sentry_servo tests --max-line-length=100
```

Expected: 无严重风格错误。

- [ ] **Step 2：更新根目录 `PLAN.md` 为全部完成状态**

```markdown
# RDK X5 云台舵机直接驱动追踪

- [x] Task 1：创建 `sentry_servo` ROS2 Python 包骨架
- [x] Task 2：TDD 实现 `servo_driver.py`
- [x] Task 3：实现独立键盘脚本 `servo_keyboard.py`
- [x] Task 4：实现并测试 ROS2 节点 `servo_driver_node.py`
- [x] Task 5：`colcon build` 与 `colcon test`
- [x] Task 6：最终审查与提交
```

- [ ] **Step 3：提交（仅当用户明确同意提交时）**

```bash
git add src/sentry_servo docs/superpowers/specs/2026-06-16-rdk-x5-servo-control-design.md docs/superpowers/plans/2026-06-16-rdk-x5-servo-control.md PLAN.md
git status
git commit -m "feat(sentry_servo): add RDK X5 direct PWM servo driver

- Add Servo sysfs PWM driver with angle clamping
- Add standalone keyboard controller
- Add ROS2 node for /sentry/servo_cmd
- Add config, unit tests and package skeleton"
```

---

## Self-Review Checklist

- [x] **Spec coverage**：包骨架、驱动、独立脚本、ROS2 节点、配置、测试、PWM sysfs 映射均覆盖。
- [x] **Placeholder scan**：无 TBD、TODO、"implement later"、"add appropriate error handling" 等占位。
- [x] **Type consistency**：`ServoCmd` 的 `pitch`/`yaw` 为 `uint8`，节点中统一用 `float()` 转换；`Servo` 类属性命名一致。
- [x] **文件路径**：所有 create/modify 路径均为相对仓库根目录的精确路径。
- [x] **测试命令**：每个任务均给出明确的 `pytest` / `colcon` 命令与期望输出。

---

## 执行方式

Plan complete and saved to `docs/superpowers/plans/2026-06-16-rdk-x5-servo-control.md`.

两个执行选项：

1. **Subagent-Driven（推荐）**：每个 Task 派一个独立 subagent 执行，我在每轮后 review 结果，适合复杂/多文件改动。
2. **Inline Execution**：在当前会话中使用 `superpowers:executing-plans` 批量执行，中间设 checkpoint 供你确认。

你想用哪种？
