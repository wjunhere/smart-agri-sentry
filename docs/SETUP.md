# 环境搭建与编译烧录

> 更新日期：2026-07-05

---

## 1. 开发环境

### 1.1 本地开发机

- **OS**：Ubuntu 22.04（推荐）
- **ROS2**：Humble Hawksbill
- **工具链**：
  - `colcon`：ROS2 构建
  - `rosdep`：依赖安装
  - `git`、`gh`：版本控制
  - `rg`、`fd`、`bat`、`jq`：日常搜索查看（详见 `CLAUDE.md`）

### 1.2 RDK X5 板端

- **OS**：Ubuntu 22.04
- **SSH**：`ssh sunrise@ubuntu.local`
- **密码**：`sunrise`
- **工作区**：`~/dev_ws`（从本仓库克隆，仅文件夹重命名）
- **预装**：ROS2 Humble、NPU 运行时

### 1.3 STM32 开发工具

本地已安装：

- STM32CubeMX
- STM32CubeCLT
- STM32_Programmer_CLI

HAL 库位置：`E:\stm32cubeMXrepository`

---

## 2. 克隆与编译

### 2.1 克隆仓库

```bash
cd ~/agri_ws/src
git clone https://github.com/wjun/smart-agri-sentry.git
cd ..
```

### 2.2 安装依赖

```bash
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/smart_agri_sentry/requirements.txt
```

### 2.3 编译

```bash
colcon build --packages-select \
  sentry_interfaces sentry_bringup sentry_vision \
  sentry_fusion sentry_forecast sentry_advisory \
  sentry_mission sentry_sensors sentry_servo sentry_data_logger

source install/setup.bash
```

### 2.4 运行测试

```bash
colcon test --packages-select sentry_vision sentry_fusion sentry_forecast sentry_advisory sentry_mission sentry_sensors sentry_servo sentry_data_logger
```

---

## 3. 配置

### 3.1 复制示例配置

```bash
# 作物配置
cp config/crop_profiles.yaml.example config/crop_profiles.yaml

# 农艺建议规则
cp config/advisory_rules.yaml.example config/advisory_rules.yaml

# 巡检参数
cp config/mission_params.yaml.example config/mission_params.yaml
```

### 3.2 关键参数说明

| 文件 | 用途 |
|---|---|
| `config/crop_profiles.yaml` | 番茄/小麦/草莓的温度窗口、LWD 阈值 |
| `config/advisory_rules.yaml` | 作物-病害-条件-动作规则 |
| `config/mission_params.yaml` | 巡航速度、植株检测阈值、停车距离 |
| `src/sentry_servo/config/servo_config.yaml` | 云台 yaw/pitch 限位与步长 |
| `src/sentry_lidar/config/stl19p.yaml` | LiDAR 串口、扇区、危险阈值 |
| `src/sentry_mission/config/nav2_no_map.yaml` | 当前无地图 Nav2 配置 |
| `src/sentry_mission/config/ekf.yaml` | EKF 融合参数 |
| `src/sentry_mission/config/waypoints.yaml` | 巡航航点 |

---

## 4. 启动系统

### 4.1 完整系统

```bash
ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato
```

### 4.2 单独调试节点

```bash
# 融合节点
ros2 run sentry_fusion fusion_node --ros-args -p crop_type:=tomato

# 激光雷达
ros2 launch sentry_lidar stl19p.launch.py

# MIPI 摄像头
ros2 run sentry_bringup mipi_camera_node

# 云台键盘控制
ros2 run sentry_servo servo_keyboard_node
```

### 4.3 回放 bag

```bash
ros2 bag play records/critical_20250430_143022/
```

---

## 5. STM32 固件编译与烧录

### 5.1 生成工程

1. 使用 STM32CubeMX 打开 `firmware/` 下的 `.ioc` 文件
2. 检查时钟、UART、TIM、GPIO 配置
3. 生成代码到 `firmware/` 对应目录

### 5.2 编译

```bash
cd firmware/stm32f407_chassis
make -j$(nproc)
```

### 5.3 烧录

```bash
STM32_Programmer_CLI -c port=SWD -w build/chassis.bin 0x08000000 -v -rst
```

### 5.4 关键协议检查

烧录后通过串口验证：

- `TYPE=0x01` 传感器汇总帧：30 字节
- `TYPE=0x03` 底盘状态帧：25 字节（含 19 字节 payload）
- CRC16-CCITT 校验通过

---

## 6. 模型部署

### 6.1 病害分类模型（MobileNetV3）

已量化为 BPU `.bin`，位于 `models/quantization/`：

```
models/
├── quantization/
│   ├── tomato_mobilenetv3_output/
│   │   └── tomato_mobilenetv3_bayese_224x224_nv12.bin     # 7-class, int8, 5.0MB
│   ├── wheat_mobilenetv3_output/
│   │   └── wheat_mobilenetv3_bayese_224x224_nv12.bin      # 5-class, int8, 2.1MB
│   └── strawberry_mobilenetv3_output/
│       └── strawberry_mobilenetv3_bayese_224x224_rgb.bin   # 8-class, int16, 2.9MB
├── tomato_mobilenetv3.onnx
├── wheat_mobilenetv3.onnx
└── strawberry_mobilenetv3.onnx
```

### 6.2 植株检测模型（YOLOv8n）

```
models/
└── yolov8n_crop_weed_bayese_640x640_nv12.bin     # 2-class, int8, 3.7MB
```

| 属性 | 值 |
|------|-----|
| 任务 | Crop/Weed 二分类检测 |
| 输入 | NV12 640×640 |
| 输出 | 6 tensors (3 cls + 3 bbox, NHWC) |
| Cosine | ≥ 0.997 |
| mAP50 | 0.860 |
| 训练数据 | Roboflow Crop and Weed Detection (3071 张) |

### 6.3 量化方式

使用地平线 OpenExplore v1.2.8 工具链（Docker 容器 `oe_cpu`）：

```bash
# 病害分类
hb_mapper makertbin -c <crop>_config.yaml --model-type onnx

# YOLOv8 检测（需 BPU 友好 ONNX + mapper.py）
python3 export_monkey_patch.py --pt best.pt    # 导出 NHWC 6 输出 ONNX
python3 mapper.py --onnx best.onnx --cal-images ./calibration \
  --cal-sample-num 50 --optimize-level O3 --output-dir .
```

### 6.4 板端推理 API

RDK X5 OS 3.3.1 使用 `pyeasy_dnn`（非 `hbm_runtime`）：

```python
from hobot_dnn import pyeasy_dnn as dnn
import numpy as np

# 分类
m = dnn.load('models/quantization/tomato_mobilenetv3_output/tomato_mobilenetv3_bayese_224x224_nv12.bin')[0]
nv12 = np.zeros(224 * 224 * 3 // 2, dtype=np.uint8)
out = m.forward([nv12])

# 检测（6 输出，需板端 DFL + NMS 后处理）
m = dnn.load('models/yolov8n_crop_weed_bayese_640x640_nv12.bin')[0]
nv12 = np.zeros(640 * 640 * 3 // 2, dtype=np.uint8)
outputs = m.forward([nv12])
```

### 6.5 输入尺寸

- 分类模型：224×224
- 检测模型：640×640

---

## 7. RDK X5 板端工作流程

代码优先在本地开发、提交到远程仓库，再到 RDK X5 板端拉取测试：

```bash
# 本地
git add <files>
git commit -m "..."
git push origin main

# 板端
ssh sunrise@ubuntu.local
cd ~/dev_ws/src/smart-agri-sentry
git pull
cd ~/dev_ws
colcon build --packages-select ...
source install/setup.bash
ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato
```

---

## 8. 常用调试

### 8.1 检查话题

```bash
ros2 topic list
ros2 topic hz /vision/plant_detected
ros2 topic echo /fusion/diagnosis
```

### 8.2 检查 TF

```bash
ros2 run tf2_tools view_frames
```

### 8.3 检查串口

```bash
ls -l /dev/tty*
ls -l /dev/wheeltec_lidar
ls -l /dev/myimu
```

### 8.4 PWM 权限

```bash
groups sunrise  # 确认包含 gpio
```

### 8.5 底盘运动测试

```bash
# 编码器闭环：前进 0.3 m/s × 2 米
ros2 run sentry_mission chassis_cmd --forward 0.3 --dist 2.0

# IMU 闭环转弯：左转 90°
ros2 run sentry_mission imu_turn --angle 90
```

详细用法见 [`docs/ROS2.md`](ROS2.md) §8。

---

## 9. 参考文档

- 系统架构 → [`docs/ARCHITECTURE.md`](ARCHITECTURE.md)
- 硬件规格 → [`docs/HARDWARE.md`](HARDWARE.md)
- ROS2 接口 → [`docs/ROS2.md`](ROS2.md)
- 技术决策 → [`docs/DECISIONS.md`](DECISIONS.md)
- 任务与阻塞 → [`docs/TODO.md`](TODO.md)
- 已知问题 → [`docs/ISSUES.md`](ISSUES.md)
- 开发规范 → `CLAUDE.md`
