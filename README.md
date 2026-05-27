# 智农哨兵 · Smart Agri Sentry v2.0

> 基于 RDK X5 的番茄/小麦/草莓多作物病害巡检机器人，融合视觉推理、环境感知与农艺决策。

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-RDK%20X5-orange)](https://developer.d-robotics.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 系统架构

智农哨兵 v2.0 采用**三层解耦 + 事件驱动巡检**架构：

- **感知层**：视觉推理（病害分类 + 植株检测）、移动传感器（随车）、固定环境节点（24h 田间微气候）
- **决策层**：实时融合（LWD 滑动窗口）、趋势预测（简化外推）、农艺建议（YAML 规则引擎）
- **控制层**：巡检状态机（停-拍-判-走）、数据记录（7 天循环 + CRITICAL 永久保留）

```
┌─────────────────────────────────────────────────────────────┐
│ Perception Layer                                            │
│  ├─ plant_detector_node  → /vision/plant_detected          │
│  ├─ vision_diagnosis_node→ /vision/diagnosis               │
│  ├─ uart_bridge_node     → /sensor/environment_mobile      │
│  │                       → /sensor/soil_nutrition          │
│  └─ env_bridge_node      → /sensor/environment_fixed       │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Decision Layer                                              │
│  ├─ fusion_node      → /fusion/diagnosis  (2Hz, LWD+门控)  │
│  ├─ forecast_node    → /forecast/alert    (10min, 趋势外推)│
│  └─ advisory_node    → /advisory/action   (YAML规则引擎)   │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│ Control Layer                                               │
│  ├─ mission_control_node → /mission/status + cmd_vel       │
│  └─ data_logger_node     → ros2 bag (循环+CRITICAL保留)    │
└─────────────────────────────────────────────────────────────┘
```

### 核心特性

- **多作物支持**：番茄（10 类）、小麦（5 类）、草莓（8 类），动态切换
- **事件驱动巡检**：植株检测（YOLO-Nano）触发停车 → 病害分类 → 融合决策 → 恢复巡航
- **24h 叶面湿润时长（LWD）**：固定环境节点 5min 采样，288 点滑动窗口，支持冷启动降级
- **严格优先级门控**：VISION_DOMINANT → LATENT_SUSPICION → HIGH_HUMIDITY_PATHOGEN → DROUGHT_STRESS → BALANCED，消除模式抖动
- **双轨预测**：默认本地趋势外推，预留 SIR-like 微分方程与外部天气接口
- **结构化农艺建议**：YAML 规则引擎，毫秒级响应，比赛可解释

---

## 快速开始

### 硬件清单

| 模块 | 型号/方案 | 备注 |
|------|----------|------|
| 主控 | RDK X5 (ROS2 Humble) | 视觉推理 + 决策节点 |
| 摄像头 | MIPI CSI / USB | plant_detector + vision_diagnosis 共享 |
| 底盘 | STM32F4 + 差速轮 | UART 协议，100Hz 控制 + 1Hz 传感器 |
| 移动传感器 | 空气温湿/CO₂ + 土壤温湿/EC/NPK/pH | 随车，通过底盘串口回传 |
| 固定环境节点 | STM32L072 + SX1262 (LoRa) | 低功耗野外版，太阳能供电 |
| LoRa 网关 | ESP32-S3 + SX1262 | USB 转串口直连 RDK X5 |
| 固定节点传感器 | SHT30 + SCD40 + RS485 土壤 + LWS10 | 空气/土壤/叶面湿度 |

### 环境准备

```bash
# 1. 克隆仓库
cd ~/agri_ws/src
git clone https://github.com/wjun/smart-agri-sentry.git
cd ..

# 2. 安装依赖
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/smart_agri_sentry/requirements.txt

# 3. 编译
colcon build --packages-select sentry_interfaces sentry_bringup \
  sentry_vision sentry_fusion sentry_forecast sentry_advisory \
  sentry_mission sentry_sensors
source install/setup.bash
```

### 配置

```bash
# 作物配置
cp config/crop_profiles.yaml.example config/crop_profiles.yaml
# 编辑番茄/小麦/草莓的温度窗口、LWD阈值

# 农艺建议规则
cp config/advisory_rules.yaml.example config/advisory_rules.yaml
# 编辑作物-病害-条件-动作规则

# 巡检参数
cp config/mission_params.yaml.example config/mission_params.yaml
# 编辑巡航速度、植株检测阈值、停车距离等
```

### 启动

```bash
# 完整系统启动
ros2 launch sentry_bringup sentry_v2.launch.py crop_type:=tomato

# 单独调试融合节点
ros2 run sentry_fusion fusion_node --ros-args -p crop_type:=tomato

# 回放 bag 分析
ros2 bag play records/critical_20250430_143022/
```

---

## 项目结构

```
smart_agri_sentry/
├── README.md                     # 本文档
├── scheme1.md                    # v2.0 详细设计方案
├── config/
│   ├── crop_profiles.yaml        # 作物特异性参数（温度窗口、LWD阈值）
│   ├── advisory_rules.yaml       # 农艺建议规则库
│   └── mission_params.yaml       # 巡检状态机参数
├── models/
│   ├── tomatoes_mobilenetv2_int8.tflite   # 番茄病害分类
│   ├── wheat_mobilenetv2_int8.tflite      # 小麦病害分类
│   ├── strawberry_mobilenetv2_int8.tflite # 草莓病害分类
│   └── plant_detector_nano.tflite         # 植株检测
├── src/
│   ├── sentry_interfaces/        # ROS2 消息定义
│   │   ├── msg/Diagnosis.msg
│   │   ├── msg/PlantDetection.msg
│   │   ├── msg/Environment.msg
│   │   ├── msg/SoilNutrition.msg
│   │   ├── msg/FusionResult.msg
│   │   ├── msg/ForecastAlert.msg
│   │   ├── msg/AdvisoryAction.msg
│   │   └── msg/MissionStatus.msg
│   ├── sentry_bringup/           # Launch 文件与启动配置
│   ├── sentry_vision/            # 视觉感知
│   │   ├── plant_detector_node.py
│   │   └── vision_diagnosis_node.py
│   ├── sentry_fusion/            # 实时融合决策
│   │   ├── fusion_node.py
│   │   └── lwd_calculator.py
│   ├── sentry_forecast/          # 预测预警
│   │   └── forecast_node.py
│   ├── sentry_advisory/          # 农艺建议
│   │   ├── advisory_node.py
│   │   └── rule_engine.py
│   ├── sentry_mission/           # 巡检状态机
│   │   └── mission_control_node.py
│   ├── sentry_sensors/           # 传感器桥接与数据记录
│   │   ├── env_bridge_node.py
│   │   ├── uart_bridge_node.py
│   │   └── data_logger_node.py
│   └── sentry_hardware/          # 固定环境节点固件
│       └── fixed_env_node/
│           ├── stm32l072_lora/   # 低功耗野外版主程序
│           └── lora_gateway/     # USB 串口网关程序
├── firmware/                     # 下位机固件
├── docs/
│   ├── architecture/             # 架构与数据流文档
│   └── requirements/             # 需求与方案文档
└── tests/                        # 单元测试与离线验证
```

---

## 节点说明

### 感知层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `camera_node` | - | `/sentry/camera/image_raw` | 摄像头驱动 |
| `plant_detector_node` | `image_raw` | `/vision/plant_detected` | YOLO-Nano 植株检测，决定停车时机 |
| `vision_diagnosis_node` | `image_raw` | `/vision/diagnosis` | TFLite 作物-specific 病害分类 |
| `uart_bridge_node` | `/sentry/cmd_vel`, `/sentry/servo_cmd` | `/sensor/environment_mobile`, `/sensor/soil_nutrition`, `/sentry/chassis/status` | 底盘串口桥接，解析 STM32F4 协议 |
| `env_bridge_node` | LoRa 网关串口 | `/sensor/environment_fixed` | 固定环境节点数据，支持多点，Fusion 内取平均 |

### 决策层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `fusion_node` | `/vision/diagnosis`, `/sensor/environment_fixed`, `/sensor/environment_mobile` | `/fusion/diagnosis` | 实时融合：LWD 滑动窗口 + 优先级门控 + 证据链 |
| `forecast_node` | `/fusion/diagnosis` | `/forecast/alert` | 简化趋势外推（默认），预留 SIR-like 与天气接口 |
| `advisory_node` | `/fusion/diagnosis`, `/forecast/alert` | `/advisory/action` | YAML 规则引擎，事件触发 |

### 控制层

| 节点 | 订阅 | 发布 | 说明 |
|------|------|------|------|
| `mission_control_node` | `/vision/plant_detected`, `/fusion/diagnosis`, `/advisory/action`, `/sentry/chassis/status` | `/sentry/cmd_vel`, `/mission/status` | 停-拍-判-走状态机 |
| `data_logger_node` | 核心 topic | bag 文件 | 7 天循环录制，CRITICAL 事件永久保留 |

---

## 消息接口

### Diagnosis（视觉输出）

```yaml
std_msgs/Header header
string crop_type              # tomato / wheat / strawberry
string disease_class          # 如 "late_blight"
uint8 disease_class_id        # 模型原始 class_id
float32 confidence            # 最高类概率
float32[] probabilities       # 全类概率分布
```

### PlantDetection（植株检测）

```yaml
std_msgs/Header header
bool detected                 # 是否检测到植株
float32 confidence            # 检测置信度
float32[] bbox                # [x_min, y_min, x_max, y_max] 归一化
float32 area_ratio            # 叶片占画面比例
```

### Environment（统一环境）

```yaml
std_msgs/Header header
float32 temperature           # 空气温度 °C
float32 humidity              # 空气湿度 %RH
float32 soil_temperature      # 土壤温度 °C
float32 soil_humidity         # 土壤湿度 %
float32 soil_ec               # 土壤电导率 dS/m
float32 leaf_wetness          # 叶面湿度 %
float32 co2                   # CO₂ ppm
string data_source            # MOBILE / FIXED_NODE_01 / FIXED_NODE_02 / ...
```

### FusionResult（融合输出）

```yaml
std_msgs/Header header
string crop_type
string disease_class
uint8 disease_class_id
float32 risk                  # [0.0, 1.0]
float32 confidence            # [0.0, 1.0]
string alert                  # NORMAL / SUSPICION / WARNING / CRITICAL
string mode                   # 门控模式
string data_quality           # COLD_BOOT / WARM_UP / NORMAL
float32 p_vis                 # 视觉概率
float32 e_norm                # 当前环境危险度
float32 e_norm_history        # 24h 滑动平均
float32 lwd_hours             # 叶面湿润时长（-1 表示冷启动无效）
float32 interaction           # 交互项值
float32 trend_factor          # 湿度趋势修正系数
string[] evidence_chain       # 人类可读证据列表
```

### AdvisoryAction（农艺建议）

```yaml
std_msgs/Header header
string advisory_id            # 规则 ID，可追溯
string crop_type
string disease_class
string action_text            # 人类可读建议
uint32 urgency_hours          # 建议多少小时内执行
string[] prerequisites        # 执行前提条件
string fungicide_hint         # 推荐药剂
float32 cost_estimate         # 预估成本
```

---

## 核心算法

### 1. LWD 滑动窗口与冷启动

固定环境节点 5 分钟采样，维护 288 点（24h）滑动窗口：

| 阶段 | 时间 | LWD 策略 | LATENT_SUSPICION | 置信度 |
|------|------|---------|------------------|--------|
| COLD_BOOT | 0–30min | 回退瞬时湿度，上限 0.70 | 禁用 | ×0.75 |
| WARM_UP | 30min–24h | 短时 LWD 线性外推 | 条件放宽 | ×0.90 |
| NORMAL | ≥24h | 完整 24h 查表 | 正常触发 | ×1.0 |

作物特异性 LWD 阈值：

| 作物 | critical (≥h) | high (≥h) | moderate (≥h) | h_risk |
|------|--------------|-----------|---------------|--------|
| 番茄 | 6 | 4 | 2 | 0.95 / 0.80 / 0.55 |
| 小麦 | 4 | 3 | 1.5 | 0.95 / 0.80 / 0.55 |
| 草莓 | 8 | 5 | 3 | 0.95 / 0.80 / 0.55 |

### 2. 优先级门控

```python
def select_mode(P_vis, env, E_norm, h_risk, t_risk, env_history):
    # ① 视觉绝对主导（病斑已肉眼可见）
    if P_vis >= 0.80:
        return "VISION_DOMINANT"

    # ② 潜伏期预警（需 24h LWD，冷启动禁用）
    if not env_history.is_cold_boot():
        lwd = env_history.get_lwd_hours()
        if lwd >= LWD_THRESHOLD[crop] and P_vis <= 0.30 and t_risk >= 0.60:
            return "LATENT_SUSPICION"

    # ③ 高湿病原爆发（真菌典型场景）
    hum_threshold = 90 if env_history.is_cold_boot() else 80
    if env.humidity >= hum_threshold and 15 <= env.temperature <= 28 and P_vis >= 0.50:
        return "HIGH_HUMIDITY_PATHOGEN"

    # ④ 干旱胁迫（高温低湿）
    if env.humidity <= 40 and env.temperature >= 30:
        return "DROUGHT_STRESS"

    # ⑤ 兜底平衡模式
    return "BALANCED"
```

滞回缓冲带：
- `VISION_DOMINANT`：需 `P_vis` 掉到 0.75 以下才退出
- `LATENT_SUSPICION`：需湿度降到 80% 以下或 `P_vis` 涨到 0.35 以上才退出

### 3. 融合公式

```
interaction   = P_vis × E_norm
trend_factor  = 1.0 + 0.2 × max(0, humidity_trend_2h)

Risk = w_v·P_vis + w_e·E_norm·trend_factor + w_i·interaction + bias
Risk = clip(Risk, 0.0, 1.0)

agreement = 1.0 - |P_vis - E_norm|
base_confidence = 0.55 + 0.45 × agreement

# 冷启动置信度惩罚
if COLD_BOOT:   confidence = base_confidence × 0.75
elif WARM_UP:     confidence = base_confidence × 0.90
else:             confidence = base_confidence
```

报警分级：
- `CRITICAL`: Risk ≥ 0.80 且 confidence ≥ 0.80（冷启动最多降级为 WARNING）
- `WARNING`: Risk ≥ 0.60
- `SUSPICION`: mode == LATENT_SUSPICION 且 Risk ≥ 0.40
- `NORMAL`: 其余

---

## 病害支持列表

### 番茄（10 类）

| class_id | 英文名称 | 中文名称 |
|----------|---------|---------|
| 0 | Bacterial Spot | 细菌性斑点病 |
| 1 | Early Blight | 早疫病 |
| 2 | Healthy | 健康 |
| 3 | Late Blight | 晚疫病 |
| 4 | Leaf Mold | 叶霉病 |
| 5 | Septoria Leaf Spot | 壳针孢叶斑病（斑枯病） |
| 6 | Spider Mites (Two-spotted spider mite) | 蜘蛛螨（二斑叶螨） |
| 7 | Target Spot | 靶斑病 |
| 8 | Tomato Mosaic Virus | 番茄花叶病毒 |
| 9 | Tomato Yellow Leaf Curl Virus | 番茄黄化曲叶病毒 |

### 小麦（5 类）

| class_id | 英文名称 | 中文名称 |
|----------|---------|---------|
| 0 | Healthy | 健康 |
| 1 | Wheat Powdery Mildew | 小麦白粉病 |
| 2 | Wheat Scab | 小麦赤霉病 |
| 3 | Wheat Stripe Rust | 小麦条锈病 |
| 4 | Wheat Yellow Dwarf | 小麦黄矮病 |

### 草莓（8 类）

| class_id | 英文名称 | 中文名称 |
|----------|---------|---------|
| 0 | Leaf Spot | 叶斑病 |
| 1 | Powdery Mildew Leaf | 白粉病（叶片） |
| 2 | Gray Mold | 灰霉病 |
| 3 | Angular Leaf Spot | 角斑病 |
| 4 | Blossom Blight | 花腐病 |
| 5 | Powdery Mildew Fruit | 白粉病（果实） |
| 6 | Anthracnose Fruit Rot | 炭疽病（果实腐烂） |
| 7 | Healthy | 健康 |

---

## 硬件部署

### 固定环境节点（低功耗野外版）

```
[太阳能板 10W] ──┐
                 │
    ┌────────────┴──────────────┐
    │  IP65 防水盒              │
    │  ├── STM32L072 + SX1262   │
    │  ├── 18650×2 电池         │
    │  └── CN3791 MPPT 模块     │
    └────────────┬──────────────┘
                 │
    ┌────────────┴──────────────┐
    │  传感器探头               │
    │  ├── SHT30: 百叶箱内，冠层中部
    │  ├── SCD40: 同百叶箱 (CO₂)
    │  ├── LWS10: 叶片背面 (叶面湿度)
    │  └── RS485: 根区 10–15cm (土壤温湿+EC)
    └───────────────────────────┘
```

- 采样周期：5 分钟（深度睡眠唤醒）
- LoRa 发送：每小时汇总 12 条数据批量发送，或异常时立即上报
- 理论自持：> 1 年（10W 太阳能 + 4000mAh 电池）

### LoRa 网关

ESP32-S3 + SX1262 作为网关，USB 转串口直连 RDK X5。RDK X5 侧运行 `env_bridge_node` 解析串口数据并转为 ROS2 Topic。

---

## 数据存储策略

| 场景 | 方案 | 位置 |
|------|------|------|
| 实时录制 | `ros2 bag` 选择性录制核心 topic | RDK X5 SD 卡 |
| 循环策略 | 7 天自动覆盖 | RDK X5 SD 卡 |
| CRITICAL 事件 | 前后 5 分钟片段永久保留 | RDK X5 SD 卡 `records/critical/` |
| 离线分析 | `ros2 bag play` → InfluxDB + Grafana | 办公室 PC |

---

## 开发路线图

- [x] v1.0 基础架构：视觉推理 + 底盘控制 + 移动传感器
- [ ] **v2.0 综合决策系统**
  - [ ] Phase 1: 消息接口重构 + 植株检测 + 病害分类 + 融合 + 巡检状态机
  - [ ] Phase 2: 预测预警 + 农艺建议 + 数据记录
  - [ ] Phase 3: 固定环境节点硬件固件 + LoRa 网关对接
  - [ ] Phase 4: 外部天气源 + Web 前端 + InfluxDB 离线分析
- [ ] v3.0 端侧大模型增强：LLM 异步建议润色、自然语言交互

---

## 贡献

欢迎提交 Issue 和 PR。请遵循 `CLAUDE.md` 中的开发规范。

## License

MIT License. See [LICENSE](LICENSE) for details.
