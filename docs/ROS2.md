# ROS2 节点与接口

> 更新日期：2026-07-05

---

## 1. 节点图

```
                              ┌─────────────────┐
                              │   camera_node   │
                              └────────┬────────┘
                                       │ /sentry/camera/image_raw
                    ┌──────────────────┼──────────────────┐
                    ▼                  ▼                  ▼
         ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
         │ plant_detector_  │ │ vision_diagnosis_│ │  servo_driver_   │
         │     node         │ │     node         │ │     node         │
         └────────┬─────────┘ └────────┬─────────┘ └────────▲─────────┘
                  │ /vision/plant_     │ /vision/diagnosis   │ /sentry/servo_cmd
                  │    detected        │                     │
                  ▼                    ▼                     │
         ┌──────────────────┐ ┌──────────────────┐           │
         │  mission_control_  │ │   fusion_node    │           │
         │      node          │ └────────┬─────────┘           │
         └────────┬──────────┘          │ /fusion/diagnosis    │
                  │                      ▼                      │
         ┌────────┴──────────┐ ┌──────────────────┐            │
         │      Nav2         │ │ forecast_node    │            │
         │  (nav2_bringup)   │ │ advisory_node    │            │
         └────────┬──────────┘ └──────────────────┘            │
                  │ /cmd_vel                                  │
                  ▼                                             │
         ┌──────────────────┐                                  │
         │  uart_bridge_    │                                  │
         │      node        │                                  │
         └────────┬─────────┘                                  │
                  │ UART2                                      │
                  ▼                                            │
         ┌──────────────────┐                                  │
         │   STM32F407ZGT6  │◄─────────────────────────────────┘
         └──────────────────┘

  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐
  │ sentry_lidar│    │ wheel_odom_ │    │     ekf_filter          │
  │  → /scan    │    │    node     │    │  → /odom                │
  │  → /lidar/  │    │  → /wheel/  │    │                         │
  │  obstacle_  │    │     odom    │    │                         │
  │    info     │    │             │    │                         │
  └─────────────┘    └─────────────┘    └─────────────────────────┘

  ┌─────────────┐    ┌─────────────────────────┐
  │ lora_bridge_ │    │      imu_node           │
  │    node     │    │  → /sensor/imu/data_raw │
  │ → /sensor/  │    │  → /sensor/imu/data     │
  │ environment_│    │      (Madgwick)         │
  │    fixed    │    └─────────────────────────┘
  └─────────────┘

  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
  │  web_remote_    │    │ data_logger_    │    │  keyboard_      │
  │     node        │    │     node        │    │  control_node   │
  └─────────────────┘    └─────────────────┘    └─────────────────┘

  ┌──────────────────────────┐  ┌──────────────────────────┐
  │  chassis_cmd (CLI工具)    │  │  imu_turn (CLI工具)       │
  │  → /cmd_vel              │  │  → /cmd_vel              │
  │  ← /sentry/chassis/status│  │  ← /sensor/imu/data_raw  │
  └──────────────────────────┘  └──────────────────────────┘
```

---

## 2. 话题列表

### 2.1 感知层

| 话题名 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|
| `/sentry/camera/image_raw` | `sensor_msgs/Image` | `camera_node` | `plant_detector`, `vision_diagnosis` | 2 Hz | 摄像头原始图像 |
| `/vision/plant_detected` | `PlantDetection` | `plant_detector_node` | `mission_control` | 5 Hz | 植株检测结果（bbox + 置信度） |
| `/vision/diagnosis` | `Diagnosis` | `vision_diagnosis_node` | `fusion_node` | 2 Hz | 病害分类结果 |
| `/sensor/environment_mobile` | `Environment` | `uart_bridge_node` | `fusion_node` | 1 Hz | 移动传感器环境数据 |
| `/sensor/soil_nutrition` | `SoilNutrition` | `uart_bridge_node` | `data_logger` | 1 Hz | 土壤营养分离（N/P/K/pH/EC） |
| `/sensor/environment_fixed` | `Environment` | `lora_bridge_node` | `fusion_node` | 1/60 Hz | 固定 LoRa 环境节点数据（每 60 秒一帧） |
| `/scan` | `sensor_msgs/LaserScan` | `sentry_lidar` | Nav2/避障 | 10 Hz | 激光雷达点云 |
| `/lidar/obstacle_info` | `ObstacleInfo` | `sentry_lidar` | `fusion_node` | 10 Hz | 前方扇区障碍物简化信息 |
| `/sensor/imu/data_raw` | `sensor_msgs/Imu` | `imu_node` | Madgwick | 100 Hz | IMU 原始数据 |
| `/sensor/imu/data` | `sensor_msgs/Imu` | `imu_filter_madgwick` | EKF | 100 Hz | IMU 滤波后数据 |
| `/sentry/chassis/status` | `ChassisStatus` | `uart_bridge_node` | `mission_control`, `wheel_odom` | 10 Hz | 底盘状态：左右轮速(m/s)、电池电压(V)、报警位、累计脉冲(int32，可正可负)、编码器时间戳 |

### 2.2 决策层

| 话题名 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|
| `/fusion/diagnosis` | `FusionResult` | `fusion_node` | `forecast`, `advisory`, `mission_control`, `data_logger` | 事件 | 融合输出：risk + alert + mode + 证据链 |
| `/forecast/alert` | `ForecastAlert` | `forecast_node` | `advisory`, `data_logger` | 10 min | 预测预警 |
| `/advisory/action` | `AdvisoryAction` | `advisory_node` | `mission_control`, `data_logger` | 事件 | 农艺建议 |

### 2.3 控制层

| 话题名 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / `mission_control` / `web_remote` / `keyboard_control` | `uart_bridge_node` | 10–20 Hz | 统一底盘速度指令；uart_bridge_node 按轮距 0.23 m 转为左右轮 mm/s 下发 |
| `/sentry/servo_cmd` | `ServoCmd` | `servo_keyboard_node` / `mission_control`(未来) | `servo_driver_node` / `uart_bridge_node`（可选） | 事件 | 云台角度指令 |
| `/mission/status` | `MissionStatus` | `mission_control` | `data_logger` | 10 Hz | 巡检状态机状态 |
| `/wheel/odom` | `nav_msgs/Odometry` | `wheel_odom_node` | EKF, Nav2 | 20 Hz | 编码器里程计 |
| `/odom` | `nav_msgs/Odometry` | `ekf_filter` | Nav2, TF | 30 Hz | EKF 融合后的里程计 |
| `/resume_navigation` | `std_msgs/Bool` | (外部) | `mission_control` | 事件 | 恢复导航指令 |

### 2.4 服务

| 服务名 | 类型 | 服务端 | 说明 |
|---|---|---|---|
| `/set_auto_mode` | `std_srvs/SetBool` | `mission_control_node` | `true`=AUTO, `false`=MANUAL |

---

## 3. TF 树

```
odom
 └── base_link                          ← EKF (dynamic, publish_tf: true)
      ├── laser        z=0.0804m        ← URDF (robot_state_publisher)
      │   rpy=(0, 0, -90°) for native left-hand frame
      ├── imu_link     z=0.191m         ← URDF (robot_state_publisher)
      │   rpy=(180°, 0, -90°) for native left-hand frame
      └── camera_link                   ← 待实际测量后加入 URDF
```

- `odom → base_link`：由 `robot_localization` EKF 动态发布（融合 `/wheel/odom` + `/sensor/imu/data`）
- 所有静态 TF 统一由 `robot_state_publisher` 从 `sentry_bringup/urdf/sentry.urdf` 发布
- 实测位置：LiDAR=(80.4, 0, 282)mm，IMU=(27.5, 19.8, 191)mm（以底盘几何中心为原点，车头为 X 轴）
- Madgwick `publish_tf: false`，避免与 EKF 冲突
- `sentry_lidar` 和 `imu.launch.py` 不再各自发布静态 TF

---

## 4. 消息接口定义

### 4.1 Diagnosis（视觉输出）

```yaml
std_msgs/Header header
string crop_type              # tomato / wheat / strawberry
string disease_class          # 如 "late_blight"
uint8 disease_class_id        # 模型原始 class_id
float32 confidence            # 最高类概率
float32[] probabilities       # 全类概率分布
```

### 4.2 PlantDetection（植株检测）

```yaml
std_msgs/Header header
bool detected                 # 是否检测到植株
float32 confidence            # 检测置信度
float32[] bbox                # [x_min, y_min, x_max, y_max] 归一化
float32 area_ratio            # 叶片占画面比例
```

### 4.3 Environment（统一环境）

```yaml
std_msgs/Header header
float32 air_temp               # 空气温度 °C
float32 air_humidity           # 空气湿度 %RH
float32 air_co2                # CO₂ ppm
float32 soil_temp              # 土壤温度 °C
float32 soil_humidity          # 土壤湿度 %RH
float32 leaf_wetness           # 叶面湿度 %RH
float32 hcho                   # 甲醛 raw（CJ-702）
float32 tvoc                   # TVOC ppb（CJ-702）
float32 pm25                   # PM2.5 μg/m³（CJ-702）
float32 pm10                   # PM10 μg/m³（CJ-702）
float32 leaf_temp              # 叶面温度 °C
float32 ec                     # 土壤电导率
string data_source             # MOBILE / FIXED_LORA / FIXED_NODE_01 / ...
```

### 4.4 SoilNutrition（土壤营养）

```yaml
std_msgs/Header header
float32 nitrogen              # 氮 mg/kg
float32 phosphorus            # 磷 mg/kg
float32 potassium             # 钾 mg/kg
float32 ph                    # pH
float32 ec                    # 电导率 us/cm
```

### 4.5 FusionResult（融合输出）

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

### 4.6 ForecastAlert（预测预警）

```yaml
std_msgs/Header header
string crop_type
string disease_class
float32[] risk_24h            # 未来 24h 风险序列
string trend                  # RISING / STABLE / FALLING
float32 alert_level           # [0.0, 1.0]
```

### 4.7 AdvisoryAction（农艺建议）

```yaml
std_msgs/Header header
string advisory_id            # 规则 ID
string crop_type
string disease_class
string action_text            # 人类可读建议
uint32 urgency_hours          # 建议多少小时内执行
string[] prerequisites        # 执行前提条件
string fungicide_hint         # 推荐药剂
float32 cost_estimate         # 预估成本
```

### 4.8 MissionStatus（任务状态）

```yaml
std_msgs/Header header
string state                  # IDLE / PATROL / APPROACHING / STOPPED / ANALYZING / ACTION / RESUME / MANUAL / ESTOP
string crop_type
string last_diagnosis
float32 battery_voltage
bool auto_mode
```

### 4.9 ChassisStatus（底盘状态）

```yaml
std_msgs/Header header
float32 left_speed            # m/s
float32 right_speed           # m/s
float32 battery_voltage       # V
uint8 alarm_bits
int32 left_pulse              # 左轮编码器累计脉冲，可正可负
int32 right_pulse             # 右轮编码器累计脉冲，可正可负
uint32 encoder_timestamp      # STM32 时间戳 ms
```

### 4.10 ServoCmd（云台指令）

```yaml
std_msgs/Header header
float32 yaw_angle             # 0–180°
float32 pitch_angle           # 0–180°
```

### 4.11 ObstacleInfo（前方障碍物）

```yaml
std_msgs/Header header
float32 front_min_distance    # 前方扇区最近距离 m，无效时 NaN
float32 front_avg_distance    # 前方扇区平均距离 m，无效时 NaN
bool obstacle_detected        # 是否进入危险阈值
float32 danger_threshold      # 危险阈值 m
int32 front_point_count       # 前方扇区有效点数量
```

---

## 5. 病害支持列表

### 番茄（10 类）

| class_id | 英文名称 | 中文名称 |
|---|---|---|
| 0 | Bacterial Spot | 细菌性斑点病 |
| 1 | Early Blight | 早疫病 |
| 2 | Healthy | 健康 |
| 3 | Late Blight | 晚疫病 |
| 4 | Leaf Mold | 叶霉病 |
| 5 | Septoria Leaf Spot | 壳针孢叶斑病 |
| 6 | Spider Mites | 蜘蛛螨（二斑叶螨） |
| 7 | Target Spot | 靶斑病 |
| 8 | Tomato Mosaic Virus | 番茄花叶病毒 |
| 9 | Tomato Yellow Leaf Curl Virus | 番茄黄化曲叶病毒 |

### 小麦（5 类）

| class_id | 英文名称 | 中文名称 |
|---|---|---|
| 0 | Healthy | 健康 |
| 1 | Wheat Powdery Mildew | 小麦白粉病 |
| 2 | Wheat Scab | 小麦赤霉病 |
| 3 | Wheat Stripe Rust | 小麦条锈病 |
| 4 | Wheat Yellow Dwarf | 小麦黄矮病 |

### 草莓（8 类）

| class_id | 英文名称 | 中文名称 |
|---|---|---|
| 0 | Leaf Spot | 叶斑病 |
| 1 | Powdery Mildew Leaf | 白粉病（叶片） |
| 2 | Gray Mold | 灰霉病 |
| 3 | Angular Leaf Spot | 角斑病 |
| 4 | Blossom Blight | 花腐病 |
| 5 | Powdery Mildew Fruit | 白粉病（果实） |
| 6 | Anthracnose Fruit Rot | 炭疽病（果实腐烂） |
| 7 | Healthy | 健康 |

---

## 6. 参数配置文件

| 文件 | 用途 |
|---|---|
| `config/crop_profiles.yaml` | 作物特异性参数（温度窗口、LWD 阈值） |
| `config/advisory_rules.yaml` | 农艺建议规则库 |
| `config/mission_params.yaml` | 巡检参数（速度、阈值、停车距离） |
| `config/servo_config.yaml` | 云台 PWM 配置 |
| `src/sentry_lidar/config/stl19p.yaml` | LiDAR 参数 |
| `src/sentry_mission/config/nav2_no_map.yaml` | Nav2 无地图配置 |
| `src/sentry_mission/config/ekf.yaml` | EKF 配置 |
| `src/sentry_mission/config/waypoints.yaml` | 巡航航点 |
| `src/sentry_sensors/config/imu.yaml` | IMU/Madgwick 配置 |

---

## 7. 已移除接口

### GPS Topic（已弃用）

`/sentry/gps/fix`（`sensor_msgs/NavSatFix`）不再使用，已从话题表中移除。遗留节点仅存在于旧版启动文件 `sentry_bringup/launch/sentry.launch.py`。

---

## 8. 底盘控制 CLI 工具

`sentry_mission` 包提供两个独立的调试/测试工具，用于底盘运动验证。

### 8.1 chassis_cmd — 编码器闭环运动测试

基于编码器反馈的底盘直线/转弯/弧线运动，到达目标后自动停止。

```bash
ros2 run sentry_mission chassis_cmd --forward 0.3 --dist 2.0    # 前进 2 米
ros2 run sentry_mission chassis_cmd --turn-left 0.5 --angle 90  # 左转 90°
ros2 run sentry_mission chassis_cmd --turn-right 0.3 --angle 45 # 右转 45°
ros2 run sentry_mission chassis_cmd --stop                      # 急停
```

依赖：`uart_bridge_node` 正在运行以提供 `/sentry/chassis/status` 编码器反馈。

### 8.2 imu_turn — IMU 陀螺仪闭环原地转弯

利用陀螺仪 `angular_velocity.z` 梯形积分，实现与地面无关的高精度原地旋转。三阶段控制：TURN（PID @ 0.3 rad/s）→ BRAKE（反向推力 0.15s）→ LOCK（零速保持 2s），精度 ~4%。

```bash
ros2 run sentry_mission imu_turn --angle 90     # 左转 90°
ros2 run sentry_mission imu_turn --angle -45    # 右转 45°
ros2 run sentry_mission imu_turn --angle 180 --max-speed 0.5 --kp 0.8
ros2 run sentry_mission imu_turn --stop         # 急停
```

依赖：IMU 节点正在运行，启动时自动校准陀螺仪零偏（100 样本）。
