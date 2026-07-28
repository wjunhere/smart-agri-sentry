# ROS2 节点与接口

> 更新日期：2026-07-17

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
| `/sentry/camera/image_raw` | `sensor_msgs/Image` | `hikrobot_camera_node`（默认）/ `mipi_camera_node` | `plant_detector`, `vision_diagnosis` | 5 Hz | 摄像头原始图像，backend 由 launch 参数 `camera_backend` 选择 |
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
| `/sentry/servo_cmd` | `ServoCmd` | `servo_keyboard_node` / `mission_control`（换行自动翻转） | `servo_driver_node` / `uart_bridge_node`（可选） | 事件 | 云台角度指令 |
| `/mission/status` | `MissionStatus` | `mission_control` | `data_logger` | 10 Hz | 巡检状态机状态 |
| `/wheel/odom` | `nav_msgs/Odometry` | `wheel_odom_node` | EKF, Nav2 | 20 Hz | 编码器里程计 |
| `/odom` | `nav_msgs/Odometry` | `ekf_filter` | Nav2, TF | 30 Hz | EKF 融合后的里程计 |
| `/resume_navigation` | `std_msgs/Bool` | (外部) | `mission_control` | 事件 | 恢复导航指令 |

### 2.4 服务

| 服务名 | 类型 | 服务端 | 说明 |
|---|---|---|---|
| `/set_auto_mode` | `std_srvs/SetBool` | `mission_control_node` | `true`=AUTO, `false`=MANUAL |

---

### 2.5 Web and Stack HTTP API

`web_remote_node` serves the browser UI on port 5000 and keeps the operator-facing stack state. rosbridge listens on port 9090 for browser-side ROS subscriptions.

| Endpoint | Method | Purpose |
|---|---|---|
| `/status` | GET | Current mode, service readiness, stack readiness, and manual velocity state |
| `/stack/preheat` | POST | Run `start_robot_stack.sh`; keep MANUAL and do not start cruise |
| `/stack/start` | POST | Start stack if needed, then call `/set_auto_mode=true` |
| `/stack/stop` | POST | Publish zero velocity and run `stop_robot_stack.sh` |
| `/waypoints` | GET/POST | Read or write cruise waypoints; POST updates install and source YAML files |
| `/crop_type` | POST | Set the current crop type |
| `/api/messages` | GET | 巡航消息列表：批次（启动巡航→停止）+ 每株检测记录（快照 URL、病害类别、置信度、时间），附未读批次数 |
| `/api/messages/<batch_id>/<seq>/snapshot` | GET | 某条检测记录的快照 JPEG（已画检测框） |
| `/api/messages/read` | POST | 清除未读角标 |
| `/api/messages/clear` | POST | 一键清空全部批次与快照 |

`miniprogram_bridge_node`（FastAPI，端口 8765，微信小程序网关）同样暴露整栈编排端点，语义与上表一致（复用同一 `start_robot_stack.sh`/`stop_robot_stack.sh`，以 `SENTRY_PRESERVE_WEB=1` 保留控制面）：

| Endpoint | Method | Purpose |
|---|---|---|
| `/stack/status` | GET | 状态机状态（`idle/preheating/starting/cruising/stopping/error`）+ 进程级存活检测 |
| `/stack/preheat` | POST | 后台运行 `start_robot_stack.sh`，不切 AUTO；状态经 WS `stack_status` 推送 |
| `/stack/start` | POST | 幂等启动主栈并切 AUTO 巡航；状态经 WS 推送 |
| `/stack/stop` | POST | 后台运行 `stop_robot_stack.sh` |

开机自启：`scripts/rdk/install_autostart.sh` 安装 `sentry-bridge.service`（systemd），开机拉起 `miniprogram_bridge.launch.py`（bridge + web_remote + weather_node + llm_advisor_node）——仅常驻网关层，相机/Nav2 等工作节点由前端 `/stack/*` 按钮按需启停。

**本地开发模式（2026-07-23）**：Web 面板可在本地运行、像小程序一样连接小车。在 `src/sentry_mission/static_v2/` 起任意静态服务（如 `python -m http.server 8899`），打开 `http://localhost:8899/?car=<小车IP>`。`?car=` 参数会存入 localStorage（之后可省略）；rosbridge 与全部 REST 改指向 `<car>:9090` / `<car>:5000`，Flask 已带 CORS 头。板端托管（`http://<car>:5000/`）同源行为不变。注意 rosbridge :9090 随主栈启停，idle 状态下页面显示 ROS OFFLINE 但 REST 控制仍可用。

Field topics to watch during cruise/avoidance:

- `/mission/status`
- `/cmd_vel`
- `/odometry/filtered`
- `/wheel/odom`
- `/sentry/chassis/status`
- `/lidar/obstacle_info`
- `/plan`, `/received_global_plan`, `/lookahead_point`

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

---

## 9. 相机节点与软件自适应曝光

默认相机后端为 `hikrobot_camera_node`（海康 MV-CS016-10UC, USB3），launch 参数 `camera_backend=hikrobot|mipi` 切换。两种后端发布同一话题 `/sentry/camera/image_raw`，下游节点无感知。

### 9.1 为什么需要软件 AE

该型号硬件自动曝光失效（任何场景都收敛到 ~6ms），且 `AutoExposureTimeLimit` 等寄存器返回 `0x80000109` 不支持。因此节点内置纯 Python 闭环控制器（`sentry_bringup/auto_exposure.py`，无 ROS 依赖，可离线单测），按帧亮度反馈直接写曝光/增益寄存器。

### 9.2 控制策略（每帧评估，寄存器写入限频 0.4s）

1. **饱和保护**（绕过限频）：>2% 像素过曝 → 曝光 ×0.6
2. **运动钳制**（绕过限频）：曝光超过当前模式上限 → 立即钳制
3. **死区**：|目标/均值 − 1| < 5% 不动
4. **调亮**：先加曝光，到上限后加增益；**调暗**：先减增益，到下限后减曝光

运动模式由 `/wheel/odom` 速度判定（>0.05 m/s 为运动；<0.02 m/s 持续 1s 为静止；里程计超时/缺失按运动处理，偏安全）。

### 9.3 主要参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ae_enabled` | `true` | 软件 AE 总开关（逃生舱；`false` 时回退固定曝光） |
| `ae_target_luma` | `80.0` | 目标亮度均值（0–255） |
| `ae_exp_min_us` | `2000.0` | 曝光下限 µs |
| `ae_exp_max_moving_us` | `20000.0` | 运动（巡航）曝光上限，防拖影 |
| `ae_exp_max_still_us` | `100000.0` | 静止（停车拍照）曝光上限 |
| `ae_gain_min` / `ae_gain_max` | `0.0` / `12.0` | 增益范围 dB |
| `exposure_auto` / `gain_auto` | `false` | 硬件 AE/AGC（本型号失效，必须关） |
| `exposure_time_us` / `gain` | `20000.0` / `3.0` | 固定曝光初值（AE 关闭时生效） |
| `gamma` | `2.0` | 软件 gamma LUT 提亮暗部 |
| `enable_image_enhancement` | `true` | gamma LUT 开关 |

板端实测（2026-07-17）：暗场景静止收敛 mean→80.0（目标值），运动/静止切换按里程计迟滞工作，硬件回读与控制器状态一致（ExposureAuto=0, ExposureTime=20000, GainAuto=0, Gain≈12）。


---

## 10. 视觉栈与前端开关接口（2026-07-28）

### HTTP 端点（web_remote_node :5000）

| 端点 | 方法 | 说明 |
|---|---|---|
| `/vision/start` | POST | 清理残留相机进程后启动相机栈（mipi_camera + republish + rosbridge），验证 `/out/compressed` 有帧才返回 ok |
| `/vision/stop` | POST | 停止相机 + 转发（rosbridge 保留） |
| `/inference/start` | POST | 清理残留推理节点后启动 `plant_detector_node` + `vision_pipeline_node`（YOLO+MobileNet） |
| `/inference/stop` | POST | 停止推理节点 |
| `/status` | GET | 新增 `camera_running` / `inference_running`（实时读 ROS 图，非缓存标志） |

启动/停止脚本：`scripts/rdk/start_camera_stack.sh`、`start_inference_stack.sh`、`stop_camera_stack.sh`、`stop_inference_stack.sh`（均先杀后启，防重复节点抢 MIPI 设备/BPU）。

### plant_detector_node 新参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `confidence_threshold` | 0.5（启动脚本传 0.35） | YOLO 置信度阈值 |
| `vote_window` / `vote_min` | 3 / 2 | 时序投票：近 N 帧至少 M 帧出框才上报 `detected=true`，单帧漏检沿用上一帧的框；`vote_window<=1` 关闭 |
| `model_path` | 见启动脚本 | 现役 `best_plant_11s_bayese_640x640_nv12.bin`（单类 yolo11s） |

### mipi_camera_node 去畸变参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `enable_undistort` | false | 开启 initUndistortRectifyMap + remap 去畸变 |
| `undistort_calib_file` | — | 标定 YAML（`config/imx477_640x480.yaml`，k3 模型 RMS 0.200px） |
| `undistort_alpha` | 0.0 | 0=裁切无黑边，1=保留全部视野 |
| `fps` | 10（监测脚本 15） | 15fps 时建议关闭低光增强/锐化（见 `start_camera_stack.sh`） |

### yolo_utils 后处理（性能关键）

- 类别通道数动态化：`cls.reshape(h, w, -1)`，兼容 2 类 crop/weed 与单类 plant 模型。
- **置信度优先 DFL 解码**：先以 `scores > conf` 过滤锚点，仅对存活锚点做 DFL softmax 解码（8400 → 通常 <100），后处理 ~65ms → <5ms，检测帧率 13 → 15fps 满帧，检测节点 CPU 117% → 65%。

### static_v2 前端

- 顶栏相机/推理按钮为开关式（toggle），文字反映 ROS 图真实状态（3s 轮询 `/status`，启动/停止进行中不同步防打架）。
- 视频帧渲染：requestAnimationFrame 只画最新帧，修复 rosbridge 积帧导致的"快速回放"。
