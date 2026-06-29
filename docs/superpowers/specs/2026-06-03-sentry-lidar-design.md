# sentry_lidar 雷达节点设计文档

**日期**: 2026-06-03  
**作者**: Claude Code  
**状态**: 已批准，待实现  

---

## 1. 背景与目标

将 `example/lidar/ldlidar_ros2/` 中的 LDLiDAR ROS2 驱动集成到 `src/` 中，作为正式的 `sentry_lidar` 功能包。适配硬件为 **STL19P 雷达 + CP2102 电平转换芯片**，运行环境为 **Ubuntu 22.04 + ROS2 Humble**。

### 成功标准
- `colcon build` 能编译通过
- `ros2 launch sentry_lidar stl19p.launch.py` 能独立启动并发布 `/scan` 和 `/lidar/obstacle_info`
- `ros2 launch sentry_bringup sentry_bringup.launch.py` 能一键拉起雷达节点
- RDK 板端插入 CP2102 后自动识别为 `/dev/wheeltec_lidar`

---

## 2. 架构设计

`sentry_lidar` 作为 `src/` 下独立的 C++ ROS2 包：

- **硬件对接层**: 通过串口与 STL19P 通信（波特率 230400）
- **协议解析层**: 解析 LDLiDAR 私有协议（`0x54` 头部定长包）
- **预处理层**: 计算前方扇区障碍物信息
- **输出层**: 同时发布标准 `LaserScan` 和自定义 `ObstacleInfo`

### 数据流

```
STL19P 雷达 --(UART/CP2102)--> serial_interface_linux --> lipkg(协议解析)
                                                                   |
                                    +------------------------------+------------------------------+
                                    |                                                             |
                                    v                                                             v
                           sensor_msgs/LaserScan                                    ObstacleInfo
                                 topic: /scan                                      topic: /lidar/obstacle_info
                                    |                                                             |
                              Nav2 / 避障模块                                          sentry_fusion
                                                                                     (风险评估)
```

---

## 3. 包结构与文件布局

```
src/
├── sentry_interfaces/
│   └── msg/
│       └── ObstacleInfo.msg              # 新增：障碍物简讯消息
├── sentry_lidar/                         # 新增：雷达驱动包（C++，ament_cmake）
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── stl19p.yaml                   # STL19P 参数配置
│   ├── launch/
│   │   ├── stl19p.launch.py              # 独立启动
│   │   └── viewer_stl19p.launch.py       # 带 rviz2 调试用
│   ├── udev/
│   │   └── 99-cp2102-lidar.rules         # CP2102 udev 规则
│   ├── include/sentry_lidar/
│   │   ├── ros2_api.h
│   │   ├── ldlidar_driver.h
│   │   ├── ldlidar_datatype.h
│   │   ├── lipkg.h
│   │   ├── serial_interface_linux.h
│   │   ├── tofbf.h
│   │   ├── log_module.h
│   │   └── obstacle_processor.h          # 新增
│   └── src/
│       ├── main.cpp                      # ROS2 节点入口（重构）
│       ├── ldlidar_driver.cpp            # 从 example 迁移
│       ├── lipkg.cpp                     # 从 example 迁移
│       ├── serial_interface_linux.cpp    # 从 example 迁移
│       ├── tofbf.cpp                     # 从 example 迁移
│       ├── log_module.cpp                # 从 example 迁移
│       └── obstacle_processor.cpp        # 新增：前方扇区预处理
└── sentry_bringup/
    └── launch/
        └── sentry_bringup.launch.py      # 修改：集成雷达启动
```

---

## 4. 接口定义

### 4.1 新增消息：`sentry_interfaces/msg/ObstacleInfo.msg`

```yaml
std_msgs/Header header
float32 front_min_distance      # 前方扇区(默认+-30deg)最近距离(m)，无效时NaN
float32 front_avg_distance      # 前方扇区平均距离(m)，无效时NaN
bool    obstacle_detected       # 是否有障碍物进入危险距离阈值内
float32 danger_threshold        # 配置的危险距离阈值(m)
int32   front_point_count       # 前方扇区内有效点数量
```

### 4.2 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 标准激光雷达点云，供导航/避障使用 |
| `/lidar/obstacle_info` | `sentry_interfaces/ObstacleInfo` | 前方扇区简化信息，供融合节点使用 |

### 4.3 TF

发布 `base_link` -> `laser` 的静态 TF，z=0.18m（可按实际安装调整）。

---

## 5. STL19P + CP2102 适配要点

| 项 | 值 | 说明 |
|---|---|---|
| `product_name` | `LDLiDAR_LD19` | 型号映射到 `LDType::LD_19` |
| `port_baudrate` | `230400` | 与 example 一致 |
| `port_name` | `/dev/wheeltec_lidar` | udev 别名 |
| `frame_id` | `laser` | LaserScan 的坐标系 |
| `measure_point_freq` | `4500` | LD_19 默认采样率 |
| udev 规则 | `99-cp2102-lidar.rules` | 匹配 `idVendor=10c4, idProduct=ea60` |

---

## 6. 关键参数（`config/stl19p.yaml`）

```yaml
sentry_lidar:
  ros__parameters:
    product_name: "LDLiDAR_LD19"
    port_name: "/dev/wheeltec_lidar"
    port_baudrate: 230400
    frame_id: "laser"
    laser_scan_dir: true
    enable_angle_crop_func: false
    angle_crop_min: 135.0
    angle_crop_max: 225.0
    min_range: 0.3
    max_range: 20.0
    front_sector_half_angle: 30.0
    danger_threshold: 0.5
    enable_filter: false
```

---

## 7. 启动集成

### 7.1 独立启动（调试）

```bash
ros2 launch sentry_lidar stl19p.launch.py
```

### 7.2 一键 bringup（部署）

```bash
ros2 launch sentry_bringup sentry_bringup.launch.py
```

`sentry_bringup.launch.py` 通过 `IncludeLaunchDescription` 引入 `stl19p.launch.py`，保持解耦。

### 7.3 可视化调试

```bash
ros2 launch sentry_lidar viewer_stl19p.launch.py
```

自动启动 rviz2，加载 `laser` 坐标系和 `/scan` 话题。

---

## 8. 实现注意事项

### 8.1 代码来源

驱动核心代码（`ldlidar_driver.cpp`、`lipkg.cpp`、`serial_interface_linux.cpp`、`tofbf.cpp`、`log_module.cpp`）直接迁移自 `example/lidar/ldlidar_ros2/ldlidar_driver/`，保持协议解析逻辑不变。

### 8.2 重构点

- `main.cpp`：从 example 的单文件节点重构为支持 `ObstacleInfo` 发布的节点，加入 `ObstacleProcessor` 调用
- 新增 `obstacle_processor.cpp/h`：从 `Points2D` 中提取前方扇区信息，计算 `front_min_distance`、`front_avg_distance`、`obstacle_detected`
- `CMakeLists.txt`：独立包配置，依赖 `rclcpp`、`sensor_msgs`、`tf2_ros`、`sentry_interfaces`

### 8.3 雷达参数调整

本驱动为**单向接收**设计，不支持通过 ROS 节点调整雷达硬件参数（转速、采样率等）。所有可调项均为节点端处理参数（见第 6 节）。

---

## 9. 测试计划

1. **编译测试**: `colcon build --packages-select sentry_lidar sentry_interfaces`
2. **独立启动测试**: 确认 `/scan` 和 `/lidar/obstacle_info` 均有数据
3. ** bringup 集成测试**: 确认一键启动后雷达节点正常加入
4. **rviz2 可视化**: 确认点云显示正常，TF 树正确
5. **CP2102 热插拔测试**: 拔插后 `/dev/wheeltec_lidar` 自动出现

---

## 10. 决策记录

| 决策 | 选项 | 理由 |
|------|------|------|
| 包形式 | 独立 C++ 包 `sentry_lidar` | 驱动为 C++，保持独立最干净 |
| 输出设计 | `/scan` + `/lidar/obstacle_info` | 原始点云给导航，简化信息给融合 |
| 预处理位置 | 雷达节点内 | 减轻 fusion 负担，降低耦合 |
| 启动方式 | 独立 launch + bringup 集成 | 既方便调试，也支持一键部署 |
| 硬件适配 | 同时迁移代码 + 修正配置 | udev 和参数是正常工作的前提 |
| 雷达参数 | 不支持硬件参数调整 | 协议为单向接收，无配置指令通道 |
