# 已知问题、硬件限制与规避方案

> 更新日期：2026-07-19

---

## 1. 硬件限制

| 限制 | 影响 | 当前方案 |
|---|---|---|
| **无机械加工条件** | 无法定制支架、外壳 | 全部采购成品底盘、云台、防水盒 |
| **原型机续航未优化** | 电池容量、功耗未达极限 | 比赛演示场景优先，续航优化后置 |
| **无 RTK 定位** | 绝对定位精度受限 | 已移除 GPS；当前采用 odom + IMU 融合，目标迁移到 LiDAR SLAM |
| **TB6612FNG 持续电流 1.2 A** | 可能无法驱动 24 V 减速电机 | 先用小功率测试，必要时更换 BTN7971B |
| **MIPI 摄像头 ISP 限制** | 第一个输出通道分辨率不能等于 sensor 原始分辨率 | ✅ 已解决：`out_w=[512, target]`, `out_h=[512, target]`；`get_img(type=2, w, h)` 中 type=2=NV12，w×h 匹配通道；NV12 stride 根据 `actual_size/(h*1.5)` 自动检测 |
| **CH340 USB 串口 ARM 驱动** | RDK X5 上 `in_waiting` 报告有数据但 `read()` 返回 0，导致 YbImuSerial 读线程崩溃 | ✅ 已解决：`imu_node.py` monkey-patch `read_all` 加容错 + 固定大小读取 fallback |
| **LoRa 野外丢包/延迟** | 固定环境节点数据可能不完整 | 协议层加 seq 序号和 ACK 重传，非关键数据允许丢包 |
| **海康 MV-CS016-10UC 硬件自动曝光失效** | 任何场景曝光收敛到 ~6ms；`AutoExposureTimeLimit` 寄存器返回 `0x80000109` 不支持 | ✅ 已解决：节点内置软件闭环 AE（`auto_exposure.py`），帧亮度反馈写寄存器，运动/静止分档曝光上限（2026-07-17） |

---

## 2. 软件风险

| 风险 | 影响 | 缓解措施 |
|---|---|---|
| **NPU 推理延迟 > 500 ms** | 影响巡检节拍 | 已量化 int8；必要时降低输入分辨率 |
| **Madgwick/EKF TF 冲突** | TF 树异常 | Madgwick `publish_tf: false`，EKF 单独发布 `odom → base_link` |
| **`rosbag2_py` 在 RDK X5 不可用** | `data_logger_node` 无法录制 | 已实现 JSON fallback |
| **`YbImuLib` CH340 ARM 驱动 bug** | IMU 读线程崩溃，无法发布 `/sensor/imu/data` | ✅ 已解决：`imu_node.py` `_patch_ch340_read()` monkey-patch 容错（2026-07-08） |
| **mapless Nav2 里程计漂移** | 长距离航点偏离 | 已决策迁移到 LiDAR SLAM；当前用 odom 帧短距离航点 |
| **`/cmd_vel` 多发布者冲突** | Nav2、mission_control、keyboard_control 同时发 `/cmd_vel` | ✅ 已修复：keyboard_control 仅 MANUAL 模式发布；mission_control RESUME 不发 cruise_speed（2026-07-08） |
| **EKF 频率过高** | RDK X5 跑不动 30Hz EKF | ✅ 已修复：降至 10Hz，偶发 1-2 次 / 30s（2026-07-08） |
| **data_logger YAML 格式** | ROS2 参数不支持顶层列表 | ✅ 已修复：加 `ros__parameters` 包装层（2026-07-08） |
| **bridge 订阅话题名错误** | 订阅 `/sentry/sensor/*`，uart_bridge 实际发 `/sensor/*`，小程序环境数据全断 | ✅ 已修复：改为 `/sensor/*`（2026-07-19, PR #3） |
| **小程序 WS 通道未接线** | `wsConnect()` 定义但无调用点，实时推送全断、只剩 REST 轮询 | ✅ 已修复：`app.ts onLaunch` 接线 + 断线重连（2026-07-19, PR #3） |

---

## 3. 常见问题与规避方案

### 3.1 MIPI 摄像头

| 现象 | 原因 | 解决 |
|---|---|---|
| `vp_isp_init failed, ret(-10)` | `open_cam` 第一个通道分辨率太大 | 改为 `[512, 1920]` / `[512, 1080]` |
| `hbn_vflow_stop failed, ret(-11)` | `close_cam()` 被重复调用 | `destroy_node()` 中加 `self.cam = None` 防重入 |
| `RuntimeError: Context must be initialized...` | `rclpy.shutdown()` 重复执行 | 删除自定义 signal handler，`finally` 中加 `if rclpy.ok():` |
| 画面条纹/花屏 | NV12 按错误分辨率解析 | 根据 `len(img_buf)` 实际大小判断真实 stride |
| `get_img(3)` 报 "module not supported" | RDK API 中 type=3 不存在 | type=2 固定 NV12，通道由传入 w×h 匹配 `out_w/out_h` 列表决定 |
| `get_img(0)` 返回全绿画面 | type=0 是 raw Bayer 非 NV12 | 改用 `get_img(2, target_w, target_h)` |
| 前端视频黑屏 | `image_transport republish raw` 只发 `/out`，前端订阅 `/out/compressed` | 改为 `arguments=['raw', 'compressed']` |
| NV12 size mismatch: actual=4147200 expected=3110400 | ISP stride=2560 ≠ width=1920 | `_nv12_to_bgr` 增加自动检测：`stride = actual_size / (height * 1.5)` |

### 3.2 PWM 舵机

| 现象 | 原因 | 解决 |
|---|---|---|
| 无 PWM 输出 | 32/33 脚仍为 I2C 功能 | 启用 PWM overlay：`srpi-config` → Interface Options → PWM |
| Permission denied | 用户不在 `gpio` 组 | `sudo usermod -aG gpio sunrise` 后重新登录 |
| STM32 与 RDK 同时驱动冲突 | `forward_servo_cmd=True` | 默认设为 `False`，RDK X5 直接驱动 |

### 3.3 STM32 通信

| 现象 | 原因 | 解决 |
|---|---|---|
| RDK 收不到传感器数据 | USART 接收使能位未设置 | 检查 `Init.Mode` 是否包含 `UART_MODE_RX` |
| CRC 校验失败 | 字节序或初始值错误 | CRC16-CCITT，初始值 `0xFFFF`，范围从 TYPE 到 payload 末尾 |
| 编码器脉冲不递增 | 定时器正交编码器模式未配置 | 检查 TIM 编码器接口与 GPIO 复用 |

---


### 3.4 Frontend and Cruise Startup

| Symptom | Likely cause | Mitigation |
|---|---|---|
| Browser cannot open `:5000` | `web_remote_node` is not running or not listening on 5000 | SSH to RDK, run `ss -ltnp`, then use `./scripts/rdk/start_robot_stack.sh` or start `web_remote_node` manually |
| ROS websocket keeps reconnecting | `rosbridge_websocket` is not running on 9090 | Start through the stack script; if preserving frontend, the start script launches rosbridge when needed |
| Start Cruise button does nothing | Stack not preheated, `/set_auto_mode` not ready, or stale ROS process conflict | Use Preheat first; if it still fails, run `./scripts/rdk/stop_robot_stack.sh` and preheat again |
| Robot turns immediately at startup | Obstacle is considered blocking the active waypoint, or stale costmap/process state exists | Clear stale stack and verify obstacle is not between robot and current waypoint |
| Avoidance re-triggers after rejoin | Robot is still close to the obstacle or side crops are briefly seen | `avoidance_retrigger_suppression_sec=2.5` suppresses the short handoff window; internal hard thresholds still protect the robot |

## 4. 已弃用 / 遗留代码

### GPS 模块

- **状态**：项目不再使用 GPS 模块。
- **遗留代码**：`sentry_bringup/launch/sentry.launch.py` 中仍包含 `gps_node` 节点。
- **影响**：旧版启动文件不可用于当前硬件；`sentry_v2.launch.py` 已移除 GPS。
- **建议**：在清理旧版 launch 文件时彻底删除 `gps_node` 及相关参数。

---

## 5. 待确认事项

- [ ] 24 V 减速电机额定电流和堵转电流
- [ ] 24 V→5 V 大功率 DC-DC 降压模块（建议 ≥5 A）是否已采购
- [ ] 比赛规则是否强制要求机械臂/土壤采样动作
- [ ] 固定环境节点外壳尺寸与太阳能板安装方式
- [ ] LoRa 实际通信距离与频段选择
