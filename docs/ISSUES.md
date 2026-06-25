# 已知问题、硬件限制与规避方案

> 更新日期：2026-06-25

---

## 1. 硬件限制

| 限制 | 影响 | 当前方案 |
|---|---|---|
| **无机械加工条件** | 无法定制支架、外壳 | 全部采购成品底盘、云台、防水盒 |
| **原型机续航未优化** | 电池容量、功耗未达极限 | 比赛演示场景优先，续航优化后置 |
| **无 RTK 定位** | 绝对定位精度受限 | 已移除 GPS；当前采用 odom + IMU 融合，目标迁移到 LiDAR SLAM |
| **TB6612FNG 持续电流 1.2 A** | 可能无法驱动 24 V 减速电机 | 先用小功率测试，必要时更换 BTN7971B |
| **MIPI 摄像头 ISP 限制** | 第一个输出通道分辨率不能等于 sensor 原始分辨率 | 512×512 作为第一通道，1920×1080 作为第二通道 |
| **LoRa 野外丢包/延迟** | 固定环境节点数据可能不完整 | 协议层加 seq 序号和 ACK 重传，非关键数据允许丢包 |

---

## 2. 软件风险

| 风险 | 影响 | 缓解措施 |
|---|---|---|
| **NPU 推理延迟 > 500 ms** | 影响巡检节拍 | 已量化 int8；必要时降低输入分辨率 |
| **Madgwick/EKF TF 冲突** | TF 树异常 | Madgwick `publish_tf: false`，EKF 单独发布 `odom → base_link` |
| **`rosbag2_py` 在 RDK X5 不可用** | `data_logger_node` 无法录制 | 已实现 JSON fallback |
| **`YbImuLib` 依赖未验证** | IMU 节点无法启动 | 先板端验证依赖，必要时改用标准驱动 |
| **mapless Nav2 里程计漂移** | 长距离航点偏离 | 已决策迁移到 LiDAR SLAM；当前用 odom 帧短距离航点 |
| **多作物模型不完整** | 小麦/草莓无法识别 | 框架已通用化，模型后续补全；v2.0 优先番茄端到端跑通 |

---

## 3. 常见问题与规避方案

### 3.1 MIPI 摄像头

| 现象 | 原因 | 解决 |
|---|---|---|
| `vp_isp_init failed, ret(-10)` | `open_cam` 第一个通道分辨率太大 | 改为 `[512, 1920]` / `[512, 1080]` |
| `hbn_vflow_stop failed, ret(-11)` | `close_cam()` 被重复调用 | `destroy_node()` 中加 `self.cam = None` 防重入 |
| `RuntimeError: Context must be initialized...` | `rclpy.shutdown()` 重复执行 | 删除自定义 signal handler，`finally` 中加 `if rclpy.ok():` |
| 画面条纹/花屏 | NV12 按错误分辨率解析 | 根据 `len(img_buf)` 实际大小判断真实分辨率 |
| 节点启动失败，sensor 已识别 | 上次崩溃未释放 MIPI | `sudo reboot` 后再试 |

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
