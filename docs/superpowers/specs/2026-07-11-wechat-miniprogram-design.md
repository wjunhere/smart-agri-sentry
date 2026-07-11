# 微信小程序 · 智农哨兵设计文档

> 版本 v1.0 · 2026-07-11
> 状态：设计确认，待实现

---

## 1. 目标

为智农哨兵 RDK X5 机器人开发微信小程序遥控终端，取代现有 `static_v2` Web 前端在小屏移动端的体验短板，提供视频流、病害识别、趋势预测、天气、手动操控、传感器数据、自动巡航控制等一站式功能。

---

## 2. 决策总览

| 决策 | 选择 | 理由 |
|------|------|------|
| 部署场景 | 局域网优先，架构预留云扩展 | 比赛原型为主，后续可加云中继 |
| 开发框架 | 原生 TS + Less + Skyline | 性能最佳，脚手架已就绪，无编译层 |
| 导航结构 | 底部 4 Tab（控制/监测/分析/天气） | 功能归拢清晰，符合微信限制 2-5 Tab |
| 通信方式 | WebSocket 实时 + HTTP 低频 | 视频/传感器/状态走 WS，天气/预测走 HTTP |
| 后端桥接 | 新建独立 `miniprogram_bridge_node` | 职责单一，不碰现有 web_remote_node |
| 设计风格 | Grafana 深色工业风 | 与现有 `static_v2` 视觉一致 |

---

## 3. 系统架构

```
┌─────────────────────────┐      ┌──────────────────────────────────────┐
│   微信小程序 (手机)       │      │  RDK X5 (机器人)                      │
│                          │      │                                      │
│  ┌────────────────────┐ │      │  ┌──────────────────────────────┐    │
│  │  4 Tab 页面         │ │      │  │  miniprogram_bridge_node     │    │
│  │  控制/监测/分析/天气 │ │      │  │  (新增, FastAPI + websockets) │    │
│  └────────┬───────────┘ │      │  │  :8765                        │    │
│           │              │      │  └────┬──────────┬──────────────┘    │
│    ┌──────┴──────┐       │      │       │          │                   │
│    │ ws (实时)    │       │      │  subscribe   REST API              │
│    │ + HTTP (低频)│       │      │  ROS2 topics (mode/control/...)     │
│    └──────┬──────┘       │      │       │          │                   │
│           │              │      │  ┌────▼──────────▼──────────┐        │
│   WiFi / 局域网          │      │  │  ROS2 话题/服务/参数       │        │
│           │              │      │  │  + rosbridge :9090        │        │
│           └──────────────┼──────┼──┤  + web_remote_node :5000  │        │
│                          │      │  └──────────────────────────┘        │
└─────────────────────────┘      └──────────────────────────────────────┘
```

- **新增节点** `miniprogram_bridge_node`：FastAPI + Python websockets，监听 `:8765`
- **现有节点不变**：`web_remote_node :5000`、`rosbridge :9090` 继续服务于 Web 前端
- **云扩展预留**：后续可在云服务器部署同接口中继，RDK X5 主动连接云端 WebSocket

---

## 4. 小程序结构

```
wechat/miniprogram/
├── app.ts / app.json / app.less          # 全局 + Tab 导航配置
├── pages/
│   ├── control/                          # Tab 1: 控制
│   │   ├── control.ts / .wxml / .less
│   │   └── 局部组件: dpad, mode-switch, cruise-status, crop-selector
│   ├── monitor/                          # Tab 2: 监测
│   │   ├── monitor.ts / .wxml / .less
│   │   └── 局部组件: camera-view, sensor-card, env-grid
│   ├── analysis/                         # Tab 3: 分析
│   │   ├── analysis.ts / .wxml / .less
│   │   └── 局部组件: diagnosis-result, prob-bars, advisory-card, forecast-chart
│   └── weather/                          # Tab 4: 天气
│       ├── weather.ts / .wxml / .less
│       └── 局部组件: weather-now, day-forecast, hour-forecast, disaster-alert
├── services/
│   ├── ws.ts                             # WebSocket 连接管理
│   ├── api.ts                            # HTTP 请求封装 (wx.request)
│   └── store.ts                          # 全局响应式状态
├── components/                           # 跨页面共享组件
│   ├── status-badge/                     # 模式/连接状态胶囊
│   ├── data-value/                       # 数值展示 (带单位、颜色)
│   └── alert-bar/                        # 预警条
└── utils/
    ├── format.ts                         # 单位格式化
    └── ros-parser.ts                     # 后端 JSON 消息解析
```

### 4.1 导航配置

`app.json` 使用 `tabBar` 配置 4 个底部 Tab：

| Tab | 页面路径 | 文字 | 图标 |
|-----|---------|------|------|
| 控制 | `pages/control/control` | 控制 | 🎮 |
| 监测 | `pages/monitor/monitor` | 监测 | 📷 |
| 分析 | `pages/analysis/analysis` | 分析 | 🧠 |
| 天气 | `pages/weather/weather` | 天气 | 🌤️ |

### 4.2 视觉风格

与现有 `static_v2/style.css` 完全一致：

- **背景**: `#0B1120` (最深) / `#0F172A` (表面) / `#111827` (卡片)
- **边框**: `#1F2937` (默认) / `#374151` (高亮)
- **文字**: `#F8FAFC` (主) / `#94A3B8` (次) / `#64748B` (禁用)
- **状态色**: 绿 `#10B981` / 琥珀 `#F59E0B` / 红 `#EF4444` / 蓝 `#38BDF8` / 紫 `#A78BFA`
- **字体**: 中文 Inter/PingFang SC，数据 JetBrains Mono/Cascadia Code
- **圆角**: `4px` (卡片/按钮) / `6px` (胶囊)

### 4.3 页面布局详情

#### 控制页 (Tab 1)
- 模式状态胶囊 (AUTO=绿 / MANUAL=蓝)
- 圆形 D-Pad 方向盘 (3x3 grid，中间急停)
- 速度滑块 + 线性/角速度数值显示
- 作物切换胶囊 (番茄/小麦/草莓)
- 巡航状态条 (状态/当前航点/检测株数/进度)
- 航点标签列表 + 航点编辑入口
- AUTO 模式切换按钮

#### 监测页 (Tab 2)
- 实时视频区域 (MJPEG 流，带 REC 指示和分辨率)
- 植株检测状态条 (活跃/置信度/面积比)
- 空气传感器网格 (温度/湿度/CO₂)
- 土壤传感器网格 (N/P/K)
- 叶面传感器 (湿度)
- 数据源标注 + 最后更新时间

注：土壤传感器不测 pH，已从布局中移除。

#### 分析页 (Tab 3)
- 病害诊断结果卡片 (边框高亮色区分严重度)
- 分类概率柱状图 (各病害类别 + 置信度)
- 时序平滑指示
- 农艺建议卡片 (施药方案 + 紧急度)
- 风险趋势迷你图 (3日)
- 病害预警条

#### 天气页 (Tab 4)
- 当前天气大卡片 (温度/天气/湿度/风速/城市)
- 7日预报横向滚动 (日期/图标/温度)
- 逐时预报横向滚动
- 气象灾害预警条
- 农业气象指标 (积温/降雨/风速)

---

## 5. 后端桥接服务 (`miniprogram_bridge_node`)

### 5.1 技术栈

- Python FastAPI + `websockets` 库
- ROS2 节点 (订阅话题 + 调用服务)
- uvicorn 运行，监听 `0.0.0.0:8765`

### 5.2 WebSocket 频道 (`ws://<rdk>:8765/ws`)

统一 JSON 消息格式：`{ "type": "<channel>", "ts": <unix_ms>, "data": {...} }`

| 频道 | 方向 | 频率 | data 内容 |
|------|------|------|-----------|
| `sensor` | 推 | 1Hz | air_temp, air_humidity, co2, soil_temp, soil_humidity, soil_n, soil_p, soil_k, leaf_wetness, source |
| `status` | 推 | 5Hz | mode, linear, angular, battery_voltage, ros_connected |
| `mission` | 推 | 2Hz | state, progress, current_action, plants_detected, plants_analyzed, current_wp_idx, total_wps, waypoint_labels |
| `diagnosis` | 推 | 事件 | crop_type, disease, confidence, probabilities[], smoothed |
| `plant_detect` | 推 | 5Hz | bbox[4], confidence, area_ratio |
| `alert` | 推 | 事件 | level, title, message |

### 5.3 HTTP REST API

| 方法 | 路径 | 请求体 | 响应 | 用途 |
|------|------|--------|------|------|
| `GET` | `/api/status` | - | 全量状态 JSON | 初始加载 / 断线重连同步 |
| `GET` | `/api/weather` | - | 天气 JSON (7日+逐时+预警) | 天气页初始 + 轮询 |
| `GET` | `/api/forecast` | - | 预测+建议 JSON | 分析页轮询 |
| `POST` | `/api/mode` | `{"auto": bool}` | `{"status":"ok","mode":"AUTO"}` | 切换模式 |
| `POST` | `/api/control` | `{"linear":0.3,"angular":0.0}` | `{"status":"ok"}` | 手动速度控制 |
| `POST` | `/api/stop` | - | `{"status":"stopped"}` | 急停 |
| `POST` | `/api/crop_type` | `{"crop_type":"tomato"}` | `{"status":"ok"}` | 切换作物 |
| `GET` | `/api/camera` | - | MJPEG 流 | 视频流 |

### 5.4 ROS2 对接

节点启动时订阅以下话题：

| ROS2 Topic | 消息类型 | 用途 |
|------------|----------|------|
| `/sentry/sensor/environment_mobile` | 自定义 | 移动环境传感器 |
| `/sentry/sensor/soil_nutrition` | 自定义 | 土壤 NPK |
| `/sentry/sensor/leaf` | 自定义 | 叶面传感器 |
| `/sentry/chassis/status` | 自定义 | 底盘状态 |
| `/mission/status` | 自定义 | 巡航状态 |
| `/vision/diagnosis` | 自定义 | 病害诊断结果 |
| `/vision/plant_detected` | 自定义 | 植株检测 |

控制指令通过现有 ROS2 服务/话题转发：
- 模式切换：调用 `/set_auto_mode` 服务
- 速度控制：发布 `/cmd_vel` (MANUAL 模式)
- 作物切换：调用 `/set_crop_type` 服务

---

## 6. 数据流

### 6.1 实时数据路径 (WebSocket)

```
ROS2 Topic → miniprogram_bridge_node 订阅回调
  → asyncio Queue → WebSocket 协程 → JSON 推送
  → 小程序 ws.ts → store.ts 更新 → 页面响应式渲染
```

### 6.2 低频数据路径 (HTTP)

```
小程序 api.ts → wx.request → FastAPI handler
  → 读取最新缓存 / 调用 ROS2 Service
  → JSON 响应 → 页面渲染
```

### 6.3 控制指令路径

```
小程序页面事件 → api.ts POST → FastAPI handler
  → ROS2 Publisher (cmd_vel) / Service (set_auto_mode, set_crop_type)
  → 机器人执行
```

### 6.4 视频流路径

```
IMX219 → camera_node → /sentry/camera/image_raw
  → miniprogram_bridge_node OpenCV 解码 → JPEG 编码
  → multipart/x-mixed-replace HTTP 流
  → 小程序 <image> 或 <live-player>
```

注意：微信小程序的 `<image>` 组件可以直接显示远程 URL 图片，对于 MJPEG 流需要评估兼容性。备选方案是前端定时刷新 `<image>` src（带 cache-busting query），或用 `<live-player>` 组件（需要 RTMP/FLV 推流，实现更复杂）。

---

## 7. 错误处理

| 场景 | 小程序行为 |
|------|-----------|
| WebSocket 断开 | 状态胶囊变红，显示"已断连"，自动重试（3s/15s/30s 退避） |
| HTTP 超时 | 该区域显示 "--" 占位 + 灰色文本"加载失败" |
| 视频流中断 | 显示黑色占位 + "视频信号丢失" + 自动重试 |
| 传感器数据超时 (>10s) | 数值变灰 + "stale" 标记 |
| ROS 节点未启动 | 桥接服务返回 `ros_connected: false`，小程序全局提示 |

---

## 8. 测试策略

| 层级 | 策略 |
|------|------|
| 工具函数 (format.ts, ros-parser.ts) | 微信小程序单元测试 |
| 组件 (dpad, sensor-card 等) | 模拟数据渲染验证 |
| 服务层 (ws.ts, api.ts) | Mock WebSocket/HTTP 服务器 |
| 桥接服务 (Python) | pytest + mock ROS2 话题 |
| 端到端 | Mock 模式：桥接服务注入硬编码数据，小程序全链路验证 |
| 板端集成 | RDK X5 实机测试，与现有 Web 前端对比验证 |

---

## 9. 风险与注意事项

1. **MJPEG 视频流兼容性**：微信小程序对 MJPEG 支持有限，需实际验证。降级方案为 1Hz JPEG 帧刷新。
2. **Skyline 渲染引擎**：比 WebView 性能好，但部分 CSS 属性不支持（如 `overflow-y: auto` 在 Skyline 需用 `scroll-view`），开发时需注意。
3. **WebSocket 域名限制**：开发阶段在微信开发者工具关闭域名校验。正式使用需配置 wss 域名白名单，当前阶段不做。
4. **双端并行**：开发期间 RDK X5 上同时运行 `web_remote_node` (Web 前端) 和 `miniprogram_bridge_node` (小程序)，两者独立，互不影响。
5. **现有前端依赖**：`miniprogram_bridge_node` 的视频流用 OpenCV 而非 `image_transport republish`，需要确保 RDK X5 上 OpenCV 可用（已有相机节点依赖，应无问题）。
