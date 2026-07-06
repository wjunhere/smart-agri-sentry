# Web 前端 Dashboard — 设计方案

> 日期：2026-07-06 · 状态：设计已确认，待实施

---

## Context

当前项目只有一个简陋的遥控页面（`web_remote_node` + 单文件 `index.html`），仅支持方向键控车和 AUTO/MANUAL 切换。需要新增一个功能完整的仪表盘页面，实现：远程驾驶、实时画面、YOLO/MobileNet 识别结果、分级预警趋势、农艺建议追溯、固定环境节点数据展示、作物类型切换、一键自动巡航（含航点选择 + 暂停恢复）。

设计目标：**可解释、可追溯** — 每条预警记录关联现场图像快照与环境快照，点击即可查看完整决策依据。

---

## 技术选型

| 层 | 方案 | 理由 |
|---|---|---|
| 前端框架 | Vue 3 (CDN 引入，无构建工具链) | 多数据面板仪表盘，响应式绑定自然适配实时推送；RDK X5 上零构建成本 |
| 前端 ROS 库 | roslibjs (CDN) | 官方 rosbridge 客户端，直接订阅话题/调用服务 |
| 桥接层 | rosbridge_server (rosbridge_suite) | WebSocket 长连接，延迟最低；compressed_image_transport 传图省带宽 |
| 图表 | Chart.js (CDN) | 轻量，24h 风险趋势折线图够用 |
| CSS | 手写 CSS Grid/Flexbox | 仪表盘布局，不需要 UI 库 |

---

## 后端改造清单

| 改造点 | 说明 |
|---|---|
| **rosbridge_server** | 新依赖，`apt install ros-humble-rosbridge-server`，WebSocket 9090 端口 |
| **compressed_image_transport** | 相机节点开启 `image_transport` 插件，发布 `/sentry/camera/image_raw/compressed` |
| **MissionStatus 增强** | 消息新增 `current_wp_idx`、`total_wps`、`waypoint_names` 字段，前端展示航点列表 |
| **/set_crop_type 服务** | 新增自定义服务，调用后 shell 脚本重启 `vision_diagnosis_node` + `vision_pipeline_node` + `fusion_node` + `forecast_node` + `advisory_node`，约 5-10 秒 |
| **历史预警存储** | `data_logger_node` 订阅 `/fusion/diagnosis`，落盘为 JSON（含 timestamp、图像 base64、环境快照、证据链） |

---

## 前端架构

### 组件树

```
App.vue
├── TopBar.vue              — 模式指示、电池电压、ROS/LoRa 连接状态
├── MainContent.vue         — 左右分栏容器
│   ├── CameraPanel.vue     — 实时画面 (roslibjs 订阅 compressed image)
│   ├── DetectionCard.vue   — YOLO 检测结果 (bbox 叠加、置信度、面积比)
│   ├── DiagnosisCard.vue   — MobileNet 病害分类 (病害名、置信度、概率分布)
│   └── AdvisoryCard.vue    — 当前农艺建议 (行动文本、药剂、紧急程度)
├── ForecastPanel.vue       — 24h 预警趋势折线图 (Chart.js) + 时间轴列表
├── AlertDetailModal.vue    — 预警详情弹窗 (图像快照 + 环境快照 + 证据链 + 处置措施)
├── EnvDataBar.vue          — 固定环境节点数据条 (温湿度/CO2/土壤NPK/pH)
├── ControlPanel.vue        — 控制区
│   ├── Dpad.vue            — 方向键 (发布 /cmd_vel)
│   ├── CropSelector.vue    — 作物类型下拉 (调用 /set_crop_type)
│   └── CruisePanel.vue     — 巡航控制 (航点勾选、启动/暂停/恢复)
└── StatusBar.vue           — 当前状态机状态、时间戳
```

### 数据流

```
ROS2 话题                                → rosbridge WS → Vue 组件
──────────────────────────────────────────────────────────────────
/sentry/camera/image_raw/compressed      → CameraPanel
/vision/plant_detected                   → CameraPanel (bbox overlay) + DetectionCard
/vision/diagnosis                        → DiagnosisCard
/advisory/action                         → AdvisoryCard
/forecast/alert                          → ForecastPanel
/fusion/diagnosis                        → AlertDetailModal (点击时)
/sensor/environment_fixed                → EnvDataBar
/mission/status                          → StatusBar + CruisePanel
/sentry/chassis/status                   → TopBar (电池)

Vue 组件                     → rosbridge → ROS2 服务/话题
──────────────────────────────────────────────────────────────────
Dpad                         → publish /cmd_vel (Twist)
CruisePanel [启动/暂停]      → call /set_auto_mode (SetBool)
CruisePanel [恢复]           → publish /resume_navigation (Bool)
CropSelector                 → call /set_crop_type (自定义 srv)
```

### 路由

单页应用，无需 Vue Router。以卡片/Tab 切换区分功能区域，所有面板在同一视口内可见（仪表盘设计）。

---

## 关键交互设计

### 1. 实时画面 + YOLO 叠加

- `CameraPanel` 订阅 `/sentry/camera/image_raw/compressed`（JPEG, ~2Hz）
- 收到 YOLO 检测结果 (`/vision/plant_detected`) 后，用 Canvas 在原图上绘制 bbox + 类别标签 + 置信度
- 检测到植株时边框闪烁绿色，未检测时画面正常

### 2. 分级预警趋势

- `ForecastPanel` 顶部 Chart.js 折线图，X 轴 24h 时间，Y 轴风险值 [0,1]
- 每条数据点按 alert 级别着色：NORMAL=绿, SUSPICION=黄, WARNING=橙, CRITICAL=红
- 折线图下方时间轴列表，每条预警记录含：时间、病害名、风险值、📷快照 🌡️环境 📋详情 三个按钮
- 点击按钮 → 打开 `AlertDetailModal`

### 3. 预警详情弹窗（可解释、可追溯）

```
┌───────────────────────────────────────┐
│  预警详情 — 2026-07-06 14:23          │
│                                       │
│  ┌──────────────┐ ┌─────────────────┐ │
│  │ 📷 现场快照   │ │ 环境快照        │ │
│  │ (当时摄像头帧) │ │ 气温 26°C       │ │
│  │              │ │ 湿度 88%RH      │ │
│  │              │ │ 叶面湿润 6.2h   │ │
│  └──────────────┘ │ 土壤 T22°C H60% │ │
│                   └─────────────────┘ │
│  📋 农艺建议                           │
│  ▸ 建议2小时内喷施代森锰锌800倍液       │
│  ▸ 间隔7天后二次喷施                   │
│                                       │
│  🧾 决策依据 (证据链)                  │
│  ▸ 视觉: 92.3% 晚疫病                 │
│  ▸ 环境: 湿度 88% > 阈值 85%          │
│  ▸ 交互: 叶面湿润 6.2h > 阈值 6h      │
│                                       │
│                  [关闭] [导出报告]      │
└───────────────────────────────────────┘
```

### 4. 作物类型切换

- `CropSelector` 下拉框：番茄 / 小麦 / 草莓
- 选择后弹出确认对话框提示"切换将重启相关节点，约 5-10 秒不可用"
- 确认后调用 `/set_crop_type` 服务，显示加载动画，轮询等待节点恢复

### 5. 自动巡航控制

- `CruisePanel` 加载时从 `/mission/status` 获取航点列表
- 每个航点前有 checkbox，默认全选，用户可取消勾选跳过某些航点
- **启动**：一键切换到 AUTO 模式，mission_control 从选中航点开始巡逻
- **暂停**：切到 MANUAL，保存当前进度
- **恢复**：publish `/resume_navigation`，从断点继续
- 当前航点高亮，已完成的变灰

### 6. 方向键遥控

- 复用老页面 `web_remote_node` 的 D-pad 逻辑：mousedown/touchstart 发送速度，mouseup/touchend 归零
- 速度滑块可调最大线速度/角速度
- 急停按钮（红色大按钮，醒目）

---

## 文件结构

### 新增文件

```
src/sentry_mission/static_v2/        # 新版前端（与老 static/ 并存）
├── index.html                       # 入口，CDN 引入 Vue 3 + roslibjs + Chart.js
├── app.js                           # Vue 3 应用入口，createApp + 组件注册
├── components/
│   ├── TopBar.js                    # 顶栏
│   ├── CameraPanel.js              # 实时画面 + YOLO bbox 叠加
│   ├── DetectionCard.js            # YOLO 检测卡片
│   ├── DiagnosisCard.js            # 病害分类卡片
│   ├── AdvisoryCard.js             # 农艺建议卡片
│   ├── ForecastPanel.js            # 预警趋势 + 时间轴
│   ├── AlertDetailModal.js         # 预警详情弹窗
│   ├── EnvDataBar.js              # 环境数据条
│   ├── Dpad.js                    # 方向键
│   ├── CropSelector.js            # 作物选择器
│   ├── CruisePanel.js             # 巡航控制
│   └── StatusBar.js               # 状态栏
├── ros.js                           # roslibjs 封装：连接管理、话题订阅、服务调用
└── style.css                        # 全局样式
```

### 后端修改文件

```
src/sentry_mission/sentry_mission/web_remote_node.py  # 新增 /v2 路由，serve static_v2/
src/sentry_interfaces/msg/MissionStatus.msg           # 新增字段
src/sentry_interfaces/srv/SetCropType.srv              # 新增服务定义
src/sentry_mission/sentry_mission/mission_control_node.py # 服务端实现 + 航点状态发布
```

---

## 验证方法

1. **本地静态测试**：浏览器打开 `index.html`，确认 UI 布局和 mock 数据渲染
2. **板端 rosbridge 连通性**：启动 `rosbridge_server`，浏览器连接 `ws://<rdk-ip>:9090`，确认 `roslibjs` 能订阅话题
3. **全链路集成测试**：
   - 浏览器打开仪表盘 → 确认实时画面显示
   - 方向键控车 → 确认底盘响应
   - 切换作物类型 → 确认节点重启并恢复
   - 一键巡航 → 确认航点列表正确、启动/暂停/恢复正常
   - 预警弹窗 → 确认图像快照、环境数据、证据链、农艺建议完整
4. **老页面不受影响**：访问 `http://<rdk>:5000/` 仍显示老遥控页面，`/v2` 显示新仪表盘
