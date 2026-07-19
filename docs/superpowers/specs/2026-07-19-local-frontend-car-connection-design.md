# 本地前端直连小车 · 设计文档

> 版本 v1.0 · 2026-07-19
> 状态：设计确认，待实现

---

## 1. 目标

让两个前端（微信小程序 `wechat/miniprogram` + Web 面板 `src/sentry_mission/static_v2`）在同一局域网下直连真实小车，**全程免 SSH** 实现全部功能：视频流、天气、环境监测数据、自动巡航控制（含整栈一键启停）、病害分析、LLM 深度分析。

用户体验目标：小车上电 → 等约 10 秒 → 打开小程序/浏览器 → 按「启动巡航」→ 所有节点由前端按钮拉起。

---

## 2. 决策总览

| 决策 | 选择 | 理由 |
|------|------|------|
| 整体架构 | 方案 A：统一网关 | 小程序只连 `miniprogram_bridge_node:8765` 一个后端；Web 面板维持 :5000 + rosbridge :9090 现状，风险最小 |
| 巡航编排归属 | 并入 bridge（新增 `/stack/*`） | 复用现有 `scripts/rdk/start_robot_stack.sh`，避免两套编排逻辑 |
| 免 SSH 方案 | systemd 自启网关 + 前端按钮启停工作节点 | 前端按钮的 HTTP 请求必须有进程监听；网关常驻是唯一可行路径（前端代 SSH 不可行：小程序无法 SSH，且凭证入前端代码不安全） |
| IP 配置 | 小程序单点配置 + 设置页可改 | 当前 `10.101.47.106` 硬编码在 api.ts/ws.ts 两处，换网络即断 |
| LLM key | 参数文件/环境变量读取 | 当前 launch 默认空字符串，LLM 端点形同虚设 |
| 网络范围 | 仅局域网，不做真机域名方案 | 开发者工具勾选"不校验合法域名"即可调试，https/wss 白名单留待发布阶段 |

---

## 3. 系统架构

```
微信开发者工具(小程序)                浏览器(Web 面板)
   │ HTTP+WS → 小车IP:8765              │ HTTP:5000 + rosbridge:9090（不变）
   ▼                                   ▼
miniprogram_bridge_node (FastAPI)     web_remote_node + rosbridge
   │ 新增 POST/GET /stack/* 编排端点
   │ 调用 scripts/rdk/start_robot_stack.sh
   └──────────────┬────────────────────┘
                  ▼ ROS2 topics/services
   /cmd_vel, /sensor/*, /weather/forecast, /out/compressed,
   /mission/status, /vision/diagnosis, /llm/analyze

小车上电：
  systemd sentry-bridge.service
    └─ 自动起 miniprogram_bridge.launch.py（bridge + weather_node + LLM 节点）
       + web_remote_node（:5000）
    仅常驻网关，不起相机/Nav2 等重资源节点
前端按钮：
  POST /stack/start → start_robot_stack.sh → 拉起全栈工作节点
  POST /stack/stop  → 停止主栈，网关保留，可再次启动
```

---

## 4. 板端改动

### 4.1 `src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py`

- **修 bug**：订阅话题名 `/sentry/sensor/environment_mobile`、`/sentry/sensor/soil_nutrition` → `/sensor/environment_mobile`、`/sensor/soil_nutrition`（对齐 `uart_bridge_node.py:166-170` 实际发布名）
- **新增端点**（对齐 `web_remote_node` 现有 `/stack/preheat|start|stop` 语义）：
  - `POST /stack/preheat` — 预热（传感器/相机就绪检查）
  - `POST /stack/start` — 幂等，已在运行直接返回当前状态；内部调用 `scripts/rdk/start_robot_stack.sh`
  - `POST /stack/stop` — 停止主栈进程组
  - `GET /stack/status` — 返回 `idle | starting | preheating | cruising | stopping | error`
- **状态检测基于进程而非内存标志**：因 bridge 和 web_remote 都会暴露 `/stack/*` 且各自有独立状态，bridge 判断"主栈是否在跑"时须检测实际进程/节点存活（与 start_robot_stack.sh 的幂等性配合），避免两个前端同时使用时状态视图发散
- **WS 推送**：新增 `stack_status` 消息类型，状态迁移时主动推送，前端实时显示预热/启动进度

### 4.2 `src/sentry_bringup/launch/miniprogram_bridge.launch.py`

- 加入 `sentry_weather/weather_node`（带 `src/sentry_weather/config/weather_params.yaml`），解决"不在任何 launch 中、漏启天气页为空"
- LLM `api_key` 改为从参数文件或环境变量 `SENTRY_LLM_API_KEY` 读取；为空时 `/api/llm/analyze` 返回 503 + 明确文案，而不是静默失败

### 4.3 开机自启 `scripts/rdk/install_autostart.sh`

- 生成并安装 `sentry-bridge.service`（systemd）：开机拉起 `miniprogram_bridge.launch.py` + `web_remote_node`
- 脚本内容：写 service 文件 → `systemctl daemon-reload` → `systemctl enable --now`
- 只需 SSH 执行一次，之后永久免 SSH；网关仅几 MB Python 进程，待机近零功耗

---

## 5. 小程序改动（`wechat/miniprogram/`）

### 5.1 IP 配置化

- 新增 `config.ts`：`CAR_IP` 单点配置（默认值 = 当前硬编码 IP）
- 新增设置入口（控制页或独立设置区）：可修改小车 IP，存 `wx.setStorageSync`，重启后生效
- `services/api.ts:4`、`services/ws.ts:6` 改为从配置读取，消除硬编码

### 5.2 WS 通道接线（修 bug）

- 现状：`wsConnect()` 定义了但**全项目无任何调用点**，实时通道是断的，页面只靠 REST 轮询
- 修复：`app.ts` `onLaunch` 调用 `wsConnect()`；断线指数退避自动重连；连接状态暴露到 store，页面顶部可见

### 5.3 巡航控制增强

- 控制页新增按钮组：「预热」「启动巡航」「停止巡航」→ 调 bridge `/stack/*`
- 巡航状态展示：WS `stack_status` + `/mission/status` 订阅
- 保留现有 `/set_auto_mode` 模式切换

### 5.4 监测页数据源切换

- WS 接通后，环境数据改走 WS `sensor` 推送（实时）；REST 轮询保留作兜底

---

## 6. 错误处理

| 场景 | 处理 |
|------|------|
| 连不上小车 | API 层统一 3s 超时；设置页显示当前 IP；WS 重连状态页面可见 |
| LLM key 未配置 | `/api/llm/analyze` 返回 503 + 文案；小程序 toast 提示 |
| 重复按启动 | `/stack/start` 幂等，返回当前状态，不重复拉起 |
| stack 脚本失败 | `/stack/status` 进入 `error` 态，WS 推送错误原因，前端可重试 |

---

## 7. 验证方式

- **板端**：SSH 上小车跑一次 `install_autostart.sh`，重启验证网关切换自动就绪；curl 验证 `/api/weather`、`/stack/status`；`ros2 topic echo /sensor/environment_mobile` 确认话题名修复后数据流通
- **小程序**：微信开发者工具（勾选"不校验合法域名"）连小车 IP，逐页验证：视频帧刷新、环境数据 WS 推送、天气数据、巡航三按钮全流程、修改 IP 后重连
- **Web 面板回归**：浏览器开 `http://小车IP:5000`，确认现有功能零影响
- **单元测试**：bridge 新增 `/stack/*` 端点用 FastAPI TestClient 覆盖（项目已有 pytest 习惯，见 `.pytest_cache`）

---

## 8. 明确不做（YAGNI）

- 真机 https/wss 合法域名方案（发布阶段再议）
- Web 面板重构或迁移到 bridge（方案 C，回归风险大）
- 云端中继（设计稿已预留，本次不实现）
- 板端端口配置化（8765/5000/9090 固定即可）
