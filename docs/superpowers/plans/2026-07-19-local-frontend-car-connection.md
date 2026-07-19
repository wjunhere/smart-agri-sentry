# 本地前端直连小车 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让微信小程序和 Web 面板在同一局域网下直连真实小车，免 SSH 实现视频流、天气、环境监测、自动巡航一键启停、LLM 分析全部功能。

**Architecture:** 统一网关方案——板端 `miniprogram_bridge_node`(FastAPI :8765) 新增 `/stack/*` 编排端点（复用 `scripts/rdk/start_robot_stack.sh`），systemd 开机自启网关层（bridge + web_remote + weather + LLM），小程序侧 IP 配置化 + WS 通道接线 + 巡航按钮组。Web 面板（:5000 + rosbridge :9090）不动。

**Tech Stack:** ROS2 Humble (rclpy), FastAPI + uvicorn, pytest + FastAPI TestClient, 微信原生小程序 (TS), bash/systemd。

**Spec:** `docs/superpowers/specs/2026-07-19-local-frontend-car-connection-design.md`

**关键背景（执行者需知）：**
- 板端代码实际部署在 RDK X5 的 `/home/sunrise/dev_ws`，本仓库 `E:/smart_agri_sentry` 是开发机镜像；ROS2 相关测试只能在小车上跑（`cd ~/dev_ws && python3 -m pytest src/sentry_miniprogram/test/ -v`）
- 现有测试 `src/sentry_miniprogram/test/test_bridge.py` 用 MagicMock  mock 掉全部 ROS2 依赖，新增测试沿用同一模式
- `llm_advisor_node.py:112-118` 已有 env 兜底：参数为空时读 `DEEPSEEK_API_KEY`，launch 里传 `''` 不阻塞 env 兜底，但要改成显式传递以便 systemd 注入
- `web_remote_node.py:43-56` 的 `_stack_script_env()` 设置 `SENTRY_PRESERVE_WEB=1` + `ENABLE_WEB=false`，前端触发 start 脚本时保留 web 控制面——bridge 必须复用同样语义，否则按按钮会杀掉服务自己的进程

---

### Task 1: 修复 bridge 订阅话题名不匹配

**Files:**
- Modify: `src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py:76-84`
- Test: `src/sentry_miniprogram/test/test_bridge.py`

**背景：** bridge 订阅 `/sentry/sensor/environment_mobile`、`/sentry/sensor/soil_nutrition`，但 `uart_bridge_node.py:166-168` 实际发布 `/sensor/environment_mobile`、`/sensor/soil_nutrition`（其他所有订阅方 forecast/advisory/fusion/llm/data_logger 都用 `/sensor/*`）。`/sentry/chassis/status` 是正确的（uart_bridge 确实发这个名字），不动。

- [ ] **Step 1: 写失败测试**

在 `src/sentry_miniprogram/test/test_bridge.py` 末尾追加：

```python
def test_sensor_topic_names():
    """Bridge must subscribe to the topics uart_bridge actually publishes."""
    node = MiniProgramBridgeNode()
    subscribed = [c.args[1] for c in node.create_subscription.call_args_list]
    assert '/sensor/environment_mobile' in subscribed
    assert '/sensor/soil_nutrition' in subscribed
    assert '/sentry/sensor/environment_mobile' not in subscribed
    assert '/sentry/sensor/soil_nutrition' not in subscribed
```

注意：现有 mock 模式下 `create_subscription` 是 MagicMock（继承自 mock 的 Node），`call_args_list` 可直接断言。

- [ ] **Step 2: 跑测试确认失败**

Run: `python3 -m pytest src/sentry_miniprogram/test/test_bridge.py::test_sensor_topic_names -v`
Expected: FAIL（assert 不通过，当前订阅的是 `/sentry/sensor/*`）

- [ ] **Step 3: 修改订阅话题名**

`miniprogram_bridge_node.py:76-81`：

```python
        self.create_subscription(
            Environment, '/sensor/environment_mobile',
            self._on_environment, 10)
        self.create_subscription(
            SoilNutrition, '/sensor/soil_nutrition',
            self._on_soil, 10)
```

- [ ] **Step 4: 跑测试确认通过 + 全量回归**

Run: `python3 -m pytest src/sentry_miniprogram/test/test_bridge.py -v`
Expected: 全部 PASS

- [ ] **Step 5: Commit**

```bash
git add src/sentry_miniprogram/
git commit -m "fix(bridge): subscribe actual /sensor/* topics published by uart_bridge"
```

---

### Task 2: bridge 新增 /stack/* 巡航编排端点

**Files:**
- Modify: `src/sentry_miniprogram/sentry_miniprogram/miniprogram_bridge_node.py`
- Test: `src/sentry_miniprogram/test/test_bridge.py`

**语义（对齐 `web_remote_node.py:548-607`）：**
- preheat / start 都跑 `start_robot_stack.sh`；start 额外切 AUTO 模式
- 脚本同步执行可能长达 180s，必须放后台线程，端点立即返回，状态经 WS 推送
- "主栈是否在跑"以 `/set_auto_mode` 服务存活判定（进程级事实，非内存标志），避免与 web_remote 状态发散

- [ ] **Step 1: 写失败测试**

在 `test_bridge.py` 追加：

```python
def test_stack_status_idle(client):
    """GET /stack/status returns state machine state."""
    resp = client.get('/stack/status')
    assert resp.status_code == 200
    data = resp.json()
    assert data['state'] in ('idle', 'preheating', 'starting', 'cruising', 'stopping', 'error')


def test_stack_preheat_accepted(node, client):
    """POST /stack/preheat runs the start script in background."""
    node._run_stack_script = MagicMock(return_value=(True, 'ok'))
    resp = client.post('/stack/preheat')
    assert resp.status_code == 200
    assert resp.json()['status'] == 'accepted'


def test_stack_start_calls_script(node, client):
    """POST /stack/start triggers start script."""
    node._run_stack_script = MagicMock(return_value=(True, 'ok'))
    node.mode_srv.service_is_ready = MagicMock(return_value=True)
    node.mode_srv.call_async = MagicMock()
    resp = client.post('/stack/start')
    assert resp.status_code == 200
    assert resp.json()['status'] == 'accepted'


def test_stack_stop_accepted(node, client):
    """POST /stack/stop runs the stop script."""
    node._run_stack_script = MagicMock(return_value=(True, 'ok'))
    resp = client.post('/stack/stop')
    assert resp.status_code == 200
    assert resp.json()['status'] == 'accepted'


def test_llm_analyze_503_when_unavailable(node, client):
    """POST /api/llm/analyze returns 503 when LLM service is absent (no key)."""
    import sentry_miniprogram.miniprogram_bridge_node as bm
    mock_srv = MagicMock()
    mock_srv.wait_for_service = MagicMock(return_value=False)
    node.create_client = MagicMock(return_value=mock_srv)
    resp = client.post('/api/llm/analyze')
    assert resp.status_code == 503
    assert resp.json()['status'] == 'error'
```

- [ ] **Step 2: 跑测试确认失败**

Run: `python3 -m pytest src/sentry_miniprogram/test/test_bridge.py -k stack -v`
Expected: FAIL（404，端点不存在）

- [ ] **Step 3: 实现 stack 编排**

3a. `miniprogram_bridge_node.py` 顶部 import 区追加：

```python
import os
import subprocess
from pathlib import Path
```

3b. 在 `class MiniProgramBridgeNode` 之前加模块级函数：

```python
def _stack_script_env():
    """Environment for frontend-triggered stack scripts.

    Mirrors web_remote_node: preserve the web control plane (web_remote +
    rosbridge + this bridge) when the full stack starts, otherwise the
    frontend's own button click would kill the server handling it.
    """
    env = dict(os.environ)
    env['SENTRY_PRESERVE_WEB'] = '1'
    env['ENABLE_WEB'] = 'false'
    env['ENABLE_VISION'] = 'true'
    env['ENABLE_ADVISORY'] = 'true'
    env['CAMERA_BACKEND'] = 'mipi'
    return env
```

3c. `MiniProgramBridgeNode.__init__` 中，`self._loop = None` 之后追加：

```python
        # Stack orchestration (mirrors web_remote_node semantics)
        self.declare_parameter(
            'stack_start_script',
            '/home/sunrise/dev_ws/scripts/rdk/start_robot_stack.sh')
        self.declare_parameter(
            'stack_stop_script',
            '/home/sunrise/dev_ws/scripts/rdk/stop_robot_stack.sh')
        self.declare_parameter('stack_script_timeout_sec', 180.0)
        self.stack_start_script = self.get_parameter('stack_start_script').value
        self.stack_stop_script = self.get_parameter('stack_stop_script').value
        self.stack_script_timeout = float(
            self.get_parameter('stack_script_timeout_sec').value)
        self.stack_lock = threading.Lock()
        self.stack_state = 'idle'  # idle|preheating|starting|cruising|stopping|error
        self.last_stack_output = ''
```

3d. 类中新增方法（放在 `get_status` 之前）：

```python
    # --- Stack orchestration ---

    def _run_stack_script(self, script_path: str):
        path = Path(script_path)
        if not path.exists():
            return False, f'Script not found: {path}'
        try:
            result = subprocess.run(
                ['bash', str(path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.stack_script_timeout,
                env=_stack_script_env())
        except subprocess.TimeoutExpired as exc:
            return False, f'Script timed out: {path}\n{exc.stdout or ""}'
        except Exception as exc:
            return False, f'Script failed to start: {path}: {exc}'
        output = result.stdout or ''
        self.last_stack_output = output[-4000:]
        if result.returncode != 0:
            return False, output
        return True, output

    def _set_stack_state(self, state: str, message: str = ''):
        self.stack_state = state
        self._push_ws({
            'type': 'stack_status',
            'ts': self._now_ms(),
            'data': {'state': state, 'message': message},
        })

    def _stack_alive(self) -> bool:
        """Process-level fact: mode service only exists when main stack runs."""
        return self.mode_srv.service_is_ready()

    def stack_preheat(self):
        def work():
            with self.stack_lock:
                self._set_stack_state('preheating', 'Running start_robot_stack.sh')
                ok, output = self._run_stack_script(self.stack_start_script)
                if ok and self._stack_alive():
                    self._set_stack_state('idle', 'Preheated; ready to cruise')
                else:
                    self._set_stack_state('error', output[-500:])
        threading.Thread(target=work, daemon=True).start()

    def stack_start(self):
        def work():
            with self.stack_lock:
                self._set_stack_state('starting', 'Running start_robot_stack.sh')
                if not self._stack_alive():
                    ok, output = self._run_stack_script(self.stack_start_script)
                    if not ok:
                        self._set_stack_state('error', output[-500:])
                        return
                if not self._stack_alive():
                    self._set_stack_state('error', '/set_auto_mode not available after stack start')
                    return
                self.set_mode(True)
                self._set_stack_state('cruising', 'Stack running, AUTO mode engaged')
        threading.Thread(target=work, daemon=True).start()

    def stack_stop(self):
        def work():
            with self.stack_lock:
                self._set_stack_state('stopping', 'Running stop_robot_stack.sh')
                ok, output = self._run_stack_script(self.stack_stop_script)
                if ok:
                    self._set_stack_state('idle', 'Stack stopped')
                else:
                    self._set_stack_state('error', output[-500:])
        threading.Thread(target=work, daemon=True).start()

    def get_stack_status(self) -> dict:
        return {
            'state': self.stack_state,
            'stack_alive': self._stack_alive(),
            'last_output': self.last_stack_output[-500:],
        }
```

3e. FastAPI 端点（加在 `get_app()` 里 `@_app.websocket('/ws')` 之前）：

```python
    @_app.get('/stack/status')
    async def stack_status():
        if _node is None:
            return {'state': 'idle', 'stack_alive': False, 'last_output': ''}
        return _node.get_stack_status()

    @_app.post('/stack/preheat')
    async def stack_preheat():
        if _node is None:
            return {'status': 'error', 'message': 'Bridge node not ready'}
        _node.stack_preheat()
        return {'status': 'accepted', 'state': _node.stack_state}

    @_app.post('/stack/start')
    async def stack_start():
        if _node is None:
            return {'status': 'error', 'message': 'Bridge node not ready'}
        _node.stack_start()
        return {'status': 'accepted', 'state': _node.stack_state}

    @_app.post('/stack/stop')
    async def stack_stop():
        if _node is None:
            return {'status': 'error', 'message': 'Bridge node not ready'}
        _node.stack_stop()
        return {'status': 'accepted', 'state': _node.stack_state}
```

3f. LLM 端点改为 503（spec §6 错误处理要求）。`miniprogram_bridge_node.py` 顶部 import 加 `from fastapi.responses import JSONResponse`，现有 `api_llm_analyze` 中：

```python
        if not srv.wait_for_service(timeout_sec=5.0):
            return JSONResponse(
                status_code=503,
                content={'status': 'error',
                         'summary': 'LLM service not available (api_key 未配置或节点未启动)'})
```

- [ ] **Step 4: 跑测试确认通过 + 全量回归**

Run: `python3 -m pytest src/sentry_miniprogram/test/test_bridge.py -v`
Expected: 全部 PASS（注意 mock 模式下 `threading` 未 mock，后台线程会真跑但被 mock 的 `_run_stack_script` 立即返回，无碍）

- [ ] **Step 5: Commit**

```bash
git add src/sentry_miniprogram/
git commit -m "feat(bridge): add /stack/* orchestration endpoints with WS stack_status push"
```

---

### Task 3: 网关联 launch——加入 weather_node、web_remote_node、LLM key 显式传递

**Files:**
- Modify: `src/sentry_bringup/launch/miniprogram_bridge.launch.py`

**背景：** 该 launch 升级为"网关层 launch"：bridge + web_remote（Web 面板及 /stack 备用通道）+ weather + LLM。weather_node 目前不在任何 launch 中。web_remote 放这里后，start 脚本以 `SENTRY_PRESERVE_WEB=1` 运行不会重复拉起（`ENABLE_WEB=false`）。

- [ ] **Step 1: 重写 launch 文件**

`src/sentry_bringup/launch/miniprogram_bridge.launch.py` 完整替换为：

```python
"""Gateway layer launch: miniprogram bridge + web remote + weather + LLM advisor.

Started at boot by systemd sentry-bridge.service (scripts/rdk/install_autostart.sh).
Only lightweight gateway nodes run here; heavy work nodes (camera/Nav2/mission)
are started on demand via POST /stack/* -> start_robot_stack.sh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    weather_params = os.path.join(
        get_package_share_directory('sentry_weather'),
        'config', 'weather_params.yaml')

    llm_api_key = (
        os.environ.get('SENTRY_LLM_API_KEY')
        or os.environ.get('DEEPSEEK_API_KEY', ''))

    return LaunchDescription([
        Node(
            package='sentry_miniprogram',
            executable='miniprogram_bridge_node',
            name='miniprogram_bridge_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='sentry_mission',
            executable='web_remote_node',
            name='web_remote_node',
            output='screen',
        ),
        Node(
            package='sentry_weather',
            executable='weather_node',
            name='weather_node',
            output='screen',
            parameters=[weather_params],
        ),
        Node(
            package='sentry_llm',
            executable='llm_advisor_node',
            name='llm_advisor_node',
            output='screen',
            parameters=[{
                'api_key': llm_api_key,
                'auto_period_sec': 600,
            }],
        ),
    ])
```

注意：两项前置已核实——`sentry_mission/setup.py:42` 有 `web_remote_node` entry point；`sentry_weather/setup.py:13` 已把 `config/weather_params.yaml` 装入 share 目录。`llm_advisor_node.py:113-116` 确认参数为空时兜底读 `DEEPSEEK_API_KEY` 环境变量。

- [ ] **Step 2: 静态校验**

Run: `python3 -c "import ast; ast.parse(open('src/sentry_bringup/launch/miniprogram_bridge.launch.py').read())"`
Expected: 无输出（语法 OK）

- [ ] **Step 3: Commit**

```bash
git add src/sentry_bringup/launch/miniprogram_bridge.launch.py
git commit -m "feat(bringup): gateway launch with web_remote + weather + LLM key from env"
```

---

### Task 4: 开机自启脚本 install_autostart.sh

**Files:**
- Create: `scripts/rdk/install_autostart.sh`

- [ ] **Step 1: 写脚本**

```bash
#!/usr/bin/env bash
# Install systemd autostart for the sentry gateway layer (bridge :8765 +
# web_remote :5000 + weather + LLM). Run once over SSH; afterwards the car
# boots straight into "frontend reachable" state, no SSH needed.
set -euo pipefail

WS_DIR="${SENTRY_WS:-/home/sunrise/dev_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
SERVICE_NAME="sentry-bridge.service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"
RUN_USER="${SENTRY_USER:-sunrise}"

if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
  echo "ERROR: ${WS_DIR}/install/setup.bash missing; build the workspace first." >&2
  exit 1
fi

sudo tee "${SERVICE_FILE}" >/dev/null <<EOF
[Unit]
Description=Sentry gateway layer (miniprogram bridge + web_remote + weather + LLM)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
Environment=SENTRY_LLM_API_KEY=${SENTRY_LLM_API_KEY:-}
Environment=DEEPSEEK_API_KEY=${DEEPSEEK_API_KEY:-}
ExecStart=/bin/bash -lc 'source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && source ${WS_DIR}/install/setup.bash && exec ros2 launch sentry_bringup miniprogram_bridge.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}"
echo "Installed and started ${SERVICE_NAME}."
echo "Check: systemctl status ${SERVICE_NAME}"
echo "Logs:  journalctl -u ${SERVICE_NAME} -f"
```

- [ ] **Step 2: 静态校验 + 赋可执行权限**

Run: `bash -n scripts/rdk/install_autostart.sh && chmod +x scripts/rdk/install_autostart.sh`
Expected: 无输出

- [ ] **Step 3: Commit**

```bash
git add scripts/rdk/install_autostart.sh
git commit -m "feat(rdk): systemd autostart installer for gateway layer"
```

---

### Task 5: 小程序 IP 配置化

**Files:**
- Create: `wechat/miniprogram/services/config.ts`
- Modify: `wechat/miniprogram/services/api.ts:4,9`
- Modify: `wechat/miniprogram/services/ws.ts:6,20`

- [ ] **Step 1: 新建 config.ts**

```typescript
// services/config.ts
// Car connection config — single source of truth, user-overridable in settings.

const DEFAULT_CAR_IP = '10.101.47.106';
const API_PORT = 8765;
const STORAGE_KEY = 'car_ip';

export function getCarIp(): string {
  return wx.getStorageSync(STORAGE_KEY) || DEFAULT_CAR_IP;
}

export function setCarIp(ip: string): void {
  wx.setStorageSync(STORAGE_KEY, ip.trim());
}

export function getBaseUrl(): string {
  return `http://${getCarIp()}:${API_PORT}`;
}

export function getWsUrl(): string {
  return `ws://${getCarIp()}:${API_PORT}/ws`;
}
```

- [ ] **Step 2: api.ts 改为动态读取**

`api.ts:4` 删除 `const BASE_URL = ...`，import 并改 `request` 与两个 URL getter：

```typescript
import { getBaseUrl } from './config';

async function request<T>(method: 'GET' | 'POST', path: string, body?: any): Promise<T> {
  return new Promise((resolve, reject) => {
    wx.request({
      url: getBaseUrl() + path,
      timeout: 3000,  // spec §6: 统一 3s 超时，连不上小车时快速失败
      // ...其余不变
```

`getCameraUrl()` / `getCameraSnapshotUrl()` 里 `BASE_URL` 替换为 `getBaseUrl()`。

注意：`timeout: 3000` 是全局默认，但 LLM 分析是长请求（bridge 侧最多等 65s）——`apiLLMAnalyze` 需改用单独的超时。做法：`request` 增加可选参数 `timeoutMs = 3000` 透传给 `wx.request`，`apiLLMAnalyze` 调用时传 `70000`。

- [ ] **Step 3: ws.ts 改为动态读取**

`ws.ts:6` 删除 `const WS_URL = ...`，顶部 `import { getWsUrl } from './config';`，`wsConnect()` 里 `url: WS_URL` 改 `url: getWsUrl()`。

- [ ] **Step 4: 追加 stack API 到 api.ts 末尾**

```typescript
export function apiStackPreheat() {
  return request<{status: string; state: string}>('POST', '/stack/preheat');
}

export function apiStackStart() {
  return request<{status: string; state: string}>('POST', '/stack/start');
}

export function apiStackStop() {
  return request<{status: string; state: string}>('POST', '/stack/stop');
}

export function apiStackStatus() {
  return request<{state: string; stack_alive: boolean; last_output: string}>('GET', '/stack/status');
}
```

- [ ] **Step 5: 编译验证**

微信开发者工具中编译无 TS 报错（`wechat/` 目录，工具自动 tsc）。
Run（可选，若本机有 tsc）: `cd wechat && npx tsc --noEmit -p tsconfig.json`
Expected: 无错误

- [ ] **Step 6: Commit**

```bash
git add wechat/miniprogram/services/
git commit -m "feat(miniprogram): configurable car IP + stack orchestration APIs"
```

---

### Task 6: 小程序 WS 接线 + stack_status 消息 + 设置入口

**Files:**
- Modify: `wechat/miniprogram/app.ts`
- Modify: `wechat/miniprogram/services/store.ts`
- Modify: `wechat/miniprogram/services/ws.ts`
- Modify: `wechat/miniprogram/pages/control/control.ts`
- Modify: `wechat/miniprogram/pages/control/control.wxml`
- Modify: `wechat/miniprogram/pages/control/control.less`（如需要样式）

- [ ] **Step 1: store.ts 增加 stack 字段**

在 `missionWaypointLabels: [] as string[],` 之后加：

```typescript
  stackState: 'idle' as string,
  stackMessage: '',
  carIp: '',
```

- [ ] **Step 2: app.ts 启动时连 WS**

```typescript
// app.ts
import { wsConnect } from './services/ws';
import { updateStore } from './services/store';
import { getCarIp } from './services/config';

App<IAppOption>({
  globalData: {},
  onLaunch() {
    updateStore({ carIp: getCarIp() });
    wsConnect();
  },
})
```

- [ ] **Step 3: ws.ts 处理 stack_status**

`handleMessage` 的 switch 中 `case 'llm':` 之后加：

```typescript
    case 'stack_status':
      updateStore({
        stackState: data.state,
        stackMessage: data.message || '',
      });
      break;
```

同时在 `snapshot` case 里可顺带同步（bridge 的 snapshot 不含 stack，跳过）。

- [ ] **Step 4: 控制页加巡航按钮组 + IP 设置**

`control.ts`：

4a. import 更新：

```typescript
import { apiSetMode, apiControl, apiStop, apiSetCropType,
         apiStackPreheat, apiStackStart, apiStackStop } from '../../services/api';
import { setCarIp } from '../../services/config';
import { wsConnect } from '../../services/ws';
```

4b. data 增加 `stackState: 'idle', stackMessage: '', carIp: '', connected: false`，`sync()` 里同步这四个字段。

4c. methods 增加：

```typescript
    onStackPreheat() { apiStackPreheat(); },
    onStackStart()   { apiStackStart(); },
    onStackStop()    { apiStackStop(); },

    onIpInput(e: any) {
      this.setData({ carIp: e.detail.value });
    },
    onSaveIp() {
      const ip = (this.data.carIp || '').trim();
      if (!/^\d{1,3}(\.\d{1,3}){3}$/.test(ip)) {
        wx.showToast({ title: 'IP 格式不对', icon: 'none' });
        return;
      }
      setCarIp(ip);
      updateStore({ carIp: ip });
      wsConnect();  // reconnect with new IP
      wx.showToast({ title: '已保存并重连', icon: 'none' });
    },
```

4d. `control.wxml` 在模式切换区附近加（按现有 wxml 结构调整类名以贴合现有样式）：

```xml
  <!-- 连接状态（spec §6：WS 连接状态页面可见） -->
  <view class="conn-badge {{connected ? 'conn-ok' : 'conn-bad'}}">
    {{connected ? '已连接' : '未连接'}}
  </view>

  <!-- 巡航编排 -->
  <view class="stack-panel">
    <view class="stack-status">整栈状态: {{stackState}} {{stackMessage}}</view>
    <view class="stack-btns">
      <button size="mini" bindtap="onStackPreheat">预热</button>
      <button size="mini" type="primary" bindtap="onStackStart">启动巡航</button>
      <button size="mini" type="warn" bindtap="onStackStop">停止巡航</button>
    </view>
  </view>

  <!-- 小车 IP 设置 -->
  <view class="ip-panel">
    <input class="ip-input" value="{{carIp}}" bindinput="onIpInput" placeholder="小车 IP" />
    <button size="mini" bindtap="onSaveIp">保存</button>
  </view>
```

- [ ] **Step 5: 编译验证**

微信开发者工具编译通过；用 mock 方式（开发者工具 Network 面板）确认 `onLaunch` 发起了 `ws://<ip>:8765/ws` 连接。

- [ ] **Step 6: Commit**

```bash
git add wechat/miniprogram/
git commit -m "feat(miniprogram): wire WS at launch, cruise stack buttons, IP settings"
```

---

### Task 7: 小车端部署与联调验证（在 RDK X5 上执行）

**Files:** 无新文件；部署 + 验证清单

前置：开发机改动同步到小车（git pull 或 rsync 到 `/home/sunrise/dev_ws`），`colcon build --packages-select sentry_miniprogram sentry_bringup sentry_weather sentry_llm`（或全量），`source install/setup.bash`。

- [ ] **Step 1: 小车跑 bridge 单测**

Run（小车）: `cd ~/dev_ws && python3 -m pytest src/sentry_miniprogram/test/ -v`
Expected: 全部 PASS

- [ ] **Step 2: 安装自启并重启验证**

Run（小车）: `bash scripts/rdk/install_autostart.sh && sudo reboot`
重启后（小车）: `systemctl status sentry-bridge.service`、`curl -s http://127.0.0.1:8765/stack/status`、`curl -s http://127.0.0.1:5000/status`
Expected: 服务 active；8765 返回 `{"state":"idle",...}`；5000 返回 JSON

- [ ] **Step 3: 一键启动全栈**

Run（小车或任意同网机器）: `curl -X POST http://<car-ip>:8765/stack/start`，然后轮询 `curl -s http://<car-ip>:8765/stack/status`
Expected: 状态 `starting → cruising`；`ros2 node list` 见 mission/uart_bridge/nav2 等节点

- [ ] **Step 4: 话题数据验证**

Run（小车）: `ros2 topic echo /sensor/environment_mobile --once`
Expected: 有数据（uart_bridge 发布后 bridge 能收到——Task 1 修复点）

- [ ] **Step 5: 小程序端到端**

微信开发者工具（详情→本地设置→勾选"不校验合法域名"）：
- 监测页：视频帧刷新（200ms snapshot 轮询）+ 环境数据经 WS 实时更新
- 天气页：`GET /api/weather` 有数据（weather_node 已由网关 launch 拉起）
- 控制页：预热 → 启动巡航 → 状态实时变化 → 停止巡航；修改 IP 保存后自动重连
- 分析页：诊断概率条、预警、LLM 深度分析（key 已配则返回结果，未配则 503 文案提示）

- [ ] **Step 6: Web 面板回归**

浏览器开 `http://<car-ip>:5000`：视频、手动控制、巡航按钮、rosbridge 数据全部正常（零改动回归）。

- [ ] **Step 7: 更新文档 + 最终 commit**

- `docs/ROS2.md` §2.5 补 bridge 的 `/stack/*` 端点表
- `docs/ARCHITECTURE.md` 补 systemd 网关自启说明（sentry-bridge.service）
- 验证记录写入 commit message 或 docs/ISSUES.md 关闭对应条目

```bash
git add docs/
git commit -m "docs: gateway autostart + bridge /stack endpoints + field verification"
```

---

## 风险与备注

- **Task 3 前置检查**：`weather_params.yaml` 若未被 `sentry_weather/setup.py` 安装到 share 目录，`get_package_share_directory` 路径会找不到文件——执行时先验证，必要时补 `data_files`
- **mock 测试的线程**：Task 2 测试中后台线程真实运行，依赖被 mock 的方法立即返回；若出现竞态（状态断言早于线程执行），测试只断言端点返回值，不断言最终状态
- **IP 默认值的现实**：`10.101.47.106` 是学校内网地址，换环境后第一次使用需在设置里改 IP——这是设计内的行为
- **微信小程序真机**：本次只在开发者工具运行；真机需 https/wss 合法域名（spec §8 明确不做）
