# Web 前端 Dashboard — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增一个 Vue 3 仪表盘页面（`/v2`），实现远程驾驶、实时画面+YOLO叠加、病害识别结果、分级预警趋势、农艺建议追溯、环境数据展示、作物切换、一键巡航。

**Architecture:** rosbridge_server (WebSocket 9090) 桥接 ROS2 与浏览器，Vue 3 CDN 单页应用通过 roslibjs 直接订阅话题/调用服务。后端新增 SetCropType 服务 + MissionStatus 字段增强。所有静态文件由 web_remote_node Flask 新增的 `/v2` 路由 serve，与老页面并存。

**Tech Stack:** Vue 3 (CDN), roslibjs (CDN), Chart.js (CDN), rosbridge_server, Flask, Python

---

### Task 1: MissionStatus 消息增强 — 航点信息

**Files:**
- Modify: `src/sentry_interfaces/msg/MissionStatus.msg`
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py:257-260,380-381`

让前端能展示航点列表和当前进度。

- [ ] **Step 1: 修改 MissionStatus.msg，新增航点字段**

```msg
std_msgs/Header header
string state
float32 progress
string current_action
uint32 plants_analyzed
uint32 plants_detected
uint32 current_wp_idx
uint32 total_wps
string[] waypoint_labels
```

- [ ] **Step 2: 在 mission_control_node tick() 中填充新字段**

修改 `tick()` 中 MissionStatus 构造部分 (line 257-260):

```python
status = MissionStatus()
status.header.stamp = self.get_clock().now().to_msg()
status.plants_detected = self.plants_detected
status.plants_analyzed = self.plants_analyzed
status.current_wp_idx = self.current_wp_idx
status.total_wps = len(self.waypoints)
status.waypoint_labels = [
    f'WP{i}: ({wp["x"]:.1f}, {wp["y"]:.1f})'
    for i, wp in enumerate(self.waypoints)
]
```

- [ ] **Step 3: 提交**

```bash
git add src/sentry_interfaces/msg/MissionStatus.msg src/sentry_mission/sentry_mission/mission_control_node.py
git commit -m "feat: add waypoint fields to MissionStatus message"
```

---

### Task 2: SetCropType 服务定义与实现

**Files:**
- Create: `src/sentry_interfaces/srv/SetCropType.srv`
- Modify: `src/sentry_mission/sentry_mission/mission_control_node.py` — 新增服务端
- Modify: `src/sentry_mission/sentry_mission/web_remote_node.py` — 新增 HTTP 端点转发

提供运行时作物类型切换能力（通过重启相关节点实现）。

- [ ] **Step 1: 创建服务定义**

```srv
string crop_type
---
bool success
string message
```

文件: `src/sentry_interfaces/srv/SetCropType.srv`

- [ ] **Step 2: 修改 CMakeLists.txt 注册新服务**

需要找到 srv 的 CMakeLists.txt。检查 `src/sentry_interfaces/CMakeLists.txt`，在 `rosidl_generate_interfaces` 中添加 `srv/SetCropType.srv`。

- [ ] **Step 3: 在 mission_control_node 实现服务端**

在 `__init__` 中添加服务创建:

```python
self.crop_type_srv = self.create_service(
    SetCropType, '/set_crop_type', self.set_crop_type_cb)
```

回调实现:

```python
def set_crop_type_cb(self, request, response):
    valid = {'tomato', 'wheat', 'strawberry'}
    if request.crop_type not in valid:
        response.success = False
        response.message = f'Invalid crop type: {request.crop_type}. Valid: {valid}'
        return response

    self.crop_type = request.crop_type
    self.get_logger().info(f'Crop type set to {request.crop_type}')

    # Restart affected nodes via shell
    import subprocess
    nodes = ['vision_diagnosis_node', 'vision_pipeline_node',
             'fusion_node', 'forecast_node', 'advisory_node']
    try:
        for node_name in nodes:
            subprocess.run(
                ['ros2', 'lifecycle', 'set', node_name, 'inactive'],
                capture_output=True, timeout=10)
    except Exception:
        pass

    response.success = True
    response.message = f'Crop type set to {request.crop_type}. Restarting vision nodes...'
    return response
```

需要在文件顶部导入: `from sentry_interfaces.srv import SetCropType`

- [ ] **Step 4: 在 web_remote_node 添加 HTTP 端点**

在 `_get_app()` 中添加:

```python
@_app.route('/crop_type', methods=['POST'])
def set_crop_type():
    data = request.get_json()
    crop = data.get('crop_type', '')
    # Forward to ROS2 service via a simple mechanism:
    # Store in node attribute, timer picks it up
    node._pending_crop_type = crop
    return jsonify({'status': 'ok', 'crop_type': crop})
```

由于 Flask 线程和 ROS2 主线程隔离，需要加一个轻量机制。更简单方案：在 WebRemoteNode 加一个 `/set_crop_type` 的 service client，HTTP 端点直接调异步:

```python
# In WebRemoteNode.__init__:
self.crop_type_client = self.create_client(SetCropType, '/set_crop_type')

# In _get_app:
@_app.route('/crop_type', methods=['POST'])
def set_crop_type():
    data = request.get_json()
    crop = data.get('crop_type', '')
    if node.crop_type_client.wait_for_service(timeout_sec=1.0):
        req = SetCropType.Request()
        req.crop_type = crop
        future = node.crop_type_client.call_async(req)
        # Spin a bit to get response
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if future.result() is not None and future.result().success:
            return jsonify({'status': 'ok', 'message': future.result().message})
    return jsonify({'status': 'error', 'message': 'Service unavailable'})
```

- [ ] **Step 5: 提交**

```bash
git add src/sentry_interfaces/srv/SetCropType.srv src/sentry_interfaces/CMakeLists.txt \
        src/sentry_mission/sentry_mission/mission_control_node.py \
        src/sentry_mission/sentry_mission/web_remote_node.py
git commit -m "feat: add /set_crop_type service for runtime crop type switching"
```

---

### Task 3: web_remote_node 新增 /v2 路由

**Files:**
- Modify: `src/sentry_mission/sentry_mission/web_remote_node.py:131-172`
- Create: `src/sentry_mission/static_v2/` (目录)

在现有 Flask 应用上新增 `/v2` 路由 serve 新版仪表盘，老页面 `/` 保持不变。

- [ ] **Step 1: 在 _get_app() 中添加 /v2 路由**

```python
STATIC_V2_DIR = Path(__file__).parent / 'static_v2'

@_app.route('/v2')
def v2_index():
    return send_from_directory(str(STATIC_V2_DIR), 'index.html')

@_app.route('/v2/<path:filename>')
def v2_static(filename):
    return send_from_directory(str(STATIC_V2_DIR), filename)
```

- [ ] **Step 2: 更新 setup.py data_files 打包 static_v2**

在 `setup.py` 的 `data_files` 中添加 `static_v2/` 目录及其所有子文件。

- [ ] **Step 3: 创建 static_v2 目录结构**

```bash
mkdir -p src/sentry_mission/static_v2/components
```

- [ ] **Step 4: 提交**

```bash
git add src/sentry_mission/sentry_mission/web_remote_node.py \
        src/sentry_mission/setup.py
git commit -m "feat: add /v2 route for new dashboard (Flask)"
```

---

### Task 4: rosbridge 封装层 (ros.js) + 入口文件

**Files:**
- Create: `src/sentry_mission/static_v2/ros.js`
- Create: `src/sentry_mission/static_v2/app.js`
- Create: `src/sentry_mission/static_v2/index.html`
- Create: `src/sentry_mission/static_v2/style.css`

CDN 依赖管理 + roslibjs 连接封装 + 全局状态(响应式) + 应用入口。

- [ ] **Step 1: 创建 index.html**

```html
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>智农哨兵 · 仪表盘</title>
  <script src="https://unpkg.com/vue@3/dist/vue.global.prod.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>
  <link rel="stylesheet" href="style.css">
</head>
<body>
  <div id="app">
    <top-bar></top-bar>
    <div class="main-layout">
      <div class="left-panel">
        <camera-panel></camera-panel>
        <env-data-bar></env-data-bar>
      </div>
      <div class="right-panel">
        <detection-card></detection-card>
        <diagnosis-card></diagnosis-card>
        <advisory-card></advisory-card>
        <forecast-panel></forecast-panel>
      </div>
    </div>
    <control-panel></control-panel>
    <status-bar></status-bar>
    <alert-detail-modal></alert-detail-modal>
  </div>

  <!-- Component files (order matters: ros.js first, then components, app.js last) -->
  <script src="ros.js"></script>
  <script src="components/top-bar.js"></script>
  <script src="components/camera-panel.js"></script>
  <script src="components/detection-card.js"></script>
  <script src="components/diagnosis-card.js"></script>
  <script src="components/advisory-card.js"></script>
  <script src="components/forecast-panel.js"></script>
  <script src="components/alert-detail-modal.js"></script>
  <script src="components/env-data-bar.js"></script>
  <script src="components/dpad.js"></script>
  <script src="components/crop-selector.js"></script>
  <script src="components/cruise-panel.js"></script>
  <script src="components/status-bar.js"></script>
  <script src="components/control-panel.js"></script>
  <script src="app.js"></script>
</body>
</html>
```

- [ ] **Step 2: 创建 ros.js — 连接管理 + 话题订阅 + 服务调用**

```javascript
// ros.js — roslibjs connection wrapper and reactive state store
const ROS_CONFIG = {
  url: 'ws://' + window.location.hostname + ':9090'
};

// Reactive global state (Vue 3 reactive, imported after Vue loads)
const store = Vue.reactive({
  connected: false,
  // Camera
  cameraFrame: null,       // base64 JPEG
  // YOLO detection
  plantDetected: false,
  plantConfidence: 0,
  plantBbox: [0, 0, 0, 0],
  plantAreaRatio: 0,
  // Diagnosis
  diagnosisCropType: '',
  diagnosisDisease: '',
  diagnosisConfidence: 0,
  diagnosisProbabilities: [],
  // Advisory
  advisoryText: '',
  advisoryUrgency: 0,
  advisoryFungicide: '',
  // Forecast
  forecastAlerts: [],      // array of {time, risk, level, disease, ...}
  // Fusion (for alert details)
  fusionResults: [],       // history buffer
  // Environment (fixed node)
  envAirTemp: null,
  envAirHumidity: null,
  envCO2: null,
  envSoilTemp: null,
  envSoilHumidity: null,
  envSoilN: null,
  envSoilP: null,
  envSoilK: null,
  envSoilPH: null,
  envLeafWetness: null,
  // Mission status
  missionState: 'IDLE',
  missionProgress: 0,
  missionCurrentAction: '',
  missionPlantsDetected: 0,
  missionPlantsAnalyzed: 0,
  missionCurrentWpIdx: 0,
  missionTotalWps: 0,
  missionWaypointLabels: [],
  // Chassis
  batteryVoltage: null,
  leftSpeed: 0,
  rightSpeed: 0,
  // Mode
  mode: 'AUTO',
  cropType: 'tomato',
  // Selected alert for modal
  selectedAlert: null,
});

let ros = null;

function rosConnect() {
  ros = new ROSLIB.Ros({ url: ROS_CONFIG.url });

  ros.on('connection', () => {
    store.connected = true;
    console.log('[ROS] Connected');
    subscribeAll();
  });

  ros.on('close', () => {
    store.connected = false;
    console.log('[ROS] Disconnected, retrying in 3s...');
    setTimeout(rosConnect, 3000);
  });

  ros.on('error', (err) => {
    console.warn('[ROS] Error:', err);
  });
}

// Topic definitions: [topic_name, message_type, callback]
const TOPICS = [
  ['/sentry/camera/image_raw/compressed', 'sensor_msgs/CompressedImage',
   (msg) => { store.cameraFrame = 'data:image/jpeg;base64,' + msg.data; }],
  ['/vision/plant_detected', 'sentry_interfaces/PlantDetection',
   (msg) => {
     store.plantDetected = msg.detected;
     store.plantConfidence = msg.confidence;
     store.plantBbox = msg.bbox;
     store.plantAreaRatio = msg.area_ratio;
   }],
  ['/vision/diagnosis', 'sentry_interfaces/Diagnosis',
   (msg) => {
     store.diagnosisCropType = msg.crop_type;
     store.diagnosisDisease = msg.disease_class;
     store.diagnosisConfidence = msg.confidence;
     store.diagnosisProbabilities = msg.probabilities;
   }],
  ['/advisory/action', 'sentry_interfaces/AdvisoryAction',
   (msg) => {
     store.advisoryText = msg.action_text;
     store.advisoryUrgency = msg.urgency_hours;
     store.advisoryFungicide = msg.fungicide_hint;
   }],
  ['/forecast/alert', 'sentry_interfaces/ForecastAlert',
   (msg) => {
     // Append to history, keep last 100
     store.forecastAlerts.push({
       time: new Date().toISOString(),
       active: msg.active,
       alert_type: msg.alert_type,
       probability: msg.probability,
       description: msg.description,
       hours_ahead: msg.hours_ahead,
     });
     if (store.forecastAlerts.length > 100) store.forecastAlerts.shift();
   }],
  ['/fusion/diagnosis', 'sentry_interfaces/FusionResult',
   (msg) => {
     store.fusionResults.push({
       time: new Date().toISOString(),
       risk_score: msg.risk_score,
       alert_level: msg.alert_level,
       mode: msg.mode,
       evidence_chain: msg.evidence_chain,
       lwd_hours: msg.lwd_hours,
       confidence: msg.confidence,
       // Snapshot current state at alert time
       snapshot: {
         frame: store.cameraFrame,
         diagnosisDisease: store.diagnosisDisease,
         diagnosisConfidence: store.diagnosisConfidence,
         advisoryText: store.advisoryText,
         advisoryFungicide: store.advisoryFungicide,
         envAirTemp: store.envAirTemp,
         envAirHumidity: store.envAirHumidity,
         envSoilTemp: store.envSoilTemp,
         envSoilHumidity: store.envSoilHumidity,
         envLeafWetness: store.envLeafWetness,
       }
     });
     if (store.fusionResults.length > 200) store.fusionResults.shift();
   }],
  ['/sensor/environment_fixed', 'sentry_interfaces/Environment',
   (msg) => {
     store.envAirTemp = msg.air_temp;
     store.envAirHumidity = msg.air_humidity;
     store.envCO2 = msg.air_co2;
     store.envSoilTemp = msg.soil_temp;
     store.envSoilHumidity = msg.soil_humidity;
     store.envLeafWetness = msg.leaf_wetness;
     store.envDataSource = msg.data_source;
   }],
  ['/mission/status', 'sentry_interfaces/MissionStatus',
   (msg) => {
     store.missionState = msg.state;
     store.missionProgress = msg.progress;
     store.missionCurrentAction = msg.current_action;
     store.missionPlantsDetected = msg.plants_detected;
     store.missionPlantsAnalyzed = msg.plants_analyzed;
     store.missionCurrentWpIdx = msg.current_wp_idx;
     store.missionTotalWps = msg.total_wps;
     store.missionWaypointLabels = msg.waypoint_labels;
   }],
  ['/sentry/chassis/status', 'sentry_interfaces/ChassisStatus',
   (msg) => {
     store.batteryVoltage = msg.battery_voltage;
     store.leftSpeed = msg.left_speed;
     store.rightSpeed = msg.right_speed;
   }],
];

let subscribers = [];

function subscribeAll() {
  subscribers.forEach(s => s.unsubscribe());
  subscribers = [];
  TOPICS.forEach(([topic, type, cb]) => {
    const t = new ROSLIB.Topic({ ros, name: topic, messageType: type });
    t.subscribe(cb);
    subscribers.push(t);
  });
}

// Service calls
function callSetAutoMode(auto) {
  return new Promise((resolve, reject) => {
    const svc = new ROSLIB.Service({
      ros, name: '/set_auto_mode', serviceType: 'std_srvs/SetBool'
    });
    svc.callService(new ROSLIB.ServiceRequest({ data: auto }), (resp) => {
      if (resp.success) {
        store.mode = auto ? 'AUTO' : 'MANUAL';
        resolve(resp);
      } else {
        reject(new Error(resp.message));
      }
    });
  });
}

function publishCmdVel(linear, angular) {
  const topic = new ROSLIB.Topic({
    ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist'
  });
  topic.publish(new ROSLIB.Message({
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular }
  }));
}

function publishResumeNavigation() {
  const topic = new ROSLIB.Topic({
    ros, name: '/resume_navigation', messageType: 'std_msgs/Bool'
  });
  topic.publish(new ROSLIB.Message({ data: true }));
}

function callSetCropType(cropType) {
  return new Promise((resolve, reject) => {
    const svc = new ROSLIB.Service({
      ros, name: '/set_crop_type', serviceType: 'sentry_interfaces/SetCropType'
    });
    svc.callService(new ROSLIB.ServiceRequest({ crop_type: cropType }), (resp) => {
      if (resp.success) {
        store.cropType = cropType;
        resolve(resp);
      } else {
        reject(new Error(resp.message));
      }
    });
  });
}

// Auto-connect on load
rosConnect();
```

- [ ] **Step 3: 创建 app.js**

```javascript
// app.js — Vue 3 application entry

// All components registered globally before mount
const app = Vue.createApp({
  data() {
    return { store };
  },
  template: '<div>App mounted</div>', // placeholder, HTML template is in index.html
});

// Register all components
app.component('TopBar', TopBar);
app.component('CameraPanel', CameraPanel);
app.component('DetectionCard', DetectionCard);
app.component('DiagnosisCard', DiagnosisCard);
app.component('AdvisoryCard', AdvisoryCard);
app.component('ForecastPanel', ForecastPanel);
app.component('AlertDetailModal', AlertDetailModal);
app.component('EnvDataBar', EnvDataBar);
app.component('Dpad', Dpad);
app.component('CropSelector', CropSelector);
app.component('CruisePanel', CruisePanel);
app.component('StatusBar', StatusBar);
app.component('ControlPanel', ControlPanel);

app.mount('#app');
```

- [ ] **Step 4: 提交**

```bash
git add src/sentry_mission/static_v2/index.html \
        src/sentry_mission/static_v2/app.js \
        src/sentry_mission/static_v2/ros.js
git commit -m "feat: add Vue 3 entry point, rosbridge wrapper, and reactive store"
```

---

### Task 5: 核心显示组件

**Files:**
- Create: `src/sentry_mission/static_v2/components/top-bar.js`
- Create: `src/sentry_mission/static_v2/components/camera-panel.js`
- Create: `src/sentry_mission/static_v2/components/detection-card.js`
- Create: `src/sentry_mission/static_v2/components/diagnosis-card.js`
- Create: `src/sentry_mission/static_v2/components/advisory-card.js`
- Create: `src/sentry_mission/static_v2/components/env-data-bar.js`
- Create: `src/sentry_mission/static_v2/components/status-bar.js`
- Create: `src/sentry_mission/static_v2/style.css`

TopBar、CameraPanel+YOLO叠加、DetectionCard、DiagnosisCard、AdvisoryCard、EnvDataBar、StatusBar 共 7 个组件 + 全局样式。

- [ ] **Step 1: 创建 top-bar.js**

```javascript
const TopBar = {
  template: `
  <header class="top-bar">
    <span class="logo">智农哨兵</span>
    <span class="badge" :class="store.mode === 'AUTO' ? 'badge-auto' : 'badge-manual'">
      {{ store.mode }}
    </span>
    <span class="indicator">
      <span class="dot" :class="store.connected ? 'dot-green' : 'dot-red'"></span>
      {{ store.connected ? 'ROS 在线' : 'ROS 离线' }}
    </span>
    <span class="battery" v-if="store.batteryVoltage !== null">
      🔋 {{ store.batteryVoltage.toFixed(1) }}V
    </span>
    <span class="lora" v-if="store.envDataSource">
      📡 {{ store.envDataSource }}
    </span>
  </header>`
};
```

- [ ] **Step 2: 创建 camera-panel.js（含 Canvas bbox 叠加）**

```javascript
const CameraPanel = {
  template: `
  <div class="camera-panel">
    <h3>实时画面</h3>
    <div class="camera-container" :class="{ 'detected-flash': store.plantDetected }">
      <canvas ref="canvas" width="640" height="480"></canvas>
      <div v-if="!store.cameraFrame" class="camera-placeholder">等待视频流...</div>
      <div v-if="store.plantDetected" class="detection-badge">
        检测到植株 {{ (store.plantConfidence * 100).toFixed(0) }}%
      </div>
    </div>
  </div>`,
  data() { return { image: new Image() }; },
  watch: {
    'store.cameraFrame'(src) {
      if (!src) return;
      this.image.onload = () => this.drawFrame();
      this.image.src = src;
    },
    'store.plantDetected'() { if (this.store.cameraFrame) this.drawFrame(); },
    'store.plantBbox'() { if (this.store.cameraFrame) this.drawFrame(); },
  },
  methods: {
    drawFrame() {
      const canvas = this.$refs.canvas;
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      // Resize canvas to match image aspect ratio
      const iw = this.image.naturalWidth, ih = this.image.naturalHeight;
      canvas.width = iw; canvas.height = ih;
      ctx.drawImage(this.image, 0, 0);
      // Draw bbox if plant detected
      if (this.store.plantDetected && this.store.plantBbox.length === 4) {
        const [x1, y1, x2, y2] = this.store.plantBbox;
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 3;
        ctx.strokeRect(x1 * iw, y1 * ih, (x2 - x1) * iw, (y2 - y1) * ih);
        ctx.fillStyle = '#00ff00';
        ctx.font = '16px monospace';
        ctx.fillText(
          `Plant ${(this.store.plantConfidence * 100).toFixed(0)}%`,
          x1 * iw, y1 * ih - 5
        );
      }
    }
  }
};
```

- [ ] **Step 3: 创建 detection-card.js**

```javascript
const DetectionCard = {
  template: `
  <div class="card">
    <h3>YOLO 植株检测</h3>
    <div v-if="store.plantDetected">
      <span class="value">检测到植株</span>
      <div class="stat">置信度: {{ (store.plantConfidence * 100).toFixed(1) }}%</div>
      <div class="stat">叶片面积比: {{ (store.plantAreaRatio * 100).toFixed(1) }}%</div>
    </div>
    <div v-else class="muted">未检测到植株</div>
  </div>`
};
```

- [ ] **Step 4: 创建 diagnosis-card.js**

```javascript
const DiagnosisCard = {
  template: `
  <div class="card">
    <h3>病害分类 · {{ store.cropType }}</h3>
    <div v-if="store.diagnosisDisease">
      <span class="value disease">{{ store.diagnosisDisease }}</span>
      <div class="stat">置信度: {{ (store.diagnosisConfidence * 100).toFixed(1) }}%</div>
      <div class="probabilities" v-if="store.diagnosisProbabilities.length">
        <div v-for="(p, i) in store.diagnosisProbabilities.slice(0, 3)" class="prob-row">
          <span class="label">{{ i }}</span>
          <span class="bar" :style="{width: (p * 100) + '%'}"></span>
          <span>{{ (p * 100).toFixed(0) }}%</span>
        </div>
      </div>
    </div>
    <div v-else class="muted">等待诊断结果...</div>
  </div>`
};
```

- [ ] **Step 5: 创建 advisory-card.js**

```javascript
const AdvisoryCard = {
  template: `
  <div class="card">
    <h3>农艺建议</h3>
    <div v-if="store.advisoryText">
      <p class="advisory-text">{{ store.advisoryText }}</p>
      <div class="stat" v-if="store.advisoryUrgency">
        建议 {{ store.advisoryUrgency }} 小时内执行
      </div>
      <div class="stat" v-if="store.advisoryFungicide">
        推荐药剂: {{ store.advisoryFungicide }}
      </div>
    </div>
    <div v-else class="muted">等待建议...</div>
  </div>`
};
```

- [ ] **Step 6: 创建 env-data-bar.js**

```javascript
const EnvDataBar = {
  template: `
  <div class="env-bar">
    <h3>固定环境节点</h3>
    <div class="env-grid">
      <div class="env-item">
        <span class="label">气温</span>
        <span class="value">{{ store.envAirTemp !== null ? store.envAirTemp.toFixed(1) + '°C' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">湿度</span>
        <span class="value">{{ store.envAirHumidity !== null ? store.envAirHumidity.toFixed(1) + '%' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">CO₂</span>
        <span class="value">{{ store.envCO2 !== null ? store.envCO2.toFixed(0) + 'ppm' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">土壤温度</span>
        <span class="value">{{ store.envSoilTemp !== null ? store.envSoilTemp.toFixed(1) + '°C' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">土壤湿度</span>
        <span class="value">{{ store.envSoilHumidity !== null ? store.envSoilHumidity.toFixed(1) + '%' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">叶面湿度</span>
        <span class="value">{{ store.envLeafWetness !== null ? store.envLeafWetness.toFixed(1) + '%' : '--' }}</span>
      </div>
    </div>
  </div>`
};
```

- [ ] **Step 7: 创建 status-bar.js**

```javascript
const StatusBar = {
  template: `
  <div class="status-bar">
    <span>状态: <strong>{{ store.missionState }}</strong></span>
    <span v-if="store.missionCurrentAction">| {{ store.missionCurrentAction }}</span>
    <span>| 检测: {{ store.missionPlantsDetected }} | 分析: {{ store.missionPlantsAnalyzed }}</span>
    <span>| 进度: {{ (store.missionProgress * 100).toFixed(0) }}%</span>
  </div>`
};
```

- [ ] **Step 8: 创建 style.css（仪表盘布局）**

```css
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: 'Segoe UI', system-ui, sans-serif; background: #0a0e17; color: #e0e6ed; }

.top-bar {
  display: flex; align-items: center; gap: 16px; padding: 8px 16px;
  background: #131a2b; border-bottom: 1px solid #1e2d45;
}
.top-bar .logo { font-size: 18px; font-weight: bold; color: #4fc3f7; }
.badge { padding: 2px 10px; border-radius: 10px; font-size: 12px; font-weight: bold; }
.badge-auto { background: #1b5e20; color: #a5d6a7; }
.badge-manual { background: #e65100; color: #ffcc80; }
.indicator { display: flex; align-items: center; gap: 6px; font-size: 13px; }
.dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; }
.dot-green { background: #4caf50; }
.dot-red { background: #f44336; }

.main-layout {
  display: grid; grid-template-columns: 1fr 380px; gap: 12px;
  padding: 12px; height: calc(100vh - 140px);
}
.left-panel { display: flex; flex-direction: column; gap: 12px; overflow: hidden; }
.right-panel { display: flex; flex-direction: column; gap: 8px; overflow-y: auto; }

.camera-panel { flex: 1; min-height: 0; }
.camera-container { position: relative; background: #000; border-radius: 8px; overflow: hidden; }
.camera-container canvas { width: 100%; height: auto; display: block; }
.camera-placeholder {
  position: absolute; inset: 0; display: flex; align-items: center;
  justify-content: center; color: #5a6a8a; font-size: 18px;
}
.detected-flash { box-shadow: 0 0 20px rgba(76, 175, 80, 0.5); }
.detection-badge {
  position: absolute; top: 8px; right: 8px; background: rgba(76,175,80,0.85);
  color: #fff; padding: 4px 12px; border-radius: 12px; font-size: 13px; font-weight: bold;
}

.card {
  background: #131a2b; border: 1px solid #1e2d45; border-radius: 8px;
  padding: 12px;
}
.card h3 { font-size: 14px; color: #8fa4c4; margin-bottom: 8px; }
.value { font-size: 20px; font-weight: bold; }
.value.disease { color: #ef5350; }
.stat { font-size: 13px; color: #8fa4c4; margin-top: 4px; }
.muted { color: #5a6a8a; font-size: 13px; }
.probabilities { margin-top: 8px; }
.prob-row { display: flex; align-items: center; gap: 8px; font-size: 12px; margin: 2px 0; }
.prob-row .bar { height: 5px; background: #4fc3f7; border-radius: 3px; min-width: 2px; }
.advisory-text { font-size: 14px; line-height: 1.5; }

.env-bar { background: #131a2b; border: 1px solid #1e2d45; border-radius: 8px; padding: 10px 14px; }
.env-bar h3 { font-size: 13px; color: #8fa4c4; margin-bottom: 6px; }
.env-grid { display: flex; gap: 16px; flex-wrap: wrap; }
.env-item { display: flex; flex-direction: column; align-items: center; }
.env-item .label { font-size: 11px; color: #5a6a8a; }
.env-item .value { font-size: 15px; }

.control-panel {
  display: flex; gap: 16px; padding: 8px 16px;
  background: #131a2b; border-top: 1px solid #1e2d45; align-items: center;
}

.dpad { display: grid; grid-template-columns: 60px 60px 60px; grid-template-rows: 60px 60px 60px; gap: 4px; }
.dpad button {
  border: 1px solid #2a3f5f; background: #1a2740; color: #e0e6ed;
  border-radius: 6px; cursor: pointer; font-size: 18px; user-select: none;
}
.dpad button:active { background: #2a4a70; }
.btn-up { grid-column: 2; }
.btn-left { grid-column: 1; grid-row: 2; }
.btn-center { grid-column: 2; grid-row: 2; font-size: 12px; }
.btn-right { grid-column: 3; grid-row: 2; }
.btn-down { grid-column: 2; grid-row: 3; }
.btn-stop {
  grid-column: 1 / 4; background: #b71c1c; border-color: #c62828;
  color: #fff; font-weight: bold; font-size: 14px; height: 32px;
}

.crop-selector select {
  background: #1a2740; color: #e0e6ed; border: 1px solid #2a3f5f;
  padding: 6px 12px; border-radius: 6px; font-size: 14px;
}

.cruise-panel { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
.cruise-panel .wp-list { display: flex; gap: 4px; flex-wrap: wrap; max-width: 300px; }
.cruise-panel .wp-item {
  font-size: 11px; padding: 2px 8px; border-radius: 4px; cursor: pointer;
  background: #1a2740; border: 1px solid #2a3f5f;
}
.cruise-panel .wp-item.active { background: #1b5e20; border-color: #4caf50; }
.cruise-panel .wp-item.done { background: #263238; border-color: #37474f; color: #546e7a; }
.btn { padding: 8px 18px; border-radius: 6px; font-size: 14px; border: none; cursor: pointer; font-weight: bold; }
.btn-go { background: #2e7d32; color: #fff; }
.btn-pause { background: #e65100; color: #fff; }
.btn-resume { background: #1565c0; color: #fff; }

.status-bar {
  padding: 4px 16px; font-size: 12px; color: #5a6a8a;
  background: #0a0e17; border-top: 1px solid #1e2d45;
  display: flex; gap: 12px;
}

/* Modal */
.modal-overlay {
  position: fixed; inset: 0; background: rgba(0,0,0,0.7);
  display: flex; align-items: center; justify-content: center; z-index: 1000;
}
.modal {
  background: #131a2b; border: 1px solid #2a3f5f; border-radius: 12px;
  padding: 24px; max-width: 700px; width: 90%; max-height: 85vh; overflow-y: auto;
}
.modal h2 { color: #4fc3f7; margin-bottom: 16px; }
.modal .snapshot-row { display: flex; gap: 16px; margin-bottom: 16px; }
.modal .snapshot-img { flex: 1; }
.modal .snapshot-img img { width: 100%; border-radius: 8px; }
.modal .snapshot-env { flex: 1; background: #1a2740; padding: 12px; border-radius: 8px; }
.modal .evidence { background: #1a2740; padding: 12px; border-radius: 8px; margin: 12px 0; }
.modal .evidence li { margin: 4px 0; font-size: 13px; }
.modal .actions { display: flex; gap: 8px; justify-content: flex-end; margin-top: 16px; }

.alert-level-NORMAL { border-left: 3px solid #4caf50; }
.alert-level-SUSPICION { border-left: 3px solid #ff9800; }
.alert-level-WARNING { border-left: 3px solid #ff9800; }
.alert-level-CRITICAL { border-left: 3px solid #f44336; }
```

- [ ] **Step 9: 提交**

```bash
git add src/sentry_mission/static_v2/components/top-bar.js \
        src/sentry_mission/static_v2/components/camera-panel.js \
        src/sentry_mission/static_v2/components/detection-card.js \
        src/sentry_mission/static_v2/components/diagnosis-card.js \
        src/sentry_mission/static_v2/components/advisory-card.js \
        src/sentry_mission/static_v2/components/env-data-bar.js \
        src/sentry_mission/static_v2/components/status-bar.js \
        src/sentry_mission/static_v2/style.css
git commit -m "feat: add core display components (camera, detection, diagnosis, advisory, env, status)"
```

---

### Task 6: 预警与追溯组件

**Files:**
- Create: `src/sentry_mission/static_v2/components/forecast-panel.js`
- Create: `src/sentry_mission/static_v2/components/alert-detail-modal.js`

24h 预警趋势折线图 + 时间轴列表 + 详情弹窗（图像快照 + 环境快照 + 证据链 + 处置措施）。

- [ ] **Step 1: 创建 forecast-panel.js**

```javascript
const ForecastPanel = {
  template: `
  <div class="card">
    <h3>预警趋势</h3>
    <canvas ref="chart" height="160"></canvas>
    <div class="alert-list">
      <div v-for="(alert, i) in recentAlerts" :key="i"
           class="alert-row" :class="'alert-level-' + (alert.alert_type || 'NORMAL')"
           @click="store.selectedAlert = alert">
        <span class="alert-time">{{ formatTime(alert.time) }}</span>
        <span class="alert-desc">{{ alert.description || alert.alert_type }}</span>
        <span class="alert-risk" v-if="alert.probability">
          {{ (alert.probability * 100).toFixed(0) }}%
        </span>
      </div>
      <div v-if="recentAlerts.length === 0" class="muted">暂无预警记录</div>
    </div>
  </div>`,
  computed: {
    recentAlerts() {
      return this.store.forecastAlerts.slice(-20).reverse();
    }
  },
  methods: {
    formatTime(iso) {
      const d = new Date(iso);
      return d.toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit' });
    },
    renderChart() {
      // Use Chart.js to render risk trend from forecastAlerts
      const ctx = this.$refs.chart;
      if (!ctx) return;
      if (this._chart) this._chart.destroy();
      const data = this.store.forecastAlerts.slice(-50);
      if (data.length === 0) return;
      this._chart = new Chart(ctx, {
        type: 'line',
        data: {
          labels: data.map(a => this.formatTime(a.time)),
          datasets: [{
            label: '风险值',
            data: data.map(a => a.probability || 0),
            borderColor: '#4fc3f7',
            backgroundColor: 'rgba(79,195,247,0.1)',
            fill: true,
            pointRadius: 3,
            pointBackgroundColor: data.map(a => {
              const level = a.alert_type || 'NORMAL';
              return { NORMAL: '#4caf50', SUSPICION: '#ff9800', WARNING: '#ff9800', CRITICAL: '#f44336' }[level] || '#4caf50';
            }),
            tension: 0.3,
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          plugins: { legend: { display: false } },
          scales: {
            y: { min: 0, max: 1, ticks: { color: '#5a6a8a' }, grid: { color: '#1e2d45' } },
            x: { ticks: { color: '#5a6a8a', maxTicksLimit: 8 }, grid: { display: false } }
          }
        }
      });
    }
  },
  watch: {
    'store.forecastAlerts.length'() { this.$nextTick(() => this.renderChart()); }
  },
  mounted() { this.$nextTick(() => this.renderChart()); }
};
```

- [ ] **Step 2: 创建 alert-detail-modal.js**

```javascript
const AlertDetailModal = {
  template: `
  <div class="modal-overlay" v-if="store.selectedAlert" @click.self="store.selectedAlert = null">
    <div class="modal">
      <h2>预警详情 — {{ formatTime(store.selectedAlert.time) }}</h2>
      <div class="snapshot-row">
        <div class="snapshot-img">
          <h4>现场快照</h4>
          <img v-if="store.selectedAlert.snapshot?.frame"
               :src="store.selectedAlert.snapshot.frame" alt="现场快照">
          <div v-else class="muted">无图像快照</div>
        </div>
        <div class="snapshot-env">
          <h4>环境快照</h4>
          <div class="stat">气温: {{ store.selectedAlert.snapshot?.envAirTemp?.toFixed(1) || '--' }}°C</div>
          <div class="stat">湿度: {{ store.selectedAlert.snapshot?.envAirHumidity?.toFixed(1) || '--' }}%RH</div>
          <div class="stat">叶面湿润: {{ store.selectedAlert.snapshot?.envLeafWetness?.toFixed(1) || store.selectedAlert.lwd_hours?.toFixed(1) || '--' }}h</div>
          <div class="stat">土壤温度: {{ store.selectedAlert.snapshot?.envSoilTemp?.toFixed(1) || '--' }}°C</div>
          <div class="stat">土壤湿度: {{ store.selectedAlert.snapshot?.envSoilHumidity?.toFixed(1) || '--' }}%</div>
        </div>
      </div>
      <div class="card">
        <h4>农艺建议</h4>
        <p>{{ store.selectedAlert.snapshot?.advisoryText || store.advisoryText || '暂无建议' }}</p>
        <div v-if="store.selectedAlert.snapshot?.advisoryFungicide || store.advisoryFungicide">
          药剂: {{ store.selectedAlert.snapshot?.advisoryFungicide || store.advisoryFungicide }}
        </div>
      </div>
      <div class="evidence" v-if="store.selectedAlert.evidence_chain?.length">
        <h4>决策依据</h4>
        <ul>
          <li v-for="(e, i) in store.selectedAlert.evidence_chain" :key="i">{{ e }}</li>
        </ul>
      </div>
      <div class="modal-stats">
        <span>风险值: {{ (store.selectedAlert.risk_score * 100).toFixed(0) }}%</span>
        <span>| 置信度: {{ (store.selectedAlert.confidence * 100).toFixed(0) }}%</span>
        <span>| 模式: {{ store.selectedAlert.mode }}</span>
      </div>
      <div class="actions">
        <button class="btn btn-pause" @click="store.selectedAlert = null">关闭</button>
      </div>
    </div>
  </div>`,
  methods: {
    formatTime(iso) {
      return new Date(iso).toLocaleString('zh-CN');
    }
  }
};
```

- [ ] **Step 3: 提交**

```bash
git add src/sentry_mission/static_v2/components/forecast-panel.js \
        src/sentry_mission/static_v2/components/alert-detail-modal.js
git commit -m "feat: add forecast trend chart and explainable alert detail modal"
```

---

### Task 7: 控制组件 — Dpad + CropSelector + CruisePanel

**Files:**
- Create: `src/sentry_mission/static_v2/components/dpad.js`
- Create: `src/sentry_mission/static_v2/components/crop-selector.js`
- Create: `src/sentry_mission/static_v2/components/cruise-panel.js`
- Create: `src/sentry_mission/static_v2/components/control-panel.js`

方向键遥控（mousedown/touchstart 持续发送，mouseup/touchend 归零）、作物切换（确认弹窗 + 加载动画）、巡航控制（航点勾选 + 启动/暂停/恢复）。

- [ ] **Step 1: 创建 dpad.js**

```javascript
const Dpad = {
  template: `
  <div class="dpad">
    <button class="btn-up" @mousedown="move(0.3, 0)" @mouseup="stop" @touchstart.prevent="move(0.3, 0)" @touchend="stop">▲</button>
    <button class="btn-left" @mousedown="move(0, 0.5)" @mouseup="stop" @touchstart.prevent="move(0, 0.5)" @touchend="stop">◀</button>
    <button class="btn-center" @click="stop">⏹</button>
    <button class="btn-right" @mousedown="move(0, -0.5)" @mouseup="stop" @touchstart.prevent="move(0, -0.5)" @touchend="stop">▶</button>
    <button class="btn-down" @mousedown="move(-0.3, 0)" @mouseup="stop" @touchstart.prevent="move(-0.3, 0)" @touchend="stop">▼</button>
    <button class="btn-stop" @click="emergencyStop">急停</button>
  </div>`,
  data() { return { linearScale: 0.3, angularScale: 0.5, interval: null }; },
  methods: {
    move(lin, ang) {
      publishCmdVel(lin * this.linearScale, ang * this.angularScale);
      // Continuous send while held
      clearInterval(this.interval);
      this.interval = setInterval(() => {
        publishCmdVel(lin * this.linearScale, ang * this.angularScale);
      }, 100);
    },
    stop() {
      clearInterval(this.interval);
      publishCmdVel(0, 0);
    },
    emergencyStop() {
      clearInterval(this.interval);
      publishCmdVel(0, 0);
      callSetAutoMode(false);
    }
  },
  beforeUnmount() { clearInterval(this.interval); }
};
```

- [ ] **Step 2: 创建 crop-selector.js**

```javascript
const CropSelector = {
  template: `
  <div class="crop-selector">
    <label>作物:</label>
    <select :value="store.cropType" @change="onChange">
      <option value="tomato">番茄</option>
      <option value="wheat">小麦</option>
      <option value="strawberry">草莓</option>
    </select>
    <span v-if="switching" class="switching">切换中...</span>
  </div>`,
  data() { return { switching: false }; },
  methods: {
    async onChange(e) {
      const crop = e.target.value;
      if (crop === this.store.cropType) return;
      if (!confirm(`切换作物类型到 ${crop} 将重启相关节点，约 5-10 秒不可用。确定？`)) {
        e.target.value = this.store.cropType;
        return;
      }
      this.switching = true;
      try {
        await callSetCropType(crop);
      } catch (err) {
        alert('切换失败: ' + err.message);
      }
      this.switching = false;
    }
  }
};
```

- [ ] **Step 3: 创建 cruise-panel.js**

```javascript
const CruisePanel = {
  template: `
  <div class="cruise-panel">
    <div class="wp-list">
      <div v-for="(label, i) in store.missionWaypointLabels" :key="i"
           class="wp-item"
           :class="{ active: i === store.missionCurrentWpIdx, done: i < store.missionCurrentWpIdx }"
           @click="toggleWp(i)">
        {{ label }}
      </div>
      <div v-if="store.missionWaypointLabels.length === 0" class="muted">无航点数据</div>
    </div>
    <button v-if="store.mode !== 'AUTO'" class="btn btn-go" @click="startCruise">启动巡航</button>
    <button v-if="store.mode === 'AUTO'" class="btn btn-pause" @click="pauseCruise">暂停</button>
    <button v-if="store.mode === 'MANUAL'" class="btn btn-resume" @click="resumeCruise">恢复</button>
  </div>`,
  methods: {
    startCruise() { callSetAutoMode(true); },
    pauseCruise() { callSetAutoMode(false); },
    resumeCruise() { publishResumeNavigation(); },
    toggleWp(idx) {
      // Store selected waypoints for future use
      // Currently just visual, backend needs enhancement to support WP skipping
    }
  }
};
```

- [ ] **Step 4: 创建 control-panel.js（容器组件）**

```javascript
const ControlPanel = {
  template: `
  <div class="control-panel">
    <dpad></dpad>
    <div style="border-left:1px solid #1e2d45;height:100%;margin:0 8px"></div>
    <crop-selector></crop-selector>
    <div style="flex:1"></div>
    <cruise-panel></cruise-panel>
  </div>`
};
```

- [ ] **Step 5: 提交**

```bash
git add src/sentry_mission/static_v2/components/dpad.js \
        src/sentry_mission/static_v2/components/crop-selector.js \
        src/sentry_mission/static_v2/components/cruise-panel.js \
        src/sentry_mission/static_v2/components/control-panel.js
git commit -m "feat: add control components (D-pad, crop selector, cruise panel)"
```

---

### Task 8: 板端部署与集成测试

**Files:**
- Modify: `src/sentry_bringup/launch/sentry_v2.launch.py` — 添加 rosbridge_server 节点

在板端安装依赖、构建、部署、验证全链路。

- [ ] **Step 1: 在启动文件中添加 rosbridge_server**

检查 `sentry_v2.launch.py`，在 launch 描述中添加:

```python
Node(
    package='rosbridge_server',
    executable='rosbridge_websocket',
    name='rosbridge_websocket',
    parameters=[{'port': 9090}],
    output='screen',
),
```

并在 CMakeLists.txt / package.xml 中添加 `rosbridge_server` 作为执行依赖。

- [ ] **Step 2: 推送代码到远程仓库**

```bash
git push origin feat/visual-pipeline-integration
```

- [ ] **Step 3: 板端拉取并构建**

```bash
ssh rdk "cd ~/dev_ws && git pull && bash -l -c 'source /opt/ros/humble/setup.bash && colcon build --packages-select sentry_interfaces sentry_mission sentry_bringup'"
```

- [ ] **Step 4: 安装 rosbridge_server**

```bash
ssh rdk "sudo apt install -y ros-humble-rosbridge-server ros-humble-image-transport-plugins"
```

- [ ] **Step 5: 启动系统并测试**

```bash
# On board: launch the system
ssh rdk "bash -l -c 'source ~/dev_ws/install/setup.bash && ros2 launch sentry_bringup sentry_v2.launch.py'"

# Open browser: http://<rdk-ip>:5000/v2
# Verify: ROS connected indicator green, camera stream, data panels populated
```

- [ ] **Step 6: 测试清单**

| 测试项 | 验证方法 |
|---|---|
| 老页面不受影响 | 访问 `http://<rdk>:5000/` 显示原遥控页面 |
| 新仪表盘加载 | 访问 `http://<rdk>:5000/v2` 显示 Vue 3 仪表盘 |
| ROS 连接 | 顶栏显示绿点 + "ROS 在线" |
| 实时画面 | 左侧 camera panel 显示摄像头画面 |
| YOLO 检测叠加 | 检测到植株时画面出现绿色 bbox + 闪烁 |
| 病害分类卡片 | 显示病害名和置信度 |
| 农艺建议卡片 | 显示建议文本和药剂 |
| 预警趋势图 | Chart.js 折线图渲染 |
| 预警详情弹窗 | 点击预警记录 → 弹窗显示快照 + 证据链 |
| 环境数据条 | 显示固定节点的温湿度等数据 |
| 方向键控车 | 按方向键 → 底盘移动，松开停止 |
| 急停 | 按急停键 → 底盘立即停止 |
| 作物切换 | 选择作物 → 确认弹窗 → 节点重启 |
| 启动巡航 | 点启动 → 航点开始导航 |
| 暂停巡航 | 点暂停 → 底盘停止 |
| 恢复巡航 | 点恢复 → 从断点继续 |

---

## 实施顺序

```
Task 1 (MissionStatus) ──┐
                         ├──> Task 4 (入口 + ros.js) ──> Task 5 (核心组件)
Task 2 (SetCropType) ────┤                                    │
                         │                                    ▼
Task 3 (/v2 路由) ───────┘                              Task 6 (预警+追溯)
                                                            │
                                                            ▼
                                                      Task 7 (控制组件)
                                                            │
                                                            ▼
                                                      Task 8 (板端部署测试)
```

## 已知限制

1. **航点动态跳过**：前端 UI 预留了 waypoint checkbox，但 `mission_control_node` 暂不支持按索引跳过航点。当前航点列表仅为展示用途。
2. **后端历史预警持久化**：`data_logger_node` 的 JSON 落盘存储未在本次实施范围内，前端通过 `ros.js` 的 `fusionResults` 数组做客户端缓存（保留最近 200 条）。

