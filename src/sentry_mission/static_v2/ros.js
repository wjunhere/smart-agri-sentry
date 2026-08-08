// ros.js — roslibjs connection wrapper and reactive state store
//
// Local dev mode（本地开发模式）:
//   本地起静态服务打开页面，用 ?car=<小车IP> 指定小车，例如
//     python -m http.server 8899  →  http://localhost:8899/?car=10.66.175.213
//   IP 会存入 localStorage，之后不带参数也会记住。
//   板端托管（http://<car>:5000/）时一切保持同源，行为不变。
const CAR_HOST = (() => {
  const q = new URLSearchParams(window.location.search).get('car');
  if (q) { localStorage.setItem('sentry_car_ip', q); return q; }
  return localStorage.getItem('sentry_car_ip') || window.location.hostname;
})();
const API_BASE = (CAR_HOST === window.location.hostname) ? '' : ('http://' + CAR_HOST + ':5000');
// 把后端相对路径（如 snapshot_url）补全为完整 URL
function apiFullUrl(path) {
  if (!path || path.startsWith('http')) return path;
  return API_BASE + path;
}

const ROS_CONFIG = {
  url: 'ws://' + CAR_HOST + ':9090'
};

// Reactive global state — on window so Vue templates can access it
window.store = Vue.reactive({
  connected: false,
  cameraFrame: null,
  plantDetected: false,
  plantConfidence: 0,
  plantBbox: [0, 0, 0, 0],
  plantAreaRatio: 0,
  diagnosisCropType: '',
  diagnosisDisease: '',
  diagnosisConfidence: 0,
  diagnosisProbabilities: [],
  _diagBuf: [],        // temporal smoothing buffer
  _diagBufSize: 10,    // buffer last 10 frames
  advisoryText: '',
  advisoryUrgency: 0,
  advisoryFungicide: '',
  forecastAlerts: [],
  weatherDays: [],
  weatherDisasterAlerts: [],
  weatherStale: false,
  weatherLat: null,
  weatherLon: null,
  fusionResults: [],
  weatherDays: [],
  weatherHours: [],
  weatherDisasterAlerts: [],
  weatherStale: false,
  weatherAutoFetch: true,
  weatherCity: '',
  weatherLat: 39.9,
  weatherLon: 116.4,
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
  envDataSource: null,
  missionState: 'IDLE',
  missionProgress: 0,
  missionCurrentAction: '',
  missionPlantsDetected: 0,
  missionPlantsAnalyzed: 0,
  missionCurrentWpIdx: 0,
  missionTotalWps: 0,
  missionWaypointLabels: [],
  batteryVoltage: null,
  leftSpeed: 0,
  rightSpeed: 0,
  mode: 'MANUAL',
  cropType: 'tomato',
  selectedAlert: null,
  showWpEditor: false,
  stackStarting: false,
  stackPreheating: false,
  stackShuttingDown: false,
  stackReady: false,
  visionStarting: false,
  visionStopping: false,
  cameraReady: false,
  inferenceStarting: false,
  inferenceStopping: false,
  inferenceReady: false,
  cameraCaptureBusy: false,
  cruiseSpeed: 0.18,
  cruiseSpeedBusy: false,
  visionInferenceMode: 'triggered',
  visionInferenceModeBusy: false,
  fixedPointStops: [],
  fixedPointStopsBusy: false,
  _rawWaypoints: [],
  messageUnread: 0,
  messageBatches: [],
  showMessages: false,
  showSettings: false,
  settings: {
    low_light_enhancement: null,
    detection_confidence: null,
    servo_start_side: null,
  },
  settingsBusy: false,
  settingsMsg: '',
});
const store = window.store;  // local alias for internal use in this file

function missionStateToMode(state) {
  const autoStates = new Set([
    'PATROL',
    'OBSTACLE_STOP',
    'OBSTACLE_BACKUP',
    'OBSTACLE_TURN',
    'OBSTACLE_ARC_DRIVE',
    'OBSTACLE_TURN_BACK',
    'OBSTACLE_REJOIN_FORWARD',
    'AVOIDING',
    'ANALYZING',
    'ACTION',
    'RESUME',
  ]);
  return autoStates.has(state) ? 'AUTO' : 'MANUAL';
}

let ros = null;
let lastRealEnvTime = 0;  // real /sensor/environment_fixed overrides sim

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

// Camera frames: always render only the LATEST frame. After sleep/background
// throttling, rosbridge delivers a backlog of queued frames; decoding every
// one causes a "fast-forward replay". Coalesce bursts via requestAnimationFrame
// so stale frames are skipped and the view jumps straight to live.
let _latestFrameSrc = null;
let _frameFlushScheduled = false;

// Topic definitions: [topic_name, message_type, callback]
const TOPICS = [
  ['/out/compressed', 'sensor_msgs/CompressedImage',
   (msg) => {
     _latestFrameSrc = 'data:image/jpeg;base64,' + msg.data;
     if (!_frameFlushScheduled) {
       _frameFlushScheduled = true;
       requestAnimationFrame(() => {
         _frameFlushScheduled = false;
         store.cameraFrame = _latestFrameSrc;
       });
     }
   }],
  ['/vision/plant_detected', 'sentry_interfaces/PlantDetection',
   (msg) => {
     store.plantDetected = msg.detected;
     store.plantConfidence = msg.confidence;
     store.plantBbox = msg.bbox;
     store.plantAreaRatio = msg.area_ratio;
   }],
  ['/vision/diagnosis', 'sentry_interfaces/Diagnosis',
   (msg) => {
     if (msg.class_id === 254) store._diagBuf = [];
     // Temporal smoothing: buffer N recent predictions, show majority class
     store._diagBuf.push({ cls: msg.disease_class, conf: msg.confidence, probs: msg.probabilities });
     if (store._diagBuf.length > store._diagBufSize) store._diagBuf.shift();

     const counts = {};
     let maxCls = msg.disease_class, maxCnt = 0;
     store._diagBuf.forEach(d => {
       counts[d.cls] = (counts[d.cls] || 0) + 1;
       if (counts[d.cls] > maxCnt) { maxCnt = counts[d.cls]; maxCls = d.cls; }
     });

     const matching = store._diagBuf.filter(d => d.cls === maxCls);
     const avgConf = matching.reduce((s, d) => s + d.conf, 0) / matching.length;
     const avgProbs = msg.probabilities.length
       ? store._diagBuf[0].probs.map((_, i) =>
           store._diagBuf.reduce((s, d) => s + (d.probs[i] || 0), 0) / store._diagBuf.length)
       : [];

     store.diagnosisCropType = msg.crop_type;
     store.diagnosisDisease = maxCls;
     store.diagnosisConfidence = avgConf;
     store.diagnosisProbabilities = avgProbs;

     // Mock mode: real inference runs unchanged; only the displayed class is
     // forced to the selected disease with confidence jitter in [0.80, 0.90].
     if (store.mockDiagnosisMode !== 'real') {
       const mockCls = store.mockDiagnosisMode;
       const conf = 0.80 + Math.random() * 0.10;
       const idx = TOMATO_DISEASE_CLASSES.indexOf(mockCls);
       store.diagnosisDisease = mockCls;
       store.diagnosisConfidence = conf;
       if (avgProbs.length && idx >= 0 && idx < avgProbs.length) {
         const restSum = avgProbs.reduce((s, p, i) => i === idx ? s : s + p, 0);
         store.diagnosisProbabilities = avgProbs.map((p, i) => {
           if (i === idx) return conf;
           const share = restSum > 0 ? p / restSum : 1 / (avgProbs.length - 1);
           return share * (1 - conf);
         });
       }
     }
   }],
  // === MOCK START: advisory (remove after test) ===
  ['/advisory/action', 'sentry_interfaces/AdvisoryAction',
   (msg) => { /* real data ignored; injectMock handles advisory */ }],
  // === MOCK END ===
  // === MOCK START: forecast (remove after test) ===
  ['/forecast/alert', 'sentry_interfaces/ForecastAlert',
   (msg) => { /* real data ignored; injectMock handles forecast */ }],
  // === MOCK END ===
  ['/weather/forecast', 'sentry_interfaces/WeatherForecast',
   (msg) => {
     store.weatherDays = msg.days || [];
     store.weatherHours = msg.hours || [];
     store.weatherDisasterAlerts = msg.disaster_alerts || [];
     store.weatherStale = msg.stale;
     store.weatherCity = msg.city;
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
     lastRealEnvTime = Date.now();
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
     store.mode = missionStateToMode(msg.state);
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

// Poll /status until cond(data) holds (used for async stack operations
// that return 202 and run in the background on the board).
function waitForStackState(cond, timeoutMs = 300000) {
  const t0 = Date.now();
  const tick = () => refreshStackStatus().then(data => {
    if (data && cond(data)) return data;
    if (Date.now() - t0 > timeoutMs) throw new Error('等待栈状态超时');
    return new Promise(r => setTimeout(r, 2000)).then(tick);
  });
  return tick();
}

function callStackStart() {
  store.stackStarting = true;
  return fetch(API_BASE + '/stack/start', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (resp.status === 202) {
        // Slow path: stack is (re)starting in background; wait for AUTO.
        const finalData = await waitForStackState(
          d => d.mode === 'AUTO' || (!d.stack_busy && !d.stack_operation));
        if (finalData.mode !== 'AUTO') {
          throw new Error('栈启动失败，请查看板端日志');
        }
        store.mode = 'AUTO';
        store.stackReady = Boolean(finalData.stack_ready);
        return finalData;
      }
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to start robot stack');
      }
      store.mode = 'AUTO';
      store.stackReady = Boolean(data.stack_ready);
      return data;
    })
    .finally(() => { store.stackStarting = false; });
}

function callStackStop() {
  store.stackStarting = false;
  return fetch(API_BASE + '/stack/stop', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to stop cruise');
      }
      store.mode = 'MANUAL';
      // 栈保持常驻，stack_ready 仍为 true
      store.stackReady = Boolean(data.stack_ready);
      return data;
    });
}

function callStackPreheat() {
  store.stackPreheating = true;
  return fetch(API_BASE + '/stack/preheat', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (resp.status === 202) {
        const finalData = await waitForStackState(
          d => !d.stack_busy && !d.stack_operation);
        if (!finalData.stack_ready) {
          throw new Error('预热失败，请查看板端日志');
        }
        store.mode = 'MANUAL';
        store.stackReady = true;
        return finalData;
      }
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to preheat robot stack');
      }
      store.mode = 'MANUAL';
      store.stackReady = Boolean(data.stack_ready);
      return data;
    })
    .finally(() => { store.stackPreheating = false; });
}

function callStackShutdown() {
  store.stackShuttingDown = true;
  return fetch(API_BASE + '/stack/shutdown', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (resp.status === 202) {
        const finalData = await waitForStackState(
          d => !d.stack_busy && !d.stack_operation);
        if (finalData.stack_ready) {
          throw new Error('结束栈失败，请查看板端日志');
        }
        store.stackReady = false;
        store.mode = 'MANUAL';
        return finalData;
      }
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to shut down robot stack');
      }
      store.stackReady = false;
      return data;
    })
    .finally(() => { store.stackShuttingDown = false; });
}

function callVisionStart() {
  store.visionStarting = true;
  return fetch(API_BASE + '/vision/start', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to start camera stack');
      }
      store.cameraReady = Boolean(data.camera_ready);
      return data;
    })
    .finally(() => { store.visionStarting = false; });
}

function callInferenceStart() {
  store.inferenceStarting = true;
  return fetch(API_BASE + '/inference/start', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to start inference stack');
      }
      store.inferenceReady = Boolean(data.inference_ready);
      return data;
    })
    .finally(() => { store.inferenceStarting = false; });
}

function callVisionStop() {
  store.visionStopping = true;
  return fetch(API_BASE + '/vision/stop', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to stop camera stack');
      }
      store.cameraReady = false;
      store.cameraFrame = null;
      return data;
    })
    .finally(() => { store.visionStopping = false; });
}

function callInferenceStop() {
  store.inferenceStopping = true;
  return fetch(API_BASE + '/inference/stop', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to stop inference stack');
      }
      store.inferenceReady = false;
      return data;
    })
    .finally(() => { store.inferenceStopping = false; });
}

function callCaptureImage() {
  store.cameraCaptureBusy = true;
  return fetch(API_BASE + '/camera/capture', { method: 'POST' })
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to capture camera image');
      }
      console.info('[camera] captured:', data.filename);
      return data;
    })
    .finally(() => { store.cameraCaptureBusy = false; });
}

function fetchSettings() {
  store.settingsBusy = true;
  return fetch(API_BASE + '/api/settings')
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      Object.assign(store.settings, data);
      return data;
    })
    .finally(() => { store.settingsBusy = false; });
}

function updateSetting(key, value) {
  store.settingsBusy = true;
  store.settingsMsg = '';
  return fetch(API_BASE + '/api/settings', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ [key]: value }),
  }).then(async (resp) => {
    const data = await resp.json().catch(() => ({}));
    const result = data.results && data.results[key];
    if (!resp.ok || result !== 'ok') {
      throw new Error(result || '设置失败');
    }
    store.settings[key] = value;
    store.settingsMsg = '已生效';
    return data;
  }).catch((err) => {
    store.settingsMsg = String(err.message || err);
    throw err;
  }).finally(() => { store.settingsBusy = false; });
}

function callSetCruiseSpeed(speed) {
  store.cruiseSpeedBusy = true;
  return fetch(API_BASE + '/cruise-speed', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ speed: Number(speed) }),
  }).then(async (resp) => {
    const data = await resp.json().catch(() => ({}));
    if (!resp.ok || data.status !== 'ok') {
      throw new Error(data.message || 'Failed to update cruise speed');
    }
    store.cruiseSpeed = Number(data.speed);
    return data;
  }).finally(() => { store.cruiseSpeedBusy = false; });
}
function callGetWaypoints() {
  return fetch(API_BASE + '/waypoints')
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to load waypoints');
      }
      store._rawWaypoints = data.waypoints || [];
      store.missionWaypointLabels = store._rawWaypoints.map((wp, i) =>
        `WP${i}: (${Number(wp.x).toFixed(1)}, ${Number(wp.y).toFixed(1)})`
      );
      store.missionTotalWps = store._rawWaypoints.length;
      return store._rawWaypoints;
    });
}

function callSaveWaypoints(waypoints) {
  return fetch(API_BASE + '/waypoints', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ waypoints })
  }).then(async (resp) => {
    const data = await resp.json().catch(() => ({}));
    if (!resp.ok || data.status !== 'ok') {
      throw new Error(data.message || 'Failed to save waypoints');
    }
    store._rawWaypoints = data.waypoints || waypoints;
    store.missionWaypointLabels = store._rawWaypoints.map((wp, i) =>
      `WP${i}: (${Number(wp.x).toFixed(1)}, ${Number(wp.y).toFixed(1)})`
    );
    store.missionTotalWps = store._rawWaypoints.length;
    return store._rawWaypoints;
  });
}

const TOMATO_DISEASE_CLASSES = [
  'late_blight',
  'healthy',
  'early_blight',
  'bacterial_spot',
  'leaf_mold',
  'septoria_leaf_spot',
  'tomato_yellow_leaf_curl_virus',
];

function normalizeFixedPointStop(stop) {
  return {
    x: Number(stop.x),
    y: Number(stop.y),
    radius: Number(stop.radius ?? 0.20),
    disease_class: String(stop.disease_class || 'healthy'),
  };
}

function fetchFixedPointStops() {
  return fetch(API_BASE + '/fixed-point-stops')
    .then(async (resp) => {
      const data = await resp.json().catch(() => ({}));
      if (!resp.ok || data.status !== 'ok') {
        throw new Error(data.message || 'Failed to load fixed-point stops');
      }
      store.fixedPointStops = (data.fixed_point_stops || []).map(normalizeFixedPointStop);
      return store.fixedPointStops;
    });
}

store.fixedPointDiseaseClasses = TOMATO_DISEASE_CLASSES;
store.addFixedPointStop = function() {
  store.fixedPointStops.push({
    x: 0,
    y: 0,
    radius: 0.20,
    disease_class: 'healthy',
  });
};
store.removeFixedPointStop = function(index) {
  store.fixedPointStops.splice(index, 1);
};
store.saveFixedPointStops = async function() {
  store.fixedPointStopsBusy = true;
  try {
    const fixed_point_stops = store.fixedPointStops.map(normalizeFixedPointStop);
    const resp = await fetch(API_BASE + '/fixed-point-stops', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ fixed_point_stops }),
    });
    const data = await resp.json().catch(() => ({}));
    if (!resp.ok || data.status !== 'ok') {
      throw new Error(data.message || 'Failed to save fixed-point stops');
    }
    store.fixedPointStops = (data.fixed_point_stops || []).map(normalizeFixedPointStop);
    return data;
  } finally {
    store.fixedPointStopsBusy = false;
  }
};

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
  // Goes through the always-on gateway (hot-reloads the diagnosis model via
  // the latched /vision/crop_type topic); works without mission_control.
  return fetch(API_BASE + '/crop_type', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ crop_type: cropType })
  }).then(async (resp) => {
    const data = await resp.json().catch(() => ({}));
    if (resp.ok && data.status === 'ok') {
      store.cropType = cropType;
      return data;
    }
    throw new Error(data.message || `HTTP ${resp.status}`);
  });
}

// ── Mock data for local testing (inject before ROS connect) ──
(function injectMock() {
  // Camera — generate a dark placeholder image
  const canvas = document.createElement('canvas');
  canvas.width = 640; canvas.height = 480;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#0a1a0a';
  ctx.fillRect(0, 0, 640, 480);
  // Draw fake plant rows
  for (let r = 0; r < 5; r++) {
    ctx.fillStyle = '#1a3a1a';
    ctx.fillRect(0, r * 100 + 20, 640, 15);
    ctx.fillStyle = '#2a5a2a';
    ctx.fillRect(100 + r * 30, r * 100 - 40, 120, 70);
    // Draw detection bbox
    ctx.strokeStyle = '#10B981';
    ctx.lineWidth = 3;
    ctx.strokeRect(140, r * 100 - 30, 80, 60);
    ctx.fillStyle = '#10B981';
    ctx.font = '14px monospace';
    ctx.fillText('Plant 87%', 142, r * 100 - 38);
  }
  store.cameraFrame = canvas.toDataURL('image/jpeg', 0.7);

  // YOLO detection — mock not detected
  store.plantDetected = false;
  store.plantConfidence = 0;
  store.plantBbox = [];
  store.plantAreaRatio = 0;

  // Diagnosis — no real data yet: show placeholder instead of a fake class
  store.diagnosisDisease = '--';
  store.diagnosisConfidence = 0;
  store.diagnosisProbabilities = [];
  store.diagnosisCropType = 'tomato';

  // Advisory
  // === MOCK START: advisory (remove after test) ===
  store.advisoryText = '建议 24h 内喷施嘧菌酯 250g/L SC 800 倍液，间隔 7-10 天二次喷施。重点喷洒中下部叶片，注意正反面均匀着药。';
  store.advisoryUrgency = 24;
  store.advisoryFungicide = '嘧菌酯 250g/L SC 800x';
  // === MOCK END ===

  // Environment — Xuzhou July 11:00 overcast scenario: bounded random walk, 60s tick.
  // Real /sensor/environment_fixed data overrides sim for 90s after the last message.
  // [storeKey, base, min, max, maxStepPerTick, decimals]
  const ENV_SIM = [
    ['envAirTemp',      28.0, 26.0,  30.5, 0.4,  1],
    ['envAirHumidity',  78.0, 70.0,  88.0, 2.0,  1],
    ['envCO2',         420.0, 400.0, 460.0, 8.0, 0],
    ['envSoilTemp',     26.0, 24.5,  27.5, 0.2,  1],
    ['envSoilHumidity', 58.0, 50.0,  65.0, 1.5,  1],
    ['envLeafWetness',   3.0,  0.5,   5.0, 0.6,  1],
    ['envSoilN',        12.5, 10.0,  15.0, 0.3,  1],
    ['envSoilP',         8.3,  7.0,  10.0, 0.2,  1],
    ['envSoilK',        15.7, 13.0,  18.0, 0.3,  1],
    ['envSoilPH',        6.5,  6.3,   6.8, 0.05, 2],
  ];
  function envSimTick() {
    if (Date.now() - lastRealEnvTime < 90000) return;  // real sensor active
    ENV_SIM.forEach(([key, , min, max, maxStep, decimals]) => {
      let v = store[key] + (Math.random() * 2 - 1) * maxStep;
      v = Math.min(max, Math.max(min, v));
      store[key] = parseFloat(v.toFixed(decimals));
    });
  }
  ENV_SIM.forEach(([key, base]) => { store[key] = base; });
  store.envDataSource = 'FIXED_NODE_01';
  setInterval(envSimTick, 60000);

  // Chassis
  store.batteryVoltage = 12.1;
  store.leftSpeed = 0.15;
  store.rightSpeed = 0.14;
  // Keep mission state empty until real /mission/status or /waypoints data arrives.
  store.missionState = 'IDLE';
  store.missionCurrentAction = '';
  store.missionPlantsDetected = 0;
  store.missionPlantsAnalyzed = 0;
  store.missionProgress = 0;
  store.missionCurrentWpIdx = 0;
  store.missionTotalWps = 0;
  store.missionWaypointLabels = [];
  store._rawWaypoints = [];

  // Weather — fetch from local proxy, fallback to static mock
  (function initWeather() {
    const WEATHER_PROXY = `http://${window.location.hostname}:8090/weather.json`;
    const REFRESH_MS = 3600000; // 1 hour

    function applyWeather(data) {
      store.weatherDays = (data.days || []).map(d => ({
        day_offset: d.day_offset, temp_high: d.temp_high, temp_low: d.temp_low,
        humidity: d.humidity, precipitation: d.precipitation,
        wind_speed: d.wind_speed, weather_desc: d.weather_desc,
      }));
      store.weatherDisasterAlerts = data.disaster_alerts || [];
      store.weatherCity = data.city || '';
      store.weatherLat = data.lat || 32.06;
      store.weatherLon = data.lon || 118.79;
      store.weatherStale = false;
    }

    function fetchWeather() {
      if (!store.weatherAutoFetch) return;
      fetch(WEATHER_PROXY)
        .then(r => r.json())
        .then(data => { if (data && data.days) applyWeather(data); })
        .catch(() => {}); // silent fallback to existing data
    }

    // Try immediately, then every hour
    fetchWeather();
    setInterval(fetchWeather, REFRESH_MS);

    store.setWeatherAutoFetch = function(on) {
      store.weatherAutoFetch = Boolean(on);
      if (store.weatherAutoFetch) fetchWeather();
    };
  })();

  // === MOCK START: forecast alerts (remove after test) ===
  const now = Date.now();
  const BASE_HOUR = 1;  // data starts at 01:00 today
  const totalSpan = 21 * 60;  // 01:00 → 22:00 = 21 hours = 1260 min
  const forecastData = [
    { hour: 1,  level: 'SUSPICION', prob: 0.35, risk: 0.35, mode: 'LATENT_SUSPICION', conf: 0.42, lwd: 9.2,
      humidity: 92, temp: 19.2, soilTemp: 21.0, soilHumidity: 62, disease: 'healthy', diagConf: 0,
      desc: '凌晨高湿，LWD 达 9.2h，叶面持续湿润',
      evidence: ['环境: 湿度 92% > 阈值 85%', 'LWD: 9.2h > 阈值 6.0h', '模式: LATENT_SUSPICION'] },
    { hour: 3,  level: 'WARNING',   prob: 0.48, risk: 0.48, mode: 'HIGH_HUMIDITY_PATHOGEN', conf: 0.52, lwd: 11.0,
      humidity: 94, temp: 18.5, soilTemp: 20.8, soilHumidity: 63, disease: 'healthy', diagConf: 0,
      desc: '凌晨气温降至最低 18.5°C，湿度峰值 94%，LWD 达 11h',
      evidence: ['环境: 湿度 94% > 阈值 85%', 'LWD: 11.0h > 阈值 6.0h', '温度: 18.5°C 适宜病原', '模式: HIGH_HUMIDITY_PATHOGEN'] },
    { hour: 6,  level: 'SUSPICION', prob: 0.38, risk: 0.38, mode: 'LATENT_SUSPICION', conf: 0.45, lwd: 5.8,
      humidity: 88, temp: 19.8, soilTemp: 20.9, soilHumidity: 62, disease: 'healthy', diagConf: 0,
      desc: '日出升温，湿度回落至 88%，LWD 降至 5.8h',
      evidence: ['环境: 湿度 88%', 'LWD: 5.8h < 阈值 6.0h', '趋势: 湿度下降 -6%'] },
    { hour: 8,  level: 'NORMAL',    prob: 0.18, risk: 0.18, mode: 'BALANCED', conf: 0.55, lwd: 3.2,
      humidity: 82, temp: 22.0, soilTemp: 21.2, soilHumidity: 61, disease: 'healthy', diagConf: 0,
      desc: '叶片表面开始干燥，LWD 降至 3.2h，风险回落',
      evidence: ['环境: 湿度 82% < 阈值 85%', 'LWD: 3.2h', '模式: BALANCED'] },
    { hour: 11, level: 'NORMAL',    prob: 0.10, risk: 0.10, mode: 'BALANCED', conf: 0.65, lwd: 0.8,
      humidity: 68, temp: 25.8, soilTemp: 22.5, soilHumidity: 58, disease: 'healthy', diagConf: 0,
      desc: '午间升温至 25.8°C，湿度降至 68%，叶面完全干燥',
      evidence: ['环境: 湿度 68% < 阈值 85%', 'LWD: 0.8h', '温度: 25.8°C 不利病原', '模式: BALANCED'] },
    { hour: 14, level: 'NORMAL',    prob: 0.08, risk: 0.08, mode: 'BALANCED', conf: 0.70, lwd: 0.3,
      humidity: 62, temp: 28.1, soilTemp: 23.8, soilHumidity: 55, disease: 'healthy', diagConf: 0,
      desc: '午后温度峰值 28.1°C，湿度最低 62%，病害风险最低',
      evidence: ['环境: 湿度 62% 远低于阈值', 'LWD: 0.3h', '温度: 28.1°C 抑制病原', '模式: BALANCED'] },
    { hour: 17, level: 'NORMAL',    prob: 0.14, risk: 0.14, mode: 'BALANCED', conf: 0.60, lwd: 0.5,
      humidity: 72, temp: 25.2, soilTemp: 23.5, soilHumidity: 56, disease: 'healthy', diagConf: 0,
      desc: '傍晚温度回落，湿度开始回升至 72%',
      evidence: ['环境: 湿度 72% < 阈值 85%', 'LWD: 0.5h', '模式: BALANCED'] },
    { hour: 19, level: 'SUSPICION', prob: 0.32, risk: 0.32, mode: 'LATENT_SUSPICION', conf: 0.48, lwd: 3.8,
      humidity: 84, temp: 22.3, soilTemp: 22.8, soilHumidity: 59, disease: 'healthy', diagConf: 0,
      desc: '入夜降温，湿度升至 84%，LWD 开始累积至 3.8h',
      evidence: ['环境: 湿度 84% 接近阈值', 'LWD: 3.8h 上升中', '模式: LATENT_SUSPICION'] },
    { hour: 22, level: 'WARNING',   prob: 0.58, risk: 0.58, mode: 'HIGH_HUMIDITY_PATHOGEN', conf: 0.55, lwd: 7.2,
      humidity: 89, temp: 20.5, soilTemp: 22.0, soilHumidity: 61, disease: 'healthy', diagConf: 0,
      desc: '夜间高湿 89%，LWD 突破阈值达 7.2h，早疫病风险预警',
      evidence: ['环境: 湿度 89% > 阈值 85%', 'LWD: 7.2h > 阈值 6.0h', '温度: 20.5°C 适宜病原', '模式: HIGH_HUMIDITY_PATHOGEN'] },
  ];
  forecastData.forEach(d => {
    const minOffset = totalSpan - (d.hour - BASE_HOUR) * 60;
    store.forecastAlerts.push({
      time: new Date(now - minOffset * 60000).toISOString(),
      active: d.level !== 'NORMAL',
      alert_type: d.level,
      probability: d.prob,
      description: d.desc,
      hours_ahead: 3,
      risk_score: d.risk,
      mode: d.mode,
      confidence: d.conf,
      lwd_hours: d.lwd,
      evidence_chain: d.evidence,
      snapshot: {
        frame: canvas.toDataURL('image/jpeg', 0.6),
        diagnosisDisease: d.disease,
        diagnosisConfidence: d.diagConf,
        advisoryText: d.level === 'WARNING'
          ? '建议 24h 内喷施嘧菌酯 250g/L SC 800 倍液，间隔 7-10 天二次喷施'
          : d.level === 'SUSPICION'
          ? '加强监测，若 LWD 持续上升考虑预防性施药'
          : '暂无农艺建议，继续常规巡检',
        advisoryFungicide: d.level === 'WARNING' ? '嘧菌酯 250g/L SC 800x' : '',
        envAirTemp: d.temp,
        envAirHumidity: d.humidity,
        envSoilTemp: d.soilTemp,
        envSoilHumidity: d.soilHumidity,
        envLeafWetness: d.lwd,
      },
    });
  });
  // === MOCK END ===

  // === MOCK START: fusion history (remove after test) ===
  const fusionSnapshots = [
    { hour: 3,  risk: 0.48, level: 'WARNING',  disease: 'healthy', conf: 0.52, lwd: 11.0, humidity: 94, temp: 18.5 },
    { hour: 19, risk: 0.32, level: 'SUSPICION',disease: 'healthy', conf: 0.48, lwd: 3.8,  humidity: 84, temp: 22.3 },
    { hour: 22, risk: 0.58, level: 'WARNING',  disease: 'healthy', conf: 0.55, lwd: 7.2,  humidity: 89, temp: 20.5 },
  ];
  fusionSnapshots.forEach(s => {
    const minOffset = totalSpan - (s.hour - BASE_HOUR) * 60;
    store.fusionResults.push({
      time: new Date(now - minOffset * 60000).toISOString(),
      risk_score: s.risk,
      alert_level: s.level,
      mode: 'VISION_DOMINANT',
      evidence_chain: [
        '视觉: ' + Math.round(s.conf * 100) + '% 早疫病 (early_blight)',
        '环境: 湿度 ' + s.humidity + '% > 阈值 85%',
        'LWD: ' + s.lwd + 'h > 阈值 6.0h',
        '趋势: 过去 2h 湿度趋势 +0.15',
      ],
      lwd_hours: s.lwd,
      confidence: s.conf,
      snapshot: {
        frame: canvas.toDataURL('image/jpeg', 0.6),
        diagnosisDisease: s.disease,
        diagnosisConfidence: s.conf,
        advisoryText: '建议 24h 内喷施嘧菌酯 250g/L SC 800 倍液，间隔 7-10 天二次喷施',
        advisoryFungicide: '嘧菌酯 250g/L SC 800x',
        envAirTemp: s.temp,
        envAirHumidity: s.humidity,
        envSoilTemp: 22.1,
        envSoilHumidity: 60.5,
        envLeafWetness: s.lwd,
      }
    });
  });
  // === MOCK END (fusion history) ===
})();

// ── Diagnosis mock toggle: 'real' | 'healthy' | 'early_blight' | 'leaf_mold' ──
// Shared via Flask server so all clients see the same mode
const MOCK_MODE_URL = 'http://' + window.location.hostname + ':5000/mock-diagnosis-mode';

store.mockDiagnosisMode = 'real';

async function fetchMockMode() {
  try {
    const resp = await fetch(MOCK_MODE_URL);
    const data = await resp.json();
    store.mockDiagnosisMode = data.mode;
  } catch (e) { /* server not reachable, keep local mode */ }
}

store.setMockMode = async function(mode) {
  store.mockDiagnosisMode = mode;
  try {
    await fetch(MOCK_MODE_URL, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode })
    });
  } catch (e) { /* server not reachable, local-only */ }
};

let cruiseSpeedLoaded = false;

function refreshStackStatus() {
  return fetch(API_BASE + '/status')
    .then(resp => resp.json())
    .then(data => {
      store.stackReady = Boolean(data.stack_ready);
      // Sync live node states from the ROS graph, but don't fight
      // with an in-flight start/stop request.
      if (!store.visionStarting && !store.visionStopping) {
        store.cameraReady = Boolean(data.camera_running);
      }
      if (!store.inferenceStarting && !store.inferenceStopping) {
        store.inferenceReady = Boolean(data.inference_running);
      }
      if (!cruiseSpeedLoaded && Number.isFinite(Number(data.cruise_speed))) {
        store.cruiseSpeed = Number(data.cruise_speed);
        cruiseSpeedLoaded = true;
      }
      if (data.vision_inference_mode) {
        store.visionInferenceMode = data.vision_inference_mode;
      }
      store.messageUnread = Number(data.message_unread || 0);
      return data;
    })
    .catch(() => null);
}

function fetchVisionInferenceMode() {
  return fetch(API_BASE + '/vision/inference-mode')
    .then(resp => resp.json())
    .then(data => {
      if (data.mode) store.visionInferenceMode = data.mode;
      return data;
    })
    .catch(() => null);
}

store.toggleVisionInferenceMode = async function() {
  const next = store.visionInferenceMode === 'triggered'
    ? 'independent'
    : 'triggered';
  store.visionInferenceModeBusy = true;
  try {
    const resp = await fetch(API_BASE + '/vision/inference-mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode: next }),
    });
    const data = await resp.json().catch(() => ({}));
    if (!resp.ok || data.status !== 'ok') {
      throw new Error(data.message || 'Failed to switch vision inference mode');
    }
    store.visionInferenceMode = data.mode || next;
    return data;
  } finally {
    store.visionInferenceModeBusy = false;
  }
};
// Sync from server on load, then poll every 1s
fetchMockMode();
fetchVisionInferenceMode();
callGetWaypoints().catch(err => console.warn('[waypoints] initial load failed:', err));
fetchFixedPointStops().catch(err => console.warn('[fixed-point-stops] initial load failed:', err));
setInterval(fetchMockMode, 1000);
refreshStackStatus();
setInterval(refreshStackStatus, 3000);

// ── Mission message center ──
function fetchMessages() {
  return fetch(API_BASE + '/api/messages')
    .then(resp => resp.json())
    .then(data => {
      store.messageBatches = data.batches || [];
      store.messageUnread = Number(data.unread || 0);
      return data;
    })
    .catch(() => null);
}

store.openMessages = async function() {
  await fetchMessages();
  store.showMessages = true;
  store.messageUnread = 0;
  fetch(API_BASE + '/api/messages/read', { method: 'POST' }).catch(() => {});
};

store.clearMessages = async function() {
  if (!window.confirm('确定清空所有巡航批次的快照与记录？')) return;
  await fetch(API_BASE + '/api/messages/clear', { method: 'POST' }).catch(() => {});
  store.messageBatches = [];
  store.messageUnread = 0;
};

store.formatMsgTime = function(ts) {
  return new Date(ts * 1000).toLocaleTimeString('zh-CN', { hour12: false });
};

// Auto-connect on load (real ROS data will overwrite mock if connected)
try { rosConnect(); } catch(e) { console.log('[ROS] Connection deferred'); }
