// ros.js — roslibjs connection wrapper and reactive state store
const ROS_CONFIG = {
  url: 'ws://' + window.location.hostname + ':9090'
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
  fusionResults: [],
  weatherDays: [],
  weatherHours: [],
  weatherDisasterAlerts: [],
  weatherStale: false,
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
  mode: 'AUTO',
  cropType: 'tomato',
  selectedAlert: null,
  showWpEditor: false,
  _rawWaypoints: [],
});
const store = window.store;  // local alias for internal use in this file

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
  ['/out/compressed', 'sensor_msgs/CompressedImage',
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
     if (store.mockDiagnosisMode !== 'real') return;
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

  // Diagnosis — mock healthy
  store.diagnosisDisease = 'healthy';
  store.diagnosisConfidence = 0.85;
  store.diagnosisProbabilities = [0.85, 0.06, 0.04, 0.02, 0.01, 0.01, 0.01];
  store.diagnosisCropType = 'tomato';

  // Advisory
  // === MOCK START: advisory (remove after test) ===
  store.advisoryText = '建议 24h 内喷施嘧菌酯 250g/L SC 800 倍液，间隔 7-10 天二次喷施。重点喷洒中下部叶片，注意正反面均匀着药。';
  store.advisoryUrgency = 24;
  store.advisoryFungicide = '嘧菌酯 250g/L SC 800x';
  // === MOCK END ===

  // Environment — fixed node
  store.envAirTemp = 26.4;
  store.envAirHumidity = 88.2;
  store.envCO2 = 420;
  store.envSoilTemp = 22.1;
  store.envSoilHumidity = 60.5;
  store.envLeafWetness = 6.2;
  store.envSoilN = 12.5;
  store.envSoilP = 8.3;
  store.envSoilK = 15.7;
  store.envSoilPH = 6.5;
  store.envDataSource = 'FIXED_NODE_01';

  // Chassis
  store.batteryVoltage = 12.1;
  store.leftSpeed = 0.15;
  store.rightSpeed = 0.14;

  // Mission
  store.missionState = 'PATROL';
  store.missionCurrentAction = '巡航空点 2/5';
  store.missionPlantsDetected = 23;
  store.missionPlantsAnalyzed = 18;
  store.missionProgress = 0.4;
  store.missionCurrentWpIdx = 1;
  store.missionTotalWps = 5;
  store.missionWaypointLabels = [
    'WP0: (0.0, 0.0)', 'WP1: (4.0, 0.0)',
    'WP2: (4.0, 1.2)', 'WP3: (0.0, 1.2)', 'WP4: (0.0, 2.4)'
  ];
  store._rawWaypoints = [
    { x: 2.5, y: 0.0, yaw: 0.0 },
    { x: 2.5, y: 0.6, yaw: 1.5708 },
    { x: 0.0, y: 0.6, yaw: 3.1416 },
  ];

  // Weather — fetch from local proxy, fallback to static mock
  (function initWeather() {
    const WEATHER_PROXY = 'http://localhost:8090/weather.json';
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
      fetch(WEATHER_PROXY)
        .then(r => r.json())
        .then(data => { if (data && data.days) applyWeather(data); })
        .catch(() => {}); // silent fallback to existing data
    }

    // Try immediately, then every hour
    fetchWeather();
    setInterval(fetchWeather, REFRESH_MS);
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

// ── Diagnosis mock toggle: 'real' | 'healthy' | 'early_blight' ──
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

// Sync from server on load, then poll every 1s
fetchMockMode();
setInterval(fetchMockMode, 1000);

setInterval(() => {
  if (store.mockDiagnosisMode === 'real') return;

  // In mock mode, simulate plant detection so diagnosis is visible
  store.plantDetected = true;
  store.plantConfidence = 0.82 + Math.random() * 0.10;
  store.plantBbox = [140, 100, 80, 60];
  store.plantAreaRatio = 0.05 + Math.random() * 0.03;

  const conf = 0.80 + Math.random() * 0.10;
  store.diagnosisCropType = store.cropType;
  store.diagnosisConfidence = conf;
  store._diagBuf = [];

  if (store.mockDiagnosisMode === 'healthy') {
    store.diagnosisDisease = 'healthy';
    store.diagnosisProbabilities = [conf, 0.06, 0.04, 0.02, 0.01, 0.01, Math.max(0, 1 - conf - 0.14)];
  } else if (store.mockDiagnosisMode === 'early_blight') {
    store.diagnosisDisease = 'early_blight';
    store.diagnosisProbabilities = [0.06, conf, 0.04, 0.02, 0.01, 0.01, Math.max(0, 1 - conf - 0.14)];
  }
}, 1500);

// Auto-connect on load (real ROS data will overwrite mock if connected)
try { rosConnect(); } catch(e) { console.log('[ROS] Connection deferred'); }
