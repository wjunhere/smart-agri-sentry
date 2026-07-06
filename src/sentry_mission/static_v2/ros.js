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
  advisoryText: '',
  advisoryUrgency: 0,
  advisoryFungicide: '',
  forecastAlerts: [],
  fusionResults: [],
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

// ── Mock data for local testing (remove in production) ──
setTimeout(() => {
  if (store.connected) return;  // skip if real ROS is connected

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

  // YOLO detection
  store.plantDetected = true;
  store.plantConfidence = 0.873;
  store.plantBbox = [0.22, 0.15, 0.34, 0.28];
  store.plantAreaRatio = 0.032;

  // Diagnosis
  store.diagnosisDisease = 'late_blight';
  store.diagnosisConfidence = 0.923;
  store.diagnosisProbabilities = [0.923, 0.041, 0.018, 0.009, 0.005, 0.003, 0.001];
  store.diagnosisCropType = 'tomato';

  // Advisory
  store.advisoryText = '建议 2 小时内喷施代森锰锌 800 倍液，间隔 7 天后二次喷施。注意叶片正反面均匀喷洒。';
  store.advisoryUrgency = 2;
  store.advisoryFungicide = '代森锰锌 800x';

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

  // Forecast alerts — generate last 20 entries
  const alertTypes = ['NORMAL', 'NORMAL', 'SUSPICION', 'WARNING', 'NORMAL', 'CRITICAL', 'WARNING', 'SUSPICION', 'NORMAL', 'NORMAL'];
  for (let i = 0; i < 20; i++) {
    const level = alertTypes[i % alertTypes.length];
    store.forecastAlerts.push({
      time: new Date(Date.now() - (20 - i) * 3600000).toISOString(),
      active: level !== 'NORMAL',
      alert_type: level,
      probability: level === 'CRITICAL' ? 0.91 : level === 'WARNING' ? 0.73 : level === 'SUSPICION' ? 0.45 : 0.12,
      description: level === 'NORMAL' ? '正常' : level === 'SUSPICION' ? '湿度偏高' : level === 'WARNING' ? '叶霉病风险上升' : '晚疫病高风险',
      hours_ahead: 3,
    });
  }

  // Fusion history — generate 3 past alerts for detail modal
  for (let i = 2; i >= 0; i--) {
    store.fusionResults.push({
      time: new Date(Date.now() - i * 7200000).toISOString(),
      risk_score: 0.91 - i * 0.15,
      alert_level: i === 0 ? 'CRITICAL' : i === 1 ? 'WARNING' : 'SUSPICION',
      mode: 'VISION_DOMINANT',
      evidence_chain: [
        '视觉: 92.3% 晚疫病 (Late Blight)',
        '环境: 湿度 88% > 阈值 85%',
        '交互: 叶面湿润 6.2h > 阈值 6.0h',
      ],
      lwd_hours: 6.2,
      confidence: 0.92,
      snapshot: {
        frame: canvas.toDataURL('image/jpeg', 0.6),
        diagnosisDisease: 'late_blight',
        diagnosisConfidence: 0.923,
        advisoryText: '建议 2 小时内喷施代森锰锌 800 倍液',
        advisoryFungicide: '代森锰锌 800x',
        envAirTemp: 26.4,
        envAirHumidity: 88.2,
        envSoilTemp: 22.1,
        envSoilHumidity: 60.5,
        envLeafWetness: 6.2,
      }
    });
  }

  console.log('[MOCK] Injected test data into store');
}, 500);
