// ros.js — roslibjs connection wrapper and reactive state store
const ROS_CONFIG = {
  url: 'ws://' + window.location.hostname + ':9090'
};

// Reactive global state (Vue 3 reactive)
const store = Vue.reactive({
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
