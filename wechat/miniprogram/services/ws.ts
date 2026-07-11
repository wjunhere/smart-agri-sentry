// services/ws.ts
// WebSocket connection manager — real-time data from miniprogram_bridge_node

import { updateStore } from './store';

const WS_URL = 'ws://192.168.1.100:8765/ws';
const RETRY_DELAYS = [3000, 15000, 30000];

let ws: WechatMiniprogram.SocketTask | null = null;
let retryIdx = 0;
let retryTimer: number | null = null;

export function wsConnect() {
  if (ws) {
    try { ws.close({}); } catch (_) {}
    ws = null;
  }

  ws = wx.connectSocket({
    url: WS_URL,
    header: { 'Content-Type': 'application/json' },
  });

  ws.onOpen(() => {
    console.log('[WS] Connected');
    updateStore({ connected: true });
    retryIdx = 0;
  });

  ws.onMessage((res) => {
    try {
      const msg = JSON.parse(res.data as string);
      handleMessage(msg);
    } catch (e) {
      console.warn('[WS] Bad message:', e);
    }
  });

  ws.onClose(() => {
    console.log('[WS] Disconnected');
    updateStore({ connected: false });
    scheduleReconnect();
  });

  ws.onError((err) => {
    console.error('[WS] Error:', err);
    updateStore({ connected: false });
  });
}

function scheduleReconnect() {
  const delay = RETRY_DELAYS[Math.min(retryIdx, RETRY_DELAYS.length - 1)];
  retryIdx++;
  console.log(`[WS] Reconnecting in ${delay}ms...`);
  if (retryTimer) clearTimeout(retryTimer);
  retryTimer = setTimeout(wsConnect, delay) as unknown as number;
}

function handleMessage(msg: { type: string; ts: number; data: any }) {
  const { type, data } = msg;

  switch (type) {
    case 'snapshot':
      updateStore({
        mode: data.mode,
        linear: data.linear,
        angular: data.angular,
        rosConnected: data.ros_connected,
      });
      if (data.sensors) applySensorData(data.sensors);
      if (data.mission) applyMissionData(data.mission);
      break;

    case 'sensor':
      applySensorData(data);
      break;

    case 'status':
      updateStore({
        mode: data.mode,
        rosConnected: data.ros_connected,
        batteryVoltage: data.battery_voltage,
      });
      break;

    case 'mission':
      applyMissionData(data);
      break;

    case 'diagnosis':
      updateStore({
        diagnosisCropType: data.crop_type,
        diagnosisDisease: data.disease,
        diagnosisConfidence: data.confidence,
        diagnosisProbabilities: data.probabilities || [],
      });
      break;

    case 'plant_detect':
      updateStore({
        plantDetected: data.detected,
        plantConfidence: data.confidence,
        plantBbox: data.bbox,
        plantAreaRatio: data.area_ratio,
      });
      break;

    case 'alert':
      console.log('[WS] Alert:', data.title, data.message);
      break;
  }
}

function applySensorData(d: any) {
  updateStore({
    envAirTemp: d.air_temp ?? null,
    envAirHumidity: d.air_humidity ?? null,
    envCO2: d.co2 ?? null,
    envSoilTemp: d.soil_temp ?? null,
    envSoilHumidity: d.soil_humidity ?? null,
    envSoilN: d.soil_n ?? null,
    envSoilP: d.soil_p ?? null,
    envSoilK: d.soil_k ?? null,
    envLeafWetness: d.leaf_wetness ?? null,
    envDataSource: d.data_source || '',
  });
}

function applyMissionData(d: any) {
  updateStore({
    missionState: d.state,
    missionProgress: d.progress,
    missionCurrentAction: d.current_action,
    missionPlantsDetected: d.plants_detected,
    missionPlantsAnalyzed: d.plants_analyzed,
    missionCurrentWpIdx: d.current_wp_idx,
    missionTotalWps: d.total_wps,
    missionWaypointLabels: d.waypoint_labels || [],
  });
}

export function wsDisconnect() {
  if (ws) {
    ws.close({});
    ws = null;
  }
  if (retryTimer) {
    clearTimeout(retryTimer);
    retryTimer = null;
  }
}
