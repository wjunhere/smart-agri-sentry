// services/ws.ts
// WebSocket connection manager — real-time data from miniprogram_bridge_node

import { updateStore } from './store';
import { getWsUrl } from './config';

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
    url: getWsUrl(),
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
    case 'camera':
      updateStore({ cameraFrameUrl: 'data:image/jpeg;base64,' + data.jpeg_b64 });
      break;

    case 'weather':
      updateStore({
        weatherCity: data.city || '',
        weatherLat: data.lat != null ? data.lat : null,
        weatherLon: data.lon != null ? data.lon : null,
        weatherDays: data.days || [],
        weatherHours: data.hours || [],
        weatherDisasterAlerts: data.disaster_alerts || [],
        weatherStale: Boolean(data.stale),
        weatherTs: msg.ts || Date.now(),
      });
      break;

    case 'snapshot':
      updateStore({
        mode: data.mode,
        linear: data.linear,
        angular: data.angular,
        rosConnected: data.ros_connected,
      });
      if (data.stack) {
        updateStore({
          stackState: data.stack.state,
          stackAlive: Boolean(data.stack.stack_alive),
        });
      }
      if (data.sensors) applySensorData(data.sensors);
      if (data.mission) applyMissionData(data.mission);
      if (data.fusion) applyFusionData(data.fusion, msg.ts);
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
        diagnosisTs: msg.ts || Date.now(),
      });
      break;

    case 'fusion':
      applyFusionData(data, msg.ts);
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

    case 'llm':
      updateStore({
        llmStatus: data.status,
        llmSummary: data.summary,
        llmSuggestions: data.suggestions || [],
        llmRiskLevel: data.risk_level,
        llmFocusAreas: data.focus_areas || [],
        llmNextCheck: data.next_check,
        llmTrigger: data.trigger,
        llmLoading: false,
      });
      break;

    case 'stack_status':
      updateStore({
        stackState: data.state,
        stackMessage: data.message || '',
        stackAlive: Boolean(data.alive),
      });
      break;
  }
}

function applyFusionData(d: any, ts?: number) {
  if (!d) return;
  updateStore({
    fusionRiskScore: d.risk_score != null ? d.risk_score : null,
    fusionAlertLevel: d.alert_level != null ? d.alert_level : 0,
    fusionAlertName: d.alert_name || 'NORMAL',
    fusionMode: d.mode || '',
    fusionEvidence: d.evidence_chain || [],
    fusionLwdHours: d.lwd_hours != null ? d.lwd_hours : null,
    fusionConfidence: d.confidence != null ? d.confidence : null,
    fusionVisionTerm: d.vision_term || 0,
    fusionEnvTerm: d.env_term || 0,
    fusionInteractionTerm: d.interaction_term || 0,
    fusionTs: ts || Date.now(),
  });
}

function applySensorData(d: any) {
  updateStore({
    envTs: Date.now(),
    envAirTemp: d.air_temp != null ? d.air_temp : null,
    envAirHumidity: d.air_humidity != null ? d.air_humidity : null,
    envCO2: d.co2 != null ? d.co2 : null,
    envSoilTemp: d.soil_temp != null ? d.soil_temp : null,
    envSoilHumidity: d.soil_humidity != null ? d.soil_humidity : null,
    envSoilN: d.soil_n != null ? d.soil_n : null,
    envSoilP: d.soil_p != null ? d.soil_p : null,
    envSoilK: d.soil_k != null ? d.soil_k : null,
    envLeafWetness: d.leaf_wetness != null ? d.leaf_wetness : null,
    envLeafTemp: d.leaf_temp != null ? d.leaf_temp : null,
    envHcho: d.hcho != null ? d.hcho : null,
    envTvoc: d.tvoc != null ? d.tvoc : null,
    envPm25: d.pm25 != null ? d.pm25 : null,
    envPm10: d.pm10 != null ? d.pm10 : null,
    envEc: d.ec != null ? d.ec : null,
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
