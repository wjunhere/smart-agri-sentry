// services/store.ts
// Reactive global state — mirrors static_v2/ros.js window.store

const store = {
  connected: false,
  mode: 'AUTO',
  linear: 0,
  angular: 0,
  batteryVoltage: null as number | null,
  rosConnected: false,

  cameraFrameUrl: '',
  plantDetected: false,
  plantConfidence: 0,
  plantBbox: [0, 0, 0, 0] as number[],
  plantAreaRatio: 0,

  diagnosisCropType: '',
  diagnosisDisease: '',
  diagnosisConfidence: 0,
  diagnosisProbabilities: [] as number[],

  advisoryText: '',
  advisoryPriority: '',
  advisorySteps: [] as string[],

  forecastActive: false,
  forecastAlertType: '',
  forecastDescription: '',
  forecastHoursAhead: 0,

  weatherCity: '',
  weatherDays: [] as Array<{high: number, low: number, icon: string, desc: string}>,
  weatherHours: [] as Array<{time: string, temp: number, icon: string, desc: string}>,
  weatherDisasterAlerts: [] as string[],
  weatherStale: false,

  envAirTemp: null as number | null,
  envAirHumidity: null as number | null,
  envCO2: null as number | null,
  envSoilTemp: null as number | null,
  envSoilHumidity: null as number | null,
  envSoilN: null as number | null,
  envSoilP: null as number | null,
  envSoilK: null as number | null,
  envLeafWetness: null as number | null,
  envDataSource: '',

  missionState: 'IDLE',
  missionProgress: 0,
  missionCurrentAction: '',
  missionPlantsDetected: 0,
  missionPlantsAnalyzed: 0,
  missionCurrentWpIdx: 0,
  missionTotalWps: 0,
  missionWaypointLabels: [] as string[],

  cropType: 'tomato',
};

type Store = typeof store;

const listeners: Array<(s: Store) => void> = [];

export function getStore(): Store {
  return store;
}

export function updateStore(partial: Partial<Store>) {
  Object.assign(store, partial);
  for (const fn of listeners) {
    fn(store);
  }
}

export function onStoreChange(fn: (s: Store) => void) {
  listeners.push(fn);
}

export function offStoreChange(fn: (s: Store) => void) {
  const i = listeners.indexOf(fn);
  if (i >= 0) listeners.splice(i, 1);
}
