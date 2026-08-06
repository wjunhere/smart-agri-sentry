import { getStore, onStoreChange } from '../../services/store';
import { getCameraSnapshotUrl } from '../../services/api';
import { formatTemp, formatHumidity, formatCO2, formatNPK, formatUgM3, formatPpb, formatEc } from '../../utils/format';

const REFRESH_MS = 200;

Component({
  data: {
    active: 'A' as 'A' | 'B',
    urlA: '',
    urlB: '',
    cameraLoading: true,
    connected: false,
    plantDetected: false,
    plantConfidence: '0.0',
    plantAreaRatio: '0.00',
    airTemp: '--',
    airHumidity: '--',
    co2: '--',
    hcho: '--',
    tvoc: '--',
    pm25: '--',
    pm10: '--',
    soilTemp: '--',
    soilHumidity: '--',
    soilEc: '--',
    soilN: '--',
    soilP: '--',
    soilK: '--',
    leafWetness: '--',
    leafTemp: '--',
    dataSource: '--',
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
    },
    detached() {
      if (this._unsub) this._unsub();
      this.stopCamera();
    },
  },
  pageLifetimes: {
    show() {
      this.startCamera();
    },
    hide() {
      this.stopCamera();
    },
  },
  methods: {
    _camTimer: null as any,
    _errCount: 0,
    _pendingA: false,
    _pendingB: false,

    sync(s: any) {
      const ph = (v: number | null, fmt: (x: number) => string) =>
        v == null ? (s.connected ? '·' : '--') : fmt(v);
      this.setData({
        connected: s.connected,
        plantDetected: s.plantDetected,
        plantConfidence: s.plantConfidence ? (s.plantConfidence * 100).toFixed(1) : '0.0',
        plantAreaRatio: s.plantAreaRatio ? s.plantAreaRatio.toFixed(2) : '0.00',
        airTemp: ph(s.envAirTemp, formatTemp),
        airHumidity: ph(s.envAirHumidity, formatHumidity),
        co2: ph(s.envCO2, formatCO2),
        hcho: ph(s.envHcho, formatUgM3),
        tvoc: ph(s.envTvoc, formatPpb),
        pm25: ph(s.envPm25, formatUgM3),
        pm10: ph(s.envPm10, formatUgM3),
        soilTemp: ph(s.envSoilTemp, formatTemp),
        soilHumidity: ph(s.envSoilHumidity, formatHumidity),
        soilEc: ph(s.envEc, formatEc),
        soilN: ph(s.envSoilN, formatNPK),
        soilP: ph(s.envSoilP, formatNPK),
        soilK: ph(s.envSoilK, formatNPK),
        leafWetness: ph(s.envLeafWetness, formatHumidity),
        leafTemp: ph(s.envLeafTemp, formatTemp),
        dataSource: s.envDataSource || (s.connected ? '·' : '--'),
      });
    },

    startCamera() {
      this.stopCamera();
      this._errCount = 0;
      this._pendingA = true;
      this._pendingB = false;
      this.setData({ cameraLoading: true });
      this.setData({ active: 'A', urlA: getCameraSnapshotUrl() + '?t=' + Date.now() });
      this._camTimer = setInterval(() => this._tick(), REFRESH_MS);
    },

    stopCamera() {
      if (this._camTimer) {
        clearInterval(this._camTimer as number);
        this._camTimer = null;
      }
    },

    _tick() {
      const next = getCameraSnapshotUrl() + '?t=' + Date.now();
      const { active } = this.data;
      // Preload the next frame on the hidden image, but don't overwrite one
      // that is still loading to avoid cancelling in-flight requests.
      if (active === 'A' && !this._pendingB) {
        this._pendingB = true;
        this.setData({ urlB: next });
      } else if (active === 'B' && !this._pendingA) {
        this._pendingA = true;
        this.setData({ urlA: next });
      }
    },

    onImgLoad(e: any) {
      const key = (e.currentTarget.dataset?.key as string) || '';
      const { active } = this.data;
      if (key === 'A') this._pendingA = false;
      if (key === 'B') this._pendingB = false;
      // Only swap when the hidden image finishes loading
      if (key && key !== active) {
        this.setData({ active: key, cameraLoading: false });
      } else if (key === active) {
        this.setData({ cameraLoading: false });
      }
    },

    onImgError(e: any) {
      const key = (e.currentTarget.dataset?.key as string) || '';
      if (key === 'A') this._pendingA = false;
      if (key === 'B') this._pendingB = false;
      this._errCount += 1;
      if (this._errCount > 3) return;
      setTimeout(() => {
        const next = getCameraSnapshotUrl() + '?t=' + Date.now();
        if (key === 'A') {
          this._pendingA = true;
          this.setData({ urlA: next });
        } else if (key === 'B') {
          this._pendingB = true;
          this.setData({ urlB: next });
        }
      }, 500);
    },
  },
})
