import { getStore, onStoreChange } from '../../services/store';
import { getCameraSnapshotUrl } from '../../services/api';
import { formatTemp, formatHumidity, formatCO2, formatNPK } from '../../utils/format';

const REFRESH_MS = 150;

Component({
  data: {
    active: 'A' as 'A' | 'B',
    urlA: '',
    urlB: '',
    cameraLoading: true,
    plantDetected: false,
    plantConfidence: '0.0',
    plantAreaRatio: '0.00',
    airTemp: '--',
    airHumidity: '--',
    co2: '--',
    soilTemp: '--',
    soilHumidity: '--',
    soilN: '--',
    soilP: '--',
    soilK: '--',
    leafWetness: '--',
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
      this.setData({
        plantDetected: s.plantDetected,
        plantConfidence: s.plantConfidence ? (s.plantConfidence * 100).toFixed(1) : '0.0',
        plantAreaRatio: s.plantAreaRatio ? s.plantAreaRatio.toFixed(2) : '0.00',
        airTemp: formatTemp(s.envAirTemp),
        airHumidity: formatHumidity(s.envAirHumidity),
        co2: formatCO2(s.envCO2),
        soilTemp: formatTemp(s.envSoilTemp),
        soilHumidity: formatHumidity(s.envSoilHumidity),
        soilN: formatNPK(s.envSoilN),
        soilP: formatNPK(s.envSoilP),
        soilK: formatNPK(s.envSoilK),
        leafWetness: formatHumidity(s.envLeafWetness),
        dataSource: s.envDataSource || '--',
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
