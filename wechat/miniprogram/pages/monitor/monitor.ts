import { getStore, onStoreChange } from '../../services/store';
import { formatTemp, formatHumidity, formatCO2, formatNPK, formatUgM3, formatPpb, formatEc } from '../../utils/format';

Component({
  data: {
    cameraUrl: '',
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
    },
  },
  methods: {
    _unsub: null as any,

    sync(s: any) {
      const ph = (v: number | null, fmt: (x: number) => string) =>
        v == null ? (s.connected ? '·' : '--') : fmt(v);
      this.setData({
        cameraUrl: s.cameraFrameUrl || '',
        cameraLoading: !s.cameraFrameUrl,
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
  },
})
