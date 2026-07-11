import { getStore, onStoreChange } from '../../services/store';
import { getCameraUrl } from '../../services/api';
import { formatTemp, formatHumidity, formatCO2, formatNPK } from '../../utils/format';

Component({
  data: {
    cameraUrl: '',
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
      this.setData({ cameraUrl: getCameraUrl() + '?t=' + Date.now() });
      this._camTimer = setInterval(() => {
        this.setData({ cameraUrl: getCameraUrl() + '?t=' + Date.now() });
      }, 2000);
    },
    detached() {
      if (this._unsub) this._unsub();
      if (this._camTimer) clearInterval(this._camTimer as number);
    },
  },
  methods: {
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
  },
})
