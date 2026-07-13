import { getStore, onStoreChange, updateStore } from '../../services/store';
import { apiGetWeather } from '../../services/api';

Component({
  data: {
    city: '--',
    currentTemp: '--',
    currentDesc: '--',
    humidity: '--',
    days: [] as any[],
    hours: [] as any[],
    disasterAlerts: [] as string[],
    stale: false,
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
      this._pollTimer = setInterval(() => this.fetchWeather(), 60000);
      this.fetchWeather();
    },
    detached() {
      if (this._unsub) this._unsub();
      if (this._pollTimer) clearInterval(this._pollTimer as number);
    },
  },
  methods: {
    sync(s: any) {
      const day0 = s.weatherDays && s.weatherDays[0];
      this.setData({
        city: s.weatherCity || '--',
        currentTemp: day0 ? day0.temp_high + '°' : '--',
        currentDesc: day0 ? day0.weather_desc : '--',
        humidity: '--',
        days: s.weatherDays || [],
        hours: s.weatherHours || [],
        disasterAlerts: s.weatherDisasterAlerts || [],
        stale: s.weatherStale,
      });
    },

    async fetchWeather() {
      try {
        const data = await apiGetWeather();
        if (data && data.days) {
          updateStore({
            weatherCity: data.city,
            weatherDays: data.days,
            weatherHours: data.hours,
            weatherDisasterAlerts: data.disaster_alerts || [],
            weatherStale: data.stale || false,
          });
        }
      } catch (_) {}
    },
  },
})
