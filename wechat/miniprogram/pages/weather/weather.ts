import { getStore, onStoreChange, updateStore } from '../../services/store';
import { apiGetWeather } from '../../services/api';

const WEATHER_ICONS: Record<string, string> = {
  '晴': '☀️', '少云': '🌤️', '多云': '⛅', '阴': '☁️',
  '小雨': '🌧️', '中雨': '🌧️', '大雨': '🌧️', '暴雨': '⛈️',
  '雷阵雨': '⛈️', '雪': '❄️', '小雪': '🌨️', '中雪': '❄️', '大雪': '❄️',
  '雾': '🌫️', '霾': '🌫️', '风': '💨', '沙尘': '💨',
};

function weatherIcon(desc: string): string {
  if (!desc) return '🌈';
  for (const [k, v] of Object.entries(WEATHER_ICONS)) {
    if (desc.includes(k)) return v;
  }
  return '🌤️';
}

function tempColor(t: number, tMin: number, tMax: number): string {
  const ratio = tMax > tMin ? (t - tMin) / (tMax - tMin) : 0.5;
  // Cool blue (#38BDF8) → warm amber (#F59E0B)
  const r = Math.round(56 + ratio * (245 - 56));
  const g = Math.round(189 + ratio * (158 - 189));
  const b = Math.round(248 + ratio * (11 - 248));
  return `rgb(${r},${g},${b})`;
}

function buildChart(days: any[], hours: any[]) {
  const dayTemps = days.flatMap(d => [d.temp_high, d.temp_low]);
  const dayMax = Math.max(...dayTemps, 10);
  const dayMin = Math.min(...dayTemps, 0);
  const dayRange = dayMax - dayMin || 1;
  const dayBars = days.map(d => ({
    ...d,
    icon: weatherIcon(d.weather_desc),
    barLow: ((d.temp_low - dayMin) / dayRange * 100).toFixed(1),
    barHeight: ((d.temp_high - d.temp_low) / dayRange * 100).toFixed(1),
  }));

  const h24 = hours.slice(0, 24);
  const hTemps = h24.map(h => h.temp);
  const hMax = Math.max(...hTemps, 10);
  const hMin = Math.min(...hTemps, 0);
  const hRange = hMax - hMin || 1;
  const hPoints = h24.map(h => ({
    ...h,
    y: ((h.temp - hMin) / hRange * 100).toFixed(1),
    color: tempColor(h.temp, hMin, hMax),
  }));

  return { dayBars, hPoints, hMax, hMin };
}

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
    dayBars: [] as any[],
    hPoints: [] as Array<{y: string, temp: number, color: string, hour_offset: number}>,
    hMax: 40, hMin: 0,
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
      const days = s.weatherDays || [];
      const hours = s.weatherHours || [];
      const chart = buildChart(days, hours);
      this.setData({
        city: s.weatherCity || '--',
        currentTemp: day0 ? day0.temp_high + '°' : '--',
        currentDesc: day0 ? day0.weather_desc : '--',
        humidity: '--',
        days: days,
        hours: hours,
        disasterAlerts: s.weatherDisasterAlerts || [],
        stale: s.weatherStale,
        dayBars: chart.dayBars,
        hPoints: chart.hPoints,
        hMax: chart.hMax,
        hMin: chart.hMin,
      }, () => {
        this._drawSparkline();
      });
    },

    _drawSparkline() {
      const query = wx.createSelectorQuery().in(this);
      query.select('.spark-track').boundingClientRect((rect: any) => {
        if (!rect) return;
        const w = rect.width;
        const h = rect.height;
        const pts = this.data.hPoints;
        if (!pts || pts.length < 2) return;

        const ctx = wx.createCanvasContext('sparklineCanvas', this);
        const N = pts.length;
        const stepX = w / N;
        const barW = stepX * 0.85; // bar takes ~85% of step width

        // Polyline through dot centers (dot is at top-center of each bar)
        ctx.beginPath();
        for (let i = 0; i < N; i++) {
          const x = stepX * i + stepX / 2; // center of bar
          const y = h * (1 - parseFloat(pts[i].y) / 100); // top of bar
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.setStrokeStyle('rgba(255,255,255,0.6)');
        ctx.setLineWidth(2);
        ctx.stroke();

        // Dots
        for (let i = 0; i < N; i++) {
          const x = stepX * i + stepX / 2;
          const y = h * (1 - parseFloat(pts[i].y) / 100);
          ctx.beginPath();
          ctx.arc(x, y, 3, 0, Math.PI * 2);
          ctx.setFillStyle('#fff');
          ctx.fill();
        }
        ctx.draw();
      }).exec();
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
