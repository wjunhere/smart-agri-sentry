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

function blueShade(t: number, tMin: number, tMax: number): string {
  const ratio = tMax > tMin ? (t - tMin) / (tMax - tMin) : 0.5;
  // Deep blue → bright purple
  const r = Math.round(56 + ratio * 111);   // 56 → 167
  const g = Math.round(100 + ratio * 39);   // 100 → 139
  const b = Math.round(200 + ratio * 55);   // 200 → 255
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
  const hBars = h24.map(h => ({
    ...h,
    hPct: ((h.temp - hMin) / hRange * 100).toFixed(1),
    color: blueShade(h.temp, hMin, hMax),
  }));

  return { dayBars, hBars, h24, hMax, hMin };
}

Component({
  data: {
    city: '--',
    currentTemp: '--',
    currentDesc: '--',
    humidity: '·',
    loadFailed: false,
    days: [] as any[],
    hours: [] as any[],
    disasterAlerts: [] as string[],
    stale: false,
    dayBars: [] as any[],
    hBars: [] as any[],
    hMax: 40, hMin: 0,
    lineSegs: [] as any[],
    lineDots: [] as any[],
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
        humidity: day0 && day0.humidity != null ? day0.humidity + '%' : '·',
        days: days,
        hours: hours,
        disasterAlerts: s.weatherDisasterAlerts || [],
        stale: s.weatherStale,
        dayBars: chart.dayBars,
        hBars: chart.hBars,
        hMax: chart.hMax,
        hMin: chart.hMin,
      });
      this._renderTempLine(chart.hBars);
    },

    // 在柱状图上叠加温度折线：连接各柱顶点
    _renderTempLine(hBars: any[]) {
      if (!hBars || hBars.length < 2) {
        this.setData({ lineSegs: [], lineDots: [] });
        return;
      }
      const r = wx.getSystemInfoSync().windowWidth / 750;  // rpx → px
      this.createSelectorQuery()
        .select('.spark-track')
        .boundingClientRect((rect: any) => {
          if (!rect) return;
          const W = rect.width, H = rect.height;
          const pad = 4 * r, gap = 3 * r;
          const n = hBars.length;
          const barW = (W - 2 * pad - (n - 1) * gap) / n;
          const pts = hBars.map((b: any, i: number) => ({
            x: pad + barW / 2 + i * (barW + gap),
            y: (parseFloat(b.hPct) / 100) * H,
          }));
          const segs = [];
          for (let i = 0; i < n - 1; i++) {
            const dx = pts[i + 1].x - pts[i].x;
            const dy = pts[i + 1].y - pts[i].y;
            segs.push({
              x: pts[i].x,
              y: pts[i].y,
              len: Math.sqrt(dx * dx + dy * dy),
              deg: (-Math.atan2(dy, dx) * 180 / Math.PI).toFixed(2),
            });
          }
          this.setData({ lineSegs: segs, lineDots: pts });
        })
        .exec();
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
          this.setData({ loadFailed: false });
        }
      } catch (_) {
        this.setData({ loadFailed: true });
      }
    },
  },
})
