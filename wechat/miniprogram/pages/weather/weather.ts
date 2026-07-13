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

  const N = h24.length;
  const gap = 1; // px gap between bars (CSS rpx)
  const barWPct = (100 - gap * (N - 1)) / N; // bar width in %

  const hPoints = h24.map((h, i) => ({
    ...h,
    y: ((h.temp - hMin) / hRange * 100).toFixed(1),
    color: tempColor(h.temp, hMin, hMax),
    l: (i * (barWPct + gap)).toFixed(1),
  }));

  // Stepped connectors
  const hBridges: any[] = [];
  const hRisers: any[] = [];
  for (let i = 0; i < N - 1; i++) {
    const y1 = parseFloat(hPoints[i].y);
    const y2 = parseFloat(hPoints[i + 1].y);
    const l1 = parseFloat(hPoints[i].l);
    const l2 = parseFloat(hPoints[i + 1].l);
    const bridgeL = (l1 + barWPct).toFixed(1);
    const bridgeW = (l2 - l1 - barWPct).toFixed(1);
    const lowY = Math.min(y1, y2);
    const hiY = Math.max(y1, y2);
    // horizontal bridge from right edge of bar i to left edge of bar i+1, at lower height
    hBridges.push({
      l1: bridgeL,
      b1: lowY.toFixed(1),
      w: bridgeW,
    });
    // vertical riser at the right edge of the bridge, up to the taller bar
    if (hiY - lowY > 0.3) {
      hRisers.push({
        l: l2.toFixed(1), // at the left edge of bar i+1
        b: lowY.toFixed(1),
        h: (hiY - lowY).toFixed(1),
      });
    }
  }

  return { dayBars, hPoints, hBridges, hRisers, barWPct: barWPct.toFixed(1), hMax, hMin };
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
    hPoints: [] as Array<{y: string, temp: number, color: string, hour_offset: number, l: string}>,
    hBridges: [] as any[],
    hRisers: [] as any[],
    barW: '4.2',
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
        hBridges: chart.hBridges,
        hRisers: chart.hRisers,
        barW: chart.barWPct,
        hMax: chart.hMax,
        hMin: chart.hMin,
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
