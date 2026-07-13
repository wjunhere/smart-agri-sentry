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
    y: (h.temp - hMin) / hRange,
    label: `+${h.hour_offset}h`,
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
    hPoints: [] as Array<{y: number, temp: number, hour_offset: number}>,
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
    ready() {
      this._drawSparkline();
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
      const query = this.createSelectorQuery();
      query.select('#sparkline-canvas')
        .fields({ node: true, size: true })
        .exec((res: any) => {
          if (!res || !res[0] || !res[0].node) return;
          const canvas = res[0].node;
          const ctx = canvas.getContext('2d');
          const w = res[0].width;
          const h = res[0].height;
          const pts = this.data.hPoints;
          if (!pts || pts.length < 2 || w === 0 || h === 0) return;
          canvas.width = w * 2;
          canvas.height = h * 2;
          ctx.scale(2, 2);

          const pad = { top: 14, bottom: 16, left: 4, right: 4 };
          const pw = w - pad.left - pad.right;
          const ph = h - pad.top - pad.bottom;
          const toY = (y: number) => pad.top + ph * (1 - y);
          const toX = (i: number) => pad.left + (pw * i) / (pts.length - 1 || 1);

          // Gradient fill
          const grad = ctx.createLinearGradient(0, pad.top, 0, h - pad.bottom);
          grad.addColorStop(0, 'rgba(56,189,248,0.35)');
          grad.addColorStop(0.5, 'rgba(56,189,248,0.10)');
          grad.addColorStop(1, 'rgba(56,189,248,0.01)');

          ctx.beginPath();
          ctx.moveTo(toX(0), h - pad.bottom);
          for (let i = 0; i < pts.length; i++) {
            ctx.lineTo(toX(i), toY(pts[i].y));
          }
          ctx.lineTo(toX(pts.length - 1), h - pad.bottom);
          ctx.closePath();
          ctx.fillStyle = grad;
          ctx.fill();

          // Line
          ctx.beginPath();
          for (let i = 0; i < pts.length; i++) {
            if (i === 0) ctx.moveTo(toX(i), toY(pts[i].y));
            else ctx.lineTo(toX(i), toY(pts[i].y));
          }
          ctx.strokeStyle = '#38BDF8';
          ctx.lineWidth = 2;
          ctx.lineJoin = 'round';
          ctx.stroke();

          // Dots
          for (let i = 0; i < pts.length; i++) {
            ctx.beginPath();
            ctx.arc(toX(i), toY(pts[i].y), 2.5, 0, Math.PI * 2);
            ctx.fillStyle = '#0F172A';
            ctx.fill();
            ctx.strokeStyle = '#38BDF8';
            ctx.lineWidth = 1.5;
            ctx.stroke();
          }

          // Labels
          ctx.fillStyle = 'rgba(148,163,184,0.6)';
          ctx.font = '10px sans-serif';
          ctx.textAlign = 'center';
          for (const lh of [0, 6, 12, 18, 24]) {
            const idx = Math.min(lh, pts.length - 1);
            ctx.fillText(`+${lh}h`, toX(idx), h - 2);
          }
          ctx.textAlign = 'left';
          ctx.fillText(`${this.data.hMax.toFixed(0)}°`, 2, pad.top + 8);
          ctx.fillText(`${this.data.hMin.toFixed(0)}°`, 2, h - pad.bottom - 2);
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
