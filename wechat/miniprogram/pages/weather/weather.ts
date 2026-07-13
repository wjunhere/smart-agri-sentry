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

  return { dayBars, h24, hMax, hMin };
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
    h24: [] as any[],
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
        h24: chart.h24,
        hMax: chart.hMax,
        hMin: chart.hMin,
      }, () => {
        // Only redraw if the h24 data actually changed
        const key = JSON.stringify(this.data.h24);
        if (key !== (this as any)._lastH24) {
          (this as any)._lastH24 = key;
          this._drawSparkline();
        }
      });
    },

    _drawSparkline() {
      const query = wx.createSelectorQuery().in(this);
      query.select('.spark-canvas').boundingClientRect((rect: any) => {
        if (!rect) return;
        const W = rect.width;
        const H = rect.height;
        const pts = this.data.h24;
        if (!pts || pts.length < 2) return;

        const ctx = wx.createCanvasContext('sparklineCanvas', this);
        ctx.clearRect(0, 0, W, H);  // clear previous frame

        const N = pts.length;
        const hMax = this.data.hMax;
        const hMin = this.data.hMin;
        const range = hMax - hMin || 1;
        const pad = { top: 14, bottom: 18, left: 2, right: 2 };
        const pw = W - pad.left - pad.right;
        const ph = H - pad.top - pad.bottom;
        const stepX = pw / (N - 1);
        const toX = (i: number) => pad.left + i * stepX;
        const toY = (t: number) => pad.top + ph * (1 - (t - hMin) / range);

        // Gradient area fill under the curve
        const grad = ctx.createLinearGradient(0, pad.top, 0, H - pad.bottom);
        grad.addColorStop(0, 'rgba(56,189,248,0.25)');
        grad.addColorStop(0.6, 'rgba(56,189,248,0.06)');
        grad.addColorStop(1, 'rgba(56,189,248,0.0)');

        ctx.beginPath();
        ctx.moveTo(toX(0), H - pad.bottom);
        for (let i = 0; i < N; i++) ctx.lineTo(toX(i), toY(pts[i].temp));
        ctx.lineTo(toX(N - 1), H - pad.bottom);
        ctx.closePath();
        ctx.setFillStyle(grad);
        ctx.fill();

        // Glow line
        ctx.beginPath();
        ctx.moveTo(toX(0), toY(pts[0].temp));
        for (let i = 1; i < N; i++) ctx.lineTo(toX(i), toY(pts[i].temp));
        ctx.setStrokeStyle('rgba(56,189,248,0.5)');
        ctx.setLineWidth(4);
        ctx.setLineCap('round');
        ctx.setLineJoin('round');
        ctx.stroke();

        // Main line
        ctx.beginPath();
        ctx.moveTo(toX(0), toY(pts[0].temp));
        for (let i = 1; i < N; i++) ctx.lineTo(toX(i), toY(pts[i].temp));
        ctx.setStrokeStyle('#38BDF8');
        ctx.setLineWidth(2);
        ctx.stroke();

        // Dots
        for (let i = 0; i < N; i++) {
          const x = toX(i);
          const y = toY(pts[i].temp);
          ctx.beginPath();
          ctx.arc(x, y, 3, 0, Math.PI * 2);
          ctx.setFillStyle('#0F172A');
          ctx.fill();
          ctx.setStrokeStyle('#38BDF8');
          ctx.setLineWidth(2);
          ctx.stroke();
        }

        // Temp labels
        ctx.setFillStyle('rgba(148,163,184,0.7)');
        ctx.setFontSize(10);
        ctx.setTextAlign('left');
        ctx.fillText(`${hMax.toFixed(0)}°`, 2, pad.top + 10);
        ctx.fillText(`${hMin.toFixed(0)}°`, 2, H - pad.bottom - 2);

        // Time labels
        ctx.setTextAlign('center');
        for (const lh of [0, 6, 12, 18, 24]) {
          const idx = Math.min(lh, N - 1);
          ctx.fillText(`+${lh}h`, toX(idx), H - 2);
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
