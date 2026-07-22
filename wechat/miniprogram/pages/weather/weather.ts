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
    lineReady: false,
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
    _lineTimer: null as any,

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
      this._scheduleLineRender(chart.hBars);
    },

    // 原生 canvas 定位在布局稳定后才可靠：先销毁，延迟重建再绘制
    _scheduleLineRender(hBars: any[]) {
      if (!hBars || hBars.length < 2) return;
      if (this._lineTimer) clearTimeout(this._lineTimer as number);
      if (this.data.lineReady) {
        this._renderTempLine(hBars);
        return;
      }
      this._lineTimer = setTimeout(() => {
        this.setData({ lineReady: true });
        wx.nextTick(() => this._renderTempLine(hBars));
      }, 400);
    },

    // 在柱状图上叠加温度曲线：Canvas 平滑样条 + 渐变面积填充
    _renderTempLine(hBars: any[]) {
      if (!hBars || hBars.length < 2) return;
      const r = wx.getSystemInfoSync().windowWidth / 750;  // rpx → px
      this.createSelectorQuery()
        .select('.spark-track').boundingClientRect()
        .select('#tempLineCanvas').fields({ node: true, size: true })
        .exec((res: any[]) => {
          const rect = res[0];
          const info = res[1];
          if (!rect || !info || !info.node) return;
          const canvas = info.node;
          const ctx = canvas.getContext('2d');
          const dpr = wx.getSystemInfoSync().pixelRatio;
          const W = rect.width, H = rect.height;
          canvas.width = W * dpr;   // 设置 width 会重置上下文状态
          canvas.height = H * dpr;
          ctx.scale(dpr, dpr);

          const pad = 4 * r, gap = 3 * r, n = hBars.length;
          const barW = (W - 2 * pad - (n - 1) * gap) / n;
          // 上下留白，避免最高点曲线/圆点/光晕被画布顶边裁切
          const topPad = 12, bottomPad = 4;
          const usable = H - topPad - bottomPad;
          const pts = hBars.map((b: any, i: number) => ({
            x: pad + barW / 2 + i * (barW + gap),
            y: topPad + (1 - parseFloat(b.hPct) / 100) * usable,
          }));

          // 曲线下方面积：顶部浅蓝 → 底部透明
          const grad = ctx.createLinearGradient(0, 0, 0, H);
          grad.addColorStop(0, 'rgba(125,211,252,0.28)');
          grad.addColorStop(1, 'rgba(125,211,252,0)');
          ctx.beginPath();
          ctx.moveTo(pts[0].x, H);
          ctx.lineTo(pts[0].x, pts[0].y);
          this._spline(ctx, pts);
          ctx.lineTo(pts[n - 1].x, H);
          ctx.closePath();
          ctx.fillStyle = grad;
          ctx.fill();

          // 平滑曲线 + 柔光
          ctx.beginPath();
          ctx.moveTo(pts[0].x, pts[0].y);
          this._spline(ctx, pts);
          ctx.strokeStyle = '#7DD3FC';
          ctx.lineWidth = 2;
          ctx.lineJoin = 'round';
          ctx.lineCap = 'round';
          ctx.shadowColor = 'rgba(125,211,252,0.5)';
          ctx.shadowBlur = 6;
          ctx.stroke();
          ctx.shadowBlur = 0;

          // 数据点
          ctx.fillStyle = '#0B1120';
          ctx.lineWidth = 1.5;
          for (const p of pts) {
            ctx.beginPath();
            ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();
          }
        });
    },

    // Catmull-Rom 样条转三次贝塞尔（不含 moveTo，从 pts[1] 开始画）
    _spline(ctx: any, pts: Array<{x: number, y: number}>) {
      for (let i = 0; i < pts.length - 1; i++) {
        const p0 = pts[Math.max(0, i - 1)];
        const p1 = pts[i];
        const p2 = pts[i + 1];
        const p3 = pts[Math.min(pts.length - 1, i + 2)];
        const c1x = p1.x + (p2.x - p0.x) / 6;
        const c1y = p1.y + (p2.y - p0.y) / 6;
        const c2x = p2.x - (p3.x - p1.x) / 6;
        const c2y = p2.y - (p3.y - p1.y) / 6;
        ctx.bezierCurveTo(c1x, c1y, c2x, c2y, p2.x, p2.y);
      }
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
