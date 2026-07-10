const WeatherPanel = {
  data() {
    return { collapsed: true };
  },
  template: `
  <div class="card weather-card">
    <h3 class="weather-header" @click="collapsed = !collapsed" style="cursor:pointer; user-select:none;">
      <span class="collapse-arrow">{{ collapsed ? '▸' : '▾' }}</span>
      未来天气
      <span class="count-badge danger" v-if="store.weatherDisasterAlerts.length > 0">
        {{ store.weatherDisasterAlerts.length }}预警
      </span>
      <span class="stale-badge" v-if="store.weatherStale">缓存</span>
      <span style="margin-left:auto; display:flex; gap:4px; align-items:center;">
        <input type="number" :value="store.weatherLat" step="0.01"
               @change="e => store.weatherLat = parseFloat(e.target.value)"
               @click.stop
               style="width:64px; background:#1E293B; border:1px solid #334155; color:#F8FAFC; padding:2px 6px; border-radius:3px; font-size:10px; font-family:JetBrains Mono;" placeholder="纬度">
        <input type="number" :value="store.weatherLon" step="0.01"
               @change="e => store.weatherLon = parseFloat(e.target.value)"
               @click.stop
               style="width:64px; background:#1E293B; border:1px solid #334155; color:#F8FAFC; padding:2px 6px; border-radius:3px; font-size:10px; font-family:JetBrains Mono;" placeholder="经度">
      </span>
    </h3>
    <div v-show="!collapsed">
      <div ref="chart" class="forecast-chart" style="height:140px"></div>
      <div class="weather-days">
        <div v-for="d in store.weatherDays" :key="d.day_offset"
             class="weather-day-row">
          <span class="day-label">{{ dayLabel(d.day_offset) }}</span>
          <span class="day-icon">{{ weatherIcon(d.weather_desc) }}</span>
          <span class="day-desc">{{ d.weather_desc }}</span>
          <span class="day-temp">{{ (d.temp_low || 0).toFixed(0) }}° / {{ (d.temp_high || 0).toFixed(0) }}°</span>
          <span class="day-rain" v-if="d.precipitation > 0">{{ (d.precipitation || 0).toFixed(0) }}mm</span>
        </div>
      </div>
      <div v-if="store.weatherDisasterAlerts.length > 0" class="disaster-alerts">
        <div v-for="a in store.weatherDisasterAlerts" :key="a" class="disaster-tag">
          {{ a }}
        </div>
      </div>
    </div>
  </div>`,
  methods: {
    dayLabel(offset) {
      if (offset === 0) return '今天';
      if (offset === 1) return '明天';
      const d = new Date();
      d.setDate(d.getDate() + offset);
      return ['周日', '周一', '周二', '周三', '周四', '周五', '周六'][d.getDay()];
    },
    weatherIcon(desc) {
      const map = { '晴': '☀️', '多云': '⛅', '阴': '☁️', '小雨': '🌧️',
                    '中雨': '🌧️', '暴雨': '⛈️', '雨': '🌧️', '雪': '❄️' };
      for (const [k, v] of Object.entries(map)) {
        if (desc && desc.includes(k)) return v;
      }
      return '🌤️';
    },
    renderChart() {
      const dom = this.$refs.chart;
      if (!dom) return;
      if (this._chart) this._chart.dispose();

      const days = this.store.weatherDays;
      if (!days || days.length === 0) { this._chart = null; return; }

      this._chart = echarts.init(dom, null, { devicePixelRatio: 2 });
      const labels = days.map(d => this.dayLabel(d.day_offset));
      const highs = days.map(d => d.temp_high);
      const lows = days.map(d => d.temp_low);
      const rain = days.map(d => d.precipitation);

      this._chart.setOption({
        grid: { top: 10, right: 50, bottom: 24, left: 40 },
        xAxis: {
          type: 'category', data: labels,
          axisLabel: { color: '#64748B', fontSize: 10, fontFamily: 'JetBrains Mono' },
          axisLine: { lineStyle: { color: '#1F2937' } },
        },
        yAxis: [
          {
            type: 'value', name: '°C',
            axisLabel: { color: '#64748B', fontSize: 10 },
            splitLine: { lineStyle: { color: '#1F2937', type: 'dashed' } },
          },
          {
            type: 'value', name: 'mm',
            axisLabel: { color: '#64748B', fontSize: 10 },
            splitLine: { show: false },
          },
        ],
        series: [
          {
            type: 'line', data: highs, name: '最高温',
            lineStyle: { color: '#EF4444', width: 2 },
            symbol: 'circle', symbolSize: 4,
            itemStyle: { color: '#EF4444' },
            smooth: true,
          },
          {
            type: 'line', data: lows, name: '最低温',
            lineStyle: { color: '#38BDF8', width: 2 },
            symbol: 'circle', symbolSize: 4,
            itemStyle: { color: '#38BDF8' },
            smooth: true,
          },
          {
            type: 'bar', data: rain, name: '降水', yAxisIndex: 1,
            itemStyle: { color: '#6366F1' },
            barWidth: 12,
          },
        ],
        tooltip: {
          trigger: 'axis',
          backgroundColor: '#0F172A',
          borderColor: '#1F2937',
          textStyle: { color: '#F8FAFC', fontSize: 11 },
        },
      });
    },
  },
  watch: {
    'store.weatherDays': {
      deep: true,
      handler() { this.$nextTick(() => this.renderChart()); },
    },
    collapsed(val) {
      if (!val) this.$nextTick(() => this.renderChart());
    },
  },
  mounted() { this.$nextTick(() => this.renderChart()); },
  beforeUnmount() { if (this._chart) this._chart.dispose(); },
};
