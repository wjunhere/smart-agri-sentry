const ForecastPanel = {
  template: `
  <div class="card forecast-card">
    <h3>
      预警趋势
      <span class="count-badge" v-if="alarmCount > 0">{{ alarmCount }}</span>
    </h3>
    <div ref="chart" class="forecast-chart"></div>
    <div class="alert-list">
      <div v-for="(alert, i) in recentAlerts" :key="i"
           class="alert-row" :class="'alert-level-' + (alert.alert_type || 'NORMAL')"
           @click="store.selectedAlert = alert">
        <span class="alert-time">{{ formatTime(alert.time) }}</span>
        <span class="alert-desc">{{ alert.description || alert.alert_type }}</span>
        <span class="alert-risk" v-if="alert.probability">
          {{ (alert.probability * 100).toFixed(0) }}%
        </span>
      </div>
      <div v-if="recentAlerts.length === 0" class="muted">暂无预警记录</div>
    </div>
  </div>`,
  computed: {
    recentAlerts() { return this.store.forecastAlerts.slice(-20).reverse(); },
    alarmCount() {
      return this.store.fusionResults.filter(
        r => r.alert_level === 'WARNING' || r.alert_level === 'CRITICAL'
      ).length;
    }
  },
  methods: {
    formatTime(iso) {
      const d = new Date(iso);
      return d.toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit' });
    },
    renderChart() {
      const dom = this.$refs.chart;
      if (!dom) return;
      if (this._chart) this._chart.dispose();
      const data = this.store.forecastAlerts.slice(-50);
      if (data.length === 0) { this._chart = null; return; }

      this._chart = echarts.init(dom, null, { devicePixelRatio: 2 });
      const labels = data.map(a => this.formatTime(a.time));
      const values = data.map(a => a.probability || 0);
      const colors = data.map(a => {
        const level = a.alert_type || 'NORMAL';
        return { NORMAL: '#10B981', SUSPICION: '#F59E0B', WARNING: '#F59E0B', CRITICAL: '#EF4444' }[level] || '#10B981';
      });

      this._chart.setOption({
        grid: { top: 10, right: 20, bottom: 24, left: 40 },
        xAxis: {
          type: 'category', data: labels,
          axisLine: { lineStyle: { color: '#1F2937' } },
          axisLabel: { color: '#64748B', fontSize: 10, fontFamily: 'JetBrains Mono', interval: 'auto' },
        },
        yAxis: {
          type: 'value', min: 0, max: 1,
          axisLine: { show: false },
          axisTick: { show: false },
          splitLine: { lineStyle: { color: '#1F2937', type: 'dashed' } },
          axisLabel: { color: '#64748B', fontSize: 10, fontFamily: 'JetBrains Mono' },
        },
        series: [{
          type: 'line', data: values,
          lineStyle: { color: '#38BDF8', width: 2 },
          symbol: 'circle', symbolSize: 6,
          itemStyle: { color: (p) => colors[p.dataIndex] },
          areaStyle: { color: new echarts.graphic.LinearGradient(0,0,0,1, [
            { offset: 0, color: 'rgba(56,189,248,0.2)' },
            { offset: 1, color: 'rgba(56,189,248,0.02)' }
          ])},
          smooth: true, smoothMonotone: 'x',
          animationDuration: 800,
        }],
        tooltip: {
          trigger: 'axis',
          backgroundColor: '#0F172A',
          borderColor: '#1F2937',
          textStyle: { color: '#F8FAFC', fontSize: 11, fontFamily: 'JetBrains Mono' },
        },
      });
    }
  },
  watch: {
    'store.forecastAlerts.length'() { this.$nextTick(() => this.renderChart()); }
  },
  mounted() { this.$nextTick(() => this.renderChart()); },
  beforeUnmount() { if (this._chart) this._chart.dispose(); }
};
