const ForecastPanel = {
  template: `
  <div class="card">
    <h3>预警趋势</h3>
    <canvas ref="chart" height="160"></canvas>
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
    recentAlerts() {
      return this.store.forecastAlerts.slice(-20).reverse();
    }
  },
  methods: {
    formatTime(iso) {
      const d = new Date(iso);
      return d.toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit' });
    },
    renderChart() {
      const ctx = this.$refs.chart;
      if (!ctx) return;
      if (this._chart) this._chart.destroy();
      const data = this.store.forecastAlerts.slice(-50);
      if (data.length === 0) return;
      this._chart = new Chart(ctx, {
        type: 'line',
        data: {
          labels: data.map(a => this.formatTime(a.time)),
          datasets: [{
            label: '风险值',
            data: data.map(a => a.probability || 0),
            borderColor: '#4fc3f7',
            backgroundColor: 'rgba(79,195,247,0.1)',
            fill: true,
            pointRadius: 3,
            pointBackgroundColor: data.map(a => {
              const level = a.alert_type || 'NORMAL';
              return { NORMAL: '#4caf50', SUSPICION: '#ff9800', WARNING: '#ff9800', CRITICAL: '#f44336' }[level] || '#4caf50';
            }),
            tension: 0.3,
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          plugins: { legend: { display: false } },
          scales: {
            y: { min: 0, max: 1, ticks: { color: '#5a6a8a' }, grid: { color: '#1e2d45' } },
            x: { ticks: { color: '#5a6a8a', maxTicksLimit: 8 }, grid: { display: false } }
          }
        }
      });
    }
  },
  watch: {
    'store.forecastAlerts.length'() { this.$nextTick(() => this.renderChart()); }
  },
  mounted() { this.$nextTick(() => this.renderChart()); }
};
