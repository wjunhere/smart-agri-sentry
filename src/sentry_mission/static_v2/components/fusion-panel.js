// fusion-panel.js — 融合决策链面板
// 数据流可视化：输入三源 → 风险融合（贡献分解 + 判级刻度）→ 判定依据
// 输出层由下方的 advisory-card / forecast-panel 承接。

const FusionPanel = {
  template: `
  <div class="card fusion-card" :class="latest ? 'fusion-' + alertClass : ''">
    <h3>
      融合决策链
      <span class="fusion-age" v-if="latest">{{ age(fusionTs) }}</span>
    </h3>

    <!-- ① 输入层 -->
    <div class="fs-sources">
      <div class="fs-src">
        <span class="fs-dot" style="background:var(--amber)"></span>
        <div class="fs-src-main">
          <span class="fs-label">视觉巡航</span>
          <span class="fs-value">{{ visionText }}</span>
        </div>
        <span class="fs-age">{{ age(store.diagnosisTs) }}</span>
      </div>
      <div class="fs-src">
        <span class="fs-dot" style="background:var(--blue)"></span>
        <div class="fs-src-main">
          <span class="fs-label">田间节点<template v-if="store.envDataSource"> · {{ store.envDataSource }}</template></span>
          <span class="fs-value">{{ envText }}</span>
        </div>
        <span class="fs-age">{{ age(store.envTs) }}</span>
      </div>
      <div class="fs-src">
        <span class="fs-dot" style="background:var(--green)"></span>
        <div class="fs-src-main">
          <span class="fs-label">天气趋势</span>
          <span class="fs-value">{{ weatherText }}</span>
        </div>
        <span class="fs-age">{{ store.weatherStale ? '数据过期' : age(store.weatherTs) }}</span>
      </div>
    </div>

    <template v-if="latest">
      <div class="fs-arrow">▼ 融合引擎</div>

      <!-- ② 融合层 -->
      <div class="fs-score-row">
        <span class="fs-score" :class="alertClass">{{ riskPct }}</span>
        <div class="fs-score-side">
          <span class="fs-badge" :class="alertClass">{{ alertCn }}</span>
          <span class="fs-mode">{{ modeCn }}</span>
        </div>
      </div>

      <div class="risk-track">
        <div class="risk-seg" style="background:#F59E0B" :style="{width: segVisionW + '%'}"></div>
        <div class="risk-seg" style="background:#38BDF8" :style="{width: segEnvW + '%'}"></div>
        <div class="risk-seg" style="background:#A78BFA" :style="{width: segInterW + '%'}"></div>
        <div class="risk-tick" style="left:35%"></div>
        <div class="risk-tick" style="left:60%"></div>
        <div class="risk-tick tick-red" style="left:80%"></div>
      </div>
      <div class="risk-scale">
        <span>0</span><span>关注 35</span><span>预警 60</span><span>紧急 80</span><span>100</span>
      </div>
      <div class="risk-legend">
        <span><i class="lg-dot" style="background:#F59E0B"></i>视觉 {{ latest.vision_term.toFixed(2) }}</span>
        <span><i class="lg-dot" style="background:#38BDF8"></i>环境 {{ latest.env_term.toFixed(2) }}</span>
        <span><i class="lg-dot" style="background:#A78BFA"></i>交互 {{ latest.interaction_term.toFixed(2) }}</span>
      </div>
      <div class="fs-meta">
        数据充分度 {{ (latest.confidence * 100).toFixed(0) }}%
        <template v-if="latest.lwd_hours != null"> · 叶面湿润 {{ latest.lwd_hours.toFixed(1) }}h</template>
      </div>

      <!-- ③ 推理层 -->
      <div class="fs-evidence" v-if="latest.evidence_chain.length">
        <div class="fs-ev-title">判定依据</div>
        <div class="fs-ev-item" v-for="(e, i) in latest.evidence_chain" :key="i">
          <span class="fs-ev-dot"></span>
          <span class="fs-ev-text">{{ e }}</span>
        </div>
      </div>
      <div class="fs-arrow" v-if="store.advisoryText">▼ 决策输出 ↓</div>
    </template>
    <div v-else class="muted">等待融合数据（fusion_node 启动后每秒更新）...</div>
  </div>`,
  data() {
    return { nowTick: Date.now() };
  },
  computed: {
    latest() { return this.store.fusionLatest; },
    fusionTs() { return this.latest ? new Date(this.latest.time).getTime() : 0; },
    riskPct() { return this.latest ? (this.latest.risk_score * 100).toFixed(0) : '--'; },
    alertClass() { return (this.latest && this.latest.alert_level || 'NORMAL').toLowerCase(); },
    alertCn() { return ALERT_CN[(this.latest || {}).alert_level] || '--'; },
    modeCn() { return MODE_CN[(this.latest || {}).mode] || (this.latest || {}).mode || '--'; },
    segVisionW() { return this._segW(this.latest.vision_term); },
    segEnvW() { return this._segW(this.latest.env_term); },
    segInterW() { return this._segW(this.latest.interaction_term); },
    visionText() {
      if (!this.store.diagnosisDisease || this.store.diagnosisDisease === '--') return '--';
      const conf = this.store.diagnosisConfidence
        ? ` · 置信度 ${(this.store.diagnosisConfidence * 100).toFixed(1)}%` : '';
      return `${this.store.diagnosisDisease}${conf}`;
    },
    envText() {
      const parts = [];
      if (this.store.envAirHumidity != null) parts.push(`湿度${this.store.envAirHumidity.toFixed(1)}%`);
      if (this.store.envAirTemp != null) parts.push(`气温${this.store.envAirTemp.toFixed(1)}°C`);
      if (this.store.envSoilHumidity != null) parts.push(`土湿${this.store.envSoilHumidity.toFixed(1)}%`);
      if (this.store.envLeafWetness != null) parts.push(`叶湿${this.store.envLeafWetness.toFixed(1)}`);
      return parts.length ? parts.join(' · ') : '--';
    },
    weatherText() {
      const d = (this.store.weatherDays || [])[0];
      if (!d) return '--';
      return `${d.weather_desc || ''} ${d.temp_low}~${d.temp_high}°C · 降水${d.precipitation}mm`;
    },
  },
  methods: {
    _segW(term) {
      const l = this.latest;
      const total = (l.vision_term + l.env_term + l.interaction_term) || 1;
      return ((term / total) * l.risk_score * 100).toFixed(1);
    },
    age(ts) {
      void this.nowTick;  // 依赖 nowTick 触发定时重算
      if (!ts) return '无数据';
      const diff = Date.now() - ts;
      if (diff < 60e3) return '刚刚';
      if (diff < 3600e3) return `${Math.floor(diff / 60e3)}分钟前`;
      if (diff < 86400e3) return `${Math.floor(diff / 3600e3)}小时前`;
      return `${Math.floor(diff / 86400e3)}天前`;
    },
  },
  mounted() {
    this._ageTimer = setInterval(() => { this.nowTick = Date.now(); }, 30000);
  },
  beforeUnmount() { clearInterval(this._ageTimer); },
};
