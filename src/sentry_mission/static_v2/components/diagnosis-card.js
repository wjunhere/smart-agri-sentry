const DiagnosisCard = {
  template: `
  <div class="card">
    <h3>
      病害分类 · {{ store.cropType.toUpperCase() }}
      <span class="status-dot" :class="store.plantDetected ? (store.diagnosisDisease === 'healthy' ? 'green' : 'red') : 'grey'"></span>
    </h3>
    <div v-if="!store.plantDetected">
      <span class="value" style="color:var(--text-muted)">无</span>
      <div class="stat">未检测到植株，无法分类</div>
    </div>
    <div v-else-if="store.diagnosisDisease">
      <span class="value disease">{{ store.diagnosisDisease }}</span>
      <div class="stat">置信度 {{ (store.diagnosisConfidence * 100).toFixed(1) }}%</div>
      <div class="probabilities" v-if="store.diagnosisProbabilities.length">
        <div v-for="(p, i) in store.diagnosisProbabilities.slice(0, 3)" class="prob-row">
          <span style="min-width:20px;color:var(--text-muted)">#{{ i }}</span>
          <span class="bar" :style="{width: (p * 100) + '%'}"></span>
          <span>{{ (p * 100).toFixed(0) }}%</span>
        </div>
      </div>
    </div>
    <div v-else class="muted">等待诊断结果...</div>
  </div>`
};
