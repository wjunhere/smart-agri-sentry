const DiagnosisCard = {
  template: `
  <div class="card">
    <h3>病害分类 · {{ store.cropType }}</h3>
    <div v-if="store.diagnosisDisease">
      <span class="value disease">{{ store.diagnosisDisease }}</span>
      <div class="stat">置信度: {{ (store.diagnosisConfidence * 100).toFixed(1) }}%</div>
      <div class="probabilities" v-if="store.diagnosisProbabilities.length">
        <div v-for="(p, i) in store.diagnosisProbabilities.slice(0, 3)" class="prob-row">
          <span class="label">{{ i }}</span>
          <span class="bar" :style="{width: (p * 100) + '%'}"></span>
          <span>{{ (p * 100).toFixed(0) }}%</span>
        </div>
      </div>
    </div>
    <div v-else class="muted">等待诊断结果...</div>
  </div>`
};
