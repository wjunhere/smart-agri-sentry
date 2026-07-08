const DiagnosisToggle = {
  template: `
  <div class="card diag-toggle-section">
    <span class="diag-toggle-label">病害模拟</span>
    <div class="diag-toggle-pills">
      <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'real' }"
              @click="store.mockDiagnosisMode = 'real'">真实数据</button>
      <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'healthy' }"
              @click="store.mockDiagnosisMode = 'healthy'">Healthy</button>
      <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'early_blight' }"
              @click="store.mockDiagnosisMode = 'early_blight'">Early Blight</button>
    </div>
    <span class="diag-toggle-hint">↓ 下滑可见，默认隐藏</span>
  </div>`
};
