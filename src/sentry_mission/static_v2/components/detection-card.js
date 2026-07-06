const DetectionCard = {
  template: `
  <div class="card">
    <h3>
      YOLO 植株检测
      <span class="status-dot" :class="store.plantDetected ? 'green' : 'grey'"></span>
    </h3>
    <div v-if="store.plantDetected">
      <span class="value">{{ (store.plantConfidence * 100).toFixed(0) }}%</span>
      <div class="stat">置信度 · 叶片面积比 {{ (store.plantAreaRatio * 100).toFixed(1) }}%</div>
    </div>
    <div v-else>
      <span class="value" style="color:var(--text-muted)">--</span>
      <div class="stat">未检测到植株</div>
    </div>
  </div>`
};
