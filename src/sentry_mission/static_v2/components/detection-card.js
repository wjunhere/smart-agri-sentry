const DetectionCard = {
  template: `
  <div class="card">
    <h3>YOLO 植株检测</h3>
    <div v-if="store.plantDetected">
      <span class="value">检测到植株</span>
      <div class="stat">置信度: {{ (store.plantConfidence * 100).toFixed(1) }}%</div>
      <div class="stat">叶片面积比: {{ (store.plantAreaRatio * 100).toFixed(1) }}%</div>
    </div>
    <div v-else class="muted">未检测到植株</div>
  </div>`
};
