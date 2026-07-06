const StatusBar = {
  template: `
  <div class="status-bar">
    <span>状态: <strong>{{ store.missionState }}</strong></span>
    <span v-if="store.missionCurrentAction">| {{ store.missionCurrentAction }}</span>
    <span>| 检测: {{ store.missionPlantsDetected }} | 分析: {{ store.missionPlantsAnalyzed }}</span>
    <span>| 进度: {{ (store.missionProgress * 100).toFixed(0) }}%</span>
  </div>`
};
