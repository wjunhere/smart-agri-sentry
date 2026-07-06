const StatusBar = {
  template: `
  <div class="status-bar">
    <span>{{ store.missionState }}</span>
    <span v-if="store.missionCurrentAction">| {{ store.missionCurrentAction }}</span>
    <span>| DET: {{ store.missionPlantsDetected }}</span>
    <span>| ANZ: {{ store.missionPlantsAnalyzed }}</span>
    <span>| WP: {{ store.missionCurrentWpIdx + 1 }}/{{ store.missionTotalWps || '--' }}</span>
    <span class="progress-track">
      <span class="progress-fill" :style="{ width: (store.missionProgress * 100) + '%' }"></span>
    </span>
  </div>`
};
