const CruisePanel = {
  template: `
  <div class="cruise-panel">
    <span class="cruise-label">自动巡航</span>
    <div class="wp-list">
      <div v-for="(label, i) in store.missionWaypointLabels" :key="i"
           class="wp-item"
           :class="{ active: i === store.missionCurrentWpIdx, done: i < store.missionCurrentWpIdx }"
           @click="toggleWp(i)">
        {{ label }}
      </div>
      <div v-if="store.missionWaypointLabels.length === 0" class="muted">无航点</div>
    </div>
    <button class="btn btn-resume" @click="store.showWpEditor = true">编辑航点</button>
    <button class="btn btn-resume" @click="preheatCruise" :disabled="store.mode === 'AUTO' || store.stackPreheating || store.stackStarting || store.stackReady">{{ preheatLabel }}</button>
    <button v-if="store.mode !== 'AUTO'" class="btn btn-go" @click="startCruise" :disabled="store.stackStarting || store.stackPreheating">{{ store.stackStarting ? '启动中...' : '启动巡航' }}</button>
    <button v-if="store.mode === 'AUTO'" class="btn btn-pause" @click="pauseCruise">暂停</button>
  </div>`,
  computed: {
    preheatLabel() {
      if (store.stackPreheating) return '预热中...';
      if (store.mode === 'AUTO' || store.stackReady) return '正常运行中';
      return '预热模式';
    }
  },
  methods: {
    preheatCruise() { callStackPreheat().catch(err => console.error(err)); },
    startCruise() { callStackStart().catch(err => console.error(err)); },
    pauseCruise() { callStackStop().catch(err => console.error(err)); },
    toggleWp(idx) {
      // UI only - backend doesn't support per-waypoint skipping yet
    }
  }
};
