const CruisePanel = {
  template: `
  <div class="cruise-panel">
    <div class="wp-list">
      <div v-for="(label, i) in store.missionWaypointLabels" :key="i"
           class="wp-item"
           :class="{ active: i === store.missionCurrentWpIdx, done: i < store.missionCurrentWpIdx }"
           @click="toggleWp(i)">
        {{ label }}
      </div>
      <div v-if="store.missionWaypointLabels.length === 0" class="muted">无航点数据</div>
    </div>
    <button v-if="store.mode !== 'AUTO'" class="btn btn-go" @click="startCruise">启动巡航</button>
    <button v-if="store.mode === 'AUTO'" class="btn btn-pause" @click="pauseCruise">暂停</button>
    <button v-if="store.mode === 'MANUAL'" class="btn btn-resume" @click="resumeCruise">恢复</button>
  </div>`,
  methods: {
    startCruise() { callSetAutoMode(true); },
    pauseCruise() { callSetAutoMode(false); },
    resumeCruise() { publishResumeNavigation(); },
    toggleWp(idx) {
      // UI only — backend doesn't support per-waypoint skipping yet
    }
  }
};
