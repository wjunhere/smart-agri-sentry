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
    <label class="cruise-speed-control">
      <span>巡航速度</span>
      <input v-model.number="store.cruiseSpeed" type="number" min="0.05" max="0.35" step="0.01"
             :disabled="store.cruiseSpeedBusy">
      <span>m/s</span>
      <button class="btn btn-resume" @click="applyCruiseSpeed"
              :disabled="store.cruiseSpeedBusy">应用</button>
    </label>
    <button class="btn" :class="store.stackReady ? 'btn-pause' : 'btn-resume'" @click="onPreheatOrShutdown" :disabled="preheatDisabled">{{ preheatLabel }}</button>
    <button v-if="store.mode !== 'AUTO'" class="btn btn-go" @click="startCruise" :disabled="store.stackStarting || store.stackPreheating || store.stackShuttingDown">{{ store.stackStarting ? '启动中...' : '启动巡航' }}</button>
    <button v-if="store.mode === 'AUTO'" class="btn btn-pause" @click="pauseCruise">结束巡航</button>
  </div>`,
  computed: {
    preheatLabel() {
      if (store.stackShuttingDown) return '结束中...';
      if (store.stackPreheating) return '预热中...';
      if (store.stackReady) return '结束栈';
      return '预热模式';
    },
    preheatDisabled() {
      // 巡航中不允许动栈；栈操作进行中也不允许重复触发
      return store.mode === 'AUTO' || store.stackPreheating
        || store.stackStarting || store.stackShuttingDown;
    }
  },
  methods: {
    onPreheatOrShutdown() {
      const p = store.stackReady ? callStackShutdown() : callStackPreheat();
      p.catch(err => console.error(err));
    },
    startCruise() { callStackStart().catch(err => console.error(err)); },
    pauseCruise() { callStackStop().catch(err => console.error(err)); },
    applyCruiseSpeed() { callSetCruiseSpeed(store.cruiseSpeed).catch(err => console.error(err)); },
    toggleWp(idx) {
      // UI only - backend doesn't support per-waypoint skipping yet
    }
  }
};
