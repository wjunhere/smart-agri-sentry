const TopBar = {
  template: `
  <header class="top-bar">
    <span class="logo">智农哨兵</span>
    <span class="badge" :class="store.mode === 'AUTO' ? 'badge-auto' : 'badge-manual'">
      {{ store.mode }}
    </span>
    <span class="indicator">
      <span class="dot" :class="store.connected ? 'dot-green' : 'dot-red'"></span>
      {{ store.connected ? 'ROS 在线' : 'ROS 离线' }}
    </span>
    <span class="battery" v-if="store.batteryVoltage !== null">
      🔋 {{ store.batteryVoltage.toFixed(1) }}V
    </span>
    <span class="lora" v-if="store.envDataSource">
      📡 {{ store.envDataSource }}
    </span>
  </header>`
};
