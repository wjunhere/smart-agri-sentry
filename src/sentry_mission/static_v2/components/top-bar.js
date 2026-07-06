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
      <svg class="battery-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <rect x="1" y="6" width="18" height="12" rx="2"/>
        <line x1="23" y1="10" x2="23" y2="14"/>
      </svg>
      {{ store.batteryVoltage.toFixed(1) }}V
    </span>
    <span class="lora" v-if="store.envDataSource">
      <svg class="signal-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <circle cx="12" cy="16" r="2"/>
        <line x1="12" y1="16" x2="12" y2="22"/>
        <path d="M7 10a5 5 0 0 1 10 0"/>
      </svg>
      {{ store.envDataSource }}
    </span>
  </header>`
};
