const TopBar = {
  template: `
  <header class="top-bar">
    <span class="logo">智农哨兵</span>
    <span class="status-capsule" :class="{ manual: store.mode === 'MANUAL', estop: store.missionState === 'ESTOP' }">
      <span class="dot" :class="store.connected ? 'dot-green' : 'dot-red'"></span>
      {{ store.mode === 'AUTO' ? 'AUTO' : store.mode === 'MANUAL' ? 'MANUAL' : 'ESTOP' }}
    </span>
    <button class="camera-start-btn" :disabled="store.visionStarting || store.stackReady"
            @click="startVision">
      {{ store.visionStarting ? '启动中...' : store.stackReady ? '摄像头已开启' : '开启摄像头' }}
    </button>
    <button class="camera-capture-btn" :disabled="store.cameraCaptureBusy"
            @click="captureImage">
      {{ store.cameraCaptureBusy ? '保存中...' : '拍摄' }}
    </button>
    <span class="spacer"></span>
    <span class="ros-indicator">
      <span class="dot" :class="store.connected ? 'dot-green' : 'dot-red'"></span>
      {{ store.connected ? 'ROS ONLINE' : 'ROS OFFLINE' }}
    </span>
    <span class="battery-row" v-if="store.batteryVoltage !== null">
      <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <rect x="1" y="6" width="18" height="12" rx="2"/><line x1="23" y1="10" x2="23" y2="14"/>
      </svg>
      {{ store.batteryVoltage.toFixed(1) }}V
    </span>
    <span class="alarm-badge" v-if="alarmCount > 0">{{ alarmCount }}</span>
    <span class="time">{{ timeStr }}</span>
  </header>`,
  data() {
    return { timeStr: '', timer: null };
  },
  computed: {
    alarmCount() {
      return this.store.fusionResults.filter(
        r => r.alert_level === 'WARNING' || r.alert_level === 'CRITICAL'
      ).length;
    }
  },
  methods: {
    startVision() {
      callVisionStart().catch(err => console.error(err));
    },
    captureImage() {
      callCaptureImage().catch(err => console.error(err));
    },
    updateTime() {
      const d = new Date();
      this.timeStr = d.toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    }
  },
  mounted() {
    this.updateTime();
    this.timer = setInterval(() => this.updateTime(), 1000);
  },
  beforeUnmount() { clearInterval(this.timer); }
};
