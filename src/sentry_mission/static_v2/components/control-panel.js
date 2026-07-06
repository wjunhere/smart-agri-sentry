const ControlPanel = {
  template: `
  <div class="control-panel">
    <dpad></dpad>
    <div class="estop-wrap">
      <button class="estop-btn" @click="emergencyStop">急停</button>
      <span class="estop-label">E-STOP</span>
    </div>
    <crop-selector></crop-selector>
    <div style="flex:1"></div>
    <cruise-panel></cruise-panel>
  </div>`,
  methods: {
    emergencyStop() {
      publishCmdVel(0, 0);
      callSetAutoMode(false);
    }
  }
};
