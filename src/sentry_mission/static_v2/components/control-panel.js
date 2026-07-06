const ControlPanel = {
  template: `
  <div class="control-panel">
    <dpad></dpad>
    <div class="estop-wrap">
      <button class="estop-btn" @click="emergencyStop">急停</button>
      <span class="estop-label">E-STOP</span>
    </div>
    <div style="flex:1"></div>
    <crop-selector></crop-selector>
    <cruise-panel></cruise-panel>
  </div>`,
  methods: {
    emergencyStop() {
      publishCmdVel(0, 0);
      callSetAutoMode(false);
    }
  }
};
