const Dpad = {
  template: `
  <div class="dpad">
    <button class="btn-up" @mousedown="move(0.3, 0)" @mouseup="stop" @touchstart.prevent="move(0.3, 0)" @touchend="stop">▲</button>
    <button class="btn-left" @mousedown="move(0, 0.5)" @mouseup="stop" @touchstart.prevent="move(0, 0.5)" @touchend="stop">◀</button>
    <button class="btn-center" @click="stop">·</button>
    <button class="btn-right" @mousedown="move(0, -0.5)" @mouseup="stop" @touchstart.prevent="move(0, -0.5)" @touchend="stop">▶</button>
    <button class="btn-down" @mousedown="move(-0.3, 0)" @mouseup="stop" @touchstart.prevent="move(-0.3, 0)" @touchend="stop">▼</button>
  </div>`,
  data() { return { linearScale: 0.3, angularScale: 0.5, interval: null }; },
  methods: {
    move(lin, ang) {
      publishCmdVel(lin * this.linearScale, ang * this.angularScale);
      clearInterval(this.interval);
      this.interval = setInterval(() => {
        publishCmdVel(lin * this.linearScale, ang * this.angularScale);
      }, 100);
    },
    stop() {
      clearInterval(this.interval);
      publishCmdVel(0, 0);
    }
  },
  beforeUnmount() { clearInterval(this.interval); }
};
