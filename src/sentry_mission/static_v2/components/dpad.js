const Dpad = {
  template: `
  <div class="dpad">
    <button class="btn-up" :disabled="store.mode !== 'MANUAL'" @mousedown="move(0.3, 0)" @mouseup="stop" @touchstart.prevent="move(0.3, 0)" @touchend="stop">&uarr;</button>
    <button class="btn-left" :disabled="store.mode !== 'MANUAL'" @mousedown="move(0, 0.5)" @mouseup="stop" @touchstart.prevent="move(0, 0.5)" @touchend="stop">&larr;</button>
    <button class="btn-center" :disabled="store.mode !== 'MANUAL'" @click="stop">&bull;</button>
    <button class="btn-right" :disabled="store.mode !== 'MANUAL'" @mousedown="move(0, -0.5)" @mouseup="stop" @touchstart.prevent="move(0, -0.5)" @touchend="stop">&rarr;</button>
    <button class="btn-down" :disabled="store.mode !== 'MANUAL'" @mousedown="move(-0.3, 0)" @mouseup="stop" @touchstart.prevent="move(-0.3, 0)" @touchend="stop">&darr;</button>
  </div>`,
  data() { return { linearScale: 0.3, angularScale: 0.5, interval: null }; },
  methods: {
    move(lin, ang) {
      if (store.mode !== 'MANUAL') return;
      publishCmdVel(lin * this.linearScale, ang * this.angularScale);
      clearInterval(this.interval);
      this.interval = setInterval(() => {
        if (store.mode !== 'MANUAL') {
          clearInterval(this.interval);
          return;
        }
        publishCmdVel(lin * this.linearScale, ang * this.angularScale);
      }, 100);
    },
    stop() {
      clearInterval(this.interval);
      if (store.mode !== 'MANUAL') return;
      publishCmdVel(0, 0);
    }
  },
  beforeUnmount() { clearInterval(this.interval); }
};
