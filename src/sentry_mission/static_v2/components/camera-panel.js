const CameraPanel = {
  template: `
  <div class="camera-panel">
    <div class="panel-header">
      <span class="rec-dot" v-if="store.plantDetected"></span>
      实时画面 · {{ store.envDataSource || '固定环境节点' }}
      <span style="margin-left:auto;font-size:10px;color:var(--text-muted)">
        {{ store.cameraFrame ? 'STREAMING' : 'NO SIGNAL' }}
      </span>
    </div>
    <div class="camera-container" :class="{ online: store.cameraFrame }">
      <canvas ref="canvas" width="640" height="480" v-show="store.cameraFrame"></canvas>
      <div v-if="!store.cameraFrame" class="camera-placeholder">
        <div class="loader-ring"></div>
        正在建立视频流...
      </div>
      <div v-if="store.plantDetected" class="detection-badge">
        检测到植株 {{ (store.plantConfidence * 100).toFixed(0) }}%
      </div>
    </div>
  </div>`,
  data() { return { image: new Image() }; },
  watch: {
    'store.cameraFrame'(src) {
      if (!src) return;
      this.image.onload = () => this.drawFrame();
      this.image.src = src;
    },
    'store.plantDetected'() { if (this.store.cameraFrame) this.drawFrame(); },
    'store.plantBbox'() { if (this.store.cameraFrame) this.drawFrame(); },
  },
  methods: {
    drawFrame() {
      const canvas = this.$refs.canvas;
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const iw = this.image.naturalWidth, ih = this.image.naturalHeight;
      canvas.width = iw; canvas.height = ih;
      ctx.drawImage(this.image, 0, 0);
      if (this.store.plantDetected && this.store.plantBbox.length === 4) {
        const [x1, y1, x2, y2] = this.store.plantBbox;
        ctx.strokeStyle = '#10B981';
        ctx.lineWidth = 3;
        ctx.strokeRect(x1 * iw, y1 * ih, (x2 - x1) * iw, (y2 - y1) * ih);
        ctx.fillStyle = '#10B981';
        ctx.font = '14px "JetBrains Mono", monospace';
        ctx.fillText(
          `Plant ${(this.store.plantConfidence * 100).toFixed(0)}%`,
          x1 * iw, y1 * ih - 5
        );
      }
    }
  }
};
