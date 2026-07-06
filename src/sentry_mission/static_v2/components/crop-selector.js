const CropSelector = {
  template: `
  <div class="crop-selector">
    <label>作物</label>
    <div class="crop-pills">
      <span class="crop-pill" :class="{ active: store.cropType === 'tomato' }"
            @click="select('tomato')">番茄</span>
      <span class="crop-pill" :class="{ active: store.cropType === 'wheat' }"
            @click="select('wheat')">小麦</span>
      <span class="crop-pill" :class="{ active: store.cropType === 'strawberry' }"
            @click="select('strawberry')">草莓</span>
    </div>
    <span v-if="switching" class="switching">切换中...</span>
  </div>`,
  data() { return { switching: false }; },
  methods: {
    async select(crop) {
      if (crop === this.store.cropType) return;
      if (!confirm(`切换作物类型到 ${crop} 将重启相关节点，约 5-10 秒不可用。确定？`)) return;
      this.switching = true;
      try { await callSetCropType(crop); }
      catch (err) { alert('切换失败: ' + err.message); }
      this.switching = false;
    }
  }
};
