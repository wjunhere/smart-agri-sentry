const CropSelector = {
  template: `
  <div class="crop-selector">
    <label>作物:</label>
    <select :value="store.cropType" @change="onChange">
      <option value="tomato">番茄</option>
      <option value="wheat">小麦</option>
      <option value="strawberry">草莓</option>
    </select>
    <span v-if="switching" class="switching">切换中...</span>
  </div>`,
  data() { return { switching: false }; },
  methods: {
    async onChange(e) {
      const crop = e.target.value;
      if (crop === this.store.cropType) return;
      if (!confirm(`切换作物类型到 ${crop} 将重启相关节点，约 5-10 秒不可用。确定？`)) {
        e.target.value = this.store.cropType;
        return;
      }
      this.switching = true;
      try {
        await callSetCropType(crop);
      } catch (err) {
        alert('切换失败: ' + err.message);
      }
      this.switching = false;
    }
  }
};
