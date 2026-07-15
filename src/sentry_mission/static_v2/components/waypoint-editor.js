const WaypointEditor = {
  template: `
  <div class="modal-overlay" v-if="visible" @click.self="close">
    <div class="modal" style="max-width: 560px;">
      <h2>航点编辑器</h2>
      <div class="wp-table-wrap">
        <table class="wp-table">
          <thead>
            <tr>
              <th style="width:32px">#</th>
              <th>X (m)</th>
              <th>Y (m)</th>
              <th>Yaw (°)</th>
              <th style="width:40px"></th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="(wp, i) in editing" :key="i" :class="{ active: i === currentIdx }">
              <td class="idx">{{ i }}</td>
              <td><input type="number" step="0.1" v-model.number="wp.x" /></td>
              <td><input type="number" step="0.1" v-model.number="wp.y" /></td>
              <td><input type="number" step="1" v-model.number="wp.deg" /></td>
              <td><button class="wp-del" @click="remove(i)">X</button></td>
            </tr>
          </tbody>
        </table>
        <div v-if="loading" class="muted" style="text-align:center;padding:20px">正在读取航点...</div>
        <div v-else-if="editing.length === 0" class="muted" style="text-align:center;padding:20px">
          暂无航点，点击下方按钮添加
        </div>
        <div v-if="error" class="muted" style="color:#EF4444;text-align:center;padding:8px">{{ error }}</div>
      </div>
      <div class="wp-editor-actions">
        <button class="btn btn-resume" @click="add" :disabled="saving">+ 添加航点</button>
        <span class="wp-count">{{ editing.length }} 个航点</span>
        <button class="btn btn-go" @click="save" :disabled="saving || loading">{{ saving ? '保存中...' : '保存' }}</button>
        <button class="btn btn-pause" @click="close" :disabled="saving">取消</button>
      </div>
    </div>
  </div>`,
  props: { visible: Boolean },
  emits: ['close', 'save'],
  data() {
    return {
      editing: [],
      currentIdx: -1,
      loading: false,
      saving: false,
      error: '',
    };
  },
  watch: {
    visible(val) {
      if (val) this.loadWaypoints();
    }
  },
  methods: {
    defaultWaypoints() {
      return [
        { x: 2.5, y: 0.0, yaw: 1.5708 },
        { x: 2.5, y: 0.6, yaw: 3.1416 },
        { x: 0.0, y: 0.6, yaw: 3.1416 },
      ];
    },
    toEditing(waypoints) {
      return (waypoints || []).map(wp => ({
        x: Number(wp.x),
        y: Number(wp.y),
        deg: Math.round(Number(wp.yaw) * 180 / Math.PI),
      }));
    },
    loadWaypoints() {
      this.currentIdx = store.missionCurrentWpIdx;
      this.error = '';
      this.loading = true;
      callGetWaypoints()
        .then(waypoints => {
          const data = waypoints.length ? waypoints : this.defaultWaypoints();
          this.editing = this.toEditing(data);
        })
        .catch(err => {
          console.error(err);
          this.error = err.message || '读取航点失败';
          const data = (store._rawWaypoints || []).length ? store._rawWaypoints : this.defaultWaypoints();
          this.editing = this.toEditing(data);
        })
        .finally(() => { this.loading = false; });
    },
    add() {
      this.editing.push({ x: 0, y: 0, deg: 0 });
    },
    remove(i) {
      this.editing.splice(i, 1);
    },
    save() {
      this.error = '';
      this.saving = true;
      const waypoints = this.editing.map(wp => ({
        x: Number(wp.x),
        y: Number(wp.y),
        yaw: Number(wp.deg) * Math.PI / 180,
      }));
      callSaveWaypoints(waypoints)
        .then(saved => {
          this.$emit('save', saved);
          this.$emit('close');
        })
        .catch(err => {
          console.error(err);
          this.error = err.message || '保存航点失败';
        })
        .finally(() => { this.saving = false; });
    },
    close() {
      this.$emit('close');
    }
  }
};
