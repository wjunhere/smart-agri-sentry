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
              <td><button class="wp-del" @click="remove(i)">✕</button></td>
            </tr>
          </tbody>
        </table>
        <div v-if="editing.length === 0" class="muted" style="text-align:center;padding:20px">
          暂无航点，点击下方按钮添加
        </div>
      </div>
      <div class="wp-editor-actions">
        <button class="btn btn-resume" @click="add">+ 添加航点</button>
        <span class="wp-count">{{ editing.length }} 个航点</span>
        <button class="btn btn-go" @click="save">保存</button>
        <button class="btn btn-pause" @click="close">取消</button>
      </div>
    </div>
  </div>`,
  props: { visible: Boolean },
  emits: ['close', 'save'],
  data() {
    return {
      editing: [],
      currentIdx: -1,
    };
  },
  watch: {
    visible(val) {
      if (val) {
        // Deep-copy current waypoints when opening
        this.currentIdx = store.missionCurrentWpIdx;
        this.editing = (store._rawWaypoints || []).map(wp => ({
          x: wp.x, y: wp.y, deg: Math.round(wp.yaw * 180 / Math.PI)
        }));
        if (this.editing.length === 0) {
          // Default: 2 rows serpentine
          this.editing = [
            { x: 2.5, y: 0.0, deg: 0 },
            { x: 2.5, y: 0.6, deg: 90 },
            { x: 0.0, y: 0.6, deg: 180 },
          ];
        }
      }
    }
  },
  methods: {
    add() {
      this.editing.push({ x: 0, y: 0, deg: 0 });
    },
    remove(i) {
      this.editing.splice(i, 1);
    },
    save() {
      store._rawWaypoints = this.editing.map(wp => ({
        x: wp.x, y: wp.y, yaw: wp.deg * Math.PI / 180
      }));
      store.missionWaypointLabels = this.editing.map((wp, i) =>
        `WP${i}: (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})`
      );
      store.missionTotalWps = this.editing.length;
      this.$emit('save', store._rawWaypoints);
      this.$emit('close');
    },
    close() {
      this.$emit('close');
    }
  }
};
