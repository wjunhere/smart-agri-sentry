import { getStore, updateStore } from '../../services/store';

Component({
  properties: {
    visible: { type: Boolean, value: false },
  },
  data: {
    waypoints: [] as Array<{x: number, y: number, deg: number}>,
    currentWpIdx: -1,
  },
  observers: {
    'visible': function(val: boolean) {
      if (!val) return;
      const s = getStore();
      const raw = (s as any)._rawWaypoints || [];
      let wps: Array<{x: number, y: number, deg: number}>;
      if (raw.length > 0) {
        wps = raw.map((wp: any) => ({
          x: wp.x,
          y: wp.y,
          deg: Math.round(wp.yaw * 180 / Math.PI),
        }));
      } else {
        wps = [
          { x: 2.5, y: 0.0, deg: 0 },
          { x: 2.5, y: 0.6, deg: 90 },
          { x: 0.0, y: 0.6, deg: 180 },
        ];
      }
      this.setData({
        waypoints: wps,
        currentWpIdx: s.missionCurrentWpIdx,
      });
    },
  },
  methods: {
    onXChange(e: any) {
      const idx = e.currentTarget.dataset.index;
      this.data.waypoints[idx].x = parseFloat(e.detail.value) || 0;
    },
    onYChange(e: any) {
      const idx = e.currentTarget.dataset.index;
      this.data.waypoints[idx].y = parseFloat(e.detail.value) || 0;
    },
    onDegChange(e: any) {
      const idx = e.currentTarget.dataset.index;
      this.data.waypoints[idx].deg = parseFloat(e.detail.value) || 0;
    },
    onAdd() {
      const wps = [...this.data.waypoints, { x: 0, y: 0, deg: 0 }];
      this.setData({ waypoints: wps });
    },
    onRemove(e: any) {
      const idx = e.currentTarget.dataset.index;
      const wps = this.data.waypoints.filter((_, i) => i !== idx);
      this.setData({ waypoints: wps });
    },
    onSave() {
      const rawWps = this.data.waypoints.map(wp => ({
        x: wp.x,
        y: wp.y,
        yaw: wp.deg * Math.PI / 180,
      }));
      const labels = this.data.waypoints.map((wp, i) =>
        `WP${i}: (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})`
      );
      (getStore() as any)._rawWaypoints = rawWps;
      updateStore({
        missionTotalWps: rawWps.length,
        missionWaypointLabels: labels,
      });
      this.triggerEvent('close');
    },
    onCancel() {
      this.triggerEvent('close');
    },
  },
})
