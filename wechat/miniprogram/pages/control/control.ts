import { getStore, updateStore, onStoreChange } from '../../services/store';
import { apiSetMode, apiControl, apiStop, apiSetCropType } from '../../services/api';

Component({
  data: {
    mode: 'AUTO',
    linear: 0,
    angular: 0,
    cropType: 'tomato',
    missionState: 'IDLE',
    missionProgress: 0,
    missionPlantsDetected: 0,
    missionCurrentWpIdx: 0,
    missionTotalWps: 0,
    missionWaypointLabels: [] as string[],
    showWaypointEditor: false,
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
    },
    detached() {
      if (this._unsub) this._unsub();
    },
  },
  methods: {
    sync(s: any) {
      this.setData({
        mode: s.mode,
        linear: s.linear,
        angular: s.angular,
        cropType: s.cropType,
        missionState: s.missionState,
        missionProgress: s.missionProgress,
        missionPlantsDetected: s.missionPlantsDetected,
        missionCurrentWpIdx: s.missionCurrentWpIdx,
        missionTotalWps: s.missionTotalWps,
        missionWaypointLabels: s.missionWaypointLabels,
      });
    },

    _linear: 0,
    _angular: 0,

    onBtnUp()    { this._linear += 0.05; this.sendCmd(); },
    onBtnDown()  { this._linear -= 0.05; this.sendCmd(); },
    onBtnLeft()  { this._angular += 0.05; this.sendCmd(); },
    onBtnRight() { this._angular -= 0.05; this.sendCmd(); },
    onBtnStop()  { this._linear = 0; this._angular = 0; apiStop(); },

    sendCmd() {
      apiControl(this._linear, this._angular);
      updateStore({ linear: this._linear, angular: this._angular });
    },

    onToggleMode() {
      const newAuto = this.data.mode !== 'AUTO';
      apiSetMode(newAuto);
    },

    onSelectCrop(e: any) {
      const crop = e.currentTarget.dataset.crop;
      apiSetCropType(crop);
      updateStore({ cropType: crop });
    },

    onOpenWaypoints() {
      this.setData({ showWaypointEditor: true });
    },
    onCloseWaypoints() {
      this.setData({ showWaypointEditor: false });
    },
  },
})
