// settings-modal.js — runtime-tunable node parameters and cruise-history reader
const SettingsModal = {
  props: { visible: Boolean },
  emits: ['close'],
  template: `
  <div class="modal-overlay" v-if="visible" @click.self="$emit('close')">
    <div class="modal settings-modal">
      <h2>参数设置</h2>
      <div class="modal-body">
        <div class="setting-row"><div class="setting-label"><span class="setting-name">低光增强</span><span class="setting-desc">暗光环境下提亮相机画面</span></div><button class="setting-toggle" :class="{ on: store.settings.low_light_enhancement }" :disabled="store.settingsBusy" @click="toggleLowLight">{{ store.settings.low_light_enhancement ? '开' : '关' }}</button></div>
        <div class="setting-row"><div class="setting-label"><span class="setting-name">检测置信度阈值</span><span class="setting-desc">越高越少误检，越低越灵敏（当前 {{ confText }}）</span></div><input type="range" min="0.2" max="0.8" step="0.05" :value="store.settings.detection_confidence" :disabled="store.settingsBusy" @change="onConfChange" class="setting-slider" /></div>
        <div class="setting-row"><div class="setting-label"><span class="setting-name">舵机初始朝向</span><span class="setting-desc">巡航开始/结束时摄像头朝向</span></div><div class="setting-pills"><button class="pill" :class="{ active: store.settings.servo_start_side === 'left' }" :disabled="store.settingsBusy" @click="setSide('left')">朝左</button><button class="pill" :class="{ active: store.settings.servo_start_side === 'right' }" :disabled="store.settingsBusy" @click="setSide('right')">朝右</button></div></div>
        <div class="setting-row"><div class="setting-label"><span class="setting-name">停车舵机偏移角</span><span class="setting-desc">检测到植株停车后，摄像头向车头方向偏转角度（当前 {{ offsetText }}°）</span></div><input type="range" min="0" max="45" step="1" :value="store.settings.plant_stop_offset" :disabled="store.settingsBusy" @change="onOffsetChange" class="setting-slider" /></div>
        <div class="setting-row"><div class="setting-label"><span class="setting-name">模拟田间数据（MOCK）</span><span class="setting-desc">仅用于演示，不影响真实传感器数据</span></div><button class="setting-toggle" :class="{ on: store.mockFieldOn }" @click="store.toggleMockField()">{{ store.mockFieldOn ? '开' : '关' }}</button></div>

        <section class="history-settings">
          <div class="history-title"><span class="setting-name">巡航历史</span><span class="setting-desc">板端保留 90 天、最多 30 批；每批最多 10 张检测截图</span></div>
          <div class="history-grid">
            <label>批次数<select v-model.number="store.historyFilters.limit"><option :value="1">1</option><option :value="5">5</option><option :value="10">10</option><option :value="20">20</option><option :value="30">30</option></select></label>
            <label>日期范围<select v-model="store.historyFilters.datePreset"><option value="all">全部（90天内）</option><option value="1d">今天</option><option value="7d">近 7 天</option><option value="30d">近 30 天</option></select></label>
            <label>开始日期<input type="date" v-model="store.historyFilters.startDate" @change="store.historyFilters.datePreset=''" /></label>
            <label>结束日期<input type="date" v-model="store.historyFilters.endDate" @change="store.historyFilters.datePreset=''" /></label>
            <label>作物<select v-model="store.historyFilters.cropType"><option value="">全部</option><option value="tomato">番茄</option><option value="wheat">小麦</option><option value="strawberry">草莓</option></select></label>
            <label>病害（英文类别）<input v-model.trim="store.historyFilters.disease" placeholder="例如 late_blight" /></label>
          </div>
          <div class="history-actions"><button class="btn btn-primary" :disabled="store.historyBusy" @click="store.loadHistory()">{{ store.historyBusy ? '读取中…' : '读取并回放' }}</button><button class="btn" :disabled="store.historyBusy" @click="store.clearHistoryView()">仅清空网页显示</button><button class="btn history-danger" :disabled="store.historyBusy" @click="deleteBoardHistory">删除板端归档</button></div>
          <div class="settings-msg" v-if="store.historyMsg">{{ store.historyMsg }}</div>
          <div class="history-batches" v-if="store.historyBatches.length">
            <article class="history-batch" v-for="batch in store.historyBatches" :key="batch.id">
              <div class="history-batch-head"><strong>{{ formatTime(batch.started_at) }}</strong><span>{{ batch.crop_type || '未标注作物' }} · {{ batch.records.length }} 次检测 · {{ batch.end_reason }}</span></div>
              <div class="history-thumbs" v-if="batch.records.some(r => r.snapshot_url)">
                <a v-for="record in batch.records.filter(r => r.snapshot_url)" :key="record.seq" :href="imageUrl(record.snapshot_url)" target="_blank" rel="noopener"><img :src="imageUrl(record.snapshot_url)" :alt="'检测截图 ' + (record.seq + 1)" /></a>
              </div>
            </article>
          </div>
        </section>
        <div class="settings-msg" v-if="store.settingsMsg">{{ store.settingsMsg }}</div>
        <div class="settings-hint">运行参数立即生效；历史读取不会改变板端数据。</div>
      </div>
    </div>
  </div>`,
  computed: {
    confText() { const v = this.store.settings.detection_confidence; return v == null ? '--' : Number(v).toFixed(2); },
    offsetText() { const v = this.store.settings.plant_stop_offset; return v == null ? '--' : Number(v).toFixed(0); }
  },
  watch: { visible(v) { if (v) fetchSettings().then(() => store.syncMockFromSettings && store.syncMockFromSettings()).catch(err => console.error(err)); } },
  methods: {
    toggleLowLight() { updateSetting('low_light_enhancement', !this.store.settings.low_light_enhancement).catch(err => console.error(err)); },
    onConfChange(e) { updateSetting('detection_confidence', Number(e.target.value)).catch(err => console.error(err)); },
    setSide(side) { updateSetting('servo_start_side', side).catch(err => console.error(err)); },
    onOffsetChange(e) { updateSetting('plant_stop_offset', Number(e.target.value)).catch(err => console.error(err)); },
    formatTime(ts) { return new Date(ts * 1000).toLocaleString('zh-CN', { hour12: false }); },
    imageUrl(path) { return apiFullUrl(path); },
    deleteBoardHistory() {
      if (!window.confirm('删除板端中符合当前筛选条件的巡航历史？此操作会删除对应截图。')) return;
      if (!window.confirm('请再次确认：这会永久删除板端归档，确定继续吗？')) return;
      store.deleteBoardHistory();
    }
  }
};