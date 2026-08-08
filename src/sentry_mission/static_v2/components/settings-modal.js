// settings-modal.js — runtime-tunable node parameters panel
const SettingsModal = {
  props: { visible: Boolean },
  emits: ['close'],
  template: `
  <div class="modal-overlay" v-if="visible" @click.self="$emit('close')">
    <div class="modal settings-modal">
      <h2>参数设置</h2>
      <div class="modal-body">
        <div class="setting-row">
          <div class="setting-label">
            <span class="setting-name">低光增强</span>
            <span class="setting-desc">暗光环境下提亮相机画面</span>
          </div>
          <button class="setting-toggle" :class="{ on: store.settings.low_light_enhancement }"
                  :disabled="store.settingsBusy"
                  @click="toggleLowLight">
            {{ store.settings.low_light_enhancement ? '开' : '关' }}
          </button>
        </div>
        <div class="setting-row">
          <div class="setting-label">
            <span class="setting-name">检测置信度阈值</span>
            <span class="setting-desc">越高越少误检，越低越灵敏（当前 {{ confText }}）</span>
          </div>
          <input type="range" min="0.2" max="0.8" step="0.05"
                 :value="store.settings.detection_confidence"
                 :disabled="store.settingsBusy"
                 @change="onConfChange" class="setting-slider" />
        </div>
        <div class="setting-row">
          <div class="setting-label">
            <span class="setting-name">舵机初始朝向</span>
            <span class="setting-desc">巡航开始/结束时摄像头朝向</span>
          </div>
          <div class="setting-pills">
            <button class="pill" :class="{ active: store.settings.servo_start_side === 'left' }"
                    :disabled="store.settingsBusy"
                    @click="setSide('left')">朝左</button>
            <button class="pill" :class="{ active: store.settings.servo_start_side === 'right' }"
                    :disabled="store.settingsBusy"
                    @click="setSide('right')">朝右</button>
          </div>
        </div>
        <div class="settings-msg" v-if="store.settingsMsg">{{ store.settingsMsg }}</div>
        <div class="settings-hint">参数立即生效，栈重启后恢复默认值</div>
      </div>
    </div>
  </div>`,
  computed: {
    confText() {
      const v = this.store.settings.detection_confidence;
      return v == null ? '--' : Number(v).toFixed(2);
    }
  },
  watch: {
    visible(v) {
      if (v) fetchSettings().catch(err => console.error(err));
    }
  },
  methods: {
    toggleLowLight() {
      const next = !this.store.settings.low_light_enhancement;
      updateSetting('low_light_enhancement', next)
        .catch(err => console.error(err));
    },
    onConfChange(e) {
      updateSetting('detection_confidence', Number(e.target.value))
        .catch(err => console.error(err));
    },
    setSide(side) {
      updateSetting('servo_start_side', side)
        .catch(err => console.error(err));
    }
  }
};
