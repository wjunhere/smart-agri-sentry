const MessageCenter = {
  template: `
  <div class="modal-overlay" v-if="visible" @click.self="close">
    <div class="modal" style="max-width: 640px;">
      <h2>巡航消息</h2>
      <div class="msg-actions">
        <button class="btn btn-pause" @click="store.clearMessages()"
                :disabled="store.messageBatches.length === 0">一键清理</button>
        <button class="btn btn-resume" @click="close">关闭</button>
      </div>
      <div v-if="store.messageBatches.length === 0" class="muted"
           style="text-align:center;padding:24px">
        暂无巡航检测记录
      </div>
      <div v-for="batch in store.messageBatches" :key="batch.id" class="msg-batch">
        <div class="msg-batch-header">
          {{ batch.name }} · {{ batch.records.length }} 株
        </div>
        <div v-for="rec in batch.records" :key="rec.seq" class="msg-record"
             @click="preview = rec.snapshot_url">
          <img class="msg-thumb" :src="rec.snapshot_url" loading="lazy" />
          <div class="msg-record-info">
            <div class="msg-disease">{{ rec.disease_class || '未知' }}</div>
            <div class="muted">
              检测 {{ (rec.plant_confidence * 100).toFixed(0) }}%
              <template v-if="rec.disease_confidence !== null">
                · 诊断 {{ (rec.disease_confidence * 100).toFixed(0) }}%
              </template>
            </div>
            <div class="muted">{{ store.formatMsgTime(rec.timestamp) }}</div>
          </div>
        </div>
      </div>
      <div class="modal-overlay" v-if="preview" @click.self="preview = null">
        <img class="msg-preview" :src="preview" @click="preview = null" />
      </div>
    </div>
  </div>`,
  props: { visible: Boolean },
  emits: ['close'],
  data() { return { preview: null }; },
  methods: { close() { this.preview = null; this.$emit('close'); } },
};
