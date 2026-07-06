const AlertDetailModal = {
  template: `
  <div class="modal-overlay" v-if="store.selectedAlert" @click.self="store.selectedAlert = null">
    <div class="modal">
      <h2>预警详情 — {{ formatTime(store.selectedAlert.time) }}</h2>
      <div class="snapshot-row">
        <div class="snapshot-img">
          <h4>现场快照</h4>
          <img v-if="store.selectedAlert.snapshot?.frame"
               :src="store.selectedAlert.snapshot.frame" alt="现场快照">
          <div v-else class="muted">无图像快照</div>
        </div>
        <div class="snapshot-env">
          <h4>环境快照</h4>
          <div class="stat">气温: {{ store.selectedAlert.snapshot?.envAirTemp?.toFixed(1) || '--' }}°C</div>
          <div class="stat">湿度: {{ store.selectedAlert.snapshot?.envAirHumidity?.toFixed(1) || '--' }}%RH</div>
          <div class="stat">叶面湿润: {{ store.selectedAlert.snapshot?.envLeafWetness?.toFixed(1) || store.selectedAlert.lwd_hours?.toFixed(1) || '--' }}h</div>
          <div class="stat">土壤温度: {{ store.selectedAlert.snapshot?.envSoilTemp?.toFixed(1) || '--' }}°C</div>
          <div class="stat">土壤湿度: {{ store.selectedAlert.snapshot?.envSoilHumidity?.toFixed(1) || '--' }}%</div>
        </div>
      </div>
      <div class="card">
        <h4>农艺建议</h4>
        <p>{{ store.selectedAlert.snapshot?.advisoryText || store.advisoryText || '暂无建议' }}</p>
        <div v-if="store.selectedAlert.snapshot?.advisoryFungicide || store.advisoryFungicide">
          药剂: {{ store.selectedAlert.snapshot?.advisoryFungicide || store.advisoryFungicide }}
        </div>
      </div>
      <div class="evidence" v-if="store.selectedAlert.evidence_chain?.length">
        <h4>决策依据</h4>
        <ul>
          <li v-for="(e, i) in store.selectedAlert.evidence_chain" :key="i">{{ e }}</li>
        </ul>
      </div>
      <div class="modal-stats">
        <span>风险值: {{ (store.selectedAlert.risk_score * 100).toFixed(0) }}%</span>
        <span>| 置信度: {{ (store.selectedAlert.confidence * 100).toFixed(0) }}%</span>
        <span>| 模式: {{ store.selectedAlert.mode }}</span>
      </div>
      <div class="actions">
        <button class="btn btn-pause" @click="store.selectedAlert = null">关闭</button>
      </div>
    </div>
  </div>`,
  methods: {
    formatTime(iso) {
      return new Date(iso).toLocaleString('zh-CN');
    }
  }
};
