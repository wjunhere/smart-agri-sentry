const AdvisoryCard = {
  template: `
  <div class="card">
    <h3>农艺建议</h3>
    <div v-if="store.advisoryText">
      <p class="advisory-text">{{ store.advisoryText }}</p>
      <div class="stat" v-if="store.advisoryUrgency">
        建议 {{ store.advisoryUrgency }} 小时内执行
      </div>
      <div class="stat" v-if="store.advisoryFungicide">
        推荐药剂: {{ store.advisoryFungicide }}
      </div>
    </div>
    <div v-else class="muted">等待建议...</div>
  </div>`
};
