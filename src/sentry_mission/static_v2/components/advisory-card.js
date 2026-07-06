const AdvisoryCard = {
  template: `
  <div class="card">
    <h3>
      农艺建议
      <span class="count-badge" v-if="store.advisoryText">1</span>
    </h3>
    <div v-if="store.advisoryText">
      <p class="advisory-text">{{ store.advisoryText }}</p>
      <div class="stat" v-if="store.advisoryUrgency">
        建议 {{ store.advisoryUrgency }}h 内执行
      </div>
      <div class="stat" v-if="store.advisoryFungicide" style="color:var(--blue)">
        {{ store.advisoryFungicide }}
      </div>
    </div>
    <div v-else class="muted">等待建议...</div>
  </div>`
};
