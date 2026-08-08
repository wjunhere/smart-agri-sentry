const AdvisoryCard = {
  template: `
  <div class="card">
    <h3>
      农艺建议
      <span class="action-badge" :class="prioClass" v-if="actionCn">{{ actionCn }}</span>
    </h3>
    <div v-if="store.advisoryText">
      <p class="advisory-text">{{ store.advisoryText }}</p>
      <div class="stat" v-if="store.advisoryPriority">
        紧急度 <span :style="{color: prioColor}">{{ store.advisoryPriority }}</span>
      </div>
      <ol class="adv-steps" v-if="store.advisorySteps && store.advisorySteps.length">
        <li v-for="(s, i) in store.advisorySteps" :key="i">{{ s }}</li>
      </ol>
    </div>
    <div v-else class="muted">等待建议...</div>
  </div>`,
  computed: {
    actionCn() { return ACTION_CN[this.store.advisoryActionType] || ''; },
    prioClass() {
      return { CRITICAL: 'prio-critical', HIGH: 'prio-high' }[this.store.advisoryPriority] || 'prio-normal';
    },
    prioColor() {
      return { CRITICAL: 'var(--red)', HIGH: 'var(--amber)', MEDIUM: 'var(--blue)' }[this.store.advisoryPriority] || 'var(--green)';
    },
  },
};
