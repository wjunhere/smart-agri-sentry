import { getStore, onStoreChange } from '../../services/store';
import { apiGetForecast } from '../../services/api';

const DISEASE_NAMES: Record<string, string[]> = {
  tomato: ['早疫病', '晚疫病', '白粉病', '灰霉病', '叶霉病', '斑枯病', '健康'],
  wheat: ['赤霉病', '锈病', '白粉病', '纹枯病', '健康'],
  strawberry: ['白粉病', '灰霉病', '炭疽病', '红中柱根腐病', '叶斑病', '蛇眼病', '枯萎病', '健康'],
};

Component({
  data: {
    cropType: 'tomato',
    disease: '--',
    confidence: '--',
    probs: [] as {name: string, pct: string, color: string, width: string}[],
    advisoryText: '',
    advisoryPriority: '',
    advisorySteps: [] as string[],
    forecastActive: false,
    forecastDescription: '',
    forecastHoursAhead: 0,
  },
  lifetimes: {
    attached() {
      const s = getStore();
      this.sync(s);
      this._unsub = onStoreChange((s) => this.sync(s));
      this._pollTimer = setInterval(() => this.fetchForecast(), 30000);
      this.fetchForecast();
    },
    detached() {
      if (this._unsub) this._unsub();
      if (this._pollTimer) clearInterval(this._pollTimer as number);
    },
  },
  methods: {
    sync(s: any) {
      const names = DISEASE_NAMES[s.diagnosisCropType || s.cropType] || DISEASE_NAMES.tomato;
      const probs = (s.diagnosisProbabilities || []).map((p: number, i: number) => {
        const pct = (p * 100).toFixed(1);
        return {
          name: names[i] || `类别${i}`,
          pct,
          width: pct,
          color: i === 0 ? '#F59E0B' : i === 1 ? '#38BDF8' : i === 2 ? '#10B981' : '#64748B',
        };
      });
      this.setData({
        cropType: s.diagnosisCropType || s.cropType,
        disease: s.diagnosisDisease || '--',
        confidence: s.diagnosisConfidence ? (s.diagnosisConfidence * 100).toFixed(1) : '--',
        probs,
        advisoryText: s.advisoryText,
        advisoryPriority: s.advisoryPriority,
        advisorySteps: s.advisorySteps,
        forecastActive: s.forecastActive,
        forecastDescription: s.forecastDescription,
      });
    },

    async fetchForecast() {
      try {
        await apiGetForecast();
      } catch (_) {}
    },
  },
})
