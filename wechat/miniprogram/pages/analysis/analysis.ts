import { getStore, onStoreChange, updateStore } from '../../services/store';
import { apiGetForecast, apiGetFusionHistory, apiLLMAnalyze } from '../../services/api';

const DISEASE_NAMES: Record<string, string[]> = {
  tomato: ['早疫病', '晚疫病', '白粉病', '灰霉病', '叶霉病', '斑枯病', '健康'],
  wheat: ['赤霉病', '锈病', '白粉病', '纹枯病', '健康'],
  strawberry: ['白粉病', '灰霉病', '炭疽病', '红中柱根腐病', '叶斑病', '蛇眼病', '枯萎病', '健康'],
};

const CROP_NAMES: Record<string, string> = {
  tomato: '番茄', wheat: '小麦', strawberry: '草莓',
};

const MODE_CN: Record<string, string> = {
  VISION_DOMINANT: '视觉确诊主导',
  LATENT_SUSPICION: '潜伏疑似',
  HIGH_HUMIDITY_PATHOGEN: '高湿致病路径',
  DROUGHT_STRESS: '干旱胁迫',
  UNKNOWN_DISEASE: '未知病害',
  BALANCED: '综合平衡',
};

const ALERT_CN: Record<string, string> = {
  NORMAL: '正常', SUSPICION: '关注', WARNING: '预警', CRITICAL: '紧急',
};

const ACTION_CN: Record<string, string> = {
  SPRAY: '喷施作业', IRRIGATE: '灌溉补水', PROTECT: '防护保温',
  MONITOR: '加强监测', NONE: '暂无操作',
};

function fmtAge(ts: number): string {
  if (!ts) return '无数据';
  const diff = Date.now() - ts;
  if (diff < 60e3) return '刚刚';
  if (diff < 3600e3) return `${Math.floor(diff / 60e3)}分钟前`;
  if (diff < 86400e3) return `${Math.floor(diff / 3600e3)}小时前`;
  return `${Math.floor(diff / 86400e3)}天前`;
}

Component({
  data: {
    // ① 输入层
    cropName: '番茄',
    visionText: '--',
    visionConf: '',
    visionAge: '无数据',
    envText: '--',
    envAge: '无数据',
    envSource: '',
    weatherText: '--',
    weatherAge: '无数据',
    // ② 融合层
    hasFusion: false,
    riskPct: '--',
    alertCn: '正常',
    alertClass: 'normal',
    segVisionW: '0',
    segEnvW: '0',
    segInterW: '0',
    visionTerm: '0.00',
    envTerm: '0.00',
    interTerm: '0.00',
    modeCn: '',
    fusionConfPct: '',
    lwdText: '',
    fusionAge: '',
    // ③ 证据链
    evidence: [] as string[],
    // 诊断细节
    disease: '--',
    confidence: '--',
    probs: [] as {name: string, pct: string, color: string, width: string}[],
    // ④ 决策层
    advisoryText: '',
    advisoryPriority: '',
    advisoryAction: '',
    advisorySteps: [] as string[],
    forecastActive: false,
    forecastDescription: '',
    forecastHoursAhead: 0,
    // 风险趋势
    hasTrend: false,
    trendReady: false,
    // AI 综合分析
    llmStatus: '',
    llmSummary: '',
    llmSuggestions: [] as string[],
    llmRiskLevel: 'low',
    llmFocusAreas: [] as string[],
    llmNextCheck: '',
    llmTrigger: '',
    llmLoading: false,
  },
  lifetimes: {
    attached() {
      const self = this as any;
      const s = getStore();
      this.sync(s);
      self._unsub = onStoreChange((s) => this.sync(s));
      self._pollTimer = setInterval(() => {
        this.fetchForecast();
        this.fetchTrend();
      }, 60000);
      this.fetchForecast();
      this.fetchTrend();
    },
    detached() {
      const self = this as any;
      if (self._unsub) self._unsub();
      if (self._pollTimer) clearInterval(self._pollTimer as number);
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

      // ① 输入层摘要
      const visionText = s.diagnosisDisease
        ? `${CROP_NAMES[s.diagnosisCropType] || s.diagnosisCropType} · ${s.diagnosisDisease}`
        : '--';
      const visionConf = s.diagnosisConfidence
        ? `置信度 ${(s.diagnosisConfidence * 100).toFixed(1)}%` : '';
      const envParts: string[] = [];
      if (s.envAirHumidity != null) envParts.push(`湿度${s.envAirHumidity}%`);
      if (s.envAirTemp != null) envParts.push(`气温${s.envAirTemp}°C`);
      if (s.envSoilHumidity != null) envParts.push(`土湿${s.envSoilHumidity}%`);
      if (s.envLeafWetness != null) envParts.push(`叶湿${s.envLeafWetness}`);
      const today = (s.weatherDays || [])[0];
      const weatherText = today
        ? `${today.weather_desc || ''} ${today.temp_low}~${today.temp_high}°C · 降水${today.precipitation}mm`
        : '--';

      // ② 融合层
      const risk = s.fusionRiskScore;
      const hasFusion = risk != null;
      const riskPct = hasFusion ? (risk * 100).toFixed(0) : '--';
      const alertName = s.fusionAlertName || 'NORMAL';
      const total = (s.fusionVisionTerm + s.fusionEnvTerm + s.fusionInteractionTerm) || 1;

      this.setData({
        cropName: CROP_NAMES[s.diagnosisCropType || s.cropType] || '番茄',
        visionText,
        visionConf,
        visionAge: fmtAge(s.diagnosisTs),
        envText: envParts.length ? envParts.join(' · ') : '--',
        envAge: fmtAge(s.envTs),
        envSource: s.envDataSource || '',
        weatherText,
        weatherAge: s.weatherStale ? '数据过期' : fmtAge(s.weatherTs),

        hasFusion,
        riskPct,
        alertCn: ALERT_CN[alertName] || alertName,
        alertClass: alertName.toLowerCase(),
        segVisionW: hasFusion ? ((s.fusionVisionTerm / total) * risk * 100).toFixed(1) : '0',
        segEnvW: hasFusion ? ((s.fusionEnvTerm / total) * risk * 100).toFixed(1) : '0',
        segInterW: hasFusion ? ((s.fusionInteractionTerm / total) * risk * 100).toFixed(1) : '0',
        visionTerm: (s.fusionVisionTerm || 0).toFixed(2),
        envTerm: (s.fusionEnvTerm || 0).toFixed(2),
        interTerm: (s.fusionInteractionTerm || 0).toFixed(2),
        modeCn: MODE_CN[s.fusionMode] || s.fusionMode || '--',
        fusionConfPct: s.fusionConfidence != null
          ? `数据充分度 ${(s.fusionConfidence * 100).toFixed(0)}%` : '',
        lwdText: s.fusionLwdHours != null ? `叶面湿润 ${s.fusionLwdHours}h` : '',
        fusionAge: fmtAge(s.fusionTs),

        evidence: s.fusionEvidence || [],

        disease: s.diagnosisDisease || '--',
        confidence: s.diagnosisConfidence ? (s.diagnosisConfidence * 100).toFixed(1) : '--',
        probs,

        advisoryText: s.advisoryText,
        advisoryPriority: s.advisoryPriority,
        advisoryAction: ACTION_CN[s.advisoryActionType] || '',
        advisorySteps: s.advisorySteps,
        forecastActive: s.forecastActive,
        forecastDescription: s.forecastDescription,
        forecastHoursAhead: s.forecastHoursAhead,

        llmStatus: s.llmStatus,
        llmSummary: s.llmSummary,
        llmSuggestions: s.llmSuggestions || [],
        llmRiskLevel: s.llmRiskLevel,
        llmFocusAreas: s.llmFocusAreas || [],
        llmNextCheck: s.llmNextCheck,
        llmTrigger: s.llmTrigger,
        llmLoading: s.llmLoading,
      });
    },

    async fetchForecast() {
      try {
        const res: any = await apiGetForecast();
        const partial: any = {};
        if (res.advisory) {
          partial.advisoryText = res.advisory.description || '';
          partial.advisoryPriority = res.advisory.priority || '';
          partial.advisoryActionType = res.advisory.action_type || '';
          partial.advisorySteps = res.advisory.steps || [];
        }
        if (res.forecast) {
          partial.forecastActive = Boolean(res.forecast.active);
          partial.forecastAlertType = res.forecast.alert_type || '';
          partial.forecastDescription = res.forecast.description || '';
          partial.forecastHoursAhead = res.forecast.hours_ahead || 0;
        }
        if (res.fusion && getStore().fusionRiskScore == null) {
          // WS 未推过融合数据时才用 REST 兜底（REST 无 ts，按现在计）
          partial.fusionRiskScore = res.fusion.risk_score;
          partial.fusionAlertLevel = res.fusion.alert_level;
          partial.fusionAlertName = res.fusion.alert_name;
          partial.fusionMode = res.fusion.mode;
          partial.fusionEvidence = res.fusion.evidence_chain || [];
          partial.fusionLwdHours = res.fusion.lwd_hours;
          partial.fusionConfidence = res.fusion.confidence;
          partial.fusionVisionTerm = res.fusion.vision_term || 0;
          partial.fusionEnvTerm = res.fusion.env_term || 0;
          partial.fusionInteractionTerm = res.fusion.interaction_term || 0;
          partial.fusionTs = Date.now();
        }
        updateStore(partial);
      } catch (_) {}
    },

    async fetchTrend() {
      try {
        const res = await apiGetFusionHistory(72);
        const points = (res.points || []).filter(
          (p) => typeof p.risk === 'number');
        (this as any)._trendPoints = points;
        this.setData({ hasTrend: points.length >= 2 });
        if (points.length >= 2) this._scheduleTrendRender();
      } catch (_) {}
    },

    // 原生 canvas 定位在布局稳定后才可靠：先挂载，下一拍再绘制
    _scheduleTrendRender() {
      const self = this as any;
      if (self._trendTimer) clearTimeout(self._trendTimer as number);
      if (this.data.trendReady) {
        this._renderTrend();
        return;
      }
      self._trendTimer = setTimeout(() => {
        this.setData({ trendReady: true });
        wx.nextTick(() => this._renderTrend());
      }, 400);
    },

    // 风险趋势折线：渐变面积 + 阈值虚线 (0.35/0.6/0.8) + 末点高亮
    _renderTrend() {
      const points = (this as any)._trendPoints || [];
      if (points.length < 2) return;
      this.createSelectorQuery()
        .select('.trend-track').boundingClientRect()
        .select('#riskTrendCanvas').fields({ node: true, size: true })
        .exec((res: any[]) => {
          const rect = res[0];
          const info = res[1];
          if (!rect || !info || !info.node) return;
          const canvas = info.node;
          const ctx = canvas.getContext('2d');
          const dpr = wx.getSystemInfoSync().pixelRatio;
          const W = rect.width, H = rect.height;
          canvas.width = W * dpr;
          canvas.height = H * dpr;
          ctx.scale(dpr, dpr);

          const padL = 6, padR = 10, topPad = 8, bottomPad = 6;
          const usable = H - topPad - bottomPad;
          const n = points.length;
          const pts = points.map((p: any, i: number) => ({
            x: padL + (i / (n - 1)) * (W - padL - padR),
            y: topPad + (1 - Math.max(0, Math.min(1, p.risk))) * usable,
          }));

          // 阈值虚线
          ctx.setLineDash([4, 4]);
          ctx.lineWidth = 1;
          for (const th of [0.35, 0.6, 0.8]) {
            const y = topPad + (1 - th) * usable;
            ctx.strokeStyle = th >= 0.8
              ? 'rgba(248,113,113,0.35)' : 'rgba(148,163,184,0.25)';
            ctx.beginPath();
            ctx.moveTo(padL, y);
            ctx.lineTo(W - padR, y);
            ctx.stroke();
          }
          ctx.setLineDash([]);

          // 面积填充
          const grad = ctx.createLinearGradient(0, 0, 0, H);
          grad.addColorStop(0, 'rgba(245,158,11,0.30)');
          grad.addColorStop(1, 'rgba(245,158,11,0)');
          ctx.beginPath();
          ctx.moveTo(pts[0].x, H - bottomPad);
          ctx.lineTo(pts[0].x, pts[0].y);
          for (let i = 1; i < n; i++) ctx.lineTo(pts[i].x, pts[i].y);
          ctx.lineTo(pts[n - 1].x, H - bottomPad);
          ctx.closePath();
          ctx.fillStyle = grad;
          ctx.fill();

          // 折线
          ctx.beginPath();
          ctx.moveTo(pts[0].x, pts[0].y);
          for (let i = 1; i < n; i++) ctx.lineTo(pts[i].x, pts[i].y);
          ctx.strokeStyle = '#F59E0B';
          ctx.lineWidth = 2;
          ctx.lineJoin = 'round';
          ctx.lineCap = 'round';
          ctx.stroke();

          // 末点
          const last = pts[n - 1];
          ctx.beginPath();
          ctx.arc(last.x, last.y, 3.5, 0, Math.PI * 2);
          ctx.fillStyle = '#F59E0B';
          ctx.fill();
          ctx.beginPath();
          ctx.arc(last.x, last.y, 6, 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(245,158,11,0.4)';
          ctx.lineWidth = 1.5;
          ctx.stroke();
        });
    },

    onDeepAnalysis() {
      if (this.data.llmLoading) return;
      this.setData({ llmLoading: true });
      apiLLMAnalyze().then((res: any) => {
        this.setData({
          llmLoading: false,
          llmStatus: res.status || 'error',
          llmSummary: res.summary || '',
          llmSuggestions: res.suggestions || [],
          llmRiskLevel: res.risk_level || 'low',
          llmFocusAreas: res.focus_areas || [],
          llmNextCheck: res.next_check || '',
          llmTrigger: 'manual',
        });
      }).catch(() => {
        this.setData({ llmLoading: false, llmStatus: 'error', llmSummary: '请求失败，请重试' });
      });
    },
  },
})
