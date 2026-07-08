const EnvDataBar = {
  template: `
  <div class="env-bar">
    <div class="env-bar-inner">
      <div class="env-grid">
        <div class="env-item" :class="{ alert: store.envAirTemp !== null && store.envAirTemp > 35 }">
          <span class="label">TEMP</span>
          <span class="value">{{ store.envAirTemp !== null ? store.envAirTemp.toFixed(1) : '--' }}<span class="unit">°C</span></span>
          <div class="sparkline"><canvas></canvas></div>
        </div>
        <div class="env-item">
          <span class="label">HUMI</span>
          <span class="value">{{ store.envAirHumidity !== null ? store.envAirHumidity.toFixed(1) : '--' }}<span class="unit">%</span></span>
          <div class="sparkline"><canvas></canvas></div>
        </div>
        <div class="env-item">
          <span class="label">CO2</span>
          <span class="value">{{ store.envCO2 !== null ? store.envCO2.toFixed(0) : '--' }}<span class="unit">ppm</span></span>
          <div class="sparkline"><canvas></canvas></div>
        </div>
        <div class="env-item">
          <span class="label">SOIL-T</span>
          <span class="value">{{ store.envSoilTemp !== null ? store.envSoilTemp.toFixed(1) : '--' }}<span class="unit">°C</span></span>
          <div class="sparkline"><canvas></canvas></div>
        </div>
        <div class="env-item">
          <span class="label">SOIL-M</span>
          <span class="value">{{ store.envSoilHumidity !== null ? store.envSoilHumidity.toFixed(1) : '--' }}<span class="unit">%</span></span>
          <div class="sparkline"><canvas></canvas></div>
        </div>
        <div class="env-item">
          <span class="label">LEAF</span>
          <span class="value">{{ store.envLeafWetness !== null ? store.envLeafWetness.toFixed(1) : '--' }}<span class="unit">%</span></span>
          <div class="sparkline"><canvas></canvas></div>
        </div>
        <div class="env-item" v-if="store.envSoilN !== null">
          <span class="label">N</span>
          <span class="value">{{ store.envSoilN.toFixed(0) }}<span class="unit">mg/kg</span></span>
        </div>
        <div class="env-item" v-if="store.envSoilP !== null">
          <span class="label">P</span>
          <span class="value">{{ store.envSoilP.toFixed(0) }}<span class="unit">mg/kg</span></span>
        </div>
        <div class="env-item" v-if="store.envSoilK !== null">
          <span class="label">K</span>
          <span class="value">{{ store.envSoilK.toFixed(0) }}<span class="unit">mg/kg</span></span>
        </div>
        <div class="env-item" v-if="store.envSoilPH !== null">
          <span class="label">PH</span>
          <span class="value">{{ store.envSoilPH.toFixed(1) }}<span class="unit"></span></span>
        </div>
      </div>
      <div class="diag-toggle-section">
        <span class="diag-toggle-label">病害模拟</span>
        <div class="diag-toggle-pills">
          <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'real' }"
                  @click="store.mockDiagnosisMode = 'real'">真实数据</button>
          <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'healthy' }"
                  @click="store.mockDiagnosisMode = 'healthy'">Healthy</button>
          <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'early_blight' }"
                  @click="store.mockDiagnosisMode = 'early_blight'">Early Blight</button>
        </div>
      </div>
    </div>
  </div>`
};
