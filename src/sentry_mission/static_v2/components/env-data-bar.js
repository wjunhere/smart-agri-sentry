const EnvDataBar = {
  template: `
  <div class="env-bar">
    <h3>固定环境节点</h3>
    <div class="env-grid">
      <div class="env-item">
        <span class="label">气温</span>
        <span class="value">{{ store.envAirTemp !== null ? store.envAirTemp.toFixed(1) + '°C' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">湿度</span>
        <span class="value">{{ store.envAirHumidity !== null ? store.envAirHumidity.toFixed(1) + '%' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">CO₂</span>
        <span class="value">{{ store.envCO2 !== null ? store.envCO2.toFixed(0) + 'ppm' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">土壤温度</span>
        <span class="value">{{ store.envSoilTemp !== null ? store.envSoilTemp.toFixed(1) + '°C' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">土壤湿度</span>
        <span class="value">{{ store.envSoilHumidity !== null ? store.envSoilHumidity.toFixed(1) + '%' : '--' }}</span>
      </div>
      <div class="env-item">
        <span class="label">叶面湿度</span>
        <span class="value">{{ store.envLeafWetness !== null ? store.envLeafWetness.toFixed(1) + '%' : '--' }}</span>
      </div>
    </div>
  </div>`
};
