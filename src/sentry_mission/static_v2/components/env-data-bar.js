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
                  @click="store.setMockMode('real')">真实数据</button>
          <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'healthy' }"
                  @click="store.setMockMode('healthy')">Healthy</button>
          <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'early_blight' }"
                  @click="store.setMockMode('early_blight')">Early Blight</button>
          <button class="diag-pill" :class="{ active: store.mockDiagnosisMode === 'leaf_mold' }"
                  @click="store.setMockMode('leaf_mold')">Leaf Mold</button>
        </div>
      </div>
      <div class="diag-toggle-section hidden-tool-section">
        <span class="diag-toggle-label">视觉逻辑</span>
        <div class="diag-toggle-pills">
          <button class="diag-pill vision-mode-pill"
                  :class="{ active: store.visionInferenceMode === 'triggered' }"
                  :disabled="store.visionInferenceModeBusy"
                  @click="store.toggleVisionInferenceMode()">
            {{ store.visionInferenceMode === 'triggered' ? '植株后分类' : '独立分类' }}
          </button>
        </div>
      </div>
      <div class="fixed-point-stop-section">
        <div class="fixed-point-stop-header">
          <span class="diag-toggle-label">固定点停车</span>
          <div class="fixed-point-stop-actions">
            <button class="diag-pill" @click="store.addFixedPointStop()">添加</button>
            <button class="diag-pill active" :disabled="store.fixedPointStopsBusy"
                    @click="store.saveFixedPointStops()">保存</button>
          </div>
        </div>
        <div class="fixed-point-stop-row fixed-point-stop-labels">
          <span>#</span><span>X (m)</span><span>Y (m)</span><span>半径 (m)</span><span>病害</span><span></span>
        </div>
        <div v-for="(stop, index) in store.fixedPointStops" :key="index" class="fixed-point-stop-row">
          <span class="fixed-point-index">{{ index + 1 }}</span>
          <input v-model.number="stop.x" type="number" step="0.1" aria-label="固定点 X 坐标">
          <input v-model.number="stop.y" type="number" step="0.1" aria-label="固定点 Y 坐标">
          <input v-model.number="stop.radius" type="number" min="0.01" step="0.01" aria-label="固定点触发半径">
          <select v-model="stop.disease_class" aria-label="固定点病害类别">
            <option v-for="disease in store.fixedPointDiseaseClasses" :key="disease" :value="disease">{{ disease }}</option>
          </select>
          <button class="fixed-point-delete" title="删除固定点" aria-label="删除固定点"
                  @click="store.removeFixedPointStop(index)">&times;</button>
        </div>
      </div>
    </div>
  </div>`
};
