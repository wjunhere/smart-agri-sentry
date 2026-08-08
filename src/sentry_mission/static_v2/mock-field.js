// mock-field.js — 演示用模拟环境数据显示注入（仅环境传感器，显示层）
//
// 按"南京 8 月"昼夜气象规律生成田间节点环境读数（气温/湿度/土温/土湿/
// 叶面湿度），只影响页面显示，不进入 ROS。
//
// 注意：真正喂给融合算法的模拟历史数据在板端 fusion_node 侧
// （mock_history_hours 启动参数，回填 LWD 24h 窗口）。
// 本开关只用于无实车/无节点时的前端离线预览；视觉诊断、融合结果、
// 农艺建议、天气一律走真实数据，不在此处 mock。

(function () {
  store.mockFieldOn = false;

  const TICK_MS = 3000;

  let timer = null;
  const m = { leafWet: null, soilT: null, soilH: 58.0 };

  function rand(amp) { return (Math.random() * 2 - 1) * amp; }

  // 日变化曲线（分段）：05:00 最低 → 15:00 平滑升至最高 → 夜间线性回落
  function diurnal(hour) {
    if (hour >= 5 && hour <= 15) {
      return (1 - Math.cos(Math.PI * (hour - 5) / 10)) / 2;
    }
    const past = hour > 15 ? hour - 15 : hour + 24 - 15;
    return Math.max(0, 1 - past / 14);
  }

  // 温度包络：优先用真实天气预报的当日高/低温，兜底南京 8 月典型值
  function tempEnvelope() {
    const d = (store.weatherDays || [])[0] || {};
    const tHigh = Number.isFinite(d.temp_high) ? d.temp_high : 34.5;
    const tLow = Number.isFinite(d.temp_low) ? d.temp_low : 27.0;
    return { tLow, tHigh };
  }

  function humiAt(hour, tLow, tHigh) {
    const humiMin = Math.max(55, 96 - (tHigh - tLow) * 2.8);
    return 96 - (96 - humiMin) * diurnal(hour);
  }

  function tick() {
    const now = new Date();
    const hour = now.getHours() + now.getMinutes() / 60;
    const { tLow, tHigh } = tempEnvelope();

    if (m.leafWet === null) {
      // 按当前时刻初始化叶面湿度：夜间高湿时段有叶湿，白天已蒸发
      m.leafWet = humiAt(hour, tLow, tHigh) >= 85 ? 6.0 : 0.0;
      m.soilT = tLow + 1.5;
    }
    const airT = tLow + (tHigh - tLow) * diurnal(hour) + rand(0.3);
    const humi = Math.min(99, humiAt(hour, tLow, tHigh) + rand(1.2));

    if (humi >= 85) {
      m.leafWet = Math.min(10, m.leafWet + 0.05);
    } else if (humi < 80) {
      m.leafWet = Math.max(0, m.leafWet - 0.12);
    }
    m.soilT = m.soilT * 0.92 + airT * 0.08;
    m.soilH = Math.min(66, Math.max(54, m.soilH + rand(0.15)));

    store.envAirTemp = parseFloat(airT.toFixed(1));
    store.envAirHumidity = parseFloat(humi.toFixed(1));
    store.envSoilTemp = parseFloat(m.soilT.toFixed(1));
    store.envSoilHumidity = parseFloat(m.soilH.toFixed(1));
    store.envLeafWetness = parseFloat(m.leafWet.toFixed(1));
    store.envCO2 = Math.round(420 + rand(15));
    store.envDataSource = 'FIXED_NODE_01';
    store.envTs = Date.now();
  }

  store.toggleMockField = function () {
    store.mockFieldOn = !store.mockFieldOn;
    if (store.mockFieldOn) {
      m.leafWet = null;
      tick();
      timer = setInterval(tick, TICK_MS);
      console.log('[mock-field] 模拟环境数据（显示层）已开启');
    } else {
      if (timer) clearInterval(timer);
      timer = null;
      console.log('[mock-field] 已关闭，恢复真实数据');
    }
    // 同步到小车：前端"启动栈"时 fusion_node 回填 24h LWD 历史。
    // 离线预览时小车不可达，静默失败即可。
    if (typeof updateSetting === 'function') {
      updateSetting('mock_history_hours', store.mockFieldOn ? 24 : 0)
        .catch(() => {});
    }
  };

  // 设置面板打开时对齐两边状态（板端设置项 → 前端开关）
  store.syncMockFromSettings = function () {
    const backendOn = Number(store.settings.mock_history_hours) > 0;
    if (Boolean(backendOn) !== store.mockFieldOn) {
      store.toggleMockField();
    }
  };
})();
