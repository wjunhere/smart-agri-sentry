// app.js — Vue 3 application entry

const app = Vue.createApp({
  data() {
    return { store: window.store };
  },
});

// Global mixin injects store into every component's data scope
app.mixin({
  data() {
    return { store: window.store };
  }
});

// Register all components
app.component('TopBar', TopBar);
app.component('CameraPanel', CameraPanel);
app.component('DetectionCard', DetectionCard);
app.component('DiagnosisCard', DiagnosisCard);
app.component('DiagnosisToggle', DiagnosisToggle);
app.component('AdvisoryCard', AdvisoryCard);
app.component('ForecastPanel', ForecastPanel);
app.component('AlertDetailModal', AlertDetailModal);
app.component('EnvDataBar', EnvDataBar);
app.component('Dpad', Dpad);
app.component('CropSelector', CropSelector);
app.component('CruisePanel', CruisePanel);
app.component('WaypointEditor', WaypointEditor);
app.component('StatusBar', StatusBar);
app.component('ControlPanel', ControlPanel);

app.mount('#app');
