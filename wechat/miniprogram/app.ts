// app.ts
import { wsConnect } from './services/ws';
import { updateStore } from './services/store';
import { getCarIp } from './services/config';

App<IAppOption>({
  globalData: {},
  onLaunch() {
    updateStore({ carIp: getCarIp() });
    wsConnect();
  },
})
