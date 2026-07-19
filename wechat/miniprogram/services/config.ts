// services/config.ts
// Car connection config — single source of truth, user-overridable in settings.

const DEFAULT_CAR_IP = '10.101.47.106';
const API_PORT = 8765;
const STORAGE_KEY = 'car_ip';

export function getCarIp(): string {
  return wx.getStorageSync(STORAGE_KEY) || DEFAULT_CAR_IP;
}

export function setCarIp(ip: string): void {
  wx.setStorageSync(STORAGE_KEY, ip.trim());
}

export function getBaseUrl(): string {
  return `http://${getCarIp()}:${API_PORT}`;
}

export function getWsUrl(): string {
  return `ws://${getCarIp()}:${API_PORT}/ws`;
}
