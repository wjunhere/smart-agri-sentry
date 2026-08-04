// services/api.ts
// HTTP request wrapper for low-frequency data + control commands

import { getBaseUrl } from './config';

async function request<T>(method: 'GET' | 'POST', path: string, body?: any, timeoutMs = 3000): Promise<T> {
  return new Promise((resolve, reject) => {
    wx.request({
      url: getBaseUrl() + path,
      timeout: timeoutMs,
      method,
      header: { 'Content-Type': 'application/json' },
      data: body,
      success(res) {
        if (res.statusCode === 200) {
          resolve(res.data as T);
        } else {
          reject(new Error(`HTTP ${res.statusCode}: ${res.errMsg}`));
        }
      },
      fail(err) {
        reject(new Error(err.errMsg));
      },
    });
  });
}

export function apiSetMode(auto: boolean) {
  return request<{status: string; mode: string}>('POST', '/api/mode', { auto });
}

export function apiControl(linear: number, angular: number) {
  return request<{status: string}>('POST', '/api/control', { linear, angular });
}

export function apiStop() {
  return request<{status: string; mode: string}>('POST', '/api/stop');
}

export function apiSetCropType(cropType: string) {
  return request<{status: string}>('POST', '/api/crop_type', { crop_type: cropType });
}

export function apiGetStatus() {
  return request<any>('GET', '/api/status');
}

export function apiGetWeather() {
  return request<any>('GET', '/api/weather');
}

export function apiGetForecast() {
  return request<any>('GET', '/api/forecast');
}

export function getCameraUrl(): string {
  return getBaseUrl() + '/api/camera';
}

export function getCameraSnapshotUrl(): string {
  return getBaseUrl() + '/api/camera/snapshot';
}

export function apiLLMAnalyze() {
  // LLM 分析是长请求（bridge 侧最多等 65s），不能用全局 3s 超时
  return request<any>('POST', '/api/llm/analyze', undefined, 70000);
}

export function apiStackPreheat() {
  return request<{status: string; state: string}>('POST', '/stack/preheat');
}

export function apiStackStart() {
  return request<{status: string; state: string}>('POST', '/stack/start');
}

export function apiStackStop() {
  return request<{status: string; state: string}>('POST', '/stack/stop');
}

export function apiStackShutdown() {
  return request<{status: string; state: string}>('POST', '/stack/shutdown');
}

export function apiStackStatus() {
  return request<{state: string; stack_alive: boolean; last_output: string}>('GET', '/stack/status');
}
