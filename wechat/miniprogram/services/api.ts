// services/api.ts
// HTTP request wrapper for low-frequency data + control commands

const BASE_URL = 'http://10.101.47.106:8765';

async function request<T>(method: 'GET' | 'POST', path: string, body?: any): Promise<T> {
  return new Promise((resolve, reject) => {
    wx.request({
      url: BASE_URL + path,
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
  return BASE_URL + '/api/camera';
}

export function getCameraSnapshotUrl(): string {
  return BASE_URL + '/api/camera/snapshot';
}

export function apiLLMAnalyze() {
  return request<any>('POST', '/api/llm/analyze');
}
