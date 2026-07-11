// utils/format.ts
// Unit formatting helpers

export function formatTemp(val: number | null): string {
  if (val == null) return '--';
  return val.toFixed(1) + '°C';
}

export function formatHumidity(val: number | null): string {
  if (val == null) return '--';
  return val.toFixed(1) + '%';
}

export function formatCO2(val: number | null): string {
  if (val == null) return '--';
  return Math.round(val).toString() + ' ppm';
}

export function formatNPK(val: number | null): string {
  if (val == null) return '--';
  return val.toFixed(1);
}

export function formatPercent(val: number | null): string {
  if (val == null) return '--';
  return (val * 100).toFixed(1) + '%';
}

export function formatSpeed(val: number): string {
  return val.toFixed(2) + ' m/s';
}

export function formatVoltage(val: number | null): string {
  if (val == null) return '--V';
  return val.toFixed(1) + 'V';
}
