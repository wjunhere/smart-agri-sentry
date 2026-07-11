// utils/ros-parser.ts
// Parse backend JSON into typed structures

export function parseSensorData(data: any) {
  return data;
}

export function parseWeatherData(data: any) {
  if (!data || !data.days) return data;
  return data;
}
