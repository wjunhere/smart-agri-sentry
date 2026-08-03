#include "obstacle_processor.h"
#include <cmath>
#include <limits>

namespace ldlidar {
namespace {

float NormalizeAngleDegrees(float angle_deg) {
  angle_deg = std::fmod(angle_deg, 360.0f);
  if (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

float AngularDistanceDegrees(float a_deg, float b_deg) {
  float diff = std::fabs(NormalizeAngleDegrees(a_deg) - NormalizeAngleDegrees(b_deg));
  return diff > 180.0f ? 360.0f - diff : diff;
}

}  // namespace

sentry_interfaces::msg::ObstacleInfo ObstacleProcessor::process(
    const Points2D& points,
    float front_sector_half_angle_deg,
    float danger_threshold_m,
    float front_sector_center_angle_deg,
    bool reverse_direction,
    float front_corridor_half_width_m,
    float min_obstacle_range_m) {

  sentry_interfaces::msg::ObstacleInfo info;
  info.danger_threshold = danger_threshold_m;

  if (points.empty()) {
    info.front_min_distance = std::numeric_limits<float>::quiet_NaN();
    info.front_avg_distance = std::numeric_limits<float>::quiet_NaN();
    info.obstacle_detected = false;
    info.front_point_count = 0;
    return info;
  }

  float min_dist_mm = std::numeric_limits<float>::max();
  double sum_dist_mm = 0.0;
  int count = 0;

  for (const auto& p : points) {
    float scan_angle = reverse_direction ? (360.0f - p.angle) : p.angle;
    bool in_front_sector =
        AngularDistanceDegrees(scan_angle, front_sector_center_angle_deg) <=
        front_sector_half_angle_deg;

    const float distance_m = p.distance / 1000.0f;
    const float relative_angle_rad =
        (scan_angle - front_sector_center_angle_deg) * static_cast<float>(M_PI / 180.0);
    const float forward_m = distance_m * std::cos(relative_angle_rad);
    const float lateral_m = std::fabs(distance_m * std::sin(relative_angle_rad));
    const bool in_front_corridor =
        forward_m > 0.001f && lateral_m <= front_corridor_half_width_m;

    if (in_front_sector && in_front_corridor &&
        distance_m >= min_obstacle_range_m) {
      if (p.distance < min_dist_mm) {
        min_dist_mm = p.distance;
      }
      sum_dist_mm += p.distance;
      ++count;
    }
  }

  if (count == 0) {
    info.front_min_distance = std::numeric_limits<float>::quiet_NaN();
    info.front_avg_distance = std::numeric_limits<float>::quiet_NaN();
    info.obstacle_detected = false;
  } else {
    float min_dist_m = min_dist_mm / 1000.0f;
    float avg_dist_m = static_cast<float>(sum_dist_mm / count) / 1000.0f;
    info.front_min_distance = min_dist_m;
    info.front_avg_distance = avg_dist_m;
    info.obstacle_detected = (min_dist_m <= danger_threshold_m);
  }

  info.front_point_count = count;
  return info;
}

}  // namespace ldlidar
