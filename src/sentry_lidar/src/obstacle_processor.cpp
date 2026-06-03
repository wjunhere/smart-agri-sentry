#include "obstacle_processor.h"
#include <cmath>
#include <limits>

namespace ldlidar {

sentry_interfaces::msg::ObstacleInfo ObstacleProcessor::process(
    const Points2D& points,
    float front_sector_half_angle_deg,
    float danger_threshold_m) {

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
    bool in_front_sector = false;
    if (p.angle <= front_sector_half_angle_deg ||
        p.angle >= (360.0f - front_sector_half_angle_deg)) {
      in_front_sector = true;
    }

    if (in_front_sector && p.distance > 0) {
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
