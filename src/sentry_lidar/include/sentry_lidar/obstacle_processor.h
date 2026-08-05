#ifndef SENTRY_LIDAR_OBSTACLE_PROCESSOR_H_
#define SENTRY_LIDAR_OBSTACLE_PROCESSOR_H_

#include "ldlidar_datatype.h"
#include "sentry_interfaces/msg/obstacle_info.hpp"

namespace ldlidar {

class ObstacleProcessor {
 public:
  static sentry_interfaces::msg::ObstacleInfo process(
    const Points2D& points,
    float front_sector_half_angle_deg,
    float danger_threshold_m,
    float front_sector_center_angle_deg = 0.0f,
    bool reverse_direction = false,
    float front_corridor_half_width_m = 0.24f,
    float min_obstacle_range_m = 0.30f);
};

}  // namespace ldlidar

#endif  // SENTRY_LIDAR_OBSTACLE_PROCESSOR_H_
