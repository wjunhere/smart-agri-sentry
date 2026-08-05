#include <gtest/gtest.h>
#include "obstacle_processor.h"
#include "ldlidar_datatype.h"
#include "ros2_api.h"
#include "sentry_interfaces/msg/obstacle_info.hpp"
#include <cmath>
#include <limits>

using namespace ldlidar;
using sentry_interfaces::msg::ObstacleInfo;

TEST(ObstacleProcessorTest, EmptyInput) {
  Points2D points;
  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f);
  EXPECT_TRUE(std::isnan(info.front_min_distance));
  EXPECT_TRUE(std::isnan(info.front_avg_distance));
  EXPECT_FALSE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 0);
  EXPECT_FLOAT_EQ(info.danger_threshold, 0.5f);
}

TEST(ObstacleProcessorTest, FrontObstacleDetected) {
  Points2D points;
  points.emplace_back(10.0f, 300, 100, 0);
  points.emplace_back(350.0f, 400, 100, 0);
  points.emplace_back(180.0f, 1000, 100, 0);

  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f);

  EXPECT_FLOAT_EQ(info.front_min_distance, 0.3f);
  EXPECT_NEAR(info.front_avg_distance, 0.35f, 0.01f);
  EXPECT_TRUE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 2);
}

TEST(ObstacleProcessorTest, FrontClear) {
  Points2D points;
  points.emplace_back(10.0f, 800, 100, 0);
  points.emplace_back(350.0f, 1000, 100, 0);

  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f);

  EXPECT_FLOAT_EQ(info.front_min_distance, 0.8f);
  EXPECT_FALSE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 2);
}

TEST(ObstacleProcessorTest, OutOfSectorIgnored) {
  Points2D points;
  points.emplace_back(180.0f, 100, 100, 0);
  points.emplace_back(90.0f, 100, 100, 0);

  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f);

  EXPECT_TRUE(std::isnan(info.front_min_distance));
  EXPECT_FALSE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 0);
}

TEST(ObstacleProcessorTest, VehicleFrontUsesCorrectedLaserAngle) {
  Points2D points;
  points.emplace_back(270.0f, 400, 100, 0);
  points.emplace_back(0.0f, 200, 100, 0);

  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f, 90.0f, true);

  EXPECT_FLOAT_EQ(info.front_min_distance, 0.4f);
  EXPECT_TRUE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 1);
}

TEST(ObstacleProcessorTest, NearRangeSelfReflectionIsIgnored) {
  Points2D points;
  // 27 mm is below the configured 0.30 m lidar minimum range.
  points.emplace_back(270.0f, 27, 100, 0);

  ObstacleInfo info = ObstacleProcessor::process(
      points, 90.0f, 0.5f, 90.0f, true);

  EXPECT_TRUE(std::isnan(info.front_min_distance));
  EXPECT_FALSE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 0);
}
TEST(ObstacleProcessorTest, SideWallAtFortyFiveDegreesOutsideBodyCorridorIsIgnored) {
  Points2D points;
  // laser_scan_dir=true reverses 225 deg to 135 deg. With a vehicle-front
  // angle of 90 deg this is a 45 deg side-wall return, not a path obstacle.
  points.emplace_back(225.0f, 500, 100, 0);

  ObstacleInfo info = ObstacleProcessor::process(
      points, 90.0f, 0.5f, 90.0f, true);

  EXPECT_TRUE(std::isnan(info.front_min_distance));
  EXPECT_FALSE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 0);
}
TEST(LaserScanAngleTest, ReversesRawClockwiseAnglesWhenConfigured) {
  EXPECT_FLOAT_EQ(NormalizeScanAngleDegrees(0.0f, true), 0.0f);
  EXPECT_FLOAT_EQ(NormalizeScanAngleDegrees(90.0f, true), 270.0f);
  EXPECT_FLOAT_EQ(NormalizeScanAngleDegrees(180.0f, true), 180.0f);
  EXPECT_FLOAT_EQ(NormalizeScanAngleDegrees(270.0f, true), 90.0f);
}

TEST(LaserScanAngleTest, KeepsAnglesWhenReverseDirectionDisabled) {
  EXPECT_FLOAT_EQ(NormalizeScanAngleDegrees(90.0f, false), 90.0f);
  EXPECT_FLOAT_EQ(NormalizeScanAngleDegrees(270.0f, false), 270.0f);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
