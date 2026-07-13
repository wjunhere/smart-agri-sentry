#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "ros2_api.h"
#include "ldlidar_driver.h"
#include "obstacle_processor.h"
#include "sentry_interfaces/msg/obstacle_info.hpp"

void ToLaserscanMessagePublish(
    ldlidar::Points2D& src,
    double lidar_spin_freq,
    LaserScanSetting& setting,
    rclcpp::Node::SharedPtr& node,
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sentry_lidar");

  std::string product_name;
  std::string topic_name = "scan";
  std::string port_name;
  int serial_port_baudrate;
  bool enable_serial;
  ldlidar::LDType type_name;
  LaserScanSetting setting;
  setting.frame_id = "laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;
  setting.min_range = 0.3;
  setting.max_range = 20.0;
  setting.measure_point_freq = 4500;

  float front_sector_half_angle = 30.0f;
  float danger_threshold = 0.5f;
  bool enable_filter = false;

  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<bool>("enable_serial_or_network_communication", enable_serial);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);
  node->declare_parameter<double>("min_range", setting.min_range);
  node->declare_parameter<double>("max_range", setting.max_range);
  node->declare_parameter<int>("measure_point_freq", setting.measure_point_freq);
  node->declare_parameter<float>("front_sector_half_angle", front_sector_half_angle);
  node->declare_parameter<float>("danger_threshold", danger_threshold);
  node->declare_parameter<bool>("enable_filter", enable_filter);

  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("enable_serial_or_network_communication", enable_serial);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);
  node->get_parameter("min_range", setting.min_range);
  node->get_parameter("max_range", setting.max_range);
  node->get_parameter("measure_point_freq", setting.measure_point_freq);
  node->get_parameter("front_sector_half_angle", front_sector_half_angle);
  node->get_parameter("danger_threshold", danger_threshold);
  node->get_parameter("enable_filter", enable_filter);

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06;
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else if (product_name == "LDLiDAR_STL06P") {
    type_name = ldlidar::LDType::STL_06P;
  } else if (product_name == "LDLiDAR_STL27L") {
    type_name = ldlidar::LDType::STL_27L;
  } else if (product_name == "LDLiDAR_STL26") {
    type_name = ldlidar::LDType::STL_26;
  } else if (product_name == "LDLiDAR_STL06N") {
    type_name = ldlidar::LDType::STL_06N;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error, input <product_name> is illegal.");
    return EXIT_FAILURE;
  }

  ldlidar::LDLidarDriver* lidar = new ldlidar::LDLidarDriver();
  lidar->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));
  lidar->EnableFilterAlgorithnmProcess(enable_filter);

  if (enable_serial) {
    if (!lidar->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
      RCLCPP_ERROR(node->get_logger(), "lidar start failed");
      delete lidar;
      return EXIT_FAILURE;
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Network mode not supported yet");
    delete lidar;
    return EXIT_FAILURE;
  }

  if (!lidar->WaitLidarCommConnect(500)) {
    RCLCPP_ERROR(node->get_logger(), "lidar communication abnormal");
    delete lidar;
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "sentry_lidar started, product=%s, port=%s, baud=%d",
              product_name.c_str(), port_name.c_str(), serial_port_baudrate);

  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  auto obstacle_pub = node->create_publisher<sentry_interfaces::msg::ObstacleInfo>(
    "/lidar/obstacle_info", 10);

  rclcpp::WallRate r(10);
  ldlidar::Points2D laser_scan_points;
  double lidar_spin_freq;
  bool is_get = false;

  while (rclcpp::ok()) {
    switch (lidar->GetLaserScanData(laser_scan_points, 1000)) {
      case ldlidar::LidarStatus::NORMAL:
        if (!is_get) {
          is_get = true;
          RCLCPP_INFO(node->get_logger(), "publishing lidar data");
        }
        lidar->GetLidarSpinFreq(lidar_spin_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_spin_freq, setting, node, scan_pub);

        {
          auto obstacle_info = ldlidar::ObstacleProcessor::process(
            laser_scan_points, front_sector_half_angle, danger_threshold);
          obstacle_info.header.stamp = node->now();
          obstacle_info.header.frame_id = setting.frame_id;
          obstacle_pub->publish(obstacle_info);
        }
        break;

      case ldlidar::LidarStatus::ERROR:
        RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                              "lidar driver error");
        break;

      case ldlidar::LidarStatus::DATA_TIME_OUT:
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                             "lidar data timeout");
        break;

      case ldlidar::LidarStatus::DATA_WAIT:
      default:
        break;
    }
    r.sleep();
  }

  lidar->Stop();
  delete lidar;
  rclcpp::shutdown();
  return 0;
}

void ToLaserscanMessagePublish(
    ldlidar::Points2D& src,
    double lidar_spin_freq,
    LaserScanSetting& setting,
    rclcpp::Node::SharedPtr& node,
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  float angle_min = 0;
  float angle_max = 2 * M_PI;
  float range_min = 0.02f;
  float range_max = 30.0f;
  int beam_size = static_cast<int>(src.size());
  if (beam_size <= 1) return;

  float angle_increment = (angle_max - angle_min) / static_cast<float>(beam_size - 1);
  rclcpp::Time start_scan_time = node->now();

  sensor_msgs::msg::LaserScan output;
  output.header.stamp = start_scan_time;
  output.header.frame_id = setting.frame_id;
  output.angle_min = angle_min;
  output.angle_max = angle_max;
  output.range_min = range_min;
  output.range_max = range_max;
  output.angle_increment = angle_increment;
  output.time_increment = 0.0f;
  output.scan_time = 0.1f;
  output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
  output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

  for (auto& point : src) {
    float range = point.distance / 1000.0f;
    float intensity = point.intensity;
    float dir_angle = point.angle;

    if (point.distance == 0 && point.intensity == 0) {
      range = std::numeric_limits<float>::quiet_NaN();
      intensity = std::numeric_limits<float>::quiet_NaN();
    }

    if (setting.enable_angle_crop_func) {
      if (dir_angle >= setting.angle_crop_min && dir_angle <= setting.angle_crop_max) {
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }
    }

    if (range >= setting.max_range || range <= setting.min_range) {
      range = std::numeric_limits<float>::quiet_NaN();
      intensity = std::numeric_limits<float>::quiet_NaN();
    }

    float angle = ANGLE_TO_RADIAN(dir_angle);
    int index = static_cast<int>(std::floor((angle - angle_min) / angle_increment));
    if (index < 0) index = 0;
    if (index >= beam_size) index = beam_size - 1;

    if (std::isnan(output.ranges[index]) || range < output.ranges[index]) {
      output.ranges[index] = range;
    }
    output.intensities[index] = intensity;
  }

  lidarpub->publish(output);
}

uint64_t GetSystemTimeStamp(void) {
  auto now = std::chrono::system_clock::now();
  auto ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  return static_cast<uint64_t>(ns.time_since_epoch().count());
}
