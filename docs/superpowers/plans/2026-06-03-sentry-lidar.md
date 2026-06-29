# sentry_lidar 雷达节点实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `src/` 中新建 `sentry_lidar` C++ ROS2 包，迁移 LDLiDAR 驱动，新增前方扇区预处理，发布 `/scan` 和 `/lidar/obstacle_info`，集成到 bringup，适配 STL19P + CP2102。

**Architecture:** 独立 C++ 包 + 迁移 example 驱动核心 + 新增 obstacle_processor 模块 + 双话题输出。节点内部先由串口线程接收原始字节，经 lipkg 协议解析为 Points2D，再经 obstacle_processor 计算前方扇区信息，最后分别封装为 LaserScan 和 ObstacleInfo 发布。

**Tech Stack:** ROS2 Humble (C++14), ament_cmake, gtest, sensor_msgs, std_msgs, tf2_ros

---

## 文件结构

| 文件 | 操作 | 说明 |
|------|------|------|
| `src/sentry_interfaces/msg/ObstacleInfo.msg` | 创建 | 自定义障碍物简讯消息 |
| `src/sentry_interfaces/CMakeLists.txt` | 修改 | 加入 ObstacleInfo.msg 到 rosidl_generate_interfaces |
| `src/sentry_lidar/CMakeLists.txt` | 创建 | 包构建配置，含 gtest 测试目标 |
| `src/sentry_lidar/package.xml` | 创建 | 包元数据 |
| `src/sentry_lidar/config/stl19p.yaml` | 创建 | STL19P 参数配置文件 |
| `src/sentry_lidar/launch/stl19p.launch.py` | 创建 | 独立启动雷达 |
| `src/sentry_lidar/launch/viewer_stl19p.launch.py` | 创建 | 带 rviz2 调试启动 |
| `src/sentry_lidar/udev/99-cp2102-lidar.rules` | 创建 | CP2102 udev 规则 |
| `src/sentry_lidar/include/sentry_lidar/*.h` | 创建/迁移 | 头文件（5 个迁移 + 1 个新增） |
| `src/sentry_lidar/src/*.cpp` | 创建/迁移 | 源文件（5 个迁移 + 2 个新增） |
| `src/sentry_lidar/test/test_obstacle_processor.cpp` | 创建 | gtest 单元测试 |
| `src/sentry_bringup/launch/sentry_v2.launch.py` | 修改 | 集成雷达启动 |

---

## 任务依赖图

```
Task 1 (消息定义)
    |
    v
Task 2 (包骨架) --
    |             |
    v             v
Task 3 (迁移驱动核心) --
    |                   |
    v                   v
Task 4 (obstacle_processor TDD)
    |
    v
Task 5 (重构 main.cpp)
    |
    v
Task 6 (launch + config)
    |
    v
Task 7 (bringup 集成)  Task 8 (udev 规则)
    |                        |
    +-----------+------------+
                v
          Task 9 (编译验证)
```

---

### Task 1: 定义 ObstacleInfo.msg 消息接口

**依赖:** 无  
**Files:**
- 创建: `src/sentry_interfaces/msg/ObstacleInfo.msg`
- 修改: `src/sentry_interfaces/CMakeLists.txt`

- [ ] **Step 1: 创建消息定义文件**

创建 `src/sentry_interfaces/msg/ObstacleInfo.msg`：

```yaml
std_msgs/Header header
float32 front_min_distance
float32 front_avg_distance
bool obstacle_detected
float32 danger_threshold
int32 front_point_count
```

- [ ] **Step 2: 修改 CMakeLists.txt 注册消息**

在 `src/sentry_interfaces/CMakeLists.txt` 的 `rosidl_generate_interfaces` 调用中加入 `"msg/ObstacleInfo.msg"`：

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Diagnosis.msg"
  "msg/PlantDetection.msg"
  "msg/Environment.msg"
  "msg/SoilNutrition.msg"
  "msg/FusionResult.msg"
  "msg/ForecastAlert.msg"
  "msg/AdvisoryAction.msg"
  "msg/MissionStatus.msg"
  "msg/ChassisStatus.msg"
  "msg/ServoCmd.msg"
  "msg/ObstacleInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)
```

- [ ] **Step 3: 编译 sentry_interfaces 并验证消息**

```bash
colcon build --packages-select sentry_interfaces
source install/setup.bash
ros2 interface show sentry_interfaces/msg/ObstacleInfo
```

**Expected:** 正确显示 ObstacleInfo 的 5 个字段。

- [ ] **Step 4: Commit**

```bash
git add src/sentry_interfaces/msg/ObstacleInfo.msg src/sentry_interfaces/CMakeLists.txt
git commit -m "feat(interfaces): add ObstacleInfo.msg for lidar obstacle reporting"
```

---

### Task 2: 创建 sentry_lidar 包骨架

**依赖:** Task 1  
**Files:**
- 创建: `src/sentry_lidar/CMakeLists.txt`
- 创建: `src/sentry_lidar/package.xml`
- 创建: `src/sentry_lidar/config/stl19p.yaml`

- [ ] **Step 1: 创建目录结构**

```bash
mkdir -p src/sentry_lidar/{include/sentry_lidar,src,launch,config,udev,test}
```

- [ ] **Step 2: 写 CMakeLists.txt**

创建 `src/sentry_lidar/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(sentry_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sentry_interfaces REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/sentry_lidar
)

set(LDLIDAR_SRCS
  src/ldlidar_driver.cpp
  src/lipkg.cpp
  src/serial_interface_linux.cpp
  src/tofbf.cpp
  src/log_module.cpp
  src/obstacle_processor.cpp
  src/main.cpp
)

add_executable(${PROJECT_NAME} ${LDLIDAR_SRCS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs std_msgs tf2_ros sentry_interfaces
)
target_link_libraries(${PROJECT_NAME} pthread)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_obstacle_processor test/test_obstacle_processor.cpp src/obstacle_processor.cpp)
  target_include_directories(test_obstacle_processor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/sentry_lidar)
  ament_target_dependencies(test_obstacle_processor rclcpp sentry_interfaces)
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: 写 package.xml**

创建 `src/sentry_lidar/package.xml`：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sentry_lidar</name>
  <version>0.2.0</version>
  <description>STL19P LiDAR driver node for Smart Agri Sentry</description>
  <maintainer email="team@example.com">team</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>sentry_interfaces</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: 写参数配置文件**

创建 `src/sentry_lidar/config/stl19p.yaml`：

```yaml
sentry_lidar:
  ros__parameters:
    product_name: "LDLiDAR_LD19"
    port_name: "/dev/wheeltec_lidar"
    port_baudrate: 230400
    frame_id: "laser"
    laser_scan_dir: true
    enable_angle_crop_func: false
    angle_crop_min: 135.0
    angle_crop_max: 225.0
    min_range: 0.3
    max_range: 20.0
    front_sector_half_angle: 30.0
    danger_threshold: 0.5
    enable_filter: false
```

- [ ] **Step 5: Commit**

```bash
git add src/sentry_lidar/CMakeLists.txt src/sentry_lidar/package.xml src/sentry_lidar/config/stl19p.yaml
git commit -m "chore(sentry_lidar): create package skeleton, cmake, package.xml and config"
```

---

### Task 3: 迁移 LDLiDAR 驱动核心代码

**依赖:** Task 2  
**Files:**
- 创建: `src/sentry_lidar/include/sentry_lidar/{ros2_api.h,ldlidar_driver.h,ldlidar_datatype.h,lipkg.h,serial_interface_linux.h,tofbf.h,log_module.h}`
- 创建: `src/sentry_lidar/src/{ldlidar_driver.cpp,lipkg.cpp,serial_interface_linux.cpp,tofbf.cpp,log_module.cpp}`

- [ ] **Step 1: 复制头文件**

从 `example/lidar/ldlidar_ros2/ldlidar_driver/include/` 下的各子目录复制头文件到 `src/sentry_lidar/include/sentry_lidar/`：

```bash
cp example/lidar/ldlidar_ros2/ldlidar/include/ros2_api.h src/sentry_lidar/include/sentry_lidar/
cp example/lidar/ldlidar_ros2/ldlidar_driver/include/core/ldlidar_driver.h src/sentry_lidar/include/sentry_lidar/
cp example/lidar/ldlidar_ros2/ldlidar_driver/include/core/ldlidar_datatype.h src/sentry_lidar/include/sentry_lidar/
cp example/lidar/ldlidar_ros2/ldlidar_driver/include/dataprocess/lipkg.h src/sentry_lidar/include/sentry_lidar/
cp example/lidar/ldlidar_ros2/ldlidar_driver/include/serialcom/serial_interface_linux.h src/sentry_lidar/include/sentry_lidar/
cp example/lidar/ldlidar_ros2/ldlidar_driver/include/filter/tofbf.h src/sentry_lidar/include/sentry_lidar/
cp example/lidar/ldlidar_ros2/ldlidar_driver/include/logger/log_module.h src/sentry_lidar/include/sentry_lidar/
```

- [ ] **Step 2: 复制源文件**

```bash
cp example/lidar/ldlidar_ros2/ldlidar_driver/src/core/ldlidar_driver.cpp src/sentry_lidar/src/
cp example/lidar/ldlidar_ros2/ldlidar_driver/src/dataprocess/lipkg.cpp src/sentry_lidar/src/
cp example/lidar/ldlidar_ros2/ldlidar_driver/src/serialcom/serial_interface_linux.cpp src/sentry_lidar/src/
cp example/lidar/ldlidar_ros2/ldlidar_driver/src/filter/tofbf.cpp src/sentry_lidar/src/
cp example/lidar/ldlidar_ros2/ldlidar_driver/src/logger/log_module.cpp src/sentry_lidar/src/
```

- [ ] **Step 3: 调整头文件 include 路径**

将所有迁移过来的 `.cpp` 文件中的 `#include` 从原始子目录路径改为扁平路径。例如：

在 `ldlidar_driver.cpp` 中：
```cpp
// 修改前
#include "ldlidar_driver.h"
// 已经是扁平路径，无需修改
```

在 `lipkg.cpp` 中：
```cpp
// 修改前
#include "lipkg.h"
// 已经是扁平路径，无需修改
```

**注意:** 原始代码中头文件 include 已经是裸文件名（如 `#include "lipkg.h"`），而 `CMakeLists.txt` 中已通过 `include_directories` 将 `include/sentry_lidar` 加入搜索路径，因此**无需修改 include 语句**。

- [ ] **Step 4: 编译验证（预期失败，缺 main.cpp 和 obstacle_processor）**

```bash
colcon build --packages-select sentry_lidar 2>&1 | tail -20
```

**Expected:** 链接错误，缺少 `main.cpp` 和 `obstacle_processor.cpp` 中的符号。这是正常的。

- [ ] **Step 5: Commit**

```bash
git add src/sentry_lidar/include/ src/sentry_lidar/src/ldlidar_driver.cpp src/sentry_lidar/src/lipkg.cpp src/sentry_lidar/src/serial_interface_linux.cpp src/sentry_lidar/src/tofbf.cpp src/sentry_lidar/src/log_module.cpp
git commit -m "feat(sentry_lidar): migrate LDLiDAR driver core from example"
```

---

### Task 4: obstacle_processor TDD 实现

**依赖:** Task 3  
**Files:**
- 创建: `src/sentry_lidar/include/sentry_lidar/obstacle_processor.h`
- 创建: `src/sentry_lidar/src/obstacle_processor.cpp`
- 创建: `src/sentry_lidar/test/test_obstacle_processor.cpp`

- [ ] **Step 1: 写 gtest 测试（先写测试）**

创建 `src/sentry_lidar/test/test_obstacle_processor.cpp`：

```cpp
#include <gtest/gtest.h>
#include "obstacle_processor.h"
#include "ldlidar_datatype.h"
#include "sentry_interfaces/msg/obstacle_info.hpp"
#include <cmath>

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
  // 前方 10deg, 距离 0.3m (< danger_threshold 0.5)
  points.emplace_back(10.0f, 300, 100, 0);
  // 前方 350deg, 距离 0.4m
  points.emplace_back(350.0f, 400, 100, 0);
  // 后方 180deg, 距离 1.0m (不应被计入)
  points.emplace_back(180.0f, 1000, 100, 0);

  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f);

  EXPECT_FLOAT_EQ(info.front_min_distance, 0.3f);
  EXPECT_NEAR(info.front_avg_distance, 0.35f, 0.01f);
  EXPECT_TRUE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 2);
}

TEST(ObstacleProcessorTest, FrontClear) {
  Points2D points;
  // 前方所有点都大于 threshold
  points.emplace_back(10.0f, 800, 100, 0);   // 0.8m
  points.emplace_back(350.0f, 1000, 100, 0); // 1.0m

  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f);

  EXPECT_FLOAT_EQ(info.front_min_distance, 0.8f);
  EXPECT_FALSE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 2);
}

TEST(ObstacleProcessorTest, OutOfSectorIgnored) {
  Points2D points;
  // 后方近距离，不应被计入前方扇区
  points.emplace_back(180.0f, 100, 100, 0);  // 0.1m
  points.emplace_back(90.0f, 100, 100, 0);   // 0.1m

  ObstacleInfo info = ObstacleProcessor::process(points, 30.0f, 0.5f);

  EXPECT_TRUE(std::isnan(info.front_min_distance));
  EXPECT_FALSE(info.obstacle_detected);
  EXPECT_EQ(info.front_point_count, 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 2: 运行测试（预期失败）**

```bash
colcon test --packages-select sentry_lidar --ctest-args "-R test_obstacle_processor" --event-handlers console_direct+
```

**Expected:** 编译失败，`obstacle_processor.h` 不存在。

- [ ] **Step 3: 写 obstacle_processor.h**

创建 `src/sentry_lidar/include/sentry_lidar/obstacle_processor.h`：

```cpp
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
    float danger_threshold_m);
};

}  // namespace ldlidar

#endif  // SENTRY_LIDAR_OBSTACLE_PROCESSOR_H_
```

- [ ] **Step 4: 写 obstacle_processor.cpp**

创建 `src/sentry_lidar/src/obstacle_processor.cpp`：

```cpp
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
    // 前方扇区: [360 - half, 360] U [0, half]
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
```

- [ ] **Step 5: 运行测试（预期通过）**

```bash
colcon test --packages-select sentry_lidar --ctest-args "-R test_obstacle_processor" --event-handlers console_direct+
```

**Expected:** 4 个测试全部 PASS。

- [ ] **Step 6: Commit**

```bash
git add src/sentry_lidar/include/sentry_lidar/obstacle_processor.h src/sentry_lidar/src/obstacle_processor.cpp src/sentry_lidar/test/test_obstacle_processor.cpp
git commit -m "feat(sentry_lidar): add obstacle_processor with TDD (gtest)"
```

---

### Task 5: 重构 main.cpp

**依赖:** Task 4  
**Files:**
- 创建: `src/sentry_lidar/src/main.cpp`
- 创建: `src/sentry_lidar/include/sentry_lidar/ros2_api.h`

- [ ] **Step 1: 写 ros2_api.h**

创建 `src/sentry_lidar/include/sentry_lidar/ros2_api.h`（从 example 迁移的辅助头）：

```cpp
#ifndef ROS2_API_H_
#define ROS2_API_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

struct LaserScanSetting {
  std::string frame_id;
  bool laser_scan_dir;
  bool enable_angle_crop_func;
  double angle_crop_min;
  double angle_crop_max;
  double min_range;
  double max_range;
  int measure_point_freq;
};

#endif  // ROS2_API_H_
```

- [ ] **Step 2: 写 main.cpp**

创建 `src/sentry_lidar/src/main.cpp`：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

  // --- Parameters ---
  std::string product_name;
  std::string topic_name;
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

  // --- Product type mapping ---
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

  // --- TF broadcaster ---
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = node->now();
  tf_stamped.header.frame_id = "base_link";
  tf_stamped.child_frame_id = setting.frame_id;
  tf_stamped.transform.translation.x = 0.0;
  tf_stamped.transform.translation.y = 0.0;
  tf_stamped.transform.translation.z = 0.18;
  tf_stamped.transform.rotation.x = 0.0;
  tf_stamped.transform.rotation.y = 0.0;
  tf_stamped.transform.rotation.z = 0.0;
  tf_stamped.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(tf_stamped);

  // --- Lidar driver init ---
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

  // --- Publishers ---
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
        RCLCPP_ERROR(node->get_logger(), "lidar driver error");
        break;

      case ldlidar::LidarStatus::DATA_TIME_OUT:
        RCLCPP_ERROR(node->get_logger(), "lidar data timeout");
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
```

- [ ] **Step 3: 编译验证**

```bash
colcon build --packages-select sentry_lidar
```

**Expected:** 编译成功，无错误。

- [ ] **Step 4: Commit**

```bash
git add src/sentry_lidar/src/main.cpp src/sentry_lidar/include/sentry_lidar/ros2_api.h
git commit -m "feat(sentry_lidar): restructure main.cpp with dual-publish (/scan + /lidar/obstacle_info) and TF"
```

---

### Task 6: 创建 Launch 文件

**依赖:** Task 5  
**Files:**
- 创建: `src/sentry_lidar/launch/stl19p.launch.py`
- 创建: `src/sentry_lidar/launch/viewer_stl19p.launch.py`

- [ ] **Step 1: 写 stl19p.launch.py**

创建 `src/sentry_lidar/launch/stl19p.launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_lidar')
    config_path = os.path.join(pkg_share, 'config', 'stl19p.yaml')

    return LaunchDescription([
        Node(
            package='sentry_lidar',
            executable='sentry_lidar',
            name='sentry_lidar',
            output='screen',
            parameters=[config_path],
        ),
    ])
```

- [ ] **Step 2: 写 viewer_stl19p.launch.py**

创建 `src/sentry_lidar/launch/viewer_stl19p.launch.py`：

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_lidar')
    stl19p_launch = os.path.join(pkg_share, 'launch', 'stl19p.launch.py')
    rviz_config = os.path.join(pkg_share, '..', '..', '..', '..', 'example', 'lidar',
                               'ldlidar_ros2', 'ldlidar', 'rviz2', 'ldlidar.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stl19p_launch)
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen',
        ),
    ])
```

- [ ] **Step 3: 编译并验证 launch 文件语法**

```bash
colcon build --packages-select sentry_lidar
source install/setup.bash
ros2 launch --show-args src/sentry_lidar/launch/stl19p.launch.py
```

**Expected:** 显示参数列表，无语法错误。

- [ ] **Step 4: Commit**

```bash
git add src/sentry_lidar/launch/
git commit -m "feat(sentry_lidar): add stl19p launch files (standalone + viewer)"
```

---

### Task 7: 集成到 sentry_bringup

**依赖:** Task 6  
**Files:**
- 修改: `src/sentry_bringup/launch/sentry_v2.launch.py`

- [ ] **Step 1: 修改 sentry_v2.launch.py 集成雷达**

在 `src/sentry_bringup/launch/sentry_v2.launch.py` 的 `generate_launch_description()` 返回的列表中，在 Fusion node 之前加入雷达启动：

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# ... 已有 import ...

def generate_launch_description():
    pkg_dir = get_package_share_directory('sentry_bringup')
    config_dir = os.path.join(pkg_dir, '..', '..', '..', 'config')
    if not os.path.exists(config_dir):
        config_dir = os.path.join(os.getcwd(), 'config')

    crop_profiles_path = os.path.join(config_dir, 'crop_profiles.yaml')
    mission_params_path = os.path.join(config_dir, 'mission_params.yaml')

    # 雷达 launch 路径
    lidar_launch_path = os.path.join(
        get_package_share_directory('sentry_lidar'), 'launch', 'stl19p.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('crop_type', default_value='tomato'),
        DeclareLaunchArgument('use_sim_plant', default_value='false'),

        # Vision nodes ...

        # Sensor bridge ...

        # LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),

        # Fusion node ...
        # Mission control ...
    ])
```

**注意:** 实际修改时只需在现有列表中插入 `IncludeLaunchDescription` 项，保留其他节点不动。

- [ ] **Step 2: 验证 bringup launch 语法**

```bash
colcon build --packages-select sentry_bringup
source install/setup.bash
ros2 launch --show-args src/sentry_bringup/launch/sentry_v2.launch.py
```

**Expected:** 显示全部参数，无报错。

- [ ] **Step 3: Commit**

```bash
git add src/sentry_bringup/launch/sentry_v2.launch.py
git commit -m "feat(bringup): integrate sentry_lidar into sentry_v2.launch.py"
```

---

### Task 8: 添加 CP2102 udev 规则

**依赖:** 无（可与 Task 7 并行）  
**Files:**
- 创建: `src/sentry_lidar/udev/99-cp2102-lidar.rules`

- [ ] **Step 1: 写 udev 规则文件**

创建 `src/sentry_lidar/udev/99-cp2102-lidar.rules`：

```bash
# CP2102 USB-to-UART bridge for STL19P LiDAR
# Creates /dev/wheeltec_lidar symlink
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"
```

- [ ] **Step 2: Commit**

```bash
git add src/sentry_lidar/udev/99-cp2102-lidar.rules
git commit -m "feat(sentry_lidar): add CP2102 udev rule for STL19P"
```

---

### Task 9: 完整编译与集成验证

**依赖:** Task 7, Task 8  
**Files:** 无新增

- [ ] **Step 1: 完整编译 workspace**

```bash
colcon build --packages-select sentry_interfaces sentry_lidar sentry_bringup
```

**Expected:** 全部编译通过，无警告/错误。

- [ ] **Step 2: 验证消息接口**

```bash
source install/setup.bash
ros2 interface show sentry_interfaces/msg/ObstacleInfo
```

**Expected:** 显示 5 个字段。

- [ ] **Step 3: 验证节点可发现**

```bash
source install/setup.bash
ros2 pkg executables sentry_lidar
```

**Expected:** 输出 `sentry_lidar sentry_lidar`。

- [ ] **Step 4: 运行单元测试**

```bash
colcon test --packages-select sentry_lidar --ctest-args "-R test_obstacle_processor" --event-handlers console_direct+
```

**Expected:** 4 个测试全部 PASS。

- [ ] **Step 5: 更新根目录 PLAN.md**

在 `PLAN.md` 中追加 sentry_lidar 相关完成状态：

```markdown
### sentry_lidar 雷达节点
- [x] ObstacleInfo.msg 消息定义
- [x] sentry_lidar 包骨架
- [x] LDLiDAR 驱动核心迁移
- [x] obstacle_processor TDD 实现
- [x] main.cpp 重构（/scan + /lidar/obstacle_info）
- [x] launch 文件（stl19p + viewer）
- [x] bringup 集成
- [x] CP2102 udev 规则
```

- [ ] **Step 6: Commit PLAN.md**

```bash
git add PLAN.md
git commit -m "docs: update PLAN.md with sentry_lidar completion status"
```

---

## 自检清单

| 检查项 | 状态 |
|--------|------|
| Spec 覆盖: ObstacleInfo.msg | Task 1 |
| Spec 覆盖: sentry_lidar 包 | Task 2-6 |
| Spec 覆盖: 驱动迁移 | Task 3 |
| Spec 覆盖: 双话题输出 | Task 5 |
| Spec 覆盖: 前方扇区预处理 | Task 4 |
| Spec 覆盖: bringup 集成 | Task 7 |
| Spec 覆盖: CP2102 适配 | Task 8 |
| 无占位符 (TBD/TODO) | 已检查，无 |
| 类型一致性 | ObstacleInfo 字段在各任务中一致 |
| 文件路径具体 | 全部使用绝对或相对精确路径 |

---

## 执行选项

**Plan complete and saved to `docs/superpowers/plans/2026-06-03-sentry-lidar.md`. Two execution options:**

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
