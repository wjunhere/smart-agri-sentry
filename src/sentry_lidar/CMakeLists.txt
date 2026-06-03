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
