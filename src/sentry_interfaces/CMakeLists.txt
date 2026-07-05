cmake_minimum_required(VERSION 3.8)
project(sentry_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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
  "msg/ChassisConfig.msg"
  "msg/ObstacleInfo.msg"
  "srv/PipelineTrigger.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
