cmake_minimum_required(VERSION 3.8)
project(lidar_tilt_safe_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(more_interfaces REQUIRED)

add_executable(speed_input_publisher src/speed_input_publisher.cpp)
ament_target_dependencies(speed_input_publisher
  rclcpp
  std_msgs
)

add_executable(angle_range_input_publisher src/angle_range_input_publisher.cpp)
ament_target_dependencies(angle_range_input_publisher
  rclcpp
  more_interfaces
)

add_executable(lidar_tilt_safe_drive src/lidar_tilt_safe_drive.cpp)
ament_target_dependencies(lidar_tilt_safe_drive
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  more_interfaces
)

install(TARGETS
  speed_input_publisher
  angle_range_input_publisher
  lidar_tilt_safe_drive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
