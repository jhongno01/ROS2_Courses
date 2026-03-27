cmake_minimum_required(VERSION 3.8)
project(hw01_topic_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(more_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable and ament_target_dependencies for each node (Build Options)
add_executable(hw01_pub src/hw01_pub.cpp)
ament_target_dependencies(hw01_pub rclcpp std_msgs) # Topic -> rclcpp, std_msgs

add_executable(hw01_sub_cli src/hw01_sub_cli.cpp)
ament_target_dependencies(hw01_sub_cli rclcpp std_msgs more_interfaces) # Topic -> rclcpp, std_msgs; Service Client -> rclcpp, more_interfaces

add_executable(hw01_srv src/hw01_srv.cpp)
ament_target_dependencies(hw01_srv rclcpp more_interfaces) # Service Server -> rclcpp, more_interfaces

install(TARGETS
  hw01_pub
  hw01_sub_cli
  hw01_srv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
