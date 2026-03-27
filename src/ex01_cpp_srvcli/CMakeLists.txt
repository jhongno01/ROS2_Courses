cmake_minimum_required(VERSION 3.8)
project(ex01_cpp_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(ex01_cpp_srvcli)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

# add_two_ints_server
add_executable(ex01_server src/add_two_ints_server.cpp)
ament_target_dependencies(ex01_server rclcpp more_interfaces)

# add_two_ints_client
add_executable(ex01_client src/add_two_ints_client.cpp)
ament_target_dependencies(ex01_client rclcpp more_interfaces)

install(TARGETS
  ex01_server
  ex01_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
