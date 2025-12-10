cmake_minimum_required(VERSION 3.8)
project(inspire_hand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH /serial)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(serial REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(Poor_node src/Poor.cpp)
ament_target_dependencies(Poor_node rclcpp service_interfaces)

add_executable(Hand_control_node src/Hand_control.cpp)
ament_target_dependencies(Hand_control_node rclcpp service_interfaces serial)

install(TARGETS
  Poor_node
  Hand_control_node
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
