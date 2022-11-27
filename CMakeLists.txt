cmake_minimum_required(VERSION 3.8)
project(can_plugins2)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_plugins2 REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


include_directories(include ${Boost_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frame.msg"
  DEPENDENCIES std_msgs
)

# add_executable(slcan_bridge src/slcan_bridge.cpp)
# ament_target_dependencies(
#   slcan_bridge rclcpp can_plugins2 diagnostic_updater diagnostic_msgs)



add_library(slcan_bridge SHARED
  src/slcan_bridge.cpp)

ament_target_dependencies(slcan_bridge
"rclcpp"
"rclcpp_components"
"std_msgs")

rclcpp_components_register_nodes(slcan_bridge "slcan_bridge::SlcanBridge")


install(TARGETS
  slcan_bridge
  DESTINATION lib/${PROJECT_NAME})


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