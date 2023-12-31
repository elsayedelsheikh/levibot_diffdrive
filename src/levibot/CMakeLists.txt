cmake_minimum_required(VERSION 3.8)
project(levibot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Compile
install(
  DIRECTORY description/launch description/urdf description/rviz description/ros2_control
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY simulation/launch simulation/worlds simulation/rviz simulation/config simulation/urdf
  DESTINATION share/${PROJECT_NAME}
)


# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   # EXPORT export_levibot
# )


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
