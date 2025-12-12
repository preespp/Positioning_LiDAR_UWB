cmake_minimum_required(VERSION 3.8)
project(uwb_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelCommand.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(central src/central.cpp)
add_executable(uwb src/uwb.cpp)
add_executable(command_wheel src/command_wheel.cpp)
add_executable(wheel src/wheel.cpp)

ament_target_dependencies(central rclcpp tf2 tf2_ros sensor_msgs geometry_msgs)
ament_target_dependencies(uwb rclcpp geometry_msgs)
ament_target_dependencies(command_wheel rclcpp std_msgs)
ament_target_dependencies(wheel rclcpp std_msgs)

# include generated headers only for your package
foreach(target central uwb command_wheel wheel)
  add_dependencies(${target} ${PROJECT_NAME}__rosidl_generator_cpp)
  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
  target_link_libraries(${target} ${cpp_typesupport_target})
endforeach()

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

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  central
  uwb
  command_wheel
  wheel
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    launch/lidar_node_launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

