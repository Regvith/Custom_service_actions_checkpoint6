cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

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
rosidl_generate_interfaces(${PROJECT_NAME} "srv/GetDirection.srv"   DEPENDENCIES sensor_msgs)


add_executable(patrol_exe src/patrol.cpp)
ament_target_dependencies(patrol_exe rclcpp geometry_msgs nav_msgs sensor_msgs)
add_executable(direction_server_exe src/direction_service.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(direction_server_exe ${cpp_typesupport_target})

ament_target_dependencies(direction_server_exe rclcpp geometry_msgs nav_msgs sensor_msgs )
add_executable(test_service_exe src/test_service.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(test_service_exe ${cpp_typesupport_target})

ament_target_dependencies(test_service_exe rclcpp geometry_msgs nav_msgs sensor_msgs )
add_executable(patrol_service_exe src/patrol_with_service.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(patrol_service_exe ${cpp_typesupport_target})

ament_target_dependencies(patrol_service_exe rclcpp geometry_msgs nav_msgs sensor_msgs )
install(TARGETS patrol_exe patrol_service_exe direction_server_exe test_service_exe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()
