cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

# Generate code for your .srv and declare its message deps
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  DEPENDENCIES sensor_msgs         # <-- REQUIRED because you use LaserScan
)

# Make the generated interfaces available to dependents
ament_export_dependencies(rosidl_default_runtime)

# Your node
add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

# Ensure the node links the generated typesupport when it uses the service types
# 1) Ask rosidl for the correct typesupport target name for this package/back-end
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 2) Link your node against that target (and thus depend on it)
target_link_libraries(patrol_node ${cpp_typesupport_target})

install(TARGETS
   patrol_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the RViz config directory
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
