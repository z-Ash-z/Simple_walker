cmake_minimum_required(VERSION 3.8)
project(simple_walker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include/simple_walker)

#-------- Turtlebot walker : start -------
add_executable(walker app/driver.cpp src/walker.cpp)
ament_target_dependencies(walker rclcpp geometry_msgs tf2 tf2_ros sensor_msgs)

install(TARGETS walker DESTINATION lib/${PROJECT_NAME})
#-------- Turtlebot walker : end ---------

# For launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
