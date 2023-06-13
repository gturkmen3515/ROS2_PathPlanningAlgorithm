cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(astar_sub src/astar_subscriber.cpp)
ament_target_dependencies(astar_sub rclcpp std_msgs geometry_msgs)

add_executable(astar_sub2 src/astar_subscriber2.cpp)
ament_target_dependencies(astar_sub2 rclcpp std_msgs geometry_msgs)

add_executable(astar_sub4 src/astar_subscriber4.cpp)
ament_target_dependencies(astar_sub4 rclcpp std_msgs geometry_msgs)

add_executable(astar_sub5 src/astar_subscriber5.cpp)
ament_target_dependencies(astar_sub5 rclcpp std_msgs geometry_msgs)

 install(TARGETS
  astar_sub
 astar_sub2
 astar_sub4
  astar_sub5
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(astar_sub3 ${DEPENDENCIES})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
