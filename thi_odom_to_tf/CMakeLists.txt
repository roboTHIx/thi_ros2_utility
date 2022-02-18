cmake_minimum_required(VERSION 3.8)
project(thi_odom_to_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -std=c++17)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies ament_cmake
                 rclcpp
                 std_msgs
                 std_srvs
                 sensor_msgs
                 nav_msgs
                 tf2
                 tf2_ros)

add_executable(thi_odom_to_tf_node src/main.cpp)
ament_target_dependencies(thi_odom_to_tf_node ${dependencies})


install(TARGETS thi_odom_to_tf_node
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY images
#   DESTINATION share/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})


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
