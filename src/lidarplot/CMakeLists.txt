cmake_minimum_required(VERSION 3.16)
project(lidarplot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Build
add_executable(lidarplot src/lidarplot.cpp)
ament_target_dependencies(lidarplot rclcpp sensor_msgs OpenCV)

install(TARGETS
  lidarplot
  DESTINATION lib/${PROJECT_NAME})

ament_package()
