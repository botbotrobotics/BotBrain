cmake_minimum_required(VERSION 3.8)
project(bot_realsense)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# BotBrain builds with plain `colcon build`; unoptimised per-pixel loops cost several times more CPU.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realsense2 2.55 REQUIRED)  # rs2::context JSON settings ("format-conversion")
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-rtsp-server-1.0
)

add_library(bot_realsense_component SHARED
  src/realsense_node.cpp
  src/image_ops.cpp
  src/streamer.cpp
  src/depth_video.cpp
)
target_include_directories(bot_realsense_component PRIVATE include src)
ament_target_dependencies(bot_realsense_component
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(bot_realsense_component realsense2::realsense2 PkgConfig::GST)

rclcpp_components_register_node(bot_realsense_component
  PLUGIN "bot_realsense::RealsenseNode"
  EXECUTABLE bot_realsense_node
)

install(TARGETS bot_realsense_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_image_ops test/test_image_ops.cpp src/image_ops.cpp)
  target_include_directories(test_image_ops PRIVATE include)
  target_link_libraries(test_image_ops realsense2::realsense2)

  ament_add_gtest(test_depth_video test/test_depth_video.cpp src/depth_video.cpp)
  target_include_directories(test_depth_video PRIVATE src)

  ament_add_gtest(test_latest_slot test/test_latest_slot.cpp)
  target_include_directories(test_latest_slot PRIVATE include)

  ament_add_gtest(test_timing test/test_timing.cpp)
  target_include_directories(test_timing PRIVATE include)
endif()

ament_package()
