cmake_minimum_required(VERSION 3.8)
project(booster_rtc_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(booster_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(BEFORE $(PROJECT_SOURCE_DIR)/../../../booster_robotics_sdk/include)
include_directories(BEFORE $(PROJECT_SOURCE_DIR))

# client 添加可执行文件
add_executable(rtc_client_node src/client.cpp)

ament_target_dependencies(rtc_client_node 
  rclcpp 
  std_msgs
  booster_interface
  rosidl_default_runtime)

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

# 安装可执行文件
install(TARGETS
  rtc_client_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
