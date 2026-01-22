cmake_minimum_required(VERSION 3.8)
project(g1_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(bot_custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(nav_msgs REQUIRED)

find_library(UNITREE_SDK2 NAMES unitree_sdk2 PATHS /usr/local/lib)
find_library(DDSCXX NAMES ddscxx PATHS /usr/local/lib)
find_library(DDSC NAMES ddsc PATHS /usr/local/lib)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/g1_driver
    /usr/local/include/ddscxx
)

add_executable(g1_write_node 
               src/g1_driver/g1_driver.cpp
               src/g1_write_node.cpp
               src/g1_write.cpp
)

install(TARGETS g1_write_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(g1_write_node 
              rclcpp
              rclcpp_lifecycle
              bot_custom_interfaces
              geometry_msgs
              std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    config
    urdf
    meshes
    xacro
    launch
    maps
  DESTINATION
    share/${PROJECT_NAME}/
)

target_compile_definitions(g1_write_node
  PRIVATE
    G1_PKG_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(g1_write_node
  ${UNITREE_SDK2}
  ${DDSCXX}
  ${DDSC}
  pthread
)

# Python nodes and modules
install(PROGRAMS
  scripts/g1_read.py
  scripts/g1_controller_commands.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME}) # Only needed in case rosidl_generate_interfaces is not called

ament_package()
