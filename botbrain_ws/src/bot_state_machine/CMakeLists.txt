cmake_minimum_required(VERSION 3.5)
project(bot_state_machine)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(bot_custom_interfaces REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(state_machine_node 
               src/state_machine_node.cpp 
               src/lifecycle_manager.cpp
               src/state_controller.cpp
               src/graph_node.cpp
)

ament_target_dependencies(state_machine_node 
               rclcpp
               rclcpp_lifecycle
               lifecycle_msgs
               nlohmann_json
               bot_custom_interfaces
)

target_link_libraries(state_machine_node nlohmann_json::nlohmann_json)


# Install targets
install(TARGETS
  state_machine_node 
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()