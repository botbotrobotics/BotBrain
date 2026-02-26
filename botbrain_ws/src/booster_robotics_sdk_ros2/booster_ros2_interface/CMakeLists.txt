cmake_minimum_required(VERSION 3.8)
project(booster_interface)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoosterApiReqMsg.msg"
  "msg/BoosterApiRespMsg.msg"
  "msg/ButtonEventMsg.msg"
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  "msg/LowCmd.msg"
  "msg/ImuState.msg"
  "msg/LowState.msg"
  "msg/RawBytesMsg.msg"
  "msg/Odometer.msg"
  "msg/FallDownState.msg"
  "msg/RemoteControllerState.msg"
  "msg/HandCommand.msg"
  "msg/HandDdsMsg.msg"
  "msg/HandParam.msg"
  "msg/ProneBodyControlStatus.msg"
  "msg/RobotStatesMsg.msg"
  "msg/RobotReplayTrajID.msg"
  "msg/Subtitle.msg"
  "srv/RpcService.srv"
  "srv/AgentService.srv"
)

include("${CMAKE_CURRENT_LIST_DIR}/../.workflow/project-defined.cmake")


# 安装头文件
install(DIRECTORY include/
        DESTINATION include/
)

macro(default_install_none)
endmacro()

macro(custom_install_env)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
endmacro()

set(CUSTOM_VAR_LIB_BINARY_INSTALL "1")
closure_import_install_project(default_install_none custom_install_env)


# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()