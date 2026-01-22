cmake_minimum_required(VERSION 3.8)
project(bot_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerButtonsState.msg"
  "msg/RobotStatus.msg"
  "msg/DeviceInfo.msg"
  "msg/Status.msg"
  "msg/StatusArray.msg"
  "srv/Mode.srv"
  "srv/Pose.srv"
  "srv/BodyHeight.srv"
  "srv/ContinuousGait.srv"
  "srv/Euler.srv"
  "srv/FootRaiseHeight.srv"
  "srv/SwitchGait.srv"
  "srv/SwitchJoystick.srv"
  "srv/SpeedLevel.srv"
  "srv/LLMPrompt.srv"
  "srv/MoveForward.srv"
  "srv/Turn.srv"
  "srv/CurrentMode.srv"
  "srv/LightControl.srv"
  "srv/ObstacleAvoidance.srv"
  "srv/HikvisionParameters.srv"
  "srv/MotorControl.srv"
  "srv/StartMapping.srv"
  "srv/StopMapping.srv"
  "srv/StateMachine.srv"
  "srv/ArmCmd.srv"
  "msg/Names.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
