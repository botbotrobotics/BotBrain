

# booster_ros2_interface

booster_ros2_interface is a ROS 2 interface package that provides a series of message and service definitions for robot control.

## Overview

This interface package contains various message types and services that can be used in scenarios such as robot state publishing, command control, and RPC service invocation. It mainly includes the following:

- Various message definitions (under the msg/ directory): including robot low-level state (LowState), motor state (MotorState), IMU data (ImuState), etc.
- Service definitions (under the srv/ directory): including RPC service (RpcService) and Agent service (AgentService).

## Message Types

- **BoosterApiReqMsg/BoosterApiRespMsg**: Used for API request and response messages.
- **ButtonEventMsg**: Button event message.
- **FallDownState**: Robot fall down state message.
- **HandCommand/HandDdsMsg/HandParam**: Hand control-related messages.
- **ImuState**: IMU sensor state message.
- **LowCmd/LowState**: Low-level command and state messages.
- **MotorCmd/MotorState**: Motor command and state messages.
- **Odometer**: Odometer message.
- **RawBytesMsg**: Raw bytes message.
- **RemoteControllerState**: Remote controller state message.

## Service Types

- **AgentService**: Agent service used for request and response of string messages.
- **RpcService**: RPC service used for more complex data interactions.

## Usage

To use these messages and services, ensure that this interface package is correctly installed and built in your ROS 2 workspace. Then, you can create publishers, subscribers, clients, and service servers to utilize these defined messages and services.

## Dependencies

- ROS 2 (rclcpp)
- ament_cmake
- rosidl_default_generators
- rosidl_default_runtime

## Building

To build this package, use the ROS 2 build tool `ament_cmake`. Make sure all dependencies are installed, and then run the `colcon build` command.

## Contributing

Contributions of new message types or improvements to existing definitions are welcome. Please submit a PR or Issue to discuss and contribute code.

## License

This project currently uses the default ROS 2 license. Please see the LICENSE file for details.