# bot_custom_interfaces

The custom ROS2 interfaces package for the BotBrain robot platform. This package defines all custom message types, service definitions, and action interfaces used throughout the robot system.

## Description

The `bot_custom_interfaces` package serves as the central repository for all custom ROS2 communication interfaces used by the R1 robot platform. It provides standardized message types for robot control, status reporting, device management, and system communication across all robot packages.

## Directory Structure

```
bot_custom_interfaces/
├── msg/                          # Custom message definitions
├── srv/                          # Custom service definitions
├── action/                       # Custom action definitions
├── CMakeLists.txt                # CMake build configuration
├── package.xml                   # ROS2 package manifest
└── README.md                     # This file
```

## Adding New Interfaces

### Adding a New Message

1. **Create Message File**: Create a new `.msg` file in the `msg/` directory
   ```msg
   # Example: MyCustomMessage.msg
   std_msgs/Header header
   string data
   int32 value
   ```

2. **Update CMakeLists.txt**: Add the message to the `rosidl_generate_interfaces` call
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... existing interfaces ...
     "msg/MyCustomMessage.msg"
   )
   ```

3. **Build and Test**: Build the package and verify the interface is generated
   ```bash
   colcon build --packages-select bot_custom_interfaces
   ros2 interface show bot_custom_interfaces/msg/MyCustomMessage
   ```

### Adding a New Service

1. **Create Service File**: Create a new `.srv` file in the `srv/` directory
   ```srv
   # Example: MyCustomService.srv
   # Request
   string command
   int32 value
   ---
   # Response
   bool success
   string message
   ```

2. **Update CMakeLists.txt**: Add the service to the `rosidl_generate_interfaces` call
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... existing interfaces ...
     "srv/MyCustomService.srv"
   )
   ```

### Adding a New Action

1. **Create Action File**: Create a new `.action` file in the `action/` directory
   ```action
   # Example: MyCustomAction.action
   # Goal
   string goal_data
   ---
   # Result
   bool success
   string result_message
   ---
   # Feedback
   float32 progress
   string status
   ```

2. **Update CMakeLists.txt**: Add the action to the `rosidl_generate_interfaces` call
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... existing interfaces ...
     "action/MyCustomAction.action"
   )
   ```

### Interface Naming Conventions

- **Messages**: Use descriptive names ending with the data type (e.g., `RobotStatus.msg`, `DeviceInfo.msg`)
- **Services**: Use action-oriented names (e.g., `StartMapping.srv`, `SwitchGait.srv`)
- **Actions**: Use action-oriented names ending with "Action" (e.g., `NavigateAction.action`)
- **Fields**: Use snake_case for field names (e.g., `robot_status`, `device_id`)

### Interface Design Guidelines

1. **Keep Interfaces Simple**: Use basic ROS2 types when possible
2. **Use Standard Types**: Prefer `std_msgs` types over custom types
3. **Include Headers**: Add `std_msgs/Header` for timestamped messages
4. **Provide Feedback**: Include success/error information in service responses
5. **Document Fields**: Add comments explaining field purposes
6. **Version Compatibility**: Consider backward compatibility when modifying interfaces

## Dependencies

### Build Dependencies
- `ament_cmake` - CMake build system
- `std_msgs` - Standard ROS2 message types
- `action_msgs` - Standard ROS2 action types
- `rosidl_default_generators` - Interface generation

### Runtime Dependencies
- `rosidl_default_runtime` - Interface runtime support

---

**Note**: This package is fundamental to the entire robot system. Changes to interfaces can break compatibility with existing packages. Always test thoroughly and consider backward compatibility when modifying existing interfaces.
