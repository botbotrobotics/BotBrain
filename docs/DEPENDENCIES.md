# Third-Party Dependencies

This document lists all third-party libraries and packages used in BotBrain.

## Frontend Dependencies

The web dashboard is built with Next.js 15 and React 19.

### Core Framework

| Package | Version | Description |
|---------|---------|-------------|
| next | ^15.2.3 | React framework for production |
| react | ^19.0.0 | UI component library |
| react-dom | ^19.0.0 | React DOM renderer |
| typescript | ^5 | TypeScript support |

### Supabase (Backend)

| Package | Version | Description |
|---------|---------|-------------|
| @supabase/ssr | ^0.6.1 | Server-side rendering support |
| @supabase/supabase-js | ^2.49.8 | Supabase JavaScript client |

### ROS & Robotics

| Package | Version | Description |
|---------|---------|-------------|
| roslib | ^1.4.1 | ROS JavaScript library |
| ros2d | ^0.10.0 | 2D visualization for ROS |
| ros3d | ^0.17.0 | 3D visualization for ROS |
| three | ^0.174.0 | 3D graphics library |
| urdf-loader | ^0.12.4 | URDF robot model loader |
| cbor-js | ^0.1.0 | CBOR encoding for ROS messages |

### UI & Styling

| Package | Version | Description |
|---------|---------|-------------|
| tailwindcss | ^3.4.1 | Utility-first CSS framework |
| tailwind-merge | ^3.0.2 | Tailwind class merging |
| tailwindcss-animate | ^1.0.7 | Animation utilities |
| clsx | ^2.1.1 | Class name utilities |
| lucide-react | ^0.479.0 | Icon library |
| @radix-ui/react-dropdown-menu | ^2.1.6 | Accessible dropdown menus |

### Data Visualization

| Package | Version | Description |
|---------|---------|-------------|
| apexcharts | ^4.5.0 | Interactive charts |
| react-apexcharts | ^1.7.0 | ApexCharts React wrapper |
| recharts | ^2.15.1 | Composable charting library |
| leaflet | ^1.9.4 | Interactive maps |

### Utilities

| Package | Version | Description |
|---------|---------|-------------|
| date-fns | ^4.1.0 | Date manipulation |
| lodash | ^4.17.21 | Utility functions |
| uuid | ^11.1.0 | UUID generation |
| jszip | ^3.10.1 | ZIP file handling |
| nipplejs | ^0.10.2 | Virtual joystick |
| eventemitter2 | ^6.4.9 | Event emitter |
| easeljs | ^1.0.2 | Canvas library |

### Development & Build

| Package | Version | Description |
|---------|---------|-------------|
| eslint | ^9.22.0 | Code linting |
| eslint-config-next | ^15.2.3 | Next.js ESLint config |
| eslint-plugin-react | ^7.37.4 | React linting rules |
| eslint-plugin-react-hooks | ^5.2.0 | React hooks linting |
| eslint-plugin-unused-imports | ^4.1.4 | Unused imports detection |
| @babel/eslint-parser | ^7.26.10 | Babel ESLint parser |
| @typescript-eslint/parser | ^8.27.0 | TypeScript ESLint parser |
| postcss | ^8 | CSS processing |
| dotenv | ^16.5.0 | Environment variables |
| string-replace-loader | ^3.1.0 | Webpack string replacement |

### TypeScript Types

| Package | Version | Description |
|---------|---------|-------------|
| @types/d3 | ^7.4.3 | D3.js type definitions |
| @types/jszip | ^3.4.0 | JSZip type definitions |
| @types/leaflet | ^1.9.16 | Leaflet type definitions |
| @types/lodash | ^4.17.16 | Lodash type definitions |
| @types/node | ^20 | Node.js type definitions |
| @types/react | ^19 | React type definitions |
| @types/react-dom | ^19 | React DOM type definitions |
| @types/react-gauge-chart | ^0.4.3 | Gauge chart types |
| @types/roslib | ^1.3.5 | ROSLIB type definitions |
| @types/three | ^0.174.0 | Three.js type definitions |
| @types/uuid | ^10.0.0 | UUID type definitions |

### Analytics

| Package | Version | Description |
|---------|---------|-------------|
| @vercel/speed-insights | ^1.2.0 | Performance monitoring |

---

## ROS 2 Dependencies

The BotBrain workspace uses ROS 2 Humble and the following packages.

### Core ROS 2

| Package | Description |
|---------|-------------|
| rclcpp | ROS 2 C++ client library |
| rclpy | ROS 2 Python client library |
| rclcpp_lifecycle | Lifecycle node support |
| std_msgs | Standard message types |
| geometry_msgs | Geometry message types |
| sensor_msgs | Sensor message types |
| nav_msgs | Navigation message types |
| tf2_ros | Transform library |

### Navigation & Localization

| Package | Description |
|---------|-------------|
| nav2_bringup | Nav2 navigation stack |
| nav2_bt_navigator | Behavior tree navigator |
| nav2_controller | Path following controller |
| nav2_planner | Path planning |
| nav2_behaviors | Recovery behaviors |
| rtabmap_ros | Visual SLAM |

### Perception

| Package | Description |
|---------|-------------|
| realsense2_camera | Intel RealSense driver |
| image_transport | Image transport |
| cv_bridge | OpenCV bridge |
| depthimage_to_laserscan | Depth to laser conversion |

### Communication

| Package | Description |
|---------|-------------|
| rosbridge_server | WebSocket bridge |
| twist_mux | Velocity multiplexer |

### AI & Vision

| Package | Description |
|---------|-------------|
| ultralytics | YOLOv8/v11 inference |
| TensorRT | NVIDIA GPU acceleration |

---

## License Information

### Frontend Licenses

Most frontend packages use MIT or Apache 2.0 licenses. Notable exceptions:
- **ultralytics (YOLO)**: AGPL-3.0 - Requires open-source distribution if modified

### ROS 2 Licenses

| Package | License |
|---------|---------|
| ROS 2 Core | Apache 2.0 |
| Nav2 | Apache 2.0 |
| RTABMap | BSD |
| Unitree SDK | Unitree License |

---

## Updating Dependencies

### Frontend
```bash
cd frontend
npm update
npm audit fix
```

### ROS 2 Workspace
```bash
cd botbrain_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

See the [main README](../README.md) for installation instructions.
