<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_yolo - ROS2 YOLO Inference + Tracking (Ultralytics / TensorRT)

[🇧🇷 Versão em Português](README_br.md)

A ROS 2 package that runs Ultralytics YOLO with a TensorRT engine for fast object detection (and optional tracking). It subscribes to a camera `sensor_msgs/Image`, performs inference, publishes an annotated image (raw and compressed), and publishes detections as compact JSON.

## Features

- **Lifecycle node architecture**
- **TensorRT export-on-first-run**: exports `.engine` from a `.pt` if missing
- **Detection or tracking**: `predict()` or `track()` (BoT-SORT via Ultralytics tracker cfg)
- **Annotated outputs**: publishes annotated `Image` and JPEG `CompressedImage`
- **Detections as JSON**: publishes compact JSON for easy integration
- **YAML configuration**: user-tunable parameters live in `config/yolo.yaml` 

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Published Topics](#published-topics)
- [Lifecycle Management](#lifecycle-management)
- [Directory Structure](#directory-structure)
- [Dependencies](#dependencies)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)
- [License Notice](#license-notice)

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- Python 3
- OpenCV + cv_bridge
- Ultralytics
- NVIDIA GPU runtime suitable for TensorRT (required for `.engine` export / inference)

### Install Python Dependencies
NOTE: if you are working with ```botbrain_ws```, you can skip this step

```bash
pip3 install ultralytics

Build the Package

cd ~/your_ros2_workspace
colcon build --packages-select bot_yolo
source install/setup.bash
```

## Quick Start

### Launch the Node

```bash
ros2 launch bot_yolo yolo.launch.py
```
### Configure and Activate (Lifecycle)

The node starts in the **unconfigured** state. To run inference:
```bash
ros2 lifecycle set /yolo_node configure
ros2 lifecycle set /yolo_node activate
```
Inspect node state
```bash
ros2 lifecycle get /yolo_node
```
Inspect Outputs
```bash
# annotated image (raw)
ros2 topic echo /yolo/image

# annotated image (compressed)
ros2 topic echo /yolo/image_compressed

# JSON detections
ros2 topic echo /yolo/detections
```

## Configuration

The main configuration file is:
- config/yolo.yaml

This file contains the parameters most users will change: input topic, inference knobs, tracking, and overlay settings.

### Default Configuration
```
yolo_node:
  ros__parameters:
    # Subscriptions
    camera_topic: "front_camera/color/image_raw" #substitute with your own camera topic

    # Inference
    imgsz: 640 #image size
    conf: 0.25 #confidence score
    device: 0 #for CUDA

    # Tracking
    use_tracking: true
    tracker_cfg: "botsort.yaml"

    # Drawing
    draw_labels: true
    label_every_n: 1 #skips n frames when drawing (for efficiency)
    line_thickness: 2
    font_scale: 0.5
    font_thickness: 1
```

Behavior:
- If engine_path does not exist, the node exports a TensorRT engine from pt_path during **configure**.

## Published Topics

| **Topic** | **Type** | **Description** |
|---|---|---|
| `/yolo/image` | `sensor_msgs/Image` | Annotated image (raw) |
| `/yolo/image_compressed` | `sensor_msgs/CompressedImage` | Annotated image as JPEG (resized + compressed) |
| `/yolo/detections` | `std_msgs/String` | JSON payload with detections |

### Detections JSON Format

Published on /yolo/detections as a JSON string:
```
{
  "detections_num": "2",
  "detected_objects": [
    {"object_id":"0","object":"person","confidence":"0.932","track_id":12},
    {"object_id":"2","object":"car","confidence":"0.811","track_id":null}
  ]
}
```
## Lifecycle Management

### Lifecycle states:
- **Unconfigured**: no inference resources allocated
- **Inactive**: configured, but not running inference
- **Active**: inference running, publishing outputs
- **Finalized**: cleaned up and shut down

### Lifecycle commands:
```bash
ros2 lifecycle set /yolo_node configure
ros2 lifecycle set /yolo_node activate
ros2 lifecycle set /yolo_node deactivate
ros2 lifecycle set /yolo_node cleanup
ros2 lifecycle set /yolo_node shutdown
```
## Directory Structure
```
bot_yolo/
├── bot_yolo/                 # Python package
│   ├── __init__.py
│   └── yolo_node.py
├── config/
│   └── yolo.yaml
├── launch/
│   └── yolo.launch.py
├── resource/
│   └── bot_yolo
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```
## Dependencies

ROS 2 Packages
- rclpy
- sensor_msgs
- std_msgs
- cv_bridge

Python / External
- numpy (1.23)
- opencv-python (or system OpenCV)
- ultralytics

## Troubleshooting

### Engine export fails

- Verify `pt_path` exists
- Ensure your TensorRT / CUDA runtime is installed and compatible
- Check available GPU memory (engine export may require significant VRAM)

### Input topic mismatch

- Confirm the camera topic name in `config/yolo.yaml`
- Ensure the input message type is `sensor_msgs/Image`

## Credits

This project uses **YOLO models and APIs from Ultralytics**.
- Ultralytics YOLO: https://github.com/ultralytics/ultralytics
- © Ultralytics, licensed under the AGPL-3.0 License

## License Notice

Ultralytics YOLO is licensed under the AGPL-3.0 license.
This repository does not include Ultralytics source code; it depends on the `ultralytics` Python package.

<p align="center">Made with ❤️ in Brazil</p>


<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
