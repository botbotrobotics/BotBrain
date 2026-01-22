<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

<p align="center">
  一个大脑，任意机器人。
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-官网-000?logo=vercel&logoColor=white" alt="官网"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
  <a href="https://huggingface.co/botbot-ai"><img src="https://img.shields.io/badge/-Hugging%20Face-FFD54F?logo=huggingface&logoColor=black" alt="Hugging Face"></a>
</p>

<h1 align="center">BotBrain ROS2 工作空间</h1>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="许可证: MIT">
  <img src="https://img.shields.io/badge/Platform-Ubuntu_22.04-orange" alt="Ubuntu 22.04">
</p>

<p align="center">
  <a href="../../../botbrain_ws/README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-green" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

> **注意：** 英文版本是官方且最新的文档。此翻译可能不反映最新的更改。

## 概述

**BotBrain 工作空间**是一个模块化、开源的 ROS2 框架，用于自主机器人控制、导航和定位。采用机器人无关架构设计，可在多个机器人平台上快速开发和部署先进的机器人应用。

**主要特性：**
- 🤖 **多机器人支持**：单一代码库支持 Go2、Tita、G1 和自定义机器人
- 🗺️ **视觉 SLAM**：基于 RTABMap 的定位，支持双摄像头
- 🎮 **多种控制模式**：手柄、Web 界面和自主导航
- 👁️ **AI 视觉**：YOLOv8/v11 目标检测
- 🐳 **Docker 就绪**：支持 GPU 加速的容器化部署
- 🔄 **生命周期管理**：强健的节点编排和故障恢复


## 目录

- [硬件要求](#硬件要求)
- [快速开始](#快速开始)
- [仓库结构](#仓库结构)
- [创建自定义机器人包](#创建自定义机器人包)
- [包概述](#包概述)
- [Docker 服务](#docker-服务)
- [配置](#配置)

## 硬件要求

### 支持的机器人平台
- **Unitree Go2**
- **Unitree G1**
- **Tita**
- **自定义机器人** - 请参阅[自定义机器人包指南](#创建自定义机器人包)

### 必需硬件
- **机器人平台**：上述支持的机器人之一
- **机载计算机**：
  - Nvidia Jetson Orin 系列或更新版本
- **传感器**：
  - Intel RealSense 摄像头（用于视觉 SLAM）
  - LiDAR（用于基于 LiDAR 的 SLAM）
- **网络**：
  - 与机器人的以太网连接
  - Wi-Fi 适配器（用于远程控制）

### 可选硬件
- **游戏手柄**：用于遥操作

## 快速开始

### 使用 Docker Compose 启动

用于容器化部署：

```bash
# 启动所有服务
docker compose up -d

# 启动特定服务
docker compose up -d state_machine bringup localization navigation

# 查看日志
docker compose logs -f bringup

# 停止服务
docker compose down
```

### 验证系统正在运行

```bash
# 检查活动节点
ros2 node list

# 检查话题
ros2 topic list
```

### 开发容器

如果您想使用相同的 docker 镜像进行开发，而不创建新服务，可以运行交互式开发容器：

```bash
# 初始化开发容器
cd botbrain_ws
docker compose up dev -d

# 初始化交互式终端
docker compose exec dev bash
```

一旦交互式终端打开，您可以使用它来创建、编译和运行尚未与 docker 服务集成的新功能。

## 仓库结构

```
botbrain_ws/
├── README.md                          # 本文件
├── LICENSE                            # MIT 许可证
│
├── robot_config.yaml                  # 主配置文件
├── install.sh                         # 自动安装脚本
├── robot_select.sh                    # 机器人选择助手
│
├── docker-compose.yaml                # Docker 服务定义
├── botbrain.service                   # Systemd 自启动服务
├── cyclonedds_config.xml              # DDS 中间件配置
│
└── src/                               # ROS 2 包
    │
    ├── 核心系统包
    │   ├── bot_bringup/               # 主启动 & twist mux 协调
    │   ├── bot_custom_interfaces/     # 自定义消息、服务、动作
    │   └── bot_state_machine/         # 生命周期 & 状态管理
    │
    ├── 机器人模型 & 可视化
    │   └── bot_description/           # URDF/XACRO 模型 & robot_state_publisher
    │
    ├── 导航 & 定位
    │   ├── bot_localization/          # RTABMap SLAM（视觉 & LiDAR）
    │   └── bot_navigation/            # Nav2 导航栈
    │
    ├── 感知 & 控制
    │   ├── bot_yolo/                  # YOLOv8/v11 目标检测
    │   └── joystick-bot/              # 游戏手柄接口
    │
    ├── AI & 监控
    │   ├── bot_jetson_stats/          # Jetson 硬件监控
    │   └── bot_rosa/                  # ROSA AI 自然语言控制
    │
    └── 机器人专用包
        ├── g1_pkg/                    # Unitree G1 硬件接口
        ├── go2_pkg/                   # Unitree Go2 硬件接口
        ├── tita_pkg/                  # Tita 硬件接口
        └── your_robot_pkg/            # 您的自定义机器人（见下方指南）
```

## 创建自定义机器人包

要添加对新机器人平台的支持，请按照此指南使用 [go2_pkg](../../../botbrain_ws/src/go2_pkg) 作为参考模板。

**注意**：go2_pkg 包通过 ROS 2 话题与 Unitree Go2 机器人通信（订阅 Unitree 的原生 ROS 2 话题并以 BotBrain 格式重新发布）。您的自定义机器人包可以使用类似的基于话题的通信、直接硬件 API 或 SDK 接口，具体取决于您机器人的架构。其目的是在 botbrain_ws 包和机器人之间创建标准的包接口。

### 必需的包结构

您的自定义机器人包必须遵循此命名约定才能与所有包无缝配合：`{robot_model}_pkg`

```
{robot_model}_pkg/
├── package.xml                        # ROS 2 包清单
├── CMakeLists.txt                     # 构建配置
├── README.md                          # 包文档
├── launch/
│   └── robot_interface.launch.py     # 必需：主硬件接口启动器
├── config/
│   └── nav2_params.yaml               # 必需：导航参数
├── scripts/
│   ├── {robot_model}_read.py                # 必需：从机器人读取传感器数据
│   └── {robot_model}_write.py               # 必需：向机器人发送命令
├── {robot_model}_pkg/                 # Python 包目录
│   └── tools/                         # 可选：ROSA AI 助手工具
│       ├── __init__.py                # 空包初始化
│       └── {robot_model}.py           # 用于 ROSA 集成的 LangChain 工具
├── xacro/
│   └── robot.xacro                    # 必需：机器人 URDF 模型
└── meshes/
    └── *.dae, *.stl                   # 视觉和碰撞网格
```

### 分步创建指南

#### 1. 创建新的 ROS 2 包

使用 ROS 2 工具创建包结构（如果您的主机系统未安装 ros2，可以从开发容器中执行此操作）：

```bash
cd src/
ros2 pkg create {robot_model}_pkg --build-type ament_cmake --dependencies rclcpp rclpy
cd {robot_model}_pkg
```

创建必需的目录：
```bash
mkdir -p launch config scripts xacro meshes maps
```

#### 2. 配置 package.xml

编辑 `package.xml` 并添加必需的依赖项：
- 将 `bot_custom_interfaces` 添加为依赖项
- 更新包名称、版本、描述和维护者信息
- 确保包含所有传感器消息依赖项

#### 3. 配置 CMakeLists.txt

更新构建配置以安装所有包资源：
- 安装 launch 文件目录
- 安装 config 文件目录
- 将脚本安装为可执行文件
- 安装 xacro、urdf 和 meshes 目录
- 对 Python 模块使用 `ament_python_install_package()`

#### 4. 创建硬件接口启动文件

**关键**：创建 `launch/robot_interface.launch.py`（必须使用此确切名称）

此启动文件必须：
- 从工作空间根目录读取 `robot_config.yaml`
- 提取 `robot_name` 用于命名空间配置
- 启动用于硬件读写的生命周期节点
- 使用 `launch_ros.actions` 中的 `LifecycleNode`
- 为所有节点应用正确的命名空间

参考：请参阅 [go2_pkg/launch/robot_interface.launch.py](../../../botbrain_ws/src/go2_pkg/launch/robot_interface.launch.py) 获取完整示例。

#### 5. 实现硬件接口节点

**创建 `scripts/{robot_model}_read.py`** - 读取传感器数据并发布到 ROS 2：

此生命周期节点必须：
- 初始化为名为 `robot_read_node` 的 `LifecycleNode`
- 实现生命周期回调：`on_configure`、`on_activate`、`on_deactivate`、`on_cleanup`
- 在 `on_configure` 中：创建里程计、IMU、关节状态和电池的发布者
- 在 `on_activate` 中：从机器人硬件/话题启动数据读取循环（通常 50Hz）
- 处理机器人传感器数据并发布到 ROS 2 话题
- 在 `on_deactivate` 中：停止数据发布但保持连接
- 在 `on_cleanup` 中：关闭硬件连接并释放资源

参考：请参阅 [go2_pkg/scripts/go2_read.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_read.py) 获取完整实现。

**创建 `scripts/{robot_model}_write.py`** - 接收命令并发送到机器人：

此生命周期节点必须：
- 初始化为名为 `robot_write_node` 的 `LifecycleNode`
- 在 `on_configure` 中：创建 `cmd_vel_out` 话题的订阅者并建立与机器人的通信
- 实现回调以接收速度命令并转发到机器人硬件
- 在 `on_deactivate` 中：向机器人发送停止命令（零速度）
- 在 `on_cleanup` 中：关闭硬件连接并释放资源
- 可选：实现机器人特定的服务（模式切换、步态控制等）

参考：请参阅 [go2_pkg/scripts/go2_write.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_write.py) 获取完整实现。

#### 6. 创建导航参数

使用您机器人的规格创建 `config/nav2_params.yaml`。请参阅 [Nav2 文档](https://docs.nav2.org/) 作为参考。

您需要在节点配置部分添加通配符。请参阅 [go2_pkg/config/nav2_params.yaml](../../../botbrain_ws/src/go2_pkg/config/nav2_params.yaml) 获取完整配置示例。

#### 7. 创建机器人描述（XACRO）

使用您机器人的 URDF 模型创建 `xacro/robot.xacro`：

您的 XACRO 文件应定义：
- `base_link` 作为机器人主体的主链接
- `interface_link` 作为机器人和 BotBrain 之间的接口部分
- 所有机器人关节和链接（腿、臂等）
- 传感器链接（摄像头、LiDAR、IMU）
- 用于 RViz 可视化的视觉网格
- 用于导航的碰撞网格
- 关节限制和动力学
- 惯性属性

参考：请参阅 [go2_pkg/xacro/robot.xacro](../../../botbrain_ws/src/go2_pkg/xacro/robot.xacro) 获取完整的机器人描述。

#### 8. 配置工作空间

更新工作空间的 `robot_config.yaml`（可以通过 install.sh 完成）：

```yaml
robot_configuration:
  robot_name: "my_robot"               # 所有话题的命名空间
  robot_model: "your_robot"            # 必须匹配包名（不含"_pkg"）
  description_file_type: "xacro"       # "xacro" 或 "urdf"
  network_interface: "eth0"            # 用于机器人通信的网络接口
```

**重要**：`robot_model` 字段必须匹配您的包名（**不含** `_pkg` 后缀）：
- 包名：`your_robot_pkg`
- robot_model：`your_robot`

#### 9. 编译和测试

```bash
# 编译您的包
cd ~/botbrain_workspace/BotBrain/botbrain_ws
colcon build --packages-select your_robot_pkg

# Source 工作空间
source install/setup.bash

# 测试您的硬件接口
ros2 launch your_robot_pkg robot_interface.launch.py

# 使用完整系统启动
ros2 launch bot_bringup bringup.launch.py
```

您可以使用开发容器编译和测试新包。

#### 10. 创建 ROSA 工具（可选）

**ROSA**（机器人操作系统助手）是一个 AI 助手，可以使用自然语言控制您的机器人。通过为 ROSA 创建工具，用户可以使用对话式命令与您的机器人交互。

**创建工具目录结构：**

```bash
mkdir -p {robot_model}_pkg/tools
touch {robot_model}_pkg/tools/__init__.py
touch {robot_model}_pkg/tools/{robot_model}.py
```

### 包集成点

BotBrain 系统将根据以下约定自动查找并使用您的包：

1. **包命名**：`{robot_model}_pkg` 格式
2. **启动文件**：`launch/robot_interface.launch.py`（必须使用此确切名称）
3. **导航配置**：`config/nav2_params.yaml`（由 bot_navigation 使用）
4. **描述文件**：`xacro/robot.xacro` 或 `urdf/robot.urdf`（由 bot_description 使用）

### 您的包必须提供的必需话题

为实现完整的系统集成，您的硬件接口应发布：

| 话题 | 消息类型 | 描述 | 频率 |
|-------|--------------|-------------|-----------|
| `/{namespace}/odom` | nav_msgs/Odometry | 机器人里程计 | 50Hz |
| `/{namespace}/imu` | sensor_msgs/Imu | IMU 数据 | 100Hz |
| `/{namespace}/joint_states` | sensor_msgs/JointState | 关节位置/速度 | 50Hz |

并订阅：

| 话题 | 消息类型 | 描述 |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | geometry_msgs/Twist | 来自 twist_mux 的速度命令 |


## 包概述

### 核心系统包

| 包 | 描述 | 文档 |
|---------|-------------|---------------|
| [bot_bringup](../../../botbrain_ws/src/bot_bringup) | 主启动协调、twist 多路复用器和系统编排 | [README](../../../botbrain_ws/src/bot_bringup/README.md) |
| [bot_state_machine](../../../botbrain_ws/src/bot_state_machine) | 生命周期管理、节点协调和系统状态控制 | [README](../../../botbrain_ws/src/bot_state_machine/README.md) |
| [bot_custom_interfaces](../../../botbrain_ws/src/bot_custom_interfaces) | 自定义 ROS 2 消息、服务和动作 | [README](../../../botbrain_ws/src/bot_custom_interfaces/README.md) |
| [bot_description](../../../botbrain_ws/src/bot_description) | 机器人 URDF/XACRO 模型和 robot_state_publisher | [README](../../../botbrain_ws/src/bot_description/README.md) |

### 导航 & 定位

| 包 | 描述 | 文档 |
|---------|-------------|---------------|
| [bot_localization](../../../botbrain_ws/src/bot_localization) | RTABMap SLAM，支持视觉和基于 LiDAR 的建图 | [README](../../../botbrain_ws/src/bot_localization/README.md) |
| [bot_navigation](../../../botbrain_ws/src/bot_navigation) | Nav2 导航栈，采用机器人无关配置 | [README](../../../botbrain_ws/src/bot_navigation/README.md) |

### 感知 & 控制

| 包 | 描述 | 文档 |
|---------|-------------|---------------|
| [bot_yolo](../../../botbrain_ws/src/bot_yolo) | YOLOv8/v11 目标检测，支持 TensorRT 加速 | [README](../../../botbrain_ws/src/bot_yolo/README.md) |
| [joystick-bot](../../../botbrain_ws/src/joystick-bot) | 游戏手柄接口，带死人开关安全功能 | [README](../../../botbrain_ws/src/joystick-bot/README.md) |

### 机器人专用包

| 包 | 描述 | 文档 |
|---------|-------------|---------------|
| [go2_pkg](../../../botbrain_ws/src/go2_pkg) | Unitree Go2 四足机器人硬件接口和描述 | [README](../../../botbrain_ws/src/go2_pkg/README.md) |
| [tita_pkg](../../../botbrain_ws/src/tita_pkg) | Tita 四足机器人硬件接口和描述 | [README](../../../botbrain_ws/src/tita_pkg/README.md) |

## Docker 服务

工作空间包含多个 Docker 服务用于容器化部署：

| 服务 | 描述 | 自启动 | 依赖 |
|---------|-------------|------------|--------------|
| `dev` | 开发容器（交互式） | 否 | - |
| `builder_base` | 编译所有工作空间包 | 否 | - |
| `state_machine` | 生命周期管理服务 | 是 | - |
| `bringup` | 主机器人启动 | 是 | state_machine |
| `localization` | RTABMap 定位 | 是 | bringup |
| `navigation` | Nav2 导航服务器 | 是 | localization |
| `rosa` | AI 工具调用服务 | 是 | bringup |
| `yolo` | 目标检测服务 | 是 | bringup |

### Docker 使用

```bash
# 启动所有服务
docker compose up -d

# 启动特定服务及其依赖
docker compose up -d navigation  # 自动启动 bringup、localization

# 查看日志
docker compose logs -f bringup

# 停止所有服务
docker compose down

# 代码更改后重新编译
docker compose build
docker compose up -d
```

## 配置

### 主配置文件

[robot_config.yaml](../../../botbrain_ws/robot_config.yaml) 文件是中心配置点：

```yaml
robot_configuration:

  # 机器人标识符 - 用作所有话题的命名空间
  robot_name: ""                    # 示例："go2_robot1"、"tita_lab"

  # 机器人类型 - 决定启动哪个硬件包
  robot_model: "go2"                # 选项："go2"、"tita"、"your_robot"

  # 描述文件格式
  description_file_type: "xacro"    # 选项："xacro"、"urdf"

  # 用于 ROS2 通信的网络接口
  network_interface: "eno1"         # 示例："eth0"、"wlan0"、"eno1"

  # Tita 专用：Tita 机器人通信的命名空间
  tita_namespace: "tita3036731"     # 仅当 robot_model: "tita" 时使用

  # 用于 AI 功能的 OpenAI API 密钥（可选）
  openai_api_key: ""                # ROSA AI 助手必需

  # Wi-Fi 配置（可选）
  wifi_interface: ""                # Wi-Fi 接口名称（例如："wlan0"）
  wifi_ssid: ""                     # Wi-Fi 网络 SSID
  wifi_password: ""                 # Wi-Fi 网络密码
```

### 网络配置

工作空间使用 CycloneDDS 进行 ROS 2 通信。配置在 [cyclonedds_config.xml](../../../botbrain_ws/cyclonedds_config.xml)：

在 [robot_config.yaml](../../../botbrain_ws/robot_config.yaml) 中设置网络接口以匹配您的硬件连接。

### Systemd 自启动服务

[botbrain.service](../../../botbrain_ws/botbrain.service) 文件启用开机自启动：

```bash
# 安装服务（由 install.sh 完成）
sudo cp botbrain.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable botbrain.service

# 手动控制
sudo systemctl start botbrain.service   # 立即启动
sudo systemctl stop botbrain.service    # 停止
sudo systemctl status botbrain.service  # 检查状态

# 查看日志
journalctl -u botbrain.service -f
```

### 添加新机器人支持

请参阅上面的[创建自定义机器人包](#创建自定义机器人包)指南。我们特别欢迎添加新机器人平台支持的贡献！

<p align="center">用 ❤️ 在巴西制作</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot 图标" width="110">
</p>
