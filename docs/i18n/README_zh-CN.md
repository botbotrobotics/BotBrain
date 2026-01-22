<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="../images/Botbrainlogo.png" alt="BotBot" width="400">
  </a>
</p>

<p align="center">
  一个大脑，任意机器人。
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Next.js-15-black?logo=next.js" alt="Next.js 15">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="MIT 许可证">
  <img src="https://img.shields.io/badge/Platform-Jetson-76B900?logo=nvidia" alt="Jetson">
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-官网-000?logo=vercel&logoColor=white" alt="官网"></a>
  <a href="https://discord.gg/CrTbJzxXes"><img src="https://img.shields.io/badge/-Discord-5865F2?logo=discord&logoColor=white" alt="Discord"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
</p>

<p align="center">
  <a href="../../README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-green" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

> **注意：** 英文版本是官方且最新的文档。此翻译可能不反映最新的更改。

# BotBrain 开源版 (BBOSS) <img src="../images/bot_eyes.png" alt="🤖" width="50" style="vertical-align: middle;">

BotBrain 是一套模块化的开源软件和硬件组件集合，让您能够通过简单但功能强大的 Web 界面来驾驶、观看、建图、导航（手动或自主）、监控和管理腿式（四足、双足和人形）或轮式 ROS2 机器人。硬件提供可 3D 打印的支架和外壳，让您可以轻松地将 BotBrain 安装到您的机器人上。

- 围绕 Intel RealSense D435i 和 NVIDIA Jetson 系列设计
- 官方支持的开发板：Jetson Nano、Jetson Orin Nano（AGX 和 Thor 支持即将推出）
- 一切都是模块化的 - 您不需要运行所有模块（一些重型 AI 模块需要 Orin AGX）

<h2 align="center">✨ 功能概览</h2>

<table>
  <tr>
    <td align="center" width="50%">
      <img src="../images/gifs/Dash:Fleet.gif" alt="仪表板和车队控制" width="400"><br>
      <h3>仪表板和车队控制</h3>
      <p>完整的仪表板，可查看状态、机器人信息并快速跳转到其他部分</p>
    </td>
    <td align="center" width="50%">
      <img src="../images/gifs/Cockpitscreenstudio.gif" alt="驾驶舱" width="400"><br>
      <h3>驾驶舱</h3>
      <p>预定义的控制页面，包含完整的前后摄像头、3D 模型、地图和导航以及快捷控制</p>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="../images/gifs/MyUI.gif" alt="我的界面" width="400"><br>
      <h3>我的界面</h3>
      <p>可自定义的控制界面，具有驾驶舱的所有功能</p>
    </td>
    <td align="center" width="50%">
      <img src="../images/gifs/Missions.gif" alt="任务" width="400"><br>
      <h3>任务</h3>
      <p>创建任务让机器人执行并自主导航</p>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="../images/gifs/Health.gif" alt="系统健康" width="400"><br>
      <h3>系统健康</h3>
      <p>查看 BotBrain 的完整健康状态：CPU/GPU/RAM 使用率、状态机节点控制和状态、WiFi 连接控制</p>
    </td>
    <td align="center" width="50%">
      <img src="../images/gifs/Profile.gif" alt="用户配置" width="400"><br>
      <h3>用户配置</h3>
      <p>自定义 BotBrain 的外观，设置自定义颜色和速度配置</p>
    </td>
  </tr>
</table>

<p align="center">
  <img src="../images/assembly.gif" alt="BotBrain 组装" width="600"><br>
  <h3 align="center">开源硬件</h3>
  <p>快速 3D 打印，易于组装，专为适配任何机器人而设计。
  让您的机器人在 30 分钟内使用 BotBrain 运行起来。</p>
</p>

<p align="center">
  <a href="https://youtu.be/VBv4Y7lat8Y">📹 观看 BotBrain 在我们办公室完成 1 小时自主巡逻</a>
</p>


## 完整功能列表

### 多机器人平台支持

- **Unitree Go2 & Go2-W** - 四足机器人，具有完整的硬件接口和控制
- **Unitree G1** - 人形机器人，具有上身姿态控制和 FSM 转换
- **DirectDrive Tita** - 双足机器人，具有完整控制
- **自定义机器人** - 可扩展框架，用于添加任何 ROS2 兼容平台
- **腿式和轮式** - 架构支持两种运动类型

### 硬件和传感器

- **可 3D 打印外壳** - 卡扣式设计，带有机器人专用安装适配器（Go2、G1 和 Direct drive Tita）
- **Intel RealSense D435i** - 双摄像头支持，用于查看和 SLAM/导航
- **IMU 和里程计** - 从所有支持平台实时姿态估计
- **电池监控** - 每个机器人的电池状态和续航估计

### AI 和感知

- **YOLOv8/v11 目标检测** - 80+ 类别，TensorRT 优化，BotBrain 上实时跟踪
- **Moondream AI** - 多模态视觉理解和场景分析
- **ROSA 自然语言控制** - 通过 LLM 进行对话式机器人命令
- **检测历史** - 可搜索的日志，包含图像和信息/描述

### 自主导航

- **RTABMap SLAM** - 使用单个或双 RealSense D435i 摄像头进行视觉建图
- **Nav2 集成** - 路径规划、动态避障、恢复行为
- **任务规划** - 创建和执行多航点自主巡逻
- **点击导航** - 直接在地图界面上设置目标
- **地图管理** - 保存、加载、切换和设置起始位置

### 系统编排

- **生命周期管理** - 带有依赖排序的协调节点启动/关闭
- **状态机** - 带有自动开/关的系统状态
- **基于优先级的速度控制** - 6 级命令仲裁（手柄 > 导航 > AI）
- **死人开关** - 所有运动命令的硬件/软件安全锁
- **紧急停止** - 全面的急停序列

### 控制接口

- **驾驶舱** - 预配置的控制页面，包含摄像头、3D 模型、地图和快捷操作
- **我的界面** - 可拖放的可自定义仪表板，带有可调整大小的小部件
- **虚拟摇杆** - 触摸/鼠标双摇杆控制，带有速度调节
- **游戏手柄支持** - PS5、Xbox 或通用手柄，带有自定义按钮映射和模式切换
- **键盘控制** - WASD 控制
- **速度配置** - 多种速度预设，适用于不同操作模式（初学者、普通和疯狂模式）
- **机器人动作** - 站立/坐下、锁定/解锁、步态选择、灯光、模式转换

### 摄像头和视频

- **多摄像头流** - 动态发现前、后和自定义话题摄像头
- **H.264/H.265 编解码器** - 分辨率缩放、帧率控制、带宽优化
- **浏览器录制** - 从摄像头录制视频并保存到下载文件夹
- **3D 可视化** - 基于 URDF 的机器人模型，带有激光扫描叠加和导航路径

### 系统监控

- **Jetson 统计** - 板型号、JetPack 版本、电源模式、运行时间
- **CPU/GPU 监控** - 每核使用率、频率、内存、热节流
- **功耗跟踪** - 每通道电压、电流和功率，带峰值检测
- **温度和风扇** - CPU/GPU/SOC 温度和风扇速度控制
- **存储和内存** - 磁盘使用警报、RAM/交换监控

### 网络和车队

- **WiFi 控制面板** - 网络扫描、切换和信号监控
- **连接模式** - WiFi、以太网、4G、热点，带延迟跟踪
- **多机器人车队** - 同时连接、全车队命令、状态仪表板
- **诊断** - 节点健康、错误/警告日志、状态机可视化

### 自定义和用户体验

- **亮/暗主题** - 自定义强调色、持久偏好
- **响应式布局** - 移动端、平板和桌面，支持触摸
- **用户配置** - 通过 Supabase Auth 设置头像、显示名称、主题颜色
- **多语言** - 英语和葡萄牙语，带地区格式
- **审计日志** - 10+ 类别的可搜索事件历史，支持 CSV 导出
- **活动分析** - 使用热图和机器人利用率跟踪

## 目录

- [概述](#概述)
- [项目结构](#项目结构)
- [系统要求](#系统要求)
- [安装](#安装)
  - [硬件设置](#1-硬件设置)
  - [Supabase 设置](#2-supabase-设置)
  - [软件设置](#3-软件设置)
- [前端开发](#前端开发)
- [功能](#功能)
- [配置](#配置)
- [自定义机器人](#添加其他机器人支持--自定义机器人)
- [故障排除](#故障排除)
- [贡献](#贡献)
- [许可证](#许可证--引用)

## 概述

BotBrain 由三个主要组件组成：

### 硬件
可 3D 打印的外壳，带有内部支架，设计用于容纳 NVIDIA Jetson 板和两个 Intel RealSense D435i 摄像头。模块化设计允许您将 BotBrain 连接到各种机器人平台，无需定制制造。

### 前端
基于 React 19 和 TypeScript 构建的 Next.js 15 Web 仪表板。它提供实时机器人控制、摄像头流、地图可视化、任务规划、系统监控和车队管理——所有这些都可从网络上的任何浏览器访问。

### 机器人（ROS2 工作空间）
一组 ROS2 Humble 包，用于处理：
- **启动和编排**（`bot_bringup`）- 系统启动和协调
- **定位**（`bot_localization`）- 基于 RTABMap 的 SLAM，用于建图和定位
- **导航**（`bot_navigation`）- Nav2 集成，用于自主移动
- **感知**（`bot_yolo`）- YOLOv8/v11 目标检测
- **机器人驱动** - 用于 Unitree Go2/G1、DirectDrive Tita 和自定义机器人的平台特定包

---

## 项目结构

```
BotBrain/
├── frontend/          # Next.js 15 Web 仪表板（React 19、TypeScript）
├── botbrain_ws/       # ROS 2 Humble 工作空间
│   └── src/
│       ├── bot_bringup/          # 主启动和系统编排
│       ├── bot_custom_interfaces/# 自定义 ROS 2 消息、服务、动作
│       ├── bot_description/      # URDF/XACRO 模型和 robot_state_publisher
│       ├── bot_jetson_stats/     # Jetson 硬件监控
│       ├── bot_localization/     # RTABMap SLAM
│       ├── bot_navigation/       # Nav2 自主导航
│       ├── bot_rosa/             # ROSA AI 自然语言控制
│       ├── bot_state_machine/    # 生命周期和状态管理
│       ├── bot_yolo/             # YOLOv8/v11 目标检测
│       ├── g1_pkg/               # Unitree G1 支持
│       ├── go2_pkg/              # Unitree Go2 支持
│       ├── joystick-bot/         # 游戏手柄接口
│       └── tita_pkg/             # DirectDrive Tita 支持
├── hardware/          # 可 3D 打印外壳（STL/STEP/3MF）
└── docs/              # 文档
```

---

## 系统要求

### 硬件

| 组件 | 要求 |
|-----------|-------------|
| **计算** | NVIDIA Jetson（Nano、Orin Nano 或 AGX 系列） |
| **摄像头** | 2x Intel RealSense D435i |
| **机器人** | ROS2 Humble 机器人或 Unitree Go2 和 Go2-W、Unitree G1、Direct Drive Tita，或[自定义机器人](../../botbrain_ws/README.md#creating-a-custom-robot-package) |
| **网络** | 以太网或 WiFi 连接 |

### 软件

| 组件 | 要求 |
|-----------|-------------|
| **操作系统** | JetPack 6.2（Ubuntu 22.04）推荐 |
| **容器** | Docker & Docker Compose |
| **Node.js** | v20+（仅用于本地前端开发） |

---

## 安装

BotBrain 有两个主要组件：**硬件**（3D 打印外壳和内部组件）和**软件**（前端 Web 应用和 ROS2 工作空间）。

### 1. 硬件设置

3D 打印外壳并组装电子元件。

**主要部件：** 3D 打印机、PLA 耗材、NVIDIA Jetson、2x RealSense D435i、电压转换器。

> **[硬件组装指南](hardware/README_zh-CN.md)** - 关于如何构建 BotBrain 的详细说明
>
> **[完整组装视频](https://youtu.be/xZ5c619bTEQ)** - BotBrain 组装过程的完整分步视频教程

### 2. Supabase 设置

Web 仪表板需要 Supabase 进行身份验证和数据存储。您需要创建自己的免费 Supabase 项目。

> **[Supabase 设置指南](../SUPABASE_SETUP.md)** - 包含数据库架构的完整说明

**快速摘要：**
1. 在 [supabase.com](https://supabase.com) 创建项目
2. 从设置指南运行 SQL 迁移
3. 复制您的 API 密钥用于下一步

### 3. 软件设置

#### 外部依赖

**操作系统：**
- **NVIDIA JetPack 6.2**（推荐）
- 其他 Linux 发行版可能可以工作，但不受官方支持

**Docker & Docker Compose：**

容器化部署所需：

1. 安装 Docker：

```bash
# 添加 Docker 的官方 GPG 密钥：
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# 将仓库添加到 Apt 源：
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# 安装 Docker 包：
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

请参阅[官方 Docker 安装指南](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)了解更多详情。

2. 启用无 sudo 的 Docker：

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

请参阅[安装后步骤](https://docs.docker.com/engine/install/linux-postinstall/)了解更多详情。

#### 安装步骤

**步骤 1：克隆仓库**

```bash
git clone https://github.com/botbotrobotics/BotBrain.git
cd BotBrain
```

**步骤 2：运行安装脚本**

自动安装脚本将配置您的机器人并设置自启动服务：

```bash
sudo ./install.sh
```
有关安装程序中询问信息的更多详情，请参阅[这里](../installation-guide.md)

**步骤 3：重启系统**

```bash
sudo reboot
```

重启后，系统将自动启动所有 ROS2 节点和 Web 服务器的 Docker 容器。

**步骤 4：访问 Web 界面**

| 访问方式 | URL |
|---------------|-----|
| 同一台电脑 | `http://localhost` |
| 网络访问 | `http://<JETSON_IP>` |

查找您的 Jetson IP 地址：
```bash
hostname -I
```

> **注意：** 确保两台设备在同一网络上，且端口 80 未被防火墙阻止。

---

## 前端开发

用于本地前端开发（无需完整机器人栈）：

### 设置

```bash
cd frontend

# 复制环境模板
cp .env.example .env.local

# 使用您的 Supabase 凭据编辑
nano .env.local
```

### 环境变量

| 变量 | 必需 | 描述 |
|----------|----------|-------------|
| `NEXT_PUBLIC_SUPABASE_URL` | 是 | 您的 Supabase 项目 URL |
| `NEXT_PUBLIC_SUPABASE_ANON_KEY` | 是 | 您的 Supabase anon/公共密钥 |
| `NEXT_PUBLIC_ROS_IP` | 否 | 默认机器人 IP（默认：192.168.1.95） |
| `NEXT_PUBLIC_ROS_PORT` | 否 | ROS bridge 端口（默认：9090） |

### 运行

```bash
# 安装依赖
npm install

# 开发服务器（完整功能）
npm run dev

# 开发服务器（开源版）
npm run dev:oss

# 生产构建
npm run build
npm start
```
---

## 配置

### 机器人配置

编辑 `botbrain_ws/robot_config.yaml`：

```yaml
robot_configuration:
  robot_name: "my_robot"           # 所有话题的命名空间
  robot_model: "go2"               # go2、tita、g1 或自定义
  network_interface: "eth0"        # ROS2 的网络接口
  openai_api_key: ""               # 用于 AI 功能（可选）
```

### 摄像头配置

摄像头序列号和变换在每个机器人中配置：
- `botbrain_ws/src/go2_pkg/config/camera_config.yaml`
- `botbrain_ws/src/g1_pkg/config/camera_config.yaml`
- `botbrain_ws/src/tita_pkg/config/camera_config.yaml`

查找您的摄像头序列号：
```bash
rs-enumerate-devices | grep "Serial Number"
```

---

## 添加其他机器人支持 / 自定义机器人

要向 BotBrain 添加新机器人平台支持：

1. **后端/ROS2 栈**：按照完整的[创建自定义机器人包](../../botbrain_ws/README.md#creating-a-custom-robot-package)指南操作
2. **前端**：在 Web 界面设置中添加机器人配置

---

## 故障排除

### WebSocket 连接失败
- 验证 rosbridge 正在运行：`ros2 node list | grep rosbridge`
- 检查防火墙是否允许端口 9090：`sudo ufw allow 9090`
- 确保 UI 中机器人连接设置的 IP 正确

### 摄像头未检测到
- 列出已连接的摄像头：`rs-enumerate-devices`
- 检查 USB 连接并确保摄像头有电
- 验证 `camera_config.yaml` 中的序列号与您的摄像头匹配
- 检查 USB 权限：`sudo usermod -a -G video $USER`

### Docker 问题
- 确保 Docker 无需 sudo 即可运行（参见安装说明）
- 检查 GPU 访问：`docker run --gpus all nvidia/cuda:11.0-base nvidia-smi`
- 查看容器日志：`docker compose logs -f bringup`

### 前端无法加载
- 验证 `.env.local` 中的 Supabase 凭据
- 检查浏览器控制台是否有错误
- 确保已安装 Node.js v20+：`node --version`

### 机器人不移动
- 检查 twist_mux 是否正在运行：`ros2 topic echo /cmd_vel_out`
- 验证机器人硬件接口是否活动：`ros2 lifecycle get /robot_write_node`
- 检查 UI 中的紧急停止是否已启用

### 需要更多帮助？
加入我们的 [Discord 社区](https://discord.gg/CrTbJzxXes)，获得实时支持并与 BotBrain 社区讨论。

---

## 第三方库

请参阅 [docs/DEPENDENCIES.md](../DEPENDENCIES.md) 获取使用的前端和 ROS 包的完整列表。

---

## 贡献

我们欢迎贡献！无论您是修复 bug、添加功能、改进文档还是添加新机器人支持，我们都非常感谢您的帮助。如果您能使 BotBrain 更好或更快，请带来您的贡献。

加入我们的 [Discord 服务器](https://discord.gg/CrTbJzxXes)讨论想法、获得帮助或与其他贡献者协调。

### 开发工作流程

1. **Fork 仓库**
   ```bash
   # 通过 GitHub UI 进行 Fork，然后克隆您的 fork
   git clone https://github.com/botbotrobotics/BotBrain.git
   cd BotBrain
   ```

2. **创建功能分支**
   ```bash
   git checkout -b feature/your-amazing-feature
   ```

3. **进行更改**
   - 为新功能添加测试
   - 更新相关 README 文件
   - 确保所有包成功构建
   - 遵循 ROS 2 编码标准

4. **彻底测试**

5. **提交更改**
   ```bash
   git add .
   git commit -m "Add feature: 更改的简要描述"
   ```

6. **推送到您的 Fork**
   ```bash
   git push origin feature/your-amazing-feature
   ```

7. **提交 Pull Request**
   - 提供更改的清晰描述
   - 引用任何相关问题
   - 包含 UI/行为更改的截图或视频

---

## BotBrain Pro

<p align="center">
  <img src="../images/botbrainpro.png" alt="BotBrain Pro" width="600">
</p>

BotBrain 的专业/企业版本，具有 IP67 防护、自定义载荷如 CamCam（热成像 + 红外摄像头）、ZoomZoom（30 倍长焦 RGB 摄像头）、高级 AI 模型、IoT 集成（LoRA）、3-5G 数据连接、服务和维护、自定义载荷的高级集成等更多功能。[在此了解更多](https://botbot.bot)。

---

## 安全

机器人在操作不当或开发期间可能会伤害人员和自身。请遵守以下安全实践：

- **使用物理急停按钮** - 切勿仅依赖软件停止
- **如果泄露，请轮换 API 密钥**
- **在物理硬件上运行之前，先在仿真中测试更改**
- **初始测试期间远离机器人**

> **免责声明：** BotBot 对使用本软件或硬件造成的任何故障、事故或损坏不承担责任。用户对使用 BotBrain 的机器人的安全操作、测试和部署承担全部责任。

---

## 许可证

本项目根据 **MIT 许可证**授权 - 请参阅 [LICENSE](../LICENSE) 文件了解详情。

---

<p align="center">用 💜 在巴西制作</p>

<p align="right">
  <img src="../images/icon.png" width="110">
</p>
