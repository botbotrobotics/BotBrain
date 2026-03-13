# Go2 Local Setup Guide

A step-by-step guide for setting up BotBrain on the Unitree Go2 EDU Docking Station.

---

## Table of Contents

- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
  - [Step 1 — Internet Connectivity](#step-1--internet-connectivity)
  - [Step 2 — Clone the BotBrain Repository](#step-2--clone-the-botbrain-repository)
  - [Step 3 — Boot Procedure](#step-3--boot-procedure)

---

## Hardware Setup

### Docking Station

Mount the docking station as provided in Unitree's official instructions.

### RealSense Camera

Mount the RealSense camera according to the image below:

<img src="./images/mechanics/go2_local_mount_01.png" width="600">

> **Important:** By default, the mounting bracket is screwed into the bottom hole of the mount. Move it to the top hole as shown in the image above. Also ensure the RealSense is flush against the robot's head. This placement matches the transform offsets used in the software configuration.

---
## Software Setup

### Step 1 — Internet Connectivity

The docking station's Jetson that comes with Go2W and Go2Edu versions does not come with a WiFi card. Installing and utilizing BotBrain requires an internet connection, so a solution is provided in this guide.

### Recommended: WiFi Dongle

Purchase a **ALFA AWUS036NHA** (Atheros AR9271 chipset). The driver for this chipset is natively supported on the docking station, making it plug-and-play — no additional configuration required.

### Alternative Options

> **Note:** The following alternatives are recommended for experienced developers only.

- **Ethernet cable** — Connect the Jetson directly via cable. This works out of the box but may require additional network management configuration on the Jetson to route traffic correctly.

- **Different WiFi dongle** — Other dongles may work, but their drivers may not be natively available on the docking station. In that case, a first internet connection via cable will be necessary to download and install the corresponding driver.

---

### Step 2 — Clone the BotBrain Repository

Clone the BotBrain repository using the `feature/go2_local` branch:

```bash
git clone -b feature/go2_local https://github.com/botbotrobotics/BotBrain.git
```

Then follow the standard BotBrain installation steps as described in [installation-guide.md](./installation-guide.md).

> **Why this branch?**
>
> The `feature/go2_local` branch contains two changes specific to this hardware configuration:
>
> - **Camera transform offsets** — On the docking station, the RealSense camera is mounted on the robot's head rather than inside the BotBrain enclosure. The position transform parameters in the camera config have been adjusted to match this physical placement.
>
> - **RTABMap `initial_reset` patch** — JetPack 5.x (present on the docking station) does not support launching RTABMap with `initial_reset: True`, which is used in every other BotBrain robot configuration to initialize the RealSense properly. This branch removes that parameter for the Go2 local setup.

---

### Step 3 — Boot Procedure

> **Important:** This procedure must be repeated every time the robot boots.

This is required as a workaround for the unsupported `initial_reset` behavior on JetPack 5.x.

1. Open BotBrain and navigate to the **Health** tab.
2. Wait for all nodes to activate properly.
2. Find the **Front Camera** node and restart it.
3. Wait until the **Front Camera** node status shows **active**.
4. Find the **Controller Server (Nav2)** node and restart it.

> **Note:** Restarting the Controller Server node will cascade-restart several nodes below it. This is expected behavior. If the following nodes are not restarted automatically, you can do that manually.
