# BotBrain Hardware Assembly Manual

3D printable enclosures and robot interface parts for BotBrain. This guide covers printing, bill of materials, and step-by-step assembly instructions.

<p align="center">
  <img src="../docs/images/assembly.gif" alt="BotBrain Assembly" width="600">
</p>

<p align="center">
  <a href="README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="../docs/i18n/hardware/README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="../docs/i18n/hardware/README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="../docs/i18n/hardware/README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="../docs/i18n/hardware/README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

---

## Table of Contents

- [Overview](#overview)
- [General Requirements](#general-requirements)
- [Print Settings](#print-settings)
- [BotBrain Assembly](#botbrain-assembly)
- [Robot Interface Assemblies](#robot-interface-assemblies)
  - [Unitree G1 Interface](#unitree-g1-interface)
  - [Unitree Go2 Interface](#unitree-go2-interface)
  - [Direct Drive Tita Interface](#tita-interface)

---

## Overview

The BotBrain hardware consists of two main components:

1. **BotBrain** - The main enclosure housing all electronics
2. **Robot Interface** - A mounting adapter specific to your robot platform

You will need to assemble the BotBrain first, then attach it to the appropriate robot interface for your platform.

### Directory Structure

```
hardware/
├── BotBrain/          # Core enclosure files
├── G1/                # Unitree G1 interface
├── Go2/               # Unitree Go2 interface
└── Tita/              # Direct Drive Tita interface
```

### File Formats

| Format | Use Case |
|--------|----------|
| **.3mf** | Recommended for most slicers (PrusaSlicer, Bambu Studio, Cura) |
| **.stl** | Universal format, works with any slicer |
| **.step** | CAD format for modifications |

---

## General Requirements

### Tools Required

| Tool | Purpose |
|------|---------|
| 3D Printer | Printing enclosure and interface parts |
| Screwdriver set (Phillips/Hex) | Fastening components |
| Wire strippers | Cable preparation |
| Tweezers | Handling small components |

### Safety Precautions

> **Warning:** Always disconnect power before assembly or disassembly.

- Handle electronics carefully to avoid static discharge (use ESD strap if available)
- Ensure proper ventilation when soldering
- Wear safety glasses when removing supports from prints
- Double-check polarity before connecting power

---

## Print Settings

Use these settings for all BotBrain hardware parts:

| Setting | Recommended Value | Notes |
|---------|-------------------|-------|
| Material | PLA | PETG also acceptable for higher temp environments |
| Layer Height | 0.2 mm | Use 0.1 mm for finer detail |
| Infill | 20-30% | Higher infill for structural parts |
| Supports | Tree supports | - |
| Bed Adhesion | Brim (optional) | Helps prevent warping |

---

## BotBrain Assembly

The BotBrain Core is the main enclosure that houses all the electronics. Complete this assembly before attaching any robot interface.

[Complete Assembly Video](https://youtu.be/xZ5c619bTEQ) - Full step-by-step video walkthrough of the BotBrain assembly process

### Bill of Materials - BotBrain

#### 3D Printed Parts

| Part | Quantity | File | Notes |
|------|----------|------|-------|
| Top Case | 1 | [BotBrain/top_case.stl](BotBrain/top_case.stl) | Main cover |
| Bottom Case | 1 | [BotBrain/bottom_case.stl](BotBrain/bottom_case.stl) | Component housing |

#### Electronics

| Component | Quantity | Notes |
|-----------|----------|-------|
| Jetson Orin Nano | 1 | Without Base |
| USB-A/USB-C Cable | 2 | 15cm length for better fit |
| RealSense Camera | 2 | D435i |
| 12V DC-DC Converter | 1 | For D435i cameras |
| WAGO Connector | 2 | 2-way |
| Barrel Jack Pigtail | 1 | - |

#### Fasteners & Hardware

| Item | Quantity | Notes |
|------|----------|-------|
| M3x10 Thread Forming | 4 | Preferably Hex/Allen |
| M3 Flat Washer  | 4 | - |

### Assembly Steps - BotBrain Core

#### Step 1: Print the Enclosure Parts

Print both the top and bottom cases using the [print settings](#print-settings) above.

---

#### Step 2: Prepare the Bottom Case

Remove all support material and clean up any rough edges.

**Tasks:**
1. Remove all support material with flush cutters
2. Sand any rough edges or bumps
3. Verify all mounting points are clear

---

#### Step 3: Install Electronics

**Tasks:**

1. Place the screws in the mounting bosses on the bottom case
2. Connect the WAGO connectors to both the input and output terminals of the 12V DC-DC converter
3. Mount the 12V converter in the designated location on the bottom case
4. Connect the barrel jack pigtail to the DC converter output WAGO connector
5. Place the RealSense cameras in their mounting positions with USB cables pre-connected
6. Place the Jetson Orin Nano in the correct position, routing the WiFi/Bluetooth antennas into the side pocket
7. Connect the USB cables from the RealSense cameras to the Jetson Orin Nano 
8. Connect the power cable (barrel jack) to the Jetson Orin Nano power input

---

#### Step 4: Close the Enclosure

Attach the top case to complete the BotBrain assembly.

**Tasks:**
1. Align the top case with the bottom case
2. Gently press down until the snaps engage

> **Opening the case:** To reopen, gently flex the sides of the bottom section to release the mounting snaps.

---

## Robot Interface Assemblies

Choose the interface assembly guide for your specific robot platform.

---

### Unitree G1 Interface

Mounting interface for the Unitree G1 humanoid robot.

#### Bill of Materials - G1 Interface

##### 3D Printed Parts

| Part | Quantity | File |
|------|----------|------|
| G1 Interface Mount | 1 | [G1/g1_interface.stl](G1/g1_interface.stl) |


##### Fasteners & Hardware

| Item | Quantity | Size/Type | Notes |
|------|----------|-----------|-------|
| M6x30 | 4 | Preferably Hex/Allen | 
| M6 Spring Lock Washer | 4 | - |

##### Additional Components

| Item | Quantity | Notes |
|------|----------|-------|
| Ethernet Cable | 1 | - |
| XT-30 Pigtail | 1 | - |

#### Assembly Steps - G1 Interface

> ***Note:** Easier to assemble with the robot laying down, with its back upwards* 

##### Step 1: Print the Interface

Print the G1 interface mount using the [print settings](#print-settings).

---

##### Step 2: Prepare the Interface

**Tasks:**
1. Remove all support material
2. Sand any rough edges or bumps
3. Verify all mounting points are clear

---

##### Step 3: Mount to Robot

**Tasks:**
1. Tighten BotBrain to the inteface with 4 thread forming M3 screws
  ![g1_mount_01](../docs/images/mechanics/g1_mount_01.png)

2. Remove the sticker protection of the back mounting holes
3. Connect the ethernet and power cables to the robot
4. Pass the cables through the panel opening
  ![g1_mount_04](../docs/images/mechanics/g1_mount_04.png)

5. Place the interface on the robot's back, aligning the screw holes

6. Tighten the inteface on the robot using M6x30 screws
  ![g1_mount_06](../docs/images/mechanics/g1_mount_06.png)

---

##### Step 4: Attach BotBrain

**Tasks:**
1. Connect the power and ethernet cables to the BotBrain
2. Position the BotBrain using the alignment pins and tighten the screws

---

### Unitree Go2 Interface

Mounting interface for the Unitree Go2 quadruped robot.

#### Bill of Materials - Go2 Interface

##### 3D Printed Parts

| Part | Quantity | File | Notes |
|------|----------|------|-------|
| Go2 Interface Mount | 1 | [Go2/go2_interface.stl](Go2/go2_interface.stl) | Main mounting bracket |

##### Fasteners & Hardware

| Item | Quantity | Notes |
|------|----------|-------|
| M3x30 | 2 | Preferably Hex/Allen | 
| M3x20 | 2 | Preferably Hex/Allen | 
| M3 Spring Lock Washer | 4 | - |

##### Additional Components

| Item | Quantity | Notes |
|------|----------|-------|
| Ethernet Cable | 1 | - |
| XT-30 Pigtail | 1 | - |
|

#### Assembly Steps - Go2 Interface

##### Step 1: Print the Interface

Print the Go2 interface mount using the [print settings](#print-settings).

> The interface for the Unitree Go2 can also be found on [Makerworld](https://makerworld.com/en/models/2349666-go2_interface-botbrain-open-source-bboss#profileId-2569625)

---

##### Step 2: Prepare the Interface

**Tasks:**
1. Remove all support material
2. Sand any rough edges or bumps
3. Verify all mounting points are clear

---

##### Step 3: Mount to Robot

**Tasks:**
1. Unscrew the top lid screws
  ![go2_mount_01](../docs/images/mechanics/go2_mount_01.png)

2. Connect the ethernet and power cables to the robot
  ![go2_mount_02](../docs/images/mechanics/go2_mount_02.png)

3. Pass the cables through the interface opening
  ![go2_mount_03](../docs/images/mechanics/go2_mount_03.png)

4. Place the interface on the robot back and tighten the M3x30 bolts on the front flange and the M3x20 bolt on the rear flange
  ![go2_mount_04](../docs/images/mechanics/go2_mount_04.png)

5. Tighten the BotBrain to the inteface with 4 thread forming M3 screws
  ![go2_mount_05](../docs/images/mechanics/go2_mount_05.png)

---

##### Step 4: Attach BotBrain

**Tasks:**
1. Connect the power and ethernet cables to the BotBrain
2. Position the BotBrain using the alignment pins and tighten the screws

---

### Tita Interface

Mounting interface for the Legged Robotics Tita robot.

#### Bill of Materials - Tita Interface

##### 3D Printed Parts

| Part | Quantity | File | Notes |
|------|----------|------|-------|
| Tita Interface Mount | 1 | [Tita/tita_interface2.stl](Tita/tita_interface.stl) | Main mounting bracket |

##### Fasteners & Hardware

| Item | Quantity | Notes |
|------|----------|-------|
| M4x35 | 1 | Preferably Hex/Allen | 
| M4 flat washer | 1 | - |

##### Additional Components

| Item | Quantity | Notes |
|------|----------|-------|
| DB25 to Ethernet/Power Cable | 1 | Custom made |

> ***Note**: The power and communication cable for the Tita robot uses a DB25 connector on Tita's side. This cable can be made/soldered using this [Tita electrical schematic](../docs/tita_conn_sch.pdf) as reference.*  

#### Assembly Steps - Tita Interface

##### Step 1: Print the Interface

Print the Tita interface mount using the [print settings](#print-settings).

---

##### Step 2: Prepare the Interface

**Tasks:**
1. Remove all support material
2. Sand any rough edges

---

##### Step 3: Mount to Robot

**Tasks:**

1. Slide the interface onto Tita's rails until the interface screw hole is aligned with Tita's mounting point
![tita_mount_01](../docs/images/mechanics/tita_mount_01.png)

2. Pass the cable with the power and ethernet connectors through the front opening of the interface
![tita_mount_02](../docs/images/mechanics/tita_mount_02.png)

3. Connect the Ethernet cable to the Botbrain and tighten 4 thread forming M3x10 screws to ensure the component's stability
![tita_mount_03](../docs/images/mechanics/tita_mount_03.png)

---

##### Step 4: Attach BotBrain

**Tasks:**
1. Connect the power and ethernet cables to the BotBrain
2. Position the BotBrain using the alignment pins and secure all of the screws

---
