<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

<p align="center">
  Un Cerebro, cualquier Robot.
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-Sitio_Web-000?logo=vercel&logoColor=white" alt="Sitio Web"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
  <a href="https://huggingface.co/botbot-ai"><img src="https://img.shields.io/badge/-Hugging%20Face-FFD54F?logo=huggingface&logoColor=black" alt="Hugging Face"></a>
</p>

<h1 align="center">BotBrain ROS2 Workspace</h1>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="Licencia: MIT">
  <img src="https://img.shields.io/badge/Platform-Ubuntu_22.04-orange" alt="Ubuntu 22.04">
</p>

<p align="center">
  <a href="../../../botbrain_ws/README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-green" alt="Español"></a>
</p>

> **Nota:** La versión en inglés es la documentación oficial y más actualizada. Esta traducción puede no reflejar los últimos cambios.

## Descripción General

El **BotBrain Workspace** es un framework ROS2 modular y de código abierto para control autónomo de robots, navegación y localización. Diseñado con una arquitectura agnóstica a robots, permite el desarrollo y despliegue rápidos de aplicaciones robóticas avanzadas en múltiples plataformas de robots.

**Características Principales:**
- 🤖 **Soporte Multi-Robot**: Base de código única para Go2, Tita, G1 y robots personalizados
- 🗺️ **SLAM Visual**: Localización basada en RTABMap con soporte de cámara dual
- 🎮 **Múltiples Modos de Control**: Joystick, interfaz web y navegación autónoma
- 👁️ **Visión IA**: Detección de objetos YOLOv8/v11
- 🐳 **Listo para Docker**: Despliegue containerizado con aceleración GPU
- 🔄 **Gestión del Ciclo de Vida**: Orquestación robusta de nodos y recuperación de fallos


## Tabla de Contenidos

- [Requisitos de Hardware](#requisitos-de-hardware)
- [Inicio Rápido](#inicio-rápido)
- [Estructura del Repositorio](#estructura-del-repositorio)
- [Creación de un Paquete de Robot Personalizado](#creación-de-un-paquete-de-robot-personalizado)
- [Descripción General de Paquetes](#descripción-general-de-paquetes)
- [Servicios Docker](#servicios-docker)
- [Configuración](#configuración)

## Requisitos de Hardware

### Plataformas de Robot Soportadas
- **Unitree Go2**
- **Unitree G1**
- **Tita**
- **Robots Personalizados** - Sigue la [Guía de Paquete de Robot Personalizado](#creación-de-un-paquete-de-robot-personalizado)

### Hardware Requerido
- **Plataforma de Robot**: Uno de los robots soportados arriba
- **Computador de Abordo**:
  - Nvidia Jetson Orin Series o más reciente
- **Sensores**:
  - Cámaras Intel RealSense (para SLAM visual)
  - LiDAR (para SLAM basado en LiDAR)
- **Red**:
  - Conexión Ethernet al robot
  - Adaptador Wi-Fi (para control remoto)

### Hardware Opcional
- **Controlador de Juego**: Para teleoperación

## Inicio Rápido

### Iniciar Usando Docker Compose

Para despliegue containerizado:

```bash
# Iniciar todos los servicios
docker compose up -d

# Iniciar servicios específicos
docker compose up -d state_machine bringup localization navigation

# Ver logs
docker compose logs -f bringup

# Detener servicios
docker compose down
```

### Verificar que el Sistema Está Corriendo

```bash
# Verificar nodos activos
ros2 node list

# Verificar topics
ros2 topic list
```

### Contenedor de Desarrollo

Si quieres usar la misma imagen docker para desarrollo, sin crear un nuevo servicio, es posible ejecutar un contenedor de desarrollo interactivo:

```bash
# Iniciar el contenedor de desarrollo
cd botbrain_ws
docker compose up dev -d

# Iniciar un terminal interactivo
docker compose exec dev bash
```

Una vez que el terminal interactivo se abra, puedes usarlo para crear, compilar y ejecutar nuevas características que aún no están integradas con los servicios docker.

## Estructura del Repositorio

```
botbrain_ws/
├── README.md                          # Este archivo
├── LICENSE                            # Licencia MIT
│
├── robot_config.yaml                  # Archivo de configuración principal
├── install.sh                         # Script de instalación automatizada
├── robot_select.sh                    # Ayudante de selección de robot
│
├── docker-compose.yaml                # Definición de servicios Docker
├── botbrain.service                   # Servicio systemd de autoarranque
├── cyclonedds_config.xml              # Configuración del middleware DDS
│
└── src/                               # Paquetes ROS 2
    │
    ├── Paquetes del Sistema Central
    │   ├── bot_bringup/               # Lanzamiento principal & coordinación twist mux
    │   ├── bot_custom_interfaces/     # Mensajes, servicios, acciones personalizados
    │   └── bot_state_machine/         # Gestión de ciclo de vida & estados
    │
    ├── Modelo de Robot & Visualización
    │   └── bot_description/           # Modelos URDF/XACRO & robot_state_publisher
    │
    ├── Navegación & Localización
    │   ├── bot_localization/          # SLAM RTABMap (visual & LiDAR)
    │   └── bot_navigation/            # Stack de navegación Nav2
    │
    ├── Percepción & Control
    │   ├── bot_yolo/                  # Detección de objetos YOLOv8/v11
    │   └── joystick-bot/              # Interfaz de controlador de juego
    │
    ├── IA & Monitoreo
    │   ├── bot_jetson_stats/          # Monitoreo de hardware Jetson
    │   └── bot_rosa/                  # Control por lenguaje natural ROSA AI
    │
    └── Paquetes Específicos de Robot
        ├── g1_pkg/                    # Interfaz de hardware Unitree G1
        ├── go2_pkg/                   # Interfaz de hardware Unitree Go2
        ├── tita_pkg/                  # Interfaz de hardware Tita
        └── your_robot_pkg/            # Tu robot personalizado (ver guía abajo)
```

## Creación de un Paquete de Robot Personalizado

Para agregar soporte para una nueva plataforma de robot, sigue esta guía usando [go2_pkg](../../../botbrain_ws/src/go2_pkg) como plantilla de referencia.

**Nota**: El paquete go2_pkg se comunica con el robot Unitree Go2 vía topics ROS 2 (suscribiéndose a los topics nativos ROS 2 de Unitree y republicando en formato BotBrain). Tu paquete de robot personalizado puede usar comunicación similar basada en topics, APIs de hardware directas o interfaces SDK dependiendo de la arquitectura de tu robot. La idea es crear una interfaz de paquete estándar entre los paquetes botbrain_ws y el robot.

### Estructura de Paquete Requerida

Tu paquete de robot personalizado debe seguir esta convención de nomenclatura para funcionar perfectamente con todos los paquetes: `{robot_model}_pkg`

```
{robot_model}_pkg/
├── package.xml                        # Manifiesto del paquete ROS 2
├── CMakeLists.txt                     # Configuración de build
├── README.md                          # Documentación del paquete
├── launch/
│   └── robot_interface.launch.py     # REQUERIDO: Launcher principal de la interfaz de hardware
├── config/
│   └── nav2_params.yaml               # REQUERIDO: Parámetros de navegación
├── scripts/
│   ├── {robot_model}_read.py                # REQUERIDO: Lee datos de sensores del robot
│   └── {robot_model}_write.py               # REQUERIDO: Envía comandos al robot
├── {robot_model}_pkg/                 # Directorio del paquete Python
│   └── tools/                         # OPCIONAL: Herramientas del asistente ROSA AI
│       ├── __init__.py                # Inicialización vacía del paquete
│       └── {robot_model}.py           # Herramientas LangChain para integración ROSA
├── xacro/
│   └── robot.xacro                    # REQUERIDO: Modelo URDF del robot
└── meshes/
    └── *.dae, *.stl                   # Mallas visuales & de colisión
```

### Guía de Creación Paso a Paso

#### 1. Crear Nuevo Paquete ROS 2

Crea la estructura del paquete usando herramientas ROS 2 (si no tienes ros2 instalado en tu sistema host, esto puede hacerse desde un contenedor de desarrollo):

```bash
cd src/
ros2 pkg create {robot_model}_pkg --build-type ament_cmake --dependencies rclcpp rclpy
cd {robot_model}_pkg
```

Crea los directorios requeridos:
```bash
mkdir -p launch config scripts xacro meshes maps
```

#### 2. Configurar package.xml

Edita `package.xml` y agrega las dependencias requeridas:
- Agrega `bot_custom_interfaces` como dependencia
- Actualiza nombre del paquete, versión, descripción e información del mantenedor
- Asegúrate de que todas las dependencias de mensajes de sensores estén incluidas

#### 3. Configurar CMakeLists.txt

Actualiza la configuración de build para instalar todos los recursos del paquete:
- Instala el directorio de archivos launch
- Instala el directorio de archivos config
- Instala scripts como ejecutables
- Instala directorios xacro, urdf y meshes
- Usa `ament_python_install_package()` para módulos Python

#### 4. Crear Archivo Launch de la Interfaz de Hardware

**CRÍTICO**: Crea `launch/robot_interface.launch.py` (nombre exacto requerido)

Este archivo launch debe:
- Leer `robot_config.yaml` desde la raíz del workspace
- Extraer `robot_name` para configuración de namespace
- Lanzar nodos de ciclo de vida para lectura y escritura de hardware
- Usar `LifecycleNode` de `launch_ros.actions`
- Aplicar namespace correcto a todos los nodos

Referencia: Ver [go2_pkg/launch/robot_interface.launch.py](../../../botbrain_ws/src/go2_pkg/launch/robot_interface.launch.py) para ejemplo completo.

#### 5. Implementar Nodos de Interfaz de Hardware

**Crea `scripts/{robot_model}_read.py`** - Lee datos de sensores y publica en ROS 2:

Este nodo de ciclo de vida debe:
- Inicializarse como `LifecycleNode` con nombre `robot_read_node`
- Implementar callbacks de ciclo de vida: `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
- En `on_configure`: Crear publishers para odometría, IMU, estados de articulaciones y batería
- En `on_activate`: Iniciar loop de lectura de datos (típicamente 50Hz) del hardware/topics del robot
- Procesar datos de sensores del robot y publicar en topics ROS 2
- En `on_deactivate`: Detener publicación de datos pero mantener conexiones
- En `on_cleanup`: Cerrar conexiones de hardware y liberar recursos

Referencia: Ver [go2_pkg/scripts/go2_read.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_read.py) para implementación completa.

**Crea `scripts/{robot_model}_write.py`** - Recibe comandos y envía al robot:

Este nodo de ciclo de vida debe:
- Inicializarse como `LifecycleNode` con nombre `robot_write_node`
- En `on_configure`: Crear subscriber para topic `cmd_vel_out` y establecer comunicación con el robot
- Implementar callback para recibir comandos de velocidad y reenviar al hardware del robot
- En `on_deactivate`: Enviar comando de parada (velocidad cero) al robot
- En `on_cleanup`: Cerrar conexiones de hardware y liberar recursos
- Opcionalmente: Implementar servicios específicos del robot (cambio de modo, control de marcha, etc.)

Referencia: Ver [go2_pkg/scripts/go2_write.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_write.py) para implementación completa.

#### 6. Crear Parámetros de Navegación

Crea `config/nav2_params.yaml` con las especificaciones de tu robot. Ver [Documentación Nav2](https://docs.nav2.org/) como referencia.

Necesitarás agregar un comodín a las secciones de configuración de nodos. Ver [go2_pkg/config/nav2_params.yaml](../../../botbrain_ws/src/go2_pkg/config/nav2_params.yaml) para ejemplo de configuración completa.

#### 7. Crear Descripción del Robot (XACRO)

Crea `xacro/robot.xacro` con el modelo URDF de tu robot:

Tu archivo XACRO debe definir:
- `base_link` como el link principal del cuerpo del robot
- `interface_link` como la parte de interfaz entre robot y BotBrain
- Todas las articulaciones y links del robot (piernas, brazos, etc.)
- Links de sensores (cámaras, LiDAR, IMU)
- Mallas visuales para visualización RViz
- Mallas de colisión para navegación
- Límites de articulaciones y dinámica
- Propiedades inerciales

Referencia: Ver [go2_pkg/xacro/robot.xacro](../../../botbrain_ws/src/go2_pkg/xacro/robot.xacro) para descripción completa del robot.

#### 8. Configurar Workspace

Actualiza el `robot_config.yaml` del workspace (puede hacerse desde install.sh):

```yaml
robot_configuration:
  robot_name: "mi_robot"               # Namespace para todos los topics
  robot_model: "tu_robot"              # Debe coincidir con el nombre del paquete sin "_pkg"
  description_file_type: "xacro"       # "xacro" o "urdf"
  network_interface: "eth0"            # Interfaz de red para comunicación del robot
```

**IMPORTANTE**: El campo `robot_model` debe coincidir con el nombre de tu paquete **sin** el sufijo `_pkg`:
- Nombre del paquete: `tu_robot_pkg`
- robot_model: `tu_robot`

#### 9. Compilar y Probar

```bash
# Compila tu paquete
cd ~/botbrain_workspace/BotBrain/botbrain_ws
colcon build --packages-select tu_robot_pkg

# Source el workspace
source install/setup.bash

# Prueba tu interfaz de hardware
ros2 launch tu_robot_pkg robot_interface.launch.py

# Lanza con sistema completo
ros2 launch bot_bringup bringup.launch.py
```

Puedes compilar y probar el nuevo paquete usando un contenedor de desarrollo.

#### 10. Crear Herramientas ROSA (Opcional)

**ROSA** (Robot Operating System Assistant) es un asistente de IA que permite el control por lenguaje natural de tu robot. Al crear herramientas para ROSA, los usuarios pueden interactuar con tu robot usando comandos conversacionales.

**Crea la estructura del directorio de herramientas:**

```bash
mkdir -p {robot_model}_pkg/tools
touch {robot_model}_pkg/tools/__init__.py
touch {robot_model}_pkg/tools/{robot_model}.py
```

### Puntos de Integración del Paquete

El sistema BotBrain encontrará y usará automáticamente tu paquete basándose en estas convenciones:

1. **Nomenclatura del Paquete**: formato `{robot_model}_pkg`
2. **Archivo Launch**: `launch/robot_interface.launch.py` (nombre exacto requerido)
3. **Config de Navegación**: `config/nav2_params.yaml` (usado por bot_navigation)
4. **Archivos de Descripción**: `xacro/robot.xacro` o `urdf/robot.urdf` (usado por bot_description)

### Topics Requeridos que Tu Paquete Debe Proporcionar

Para integración completa del sistema, tu interfaz de hardware debe publicar:

| Topic | Tipo de Mensaje | Descripción | Frecuencia |
|-------|--------------|-------------|-----------|
| `/{namespace}/odom` | nav_msgs/Odometry | Odometría del robot | 50Hz |
| `/{namespace}/imu` | sensor_msgs/Imu | Datos IMU | 100Hz |
| `/{namespace}/joint_states` | sensor_msgs/JointState | Posiciones/velocidades de articulaciones | 50Hz |

Y suscribirse a:

| Topic | Tipo de Mensaje | Descripción |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | geometry_msgs/Twist | Comandos de velocidad desde twist_mux |


## Descripción General de Paquetes

### Paquetes del Sistema Central

| Paquete | Descripción | Documentación |
|---------|-------------|---------------|
| [bot_bringup](../../../botbrain_ws/src/bot_bringup) | Coordinación principal de lanzamiento, multiplexor twist y orquestación del sistema | [README](../../../botbrain_ws/src/bot_bringup/README.md) |
| [bot_state_machine](../../../botbrain_ws/src/bot_state_machine) | Gestión del ciclo de vida, coordinación de nodos y control de estado del sistema | [README](../../../botbrain_ws/src/bot_state_machine/README.md) |
| [bot_custom_interfaces](../../../botbrain_ws/src/bot_custom_interfaces) | Mensajes, servicios y acciones ROS 2 personalizados | [README](../../../botbrain_ws/src/bot_custom_interfaces/README.md) |
| [bot_description](../../../botbrain_ws/src/bot_description) | Modelos URDF/XACRO del robot y robot_state_publisher | [README](../../../botbrain_ws/src/bot_description/README.md) |

### Navegación & Localización

| Paquete | Descripción | Documentación |
|---------|-------------|---------------|
| [bot_localization](../../../botbrain_ws/src/bot_localization) | SLAM RTABMap con soporte para mapeo visual y basado en LiDAR | [README](../../../botbrain_ws/src/bot_localization/README.md) |
| [bot_navigation](../../../botbrain_ws/src/bot_navigation) | Stack de navegación Nav2 con configuración agnóstica a robots | [README](../../../botbrain_ws/src/bot_navigation/README.md) |

### Percepción & Control

| Paquete | Descripción | Documentación |
|---------|-------------|---------------|
| [bot_yolo](../../../botbrain_ws/src/bot_yolo) | Detección de objetos YOLOv8/v11 con aceleración TensorRT | [README](../../../botbrain_ws/src/bot_yolo/README.md) |
| [joystick-bot](../../../botbrain_ws/src/joystick-bot) | Interfaz de controlador de juego con interruptor de seguridad dead-man | [README](../../../botbrain_ws/src/joystick-bot/README.md) |

### Paquetes Específicos de Robot

| Paquete | Descripción | Documentación |
|---------|-------------|---------------|
| [go2_pkg](../../../botbrain_ws/src/go2_pkg) | Interfaz de hardware y descripción del Unitree Go2 cuadrúpedo | [README](../../../botbrain_ws/src/go2_pkg/README.md) |
| [tita_pkg](../../../botbrain_ws/src/tita_pkg) | Interfaz de hardware y descripción del Tita cuadrúpedo | [README](../../../botbrain_ws/src/tita_pkg/README.md) |

## Servicios Docker

El workspace incluye múltiples servicios Docker para despliegue containerizado:

| Servicio | Descripción | Auto-arranque | Dependencias |
|---------|-------------|------------|--------------|
| `dev` | Contenedor de desarrollo (interactivo) | No | - |
| `builder_base` | Compila todos los paquetes del workspace | No | - |
| `state_machine` | Servicio de gestión del ciclo de vida | Sí | - |
| `bringup` | Bringup principal del robot | Sí | state_machine |
| `localization` | Localización RTABMap | Sí | bringup |
| `navigation` | Servidores de navegación Nav2 | Sí | localization |
| `rosa` | Servicios de llamada de herramientas IA | Sí | bringup |
| `yolo` | Servicio de detección de objetos | Sí | bringup |

### Uso de Docker

```bash
# Iniciar todos los servicios
docker compose up -d

# Iniciar servicio específico con dependencias
docker compose up -d navigation  # Automáticamente inicia bringup, localization

# Ver logs
docker compose logs -f bringup

# Detener todos los servicios
docker compose down

# Recompilar después de cambios de código
docker compose build
docker compose up -d
```

## Configuración

### Archivo de Configuración Principal

El archivo [robot_config.yaml](../../../botbrain_ws/robot_config.yaml) es el punto central de configuración:

```yaml
robot_configuration:

  # Identificador del robot - usado como namespace para todos los topics
  robot_name: ""                    # Ejemplo: "go2_robot1", "tita_lab"

  # Tipo de robot - determina qué paquete de hardware lanzar
  robot_model: "go2"                # Opciones: "go2", "tita", "tu_robot"

  # Formato del archivo de descripción
  description_file_type: "xacro"    # Opciones: "xacro", "urdf"

  # Interfaz de red para comunicación ROS2
  network_interface: "eno1"         # Ejemplo: "eth0", "wlan0", "eno1"

  # Específico de Tita: namespace para comunicación del robot Tita
  tita_namespace: "tita3036731"     # Solo usado cuando robot_model: "tita"

  # Clave API OpenAI para características de IA (opcional)
  openai_api_key: ""                # Requerido para asistente ROSA AI

  # Configuración Wi-Fi (opcional)
  wifi_interface: ""                # Nombre de la interfaz Wi-Fi (ej.: "wlan0")
  wifi_ssid: ""                     # SSID de la red Wi-Fi
  wifi_password: ""                 # Contraseña de la red Wi-Fi
```

### Configuración de Red

El workspace usa CycloneDDS para comunicación ROS 2. Configuración en [cyclonedds_config.xml](../../../botbrain_ws/cyclonedds_config.xml):

Configura la interfaz de red en [robot_config.yaml](../../../botbrain_ws/robot_config.yaml) para que coincida con tu conexión de hardware.

### Servicio Systemd de Autoarranque

El archivo [botbrain.service](../../../botbrain_ws/botbrain.service) habilita el arranque automático al inicio:

```bash
# Instalar servicio (hecho por install.sh)
sudo cp botbrain.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable botbrain.service

# Control manual
sudo systemctl start botbrain.service   # Iniciar ahora
sudo systemctl stop botbrain.service    # Detener
sudo systemctl status botbrain.service  # Verificar estado

# Ver logs
journalctl -u botbrain.service -f
```

### Agregando Soporte para Nuevos Robots

Ver la guía [Creación de un Paquete de Robot Personalizado](#creación-de-un-paquete-de-robot-personalizado) arriba. ¡Especialmente damos la bienvenida a contribuciones que agreguen soporte para nuevas plataformas de robots!

<p align="center">Hecho con ❤️ en Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Ícono Bot" width="110">
</p>
