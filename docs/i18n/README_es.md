<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="../images/Botbrainlogo.png" alt="BotBot" width="400">
  </a>
</p>

<p align="center">
  Un Cerebro, cualquier Robot.
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Next.js-15-black?logo=next.js" alt="Next.js 15">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="Licencia MIT">
  <img src="https://img.shields.io/badge/Platform-Jetson-76B900?logo=nvidia" alt="Jetson">
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-Sitio_Web-000?logo=vercel&logoColor=white" alt="Sitio Web"></a>
  <a href="https://discord.gg/CrTbJzxXes"><img src="https://img.shields.io/badge/-Discord-5865F2?logo=discord&logoColor=white" alt="Discord"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
</p>

<p align="center">
  <a href="../../README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-green" alt="Español"></a>
</p>

> **Nota:** La versión en inglés es la documentación oficial y más actualizada. Esta traducción puede no reflejar los últimos cambios.

# BotBrain Open Source (BBOSS) <img src="../images/bot_eyes.png" alt="🤖" width="50" style="vertical-align: middle;">

BotBrain es una colección modular de componentes de software y hardware de código abierto que te permite conducir, ver, mapear, navegar (manualmente o de forma autónoma), monitorear y gestionar robots con patas (cuadrúpedos, bípedos y humanoides) o robots con ruedas ROS2 desde una interfaz web simple pero potente. El hardware proporciona soportes imprimibles en 3D y una carcasa externa para que puedas instalar BotBrain en tu robot sin complicaciones.

- Diseñado alrededor del Intel RealSense D435i y la línea NVIDIA Jetson
- Placas oficialmente soportadas: Jetson Nano, Jetson Orin Nano (soporte para AGX y Thor próximamente)
- Todo es modular - no necesitas ejecutar todos los módulos (algunos módulos pesados de IA requieren Orin AGX)

<h2 align="center">✨ Características Destacadas</h2>

<table>
  <tr>
    <td align="center" width="50%">
      <img src="../images/gifs/Dash:Fleet.gif" alt="Panel de Control y Gestión de Flota" width="400"><br>
      <h3>Panel de Control y Gestión de Flota</h3>
      <p>Panel completo para ver estado, información del robot y acceder rápidamente a otras secciones</p>
    </td>
    <td align="center" width="50%">
      <img src="../images/gifs/Cockpitscreenstudio.gif" alt="CockPit" width="400"><br>
      <h3>CockPit</h3>
      <p>Página de control predefinida con cámaras frontal/trasera completas, modelo 3D, mapa y navegación, además de controles rápidos</p>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="../images/gifs/MyUI.gif" alt="Mi Interfaz" width="400"><br>
      <h3>Mi Interfaz</h3>
      <p>Interfaz de control personalizable con todas las características del cockpit</p>
    </td>
    <td align="center" width="50%">
      <img src="../images/gifs/Missions.gif" alt="Misiones" width="400"><br>
      <h3>Misiones</h3>
      <p>Crea misiones para que el robot ejecute y navegue de forma autónoma</p>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="../images/gifs/Health.gif" alt="Salud del Sistema" width="400"><br>
      <h3>Salud del Sistema</h3>
      <p>Visualiza la salud completa de BotBrain: uso de CPU/GPU/RAM, control y estado de nodos de la máquina de estados, control de conexión WiFi</p>
    </td>
    <td align="center" width="50%">
      <img src="../images/gifs/Profile.gif" alt="Perfil de Usuario" width="400"><br>
      <h3>Perfil de Usuario</h3>
      <p>Personaliza la apariencia de BotBrain, configura colores personalizados y perfiles de velocidad</p>
    </td>
  </tr>
</table>

<p align="center">
  <img src="../images/assembly.gif" alt="Ensamblaje de BotBrain" width="600"><br>
  <h3 align="center">Hardware de Código Abierto</h3>
  <p>Rápido de imprimir en 3D, fácil de ensamblar y diseñado para encajar en cualquier robot.
  Pon tu robot funcionando con BotBrain en menos de 30 minutos.</p>
</p>

<p align="center">
  <a href="https://youtu.be/VBv4Y7lat8Y">📹 Mira a BotBrain completar 1 hora de patrullas autónomas en nuestra oficina</a>
</p>


## Lista Completa de Características

### Soporte Multi-Robot

- **Unitree Go2 & Go2-W** - Robots cuadrúpedos con interfaz de hardware y control completos
- **Unitree G1** - Humanoide con control de postura del cuerpo superior y transiciones FSM
- **DirectDrive Tita** - Bípedo con control completo
- **Robots personalizados** - Framework extensible para agregar cualquier plataforma compatible con ROS2
- **Con patas y ruedas** - La arquitectura soporta ambos tipos de locomoción

### Hardware y Sensores

- **Carcasa imprimible en 3D** - Diseño de encaje con adaptadores de montaje específicos para cada robot (Go2, G1 y Direct drive Tita)
- **Intel RealSense D435i** - Soporte de doble cámara para visualización y SLAM/Navegación
- **IMU y odometría** - Estimación de pose en tiempo real de todas las plataformas soportadas
- **Monitoreo de batería** - Estado de batería por robot con estimación de autonomía

### IA y Percepción

- **Detección de objetos YOLOv8/v11** - Más de 80 clases, optimizado con TensorRT, seguimiento en tiempo real en BotBrain
- **Moondream AI** - Comprensión de visión multimodal y análisis de escena
- **Control por lenguaje natural ROSA** - Comandos conversacionales para el robot vía LLM
- **Historial de detecciones** - Registro buscable con imagen e información/descripción

### Navegación Autónoma

- **RTABMap SLAM** - Mapeo visual con una o dos cámaras RealSense D435i
- **Integración Nav2** - Planificación de trayectorias, evasión de obstáculos dinámicos, comportamientos de recuperación
- **Planificación de misiones** - Crea y ejecuta patrullas autónomas con múltiples waypoints
- **Navegación por clic** - Define destinos directamente en la interfaz del mapa
- **Gestión de mapas** - Guarda, carga, cambia y define posiciones iniciales

### Orquestación del Sistema

- **Gestión del ciclo de vida** - Inicio/apagado coordinado de nodos con ordenamiento de dependencias
- **Máquina de estados** - Estados del sistema con encendido/apagado automático
- **Control de velocidad por prioridad** - Arbitraje de comandos en 6 niveles (joystick > nav > IA)
- **Interruptor de seguridad** - Bloqueo de seguridad de hardware/software para todos los comandos de movimiento
- **Parada de emergencia** - Secuencia completa de e-stop

### Interfaces de Control

- **CockPit** - Página de control preconfigurada con cámaras, modelo 3D, mapa y acciones rápidas
- **Mi Interfaz** - Panel personalizable con arrastrar y soltar y widgets redimensionables
- **Joysticks virtuales** - Control de doble stick táctil/ratón con ajuste de velocidad
- **Soporte de gamepad** - PS5, Xbox o joystick genérico con mapeo de botones personalizado y cambio de modo
- **Control por teclado** - Controles WASD
- **Perfiles de velocidad** - Múltiples preajustes de velocidad para diferentes modos operacionales (Principiante, Normal y modo Extremo)
- **Acciones del robot** - Levantarse/sentarse, bloquear/desbloquear, selección de marcha, luces, transiciones de modo

### Cámara y Video

- **Streaming multi-cámara** - Descubrimiento dinámico para topics de cámara frontal, trasera y personalizados
- **Códecs H.264/H.265** - Escalado de resolución, control de tasa de fotogramas, optimización de ancho de banda
- **Grabación en el navegador** - Graba video de las cámaras y guárdalo en tu carpeta de descargas
- **Visualización 3D** - Modelo de robot basado en URDF con superposición de escaneo láser y ruta de navegación

### Monitoreo del Sistema

- **Estadísticas Jetson** - Modelo de placa, versión de JetPack, modo de energía, tiempo de actividad
- **Monitoreo de CPU/GPU** - Uso por núcleo, frecuencia, memoria, limitación térmica
- **Seguimiento de energía** - Voltaje, corriente y potencia por canal con detección de picos
- **Temperaturas y ventiladores** - Temperaturas de CPU/GPU/SOC con control de velocidad del ventilador
- **Almacenamiento y memoria** - Alertas de uso de disco, monitoreo de RAM/swap

### Red y Flota

- **Panel de control WiFi** - Escaneo de redes, cambio y monitoreo de señal
- **Modos de conexión** - WiFi, Ethernet, 4G, hotspot con seguimiento de latencia
- **Flota multi-robot** - Conexiones simultáneas, comandos para toda la flota, panel de estado
- **Diagnósticos** - Salud de nodos, registros de error/advertencia, visualización de la máquina de estados

### Personalización y UX

- **Temas claro/oscuro** - Colores de acento personalizados, preferencias persistentes
- **Diseños responsivos** - Móvil, tablet y escritorio con soporte táctil
- **Perfiles de usuario** - Avatar, nombre para mostrar, color del tema vía Supabase Auth
- **Multi-idioma** - Inglés y Portugués con formatos regionales
- **Registro de auditoría** - Historial de eventos buscable en más de 10 categorías con exportación CSV
- **Análisis de actividad** - Mapas de calor de uso y seguimiento de utilización del robot

## Tabla de Contenidos

- [Descripción General](#descripción-general)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Requisitos](#requisitos)
- [Instalación](#instalación)
  - [Configuración de Hardware](#1-configuración-de-hardware)
  - [Configuración de Supabase](#2-configuración-de-supabase)
  - [Configuración de Software](#3-configuración-de-software)
- [Desarrollo Frontend](#desarrollo-frontend)
- [Características](#características)
- [Configuración](#configuración)
- [Robots Personalizados](#agregar-soporte-para-otros-robots--robots-personalizados)
- [Solución de Problemas](#solución-de-problemas)
- [Contribuir](#contribuir)
- [Licencia](#licencia--citación)

## Descripción General

BotBrain consiste en tres componentes principales:

### Hardware
Una carcasa imprimible en 3D con soportes internos diseñados para alojar una placa NVIDIA Jetson y dos cámaras Intel RealSense D435i. El diseño modular te permite conectar BotBrain a varias plataformas de robots sin fabricación personalizada.

### Frontend
Un panel web Next.js 15 construido con React 19 y TypeScript. Proporciona control del robot en tiempo real, streaming de cámaras, visualización de mapas, planificación de misiones, monitoreo del sistema y gestión de flotas—todo accesible desde cualquier navegador en tu red.

### Robot (Workspace ROS2)
Una colección de paquetes ROS2 Humble que gestionan:
- **Bringup y Orquestación** (`bot_bringup`) - Lanzamiento y coordinación del sistema
- **Localización** (`bot_localization`) - SLAM basado en RTABMap para mapeo y posicionamiento
- **Navegación** (`bot_navigation`) - Integración Nav2 para movimiento autónomo
- **Percepción** (`bot_yolo`) - Detección de objetos YOLOv8/v11
- **Drivers de Robot** - Paquetes específicos de plataforma para Unitree Go2/G1, DirectDrive Tita y robots personalizados

---

## Estructura del Proyecto

```
BotBrain/
├── frontend/          # Panel web Next.js 15 (React 19, TypeScript)
├── botbrain_ws/       # Workspace ROS 2 Humble
│   └── src/
│       ├── bot_bringup/          # Lanzamiento principal y orquestación del sistema
│       ├── bot_custom_interfaces/# Mensajes, servicios y acciones ROS 2 personalizados
│       ├── bot_description/      # Modelos URDF/XACRO y robot_state_publisher
│       ├── bot_jetson_stats/     # Monitoreo de hardware Jetson
│       ├── bot_localization/     # SLAM RTABMap
│       ├── bot_navigation/       # Navegación autónoma Nav2
│       ├── bot_rosa/             # Control por lenguaje natural ROSA AI
│       ├── bot_state_machine/    # Gestión del ciclo de vida y estados
│       ├── bot_yolo/             # Detección de objetos YOLOv8/v11
│       ├── g1_pkg/               # Soporte Unitree G1
│       ├── go2_pkg/              # Soporte Unitree Go2
│       ├── joystick-bot/         # Interfaz de controlador de juego
│       └── tita_pkg/             # Soporte DirectDrive Tita
├── hardware/          # Carcasa imprimible en 3D (STL/STEP/3MF)
└── docs/              # Documentación
```

---

## Requisitos

### Hardware

| Componente | Requisito |
|-----------|-------------|
| **Cómputo** | NVIDIA Jetson (Nano, Orin Nano o serie AGX) |
| **Cámaras** | 2x Intel RealSense D435i |
| **Robot** | Robot ROS2 Humble o Unitree Go2 y Go2-W, Unitree G1, Direct Drive Tita, o [robot personalizado](../../botbrain_ws/README.md#creating-a-custom-robot-package) |
| **Red** | Conexión Ethernet o WiFi |

### Software

| Componente | Requisito |
|-----------|-------------|
| **SO** | JetPack 6.2 (Ubuntu 22.04) recomendado |
| **Contenedor** | Docker & Docker Compose |
| **Node.js** | v20+ (solo para desarrollo frontend local) |

---

## Instalación

BotBrain tiene dos componentes principales: **hardware** (carcasa impresa en 3D y componentes internos) y **software** (aplicación web frontend y workspace ROS2).

### 1. Configuración de Hardware

Imprime la carcasa en 3D y ensambla los componentes electrónicos.

**Partes Principales:** Impresora 3D, filamento PLA, NVIDIA Jetson, 2x RealSense D435i, convertidor de voltaje.

> **[Guía de Ensamblaje de Hardware](hardware/README_es.md)** - Instrucciones detalladas sobre cómo construir tu BotBrain
>
> **[Video Completo de Ensamblaje](https://youtu.be/xZ5c619bTEQ)** - Tutorial completo en video paso a paso del proceso de ensamblaje de BotBrain

### 2. Configuración de Supabase

El panel web requiere Supabase para autenticación y almacenamiento de datos. Necesitarás crear tu propio proyecto Supabase gratuito.

> **[Guía de Configuración de Supabase](../SUPABASE_SETUP.md)** - Instrucciones completas con esquema de base de datos

**Resumen rápido:**
1. Crea un proyecto en [supabase.com](https://supabase.com)
2. Ejecuta las migraciones SQL de la guía de configuración
3. Copia tus claves API para el siguiente paso

### 3. Configuración de Software

#### Dependencias Externas

**Sistema Operativo:**
- **NVIDIA JetPack 6.2** (recomendado)
- Otras distribuciones Linux pueden funcionar pero no están oficialmente soportadas

**Docker & Docker Compose:**

Requerido para despliegue containerizado:

1. Instala Docker:

```bash
# Agrega la clave GPG oficial de Docker:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Agrega el repositorio a las fuentes de Apt:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Instala los paquetes de Docker:
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Consulta la [guía oficial de instalación de Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) para más detalles.

2. Habilita Docker sin sudo:

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Consulta los [pasos post-instalación](https://docs.docker.com/engine/install/linux-postinstall/) para más detalles.

#### Pasos de Instalación

**Paso 1: Clona el Repositorio**

```bash
git clone https://github.com/botbotrobotics/BotBrain.git
cd BotBrain
```

**Paso 2: Ejecuta el Script de Instalación**

El script de instalación automatizado configurará tu robot y establecerá el servicio de autoarranque:

```bash
sudo ./install.sh
```
Más detalles sobre la información solicitada en el instalador se pueden encontrar [aquí](../installation-guide.md)

**Paso 3: Reinicia el Sistema**

```bash
sudo reboot
```

Una vez reiniciado, el sistema iniciará automáticamente los contenedores Docker para todos los nodos ROS2 y el servidor web.

**Paso 4: Accede a la Interfaz Web**

| Método de Acceso | URL |
|---------------|-----|
| Misma computadora | `http://localhost` |
| Acceso por red | `http://<IP_DEL_JETSON>` |

Encuentra la dirección IP de tu Jetson:
```bash
hostname -I
```

> **Nota:** Asegúrate de que ambos dispositivos estén en la misma red y que el puerto 80 no esté bloqueado por un firewall.

---

## Desarrollo Frontend

Para desarrollo frontend local (sin la stack completa del robot):

### Configuración

```bash
cd frontend

# Copia la plantilla de entorno
cp .env.example .env.local

# Edita con tus credenciales de Supabase
nano .env.local
```

### Variables de Entorno

| Variable | Requerida | Descripción |
|----------|----------|-------------|
| `NEXT_PUBLIC_SUPABASE_URL` | Sí | URL de tu proyecto Supabase |
| `NEXT_PUBLIC_SUPABASE_ANON_KEY` | Sí | Clave anon/pública de tu Supabase |
| `NEXT_PUBLIC_ROS_IP` | No | IP por defecto del robot (default: 192.168.1.95) |
| `NEXT_PUBLIC_ROS_PORT` | No | Puerto del ROS bridge (default: 9090) |

### Ejecución

```bash
# Instala dependencias
npm install

# Servidor de desarrollo (características completas)
npm run dev

# Servidor de desarrollo (edición open source)
npm run dev:oss

# Build de producción
npm run build
npm start
```
---

## Configuración

### Configuración del Robot

Edita `botbrain_ws/robot_config.yaml`:

```yaml
robot_configuration:
  robot_name: "mi_robot"           # Namespace para todos los topics
  robot_model: "go2"               # go2, tita, g1, o personalizado
  network_interface: "eth0"        # Interfaz de red para ROS2
  openai_api_key: ""               # Para características de IA (opcional)
```

### Configuración de Cámaras

Los números de serie de las cámaras y las transformaciones se configuran por robot en:
- `botbrain_ws/src/go2_pkg/config/camera_config.yaml`
- `botbrain_ws/src/g1_pkg/config/camera_config.yaml`
- `botbrain_ws/src/tita_pkg/config/camera_config.yaml`

Encuentra los números de serie de tus cámaras:
```bash
rs-enumerate-devices | grep "Serial Number"
```

---

## Agregar Soporte para Otros Robots / Robots Personalizados

Para agregar soporte para una nueva plataforma de robot a BotBrain:

1. **Backend/Stack ROS2**: Sigue la guía completa [Creando un Paquete de Robot Personalizado](../../botbrain_ws/README.md#creating-a-custom-robot-package)
2. **Frontend**: Agrega un perfil de robot en la configuración de la interfaz web

---

## Solución de Problemas

### Conexión WebSocket Falló
- Verifica que rosbridge esté ejecutándose: `ros2 node list | grep rosbridge`
- Verifica que el firewall permita el puerto 9090: `sudo ufw allow 9090`
- Asegúrate de que la IP sea correcta en la configuración de conexión del robot en la UI

### Cámara No Detectada
- Lista las cámaras conectadas: `rs-enumerate-devices`
- Verifica las conexiones USB y asegúrate de que las cámaras tengan energía
- Verifica que los números de serie en `camera_config.yaml` coincidan con tus cámaras
- Verifica los permisos USB: `sudo usermod -a -G video $USER`

### Problemas con Docker
- Asegúrate de que Docker se ejecute sin sudo (ver instrucciones de instalación)
- Verifica el acceso a GPU: `docker run --gpus all nvidia/cuda:11.0-base nvidia-smi`
- Ve los logs del contenedor: `docker compose logs -f bringup`

### Frontend No Carga
- Verifica las credenciales de Supabase en `.env.local`
- Revisa la consola del navegador para errores
- Asegúrate de que Node.js v20+ esté instalado: `node --version`

### Robot No Se Mueve
- Verifica que twist_mux esté ejecutándose: `ros2 topic echo /cmd_vel_out`
- Verifica que la interfaz de hardware del robot esté activa: `ros2 lifecycle get /robot_write_node`
- Verifica si la parada de emergencia está activada en la UI

### ¿Necesitas Más Ayuda?
Únete a nuestra [comunidad de Discord](https://discord.gg/CrTbJzxXes) para soporte en tiempo real y discusiones con la comunidad BotBrain.

---

## Bibliotecas de Terceros

Consulta [docs/DEPENDENCIES.md](../DEPENDENCIES.md) para una lista completa de los paquetes frontend y ROS utilizados.

---

## Contribuir

¡Aceptamos contribuciones! Ya sea que estés corrigiendo bugs, agregando características, mejorando documentación o agregando soporte para nuevos robots, tu ayuda es apreciada. Si puedes hacer BotBrain mejor o más rápido, trae tu contribución.

Únete a nuestro [servidor de Discord](https://discord.gg/CrTbJzxXes) para discutir ideas, obtener ayuda o coordinarte con otros contribuidores.

### Flujo de Desarrollo

1. **Haz Fork del Repositorio**
   ```bash
   # Haz fork vía la interfaz de GitHub, luego clona tu fork
   git clone https://github.com/botbotrobotics/BotBrain.git
   cd BotBrain
   ```

2. **Crea una Rama de Característica**
   ```bash
   git checkout -b feature/tu-caracteristica-increible
   ```

3. **Realiza Tus Cambios**
   - Agrega tests para nuevas funcionalidades
   - Actualiza los archivos README relevantes
   - Asegúrate de que todos los paquetes compilen exitosamente
   - Sigue los estándares de codificación de ROS 2

4. **Prueba Completamente**

5. **Haz Commit de Tus Cambios**
   ```bash
   git add .
   git commit -m "Add feature: breve descripción de los cambios"
   ```

6. **Empuja a Tu Fork**
   ```bash
   git push origin feature/tu-caracteristica-increible
   ```

7. **Envía un Pull Request**
   - Proporciona una descripción clara de tus cambios
   - Referencia cualquier issue relacionado
   - Incluye capturas de pantalla o videos para cambios de UI/comportamiento

---

## BotBrain Pro

<p align="center">
  <img src="../images/botbrainpro.png" alt="BotBrain Pro" width="600">
</p>

Versión Profesional / Enterprise de BotBrain con protección IP67, cargas útiles personalizadas como CamCam (Cámara Térmica + Infrarroja), ZoomZoom (cámara RGB de largo alcance 30x), modelos de IA avanzados, integración IoT (LoRA), conectividad de datos 3-5g, servicio y mantenimiento, integraciones avanzadas con cargas útiles personalizadas, y mucho más. [Aprende más aquí](https://botbot.bot).

---

## Seguridad

Los robots pueden lastimar a personas y a sí mismos cuando se operan incorrectamente o durante el desarrollo. Por favor, observa estas prácticas de seguridad:

- **Usa un E-stop físico** - Nunca confíes únicamente en paradas por software
- **Rota las claves API** si se filtran
- **Prueba los cambios en simulación** antes de ejecutarlos en hardware físico
- **Mantén distancia del robot** durante las pruebas iniciales

> **Aviso:** BotBot no es responsable de ninguna falla, accidente o daño resultante del uso de este software o hardware. El usuario asume toda la responsabilidad por la operación segura, pruebas y despliegue de robots usando BotBrain.

---

## Licencia

Este proyecto está licenciado bajo la **Licencia MIT** - consulta el archivo [LICENSE](../LICENSE) para más detalles.

---

<p align="center">Hecho con 💜 en Brasil</p>

<p align="right">
  <img src="../images/icon.png" width="110">
</p>
