<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# g1_pkg

**Pacote de interface de hardware do robô humanoide Unitree G1**

O pacote `g1_pkg` fornece a camada de abstração de hardware ROS 2 para o robô humanoide Unitree G1. Ele entrega telemetria em tempo real, execução de comandos de locomoção, controle de pose do tronco superior, integração com joystick, streaming do Livox MID360 e ferramentas auxiliares para bringup do sistema.

## Objetivo do pacote

Este pacote conecta o G1 ao ecossistema ROS 2, permitindo:

- **Comunicação de hardware**: interface Unitree DDS nativa para troca de comandos/estado
- **Integração de sensores**: publica bateria, IMU, odometria, estados de juntas e TF
- **Controle de movimento**: consome `cmd_vel_out`, altera modos da Finite State Machine (FSM) (máquina de estados interna de locomoção Unitree) e lida com paradas de emergência
- **Controle do tronco superior**: salva e reproduz poses de braço armazenadas em `arm_poses.txt`
- **Integração de controle**: mapeia combinações de botões do joystick para transições da FSM
- **Pipeline de LiDAR**: driver Livox MID360 + conversão PointCloud→LaserScan

## Nós

Todos os nós são **nós de ciclo de vida** para configuração, ativação e teardown previsíveis.

### Gerenciamento de ciclo de vida

#### Estados comuns de ciclo de vida

| Estado | Descrição |
|-------|-------------|
| **Unconfigured** | Nó recém-criado sem recursos |
| **Configured** | Parâmetros carregados, publishers/subscribers/services criados |
| **Active** | Processamento completo; tópicos/serviços ativos |
| **Deactivated** | Recursos mantidos, mas processamento pausado |
| **Finalized** | Todos os recursos limpos antes do término |

#### Transições padrão de ciclo de vida

| Transição | Descrição |
|------------|-------------|
| `configure` | Declara parâmetros e inicializa drivers Unitree |
| `activate` | Conecta publishers/subscribers/services e inicia o processamento |
| `deactivate` | Para publicação/comandos mantendo recursos |
| `cleanup` | Destrói recursos e fecha conexões |
| `shutdown` | Finalização imediata com lógica de cleanup |

#### Gerenciando estados de ciclo de vida

```bash
# Verificar estado atual
ros2 lifecycle get /{namespace}/robot_read_node

# Sequência padrão
ros2 lifecycle set /{namespace}/robot_read_node configure
ros2 lifecycle set /{namespace}/robot_read_node activate

# Pausar e limpar
ros2 lifecycle set /{namespace}/robot_read_node deactivate
ros2 lifecycle set /{namespace}/robot_read_node cleanup
```

**Nota**: O pacote `bot_state_machine` gerencia automaticamente as transições de ciclo de vida para todos os nós durante startup e shutdown do sistema.

---

### robot_read_node

Nó de ciclo de vida que assina tópicos Unitree (`/lf/bmsstate`, `/lf/lowstate`, `/lf/odommodestate`) e republica mensagens padrão ROS.

**Executável**: `g1_read.py`

**Descrição**: Converte telemetria bruta em `sensor_msgs`/`nav_msgs`, gera `JointState` para todos os 23 atuadores e transmite a transformação `odom → base_link` (stream de 20 Hz).

#### Publishers

| Tópico | Tipo de mensagem | Taxa | Descrição |
|-------|--------------|------|-------------|
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | 20 Hz | Tensão/corrente/SOC da bateria |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | 20 Hz | 23 juntas (pernas + tronco superior) |
| `/{namespace}/odom` | `nav_msgs/Odometry` | 20 Hz | Pose e twist em `base_link` |
| `/{namespace}/imu/data` | `sensor_msgs/Imu` | 20 Hz | Quaternion, gyro e aceleração |
| `/{namespace}/imu_temp` | `std_msgs/Float32` | 20 Hz | Temperatura da IMU |

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/lf/bmsstate` | `unitree_hg/BmsState` | Informação do pack de bateria |
| `/lf/lowstate` | `unitree_hg/LowState` | Telemetria por atuador |
| `/lf/odommodestate` | `unitree_go/SportModeState` | IMU, odometria, velocidades |

#### Serviços

Nenhum

#### Ações

Nenhuma

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo de namespace anexado a frames e tópicos |

---

### robot_write_node

Nó de ciclo de vida (C++) que envia comandos de locomoção ao G1, controla a FSM, aplica poses de braço e gerencia paradas de emergência.

**Executável**: `g1_write_node` (encapsula `g1_write.cpp` + `g1_driver`)

**Descrição**: Cria uma instância `G1Driver` (DDS oficial), recebe comandos de velocidade, expõe serviços para controle de modo/atendimento de emergência/gestão de poses e publica a lista de poses armazenadas.

#### Publishers

| Tópico | Tipo de mensagem | Taxa | Descrição |
|-------|--------------|------|-------------|
| `/{namespace}/pose/names` | `bot_custom_interfaces/msg/Names` | 0.5 Hz | Nomes das poses atuais de `arm_poses.txt` |

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Comandos de velocidade (limitados internamente a 0.6 m/s e 1.0 rad/s) |

#### Serviços

| Nome do serviço | Tipo de serviço | Descrição |
|--------------|--------------|-------------|
| `/{namespace}/mode` | `bot_custom_interfaces/srv/Mode` | Define FSM (`zero_torque`, `damp`, `preparation`, `run`, `squat`, `start`) com confirmação |
| `/{namespace}/current_mode` | `bot_custom_interfaces/srv/CurrentMode` | Obtém o estado atual da FSM |
| `/{namespace}/emergency_stop` | `std_srvs/srv/SetBool` | Sequência completa de emergência (stop → squat → damping) |
| `/{namespace}/arm_cmd` | `bot_custom_interfaces/srv/ArmCmd` | Salvar/carregar/aplicar/liberar/deletar poses de braço |

#### Ações

Nenhuma

#### Parâmetros

Valores definidos em [`config/g1_params.yaml`](config/g1_params.yaml):

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|---------|-------------|
| `robot.keep_move` | bool | `false` | Mantém o gait contínuo após comandos |
| `robot.speed_mode` | int | `0` | Perfil de velocidade (`0`=1.0 m/s … `3`=3.0 m/s) |

> Durante `on_deactivate()` o nó emite `stop_move()` automaticamente por segurança.

---

### controller_commands_node

Nó de ciclo de vida (Python) que lê eventos do joystick e traduz em comandos de modo FSM.

**Executável**: `g1_controller_commands.py`

**Descrição**: Assina `button_state`, rastreia histórico de botões e dispara serviços de modo (`mode`, `current_mode`) para combos como `L2+B`, `R2+A` e long-press `L2+B` para `run → damp`.

#### Publishers

Nenhum (atua via clientes de serviço)

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/button_state` | `joystick_bot/ControllerButtonsState` | Estados brutos dos botões do joystick |

#### Serviços

Nenhum (usa clientes de serviço para `mode` e `current_mode`)

#### Ações

Nenhuma

#### Parâmetros

Nenhum

---

### Pipeline Livox MID360

Dois componentes adicionais são iniciados junto aos nós principais:

1. **livox_lidar_publisher**  
   - **Executável**: `livox_ros_driver2_node`  
   - **Config**: [`config/MID360_config.json`](config/MID360_config.json)  
   - **Saída**: `/{namespace}/pointcloud` (remapeado em `pc2ls`)  
   - **Parâmetros**: `xfer_format`, `publish_freq`, `frame_id = {prefix}mid360_link`, etc.

2. **pointcloud_to_laserscan_node**  
   - **Executável**: `pointcloud_to_laserscan_node`  
   - **Config**: [`config/pointcloud_to_laserscan_params.yaml`](config/pointcloud_to_laserscan_params.yaml)  
    - **Remappings**: `cloud_in → pointcloud`, `scan → scan`  
    - **Saída**: `/{namespace}/scan` (laser scan 2D gerado a partir do cloud MID360)

## Arquivos de launch

### robot_interface.launch.py

**Caminho**: [`launch/robot_interface.launch.py`](launch/robot_interface.launch.py)

**Descrição**: Lê `robot_config.yaml` no nível do workspace para obter `robot_name`, calcula o prefixo do namespace e inicia:

1. `robot_write_node` (C++)  
2. `robot_read_node`  
3. `controller_commands_node`  
4. `livox_lidar_publisher`  
5. `pointcloud_to_laserscan_node`

#### Argumentos de launch

Nenhum (prefixo derivado do `robot_config.yaml` global).

#### Fonte de configuração

```yaml
robot_configuration:
  robot_name: "g1_robot"         # Namespace aplicado a cada nó
  network_interface: "eno1"      # Interface de rede para comunicação do robô
```

## Arquivos de configuração

| Arquivo | Descrição |
|------|-------------|
| [`config/g1_params.yaml`](config/g1_params.yaml) | Parâmetros de `robot_write_node` (keep_move, speed_mode) |
| [`config/nav2_params.yaml`](config/nav2_params.yaml) | Ajustes Nav2 para cinemática humanoide (limites de velocidade/aceleração, footprint) |
| [`config/camera_config.yaml`](config/camera_config.yaml) | Calibração Intel RealSense (frames, offsets, seriais) |
| [`config/MID360_config.json`](config/MID360_config.json) | Rede e extrínsecos do LiDAR Livox |
| [`config/pointcloud_to_laserscan_params.yaml`](config/pointcloud_to_laserscan_params.yaml) | Filtros de altura, alcance min/max, passo angular para LaserScan |
| [`config/arm_poses.txt`](config/arm_poses.txt) | Biblioteca de poses de braço consumida pelo serviço `arm_cmd` |

### nav2_params.yaml

Parâmetros do Navigation2 ajustados especificamente para a cinemática e dinâmica do humanoide Unitree G1.

**Caminho**: [`config/nav2_params.yaml`](config/nav2_params.yaml)

**Descrição**: Ajustes específicos do robô para a stack Nav2 incluindo limites de velocidade, restrições de aceleração, footprint e configurações de controlador sob medida para locomoção humanoide. Este arquivo é geralmente carregado por `bot_navigation` quando o `robot_model` é definido como `g1`.

### camera_config.yaml

Parâmetros de calibração de câmera para os sensores Intel RealSense onboard.

**Caminho**: [`config/camera_config.yaml`](config/camera_config.yaml)

**Descrição**: Define tipos de câmera, números de série, frames de montagem e transformações TF para que nós de SLAM/percepção consumam os feeds com os extrínsecos corretos.

#### Estrutura de configuração

```yaml
camera_configuration:
  front:
    type: "d435i"
    serial_number: "344422070967"
    parent_frame: "torso_link"
    child_frame: "front_camera_link"
    tf:
      x: 0.0576235
      y: 0.01753
      z: 0.41987
      roll: 0
      pitch: 0.8307767239493009
      yaw: 0

  back:
    type: ""
    serial_number: "339222071455"
    parent_frame: "base_link"
    child_frame: "back_camera_link"
    tf:
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
```

#### Descrição dos parâmetros

**Identificação da câmera**
- `type`: Identificador do modelo de câmera (ex.: `d435i`, `d455`, `t265`) usado para selecionar os parâmetros de launch corretos.
- `serial_number`: Serial único do dispositivo; necessário quando múltiplas câmeras idênticas estão conectadas (descobrir via `rs-enumerate-devices`).

**Frames de transformação**
- `parent_frame`: Frame onde a câmera está montada (`torso_link`, `base_link`, etc.) e deve existir no URDF/XACRO.
- `child_frame`: Frame óptico da câmera (convenção: `{position}_camera_link`) publicado pelo driver ou por TF estático.

**Parâmetros de transformação (TF)**
- `x`, `y`, `z`: Offsets de posição (metros) a partir do frame pai. `x` positivo para frente, `y` para a esquerda, `z` para cima.
- `roll`, `pitch`, `yaw`: Offsets de orientação (radianos). Exemplo: a câmera frontal tem ~0.83 rad de pitch para apontar levemente para baixo.

## Arquivos de descrição do robô

- **URDF**: [`urdf/`](urdf/) contém descrições prontas para RViz/Gazebo.
- **XACRO**: [`xacro/`](xacro/) contém blocos reutilizáveis (`robot.xacro`, `leg.xacro`, `const.xacro`, `materials.xacro`).
- **Meshes**: [`meshes/`](meshes/) armazena modelos STL/DAE para tronco, membros e acessórios (visual + colisão).

## Transformações (TF)

### TF Broadcasters

| Frame pai | Frame filho | Fonte | Taxa |
|--------------|-------------|--------|------|
| `/{namespace}/odom` | `/{namespace}/base_link` | `robot_read_node` (integração de odometria) | 20 Hz |

A árvore cinemática restante é publicada pelo `robot_state_publisher`, que consome os estados de juntas emitidos por `robot_read_node`.

### Frames auxiliares

- **Câmeras**: `front_camera_link`, `back_camera_link` de acordo com `config/camera_config.yaml`.  
- **LiDAR**: `mid360_link` fornecido por `livox_lidar_publisher` (prefixo aplicado via argumentos de launch).  
- **Poses de braço**: quaisquer transformações estáticas adicionais definidas no URDF/XACRO são mantidas pelo pacote de descrição. 

Esses frames permitem que pipelines de percepção e SLAM alinhem dados de câmera/LiDAR com a base humanoide.

## Fluxo de poses de braço

```bash
# Salvar pose atual
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 0, name: 'pose_wave'}"

# Monitorar tópico de nomes de pose (lista a 20 Hz)
ros2 topic hz /{namespace}/pose/names

# Reaplicar pose
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 1, name: 'pose_wave'}"

# Liberar juntas do braço (retorna controle ao controlador onboard)
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 2, name: ''}"

# Deletar uma pose armazenada
ros2 service call /{namespace}/arm_cmd bot_custom_interfaces/srv/ArmCmd \
"{command: 3, name: 'pose_wave'}"
```

## Integração com o sistema BotBrain

### Carregamento automático

Este pacote é iniciado automaticamente por `bot_bringup` quando `robot_model: "g1"` é selecionado no `robot_config.yaml` do workspace:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "g1"   # Dispara o carregamento de g1_pkg
```

O pipeline de bringup então:

1. Lê `robot_model: "g1"` do arquivo de configuração.
2. Resolve o nome do pacote `g1_pkg`.
3. Inclui `g1_pkg/launch/robot_interface.launch.py` no launch do sistema.
4. Carrega a descrição do humanoide de `g1_pkg/xacro/robot.xacro`.

Quando `bot_bringup` está rodando com `robot_model: "g1"`, todos os nós de ciclo de vida acima iniciam automaticamente e o `bot_state_machine` gerencia suas transições (configure → activate) durante startup e shutdown.

## Uso

### Teste standalone

```bash
# Source do workspace
source install/setup.bash

# Iniciar a interface completa (fora do bot_bringup)
ros2 launch g1_pkg robot_interface.launch.py

# Verificar nós ativos
ros2 node list | grep g1_robot

# Saída esperada:
# /robot_name/robot_read_node
# /robot_name/robot_write_node
# /robot_name/controller_commands_node
# /robot_name/robot_video_stream

# Alterar ciclo de vida manualmente (exemplo)
ros2 lifecycle set /g1_robot/robot_write_node activate
```

## Estrutura de diretórios

```
g1_pkg/
├── launch/
│   ├── robot_interface.launch.py     # Bringup primário (hardware + sensores)
│   ├── livox_MID360.launch.py        # Driver Livox com config dedicada
│   └── pc2ls.launch.py               # Conversão PointCloud → LaserScan
├── scripts/
│   ├── g1_read.py                    # Nó de telemetria (ciclo de vida)
│   └── g1_controller_commands.py     # Nó de joystick (ciclo de vida)
├── src/
│   ├── g1_write.cpp                  # Implementação do writer de ciclo de vida
│   ├── g1_write_node.cpp             # C++ principal
│   └── g1_driver/                    # Wrapper Unitree DDS para o G1
├── include/
│   └── g1_write.hpp                  # Declaração da classe G1Write
├── config/
│   ├── g1_params.yaml                # Parâmetros do write-node (speed_mode, keep_move)
│   ├── nav2_params.yaml              # Parâmetros de navegação ajustados para o G1
│   ├── camera_config.yaml            # Calibração RealSense (frames, offsets, seriais)
│   ├── MID360_config.json            # Rede/extrínsecos do Livox MID360
│   ├── pointcloud_to_laserscan_params.yaml  # Filtros de PointCloud→LaserScan
│   └── arm_poses.txt                 # Poses do tronco superior salvas
├── urdf/                             # Modelos URDF prontos
├── xacro/                            # Blocos reutilizáveis Xacro
├── meshes/                           # Malhas DAE/STL de visual e colisão
├── maps/                             # Mapas de navegação pré-construídos (se houver)
├── g1_pkg/__init__.py                # Pacote helper Python
├── g1_setup.bash                     # Script de setup do ambiente
├── CMakeLists.txt / package.xml      # Metadados ROS 2
└── README.md                         # Este arquivo
```

---

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
