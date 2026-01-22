<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# tita_pkg

**Pacote de interface de hardware do robô Tita**

O pacote `tita_pkg` fornece a camada de abstração de hardware ROS 2 para robôs Tita. Ele lida com comunicação bidirecional com o robô, publicação de dados de sensores, execução de comandos e serviços específicos do robô por meio de bridging de tópicos.

<!-- INSERIR IMAGEM DO ROBÔ AQUI -->
<!-- ![Tita Robot](docs/tita_robot.jpg) -->

## Objetivo do pacote

Este pacote faz a ponte entre os tópicos ROS 2 internos do robô Tita e o framework BotBrain, permitindo:

- **Bridge de tópicos**: comunicação entre o namespace interno do Tita e o sistema BotBrain
- **Integração de sensores**: publicação de odometria, IMU, estados de juntas, bateria e dados de percepção
- **Controle de movimento**: recebimento e execução de comandos de velocidade do twist_mux
- **Serviços do robô**: troca de modo via chamadas de serviço
- **Integração de controle**: tradução de entradas de joystick em comandos específicos do robô

## Nós

Todos os nós deste pacote são **nós de ciclo de vida**, fornecendo transições de estado gerenciadas para startup, shutdown e recuperação de erros robustos.

### Gerenciamento de ciclo de vida

#### Estados comuns de ciclo de vida

| Estado | Descrição |
|-------|-------------|
| **Unconfigured** | Estado inicial após criação do nó, sem recursos alocados |
| **Configured** | Recursos criados (publishers, subscribers, services), pronto para ativar |
| **Active** | Nó totalmente operacional, processando dados e executando funções |
| **Deactivated** | Nó pausado, recursos mantidos mas processamento interrompido |
| **Finalized** | Todos os recursos limpos, nó pronto para encerramento |

#### Transições padrão de ciclo de vida

| Transição | Descrição |
|------------|-------------|
| `configure` | Alocar recursos (criar publishers, subscribers, services) |
| `activate` | Iniciar processamento (começar a publicar, aceitar comandos) |
| `deactivate` | Pausar processamento (parar publicação mantendo recursos) |
| `cleanup` | Destruir recursos (fechar conexões, liberar memória) |
| `shutdown` | Limpeza emergencial e encerramento imediato |

#### Gerenciando estados de ciclo de vida

```bash
# Verificar estado atual
ros2 lifecycle get /{namespace}/robot_read_node

# Transitar pelos estados
ros2 lifecycle set /{namespace}/robot_read_node configure
ros2 lifecycle set /{namespace}/robot_read_node activate

# Desativar (pausar)
ros2 lifecycle set /{namespace}/robot_read_node deactivate

# Cleanup (liberar recursos)
ros2 lifecycle set /{namespace}/robot_read_node cleanup
```

**Nota**: O pacote `bot_state_machine` gerencia automaticamente as transições de ciclo de vida para todos os nós durante startup e shutdown do sistema.

---

### robot_read_node

Nó de ciclo de vida que assina tópicos internos do Tita e republica no namespace do BotBrain.

**Executável**: `tita_read.py`

**Descrição**: Atua como ponte de tópicos, assinando tópicos internos do Tita (sob `tita_namespace`) e republicando dados sob o namespace BotBrain do robô para integração em todo o sistema.

#### Publishers

| Tópico | Tipo de mensagem | Taxa | Descrição |
|-------|--------------|------|-------------|
| `/{namespace}/odom` | `nav_msgs/Odometry` | 50 Hz | Odometria do robô (posição, velocidade) do controlador do chassi Tita |
| `/{namespace}/imu/data` | `sensor_msgs/Imu` | 100 Hz | IMU (orientação, velocidade angular, aceleração linear) |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | 50 Hz | Posições e velocidades das juntas das pernas |
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | 1 Hz | Estado de bateria agregado dos pacotes esquerdo e direito |
| `/{namespace}/pointcloud` | `sensor_msgs/PointCloud2` | 10 Hz | Nuvem de pontos 3D do sistema de percepção |
| `/{namespace}/camera/image` | `sensor_msgs/Image` | 30 Hz | Feed de câmera das câmeras Tita |

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{tita_namespace}/chassis/odometry` | `nav_msgs/Odometry` | Odometria interna do Tita |
| `/{tita_namespace}/imu_sensor_broadcaster` | `sensor_msgs/Imu` | IMU interna do Tita |
| `/{tita_namespace}/joint_states` | `sensor_msgs/JointState` | Estados internos das juntas |
| `/{tita_namespace}/system/battery/left` | `sensor_msgs/BatteryState` | Estado da bateria esquerda |
| `/{tita_namespace}/system/battery/right` | `sensor_msgs/BatteryState` | Estado da bateria direita |
| `/{tita_namespace}/perception/camera/point_cloud` | `sensor_msgs/PointCloud2` | Nuvem de pontos de percepção |
| `/{tita_namespace}/perception/camera/image/raw` | `sensor_msgs/Image` | Imagens de câmera do Tita |

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo para frame ids (ex.: `odom`, `base_link`) |
| `tita_namespace` | string | `""` | Namespace interno do robô Tita para bridge de tópicos |

---

### robot_write_node

Nó de ciclo de vida que recebe comandos BotBrain e encaminha para tópicos internos do Tita.

**Executável**: `tita_write.py`

**Descrição**: Assina comandos de velocidade BotBrain e republica nos tópicos internos de comando do Tita. Fornece um serviço para controle de modo e configura `use_sdk` no nó de comandos do Tita.

#### Publishers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{tita_namespace}/command/user/command` | `tita_locomotion_interfaces/msg/LocomotionCmd` | Comandos de locomoção encaminhados ao controlador Tita |

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Comandos de velocidade do twist_mux |

#### Serviços

| Nome do serviço | Tipo de serviço | Descrição |
|--------------|--------------|-------------|
| `/{namespace}/mode` | `bot_custom_interfaces/srv/Mode` | Troca modo operacional (stand up/down) |

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo para tópicos assinados |
| `tita_namespace` | string | `""` | Namespace interno do robô Tita |

**Nota**:
- Em `activate`, este nó tenta definir `use_sdk=true` via `/{tita_namespace}/active_command_node/set_parameters`.

---

### controller_commands_node

Nó de ciclo de vida que traduz entradas do controle para comandos específicos do Tita.

**Executável**: `tita_controller_commands.py`

**Descrição**: Recebe eventos de botões do joystick e chama o serviço `mode` para levantar ou deitar com base em combinações de botões.

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/button_state` | `joystick_bot/msg/ControllerButtonsState` | Estados de botões processados |
| `/{tita_namespace}/locomotion/body/fsm_mode` | `std_msgs/String` | Modo atual do FSM do robô |

---

## Arquivos de launch

### robot_interface.launch.py

Lançador principal de interface de hardware que inicia todos os nós do Tita.

**Caminho**: [launch/robot_interface.launch.py](launch/robot_interface.launch.py)

**Descrição**: Inicia os três nós de ciclo de vida para integração completa do Tita. Handlers de ciclo de vida existem, mas estão comentados.

#### O que é iniciado

1. **robot_read_node**: Ponte de tópicos para dados de sensores
2. **robot_write_node**: Encaminhador de comandos e provedor de serviços
3. **controller_commands_node**: Tradutor de comandos do joystick

#### Argumentos de launch

Nenhum - configuração lida do [robot_config.yaml](../../../../robot_config.yaml)

#### Fonte de configuração

```yaml
robot_configuration:
  robot_name: "my_tita"            # Namespace BotBrain
  robot_model: "tita"              # Deve ser "tita"
  tita_namespace: "tita3036731"    # Namespace interno do Tita (único por robô)
```

#### Uso

```bash
# Iniciar interface de hardware Tita
ros2 launch tita_pkg robot_interface.launch.py

# Verificar nós em execução
ros2 node list | grep tita

# Verificar estados de ciclo de vida
ros2 lifecycle list robot_read_node
ros2 lifecycle get robot_read_node
```

**Nota**: Este launch file é incluído automaticamente por `bot_bringup` quando `robot_model: "tita"` em `robot_config.yaml`.

## Arquivos de configuração

### nav2_params.yaml

Parâmetros Navigation2 ajustados especificamente para a cinemática e dinâmica do robô Tita.

**Caminho**: [config/nav2_params.yaml](config/nav2_params.yaml)

**Descrição**: Ajustes específicos do robô para a stack Nav2 incluindo limites de velocidade, limites de aceleração, footprint e parâmetros de controlador otimizados para a locomoção do Tita.

Este arquivo é carregado automaticamente por `bot_navigation` quando Tita é selecionado.

### camera_config.yaml

Configuração de câmera e percepção para sensores Tita.

**Caminho**: [config/camera_config.yaml](config/camera_config.yaml)

**Descrição**: Configuração para câmeras Intel RealSense montadas no robô Tita. Define tipos de câmera, números de série, posições de montagem e transformações TF para SLAM e percepção com uma ou múltiplas câmeras.

#### Estrutura de configuração

```yaml
camera_configuration:
  front:
    type: "d435i"                      # Modelo de câmera Intel RealSense
    serial_number: ""                  # Identificador único da câmera
    parent_frame: "botbrain_base"      # Frame pai (base do robô)
    child_frame: "front_camera_link"   # Nome do frame da câmera
    tf:
      x: 0.08                          # Offset para frente (metros)
      y: 0.0175                        # Offset lateral (metros)
      z: 0.043                         # Offset vertical (metros)
      roll: 0                          # Rotação no eixo X (radianos)
      pitch: 0                         # Rotação no eixo Y (radianos)
      yaw: 0                           # Rotação no eixo Z (radianos)

  back:
    type: "d435i"                      # Câmera Intel RealSense D435i
    serial_number: ""                  # Identificador único da câmera
    parent_frame: "botbrain_base"      # Frame pai (base do robô)
    child_frame: "back_camera_link"    # Nome do frame da câmera
    tf:
      x: -0.08                         # Offset para trás (metros)
      y: -0.0175                       # Offset lateral (metros)
      z: 0.043                         # Offset vertical (metros)
      roll: 0                          # Sem rotação em roll
      pitch: 0                         # Sem rotação em pitch
      yaw: 3.14159                     # Rotação de 180° (apontando para trás)
```

#### Descrição dos parâmetros

**Identificação da câmera**:
- `type`: Identificador do modelo de câmera (ex.: "d435i", "d455", "t265")
  - Usado para selecionar parâmetros de launch ROS 2 apropriados
- `serial_number`: Serial único do dispositivo
  - Necessário quando múltiplas câmeras do mesmo tipo estão conectadas
  - Garante que a câmera correta seja iniciada com a configuração correta
  - Descobrir via: `rs-enumerate-devices`

**Frames de transformação**:
- `parent_frame`: Frame de referência para montagem da câmera
  - Tipicamente `"botbrain_base"` ou `"base_link"`
  - Deve corresponder ao frame definido no URDF/XACRO do robô
- `child_frame`: Nome do frame óptico da câmera
  - Convenção: `"{position}_camera_link"` (ex.: "front_camera_link")
  - Publicado por drivers de câmera ou publicadores de transformação estática

**Parâmetros de transformação (TF)**:
- `x`, `y`, `z`: Offset de posição 3D a partir do frame pai (metros)
  - **x**: Positivo para frente, negativo para trás
  - **y**: Positivo para a esquerda, negativo para a direita (perspectiva do robô)
  - **z**: Positivo para cima, negativo para baixo
  - Exemplo: câmera frontal em `x: 0.08` está 8 cm à frente da base

- `roll`, `pitch`, `yaw`: Ângulos de rotação em radianos
  - **roll**: Rotação no eixo X (eixo frontal)
  - **pitch**: Rotação no eixo Y (eixo lateral)
  - **yaw**: Rotação no eixo Z (eixo vertical)
  - Exemplo: câmera traseira com `yaw: 3.14159` (pi rad = 180°) aponta para trás

## Arquivos de descrição do robô

### Arquivos XACRO

Localizados no diretório [xacro/](xacro/):

- **robot.xacro**: Descrição principal do robô Tita incluindo corpo e pernas
- **[component].xacro**: Componentes XACRO adicionais para pernas, sensores, etc.

### Malhas

Localizadas no diretório [meshes/](meshes/):

Malhas visuais e de colisão para visualização no RViz e detecção de colisão no Nav2.

## Transformações (TF)

### TF Broadcasters

O robot_read_node publica transformações baseadas em odometria:

| Frame pai | Frame filho | Fonte | Taxa |
|--------------|-------------|--------|------|
| `odom` | `base_link` | Odometria Tita | 50 Hz |

A árvore cinemática completa é publicada pelo `robot_state_publisher` (de `bot_description`) usando estados de juntas deste pacote.

## Integração com o sistema BotBrain

### Carregamento automático

Este pacote é carregado automaticamente por `bot_bringup` quando configurado:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "tita"  # Dispara o carregamento do tita_pkg
```

O sistema de bringup:
1. Lê `robot_model: "tita"`
2. Constrói o nome do pacote: `tita_pkg`
3. Inclui `tita_pkg/launch/robot_interface.launch.py`
4. Carrega a descrição de `tita_pkg/xacro/robot.xacro`

### Arquitetura de bridge de tópicos

O pacote implementa uma ponte bidirecional de tópicos:

**Fluxo de dados de sensores** (Tita → BotBrain):
```
Tita Internal Topic → robot_read_node → BotBrain Topic
/{tita_namespace}/chassis/odometry → [bridge] → /{robot_name}/odom
/{tita_namespace}/imu_sensor_broadcaster → [bridge] → /{robot_name}/imu/data
/{tita_namespace}/perception/camera/point_cloud → [bridge] → /{robot_name}/pointcloud
/{tita_namespace}/perception/camera/image/raw → [bridge] → /{robot_name}/camera/image
```

**Fluxo de comandos** (BotBrain → Tita):
```
BotBrain Topic → robot_write_node → Tita Internal Topic
/{robot_name}/cmd_vel_out → [bridge] → /{tita_namespace}/command/user/command
```

### Dependências necessárias

**Dependências de sistema**:
- Conectividade de rede com o robô Tita (Ethernet ou WiFi)
- Configuração correta do domínio ROS 2

**Dependências de pacote ROS 2**:
- `rclcpp` / `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `std_msgs`
- `bot_custom_interfaces`
- `tita_locomotion_interfaces`
- `joystick_bot`
- `rcl_interfaces`
- `tf2_ros`

### Integração de tópicos

**Publica no sistema BotBrain**:
- Tópicos de sensores → usados por `bot_localization`, `bot_navigation`
- Estados de juntas → usados por `bot_description` (robot_state_publisher)

**Assina do sistema BotBrain**:
- `cmd_vel_out` de `bot_bringup` (saída do twist_mux)
- `button_state` de `joystick_bot`

## Uso

### Teste standalone

Teste a interface Tita sem o sistema completo:

```bash
# Source do workspace
source install/setup.bash

# Iniciar apenas a interface Tita
ros2 launch tita_pkg robot_interface.launch.py

# Em outro terminal, verifique os nós
ros2 node list

# Saída esperada:
# /robot_name/robot_read_node
# /robot_name/robot_write_node
# /robot_name/controller_commands_node
```

## Estrutura de diretórios

```
tita_pkg/
├── launch/
│   └── robot_interface.launch.py     # Lançador principal de interface de hardware
│
├── scripts/
│   ├── tita_read.py                  # Ponte de tópicos para dados de sensores
│   ├── tita_write.py                 # Nó encaminhador de comandos
│   └── tita_controller_commands.py   # Nó tradutor de controle
│
├── config/
│   ├── nav2_params.yaml              # Parâmetros de navegação para Tita
│   └── camera_config.yaml            # Configuração de câmera/percepção
│
├── xacro/
│   └── [XACRO files]                 # Arquivos de descrição do robô
│
├── meshes/
│   └── [mesh files]                  # Malhas 3D de visualização
│
├── maps/
│   └── [map files]                   # Mapas pré-construídos para Tita
│
├── tita_pkg/
│   └── tools/                        # Módulos utilitários
│
├── tita_setup.bash                   # Script de setup do ambiente
├── CMakeLists.txt                    # Configuração de build
├── package.xml                       # Manifesto do pacote
└── README.md                         # Este arquivo
```

---

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
