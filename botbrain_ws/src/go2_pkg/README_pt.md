<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# go2_pkg

**Pacote de interface de hardware do robô quadrúpede Unitree Go2**

O pacote `go2_pkg` fornece a camada de abstração de hardware ROS 2 para robôs quadrúpedes Unitree Go2. Ele lida com comunicação bidirecional com o robô, publicação de dados de sensores, execução de comandos, streaming de vídeo e serviços específicos do robô.


## Objetivo do pacote

Este pacote faz a interface com o robô Unitree Go2 via tópicos ROS 2, permitindo:

- **Comunicação de hardware**: troca de dados bidirecional em tempo real com o robô Go2
- **Integração de sensores**: publicação de odometria, IMU, estados de juntas e bateria
- **Controle de movimento**: recebimento e execução de comandos de velocidade do twist_mux
- **Streaming de vídeo**: publicação de feeds de câmera
- **Serviços do robô**: troca de modos, controle de gait, ajuste de pose e recursos de segurança
- **Integração de controle**: tradução de entradas de joystick em comandos específicos do robô

## Nós

Todos os nós neste pacote são **nós de ciclo de vida**, fornecendo transições de estado gerenciadas para startup, shutdown e recuperação de erros robustos.

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

Nó de ciclo de vida que lê dados de sensores do robô Go2 e publica em tópicos ROS 2.

**Executável**: `go2_read.py`

**Descrição**: Assina tópicos ROS 2 Unitree, processa dados de estado do robô e publica mensagens padrão ROS 2 de sensores. Fornece odometria, IMU, estados de juntas, status de bateria e dados de LiDAR.

#### Publishers

| Tópico | Tipo de mensagem | Taxa | Descrição |
|-------|--------------|------|-------------|
| `/{namespace}/odom` | `nav_msgs/Odometry` | 50 Hz | Odometria do robô (posição, velocidade) no frame base_link |
| `/{namespace}/imu/data` | `sensor_msgs/Imu` | 100 Hz | IMU (orientação, velocidade angular, aceleração linear) |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | 50 Hz | Posições e velocidades das 12 juntas das pernas |
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | 1 Hz | Tensão, corrente, porcentagem e saúde da bateria |
| `/{namespace}/imu_temp` | `std_msgs/Float32` | 1 Hz | Temperatura da IMU em Celsius |
| `/{namespace}/pointcloud` | `sensor_msgs/PointCloud2` | 10 Hz | Nuvem de pontos LiDAR (se equipado) |

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/lf/sportmodestate` | `unitree_go/SportModeState` | Estado de modo sport da Unitree |
| `/lf/lowstate` | `unitree_go/LowState` | Estado de baixo nível da Unitree |
| `/utlidar/robot_pose` | `geometry_msgs/PoseStamped` | Pose do robô via LiDAR Unitree |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Nuvem de pontos do LiDAR Unitree |

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo de tópico (namespace) para dados publicados |

---

### lifecycle_robot_write_node

Nó de ciclo de vida que recebe comandos ROS 2 e envia para o robô Go2.

**Executável**: `go2_write.py`

**Descrição**: Assina comandos de velocidade do twist_mux e envia ao robô Go2 via tópicos da API ROS 2 Unitree. Fornece serviços para troca de modo, controle de gait, ajuste de pose e recursos de segurança. Monitora o estado do robô e publica informações de status.

#### Publishers

| Tópico | Tipo de mensagem | Taxa | Descrição |
|-------|--------------|------|-------------|
| `/{namespace}/robot_status` | `bot_custom_interfaces/msg/RobotStatus` | 2 Hz | Status operacional do robô (modo, gait, emergência) |
| `/api/sport/request` | `unitree_api/msg/Request` | Event | Requisições da API de modo sport Unitree |
| `/api/robot_state/request` | `unitree_api/msg/Request` | Event | Requisições da API de estado Unitree |
| `/api/vui/request` | `unitree_api/msg/Request` | Event | Requisições da API de voz Unitree |
| `/api/obstacles_avoid/request` | `unitree_api/msg/Request` | Event | Requisições da API de desvio de obstáculos Unitree |

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Comandos de velocidade do twist_mux (linear.x, linear.y, angular.z) |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | Feedback de modo sport Unitree |
| `/api/sport/response` | `unitree_api/msg/Response` | Respostas da API sport Unitree |
| `/api/robot_state/response` | `unitree_api/msg/Response` | Respostas da API de estado Unitree |
| `/api/vui/response` | `unitree_api/msg/Response` | Respostas da API de voz Unitree |

#### Serviços

| Nome do serviço | Tipo de serviço | Descrição |
|--------------|--------------|-------------|
| `/{namespace}/mode` | `bot_custom_interfaces/srv/Mode` | Troca modo operacional (stand, walk, lie down) |
| `/{namespace}/switch_gait` | `bot_custom_interfaces/srv/SwitchGait` | Muda padrão de gait (trot, walk, run) |
| `/{namespace}/body_height` | `bot_custom_interfaces/srv/BodyHeight` | Ajusta altura do corpo (-0.1 a +0.1 m) |
| `/{namespace}/foot_raise_height` | `bot_custom_interfaces/srv/FootRaiseHeight` | Ajusta altura de levantamento do pé (0.06-0.1 m) |
| `/{namespace}/speed_level` | `bot_custom_interfaces/srv/SpeedLevel` | Define nível de velocidade (1-5, 5 mais rápido) |
| `/{namespace}/pose` | `bot_custom_interfaces/srv/Pose` | Define pose do corpo (roll, pitch, yaw) |
| `/{namespace}/euler` | `bot_custom_interfaces/srv/Euler` | Define orientação por ângulos de Euler |
| `/{namespace}/continuous_gait` | `bot_custom_interfaces/srv/ContinuousGait` | Habilita transições contínuas de gait |
| `/{namespace}/switch_joystick` | `bot_custom_interfaces/srv/SwitchJoystick` | Alterna modos de controle por joystick |
| `/{namespace}/current_mode` | `bot_custom_interfaces/srv/CurrentMode` | Consulta modo operacional atual |
| `/{namespace}/emergency_stop` | `std_srvs/srv/SetBool` | Dispara parada de emergência (interrompe todo movimento) |
| `/{namespace}/light_control` | `bot_custom_interfaces/srv/LightControl` | Controla LEDs do robô |
| `/{namespace}/obstacle_avoidance` | `bot_custom_interfaces/srv/ObstacleAvoidance` | Habilita/desabilita desvio de obstáculos onboard |

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo de tópico (namespace) para tópicos assinados |

**Nota**:
- Em `activate`, este nó executa automaticamente a sequência de inicialização:
  1. Envia comando `stand_down` para deitar o robô
  2. Desabilita o modo **MCF (Motion Control Framework)** e troca para **Sport Mode**
  3. Envia comando `stand_up` para colocar o robô em pé
- Em `deactivate`, este nó envia automaticamente um comando de parada (velocidade zero) por segurança.

---

### controller_commands_node

Nó de ciclo de vida que traduz entradas do controle para comandos específicos do robô.

**Executável**: `go2_controller_commands.py`

**Descrição**: Recebe eventos de botões e eixos do joystick e traduz em comandos específicos do Go2 como troca de modo, mudança de gait e ajuste de pose. Fornece mapeamento de botões para funções especiais do robô.

#### Publishers

Vários tópicos de comando baseados nos mapeamentos de botões (chama serviços no robot_write_node)

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/joy` | `sensor_msgs/Joy` | Entrada bruta do joystick |
| `/{namespace}/controller_state` | `bot_custom_interfaces/msg/ControllerButtonsState` | Estados de botões processados |

### robot_video_stream

Nó de ciclo de vida que captura e publica vídeo das câmeras onboard do Go2.

**Executável**: `go2_video_stream.py`

**Descrição**: Conecta aos streams de câmera do Go2, decodifica H.264/H.265 e publica mensagens ROS 2 Image. Suporta múltiplas câmeras e fornece informações de calibração.

#### Publishers

| Tópico | Tipo de mensagem | Taxa | Descrição |
|-------|--------------|------|-------------|
| `/{namespace}/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Imagem bruta (decodificada de H.264/H.265) |
| `/{namespace}/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Parâmetros de calibração da câmera |

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo de tópico para imagens publicadas |
| `network_interface` | string | `"eth0"` | Interface de rede para streaming de vídeo |

---

## Arquivos de launch

### robot_interface.launch.py

Lançador principal de interface de hardware que inicia todos os nós do Go2.

**Caminho**: [launch/robot_interface.launch.py](launch/robot_interface.launch.py)

**Descrição**: Inicia os quatro nós de ciclo de vida para integração completa do Go2. Configura e ativa nós automaticamente (handlers de ciclo de vida comentados, mas disponíveis).

#### O que é iniciado

1. **robot_read_node**: Publicador de dados de sensores
2. **lifecycle_robot_write_node**: Executor de comandos e monitor de status do robô
3. **controller_commands_node**: Tradutor de comandos do joystick
4. **robot_video_stream**: Publicador de stream de câmera

#### Argumentos de launch

Nenhum - configuração lida do [robot_config.yaml](../../../../robot_config.yaml)

#### Fonte de configuração

```yaml
robot_configuration:
  robot_name: "go2_robot"          # Namespace para todos os nós
  network_interface: "eth0"         # Interface de rede para comunicação
```

#### Uso

```bash
# Iniciar interface de hardware Go2
ros2 launch go2_pkg robot_interface.launch.py

# Verificar nós em execução
ros2 node list | grep go2

# Verificar estados de ciclo de vida
ros2 lifecycle list robot_read_node
ros2 lifecycle get robot_read_node
```

**Nota**: Este launch file é incluído automaticamente por `bot_bringup` quando `robot_model: "go2"` em `robot_config.yaml`.

## Arquivos de configuração

### nav2_params.yaml

Parâmetros Navigation2 ajustados especificamente para a cinemática e dinâmica do quadrúpede Unitree Go2.

**Caminho**: [config/nav2_params.yaml](config/nav2_params.yaml)

**Descrição**: Ajustes específicos do robô para a stack Nav2 incluindo limites de velocidade, limites de aceleração, footprint e parâmetros de controlador otimizados para locomoção quadrúpede.

Este arquivo é carregado automaticamente por `bot_navigation` quando Go2 é selecionado.

### camera_config.yaml

Parâmetros de calibração de câmera para as câmeras onboard do Go2.

**Caminho**: [config/camera_config.yaml](config/camera_config.yaml)

**Descrição**: Configuração para câmeras Intel RealSense montadas no Go2. Define tipos de câmera, números de série, posições de montagem e transformações TF para SLAM e percepção com uma ou múltiplas câmeras.

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
  - Exemplo: câmera traseira com `yaw: 3.14159` (π rad = 180°) aponta para trás

## Arquivos de descrição do robô

### Arquivos XACRO

Localizados no diretório [xacro/](xacro/):

- **robot.xacro**: Descrição principal do robô Go2 incluindo corpo e pernas
- **leg.xacro**: Cadeia cinemática da perna (quadril, coxa, canela)
- **const.xacro**: Constantes e medidas (comprimentos, massas, inércias)
- **materials.xacro**: Definições de materiais visuais para RViz

### Malhas

Localizadas no diretório [meshes/](meshes/):

Malhas visuais e de colisão em formato DAE/STL:
- `trunk.dae` - Corpo do robô
- `hip.dae` - Junta do quadril
- `thigh.dae` / `thigh_mirror.dae` - Links de coxa
- `calf.dae` / `calf_mirror.dae` - Links de canela
- `foot.dae` - Link do pé

Esses arquivos são usados pelo `bot_description` para visualização no RViz e pelo Nav2 para detecção de colisão.

## Transformações (TF)

### TF Broadcasters

O robot_read_node transmite transformações com base na odometria:

| Frame pai | Frame filho | Fonte | Taxa |
|--------------|-------------|--------|------|
| `odom` | `base_link` | Integração de odometria | 50 Hz |

A árvore cinemática completa é publicada pelo `robot_state_publisher` (de `bot_description`) usando os estados de juntas deste pacote.

## Integração com o sistema BotBrain

### Carregamento automático

Este pacote é carregado automaticamente por `bot_bringup` quando configurado:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "go2"  # Dispara o carregamento de go2_pkg
```

O sistema de bringup:
1. Lê `robot_model: "go2"`
2. Constrói o nome do pacote: `go2_pkg`
3. Inclui `go2_pkg/launch/robot_interface.launch.py`
4. Carrega a descrição de `go2_pkg/xacro/robot.xacro`


## Uso

### Teste standalone

Teste a interface Go2 sem o sistema completo:

```bash
# Source do workspace
source install/setup.bash

# Iniciar apenas a interface Go2
ros2 launch go2_pkg robot_interface.launch.py

# Em outro terminal, verifique os nós
ros2 node list

# Saída esperada:
# /robot_name/robot_read_node
# /robot_name/lifecycle_robot_write_node
# /robot_name/controller_commands_node
# /robot_name/robot_video_stream
```

## Estrutura de diretórios

```
go2_pkg/
├── launch/
│   └── robot_interface.launch.py     # Lançador principal de interface de hardware
│
├── scripts/
│   ├── go2_read.py                   # Nó publicador de dados de sensores
│   ├── go2_write.py                  # Nó executor de comandos
│   ├── go2_controller_commands.py    # Nó tradutor de controle
│   └── go2_video_stream.py           # Nó publicador de vídeo
│
├── config/
│   ├── nav2_params.yaml              # Parâmetros de navegação para Go2
│   └── camera_config.yaml            # Calibração de câmera
│
├── xacro/
│   ├── robot.xacro                   # Descrição principal do robô
│   ├── leg.xacro                     # Cadeia cinemática da perna
│   ├── const.xacro                   # Constantes e medidas
│   └── materials.xacro               # Materiais visuais
│
├── meshes/
│   ├── trunk.dae                     # Malha do corpo
│   ├── hip.dae                       # Malhas da junta do quadril
│   ├── thigh.dae, thigh_mirror.dae   # Malhas da coxa
│   ├── calf.dae, calf_mirror.dae     # Malhas da canela
│   └── foot.dae                      # Malha do pé
│
├── maps/
│   └── [environment maps]            # Mapas pré-construídos para Go2
│
├── go2_pkg/
│   └── tools/
│       └── go2.py                    # Funções utilitárias
│
├── go2_setup.bash                    # Script de setup do ambiente
├── CMakeLists.txt                    # Configuração de build
├── package.xml                       # Manifesto do pacote
└── README.md                         # Este arquivo
```


---

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
