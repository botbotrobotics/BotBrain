<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# k1_pkg

**Pacote de interface de hardware do robô humanoide Booster Robotics K1**

O pacote `k1_pkg` fornece a camada de abstração de hardware ROS 2 para o robô humanoide Booster Robotics K1. Ele lida com comunicação bidirecional com o robô, publicação de dados de sensores, execução de comandos, streaming de câmera estéreo e serviços específicos do robô.


## Objetivo do pacote

Este pacote faz a interface com o robô Booster Robotics K1 via tópicos ROS 2, permitindo:

- **Comunicação de hardware**: troca de dados bidirecional em tempo real com o K1 via serviço RPC Booster
- **Integração de sensores**: publicação de odometria, IMU, bateria e câmera estéreo
- **Controle de movimento**: recebimento e execução de comandos de velocidade do twist_mux
- **Troca de modo**: alternância entre modos operacionais (damping, prepare, walking, soccer)

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

Nó de ciclo de vida que lê dados de sensores do robô K1 e publica em tópicos ROS 2.

**Executável**: `k1_read.py`

**Descrição**: Assina tópicos ROS 2 da Booster Robotics, processa dados de estado do robô e publica mensagens padrão ROS 2 de sensores. Fornece odometria, IMU, status de bateria e dados de câmera estéreo.

#### Publishers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/odom` | `nav_msgs/Odometry` | Odometria do robô (posição, yaw) no frame base_link |
| `/imu/data` | `sensor_msgs/Imu` | IMU (orientação, velocidade angular, aceleração linear) |
| `/battery` | `sensor_msgs/BatteryState` | Tensão, corrente e estado de carga da bateria |
| `/rgb/image` | `sensor_msgs/Image` | Imagem RGB estéreo retificada |
| `/depth/image` | `sensor_msgs/Image` | Imagem de profundidade estéreo |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` | Parâmetros de calibração da câmera estéreo |

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/odometer_state` | `booster_interface/Odometer` | Estado de odometria Booster (x, y, theta) |
| `/low_state` | `booster_interface/LowState` | Estado de baixo nível incluindo IMU (RPY, gyro, acc) |
| `/battery_state` | `booster_interface/BatteryState` | Tensão, corrente e estado de carga da bateria |
| `/StereoNetNode/rectified_image` | `sensor_msgs/Image` | Imagem RGB estéreo retificada |
| `/StereoNetNode/stereonet_depth` | `sensor_msgs/Image` | Imagem de profundidade estéreo |
| `/StereoNetNode/stereonet_depth/camera_info` | `sensor_msgs/CameraInfo` | Informações da câmera de profundidade |

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo de tópico (namespace) para dados publicados |

---

### lifecycle_robot_write_node

Nó de ciclo de vida que recebe comandos ROS 2 e envia para o robô K1.

**Executável**: `k1_write.py`

**Descrição**: Assina comandos de velocidade do twist_mux e envia requisições de movimento ao K1 via serviço RPC Booster. Fornece um serviço para troca de modo entre os estados operacionais do robô.

#### Subscribers

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `cmd_vel_out` | `geometry_msgs/Twist` | Comandos de velocidade (linear.x, linear.y, angular.z) |

#### Serviços

| Nome do serviço | Tipo de serviço | Descrição |
|--------------|--------------|-------------|
| `change_mode` | `bot_custom_interfaces/srv/Mode` | Troca modo operacional (damping, prepare, walking, soccer) |

#### Clientes RPC

| Nome do serviço | Tipo de serviço | Descrição |
|--------------|--------------|-------------|
| `booster_rpc_service` | `booster_interface/srv/RpcService` | Interface RPC Booster para comandos de movimento e modo |

#### Mapeamento de modos

| Nome do modo | API ID | Descrição |
|-----------|--------|-------------|
| `damping` | 0 | Modo damping — juntas relaxadas (parada segura) |
| `prepare` | 1 | Modo prepare — robô em posição de prontidão |
| `walking` | 2 | Modo walking — marcha bípede similar à humana |
| `soccer` | 4 | Modo soccer — marcha ágil para futebol |

#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `prefix` | string | `""` | Prefixo de tópico (namespace) para tópicos assinados |

---

## Arquivos de launch

### robot_interface.launch.py

Lançador principal de interface de hardware que inicia todos os nós do K1.

**Caminho**: [launch/robot_interface.launch.py](launch/robot_interface.launch.py)

**Descrição**: Inicia os nós de ciclo de vida para integração completa do K1.

#### O que é iniciado

1. **robot_read_node**: Publicador de dados de sensores
2. **lifecycle_robot_write_node**: Executor de comandos

#### Argumentos de launch

Nenhum - configuração lida do [robot_config.yaml](../../../../robot_config.yaml)

#### Fonte de configuração

```yaml
robot_configuration:
  robot_name: "k1_robot"           # Namespace para todos os nós
  network_interface: "eth0"         # Interface de rede para comunicação
```

#### Uso

```bash
# Iniciar interface de hardware K1
ros2 launch k1_pkg robot_interface.launch.py

# Verificar nós em execução
ros2 node list | grep k1

# Verificar estados de ciclo de vida
ros2 lifecycle list robot_read_node
ros2 lifecycle get robot_read_node
```

**Nota**: Este launch file é incluído automaticamente por `bot_bringup` quando `robot_model: "k1"` em `robot_config.yaml`.

## Arquivos de configuração

### nav2_params.yaml

Parâmetros Navigation2 ajustados para a dinâmica do robô humanoide K1.

**Caminho**: [config/nav2_params.yaml](config/nav2_params.yaml)

### camera_config.yaml

Configuração de câmera para o sistema de câmera estéreo do K1.

**Caminho**: [config/camera_config.yaml](config/camera_config.yaml)

## Arquivos de descrição do robô

### Arquivos URDF

Localizados no diretório [urdf/](urdf/):

- **robot.urdf**: Descrição principal do K1 incluindo tronco, braços e pernas

### Malhas

Localizadas no diretório [meshes/](meshes/):

Malhas visuais e de colisão em formato STL para o corpo humanoide completo:
- `Trunk.STL` / `Trunk_Collision.STL` - Tronco do robô
- `Head_1.STL`, `Head_2.STL` - Componentes da cabeça
- `Left_Hip_Yaw.STL`, `Left_Hip_Roll.STL`, `Left_Hip_Pitch.STL` - Juntas do quadril esquerdo
- `Left_Shank.STL`, `Left_Foot.STL` - Perna e pé esquerdos
- `Left_Arm_1-4.STL` - Links do braço esquerdo
- Malhas equivalentes para o lado direito

Esses arquivos são usados pelo `bot_description` para visualização no RViz e pelo Nav2 para detecção de colisão.

## Transformações (TF)

### TF Broadcasters

O robot_read_node transmite transformações com base na odometria:

| Frame pai | Frame filho | Fonte | Descrição |
|--------------|-------------|--------|-------------|
| `odom` | `base_link` | Integração de odometria | Pose 2D do `/odometer_state` |

## Integração com o sistema BotBrain

### Carregamento automático

Este pacote é carregado automaticamente por `bot_bringup` quando configurado:

```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "k1"  # Dispara o carregamento de k1_pkg
```

O sistema de bringup:
1. Lê `robot_model: "k1"`
2. Constrói o nome do pacote: `k1_pkg`
3. Inclui `k1_pkg/launch/robot_interface.launch.py`
4. Carrega a descrição de `k1_pkg/urdf/robot.urdf`


## Uso

### Teste standalone

Teste a interface K1 sem o sistema completo:

```bash
# Source do workspace
source install/setup.bash

# Iniciar apenas a interface K1
ros2 launch k1_pkg robot_interface.launch.py

# Em outro terminal, verifique os nós
ros2 node list

# Saída esperada:
# /robot_name/robot_read_node
# /robot_name/lifecycle_robot_write_node
```

### Trocar modo do robô

```bash
# Mudar para modo walking
ros2 service call /{namespace}/change_mode bot_custom_interfaces/srv/Mode "{mode: 'walking'}"

# Mudar para damping (parada segura)
ros2 service call /{namespace}/change_mode bot_custom_interfaces/srv/Mode "{mode: 'damping'}"
```

## Estrutura de diretórios

```
k1_pkg/
├── launch/
│   └── robot_interface.launch.py     # Lançador principal de interface de hardware
│
├── scripts/
│   ├── k1_read.py                    # Nó publicador de dados de sensores
│   └── k1_write.py                   # Nó executor de comandos
│
├── config/
│   ├── nav2_params.yaml              # Parâmetros de navegação para K1
│   └── camera_config.yaml            # Configuração de câmera
│
├── urdf/
│   └── robot.urdf                    # Descrição principal do robô
│
├── meshes/
│   ├── Trunk.STL                     # Malha do tronco
│   ├── Head_1.STL, Head_2.STL        # Malhas da cabeça
│   ├── Left_Hip_*.STL                # Malhas das juntas do quadril esquerdo
│   ├── Left_Shank.STL                # Malha da perna esquerda
│   ├── Left_Foot.STL                 # Malha do pé esquerdo
│   ├── Left_Arm_*.STL                # Malhas do braço esquerdo
│   └── [Equivalentes lado direito]   # Malhas espelhadas para o lado direito
│
├── maps/
│   └── [environment maps]            # Mapas pré-construídos para K1
│
├── k1_pkg/
│   └── tools/
│
├── k1_setup.bash                     # Script de setup do ambiente
├── CMakeLists.txt                    # Configuração de build
├── package.xml                       # Manifesto do pacote
└── README.md                         # Este arquivo
```


---

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
