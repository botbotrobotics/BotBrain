<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_localization - SLAM Visual e Sistema de Localização

Pacote de SLAM visual e localização para navegação autônoma de robôs. Este pacote fornece SLAM visual baseado em RTAB-Map usando uma ou duas câmeras Intel RealSense, ou Lidar para localização robusta do robô, mapeamento e suporte à navegação autônoma.

## Objetivo

O pacote `bot_localization` serve como o sistema central de localização para a plataforma de robôs BotBrain. Ele gerencia:
- SLAM visual (Localização e Mapeamento Simultâneos) usando RTAB-Map
- Integração de câmera RealSense única/dupla ou Lidar
- Modos de construção de mapas e localização
- Gerenciamento da cadeia de transformações (map → odom → base_link)
- Geração de grade de ocupação 2D para navegação

## Arquivos do pacote

### Arquivos de launch

---

#### `launch/localization.launch.py`

Lançador principal do sistema de localização que orquestra todos os componentes de SLAM e percepção.

**Descrição**: arquivo launch mestre que inicializa o sistema completo de localização baseado em visão. Este arquivo é o ponto de entrada para toda a funcionalidade de localização e inclui vários sub-launch files.

**O que é iniciado**:

Este launch file inclui os seguintes sub-launch files em sequência:

1. **realsense.launch.py**: Drivers de câmera e processamento de profundidade
2. **rtabmap.launch.py**: Sistema de SLAM RTAB-Map
3. **rtab_manager.launch.py**: Gerenciamento de serviços do RTAB-Map
4. **map_odom.launch.py**: Publicador de transformações de mapa para odometria

**Argumentos de launch**:

Nenhum - configuração lida do `robot_config.yaml` do workspace e de `{robot_model}_pkg/config/camera_config.yaml`

**Uso**:
```bash
# Iniciar o sistema completo de localização
ros2 launch bot_localization localization.launch.py

```

---

#### `launch/realsense.launch.py`

Drivers de câmera Intel RealSense e lançador de conversão depth-to-laser.

**Descrição**: configura dinamicamente e inicia nós de câmera Intel RealSense com base na configuração de câmera. Cria drivers de câmera, publicadores estáticos de TF e conversores de depth-to-laserscan para detecção de obstáculos.

**O que é iniciado**:

O launch file lê `{robot_model}_pkg/config/camera_config.yaml` e inicia:

**Para cada câmera configurada (frente/traseira)**:
- **Nó de câmera RealSense** (LifecycleNode): `realsense2_camera_node`
- **Publicador TF estático**: Publica transformação câmera-para-base
  - Baseado nos parâmetros `tf` em camera_config.yaml (x, y, z, roll, pitch, yaw)
- **Nó depth_to_laserscan**: Converte imagem de profundidade em laser scan 2D

**Nós adicionais**:
- **realsense_compressed_node** (LifecycleNode): Comprime tópicos RealSense para otimização de banda

**Fonte de configuração**:

- Namespace do robô a partir de `robot_config.yaml`
- Configuração de câmera de `{robot_model}_pkg/config/camera_config.yaml`
  - Tipos de câmera (d435i, d455, etc.)
  - Números de série para identificação do dispositivo
  - Transformações TF (posição e orientação)

**Exemplo de configuração**:

```yaml
camera_configuration:
  front:
    type: "d435i"
    serial_number: "123456789"
    parent_frame: "botbrain_base"
    child_frame: "front_camera_link"
    tf: {x: 0.08, y: 0.0175, z: 0.043, roll: 0, pitch: 0, yaw: 0}
  back:
    type: "d435i"
    serial_number: "987654321"
    parent_frame: "botbrain_base"
    child_frame: "back_camera_link"
    tf: {x: -0.08, y: -0.0175, z: 0.043, roll: 0, pitch: 0, yaw: 3.14159}
```

---

#### `launch/map_odom.launch.py`

Publicador de pose map-to-base_link para monitoramento de localização.

**Descrição**: inicia um nó de ciclo de vida que lê a transformação TF de map para base_link e publica como uma mensagem PoseStamped em um tópico.

**O que é iniciado**:

- **map_odom_node** (LifecycleNode): `map_odom.py`
  - Lê a transformação TF de `map` para `base_link` a 10Hz
  - Publica a transformação como mensagem PoseStamped no tópico `titan/map_odom`
  - Útil para monitorar a posição do robô no frame do mapa

**Parâmetros**:

| Parâmetro | Tipo | Descrição |
|-----------|------|-------------|
| `prefix` | string | Prefixo de namespace para frames TF (ex.: "tita/") |

**Tópicos publicados**:

- `map_odom` (PoseStamped): Pose do robô no frame do mapa (posição e orientação)

**Nota**: Este nó NÃO publica transformações TF. Ele apenas publica mensagens de pose lendo dados TF existentes do RTAB-Map.

---

#### `launch/rtabmap.launch.py`

Lançador dinâmico do RTAB-Map que seleciona a configuração de SLAM apropriada.

**Descrição**: lançador inteligente que lê a configuração de câmera/sensor e inclui o arquivo RTAB-Map correto com base no modelo do robô e sensores disponíveis.

**Lógica de seleção**:

O launch file determina qual configuração RTAB-Map usar:

1. **Se robot_model == "g1"**:
   - Inclui `rtabmap_lidar.launch.py`
   - SLAM baseado em LiDAR para o robô humanoide G1

2. **Se o robô tem câmeras**:
   - **0 câmeras**: Nenhum RTAB-Map iniciado
   - **1 câmera** (frente OU traseira): Inclui `rtabmap_single_camera.launch.py`
   - **2 câmeras** (frente E traseira): Inclui `rtabmap_double_camera.launch.py`

**Fonte de configuração**:

- Lê `robot_config.yaml` para `robot_model` e `robot_name`
- Lê `{robot_model}_pkg/config/camera_config.yaml` para contar câmeras
- Conta câmeras com base no campo `type` nas seções `front` e `back`

**Launch files incluídos**:

**rtabmap_single_camera.launch.py**:
- RTAB-Map configurado para uma câmera RGB-D
- Assinatura direta nos tópicos de câmera (sem sincronização)
- Tópicos: `front_camera/color/image_raw`, `front_camera/aligned_depth_to_color/image_raw`

**rtabmap_double_camera.launch.py**:
- RTAB-Map configurado para duas câmeras RGB-D
- Inclui dois nós `rgbd_sync` para câmeras front e back
- RTAB-Map assina `rgbd_image0` e `rgbd_image1` sincronizados
- Parâmetros: `rgbd_cameras: 2`, `subscribe_rgbd: True`

**rtabmap_lidar.launch.py** (apenas robô G1):
- RTAB-Map configurado para SLAM com LiDAR 3D
- Inclui nó `lidar_deskewing` para compensação de movimento
- Registro ICP com correspondência ponto-plano
- Assina o tópico `/{namespace}/pointcloud`

**Configuração comum do RTAB-Map**:

Todas as variantes do RTAB-Map compartilham estas configurações:

**Modo de localização** (não mapeamento):
- `Mem/IncrementalMemory: False`: Apenas localização, sem criação de novo mapa
- `Mem/InitWMWithAllNodes: True`: Carrega o mapa completo na memória
- `database_path`: `{robot_model}_pkg/maps/rtabmap.db`
- `delete_db_on_start: False`: Preserva o mapa existente

**Parâmetros de SLAM 2D** (baseado em câmera):
- `Reg/Force3DoF: true`: Restringe ao plano do chão (x, y, yaw)
- `Grid/3D: false`: Gera grade de ocupação 2D
- `Grid/RangeMax: 3.0m`: Alcance máximo de detecção de obstáculos
- `Grid/CellSize: 0.10m`: Resolução de grade de 10cm
- `Grid/MaxGroundHeight: 0.05m`: Tolerância do plano do chão
- `Grid/MaxObstacleHeight: 0.8m`: Altura máxima de obstáculo

**Configuração de frames**:
- `frame_id`: `{prefix}base_link`
- `map_frame_id`: `{prefix}map`
- `odom_frame_id`: `{prefix}odom`

---

#### `launch/rtab_manager.launch.py`

Gerenciador de ciclo de vida do RTAB-Map para controle de modos de mapeamento/localização.

**Descrição**: inicia um nó de ciclo de vida que fornece serviços para gerenciar modos de operação do RTAB-Map e estado do banco de dados.

**O que é iniciado**:

- **rtab_manager** (LifecycleNode): `rtab_manager.py`
  - Gerencia transições de ciclo de vida do RTAB-Map
  - Fornece serviços para resetar mapa, alternar modos, salvar/carregar bancos
  - Monitora a saúde do nó RTAB-Map

**Parâmetros**:

| Parâmetro | Tipo | Descrição |
|-----------|------|-------------|
| `robot_model` | string | Nome do modelo do robô para resolução do caminho do banco |

**Serviços fornecidos**:

Serviços para controlar a operação do RTAB-Map (dependente da implementação):
- Resetar banco de dados
- Alternar entre modos de mapeamento e localização
- Salvar/carregar bancos de mapas
- Disparar otimização de mapa

### Scripts

#### `scripts/map_odom.py`
Nó de ciclo de vida que gerencia publicação de transformação map-to-odom:

**Funcionalidade**:
- Escuta estimativas de pose do RTAB-Map
- Computa a transformação map → odom a partir de map → base_link
- Publica a transformação a 20Hz para navegação suave
- Gerencia transições de ciclo de vida (configure/activate/deactivate)

**Tópicos**:
- Assina: `/{namespace}/rtabmap/localization_pose` (PoseStamped)
- Publica: `/tf` (TFMessage) - transformação map → odom

## Configuração

O pacote lê do `robot_config.yaml` no nível do workspace:

```yaml
robot_configuration:
  robot_name: "my_robot"                    # Namespace para todos os tópicos
  robot_model: "go2"                        # Modelo do robô
  d435i_serial_number: ""       # Serial da câmera traseira
  d455_serial_number: ""        # Serial da câmera frontal
```


## Tópicos

### Publicados
- `/{namespace}/rtabmap/grid_map` (OccupancyGrid) - Mapa 2D para navegação
- `/{namespace}/rtabmap/cloud_map` (PointCloud2) - Mapa 3D de nuvem de pontos
- `/{namespace}/rtabmap/localization_pose` (PoseStamped) - Pose do robô no mapa
- `/{namespace}/rtabmap/mapData` (MapData) - Dados do mapa SLAM
- `/tf` (TFMessage) - transformação map → odom

### Assinados
- `/{namespace}/d455/color/image_raw` (Image) - RGB da câmera frontal
- `/{namespace}/d455/depth/image_rect_raw` (Image) - Profundidade frontal
- `/{namespace}/d435i/color/image_raw` (Image) - RGB da câmera traseira
- `/{namespace}/d435i/depth/image_rect_raw` (Image) - Profundidade traseira
- `/{namespace}/d435i/imu` (Imu) - Dados de IMU da D435i

## Serviços

### Serviços do RTAB-Map
- `/{namespace}/rtabmap/reset_session` - Iniciar nova sessão de mapeamento
- `/{namespace}/rtabmap/save_map` - Salvar mapa atual no banco
- `/{namespace}/rtabmap/load_map` - Carregar mapa do banco
- `/{namespace}/rtabmap/pause` - Pausar processamento do SLAM
- `/{namespace}/rtabmap/resume` - Retomar processamento do SLAM

### Serviços customizados (via bot_custom_interfaces)
- `/{namespace}/start_mapping` - Iniciar modo de mapeamento (cria novo mapa)
- `/{namespace}/stop_mapping` - Parar mapeamento e salvar banco

## Fluxo de mapeamento

### Criando um novo mapa

1. **Iniciar sessão de mapeamento**:
```bash
ros2 launch bot_localization localization.launch.py
ros2 service call /{namespace}/start_mapping bot_custom_interfaces/srv/StartMapping
```

2. **Conduzir o robô**: Teleoperar o robô pelo ambiente
   - Cobrir todas as áreas a serem mapeadas
   - Garantir boa iluminação e recursos visuais
   - Mover lentamente para melhor rastreamento de features

3. **Salvar o mapa**:
```bash
ros2 service call /{namespace}/stop_mapping bot_custom_interfaces/srv/StopMapping
```

4. **Banco salvo**: Banco de mapas salvo no caminho configurado

### Usando um mapa existente para localização

1. **Configurar o launch file**: Defina o caminho do banco para o mapa existente
2. **Iniciar localização**:
```bash
ros2 launch bot_localization localization.launch.py
```
3. **Pose inicial**: O robô pode precisar de dica de pose inicial se estiver longe da origem do mapa
4. **Localização automática**: RTAB-Map localiza automaticamente usando features visuais


## Estrutura de diretórios

```
bot_localization/
├── bot_localization/                          # Diretório principal do pacote
│   ├── bot_localization/                      # Pacote Python
│   │   └── __init__.py                       # Inicialização do pacote
│   ├── launch/
│   │   ├── localization.launch.py            # Lançador principal (orquestra todos os componentes)
│   │   ├── realsense.launch.py               # Drivers de câmera e processamento de profundidade
│   │   ├── map_odom.launch.py                # Nó publicador de pose
│   │   ├── rtabmap.launch.py                 # Lançador dinâmico do RTAB-Map (seletor)
│   │   ├── rtabmap_single_camera.launch.py   # Configuração RTAB-Map de câmera única
│   │   ├── rtabmap_double_camera.launch.py   # Configuração RTAB-Map de câmera dupla
│   │   ├── rtabmap_lidar.launch.py           # Configuração RTAB-Map LiDAR (G1)
│   │   └── rtab_manager.launch.py            # Gerenciador de ciclo de vida do RTAB-Map
│   ├── scripts/
│   │   ├── map_odom.py                       # Publicador de pose map-to-base_link
│   │   ├── rtab_manager.py                   # Nó gerenciador do RTAB-Map
│   │   └── compressed_realsense.py           # Nó de compressão RealSense
│   ├── CMakeLists.txt                        # Configuração de build
│   ├── package.xml                           # Manifesto do pacote
│   └── README.md                             # Este arquivo
├── bot_localization_interfaces/               # Interfaces customizadas de serviço/mensagem
│   ├── srv/                                  # Definições de serviço
│   ├── CMakeLists.txt                        # Configuração de build de interfaces
│   └── package.xml                           # Manifesto do pacote de interfaces
└── README.md                                 # README de topo
```

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
