<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_description - Modelo do Robô e Publicação de Transformações

Pacote de descrição do robô que fornece modelos URDF/XACRO e publicação de transformações para o sistema de robôs BotBrain. Este pacote gerencia o modelo cinemático do robô, as malhas de visualização e publica o estado do robô na árvore TF para navegação e controle.

## Objetivo

O pacote `bot_description` fornece a descrição mecânica e visual do robô para o ecossistema ROS2. Ele gerencia:
- Definições de modelos URDF/XACRO do robô
- Malhas 3D de visualização para o RViz
- Publicação do estado do robô (juntas para transformações TF)
- Transformações estáticas entre componentes do robô
- Integração do modelo de hardware do robô com o módulo de interface BotBrain

## Arquivos do pacote

### Arquivos de launch

#### `launch/description.launch.py`
Lançador principal de descrição que orquestra o modelo completo do robô:

**O que inicia:**
1. **robot_description.launch.py**: Modelo de hardware do robô (ex.: Unitree Go2 quadrúpede)
2. **botbrain_description.launch.py**: Modelo do módulo de interface BotBrain (câmeras, sensores)
3. **static_transform_publisher**: Liga a base do robô ao módulo BotBrain (`interface_link` para `botbrain_base`)

**Configuração:**
- Lê `robot_config.yaml` na raiz do workspace
- Usa o campo `robot_name` para namespace e prefixos de frames TF

**Uso:**
```bash
ros2 launch bot_description description.launch.py
```

Normalmente isso é iniciado automaticamente pelo `bot_bringup` durante o startup do sistema.

#### `launch/robot_description.launch.py`
Publica o modelo URDF de hardware do robô e o estado do robô:

**Nós:**
- **robot_state_publisher**: Converte URDF em transformações TF
- Publica estados de junta do robô no tópico `/robot_description`
- Fornece cinemática direta para a base do robô

**Fonte do modelo:**
- Carrega o arquivo XACRO para a plataforma específica do robô
- Suporta diferentes modelos de robô (Go2, Tita, etc.)

#### `launch/botbrain_description.launch.py`
Publica o modelo URDF do módulo BotBrain:

**Nós:**
- **robot_state_publisher**: Publica transformações do módulo BotBrain
- Inclui suportes de câmera, posições de sensores e links de interface
- Fornece geometria de montagem para sensores de percepção

**Integração:**
- O módulo BotBrain é ligado ao hardware do robô via transformação estática
- Transformação publicada de `interface_link` para `botbrain_base`

### Arquivos XACRO

Localizados no diretório `xacro/`:

#### `xacro/botbrain.xacro`
Modelo XACRO que define o módulo de interface BotBrain:

**Inclui:**
- Link de montagem da interface
- Posições de montagem de câmeras (D435i, D455)
- Pontos de montagem de sensores
- Geometria do módulo e malhas de colisão

**Parâmetros:**
- Offsets de montagem configuráveis
- Posicionamento e orientação de câmeras
- Definições de frames de sensores

### Malhas

Localizadas no diretório `meshes/`:

**Conteúdo:**
- Malhas 3D de visualização para o RViz
- Arquivos de malha em formato STL ou DAE
- Geometria visual do robô e do módulo
- Geometria de colisão para planejamento de movimento

**Uso:**
- Referenciadas pelos arquivos URDF/XACRO
- Carregadas pelo RViz para visualização
- Usadas por sistemas de detecção de colisão

## Tópicos

### Publicados
- `/{namespace}/robot_description` (std_msgs/String) - URDF completo do robô
- `/tf_static` (tf2_msgs/TFMessage) - Transformações estáticas
- `/tf` (tf2_msgs/TFMessage) - Transformações dinâmicas (de estados de junta)

### Assinados
- `/{namespace}/joint_states` (sensor_msgs/JointState) - Posições das juntas do robô (se disponível)

## Estrutura de diretórios

```
bot_description/
├── launch/
│   ├── description.launch.py           # Lançador principal
│   ├── robot_description.launch.py     # Modelo de hardware do robô
│   └── botbrain_description.launch.py  # Modelo do módulo BotBrain
├── xacro/
│   └── botbrain.xacro                  # Modelo XACRO do BotBrain
├── meshes/
│   └── [arquivos de malha]             # Malhas 3D de visualização
├── CMakeLists.txt                      # Configuração de build
└── package.xml                         # Manifesto do pacote
```

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
