<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

<p align="center">
  Um cérebro, qualquer robô.
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-Website-000?logo=vercel&logoColor=white" alt="Website"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
  <a href="https://huggingface.co/botbot-ai"><img src="https://img.shields.io/badge/-Hugging%20Face-FFD54F?logo=huggingface&logoColor=black" alt="Hugging Face"></a>
</p>

<h1 align="center">Workspace ROS2 BotBrain</h1>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="License: MIT">
  <img src="https://img.shields.io/badge/Platform-Ubuntu_22.04-orange" alt="Ubuntu 22.04">
</p>

## Visão geral

O **BotBrain Workspace** é um framework ROS2 modular e open-source para controle autônomo, navegação e localização de robôs. Projetado com uma arquitetura agnóstica de robôs, permite desenvolvimento e implantação rápidos de aplicações avançadas de robótica em múltiplas plataformas.

**Principais recursos:**
- 🤖 **Suporte multi-robô**: base de código única para Go2, Tita, G1 e robôs customizados
- 🗺️ **SLAM visual**: localização baseada em RTABMap com suporte a duas câmeras
- 🎮 **Múltiplos modos de controle**: joystick, interface web e navegação autônoma
- 👁️ **Visão com IA**: detecção de objetos YOLOv8/v11
- 🐳 **Pronto para Docker**: implantação em containers com aceleração por GPU
- 🔄 **Gerenciamento de ciclo de vida**: orquestração robusta de nós e recuperação de falhas


## Sumário

- [Requisitos de hardware](#requisitos-de-hardware)
- [Início rápido](#início-rápido)
- [Estrutura do repositório](#estrutura-do-repositório)
- [Criando um pacote de robô customizado](#criando-um-pacote-de-robô-customizado)
- [Visão geral dos pacotes](#visão-geral-dos-pacotes)
- [Serviços Docker](#serviços-docker)
- [Configuração](#configuração)

## Requisitos de hardware

### Plataformas de robô suportadas
- **Unitree Go2**
- **Unitree G1**
- **Tita**
- **Robôs customizados** - Siga o [Guia de Pacote de Robô Customizado](#criando-um-pacote-de-robô-customizado)

### Hardware necessário
- **Plataforma de robô**: um dos robôs suportados acima
- **Computador onboard**:
  - Nvidia Jetson Orin Series ou mais recente
- **Sensores**:
  - Câmeras Intel RealSense (para SLAM visual)
  - LiDAR (para SLAM baseado em LiDAR)
- **Rede**:
  - Conexão Ethernet com o robô
  - Adaptador Wi-Fi (para controle remoto)

### Hardware opcional
- **Controle de game**: para teleoperação

## Início rápido

### Iniciar com Docker Compose

Para implantação em containers:

```bash
# Iniciar todos os serviços
docker compose up -d

# Iniciar serviços específicos
docker compose up -d state_machine bringup localization navigation

# Ver logs
docker compose logs -f bringup

# Parar serviços
docker compose down
```

### Verificar se o sistema está rodando

```bash
# Checar nós ativos
ros2 node list

# Checar tópicos
ros2 topic list
```

### Container de desenvolvimento

Se você quiser usar a mesma imagem Docker para desenvolvimento, sem criar um novo serviço, é possível rodar um container de dev iterativo:

```bash
# Iniciar o container de dev
cd botbrain_ws
docker compose up dev -d

# Abrir um terminal iterativo
docker compose exec dev bash
```

Quando o terminal iterativo abrir, você pode usá-lo para criar, compilar e executar novas funcionalidades que ainda não estão integradas aos serviços Docker.

## Estrutura do repositório

```
botbrain_ws/
├── README.md                          # Este arquivo
├── LICENSE                            # Licença MIT
│
├── robot_config.yaml                  # Arquivo principal de configuração
├── install.sh                         # Script de instalação automatizada
├── robot_select.sh                    # Helper de seleção de robô
│
├── docker-compose.yaml                # Definição de serviços Docker
├── botbrain.service                   # Serviço systemd de autostart
├── cyclonedds_config.xml              # Configuração do middleware DDS
│
└── src/                               # Pacotes ROS 2
    │
    ├── Core System Packages
    │   ├── bot_bringup/               # Coordenação principal de launch e twist mux
    │   ├── bot_custom_interfaces/     # Mensagens, serviços e ações customizados
    │   └── bot_state_machine/         # Gerenciamento de ciclo de vida e estado
    │
    ├── Robot Model & Visualization
    │   └── bot_description/           # Modelos URDF/XACRO e robot_state_publisher
    │
    ├── Navigation & Localization
    │   ├── bot_localization/          # SLAM RTABMap (visual e LiDAR)
    │   └── bot_navigation/            # Stack de navegação Nav2
    │
    ├── Perception & Control
    │   ├── bot_yolo/                  # Detecção de objetos YOLOv8/v11
    │   └── joystick-bot/              # Interface de controle (gamepad)
    │
    ├── IA & Monitoramento
    │   ├── bot_jetson_stats/          # Monitoramento de hardware Jetson
    │   └── bot_rosa/                  # Controle por linguagem natural ROSA AI
    │
    └── Robot-Specific Packages
        ├── g1_pkg/                    # Interface de hardware Unitree G1
        ├── go2_pkg/                   # Interface de hardware Unitree Go2
        ├── tita_pkg/                  # Interface de hardware Tita
        └── your_robot_pkg/            # Seu robô customizado (veja o guia abaixo)
```

## Criando um pacote de robô customizado

Para adicionar suporte a uma nova plataforma de robô, siga este guia usando [go2_pkg](src/go2_pkg) como referência.

**Nota**: O pacote go2_pkg se comunica com o robô Unitree Go2 via tópicos ROS 2 (assinando tópicos ROS 2 nativos da Unitree e republicando em formato BotBrain). Seu pacote de robô customizado pode usar comunicação por tópicos, APIs diretas de hardware ou interfaces SDK, dependendo da arquitetura do seu robô. A ideia é criar uma interface padrão entre os pacotes do botbrain_ws e o robô.

### Estrutura de pacote obrigatória

Seu pacote de robô customizado deve seguir esta convenção de nomes para funcionar com todos os pacotes: `{robot_model}_pkg`

```
{robot_model}_pkg/
├── package.xml                        # Manifesto do pacote ROS 2
├── CMakeLists.txt                     # Configuração de build
├── README.md                          # Documentação do pacote
├── launch/
│   └── robot_interface.launch.py     # OBRIGATÓRIO: launch principal da interface
├── config/
│   └── nav2_params.yaml               # OBRIGATÓRIO: parâmetros de navegação
├── scripts/
│   ├── {robot_model}_read.py                # OBRIGATÓRIO: lê sensores do robô
│   └── {robot_model}_write.py               # OBRIGATÓRIO: envia comandos ao robô
├── {robot_model}_pkg/                 # Diretório do pacote Python
│   └── tools/                         # OPCIONAL: ferramentas do assistente ROSA
│       ├── __init__.py                # Inicialização vazia do pacote
│       └── {robot_model}.py           # Ferramentas LangChain para ROSA
├── xacro/
│   └── robot.xacro                    # OBRIGATÓRIO: modelo URDF do robô
└── meshes/
    └── *.dae, *.stl                   # Malhas visuais e de colisão
```

### Guia passo a passo

#### 1. Criar um novo pacote ROS 2

Crie a estrutura do pacote usando as ferramentas ROS 2 (se você não tiver ROS 2 instalado no host, isso pode ser feito via dev container):

```bash
cd src/
ros2 pkg create {robot_model}_pkg --build-type ament_cmake --dependencies rclcpp rclpy 
cd {robot_model}_pkg
```

Crie os diretórios necessários:
```bash
mkdir -p launch config scripts xacro meshes maps
```

#### 2. Configurar package.xml

Edite `package.xml` e adicione as dependências necessárias:
- Adicione `bot_custom_interfaces` como dependência
- Atualize nome do pacote, versão, descrição e informações de mantenedor
- Garanta que todas as dependências de mensagens de sensores estejam incluídas

#### 3. Configurar CMakeLists.txt

Atualize a configuração de build para instalar todos os recursos do pacote:
- Instalar diretório de launch files
- Instalar diretório de config
- Instalar scripts como executáveis
- Instalar diretórios xacro, urdf e meshes
- Usar `ament_python_install_package()` para módulos Python

#### 4. Criar o launch file da interface de hardware

**CRÍTICO**: Crie `launch/robot_interface.launch.py` (nome exato obrigatório)

Este launch file deve:
- Ler `robot_config.yaml` na raiz do workspace
- Extrair `robot_name` para configuração de namespace
- Iniciar nós de ciclo de vida para leitura e escrita de hardware
- Usar `LifecycleNode` de `launch_ros.actions`
- Aplicar o namespace correto a todos os nós

Referência: veja [go2_pkg/launch/robot_interface.launch.py](src/go2_pkg/launch/robot_interface.launch.py).

#### 5. Implementar nós de interface de hardware

**Crie `scripts/{robot_model}_read.py`** - Lê dados de sensores e publica em ROS 2:

Este nó de ciclo de vida deve:
- Inicializar como `LifecycleNode` com nome `robot_read_node`
- Implementar callbacks de ciclo de vida: `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
- Em `on_configure`: criar publishers para odometria, IMU, joint states e bateria
- Em `on_activate`: iniciar loop de leitura de dados (tipicamente 50Hz) do hardware/tópicos
- Processar dados de sensores e publicar em tópicos ROS 2
- Em `on_deactivate`: parar publicação de dados mantendo conexões
- Em `on_cleanup`: fechar conexões de hardware e liberar recursos

Referência: veja [go2_pkg/scripts/go2_read.py](src/go2_pkg/scripts/go2_read.py).

**Crie `scripts/{robot_model}_write.py`** - Recebe comandos e envia ao robô:

Este nó de ciclo de vida deve:
- Inicializar como `LifecycleNode` com nome `robot_write_node`
- Em `on_configure`: criar subscriber para `cmd_vel_out` e estabelecer comunicação com o robô
- Implementar callback para receber comandos e encaminhar ao hardware
- Em `on_deactivate`: enviar comando de parada (velocidade zero) ao robô
- Em `on_cleanup`: fechar conexões de hardware e liberar recursos
- Opcional: implementar serviços específicos do robô (troca de modo, controle de gait, etc.)

Referência: veja [go2_pkg/scripts/go2_write.py](src/go2_pkg/scripts/go2_write.py).

#### 6. Criar parâmetros de navegação

Crie `config/nav2_params.yaml` com as especificações do seu robô. Veja a [Documentação Nav2](https://docs.nav2.org/) como referência.

Você precisará adicionar um curinga nas seções de configuração de nós. Veja [go2_pkg/config/nav2_params.yaml](src/go2_pkg/config/nav2_params.yaml).

#### 7. Criar descrição do robô (XACRO)

Crie `xacro/robot.xacro` com o modelo URDF do seu robô:

Seu arquivo XACRO deve definir:
- `base_link` como o link principal do corpo do robô
- `interface_link` como a parte de interface entre robô e BotBrain
- Todas as juntas e links do robô (pernas, braços, etc.)
- Links de sensores (câmeras, LiDAR, IMU)
- Malhas visuais para visualização no RViz
- Malhas de colisão para navegação
- Limites e dinâmica de juntas
- Propriedades inerciais

Referência: veja [go2_pkg/xacro/robot.xacro](src/go2_pkg/xacro/robot.xacro).

#### 8. Configurar o workspace

Atualize o `robot_config.yaml` do workspace (pode ser feito a partir do install.sh):

```yaml
robot_configuration:
  robot_name: "my_robot"               # Namespace para todos os tópicos
  robot_model: "your_robot"            # Deve corresponder ao nome do pacote sem "_pkg"
  description_file_type: "xacro"       # "xacro" ou "urdf"
  network_interface: "eth0"            # Interface de rede para comunicação
```

**IMPORTANTE**: O campo `robot_model` deve corresponder ao nome do pacote **sem** o sufixo `_pkg`:
- Nome do pacote: `your_robot_pkg`
- robot_model: `your_robot`

#### 9. Compilar e testar

```bash
# Compilar seu pacote
cd ~/botbrain_workspace/BotBrain/botbrain_ws
colcon build --packages-select your_robot_pkg

# Source do workspace
source install/setup.bash

# Testar interface de hardware
ros2 launch your_robot_pkg robot_interface.launch.py

# Iniciar com o sistema completo
ros2 launch bot_bringup bringup.launch.py
```

Você pode compilar e testar o novo pacote usando um dev container.

#### 10. Criar ferramentas ROSA (opcional)

**ROSA** (Robot Operating System Assistant) é um assistente de IA que permite controle do robô em linguagem natural. Ao criar ferramentas para o ROSA, usuários podem interagir com o robô por comandos conversacionais.

**Crie a estrutura do diretório de ferramentas:**

```bash
mkdir -p {robot_model}_pkg/tools
touch {robot_model}_pkg/tools/__init__.py
touch {robot_model}_pkg/tools/{robot_model}.py
```

### Pontos de integração do pacote

O sistema BotBrain encontrará e usará seu pacote automaticamente com base nestas convenções:

1. **Nome do pacote**: formato `{robot_model}_pkg`
2. **Launch file**: `launch/robot_interface.launch.py` (nome exato obrigatório)
3. **Configuração de navegação**: `config/nav2_params.yaml` (usado por bot_navigation)
4. **Arquivos de descrição**: `xacro/robot.xacro` ou `urdf/robot.urdf` (usado por bot_description)

### Tópicos obrigatórios que seu pacote deve fornecer

Para integração completa, sua interface de hardware deve publicar:

| Tópico | Tipo de mensagem | Descrição | Frequência |
|-------|--------------|-------------|-----------|
| `/{namespace}/odom` | nav_msgs/Odometry | Odometria do robô | 50Hz |
| `/{namespace}/imu` | sensor_msgs/Imu | Dados de IMU | 100Hz |
| `/{namespace}/joint_states` | sensor_msgs/JointState | Posições/velocidades das juntas | 50Hz |

E assinar:

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | geometry_msgs/Twist | Comandos de velocidade do twist_mux |


## Visão geral dos pacotes

### Pacotes core do sistema

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [bot_bringup](src/bot_bringup) | Coordenação principal de launch, multiplexador twist e orquestração do sistema | [README](src/bot_bringup/README.md) |
| [bot_state_machine](src/bot_state_machine) | Gerenciamento de ciclo de vida, coordenação de nós e controle de estado do sistema | [README](src/bot_state_machine/README.md) |
| [bot_custom_interfaces](src/bot_custom_interfaces) | Mensagens, serviços e ações ROS 2 customizadas | [README](src/bot_custom_interfaces/README.md) |
| [bot_description](src/bot_description) | Modelos URDF/XACRO e robot_state_publisher | [README](src/bot_description/README.md) |

### Navegação e localização

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [bot_localization](src/bot_localization) | SLAM RTABMap com suporte a mapeamento visual e LiDAR | [README](src/bot_localization/README.md) |
| [bot_navigation](src/bot_navigation) | Stack de navegação Nav2 com configuração agnóstica | [README](src/bot_navigation/README.md) |

### Percepção e controle

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [bot_yolo](src/bot_yolo) | Detecção de objetos YOLOv8/v11 com aceleração TensorRT | [README](src/bot_yolo/README.md) |
| [joystick-bot](src/joystick-bot) | Interface de gamepad com segurança de dead-man switch | [README](src/joystick-bot/README.md) |

### Pacotes específicos de robô

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [go2_pkg](src/go2_pkg) | Interface e descrição do hardware Unitree Go2 | [README](src/go2_pkg/README.md) |
| [tita_pkg](src/tita_pkg) | Interface e descrição do hardware Tita | [README](src/tita_pkg/README.md) |

## Serviços Docker

O workspace inclui múltiplos serviços Docker para implantação em containers:

| Serviço | Descrição | Auto-start | Dependências |
|---------|-------------|------------|--------------|
| `dev` | Container de desenvolvimento (interativo) | Não | - |
| `builder_base` | Compila todos os pacotes do workspace | Não | - |
| `state_machine` | Serviço de gerenciamento de ciclo de vida | Sim | - |
| `bringup` | Bringup principal do robô | Sim | state_machine |
| `localization` | Localização RTABMap | Sim | bringup |
| `navigation` | Servidores de navegação Nav2 | Sim | localization |
| `rosa` | Serviços de tool calling de IA | Sim | bringup |
| `yolo` | Serviço de detecção de objetos | Sim | bringup |

### Uso do Docker

```bash
# Iniciar todos os serviços
docker compose up -d

# Iniciar serviço específico com dependências
docker compose up -d navigation  # Inicia automaticamente bringup, localization

# Ver logs
docker compose logs -f bringup

# Parar todos os serviços
docker compose down

# Rebuild após mudanças de código
docker compose build
docker compose up -d
```

## Configuração

### Arquivo principal de configuração

O arquivo [robot_config.yaml](robot_config.yaml) é o ponto central de configuração:

```yaml
robot_configuration:

  # Identificador do robô - usado como namespace para todos os tópicos
  robot_name: ""                    # Exemplo: "go2_robot1", "tita_lab"

  # Tipo de robô - determina qual pacote de hardware iniciar
  robot_model: "go2"                # Opções: "go2", "tita", "your_robot"

  # Formato do arquivo de descrição
  description_file_type: "xacro"    # Opções: "xacro", "urdf"

  # Interface de rede para comunicação ROS2
  network_interface: "eno1"         # Exemplo: "eth0", "wlan0", "eno1"

  # Tita: namespace para comunicação com o robô
  tita_namespace: "tita3036731"     # Usado apenas quando robot_model: "tita"

  # OpenAI API Key para recursos de IA (opcional)
  openai_api_key: ""                # Necessário para o assistente ROSA

  # Configuração de Wi-Fi (opcional)
  wifi_interface: ""                # Nome da interface Wi-Fi (ex.: "wlan0")
  wifi_ssid: ""                     # SSID da rede Wi-Fi
  wifi_password: ""                 # Senha da rede Wi-Fi
```

### Configuração de rede

O workspace usa CycloneDDS para comunicação ROS 2. Configuração em [cyclonedds_config.xml](cyclonedds_config.xml).

Defina a interface de rede em [robot_config.yaml](robot_config.yaml) para corresponder à sua conexão de hardware.

### Serviço de autostart systemd

O arquivo [botbrain.service](botbrain.service) habilita inicialização automática no boot:

```bash
# Instalar serviço (feito pelo install.sh)
sudo cp botbrain.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable botbrain.service

# Controle manual
sudo systemctl start botbrain.service   # Iniciar agora
sudo systemctl stop botbrain.service    # Parar
sudo systemctl status botbrain.service  # Ver status

# Ver logs
journalctl -u botbrain.service -f
```

### Adicionando suporte para novos robôs

Veja o guia [Criando um pacote de robô customizado](#criando-um-pacote-de-robô-customizado). Contribuições que adicionam suporte a novas plataformas são muito bem-vindas!

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
