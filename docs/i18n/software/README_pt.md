<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

<p align="center">
  Um Cérebro, qualquer Robô.
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-Website-000?logo=vercel&logoColor=white" alt="Website"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
  <a href="https://huggingface.co/botbot-ai"><img src="https://img.shields.io/badge/-Hugging%20Face-FFD54F?logo=huggingface&logoColor=black" alt="Hugging Face"></a>
</p>

<h1 align="center">BotBrain ROS2 Workspace</h1>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="Licença: MIT">
  <img src="https://img.shields.io/badge/Platform-Ubuntu_22.04-orange" alt="Ubuntu 22.04">
</p>

<p align="center">
  <a href="../../../botbrain_ws/README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-green" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

> **Nota:** A versão em inglês é a documentação oficial e mais atualizada. Esta tradução pode não refletir as últimas alterações.

## Visão Geral

O **BotBrain Workspace** é um framework ROS2 modular e de código aberto para controle autônomo de robôs, navegação e localização. Projetado com uma arquitetura agnóstica a robôs, ele permite o desenvolvimento e implantação rápidos de aplicações robóticas avançadas em múltiplas plataformas de robôs.

**Principais Características:**
- 🤖 **Suporte Multi-Robô**: Base de código única para Go2, Tita, G1 e robôs personalizados
- 🗺️ **SLAM Visual**: Localização baseada em RTABMap com suporte a câmera dupla
- 🎮 **Múltiplos Modos de Controle**: Joystick, interface web e navegação autônoma
- 👁️ **Visão IA**: Detecção de objetos YOLOv8/v11
- 🐳 **Pronto para Docker**: Implantação containerizada com aceleração GPU
- 🔄 **Gerenciamento de Ciclo de Vida**: Orquestração robusta de nós e recuperação de falhas


## Índice

- [Requisitos de Hardware](#requisitos-de-hardware)
- [Início Rápido](#início-rápido)
- [Estrutura do Repositório](#estrutura-do-repositório)
- [Criando um Pacote de Robô Personalizado](#criando-um-pacote-de-robô-personalizado)
- [Visão Geral dos Pacotes](#visão-geral-dos-pacotes)
- [Serviços Docker](#serviços-docker)
- [Configuração](#configuração)

## Requisitos de Hardware

### Plataformas de Robô Suportadas
- **Unitree Go2**
- **Unitree G1**
- **Tita**
- **Robôs Personalizados** - Siga o [Guia de Pacote de Robô Personalizado](#criando-um-pacote-de-robô-personalizado)

### Hardware Necessário
- **Plataforma de Robô**: Um dos robôs suportados acima
- **Computador de Bordo**:
  - Nvidia Jetson Orin Series ou mais recente
- **Sensores**:
  - Câmeras Intel RealSense (para SLAM visual)
  - LiDAR (para SLAM baseado em LiDAR)
- **Rede**:
  - Conexão Ethernet com o robô
  - Adaptador Wi-Fi (para controle remoto)

### Hardware Opcional
- **Controle de Jogo**: Para teleoperação

## Início Rápido

### Iniciar Usando Docker Compose

Para implantação containerizada:

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

### Verificar se o Sistema Está Rodando

```bash
# Verificar nós ativos
ros2 node list

# Verificar tópicos
ros2 topic list
```

### Container de Desenvolvimento

Se você quiser usar a mesma imagem docker para desenvolvimento, sem criar um novo serviço, é possível executar um container de desenvolvimento interativo:

```bash
# Iniciar o container de desenvolvimento
cd botbrain_ws
docker compose up dev -d

# Iniciar um terminal interativo
docker compose exec dev bash
```

Uma vez que o terminal interativo abrir, você pode usá-lo para criar, compilar e executar novos recursos que ainda não estão integrados com os serviços docker.

## Estrutura do Repositório

```
botbrain_ws/
├── README.md                          # Este arquivo
├── LICENSE                            # Licença MIT
│
├── robot_config.yaml                  # Arquivo de configuração principal
├── install.sh                         # Script de instalação automatizada
├── robot_select.sh                    # Auxiliar de seleção de robô
│
├── docker-compose.yaml                # Definição dos serviços Docker
├── botbrain.service                   # Serviço systemd de autostart
├── cyclonedds_config.xml              # Configuração do middleware DDS
│
└── src/                               # Pacotes ROS 2
    │
    ├── Pacotes do Sistema Central
    │   ├── bot_bringup/               # Lançamento principal & coordenação twist mux
    │   ├── bot_custom_interfaces/     # Mensagens, serviços, ações personalizados
    │   └── bot_state_machine/         # Gerenciamento de ciclo de vida & estados
    │
    ├── Modelo do Robô & Visualização
    │   └── bot_description/           # Modelos URDF/XACRO & robot_state_publisher
    │
    ├── Navegação & Localização
    │   ├── bot_localization/          # SLAM RTABMap (visual & LiDAR)
    │   └── bot_navigation/            # Stack de navegação Nav2
    │
    ├── Percepção & Controle
    │   ├── bot_yolo/                  # Detecção de objetos YOLOv8/v11
    │   └── joystick-bot/              # Interface de controle de jogo
    │
    ├── IA & Monitoramento
    │   ├── bot_jetson_stats/          # Monitoramento de hardware Jetson
    │   └── bot_rosa/                  # Controle por linguagem natural ROSA AI
    │
    └── Pacotes Específicos de Robô
        ├── g1_pkg/                    # Interface de hardware Unitree G1
        ├── go2_pkg/                   # Interface de hardware Unitree Go2
        ├── tita_pkg/                  # Interface de hardware Tita
        └── your_robot_pkg/            # Seu robô personalizado (veja guia abaixo)
```

## Criando um Pacote de Robô Personalizado

Para adicionar suporte para uma nova plataforma de robô, siga este guia usando [go2_pkg](../../../botbrain_ws/src/go2_pkg) como modelo de referência.

**Nota**: O pacote go2_pkg se comunica com o robô Unitree Go2 via tópicos ROS 2 (inscrevendo-se nos tópicos nativos ROS 2 da Unitree e republicando no formato BotBrain). Seu pacote de robô personalizado pode usar comunicação similar baseada em tópicos, APIs de hardware diretas ou interfaces SDK dependendo da arquitetura do seu robô. A ideia é criar uma interface de pacote padrão entre os pacotes botbrain_ws e o robô.

### Estrutura de Pacote Necessária

Seu pacote de robô personalizado deve seguir esta convenção de nomenclatura para funcionar perfeitamente com todos os pacotes: `{robot_model}_pkg`

```
{robot_model}_pkg/
├── package.xml                        # Manifesto do pacote ROS 2
├── CMakeLists.txt                     # Configuração de build
├── README.md                          # Documentação do pacote
├── launch/
│   └── robot_interface.launch.py     # OBRIGATÓRIO: Launcher principal da interface de hardware
├── config/
│   └── nav2_params.yaml               # OBRIGATÓRIO: Parâmetros de navegação
├── scripts/
│   ├── {robot_model}_read.py                # OBRIGATÓRIO: Lê dados de sensores do robô
│   └── {robot_model}_write.py               # OBRIGATÓRIO: Envia comandos para o robô
├── {robot_model}_pkg/                 # Diretório do pacote Python
│   └── tools/                         # OPCIONAL: Ferramentas do assistente ROSA AI
│       ├── __init__.py                # Inicialização vazia do pacote
│       └── {robot_model}.py           # Ferramentas LangChain para integração ROSA
├── xacro/
│   └── robot.xacro                    # OBRIGATÓRIO: Modelo URDF do robô
└── meshes/
    └── *.dae, *.stl                   # Meshes visuais e de colisão
```

### Guia de Criação Passo a Passo

#### 1. Criar Novo Pacote ROS 2

Crie a estrutura do pacote usando ferramentas ROS 2 (se você não tem ros2 instalado no seu sistema host, isso pode ser feito a partir de um container de desenvolvimento):

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
- Atualize nome do pacote, versão, descrição e informações do mantenedor
- Certifique-se de que todas as dependências de mensagens de sensores estão incluídas

#### 3. Configurar CMakeLists.txt

Atualize a configuração de build para instalar todos os recursos do pacote:
- Instale o diretório de arquivos launch
- Instale o diretório de arquivos config
- Instale scripts como executáveis
- Instale diretórios xacro, urdf e meshes
- Use `ament_python_install_package()` para módulos Python

#### 4. Criar Arquivo Launch da Interface de Hardware

**CRÍTICO**: Crie `launch/robot_interface.launch.py` (nome exato obrigatório)

Este arquivo launch deve:
- Ler `robot_config.yaml` da raiz do workspace
- Extrair `robot_name` para configuração de namespace
- Lançar nós de ciclo de vida para leitura e escrita de hardware
- Usar `LifecycleNode` de `launch_ros.actions`
- Aplicar namespace correto a todos os nós

Referência: Veja [go2_pkg/launch/robot_interface.launch.py](../../../botbrain_ws/src/go2_pkg/launch/robot_interface.launch.py) para exemplo completo.

#### 5. Implementar Nós de Interface de Hardware

**Crie `scripts/{robot_model}_read.py`** - Lê dados de sensores e publica no ROS 2:

Este nó de ciclo de vida deve:
- Inicializar como `LifecycleNode` com nome `robot_read_node`
- Implementar callbacks de ciclo de vida: `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
- Em `on_configure`: Criar publishers para odometria, IMU, estados das juntas e bateria
- Em `on_activate`: Iniciar loop de leitura de dados (tipicamente 50Hz) do hardware/tópicos do robô
- Processar dados de sensores do robô e publicar em tópicos ROS 2
- Em `on_deactivate`: Parar publicação de dados mas manter conexões
- Em `on_cleanup`: Fechar conexões de hardware e liberar recursos

Referência: Veja [go2_pkg/scripts/go2_read.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_read.py) para implementação completa.

**Crie `scripts/{robot_model}_write.py`** - Recebe comandos e envia para o robô:

Este nó de ciclo de vida deve:
- Inicializar como `LifecycleNode` com nome `robot_write_node`
- Em `on_configure`: Criar subscriber para tópico `cmd_vel_out` e estabelecer comunicação com o robô
- Implementar callback para receber comandos de velocidade e encaminhar para hardware do robô
- Em `on_deactivate`: Enviar comando de parada (velocidade zero) para o robô
- Em `on_cleanup`: Fechar conexões de hardware e liberar recursos
- Opcionalmente: Implementar serviços específicos do robô (troca de modo, controle de marcha, etc.)

Referência: Veja [go2_pkg/scripts/go2_write.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_write.py) para implementação completa.

#### 6. Criar Parâmetros de Navegação

Crie `config/nav2_params.yaml` com as especificações do seu robô. Veja [Documentação Nav2](https://docs.nav2.org/) como referência.

Você precisará adicionar um curinga às seções de configuração de nós. Veja [go2_pkg/config/nav2_params.yaml](../../../botbrain_ws/src/go2_pkg/config/nav2_params.yaml) para exemplo de configuração completa.

#### 7. Criar Descrição do Robô (XACRO)

Crie `xacro/robot.xacro` com o modelo URDF do seu robô:

Seu arquivo XACRO deve definir:
- `base_link` como o link principal do corpo do robô
- `interface_link` como a parte de interface entre robô e BotBrain
- Todas as juntas e links do robô (pernas, braços, etc.)
- Links de sensores (câmeras, LiDAR, IMU)
- Meshes visuais para visualização RViz
- Meshes de colisão para navegação
- Limites de juntas e dinâmica
- Propriedades inerciais

Referência: Veja [go2_pkg/xacro/robot.xacro](../../../botbrain_ws/src/go2_pkg/xacro/robot.xacro) para descrição completa do robô.

#### 8. Configurar Workspace

Atualize o `robot_config.yaml` do workspace (pode ser feito pelo install.sh):

```yaml
robot_configuration:
  robot_name: "meu_robo"               # Namespace para todos os tópicos
  robot_model: "seu_robo"              # Deve corresponder ao nome do pacote sem "_pkg"
  description_file_type: "xacro"       # "xacro" ou "urdf"
  network_interface: "eth0"            # Interface de rede para comunicação com o robô
```

**IMPORTANTE**: O campo `robot_model` deve corresponder ao nome do seu pacote **sem** o sufixo `_pkg`:
- Nome do pacote: `seu_robo_pkg`
- robot_model: `seu_robo`

#### 9. Compilar e Testar

```bash
# Compile seu pacote
cd ~/botbrain_workspace/BotBrain/botbrain_ws
colcon build --packages-select seu_robo_pkg

# Source o workspace
source install/setup.bash

# Teste sua interface de hardware
ros2 launch seu_robo_pkg robot_interface.launch.py

# Lance com sistema completo
ros2 launch bot_bringup bringup.launch.py
```

Você pode compilar e testar o novo pacote usando um container de desenvolvimento.

#### 10. Criar Ferramentas ROSA (Opcional)

**ROSA** (Robot Operating System Assistant) é um assistente de IA que permite o controle por linguagem natural do seu robô. Ao criar ferramentas para ROSA, os usuários podem interagir com seu robô usando comandos conversacionais.

**Crie a estrutura do diretório de ferramentas:**

```bash
mkdir -p {robot_model}_pkg/tools
touch {robot_model}_pkg/tools/__init__.py
touch {robot_model}_pkg/tools/{robot_model}.py
```

### Pontos de Integração do Pacote

O sistema BotBrain encontrará e usará automaticamente seu pacote com base nestas convenções:

1. **Nomenclatura do Pacote**: formato `{robot_model}_pkg`
2. **Arquivo Launch**: `launch/robot_interface.launch.py` (nome exato obrigatório)
3. **Config de Navegação**: `config/nav2_params.yaml` (usado por bot_navigation)
4. **Arquivos de Descrição**: `xacro/robot.xacro` ou `urdf/robot.urdf` (usado por bot_description)

### Tópicos Obrigatórios que Seu Pacote Deve Fornecer

Para integração completa do sistema, sua interface de hardware deve publicar:

| Tópico | Tipo de Mensagem | Descrição | Frequência |
|-------|--------------|-------------|-----------|
| `/{namespace}/odom` | nav_msgs/Odometry | Odometria do robô | 50Hz |
| `/{namespace}/imu` | sensor_msgs/Imu | Dados IMU | 100Hz |
| `/{namespace}/joint_states` | sensor_msgs/JointState | Posições/velocidades das juntas | 50Hz |

E se inscrever em:

| Tópico | Tipo de Mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | geometry_msgs/Twist | Comandos de velocidade do twist_mux |


## Visão Geral dos Pacotes

### Pacotes do Sistema Central

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [bot_bringup](../../../botbrain_ws/src/bot_bringup) | Coordenação principal de lançamento, multiplexador twist e orquestração do sistema | [README](../../../botbrain_ws/src/bot_bringup/README.md) |
| [bot_state_machine](../../../botbrain_ws/src/bot_state_machine) | Gerenciamento de ciclo de vida, coordenação de nós e controle de estado do sistema | [README](../../../botbrain_ws/src/bot_state_machine/README.md) |
| [bot_custom_interfaces](../../../botbrain_ws/src/bot_custom_interfaces) | Mensagens, serviços e ações ROS 2 personalizados | [README](../../../botbrain_ws/src/bot_custom_interfaces/README.md) |
| [bot_description](../../../botbrain_ws/src/bot_description) | Modelos URDF/XACRO do robô e robot_state_publisher | [README](../../../botbrain_ws/src/bot_description/README.md) |

### Navegação & Localização

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [bot_localization](../../../botbrain_ws/src/bot_localization) | SLAM RTABMap com suporte para mapeamento visual e baseado em LiDAR | [README](../../../botbrain_ws/src/bot_localization/README.md) |
| [bot_navigation](../../../botbrain_ws/src/bot_navigation) | Stack de navegação Nav2 com configuração agnóstica a robôs | [README](../../../botbrain_ws/src/bot_navigation/README.md) |

### Percepção & Controle

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [bot_yolo](../../../botbrain_ws/src/bot_yolo) | Detecção de objetos YOLOv8/v11 com aceleração TensorRT | [README](../../../botbrain_ws/src/bot_yolo/README.md) |
| [joystick-bot](../../../botbrain_ws/src/joystick-bot) | Interface de controle de jogo com interruptor de segurança dead-man | [README](../../../botbrain_ws/src/joystick-bot/README.md) |

### Pacotes Específicos de Robô

| Pacote | Descrição | Documentação |
|---------|-------------|---------------|
| [go2_pkg](../../../botbrain_ws/src/go2_pkg) | Interface de hardware e descrição do Unitree Go2 quadrúpede | [README](../../../botbrain_ws/src/go2_pkg/README.md) |
| [tita_pkg](../../../botbrain_ws/src/tita_pkg) | Interface de hardware e descrição do Tita quadrúpede | [README](../../../botbrain_ws/src/tita_pkg/README.md) |

## Serviços Docker

O workspace inclui múltiplos serviços Docker para implantação containerizada:

| Serviço | Descrição | Auto-start | Dependências |
|---------|-------------|------------|--------------|
| `dev` | Container de desenvolvimento (interativo) | Não | - |
| `builder_base` | Compila todos os pacotes do workspace | Não | - |
| `state_machine` | Serviço de gerenciamento de ciclo de vida | Sim | - |
| `bringup` | Bringup principal do robô | Sim | state_machine |
| `localization` | Localização RTABMap | Sim | bringup |
| `navigation` | Servidores de navegação Nav2 | Sim | localization |
| `rosa` | Serviços de chamada de ferramentas IA | Sim | bringup |
| `yolo` | Serviço de detecção de objetos | Sim | bringup |

### Uso do Docker

```bash
# Iniciar todos os serviços
docker compose up -d

# Iniciar serviço específico com dependências
docker compose up -d navigation  # Automaticamente inicia bringup, localization

# Ver logs
docker compose logs -f bringup

# Parar todos os serviços
docker compose down

# Recompilar após alterações de código
docker compose build
docker compose up -d
```

## Configuração

### Arquivo de Configuração Principal

O arquivo [robot_config.yaml](../../../botbrain_ws/robot_config.yaml) é o ponto central de configuração:

```yaml
robot_configuration:

  # Identificador do robô - usado como namespace para todos os tópicos
  robot_name: ""                    # Exemplo: "go2_robot1", "tita_lab"

  # Tipo de robô - determina qual pacote de hardware lançar
  robot_model: "go2"                # Opções: "go2", "tita", "seu_robo"

  # Formato do arquivo de descrição
  description_file_type: "xacro"    # Opções: "xacro", "urdf"

  # Interface de rede para comunicação ROS2
  network_interface: "eno1"         # Exemplo: "eth0", "wlan0", "eno1"

  # Específico do Tita: namespace para comunicação do robô Tita
  tita_namespace: "tita3036731"     # Usado apenas quando robot_model: "tita"

  # Chave API OpenAI para recursos de IA (opcional)
  openai_api_key: ""                # Obrigatório para assistente ROSA AI

  # Configuração Wi-Fi (opcional)
  wifi_interface: ""                # Nome da interface Wi-Fi (ex.: "wlan0")
  wifi_ssid: ""                     # SSID da rede Wi-Fi
  wifi_password: ""                 # Senha da rede Wi-Fi
```

### Configuração de Rede

O workspace usa CycloneDDS para comunicação ROS 2. Configuração em [cyclonedds_config.xml](../../../botbrain_ws/cyclonedds_config.xml):

Defina a interface de rede em [robot_config.yaml](../../../botbrain_ws/robot_config.yaml) para corresponder à sua conexão de hardware.

### Serviço Systemd de Autostart

O arquivo [botbrain.service](../../../botbrain_ws/botbrain.service) habilita inicialização automática no boot:

```bash
# Instalar serviço (feito pelo install.sh)
sudo cp botbrain.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable botbrain.service

# Controle manual
sudo systemctl start botbrain.service   # Iniciar agora
sudo systemctl stop botbrain.service    # Parar
sudo systemctl status botbrain.service  # Verificar status

# Ver logs
journalctl -u botbrain.service -f
```

### Adicionando Suporte para Novos Robôs

Veja o guia [Criando um Pacote de Robô Personalizado](#criando-um-pacote-de-robô-personalizado) acima. Especialmente recebemos contribuições que adicionam suporte para novas plataformas de robôs!

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Ícone Bot" width="110">
</p>
