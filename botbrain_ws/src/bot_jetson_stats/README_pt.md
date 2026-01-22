# Bot Jetson Stats

Um pacote ROS2 abrangente para monitorar e controlar estatísticas de hardware NVIDIA Jetson, fornecendo informações diagnósticas em tempo real e capacidades de controle de hardware para aplicações de robótica.

## Visão geral

Bot Jetson Stats é um sistema de monitoramento e controle baseado em ROS2, desenvolvido especificamente para dispositivos NVIDIA Jetson. Ele fornece monitoramento completo de hardware, relatórios de diagnóstico e serviços de controle para vários componentes do Jetson, incluindo CPU, GPU, memória, temperatura, consumo de energia e controle de ventoinha.

**Autor original**: [Raffaello Bonghi](https://github.com/rbonghi/jetson_stats) - Criador das utilidades de monitoramento [jetson_stats](https://github.com/rbonghi/jetson_stats) e da biblioteca jtop que formam a base deste pacote ROS2.

## Recursos

### 🔍 **Monitoramento de hardware**
- **Monitoramento de CPU**: Uso, frequência e temperatura em tempo real para cada núcleo
- **Monitoramento de GPU**: Utilização da GPU, uso de memória e temperatura
- **Monitoramento de memória**: Estatísticas de uso de RAM, SWAP e memória EMC
- **Monitoramento de energia**: Monitoramento de consumo e voltagem
- **Monitoramento de temperatura**: Leituras completas com limites configuráveis de aviso/erro
- **Controle de ventoinha**: Gerenciamento de velocidade e perfil da ventoinha
- **Monitoramento de disco**: Saúde e uso de armazenamento

### 🎛️ **Serviços de controle de hardware**
- **Serviço de controle de ventoinha**: Ajuste de velocidade e perfil (modos quiet/cool)
- **Serviço de modo de energia**: Alterna entre modos de energia NVIDIA (NVPModel)
- **Serviço Jetson Clocks**: Habilita/desabilita clocks de desempenho do Jetson

### 📊 **Sistema de diagnóstico**
- Mensagens de diagnóstico ROS2 para integração com ferramentas de diagnóstico
- Saída legível em terminal para monitoramento fácil
- Intervalos de monitoramento e limiares configuráveis
- Publicação de status do modo de rede

### 🚀 **Integração ROS2**
- Nós e serviços nativos ROS2
- Publicação de mensagens de diagnóstico
- Configurações de launch files
- Suporte a namespace para sistemas multi-robô

## Estrutura do pacote

```
bot_jetson_stats/
├── bot_jetson_stats/                 # Pacote principal
│   ├── launch/                       # Launch files
│   │   ├── jetson_stats.launch.py   # Launch principal de monitoramento
│   │   ├── stats.launch.py          # Monitoramento de estatísticas
│   │   └── network_mode_service.launch.py  # Status de rede
│   ├── scripts/                      # Executáveis Python
│   │   ├── ros2_jtop_node.py        # Nó principal de monitoramento
│   │   ├── network_mode_publisher.py # Publicador de status de rede
│   │   └── diagnostic_stats_terminal_viewer.py # Visualizador no terminal
│   ├── bot_jetson_stats/            # Utilitários Python
│   │   └── utils.py                 # Utilitários de mensagens de diagnóstico
│   └── CMakeLists.txt               # Configuração de build
├── bot_jetson_stats_interfaces/      # Definições de serviços
│   ├── srv/                         # Arquivos de serviço
│   │   ├── Fan.srv                  # Serviço de controle de ventoinha
│   │   ├── JetsonClocks.srv         # Serviço de clocks do Jetson
│   │   └── NVPModel.srv             # Serviço de modo de energia
│   └── CMakeLists.txt               # Configuração de build de interfaces
└── README.md                         # Este arquivo
```

## Dependências

### Dependências de sistema
- **jtop**: Biblioteca de monitoramento NVIDIA Jetson (de [jetson_stats](https://github.com/rbonghi/jetson_stats))
- **Python 3**: Runtime Python 3.x
- **Linux**: Sistema operacional Linux (testado em sistemas Tegra)

### Dependências ROS2
- **rclpy**: Biblioteca cliente ROS2 para Python
- **rclcpp**: Biblioteca cliente ROS2 para C++
- **diagnostic_msgs**: Tipos de mensagem de diagnóstico ROS2
- **std_msgs**: Tipos de mensagem padrão ROS2
- **ament_cmake**: Sistema de build
- **ament_cmake_python**: Suporte de build para Python

## Instalação

### Pré-requisitos
1. Instale o ROS2 (Humble ou superior recomendado)
2. Instale a biblioteca jtop:
   ```bash
   sudo pip3 install jtop
   ```

### Build a partir do código-fonte
1. Clone este repositório para o seu workspace ROS2:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> bot_jetson_stats
   ```

2. Compile o workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select bot_jetson_stats bot_jetson_stats_interfaces
   ```

3. Source do workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Uso

### Início rápido

1. **Inicie o sistema principal de monitoramento**:
   ```bash
   ros2 launch bot_jetson_stats jetson_stats.launch.py
   ```

2. **Veja estatísticas de diagnóstico no terminal**:
   ```bash
   ros2 run bot_jetson_stats diagnostic_stats_terminal_viewer
   ```

3. **Monitore mensagens de diagnóstico**:
   ```bash
   ros2 topic echo /diagnostics
   ```

### Launch files

#### Launch principal de monitoramento (`jetson_stats.launch.py`)
Inicia o sistema completo de monitoramento Jetson incluindo:
- Nó de monitoramento Jetson stats
- Visualizador de diagnóstico no terminal
- Suporte a namespace configurável do robô

#### Launch de estatísticas (`stats.launch.py`)
Inicia apenas os componentes de monitoramento de estatísticas.

#### Serviço de modo de rede (`network_mode_service.launch.py`)
Inicia o publicador de status do modo de rede.

### Serviços

#### Serviço de controle de ventoinha (`/jtop/fan`)
- **Request**: `mode` (string: "quiet" ou "cool"), `speed` (int64)
- **Response**: `set_fan_mode`, `set_fan_speed`
- **Uso**: Controle de velocidade e perfil da ventoinha

#### Serviço de modo de energia (`/jtop/nvpmodel`)
- **Request**: `nvpmodel` (int64: ID do modo de energia)
- **Response**: `power_mode` (string: modo de energia atual)
- **Uso**: Alterna entre modos de energia NVIDIA

#### Serviço Jetson Clocks (`/jtop/jetson_clocks`)
- **Request**: `status` (bool: habilitar/desabilitar)
- **Response**: `done` (bool: status de conclusão da operação)
- **Uso**: Habilita/desabilita clocks de desempenho do Jetson

### Tópicos

#### Tópicos publicados
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): Mensagens de diagnóstico ROS2
- `/diagnostic_stats` (std_msgs/String): Resumo diagnóstico legível
- `/network_mode_status` (std_msgs/String): Atualizações de status do modo de rede

## Configuração

### Parâmetros do nó

O nó principal de monitoramento (`ros2_jtop_node.py`) suporta os seguintes parâmetros:

- **`interval`** (padrão: 2): Intervalo de atualização em segundos
- **`level_error`** (padrão: 60): Limite de erro de temperatura em Celsius
- **`level_warning`** (padrão: 40): Limite de aviso de temperatura em Celsius
- **`level_ok`** (padrão: 20): Limite de OK de temperatura em Celsius

### Configuração do robô

O sistema suporta configuração específica por robô por meio de um arquivo `robot_config.yaml` na raiz do workspace:

```yaml
robot_configuration:
  robot_name: "my_robot"
```

## Recursos de monitoramento

### Estatísticas em tempo real
- **CPU**: Utilização por núcleo, frequência e temperatura
- **GPU**: Utilização, uso de memória e temperatura
- **Memória**: Uso de RAM, SWAP e EMC com limiares
- **Energia**: Monitoramento de consumo e leituras de voltagem
- **Temperatura**: Monitoramento multi-zona com alertas configuráveis
- **Ventoinha**: Controle de velocidade e gerenciamento de perfil
- **Sistema**: Uptime, modo de energia e status do Jetson Clocks

### Integração de diagnóstico
- Compatível com ferramentas de diagnóstico ROS2 (rqt_diagnostic, rqt_topic)
- Formato padrão de mensagens de diagnóstico
- Limiares de aviso e erro configuráveis
- Identificação de hardware e reporte de status

## Solução de problemas

### Problemas comuns

1. **Erros de permissão**: Garanta que o jtop tenha permissões para acessar o hardware Jetson
2. **Falhas de serviço**: Verifique se o dispositivo Jetson suporta a operação solicitada
3. **Leituras de temperatura**: Verifique acesso e calibração dos sensores


### Ferramentas de monitoramento

- **rqt_topic**: Ver e monitorar tópicos ROS2
- **rqt_diagnostic**: Visualizar informações de diagnóstico
- **rqt_graph**: Ver conexões de nós e tópicos

## Desenvolvimento

### Adicionando novos recursos de monitoramento

1. Estenda o arquivo `utils.py` com novas funções de diagnóstico
2. Adicione novas definições de serviço no pacote de interfaces
3. Atualize o nó principal para incluir novas capacidades de monitoramento
4. Adicione configurações adequadas de launch files


## Agradecimentos

- **[Raffaello Bonghi](https://github.com/rbonghi/jetson_stats)**: Autor original e criador das utilidades de monitoramento [jetson_stats](https://github.com/rbonghi/jetson_stats) e da biblioteca jtop
- **Biblioteca jtop**: Capacidades de monitoramento NVIDIA Jetson (parte do projeto jetson_stats)
- **Comunidade ROS2**: Framework e suporte a mensagens de diagnóstico
- **Contribuidores originais**: Utilidades e conceitos base do projeto jetson_stats

## Aviso de licença

O projeto `jetson_stats` e a biblioteca `jtop` são licenciados sob a **Licença MIT**.  
Este repositório os usa como dependências externas e não inclui seu código-fonte.

---

**Nota**: Este pacote foi projetado especificamente para dispositivos NVIDIA Jetson e requer hardware apropriado para funcionar corretamente.
