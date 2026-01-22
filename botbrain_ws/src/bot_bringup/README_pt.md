<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_bringup

**Pacote central de orquestração do sistema para o framework de robótica BotBrain**

O pacote `bot_bringup` atua como coordenador central de launch do sistema principal do robô. Ele gerencia o carregamento da descrição do robô, a inicialização da interface de hardware, o multiplexador de comandos de velocidade com arbitragem baseada em prioridade e as interfaces de comunicação.


## Objetivo do pacote

Este pacote é responsável por:

- **Orquestração do sistema**: iniciar todos os subsistemas principais na ordem correta de dependências
- **Gerenciamento de interface do robô**: carregar dinamicamente pacotes de hardware específicos do robô
- **Arbitragem de comandos de velocidade**: multiplexação baseada em prioridade de comandos vindos de múltiplas fontes
- **Baseline de segurança**: garantir que o robô pare quando não houver comandos ativos
- **Pontes de comunicação**: interface web e conectividade de controle remoto

## Nós

### zero_vel_publisher

Nó de ciclo de vida que publica continuamente comandos de velocidade zero como baseline de segurança.

**Executável**: `zero_vel_publisher.py`

**Descrição**: fornece o comando de velocidade de menor prioridade para garantir que o robô pare quando todas as outras fontes de comando expirarem ou ficarem inativas. Isso atua como mecanismo de segurança.

#### Publicadores

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `cmd_vel_zero` | `geometry_msgs/Twist` | Comando de velocidade zero (todos os campos = 0.0) |

#### Estados do ciclo de vida

| Estado | Descrição |
|-------|-------------|
| **Não configurado** | Estado inicial, nenhum recurso alocado |
| **Configurado** | Publicador criado, mas inativo |
| **Ativo** | Timer em execução, publicando velocidade zero a 10Hz |
| **Desativado** | Timer parado, publicador existe mas inativo |
| **Finalizado** | Todos os recursos limpos |

**Transições de ciclo de vida**:
- `configure`: cria o publicador para `cmd_vel_zero`
- `activate`: inicia o timer de 10Hz para publicar velocidade zero
- `deactivate`: para o timer, publicador permanece
- `cleanup`: destrói publicador e timer
- `shutdown`: limpeza emergencial de todos os recursos

### twist_mux

Nó de terceiros do pacote `twist_mux` que arbitra comandos de velocidade com base em prioridade.

**Pacote**: `twist_mux`

**Descrição**: multiplexa comandos de velocidade de múltiplas fontes (joystick, web, navegação, IA) e encaminha o comando ativo de maior prioridade ao robô. Inclui um bloqueio de segurança do tipo dead-man switch.

#### Publicadores

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `cmd_vel_out` | `geometry_msgs/Twist` | Comando de velocidade final arbitrado enviado ao robô |
| `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Informações diagnósticas sobre entradas ativas |

#### Assinantes

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `cmd_vel_zero` | `geometry_msgs/Twist` | Baseline de velocidade zero (prioridade: 1) |
| `cmd_vel_rosa` | `geometry_msgs/Twist` | Comandos do assistente de IA (prioridade: 5) |
| `cmd_vel_nav` | `geometry_msgs/Twist` | Comandos da pilha de navegação (prioridade: 10) |
| `cmd_vel_nipple` | `geometry_msgs/Twist` | Joystick da interface web (prioridade: 99) |
| `cmd_vel_joy` | `geometry_msgs/Twist` | Comandos do joystick físico (prioridade: 100) |
| `dead_man_switch` | `std_msgs/Bool` | Trava de segurança, deve ser true para mover (prioridade: 200) |


#### Parâmetros

| Nome do parâmetro | Tipo | Valor padrão | Descrição |
|----------------|------|---------------|-------------|
| `twist_watchdog_timeout` | double | `0.5` | Timeout global (segundos) para todas as entradas de twist |

**Configuração de tópicos de entrada** (de [config/twist_mux.yaml](config/twist_mux.yaml)):

| Nome da entrada | Tópico | Prioridade | Timeout (s) | Descrição |
|------------|-------|----------|-------------|-------------|
| `zero_velocity` | `cmd_vel_zero` | 1 | 0.1 | Baseline de segurança - sempre ativo |
| `rosa` | `cmd_vel_rosa` | 5 | 0.2 | Comandos gerados pelo assistente de IA |
| `navigation` | `cmd_vel_nav` | 10 | 0.2 | Navegação autônoma via Nav2 |
| `nipple` | `cmd_vel_nipple` | 99 | 0.2 | Joystick virtual via navegador |
| `joystick` | `cmd_vel_joy` | 100 | 0.2 | Controle físico (maior prioridade) |

**Configuração de tópicos de lock**:

| Nome do lock | Tópico | Prioridade | Timeout (s) | Descrição |
|-----------|-------|----------|-------------|-------------|
| `dead_man` | `dead_man_switch` | 200 | 0.0 | Chave de segurança - bloqueia todo movimento quando false |

**Comportamento de prioridade**:
- Valores de prioridade mais altos sobrepõem comandos de menor prioridade
- Comandos expiram se não forem recebidos dentro do tempo especificado
- Dead-man switch (prioridade 200) bloqueia TODO movimento quando inativo/false
- Velocidade zero (prioridade 1) garante que o robô pare quando todos os comandos expiram

## Arquivos de launch

### bringup.launch.py

Lançador principal do sistema que orquestra todos os componentes principais do robô.

**Caminho**: [launch/bringup.launch.py](launch/bringup.launch.py)

**Descrição**: inicia o sistema completo do robô incluindo descrição, interface de hardware, controle de movimento, teleoperação e comunicação web.

#### O que é iniciado

1. **Descrição do robô** (`bot_description`):
   - Carrega o modelo URDF/XACRO a partir do pacote específico do robô
   - Inicia `robot_state_publisher` para publicar TF
   - Configurado com base em `robot_model` em `robot_config.yaml`

2. **Interface de hardware do robô** (`{robot_model}_pkg`):
   - Inclui dinamicamente o launch file específico do robô
   - Exemplo: `go2_pkg/launch/robot_interface.launch.py` para Go2
   - Inicia nós de leitura/escrita de hardware para comunicação do robô

3. **Multiplexador de twist** (via `twist_mux.launch.py`):
   - Arbitragem de comandos de velocidade por prioridade
   - Baseline de segurança de velocidade zero
   - Integração com dead-man switch

4. **Interface de joystick** (`joystick_bot`):
   - Teleoperação por controle
   - Publica em `cmd_vel_joy`
   - Gerencia o dead-man switch

5. **Servidor ROSBridge WebSocket** (`rosbridge_server`):
   - Ponte WebSocket para controle via navegador
   - Habilita UI web e monitoramento remoto
   - Porta: 9090 (padrão)

#### Argumentos de launch

| Argumento | Tipo | Padrão | Descrição |
|----------|------|---------|-------------|
| Nenhum - configurado via `robot_config.yaml` | - | - | Todas as configurações lidas do arquivo de configuração do workspace |

#### Fonte de configuração

O launch file lê [robot_config.yaml](../../../../robot_config.yaml) na raiz do workspace:

### twist_mux.launch.py

Inicia o subsistema de controle de movimento com arbitragem de velocidade.

**Caminho**: [launch/twist_mux.launch.py](launch/twist_mux.launch.py)

**Descrição**: inicia o nó twist_mux para arbitragem de comandos e o zero_vel_publisher para baseline de segurança.

#### O que é iniciado

1. **Nó twist_mux**:
   - Multiplexa comandos de velocidade de múltiplas fontes
   - Configurado via `config/twist_mux.yaml`
   - Compatível com namespace, baseado em `robot_config.yaml`

2. **zero_vel_publisher (Nó Lifecycle)**:
   - Publica velocidade zero a 10Hz
   - Auto-configurado e ativado via transições do ciclo de vida
   - Fornece baseline de segurança

#### Argumentos de launch

| Argumento | Tipo | Padrão | Descrição |
|----------|------|---------|-------------|
| Nenhum - configurado via arquivos | - | - | Usa `robot_config.yaml` e `twist_mux.yaml` |

## Arquivos de configuração

### twist_mux.yaml

Configuração de prioridade e timeout de comandos de velocidade.

**Caminho**: [config/twist_mux.yaml](config/twist_mux.yaml)

**Descrição**: define níveis de prioridade, timeouts e travas de segurança para arbitragem de comandos de velocidade.


#### Níveis de prioridade explicados

| Prioridade | Fonte | Caso de uso | Comportamento de override |
|----------|--------|----------|-------------------|
| 1 | Velocidade zero | Baseline de segurança | Sobreposto por qualquer comando ativo |
| 5 | Assistente de IA (ROSA) | Tarefas autônomas | Sobreposto por navegação e controle manual |
| 10 | Navegação (Nav2) | Navegação autônoma | Sobreposto por controle manual |
| 99 | Joystick web | Controle manual remoto | Sobreposto apenas pelo joystick físico |
| 100 | Joystick físico | Controle manual direto | Maior prioridade de comando |
| 200 | Dead-Man Switch | Trava de segurança | Bloqueia TODOS os comandos quando false |

#### Comportamento de timeout

- Se uma fonte de comando parar de publicar por mais tempo que seu timeout, ela fica inativa
- A próxima fonte ativa de maior prioridade assume
- Velocidade zero (prioridade 1) tem o menor timeout (0.1s) para garantir ativação rápida
- Dead-man switch tem timeout 0.0 - bloqueia imediatamente o movimento quando false

## Tópicos

### Tópicos publicados

| Tópico | Tipo de mensagem | Taxa | Descrição |
|-------|--------------|------|-------------|
| `/{namespace}/cmd_vel_out` | `geometry_msgs/Twist` | Variável | Comando de velocidade final arbitrado enviado ao hardware do robô |
| `/{namespace}/cmd_vel_zero` | `geometry_msgs/Twist` | 10 Hz | Baseline de segurança de velocidade zero |
| `/{namespace}/twist_mux/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Status diagnóstico das entradas do twist_mux |

### Tópicos assinados

Todas as entradas de comandos de velocidade (assinadas pelo twist_mux):

| Tópico | Tipo de mensagem | Prioridade | Timeout | Descrição |
|-------|--------------|----------|---------|-------------|
| `/{namespace}/cmd_vel_joy` | `geometry_msgs/Twist` | 100 | 0.2s | Comandos do joystick físico |
| `/{namespace}/cmd_vel_nipple` | `geometry_msgs/Twist` | 99 | 0.2s | Comandos da interface web |
| `/{namespace}/cmd_vel_nav` | `geometry_msgs/Twist` | 10 | 0.2s | Comandos da pilha de navegação |
| `/{namespace}/cmd_vel_rosa` | `geometry_msgs/Twist` | 5 | 0.2s | Comandos do assistente de IA |
| `/{namespace}/cmd_vel_zero` | `geometry_msgs/Twist` | 1 | 0.1s | Baseline de velocidade zero |
| `/{namespace}/dead_man_switch` | `std_msgs/Bool` | 200 (lock) | 0.0s | Chave de segurança para habilitar movimento |

## Serviços

Nenhum - este pacote fornece orquestração de launch e não expõe serviços diretamente. Nós iniciados (de outros pacotes) podem fornecer seus próprios serviços.

## Transformações (TF)

Este pacote não fornece transformações TF diretamente. O `robot_state_publisher` iniciado a partir de `bot_description` publica a árvore cinemática do robô.

### Listeners TF

Nenhum diretamente, mas nós iniciados podem ouvir transformações do `bot_description`

### Broadcasters TF

Nenhum diretamente - a publicação de TF é feita pelo `robot_state_publisher` do pacote `bot_description`

## Integração com outros pacotes

### Dependências

| Pacote | Objetivo |
|---------|---------|
| `bot_description` | Modelos URDF/XACRO do robô e robot_state_publisher |
| `{robot_model}_pkg` | Interface de hardware específica do robô (ex.: go2_pkg, tita_pkg) |
| `joystick_bot` | Interface de controle e dead-man switch |
| `rosbridge_server` | Ponte WebSocket para interface web |
| `twist_mux` | Multiplexação de comandos de velocidade (terceiros) |

### Carregamento dinâmico de pacotes de robô

O sistema de bringup carrega pacotes de robô com base na configuração:

**Configuração**:
```yaml
# robot_config.yaml
robot_configuration:
  robot_model: "go2"  # Opções: "go2", "tita", "g1", "custom_robot"
```

**Lógica de carregamento**:
1. Ler `robot_model` na configuração
2. Montar o nome do pacote: `{robot_model}_pkg`
3. Incluir o launch file: `{robot_model}_pkg/launch/robot_interface.launch.py`

**Exemplo**:
- `robot_model: "go2"` -> Inicia `go2_pkg/launch/robot_interface.launch.py`
- `robot_model: "tita"` -> Inicia `tita_pkg/launch/robot_interface.launch.py`

### Tópicos esperados dos pacotes de robô

Pacotes específicos de robô devem publicar estes tópicos para integração completa:

| Tópico | Tipo de mensagem | Obrigatório | Descrição |
|-------|--------------|----------|-------------|
| `/{namespace}/odom` | `nav_msgs/Odometry` | Sim | Odometria do robô para navegação |
| `/{namespace}/imu` | `sensor_msgs/Imu` | Sim | Dados de IMU para localização |
| `/{namespace}/joint_states` | `sensor_msgs/JointState` | Sim | Estados das juntas para robot_state_publisher |
| `/{namespace}/battery` | `sensor_msgs/BatteryState` | Sim | Status da bateria e nível de carga |

E assinar:

| Tópico | Tipo de mensagem | Descrição |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel` | `geometry_msgs/Twist` | Comandos finais de velocidade do twist_mux |


## Estrutura de diretórios

```
bot_bringup/
├── bot_bringup/
│   └── __init__.py                # Marcador do pacote Python
│
├── launch/
│   ├── bringup.launch.py          # Lançador principal do sistema
│   └── twist_mux.launch.py        # Lançador do subsistema de controle de movimento
│
├── config/
│   └── twist_mux.yaml             # Prioridades e timeouts de comandos de velocidade
│
├── scripts/
│   └── zero_vel_publisher.py      # Nó publicador de velocidade zero de segurança
│
├── CMakeLists.txt                 # Configuração de build
├── package.xml                    # Manifesto do pacote
└── README.md                      # Este arquivo
```

---

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
