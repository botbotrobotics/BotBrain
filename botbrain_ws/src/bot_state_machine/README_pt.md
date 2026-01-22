<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_state_machine

**Camada de coordenação de ciclo de vida para nós ROS 2 na stack BotBot.**

`bot_state_machine` mantém os nós de ciclo de vida da plataforma sincronizados: ele analisa o grafo de nós, aplica transições de configure/activate/deactivate na ordem correta, expõe um serviço de comando de alto nível, monitora eventos de transição e alterna automaticamente entre modos autônomo e teleop quando nós de navegação se comportam mal.

## Objetivo do pacote

Este pacote integra o restante do sistema ao:

- **Bringup/Shutdown coordenados** – Conduz cada nó pelo ciclo de vida ROS 2 na ordem de dependências (CORE → NAVIGATION → accessories, etc.).
- **Gerenciamento de dependências** – Valida dependências/dependentes recursivamente e garante que nós da classe PAYLOAD nunca sejam alternados automaticamente.
- **Monitoramento de eventos** – Assina eventos de transição de ciclo de vida e reage a mudanças inesperadas de UNCONFIGURED/ACTIVE.
- **Interface de comando** – Oferece um serviço `StateMachine` para acionar ativação, desativação ou restart para qualquer nó gerenciado.
- **Agregação de status** – Publica `bot_custom_interfaces/msg/StatusArray` com estado de ciclo de vida, prioridade e saúde de cada nó.
- **Arbitragem de modo** – Mantém estados BRINGUP / AUTONOMOUS / TELEOP / BRINGDOWN / RESTARTING, alternando modos com base na saúde da navegação.

---

## Componentes principais

### `LifecycleManager`

- Carrega descrições de nós em JSON (por classe) e converte em entradas `NodeProfile`.
- Cria clientes de ciclo de vida (`/get_state`, `/change_state`) para cada nó gerenciado.
- Publica `StatusArray` e expõe `bot_custom_interfaces/srv/StateMachine`.
- Roteia eventos de transição de ciclo de vida para o controlador e limpa eventos obsoletos.
- Invoca periodicamente `update_callback`, `publish_callback` e `event_callback`.

### `StateController`

Máquina de estados finita:

- Estados: `BRINGUP`, `AUTONOMOUS`, `TELEOP`, `BRINGDOWN`, `IDLE`, `ERROR`, `RESTARTING`.
- Realiza checagens de dependência, ativação/desativação e lógica de restart.
- Mantém filas de eventos, conjuntos de nós protegidos e processamento de comandos.
- Trata fallback de navegação: se algum nó NAVIGATION falhar na ativação, todos os nós NAV são desativados e o controlador transita para TELEOP.

### `GraphNode`

- Helper leve que analisa manifestos JSON por classe.
- Converte entradas em objetos `NodeProfile` e constrói listas de dependências/dependentes.
- Ordena nós por classe e prioridade para garantir ordenação determinística de bringup/bringdown.

| Arquivo | Descrição |
|------|-------------|
| `core.json` | Serviços core (Docker, web socket, etc.) |
| `navigation.json` | Stack Nav2, localização, planners |
| `camera.json` | Nós RealSense / percepção |
| `audio.json` | Pipeline de voz |
| `accessories.json` | Utilitários diversos |
| `ia_stack.json` | Componentes de IA/ML |
| `payload.json` | Payloads de terceiros que o controlador apenas monitora |

Cada entrada JSON contém o nome do nó, nome de exibição, classe, ordem de startup, descrição e lista de dependências.

---

## Modelo de ciclo de vida

### Estados do controlador

| Estado | Descrição |
|-------|-------------|
| `BRINGUP` | Configura e ativa nós sequencialmente por classe/ordem. |
| `AUTONOMOUS` | Nós de navegação ativos; eventos podem disparar restarts. |
| `TELEOP` | Navegação falhou; sistema segue operacional sem stack autônoma. |
| `BRINGDOWN` | Desativação e limpeza em ordem reversa. |
| `IDLE` | Todos os nós inativos, aguardando próximo bringup. |
| `ERROR` | Bringdown encontrou falha irrecuperável. |
| `RESTARTING` | Controlador forçando bringdown seguido de bringup. |

---

## Inicialização

### `launch/state_machine.launch.py`

| Entrada | Descrição |
|-------|-------------|
| `robot_config.yaml` (raiz do workspace) | Fornece `robot_configuration.robot_name` usado como prefixo de namespace. |
| `config/` | Passado via parâmetro `nodes_json_path`. |

O launch file:

1. Lê `robot_config.yaml` para determinar o namespace (ex.: `g1_robot/`).
2. Inicia o nó de ciclo de vida `state_machine_node`.
3. Após o processo aparecer, emite a transição `configure`.
4. Quando o nó chega em `inactive`, emite automaticamente `activate`.

```bash
# A partir da raiz do workspace
source install/setup.bash
ros2 launch bot_state_machine state_machine.launch.py
```

---

## Monitoramento e controle

### Consultas de ciclo de vida

```bash
# Listar estados de ciclo de vida para um nó gerenciado
ros2 lifecycle get /<ns>/<node_name>

# Solicitar manualmente uma transição (uso avançado)
ros2 lifecycle set /<ns>/<node_name> deactivate
```

### Serviço da máquina de estados

```bash
# Reiniciar um nó de navegação
ros2 service call /<ns>/state_machine bot_custom_interfaces/srv/StateMachine \
"{node: 'nav2_bt_navigator', command: 3}"
```

### Tópico de status

`/<ns>/state_machine/status` publica `bot_custom_interfaces/msg/StatusArray` contendo estado de ciclo de vida, prioridade, classe e mensagem diagnóstica de cada nó. Use `ros2 topic echo` ou integre em dashboards.

---

## Fluxo de configuração

1. **Declarar nós** – Atualize o arquivo JSON que corresponde à classe do nó (ex.: `navigation.json`).
2. **Definir dependências** – Adicione `dependencies` (nós que devem estar ativos antes deste nó) e `dependents_of` (nós que dependem deste nó para ordenação de teardown).
3. **Atribuir prioridade e ordem** – `priority` impacta o status e a ordenação na UI; `order` controla a ordem de startup dentro de cada classe.
4. **Deploy** – Rebuild do workspace ou redeploy de containers; sem alterações de código necessárias.

O controlador recarrega definições automaticamente no configure.

---

## Estrutura de diretórios

```
bot_state_machine/
├── include/
│   ├── lifecycle_manager.hpp    # Orquestrador do nó de ciclo de vida
│   ├── state_controller.hpp     # Declaração do controlador FSM
│   ├── graph_node.hpp           # Helpers para parsing de JSON
│   └── types.hpp                # Enums, structs e timeouts compartilhados
├── src/
│   ├── state_controller.cpp     # Lógica de bringup/bringdown + gerenciador de eventos
│   ├── lifecycle_manager.cpp    # Implementação de LifecycleNode
│   └── state_machine_node.cpp   # Executável principal
├── config/                      # Grafo de nós JSON agrupado por classe
├── launch/
│   └── state_machine.launch.py  # Launch de ciclo de vida + auto configure/activate
├── CMakeLists.txt / package.xml # Metadados de build ROS 2
└── README.md                    # Este documento
```

---

## Exemplos de uso

```bash
# Iniciar a máquina de estados sob um namespace específico (de robot_config.yaml)
ros2 launch bot_state_machine state_machine.launch.py

# Monitorar saída de status
ros2 topic echo /g1_robot/state_machine/status

# Forçar um restart completo (via API de serviço)
ros2 service call /g1_robot/state_machine bot_custom_interfaces/srv/StateMachine \
"{node: 'state_machine', command: 3}"
```

---

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
