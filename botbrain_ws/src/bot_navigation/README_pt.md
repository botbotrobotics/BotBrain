<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# **bot_navigation – Stack de Navegação Autônoma**

Pacote de navegação autônoma baseado no ROS 2 Navigation2 (Nav2). Este pacote fornece planejamento de trajetos, desvio de obstáculos e navegação autônoma orientada a objetivos para a plataforma de robôs BotBrain.

## **Objetivo**

O pacote `bot_navigation` habilita a navegação autônoma do robô ao integrar a stack Nav2 com o sistema BotBrain. Ele gerencia:
- Planejamento global de trajetos
- Planejamento local de trajetória e desvio de obstáculos
- Coordenação de comportamentos (recuperação, seguir waypoints)
- Geração de comandos de velocidade para movimento autônomo
- Integração com sistemas de localização e mapeamento
- Utilitários em tempo de execução para interagir com ações do Nav2

## **Arquivos do pacote**

### **Arquivos de launch**

### **launch/nav2.launch.py**

Lançador principal do Nav2 que inicia todos os componentes:

**Nós iniciados:**

1. **controller_server** – Executa trajetos planejados e gera comandos de velocidade
2. **smoother_server** – Suaviza trajetos planejados
3. **planner_server** – Computa trajetos globais
4. **behavior_server** – Lida com comportamentos de recuperação (recuar, girar, aguardar)
5. **bt_navigator** – Coordenador de navegação baseado em Behavior Tree
6. **waypoint_follower** – Executa navegação por waypoints

---

### **launch/nav_utils.launch.py**

Inicia nós auxiliares de utilidades de navegação.

**Nós iniciados:**

- **nav2_utils** – Nó de ciclo de vida que fornece serviços de tempo de execução para interação com Nav2

---

### **launch/navigation.launch.py**

Lançador de alto nível que compõe a stack de navegação.

**Inclui:**

- nav_utils.launch.py
- nav2.launch.py

Este é o ponto de entrada recomendado para iniciar a navegação.

**Uso:**

```
ros2 launch bot_navigation navigation.launch.py
```

### **Utilitários de navegação**

### **scripts/nav2_utils.py**

Nó de ciclo de vida que expõe uma interface de serviço para interagir com ações do Nav2.

**Recursos:**

- Fornece o serviço /cancel_nav2_goal (std_srvs/Trigger)
- Cancela objetivos ativos do Nav2 para:
    - NavigateToPose
    - FollowWaypoints
- Projetado para rodar junto com o Nav2 usando um executor multithread
- Namespaced automaticamente usando robot_name do robot_config.yaml

**Exemplos de uso:**

- Parada de emergência ou cancelamento externo de objetivos de navegação
- Integração com sistemas de comportamento de alto nível ou segurança

## **Configuração específica do robô**

Cada pacote de robô fornece parâmetros de navegação ajustados:

- **go2_pkg/config/nav2_params.yaml** – Unitree Go2
- **tita_pkg/config/nav2_params.yaml** – Tita quadrúpede
- **g1_pkg/config/nav2_params.yaml** – G1 humanoide

**Os parâmetros incluem:**

- Footprint do robô e geometria de colisão
- Limites cinemáticos (velocidade, aceleração)
- Ajuste do controlador
- Configuração de camadas do costmap

## **Integração com o sistema do robô**

### **Componentes necessários**

A navegação requer os seguintes pacotes BotBrain:

1. **Localização** (bot_localization) – Fornece o frame do mapa e a pose do robô
2. **Descrição** (bot_description) – Fornece o modelo do robô e a árvore TF
3. **Twist Mux** (bot_bringup) – Multiplexa comandos de navegação com outras fontes de movimento

## **Estrutura de diretórios**

```
bot_navigation/
├── bot_navigation/
│   └── __init__.py
├── launch/
│   ├── nav2.launch.py              # Lançador principal do Nav2
│   ├── nav_utils.launch.py         # Lançador de utilidades do Nav2
│   └── navigation.launch.py        # Lançador de navegação de alto nível
├── scripts/
│   └── nav2_utils.py               # Nó de ciclo de vida utilitário do Nav2
├── CMakeLists.txt
├── package.xml
└── README.md
```
---

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>
