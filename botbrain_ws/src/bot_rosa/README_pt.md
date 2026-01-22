# bot_rosa

Pacote de assistente de IA para a plataforma de robôs BotBrain. Este pacote fornece uma interface conversacional inteligente usando o framework ROSA (Robot Operating System Assistant) com integração de LLM para interação em linguagem natural e controle do robô.

## Descrição

O pacote `bot_rosa` atua como o sistema de assistente de IA para a plataforma do robô R1, oferecendo capacidades de processamento de linguagem natural e controle inteligente do robô por meio de interfaces conversacionais. Ele se integra ao framework ROSA para permitir comandos de voz, perguntas e respostas e operações autônomas do robô usando Modelos de Linguagem de Grande Porte (LLMs).

## Estrutura de diretórios

```
bot_rosa/
├── bot_rosa/                    # Pacote Python
│   ├── __init__.py             # Inicialização do pacote
│   ├── agent.py                # Implementação principal do agente ROSA
│   ├── llm.py                  # Configuração e setup do LLM
│   └── prompts.py              # Prompts do sistema e configuração de personalidade
├── launch/                     # Arquivos de launch
│   └── rosa.launch.py          # Lançador principal do serviço ROSA
├── scripts/                    # Scripts executáveis
│   └── rosa_service.py         # Servidor de serviço ROSA
├── CMakeLists.txt              # Configuração de build CMake
└── package.xml                 # Manifesto do pacote
```

## Explicação dos arquivos

### Componentes principais

#### `bot_rosa/agent.py`
Implementação principal do agente ROSA que fornece:

- **Integração com ROSA**: Estende o framework ROSA para funcionalidades específicas do robô
- **Integração com LLM**: Conecta a modelos de linguagem para processamento de linguagem natural
- **Integração ROS2**: Gerencia serviços e tópicos ROS2 para controle do robô
- **Interface interativa**: Fornece capacidades de interação conversacional
- **Carregamento dinâmico de ferramentas**: Carrega ferramentas específicas do robô a partir do pacote correspondente (ex.: ferramentas para um robô Go2 são carregadas do pacote go2_pkg)

#### `bot_rosa/llm.py`
Configuração e setup do LLM:

- **Configuração de modelo**: Configuração centralizada para seleção do modelo de linguagem
- **Configuração do provedor de LLM**: Configura o backend do LLM (OpenAI, Ollama, etc.)
- **Configurações de temperatura**: Controla aleatoriedade e criatividade das respostas

#### `bot_rosa/prompts.py`
Prompts do sistema e configuração de personalidade:

- **Prompts do sistema**: Define o comportamento e as capacidades do assistente de IA
- **Contexto do robô**: Fornece contexto sobre o ambiente e restrições do robô
- **Diretrizes de resposta**: Define regras de como o agente deve responder aos usuários

### Arquivos de launch

#### `launch/rosa.launch.py`
Lançador principal do serviço ROSA:

- **Servidor de serviço**: Inicia o serviço ROSA para interação externa
- **Configuração do robô**: Lê a configuração do robô em robot_config.yaml
- **Suporte a namespace**: Usa o nome do robô para implantação multi-robô

### Scripts

#### `scripts/rosa_service.py`
Implementação do servidor de serviço ROSA:

- **Interface de serviço**: Fornece o serviço LLMPrompt para consultas externas
- **Integração do agente**: Conecta ao agente ROSA para processar requisições
- **Tratamento de respostas**: Retorna respostas geradas por IA para clientes do serviço

---

**Nota**: Este pacote exige acesso adequado a APIs de LLM e hardware do robô. Garanta que as credenciais da OpenAI estejam configuradas e que o robô esteja devidamente conectado antes do uso.
