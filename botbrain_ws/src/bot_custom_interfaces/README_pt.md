# bot_custom_interfaces

O pacote de interfaces ROS2 customizadas para a plataforma de robôs BotBrain. Este pacote define todos os tipos de mensagens customizadas, definições de serviços e interfaces de ação usadas em todo o sistema do robô.

## Descrição

O pacote `bot_custom_interfaces` funciona como o repositório central para todas as interfaces de comunicação ROS2 customizadas usadas pela plataforma do robô R1. Ele fornece tipos de mensagem padronizados para controle do robô, relatórios de status, gerenciamento de dispositivos e comunicação do sistema em todos os pacotes do robô.

## Estrutura de diretórios

```
bot_custom_interfaces/
├── msg/                          # Definições de mensagens customizadas
├── srv/                          # Definições de serviços customizados
├── action/                       # Definições de ações customizadas
├── CMakeLists.txt                # Configuração de build CMake
├── package.xml                   # Manifesto do pacote ROS2
└── README.md                     # Este arquivo
```

## Adicionando novas interfaces

### Adicionando uma nova mensagem

1. **Criar arquivo de mensagem**: Crie um novo arquivo `.msg` no diretório `msg/`
   ```msg
   # Exemplo: MyCustomMessage.msg
   std_msgs/Header header
   string data
   int32 value
   ```

2. **Atualizar CMakeLists.txt**: Adicione a mensagem na chamada `rosidl_generate_interfaces`
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... interfaces existentes ...
     "msg/MyCustomMessage.msg"
   )
   ```

3. **Build e teste**: Compile o pacote e verifique se a interface foi gerada
   ```bash
   colcon build --packages-select bot_custom_interfaces
   ros2 interface show bot_custom_interfaces/msg/MyCustomMessage
   ```

### Adicionando um novo serviço

1. **Criar arquivo de serviço**: Crie um novo arquivo `.srv` no diretório `srv/`
   ```srv
   # Exemplo: MyCustomService.srv
   # Request
   string command
   int32 value
   ---
   # Response
   bool success
   string message
   ```

2. **Atualizar CMakeLists.txt**: Adicione o serviço na chamada `rosidl_generate_interfaces`
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... interfaces existentes ...
     "srv/MyCustomService.srv"
   )
   ```

### Adicionando uma nova ação

1. **Criar arquivo de ação**: Crie um novo arquivo `.action` no diretório `action/`
   ```action
   # Exemplo: MinhaAcao.action
   # Goal
   string goal_data
   ---
   # Result
   bool success
   string result_message
   ---
   # Feedback
   float32 progress
   string status
   ```

2. **Atualizar CMakeLists.txt**: Adicione a ação na chamada `rosidl_generate_interfaces`
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... interfaces existentes ...
     "action/MinhaAcao.action"
   )
   ```

### Convenções de nomenclatura de interfaces

- **Mensagens**: Use nomes descritivos terminando com o tipo de dado (ex.: `RobotStatus.msg`, `DeviceInfo.msg`)
- **Serviços**: Use nomes orientados a ação (ex.: `StartMapping.srv`, `SwitchGait.srv`)
- **Ações**: Use nomes orientados a ação terminando com "Action" (ex.: `NavigateAction.action`)
- **Campos**: Use snake_case para nomes de campos (ex.: `robot_status`, `device_id`)

### Diretrizes de design de interfaces

1. **Mantenha as interfaces simples**: Use tipos básicos do ROS2 quando possível
2. **Use tipos padrão**: Prefira tipos `std_msgs` em vez de tipos customizados
3. **Inclua headers**: Adicione `std_msgs/Header` para mensagens com timestamp
4. **Forneça feedback**: Inclua informações de sucesso/erro nas respostas de serviço
5. **Documente campos**: Adicione comentários explicando o objetivo dos campos
6. **Compatibilidade de versões**: Considere compatibilidade retroativa ao modificar interfaces

## Dependências

### Dependências de build
- `ament_cmake` - Sistema de build CMake
- `std_msgs` - Tipos de mensagem padrão ROS2
- `action_msgs` - Tipos de ação padrão ROS2
- `rosidl_default_generators` - Geração de interfaces

### Dependências de runtime
- `rosidl_default_runtime` - Suporte de runtime para interfaces

---

**Nota**: Este pacote é fundamental para todo o sistema do robô. Mudanças nas interfaces podem quebrar compatibilidade com pacotes existentes. Sempre teste com cuidado e considere compatibilidade retroativa ao modificar interfaces existentes.
