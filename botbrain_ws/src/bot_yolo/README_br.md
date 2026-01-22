<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_yolo - ROS2 YOLO Inferência + Rastreamento (Tracking) (Ultralytics / TensorRT)

[🇺🇸 English Version](README.md)

Um pacote ROS 2 que executa o Ultralytics YOLO com um engine TensorRT para detecção de objetos rápida (e tracking opcional). Ele assina uma `sensor_msgs/Image` de uma câmera, realiza a inferência, publica uma imagem anotada (raw e compressed) e publica as detecções como JSON compacto.

## Recursos

- **Arquitetura de Lifecycle**
- **Exportação TensorRT na primeira execução**: exporta `.engine` a partir de um `.pt` se estiver ausente
- **Detecção ou rastreamento**: `predict()` ou `track()` (BoT-SORT via cfg do tracker do Ultralytics)
- **Saídas anotadas**: publica `Image` anotada e `CompressedImage` JPEG
- **Detecções em JSON**: publica JSON compacto para fácil integração
- **Configuração em YAML**: parâmetros ajustáveis pelo usuário ficam em `config/yolo.yaml` 

## Sumário

- [Recursos](#recursos)
- [Instalação](#instalação)
- [Início Rápido](#início-rápido)
- [Configuração](#configuração)
- [Tópicos Publicados](#tópicos-publicados)
- [Gerenciamento Lifecycle](#gerenciamento-lifecycle)
- [Estrutura do Diretório](#estrutura-do-diretório)
- [Dependências](#dependências)
- [Solução de Problemas](#solução-de-problemas)
- [Créditos](#créditos)
- [Aviso de Licença](#aviso-de-licença)

## Instalação

### Pré-requisitos

- ROS 2 (Humble ou superior)
- Python 3
- OpenCV + cv_bridge
- Ultralytics
- NVIDIA GPU runtime adequado para TensorRT (necessário para exportação / inferência em .engine)

### Instalar dependências Python
OBS: Se você está trabalhando com o ```botbrain_ws```, você pode pular esta etapa

```bash
pip3 install ultralytics

Compilar o pacote

cd ~/seu_workspace_ros2
colcon build --packages-select bot_yolo
source install/setup.bash
```

## Início rápido

### Executar o nó

```bash
ros2 launch bot_yolo yolo.launch.py
```
### Configurar e ativar (Lifecycle)

O nó inicia no estado **unconfigured** (não configurado). Para rodar a inferência:
```bash
ros2 lifecycle set /yolo_node configure
ros2 lifecycle set /yolo_node activate
```
Inspecionar estado do nó
```bash
ros2 lifecycle get /yolo_node
```
Inspecionar saídas
```bash
# imagem anotada (raw)
ros2 topic echo /yolo/image

# imagem anotada (comprimida)
ros2 topic echo /yolo/image_compressed

# detecções em JSON
ros2 topic echo /yolo/detections
```

## Configuração

O principal arquivo de configuração é:
- config/yolo.yaml

Esse arquivo contém os parâmetros que a maioria dos usuários vai alterar: tópico de entrada, ajustes de inferência, tracking e configurações de overlay.

### Configuração padrão
```
yolo_node:
  ros__parameters:
    # Subscrição
    camera_topic: "front_camera/color/image_raw" #coloque seu próprio tópico de câmera

    # Inferência
    imgsz: 640 #tamanho da imagem
    conf: 0.25 #score de confiança
    device: 0 #para CUDA

    # Tracking
    use_tracking: true
    tracker_cfg: "botsort.yaml"

    # Desenho
    draw_labels: true
    label_every_n: 1 #pula n frames de desenho (para eficiência)
    line_thickness: 2
    font_scale: 0.5
    font_thickness: 1
```

Comportamento:
- Se engine_path não existir, o nó exporta um engine TensorRT a partir de pt_path durante o **configure**.

## Tópicos publicados

| **Tópico** | **Tipo** | **Descrição** |
| --- | --- | --- |
| /yolo/image | sensor_msgs/Image | Imagem anotada (raw) |
| /yolo/image_compressed | sensor_msgs/CompressedImage | Imagem anotada em JPEG (redimensionada + comprimida) |
| /yolo/detections | std_msgs/String | Payload JSON com as detecções |

### Formato do JSON de detecções

Publicado em /yolo/detections como uma string JSON:
```
{
  "detections_num": "2",
  "detected_objects": [
    {"object_id":"0","object":"person","confidence":"0.932","track_id":12},
    {"object_id":"2","object":"car","confidence":"0.811","track_id":null}
  ]
}
```
## Gerenciamento Lifecycle

### Estados do lifecycle:
- **Unconfigured**: nenhum recurso de inferência alocado
- **Inactive**: configurado, mas sem rodar inferência
- **Active**: inferência rodando, publicando saídas
- **Finalized**: limpo e encerrado

### Comandos de lifecycle:
```bash
ros2 lifecycle set /yolo_node configure
ros2 lifecycle set /yolo_node activate
ros2 lifecycle set /yolo_node deactivate
ros2 lifecycle set /yolo_node cleanup
ros2 lifecycle set /yolo_node shutdown
```
## Estrutura do diretório
```
bot_yolo/
├── bot_yolo/                 # Pacote Python
│   ├── __init__.py
│   └── yolo_node.py
├── config/
│   └── yolo.yaml
├── launch/
│   └── yolo.launch.py
├── resource/
│   └── bot_yolo
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```
## Dependências

Pacotes ROS 2
- rclpy
- sensor_msgs
- std_msgs
- cv_bridge

Python / Externas
- numpy (1.23)
- opencv-python (ou OpenCV do sistema)
- ultralytics

## Solução de problemas

### Falha ao exportar o engine

- Verifique se pt_path existe
- Garanta que o runtime do TensorRT / CUDA está instalado e compatível
- Verifique a memória disponível da GPU (a exportação do engine pode exigir bastante VRAM)

### Tópico de entrada não corresponde

- Confirme o nome do tópico da câmera em config/yolo.yaml
- Garanta que o tipo da mensagem de entrada é sensor_msgs/Image

## Créditos

Este projeto utiliza **modelos e APIs YOLO da Ultralytics**.
- Ultralytics YOLO: https://github.com/ultralytics/ultralytics
- © Ultralytics, licenciado sob a licença AGPL-3.0

## Aviso de Licença

O Ultralytics YOLO é licenciado sob a licença **AGPL-3.0**.  
Este repositório não inclui o código-fonte da Ultralytics; ele depende do pacote Python `ultralytics`.

<p align="center">Feito com ❤️ no Brasil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Ícone do Bot" width="110">
</p>
