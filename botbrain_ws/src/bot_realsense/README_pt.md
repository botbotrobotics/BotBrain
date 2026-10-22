<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_realsense

**Um nó de ciclo de vida por câmera Intel RealSense D400: tópicos ROS para SLAM, Nav2 e YOLO, mais vídeo H.264 para o navegador.**

`bot_realsense` é dono da câmera e atende a todos a partir de uma única captura. Ele substitui o `realsense2_camera`, o nó Python de JPEG e o JPEG via rosbridge.

- **Tópicos ROS** com os mesmos nomes do `realsense2_camera`, então RTAB-Map, `depthimage_to_laserscan` e YOLO não precisam de mudanças. Cada tópico só é calculado enquanto tem assinantes, no máximo na taxa configurada.
- **Vídeo no navegador**: cor e profundidade em tons de cinza são codificadas uma única vez em H.264 e servidas via RTSP em `127.0.0.1`. O MediaMTX (`config/mediamtx.yml`) só puxa um stream enquanto um navegador o assiste e o entrega a todos os espectadores via WebRTC (WHEP). Um codificador só roda enquanto seu stream tem espectador.

## Propósito do Pacote

- **Captura**: API por sensor da librealsense com formatos nativos (`format-conversion: basic`): profundidade Z16, cor YUYV. Sem `rs2::pipeline` e sem a colorização ou dizimação da librealsense.
- **Repasse**: os callbacks da librealsense apenas colocam o quadro mais novo num slot de um quadro. Uma thread por sensor faz o trabalho, então um consumidor lento descarta quadros em vez de enfileirá-los.
- **Sob demanda**: sem assinantes e sem espectadores não há conversão nem codificação, apenas captura.
- **Ciclo de vida**: integra-se ao `bot_state_machine` como qualquer nó de ciclo de vida do BotBrain.
- **Recuperação**: se a câmera for desconectada, ou não entregar quadros por `frame_timeout_s`, o nó desativa e faz cleanup pelas suas próprias transições de ciclo de vida, para que a máquina de estados as veja. Quando a câmera volta, ele configura e ativa de novo.

Testado com a D435i. Espera-se que a D455 e a D555 funcionem se oferecerem os modos configurados. Quando não oferecem, o configure falha listando os modos disponíveis.

---

## Ciclo de Vida

| Transição | Efeito |
|---|---|
| configure | Abre a câmera pelo número de série (`initial_reset` opcional), lê intrínsecos, extrínsecos e escala de profundidade, cria os publicadores, publica o TF estático e monta o servidor RTSP (ainda sem codificar). Parâmetros inválidos, câmera ausente ou modo inexistente resultam em FAILURE com uma linha de log. |
| activate | Abre e inicia os dois sensores, as threads de trabalho e o servidor RTSP. |
| deactivate | Para e fecha os dois sensores (a câmera fica ociosa e seus `/dev/video*` são fechados), aguarda as threads e para o servidor RTSP. |
| cleanup | Libera a câmera, os publicadores e o streamer. |

O nó nunca chama `exit()` e só reinicia o hardware quando `initial_reset` é verdadeiro. Com `serial_no` definido, a câmera é procurada em todas as linhas de produto RealSense. Sem ele, usa-se a primeira D400 ou, na falta dela, a primeira D500.

Enquanto o nó está ativo, um timer verifica duas coisas: se a câmera continua conectada e se os dois sensores entregaram um quadro dentro de `frame_timeout_s`. Se alguma falhar, o nó registra um erro e executa deactivate e cleanup. Depois verifica a cada 2 s se a câmera voltou. Quando volta, executa configure e activate. Uma tentativa que falha espera mais antes da próxima, até 30 s.

## Tópicos

Os tópicos são relativos ao nó, `/<namespace>/<nó>/...`. As imagens só são publicadas enquanto há assinantes, com QoS confiável e keep-last-1.

| Tópico | Tipo | Observações |
|---|---|---|
| `color/image_raw` | `sensor_msgs/Image` `rgb8` | YUYV convertido com a mesma matemática BT.601 da librealsense |
| `color/camera_info` | `sensor_msgs/CameraInfo` | |
| `depth/image_rect_raw` | `sensor_msgs/Image` `16UC1` | milímetros |
| `depth/camera_info` | `sensor_msgs/CameraInfo` | |
| `aligned_depth_to_color/image_raw` | `sensor_msgs/Image` `16UC1` | profundidade registrada na imagem de cor, só com `align_depth` |
| `aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | intrínsecos da cor |
| `/tf_static` | | `<tf_prefix><camera_name>_link` → `_depth_frame` → `_depth_optical_frame`, `_link` → `_color_frame` → `_color_optical_frame` (convenção do realsense-ros) |

O carimbo de tempo do cabeçalho é o instante de captura no relógio do host (tempo global da librealsense). Imagens de profundidade usam o frame id `_depth_optical_frame`; imagens de cor e alinhadas usam `_color_optical_frame`.

A profundidade alinhada é publicada uma vez para cada quadro de cor publicado, com o carimbo de tempo exato desse quadro de cor, então o RTAB-Map recebe pares com carimbos idênticos. Ela é calculada a partir do quadro de profundidade mais próximo no tempo. A thread de cor escolhe esses quadros na taxa `color_rate`, e o tópico segue essas escolhas mesmo quando só a profundidade alinhada tem assinantes. O tópico de profundidade mantém sua própria `depth_rate`.

A profundidade alinhada é calculada pelo nosso próprio alinhador. Ele coincide com a implementação NEON do `rs2::align` com diferença de até 1 mm em mais de 99,99% dos pixels válidos de quadros reais da D435i. No Orin NX ele leva cerca de 70% do tempo do `rs2::align`. Por ser código nosso, não aloca memória por quadro.

## Streams de vídeo

- `rtsp://127.0.0.1:<stream_rtsp_port>/<nó>/color`: cor, H.264 constrained baseline, CBR, IDR a cada `stream_gop_s`.
- `rtsp://127.0.0.1:<stream_rtsp_port>/<nó>/depth`: profundidade dizimada 2x2 em luma cinza, com faixa completa sinalizada no SPS.
  - 0 significa sem profundidade. Uma profundidade válida z vira t = (zmin^-0.5 - z^-0.5) / (zmin^-0.5 - zmax^-0.5), e Y = 16 + round(219 t).
  - O navegador inverte esse mapeamento e colore a profundidade.
- O codificador é o `nvv4l2h264enc` onde há NVENC (Orin NX/AGX) e o `x264enc` nos demais (Orin Nano).

Os navegadores acessam os streams pelo MediaMTX em `http://<robô>:8889/<nó>/<color|depth>/whep`.

## Parâmetros

Todos os parâmetros são lidos no configure e são somente leitura em tempo de execução: o `ros2 param set` é recusado em vez de ser ignorado em silêncio. Defina-os no arquivo de launch.

| Nome | Padrão | Significado |
|---|---|---|
| `serial_no` | `""` | número de série; `""` = primeira D400 encontrada. Também aceita um inteiro ou uma string com prefixo `_` |
| `camera_name` | nome do nó | nome base dos frames TF |
| `tf_prefix` | `""` | prefixo literal dos nomes de frame |
| `initial_reset` | `false` | reinicia o hardware uma vez no configure |
| `depth_width` / `depth_height` / `depth_fps` | 848 / 480 / 30 | modo de profundidade (Z16) |
| `color_width` / `color_height` / `color_fps` | 640 / 480 / 30 | modo de cor (YUYV) |
| `color_rate` / `depth_rate` | 0.0 | taxa máxima de publicação em Hz, 0 = todo quadro; a profundidade alinhada segue `color_rate` |
| `frame_timeout_s` | 5.0 | segundos sem quadros de algum sensor, com o nó ativo, até o nó se recuperar |
| `align_depth` | `true` | oferece o tópico de profundidade alinhada |
| `stream_enable` | `true` | serve os streams RTSP |
| `stream_rtsp_port` | 8560 | porta RTSP, uma por nó de câmera |
| `stream_encoder` | `auto` | `auto`, `nvenc` ou `x264` |
| `stream_color_fps` / `stream_depth_fps` | 15 / 15 | quadros codificados por segundo |
| `stream_color_bitrate` / `stream_depth_bitrate` | 1000000 / 2000000 | bits por segundo |
| `stream_gop_s` | 1.0 | intervalo entre IDRs em segundos |
| `depth_min_m` / `depth_max_m` | 0.3 / 8.0 | faixa da profundidade em cinza |
| `auto_exposure_priority` | `false` | sensor de cor; `false` mantém a taxa de quadros com pouca luz |

## Execução

```bash
ros2 run bot_realsense bot_realsense_node --ros-args -r __node:=front_camera -p camera_name:=front_camera
ros2 lifecycle set /front_camera configure
ros2 lifecycle set /front_camera activate
```

O nó também é um componente, `bot_realsense::RealsenseNode`. No BotBrain, ele é lançado por `bot_localization/launch/realsense.launch.py`, e o `bot_state_machine` conduz suas transições.

## Compilação e testes

```bash
colcon build --packages-select bot_realsense      # Release, a menos que CMAKE_BUILD_TYPE seja definido
colcon test --packages-select bot_realsense && colcon test-result --verbose
```

Dependências:
- librealsense2 2.55 ou mais recente, pelas configurações de contexto que selecionam `format-conversion: basic`. O `ros-humble-librealsense2` do Humble é mais recente que isso.
- GStreamer 1.20 ou mais recente com gst-rtsp-server, mais os plugins base, good e ugly.
- No Jetson, os plugins multimídia do L4T fornecem o NVENC.
- No BotBrain, o `docker/bot_realsense.Dockerfile` adiciona essas dependências à imagem base. O Compose a constrói como `camera_base` no primeiro uso, e os serviços `builder_base` e `localization` rodam sobre ela.

Os testes unitários cobrem as partes puras:
- o limitador de taxa;
- YUYV para RGB;
- a conversão para milímetros;
- o alinhador comparado ao `rs2::align` num dispositivo de software;
- o slot de repasse de quadros;
- a codificação de profundidade em cinza e a dizimação;
- o agendamento de novas tentativas da recuperação e o pareamento de quadros de cor e profundidade.
