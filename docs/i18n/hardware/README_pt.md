# Manual de Montagem do Hardware BotBrain

Carcaças imprimíveis em 3D e peças de interface para robôs do BotBrain. Este guia cobre impressão, lista de materiais e instruções de montagem passo a passo.

<p align="center">
  <img src="../../images/assembly.gif" alt="Montagem do BotBrain" width="600">
</p>

<p align="center">
  <a href="../../../hardware/README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-green" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

> **Nota:** A versão em inglês é a documentação oficial e mais atualizada. Esta tradução pode não refletir as últimas alterações.

---

## Índice

- [Visão Geral](#visão-geral)
- [Requisitos Gerais](#requisitos-gerais)
- [Configurações de Impressão](#configurações-de-impressão)
- [Montagem do BotBrain](#montagem-do-botbrain)
- [Montagens de Interface do Robô](#montagens-de-interface-do-robô)
  - [Interface Unitree G1](#interface-unitree-g1)
  - [Interface Unitree Go2](#interface-unitree-go2)
  - [Interface Direct Drive Tita](#interface-tita)

---

## Visão Geral

O hardware do BotBrain consiste em dois componentes principais:

1. **BotBrain** - A carcaça principal que abriga toda a eletrônica
2. **Interface do Robô** - Um adaptador de montagem específico para sua plataforma de robô

Você precisará montar o BotBrain primeiro e depois conectá-lo à interface de robô apropriada para sua plataforma.

### Estrutura de Diretórios

```
hardware/
├── BotBrain/          # Arquivos da carcaça principal
├── G1/                # Interface Unitree G1
├── Go2/               # Interface Unitree Go2
└── Tita/              # Interface Direct Drive Tita
```

### Formatos de Arquivo

| Formato | Caso de Uso |
|---------|----------|
| **.3mf** | Recomendado para a maioria dos fatiadores (PrusaSlicer, Bambu Studio, Cura) |
| **.stl** | Formato universal, funciona com qualquer fatiador |
| **.step** | Formato CAD para modificações |

---

## Requisitos Gerais

### Ferramentas Necessárias

| Ferramenta | Finalidade |
|------|---------|
| Impressora 3D | Impressão da carcaça e peças de interface |
| Conjunto de chaves de fenda (Phillips/Allen) | Fixação de componentes |
| Descascadores de fio | Preparação de cabos |
| Pinças | Manuseio de pequenos componentes |

### Precauções de Segurança

> **Aviso:** Sempre desconecte a energia antes da montagem ou desmontagem.

- Manuseie os componentes eletrônicos com cuidado para evitar descarga estática (use pulseira ESD se disponível)
- Garanta ventilação adequada ao soldar
- Use óculos de segurança ao remover suportes das impressões
- Verifique duas vezes a polaridade antes de conectar a energia

---

## Configurações de Impressão

Use estas configurações para todas as peças de hardware do BotBrain:

| Configuração | Valor Recomendado | Notas |
|---------|-------------------|-------|
| Material | PLA | PETG também é aceitável para ambientes de temperatura mais alta |
| Altura da Camada | 0.2 mm | Use 0.1 mm para detalhes mais finos |
| Preenchimento | 20-30% | Preenchimento maior para peças estruturais |
| Suportes | Suportes em árvore | - |
| Adesão à Mesa | Brim (opcional) | Ajuda a prevenir empenamento |

---

## Montagem do BotBrain

O BotBrain Core é a carcaça principal que abriga toda a eletrônica. Complete esta montagem antes de conectar qualquer interface de robô.

[Vídeo Completo de Montagem](https://youtu.be/xZ5c619bTEQ) - Tutorial completo em vídeo passo a passo do processo de montagem do BotBrain

### Lista de Materiais - BotBrain

#### Peças Impressas em 3D

| Peça | Quantidade | Arquivo | Notas |
|------|----------|------|-------|
| Carcaça Superior | 1 | [BotBrain/top_case.stl](../../../hardware/BotBrain/top_case.stl) | Tampa principal |
| Carcaça Inferior | 1 | [BotBrain/bottom_case.stl](../../../hardware/BotBrain/bottom_case.stl) | Alojamento dos componentes |

#### Eletrônica

| Componente | Quantidade | Notas |
|-----------|----------|-------|
| Jetson Orin Nano | 1 | Sem Base |
| Cabo USB-A/USB-C | 2 | Comprimento de 15cm para melhor encaixe |
| Câmera RealSense | 2 | D435i |
| Conversor DC-DC 12V | 1 | Para câmeras D435i |
| Conector WAGO | 2 | 2 vias |
| Pigtail Jack Barrel | 1 | - |

#### Fixadores e Hardware

| Item | Quantidade | Notas |
|------|----------|-------|
| M3x10 Autoatarraxante | 4 | Preferencialmente Allen/Hexagonal |
| Arruela Plana M3 | 4 | - |

### Etapas de Montagem - BotBrain Core

#### Etapa 1: Imprima as Peças da Carcaça

Imprima tanto a carcaça superior quanto a inferior usando as [configurações de impressão](#configurações-de-impressão) acima.

---

#### Etapa 2: Prepare a Carcaça Inferior

Remova todo o material de suporte e limpe quaisquer bordas ásperas.

**Tarefas:**
1. Remova todo o material de suporte com alicate de corte
2. Lixe quaisquer bordas ou saliências ásperas
3. Verifique se todos os pontos de montagem estão livres

---

#### Etapa 3: Instale os Componentes Eletrônicos

**Tarefas:**

1. Coloque os parafusos nos suportes de montagem na carcaça inferior
2. Conecte os conectores WAGO aos terminais de entrada e saída do conversor DC-DC de 12V
3. Monte o conversor de 12V no local designado na carcaça inferior
4. Conecte o pigtail do jack barrel ao conector WAGO de saída do conversor DC
5. Coloque as câmeras RealSense em suas posições de montagem com os cabos USB pré-conectados
6. Coloque o Jetson Orin Nano na posição correta, direcionando as antenas WiFi/Bluetooth para o bolso lateral
7. Conecte os cabos USB das câmeras RealSense ao Jetson Orin Nano
8. Conecte o cabo de energia (jack barrel) à entrada de energia do Jetson Orin Nano

---

#### Etapa 4: Feche a Carcaça

Conecte a carcaça superior para completar a montagem do BotBrain.

**Tarefas:**
1. Alinhe a carcaça superior com a carcaça inferior
2. Pressione suavemente para baixo até que os encaixes se prendam

> **Abrindo a carcaça:** Para reabrir, flexione suavemente as laterais da seção inferior para liberar os encaixes de montagem.

---

## Montagens de Interface do Robô

Escolha o guia de montagem de interface para sua plataforma de robô específica.

---

### Interface Unitree G1

Interface de montagem para o robô humanoide Unitree G1.

#### Lista de Materiais - Interface G1

##### Peças Impressas em 3D

| Peça | Quantidade | Arquivo |
|------|----------|------|
| Suporte de Interface G1 | 1 | [G1/g1_interface.stl](../../../hardware/G1/g1_interface.stl) |


##### Fixadores e Hardware

| Item | Quantidade | Tamanho/Tipo | Notas |
|------|----------|-----------|-------|
| M6x30 | 4 | Preferencialmente Allen/Hexagonal |
| Arruela de Pressão M6 | 4 | - |

##### Componentes Adicionais

| Item | Quantidade | Notas |
|------|----------|-------|
| Cabo Ethernet | 1 | - |
| Pigtail XT-30 | 1 | - |

#### Etapas de Montagem - Interface G1

> ***Nota:** Mais fácil de montar com o robô deitado, com as costas para cima*

##### Etapa 1: Imprima a Interface

Imprima o suporte de interface G1 usando as [configurações de impressão](#configurações-de-impressão).

---

##### Etapa 2: Prepare a Interface

**Tarefas:**
1. Remova todo o material de suporte
2. Lixe quaisquer bordas ou saliências ásperas
3. Verifique se todos os pontos de montagem estão livres

---

##### Etapa 3: Monte no Robô

**Tarefas:**
1. Aperte o BotBrain na interface com 4 parafusos autoatarraxantes M3
  ![g1_mount_01](../../images/mechanics/g1_mount_01.png)

2. Remova a proteção adesiva dos furos de montagem traseiros
3. Conecte os cabos ethernet e de energia ao robô
4. Passe os cabos pela abertura do painel
  ![g1_mount_04](../../images/mechanics/g1_mount_04.png)

5. Coloque a interface nas costas do robô, alinhando os furos dos parafusos

6. Aperte a interface no robô usando parafusos M6x30
  ![g1_mount_06](../../images/mechanics/g1_mount_06.png)

---

##### Etapa 4: Conecte o BotBrain

**Tarefas:**
1. Conecte os cabos de energia e ethernet ao BotBrain
2. Posicione o BotBrain usando os pinos de alinhamento e aperte os parafusos

---

### Interface Unitree Go2

Interface de montagem para o robô quadrúpede Unitree Go2.

#### Lista de Materiais - Interface Go2

##### Peças Impressas em 3D

| Peça | Quantidade | Arquivo | Notas |
|------|----------|------|-------|
| Suporte de Interface Go2 | 1 | [Go2/go2_interface.stl](../../../hardware/Go2/go2_interface.stl) | Suporte de montagem principal |

##### Fixadores e Hardware

| Item | Quantidade | Notas |
|------|----------|-------|
| M3x30 | 2 | Preferencialmente Allen/Hexagonal |
| M3x20 | 2 | Preferencialmente Allen/Hexagonal |
| Arruela de Pressão M3 | 4 | - |

##### Componentes Adicionais

| Item | Quantidade | Notas |
|------|----------|-------|
| Cabo Ethernet | 1 | - |
| Pigtail XT-30 | 1 | - |
|

#### Etapas de Montagem - Interface Go2

##### Etapa 1: Imprima a Interface

Imprima o suporte de interface Go2 usando as [configurações de impressão](#configurações-de-impressão).

---

##### Etapa 2: Prepare a Interface

**Tarefas:**
1. Remova todo o material de suporte
2. Lixe quaisquer bordas ou saliências ásperas
3. Verifique se todos os pontos de montagem estão livres

---

##### Etapa 3: Monte no Robô

**Tarefas:**
1. Desparafuse os parafusos da tampa superior
  ![go2_mount_01](../../images/mechanics/go2_mount_01.png)

2. Conecte os cabos ethernet e de energia ao robô
  ![go2_mount_02](../../images/mechanics/go2_mount_02.png)

3. Passe os cabos pela abertura da interface
  ![go2_mount_03](../../images/mechanics/go2_mount_03.png)

4. Coloque a interface nas costas do robô e aperte os parafusos M3x30 na flange frontal e o parafuso M3x20 na flange traseira
  ![go2_mount_04](../../images/mechanics/go2_mount_04.png)

5. Aperte o BotBrain na interface com 4 parafusos autoatarraxantes M3
  ![go2_mount_05](../../images/mechanics/go2_mount_05.png)

---

##### Etapa 4: Conecte o BotBrain

**Tarefas:**
1. Conecte os cabos de energia e ethernet ao BotBrain
2. Posicione o BotBrain usando os pinos de alinhamento e aperte os parafusos

---

### Interface Tita

Interface de montagem para o robô Legged Robotics Tita.

#### Lista de Materiais - Interface Tita

##### Peças Impressas em 3D

| Peça | Quantidade | Arquivo | Notas |
|------|----------|------|-------|
| Suporte de Interface Tita | 1 | [Tita/tita_interface2.stl](../../../hardware/Tita/tita_interface.stl) | Suporte de montagem principal |

##### Fixadores e Hardware

| Item | Quantidade | Notas |
|------|----------|-------|
| M4x35 | 1 | Preferencialmente Allen/Hexagonal |
| Arruela plana M4 | 1 | - |

##### Componentes Adicionais

| Item | Quantidade | Notas |
|------|----------|-------|
| Cabo DB25 para Ethernet/Energia | 1 | Feito sob medida |

> ***Nota**: O cabo de energia e comunicação para o robô Tita usa um conector DB25 no lado do Tita. Este cabo pode ser feito/soldado usando este [esquema elétrico do Tita](../../tita_conn_sch.pdf) como referência.*

#### Etapas de Montagem - Interface Tita

##### Etapa 1: Imprima a Interface

Imprima o suporte de interface Tita usando as [configurações de impressão](#configurações-de-impressão).

---

##### Etapa 2: Prepare a Interface

**Tarefas:**
1. Remova todo o material de suporte
2. Lixe quaisquer bordas ásperas

---

##### Etapa 3: Monte no Robô

**Tarefas:**

1. Deslize a interface nos trilhos do Tita até que o furo do parafuso da interface esteja alinhado com o ponto de montagem do Tita
![tita_mount_01](../../images/mechanics/tita_mount_01.png)

2. Passe o cabo com os conectores de energia e ethernet pela abertura frontal da interface
![tita_mount_02](../../images/mechanics/tita_mount_02.png)

3. Conecte o cabo Ethernet ao Botbrain e aperte 4 parafusos autoatarraxantes M3x10 para garantir a estabilidade do componente
![tita_mount_03](../../images/mechanics/tita_mount_03.png)

---

##### Etapa 4: Conecte o BotBrain

**Tarefas:**
1. Conecte os cabos de energia e ethernet ao BotBrain
2. Posicione o BotBrain usando os pinos de alinhamento e prenda todos os parafusos

---
