# Manual de Ensamblaje de Hardware BotBrain

Carcasas imprimibles en 3D y piezas de interfaz de robot para BotBrain. Esta guía cubre impresión, lista de materiales e instrucciones de ensamblaje paso a paso.

<p align="center">
  <img src="../../images/assembly.gif" alt="Ensamblaje BotBrain" width="600">
</p>

<p align="center">
  <a href="../../../hardware/README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-blue" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-green" alt="Español"></a>
</p>

> **Nota:** La versión en inglés es la documentación oficial y más actualizada. Esta traducción puede no reflejar los últimos cambios.

---

## Tabla de Contenidos

- [Descripción General](#descripción-general)
- [Requisitos Generales](#requisitos-generales)
- [Configuración de Impresión](#configuración-de-impresión)
- [Ensamblaje de BotBrain](#ensamblaje-de-botbrain)
- [Ensamblajes de Interfaz de Robot](#ensamblajes-de-interfaz-de-robot)
  - [Interfaz Unitree G1](#interfaz-unitree-g1)
  - [Interfaz Unitree Go2](#interfaz-unitree-go2)
  - [Interfaz Direct Drive Tita](#interfaz-tita)

---

## Descripción General

El hardware de BotBrain consta de dos componentes principales:

1. **BotBrain** - La carcasa principal que aloja toda la electrónica
2. **Interfaz de Robot** - Un adaptador de montaje específico para tu plataforma de robot

Necesitarás ensamblar primero el BotBrain y luego conectarlo a la interfaz de robot apropiada para tu plataforma.

### Estructura de Directorios

```
hardware/
├── BotBrain/          # Archivos de la carcasa principal
├── G1/                # Interfaz Unitree G1
├── Go2/               # Interfaz Unitree Go2
└── Tita/              # Interfaz Direct Drive Tita
```

### Formatos de Archivo

| Formato | Caso de Uso |
|--------|----------|
| **.3mf** | Recomendado para la mayoría de los slicers (PrusaSlicer, Bambu Studio, Cura) |
| **.stl** | Formato universal, funciona con cualquier slicer |
| **.step** | Formato CAD para modificaciones |

---

## Requisitos Generales

### Herramientas Requeridas

| Herramienta | Propósito |
|------|---------|
| Impresora 3D | Impresión de carcasa y piezas de interfaz |
| Juego de destornilladores (Phillips/Hex) | Fijación de componentes |
| Pelacables | Preparación de cables |
| Pinzas | Manipulación de componentes pequeños |

### Precauciones de Seguridad

> **Advertencia:** Siempre desconecta la energía antes del ensamblaje o desensamblaje.

- Manipula la electrónica con cuidado para evitar descargas estáticas (usa pulsera ESD si está disponible)
- Asegura una ventilación adecuada al soldar
- Usa gafas de seguridad al remover soportes de las impresiones
- Verifica dos veces la polaridad antes de conectar la energía

---

## Configuración de Impresión

Usa estas configuraciones para todas las piezas de hardware de BotBrain:

| Configuración | Valor Recomendado | Notas |
|---------|-------------------|-------|
| Material | PLA | PETG también es aceptable para ambientes de temperatura más alta |
| Altura de Capa | 0.2 mm | Usa 0.1 mm para detalles más finos |
| Relleno | 20-30% | Mayor relleno para piezas estructurales |
| Soportes | Soportes de árbol | - |
| Adhesión a la Cama | Brim (opcional) | Ayuda a prevenir el alabeo |

---

## Ensamblaje de BotBrain

El BotBrain Core es la carcasa principal que aloja toda la electrónica. Completa este ensamblaje antes de conectar cualquier interfaz de robot.

[Video Completo de Ensamblaje](https://youtu.be/xZ5c619bTEQ) - Tutorial completo en video paso a paso del proceso de ensamblaje de BotBrain

### Lista de Materiales - BotBrain

#### Piezas Impresas en 3D

| Pieza | Cantidad | Archivo | Notas |
|------|----------|------|-------|
| Carcasa Superior | 1 | [BotBrain/top_case.stl](../../../hardware/BotBrain/top_case.stl) | Tapa principal |
| Carcasa Inferior | 1 | [BotBrain/bottom_case.stl](../../../hardware/BotBrain/bottom_case.stl) | Alojamiento de componentes |

#### Electrónica

| Componente | Cantidad | Notas |
|-----------|----------|-------|
| Jetson Orin Nano | 1 | Sin Base |
| Cable USB-A/USB-C | 2 | Longitud de 15cm para mejor ajuste |
| Cámara RealSense | 2 | D435i |
| Convertidor DC-DC 12V | 1 | Para cámaras D435i |
| Conector WAGO | 2 | 2 vías |
| Pigtail Jack Barrel | 1 | - |

#### Sujetadores y Hardware

| Artículo | Cantidad | Notas |
|------|----------|-------|
| M3x10 Autorroscante | 4 | Preferiblemente Hex/Allen |
| Arandela Plana M3 | 4 | - |

### Pasos de Ensamblaje - BotBrain Core

#### Paso 1: Imprime las Piezas de la Carcasa

Imprime tanto la carcasa superior como la inferior usando la [configuración de impresión](#configuración-de-impresión) anterior.

---

#### Paso 2: Prepara la Carcasa Inferior

Remueve todo el material de soporte y limpia los bordes ásperos.

**Tareas:**
1. Remueve todo el material de soporte con cortadores al ras
2. Lija cualquier borde o protuberancia áspera
3. Verifica que todos los puntos de montaje estén despejados

---

#### Paso 3: Instala la Electrónica

**Tareas:**

1. Coloca los tornillos en los puntos de montaje de la carcasa inferior
2. Conecta los conectores WAGO a los terminales de entrada y salida del convertidor DC-DC de 12V
3. Monta el convertidor de 12V en la ubicación designada en la carcasa inferior
4. Conecta el pigtail del jack barrel al conector WAGO de salida del convertidor DC
5. Coloca las cámaras RealSense en sus posiciones de montaje con los cables USB pre-conectados
6. Coloca el Jetson Orin Nano en la posición correcta, dirigiendo las antenas WiFi/Bluetooth hacia el bolsillo lateral
7. Conecta los cables USB de las cámaras RealSense al Jetson Orin Nano
8. Conecta el cable de energía (jack barrel) a la entrada de energía del Jetson Orin Nano

---

#### Paso 4: Cierra la Carcasa

Conecta la carcasa superior para completar el ensamblaje del BotBrain.

**Tareas:**
1. Alinea la carcasa superior con la carcasa inferior
2. Presiona suavemente hacia abajo hasta que los clips se enganchen

> **Abriendo la carcasa:** Para volver a abrir, flexiona suavemente los lados de la sección inferior para liberar los clips de montaje.

---

## Ensamblajes de Interfaz de Robot

Elige la guía de ensamblaje de interfaz para tu plataforma de robot específica.

---

### Interfaz Unitree G1

Interfaz de montaje para el robot humanoide Unitree G1.

#### Lista de Materiales - Interfaz G1

##### Piezas Impresas en 3D

| Pieza | Cantidad | Archivo |
|------|----------|------|
| Soporte de Interfaz G1 | 1 | [G1/g1_interface.stl](../../../hardware/G1/g1_interface.stl) |


##### Sujetadores y Hardware

| Artículo | Cantidad | Tamaño/Tipo | Notas |
|------|----------|-----------|-------|
| M6x30 | 4 | Preferiblemente Hex/Allen |
| Arandela de Presión M6 | 4 | - |

##### Componentes Adicionales

| Artículo | Cantidad | Notas |
|------|----------|-------|
| Cable Ethernet | 1 | - |
| Pigtail XT-30 | 1 | - |

#### Pasos de Ensamblaje - Interfaz G1

> ***Nota:** Más fácil de ensamblar con el robot acostado, con la espalda hacia arriba*

##### Paso 1: Imprime la Interfaz

Imprime el soporte de interfaz G1 usando la [configuración de impresión](#configuración-de-impresión).

---

##### Paso 2: Prepara la Interfaz

**Tareas:**
1. Remueve todo el material de soporte
2. Lija cualquier borde o protuberancia áspera
3. Verifica que todos los puntos de montaje estén despejados

---

##### Paso 3: Monta en el Robot

**Tareas:**
1. Aprieta el BotBrain a la interfaz con 4 tornillos autorroscantes M3
  ![g1_mount_01](../../images/mechanics/g1_mount_01.png)

2. Remueve la protección adhesiva de los agujeros de montaje traseros
3. Conecta los cables ethernet y de energía al robot
4. Pasa los cables a través de la abertura del panel
  ![g1_mount_04](../../images/mechanics/g1_mount_04.png)

5. Coloca la interfaz en la espalda del robot, alineando los agujeros de los tornillos

6. Aprieta la interfaz en el robot usando tornillos M6x30
  ![g1_mount_06](../../images/mechanics/g1_mount_06.png)

---

##### Paso 4: Conecta el BotBrain

**Tareas:**
1. Conecta los cables de energía y ethernet al BotBrain
2. Posiciona el BotBrain usando los pines de alineación y aprieta los tornillos

---

### Interfaz Unitree Go2

Interfaz de montaje para el robot cuadrúpedo Unitree Go2.

#### Lista de Materiales - Interfaz Go2

##### Piezas Impresas en 3D

| Pieza | Cantidad | Archivo | Notas |
|------|----------|------|-------|
| Soporte de Interfaz Go2 | 1 | [Go2/go2_interface.stl](../../../hardware/Go2/go2_interface.stl) | Soporte de montaje principal |

##### Sujetadores y Hardware

| Artículo | Cantidad | Notas |
|------|----------|-------|
| M3x30 | 2 | Preferiblemente Hex/Allen |
| M3x20 | 2 | Preferiblemente Hex/Allen |
| Arandela de Presión M3 | 4 | - |

##### Componentes Adicionales

| Artículo | Cantidad | Notas |
|------|----------|-------|
| Cable Ethernet | 1 | - |
| Pigtail XT-30 | 1 | - |
|

#### Pasos de Ensamblaje - Interfaz Go2

##### Paso 1: Imprime la Interfaz

Imprime el soporte de interfaz Go2 usando la [configuración de impresión](#configuración-de-impresión).

---

##### Paso 2: Prepara la Interfaz

**Tareas:**
1. Remueve todo el material de soporte
2. Lija cualquier borde o protuberancia áspera
3. Verifica que todos los puntos de montaje estén despejados

---

##### Paso 3: Monta en el Robot

**Tareas:**
1. Desatornilla los tornillos de la tapa superior
  ![go2_mount_01](../../images/mechanics/go2_mount_01.png)

2. Conecta los cables ethernet y de energía al robot
  ![go2_mount_02](../../images/mechanics/go2_mount_02.png)

3. Pasa los cables a través de la abertura de la interfaz
  ![go2_mount_03](../../images/mechanics/go2_mount_03.png)

4. Coloca la interfaz en la espalda del robot y aprieta los pernos M3x30 en la brida frontal y el perno M3x20 en la brida trasera
  ![go2_mount_04](../../images/mechanics/go2_mount_04.png)

5. Aprieta el BotBrain a la interfaz con 4 tornillos autorroscantes M3
  ![go2_mount_05](../../images/mechanics/go2_mount_05.png)

---

##### Paso 4: Conecta el BotBrain

**Tareas:**
1. Conecta los cables de energía y ethernet al BotBrain
2. Posiciona el BotBrain usando los pines de alineación y aprieta los tornillos

---

### Interfaz Tita

Interfaz de montaje para el robot Legged Robotics Tita.

#### Lista de Materiales - Interfaz Tita

##### Piezas Impresas en 3D

| Pieza | Cantidad | Archivo | Notas |
|------|----------|------|-------|
| Soporte de Interfaz Tita | 1 | [Tita/tita_interface2.stl](../../../hardware/Tita/tita_interface.stl) | Soporte de montaje principal |

##### Sujetadores y Hardware

| Artículo | Cantidad | Notas |
|------|----------|-------|
| M4x35 | 1 | Preferiblemente Hex/Allen |
| Arandela plana M4 | 1 | - |

##### Componentes Adicionales

| Artículo | Cantidad | Notas |
|------|----------|-------|
| Cable DB25 a Ethernet/Energía | 1 | Hecho a medida |

> ***Nota**: El cable de energía y comunicación para el robot Tita usa un conector DB25 en el lado de Tita. Este cable puede hacerse/soldarse usando este [esquema eléctrico de Tita](../../tita_conn_sch.pdf) como referencia.*

#### Pasos de Ensamblaje - Interfaz Tita

##### Paso 1: Imprime la Interfaz

Imprime el soporte de interfaz Tita usando la [configuración de impresión](#configuración-de-impresión).

---

##### Paso 2: Prepara la Interfaz

**Tareas:**
1. Remueve todo el material de soporte
2. Lija los bordes ásperos

---

##### Paso 3: Monta en el Robot

**Tareas:**

1. Desliza la interfaz en los rieles de Tita hasta que el agujero del tornillo de la interfaz esté alineado con el punto de montaje de Tita
![tita_mount_01](../../images/mechanics/tita_mount_01.png)

2. Pasa el cable con los conectores de energía y ethernet a través de la abertura frontal de la interfaz
![tita_mount_02](../../images/mechanics/tita_mount_02.png)

3. Conecta el cable Ethernet al Botbrain y aprieta 4 tornillos autorroscantes M3x10 para asegurar la estabilidad del componente
![tita_mount_03](../../images/mechanics/tita_mount_03.png)

---

##### Paso 4: Conecta el BotBrain

**Tareas:**
1. Conecta los cables de energía y ethernet al BotBrain
2. Posiciona el BotBrain usando los pines de alineación y asegura todos los tornillos

---
