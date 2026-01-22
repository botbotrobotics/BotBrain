<!-- LOGO -->
<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

<p align="center">
  Un Cerveau, n'importe quel Robot.
</p>

<p align="center">
  <a href="https://botbot.bot"><img src="https://img.shields.io/badge/-Website-000?logo=vercel&logoColor=white" alt="Site Web"></a>
  <a href="https://www.linkedin.com/company/botbotrobotics"><img src="https://img.shields.io/badge/-LinkedIn-0A66C2?logo=linkedin&logoColor=white" alt="LinkedIn"></a>
  <a href="https://www.youtube.com/@botbotrobotics"><img src="https://img.shields.io/badge/-YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube"></a>
  <a href="https://huggingface.co/botbot-ai"><img src="https://img.shields.io/badge/-Hugging%20Face-FFD54F?logo=huggingface&logoColor=black" alt="Hugging Face"></a>
</p>

<h1 align="center">BotBrain ROS2 Workspace</h1>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/License-MIT-purple" alt="Licence: MIT">
  <img src="https://img.shields.io/badge/Platform-Ubuntu_22.04-orange" alt="Ubuntu 22.04">
</p>

<p align="center">
  <a href="../../../botbrain_ws/README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-green" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

> **Note :** La version anglaise est la documentation officielle et la plus à jour. Cette traduction peut ne pas refléter les dernières modifications.

## Vue d'Ensemble

Le **BotBrain Workspace** est un framework ROS2 modulaire et open source pour le contrôle autonome de robots, la navigation et la localisation. Conçu avec une architecture agnostique aux robots, il permet le développement et le déploiement rapides d'applications robotiques avancées sur plusieurs plateformes de robots.

**Caractéristiques Principales :**
- 🤖 **Support Multi-Robot** : Base de code unique pour Go2, Tita, G1 et robots personnalisés
- 🗺️ **SLAM Visuel** : Localisation basée sur RTABMap avec support double caméra
- 🎮 **Modes de Contrôle Multiples** : Joystick, interface web et navigation autonome
- 👁️ **Vision IA** : Détection d'objets YOLOv8/v11
- 🐳 **Prêt pour Docker** : Déploiement conteneurisé avec accélération GPU
- 🔄 **Gestion du Cycle de Vie** : Orchestration robuste des nœuds et récupération des pannes


## Table des Matières

- [Prérequis Matériels](#prérequis-matériels)
- [Démarrage Rapide](#démarrage-rapide)
- [Structure du Dépôt](#structure-du-dépôt)
- [Création d'un Package Robot Personnalisé](#création-dun-package-robot-personnalisé)
- [Vue d'Ensemble des Packages](#vue-densemble-des-packages)
- [Services Docker](#services-docker)
- [Configuration](#configuration)

## Prérequis Matériels

### Plateformes Robot Supportées
- **Unitree Go2**
- **Unitree G1**
- **Tita**
- **Robots Personnalisés** - Suivez le [Guide de Package Robot Personnalisé](#création-dun-package-robot-personnalisé)

### Matériel Requis
- **Plateforme Robot** : Un des robots supportés ci-dessus
- **Ordinateur Embarqué** :
  - Nvidia Jetson Orin Series ou plus récent
- **Capteurs** :
  - Caméras Intel RealSense (pour SLAM visuel)
  - LiDAR (pour SLAM basé sur LiDAR)
- **Réseau** :
  - Connexion Ethernet au robot
  - Adaptateur Wi-Fi (pour contrôle à distance)

### Matériel Optionnel
- **Manette de Jeu** : Pour la téléopération

## Démarrage Rapide

### Lancement avec Docker Compose

Pour le déploiement conteneurisé :

```bash
# Démarrer tous les services
docker compose up -d

# Démarrer des services spécifiques
docker compose up -d state_machine bringup localization navigation

# Voir les logs
docker compose logs -f bringup

# Arrêter les services
docker compose down
```

### Vérifier que le Système Fonctionne

```bash
# Vérifier les nœuds actifs
ros2 node list

# Vérifier les topics
ros2 topic list
```

### Conteneur de Développement

Si vous voulez utiliser la même image docker pour le développement, sans créer un nouveau service, il est possible d'exécuter un conteneur de développement interactif :

```bash
# Initialiser le conteneur de développement
cd botbrain_ws
docker compose up dev -d

# Initialiser un terminal interactif
docker compose exec dev bash
```

Une fois le terminal interactif ouvert, vous pouvez l'utiliser pour créer, compiler et exécuter de nouvelles fonctionnalités qui ne sont pas encore intégrées aux services docker.

## Structure du Dépôt

```
botbrain_ws/
├── README.md                          # Ce fichier
├── LICENSE                            # Licence MIT
│
├── robot_config.yaml                  # Fichier de configuration principal
├── install.sh                         # Script d'installation automatisée
├── robot_select.sh                    # Aide à la sélection du robot
│
├── docker-compose.yaml                # Définition des services Docker
├── botbrain.service                   # Service systemd de démarrage automatique
├── cyclonedds_config.xml              # Configuration du middleware DDS
│
└── src/                               # Packages ROS 2
    │
    ├── Packages Système Central
    │   ├── bot_bringup/               # Lancement principal & coordination twist mux
    │   ├── bot_custom_interfaces/     # Messages, services, actions personnalisés
    │   └── bot_state_machine/         # Gestion du cycle de vie & états
    │
    ├── Modèle Robot & Visualisation
    │   └── bot_description/           # Modèles URDF/XACRO & robot_state_publisher
    │
    ├── Navigation & Localisation
    │   ├── bot_localization/          # SLAM RTABMap (visuel & LiDAR)
    │   └── bot_navigation/            # Stack de navigation Nav2
    │
    ├── Perception & Contrôle
    │   ├── bot_yolo/                  # Détection d'objets YOLOv8/v11
    │   └── joystick-bot/              # Interface manette de jeu
    │
    ├── IA & Surveillance
    │   ├── bot_jetson_stats/          # Surveillance matérielle Jetson
    │   └── bot_rosa/                  # Contrôle par langage naturel ROSA AI
    │
    └── Packages Spécifiques aux Robots
        ├── g1_pkg/                    # Interface matérielle Unitree G1
        ├── go2_pkg/                   # Interface matérielle Unitree Go2
        ├── tita_pkg/                  # Interface matérielle Tita
        └── your_robot_pkg/            # Votre robot personnalisé (voir guide ci-dessous)
```

## Création d'un Package Robot Personnalisé

Pour ajouter le support d'une nouvelle plateforme robot, suivez ce guide en utilisant [go2_pkg](../../../botbrain_ws/src/go2_pkg) comme modèle de référence.

**Note** : Le package go2_pkg communique avec le robot Unitree Go2 via des topics ROS 2 (s'abonnant aux topics ROS 2 natifs d'Unitree et les republiant au format BotBrain). Votre package robot personnalisé peut utiliser une communication similaire basée sur les topics, des APIs matérielles directes ou des interfaces SDK selon l'architecture de votre robot. L'idée est de créer une interface de package standard entre les packages botbrain_ws et le robot.

### Structure de Package Requise

Votre package robot personnalisé doit suivre cette convention de nommage pour fonctionner parfaitement avec tous les packages : `{robot_model}_pkg`

```
{robot_model}_pkg/
├── package.xml                        # Manifeste du package ROS 2
├── CMakeLists.txt                     # Configuration de build
├── README.md                          # Documentation du package
├── launch/
│   └── robot_interface.launch.py     # REQUIS : Lanceur principal de l'interface matérielle
├── config/
│   └── nav2_params.yaml               # REQUIS : Paramètres de navigation
├── scripts/
│   ├── {robot_model}_read.py                # REQUIS : Lit les données des capteurs du robot
│   └── {robot_model}_write.py               # REQUIS : Envoie les commandes au robot
├── {robot_model}_pkg/                 # Répertoire du package Python
│   └── tools/                         # OPTIONNEL : Outils de l'assistant ROSA AI
│       ├── __init__.py                # Initialisation vide du package
│       └── {robot_model}.py           # Outils LangChain pour l'intégration ROSA
├── xacro/
│   └── robot.xacro                    # REQUIS : Modèle URDF du robot
└── meshes/
    └── *.dae, *.stl                   # Maillages visuels & de collision
```

### Guide de Création Étape par Étape

#### 1. Créer un Nouveau Package ROS 2

Créez la structure du package en utilisant les outils ROS 2 (si vous n'avez pas ros2 installé sur votre système hôte, cela peut être fait depuis un conteneur de développement) :

```bash
cd src/
ros2 pkg create {robot_model}_pkg --build-type ament_cmake --dependencies rclcpp rclpy
cd {robot_model}_pkg
```

Créez les répertoires requis :
```bash
mkdir -p launch config scripts xacro meshes maps
```

#### 2. Configurer package.xml

Éditez `package.xml` et ajoutez les dépendances requises :
- Ajoutez `bot_custom_interfaces` comme dépendance
- Mettez à jour le nom du package, la version, la description et les informations du mainteneur
- Assurez-vous que toutes les dépendances de messages de capteurs sont incluses

#### 3. Configurer CMakeLists.txt

Mettez à jour la configuration de build pour installer toutes les ressources du package :
- Installez le répertoire des fichiers launch
- Installez le répertoire des fichiers config
- Installez les scripts comme exécutables
- Installez les répertoires xacro, urdf et meshes
- Utilisez `ament_python_install_package()` pour les modules Python

#### 4. Créer le Fichier Launch de l'Interface Matérielle

**CRITIQUE** : Créez `launch/robot_interface.launch.py` (nom exact requis)

Ce fichier launch doit :
- Lire `robot_config.yaml` depuis la racine du workspace
- Extraire `robot_name` pour la configuration du namespace
- Lancer des nœuds de cycle de vie pour la lecture et l'écriture matérielle
- Utiliser `LifecycleNode` de `launch_ros.actions`
- Appliquer le namespace correct à tous les nœuds

Référence : Voir [go2_pkg/launch/robot_interface.launch.py](../../../botbrain_ws/src/go2_pkg/launch/robot_interface.launch.py) pour un exemple complet.

#### 5. Implémenter les Nœuds d'Interface Matérielle

**Créez `scripts/{robot_model}_read.py`** - Lit les données des capteurs et publie sur ROS 2 :

Ce nœud de cycle de vie doit :
- S'initialiser comme `LifecycleNode` avec le nom `robot_read_node`
- Implémenter les callbacks de cycle de vie : `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
- Dans `on_configure` : Créer des publishers pour l'odométrie, l'IMU, les états des articulations et la batterie
- Dans `on_activate` : Démarrer la boucle de lecture des données (typiquement 50Hz) depuis le matériel/topics du robot
- Traiter les données des capteurs du robot et publier sur les topics ROS 2
- Dans `on_deactivate` : Arrêter la publication des données mais maintenir les connexions
- Dans `on_cleanup` : Fermer les connexions matérielles et libérer les ressources

Référence : Voir [go2_pkg/scripts/go2_read.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_read.py) pour l'implémentation complète.

**Créez `scripts/{robot_model}_write.py`** - Reçoit les commandes et les envoie au robot :

Ce nœud de cycle de vie doit :
- S'initialiser comme `LifecycleNode` avec le nom `robot_write_node`
- Dans `on_configure` : Créer un subscriber pour le topic `cmd_vel_out` et établir la communication avec le robot
- Implémenter un callback pour recevoir les commandes de vitesse et les transmettre au matériel du robot
- Dans `on_deactivate` : Envoyer une commande d'arrêt (vitesse nulle) au robot
- Dans `on_cleanup` : Fermer les connexions matérielles et libérer les ressources
- Optionnellement : Implémenter des services spécifiques au robot (changement de mode, contrôle de démarche, etc.)

Référence : Voir [go2_pkg/scripts/go2_write.py](../../../botbrain_ws/src/go2_pkg/scripts/go2_write.py) pour l'implémentation complète.

#### 6. Créer les Paramètres de Navigation

Créez `config/nav2_params.yaml` avec les spécifications de votre robot. Voir la [Documentation Nav2](https://docs.nav2.org/) comme référence.

Vous devrez ajouter un caractère joker aux sections de configuration des nœuds. Voir [go2_pkg/config/nav2_params.yaml](../../../botbrain_ws/src/go2_pkg/config/nav2_params.yaml) pour un exemple de configuration complète.

#### 7. Créer la Description du Robot (XACRO)

Créez `xacro/robot.xacro` avec le modèle URDF de votre robot :

Votre fichier XACRO doit définir :
- `base_link` comme le lien principal du corps du robot
- `interface_link` comme la partie d'interface entre le robot et BotBrain
- Toutes les articulations et liens du robot (jambes, bras, etc.)
- Les liens des capteurs (caméras, LiDAR, IMU)
- Les maillages visuels pour la visualisation RViz
- Les maillages de collision pour la navigation
- Les limites des articulations et la dynamique
- Les propriétés inertielles

Référence : Voir [go2_pkg/xacro/robot.xacro](../../../botbrain_ws/src/go2_pkg/xacro/robot.xacro) pour la description complète du robot.

#### 8. Configurer le Workspace

Mettez à jour le `robot_config.yaml` du workspace (cela peut être fait via install.sh) :

```yaml
robot_configuration:
  robot_name: "mon_robot"              # Namespace pour tous les topics
  robot_model: "votre_robot"           # Doit correspondre au nom du package sans "_pkg"
  description_file_type: "xacro"       # "xacro" ou "urdf"
  network_interface: "eth0"            # Interface réseau pour la communication robot
```

**IMPORTANT** : Le champ `robot_model` doit correspondre au nom de votre package **sans** le suffixe `_pkg` :
- Nom du package : `votre_robot_pkg`
- robot_model : `votre_robot`

#### 9. Compiler et Tester

```bash
# Compilez votre package
cd ~/botbrain_workspace/BotBrain/botbrain_ws
colcon build --packages-select votre_robot_pkg

# Sourcez le workspace
source install/setup.bash

# Testez votre interface matérielle
ros2 launch votre_robot_pkg robot_interface.launch.py

# Lancez avec le système complet
ros2 launch bot_bringup bringup.launch.py
```

Vous pouvez compiler et tester le nouveau package en utilisant un conteneur de développement.

#### 10. Créer les Outils ROSA (Optionnel)

**ROSA** (Robot Operating System Assistant) est un assistant IA qui permet le contrôle en langage naturel de votre robot. En créant des outils pour ROSA, les utilisateurs peuvent interagir avec votre robot en utilisant des commandes conversationnelles.

**Créez la structure du répertoire des outils :**

```bash
mkdir -p {robot_model}_pkg/tools
touch {robot_model}_pkg/tools/__init__.py
touch {robot_model}_pkg/tools/{robot_model}.py
```

### Points d'Intégration du Package

Le système BotBrain trouvera et utilisera automatiquement votre package basé sur ces conventions :

1. **Nommage du Package** : format `{robot_model}_pkg`
2. **Fichier Launch** : `launch/robot_interface.launch.py` (nom exact requis)
3. **Config Navigation** : `config/nav2_params.yaml` (utilisé par bot_navigation)
4. **Fichiers de Description** : `xacro/robot.xacro` ou `urdf/robot.urdf` (utilisé par bot_description)

### Topics Requis que Votre Package Doit Fournir

Pour une intégration système complète, votre interface matérielle doit publier :

| Topic | Type de Message | Description | Fréquence |
|-------|--------------|-------------|-----------|
| `/{namespace}/odom` | nav_msgs/Odometry | Odométrie du robot | 50Hz |
| `/{namespace}/imu` | sensor_msgs/Imu | Données IMU | 100Hz |
| `/{namespace}/joint_states` | sensor_msgs/JointState | Positions/vitesses des articulations | 50Hz |

Et s'abonner à :

| Topic | Type de Message | Description |
|-------|--------------|-------------|
| `/{namespace}/cmd_vel_out` | geometry_msgs/Twist | Commandes de vitesse depuis twist_mux |


## Vue d'Ensemble des Packages

### Packages Système Central

| Package | Description | Documentation |
|---------|-------------|---------------|
| [bot_bringup](../../../botbrain_ws/src/bot_bringup) | Coordination principale du lancement, multiplexeur twist et orchestration système | [README](../../../botbrain_ws/src/bot_bringup/README.md) |
| [bot_state_machine](../../../botbrain_ws/src/bot_state_machine) | Gestion du cycle de vie, coordination des nœuds et contrôle de l'état du système | [README](../../../botbrain_ws/src/bot_state_machine/README.md) |
| [bot_custom_interfaces](../../../botbrain_ws/src/bot_custom_interfaces) | Messages, services et actions ROS 2 personnalisés | [README](../../../botbrain_ws/src/bot_custom_interfaces/README.md) |
| [bot_description](../../../botbrain_ws/src/bot_description) | Modèles URDF/XACRO du robot et robot_state_publisher | [README](../../../botbrain_ws/src/bot_description/README.md) |

### Navigation & Localisation

| Package | Description | Documentation |
|---------|-------------|---------------|
| [bot_localization](../../../botbrain_ws/src/bot_localization) | SLAM RTABMap avec support pour le mapping visuel et basé sur LiDAR | [README](../../../botbrain_ws/src/bot_localization/README.md) |
| [bot_navigation](../../../botbrain_ws/src/bot_navigation) | Stack de navigation Nav2 avec configuration agnostique au robot | [README](../../../botbrain_ws/src/bot_navigation/README.md) |

### Perception & Contrôle

| Package | Description | Documentation |
|---------|-------------|---------------|
| [bot_yolo](../../../botbrain_ws/src/bot_yolo) | Détection d'objets YOLOv8/v11 avec accélération TensorRT | [README](../../../botbrain_ws/src/bot_yolo/README.md) |
| [joystick-bot](../../../botbrain_ws/src/joystick-bot) | Interface manette de jeu avec interrupteur de sécurité dead-man | [README](../../../botbrain_ws/src/joystick-bot/README.md) |

### Packages Spécifiques aux Robots

| Package | Description | Documentation |
|---------|-------------|---------------|
| [go2_pkg](../../../botbrain_ws/src/go2_pkg) | Interface matérielle et description du Unitree Go2 quadrupède | [README](../../../botbrain_ws/src/go2_pkg/README.md) |
| [tita_pkg](../../../botbrain_ws/src/tita_pkg) | Interface matérielle et description du Tita quadrupède | [README](../../../botbrain_ws/src/tita_pkg/README.md) |

## Services Docker

Le workspace inclut plusieurs services Docker pour le déploiement conteneurisé :

| Service | Description | Démarrage Auto | Dépendances |
|---------|-------------|------------|--------------|
| `dev` | Conteneur de développement (interactif) | Non | - |
| `builder_base` | Compile tous les packages du workspace | Non | - |
| `state_machine` | Service de gestion du cycle de vie | Oui | - |
| `bringup` | Bringup principal du robot | Oui | state_machine |
| `localization` | Localisation RTABMap | Oui | bringup |
| `navigation` | Serveurs de navigation Nav2 | Oui | localization |
| `rosa` | Services d'appel d'outils IA | Oui | bringup |
| `yolo` | Service de détection d'objets | Oui | bringup |

### Utilisation de Docker

```bash
# Démarrer tous les services
docker compose up -d

# Démarrer un service spécifique avec ses dépendances
docker compose up -d navigation  # Démarre automatiquement bringup, localization

# Voir les logs
docker compose logs -f bringup

# Arrêter tous les services
docker compose down

# Recompiler après des modifications de code
docker compose build
docker compose up -d
```

## Configuration

### Fichier de Configuration Principal

Le fichier [robot_config.yaml](../../../botbrain_ws/robot_config.yaml) est le point central de configuration :

```yaml
robot_configuration:

  # Identifiant du robot - utilisé comme namespace pour tous les topics
  robot_name: ""                    # Exemple : "go2_robot1", "tita_lab"

  # Type de robot - détermine quel package matériel lancer
  robot_model: "go2"                # Options : "go2", "tita", "votre_robot"

  # Format du fichier de description
  description_file_type: "xacro"    # Options : "xacro", "urdf"

  # Interface réseau pour la communication ROS2
  network_interface: "eno1"         # Exemple : "eth0", "wlan0", "eno1"

  # Spécifique à Tita : namespace pour la communication du robot Tita
  tita_namespace: "tita3036731"     # Utilisé uniquement quand robot_model: "tita"

  # Clé API OpenAI pour les fonctionnalités IA (optionnel)
  openai_api_key: ""                # Requis pour l'assistant ROSA AI

  # Configuration Wi-Fi (optionnel)
  wifi_interface: ""                # Nom de l'interface Wi-Fi (ex. : "wlan0")
  wifi_ssid: ""                     # SSID du réseau Wi-Fi
  wifi_password: ""                 # Mot de passe du réseau Wi-Fi
```

### Configuration Réseau

Le workspace utilise CycloneDDS pour la communication ROS 2. Configuration dans [cyclonedds_config.xml](../../../botbrain_ws/cyclonedds_config.xml) :

Définissez l'interface réseau dans [robot_config.yaml](../../../botbrain_ws/robot_config.yaml) pour correspondre à votre connexion matérielle.

### Service Systemd de Démarrage Automatique

Le fichier [botbrain.service](../../../botbrain_ws/botbrain.service) active le démarrage automatique au boot :

```bash
# Installer le service (fait par install.sh)
sudo cp botbrain.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable botbrain.service

# Contrôle manuel
sudo systemctl start botbrain.service   # Démarrer maintenant
sudo systemctl stop botbrain.service    # Arrêter
sudo systemctl status botbrain.service  # Vérifier le statut

# Voir les logs
journalctl -u botbrain.service -f
```

### Ajout du Support pour de Nouveaux Robots

Voir le guide [Création d'un Package Robot Personnalisé](#création-dun-package-robot-personnalisé) ci-dessus. Nous accueillons particulièrement les contributions qui ajoutent le support pour de nouvelles plateformes de robots !

<p align="center">Fait avec ❤️ au Brésil</p>

<p align="right">
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Icône Bot" width="110">
</p>
