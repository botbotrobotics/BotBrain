# Manuel d'Assemblage du Matériel BotBrain

Boîtiers imprimables en 3D et pièces d'interface robot pour BotBrain. Ce guide couvre l'impression, la nomenclature et les instructions d'assemblage étape par étape.

<p align="center">
  <img src="../../images/assembly.gif" alt="Assemblage BotBrain" width="600">
</p>

<p align="center">
  <a href="../../../hardware/README.md"><img src="https://img.shields.io/badge/🇺🇸_English-blue" alt="English"></a>
  <a href="README_pt.md"><img src="https://img.shields.io/badge/🇧🇷_Português-blue" alt="Português"></a>
  <a href="README_fr.md"><img src="https://img.shields.io/badge/🇫🇷_Français-green" alt="Français"></a>
  <a href="README_zh-CN.md"><img src="https://img.shields.io/badge/🇨🇳_中文-blue" alt="中文"></a>
  <a href="README_es.md"><img src="https://img.shields.io/badge/🇪🇸_Español-blue" alt="Español"></a>
</p>

> **Note :** La version anglaise est la documentation officielle et la plus à jour. Cette traduction peut ne pas refléter les dernières modifications.

---

## Table des Matières

- [Vue d'Ensemble](#vue-densemble)
- [Prérequis Généraux](#prérequis-généraux)
- [Paramètres d'Impression](#paramètres-dimpression)
- [Assemblage BotBrain](#assemblage-botbrain)
- [Assemblages d'Interface Robot](#assemblages-dinterface-robot)
  - [Interface Unitree G1](#interface-unitree-g1)
  - [Interface Unitree Go2](#interface-unitree-go2)
  - [Interface Direct Drive Tita](#interface-tita)

---

## Vue d'Ensemble

Le matériel BotBrain se compose de deux éléments principaux :

1. **BotBrain** - Le boîtier principal abritant toute l'électronique
2. **Interface Robot** - Un adaptateur de montage spécifique à votre plateforme robot

Vous devrez d'abord assembler le BotBrain, puis le fixer à l'interface robot appropriée pour votre plateforme.

### Structure des Répertoires

```
hardware/
├── BotBrain/          # Fichiers du boîtier principal
├── G1/                # Interface Unitree G1
├── Go2/               # Interface Unitree Go2
└── Tita/              # Interface Direct Drive Tita
```

### Formats de Fichiers

| Format | Cas d'Utilisation |
|--------|----------|
| **.3mf** | Recommandé pour la plupart des slicers (PrusaSlicer, Bambu Studio, Cura) |
| **.stl** | Format universel, fonctionne avec tous les slicers |
| **.step** | Format CAO pour modifications |

---

## Prérequis Généraux

### Outils Requis

| Outil | Usage |
|------|---------|
| Imprimante 3D | Impression du boîtier et des pièces d'interface |
| Jeu de tournevis (Phillips/Hex) | Fixation des composants |
| Pinces à dénuder | Préparation des câbles |
| Pinces fines | Manipulation des petits composants |

### Précautions de Sécurité

> **Avertissement :** Toujours déconnecter l'alimentation avant l'assemblage ou le désassemblage.

- Manipulez l'électronique avec précaution pour éviter les décharges statiques (utilisez un bracelet ESD si disponible)
- Assurez une ventilation adéquate lors des soudures
- Portez des lunettes de sécurité lors du retrait des supports des impressions
- Vérifiez la polarité avant de connecter l'alimentation

---

## Paramètres d'Impression

Utilisez ces paramètres pour toutes les pièces matérielles BotBrain :

| Paramètre | Valeur Recommandée | Notes |
|---------|-------------------|-------|
| Matériau | PLA | PETG également acceptable pour les environnements à température élevée |
| Hauteur de Couche | 0.2 mm | Utilisez 0.1 mm pour des détails plus fins |
| Remplissage | 20-30% | Remplissage plus élevé pour les pièces structurelles |
| Supports | Supports en arbre | - |
| Adhérence au Plateau | Brim (optionnel) | Aide à prévenir le gauchissement |

---

## Assemblage BotBrain

Le BotBrain Core est le boîtier principal qui abrite toute l'électronique. Terminez cet assemblage avant de fixer toute interface robot.

[Vidéo Complète d'Assemblage](https://youtu.be/xZ5c619bTEQ) - Tutoriel vidéo complet étape par étape du processus d'assemblage BotBrain

### Nomenclature - BotBrain

#### Pièces Imprimées en 3D

| Pièce | Quantité | Fichier | Notes |
|------|----------|------|-------|
| Boîtier Supérieur | 1 | [BotBrain/top_case.stl](../../../hardware/BotBrain/top_case.stl) | Couvercle principal |
| Boîtier Inférieur | 1 | [BotBrain/bottom_case.stl](../../../hardware/BotBrain/bottom_case.stl) | Logement des composants |

#### Électronique

| Composant | Quantité | Notes |
|-----------|----------|-------|
| Jetson Orin Nano | 1 | Sans Base |
| Câble USB-A/USB-C | 2 | Longueur de 15cm pour un meilleur ajustement |
| Caméra RealSense | 2 | D435i |
| Convertisseur DC-DC 12V | 1 | Pour caméras D435i |
| Connecteur WAGO | 2 | 2 voies |
| Pigtail Jack Barrel | 1 | - |

#### Fixations et Quincaillerie

| Article | Quantité | Notes |
|------|----------|-------|
| M3x10 Autotaraudeuse | 4 | De préférence Hex/Allen |
| Rondelle Plate M3 | 4 | - |

### Étapes d'Assemblage - BotBrain Core

#### Étape 1 : Imprimez les Pièces du Boîtier

Imprimez les boîtiers supérieur et inférieur en utilisant les [paramètres d'impression](#paramètres-dimpression) ci-dessus.

---

#### Étape 2 : Préparez le Boîtier Inférieur

Retirez tout le matériau de support et nettoyez les bords rugueux.

**Tâches :**
1. Retirez tout le matériau de support avec des pinces coupantes
2. Poncez les bords ou bosses rugueux
3. Vérifiez que tous les points de montage sont dégagés

---

#### Étape 3 : Installez l'Électronique

**Tâches :**

1. Placez les vis dans les bossages de montage sur le boîtier inférieur
2. Connectez les connecteurs WAGO aux bornes d'entrée et de sortie du convertisseur DC-DC 12V
3. Montez le convertisseur 12V à l'emplacement désigné sur le boîtier inférieur
4. Connectez le pigtail jack barrel au connecteur WAGO de sortie du convertisseur DC
5. Placez les caméras RealSense dans leurs positions de montage avec les câbles USB pré-connectés
6. Placez le Jetson Orin Nano dans la bonne position, en dirigeant les antennes WiFi/Bluetooth vers la poche latérale
7. Connectez les câbles USB des caméras RealSense au Jetson Orin Nano
8. Connectez le câble d'alimentation (jack barrel) à l'entrée d'alimentation du Jetson Orin Nano

---

#### Étape 4 : Fermez le Boîtier

Fixez le boîtier supérieur pour terminer l'assemblage du BotBrain.

**Tâches :**
1. Alignez le boîtier supérieur avec le boîtier inférieur
2. Appuyez doucement vers le bas jusqu'à ce que les clips s'enclenchent

> **Ouverture du boîtier :** Pour rouvrir, fléchissez doucement les côtés de la section inférieure pour libérer les clips de montage.

---

## Assemblages d'Interface Robot

Choisissez le guide d'assemblage d'interface pour votre plateforme robot spécifique.

---

### Interface Unitree G1

Interface de montage pour le robot humanoïde Unitree G1.

#### Nomenclature - Interface G1

##### Pièces Imprimées en 3D

| Pièce | Quantité | Fichier |
|------|----------|------|
| Support d'Interface G1 | 1 | [G1/g1_interface.stl](../../../hardware/G1/g1_interface.stl) |


##### Fixations et Quincaillerie

| Article | Quantité | Taille/Type | Notes |
|------|----------|-----------|-------|
| M6x30 | 4 | De préférence Hex/Allen |
| Rondelle Grower M6 | 4 | - |

##### Composants Supplémentaires

| Article | Quantité | Notes |
|------|----------|-------|
| Câble Ethernet | 1 | - |
| Pigtail XT-30 | 1 | - |

#### Étapes d'Assemblage - Interface G1

> ***Note :** Plus facile à assembler avec le robot couché, dos vers le haut*

##### Étape 1 : Imprimez l'Interface

Imprimez le support d'interface G1 en utilisant les [paramètres d'impression](#paramètres-dimpression).

---

##### Étape 2 : Préparez l'Interface

**Tâches :**
1. Retirez tout le matériau de support
2. Poncez les bords ou bosses rugueux
3. Vérifiez que tous les points de montage sont dégagés

---

##### Étape 3 : Montez sur le Robot

**Tâches :**
1. Serrez le BotBrain sur l'interface avec 4 vis autotaraudeuses M3
  ![g1_mount_01](../../images/mechanics/g1_mount_01.png)

2. Retirez la protection autocollante des trous de montage arrière
3. Connectez les câbles ethernet et d'alimentation au robot
4. Passez les câbles par l'ouverture du panneau
  ![g1_mount_04](../../images/mechanics/g1_mount_04.png)

5. Placez l'interface sur le dos du robot, en alignant les trous de vis

6. Serrez l'interface sur le robot à l'aide de vis M6x30
  ![g1_mount_06](../../images/mechanics/g1_mount_06.png)

---

##### Étape 4 : Fixez le BotBrain

**Tâches :**
1. Connectez les câbles d'alimentation et ethernet au BotBrain
2. Positionnez le BotBrain à l'aide des goupilles d'alignement et serrez les vis

---

### Interface Unitree Go2

Interface de montage pour le robot quadrupède Unitree Go2.

#### Nomenclature - Interface Go2

##### Pièces Imprimées en 3D

| Pièce | Quantité | Fichier | Notes |
|------|----------|------|-------|
| Support d'Interface Go2 | 1 | [Go2/go2_interface.stl](../../../hardware/Go2/go2_interface.stl) | Support de montage principal |

##### Fixations et Quincaillerie

| Article | Quantité | Notes |
|------|----------|-------|
| M3x30 | 2 | De préférence Hex/Allen |
| M3x20 | 2 | De préférence Hex/Allen |
| Rondelle Grower M3 | 4 | - |

##### Composants Supplémentaires

| Article | Quantité | Notes |
|------|----------|-------|
| Câble Ethernet | 1 | - |
| Pigtail XT-30 | 1 | - |
|

#### Étapes d'Assemblage - Interface Go2

##### Étape 1 : Imprimez l'Interface

Imprimez le support d'interface Go2 en utilisant les [paramètres d'impression](#paramètres-dimpression).

---

##### Étape 2 : Préparez l'Interface

**Tâches :**
1. Retirez tout le matériau de support
2. Poncez les bords ou bosses rugueux
3. Vérifiez que tous les points de montage sont dégagés

---

##### Étape 3 : Montez sur le Robot

**Tâches :**
1. Dévissez les vis du couvercle supérieur
  ![go2_mount_01](../../images/mechanics/go2_mount_01.png)

2. Connectez les câbles ethernet et d'alimentation au robot
  ![go2_mount_02](../../images/mechanics/go2_mount_02.png)

3. Passez les câbles par l'ouverture de l'interface
  ![go2_mount_03](../../images/mechanics/go2_mount_03.png)

4. Placez l'interface sur le dos du robot et serrez les boulons M3x30 sur la bride avant et le boulon M3x20 sur la bride arrière
  ![go2_mount_04](../../images/mechanics/go2_mount_04.png)

5. Serrez le BotBrain sur l'interface avec 4 vis autotaraudeuses M3
  ![go2_mount_05](../../images/mechanics/go2_mount_05.png)

---

##### Étape 4 : Fixez le BotBrain

**Tâches :**
1. Connectez les câbles d'alimentation et ethernet au BotBrain
2. Positionnez le BotBrain à l'aide des goupilles d'alignement et serrez les vis

---

### Interface Tita

Interface de montage pour le robot Legged Robotics Tita.

#### Nomenclature - Interface Tita

##### Pièces Imprimées en 3D

| Pièce | Quantité | Fichier | Notes |
|------|----------|------|-------|
| Support d'Interface Tita | 1 | [Tita/tita_interface2.stl](../../../hardware/Tita/tita_interface.stl) | Support de montage principal |

##### Fixations et Quincaillerie

| Article | Quantité | Notes |
|------|----------|-------|
| M4x35 | 1 | De préférence Hex/Allen |
| Rondelle plate M4 | 1 | - |

##### Composants Supplémentaires

| Article | Quantité | Notes |
|------|----------|-------|
| Câble DB25 vers Ethernet/Alimentation | 1 | Fabriqué sur mesure |

> ***Note** : Le câble d'alimentation et de communication pour le robot Tita utilise un connecteur DB25 côté Tita. Ce câble peut être fabriqué/soudé en utilisant ce [schéma électrique Tita](../../tita_conn_sch.pdf) comme référence.*

#### Étapes d'Assemblage - Interface Tita

##### Étape 1 : Imprimez l'Interface

Imprimez le support d'interface Tita en utilisant les [paramètres d'impression](#paramètres-dimpression).

---

##### Étape 2 : Préparez l'Interface

**Tâches :**
1. Retirez tout le matériau de support
2. Poncez les bords rugueux

---

##### Étape 3 : Montez sur le Robot

**Tâches :**

1. Faites glisser l'interface sur les rails du Tita jusqu'à ce que le trou de vis de l'interface soit aligné avec le point de montage du Tita
![tita_mount_01](../../images/mechanics/tita_mount_01.png)

2. Passez le câble avec les connecteurs d'alimentation et ethernet par l'ouverture avant de l'interface
![tita_mount_02](../../images/mechanics/tita_mount_02.png)

3. Connectez le câble Ethernet au Botbrain et serrez 4 vis autotaraudeuses M3x10 pour assurer la stabilité du composant
![tita_mount_03](../../images/mechanics/tita_mount_03.png)

---

##### Étape 4 : Fixez le BotBrain

**Tâches :**
1. Connectez les câbles d'alimentation et ethernet au BotBrain
2. Positionnez le BotBrain à l'aide des goupilles d'alignement et fixez toutes les vis

---
