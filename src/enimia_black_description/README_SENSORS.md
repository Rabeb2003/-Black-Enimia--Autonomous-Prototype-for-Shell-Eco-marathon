# Intégration des Capteurs Intelligents - Black ENIMIA

Ce document décrit l'intégration des capteurs intelligents pour la compétition Shell Eco-marathon Poland 2027 (Autonomous Driving Competition).

## Capteurs Intégrés

1. **Livox Mid-360** - LiDAR 3D 360°
2. **BNO055** - IMU 9-DOF (gyroscope, accéléromètre, magnétomètre)
3. **OAK-D Lite** - Caméra stéréo RGB + Depth AI

## Structure des Fichiers

```
enimia_black_description/
├── urdf/
│   ├── enimia_black.urdf                    # URDF original du véhicule
│   ├── enimia_black_sensors.xacro           # URDF principal avec capteurs
│   └── sensors/
│       ├── livox_mid360.xacro               # Macro Livox Mid-360
│       ├── bno055.xacro                      # Macro BNO055 IMU
│       └── oak_d_lite.xacro                 # Macro OAK-D Lite
├── meshes/
│   └── sensors/                             # Dossier pour les meshes STL/DAE des capteurs
├── launch/
│   ├── display.launch.py                     # Launch original (sans capteurs)
│   ├── display_sensors.launch.py            # Launch avec capteurs (RViz2)
│   ├── gazebo_sensors.launch.py             # Launch avec capteurs (Gazebo)
│   └── real_sensors.launch.py               # Capteurs matériels (OAK/IMU/Livox)
├── config/real_sensors/                     # Params OAK, Madgwick, laserscan, MID360
├── rviz/
│   ├── display.rviz                          # RViz original
│   └── display_sensors.rviz                  # RViz avec visualisation des capteurs
└── config/
    ├── controllers.yaml                      # Configuration ros2_control
    └── sensors_calibration.yaml              # Calibration des positions/angles
```

## Utilisation

### 1. Capteurs réels (OAK + IMU + Livox optionnel)

Prérequis installés sur la machine : `depthai_ros_driver`, `imu_filter_madgwick`,
`pointcloud_to_laserscan`. Le pilote Livox (`livox_ros_driver2`) est **optionnel**.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# Si Livox est compilé ailleurs :
# source ~/ws_livox/install/setup.bash

ros2 launch enimia_black_description real_sensors.launch.py
```

Comportement :
- **Toujours** : OAK-D Lite (`depthai_ros_driver`) + filtre Madgwick → `/imu/data`
- **Si `livox_ros_driver2` est absent** : message d’avertissement clair, OAK/IMU démarrent quand même
- **Après installation** de `livox_ros_driver2` (et `source` du workspace) : le **même** launch
  active Mid-360 + `pointcloud_to_laserscan` → `/livox/scan` sans modifier le code

Config réseau Livox : `config/real_sensors/MID360_config.json` (IPs host / LiDAR).

### 1b. Bringup complet (OAK RGB+Depth + capture + Livox + FAST-LIO)

Un seul launch pour caméra (fenêtres RGB/Depth + enregistrement frames), LiDAR et mapping FAST-LIO :

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/Downloads/enimia_black/install/setup.bash

ros2 launch enimia_black_description real_bringup_mapping.launch.py
```

- **RobotModel** au sol : Fixed Frame **`odom`**, pose = odométrie roues/téléop
- **TF** : `map → odom → base_footprint → … → livox_link` (LiDAR fixé sur le prototype)
- **Nuage** : `/livox/points` en frame **`livox_link`** (bouger le LiDAR ne déplace plus le robot)
- **Odométrie** : intégration `/cmd_vel` (teleop) ; plus tard `odom_source:=wheel` + `/wheel_odom`
- **RGB/Depth** : `/oak/rgb/image_raw`, `/oak/stereo/image_raw` (driver reconstruit dans le workspace — brancher l’OAK-D Lite USB avant le launch)
- **FAST-LIO** : carte PCD seulement (`maps/fastlio_mid360_map.pcd`) — ne pilote pas le RobotModel
- Téléop intégré : `use_teleop:=true`
- Téléop terminal séparé (recommandé) :
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  (`i/j/l/,` avancer/tourner ; espace = stop)

### 1c. Mapping 2D alternatif (Livox + slam_toolbox)

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/Downloads/enimia_black/install/setup.bash

ros2 launch enimia_black_description real_mapping.launch.py
```

- Capteurs via `real_sensors.launch.py` (dont `/livox/scan` en PointCloud2)
- `slam_toolbox` en mode mapping (`config/slam_toolbox_mapping_real.yaml`)
- Besoin du TF `odom -> base_footprint` (odométrie véhicule ou FAST-LIO).  
  Bench sans odom : `use_fake_odom:=true` (debug seulement)

### 2. Visualisation dans RViz2 (sans simulation)

```bash
ros2 launch enimia_black_description display_sensors.launch.py
```

Cela lance:
- robot_state_publisher avec l'URDF complet
- joint_state_publisher_gui pour contrôler les joints
- RViz2 avec configuration pré-configurée pour les capteurs

### 3. Simulation dans Gazebo

```bash
ros2 launch enimia_black_description gazebo_sensors.launch.py
```

Cela lance:
- Gazebo avec le robot et les capteurs simulés
- robot_state_publisher
- joint_state_publisher
- Les plugins Gazebo pour chaque capteur

### 4. Topics ROS 2

#### Livox Mid-360
- `/livox/scan` - LaserScan (2D)
- `/livox/points` - PointCloud2 (3D)

#### BNO055 IMU
- `/imu/data` - sensor_msgs/Imu (orientation, vitesse angulaire, accélération)

#### OAK-D Lite
- `/oak/rgb/image_raw` - Image RGB
- `/oak/rgb/camera_info` - Infos caméra RGB
- `/oak/depth/image_raw` - Image Depth
- `/oak/depth/camera_info` - Infos caméra Depth

## Calibration des Capteurs

Les positions et orientations actuelles sont des ESTIMATIONS. Elles doivent être calibrées précisément sur le véhicule réel.

### Fichier de Calibration

`config/sensors_calibration.yaml` contient les positions et orientations de chaque capteur.

### Procédure de Calibration

1. **Calibration mécanique**:
   - Monter les capteurs physiquement sur le véhicule
   - Mesurer les positions xyz avec un pied à coulisse
   - Mesurer les angles rpy avec un niveau à bulle / inclinomètre
   - Mettre à jour `sensors_calibration.yaml`

2. **Calibration extrinsèque LiDAR-Caméra**:
   - Installer le package `lidar_camera_calibration` (ROS 2)
   - Placer une cible de calibration (chessboard + plan)
   - Enregistrer les données LiDAR et caméra
   - Calculer la transformation entre `livox_optical_frame` et `oak_rgb_optical_frame`

3. **Calibration IMU**:
   - Utiliser le driver BNO055 pour calibrer l'accéléromètre et le gyroscope
   - Faire une calibration magnétique (magnetometer calibration)
   - Vérifier l'alignement des axes avec `imu_tools`

### Mise à jour de l'URDF

Après calibration, modifier les paramètres dans `enimia_black_sensors.xacro`:

```xml
<xacro:livox_mid360 
  parent="base_link"
  name="livox"
  xyz="X_calibré Y_calibré Z_calibré"
  rpy="roll_calibré pitch_calibré yaw_calibré"/>
```

## Packages ROS 2 Recommandés

### Drivers Réels

- **Livox Mid-360**: `livox_ros_driver2` (officiel Livox)
- **BNO055**: `bno055` (flynneva) ou `libbno055_linux`
- **OAK-D Lite**: `depthai-ros` + `depthai_descriptions` (Luxonis officiel)

### SLAM / Navigation

- **FAST-LIO2** ou **Point-LIO**: LiDAR-Inertial Odometry (adapté Mid-360)
- **Nav2**: Navigation Stack ROS 2
- **RTAB-Map** ou **Cartographer**: SLAM 3D

### Calibration

- **lidar_camera_calibration**: Calibration extrinsèque LiDAR-Caméra
- **camera_calibration**: Calibration intrinsèque caméra
- **imu_tools**: Visualisation et calibration IMU

## Plugins Gazebo

Les plugins Gazebo inclus dans les fichiers xacro simulent les capteurs:
- `libgazebo_ros_laser.so` - Pour le Livox Mid-360
- `libgazebo_ros_imu_sensor.so` - Pour le BNO055
- `libgazebo_ros_camera.so` - Pour l'OAK-D Lite (RGB + Depth)

## Prochaines Étapes

1. **Télécharger les meshes STL**:
   - Livox Mid-360: depuis le site officiel Livox
   - OAK-D Lite: depuis le repository depthai_descriptions
   - BNO055: créer un mesh simple ou utiliser un modèle Adafruit

2. **Tester en simulation**:
   - Lancer `gazebo_sensors.launch.py`
   - Vérifier que tous les topics sont publiés
   - Tester avec FAST-LIO2 / Nav2

3. **Intégration réelle**:
   - Installer les drivers réels
   - Calibrer les positions et orientations
   - Tester sur le véhicule réel

## Notes Importantes

- Les positions actuelles sont des estimations basées sur la géométrie du véhicule
- La calibration précise est critique pour la performance de l'autonomie
- Les plugins Gazebo simulent les capteurs mais ne sont pas parfaits
- Pour la compétition, les drivers réels doivent être testés et validés

## Support

Pour toute question sur l'intégration ou la calibration, consulter:
- Documentation officielle de chaque capteur
- ROS 2 documentation
- Shell Eco-marathon Chapter IV (Autonomous Competitions)
