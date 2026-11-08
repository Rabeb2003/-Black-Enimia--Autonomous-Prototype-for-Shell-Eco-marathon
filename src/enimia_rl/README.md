# Enimia RL - Hybrid RL + Classical Control for Black Enimia

Ce package implémente un système hybride combinant le contrôle classique existant (lane_assist_ros) avec l'apprentissage par renforcement pour améliorer le suivi de voie de Black Enimia.

## Architecture

Le système hybride fonctionne comme suit:

1. **Contrôle classique (baseline)**: Le système `lane_assist_ros` détecte les voies et fournit un signal d'erreur
2. **Agent RL**: Un réseau de neurones apprend à corriger/améliorer les décisions du contrôle classique
3. **Combinaison**: Les deux signaux sont combinés pour produire la commande finale

## Installation

Le package est déjà construit. Assurez-vous que les dépendances sont installées:

```bash
pip install stable-baselines3[extra] gymnasium pyyaml opencv-python
```

## Configuration

La configuration se trouve dans `config/rl_config.yaml`:

- `env`: Paramètres de l'environnement (taille image, fréquence)
- `rl`: Paramètres de l'algorithme RL (PPO)
- `hybrid`: Paramètres de la combinaison classique + RL
- `reward`: Fonction de récompense
- `safety`: Limites de sécurité

## Utilisation

### 1. Entraînement

Pour entraîner l'agent RL sur le robot réel:

```bash
# Source l'environnement ROS
source /opt/ros/humble/setup.bash
source /home/rabeb/Downloads/enimia_black/install/setup.bash

# Lancer le système classique (lane_assist_ros)
ros2 launch lane_assist_ros lane_assist.launch.py

# Dans un autre terminal, lancer l'entraînement
source /opt/ros/humble/setup.bash
source /home/rabeb/Downloads/enimia_black/install/setup.bash
python3 /home/rabeb/Downloads/enimia_black/src/enimia_rl/enimia_rl/train.py
```

**IMPORTANT**: L'entraînement se fait sur le robot réel. Assurez-vous que:
- Le robot est dans un environnement sûr
- Vous êtes prêt à arrêter l'entraînement en cas d'urgence
- Le système classique `lane_assist_ros` fonctionne correctement

### 2. Utilisation du modèle entraîné

Une fois le modèle entraîné, utilisez le wrapper:

```bash
# Source l'environnement ROS
source /opt/ros/humble/setup.bash
source /home/rabeb/Downloads/enimia_black/install/setup.bash

# Lancer le système classique
ros2 launch lane_assist_ros lane_assist.launch.py

# Dans un autre terminal, lancer le wrapper RL
source /opt/ros/humble/setup.bash
source /home/rabeb/Downloads/enimia_black/install/setup.bash
ros2 run enimia_rl rl_lane_assist
```

Le wrapper publiera:
- `/cmd_vel`: Commandes de vitesse/direction (combinées classique + RL)
- `/rl/correction`: Correction RL (pour debug)

## Observations

L'agent RL observe:
- **Image**: 64x64x3 RGB (downsampled depuis OAK-D)
- **Erreur classique**: Signal d'erreur de lane_assist_ros
- **Vitesse**: Vitesse actuelle du robot
- **Direction**: Angle de direction actuel

## Actions

L'agent RL produit:
- **Correction**: Ajustement [-0.2, 0.2] radians à la direction classique

## Récompense

La récompense combine:
- Pénalité pour écart du centre de voie
- Pénalité pour corrections brusques (smoothness)
- Bonus pour vitesse
- Bonus pour détection de voie valide

## Sécurité

Le système inclut plusieurs mécanismes de sécurité:
- Limitation de l'angle de direction (max 0.52 rad)
- Arrêt d'urgence si voie invalide
- Timeout si pas d'image
- Le contrôle classique garde toujours un rôle de base

## Monitoring

Pendant l'entraînement:
- Les logs sont sauvegardés dans `logs/tensorboard/`
- Les checkpoints sont sauvegardés dans `models/checkpoints/`
- Visualisez avec: `tensorboard --logdir logs/tensorboard/`

## Personnalisation

Pour modifier le comportement:
1. Éditez `config/rl_config.yaml`
2. Pour changer l'algorithme RL, modifiez `train.py`
3. Pour changer la fonction de récompense, modifiez `gym_env.py`

## Résultats attendus

- **Amélioration**: 10-30% par rapport au contrôle classique seul
- **Temps d'entraînement**: Quelques jours (100k timesteps)
- **Robustesse**: Meilleure performance dans conditions variées
