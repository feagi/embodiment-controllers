# MuJoCo Ant Controller

Quadruped robot locomotion.

## Description
- **DOF**: 15 (7 free + 8 joints)
- **Actuators**: 8 motors (4 legs × 2 joints)
- **Task**: Forward locomotion
- **Difficulty**: Advanced

## Capabilities
- **Sensory**: 8 joint positions
- **Motor**: 8 joint torques

## Quick Start
```bash
python controller.py --ip 127.0.0.1 --port 8000
```

## Model Details
- Spherical torso
- 4 legs with 2 DOF each (hip + ankle)
- Free floating body (6 DOF position + orientation)
- Suitable for reinforcement learning

