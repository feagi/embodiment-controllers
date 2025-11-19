# MuJoCo Reacher Controller

Simple 2-DOF robotic arm reaching task.

## Description
- **DOF**: 2 joints
- **Actuators**: 2 motors
- **Task**: Reach target position with end effector
- **Difficulty**: Beginner

## Capabilities
- **Sensory**: 2 joint positions
- **Motor**: 2 joint torques

## Quick Start
```bash
python controller.py --ip 127.0.0.1 --port 8000
```

## Model Details
- Base mounted on table
- 2-link planar arm
- Spherical target (red)
- Continuous rotation joints

