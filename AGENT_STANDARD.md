# FEAGI Agent Standard v1.0

This document defines the technical requirements for all agents in the `feagi-agents` repository. Following this standard ensures your agent is compatible with FEAGI Core and can be discovered, installed, and used by the community.

---

## What is a FEAGI Agent?

A FEAGI Agent is software that bridges the gap between an embodiment (physical robot, simulator, sensor platform, or IoT device) and FEAGI's neural engine. Agents handle:

- **Sensor Data Translation**: Converting embodiment sensor readings into FEAGI-compatible formats
- **Actuator Control**: Translating FEAGI motor commands into embodiment-specific actions
- **Connection Management**: Establishing and maintaining communication with FEAGI Core
- **Protocol Adaptation**: Handling embodiment-specific communication protocols (serial, Bluetooth, WebSocket, etc.)

---

## Repository Structure

Agents are organized by manufacturer/platform, then by specific product/model:

```
feagi-agents/
├── embodiments/
│   └── {manufacturer}/
│       └── {product}/
│           ├── controller.py
│           ├── capabilities.json
│           ├── networking.json
│           ├── requirements.txt
│           └── README.md
└── simulators/
    └── {simulator_name}/
        └── {model}/
            ├── controller.py
            ├── capabilities.json
            ├── networking.json
            ├── requirements.txt
            └── README.md
```

**Examples**:
- `embodiments/petoi/bittle/`
- `embodiments/elephant_robotics/mycobot_280/`
- `simulators/mujoco/humanoid/`
- `simulators/gazebo/turtlebot/`

---

## Required Files

Every agent **must** include these five files:

### 1. `controller.py`

The main Python script that:
- Connects to FEAGI Core (via `feagi_connector`)
- Reads sensor data from the embodiment
- Sends actuator commands to the embodiment
- Runs the main control loop

**Naming**: Must be named `controller.py` (for now, will migrate to `agent.py` in future versions)

**Required Structure**:
```python
#!/usr/bin/env python
"""
Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
Licensed under the Apache License, Version 2.0
"""

from feagi_connector import sensors, actuators, pns_gateway as pns
from feagi_connector import feagi_interface as feagi
from feagi_connector.version import __version__

def action(obtained_data):
    """Process FEAGI output and control embodiment"""
    pass

if __name__ == "__main__":
    config = feagi.build_up_from_configuration()
    
    # Connect to FEAGI
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, 
                               capabilities, __version__)
    
    # Main control loop
    while True:
        # Read sensors from embodiment
        sensor_data = read_sensors()
        
        # Get commands from FEAGI
        message_from_feagi = pns.message_from_feagi
        if message_from_feagi:
            obtained_signals = pns.obtain_opu_data(message_from_feagi)
            action(obtained_signals)
        
        # Send sensor data to FEAGI
        pns.signals_to_feagi(sensor_data, feagi_ipu_channel, 
                            agent_settings, feagi_settings)
```

---

### 2. `capabilities.json`

Defines the sensors (inputs) and actuators (outputs) of your embodiment.

**Structure**:
```json
{
  "capabilities": {
    "input": {
      "camera": {
        "0": {
          "custom_name": "front_camera",
          "disabled": false,
          "feagi_index": 0,
          "threshold_default": 1
        }
      },
      "servo_position": {
        "0": {
          "custom_name": "joint_1",
          "disabled": false,
          "feagi_index": 0,
          "max_value": 180,
          "min_value": 0
        }
      },
      "accelerometer": {
        "0": {
          "custom_name": "imu",
          "disabled": false,
          "feagi_index": 0,
          "max_value": [1, 1, 1],
          "min_value": [-1, -1, -1]
        }
      }
    },
    "output": {
      "servo": {
        "0": {
          "custom_name": "joint_1",
          "default_value": 90,
          "disabled": false,
          "feagi_index": 0,
          "max_power": 10,
          "max_value": 180,
          "min_value": 0
        }
      },
      "motor": {
        "0": {
          "custom_name": "left_wheel",
          "default_value": 0,
          "disabled": false,
          "feagi_index": 0,
          "max_power": 100,
          "rolling_window": 1
        }
      }
    }
  }
}
```

**Supported Sensor Types**:
- `camera` - Vision (images/video)
- `servo_position` - Joint angle feedback
- `accelerometer` - 3-axis acceleration
- `gyro` - 3-axis gyroscope
- `proximity` - Distance sensors (ultrasonic, LiDAR, IR)
- `pressure` - Force/touch sensors
- `infrared` - IR light detection
- `battery` - Battery level percentage

**Supported Actuator Types**:
- `servo` - Position-controlled joints
- `motor` - Velocity-controlled motors
- `led` - Light emitters
- `motion_control` - 6DOF movement (roll, pitch, yaw, x, y, z)
- `misc` - Custom actuators

**Tool**: Use the [Controller Configurator](https://github.com/feagi/controller_configurator) to generate this file

---

### 3. `networking.json`

Defines default connection settings for FEAGI.

**Structure**:
```json
{
  "description": "Agent Name",
  "version": "v1.0.0",
  "feagi_settings": {
    "feagi_url": "http://127.0.0.1",
    "feagi_api_port": 8000,
    "magic_link": "null"
  },
  "agent_settings": {
    "agent_data_port": 10009,
    "agent_id": "unique_agent_id",
    "agent_type": "embodiment",
    "compression": true,
    "godot_websocket_ip": "0.0.0.0",
    "godot_websocket_port": 9052
  }
}
```

**Key Fields**:
- `agent_id` - Unique identifier for your agent (e.g., "petoi_bittle", "mujoco_humanoid")
- `agent_type` - Always `"embodiment"` for physical/simulated robots
- `compression` - Enable ZMQ compression (recommended: `true`)

**Note**: Users can override these via command-line flags:
```bash
python controller.py --ip 192.168.1.100 --port 30000 --magic_link "..."
```

---

### 4. `requirements.txt`

Lists Python dependencies.

**Minimum Requirement**:
```
feagi_connector>=1.0.0
```

**Example** (for a robot with serial communication):
```
feagi_connector>=1.0.0
pyserial>=3.5
numpy>=1.21.0
```

**Best Practices**:
- Pin major versions, allow minor updates: `package>=1.2.0,<2.0.0`
- Include all direct dependencies
- Test in a fresh virtual environment

---

### 5. `README.md`

Technical documentation for developers using your agent.

**Required Sections**:

```markdown
# Agent Name

Brief description of the embodiment.

## Hardware/Software Requirements

- List any physical hardware needed
- External software (simulators, drivers, firmware)
- Operating system compatibility

## Setup Instructions

### Local Environment

1. Clone repository: `git clone ...`
2. Navigate to agent: `cd feagi-agents/embodiments/.../...`
3. Create virtual environment: `python3 -m venv venv`
4. Activate: `source venv/bin/activate` (Linux/Mac) or `venv\Scripts\activate` (Windows)
5. Install dependencies: `pip install -r requirements.txt`

### Connection Methods

**Local FEAGI** (running on same machine):
```bash
python controller.py
```

**Remote FEAGI** (Docker, another computer):
```bash
python controller.py --ip 192.168.1.100 --port 30000
```

**Neurorobotics Studio** (cloud):
```bash
python controller.py --magic_link "paste_your_magic_link_here"
```

## Capabilities

- List sensors available
- List actuators available
- Any special features

## Troubleshooting

Common issues and solutions

## License

Apache 2.0
```

---

## Optional Files

### Model Files (for simulators)

Place simulator-specific model files in the same directory:
- `model.xml` (MuJoCo)
- `model.sdf` (Gazebo)
- `model.urdf` (ROS/Webots)

### Firmware (for microcontrollers)

Include firmware in a `firmware/` subdirectory:
```
embodiments/arduino/uno/
├── controller.py
├── firmware/
│   └── arduino_feagi.ino
└── ...
```

### Helper Libraries

Embodiment-specific code:
```python
# embodiments/petoi/bittle/petoi_library.py
def send_command(serial_port, command):
    """Petoi-specific command protocol"""
    pass
```

Import in `controller.py`:
```python
from petoi_library import send_command
```

---

## Command-Line Flags

All agents **must** support these standard flags:

| Flag | Description | Example |
|------|-------------|---------|
| `--ip` | FEAGI host IP | `--ip 192.168.1.100` |
| `--port` | FEAGI ZMQ port | `--port 30000` |
| `--magic_link` | NRS magic link | `--magic_link "https://..."` |
| `--help` | Show help | `--help` |

Additional embodiment-specific flags are allowed:
```bash
python controller.py --usb_port /dev/ttyUSB0 --baudrate 115200
```

---

## Testing Your Agent

### 1. Test with Local FEAGI

1. Start FEAGI Core locally (via Docker or source)
2. Run your agent: `python controller.py`
3. Verify connection in FEAGI logs
4. Test sensor data flow (check FEAGI brain visualizer)
5. Test actuator commands (send commands from FEAGI)

### 2. Test with Neurorobotics Studio

1. Visit [Neurorobotics Studio](https://studio.feagi.org)
2. Get magic link from Embodiment tab
3. Run: `python controller.py --magic_link "..."`
4. Verify connection

### 3. Test in Docker

1. Start FEAGI via Docker Compose
2. Run: `python controller.py --ip 127.0.0.1 --port 30000`
3. Verify connection

---

## Code Quality Guidelines

### Style
- Follow PEP 8
- Use type hints where practical
- Keep functions small and focused
- Use descriptive variable names

### Comments
- Explain **why**, not **what**
- Document non-obvious behavior
- Include license header

### Error Handling
- Gracefully handle connection failures
- Log errors clearly
- Don't crash on bad sensor data

### Performance
- Minimize latency in control loop
- Avoid blocking operations
- Use threading for camera processing

---

## Contribution Workflow

1. **Fork** the `feagi-agents` repository
2. **Create** your agent in the appropriate directory
3. **Test** locally with FEAGI
4. **Document** thoroughly in README.md
5. **Submit** a pull request

**Pull Request Requirements**:
- Agent follows this standard
- All required files present
- README is complete
- Tested with FEAGI Core
- No proprietary/commercial code

---

## Marketplace Submission

Want your agent listed on the **FEAGI Marketplace** for easy installation by end users?

1. **Ensure** your agent is in `feagi-agents` repository and follows this standard
2. **Visit** [marketplace.feagi.io/submit](https://marketplace.feagi.io/submit) (when available)
3. **Provide**:
   - Link to your agent in GitHub
   - Marketing materials (screenshots, video, description)
   - Category and pricing information
4. **Wait** for Neuraville review and approval

The marketplace handles:
- Quality assurance and testing
- Packaging for easy installation
- Media hosting and presentation
- Commercial distribution
- User support

---

## Version History

- **v1.0** (2025-11-18): Initial standard

---

## Questions or Help?

- **Discord**: [FEAGI Community](https://discord.gg/PTVC8fyGN8)
- **GitHub Issues**: [feagi-agents/issues](https://github.com/feagi/feagi-agents/issues)
- **Documentation**: [docs.feagi.org](https://docs.feagi.org)

---

## License

This standard document: Apache 2.0  
All agents in this repository: Apache 2.0 (unless explicitly stated otherwise)

