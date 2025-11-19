# MuJoCo Humanoid Controller

**Status**: ✅ Production Ready  
**Simulator**: MuJoCo Physics Simulator  
**Model**: 21-DOF Humanoid  
**FEAGI SDK**: v2.0+ (from consolidated `feagi/feagi` repo)

## Overview

This controller enables a 21 degree-of-freedom humanoid model in MuJoCo to be controlled by FEAGI's neuromorphic brain. It provides bidirectional communication between the MuJoCo physics simulator and FEAGI, allowing the AI to receive sensory data (joint positions, camera, gyro) and send motor commands (joint actuator positions).

## Hardware Requirements

- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 4GB+ available
- **GPU**: Optional (MuJoCo can use GPU acceleration)
- **OS**: Linux, macOS, Windows

## Software Requirements

- **Python**: 3.12 (recommended for MuJoCo wheel compatibility)
- **MuJoCo**: 3.2.3 (installed via pip as Python library)
- **FEAGI**: Running FEAGI 2.0+ instance (Desktop or Cloud)
- **FEAGI Python SDK**: 2.0+ (installed from requirements.txt)

## Installation

### 1. Install Python 3.12

**macOS** (using Homebrew):
```bash
brew install python@3.12
```

**Linux** (using apt):
```bash
sudo apt update
sudo apt install python3.12 python3.12-venv
```

**Windows**:
Download from [python.org](https://www.python.org/downloads/)

### 2. Create Virtual Environment

```bash
cd embodiment-controllers/simulators/mujoco/humanoid/
python3.12 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

**Note**: On macOS, MuJoCo requires `mjpython` for GUI viewer. This is automatically installed with MuJoCo 3.2.3.

## Running the Controller

### Local Mode (Same Machine as FEAGI)

```bash
source venv/bin/activate
python controller.py --ip 127.0.0.1 --port 8000
```

### Remote Mode (Connect to Remote FEAGI)

```bash
source venv/bin/activate
python controller.py --ip 192.168.1.100 --port 8000
```

### Cloud Mode (FEAGI on Kubernetes)

```bash
source venv/bin/activate
python controller.py --ip feagi.example.com --port 8000
```

### macOS Specific

On macOS, use `mjpython` for GUI viewer support:

```bash
source venv/bin/activate
venv/bin/mjpython controller.py --ip 127.0.0.1 --port 8000
```

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ip` | `127.0.0.1` | FEAGI server IP address |
| `--port` | `8000` | FEAGI HTTP API port |
| `--model_xml_path` | `./humanoid.xml` | Path to MuJoCo model file |

## Capabilities

### Sensors (Input to FEAGI)

| Sensor Type | Count | Description | Cortical Area |
|-------------|-------|-------------|---------------|
| **Vision** | 1 camera | 16x16 RGB camera view | `i_vision` |
| **Proprioception** | 21 joints | Joint position feedback | `i_proprioception` |
| **Vestibular** | 1 gyro | Orientation/balance | `i_vestibular` |

### Actuators (Output from FEAGI)

| Actuator Type | Count | Description | Cortical Area |
|---------------|-------|-------------|---------------|
| **Servos** | 21 motors | Position-controlled joints | `o_motor` |

## Example Genomes

*(Coming soon - requires FEAGI training)*

Sample genomes for common behaviors:
- **Standing Balance**: Maintain upright posture
- **Walking Gait**: Basic forward locomotion
- **Reaching**: Arm extension and grasping

## Quick Start with Docker FEAGI

1. **Start FEAGI**:
```bash
cd feagi-core
docker-compose up -d
```

2. **Wait for FEAGI to be ready** (check http://localhost:8000/v1/health)

3. **Launch Controller**:
```bash
cd embodiment-controllers/simulators/mujoco/humanoid/
source venv/bin/activate
mjpython controller.py  # macOS
# OR
python controller.py    # Linux/Windows
```

4. **Open Brain Visualizer** at http://localhost:3000

## Troubleshooting

### MuJoCo window not opening

**Issue**: Controller runs but viewer window doesn't appear

**Solutions**:
- **macOS**: Use `mjpython` instead of `python`
  ```bash
  venv/bin/mjpython controller.py
  ```
- **Linux**: Install required GL libraries
  ```bash
  sudo apt install libgl1-mesa-glx libglew-dev
  ```
- **All OS**: Check DISPLAY environment variable (for SSH/remote)

### "FEAGI registration failed"

**Issue**: Cannot connect to FEAGI

**Solutions**:
1. Verify FEAGI is running:
   ```bash
   curl http://127.0.0.1:8000/v1/health
   ```
2. Check firewall settings (ports 8000, 5555, 30001, 30005)
3. Ensure correct IP address (use `ifconfig`/`ipconfig`)

### "ModuleNotFoundError: No module named 'feagi'"

**Issue**: FEAGI SDK not installed

**Solution**:
```bash
source venv/bin/activate
pip install -r requirements.txt
```

### "RuntimeError: MUJOCO_PATH environment variable is not set"

**Issue**: MuJoCo trying to build from source (wrong Python version)

**Solution**: Use Python 3.12 (has pre-built wheels)
```bash
python3.12 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Slow performance / lag

**Solutions**:
- Reduce simulation speed in `controller.py` (`SPEED = 60` instead of `120`)
- Reduce sensor data frequency (send every 20 frames instead of 10)
- Close other applications
- Use GPU acceleration (if available)

## Development

### Project Structure

```
humanoid/
├── controller.py           # Main controller script
├── humanoid.xml           # MuJoCo model definition
├── requirements.txt       # Python dependencies
├── capabilities.json      # Sensor/actuator definitions
├── networking.json        # Network configuration
└── README.md             # This file
```

### Modifying the Controller

1. **Add new sensors**: Update `capabilities.json` and sensor reading code
2. **Change actuators**: Modify motor command processing
3. **Adjust model**: Edit `humanoid.xml` (MuJoCo XML format)

### Testing Changes

```bash
# Test standalone (no FEAGI)
python controller.py  # Should open viewer with default pose

# Test with FEAGI
# 1. Start FEAGI
# 2. Load test genome
# 3. Run controller
# 4. Observe behavior in Brain Visualizer
```

## Technical Details

### Communication Protocol

- **Registration**: ZMQ REQ/REP on port 30001
- **Sensory Data**: ZMQ PUSH on port 5555
- **Motor Commands**: ZMQ SUB on port 30005
- **Transport**: Direct ZMQ (no HTTP/WebSocket overhead)

### Data Format

**Sensory Data** (Controller → FEAGI):
```json
{
  "neuron_id_potential_pairs": [[0, 50.0], [1, 75.0], ...],
  "agent_id": "mujoco_humanoid_01",
  "frame_number": 42
}
```

**Motor Commands** (FEAGI → Controller):
```json
{
  "agent_id": "mujoco_humanoid_01",
  "motor_commands": {
    "0": 0.5,
    "1": -0.3,
    ...
  }
}
```

### Performance Characteristics

- **Simulation Rate**: 120 Hz
- **Sensor Update**: Every 10 frames (~12 Hz)
- **Latency**: <10ms (local), <50ms (LAN), variable (cloud)
- **CPU Usage**: 1-2 cores @ 40-60%
- **Memory**: ~200MB (controller + MuJoCo)

## Contributing

See [CONTROLLER_STANDARD.md](../../../CONTROLLER_STANDARD.md) for:
- Code style guidelines
- Documentation requirements
- Testing procedures
- Marketplace submission process

## Support

- **Issues**: https://github.com/feagi/embodiment-controllers/issues
- **Discussions**: https://github.com/feagi/embodiment-controllers/discussions
- **Discord**: https://discord.gg/feagi

## License

Apache-2.0 - See [LICENSE](../../../LICENSE) for details

## Authors

Neuraville Inc. - <feagi@neuraville.com>

Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
