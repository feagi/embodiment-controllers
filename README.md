# Embodiment Controllers

[![GitHub Release](https://img.shields.io/github/v/release/feagi/embodiment-controllers)](https://github.com/feagi/embodiment-controllers/releases) [![Discord](https://img.shields.io/discord/1242546683791933480)](https://discord.gg/PTVC8fyGN8) [![GitHub License](https://img.shields.io/github/license/feagi/embodiment-controllers)](https://www.apache.org/licenses/LICENSE-2.0.txt)

Controllers for connecting embodiments (robots, simulators, sensors, IoT devices) to [FEAGI](https://github.com/feagi/feagi)'s neural engine. This repository contains controllers for common platforms, and you can create your own.

**Prerequisite:** FEAGI and Brain Visualizer must be started **before** launching any controller. See [FEAGI Installation](https://github.com/feagi/feagi-python-sdk/blob/main/DEPLOY.md) for setup.

---

## What's a Controller vs. an Agent?

- **Controller** = Software that bridges an embodiment to FEAGI (this repository)
- **Agent** = Autonomous entity with decision-making capability
- **Embodied Agent** = Embodiment + Controller + FEAGI Brain working together

The controller is the bridge. The agent is the result when everything works together with autonomy.

---

## What's Inside

This repository is organized into two main categories:

### Physical Embodiments (`/embodiments`)

Real-world hardware platforms organized by manufacturer:

- **Robots**: Petoi (Bittle), Elephant Robotics (MyCobot), Digital Dream Labs (Cozmo), Freenove, uFactory (Lite 6), Neuromaker (Hand 2)
- **Microcontrollers**: Arduino, ESP32, Raspberry Pi, Micro:bit
- **BCI**: Interaxon
- **Sensors**: Hokuyo LiDAR
- **Drones**: Ryze (Tello)
- **Other**: Elecfreaks (Cutebot), Neuraville (audio, video, webcam), Keystudio

### Simulators (`/simulators`)

Virtual environments for development and testing:

- **MuJoCo**: Physics simulation (humanoid, ant, reacher). Generic controller at `simulators/mujoco/`.
- **Gazebo**: Robotics simulator
- **Webots**: Robot simulator
- **Blender**: 3D environment integration

---

## Quick Start

### Using an Existing Controller

1. **Start FEAGI and Brain Visualizer first** (required):
   ```bash
   feagi start
   feagi bv start
   ```
   See [FEAGI Installation Guide](https://github.com/feagi/feagi-python-sdk/blob/main/DEPLOY.md) if not yet installed.

2. **Clone the repository**:
   ```bash
   git clone https://github.com/feagi/embodiment-controllers.git
   cd embodiment-controllers
   ```
   Or, if using the FEAGI 2.0 monorepo: `cd embodiment-controllers` from the project root.

3. **Navigate to your controller**:
   ```bash
   cd embodiments/petoi/bittle
   # or (MuJoCo: controller at simulators/mujoco root)
   cd simulators/mujoco
   ```

4. **Set up Python environment**:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # Linux/macOS
   # Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

5. **Run the controller**:
   ```bash
   # Local FEAGI (default 127.0.0.1:8000)
   python controller.py
   
   # Remote FEAGI
   python controller.py --ip 192.168.1.100 --port 8000
   ```
   MuJoCo and some controllers require additional arguments (e.g. `--feagi-zmq-motor-port`, `--model_xml`). See each controller's `README.md` for details.

---

## Creating Your Own Controller

Want to connect FEAGI to your own robot or device?

1. **Read the standard**: [CONTROLLER_STANDARD.md](CONTROLLER_STANDARD.md)
2. **Use the template**: `embodiments/template/`
3. **Follow the structure**: Required files and conventions
4. **Test thoroughly**: Local, Docker, and cloud environments
5. **Submit a PR**: Contribute back to the community

For detailed guidance, see:
- [CONTROLLER_STANDARD.md](CONTROLLER_STANDARD.md) - Technical requirements
- [docs/create_controller.md](docs/create_controller.md) - Development guide
- [connectivity.md](connectivity.md) - Connection options

---

## Marketplace Distribution

Controllers in this repository are **open source** (Apache 2.0) and community-maintained.

Want your controller to be **easily installable** by end users through the FEAGI Marketplace?

1. Ensure your controller follows [CONTROLLER_STANDARD.md](CONTROLLER_STANDARD.md)
2. Submit to the marketplace (coming soon: marketplace.feagi.io/submit)
3. Neuraville will review, package, and distribute
4. Users can install with one click from FEAGI Desktop or Cloud

The marketplace handles:
- Quality assurance
- Professional packaging
- Media and documentation
- Easy installation
- User support

---

## Repository Structure

```
embodiment-controllers/
├── embodiments/
│   ├── arduino/
│   ├── petoi/
│   │   ├── bittle/
│   │   ├── bluetooth/
│   │   └── pyserial/
│   ├── elephant_robotics/
│   ├── esp32/
│   ├── microbit/
│   ├── digital_dream_labs/
│   │   └── cozmo_1.0/
│   ├── template/
│   └── ...
├── simulators/
│   ├── mujoco/          # Generic controller (ant, humanoid, reacher models)
│   ├── gazebo/
│   ├── webots/
│   └── blender/
├── docs/
│   └── create_controller.md
├── CONTROLLER_STANDARD.md    # Technical requirements
├── connectivity.md           # Connection guide
└── README.md                 # You are here
```

---

## Community

Join the FEAGI community:

- **Discord**: [FEAGI Community](https://discord.gg/PTVC8fyGN8)
- **Twitter/X**: [@neuraville](https://x.com/neuraville)
- **YouTube**: [Neuraville Channel](https://www.youtube.com/@Neuraville)
- **LinkedIn**: [FEAGI Group](https://www.linkedin.com/groups/12777894/)

---

## Contributing

We welcome contributions! Please:

1. Fork this repository
2. Follow [CONTROLLER_STANDARD.md](CONTROLLER_STANDARD.md)
3. Test your controller thoroughly
4. Submit a pull request

See also: [FEAGI Contribution Guide](https://github.com/feagi/feagi/blob/staging/CONTRIBUTING.md)

---

## License

All controllers in this repository are distributed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0.txt).
