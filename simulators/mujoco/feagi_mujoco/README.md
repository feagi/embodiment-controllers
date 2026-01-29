# FEAGI MuJoCo Connector
[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![PyPI version](https://badge.fury.io/py/feagi-connector-mujoco.svg)](https://badge.fury.io/py/feagi-connector-mujoco)
[![License](https://img.shields.io/badge/license-Apache%202.0-green.svg)](https://github.com/Neuraville/controllers/blob/14f4f8d6f010f134a48fa40d1e3b25a85a364fe1/LICENSE.txt)

A Python-based connector that enables seamless integration between FEAGI (The Framework for Evolutionary Artificial General Intelligence) and the MuJoCo physics simulation environment. This connector facilitates neural network-driven control of physical simulations.

## 🚀 Quick Start
### Installation
```
# Windows
pip install feagi_connector_mujoco

# Mac/Linux
pip3 install feagi_connector_mujoco
```

Prefer to do it in a venv? Do it here:
```
# Windows
python -m venv venv
venv\Scripts\activate

# Mac/Linux
python3 -m venv venv
source venv/bin/activate
```

## 💻 Usage Options
### 1. Neurorobotics Studio (Recommended)

The Neurorobotics Studio provides a user-friendly web interface for quick setup and experimentation.

### Prerequisites

- Python 3.9 or higher ([Download Python](https://www.python.org/downloads/))


#### Getting Started with Neurorobotics Studio


1. Visit [Neurorobotics Studio](https://brainsforrobots.com/lab)

2. Create a New Experiment:
   - Click "Create"
   - Select "Mujoco simulation"
   - Choose any genome. "Barebones genome" is highly recommended.
   - Name your experiment
   - Click "Create"

3. Connect via Magic Link:
   - Navigate to "Embodiment" in the top menu
   - Click "Magic Link"
   - Run the provided command:

```
# Windows
python -m feagi_connector_mujoco --magic_link "YOUR_MAGIC_LINK"

# Mac
mjpython -m feagi_connector_mujoco --magic_link "YOUR_MAGIC_LINK"

# Linux
python3 -m feagi_connector_mujoco --magic_link "YOUR_MAGIC_LINK"
```

### 2. Docker Deployment
### Prerequisites

- Python 3.9 or higher ([Download Python](https://www.python.org/downloads/))
- For Docker deployment:
  - Git ([Windows only Download](https://gitforwindows.org/))
  - Docker Desktop ([Installation Guide](https://docs.docker.com/get-started/introduction/get-docker-desktop/))


#### Setup Instructions

1. Clone the repository after you launch CMD or Terminal:
```
git clone git@github.com:feagi/feagi.git
```

2. Navigate to the Docker directory:
```
cd feagi/docker
```

3. Pull and start the Docker containers:
```
docker compose -f playground.yml pull
docker compose -f playground.yml up
```

4. Access the Playground:
   - Open `http://127.0.0.1:4000/`
   - Click "GENOME" (top right, next to "API")
   - Select "Essential"

5. Start the connector:
```
# Windows
python -m feagi_connector_mujoco --port 30000

# Mac
mjpython -m feagi_connector_mujoco --port 30000

# Linux
python3 -m feagi_connector_mujoco --port 30000
```

## 🛠️ Configuration Options

### Command-Line Arguments

```
python -m feagi_connector_mujoco --help
```

| Argument | Description | Default |
|----------|-------------|---------|
| `-h, --help` | Display help message | - |
| `-magic_link, --magic_link` | NRS Studio magic link | - |
| `-ip, --ip` | FEAGI IP address | localhost |
| `-port, --port` | ZMQ port (30000 for Docker, 3000 for localhost) | 3000 |
| `--model_xml_path` | Custom MuJoCo XML file path | './humanoid.xml' |

## 🔧 Custom MuJoCo Configuration

To use custom MuJoCo files, specify the path using the `--model_xml_path` flag:

```
# Windows
python -m feagi_connector_mujoco --model_xml_path /path/to/your/model.xml

# Mac 
mjpython -m feagi_connector_mujoco --model_xml_path /path/to/your/model.xml

# Linux
python3 -m feagi_connector_mujoco --model_xml_path /path/to/your/model.xml
```

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guidelines](https://github.com/feagi/feagi/blob/staging/CONTRIBUTING.md) for details.

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](https://github.com/feagi/feagi/blob/staging/LICENSE.txt) file for details.

## 🔗 Links
- [FEAGI Website](https://feagi.org)
- [Documentation](https://docs.feagi.org)
- [GitHub Repository](https://github.com/feagi/feagi)
- [Issue Tracker](https://github.com/feagi/feagi/issues)