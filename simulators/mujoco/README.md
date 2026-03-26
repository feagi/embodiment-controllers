# MuJoCo Generic Controller

Generic MuJoCo controller supporting any MuJoCo XML model via FEAGI Python SDK.

## Prerequisites: Start FEAGI and Brain Visualizer First

**Important:** You must start FEAGI and Brain Visualizer (BV) **before** launching the MuJoCo controller. The controller connects to a running FEAGI instance; it will fail or fall back to standalone mode if FEAGI is not available.

### Starting FEAGI

1. Install FEAGI (see [FEAGI Installation Guide](../../../feagi-python-sdk/DEPLOY.md) for detailed setup).
2. Start the FEAGI runtime:

   ```bash
   feagi start
   ```

3. Start Brain Visualizer (optional but recommended for visualization):

   ```bash
   feagi bv start
   ```

4. Ensure FEAGI has a genome loaded (via API, `--genome` at startup, or use `--load-genome` when running the controller).

For full installation and configuration details, see the [FEAGI Installation Guide](../../../feagi-python-sdk/DEPLOY.md).

## Requirements

- Python 3.10, 3.11, or 3.12 (not 3.13+; mujoco 3.2.3 has no wheels for newer Python)
- feagi-core>=2.1.35
- mujoco==3.6.0
- numpy==1.26.4
- **macOS**: Use `mjpython` (installed with mujoco) instead of `python` for the viewer to open. Linux and Windows use standard `python`.

## Installation

```bash
cd embodiment-controllers/simulators/mujoco
python3.12 -m venv venv   # Use 3.10, 3.11, or 3.12 (not 3.13+)
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

## Usage

With FEAGI and Brain Visualizer already running (see [Prerequisites](#prerequisites-start-feagi-and-brain-visualizer-first)), run the controller. All network parameters are required (no defaults). Obtain values from your FEAGI network configuration.

### Platform-specific interpreter

| Platform | Interpreter | Path (when not activated) |
|----------|-------------|----------------------------|
| macOS | `mjpython` (required for viewer) | `venv/bin/mjpython` or `.venv/bin/mjpython` |
| Linux | `python` | `venv/bin/python` or `.venv/bin/python` |
| Windows | `python` | `venv\Scripts\python.exe` |

### Example: Local FEAGI with Humanoid

For local FEAGI with DummyAuth:

```bash
# Set auth token (Unix)
export FEAGI_AUTH_TOKEN_B64="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
# Windows cmd: set FEAGI_AUTH_TOKEN_B64=AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=
# Windows PowerShell: $env:FEAGI_AUTH_TOKEN_B64="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="

python controller.py \
  --ip 127.0.0.1 \
  --port 8000 \
  --feagi-zmq-motor-port 5564 \
  --feagi-zmq-registration-port 30001 \
  --feagi-zmq-sensory-port 5558 \
  --feagi-zmq-connection-timeout-ms 5000 \
  --feagi-zmq-registration-retries 10 \
  --feagi-zmq-heartbeat-interval-s 5.0 \
  --feagi-http-timeout-s 30.0 \
  --model_xml humanoid/humanoid.xml \
  --agent_id "AAAAAGZlYWdpICAgICAgICAgICAgICAgbXVqb2NvX2h1bWFub2lkXzAxICABAAAA" \
  --auth-token-b64 "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
```

### If FEAGI has no genome loaded

Add `--load-genome` to load the essential genome before connecting. Use the platform-specific interpreter from the table above.

```bash
python controller.py ... --load-genome
```

### Argument Reference

| Argument | Example | Description |
|----------|---------|--------------|
| `--ip` | `127.0.0.1` | FEAGI API host |
| `--port` | `8000` | FEAGI HTTP API port |
| `--feagi-zmq-motor-port` | `5564` | ZMQ motor port (must match FEAGI ports.zmq_motor_port) |
| `--feagi-zmq-registration-port` | `30001` | ZMQ registration port |
| `--feagi-zmq-sensory-port` | `5558` | ZMQ sensory port (must match FEAGI zmq_sensory_port) |
| `--feagi-zmq-connection-timeout-ms` | `5000` | ZMQ timeout (ms) |
| `--feagi-zmq-registration-retries` | `10` | Registration retries |
| `--feagi-zmq-heartbeat-interval-s` | `5.0` | Heartbeat interval (s) |
| `--feagi-http-timeout-s` | `30.0` | HTTP timeout (s) |
| `--model_xml` | `humanoid/humanoid.xml` | Path to MuJoCo XML |
| `--agent_id` | (Base64 string) | 48-byte AgentDescriptor |
| `--load-genome` | (flag) | Load essential genome before connecting |

Optional: `--motor_gain` (default 10.0), `--cortical_input`, `--cortical_output`.

## Troubleshooting

**`RuntimeError: launch_passive requires that the Python script be run under mjpython on macOS`**  
macOS only. MuJoCo's viewer requires `mjpython`. Use `venv/bin/mjpython` or `.venv/bin/mjpython` instead of `python`. The mujoco pip package installs `mjpython` automatically. Linux and Windows use standard `python`.

**`RuntimeError: MUJOCO_PATH environment variable is not set`**  
Use Python 3.10, 3.11, or 3.12 (not 3.13+). MuJoCo 3.2.3 has no pre-built wheels for newer versions.

**`Device registration incomplete; missing cortical areas`**  
FEAGI must have a genome loaded and `auto_create_missing_cortical_areas = true` in its config. Try:
1. `--load-genome` to load essential genome before connecting
2. Increase retries: `--feagi-zmq-registration-retries 10 --feagi-zmq-heartbeat-interval-s 5.0`
3. Ensure FEAGI was started with a genome (e.g. `--genome path/to/genome.json`) or load one via its API first

## Subfolders

- `humanoid/` - Humanoid-specific controller and model
- `ant/` - Ant model
- `reacher/` - Reacher model
- `feagi_mujoco/` - Legacy feagi_connector-based controller
