# Humanoid Walking Example

This example demonstrates a basic bipedal walking gait for the MuJoCo humanoid, showcasing coordinated leg movement and balance control through FEAGI's neural engine.

## Overview

The walking gait uses:
- **Motor cortex** → Controls leg joints (hip, knee, ankle)
- **Proprioception** → Monitors joint positions for feedback
- **Vestibular cortex** → Uses gyro data for balance
- **Central pattern generator (CPG)** → Rhythmic walking pattern

## Files

- `genome.json` - Neural network structure and cortical areas
- `connectome.json` - Connections between cortical areas and synaptic weights

## How to Use

### 1. Start FEAGI Core

**Docker**:
```bash
cd feagi/docker
docker compose -f playground.yml up
```

**Or from source** (follow FEAGI installation guide)

### 2. Load This Genome

**Method A: Via API**:
```bash
curl -X POST http://127.0.0.1:8000/v1/genome/upload \
  -F "genome=@genome.json" \
  -F "connectome=@connectome.json"
```

**Method B: Via Brain Visualizer**:
1. Open Brain Visualizer: `http://127.0.0.1:4000`
2. Click "GENOME" → "Upload"
3. Select `genome.json` and `connectome.json`

### 3. Start MuJoCo Controller

```bash
cd ../../  # Back to humanoid directory
python controller.py --port 30000  # For Docker
# or
python controller.py  # For local FEAGI
```

### 4. Observe

You should see:
- Humanoid stands upright
- Legs begin coordinated movement
- Humanoid walks forward
- Balance maintained via gyro feedback

## How It Works

### Neural Architecture

```
┌─────────────────────┐
│  Sensory Input      │
│  - Joint angles     │
│  - Gyro (balance)   │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Sensorimotor       │
│  Integration        │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Central Pattern    │
│  Generator (CPG)    │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Motor Output       │
│  - Hip flexion      │
│  - Knee extension   │
│  - Ankle control    │
└─────────────────────┘
```

### Gait Cycle

1. **Stance Phase** (60%): One leg supports weight
2. **Swing Phase** (40%): Other leg swings forward
3. **Balance**: Gyro feedback adjusts torso position
4. **Rhythm**: CPG maintains walking frequency (~1 Hz)

## Customization

### Adjust Walking Speed

Edit the CPG oscillation frequency in `genome.json`:

```json
{
  "cortical_areas": {
    "cpg_left": {
      "fire_rate": 1.0  // Increase for faster walking
    }
  }
}
```

### Modify Gait Pattern

Adjust synaptic weights in `connectome.json` between:
- CPG → Hip joints
- CPG → Knee joints
- Proprioception → Motor cortex

## Creating Your Own Genome

Want to design a custom walking gait?

### 1. Start with Essential Genome

```bash
# Load basic genome
curl http://127.0.0.1:8000/v1/genome/load?name=essential
```

### 2. Design in Brain Visualizer

1. Create cortical areas for:
   - Motor control (one per joint)
   - Sensory processing (proprioception, gyro)
   - Central pattern generators (left/right leg)

2. Connect areas:
   - Sensors → Integration
   - Integration → Motor
   - CPG → Motor (for rhythmic pattern)

3. Tune synaptic weights

### 3. Train

- Run simulation
- Observe humanoid behavior
- Adjust weights until walking is stable
- May require reinforcement learning integration

### 4. Export

```bash
curl http://127.0.0.1:8000/v1/genome/export > genome.json
curl http://127.0.0.1:8000/v1/connectome/export > connectome.json
```

## Troubleshooting

### Humanoid Falls Over

**Problem**: Unstable balance

**Solutions**:
- Increase gyro sensor weight
- Adjust torso stabilization gains
- Lower walking speed

### No Movement

**Problem**: CPG not firing

**Solutions**:
- Check CPG fire rate > 0
- Verify connections from CPG to motor cortex
- Ensure genome is loaded (check Brain Visualizer)

### Uncoordinated Movement

**Problem**: Legs moving out of phase

**Solutions**:
- Verify left/right CPGs have opposite phases
- Check synaptic delays
- Ensure proprioception feedback is connected

## Advanced Topics

### Adding Turning

To enable turning, connect additional cortical areas:
- Vision → Turn decision
- Turn decision → Differential leg control

### Object Avoidance

Use proximity sensors:
- Proximity → Obstacle detection
- Obstacle detection → Gait modification

### Energy Efficiency

Optimize the gait for minimal joint torque:
- Monitor pressure sensors
- Adjust joint trajectories
- Minimize oscillations

## References

- FEAGI Documentation: [docs.feagi.org](https://docs.feagi.org)
- MuJoCo Humanoid Model: [mujoco.org](https://mujoco.org)
- Central Pattern Generators: [Research papers]

## Support

Questions about this example?
- Discord: [FEAGI Community](https://discord.gg/PTVC8fyGN8)
- GitHub: [embodiment-controllers/issues](https://github.com/feagi/embodiment-controllers/issues)

---

**Note**: `genome.json` and `connectome.json` will be added after training. Check back soon or create your own!
