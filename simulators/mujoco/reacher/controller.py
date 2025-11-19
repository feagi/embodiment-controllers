#!/usr/bin/env python
"""
MuJoCo Reacher Controller - Using FEAGI Python SDK
Copyright 2016-2025 Neuraville Inc.
"""
import sys
import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer

# Try to use FEAGI SDK (feagi-python-sdk)
try:
    from feagi.pns.client import FeagiAgentClient, AgentType
    SDK_AVAILABLE = True
except ImportError:
    print("❌ FEAGI SDK not installed")
    print("   Install with: pip install feagi")
    SDK_AVAILABLE = False

# Configuration
RUNTIME = float('inf')
SPEED = 120


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='127.0.0.1', help='FEAGI IP address')
    parser.add_argument('--port', type=int, default=8000, help='FEAGI HTTP port')
    parser.add_argument('--model_xml_path', default='./reacher.xml', help='MuJoCo model path')
    args = parser.parse_args()

    print("🚀 MuJoCo Reacher Controller (FEAGI Python SDK)")
    print(f"📍 FEAGI: {args.ip}:{args.port}")
    print(f"🤖 Model: {args.model_xml_path}")
    
    # Load MuJoCo model
    try:
        model = mujoco.MjModel.from_xml_path(args.model_xml_path)
        data = mujoco.MjData(model)
        print(f"✅ Model loaded: {model.nq} DOF, {model.nu} actuators")
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        return 1
    
    # Create FEAGI client - REQUIRED (no fallback)
    if not SDK_AVAILABLE:
        print("❌ FEAGI SDK not installed")
        print("   Install with: pip install feagi")
        return 1

    print("\n🔌 Connecting to FEAGI...")
    feagi_client = FeagiAgentClient("mujoco_reacher_01", AgentType.BOTH)
    
    # Configure with vision and motor capabilities
    feagi_client.configure(
        feagi_host=args.ip,
        registration_port=30001,  # ZMQ registration port
        sensory_port=5555,        # ZMQ sensory input port
        motor_port=30005,         # ZMQ motor output port
        vision_capability=(
            "camera",              # modality
            16,                    # width
            16,                    # height
            3,                     # channels (RGB)
            "iic400"               # cortical area
        ),
        motor_capability=(
            "servo",               # modality
            2,                     # output count (2 joints)
            ["o_motor"]            # source cortical areas
        ),
        heartbeat_interval=5.0,
        connection_timeout_ms=5000,
        registration_retries=3
    )
    
    # Connect (with automatic retry) - will throw exception if fails
    feagi_client.connect()
    print("✅ Connected to FEAGI!")
    
    # Launch MuJoCo viewer
    print("\n🎮 Launching MuJoCo viewer...")
    
    feagi_ok = True  # Track if FEAGI communication is working
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Reset to initial pose
        mujoco.mj_resetData(model, data)
        
        print("✅ Viewer running!")
        print("   Press ESC in the viewer window to exit")
        print("   MuJoCo will run even if FEAGI communication fails")
        
        start_time = time.time()
        frame_number = 0
        
        print("🔄 Starting simulation loop...")

        while viewer.is_running() and time.time() - start_time < RUNTIME:
            step_start = time.time()
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Debug: Log every 120 frames (1 second at 120Hz) to show loop is running
            if frame_number % 120 == 0:
                elapsed = time.time() - start_time
                status = "✅ FEAGI OK" if feagi_ok else "⚠️ FEAGI offline (MuJoCo still running)"
                print(f"🔄 Frame {frame_number} | {elapsed:.1f}s | {status}")
            
            # Try FEAGI communication (but don't block simulation if it fails)
            if feagi_ok:
                # Send sensor data to FEAGI (every 10 frames to reduce bandwidth)
                if frame_number % 10 == 0:
                try:
                    # Convert joint positions to neuron activations
                    neuron_pairs = []
                    for i in range(min(2, len(data.qpos))):
                        neuron_id = i
                        potential = float(data.qpos[i] * 50.0)  # Scale to reasonable range
                        neuron_pairs.append((neuron_id, potential))
                        
                        feagi_client.send_sensory_data(neuron_pairs)
                    except Exception as e:
                        print(f"❌ FEAGI send failed: {e}")
                        print(f"   Continuing MuJoCo simulation without FEAGI")
                        feagi_ok = False  # Disable FEAGI communication
                
                # Receive motor commands from FEAGI (non-blocking)
                if feagi_ok:  # Only try if send succeeded
                    try:
                        motor_data = feagi_client.receive_motor_data()
                        if motor_data:
                            # Apply motor commands to actuators
                            if isinstance(motor_data, dict):
                                motor_commands = motor_data.get("motor_commands", {})
                                for actuator_id_str, command in motor_commands.items():
                                    actuator_id = int(actuator_id_str)
                                    if actuator_id < len(data.ctrl):
                                        data.ctrl[actuator_id] = command
                    except Exception as e:
                        if frame_number % 120 == 0:  # Log occasionally
                            print(f"⚠️ FEAGI receive error: {e}")
            
            # Sync viewer
            viewer.sync()
            
            # Maintain simulation speed
            elapsed = time.time() - step_start
            sleep_time = (1.0 / SPEED) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            frame_number += 1
        
        print(f"🛑 Simulation loop ended. Total frames: {frame_number}")
    
    # Cleanup
    feagi_client.disconnect()
    print("✅ Disconnected from FEAGI")
    
    print("👋 Shutdown complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
