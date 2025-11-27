#!/usr/bin/env python
"""
Generic MuJoCo Controller - Using FEAGI Python SDK
Supports any MuJoCo model by passing --model_xml argument
Copyright 2016-2025 Neuraville Inc.
"""
import sys
import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer

# Configuration
RUNTIME = float('inf')
SPEED = 120


def main():
    parser = argparse.ArgumentParser(description='Generic MuJoCo Controller for FEAGI')
    parser.add_argument('--ip', default='127.0.0.1', help='FEAGI IP address')
    parser.add_argument('--port', type=int, default=8000, help='FEAGI HTTP port')
    parser.add_argument('--model_xml', required=True, help='Path to MuJoCo model XML file')
    parser.add_argument('--agent_id', required=True, help='Unique agent ID for FEAGI registration')
    parser.add_argument('--cortical_input', default='iic400', help='Cortical area for sensory input')
    parser.add_argument('--cortical_output', default='o_motor', help='Cortical area for motor output')
    args = parser.parse_args()

    print("🚀 Generic MuJoCo Controller (FEAGI Python SDK)")
    print(f"📍 FEAGI: {args.ip}:{args.port}")
    print(f"🤖 Model: {args.model_xml}")
    print(f"🆔 Agent ID: {args.agent_id}")

    # Load MuJoCo model from provided path
    try:
        print(f"📂 Loading model from: {args.model_xml}")
        model = mujoco.MjModel.from_xml_path(args.model_xml)
        data = mujoco.MjData(model)
        print(f"✅ Model loaded: {model.nq} DOF, {model.nu} actuators")
    except Exception as e:
        print(f"❌ Failed to load model '{args.model_xml}': {e}")
        return 1

    # Determine number of actuated joints (skip free joints)
    # Free joints have 7 DOF (3 position, 4 quaternion)
    free_joint_dofs = 0
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
            free_joint_dofs += 7
    
    actuated_joints = model.nu  # Number of actuators
    print(f"📊 Free joint DOFs: {free_joint_dofs}, Actuated joints: {actuated_joints}")

    # FEAGI SDK DISABLED - SDK has fundamental threading issues that block event loops
    # This needs to be fixed in the SDK itself, not worked around in controllers
    print("\n⚠️  FEAGI SDK DISABLED")
    print("   MuJoCo will run in standalone mode (physics simulation only)")
    print("   ")
    print("   SDK ARCHITECTURAL ISSUES (must be fixed at SDK level):")
    print("   1. Rust SDK spawns background threads during connect()")
    print("   2. These threads block Python's GIL and interfere with GUI event loops")
    print("   3. Even with Python threading workarounds, Rust threads still block")
    print("   4. Controllers shouldn't need complex threading code - SDK should 'just work'")
    print("   ")
    print("   REQUIRED SDK FIXES:")
    print("   → Use async/await instead of threads in Rust")
    print("   → Or: Pure Python implementation without Rust backend")
    print("   → Or: IPC-based design (separate process for SDK)")
    print("   ")
    print("   Until fixed, MuJoCo runs perfectly in standalone mode for testing.")

    # Launch MuJoCo viewer
    print("\n🎮 Launching MuJoCo viewer...")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # Reset to initial pose
        # For models with keyframes, try to use standing pose (keyframe 4 for humanoid)
        if model.nkey > 4:
            mujoco.mj_resetDataKeyframe(model, data, 4)
        else:
            mujoco.mj_resetData(model, data)

        print("✅ Viewer running!")
        print("   Press ESC in the viewer window to exit")
        print("   You can manually move joints with the mouse")
        print("   Physics simulation runs at 120 FPS")

        start_time = time.time()
        frame_number = 0

        while viewer.is_running() and time.time() - start_time < RUNTIME:
            step_start = time.time()

            # Step simulation
            mujoco.mj_step(model, data)

            # Log every 120 frames (1 second at 120Hz)
            if frame_number % 120 == 0:
                elapsed = time.time() - start_time
                print(f"🔄 Frame {frame_number} | Time: {elapsed:.1f}s | Standalone mode")

            # Sync viewer
            viewer.sync()

            # Maintain simulation speed
            elapsed = time.time() - step_start
            sleep_time = (1.0 / SPEED) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            frame_number += 1

        print(f"\n🛑 Simulation ended")
        print(f"   Total frames: {frame_number}")
        print(f"   Total time: {time.time() - start_time:.1f}s")

    print("👋 MuJoCo controller shutdown complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())

