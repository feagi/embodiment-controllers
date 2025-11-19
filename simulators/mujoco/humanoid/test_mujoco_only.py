#!/usr/bin/env python
"""
MuJoCo-only test (no FEAGI) to verify simulation works
"""
import sys
import time
import mujoco
import mujoco.viewer

def main():
    print("🚀 MuJoCo-only test (no FEAGI)")
    
    # Load MuJoCo model
    try:
        model = mujoco.MjModel.from_xml_path('./humanoid.xml')
        data = mujoco.MjData(model)
        print(f"✅ Model loaded: {model.nq} DOF, {model.nu} actuators")
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        return 1
    
    # Launch MuJoCo viewer
    print("\n🎮 Launching MuJoCo viewer...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Reset to standing pose (keyframe 4 is standing)
        mujoco.mj_resetDataKeyframe(model, data, 4)
        
        print("✅ Viewer running! Try moving the joints manually.")
        print("   Press ESC in the viewer window to exit")
        
        frame_number = 0
        start_time = time.time()
        RUNTIME = 300.0  # 5 minutes
        SPEED = 120  # Hz
        
        while viewer.is_running() and time.time() - start_time < RUNTIME:
            step_start = time.time()
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Log every 120 frames (1 second at 120Hz)
            if frame_number % 120 == 0:
                elapsed = time.time() - start_time
                print(f"🔄 Frame {frame_number} | Time: {elapsed:.1f}s | Running OK")
            
            # Sync viewer
            viewer.sync()
            
            # Maintain simulation speed
            elapsed = time.time() - step_start
            sleep_time = (1.0 / SPEED) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            frame_number += 1
        
        print(f"🛑 Simulation ended. Total frames: {frame_number}")
    
    print("👋 Shutdown complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())

