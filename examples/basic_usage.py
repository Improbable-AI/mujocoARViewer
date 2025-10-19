#!/usr/bin/env python3
"""
Example usage of the mujoco-ar-viewer package
"""
import time
import mujoco
from mujoco_arviewer import MJARViewer

def main():
    # Initialize the AR viewer with your Vision Pro's IP address
    print("🚀 Initializing MJARViewer...")
    viewer = MJARViewer(avp_ip="192.168.1.100")  # Replace with your device's IP
    
    # Path to your MuJoCo XML file
    xml_path = "scenes/universal_robots_ur5e/scene.xml"
    
    try:
        # Send the model to the AR device
        print("📱 Sending model to AR device...")
        viewer.send_model(xml_path)
        
        # Load the MuJoCo model for simulation
        print("🔄 Setting up MuJoCo simulation...")
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        
        # Register the model and data with the viewer
        viewer.register(model, data)
        
        print("✅ Setup complete! Starting simulation...")
        print("Press Ctrl+C to stop")
        
        # Simulation loop
        start_time = time.time()
        step_count = 0
        
        while True:
            # Step the simulation
            mujoco.mj_step(model, data)
            step_count += 1
            
            # Sync with AR device every 10 steps (adjust as needed)
            if step_count % 10 == 0:
                viewer.sync()
            
            # Print progress every 1000 steps
            if step_count % 1000 == 0:
                elapsed_time = time.time() - start_time
                print(f"⏱️  Steps: {step_count}, Time: {elapsed_time:.1f}s")
            
            # Small delay to control simulation speed
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping simulation...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        viewer.close()
        print("✅ Cleanup completed")

if __name__ == "__main__":
    main()