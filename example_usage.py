#!/usr/bin/env python3
"""
Example usage of MJARView for MuJoCo AR visualization
"""
import time
import mujoco
from mjarview import MJARView

def main():
    # Example XML path (replace with your actual MuJoCo XML file)
    xml_path = "path/to/your/model.xml"
    
    # VR device IP (replace with your Vision Pro's IP)
    vr_device_ip = "192.168.1.100"  # Replace with actual IP
    
    try:
        # Initialize MJARView
        print("🚀 Initializing MJARView...")
        ar_view = MJARView(
            xml_path=xml_path,
            vr_device_ip=vr_device_ip,
            grpc_port=50051,
            http_port=8083
        )
        
        print("📱 MJARView initialized. USDZ file created and HTTP server started.")
        print("🔗 Waiting for Vision Pro to connect...")
        
        # Simulation loop
        simulation_time = 0.0
        dt = 0.01  # 10ms timestep
        
        while True:
            # Step the MuJoCo simulation
            mujoco.mj_step(ar_view.model, ar_view.data)
            simulation_time += dt
            
            # Send pose updates to VR device every 50ms (20 Hz)
            if int(simulation_time * 1000) % 50 == 0:
                ar_view.update()
            
            # Real-time delay
            time.sleep(dt)
            
            # Optional: break after some time for testing
            if simulation_time > 60.0:  # Run for 60 seconds
                break
                
    except KeyboardInterrupt:
        print("\n🛑 Stopping simulation...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'ar_view' in locals():
            ar_view.close()
        print("✅ Cleanup completed")

if __name__ == "__main__":
    main()