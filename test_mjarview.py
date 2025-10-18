#!/usr/bin/env python3

"""
Test script for MJARView
"""

from mjarview import MJARView
import time

def main():
    # Example usage - you'll need to provide a real MuJoCo XML file path
    xml_path = "example_model.xml"  # Replace with your actual XML file path
    vr_device_ip = "192.168.1.100"  # Replace with your Vision Pro IP address
    
    try:
        # Initialize MJARView
        print("🚀 Initializing MJARView...")
        mjar = MJARView(
            xml_path=xml_path,
            vr_device_ip=vr_device_ip,
            grpc_port=50051,
            http_port=8083
        )
        
        # Wait a moment for HTTP server to start
        time.sleep(2)
        
        # Send handshake to establish connection
        print("🤝 Sending handshake...")
        if mjar.send_handshake():
            print("✅ Handshake successful! VR device should start loading USDZ...")
            
            # Simulation loop - send pose updates
            print("🔄 Starting pose update loop...")
            for i in range(10):
                # Step the simulation (you might want to call mujoco.mj_step here)
                # mujoco.mj_step(mjar.model, mjar.data)
                
                # Send pose update
                mjar.update()
                
                # Wait before next update
                time.sleep(0.1)  # 10 Hz update rate
                
        else:
            print("❌ Handshake failed. Make sure VR device is running and accessible.")
            
    except FileNotFoundError:
        print(f"❌ XML file not found: {xml_path}")
        print("Please provide a valid MuJoCo XML file path.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'mjar' in locals():
            mjar.close()

if __name__ == "__main__":
    main()