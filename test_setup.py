#!/usr/bin/env python3
"""
Test setup for MuJoCo AR Viewer
Creates a simple test environment and tests the gRPC connection
"""
import os
import tempfile
import time
from pathlib import Path

def create_test_xml():
    """Create a simple test MuJoCo XML file"""
    xml_content = """<?xml version="1.0" ?>
<mujoco model="test_model">
    <worldbody>
        <body name="box" pos="0 0 1">
            <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
            <joint type="free"/>
        </body>
        <body name="sphere" pos="0.5 0 0.5">
            <geom type="sphere" size="0.05" rgba="0 1 0 1"/>
            <joint type="free"/>
        </body>
        <body name="floor" pos="0 0 0">
            <geom type="plane" size="1 1 0.1" rgba="0.5 0.5 0.5 1"/>
        </body>
    </worldbody>
</mujoco>"""
    
    # Create test XML file
    test_dir = Path(__file__).parent / "test_data"
    test_dir.mkdir(exist_ok=True)
    
    xml_path = test_dir / "test_model.xml"
    with open(xml_path, 'w') as f:
        f.write(xml_content)
    
    print(f"✅ Test XML created: {xml_path}")
    return str(xml_path)

def test_grpc_connection():
    """Test gRPC connection without full MuJoCo dependencies"""
    try:
        import grpc
        from generated import mujoco_ar_pb2, mujoco_ar_pb2_grpc
        
        # Test creating gRPC objects
        request = mujoco_ar_pb2.UsdzUrlRequest(
            usdz_url="http://test.com/test.usdz",
            session_id="test_session"
        )
        
        print("✅ gRPC protobuf objects created successfully")
        print(f"   Request: {request.usdz_url}")
        
        # Test connection (this will fail if no server, but that's expected)
        try:
            channel = grpc.insecure_channel("localhost:50051")
            stub = mujoco_ar_pb2_grpc.MuJoCoARServiceStub(channel)
            print("✅ gRPC client stub created successfully")
            channel.close()
        except Exception as e:
            print(f"⚠️  gRPC server not running (expected): {e}")
            
    except ImportError as e:
        print(f"❌ gRPC import failed: {e}")
        print("   Install with: pip install grpcio grpcio-tools")
        return False
        
    return True

def test_dependencies():
    """Test all required dependencies"""
    print("🔍 Testing dependencies...")
    
    deps = {
        'mujoco': 'pip install mujoco',
        'mujoco_usd_converter': 'pip install mujoco-usd-converter', 
        'pxr': 'pip install usd-core',
        'usdex.core': 'pip install usd-core',
        'grpc': 'pip install grpcio grpcio-tools'
    }
    
    missing = []
    for dep, install_cmd in deps.items():
        try:
            __import__(dep)
            print(f"✅ {dep}")
        except ImportError:
            print(f"❌ {dep} - Install with: {install_cmd}")
            missing.append(dep)
    
    if missing:
        print(f"\n📦 Missing dependencies: {', '.join(missing)}")
        return False
    else:
        print("\n✅ All dependencies satisfied!")
        return True

def main():
    print("🧪 MuJoCo AR Viewer Test Setup")
    print("=" * 40)
    
    # Test dependencies
    if not test_dependencies():
        print("\n❌ Please install missing dependencies before proceeding")
        return
    
    # Test gRPC
    if not test_grpc_connection():
        print("\n❌ gRPC setup failed")
        return
    
    # Create test XML
    xml_path = create_test_xml()
    
    print("\n🎯 Next steps:")
    print("1. Start your Vision Pro app (ImmersiveMoveAndRotate)")
    print("2. Note your Vision Pro's IP address")
    print("3. Run the example:")
    print(f"   python example_usage.py")
    print(f"   (Edit the xml_path to: {xml_path})")
    print("   (Edit the vr_device_ip to your Vision Pro's IP)")
    
    print("\n📱 Vision Pro Setup:")
    print("1. Open Xcode and build the ImmersiveMoveAndRotate app")
    print("2. Install on your Vision Pro")
    print("3. Make sure both devices are on the same network")
    print("4. The app will listen on port 50051 for gRPC connections")

if __name__ == "__main__":
    main()