# MuJoCo AR Viewer

A mixed-reality application that visualizes MuJoCo simulations on Apple Vision Pro using RealityKit and gRPC communication.

## Overview

This project consists of two main components:

1. **Python MuJoCo Wrapper (`MJARView`)**: Converts MuJoCo XML files to USDZ format and streams pose updates via gRPC
2. **Vision Pro App**: Receives USDZ models via HTTP and pose updates via gRPC to render real-time MuJoCo simulations in mixed reality

## Workflow

1. `MJARView` loads a MuJoCo XML file and converts it to USDZ format
2. Starts an HTTP server to serve the USDZ file 
3. Sends handshake to Vision Pro app with USDZ download URL
4. Vision Pro app downloads and loads the USDZ model in RealityKit
5. `MJARView` streams real-time pose updates via gRPC
6. Vision Pro app updates entity positions/rotations in real-time

## Setup Instructions

### Python Environment (MuJoCo Side)

1. Install dependencies:
```bash
pip install mujoco grpcio grpcio-tools protobuf
# Install mujoco-usd-converter and usdex.core as needed for your setup
```

2. Generate Python gRPC code (already done):
```bash
python -m grpc_tools.protoc -I./proto --python_out=./generated --grpc_python_out=./generated proto/mujoco_ar.proto
```

3. Update IP address in `example_usage.py` to match your Vision Pro's IP address

### Vision Pro App (Swift/RealityKit)

1. Install Swift gRPC dependencies via Homebrew:
```bash
brew install swift-protobuf protoc-gen-grpc-swift
```

2. Generate Swift gRPC code (already done):
```bash
protoc --swift_out=./ImmersiveMoveAndRotate/Generated --plugin=protoc-gen-grpc-swift=/opt/homebrew/bin/protoc-gen-grpc-swift-2 --grpc-swift_out=./ImmersiveMoveAndRotate/Generated proto/mujoco_ar.proto
```

3. Add the generated Swift files to your Xcode project

4. Build and deploy to Vision Pro

## Usage

### Python Side

```python
from mjarview import MJARView
import time

# Initialize with your MuJoCo XML file and Vision Pro IP
viewer = MJARView(
    xml_path="path/to/your/model.xml",
    vr_device_ip="192.168.1.100",  # Your Vision Pro IP
    grpc_port=50051,
    http_port=8083
)

# Simulation loop
while True:
    viewer.step()  # Steps simulation and sends pose updates
    time.sleep(0.016)  # ~60 FPS
```

### Vision Pro Side

1. Launch the app on Vision Pro
2. The app automatically starts listening for gRPC connections on port 50051
3. Run your Python script - it will:
   - Send handshake with USDZ URL
   - App downloads and loads the 3D model
   - Real-time pose updates stream from Python to Vision Pro

## gRPC Protocol

The communication uses Protocol Buffers with the following key messages:

- `HandshakeRequest`: Initial setup with USDZ URL and session info
- `PoseUpdate`: Real-time body pose updates (position + quaternion rotation)
- `BodyPose`: Individual body position (Vector3) and rotation (Quaternion)

## File Structure

```
├── proto/
│   └── mujoco_ar.proto          # Protocol buffer definition
├── generated/                   # Generated Python gRPC code
├── ImmersiveMoveAndRotate/
│   ├── Generated/               # Generated Swift gRPC code
│   ├── GRPCServer.swift         # gRPC server implementation
│   ├── ImmersiveView.swift      # Main RealityKit view
│   └── APIHandler.swift         # USDZ loading helper
├── mjarview.py                  # Main Python wrapper class
├── example_usage.py             # Example usage script
└── requirements.txt             # Python dependencies
```

## Network Configuration

- **gRPC Port**: 50051 (configurable)
- **HTTP Port**: 8083 (configurable) 
- Ensure both devices are on the same network
- Vision Pro must be able to reach the Python machine's IP address

## Troubleshooting

1. **Connection Issues**: Verify both devices are on same network and firewall allows traffic on specified ports
2. **USDZ Loading Errors**: Check HTTP server is accessible and USDZ file was generated correctly
3. **Pose Update Issues**: Verify gRPC connection is established and body names match between MuJoCo model and USDZ

## Dependencies

### Python
- `mujoco`
- `grpcio` & `grpcio-tools`
- `protobuf`
- `mujoco-usd-converter` & `usdex.core`

### Swift/iOS
- GRPC Swift package
- RealityKit
- SwiftUI

## Future Enhancements

- [ ] Support for joint visualization
- [ ] Real-time physics parameter adjustment
- [ ] Multiple simultaneous sessions
- [ ] Gesture-based interaction with MuJoCo bodies
- [ ] Recording and playback of simulations