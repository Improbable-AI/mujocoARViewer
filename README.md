# MuJoCo AR Viewer

A real-time AR visualization system that streams MuJoCo physics simulations to Apple Vision Pro using gRPC and RealityKit.

## Overview

This project consists of two main components:

1. **Python MuJoCo Wrapper** (`mjarview.py`) - Converts MuJoCo XML models to USDZ format and streams pose updates
2. **Vision Pro AR App** (Swift/RealityKit) - Receives and visualizes the 3D models with real-time pose updates

## Architecture

```
[MuJoCo Simulation] → [USDZ Conversion] → [HTTP Server] → [gRPC Updates] → [Vision Pro AR]
```

### Workflow

1. **Model Conversion**: MJARView converts MuJoCo XML to USDZ format
2. **File Serving**: HTTP server serves the USDZ file for download
3. **Handshake**: gRPC handshake sends USDZ URL to Vision Pro
4. **Model Loading**: Vision Pro downloads and loads the USDZ model
5. **Real-time Updates**: gRPC streams pose updates for real-time visualization

## Setup

### Python Environment

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Generate gRPC stubs:
```bash
./generate_proto.sh
```

### Swift/Vision Pro Setup

1. Install gRPC tools:
```bash
brew install swift-protobuf grpc-swift
```

2. Generate Swift gRPC files:
```bash
./generate_swift_proto.sh
```

3. Add the generated files to your Xcode project
4. Add gRPC-Swift dependency to your project

## Usage

### Python Side

```python
from mjarview import MJARView

# Initialize with MuJoCo XML file and Vision Pro IP
mjar = MJARView(
    xml_path="your_model.xml",
    vr_device_ip="192.168.1.100",  # Vision Pro IP
    grpc_port=50051,
    http_port=8083
)

# Send initial handshake
mjar.send_handshake()

# Simulation loop
while True:
    # Step your MuJoCo simulation
    mujoco.mj_step(mjar.model, mjar.data)
    
    # Send pose updates to AR
    mjar.update()
    
    time.sleep(1/60)  # 60 Hz

# Cleanup
mjar.close()
```

### Vision Pro Side

The Vision Pro app automatically:
1. Starts a gRPC server on port 50051
2. Receives handshake with USDZ URL
3. Downloads and loads the 3D model
4. Applies real-time pose updates from the simulation

## gRPC Protocol

The communication uses a custom gRPC protocol defined in `proto/mujoco_ar.proto`:

- **SendHandshake**: Initial connection with USDZ URL
- **UpdatePoses**: Single pose update for all bodies
- **StreamPoseUpdates**: Streaming pose updates (for high-frequency updates)

## File Structure

```
├── mjarview.py              # Main Python wrapper class
├── test_mjarview.py         # Example usage script
├── proto/
│   └── mujoco_ar.proto      # gRPC protocol definition
├── ImmersiveMoveAndRotate/  # Vision Pro Swift app
│   ├── ImmersiveView.swift  # Main AR view
│   └── GRPCServer.swift     # gRPC server implementation
├── requirements.txt         # Python dependencies
└── Package.swift           # Swift package dependencies
```

## Requirements

### Python
- Python 3.8+
- MuJoCo
- gRPC
- USD/USDZ libraries

### Vision Pro
- visionOS 1.0+
- Swift 5.9+
- RealityKit
- gRPC-Swift

## Network Configuration

1. Ensure your Vision Pro and development machine are on the same network
2. Find your Vision Pro's IP address in Settings → Wi-Fi
3. Configure firewall to allow connections on ports 50051 (gRPC) and 8083 (HTTP)

## Troubleshooting

- **Connection Issues**: Check network connectivity and firewall settings
- **USDZ Loading**: Verify HTTP server is accessible from Vision Pro
- **Pose Updates**: Ensure gRPC server is running and accepting connections
- **Model Conversion**: Check MuJoCo XML file is valid and USD libraries are installed

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request