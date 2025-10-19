# MuJoCo AR Viewer

A Python package for visualizing MuJoCo physics simulations in Augmented Reality using Apple Vision Pro and other AR devices.

## Installation

```bash
pip install mujoco-ar-viewer
```

## Quick Start

```python
from mujoco_arviewer import MJARViewer
import mujoco

# Initialize the AR viewer with your device's IP
viewer = MJARViewer(avp_ip="192.168.1.100")

# Send a MuJoCo model to the AR device
viewer.send_model("path/to/your/model.xml")

# Set up your MuJoCo simulation
model = mujoco.MjModel.from_xml_path("path/to/your/model.xml")
data = mujoco.MjData(model)

# Register the model and data with the viewer
viewer.register(model, data)

# Simulation loop
while True:
    # Step the simulation
    mujoco.mj_step(model, data)
    
    # Sync with AR device
    viewer.sync()
```

## Features

- **Easy Integration**: Simple API that integrates seamlessly with existing MuJoCo simulations
- **Real-time Visualization**: Stream live simulation data to AR devices
- **Cross-platform**: Works on macOS, Linux, and Windows
- **Efficient Transfer**: Optimized data transfer protocols for large models
- **Multiple Formats**: Support for both MuJoCo XML and USDZ files

## API Reference

### MJARViewer

The main class for AR visualization of MuJoCo simulations.

#### `__init__(avp_ip, grpc_port=50051)`

Initialize the AR viewer.

**Parameters:**
- `avp_ip` (str): IP address of the AR device
- `grpc_port` (int, optional): gRPC port for communication. Defaults to 50051.

#### `send_model(model_path)`

Send a MuJoCo model to the AR device.

**Parameters:**
- `model_path` (str): Path to MuJoCo XML file or USDZ file

#### `register(model, data)`

Register MuJoCo model and data for pose updates.

**Parameters:**
- `model`: MuJoCo MjModel instance
- `data`: MuJoCo MjData instance

#### `sync()`

Synchronize the current simulation state with the AR device. Call this regularly in your simulation loop.

#### `close()`

Close the viewer and clean up resources.

## Requirements

- Python 3.8+
- MuJoCo 3.1.0+
- gRPC
- USD/USDZ support
- Apple Vision Pro or compatible AR device

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.