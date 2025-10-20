# MuJoCo AR Viewer

A Python package for visualizing MuJoCo physics simulations in Augmented Reality using Apple Vision Pro and other AR devices.

![assets/diagram-mjar3.png](assets/diagram-mjar3.png)


## Installation

### Python API 

```bash
pip install mujoco-ar-viewer
```

To use automatic MuJoCo XML-to-USD conversion feature (supported only on Linux and Windows via ), use: 

```bash 
pip install "mujoco-ar-viewer[usd]"
```

Note that 

### VisionOS App 

...

## Quick Start

```python
from mujoco_arviewer import MJARViewer
import mujoco

# path to mujoco XML 
xml_path = "path/to/your/model.xml"

# Set up your MuJoCo simulation
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Initialize the AR viewer with your device's IP
# Device's IP will be presented when you launch the app 
viewer = MJARViewer(avp_ip="192.168.1.100", \
                    enable_hand_tracking = True)
# Send a MuJoCo model to the AR device
# (Linux Only) it will automatically convert to USD
viewer.load_scene(xml_path) 
# Register the model and data with the viewer
viewer.register(model, data)

# Simulation loop
while True:
    # (Optional) access hand tracking results 
    hand_tracking = viewer.get_hand_tracking() 
    # (Optional) map hand tracking to mujoco ctrl
    data.ctrl = hand2ctrl(hand_tracking)

    # Step the simulation
    mujoco.mj_step(model, data)
    # Sync with AR device
    viewer.sync()
```

## License

MIT License

