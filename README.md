# MuJoCo AR Viewer

A Python package for visualizing MuJoCo physics simulations in Augmented Reality using Apple Vision Pro and other AR devices. 

![assets/diagram-mjar3.png](assets/diagram-mjar3.png)


## Installation

### Python API 

```bash
pip install mujoco-ar-viewer
```

To run MuJoCo XML-to-USD conversion locally (supported only on Linux and Windows via [mujoco-usd-converter](https://github.com/newton-physics/mujoco-usd-converter) from project [Newton](https://github.com/newton-physics)), use 

```bash 
pip install "mujoco-ar-viewer[usd]"
```

If not, it will use the [public-hosted server API](http://mujoco-usd-convert.xyz) for the USD conversion. 


### VisionOS App 

Open App Store on VisionOS, and search for [mujocoARViewer](https://apps.apple.com/us/app/mujocoarviewer/id6754344108). 


## Quick Start

```python
from mujoco_ar_viewer import mujocoARViewer
import mujoco

# path to mujoco XML 
xml_path = "path/to/your/model.xml"

# Set up your MuJoCo simulation
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Device's IP will be presented when you launch the app 
viewer = mujocoARViewer(avp_ip = "192.168.1.100")
viewer.load_scene(xml_path)
viewer.register(model, data)

# (Optional) it is very similar to how we define mujoco GUI viewer
import mujoco.viewer
mj_gui = mujoco.viewer.launch_passive(model, data, \
                  show_left_ui=True, show_right_ui=True)

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
    # (Optional) Render mujoco native GUI  
    mj_gui.sync()
```

## Example Use cases 

1. **Teleoperation in AR**

    This repository is an open-sourced, developer friendly package that powers [DART](https://younghyopark.me/dexhub). You can check out some manipulation environments we've made and teleoperation codes [here](https://github.com/Improbable-AI/mujoco-manipulation-envs).

    ```bash
    git clone git@github.com:Improbable-AI/mujoco-manipulation-envs.git
    cd mujoco-manipulation-envs
    python teleop.py --ip 10.31.191.37 --task reorient_esh --ver test --attach_to 0 -0.1 0.8 90
    ```

    

2. **Just bring your robot in mujoco world to real life in AR**

    It's just better to watch robots doing things in an AR environment rather than a 2D mujoco viewer. You get a better sense of how the robot's behaving in 3D space. 

    ```bash
    mjpython examples/replay.py  --viewer ar
    ```



## Recommended Read  

### Where to attach your mujoco `world` frame in AR 

Since this is a viewer in augmented reality (which by defintion, blends your simulated environment with your real world environment), deciding where to attach your simulation scene's `world` frame in your actual physical space in real world is important. You can determine this by passing in `attach_to` as an argument either by 
1. a 7-dim vector of `xyz` translation and scalar-first quaternion representation (i.e., `[x,y,z,qw,qx,qy,qz]`)
2. a 4-dim vector of `xyz` translation and rotation around `z-axis`, specified as a degree. (i.e., `[x,y,z,zrot]`)
    > Assuming you have a Z-UP convention for your mujoco environment, this second option might be enough. 

```python 
# attach the `world` frame 0.3m above the visionOS origin, rotating 90 degrees around z-axis. 
viewer.load_scene(scene_path, attach_to=[0, 0, 0.3, 90]) 
```

1. **Default Setting**: When `viewer.load_scene` is called without `attach_to` specified, it attahces the simualtion scene to the origin frame registered inside VisionOS. VisionOS automatically detects the physical ground of your surrounding using its sensors and defines the origin on the ground. For instance, if you're standing, visionOS will attach origin frame right below your feet. If you're sitting down, it's gonna be right below your chair. For most *humanoid/Quadruped Locomotion* scenes or *mobile manipulation* scenes, for instance, the world frame is often defined on a surface that can be considered as a "ground". Then you don't need no offset, at least for the `z-axis`. Based on your use cases, you might still want to some offset for `x` and `y` translation, or rotation around `z-axis`. 

2. **Custom Setting**: For many other cases, you might want to define a custom position to attach the `world` frame of a simulation scene. For most **Table-top Manipulation Scenes**, for instance, if your XML file is designed for table-top manipulation using fixed-base manipulators with your world frame defined on the surface of the table, you might want to attach the `world` frame with a slight `z-axis` offset in your AR environment. 



| Examples from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | [Unitree G1 XML](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1/scene.xml) | [Google Robot XML](https://github.com/google-deepmind/mujoco_menagerie/tree/main/google_robot/scene.xml) | [ALOHA 2 XML](https://github.com/google-deepmind/mujoco_menagerie/blob/main/aloha/scene.xml) |
|-------|---------|----------|----------|
| Visualization of `world` frame | ![](assets/unitree_g1.png)  | ![](assets/google_robot.png)     | ![](assets/aloha2.png)     |
|  | `world` frame is attached on a "ground".     | `world` frame is attached on a "ground".     | `world` frame is attached on a "table".     |
| Recommended `attach_to` | Default Setting    | Default Setting     | Offset in `z-axis`, that can bring up the table surface to reasonable height in your real world.    |


## FAQ 

1. Why did you develop this package? 

    Collecting robot demonstrations in simulation is often useful and necessary for conducting research on robot learning pipelines. However, if you try to do that by watching a 2D viewer of a simulation scene on a 2D monitor, you quickly run into limitations since you really have no accurate perception of depth. Presenting a simulation scene as an AR environment offers a nice solution. Consider this as a **3D-lifted version of your existing 2D `mujoco.viewer`.** 

2. Why is USD conversion only supported on Linux and Windows, and how should I use this for macOS then? 

    Limited macOS compatibility for automatic XML-to-USD conversion comes from [mujoco-usd-converter](https://github.com/newton-physics/mujoco-usd-converter), which internally relies on [OpenUSD Exchange SDK](https://github.com/NVIDIA-Omniverse/usd-exchange) which only supports Linux and Windows at this moment. For now, if you want to use this software on macOS, you can separately convert your XML into USD using [mujoco-usd-converter](https://github.com/newton-physics/mujoco-usd-converter) and bring the USD file over to macOS. Then, you instead of specifying a path to XML file, you can specify a path to USD file when calling `load_scene`:

    ```python
    # instead of passing in a path to XML, pass in a path to converted USDZ 
    # convert XML to USDZ with any Linux system you have access to 
    viewer.load_scene("/path/to/your/converted/scene.usdz")
    ```

3. What axis convention does hand-tracking data stream use? 

    It uses the convention defined in [VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop). 

    ![](https://github.com/Improbable-AI/VisionProTeleop/blob/main/assets/axis_convention.png)
    ![](https://github.com/Improbable-AI/VisionProTeleop/blob/main/assets/hand_skeleton_convention.png) 

4. Are there any limitations? When does it fail to load the model properly?  

    We've recently ran sanity checking on every models on [mujoco-menagerie](https://github.com/google-deepmind/mujoco_menagerie), and these are the results.

    [benchmark_on_mujoco_menagerie.md](docs/benchmark.md)






## License

MIT License

