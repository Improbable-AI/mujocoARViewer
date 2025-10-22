import mujoco 
from mujoco_arviewer import MJARViewer

# Load the model and data.
model = mujoco.MjModel.from_xml_path("./scenes/franka_emika_panda/scene_mughang.xml")
data = mujoco.MjData(model)

