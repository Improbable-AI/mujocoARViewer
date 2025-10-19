import mujoco 
from mujoco_arviewer import MJARViewer

xml_path = "scenes/universal_robots_ur5e/scene.xml"
usdz_path = "scenes/universal_robots_ur5e/scene.usdz"

avp_ip = "10.29.194.74"

arviewer = MJARViewer(avp_ip=avp_ip, grpc_port=50051)
arviewer.load_scene(usdz_path)

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

arviewer.register(model, data)

while True:
    mujoco.mj_step(model, data)
    arviewer.sync() 