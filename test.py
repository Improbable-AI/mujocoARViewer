import mujoco 
from mujoco_ar_viewer import mujocoARViewer
import mujoco.viewer

xml_path = "scenes/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

avp_ip = "192.168.86.20"
arviewer = mujocoARViewer(avp_ip=avp_ip, grpc_port=50051, enable_hand_tracking=False)
arviewer.load_scene(xml_path, attach_to=[0, 1, 0.0, 1, 0, 0, 0])  # Load scene 0.5m up
arviewer.register(model, data)

viewer = mujoco.viewer.launch_passive(model, data)

cnt = 0 

while True:
    mujoco.mj_step(model, data)

    if cnt % 1 == 0:
        arviewer.sync() 
    viewer.sync()