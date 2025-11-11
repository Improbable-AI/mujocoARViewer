import mujoco 
import mujoco.viewer as viewer
from mujoco_ar_viewer import mujocoARViewer
from avp_stream import VisionProStreamer
from scipy.spatial.transform import Rotation as R
import time


model = mujoco.MjModel.from_xml_path("./scenes/mocaps.xml")
data = mujoco.MjData(model)

arviewer = mujocoARViewer(avp_ip="10.29.178.87", enable_hand_tracking=True)
# streamer = VisionProStreamer(ip="10.29.178.87")
for _ in range(10):
    time.sleep(0.5)

with viewer.launch_passive(model, data) as gui:
    while True:
        r = arviewer.get_hand_tracking()
        # r = streamer.latest 
        print(r)

        right_wrist_from_world = r['right_wrist']
        right_fingers_from_wrist = r['right_fingers']
        print(right_fingers_from_wrist.shape)

        right_fingers_from_world = right_wrist_from_world @ right_fingers_from_wrist

        for i in range(right_fingers_from_world.shape[0]):
            data.mocap_pos[i] = right_fingers_from_world[i][:3, 3]
            quat = R.from_matrix(right_fingers_from_world[i][:3, :3]).as_quat(scalar_first=True)
            data.mocap_quat[i] = quat

        left_wrist_from_world = r['left_wrist']
        left_fingers_from_wrist = r['left_fingers']
        left_fingers_from_world = left_wrist_from_world @ left_fingers_from_wrist
        for i in range(left_fingers_from_world.shape[0]):
            data.mocap_pos[i + right_fingers_from_world.shape[0]] = left_fingers_from_world[i][:3, 3]
            quat = R.from_matrix(left_fingers_from_world[i][:3, :3]).as_quat(scalar_first=True)
            data.mocap_quat[i + right_fingers_from_world.shape[0]] = quat

        mujoco.mj_step(model, data)
        gui.sync()

