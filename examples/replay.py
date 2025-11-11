import numpy as np 
from tqdm import trange
import time 
from pathlib import Path

CUR_DIR = Path(__file__).resolve().parent

def main(args):

    import mujoco 

    xml_path = f"{CUR_DIR}/../scenes/franka_emika_panda/scene_blockpush.xml"

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    if args.viewer == "ar": 
        from mujoco_ar_viewer import mujocoARViewer
        viewer = mujocoARViewer.launch(args.ip)
        viewer.load_scene(xml_path, attach_to=[0.2, 1.0, 0.7, -90])
        viewer.register(model, data)

    elif args.viewer == "mujoco": 
        import mujoco.viewer 
        viewer = mujoco.viewer.launch_passive(model, data)


    for ep_idx in range(1,5): 
        
        traj = np.load(f"./logs/ep{ep_idx}.npz")

        qpos_log = traj['qpos']
        qvel_log = traj['qvel']
        ctrl_log = traj['ctrl']
        mocap_log = traj['mocap']

        T = qpos_log.shape[0]

        data.qpos[:] = qpos_log[0]
        data.qvel[:] = qvel_log[0]

        data.mocap_pos[1] = mocap_log[0, :3]
        data.mocap_quat[1] = mocap_log[0, 3:]

        data.qacc_warmstart[:] = 0.0

        mujoco.mj_forward(model, data)

        for t in trange(T): 
            data.ctrl = ctrl_log[t]
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(1/1000.)

if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", default="mujoco", choices=["mujoco", "ar"])
    parser.add_argument("--ip", default="192.168.0.1")
    args = parser.parse_args()

    main(args)