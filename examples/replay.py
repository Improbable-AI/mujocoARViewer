import numpy as np 
from tqdm import trange

def main(args):

    import mujoco 

    xml_path = "./scenes/franka_emika_panda/scene_blockpush.xml"

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    if args.viewer == "ar": 
        from mujoco_arviewer import MJARViewer
        viewer = MJARViewer(avp_ip=args.ip)
        viewer.load_scene(xml_path, attach_to=[0, 0.3, 0.6, 180])
        viewer.register(model, data)

    elif args.viewer == "mujoco": 
        import mujoco.viewer 
        viewer = mujoco.viewer.launch_passive(model, data)


    for ep_idx in range(1,50): 
        
        traj = np.load(f"./logs/blockpush_ep{ep_idx}.npz")

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
            drift = np.linalg.norm(data.qpos - qpos_log[t])
            if drift > 1e-2:
                print(f"Warning: drift at step {t}: {drift}")
            try: 
                viewer.sync()
            except Exception as e:
                pass 

if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", default="mujoco", choices=["mujoco", "ar"])
    parser.add_argument("--ip", default="192.168.0.1")
    args = parser.parse_args()

    main(args)