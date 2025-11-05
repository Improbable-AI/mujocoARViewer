import numpy as np 
from tqdm import trange
import time
from scipy.spatial.transform import Rotation as R

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

    else: 
        viewer = None 


    kps = [16, 32, 64, 128, 256, 512, 1024, 2048]
    kds = [1, 2, 4, 8, 16, 32, 64]


    for KP in kps:
        for KD in kds:
            
            KP = 16 
            KD = 4
            decimation = 10


            def success(model, data):

                block_pos = data.body("cube").xpos
                target_pos = data.mocap_pos[model.body("target_pos").mocapid[0]]

                block_quat = data.body("cube").xquat
                target_quat = data.mocap_quat[model.body("target_pos").mocapid[0]]

                dist = np.linalg.norm(block_pos - target_pos)
                quat_diff = R.from_quat(block_quat).inv() * R.from_quat(target_quat)
                quat_diff = quat_diff.magnitude()
                
                if dist < 0.1 and quat_diff < 1.0:
                    return True 
                else:
                    return False


            success_cnt = 0 
            pbar = trange(190)
            for ep_idx in pbar: 

                pbar.set_description(f"KP {KP} | KD {KD} | Episode {ep_idx} | Success Rate : {success_cnt/(ep_idx+1):.3f}")
                
                traj = np.load(f"./logs/blockpush_ep{ep_idx}.npz")

                qpos_log = traj['qpos']
                qvel_log = traj['qvel']
                ctrl_log = traj['ctrl']
                mocap_log = traj['mocap']

                qpos_desired = qpos_log[:, :7] + (ctrl_log[:, :7] + qvel_log[:, :7] * KD) / KP 


                T = qpos_log.shape[0]

                data.qpos[:] = qpos_log[0]
                data.qvel[:] = qvel_log[0]

                data.mocap_pos[1] = mocap_log[0, :3]
                data.mocap_quat[1] = mocap_log[0, 3:]

                data.qacc_warmstart[:] = 0.0

                mujoco.mj_forward(model, data)

                for t in range(T//decimation): 
                    qdes = qpos_desired[t * decimation, :7]
                    for _ in range(decimation): 
                        ctrl = (qdes - data.qpos[:7]) * KP  - data.qvel[:7] * KD
                        data.ctrl[:7] = ctrl
                        mujoco.mj_step(model, data)
                        if viewer is not None:
                            viewer.sync()
                    
                success_flag = success(model, data)
                if success_flag:
                    success_cnt += 1
                    # print(f"Episode {ep_idx} success! Total success count: {success_cnt}")
                # else:
                    # print(f"Episode {ep_idx} failed.")
                # print(f"Episode {ep_idx} success: {success_flag}. Total success count: {success_cnt} Success Rate: {success_cnt/(ep_idx+1):.3f}")
                # time.sleep(2)


if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", default="mujoco", choices=["mujoco", "ar", "headless"])
    parser.add_argument("--ip", default="192.168.0.1")
    args = parser.parse_args()

    main(args)