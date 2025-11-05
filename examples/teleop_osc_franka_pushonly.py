import mujoco 
from mujoco_arviewer import MJARViewer
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from copy import deepcopy
from mujoco_arviewer import MJARViewer
import time 


def hand2pose(hand, side = "right"): 

    wrist = hand[f"{side}_wrist"]
    finger = wrist @ hand[f"{side}_fingers"]

    thumb_tip = finger[4, :3, -1]
    thumb_base = finger[2, :3, -1]
    index_tip = finger[9, :3, -1]
    index_base = finger[7, :3, -1]

    base_middle = (thumb_base + index_base) * 0.5
    tip_middle = (thumb_tip + index_tip) * 0.5

    z_axis = tip_middle - base_middle
    z_axis /= np.linalg.norm(z_axis)

    # use thumb→index direction as x
    x_axis = index_base - thumb_base
    x_axis -= np.dot(x_axis, z_axis) * z_axis   # make x ⟂ z
    x_axis /= np.linalg.norm(x_axis)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    rot = np.column_stack((x_axis, y_axis, z_axis))
    rot /= np.cbrt(np.linalg.det(rot))  # ensure det = 1

    mat = np.eye(4) 
    mat[:3, :3] = rot
    mat[:3, 3] = tip_middle

    return mat

def main(args):

    # Load the model and data.
    model = mujoco.MjModel.from_xml_path("./scenes/franka_emika_panda/scene_blockpush.xml")
    data = mujoco.MjData(model)

    viewer = MJARViewer(avp_ip=args.ip, enable_hand_tracking=True)
    viewer.load_scene("./scenes/franka_emika_panda/scene_blockpush.xml", attach_to=[0, -0.1, 0.75, 90])
    viewer.register(model, data)


    class OperationalSpaceController:

        impedance_pos = np.asarray([5.0, 5.0, 5.0])  # [N/m]
        impedance_ori = np.asarray([1.0, 1.0, 1.0])  # [Nm/rad]

        # Joint impedance control gains.
        Kp_null = np.asarray([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.1]) * 10.0

        # Damping ratio for both Cartesian and joint impedance control.
        damping_ratio = 2.0

        # Gains for the twist computation. These should be between 0 and 1. 0 means no
        # movement, 1 means move the end-effector to the target in one integration step.
        Kpos: float = 10

        # Gain for the orientation component of the twist computation. This should be
        # between 0 and 1. 0 means no movement, 1 means move the end-effector to the target
        # orientation in one integration step.
        Kori: float = 10

        # Integration timestep in seconds.
        integration_dt: float = 1.0

        # Whether to enable gravity compensation.
        gravity_compensation: bool = False
        headless: bool = False 

        # Simulation timestep in seconds.
        dt: float = 0.002

        # Compute damping and stiffness matrices.
        damping_pos = damping_ratio * 2 * np.sqrt(impedance_pos)
        damping_ori = damping_ratio * 2 * np.sqrt(impedance_ori)
        Kp = np.concatenate([impedance_pos, impedance_ori], axis=0)
        Kd = np.concatenate([damping_pos, damping_ori], axis=0)
        Kd_null = damping_ratio * 2 * np.sqrt(Kp_null)

        def __init__(self, model):  
        
            # End-effector site we wish to control.
            site_name = "attachment_site"
            self.site_id = model.site(site_name).id

            # Get the dof and actuator ids for the joints we wish to control. These are copied
            # from the XML file. Feel free to comment out some joints to see the effect on
            # the controller.
            joint_names = [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "joint7",
            ]
            self.dof_ids = np.array([model.joint(name).id for name in joint_names])
            self.actuator_ids = np.array([model.actuator(name).id for name in joint_names])

            # Initial joint configuration saved as a keyframe in the XML file.
            key_name = "home"
            self.key_id = model.key(key_name).id
            self.q0 = model.key(key_name).qpos
            self.q0 = self.q0[:7]

            # Mocap body we will control with our mouse.
            mocap_name = "target"
            self.mocap_id = model.body(mocap_name).mocapid[0]

            # Pre-allocate numpy arrays.
            self.jac = np.zeros((6, model.nv))
            self.twist = np.zeros(6)
            self.site_quat = np.zeros(4)
            self.site_quat_conj = np.zeros(4)
            self.error_quat = np.zeros(4)
            self.M_inv = np.zeros((model.nv, model.nv))
            self.Mx = np.zeros((6, 6))

            self.qpos_log = [] 
            self.qvel_log = [] 
            self.ctrl_log = [] 
            self.mocap_log = [] 

            self.target_mocap_id = data.model.body("target_pos").mocapid[0]


        def log(self, data): 

            self.qpos_log.append(deepcopy(data.qpos))
            self.qvel_log.append(deepcopy(data.qvel))
            self.ctrl_log.append(deepcopy(data.ctrl))
            mocap_pose = np.concatenate([data.mocap_pos[self.target_mocap_id], data.mocap_quat[self.target_mocap_id]])
            self.mocap_log.append(deepcopy(mocap_pose)) 


        def save(self, ep_idx): 

            fp = f"logs/blockpush_zonly_ep{ep_idx}.npz"

            qpos_log = np.stack(self.qpos_log, axis=0)
            qvel_log = np.stack(self.qvel_log, axis=0)
            ctrl_log =  np.stack(self.ctrl_log, axis=0)
            mocap_log = np.stack(self.mocap_log, axis=0)

            np.savez(fp, qpos=qpos_log, qvel=qvel_log, ctrl=ctrl_log, mocap=mocap_log)


        def clear_log(self): 
            self.qpos_log = [] 
            self.qvel_log = [] 
            self.ctrl_log = [] 
            self.mocap_log = []

        def pose2torque(self, model, data): 

            # Spatial velocity (aka twist) 
            dx = data.mocap_pos[self.mocap_id] - data.site(self.site_id).xpos
            self.twist[:3] = self.Kpos * dx / self.integration_dt
            mujoco.mju_mat2Quat(self.site_quat, data.site(self.site_id).xmat)
            mujoco.mju_negQuat(self.site_quat_conj, self.site_quat)
            mujoco.mju_mulQuat(self.error_quat, data.mocap_quat[self.mocap_id], self.site_quat_conj)
            mujoco.mju_quat2Vel(self.twist[3:], self.error_quat, 1.0)
            self.twist[3:] *= self.Kori / self.integration_dt
            
            # Jacobian.
            mujoco.mj_jacSite(model, data, self.jac[:3], self.jac[3:], self.site_id)

            # Compute the task-space inertia matrix.
            mujoco.mj_solveM(model, data, self.M_inv, np.eye(model.nv))
            # print(jac.shape)
            new_jac = self.jac[:, :7] 

            new_M_inv = self.M_inv[:7, :7]
            # print(M)
            Mx_inv = new_jac @ new_M_inv @ new_jac.T
            if abs(np.linalg.det(Mx_inv)) >= 1e-2:
                Mx = np.linalg.inv(Mx_inv)
            else:
                Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)

            # Compute generalized forces.
            tau = new_jac.T @ Mx @ (self.Kp * self.twist - self.Kd * (new_jac @ data.qvel[self.dof_ids]))

            # Add joint task in nullspace.
            Jbar = new_M_inv @ new_jac.T @ Mx

            ddq = self.Kp_null * (self.q0 - data.qpos[self.dof_ids]) - self.Kd_null * data.qvel[self.dof_ids]
            tau += (np.eye(7) - new_jac.T @ Jbar.T) @ ddq    

            data.ctrl[self.actuator_ids] = tau
            data.ctrl[-1] = 0.0

            self.log(data)

            mujoco.mj_step(model, data)

            return tau 
        

    controller = OperationalSpaceController(model)


    def reset(model, data): 
        key_name = "home"
        key_id = model.key(key_name).id

        mujoco.mj_resetDataKeyframe(model, data, key_id)


        mocap_target_id = model.body("target_pos").mocapid[0]

        # random 90 degree rotation 
        # euler angles pick from either [-90, 0, 90] degrees for each axis
        rot = R.from_euler('x', 0).as_quat()

        data.mocap_pos[mocap_target_id][0:2] = np.array([0.5, 0.1])# + np.random.uniform(-0.1, 0.1, 2)
        data.mocap_pos[mocap_target_id][2] = 0.05
        data.mocap_quat[mocap_target_id] = rot

        data.qpos[9:11]= np.array([0.5, -0.1]) + np.random.uniform(-0.1, 0.1, 2)
        data.qpos[11] = 0.05

        data.mocap_pos[controller.mocap_id] = np.array([0.5, 0.0, 0.5])
        data.mocap_quat[controller.mocap_id] = np.array([0, 1, 0, 0])

        data.qvel[:] = 0.0

        rot = R.from_euler('x', np.random.uniform(-90, 90)).as_quat()

        data.qpos[12:16] = rot

        data.qacc_warmstart[:] = 0.0
        mujoco.mj_forward(model, data)

        for i in range(1000):

            # data.mocap_pos[controller]

            hand = viewer.get_hand_tracking() 
            frame = hand2pose(hand, side="right")

            data.mocap_pos[controller.mocap_id] = frame[:3, 3]
            data.mocap_quat[controller.mocap_id] = R.from_matrix(frame[:3, :3]).as_quat(scalar_first = True)
            mujoco.mj_forward(model, data)
            time.sleep(1/600.)
            viewer.sync()

        for i in range(1000):


            hand = viewer.get_hand_tracking() 
            frame = hand2pose(hand, side="right")

            mujoco.mj_forward(model, data)

            target_pos = frame[:3, 3]
            target_quat = R.from_matrix(frame[:3, :3]).as_quat(scalar_first = True)

            cur_pos = data.site(controller.site_id).xpos
            cur_quat = np.zeros(4)
            mujoco.mju_mat2Quat(cur_quat, data.site(controller.site_id).xmat)

            slerp = Slerp([0, 1], R.from_quat([cur_quat, target_quat]))
            alpha = i / 1000

            data.mocap_pos[controller.mocap_id] = cur_pos * (1 - alpha) + target_pos * alpha
            data.mocap_quat[controller.mocap_id] = slerp(alpha).as_quat()
            controller.pose2torque(model, data)
            time.sleep(1/600.)
            viewer.sync()
            


    def success(model, data):

        block_pos = data.body("cube").xpos
        target_pos = data.mocap_pos[model.body("target_pos").mocapid[0]]

        block_quat = data.body("cube").xquat
        target_quat = data.mocap_quat[model.body("target_pos").mocapid[0]]

        dist = np.linalg.norm(block_pos - target_pos)
        quat_diff = R.from_quat(block_quat).inv() * R.from_quat(target_quat)
        quat_diff = quat_diff.magnitude()
        
        if dist < 0.01 and quat_diff < 0.5:
            return True 
        else:
            return False


    ep_idx = args.start



    controller.clear_log()
    reset(model, data)

    while True: 

        hand = viewer.get_hand_tracking()
        frame = hand2pose(hand, side="right")

        data.mocap_pos[controller.mocap_id] = frame[:3, 3]
        # data.mocap_quat[controller.mocap_id] = R.from_matrix(frame[:3, :3]).as_quat(scalar_first = True) 

        controller.pose2torque(model, data)
        viewer.sync() 
        time.sleep(1/600.)

        if success(model, data):
            print("Success! Episode:", ep_idx)
            controller.save(ep_idx)
            controller.clear_log()
            reset(model, data)
            ep_idx += 1



if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--start", default=0, type=int)
    args = parser.parse_args()

    main(args)
