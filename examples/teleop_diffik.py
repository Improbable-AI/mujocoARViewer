
from pathlib import Path

import mujoco
import mujoco.viewer
import mink

_HERE = Path(__file__).parent
_XML = _HERE / "shadow_hand" / "scene_left.xml"


if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())

    configuration = mink.Configuration(model)

    posture_task = mink.PostureTask(model, cost=1e-2)

    fingers = ["thumb", "first", "middle", "ring", "little"]
    finger_tasks = []
    for finger in fingers:
        task = mink.FrameTask(
            frame_name=finger,
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.0,
            lm_damping=1.0,
        )
        finger_tasks.append(task)

    tasks = [
        posture_task,
        *finger_tasks,
    ]

    model = configuration.model
    data = configuration.data
    solver = "daqp"

    from mujoco_arviewer import MJARViewer
    viewer = MJARViewer(avp_ip="192.168.0.1", enable_hand_tracking=True)
    viewer.load_scene(_XML.as_posix(), attach_to=[0, 0.3, 0.6, 180])
    viewer.register(model, data)

    # configuration.update_from_keyframe("grasp hard")

    # Initialize mocap bodies at their respective sites.
    posture_task.set_target_from_configuration(configuration)
    for finger in fingers:
        mink.move_mocap_to_frame(model, data, f"{finger}_target", finger, "site")

    dt = 1.0 / 500.0
    t = 0
    while True: 

        hand = viewer.get_hand_tracking()
        right_fingers = hand["right_wrist"] @ hand["right_fingers"]
        # Update posture target.
        mocap_ids = [mink.get_mocap_id(model, finger + "_target") for finger in fingers]
        finger_ids = [0, 4, 9, 14, 19]
        for mcid, fid in zip(mocap_ids, finger_ids):
            data.mocap_pos[mcid] = right_fingers[fid][:3, 3]

        # Update task target.
        for finger, task in zip(fingers, finger_tasks):
            task.set_target(
                mink.SE3.from_mocap_name(model, data, f"{finger}_target")
            )

        vel = mink.solve_ik(configuration, tasks, dt, solver, 1e-5)
        configuration.integrate_inplace(vel, dt)
        mujoco.mj_camlight(model, data)

        # Visualize at fixed FPS.
        viewer.sync()
        t += dt