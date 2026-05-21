import time
import numpy as np
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(
    r"C:\mujoco\mujoco_menagerie\unitree_h1\scene.xml"
)
data = mujoco.MjData(model)

# desna ruka nas zanima
joint_names = [
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
]

joint_ids = [
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    for name in joint_names
]

actuator_ids = [
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    for name in joint_names
]

qpos_ids = np.array([model.jnt_qposadr[jid] for jid in joint_ids])
dof_ids = np.array([model.jnt_dofadr[jid] for jid in joint_ids])

print("\nControlled joints/actuators:")
for name, jid, aid, qid, did in zip(joint_names, joint_ids, actuator_ids, qpos_ids, dof_ids):
    print(f"{name:25s} joint_id={jid:2d}, actuator_id={aid:2d}, qpos_id={qid:2d}, dof_id={did:2d}")

# generiranje reference
q_start = np.array([0.0, 0.0, 0.0, 0.0])
q_goal = np.array([-0.7, 0.35, 0.25, -1.0])

T = 3.0  # demonstracijske posture transition reference

def quintic_reference(t, T, q0, qf):
    """
    Glatka interpolacija reference:
    qd(0)=q0, qd(T)=qf
    qd_dot(0)=qd_dot(T)=0
    """
    tau = np.clip(t / T, 0.0, 1.0)

    s = 10*tau**3 - 15*tau**4 + 6*tau**5
    s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / T

    qd = q0 + s * (qf - q0)
    qd_dot = s_dot * (qf - q0)

    return qd, qd_dot

# PD torque control
Kp = np.array([25.0, 25.0, 15.0, 12.0])
Kd = np.array([3.0, 3.0, 2.0, 1.5])

# sigurnosno ograničenje momenta
torque_limit = np.array([40.0, 40.0, 18.0, 18.0])

# simulacija
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    while viewer.is_running():
        t = time.time() - start_time

        mujoco.mj_forward(model, data)

        q = data.qpos[qpos_ids].copy()
        qdot = data.qvel[dof_ids].copy()

        qd, qd_dot = quintic_reference(t, T, q_start, q_goal)

        # PD torque law:
        tau = Kp * (qd - q) + Kd * (qd_dot - qdot)

        tau = np.clip(tau, -torque_limit, torque_limit)

        data.ctrl[:] = 0.0
        data.ctrl[actuator_ids] = tau

        mujoco.mj_step(model, data)
        viewer.sync()

        time.sleep(model.opt.timestep)
