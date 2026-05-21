import time
import os
import numpy as np
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(
    r"C:\mujoco\mujoco_menagerie\franka_emika_panda\scene.xml"
)

data = mujoco.MjData(model)

q_init = np.array([
     0.0,
    -0.5,
     0.0,
    -2.0,
     0.0,
     1.5,
     0.8
])

data.qpos[:7] = q_init
mujoco.mj_forward(model, data)

body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")

joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7"]
joint_ids   = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names]
dof_ids     = np.array([model.jnt_dofadr[j] for j in joint_ids])

# početni podaci
x0      = data.xpos[body_id].copy()
R0      = data.xmat[body_id].copy().reshape(3, 3)
q_ref   = q_init.copy()
qdot_ref = np.zeros(7)

# pojačanja (za upravljački zakon u task space-u)
K_pos = 15
K_rot = 10

# generiranje trajektorije
def desired_trajectory(t):
    r = 0.08
    w = 3.5
    x_d     = x0 + np.array([0.0, r*(np.cos(w*t)-1.0), r*np.sin(w*t)])
    xdot_d  = np.array([0.0, -r*w*np.sin(w*t), r*w*np.cos(w*t)])
    R_d     = R0.copy()
    omega_d = np.zeros(3)
    return x_d, xdot_d, R_d, omega_d

# rotacijska greška
def rotation_error(R, R_d):
    R_err = R_d @ R.T - R @ R_d.T
    return 0.5 * np.array([R_err[2,1], R_err[0,2], R_err[1,0]])

# pseudoinverz
def damped_pinv(J, damping=0.03):
    return J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(6))


t_log       = []
q_ref_log   = []
q_meas_log  = []
dq_ref_log  = []
dq_meas_log = []
u_log       = []

# simulacija
with mujoco.viewer.launch_passive(model, data) as viewer:

    t = 0.0
    dt = model.opt.timestep
    step_count = 0

    while viewer.is_running():

        x  = data.xpos[body_id].copy()
        R  = data.xmat[body_id].copy().reshape(3, 3)
        q  = data.qpos[dof_ids].copy()
        dq = data.qvel[dof_ids].copy()

        x_d, xdot_d, R_d, omega_d = desired_trajectory(t)

        # definiranje pozicijske i rotacijske pogreške
        e_pos = x_d - x
        e_rot = rotation_error(R, R_d)

        # upravljački zakon u task space-u
        v_cmd     = xdot_d  + K_pos * e_pos
        omega_cmd = omega_d + 2.0 * K_rot * e_rot
        task_cmd  = np.concatenate([v_cmd, omega_cmd])

        # preslikavanje u prostor zglobova
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        J = np.vstack([jacp[:, dof_ids], jacr[:, dof_ids]])

        qdot_ref  = damped_pinv(J) @ task_cmd
        q_ref    += qdot_ref * dt

        # slanje naredbe aktuatorima
        u = q_ref.copy()
        data.ctrl[:7] = u

        t_log.append(t)
        q_ref_log.append(q_ref.copy())
        q_meas_log.append(q.copy())
        dq_ref_log.append(qdot_ref.copy())
        dq_meas_log.append(dq.copy())
        u_log.append(u.copy())

        mujoco.mj_step(model, data)
        viewer.sync()

        t          += dt
        step_count += 1


os.makedirs("data", exist_ok=True)

np.savez(
    "data/sim_data_task_space.npz",
    t        = np.array(t_log),
    q_ref    = np.array(q_ref_log),
    q_meas   = np.array(q_meas_log),
    dq_ref   = np.array(dq_ref_log),
    dq_meas  = np.array(dq_meas_log),
    u        = np.array(u_log)
)

print("Podaci spremljeni u data/sim_data_task_space.npz")
print(f"Broj koraka: {len(t_log)}")
print(f"Trajanje simulacije: {t_log[-1]:.2f} s")
