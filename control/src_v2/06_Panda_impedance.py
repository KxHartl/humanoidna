import time
import os
import numpy as np
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(
    r"C:\mujoco\mujoco_menagerie\franka_emika_panda\panda_table.xml"
)

data = mujoco.MjData(model)

q_init = np.array([
     0.0,
    -0.3,
     0.0,
    -2.5,
     0.0,
     2.2,
     0.8
])

data.qpos[:7] = q_init
data.qvel[:7] = 0.0
mujoco.mj_forward(model, data)

body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")

joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7"]
joint_ids   = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names]
dof_ids     = np.array([model.jnt_dofadr[j] for j in joint_ids])

x0  = data.xpos[body_id].copy()
R_d = data.xmat[body_id].copy().reshape(3, 3)

print(f"Početna pozicija TCP: {x0}")

z_table   = 0.22
T_descent = 6.0

x_target    = x0.copy()
x_target[2] = z_table

print(f"Ciljna pozicija:      {x_target}")


def desired_trajectory(t):
    if t >= T_descent:
        return x_target.copy(), np.zeros(3)
    tau  = t / T_descent
    s    = 10*tau**3 - 15*tau**4 + 6*tau**5
    sdot = (30*tau**2 - 60*tau**3 + 30*tau**4) / T_descent
    x_d    = x0 + (x_target - x0) * s
    xdot_d = (x_target - x0) * sdot
    return x_d, xdot_d


def rotation_error(R, R_d):
    R_err = R_d @ R.T - R @ R_d.T
    return 0.5 * np.array([R_err[2,1], R_err[0,2], R_err[1,0]])


def damped_pinv(J, damping=0.01):
    return J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(J.shape[0]))


Md_t     = np.diag([0.5, 0.5, 0.5])
Dd_t     = np.diag([80.0, 80.0, 80.0])
Kd_t     = np.diag([400.0, 400.0, 400.0])
Md_t_inv = np.linalg.inv(Md_t)

Kp_rot = 300.0
Kd_rot = 20.0

x_prev   = x0.copy()
xdot_est = np.zeros(3)

t_log       = []
x_log       = []
x_d_log     = []
e_pos_log   = []
e_rot_log   = []
F_ext_log   = []
F_est_log   = []
q_log       = []
dq_log      = []
tau_log     = []


with mujoco.viewer.launch_passive(model, data) as viewer:

    t          = 0.0
    dt         = model.opt.timestep
    step_count = 0

    print("\n=== IMPEDANCE CONTROL + NULL-SPACE + STOL ===")
    print(f"Robot se spušta prema stolu za {T_descent} sekundi.")
    print("Ctrl+klik za interakciju!\n")

    while viewer.is_running():

        x  = data.xpos[body_id].copy()
        R  = data.xmat[body_id].copy().reshape(3, 3)
        q  = data.qpos[dof_ids].copy()
        dq = data.qvel[dof_ids].copy()

        xdot_est = (x - x_prev) / dt
        x_prev   = x.copy()

        x_d, xdot_d = desired_trajectory(t)

        e_pos    = x_d - x
        edot_pos = xdot_d - xdot_est

        cfrc  = data.cfrc_ext[body_id]
        F_ext = cfrc[3:6]

        acc_cmd = Md_t_inv @ (Dd_t @ edot_pos + Kd_t @ e_pos + F_ext)

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        Jp = jacp[:, dof_ids]   # (3, 7)

        M_full = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, M_full, data.qM)
        M = M_full[np.ix_(dof_ids, dof_ids)]

        Jp_pinv = damped_pinv(Jp)   # (7, 3)

        # task space moment
        tau_task = M @ Jp_pinv @ acc_cmd

        # null-space projektor — (I - Jp^T @ Jp^T+)
        # primjenjuje joint regularizaciju samo gdje ne utječe na task space
        JpT      = Jp.T                          # (7, 3)
        JpT_pinv = np.linalg.pinv(JpT)          # (3, 7)
        N        = np.eye(7) - JpT @ JpT_pinv   # (7, 7) null-space projektor

        tau_rot_raw = Kp_rot * (q_init - q) - Kd_rot * dq
        tau_rot     = N @ tau_rot_raw            # projicirano u null-space

        tau_grav = data.qfrc_bias[dof_ids]
        tau_cmd  = tau_task + tau_rot + tau_grav

        # procjena kontaktne sile u task spaceu iz momenata
        # F_est = (Jp^T)^+ @ (tau_cmd - tau_grav)
        tau_contact = tau_cmd - tau_grav
        F_est       = JpT_pinv @ tau_contact     # (3,)

        data.qfrc_applied[dof_ids] = tau_cmd
        data.ctrl[:7] = 0.0

        e_rot = rotation_error(R, R_d)

        t_log.append(t)
        x_log.append(x.copy())
        x_d_log.append(x_d.copy())
        e_pos_log.append(e_pos.copy())
        e_rot_log.append(e_rot.copy())
        F_ext_log.append(F_ext.copy())
        F_est_log.append(F_est.copy())
        q_log.append(q.copy())
        dq_log.append(dq.copy())
        tau_log.append(tau_cmd.copy())

        if step_count % 500 == 0:
            print(f"t={t:.2f} | "
                  f"z={x[2]:.4f} m | "
                  f"z_ref={x_d[2]:.4f} m | "
                  f"|e_pos|={np.linalg.norm(e_pos):.4f} m | "
                  f"|e_rot|={np.linalg.norm(e_rot):.4f} rad | "
                  f"|F_ext|={np.linalg.norm(F_ext):.2f} N | "
                  f"|F_est|={np.linalg.norm(F_est):.2f} N")

        mujoco.mj_step(model, data)
        viewer.sync()

        t          += dt
        step_count += 1


os.makedirs("data", exist_ok=True)

np.savez(
    "data/sim_data_impedance.npz",
    t     = np.array(t_log),
    x     = np.array(x_log),
    x_d   = np.array(x_d_log),
    e_pos = np.array(e_pos_log),
    e_rot = np.array(e_rot_log),
    F_ext = np.array(F_ext_log),
    F_est = np.array(F_est_log),
    q     = np.array(q_log),
    dq    = np.array(dq_log),
    tau   = np.array(tau_log)
)

print("Podaci spremljeni u data/sim_data_impedance.npz")
print(f"Broj koraka: {len(t_log)}")
print(f"Trajanje simulacije: {t_log[-1]:.2f} s")