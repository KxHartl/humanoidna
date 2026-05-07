import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# -------------------------
# MODEL
# -------------------------
model = mujoco.MjModel.from_xml_path("models/model.xml")
data = mujoco.MjData(model)

dt = model.opt.timestep

# -------------------------
# TRAJEKTORIJA
# -------------------------
T = 3.0

q0 = np.array([0.0, 0.0, 0.0])
qf = np.array([1.0, -0.8, 0.5])

def trajectory(t):
    if t >= T:
        return qf

    tau = t / T
    s = 10*tau**3 - 15*tau**4 + 6*tau**5

    return q0 + (qf - q0) * s

# -------------------------
# P REGULATOR
# -------------------------
Kp = np.array([80, 80, 80])

# -------------------------
# LOG VARIJABLE
# -------------------------
t_log = []
q_log = []
qd_log = []
u_log = []

# -------------------------
# SIMULACIJA
# -------------------------
sim_time = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:

    data.qpos[:] = q0
    data.qvel[:] = 0.0

    while viewer.is_running():

        q_d = trajectory(sim_time)
        q = data.qpos.copy()

        e = q_d - q
        u = Kp * e

        data.ctrl[:] = u

        # spremanje
        t_log.append(sim_time)
        q_log.append(q.copy())
        qd_log.append(q_d.copy())
        u_log.append(u.copy())

        mujoco.mj_step(model, data)
        viewer.sync()

        sim_time += dt
        time.sleep(0.01) # kako bi usporili prikaz simulacije

# -------------------------
# SPREMANJE U FILE
# -------------------------
if not os.path.exists("data"):
    os.makedirs("data")

filename = "data/sim_data_primjer1.npz"
np.savez(
    filename,
    t=np.array(t_log),
    q=np.array(q_log),
    qd=np.array(qd_log),
    u=np.array(u_log)
)

print(f"Podaci spremljeni u {filename}")
