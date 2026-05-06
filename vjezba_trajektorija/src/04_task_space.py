import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import os

# ograničenja
v_max = 0.5  # m/s
a_max = 1.0  # m/s^2
omega_max = 1.0  # rad/s

# početna i konačna pozicija (m)
p_i = np.array([-0.30, -0.120, 0.3])
p_f = np.array([-0.225, -0.120, 0.33])

# početna i konačna orijentacija (stupnjevi)
R_i_deg = [90, 0, 0]
R_f_deg = [120, 0, 45]

# Rotacije
rot_i = R.from_euler('xyz', R_i_deg, degrees=True)
rot_f = R.from_euler('xyz', R_f_deg, degrees=True)

# Pomak translacije
D_p = np.linalg.norm(p_f - p_i)

# Kutna razlika
rot_diff = rot_i.inv() * rot_f
D_rot = rot_diff.magnitude()

# Vrijeme potrebno za translaciju
k_v = 15/8
k_a = 10 * np.sqrt(3) / 3

t_v_trans = k_v * D_p / v_max
t_a_trans = np.sqrt(k_a * D_p / a_max)

# Vrijeme potrebno za rotaciju
t_v_rot = k_v * D_rot / omega_max

t_f = max(t_v_trans, t_a_trans, t_v_rot)

print(f"Minimalno vrijeme: {t_f:.3f} s")

# diskretizacija
N = 1000
t = np.linspace(0, t_f, N)
tau = t / t_f

# Polinom 5. reda za s
s = 10*tau**3 - 15*tau**4 + 6*tau**5

# Translacija
p_t = p_i[:, None] + (p_f - p_i)[:, None] * s

# Rotacija (SLERP)
key_rots = R.concatenate([rot_i, rot_f])
key_times = [0, 1]
slerp = Slerp(key_times, key_rots)
rots_t = slerp(s)
rot_vecs = rots_t.as_rotvec()

# Plot
fig, axs = plt.subplots(2, 1, figsize=(8, 8))
for i, label in enumerate(['x', 'y', 'z']):
    axs[0].plot(t, p_t[i, :], label=label)
axs[0].set_title("Pozicija TCP-a (m)")
axs[0].legend()
axs[0].grid()

for i, label in enumerate(['rx', 'ry', 'rz']):
    axs[1].plot(t, rot_vecs[:, i], label=label)
axs[1].set_title("Orijentacija TCP-a (rotvec, rad)")
axs[1].legend()
axs[1].grid()

plt.tight_layout()

# 3D prikaz putanje
fig3d = plt.figure(figsize=(10, 8))
ax3d = fig3d.add_subplot(111, projection='3d')
ax3d.plot(p_t[0, :], p_t[1, :], p_t[2, :], label='TCP putanja', linewidth=2)
ax3d.scatter(p_i[0], p_i[1], p_i[2], color='tab:blue', s=50, label='start')
ax3d.scatter(p_f[0], p_f[1], p_f[2], color='tab:orange', s=50, label='cilj')

ax3d.set_xlabel('x [m]')
ax3d.set_ylabel('y [m]')
ax3d.set_zlabel('z [m]')
ax3d.set_title('Task-space putanja (quintic)')
ax3d.legend()
ax3d.grid(True)

# Spremanje trajektorije u datoteku
output_path = os.path.join(os.getcwd(), "trajectory_task_space.txt")
dt = t[1] - t[0]

with open(output_path, "w") as f:
    for k in range(N):
        pose = np.concatenate([p_t[:, k], rot_vecs[k, :]])
        # URScript get_inverse_kin converts pose to joint positions
        cmd = f"servoj(get_inverse_kin(p[{pose[0]:.5f}, {pose[1]:.5f}, {pose[2]:.5f}, {pose[3]:.5f}, {pose[4]:.5f}, {pose[5]:.5f}]), t={dt:.4f})\n"
        f.write(cmd)

print(f"Trajektorija spremljena u: {output_path}")
plt.show()
