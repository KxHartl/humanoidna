import numpy as np
import matplotlib.pyplot as plt
import os

# ograničenja robota - definirati prema datasheet-u
q_dot_max = np.array([
    np.deg2rad(360), 
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360)
])  # [rad/s] maksimalna brzina zgloba

# početna i konačna konfiguracija - definirati prema zadatku
q_i = np.array([-1.571, -2.094, -1.571, 0.524, 1.571, 0.000])
q_f = np.array([-0.785, -2.094, 0.000, -1.571, 0.000, 0.000])

# pomak
D = q_f - q_i

# minimalno vrijeme (po zglobu)
t_f_joints = np.abs(D) / q_dot_max

# ograničavajuća os
limiting_joint = np.argmax(t_f_joints)

t_f = np.max(t_f_joints)  # sinkronizacija svih zglobova

print(f"Minimalno vrijeme trajektorije: {t_f:.3f} s")
print(f"Ograničavajuća os: Joint {limiting_joint + 1}")
print(f"Vrijeme po zglobovima: {t_f_joints}")

# diskretizacija
N = 1000
t = np.linspace(0, t_f, N)

# linearna interpolacija
s = t / t_f
s_dot = 1 / t_f * np.ones_like(t)
s_ddot = np.zeros_like(t)

# trajektorije
q = q_i[:, None] + D[:, None] * s
q_dot = D[:, None] * s_dot
q_ddot = D[:, None] * s_ddot

# plot
fig, axs = plt.subplots(3, 1, figsize=(8, 10))

for i in range(6):
    axs[0].plot(t, q[i], label=f'Joint {i+1}')
    axs[1].plot(t, q_dot[i])
    axs[2].plot(t, q_ddot[i])

axs[0].set_title("Pozicija q(t)")
axs[1].set_title("Brzina q_dot(t)")
axs[2].set_title("Akceleracija q_ddot(t)")

for ax in axs:
    ax.grid()

axs[0].legend()

plt.tight_layout()

# spremanje trajektorije 
output_path = os.path.join(os.getcwd(), "trajectory.txt")

with open(output_path, "w") as f:
    for k in range(N):
        q_k = q[:, k]

        cmd = f"servoj([{q_k[0]:.4f}, {q_k[1]:.4f}, {q_k[2]:.4f}, {q_k[3]:.4f}, {q_k[4]:.4f}, {q_k[5]:.4f}], t={t_f/N:.4f})\n"
        f.write(cmd)

print(f"Trajektorija spremljena u: {output_path}")

plt.show()