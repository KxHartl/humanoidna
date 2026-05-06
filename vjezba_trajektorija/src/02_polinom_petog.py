import numpy as np
import matplotlib.pyplot as plt
import os

# ograničenja robota
q_dot_max = np.array([
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360)
])  # [rad/s] 
q_ddot_max = np.array([
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(360),
    np.deg2rad(720),
    np.deg2rad(720),
    np.deg2rad(720)
])  # [rad/s^2]

# početna i konačna konfiguracija
q_i = np.array([-1.571, -2.094, -1.571, 0.524, 1.571, 0.000])
q_f = np.array([-0.785, -2.094, 0.000, -1.571, 0.000, 0.000])

# pomak
D = q_f - q_i

# faktor za maksimalnu brzinu kod 5. reda
k_v = 1.875 # 15/8
# faktor za akceleraciju
k_a = 5.7735 # 10 * sqrt(3) / 3

# vrijeme po zglobu (brzina)
t_vel = k_v * np.abs(D) / q_dot_max
# vrijeme po zglobu (akceleracija)
t_acc = np.sqrt(k_a * np.abs(D) / q_ddot_max)

# uzmi kritično ograničenje
t_f_joints = np.maximum(t_vel, t_acc)
t_f = np.max(t_f_joints)

# ograničenja
limiting_joint = np.argmax(t_f_joints)
is_acc_dominant = t_acc[limiting_joint] > t_vel[limiting_joint]

print(f"Minimalno vrijeme: {t_f:.3f} s")
print(f"Ograničavajuća os: Joint {limiting_joint + 1}")
print(f"Vrijeme po zglobovima: {t_f_joints}")
print(f"Kritično ograničenje: {'akceleracija' if is_acc_dominant else 'brzina'}")

# diskretizacija
N = 1000
t = np.linspace(0, t_f, N)
tau = t / t_f

# quintic interpolacija
s = 10*tau**3 - 15*tau**4 + 6*tau**5
s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / t_f
s_ddot = (60*tau - 180*tau**2 + 120*tau**3) / (t_f**2)

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
output_path = os.path.join(os.getcwd(), "trajectory_fifth_order.txt")

dt = t[1] - t[0]

with open(output_path, "w") as f:
    for k in range(N):
        q_k = q[:, k]
        cmd = f"servoj([{q_k[0]:.4f}, {q_k[1]:.4f}, {q_k[2]:.4f}, {q_k[3]:.4f}, {q_k[4]:.4f}, {q_k[5]:.4f}], t={dt:.4f})\n"
        f.write(cmd)

print(f"Trajektorija spremljena u: {output_path}")

plt.show()