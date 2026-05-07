import numpy as np
import matplotlib.pyplot as plt
import os

# Putanja do podataka (provjeravamo root i src folder)
if os.path.exists("data/sim_data_primjer1.npz"):
    data_path = "data/sim_data_primjer1.npz"
elif os.path.exists("src/data/sim_data_primjer1.npz"):
    data_path = "src/data/sim_data_primjer1.npz"
else:
    # Ako fajl još ne postoji, probajmo potražiti onaj stari u src
    if os.path.exists("src/sim_data.npz"):
        data_path = "src/sim_data.npz"
    else:
        print("Greška: Podaci nisu pronađeni. Prvo pokreni simulaciju!")
        exit()

print(f"Učitavam podatke iz: {data_path}")
data = np.load(data_path)
t = data['t']
q = data['q']
qd = data['qd']
u = data['u']

# -------------------------
# FIGURE 1: POZICIJE
# -------------------------
fig1, axs1 = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
fig1.suptitle('Praćenje trajektorije (Pozicije)', fontsize=14)

for i in range(3):
    axs1[i].plot(t, qd[:, i], '--', label=f'q_d[{i}] (željeno)')
    axs1[i].plot(t, q[:, i], '-', label=f'q[{i}] (stvarno)')
    axs1[i].set_ylabel(f'Zglob {i+1} [rad]')
    axs1[i].legend(loc='upper right')
    axs1[i].grid(True, linestyle=':', alpha=0.7)

axs1[2].set_xlabel('Vrijeme [s]')
fig1.tight_layout()
fig1.subplots_adjust(top=0.92)
fig1.savefig(data_path.replace('.npz', '_pozicije.png'))

# -------------------------
# FIGURE 2: UPRAVLJAČKI SIGNALI (u)
# -------------------------
fig2, ax2 = plt.subplots(figsize=(10, 5))
fig2.suptitle('Upravljački signal', fontsize=14)

# Crtamo sva tri signala na istim osima
ax2.plot(t, u[:, 0], label='u[0]')
ax2.plot(t, u[:, 1], label='u[1]')
ax2.plot(t, u[:, 2], label='u[2]')

ax2.set_ylabel('control')
ax2.set_xlabel('time [s]')
ax2.legend(loc='upper left')
ax2.grid(True)

fig2.tight_layout()
fig2.subplots_adjust(top=0.90)
fig2.savefig(data_path.replace('.npz', '_upravljanje.png'))

print(f"Grafikoni su spremljeni kao slike u {os.path.dirname(data_path)}")
plt.show()
