import numpy as np
import matplotlib.pyplot as plt
import os
import glob

# Tražimo sve .npz datoteke u data folderu
npz_files = glob.glob("data/*.npz")

if not npz_files:
    # Ako skriptu pokrećemo iz roota umjesto iz src
    npz_files = glob.glob("src/data/*.npz")

if not npz_files:
    print("Greška: Nisu pronađene .npz datoteke! Prvo pokrenite simulaciju.")
    exit()

tasks = [
        "primjer1",
        "zadatak1_step",
        "zadatak2_sine",
        "zadatak3_step",
        "zadatak4_step",
        "zadatak5_step",
        "zadatak6_step",
        "zadatak6_2_step"
    ]

for data_path in npz_files:
    print(f"Obradujem: {data_path}")
    data = np.load(data_path)
    t = data['t']
    q = data['q']
    qd = data['qd']
    u = data['u']
    
    # -------------------------
    # FIGURE 1: POZICIJE
    # -------------------------
    fig1, axs1 = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    task_name = os.path.basename(data_path).replace('.npz', '')
    fig1.suptitle(f'{task_name} - Pracenje trajektorije', fontsize=14)

    for i in range(3):
        axs1[i].plot(t, qd[:, i], '--', label=f'q_d[{i}] (zeljeno)')
        axs1[i].plot(t, q[:, i], '-', label=f'q[{i}] (stvarno)')
        axs1[i].set_ylabel(f'Zglob {i+1} [rad]')
        axs1[i].legend(loc='upper right')
        axs1[i].grid(True, linestyle=':', alpha=0.7)

    axs1[2].set_xlabel('Vrijeme [s]')
    fig1.tight_layout()
    fig1.subplots_adjust(top=0.92)
    out1 = data_path.replace('.npz', '_pozicije.png')
    fig1.savefig(out1)
    plt.close(fig1)

    # -------------------------
    # FIGURE 2: UPRAVLJAČKI SIGNALI (u)
    # -------------------------
    fig2, ax2 = plt.subplots(figsize=(10, 3.5))
    fig2.suptitle(f'{task_name} - Upravljacki signal', fontsize=14)

    # Crtamo sva tri signala na istim osima
    ax2.plot(t, u[:, 0], label='u[0]')
    ax2.plot(t, u[:, 1], label='u[1]')
    ax2.plot(t, u[:, 2], label='u[2]')

    ax2.set_ylabel('Upravljacki signal [Nm]')
    ax2.set_xlabel('Vrijeme [s]')
    ax2.legend(loc='upper left')
    ax2.grid(True)

    fig2.tight_layout()
    fig2.subplots_adjust(top=0.90)
    out2 = data_path.replace('.npz', '_upravljanje.png')
    fig2.savefig(out2)
    plt.close(fig2)

print("Svi grafikoni su uspjesno spremljeni u data folder.")
