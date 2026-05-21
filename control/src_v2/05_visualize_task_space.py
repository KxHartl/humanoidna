import os
import numpy as np
import matplotlib.pyplot as plt

def main():
    # Pokušaj pronaći datoteku ovisno o radnom direktoriju
    file_path = "data/sim_data_task_space.npz"
    if not os.path.exists(file_path):
        alt_path = os.path.join(os.path.dirname(__file__), "..", "data", "sim_data_task_space.npz")
        if os.path.exists(alt_path):
            file_path = alt_path
        else:
            print(f"Greška: Datoteka nije pronađena na {file_path} niti na {alt_path}")
            return

    print(f"Učitavam podatke iz: {file_path}")
    data = np.load(file_path)
    
    t = data['t']
    q_ref = data['q_ref']
    q_meas = data['q_meas']
    dq_ref = data['dq_ref']
    dq_meas = data['dq_meas']
    u = data['u']

    num_joints = 7
    joint_names = [f"Joint {i+1}" for i in range(num_joints)]

    # Postavljanje stila
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

    # --- 1. Praćenje pozicije ---
    fig1, axes1 = plt.subplots(4, 2, figsize=(14, 12))
    fig1.suptitle("Zadatak 2: Praćenje pozicije ($q_{ref}$ vs $q_{meas}$)", fontsize=16, fontweight='bold')
    axes1 = axes1.flatten()
    for i in range(num_joints):
        axes1[i].plot(t, q_ref[:, i], label="Referenca ($q_{ref}$)", color='tab:blue', linestyle='--', linewidth=2)
        axes1[i].plot(t, q_meas[:, i], label="Mjereno ($q_{meas}$)", color='tab:orange', linewidth=1.5)
        axes1[i].set_title(joint_names[i], fontsize=12)
        axes1[i].set_xlabel("Vrijeme [s]")
        axes1[i].set_ylabel("Kut [rad]")
        axes1[i].legend(loc="upper right")
    axes1[-1].axis('off')  # Sakrij zadnji prazan subplot
    plt.tight_layout()

    # --- 2. Praćenje brzine ---
    fig2, axes2 = plt.subplots(4, 2, figsize=(14, 12))
    fig2.suptitle("Zadatak 2: Praćenje brzine ($\dot{q}_{ref}$ vs $\dot{q}_{meas}$)", fontsize=16, fontweight='bold')
    axes2 = axes2.flatten()
    for i in range(num_joints):
        axes2[i].plot(t, dq_ref[:, i], label="Referenca ($\dot{q}_{ref}$)", color='tab:green', linestyle='--', linewidth=2)
        axes2[i].plot(t, dq_meas[:, i], label="Mjereno ($\dot{q}_{meas}$)", color='tab:red', linewidth=1.5)
        axes2[i].set_title(joint_names[i], fontsize=12)
        axes2[i].set_xlabel("Vrijeme [s]")
        axes2[i].set_ylabel("Brzina [rad/s]")
        axes2[i].legend(loc="upper right")
    axes2[-1].axis('off')
    plt.tight_layout()

    # --- 3. Upravljački signal ---
    fig3, axes3 = plt.subplots(4, 2, figsize=(14, 12))
    fig3.suptitle("Zadatak 2: Upravljački signal $u(t)$ (pozicijska naredba)", fontsize=16, fontweight='bold')
    axes3 = axes3.flatten()
    for i in range(num_joints):
        axes3[i].plot(t, u[:, i], label="Upravljački signal $u(t)$", color='tab:purple', linewidth=2)
        axes3[i].set_title(joint_names[i], fontsize=12)
        axes3[i].set_xlabel("Vrijeme [s]")
        axes3[i].set_ylabel("Naredba [rad]")
        axes3[i].legend(loc="upper right")
    axes3[-1].axis('off')
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
