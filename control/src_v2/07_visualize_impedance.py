import os
import numpy as np
import matplotlib.pyplot as plt

def main():
    file_path = "data/sim_data_impedance.npz"
    if not os.path.exists(file_path):
        alt_path = os.path.join(os.path.dirname(__file__), "..", "data", "sim_data_impedance.npz")
        if os.path.exists(alt_path):
            file_path = alt_path
        else:
            print(f"Greška: Datoteka nije pronađena na {file_path} niti na {alt_path}")
            print("Pokrenite prvo simulaciju 06_Panda_impedance.py kako bi se generirali podaci.")
            return

    print(f"Učitavam podatke iz: {file_path}")
    data = np.load(file_path)
    
    t = data['t']
    x = data['x']
    x_d = data['x_d']
    F_ext = data['F_ext']
    F_est = data['F_est']
    q = data['q']
    tau = data['tau']

    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

    # --- 1. Praćenje pozicije u Task Space-u ---
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig1.suptitle("Zadatak 4: Praćenje Kartezijeve pozicije EEF ($x_d$ vs $x$)", fontsize=16, fontweight='bold')
    labels = ['X [m]', 'Y [m]', 'Z [m]']
    colors_ref = ['tab:blue', 'tab:green', 'tab:purple']
    colors_meas = ['tab:orange', 'tab:red', 'tab:brown']
    
    for i in range(3):
        axes1[i].plot(t, x_d[:, i], label=f"Referenca {labels[i][0]}d", color=colors_ref[i], linestyle='--', linewidth=2)
        axes1[i].plot(t, x[:, i], label=f"Mjereno {labels[i][0]}", color=colors_meas[i], linewidth=1.5)
        axes1[i].set_ylabel(labels[i], fontsize=12)
        axes1[i].legend(loc="upper right")
        if i == 2:
            # Dodaj liniju za visinu stola
            axes1[i].axhline(0.22, color='gray', linestyle=':', label='Površina stola (z=0.22m)')
            axes1[i].legend(loc="upper right")
            axes1[i].set_title("Uočite ponašanje pri kontaktu sa stolom duž Z osi", fontsize=11)

    axes1[-1].set_xlabel("Vrijeme [s]", fontsize=12)
    plt.tight_layout()

    # --- 2. Usporedba Sila (Eksterna vs Procijenjena) ---
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig2.suptitle("Zadatak 4: Kontaktne sile na EEF ($F_{ext}$ vs $\hat{F}_{est}$)", fontsize=16, fontweight='bold')
    force_labels = ['Sila X [N]', 'Sila Y [N]', 'Sila Z [N]']
    
    for i in range(3):
        axes2[i].plot(t, F_ext[:, i], label=f"Eksterna sila iz MuJoCo ($F_{{ext,{force_labels[i][5]}}}$)", color='tab:red', linewidth=2)
        axes2[i].plot(t, F_est[:, i], label=f"Procijenjena sila iz momenata ($\hat{{F}}_{{est,{force_labels[i][5]}}}$)", color='tab:blue', linestyle='--', linewidth=1.5)
        axes2[i].set_ylabel(force_labels[i], fontsize=12)
        axes2[i].legend(loc="upper right")
        
    axes2[-1].set_xlabel("Vrijeme [s]", fontsize=12)
    plt.tight_layout()

    # --- 3. Momenti u zglobovima ---
    fig3, axes3 = plt.subplots(4, 2, figsize=(14, 12))
    fig3.suptitle("Zadatak 4: Upravljački momenti u zglobovima (Joint Space Torques)", fontsize=16, fontweight='bold')
    axes3 = axes3.flatten()
    for i in range(7):
        axes3[i].plot(t, tau[:, i], label=f"$\tau_{{{i+1}}}$", color='tab:brown', linewidth=1.5)
        axes3[i].set_title(f"Zglob {i+1}", fontsize=12)
        axes3[i].set_xlabel("Vrijeme [s]")
        axes3[i].set_ylabel("Moment [Nm]")
        axes3[i].legend(loc="upper right")
    axes3[-1].axis('off')
    plt.tight_layout()

    # --- 4. Kretanje zglobova (q) kroz vrijeme ---
    fig4, axes4 = plt.subplots(4, 2, figsize=(14, 12))
    fig4.suptitle("Prikaz kretanja zglobova ($q_{meas}$) kroz vrijeme", fontsize=16, fontweight='bold')
    axes4 = axes4.flatten()
    for i in range(7):
        axes4[i].plot(t, q[:, i], label=f"$q_{{{i+1}}}$", color='tab:olive', linewidth=1.5)
        axes4[i].set_title(f"Zglob {i+1}", fontsize=12)
        axes4[i].set_xlabel("Vrijeme [s]")
        axes4[i].set_ylabel("Kut [rad]")
        axes4[i].legend(loc="upper right")
    axes4[-1].axis('off')
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
