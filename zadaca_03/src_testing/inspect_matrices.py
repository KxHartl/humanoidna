# src_testing/inspect_matrices.py
import numpy as np
from pathlib import Path

def inspect():
    p = Path("voce_zadaca/calibration/T_cam_from_tcp.npy")
    if not p.exists():
        print(f"File not found: {p}")
        return
        
    T = np.load(p)
    print(f"\n--- Matrica: {p.name} ---")
    print(np.array2string(T, precision=6, suppress_small=True))
    
    # Provjera translacije (zadnji stupac)
    translation = T[:3, 3]
    print(f"\nTranslacija (x, y, z): {translation}")
    
    # Ako su vrijednosti tipa 50, 100, vjerojatno su milimetri
    if np.any(np.abs(translation) > 1.0):
        print("!!! UPOZORENJE: Translacija ima vrijednosti veće od 1.0. Vjerojatno je u MILIMETRIMA, a robot koristi METRE.")
    else:
        print("Translacija se čini u METRIMA (vrijednosti < 1.0).")

    # Provjera rotacijskog dijela (mora biti ortonormalan)
    R = T[:3, :3]
    det = np.linalg.det(R)
    print(f"Determinanta rotacijske matrice: {det:.6f} (treba biti 1.0)")
    
    # Ispis inverzne matrice za usporedbu
    T_inv = np.linalg.inv(T)
    print("\n--- Inverzna Matrica (T_inv) ---")
    print(np.array2string(T_inv, precision=6, suppress_small=True))

if __name__ == "__main__":
    inspect()
