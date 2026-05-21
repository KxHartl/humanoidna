# apply_calibration_fix.py
import numpy as np
from pathlib import Path
import shutil

def apply_fix():
    calib_path = Path("voce_zadaca/calibration/T_cam_from_tcp.npy")
    backup_path = Path("voce_zadaca/calibration/T_cam_from_tcp_backup.npy")
    
    if not calib_path.exists():
        print(f"Greška: {calib_path} ne postoji.")
        return
        
    # 1. Napravi backup originala
    shutil.copy(calib_path, backup_path)
    print(f"Backup napravljen: {backup_path}")
    
    # 2. Učitaj i modificiraj matricu
    T = np.load(calib_path)
    
    # Pobjednički parametri: dx=50mm, dy=20mm, dz=-30mm
    dx = 0.050
    dy = 0.020
    dz = -0.030
    
    T[0, 3] += dx
    T[1, 3] += dy
    T[2, 3] += dz
    
    # 3. Spremi popravljenu matricu
    np.save(calib_path, T)
    
    print("\n--- KALIBRACIJA USPJEŠNO AŽURIRANA ---")
    print(f"Nova translacija u T_cam_from_tcp.npy: {T[:3, 3]}")
    print("Sada će glavni program koristiti ove popravljene vrijednosti.")

if __name__ == "__main__":
    apply_fix()
