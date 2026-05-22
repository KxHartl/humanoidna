# src_testing/brute_force_translation.py
import sys
from pathlib import Path
import numpy as np
import open3d as o3d

# Dodajemo src_testing u path
sys.path.append(str(Path(__file__).parent))

from capture_scene import load_capture_scene_result, get_pairs_and_transforms_from_capture_result
from io_utils import ensure_dir, load_point_cloud, save_point_cloud

def transform_cloud(pcd, T):
    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    return out

def run_translation_tuning(run_dir: Path):
    print(f"Pokrećem Translation Tuning za: {run_dir.name}")
    
    captures_dir = run_dir / "captures"
    capture_result = load_capture_scene_result(captures_dir)
    pairs, base_tcp_transforms = get_pairs_and_transforms_from_capture_result(capture_result)
    
    # Učitaj originalnu Hand-Eye matricu
    H_orig = np.load("voce_zadaca/calibration/T_cam_from_tcp.npy")
    
    output_base = Path(__file__).parent / "translation_tuning_output" / run_dir.name
    ensure_dir(output_base)

    # Definiramo grid pretraživanja na temelju tvog feedbacka
    # x: 2 do 4 cm (0.02 do 0.04 m)
    # y: 1 do 2 cm (0.01 do 0.02 m)
    # Testirat ćemo i pozitivne i negativne pomake jer ne znamo u kojem smjeru bježi
    x_offsets = [-0.04, -0.02, 0.0, 0.02, 0.04]
    y_offsets = [-0.02, -0.01, 0.0, 0.01, 0.02]

    print(f"Generiram {len(x_offsets) * len(y_offsets)} varijanti...")

    for dx in x_offsets:
        for dy in y_offsets:
            name = f"dx_{int(dx*100)}cm_dy_{int(dy*100)}cm"
            
            # Modificiraj matricu H
            H_mod = H_orig.copy()
            H_mod[0, 3] += dx
            H_mod[1, 3] += dy
            
            merged = o3d.geometry.PointCloud()
            
            for idx, (pair, Tb) in enumerate(zip(pairs, base_tcp_transforms)):
                pcd_cam = load_point_cloud(pair[1])
                # Koristimo dobitnu formulu: T_final = Tb @ H
                T_final = Tb @ H_mod
                pcd_transformed = transform_cloud(pcd_cam, T_final)
                merged += pcd_transformed
            
            # Spremi rezultat
            save_path = output_base / f"merged_{name}.pcd"
            save_point_cloud(save_path, merged)
            # print(f"    Spremljeno: {name}")

    print("\n--- TUNING GOTOV ---")
    print(f"Provjeri datoteke u: {output_base}")
    print("Otvori ih i nađi onu u kojoj je limun JEDAN (savršeno poklopljen).")
    print("Kad je nađeš, javi mi 'dx' i 'dy' iz imena datoteke pa ćemo trajno ažurirati kalibraciju.")

if __name__ == "__main__":
    base_output = Path("voce_zadaca/output")
    # Koristimo run koji si naveo
    run_dir = base_output / "run_20260521_011532"
    if run_dir.exists():
        run_translation_tuning(run_dir)
    else:
        print("Run nije pronađen.")
