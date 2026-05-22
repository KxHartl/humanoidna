# src_testing/brute_force_stitch.py
import sys
from pathlib import Path
import numpy as np
import open3d as o3d

# Dodajemo src_testing u path
sys.path.append(str(Path(__file__).parent))

from config import cfg
from capture_scene import load_capture_scene_result, get_pairs_and_transforms_from_capture_result
from io_utils import ensure_dir, load_point_cloud, save_point_cloud

def transform_cloud(pcd, T):
    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    return out

def run_brute_force(run_dir: Path):
    print(f"Pokrećem Brute-Force test za: {run_dir.name}")
    
    captures_dir = run_dir / "captures"
    capture_result = load_capture_scene_result(captures_dir)
    pairs, base_tcp_transforms = get_pairs_and_transforms_from_capture_result(capture_result)
    
    # Učitaj Hand-Eye matricu
    H = np.load("voce_zadaca/calibration/T_cam_from_tcp.npy")
    H_inv = np.linalg.inv(H)
    
    output_base = Path(__file__).parent / "brute_force_output" / run_dir.name
    ensure_dir(output_base)

    # Definiramo kombinacije koje ćemo testirati
    # H = Hand-Eye, Tb = Base-to-TCP
    combinations = [
        ("Tb_mul_H", lambda Tb, H: Tb @ H),
        ("Tb_mul_Hinv", lambda Tb, H: Tb @ np.linalg.inv(H)),
        ("H_mul_Tb", lambda Tb, H: H @ Tb),
        ("Hinv_mul_Tb", lambda Tb, H: np.linalg.inv(H) @ Tb),
        ("Tb_inv_mul_H", lambda Tb, H: np.linalg.inv(Tb) @ H),
    ]

    for name, transform_func in combinations:
        print(f"  Testiram: {name}...")
        merged = o3d.geometry.PointCloud()
        
        for idx, (pair, Tb) in enumerate(zip(pairs, base_tcp_transforms)):
            # Učitaj lokalni pointcloud (u cam frame-u)
            # Ovdje pretpostavljamo da koristimo SVE točke iz view-a za lakšu vizualizaciju
            pcd_cam = load_point_cloud(pair[1])
            
            try:
                T_final = transform_func(Tb, H)
                pcd_transformed = transform_cloud(pcd_cam, T_final)
                merged += pcd_transformed
            except Exception as e:
                print(f"    Greška za {name}: {e}")
                continue
        
        # Spremi rezultat
        save_path = output_base / f"merged_{name}.pcd"
        save_point_cloud(save_path, merged)
        print(f"    Spremljeno u: {save_path.name}")

    print("\n--- TEST GOTOV ---")
    print(f"Provjeri datoteke u: {output_base}")
    print("Otvori ih jednu po jednu u Open3D-u. Ona u kojoj je limun JEDAN (nema duhova) je dobitna kombinacija.")

if __name__ == "__main__":
    base_output = Path("voce_zadaca/output")
    runs = sorted([d for d in base_output.iterdir() if d.is_dir() and d.name.startswith("run_")])
    if runs:
        run_brute_force(runs[-1])
    else:
        print("Nema run-ova.")
