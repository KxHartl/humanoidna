# src_testing/final_precision_tuning.py
import sys
from pathlib import Path
import numpy as np
import open3d as o3d

# Dodajemo src_testing u path
sys.path.append(str(Path(__file__).parent))

from capture_scene import load_capture_scene_result, get_pairs_and_transforms_from_capture_result
from io_utils import ensure_dir, load_point_cloud, save_point_cloud
from segment import segment_multiple_views

def transform_cloud(pcd, T):
    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    return out

def run_precision_tuning(run_dir: Path, cfg):
    print(f"Pokrećem Finalno Precizno Tuning (X, Y, Z) za: {run_dir.name}")
    
    captures_dir = run_dir / "captures"
    capture_result = load_capture_scene_result(captures_dir)
    pairs, base_tcp_transforms = get_pairs_and_transforms_from_capture_result(capture_result)
    
    # Prvo segmentiramo da vidimo samo VOĆE (čišći pregled)
    print("Segmentiram voće za čišći prikaz...")
    segment_results = segment_multiple_views(pairs, cfg, output_dir=run_dir / "segment_test")
    
    # Učitaj originalnu Hand-Eye matricu
    H_orig = np.load("voce_zadaca/calibration/T_cam_from_tcp.npy")
    
    output_base = Path(__file__).parent / "precision_tuning_output" / run_dir.name
    ensure_dir(output_base)

    # Grid oko novih najboljih vrijednosti (dx=50, dy=20)
    # Korisnik traži dublji Z pomak (-30, -40)
    x_range = [0.045, 0.05, 0.055]
    y_range = [0.015, 0.02, 0.025]
    z_range = [-0.02, -0.03, -0.04]

    print(f"Generiram {len(x_range) * len(y_range) * len(z_range)} preciznih varijanti...")

    for dx in x_range:
        for dy in y_range:
            for dz in z_range:
                name = f"dx_{int(dx*1000)}mm_dy_{int(dy*1000)}mm_dz_{int(dz*1000)}mm"
                
                H_mod = H_orig.copy()
                H_mod[0, 3] += dx
                H_mod[1, 3] += dy
                H_mod[2, 3] += dz
                
                merged = o3d.geometry.PointCloud()
                
                for idx, (seg_res, Tb) in enumerate(zip(segment_results, base_tcp_transforms)):
                    # Uzmi samo segmentirano voće
                    if seg_res.best_instance_id is not None:
                        # Nađi pcd putanju za tu instancu
                        pcd_path = next(inst.segmented_pcd_path for inst in seg_res.instances if inst.instance_id == seg_res.best_instance_id)
                        pcd_cam = load_point_cloud(pcd_path)
                        
                        T_final = Tb @ H_mod
                        merged += transform_cloud(pcd_cam, T_final)
                
                if not merged.is_empty():
                    save_path = output_base / f"fruit_{name}.pcd"
                    save_point_cloud(save_path, merged)

    print("\n--- FINALNI TUNING GOTOV ---")
    print(f"Rezultati su u: {output_base}")
    print("Nađi savršenu datoteku. Kad je nađeš, javi mi brojeve i JA ĆU AŽURIRATI TVOJU .NPY DATOTEKU.")

if __name__ == "__main__":
    from config import cfg
    base_output = Path("voce_zadaca/output")
    run_dir = base_output / "run_20260521_011532"
    if run_dir.exists():
        run_precision_tuning(run_dir, cfg)
    else:
        print("Run nije pronađen.")
