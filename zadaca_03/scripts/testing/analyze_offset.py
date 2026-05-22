# src_testing/analyze_offset.py
import numpy as np
import open3d as o3d
from pathlib import Path
import sys

# Add current dir to path
sys.path.append(str(Path(__file__).parent))

from capture_scene import load_capture_scene_result, get_pairs_and_transforms_from_capture_result
from io_utils import load_point_cloud

def analyze(run_dir: Path):
    captures_dir = run_dir / "captures"
    capture_result = load_capture_scene_result(captures_dir)
    pairs, base_tcp_transforms = get_pairs_and_transforms_from_capture_result(capture_result)
    
    # Trenutna matrica
    H = np.load("voce_zadaca/calibration/T_cam_from_tcp.npy")
    
    print(f"\n--- Analiza pomaka za: {run_dir.name} ---")
    print(f"Trenutna Hand-Eye translacija (x, y, z): {H[:3, 3]}")
    
    # Rekonstruiramo pojedinačne oblake u bazi
    view_centroids = []
    for idx, (pair, Tb) in enumerate(zip(pairs, base_tcp_transforms)):
        pcd_cam = load_point_cloud(pair[1])
        # P_base = Tb * H * P_cam
        T_base_cam = Tb @ H
        
        # Transformiramo cijeli cloud
        pcd_base = pcd_cam.transform(T_base_cam)
        
        # Centroid (za grubu procjenu)
        # NAPOMENA: Ovo je centroid CIJELOG view-a. 
        # Bolje bi bilo da ovdje koristimo samo segmentirani limun.
        points = np.asarray(pcd_base.points)
        if len(points) > 0:
            centroid = np.mean(points, axis=0)
            view_centroids.append(centroid)
            print(f"View {idx} centroid baze: {centroid}")

    if len(view_centroids) >= 2:
        dist = np.linalg.norm(view_centroids[0] - view_centroids[1])
        print(f"\nUdaljenost između view 0 i view 1: {dist*100:.2f} cm")
        
    print("\nSAVJET: Ako su limuni paralelni ali nisu poklopljeni,")
    print("vjerojatno trebate korigirati translaciju u T_cam_from_tcp.npy.")
    print("Izmjerite fizički (metrom) udaljenost od vrha prirubnice robota (TCP) ")
    print("do optičkog centra RealSense-a i usporedite s gore ispisanim [x, y, z].")

if __name__ == "__main__":
    base_output = Path("voce_zadaca/output")
    # Koristimo run koji je korisnik naveo kao najbolji
    run_dir = base_output / "run_20260521_011532"
    if run_dir.exists():
        analyze(run_dir)
    else:
        print(f"Run {run_dir} nije pronađen.")
