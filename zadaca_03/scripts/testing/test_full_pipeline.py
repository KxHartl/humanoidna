# src_testing/test_full_pipeline.py
import sys
from pathlib import Path
import time
import numpy as np
import open3d as o3d
from dataclasses import asdict

# Dodajemo src_testing u path
sys.path.append(str(Path(__file__).parent))

from config import cfg
from capture_scene import capture_scene, get_pairs_and_transforms_from_capture_result
from reconstruct_scene import reconstruct_scene_from_pairs
from io_utils import ensure_dir, save_json

def run_test_pipeline():
    print("=== POKRETANJE TESTNOG PIPELINE-A (src_testing) ===")
    
    # 1. Definiranje izlaznog direktorija za ovaj testni run
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    test_run_dir = Path(__file__).parent / "output" / f"test_run_{timestamp}"
    ensure_dir(test_run_dir)
    
    print(f"\n[KORAK 1/3] Snimanje scene (3 poze)...")
    # capture_scene ce: 
    # - spojiti se na robot i RealSense
    # - otici u 3 poze definirane u config.py
    # - u svakoj pozi uzeti sliku i TOČNU matricu transformacije robota
    try:
        capture_result = capture_scene(
            cfg=cfg,
            output_dir=test_run_dir / "captures",
            move_robot=True # Postavi na False ako želiš samo testirati bez micanja robota
        )
        print(f"Snimanje završeno. Snimljeno view-a: {capture_result.views_captured}")
    except Exception as e:
        print(f"Greška tijekom snimanja: {e}")
        return

    # Dohvati parove (slika, pcd) i pripadajuće matrice transformacije
    pairs, base_tcp_transforms = get_pairs_and_transforms_from_capture_result(capture_result)

    print(f"\n[KORAK 2/3] Rekonstrukcija i Stitching (isključivo matrice)...")
    # reconstruct_scene_from_pairs ce:
    # - segmentirati svaku sliku (YOLO)
    # - izdvojiti pointcloudove voća
    # - transformirati ih u base frame koristeći matrice s robota
    # - spojiti ih u jedan globalni cloud (bez ICP-a)
    try:
        reconstruction_result = reconstruct_scene_from_pairs(
            pairs=pairs,
            base_tcp_transforms=base_tcp_transforms,
            cfg=cfg,
            segment_output_dir=test_run_dir / "segment",
            reconstruct_output_dir=test_run_dir / "reconstruct",
            invert_cam_tcp=False # Provjeri treba li invertirati ovisno o kalibraciji
        )
        print(f"Rekonstrukcija gotova. Centroid voća: {reconstruction_result.centroid_base_xyz}")
    except Exception as e:
        print(f"Greška tijekom rekonstrukcije: {e}")
        return

    print(f"\n[KORAK 3/3] Vizualizacija rezultata...")
    if reconstruction_result.registered_pcd_path:
        # Učitaj spojeni oblak
        pcd = o3d.io.read_point_cloud(reconstruction_result.registered_pcd_path)
        
        # Stvori marker za centroid (crvena kugla)
        centroid = np.array(reconstruction_result.centroid_base_xyz)
        marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.015)
        marker.paint_uniform_color([1, 0, 0]) # Crvena
        marker.translate(centroid)
        
        # Vizualizacija
        print("Otvaram prozor s rezultatima. Provjeri poklapaju li se oblaci iz različitih kutova!")
        o3d.visualization.draw_geometries([pcd, marker], window_name=f"Test Stitching: {cfg.target.target_class}")

    # Spremi sumu testa
    save_json(test_run_dir / "test_summary.json", asdict(reconstruction_result))
    print(f"\nSvi podaci su spremljeni u: {test_run_dir}")

if __name__ == "__main__":
    # NAPOMENA: Prije pokretanja provjeri:
    # 1. Je li robot upaljen i dostupan na IP adresi u config.py
    # 2. Je li RealSense spojen
    # 3. Jesu li instalirane biblioteke: pip install pyrealsense2 ur_rtde ultralytics open3d
    
    run_test_pipeline()
