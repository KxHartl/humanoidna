# src_testing/test_reconstruct.py
import sys
import os
from pathlib import Path
import numpy as np
import open3d as o3d

# Add current dir to path to import copied files
sys.path.append(str(Path(__file__).parent))

from config import cfg
from capture_scene import load_capture_scene_result, get_pairs_and_transforms_from_capture_result
from reconstruct_scene import reconstruct_scene_from_pairs
from io_utils import ensure_dir

def test_offline_reconstruction(run_dir: Path):
    print(f"\n--- Testiranje rekonstrukcije za run: {run_dir.name} ---")
    
    # Putanja do foldera s capture-ima unutar run-a
    captures_dir = run_dir / "captures"
    if not captures_dir.exists():
        print(f"Greška: Ne postoji folder {captures_dir}")
        return

    # 1. Učitaj rezultate snimanja (slike, pointcloudove i MATRICE)
    try:
        capture_result = load_capture_scene_result(captures_dir)
    except Exception as e:
        print(f"Greška pri učitavanju capture rezultata: {e}")
        return

    pairs, base_tcp_transforms = get_pairs_and_transforms_from_capture_result(capture_result)
    print(f"Učitano {len(pairs)} view-a.")

    # 2. Pokreni rekonstrukciju (koja sada koristi isključivo matrice)
    output_test_dir = Path(__file__).parent / "output" / run_dir.name
    ensure_dir(output_test_dir)

    print("Pokrećem rekonstrukciju (segmentacija + transformacija pomoću matrica)...")
    try:
        # invert_cam_tcp=False jer pretpostavljamo da je T_cam_from_tcp ispravan
        # Ako dobiješ "razbacane" cloudove, probaj invert_cam_tcp=True
        reconstruction_result = reconstruct_scene_from_pairs(
            pairs=pairs,
            base_tcp_transforms=base_tcp_transforms,
            cfg=cfg,
            segment_output_dir=output_test_dir / "segment",
            reconstruct_output_dir=output_test_dir / "reconstruct",
            invert_cam_tcp=False
        )
        
        print(f"Rekonstrukcija gotova!")
        print(f"Putanja do stichanog clouda: {reconstruction_result.registered_pcd_path}")
        print(f"Izračunati centroid (Base): {reconstruction_result.centroid_base_xyz}")

        # 3. Vizualizacija (opcionalno, ako imaš GUI)
        if reconstruction_result.registered_pcd_path:
            pcd = o3d.io.read_point_cloud(reconstruction_result.registered_pcd_path)
            # Dodaj marker centroida za vizualnu provjeru
            centroid = np.array(reconstruction_result.centroid_base_xyz)
            marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            marker.paint_uniform_color([1, 0, 0])
            marker.translate(centroid)
            
            print("Prikazujem stichani pointcloud (zatvori prozor za nastavak)...")
            o3d.visualization.draw_geometries([pcd, marker], window_name=f"Test: {run_dir.name}")

    except Exception as e:
        print(f"Greška tijekom rekonstrukcije: {e}")

if __name__ == "__main__":
    # Ovdje upiši putanju do run-a koji želiš testirati
    # Primjer: zadnji run iz output direktorija
    base_output = Path("voce_zadaca/output")
    runs = sorted([d for d in base_output.iterdir() if d.is_dir() and d.name.startswith("run_")])
    
    if not runs:
        print("Nema pronađenih run-ova u voce_zadaca/output")
    else:
        # Testiraj zadnji run
        latest_run = runs[-1]
        test_offline_reconstruction(latest_run)
        
        # Ako želiš testirati neki specifičan run, odkomentiraj ovo:
        # specific_run = base_output / "run_20260521_005620"
        # test_offline_reconstruction(specific_run)
