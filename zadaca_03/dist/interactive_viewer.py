import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import cv2
import open3d as o3d

# Dodaj src u putanju
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from core.config import AppConfig
from adapters.vision_adapter import PointCloudProcessor, VisionAdapter
from adapters.storage_adapter import StorageAdapter

def view_pcd(geometries, window_name):
    print(f"Otvaram prozor: '{window_name}'")
    print("Koristi miša za rotaciju i zumiranje. Pritisni 'Q' ili zatvori prozor za nastavak na sljedeći korak.")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=1280, height=720, left=50, top=50)
    
    # Dodavanje koordinatnog sustava
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    vis.add_geometry(coord)
    
    if not isinstance(geometries, list):
        geometries = [geometries]
        
    for geom in geometries:
        vis.add_geometry(geom)
        
    opt = vis.get_render_option()
    opt.point_size = 2.0  # Smanjena veličina točkica (vizualnih "voxela")
    
    vis.run()
    vis.destroy_window()

def run_interactive_viewer(run_dir):
    print("Učitavam podatke...")
    config = AppConfig()
    config.vision.model_path = "data/models/custom_fruit_seg.pt"
    
    try:
        intrinsics = np.load(config.calibration.camera_matrix_path)
    except:
        intrinsics = np.eye(3)
        
    vision = VisionAdapter(config.vision, intrinsics)
    processor = vision.pc_processor
    
    try:
        t_cam_from_tcp = np.load(config.calibration.t_cam_from_tcp_path)
    except:
        t_cam_from_tcp = np.eye(4)
        
    storage = StorageAdapter(config.storage)
    views = storage.load_views_for_offline(run_dir)
    
    step = 1
    total_steps = 11
    
    # KORACI 1-9: Po 3 slike za svaki od 3 pogleda
    for i, view in enumerate(views):
        view_name = f"Pogled {i+1}"
        
        color = view["color"]
        depth = view["depth"]
        intr = view.get("intrinsics")
        if intr is None:
            intr = intrinsics
        
        # 1. Originalni PC
        original_pcd = processor.create_pcd_from_rgbd(color, depth, intr)
        view_pcd(original_pcd, f"{step}/{total_steps}: {view_name} - Originalni PC")
        step += 1
        
        # 2. Filtrirani PC
        # Smanjujemo voxel eksplicitno ako je potrebno (ili oslanjamo na point_size)
        filtered_pcd = processor.filter_pcd(original_pcd)
        view_pcd(filtered_pcd, f"{step}/{total_steps}: {view_name} - Filtrirani PC (Statisticki filter k=20, std=2.0)")
        step += 1
        
        # 3. Segmentirani PC
        if not vision.model:
            vision.load_model()
        
        # Originalni kod rotira sliku za 180 prije YOLO inferencije, a YOLO vraća maske
        color_upright = cv2.rotate(color, cv2.ROTATE_180)
        res = vision.model(color_upright, conf=vision.config.confidence_threshold, verbose=False)
        
        segmented_pcd = o3d.geometry.PointCloud()
        if res[0].masks is not None:
            masks = res[0].masks.data.cpu().numpy()
            for mask in masks:
                # Maske se moraju rotirati natrag za 180
                mask_back = cv2.rotate(mask, cv2.ROTATE_180)
                mask_back = cv2.resize(mask_back, (depth.shape[1], depth.shape[0]))
                
                # Primjena maske na dubinsku sliku i stvaranje PC-a
                masked_depth = depth.copy()
                masked_depth[mask_back < 0.5] = 0
                obj_pcd = processor.create_pcd_from_rgbd(color, masked_depth, intr)
                
                # Dodatni klastering (DBSCAN)
                cl, ind = obj_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                obj_pcd = obj_pcd.select_by_index(ind)
                
                labels = np.array(obj_pcd.cluster_dbscan(eps=0.03, min_points=50, print_progress=False))
                if len(labels) > 0:
                    max_label = labels.max()
                    if max_label >= 0:
                        counts = np.bincount(labels[labels >= 0])
                        largest_cluster_idx = np.argmax(counts)
                        obj_pcd = obj_pcd.select_by_index(np.where(labels == largest_cluster_idx)[0])
                        segmented_pcd += obj_pcd
        
        if len(segmented_pcd.points) == 0:
            segmented_pcd = filtered_pcd # fallback
            
        view_pcd(segmented_pcd, f"{step}/{total_steps}: {view_name} - Segmentirani PC (YOLO Maska + DBSCAN)")
        step += 1

    # KORAK 10: Konačni registrirani PC sva 3 pogleda
    merged_pcd, objects = vision.process_scene_views(views, t_cam_from_tcp, strategy=5)
    view_pcd(merged_pcd, f"{step}/{total_steps}: Registracija sva tri PC-a - ICP (prag 20mm)")
    step += 1
    
    # KORAK 11: Klasifikacija, centroidi i bounding box
    geometries_to_draw = [merged_pcd]
    
    for obj in objects:
        print(f"Detektiran objekt: {obj.class_name} na poziciji {obj.centroid_robot_base}")
        # Bounding box
        bbox = obj.pcd.get_axis_aligned_bounding_box()
        bbox.color = (1, 0, 0) # Crvena
        geometries_to_draw.append(bbox)
        
        # Centroid (crvena kuglica)
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.015)
        sphere.translate(obj.centroid_robot_base)
        sphere.paint_uniform_color([1, 0, 0])
        geometries_to_draw.append(sphere)
        
    view_pcd(geometries_to_draw, f"{step}/{total_steps}: Klasifikacija, Bounding Box i Centroid")

    print("Svi koraci pregledani!")

if __name__ == "__main__":
    run_path = "data/raw/outputs/run_20260522_194852"
    if len(sys.argv) > 1:
        run_path = sys.argv[1]
        
    print(f"Pokrećem napredni interaktivni preglednik za sve poglede u: {run_path}")
    run_interactive_viewer(run_path)
