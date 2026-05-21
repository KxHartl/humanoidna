import os
import sys
import json
import numpy as np
import open3d as o3d
import cv2
from pathlib import Path
import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(__file__)))

from adapters.vision_adapter import PointCloudProcessor, VisionAdapter
from core.config import AppConfig
from adapters.storage_adapter import StorageAdapter

def create_trajectory_plot(traj_json_path, output_path):
    if not os.path.exists(traj_json_path):
        print(f"Trajektorija nije pronađena na {traj_json_path}")
        return
        
    with open(traj_json_path, 'r') as f:
        data = json.load(f)
        
    t = [pt["t"] for pt in data["points"]]
    x = [pt["pos"][0] for pt in data["points"]]
    y = [pt["pos"][1] for pt in data["points"]]
    z = [pt["pos"][2] for pt in data["points"]]
    v = [np.linalg.norm(pt["vel"]) for pt in data["points"]]
    
    plt.figure(figsize=(10, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(t, x, label="X [m]")
    plt.plot(t, y, label="Y [m]")
    plt.plot(t, z, label="Z [m]")
    plt.title("Trajektorija vrha robota - Pozicija")
    plt.ylabel("Pozicija [m]")
    plt.legend()
    plt.grid()
    
    plt.subplot(2, 1, 2)
    plt.plot(t, v, label="V_norm [m/s]", color='orange')
    plt.title("Trajektorija vrha robota - Brzina")
    plt.xlabel("Vrijeme [s]")
    plt.ylabel("Brzina [m/s]")
    plt.legend()
    plt.grid()
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"Graf trajektorije spremljen u {output_path}")

def generate_report_data():
    capture_dir = "data/raw/outputs/run_20260521_030300/captures"
    report_pc_dir = "data/for_report/pointclouds"
    report_rgb_dir = "data/for_report/pictures/rgb_views"
    report_traj_dir = "data/for_report/pictures/trajectories"
    
    os.makedirs(report_pc_dir, exist_ok=True)
    os.makedirs(report_rgb_dir, exist_ok=True)
    os.makedirs(report_traj_dir, exist_ok=True)
    
    config = AppConfig()
    storage = StorageAdapter(config.storage)
    processor = PointCloudProcessor()
    
    try:
        intrinsics = np.load(config.calibration.camera_matrix_path)
    except:
        intrinsics = np.eye(3)
    
    views = storage.load_views_for_offline(capture_dir)
    print(f"Pronađeno {len(views)} pogleda. Krećem s procesiranjem za izvještaj...")
    
    # Initialize VisionAdapter for YOLO segmentation
    vision = VisionAdapter(config.vision, intrinsics)
    print("Učitavam YOLO model...")
    vision.load_model()
    
    for i, view in enumerate(views):
        print(f"\n--- Obrada View {i} ---")
        
        # 1. Spremanje RGB slike
        rgb_path = os.path.join(report_rgb_dir, f"view_{i}_rgb.png")
        cv2.imwrite(rgb_path, view["color"])
        print(f"Spremljen RGB: {rgb_path}")
        
        # 2. Original PointCloud
        pcd_orig = processor.create_pcd_from_rgbd(view["color"], view["depth"], intrinsics)
        orig_path = os.path.join(report_pc_dir, f"view_{i}_original.pcd")
        o3d.io.write_point_cloud(orig_path, pcd_orig)
        print(f"Spremljen Originalni PCD: {orig_path}")
        
        # 3. Filtrirani PointCloud
        pcd_filt = processor.filter_pcd(pcd_orig)
        filt_path = os.path.join(report_pc_dir, f"view_{i}_filtered.pcd")
        o3d.io.write_point_cloud(filt_path, pcd_filt)
        print(f"Spremljen Filtrirani PCD: {filt_path}")
        
        # 4. Segmentirani PointCloudovi (YOLO)
        results = vision.model(view["color"], conf=0.5, verbose=False)
        if len(results) > 0 and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            boxes = results[0].boxes
            names = results[0].names
            
            for obj_idx, mask in enumerate(masks):
                cls_id = int(boxes.cls[obj_idx].item())
                orig_name = names[cls_id]
                
                # Ignoriraj ljude, stolice i sl. iz COCO dataseta
                if orig_name not in ['apple', 'banana', 'orange', 'broccoli', 'sports ball', 'lemon']:
                    continue
                
                class_name = orig_name
                
                # Resize mask to depth shape if needed
                if mask.shape != view["depth"].shape:
                    mask = cv2.resize(mask, (view["depth"].shape[1], view["depth"].shape[0]), interpolation=cv2.INTER_NEAREST)
                
                # Određivanje boje voća (za razlikovanje Crvene i Zelene jabuke, te mapiranje COCO u HR nazive)
                mask_bool = mask > 0
                if np.any(mask_bool):
                    hsv = cv2.cvtColor(view["color"], cv2.COLOR_BGR2HSV)
                    # Srednja Hue vrijednost unutar maske
                    mean_hue = np.median(hsv[:,:,0][mask_bool])
                    
                    if orig_name == 'apple':
                        if mean_hue > 30 and mean_hue < 90:
                            class_name = "Zelena_jabuka"
                        else:
                            class_name = "Crvena_jabuka"
                    elif orig_name == 'orange' or orig_name == 'sports ball':
                        class_name = "Naranca"
                    elif orig_name == 'banana' or orig_name == 'lemon':
                        class_name = "Limun"
                    elif orig_name == 'broccoli':
                        class_name = "Orah"
                    else:
                        class_name = "Nepoznato_voce"
                else:
                    continue
                    
                # Create isolated object point cloud
                obj_depth = view["depth"].copy()
                obj_depth[mask == 0] = 0
                
                obj_pcd = processor.create_pcd_from_rgbd(view["color"], obj_depth, intrinsics)
                obj_pcd = processor.filter_pcd(obj_pcd)
                
                if len(obj_pcd.points) < 50:
                    continue
                    
                seg_path = os.path.join(report_pc_dir, f"view_{i}_segmented_{class_name}_{obj_idx}.pcd")
                o3d.io.write_point_cloud(seg_path, obj_pcd)
                print(f"Spremljen Segmentirani PCD (Voće): {seg_path}")
        else:
            print("Nije pronađeno voće u ovom pogledu.")
    
    # 5. Kopiranje final merged point cloud
    merged_src = "data/processed/final_merged_point_cloud.pcd"
    merged_dst = os.path.join(report_pc_dir, "final_merged_result.pcd")
    if os.path.exists(merged_src):
        import shutil
        shutil.copy2(merged_src, merged_dst)
        print(f"\nKopiran završni PointCloud u {merged_dst}")
        
    # 6. Graf trajektorije
    traj_src = "data/processed/last_trajectory.json"
    traj_dst = os.path.join(report_traj_dir, "trajectory_kinematics.png")
    create_trajectory_plot(traj_src, traj_dst)

if __name__ == "__main__":
    generate_report_data()
