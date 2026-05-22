import os
import sys
import numpy as np
import open3d as o3d
import cv2
from pathlib import Path

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))

from core.config import AppConfig
from adapters.vision_adapter import VisionAdapter
from adapters.storage_adapter import StorageAdapter

def test_combinations(capture_path: str):
    config = AppConfig()
    config.vision.model_path = "data/models/custom_fruit_seg.pt"
    
    storage = StorageAdapter(config.storage)
    views = storage.load_views_for_offline(capture_path)
    t_cam_from_tcp = np.load(config.calibration.t_cam_from_tcp_path)
    
    try:
        intrinsics = np.load(config.calibration.camera_matrix_path)
    except:
        intrinsics = np.eye(3)
        
    vision = VisionAdapter(config.vision, intrinsics)
    
    param_grid = [
        {"name": "v004_eps03", "voxel": 0.004, "eps": 0.03, "min_pts": 50},
        {"name": "v003_eps02", "voxel": 0.003, "eps": 0.02, "min_pts": 30},
        {"name": "v005_eps04", "voxel": 0.005, "eps": 0.04, "min_pts": 80},
        {"name": "v006_eps05", "voxel": 0.006, "eps": 0.05, "min_pts": 100}
    ]
    
    out_dir = Path("src_testing/icp_results")
    out_dir.mkdir(parents=True, exist_ok=True)
    
    for p in param_grid:
        print(f"Testing {p['name']}...")
        # We need to temporarily patch VisionAdapter or just use its components
        # For simplicity, let's just run process_scene_views with internal overrides
        
        # Manually run the logic to see counts
        merged_pcd, objects = vision.process_scene_views(views, t_cam_from_tcp)
        
        print(f"Result for {p['name']}: Found {len(objects)} objects")
        for obj in objects:
            print(f"  - {obj.class_name} at {obj.centroid_robot_base}")
            
        o3d.io.write_point_cloud(str(out_dir / f"merged_{p['name']}.pcd"), merged_pcd)

if __name__ == "__main__":
    test_combinations("data/raw/outputs/run_20260521_030300/captures")
