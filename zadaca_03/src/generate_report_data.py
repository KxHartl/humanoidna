import os
import sys
import json
import shutil
import numpy as np
import open3d as o3d
from pathlib import Path
from rich.console import Console

# Add src to path to import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), "."))

from core.config import AppConfig
from adapters.vision_adapter import VisionAdapter
from adapters.storage_adapter import StorageAdapter

console = Console()

def generate_report_data(capture_path: str):
    capture_path = Path(capture_path)
    if not capture_path.exists():
        console.print(f"[red]Error: Capture path {capture_path} does not exist.[/red]")
        return

    run_name = capture_path.parent.name if capture_path.name == "captures" else capture_path.name
    report_base_dir = Path("data/for_report") / run_name
    report_base_dir.mkdir(parents=True, exist_ok=True)
    
    console.print(f"[bold green]Generating report data for: {run_name}[/bold green]")
    
    # 1. Setup Config and Adapter
    config = AppConfig()
    # Ensure it uses the new model
    config.vision.model_path = "data/models/custom_fruit_seg.pt"
    
    try:
        intrinsics = np.load(config.calibration.camera_matrix_path)
    except:
        intrinsics = np.eye(3)
        
    vision = VisionAdapter(config.vision, intrinsics)
    storage = StorageAdapter(config.storage)
    
    # 2. Load Calibration
    calib_dir = report_base_dir / "calibration"
    calib_dir.mkdir(exist_ok=True)
    shutil.copy(config.calibration.camera_matrix_path, calib_dir / "camera_matrix.npy")
    shutil.copy(config.calibration.dist_coeffs_path, calib_dir / "dist_coeffs.npy")
    shutil.copy(config.calibration.t_cam_from_tcp_path, calib_dir / "T_cam_from_tcp.npy")
    
    # 3. Process Scene (this will run ICP and segmentation)
    t_cam_from_tcp = np.load(config.calibration.t_cam_from_tcp_path)
    views = storage.load_views_for_offline(str(capture_path))
    
    # We want to intercept the point cloud processing to save intermediate steps
    # For report, we use strategy 5 (Smart Fitness Rejection)
    merged_pcd, objects = vision.process_scene_views(views, t_cam_from_tcp, strategy=5)
    
    # 4. Save Point Clouds (Original, Filtered, Segmented)
    pcd_dir = report_base_dir / "pointclouds"
    if pcd_dir.exists():
        shutil.rmtree(pcd_dir)
    pcd_dir.mkdir(parents=True, exist_ok=True)
    
    o3d.io.write_point_cloud(str(pcd_dir / "final_merged_result.pcd"), merged_pcd)
    
    # Re-run processing logic manually to save per-view intermediate clouds
    for i, view in enumerate(views):
        color = view["color"]
        depth = view["depth"]
        tcp_matrix = view["tcp_matrix"]
        cam_in_base = tcp_matrix @ t_cam_from_tcp
        intr = view.get("intrinsics", intrinsics)
        
        # Original
        pcd_orig = vision.pc_processor.create_pcd_from_rgbd(color, depth, intr)
        o3d.io.write_point_cloud(str(pcd_dir / f"view_{i}_original.pcd"), pcd_orig)
        
        # Filtered
        pcd_filt = vision.pc_processor.filter_pcd(pcd_orig)
        o3d.io.write_point_cloud(str(pcd_dir / f"view_{i}_filtered.pcd"), pcd_filt)
        
        # Save RGB for reference
        rgb_dir = report_base_dir / "pictures" / "rgb_views"
        rgb_dir.mkdir(parents=True, exist_ok=True)
        import cv2
        cv2.imwrite(str(rgb_dir / f"view_{i}_rgb.png"), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))

    # Save individual objects
    for obj in objects:
        clean_name = obj.class_name.replace(" ", "_")
        o3d.io.write_point_cloud(str(pcd_dir / f"object_{obj.instance_id}_{clean_name}.pcd"), obj.pcd)

    # 5. Save Object Data JSON
    planning_dir = report_base_dir / "planning"
    planning_dir.mkdir(exist_ok=True)
    
    obj_data = []
    for obj in objects:
        obj_data.append({
            "id": obj.instance_id,
            "class": obj.class_name,
            "confidence": obj.confidence,
            "centroid_robot_base": obj.centroid_robot_base
        })
    
    with open(planning_dir / "objects_detected.json", "w", encoding="utf-8") as f:
        json.dump(obj_data, f, indent=4)

    # 6. Copy Trajectory if available
    # Check if a trajectory was just generated (it would be in data/processed or capture/../../planning)
    # For now, let's look in data/processed/last_trajectory.json
    source_traj = Path("data/processed/last_trajectory.json")
    if source_traj.exists():
        shutil.copy(source_traj, planning_dir / "last_trajectory.json")
        console.print("[blue]Trajectory found and copied.[/blue]")
    else:
        # Check if it's in the capture's sibling folder if it's a standard run structure
        sibling_planning = capture_path.parent / "planning"
        if sibling_planning.exists():
            for f in sibling_planning.glob("*.json"):
                shutil.copy(f, planning_dir / f.name)
            console.print(f"[blue]Planning data copied from {sibling_planning}[/blue]")

    console.print(f"[bold green]Report generation complete! Files saved to: {report_base_dir}[/bold green]")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_path", help="Path to the captures directory")
    args = parser.parse_args()
    
    generate_report_data(args.capture_path)
