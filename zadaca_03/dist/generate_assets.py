import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
import cv2
import open3d as o3d

# Add src to path to import modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from core.config import AppConfig
from adapters.vision_adapter import PointCloudProcessor

def save_pcd_image(pcd, filename):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=800, height=600)
    
    # Optional: add coordinate frame
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    vis.add_geometry(coord)
    
    vis.add_geometry(pcd)
    
    # Set camera lookat
    ctr = vis.get_view_control()
    ctr.set_front([0.0, -1.0, -1.0])
    ctr.set_lookat([0.0, 0.0, 0.5])
    ctr.set_up([0.0, -1.0, 0.0])
    ctr.set_zoom(0.8)
    
    # Run one step
    vis.poll_events()
    vis.update_renderer()
    
    # Save image
    vis.capture_screen_image(filename)
    vis.destroy_window()

def generate_pointcloud_assets():
    print("Generating point cloud assets...")
    config = AppConfig()
    processor = PointCloudProcessor()
    
    run_dir = "data/raw/outputs/run_20260522_194852/view_1"
    
    color_img = cv2.imread(os.path.join(run_dir, "rgb.png"))
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
    depth_img = np.load(os.path.join(run_dir, "depth_m.npy"))
    
    # Load actual intrinsics
    try:
        intr = np.load(config.calibration.camera_matrix_path)
    except:
        intr = np.eye(3)
        
    # 1. Original PC
    original_pcd = processor.create_pcd_from_rgbd(color_img, depth_img, intr)
    save_pcd_image(original_pcd, "dist/assets/pc_original.png")
    
    # 2. Filtered PC
    filtered_pcd = processor.filter_pcd(original_pcd)
    save_pcd_image(filtered_pcd, "dist/assets/pc_filtered.png")
    
    # 3. Segmented PC (dummy objects just to show clustering)
    labels = np.array(filtered_pcd.cluster_dbscan(eps=0.03, min_points=50, print_progress=False))
    max_label = labels.max()
    segmented_pcd = o3d.geometry.PointCloud(filtered_pcd)
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0  # noise is black
    segmented_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    save_pcd_image(segmented_pcd, "dist/assets/pc_segmented.png")
    
    # 4. Final Merged PC
    final_pcd = o3d.io.read_point_cloud("data/processed/final_merged_point_cloud.pcd")
    save_pcd_image(final_pcd, "dist/assets/pc_final.png")
    print("Point cloud assets generated.")

def generate_trajectory_assets():
    print("Generating trajectory assets...")
    with open("data/processed/last_trajectory.json", "r") as f:
        traj = json.load(f)
    
    pts = traj["points"]
    t = [p["t"] for p in pts]
    x = [p["pos"][0] for p in pts]
    vx = [p["vel"][0] for p in pts]
    ax = [p["acc"][0] for p in pts]
    
    # Kinematics graph
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(t, x, 'b-')
    plt.ylabel('x [m]')
    plt.title('Kinematika prvog segmenta putanje u X smjeru')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(t, vx, 'g-')
    plt.ylabel('v_x [m/s]')
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(t, ax, 'r-')
    plt.ylabel('a_x [m/s^2]')
    plt.xlabel('t [s]')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig("dist/assets/traj_kinematics.png")
    plt.close()
    
    # 3D trajectory path
    fig = plt.figure(figsize=(10, 8))
    ax_3d = fig.add_subplot(111, projection='3d')
    
    X = [p["pos"][0] for p in pts]
    Y = [p["pos"][1] for p in pts]
    Z = [p["pos"][2] for p in pts]
    
    ax_3d.plot(X, Y, Z, 'b-', label='Putanja TCP-a')
    
    # We want exactly 5 standard waypoints:
    # A simple way to get 5 evenly spaced out waypoints along time
    wp_indices = np.linspace(0, len(pts)-1, 5, dtype=int)
        
    wp_x = [X[i] for i in wp_indices]
    wp_y = [Y[i] for i in wp_indices]
    wp_z = [Z[i] for i in wp_indices]
    
    ax_3d.scatter(wp_x, wp_y, wp_z, c='r', s=100, label='Glavne poze (P1-P5)')
    
    for i, (wx, wy, wz) in enumerate(zip(wp_x, wp_y, wp_z)):
        ax_3d.text(wx, wy, wz, f' P{i+1}', color='red', fontsize=12, fontweight='bold')
        
    ax_3d.set_xlabel('X [m]')
    ax_3d.set_ylabel('Y [m]')
    ax_3d.set_zlabel('Z [m]')
    ax_3d.set_title('3D Prikaz generirane trajektorije s 5 zadanih poza')
    ax_3d.legend()
    
    plt.savefig("dist/assets/traj_3d.png")
    plt.close()
    print("Trajectory assets generated.")

if __name__ == "__main__":
    generate_pointcloud_assets()
    generate_trajectory_assets()
