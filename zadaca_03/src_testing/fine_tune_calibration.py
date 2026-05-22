import sys
import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import scipy.optimize as opt

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from adapters.storage_adapter import StorageAdapter
from core.config import StorageConfig
from adapters.vision_adapter import PointCloudProcessor

def fine_tune(capture_dir):
    storage = StorageAdapter(StorageConfig())
    views = storage.load_views_for_offline(capture_dir)
    
    t_cam_from_tcp_orig = np.load("data/camera_calibration/T_cam_from_tcp.npy")
    processor = PointCloudProcessor()
    
    base_pcds = []
    
    # Pre-generate point clouds in local camera frame
    for view in views:
        color = view["color"]
        depth = view["depth"]
        intr = view.get("intrinsics")
        if intr is None:
            intr = np.load("data/camera_calibration/camera_matrix.npy")
        tcp_mat = view["tcp_matrix"]
        
        h, w = depth.shape
        import cv2
        o3d_c = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
        o3d_d = o3d.geometry.Image(depth.astype(np.float32))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_c, o3d_d, 1.0, 1.5, False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(w, h, intr[0,0], intr[1,1], intr[0,2], intr[1,2]))
        
        pcd = processor.filter_pcd(pcd)
        pcd.estimate_normals()
        base_pcds.append((pcd.voxel_down_sample(0.005), tcp_mat))

    def evaluate_tweak(params):
        rx, ry, rz, tx, ty, tz = params
        rot_matrix = np.eye(4)
        rot_matrix[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        rot_matrix[0, 3] = tx
        rot_matrix[1, 3] = ty
        rot_matrix[2, 3] = tz
        
        T = t_cam_from_tcp_orig @ rot_matrix
        
        import copy
        prep = []
        for pcd, tcp_mat in base_pcds:
            cam_pose = tcp_mat @ T
            prep.append(copy.deepcopy(pcd).transform(cam_pose))
            
        target = prep[0]
        fits = []
        for i in range(1, len(prep)):
            # Evaluate fitness before ICP
            res = o3d.pipelines.registration.evaluate_registration(prep[i], target, 0.05, np.eye(4))
            fits.append(res.fitness)
            
        return -np.mean(fits) # Minimize negative fitness

    print(f"Starting optimization for {capture_dir}...")
    
    # We will do a simple grid search to find a good starting point, then optimize
    best_score = 1.0 # (which means 0 fitness since it's negated)
    best_params = [0, 0, 0, 0, 0, 0]
    
    # Grid search over rotations
    for rx in [-5, 0, 5]:
        for ry in [-5, 0, 5]:
            for rz in [-15, 0, 15]:
                score = evaluate_tweak([rx, ry, rz, 0, 0, 0])
                if score < best_score:
                    best_score = score
                    best_params = [rx, ry, rz, 0, 0, 0]
                    
    print(f"Grid search best initial params: {best_params} with initial fitness {-best_score*100:.2f}%")
    
    # Now run Nelder-Mead optimization
    bounds = [(-15, 15), (-15, 15), (-30, 30), (-0.05, 0.05), (-0.05, 0.05), (-0.05, 0.05)]
    res = opt.minimize(evaluate_tweak, best_params, method='Nelder-Mead', options={'maxiter': 100, 'xatol': 1e-3, 'fatol': 1e-3})
    
    print("\n" + "="*50)
    print(f"OPTIMIZATION COMPLETE")
    print(f"Best parameters (rx, ry, rz, tx, ty, tz):")
    print(np.round(res.x, 4))
    print(f"Best fitness (overlap): {-res.fun*100:.2f}%")
    print("="*50)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        fine_tune(sys.argv[1])
    else:
        print("Usage: python fine_tune_calibration.py <capture_dir>")
