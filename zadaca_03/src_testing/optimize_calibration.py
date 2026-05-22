import sys
import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from adapters.storage_adapter import StorageAdapter
from core.config import StorageConfig
from adapters.vision_adapter import PointCloudProcessor

def test_calibration(capture_dir):
    storage = StorageAdapter(StorageConfig())
    views = storage.load_views_for_offline(capture_dir)
    
    t_cam_from_tcp_orig = np.load("data/camera_calibration/T_cam_from_tcp.npy")
    processor = PointCloudProcessor()
    
    best_fitness = -1.0
    best_angles = (0, 0, 0)
    
    print(f"Testing captures from {capture_dir}")
    
    # We will test combinations of rotations around X, Y, Z in steps of 90 degrees
    # because if the camera was flipped, it's usually by 90 or 180 degrees.
    for rx in [0, 90, 180, 270]:
        for ry in [0, 90, 180, 270]:
            for rz in [0, 90, 180, 270]:
                rot_matrix = np.eye(4)
                rot_matrix[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
                
                t_cam_from_tcp = t_cam_from_tcp_orig @ rot_matrix
                
                # Generate PCDs
                all_pcds = []
                for view in views:
                    color = view["color"]
                    depth = view["depth"]
                    intr = view.get("intrinsics")
                    if intr is None:
                        intr = np.load("data/camera_calibration/camera_matrix.npy")
                    tcp_mat = view["tcp_matrix"]
                    
                    cam_pose = tcp_mat @ t_cam_from_tcp
                    
                    # Create PointCloud
                    h, w = depth.shape
                    import cv2
                    o3d_c = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
                    o3d_d = o3d.geometry.Image(depth.astype(np.float32))
                    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_c, o3d_d, 1.0, 1.5, False)
                    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(w, h, intr[0,0], intr[1,1], intr[0,2], intr[1,2]))
                    
                    pcd = processor.filter_pcd(pcd)
                    all_pcds.append((pcd, cam_pose))
                
                # Try to register
                n_v = len(all_pcds)
                import copy
                prep = [copy.deepcopy(p[0]).transform(p[1]) for p in all_pcds]
                for p in prep: p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.02, 30))
                
                target = prep[0].voxel_down_sample(0.005)
                target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.02, 30))
                
                fits = []
                for i in range(1, n_v):
                    source = prep[i].voxel_down_sample(0.005)
                    r1 = o3d.pipelines.registration.registration_icp(source, target, 0.05, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane())
                    fits.append(r1.fitness)
                
                score = np.mean(fits) if fits else 0
                if score > 0.5:
                    print(f"rx={rx:3d}, ry={ry:3d}, rz={rz:3d} => Fitness: {score*100:.1f}%")
                    
                if score > best_fitness:
                    best_fitness = score
                    best_angles = (rx, ry, rz)
                    
    print("\n" + "="*50)
    print(f"BEST PARAMS: rx={best_angles[0]}, ry={best_angles[1]}, rz={best_angles[2]} with fitness {best_fitness*100:.1f}%")
    print("="*50)

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        test_calibration(sys.argv[1])
    else:
        print("Usage: python optimize_calibration.py <capture_dir>")
