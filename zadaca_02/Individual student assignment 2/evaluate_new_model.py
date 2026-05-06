import os
import cv2
import numpy as np
import open3d as o3d
from ultralytics import YOLO
import json

class PoseEstimator:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.class_name = "Crvena jabuka"
        self.class_id = 1 # From data.yaml

    def fit_sphere(self, points):
        """Fits a sphere to a set of 3D points using least squares."""
        if len(points) < 4:
            return None, None
        
        A = np.zeros((len(points), 4))
        A[:, 0] = 2 * points[:, 0]
        A[:, 1] = 2 * points[:, 1]
        A[:, 2] = 2 * points[:, 2]
        A[:, 3] = 1
        
        f = points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2
        
        C, residuals, rank, s = np.linalg.lstsq(A, f, rcond=None)
        
        xc, yc, zc = C[0], C[1], C[2]
        R = np.sqrt(C[3] + xc**2 + yc**2 + zc**2)
        
        return np.array([xc, yc, zc]), R

    def process_scene(self, image_path, pcd_path, scene_name):
        img = cv2.imread(image_path)
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        h, w = 480, 640
        points_3d = points.reshape((h, w, 3))
        
        results = self.model(img, conf=0.5)[0]
        
        detections = []
        geometries = [pcd]
        
        if results.masks is not None:
            for i, mask in enumerate(results.masks.data):
                cls = int(results.boxes.cls[i])
                if cls == self.class_id:
                    m = mask.cpu().numpy()
                    m = cv2.resize(m, (w, h))
                    mask_indices = m > 0.5
                    
                    instance_points = points_3d[mask_indices]
                    valid_mask = np.all(np.abs(instance_points) > 1e-6, axis=1) & ~np.any(np.isnan(instance_points), axis=1)
                    instance_points = instance_points[valid_mask]
                    
                    if len(instance_points) < 10:
                        continue
                        
                    pcd_instance = o3d.geometry.PointCloud()
                    pcd_instance.points = o3d.utility.Vector3dVector(instance_points)
                    pcd_instance, _ = pcd_instance.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                    instance_points = np.asarray(pcd_instance.points)
                    
                    # Centroid AABB
                    aabb = pcd_instance.get_axis_aligned_bounding_box()
                    centroid_aabb = aabb.get_center()
                    
                    # Sphere Fitting
                    centroid_sphere, radius = self.fit_sphere(instance_points)
                    
                    dist = np.linalg.norm(centroid_aabb - centroid_sphere) if centroid_sphere is not None else -1
                    
                    detections.append({
                        "scene": scene_name,
                        "id": i,
                        "centroid_aabb": centroid_aabb.tolist(),
                        "centroid_sphere": centroid_sphere.tolist() if centroid_sphere is not None else None,
                        "radius": float(radius) if radius is not None else -1.0,
                        "distance": float(dist)
                    })
                    
        return detections, geometries

def run_evaluation():
    model_path = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\Individual student assignment 2\runs\yolo26n_seg_fruit\weights\best.pt"
    if not os.path.exists(model_path):
        model_path = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\Individual student assignment 2\runs\yolo26n_seg_fruit\weights\last.pt"
        if not os.path.exists(model_path):
            print(f"Model not found at {model_path}. Training might still be in progress.")
            return

    print(f"Loading model from {model_path}...")
    estimator = PoseEstimator(model_path)
    
    # Also evaluate the model using ultralytics built-in method
    eval_data_path = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\datasets\humanoidna_voce_crvena_jabuka.yolov8\data_abs.yaml"
    val_results = estimator.model.val(data=eval_data_path)
    print("Validation results on evaluation set (built-in):")
    print(val_results)

    raw_dir = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\datasets\humanoidna_voce_crvena_jabuka_raw"
    all_results = []
    
    files = os.listdir(raw_dir)
    images = sorted([f for f in files if f.endswith('.png')])
    
    print(f"Found {len(images)} images. Processing first 10 for pose estimation...")
    for img_name in images[:10]:
        base = img_name.replace('_image_', '_').replace('.png', '')
        pcd_name = img_name.replace('_image_', '_point_cloud_').replace('.png', '.pcd')
        
        img_path = os.path.join(raw_dir, img_name)
        pcd_path = os.path.join(raw_dir, pcd_name)
        
        if os.path.exists(pcd_path):
            detections, geometries = estimator.process_scene(img_path, pcd_path, base)
            print(f"  {base}: Detected {len(detections)} instances of '{estimator.class_name}'")
            all_results.extend(detections)
            
    with open(r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\Individual student assignment 2\pose_results_2.json", "w") as f:
        json.dump(all_results, f, indent=4)
    print(f"Saved results for {len(all_results)} detections to pose_results_2.json")

if __name__ == "__main__":
    run_evaluation()
