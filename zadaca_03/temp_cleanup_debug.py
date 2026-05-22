import sys
import os
import numpy as np
import open3d as o3d

sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.main import setup_components

def test_cleanup():
    capture_dir = r"D:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03\data\raw\outputs\run_20260522_145403"
    orchestrator = setup_components()
    orchestrator.robot.config.use_mock = True
    
    orchestrator.run_perception_offline(capture_dir, calib_tweak=0, strategy=5)
    
    print("\n--- After Cleanup ---")
    for obj in orchestrator.ctx.segmented_objects:
        pts = np.asarray(obj.pcd.points)
        median_centroid = np.median(pts, axis=0)
        
        # Run tighter DBScan
        labels = np.array(obj.pcd.cluster_dbscan(eps=0.02, min_points=15, print_progress=False))
        if len(labels) > 0 and labels.max() >= 0:
            unique_labels = np.unique(labels[labels >= 0])
            
            # Find the cluster whose mean is closest to the median centroid
            best_lbl = -1
            min_dist = 9999
            for lbl in unique_labels:
                cluster_mask = (labels == lbl)
                cluster_pts = pts[cluster_mask]
                cluster_center = np.mean(cluster_pts, axis=0)
                dist = np.linalg.norm(cluster_center - median_centroid)
                if dist < min_dist:
                    min_dist = dist
                    best_lbl = lbl
                    
            if best_lbl != -1:
                final_mask = (labels == best_lbl)
                clean_pcd = obj.pcd.select_by_index(np.where(final_mask)[0])
                
                bbox = clean_pcd.get_axis_aligned_bounding_box()
                extent = bbox.get_max_bound() - bbox.get_min_bound()
                print(f"[{obj.instance_id}] {obj.class_name}")
                print(f"  Old Extent: {obj.pcd.get_axis_aligned_bounding_box().get_max_bound() - obj.pcd.get_axis_aligned_bounding_box().get_min_bound()}")
                print(f"  New Extent: {extent}")
                print(f"  Points: {len(pts)} -> {len(clean_pcd.points)}")
        
if __name__ == "__main__":
    test_cleanup()
