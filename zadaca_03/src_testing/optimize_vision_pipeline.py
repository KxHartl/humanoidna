import os
import sys
import numpy as np
import open3d as o3d
import cv2
import copy
from typing import List, Dict, Any, Tuple
from ultralytics import YOLO

# Add src to path to use project modules
sys.path.append(os.path.abspath("src"))

from core.config import VisionConfig
from core.models import SegmentedObject
from adapters.vision_adapter import PointCloudProcessor
from adapters.storage_adapter import StorageAdapter

def main():
    capture_dir = "data/raw/outputs/run_20260522_150905"
    t_cam_from_tcp_path = "data/camera_calibration/T_cam_from_tcp.npy"
    model_path = "data/models/custom_fruit_seg.pt"
    intrinsics_path = "data/camera_calibration/camera_matrix.npy"

    if not os.path.exists(capture_dir):
        print(f"Error: {capture_dir} not found.")
        return

    print(f"Loading data from {capture_dir}...")
    from core.config import StorageConfig
    storage = StorageAdapter(StorageConfig())
    views = storage.load_views_for_offline(capture_dir)
    t_cam_from_tcp = np.load(t_cam_from_tcp_path)
    intrinsics = np.load(intrinsics_path)
    
    print(f"Loading model {model_path}...")
    model = YOLO(model_path)
    pc_processor = PointCloudProcessor()

    # Pre-process views: project to 3D for each view
    # We do this once to save time, but we need to run YOLO with different confidences
    # So we'll cache the color, depth, and camera pose.
    
    view_data = []
    for view in views:
        color = view["color"]
        depth = view["depth"]
        tcp_mat = view["tcp_matrix"]
        cam_pose = tcp_mat @ t_cam_from_tcp
        view_data.append({
            "color": color,
            "depth": depth,
            "cam_pose": cam_pose
        })

    # Deep dive into detections for all possible classes at very low confidence
    target_conf = 0.001 # Even lower
    print(f"\nScanning for all classes at confidence {target_conf}...")
    found_anywhere = set()
    for v_idx, v in enumerate(view_data):
        color_upright = cv2.rotate(v["color"], cv2.ROTATE_180)
        res = model(color_upright, conf=target_conf, verbose=False)
        if len(res) > 0:
            names = res[0].names
            classes = [names[int(c)] for c in res[0].boxes.cls]
            confs = res[0].boxes.conf.cpu().numpy()
            print(f"  View {v_idx} detections: {list(zip(classes, confs))[:15]}...") # Top 15
            for c in classes: found_anywhere.add(c)
    
    print(f"\nTotal unique classes found in any view at conf {target_conf}: {found_anywhere}")
    
    # Parameters to sweep
    conf_values = [0.20]
    eps_values = [0.030, 0.050, 0.080]
    min_pts_values = [30]

    best_total_score = -1
    best_params = None
    best_results = None

    print("\nStarting parameter sweep...")
    print(f"{'Conf':<6} {'Eps':<6} {'MinPts':<8} {'Found':<6} {'Unique':<8} {'Score':<8}")
    print("-" * 50)

    for conf in conf_values:
        # Run YOLO for all views with this confidence
        view_objs_all = []
        for v_idx, v in enumerate(view_data):
            color_upright = cv2.rotate(v["color"], cv2.ROTATE_180)
            res = model(color_upright, conf=conf, verbose=False)
            v_objs = []
            if len(res) > 0 and res[0].masks is not None:
                m, b, n = res[0].masks.data.cpu().numpy(), res[0].boxes, res[0].names
                print(f"  Conf {conf} | View {v_idx} | Raw Detections: {[n[int(c)] for c in b.cls]}")
                for i in range(len(m)):
                    cls_name = n[int(b.cls[i])]
                    mask_upright = cv2.resize(m[i], (v["depth"].shape[1], v["depth"].shape[0]), interpolation=cv2.INTER_NEAREST) if m[i].shape != v["depth"].shape else m[i]
                    mask_orig = cv2.rotate(mask_upright, cv2.ROTATE_180)
                    od = v["depth"].copy()
                    od[mask_orig < 0.5] = 0
                    
                    raw_pcd = pc_processor.create_pcd_from_rgbd(v["color"], od, intrinsics)
                    filtered_pcd = pc_processor.filter_pcd(raw_pcd)
                    
                    if len(filtered_pcd.points) > 10:
                        filtered_pcd.transform(v["cam_pose"])
                        v_objs.append((cls_name, float(b.conf[i]), filtered_pcd))
            view_objs_all.append(v_objs)

        for eps in eps_values:
            for min_pts in min_pts_values:
                # NEW STRATEGY: Cluster per-class first to avoid merging different fruits
                # But we want to allow merging if YOLO mis-classified (e.g. one fruit seen as Naranca in view 1 and Limun in view 2)
                # Actually, let's stick to global DBSCAN but use a very small eps.
                
                pts_l, cols_l, conf_l, lab_l = [], [], [], []
                for i, v_objs in enumerate(view_objs_all):
                    for name, c, op in v_objs:
                        pts_l.append(np.asarray(op.points))
                        cols_l.append(np.asarray(op.colors))
                        conf_l.append(np.full(len(op.points), c))
                        lab_l.append(np.full(len(op.points), name, dtype=object))

                if not pts_l: continue
                pts = np.concatenate(pts_l)
                confs = np.concatenate(conf_l)
                labels = np.concatenate(lab_l)

                # 1. Cluster per-class to find distinct instances of the same fruit type
                candidates = []
                unique_labels = np.unique(labels)
                for label in unique_labels:
                    mask = (labels == label)
                    l_pts, l_confs = pts[mask], confs[mask]
                    if len(l_pts) < min_pts: continue
                    p_l = o3d.geometry.PointCloud()
                    p_l.points = o3d.utility.Vector3dVector(l_pts)
                    idx_l = np.array(p_l.cluster_dbscan(eps=eps, min_points=min_pts))
                    for lbl in range(idx_l.max() + 1):
                        c_mask = (idx_l == lbl)
                        cp, cf = l_pts[c_mask], l_confs[c_mask]
                        candidates.append({
                            "label": label,
                            "centroid": cp.mean(axis=0),
                            "conf_sum": np.sum(cf)
                        })

                # 2. Merge candidates of DIFFERENT classes if they are very close (physical object merge)
                # But keep candidates of the same class separate if they didn't merge in step 1.
                final_objs = []
                candidates.sort(key=lambda x: x["conf_sum"], reverse=True)
                merge_dist = 0.04 # 4cm threshold
                
                for cand in candidates:
                    merged = False
                    for existing in final_objs:
                        # Only merge if they are DIFFERENT classes and close
                        if cand["label"] != existing["label"]:
                            dist = np.linalg.norm(cand["centroid"] - existing["centroid"])
                            if dist < merge_dist:
                                # They represent the same physical object, strongest label wins
                                merged = True
                                break
                    if not merged:
                        final_objs.append(cand)

                all_found_objs = [o["label"] for o in final_objs]
                num_found = len(all_found_objs)
                unique_classes = set(all_found_objs)
                num_unique = len(unique_classes)
                duplicates = num_found - num_unique
                score = num_unique * 10.0 - duplicates * 20.0
                
                if num_unique >= 5:
                    print(f"DEBUG (Hierarchical): conf={conf:.2f} eps={eps:.3f} min_pts={min_pts} -> {num_found} found, {num_unique} unique: {all_found_objs}")

                if score > best_total_score:
                    best_total_score, best_params, best_results = score, (conf, eps, min_pts), (num_found, num_unique, all_found_objs)
                    print(f"{conf:<6.2f} {eps:<6.3f} {min_pts:<8} {num_found:<6} {num_unique:<8} {score:<8.1f} *")
                # else:
                #    print(f"{conf:<6.2f} {eps:<6.3f} {min_pts:<8} {num_found:<6} {num_unique:<8} {score:<8.1f}")

    print("\n" + "="*50)
    print("BEST PARAMETERS FOUND:")
    print(f"YOLO Confidence: {best_params[0]}")
    print(f"DBSCAN Eps:      {best_params[1]}")
    print(f"DBSCAN Min Pts:  {best_params[2]}")
    print(f"Score:           {best_total_score}")
    print(f"Objects found:   {best_results[0]} total, {best_results[1]} unique")
    print(f"Classes:         {best_results[2]}")
    print("="*50)

if __name__ == "__main__":
    main()
