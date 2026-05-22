from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import open3d as o3d
from core.interfaces import IVisionAdapter
from core.models import SegmentedObject
from core.config import VisionConfig
import cv2
import torch
import copy
from ultralytics import YOLO

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

class PointCloudProcessor:
    def __init__(self): pass
    def create_pcd_from_rgbd(self, color_img: np.ndarray, depth_img: np.ndarray, intrinsics: np.ndarray) -> o3d.geometry.PointCloud:
        h, w = depth_img.shape
        o3d_c = o3d.geometry.Image(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB))
        o3d_d = o3d.geometry.Image(depth_img.astype(np.float32))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_c, o3d_d, 1.0, 1.5, False)
        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(w, h, intrinsics[0,0], intrinsics[1,1], intrinsics[0,2], intrinsics[1,2]))
    def filter_pcd(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        if pcd.is_empty(): return pcd
        p_d = pcd.voxel_down_sample(0.004)
        cl, ind = p_d.remove_statistical_outlier(25, 1.5)
        return p_d.select_by_index(ind).crop(o3d.geometry.AxisAlignedBoundingBox((-1,-1,0),(1,1,1.2)))

    def register_and_stitch(self, pcds: List[Tuple[o3d.geometry.PointCloud, np.ndarray]], strategy: int = 5) -> Tuple[o3d.geometry.PointCloud, List[Any]]:
        if not pcds: return o3d.geometry.PointCloud(), []
        n_v = len(pcds); import copy, itertools
        if n_v == 1:
            return pcds[0][0].voxel_down_sample(0.003), [np.eye(4)]
            
        prep = [copy.deepcopy(p[0]).transform(p[1]) for p in pcds]
        for p in prep: p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.02, 30))
        
        best_pair = None
        best_pair_score = -1.0
        best_pair_T = np.eye(4)
        best_voxel, best_thresh = 0.003, 0.03
        
        def calc_score(fit, rmse):
            return fit / (rmse + 1e-6)
        
        for i, j in itertools.combinations(range(n_v), 2):
            for voxel in [0.003, 0.005]:
                for thresh in [0.03, 0.05, 0.08]:
                    target = prep[i].voxel_down_sample(voxel); target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.02, 30))
                    source = prep[j].voxel_down_sample(voxel)
                    r1 = o3d.pipelines.registration.registration_icp(source, target, thresh, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane())
                    try:
                        r2 = o3d.pipelines.registration.registration_colored_icp(source, target, thresh/2, r1.transformation, o3d.pipelines.registration.TransformationEstimationForColoredICP())
                        fit, rmse, T = r2.fitness, r2.inlier_rmse, r2.transformation
                    except:
                        fit, rmse, T = r1.fitness, r1.inlier_rmse, r1.transformation
                        
                    score = calc_score(fit, rmse)
                    if score > best_pair_score:
                        best_pair_score = score
                        best_pair_T = T
                        best_pair = (i, j)
                        best_voxel, best_thresh = voxel, thresh

        A, B = best_pair
        merged = copy.deepcopy(prep[A])
        merged += copy.deepcopy(prep[B]).transform(best_pair_T)
        
        t_curr = [None] * n_v
        t_curr[A] = np.eye(4)
        t_curr[B] = best_pair_T
        
        rem = [k for k in range(n_v) if k not in (A, B)]
        
        for k in rem:
            target = merged.voxel_down_sample(best_voxel); target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.02, 30))
            source = prep[k].voxel_down_sample(best_voxel)
            r1 = o3d.pipelines.registration.registration_icp(source, target, best_thresh, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane())
            try:
                r2 = o3d.pipelines.registration.registration_colored_icp(source, target, best_thresh/2, r1.transformation, o3d.pipelines.registration.TransformationEstimationForColoredICP())
                fit, rmse, T = r2.fitness, r2.inlier_rmse, r2.transformation
            except:
                fit, rmse, T = r1.fitness, r1.inlier_rmse, r1.transformation
                
            score = calc_score(fit, rmse)
            if score >= best_pair_score * 0.85:
                merged += copy.deepcopy(prep[k]).transform(T)
                t_curr[k] = T
            else:
                print(f"  [Stitch] Rejected view {k} (Score {score:.2f} < {best_pair_score*0.85:.2f})")
                
        return merged.voxel_down_sample(0.003), t_curr

class VisionAdapter(IVisionAdapter):
    def __init__(self, config: VisionConfig, intrinsics: np.ndarray):
        self.config, self.intrinsics, self.model = config, intrinsics, None
        self.pc_processor, self.rs_pipeline, self.rs_align, self.rs_depth_scale = PointCloudProcessor(), None, None, 1.0
    def load_model(self) -> None: self.model = YOLO(self.config.model_path)
    def setup_realsense(self) -> bool:
        if rs is None: return False
        try:
            self.rs_pipeline = rs.pipeline(); c = rs.config()
            c.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30); c.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            p = self.rs_pipeline.start(c); self.rs_align = rs.align(rs.stream.color)
            self.rs_depth_scale = float(p.get_device().first_depth_sensor().get_depth_scale())
            for _ in range(15): self.rs_pipeline.wait_for_frames()
            return True
        except: return False
    def stop_realsense(self):
        if self.rs_pipeline: self.rs_pipeline.stop(); self.rs_pipeline = None
    def capture_live_view(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self.rs_pipeline: raise RuntimeError("RealSense not ready.")
        f = self.rs_pipeline.wait_for_frames(); a = self.rs_align.process(f)
        color = np.asanyarray(a.get_color_frame().get_data())
        depth = np.asanyarray(a.get_depth_frame().get_data()).astype(np.float32) * self.rs_depth_scale
        
        # Return BGR directly (YOLO and cv2 like BGR, PointCloudProcessor converts BGR2RGB)
        # Ne radimo rotaciju ovdje da bi sacuvali T_cam_from_tcp kalibraciju 3D prostora!
        return color.copy(), depth
        
    def process_scene_views(self, views: List[Dict[str, Any]], t_cam_from_tcp: np.ndarray, strategy: int = 5) -> Tuple[Any, List[SegmentedObject]]:
        if not self.model: self.load_model()
        all_pcds, view_objs_all = [], []
        for view in views:
            color, depth, tcp_mat = view["color"], view["depth"], view["tcp_matrix"]
            intr = view.get("intrinsics")
            if intr is None:
                intr = self.intrinsics
            cam_pose = tcp_mat @ t_cam_from_tcp
            all_pcds.append((self.pc_processor.filter_pcd(self.pc_processor.create_pcd_from_rgbd(color, depth, intr)), cam_pose))
            
            # YOLO detekcija na uspravnoj slici
            color_upright = cv2.rotate(color, cv2.ROTATE_180)
            res = self.model(color_upright, conf=self.config.confidence_threshold, verbose=False)
            v_objs = []
            if len(res) > 0 and res[0].masks is not None:
                m, b, n = res[0].masks.data.cpu().numpy(), res[0].boxes, res[0].names
                for i in range(len(m)):
                    mask_upright = cv2.resize(m[i], (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST) if m[i].shape != depth.shape else m[i]
                    mask_orig = cv2.rotate(mask_upright, cv2.ROTATE_180)
                    od = depth.copy(); od[mask_orig < 0.5] = 0
                    op = self.pc_processor.filter_pcd(self.pc_processor.create_pcd_from_rgbd(color, od, intr))
                    if len(op.points) > 10: 
                        op.transform(cam_pose)
                        v_objs.append((n[int(b.cls[i])], float(b.conf[i]), op))
            view_objs_all.append(v_objs)
                    
        stitched_pcd, icp_t = self.pc_processor.register_and_stitch(all_pcds, strategy)
        
        # 1. Gather all points from all views after registration
        pts_l, cols_l, conf_l, lab_l = [], [], [], []
        for i, v_objs in enumerate(view_objs_all):
            if icp_t[i] is None: continue
            for name, conf, op in v_objs:
                op_c = copy.deepcopy(op).transform(icp_t[i])
                pts_l.append(np.asarray(op_c.points))
                cols_l.append(np.asarray(op_c.colors))
                conf_l.append(np.full(len(op_c.points), conf))
                lab_l.append(np.full(len(op_c.points), name, dtype=object))

        if not pts_l:
            return stitched_pcd, []

        all_pts = np.concatenate(pts_l)
        all_cols = np.concatenate(cols_l)
        all_confs = np.concatenate(conf_l)
        all_labels = np.concatenate(lab_l)

        # 2. Hierarchical Clustering:
        # 2a. First cluster per-class to isolate instances
        candidate_objs = []
        unique_labels = np.unique(all_labels)
        eps = self.config.dbscan_eps
        min_pts = self.config.dbscan_min_pts

        for label in unique_labels:
            mask = (all_labels == label)
            l_pts, l_cols, l_confs = all_pts[mask], all_cols[mask], all_confs[mask]
            
            if len(l_pts) < min_pts: continue
            
            p_l = o3d.geometry.PointCloud()
            p_l.points = o3d.utility.Vector3dVector(l_pts)
            l_idx = np.array(p_l.cluster_dbscan(eps=eps, min_points=min_pts))
            
            for lbl in range(l_idx.max() + 1):
                c_mask = (l_idx == lbl)
                cp, cc, cf = l_pts[c_mask], l_cols[c_mask], l_confs[c_mask]
                
                res_pcd = o3d.geometry.PointCloud()
                res_pcd.points = o3d.utility.Vector3dVector(cp)
                res_pcd.colors = o3d.utility.Vector3dVector(cc)
                
                candidate_objs.append({
                    "label": str(label),
                    "confidence": float(np.mean(cf)),
                    "conf_sum": float(np.sum(cf)),
                    "centroid": cp.mean(axis=0),
                    "pcd": res_pcd
                })

        # 2b. Spatial centroid merge (handle mis-classifications of same physical object)
        # Strongest label (by conf_sum) wins the physical object.
        final_objects = []
        candidate_objs.sort(key=lambda x: x["conf_sum"], reverse=True)
        merge_dist_thresh = 0.05 # 5cm

        for cand in candidate_objs:
            is_merged = False
            for existing in final_objects:
                # Merge DIFFERENT classes if they are physically at the same spot
                if cand["label"] != existing.class_name:
                    dist = np.linalg.norm(cand["centroid"] - np.array(existing.centroid_robot_base))
                    if dist < merge_dist_thresh:
                        is_merged = True
                        break
            if not is_merged:
                obj = SegmentedObject(
                    instance_id=len(final_objects),
                    class_name=cand["label"],
                    confidence=cand["confidence"],
                    centroid_robot_base=tuple(cand["centroid"]),
                    pcd=cand["pcd"]
                )
                final_objects.append(obj)
                
        import colorsys
        for obj in final_objects:
            if obj.class_name in ["Naranca", "Crvena Jabuka"]:
                colors = np.asarray(obj.pcd.colors)
                if len(colors) > 0:
                    mean_rgb = np.mean(colors, axis=0)
                    h, s, v = colorsys.rgb_to_hsv(mean_rgb[0], mean_rgb[1], mean_rgb[2])
                    hue_deg = h * 360
                    # Red hue is around 0-15 and 345-360, Orange is 15-45
                    if hue_deg < 16 or hue_deg > 340:
                        obj.class_name = "Crvena Jabuka"
                    else:
                        obj.class_name = "Naranca"

        print(f"  [VisionAdapter] Hierarchical Perception: Found {len(candidate_objs)} raw clusters, merged into {len(final_objects)} unique objects.")
        return stitched_pcd, final_objects

