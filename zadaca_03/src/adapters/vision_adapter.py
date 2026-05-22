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

    def register_and_stitch(self, pcds: List[Tuple[o3d.geometry.PointCloud, np.ndarray]], strategy: int = 5) -> Tuple[o3d.geometry.PointCloud, List[np.ndarray]]:
        if not pcds: return o3d.geometry.PointCloud(), []
        n_v = len(pcds); import copy
        prep = [copy.deepcopy(p[0]).transform(p[1]) for p in pcds]
        for p in prep: p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.02, 30))
        
        best_merged, best_t, best_score = None, [np.eye(4)] * n_v, -1.0
        for voxel in [0.003, 0.005]:
            for thresh in [0.03, 0.05, 0.08]:
                t_curr = [np.eye(4)] * n_v; fits = []
                target = prep[0].voxel_down_sample(voxel); target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.02, 30))
                for i in range(1, n_v):
                    source = prep[i].voxel_down_sample(voxel)
                    r1 = o3d.pipelines.registration.registration_icp(source, target, thresh, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane())
                    try:
                        r2 = o3d.pipelines.registration.registration_colored_icp(source, target, thresh/2, r1.transformation, o3d.pipelines.registration.TransformationEstimationForColoredICP())
                        t_curr[i], fit = r2.transformation, r2.fitness
                    except: t_curr[i], fit = r1.transformation, r1.fitness
                    fits.append(fit)
                score = np.mean(fits) if fits else 1.0
                if score > best_score:
                    best_score, best_t = score, t_curr
                    m = copy.deepcopy(prep[0])
                    for i in range(1, n_v): m += copy.deepcopy(prep[i]).transform(t_curr[i])
                    best_merged = m
        print(f"  [Stitch] Best Alignment Fitness: {best_score*100:.1f}%")
        return best_merged.voxel_down_sample(0.003), best_t

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
        return color[..., ::-1].copy(), depth
        
    def process_scene_views(self, views: List[Dict[str, Any]], t_cam_from_tcp: np.ndarray, strategy: int = 5) -> Tuple[Any, List[SegmentedObject]]:
        if not self.model: self.load_model()
        all_pcds, view_objs_all = [], []
        for v_idx, view in enumerate(views):
            color, depth, tcp_mat = view["color"], view["depth"], view["tcp_matrix"]
            cam_pose, intr = tcp_mat @ t_cam_from_tcp, view.get("intrinsics", self.intrinsics)
            all_pcds.append((self.pc_processor.filter_pcd(self.pc_processor.create_pcd_from_rgbd(color, depth, intr)), cam_pose))
            res = self.model(color, conf=0.15, verbose=False)
            v_objs = []
            if len(res) > 0 and res[0].masks is not None:
                m, b, n = res[0].masks.data.cpu().numpy(), res[0].boxes, res[0].names
                for i in range(len(m)):
                    mask = cv2.resize(m[i], (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST) if m[i].shape != depth.shape else m[i]
                    od = depth.copy(); od[mask == 0] = 0
                    op = self.pc_processor.filter_pcd(self.pc_processor.create_pcd_from_rgbd(color, od, intr))
                    if len(op.points) > 10: 
                        op.transform(cam_pose)
                        v_objs.append((n[int(b.cls[i])], float(b.conf[i]), op))
            view_objs_all.append(v_objs)
                    
        stitched_pcd, icp_t = self.pc_processor.register_and_stitch(all_pcds, strategy)
        
        # Iterative Search for BEST SEGMENTATION
        best_objs = []; best_score = -1.0
        for eps in [0.015, 0.022, 0.028, 0.035]:
            for min_pts in [10, 25]:
                pts_l, cols_l, conf_l, lab_l = [], [], [], []
                for i, v_objs in enumerate(view_objs_all):
                    for name, conf, op in v_objs:
                        op_c = copy.deepcopy(op).transform(icp_t[i])
                        pts_l.append(np.asarray(op_c.points)); cols_l.append(np.asarray(op_c.colors))
                        conf_l.append(np.full(len(op_c.points), conf)); lab_l.append(np.full(len(op_c.points), name, dtype=object))

                if not pts_l: continue
                pts = np.concatenate(pts_l); cols = np.concatenate(cols_l); confs = np.concatenate(conf_l); labels = np.concatenate(lab_l)
                p = o3d.geometry.PointCloud(); p.points, p.colors = o3d.utility.Vector3dVector(pts), o3d.utility.Vector3dVector(cols)
                idx = np.array(p.cluster_dbscan(eps=eps, min_points=min_pts))
                
                cur = []
                for lbl in range(idx.max() + 1):
                    mask = (idx == lbl); cp, cl, cf = pts[mask], labels[mask], confs[mask]
                    uc = np.unique(cl); scores = {c: np.sum(cf[cl == c]) * np.sum(cl == c) for c in uc}
                    win = max(scores, key=scores.get)
                    res = o3d.geometry.PointCloud(); res.points, res.colors = o3d.utility.Vector3dVector(cp), o3d.utility.Vector3dVector(cols[mask])
                    cur.append(SegmentedObject(len(cur), win, float(np.mean(cf[cl == win])), tuple(cp.mean(axis=0)), res))
                
                # Ultimate Scoring:
                # 1. Favor unique classes (most important)
                # 2. Favor finding exactly 6 objects
                # 3. Penalty for multiple instances of same class
                unique_classes = set([o.class_name for o in cur])
                diversity_score = len(unique_classes) * 20.0
                count_score = 5.0 / (1.0 + abs(6 - len(cur)))
                uniqueness_penalty = (len(cur) - len(unique_classes)) * 2.0
                total_score = diversity_score + count_score - uniqueness_penalty
                
                if total_score > best_score:
                    best_score, best_objs = total_score, cur
                    print(f"  [Optimizer] eps={eps:.3f}, min_pts={min_pts} -> {len(cur)} objs ({len(unique_classes)} unique). Score: {total_score:.2f}")

        return stitched_pcd, best_objs
