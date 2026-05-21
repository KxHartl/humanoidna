from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import open3d as o3d
from core.interfaces import IVisionAdapter
from core.models import SegmentedObject
from core.config import VisionConfig
import cv2
import torch
from ultralytics import YOLO

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

class PointCloudProcessor:
    def __init__(self):
        pass
        
    def create_pcd_from_rgbd(self, color_img: np.ndarray, depth_img: np.ndarray, intrinsics: np.ndarray) -> o3d.geometry.PointCloud:
        height, width = depth_img.shape
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]
        
        o3d_color = o3d.geometry.Image(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB))
        o3d_depth = o3d.geometry.Image(depth_img.astype(np.float32))
        
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, o3d_depth, depth_scale=1.0, depth_trunc=1.5, convert_rgb_to_intensity=False
        )
        
        intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)
        return pcd
        
    def filter_pcd(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        if pcd.is_empty():
            return pcd
        # Voxel Downsample
        pcd_down = pcd.voxel_down_sample(voxel_size=0.005)
        if len(pcd_down.points) < 20:
            return pcd_down
        # Statistical Outlier Removal
        cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd_filtered = pcd_down.select_by_index(ind)
        # PassThrough filter (Z axis thresholding or similar if needed)
        # Let's crop Z > 0.05 min, < 1.0 max relative to cam
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, 0.0), max_bound=(1, 1, 1.2))
        return pcd_filtered.crop(bbox)

    def register_and_stitch(self, pcds: List[Tuple[o3d.geometry.PointCloud, np.ndarray]], strategy: int = 2) -> Tuple[o3d.geometry.PointCloud, List[np.ndarray]]:
        if not pcds:
            return o3d.geometry.PointCloud(), []
            
        merged = o3d.geometry.PointCloud()
        icp_transforms = []
        icp_threshold = 0.08 # 8cm max correspondence distance
        
        import copy
        target_global = copy.deepcopy(pcds[0][0])
        target_global.transform(pcds[0][1])
        merged += target_global
        icp_transforms.append(np.eye(4)) # Anchor view
        
        # Strategies:
        # 0: Align to Anchor (View 0)
        # 1: Align to Accumulated Merged Cloud
        # 2: Align to Previous View (Sequential)
        # 3: Colored ICP Sequential
        
        if strategy == 0:
            target_down = target_global.voxel_down_sample(voxel_size=0.005)
            target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
            
            for i in range(1, len(pcds)):
                source_global = copy.deepcopy(pcds[i][0])
                source_global.transform(pcds[i][1])
                
                source_down = source_global.voxel_down_sample(voxel_size=0.005)
                source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                reg = o3d.pipelines.registration.registration_icp(
                    source_down, target_down, icp_threshold, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                )
                
                # Ocijeni i ispiši
                eval_res = o3d.pipelines.registration.evaluate_registration(source_down, target_down, 0.02, reg.transformation)
                print(f"  [ICP] View {i} -> Anchor: Fitness {eval_res.fitness*100:.2f}%, RMSE {eval_res.inlier_rmse:.4f}m")
                
                icp_transforms.append(reg.transformation)
                source_global.transform(reg.transformation)
                merged += source_global
                
        elif strategy == 1:
            for i in range(1, len(pcds)):
                source_global = copy.deepcopy(pcds[i][0])
                source_global.transform(pcds[i][1])
                
                source_down = source_global.voxel_down_sample(voxel_size=0.005)
                source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                target_down = merged.voxel_down_sample(voxel_size=0.005)
                target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                reg = o3d.pipelines.registration.registration_icp(
                    source_down, target_down, icp_threshold, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                )
                
                eval_res = o3d.pipelines.registration.evaluate_registration(source_down, target_down, 0.02, reg.transformation)
                print(f"  [ICP] View {i} -> Accumulated: Fitness {eval_res.fitness*100:.2f}%, RMSE {eval_res.inlier_rmse:.4f}m")
                
                icp_transforms.append(reg.transformation)
                source_global.transform(reg.transformation)
                merged += source_global
                
        elif strategy == 2:
            prev_global = copy.deepcopy(target_global)
            
            for i in range(1, len(pcds)):
                source_global = copy.deepcopy(pcds[i][0])
                source_global.transform(pcds[i][1]) # initial robot kinematic guess
                
                source_down = source_global.voxel_down_sample(voxel_size=0.005)
                source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                target_down = prev_global.voxel_down_sample(voxel_size=0.005)
                target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                reg = o3d.pipelines.registration.registration_icp(
                    source_down, target_down, icp_threshold, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                )
                
                eval_res = o3d.pipelines.registration.evaluate_registration(source_down, target_down, 0.02, reg.transformation)
                print(f"  [ICP] View {i} -> Previous: Fitness {eval_res.fitness*100:.2f}%, RMSE {eval_res.inlier_rmse:.4f}m")
                
                icp_transforms.append(reg.transformation)
                
                source_global.transform(reg.transformation)
                merged += source_global
                prev_global = copy.deepcopy(source_global)
                
        elif strategy == 3:
            prev_global = copy.deepcopy(target_global)
            
            for i in range(1, len(pcds)):
                source_global = copy.deepcopy(pcds[i][0])
                source_global.transform(pcds[i][1])
                
                source_down = source_global.voxel_down_sample(voxel_size=0.005)
                source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                target_down = prev_global.voxel_down_sample(voxel_size=0.005)
                target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                try:
                    reg = o3d.pipelines.registration.registration_colored_icp(
                        source_down, target_down, icp_threshold, np.eye(4),
                        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=50)
                    )
                    
                    eval_res = o3d.pipelines.registration.evaluate_registration(source_down, target_down, 0.02, reg.transformation)
                    print(f"  [Colored ICP] View {i} -> Previous: Fitness {eval_res.fitness*100:.2f}%, RMSE {eval_res.inlier_rmse:.4f}m")
                    
                    icp_transforms.append(reg.transformation)
                    source_global.transform(reg.transformation)
                    merged += source_global
                    prev_global = copy.deepcopy(source_global)
                except RuntimeError as e:
                    print(f"Colored ICP failed (no correspondences): {e}")
                    icp_transforms.append(np.eye(4))
                    merged += source_global
                    prev_global = copy.deepcopy(source_global)
                
        elif strategy == 4:
            # STRATEGY 4: Pose Graph Optimization (Multi-way registration)
            global_pcds = []
            for i, pcd_tuple in enumerate(pcds):
                pcd_i = copy.deepcopy(pcd_tuple[0])
                pcd_i.transform(pcd_tuple[1]) # Apply initial guess
                global_pcds.append(pcd_i)
            
            down_pcds = []
            for pcd in global_pcds:
                down = pcd.voxel_down_sample(voxel_size=0.005)
                down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                down_pcds.append(down)
                
            pose_graph = o3d.pipelines.registration.PoseGraph()
            odometry = np.eye(4)
            pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
            
            for source_id in range(len(global_pcds)):
                for target_id in range(source_id + 1, len(global_pcds)):
                    source = down_pcds[source_id]
                    target = down_pcds[target_id]
                    
                    reg = o3d.pipelines.registration.registration_icp(
                        source, target, icp_threshold, np.eye(4),
                        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                    )
                    
                    info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                        source, target, icp_threshold, reg.transformation)
                    
                    if target_id == source_id + 1: # odometry case
                        odometry = reg.transformation @ odometry
                        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                        pose_graph.edges.append(
                            o3d.pipelines.registration.PoseGraphEdge(source_id, target_id, reg.transformation, info, uncertain=False))
                    else: # loop closure case
                        pose_graph.edges.append(
                            o3d.pipelines.registration.PoseGraphEdge(source_id, target_id, reg.transformation, info, uncertain=True))

            option = o3d.pipelines.registration.GlobalOptimizationOption(
                max_correspondence_distance=icp_threshold,
                edge_prune_threshold=0.25,
                reference_node=0)
            
            o3d.pipelines.registration.global_optimization(
                pose_graph,
                o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                option)
                
            merged = o3d.geometry.PointCloud()
            icp_transforms = [] # Ovdje moramo prepisati icp_transforms da ih vratimo ispravno
            for point_id in range(len(global_pcds)):
                opt_transform = pose_graph.nodes[point_id].pose
                
                pcd_opt = copy.deepcopy(global_pcds[point_id])
                pcd_opt.transform(opt_transform)
                merged += pcd_opt
                icp_transforms.append(opt_transform)
                
            print(f"  [Pose Graph] Optimizirano {len(global_pcds)} pogleda.")
            
        elif strategy == 5:
            # STRATEGY 5: Smart Fitness Rejection (Anchor-based with Outlier Rejection)
            target_global = copy.deepcopy(pcds[0][0])
            target_global.transform(pcds[0][1])
            merged = target_global
            
            target_down = target_global.voxel_down_sample(voxel_size=0.005)
            target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
            
            icp_transforms = [np.eye(4)]
            FITNESS_THRESHOLD = 0.90 # 90%
            
            for i in range(1, len(pcds)):
                source_global = copy.deepcopy(pcds[i][0])
                source_global.transform(pcds[i][1]) # Inicijalni kinematički guess
                
                source_down = source_global.voxel_down_sample(voxel_size=0.005)
                source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                
                reg = o3d.pipelines.registration.registration_icp(
                    source_down, target_down, icp_threshold, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                )
                
                eval_res = o3d.pipelines.registration.evaluate_registration(source_down, target_down, 0.02, reg.transformation)
                fit_percent = eval_res.fitness * 100
                
                if fit_percent >= FITNESS_THRESHOLD * 100:
                    print(f"  [Smart ICP] View {i} -> Prihvaćen: Fitness {fit_percent:.2f}%, RMSE {eval_res.inlier_rmse:.4f}m")
                    icp_transforms.append(reg.transformation)
                    source_global.transform(reg.transformation)
                    merged += source_global
                else:
                    print(f"  [Smart ICP] View {i} -> ODBIJEN (Outlier): Fitness {fit_percent:.2f}% < {FITNESS_THRESHOLD*100}% prag.")
                    # Vraćamo praznu transformaciju (ili identitet) no pogled ne dodajemo u `merged`
                    icp_transforms.append(np.eye(4))
                    
        else:
            raise ValueError(f"Nepoznata strategija: {strategy}")
            
        merged_down = merged.voxel_down_sample(voxel_size=0.005)
        return merged_down, icp_transforms

class VisionAdapter(IVisionAdapter):
    def __init__(self, config: VisionConfig, intrinsics: np.ndarray):
        self.config = config
        self.intrinsics = intrinsics
        self.model = None
        self.pc_processor = PointCloudProcessor()
        self.rs_pipeline = None
        self.rs_align = None
        self.rs_depth_scale = 1.0
        
    def load_model(self) -> None:
        self.model = YOLO(self.config.model_path)
        
    def setup_realsense(self) -> bool:
        if rs is None:
            print("pyrealsense2 is not installed!")
            return False
            
        try:
            self.rs_pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            profile = self.rs_pipeline.start(config)
            self.rs_align = rs.align(rs.stream.color)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.rs_depth_scale = float(depth_sensor.get_depth_scale())
            
            for _ in range(15): # warmup
                self.rs_pipeline.wait_for_frames()
            return True
        except Exception as e:
            print(f"RealSense setup failed: {e}")
            return False
            
    def stop_realsense(self):
        if self.rs_pipeline:
            self.rs_pipeline.stop()
            self.rs_pipeline = None
            
    def capture_live_view(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self.rs_pipeline:
            raise RuntimeError("RealSense is not initialized. Call setup_realsense() first.")
            
        frames = self.rs_pipeline.wait_for_frames()
        aligned_frames = self.rs_align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        color_bgr = np.asanyarray(color_frame.get_data())
        color_rgb = color_bgr[..., ::-1].copy()
        
        depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_m = depth_raw * self.rs_depth_scale
        
        return color_rgb, depth_m
        
    def process_scene_views(self, views: List[Dict[str, Any]], t_cam_from_tcp: np.ndarray, strategy: int = 2) -> Tuple[Any, List[SegmentedObject]]:
        if not self.model:
            self.load_model()
            
        all_pcds_with_transforms = []
        view_objects = []
        
        for view_idx, view in enumerate(views):
            color = view["color"]
            depth = view["depth"]
            tcp_pose_matrix = view["tcp_matrix"] # 4x4 matrix
            
            # transform: robot_base -> tcp -> camera
            cam_pose_in_base = tcp_pose_matrix @ t_cam_from_tcp
            
            # Use specific intrinsic matrix if available from offline capture
            intr_matrix = view.get("intrinsics")
            if intr_matrix is None:
                intr_matrix = self.intrinsics
            
            # Create full cloud
            pcd = self.pc_processor.create_pcd_from_rgbd(color, depth, intr_matrix)
            pcd_filtered = self.pc_processor.filter_pcd(pcd)
            
            all_pcds_with_transforms.append((pcd_filtered, cam_pose_in_base))
            
            # YOLO Inference
            results = self.model(color, conf=self.config.confidence_threshold, verbose=False)
            current_view_objs = []
            
            if len(results) > 0 and results[0].masks is not None:
                masks = results[0].masks.data.cpu().numpy()
                boxes = results[0].boxes
                names = results[0].names
                
                for i, mask in enumerate(masks):
                    cls_id = int(boxes.cls[i].item())
                    conf = float(boxes.conf[i].item())
                    class_name = names[cls_id]
                    
                    # Resize mask to depth shape if needed
                    if mask.shape != depth.shape:
                        mask = cv2.resize(mask, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST)
                    
                    # Create isolated object point cloud
                    obj_depth = depth.copy()
                    obj_depth[mask == 0] = 0
                    
                    obj_pcd = self.pc_processor.create_pcd_from_rgbd(color, obj_depth, intr_matrix)
                    obj_pcd = self.pc_processor.filter_pcd(obj_pcd)
                    
                    if len(obj_pcd.points) < 50:
                        continue
                        
                    # Transform to robot base
                    obj_pcd.transform(cam_pose_in_base)
                    current_view_objs.append((class_name, conf, obj_pcd))
                    
            view_objects.append(current_view_objs)
                    
        # 1. Run ICP on full point clouds
        stitched_pcd, icp_transforms = self.pc_processor.register_and_stitch(all_pcds_with_transforms, strategy)
        
        # 2. Assemble the filtered objects with the refined ICP transformations
        # Group by class_name so we can find ALL fruit types separately
        from collections import defaultdict
        grouped_pts = defaultdict(list)
        grouped_cols = defaultdict(list)
        grouped_conf = defaultdict(list)
        
        for i in range(len(view_objects)):
            T_icp = icp_transforms[i]
            for class_name, conf, obj_pcd_base in view_objects[i]:
                obj_pcd_base.transform(T_icp)
                grouped_pts[class_name].append(np.asarray(obj_pcd_base.points))
                grouped_cols[class_name].append(np.asarray(obj_pcd_base.colors))
                grouped_conf[class_name].append(conf)
                
        all_objects = []
        if not grouped_pts:
            return stitched_pcd, all_objects
            
        # 3. DBSCAN per fruit type
        eps = 0.05  # Increased from 0.035 to better merge close clusters
        min_points = 50
        instance_counter = 0
        
        pre_merged_objects = []
        for class_name, pts_list in grouped_pts.items():
            class_pts = np.concatenate(pts_list, axis=0)
            class_cols = np.concatenate(grouped_cols[class_name], axis=0)
            avg_conf = float(np.mean(grouped_conf[class_name]))
            
            fruit_pcd = o3d.geometry.PointCloud()
            fruit_pcd.points = o3d.utility.Vector3dVector(class_pts)
            fruit_pcd.colors = o3d.utility.Vector3dVector(class_cols)
            
            labels = np.array(fruit_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
            max_label = labels.max() if len(labels) > 0 else -1
            
            for lbl in range(max_label + 1):
                idx_bool = (labels == lbl)
                if not np.any(idx_bool): continue
                
                inst_pcd = o3d.geometry.PointCloud()
                inst_pcd.points = o3d.utility.Vector3dVector(class_pts[idx_bool])
                inst_pcd.colors = o3d.utility.Vector3dVector(class_cols[idx_bool])
                
                centroid = np.asarray(inst_pcd.points).mean(axis=0)
                
                pre_merged_objects.append(SegmentedObject(
                    instance_id=instance_counter,
                    class_name=class_name,
                    confidence=avg_conf,
                    centroid_robot_base=tuple(centroid),
                    pcd=inst_pcd
                ))
                instance_counter += 1

        # 4. Final Centroid Merge (Consolidate clusters of SAME CLASS that are very close)
        final_objects = []
        merge_dist_thresh = 0.15  # Sufficient to merge duplicate detections of the same fruit
        
        used_indices = set()
        for i in range(len(pre_merged_objects)):
            if i in used_indices: continue
            
            current_obj = pre_merged_objects[i]
            to_merge = [current_obj]
            used_indices.add(i)
            
            for j in range(i + 1, len(pre_merged_objects)):
                if j in used_indices: continue
                next_obj = pre_merged_objects[j]
                
                if current_obj.class_name == next_obj.class_name:
                    dist = np.linalg.norm(np.array(current_obj.centroid_robot_base) - np.array(next_obj.centroid_robot_base))
                    if dist < merge_dist_thresh:
                        to_merge.append(next_obj)
                        used_indices.add(j)
            
            if len(to_merge) > 1:
                # Merge PCDs and re-centroid
                merged_pcd = o3d.geometry.PointCloud()
                all_pts = []
                all_cols = []
                for obj in to_merge:
                    all_pts.append(np.asarray(obj.pcd.points))
                    all_cols.append(np.asarray(obj.pcd.colors))
                
                merged_pcd.points = o3d.utility.Vector3dVector(np.concatenate(all_pts, axis=0))
                merged_pcd.colors = o3d.utility.Vector3dVector(np.concatenate(all_cols, axis=0))
                new_centroid = np.asarray(merged_pcd.points).mean(axis=0)
                
                final_objects.append(SegmentedObject(
                    instance_id=len(final_objects),
                    class_name=current_obj.class_name,
                    confidence=current_obj.confidence,
                    centroid_robot_base=tuple(new_centroid),
                    pcd=merged_pcd
                ))
            else:
                current_obj.instance_id = len(final_objects)
                final_objects.append(current_obj)
            
        return stitched_pcd, final_objects
