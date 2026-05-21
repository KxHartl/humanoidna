from pathlib import Path
from typing import Any, List, Tuple
import numpy as np
import open3d as o3d
import cv2
from ultralytics import YOLO

from dataclasses import dataclass

@dataclass
class ReconstructedInstance:
    instance_id: int
    class_name: str
    confidence: float
    centroid_base_xyz: tuple[float, float, float]
    pcd_base: o3d.geometry.PointCloud

@dataclass
class ReconstructionResult:
    instances: list[ReconstructedInstance]
    scene_pcd_path: str
from io_utils import ensure_dir, load_npy_matrix, load_rgb_image
import json

def _transform_cloud(pcd: o3d.geometry.PointCloud, matrix: np.ndarray) -> o3d.geometry.PointCloud:
    import copy
    cloned = copy.deepcopy(pcd)
    cloned.transform(matrix)
    return cloned

def create_pcd_from_rgbd(color_img: np.ndarray, depth_img: np.ndarray, intrinsics: np.ndarray) -> o3d.geometry.PointCloud:
    height, width = depth_img.shape
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]

    o3d_color = o3d.geometry.Image(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB))
    o3d_depth = o3d.geometry.Image(depth_img.astype(np.float32))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color, o3d_depth, depth_scale=1.0, depth_trunc=1.5, convert_rgb_to_intensity=False
    )
    intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)

def reconstruct_scene_from_pairs(
    pairs: list[list[str]],
    base_tcp_transforms: list[str],
    cfg: Any,
    segment_output_dir: Path | str | None = None,
    reconstruct_output_dir: Path | str | None = None,
    invert_cam_tcp: bool = False
) -> ReconstructionResult:
    
    ensure_dir(segment_output_dir)
    ensure_dir(reconstruct_output_dir)
    print("\n╔══════════════════════════════════════════════════════════════╗")
    print("║     V2 LOGIC: 2D YOLO -> Point-To-Plane ICP -> DBSCAN        ║")
    print("╚══════════════════════════════════════════════════════════════╝", flush=True)

    t_cam_from_tcp = load_npy_matrix(getattr(cfg.paths, "t_cam_from_tcp", None))
    if invert_cam_tcp:
        t_cam_from_tcp = np.linalg.inv(t_cam_from_tcp)
    model = YOLO(str(cfg.paths.segmentation_model))

    # Kontejneri za svaki View
    scene_base_clouds = []
    view_objects = []

    # FAZA 1: 2D YOLO i stvaranje Base Oblaka
    for idx, (rgb_path, pcd_path) in enumerate(pairs):
        view_dir = Path(rgb_path).parent
        color_img = cv2.imread(rgb_path)
        if color_img is None:
            raise FileNotFoundError(f"Ne mogu ucitati sliku: {rgb_path}")
        depth_img = np.load(str(view_dir / "depth_m.npy"))
        with open(str(view_dir / "intrinsics.json"), "r") as f:
            intrinsics = json.load(f)
        if "intrinsic_matrix" in intrinsics:
            intr_matrix = np.array(intrinsics["intrinsic_matrix"])
        else:
            fx = intrinsics["fx"]
            fy = intrinsics["fy"]
            cx = intrinsics["cx"]
            cy = intrinsics["cy"]
            intr_matrix = np.array([
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0]
            ])
        
        # 1.1 Transformacije (Robotički Base -> TCP -> Cam)
        val = base_tcp_transforms[idx]
        if isinstance(val, np.ndarray):
            T_base_tcp = val
        else:
            t_base_tcp_path = Path(val)
            if t_base_tcp_path.exists():
                T_base_tcp = load_npy_matrix(t_base_tcp_path)
            else:
                T_base_tcp = np.eye(4)
        
        T_base_cam = T_base_tcp @ t_cam_from_tcp
        
        # 1.2 Glavni PointCloud Scene za ovaj View
        full_pcd = create_pcd_from_rgbd(color_img, depth_img, intr_matrix)
        # Downsample za ICP i noise filtering
        full_pcd = full_pcd.voxel_down_sample(voxel_size=0.005)
        cl, ind = full_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        full_pcd = full_pcd.select_by_index(ind)
        # Crop
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, 0.0), max_bound=(1, 1, 1.2))
        full_pcd = full_pcd.crop(bbox)

        # Base frame prebačen glavni cloud
        full_pcd_base = _transform_cloud(full_pcd, T_base_cam)
        scene_base_clouds.append(full_pcd_base)

        # 1.3 YOLO objekti - Radimo isključivo na 2D slikama i rešemo Depth maske
        results = model(color_img, conf=cfg.target.confidence_threshold, verbose=False)
        current_view_objs = []
        target_class = getattr(cfg.target, "target_class", "orange")
        print(f"  [DEBUG] View {idx}: target_class='{target_class}', confidence_threshold={cfg.target.confidence_threshold}")
        if len(results) > 0 and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            boxes = results[0].boxes
            names = results[0].names
            print(f"  [DEBUG] Found {len(masks)} total masks in this view.")
            
            for i, mask in enumerate(masks):
                cls_id = int(boxes.cls[i].item())
                conf = float(boxes.conf[i].item())
                class_name = names[cls_id]
                print(f"  [DEBUG] Mask {i}: class='{class_name}', conf={conf:.3f}")

                # Target klase (samo ono sto je trazeno, npr orange)
                if class_name != target_class:
                    print(f"  [DEBUG] Skipping class '{class_name}' because it does not match target_class '{target_class}'")
                    continue

                if mask.shape != depth_img.shape:
                    mask = cv2.resize(mask, (depth_img.shape[1], depth_img.shape[0]), interpolation=cv2.INTER_NEAREST)
                
                obj_depth = depth_img.copy()
                obj_depth[mask == 0] = 0
                
                obj_pcd = create_pcd_from_rgbd(color_img, obj_depth, intr_matrix)
                print(f"  [DEBUG] Created point cloud for '{class_name}' with {len(obj_pcd.points)} points.")
                if len(obj_pcd.points) < 50:
                    print(f"  [DEBUG] Skipping object because points count ({len(obj_pcd.points)}) < 50")
                    continue
                
                # Transform prebačen u Base frame s T_base_cam
                obj_pcd_base = _transform_cloud(obj_pcd, T_base_cam)
                current_view_objs.append((class_name, conf, obj_pcd_base))
        else:
            print("  [DEBUG] No masks detected in this view.")

        view_objects.append(current_view_objs)

    # FAZA 2: ICP po IDENTIČNOJ V2 METODI B
    icp_transforms = [np.eye(4)]
    if len(scene_base_clouds) > 0:
        target_global = _transform_cloud(scene_base_clouds[0], np.eye(4)) # Prvi cloud je na nuli
        
        for i in range(1, len(scene_base_clouds)):
            source_global = scene_base_clouds[i]

            target_down = target_global.voxel_down_sample(voxel_size=0.005)
            source_down = source_global.voxel_down_sample(voxel_size=0.005)
            
            # V2 Normale
            target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
            source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

            # V2 Metoda B: Point-To-Plane (0.08 prag)
            reg_p2l = o3d.pipelines.registration.registration_icp(
                source_down, target_down, 0.08, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
            )
            
            icp_transforms.append(reg_p2l.transformation)

            # += na PointCloud bacao Segmentation fault. Zato zbrajamo rucno.
            new_target = _transform_cloud(source_global, reg_p2l.transformation)
            
            # Spajanje 
            pts1 = np.asarray(target_global.points)
            col1 = np.asarray(target_global.colors)
            pts2 = np.asarray(new_target.points)
            col2 = np.asarray(new_target.colors)
            
            target_global = o3d.geometry.PointCloud()
            target_global.points = o3d.utility.Vector3dVector(np.concatenate([pts1, pts2]))
            target_global.colors = o3d.utility.Vector3dVector(np.concatenate([col1, col2]))

    # FAZA 3: Sastavljanje Objekata i Finalni DBSCAN
    all_instance_points = []
    all_instance_colors = []
    
    for i in range(len(view_objects)):
        T_icp = icp_transforms[i]
        
        for class_name, conf, obj_pcd_base in view_objects[i]:
            # Primjena preklapanja ICP-a na izrezano voće
            final_obj = _transform_cloud(obj_pcd_base, T_icp)
            all_instance_points.append(np.asarray(final_obj.points))
            all_instance_colors.append(np.asarray(final_obj.colors))
            
    if not all_instance_points:
        return ReconstructionResult(instances=[], scene_pcd_path="")

    global_pts = np.concatenate(all_instance_points, axis=0)
    global_cols = np.concatenate(all_instance_colors, axis=0)
    
    global_fruit_pcd = o3d.geometry.PointCloud()
    global_fruit_pcd.points = o3d.utility.Vector3dVector(global_pts)
    global_fruit_pcd.colors = o3d.utility.Vector3dVector(global_cols)
    
    # DBSCAN
    eps = float(getattr(cfg.point_cloud, "cluster_tolerance", 0.035))
    min_points = int(getattr(cfg.point_cloud, "min_cluster_size", 50))
    
    labels = np.array(global_fruit_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    max_label = labels.max() if len(labels) > 0 else -1

    reconstructed_instances = []
    for lbl in range(max_label + 1):
        idx_bool = (labels == lbl)
        if not np.any(idx_bool): continue
        
        inst_pcd = o3d.geometry.PointCloud()
        inst_pcd.points = o3d.utility.Vector3dVector(global_pts[idx_bool])
        inst_pcd.colors = o3d.utility.Vector3dVector(global_cols[idx_bool])

        # Centroid za Pick
        centroid = np.asarray(inst_pcd.points).mean(axis=0)
        
        reconstructed_instances.append(ReconstructedInstance(
            instance_id=lbl,
            class_name=getattr(cfg.target, "target_class", "fruit"),
            confidence=1.0,
            centroid_base_xyz=tuple(centroid),
            pcd_base=inst_pcd
        ))

    # Spremi vizualizaciju globalnog spojenog oblaka
    scene_path = Path(reconstruct_output_dir) / "scene_all_registered.pcd"
    o3d.io.write_point_cloud(str(scene_path), target_global)

    return ReconstructionResult(
        instances=reconstructed_instances,
        scene_pcd_path=str(scene_path)
    )