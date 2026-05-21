import re
import os

with open("d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_03/src/02_fruit_pick_and_place/reconstruct_scene.py", "r", encoding="utf-8") as f:
    code = f.read()

new_reconstruct = """
def _save_pcd_and_image(pcd: o3d.geometry.PointCloud, name: str, pc_dir: Path, pic_dir: Path):
    if pcd.is_empty():
        return
    # Save pcd
    pcd_path = pc_dir / f"{name}.pcd"
    o3d.io.write_point_cloud(str(pcd_path), pcd)
    
    # Save screenshot
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False) # Try invisible, if not supported it might flash
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    pic_path = pic_dir / f"{name}.png"
    vis.capture_screen_image(str(pic_path))
    vis.destroy_window()

def reconstruct_scene_from_results(
    segment_results: Iterable[SegmentationResult],
    base_tcp_transforms: Iterable[np.ndarray],
    cfg: Any,
    output_dir: str | Path | None = None,
    invert_cam_tcp: bool = False,
) -> ReconstructionResult:
    segment_results = list(segment_results)
    base_tcp_transforms = [np.asarray(T, dtype=np.float64) for T in base_tcp_transforms]

    if len(segment_results) == 0:
        raise ValueError("segment_results je prazan.")
    if len(segment_results) != len(base_tcp_transforms):
        raise ValueError("Broj segment_results i broj base_tcp_transforms mora biti isti.")

    output_dir = Path(output_dir) if output_dir is not None else Path(cfg.paths.output_dir) / "reconstruct_scene"
    ensure_dir(output_dir)
    
    report_pc_dir = ensure_dir(Path(cfg.paths.output_dir).parent.parent / "for_report" / "pointclouds")
    report_pic_dir = ensure_dir(Path(cfg.paths.output_dir).parent.parent / "for_report" / "pictures")

    T_tcp_cam = _resolve_tcp_from_cam(cfg, invert_cam_tcp=invert_cam_tcp)
    
    # Metoda B: Point-to-Plane ICP s prosirenim pragom od 8 cm
    icp_threshold = 0.08
    voxel_size = getattr(cfg.point_cloud, "voxel_size", 0.001)

    full_base_clouds = []
    initial_T_base_cam = []
    
    for idx, (seg_result, T_base_tcp) in enumerate(zip(segment_results, base_tcp_transforms)):
        T_base_cam = T_base_tcp @ T_tcp_cam
        initial_T_base_cam.append(T_base_cam)
        
        # Original PC
        pcd_cam = load_point_cloud(seg_result.pcd_path)
        _save_pcd_and_image(pcd_cam, f"view_{idx:02d}_01_original", report_pc_dir, report_pic_dir)
        
        # Filtered PC
        pcd_filtered = _voxel_downsample_if_needed(pcd_cam, voxel_size)
        pcd_filtered = _light_post_merge_clean(pcd_filtered, cfg)
        _save_pcd_and_image(pcd_filtered, f"view_{idx:02d}_02_filtered", report_pc_dir, report_pic_dir)
        
        # Segmented PC (best instance)
        if seg_result.instances:
            # save the first one or best one
            inst = seg_result.instances[0]
            if inst.segmented_pcd_path:
                seg_pcd = load_point_cloud(inst.segmented_pcd_path)
                _save_pcd_and_image(seg_pcd, f"view_{idx:02d}_03_segmented", report_pc_dir, report_pic_dir)

        # For ICP, we should use the full cloud (with table) to constrain Z/roll/pitch.
        # We also use voxel_size=0.005 and normal radius=0.02 as in the test script Option B.
        pcd_icp = _voxel_downsample_if_needed(pcd_cam, 0.005)
        
        # Crop the point cloud in camera frame (Z > 0, Z < 1.2) exactly like in the test script
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, 0.0), max_bound=(1, 1, 1.2))
        pcd_icp = pcd_icp.crop(bbox)
        
        pcd_icp = _light_post_merge_clean(pcd_icp, cfg)
        pcd_base = _transform_cloud(pcd_icp, T_base_cam)
        pcd_base.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        full_base_clouds.append(pcd_base)

    icp_transforms = []
    icp_transforms.append(np.eye(4))

    for i in range(1, len(full_base_clouds)):
        source = full_base_clouds[i]
        target = full_base_clouds[0]
        
        reg_p2l = o3d.pipelines.registration.registration_icp(
            source, target, icp_threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )
        icp_transforms.append(reg_p2l.transformation)
        print(f"ICP view {i} to 0 fitness: {reg_p2l.fitness}, RMSE: {reg_p2l.inlier_rmse}")

    all_transformed_points = []
    all_transformed_colors = []
    
    # Global merged of full scene for report
    scene_points = []
    scene_colors = []

    for idx, (seg_result, T_base_tcp) in enumerate(zip(segment_results, base_tcp_transforms)):
        T_base_cam = initial_T_base_cam[idx]
        T_icp = icp_transforms[idx]
        T_final = T_icp @ T_base_cam
        
        # apply to full cloud for report
        full_pcd = _transform_cloud(load_point_cloud(seg_result.pcd_path), T_final)
        scene_points.append(np.asarray(full_pcd.points))
        scene_colors.append(np.asarray(full_pcd.colors))
        
        for inst in seg_result.instances:
            if not inst.segmented_pcd_path:
                continue
            
            pcd_cam = load_point_cloud(inst.segmented_pcd_path)
            if pcd_cam.is_empty():
                continue
                
            pcd_base = _transform_cloud(pcd_cam, T_final)
            all_transformed_points.append(np.asarray(pcd_base.points))
            all_transformed_colors.append(np.asarray(pcd_base.colors))
            
    # Save registered full scene
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(np.concatenate(scene_points, axis=0))
    scene_pcd.colors = o3d.utility.Vector3dVector(np.concatenate(scene_colors, axis=0))
    scene_pcd = _voxel_downsample_if_needed(scene_pcd, voxel_size)
    _save_pcd_and_image(scene_pcd, "scene_all_registered", report_pc_dir, report_pic_dir)

    if not all_transformed_points:
        raise RuntimeError("Nema detektiranih instanci za rekonstrukciju.")

    global_pcd = o3d.geometry.PointCloud()
    global_pcd.points = o3d.utility.Vector3dVector(np.concatenate(all_transformed_points, axis=0))
    global_pcd.colors = o3d.utility.Vector3dVector(np.concatenate(all_transformed_colors, axis=0))
    
    global_pcd = _light_post_merge_clean(global_pcd, cfg)
    
    global_pcd_path = output_dir / "global_merged.pcd"
    save_point_cloud(global_pcd_path, global_pcd)

    # DBSCAN Clustering za razdvajanje pojedinacnih vocaka
    eps = float(getattr(cfg.point_cloud, "cluster_tolerance", 0.02))
    min_points = int(getattr(cfg.point_cloud, "min_cluster_size", 50))
    
    labels = np.array(global_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    max_label = labels.max() if len(labels) > 0 else -1
    
    reconstructed_instances = []
    
    for i in range(max_label + 1):
        indices = np.where(labels == i)[0]
        cluster_pcd = global_pcd.select_by_index(indices)
        
        # Clean cluster
        cluster_pcd = _light_post_merge_clean(cluster_pcd, cfg)
        if cluster_pcd.is_empty():
            continue
            
        centroid = np.mean(np.asarray(cluster_pcd.points), axis=0)
        
        pcd_path = output_dir / f"instance_{i:02d}.pcd"
        save_point_cloud(pcd_path, cluster_pcd)
        
        reconstructed_instances.append(
            ReconstructedInstance(
                instance_id=i,
                centroid_base_xyz=centroid.tolist(),
                pcd_path=str(pcd_path),
                num_points=len(cluster_pcd.points)
            )
        )

    result = ReconstructionResult(
        target_class=cfg.target.target_class,
        views_used=len(segment_results),
        instances=reconstructed_instances,
        global_pcd_path=str(global_pcd_path)
    )

    save_json(output_dir / "reconstruction_result.json", asdict(result))
    return result
"""

# Replace the function `reconstruct_scene_from_results`
start_idx = code.find("def reconstruct_scene_from_results(")
end_idx = code.find("def reconstruct_scene_from_pairs(")

new_code = code[:start_idx] + new_reconstruct + "\n\n" + code[end_idx:]

with open("d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_03/src/02_fruit_pick_and_place/reconstruct_scene.py", "w", encoding="utf-8") as f:
    f.write(new_code)
print("Updated reconstruct_scene.py")
