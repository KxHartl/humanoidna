import os
import sys
import copy
import numpy as np
import open3d as o3d
from rich.console import Console
from rich.table import Table

# Dodavanje src_v2 u sys.path
sys.path.append(os.path.join(os.path.dirname(__file__)))

from core.config import AppConfig
from adapters.storage_adapter import StorageAdapter
from adapters.vision_adapter import PointCloudProcessor

console = Console()

def evaluate_registration(source, target, threshold, trans_init):
    # Evaluacija kvalitete registracije
    eval_result = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    return eval_result.fitness, eval_result.inlier_rmse

def test_icp_methods():
    capture_dir = "data/raw/outputs/run_20260521_030300/captures"
    report_dir = "data/report_outputs"
    
    config = AppConfig()
    storage = StorageAdapter(config.storage)
    views = storage.load_views_for_offline(capture_dir)
    t_cam_from_tcp = np.load(config.calibration.t_cam_from_tcp_path)
    intrinsics = np.load(config.calibration.camera_matrix_path)
    processor = PointCloudProcessor()

    # Uzimamo samo prva dva pogleda za testiranje kako bi bilo brzo i jasno
    if len(views) < 2:
        console.print("[red]Nedovoljno pogleda za testiranje![/red]")
        return
        
    pcds = []
    for i in range(2):
        view = views[i]
        cam_pose = view["tcp_matrix"] @ t_cam_from_tcp
        pcd = processor.create_pcd_from_rgbd(view["color"], view["depth"], intrinsics)
        pcd = processor.filter_pcd(pcd)
        pcds.append((pcd, cam_pose))

    # Priprema izvornog i ciljnog oblaka
    target, target_pose = pcds[0]
    source, source_pose = pcds[1]
    
    # Prebacivanje u isti (globalni) koordinatni sustav
    target_global = copy.deepcopy(target)
    target_global.transform(target_pose)
    
    source_global = copy.deepcopy(source)
    source_global.transform(source_pose)

    target_down = target_global.voxel_down_sample(voxel_size=0.005)
    source_down = source_global.voxel_down_sample(voxel_size=0.005)
    
    target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

    init_guess = np.eye(4) # Budući da su već u robot_base preko source_pose/target_pose

    methods = []
    
    # Metoda A: Trenutni Point-to-Plane (Thr=0.02)
    reg_A = o3d.pipelines.registration.registration_icp(
        source_down, target_down, 0.02, init_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )
    fit_A, rmse_A = evaluate_registration(source_down, target_down, 0.02, reg_A.transformation)
    methods.append(("A: Point-to-Plane (Thr 0.02)", reg_A.transformation, fit_A, rmse_A))

    # Metoda B: Point-to-Plane veci prag (Thr=0.08)
    reg_B = o3d.pipelines.registration.registration_icp(
        source_down, target_down, 0.08, init_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )
    fit_B, rmse_B = evaluate_registration(source_down, target_down, 0.02, reg_B.transformation) # ocjenjujemo isto na 2cm
    methods.append(("B: Point-to-Plane (Thr 0.08)", reg_B.transformation, fit_B, rmse_B))

    # Metoda C: Colored ICP
    reg_C = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, 0.08, init_guess,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=50)
    )
    fit_C, rmse_C = evaluate_registration(source_down, target_down, 0.02, reg_C.transformation)
    methods.append(("C: Colored ICP (Thr 0.08)", reg_C.transformation, fit_C, rmse_C))

    # Metoda D: Multi-scale Colored ICP
    voxel_radius = [0.05, 0.02, 0.005]
    max_iter = [50, 30, 14]
    current_transformation = init_guess
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        
        source_s = source_global.voxel_down_sample(radius)
        target_s = target_global.voxel_down_sample(radius)
        source_s.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=30))
        target_s.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=30))
        
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_s, target_s, radius*2, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iter)
        )
        current_transformation = result_icp.transformation

    fit_D, rmse_D = evaluate_registration(source_down, target_down, 0.02, current_transformation)
    methods.append(("D: Multi-scale Colored ICP", current_transformation, fit_D, rmse_D))

    # Print Table
    table = Table(title="ICP Optimizacija Rezultati")
    table.add_column("Metoda", style="cyan")
    table.add_column("Fitness (%)", justify="right", style="green")
    table.add_column("Inlier RMSE (m)", justify="right", style="magenta")

    for name, trans, fit, rmse in methods:
        table.add_row(name, f"{fit*100:.2f}%", f"{rmse:.4f}")
        
        # Save merged PCD for visual check
        test_source = copy.deepcopy(source_global)
        test_source.transform(trans)
        merged = target_global + test_source
        safe_name = name.split(':')[0].strip()
        o3d.io.write_point_cloud(os.path.join(report_dir, f"test_merged_{safe_name}.pcd"), merged)

    console.print(table)
    console.print(f"Testni preklopljeni oblaci spremljeni su u {report_dir}")

if __name__ == "__main__":
    test_icp_methods()
