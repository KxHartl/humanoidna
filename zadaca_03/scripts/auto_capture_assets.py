import os
import sys
import numpy as np
import cv2
import open3d as o3d
from pathlib import Path

# Add src to path to import modules
sys.path.append(r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03\src")
from core.config import AppConfig
from adapters.vision_adapter import PointCloudProcessor, VisionAdapter
from adapters.storage_adapter import StorageAdapter

def save_view(geometries, filename, add_labels=False, objects=None, perspective="camera"):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=1280, height=720)
    
    if perspective != "camera":
        # Koordinatni sustav dodajemo samo za ptičju perspektivu (baza robota)
        # jer za 'camera' perspektivu kamera stoji točno u ishodištu (0,0,0) i nalazi se unutar plave Z osi!
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        vis.add_geometry(coord)
    
    if not isinstance(geometries, list):
        geometries = [geometries]
        
    for geom in geometries:
        vis.add_geometry(geom)
        
    opt = vis.get_render_option()
    opt.point_size = 2.0
    
    ctr = vis.get_view_control()
    if perspective == "camera":
        ctr.set_front([0.0, 0.0, -1.0])
        ctr.set_lookat([0.0, 0.0, 0.5]) 
        ctr.set_up([0.0, -1.0, 0.0])    
        ctr.set_zoom(0.70)
    else:
        ctr.set_front([0.0, 0.0, 1.0])
        ctr.set_lookat([-0.6, 0.1, 0.0])
        ctr.set_up([-1.0, 0.0, 0.0])  
        ctr.set_zoom(0.45)
    
    vis.poll_events()
    vis.update_renderer()
    
    img = vis.capture_screen_float_buffer(True)
    img = (np.asarray(img) * 255).astype(np.uint8)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    if add_labels and objects:
        # Konvertiraj 3D točku u 2D koordinate na ekranu
        params = ctr.convert_to_pinhole_camera_parameters()
        extrinsic = params.extrinsic
        intrinsic = params.intrinsic.intrinsic_matrix
        
        for obj in objects:
            pos_3d = np.array([obj.centroid_robot_base[0], obj.centroid_robot_base[1], obj.centroid_robot_base[2], 1.0])
            pos_cam = extrinsic @ pos_3d
            if pos_cam[2] > 0:
                u = (intrinsic[0, 0] * pos_cam[0] + intrinsic[0, 2] * pos_cam[2]) / pos_cam[2]
                v = (intrinsic[1, 1] * pos_cam[1] + intrinsic[1, 2] * pos_cam[2]) / pos_cam[2]
                
                # Crtaj tekst sa laganom sjenom za bolju vidljivost
                text = obj.class_name
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(img_bgr, text, (int(u)-40, int(v)-40), font, 1.2, (0, 0, 0), 5, cv2.LINE_AA)
                cv2.putText(img_bgr, text, (int(u)-40, int(v)-40), font, 1.2, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imwrite(filename, img_bgr)
    vis.destroy_window()
    print(f"Spremljeno: {filename}")

def remove_close_points(pcd, min_z=0.25):
    if pcd.is_empty(): return pcd
    pts = np.asarray(pcd.points)
    mask = pts[:, 2] > min_z
    return pcd.select_by_index(np.where(mask)[0])

def run_auto_capture(run_dir, out_dir_str):
    print("Učitavam podatke...")
    out_dir = Path(out_dir_str)
    out_dir.mkdir(parents=True, exist_ok=True)
    
    # Change CWD manually because AppConfig assumes local paths
    os.chdir(r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03")
    
    config = AppConfig()
    config.vision.model_path = "data/models/custom_fruit_seg.pt"
    
    try:
        intrinsics = np.load(config.calibration.camera_matrix_path)
    except:
        intrinsics = np.eye(3)
        
    vision = VisionAdapter(config.vision, intrinsics)
    processor = vision.pc_processor
    
    try:
        t_cam_from_tcp = np.load(config.calibration.t_cam_from_tcp_path)
    except:
        t_cam_from_tcp = np.eye(4)
        
    storage = StorageAdapter(config.storage)
    views = storage.load_views_for_offline(run_dir)
    
    step = 1
    
    for i, view in enumerate(views):
        color = view["color"]
        depth = view["depth"]
        intr = view.get("intrinsics")
        if intr is None: intr = intrinsics
        
        original_pcd = processor.create_pcd_from_rgbd(color, depth, intr)
        original_pcd = remove_close_points(original_pcd, min_z=0.3)
        save_view(original_pcd, str(out_dir / f"step_{step:02d}_view_{i+1}_original.png"))
        step += 1
        
        filtered_pcd = processor.filter_pcd(original_pcd)
        save_view(filtered_pcd, str(out_dir / f"step_{step:02d}_view_{i+1}_filtered.png"))
        step += 1
        
        if not vision.model: vision.load_model()
        color_upright = cv2.rotate(color, cv2.ROTATE_180)
        res = vision.model(color_upright, conf=vision.config.confidence_threshold, verbose=False)
        
        segmented_pcd = o3d.geometry.PointCloud()
        if res[0].masks is not None:
            masks = res[0].masks.data.cpu().numpy()
            for mask in masks:
                mask_back = cv2.rotate(mask, cv2.ROTATE_180)
                mask_back = cv2.resize(mask_back, (depth.shape[1], depth.shape[0]))
                masked_depth = depth.copy()
                masked_depth[mask_back < 0.5] = 0
                obj_pcd = processor.create_pcd_from_rgbd(color, masked_depth, intr)
                cl, ind = obj_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                obj_pcd = obj_pcd.select_by_index(ind)
                labels = np.array(obj_pcd.cluster_dbscan(eps=0.03, min_points=50, print_progress=False))
                if len(labels) > 0:
                    max_label = labels.max()
                    if max_label >= 0:
                        counts = np.bincount(labels[labels >= 0])
                        obj_pcd = obj_pcd.select_by_index(np.where(labels == np.argmax(counts))[0])
                        segmented_pcd += obj_pcd
        if len(segmented_pcd.points) == 0: segmented_pcd = filtered_pcd
            
        save_view(segmented_pcd, str(out_dir / f"step_{step:02d}_view_{i+1}_segmented.png"))
        step += 1

    merged_pcd, objects = vision.process_scene_views(views, t_cam_from_tcp, strategy=5)
    save_view(merged_pcd, str(out_dir / f"step_{step:02d}_final_merged.png"), perspective="top-down")
    step += 1
    
    geometries = [merged_pcd]
    for obj in objects:
        bbox = obj.pcd.get_axis_aligned_bounding_box()
        bbox.color = (1, 0, 0)
        geometries.append(bbox)
        
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.015)
        sphere.translate(obj.centroid_robot_base)
        sphere.paint_uniform_color([1, 0, 0])
        geometries.append(sphere)
        
    save_view(geometries, str(out_dir / f"step_{step:02d}_final_classification.png"), add_labels=True, objects=objects, perspective="top-down")
    print(f"Sve je generirano i spremljeno u {out_dir}")

if __name__ == "__main__":
    run_path = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03\data\raw\outputs\run_20260522_194852"
    out_path = str(Path(__file__).parent / "auto_assets")
    if len(sys.argv) > 1:
        run_path = sys.argv[1]
    if len(sys.argv) > 2:
        out_path = sys.argv[2]
    run_auto_capture(run_path, out_path)
