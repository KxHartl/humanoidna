import os
import glob
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def render_pcd_to_image(pcd_path, out_path, title):
    if not os.path.exists(pcd_path):
        return False
        
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return False
        
    colors = np.asarray(pcd.colors) if pcd.has_colors() else 'blue'
    
    # 2D projiciranje (tlocrt, x i y os)
    plt.figure(figsize=(6, 6))
    if isinstance(colors, str):
        plt.scatter(points[:, 0], points[:, 1], s=1, c=colors, alpha=0.5)
    else:
        plt.scatter(points[:, 0], points[:, 1], s=1, c=colors)
        
    plt.title(title)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    return True

def generate_report_images():
    report_dir = "data/report_outputs"
    img_dir = "data/for_report/pointclouds/images"
    os.makedirs(img_dir, exist_ok=True)
    
    # Render view 0 (Original, Filtered, Segmented)
    render_pcd_to_image(f"{report_dir}/view_0_01_original.pcd", f"{img_dir}/v0_original.png", "Pogled 0: Originalni")
    render_pcd_to_image(f"{report_dir}/view_0_02_filtered.pcd", f"{img_dir}/v0_filtered.png", "Pogled 0: Filtrirani")
    
    # Segmentirani (bilo koji)
    seg_files = glob.glob(f"{report_dir}/view_0_03_segmented*.pcd")
    if seg_files:
        render_pcd_to_image(seg_files[0], f"{img_dir}/v0_segmented.png", "Pogled 0: Segmentirani objekt")
        
    # Render view 1 (Original, Filtered, Segmented)
    render_pcd_to_image(f"{report_dir}/view_1_01_original.pcd", f"{img_dir}/v1_original.png", "Pogled 1: Originalni")
    render_pcd_to_image(f"{report_dir}/view_1_02_filtered.pcd", f"{img_dir}/v1_filtered.png", "Pogled 1: Filtrirani")
    seg_files = glob.glob(f"{report_dir}/view_1_03_segmented*.pcd")
    if seg_files:
        render_pcd_to_image(seg_files[0], f"{img_dir}/v1_segmented.png", "Pogled 1: Segmentirani objekt")
        
    # Render view 2
    render_pcd_to_image(f"{report_dir}/view_2_01_original.pcd", f"{img_dir}/v2_original.png", "Pogled 2: Originalni")
    render_pcd_to_image(f"{report_dir}/view_2_02_filtered.pcd", f"{img_dir}/v2_filtered.png", "Pogled 2: Filtrirani")
    seg_files = glob.glob(f"{report_dir}/view_2_03_segmented*.pcd")
    if seg_files:
        render_pcd_to_image(seg_files[0], f"{img_dir}/v2_segmented.png", "Pogled 2: Segmentirani objekt")
        
    # Konačni
    render_pcd_to_image(f"{report_dir}/04_final_merged_point_cloud.pcd", f"{img_dir}/final_merged.png", "Konačni registrirani oblak")
    print(f"Images generated in {img_dir}")

if __name__ == "__main__":
    generate_report_images()
