import open3d as o3d

pcd_path = r"output/run_20260518_172502/reconstruct_scene/target_registered.pcd"

pcd = o3d.io.read_point_cloud(pcd_path)
o3d.visualization.draw_geometries([pcd])