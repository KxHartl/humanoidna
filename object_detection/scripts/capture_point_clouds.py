#!/usr/bin/env python3
"""
Point Cloud Capture Module for Zadaca 3
Snimanje scene s RGB-D kamere ili drugog izvora point cloud-a

Zahtjevi:
- Snimiti najmanje 3 point clouda iste scene
- Različite pozicije kamere/robota
- Eksportirati u .pcd ili .ply format
"""

import os
import numpy as np
from datetime import datetime
from pathlib import Path

# Probaj importati open3d
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: open3d nije instaliran. Instaliraj s: pip install open3d")

# Probaj importati sensor podatke
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    print("Info: python-realsense2 nije instaliran (opcionalno)")


class PointCloudCapture:
    """Klasa za snimanje point cloud-a s RGB-D kamere"""
    
    def __init__(self, output_dir="point_clouds"):
        """
        Inicijalizacija capture modula
        
        Args:
            output_dir (str): Direktorij gdje se sprema point cloud
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        if not HAS_OPEN3D:
            raise ImportError("open3d je obavezan! Instaliraj s: pip install open3d")
    
    def capture_from_realsense(self, duration_seconds=5, resolution=(640, 480)):
        """
        Snimanje s Intel RealSense D455/D435 kamere
        
        Args:
            duration_seconds (int): Trajanje snimanja
            resolution (tuple): Rezolucija (width, height)
            
        Returns:
            open3d.geometry.PointCloud: Snimljeni point cloud
        """
        if not HAS_REALSENSE:
            raise ImportError("python-realsense2 nije instaliran!")
        
        # Konfiguracija pipeline-a
        pipeline = rs.pipeline()
        config = rs.config()
        
        # Postavi rezoluciju
        config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
        
        # Kreni s snimanjem
        profile = pipeline.start(config)
        
        # Dohvati intrinsic parametre kamere
        depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        depth_frame_list = []
        color_frame_list = []
        
        print(f"Snimam scene u trajanju od {duration_seconds} sekundi...")
        start_time = datetime.now()
        
        try:
            while True:
                # Čekaj za framove
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                depth_frame_list.append(np.asanyarray(depth_frame.get_data()))
                color_frame_list.append(np.asanyarray(color_frame.get_data()))
                
                elapsed = (datetime.now() - start_time).total_seconds()
                if elapsed >= duration_seconds:
                    break
        
        finally:
            pipeline.stop()
        
        # Kreiraj point cloud iz akumuliranih framova
        print(f"Kreiram point cloud iz {len(depth_frame_list)} framova...")
        pcd = self._frames_to_pointcloud(depth_frame_list, color_frame_list, depth_intrinsics)
        
        return pcd
    
    def capture_from_synthetic(self):
        """
        Snimanje sintetičkog point cloud-a za testiranje
        
        Returns:
            open3d.geometry.PointCloud: Sintetički point cloud
        """
        print("Kreiram sintetički point cloud za testiranje...")
        
        # Kreiraj sferu i kocku kao test
        mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        mesh_sphere.translate([0.2, 0.3, 1.0])
        
        mesh_cube = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
        mesh_cube.translate([0.0, 0.0, 1.0])
        
        # Konvertiraj mesh u point cloud
        pcd_sphere = mesh_sphere.sample_points_uniformly(number_of_points=5000)
        pcd_cube = mesh_cube.sample_points_uniformly(number_of_points=5000)
        
        # Kombinira point cloud-ove
        pcd_combined = pcd_sphere + pcd_cube
        
        # Dodaj boje
        colors = np.asarray(pcd_combined.colors)
        if len(colors) == 0:
            pcd_combined.paint_uniform_color([0.5, 0.5, 0.5])
        
        return pcd_combined
    
    def _frames_to_pointcloud(self, depth_frames, color_frames, intrinsics):
        """
        Konverzija depth framova u point cloud
        
        Args:
            depth_frames: Lista depth framova
            color_frames: Lista color framova
            intrinsics: Intrinsic parametri kamere
            
        Returns:
            open3d.geometry.PointCloud
        """
        points = []
        colors = []
        
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy
        depth_scale = 0.001  # RealSense standardno koristi mm
        
        for depth_frame, color_frame in zip(depth_frames, color_frames):
            h, w = depth_frame.shape
            
            for v in range(h):
                for u in range(w):
                    depth = depth_frame[v, u] * depth_scale
                    
                    if depth <= 0:
                        continue
                    
                    # Izračunaj 3D koordinate
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    z = depth
                    
                    points.append([x, y, z])
                    
                    # Boja iz color frame-a
                    color_pixel = color_frame[v, u]
                    color = [c / 255.0 for c in color_pixel]
                    colors.append(color)
        
        # Kreiraj point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        if colors:
            pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
        
        return pcd
    
    def save_pointcloud(self, pcd, scene_name, position_index=1, file_format="pcd"):
        """
        Spremi point cloud u datoteku
        
        Args:
            pcd: open3d.geometry.PointCloud
            scene_name (str): Naziv scene (npr. "fruit_scene_01")
            position_index (int): Indeks pozicije (1, 2, 3, ...)
            file_format (str): "pcd" ili "ply"
            
        Returns:
            str: Putanja do sprema datoteke
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{scene_name}_pos{position_index}_{timestamp}.{file_format}"
        filepath = self.output_dir / filename
        
        if file_format == "pcd":
            o3d.io.write_point_cloud(str(filepath), pcd)
        elif file_format == "ply":
            o3d.io.write_point_cloud(str(filepath), pcd)
        else:
            raise ValueError(f"Nepoznat format: {file_format}")
        
        print(f"✓ Point cloud spreman: {filepath}")
        return str(filepath)
    
    def capture_multiple_positions(self, scene_name, num_positions=3, method="synthetic"):
        """
        Snimanje scene iz multiple pozicije
        
        Args:
            scene_name (str): Naziv scene
            num_positions (int): Broj pozicija za snimanje
            method (str): "realsense" ili "synthetic"
            
        Returns:
            list: Lista putanja do snimljenih point cloud-a
        """
        captured_files = []
        
        for pos_idx in range(1, num_positions + 1):
            print(f"\n--- Pozicija {pos_idx}/{num_positions} ---")
            
            if method == "synthetic":
                pcd = self.capture_from_synthetic()
            elif method == "realsense":
                pcd = self.capture_from_realsense(duration_seconds=5)
            else:
                raise ValueError(f"Nepoznata metoda: {method}")
            
            filepath = self.save_pointcloud(pcd, scene_name, pos_idx)
            captured_files.append(filepath)
            
            print(f"Point cloud: {len(np.asarray(pcd.points))} točaka")
            
            # Pauza između pozicija
            if pos_idx < num_positions:
                print("Promenji poziciju kamere/robota za sljedeću snimku...")
                input("Pritisni ENTER kada si spreman/sprema...")
        
        return captured_files


def main():
    """Primjer korištenja"""
    
    # Kreiraj capture modul
    capture = PointCloudCapture(output_dir="point_clouds")
    
    # Snimanje scene (koristi synthetic za testiranje)
    print("=" * 60)
    print("SNIMANJE POINT CLOUD-a")
    print("=" * 60)
    
    # Za realnu snimku s RealSense-om:
    # files = capture.capture_multiple_positions("voce_scene_01", num_positions=3, method="realsense")
    
    # Za testiranje s sintetičkim podacima:
    files = capture.capture_multiple_positions("voce_scene_test", num_positions=3, method="synthetic")
    
    print("\n" + "=" * 60)
    print("SNIMANJE GOTOVO!")
    print("=" * 60)
    print(f"Snimljeni fajlovi:")
    for f in files:
        print(f"  - {f}")


if __name__ == "__main__":
    main()
