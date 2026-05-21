#!/usr/bin/env python3
"""
Object Detection & Localization Module za Zadaca 3
Detekcija i lokalizacija specifičnog objekta (voća) u sceni

Koraci:
1. Segmentacija zadanog objekta iz scene
2. Pronalaženje centra masa (3D centroid)
3. 3D lokalizacija u kamerom sustavu
4. Transformacija u robot bazni sustav
"""

import numpy as np
from pathlib import Path
from typing import Tuple, Optional

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class ObjectDetectionAndLocalization:
    """Klasa za detekciju i lokalizaciju objekta u 3D sceni"""
    
    def __init__(self, yolo_model_path: Optional[str] = None):
        """
        Inicijalizacija
        
        Args:
            yolo_model_path (str): Putanja do YOLO modela
        """
        if not HAS_OPEN3D:
            raise ImportError("open3d je obavezan!")
        
        self.yolo_model = None
        if yolo_model_path and HAS_YOLO:
            print(f"Učitavam YOLO model: {yolo_model_path}")
            self.yolo_model = YOLO(yolo_model_path)
    
    # ============================================================
    # 1. SEGMENTACIJA OBJEKTA
    # ============================================================
    
    def segment_by_color(self, pcd, target_color_hsv, tolerance=30):
        """
        Segmentacija objekta po boji (HSV prostor)
        
        Args:
            pcd: open3d.geometry.PointCloud
            target_color_hsv: Ciljna HSV boja (h, s, v)
            tolerance (int): Tolerancija za H, S, V kanale
            
        Returns:
            open3d.geometry.PointCloud: Segmentirani cloud
        """
        print(f"Segmentacija po boji: HSV {target_color_hsv}...")
        
        if not pcd.has_colors():
            print("Warning: Point cloud nema boja!")
            return pcd
        
        colors_rgb = np.asarray(pcd.colors)  # 0-1 raspon
        colors_bgr = colors_rgb[:, [2, 1, 0]]  # RGB -> BGR za OpenCV
        colors_256 = (colors_bgr * 255).astype(np.uint8)
        
        # Konverzija RGB -> HSV
        # OpenCV ima specifičan format za batch konverziju
        colors_hsv = np.zeros_like(colors_256)
        for i, bgr in enumerate(colors_256):
            img_pixel = np.array([[[bgr[0], bgr[1], bgr[2]]]], dtype=np.uint8)
            hsv_pixel = cv2.cvtColor(img_pixel, cv2.COLOR_BGR2HSV)[0, 0]
            colors_hsv[i] = hsv_pixel
        
        # Kreiraj masku za targetnu boju
        h_target, s_target, v_target = target_color_hsv
        
        h_mask = np.abs(colors_hsv[:, 0].astype(float) - h_target) < tolerance
        s_mask = (colors_hsv[:, 1] > s_target - tolerance) & (colors_hsv[:, 1] < s_target + tolerance)
        v_mask = (colors_hsv[:, 2] > v_target - tolerance) & (colors_hsv[:, 2] < v_target + tolerance)
        
        mask = h_mask & s_mask & v_mask
        
        # Ekstraktuj segmentirani cloud
        points = np.asarray(pcd.points)
        pcd_segmented = o3d.geometry.PointCloud()
        pcd_segmented.points = o3d.utility.Vector3dVector(points[mask])
        
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            pcd_segmented.colors = o3d.utility.Vector3dVector(colors[mask])
        
        print(f"  Segmentovano: {len(np.asarray(pcd_segmented.points))} točaka")
        
        return pcd_segmented
    
    def segment_by_size(self, clusters, min_points=50, max_points=50000):
        """
        Segmentacija po veličini - filtrira clustere po broju točaka
        
        Args:
            clusters: Lista clustera (point cloudova)
            min_points (int): Minimalan broj točaka
            max_points (int): Maksimalan broj točaka
            
        Returns:
            List: Filtrirani clusteri
        """
        print(f"Filtriranje po veličini ({min_points}-{max_points} točaka)...")
        
        filtered = []
        for cluster in clusters:
            num_points = len(np.asarray(cluster.points))
            if min_points <= num_points <= max_points:
                filtered.append(cluster)
        
        print(f"  Preostalo: {len(filtered)} od {len(clusters)} clustera")
        
        return filtered
    
    def segment_by_shape(self, pcd, shape_type='sphere', radius_range=(0.05, 0.15)):
        """
        Segmentacija po obliku (sferast objekt)
        
        Args:
            pcd: open3d.geometry.PointCloud
            shape_type (str): 'sphere' ili drugi
            radius_range (Tuple): Min i max radijus
            
        Returns:
            open3d.geometry.PointCloud: Segmentirani cloud
        """
        print(f"Segmentacija po obliku: {shape_type}...")
        
        # Izračunaj centroid
        centroid = np.asarray(pcd.get_center())
        
        # Za sferičan oblik - koristi distancu od centroida
        points = np.asarray(pcd.points)
        distances = np.linalg.norm(points - centroid, axis=1)
        
        min_r, max_r = radius_range
        mask = (distances >= min_r) & (distances <= max_r)
        
        pcd_segmented = o3d.geometry.PointCloud()
        pcd_segmented.points = o3d.utility.Vector3dVector(points[mask])
        
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            pcd_segmented.colors = o3d.utility.Vector3dVector(colors[mask])
        
        print(f"  Segmentovano: {len(np.asarray(pcd_segmented.points))} točaka")
        
        return pcd_segmented
    
    # ============================================================
    # 2. PRONALAŽENJE CENTRA MASA
    # ============================================================
    
    def find_centroid(self, pcd) -> np.ndarray:
        """
        Pronalaženje centra masa (centroida) point clouda
        
        Args:
            pcd: open3d.geometry.PointCloud
            
        Returns:
            np.ndarray: 3D koordinate centroida (x, y, z)
        """
        centroid = np.asarray(pcd.get_center())
        print(f"Centroid pronađen: ({centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f})")
        return centroid
    
    def find_bounding_box(self, pcd) -> Tuple[np.ndarray, np.ndarray]:
        """
        Pronalaženje bounding box-a
        
        Args:
            pcd: open3d.geometry.PointCloud
            
        Returns:
            Tuple: (min_bound, max_bound)
        """
        points = np.asarray(pcd.points)
        min_bound = np.min(points, axis=0)
        max_bound = np.max(points, axis=0)
        
        print(f"Bounding box:")
        print(f"  Min: ({min_bound[0]:.4f}, {min_bound[1]:.4f}, {min_bound[2]:.4f})")
        print(f"  Max: ({max_bound[0]:.4f}, {max_bound[1]:.4f}, {max_bound[2]:.4f})")
        
        return min_bound, max_bound
    
    def compute_principal_axes(self, pcd):
        """
        Izračunavanje glavnih osa objekta (PCA)
        
        Args:
            pcd: open3d.geometry.PointCloud
            
        Returns:
            Tuple: (centroid, eigenvalues, eigenvectors)
        """
        points = np.asarray(pcd.points)
        centroid = np.mean(points, axis=0)
        
        # Centrira točke
        points_centered = points - centroid
        
        # Kovarijanica matrica
        cov_matrix = np.cov(points_centered.T)
        
        # Eigenvalues i eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        print("Glavne ose:")
        for i, (ev, eiv) in enumerate(zip(eigenvalues, eigenvectors.T)):
            print(f"  Os {i+1}: eigenvalue={ev:.6f}, direction={eiv}")
        
        return centroid, eigenvalues, eigenvectors
    
    # ============================================================
    # 3. 3D LOKALIZACIJA U KAMEROM SUSTAVU
    # ============================================================
    
    def localize_in_camera_frame(self, object_pcd, camera_pose=None) -> np.ndarray:
        """
        3D lokalizacija objekta u kamerom koordinatnom sustavu
        
        Args:
            object_pcd: open3d.geometry.PointCloud (segmentirani objekt)
            camera_pose: Pose kamere u world sustavu (opcionalno)
            
        Returns:
            np.ndarray: 3D koordinate objekta u kamerom sustavu
        """
        print("\n3D Lokalizacija u kamerom sustavu...")
        
        # Centroid objekta
        centroid_object = self.find_centroid(object_pcd)
        
        # Ako nema camera pose, pretpostavi da je kamera u ishodištu
        if camera_pose is None:
            print("  Kamera u ishodištu - centroid je ispravan")
            return centroid_object
        
        # Transformacija točke iz world u camera sustav
        # P_camera = R^T * (P_world - t)
        # ili P_camera = R^T * P_world + (-R^T * t)
        
        R = camera_pose[:3, :3]
        t = camera_pose[:3, 3]
        
        # Inverzna transformacija (world -> camera)
        R_inv = R.T
        t_inv = -R_inv @ t
        
        centroid_camera = R_inv @ centroid_object + t_inv
        
        print(f"  Centroid u camera sustavu:")
        print(f"    ({centroid_camera[0]:.4f}, {centroid_camera[1]:.4f}, {centroid_camera[2]:.4f})")
        
        return centroid_camera
    
    # ============================================================
    # 4. TRANSFORMACIJA U ROBOT BAZNI SUSTAV
    # ============================================================
    
    def transform_to_robot_frame(self, object_coordinates_camera, 
                                camera_to_robot_transform) -> np.ndarray:
        """
        Transformacija 3D koordinata iz camera u robot bazni sustav
        
        Args:
            object_coordinates_camera (np.ndarray): 3D točka u kamerom sustavu
            camera_to_robot_transform (np.ndarray): 4x4 transformacijska matrica
                                                   (camera -> robot)
            
        Returns:
            np.ndarray: 3D koordinate u robot baznom sustavu
        """
        print("\nTransformacija u robot bazni sustav...")
        
        # Homogena koordinata
        point_homogeneous = np.array([
            object_coordinates_camera[0],
            object_coordinates_camera[1],
            object_coordinates_camera[2],
            1.0
        ])
        
        # Primijeni transformaciju
        point_robot = camera_to_robot_transform @ point_homogeneous
        
        # Vrati u 3D koordinate
        result = point_robot[:3] / point_robot[3]
        
        print(f"  Centroid u robot sustavu:")
        print(f"    ({result[0]:.4f}, {result[1]:.4f}, {result[2]:.4f})")
        
        return result
    
    # ============================================================
    # HELPER METODE ZA TRANSFORMACIJE
    # ============================================================
    
    @staticmethod
    def create_transformation_matrix(translation: np.ndarray, 
                                    rotation_matrix: np.ndarray) -> np.ndarray:
        """
        Kreiraj 4x4 transformacijsku matricu
        
        Args:
            translation (np.ndarray): 3D vektor translacije
            rotation_matrix (np.ndarray): 3x3 rotacijska matrica
            
        Returns:
            np.ndarray: 4x4 transformacijska matrica
        """
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T
    
    @staticmethod
    def create_rotation_matrix_x(angle_rad: float) -> np.ndarray:
        """Rotacija oko X ose"""
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])
    
    @staticmethod
    def create_rotation_matrix_y(angle_rad: float) -> np.ndarray:
        """Rotacija oko Y ose"""
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])
    
    @staticmethod
    def create_rotation_matrix_z(angle_rad: float) -> np.ndarray:
        """Rotacija oko Z ose"""
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
    
    # ============================================================
    # KOMPLETAN PIPELINE
    # ============================================================
    
    def detect_and_localize_object(self, 
                                   pcd: 'o3d.geometry.PointCloud',
                                   object_type: str = 'red_apple',
                                   camera_to_robot_transform: Optional[np.ndarray] = None) -> dict:
        """
        Kompletan pipeline detekcije i lokalizacije
        
        Args:
            pcd: Point cloud scene
            object_type (str): Tip objekta koji trebam detektirati
            camera_to_robot_transform: Transformacijska matrica
            
        Returns:
            dict: Rezultati (centroid_camera, centroid_robot, itd.)
        """
        
        print("\n" + "="*60)
        print(f"DETEKCIJA I LOKALIZACIJA: {object_type}")
        print("="*60)
        
        results = {
            'object_type': object_type,
            'centroid_camera': None,
            'centroid_robot': None,
            'bounding_box': None,
            'principal_axes': None,
        }
        
        # 1. Segmentacija objekta
        print("\n1. SEGMENTACIJA")
        print("-" * 40)
        
        # Definiraj HSV boja za različite voće
        color_ranges = {
            'red_apple': (0, 150, 150),  # H, S, V
            'yellow_banana': (25, 200, 200),
            'orange': (15, 200, 180),
            'green_lime': (80, 150, 150),
        }
        
        if object_type in color_ranges:
            object_pcd = self.segment_by_color(pcd, color_ranges[object_type])
        else:
            print(f"Warning: Nepoznat tip objekta {object_type}")
            object_pcd = pcd
        
        # 2. Pronalaženje centra
        print("\n2. PRONALAŽENJE CENTRA")
        print("-" * 40)
        
        centroid_camera = self.find_centroid(object_pcd)
        min_bound, max_bound = self.find_bounding_box(object_pcd)
        centroid, eigenvalues, eigenvectors = self.compute_principal_axes(object_pcd)
        
        results['centroid_camera'] = centroid_camera
        results['bounding_box'] = (min_bound, max_bound)
        results['principal_axes'] = (eigenvalues, eigenvectors)
        
        # 3. Transformacija u robot sustav
        print("\n3. TRANSFORMACIJA U ROBOT SUSTAV")
        print("-" * 40)
        
        if camera_to_robot_transform is not None:
            centroid_robot = self.transform_to_robot_frame(
                centroid_camera,
                camera_to_robot_transform
            )
            results['centroid_robot'] = centroid_robot
        else:
            print("  Upozorenje: Nema transformacijske matrice")
            results['centroid_robot'] = centroid_camera
        
        # Ispis rezultata
        print("\n" + "="*60)
        print("REZULTATI")
        print("="*60)
        print(f"Objekat: {object_type}")
        print(f"Centroid (camera): {centroid_camera}")
        print(f"Centroid (robot): {results['centroid_robot']}")
        
        return results


def main():
    """Primjer korištenja"""
    print("Object Detection & Localization - Primjer")


if __name__ == "__main__":
    main()
