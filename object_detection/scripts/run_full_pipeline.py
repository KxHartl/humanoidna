#!/usr/bin/env python3
"""
Integration & Test Script za Zadaca 3
Integrira sve module za kompletan pipeline:
1. Snimanje point cloudova
2. Obrada point cloudova
3. Detekcija i lokalizacija objekta
4. Transformacija u robot sustav
"""

import sys
from pathlib import Path

# Dodaj scripts folder u path
scripts_path = Path(__file__).parent
sys.path.insert(0, str(scripts_path))

try:
    import open3d as o3d
    import numpy as np
except ImportError as e:
    print(f"Error: Nedostaju dependencije - {e}")
    print("Instaliraj s: pip install -r requirements.txt")
    sys.exit(1)

from capture_point_clouds import PointCloudCapture
from pcl_processing import PointCloudProcessor
from object_detection_3d import ObjectDetectionAndLocalization


class TaskaPipeline:
    """Klasa koja integrira sve korake za zadacu 3"""
    
    def __init__(self, output_dir="output"):
        """
        Inicijalizacija pipeline-a
        
        Args:
            output_dir (str): Direktorij za output rezultate
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.capture = PointCloudCapture(output_dir="point_clouds")
        self.processor = PointCloudProcessor()
        self.detector = ObjectDetectionAndLocalization()
        
        self.raw_point_clouds = []
        self.processed_point_cloud = None
        self.clusters = []
    
    def step1_capture_point_clouds(self, scene_name="voce_scene", num_positions=3, method="synthetic"):
        """
        KORAK 1: Snimanje point cloudova
        
        Args:
            scene_name (str): Naziv scene
            num_positions (int): Broj pozicija
            method (str): "synthetic" ili "realsense"
        """
        print("\n" + "="*70)
        print("KORAK 1: SNIMANJE POINT CLOUDOVA")
        print("="*70)
        
        files = self.capture.capture_multiple_positions(
            scene_name,
            num_positions=num_positions,
            method=method
        )
        
        # Učitaj point cloudove
        self.raw_point_clouds = []
        for filepath in files:
            pcd = o3d.io.read_point_cloud(filepath)
            self.raw_point_clouds.append(pcd)
            print(f"Učitao: {filepath} ({len(np.asarray(pcd.points))} točaka)")
        
        return files
    
    def step2_process_point_clouds(self):
        """
        KORAK 2: Obrada individualnih point cloudova
        """
        print("\n" + "="*70)
        print("KORAK 2: OBRADA INDIVIDUALNIH POINT CLOUDOVA")
        print("="*70)
        
        processed_clouds = []
        
        for i, pcd in enumerate(self.raw_point_clouds):
            print(f"\nObrada clouda {i+1}/{len(self.raw_point_clouds)}...")
            
            # Passthrough filtering
            pcd_filtered = self.processor.passthrough_filter(
                pcd,
                axis='z',
                min_val=0.0,
                max_val=2.5
            )
            
            # Statistical Outlier Removal
            pcd_cleaned = self.processor.statistical_outlier_removal(
                pcd_filtered,
                nb_neighbors=20,
                std_ratio=2.0
            )
            
            # Voxel Grid Downsampling
            pcd_downsampled = self.processor.voxel_grid_downsampling(
                pcd_cleaned,
                voxel_size=0.005
            )
            
            processed_clouds.append(pcd_downsampled)
        
        self.processed_point_clouds = processed_clouds
        return processed_clouds
    
    def step3_register_and_merge(self):
        """
        KORAK 3: Registracija i spajanje point cloudova
        """
        print("\n" + "="*70)
        print("KORAK 3: REGISTRACIJA I SPAJANJE POINT CLOUDOVA")
        print("="*70)
        
        self.processed_point_cloud = self.processor.register_and_merge_clouds(
            self.processed_point_clouds,
            voxel_size=0.01
        )
        
        # Spremi konačni cloud
        output_path = self.output_dir / "final_merged_point_cloud.pcd"
        o3d.io.write_point_cloud(str(output_path), self.processed_point_cloud)
        print(f"\n✓ Konačan cloud spreman: {output_path}")
        
        return self.processed_point_cloud
    
    def step4_cluster_objects(self):
        """
        KORAK 4: Segmentacija objekata (clustering)
        """
        print("\n" + "="*70)
        print("KORAK 4: SEGMENTACIJA OBJEKATA")
        print("="*70)
        
        # Euclidean clustering
        labels, num_clusters = self.processor.euclidean_clustering(
            self.processed_point_cloud,
            eps=0.05,
            min_points=10
        )
        
        # Ekstraktuj clustere
        self.clusters = self.processor.extract_clusters(
            self.processed_point_cloud,
            labels,
            num_clusters
        )
        
        return self.clusters
    
    def step5_detect_target_object(self, object_type="red_apple", 
                                   camera_to_robot_transform=None):
        """
        KORAK 5: Detekcija i lokalizacija ciljnog objekta
        
        Args:
            object_type (str): Tip objekta koji trebam detektirati
            camera_to_robot_transform: Transformacijska matrica (4x4)
        """
        print("\n" + "="*70)
        print("KORAK 5: DETEKCIJA I LOKALIZACIJA OBJEKTA")
        print("="*70)
        
        if camera_to_robot_transform is None:
            # Koristi default transformaciju (identiteta)
            camera_to_robot_transform = np.eye(4)
            print("\nUpozorenje: Koristi default transformaciju (kamera = robot)")
        
        # Detektiranje
        results = self.detector.detect_and_localize_object(
            self.processed_point_cloud,
            object_type=object_type,
            camera_to_robot_transform=camera_to_robot_transform
        )
        
        return results
    
    def visualize_results(self):
        """
        Vizualizacija rezultata
        """
        print("\n" + "="*70)
        print("VIZUALIZACIJA REZULTATA")
        print("="*70)
        
        if self.processed_point_cloud is not None:
            print("Prikazujem konačan point cloud...")
            # o3d.visualization.draw_geometries([self.processed_point_cloud])
            print("(Vizualizacija deaktivirana - koristi headless mod)")


def example_full_pipeline():
    """Primjer korištenja cijelog pipeline-a"""
    
    print("\n" + "╔" + "="*68 + "╗")
    print("║" + " "*15 + "ZADACA 3 - PUNI PIPELINE" + " "*29 + "║")
    print("╚" + "="*68 + "╝")
    
    # Inicijalizacija
    pipeline = TaskaPipeline(output_dir="output")
    
    # Korak 1: Snimanje
    print("\n>>> Počinjem snimanje...")
    captured_files = pipeline.step1_capture_point_clouds(
        scene_name="voce_scene",
        num_positions=3,
        method="synthetic"  # Za testiranje
    )
    
    # Korak 2: Obrada
    print("\n>>> Obrada point cloudova...")
    pipeline.step2_process_point_clouds()
    
    # Korak 3: Registracija i spajanje
    print("\n>>> Registracija i spajanje...")
    pipeline.step3_register_and_merge()
    
    # Korak 4: Clustering
    print("\n>>> Segmentacija objekata...")
    pipeline.step4_cluster_objects()
    
    # Korak 5: Detekcija i lokalizacija
    print("\n>>> Detekcija ciljnog objekta...")
    
    # Definiraj transformaciju camera -> robot
    # Primjer: Kamera je 0.1m iznad baze, rotirana 45°
    camera_translation = np.array([0.0, 0.1, 0.0])
    camera_rotation = pipeline.detector.create_rotation_matrix_z(np.pi/4)
    camera_to_robot = pipeline.detector.create_transformation_matrix(
        camera_translation,
        camera_rotation
    )
    
    results = pipeline.step5_detect_target_object(
        object_type="red_apple",
        camera_to_robot_transform=camera_to_robot
    )
    
    # Vizualizacija
    print("\n>>> Vizualizacija...")
    pipeline.visualize_results()
    
    print("\n" + "="*70)
    print("PIPELINE GOTOV!")
    print("="*70)
    print("\nRezultati sprema u: output/")
    
    return pipeline, results


def example_simple():
    """Primjer: Samo učitaj i obradi jedan point cloud"""
    
    print("\nPrimjer: Učitavanje i obrada točnog point cloud-a")
    
    processor = PointCloudProcessor()
    
    # Kreiraj sintetički point cloud
    print("Kreiram sintetički point cloud...")
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    mesh.translate([0.2, 0.3, 1.0])
    pcd = mesh.sample_points_uniformly(number_of_points=10000)
    
    # Obradi ga
    print("\nObrada...")
    pcd = processor.passthrough_filter(pcd, axis='z', min_val=0.5, max_val=1.5)
    pcd = processor.statistical_outlier_removal(pcd)
    pcd = processor.voxel_grid_downsampling(pcd, voxel_size=0.005)
    
    print("\nObrađeni cloud ima {} točaka".format(len(np.asarray(pcd.points))))


if __name__ == "__main__":
    
    if len(sys.argv) > 1 and sys.argv[1] == "--simple":
        example_simple()
    else:
        pipeline, results = example_full_pipeline()
        
        print("\n" + "="*70)
        print("INFORMACIJE ZA ROBOT")
        print("="*70)
        print(f"Centroid u camera sustavu: {results['centroid_camera']}")
        print(f"Centroid u robot sustavu: {results['centroid_robot']}")
