#!/usr/bin/env python3
"""
Point Cloud Processing Pipeline za Zadaca 3
Obrada point cloud-a prema PCL metodama:
- PassThrough filtering
- Statistical Outlier Removal
- VoxelGrid downsampling
- ICP registration
- Point cloud fusion/stitching
- Euclidean clustering
- RANSAC plane segmentation
"""

import numpy as np
from pathlib import Path
from typing import List, Tuple

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: open3d nije instaliran. Instaliraj s: pip install open3d")

try:
    import pyntcloud
    HAS_PYNTCLOUD = True
except ImportError:
    HAS_PYNTCLOUD = False


class PointCloudProcessor:
    """Klasa za obradu point cloud-a"""
    
    def __init__(self):
        if not HAS_OPEN3D:
            raise ImportError("open3d je obavezan!")
    
    # ============================================================
    # 1. PASSTHROUGH FILTERING - Uklanjanje točaka van ROI-a
    # ============================================================
    
    def passthrough_filter(self, pcd, axis='z', min_val=0.0, max_val=2.0):
        """
        PassThrough filter - uklanja točke izvan zadanog intervala
        
        Args:
            pcd: open3d.geometry.PointCloud
            axis (str): 'x', 'y', ili 'z'
            min_val (float): Minimalna vrijednost
            max_val (float): Maksimalna vrijednost
            
        Returns:
            open3d.geometry.PointCloud: Filtrirani point cloud
        """
        print(f"PassThrough filtering ({axis}: {min_val}-{max_val})...")
        
        points = np.asarray(pcd.points)
        
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        axis_idx = axis_map[axis]
        
        # Filtriraj točke
        mask = (points[:, axis_idx] >= min_val) & (points[:, axis_idx] <= max_val)
        
        # Kreiraj novi point cloud
        pcd_filtered = o3d.geometry.PointCloud()
        pcd_filtered.points = o3d.utility.Vector3dVector(points[mask])
        
        # Prosljeđi boje ako postoje
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            pcd_filtered.colors = o3d.utility.Vector3dVector(colors[mask])
        
        # Prosljeđi normale ako postoje
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            pcd_filtered.normals = o3d.utility.Vector3dVector(normals[mask])
        
        print(f"  Prije: {len(points)} točaka → Nakon: {len(np.asarray(pcd_filtered.points))} točaka")
        return pcd_filtered
    
    # ============================================================
    # 2. STATISTICAL OUTLIER REMOVAL - Uklanjanje outliera
    # ============================================================
    
    def statistical_outlier_removal(self, pcd, nb_neighbors=20, std_ratio=2.0):
        """
        Statistical Outlier Removal - uklanja anomalne točke
        
        Args:
            pcd: open3d.geometry.PointCloud
            nb_neighbors (int): Broj susjednih točaka za analizu
            std_ratio (float): Standardna devijacija omjer
            
        Returns:
            open3d.geometry.PointCloud: Čišćeni point cloud
        """
        print(f"Statistical Outlier Removal (neighbors={nb_neighbors}, std_ratio={std_ratio})...")
        
        points_before = len(np.asarray(pcd.points))
        
        # Primijeni SOR filter
        pcd_cleaned, inliers = pcd.remove_statistical_outliers(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        
        points_after = len(np.asarray(pcd_cleaned.points))
        removed = points_before - points_after
        
        print(f"  Prije: {points_before} točaka → Nakon: {points_after} točaka")
        print(f"  Uklonjeno outliera: {removed} ({100*removed/points_before:.1f}%)")
        
        return pcd_cleaned
    
    # ============================================================
    # 3. VOXEL GRID DOWNSAMPLING - Smanjenje broja točaka
    # ============================================================
    
    def voxel_grid_downsampling(self, pcd, voxel_size=0.01):
        """
        VoxelGrid downsampling - smanjuje broj točaka
        
        Args:
            pcd: open3d.geometry.PointCloud
            voxel_size (float): Veličina voxela
            
        Returns:
            open3d.geometry.PointCloud: Downsampled point cloud
        """
        print(f"VoxelGrid Downsampling (voxel_size={voxel_size})...")
        
        points_before = len(np.asarray(pcd.points))
        
        pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
        
        points_after = len(np.asarray(pcd_downsampled.points))
        reduction = (1 - points_after/points_before) * 100
        
        print(f"  Prije: {points_before} točaka → Nakon: {points_after} točaka")
        print(f"  Redukcija: {reduction:.1f}%")
        
        return pcd_downsampled
    
    # ============================================================
    # 4. ICP REGISTRATION - Usklađivanje point cloudova
    # ============================================================
    
    def estimate_normals(self, pcd, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)):
        """
        Procjena normala za point cloud
        
        Args:
            pcd: open3d.geometry.PointCloud
            search_param: Parametri pretrage
            
        Returns:
            open3d.geometry.PointCloud: Point cloud s normalama
        """
        print("Procjenjujem normale...")
        pcd.estimate_normals(search_param=search_param)
        return pcd
    
    def icp_registration(self, source, target, init_transform=None, threshold=0.02, max_iteration=200):
        """
        ICP (Iterative Closest Point) registration
        
        Args:
            source: Izvorni point cloud
            target: Ciljni point cloud
            init_transform: Inicijalna transformacija
            threshold (float): Distanca za opozvanje
            max_iteration (int): Maksimalan broj iteracija
            
        Returns:
            Tuple: (transformacija, informacije o registraciji)
        """
        print("ICP Registration...")
        
        # Ako nema normala, procijeni ih
        if not source.has_normals():
            self.estimate_normals(source)
        if not target.has_normals():
            self.estimate_normals(target)
        
        # ICP registracija
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            threshold,
            init_transform if init_transform is not None else np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=max_iteration
            )
        )
        
        print(f"  Fitness: {result.fitness:.4f}")
        print(f"  RMSE: {result.inlier_rmse:.6f}")
        print(f"  Iteracija: {result.correspondence_set.shape[0]}")
        
        return result.transformation, result
    
    # ============================================================
    # 5. POINT CLOUD FUSION/STITCHING - Spajanje cloudova
    # ============================================================
    
    def register_and_merge_clouds(self, point_clouds: List, voxel_size=0.01):
        """
        Registracija i spajanje više point cloudova u jedan
        
        Args:
            point_clouds (List): Lista point cloudova za spajanje
            voxel_size (float): Veličina voxela za finalnu fuziju
            
        Returns:
            open3d.geometry.PointCloud: Konačan fusionirani cloud
        """
        print("\n" + "="*60)
        print("REGISTRACIJA I SPAJANJE POINT CLOUDOVA")
        print("="*60)
        
        if len(point_clouds) == 0:
            raise ValueError("Nema point cloudova za spajanje!")
        
        if len(point_clouds) == 1:
            print("Samo jedan point cloud - vraćam kao je")
            return point_clouds[0]
        
        # Koristi prvi kao referentni
        reference = point_clouds[0]
        merged = reference
        
        # Registriraj sve ostale prema referentnom
        for i, source in enumerate(point_clouds[1:], start=1):
            print(f"\nRegistriranje clouda {i+1} prema referentnom...")
            
            # Downsample za bržu registraciju
            source_down = source.voxel_down_sample(voxel_size=voxel_size)
            reference_down = reference.voxel_down_sample(voxel_size=voxel_size)
            
            # ICP registracija
            transform, _ = self.icp_registration(source_down, reference_down)
            
            # Primijeni transformaciju
            source_transformed = source.transform(transform)
            
            # Spoji s mergeom
            merged += source_transformed
        
        # Finalna deduplikacija (downsampling)
        print("\nFinalna deduplikacija...")
        merged_dedup = merged.voxel_down_sample(voxel_size=voxel_size)
        
        print(f"Konačan point cloud: {len(np.asarray(merged_dedup.points))} točaka")
        
        return merged_dedup
    
    # ============================================================
    # 6. EUCLIDEAN CLUSTERING - Segmentacija objekata
    # ============================================================
    
    def euclidean_clustering(self, pcd, eps=0.05, min_points=10):
        """
        Euclidean Clustering - odvajanje objekata
        
        Args:
            pcd: open3d.geometry.PointCloud
            eps (float): Clustering distanca
            min_points (int): Minimalan broj točaka u clusteru
            
        Returns:
            Tuple: (labels, broj clustera)
        """
        print(f"\nEuclidean Clustering (eps={eps}, min_points={min_points})...")
        
        # Kreiraj KDTree
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        
        points = np.asarray(pcd.points)
        labels = np.full(len(points), -1, dtype=int)
        
        cluster_id = 0
        
        for i in range(len(points)):
            if labels[i] != -1:  # Već dodijeljen
                continue
            
            # Pronađi k-nearest neighbours
            [k, idx, _] = pcd_tree.search_knn_vector_3d(points[i], eps+1)
            
            # Kreiraj seed queue
            if len(idx) < min_points:
                labels[i] = -2  # Šum
                continue
            
            # BFS za pronalaženje clustera
            queue = idx[:k]
            cluster_size = 0
            
            while len(queue) > 0:
                idx_curr = queue[0]
                queue = queue[1:]
                
                if labels[idx_curr] != -1:
                    continue
                
                labels[idx_curr] = cluster_id
                cluster_size += 1
                
                # Pronađi susjedne točke
                [k_n, idx_n, _] = pcd_tree.search_knn_vector_3d(points[idx_curr], eps+1)
                
                if k_n > min_points:
                    for idx_next in idx_n:
                        if labels[idx_next] == -1:
                            queue.append(idx_next)
            
            if cluster_size >= min_points:
                cluster_id += 1
            else:
                labels[np.where(labels == cluster_id)] = -2
                cluster_id -= 1
        
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0) - (1 if -2 in labels else 0)
        
        print(f"  Pronađeno {num_clusters} clustera")
        
        return labels, num_clusters
    
    def extract_clusters(self, pcd, labels, num_clusters):
        """
        Ekstraktuj pojedinatne clustere
        
        Args:
            pcd: open3d.geometry.PointCloud
            labels: Labele iz clusteringa
            num_clusters (int): Broj clustera
            
        Returns:
            List: Lista pojedinatnih point cloudova (po jedan za svaki cluster)
        """
        clusters = []
        
        for cluster_id in range(num_clusters):
            mask = labels == cluster_id
            
            pcd_cluster = o3d.geometry.PointCloud()
            points = np.asarray(pcd.points)
            pcd_cluster.points = o3d.utility.Vector3dVector(points[mask])
            
            if pcd.has_colors():
                colors = np.asarray(pcd.colors)
                pcd_cluster.colors = o3d.utility.Vector3dVector(colors[mask])
            
            clusters.append(pcd_cluster)
            
            print(f"  Cluster {cluster_id}: {len(np.asarray(pcd_cluster.points))} točaka")
        
        return clusters
    
    # ============================================================
    # 7. RANSAC PLANE SEGMENTATION - Detekcija planarnih površina
    # ============================================================
    
    def ransac_plane_segmentation(self, pcd, distance_threshold=0.01, ransac_n=3, num_iterations=100):
        """
        RANSAC plane segmentation - pronalazi planarnu površinu
        
        Args:
            pcd: open3d.geometry.PointCloud
            distance_threshold (float): Distanca do ravnine
            ransac_n (int): Broj točaka za fit
            num_iterations (int): Broj RANSAC iteracija
            
        Returns:
            Tuple: (plane_model, inliers_indices, outliers_pcd)
        """
        print(f"\nRANSAC Plane Segmentation (distance_threshold={distance_threshold})...")
        
        # Procijeni normale ako ne postoje
        if not pcd.has_normals():
            self.estimate_normals(pcd)
        
        # RANSAC plane fitting
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        
        print(f"  Ravnina: {plane_model[:3]} * (x,y,z) + {plane_model[3]:.6f} = 0")
        print(f"  Inliers: {len(inliers)} točaka")
        
        # Ekstraktuj inliers i outliers
        pcd_inliers = pcd.select_by_index(inliers)
        pcd_outliers = pcd.select_by_index(inliers, invert=True)
        
        return plane_model, inliers, pcd_outliers
    
    # ============================================================
    # KOMPLETNA OBRADA PIPELINE
    # ============================================================
    
    def process_point_cloud(self, pcd, config=None):
        """
        Kompletna obrada point clouda
        
        Args:
            pcd: open3d.geometry.PointCloud
            config (dict): Konfiguracija parametara
            
        Returns:
            open3d.geometry.PointCloud: Obrađeni point cloud
        """
        if config is None:
            config = {
                'passthrough': {'axis': 'z', 'min_val': 0.0, 'max_val': 2.0},
                'statistical_outlier': {'nb_neighbors': 20, 'std_ratio': 2.0},
                'voxel_grid': {'voxel_size': 0.01},
            }
        
        print("\n" + "="*60)
        print("OBRADA POINT CLOUDA")
        print("="*60)
        
        # 1. PassThrough filtering
        if 'passthrough' in config:
            pcd = self.passthrough_filter(pcd, **config['passthrough'])
        
        # 2. Statistical Outlier Removal
        if 'statistical_outlier' in config:
            pcd = self.statistical_outlier_removal(pcd, **config['statistical_outlier'])
        
        # 3. VoxelGrid downsampling
        if 'voxel_grid' in config:
            pcd = self.voxel_grid_downsampling(pcd, **config['voxel_grid'])
        
        return pcd


def main():
    """Primjer korištenja"""
    print("Point Cloud Processing Pipeline - Primjer")


if __name__ == "__main__":
    main()
