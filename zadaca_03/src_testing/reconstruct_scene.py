# reconstruct_scene.py
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import open3d as o3d

from io_utils import (
    ensure_dir,
    load_npy_matrix,
    load_point_cloud,
    make_point_cloud,
    save_json,
    save_point_cloud,
)
from segment import SegmentationResult, segment_multiple_views


@dataclass
class ReconstructionResult:
    target_class: str
    views_used: int
    registered_pcd_path: str | None
    centroid_marker_pcd_path: str | None
    centroid_base_xyz: list[float] | None


def _resolve_tcp_from_cam(cfg: Any, invert_cam_tcp: bool = True) -> np.ndarray:
    """
    Preferira eksplicitni T_tcp_from_cam ako postoji.
    Ako ne postoji, koristi cfg.paths.t_cam_from_tcp.
    Ako znaš da je u toj datoteci zapravo spremljen T_cam_from_tcp,
    postavi invert_cam_tcp=True.
    """
    tcp_from_cam_path = Path(cfg.paths.t_tcp_from_cam)
    cam_from_tcp_path = Path(cfg.paths.t_cam_from_tcp)

    if tcp_from_cam_path.exists():
        return load_npy_matrix(tcp_from_cam_path, expected_shape=(4, 4))

    T = load_npy_matrix(cam_from_tcp_path, expected_shape=(4, 4))
    if invert_cam_tcp:
        return np.linalg.inv(T)
    return T


def _get_best_instance_pcd_path(seg_result: SegmentationResult) -> str:
    if seg_result.best_instance_id is None:
        raise RuntimeError(
            f"Nema best_instance_id za {seg_result.image_path}. "
            "Najvjerojatnije segmentacija nije pronašla ciljnu instancu."
        )

    for inst in seg_result.instances:
        if inst.instance_id == seg_result.best_instance_id:
            if not inst.segmented_pcd_path:
                raise RuntimeError(
                    f"Best instanca za {seg_result.image_path} nema spremljen segmented_pcd_path. "
                    "Provjeri cfg.debug.save_segmented_clouds=True."
                )
            return inst.segmented_pcd_path

    raise RuntimeError(f"Best instanca nije pronađena u listi instanci za {seg_result.image_path}.")


def _light_post_merge_clean(pcd: o3d.geometry.PointCloud, cfg: Any) -> o3d.geometry.PointCloud:
    if pcd.is_empty():
        return pcd

    clean = pcd
    voxel_size = getattr(cfg.point_cloud, "voxel_size", 0.0)
    if voxel_size and voxel_size > 0.0:
        clean = clean.voxel_down_sample(voxel_size)

    if len(clean.points) >= max(10, getattr(cfg.point_cloud, "sor_nb_neighbors", 20)):
        clean, ind = clean.remove_statistical_outlier(
            nb_neighbors=getattr(cfg.point_cloud, "sor_nb_neighbors", 20),
            std_ratio=getattr(cfg.point_cloud, "sor_std_ratio", 2.0),
        )
        if len(ind) == 0:
            return pcd

    radius_nb = getattr(cfg.point_cloud, "radius_outlier_nb_points", 0)
    radius = getattr(cfg.point_cloud, "radius_outlier_radius", 0.0)
    if radius_nb and radius and len(clean.points) >= radius_nb:
        clean, ind = clean.remove_radius_outlier(
            nb_points=radius_nb,
            radius=radius,
        )
        if len(ind) == 0:
            return clean

    return clean


def _transform_cloud(pcd: o3d.geometry.PointCloud, T: np.ndarray) -> o3d.geometry.PointCloud:
    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    return out


def _centroid_of_cloud(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    points = np.asarray(pcd.points)
    if len(points) == 0:
        raise RuntimeError("Finalni registrirani cloud je prazan, ne mogu izračunati centroid.")
    return np.mean(points, axis=0)


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

    T_tcp_cam = _resolve_tcp_from_cam(cfg, invert_cam_tcp=invert_cam_tcp)

    transformed_clouds: list[o3d.geometry.PointCloud] = []

    for idx, (seg_result, T_base_tcp) in enumerate(zip(segment_results, base_tcp_transforms)):
        if T_base_tcp.shape != (4, 4):
            raise ValueError(f"T_base_tcp za view {idx} nije 4x4 matrica.")

        segmented_pcd_path = _get_best_instance_pcd_path(seg_result)
        pcd_cam = load_point_cloud(segmented_pcd_path)

        # Matematicko stichanje: P_base = T_base_tcp * T_tcp_cam * P_cam
        T_base_cam = T_base_tcp @ T_tcp_cam
        pcd_base = _transform_cloud(pcd_cam, T_base_cam)
        transformed_clouds.append(pcd_base)

    if len(transformed_clouds) == 0:
        raise RuntimeError("Nema nijednog oblaka za rekonstrukciju.")

    # Spajanje svih transformiranih oblaka u jedan bez ICP-a (koristimo samo robota)
    merged = o3d.geometry.PointCloud()
    for cloud in transformed_clouds:
        merged += cloud

    merged = _light_post_merge_clean(merged, cfg)
    centroid = _centroid_of_cloud(merged)

    registered_pcd_path = None
    if getattr(cfg.debug, "save_registered_cloud", True):
        registered_pcd_path = output_dir / "target_registered.pcd"
        save_point_cloud(registered_pcd_path, merged)

    centroid_marker_pcd_path = None
    if getattr(cfg.debug, "save_centroid_marker", True):
        centroid_marker = make_point_cloud(
            points=np.asarray([centroid], dtype=np.float64),
            colors=np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32),
        )
        centroid_marker_pcd_path = output_dir / "target_centroid_marker.pcd"
        save_point_cloud(centroid_marker_pcd_path, centroid_marker)

    result = ReconstructionResult(
        target_class=cfg.target.target_class,
        views_used=len(segment_results),
        registered_pcd_path=str(registered_pcd_path) if registered_pcd_path is not None else None,
        centroid_marker_pcd_path=str(centroid_marker_pcd_path) if centroid_marker_pcd_path is not None else None,
        centroid_base_xyz=centroid.astype(float).tolist(),
    )

    save_json(output_dir / "reconstruction_result.json", asdict(result))
    return result


def reconstruct_scene_from_pairs(
    pairs: Iterable[tuple[str | Path, str | Path]],
    base_tcp_transforms: Iterable[np.ndarray],
    cfg: Any,
    segment_output_dir: str | Path | None = None,
    reconstruct_output_dir: str | Path | None = None,
    invert_cam_tcp: bool = False,
) -> ReconstructionResult:
    pairs = list(pairs)
    segment_output_dir = (
        Path(segment_output_dir)
        if segment_output_dir is not None
        else Path(cfg.paths.output_dir) / "segment"
    )

    segment_results = segment_multiple_views(
        pairs=pairs,
        cfg=cfg,
        output_dir=segment_output_dir,
    )

    return reconstruct_scene_from_results(
        segment_results=segment_results,
        base_tcp_transforms=base_tcp_transforms,
        cfg=cfg,
        output_dir=reconstruct_output_dir,
        invert_cam_tcp=invert_cam_tcp,
    )


if __name__ == "__main__":
    from config import cfg

    raise SystemExit(
        "Koristi reconstruct_scene_from_results(...) ili reconstruct_scene_from_pairs(...)."
    )
