from pydantic import BaseModel, Field
from pathlib import Path
from typing import List, Tuple

class CalibrationConfig(BaseModel):
    camera_matrix_path: str = "data/camera_calibration/camera_matrix.npy"
    dist_coeffs_path: str = "data/camera_calibration/dist_coeffs.npy"
    t_cam_from_tcp_path: str = "data/camera_calibration/T_cam_from_tcp.npy"

class RobotConfig(BaseModel):
    ip: str = "192.168.40.14"
    port: int = 30002
    use_mock: bool = True
    vmax: float = 0.5
    amax: float = 0.5

class VisionConfig(BaseModel):
    model_path: str = "data/models/custom_fruit_seg.pt"
    confidence_threshold: float = 0.20
    dbscan_eps: float = 0.08
    dbscan_min_pts: int = 30
    robot_base_frame_override: bool = True
    capture_poses: List[Tuple[float, float, float, float, float, float]] = [
        (-0.49842, 0.13950, 0.39228, 0.812, -2.577, -0.305),
        (-0.36363, 0.27000, 0.26670, 2.614, 1.652, -0.906),
        (-0.39677, 0.05200, 0.37827, 3.09, -0.471, -0.472)
    ]

class TrajectoryConfig(BaseModel):
    time_step: float = 0.05
    z_offset_approach: float = 0.1  # 100mm
    home_pose_xyz: Tuple[float, float, float] = (-0.53942, 0.12400, 0.25632)
    home_pose_rvec: Tuple[float, float, float] = (3.155, 0.653, -0.057)
    place_pose_xyz: Tuple[float, float, float] = (-0.45782, -0.02120, 0.11682)
    place_pose_rvec: Tuple[float, float, float] = (2.964, 1.097, -0.078)

class StorageConfig(BaseModel):
    raw_dir: str = "data/raw"
    processed_dir: str = "data/processed"

class AppConfig(BaseModel):
    calibration: CalibrationConfig = CalibrationConfig()
    robot: RobotConfig = RobotConfig()
    vision: VisionConfig = VisionConfig()
    trajectory: TrajectoryConfig = TrajectoryConfig()
    storage: StorageConfig = StorageConfig()
