from pydantic import BaseModel, Field
from pathlib import Path
from typing import List, Tuple

class CalibrationConfig(BaseModel):
    camera_matrix_path: str = "data/camera_calibration/camera_matrix.npy"
    dist_coeffs_path: str = "data/camera_calibration/dist_coeffs.npy"
    t_cam_from_tcp_path: str = "data/camera_calibration/T_cam_from_tcp.npy"

class RobotConfig(BaseModel):
    ip: str = "127.0.0.1"
    port: int = 30002
    use_mock: bool = True
    vmax: float = 0.5
    amax: float = 0.5
    
class VisionConfig(BaseModel):
    model_path: str = "data/models/custom_fruit_seg.pt"
    confidence_threshold: float = 0.5
    robot_base_frame_override: bool = True
    capture_poses: List[Tuple[float, float, float, float, float, float]] = [
        (-0.25, -0.4, 0.4, 3.14, 0.0, 0.0),
        (-0.15, -0.3, 0.4, 3.14, 0.0, 0.0),
        (-0.35, -0.3, 0.4, 3.14, 0.0, 0.0)
    ]

class TrajectoryConfig(BaseModel):
    time_step: float = 0.05
    z_offset_approach: float = 0.1  # 100mm
    home_pose_xyz: Tuple[float, float, float] = (-0.2, -0.4, 0.3)
    home_pose_rvec: Tuple[float, float, float] = (3.14, 0.0, 0.0)
    place_pose_xyz: Tuple[float, float, float] = (0.2, -0.4, 0.1)
    place_pose_rvec: Tuple[float, float, float] = (3.14, 0.0, 0.0)

class StorageConfig(BaseModel):
    raw_dir: str = "data/raw"
    processed_dir: str = "data/processed"

class AppConfig(BaseModel):
    calibration: CalibrationConfig = CalibrationConfig()
    robot: RobotConfig = RobotConfig()
    vision: VisionConfig = VisionConfig()
    trajectory: TrajectoryConfig = TrajectoryConfig()
    storage: StorageConfig = StorageConfig()
