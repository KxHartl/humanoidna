from pydantic import BaseModel, Field
from pathlib import Path
from typing import List, Tuple

class CalibrationConfig(BaseModel):
    camera_matrix_path: str = "data/camera_calibration/camera_matrix.npy"
    dist_coeffs_path: str = "data/camera_calibration/dist_coeffs.npy"
    t_cam_from_tcp_path: str = "data/camera_calibration/T_cam_from_tcp.npy"
    tcp_cam_translation_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # Reset, using ideal calibration

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
    z_offset_correction: float = 0.03  # Correction for robot TCP tool offset mismatch
    capture_poses: List[Tuple[float, float, float, float, float, float]] = [
        (-0.51405, 0.19615, 0.38585, 0.455, -2.710, -0.177),
        (-0.51144, 0.28870, 0.21725, 1.772, 2.439, -0.421),
        (-0.36685, 0.07215, 0.27685, 3.085, 0.550, -0.448)
    ]

class TrajectoryConfig(BaseModel):
    time_step: float = 0.05
    z_offset_approach: float = 0.1  # 100mm
    grasp_offset_z: float = -0.02  # -20mm
    home_pose_xyz: Tuple[float, float, float] = (-0.54366, 0.09350, 0.19115)
    home_pose_rvec: Tuple[float, float, float] = (0.040, 3.146, -0.051)
    place_pose_xyz: Tuple[float, float, float] = (-0.54661, 0.28646, 0.01218)
    place_pose_rvec: Tuple[float, float, float] = (0.512, 3.111, 0.059)

class StorageConfig(BaseModel):
    raw_dir: str = "data/raw"
    processed_dir: str = "data/processed"

class AppConfig(BaseModel):
    calibration: CalibrationConfig = CalibrationConfig()
    robot: RobotConfig = RobotConfig()
    vision: VisionConfig = VisionConfig()
    trajectory: TrajectoryConfig = TrajectoryConfig()
    storage: StorageConfig = StorageConfig()
