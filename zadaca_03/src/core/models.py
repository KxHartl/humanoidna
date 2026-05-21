import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple, Any, Optional

class PipelineState(str, Enum):
    IDLE = "idle"
    CAPTURING = "capturing"
    RECONSTRUCTING = "reconstructing"
    PERCEPTION = "perception"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class Pose:
    """Homogeneous 6D pose: [x, y, z, rx, ry, rz]"""
    xyz: Tuple[float, float, float]
    rvec: Tuple[float, float, float]
    
    def to_list(self) -> List[float]:
        return list(self.xyz) + list(self.rvec)

@dataclass
class SegmentedObject:
    """Detected object in 3D space"""
    instance_id: int
    class_name: str
    confidence: float
    centroid_robot_base: Tuple[float, float, float]  # x, y, z
    pcd: Any  # open3d.geometry.PointCloud - typed as Any to avoid strict O3D dependency in core

@dataclass
class PickPlacePoses:
    """Target grasp configuration"""
    target_object: SegmentedObject
    home: Pose
    approach_pick: Pose
    pick: Pose
    approach_place: Pose
    place: Pose

@dataclass
class TrajectoryPoint:
    time: float
    positions: List[float]
    velocities: List[float]
    accelerations: List[float]

@dataclass
class TrajectoryPlan:
    points: List[TrajectoryPoint]
    total_duration: float
    
@dataclass
class PipelineContext:
    """Thread-safe runtime state"""
    state: PipelineState = PipelineState.IDLE
    run_id: str = ""
    # Results
    captured_files: List[str] = field(default_factory=list)
    merged_pcd_path: Optional[str] = None
    segmented_objects: List[SegmentedObject] = field(default_factory=list)
    active_target: Optional[SegmentedObject] = None
    pick_place_poses: Optional[PickPlacePoses] = None
    trajectory: Optional[TrajectoryPlan] = None
    
    def transition(self, new_state: PipelineState) -> None:
        self.state = new_state
