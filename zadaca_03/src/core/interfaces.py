from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Optional, Any
from .models import Pose, SegmentedObject, TrajectoryPlan

class IRobotAdapter(ABC):
    @abstractmethod
    def connect(self) -> bool:
        pass
        
    @abstractmethod
    def disconnect(self) -> None:
        pass
        
    @abstractmethod
    def get_tcp_pose(self) -> Pose:
        pass

    @abstractmethod
    def move_to_pose(self, pose: Pose, velocity: float, acceleration: float, async_move: bool = False) -> bool:
        pass
        
    @abstractmethod
    def execute_trajectory(self, trajectory: TrajectoryPlan) -> bool:
        pass

    @abstractmethod
    def set_gripper(self, close: bool) -> None:
        pass

class IVisionAdapter(ABC):
    @abstractmethod
    def load_model(self) -> None:
        pass
        
    @abstractmethod
    def process_scene_views(self, views: List[Dict[str, Any]], t_cam_from_tcp: Any) -> Tuple[Any, List[SegmentedObject]]:
        """
        Takes multiple views (RGB, Depth, TCP poses), performs reconstruction, stitching, 
        and YOLO inference, then returns the final standard point cloud and a list of segmented objects.
        """
        pass

class IStorageAdapter(ABC):
    @abstractmethod
    def save_point_cloud(self, name: str, pcd: Any) -> str:
        pass
        
    @abstractmethod
    def save_trajectory(self, name: str, trajectory: TrajectoryPlan, poses: Any) -> str:
        pass
        
    @abstractmethod
    def save_objects_data(self, name: str, objects: List[SegmentedObject]) -> str:
        pass

    @abstractmethod
    def save_pickup_coords(self, name: str, poses: Any) -> str:
        pass
    
    @abstractmethod
    def load_views_for_offline(self, capture_dir: str) -> List[Dict[str, Any]]:
        pass
