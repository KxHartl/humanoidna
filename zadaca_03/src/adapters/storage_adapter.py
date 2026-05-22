import os
import json
import numpy as np
import open3d as o3d
from pathlib import Path
from typing import List, Dict, Any
import matplotlib.pyplot as plt
from core.interfaces import IStorageAdapter
from core.models import SegmentedObject, TrajectoryPlan, PickPlacePoses
from core.config import StorageConfig

class StorageAdapter(IStorageAdapter):
    def __init__(self, config: StorageConfig):
        self.config = config
        os.makedirs(self.config.processed_dir, exist_ok=True)
        self.report_dir = "data/for_report"
        self.report_dirs = {
            "calibration": os.path.join(self.report_dir, "calibration"),
            "models": os.path.join(self.report_dir, "models"),
            "pictures": os.path.join(self.report_dir, "pictures"),
            "planning": os.path.join(self.report_dir, "planning"),
        }
        for d in self.report_dirs.values():
            os.makedirs(d, exist_ok=True)
        
    def save_point_cloud(self, name: str, pcd: Any) -> str:
        path = os.path.join(self.config.processed_dir, f"{name}.pcd")
        o3d.io.write_point_cloud(path, pcd)
        
        report_path = os.path.join(self.report_dirs["pictures"], f"{name}.pcd")
        o3d.io.write_point_cloud(report_path, pcd)
        return path
        
    def save_trajectory(self, name: str, trajectory: TrajectoryPlan, poses: PickPlacePoses) -> str:
        path = os.path.join(self.config.processed_dir, f"{name}.json")
        data = {
            "total_duration": trajectory.total_duration,
            "points": [
                {
                    "t": pt.time,
                    "pos": pt.positions,
                    "vel": pt.velocities,
                    "acc": pt.accelerations
                } for pt in trajectory.points
            ],
            "key_poses": {
                "home": poses.home.to_list(),
                "approach_pick": poses.approach_pick.to_list(),
                "pick": poses.pick.to_list(),
                "approach_place": poses.approach_place.to_list(),
                "place": poses.place.to_list()
            }
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
            
        import shutil
        shutil.copy2(path, os.path.join(self.report_dirs["planning"], f"{name}.json"))
            
        self._plot_trajectory_graphs(name, trajectory)
        return path
        
    def _plot_trajectory_graphs(self, name: str, traj: TrajectoryPlan):
        t = [pt.time for pt in traj.points]
        x = [pt.positions[0] for pt in traj.points]
        v = [pt.velocities[0] for pt in traj.points]
        a = [pt.accelerations[0] for pt in traj.points]
        
        plt.figure(figsize=(10, 8))
        plt.subplot(3, 1, 1)
        plt.plot(t, x, label="x(t) [m]")
        plt.ylabel("Pozicija")
        plt.legend()
        plt.grid()
        
        plt.subplot(3, 1, 2)
        plt.plot(t, v, label="v_x(t) [m/s]", color='orange')
        plt.ylabel("Brzina")
        plt.legend()
        plt.grid()
        
        plt.subplot(3, 1, 3)
        plt.plot(t, a, label="a_x(t) [m/s^2]", color='green')
        plt.xlabel("Vrijeme [s]")
        plt.ylabel("Ubrzanje")
        plt.legend()
        plt.grid()
        
        plt.tight_layout()
        img_path = os.path.join(self.config.processed_dir, f"{name}_kinematics.png")
        plt.savefig(img_path)
        plt.close()
        
        import shutil
        shutil.copy2(img_path, os.path.join(self.report_dirs["planning"], f"{name}_kinematics.png"))

    def save_objects_data(self, name: str, objects: List[SegmentedObject]) -> str:
        path = os.path.join(self.config.processed_dir, f"{name}.json")
        data = []
        for obj in objects:
            data.append({
                "id": obj.instance_id,
                "class": obj.class_name,
                "confidence": float(obj.confidence),
                "centroid": list(obj.centroid_robot_base)
            })
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
            
        import shutil
        shutil.copy2(path, os.path.join(self.report_dirs["planning"], f"{name}.json"))
        return path
        
    def save_pickup_coords(self, name: str, poses: PickPlacePoses) -> str:
        path = os.path.join(self.config.processed_dir, f"{name}.json")
        data = {
            "home": poses.home.to_list(),
            "approach": poses.approach_pick.to_list(),
            "pick": poses.pick.to_list(),
            "approach_place": poses.approach_place.to_list(),
            "place": poses.place.to_list()
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
            
        import shutil
        shutil.copy2(path, os.path.join(self.report_dirs["planning"], f"{name}.json"))
        return path
        
    def load_views_for_offline(self, capture_dir: str) -> List[Dict[str, Any]]:
        views = []
        p = Path(capture_dir)
        if (p / "captures").exists() and (p / "captures").is_dir():
            p = p / "captures"
        pos_dirs = sorted([d for d in p.iterdir() if d.is_dir() and "view" in d.name])
        for pdir in pos_dirs:
            color_path = pdir / "rgb.png"
            depth_path = pdir / "depth_m.npy"
            tcp_mat_path = pdir / "T_base_tcp.npy"
            tcp_pose_path = pdir / "tcp_pose_base.npy"
            import cv2
            color = cv2.imread(str(color_path))
            depth = np.load(str(depth_path))
            # Depth is already in meters, so pass it via dict
            tcp_mat = np.load(str(tcp_mat_path))
            tcp_pose = np.load(str(tcp_pose_path))
            
            import shutil
            shutil.copy2(str(color_path), os.path.join(self.report_dirs["pictures"], f"{pdir.name}_rgb.png"))
            
            
            intr_matrix = None
            intrinsics_path = pdir / "intrinsics.json"
            if intrinsics_path.exists():
                with open(intrinsics_path, "r") as f:
                    intrinsics_data = json.load(f)
                if "intrinsic_matrix" in intrinsics_data:
                    intr_matrix = np.array(intrinsics_data["intrinsic_matrix"])
                else:
                    fx, fy = intrinsics_data["fx"], intrinsics_data["fy"]
                    cx, cy = intrinsics_data["cx"], intrinsics_data["cy"]
                    intr_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
            
            views.append({
                "color": color,
                "depth": depth,
                "intrinsics": intr_matrix,
                "tcp_matrix": tcp_mat,
                "tcp_pose_raw": tcp_pose
            })
            
        return views

    def save_run_data(self, views: List[Dict[str, Any]]) -> str:
        from datetime import datetime
        run_name = f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        run_dir = os.path.join("data", "raw", "outputs", run_name)
        os.makedirs(run_dir, exist_ok=True)
        
        import cv2
        for i, view in enumerate(views):
            v_dir = os.path.join(run_dir, f"view_{i+1}")
            os.makedirs(v_dir, exist_ok=True)
            
            cv2.imwrite(os.path.join(v_dir, "rgb.png"), view["color"])
            np.save(os.path.join(v_dir, "depth_m.npy"), view["depth"])
            np.save(os.path.join(v_dir, "T_base_tcp.npy"), view["tcp_matrix"])
            
            if "tcp_pose_raw" in view:
                np.save(os.path.join(v_dir, "tcp_pose_base.npy"), view["tcp_pose_raw"])
            else:
                np.save(os.path.join(v_dir, "tcp_pose_base.npy"), np.zeros(6))
                
        return run_dir
