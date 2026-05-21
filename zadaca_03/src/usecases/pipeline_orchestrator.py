import time
import numpy as np
from rich.console import Console
from rich.table import Table
from typing import List, Optional
from core.models import PipelineContext, PipelineState
from core.interfaces import IRobotAdapter, IVisionAdapter, IStorageAdapter
from usecases.planning_usecase import PlanningUseCase

console = Console()

class PipelineOrchestrator:
    def __init__(self, 
                 context: PipelineContext,
                 robot: IRobotAdapter, 
                 vision: IVisionAdapter, 
                 storage: IStorageAdapter,
                 planner: PlanningUseCase):
        self.ctx = context
        self.robot = robot
        self.vision = vision
        self.storage = storage
        self.planner = planner
        
    def run_perception_offline(self, capture_dir: str, strategy: int = 2, calib_tweak: int = 0):
        self.ctx.transition(PipelineState.PERCEPTION)
        with console.status("[bold green]Loading offline data and processing perception...") as status:
            t_cam_from_tcp = np.load("data/camera_calibration/T_cam_from_tcp.npy")
            
            if calib_tweak == 1:
                t_cam_from_tcp = np.linalg.inv(t_cam_from_tcp)
            elif calib_tweak == 2:
                # Rotate 180 around Z (čest ručni fix)
                from scipy.spatial.transform import Rotation as R
                rot180 = np.eye(4)
                rot180[:3, :3] = R.from_euler('z', 180, degrees=True).as_matrix()
                t_cam_from_tcp = t_cam_from_tcp @ rot180
            elif calib_tweak == 3:
                # Rotate 180 around X
                from scipy.spatial.transform import Rotation as R
                rot180 = np.eye(4)
                rot180[:3, :3] = R.from_euler('x', 180, degrees=True).as_matrix()
                t_cam_from_tcp = t_cam_from_tcp @ rot180
            
            # 1. Load views
            views = self.storage.load_views_for_offline(capture_dir)
            
            # 2. Process
            merged_pcd, objects = self.vision.process_scene_views(views, t_cam_from_tcp, strategy)
            
            # 3. Save artifacts
            pcd_path = self.storage.save_point_cloud("final_merged_point_cloud", merged_pcd)
            obj_path = self.storage.save_objects_data("objects_with_robot_coords", objects)
            
            self.ctx.merged_pcd_path = pcd_path
            self.ctx.segmented_objects = objects
            
        console.print(f"[bold cyan]Perception Done![/bold cyan] Found {len(objects)} objects. PCD saved to {pcd_path}")
        self._display_objects_table(objects)
        
    def run_perception_live(self):
        from core.models import Pose
        self.ctx.transition(PipelineState.PERCEPTION)
        views = []
        
        self.robot.connect()
        if not self.vision.setup_realsense():
            console.print("[bold red]Failed to start RealSense![/bold red]")
            return
            
        try:
            for i, p_tuple in enumerate(self.vision.config.capture_poses):
                pose = Pose(xyz=p_tuple[:3], rvec=p_tuple[3:])
                console.print(f"[yellow]Moving to capture pose {i+1}...[/yellow]")
                self.robot.move_to_pose(pose, velocity=0.5, acceleration=0.5)
                time.sleep(0.5) # settle time
                
                console.print(f"[green]Capturing view {i+1}...[/green]")
                color, depth = self.vision.capture_live_view()
                
                actual_tcp = self.robot.get_tcp_pose()
                from scipy.spatial.transform import Rotation as R
                t_matrix = np.eye(4)
                t_matrix[:3, :3] = R.from_rotvec(actual_tcp.rvec).as_matrix()
                t_matrix[:3, 3] = actual_tcp.xyz
                
                views.append({
                    "color": color,
                    "depth": depth,
                    "tcp_matrix": t_matrix
                })
        finally:
            self.vision.stop_realsense()
            home = Pose(self.planner.config.home_pose_xyz, self.planner.config.home_pose_rvec)
            self.robot.move_to_pose(home, 0.5, 0.5)
            
        with console.status("[bold green]Processing live perception data...") as status:
            t_cam_from_tcp = np.load("data/camera_calibration/T_cam_from_tcp.npy")
            merged_pcd, objects = self.vision.process_scene_views(views, t_cam_from_tcp, strategy=2)
            
            pcd_path = self.storage.save_point_cloud("final_merged_point_cloud", merged_pcd)
            obj_path = self.storage.save_objects_data("objects_with_robot_coords", objects)
            
            self.ctx.merged_pcd_path = pcd_path
            self.ctx.segmented_objects = objects
            
        console.print(f"[bold cyan]Live Perception Done![/bold cyan] Found {len(objects)} objects.")
        self._display_objects_table(objects)

    def _display_objects_table(self, objects):
        table = Table(title="Segmented Objects")
        table.add_column("ID", justify="right", style="cyan")
        table.add_column("Class", style="magenta")
        table.add_column("Confidence", style="green")
        table.add_column("Centroid [X, Y, Z]", style="yellow")
        
        for obj in objects:
            cx, cy, cz = obj.centroid_robot_base
            table.add_row(
                str(obj.instance_id),
                obj.class_name,
                f"{obj.confidence:.2f}",
                f"[{cx:.3f}, {cy:.3f}, {cz:.3f}]"
            )
        console.print(table)
        
    def plan_for_object(self, object_id: int):
        self.ctx.transition(PipelineState.PLANNING)
        target = next((o for o in self.ctx.segmented_objects if o.instance_id == object_id), None)
        if not target:
            console.print(f"[bold red]Object {object_id} not found![/bold red]")
            return
            
        self.ctx.active_target = target
        poses = self.planner.generate_pick_place_poses(target)
        traj = self.planner.plan_trajectory(poses)
        
        self.ctx.pick_place_poses = poses
        self.ctx.trajectory = traj
        
        # Save artifacts
        traj_path = self.storage.save_trajectory("last_trajectory", traj, poses)
        pickup_path = self.storage.save_pickup_coords("pickup_coordinates", poses)
        console.print(f"[bold cyan]Planning Done![/bold cyan] Artifacts saved to {traj_path} and {pickup_path}")
        
    def execute_current_plan(self):
        self.ctx.transition(PipelineState.EXECUTING)
        console.print("[bold yellow]Starting Robot Execution...[/bold yellow]")
        
        poses = self.ctx.pick_place_poses
        
        self.robot.connect()
        self.robot.move_to_pose(poses.home, 0.5, 0.5)
        
        self.robot.set_gripper(close=False)
        self.robot.execute_trajectory(self.ctx.trajectory)
        self.robot.move_to_pose(poses.home, 0.5, 0.5)
        self.robot.disconnect()
        
        self.ctx.transition(PipelineState.IDLE)
        console.print("[bold green]Execution Done! Ready for next object.[/bold green]")
