import numpy as np
from typing import List, Tuple, Optional
from core.models import Pose, TrajectoryPlan, TrajectoryPoint, PickPlacePoses, SegmentedObject
from core.config import TrajectoryConfig

class TrajectoryGenerator:
    @staticmethod
    def quintic_polynomial(t: float, t_total: float, q0: float, qf: float) -> Tuple[float, float, float]:
        """Calculates position, velocity, and acceleration for a quintic profile."""
        if t <= 0:
            return q0, 0.0, 0.0
        if t >= t_total:
            return qf, 0.0, 0.0
            
        tau = t / t_total
        h = qf - q0
        
        pos = q0 + h * (10*(tau**3) - 15*(tau**4) + 6*(tau**5))
        vel = (h / t_total) * (30*(tau**2) - 60*(tau**3) + 30*(tau**4))
        acc = (h / (t_total**2)) * (60*tau - 180*(tau**2) + 120*(tau**3))
        
        return pos, vel, acc

class PlanningUseCase:
    def __init__(self, config: TrajectoryConfig):
        self.config = config
        
    def generate_pick_place_poses(self, target: SegmentedObject, 
                                 place_loc: Optional[Tuple[float,float,float]] = None) -> PickPlacePoses:
        home = Pose(self.config.home_pose_xyz, self.config.home_pose_rvec)
        
        pick_z = target.centroid_robot_base[2] + getattr(self.config, 'grasp_offset_z', -0.02)
        pick_pose = Pose(
            (target.centroid_robot_base[0], target.centroid_robot_base[1], pick_z),
            self.config.home_pose_rvec
        )
        
        app_pick_pose = Pose(
            (pick_pose.xyz[0], pick_pose.xyz[1], pick_pose.xyz[2] + self.config.z_offset_approach),
            self.config.home_pose_rvec
        )
        
        p_loc = place_loc if place_loc else self.config.place_pose_xyz
        place_pose = Pose(p_loc, self.config.place_pose_rvec)
        
        app_place_pose = Pose(
            (place_pose.xyz[0], place_pose.xyz[1], place_pose.xyz[2] + self.config.z_offset_approach),
            self.config.place_pose_rvec
        )
        
        return PickPlacePoses(
            target_object=target,
            home=home,
            approach_pick=app_pick_pose,
            pick=pick_pose,
            approach_place=app_place_pose,
            place=place_pose
        )
        
    def _create_segment(self, start_pose: Pose, end_pose: Pose, duration: float, t_start: float) -> List[TrajectoryPoint]:
        points = []
        steps = int(duration / self.config.time_step)
        
        start_vec = np.array(start_pose.to_list())
        end_vec = np.array(end_pose.to_list())
        
        for i in range(steps + 1):
            t_local = i * self.config.time_step
            pos_list = []
            vel_list = []
            acc_list = []
            
            for j in range(6):
                p, v, a = TrajectoryGenerator.quintic_polynomial(t_local, duration, start_vec[j], end_vec[j])
                pos_list.append(p)
                vel_list.append(v)
                acc_list.append(a)
                
            points.append(TrajectoryPoint(
                time=t_start + t_local,
                positions=pos_list,
                velocities=vel_list,
                accelerations=acc_list
            ))
            
        return points

    def plan_trajectory(self, poses: PickPlacePoses) -> TrajectoryPlan:
        # Define segments and durations
        segments = [
            (poses.home, poses.approach_pick, 2.5),
            (poses.approach_pick, poses.pick, 1.5),
            (poses.pick, poses.approach_pick, 1.5),
            (poses.approach_pick, poses.approach_place, 2.5),
            (poses.approach_place, poses.place, 1.5),
            (poses.place, poses.approach_place, 1.5),
            (poses.approach_place, poses.home, 2.0)
        ]
        
        all_points = []
        current_time = 0.0
        
        for start, end, dur in segments:
            seg_points = self._create_segment(start, end, dur, current_time)
            # Remove last point if not the final segment to avoid duplication
            all_points.extend(seg_points[:-1])
            current_time += dur
            
        # Add the very last point
        last_seg = self._create_segment(segments[-1][0], segments[-1][1], segments[-1][2], current_time - segments[-1][2])
        all_points.append(last_seg[-1])
        
        return TrajectoryPlan(
            points=all_points,
            total_duration=current_time
        )
        
    def plan_trajectory_segments(self, poses: PickPlacePoses) -> List[TrajectoryPlan]:
        segments = [
            (poses.home, poses.approach_pick, 2.5),
            (poses.approach_pick, poses.pick, 1.5),
            (poses.pick, poses.approach_pick, 1.5),
            (poses.approach_pick, poses.approach_place, 2.5),
            (poses.approach_place, poses.place, 1.5),
            (poses.place, poses.approach_place, 1.5),
            (poses.approach_place, poses.home, 2.0)
        ]
        
        plans = []
        for start, end, dur in segments:
            pts = self._create_segment(start, end, dur, 0.0)
            plans.append(TrajectoryPlan(points=pts, total_duration=dur))
            
        return plans
