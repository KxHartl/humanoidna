import numpy as np
import time
import socket
from typing import List, Optional
from core.interfaces import IRobotAdapter
from core.models import Pose, TrajectoryPlan, TrajectoryPoint
from core.config import RobotConfig

try:
    from rtde_receive import RTDEReceiveInterface
except ImportError:
    RTDEReceiveInterface = None

class RobotAdapter(IRobotAdapter):
    def __init__(self, config: RobotConfig):
        self.config = config
        self.connected = False
        self.mock_tcp_pose = Pose((0.0, -0.4, 0.2), (3.14, 0.0, 0.0))
        self.rtde_receiver = None
        
    def connect(self) -> bool:
        if self.config.use_mock:
            self.connected = True
            return True
            
        if RTDEReceiveInterface is None:
            raise ImportError("ur_rtde not installed. Cannot use real robot.")
            
        try:
            self.rtde_receiver = RTDEReceiveInterface(self.config.ip)
            self.connected = True
            return True
        except Exception as e:
            print(f"Failed to connect to UR robot at {self.config.ip}: {e}")
            return False
            
    def disconnect(self) -> None:
        if self.rtde_receiver:
            self.rtde_receiver.disconnect()
            self.rtde_receiver = None
        self.connected = False
        
    def _send_urscript(self, program: str, timeout_s: float = 5.0) -> None:
        if self.config.use_mock:
            return
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout_s)
            s.connect((self.config.ip, self.config.port))
            s.sendall(program.encode("utf-8"))
            
    def _pose_to_urscript(self, pose: Pose) -> str:
        x, y, z = pose.xyz
        rx, ry, rz = pose.rvec
        return f"p[{x:.6f}, {y:.6f}, {z:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f}]"

    def get_tcp_pose(self) -> Pose:
        if self.config.use_mock or not self.rtde_receiver:
            return self.mock_tcp_pose
            
        actual = self.rtde_receiver.getActualTCPPose()
        return Pose(
            xyz=(actual[0], actual[1], actual[2]),
            rvec=(actual[3], actual[4], actual[5])
        )
        
    def move_to_pose(self, pose: Pose, velocity: float, acceleration: float, async_move: bool = False) -> bool:
        if not self.connected:
            return False
            
        self.mock_tcp_pose = pose
        if self.config.use_mock:
            if not async_move:
                time.sleep(0.5)
            return True
            
        pose_str = self._pose_to_urscript(pose)
        cmd = f"def move_prog():\n  movej(get_inverse_kin({pose_str}), a={acceleration:.6f}, v={velocity:.6f})\nend\nmove_prog()\n"
        self._send_urscript(cmd)
        
        if not async_move:
            # Wait for robot to reach target
            t0 = time.time()
            while time.time() - t0 < 20.0:
                current = self.get_tcp_pose()
                p_err = np.linalg.norm(np.array(current.xyz) - np.array(pose.xyz))
                if p_err < 0.005:
                    time.sleep(0.3)
                    break
                time.sleep(0.1)
                
        return True
        
    def execute_trajectory(self, trajectory: TrajectoryPlan) -> bool:
        if not self.connected:
            return False
            
        if self.config.use_mock:
            time.sleep(trajectory.total_duration)
            return True
            
        # Convert TrajectoryPlan to a series of servoj commands
        lines = ["def traj_prog():"]
        # Assuming the dt is relatively constant
        if len(trajectory.points) < 2:
            return False
            
        dt = trajectory.points[1].time - trajectory.points[0].time
        lookahead = 0.1
        gain = 300
        
        for pt in trajectory.points:
            pose_str = f"p[{pt.positions[0]:.6f}, {pt.positions[1]:.6f}, {pt.positions[2]:.6f}, {pt.positions[3]:.6f}, {pt.positions[4]:.6f}, {pt.positions[5]:.6f}]"
            lines.append(f"  servoj(get_inverse_kin({pose_str}), t={dt:.6f}, lookahead_time={lookahead:.3f}, gain={gain})")
            
        lines.append("end")
        lines.append("traj_prog()\n")
        
        program = "\n".join(lines)
        self._send_urscript(program)
        time.sleep(trajectory.total_duration + 0.5)
        return True
        
    def set_gripper(self, close: bool) -> None:
        if self.config.use_mock:
            time.sleep(0.2)
            return
            
        pulse_s = 0.5
        lines = ["def grip_prog():"]
        if close:
            lines.extend([
                "  set_standard_digital_out(5, False)",
                "  set_standard_digital_out(4, True)",
                f"  sleep({pulse_s:.3f})",
                "  set_standard_digital_out(4, False)",
                "  set_standard_digital_out(5, False)"
            ])
        else:
            lines.extend([
                "  set_standard_digital_out(4, False)",
                "  set_standard_digital_out(5, True)",
                f"  sleep({pulse_s:.3f})",
                "  set_standard_digital_out(4, False)",
                "  set_standard_digital_out(5, False)"
            ])
        lines.append("end")
        lines.append("grip_prog()\n")
        
        self._send_urscript("\n".join(lines))
        time.sleep(pulse_s + 0.1)
