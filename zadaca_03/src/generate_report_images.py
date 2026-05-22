import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rich.console import Console

console = Console()

def generate_trajectory_plots(trajectory_json_path: str, output_dir: str):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    with open(trajectory_json_path, "r") as f:
        data = json.load(f)
    
    # Check structure
    # Standard format: {"points": [{"time": 0.0, "positions": [...], "velocities": [...], "accelerations": [...]}, ...]}
    if "points" not in data:
        console.print("[red]Invalid trajectory format.[/red]")
        return

    points = data["points"]
    t = [p["t"] for p in points]
    pos = np.array([p["pos"] for p in points])
    vel = np.array([p["vel"] for p in points])
    acc = np.array([p["acc"] for p in points])
    
    # Names for 6 DOF
    names = ["X", "Y", "Z", "RX", "RY", "RZ"]
    
    # 1. Plot Position, Velocity, Acceleration for X, Y, Z
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    for i in range(3):
        axs[0].plot(t, pos[:, i], label=names[i])
        axs[1].plot(t, vel[:, i], label=names[i])
        axs[2].plot(t, acc[:, i], label=names[i])
        
    axs[0].set_ylabel("Position [m]")
    axs[0].legend()
    axs[0].set_title("Cartesian Trajectory Analysis (X, Y, Z)")
    
    axs[1].set_ylabel("Velocity [m/s]")
    axs[1].legend()
    
    axs[2].set_ylabel("Acceleration [m/s²]")
    axs[2].set_xlabel("Time [s]")
    axs[2].legend()
    
    plt.tight_layout()
    plt.savefig(output_dir / "trajectory_xyz_kinematics.png")
    plt.close()
    
    # 2. Plot 3D path
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'b-', label='Path')
    ax.scatter(pos[0, 0], pos[0, 1], pos[0, 2], color='green', s=100, label='Start')
    ax.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2], color='red', s=100, label='End')
    
    # Highlight some key points
    n_points = len(pos)
    indices = [n_points//4, n_points//2, 3*n_points//4]
    for idx in indices:
        ax.scatter(pos[idx, 0], pos[idx, 1], pos[idx, 2], color='orange', s=50)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("3D Trajectory Path")
    ax.legend()
    
    plt.savefig(output_dir / "trajectory_3d_path.png")
    plt.close()
    
    console.print(f"[green]Trajectory plots saved to {output_dir}[/green]")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory_json", help="Path to last_trajectory.json")
    parser.add_argument("output_dir", help="Where to save plots")
    args = parser.parse_args()
    
    generate_trajectory_plots(args.trajectory_json, args.output_dir)
