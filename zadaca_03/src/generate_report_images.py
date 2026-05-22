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
    
    if "points" not in data:
        console.print("[red]Invalid trajectory format.[/red]")
        return

    points = data["points"]
    t = [p["t"] for p in points]
    pos = np.array([p["pos"] for p in points])
    vel = np.array([p["vel"] for p in points])
    acc = np.array([p["acc"] for p in points])
    
    names = ["X", "Y", "Z", "RX", "RY", "RZ"]
    
    # 1. Find segments
    v_norm = np.linalg.norm(vel, axis=1)
    zero_indices = np.where(v_norm < 1e-5)[0]
    
    waypoints = []
    if len(zero_indices) > 0:
        waypoints.append(zero_indices[0])
        for i in range(1, len(zero_indices)):
            if zero_indices[i] - zero_indices[i-1] > 1:
                waypoints.append(zero_indices[i])
        if zero_indices[-1] != waypoints[-1]:
            waypoints.append(zero_indices[-1])

    # 2. Plot Kinematics for Segment 1 (home to approach pick)
    if len(waypoints) >= 2:
        start_idx = waypoints[0]
        end_idx = waypoints[1]
        
        fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
        
        t_seg = t[start_idx:end_idx+1]
        pos_seg = pos[start_idx:end_idx+1]
        vel_seg = vel[start_idx:end_idx+1]
        acc_seg = acc[start_idx:end_idx+1]
        
        for i in range(3):
            axs[0].plot(t_seg, pos_seg[:, i], label=names[i])
            axs[1].plot(t_seg, vel_seg[:, i], label=names[i])
            axs[2].plot(t_seg, acc_seg[:, i], label=names[i])
            
        axs[0].set_ylabel("Položaj [m]")
        axs[0].legend()
        axs[0].set_title("Međutrajektorija 1: home0 -> iznad pick1")
        axs[0].grid(True)
        
        axs[1].set_ylabel("Brzina [m/s]")
        axs[1].legend()
        axs[1].grid(True)
        
        axs[2].set_ylabel("Ubrzanje [m/s²]")
        axs[2].set_xlabel("Vrijeme [s]")
        axs[2].legend()
        axs[2].grid(True)
        
        plt.tight_layout()
        plt.savefig(output_dir / "trajectory_kinematics.png")
        plt.close()

    # 3. Plot 3D path with specific keypoints
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'gray', linestyle='--', label='Cijela trajektorija', alpha=0.7)
    
    # We expect waypoints to be: [home0, iznad pick1, pick2, iznad pick1, iznad place3, place4, iznad place3, home0]
    # Highlight the 5 unique ones:
    keypoints = []
    labels = []
    colors = []
    
    if len(waypoints) >= 1:
        keypoints.append(waypoints[0])
        labels.append("home0")
        colors.append("blue")
    if len(waypoints) >= 2:
        keypoints.append(waypoints[1])
        labels.append("iznad pick1")
        colors.append("orange")
    if len(waypoints) >= 3:
        keypoints.append(waypoints[2])
        labels.append("pick2")
        colors.append("green")
    if len(waypoints) >= 5:
        keypoints.append(waypoints[4])
        labels.append("iznad place3")
        colors.append("purple")
    if len(waypoints) >= 6:
        keypoints.append(waypoints[5])
        labels.append("place4")
        colors.append("red")
        
    for idx, label, color in zip(keypoints, labels, colors):
        ax.scatter(pos[idx, 0], pos[idx, 1], pos[idx, 2], color=color, s=100, label=label, edgecolor='black', zorder=5)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("3D Trajektorija s 5 glavnih poza")
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
