import sys
import os
import numpy as np
import open3d as o3d

sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from src.main import setup_components

def test_older_runs():
    runs = [
        r"D:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03\data\raw\outputs\run_20260521_030300",
    ]
    
    orchestrator = setup_components()
    orchestrator.robot.config.use_mock = True
    
    for run in runs:
        print(f"\n===========================================")
        print(f"Testing RUN: {run}")
        try:
            orchestrator.run_perception_offline(run, calib_tweak=0, strategy=5)
            print(f"Found {len(orchestrator.ctx.segmented_objects)} objects.")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    test_older_runs()
