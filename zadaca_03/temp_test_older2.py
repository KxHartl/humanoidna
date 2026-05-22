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
    
    import src.adapters.vision_adapter as va
    orig_stitch = va.VisionAdapter._stitch_and_segment
    
    def mock_stitch(*args, **kwargs):
        view_objs_all = args[1]
        print(f"\n[DEBUG] view_objs_all has {len(view_objs_all)} views")
        for i, v in enumerate(view_objs_all):
            print(f"  View {i}: {len(v)} objects detected by YOLO")
            for obj in v:
                print(f"    - {obj[0]} (conf {obj[1]}), pts: {len(obj[2].points)}")
                
        return orig_stitch(*args, **kwargs)
        
    va.VisionAdapter._stitch_and_segment = mock_stitch
    
    for run in runs:
        print(f"\n===========================================")
        print(f"Testing RUN: {run}")
        try:
            orchestrator.run_perception_offline(run, calib_tweak=0, strategy=5)
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    test_older_runs()
