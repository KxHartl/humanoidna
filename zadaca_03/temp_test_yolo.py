import sys
import os
import cv2
from ultralytics import YOLO

def test_yolo_on_raw_data():
    model = YOLO("data/models/custom_fruit_seg.pt")
    
    runs = [
        r"D:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03\data\raw\outputs\run_20260521_030300",
        r"D:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03\data\raw\outputs\run_20260521_030359",
        r"D:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_03\data\raw\outputs\run_20260522_145403"
    ]
    
    for run in runs:
        print(f"\nTesting RUN: {run}")
        for i in range(3):
            img_path = os.path.join(run, f"view_{i}_color.png")
            if not os.path.exists(img_path):
                print(f"Missing {img_path}")
                continue
                
            color = cv2.imread(img_path)
            color_upright = cv2.rotate(color, cv2.ROTATE_180)
            
            # Run with very low confidence
            res = model(color_upright, conf=0.01, verbose=False)
            
            if len(res) > 0 and res[0].masks is not None:
                boxes = res[0].boxes
                names = res[0].names
                print(f"  View {i}: Found {len(boxes)} objects")
                for j in range(len(boxes)):
                    print(f"    - {names[int(boxes.cls[j])]} (conf {float(boxes.conf[j]):.3f})")
            else:
                print(f"  View {i}: Found 0 objects!")

if __name__ == "__main__":
    test_yolo_on_raw_data()
