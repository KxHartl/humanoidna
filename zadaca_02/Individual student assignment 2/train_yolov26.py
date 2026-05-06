from ultralytics import YOLO
import os

def train():
    # Load the specified YOLOv26nano segmentation model
    model_path = r"yolo26n-seg.pt" # Relying on ultralytics to auto-download/find it if in PATH
    print(f"Loading model: {model_path}")
    model = YOLO(model_path)

    # Use the absolute path to the data.yaml
    data_path = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\datasets\humanoidna_voce_sve_v3.yolov8\data_abs.yaml"
    
    # Train the model
    # Saving runs inside the new directory
    project_dir = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\Individual student assignment 2\runs"
    
    results = model.train(
        data=data_path,
        epochs=100,
        imgsz=640,
        device=0,  # use GPU 0
        project=project_dir,
        name="yolo26n_seg_fruit"
    )
    
    print("Training finished.")

if __name__ == "__main__":
    train()
