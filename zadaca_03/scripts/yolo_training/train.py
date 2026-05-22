import os
import shutil
from ultralytics import YOLO
from pathlib import Path

def train_custom_yolo():
    # Paths
    yolo_dataset_path = Path("data/datasets/yolo_seg_dataset")
    data_yaml = yolo_dataset_path / "data_seg.yaml"
    model_output_dir = Path("data/models")
    model_output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Initialize YOLOv8 segmentation model
    # We use yolov8n-seg.pt (nano) for speed, you can upgrade to 's' or 'm' if needed
    model = YOLO("yolov8n-seg.pt")

    # 2. Start training
    # Device='0' forces the first NVIDIA GPU.
    print("Starting YOLO training on GPU...")
    results = model.train(
        data=str(data_yaml.absolute()),
        epochs=100,  # Increased for better accuracy
        imgsz=640,
        batch=16,
        device='0',  # Force GPU usage
        name="fruit_seg_train",
        exist_ok=True
    )

    # 3. Copy best model to our models directory
    best_model_path = Path("runs/segment/fruit_seg_train/weights/best.pt")
    if best_model_path.exists():
        final_model_path = model_output_dir / "custom_fruit_seg.pt"
        shutil.copy(best_model_path, final_model_path)
        print(f"Success! Best model saved to {final_model_path}")
    else:
        print("Error: Could not find best.pt after training.")

if __name__ == "__main__":
    train_custom_yolo()
