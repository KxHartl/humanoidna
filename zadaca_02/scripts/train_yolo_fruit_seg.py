import os
from ultralytics import YOLO

def train_fruit_seg():
    # Load a model
    model = YOLO('yolov8n-seg.pt')  # load a pretrained model (recommended for training)

    # Path to the dataset configuration file
    data_path = os.path.abspath('datasets/Humanoidna_svo_voce_02.yolov8-obb/data.yaml')
    
    # Project and name for saving runs
    project_dir = os.path.abspath('output/runs')
    name_dir = 'fruit_seg_experiment'

    # Train the model
    results = model.train(
        data=data_path,
        epochs=100,          # Student assignment: 50 epochs is usually enough for nano
        imgsz=640,          # Standard YOLO resolution
        batch=16,           # Reasonable batch size for a consumer GPU
        device=0,           # Use NVIDIA GPU
        project=project_dir,
        name=name_dir,
        exist_ok=True,
        save=True,
        plots=True,
        # Augmentations (default YOLO ones are good)
        lr0=0.01,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3.0,
    )
    
    print(f"Training complete. Weights saved to {os.path.join(project_dir, name_dir, 'weights')}")

if __name__ == "__main__":
    train_fruit_seg()
