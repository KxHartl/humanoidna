import os
import argparse
from ultralytics import YOLO

def evaluate_model(weights_path):
    # Load the trained model
    model = YOLO(weights_path)

    # Validate on the validation set (configured in data.yaml)
    # Note: data.yaml is typically referenced in the weights metadata, 
    # but we can specify it explicitly.
    data_path = os.path.abspath('datasets/Humanoidna_svo_voce_02.yolov8-obb/data.yaml')
    
    metrics = model.val(
        data=data_path,
        split='test',  # Evaluate specifically on the test split
        device=0,
        project='output/evaluate',
        name='test_results',
        exist_ok=True,
        save_json=True,
        plots=True
    )
    
    print("\nMain Metrics (Metrics/Precision, Metrics/Recall, Metrics/mAP50, Metrics/mAP50-95):")
    print(f"Boxing - mAP50: {metrics.box.map50:.4f}, mAP50-95: {metrics.box.map:.4f}")
    print(f"Segmentation - mAP50: {metrics.seg.map50:.4f}, mAP50-95: {metrics.seg.map:.4f}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate YOLO fruit segmentation model.")
    parser.add_argument("--weights", type=str, default="output/runs/fruit_seg_experiment/weights/best.pt", help="Path to trained weights.")
    args = parser.parse_args()
    
    if os.path.exists(args.weights):
        evaluate_model(args.weights)
    else:
        print(f"Error: Weights file not found at {args.weights}")
