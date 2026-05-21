import os
import argparse
from ultralytics import YOLO

def run_prediction(weights_path, source, conf=0.25):
    # Load model
    model = YOLO(weights_path)

    # Run inference
    # results is a list of Result objects
    results = model.predict(
        source=source,
        conf=conf,
        device=0,
        save=True,           # save results to project/name
        save_txt=True,       # save labels to *.txt
        project='output/predictions',
        name='exp',
        exist_ok=True,
        line_width=2,        # line width of the boxes
        show_labels=True,
        show_conf=True
    )
    
    print(f"Inference complete. Results saved in {os.path.join('output/predictions', 'exp')}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Predict with YOLO fruit segmentation model.")
    parser.add_argument("--weights", type=str, default="output/runs/fruit_seg_experiment/weights/best.pt", help="Path to best weights.")
    parser.add_argument("--source", type=str, required=True, help="Folder or path to images for prediction.")
    parser.add_argument("--conf", type=float, default=0.4, help="Confidence threshold.")
    args = parser.parse_args()
    
    if os.path.exists(args.weights):
        run_prediction(args.weights, args.source, args.conf)
    else:
        print(f"Error: Weights file not found at {args.weights}")
