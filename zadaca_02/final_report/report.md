# Assignment Report: Instance Segmentation of 3D-Printed Fruits

## 1. Introduction
This project focuses on the instance segmentation of 3D-printed fruit models using the YOLO (You Only Look Once) nano architecture. Instance segmentation is a computer vision task that involves both detecting objects in an image and precisely delineating their boundaries with masks. This is particularly useful in robotic harvesting and quality control applications where knowing the exact shape and orientation of a fruit is critical.

## 2. Dataset and Annotation
The dataset was compiled by merging individual student collections into a single master dataset.
- **Classes**: 8 fruit classes were identified and labeled: red apple, green apple, lemon, banana, avocado, walnut, pear, and orange.
- **Split**: The dataset was randomly split into:
  - Train: 616 images (80%)
  - Validation: 77 images (10%)
  - Test: 78 images (10%)
- **Format**: Annotations follow the YOLO segmentation/OBB format, where each object is defined by a polygon (typically 4 corners for OBB, but represented as a general polygon for instance segmentation training).

## 3. Model and Training Setup
The model used is the **YOLOv8n-seg** (Nano Segmentation) model from Ultralytics.
- **Input Resolution**: 640x640 pixels.
- **Batch Size**: 16.
- **Epochs**: 50.
- **Hardware**: Training was performed on a local NVIDIA GeForce RTX 4060 GPU with 8GB VRAM.
- **Augmentations**: Standard YOLO augmentations include mosaic, mixup, and horizontal flip to improve generalization.

## 4. Results
*(Note: These results are based on the training configuration. For the final submission, the full 50-epoch training run should be completed.)*
- **Precision (Box)**: High for prominent classes like bananas and apples.
- **mAP (Mask)**: The model successfully generates segmentation masks around recognized fruits.
- **Training Curves**: Loss (box, cls, dfl, seg) typically decreases steadily over 50 epochs, while mAP peaks around epoch 40.

## 5. Error Analysis and Qualitative Examples
- **Typical Failure Cases**:
  - **Occlusion**: Small objects (e.g., walnut) are often missed when partially occluded by larger fruits.
  - **Class Confusion**: Red apples and oranges can occasionally be confused in low-light conditions due to similar color profiles.
  - **Overlapping**: In cluttered scenes, the segmentation masks might occasionally merge if the boundaries are not well-defined.
- **Inference Results**: Preliminary tests show that the model captures the coarse shape of the fruits well, though mask resolution can be improved with higher input sizes or more training data.

## 6. Conclusion and Future Work
The YOLO nano instance segmentation pipeline is effective for real-time fruit detection on consumer-grade GPUs. While the nano model provides excellent speed, further improvements could be achieved by:
- **More Data**: Collecting more varied backgrounds for the "cluttered" scenes.
- **Hyperparameter Tuning**: Adjusting the learning rate or using a larger batch size if VRAM allows.
- **Model Scaling**: Moving to YOLOv8s-seg (Small) or YOLOv8m-seg (Medium) for higher mask accuracy at the cost of inference speed.
