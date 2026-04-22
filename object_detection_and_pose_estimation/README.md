# RGB + PCD Object Detection and Pose Estimation

This project implements an interactive 2D/3D processing pipeline in C++ with OpenCV and PCL.

## Implemented Features

- Loads RGB image and `.pcd` cloud pairs from disk using shared filename keys.
- Verifies cloud organization and pixel-to-point alignment (`image[x,y] <-> cloud[index]`).
- Applies BGR -> HSV conversion with interactive `cv::inRange()` segmentation.
- Provides live OpenCV trackbars for:
  - dataset pair selection,
  - HSV lower/upper bounds,
  - geometric processing mode.
- Filters both image and cloud synchronously using the same binary mask.
- Shows original and filtered images side-by-side with status overlays.
- Displays filtered RGB point cloud in a PCL visualizer with coordinate axes and live status text.
- Supports mode switching between:
  - no fitting,
  - sphere fitting (RANSAC),
  - Euclidean clustering + oriented bounding boxes,
  - cylinder fitting (RANSAC with normal estimation).
- Prints estimated object centers and model parameters to the terminal.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

### Option 1: scan one root recursively

```bash
./build/rgb_pcd_object_analysis --data-root /path/to/dataset
```

### Option 2: separate image/cloud directories

```bash
./build/rgb_pcd_object_analysis --images-dir /path/to/images --clouds-dir /path/to/clouds
```

## Dataset Naming

Image and cloud files must share the same stem key.

Examples:
- `frame_0001.png`
- `frame_0001.pcd`

## Controls

- `Dataset`: switch active image-cloud pair.
- `Mode 0None1Sphere2OBB3Cyl`: select geometric method.
- `Low/High H,S,V`: tune HSV threshold.
- `q` or `Esc`: exit.
