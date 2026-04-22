# Task 2: Data Collection Guide

## Overview
Collect synchronized pairs of:
1. RGB images from RealSense D435 camera
2. Robot TCP poses from UR5e arm

## Implementation Details

### Robot Communication
- **Protocol**: TCP socket connection on port 30002
- **IP**: 192.168.40.14 (verified working ✓)
- **Data Format**: TCP pose as [x, y, z, rx, ry, rz] (in robot base frame)
- **Conversion**: Rodrigues vector → 4x4 transformation matrix

### Camera Setup
- **Camera**: Intel RealSense D435
- **Stream**: RGB color only (for this task)
- **Resolution**: 640x480 @ 30fps
- **Warmup**: 30 frames before starting capture

### Data Structure
```
/data/
  ├── image_001.png
  ├── image_002.png
  ├── image_003.png
  └── ...
  └── robot_positions.txt  (4x4 matrices stacked)
```

### Workflow

#### 1. Start the Program
```bash
python3 src/02_skupljanje_podataka.py
```

#### 2. Camera Preview
- Live feed displays with capture count
- Frame shows "SPACE=save, Q=exit"

#### 3. Capture Procedure (repeat for each pose)
```
a) Position robot in desired pose
b) Press SPACE → Image saved automatically
c) Program waits for robot TCP data
d) Robot sends transformation matrix
e) 4x4 matrix appended to robot_positions.txt
f) Counter increments
```

#### 4. Exit
- Press Q or ESC to stop
- Summary printed: "Completed. Total pairs saved: N"

### Important Notes

#### Quality Requirements
- ✓ ChArUco board must be **fully visible** in image
- ✓ Poses should be **diverse**:
  - Different distances from board (near, medium, far)
  - Different angles (front, side, top-down)
  - Different orientations
- ✓ Good lighting recommended
- ✓ Minimum 5-10 pairs for calibration (more is better)

#### Robot Data
- TCP pose comes as Euler angles (rx, ry, rz)
- Script converts to rotation matrix using cv2.Rodrigues
- Result: 4x4 homogeneous transformation matrix

#### File Management
- Images auto-numbered: image_001.png, image_002.png, ...
- If image saved but robot data fails, image is deleted
- Continues with next capture
- Resume mode: Can run program multiple times, images/poses append

### Troubleshooting

#### Robot Connection Issues
```
[GREŠKA] Robot nije poslao podatke u zadanom vremenu.
```
- Check: Robot is running and streaming on port 30002
- Check: IP address is correct (192.168.40.14)
- Check: Firewall allows TCP port 30002

#### Missing ChArUco Board
- Program doesn't validate board presence (Task 3 does)
- But images are saved regardless
- Ensure board is in frame for all captures!

#### Socket Timeout
- Default timeout: 5 seconds
- Robot must send data within this window
- If robot is slow, may need to increase timeout

### Output Validation

After running, check:

```bash
# View number of images collected
ls -la data/image_*.png | wc -l

# View robot positions file size
ls -la data/robot_positions.txt

# Check first captured image
file data/image_001.png

# Verify matrix format (4 lines per transformation)
head -10 data/robot_positions.txt
```

### File Format: robot_positions.txt

Each 4x4 transformation matrix is saved as:
```
f00 f01 f02 f03
f10 f11 f12 f13
f20 f21 f22 f23
f30 f31 f32 f33

```
(blank line between matrices)

### Next Step (Task 3)
After collecting data:
1. Verify all images contain visible ChArUco board
2. Run Task 3 to detect board poses from images
3. This generates the pose data needed for hand-eye calibration

## Status
- ✓ Robot connectivity verified
- ✓ Camera connected and working
- ✓ Directories created
- ✓ Code configured with correct IP
- **Ready for data collection!**

Run with: `python3 src/02_skupljanje_podataka.py`
