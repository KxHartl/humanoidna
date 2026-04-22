import cv2
import numpy as np
import glob
import os
from natsort import natsorted

# Define file paths, board dimensions, and detection settings
IMAGES_GLOB = "data/calibration_images/*.png"
ROBOT_FILE = "data/calibration_images/robot_positions.txt"

SQUARE_LENGTH = 0.029   # m
MARKER_LENGTH = 0.015   # m
MIN_CHARUCO_CORNERS = 6

# Helper functions for matrix operations and pose detection
def load_4x4_matrices(filepath):
    matrices = []
    current = []

    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()

            if not line:
                if current:
                    M = np.array(current, dtype=np.float64)
                    if M.shape == (4, 4):
                        matrices.append(M)
                    current = []
                continue

            row = [float(x) for x in line.split()]
            current.append(row)

    if current:
        M = np.array(current, dtype=np.float64)
        if M.shape == (4, 4):
            matrices.append(M)

    return matrices


def make_4x4(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def invert_4x4(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

# Detect ChArUco board and return target-to-camera transformation matrix
def detect_target_to_camera(image_path, mtx, dist, board, charuco_detector):
    img = cv2.imread(image_path)
    if img is None:
        return None

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect ChArUco using new API
    charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray)

    if charuco_ids is None or len(charuco_ids) < MIN_CHARUCO_CORNERS:
        return None

    # Use solvePnP for pose estimation
    obj_points = []
    img_points = []
    
    for i, charuco_id in enumerate(charuco_ids.flatten()):
        obj_point = board.getChessboardCorners()[charuco_id]
        img_point = charuco_corners[i].flatten()
        obj_points.append(obj_point)
        img_points.append(img_point)
    
    obj_points = np.array(obj_points, dtype=np.float32)
    img_points = np.array(img_points, dtype=np.float32)
    
    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        mtx,
        dist,
        useExtrinsicGuess=False,
        flags=cv2.SOLVEPNP_EPNP
    )

    if not success:
        return None

    R, _ = cv2.Rodrigues(rvec)
    T_target_cam = make_4x4(R, tvec.reshape(3))
    return T_target_cam


def print_matrix(name, T, unit="m"):
    print(f"\n{name}:")
    np.set_printoptions(suppress=True, precision=6)
    print(T)
    print(f"Translacija [{unit}]: {T[:3, 3]}")


# Load camera parameters, images, and robot pose data
mtx = np.load("camera_matrix.npy")
dist = np.load("dist_coeffs.npy")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard((5, 7), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)
charuco_params = cv2.aruco.CharucoParameters()
charuco_detector = cv2.aruco.CharucoDetector(board, charuco_params)

image_paths = natsorted(glob.glob(IMAGES_GLOB))
robot_matrices = load_4x4_matrices(ROBOT_FILE)

print(f"Broj slika: {len(image_paths)}")
print(f"Broj robot matrica: {len(robot_matrices)}")

# Collect valid robot and camera pose pairs for hand-eye calibration
R_gripper2base = []
t_gripper2base = []

R_target2cam = []
t_target2cam = []

valid_pairs = 0

# Match image data with robot poses and collect valid pairs for hand-eye calibration
for i, image_path in enumerate(image_paths):
    if i >= len(robot_matrices):
        break

    # TCP pose in robot base coordinate system (^base T_tcp)
    T_base_tcp = robot_matrices[i]

    # Detect target (ChArUco board) pose in camera coordinate system
    T_target_cam = detect_target_to_camera(
        image_path, mtx, dist, board, charuco_detector
    )

    if T_target_cam is None:
        print(f"[SKIP] No valid pose for: {image_path}")
        continue

    # OpenCV calibrateHandEye expects:
    # gripper2base = ^base T_gripper
    # target2cam   = ^cam T_target ? OpenCV uses the name target2cam,
    # and in practice we use R,t values returned by
    # estimatePoseCharucoBoard describing target pose in camera frame.
    #
    # estimatePoseCharucoBoard returns:
    # target (board) pose in camera coordinate system,
    # i.e. ^cam T_target
    #
    # This is exactly what is needed
    # for the target2cam input lists.

    R_gripper2base.append(T_base_tcp[:3, :3])
    t_gripper2base.append(T_base_tcp[:3, 3].reshape(3, 1))

    R_target2cam.append(T_target_cam[:3, :3])
    t_target2cam.append(T_target_cam[:3, 3].reshape(3, 1))

    valid_pairs += 1

print(f"\nValjanih parova za hand-eye: {valid_pairs}")

if valid_pairs < 5:
    raise RuntimeError("Premalo valjanih parova. Trebaš barem nekoliko dobrih snimki.")

# Perform hand-eye calibration and compute TCP-to-camera transformation
# OpenCV returns TCP -> Camera transformation
R_tcp2cam, t_tcp2cam = cv2.calibrateHandEye(
    R_gripper2base=R_gripper2base,
    t_gripper2base=t_gripper2base,
    R_target2cam=R_target2cam,
    t_target2cam=t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

# TCP -> Camera
T_cam_from_tcp = make_4x4(R_tcp2cam, t_tcp2cam.reshape(3))

# Camera -> TCP
T_tcp_from_cam = invert_4x4(T_cam_from_tcp)

print_matrix("T_cam_from_tcp (TCP -> CAMERA)", T_cam_from_tcp)
print_matrix("T_tcp_from_cam (CAMERA -> TCP)", T_tcp_from_cam)

np.save("T_cam_from_tcp.npy", T_cam_from_tcp)
np.save("T_tcp_from_cam.npy", T_tcp_from_cam)

print("\nSpremljeno:")
print(" - T_tcp_from_cam.npy")
print(" - T_cam_from_tcp.npy")