import cv2
import numpy as np
import glob
from natsort import natsorted
import os

# Load camera intrinsic parameters
mtx = np.load("camera_matrix.npy")
dist = np.load("dist_coeffs.npy")

# Define ChArUco board with real dimensions
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard((5, 7), 0.029, 0.015, aruco_dict)
parameters = cv2.aruco.DetectorParameters()

# Minimum number of detected ChArUco corners for reliable pose estimation
MIN_CHARUCO_CORNERS = 6

# Load all images from dataset folder
images = glob.glob("data/*.png")
images = natsorted(images)
print("Pronađene slike:")
for p in images:
    print(" -", p)

# Create folder for saving result files
os.makedirs("data/results", exist_ok=True)

# Open output file for saving pose results
results_path = "data/results/koordinate.txt"

# Process all images: detect ChArUco board, estimate pose, and save results
with open(results_path, "w", encoding="utf-8") as f:
    for image_path in images:
        img = cv2.imread(image_path)

        if img is None:
            print(f"[ERROR] Cannot load image: {image_path}")
            continue

        # Use original image with camera matrix and distortion coefficients
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        display_img = img.copy()

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters
        )

        if ids is None or len(ids) == 0:
            print(f"[INFO] No markers detected: {image_path}")
            output_image_path = f"data/results/{os.path.basename(image_path)}"
            cv2.imwrite(output_image_path, display_img)
            continue

        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(display_img, corners, ids)

        # Interpolate ChArUco corners
        retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board
        )

        if (
            retval is None
            or retval < MIN_CHARUCO_CORNERS
            or charuco_corners is None
            or charuco_ids is None
        ):
            print(
                f"[WARNING] Not enough ChArUco corners: {image_path} "
                f"(detected: {0 if retval is None else int(retval)})"
            )

            output_image_path = f"data/results/{os.path.basename(image_path)}"
            cv2.imwrite(output_image_path, display_img)
            continue

        # Draw detected ChArUco corners
        cv2.aruco.drawDetectedCornersCharuco(
            display_img, charuco_corners, charuco_ids
        )

        # Estimate ChArUco board pose
        success, rvec_charuco, tvec_charuco = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            board,
            mtx,
            dist,
            None,
            None
        )

        if not success:
            print(f"[WARNING] Pose estimation failed: {image_path}")
            output_image_path = f"data/results/{os.path.basename(image_path)}"
            cv2.imwrite(output_image_path, display_img)
            continue

        # Extract translation and rotation values
        tx, ty, tz = tvec_charuco.flatten()
        rx, ry, rz = rvec_charuco.flatten()

        # Print pose values
        print(
            f"{image_path}: "
            f"x={tx:.6f}, y={ty:.6f}, z={tz:.6f}, "
            f"rx={rx:.6f}, ry={ry:.6f}, rz={rz:.6f}"
        )

        # Save pose values to file
        f.write(
            f"{os.path.basename(image_path)}: "
            f"x={tx:.8f}, y={ty:.8f}, z={tz:.8f}, "
            f"rx={rx:.8f}, ry={ry:.8f}, rz={rz:.8f}\n"
        )

        # Draw coordinate axes
        cv2.drawFrameAxes(
            display_img, mtx, dist,
            rvec_charuco, tvec_charuco, 0.1
        )

        # Save processed result image
        output_image_path = f"data/results/{os.path.basename(image_path)}"
        cv2.imwrite(output_image_path, display_img)

        # Show result image
        cv2.imshow("Detection and Pose Estimation", display_img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

print("\nGotovo.")
print(f"Rezultati zapisani u: {results_path}")
print("Rezultatske slike spremljene u: data/results/")