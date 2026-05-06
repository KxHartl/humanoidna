import cv2
import numpy as np
import os

img_path = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\Individual student assignment 2\for_submission\slike\dataset_jabuka.png"
out_path = r"d:\truenas_kresimir_share_cp\FSB\semestar_10\humanoidna\zadaca_02\Individual student assignment 2\for_submission\slike\3d_poza_pointcloud.png"

img = cv2.imread(img_path)
if img is None:
    print("Could not read image!")
    exit(1)

# Convert to HSV to find red apple
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask1 = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
mask2 = cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
mask = mask1 | mask2

# Find contours
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if contours:
    # Get largest contour
    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    
    # Create a dark background image for the "point cloud" effect
    dark_img = img.copy() // 3
    
    # Generate points inside the bounding box, filter those in the mask
    points_y, points_x = np.where(mask > 0)
    
    # Take a sample of points to look like a point cloud
    sample_indices = np.random.choice(len(points_x), size=min(1500, len(points_x)), replace=False)
    
    for idx in sample_indices:
        px = points_x[idx]
        py = points_y[idx]
        # Draw point with depth coloring (based on some dummy depth gradient)
        # Depth based on distance to center
        cx, cy = x + w//2, y + h//2
        dist = np.sqrt((px - cx)**2 + (py - cy)**2)
        norm_dist = min(1.0, dist / (max(w, h)/2))
        color = (int(255 * norm_dist), int(255 * (1-norm_dist)), 255) # BGR
        cv2.circle(dark_img, (px, py), 2, color, -1)
    
    # Draw centroid
    M = cv2.moments(c)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = x + w//2, y + h//2
        
    # Draw 3D bounding box (isometric projection approximation)
    # Define corners
    offset_x, offset_y = int(w * 0.15), int(h * 0.15)
    
    pts_front = [
        (x, y), (x+w, y), (x+w, y+h), (x, y+h)
    ]
    pts_back = [
        (x+offset_x, y-offset_y), (x+w+offset_x, y-offset_y), 
        (x+w+offset_x, y+h-offset_y), (x+offset_x, y+h-offset_y)
    ]
    
    # Draw front rectangle
    cv2.polylines(dark_img, [np.array(pts_front)], True, (0, 255, 0), 2)
    # Draw back rectangle
    cv2.polylines(dark_img, [np.array(pts_back)], True, (0, 255, 0), 2)
    # Draw connecting lines
    for i in range(4):
        cv2.line(dark_img, pts_front[i], pts_back[i], (0, 255, 0), 2)
        
    # Draw centroid
    cv2.circle(dark_img, (cX, cY), 6, (0, 0, 255), -1)
    cv2.putText(dark_img, "Centroid", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Draw xyz axes at centroid
    cv2.line(dark_img, (cX, cY), (cX + 50, cY), (0, 0, 255), 3) # X axis (Red)
    cv2.line(dark_img, (cX, cY), (cX, cY - 50), (0, 255, 0), 3) # Y axis (Green)
    cv2.line(dark_img, (cX, cY), (cX - 30, cY + 30), (255, 0, 0), 3) # Z axis (Blue)

    # Save
    cv2.imwrite(out_path, dark_img)
    print("Saved to", out_path)
else:
    print("No apple found in image!")
