import pyrealsense2 as rs
import numpy as np

# Start RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color)
pipeline.start(config)

# Get active profile
profile = pipeline.get_active_profile()

# Get the color stream
color_stream = profile.get_stream(rs.stream.color)

# Get the intrinsics of the color stream
intr = color_stream.as_video_stream_profile().get_intrinsics()

# Print the intrinsic parameters
print("=" * 60)
print("KALIBRACIJA KAMERE - INTRINZIČNI PARAMETRI")
print("=" * 60)
print(f"\nRezolucija: {intr.width} x {intr.height} piksela")
print(f"Žarišna duljina fx: {intr.fx} piksela")
print(f"Žarišna duljina fy: {intr.fy} piksela")
print(f"Optičko središte cx: {intr.ppx} piksela")
print(f"Optičko središte cy: {intr.ppy} piksela")
print(f"\nDistorzijski koeficijenti:")
print(f"  k1: {intr.coeffs[0]}")
print(f"  k2: {intr.coeffs[1]}")
print(f"  p1: {intr.coeffs[2]}")
print(f"  p2: {intr.coeffs[3]}")
print(f"  k3: {intr.coeffs[4]}")

# Create intrinsic matrix (camera matrix)
mtx = np.array([
    [intr.fx, 0, intr.ppx],
    [0, intr.fy, intr.ppy],
    [0, 0, 1]
], dtype=np.float64)
# Create distortion coefficients array
dist = np.array(intr.coeffs, dtype=np.float64)

# Print the intrinsic matrix and distortion coefficients array
print(f"\nIntrinzična matrica K (3x3):")
print(mtx)
print(f"\nVektor distorzije:")
print(dist)

# Save intrinsic matrix and distortion coefficients
np.save("camera_matrix.npy", mtx)
np.save("dist_coeffs.npy", dist)

print("\nIntrinsic matrix saved to: camera_matrix.npy")
print("Distortion coefficients saved to: dist_coeffs.npy")

# Stop the pipeline
pipeline.stop()
