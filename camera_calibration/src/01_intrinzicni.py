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
### DODAJTE ISPIS POTREBNIH INFORMACIJA

# Create intrinsic matrix (camera matrix)
### KONTRUIRAJTE INTRINZIČNU MATRICU
mtx = np.array([
    [, , ],
    [, , ],
    [, , ]
], dtype=np.float64)
# Create distortion coefficients array
dist = np.array(intr.coeffs, dtype=np.float64)

# Print the intrinsic matrix and distortion coefficients array
### DODAJTE ISPIS POTREBNIH INFORMACIJA

# Save intrinsic matrix and distortion coefficients
np.save("camera_matrix.npy", mtx)
np.save("dist_coeffs.npy", dist)

print("\nIntrinsic matrix saved to: camera_matrix.npy")
print("Distortion coefficients saved to: dist_coeffs.npy")

# Stop the pipeline
pipeline.stop()
