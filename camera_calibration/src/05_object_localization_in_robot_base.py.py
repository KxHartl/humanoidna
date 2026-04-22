import cv2
import numpy as np
import pyrealsense2 as rs
import socket
from ast import literal_eval

# Load camera intrinsics and hand-eye calibration matrices
K = np.load("camera_matrix.npy")
T_cam_from_tcp = np.load("T_cam_from_tcp.npy")   # TCP -> kamera

# Define robot network connection settings
ROBOT_HOST = "192.168.40.171"  # UR5e robot IP
ROBOT_PORT = 30002

clicked_point = None

# Helper functions for robot communication and coordinate transformations
def get_robot_tcp():
    """
    Prima TCP pozu robota:
    [x, y, z, rx, ry, rz]
    """
    server = None
    client = None

    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((ROBOT_HOST, ROBOT_PORT))
        server.listen(1)
        server.settimeout(5.0)

        print("[ROBOT] Čekam TCP...")
        client, addr = server.accept()

        data = client.recv(1024)
        tcp = literal_eval(data.decode("utf-8").strip())

        return tcp

    except Exception as e:
        print("[ROBOT ERROR]", e)
        return None

    finally:
        if client:
            client.close()
        if server:
            server.close()


def tcp_to_4x4(tcp):
    x, y, z, rx, ry, rz = tcp

    rvec = np.array([rx, ry, rz], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T


def pixel_depth_to_base(u, v, depth_m, T_base_tcp):
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    # piksel + dubina -> kamera
    Xc = (u - cx) * depth_m / fx
    Yc = (v - cy) * depth_m / fy
    Zc = depth_m

    p_cam = np.array([Xc, Yc, Zc, 1.0])

    # baza -> kamera
    T_base_cam = T_base_tcp @ T_cam_from_tcp

    # kamera -> baza
    p_base = T_base_cam @ p_cam

    return p_base[:3]


def mouse_callback(event, x, y, flags, param):
    global clicked_point

    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)


# Initialize RealSense camera streams and alignment
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color)
config.enable_stream(rs.stream.depth)

profile = pipeline.start(config)

align = rs.align(rs.stream.color)

print("Klikni objekt")
print("Q / ESC = izlaz")
print("R = reset")

cv2.namedWindow("Live Robot Vision")
cv2.setMouseCallback("Live Robot Vision", mouse_callback)

# Main loop for live object selection and coordinate computation
try:
    while True:
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        display = frame.copy()

        if clicked_point is not None:
            u, v = clicked_point

            depth_m = depth_frame.get_distance(u, v)

            cv2.circle(display, (u, v), 6, (0, 0, 255), -1)

            if depth_m > 0:
                # Get current robot TCP pose
                tcp = get_robot_tcp()

                if tcp is not None:
                    T_base_tcp = tcp_to_4x4(tcp)

                    p = pixel_depth_to_base(u, v, depth_m, T_base_tcp)

                    cv2.putText(display, f"Depth={depth_m*1000:.1f} mm",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (255, 255, 0), 2)

                    cv2.putText(display, f"X={p[0]*1000:.1f} mm",
                                (20, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)

                    cv2.putText(display, f"Y={p[1]*1000:.1f} mm",
                                (20, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)

                    cv2.putText(display, f"Z={p[2]*1000:.1f} mm",
                                (20, 120), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)

                    print("TCP:", tcp)
                    print("Objekt u bazi [mm]:", p * 1000)

                else:
                    cv2.putText(display, "Robot TCP nije dostupan",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 255), 2)

            else:
                cv2.putText(display, "Nema depth podatka",
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)

        cv2.imshow("Live Robot Vision", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:
            break

        if key == ord("r"):
            clicked_point = None

finally:
    pipeline.stop()
    cv2.destroyAllWindows()