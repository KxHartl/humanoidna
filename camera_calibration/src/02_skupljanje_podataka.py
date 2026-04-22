import cv2
import numpy as np
import os
import socket
from ast import literal_eval
import pyrealsense2 as rs

# Settings
SAVE_FOLDER = "data/calibration_images"
POSITIONS_FILE = os.path.join(SAVE_FOLDER, "robot_positions.txt")
IMAGE_PREFIX = "image"
IMAGE_FORMAT = ".png"

ROBOT_HOST = "192.168.40.171"  # UR5e robot IP
ROBOT_PORT = 30002

# Function definition

# Convert TCP pose to 4x4 transformation matrix
def tcp_to_4x4(tcp):
    x, y, z, rx, ry, rz = tcp

    rvec = np.array([rx, ry, rz], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T

# Save 4x4 transformation matrix to text file
def save_4x4(filepath, matrix):
    with open(filepath, "a", encoding="utf-8") as f:
        for row in matrix:
            f.write(" ".join(f"{value:.8f}" for value in row) + "\n")
        f.write("\n")

# Receive TCP pose data from robot via socket connection
def get_robot_tcp():
    server_socket = None
    client_socket = None

    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((ROBOT_HOST, ROBOT_PORT))
        server_socket.listen(1)
        server_socket.settimeout(5.0)

        print("[ROBOT] Čekam podatke od robota...")
        client_socket, addr = server_socket.accept()
        data = client_socket.recv(1024)

        tcp = literal_eval(data.decode("utf-8").strip())
        print(f"[ROBOT] Primljeno od {addr}: {tcp}")

        return tcp

    except socket.timeout:
        print("[GREŠKA] Robot nije poslao podatke u zadanom vremenu.")
        return None

    except Exception as e:
        print(f"[GREŠKA] Socket problem: {e}")
        return None

    finally:
        if client_socket is not None:
            client_socket.close()
        if server_socket is not None:
            server_socket.close()

# Initialize save folder, image counter, and camera stream
os.makedirs(SAVE_FOLDER, exist_ok=True)

existing_images = [
    f for f in os.listdir(SAVE_FOLDER)
    if f.startswith(IMAGE_PREFIX) and f.endswith(IMAGE_FORMAT)
]
counter = len(existing_images) + 1

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color)

pipeline.start(config)

profile = pipeline.get_active_profile()
stream = profile.get_stream(rs.stream.color).as_video_stream_profile()

width = stream.width()
height = stream.height()
fps = stream.fps()

print(f"[KAMERA] Rezolucija: {width}x{height} @ {fps} fps")

print("[KAMERA] Zagrijavanje kamere...")
for _ in range(30):
    pipeline.wait_for_frames()
print("[KAMERA] Kamera spremna.")

print("\n" + "=" * 55)
print("  SNIMANJE SLIKA + POZICIJA ROBOTA")
print("=" * 55)
print("  SPACE  -> spremi sliku i robot pozu")
print("  Q / ESC -> izlaz")
print(f"  Slike: {SAVE_FOLDER}/")
print(f"  Pozicije: {POSITIONS_FILE}")
print("=" * 55 + "\n")

# Main program loop for image capture and robot data collection
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        preview = frame.copy()

        cv2.rectangle(preview, (0, 0), (520, 60), (0, 0, 0), -1)
        cv2.putText(
            preview,
            f"Snimljeno: {counter - 1} | SPACE = spremi | Q = izlaz",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

        cv2.imshow("Snimanje podataka", preview)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):
            img_name = f"{IMAGE_PREFIX}_{counter:03d}{IMAGE_FORMAT}"
            img_path = os.path.join(SAVE_FOLDER, img_name)

            # Spremi RAW sliku
            cv2.imwrite(img_path, frame)
            print(f"[SLIKA] Spremljeno: {img_path}")

            # Dohvati robot TCP
            tcp = get_robot_tcp()

            if tcp is None:
                if os.path.exists(img_path):
                    os.remove(img_path)
                print(f"[GREŠKA] Nema robot poze — slika {img_name} obrisana.")
                continue

            # Spremi 4x4 matricu
            T = tcp_to_4x4(tcp)
            save_4x4(POSITIONS_FILE, T)

            print(f"[MATRICA] Spremljeno u: {POSITIONS_FILE}")
            print(T)
            print(f"[OK] Par #{counter} uspješno spremljen.\n")

            counter += 1

        elif key == ord("q") or key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print(f"\nZavršeno. Ukupno spremljenih parova: {counter - 1}")