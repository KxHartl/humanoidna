# src_testing/ur_pose_to_matrix.py
import numpy as np
from scipy.spatial.transform import Rotation as R
try:
    import rtde_receive
except ImportError:
    print("Biblioteka 'ur_rtde' nije pronađena. Molimo instalirajte je s: pip install ur_rtde")
    sys.exit(1)

def pose6_to_matrix(pose6):
    """
    Pretvara UR pose [x, y, z, rx, ry, rz] u 4x4 homogenu matricu transformacije.
    UR koristi rotacijski vektor (axis-angle) za rx, ry, rz.
    """
    T = np.eye(4)
    # x, y, z su prva tri elementa
    T[:3, 3] = pose6[:3]
    
    # rx, ry, rz su rotacijski vektor (u radijanima)
    rotvec = pose6[3:]
    rotation_matrix = R.from_rotvec(rotvec).as_matrix()
    T[:3, :3] = rotation_matrix
    
    return T

def transform_point_cloud(pcd_points, T):
    """
    Transformira set točaka (N, 3) koristeći 4x4 matricu T.
    """
    # Pretvaranje u homogene koordinate (dodavanje jedinice na kraj svakog vektora)
    num_points = pcd_points.shape[0]
    points_homogeneous = np.hstack([pcd_points, np.ones((num_points, 1))])
    
    # Transformacija: P' = T * P
    transformed_points_homogeneous = points_homogeneous @ T.T
    
    # Povratak u 3D koordinate
    return transformed_points_homogeneous[:, :3]

def main():
    # IP adresa vašeg UR5e robota
    ROBOT_IP = "192.168.40.14" # Promijeni u stvarnu IP adresu robota
    
    print(f"Pokušavam se spojiti na robot na adresi: {ROBOT_IP}...")
    
    try:
        # 1. Spajanje na robot (RTDE Interface)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        
        # 2. Dohvaćanje trenutne TCP poze [x, y, z, rx, ry, rz]
        # Jedinice su metri i radijani
        actual_tcp_pose = rtde_r.getActualTCPPose()
        print(f"\nDohvaćena TCP poza (pose6): {actual_tcp_pose}")
        
        # 3. Izračun 4x4 matrice T_base_tcp
        T_base_tcp = pose6_to_matrix(actual_tcp_pose)
        
        print("\n4x4 Homogena transformacijska matrica (T_base_tcp):")
        print(np.array2string(T_base_tcp, precision=6, suppress_small=True))
        
        # Primjer transformacije dummy point clouda
        print("\nPrimjer transformacije point clouda...")
        dummy_pc = np.array([
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ])
        
        transformed_pc = transform_point_cloud(dummy_pc, T_base_tcp)
        print("Originalne točke (local):")
        print(dummy_pc)
        print("Transformirane točke (base):")
        print(transformed_pc)

    except Exception as e:
        print(f"Greška pri komunikaciji s robotom: {e}")

if __name__ == "__main__":
    main()
