#!/usr/bin/env python3
"""
Pick and Place Trajectory Generator
Geneirira trajektorije za autonomnu hvatanja i odlaganja objekta

Točke trajektorije:
1. HOME - početna pozicija
2. APPROACH - iznad objekta (100mm offset po Z)
3. PICK - hvatanja objekta (iz point cloud obrade)
4. APPROACH_PLACE - iznad mjesta odlaganja
5. PLACE - odlaganja objekta
"""

import numpy as np
import json
from pathlib import Path
from typing import Tuple, Dict, List
from scipy.spatial.transform import Rotation as R


class ReferencePointsManager:
    """Upravljač referentnim točkama za pick and place"""
    
    def __init__(self):
        """Inicijalizacija"""
        self.points = {
            'HOME': None,
            'APPROACH': None,
            'PICK': None,
            'APPROACH_PLACE': None,
            'PLACE': None,
        }
        
        self.approach_offset = np.array([0.0, 0.0, -0.1])  # 100mm iznad objekta
    
    def set_home(self, configuration: np.ndarray):
        """
        Postavi HOME točku (joint konfiguracija)
        
        Args:
            configuration: 6D joint vektor [q1, q2, q3, q4, q5, q6]
        """
        self.points['HOME'] = configuration
        print(f"✓ HOME postaven: {configuration}")
    
    def set_pick_from_point_cloud(self, pick_3d_coords: np.ndarray, 
                                   pick_orientation: np.ndarray = None):
        """
        Postavi PICK točku iz point cloud obrade (zadaća 3)
        
        Args:
            pick_3d_coords: 3D koordinate objekta (x, y, z) u robot sustavu
            pick_orientation: Orijentacija pri zahvaćanju [rx, ry, rz] (optional)
        """
        if pick_orientation is None:
            # Standardna orijentacija za zahvaćanje (vertikalno)
            pick_orientation = np.array([np.pi, 0, 0])  # Roll 180°
        
        self.points['PICK'] = np.concatenate([pick_3d_coords, pick_orientation])
        print(f"✓ PICK postaven: {self.points['PICK']}")
    
    def set_place(self, place_coords: np.ndarray, 
                  place_orientation: np.ndarray = None):
        """
        Postavi PLACE točku (fiksnu za odlaganja)
        
        Args:
            place_coords: 3D koordinate mjesta odlaganja
            place_orientation: Orijentacija pri odlaganju (optional)
        """
        if place_orientation is None:
            place_orientation = np.array([np.pi, 0, 0])  # Kao i PICK
        
        self.points['PLACE'] = np.concatenate([place_coords, place_orientation])
        print(f"✓ PLACE postaven: {self.points['PLACE']}")
    
    def compute_approach_points(self, offset: float = 0.1):
        """
        Izračunaj APPROACH točke s offset-om
        
        Args:
            offset: Offset po Z osi (m), default 0.1 = 100mm
        """
        if self.points['PICK'] is None:
            print("Error: PICK točka nije postavljena!")
            return
        
        if self.points['PLACE'] is None:
            print("Error: PLACE točka nije postavljena!")
            return
        
        # APPROACH - 100mm iznad PICK točke
        pick_xyz = self.points['PICK'][:3]
        pick_xyz_offset = pick_xyz + np.array([0, 0, offset])
        self.points['APPROACH'] = np.concatenate([
            pick_xyz_offset,
            self.points['PICK'][3:]  # Ista orijentacija kao PICK
        ])
        
        # APPROACH_PLACE - 100mm iznad PLACE točke
        place_xyz = self.points['PLACE'][:3]
        place_xyz_offset = place_xyz + np.array([0, 0, offset])
        self.points['APPROACH_PLACE'] = np.concatenate([
            place_xyz_offset,
            self.points['PLACE'][3:]  # Ista orijentacija kao PLACE
        ])
        
        print(f"✓ APPROACH izračunana: {self.points['APPROACH']}")
        print(f"✓ APPROACH_PLACE izračunana: {self.points['APPROACH_PLACE']}")
    
    def get_point(self, point_name: str) -> np.ndarray:
        """Dohvati točku po imenu"""
        return self.points.get(point_name)
    
    def get_all_points(self) -> Dict:
        """Dohvati sve točke"""
        return self.points
    
    def save_to_json(self, filepath: str):
        """
        Spremi točke u JSON datoteku
        
        Args:
            filepath: Putanja do datoteke
        """
        data = {}
        for key, value in self.points.items():
            if value is not None:
                data[key] = value.tolist()
            else:
                data[key] = None
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✓ Točke sprema u: {filepath}")
    
    def load_from_json(self, filepath: str):
        """
        Učitaj točke iz JSON datoteke
        
        Args:
            filepath: Putanja do datoteke
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        for key, value in data.items():
            if value is not None:
                self.points[key] = np.array(value)
        
        print(f"✓ Točke učitane iz: {filepath}")


class TrajectoryPlanner:
    """Planiranje trajektorija između referentnih točaka"""
    
    def __init__(self):
        """Inicijalizacija"""
        self.trajectories = {}
        
        # Ograničenja robota
        self.v_max = 0.5  # m/s
        self.a_max = 1.0  # m/s^2
        self.omega_max = 1.0  # rad/s
    
    def quintic_interpolation(self, start: np.ndarray, end: np.ndarray,
                             num_points: int = 100) -> np.ndarray:
        """
        Quintic (5. red polinom) interpolacija između dvije točke
        
        Args:
            start: Početna točka (6D vektor)
            end: Konačna točka (6D vektor)
            num_points: Broj diskretnih točaka
            
        Returns:
            Trajektorija (num_points x 6)
        """
        # Pomak
        delta = end - start
        
        # Vrijeme trajanja (prema ograničenjima)
        translation_distance = np.linalg.norm(delta[:3])
        rotation_distance = np.linalg.norm(delta[3:])
        
        # Jednostavna procjena vremena
        t_trans = translation_distance / self.v_max
        t_rot = rotation_distance / self.omega_max
        t_final = max(t_trans, t_rot)
        
        # Diskretizacija vremena
        t = np.linspace(0, t_final, num_points)
        tau = t / t_final
        
        # Polinom 5. reda
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        
        # Interpolacija
        trajectory = np.zeros((num_points, 6))
        for i in range(6):
            trajectory[:, i] = start[i] + delta[i] * s
        
        print(f"✓ Trajektorija generiirana ({num_points} točaka, {t_final:.2f}s)")
        
        return trajectory, t
    
    def generate_pick_and_place_trajectory(self, reference_points: Dict) -> Dict:
        """
        Generiraj kompletan pick and place slijed trajektorija
        
        Args:
            reference_points: Dict s 5 ključnih točaka
            
        Returns:
            Dict s trajektorijama između točaka
        """
        print("\n" + "="*60)
        print("GENEIRIRANJE PICK AND PLACE TRAJEKTORIJE")
        print("="*60)
        
        trajectories = {}
        
        # Segmenti:
        # 1. HOME → APPROACH
        # 2. APPROACH → PICK
        # 3. PICK (pause/zahvat)
        # 4. PICK → APPROACH_PLACE
        # 5. APPROACH_PLACE → PLACE
        # 6. PLACE (pause/odlaganja)
        # 7. PLACE → HOME
        
        segments = [
            ('HOME', 'APPROACH', 'HOME_to_APPROACH'),
            ('APPROACH', 'PICK', 'APPROACH_to_PICK'),
            ('PICK', 'APPROACH_PLACE', 'PICK_to_APPROACH_PLACE'),
            ('APPROACH_PLACE', 'PLACE', 'APPROACH_PLACE_to_PLACE'),
            ('PLACE', 'HOME', 'PLACE_to_HOME'),
        ]
        
        print("\nGeneiriranje segmenata:")
        for start_name, end_name, segment_name in segments:
            start_point = reference_points[start_name]
            end_point = reference_points[end_name]
            
            if start_point is None or end_point is None:
                print(f"⚠ Segment {segment_name}: Nedostaju točke!")
                continue
            
            print(f"\n  {segment_name}:")
            print(f"    Od: {start_name} {start_point}")
            print(f"    Do: {end_name} {end_point}")
            
            traj, time = self.quintic_interpolation(start_point, end_point)
            trajectories[segment_name] = {
                'trajectory': traj,
                'time': time,
                'duration': time[-1]
            }
        
        return trajectories
    
    def save_trajectory_to_file(self, trajectory: np.ndarray, 
                                filename: str, time_array: np.ndarray = None):
        """
        Spremi trajektoriju u datoteku (format za robota)
        
        Args:
            trajectory: Trajektorija (N x 6)
            filename: Naziv datoteke
            time_array: Vremenske vrijednosti (optional)
        """
        output = []
        
        for i, point in enumerate(trajectory):
            if time_array is not None:
                t = time_array[i]
                line = f"{t:.4f}\t{point[0]:.6f}\t{point[1]:.6f}\t{point[2]:.6f}\t{point[3]:.6f}\t{point[4]:.6f}\t{point[5]:.6f}"
            else:
                line = f"{point[0]:.6f}\t{point[1]:.6f}\t{point[2]:.6f}\t{point[3]:.6f}\t{point[4]:.6f}\t{point[5]:.6f}"
            
            output.append(line)
        
        with open(filename, 'w') as f:
            f.write('\n'.join(output))
        
        print(f"✓ Trajektorija sprema u: {filename}")


def main_example():
    """Primjer korištenja"""
    
    print("\n" + "╔" + "="*58 + "╗")
    print("║" + " "*10 + "PICK AND PLACE TRAJECTORY GENERATOR" + " "*13 + "║")
    print("╚" + "="*58 + "╝")
    
    # 1. Kreiraj manager referentnih točaka
    print("\n1. INICIJALIZACIJA REFERENTNIH TOČAKA")
    print("-"*60)
    
    pm = ReferencePointsManager()
    
    # HOME konfiguracija (sigurna pozicija)
    q_home = np.array([-1.571, -2.094, -1.571, 0.524, 1.571, 0.000])
    pm.set_home(q_home)
    
    # PICK točka (iz point cloud obrade - ZADAĆA 3)
    # Simulacija: predmeti su na (x=0.3, y=0.2, z=0.5) u robot sustavu
    pick_coords = np.array([0.3, 0.2, 0.5])
    pick_orientation = np.array([np.pi, 0, 0])  # Vertikalno
    pm.set_pick_from_point_cloud(pick_coords, pick_orientation)
    
    # PLACE točka (fiksna - gdje trebam odlagati)
    place_coords = np.array([0.2, 0.4, 0.5])
    place_orientation = np.array([np.pi, 0, 0])
    pm.set_place(place_coords, place_orientation)
    
    # Izračunaj APPROACH točke
    pm.compute_approach_points(offset=0.1)
    
    # 2. Kreiraj planer trajektorija
    print("\n2. PLANIRANJE TRAJEKTORIJA")
    print("-"*60)
    
    planner = TrajectoryPlanner()
    reference_points = pm.get_all_points()
    trajectories = planner.generate_pick_and_place_trajectory(reference_points)
    
    # 3. Spremi rezultate
    print("\n3. SPREMANJE REZULTATA")
    print("-"*60)
    
    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)
    
    # Spremi reference točke
    pm.save_to_json(str(output_dir / "reference_points.json"))
    
    # Spremi trajektorije
    for seg_name, seg_data in trajectories.items():
        filename = str(output_dir / f"trajectory_{seg_name}.txt")
        planner.save_trajectory_to_file(
            seg_data['trajectory'],
            filename,
            seg_data['time']
        )
    
    print("\n" + "="*60)
    print("PICK AND PLACE TRAJEKTORIJE GENERIRANE!")
    print("="*60)
    print("\nOutput datoteke:")
    for file in output_dir.glob("*"):
        print(f"  - {file.name}")


if __name__ == "__main__":
    main_example()
