#!/usr/bin/env python3
"""
Integration Script - Spajanje rezultata Zadaće 3 s Pick and Place

Ova skripta čita 3D koordinate objekta iz rezultata Zadaće 3
(point cloud obrada i 3D lokalizacija) i koristi ih za geneiriranje
pick and place trajektorija.

Tijek:
1. Učitaj PICK koordinate iz point_cloud_results.json
2. Definiraj HOME, PLACE točke
3. Generiraj sve 5 segmentnih trajektorija
4. Spremi rezultate
"""

import json
import numpy as np
from pathlib import Path
from typing import Dict, Optional
import sys


class PickAndPlaceIntegration:
    """Integracija između Zadaće 3 i Pick and Place"""
    
    def __init__(self, task3_output_dir: str = "../for_zadaca_03/output",
                 pp_reference_dir: str = "reference_points"):
        """
        Inicijalizacija
        
        Args:
            task3_output_dir: Direktorij s rezultatima zadaće 3
            pp_reference_dir: Direktorij za reference točke
        """
        self.task3_dir = Path(task3_output_dir)
        self.pp_ref_dir = Path(pp_reference_dir)
        self.pp_ref_dir.mkdir(exist_ok=True)
        
        self.pick_coordinates = None
        self.reference_points = {}
    
    def load_pick_from_task3(self, results_file: str = "object_detection_results.json") -> bool:
        """
        Učitaj PICK koordinate iz rezultata zadaće 3
        
        Args:
            results_file: Naziv datoteke s rezultatima
            
        Returns:
            bool: True ako je uspješno učitano
        """
        filepath = self.task3_dir / results_file
        
        if not filepath.exists():
            print(f"✗ Datoteka nije pronađena: {filepath}")
            print("  Tražim alternativne nazive...")
            
            # Tražim alternative
            for potential_file in self.task3_dir.glob("*results*.json"):
                print(f"  Pronašao: {potential_file.name}")
                filepath = potential_file
                break
            
            if not filepath.exists():
                print("✗ Nisam pronašao datoteku s rezultatima!")
                return False
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Pronađi centroid robotskog sustava
            if 'centroid_robot' in data:
                self.pick_coordinates = np.array(data['centroid_robot'])
                print(f"✓ PICK koordinate učitane iz {filepath.name}")
                print(f"  Pozicija: {self.pick_coordinates[:3]}")
                print(f"  Orijentacija: {self.pick_coordinates[3:]}")
                return True
            else:
                print("✗ 'centroid_robot' nije pronađen u datoteci!")
                return False
        
        except json.JSONDecodeError:
            print(f"✗ Greška pri čitanju JSON datoteke: {filepath}")
            return False
        except Exception as e:
            print(f"✗ Greška: {e}")
            return False
    
    def set_home_configuration(self, q_home: np.ndarray):
        """
        Postavi HOME konfiguraciju
        
        Args:
            q_home: Joint konfiguracija [q1, q2, q3, q4, q5, q6]
        """
        self.reference_points['HOME'] = {
            'type': 'joint',
            'values': q_home.tolist(),
            'description': 'Početna sigurna pozicija'
        }
        print(f"✓ HOME postavljen: {q_home}")
    
    def set_place_location(self, place_3d: np.ndarray, 
                          place_orientation: np.ndarray = None):
        """
        Postavi PLACE lokaciju
        
        Args:
            place_3d: 3D pozicija mjesta odlaganja
            place_orientation: Orijentacija pri odlaganju
        """
        if place_orientation is None:
            place_orientation = np.array([np.pi, 0, 0])
        
        self.reference_points['PLACE'] = {
            'type': 'task_space',
            'position': place_3d.tolist(),
            'orientation': place_orientation.tolist(),
            'description': 'Gdje trebam odlagati objekt'
        }
        print(f"✓ PLACE postavljen: {place_3d}")
    
    def compute_all_reference_points(self, approach_offset: float = 0.1) -> Dict:
        """
        Izračunaj sve 5 referentnih točaka
        
        Args:
            approach_offset: Offset u metrima (default 0.1 = 100mm)
            
        Returns:
            Dict sa svim točkama
        """
        if self.pick_coordinates is None:
            print("✗ PICK koordinate nisu učitane!")
            return {}
        
        print("\nIzračunam sve referentne točke...")
        
        # PICK je već učitan
        self.reference_points['PICK'] = {
            'type': 'task_space',
            'position': self.pick_coordinates[:3].tolist(),
            'orientation': self.pick_coordinates[3:].tolist(),
            'description': 'Hvatanja objekta (iz Zadaće 3)',
            'source': 'Point cloud 3D localization'
        }
        
        # APPROACH - 100mm iznad PICK-a
        approach_pos = self.pick_coordinates[:3] + np.array([0, 0, approach_offset])
        self.reference_points['APPROACH'] = {
            'type': 'task_space',
            'position': approach_pos.tolist(),
            'orientation': self.pick_coordinates[3:].tolist(),
            'description': f'Iznad objekta ({approach_offset*1000:.0f}mm)'
        }
        
        # APPROACH_PLACE - 100mm iznad PLACE-a
        if 'PLACE' in self.reference_points:
            place_data = self.reference_points['PLACE']
            place_pos = np.array(place_data['position'])
            approach_place_pos = place_pos + np.array([0, 0, approach_offset])
            
            self.reference_points['APPROACH_PLACE'] = {
                'type': 'task_space',
                'position': approach_place_pos.tolist(),
                'orientation': place_data['orientation'],
                'description': f'Iznad mjesta odlaganja ({approach_offset*1000:.0f}mm)'
            }
        
        print("✓ Sve referentne točke izračunane!")
        return self.reference_points
    
    def generate_report(self) -> str:
        """
        Generiraj izvještaj integracije
        
        Returns:
            str: Formatiran izvještaj
        """
        report = []
        report.append("═" * 70)
        report.append("PICK AND PLACE INTEGRACIJA - IZVJEŠTAJ")
        report.append("═" * 70)
        report.append("")
        report.append("UČITANE TOČKE:")
        report.append("-" * 70)
        
        for point_name, point_data in self.reference_points.items():
            report.append(f"\n{point_name}:")
            if point_data['type'] == 'joint':
                report.append(f"  Tip: Joint konfiguracija")
                report.append(f"  Vrijednosti: {point_data['values']}")
            else:
                pos = point_data['position']
                ori = point_data['orientation']
                report.append(f"  Tip: Task space")
                report.append(f"  Pozicija: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
                report.append(f"  Orijentacija: ({ori[0]:.4f}, {ori[1]:.4f}, {ori[2]:.4f})")
            
            if 'description' in point_data:
                report.append(f"  Opis: {point_data['description']}")
        
        report.append("\n" + "═" * 70)
        
        return "\n".join(report)
    
    def save_reference_points(self, output_file: str = "pick_and_place_points.json"):
        """
        Spremi sve reference točke
        
        Args:
            output_file: Naziv izlazne datoteke
        """
        filepath = self.pp_ref_dir / output_file
        
        output_data = {
            'timestamp': str(Path.cwd()),
            'source': 'Integracija Zadaće 3 s Pick and Place',
            'reference_points': self.reference_points
        }
        
        with open(filepath, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        print(f"✓ Reference točke sprema u: {filepath}")


def main_example():
    """Primjer korištenja integracije"""
    
    print("\n" + "╔" + "═"*68 + "╗")
    print("║" + " "*15 + "INTEGRACIJA ZADAĆE 3 S PICK AND PLACE" + " "*15 + "║")
    print("╚" + "═"*68 + "╝")
    
    # 1. Inicijalizacija
    print("\n1. INICIJALIZACIJA")
    print("-"*70)
    
    integration = PickAndPlaceIntegration()
    
    # 2. Učitaj PICK iz Zadaće 3
    print("\n2. UČITAVANJE PICK KOORDINATA IZ ZADAĆE 3")
    print("-"*70)
    
    if not integration.load_pick_from_task3():
        print("\n⚠ UPOZORENJE: Nisam pronašao rezultate Zadaće 3")
        print("Koristim primjer podatke za demonstraciju...")
        
        # Primjer
        integration.pick_coordinates = np.array([0.3, 0.2, 0.5, np.pi, 0, 0])
    
    # 3. Definiraj HOME i PLACE
    print("\n3. DEFINIRANJE REFERENTNIH TOČAKA")
    print("-"*70)
    
    # HOME - sigurna pozicija
    q_home = np.array([-1.571, -2.094, -1.571, 0.524, 1.571, 0.000])
    integration.set_home_configuration(q_home)
    
    # PLACE - gdje trebam odlagati
    place_pos = np.array([0.2, 0.4, 0.5])
    integration.set_place_location(place_pos)
    
    # 4. Izračunaj sve točke
    print("\n4. IZRAČUN SVIH REFERENTNIH TOČAKA")
    print("-"*70)
    
    integration.compute_all_reference_points(approach_offset=0.1)
    
    # 5. Generiraj i ispiši izvještaj
    print("\n5. IZVJEŠTAJ")
    print("-"*70)
    
    report = integration.generate_report()
    print(report)
    
    # 6. Spremi
    print("\n6. SPREMANJE REZULTATA")
    print("-"*70)
    
    integration.save_reference_points("integrated_pick_and_place_points.json")
    
    print("\n✓ Integracija završena!")
    print("\nSljedeći korak:")
    print("  1. Koristi reference_points za geneiriranje trajektorija")
    print("  2. python 01_generate_pick_place_trajectory.py")


if __name__ == "__main__":
    main_example()
