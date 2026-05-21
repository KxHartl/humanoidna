#!/usr/bin/env python3
"""
Robot Communication Module
TCP/IP komunikacija s UR robotom za slanje trajektorija

Korišten za slanje generiranih trajektorija na UR robota
Simulacija: PolyScope (UR simulator)
"""

import socket
import numpy as np
import time
from pathlib import Path
from typing import List


class URRobotCommunication:
    """TCP/IP komunikacija s UR robotom"""
    
    def __init__(self, host: str = '192.168.1.100', 
                 port: int = 30003,
                 timeout: float = 5.0):
        """
        Inicijalizacija komunikacije
        
        Args:
            host: IP adresa robota
            port: Real-time port (30003 je UR standard)
            timeout: Timeout za konekciju
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket = None
        self.connected = False
    
    def connect(self) -> bool:
        """
        Uspostavi konekciju s robotom
        
        Returns:
            bool: True ako je uspješno, False inače
        """
        try:
            print(f"Pokušavam se povezati na {self.host}:{self.port}...")
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            
            self.connected = True
            print(f"✓ Uspješno povezana konekcija!")
            
            return True
        
        except socket.timeout:
            print(f"✗ Timeout - Robot nije dostupan na {self.host}:{self.port}")
            return False
        except ConnectionRefusedError:
            print(f"✗ Konekcija odbijena - Provjeri IP i port")
            return False
        except Exception as e:
            print(f"✗ Greška pri konekciji: {e}")
            return False
    
    def disconnect(self):
        """Prekini konekciju"""
        if self.socket:
            self.socket.close()
            self.connected = False
            print("✓ Konekcija prekinuta")
    
    def send_command(self, command: str) -> bool:
        """
        Pošalji URScript komandu robotu
        
        Args:
            command: URScript kod kao string
            
        Returns:
            bool: True ako je uspješno
        """
        if not self.connected:
            print("✗ Robot nije povezan!")
            return False
        
        try:
            self.socket.sendall(command.encode() + b'\n')
            print(f"✓ Komanda poslana")
            return True
        except Exception as e:
            print(f"✗ Greška pri slanju komande: {e}")
            return False
    
    def send_trajectory_as_servoj(self, trajectory: np.ndarray,
                                  lookahead_time: float = 0.1,
                                  gain: float = 300):
        """
        Pošalji trajektoriju kao niz servoj naredbi
        
        Args:
            trajectory: Trajektorija (N x 6) u radijanima
            lookahead_time: Look-ahead vrijeme za servoj
            gain: Gain za servoj kontrolu
        """
        if not self.connected:
            print("✗ Robot nije povezan!")
            return
        
        print(f"\nSlanje trajektorije ({len(trajectory)} točaka)...")
        
        for i, point in enumerate(trajectory):
            # URScript servoj komanda
            # servoj(q_target, t=lookahead_time, lookahead_time=lookahead_time, gain=gain)
            q_str = ', '.join([f'{q:.6f}' for q in point])
            
            cmd = f"servoj([{q_str}], t=0.002, lookahead_time={lookahead_time}, gain={gain})"
            
            if not self.send_command(cmd):
                print(f"✗ Greška pri slanju točke {i}")
                return
            
            # Minimalna pauza između naredbi
            time.sleep(0.001)
            
            # Progress
            if (i + 1) % 50 == 0:
                print(f"  {i+1}/{len(trajectory)} točaka poslano...")
        
        print(f"✓ Trajektorija uspješno poslana!")
    
    def generate_urscript_program(self, trajectory: np.ndarray,
                                  filename: str = "pick_and_place.script"):
        """
        Generiraj kompletan URScript program
        
        Args:
            trajectory: Trajektorija (N x 6)
            filename: Naziv izlazne datoteke
        """
        print(f"Generiram URScript program: {filename}")
        
        script_lines = [
            "#!/usr/bin/env python3",
            "# Auto-generated Pick and Place Program",
            "# DO NOT EDIT MANUALLY",
            "",
            "def pick_and_place():",
            "    # Program za autonomnu hvatanja i odlaganja objekta",
            "    ",
            "    # Definicije točaka trajektorije",
        ]
        
        # Dodaj trajektorijske točke kao definicije
        for i, point in enumerate(trajectory):
            q_str = '[' + ', '.join([f'{q:.6f}' for q in point]) + ']'
            script_lines.append(f"    q{i} = {q_str}")
        
        script_lines.extend([
            "",
            "    # Izvršavanje trajektorije",
            "    for i in range(len(trajectory)):",
            "        servoj(trajectory[i], t=0.002, lookahead_time=0.1, gain=300)",
            "",
            "pick_and_place()"
        ])
        
        script = '\n'.join(script_lines)
        
        with open(filename, 'w') as f:
            f.write(script)
        
        print(f"✓ Program spreman: {filename}")
    
    def send_trajectory_from_file(self, trajectory_file: str,
                                  lookahead_time: float = 0.1,
                                  gain: float = 300):
        """
        Pročitaj trajektoriju iz datoteke i pošalji je robotu
        
        Args:
            trajectory_file: Putanja do datoteke s trajektorijom
            lookahead_time: Look-ahead vrijeme
            gain: Gain za servoj kontrolu
        """
        print(f"Čitam trajektoriju iz: {trajectory_file}")
        
        # Pročitaj datoteku
        trajectory = np.loadtxt(trajectory_file)
        
        # Ako je samo jedan redak, konvertiraj u 2D
        if trajectory.ndim == 1:
            trajectory = trajectory.reshape(1, -1)
        
        print(f"Učitana trajektorija s {len(trajectory)} točaka")
        
        # Pošalji
        self.send_trajectory_as_servoj(trajectory, lookahead_time, gain)


def simulate_robot_execution(trajectory: np.ndarray,
                             output_file: str = "simulated_execution.log"):
    """
    Simulacija izvršenja trajektorije (bez pravog robota)
    Koristi se za testiranje prije slanja na pravog robota
    
    Args:
        trajectory: Trajektorija (N x 6)
        output_file: Datoteka za zapis simulacije
    """
    print(f"\n╔{'═'*58}╗")
    print("║" + " "*15 + "SIMULACIJA IZVRŠENJA TRAJEKTORIJE" + " "*11 + "║")
    print(f"╚{'═'*58}╝")
    
    print(f"Broj točaka: {len(trajectory)}")
    print(f"Trajanje: {len(trajectory) * 0.002:.2f}s (pri 500Hz)")
    
    log_lines = [
        "SIMULACIJA IZVRŠENJA TRAJEKTORIJE",
        f"Vrijeme: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"Broj točaka: {len(trajectory)}",
        "",
        "Izvršavanje:"
    ]
    
    for i, point in enumerate(trajectory):
        q_str = ', '.join([f'{q:7.4f}' for q in point])
        print(f"  {i:4d}: [{q_str}]", end='\r')
        log_lines.append(f"{i:4d}: {q_str}")
    
    print(" "*80, end='\r')
    print(f"✓ Simulacija završena!")
    
    # Spremi log
    with open(output_file, 'w') as f:
        f.write('\n'.join(log_lines))
    
    print(f"✓ Log spreman: {output_file}")


def main_example():
    """Primjer korištenja"""
    
    print("\n" + "="*60)
    print("ROBOT COMMUNICATION - Primjer")
    print("="*60)
    
    # 1. Kreiraj konekciju
    print("\n1. USPOSTAVLJANJE KONEKCIJE")
    print("-"*60)
    
    robot = URRobotCommunication(
        host='192.168.1.100',  # IP UR robota
        port=30003
    )
    
    # Pokušaj konekcije (ako robot nije dostupan, ovo će failati)
    connected = robot.connect()
    
    if not connected:
        print("\n⚠ Robot nije dostupan - Koristi simulaciju")
        
        # Simuliraj s primjernom trajektorijom
        print("\n2. SIMULACIJA (bez robota)")
        print("-"*60)
        
        # Primjer trajektorije (6 točaka)
        example_trajectory = np.array([
            [-1.571, -2.094, -1.571,  0.524,  1.571,  0.000],
            [-1.400, -2.100, -1.550,  0.550,  1.570,  0.050],
            [-1.250, -2.110, -1.520,  0.580,  1.565,  0.100],
            [-1.100, -2.115, -1.480,  0.610,  1.560,  0.150],
            [-0.950, -2.110, -1.430,  0.640,  1.555,  0.200],
            [-0.785, -2.094,  0.000, -1.571,  0.000,  0.000],
        ])
        
        simulate_robot_execution(example_trajectory)
    
    else:
        print("\n2. SLANJE TRAJEKTORIJE")
        print("-"*60)
        
        # Učitaj trajektoriju iz datoteke
        trajectory_file = "trajectories/trajectory_HOME_to_APPROACH.txt"
        
        if Path(trajectory_file).exists():
            robot.send_trajectory_from_file(trajectory_file)
        else:
            print(f"✗ Datoteka nije pronađena: {trajectory_file}")
        
        # Prekini konekciju
        robot.disconnect()
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main_example()
