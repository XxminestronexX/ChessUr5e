#!/usr/bin/env python3
import os
import re
import math
import time
import subprocess
from ament_index_python.packages import get_package_share_directory

def get_current_chessboard_pose():
    """Interroga Gazebo per ricavare la posizione corrente della scacchiera prima di sostituirla."""
    try:
        output = subprocess.check_output("gz model -m chessboard --pose", shell=True).decode('utf-8')
        for line in output.split('\n'):
            if "Pose [" in line and "|" in line:
                parts = re.findall(r'\[([^\]]+)\]', line)
                data_parts = [p for p in parts if '|' in p]
                xyz = [float(x.strip()) for x in data_parts[0].split('|')]
                rpy = [float(x.strip()) for x in data_parts[1].split('|')]
                return xyz, rpy
    except Exception:
        pass
    # Fallback standard del launch file se non rilevata
    return [-0.5, -0.15, 0.001], [0.0, 0.0, -1.5708]

def main():
    # Rilevamento della posizione per la sostituzione dinamica
    print("🔍 Rilevamento della posizione corrente della scacchiera...")
    xyz, rpy = get_current_chessboard_pose()
    
    # Parametri geometrici al millimetro richiesti
    SQUARE_SIZE = 0.057  # Caselle da 57mm (456mm / 8)
    
    # Mappatura dello schieramento iniziale sulle case reali (a1-h8)
    back_row = ['torre.stl', 'cavallo.stl', 'alfiere.stl', 'regina.stl', 're.stl', 'alfiere.stl', 'cavallo.stl', 'torre.stl']
    pieces_setup = []
    
    # Bianchi: Righe 1 e 2 (Indici 0 e 1)
    for col in range(8):
        pieces_setup.append({'name': f'white_piece_{col}', 'file': col, 'rank': 0, 'mesh': back_row[col], 'color': '1.0 1.0 1.0 1.0', 'yaw': 0.0})
        pieces_setup.append({'name': f'white_pawn_{col}', 'file': col, 'rank': 1, 'mesh': 'pedone.stl', 'color': '1.0 1.0 1.0 1.0', 'yaw': 0.0})
        
    # Neri: Righe 7 e 8 (Indici 6 e 7)
    for col in range(8):
        pieces_setup.append({'name': f'black_pawn_{col}', 'file': col, 'rank': 6, 'mesh': 'pedone.stl', 'color': '0.15 0.15 0.15 1.0', 'yaw': math.pi})
        pieces_setup.append({'name': f'black_piece_{col}', 'file': col, 'rank': 7, 'mesh': back_row[col], 'color': '0.15 0.15 0.15 1.0', 'yaw': math.pi})

    # Generazione del file unico SDF strutturato a Parent/Children
    sdf_path = os.path.join(os.path.expanduser("~"), "braccio_ws", "chessboard_full.sdf")
    
    sdf_content = """<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="chessboard_full">
    <include>
      <uri>model://chessboard</uri>
      <name>chessboard</name>
    </include>
"""

    for p in pieces_setup:
        # Calcolo delle coordinate cartesiane locali riferite al centro (0,0) della scacchiera
        loc_x = (p['file'] - 3.5) * SQUARE_SIZE
        loc_y = (3.5 - p['rank']) * SQUARE_SIZE
        loc_z = 0.015  # Quota locale di appoggio sopra il piano di gioco
        
        # 🔥 CORREZIONE FONDAMENTALE: Usiamo il formato nativo model:// compatibile con GZ_SIM_RESOURCE_PATH
        mesh_uri = f"model://robot_arms/meshes/scacchi/{p['mesh']}"
        
        # Scrittura del sotto-modello del pezzo (Child) vincolato alla scacchiera
        sdf_content += f"""
    <model name="{p['name']}">
      <pose>{loc_x} {loc_y} {loc_z} 0 0 {p['yaw']}</pose>
      <link name="link">
        <inertial>
          <mass>0.05</mass>
          <inertia>
            <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>{p['color']}</ambient>
            <diffuse>{p['color']}</diffuse>
            <specular>0.2 0.2 0.2 1.0</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.018</radius>
              <length>0.04</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
"""

    sdf_content += """  </model>
</sdf>
"""

    # Salvataggio del file SDF generato
    with open(sdf_path, "w") as f:
        f.write(sdf_content)
    print(f"✅ File strutturato SDF generato in: {sdf_path}")

    # Rimozione della vecchia scacchiera vuota per evitare collisioni fisiche
    print("🧹 Rimozione della scacchiera vuota iniziale da Gazebo...")
    del_cmd = 'gz service -s /world/default/entity/delete --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --req \'name: "chessboard"\''
    subprocess.run(del_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Spawn del nuovo blocco unificato (Scacchiera + 32 Pezzi solidali)
    print("🤖 Immissione del modello unificato solidale 'chessboard_full'...")
    spawn_cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-file', sdf_path,
        '-name', 'chessboard_full',
        '-x', str(xyz[0]), '-y', str(xyz[1]), '-z', str(xyz[2]),
        '-R', str(rpy[0]), '-P', str(rpy[1]), '-Y', str(rpy[2])
    ]
    subprocess.run(spawn_cmd)
    print("🏁 Schieramento gerarchico completato con successo!")

if __name__ == '__main__':
    main()