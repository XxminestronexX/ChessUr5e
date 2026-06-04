# spawn_utils.py
import re
import math
import subprocess
from ament_index_python.packages import get_package_share_directory
import os

def get_chessboard_world_pose():
    """Legge la VERA posa World della scacchiera da Gazebo"""
    cmds = ["gz model -m chessboard", "ign model -m chessboard"]
    for cmd in cmds:
        try:
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL).decode('utf-8')
            lines = output.split('\n')
            for i, line in enumerate(lines):
                if "Pose" in line:
                    xyz_line = lines[i+1]
                    rpy_line = lines[i+2]
                    xyz = [float(n) for n in re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?', xyz_line)]
                    rpy = [float(n) for n in re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?', rpy_line)]
                    if len(xyz) >= 3 and len(rpy) >= 3:
                        return xyz[0], xyz[1], xyz[2], rpy[2]
        except Exception:
            continue
    return -0.5, -0.15, 0.56, 0.0

def spawn_piece(name, row, col, mesh, color, piece_type, is_black):
    pkg_share = get_package_share_directory('robot_arms')
    xacro_path = os.path.join(pkg_share, 'urdf', 'chess_piece.urdf.xacro')
    
    cx, cy, cz, cb_yaw = get_chessboard_world_pose()
    
    SQUARE_SIZE = 0.057
    A1_X = 0.1995
    A1_Y = 0.1995
    
    # Calcolo coordinate Locali pulito
    lx = A1_X - (col * SQUARE_SIZE) 
    ly = A1_Y - (row * SQUARE_SIZE) 
    
    # Trasformazione in coordinate GLOBALI (World)
    gx = cx + (lx * math.cos(cb_yaw)) - (ly * math.sin(cb_yaw))
    gy = cy + (lx * math.sin(cb_yaw)) + (ly * math.cos(cb_yaw))
    
    PIECE_YAW_OFFSET = 1.5708
    yaw = cb_yaw + PIECE_YAW_OFFSET + (math.pi if is_black else 0.0)
    
    spawn_z = cz + 0.02
    
    print(f"[DEBUG] Spawnando {name} (Tipo: {piece_type}): row={row}, col={col}")
    
    # ECCO LA MAGIA: Passiamo piece_type:='{piece_type}' direttamente al file Xacro!
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', name,
        '-string', subprocess.check_output(f"xacro {xacro_path} mesh_name:='{mesh}' color_rgba:='{color}' piece_type:='{piece_type}'", shell=True).decode('utf-8'),
        '-x', str(gx), '-y', str(gy), '-z', str(spawn_z), '-Y', str(yaw)
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL)