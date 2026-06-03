# spawn_utils.py
import re
import math
import subprocess
from ament_index_python.packages import get_package_share_directory
import os

def get_chessboard_pose():
    try:
        output = subprocess.check_output("gz model -m chessboard --pose", shell=True).decode('utf-8')
        for line in output.split('\n'):
            if "Pose [" in line and "|" in line:
                parts = re.findall(r'\[([^\]]+)\]', line)
                data_parts = [p for p in parts if '|' in p]
                xyz = [float(x.strip()) for x in data_parts[0].split('|')]
                rpy = [float(x.strip()) for x in data_parts[1].split('|')]
                return xyz[0], xyz[1], xyz[2], rpy[2]
    except:
        return -0.5, -0.15, 0.001, -1.5708
    return -0.5, -0.15, 0.001, -1.5708

def spawn_piece(name, row, col, mesh, color, type, is_black):
    pkg_share = get_package_share_directory('robot_arms')
    xacro_path = os.path.join(pkg_share, 'urdf', 'chess_piece.urdf.xacro')
    cx, cy, cz, cb_yaw = get_chessboard_pose()
    
    SQUARE_SIZE = 0.057
    BORDER = 0.027
    if type == 'p':  # Pedone
        lx = (col - (4 if is_black else 1)) * SQUARE_SIZE - 2 * BORDER
    elif type == 'r':  # Torre
        lx = (col - (7.3 if is_black else -2.3)) * SQUARE_SIZE - 2 * BORDER
    elif type == 'n':  # Cavallo
        lx = (col - (7.2 if is_black else -1.2)) * SQUARE_SIZE - BORDER
    elif type == 'b':  # Alfiere
        lx = (col - (8.5 if is_black else -2.5)) * SQUARE_SIZE - BORDER
    elif type == 'q':  # Regina
        lx = (col - (9.2 if is_black else -3.2)) * SQUARE_SIZE - BORDER
    elif type == 'k':  # Re
        lx = (col - (5.8 if is_black else 0.2)) * SQUARE_SIZE - BORDER
    
    
    ly = (row - 3.5) * SQUARE_SIZE
    
    gx = cx + lx * math.cos(cb_yaw) - ly * math.sin(cb_yaw)
    gy = cy + lx * math.sin(cb_yaw) + ly * math.cos(cb_yaw)
    yaw = cb_yaw + (math.pi if is_black else 0.0)
    
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', name,
        '-string', subprocess.check_output(f"xacro {xacro_path} mesh_name:='{mesh}' color_rgba:='{color}'", shell=True).decode('utf-8'),
        '-x', str(gx), '-y', str(gy), '-z', str(cz + 1.22), '-Y', str(yaw)
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL)
