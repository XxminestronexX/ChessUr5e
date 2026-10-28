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
    return -0.58, -0.22, 1.006, 3.14159


def get_aruco_world_pose():
    """
    Restituisce la posa 3D reale del marker ArUco nel sistema di riferimento World.
    Il marker ArUco è fissato sul bordo della scacchiera a:
      X_local = +0.28 m, Y_local = +0.23 m, Z_local = +0.0012 m
    Qualsiasi traslazione o rotazione della scacchiera sposta rigidamente l'ArUco.
    """
    cx, cy, cz_gz, cb_yaw = get_chessboard_world_pose()
    ARUCO_LOCAL_X = 0.28
    ARUCO_LOCAL_Y = 0.23
    ARUCO_LOCAL_Z = 0.0012
    ax = cx + (ARUCO_LOCAL_X * math.cos(cb_yaw)) - (ARUCO_LOCAL_Y * math.sin(cb_yaw))
    ay = cy + (ARUCO_LOCAL_X * math.sin(cb_yaw)) + (ARUCO_LOCAL_Y * math.cos(cb_yaw))
    az = cz_gz + ARUCO_LOCAL_Z
    return ax, ay, az, cb_yaw


def get_photo_shoot_world_pose():
    """
    Calcola la posa di scatto (Home Panoramica) del braccio ANCORATA ALL'ARUCO MARKER.

    Nel sistema di coordinate locale dell'ArUco marker:
      - Vettore relativo al marker:
        dx = +0.04 m  (sposta il braccio a sinistra della colonna 'a', liberando A4/A5)
        dy = -0.23 m  (allineato all'asse centrale della scacchiera)
        dz = +0.45 m  (quota aerea per inquadrare tutte le 64 caselle)

    Se la scacchiera viene ruotata, girata o traslata sul tavolo,
    la posa di scatto e l'orientamento della telecamera ruotano rigidamente insieme all'ArUco.
    """
    ax, ay, az_world, cb_yaw = get_aruco_world_pose()
    PHOTO_REL_ARUCO_X = 0.04
    PHOTO_REL_ARUCO_Y = -0.23
    PHOTO_REL_ARUCO_Z = 0.45

    px = ax + (PHOTO_REL_ARUCO_X * math.cos(cb_yaw)) - (PHOTO_REL_ARUCO_Y * math.sin(cb_yaw))
    py = ay + (PHOTO_REL_ARUCO_X * math.sin(cb_yaw)) + (PHOTO_REL_ARUCO_Y * math.cos(cb_yaw))
    pz_robot = (az_world - 1.0) + PHOTO_REL_ARUCO_Z

    # Orientamento verticale sincronizzato alla rotazione della scacchiera
    ee_yaw = cb_yaw - (math.pi / 2.0)
    qx = math.cos(ee_yaw / 2.0)
    qy = math.sin(ee_yaw / 2.0)
    qz = 0.0
    qw = 0.0

    return px, py, pz_robot, (qx, qy, qz, qw)

def spawn_piece(name, row, col, mesh, color, piece_type, is_black):
    pkg_share = get_package_share_directory('robot_arms')
    xacro_path = os.path.join(pkg_share, 'urdf', 'chess_piece.urdf.xacro')
    
    cx, cy, cz, cb_yaw = get_chessboard_world_pose()
    
    # Dimensioni esatte estratte dalla mesh 3D della scacchiera (chessboard.obj)
    SQUARE_SIZE = 0.05875
    A1_X = 0.21290
    A1_Y = 0.21745
    
    # Calcolo coordinate Locali pulito
    lx = A1_X - (col * SQUARE_SIZE) 
    ly = A1_Y - (row * SQUARE_SIZE) 
    
    # Trasformazione in coordinate GLOBALI (World)
    gx = cx + (lx * math.cos(cb_yaw)) - (ly * math.sin(cb_yaw))
    gy = cy + (lx * math.sin(cb_yaw)) + (ly * math.cos(cb_yaw))
    
    PIECE_YAW_OFFSET = 1.5708
    yaw = cb_yaw + PIECE_YAW_OFFSET + (math.pi if is_black else 0.0)
    
    # Posa Z a filo con il piano superiore della scacchiera visuale (spessore mesh top = +0.0012)
    spawn_z = cz + 0.0014
    
    print(f"[DEBUG] Spawnando {name} (Tipo: {piece_type}): row={row}, col={col}")
    
    # ECCO LA MAGIA: Passiamo piece_type:='{piece_type}' direttamente al file Xacro!
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', name,
        '-string', subprocess.check_output(f"xacro {xacro_path} mesh_name:='{mesh}' color_rgba:='{color}' piece_type:='{piece_type}'", shell=True).decode('utf-8'),
        '-x', str(gx), '-y', str(gy), '-z', str(spawn_z), '-Y', str(yaw)
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL)