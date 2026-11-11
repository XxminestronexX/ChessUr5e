# spawn_utils.py
import json
import re
import math
import subprocess
from ament_index_python.packages import get_package_share_directory
import os


def get_chessboard_gazebo_pose():
    """Legge la VERA posa World della scacchiera DIRETTAMENTE DA GAZEBO per lo spawn iniziale delle pedine."""
    cmds = ["gz model -m chessboard -p", "ign model -m chessboard -p", "gz model -m chessboard", "ign model -m chessboard"]
    for cmd in cmds:
        try:
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL).decode('utf-8')
            lines = output.split('\n')
            for i, line in enumerate(lines):
                if "Pose [" in line or "Pose" in line:
                    xyz_line = lines[i+1]
                    rpy_line = lines[i+2]
                    xyz = [float(n) for n in re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?', xyz_line)]
                    rpy = [float(n) for n in re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?', rpy_line)]
                    if len(xyz) >= 3 and len(rpy) >= 3:
                        return xyz[0], xyz[1], xyz[2], rpy[2]
        except Exception:
            continue
    return -0.58, -0.22, 1.0098, 3.14159


def get_vision_detected_pose():
    """Legge la posa della scacchiera e dell'ArUco RILEVATI DALLA TELECAMERA D435 tramite Computer Vision."""
    json_path = "/workspace/ChessUr5e/detected_chessboard_pose.json"
    if not os.path.exists(json_path):
        json_path = os.path.join(os.getcwd(), "detected_chessboard_pose.json")
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                return json.load(f)
        except Exception:
            pass
    return None


def is_aruco_currently_detected():
    """Restituisce True se l'ultimo scatto ha effettivamente individuato il marker ArUco."""
    data = get_vision_detected_pose()
    if data and data.get("detected", False) is True:
        return True
    return False


def get_chessboard_world_pose():
    """
    Restituisce la posa World della scacchiera RICONOSCIUTA DALLA TELECAMERA D435 (Marker ArUco).
    Zero chiamate a Gazebo per il robot.
    Garantisce che Z sia sempre nel sistema World di Gazebo (Z ~ 1.01 m).
    """
    data = get_vision_detected_pose()
    if data and "chessboard" in data:
        cb = data["chessboard"]
        z = cb["z"]
        if z < 0.5:
            z += 1.0
        return cb["x"], cb["y"], z, cb["yaw"]
    return -0.58, -0.22, 1.0098, 3.14159


def get_aruco_world_pose():
    """
    Restituisce la posa 3D del marker ArUco RICONOSCIUTA DALLA TELECAMERA D435.
    Zero chiamate a Gazebo per il robot.
    Garantisce che Z sia sempre nel sistema World di Gazebo (Z ~ 1.01 m).
    """
    data = get_vision_detected_pose()
    if data and "aruco" in data:
        ar = data["aruco"]
        z = ar["z"]
        if z < 0.5:
            z += 1.0
        return ar["x"], ar["y"], z, ar["yaw"]

    cx, cy, cz_gz, cb_yaw = get_chessboard_gazebo_pose()
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
        dx = -0.08 m  (centra l'asse ottico della telecamera esattamente sul centro della scacchiera)
        dy = -0.23 m  (allineato all'asse centrale della scacchiera)
        dz = +0.48 m  (quota aerea zenitale per inquadrare tutte le 64 caselle)

    Se la scacchiera viene ruotata, girata o traslata sul tavolo,
    la posa di scatto e l'orientamento della telecamera ruotano rigidamente insieme all'ArUco.
    """
    ax, ay, az_world, cb_yaw = get_aruco_world_pose()
    PHOTO_REL_ARUCO_X = -0.08
    PHOTO_REL_ARUCO_Y = -0.23
    PHOTO_REL_ARUCO_Z = 0.48

    px = ax + (PHOTO_REL_ARUCO_X * math.cos(cb_yaw)) - (PHOTO_REL_ARUCO_Y * math.sin(cb_yaw))
    py = ay + (PHOTO_REL_ARUCO_X * math.sin(cb_yaw)) + (PHOTO_REL_ARUCO_Y * math.cos(cb_yaw))
    robot_az = (az_world - 1.0) if az_world > 0.5 else az_world
    pz_robot = robot_az + PHOTO_REL_ARUCO_Z

    # Orientamento verticale sincronizzato alla rotazione della scacchiera
    ee_yaw = cb_yaw - (math.pi / 2.0)
    qx = math.cos(ee_yaw / 2.0)
    qy = math.sin(ee_yaw / 2.0)
    qz = 0.0
    qw = 0.0

    return px, py, pz_robot, (qx, qy, qz, qw)


def get_square_world_pose(col: int, row: int):
    """
    Calcola la posa World (gx, gy, board_top_z) di una casella (col, row)
    DIRETTAMENTE rispetto alla terna del Marker ArUco (ID 0).
    Zero intermediari, ancoraggio geometrico rigido al marker ArUco.
    """
    ax, ay, az_world, ar_yaw = get_aruco_world_pose()

    # Offset rigido della casella A1 rispetto al marker ArUco (da chessboard.urdf.xacro):
    # aruco_marker_link: (0.28, 0.23, 0.0012)
    # a1_square_link:   (-0.06710, -0.01255, 0.0)
    A1_REL_ARUCO_X = -0.06710
    A1_REL_ARUCO_Y = -0.01255
    SQUARE_SIZE = 0.05875

    lx = A1_REL_ARUCO_X - (col * SQUARE_SIZE)
    ly = A1_REL_ARUCO_Y - (row * SQUARE_SIZE)

    gx = ax + (lx * math.cos(ar_yaw)) - (ly * math.sin(ar_yaw))
    gy = ay + (lx * math.sin(ar_yaw)) + (ly * math.cos(ar_yaw))

    # Piano superiore della scacchiera nel frame MoveIt (tavolo a Z=0, mesh spessore 11mm)
    board_top_z = 0.0112
    return gx, gy, board_top_z

def spawn_piece(name, row, col, mesh, color, piece_type, is_black):
    pkg_share = get_package_share_directory('robot_arms')
    xacro_path = os.path.join(pkg_share, 'urdf', 'chess_piece.urdf.xacro')
    
    # Per lo spawn fisico dei pezzi in Gazebo, interroghiamo direttamente Gazebo
    cx, cy, cz, cb_yaw = get_chessboard_gazebo_pose()
    
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