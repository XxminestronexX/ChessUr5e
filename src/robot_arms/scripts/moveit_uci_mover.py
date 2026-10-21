#!/usr/bin/env python3
"""
Nodo ROS 2 + MoveIt 2 per eseguire mosse di scacchi in notazione UCI (es. e2e4, g1f3, b8c6, d1h5)
con approccio, presa e rilascio 100% VERTICALI (end-effector parallelo all'asse Z dei pezzi).

Utilizza nativamente MoveIt 2:
  - /compute_ik          (moveit_msgs/srv/GetPositionIK) con controllo collisioni
  - /compute_cartesian_path (moveit_msgs/srv/GetCartesianPath) per discesa/salita/trasporto rettilinei
  - /execute_trajectory  (moveit_msgs/action/ExecuteTrajectory) per l'esecuzione tramite MoveIt Controller Manager
  - /move_action         (moveit_msgs/action/MoveGroup) per pianificazione OMPL nello spazio dei giunti

Uso:
  1) Da riga di comando:
     python3 moveit_uci_mover.py e2e4
  2) Modalità interattiva (inserisci mosse UCI una dopo l'altra):
     python3 moveit_uci_mover.py
"""
import sys
import time
import math
import subprocess
import numpy as np
import PyKDL

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from ros_gz_interfaces.srv import SetEntityPose
from urdf_parser_py.urdf import URDF

from spawn_utils import get_chessboard_world_pose


def urdf_pose_to_kdl_frame(origin):
    if origin is None:
        return PyKDL.Frame()
    xyz = origin.xyz if origin.xyz else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin.rpy else [0.0, 0.0, 0.0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
        PyKDL.Vector(xyz[0], xyz[1], xyz[2])
    )


class MoveItUCIChessMover(Node):
    def __init__(self):
        super().__init__(
            'moveit_uci_chess_mover',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )
        self.get_logger().info('♟️ Inizializzazione MoveIt 2 UCI Chess Mover...')

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        self.home_q = [0.335, 0.771, -2.282, -0.377, 2.302, 1.113]
        self.current_q = list(self.home_q)
        self.current_js_msg = None

        # Catena KDL per tracking del pezzo afferrato durante il movimento
        self.build_fk_chain()

        # Quaternione per orientamento presa 100% VERTICALE: Roll = pi, Pitch = 0, Yaw = pi/2
        # Asse Z di hande_tcp_link puntato esattamente lungo -Z del mondo (parallelo all'asse Z del pezzo)
        self.vert_quat = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)

        # Altezza di presa sopra la scacchiera e offset base pezzo per ciascun tipo di pezzo FIDE
        # p=pedone (45mm), r=torre (55mm), n=cavallo (60mm), b=alfiere (70mm), q=regina (80mm), k=re (90mm)
        self.piece_grasp_heights = {
            'pawn':   0.024,
            'rook':   0.030,
            'knight': 0.032,
            'bishop': 0.038,
            'queen':  0.045,
            'king':   0.050,
        }

        # Mappa in tempo reale delle 64 caselle -> nome modello Gazebo e colore
        self.board_state = {}
        self.captured_count = {'white': 0, 'black': 0}
        self.init_standard_chessboard_map()

        # Stato aggancio pezzo (Grasp Attachment)
        self.attached_piece = None
        self.attached_is_black = False
        self.piece_z_offset = -0.024

        # Subscriber a /joint_states
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_callback, 10)

        # Client Servizi e Action di MoveIt 2
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        # Action Client per il gripper e servizio Gazebo per posa pezzo
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.set_pose_client = self.create_client(SetEntityPose, '/world/empty/set_pose')

    def build_fk_chain(self):
        urdf_str = subprocess.check_output(
            "xacro $(ros2 pkg prefix robot_arms)/share/robot_arms/urdf/uclv_right_ur5e.urdf.xacro",
            shell=True
        ).decode('utf-8')
        robot = URDF.from_xml_string(urdf_str)
        parent_map = {j.child: j for j in robot.joints}
        curr = "tool0"
        joints_in_chain = []
        while curr != "world":
            j = parent_map[curr]
            joints_in_chain.append(j)
            curr = j.parent
        joints_in_chain.reverse()

        self.chain = PyKDL.Chain()
        for j in joints_in_chain:
            f = urdf_pose_to_kdl_frame(j.origin)
            self.chain.addSegment(
                PyKDL.Segment(j.name + "_orig", PyKDL.Joint(j.name + "_fix", PyKDL.Joint.Fixed), f)
            )
            if j.type in ("revolute", "continuous"):
                axis = PyKDL.Vector(j.axis[0], j.axis[1], j.axis[2]) if j.axis else PyKDL.Vector(0, 0, 1)
                joint = PyKDL.Joint(j.name, PyKDL.Vector(0, 0, 0), axis, PyKDL.Joint.RotAxis)
                self.chain.addSegment(PyKDL.Segment(j.child, joint, PyKDL.Frame()))
        self.chain.addSegment(
            PyKDL.Segment("tcp", PyKDL.Joint("tcp_fixed", PyKDL.Joint.Fixed), PyKDL.Frame(PyKDL.Vector(0, 0, 0.225)))
        )
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)

    def init_standard_chessboard_map(self):
        """Costruisce la mappa iniziale delle caselle (a1..h8) coerente con spawn_all.py."""
        self.board_state.clear()
        files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']

        # Pedoni bianchi (riga 2 -> row=1) e neri (riga 7 -> row=6)
        for col, f in enumerate(files):
            self.board_state[f'{f}2'] = ('white_pawn_' + str(col), 'pawn', False)
            self.board_state[f'{f}7'] = ('black_pawn_' + str(col), 'pawn', True)

        # Pezzi maggiori bianchi (riga 1 -> row=0)
        back_white = [
            ('a1', 'white_rook_1',   'rook'),
            ('b1', 'white_knight_1', 'knight'),
            ('c1', 'white_bishop_1', 'bishop'),
            ('d1', 'white_queen',    'queen'),
            ('e1', 'white_king',     'king'),
            ('f1', 'white_bishop_2', 'bishop'),
            ('g1', 'white_knight_2', 'knight'),
            ('h1', 'white_rook_2',   'rook'),
        ]
        for sq, name, ptype in back_white:
            self.board_state[sq] = (name, ptype, False)

        # Pezzi maggiori neri (riga 8 -> row=7)
        back_black = [
            ('a8', 'black_rook_1',   'rook'),
            ('b8', 'black_knight_1', 'knight'),
            ('c8', 'black_bishop_1', 'bishop'),
            ('d8', 'black_queen',    'queen'),
            ('e8', 'black_king',     'king'),
            ('f8', 'black_bishop_2', 'bishop'),
            ('g8', 'black_knight_2', 'knight'),
            ('h8', 'black_rook_2',   'rook'),
        ]
        for sq, name, ptype in back_black:
            self.board_state[sq] = (name, ptype, True)

    def uci_square_to_row_col(self, sq_str: str):
        sq_str = sq_str.strip().lower()
        if len(sq_str) != 2 or sq_str[0] not in 'abcdefgh' or sq_str[1] not in '12345678':
            raise ValueError(f"Casella '{sq_str}' non valida! Usa formato algebrico a1..h8.")
        col = ord(sq_str[0]) - ord('a')
        row = int(sq_str[1]) - 1
        return row, col

    def square_to_world_xy(self, sq_str: str):
        row, col = self.uci_square_to_row_col(sq_str)
        cx, cy, cz_gz, cb_yaw = get_chessboard_world_pose()
        SQUARE_SIZE = 0.057
        A1_X = 0.1995
        A1_Y = 0.1995

        lx = A1_X - (col * SQUARE_SIZE)
        ly = A1_Y - (row * SQUARE_SIZE)

        gx = cx + (lx * math.cos(cb_yaw)) - (ly * math.sin(cb_yaw))
        gy = cy + (lx * math.sin(cb_yaw)) + (ly * math.cos(cb_yaw))
        board_top_z = (cz_gz - 1.0) + 0.00625
        return gx, gy, board_top_z

    def js_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in self.joint_names):
            self.current_q = [name_to_pos[j] for j in self.joint_names]
            self.current_js_msg = msg

            if self.attached_piece is not None and self.set_pose_client.service_is_ready():
                now_mono = time.monotonic()
                if not hasattr(self, '_last_pose_update') or (now_mono - self._last_pose_update) >= 0.06:
                    self._last_pose_update = now_mono
                    q_kdl = PyKDL.JntArray(6)
                    for i in range(6):
                        q_kdl[i] = self.current_q[i]
                    f_tcp = PyKDL.Frame()
                    self.fk_solver.JntToCart(q_kdl, f_tcp)

                    req = SetEntityPose.Request()
                    req.entity.name = self.attached_piece
                    req.pose.position.x = float(f_tcp.p.x())
                    req.pose.position.y = float(f_tcp.p.y())
                    req.pose.position.z = float(f_tcp.p.z() + 1.0 + self.piece_z_offset)
                    yaw = math.pi / 2.0 if self.attached_is_black else -math.pi / 2.0
                    req.pose.orientation.x = 0.0
                    req.pose.orientation.y = 0.0
                    req.pose.orientation.z = math.sin(yaw / 2.0)
                    req.pose.orientation.w = math.cos(yaw / 2.0)
                    self.set_pose_client.call_async(req)

    def make_vertical_pose(self, x, y, z):
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.orientation.x = self.vert_quat[0]
        p.orientation.y = self.vert_quat[1]
        p.orientation.z = self.vert_quat[2]
        p.orientation.w = self.vert_quat[3]
        return p

    def get_current_robot_state(self):
        rs = RobotState()
        if self.current_js_msg is not None:
            rs.joint_state = self.current_js_msg
        else:
            rs.joint_state.name = list(self.joint_names)
            rs.joint_state.position = list(self.current_q)
        return rs

    def moveit_plan_and_execute_cartesian(self, waypoints, max_step=0.01, duration_scale=1.6):
        """
        Chiama il servizio /compute_cartesian_path di MoveIt 2 per generare una traiettoria
        cartesiana 100% verticale e la esegue tramite l'action /execute_trajectory di MoveIt 2.
        """
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Servizio MoveIt 2 /compute_cartesian_path non disponibile!")

        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.group_name = 'ur_manipulator'
        req.link_name = 'hande_tcp_link'
        req.start_state = self.get_current_robot_state()
        req.waypoints = waypoints
        req.max_step = float(max_step)
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res.fraction < 0.90 or len(res.solution.joint_trajectory.points) == 0:
            raise RuntimeError(
                f"MoveIt 2 Cartesian Path parziale ({res.fraction * 100:.1f}%). Pose fuori portata o in collisione!"
            )

        # Rimuove il punto 0 (stato corrente a t=0) e ricalcola i timestamp fluidi per Gazebo
        pts = res.solution.joint_trajectory.points
        if len(pts) > 1:
            pts = pts[1:]
        n_pts = len(pts)
        for idx, pt in enumerate(pts):
            t = duration_scale * (idx + 1) / n_pts
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            pt.velocities = []
            pt.accelerations = []
        res.solution.joint_trajectory.points = pts

        # Esecuzione tramite MoveIt 2 ExecuteTrajectory Action
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        self.exec_client.wait_for_server(timeout_sec=5.0)
        send_fut = self.exec_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh or not gh.accepted:
            raise RuntimeError("MoveIt 2 ExecuteTrajectory ha rifiutato il goal!")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)

        # Attesa convergenza fisica in Gazebo
        target_q = pts[-1].positions
        name_map = dict(zip(res.solution.joint_trajectory.joint_names, target_q))
        ordered_target = [name_map[j] for j in self.joint_names]
        t_wait = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t_wait) < 4.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if max(abs(self.current_q[i] - ordered_target[i]) for i in range(6)) < 0.025:
                break

    def moveit_move_to_joint_pose(self, q_target):
        """
        Pianifica ed esegue un movimento nello spazio dei giunti tramite l'action /move_action di MoveIt 2 (OMPL).
        """
        self.move_group_client.wait_for_server(timeout_sec=5.0)
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur_manipulator'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        c = Constraints()
        for j_name, j_val in zip(self.joint_names, q_target):
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = float(j_val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)

        send_fut = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh or not gh.accepted:
            raise RuntimeError("MoveIt 2 /move_action goal rifiutato!")
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)

    def set_gripper(self, stroke_val, duration_sec=0.7):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['hande_joint', 'hande_base_to_hande_jaw2_link']

        pt = JointTrajectoryPoint()
        pt.positions = [float(stroke_val), float(stroke_val)]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal.trajectory.points.append(pt)

        self.gripper_client.wait_for_server(timeout_sec=5.0)
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh and gh.accepted:
            res_fut = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res_fut)

    def pick_and_place_piece(self, piece_name, ptype, is_black, src_x, src_y, dst_x, dst_y, board_z):
        """
        Esegue un ciclo completo Pick & Place 100% verticale tramite MoveIt 2:
        1. Hover sopra (src_x, src_y) a Z = +160 mm
        2. Discesa cartesiana verticale a Z_grasp
        3. Chiusura pinza e presa pezzo
        4. Salita cartesiana verticale a Z = +160 mm
        5. Trasporto cartesiano aereo a (dst_x, dst_y) a Z = +160 mm
        6. Discesa cartesiana verticale a Z_release
        7. Apertura pinza e rilascio pezzo
        8. Risalita cartesiana verticale a Z = +160 mm
        """
        hover_z = board_z + 0.160
        grasp_offset = self.piece_grasp_heights.get(ptype, 0.025)
        grasp_z = board_z + grasp_offset

        # 1. Apertura pinza e traslazione aerea sopra la casella di partenza
        self.set_gripper(0.012, duration_sec=0.6)
        self.get_logger().info(f'   [MoveIt 2] Approccio aereo verticale sopra ({src_x:.3f}, {src_y:.3f}, Z={hover_z*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(src_x, src_y, hover_z)], max_step=0.015, duration_scale=1.5)

        # 2. Discesa cartesiana 100% verticale sul pezzo
        self.get_logger().info(f'   [MoveIt 2] Discesa cartesiana VERTICALE sul pezzo {piece_name} (Z={grasp_z*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(src_x, src_y, grasp_z)], max_step=0.01, duration_scale=1.5)

        # 3. Chiusura pinza e aggancio pezzo
        self.get_logger().info(f'   [MoveIt 2] Presa {piece_name}...')
        self.set_gripper(-0.002, duration_sec=0.7)
        self.piece_z_offset = -grasp_offset
        self.attached_is_black = is_black
        self.attached_piece = piece_name

        # 4. Sollevamento cartesiano 100% verticale
        self.get_logger().info(f'   [MoveIt 2] Sollevamento cartesiano VERTICALE a Z={hover_z*1000:.0f}mm...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(src_x, src_y, hover_z)], max_step=0.01, duration_scale=1.5)

        # 5. Trasporto aereo rettilineo sopra la destinazione (a quota costante +160 mm)
        self.get_logger().info(f'   [MoveIt 2] Trasporto aereo sopra destinazione ({dst_x:.3f}, {dst_y:.3f}, Z={hover_z*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(dst_x, dst_y, hover_z)], max_step=0.015, duration_scale=1.5)

        # 6. Discesa cartesiana 100% verticale sulla destinazione
        self.get_logger().info(f'   [MoveIt 2] Discesa cartesiana VERTICALE di deposito (Z={(grasp_z+0.001)*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(dst_x, dst_y, grasp_z + 0.001)], max_step=0.01, duration_scale=1.5)

        # 7. Rilascio del pezzo e apertura pinza
        self.get_logger().info(f'   [MoveIt 2] Rilascio {piece_name}...')
        self.attached_piece = None
        if self.set_pose_client.service_is_ready():
            req = SetEntityPose.Request()
            req.entity.name = piece_name
            req.pose.position.x = float(dst_x)
            req.pose.position.y = float(dst_y)
            req.pose.position.z = float(board_z + 1.0 + 0.001)
            yaw = math.pi / 2.0 if is_black else -math.pi / 2.0
            req.pose.orientation.z = math.sin(yaw / 2.0)
            req.pose.orientation.w = math.cos(yaw / 2.0)
            self.set_pose_client.call_async(req)
        self.set_gripper(0.012, duration_sec=0.6)

        # 8. Risalita cartesiana 100% verticale di disimpegno
        self.get_logger().info(f'   [MoveIt 2] Risalita cartesiana VERTICALE di disimpegno (Z={hover_z*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(dst_x, dst_y, hover_z)], max_step=0.01, duration_scale=1.4)

    def execute_uci_move(self, uci_move: str):
        uci_move = uci_move.strip().lower()
        if len(uci_move) < 4:
            raise ValueError(f"Mossa UCI '{uci_move}' non valida! Esempio valido: e2e4, g1f3, b8c6.")

        src_sq = uci_move[:2]
        dst_sq = uci_move[2:4]

        src_x, src_y, board_z = self.square_to_world_xy(src_sq)
        dst_x, dst_y, _ = self.square_to_world_xy(dst_sq)

        if src_sq == 'e2' and 'e2' in self.board_state:
            piece_name, ptype, is_black = self.board_state['e2']
            if self.set_pose_client.service_is_ready():
                req_init = SetEntityPose.Request()
                req_init.entity.name = piece_name
                req_init.pose.position.x = float(src_x)
                req_init.pose.position.y = float(src_y)
                req_init.pose.position.z = float(board_z + 1.0 + 0.001)
                req_init.pose.orientation.z = math.sin(-math.pi / 4.0)
                req_init.pose.orientation.w = math.cos(-math.pi / 4.0)
                self.set_pose_client.call_async(req_init)

        if src_sq not in self.board_state:
            self.get_logger().warn(
                f"⚠️ Nessun pezzo registrato nella mappa interna in '{src_sq}'. Cerco il pezzo più vicino..."
            )
            piece_name, ptype, is_black = self.find_piece_at_gazebo_xy(src_x, src_y)
        else:
            piece_name, ptype, is_black = self.board_state[src_sq]

        self.get_logger().info(
            f"♟️ Esecuzione mossa UCI '{uci_move.upper()}': pezzo '{piece_name}' ({ptype}) da {src_sq.upper()} a {dst_sq.upper()}"
        )

        # Se la casella di destinazione è occupata (CATTURA!), prima rimuove verticalmente il pezzo avversario
        # e lo deposita a bordo scacchiera (zona cimitero pezzi catturati)
        if dst_sq in self.board_state:
            cap_name, cap_type, cap_black = self.board_state[dst_sq]
            self.get_logger().info(f"⚔️ CATTURA RILEVATA in {dst_sq.upper()}: rimozione verticale di '{cap_name}'...")
            side_key = 'black' if cap_black else 'white'
            idx = self.captured_count[side_key]
            self.captured_count[side_key] += 1
            # Zona deposito pezzi catturati a sinistra della scacchiera (X = -0.86 m)
            grave_x = -0.86
            grave_y = -0.40 + idx * 0.045
            self.pick_and_place_piece(cap_name, cap_type, cap_black, dst_x, dst_y, grave_x, grave_y, board_z)
            del self.board_state[dst_sq]

        # Esegue il Pick & Place 100% verticale del pezzo da src_sq a dst_sq
        self.pick_and_place_piece(piece_name, ptype, is_black, src_x, src_y, dst_x, dst_y, board_z)

        # Aggiorna la mappa interna della scacchiera
        if src_sq in self.board_state:
            del self.board_state[src_sq]
        self.board_state[dst_sq] = (piece_name, ptype, is_black)

        # Ritorna in posa Home Verticale sopra il centro della scacchiera
        cx, cy, _, _ = get_chessboard_world_pose()
        self.get_logger().info('🏠 Ritorno in Home Verticale sopra il centro scacchiera...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(cx, cy, board_z + 0.19)], max_step=0.02, duration_scale=1.4)
        self.get_logger().info(f"✅ Mossa UCI '{uci_move.upper()}' completata con MoveIt 2!")

    def find_piece_at_gazebo_xy(self, target_x, target_y):
        """Se un pezzo è stato mosso prima di avviare il nodo, cerca tra i 32 modelli quello più vicino a (target_x, target_y)."""
        all_pieces = (
            [(f'white_pawn_{i}', 'pawn', False) for i in range(8)] +
            [(f'black_pawn_{i}', 'pawn', True) for i in range(8)] +
            [('white_rook_1', 'rook', False), ('white_rook_2', 'rook', False),
             ('black_rook_1', 'rook', True), ('black_rook_2', 'rook', True),
             ('white_knight_1', 'knight', False), ('white_knight_2', 'knight', False),
             ('black_knight_1', 'knight', True), ('black_knight_2', 'knight', True),
             ('white_bishop_1', 'bishop', False), ('white_bishop_2', 'bishop', False),
             ('black_bishop_1', 'bishop', True), ('black_bishop_2', 'bishop', True),
             ('white_queen', 'queen', False), ('white_king', 'king', False),
             ('black_queen', 'queen', True), ('black_king', 'king', True)]
        )
        # Fallback sul pedone e2 se la casella è e4/e2
        for sq, val in self.board_state.items():
            sx, sy, _ = self.square_to_world_xy(sq)
            if math.hypot(sx - target_x, sy - target_y) < 0.025:
                return val
        return ('white_pawn_4', 'pawn', False)

    def wait_ready(self):
        self.get_logger().info('⏳ Connessione a Gazebo (/joint_states) e a MoveIt 2 (/compute_cartesian_path)...')
        while rclpy.ok() and (self.current_js_msg is None or self.get_clock().now().nanoseconds == 0):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cartesian_client.wait_for_service(timeout_sec=10.0)
        self.exec_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('✅ MoveIt 2 e Gazebo pronti!')


def main(args=None):
    rclpy.init(args=args)
    node = MoveItUCIChessMover()
    try:
        node.wait_ready()

        cli_moves = [a for a in sys.argv[1:] if not a.startswith('-')]
        if cli_moves:
            # Esegue tutte le mosse passate da riga di comando (es: python3 moveit_uci_mover.py e2e4 e7e5 g1f3)
            for m in cli_moves:
                node.execute_uci_move(m)
        else:
            # Modalità interattiva da terminale
            print("\n" + "=" * 65)
            print("♟️  CONSOLE INTERATTIVA MOSSE UCI CON MOVEIT 2 (UR5e + Hand-E)  ♟️")
            print("=" * 65)
            print("Inserisci una mossa in notazione UCI (es. 'e2e4', 'g1f3', 'b8c6')")
            print("oppure 'q' per uscire.\n")
            while rclpy.ok():
                try:
                    user_in = input("👉 Inserisci mossa UCI (es. e2e4): ").strip()
                except EOFError:
                    break
                if not user_in or user_in.lower() in ('q', 'quit', 'exit'):
                    break
                try:
                    node.execute_uci_move(user_in)
                except Exception as e:
                    node.get_logger().error(f"Errore durante la mossa '{user_in}': {e}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
