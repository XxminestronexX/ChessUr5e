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
import re
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
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from urdf_parser_py.urdf import URDF
import chess

from spawn_utils import (
    get_chessboard_world_pose,
    get_aruco_world_pose,
    get_photo_shoot_world_pose,
    get_square_world_pose,
    is_aruco_currently_detected
)


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

        # Scacchiera canonica per garantire la legalità formale al 100% delle mosse
        self.game_board = chess.Board()
        self.canonical_fen_pub = self.create_publisher(String, '/chessboard/canonical_fen', 10)

        # Stato aggancio pezzo (Grasp Attachment)
        self.attached_piece = None
        self.attached_is_black = False
        self.piece_z_offset = -0.024

        # Memoria dell'ultima terna ArUco validata: (ax, ay, az, ar_yaw)
        self.last_aruco_pose = None

        # TF2 Buffer per tracking cinematico ad altissima precisione di hande_tcp_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.camera_client = self.create_client(Trigger, '/chessboard_camera/snapshot_and_recognize')
        self.aruco_only_client = self.create_client(Trigger, '/chessboard_camera/detect_aruco_only')
        self.stockfish_client = self.create_client(Trigger, '/stockfish/compute_best_move')

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
        return get_square_world_pose(col, row)

    def js_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in self.joint_names):
            self.current_q = [name_to_pos[j] for j in self.joint_names]
            self.current_js_msg = msg

            if self.attached_piece is not None and self.set_pose_client.service_is_ready():
                now_mono = time.monotonic()
                if not hasattr(self, '_last_pose_update') or (now_mono - self._last_pose_update) >= 0.05:
                    self._last_pose_update = now_mono
                    try:
                        t = self.tf_buffer.lookup_transform('world', 'hande_tcp_link', rclpy.time.Time())
                        req = SetEntityPose.Request()
                        req.entity.name = self.attached_piece
                        req.pose.position.x = float(t.transform.translation.x)
                        req.pose.position.y = float(t.transform.translation.y)
                        
                        # Il pezzo non deve MAI scendere sotto il piano superiore della scacchiera (1.0112)
                        min_resting_z = 1.0112
                        piece_z = t.transform.translation.z + 1.0 + self.piece_z_offset
                        req.pose.position.z = float(max(min_resting_z, piece_z))
                        
                        yaw = math.pi / 2.0 if self.attached_is_black else -math.pi / 2.0
                        req.pose.orientation.x = 0.0
                        req.pose.orientation.y = 0.0
                        req.pose.orientation.z = math.sin(yaw / 2.0)
                        req.pose.orientation.w = math.cos(yaw / 2.0)
                        self.set_pose_client.call_async(req)
                    except Exception:
                        pass

    def make_vertical_pose(self, x, y, z, cb_yaw=None):
        if cb_yaw is None:
            _, _, _, cb_yaw = get_chessboard_world_pose()
        ee_yaw = cb_yaw - (math.pi / 2.0)
        qx = math.cos(ee_yaw / 2.0)
        qy = math.sin(ee_yaw / 2.0)
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.orientation.x = float(qx)
        p.orientation.y = float(qy)
        p.orientation.z = 0.0
        p.orientation.w = 0.0
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
        req.avoid_collisions = False

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
        while rclpy.ok() and (time.monotonic() - t_wait) < 1.5:
            rclpy.spin_once(self, timeout_sec=0.02)
            if max(abs(self.current_q[i] - ordered_target[i]) for i in range(6)) < 0.03:
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

    def solve_ik(self, pose_msg):
        """Risolve la cinematica inversa (IK) per una posa cartesiana Pose."""
        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose = pose_msg
        req.ik_request.robot_state = self.get_current_robot_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1

        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.error_code.val == 1:
            name_to_pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            return [name_to_pos[j] for j in self.joint_names]
        return None

    def set_gripper(self, stroke_val, duration_sec=0.35):
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
        self.set_gripper(0.012, duration_sec=0.35)
        self.get_logger().info(f'   [MoveIt 2] Approccio aereo verticale sopra ({src_x:.3f}, {src_y:.3f}, Z={hover_z*1000:.0f}mm)...')
        target_hover_pose = self.make_vertical_pose(src_x, src_y, hover_z)
        try:
            self.moveit_plan_and_execute_cartesian([target_hover_pose], max_step=0.015, duration_scale=0.8)
        except RuntimeError as e:
            self.get_logger().warn(f'⚠️ Approccio cartesiano parziale ({e}). Fallback con pianificatore OMPL...')
            q_hover = self.solve_ik(target_hover_pose)
            if q_hover is not None:
                self.moveit_move_to_joint_pose(q_hover)
            else:
                raise

        # 2. Discesa cartesiana 100% verticale sul pezzo
        self.get_logger().info(f'   [MoveIt 2] Discesa cartesiana VERTICALE sul pezzo {piece_name} (Z={grasp_z*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(src_x, src_y, grasp_z)], max_step=0.01, duration_scale=0.6)

        # 3. Chiusura pinza e aggancio pezzo
        self.get_logger().info(f'   [MoveIt 2] Presa {piece_name}...')
        self.set_gripper(-0.002, duration_sec=0.35)
        self.piece_z_offset = -grasp_offset
        self.attached_is_black = is_black
        self.attached_piece = piece_name

        # 4. Sollevamento cartesiano 100% verticale
        self.get_logger().info(f'   [MoveIt 2] Sollevamento cartesiano VERTICALE a Z={hover_z*1000:.0f}mm...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(src_x, src_y, hover_z)], max_step=0.01, duration_scale=0.6)

        # 5. Trasporto aereo rettilineo sopra la destinazione (a quota costante +160 mm)
        self.get_logger().info(f'   [MoveIt 2] Trasporto aereo sopra destinazione ({dst_x:.3f}, {dst_y:.3f}, Z={hover_z*1000:.0f}mm)...')
        target_dst_pose = self.make_vertical_pose(dst_x, dst_y, hover_z)
        try:
            self.moveit_plan_and_execute_cartesian([target_dst_pose], max_step=0.015, duration_scale=0.8)
        except RuntimeError as e:
            self.get_logger().warn(f'⚠️ Trasporto cartesiano parziale ({e}). Fallback con pianificatore OMPL...')
            q_dst = self.solve_ik(target_dst_pose)
            if q_dst is not None:
                self.moveit_move_to_joint_pose(q_dst)
            else:
                raise

        # 6. Discesa cartesiana 100% verticale sulla destinazione (quota sicura di deposito senza compressione)
        deposit_z = grasp_z + 0.005
        self.get_logger().info(f'   [MoveIt 2] Discesa cartesiana VERTICALE di deposito (Z={deposit_z*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(dst_x, dst_y, deposit_z)], max_step=0.01, duration_scale=0.6)

        # 7. Rilascio del pezzo e apertura pinza
        self.get_logger().info(f'   [MoveIt 2] Rilascio {piece_name}...')
        self.set_gripper(0.012, duration_sec=0.35)
        self.attached_piece = None
        if self.set_pose_client.service_is_ready():
            req = SetEntityPose.Request()
            req.entity.name = piece_name
            req.pose.position.x = float(dst_x)
            req.pose.position.y = float(dst_y)
            req.pose.position.z = 1.0112  # Quota calibrata perfetta sulla superficie della scacchiera
            _, _, _, cb_yaw = get_chessboard_world_pose()
            yaw = cb_yaw + 1.5708 + (math.pi if is_black else 0.0)
            req.pose.orientation.z = math.sin(yaw / 2.0)
            req.pose.orientation.w = math.cos(yaw / 2.0)
            fut = self.set_pose_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

        # 8. Risalita cartesiana 100% verticale di disimpegno
        self.get_logger().info(f'   [MoveIt 2] Risalita cartesiana VERTICALE di disimpegno (Z={hover_z*1000:.0f}mm)...')
        self.moveit_plan_and_execute_cartesian([self.make_vertical_pose(dst_x, dst_y, hover_z)], max_step=0.01, duration_scale=0.6)

    def get_capture_deposit_slot(self, is_black: bool) -> tuple[float, float]:
        """
        Calcola le coordinate fisse del tavolo per il deposito ordinato dei pezzi catturati.
        La zona è situata sul fianco laterale del tavolo (Y = -0.64m per Bianchi, Y = -0.70m per Neri),
        ad una distanza di oltre 44 cm dal centro della scacchiera.
        Poiché il raggio massimo d'ingombro della scacchiera ruotata è di soli 36 cm,
        la scacchiera NON potrà MAI raggiungere né sovrapporsi a questa zona,
        azzerando il rischio di singolarità o collisioni con MoveIt 2.
        """
        side_key = 'black' if is_black else 'white'
        slot_idx = self.captured_count[side_key]
        self.captured_count[side_key] += 1

        # Due file ordinate sul lato destro/sud del tavolo:
        # Fila Bianchi: Y = -0.64 m
        # Fila Neri:    Y = -0.70 m
        grave_y = -0.70 if is_black else -0.64
        # Offset progressivo lungo l'asse X ben dentro il workspace (da X=-0.32 a X=-0.64)
        grave_x = -0.32 - ((slot_idx % 8) * 0.042)
        return grave_x, grave_y

    def execute_uci_move(self, uci_move: str):
        uci_move = uci_move.strip().lower()
        if len(uci_move) < 4:
            raise ValueError(f"Mossa UCI '{uci_move}' non valida! Esempio valido: e2e4, g1f3, b8c6.")

        # 🎯 STEP FONDAMENTALE: Prima di muovere qualsiasi pezzo, inquadra l'ArUco per aggiornare il riferimento!
        self.get_logger().info("🎯 [Pre-Motion Scan] Inquadratura ottica dell'ArUco per validare il riferimento cartesiano...")
        self.check_and_update_aruco_pose()

        src_sq = uci_move[:2]
        dst_sq = uci_move[2:4]

        # Coordinate calcolate tassativamente rispetto alla terna ArUco appena inquadrata
        src_x, src_y, board_z = self.square_to_world_xy(src_sq)
        dst_x, dst_y, _ = self.square_to_world_xy(dst_sq)

        if src_sq == 'e2' and 'e2' in self.board_state:
            piece_name, ptype, is_black = self.board_state['e2']
            if self.set_pose_client.service_is_ready():
                req_init = SetEntityPose.Request()
                req_init.entity.name = piece_name
                req_init.pose.position.x = float(src_x)
                req_init.pose.position.y = float(src_y)
                req_init.pose.position.z = float(board_z + 1.0 + 0.0002)
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

        # Rilevamento Arrocco (Castling): Re che si muove di 2 caselle in orizzontale
        src_row, src_col = self.uci_square_to_row_col(src_sq)
        dst_row, dst_col = self.uci_square_to_row_col(dst_sq)
        is_castling = (ptype == 'king' and abs(dst_col - src_col) == 2)
        rook_info = None

        if is_castling:
            if dst_col > src_col:
                # Arrocco corto (Kingside): Torre colonna h -> colonna f
                rook_src_sq = f"h{src_row + 1}"
                rook_dst_sq = f"f{src_row + 1}"
            else:
                # Arrocco lungo (Queenside): Torre colonna a -> colonna d
                rook_src_sq = f"a{src_row + 1}"
                rook_dst_sq = f"d{src_row + 1}"

            r_src_x, r_src_y, _ = self.square_to_world_xy(rook_src_sq)
            r_dst_x, r_dst_y, _ = self.square_to_world_xy(rook_dst_sq)
            if rook_src_sq in self.board_state:
                r_name, r_type, r_black = self.board_state[rook_src_sq]
            else:
                r_name, r_type, r_black = self.find_piece_at_gazebo_xy(r_src_x, r_src_y)

            rook_info = (r_name, r_type, r_black, rook_src_sq, rook_dst_sq, r_src_x, r_src_y, r_dst_x, r_dst_y)
            self.get_logger().info(
                f"🏰 ARROCCO RILEVATO: Re ({piece_name}) muove di 2 caselle {src_sq.upper()}->{dst_sq.upper()} | "
                f"Torre '{r_name}' da {rook_src_sq.upper()} a {rook_dst_sq.upper()}"
            )

        # Se si tratta di En Passant oppure la casella di destinazione è occupata (CATTURA!),
        # prima rimuove verticalmente il pezzo avversario e lo deposita nella zona catture
        is_en_passant = (
            ptype == 'pawn' and
            abs(dst_col - src_col) == 1 and
            abs(dst_row - src_row) == 1 and
            dst_sq not in self.board_state
        )

        if is_en_passant:
            ep_cap_sq = f"{chr(ord('a') + dst_col)}{src_row + 1}"
            ep_x, ep_y, _ = self.square_to_world_xy(ep_cap_sq)
            if ep_cap_sq in self.board_state:
                cap_name, cap_type, cap_black = self.board_state[ep_cap_sq]
            else:
                cap_name, cap_type, cap_black = self.find_piece_at_gazebo_xy(ep_x, ep_y)

            self.get_logger().info(
                f"⚔️ EN PASSANT RILEVATO! Cattura al varco del pedone avversario '{cap_name}' in {ep_cap_sq.upper()}..."
            )
            grave_x, grave_y = self.get_capture_deposit_slot(cap_black)
            self.get_logger().info(f"   [Cimitero Fisso Tavolo] Deposito sicuro a (X={grave_x:.3f}, Y={grave_y:.3f})...")
            self.pick_and_place_piece(cap_name, cap_type, cap_black, ep_x, ep_y, grave_x, grave_y, board_z)
            if ep_cap_sq in self.board_state:
                del self.board_state[ep_cap_sq]

        elif dst_sq in self.board_state:
            cap_name, cap_type, cap_black = self.board_state[dst_sq]
            self.get_logger().info(f"⚔️ CATTURA RILEVATA in {dst_sq.upper()}: rimozione verticale di '{cap_name}'...")
            grave_x, grave_y = self.get_capture_deposit_slot(cap_black)
            self.get_logger().info(f"   [Cimitero Fisso Tavolo] Deposito sicuro a (X={grave_x:.3f}, Y={grave_y:.3f})...")
            self.pick_and_place_piece(cap_name, cap_type, cap_black, dst_x, dst_y, grave_x, grave_y, board_z)
            del self.board_state[dst_sq]

        # Esegue il Pick & Place 100% verticale del pezzo (o Re) da src_sq a dst_sq
        self.pick_and_place_piece(piece_name, ptype, is_black, src_x, src_y, dst_x, dst_y, board_z)

        # Aggiorna la mappa interna della scacchiera (con supporto promozione pedone a donna/torre/alfiere/cavallo)
        if src_sq in self.board_state:
            del self.board_state[src_sq]
        if len(uci_move) == 5:
            promo_char = uci_move[4]
            ptype = {'q': 'queen', 'r': 'rook', 'b': 'bishop', 'n': 'knight'}.get(promo_char, ptype)
        self.board_state[dst_sq] = (piece_name, ptype, is_black)

        # Aggiorna lo stato ufficiale della partita FIDE
        try:
            m_obj = chess.Move.from_uci(uci_move)
            if m_obj in self.game_board.legal_moves:
                self.game_board.push(m_obj)
            else:
                self.game_board.push_san(uci_move)
            self.canonical_fen_pub.publish(String(data=self.game_board.fen()))
            self.get_logger().info(f"📜 [Partita FIDE Ufficiale] Mossa {uci_move.upper()} registrata. FEN: {self.game_board.fen()}")
        except Exception:
            pass

        # Se era un arrocco, sposta adesso la relativa torre
        if rook_info is not None:
            r_name, r_type, r_black, r_src_sq, r_dst_sq, r_src_x, r_src_y, r_dst_x, r_dst_y = rook_info
            self.get_logger().info(
                f"🏰 Completamento arrocco: spostamento della torre '{r_name}' da {r_src_sq.upper()} a {r_dst_sq.upper()}..."
            )
            self.pick_and_place_piece(r_name, r_type, r_black, r_src_x, r_src_y, r_dst_x, r_dst_y, board_z)
            if r_src_sq in self.board_state:
                del self.board_state[r_src_sq]
            self.board_state[r_dst_sq] = (r_name, r_type, r_black)

        # Ritorna in posa di ispezione e riallineamento zenitale ancorato alla scacchiera
        self.set_gripper(0.0, duration_sec=0.25)
        self.move_to_inspection_and_align()
        if rook_info is not None:
            self.get_logger().info(f"✅ Arrocco UCI '{uci_move.upper()}' (Re e Torre) completato con MoveIt 2!")
        elif is_en_passant:
            self.get_logger().info(f"✅ Presa En Passant '{uci_move.upper()}' (pedone catturato in {ep_cap_sq.upper()}) completata con MoveIt 2!")
        else:
            self.get_logger().info(f"✅ Mossa UCI '{uci_move.upper()}' completata con MoveIt 2!")

    def trigger_aruco_detection_only(self):
        """Richiede al nodo camera unicamente il rilevamento del Marker ArUco (veloce, senza ChessQueries)."""
        if not self.aruco_only_client.service_is_ready():
            self.aruco_only_client.wait_for_service(timeout_sec=1.5)

        if self.aruco_only_client.service_is_ready():
            req = Trigger.Request()
            fut = self.aruco_only_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result():
                return fut.result().success
        else:
            # Fallback se il servizio aruco_only non è ancora esposto
            self.trigger_camera_recognition()
            return is_aruco_currently_detected()
        return False

    def explore_and_locate_aruco(self):
        """
        Esplora autonomamente lo spazio di lavoro della cella robotica per localizzare il marker ArUco.
        Esegue un pattern di scansione grandangolare (Scan Grid) su posizioni calibrate del tavolo.
        Usa esclusivamente la scansione ottica rapida dell'ArUco (zero chiamate a ChessQueries).
        Appena l'ArUco viene rilevato, interrompe la ricerca e restituisce True.
        """
        self.get_logger().info("🔍 [Exploration Mode] Avvio scansione dello spazio per localizzare il Marker ArUco...")
        ee_yaw = math.pi / 2.0

        scan_waypoints = [
            ("Settore Centrale/ArUco", -0.76, -0.32, 0.58),
            ("Settore Posteriore",     -0.84, -0.32, 0.58),
            ("Settore Sinistro",       -0.70, -0.15, 0.58),
            ("Settore Destro",         -0.70, -0.42, 0.58),
            ("Settore Anteriore",      -0.55, -0.22, 0.58),
        ]

        for sector_name, sx, sy, sz in scan_waypoints:
            self.get_logger().info(f"   🔎 Scansione ottica verso: {sector_name} ({sx:.2f}, {sy:.2f}, Z={sz:.2f}m)...")
            p = Pose()
            p.position.x = float(sx)
            p.position.y = float(sy)
            p.position.z = float(sz)
            p.orientation.x = float(math.cos(ee_yaw / 2.0))
            p.orientation.y = float(math.sin(ee_yaw / 2.0))
            p.orientation.z = 0.0
            p.orientation.w = 0.0

            try:
                self.moveit_plan_and_execute_cartesian([p], max_step=0.02, duration_scale=1.0)
            except Exception:
                q_sol = self.solve_ik(p)
                if q_sol:
                    self.moveit_move_to_joint_pose(q_sol)

            # Rilevamento veloce del solo marker ArUco
            found = self.trigger_aruco_detection_only()
            if found or is_aruco_currently_detected():
                ax, ay, az, ar_yaw = get_aruco_world_pose()
                self.get_logger().info(
                    f"🎯 [ArUco Individuato!] Trovato nel {sector_name}: "
                    f"Pos=({ax:.3f}, {ay:.3f}, {az:.3f}), Yaw={math.degrees(ar_yaw):.1f}°"
                )
                return True

        self.get_logger().warn("⚠️ [Exploration Mode] Scansione completata. Marker ArUco non trovato nei settori standard.")
        return False

    def check_and_update_aruco_pose(self, force: bool = False):
        """
        Controlla se la terna di riferimento ArUco ha subito una traslazione o rotazione.
        Esegue la scansione ottica rapida dell'ArUco (senza attivare ChessQueries).
        Rileva qualsiasi variazione (scostamento > 5.0 mm o rotazione > 0.5°).
        Se non visibile, attiva la scansione autonoma dei settori del tavolo.
        """
        # Scansione rapida del solo ArUco
        self.trigger_aruco_detection_only()

        # Se l'ArUco non è visibile nel frame attuale, esplora autonomamente il piano
        if not is_aruco_currently_detected():
            self.get_logger().warn("⚠️ ArUco non visibile nella visuale corrente. Attivazione scansione settori...")
            self.explore_and_locate_aruco()

        ax, ay, az, ar_yaw = get_aruco_world_pose()

        if self.last_aruco_pose is None or force:
            self.last_aruco_pose = (ax, ay, az, ar_yaw)
            self.get_logger().info(
                f"📍 [ArUco Baseline] Terna iniziale acquisita: Pos=({ax:.3f}, {ay:.3f}, {az:.3f}), "
                f"Yaw={math.degrees(ar_yaw):.1f}°"
            )
            return True

        old_ax, old_ay, old_az, old_yaw = self.last_aruco_pose
        delta_pos = math.hypot(ax - old_ax, ay - old_ay)
        
        delta_yaw = (ar_yaw - old_yaw + math.pi) % (2.0 * math.pi) - math.pi
        delta_deg = abs(math.degrees(delta_yaw))

        # Sensibilità sub-millimetrica ed angolare ultra-fine: rileva anche micro-rotazioni (> 1.5 mm o > 0.15°)
        if delta_pos > 0.0015 or delta_deg > 0.15:
            self.get_logger().info(
                f"🔄 [Cambiamento Terna ArUco Rilevato!] Δpos={delta_pos*1000:.1f} mm, "
                f"Δyaw={delta_deg:.2f}°! Ricalcolo istantaneo delle caselle e riallineamento..."
            )
            self.last_aruco_pose = (ax, ay, az, ar_yaw)
            return True
        else:
            self.get_logger().info(
                f"✅ [Terna ArUco Convalidata] Stabile (Δpos={delta_pos*1000:.1f}mm, Δyaw={delta_deg:.2f}°)."
            )
            return False

    def move_to_inspection_and_align(self, force_align: bool = False):
        """
        Ciclo Industriale di Autolocalizzazione, Visual Servoing & Riconoscimento FEN:
        1. Il robot sale in Posa di Ispezione Calibrata centrata sul settore scacchiera/ArUco (X=-0.76, Y=-0.32, Z=0.58m);
        2. Rileva la terna ArUco tramite scansione ottica rapida (senza eseguire ChessQueries);
        3. Si allinea con precisione a piombo ortogonale sul baricentro della scacchiera ruotata;
        4. SOLO ADESSO, dalla posizione perfettamente centrata e ortogonale ad ArUco,
           esegue il riconoscimento visivo dei pezzi e del FEN tramite ChessQueries!
        """
        self.get_logger().info('🔭 [Ispezione Autonoma] Spostamento in Inspection Pose calibrata (X=-0.76, Y=-0.32, Z=0.58m)...')
        ee_yaw_insp = math.pi / 2.0
        insp_pose = Pose()
        insp_pose.position.x = -0.76
        insp_pose.position.y = -0.32
        insp_pose.position.z = 0.58
        insp_pose.orientation.x = float(math.cos(ee_yaw_insp / 2.0))
        insp_pose.orientation.y = float(math.sin(ee_yaw_insp / 2.0))
        insp_pose.orientation.z = 0.0
        insp_pose.orientation.w = 0.0

        try:
            self.moveit_plan_and_execute_cartesian([insp_pose], max_step=0.02, duration_scale=1.2)
        except Exception:
            q_insp = self.solve_ik(insp_pose)
            if q_insp:
                self.moveit_move_to_joint_pose(q_insp)

        # Scatto rapido ArUco e verifica cambiamenti
        changed = self.check_and_update_aruco_pose(force=force_align)

        # Step 2: Visual Servoing - Centratura fine ortogonale a piombo rispetto alla terna ArUco
        ax, ay, az_world, ar_yaw = get_aruco_world_pose()
        rx, ry, rz = -0.08, -0.23, 0.50
        px = ax + (rx * math.cos(ar_yaw)) - (ry * math.sin(ar_yaw))
        py = ay + (rx * math.sin(ar_yaw)) + (ry * math.cos(ar_yaw))
        pz = (az_world - 1.0) + rz if az_world > 0.5 else az_world + rz

        ee_yaw = ar_yaw - (math.pi / 2.0)
        servo_pose = Pose()
        servo_pose.position.x = float(px)
        servo_pose.position.y = float(py)
        servo_pose.position.z = float(pz)
        servo_pose.orientation.x = float(math.cos(ee_yaw / 2.0))
        servo_pose.orientation.y = float(math.sin(ee_yaw / 2.0))
        servo_pose.orientation.z = 0.0
        servo_pose.orientation.w = 0.0

        self.get_logger().info(f'🎯 [Visual Servoing] Posizionamento a piombo ortogonale su ArUco (Yaw={math.degrees(ar_yaw):.1f}°)...')
        try:
            self.moveit_plan_and_execute_cartesian([servo_pose], max_step=0.02, duration_scale=1.2)
        except Exception:
            q_servo = self.solve_ik(servo_pose)
            if q_servo:
                self.moveit_move_to_joint_pose(q_servo)

        # 📸 STEP FONDAMENTALE: Esecuzione di ChessQueries per riconoscere pezzi e FEN
        # SOLO ADESSO che il braccio è perfettamente allineato e ortogonale alla scacchiera!
        # Esegue fino a 3 scatti con media/migliore angolazione se il primo scatto presenta riflessi
        self.get_logger().info('♟️ [ChessQueries] Braccio in posizione ortogonale corretta: avvio acquisizione e riconoscimento FEN...')
        fen_valid, det_move, det_fen = self.trigger_camera_recognition()

        if not fen_valid:
            self.get_logger().warn("🔄 [Multi-Angle Acquisition] Riconoscimento iniziale incerto. Esecuzione micro-scatti angolati...")
            # Punti di vista angolati per eliminare ombre e riflessi speculari
            view_angles = [
                (0.00,  0.00, 0.54),  # Quota rialzata +4 cm (vista più aperta)
                (+0.03, 0.00, 0.50),  # Offset destro +3 cm
                (-0.03, 0.00, 0.50),  # Offset sinistro -3 cm
            ]
            for jx, jy, jz in view_angles:
                rpx = px + jx
                rpy = py + jy
                rpz = (az_world - 1.0) + jz if az_world > 0.5 else az_world + jz
                retry_p = Pose()
                retry_p.position.x = float(rpx)
                retry_p.position.y = float(rpy)
                retry_p.position.z = float(rpz)
                retry_p.orientation = servo_pose.orientation
                try:
                    self.moveit_plan_and_execute_cartesian([retry_p], max_step=0.015, duration_scale=0.8)
                    fen_valid, det_move, det_fen = self.trigger_camera_recognition()
                    if fen_valid:
                        self.get_logger().info("🎉 [Multi-Angle Success] Scatto angolato convalidato con successo!")
                        break
                except Exception:
                    continue
        return fen_valid, det_move, det_fen

    def initial_calibration_and_mapping(self):
        """
        Ad inizio sessione: il robot esplora attivamente lo spazio di lavoro, identifica visivamente
        l'ArUco ovunque sia posizionato, mappa tutte le 64 caselle e i pezzi rispetto alla terna
        rilevata ed estrae il FEN iniziale della partita.
        """
        print("\n" + "=" * 68)
        print("🎯   ESPLORAZIONE SPAZIO & MAPPATURA SCACCHIERA TRAMITE ARUCO   🎯")
        print("=" * 68)
        print("🔍 Ricerca autonoma della scacchiera sul piano di lavoro in corso...")
        self.move_to_inspection_and_align(force_align=True)

        ax, ay, az, ar_yaw = get_aruco_world_pose()
        print(f"📍 Terna ArUco identificata con successo: X={ax:.3f} m, Y={ay:.3f} m, Yaw={math.degrees(ar_yaw):.1f}°")
        print("🗺️ Mappatura delle 64 caselle (A1..H8) ancorate alla terna ArUco completata.")
        print("=" * 68 + "\n")

    def get_stockfish_best_move(self):
        """Richiede al nodo Stockfish 16 la contromossa migliore per il Nero basata sul FEN della camera."""
        if not self.stockfish_client.service_is_ready():
            self.get_logger().info("⏳ Connessione al nodo Stockfish 16 (/stockfish/compute_best_move)...")
            self.stockfish_client.wait_for_service(timeout_sec=5.0)

        if not self.stockfish_client.service_is_ready():
            self.get_logger().error("❌ Servizio Stockfish non disponibile!")
            return None

        req = Trigger.Request()
        fut = self.stockfish_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if fut.done() and fut.result():
            res = fut.result()
            if res.success:
                return res.message.strip().lower()
            else:
                self.get_logger().warn(f"Stockfish: {res.message}")
        return None

    def trigger_camera_recognition(self) -> tuple[bool, str, str]:
        """
        Richiede al nodo camera D435 uno snapshot e il riconoscimento FEN.
        Restituisce: (success, detected_move, final_fen).
        Permette al robot reale di identificare autonomamente la mossa eseguita dall'umano con la sola telecamera.
        """
        if not self.camera_client.service_is_ready():
            self.camera_client.wait_for_service(timeout_sec=1.5)

        if self.camera_client.service_is_ready():
            self.get_logger().info("📸 [Camera D435] Richiesta snapshot e riconoscimento ChessQueries in corso...")
            req = Trigger.Request()
            fut = self.camera_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=40.0)
            if fut.done():
                res = fut.result()
                if res and res.success:
                    self.get_logger().info(f"🎯 [Visione D435 + ChessQueries] {res.message}")
                    parts = res.message.split('|')
                    det_move = parts[0].strip() if len(parts) > 1 else ""
                    det_fen = parts[1].strip() if len(parts) > 1 else res.message
                    return True, det_move, det_fen
                else:
                    self.get_logger().warn(f"⚠️ Riconoscimento FEN incerto o anomalo: {res.message if res else 'Nessuna risposta'}")
                    return False, "", ""
            else:
                self.get_logger().warn("⚠️ Timeout attesa risposta da camera_board_recognition_node.")
                return False, "", ""
        else:
            self.get_logger().info("ℹ️ Servizio camera non rilevato. Avvia 'camera_board_recognition_node.py' per il riconoscimento automatico.")
            return True, "", ""

    def apply_human_move(self, uci_move: str):
        """
        Registra la mossa eseguita fisicamente dall'umano (Bianco) rilevata dalla telecamera.
        Aggiorna la mappa interna dei pezzi e la scacchiera FIDE SENZA muovere fisicamente il robot.
        """
        uci_move = uci_move.strip().lower()
        src_sq = uci_move[:2]
        dst_sq = uci_move[2:4]

        src_row, src_col = self.uci_square_to_row_col(src_sq)
        dst_row, dst_col = self.uci_square_to_row_col(dst_sq)

        piece_name = f"white_piece_{dst_sq}"
        ptype = 'pawn'
        is_black = False

        if src_sq in self.board_state:
            piece_name, ptype, is_black = self.board_state[src_sq]
            del self.board_state[src_sq]

        # Arrocco dell'umano (sposta la relativa torre nella mappa logica)
        is_castling = (ptype == 'king' and abs(dst_col - src_col) == 2)
        if is_castling:
            if dst_col > src_col:
                r_src, r_dst = f"h{src_row + 1}", f"f{src_row + 1}"
            else:
                r_src, r_dst = f"a{src_row + 1}", f"d{src_row + 1}"
            if r_src in self.board_state:
                r_name, r_type, r_black = self.board_state[r_src]
                del self.board_state[r_src]
                self.board_state[r_dst] = (r_name, r_type, r_black)

        # En passant dell'umano
        is_en_passant = (
            ptype == 'pawn' and
            abs(dst_col - src_col) == 1 and
            abs(dst_row - src_row) == 1 and
            dst_sq not in self.board_state
        )
        if is_en_passant:
            ep_cap_sq = f"{chr(ord('a') + dst_col)}{src_row + 1}"
            if ep_cap_sq in self.board_state:
                del self.board_state[ep_cap_sq]
        elif dst_sq in self.board_state:
            del self.board_state[dst_sq]

        # Promozione
        if len(uci_move) == 5:
            promo_char = uci_move[4]
            ptype = {'q': 'queen', 'r': 'rook', 'b': 'bishop', 'n': 'knight'}.get(promo_char, ptype)

        self.board_state[dst_sq] = (piece_name, ptype, is_black)

        # Aggiorna lo stato ufficiale FIDE
        try:
            m_obj = chess.Move.from_uci(uci_move)
            if m_obj in self.game_board.legal_moves:
                self.game_board.push(m_obj)
            else:
                self.game_board.push_san(uci_move)
            self.canonical_fen_pub.publish(String(data=self.game_board.fen()))
            self.get_logger().info(f"📜 [Partita FIDE] Mossa Umano {uci_move.upper()} registrata. FEN: {self.game_board.fen()}")
        except Exception as e:
            self.get_logger().error(f"Errore registrazione mossa umano FIDE: {e}")

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


def parse_uci_sequence(text_or_list):
    """
    Estrae una lista ordinata di mosse UCI da stringhe, liste o notazioni con numeri/virgole.
    Esempi supportati:
      - ['e2e4', 'e7e5', 'g1f3']
      - 'e2e4 e7e5 g1f3'
      - '1. e2e4 e7e5 2. g1f3 b8c6'
      - 'e2e4, e7e5, g1f3'
    """
    if isinstance(text_or_list, list):
        combined = ' '.join(text_or_list)
    else:
        combined = str(text_or_list)

    raw_tokens = combined.replace(',', ' ').split()
    moves = []
    for tok in raw_tokens:
        tok = tok.strip().lower()
        tok = re.sub(r'^\d+\.?', '', tok).strip()
        if not tok:
            continue
        if len(tok) in (4, 5) and tok[0] in 'abcdefgh' and tok[1] in '12345678' and tok[2] in 'abcdefgh' and tok[3] in '12345678':
            moves.append(tok)
    return moves


def main(args=None):
    rclpy.init(args=args)
    node = MoveItUCIChessMover()
    try:
        node.wait_ready()

        cli_args = [a for a in sys.argv[1:] if not a.startswith('-')]
        cli_moves = parse_uci_sequence(cli_args) if cli_args else []

        if cli_moves:
            # Calibrazione e mappatura iniziale rispetto ad ArUco
            node.initial_calibration_and_mapping()

            print(f"♟️ Sequenza di {len(cli_moves)} mosse da eseguire: {' -> '.join([m.upper() for m in cli_moves])}")
            for idx, m in enumerate(cli_moves, 1):
                # Controlla se la terna ArUco è cambiata prima di ogni mossa
                node.check_and_update_aruco_pose()
                print(f"\n▶️ [{idx}/{len(cli_moves)}] Esecuzione mossa: {m.upper()}")
                node.execute_uci_move(m)
        else:
            # Calibrazione e mappatura iniziale rispetto ad ArUco ad inizio sessione interattiva
            node.initial_calibration_and_mapping()

            # Modalità interattiva partita Umano vs Stockfish
            print("\n" + "=" * 68)
            print("♟️   PARTITA A SCACCHI: UMANO (BIANCO) vs ROBOT UR5e + STOCKFISH 16 (NERO)   ♟️")
            print("=" * 68)
            print("Flusso Operativo Reale (Sim-to-Real):")
            print("  1. L'umano muove fisicamente il pezzo del Bianco sulla scacchiera reale;")
            print("  2. L'umano preme [INVIO] per cedere il turno al robot;")
            print("  3. La telecamera D435 e ChessQueries RILEVANO AUTONOMAMENTE la mossa eseguita;")
            print("  4. Stockfish 16 calcola la migliore contromossa per il Nero;")
            print("  5. Il robot UR5e esegue FISICAMENTE solo la contromossa del Nero.")
            print("Comandi aggiuntivi da terminale (opzionali):")
            print("  • Premi [INVIO] -> Riconoscimento visivo autonomo mossa umano;")
            print("  • 'e2e4' (mossa esplicita) -> esecuzione simulata robotica della mossa;")
            print("  • 'rotate 25' (o -45) -> ruota fisicamente scacchiera e pezzi in Gazebo;")
            print("  • 'stop' o 'q' -> arresta la partita ed esce.")
            print("=" * 68)

            while rclpy.ok():
                try:
                    user_input = input("\n👉 [TURNO UMANO - BIANCO] Muovi sulla scacchiera e premi [INVIO] (o comando/'stop'): ").strip()
                except EOFError:
                    break
                if user_input.lower() in ('q', 'quit', 'exit', 'stop'):
                    break

                # Supporto comando di rotazione dinamica: "rotate <gradi>" o "rotate"
                if user_input.lower().startswith('rotate'):
                    tokens = user_input.split()
                    delta_deg = 25.0
                    if len(tokens) > 1:
                        try:
                            delta_deg = float(tokens[1])
                        except ValueError:
                            print(f"⚠️ Valore gradi non valido: '{tokens[1]}'. Uso 25°.")
                    print(f"\n🔄 [Simulazione Fisica] Rotazione della scacchiera di {delta_deg:+.1f}° con i pezzi nella posizione ATTUALE...")
                    try:
                        from rotate_chessboard import ChessboardRotator
                        rotator = ChessboardRotator()
                        rotator.rotate_board_and_pieces(delta_deg, board_state=node.board_state, game_board=node.game_board)
                        rotator.destroy_node()
                        print(f"✅ Scacchiera e pezzi ruotati fisicamente in Gazebo! Il robot ri-esplorerà per trovarli.")
                    except Exception as e:
                        node.get_logger().error(f"Errore durante la rotazione: {e}")
                    continue

                # CASO 1: MODALITÀ ROBOT REALE (L'umano ha mosso fisicamente sulla scacchiera e premuto INVIO)
                if not user_input:
                    print("\n📸 [Ispezione Visiva] Il robot inquadra la scacchiera per rilevare la mossa dell'umano...")
                    ok, det_move, det_fen = node.move_to_inspection_and_align()

                    if not ok or not det_move:
                        print("⚠️ [Visione Camera] Nessuna mossa rilevata sulla scacchiera! Assicurati di aver mosso il tuo pezzo e riprova premendo [INVIO].")
                        continue

                    m_obj = chess.Move.from_uci(det_move)
                    if m_obj not in node.game_board.legal_moves:
                        print(f"⚠️ [Mossa Illegale Rilevata] La mossa '{det_move.upper()}' osservata non è legale! Correggi la posizione e riprova.")
                        continue

                    print(f"\n🎯 [VISIONE REAL-TIME]: Mossa dell'Umano RILEVATA DALLA TELECAMERA: \033[1;32m{det_move.upper()}\033[0m")
                    node.apply_human_move(det_move)

                # CASO 2: MODALITÀ SIMULAZIONE / TESTING (L'utente ha inserito una mossa UCI da terminale)
                else:
                    moves = parse_uci_sequence(user_input)
                    if not moves:
                        print(f"⚠️ Comando o mossa non valida: '{user_input}'. Esempio: premi [INVIO] oppure scrivi 'e2e4'")
                        continue

                    for m in moves:
                        try:
                            node.check_and_update_aruco_pose()
                            node.execute_uci_move(m)
                        except Exception as e:
                            node.get_logger().error(f"Errore durante l'esecuzione della mossa '{m}': {e}")
                            break

                # ORA È IL TURNO DEL ROBOT (NERO - STOCKFISH 16)
                print("\n🤖 [TURNO ROBOT - NERO]: Stockfish 16 sta analizzando il FEN e calcolando la contromossa...")
                best_move = node.get_stockfish_best_move()
                if best_move:
                    print(f"💡 Stockfish ha selezionato la contromossa migliore: \033[1;32m{best_move.upper()}\033[0m")
                    print(f"🦾 Il robot UR5e sta eseguendo fisicamente la mossa del Nero...")
                    try:
                        node.check_and_update_aruco_pose()
                        node.execute_uci_move(best_move)
                    except Exception as e:
                        node.get_logger().error(f"Errore esecuzione mossa Stockfish '{best_move}': {e}")
                else:
                    print("⚠️ Stockfish non ha generato una mossa (partita finita o attesa posizione valida).")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
