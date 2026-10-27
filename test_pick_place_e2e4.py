#!/usr/bin/env python3
"""
Test Pick-and-Place Verticale: muove il pedone bianco da e2 (white_pawn_4) a e4
mantenendo l'end-effector 100% parallelo all'asse Z del pezzo (Roll=pi, Pitch=0, Yaw=pi/2)
per non urtare i pezzi adiacenti.
Sincronizzato sul completamento reale dei goal in Gazebo (nessun anticipo/inversione visiva tra salita e discesa).
"""
import time
import math
import subprocess
import numpy as np
import PyKDL

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
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


class PickPlaceE2E4Node(Node):
    def __init__(self):
        super().__init__(
            'test_pick_place_e2e4',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )
        self.get_logger().info('🤖 Inizializzazione nodo Pick & Place Verticale (e2 -> e4) [use_sim_time=True]...')

        # Costruzione catena cinematica fino al centro di presa tra le dita (tool0 + 0.225m lungo Z)
        self.build_kinematic_chain()
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_NR_JL(
            self.chain, self.q_min_kdl, self.q_max_kdl,
            self.fk_solver, self.ik_vel_solver, 400, 1e-5
        )

        # Orientamento 100% VERTICALE: asse Z pinza antiparallelo all'asse Z del pezzo
        self.vert_rot = PyKDL.Rotation.RPY(math.pi, 0.0, math.pi / 2.0)

        # Configurazione Home Verticale sopra il centro scacchiera
        self.home_q = [0.335, 0.771, -2.282, -0.377, 2.302, 1.113]
        self.current_q = list(self.home_q)

        # Stato aggancio pezzo (Grasp Attachment)
        self.attached_piece = None
        self.piece_z_offset = -0.024  # La base del pedone sta 24mm sotto il centro di presa dei polpastrelli

        # Subscriber stato giunti reale da Gazebo
        self.js_received = False
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_callback, 10)

        # Client servizio SetEntityPose per solidarizzare il pezzo afferrato con le dita durante il trasporto
        self.set_pose_client = self.create_client(SetEntityPose, '/world/empty/set_pose')

        # Action Clients per braccio e pinza
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

    def get_current_tcp_z(self):
        q_kdl = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints):
            q_kdl[i] = self.current_q[i]
        f_tcp = PyKDL.Frame()
        self.fk_solver.JntToCart(q_kdl, f_tcp)
        return f_tcp.p.z()

    def js_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in self.joint_names):
            self.current_q = [name_to_pos[j] for j in self.joint_names]
            self.js_received = True

            # Se un pezzo è stretto tra le dita, aggiorna in tempo reale la sua posa solidale al TCP
            if self.attached_piece is not None and self.set_pose_client.service_is_ready():
                now_mono = time.monotonic()
                if not hasattr(self, '_last_pose_update') or (now_mono - self._last_pose_update) >= 0.06:
                    self._last_pose_update = now_mono
                    q_kdl = PyKDL.JntArray(self.num_joints)
                    for i in range(self.num_joints):
                        q_kdl[i] = self.current_q[i]
                    f_tcp = PyKDL.Frame()
                    self.fk_solver.JntToCart(q_kdl, f_tcp)

                    req = SetEntityPose.Request()
                    req.entity.name = self.attached_piece
                    req.pose.position.x = float(f_tcp.p.x())
                    req.pose.position.y = float(f_tcp.p.y())
                    req.pose.position.z = float(f_tcp.p.z() + 1.0 + self.piece_z_offset)
                    req.pose.orientation.x = 0.0
                    req.pose.orientation.y = 0.0
                    req.pose.orientation.z = math.sin(-math.pi / 4.0)
                    req.pose.orientation.w = math.cos(-math.pi / 4.0)
                    self.set_pose_client.call_async(req)

    def build_kinematic_chain(self):
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
        self.joint_names = []
        q_min, q_max = [], []

        for j in joints_in_chain:
            f = urdf_pose_to_kdl_frame(j.origin)
            self.chain.addSegment(
                PyKDL.Segment(j.name + "_orig", PyKDL.Joint(j.name + "_fix", PyKDL.Joint.Fixed), f)
            )
            if j.type in ("revolute", "continuous"):
                axis = PyKDL.Vector(j.axis[0], j.axis[1], j.axis[2]) if j.axis else PyKDL.Vector(0, 0, 1)
                joint = PyKDL.Joint(j.name, PyKDL.Vector(0, 0, 0), axis, PyKDL.Joint.RotAxis)
                self.chain.addSegment(PyKDL.Segment(j.child, joint, PyKDL.Frame()))
                self.joint_names.append(j.name)
                q_min.append(j.limit.lower if j.limit else -2 * math.pi)
                q_max.append(j.limit.upper if j.limit else 2 * math.pi)

        # TCP = centro di presa dei polpastrelli della pinza Robotiq Hand-E (+0.225m da tool0)
        self.chain.addSegment(
            PyKDL.Segment("tcp", PyKDL.Joint("tcp_fixed", PyKDL.Joint.Fixed), PyKDL.Frame(PyKDL.Vector(0, 0, 0.225)))
        )

        self.num_joints = len(self.joint_names)
        self.q_min_kdl = PyKDL.JntArray(self.num_joints)
        self.q_max_kdl = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints):
            self.q_min_kdl[i] = q_min[i]
            self.q_max_kdl[i] = q_max[i]

    def solve_ik_vertical(self, x, y, z, seed_q=None):
        target = PyKDL.Frame(self.vert_rot, PyKDL.Vector(x, y, z))
        q_init = PyKDL.JntArray(self.num_joints)
        q_out = PyKDL.JntArray(self.num_joints)

        seeds = ([seed_q] if seed_q is not None else []) + [self.current_q, self.home_q]
        for s in seeds:
            for i in range(self.num_joints):
                q_init[i] = s[i]
            if self.ik_solver.CartToJnt(q_init, target, q_out) >= 0:
                return [q_out[i] for i in range(self.num_joints)]
        raise RuntimeError(f"Impossibile risolvere IK verticale per ({x:.3f}, {y:.3f}, {z:.3f})")

    def square_to_robot_frame(self, row, col):
        cx, cy, cz_gz, cb_yaw = get_chessboard_world_pose()
        # Dimensioni esatte estratte dalla mesh 3D della scacchiera (chessboard.obj)
        SQUARE_SIZE = 0.05875
        A1_X = 0.21290
        A1_Y = 0.21745

        lx = A1_X - (col * SQUARE_SIZE)
        ly = A1_Y - (row * SQUARE_SIZE)

        gx = cx + (lx * math.cos(cb_yaw)) - (ly * math.sin(cb_yaw))
        gy = cy + (lx * math.sin(cb_yaw)) + (ly * math.cos(cb_yaw))

        board_top_z = (cz_gz - 1.0) + 0.0012
        return gx, gy, board_top_z

    def wait_sim_seconds(self, sim_sec):
        """Attende sim_sec secondi nell'orologio di simulazione di Gazebo (/clock)."""
        start_ns = self.get_clock().now().nanoseconds
        target_ns = start_ns + int(sim_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < target_ns:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_arm_cartesian_vertical(self, x, y, z_start, z_end, duration_sec=1.6, num_steps=10):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        # Costruisce i punti esclusivamente oltre z_start così non torna mai indietro a z_start!
        zs = np.linspace(z_start, z_end, num_steps + 1)[1:]
        last_q = list(self.current_q)

        for idx, z_val in enumerate(zs):
            q_sol = self.solve_ik_vertical(x, y, float(z_val), seed_q=last_q)
            last_q = q_sol
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q_sol]
            t = duration_sec * (idx + 1) / num_steps
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(pt)

        self.send_and_wait_arm(goal, last_q)

    def move_arm_joint(self, q_target, duration_sec=1.5):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_target]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal.trajectory.points.append(pt)

        self.send_and_wait_arm(goal, q_target)

    def set_gripper(self, stroke_val, duration_sec=0.7):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['hande_joint', 'hande_base_to_hande_jaw2_link']

        pt = JointTrajectoryPoint()
        pt.positions = [float(stroke_val), float(stroke_val)]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal.trajectory.points.append(pt)

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Server gripper_controller non disponibile!")
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if gh and gh.accepted:
            res_future = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
        self.wait_sim_seconds(0.2)

    def send_and_wait_arm(self, goal, target_q):
        """
        Invia il goal al controller del braccio e attende SENZA TIMEOUT FISSO
        finché il controller in Gazebo non ha effettivamente raggiunto il punto finale.
        In questo modo le fasi di discesa e salita non si sovrappongono mai!
        """
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Server arm_controller non disponibile! Assicurati che Gazebo sia avviato.")
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("Goal braccio rifiutato!")

        # Attende la conclusione effettiva dell'azione in Gazebo (indipendentemente dal Real-Time Factor!)
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)

        # Verifica anche che l'errore fisico dei giunti in Gazebo sia < 0.02 rad prima di passare allo step successivo
        t_wait = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t_wait) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            err = max(abs(self.current_q[i] - target_q[i]) for i in range(self.num_joints))
            if err < 0.025:
                break

    def execute_e2_to_e4(self):
        self.get_logger().info('⏳ Attesa stato giunti e clock da Gazebo...')
        while rclpy.ok() and (not self.js_received or self.get_clock().now().nanoseconds == 0):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Calcolo coordinate e2 (row=1, col=4 -> white_pawn_4) ed e4 (row=3, col=4)
        e2_x, e2_y, board_z = self.square_to_robot_frame(row=1, col=4)
        e4_x, e4_y, _ = self.square_to_robot_frame(row=3, col=4)

        hover_z = board_z + 0.160   # 160 mm sopra la scacchiera (quota ALTA: Z = 0.1725 m)
        grasp_z = board_z + 0.024   # 24 mm sopra la scacchiera (quota BASSA di presa: Z = 0.0365 m)

        # Assicuriamoci che il pedone white_pawn_4 sia nella sua casella di partenza e2
        if self.set_pose_client.wait_for_service(timeout_sec=2.0):
            req_reset = SetEntityPose.Request()
            req_reset.entity.name = 'white_pawn_4'
            req_reset.pose.position.x = float(e2_x)
            req_reset.pose.position.y = float(e2_y)
            req_reset.pose.position.z = float(board_z + 1.0 + 0.0002)
            req_reset.pose.orientation.z = math.sin(-math.pi / 4.0)
            req_reset.pose.orientation.w = math.cos(-math.pi / 4.0)
            self.set_pose_client.call_async(req_reset)

        self.get_logger().info(f'📍 Casella e2 (partenza): X={e2_x:.4f}, Y={e2_y:.4f} | Casella e4 (arrivo): X={e4_x:.4f}, Y={e4_y:.4f}')

        # 1. Apertura pinza e posizionamento in Home Verticale (ALTA)
        self.get_logger().info(f'1️⃣ Apertura pinza e allineamento in Home Verticale (Z attuale = {self.get_current_tcp_z()*1000:.0f} mm)...')
        self.set_gripper(0.012, duration_sec=0.6)
        self.move_arm_joint(self.home_q, duration_sec=1.2)

        # 2. Spostamento sopra e2 a quota di sicurezza ALTA (Hover Z = +172 mm)
        self.get_logger().info('2️⃣ Spostamento aereo sopra e2 (Quota ALTA: Z = +172 mm)...')
        q_hover_e2 = self.solve_ik_vertical(e2_x, e2_y, hover_z)
        self.move_arm_joint(q_hover_e2, duration_sec=1.5)
        self.get_logger().info(f'   -> Raggiunto Hover sopra e2: Z_TCP = {self.get_current_tcp_z()*1000:.1f} mm')

        # 3. DISCESA cartesiana puramente VERTICALE su e2 (da +172 mm GIÙ fino a +36 mm)
        self.get_logger().info('3️⃣ DISCESA VERTICALE su e2 (da Z = +172 mm GIÙ a Z = +36 mm)...')
        self.move_arm_cartesian_vertical(e2_x, e2_y, hover_z, grasp_z, duration_sec=1.6, num_steps=10)
        self.get_logger().info(f'   -> Raggiunto punto di presa BASSO su e2: Z_TCP = {self.get_current_tcp_z()*1000:.1f} mm')

        # 4. Chiusura simmetrica pinza sul pedone in e2 e aggancio presa
        self.get_logger().info('4️⃣ Chiusura pinza sul pedone in e2 (white_pawn_4)...')
        self.set_gripper(-0.002, duration_sec=0.7)
        self.attached_piece = 'white_pawn_4'

        # 5. SALITA cartesiana puramente VERTICALE da e2 (da +36 mm SU fino a +172 mm)
        self.get_logger().info('5️⃣ SALITA VERTICALE con il pedone da e2 (da Z = +36 mm SU a Z = +172 mm)...')
        self.move_arm_cartesian_vertical(e2_x, e2_y, grasp_z, hover_z, duration_sec=1.6, num_steps=10)
        self.get_logger().info(f'   -> Raggiunta quota ALTA di trasporto: Z_TCP = {self.get_current_tcp_z()*1000:.1f} mm')

        # 6. Traslazione orizzontale ad alta quota da e2 a e4 (a Z = +172 mm costante)
        self.get_logger().info('6️⃣ Trasporto orizzontale aereo da e2 a e4 (a Z = +172 mm costante)...')
        q_hover_e4 = self.solve_ik_vertical(e4_x, e4_y, hover_z, seed_q=self.current_q)
        self.move_arm_joint(q_hover_e4, duration_sec=1.5)
        self.get_logger().info(f'   -> Raggiunto Hover sopra e4: Z_TCP = {self.get_current_tcp_z()*1000:.1f} mm')

        # 7. DISCESA cartesiana puramente VERTICALE su e4 (da +172 mm GIÙ fino a +37 mm)
        self.get_logger().info('7️⃣ DISCESA VERTICALE sulla casella e4 (da Z = +172 mm GIÙ a Z = +37 mm)...')
        self.move_arm_cartesian_vertical(e4_x, e4_y, hover_z, grasp_z + 0.001, duration_sec=1.6, num_steps=10)
        self.get_logger().info(f'   -> Raggiunto punto di rilascio BASSO su e4: Z_TCP = {self.get_current_tcp_z()*1000:.1f} mm')

        # 8. Rilascio del pedone in e4 (sgancio e apertura pinza)
        self.get_logger().info('8️⃣ Rilascio del pedone in e4 (apertura pinza)...')
        self.attached_piece = None
        if self.set_pose_client.service_is_ready():
            req = SetEntityPose.Request()
            req.entity.name = 'white_pawn_4'
            req.pose.position.x = float(e4_x)
            req.pose.position.y = float(e4_y)
            req.pose.position.z = float(board_z + 1.0 + 0.001)
            req.pose.orientation.z = math.sin(-math.pi / 4.0)
            req.pose.orientation.w = math.cos(-math.pi / 4.0)
            self.set_pose_client.call_async(req)
        self.set_gripper(0.012, duration_sec=0.7)

        # 9. SALITA cartesiana puramente VERTICALE di disimpegno da e4 (da +37 mm SU a +172 mm)
        self.get_logger().info('9️⃣ SALITA VERTICALE di disimpegno da e4 (da Z = +37 mm SU a Z = +172 mm)...')
        self.move_arm_cartesian_vertical(e4_x, e4_y, grasp_z + 0.001, hover_z, duration_sec=1.4, num_steps=8)
        self.get_logger().info(f'   -> Disimpegno completato: Z_TCP = {self.get_current_tcp_z()*1000:.1f} mm')

        # 10. Ritorno in posa Home Verticale
        self.get_logger().info('🔟 Ritorno in posa Home Verticale...')
        self.move_arm_joint(self.home_q, duration_sec=1.4)

        self.get_logger().info('✅ Mossa e2 -> e4 completata con successo e perfettamente sincronizzata!')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceE2E4Node()
    try:
        node.execute_e2_to_e4()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
