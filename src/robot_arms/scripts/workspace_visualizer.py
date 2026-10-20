#!/usr/bin/env python3
import math
import subprocess
import numpy as np
import PyKDL

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, JointState
from std_msgs.msg import Header
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import sensor_msgs_py.point_cloud2 as pc2
from urdf_parser_py.urdf import URDF


def urdf_pose_to_kdl_frame(origin):
    if origin is None:
        return PyKDL.Frame()
    xyz = origin.xyz if origin.xyz else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin.rpy else [0.0, 0.0, 0.0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
        PyKDL.Vector(xyz[0], xyz[1], xyz[2])
    )


class RobotWorkspaceVisualizer(Node):
    def __init__(self):
        super().__init__('robot_workspace_visualizer')
        self.get_logger().info('Inizializzazione Workspace Visualizer UR5e (Modalità Pick & Place Verticale)...')

        # Coordinate ottimali scacchiera su tavolo (Z = 0.0 m)
        self.board_cx = -0.58
        self.board_cy = -0.22
        self.table_surface_z = 0.0
        self.sq_size = 0.057

        # Costruzione catena cinematica esatta da Xacro/URDF (fino al TCP delle dita a +0.22m da tool0)
        self.build_kinematic_chain()

        # Inizializzazione solver KDL esatti con limiti di giunto (NR_JL)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(self.chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_NR_JL(
            self.chain, self.q_min_kdl, self.q_max_kdl,
            self.fk_solver, self.ik_vel_solver, 300, 1e-4
        )

        # Orientamento di presa perfettamente VERTICALE:
        # Asse Z del TCP parallelo e opposto all'asse Z del pezzo (Roll=pi, Pitch=0, Yaw=pi/2)
        self.vert_rot = PyKDL.Rotation.RPY(math.pi, 0.0, math.pi / 2.0)

        # Posa iniziale: pinza perfettamente verticale sopra il centro della scacchiera (-0.58, -0.22, 0.22)
        self.current_q = [0.335, 0.771, -2.282, -0.377, 2.302, 1.113]
        self.seeds = [
            [0.335, 0.771, -2.282, -0.377, 2.302, 1.113],
            [-0.188, 0.209, -2.465, 0.869, 2.339, 1.834],
            [0.7, 0.83, -2.18, -1.10, 2.20, 0.43],
            [1.55, -1.18, -1.22, 2.12, -2.69, 2.39],
            [-0.15, -1.14, -1.47, 2.93, -2.14, 1.81],
            [0.8, -2.0, 2.0, 1.5, -2.5, -1.4],
        ]

        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/robot_workspace/pointcloud', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/robot_workspace/markers', 1)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Precomputazione nuvola di punti del Workspace 3D
        self.get_logger().info('Generazione del volume 3D di raggiungibilità (PointCloud)...')
        self.workspace_cloud_msg = self.generate_workspace_cloud(num_samples=35000)

        # Verifica rigorosa Pick & Place verticale su tutte le 64 caselle
        self.get_logger().info(
            f'Verifica IK Pick & Place VERTICALE su tutte le 64 caselle in ({self.board_cx}, {self.board_cy})...'
        )
        self.markers_msg = self.generate_workspace_markers()

        # Server Interactive Marker
        self.server = InteractiveMarkerServer(self, 'workspace_probe_server')
        self.setup_interactive_marker()
        self.server.applyChanges()

        # Timer per pubblicazione periodica dello stato del braccio e marker
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.cloud_timer = self.create_timer(1.0, self.publish_cloud_and_markers)

        self.get_logger().info('✅ Workspace Visualizer pronto! Muovi la sonda in RViz per testare pose verticali.')

    def build_kinematic_chain(self):
        urdf_str = subprocess.check_output(
            "xacro $(ros2 pkg prefix robot_arms)/share/robot_arms/urdf/uclv_right_ur5e.urdf.xacro",
            shell=True
        ).decode('utf-8')
        robot = URDF.from_xml_string(urdf_str)

        parent_map = {j.child: j for j in robot.joints}
        tip_link_name = "tool0"
        base_link_name = "world"

        curr = tip_link_name
        joints_in_chain = []
        while curr != base_link_name:
            if curr not in parent_map:
                break
            j = parent_map[curr]
            joints_in_chain.append(j)
            curr = j.parent
        joints_in_chain.reverse()

        self.chain = PyKDL.Chain()
        self.joint_names = []
        self.q_min = []
        self.q_max = []

        for j in joints_in_chain:
            f = urdf_pose_to_kdl_frame(j.origin)
            # In URDF la trasformazione di orig precede la rotazione attorno all'asse del giunto
            self.chain.addSegment(
                PyKDL.Segment(j.name + "_orig", PyKDL.Joint(j.name + "_fix", PyKDL.Joint.Fixed), f)
            )
            if j.type in ("revolute", "continuous"):
                axis = PyKDL.Vector(j.axis[0], j.axis[1], j.axis[2]) if j.axis else PyKDL.Vector(0, 0, 1)
                joint = PyKDL.Joint(j.name, PyKDL.Vector(0, 0, 0), axis, PyKDL.Joint.RotAxis)
                self.chain.addSegment(PyKDL.Segment(j.child, joint, PyKDL.Frame()))
                self.joint_names.append(j.name)
                if j.limit:
                    self.q_min.append(j.limit.lower)
                    self.q_max.append(j.limit.upper)
                else:
                    self.q_min.append(-2 * math.pi)
                    self.q_max.append(2 * math.pi)

        # Aggiungiamo il segmento fisso del TCP (centro di presa tra i polpastrelli della Hand-E: +0.22m lungo Z di tool0)
        self.chain.addSegment(
            PyKDL.Segment("tcp", PyKDL.Joint("tcp_fixed", PyKDL.Joint.Fixed), PyKDL.Frame(PyKDL.Vector(0, 0, 0.22)))
        )

        self.q_min = np.array(self.q_min)
        self.q_max = np.array(self.q_max)
        self.num_joints = len(self.joint_names)

        self.q_min_kdl = PyKDL.JntArray(self.num_joints)
        self.q_max_kdl = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints):
            self.q_min_kdl[i] = self.q_min[i]
            self.q_max_kdl[i] = self.q_max[i]

    def solve_vertical_ik(self, x, y, z, custom_rot=None):
        rot = custom_rot if custom_rot is not None else self.vert_rot
        target = PyKDL.Frame(rot, PyKDL.Vector(x, y, z))

        if z < self.table_surface_z - 0.005 or y > 0.45:
            return False, None, 0.0

        q_init = PyKDL.JntArray(self.num_joints)
        q_out = PyKDL.JntArray(self.num_joints)

        trial_seeds = [self.current_q] + self.seeds
        for s in trial_seeds:
            for i in range(self.num_joints):
                q_init[i] = s[i]
            if self.ik_solver.CartToJnt(q_init, target, q_out) >= 0:
                sol = [q_out[i] for i in range(self.num_joints)]
                elbow_margin = min(abs(sol[2]), math.pi - abs(sol[2]))
                return True, sol, elbow_margin

        return False, None, 0.0

    def generate_workspace_cloud(self, num_samples=35000):
        qs = np.random.uniform(self.q_min, self.q_max, (num_samples, self.num_joints))
        q_kdl = PyKDL.JntArray(self.num_joints)
        f_out = PyKDL.Frame()

        points = []
        base_pos = np.array([-0.188189, 0.13575, 0.556631])

        for i in range(num_samples):
            for k in range(self.num_joints):
                q_kdl[k] = qs[i, k]
            self.fk_solver.JntToCart(q_kdl, f_out)
            x, y, z = f_out.p.x(), f_out.p.y(), f_out.p.z()

            if z < self.table_surface_z or y > 0.45:
                continue

            dist_3d = math.sqrt((x - base_pos[0])**2 + (y - base_pos[1])**2 + (z - base_pos[2])**2)
            reach_ratio = np.clip((dist_3d - 0.20) / 0.85, 0.0, 1.0)

            r = int(np.clip(255 * (reach_ratio * 1.4), 0, 255))
            g = int(np.clip(255 * (1.2 - abs(reach_ratio - 0.35) * 1.8), 0, 255))
            b = int(np.clip(255 * (0.9 - reach_ratio * 1.2), 0, 255))
            rgb = (r << 16) | (g << 8) | b

            points.append([x, y, z, float(rgb)])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        header = Header()
        header.frame_id = 'world'
        header.stamp = self.get_clock().now().to_msg()
        return pc2.create_cloud(header, fields, points)

    def generate_workspace_markers(self):
        markers = MarkerArray()
        cx, cy, cz = self.board_cx, self.board_cy, self.table_surface_z
        sq_size = self.sq_size

        # 1. Base Scacchiera Ottimizzata (-0.58, -0.22, 0.0)
        board_box = Marker()
        board_box.header.frame_id = 'world'
        board_box.ns = 'chessboard_base'
        board_box.id = 1
        board_box.type = Marker.CUBE
        board_box.action = Marker.ADD
        board_box.pose.position.x = cx
        board_box.pose.position.y = cy
        board_box.pose.position.z = cz + 0.00625
        board_box.scale.x = 0.51
        board_box.scale.y = 0.51
        board_box.scale.z = 0.0125
        board_box.color.r = 0.35
        board_box.color.g = 0.22
        board_box.color.b = 0.12
        board_box.color.a = 0.95
        markers.markers.append(board_box)

        # 2. Test rigoroso su ogni casella per:
        #    - Presa bassa verticale (Z = 0.035m, asse pinza // asse Z pezzo)
        #    - Sollevamento verticale anti-collisione sopra gli altri pezzi (Z = 0.160m, > 95mm altezza Re)
        reachable_count = 0
        min_elbow_deg = 180.0
        marker_id = 100

        for r in range(8):
            cols = range(8) if r % 2 == 0 else range(7, -1, -1)
            for c in cols:
                sq_x = cx + (c - 3.5) * sq_size
                sq_y = cy + (r - 3.5) * sq_size

                ok_grasp, sol_grasp, elb_margin = self.solve_vertical_ik(sq_x, sq_y, cz + 0.035)
                ok_hover = False
                if ok_grasp:
                    # Verifica anche che dalla posa di presa possa salire verticalmente a Z=0.16m senza urtare pezzi vicini
                    self.seeds.insert(0, sol_grasp)
                    if len(self.seeds) > 15:
                        self.seeds.pop()
                    ok_hover, _, _ = self.solve_vertical_ik(sq_x, sq_y, cz + 0.160)

                is_valid = ok_grasp and ok_hover
                if is_valid:
                    reachable_count += 1
                    min_elbow_deg = min(min_elbow_deg, math.degrees(elb_margin))

                # Cilindro sulla casella
                sq_marker = Marker()
                sq_marker.header.frame_id = 'world'
                sq_marker.ns = 'chessboard_squares'
                sq_marker.id = marker_id
                marker_id += 1
                sq_marker.type = Marker.CYLINDER
                sq_marker.action = Marker.ADD
                sq_marker.pose.position.x = sq_x
                sq_marker.pose.position.y = sq_y
                sq_marker.pose.position.z = cz + 0.015
                sq_marker.scale.x = sq_size * 0.80
                sq_marker.scale.y = sq_size * 0.80
                sq_marker.scale.z = 0.005

                if is_valid:
                    sq_marker.color.r = 0.1
                    sq_marker.color.g = 0.92
                    sq_marker.color.b = 0.25
                    sq_marker.color.a = 0.85
                else:
                    sq_marker.color.r = 0.95
                    sq_marker.color.g = 0.1
                    sq_marker.color.b = 0.1
                    sq_marker.color.a = 0.9

                markers.markers.append(sq_marker)

                # Freccia verticale che mostra il vettore di approccio verticale (da Z=0.14m a Z=0.035m)
                if is_valid and (r in (0, 1, 6, 7)):  # Mostra frecce sulle righe di partenza dei 32 pezzi
                    arrow = Marker()
                    arrow.header.frame_id = 'world'
                    arrow.ns = 'vertical_grasp_vectors'
                    arrow.id = marker_id + 200
                    arrow.type = Marker.ARROW
                    arrow.action = Marker.ADD
                    from geometry_msgs.msg import Point
                    p_top = Point(x=sq_x, y=sq_y, z=cz + 0.11)
                    p_bot = Point(x=sq_x, y=sq_y, z=cz + 0.025)
                    arrow.points = [p_top, p_bot]
                    arrow.scale.x = 0.005
                    arrow.scale.y = 0.010
                    arrow.scale.z = 0.012
                    arrow.color.r = 0.0
                    arrow.color.g = 0.85
                    arrow.color.b = 1.0
                    arrow.color.a = 0.75
                    markers.markers.append(arrow)

        # 3. Testo riepilogativo
        info_marker = Marker()
        info_marker.header.frame_id = 'world'
        info_marker.ns = 'workspace_info'
        info_marker.id = 3
        info_marker.type = Marker.TEXT_VIEW_FACING
        info_marker.action = Marker.ADD
        info_marker.pose.position.x = cx
        info_marker.pose.position.y = cy - 0.34
        info_marker.pose.position.z = cz + 0.18
        info_marker.scale.z = 0.035
        info_marker.color.r = 1.0
        info_marker.color.g = 1.0
        info_marker.color.b = 1.0
        info_marker.color.a = 1.0
        pct = (reachable_count / 64.0) * 100.0
        info_marker.text = (
            f"Configurazione Ottimale ({cx:.2f}, {cy:.2f}): {reachable_count}/64 caselle ({pct:.0f}%) "
            f"Pick&Place Verticale (Margine gomito min: {min_elbow_deg:.1f} deg)"
        )
        markers.markers.append(info_marker)

        self.get_logger().info(
            f'✅ Risultato Configurazione Ottimale ({cx}, {cy}): {reachable_count}/64 caselle ({pct:.1f}%) '
            f'raggiungibili con approccio 100% VERTICALE (margine singolarità gomito >= {min_elbow_deg:.1f}°)'
        )
        return markers

    def setup_interactive_marker(self):
        qx, qy, qz, qw = self.vert_rot.GetQuaternion()

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = 'target_pose_probe'
        int_marker.description = 'Sonda Presa Verticale (Trascina su ogni casella)'
        int_marker.scale = 0.18

        int_marker.pose.position.x = self.board_cx
        int_marker.pose.position.y = self.board_cy
        int_marker.pose.position.z = 0.22
        int_marker.pose.orientation.x = qx
        int_marker.pose.orientation.y = qy
        int_marker.pose.orientation.z = qz
        int_marker.pose.orientation.w = qw

        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.06
        sphere_marker.scale.y = 0.06
        sphere_marker.scale.z = 0.06
        sphere_marker.color.r = 0.1
        sphere_marker.color.g = 0.95
        sphere_marker.color.b = 0.1
        sphere_marker.color.a = 0.9

        status_text = Marker()
        status_text.type = Marker.TEXT_VIEW_FACING
        status_text.scale.z = 0.032
        status_text.pose.position.z = 0.09
        status_text.color.r = 0.1
        status_text.color.g = 1.0
        status_text.color.b = 0.2
        status_text.color.a = 1.0
        status_text.text = f"PRESA VERTICALE OK [X={self.board_cx:.2f}, Y={self.board_cy:.2f}, Z=0.22]"

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(sphere_marker)
        visual_control.markers.append(status_text)
        int_marker.controls.append(visual_control)

        axes = [
            (1.0, 0.0, 0.0, 'move_x', InteractiveMarkerControl.MOVE_AXIS),
            (0.0, 1.0, 0.0, 'move_y', InteractiveMarkerControl.MOVE_AXIS),
            (0.0, 0.0, 1.0, 'move_z', InteractiveMarkerControl.MOVE_AXIS),
            (1.0, 0.0, 0.0, 'rotate_x', InteractiveMarkerControl.ROTATE_AXIS),
            (0.0, 1.0, 0.0, 'rotate_y', InteractiveMarkerControl.ROTATE_AXIS),
            (0.0, 0.0, 1.0, 'rotate_z', InteractiveMarkerControl.ROTATE_AXIS),
        ]
        for ax, ay, az, name, mode in axes:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = ax
            control.orientation.y = ay
            control.orientation.z = az
            control.name = name
            control.interaction_mode = mode
            int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.process_marker_feedback)
        self.interactive_marker = int_marker

    def process_marker_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return

        p = feedback.pose.position
        o = feedback.pose.orientation
        kdl_rot = PyKDL.Rotation.Quaternion(o.x, o.y, o.z, o.w)

        is_reachable, joint_sol, elb_margin = self.solve_vertical_ik(p.x, p.y, p.z, custom_rot=kdl_rot)

        ctrl = self.interactive_marker.controls[0]
        sphere_m = ctrl.markers[0]
        text_m = ctrl.markers[1]
        self.interactive_marker.pose = feedback.pose

        if is_reachable:
            self.current_q = joint_sol
            sphere_m.color.r = 0.1
            sphere_m.color.g = 0.95
            sphere_m.color.b = 0.1
            text_m.color.r = 0.1
            text_m.color.g = 1.0
            text_m.color.b = 0.2
            text_m.text = f"RAGGIUNGIBILE [X={p.x:.2f}, Y={p.y:.2f}, Z={p.z:.2f}] (Gomito: {math.degrees(elb_margin):.0f} deg)"
        else:
            sphere_m.color.r = 0.95
            sphere_m.color.g = 0.1
            sphere_m.color.b = 0.1
            text_m.color.r = 1.0
            text_m.color.g = 0.2
            text_m.color.b = 0.2
            text_m.text = f"NON RAGGIUNGIBILE [X={p.x:.2f}, Y={p.y:.2f}, Z={p.z:.2f}]"

        self.server.erase('target_pose_probe')
        self.server.insert(self.interactive_marker, feedback_callback=self.process_marker_feedback)
        self.server.applyChanges()

    def timer_callback(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names) + ['hande_joint', 'hande_base_to_hande_jaw2_link']
        js.position = [float(q) for q in self.current_q] + [0.018, 0.018]
        self.joint_state_pub.publish(js)

    def publish_cloud_and_markers(self):
        now = self.get_clock().now().to_msg()
        self.workspace_cloud_msg.header.stamp = now
        self.cloud_pub.publish(self.workspace_cloud_msg)

        for m in self.markers_msg.markers:
            m.header.stamp = now
        self.marker_pub.publish(self.markers_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotWorkspaceVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
