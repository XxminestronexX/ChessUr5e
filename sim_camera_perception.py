#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import subprocess
import re
import math

class SimCameraPerception(Node):
    def __init__(self):
        super().__init__('sim_camera_perception')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.SQUARE_SIZE = 0.057
        self.pieces_map = self._build_chess_matrix()
        self.timer = self.create_timer(0.05, self.perception_loop)
        self.get_logger().info("📷 Telecamera Intel RealSense Attiva: Flusso TF Eseguito!")

    def _build_chess_matrix(self):
        names = ['rook', 'knight', 'bishop', 'queen', 'king', 'bishop', 'knight', 'rook']
        setup = []
        for col in range(8):
            setup.append({'name': f'white_{names[col]}_{col}', 'file': col, 'rank': 0, 'black': False})
            setup.append({'name': f'white_pawn_{col}', 'file': col, 'rank': 1, 'black': False})
            setup.append({'name': f'black_pawn_{col}', 'file': col, 'rank': 6, 'black': True})
            setup.append({'name': f'black_{names[col]}_{col}', 'file': col, 'rank': 7, 'black': True})
        return setup

    def get_gazebo_pose(self, model_name):
        try:
            output = subprocess.check_output(f"gz model -m {model_name} --pose", shell=True).decode('utf-8')
            for line in output.split('\n'):
                if "Pose [" in line and "|" in line:
                    parts = re.findall(r'\[([^\]]+)\]', line)
                    data_parts = [p for p in parts if '|' in p]
                    return [float(x.strip()) for x in data_parts[0].split('|')], [float(x.strip()) for x in data_parts[1].split('|')]
        except: pass
        return None, None

    def force_gazebo_pose(self, name, x, y, z, yaw):
        cmd = f"gz service -s /world/default/entity/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: \"{name}\", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{yaw: {yaw}}}'"
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def perception_loop(self):
        cb_xyz, cb_rpy = self.get_gazebo_pose("chessboard")
        if not cb_xyz or not cb_rpy: return

        cx, cy, cz, board_yaw = cb_xyz[0], cb_xyz[1], cb_xyz[2], cb_rpy[2]

        # Pubblica TF Scacchiera (world -> chessboard)
        t_board = TransformStamped()
        t_board.header.stamp = self.get_clock().now().to_msg()
        t_board.header.frame_id = 'world'
        t_board.child_frame_id = 'chessboard'
        t_board.transform.translation.x, t_board.transform.translation.y, t_board.transform.translation.z = cx, cy, cz
        t_board.transform.rotation.z = math.sin(board_yaw / 2.0)
        t_board.transform.rotation.w = math.cos(board_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t_board)

        # Pubblica TF Pezzi (chessboard -> pezzi) e sincronizza la fisica di Gazebo
        for p in self.pieces_map:
            loc_x = (p['file'] - 3.5) * self.SQUARE_SIZE
            loc_y = (3.5 - p['rank']) * self.SQUARE_SIZE
            p_local_yaw = math.pi if p['black'] else 0.0

            # 1. TF ROS 2
            t_piece = TransformStamped()
            t_piece.header.stamp = self.get_clock().now().to_msg()
            t_piece.header.frame_id = 'chessboard'  # 🔥 AGGANCIO DA TF FIGLIO LEGITTIMO
            t_piece.child_frame_id = p['name']
            t_piece.transform.translation.x, t_piece.transform.translation.y, t_piece.transform.translation.z = loc_x, loc_y, 0.015
            t_piece.transform.rotation.z = math.sin(p_local_yaw / 2.0)
            t_piece.transform.rotation.w = math.cos(p_local_yaw / 2.0)
            self.tf_broadcaster.sendTransform(t_piece)

            # 2. Tracking Gazebo
            world_x = cx + loc_x * math.cos(board_yaw) - loc_y * math.sin(board_yaw)
            world_y = cy + loc_x * math.sin(board_yaw) + loc_y * math.cos(board_yaw)
            self.force_gazebo_pose(p['name'], world_x, world_y, cz + 0.015, board_yaw + p_local_yaw)

def main(args=None):
    rclpy.init(args=args)
    node = SimCameraPerception()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()