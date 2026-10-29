#!/usr/bin/env python3
"""
Script per ruotare o spostare la scacchiera in Gazebo durante il gioco.
Dimostra che il sistema calcola la posa di scatto e le mosse
in modo rigidamente ancorato al marker ArUco.

Uso:
  ./rotate_chessboard.sh 25     (ruota di +25 gradi)
  ./rotate_chessboard.sh -45    (ruota di -45 gradi)
  ./rotate_chessboard.sh 90     (ruota di 90 gradi)
  ./rotate_chessboard.sh        (richiede l'angolo in modo interattivo)
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from std_srvs.srv import Trigger

from spawn_utils import (
    get_chessboard_gazebo_pose,
    get_aruco_world_pose,
    get_photo_shoot_world_pose,
)


class ChessboardRotator(Node):
    def __init__(self):
        super().__init__('chessboard_rotator')
        self.set_pose_client = self.create_client(SetEntityPose, '/world/empty/set_pose')
        self.camera_client = self.create_client(Trigger, '/chessboard_camera/snapshot_and_recognize')

        self.get_logger().info("⏳ Connessione al servizio Gazebo /world/empty/set_pose...")
        self.set_pose_client.wait_for_service(timeout_sec=5.0)

    def set_model_pose(self, name: str, x: float, y: float, z: float, yaw: float):
        req = SetEntityPose.Request()
        req.entity.name = name
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.z = math.sin(yaw / 2.0)
        req.pose.orientation.w = math.cos(yaw / 2.0)
        fut = self.set_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def rotate_board_and_pieces(self, delta_degrees: float):
        delta_rad = math.radians(delta_degrees)
        cx, cy, cz, current_yaw = get_chessboard_gazebo_pose()
        new_yaw = current_yaw + delta_rad

        print("\n" + "=" * 65)
        print(f"🔄 ROTAZIONE SCACCHIERA IN CORSO: {delta_degrees:+.1f}°")
        print("=" * 65)
        print(f"📍 Centro scacchiera: ({cx:.4f}, {cy:.4f}, {cz:.4f})")
        print(f"🧭 Yaw precedente:   {math.degrees(current_yaw):.1f}° ({current_yaw:.4f} rad)")
        print(f"🧭 Nuovo Yaw:         {math.degrees(new_yaw):.1f}° ({new_yaw:.4f} rad)")

        # 1. Ruota il modello della scacchiera
        self.set_model_pose('chessboard', cx, cy, cz, new_yaw)
        print("✅ Scacchiera ruotata in Gazebo.")

        # 2. Ruota tutti i 32 pezzi solidali alla scacchiera
        SQUARE_SIZE = 0.05875
        A1_X = 0.21290
        A1_Y = 0.21745

        # Mappatura dei 32 pezzi standard
        all_pieces = (
            [(f'white_pawn_{i}', 1, i, False) for i in range(8)] +
            [(f'black_pawn_{i}', 6, i, True) for i in range(8)] +
            [('white_rook_1', 0, 0, False), ('white_rook_2', 0, 7, False),
             ('black_rook_1', 7, 0, True), ('black_rook_2', 7, 7, True),
             ('white_knight_1', 0, 1, False), ('white_knight_2', 0, 6, False),
             ('black_knight_1', 7, 1, True), ('black_knight_2', 7, 6, True),
             ('white_bishop_1', 0, 2, False), ('white_bishop_2', 0, 5, False),
             ('black_bishop_1', 7, 2, True), ('black_bishop_2', 7, 5, True),
             ('white_queen', 0, 3, False), ('white_king', 0, 4, False),
             ('black_queen', 7, 3, True), ('black_king', 7, 4, True)]
        )

        for name, row, col, is_black in all_pieces:
            lx = A1_X - (col * SQUARE_SIZE)
            ly = A1_Y - (row * SQUARE_SIZE)

            gx = cx + (lx * math.cos(new_yaw)) - (ly * math.sin(new_yaw))
            gy = cy + (lx * math.sin(new_yaw)) + (ly * math.cos(new_yaw))
            gz = cz + 0.0014
            p_yaw = new_yaw + 1.5708 + (math.pi if is_black else 0.0)

            self.set_model_pose(name, gx, gy, gz, p_yaw)

        print("✅ Tutti i 32 pezzi riposizionati e ruotati solidalmente con la scacchiera.")

        # 3. Calcolo e stampa nuova posa ArUco e posa Home Panoramica
        ax, ay, az, _ = get_aruco_world_pose()
        px, py, pz, _ = get_photo_shoot_world_pose()

        print("-" * 65)
        print("🎯 VERIFICA ANCORAGGIO ARUCO:")
        print(f"  • Posa ArUco Marker aggiornata:      X={ax:.4f}, Y={ay:.4f}, Z={az:.4f}")
        print(f"  • Posa Home Panoramica (da ArUco):   X={px:.4f}, Y={py:.4f}, Z={pz:.4f}")
        print("=" * 65)

        # 4. Scatta una foto per verificare che la telecamera e ChessQueries rilevino la scacchiera ruotata
        time.sleep(0.5)
        if self.camera_client.wait_for_service(timeout_sec=2.0):
            print("\n📸 Richiesta scatto telecamera D435 dalla nuova configurazione...")
            req = Trigger.Request()
            fut = self.camera_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            if fut.done() and fut.result():
                print(f"🎯 Esito riconoscimento: {fut.result().message}")


def main():
    rclpy.init()
    rotator = ChessboardRotator()

    if len(sys.argv) > 1:
        try:
            delta_deg = float(sys.argv[1])
        except ValueError:
            print(f"Angolo '{sys.argv[1]}' non valido. Usa un valore numerico come 25 o -30.")
            return
    else:
        print("\n" + "=" * 65)
        print("♟️  TEST ROTAZIONE SCACCHIERA IN TEMPO REALE DURANTE IL GIOCO  ♟️")
        print("=" * 65)
        try:
            val = input("👉 Inserisci l'angolo di rotazione in gradi (es. 25, -45, 90): ").strip()
            delta_deg = float(val) if val else 25.0
        except (ValueError, EOFError):
            delta_deg = 25.0

    rotator.rotate_board_and_pieces(delta_deg)
    rotator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
