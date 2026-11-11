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


import os
import sys
import time
import math
import re
import subprocess
import json
import chess
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from std_srvs.srv import Trigger

from spawn_utils import (
    get_chessboard_gazebo_pose,
    get_aruco_world_pose,
    get_photo_shoot_world_pose,
    get_square_world_pose
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

    def get_latest_fen_or_board(self):
        """Recupera l'ultimo FEN noto della partita o mappa da moveit_uci_mover / detected_chessboard_pose.json."""
        # Prova a leggere l'ultimo FEN da file snapshot/log se presente
        try:
            with open("/workspace/ChessUr5e/detected_chessboard_pose.json", "r") as f:
                data = json.load(f)
                if "latest_fen" in data:
                    return data["latest_fen"]
        except Exception:
            pass
        return None

    def rotate_board_and_pieces(self, delta_degrees: float, board_state: dict = None, game_board: chess.Board = None):
        delta_rad = math.radians(delta_degrees)
        cx, cy, cz, current_yaw = get_chessboard_gazebo_pose()
        new_yaw = current_yaw + delta_rad

        print("\n" + "=" * 65)
        print(f"🔄 ROTAZIONE SCACCHIERA IN CORSO: {delta_degrees:+.1f}°")
        print("=" * 65)
        print(f"📍 Centro scacchiera: ({cx:.4f}, {cy:.4f}, {cz:.4f})")
        print(f"🧭 Yaw precedente:   {math.degrees(current_yaw):.1f}° ({current_yaw:.4f} rad)")
        print(f"🧭 Nuovo Yaw:         {math.degrees(new_yaw):.1f}° ({new_yaw:.4f} rad)")

        # 1. Ruota il modello della scacchiera in Gazebo
        self.set_model_pose('chessboard', cx, cy, cz, new_yaw)
        print("✅ Scacchiera ruotata in Gazebo.")

        # 2. Ruota tutti i pezzi presenti nella posizione ATTUALE della partita
        SQUARE_SIZE = 0.05875
        A1_X = 0.21290
        A1_Y = 0.21745

        # Se abbiamo board_state (da moveit_uci_mover: { 'e4': ('white_pawn_4', 'pawn', False) }), usiamo quella!
        pieces_to_move = []
        if board_state:
            for sq, (piece_name, ptype, is_black) in board_state.items():
                col = ord(sq[0]) - ord('a')
                row = int(sq[1]) - 1
                pieces_to_move.append((piece_name, row, col, is_black))
        elif game_board:
            # Deriva la mappatura dei modelli dal game_board FIDE
            type_counters = {}
            for sq, piece in game_board.piece_map().items():
                col = chess.square_file(sq)
                row = chess.square_rank(sq)
                color = 'white' if piece.color == chess.WHITE else 'black'
                ptype = chess.piece_name(piece.piece_type)
                key = (color, ptype)
                cnt = type_counters.get(key, 0) + 1
                type_counters[key] = cnt
                if ptype == 'pawn':
                    name = f'{color}_pawn_{cnt-1}'
                elif ptype in ('queen', 'king'):
                    name = f'{color}_{ptype}'
                else:
                    name = f'{color}_{ptype}_{cnt}'
                pieces_to_move.append((name, row, col, piece.color == chess.BLACK))
        else:
            latest_fen = self.get_latest_fen_or_board()
            if latest_fen:
                try:
                    gb = chess.Board(latest_fen)
                    type_counters = {}
                    for sq, piece in gb.piece_map().items():
                        col = chess.square_file(sq)
                        row = chess.square_rank(sq)
                        color = 'white' if piece.color == chess.WHITE else 'black'
                        ptype = chess.piece_name(piece.piece_type)
                        key = (color, ptype)
                        cnt = type_counters.get(key, 0) + 1
                        type_counters[key] = cnt
                        if ptype == 'pawn':
                            name = f'{color}_pawn_{cnt-1}'
                        elif ptype in ('queen', 'king'):
                            name = f'{color}_{ptype}'
                        else:
                            name = f'{color}_{ptype}_{cnt}'
                        pieces_to_move.append((name, row, col, piece.color == chess.BLACK))
                except Exception:
                    pass

            if not pieces_to_move:
                # Fallback 32 pezzi standard
                for i in range(8):
                    pieces_to_move.append((f'white_pawn_{i}', 1, i, False))
                    pieces_to_move.append((f'black_pawn_{i}', 6, i, True))
                back = [
                    ('rook_1', 0), ('knight_1', 1), ('bishop_1', 2), ('queen', 3),
                    ('king', 4), ('bishop_2', 5), ('knight_2', 6), ('rook_2', 7)
                ]
                for p, c in back:
                    pieces_to_move.append((f'white_{p}', 0, c, False))
                    pieces_to_move.append((f'black_{p}', 7, c, True))

        for name, row, col, is_black in pieces_to_move:
            lx = A1_X - (col * SQUARE_SIZE)
            ly = A1_Y - (row * SQUARE_SIZE)

            gx = cx + (lx * math.cos(new_yaw)) - (ly * math.sin(new_yaw))
            gy = cy + (lx * math.sin(new_yaw)) + (ly * math.cos(new_yaw))
            gz = cz + 0.0014
            p_yaw = new_yaw + 1.5708 + (math.pi if is_black else 0.0)

            self.set_model_pose(name, gx, gy, gz, p_yaw)

        print(f"✅ {len(pieces_to_move)} pezzi riposizionati e ruotati solidalmente con la scacchiera nella posizione attuale.")

        # 3. Calcolo e stampa nuova posa ArUco e posa Home Panoramica (SOLO se richiesta calibrazione)
        # NOTA BENE: Nessuna chiamata a camera_client per non "suggerire" la posa al robot!
        # Il robot deve accorgersene da solo esplorando lo spazio tramite telecamera.
        ARUCO_LOCAL_X = 0.28
        ARUCO_LOCAL_Y = 0.23
        ax = cx + (ARUCO_LOCAL_X * math.cos(new_yaw)) - (ARUCO_LOCAL_Y * math.sin(new_yaw))
        ay = cy + (ARUCO_LOCAL_X * math.sin(new_yaw)) + (ARUCO_LOCAL_Y * math.cos(new_yaw))

        print("-" * 65)
        print("🎯 STATO FISICO GAZEBO AGGIORNATO (All'insaputa del robot):")
        print(f"  • Nuova posizione fisica ArUco:      X={ax:.4f}, Y={ay:.4f}, Yaw={math.degrees(new_yaw):.1f}°")
        print("=" * 65)


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
