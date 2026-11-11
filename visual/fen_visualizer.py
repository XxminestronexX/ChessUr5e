#!/usr/bin/env python3
"""
visual/fen_visualizer.py
--------------------------------------------------------------------------------
Visualizzatore modulare stand-alone della scacchiera a partire dal FEN ricavato
dalla telecamera o pubblicato sui topic ROS 2.

Mostra:
  1. Grafica 2D renderizzata stile scacchiera ufficiale (con colori e simboli dei pezzi);
  2. Snapshot reale catturato dalla telecamera D435 affiancato;
  3. FEN attuale con turno e stato legale;
  4. Finestra OpenCV aggiornata in tempo reale ad ogni nuovo FEN ricevuto.

Completamente isolato e modulare: può essere rimosso cancellando semplicemente
la cartella 'visual/' senza intaccare nessun file del progetto.
"""

import os
import sys
import time
import json
import chess
import numpy as np
import cv2

# Prova ad agganciarsi a ROS 2 se disponibile
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    ROS_AVAILABLE = True
except ImportError:
    pass


UNICODE_PIECES = {
    'P': 'P', 'N': 'N', 'B': 'B', 'R': 'R', 'Q': 'Q', 'K': 'K',
    'p': 'p', 'n': 'n', 'b': 'b', 'r': 'r', 'q': 'q', 'k': 'k'
}

PIECE_LABELS = {
    'P': ('P', (255, 255, 255), (40, 40, 40)),
    'N': ('N', (255, 255, 255), (40, 40, 40)),
    'B': ('B', (255, 255, 255), (40, 40, 40)),
    'R': ('R', (255, 255, 255), (40, 40, 40)),
    'Q': ('Q', (255, 255, 255), (40, 40, 40)),
    'K': ('K', (255, 255, 255), (40, 40, 40)),
    'p': ('p', (20, 20, 20), (220, 220, 220)),
    'n': ('n', (20, 20, 20), (220, 220, 220)),
    'b': ('b', (20, 20, 20), (220, 220, 220)),
    'r': ('r', (20, 20, 20), (220, 220, 220)),
    'q': ('q', (20, 20, 20), (220, 220, 220)),
    'k': ('k', (20, 20, 20), (220, 220, 220)),
}


def draw_chessboard_from_fen(fen_str: str, square_px: int = 60) -> np.ndarray:
    """Disegna un'immagine OpenCV (RGB) della scacchiera 8x8 dal FEN."""
    board_size = square_px * 8
    margin = 35
    total_w = board_size + margin * 2
    total_h = board_size + margin * 2

    # Canvas di sfondo elegante (grigio scuro)
    img = np.full((total_h, total_w, 3), (35, 35, 35), dtype=np.uint8)

    # Colori caselle stile torneo (Verde / Bianco crema)
    color_light = (205, 230, 235)  # BGR
    color_dark = (60, 140, 75)     # BGR

    try:
        board = chess.Board(fen_str)
    except Exception:
        board = chess.Board()

    # Disegna le 64 caselle
    for rank in range(8):
        for file in range(8):
            is_light = (rank + file) % 2 == 0
            sq_color = color_light if is_light else color_dark

            x0 = margin + file * square_px
            y0 = margin + (7 - rank) * square_px
            x1 = x0 + square_px
            y1 = y0 + square_px

            cv2.rectangle(img, (x0, y0), (x1, y1), sq_color, -1)
            cv2.rectangle(img, (x0, y0), (x1, y1), (100, 100, 100), 1)

            # Pezzo sulla casella
            sq = chess.square(file, rank)
            piece = board.piece_at(sq)
            if piece is not None:
                sym = piece.symbol()
                label, fg_color, bg_color = PIECE_LABELS.get(sym, (sym, (0, 0, 0), (255, 255, 255)))

                # Cerchio indicatore pezzo (Bianco o Nero)
                center_x = x0 + square_px // 2
                center_y = y0 + square_px // 2
                radius = square_px // 2 - 6

                # Sfondo pezzo con bordo
                cv2.circle(img, (center_x, center_y), radius, fg_color, -1)
                cv2.circle(img, (center_x, center_y), radius, (0, 0, 0), 2)

                # Lettera del pezzo centrata
                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = 0.85
                thickness = 2
                (tw, th), _ = cv2.getTextSize(label, font, font_scale, thickness)
                text_x = center_x - tw // 2
                text_y = center_y + th // 2
                cv2.putText(img, label, (text_x, text_y), font, font_scale, bg_color, thickness, cv2.LINE_AA)

    # Coordinate perimetrali (a-h, 1-8)
    font_coord = cv2.FONT_HERSHEY_SIMPLEX
    for f in range(8):
        col_letter = chr(ord('a') + f)
        cx = margin + f * square_px + square_px // 2 - 6
        cv2.putText(img, col_letter, (cx, margin - 10), font_coord, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, col_letter, (cx, total_h - 12), font_coord, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

    for r in range(8):
        row_num = str(r + 1)
        cy = margin + (7 - r) * square_px + square_px // 2 + 5
        cv2.putText(img, row_num, (12, cy), font_coord, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, row_num, (total_w - margin + 12, cy), font_coord, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

    return img


def create_comparison_dashboard(fen_str: str, snapshot_path: str = None) -> np.ndarray:
    """Crea un pannello affiancato: [Foto Camera D435 Reale] vs [Rappresentazione FEN Riconosciuto]."""
    board_img = draw_chessboard_from_fen(fen_str, square_px=65)
    bh, bw = board_img.shape[:2]

    # Carica la foto reale dalla telecamera D435 se presente
    if snapshot_path and os.path.exists(snapshot_path):
        cam_raw = cv2.imread(snapshot_path)
        if cam_raw is not None:
            # Ridimensiona proporzionalmente alla stessa altezza
            cam_aspect = cam_raw.shape[1] / cam_raw.shape[0]
            target_w = int(bh * cam_aspect)
            cam_view = cv2.resize(cam_raw, (target_w, bh))
        else:
            cam_view = np.full((bh, bh, 3), 40, dtype=np.uint8)
            cv2.putText(cam_view, "Camera Snapshot Non Valido", (30, bh // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    else:
        cam_view = np.full((bh, bh, 3), 40, dtype=np.uint8)
        cv2.putText(cam_view, "In attesa Snapshot D435...", (40, bh // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)

    # Intestazione superiore informativa
    header_h = 75
    total_w = cam_view.shape[1] + bw
    total_h = header_h + bh
    dashboard = np.zeros((total_h, total_w, 3), dtype=np.uint8)

    # Header banner
    cv2.rectangle(dashboard, (0, 0), (total_w, header_h), (25, 25, 25), -1)
    cv2.putText(dashboard, "VERIFICA VISIVA: REALTA' (D435) vs STATO FEN RICONOSCIUTO", (20, 30),
                cv2.FONT_HERSHEY_DUPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)

    fen_display = f"FEN: {fen_str}" if len(fen_str) < 65 else f"FEN: {fen_str[:62]}..."
    cv2.putText(dashboard, fen_display, (20, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

    # Incolla le due viste affiancate
    dashboard[header_h:header_h + bh, 0:cam_view.shape[1]] = cam_view
    dashboard[header_h:header_h + bh, cam_view.shape[1]:total_w] = board_img

    # Etichette sui due pannelli
    cv2.rectangle(dashboard, (10, header_h + 10), (280, header_h + 40), (0, 0, 0), -1)
    cv2.putText(dashboard, "1. Vista Telecamera D435 (Gazebo)", (15, header_h + 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.rectangle(dashboard, (cam_view.shape[1] + 10, header_h + 10), (cam_view.shape[1] + 310, header_h + 40), (0, 0, 0), -1)
    cv2.putText(dashboard, "2. FEN Riconosciuto (ChessQueries)", (cam_view.shape[1] + 15, header_h + 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

    return dashboard


if ROS_AVAILABLE:
    class FENVisualizerNode(Node):
        def __init__(self):
            super().__init__('fen_visualizer_node')
            self.current_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
            self.project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            self.snapshot_file = os.path.join(self.project_dir, "latest_chessboard_snapshot.jpg")

            self.sub = self.create_subscription(
                String,
                '/chessboard/detected_fen',
                self.fen_callback,
                10
            )
            self.timer = self.create_timer(0.2, self.update_display)
            self.window_name = "ChessUr5e - Verifica FEN Camera vs Stato Reale"
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1200, 650)
            self.get_logger().info("🖥️ [Visualizer] Nodo attivo su /chessboard/detected_fen!")

        def fen_callback(self, msg: String):
            new_fen = msg.data.strip()
            if new_fen and new_fen != self.current_fen:
                self.current_fen = new_fen
                self.get_logger().info(f"📥 [Visualizer] Nuovo FEN ricevuto: {self.current_fen}")

        def update_display(self):
            # Cerca lo snapshot più recente
            snap_path = self.snapshot_file if os.path.exists(self.snapshot_file) else None
            dash = create_comparison_dashboard(self.current_fen, snap_path)
            cv2.imshow(self.window_name, dash)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                rclpy.shutdown()


def run_standalone_loop():
    """Modalità autonoma senza ROS 2: legge periodicamente lo snapshot e il FEN su disco."""
    project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    snap_path = os.path.join(project_dir, "latest_chessboard_snapshot.jpg")
    pose_file = os.path.join(project_dir, "detected_chessboard_pose.json")

    window_name = "ChessUr5e - Verifica Visiva FEN (Standalone)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1200, 650)

    last_mtime = 0
    current_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

    print("=" * 65)
    print("🖥️   VISUALIZZATORE MODULARE FEN (MODALITÀ FILE WATCHER)   🖥️")
    print("=" * 65)
    print("Premi 'q' sulla finestra per uscire.\n")

    while True:
        # Controlla se il file JSON o lo snapshot sono stati aggiornati
        if os.path.exists(pose_file):
            try:
                mtime = os.path.getmtime(pose_file)
                if mtime > last_mtime:
                    last_mtime = mtime
                    with open(pose_file, 'r') as f:
                        data = json.load(f)
                        if "latest_fen" in data:
                            current_fen = data["latest_fen"]
            except Exception:
                pass

        # Genera dashboard
        active_snap = snap_path if os.path.exists(snap_path) else None
        dash = create_comparison_dashboard(current_fen, active_snap)
        cv2.imshow(window_name, dash)

        key = cv2.waitKey(200) & 0xFF
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()


def main():
    if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
        # Modalità a riga di comando: ./show_fen.sh "rnbqkbnr/..."
        fen_arg = sys.argv[1]
        project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        snap = os.path.join(project_dir, "latest_chessboard_snapshot.jpg")
        dash = create_comparison_dashboard(fen_arg, snap)
        cv2.imshow("Verifica Singola FEN", dash)
        print("Premi un tasto per chiudere...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    # Se ROS 2 è attivo e inizializzato, usa il nodo con topic live
    if ROS_AVAILABLE:
        try:
            rclpy.init()
            node = FENVisualizerNode()
            rclpy.spin(node)
            node.destroy_node()
            cv2.destroyAllWindows()
            return
        except Exception:
            pass

    # Fallback modalità standalone continua
    run_standalone_loop()


if __name__ == '__main__':
    main()
