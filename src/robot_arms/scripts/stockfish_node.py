#!/usr/bin/env python3
"""
Nodo ROS 2 per il motore scacchistico Stockfish 16.

Funzionalità:
1. Sottoscrive il topic `/chessboard/detected_fen` prodotto dal nodo di visione (ChessQueries).
2. Fornisce il servizio `/stockfish/compute_best_move` (std_srvs/srv/Trigger):
   - All'attivazione (pressione del pulsante 'fine turno umano'):
   - Analizza la posizione FEN rilevata;
   - Calcola la migliore contromossa tramite il motore scacchistico Stockfish 16;
   - Restituisce la mossa UCI (es. 'e7e5', 'c7c5') e la pubblica su `/stockfish/best_move`.
3. Supporta il livello di abilità configurabile (Skill Level 0-20, default 10 per gioco umano realistico o 20 per massima forza).
"""

import os
import sys
import shutil
import chess
import chess.engine

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger


def find_stockfish_binary() -> str:
    """Individua l'eseguibile di Stockfish nel sistema o nel container."""
    candidates = [
        "/usr/local/bin/stockfish",
        "/usr/games/stockfish",
        "/usr/bin/stockfish",
        shutil.which("stockfish"),
    ]
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return "stockfish"


class StockfishNode(Node):
    def __init__(self):
        super().__init__('stockfish_node')
        self.get_logger().info("🧠 Inizializzazione nodo Stockfish 16 Engine...")

        # Parametri configurabili
        self.declare_parameter('stockfish_path', find_stockfish_binary())
        self.declare_parameter('skill_level', 10)  # da 0 (principiante) a 20 (Grandmaster)
        self.declare_parameter('move_time_sec', 1.0) # tempo di calcolo in secondi
        self.declare_parameter('robot_color', 'black') # il robot muove per il Nero

        self.stockfish_path = self.get_parameter('stockfish_path').get_parameter_value().string_value
        self.skill_level = self.get_parameter('skill_level').get_parameter_value().integer_value
        self.move_time = self.get_parameter('move_time_sec').get_parameter_value().double_value
        self.robot_color = self.get_parameter('robot_color').get_parameter_value().string_value

        self.get_logger().info(f"♟️ Eseguibile Stockfish: {self.stockfish_path} (Skill Level: {self.skill_level}, Robot Color: {self.robot_color})")

        # Inizializzazione motore UCI
        self.engine = None
        self.init_engine()

        # Ultimo FEN ricevuto da ChessQueries
        self.latest_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        self.fen_received = False

        # Sottoscrizione al FEN della telecamera
        self.fen_sub = self.create_subscription(
            String,
            '/chessboard/detected_fen',
            self.fen_callback,
            10
        )

        # Publisher per la migliore mossa calcolata
        self.best_move_pub = self.create_publisher(String, '/stockfish/best_move', 10)

        # Servizio e topic per calcolare la mossa (pulsante umano)
        self.srv = self.create_service(Trigger, '/stockfish/compute_best_move', self.handle_compute_best_move)
        self.trigger_sub = self.create_subscription(Empty, '/stockfish/human_turn_done', self.handle_trigger_topic, 10)

        self.get_logger().info("✅ Nodo Stockfish 16 attivo e pronto su /stockfish/compute_best_move!")

    def init_engine(self):
        try:
            self.engine = chess.engine.SimpleEngine.popen_uci(self.stockfish_path)
            self.engine.configure({"Skill Level": self.skill_level})
            self.get_logger().info("🚀 Motore Stockfish 16 connesso con successo!")
        except Exception as e:
            self.get_logger().error(f"Errore avvio motore Stockfish ({self.stockfish_path}): {e}")

    def fen_callback(self, msg: String):
        fen_candidate = msg.data.strip()
        if not fen_candidate:
            return
        # Aggiunge componenti standard FEN se manca il turno
        parts = fen_candidate.split()
        if len(parts) == 1:
            fen_candidate = f"{parts[0]} b KQkq - 0 1"
        
        self.latest_fen = fen_candidate
        self.fen_received = True
        self.get_logger().info(f"📥 Ricevuto FEN aggiornato da ChessQueries: {self.latest_fen}")

    def handle_trigger_topic(self, _msg: Empty):
        self.calculate_and_publish_best_move()

    def handle_compute_best_move(self, request, response):
        success, best_move, info = self.calculate_and_publish_best_move()
        response.success = success
        response.message = best_move if success else info
        return response

    def calculate_and_publish_best_move(self):
        """Calcola con Stockfish la mossa migliore per il Robot (Nero)."""
        if not self.engine:
            self.init_engine()
            if not self.engine:
                return False, "", "Motore Stockfish non disponibile!"

        # Isola la disposizione dei pezzi (il primo campo del FEN, es. 'rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR')
        piece_placement = self.latest_fen.split()[0] if self.latest_fen else "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"

        # Il robot gioca con il Nero ('b'): forziamo esplicitamente il turno del Nero
        is_black = not self.robot_color.lower().startswith('w')
        turn_char = 'b' if is_black else 'w'

        board = None
        for castling in ["KQkq", "kq", "-"]:
            try:
                candidate_fen = f"{piece_placement} {turn_char} {castling} - 0 1"
                b = chess.Board(candidate_fen)
                if b.is_valid():
                    board = b
                    break
            except Exception:
                continue

        if board is None:
            try:
                board = chess.Board(f"{piece_placement} {turn_char} - - 0 1")
            except Exception as e:
                err = f"FEN non valido per Stockfish ({piece_placement}): {e}"
                self.get_logger().error(err)
                return False, "", err

        # Forza in modo garantito il turno del robot
        board.turn = chess.BLACK if is_black else chess.WHITE

        color_str = "NERO (Black)" if is_black else "BIANCO (White)"
        self.get_logger().info(f"🤔 Stockfish sta calcolando la mossa migliore per il {color_str}...")
        self.get_logger().info(f"   FEN attivo per Stockfish: {board.fen()}")

        # Verifica fine partita
        if board.is_checkmate():
            msg = "🏆 Scaccomatto! Partita conclusa."
            self.get_logger().info(msg)
            return False, "", msg
        if board.is_stalemate() or board.is_insufficient_material():
            msg = "🤝 Patta! Partita conclusa."
            self.get_logger().info(msg)
            return False, "", msg

        try:
            # Calcolo mossa con Stockfish
            result = self.engine.play(board, chess.engine.Limit(time=self.move_time))
            if not result or not result.move:
                return False, "", "Nessuna mossa generata da Stockfish!"

            best_move_uci = result.move.uci()
            self.get_logger().info(f"💡 Best Move Stockfish: {best_move_uci.upper()}")

            # Pubblica la mossa sul topic ROS 2
            out_msg = String()
            out_msg.data = best_move_uci
            self.best_move_pub.publish(out_msg)

            return True, best_move_uci, f"Best move calcolata: {best_move_uci}"
        except Exception as e:
            err = f"Errore durante l'elaborazione Stockfish: {e}"
            self.get_logger().error(err)
            return False, "", err

    def destroy_node(self):
        if self.engine:
            try:
                self.engine.quit()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StockfishNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
