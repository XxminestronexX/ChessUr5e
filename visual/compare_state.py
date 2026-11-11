#!/usr/bin/env python3
"""
visual/compare_state.py
--------------------------------------------------------------------------------
Tool di comparazione esatta casella per casella tra:
  - FEN riconosciuto dalla telecamera (ChessQueries)
  - Stato reale dei pezzi posizionati in Gazebo

Stampa a terminale una tabella di mismatch con l'indicazione precisa
delle caselle che non coincidono (es: "e4: previsto Bianco Pawn, rilevato Vuoto").
"""

import os
import sys
import os
import sys
import json
import math
import chess
import subprocess
import re

def get_real_gazebo_pieces():
    """Interroga Gazebo per individuare la posizione reale di ogni pezzo sulla scacchiera."""
    piece_map = {}
    SQUARE_SIZE = 0.05875
    A1_LOCAL_X = 0.21290
    A1_LOCAL_Y = 0.21745

    # Recupera posa reale della scacchiera in Gazebo se disponibile
    cx, cy, cz, cb_yaw = -0.58, -0.22, 1.01, math.pi
    try:
        from spawn_utils import get_chessboard_gazebo_pose
        cx, cy, cz, cb_yaw = get_chessboard_gazebo_pose()
    except Exception:
        pass

    models = (
        [(f'white_pawn_{i}', chess.PAWN, chess.WHITE) for i in range(8)] +
        [(f'black_pawn_{i}', chess.PAWN, chess.BLACK) for i in range(8)] +
        [('white_rook_1', chess.ROOK, chess.WHITE), ('white_rook_2', chess.ROOK, chess.WHITE),
         ('black_rook_1', chess.ROOK, chess.BLACK), ('black_rook_2', chess.ROOK, chess.BLACK),
         ('white_knight_1', chess.KNIGHT, chess.WHITE), ('white_knight_2', chess.KNIGHT, chess.WHITE),
         ('black_knight_1', chess.KNIGHT, chess.BLACK), ('black_knight_2', chess.KNIGHT, chess.BLACK),
         ('white_bishop_1', chess.BISHOP, chess.WHITE), ('white_bishop_2', chess.BISHOP, chess.WHITE),
         ('black_bishop_1', chess.BISHOP, chess.BLACK), ('black_bishop_2', chess.BISHOP, chess.BLACK),
         ('white_queen', chess.QUEEN, chess.WHITE), ('white_king', chess.KING, chess.WHITE),
         ('black_queen', chess.QUEEN, chess.BLACK), ('black_king', chess.KING, chess.BLACK)]
    )

    for name, ptype, pcolor in models:
        try:
            out = subprocess.check_output(f"gz model -m {name} -p", shell=True, stderr=subprocess.DEVNULL).decode('utf-8')
            m = re.search(r'\[\s*([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s*\]', out)
            if m:
                gx = float(m.group(1))
                gy = float(m.group(2))
                # Se è fuori dal tavolo o nel cimitero fisso (Y <= -0.60 o X < -0.95), è catturato
                if gx < -0.95 or gy <= -0.60:
                    continue
                # Trova la casella più vicina
                best_sq = None
                min_d = 999.0
                for f_idx in range(8):
                    for r_idx in range(8):
                        # Calcolo coordinata reale della casella in Gazebo (supporta rotazioni arbitrarie)
                        lx = A1_LOCAL_X - (f_idx * SQUARE_SIZE)
                        ly = A1_LOCAL_Y - (r_idx * SQUARE_SIZE)
                        sq_x = cx + (lx * math.cos(cb_yaw)) - (ly * math.sin(cb_yaw))
                        sq_y = cy + (lx * math.sin(cb_yaw)) + (ly * math.cos(cb_yaw))
                        d = math.hypot(gx - sq_x, gy - sq_y)
                        if d < min_d:
                            min_d = d
                            best_sq = chess.square(f_idx, r_idx)
                if best_sq is not None and min_d < 0.040:
                    piece_map[best_sq] = chess.Piece(ptype, pcolor)
        except Exception:
            continue

    b = chess.Board(None)
    for sq, p in piece_map.items():
        b.set_piece_at(sq, p)
    return b


def main():
    print("=" * 68)
    print("🔍   COMPARATORE STATO REALE GAZEBO vs FEN RICONOSCIUTO CAMERA   🔍")
    print("=" * 68)

    project_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    pose_file = os.path.join(project_dir, "detected_chessboard_pose.json")

    # 1. Recupera il FEN riconosciuto dalla telecamera
    detected_fen = None
    if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
        detected_fen = sys.argv[1]
    elif os.path.exists(pose_file):
        try:
            with open(pose_file, 'r') as f:
                data = json.load(f)
                detected_fen = data.get("latest_fen")
        except Exception:
            pass

    if not detected_fen:
        detected_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

    print(f"📷 FEN Riconosciuto da Telecamera:\n   {detected_fen}\n")

    b_detected = chess.Board(detected_fen)
    b_real = get_real_gazebo_pieces()

    print("🤖 Stato Reale estratto dai modelli Gazebo:\n   " + b_real.fen() + "\n")

    # Confronto casella per casella
    matches = 0
    mismatches = []
    for sq in chess.SQUARES:
        sq_name = chess.square_name(sq).upper()
        p_real = b_real.piece_at(sq)
        p_det = b_detected.piece_at(sq)

        if p_real == p_det:
            matches += 1
        else:
            sym_real = p_real.symbol() if p_real else "VUOTA"
            sym_det = p_det.symbol() if p_det else "VUOTA"
            mismatches.append((sq_name, sym_real, sym_det))

    print("-" * 68)
    print(f"📊 ESITO CONFRONTO: {matches}/64 caselle coincidenti ({matches/64.0*100:.1f}%)")
    print("-" * 68)

    if not mismatches:
        print("🎉 PERFETTO! Nessun errore: il FEN corrisponde al 100% alla realtà in Gazebo.")
    else:
        print(f"⚠️  RILEVATI {len(mismatches)} DISCORDANZE:")
        print(f"    {'Casella':<10} | {'Reale su Gazebo':<20} | {'Rilevato da Camera':<20}")
        print("    " + "-" * 56)
        for sq_name, sym_real, sym_det in mismatches:
            print(f"    {sq_name:<10} | {sym_real:<20} | {sym_det:<20}")
    print("=" * 68 + "\n")


if __name__ == '__main__':
    main()
