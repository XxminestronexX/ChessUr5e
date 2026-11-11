#!/usr/bin/env python3
import os
import json
import chess
import subprocess

scripts = ['spawn_pawns.py', 'spawn_rooks.py', 'spawn_knights.py', 'spawn_bishops.py', 'spawn_royalty.py']

print("♟️ Schieramento completo in corso...")
for script in scripts:
    subprocess.run(['python3', script])
    print(f"✅ {script} eseguito.")

# Reimposta il FEN iniziale nel file di stato
pose_file = "detected_chessboard_pose.json"
if os.path.exists(pose_file):
    try:
        with open(pose_file, "r") as f:
            data = json.load(f)
        data["latest_fen"] = chess.STARTING_FEN
        with open(pose_file, "w") as f:
            json.dump(data, f, indent=2)
        print("✅ FEN iniziale reimpostato su detected_chessboard_pose.json")
    except Exception:
        pass