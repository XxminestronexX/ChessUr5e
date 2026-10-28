#!/usr/bin/env bash

# ==============================================================================
# Script per visualizzare in tempo reale il FEN e la scacchiera da ChessQueries
# ==============================================================================

echo "================================================================="
echo "♟️  MONITOR FEN CHESSQUERIES (Intel RealSense D435)  ♟️"
echo "================================================================="
echo "In ascolto del topic ROS 2: /chessboard/detected_fen..."
echo "A fine di ogni mossa, il nuovo FEN riconosciuto verrà mostrato qui."
echo "(Premi Ctrl+C per uscire)"
echo "================================================================="

docker exec -it chess_ur5e_sim bash -c '
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -c "
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

def print_board(fen):
    board_part = fen.split()[0]
    rows = board_part.split(\"/\")
    print(\"\033[2J\033[H\") # clear terminal
    print(\"=================================================================\")
    print(\"♟️   CHESSQUERIES REAL-TIME RECOGNITION (D435 CAMERA VIEW)   ♟️\")
    print(\"=================================================================\")
    print(f\"🕒 Timestamp: {time.strftime(\x27%H:%M:%S\x27)}\")
    print(f\"📋 FEN Rilevato:\n   \033[1;32m{fen}\033[0m\n\")
    print(\"       a   b   c   d   e   f   g   h\")
    print(\"     +---+---+---+---+---+---+---+---+\")
    for idx, row in enumerate(rows):
        rank = 8 - idx
        line = f\"   {rank} |\"
        for char in row:
            if char.isdigit():
                line += \" . |\" * int(char)
            else:
                line += f\" {char} |\"
        print(line + f\" {rank}\")
        print(\"     +---+---+---+---+---+---+---+---+\")
    print(\"       a   b   c   d   e   f   g   h\")
    print(\"=================================================================\")
    print(\"In attesa del prossimo scatto della telecamera in posa Home...\")

rclpy.init()
node = Node(\"fen_watcher\")

# Se esiste già uno snapshot precedente, leggilo come valore iniziale
try:
    with open(\"latest_chessboard_snapshot.jpg\", \"rb\") as f: pass
    # Controlla se la telecamera ha già pubblicato
except: pass

def cb(msg):
    print_board(msg.data)

sub = node.create_subscription(String, \"/chessboard/detected_fen\", cb, 10)
print(\"⏳ In attesa del primo riconoscimento...\")
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
"
'
