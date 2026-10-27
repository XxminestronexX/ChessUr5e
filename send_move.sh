#!/usr/bin/env bash

# Script per inviare mosse UCI al braccio UR5e
# Supporta:
#  1) Modalità interattiva (nessun argomento):
#       ./send_move.sh
#  2) Mossa singola:
#       ./send_move.sh e2e4
#  3) Stringa di mosse (tra virgolette o separate da spazi o numeri di mossa):
#       ./send_move.sh "e2e4 e7e5 g1f3"
#       ./send_move.sh e2e4 e7e5 g1f3
#       ./send_move.sh "1. e2e4 e7e5 2. g1f3"

DOCKER_FLAGS="-i"
if [ -t 0 ]; then
    DOCKER_FLAGS="-it"
fi

if [ $# -eq 0 ]; then
    # Nessun parametro -> Console interattiva
    docker exec $DOCKER_FLAGS chess_ur5e_sim bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 moveit_uci_mover.py"
else
    # Esegue la sequenza di mosse in ordine
    docker exec $DOCKER_FLAGS chess_ur5e_sim bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 moveit_uci_mover.py \"$*\""
fi
