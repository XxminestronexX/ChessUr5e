#!/usr/bin/env bash

# ==============================================================================
# Script per inviare mosse ed interagire con Stockfish NATIVAMENTE (SENZA DOCKER)
# ==============================================================================

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_VERSION="${ROS_DISTRO:-jazzy}"

if [ ! -d "/opt/ros/$ROS_VERSION" ]; then
    if [ -d "/opt/ros/humble" ]; then
        ROS_VERSION="humble"
    fi
fi

source "/opt/ros/${ROS_VERSION}/setup.bash"
source "$PROJECT_DIR/install/setup.bash"
export PYTHONPATH="$PROJECT_DIR:$PYTHONPATH"

if [ $# -eq 0 ]; then
    # Partita interattiva Umano vs Stockfish con pulsante
    python3 "$PROJECT_DIR/moveit_uci_mover.py"
else
    # Esecuzione mosse passate da riga di comando (es: ./send_move_native.sh "e2e4 e7e5")
    python3 "$PROJECT_DIR/moveit_uci_mover.py" "$*"
fi
