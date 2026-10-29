#!/usr/bin/env bash

# ==============================================================================
# Script per testare la rotazione della scacchiera in Gazebo durante il gioco.
#
# Uso:
#   ./rotate_chessboard.sh 25     # Ruota la scacchiera di +25 gradi
#   ./rotate_chessboard.sh -30    # Ruota la scacchiera di -30 gradi
#   ./rotate_chessboard.sh 90     # Ruota la scacchiera di 90 gradi
#   ./rotate_chessboard.sh        # Prompt interattivo
# ==============================================================================

DOCKER_FLAGS="-i"
if [ -t 0 ]; then
    DOCKER_FLAGS="-it"
fi

if docker ps --format '{{.Names}}' | grep -q "^chess_ur5e_sim$"; then
    docker exec $DOCKER_FLAGS chess_ur5e_sim bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 rotate_chessboard.py $1"
else
    # Ambiente nativo senza docker
    PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    ROS_VERSION="${ROS_DISTRO:-jazzy}"
    source "/opt/ros/${ROS_VERSION}/setup.bash"
    source "$PROJECT_DIR/install/setup.bash"
    python3 "$PROJECT_DIR/rotate_chessboard.py" "$1"
fi
