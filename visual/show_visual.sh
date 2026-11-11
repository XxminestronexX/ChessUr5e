#!/usr/bin/env bash
# ==============================================================================
# visual/show_visual.sh
# ------------------------------------------------------------------------------
# Lancia il visualizzatore grafico live di verifica FEN vs Foto reale D435.
# Può essere eseguito in qualsiasi momento (anche mentre la partita è in corso).
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

xhost +local:root > /dev/null 2>&1 || true

DOCKER_FLAGS="-i"
if [ -t 0 ]; then
    DOCKER_FLAGS="-it"
fi

if [ $# -eq 0 ]; then
    # Modalità live monitor
    docker exec $DOCKER_FLAGS -e DISPLAY="${DISPLAY:-:0}" chess_ur5e_sim bash -c \
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 /workspace/ChessUr5e/visual/fen_visualizer.py"
else
    # Verifica di un FEN specifico passato come argomento
    docker exec $DOCKER_FLAGS -e DISPLAY="${DISPLAY:-:0}" chess_ur5e_sim bash -c \
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 /workspace/ChessUr5e/visual/fen_visualizer.py \"$*\""
fi
