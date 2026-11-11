#!/usr/bin/env bash
# ==============================================================================
# visual/check_state.sh
# ------------------------------------------------------------------------------
# Esegue il comparatore terminale casella per casella tra Gazebo reale e Camera.
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker exec -it chess_ur5e_sim python3 /workspace/ChessUr5e/visual/compare_state.py "$@"
