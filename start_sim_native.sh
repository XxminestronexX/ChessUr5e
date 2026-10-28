#!/usr/bin/env bash
set -e

# ==============================================================================
# Avvio Simulazione Gazebo + MoveIt 2 NATIVA (SENZA DOCKER)
# ==============================================================================

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_VERSION="${ROS_DISTRO:-jazzy}"

if [ ! -d "/opt/ros/$ROS_VERSION" ]; then
    if [ -d "/opt/ros/humble" ]; then
        ROS_VERSION="humble"
    fi
fi

echo "================================================================="
echo "♟️   AVVIO SIMULAZIONE SCACCHI UR5e (NATIVO - SENZA DOCKER)   ♟️"
echo "================================================================="

source "/opt/ros/${ROS_VERSION}/setup.bash"
source "$PROJECT_DIR/install/setup.bash"

# Export percorso risorse Gazebo
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$PROJECT_DIR/src:$PROJECT_DIR/install/share"

echo "🚀 Avvio Gazebo Sim, robot UR5e, controller e MoveIt 2..."
ros2 launch robot_arms ur5e_moveit_gazebo.launch.py > /tmp/sim_launch_native.log 2>&1 &
LAUNCH_PID=$!

echo "⏳ Attesa attivazione controller ROS 2..."
TIMEOUT=35
ELAPSED=0
READY=false

while [ $ELAPSED -lt $TIMEOUT ]; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    echo -n "."
    CTRL_STATUS=$(ros2 control list_controllers 2>/dev/null || true)
    if echo "$CTRL_STATUS" | grep -q "arm_controller" && \
       echo "$CTRL_STATUS" | grep -q "gripper_controller" && \
       echo "$CTRL_STATUS" | grep -q "joint_state_broadcaster"; then
        READY=true
        break
    fi
done
echo ""

if [ "$READY" = false ]; then
    echo "⚠️ Timeout attesa controller. Controlla il log: /tmp/sim_launch_native.log"
    exit 1
fi

echo "✅ Gazebo e MoveIt 2 pronti!"
echo "♟️  Schieramento dei 32 pezzi sulle caselle..."
python3 "$PROJECT_DIR/spawn_all.py"

echo ""
echo "================================================================="
echo "🎉 SIMULAZIONE NATIVA AVVIATA!"
echo "================================================================="
echo "Per giocare la partita con Stockfish:"
echo "   ./send_move_native.sh"
echo "Per monitorare il FEN in un'altra shell:"
echo "   ros2 topic echo /chessboard/detected_fen"
echo "================================================================="
