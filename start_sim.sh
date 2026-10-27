#!/usr/bin/env bash
set -e

# ==============================================================================
# Script di avvio automatico della simulazione Gazebo Sim + MoveIt 2 (Chess UR5e)
# ==============================================================================

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="chess_ur5e_sim"
IMAGE_NAME="ros2-jazzy-gazebo"

echo "================================================================="
echo "♟️   AVVIO SIMULAZIONE SCACCHI UR5e (Gazebo Sim + MoveIt 2)   ♟️"
echo "================================================================="

# 1. Abilita accesso grafico al server X11
echo "🖥️  Configurazione server grafico X11..."
xhost +local:root > /dev/null 2>&1 || true

# 2. Controllo e avvio container Docker
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "📦 Creazione ed avvio nuovo container Docker '${CONTAINER_NAME}'..."
    docker run -d \
        --name "$CONTAINER_NAME" \
        --net=host \
        --ipc=host \
        --privileged \
        -e DISPLAY="${DISPLAY:-:0}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$PROJECT_DIR":/workspace/ChessUr5e \
        -w /workspace/ChessUr5e \
        "$IMAGE_NAME" sleep infinity
else
    echo "🔄 Pulizia e riavvio container '${CONTAINER_NAME}'..."
    docker restart "$CONTAINER_NAME" > /dev/null
fi

# 3. Lancio di Gazebo + MoveIt 2 in background all'interno del container
echo "🚀 Avvio Gazebo Sim, robot UR5e, controller e MoveIt 2..."
docker exec -d -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER_NAME" bash -c \
    "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch robot_arms ur5e_moveit_gazebo.launch.py > /tmp/sim_launch.log 2>&1"

# 4. Attesa inizializzazione controller
echo -n "⏳ Attesa attivazione controller ROS 2 (arm_controller, gripper_controller)..."
TIMEOUT=40
ELAPSED=0
READY=false

while [ $ELAPSED -lt $TIMEOUT ]; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    echo -n "."

    # Verifica se i controller sono nello stato 'active'
    CTRL_STATUS=$(docker exec "$CONTAINER_NAME" bash -c \
        "source /opt/ros/jazzy/setup.bash && ros2 control list_controllers 2>/dev/null" || true)

    if echo "$CTRL_STATUS" | grep -q "arm_controller" && \
       echo "$CTRL_STATUS" | grep -q "gripper_controller" && \
       echo "$CTRL_STATUS" | grep -q "joint_state_broadcaster"; then
        READY=true
        break
    fi
done
echo ""

if [ "$READY" = false ]; then
    echo "⚠️ Timeout durante l'attesa dei controller. Verifica i log con:"
    echo "   docker exec -it $CONTAINER_NAME cat /tmp/sim_launch.log"
    exit 1
fi

echo "✅ Gazebo e MoveIt 2 pronti!"

# 5. Schieramento di tutti i 32 pezzi sulla scacchiera
echo "♟️  Schieramento dei 32 pezzi sulle caselle (spawn_all.py)..."
docker exec "$CONTAINER_NAME" bash -c \
    "source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 spawn_all.py"

echo ""
echo "================================================================="
echo "🎉 SIMULAZIONE PRONTA E OPERATIVA A SCHERMO!"
echo "================================================================="
echo "Puoi inviare le mosse usando:"
echo "  • Mossa singola:     ./send_move.sh e2e4"
echo "  • Sequenza:          ./send_move.sh \"e2e4 e7e5 g1f3\""
echo "  • Console comandi:   ./send_move.sh"
echo ""
echo "Per arrestare la simulazione: ./stop_sim.sh"
echo "================================================================="
