#!/usr/bin/env bash
set -e

# ==============================================================================
# Script di Installazione e Download Dipendenze per UR5e Chess Bot
# Supporta 2 modalità:
#   [1] Con Docker (Ambiente containerizzato autonomo)
#   [2] Senza Docker (Installazione Nativa su Ubuntu con ROS 2)
# ==============================================================================

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CKPT_DIR="$PROJECT_DIR/checkpoints/release"
CKPT_FILE="$CKPT_DIR/chessqueries-vitl-644-safetensors-fp16-r1.safetensors"
CKPT_URL="https://huggingface.co/joelseytre/chessqueries/resolve/529059bd7f00c3b7d39ecdf6cc2fef5d039fc3cf/chessqueries-vitl-644-safetensors-fp16-r1.safetensors"

download_model_weights() {
    echo ""
    echo "⬇️  Download dei pesi ufficiali del modello ChessQueries (ViT-L/14, ~709 MB)..."
    mkdir -p "$CKPT_DIR"
    if [ -f "$CKPT_FILE" ] && [ $(stat -c%s "$CKPT_FILE") -gt 500000000 ]; then
        echo "✅ Checkpoint già presente e verificato in: $CKPT_FILE"
    else
        curl -L --progress-bar -o "$CKPT_FILE" "$CKPT_URL"
        echo "✅ Download pesi completato con successo!"
    fi
}

install_docker_mode() {
    echo ""
    echo "================================================================="
    echo "🐳 OPZIONE 1: INSTALLAZIONE E SETUP CON DOCKER"
    echo "================================================================="

    if ! command -v docker &> /dev/null; then
        echo "❌ Docker non trovato! Installa Docker o usa l'opzione senza Docker."
        exit 1
    fi

    echo "1. Verifica / Avvio container Docker 'chess_ur5e_sim'..."
    if ! docker ps -a --format '{{.Names}}' | grep -q "^chess_ur5e_sim$"; then
        echo "📦 Creazione del container Docker 'chess_ur5e_sim'..."
        docker run -d \
            --name chess_ur5e_sim \
            --net=host \
            --ipc=host \
            --privileged \
            -e DISPLAY="${DISPLAY:-:0}" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "$PROJECT_DIR":/workspace/ChessUr5e \
            -w /workspace/ChessUr5e \
            ros2-jazzy-gazebo sleep infinity
    else
        docker start chess_ur5e_sim > /dev/null 2>&1 || true
    fi

    echo "2. Installazione pacchetti di sistema all'interno del container..."
    docker exec chess_ur5e_sim bash -c "
        apt-get update && \
        apt-get install -y stockfish python3-pip python3-venv && \
        ln -sf /usr/games/stockfish /usr/local/bin/stockfish
    "

    echo "3. Installazione librerie Python / AI nel container..."
    docker exec chess_ur5e_sim bash -c "
        pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu --break-system-packages && \
        pip3 install timm safetensors python-chess huggingface-hub pydantic pydantic-settings requests --break-system-packages
    "

    download_model_weights

    echo "4. Compilazione del workspace ROS 2 nel container..."
    docker exec chess_ur5e_sim bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

    echo ""
    echo "================================================================="
    echo "🎉 SETUP DOCKER COMPLETATO CON SUCCESSO!"
    echo "================================================================="
    echo "Per avviare la simulazione:"
    echo "   ./start_sim.sh"
    echo "Per giocare la partita con Stockfish:"
    echo "   ./send_move.sh"
    echo "================================================================="
}

install_native_mode() {
    echo ""
    echo "================================================================="
    echo "🖥️  OPZIONE 2: INSTALLAZIONE NATIVA SENZA DOCKER (Ubuntu + ROS 2)"
    echo "================================================================="

    # Rilevamento distribuzione ROS 2
    ROS_VERSION="${ROS_DISTRO:-jazzy}"
    if [ ! -d "/opt/ros/$ROS_VERSION" ]; then
        if [ -d "/opt/ros/humble" ]; then
            ROS_VERSION="humble"
        elif [ -d "/opt/ros/iron" ]; then
            ROS_VERSION="iron"
        else
            echo "⚠️  Nessuna installazione ROS 2 trovata in /opt/ros!"
            echo "   Assicurati di aver installato ROS 2 (Jazzy o Humble) su Ubuntu."
            read -p "Inserisci manualmente il nome della tua distribuzione ROS 2 (es. jazzy, humble): " USER_DISTRO
            ROS_VERSION="$USER_DISTRO"
        fi
    fi

    echo "Distro ROS 2 rilevata: $ROS_VERSION"
    echo "1. Installazione dipendenze di sistema e ROS 2 con sudo apt..."

    sudo apt-get update
    sudo apt-get install -y \
        stockfish \
        python3-pip \
        python3-venv \
        python3-pykdl \
        ros-${ROS_VERSION}-ros-gz \
        ros-${ROS_VERSION}-ros-gz-sim \
        ros-${ROS_VERSION}-ros-gz-bridge \
        ros-${ROS_VERSION}-ros-gz-image \
        ros-${ROS_VERSION}-xacro \
        ros-${ROS_VERSION}-robot-state-publisher \
        ros-${ROS_VERSION}-controller-manager \
        ros-${ROS_VERSION}-joint-state-broadcaster \
        ros-${ROS_VERSION}-joint-trajectory-controller \
        ros-${ROS_VERSION}-moveit-ros-move-group \
        ros-${ROS_VERSION}-ur-description || true

    # Link per stockfish se installato in /usr/games
    if [ -f "/usr/games/stockfish" ] && [ ! -f "/usr/local/bin/stockfish" ]; then
        sudo ln -s /usr/games/stockfish /usr/local/bin/stockfish || true
    fi

    echo "2. Installazione librerie Python e PyTorch (CPU) per ChessQueries..."
    pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu --break-system-packages 2>/dev/null || \
    pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

    pip3 install timm safetensors python-chess huggingface-hub pydantic pydantic-settings requests opencv-python --break-system-packages 2>/dev/null || \
    pip3 install timm safetensors python-chess huggingface-hub pydantic pydantic-settings requests opencv-python

    download_model_weights

    echo "3. Compilazione pacchetto robot_arms con colcon build..."
    source "/opt/ros/${ROS_VERSION}/setup.bash"
    colcon build

    echo ""
    echo "================================================================="
    echo "🎉 SETUP NATIVO (SENZA DOCKER) COMPLETATO CON SUCCESSO!"
    echo "================================================================="
    echo "Per avviare la simulazione nativamente:"
    echo "   ./start_sim_native.sh"
    echo "Per giocare la partita con Stockfish nativamente:"
    echo "   ./send_move_native.sh"
    echo "================================================================="
}

# ------------------------------------------------------------------------------
# Menu di Selezione
# ------------------------------------------------------------------------------
if [ "$1" == "--docker" ] || [ "$1" == "-d" ]; then
    install_docker_mode
elif [ "$1" == "--native" ] || [ "$1" == "-n" ] || [ "$1" == "--no-docker" ]; then
    install_native_mode
else
    echo "================================================================="
    echo "♟️   UR5e CHESS BOT: INSTALLAZIONE E DOWNLOAD DIPENDENZE   ♟️"
    echo "================================================================="
    echo "Scegli la modalità di installazione desiderata:"
    echo ""
    echo "  [1] Con Docker     (Ambiente containerizzato completo e pronto)"
    echo "  [2] SENZA Docker   (Nativo su Ubuntu con ROS 2 Jazzy/Humble)"
    echo "  [q] Esci"
    echo ""
    read -p "Inserisci la tua scelta (1 o 2): " CHOICE

    case "$CHOICE" in
        1)
            install_docker_mode
            ;;
        2)
            install_native_mode
            ;;
        *)
            echo "Uscita..."
            exit 0
            ;;
    esac
fi
