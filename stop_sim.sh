#!/usr/bin/env bash

# ==============================================================================
# Script per arrestare la simulazione Gazebo e il container
# ==============================================================================

CONTAINER_NAME="chess_ur5e_sim"

echo "🛑 Arresto della simulazione..."

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    docker stop "$CONTAINER_NAME" > /dev/null
    echo "✅ Simulazione arrestata con successo."
else
    echo "ℹ️  La simulazione non è attualmente in esecuzione."
fi
