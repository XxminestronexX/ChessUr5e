# ChessUr5e

Progetto **UR5-Chess-Bot**. Questo sistema integra robotica avanzata, computer vision per permettere a un braccio robotico industriale Universal Robots (UR5) di giocare a scacchi contro un avversario umano in tempo reale.

## 📌 Panoramica del Progetto

L'obiettivo è creare un framework end-to-end basato su **ROS 2** capace di:
1.  **Percepire:** Riconoscere la configurazione della scacchiera e i pezzi tramite computer vision.
2.  **Ragionare:** Calcolare la mossa ottimale utilizzando il motore Stockfish.
3.  **Agire:** Pianificare ed eseguire traiettorie di precisione con il robot UR5 per muovere fisicamente i pezzi.

---

## 🛠️ Architettura del Sistema

Il progetto è suddiviso in tre moduli principali interconnessi tramite il middleware ROS 2:

### 1. Vision & Perception (Intel RealSense D435i)
Il "cuore visivo" del robot utilizza una telecamera **Intel RealSense D435i**.
* **Object Detection:** Implementazione di **YOLO (You Only Look Once)** per l'identificazione in tempo reale dei pezzi degli scacchi e delle caselle della scacchiera.
* **State Extraction:** Converte le coordinate dell'immagine in notazione FEN (Forsyth-Edwards Notation) per interpretare lo stato della partita.
* **Feedback:** Monitora costantemente la scacchiera per rilevare le mosse dell'avversario umano.

### 2. Strategic Brain (Stockfish 18 Engine)
Il modulo decisionale è un nodo ROS 2 che funge da bridge verso **Stockfish 18**.
* Analizza il file FEN ricevuto dal modulo di visione.
* Calcola la "Best Move" basandosi su livelli di difficoltà configurabili.
* Genera il comando di movimento (es. `e2e4`) da inviare al controllore cinematico.

### 3. Kinematics & Motion Control (UR5 + MoveIt 2)
La fase esecutiva trasforma la mossa scacchistica in traiettorie spaziali.
* **MoveIt 2:** Utilizzato per la pianificazione del movimento, l'evitamento degli ostacoli (collision detection tra i pezzi) e la cinematica inversa.
* **Traiettorie:** Gestione di pick-and-place precisi, inclusi i casi speciali come le mangiate, l'arrocco e la promozione dei pedoni.
* **Hardware Interface:** Comunicazione diretta con il controller UR5 per un'esecuzione fluida e sicura.

---

## 🚀 Tecnologie Utilizzate

* **Robotica:** [ROS 2 (Humble/Foxy)](https://docs.ros.org/), [MoveIt 2](https://moveit.ros.org/)
* **Hardware:** Universal Robots UR5, Intel RealSense D435i
* **AI/Vision:** YOLO (PyTorch/ONNX), OpenCV
* **Chess Engine:** Stockfish 18
* **Linguaggi:** Python, C++

## Comando importante Gazebo

colcon build

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/{PATH_CARTELLA_WS}/src

source install/setup.bash

ros2 launch robot_arms ur5e_gazebo.launch.py

## Specifiche schacchiera
schacchiera : 51x51 cm

caselle: 57mm di lato

il re alto: 95mm e base di 37mm di diametro
