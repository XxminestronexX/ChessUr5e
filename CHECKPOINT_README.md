# Riepilogo Checkpoint: Integrazione MoveIt 2 Completa + Controller UCI Interattivo

**Data Checkpoint:** 21 Ottobre 2026  
**Archivio di Backup:** `backup_checkpoint_moveit2_uci.tar.gz`

---

## 1. Resoconto Dettagliato delle Ultime Modifiche (Integrazione MoveIt 2 + Notazione UCI)

### A. Aggiunta del Frame TCP Fisico nell'URDF (`src/robot_arms/urdf/robotiq_hande.urdf.xacro`)
- Aggiunto il link `hande_tcp_link` e il giunto fisso `hande_tool0_to_tcp` a `+0.225 m` lungo l'asse $Z$ di `tool0` (esattamente al centro di presa tra i polpastrelli della pinza Robotiq Hand-E).
- In questo modo **MoveIt 2** pianifica direttamente la posa del punto di presa dei pezzi senza dover sommare offset manuali.

### B. Configurazione Completa di MoveIt 2 (`src/robot_arms/config/moveit/`)
1. **`ur5e.srdf`**:
   - Definito il planning group **`ur_manipulator`** (catena da `base_link` a `hande_tcp_link`).
   - Definito il planning group **`gripper`** (`hande_joint` e `hande_base_to_hande_jaw2_link`).
   - Definite le pose nominali `home_vertical`, `open` e `closed`.
   - Configurata la matrice di auto-collisione (**ACM - Allowed Collision Matrix**) completa tra tavolo, parete, braccio UR5e, supporto camera RealSense D435i e pinza Hand-E.
2. **`kinematics.yaml`**:
   - Configurato il solver `kdl_kinematics_plugin/KDLKinematicsPlugin` per il gruppo `ur_manipulator` (`search_resolution: 0.005`, `timeout: 0.05 s`).
3. **`joint_limits.yaml`**:
   - Configurati limiti di velocità e accelerazione per la parametrizzazione temporale delle traiettorie in MoveIt 2.
4. **`moveit_controllers.yaml`**:
   - Configurato `MoveItSimpleControllerManager` collegato agli action server `arm_controller/follow_joint_trajectory` e `gripper_controller/follow_joint_trajectory`.
5. **`ompl_planning.yaml`**:
   - Configurata la pipeline OMPL (`RRTConnectkConfigDefault`, `RRTstarkConfigDefault`) con gli adapter standard di MoveIt 2 Jazzy (`ResolveConstraintFrames`, `ValidateWorkspaceBounds`, `CheckStartStateBounds`, `CheckStartStateCollision`, `AddTimeOptimalParameterization`).

### C. Nuovi Launch File MoveIt 2 (`src/robot_arms/launch/`)
1. **`src/robot_arms/launch/ur5e_moveit_gazebo.launch.py`**:
   - Avvia in un colpo solo: **Gazebo Sim**, il robot UR5e, la scacchiera nella posizione ottimizzata `(-0.58, -0.22, 1.01)`, il bridge ROS-Gazebo (`/clock`, `/world/empty/set_pose`, `/world/empty/remove`), i controller `ros2_control` e il nodo **`move_group` di MoveIt 2**.
2. **`src/robot_arms/launch/move_group.launch.py`**:
   - Avvia il nodo **`move_group` di MoveIt 2** standalone (utile quando Gazebo è già aperto).

### D. Nodo Esecutore Mosse in Notazione UCI con MoveIt 2 (`moveit_uci_mover.py`)
- Creato lo script `moveit_uci_mover.py` (installato anche in `lib/robot_arms/moveit_uci_mover.py`) che:
  - Accetta qualsiasi mossa in **notazione algebrica UCI** (es. `e2e4`, `g1f3`, `b8c6`, `d1h5`) sia da riga di comando (`python3 moveit_uci_mover.py e2e4`) sia da **console interattiva** (`python3 moveit_uci_mover.py`).
  - Mantiene la mappa completa di tutti i 32 pezzi sulla scacchiera (pedoni, torri, cavalli, alfieri, regina, re), adattando automaticamente l'altezza di presa in base al pezzo (`pawn`: 24mm, `rook`: 30mm, `knight`: 32mm, `bishop`: 38mm, `queen`: 45mm, `king`: 50mm).
  - Gestisce automaticamente anche le **catture**: se la casella di arrivo è occupata da un pezzo avversario, prima lo preleva verticalmente e lo deposita fuori dalla scacchiera, poi esegue la mossa.
  - Usa **MoveIt 2 (`/compute_cartesian_path` + `/execute_trajectory`)** per tutte le fasi di approccio, discesa verticale, sollevamento verticale, trasporto aereo e rilascio.

---

## 2. Storico Precedente
- Visualizzazione Workspace 3D su RViz (`ur5e_workspace.launch.py`).
- Ottimizzazione coordinate scacchiera `(-0.58, -0.22)` per presa 100% verticale su 64/64 caselle.
- Sincronizzazione attendendo il completamento reale dei goal in Gazebo.
