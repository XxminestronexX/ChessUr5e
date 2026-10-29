#!/usr/bin/env python3
"""
Nodo ROS 2 per la telecamera Intel RealSense D435 in Gazebo Sim
e riconoscimento automatico della scacchiera tramite ChessQueries (ViT-L/14 + DETR Decoder).

Funzionalità:
1. Riceve lo stream video della telecamera D435 simulata in Gazebo (/camera/image_raw).
2. Fornisce il servizio `/chessboard_camera/snapshot_and_recognize` (std_srvs/srv/Trigger).
3. All'attivazione (es. a fine di ogni mossa quando il braccio è in Home):
   - Cattura un fotogramma nitido dalla telecamera;
   - Mostra a schermo una finestra grafica OpenCV con la vista live della telecamera e la griglia FEN riconosciuta;
   - Salva lo snapshot su disco ('latest_chessboard_snapshot.jpg');
   - Invia l'immagine al modello ChessQueries (https://github.com/JSeytre/chessqueries);
   - Calcola e pubblica il FEN rilevato su `/chessboard/detected_fen` (std_msgs/msg/String);
   - Stampa il diagramma della scacchiera a terminale.
"""

import os
import sys
import time
import json
import math
from pathlib import Path

# Assicura che la cartella del progetto (/workspace/ChessUr5e) sia in sys.path
WORKSPACE_DIR = Path("/workspace/ChessUr5e") if Path("/workspace/ChessUr5e").exists() else Path.cwd()
if str(WORKSPACE_DIR) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_DIR))

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger

# Importazione modulo ChessQueries
try:
    from chessqueries.models.predictor import Predictor
    CHESSQUERIES_AVAILABLE = True
except ImportError as e:
    CHESSQUERIES_AVAILABLE = False
    CHESSQUERIES_IMPORT_ERROR = str(e)


def ros_image_to_cv2(msg: Image) -> np.ndarray:
    """Converte un messaggio sensor_msgs/msg/Image in una matrice BGR OpenCV."""
    n_channels = 3
    dtype = np.uint8
    if 'rgb' in msg.encoding.lower():
        img = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width, n_channels))
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif 'bgr' in msg.encoding.lower():
        return np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width, n_channels))
    elif 'mono8' in msg.encoding.lower() or '8uc1' in msg.encoding.lower():
        img = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width))
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        # Fallback raw buffer
        img = np.frombuffer(msg.data, dtype=dtype)
        if len(img) == msg.height * msg.width * 3:
            return cv2.cvtColor(img.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR)
        return img.reshape((msg.height, msg.width, -1))


class CameraBoardRecognitionNode(Node):
    def __init__(self):
        super().__init__(
            'camera_board_recognition_node',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )
        self.get_logger().info("📷 Inizializzazione nodo Camera D435 + ChessQueries Recognition...")

        # Percorsi checkpoint e salvataggio
        self.workspace_dir = Path("/workspace/ChessUr5e") if Path("/workspace/ChessUr5e").exists() else Path.cwd()
        self.ckpt_path = self.workspace_dir / "checkpoints" / "release" / "chessqueries-vitl-644-safetensors-fp16-r1.safetensors"
        self.snapshot_path = self.workspace_dir / "latest_chessboard_snapshot.jpg"
        self.annotated_path = self.workspace_dir / "latest_recognition_viz.jpg"
        self.detected_pose_file = self.workspace_dir / "detected_chessboard_pose.json"

        # Configurazione Visione Computerizzata ArUco (DICT_4X4_50, Marker ID 0)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.05  # Dimensione fisica marker ArUco (5 cm x 5 cm)
        self.camera_hfov = 1.45  # Campo visivo orizzontale RealSense D435
        self.detected_aruco_data = None

        # Publisher per le pose calcolate via Visione
        self.aruco_pose_pub = self.create_publisher(PoseStamped, '/chessboard/aruco_pose', 10)
        self.board_pose_pub = self.create_publisher(PoseStamped, '/chessboard/detected_pose', 10)

        # Caricamento predittore ChessQueries
        self.predictor = None
        if CHESSQUERIES_AVAILABLE and self.ckpt_path.exists():
            self.get_logger().info(f"🧠 Caricamento pesi modello ChessQueries da: {self.ckpt_path}...")
            try:
                self.predictor = Predictor.from_checkpoint(str(self.ckpt_path), resolution=644, device="cpu")
                self.get_logger().info("✅ Modello ChessQueries (ViT-L/14) caricato con successo!")
            except Exception as e:
                self.get_logger().error(f"Errore nel caricamento dei pesi ChessQueries: {e}")
        else:
            if not CHESSQUERIES_AVAILABLE:
                self.get_logger().warn(f"⚠️ Modulo chessqueries non disponibile: {CHESSQUERIES_IMPORT_ERROR}")
            if not self.ckpt_path.exists():
                self.get_logger().warn(f"⚠️ Checkpoint {self.ckpt_path} non trovato!")

        # Ultimo fotogramma ricevuto
        self.latest_cv_image = None
        self.latest_image_time = 0.0

        # Subscriber stream telecamera D435 da Gazebo
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher per il FEN rilevato
        self.fen_pub = self.create_publisher(String, '/chessboard/detected_fen', 10)

        # Servizio e Topic per trigger snapshot
        self.srv = self.create_service(Trigger, '/chessboard_camera/snapshot_and_recognize', self.handle_snapshot_service)
        self.trigger_sub = self.create_subscription(Empty, '/chessboard_camera/trigger', self.handle_snapshot_topic, 10)

        # Finestra di visualizzazione GUI OpenCV
        self.window_name = "Intel RealSense D435 - Chessboard Recognition (Gazebo Sim)"
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 800, 600)
        except Exception as e:
            self.get_logger().warn(f"Visualizzazione GUI non supportata o headless: {e}")

        # Timer periodico per aggiornare la visualizzazione OpenCV (se attiva)
        self.gui_timer = self.create_timer(0.1, self.update_gui_window)
        self.get_logger().info("🚀 Nodo Camera D435 operativo! In ascolto su /camera/image_raw")

    def image_callback(self, msg: Image):
        try:
            self.latest_cv_image = ros_image_to_cv2(msg)
            self.latest_image_time = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"Errore decodifica immagine telecamera: {e}")

    def update_gui_window(self):
        """Mantiene reattiva la finestra OpenCV di visualizzazione della telecamera."""
        if self.latest_cv_image is not None:
            try:
                # Mostra l'ultimo fotogramma o visualizzazione annotata
                cv2.imshow(self.window_name, self.latest_cv_image)
                cv2.waitKey(1)
            except Exception:
                pass

    def handle_snapshot_topic(self, _msg: Empty):
        self.run_recognition()

    def handle_snapshot_service(self, request, response):
        success, message, fen = self.run_recognition()
        response.success = success
        response.message = f"{message} | FEN: {fen}"
        return response

    def run_recognition(self):
        """
        Cattura l'immagine corrente della telecamera, la visualizza,
        la salva su disco e la passa a ChessQueries per stimare il FEN.
        """
        self.get_logger().info("📸 [D435 Camera] Acquisizione fotogramma dalla scacchiera...")

        # Attende un fotogramma fresco se non ancora arrivato
        t0 = time.monotonic()
        while self.latest_cv_image is None and (time.monotonic() - t0) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_cv_image is None:
            err_msg = "Nessun fotogramma ricevuto dalla telecamera /camera/image_raw!"
            self.get_logger().error(err_msg)
            return False, err_msg, ""

        img = self.latest_cv_image.copy()

        # 1. Salva lo snapshot grezzo per il recognizer
        cv2.imwrite(str(self.snapshot_path), img)
        self.get_logger().info(f"💾 Snapshot telecamera salvato in: {self.snapshot_path}")

        # 1b. Riconoscimento Visivo Posa ArUco (Computer Vision)
        annotated = img.copy()
        h, w = annotated.shape[:2]
        self.detect_and_process_aruco(annotated, w, h)

        # 2. Esecuzione riconoscimento con ChessQueries
        detected_fen = "8/8/8/8/8/8/8/8 w - - 0 1"
        board_str = ""
        if self.predictor is not None:
            try:
                self.get_logger().info("🔍 Esecuzione inferenza ChessQueries (ViT-L + DETR)...")
                preds = self.predictor.predict([str(self.snapshot_path)])
                if preds:
                    detected_fen = preds[0].fen
                    board_str = str(preds[0].board)
                    self.get_logger().info(f"♟️ FEN Riconosciuto: {detected_fen}")
            except Exception as e:
                self.get_logger().error(f"Errore durante l'inferenza di ChessQueries: {e}")
        else:
            self.get_logger().warn("⚠️ Modello ChessQueries non caricato. Generazione preview immagine.")

        # 3. Creazione immagine composita annotata (Vista Camera + Overlay Info FEN)
        # Barra superiore informativa semitrasparente
        overlay = annotated.copy()
        cv2.rectangle(overlay, (0, 0), (w, 65), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.7, annotated, 0.3, 0, annotated)

        cv2.putText(annotated, "Intel RealSense D435 - Chessboard View", (15, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(annotated, f"FEN: {detected_fen[:40]}...", (15, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Salva e mostra la vista annotata
        cv2.imwrite(str(self.annotated_path), annotated)
        try:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)
        except Exception:
            pass

        # 4. Pubblica sul topic ROS 2
        fen_msg = String()
        fen_msg.data = detected_fen
        self.fen_pub.publish(fen_msg)

        # 5. Stampa a terminale la vista ASCII della scacchiera
        self.print_ascii_board(detected_fen)

        return True, "Riconoscimento scacchiera completato", detected_fen

    def detect_and_process_aruco(self, annotated_img: np.ndarray, w: int, h: int):
        """
        Rileva visivamente il marker ArUco ID 0 nel fotogramma della telecamera,
        stima la posa 3D relativa e calcola la posa assoluta della scacchiera e del marker
        senza alcuna interrogazione al simulatore Gazebo.
        """
        corners, ids, rejected = cv2.aruco.detectMarkers(annotated_img, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and 0 in ids:
            idx = list(ids.flatten()).index(0)
            c = corners[idx][0]

            # Intrinseci telecamera D435
            fx = (w / 2.0) / math.tan(self.camera_hfov / 2.0)
            fy = fx
            cx_p, cy_p = w / 2.0, h / 2.0
            K = np.array([[fx, 0, cx_p], [0, fy, cy_p], [0, 0, 1]], dtype=np.float32)
            dist = np.zeros(5, dtype=np.float32)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corners[idx]], self.marker_size, K, dist)
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # Disegna marker ed assi 3D (RGB) sul frame
            cv2.aruco.drawDetectedMarkers(annotated_img, corners, ids)
            cv2.drawFrameAxes(annotated_img, K, dist, rvec, tvec, 0.05, 2)

            # Matrice di rotazione dell'ArUco rispetto alla telecamera
            R_cv, _ = cv2.Rodrigues(rvec)

            # Posa nota e stabile della telecamera in Home Panoramica rispetto al mondo
            cam_pos_world = np.array([-0.842, -0.220, 0.656])
            v_fwd = np.array([0.262, 0.0, -0.646])
            v_fwd = v_fwd / np.linalg.norm(v_fwd)
            v_y_axis = np.array([0.0, -1.0, 0.0])
            v_x_axis = np.cross(v_y_axis, v_fwd)
            v_x_axis = v_x_axis / np.linalg.norm(v_x_axis)
            v_y_axis = np.cross(v_fwd, v_x_axis)
            v_y_axis = v_y_axis / np.linalg.norm(v_y_axis)
            R_cam_to_world = np.column_stack([v_x_axis, v_y_axis, v_fwd])

            # Posa 3D ArUco in coordinate World
            delta_world = R_cam_to_world @ tvec
            aruco_pos_world = cam_pos_world + delta_world

            R_marker_world = R_cam_to_world @ R_cv
            marker_x_world = R_marker_world[:, 0]
            aruco_yaw = math.atan2(marker_x_world[1], marker_x_world[0])

            # Orientamento e centro della scacchiera derivati dalla terna dell'ArUco
            board_yaw = aruco_yaw + (math.pi / 2.0)
            ax, ay, az = float(aruco_pos_world[0]), float(aruco_pos_world[1]), float(aruco_pos_world[2])

            bx = ax - (0.28 * math.cos(board_yaw) - 0.23 * math.sin(board_yaw))
            by = ay - (0.28 * math.sin(board_yaw) + 0.23 * math.cos(board_yaw))
            bz = az - 0.0012

            self.detected_aruco_data = {
                'aruco': {'x': ax, 'y': ay, 'z': az, 'yaw': board_yaw},
                'chessboard': {'x': bx, 'y': by, 'z': bz, 'yaw': board_yaw}
            }

            # Salva la posa rilevata su file JSON per accesso immediato da parte di tutti i nodi
            try:
                with open(str(self.detected_pose_file), 'w') as f:
                    json.dump(self.detected_aruco_data, f, indent=2)
            except Exception as e:
                self.get_logger().error(f"Errore scrittura {self.detected_pose_file}: {e}")

            # Pubblica PoseStamped sui topic ROS 2
            stamp = self.get_clock().now().to_msg()

            ar_msg = PoseStamped()
            ar_msg.header.stamp = stamp
            ar_msg.header.frame_id = 'world'
            ar_msg.pose.position.x, ar_msg.pose.position.y, ar_msg.pose.position.z = ax, ay, az
            ar_msg.pose.orientation.z = math.sin(aruco_yaw / 2.0)
            ar_msg.pose.orientation.w = math.cos(aruco_yaw / 2.0)
            self.aruco_pose_pub.publish(ar_msg)

            cb_msg = PoseStamped()
            cb_msg.header.stamp = stamp
            cb_msg.header.frame_id = 'world'
            cb_msg.pose.position.x, cb_msg.pose.position.y, cb_msg.pose.position.z = bx, by, bz
            cb_msg.pose.orientation.z = math.sin(aruco_yaw / 2.0)
            cb_msg.pose.orientation.w = math.cos(aruco_yaw / 2.0)
            self.board_pose_pub.publish(cb_msg)

            self.get_logger().info(
                f"🎯 [Visione ArUco D435] Marker #0 RILEVATO: ({ax:.3f}, {ay:.3f}, {az:.3f}) | "
                f"Scacchiera stimata: ({bx:.3f}, {by:.3f}) Yaw={math.degrees(aruco_yaw):.1f}°"
            )

            # Overlay a schermo
            cv2.putText(annotated_img, f"ArUco #0 RILEVATO: ({ax:.2f}, {ay:.2f}) Yaw: {math.degrees(aruco_yaw):.1f}deg",
                        (15, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        else:
            self.get_logger().warn("⚠️ [Visione ArUco] Marker ID 0 non rilevato nel fotogramma.")

    def print_ascii_board(self, fen: str):
        """Stampa la griglia 8x8 a terminale a partire dal FEN."""
        board_part = fen.split()[0]
        rows = board_part.split('/')
        print("\n" + "=" * 55)
        print("📷  STATO SCACCHIERA ACQUISITO DALLA TELECAMERA D435  📷")
        print("=" * 55)
        print("    a   b   c   d   e   f   g   h")
        print("  +---+---+---+---+---+---+---+---+")
        for idx, row in enumerate(rows):
            rank = 8 - idx
            line = f"{rank} |"
            for char in row:
                if char.isdigit():
                    line += " . |" * int(char)
                else:
                    line += f" {char} |"
            print(line + f" {rank}")
            print("  +---+---+---+---+---+---+---+---+")
        print("    a   b   c   d   e   f   g   h")
        print(f"👉 FEN: {fen}")
        print("=" * 55 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = CameraBoardRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()
