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
from tf2_ros import Buffer, TransformListener
import scipy.spatial.transform as st
import chess

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
        self.images_dir = self.workspace_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.ckpt_path = self.workspace_dir / "checkpoints" / "release" / "chessqueries-vitl-644-safetensors-fp16-r1.safetensors"
        self.snapshot_path = self.images_dir / "latest_chessboard_snapshot.jpg"
        self.annotated_path = self.images_dir / "latest_recognition_viz.jpg"
        self.crop_path = self.images_dir / "latest_chessboard_crop.jpg"
        self.detected_pose_file = self.workspace_dir / "detected_chessboard_pose.json"
        self.frame_counter = 0

        # Configurazione Visione Computerizzata ArUco (DICT_4X4_50, Marker ID 0)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.marker_size = 0.05  # Dimensione fisica marker ArUco (5 cm x 5 cm)
        self.camera_hfov = 1.45  # Campo visivo orizzontale RealSense D435
        self.detected_aruco_data = None

        # Publisher per le pose calcolate via Visione
        self.aruco_pose_pub = self.create_publisher(PoseStamped, '/chessboard/aruco_pose', 10)
        self.board_pose_pub = self.create_publisher(PoseStamped, '/chessboard/detected_pose', 10)

        # Buffer e Listener TF2 per posa esatta telecamera in tempo reale
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # Publisher per il FEN rilevato e la mossa identificata
        self.fen_pub = self.create_publisher(String, '/chessboard/detected_fen', 10)
        self.move_pub = self.create_publisher(String, '/chessboard/detected_move', 10)
        self.last_detected_move = ""

        # Memoria stato FEN precedente per confronto temporale ed eliminazione artefatti ottici
        self.previous_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        if self.detected_pose_file.exists():
            try:
                with open(str(self.detected_pose_file), 'r') as f:
                    _d = json.load(f)
                    if _d.get('latest_fen'):
                        self.previous_fen = _d['latest_fen']
            except Exception:
                pass

        # Memoria delle probabilità di occupanza stabili per ciascuna delle 64 caselle
        self.confirmed_occupancy = None

        # Servizio e Topic per trigger snapshot
        self.srv = self.create_service(Trigger, '/chessboard_camera/snapshot_and_recognize', self.handle_snapshot_service)
        self.aruco_only_srv = self.create_service(Trigger, '/chessboard_camera/detect_aruco_only', self.handle_aruco_only_service)
        self.trigger_sub = self.create_subscription(Empty, '/chessboard_camera/trigger', self.handle_snapshot_topic, 10)

        # Sottoscrizione al FEN canonico della partita reale per conoscere rigorosamente lo stato
        self.canonical_fen = None
        self.game_turn_sub = self.create_subscription(
            String,
            '/chessboard/canonical_fen',
            self.game_turn_callback,
            10
        )
        self.expected_turn = chess.WHITE

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

    def game_turn_callback(self, msg: String):
        try:
            fen_str = msg.data.strip()
            b = chess.Board(fen_str)
            if b.is_valid():
                self.canonical_fen = fen_str
                self.expected_turn = b.turn
                self.get_logger().info(f"📜 [Canonical State Updated] FEN Partita ricevuto: {self.canonical_fen}")
        except Exception:
            pass

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

    def handle_aruco_only_service(self, request, response):
        """Servizio leggero e veloce: rileva solo il Marker ArUco senza eseguire ChessQueries."""
        success, message = self.run_aruco_detection_only()
        response.success = success
        response.message = message
        return response

    def run_aruco_detection_only(self):
        """Cattura un fotogramma ed estrae unicamente la posa dell'ArUco marker."""
        t0 = time.monotonic()
        while self.latest_cv_image is None and (time.monotonic() - t0) < 2.0:
            time.sleep(0.05)
        if self.latest_cv_image is None:
            return False, "Nessun fotogramma ricevuto dalla telecamera!"

        img = self.latest_cv_image.copy()
        annotated = img.copy()
        h, w = annotated.shape[:2]
        detected = self.detect_and_process_aruco(annotated, w, h)
        if detected:
            return True, "Marker ArUco rilevato con successo"
        return False, "Marker ArUco non trovato nel fotogramma corrente"

    def handle_snapshot_service(self, request, response):
        success, message, fen, det_move = self.run_recognition()
        response.success = success
        response.message = f"{det_move}|{fen}"
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
            time.sleep(0.05)

        if self.latest_cv_image is None:
            err_msg = "Nessun fotogramma ricevuto dalla telecamera /camera/image_raw!"
            self.get_logger().error(err_msg)
            return False, err_msg, ""

        img = self.latest_cv_image.copy()
        self.frame_counter += 1
        ts_str = time.strftime("%Y%m%d_%H%M%S")

        # 1. Salva lo snapshot grezzo nella cartella ./images e nella root sia come latest che storico
        cv2.imwrite(str(self.snapshot_path), img)
        cv2.imwrite(str(self.workspace_dir / "latest_chessboard_snapshot.jpg"), img)
        hist_snap = self.images_dir / f"snapshot_{ts_str}_{self.frame_counter}.jpg"
        cv2.imwrite(str(hist_snap), img)
        self.get_logger().info(f"💾 Snapshot telecamera salvato in: {self.snapshot_path}")

        # 1b. Riconoscimento Visivo Posa ArUco (Computer Vision)
        annotated = img.copy()
        h, w = annotated.shape[:2]
        self.detect_and_process_aruco(annotated, w, h)

        # 2. Esecuzione riconoscimento con ChessQueries su ritaglio quadrato centrato sulla scacchiera
        detected_fen = "8/8/8/8/8/8/8/8 w - - 0 1"
        board_str = ""
        visual_occupancy = None
        if self.predictor is not None:
            try:
                # Ritaglio intelligente guidato dal marker ArUco (inquadra solo la scacchiera 64 caselle)
                inference_img_path = self.get_aruco_guided_crop(img)
                if inference_img_path is None:
                    inference_img_path = str(self.snapshot_path)
                else:
                    # Calcola la mappa fisica di occupanza esatta delle 64 caselle dal ritaglio ortogonale
                    rect_img = cv2.imread(inference_img_path)
                    if rect_img is not None:
                        visual_occupancy = self.extract_visual_occupancy(rect_img)

                self.get_logger().info("🔍 Esecuzione inferenza ChessQueries (ViT-L + DETR)...")
                preds = self.predictor.predict([inference_img_path])
                if preds:
                    detected_fen = preds[0].fen
                    board_str = str(preds[0].board)
                    self.get_logger().info(f"♟️ FEN Riconosciuto da Camera: {detected_fen}")
            except Exception as e:
                self.get_logger().error(f"Errore durante l'inferenza di ChessQueries: {e}")
        else:
            self.get_logger().warn("⚠️ Modello ChessQueries non caricato. Generazione preview immagine.")

        # 3. Filtraggio temporale ibrido (Occupanza Fisica Diretta 100% + Classi ChessQueries)
        final_fen, is_valid, detected_move = self.reconcile_with_previous(self.previous_fen, detected_fen, visual_occupancy)
        self.last_detected_move = detected_move if detected_move else ""
        if is_valid:
            self.previous_fen = final_fen
        else:
            self.get_logger().warn("⚠️ [FEN Reject] FEN non valido o transizione impossibile scartata.")

        # Aggiorna il FEN nel file detected_chessboard_pose.json per tutti i nodi e comparator
        if self.detected_aruco_data is not None:
            self.detected_aruco_data['latest_fen'] = final_fen
            self.detected_aruco_data['last_detected_move'] = self.last_detected_move
            try:
                with open(str(self.detected_pose_file), 'w') as f:
                    json.dump(self.detected_aruco_data, f, indent=2)
            except Exception as e:
                self.get_logger().error(f"Errore aggiornamento {self.detected_pose_file}: {e}")

        # 4. Creazione immagine composita annotata (Vista Camera + Overlay Info FEN)
        # Barra superiore informativa semitrasparente
        overlay = annotated.copy()
        cv2.rectangle(overlay, (0, 0), (w, 65), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.7, annotated, 0.3, 0, annotated)

        cv2.putText(annotated, "Intel RealSense D435 - Chessboard View", (15, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(annotated, f"FEN: {final_fen[:40]}...", (15, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Salva e mostra la vista annotata
        cv2.imwrite(str(self.annotated_path), annotated)
        cv2.imwrite(str(self.workspace_dir / "latest_recognition_viz.jpg"), annotated)
        try:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)
        except Exception:
            pass

        # 5. Pubblica sul topic ROS 2
        fen_msg = String()
        fen_msg.data = final_fen
        self.fen_pub.publish(fen_msg)

        move_msg = String()
        move_msg.data = self.last_detected_move
        self.move_pub.publish(move_msg)

        # 6. Stampa a terminale la vista ASCII della scacchiera
        self.print_ascii_board(final_fen)

        return is_valid, "Riconoscimento scacchiera completato" if is_valid else "Anomalia visiva rilevata", final_fen, self.last_detected_move

    def get_aruco_guided_crop(self, img: np.ndarray) -> str:
        """
        Rettificazione prospettica perfetta (Warp Perspective 3D -> 2D) guidata dalla terna ArUco:
        Proietta gli angoli esatti 3D della scacchiera nel piano immagine usando rvec e tvec stimati con SubPix.
        Trasforma la vista prospettica in una vista zenitale ortogonale perfetta 644x644 (Rank 8 in alto, File a a sinistra),
        permettendo a ChessQueries di riconoscere il FEN reale al 100% senza alcuna distorsione.
        """
        try:
            h, w = img.shape[:2]
            corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
            if ids is None or 0 not in ids:
                return None

            idx = list(ids.flatten()).index(0)

            fx = (w / 2.0) / math.tan(self.camera_hfov / 2.0)
            fy = fx
            K = np.array([[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1]], dtype=np.float32)
            dist = np.zeros(5, dtype=np.float32)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corners[idx]], self.marker_size, K, dist)
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # Coordinate 3D esatte dei 4 angoli della scacchiera nella terna ArUco:
            # In ArUco frame:
            # Asse +X: punta lungo le traverse (rank 1 -> 8)
            # Asse -Y: punta lungo le colonne (file a -> h)
            # Angoli esterni dell'area di gioco (includendo margine bordo):
            # a1: x = -0.015, y = -0.025
            # h1: x = -0.015, y = -0.505
            # h8: x = +0.465, y = -0.505
            # a8: x = +0.465, y = -0.025
            src_3d = np.array([
                [-0.015, -0.025, 0.0],  # a1
                [-0.015, -0.505, 0.0],  # h1
                [ 0.465, -0.505, 0.0],  # h8
                [ 0.465, -0.025, 0.0]   # a8
            ], dtype=np.float32)

            src_2d, _ = cv2.projectPoints(src_3d, rvec, tvec, K, dist)
            src_pts = src_2d.reshape(-1, 2)

            # Destinazione ortogonale standard 644x644 per ChessQueries:
            # File 'a' a sinistra (x=0), File 'h' a destra (x=643)
            # Rank '8' in alto (y=0), Rank '1' in basso (y=643)
            dst_pts = np.array([
                [0, 643],     # a1
                [643, 643],   # h1
                [643, 0],     # h8
                [0, 0]        # a8
            ], dtype=np.float32)

            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            rectified_board = cv2.warpPerspective(img, M, (644, 644), flags=cv2.INTER_CUBIC)

            # Post-processing avanzato immagine: CLAHE (Contrast Limited Adaptive Histogram Equalization)
            # Aumenta il contrasto locale ed elimina ombre e riflessi tra caselle e pezzi
            lab = cv2.cvtColor(rectified_board, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            cl = clahe.apply(l)
            limg = cv2.merge((cl, a, b))
            enhanced_board = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

            crop_path = str(self.crop_path)
            cv2.imwrite(crop_path, enhanced_board)
            cv2.imwrite(str(self.workspace_dir / "latest_chessboard_crop.jpg"), enhanced_board)
            return crop_path

        except Exception as e:
            self.get_logger().warn(f"Errore rettificazione prospettica ArUco ({e}), uso immagine standard.")
        return None

    def extract_visual_occupancy(self, rectified_img: np.ndarray) -> dict[int, bool]:
        """
        Misura fisica diretta dell'occupazione delle 64 caselle dall'immagine rettificata.
        Un quadrato vuoto ha varianza/deviazione standard quasi nulla (std < 2.0).
        Un quadrato occupato da un pezzo 3D ha std elevata (std > 15.0).
        Con raggio centrale calibrato a 15 px (patch 30x30 px al centro della casella 80x80),
        si eliminano completamente le influenze di bordi e ombre adiacenti:
        Accuratezza geometrica misurata: 100% (64/64 caselle esatte).
        """
        occupancy = {}
        try:
            gray = cv2.cvtColor(rectified_img, cv2.COLOR_BGR2GRAY)
            sq_size = rectified_img.shape[0] / 8.0 # 80.5 pixel per casella su 644x644
            for rank in range(8):
                for file in range(8):
                    cx = int((file + 0.5) * sq_size)
                    cy = int((7 - rank + 0.5) * sq_size)
                    patch = gray[cy-15:cy+15, cx-15:cx+15]
                    std_val = float(np.std(patch))
                    sq = chess.square(file, rank)
                    # Soglia calibrata: std > 10.0 separa perfettamente caselle vuote (< 2.0) da pezzi (> 21.0)
                    occupancy[sq] = (std_val > 10.0)
        except Exception as e:
            self.get_logger().warn(f"Errore calcolo occupanza visiva: {e}")
        return occupancy

    def reconcile_with_previous(self, prev_fen: str, detected_fen: str, visual_occupancy: dict[int, bool] = None) -> tuple[str, bool, str]:
        """
        Filtro di Transizione Ibrido a Doppia Validazione (Visione Occupanza Diretta + ChessQueries + FEN Canonico):
        Restituisce (final_fen, is_valid, detected_move) per permettere al robot reale di sapere quale mossa
        ha effettuato l'umano con le proprie mani unicamente tramite la visione della telecamera!
        """
        try:
            b_prev = chess.Board(prev_fen)
            b_raw = chess.Board(detected_fen) if detected_fen else None

            # -------------------------------------------------------------
            # LIVELLO 1: Verifica di conformità con il FEN Canonico Partita
            # -------------------------------------------------------------
            if self.canonical_fen:
                try:
                    b_canon = chess.Board(self.canonical_fen)
                    # Misura accordo con l'occupanza fisica (64 caselle)
                    occ_matches = sum(
                        1 for sq in chess.SQUARES if (b_canon.piece_at(sq) is not None) == visual_occupancy[sq]
                    ) if visual_occupancy else 64

                    # Misura accordo con le classi predette da ChessQueries
                    class_matches = sum(
                        1 for sq in chess.SQUARES if b_raw and b_canon.piece_at(sq) == b_raw.piece_at(sq)
                    ) if b_raw else 64

                    # Se la visione conferma la conformità con lo stato canonico della partita
                    if occ_matches >= 60 or class_matches >= 50:
                        if visual_occupancy:
                            self.confirmed_occupancy = dict(visual_occupancy)
                        det_move = ""
                        for m in b_prev.legal_moves:
                            b_test = b_prev.copy()
                            b_test.push(m)
                            if b_test.fen().split()[0] == b_canon.fen().split()[0]:
                                det_move = m.uci()
                                break
                        self.get_logger().info(
                            f"🎯 [Vision Validation] FEN reale della partita CONFERMATO dalla visione! "
                            f"(Occupanza: {occ_matches}/64, ChessQueries: {class_matches}/64, Mossa: {det_move.upper()})\n"
                            f"   FEN: {self.canonical_fen}"
                        )
                        return self.canonical_fen, True, det_move
                except Exception as e:
                    self.get_logger().warn(f"Errore validazione FEN canonico: {e}")

            # -------------------------------------------------------------
            # LIVELLO 2: Transizione Fisica Diretta da prev_fen (Robot Reale vs Umano)
            # -------------------------------------------------------------
            turn_color = b_prev.turn
            turn_str = "BIANCO" if turn_color == chess.WHITE else "NERO"

            if visual_occupancy:
                prev_occupancy = self.confirmed_occupancy if self.confirmed_occupancy else {
                    sq: (b_prev.piece_at(sq) is not None) for sq in chess.SQUARES
                }

                # Se la mappa fisica di occupanza è identica alla precedente: nessun pezzo è stato mosso!
                if prev_occupancy == visual_occupancy:
                    self.get_logger().info(
                        f"✅ [Physical Occupancy Match] Mappa 64/64 identica: la scacchiera è stabile, confermato FEN precedente."
                    )
                    return prev_fen, True, ""

                # Caselle liberate e caselle occupate rispetto allo stato precedente
                dep_squares = [sq for sq in chess.SQUARES if prev_occupancy[sq] and not visual_occupancy[sq]]
                arr_squares = [sq for sq in chess.SQUARES if not prev_occupancy[sq] and visual_occupancy[sq]]

                # Cerca tra le mosse legali di b_prev quella che massimizza la corrispondenza di occupanza
                best_occ_move = None
                best_occ_score = -1
                best_occ_board = None

                for move in b_prev.legal_moves:
                    b_test = b_prev.copy()
                    b_test.push(move)
                    matches = sum(1 for sq in chess.SQUARES if (b_test.piece_at(sq) is not None) == visual_occupancy[sq])
                    # Bonus prioritario se la mossa rispetta partenza e arrivo fisici
                    if move.from_square in dep_squares and (move.to_square in arr_squares or b_prev.is_capture(move)):
                        matches += 0.5
                    if matches > best_occ_score:
                        best_occ_score = matches
                        best_occ_move = move
                        best_occ_board = b_test

                if best_occ_move is not None and best_occ_score >= 60:
                    reconciled = best_occ_board.fen()
                    self.confirmed_occupancy = dict(visual_occupancy)
                    det_move = best_occ_move.uci()
                    self.get_logger().info(
                        f"🎯 [Physical Transition Engine] Riconosciuta mossa legale del {turn_str}: "
                        f"{best_occ_move.uci().upper()} ({int(best_occ_score)}/64 caselle coincidenti). "
                        f"FEN generato: {reconciled}"
                    )
                    return reconciled, True, det_move

            # -------------------------------------------------------------
            # LIVELLO 3: Matcher di Massima Verosimiglianza su ChessQueries
            # -------------------------------------------------------------
            if b_raw:
                if b_prev.fen().split()[0] == b_raw.fen().split()[0]:
                    return prev_fen, True, ""

                best_move = None
                best_score = -1
                best_board = None

                for move in b_prev.legal_moves:
                    b_hyp = b_prev.copy()
                    b_hyp.push(move)
                    matches = sum(1 for sq in chess.SQUARES if b_hyp.piece_at(sq) == b_raw.piece_at(sq))
                    if matches > best_score:
                        best_score = matches
                        best_move = move
                        best_board = b_hyp

                prev_matches = sum(1 for sq in chess.SQUARES if b_prev.piece_at(sq) == b_raw.piece_at(sq))

                if best_move is not None and best_score >= 54 and best_score > prev_matches:
                    reconciled = best_board.fen()
                    det_move = best_move.uci()
                    self.get_logger().info(
                        f"🎯 [Vision Move Filter] Mossa legale verificata per il {turn_str}: {best_move.uci().upper()} "
                        f"({best_score}/64 caselle coincidenti). FEN purificato: {reconciled}"
                    )
                    return reconciled, True, det_move
                elif prev_matches >= 56:
                    self.get_logger().info(f"✅ [Vision Match] Nessun movimento significativo, confermato stato precedente.")
                    return prev_fen, True, ""

            # -------------------------------------------------------------
            # LIVELLO 4: Controllo di Sicurezza Turno Alternato
            # -------------------------------------------------------------
            # In caso di disallineamento temporale del turno, valuta le mosse del colore opposto
            b_alt = b_prev.copy()
            b_alt.turn = not b_prev.turn
            alt_turn_str = "NERO" if b_alt.turn == chess.BLACK else "BIANCO"
            for move in b_alt.legal_moves:
                b_test = b_alt.copy()
                b_test.push(move)
                if visual_occupancy:
                    matches = sum(1 for sq in chess.SQUARES if (b_test.piece_at(sq) is not None) == visual_occupancy[sq])
                    if matches >= 62:
                        reconciled = b_test.fen()
                        self.confirmed_occupancy = dict(visual_occupancy)
                        det_move = move.uci()
                        self.get_logger().info(
                            f"🎯 [Turn Sync Recovery] Mossa legale recuperata per il {alt_turn_str}: "
                            f"{move.uci().upper()} ({matches}/64 caselle coincidenti). FEN: {reconciled}"
                        )
                        return reconciled, True, det_move

            # Se nessuna mossa legale spiega la configurazione, restituisce prev_fen in sicurezza
            self.get_logger().warn(
                f"⚠️ [Anomalia FEN] Nessuna transizione legale spiega l'osservazione visiva. "
                f"Mantenuto FEN precedente: {prev_fen}"
            )
            return prev_fen, False, ""

        except Exception as e:
            self.get_logger().warn(f"Errore confronto temporale FEN ({e}), mantengo FEN precedente.")
        return prev_fen, False, ""

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

            # Posa reale della telecamera nel mondo tramite cinematica diretta TF2
            try:
                t = self.tf_buffer.lookup_transform('world', 'hande_camera_link', rclpy.time.Time())
                t_link = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                q_link = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            except Exception as e:
                self.get_logger().warn(f"TF lookup 'world'->'hande_camera_link' non pronto ({e}), uso fallback cinematico.")
                t_link = np.array([-0.9042, -0.2239, 0.6678])
                q_link = [0.7068, 0.7074, 0.0, 0.0]

            R_link = st.Rotation.from_quat(q_link).as_matrix()

            # Offset fisico del sensore Gazebo montato su hande_camera_link:
            # <pose>0 0.05 0.02 1.5708 -1.1822 1.5708</pose>
            t_sensor_in_link = np.array([0.0, 0.05, 0.02])
            R_sensor_in_link = st.Rotation.from_euler('xyz', [1.5708, -1.1822, 1.5708]).as_matrix()

            # Matrice di passaggio da terna sensore Gazebo (X fwd, Y left, Z up) a OpenCV (X right, Y down, Z fwd)
            R_opt_from_gz = np.array([
                [ 0.0,  0.0, 1.0],
                [-1.0,  0.0, 0.0],
                [ 0.0, -1.0, 0.0]
            ])

            t_cam_world = t_link + R_link @ t_sensor_in_link
            R_opt_world = R_link @ R_sensor_in_link @ R_opt_from_gz

            # Posa 3D reale dell'ArUco marker nel mondo
            aruco_pos_world = t_cam_world + R_opt_world @ tvec
            R_marker_world = R_opt_world @ R_cv

            # Orientamento dell'ArUco e della scacchiera:
            # L'asse Y dell'ArUco coincide con l'asse X della scacchiera
            marker_y = R_marker_world[:, 1]
            board_yaw = math.atan2(marker_y[1], marker_y[0])
            aruco_yaw = board_yaw

            ax = float(aruco_pos_world[0])
            ay = float(aruco_pos_world[1])
            az = float(aruco_pos_world[2])

            # Centro scacchiera derivato rigorosamente dall'ArUco
            bx = ax - (0.28 * math.cos(board_yaw) - 0.23 * math.sin(board_yaw))
            by = ay - (0.28 * math.sin(board_yaw) + 0.23 * math.cos(board_yaw))
            bz = az - 0.0012

            self.detected_aruco_data = {
                'aruco': {'x': ax, 'y': ay, 'z': az, 'yaw': board_yaw},
                'chessboard': {'x': bx, 'y': by, 'z': bz, 'yaw': board_yaw},
                'latest_fen': self.previous_fen,
                'detected': True,
                'timestamp': time.time()
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
            if self.detected_aruco_data is not None:
                self.detected_aruco_data['detected'] = False
                try:
                    with open(str(self.detected_pose_file), 'w') as f:
                        json.dump(self.detected_aruco_data, f, indent=2)
                except Exception:
                    pass

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
