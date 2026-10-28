# ChessUr5e ♟️🤖

Sistema robotico end-to-end basato su **ROS 2**, **MoveIt 2**, **Gazebo Sim**, **Intel RealSense D435** e **Stockfish 16** che permette a un braccio robotico industriale **Universal Robots (UR5e)** di giocare a scacchi in tempo reale contro un avversario umano con percezione visiva tramite **ChessQueries (ViT-L/14 + DETR)**.

---

## 📌 Panoramica del Sistema

1. **Percezione Visiva (Intel RealSense D435 + ChessQueries)**:
   - Telecamera montata sul polso del braccio che fotografa la scacchiera dall'alto a ogni rientro in posizione *Home*;
   - Riconoscimento della disposizione di tutte le 64 caselle tramite **ChessQueries** (Vision Transformer ViT-L/14 con 64 learned square queries);
   - Estrazione ed emissione in tempo reale della stringa **FEN**.
2. **Motore Decisionale (Stockfish 16 Engine)**:
   - Nodo ROS 2 che riceve il FEN dalla telecamera;
   - Al segnale di fine turno dell'umano (*pulsante virtuale*), calcola la contromossa ottimale per il Nero.
3. **Controllo Cinematico e Manipolazione (MoveIt 2 + Robotiq Hand-E)**:
   - Movimenti di approccio, presa e rilascio verticali cartesiani per evitare collisioni con i pezzi adiacenti;
   - Gestione completa di mosse standard, catture (con deposito nel cimitero pezzi), **Arrocco** (spostamento combinato di Re e Torre) ed **En Passant**.

---

## 🛠️ Installazione e Download Dipendenze

È disponibile lo script **`./download_setup.sh`** che scarica automaticamente tutte le dipendenze e i pesi del modello AI (~709 MB da HuggingFace).

Puoi scegliere tra due modalità:

### Opzione A: Con Docker (Ambiente pronto all'uso)
```bash
./download_setup.sh --docker
```
*Oppure lancia `./download_setup.sh` e premi `1` dal menu.*

### Opzione B: SENZA Docker (Installazione Nativa su Ubuntu 24.04 / 22.04 con ROS 2)
Adatto a collaboratori che lavorano direttamente su Ubuntu con ROS 2 (Jazzy o Humble):
```bash
./download_setup.sh --native
```
*Oppure lancia `./download_setup.sh` e premi `2` dal menu.*  
Lo script installerà automaticamente tramite `sudo apt` e `pip` Stockfish 16, PyTorch, i pacchetti ROS 2 e scaricherà i pesi del modello.

---

## 🚀 Come Giocare la Partita

### 1. Avvio della Simulazione Gazebo + MoveIt 2

- **Se usi Docker:**
  ```bash
  ./start_sim.sh
  ```
- **Se usi l'ambiente Nativo (SENZA Docker):**
  ```bash
  ./start_sim_native.sh
  ```
Verranno aperti a schermo:
- La finestra 3D di **Gazebo Sim** con la scacchiera e i 32 pezzi rischierati;
- La finestra live della telecamera **Intel RealSense D435** con l'overlay del FEN;
- Il monitor del FEN a terminale (*ChessQueries Live Monitor*).

### 2. Invio delle Mosse e Partita contro Stockfish

- **Se usi Docker:**
  ```bash
  ./send_move.sh
  ```
- **Se usi l'ambiente Nativo (SENZA Docker):**
  ```bash
  ./send_move_native.sh
  ```

#### Flusso di Gioco Turn-Based:
1. Inserisci la tua mossa (es: `e2e4`);
2. Il braccio robotico esegue la mossa sulla scacchiera e risale in Home Panoramica;
3. La telecamera scatta una foto e ChessQueries aggiorna la posizione FEN;
4. Premi **[INVIO]** per confermare la fine del tuo turno umano (il "pulsante");
5. **Stockfish 16** calcola la mossa migliore per il Nero ed il robot la esegue fisicamente!

Puoi anche eseguire una sequenza specifica di mosse da riga di comando:
```bash
./send_move.sh "e2e4 e7e5 g1f3 b8c6"
```

### 3. Arresto della Simulazione
```bash
./stop_sim.sh
```

---

## 📐 Specifiche della Scacchiera
- **Dimensioni Scacchiera:** 51 × 51 cm
- **Lato Casella:** 58.75 mm (estratto al millimetro dal modello 3D `chessboard.obj`)
- **Pezzi FIDE:** Re (95 mm), Regina (85 mm), Alfiere (70 mm), Cavallo (60 mm), Torre (55 mm), Pedone (45 mm).
