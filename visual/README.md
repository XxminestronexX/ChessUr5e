# Modulo di Visualizzazione e Diagnosi FEN (Cartella 'visual/')

Questa cartella è **100% modulare e autosufficiente**. È stata creata per verificare rapidamente se il FEN ricavato dalla telecamera e da ChessQueries corrisponde alla realtà fisica di Gazebo.

Puoi **eliminare l'intera cartella `visual/` in qualsiasi momento** senza alterare o rompere alcun file del progetto ChessUr5e:
```bash
rm -rf visual/
```

---

## 🚀 Script Disponibili

### 1. Visualizzatore Grafico Live (Split-Screen Dashboard)
Apre una finestra GUI che mostra **affiancati**:
- A sinistra: Il fotogramma reale HD acquisito dalla telecamera Intel RealSense D435;
- A destra: La scacchiera 2D renderizzata con i pezzi derivati dal FEN riconosciuto.

```bash
./visual/show_visual.sh
```
Oppure per verificare un singolo FEN arbitrario:
```bash
./visual/show_visual.sh "rnbqkbnr/pp1ppppp/8/2p5/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
```

---

### 2. Comparatore Terminale Casella per Casella (Gazebo vs Camera)
Interroga Gazebo sulla posizione fisica reale di ciascun modello 3D dei pezzi e la confronta casella per casella con il FEN della telecamera, stampando la tabella dei mismatch:

```bash
./visual/check_state.sh
```

Esempio di output:
```text
====================================================================
🔍   COMPARATORE STATO REALE GAZEBO vs FEN RICONOSCIUTO CAMERA   🔍
====================================================================
📷 FEN Riconosciuto da Telecamera:
   rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR w - - 0 1

🤖 Stato Reale estratto dai modelli Gazebo:
   rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR w - - 0 1

--------------------------------------------------------------------
📊 ESITO CONFRONTO: 64/64 caselle coincidenti (100.0%)
--------------------------------------------------------------------
🎉 PERFETTO! Nessun errore: il FEN corrisponde al 100% alla realtà in Gazebo.
====================================================================
```
