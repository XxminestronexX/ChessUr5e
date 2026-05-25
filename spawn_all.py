#!/usr/bin/env python3
import subprocess

scripts = ['spawn_pawns.py', 'spawn_rooks.py', 'spawn_knights.py', 'spawn_bishops.py', 'spawn_royalty.py']

print("♟️ Schieramento completo in corso...")
for script in scripts:
    subprocess.run(['python3', script])
    print(f"✅ {script} eseguito.")